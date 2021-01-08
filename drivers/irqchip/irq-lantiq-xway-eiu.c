// SPDX-License-Identifier: GPL-2.0-only
/*
 * Lantiq XWAY External Interrupt Unit (EIU) GPIO interrupt controller.
 *
 * Copyright (C) 2010 John Crispin <john@phrozen.org>
 * Copyright (C) 2010 Thomas Langer <thomas.langer@lantiq.com>
 * Copyright (C) 2022 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 */

#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irqchip.h>
#include <linux/irqdomain.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/slab.h>

#define LTQ_EIU_EXIN_C						0x0
	#define LTQ_EIU_EXIN_C_DISABLED				0x0
	#define LTQ_EIU_EXIN_C_POSITIVE				0x1
	#define LTQ_EIU_EXIN_C_NEGATIVE				0x2
	#define LTQ_EIU_EXIN_C_LEVEL				0x4
	#define LTQ_EIU_EXIN_C_MASK				0xf
	#define LTQ_EIU_EXIN_C_SHIFT(_i)			((_i) * 4)

#define LTQ_EIU_EXIN_INIC					0x4
#define LTQ_EIU_EXIN_INC					0x8
#define LTQ_EIU_EXIN_INEN					0xc

struct lantiq_xway_eiu_priv {
	unsigned int num_irqs;
	u32 parent_irqs[6];
	void __iomem *base;
	raw_spinlock_t lock;
};

static void lantiq_xway_eiu_irq_set_masked(struct lantiq_xway_eiu_priv *priv,
					   unsigned long hwirq, bool masked)
{
	unsigned long flags;
	u32 val;

	raw_spin_lock_irqsave(&priv->lock, flags);

	val = ioread32be(priv->base + LTQ_EIU_EXIN_INEN);
	if (masked)
		val &= ~BIT(hwirq);
	else
		val |= BIT(hwirq);
	iowrite32be(val, priv->base + LTQ_EIU_EXIN_INEN);

	raw_spin_unlock_irqrestore(&priv->lock, flags);
}

static void lantiq_xway_eiu_irq_ack(struct irq_data *d)
{
	struct lantiq_xway_eiu_priv *priv = irq_data_get_irq_chip_data(d);

	iowrite32be(BIT(d->hwirq), priv->base + LTQ_EIU_EXIN_INC);
	irq_chip_ack_parent(d);
}

static void lantiq_xway_eiu_irq_mask(struct irq_data *d)
{
	struct lantiq_xway_eiu_priv *priv = irq_data_get_irq_chip_data(d);

	irq_chip_mask_parent(d);
	lantiq_xway_eiu_irq_set_masked(priv, d->hwirq, true);
}

static void lantiq_xway_eiu_irq_unmask(struct irq_data *d)
{
	struct lantiq_xway_eiu_priv *priv = irq_data_get_irq_chip_data(d);

	lantiq_xway_eiu_irq_set_masked(priv, d->hwirq, false);
	irq_chip_unmask_parent(d);
}

static int lantiq_xway_eiu_irq_retrigger(struct irq_data *d)
{
	struct lantiq_xway_eiu_priv *priv = irq_data_get_irq_chip_data(d);
	unsigned long flags;
	u32 val;

	raw_spin_lock_irqsave(&priv->lock, flags);
	val = ioread32be(priv->base + LTQ_EIU_EXIN_INIC);
	val |= BIT(d->hwirq);
	iowrite32be(val, priv->base + LTQ_EIU_EXIN_INIC);
	raw_spin_unlock_irqrestore(&priv->lock, flags);

	return irq_chip_retrigger_hierarchy(d);
}

static int lantiq_xway_eiu_irq_set_type(struct irq_data *d, unsigned int type)
{
	struct lantiq_xway_eiu_priv *priv = irq_data_get_irq_chip_data(d);
	irq_flow_handler_t handler;
	unsigned long flags;
	u32 val;
	u8 cfg;

	switch (type & IRQ_TYPE_SENSE_MASK) {
	case IRQ_TYPE_NONE:
		cfg = LTQ_EIU_EXIN_C_DISABLED;
		handler = handle_level_irq;
		break;

	case IRQ_TYPE_EDGE_RISING:
		cfg = LTQ_EIU_EXIN_C_POSITIVE;
		handler = handle_edge_irq;
		break;

	case IRQ_TYPE_EDGE_FALLING:
		cfg = LTQ_EIU_EXIN_C_NEGATIVE;
		handler = handle_edge_irq;
		break;

	case IRQ_TYPE_EDGE_BOTH:
		cfg = LTQ_EIU_EXIN_C_POSITIVE | LTQ_EIU_EXIN_C_NEGATIVE;
		handler = handle_edge_irq;
		break;

	case IRQ_TYPE_LEVEL_HIGH:
		cfg = LTQ_EIU_EXIN_C_POSITIVE | LTQ_EIU_EXIN_C_LEVEL;
		handler = handle_level_irq;
		break;

	case IRQ_TYPE_LEVEL_LOW:
		cfg = LTQ_EIU_EXIN_C_NEGATIVE | LTQ_EIU_EXIN_C_LEVEL;
		handler = handle_level_irq;
		break;

	default:
		return -EINVAL;
	}

	irq_set_handler_locked(d, handler);

	raw_spin_lock_irqsave(&priv->lock, flags);

	val = ioread32be(priv->base + LTQ_EIU_EXIN_C);
	val &= ~(LTQ_EIU_EXIN_C_MASK << LTQ_EIU_EXIN_C_SHIFT(d->hwirq));
	val |= cfg << LTQ_EIU_EXIN_C_SHIFT(d->hwirq);
	iowrite32be(val, priv->base + LTQ_EIU_EXIN_C);

	raw_spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}

static struct irq_chip lantiq_xway_eiu_irq_chip = {
	.name			= "EIU",
	.irq_ack		= lantiq_xway_eiu_irq_ack,
	.irq_mask		= lantiq_xway_eiu_irq_mask,
	.irq_unmask		= lantiq_xway_eiu_irq_unmask,
	.irq_set_affinity	= irq_chip_set_affinity_parent,
	.irq_retrigger		= lantiq_xway_eiu_irq_retrigger,
	.irq_set_type		= lantiq_xway_eiu_irq_set_type,
};

static int lantiq_xway_eiu_domain_alloc(struct irq_domain *domain,
					unsigned int virq,
					unsigned int nr_irqs, void *data)
{
	struct lantiq_xway_eiu_priv *priv = domain->host_data;
	struct irq_fwspec *fwspec = data;
	struct irq_fwspec parent_fwspec;
	irq_hw_number_t hwirq;
	unsigned int type;
	int ret;

	if (nr_irqs != 1)
		return -EINVAL;

	ret = irq_domain_translate_twocell(domain, fwspec, &hwirq, &type);
	if (ret)
		return ret;

	ret = irq_domain_set_hwirq_and_chip(domain, virq, hwirq,
					    &lantiq_xway_eiu_irq_chip, priv);
	if (ret)
		return ret;

	parent_fwspec.fwnode = domain->parent->fwnode;
	parent_fwspec.param_count = 1;
	parent_fwspec.param[0] = priv->parent_irqs[hwirq];

	return irq_domain_alloc_irqs_parent(domain, virq, 1, &parent_fwspec);
}

static const struct irq_domain_ops lantiq_xway_eiu_domain_ops = {
	.translate	= irq_domain_translate_twocell,
	.alloc		= lantiq_xway_eiu_domain_alloc,
	.free		= irq_domain_free_irqs_common,
};

static int lantiq_xway_eiu_of_init(struct device_node *node, struct device_node *parent)
{
	struct irq_domain *eiu_domain, *parent_domain;
	struct lantiq_xway_eiu_priv *priv;
	int ret;

	if (!parent) {
		pr_err("%pOF: missing parent interrupt node\n", node);
		return -ENODEV;
	}

	parent_domain = irq_find_host(parent);
	if (!parent_domain) {
		pr_err("%pOF: unable to obtain parent domain from %pOF\n",
		       node, parent);
		return -ENXIO;
	}

	ret = of_property_count_u32_elems(node, "lantiq,eiu-irqs");
	if (ret < 0) {
		pr_err("%pOF: 'lantiq,eiu-irqs' is absent\n", node);
		return ret;
	} else if (!ret) {
		pr_err("%pOF: 'lantiq,eiu-irqs' is empty\n", node);
		return -EINVAL;
	} else if (ret > ARRAY_SIZE(priv->parent_irqs)) {
		pr_err("%pOF: 'lantiq,eiu-irqs' is too large\n", node);
		return -EINVAL;
	}

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->num_irqs = ret;
	raw_spin_lock_init(&priv->lock);

	ret = of_property_read_u32_array(node, "lantiq,eiu-irqs",
					 priv->parent_irqs, priv->num_irqs);
	if (ret) {
		pr_err("%pOF: Failed to read the 'lantiq,eiu-irqs' property\n",
		       node);
		goto free_priv;
	}

	priv->base = of_io_request_and_map(node, 0, of_node_full_name(node));
	if (!priv->base) {
		pr_err("%pOF: Unable to map registers\n", node);
		goto free_priv;
	}

	/* Disable and clear all pending interrupts */
	iowrite32be(0, priv->base + LTQ_EIU_EXIN_INEN);
	iowrite32be(BIT(priv->num_irqs) - 1, priv->base + LTQ_EIU_EXIN_INC);

	eiu_domain = irq_domain_add_hierarchy(parent_domain, 0, priv->num_irqs,
					      node,
					      &lantiq_xway_eiu_domain_ops,
					      priv);
	if (!eiu_domain) {
		pr_err("%pOF: Failed to add hierarchical IRQ domain\n", node);
		ret = -ENODEV;
		goto unmap_io;
	}

	return 0;

unmap_io:
	iounmap(priv->base);
free_priv:
	kfree(priv);
	return ret;
}

IRQCHIP_PLATFORM_DRIVER_BEGIN(lantiq_xway_eiu)
IRQCHIP_MATCH("lantiq,xway-eiu", lantiq_xway_eiu_of_init)
IRQCHIP_MATCH("lantiq,eiu-xway", lantiq_xway_eiu_of_init) /* deprecated */
IRQCHIP_PLATFORM_DRIVER_END(lantiq_xway_eiu)

MODULE_AUTHOR("Martin Blumenstingl <martin.blumenstingl@googlemail.com>");
MODULE_DESCRIPTION("Lantiq External (GPIO) Interrupt Unit");
MODULE_LICENSE("GPL v2");
