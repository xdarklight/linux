// SPDX-License-Identifier: GPL-2.0-only
/*
 * Lantiq XWAY External Interrupt Unit (EIU) GPIO interrupt controller.
 *
 * Copyright (C) 2010 John Crispin <john@phrozen.org>
 * Copyright (C) 2010 Thomas Langer <thomas.langer@lantiq.com>
 * Copyright (C) 2019 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 */

#include <linux/bitops.h>
#include <linux/interrupt.h>
#include <linux/irqchip.h>
#include <linux/irqdomain.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>

/* configuration register */
#define LTQ_EIU_EXIN_C						0x0
	#define LTQ_EIU_EXIN_C_EDGE_RISING			0x1
	#define LTQ_EIU_EXIN_C_EDGE_FALLING			0x2
	#define LTQ_EIU_EXIN_C_EDGE_BOTH			0x3
	#define LTQ_EIU_EXIN_C_LEVEL_HIGH			0x5
	#define LTQ_EIU_EXIN_C_LEVEL_LOW			0x6
	#define LTQ_EIU_EXIN_C_MASK				0xf
	#define LTQ_EIU_EXIN_C_SHIFT(_i)			((_i) * 4)

/* status register - the status depends on the setting from LTQ_EIU_EXIN_C */
#define LTQ_EIU_EXIN_INIC					0x4

/* interrupt clear register (clear bit to clear the interrupt) */
#define LTQ_EIU_EXIN_INC					0x8

/* interrupt enable register */
#define LTQ_EIU_EXIN_INEN					0xc

#define LANTIQ_XWAY_EIU_NUM_CHIP_TYPES				2
#define LANTIQ_XWAY_EIU_MAX_IRQS				8

struct lantiq_xway_eiu_data {
	u32			parent_irqs[LANTIQ_XWAY_EIU_MAX_IRQS];
	unsigned int		num_parent_irqs;
};

struct lantiq_xway_eiu_host {
	void __iomem				*base;
	const struct lantiq_xway_eiu_data	*data;
};

static const struct lantiq_xway_eiu_data lantiq_xway_eiu_ase_data = {
	.parent_irqs[0] = 29,
	.parent_irqs[1] = 30,
	.parent_irqs[2] = 31,
	.num_parent_irqs = 3,
};

static const struct lantiq_xway_eiu_data lantiq_xway_eiu_arx100_data = {
	.parent_irqs[0] = 166,
	.parent_irqs[1] = 135,
	.parent_irqs[2] = 66,
	.parent_irqs[3] = 40,
	.parent_irqs[4] = 41,
	.parent_irqs[5] = 42,
	.num_parent_irqs = 6,
};

static const struct lantiq_xway_eiu_data lantiq_xway_eiu_danube_data = {
	.parent_irqs[0] = 166,
	.parent_irqs[1] = 135,
	.parent_irqs[2] = 66,
	.num_parent_irqs = 3,
};

static void lantiq_xway_eiu_clear_bits(struct irq_data *d, u32 reg, u32 mask)
{
	struct lantiq_xway_eiu_host *host = irq_data_get_irq_chip_data(d);
	u32 val;

	val = __raw_readl(host->base + reg);
	val &= ~mask;
	__raw_writel(val, host->base + reg);
}

static void lantiq_xway_eiu_set_bits(struct irq_data *d, u32 reg, u32 mask)
{
	struct lantiq_xway_eiu_host *host = irq_data_get_irq_chip_data(d);
	u32 val;

	val = __raw_readl(host->base + reg);
	val |= mask;
	__raw_writel(val, host->base + reg);
}

static void lantiq_xway_eiu_ack(struct irq_data *d)
{
	irq_chip_ack_parent(d);

	lantiq_xway_eiu_clear_bits(d, LTQ_EIU_EXIN_INC, d->mask);
}

static void lantiq_xway_eiu_mask(struct irq_data *d)
{
	irq_chip_unmask_parent(d);

	lantiq_xway_eiu_clear_bits(d, LTQ_EIU_EXIN_INEN, d->mask);
}

static void lantiq_xway_eiu_unmask(struct irq_data *d)
{
	irq_chip_mask_parent(d);

	lantiq_xway_eiu_clear_bits(d, LTQ_EIU_EXIN_INC, d->mask);
	lantiq_xway_eiu_set_bits(d, LTQ_EIU_EXIN_INEN, d->mask);
}

static void lantiq_xway_eiu_mask_ack(struct irq_data *d)
{
	lantiq_xway_eiu_mask(d);
	lantiq_xway_eiu_ack(d);
}

static int lantiq_xway_eiu_set_type(struct irq_data *d, unsigned int type)
{
	u32 mask;
	u8 val;

	switch (type & IRQ_TYPE_SENSE_MASK) {
	case IRQ_TYPE_EDGE_RISING:
		val = LTQ_EIU_EXIN_C_EDGE_RISING;
		break;

	case IRQ_TYPE_EDGE_FALLING:
		val = LTQ_EIU_EXIN_C_EDGE_FALLING;
		break;

	case IRQ_TYPE_EDGE_BOTH:
		val = LTQ_EIU_EXIN_C_EDGE_BOTH;
		break;

	case IRQ_TYPE_LEVEL_HIGH:
		val = LTQ_EIU_EXIN_C_LEVEL_HIGH;
		break;

	case IRQ_TYPE_LEVEL_LOW:
		val = LTQ_EIU_EXIN_C_LEVEL_LOW;
		break;

	default:
		pr_err("Invalid trigger mode %x for IRQ %d\n", type, d->irq);
		return -EINVAL;
	}

	mask = LTQ_EIU_EXIN_C_MASK << LTQ_EIU_EXIN_C_SHIFT(d->hwirq);
	lantiq_xway_eiu_clear_bits(d, LTQ_EIU_EXIN_C, mask);

	lantiq_xway_eiu_set_bits(d, LTQ_EIU_EXIN_C,
				 val << LTQ_EIU_EXIN_C_SHIFT(d->hwirq));

	return irq_setup_alt_chip(d, type);
}

static struct irq_chip lantiq_xway_eiu_chip = {
	.name			= "lantiq-xway-eiu",
	.irq_ack		= lantiq_xway_eiu_ack,
	.irq_mask		= lantiq_xway_eiu_mask,
	.irq_mask_ack		= lantiq_xway_eiu_mask_ack,
	.irq_unmask		= lantiq_xway_eiu_unmask,
	.irq_retrigger		= irq_chip_retrigger_hierarchy,
	.irq_set_type		= lantiq_xway_eiu_set_type,
};

static int lantiq_xway_eiu_domain_alloc(struct irq_domain *dm,
					unsigned int virq,
					unsigned int nr_irqs, void *data)
{
	struct lantiq_xway_eiu_host *host = dm->host_data;
	struct irq_fwspec *fwspec = data;
	struct irq_fwspec p_fwspec;
	irq_hw_number_t hwirq;

	if (WARN_ON(nr_irqs != 1))
		return -EINVAL;

	hwirq = fwspec->param[0];

	if (hwirq >= host->data->num_parent_irqs)
		return -EINVAL;

	irq_domain_set_hwirq_and_chip(dm, virq, hwirq, &lantiq_xway_eiu_chip,
				      host);

	p_fwspec.fwnode = dm->parent->fwnode;
	p_fwspec.param_count = 1;
	p_fwspec.param[0] = host->data->parent_irqs[hwirq];

	return irq_domain_alloc_irqs_parent(dm, virq, nr_irqs, &p_fwspec);
}

static const struct irq_domain_ops lantiq_xway_eiu_domain_ops = {
	.alloc	= lantiq_xway_eiu_domain_alloc,
	.free	= irq_domain_free_irqs_common,
	.xlate = irq_domain_xlate_twocell,
};

static void lantiq_xway_eiu_remove_irq_domain(void *data)
{
	struct irq_domain *domain = data;

	irq_domain_remove(domain);
}

static int lantiq_xway_eiu_probe(struct platform_device *pdev)
{
	struct irq_domain *parent_domain, *domain;
	struct lantiq_xway_eiu_host *host;
	struct device *dev = &pdev->dev;
	struct resource *res;
	int ret;

	host = devm_kzalloc(dev, sizeof(*host), GFP_KERNEL);
	if (!host)
		return -ENOMEM;

	host->data = device_get_match_data(dev);
	if (!host->data) {
		dev_err(dev, "No match data found\n");
		return -EINVAL;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	host->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(host->base))
		return PTR_ERR(host->base);

	parent_domain = irq_find_host(of_irq_find_parent(dev->of_node));
	if (!parent_domain) {
		dev_err(dev, "No interrupt-parent found\n");
		return -EINVAL;
	}

	domain = irq_domain_create_hierarchy(parent_domain, 0,
					     host->data->num_parent_irqs,
					     of_node_to_fwnode(dev->of_node),
					     &lantiq_xway_eiu_domain_ops,
					     host);
	if (!domain) {
		dev_err(dev, "Could not register exti domain\n");
		return -ENOMEM;
	}

	ret = devm_add_action_or_reset(dev, lantiq_xway_eiu_remove_irq_domain, domain);
	if (ret)
		return ret;

	return 0;
}

static const struct of_device_id lantiq_xway_eiu_of_ids[] = {
	{
		.compatible = "lantiq,arx100-eiu",
		.data = &lantiq_xway_eiu_arx100_data
	},
	{
		.compatible = "lantiq,ase-eiu",
		.data = &lantiq_xway_eiu_ase_data
	},
	{
		.compatible = "lantiq,danube-eiu",
		.data = &lantiq_xway_eiu_danube_data
	},
	{
		.compatible = "lantiq,vrx200-eiu",
		.data = &lantiq_xway_eiu_arx100_data
	},
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, lantiq_xway_eiu_of_ids);

static struct platform_driver lantiq_xway_eiu_driver = {
	.probe		= lantiq_xway_eiu_probe,
	.driver		= {
		.name	= "lantiq-xway-eiu",
		.of_match_table = of_match_ptr(lantiq_xway_eiu_of_ids),
	},
};
module_platform_driver(lantiq_xway_eiu_driver);

MODULE_AUTHOR("Martin Blumenstingl <martin.blumenstingl@googlemail.com>");
MODULE_DESCRIPTION("Lantiq XWAY External Interrupt Unit driver");
MODULE_LICENSE("GPL v2");
