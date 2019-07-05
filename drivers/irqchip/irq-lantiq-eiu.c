// SPDX-License-Identifier: GPL-2.0-only
/*
 * Lantiq XWAY External Interrupt Unit (EIU) GPIO interrupt controller.
 *
 * Copyright (C) 2010 John Crispin <john@phrozen.org>
 * Copyright (C) 2010 Thomas Langer <thomas.langer@lantiq.com>
 * Copyright (C) 2019 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 */

#include <linux/interrupt.h>
#include <linux/irqchip.h>
#include <linux/irqdomain.h>
#include <linux/irq.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/bitops.h>

#define LTQ_EIU_EXIN_C						0x0
	#define LTQ_EIU_EXIN_C_EDGE_RISING			0x1
	#define LTQ_EIU_EXIN_C_EDGE_FALLING			0x2
	#define LTQ_EIU_EXIN_C_EDGE_BOTH			0x3
	#define LTQ_EIU_EXIN_C_LEVEL_HIGH			0x5
	#define LTQ_EIU_EXIN_C_LEVEL_LOW			0x6
	#define LTQ_EIU_EXIN_C_MASK				0xf
	#define LTQ_EIU_EXIN_C_SHIFT(_i)			((_i) * 4)

#define LTQ_EIU_EXIN_INIC					0x4

#define LTQ_EIU_EXIN_INC					0x8

#define LTQ_EIU_EXIN_INEN					0xc

#define LANTIQ_XWAY_EIU_NUM_CHIP_TYPES				2
#define LANTIQ_XWAY_EIU_MAX_IRQS				8

static int lantiq_xway_eiu_set_type(struct irq_data *d, unsigned int type)
{
	struct irq_chip_generic *gc = irq_data_get_irq_chip_data(d);
	u32 reg;
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

	irq_gc_lock(gc);

	reg = irq_reg_readl(gc, LTQ_EIU_EXIN_C);
	reg &= ~(LTQ_EIU_EXIN_C_MASK << LTQ_EIU_EXIN_C_SHIFT(d->hwirq));
	reg |= val << LTQ_EIU_EXIN_C_SHIFT(d->hwirq);
	irq_reg_writel(gc, reg, LTQ_EIU_EXIN_C);

	irq_gc_unlock(gc);

	return irq_setup_alt_chip(d, type);
}

static void lantiq_xway_eiu_irq_cascade(struct irq_desc *desc)
{
	struct irq_domain *domain = irq_desc_get_handler_data(desc);
	unsigned int irq = irq_desc_get_irq(desc);

	generic_handle_irq(irq_find_mapping(domain, irq));
}

static u32 lantiq_xway_eiu_reg_read(void __iomem *base)
{
	return ioread32be(base);
}

static void lantiq_xway_eiu_reg_write(u32 val, void __iomem *base)
{
	iowrite32be(val, base);
}

static int __init lantiq_xway_eiu_of_init(struct device_node *np,
					  struct device_node *parent)
{
	struct irq_chip_generic *gc;
	struct irq_domain *domain;
	int ret, i, nr_irqs;
	void __iomem *base;

	nr_irqs = of_irq_count(np);
	if (nr_irqs < 1 || nr_irqs > LANTIQ_XWAY_EIU_MAX_IRQS) {
		pr_err("%pOF: Invalid number of interrupts\n", np);
		ret = -EINVAL;
		goto err;
	}

	base = of_iomap(np, 0);
	if (!base) {
		pr_err("%pOF: Unable to map registers\n", np);
		ret = -EINVAL;
		goto err;
	}

	domain = irq_domain_add_linear(np, nr_irqs, &irq_generic_chip_ops,
				       NULL);
	if (!domain) {
		pr_err("%pOFn: Could not register interrupt domain.\n", np);
		ret = -ENOMEM;
		goto err_unmap;
	}

	ret = irq_alloc_domain_generic_chips(domain, nr_irqs,
					     LANTIQ_XWAY_EIU_NUM_CHIP_TYPES,
					     "EIU", handle_edge_irq, 0, 0, 0);
	if (ret) {
		pr_err("%pOF: Could not allocate generic interrupt chip.\n",
		       np);
		goto err_free_domain;
	}

	gc = domain->gc->gc[0];
	gc->reg_base = base;
	gc->reg_readl = lantiq_xway_eiu_reg_read;
	gc->reg_writel = lantiq_xway_eiu_reg_write;

	for (i = 0; i < LANTIQ_XWAY_EIU_NUM_CHIP_TYPES; i++) {
		struct irq_chip_type *ct = &gc->chip_types[i];

		ct->chip.irq_ack = irq_gc_ack_clr_bit;
		ct->chip.irq_mask = irq_gc_mask_disable_reg;
		ct->chip.irq_unmask = irq_gc_unmask_enable_reg;
		ct->chip.irq_set_type = lantiq_xway_eiu_set_type;
		ct->chip.name = gc->domain->name;

		ct->regs.enable = LTQ_EIU_EXIN_INEN;
		ct->regs.disable = LTQ_EIU_EXIN_INEN;
		ct->regs.ack = LTQ_EIU_EXIN_INC;
	}

	gc->chip_types[0].type = IRQ_TYPE_LEVEL_MASK;
	gc->chip_types[0].handler = handle_level_irq;

	gc->chip_types[1].type = IRQ_TYPE_EDGE_BOTH;
	gc->chip_types[1].handler = handle_edge_irq;

	for (i = 0; i < nr_irqs; i++) {
		unsigned int irq = irq_of_parse_and_map(np, i);

		irq_set_chained_handler_and_data(irq,
						 lantiq_xway_eiu_irq_cascade,
						 domain);
	}

	return 0;

err_free_domain:
	irq_domain_remove(domain);
err_unmap:
	iounmap(base);
err:
	return ret;
}

IRQCHIP_DECLARE(lantiq_xway_eiu, "lantiq,xway-eiu", lantiq_xway_eiu_of_init);
