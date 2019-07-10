// SPDX-License-Identifier: GPL-2.0-only
/*
 *  Copyright (C) 2019 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 *  Copyright (C) 2011-2012 John Crispin <blogic@openwrt.org>
 */

#include <linux/bits.h>
#include <linux/export.h>
#include <linux/ioport.h>
#include <linux/irqchip.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>

#include <lantiq_soc.h>

#define LTQ_EBU_BUSCON0				0x0060
#define LTQ_EBU_BUSCON_WRDIS			BIT(31)
#define LTQ_EBU_PCC_CON				0x0090
#define LTQ_EBU_PCC_CON_PCCARD_ON		BIT(0)
#define LTQ_EBU_PCC_CON_IREQ_RISING_EDGE        0x2
#define LTQ_EBU_PCC_CON_IREQ_FALLING_EDGE       0x4
#define LTQ_EBU_PCC_CON_IREQ_BOTH_EDGE          0x6
#define LTQ_EBU_PCC_CON_IREQ_DIS                0x8
#define LTQ_EBU_PCC_CON_IREQ_HIGH_LEVEL_DETECT  0xa
#define LTQ_EBU_PCC_CON_IREQ_LOW_LEVEL_DETECT	0xc
#define LTQ_EBU_PCC_CON_IREQ_MASK		0xe
#define LTQ_EBU_PCC_ISTAT			0x00a0
#define LTQ_EBU_PCC_ISTAT_PCI			BIT(4)
#define LTQ_EBU_PCC_IEN				0x00a4
#define LTQ_EBU_PCC_IEN_PCI_EN			BIT(4)

void __iomem *ltq_ebu_membase;

struct ltq_ebu_data {
	bool		initialize_buscon0_wrdis;
};

static const struct ltq_ebu_data ltq_ebu_falcon_data = {
	.initialize_buscon0_wrdis = false,
};

static const struct ltq_ebu_data ltq_ebu_xway_data = {
	.initialize_buscon0_wrdis = true,
};

static const struct of_device_id of_ebu_ids[] __initconst = {
	{
		/* DEPRECATED */
		.compatible = "lantiq,ebu-falcon",
		.data = &ltq_ebu_falcon_data,
	},
	{
		.compatible = "lantiq,falcon-ebu",
		.data = &ltq_ebu_falcon_data,
	},
	{
		/* DEPRECATED */
		.compatible = "lantiq,ebu-xway",
		.data = &ltq_ebu_xway_data,
	},
	{
		.compatible = "lantiq,xway-ebu",
		.data = &ltq_ebu_xway_data,
	},
	{ /* sentinel */ },
};

static void ltq_ebu_ack_irq(struct irq_data *d)
{
	ltq_ebu_w32(ltq_ebu_r32(LTQ_EBU_PCC_ISTAT) | LTQ_EBU_PCC_ISTAT_PCI,
		    LTQ_EBU_PCC_ISTAT);
}

static void ltq_ebu_mask_irq(struct irq_data *d)
{
	ltq_ebu_w32(ltq_ebu_r32(LTQ_EBU_PCC_IEN) & ~LTQ_EBU_PCC_IEN_PCI_EN,
		    LTQ_EBU_PCC_IEN);
}

static void ltq_ebu_unmask_irq(struct irq_data *d)
{
	ltq_ebu_w32(ltq_ebu_r32(LTQ_EBU_PCC_IEN) | LTQ_EBU_PCC_IEN_PCI_EN,
		    LTQ_EBU_PCC_IEN);
}

static int ltq_ebu_set_irq_type(struct irq_data *d, unsigned int flow_type)
{
	u32 val = ltq_ebu_r32(LTQ_EBU_PCC_CON);

	val &= ~LTQ_EBU_PCC_CON_IREQ_MASK;

	switch (flow_type & IRQ_TYPE_SENSE_MASK) {
	case IRQ_TYPE_NONE:
		val |= LTQ_EBU_PCC_CON_IREQ_DIS;
		break;

	case IRQ_TYPE_EDGE_RISING:
		val |= LTQ_EBU_PCC_CON_IREQ_RISING_EDGE;
		break;

	case IRQ_TYPE_EDGE_FALLING:
		val |= LTQ_EBU_PCC_CON_IREQ_FALLING_EDGE;
		break;

	case IRQ_TYPE_EDGE_BOTH:
		val |= LTQ_EBU_PCC_CON_IREQ_BOTH_EDGE;
		break;

	case IRQ_TYPE_LEVEL_HIGH:
		val |= LTQ_EBU_PCC_CON_IREQ_HIGH_LEVEL_DETECT;
		break;

	case IRQ_TYPE_LEVEL_LOW:
		val |= LTQ_EBU_PCC_CON_IREQ_LOW_LEVEL_DETECT;
		break;

	default:
		pr_err("Invalid trigger mode %x for IRQ %d\n", flow_type,
		       d->irq);
		return -EINVAL;
	}

	ltq_ebu_w32(val, LTQ_EBU_PCC_CON);

	return 0;
}

static void ltq_ebu_irq_handler(struct irq_desc *desc)
{
	struct irq_domain *domain = irq_desc_get_handler_data(desc);
	struct irq_chip *irqchip = irq_desc_get_chip(desc);

	chained_irq_enter(irqchip, desc);

	generic_handle_irq(irq_find_mapping(domain, 0));

	chained_irq_exit(irqchip, desc);
}

static struct irq_chip ltq_ebu_irq_chip = {
	.name = "EBU",
	.irq_ack = ltq_ebu_ack_irq,
	.irq_mask = ltq_ebu_mask_irq,
	.irq_unmask = ltq_ebu_unmask_irq,
	.irq_set_type = ltq_ebu_set_irq_type,
};

static int ltq_ebu_irq_map(struct irq_domain *domain, unsigned int irq,
			   irq_hw_number_t hwirq)
{
	irq_set_chip_and_handler(irq, &ltq_ebu_irq_chip, handle_edge_irq);
	irq_set_chip_data(irq, domain->host_data);

	return 0;
}

static const struct irq_domain_ops ltq_ebu_irqdomain_ops = {
	.map = ltq_ebu_irq_map,
	.xlate = irq_domain_xlate_twocell,
};

static int ltq_ebu_add_irqchip(struct device_node *np)
{
	struct irq_domain *parent_domain, *domain;
	int irq;

	parent_domain = irq_find_host(of_irq_find_parent(np));
	if (!parent_domain) {
		pr_err("%pOF: No interrupt-parent found\n", np);
		return -EINVAL;
	}

	domain = irq_domain_add_linear(np, 1, &ltq_ebu_irqdomain_ops, NULL);
	if (!domain) {
		pr_err("%pOF: Could not register EBU IRQ domain\n", np);
		return -ENOMEM;
	}

	irq = irq_of_parse_and_map(np, 0);
	if (!irq) {
		pr_err("%pOF: Failed to map interrupt\n", np);
		irq_domain_remove(domain);
		return -EINVAL;
	}

	irq_create_mapping(domain, 0);

	irq_set_chained_handler_and_data(irq, ltq_ebu_irq_handler, domain);

	return 0;
}

static int __init ltq_ebu_setup(void)
{
	const struct ltq_ebu_data *ebu_data;
	const struct of_device_id *match;
	struct resource res_ebu;
	struct device_node *np;
	u32 val;
	int ret;

	np = of_find_matching_node_and_match(NULL, of_ebu_ids, &match);
	if (!np)
		panic("Failed to load the EBU node from devicetree");

	if (of_address_to_resource(np, 0, &res_ebu))
		panic("Failed to get the EBU resources");

	if ((request_mem_region(res_ebu.start, resource_size(&res_ebu),
				res_ebu.name) < 0))
		pr_err("Failed to request the EBU resources");

	ltq_ebu_membase = ioremap_nocache(res_ebu.start,
						resource_size(&res_ebu));
	if (!ltq_ebu_membase)
		panic("Failed to remap the EBU resources");

	ebu_data = match->data;

	if (ebu_data && ebu_data->initialize_buscon0_wrdis) {
		val = ltq_ebu_r32(LTQ_EBU_BUSCON0) & ~LTQ_EBU_BUSCON_WRDIS;
		ltq_ebu_w32(val, LTQ_EBU_BUSCON0);
	}

	if (of_property_read_bool(np, "interrupt-controller")) {
		ret = ltq_ebu_add_irqchip(np);
		if (ret)
			return ret;
	}

	return 0;
}

arch_initcall(ltq_ebu_setup);
