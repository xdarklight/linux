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
#define LTQ_EBU_PCC_CON_IREQ_DIS                0x0
#define LTQ_EBU_PCC_CON_IREQ_RISING_EDGE        0x2
#define LTQ_EBU_PCC_CON_IREQ_FALLING_EDGE       0x4
#define LTQ_EBU_PCC_CON_IREQ_LEVEL_DETECT  	0x8
#define LTQ_EBU_PCC_CON_IREQ_MASK		0xe
#define LTQ_EBU_PCC_ISTAT			0x00a0
#define LTQ_EBU_PCC_ISTAT_PCI			BIT(4)
#define LTQ_EBU_PCC_IEN				0x00a4
#define LTQ_EBU_PCC_IEN_PCI_EN			BIT(4)

void __iomem *ltq_ebu_membase;
static u32 ltq_ebu_pci_inta;

struct ltq_ebu_data {
	bool initialize_buscon0_wrdis;
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
		.compatible = "lantiq,ase-ebu",
		.data = &ltq_ebu_xway_data,
	},
	{
		.compatible = "lantiq,danube-ebu",
		.data = &ltq_ebu_xway_data,
	},
	{
		.compatible = "lantiq,xrx100-ebu",
		.data = &ltq_ebu_xway_data,
	},
	{
		.compatible = "lantiq,xrx200-ebu",
		.data = &ltq_ebu_xway_data,
	},
	{ /* sentinel */ },
};

static void ltq_ebu_ack_irq(struct irq_data *d)
{
	ltq_ebu_w32(ltq_ebu_r32(LTQ_EBU_PCC_ISTAT) | LTQ_EBU_PCC_ISTAT_PCI,
		    LTQ_EBU_PCC_ISTAT);
	irq_chip_ack_parent(d);
}

static void ltq_ebu_mask_irq(struct irq_data *d)
{
	irq_chip_mask_parent(d);
	ltq_ebu_w32(ltq_ebu_r32(LTQ_EBU_PCC_IEN) & ~LTQ_EBU_PCC_IEN_PCI_EN,
		    LTQ_EBU_PCC_IEN);
}

static void ltq_ebu_unmask_irq(struct irq_data *d)
{
	ltq_ebu_w32(ltq_ebu_r32(LTQ_EBU_PCC_IEN) | LTQ_EBU_PCC_IEN_PCI_EN,
		    LTQ_EBU_PCC_IEN);
	irq_chip_unmask_parent(d);
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
		val |= LTQ_EBU_PCC_CON_IREQ_RISING_EDGE;
		val |= LTQ_EBU_PCC_CON_IREQ_FALLING_EDGE;
		break;

	case IRQ_TYPE_LEVEL_HIGH:
		val |= LTQ_EBU_PCC_CON_IREQ_LEVEL_DETECT;
		val |= LTQ_EBU_PCC_CON_IREQ_RISING_EDGE;
		break;

	case IRQ_TYPE_LEVEL_LOW:
		val |= LTQ_EBU_PCC_CON_IREQ_LEVEL_DETECT;
		val |= LTQ_EBU_PCC_CON_IREQ_FALLING_EDGE;
		break;

	default:
		pr_err("Invalid trigger mode %x for IRQ %d\n", flow_type,
		       d->irq);
		return -EINVAL;
	}

	ltq_ebu_w32(val, LTQ_EBU_PCC_CON);

	return 0;
}

static struct irq_chip ltq_ebu_irq_chip = {
	.name = "EBU",
	.irq_ack = ltq_ebu_ack_irq,
	.irq_mask = ltq_ebu_mask_irq,
	.irq_unmask = ltq_ebu_unmask_irq,
	.irq_set_type = ltq_ebu_set_irq_type,
	.irq_set_affinity = irq_chip_set_affinity_parent,
};

static int ltq_ebu_irq_domain_alloc(struct irq_domain *domain,
				    unsigned int virq, unsigned int nr_irqs,
				    void *arg)
{
	struct irq_fwspec parent_fwspec, *fwspec = arg;
	irq_hw_number_t hwirq;
	unsigned int type;
	int ret;

	if (nr_irqs != 1)
		return -EINVAL;

	ret = irq_domain_translate_twocell(domain, fwspec, &hwirq, &type);
	if (ret)
		return ret;

	parent_fwspec.fwnode = domain->parent->fwnode;
	parent_fwspec.param_count = 1;
	parent_fwspec.param[0] = ltq_ebu_pci_inta;

	irq_domain_set_hwirq_and_chip(domain, virq, hwirq, &ltq_ebu_irq_chip,
				      NULL);

	return irq_domain_alloc_irqs_parent(domain, virq, 1, &parent_fwspec);
}

static const struct irq_domain_ops ltq_ebu_irqdomain_ops = {
	.translate	= irq_domain_translate_twocell,
	.alloc		= ltq_ebu_irq_domain_alloc,
	.free		= irq_domain_free_irqs_common,
};

static int ltq_ebu_add_irqchip(struct device_node *np)
{
	struct irq_domain *parent_domain, *domain;
	int ret;

	ret = of_property_read_u32(np, "lantiq,pci-inta", &ltq_ebu_pci_inta);
	if (ret) {
		pr_err("%pOF: Could not read the 'lantiq,pci-inta' property\n",
		       np);
		return ret;
	}

	parent_domain = irq_find_host(of_irq_find_parent(np));
	if (!parent_domain) {
		pr_err("%pOF: No interrupt-parent found\n", np);
		return -EINVAL;
	}

	/* mask and ack any pending interrupt */
	ltq_ebu_w32(ltq_ebu_r32(LTQ_EBU_PCC_IEN) & ~LTQ_EBU_PCC_IEN_PCI_EN,
		    LTQ_EBU_PCC_IEN);
	ltq_ebu_w32(ltq_ebu_r32(LTQ_EBU_PCC_ISTAT) | LTQ_EBU_PCC_ISTAT_PCI,
		    LTQ_EBU_PCC_ISTAT);

	domain = irq_domain_add_hierarchy(parent_domain, 0, 1, np,
					  &ltq_ebu_irqdomain_ops, NULL);
	if (!domain) {
		pr_err("%pOF: Could not register EBU IRQ domain\n", np);
		return -ENOMEM;
	}

	return 0;
}

static int __init ltq_ebu_setup(void)
{
	const struct ltq_ebu_data *ebu_data;
	const struct of_device_id *match;
	struct device_node *np;
	u32 val;
	int ret;

	np = of_find_matching_node_and_match(NULL, of_ebu_ids, &match);
	if (!np)
		panic("Failed to load the EBU node from devicetree");

	ltq_ebu_membase = of_io_request_and_map(np, 0, of_node_full_name(np));
	if (IS_ERR(ltq_ebu_membase))
		panic("Failed to request and map the EBU resources");

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
