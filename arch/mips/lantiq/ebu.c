// SPDX-License-Identifier: GPL-2.0-only
/*
 *  Copyright (C) 2019-2022 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
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
#include <linux/regmap.h>

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
struct regmap *ltq_ebu_regmap;
static u32 ltq_ebu_pci_inta;

struct ltq_ebu_data {
	bool initialize_buscon0_wrdis;
};

static void ltq_ebu_ack_irq(struct irq_data *d)
{
	regmap_update_bits(ltq_ebu_regmap, LTQ_EBU_PCC_ISTAT,
			   LTQ_EBU_PCC_ISTAT_PCI, LTQ_EBU_PCC_ISTAT_PCI);
	irq_chip_ack_parent(d);
}

static void ltq_ebu_mask_irq(struct irq_data *d)
{
	irq_chip_mask_parent(d);
	regmap_update_bits(ltq_ebu_regmap, LTQ_EBU_PCC_IEN,
			   LTQ_EBU_PCC_IEN_PCI_EN, 0);
}

static void ltq_ebu_unmask_irq(struct irq_data *d)
{
	regmap_update_bits(ltq_ebu_regmap, LTQ_EBU_PCC_IEN,
			   LTQ_EBU_PCC_IEN_PCI_EN, LTQ_EBU_PCC_IEN_PCI_EN);
	irq_chip_unmask_parent(d);
}

static int ltq_ebu_set_irq_type(struct irq_data *d, unsigned int flow_type)
{
	u32 val;

	switch (flow_type & IRQ_TYPE_SENSE_MASK) {
	case IRQ_TYPE_NONE:
		val = LTQ_EBU_PCC_CON_IREQ_DIS;
		break;

	case IRQ_TYPE_EDGE_RISING:
		val = LTQ_EBU_PCC_CON_IREQ_RISING_EDGE;
		break;

	case IRQ_TYPE_EDGE_FALLING:
		val = LTQ_EBU_PCC_CON_IREQ_FALLING_EDGE;
		break;

	case IRQ_TYPE_EDGE_BOTH:
		val = LTQ_EBU_PCC_CON_IREQ_RISING_EDGE |
		      LTQ_EBU_PCC_CON_IREQ_FALLING_EDGE;
		break;

	case IRQ_TYPE_LEVEL_HIGH:
		val = LTQ_EBU_PCC_CON_IREQ_LEVEL_DETECT |
		      LTQ_EBU_PCC_CON_IREQ_RISING_EDGE;
		break;

	case IRQ_TYPE_LEVEL_LOW:
		val = LTQ_EBU_PCC_CON_IREQ_LEVEL_DETECT |
		      LTQ_EBU_PCC_CON_IREQ_FALLING_EDGE;
		break;

	default:
		pr_err("Invalid trigger mode %x for IRQ %d\n", flow_type,
		       d->irq);
		return -EINVAL;
	}

	regmap_update_bits(ltq_ebu_regmap, LTQ_EBU_PCC_CON,
			   LTQ_EBU_PCC_CON_IREQ_MASK, val);

	return 0;
}

static struct irq_chip ltq_ebu_irq_chip = {
	.irq_ack = ltq_ebu_ack_irq,
	.irq_mask = ltq_ebu_mask_irq,
	.irq_unmask = ltq_ebu_unmask_irq,
	.irq_eoi = irq_chip_eoi_parent,
	.irq_set_affinity = irq_chip_set_affinity_parent,
	.irq_retrigger = irq_chip_retrigger_hierarchy,
	.irq_set_type = ltq_ebu_set_irq_type,
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

static int ltq_ebu_add_irqchip(struct device *dev)
{
	struct irq_domain *parent_domain, *domain;
	int ret;

	ret = device_property_read_u32(dev, "lantiq,pci-inta",
				       &ltq_ebu_pci_inta);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Could not read the 'lantiq,pci-inta' property\n");

	parent_domain = irq_find_host(of_irq_find_parent(dev->of_node));
	if (!parent_domain)
		return dev_err_probe(dev, -EINVAL,
				     "No interrupt-parent found\n");

	/* mask and ack any pending interrupt */
	regmap_update_bits(ltq_ebu_regmap, LTQ_EBU_PCC_IEN,
			   LTQ_EBU_PCC_IEN_PCI_EN, 0);
	regmap_update_bits(ltq_ebu_regmap, LTQ_EBU_PCC_ISTAT,
			   LTQ_EBU_PCC_ISTAT_PCI, LTQ_EBU_PCC_ISTAT_PCI);

	domain = irq_domain_add_hierarchy(parent_domain, 0, 1, dev->of_node,
					  &ltq_ebu_irqdomain_ops, NULL);
	if (!domain)
		return dev_err_probe(dev, -ENOMEM,
				     "Could not register EBU IRQ domain\n");

	return 0;
}

static const struct regmap_range ebu_regmap_ranges[] = {
	{ .range_min = 0x00, .range_max = 0x00 }, /* CLC */
	{ .range_min = 0x08, .range_max = 0x08 }, /* ID */
	{ .range_min = 0x10, .range_max = 0x10 }, /* CON */
	{ .range_min = 0x20, .range_max = 0x2c }, /* ADDRSEL0..3 */
	{ .range_min = 0x60, .range_max = 0x6c }, /* BUSCON0..3 */
	{ .range_min = LTQ_EBU_PCC_CON, .range_max = LTQ_EBU_PCC_CON },
	{ .range_min = 0x94, .range_max = 0x94 }, /* PCC_STAT */
	{ .range_min = LTQ_EBU_PCC_ISTAT, .range_max = LTQ_EBU_PCC_ISTAT },
	{ .range_min = LTQ_EBU_PCC_IEN, .range_max = LTQ_EBU_PCC_IEN },
	{ .range_min = 0xa4, .range_max = 0xa4 }, /* ECC_IEN */
	{ .range_min = 0xa8, .range_max = 0xa8 }, /* PCC_INT_OUT */
	{ .range_min = 0xac, .range_max = 0xac }, /* PCC_IRS */
	{ .range_min = 0xb0, .range_max = 0xb0 }, /* NAND_CON */
	{ .range_min = 0xb4, .range_max = 0xb4 }, /* NAND_WAIT */
	{ .range_min = 0xb8, .range_max = 0xb8 }, /* NAND_ECC0 */
	{ .range_min = 0xbc, .range_max = 0xbc }, /* NAND_ECC_AC */
	{ .range_min = 0xc0, .range_max = 0xc0 }, /* NAND_ECC_CR */
};

static const struct regmap_access_table ebu_regmap_access_table = {
	.yes_ranges = ebu_regmap_ranges,
	.n_yes_ranges = ARRAY_SIZE(ebu_regmap_ranges),
};

static const struct regmap_config ebu_regmap_config = {
	.reg_bits = 8,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = 0xc0,
	.wr_table = &ebu_regmap_access_table,
	.rd_table = &ebu_regmap_access_table,
};

static int ltq_ebu_probe(struct platform_device *pdev)
{
	const struct ltq_ebu_data *ebu_data;
	struct regmap *regmap;
	int ret;

	ltq_ebu_membase = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ltq_ebu_membase))
		return dev_err_probe(&pdev->dev, PTR_ERR(ltq_ebu_membase),
				     "Failed to map the EBU address space\n");

	ltq_ebu_regmap = devm_regmap_init_mmio(&pdev->dev, ltq_ebu_membase,
					       &ebu_regmap_config);
	if (IS_ERR(ltq_ebu_regmap))
		return dev_err_probe(&pdev->dev, PTR_ERR(ltq_ebu_regmap),
				     "Failed to register EBU MMIO regmap\n");

	ebu_data = of_device_get_match_data(&pdev->dev);
	if (ebu_data && ebu_data->initialize_buscon0_wrdis) {
		regmap_update_bits(regmap, LTQ_EBU_BUSCON0,
				   LTQ_EBU_BUSCON_WRDIS, 0);
	}

	if (device_property_read_bool(&pdev->dev, "interrupt-controller")) {
		ret = ltq_ebu_add_irqchip(&pdev->dev);
		if (ret)
			return ret;
	}

	ret = devm_of_platform_populate(&pdev->dev);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "Failed to populate child devices\n");

	return 0;
}

static const struct ltq_ebu_data ltq_ebu_falcon_data = {
	.initialize_buscon0_wrdis = false,
};

static const struct ltq_ebu_data ltq_ebu_xway_data = {
	.initialize_buscon0_wrdis = true,
};

static const struct of_device_id ltq_ebu_of_ids[] = {
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

static struct platform_driver ltq_ebu_driver = {
	.probe = ltq_ebu_probe,
	.driver = {
		.name = "ltq-ebu",
		.of_match_table = ltq_ebu_of_ids,
	},
};
builtin_platform_driver(ltq_ebu_driver);
