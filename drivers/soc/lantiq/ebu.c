// SPDX-License-Identifier: GPL-2.0-only
/*
 *  Copyright (C) 2019-2022 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 *  Copyright (C) 2011-2012 John Crispin <john@phrozen.org>
 */

#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/ioport.h>
#include <linux/irqchip.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/regmap.h>

#define LTQ_EBU_BUSCON0				0x60
#define LTQ_EBU_BUSCON_WRDIS			BIT(31)
#define LTQ_EBU_PCC_CON				0x90
#define LTQ_EBU_PCC_CON_PCCARD_ON		BIT(0)
#define LTQ_EBU_PCC_CON_IREQ_DIS		0x0
#define LTQ_EBU_PCC_CON_IREQ_RISING_EDGE	BIT(1)
#define LTQ_EBU_PCC_CON_IREQ_FALLING_EDGE	BIT(2)
#define LTQ_EBU_PCC_CON_IREQ_LEVEL_DETECT	BIT(3)
#define LTQ_EBU_PCC_CON_IREQ_MASK		GENMASK(3, 1)
#define LTQ_EBU_ISTAT				0xa0
#define LTQ_EBU_ISTAT_PCI			BIT(4)
#define LTQ_EBU_ISTAT_ECC			BIT(5)
#define LTQ_EBU_IEN				0xa4
#define LTQ_EBU_IEN_PCI				BIT(4)
#define LTQ_EBU_IEN_ECC				BIT(5)

void __iomem *ltq_ebu_membase;

struct ltq_ebu_irq_priv {
	struct regmap *regmap;
	struct irq_domain *domain;
};

struct ltq_ebu_data {
	bool initialize_buscon0_wrdis;
};

static void ltq_ebu_ack_irq(struct irq_data *d)
{
	struct ltq_ebu_irq_priv *priv = irq_data_get_irq_chip_data(d);

	regmap_write(priv->regmap, LTQ_EBU_ISTAT,
		     d->hwirq ? LTQ_EBU_ISTAT_ECC : LTQ_EBU_ISTAT_PCI);
}

static void ltq_ebu_mask_irq(struct irq_data *d)
{
	struct ltq_ebu_irq_priv *priv = irq_data_get_irq_chip_data(d);

	regmap_clear_bits(priv->regmap, LTQ_EBU_IEN,
			  d->hwirq ? LTQ_EBU_IEN_ECC : LTQ_EBU_IEN_PCI);
}

static void ltq_ebu_unmask_irq(struct irq_data *d)
{
	struct ltq_ebu_irq_priv *priv = irq_data_get_irq_chip_data(d);

	regmap_set_bits(priv->regmap, LTQ_EBU_IEN,
			d->hwirq ? LTQ_EBU_IEN_ECC : LTQ_EBU_IEN_PCI);
}

static int ltq_ebu_set_irq_type(struct irq_data *d, unsigned int flow_type)
{
	struct ltq_ebu_irq_priv *priv = irq_data_get_irq_chip_data(d);
	u32 val;

	/* Only the type of the PCI interrupt (hwirq 0) can be configured */
	if (d->hwirq != 0) {
		if (flow_type != IRQ_TYPE_NONE)
			 return -EINVAL;

		return 0;
	}

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
		return -EINVAL;
	}

	regmap_update_bits(priv->regmap, LTQ_EBU_PCC_CON,
			   LTQ_EBU_PCC_CON_IREQ_MASK, val);

	return 0;
}

static struct irq_chip ltq_ebu_irq_chip = {
	.name = "EBU",
	.irq_ack = ltq_ebu_ack_irq,
	.irq_mask = ltq_ebu_mask_irq,
	.irq_unmask = ltq_ebu_unmask_irq,
	.irq_set_type = ltq_ebu_set_irq_type,
};

static void ltq_ebu_irq_handler(struct irq_desc *desc)
{
	struct ltq_ebu_irq_priv *priv = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	u32 istat = 0;

	chained_irq_enter(chip, desc);

	regmap_read(priv->regmap, LTQ_EBU_ISTAT, &istat);

	if (unlikely(!(istat & (LTQ_EBU_ISTAT_PCI | LTQ_EBU_ISTAT_ECC)))) {
		/*
		 * EBU likes to generate spurious interrupts. ACK'ing all
		 * pending interrupts together with the PCI interrupt also
		 * ACK's the spurious EBU interrupt.
		 */
		regmap_update_bits(priv->regmap, LTQ_EBU_ISTAT,
				   LTQ_EBU_ISTAT_PCI, LTQ_EBU_ISTAT_PCI);
		chained_irq_exit(chip, desc);
		return;
	}

	if (istat & LTQ_EBU_ISTAT_PCI)
		generic_handle_domain_irq(priv->domain, 0);
	if (istat & LTQ_EBU_ISTAT_ECC)
		generic_handle_domain_irq(priv->domain, 1);

	chained_irq_exit(chip, desc);
}

static int ltq_ebu_irq_domain_alloc(struct irq_domain *domain,
				    unsigned int virq, unsigned int nr_irqs,
				    void *arg)
{
	struct ltq_ebu_irq_priv *priv = domain->host_data;
	struct irq_fwspec *fwspec = arg;
	irq_hw_number_t hwirq;
	unsigned int type;
	int ret;

	if (nr_irqs != 1)
		return -EINVAL;

	ret = irq_domain_translate_twocell(domain, fwspec, &hwirq, &type);
	if (ret)
		return ret;

	irq_domain_set_info(domain, virq, hwirq, &ltq_ebu_irq_chip, priv,
			    handle_level_irq, NULL, NULL);

	return 0;
}

static const struct irq_domain_ops ltq_ebu_irqdomain_ops = {
	.translate	= irq_domain_translate_twocell,
	.alloc		= ltq_ebu_irq_domain_alloc,
	.free		= irq_domain_free_irqs_common,
};

static const struct regmap_range ebu_regmap_ranges[] = {
	{ .range_min = 0x00, .range_max = 0x00 }, /* CLC */
	{ .range_min = 0x08, .range_max = 0x08 }, /* ID */
	{ .range_min = 0x10, .range_max = 0x10 }, /* CON */
	{ .range_min = 0x20, .range_max = 0x2c }, /* ADDRSEL0..3 */
	{ .range_min = 0x60, .range_max = 0x6c }, /* BUSCON0..3 */
	{ .range_min = LTQ_EBU_PCC_CON, .range_max = LTQ_EBU_PCC_CON },
	{ .range_min = 0x94, .range_max = 0x94 }, /* PCC_STAT */
	{ .range_min = LTQ_EBU_ISTAT, .range_max = LTQ_EBU_ISTAT },
	{ .range_min = LTQ_EBU_IEN, .range_max = LTQ_EBU_IEN },
	{ .range_min = 0xa8, .range_max = 0xa8 }, /* INT_OUT */
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

static int ltq_ebu_clk_prepare_enable(struct device *dev, struct clk *clk)
{
	int ret;

	ret = clk_prepare_enable(clk);
	if (ret)
		return ret;

	devm_add_action_or_reset(dev,
				 (void(*)(void *))clk_disable_unprepare,
				 clk);

	return 0;
}

static int ltq_ebu_probe(struct platform_device *pdev)
{
	const struct ltq_ebu_data *ebu_data;
	struct ltq_ebu_irq_priv *irq_priv;
	struct regmap *regmap;
	struct clk *clk;
	int irq, ret;

	clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(clk))
		return dev_err_probe(&pdev->dev, PTR_ERR(clk),
				     "Failed to get the EBU clock\n");

	ret = ltq_ebu_clk_prepare_enable(&pdev->dev, clk);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "Failed to enable the EBU clock\n");

	ltq_ebu_membase = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ltq_ebu_membase))
		return dev_err_probe(&pdev->dev, PTR_ERR(ltq_ebu_membase),
				     "Failed to map the EBU address space\n");

	regmap = devm_regmap_init_mmio(&pdev->dev, ltq_ebu_membase,
				       &ebu_regmap_config);
	if (IS_ERR(regmap))
		return dev_err_probe(&pdev->dev, PTR_ERR(regmap),
				     "Failed to register EBU MMIO regmap\n");

	ebu_data = of_device_get_match_data(&pdev->dev);
	if (ebu_data && ebu_data->initialize_buscon0_wrdis) {
		regmap_update_bits(regmap, LTQ_EBU_BUSCON0,
				   LTQ_EBU_BUSCON_WRDIS, 0);
	}

	if (device_property_read_bool(&pdev->dev, "interrupt-controller")) {
		/* Mask and ack any pending IRQ */
		regmap_write(regmap, LTQ_EBU_IEN, 0);
		regmap_write(regmap, LTQ_EBU_ISTAT, ~0);

		irq = platform_get_irq(pdev, 0);
		if (irq < 0)
			return irq;

		irq_priv = devm_kzalloc(&pdev->dev, sizeof(*irq_priv),
					GFP_KERNEL);
		if (!irq_priv)
			return -ENOMEM;

		irq_priv->regmap = regmap;
		irq_priv->domain = irq_domain_add_linear(pdev->dev.of_node, 2,
							 &ltq_ebu_irqdomain_ops,
							 irq_priv);
		if (!irq_priv->domain)
			return -ENOMEM;

		irq_set_chained_handler_and_data(irq, ltq_ebu_irq_handler,
						 irq_priv);
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
		.compatible = "lantiq,falcon-ebu",
		.data = &ltq_ebu_falcon_data,
	},
	{
		.compatible = "lantiq,xrx100-ebu",
		.data = &ltq_ebu_xway_data,
	},
	{
		.compatible = "lantiq,xrx200-ebu",
		.data = &ltq_ebu_xway_data,
	},
	{ /* sentinel */ }
};

static struct platform_driver ltq_ebu_driver = {
	.probe = ltq_ebu_probe,
	.driver = {
		.name = "ltq-ebu",
		.of_match_table = ltq_ebu_of_ids,
	},
};
builtin_platform_driver(ltq_ebu_driver);
