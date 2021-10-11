// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2016 BayLibre, SAS
 * Copyright (C) 2021 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/property.h>
#include <linux/mfd/syscon.h>
#include <linux/nvmem-consumer.h>

#define HHI_VDAC_CNTL0_MESON8			0x2F4 /* 0xbd offset in data sheet */
#define HHI_VDAC_CNTL1_MESON8			0x2F8 /* 0xbe offset in data sheet */

#define HHI_VDAC_CNTL0_G12A			0x2EC /* 0xbd offset in data sheet */
#define HHI_VDAC_CNTL1_G12A			0x2F0 /* 0xbe offset in data sheet */

enum phy_meson_cvbs_dac_reg {
	MESON_CDAC_CTRL_RESV1,
	MESON_CDAC_CTRL_RESV2,
	MESON_CDAC_VREF_ADJ,
	MESON_CDAC_RL_ADJ,
	MESON_CDAC_CLK_PHASE_SEL,
	MESON_CDAC_DRIVER_ADJ,
	MESON_CDAC_EXT_VREF_EN,
	MESON_CDAC_BIAS_C,
	MESON_VDAC_CNTL0_RESERVED,
	MESON_CDAC_GSW,
	MESON_CDAC_PWD,
	MESON_VDAC_CNTL1_RESERVED,
	MESON_CVBS_DAC_NUM_REGS
};

struct phy_meson_cvbs_dac_data {
	const struct reg_field	*reg_fields;
	u8			cdac_ctrl_resv2_enable_val;
	u8			cdac_vref_adj_enable_val;
	u8			cdac_rl_adj_enable_val;
	bool			disable_ignore_cdac_pwd;
	bool			has_cvbs_trimming_nvmem_cell;
};

struct phy_meson_cvbs_dac_priv {
	struct regmap_field			*regs[MESON_CVBS_DAC_NUM_REGS];
	const struct phy_meson_cvbs_dac_data	*data;
	u8					cdac_gsw_enable_val;
};

static const struct reg_field phy_meson8_cvbs_dac_reg_fields[] = {
	[MESON_CDAC_CTRL_RESV1] =	REG_FIELD(HHI_VDAC_CNTL0_MESON8, 0, 7),
	[MESON_CDAC_CTRL_RESV2] =	REG_FIELD(HHI_VDAC_CNTL0_MESON8, 8, 15),
	[MESON_CDAC_VREF_ADJ] =		REG_FIELD(HHI_VDAC_CNTL0_MESON8, 16, 20),
	[MESON_CDAC_RL_ADJ] =		REG_FIELD(HHI_VDAC_CNTL0_MESON8, 21, 23),
	[MESON_CDAC_CLK_PHASE_SEL] =	REG_FIELD(HHI_VDAC_CNTL0_MESON8, 24, 24),
	[MESON_CDAC_DRIVER_ADJ] =	REG_FIELD(HHI_VDAC_CNTL0_MESON8, 25, 25),
	[MESON_CDAC_EXT_VREF_EN] =	REG_FIELD(HHI_VDAC_CNTL0_MESON8, 26, 26),
	[MESON_CDAC_BIAS_C] =		REG_FIELD(HHI_VDAC_CNTL0_MESON8, 27, 27),
	[MESON_VDAC_CNTL0_RESERVED] =	REG_FIELD(HHI_VDAC_CNTL0_MESON8, 28, 31),
	[MESON_CDAC_GSW] =		REG_FIELD(HHI_VDAC_CNTL1_MESON8, 0, 2),
	[MESON_CDAC_PWD] =		REG_FIELD(HHI_VDAC_CNTL1_MESON8, 3, 3),
	[MESON_VDAC_CNTL1_RESERVED] =	REG_FIELD(HHI_VDAC_CNTL1_MESON8, 4, 31),
};

static const struct reg_field phy_meson_g12a_cvbs_dac_reg_fields[] = {
	[MESON_CDAC_CTRL_RESV1] =	REG_FIELD(HHI_VDAC_CNTL0_G12A, 0, 7),
	[MESON_CDAC_CTRL_RESV2] =	REG_FIELD(HHI_VDAC_CNTL0_G12A, 8, 15),
	[MESON_CDAC_VREF_ADJ] =		REG_FIELD(HHI_VDAC_CNTL0_G12A, 16, 20),
	[MESON_CDAC_RL_ADJ] =		REG_FIELD(HHI_VDAC_CNTL0_G12A, 21, 23),
	[MESON_CDAC_CLK_PHASE_SEL] =	REG_FIELD(HHI_VDAC_CNTL0_G12A, 24, 24),
	[MESON_CDAC_DRIVER_ADJ] =	REG_FIELD(HHI_VDAC_CNTL0_G12A, 25, 25),
	[MESON_CDAC_EXT_VREF_EN] =	REG_FIELD(HHI_VDAC_CNTL0_G12A, 26, 26),
	[MESON_CDAC_BIAS_C] =		REG_FIELD(HHI_VDAC_CNTL0_G12A, 27, 27),
	[MESON_VDAC_CNTL0_RESERVED] =	REG_FIELD(HHI_VDAC_CNTL0_G12A, 28, 31),
	[MESON_CDAC_GSW] =		REG_FIELD(HHI_VDAC_CNTL1_G12A, 0, 2),
	[MESON_CDAC_PWD] =		REG_FIELD(HHI_VDAC_CNTL1_G12A, 3, 3),
	[MESON_VDAC_CNTL1_RESERVED] =	REG_FIELD(HHI_VDAC_CNTL1_G12A, 4, 31),
};

static const struct phy_meson_cvbs_dac_data phy_meson8_cvbs_dac_data = {
	.reg_fields			= phy_meson8_cvbs_dac_reg_fields,
	.cdac_ctrl_resv2_enable_val	= 0x0,
	.cdac_vref_adj_enable_val	= 0x0,
	.cdac_rl_adj_enable_val		= 0x0,
	.disable_ignore_cdac_pwd	= false,
	.has_cvbs_trimming_nvmem_cell	= true,
};

static const struct phy_meson_cvbs_dac_data phy_meson_gxbb_cvbs_dac_data = {
	.reg_fields			= phy_meson8_cvbs_dac_reg_fields,
	.cdac_ctrl_resv2_enable_val	= 0x0,
	.cdac_vref_adj_enable_val	= 0x0,
	.cdac_rl_adj_enable_val		= 0x0,
	.disable_ignore_cdac_pwd	= false,
	.has_cvbs_trimming_nvmem_cell	= false,
};

static const struct phy_meson_cvbs_dac_data phy_meson_gxl_cvbs_dac_data = {
	.reg_fields			= phy_meson8_cvbs_dac_reg_fields,
	.cdac_ctrl_resv2_enable_val	= 0x0,
	.cdac_vref_adj_enable_val	= 0xf,
	.cdac_rl_adj_enable_val		= 0x0,
	.disable_ignore_cdac_pwd	= false,
	.has_cvbs_trimming_nvmem_cell	= false,
};

static const struct phy_meson_cvbs_dac_data phy_meson_g12a_cvbs_dac_data = {
	.reg_fields			= phy_meson_g12a_cvbs_dac_reg_fields,
	.cdac_ctrl_resv2_enable_val	= 0x60,
	.cdac_vref_adj_enable_val	= 0x10,
	.cdac_rl_adj_enable_val		= 0x4,
	.disable_ignore_cdac_pwd	= true,
	.has_cvbs_trimming_nvmem_cell	= false,
};

static int phy_meson_cvbs_dac_power_on(struct phy *phy)
{
	struct phy_meson_cvbs_dac_priv *priv = phy_get_drvdata(phy);

	regmap_field_write(priv->regs[MESON_CDAC_CTRL_RESV1], 0x1);
	regmap_field_write(priv->regs[MESON_CDAC_CTRL_RESV2],
			   priv->data->cdac_ctrl_resv2_enable_val);
	regmap_field_write(priv->regs[MESON_CDAC_VREF_ADJ],
			   priv->data->cdac_vref_adj_enable_val);
	regmap_field_write(priv->regs[MESON_CDAC_RL_ADJ],
			   priv->data->cdac_rl_adj_enable_val);
	regmap_field_write(priv->regs[MESON_CDAC_GSW],
			   priv->cdac_gsw_enable_val);
	regmap_field_write(priv->regs[MESON_CDAC_PWD], 0x0);

	return 0;
}

static int phy_meson_cvbs_dac_power_off(struct phy *phy)
{
	struct phy_meson_cvbs_dac_priv *priv = phy_get_drvdata(phy);

	regmap_field_write(priv->regs[MESON_CDAC_CTRL_RESV1], 0x0);
	regmap_field_write(priv->regs[MESON_CDAC_CTRL_RESV2], 0x0);
	regmap_field_write(priv->regs[MESON_CDAC_VREF_ADJ], 0x0);
	regmap_field_write(priv->regs[MESON_CDAC_RL_ADJ], 0x0);
	regmap_field_write(priv->regs[MESON_CDAC_GSW], 0x0);

	if (priv->data->disable_ignore_cdac_pwd)
		regmap_field_write(priv->regs[MESON_CDAC_PWD], 0x0);
	else
		regmap_field_write(priv->regs[MESON_CDAC_PWD], 0x1);

	return 0;
}

static int phy_meson_cvbs_dac_init(struct phy *phy)
{
	struct phy_meson_cvbs_dac_priv *priv = phy_get_drvdata(phy);

	regmap_field_write(priv->regs[MESON_CDAC_CLK_PHASE_SEL], 0x0);
	regmap_field_write(priv->regs[MESON_CDAC_DRIVER_ADJ], 0x0);
	regmap_field_write(priv->regs[MESON_CDAC_EXT_VREF_EN], 0x0);
	regmap_field_write(priv->regs[MESON_CDAC_BIAS_C], 0x0);
	regmap_field_write(priv->regs[MESON_VDAC_CNTL0_RESERVED], 0x0);
	regmap_field_write(priv->regs[MESON_VDAC_CNTL1_RESERVED], 0x0);

	return phy_meson_cvbs_dac_power_off(phy);
}

static const struct phy_ops phy_meson_cvbs_dac_ops = {
	.init		= phy_meson_cvbs_dac_init,
	.power_on	= phy_meson_cvbs_dac_power_on,
	.power_off	= phy_meson_cvbs_dac_power_off,
	.owner		= THIS_MODULE,
};

static u8 phy_meson_cvbs_trimming_version(u8 trimming1)
{
	if ((trimming1 & 0xf0) == 0xa0)
		return 5;
	else if ((trimming1 & 0xf0) == 0x40)
		return 2;
	else if ((trimming1 & 0xc0) == 0x80)
		return 1;
	else if ((trimming1 & 0xc0) == 0x00)
		return 0;
	else
		return 0xff;
}

static int phy_meson_cvbs_read_trimming(struct device *dev,
					struct phy_meson_cvbs_dac_priv *priv)
{
	struct nvmem_cell *cell;
	u8 *trimming;
	size_t len;

	cell = devm_nvmem_cell_get(dev, "cvbs_trimming");
	if (IS_ERR(cell))
		return dev_err_probe(dev, PTR_ERR(cell),
				     "Failed to get the 'cvbs_trimming' nvmem-cell\n");

	trimming = nvmem_cell_read(cell, &len);
	if (IS_ERR(trimming))
		return dev_err_probe(dev, PTR_ERR(trimming),
				     "Failed to read the 'cvbs_trimming' nvmem-cell\n");

	if (len != 2)
		return dev_err_probe(dev, -EINVAL,
				     "Read the 'cvbs_trimming' nvmem-cell with invalid length\n");

	switch (phy_meson_cvbs_trimming_version(trimming[1])) {
	case 1:
	case 2:
	case 5:
		priv->cdac_gsw_enable_val = trimming[0] & 0x7;
		break;
	default:
		priv->cdac_gsw_enable_val = 0x0;
		break;
	}

	kfree(trimming);

	return 0;
}

static int phy_meson_cvbs_dac_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct phy_meson_cvbs_dac_priv *priv;
	struct phy_provider *phy_provider;
	struct device *dev = &pdev->dev;
	struct regmap *hhi;
	struct phy *phy;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->data = device_get_match_data(dev);
	if (!priv->data)
		return dev_err_probe(dev, -EINVAL,
				     "Could not find match data\n");

	hhi = syscon_node_to_regmap(np->parent);
	if (IS_ERR(hhi))
		return PTR_ERR(hhi);

	if (priv->data->has_cvbs_trimming_nvmem_cell) {
		ret = phy_meson_cvbs_read_trimming(dev, priv);
		if (ret)
			return ret;
	}

	ret = devm_regmap_field_bulk_alloc(dev, hhi, priv->regs,
					   priv->data->reg_fields,
					   MESON_CVBS_DAC_NUM_REGS);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to create regmap fields\n");

	phy = devm_phy_create(dev, np, &phy_meson_cvbs_dac_ops);
	if (IS_ERR(phy))
		return PTR_ERR(phy);

	phy_set_drvdata(phy, priv);

	phy_provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);

	return PTR_ERR_OR_ZERO(phy_provider);
}

static const struct of_device_id phy_meson_cvbs_dac_of_match[] = {
	{
		.compatible = "amlogic,meson8-cvbs-dac",
		.data = &phy_meson8_cvbs_dac_data,
	},
	{
		.compatible = "amlogic,meson-gxbb-cvbs-dac",
		.data = &phy_meson_gxbb_cvbs_dac_data,
	},
	{
		.compatible = "amlogic,meson-gxl-cvbs-dac",
		.data = &phy_meson_gxl_cvbs_dac_data,
	},
	{
		.compatible = "amlogic,meson-g12a-cvbs-dac",
		.data = &phy_meson_g12a_cvbs_dac_data,
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, phy_meson_cvbs_dac_of_match);

static struct platform_driver phy_meson_cvbs_dac_driver = {
	.probe	= phy_meson_cvbs_dac_probe,
	.driver	= {
		.name		= "phy-meson-cvbs-dac",
		.of_match_table	= phy_meson_cvbs_dac_of_match,
	},
};
module_platform_driver(phy_meson_cvbs_dac_driver);

MODULE_AUTHOR("Martin Blumenstingl <martin.blumenstingl@googlemail.com>");
MODULE_DESCRIPTION("Amlogic Meson CVBS DAC driver");
MODULE_LICENSE("GPL v2");
