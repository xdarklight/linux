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

struct phy_meson_cvbs_dac_regs {
	struct reg_field	cdac_ctrl_resv1;
	struct reg_field	cdac_ctrl_resv2;
	struct reg_field	cdac_vref_adj;
	struct reg_field	cdac_rl_adj;
	struct reg_field	cdac_clk_phase_sel;
	struct reg_field	cdac_driver_adj;
	struct reg_field	cdac_ext_vref_en;
	struct reg_field	cdac_bias_c;
	struct reg_field	vdac_cntl0_reserved;
	struct reg_field	cdac_gsw;
	struct reg_field	cdac_pwd;
	struct reg_field	vdac_cntl1_reserved;
};

struct phy_meson_cvbs_dac_data {
	const struct phy_meson_cvbs_dac_regs	*regs;
	u8					cdac_ctrl_resv2_enable_val;
	u8					cdac_vref_adj_enable_val;
	u8					cdac_rl_adj_enable_val;
	bool					disable_ignore_cdac_pwd;
	bool					has_cvbs_trimming_nvmem_cell;
};

struct phy_meson_cvbs_dac_priv {
	struct regmap_field			*cdac_ctrl_resv1;
	struct regmap_field			*cdac_ctrl_resv2;
	struct regmap_field			*cdac_vref_adj;
	struct regmap_field			*cdac_rl_adj;
	struct regmap_field			*cdac_clk_phase_sel;
	struct regmap_field			*cdac_driver_adj;
	struct regmap_field			*cdac_ext_vref_en;
	struct regmap_field			*cdac_bias_c;
	struct regmap_field			*vdac_cntl0_reserved;
	struct regmap_field			*cdac_gsw;
	struct regmap_field			*cdac_pwd;
	struct regmap_field			*vdac_cntl1_reserved;
	const struct phy_meson_cvbs_dac_data	*data;
	u8					cdac_gsw_enable_val;
};

static const struct phy_meson_cvbs_dac_regs phy_meson8_cvbs_dac_regs = {
	.cdac_ctrl_resv1	= REG_FIELD(HHI_VDAC_CNTL0_MESON8, 0, 7),
	.cdac_ctrl_resv2	= REG_FIELD(HHI_VDAC_CNTL0_MESON8, 8, 15),
	.cdac_vref_adj		= REG_FIELD(HHI_VDAC_CNTL0_MESON8, 16, 20),
	.cdac_rl_adj		= REG_FIELD(HHI_VDAC_CNTL0_MESON8, 21, 23),
	.cdac_clk_phase_sel	= REG_FIELD(HHI_VDAC_CNTL0_MESON8, 24, 24),
	.cdac_driver_adj	= REG_FIELD(HHI_VDAC_CNTL0_MESON8, 25, 25),
	.cdac_ext_vref_en	= REG_FIELD(HHI_VDAC_CNTL0_MESON8, 26, 26),
	.cdac_bias_c		= REG_FIELD(HHI_VDAC_CNTL0_MESON8, 27, 27),
	.vdac_cntl0_reserved	= REG_FIELD(HHI_VDAC_CNTL0_MESON8, 28, 31),
	.cdac_gsw		= REG_FIELD(HHI_VDAC_CNTL1_MESON8, 0, 2),
	.cdac_pwd		= REG_FIELD(HHI_VDAC_CNTL1_MESON8, 3, 3),
	.vdac_cntl1_reserved	= REG_FIELD(HHI_VDAC_CNTL1_MESON8, 4, 31),
};

static const struct phy_meson_cvbs_dac_regs phy_meson_g12a_cvbs_dac_regs = {
	.cdac_ctrl_resv1	= REG_FIELD(HHI_VDAC_CNTL0_G12A, 0, 7),
	.cdac_ctrl_resv2	= REG_FIELD(HHI_VDAC_CNTL0_G12A, 8, 15),
	.cdac_vref_adj		= REG_FIELD(HHI_VDAC_CNTL0_G12A, 16, 20),
	.cdac_rl_adj		= REG_FIELD(HHI_VDAC_CNTL0_G12A, 21, 23),
	.cdac_clk_phase_sel	= REG_FIELD(HHI_VDAC_CNTL0_G12A, 24, 24),
	.cdac_driver_adj	= REG_FIELD(HHI_VDAC_CNTL0_G12A, 25, 25),
	.cdac_ext_vref_en	= REG_FIELD(HHI_VDAC_CNTL0_G12A, 26, 26),
	.cdac_bias_c		= REG_FIELD(HHI_VDAC_CNTL0_G12A, 27, 27),
	.vdac_cntl0_reserved	= REG_FIELD(HHI_VDAC_CNTL0_G12A, 28, 31),
	.cdac_gsw		= REG_FIELD(HHI_VDAC_CNTL1_G12A, 0, 2),
	.cdac_pwd		= REG_FIELD(HHI_VDAC_CNTL1_G12A, 3, 3),
	.vdac_cntl1_reserved	= REG_FIELD(HHI_VDAC_CNTL1_G12A, 4, 31),
};

static const struct phy_meson_cvbs_dac_data phy_meson8_cvbs_dac_data = {
	.regs				= &phy_meson8_cvbs_dac_regs,
	.cdac_ctrl_resv2_enable_val	= 0x0,
	.cdac_vref_adj_enable_val	= 0x0,
	.cdac_rl_adj_enable_val		= 0x0,
	.disable_ignore_cdac_pwd	= false,
	.has_cvbs_trimming_nvmem_cell	= true,
};

static const struct phy_meson_cvbs_dac_data phy_meson_gxbb_cvbs_dac_data = {
	.regs				= &phy_meson8_cvbs_dac_regs,
	.cdac_ctrl_resv2_enable_val	= 0x0,
	.cdac_vref_adj_enable_val	= 0x0,
	.cdac_rl_adj_enable_val		= 0x0,
	.disable_ignore_cdac_pwd	= false,
	.has_cvbs_trimming_nvmem_cell	= false,
};

static const struct phy_meson_cvbs_dac_data phy_meson_gxl_cvbs_dac_data = {
	.regs				= &phy_meson8_cvbs_dac_regs,
	.cdac_ctrl_resv2_enable_val	= 0x0,
	.cdac_vref_adj_enable_val	= 0xf,
	.cdac_rl_adj_enable_val		= 0x0,
	.disable_ignore_cdac_pwd	= false,
	.has_cvbs_trimming_nvmem_cell	= false,
};

static const struct phy_meson_cvbs_dac_data phy_meson_g12a_cvbs_dac_data = {
	.regs				= &phy_meson_g12a_cvbs_dac_regs,
	.cdac_ctrl_resv2_enable_val	= 0x60,
	.cdac_vref_adj_enable_val	= 0x10,
	.cdac_rl_adj_enable_val		= 0x4,
	.disable_ignore_cdac_pwd	= true,
	.has_cvbs_trimming_nvmem_cell	= false,
};

static int phy_meson_cvbs_dac_power_on(struct phy *phy)
{
	struct phy_meson_cvbs_dac_priv *priv = phy_get_drvdata(phy);

	regmap_field_write(priv->cdac_ctrl_resv1, 0x1);
	regmap_field_write(priv->cdac_ctrl_resv1,
			   priv->data->cdac_ctrl_resv2_enable_val);
	regmap_field_write(priv->cdac_vref_adj,
			   priv->data->cdac_vref_adj_enable_val);
	regmap_field_write(priv->cdac_rl_adj,
			   priv->data->cdac_rl_adj_enable_val);
	regmap_field_write(priv->cdac_gsw, priv->cdac_gsw_enable_val);
	regmap_field_write(priv->cdac_pwd, 0x0);

	return 0;
}

static int phy_meson_cvbs_dac_power_off(struct phy *phy)
{
	struct phy_meson_cvbs_dac_priv *priv = phy_get_drvdata(phy);


	regmap_field_write(priv->cdac_ctrl_resv1, 0x0);
	regmap_field_write(priv->cdac_ctrl_resv1, 0x0);
	regmap_field_write(priv->cdac_vref_adj, 0x0);
	regmap_field_write(priv->cdac_rl_adj, 0x0);
	regmap_field_write(priv->cdac_gsw, 0x0);

	if (priv->data->disable_ignore_cdac_pwd)
		regmap_field_write(priv->cdac_pwd, 0x0);
	else
		regmap_field_write(priv->cdac_pwd, 0x1);

	return 0;
}

static int phy_meson_cvbs_dac_init(struct phy *phy)
{
	struct phy_meson_cvbs_dac_priv *priv = phy_get_drvdata(phy);

	regmap_field_write(priv->cdac_clk_phase_sel, 0x0);
	regmap_field_write(priv->cdac_driver_adj, 0x0);
	regmap_field_write(priv->cdac_ext_vref_en, 0x0);
	regmap_field_write(priv->cdac_bias_c, 0x0);
	regmap_field_write(priv->vdac_cntl0_reserved, 0x0);
	regmap_field_write(priv->vdac_cntl1_reserved, 0x0);

	return phy_meson_cvbs_dac_power_off(phy);
}

static const struct phy_ops phy_meson_cvbs_dac_ops = {
	.init		= phy_meson_cvbs_dac_init,
	.power_on	= phy_meson_cvbs_dac_power_on,
	.power_off	= phy_meson_cvbs_dac_power_off,
	.owner		= THIS_MODULE,
};

static int phy_meson_cvbs_dac_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	const struct phy_meson_cvbs_dac_regs *regs;
	struct phy_meson_cvbs_dac_priv *priv;
	struct phy_provider *phy_provider;
	struct device *dev = &pdev->dev;
	struct regmap *hhi;
	struct phy *phy;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->data = device_get_match_data(dev);
	if (!priv->data)
		return dev_err_probe(dev, -EINVAL,
				     "Could not find match data\n");

	regs = priv->data->regs;

	hhi = syscon_node_to_regmap(np->parent);
	if (IS_ERR(hhi))
		return PTR_ERR(hhi);

	if (priv->data->has_cvbs_trimming_nvmem_cell) {
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

		if ((trimming[1] & 0xf0) == 0xa0 ||
		    (trimming[1] & 0xf0) == 0x40 ||
		    (trimming[1] & 0xc0) == 0x80)
			priv->cdac_gsw_enable_val = trimming[0] & 0x7;
		else
			priv->cdac_gsw_enable_val = 0x0;
	}

	priv->cdac_ctrl_resv1 = devm_regmap_field_alloc(dev, hhi,
							regs->cdac_ctrl_resv1);
	if (IS_ERR(priv->cdac_ctrl_resv1))
		return dev_err_probe(dev, PTR_ERR(priv->cdac_ctrl_resv1),
				     "Failed to create regmap field for cdac_ctrl_resv1\n");

	priv->cdac_ctrl_resv2 = devm_regmap_field_alloc(dev, hhi,
							regs->cdac_ctrl_resv2);
	if (IS_ERR(priv->cdac_ctrl_resv2))
		return dev_err_probe(dev, PTR_ERR(priv->cdac_ctrl_resv2),
				     "Failed to create regmap field for cdac_ctrl_resv2\n");

	priv->cdac_vref_adj = devm_regmap_field_alloc(dev, hhi,
						      regs->cdac_vref_adj);
	if (IS_ERR(priv->cdac_vref_adj))
		return dev_err_probe(dev, PTR_ERR(priv->cdac_vref_adj),
				     "Failed to create regmap field for cdac_vref_adj\n");

	priv->cdac_rl_adj = devm_regmap_field_alloc(dev, hhi,
						    regs->cdac_rl_adj);
	if (IS_ERR(priv->cdac_rl_adj))
		return dev_err_probe(dev, PTR_ERR(priv->cdac_rl_adj),
				     "Failed to create regmap field for cdac_rl_adj\n");

	priv->cdac_clk_phase_sel = devm_regmap_field_alloc(dev, hhi,
							   regs->cdac_clk_phase_sel);
	if (IS_ERR(priv->cdac_clk_phase_sel))
		return dev_err_probe(dev, PTR_ERR(priv->cdac_clk_phase_sel),
				     "Failed to create regmap field for cdac_clk_phase_sel\n");

	priv->cdac_driver_adj = devm_regmap_field_alloc(dev, hhi,
							regs->cdac_driver_adj);
	if (IS_ERR(priv->cdac_driver_adj))
		return dev_err_probe(dev, PTR_ERR(priv->cdac_driver_adj),
				     "Failed to create regmap field for cdac_driver_adj\n");

	priv->cdac_ext_vref_en = devm_regmap_field_alloc(dev, hhi,
							 regs->cdac_ext_vref_en);
	if (IS_ERR(priv->cdac_ext_vref_en))
		return dev_err_probe(dev, PTR_ERR(priv->cdac_ext_vref_en),
				     "Failed to create regmap field for cdac_ext_vref_en\n");

	priv->cdac_bias_c = devm_regmap_field_alloc(dev, hhi,
						    regs->cdac_bias_c);
	if (IS_ERR(priv->cdac_bias_c))
		return dev_err_probe(dev, PTR_ERR(priv->cdac_bias_c),
				     "Failed to create regmap field for cdac_bias_c\n");

	priv->vdac_cntl0_reserved = devm_regmap_field_alloc(dev, hhi,
							    regs->vdac_cntl0_reserved);
	if (IS_ERR(priv->vdac_cntl0_reserved))
		return dev_err_probe(dev, PTR_ERR(priv->vdac_cntl0_reserved),
				     "Failed to create regmap field for vdac_cntl0_reserved\n");

	priv->cdac_gsw = devm_regmap_field_alloc(dev, hhi, regs->cdac_gsw);
	if (IS_ERR(priv->cdac_gsw))
		return dev_err_probe(dev, PTR_ERR(priv->cdac_gsw),
				     "Failed to create regmap field for cdac_gsw\n");

	priv->cdac_pwd = devm_regmap_field_alloc(dev, hhi, regs->cdac_pwd);
	if (IS_ERR(priv->cdac_pwd))
		return dev_err_probe(dev, PTR_ERR(priv->cdac_pwd),
				     "Failed to create regmap field for cdac_pwd\n");

	priv->vdac_cntl1_reserved = devm_regmap_field_alloc(dev, hhi,
							    regs->vdac_cntl1_reserved);
	if (IS_ERR(priv->vdac_cntl1_reserved))
		return dev_err_probe(dev, PTR_ERR(priv->vdac_cntl1_reserved),
				     "Failed to create regmap field for vdac_cntl1_reserved\n");

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
