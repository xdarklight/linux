/*
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 *
 *  Copyright (C) 2012 John Crispin <blogic@openwrt.org>
 *  Copyright (C) 2016 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 */

#include <linux/delay.h>
#include <dt-bindings/mips/lantiq_rcu_gphy.h>
#include <linux/firmware.h>
#include <linux/dma-mapping.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/reboot.h>
#include <linux/reset.h>
#include <linux/of_platform.h>

#include <lantiq_soc.h>

#define XRX200_GPHY_FW_ALIGN	(16 * 1024)

struct xway_gphy_priv {
	struct clk *gphy_clk_gate;
	struct reset_control *gphy_reset;
	struct regmap *regmap;
	struct notifier_block gphy_reboot_nb;
	u32 reg_offset;
	char fw_name[30];
};

struct xway_gphy_match_data {
	char *fe_firmware_name;
	char *ge_firmware_name;
};

static const struct xway_gphy_match_data xrx200_gphy_data = {
	.fe_firmware_name = "lantiq/xrx200_phy22f_a%dx.bin",
	.ge_firmware_name = "lantiq/xrx300_phy11g_a%dx.bin",
};

static const struct xway_gphy_match_data xrx300_gphy_data = {
	.fe_firmware_name = "lantiq/xrx200_phy22f_a%dx.bin",
	.ge_firmware_name = "lantiq/xrx300_phy11g_a%dx.bin",
};

static const struct of_device_id xway_gphy_match[] = {
	{ .compatible = "lantiq,xrx200-rcu-gphy", .data = &xrx200_gphy_data },
	{ .compatible = "lantiq,xrx300-rcu-gphy", .data = &xrx300_gphy_data },
	{ .compatible = "lantiq,xrx330-rcu-gphy", .data = &xrx300_gphy_data },
	{},
};
MODULE_DEVICE_TABLE(of, xway_gphy_match);

static struct xway_gphy_priv *to_xway_gphy_priv(struct notifier_block *nb)
{
	return container_of(nb, struct xway_gphy_priv, gphy_reboot_nb);
}

static int xway_gphy_reboot_notify(struct notifier_block *reboot_nb,
				   unsigned long code, void *unused)
{
	struct xway_gphy_priv *priv = to_xway_gphy_priv(reboot_nb);

	if (priv)
		reset_control_assert(priv->gphy_reset);

	return NOTIFY_DONE;
}

static dma_addr_t xway_gphy_load(struct platform_device *pdev,
				 const char *fw_name)
{
	const struct firmware *fw;
	dma_addr_t dev_addr = 0;
	void *fw_addr;
	size_t size;

	dev_info(&pdev->dev, "requesting %s\n", fw_name);
	if (request_firmware(&fw, fw_name, &pdev->dev)) {
		dev_err(&pdev->dev, "failed to load firmware: %s\n", fw_name);
		return 0;
	}

	/*
	 * GPHY cores need the firmware code in a persistent and contiguous
	 * memory area with a 16 kB boundary aligned start address
	 */
	size = fw->size + XRX200_GPHY_FW_ALIGN;

	fw_addr = dma_alloc_coherent(&pdev->dev, size, &dev_addr, GFP_KERNEL);
	if (fw_addr) {
		fw_addr = PTR_ALIGN(fw_addr, XRX200_GPHY_FW_ALIGN);
		dev_addr = ALIGN(dev_addr, XRX200_GPHY_FW_ALIGN);
		memcpy(fw_addr, fw->data, fw->size);
	} else {
		dev_err(&pdev->dev, "failed to alloc firmware memory\n");
	}

	release_firmware(fw);

	return dev_addr;
}

static int xway_gphy_of_probe(struct platform_device *pdev,
				struct xway_gphy_priv *priv)
{
	struct device_node *np = pdev->dev.of_node;
	const struct of_device_id *match = of_match_node(xway_gphy_match, np);
	const struct xway_gphy_match_data *gphy_fw_name_cfg;
	u32 gphy_mode;

	gphy_fw_name_cfg = match->data;

	/* Ignore all errors since this clock is optional. */
	priv->gphy_clk_gate = devm_clk_get(&pdev->dev, "gphy");

	priv->regmap = syscon_regmap_lookup_by_phandle(np,
							"lantiq,rcu-syscon");
	if (IS_ERR(priv->regmap)) {
		dev_err(&pdev->dev, "Failed to lookup RCU regmap\n");
		return PTR_ERR(priv->regmap);
	}

	if (of_property_read_u32_index(np, "lantiq,rcu-syscon", 1,
		&priv->reg_offset)) {
		dev_err(&pdev->dev, "Failed to get RCU reg offset\n");
		return -EINVAL;
	}

	priv->gphy_reset = devm_reset_control_get(&pdev->dev, "gphy");
	if (IS_ERR_OR_NULL(priv->gphy_reset)) {
		dev_err(&pdev->dev, "Failed to lookup gphy reset\n");
		return PTR_ERR(priv->gphy_reset);
	}

	if (of_property_read_u32(np, "lantiq,gphy-mode", &gphy_mode))
		/* Default to GE mode */
		gphy_mode = GPHY_MODE_GE;

	switch (gphy_mode) {
	case GPHY_MODE_FE:
		snprintf(priv->fw_name, sizeof(priv->fw_name),
				gphy_fw_name_cfg->fe_firmware_name,
				ltq_soc_type());
		break;
	case GPHY_MODE_GE:
		snprintf(priv->fw_name, sizeof(priv->fw_name),
				gphy_fw_name_cfg->ge_firmware_name,
				ltq_soc_type());
		break;
	default:
		dev_err(&pdev->dev, "Unknown GPHY mode %d\n", gphy_mode);
		return -EINVAL;
	}

	return 0;
}

static int xway_gphy_probe(struct platform_device *pdev)
{
	struct xway_gphy_priv *priv;
	dma_addr_t fw_addr;
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	ret = xway_gphy_of_probe(pdev, priv);
	if (ret)
		return ret;

	if (!IS_ERR_OR_NULL(priv->gphy_clk_gate))
		clk_prepare_enable(priv->gphy_clk_gate);

	fw_addr = xway_gphy_load(pdev, priv->fw_name);
	if (!fw_addr)
		return -EINVAL;

	reset_control_assert(priv->gphy_reset);

	ret = regmap_write(priv->regmap, priv->reg_offset, fw_addr);
	if (ret) {
		dev_err(&pdev->dev, "Failed to configure FW addr\n");
		return ret;
	}

	reset_control_deassert(priv->gphy_reset);

	/* assert the gphy reset because it can hang after a reboot: */
	priv->gphy_reboot_nb.notifier_call = xway_gphy_reboot_notify;
	priv->gphy_reboot_nb.priority = -1;

	ret = register_reboot_notifier(&priv->gphy_reboot_nb);
	if (ret)
		dev_warn(&pdev->dev, "Failed to register reboot notifier\n");

	/* wait for the GPHY to come up */
	mdelay(100);

	return 0;
}

static struct platform_driver xway_gphy_driver = {
	.probe = xway_gphy_probe,
	.driver = {
		.name = "xway-rcu-gphy",
		.of_match_table = xway_gphy_match,
	},
};

module_platform_driver(xway_gphy_driver);

MODULE_FIRMWARE("lantiq/xrx300_phy11g_a2x.bin");
MODULE_FIRMWARE("lantiq/xrx300_phy22f_a2x.bin");
MODULE_FIRMWARE("lantiq/xrx200_phy11g_a1x.bin");
MODULE_FIRMWARE("lantiq/xrx200_phy11g_a2x.bin");
MODULE_FIRMWARE("lantiq/xrx200_phy22f_a1x.bin");
MODULE_FIRMWARE("lantiq/xrx200_phy22f_a2x.bin");
MODULE_AUTHOR("Martin Blumenstingl <martin.blumenstingl@googlemail.com>");
MODULE_DESCRIPTION("Lantiq XWAY GPHY Firmware Loader");
MODULE_LICENSE("GPL");
