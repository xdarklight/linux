// SPDX-License-Identifier: GPL-2.0-only
/*
 * Amlogic Meson6 and Meson8 DWMAC glue layer
 *
 * Copyright (C) 2014 Beniamino Galvani <b.galvani@gmail.com>
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/ethtool.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/stmmac.h>

#include "stmmac_platform.h"

#define ETHMAC_DIV_EN		BIT(0)
/* divides the input clock by 20 (= 0x0) or 2 (= 0x1) */
#define ETHMAC_SPEED_100	BIT(1)

struct meson_dwmac {
	struct device	*dev;
	void __iomem	*reg;
	struct clk	*ethernet_clk;
};

static void meson6_dwmac_fix_mac_speed(void *priv, unsigned int speed)
{
	struct meson_dwmac *dwmac = priv;
	unsigned int val;

	val = readl(dwmac->reg);

	switch (speed) {
	case SPEED_10:
		val &= ~ETHMAC_SPEED_100;
		break;
	case SPEED_100:
		val |= ETHMAC_SPEED_100;
		break;
	}

	writel(val, dwmac->reg);
}

static int meson6_dwmac_init(struct platform_device *pdev, void *priv)
{
	struct meson_dwmac *dwmac = priv;
	int ret;

	ret = clk_set_rate(dwmac->ethernet_clk, 50 * 1000 * 1000);
	if (ret)
		return ret;

	ret = clk_prepare_enable(dwmac->ethernet_clk);
	if (ret)
		return ret;

	writel(readl(dwmac->reg) | ETHMAC_DIV_EN, dwmac->reg);

	return 0;
}

static void meson6_dwmac_exit(struct platform_device *pdev, void *priv)
{
	struct meson_dwmac *dwmac = priv;

	writel(readl(dwmac->reg) & ~ETHMAC_DIV_EN, dwmac->reg);

	clk_disable_unprepare(dwmac->ethernet_clk);
}

static int meson6_dwmac_probe(struct platform_device *pdev)
{
	struct plat_stmmacenet_data *plat_dat;
	struct stmmac_resources stmmac_res;
	struct meson_dwmac *dwmac;
	int ret;

	ret = stmmac_get_platform_resources(pdev, &stmmac_res);
	if (ret)
		return ret;

	plat_dat = stmmac_probe_config_dt(pdev, &stmmac_res.mac);
	if (IS_ERR(plat_dat))
		return PTR_ERR(plat_dat);

	dwmac = devm_kzalloc(&pdev->dev, sizeof(*dwmac), GFP_KERNEL);
	if (!dwmac) {
		ret = -ENOMEM;
		goto err_remove_config_dt;
	}

	dwmac->reg = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(dwmac->reg)) {
		ret = PTR_ERR(dwmac->reg);
		goto err_remove_config_dt;
	}

	dwmac->ethernet_clk = devm_clk_get_optional(&pdev->dev, "ethernet");
	if (IS_ERR(dwmac->ethernet_clk)) {
		ret = PTR_ERR(dwmac->ethernet_clk);
		goto err_remove_config_dt;
	}

	plat_dat->bsp_priv = dwmac;
	plat_dat->init = meson6_dwmac_init;
	plat_dat->exit = meson6_dwmac_exit;
	plat_dat->fix_mac_speed = meson6_dwmac_fix_mac_speed;

	ret = stmmac_dvr_probe(&pdev->dev, plat_dat, &stmmac_res);
	if (ret)
		goto err_remove_config_dt;

	return 0;

err_remove_config_dt:
	stmmac_remove_config_dt(pdev, plat_dat);

	return ret;
}

static const struct of_device_id meson6_dwmac_match[] = {
	{ .compatible = "amlogic,meson6-dwmac" },
	{ }
};
MODULE_DEVICE_TABLE(of, meson6_dwmac_match);

static struct platform_driver meson6_dwmac_driver = {
	.probe  = meson6_dwmac_probe,
	.remove = stmmac_pltfr_remove,
	.driver = {
		.name           = "meson6-dwmac",
		.pm		= &stmmac_pltfr_pm_ops,
		.of_match_table = meson6_dwmac_match,
	},
};
module_platform_driver(meson6_dwmac_driver);

MODULE_AUTHOR("Beniamino Galvani <b.galvani@gmail.com>");
MODULE_DESCRIPTION("Amlogic Meson6 and Meson8 DWMAC glue layer");
MODULE_LICENSE("GPL v2");
