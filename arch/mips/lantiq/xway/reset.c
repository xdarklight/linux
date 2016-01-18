/*
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 *
 *  Copyright (C) 2010 John Crispin <john@phrozen.org>
 *  Copyright (C) 2013-2015 Lantiq Beteiligungs-GmbH & Co.KG
 */

#include <linux/init.h>
#include <linux/pm.h>
#include <linux/delay.h>
#include <linux/mfd/core.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/regmap.h>

#include <asm/reboot.h>

#include <lantiq_soc.h>

#include "../prom.h"

/* reset request register */
#define RCU_RST_REQ		0x0010
/* vr9 gphy registers */
#define RCU_GFS_ADD0_XRX200	0x0020
#define RCU_GFS_ADD1_XRX200	0x0068
/* xRX300 gphy registers */
#define RCU_GFS_ADD0_XRX300	0x0020
#define RCU_GFS_ADD1_XRX300	0x0058
#define RCU_GFS_ADD2_XRX300	0x00AC
/* xRX330 gphy registers */
#define RCU_GFS_ADD0_XRX330	0x0020
#define RCU_GFS_ADD1_XRX330	0x0058
#define RCU_GFS_ADD2_XRX330	0x00AC
#define RCU_GFS_ADD3_XRX330	0x0264

/* reboot bit */
#define RCU_RD_GPHY0_XRX200	BIT(31)
#define RCU_RD_SRST		BIT(30)
#define RCU_RD_GPHY1_XRX200	BIT(29)
/* xRX300 bits */
#define RCU_RD_GPHY0_XRX300	BIT(31)
#define RCU_RD_GPHY1_XRX300	BIT(29)
#define RCU_RD_GPHY2_XRX300	BIT(28)
/* xRX330 bits */
#define RCU_RD_GPHY0_XRX330	BIT(31)
#define RCU_RD_GPHY1_XRX330	BIT(29)
#define RCU_RD_GPHY2_XRX330	BIT(28)
#define RCU_RD_GPHY3_XRX330	BIT(10)

static struct device_node *ltq_rcu_np;
static struct regmap *rcu_regmap;

struct ltq_gphy_reset {
	u32 rd;
	u32 addr;
};

/* reset / boot a gphy */
static struct ltq_gphy_reset xrx200_gphy[] = {
	{RCU_RD_GPHY0_XRX200, RCU_GFS_ADD0_XRX200},
	{RCU_RD_GPHY1_XRX200, RCU_GFS_ADD1_XRX200},
};

/* reset / boot a gphy */
static struct ltq_gphy_reset xrx300_gphy[] = {
	{RCU_RD_GPHY0_XRX300, RCU_GFS_ADD0_XRX300},
	{RCU_RD_GPHY1_XRX300, RCU_GFS_ADD1_XRX300},
	{RCU_RD_GPHY2_XRX300, RCU_GFS_ADD2_XRX300},
};

/* reset / boot a gphy */
static struct ltq_gphy_reset xrx330_gphy[] = {
	{RCU_RD_GPHY0_XRX330, RCU_GFS_ADD0_XRX330},
	{RCU_RD_GPHY1_XRX330, RCU_GFS_ADD1_XRX330},
	{RCU_RD_GPHY2_XRX330, RCU_GFS_ADD2_XRX330},
	{RCU_RD_GPHY3_XRX330, RCU_GFS_ADD3_XRX330},
};

static void xrx200_gphy_boot_addr(struct ltq_gphy_reset *phy_regs,
				  dma_addr_t dev_addr)
{
	regmap_update_bits(rcu_regmap, RCU_RST_REQ, phy_regs->rd,
			   phy_regs->rd);
	regmap_write(rcu_regmap, phy_regs->addr, dev_addr);
	regmap_update_bits(rcu_regmap, RCU_RST_REQ, phy_regs->rd, 0);
}

/* reset and boot a gphy. these phys only exist on xrx200 SoC */
int xrx200_gphy_boot(struct device *dev, unsigned int id, dma_addr_t dev_addr)
{
	struct clk *clk;

	if (of_machine_is_compatible("lantiq,vr9")) {
		clk = clk_get_sys("1f203000.rcu", "gphy");
		if (IS_ERR(clk))
			return PTR_ERR(clk);
		clk_prepare_enable(clk);
	}

	dev_info(dev, "booting GPHY%u firmware at %X\n", id, dev_addr);

	if (of_machine_is_compatible("lantiq,vr9")) {
		if (id >= ARRAY_SIZE(xrx200_gphy)) {
			dev_err(dev, "%u is an invalid gphy id\n", id);
			return -EINVAL;
		}
		xrx200_gphy_boot_addr(&xrx200_gphy[id], dev_addr);
	} else if (of_machine_is_compatible("lantiq,ar10")) {
		if (id >= ARRAY_SIZE(xrx300_gphy)) {
			dev_err(dev, "%u is an invalid gphy id\n", id);
			return -EINVAL;
		}
		xrx200_gphy_boot_addr(&xrx300_gphy[id], dev_addr);
	} else if (of_machine_is_compatible("lantiq,grx390")) {
		if (id >= ARRAY_SIZE(xrx330_gphy)) {
			dev_err(dev, "%u is an invalid gphy id\n", id);
			return -EINVAL;
		}
		xrx200_gphy_boot_addr(&xrx330_gphy[id], dev_addr);
	}
	return 0;
}

static void ltq_machine_restart(char *command)
{
	u32 val = RCU_RD_SRST;

	if (of_device_is_compatible(ltq_rcu_np, "lantiq,rcu-xrx200"))
		val |= RCU_RD_GPHY1_XRX200 | RCU_RD_GPHY0_XRX200;

	regmap_update_bits(rcu_regmap, RCU_RST_REQ, val, val);

	local_irq_disable();
	unreachable();
}

static void ltq_machine_halt(void)
{
	local_irq_disable();
	unreachable();
}

static void ltq_machine_power_off(void)
{
	local_irq_disable();
	unreachable();
}

static struct mfd_cell sys_status_cell = {
	.name = "lantiq-rcu-sys-status"
};

static int mips_reboot_setup(struct platform_device *pdev)
{
	pr_warn("Using deprecated RCU driver\n");

	ltq_rcu_np = pdev->dev.of_node;

	rcu_regmap = syscon_node_to_regmap(ltq_rcu_np);
	mfd_add_devices(&pdev->dev, -1, &sys_status_cell, 1, NULL, 0, NULL);

	_machine_restart = ltq_machine_restart;
	_machine_halt = ltq_machine_halt;
	pm_power_off = ltq_machine_power_off;

	return 0;
}

static const struct of_device_id xway_reset_match[] = {
	{ .compatible = "lantiq,rcu-xway" },
	{ .compatible = "lantiq,rcu-xrx200" },
	{},
};
MODULE_DEVICE_TABLE(of, xway_reset_match);

static struct platform_driver xway_reset_driver = {
	.probe = mips_reboot_setup,
	.driver = {
		.name = "xway-reset",
		.of_match_table = xway_reset_match,
	},
};

int __init xway_reset_init(void)
{
	return platform_driver_register(&xway_reset_driver);
}

arch_initcall(xway_reset_init);
