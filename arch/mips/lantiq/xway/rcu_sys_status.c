/*
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 *
 *  Copyright (C) 2010 John Crispin <blogic@openwrt.org>
 *  Copyright (C) 2013-2015 Lantiq Beteiligungs-GmbH & Co.KG
 *  Copyright (C) 2016 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 */

#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/of_platform.h>

/* reset cause */
#define RCU_STAT_SHIFT		26
/* boot selection */
#define RCU_BOOT_SEL(x)		
#define RCU_BOOT_SEL_XRX200(x)	

static struct regmap *rcu_regmap;
static u32 status_reg_offset;
unsigned char (*ltq_get_boot_select_func)(u32);

unsigned char ltq_get_boot_select_xway(u32 val)
{
	return (val >> 18) & 0x7;
}

unsigned char ltq_get_boot_select_xrx(u32 val)
{
	return ((val >> 17) & 0xf) | ((val >> 8) & 0x10);
}

static const struct of_device_id lantiq_rcu_sys_status_dt_ids[] = {
	{
		.compatible = "lantiq,ase-rcu-sys-status",
		.data = &ltq_get_boot_select_xway,
	},
	{
		.compatible = "lantiq,danube-rcu-sys-status",
		.data = &ltq_get_boot_select_xway,
	},
	{
		.compatible = "lantiq,xrx100-rcu-sys-status",
		.data = &ltq_get_boot_select_xway,
	},
	{
		.compatible = "lantiq,xrx200-rcu-sys-status",
		.data = &ltq_get_boot_select_xrx,
	},
	{
		.compatible = "lantiq,xrx300-rcu-sys-status",
		.data = &ltq_get_boot_select_xrx,
	},
	{ },
};
MODULE_DEVICE_TABLE(of, lantiq_rcu_sys_status_dt_ids);

static u32 ltq_rcu_sys_status(u32 *val)
{
	return regmap_read(rcu_regmap, status_reg_offset, val);
}

/* This function is used by the watchdog driver */
int ltq_reset_cause(void)
{
	u32 val = 0;

	if (ltq_rcu_sys_status(&val))
		pr_err("%s: Failed to read RCU status\n", __func__);

	return val >> RCU_STAT_SHIFT;
}
EXPORT_SYMBOL_GPL(ltq_reset_cause);

/* allow platform code to find out what source we booted from */
unsigned char ltq_boot_select(void)
{
	u32 val = 0;

	if (ltq_rcu_sys_status(&val))
		pr_err("%s: Failed to read RCU status\n", __func__);

	return ltq_get_boot_select_func(val);
}

static int lantiq_rcu_status_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	const struct of_device_id *match =
		of_match_node(lantiq_rcu_sys_status_dt_ids, np);

	/* backwards compatibility code: */
	if (!match) {
		/*
		 * To ensure we're compatible with old .dts we look up the
		 * regmap by taking the of_node of our parent (= "xway-reset"
		 * in case of an old "lantiq,rcu-*" entry).
		 */
		rcu_regmap = syscon_node_to_regmap(pdev->dev.parent->of_node);

		if (IS_ERR_OR_NULL(rcu_regmap)) {
			dev_err(&pdev->dev, "Failed to lookup RCU regmap\n");
			return PTR_ERR(rcu_regmap);
		}

		/* Default to RCU_RST_STAT: */
		status_reg_offset = 0x14;

		if (of_machine_is_compatible("lantiq,vr9"))
			ltq_get_boot_select_func = ltq_get_boot_select_xrx;
		else
			ltq_get_boot_select_func = ltq_get_boot_select_xway;

		/* backwards compatibility code ends here: */
		return 0;
	}

	ltq_get_boot_select_func = match->data;

	rcu_regmap = syscon_regmap_lookup_by_phandle(np,
						     "lantiq,rcu-syscon");
	if (IS_ERR_OR_NULL(rcu_regmap)) {
		dev_err(&pdev->dev, "Failed to lookup RCU regmap\n");
		return PTR_ERR(rcu_regmap);
	}

	if (of_property_read_u32_index(np, "lantiq,rcu-syscon", 1,
		&status_reg_offset)) {
		dev_err(&pdev->dev, "Failed to get RCU reg offset\n");
		return -EINVAL;
	}

	return 0;
}

static struct platform_driver lantiq_rcu_sys_status_driver = {
	.probe	= lantiq_rcu_status_probe,
	.driver = {
		.name		= "lantiq-rcu-sys-status",
		.of_match_table	= lantiq_rcu_sys_status_dt_ids,
	},
};
module_platform_driver(lantiq_rcu_sys_status_driver);

MODULE_AUTHOR("Martin Blumenstingl <martin.blumenstingl@googlemail.com>");
MODULE_DESCRIPTION("Lantiq XWAY RCU System Status Driver");
MODULE_LICENSE("GPL");
