/*
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 *
 *  Copyright (C) 2011-2015 John Crispin <blogic@openwrt.org>
 *  Copyright (C) 2015 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 */

#include <linux/ioport.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/regmap.h>

#include <lantiq_soc.h>

#define XBAR_ALWAYS_LAST	0x430
#define XBAR_FPI_BURST_EN	BIT(1)
#define XBAR_AHB_BURST_EN	BIT(2)

#define RCU_VR9_BE_AHB1S	0x00000008

#define xbar_w32(x, y)	ltq_w32((x), ltq_xbar_membase + (y))
#define xbar_r32(x)	ltq_r32(ltq_xbar_membase + (x))

static void __iomem *ltq_xbar_membase;

static void xbar_fpi_burst_disable(void)
{
	u32 reg;

	/* bit 1 as 1 --burst; bit 1 as 0 -- single */
	reg = xbar_r32(XBAR_ALWAYS_LAST);
	reg &= ~XBAR_FPI_BURST_EN;
	xbar_w32(reg, XBAR_ALWAYS_LAST);
}

static int ltq_xbar_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct resource res_xbar;
	struct regmap *rcu_regmap;
	u32 rcu_ahb_endianness_reg_offset;
	u32 rcu_ahb_endianness_val;

	if (of_address_to_resource(np, 0, &res_xbar))
		panic("Failed to get xbar resources");
	if (!request_mem_region(res_xbar.start, resource_size(&res_xbar),
		res_xbar.name))
		panic("Failed to get xbar resources");

	ltq_xbar_membase = ioremap_nocache(res_xbar.start,
						resource_size(&res_xbar));
	if (!ltq_xbar_membase)
		panic("Failed to remap xbar resources");

	/* RCU configuration is optional */
	rcu_regmap = syscon_regmap_lookup_by_phandle(np, "lantiq,rcu-syscon");
	if (!IS_ERR_OR_NULL(rcu_regmap)) {
		if (of_property_read_u32_index(np, "lantiq,rcu-syscon", 1,
			&rcu_ahb_endianness_reg_offset)) {
			dev_err(&pdev->dev, "Failed to get RCU reg offset\n");
			return -EINVAL;
		}

		if (of_device_is_big_endian(np))
			rcu_ahb_endianness_val = RCU_VR9_BE_AHB1S;
		else
			rcu_ahb_endianness_val = 0;

		if (regmap_update_bits(rcu_regmap,
					rcu_ahb_endianness_reg_offset,
					RCU_VR9_BE_AHB1S,
					rcu_ahb_endianness_val))
			dev_warn(&pdev->dev,
				"Failed to configure RCU AHB endianness\n");
	}

	xbar_fpi_burst_disable();

	return 0;
}

static const struct of_device_id xbar_match[] = {
	{ .compatible = "lantiq,xbar-xway" },
	{},
};
MODULE_DEVICE_TABLE(of, xbar_match);

static struct platform_driver xbar_driver = {
	.probe = ltq_xbar_probe,
	.driver = {
		.name = "xbar-xway",
		.of_match_table = xbar_match,
	},
};

int __init xbar_init(void)
{
	return platform_driver_register(&xbar_driver);
}

postcore_initcall(xbar_init);
