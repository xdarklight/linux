/*
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 *
 *  Copyright (C) 2011-2015 John Crispin <blogic@openwrt.org>
 *  Copyright (C) 2015 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 */

#include <linux/ioport.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>

#include <lantiq_soc.h>

#define XBAR_ALWAYS_LAST	0x430
#define XBAR_FPI_BURST_EN	BIT(1)
#define XBAR_AHB_BURST_EN	BIT(2)

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

static int __init ltq_xbar_setup(void)
{
	struct resource res_xbar;
	struct device_node *np_xbar =
			of_find_compatible_node(NULL, NULL,
						"lantiq,xbar-xway");

	if (!of_machine_is_compatible("lantiq,vr9"))
		return 0;

	if (!np_xbar)
		panic("Failed to load xbar nodes from devicetree");
	if (of_address_to_resource(np_xbar, 0, &res_xbar))
		panic("Failed to get xbar resources");
	if (!request_mem_region(res_xbar.start, resource_size(&res_xbar),
		res_xbar.name))
		panic("Failed to get xbar resources");

	ltq_xbar_membase = ioremap_nocache(res_xbar.start,
						resource_size(&res_xbar));
	if (!ltq_xbar_membase)
		panic("Failed to remap xbar resources");

	xbar_fpi_burst_disable();

	return 0;
}

arch_initcall(ltq_xbar_setup);
