/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 * Copyright (C) 2015 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 */

#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/types.h>

#include "clk-xway-compat.h"

static void __init xway_cgu_compat_init_compat(struct device_node *np)
{
	pr_info("Please update your DTS to use the new CGU 'compatible' string\n");

	if (of_machine_is_compatible("lantiq,ase"))
		ase_cgu_clocks_init_compat(np);
	else if (of_machine_is_compatible("lantiq,grx390"))
		grx300_cgu_clocks_init_compat(np);
	else if (of_machine_is_compatible("lantiq,ar10"))
		arx300_cgu_clocks_init_compat(np);
	else if (of_machine_is_compatible("lantiq,vr9"))
		xrx200_cgu_clocks_init_compat(np);
	else if (of_machine_is_compatible("lantiq,ar9"))
		xrx100_cgu_clocks_init_compat(np);
	else if (of_machine_is_compatible("lantiq,gr9"))
		xrx100_cgu_clocks_init_compat(np);
	else if (of_machine_is_compatible("lantiq,danube"))
		danube_cgu_clocks_init_compat(np);
	else
		WARN(1, "Unknown of_machine in CGU compatibility code\n");
}

CLK_OF_DECLARE(cgu_xway_compat, "lantiq,cgu-xway", xway_cgu_compat_init_compat);

static void __init xway_compat_pmu_init_compat(struct device_node *np)
{
	pr_info("Please update your DTS to use the new PMU 'compatible' string\n");

	if (of_machine_is_compatible("lantiq,ase"))
		ase_pmu_clocks_init_compat(np);
	else if (of_machine_is_compatible("lantiq,grx390"))
		grx300_pmu_clocks_init_compat(np);
	else if (of_machine_is_compatible("lantiq,ar10"))
		arx300_pmu_clocks_init_compat(np);
	else if (of_machine_is_compatible("lantiq,vr9"))
		xrx200_pmu_clocks_init_compat(np);
	else if (of_machine_is_compatible("lantiq,ar9"))
		xrx100_pmu_clocks_init_compat(np);
	else if (of_machine_is_compatible("lantiq,gr9"))
		xrx100_pmu_clocks_init_compat(np);
	else if (of_machine_is_compatible("lantiq,danube"))
		danube_pmu_clocks_init_compat(np);
	else
		WARN(1, "Unknown of_machine in PMU compatibility code\n");
}

CLK_OF_DECLARE(pmu_xway_compat, "lantiq,pmu-xway", xway_compat_pmu_init_compat);
