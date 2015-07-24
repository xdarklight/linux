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

static void grx390_cgu_clocks_init_dt(struct device_node *np);
static void arx300_cgu_clocks_init_dt(struct device_node *np);
static void vrx200_cgu_clocks_init_dt(struct device_node *np);
static void arx100_cgu_clocks_init_dt(struct device_node *np);
static void danube_cgu_clocks_init_dt(struct device_node *np);
static void ase_pmu_clocks_init_dt(struct device_node *np);

static void grx390_pmu_clocks_init_dt(struct device_node *np);
static void arx300_pmu_clocks_init_dt(struct device_node *np);
static void vrx200_pmu_clocks_init_dt(struct device_node *np);
static void arx100_pmu_clocks_init_dt(struct device_node *np);
static void danube_pmu_clocks_init_dt(struct device_node *np);

static void __init xway_cgu_compat_init_dt(struct device_node *np)
{
	pr_info("Please update your DTS to use the new CGU 'compatible' string\n");

	if (of_machine_is_compatible("lantiq,ase"))
		ase_cgu_clocks_init_dt(np);
	else if (of_machine_is_compatible("lantiq,grx390"))
		grx390_cgu_clocks_init_dt(np);
	else if (of_machine_is_compatible("lantiq,ar10"))
		arx300_cgu_clocks_init_dt(np);
	else if (of_machine_is_compatible("lantiq,vr9"))
		vrx200_cgu_clocks_init_dt(np);
	else if (of_machine_is_compatible("lantiq,ar9"))
		arx100_cgu_clocks_init_dt(np);
	else if (of_machine_is_compatible("lantiq,danube"))
		danube_cgu_clocks_init_dt(np);
	else
		WARN(1, "Unknown of_machine in CGU compatibility code\n");
}

CLK_OF_DECLARE(cgu_xway_compat, "lantiq,cgu-xway", xway_cgu_compat_init_dt);

static void __init xway_compat_pmu_init_dt(struct device_node *np)
{
	pr_info("Please update your DTS to use the new PMU 'compatible' string\n");

	if (of_machine_is_compatible("lantiq,ase"))
		ase_pmu_clocks_init_dt(np);
	else if (of_machine_is_compatible("lantiq,grx390"))
		grx390_pmu_clocks_init_dt(np);
	else if (of_machine_is_compatible("lantiq,ar10"))
		arx300_pmu_clocks_init_dt(np);
	else if (of_machine_is_compatible("lantiq,vr9"))
		vrx200_pmu_clocks_init_dt(np);
	else if (of_machine_is_compatible("lantiq,ar9"))
		arx100_pmu_clocks_init_dt(np);
	else if (of_machine_is_compatible("lantiq,danube"))
		danube_pmu_clocks_init_dt(np);
	else
		WARN(1, "Unknown of_machine in PMU compatibility code\n");
}

CLK_OF_DECLARE(pmu_xway_compat, "lantiq,pmu-xway", xway_compat_pmu_init_dt);
