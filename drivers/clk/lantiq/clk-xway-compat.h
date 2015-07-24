/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 * Copyright (C) 2015 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 */

struct device_node;

void ase_cgu_clocks_init_compat(struct device_node *np) __init;
void arx300_cgu_clocks_init_compat(struct device_node *np) __init;
void danube_cgu_clocks_init_compat(struct device_node *np) __init;
void grx300_cgu_clocks_init_compat(struct device_node *np) __init;
void xrx100_cgu_clocks_init_compat(struct device_node *np) __init;
void xrx200_cgu_clocks_init_compat(struct device_node *np) __init;

void arx300_pmu_clocks_init_compat(struct device_node *np) __init;
void ase_pmu_clocks_init_compat(struct device_node *np) __init;
void danube_pmu_clocks_init_compat(struct device_node *np) __init;
void grx300_pmu_clocks_init_compat(struct device_node *np) __init;
void xrx100_pmu_clocks_init_compat(struct device_node *np) __init;
void xrx200_pmu_clocks_init_compat(struct device_node *np) __init;
