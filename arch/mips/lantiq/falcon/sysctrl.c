/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 * Copyright (C) 2011 Thomas Langer <thomas.langer@lantiq.com>
 * Copyright (C) 2011 John Crispin <blogic@openwrt.org>
 */

#include <linux/ioport.h>
#include <linux/export.h>
#include <linux/clkdev.h>
#include <linux/of_address.h>
#include <asm/delay.h>

#include <lantiq_soc.h>

void __iomem *ltq_ebu_membase;

void __init ltq_soc_init(void)
{
	struct resource res_ebu;
	struct device_node *np_ebu =
		of_find_compatible_node(NULL, NULL, "lantiq,ebu-falcon");

	if (!np_ebu)
		panic("Failed to load core nodes from devicetree");

	if (of_address_to_resource(np_ebu, 0, &res_ebu))
		panic("Failed to get core resources");

	if (request_mem_region(res_ebu.start, resource_size(&res_ebu),
				res_ebu.name) < 0)
		pr_err("Failed to request EBU resources");

	ltq_ebu_membase = ioremap_nocache(res_ebu.start,
					resource_size(&res_ebu));

	if (!ltq_ebu_membase)
		panic("Failed to remap EBU resources");
}
