/*
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 *
 *  Copyright (C) 2011-2012 John Crispin <blogic@openwrt.org>
 *  Copyright (C) 2013-2015 Lantiq Beteiligungs-GmbH & Co.KG
 */

#include <linux/ioport.h>
#include <linux/export.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>

#include <lantiq_soc.h>

void __iomem *ltq_ebu_membase;

void __init ltq_soc_init(void)
{
	struct resource res_ebu;
	struct device_node *np_ebu =
			of_find_compatible_node(NULL, NULL, "lantiq,ebu-xway");

	if (!np_ebu)
		panic("Failed to load the EBU node from devicetree");

	if (of_address_to_resource(np_ebu, 0, &res_ebu))
		panic("Failed to get the EBU resources");

	ltq_ebu_membase = ioremap_nocache(res_ebu.start,
						resource_size(&res_ebu));
	if (!ltq_ebu_membase)
		panic("Failed to remap the EBU resources");

	/* make sure to unprotect the memory region where flash is located */
	ltq_ebu_w32(ltq_ebu_r32(LTQ_EBU_BUSCON0) & ~EBU_WRDIS, LTQ_EBU_BUSCON0);
}
