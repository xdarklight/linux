/*
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 *
 *  Copyright (C) 2015 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 *  Copyright (C) 2011-2012 John Crispin <blogic@openwrt.org>
 */

#include <linux/ioport.h>
#include <linux/export.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>

#include <lantiq_soc.h>

void __iomem *ltq_ebu_membase;

struct ebu_init_sequence {
	u32	reg;
	u32	mask;
};

static const struct of_device_id of_ebu_ids[] __initconst = {
	{
		.compatible = "lantiq,ebu-falcon",
		.data = NULL
	},
	{
		.compatible = "lantiq,ebu-xway",
		.data = &(const struct ebu_init_sequence) {
			/* Unprotect the memory region where flash is located. */
			LTQ_EBU_BUSCON0, ~EBU_WRDIS
		},
	},
	{},
};

static int __init ltq_ebu_setup(void)
{
	struct resource res_ebu;
	struct device_node *np;
	const struct of_device_id *match;
	const struct ebu_init_sequence *init_seq;

	np = of_find_matching_node_and_match(NULL, of_ebu_ids, &match);

	if (!np)
		panic("Failed to load the EBU node from devicetree");

	if (of_address_to_resource(np, 0, &res_ebu))
		panic("Failed to get the EBU resources");

	if ((request_mem_region(res_ebu.start, resource_size(&res_ebu),
				res_ebu.name) < 0))
		pr_err("Failed to request the EBU resources");

	ltq_ebu_membase = ioremap_nocache(res_ebu.start,
						resource_size(&res_ebu));
	if (!ltq_ebu_membase)
		panic("Failed to remap the EBU resources");

	init_seq = match->data;

	if (init_seq) {
		ltq_ebu_w32(ltq_ebu_r32(init_seq->reg) & init_seq->mask,
			    init_seq->reg);
	}

	return 0;
}

arch_initcall(ltq_ebu_setup);
