// SPDX-License-Identifier: GPL-2.0-only
/*
 *  Copyright (C) 2019 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 *  Copyright (C) 2011-2012 John Crispin <blogic@openwrt.org>
 */

#include <linux/bits.h>
#include <linux/export.h>
#include <linux/ioport.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>

#include <lantiq_soc.h>

#define LTQ_EBU_BUSCON0				0x0060
#define LTQ_EBU_BUSCON_WRDIS			BIT(31)

void __iomem *ltq_ebu_membase;

struct ltq_ebu_data {
	bool		initialize_buscon0_wrdis;
};

static const struct ltq_ebu_data ltq_ebu_falcon_data = {
	.initialize_buscon0_wrdis = false,
};

static const struct ltq_ebu_data ltq_ebu_xway_data = {
	.initialize_buscon0_wrdis = true,
};

static const struct of_device_id of_ebu_ids[] __initconst = {
	{
		/* DEPRECATED */
		.compatible = "lantiq,ebu-falcon",
		.data = &ltq_ebu_falcon_data,
	},
	{
		.compatible = "lantiq,falcon-ebu",
		.data = &ltq_ebu_falcon_data,
	},
	{
		/* DEPRECATED */
		.compatible = "lantiq,ebu-xway",
		.data = &ltq_ebu_xway_data,
	},
	{
		.compatible = "lantiq,ase-ebu",
		.data = &ltq_ebu_xway_data,
	},
	{
		.compatible = "lantiq,danube-ebu",
		.data = &ltq_ebu_xway_data,
	},
	{
		.compatible = "lantiq,xrx100-ebu",
		.data = &ltq_ebu_xway_data,
	},
	{
		.compatible = "lantiq,xrx200-ebu",
		.data = &ltq_ebu_xway_data,
	},
	{ /* sentinel */ },
};

static int __init ltq_ebu_setup(void)
{
	const struct ltq_ebu_data *ebu_data;
	const struct of_device_id *match;
	struct resource res_ebu;
	struct device_node *np;
	u32 val;

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

	ebu_data = match->data;

	if (ebu_data && ebu_data->initialize_buscon0_wrdis) {
		val = ltq_ebu_r32(LTQ_EBU_BUSCON0) & ~LTQ_EBU_BUSCON_WRDIS;
		ltq_ebu_w32(val, LTQ_EBU_BUSCON0);
	}

	return 0;
}

arch_initcall(ltq_ebu_setup);
