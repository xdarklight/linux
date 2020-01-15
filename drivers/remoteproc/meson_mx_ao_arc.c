// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2020 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 */

#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/genalloc.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/remoteproc.h>
#include <linux/reset.h>
#include <linux/sizes.h>

#include "remoteproc_internal.h"

#define AO_REMAP_REG0					0x0
#define AO_REMAP_REG1					0x4

#define AO_CPU_CNTL					0x0
	#define AO_CPU_CNTL_MEM_ADDR_UPPER		GENMASK(31, 16)
	#define AO_CPU_CNTL_HALT			BIT(9)
	#define AO_CPU_CNTL_UNKNONWN			BIT(8)
	#define AO_CPU_CNTL_RUN				BIT(0)

#define AO_CPU_STAT					0x4

#define AO_SECURE_REG0					0x0
	#define AO_SECURE_REG0_UNKNOWN			GENMASK(23, 8)

#define MESON_AO_RPROC_SRAM_SIZE			SZ_32K
#define MESON_AO_RPROC_SRAM_UNUSABLE_BITS		GENMASK(19, 0)
#define MESON_AO_RPROC_MEMORY_OFFSET			0x10000000

struct meson_mx_ao_arc_rproc_priv {
	void __iomem		*remap_base;
	void __iomem		*cpu_base;
	unsigned long		sram_va;
	phys_addr_t		sram_pa;
	struct gen_pool		*sram_pool;
	struct reset_control	*arc625_reset;
	struct regmap		*secbus2_regmap;
};

static int meson_mx_ao_arc_rproc_start(struct rproc *rproc)
{
	struct meson_mx_ao_arc_rproc_priv *priv = rproc->priv;
	phys_addr_t phys_addr;
	int ret;

	phys_addr = priv->sram_pa - MESON_AO_RPROC_MEMORY_OFFSET;

	writel(rproc->bootaddr, priv->remap_base + AO_REMAP_REG0);
	usleep_range(10, 100);

	regmap_update_bits(priv->secbus2_regmap, AO_SECURE_REG0,
			   AO_SECURE_REG0_UNKNOWN,
			   FIELD_PREP(AO_SECURE_REG0_UNKNOWN, 0));

	ret = reset_control_reset(priv->arc625_reset);
	if (ret)
		return ret;

	usleep_range(10, 100);

	writel(FIELD_PREP(AO_CPU_CNTL_MEM_ADDR_UPPER, phys_addr >> 20) |
	       AO_CPU_CNTL_UNKNONWN | AO_CPU_CNTL_RUN,
	       priv->cpu_base + AO_CPU_CNTL);
	usleep_range(20, 200);

	return 0;
}

static int meson_mx_ao_arc_rproc_stop(struct rproc *rproc)
{
	struct meson_mx_ao_arc_rproc_priv *priv = rproc->priv;

	writel(AO_CPU_CNTL_HALT, priv->cpu_base + AO_CPU_CNTL);

	return 0;
}

static void* meson_mx_ao_arc_rproc_da_to_va(struct rproc *rproc, u64 da,
					    size_t len)
{
	struct meson_mx_ao_arc_rproc_priv *priv = rproc->priv;

	if ((da + len) >= MESON_AO_RPROC_SRAM_SIZE)
		return NULL;

	return (void*)priv->sram_va + da;
}

static struct rproc_ops meson_mx_ao_arc_rproc_ops = {
	.start		= meson_mx_ao_arc_rproc_start,
	.stop		= meson_mx_ao_arc_rproc_stop,
	.da_to_va	= meson_mx_ao_arc_rproc_da_to_va,
	.get_boot_addr	= rproc_elf_get_boot_addr,
	.load		= rproc_elf_load_segments,
	.sanity_check	= rproc_elf_sanity_check,
};

static int meson_mx_ao_arc_rproc_probe(struct platform_device *pdev)
{
	struct meson_mx_ao_arc_rproc_priv *priv;
	struct platform_device *secbus2_pdev;
	struct device *dev = &pdev->dev;
	struct device_node *np;
	const char *fw_name;
	struct rproc *rproc;
	int ret;

	ret = of_property_read_string(dev->of_node, "firmware-name", &fw_name);
	if (ret)
		fw_name = NULL;

	rproc = rproc_alloc(dev, "meson-mx-ao-arc", &meson_mx_ao_arc_rproc_ops,
			    fw_name, sizeof(*priv));
	if (!rproc)
		return -ENOMEM;

	rproc->has_iommu = false;
	priv = rproc->priv;

	priv->sram_pool = of_gen_pool_get(dev->of_node, "amlogic,sram", 0);
	if (!priv->sram_pool) {
		dev_err(dev, "Could not get amlogic,sram pool\n");
		ret = -ENODEV;
		goto err_free_rproc;
	}

	priv->sram_va = gen_pool_alloc(priv->sram_pool,
				       MESON_AO_RPROC_SRAM_SIZE);
	if (!priv->sram_va) {
		dev_err(dev, "Could not alloc memory in SRAM pool\n");
		ret = -ENOMEM;
		goto err_free_rproc;
	}

	priv->sram_pa = gen_pool_virt_to_phys(priv->sram_pool, priv->sram_va);
	if (priv->sram_pa & MESON_AO_RPROC_SRAM_UNUSABLE_BITS) {
		dev_err(dev, "SRAM address contains unusable bits\n");
		ret = -EINVAL;
		goto err_free_genpool;
	}

	np = of_parse_phandle(dev->of_node, "amlogic,secbus2", 0);
	secbus2_pdev = of_find_device_by_node(np);
	of_node_put(np);

	if (!secbus2_pdev) {
		dev_err(dev, "Failed to find amlogic,secbus2 device\n");
		ret = -ENODEV;
		goto err_free_genpool;
	}

	priv->secbus2_regmap = dev_get_regmap(&secbus2_pdev->dev, NULL);
	if (!priv->secbus2_regmap) {
		dev_err(dev, "Failed to find SECBUS2 regmap\n");
		ret = -ENODEV;
		goto err_free_genpool;
	}

	priv->remap_base = devm_platform_ioremap_resource_byname(pdev,
								 "remap");
	if (IS_ERR(priv->remap_base)) {
		ret = PTR_ERR(priv->remap_base);
		goto err_free_genpool;
	}

	priv->cpu_base = devm_platform_ioremap_resource_byname(pdev, "cpu");
	if (IS_ERR(priv->cpu_base)) {
		ret = PTR_ERR(priv->cpu_base);
		goto err_free_genpool;
	}

	priv->arc625_reset = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(priv->arc625_reset)) {
		dev_err(dev, "Failed to get ARC625 reset\n");
		ret = PTR_ERR(priv->arc625_reset);
		goto err_free_genpool;
	}

	platform_set_drvdata(pdev, rproc);

	ret = rproc_add(rproc);
	if (ret)
		goto err_free_genpool;

	return 0;

err_free_genpool:
	gen_pool_free(priv->sram_pool, priv->sram_va,
		      MESON_AO_RPROC_SRAM_SIZE);
err_free_rproc:
	rproc_free(rproc);
	return ret;
}

static int meson_mx_ao_arc_rproc_remove(struct platform_device *pdev)
{
	struct rproc *rproc = platform_get_drvdata(pdev);
	struct meson_mx_ao_arc_rproc_priv *priv = rproc->priv;

	gen_pool_free(priv->sram_pool, priv->sram_va,
		      MESON_AO_RPROC_SRAM_SIZE);
	rproc_free(rproc);

	return 0;
}

static const struct of_device_id meson_mx_ao_arc_rproc_match[] = {
	{ .compatible = "amlogic,meson-mx-ao-arc-rproc" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, meson_mx_ao_arc_rproc_match);

static struct platform_driver meson_mx_ao_arc_rproc_driver = {
	.probe = meson_mx_ao_arc_rproc_probe,
	.remove = meson_mx_ao_arc_rproc_remove,
	.driver = {
		.name = "meson-mx-ao-arc-rproc",
		.of_match_table = of_match_ptr(meson_mx_ao_arc_rproc_match),
	},
};
module_platform_driver(meson_mx_ao_arc_rproc_driver);

MODULE_DESCRIPTION("Amlogic Meson6/8/8b/8m2 AO ARC remote processor driver");
MODULE_AUTHOR("Martin Blumenstingl <martin.blumenstingl@googlemail.com>");
MODULE_LICENSE("GPL v2");
