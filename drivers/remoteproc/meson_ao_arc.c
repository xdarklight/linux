// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2020 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 */
#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/remoteproc.h>
#include <linux/reset.h>

#include "remoteproc_internal.h"

#define AO_RTI_STATUS_REG0				0x0
#define AO_RTI_STATUS_REG1				0x4
#define AO_RTI_STATUS_REG2				0xc

#define AO_REMAP_REG0					0x0
#define AO_REMAP_REG1					0x4

#define AO_CPU_CNTL					0x0
	#define AO_CPU_CNTL_RUN				BIT(0)
	#define AO_CPU_CNTL_UNKNONWN			BIT(8)
	#define AO_CPU_CNTL_HALT			BIT(9)
	#define AO_CPU_CNTL_MEM_ADDR_UPPER		GENMASK(31, 16)
#define AO_CPU_STAT					0x4

#define AO_SECURE_REG0					0x0
	#define AO_SECURE_REG0_MEM_ADDR_LOWER		GENMASK(23, 8)

struct meson_ao_arc_rproc_priv {
	void __iomem		*status_base;
	void __iomem		*remap_base;
	void __iomem		*cpu_base;
	void __iomem		*sram_base;
	struct reset_control	*arc625_reset;
	struct regmap		*secbus2_regmap;
};

static int meson_ao_arc_rproc_start(struct rproc *rproc)
{
	struct meson_ao_arc_rproc_priv *priv = rproc->priv;
	phys_addr_t phys_addr;
	int ret;

	phys_addr = rproc_va_to_pa(priv->sram_base);

	writel(0x0, priv->remap_base + AO_REMAP_REG0);
	usleep_range(10, 100);

	regmap_update_bits(priv->secbus2_regmap, AO_SECURE_REG0,
			   AO_SECURE_REG0_MEM_ADDR_LOWER,
			   FIELD_PREP(AO_SECURE_REG0_MEM_ADDR_LOWER,
				      phys_addr & 0xffff));

	/* clear the status register - TODO: move to mbox .startup()? */
	writel(0x0, priv->status_base + AO_RTI_STATUS_REG0);

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

static int meson_ao_arc_rproc_stop(struct rproc *rproc)
{
	struct meson_ao_arc_rproc_priv *priv = rproc->priv;

	writel(AO_CPU_CNTL_HALT, priv->cpu_base + AO_CPU_CNTL);

	return 0;
}

static int meson_ao_arc_rproc_load_raw(struct rproc *rproc,
				       const struct firmware *fw)
{
	struct meson_ao_arc_rproc_priv *priv = rproc->priv;

	memcpy_toio(priv->sram_base, fw->data, fw->size);

	return 0;
}

static struct rproc_ops meson_ao_arc_rproc_ops = {
	.start		= meson_ao_arc_rproc_start,
	.stop		= meson_ao_arc_rproc_stop,
	.load		= meson_ao_arc_rproc_load_raw,
};

static int meson_ao_arc_rproc_probe(struct platform_device *pdev)
{
	struct meson_ao_arc_rproc_priv *priv;
	struct platform_device *secbus2_pdev;
	struct device *dev = &pdev->dev;
	struct device_node *np;
	const char *fw_name;
	struct rproc *rproc;
	struct resource res;
	int ret;

	ret = of_property_read_string(dev->of_node, "firmware-name", &fw_name);
	if (ret)
		fw_name = NULL;

	rproc = rproc_alloc(dev, "meson-ao-arc", &meson_ao_arc_rproc_ops,
			    fw_name, sizeof(*priv));
	if (!rproc)
		return -ENOMEM;

	rproc->has_iommu = false;
	priv = rproc->priv;

	np = of_find_compatible_node(NULL, NULL,
				     "amlogic,meson-ao-arc-rproc-mem");
	ret = of_address_to_resource(np, 0, &res);
	of_node_put(np);

	if (ret)
		goto err_free_rproc;

	priv->sram_base = devm_ioremap_wc(dev, res.start, resource_size(&res));
	if (IS_ERR(priv->sram_base)) {
		ret = PTR_ERR(priv->sram_base);
		goto err_free_rproc;
	}

	np = of_find_compatible_node(NULL, NULL, "amlogic,meson-secbus2");
	secbus2_pdev = of_find_device_by_node(np);
	of_node_put(np);

	if (!secbus2_pdev) {
		ret = -ENODEV;
		goto err_free_rproc;
	}

	priv->secbus2_regmap = dev_get_regmap(&secbus2_pdev->dev, NULL);
	if (!priv->secbus2_regmap) {
		ret = -ENODEV;
		goto err_free_rproc;
	}

	priv->status_base = devm_platform_ioremap_resource_byname(pdev,
								  "status");
	if (IS_ERR(priv->status_base)) {
		ret = PTR_ERR(priv->status_base);
		goto err_free_rproc;
	}

	priv->remap_base = devm_platform_ioremap_resource_byname(pdev,
								 "remap");
	if (IS_ERR(priv->remap_base)) {
		ret = PTR_ERR(priv->remap_base);
		goto err_free_rproc;
	}

	priv->cpu_base = devm_platform_ioremap_resource_byname(pdev, "cpu");
	if (IS_ERR(priv->cpu_base)) {
		ret = PTR_ERR(priv->cpu_base);
		goto err_free_rproc;
	}

	priv->arc625_reset = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(priv->arc625_reset)) {
		ret = PTR_ERR(priv->arc625_reset);
		goto err_free_rproc;
	}

	platform_set_drvdata(pdev, rproc);

	ret = rproc_add(rproc);
	if (ret)
		goto err_free_rproc;

	return 0;

err_free_rproc:
	rproc_free(rproc);
	return ret;
}

static int meson_ao_arc_rproc_remove(struct platform_device *pdev)
{
	struct rproc *rproc = platform_get_drvdata(pdev);

	rproc_free(rproc);

	return 0;
}

static const struct of_device_id meson_ao_arc_rproc_match[] = {
	{ .compatible = "amlogic,meson-ao-arc-rproc" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, meson_ao_arc_rproc_match);

static struct platform_driver meson_ao_arc_rproc_driver = {
	.probe = meson_ao_arc_rproc_probe,
	.remove = meson_ao_arc_rproc_remove,
	.driver = {
		.name = "meson-ao-arc-rproc",
		.of_match_table = of_match_ptr(meson_ao_arc_rproc_match),
	},
};
module_platform_driver(meson_ao_arc_rproc_driver);

MODULE_DESCRIPTION("Amlogic Meson6/8/8b/8m2 AO ARC remote processor driver");
MODULE_AUTHOR("Martin Blumenstingl <martin.blumenstingl@googlemail.com>");
MODULE_LICENSE("GPL v2");
