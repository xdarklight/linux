/*
 * Amlogic SECBUS2 register access driver. The registers can be accessed
 * directly when not running in "secure mode". When "secure mode" is enabled
 * then these registers have to be accessed through secure monitor calls.
 * TODO: secure monitor operations are not implemented yet.
 *
 * Copyright (c) 2020 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/resource.h>

static const struct regmap_config meson_mx_secbus2_raw_regmap_conf = {
	.reg_bits = 16,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = 0x38,
};

static int meson_mx_secbus2_probe(struct platform_device *pdev)
{
	struct regmap *regmap;
	struct resource *res;
	void __iomem *base;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	regmap = devm_regmap_init_mmio(&pdev->dev, base,
				       &meson_mx_secbus2_raw_regmap_conf);
	return PTR_ERR_OR_ZERO(regmap);
}

static const struct of_device_id meson_mx_secbus2_match[] = {
	{ .compatible = "amlogic,meson-mx-secbus2" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, meson_mx_secbus2_match);

static struct platform_driver meson_mx_secbus2_driver = {
	.probe = meson_mx_secbus2_probe,
	.driver = {
		.name = "meson-mx-secbus2",
		.of_match_table = of_match_ptr(meson_mx_secbus2_match),
	},
};
module_platform_driver(meson_mx_secbus2_driver);

MODULE_DESCRIPTION("Amlogic Meson6/8/8b/8m2 SECBUS2 driver");
MODULE_AUTHOR("Martin Blumenstingl <martin.blumenstingl@googlemail.com>");
MODULE_LICENSE("GPL v2");
