/*
 * Copyright (c) 2018 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 *
 * SPDX-License-Identifier: GPL-2.0+
 */

#include <dt-bindings/power/meson-hhi-mem-pwrc.h>
#include <linux/platform_device.h>
#include <linux/mfd/syscon.h>

#include "meson-regmap-pwrc.h"

#define HHI_MEM_PD_REG0			0x100 /* 0x40 in the datasheet */

#define MESON_HHI_MEMORY_PWRC_DOMAIN(_id_name, _lsb, _msb)		\
	[_id_name] = {							\
		.name = #_id_name,					\
		.reg_field = REG_FIELD(HHI_MEM_PD_REG0, (_lsb), (_msb)),\
	}

static int meson_hhi_mem_pwrc_probe(struct platform_device *pdev)
{
	const struct meson_regmap_pwrc_data *pwrc_data;
	struct regmap *regmap;

	pwrc_data = device_get_match_data(&pdev->dev);
	if (!pwrc_data) {
		dev_err(&pdev->dev, "Missing match data\n");
		return -EINVAL;
	}

	regmap = syscon_node_to_regmap(of_get_parent(pdev->dev.of_node));
	if (IS_ERR(regmap)) {
		dev_err(&pdev->dev, "Failed to get HHI syscon regmap\n");
		return PTR_ERR(regmap);
	}

	return meson_regmap_pwrc_init(&pdev->dev, regmap, pwrc_data);
}

static const struct meson_regmap_pwrc_domain meson_hhi_mem_pwrc_domains[] = {
	MESON_HHI_MEMORY_PWRC_DOMAIN(HHI_AUDIO_DSP_MEMORY_PD, 0, 1),
	MESON_HHI_MEMORY_PWRC_DOMAIN(HHI_ETHERNET_MEMORY_PD, 2, 3),
	MESON_HHI_MEMORY_PWRC_DOMAIN(HHI_HDMI_MEMORY_PD, 8, 15),
};

static const struct meson_regmap_pwrc_data meson_hhi_mem_pwrc_data = {
	.domains = meson_hhi_mem_pwrc_domains,
	.num_domains = ARRAY_SIZE(meson_hhi_mem_pwrc_domains),
};

static const struct of_device_id meson_hhi_mem_pwrc_match_table[] = {
	{
		.compatible = "amlogic,meson8-hhi-mem-pwrc",
		.data = &meson_hhi_mem_pwrc_data,
	},
	{
		.compatible = "amlogic,meson8b-hhi-mem-pwrc",
		.data = &meson_hhi_mem_pwrc_data,
	},
	{
		.compatible = "amlogic,meson-gx-hhi-mem-pwrc",
		.data = &meson_hhi_mem_pwrc_data,
	},
	{ /* sentinel */ }
};

static struct platform_driver meson_hhi_mem_pwrc_driver = {
	.probe	= meson_hhi_mem_pwrc_probe,
	.driver = {
		.name		= "meson_hhi_mem_pwrc",
		.of_match_table	= meson_hhi_mem_pwrc_match_table,
	},
};
builtin_platform_driver(meson_hhi_mem_pwrc_driver);
