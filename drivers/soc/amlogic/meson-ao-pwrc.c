/*
 * Copyright (c) 2018 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 *
 * SPDX-License-Identifier: GPL-2.0+
 */

#include <dt-bindings/power/meson-ao-pwrc.h>
#include <linux/platform_device.h>
#include <linux/mfd/syscon.h>

#include "meson-regmap-pwrc.h"

#define AO_RTI_GEN_PWR_SLEEP0_MESON8	0x8
#define AO_RTI_GEN_PWR_ISO0_MESON8	0xc

#define AO_RTI_GEN_PWR_SLEEP0_GX	0xe8 /* 0x3a in the datasheet */
#define AO_RTI_GEN_PWR_ISO0_GX		0xec /* 0x3b in the datasheet */

#define MESON_AO_POWER_DOMAIN(_id_name, _reg, _lsb, _msb)	\
	[_id_name] = {						\
		.name = #_id_name,				\
		.reg_field = REG_FIELD((_reg), (_lsb), (_msb)),	\
	}

struct meson_ao_pwrc_data {
	const struct meson_regmap_pwrc_data regmap_pwrc_data;
	bool use_parent_syscon;
};

static int meson_ao_pwrc_probe(struct platform_device *pdev)
{
	const struct meson_ao_pwrc_data *pwrc_data;
	struct device_node *syscon_np;
	struct regmap *regmap;

	pwrc_data = device_get_match_data(&pdev->dev);
	if (!pwrc_data) {
		dev_err(&pdev->dev, "Missing match data\n");
		return -EINVAL;
	}

	if (pwrc_data->use_parent_syscon)
		syscon_np = of_get_parent(pdev->dev.of_node);
	else
		syscon_np = pdev->dev.of_node;

	regmap = syscon_node_to_regmap(syscon_np);
	if (IS_ERR(regmap)) {
		dev_err(&pdev->dev, "Failed to get AO regmap\n");
		return PTR_ERR(regmap);
	}

	return meson_regmap_pwrc_init(&pdev->dev, regmap,
				      &pwrc_data->regmap_pwrc_data);
}

static const struct meson_regmap_pwrc_domain meson_ao_pwrc_domains_gx[] = {
	MESON_AO_POWER_DOMAIN(AO_PWR_HCODEC_PD,
			      AO_RTI_GEN_PWR_SLEEP0_GX, 0, 1),
	MESON_AO_POWER_DOMAIN(AO_PWR_VDEC_1_PD,
			      AO_RTI_GEN_PWR_SLEEP0_GX, 2, 3),
	MESON_AO_POWER_DOMAIN(AO_PWR_VDEC_2_PD,
			      AO_RTI_GEN_PWR_SLEEP0_GX, 5, 6),
	MESON_AO_POWER_DOMAIN(AO_PWR_VDEC_HVEC_PD,
			      AO_RTI_GEN_PWR_SLEEP0_GX, 6, 7),
	MESON_AO_POWER_DOMAIN(AO_VPU_HDMI_ISO_PD,
			      AO_RTI_GEN_PWR_SLEEP0_GX, 8, 8),
	MESON_AO_POWER_DOMAIN(AO_VPU_HDMI_PD,
			      AO_RTI_GEN_PWR_SLEEP0_GX, 9, 9),
	MESON_AO_POWER_DOMAIN(AO_DEMOD_PD_COMB_PD,
			      AO_RTI_GEN_PWR_SLEEP0_GX, 10, 10),
	MESON_AO_POWER_DOMAIN(AO_DEMOD_PD,
			      AO_RTI_GEN_PWR_SLEEP0_GX, 12, 12),
	MESON_AO_POWER_DOMAIN(AO_WAVE420L_PD,
			      AO_RTI_GEN_PWR_SLEEP0_GX, 24, 25),
	MESON_AO_POWER_DOMAIN(AO_ISO_HCODEC_PD,
			      AO_RTI_GEN_PWR_ISO0_GX, 4, 5),
	MESON_AO_POWER_DOMAIN(AO_ISO_VDEC_1_PD,
			      AO_RTI_GEN_PWR_ISO0_GX, 6, 7),
	MESON_AO_POWER_DOMAIN(AO_ISO_VDEC_2_PD,
			      AO_RTI_GEN_PWR_ISO0_GX, 8, 9),
	MESON_AO_POWER_DOMAIN(AO_ISO_VDEC_HVEC_PD,
			      AO_RTI_GEN_PWR_ISO0_GX, 10, 11),
	MESON_AO_POWER_DOMAIN(AO_ISO_WAVE420L_PD,
			      AO_RTI_GEN_PWR_ISO0_GX, 12, 13),
	MESON_AO_POWER_DOMAIN(AO_ISO_DEMOD_PD,
			      AO_RTI_GEN_PWR_ISO0_GX, 14, 15),
};

static const struct meson_ao_pwrc_data meson_ao_pwrc_data_gx = {
	.regmap_pwrc_data = {
		.domains = meson_ao_pwrc_domains_gx,
		.num_domains = ARRAY_SIZE(meson_ao_pwrc_domains_gx),
	},
	.use_parent_syscon = true,
};

static const struct meson_regmap_pwrc_domain meson_ao_pwrc_domains_meson8[] = {
	MESON_AO_POWER_DOMAIN(AO_PWR_HCODEC_PD,
			      AO_RTI_GEN_PWR_SLEEP0_MESON8, 0, 1),
	MESON_AO_POWER_DOMAIN(AO_PWR_VDEC_1_PD,
			      AO_RTI_GEN_PWR_SLEEP0_MESON8, 2, 3),
	MESON_AO_POWER_DOMAIN(AO_PWR_VDEC_2_PD,
			      AO_RTI_GEN_PWR_SLEEP0_MESON8, 5, 6),
	MESON_AO_POWER_DOMAIN(AO_PWR_VDEC_HVEC_PD,
			      AO_RTI_GEN_PWR_SLEEP0_MESON8, 6, 7),
	MESON_AO_POWER_DOMAIN(AO_VPU_HDMI_ISO_PD,
			      AO_RTI_GEN_PWR_SLEEP0_MESON8, 8, 8),
	MESON_AO_POWER_DOMAIN(AO_VPU_HDMI_PD,
			      AO_RTI_GEN_PWR_SLEEP0_MESON8, 9, 9),
	MESON_AO_POWER_DOMAIN(AO_DEMOD_PD_COMB_PD,
			      AO_RTI_GEN_PWR_SLEEP0_MESON8, 10, 10),
	MESON_AO_POWER_DOMAIN(AO_DEMOD_PD,
			      AO_RTI_GEN_PWR_SLEEP0_MESON8, 12, 12),
	MESON_AO_POWER_DOMAIN(AO_ISO_HCODEC_PD,
			      AO_RTI_GEN_PWR_ISO0_MESON8, 4, 5),
	MESON_AO_POWER_DOMAIN(AO_ISO_VDEC_1_PD,
			      AO_RTI_GEN_PWR_ISO0_MESON8, 6, 7),
	MESON_AO_POWER_DOMAIN(AO_ISO_VDEC_2_PD,
			      AO_RTI_GEN_PWR_ISO0_MESON8, 8, 9),
	MESON_AO_POWER_DOMAIN(AO_ISO_VDEC_HVEC_PD,
			      AO_RTI_GEN_PWR_ISO0_MESON8, 10, 11),
	MESON_AO_POWER_DOMAIN(AO_ISO_DEMOD_PD,
			      AO_RTI_GEN_PWR_ISO0_MESON8, 14, 15),
};

static const struct meson_ao_pwrc_data meson_ao_pwrc_data_meson8 = {
	.regmap_pwrc_data = {
		.domains = meson_ao_pwrc_domains_meson8,
		.num_domains = ARRAY_SIZE(meson_ao_pwrc_domains_meson8),
	},
	.use_parent_syscon = false,
};

static const struct of_device_id meson_ao_pwrc_match_table[] = {
	{
		.compatible = "amlogic,meson8-pmu",
		.data = &meson_ao_pwrc_data_meson8,
	},
	{
		.compatible = "amlogic,meson8b-pmu",
		.data = &meson_ao_pwrc_data_meson8,
	},
	{
		.compatible = "amlogic,meson-gx-ao-pwrc",
		.data = &meson_ao_pwrc_data_gx,
	},
	{ /* sentinel */ }
};

static struct platform_driver meson_ao_pwrc_driver = {
	.probe	= meson_ao_pwrc_probe,
	.driver = {
		.name		= "meson_ao_pwrc",
		.of_match_table	= meson_ao_pwrc_match_table,
	},
};
builtin_platform_driver(meson_ao_pwrc_driver);
