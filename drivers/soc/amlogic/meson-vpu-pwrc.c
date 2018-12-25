/*
 * Copyright (c) 2018 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 *
 * SPDX-License-Identifier: GPL-2.0+
 */

#include <dt-bindings/power/meson-vpu-pwrc.h>
#include <linux/platform_device.h>
#include <linux/mfd/syscon.h>

#include "meson-regmap-pwrc.h"

#define HHI_VPU_MEM_PD_REG0		0x104 /* 0x41 in the datasheet */
#define HHI_VPU_MEM_PD_REG1		0x108 /* 0x42 in the datasheet */
#define HHI_VPU_MEM_PD_REG2		0x10c /* 0x43 in the datasheet */

#define MESON_VPU_PWRC_DOMAIN(_id_name, _reg, _lsb, _msb)	\
	[_id_name] = {						\
		.name = #_id_name,				\
		.reg_field = REG_FIELD((_reg), (_lsb), (_msb)),	\
	}

static int meson_vpu_pwrc_probe(struct platform_device *pdev)
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

static const struct meson_regmap_pwrc_domain meson_vpu_pwrc_domains[] = {
	MESON_VPU_PWRC_DOMAIN(HHI_VPU_VIU_OSD1_MEMORY_PD,
			      HHI_VPU_MEM_PD_REG0, 0, 1),
	MESON_VPU_PWRC_DOMAIN(HHI_VPU_VIU_OSD2_MEMORY_PD,
			      HHI_VPU_MEM_PD_REG0, 2, 3),
	MESON_VPU_PWRC_DOMAIN(HHI_VPU_VIU_VD1_MEMORY_PD,
			      HHI_VPU_MEM_PD_REG0, 4, 5),
	MESON_VPU_PWRC_DOMAIN(HHI_VPU_VIU_VD2_MEMORY_PD,
			      HHI_VPU_MEM_PD_REG0, 6, 7),
	MESON_VPU_PWRC_DOMAIN(HHI_VPU_VIU_CHROMA_PD,
			      HHI_VPU_MEM_PD_REG0, 8, 9),
	MESON_VPU_PWRC_DOMAIN(HHI_VPU_VIU_OUTPUT_FIFO_PD,
			      HHI_VPU_MEM_PD_REG0, 10, 11),
	MESON_VPU_PWRC_DOMAIN(HHI_VPU_VIU_SCALER_MEMORY_PD,
			      HHI_VPU_MEM_PD_REG0, 12, 13),
	MESON_VPU_PWRC_DOMAIN(HHI_VPU_VIU_OSD_SCALER_MEMORY_PD,
			      HHI_VPU_MEM_PD_REG0, 14, 15),
	MESON_VPU_PWRC_DOMAIN(HHI_VPU_VIU_VDIN0_MEMORY_PD,
			      HHI_VPU_MEM_PD_REG0, 16, 17),
	MESON_VPU_PWRC_DOMAIN(HHI_VPU_VIU_VDIN1_MEMORY_PD,
			      HHI_VPU_MEM_PD_REG0, 18, 19),
	MESON_VPU_PWRC_DOMAIN(HHI_VPU_PIC_ROT1_PD,
			      HHI_VPU_MEM_PD_REG0, 20, 21),
	MESON_VPU_PWRC_DOMAIN(HHI_VPU_PIC_ROT2_PD,
			      HHI_VPU_MEM_PD_REG0, 22, 23),
	MESON_VPU_PWRC_DOMAIN(HHI_VPU_PIC_ROT3_PD,
			      HHI_VPU_MEM_PD_REG0, 24, 25),
	MESON_VPU_PWRC_DOMAIN(HHI_VPU_DEINTERLACER_PRE_PD,
			      HHI_VPU_MEM_PD_REG0, 26, 27),
	MESON_VPU_PWRC_DOMAIN(HHI_VPU_DEINTERLACER_POST_PD,
			      HHI_VPU_MEM_PD_REG0, 28, 29),
	MESON_VPU_PWRC_DOMAIN(HHI_VPU_SHARP_PD,
			      HHI_VPU_MEM_PD_REG0, 30, 31),
	MESON_VPU_PWRC_DOMAIN(HHI_VPU_VIU2_OSD1_PD,
			      HHI_VPU_MEM_PD_REG1, 0, 1),
	MESON_VPU_PWRC_DOMAIN(HHI_VPU_VIU2_OSD2_PD,
			      HHI_VPU_MEM_PD_REG1, 2, 3),
	MESON_VPU_PWRC_DOMAIN(HHI_VPU_VIU2_VD1_PD,
			      HHI_VPU_MEM_PD_REG1, 4, 5),
	MESON_VPU_PWRC_DOMAIN(HHI_VPU_VIU2_CHROMA_PD,
			      HHI_VPU_MEM_PD_REG1, 6, 7),
	MESON_VPU_PWRC_DOMAIN(HHI_VPU_VIU2_OUTPUT_FIFO_PD,
			      HHI_VPU_MEM_PD_REG1, 8, 9),
	MESON_VPU_PWRC_DOMAIN(HHI_VPU_VIU2_SCALER_MEMORY_PD,
			      HHI_VPU_MEM_PD_REG1, 10, 11),
	MESON_VPU_PWRC_DOMAIN(HHI_VPU_VIU2_OSD_SCALER_MEMORY_PD,
			      HHI_VPU_MEM_PD_REG1, 12, 13),
	MESON_VPU_PWRC_DOMAIN(HHI_VPU_ARB_PD,
			      HHI_VPU_MEM_PD_REG1, 14, 15),
	MESON_VPU_PWRC_DOMAIN(HHI_AFBC_DEC_PD,
			      HHI_VPU_MEM_PD_REG1, 16, 17),
	MESON_VPU_PWRC_DOMAIN(HHI_VPU_VPUARB2_AM_ASYNC,
			      HHI_VPU_MEM_PD_REG1, 18, 19),
	MESON_VPU_PWRC_DOMAIN(HHI_VPU_VENCP,
			      HHI_VPU_MEM_PD_REG1, 20, 21),
	MESON_VPU_PWRC_DOMAIN(HHI_VPU_VENCL,
			      HHI_VPU_MEM_PD_REG1, 22, 23),
	MESON_VPU_PWRC_DOMAIN(HHI_VPU_VENCI,
			      HHI_VPU_MEM_PD_REG1, 24, 25),
	MESON_VPU_PWRC_DOMAIN(HHI_VPU_ISP_PD,
			      HHI_VPU_MEM_PD_REG1, 26, 27),
	MESON_VPU_PWRC_DOMAIN(HHI_VPU_LDIM_STTS_PD,
			      HHI_VPU_MEM_PD_REG1, 28, 29),
	MESON_VPU_PWRC_DOMAIN(HHI_VPU_XVYCC_LUT_PD,
			      HHI_VPU_MEM_PD_REG1, 30, 31),
	MESON_VPU_PWRC_DOMAIN(HHI_VPU_VIU1_WM_PD,
			      HHI_VPU_MEM_PD_REG2, 0, 1),
};

static const struct meson_regmap_pwrc_data meson_vpu_pwrc_data = {
	.domains = meson_vpu_pwrc_domains,
	.num_domains = ARRAY_SIZE(meson_vpu_pwrc_domains),
};

static const struct of_device_id meson_vpu_pwrc_match_table[] = {
	{
		.compatible = "amlogic,meson8-vpu-pwrc",
		.data = &meson_vpu_pwrc_data,
	},
	{
		.compatible = "amlogic,meson8b-vpu-pwrc",
		.data = &meson_vpu_pwrc_data,
	},
	{
		.compatible = "amlogic,meson-gxbb-vpu-pwrc",
		.data = &meson_vpu_pwrc_data,
	},
	{
		.compatible = "amlogic,meson-gx-vpu-pwrc",
		.data = &meson_vpu_pwrc_data,
	},
	{ /* sentinel */ }
};

static struct platform_driver meson_vpu_pwrc_driver = {
	.probe	= meson_vpu_pwrc_probe,
	.driver = {
		.name		= "meson_vpu_pwrc",
		.of_match_table	= meson_vpu_pwrc_match_table,
	},
};
builtin_platform_driver(meson_vpu_pwrc_driver);
