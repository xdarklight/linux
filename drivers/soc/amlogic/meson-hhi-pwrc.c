/*
 * Copyright (c) 2018 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 *
 * SPDX-License-Identifier: GPL-2.0+
 */

#include <linux/platform_device.h>
#include <linux/mfd/syscon.h>

#include "meson-regmap-pwrc.h"

#define HHI_MEM_PD_REG0			0x100 /* 0x40 in the datasheet */
#define HHI_VPU_MEM_PD_REG0		0x104 /* 0x41 in the datasheet */
#define HHI_VPU_MEM_PD_REG1		0x108 /* 0x42 in the datasheet */
#define HHI_VPU_MEM_PD_REG2		0x10c /* 0x43 in the datasheet */

#define HHI_AUDIO_DSP_MEMORY_PD			0
#define HHI_ETHERNET_MEMORY_PD			1
#define HHI_HDMI_MEMORY_PD			2
#define HHI_VPU_VIU_OSD1_MEMORY_PD		3
#define HHI_VPU_VIU_OSD2_MEMORY_PD		4
#define HHI_VPU_VIU_VD1_MEMORY_PD		5
#define HHI_VPU_VIU_VD2_MEMORY_PD		6
#define HHI_VPU_VIU_CHROMA_PD			7
#define HHI_VPU_VIU_OUTPUT_FIFO_PD		8
#define HHI_VPU_VIU_SCALER_MEMORY_PD		9
#define HHI_VPU_VIU_OSD_SCALER_MEMORY_PD	10
#define HHI_VPU_VIU_VDIN0_MEMORY_PD		11
#define HHI_VPU_VIU_VDIN1_MEMORY_PD		12
#define HHI_VPU_PIC_ROT1_PD			13
#define HHI_VPU_PIC_ROT2_PD			14
#define HHI_VPU_PIC_ROT3_PD			15
#define HHI_VPU_DEINTERLACER_PRE_PD		16
#define HHI_VPU_DEINTERLACER_POST_PD		17
#define HHI_VPU_SHARP_PD			18
#define HHI_VPU_VIU2_OSD1_PD			19
#define HHI_VPU_VIU2_OSD2_PD			20
#define HHI_VPU_VIU2_VD1_PD			21
#define HHI_VPU_VIU2_CHROMA_PD			22
#define HHI_VPU_VIU2_OUTPUT_FIFO_PD		23
#define HHI_VPU_VIU2_SCALER_MEMORY_PD		24
#define HHI_VPU_VIU2_OSD_SCALER_MEMORY_PD	25
#define HHI_VPU_ARB_PD				26
#define HHI_AFBC_DEC_PD				27
#define HHI_VPU_VPUARB2_AM_ASYNC		28
#define HHI_VPU_VENCP				29
#define HHI_VPU_VENCL				30
#define HHI_VPU_VENCI				31
#define HHI_VPU_ISP_PD				32
#define HHI_VPU_LDIM_STTS_PD			33
#define HHI_VPU_XVYCC_LUT_PD			34
#define HHI_VPU_VIU1_WM_PD			35

#define MESON_HHI_PWRC_DOMAIN(_id_name, _reg, _lsb, _msb)	\
	[_id_name] = {						\
		.name = #_id_name,				\
		.reg_field = REG_FIELD((_reg), (_lsb), (_msb)),	\
	}

static int meson_hhi_pwrc_probe(struct platform_device *pdev)
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

static const struct meson_pwrc_domain_data meson_pwrc_domain_datas[] = {
	MESON_HHI_PWRC_DOMAIN(HHI_AUDIO_DSP_MEMORY_PD,
			      HHI_MEM_PD_REG0, 0, 1),
	MESON_HHI_PWRC_DOMAIN(HHI_ETHERNET_MEMORY_PD,
			      HHI_MEM_PD_REG0, 2, 3),
	MESON_HHI_PWRC_DOMAIN(HHI_HDMI_MEMORY_PD,
			      HHI_MEM_PD_REG0, 8, 15),
	MESON_HHI_PWRC_DOMAIN(HHI_VPU_VIU_OSD1_MEMORY_PD,
			      HHI_VPU_MEM_PD_REG0, 0, 1),
	MESON_HHI_PWRC_DOMAIN(HHI_VPU_VIU_OSD2_MEMORY_PD,
			      HHI_VPU_MEM_PD_REG0, 2, 3),
	MESON_HHI_PWRC_DOMAIN(HHI_VPU_VIU_VD1_MEMORY_PD,
			      HHI_VPU_MEM_PD_REG0, 4, 5),
	MESON_HHI_PWRC_DOMAIN(HHI_VPU_VIU_VD2_MEMORY_PD,
			      HHI_VPU_MEM_PD_REG0, 6, 7),
	MESON_HHI_PWRC_DOMAIN(HHI_VPU_VIU_CHROMA_PD,
			      HHI_VPU_MEM_PD_REG0, 8, 9),
	MESON_HHI_PWRC_DOMAIN(HHI_VPU_VIU_OUTPUT_FIFO_PD,
			      HHI_VPU_MEM_PD_REG0, 10, 11),
	MESON_HHI_PWRC_DOMAIN(HHI_VPU_VIU_SCALER_MEMORY_PD,
			      HHI_VPU_MEM_PD_REG0, 12, 13),
	MESON_HHI_PWRC_DOMAIN(HHI_VPU_VIU_OSD_SCALER_MEMORY_PD,
			      HHI_VPU_MEM_PD_REG0, 14, 15),
	MESON_HHI_PWRC_DOMAIN(HHI_VPU_VIU_VDIN0_MEMORY_PD,
			      HHI_VPU_MEM_PD_REG0, 16, 17),
	MESON_HHI_PWRC_DOMAIN(HHI_VPU_VIU_VDIN1_MEMORY_PD,
			      HHI_VPU_MEM_PD_REG0, 18, 19),
	MESON_HHI_PWRC_DOMAIN(HHI_VPU_PIC_ROT1_PD,
			      HHI_VPU_MEM_PD_REG0, 20, 21),
	MESON_HHI_PWRC_DOMAIN(HHI_VPU_PIC_ROT2_PD,
			      HHI_VPU_MEM_PD_REG0, 22, 23),
	MESON_HHI_PWRC_DOMAIN(HHI_VPU_PIC_ROT3_PD,
			      HHI_VPU_MEM_PD_REG0, 24, 25),
	MESON_HHI_PWRC_DOMAIN(HHI_VPU_DEINTERLACER_PRE_PD,
			      HHI_VPU_MEM_PD_REG0, 26, 27),
	MESON_HHI_PWRC_DOMAIN(HHI_VPU_DEINTERLACER_POST_PD,
			      HHI_VPU_MEM_PD_REG0, 28, 29),
	MESON_HHI_PWRC_DOMAIN(HHI_VPU_SHARP_PD,
			      HHI_VPU_MEM_PD_REG0, 30, 31),
	MESON_HHI_PWRC_DOMAIN(HHI_VPU_VIU2_OSD1_PD,
			      HHI_VPU_MEM_PD_REG1, 0, 1),
	MESON_HHI_PWRC_DOMAIN(HHI_VPU_VIU2_OSD2_PD,
			      HHI_VPU_MEM_PD_REG1, 2, 3),
	MESON_HHI_PWRC_DOMAIN(HHI_VPU_VIU2_VD1_PD,
			      HHI_VPU_MEM_PD_REG1, 4, 5),
	MESON_HHI_PWRC_DOMAIN(HHI_VPU_VIU2_CHROMA_PD,
			      HHI_VPU_MEM_PD_REG1, 6, 7),
	MESON_HHI_PWRC_DOMAIN(HHI_VPU_VIU2_OUTPUT_FIFO_PD,
			      HHI_VPU_MEM_PD_REG1, 8, 9),
	MESON_HHI_PWRC_DOMAIN(HHI_VPU_VIU2_SCALER_MEMORY_PD,
			      HHI_VPU_MEM_PD_REG1, 10, 11),
	MESON_HHI_PWRC_DOMAIN(HHI_VPU_VIU2_OSD_SCALER_MEMORY_PD,
			      HHI_VPU_MEM_PD_REG1, 12, 13),
	MESON_HHI_PWRC_DOMAIN(HHI_VPU_ARB_PD,
			      HHI_VPU_MEM_PD_REG1, 14, 15),
	MESON_HHI_PWRC_DOMAIN(HHI_AFBC_DEC_PD,
			      HHI_VPU_MEM_PD_REG1, 16, 17),
	MESON_HHI_PWRC_DOMAIN(HHI_VPU_VPUARB2_AM_ASYNC,
			      HHI_VPU_MEM_PD_REG1, 18, 19),
	MESON_HHI_PWRC_DOMAIN(HHI_VPU_VENCP,
			      HHI_VPU_MEM_PD_REG1, 20, 21),
	MESON_HHI_PWRC_DOMAIN(HHI_VPU_VENCL,
			      HHI_VPU_MEM_PD_REG1, 22, 23),
	MESON_HHI_PWRC_DOMAIN(HHI_VPU_VENCI,
			      HHI_VPU_MEM_PD_REG1, 24, 25),
	MESON_HHI_PWRC_DOMAIN(HHI_VPU_ISP_PD,
			      HHI_VPU_MEM_PD_REG1, 26, 27),
	MESON_HHI_PWRC_DOMAIN(HHI_VPU_LDIM_STTS_PD,
			      HHI_VPU_MEM_PD_REG1, 28, 29),
	MESON_HHI_PWRC_DOMAIN(HHI_VPU_XVYCC_LUT_PD,
			      HHI_VPU_MEM_PD_REG1, 30, 31),
	MESON_HHI_PWRC_DOMAIN(HHI_VPU_VIU1_WM_PD,
			      HHI_VPU_MEM_PD_REG2, 0, 1),
};

static const struct meson_regmap_pwrc_data meson_hhi_pwrc_data = {
	.domains = meson_pwrc_domain_datas,
	.num_domains = ARRAY_SIZE(meson_pwrc_domain_datas),
};

static const struct of_device_id meson_hhi_pwrc_match_table[] = {
	{
		.compatible = "amlogic,meson8-hhi-pwrc",
		.data = &meson_hhi_pwrc_data,
	},
	{
		.compatible = "amlogic,meson8b-hhi-pwrc",
		.data = &meson_hhi_pwrc_data,
	},
	{
		.compatible = "amlogic,meson-gx-hhi-pwrc",
		.data = &meson_hhi_pwrc_data,
	},
	{ /* sentinel */ }
};

static struct platform_driver meson_hhi_pwrc_driver = {
	.probe	= meson_hhi_pwrc_probe,
	.driver = {
		.name		= "meson_hhi_pwrc",
		.of_match_table	= meson_hhi_pwrc_match_table,
	},
};
builtin_platform_driver(meson_hhi_pwrc_driver);
