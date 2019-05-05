// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2020 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 *
 * All registers and magic values are taken from Amlogic's GPL kernel sources:
 *   Copyright (C) 2010 Amlogic, Inc.
 */

#include <linux/clk.h>
#include <linux/component.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/hdmi.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_device.h>
#include <linux/phy/phy.h>
#include <linux/regmap.h>

#include <media/cec-notifier.h>

#include <uapi/linux/media-bus-format.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_edid.h>
#include <drm/drm_of.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>

#include "meson_drv.h"
#include "meson_mx_hdmi.h"
#include "meson_vclk.h"
#include "meson_venc.h"

#define HDMI_ADDR_PORT					0x0
#define HDMI_DATA_PORT					0x4
#define HDMI_CTRL_PORT					0x8
	#define HDMI_CTRL_PORT_APB3_ERR_EN		BIT(15)

struct meson_mx_hdmi {
	struct device			*dev;
	struct drm_device		*drm;
	struct regmap			*regmap;
	struct clk			*pclk;
	struct clk			*hdmi_clk;
	struct phy			*phy;
	struct cec_notifier		*cec_notifier;
	struct drm_connector		connector;
	struct drm_encoder		encoder;
	int				cea_mode;
	enum hdmi_colorimetry		colorimetry;
	unsigned int			input_bus_format;
	unsigned int			output_bus_format;
	bool				limited_rgb_quant_range;
	bool				sink_is_hdmi;
	bool				sink_has_audio;
	bool				phy_is_on;
};

#define to_meson_mx_hdmi(x) container_of(x, struct meson_mx_hdmi, x)

static int meson_mx_hdmi_reg_read(void *context, unsigned int addr,
				  unsigned int *data)
{
	void __iomem *base = context;

	writel(addr, base + HDMI_ADDR_PORT);
	writel(addr, base + HDMI_ADDR_PORT);

	*data = readl(base + HDMI_DATA_PORT);

	return 0;
}

static int meson_mx_hdmi_reg_write(void *context, unsigned int addr,
				   unsigned int data)
{
	void __iomem *base = context;

	writel(addr, base + HDMI_ADDR_PORT);
	writel(addr, base + HDMI_ADDR_PORT);

	writel(data, base + HDMI_DATA_PORT);

	return 0;
}

static const struct regmap_range meson_mx_hdmi_regmap_ranges[] = {
	regmap_reg_range(0x0000, 0x07ff),
	regmap_reg_range(HDMI_OTHER_CTRL0, HDMI_OTHER_INTR_STAT_CLR),
	regmap_reg_range(HDMI_OTHER_AVI_INTR_MASKN0,
			 HDMI_OTHER_RX_PACKET_INTR_CLR),
};

static const struct regmap_access_table meson_mx_hdmi_regmap_access_table = {
	.yes_ranges = meson_mx_hdmi_regmap_ranges,
	.n_yes_ranges = ARRAY_SIZE(meson_mx_hdmi_regmap_ranges),
};

static const struct regmap_config meson_mx_hdmi_regmap_config = {
	.reg_bits = 16,
	.val_bits = 16,
	.reg_stride = 1,
	.reg_read = meson_mx_hdmi_reg_read,
	.reg_write = meson_mx_hdmi_reg_write,
	.rd_table = &meson_mx_hdmi_regmap_access_table,
	.wr_table = &meson_mx_hdmi_regmap_access_table,
	.max_register = 0xffff,
	.fast_io = true,
};

static void meson_mx_hdmi_write_infoframe(struct meson_mx_hdmi *hdmi,
					  unsigned int tx_pkt_reg,
					  u8 *buf, unsigned int len,
					  bool enable)
{
	unsigned int i;

	/* write the payload starting register offset 1 and skip the header */
	for (i = HDMI_INFOFRAME_HEADER_SIZE; i < len; i++)
		regmap_write(hdmi->regmap,
			     tx_pkt_reg + i - HDMI_INFOFRAME_HEADER_SIZE + 1,
			     buf[i]);

	/* zero the remaining payload bytes */
	for (; i < 0x1c; i++)
		regmap_write(hdmi->regmap, tx_pkt_reg + i, 0x00);

	/* write the header */
	regmap_write(hdmi->regmap, tx_pkt_reg + 0x00, buf[3]);
	regmap_write(hdmi->regmap, tx_pkt_reg + 0x1c, buf[0]);
	regmap_write(hdmi->regmap, tx_pkt_reg + 0x1d, buf[1]);
	regmap_write(hdmi->regmap, tx_pkt_reg + 0x1e, buf[2]);

	regmap_write(hdmi->regmap, tx_pkt_reg + 0x1f, enable ? 0xff : 0x00);
}

static void meson_mx_hdmi_disable_infoframe(struct meson_mx_hdmi *hdmi,
					    unsigned int tx_pkt_reg)
{
	u8 buf[HDMI_INFOFRAME_HEADER_SIZE] = { 0 };

	meson_mx_hdmi_write_infoframe(hdmi, tx_pkt_reg, buf,
				     HDMI_INFOFRAME_HEADER_SIZE, false);
}

static void meson_mx_hdmi_encoder_disable(struct drm_encoder *encoder)
{
	struct meson_mx_hdmi *hdmi = to_meson_mx_hdmi(encoder);
	struct meson_drm *priv = hdmi->drm->dev_private;
	int ret;

	meson_venc_hdmi_encoder_disable(priv);

	meson_mx_hdmi_disable_infoframe(hdmi, TX_PKT_REG_AUDIO_INFO_BASE_ADDR);
	meson_mx_hdmi_disable_infoframe(hdmi, TX_PKT_REG_AVI_INFO_BASE_ADDR);
	meson_mx_hdmi_disable_infoframe(hdmi, TX_PKT_REG_EXCEPT0_BASE_ADDR);
	meson_mx_hdmi_disable_infoframe(hdmi, TX_PKT_REG_VEND_INFO_BASE_ADDR);

	regmap_update_bits(hdmi->regmap, HDMI_OTHER_CTRL1,
			   HDMI_OTHER_CTRL1_POWER_ON, 0);

	// TODO: assert resets?

	if (hdmi->phy_is_on) {
		ret = phy_power_off(hdmi->phy);
		if (ret)
			drm_err(hdmi->drm, "Failed to turn off PHY\n");
		else
			hdmi->phy_is_on = false;
	}
}

static void meson_mx_hdmi_sys5_reset_assert(struct meson_mx_hdmi *hdmi)
{
	// TODO: "bit5,6 is converted" - what does this mean ???
	regmap_write(hdmi->regmap, TX_SYS5_TX_SOFT_RESET_2,
		     TX_SYS5_TX_SOFT_RESET_2_HDMI_CH3_RST_IN |
		     TX_SYS5_TX_SOFT_RESET_2_HDMI_CH0_RST_IN);
	usleep_range(10, 20);

	regmap_write(hdmi->regmap, TX_SYS5_TX_SOFT_RESET_2,
		     TX_SYS5_TX_SOFT_RESET_2_HDMI_CH2_RST_IN |
		     TX_SYS5_TX_SOFT_RESET_2_HDMI_CH1_RST_IN);
	usleep_range(10, 20);

	regmap_write(hdmi->regmap, TX_SYS5_TX_SOFT_RESET_1,
		     TX_SYS5_TX_SOFT_RESET_1_TX_PIXEL_RSTN |
		     TX_SYS5_TX_SOFT_RESET_1_TX_TMDS_RSTN |
		     TX_SYS5_TX_SOFT_RESET_1_TX_AUDIO_MASTER_RSTN |
		     TX_SYS5_TX_SOFT_RESET_1_TX_AUDIO_RESAMPLE_RSTN |
		     TX_SYS5_TX_SOFT_RESET_1_TX_I2S_RESET_RSTN |
		     TX_SYS5_TX_SOFT_RESET_1_TX_DIG_RESET_N_CH2 |
		     TX_SYS5_TX_SOFT_RESET_1_TX_DIG_RESET_N_CH1 |
		     TX_SYS5_TX_SOFT_RESET_1_TX_DIG_RESET_N_CH0);
	usleep_range(10, 20);
}

static void meson_mx_hdmi_sys5_reset_deassert(struct meson_mx_hdmi *hdmi)
{
	/* Release the resets except tmds_clk */
	regmap_write(hdmi->regmap, TX_SYS5_TX_SOFT_RESET_1,
		     TX_SYS5_TX_SOFT_RESET_1_TX_TMDS_RSTN);
	usleep_range(10, 20);

	/* Release the tmds_clk reset as well */
	regmap_write(hdmi->regmap, TX_SYS5_TX_SOFT_RESET_1, 0x0);
	usleep_range(10, 20);

	regmap_write(hdmi->regmap, TX_SYS5_TX_SOFT_RESET_2,
		     TX_SYS5_TX_SOFT_RESET_2_HDMI_CH2_RST_IN |
		     TX_SYS5_TX_SOFT_RESET_2_HDMI_CH1_RST_IN |
		     TX_SYS5_TX_SOFT_RESET_2_HDMI_SR_RST);
	usleep_range(10, 20);

	regmap_write(hdmi->regmap, TX_SYS5_TX_SOFT_RESET_2,
		     TX_SYS5_TX_SOFT_RESET_2_HDMI_CH2_RST_IN |
		     TX_SYS5_TX_SOFT_RESET_2_HDMI_CH1_RST_IN);
	usleep_range(10, 20);
}

static void meson_mx_hdmi_config_hdcp_registers(struct meson_mx_hdmi *hdmi)
{
	regmap_write(hdmi->regmap, TX_HDCP_CONFIG0,
		     FIELD_PREP(TX_HDCP_CONFIG0_ROM_ENCRYPT_OFF, 0x3));
	regmap_write(hdmi->regmap, TX_HDCP_MEM_CONFIG, 0x0);
	regmap_write(hdmi->regmap, TX_HDCP_ENCRYPT_BYTE, 0x0);

	regmap_write(hdmi->regmap, TX_HDCP_MODE, TX_HDCP_MODE_CLEAR_AVMUTE);

	regmap_write(hdmi->regmap, TX_HDCP_MODE, TX_HDCP_MODE_ESS_CONFIG);
}

static u8 meson_mx_hdmi_bus_fmt_color_depth(unsigned int bus_format)
{
	switch (bus_format) {
	case MEDIA_BUS_FMT_RGB888_1X24:
	case MEDIA_BUS_FMT_YUV8_1X24:
	case MEDIA_BUS_FMT_UYVY8_1X16:
		/* 8 bit */
		return 0x0;

	case MEDIA_BUS_FMT_RGB101010_1X30:
	case MEDIA_BUS_FMT_YUV10_1X30:
	case MEDIA_BUS_FMT_UYVY10_1X20:
		/* 10 bit */
		return 0x1;

	case MEDIA_BUS_FMT_RGB121212_1X36:
	case MEDIA_BUS_FMT_YUV12_1X36:
	case MEDIA_BUS_FMT_UYVY12_1X24:
		/* 12 bit */
		return 0x2;

	case MEDIA_BUS_FMT_RGB161616_1X48:
	case MEDIA_BUS_FMT_YUV16_1X48:
		/* 16 bit */
		return 0x3;

	default:
		/* unknown, default to 8 bit */
		return 0x0;
	}
}

static enum hdmi_colorspace
meson_mx_hdmi_bus_fmt_hdmi_colorspace(unsigned int bus_format)
{
	switch (bus_format) {
	case MEDIA_BUS_FMT_YUV8_1X24:
	case MEDIA_BUS_FMT_YUV10_1X30:
	case MEDIA_BUS_FMT_YUV12_1X36:
	case MEDIA_BUS_FMT_YUV16_1X48:
		return HDMI_COLORSPACE_YUV444;

	case MEDIA_BUS_FMT_UYVY8_1X16:
	case MEDIA_BUS_FMT_UYVY10_1X20:
	case MEDIA_BUS_FMT_UYVY12_1X24:
		return HDMI_COLORSPACE_YUV422;

	case MEDIA_BUS_FMT_RGB888_1X24:
	case MEDIA_BUS_FMT_RGB101010_1X30:
	case MEDIA_BUS_FMT_RGB121212_1X36:
	case MEDIA_BUS_FMT_RGB161616_1X48:
	default:
		return HDMI_COLORSPACE_RGB;

	}
}

static u8 meson_mx_hdmi_bus_fmt_color_format(unsigned int bus_format)
{
	switch (meson_mx_hdmi_bus_fmt_hdmi_colorspace(bus_format)) {
	case HDMI_COLORSPACE_YUV422:
		/* Documented as YCbCr422 */
		return 0x3;

	case HDMI_COLORSPACE_YUV444:
		/* Documented as YCbCr444 */
		return 0x1;

	case HDMI_COLORSPACE_RGB:
	default:
		/* Documented as RGB444 */
		return 0x0;
	}
}

static void meson_mx_hdmi_config_color_space(struct meson_mx_hdmi *hdmi)
{
	regmap_write(hdmi->regmap, TX_VIDEO_DTV_MODE,
		     FIELD_PREP(TX_VIDEO_DTV_MODE_COLOR_DEPTH,
				meson_mx_hdmi_bus_fmt_color_depth(
					hdmi->output_bus_format)));

	regmap_write(hdmi->regmap, TX_VIDEO_DTV_OPTION_L,
		     FIELD_PREP(TX_VIDEO_DTV_OPTION_L_OUTPUT_COLOR_FORMAT,
				meson_mx_hdmi_bus_fmt_color_format(
					hdmi->output_bus_format)) |
		     FIELD_PREP(TX_VIDEO_DTV_OPTION_L_INPUT_COLOR_FORMAT,
				meson_mx_hdmi_bus_fmt_color_format(
					hdmi->input_bus_format)) |
		     FIELD_PREP(TX_VIDEO_DTV_OPTION_L_OUTPUT_COLOR_DEPTH,
				meson_mx_hdmi_bus_fmt_color_depth(
					hdmi->output_bus_format)) |
		     FIELD_PREP(TX_VIDEO_DTV_OPTION_L_INPUT_COLOR_DEPTH,
				meson_mx_hdmi_bus_fmt_color_depth(
					hdmi->input_bus_format)));

	/* TODO: color range mapping: 0=16-235/240; 1=16-240; 2=1-254; 3=0-255 - do I need to use hdmi->limited_rgb_quant_range ? */
	regmap_write(hdmi->regmap, TX_VIDEO_DTV_OPTION_H,
		     FIELD_PREP(TX_VIDEO_DTV_OPTION_H_OUTPUT_COLOR_RANGE,
				0x0) |
		     FIELD_PREP(TX_VIDEO_DTV_OPTION_H_INPUT_COLOR_RANGE,
				0x0));

	if (hdmi->colorimetry == HDMI_COLORIMETRY_ITU_709) {
		regmap_write(hdmi->regmap, TX_VIDEO_CSC_COEFF_B0, 0x7b);
		regmap_write(hdmi->regmap, TX_VIDEO_CSC_COEFF_B1, 0x12);
		regmap_write(hdmi->regmap, TX_VIDEO_CSC_COEFF_R0, 0x6c);
		regmap_write(hdmi->regmap, TX_VIDEO_CSC_COEFF_R1, 0x36);

		regmap_write(hdmi->regmap, TX_VIDEO_CSC_COEFF_CB0, 0xf2);
		regmap_write(hdmi->regmap, TX_VIDEO_CSC_COEFF_CB1, 0x2f);
		regmap_write(hdmi->regmap, TX_VIDEO_CSC_COEFF_CR0, 0xd4);
		regmap_write(hdmi->regmap, TX_VIDEO_CSC_COEFF_CR1, 0x77);
	} else {
		regmap_write(hdmi->regmap, TX_VIDEO_CSC_COEFF_B0, 0x2f);
		regmap_write(hdmi->regmap, TX_VIDEO_CSC_COEFF_B1, 0x1d);
		regmap_write(hdmi->regmap, TX_VIDEO_CSC_COEFF_R0, 0x8b);
		regmap_write(hdmi->regmap, TX_VIDEO_CSC_COEFF_R1, 0x4c);

		regmap_write(hdmi->regmap, TX_VIDEO_CSC_COEFF_CB0, 0x18);
		regmap_write(hdmi->regmap, TX_VIDEO_CSC_COEFF_CB1, 0x58);
		regmap_write(hdmi->regmap, TX_VIDEO_CSC_COEFF_CR0, 0xd0);
		regmap_write(hdmi->regmap, TX_VIDEO_CSC_COEFF_CR1, 0xb6);
	}
}

static void meson_mx_hdmi_config_serializer_clock(struct meson_mx_hdmi *hdmi)
{
	/* Serializer Internal clock setting */
	if (hdmi->colorimetry == HDMI_COLORIMETRY_ITU_709)
		regmap_write(hdmi->regmap, 0x0018, 0x22);
	else
		regmap_write(hdmi->regmap, 0x0018, 0x24);

#if 0
	// TODO: not ported yet
	if ((param->VIC==HDMI_1080p60)&&(param->color_depth==COLOR_30BIT)&&(hdmi_rd_reg(0x018)==0x22)) {
		regmap_write(hdmi->regmap, 0x0018,0x12);
	}
#endif
}

static void meson_mx_hdmi_reconfig_packet_setting(struct meson_mx_hdmi *hdmi)
{
	unsigned int alloc_active2, alloc_eof[2], alloc_vsync[3], alloc_sof[2];

	switch (hdmi->cea_mode) {
	case 31:
		/* 1920x1080p50 */
		alloc_active2 = 0x12;
		alloc_eof[0] = 0x10;
		alloc_eof[1] = 0x12;
		alloc_vsync[0] = 0x01;
		alloc_vsync[1] = 0x00;
		alloc_vsync[2] = 0x0a;
		alloc_sof[0] = 0xb6;
		alloc_sof[1] = 0x11;
		break;

	case 93:
		/* 3840x2160p24 */
		alloc_active2 = 0x12;
		alloc_eof[0] = 0x47;
		alloc_eof[1] = 0x12;
		alloc_vsync[0] = 0x01;
		alloc_vsync[1] = 0x00;
		alloc_vsync[2] = 0x0a;
		alloc_sof[0] = 0xf8;
		alloc_sof[1] = 0x52;
		break;

	case 94:
		/* 3840x2160p25 */
		alloc_active2 = 0x12;
		alloc_eof[0] = 0x44;
		alloc_eof[1] = 0x12;
		alloc_vsync[0] = 0x01;
		alloc_vsync[1] = 0x00;
		alloc_vsync[2] = 0x0a;
		alloc_sof[0] = 0xda;
		alloc_sof[1] = 0x52;
		break;

	case 95:
		/* 3840x2160p30 */
		alloc_active2 = 0x0f;
		alloc_eof[0] = 0x3a;
		alloc_eof[1] = 0x12;
		alloc_vsync[0] = 0x01;
		alloc_vsync[1] = 0x00;
		alloc_vsync[2] = 0x0a;
		alloc_sof[0] = 0x60;
		alloc_sof[1] = 0x52;
		break;

	case 98:
		/* 4096x2160p24 */
		alloc_active2 = 0x12;
		alloc_eof[0] = 0x47;
		alloc_eof[1] = 0x12;
		alloc_vsync[0] = 0x01;
		alloc_vsync[1] = 0x00;
		alloc_vsync[2] = 0x0a;
		alloc_sof[0] = 0xf8;
		alloc_sof[1] = 0x52;
		break;

	default:
		/* nothing to do */
		return;
	}

	/*
	 * The vendor driver says: manually configure these register to get
	 * stable video timings.
	 */
	regmap_write(hdmi->regmap, TX_PACKET_CONTROL_1,
		     TX_PACKET_CONTROL_1_PACKET_ALLOC_MODE |
		     FIELD_PREP(TX_PACKET_CONTROL_1_PACKET_START_LATENCY,
				26));
	regmap_write(hdmi->regmap, TX_PACKET_ALLOC_ACTIVE_1, 0x01);
	regmap_write(hdmi->regmap, TX_PACKET_ALLOC_ACTIVE_2, alloc_active2);
	regmap_write(hdmi->regmap, TX_PACKET_ALLOC_EOF_1, alloc_eof[0]);
	regmap_write(hdmi->regmap, TX_PACKET_ALLOC_EOF_2, alloc_eof[1]);
	regmap_write(hdmi->regmap, TX_CORE_ALLOC_VSYNC_0, alloc_vsync[0]);
	regmap_write(hdmi->regmap, TX_CORE_ALLOC_VSYNC_1, alloc_vsync[1]);
	regmap_write(hdmi->regmap, TX_CORE_ALLOC_VSYNC_2, alloc_vsync[2]);
	regmap_write(hdmi->regmap, TX_PACKET_ALLOC_SOF_1, alloc_sof[0]);
	regmap_write(hdmi->regmap, TX_PACKET_ALLOC_SOF_2, alloc_sof[1]);
	regmap_write(hdmi->regmap, TX_PACKET_CONTROL_1,
		     TX_PACKET_CONTROL_1_FORCE_PACKET_TIMING |
		     FIELD_PREP(TX_PACKET_CONTROL_1_PACKET_START_LATENCY,
				58));
}

static void meson_mx_hdmi_encoder_enable(struct drm_encoder *encoder)
{
	struct meson_mx_hdmi *hdmi = to_meson_mx_hdmi(encoder);
	struct meson_drm *priv = hdmi->drm->dev_private;
	int i, ret;

	regmap_update_bits(hdmi->regmap, HDMI_OTHER_CTRL1,
			   HDMI_OTHER_CTRL1_POWER_ON,
			   HDMI_OTHER_CTRL1_POWER_ON);

	meson_mx_hdmi_sys5_reset_assert(hdmi);

	regmap_write(hdmi->regmap, TX_PACKET_ALLOC_ACTIVE_1, 0x0);
	regmap_write(hdmi->regmap, TX_PACKET_CONTROL_1,
		     FIELD_PREP(TX_PACKET_CONTROL_1_PACKET_START_LATENCY, 58));
	regmap_write(hdmi->regmap, TX_PACKET_CONTROL_2,
		     TX_PACKET_CONTROL_2_HORIZONTAL_GC_PACKET_TRANSPORT_EN);

	meson_mx_hdmi_config_hdcp_registers(hdmi);

	regmap_write(hdmi->regmap, TX_AUDIO_CONTROL_MORE, 0x1);

	if (hdmi->cea_mode == 39)
		regmap_write(hdmi->regmap, TX_VIDEO_DTV_TIMING, 0x0);
	else
		regmap_write(hdmi->regmap, TX_VIDEO_DTV_TIMING,
			     TX_VIDEO_DTV_TIMING_DISABLE_VIC39_CORRECTION);

	regmap_write(hdmi->regmap, TX_CORE_DATA_CAPTURE_2,
		     TX_CORE_DATA_CAPTURE_2_INTERNAL_PACKET_ENABLE);
	regmap_write(hdmi->regmap, TX_CORE_DATA_MONITOR_1,
			TX_CORE_DATA_MONITOR_1_LANE0 |
			FIELD_PREP(TX_CORE_DATA_MONITOR_1_SELECT_LANE0, 0x7));
	regmap_write(hdmi->regmap, TX_CORE_DATA_MONITOR_2,
			FIELD_PREP(TX_CORE_DATA_MONITOR_2_MONITOR_SELECT,
				   0x2));

	if (hdmi->sink_is_hdmi)
		regmap_write(hdmi->regmap, TX_TMDS_MODE,
			     TX_TMDS_MODE_FORCED_HDMI |
			     TX_TMDS_MODE_HDMI_CONFIG);
	else
		regmap_write(hdmi->regmap, TX_TMDS_MODE,
			     TX_TMDS_MODE_FORCED_HDMI);

	regmap_write(hdmi->regmap, TX_SYS4_CONNECT_SEL_1, 0x0);

        /*
	 * Set tmds_clk pattern to be "0000011111" before being sent to AFE
	 * clock channel
	 */
	regmap_write(hdmi->regmap, TX_SYS4_CK_INV_VIDEO,
		     TX_SYS4_CK_INV_VIDEO_TMDS_CLK_PATTERN);

	regmap_write(hdmi->regmap, TX_SYS5_FIFO_CONFIG,
		     TX_SYS5_FIFO_CONFIG_CLK_CHANNEL3_OUTPUT_ENABLE |
		     TX_SYS5_FIFO_CONFIG_AFE_FIFO_CHANNEL2_ENABLE |
		     TX_SYS5_FIFO_CONFIG_AFE_FIFO_CHANNEL1_ENABLE |
		     TX_SYS5_FIFO_CONFIG_AFE_FIFO_CHANNEL0_ENABLE);

	meson_mx_hdmi_config_color_space(hdmi);

	meson_mx_hdmi_sys5_reset_deassert(hdmi);

	meson_mx_hdmi_config_serializer_clock(hdmi);
	meson_mx_hdmi_reconfig_packet_setting(hdmi);

	/* all resets need to be applied twice */
	for (i = 0; i < 2; i++) {
		regmap_write(hdmi->regmap, TX_SYS5_TX_SOFT_RESET_1,
			TX_SYS5_TX_SOFT_RESET_1_TX_PIXEL_RSTN |
			TX_SYS5_TX_SOFT_RESET_1_TX_TMDS_RSTN |
			TX_SYS5_TX_SOFT_RESET_1_TX_AUDIO_MASTER_RSTN |
			TX_SYS5_TX_SOFT_RESET_1_TX_AUDIO_RESAMPLE_RSTN |
			TX_SYS5_TX_SOFT_RESET_1_TX_I2S_RESET_RSTN |
			TX_SYS5_TX_SOFT_RESET_1_TX_DIG_RESET_N_CH2 |
			TX_SYS5_TX_SOFT_RESET_1_TX_DIG_RESET_N_CH1 |
			TX_SYS5_TX_SOFT_RESET_1_TX_DIG_RESET_N_CH0);
		regmap_write(hdmi->regmap, TX_SYS5_TX_SOFT_RESET_2,
			TX_SYS5_TX_SOFT_RESET_2_HDMI_CH3_RST_IN |
			TX_SYS5_TX_SOFT_RESET_2_HDMI_CH2_RST_IN |
			TX_SYS5_TX_SOFT_RESET_2_HDMI_CH1_RST_IN |
			TX_SYS5_TX_SOFT_RESET_2_HDMI_CH0_RST_IN |
			TX_SYS5_TX_SOFT_RESET_2_HDMI_SR_RST |
			TX_SYS5_TX_SOFT_RESET_2_TX_DDC_HDCP_RSTN |
			TX_SYS5_TX_SOFT_RESET_2_TX_DDC_EDID_RSTN |
			TX_SYS5_TX_SOFT_RESET_2_TX_DIG_RESET_N_CH3);
		usleep_range(5000, 10000);
		regmap_write(hdmi->regmap, TX_SYS5_TX_SOFT_RESET_1, 0x00);
		regmap_write(hdmi->regmap, TX_SYS5_TX_SOFT_RESET_2, 0x00);
		usleep_range(5000, 10000);
	}

	meson_venc_hdmi_bridge_reset(priv);

	if (!hdmi->phy_is_on) {
		ret = phy_power_on(hdmi->phy);
		if (ret)
			drm_err(hdmi->drm, "Failed to turn on PHY\n");
		else
			hdmi->phy_is_on = true;

		// TODO: phy_set_mode(hdmi->phy, ???); - maybe use phy_configure?
	}

	meson_venc_hdmi_encoder_enable(priv);
}

static void meson_mx_hdmi_set_avi_infoframe(struct meson_mx_hdmi *hdmi,
					    struct drm_display_mode *mode)
{
	u8 buf[HDMI_INFOFRAME_SIZE(AVI)], *video_code;
	struct hdmi_avi_infoframe frame;
	int ret;

	ret = drm_hdmi_avi_infoframe_from_display_mode(&frame,
						       &hdmi->connector, mode);
	if (ret < 0) {
		drm_err(hdmi->drm, "Failed to setup AVI infoframe: %d\n", ret);
		return;
	}

	/* fixed infoframe configuration not linked to the mode */
	frame.colorspace =
		meson_mx_hdmi_bus_fmt_hdmi_colorspace(hdmi->output_bus_format);
	if (frame.colorspace == HDMI_COLORSPACE_RGB)
		frame.colorimetry = HDMI_COLORIMETRY_NONE;
	else
		frame.colorimetry = hdmi->colorimetry;

	drm_hdmi_avi_infoframe_quant_range(&frame,
					   &hdmi->connector, mode,
					   hdmi->limited_rgb_quant_range ?
					   HDMI_QUANTIZATION_RANGE_LIMITED :
					   HDMI_QUANTIZATION_RANGE_FULL);

	ret = hdmi_avi_infoframe_pack(&frame, buf, sizeof(buf));
	if (ret < 0) {
		drm_err(hdmi->drm, "Failed to pack AVI infoframe: %d\n", ret);
		return;
	}

	video_code = &buf[HDMI_INFOFRAME_HEADER_SIZE + 3];
	if (*video_code > 109) {
		regmap_write(hdmi->regmap, TX_PKT_REG_EXCEPT0_BASE_ADDR,
			     *video_code);
		*video_code = 0x00;
	} else {
		regmap_write(hdmi->regmap, TX_PKT_REG_EXCEPT0_BASE_ADDR, 0x00);
	}

	meson_mx_hdmi_write_infoframe(hdmi, TX_PKT_REG_AVI_INFO_BASE_ADDR, buf,
				      sizeof(buf), true);
}

static void meson_mx_hdmi_set_vendor_infoframe(struct meson_mx_hdmi *hdmi,
					       struct drm_display_mode *mode)
{
	u8 buf[HDMI_INFOFRAME_HEADER_SIZE + 6];
	struct hdmi_vendor_infoframe frame;
	int ret;

	ret = drm_hdmi_vendor_infoframe_from_display_mode(&frame,
							  &hdmi->connector,
							  mode);
	if (ret) {
		drm_err(hdmi->drm, "Failed to setup vendor infoframe: %d\n",
			ret);
		return;
	}

	ret = hdmi_vendor_infoframe_pack(&frame, buf, sizeof(buf));
	if (ret < 0) {
		drm_err(hdmi->drm, "Failed to pack vendor infoframe: %d\n",
			ret);
		return;
	}

	meson_mx_hdmi_write_infoframe(hdmi, TX_PKT_REG_VEND_INFO_BASE_ADDR,
				      buf, sizeof(buf), true);
}

static void meson_mx_hdmi_set_spd_infoframe(struct meson_mx_hdmi *hdmi)
{
	u8 buf[HDMI_INFOFRAME_SIZE(SPD)];
	struct hdmi_spd_infoframe frame;
	int ret;

	ret = hdmi_spd_infoframe_init(&frame, "Amlogic", "Meson 6/8/8b");
	if (ret < 0) {
		drm_err(hdmi->drm, "Failed to setup SPD infoframe: %d\n", ret);
		return;
	}

	ret = hdmi_spd_infoframe_pack(&frame, buf, sizeof(buf));
	if (ret < 0) {
		drm_err(hdmi->drm, "Failed to pack SDP infoframe: %d\n", ret);
		return;
	}

	meson_mx_hdmi_write_infoframe(hdmi, TX_PKT_REG_SPD_INFO_BASE_ADDR, buf,
				     sizeof(buf), true);
}

static void meson_mx_hdmi_encoder_mode_set(struct drm_encoder *encoder,
					   struct drm_display_mode *mode,
					   struct drm_display_mode *adj_mode)
{
	struct meson_mx_hdmi *hdmi = to_meson_mx_hdmi(encoder);
	struct meson_drm *priv = hdmi->drm->dev_private;
	unsigned int vclk_freq, venc_freq, hdmi_freq;

	hdmi->cea_mode = drm_match_cea_mode(mode);

	switch (hdmi->cea_mode) {
	case 2 ... 3:
	case 6 ... 7:
	case 17 ... 18:
	case 21 ... 22:
		hdmi->colorimetry = HDMI_COLORIMETRY_ITU_601;
		break;

	default:
		hdmi->colorimetry = HDMI_COLORIMETRY_ITU_709;
		break;
	}

	/* VENC + VENC-DVI Mode setup */
	meson_venc_hdmi_mode_set(priv, hdmi->cea_mode, mode);

	vclk_freq = mode->clock;

	if (!hdmi->cea_mode) {
		meson_vclk_setup(priv, MESON_VCLK_TARGET_DMT, vclk_freq,
				 vclk_freq, vclk_freq, false);
	} else {
		if (mode->flags & DRM_MODE_FLAG_DBLCLK)
			vclk_freq *= 2;

		venc_freq = vclk_freq;
		hdmi_freq = vclk_freq;

		if (meson_venc_hdmi_venc_repeat(hdmi->cea_mode))
			venc_freq *= 2;

		vclk_freq = max(venc_freq, hdmi_freq);

		if (mode->flags & DRM_MODE_FLAG_DBLCLK)
			venc_freq /= 2;

		drm_dbg(hdmi->drm, "vclk:%d venc=%d hdmi=%d enci=%d\n",
			vclk_freq, venc_freq, hdmi_freq,
			priv->venc.hdmi_use_enci);

		meson_vclk_setup(priv, MESON_VCLK_TARGET_HDMI, vclk_freq,
				 venc_freq, hdmi_freq,
				 priv->venc.hdmi_use_enci);
	}

	if (hdmi->sink_is_hdmi) {
		enum hdmi_quantization_range quant_range;

		quant_range = drm_default_rgb_quant_range(mode);
		hdmi->limited_rgb_quant_range =
				quant_range == HDMI_QUANTIZATION_RANGE_LIMITED;

		meson_mx_hdmi_set_avi_infoframe(hdmi, mode);
		meson_mx_hdmi_set_vendor_infoframe(hdmi, mode);
		meson_mx_hdmi_set_spd_infoframe(hdmi);
	} else {
		hdmi->limited_rgb_quant_range = false;
	}
}

static const struct drm_encoder_helper_funcs meson_mx_hdmi_encoder_helper_funcs = {
	.disable	= meson_mx_hdmi_encoder_disable,
	.enable		= meson_mx_hdmi_encoder_enable,
	.mode_set	= meson_mx_hdmi_encoder_mode_set,
};

static const struct drm_encoder_funcs meson_mx_hdmi_encoder_funcs = {
	.destroy	= drm_encoder_cleanup,
};

static int meson_mx_hdmi_get_edid_block(void *data, u8 *buf,
					unsigned int block, size_t len)
{
	unsigned int i, regval, start = block * EDID_LENGTH;
	struct meson_mx_hdmi *hdmi = data;
	int ret;

	/* Start the DDC transaction */
	regmap_update_bits(hdmi->regmap, TX_HDCP_EDID_CONFIG,
				TX_HDCP_EDID_CONFIG_SYS_TRIGGER_CONFIG, 0);
	regmap_update_bits(hdmi->regmap, TX_HDCP_EDID_CONFIG,
				TX_HDCP_EDID_CONFIG_SYS_TRIGGER_CONFIG,
				TX_HDCP_EDID_CONFIG_SYS_TRIGGER_CONFIG);

	ret = regmap_read_poll_timeout(hdmi->regmap, TX_HDCP_ST_EDID_STATUS,
			regval,
			(regval & TX_HDCP_ST_EDID_STATUS_EDID_DATA_READY),
			1000, 200000);

	regmap_update_bits(hdmi->regmap, TX_HDCP_EDID_CONFIG,
			   TX_HDCP_EDID_CONFIG_SYS_TRIGGER_CONFIG, 0);

	if (ret)
		return ret;

	for (i = 0; i < len; i++) {
		regmap_read(hdmi->regmap, TX_RX_EDID_OFFSET + start + i,
			    &regval);
		buf[i] = regval;
	}

	return 0;
}

static int meson_mx_hdmi_connector_get_modes(struct drm_connector *connector)
{
	struct meson_mx_hdmi *hdmi = to_meson_mx_hdmi(connector);
	struct edid *edid;
	int ret;

	edid = drm_do_get_edid(connector, meson_mx_hdmi_get_edid_block, hdmi);
	if (!edid)
		return 0;

	hdmi->sink_is_hdmi = drm_detect_hdmi_monitor(edid);
	hdmi->sink_has_audio = drm_detect_monitor_audio(edid);

	drm_connector_update_edid_property(connector, edid);
	cec_notifier_set_phys_addr_from_edid(hdmi->cec_notifier, edid);

	ret = drm_add_edid_modes(connector, edid);

	kfree(edid);

	return ret;
}

static enum drm_mode_status
meson_mx_hdmi_connector_mode_valid(struct drm_connector *connector,
				   struct drm_display_mode *mode)
{
	return MODE_OK;
}

static struct drm_connector_helper_funcs meson_mx_hdmi_connector_helper_funcs = {
	.get_modes = meson_mx_hdmi_connector_get_modes,
	.mode_valid = meson_mx_hdmi_connector_mode_valid,
};

static enum drm_connector_status
meson_mx_hdmi_connector_detect(struct drm_connector *connector, bool force)
{
	struct meson_mx_hdmi *hdmi = to_meson_mx_hdmi(connector);
	enum drm_connector_status status;
	unsigned int val;

	regmap_read(hdmi->regmap, TX_HDCP_ST_EDID_STATUS, &val);
	if (val & TX_HDCP_ST_EDID_STATUS_HPD_STATUS)
		status = connector_status_connected;
	else
		status = connector_status_disconnected;

	if (status == connector_status_disconnected)
		cec_notifier_set_phys_addr(hdmi->cec_notifier,
					   CEC_PHYS_ADDR_INVALID);

	return status;
}

static const struct drm_connector_funcs meson_mx_hdmi_connector_funcs = {
	.fill_modes = drm_helper_probe_single_connector_modes,
	.detect = meson_mx_hdmi_connector_detect,
	.destroy = drm_connector_cleanup,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static int meson_mx_hdmi_register(struct drm_device *drm, struct meson_mx_hdmi *hdmi)
{
	struct drm_encoder *encoder = &hdmi->encoder;
	int ret;

	hdmi->input_bus_format = MEDIA_BUS_FMT_YUV8_1X24;
	hdmi->output_bus_format = MEDIA_BUS_FMT_RGB888_1X24;

	encoder->possible_crtcs = BIT(0);

	ret = drm_encoder_init(drm, encoder, &meson_mx_hdmi_encoder_funcs,
			       DRM_MODE_ENCODER_TMDS, NULL);
	if (ret)
		return ret;

	drm_encoder_helper_add(encoder, &meson_mx_hdmi_encoder_helper_funcs);

	hdmi->connector.polled = DRM_CONNECTOR_POLL_HPD;

	ret = drm_connector_init(drm, &hdmi->connector,
				 &meson_mx_hdmi_connector_funcs,
				 DRM_MODE_CONNECTOR_HDMIA);
	if (ret)
		return ret;

	drm_connector_helper_add(&hdmi->connector,
				 &meson_mx_hdmi_connector_helper_funcs);

	ret = drm_connector_attach_encoder(&hdmi->connector, encoder);
	if (ret)
		return ret;

	return 0;
}

static irqreturn_t meson_mx_hdmi_irq_thread(int irq, void *dev_id)
{
	struct meson_mx_hdmi *hdmi = dev_id;

	drm_helper_hpd_irq_event(hdmi->connector.dev);

	return IRQ_HANDLED;
}

static irqreturn_t meson_mx_hdmi_irq_handler(int irq, void *dev_id)
{
	struct meson_mx_hdmi *hdmi = dev_id;
	irqreturn_t ret = IRQ_WAKE_THREAD;
	u32 regval;

	regmap_read(hdmi->regmap, HDMI_OTHER_INTR_STAT, &regval);
	if (!regval)
		return IRQ_NONE;

	if (!(regval & (HDMI_OTHER_INTR_STAT_EDID_RISING |
			HDMI_OTHER_INTR_STAT_HPD_FALLING |
			HDMI_OTHER_INTR_STAT_HPD_RISING))) {
		drm_warn(hdmi->drm, "IRQ status has unknown bit set: 0x%04x\n",
			 regval);
		ret = IRQ_HANDLED;
	}

	regmap_write(hdmi->regmap, HDMI_OTHER_INTR_STAT_CLR, regval);

	return ret;
}

static void meson_mx_hdmi_exit(struct meson_mx_hdmi *hdmi)
{
	phy_exit(hdmi->phy);

	clk_disable_unprepare(hdmi->hdmi_clk);
	clk_disable_unprepare(hdmi->pclk);
}

static int meson_mx_hdmi_init(struct meson_mx_hdmi *hdmi, void __iomem *base)
{
	unsigned long hdmi_clk_hz = 24 * 1000 * 1000;
	unsigned long ddc_i2c_bus_clk_hz = 500 * 1000;
	u32 regval;
	int ret;

	ret = clk_prepare_enable(hdmi->pclk);
	if (ret) {
		drm_err(hdmi->drm, "Failed to enable the pclk: %d\n", ret);
		return ret;
	}

	ret = clk_set_rate(hdmi->hdmi_clk, hdmi_clk_hz);
	if (ret) {
		drm_err(hdmi->drm,
			"Failed to set HDMI controller clock to 24MHz: %d\n",
			ret);
		goto err_disable_pclk;
	}

	ret = clk_prepare_enable(hdmi->hdmi_clk);
	if (ret) {
		drm_err(hdmi->drm, "Failed to enable the HDMI clk: %d\n", ret);
		goto err_disable_pclk;
	}

	regval = readl(base + HDMI_CTRL_PORT);
	regval |= HDMI_CTRL_PORT_APB3_ERR_EN;
	writel(regval, base + HDMI_CTRL_PORT);

	regmap_write(hdmi->regmap, 0x0010, 0xff);

	regmap_write(hdmi->regmap, TX_HDCP_MODE, 0x40);

	ret = phy_init(hdmi->phy);
	if (ret) {
		drm_err(hdmi->drm, "Failed to initialize the PHY: %d\n", ret);
		goto err_disable_hdmi_clk;
	}

	/* Band-gap and main-bias. 0x1d = power-up, 0x00 = power-down */
	regmap_write(hdmi->regmap, 0x0017, 0x1d);

	/* Serializer Internal clock setting */
	regmap_write(hdmi->regmap, 0x0018, 0x22);

	/*
	 * bit[2:0]=011: CK channel output TMDS CLOCK
	 * bit[2:0]=101, ck channel output PHYCLCK
	 */
	regmap_write(hdmi->regmap, 0x001a, 0xfb);

	/* Termination resistor calib value */
	regmap_write(hdmi->regmap, TX_CORE_CALIB_VALUE, 0x0f);

	/* clear any pending interrupt */
	regmap_write(hdmi->regmap, HDMI_OTHER_INTR_STAT_CLR,
		     HDMI_OTHER_INTR_STAT_CLR_EDID_RISING |
		     HDMI_OTHER_INTR_STAT_CLR_HPD_FALLING |
		     HDMI_OTHER_INTR_STAT_CLR_HPD_RISING);
	/* unmask (= enable) all interrupts */
	regmap_write(hdmi->regmap, HDMI_OTHER_INTR_MASKN,
		     HDMI_OTHER_INTR_MASKN_TX_EDID_INT_RISE |
		     HDMI_OTHER_INTR_MASKN_TX_HPD_INT_FALL |
		     HDMI_OTHER_INTR_MASKN_TX_HPD_INT_RISE);

	/* HPD glitch filter */
	regmap_write(hdmi->regmap, TX_HDCP_HPD_FILTER_L, 0xa0);
	regmap_write(hdmi->regmap, TX_HDCP_HPD_FILTER_H, 0xa0);

	/* Disable MEM power-down */
	regmap_write(hdmi->regmap, TX_MEM_PD_REG0, 0);

	regmap_write(hdmi->regmap, TX_HDCP_CONFIG3,
		     FIELD_PREP(TX_HDCP_CONFIG3_DDC_I2C_BUS_CLOCK_TIME_DIVIDER,
				(hdmi_clk_hz / ddc_i2c_bus_clk_hz) - 1));

	/* Enable software controlled DDC transaction */
	regmap_write(hdmi->regmap, TX_HDCP_EDID_CONFIG,
		     TX_HDCP_EDID_CONFIG_FORCED_MEM_COPY_DONE |
		     TX_HDCP_EDID_CONFIG_MEM_COPY_DONE_CONFIG);
	regmap_write(hdmi->regmap, TX_CORE_EDID_CONFIG_MORE,
		     TX_CORE_EDID_CONFIG_MORE_SYS_TRIGGER_CONFIG_SEMI_MANU);

	return 0;

err_disable_pclk:
	clk_disable_unprepare(hdmi->pclk);
err_disable_hdmi_clk:
	clk_disable_unprepare(hdmi->hdmi_clk);
	return ret;
}

static int meson_mx_hdmi_bind(struct device *dev, struct device *master,
			      void *data)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct cec_connector_info conn_info;
	struct drm_device *drm = data;
	struct meson_mx_hdmi *hdmi;
	struct resource *res;
	void __iomem *base;
	int irq;
	int ret;

	hdmi = devm_kzalloc(dev, sizeof(*hdmi), GFP_KERNEL);
	if (!hdmi)
		return -ENOMEM;

	hdmi->dev = dev;
	hdmi->drm = drm;

	dev_set_drvdata(dev, hdmi);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	hdmi->regmap = devm_regmap_init(dev, NULL, base,
					&meson_mx_hdmi_regmap_config);
	if (IS_ERR(hdmi->regmap))
		return PTR_ERR(hdmi->regmap);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	hdmi->pclk = devm_clk_get(hdmi->dev, "pclk");
	if (IS_ERR(hdmi->pclk)) {
		ret = PTR_ERR(hdmi->pclk);
		drm_err(drm, "Failed to get the pclk: %d\n", ret);
		return ret;
	}

	hdmi->hdmi_clk = devm_clk_get(hdmi->dev, "hdmi");
	if (IS_ERR(hdmi->hdmi_clk)) {
		ret = PTR_ERR(hdmi->hdmi_clk);
		drm_err(drm, "Failed to get the hdmi clock: %d\n", ret);
		return ret;
	}

	hdmi->phy = devm_phy_get(hdmi->dev, "hdmi");
	if (IS_ERR(hdmi->phy)) {
		ret = PTR_ERR(hdmi->phy);
		drm_err(drm, "Failed to get the PHY: %d\n", ret);
		return ret;
	}

	ret = meson_mx_hdmi_init(hdmi, base);
	if (ret)
		return ret;

	ret = meson_mx_hdmi_register(drm, hdmi);
	if (ret) {
		drm_err(drm, "Failed to register HDMI: %d\n", ret);
		goto err_hdmi_exit;
	}

	cec_fill_conn_info_from_drm(&conn_info, &hdmi->connector);
	hdmi->cec_notifier = cec_notifier_conn_register(dev, NULL, &conn_info);
	if (!hdmi->cec_notifier)
		goto err_hdmi_exit;

	ret = devm_request_threaded_irq(dev, irq, meson_mx_hdmi_irq_handler,
					meson_mx_hdmi_irq_thread, 0, NULL,
					hdmi);
	if (ret) {
		drm_err(drm, "Failed to request threaded irq: %d\n", ret);
		goto err_hdmi_exit;
	}

	return 0;

err_hdmi_exit:
	meson_mx_hdmi_exit(hdmi);
	return ret;
}

static void meson_mx_hdmi_unbind(struct device *dev, struct device *master,
				 void *data)
{
	struct meson_mx_hdmi *hdmi = dev_get_drvdata(dev);

	hdmi->connector.funcs->destroy(&hdmi->connector);
	hdmi->encoder.funcs->destroy(&hdmi->encoder);

	meson_mx_hdmi_exit(hdmi);
}

static const struct component_ops meson_mx_hdmi_component_ops = {
	.bind = meson_mx_hdmi_bind,
	.unbind = meson_mx_hdmi_unbind,
};

static int meson_mx_hdmi_probe(struct platform_device *pdev)
{
	return component_add(&pdev->dev, &meson_mx_hdmi_component_ops);
}

static int meson_mx_hdmi_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &meson_mx_hdmi_component_ops);

	return 0;
}

static const struct of_device_id meson_mx_hdmi_of_table[] = {
	{ .compatible = "amlogic,meson8-hdmi-tx" },
	{ .compatible = "amlogic,meson8b-hdmi-tx" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, meson_mx_hdmi_of_table);

static struct platform_driver meson_mx_hdmi_platform_driver = {
	.probe		= meson_mx_hdmi_probe,
	.remove		= meson_mx_hdmi_remove,
	.driver		= {
		.name		= "meson-mx-hdmi",
		.of_match_table	= meson_mx_hdmi_of_table,
	},
};
module_platform_driver(meson_mx_hdmi_platform_driver);

MODULE_AUTHOR("Martin Blumenstingl <martin.blumenstingl@googlemail.com>");
MODULE_DESCRIPTION("Amlogic Meson8 and Meson8b HDMI TX DRM driver");
MODULE_LICENSE("GPL v2");
