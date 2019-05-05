// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2021 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 *
 * All registers and magic values are taken from Amlogic's GPL kernel sources:
 *   Copyright (C) 2010 Amlogic, Inc.
 */

#include <linux/clk.h>
#include <linux/component.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include <drm/display/drm_hdmi_helper.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_connector.h>
#include <drm/drm_device.h>
#include <drm/drm_edid.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_print.h>

#include <sound/hdmi-codec.h>

#include <uapi/linux/media-bus-format.h>

#include "meson_transwitch_hdmi.h"

#define HDMI_ADDR_PORT					0x0
#define HDMI_DATA_PORT					0x4
#define HDMI_CTRL_PORT					0x8
	#define HDMI_CTRL_PORT_APB3_ERR_EN		BIT(15)

struct meson_txc_hdmi {
	struct device			*dev;

	struct regmap			*regmap;

	struct clk			*pclk;
	struct clk			*sys_clk;

	struct phy			*phy;
	bool				phy_is_on;

	struct mutex			codec_mutex;
	enum drm_connector_status	last_connector_status;
	hdmi_codec_plugged_cb		codec_plugged_cb;
	struct device			*codec_dev;

	struct platform_device		*hdmi_codec_pdev;

	struct drm_connector		*current_connector;

	struct drm_bridge		bridge;
	struct drm_bridge		*next_bridge;

	bool				sink_is_hdmi;
};

#define bridge_to_meson_txc_hdmi(x) container_of(x, struct meson_txc_hdmi, bridge)

static const struct regmap_range meson_txc_hdmi_regmap_ranges[] = {
	regmap_reg_range(0x0000, 0x07ff),
	regmap_reg_range(0x8000, 0x800c),
};

static const struct regmap_access_table meson_txc_hdmi_regmap_access = {
	.yes_ranges = meson_txc_hdmi_regmap_ranges,
	.n_yes_ranges = ARRAY_SIZE(meson_txc_hdmi_regmap_ranges),
};

static int meson_txc_hdmi_reg_read(void *context, unsigned int addr,
				   unsigned int *data)
{
	void __iomem *base = context;

	writel(addr, base + HDMI_ADDR_PORT);
	writel(addr, base + HDMI_ADDR_PORT);

	*data = readl(base + HDMI_DATA_PORT);

	return 0;
}

static int meson_txc_hdmi_reg_write(void *context, unsigned int addr,
				    unsigned int data)
{
	void __iomem *base = context;

	writel(addr, base + HDMI_ADDR_PORT);
	writel(addr, base + HDMI_ADDR_PORT);

	writel(data, base + HDMI_DATA_PORT);

	return 0;
}

static const struct regmap_config meson_txc_hdmi_regmap_config = {
	.reg_bits = 16,
	.val_bits = 16,
	.reg_stride = 1,
	.reg_read = meson_txc_hdmi_reg_read,
	.reg_write = meson_txc_hdmi_reg_write,
	.rd_table = &meson_txc_hdmi_regmap_access,
	.wr_table = &meson_txc_hdmi_regmap_access,
	.max_register = HDMI_OTHER_RX_PACKET_INTR_CLR,
	.fast_io = true,
};

static void meson_txc_hdmi_write_infoframe(struct regmap *regmap,
					   unsigned int tx_pkt_reg, u8 *buf,
					   unsigned int len, bool enable)
{
	unsigned int i;

	/* Write the data bytes by starting at register offset 1 */
	for (i = HDMI_INFOFRAME_HEADER_SIZE; i < len; i++)
		regmap_write(regmap,
			     tx_pkt_reg + i - HDMI_INFOFRAME_HEADER_SIZE + 1,
			     buf[i]);

	/* Zero all remaining data bytes */
	for (; i < 0x1c; i++)
		regmap_write(regmap, tx_pkt_reg + i, 0x00);

	/* Write the header (which we skipped above) */
	regmap_write(regmap, tx_pkt_reg + 0x00, buf[3]);
	regmap_write(regmap, tx_pkt_reg + 0x1c, buf[0]);
	regmap_write(regmap, tx_pkt_reg + 0x1d, buf[1]);
	regmap_write(regmap, tx_pkt_reg + 0x1e, buf[2]);

	regmap_write(regmap, tx_pkt_reg + 0x1f, enable ? 0xff : 0x00);
}

static void meson_txc_hdmi_disable_infoframe(struct meson_txc_hdmi *priv,
					     unsigned int tx_pkt_reg)
{
	u8 buf[HDMI_INFOFRAME_HEADER_SIZE] = { 0 };

	meson_txc_hdmi_write_infoframe(priv->regmap, tx_pkt_reg, buf,
				       HDMI_INFOFRAME_HEADER_SIZE, false);
}

static void meson_txc_hdmi_sys5_reset_assert(struct meson_txc_hdmi *priv)
{
	/* A comment in the vendor driver says: bit5,6 is converted */
	regmap_write(priv->regmap, TX_SYS5_TX_SOFT_RESET_2,
		     TX_SYS5_TX_SOFT_RESET_2_HDMI_CH3_RST_IN |
		     TX_SYS5_TX_SOFT_RESET_2_HDMI_CH0_RST_IN);
	usleep_range(10, 20);

	regmap_write(priv->regmap, TX_SYS5_TX_SOFT_RESET_2,
		     TX_SYS5_TX_SOFT_RESET_2_HDMI_CH2_RST_IN |
		     TX_SYS5_TX_SOFT_RESET_2_HDMI_CH1_RST_IN);
	usleep_range(10, 20);

	regmap_write(priv->regmap, TX_SYS5_TX_SOFT_RESET_1,
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

static void meson_txc_hdmi_sys5_reset_deassert(struct meson_txc_hdmi *priv)
{
	/* Release the resets except tmds_clk */
	regmap_write(priv->regmap, TX_SYS5_TX_SOFT_RESET_1,
		     TX_SYS5_TX_SOFT_RESET_1_TX_TMDS_RSTN);
	usleep_range(10, 20);

	/* Release the tmds_clk reset as well */
	regmap_write(priv->regmap, TX_SYS5_TX_SOFT_RESET_1, 0x0);
	usleep_range(10, 20);

	regmap_write(priv->regmap, TX_SYS5_TX_SOFT_RESET_2,
		     TX_SYS5_TX_SOFT_RESET_2_HDMI_CH2_RST_IN |
		     TX_SYS5_TX_SOFT_RESET_2_HDMI_CH1_RST_IN |
		     TX_SYS5_TX_SOFT_RESET_2_HDMI_SR_RST);
	usleep_range(10, 20);

	regmap_write(priv->regmap, TX_SYS5_TX_SOFT_RESET_2,
		     TX_SYS5_TX_SOFT_RESET_2_HDMI_CH2_RST_IN |
		     TX_SYS5_TX_SOFT_RESET_2_HDMI_CH1_RST_IN);
	usleep_range(10, 20);
}

static void meson_txc_hdmi_config_hdcp_registers(struct meson_txc_hdmi *priv)
{
	regmap_write(priv->regmap, TX_HDCP_CONFIG0,
		     FIELD_PREP(TX_HDCP_CONFIG0_ROM_ENCRYPT_OFF, 0x3));
	regmap_write(priv->regmap, TX_HDCP_MEM_CONFIG, 0x0);
	regmap_write(priv->regmap, TX_HDCP_ENCRYPT_BYTE, 0x0);

	regmap_write(priv->regmap, TX_HDCP_MODE, TX_HDCP_MODE_CLEAR_AVMUTE);

	regmap_write(priv->regmap, TX_HDCP_MODE, TX_HDCP_MODE_ESS_CONFIG);
}

static u8 meson_txc_hdmi_bus_fmt_to_color_depth(unsigned int bus_format)
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

static u8 meson_txc_hdmi_bus_fmt_to_color_format(unsigned int bus_format)
{
	switch (bus_format) {
	case MEDIA_BUS_FMT_YUV8_1X24:
	case MEDIA_BUS_FMT_YUV10_1X30:
	case MEDIA_BUS_FMT_YUV12_1X36:
	case MEDIA_BUS_FMT_YUV16_1X48:
		/* Documented as YCbCr444 */
		return 0x1;

	case MEDIA_BUS_FMT_UYVY8_1X16:
	case MEDIA_BUS_FMT_UYVY10_1X20:
	case MEDIA_BUS_FMT_UYVY12_1X24:
		/* Documented as YCbCr422 */
		return 0x3;

	case MEDIA_BUS_FMT_RGB888_1X24:
	case MEDIA_BUS_FMT_RGB101010_1X30:
	case MEDIA_BUS_FMT_RGB121212_1X36:
	case MEDIA_BUS_FMT_RGB161616_1X48:
	default:
		/* Documented as RGB444 */
		return 0x0;
	}
}

static void meson_txc_hdmi_config_color_space(struct meson_txc_hdmi *priv,
					      unsigned int input_bus_format,
					      unsigned int output_bus_format,
					      enum hdmi_quantization_range quant_range,
					      enum hdmi_colorimetry colorimetry)
{
	unsigned int regval;

	regmap_write(priv->regmap, TX_VIDEO_DTV_MODE,
		     FIELD_PREP(TX_VIDEO_DTV_MODE_COLOR_DEPTH,
				meson_txc_hdmi_bus_fmt_to_color_depth(output_bus_format)));

	regmap_write(priv->regmap, TX_VIDEO_DTV_OPTION_L,
		     FIELD_PREP(TX_VIDEO_DTV_OPTION_L_OUTPUT_COLOR_FORMAT,
				meson_txc_hdmi_bus_fmt_to_color_format(output_bus_format)) |
		     FIELD_PREP(TX_VIDEO_DTV_OPTION_L_INPUT_COLOR_FORMAT,
				meson_txc_hdmi_bus_fmt_to_color_format(input_bus_format)) |
		     FIELD_PREP(TX_VIDEO_DTV_OPTION_L_OUTPUT_COLOR_DEPTH,
				meson_txc_hdmi_bus_fmt_to_color_depth(output_bus_format)) |
		     FIELD_PREP(TX_VIDEO_DTV_OPTION_L_INPUT_COLOR_DEPTH,
				meson_txc_hdmi_bus_fmt_to_color_depth(input_bus_format)));

	if (quant_range == HDMI_QUANTIZATION_RANGE_LIMITED)
		regval = FIELD_PREP(TX_VIDEO_DTV_OPTION_H_OUTPUT_COLOR_RANGE,
				    TX_VIDEO_DTV_OPTION_H_COLOR_RANGE_16_235) |
			 FIELD_PREP(TX_VIDEO_DTV_OPTION_H_INPUT_COLOR_RANGE,
				    TX_VIDEO_DTV_OPTION_H_COLOR_RANGE_16_235);
	else
		regval = FIELD_PREP(TX_VIDEO_DTV_OPTION_H_OUTPUT_COLOR_RANGE,
				    TX_VIDEO_DTV_OPTION_H_COLOR_RANGE_0_255) |
			 FIELD_PREP(TX_VIDEO_DTV_OPTION_H_INPUT_COLOR_RANGE,
				    TX_VIDEO_DTV_OPTION_H_COLOR_RANGE_0_255);

	regmap_write(priv->regmap, TX_VIDEO_DTV_OPTION_H, regval);

	if (colorimetry == HDMI_COLORIMETRY_ITU_601) {
		regmap_write(priv->regmap, TX_VIDEO_CSC_COEFF_B0, 0x2f);
		regmap_write(priv->regmap, TX_VIDEO_CSC_COEFF_B1, 0x1d);
		regmap_write(priv->regmap, TX_VIDEO_CSC_COEFF_R0, 0x8b);
		regmap_write(priv->regmap, TX_VIDEO_CSC_COEFF_R1, 0x4c);

		regmap_write(priv->regmap, TX_VIDEO_CSC_COEFF_CB0, 0x18);
		regmap_write(priv->regmap, TX_VIDEO_CSC_COEFF_CB1, 0x58);
		regmap_write(priv->regmap, TX_VIDEO_CSC_COEFF_CR0, 0xd0);
		regmap_write(priv->regmap, TX_VIDEO_CSC_COEFF_CR1, 0xb6);
	} else {
		regmap_write(priv->regmap, TX_VIDEO_CSC_COEFF_B0, 0x7b);
		regmap_write(priv->regmap, TX_VIDEO_CSC_COEFF_B1, 0x12);
		regmap_write(priv->regmap, TX_VIDEO_CSC_COEFF_R0, 0x6c);
		regmap_write(priv->regmap, TX_VIDEO_CSC_COEFF_R1, 0x36);

		regmap_write(priv->regmap, TX_VIDEO_CSC_COEFF_CB0, 0xf2);
		regmap_write(priv->regmap, TX_VIDEO_CSC_COEFF_CB1, 0x2f);
		regmap_write(priv->regmap, TX_VIDEO_CSC_COEFF_CR0, 0xd4);
		regmap_write(priv->regmap, TX_VIDEO_CSC_COEFF_CR1, 0x77);
	}
}

static void meson_txc_hdmi_config_serializer_clock(struct meson_txc_hdmi *priv,
						   enum hdmi_colorimetry colorimetry)
{
	/* Serializer Internal clock setting */
	if (colorimetry == HDMI_COLORIMETRY_ITU_601)
		regmap_write(priv->regmap, TX_SYS1_PLL, 0x24);
	else
		regmap_write(priv->regmap, TX_SYS1_PLL, 0x22);

#if 0
	// TODO: not ported yet
	if ((param->VIC==HDMI_1080p60)&&(param->color_depth==COLOR_30BIT)&&(hdmi_rd_reg(0x018)==0x22)) {
		regmap_write(priv->regmap, TX_SYS1_PLL, 0x12);
	}
#endif
}

static void meson_txc_hdmi_reconfig_packet_setting(struct meson_txc_hdmi *priv,
						   u8 cea_mode)
{
	u8 alloc_active2, alloc_eof1, alloc_sof1, alloc_sof2;

	regmap_write(priv->regmap, TX_PACKET_CONTROL_1,
		     FIELD_PREP(TX_PACKET_CONTROL_1_PACKET_START_LATENCY, 58));
	regmap_write(priv->regmap, TX_PACKET_CONTROL_2,
		     TX_PACKET_CONTROL_2_HORIZONTAL_GC_PACKET_TRANSPORT_EN);

	switch (cea_mode) {
	case 31:
		/* 1920x1080p50 */
		alloc_active2 = 0x12;
		alloc_eof1 = 0x10;
		alloc_sof1 = 0xb6;
		alloc_sof2 = 0x11;
		break;
	case 93:
		/* 3840x2160p24 */
		alloc_active2 = 0x12;
		alloc_eof1 = 0x47;
		alloc_sof1 = 0xf8;
		alloc_sof2 = 0x52;
		break;
	case 94:
		/* 3840x2160p25 */
		alloc_active2 = 0x12;
		alloc_eof1 = 0x44;
		alloc_sof1 = 0xda;
		alloc_sof2 = 0x52;
		break;
	case 95:
		/* 3840x2160p30 */
		alloc_active2 = 0x0f;
		alloc_eof1 = 0x3a;
		alloc_sof1 = 0x60;
		alloc_sof2 = 0x52;
		break;
	case 98:
		/* 4096x2160p24 */
		alloc_active2 = 0x12;
		alloc_eof1 = 0x47;
		alloc_sof1 = 0xf8;
		alloc_sof2 = 0x52;
		break;
	default:
		/* Disable the special packet settings only */
		regmap_write(priv->regmap, TX_PACKET_ALLOC_ACTIVE_1, 0x00);
		return;
	}

	/*
	 * The vendor driver says: manually configure these register to get
	 * stable video timings.
	 */
	regmap_write(priv->regmap, TX_PACKET_ALLOC_ACTIVE_1, 0x01);
	regmap_write(priv->regmap, TX_PACKET_ALLOC_ACTIVE_2, alloc_active2);
	regmap_write(priv->regmap, TX_PACKET_ALLOC_EOF_1, alloc_eof1);
	regmap_write(priv->regmap, TX_PACKET_ALLOC_EOF_2, 0x12);
	regmap_write(priv->regmap, TX_CORE_ALLOC_VSYNC_0, 0x01);
	regmap_write(priv->regmap, TX_CORE_ALLOC_VSYNC_1, 0x00);
	regmap_write(priv->regmap, TX_CORE_ALLOC_VSYNC_2, 0x0a);
	regmap_write(priv->regmap, TX_PACKET_ALLOC_SOF_1, alloc_sof1);
	regmap_write(priv->regmap, TX_PACKET_ALLOC_SOF_2, alloc_sof2);
	regmap_update_bits(priv->regmap, TX_PACKET_CONTROL_1,
			   TX_PACKET_CONTROL_1_FORCE_PACKET_TIMING,
			   TX_PACKET_CONTROL_1_FORCE_PACKET_TIMING);
}

static void meson_txc_hdmi_set_avi_infoframe(struct meson_txc_hdmi *priv,
					     struct drm_connector *conn,
					     const struct drm_display_mode *mode,
					     const struct drm_connector_state *conn_state,
					     unsigned int output_bus_format,
					     enum hdmi_quantization_range quant_range,
					     enum hdmi_colorimetry colorimetry)
{
	u8 buf[HDMI_INFOFRAME_SIZE(AVI)], *video_code;
	struct hdmi_avi_infoframe frame;
	int ret;

	ret = drm_hdmi_avi_infoframe_from_display_mode(&frame, conn, mode);
	if (ret < 0) {
		drm_err(priv->bridge.dev,
			"Failed to setup AVI infoframe: %d\n", ret);
		return;
	}

	switch (output_bus_format) {
	case MEDIA_BUS_FMT_YUV8_1X24:
	case MEDIA_BUS_FMT_YUV10_1X30:
	case MEDIA_BUS_FMT_YUV12_1X36:
	case MEDIA_BUS_FMT_YUV16_1X48:
		frame.colorspace = HDMI_COLORSPACE_YUV444;
		break;

	case MEDIA_BUS_FMT_UYVY8_1X16:
	case MEDIA_BUS_FMT_UYVY10_1X20:
	case MEDIA_BUS_FMT_UYVY12_1X24:
		frame.colorspace = HDMI_COLORSPACE_YUV422;
		break;

	case MEDIA_BUS_FMT_RGB888_1X24:
	case MEDIA_BUS_FMT_RGB101010_1X30:
	case MEDIA_BUS_FMT_RGB121212_1X36:
	case MEDIA_BUS_FMT_RGB161616_1X48:
	default:
		frame.colorspace = HDMI_COLORSPACE_RGB;
		break;
	}

	drm_hdmi_avi_infoframe_colorimetry(&frame, conn_state);
	drm_hdmi_avi_infoframe_quant_range(&frame, conn, mode, quant_range);
	drm_hdmi_avi_infoframe_bars(&frame, conn_state);

	ret = hdmi_avi_infoframe_pack(&frame, buf, sizeof(buf));
	if (ret < 0) {
		drm_err(priv->bridge.dev,
			"Failed to pack AVI infoframe: %d\n", ret);
		return;
	}

	video_code = &buf[HDMI_INFOFRAME_HEADER_SIZE + 3];
	if (*video_code > 108) {
		regmap_write(priv->regmap, TX_PKT_REG_EXCEPT0_BASE_ADDR,
			     *video_code);
		*video_code = 0x00;
	} else {
		regmap_write(priv->regmap, TX_PKT_REG_EXCEPT0_BASE_ADDR,
			     0x00);
	}

	meson_txc_hdmi_write_infoframe(priv->regmap,
				       TX_PKT_REG_AVI_INFO_BASE_ADDR, buf,
				       sizeof(buf), true);
}

static void meson_txc_hdmi_set_vendor_infoframe(struct meson_txc_hdmi *priv,
						struct drm_connector *conn,
						const struct drm_display_mode *mode)
{
	u8 buf[HDMI_INFOFRAME_HEADER_SIZE + 6];
	struct hdmi_vendor_infoframe frame;
	int ret;

	ret = drm_hdmi_vendor_infoframe_from_display_mode(&frame, conn, mode);
	if (ret) {
		drm_dbg(priv->bridge.dev,
			"Failed to setup vendor infoframe: %d\n", ret);
		return;
	}

	ret = hdmi_vendor_infoframe_pack(&frame, buf, sizeof(buf));
	if (ret < 0) {
		drm_err(priv->bridge.dev,
			"Failed to pack vendor infoframe: %d\n", ret);
		return;
	}

	meson_txc_hdmi_write_infoframe(priv->regmap,
				       TX_PKT_REG_VEND_INFO_BASE_ADDR, buf,
				       sizeof(buf), true);
}

static void meson_txc_hdmi_set_spd_infoframe(struct meson_txc_hdmi *priv)
{
	u8 buf[HDMI_INFOFRAME_SIZE(SPD)];
	struct hdmi_spd_infoframe frame;
	int ret;

	ret = hdmi_spd_infoframe_init(&frame, "Amlogic", "Meson TXC HDMI");
	if (ret < 0) {
		drm_err(priv->bridge.dev,
			"Failed to setup SPD infoframe: %d\n", ret);
		return;
	}

	ret = hdmi_spd_infoframe_pack(&frame, buf, sizeof(buf));
	if (ret < 0) {
		drm_err(priv->bridge.dev,
			"Failed to pack SDP infoframe: %d\n", ret);
		return;
	}

	meson_txc_hdmi_write_infoframe(priv->regmap,
				       TX_PKT_REG_SPD_INFO_BASE_ADDR, buf,
				       sizeof(buf), true);
}

static void meson_txc_hdmi_handle_plugged_change(struct meson_txc_hdmi *priv)
{
	bool plugged;

	plugged = priv->last_connector_status == connector_status_connected;

	if (priv->codec_dev && priv->codec_plugged_cb)
		priv->codec_plugged_cb(priv->codec_dev, plugged);
}

static int meson_txc_hdmi_bridge_attach(struct drm_bridge *bridge,
					enum drm_bridge_attach_flags flags)
{
	struct meson_txc_hdmi *priv = bridge->driver_private;

	if (!(flags & DRM_BRIDGE_ATTACH_NO_CONNECTOR)) {
		drm_err(bridge->dev,
			"DRM_BRIDGE_ATTACH_NO_CONNECTOR flag is not set but needed\n");
		return -EINVAL;
	}

	return drm_bridge_attach(bridge->encoder, priv->next_bridge, bridge,
				 flags);
}

/* Can return a maximum of 11 possible output formats for a mode/connector */
#define MAX_OUTPUT_SEL_FORMATS	11

static u32 *
meson_txc_hdmi_bridge_atomic_get_output_bus_fmts(struct drm_bridge *bridge,
						 struct drm_bridge_state *bridge_state,
						 struct drm_crtc_state *crtc_state,
						 struct drm_connector_state *conn_state,
						 unsigned int *num_output_fmts)
{
	struct drm_connector *conn = conn_state->connector;
	struct drm_display_info *info = &conn->display_info;
	u8 max_bpc = conn_state->max_requested_bpc;
	unsigned int i = 0;
	u32 *output_fmts;

	*num_output_fmts = 0;

	output_fmts = kcalloc(MAX_OUTPUT_SEL_FORMATS, sizeof(*output_fmts),
			      GFP_KERNEL);
	if (!output_fmts)
		return NULL;

	/* If we are the only bridge, avoid negotiating with ourselves */
	if (list_is_singular(&bridge->encoder->bridge_chain)) {
		*num_output_fmts = 1;
		output_fmts[0] = MEDIA_BUS_FMT_FIXED;

		return output_fmts;
	}

	/*
	 * Order bus formats from 16bit to 8bit and from YUV422 to RGB
	 * if supported. In any case the default RGB888 format is added
	 */

	if (max_bpc >= 16 && info->bpc == 16) {
		if (info->color_formats & DRM_COLOR_FORMAT_YCBCR444)
			output_fmts[i++] = MEDIA_BUS_FMT_YUV16_1X48;

		output_fmts[i++] = MEDIA_BUS_FMT_RGB161616_1X48;
	}

	if (max_bpc >= 12 && info->bpc >= 12) {
		if (info->color_formats & DRM_COLOR_FORMAT_YCBCR422)
			output_fmts[i++] = MEDIA_BUS_FMT_UYVY12_1X24;

		if (info->color_formats & DRM_COLOR_FORMAT_YCBCR444)
			output_fmts[i++] = MEDIA_BUS_FMT_YUV12_1X36;

		output_fmts[i++] = MEDIA_BUS_FMT_RGB121212_1X36;
	}

	if (max_bpc >= 10 && info->bpc >= 10) {
		if (info->color_formats & DRM_COLOR_FORMAT_YCBCR422)
			output_fmts[i++] = MEDIA_BUS_FMT_UYVY10_1X20;

		if (info->color_formats & DRM_COLOR_FORMAT_YCBCR444)
			output_fmts[i++] = MEDIA_BUS_FMT_YUV10_1X30;

		output_fmts[i++] = MEDIA_BUS_FMT_RGB101010_1X30;
	}

	if (info->color_formats & DRM_COLOR_FORMAT_YCBCR422)
		output_fmts[i++] = MEDIA_BUS_FMT_UYVY8_1X16;

	if (info->color_formats & DRM_COLOR_FORMAT_YCBCR444)
		output_fmts[i++] = MEDIA_BUS_FMT_YUV8_1X24;

	/* Default 8bit RGB fallback */
	output_fmts[i++] = MEDIA_BUS_FMT_RGB888_1X24;

	*num_output_fmts = i;

	return output_fmts;
}

/* Can return a maximum of 3 possible input formats for an output format */
#define MAX_INPUT_SEL_FORMATS	3

static u32 *
meson_txc_hdmi_bridge_atomic_get_input_bus_fmts(struct drm_bridge *bridge,
						struct drm_bridge_state *bridge_state,
						struct drm_crtc_state *crtc_state,
						struct drm_connector_state *conn_state,
						u32 output_fmt,
						unsigned int *num_input_fmts)
{
	u32 *input_fmts;
	unsigned int i = 0;

	*num_input_fmts = 0;

	input_fmts = kcalloc(MAX_INPUT_SEL_FORMATS, sizeof(*input_fmts),
			     GFP_KERNEL);
	if (!input_fmts)
		return NULL;

	switch (output_fmt) {
	/* If MEDIA_BUS_FMT_FIXED is tested, return default bus format */
	case MEDIA_BUS_FMT_FIXED:
		input_fmts[i++] = MEDIA_BUS_FMT_RGB888_1X24;
		break;

	/* 8bit */
	case MEDIA_BUS_FMT_RGB888_1X24:
		input_fmts[i++] = MEDIA_BUS_FMT_RGB888_1X24;
		input_fmts[i++] = MEDIA_BUS_FMT_YUV8_1X24;
		input_fmts[i++] = MEDIA_BUS_FMT_UYVY8_1X16;
		break;
	case MEDIA_BUS_FMT_YUV8_1X24:
		input_fmts[i++] = MEDIA_BUS_FMT_YUV8_1X24;
		input_fmts[i++] = MEDIA_BUS_FMT_UYVY8_1X16;
		input_fmts[i++] = MEDIA_BUS_FMT_RGB888_1X24;
		break;
	case MEDIA_BUS_FMT_UYVY8_1X16:
		input_fmts[i++] = MEDIA_BUS_FMT_UYVY8_1X16;
		input_fmts[i++] = MEDIA_BUS_FMT_YUV8_1X24;
		input_fmts[i++] = MEDIA_BUS_FMT_RGB888_1X24;
		break;

	/* 10bit */
	case MEDIA_BUS_FMT_RGB101010_1X30:
		input_fmts[i++] = MEDIA_BUS_FMT_RGB101010_1X30;
		input_fmts[i++] = MEDIA_BUS_FMT_YUV10_1X30;
		input_fmts[i++] = MEDIA_BUS_FMT_UYVY10_1X20;
		break;
	case MEDIA_BUS_FMT_YUV10_1X30:
		input_fmts[i++] = MEDIA_BUS_FMT_YUV10_1X30;
		input_fmts[i++] = MEDIA_BUS_FMT_UYVY10_1X20;
		input_fmts[i++] = MEDIA_BUS_FMT_RGB101010_1X30;
		break;
	case MEDIA_BUS_FMT_UYVY10_1X20:
		input_fmts[i++] = MEDIA_BUS_FMT_UYVY10_1X20;
		input_fmts[i++] = MEDIA_BUS_FMT_YUV10_1X30;
		input_fmts[i++] = MEDIA_BUS_FMT_RGB101010_1X30;
		break;

	/* 12bit */
	case MEDIA_BUS_FMT_RGB121212_1X36:
		input_fmts[i++] = MEDIA_BUS_FMT_RGB121212_1X36;
		input_fmts[i++] = MEDIA_BUS_FMT_YUV12_1X36;
		input_fmts[i++] = MEDIA_BUS_FMT_UYVY12_1X24;
		break;
	case MEDIA_BUS_FMT_YUV12_1X36:
		input_fmts[i++] = MEDIA_BUS_FMT_YUV12_1X36;
		input_fmts[i++] = MEDIA_BUS_FMT_UYVY12_1X24;
		input_fmts[i++] = MEDIA_BUS_FMT_RGB121212_1X36;
		break;
	case MEDIA_BUS_FMT_UYVY12_1X24:
		input_fmts[i++] = MEDIA_BUS_FMT_UYVY12_1X24;
		input_fmts[i++] = MEDIA_BUS_FMT_YUV12_1X36;
		input_fmts[i++] = MEDIA_BUS_FMT_RGB121212_1X36;
		break;

	/* 16bit */
	case MEDIA_BUS_FMT_RGB161616_1X48:
		input_fmts[i++] = MEDIA_BUS_FMT_RGB161616_1X48;
		input_fmts[i++] = MEDIA_BUS_FMT_YUV16_1X48;
		break;
	case MEDIA_BUS_FMT_YUV16_1X48:
		input_fmts[i++] = MEDIA_BUS_FMT_YUV16_1X48;
		input_fmts[i++] = MEDIA_BUS_FMT_RGB161616_1X48;
		break;
	}

	*num_input_fmts = i;

	if (*num_input_fmts == 0) {
		kfree(input_fmts);
		input_fmts = NULL;
	}

	return input_fmts;
}

static void meson_txc_hdmi_bridge_atomic_enable(struct drm_bridge *bridge,
						struct drm_bridge_state *old_bridge_state)
{
	struct meson_txc_hdmi *priv = bridge_to_meson_txc_hdmi(bridge);
	struct drm_atomic_state *state = old_bridge_state->base.state;
	enum hdmi_quantization_range quant_range;
	struct drm_connector_state *conn_state;
	struct drm_bridge_state *bridge_state;
	const struct drm_display_mode *mode;
	enum hdmi_colorimetry colorimetry;
	struct drm_crtc_state *crtc_state;
	struct drm_connector *connector;
	unsigned int i;
	u8 cea_mode;

	bridge_state = drm_atomic_get_new_bridge_state(state, bridge);

	connector = drm_atomic_get_new_connector_for_encoder(state,
							     bridge->encoder);
	if (WARN_ON(!connector))
		return;

	conn_state = drm_atomic_get_new_connector_state(state, connector);
	if (WARN_ON(!conn_state))
		return;

	crtc_state = drm_atomic_get_new_crtc_state(state, conn_state->crtc);
	if (WARN_ON(!crtc_state))
		return;

	priv->current_connector = connector;

	mode = &crtc_state->adjusted_mode;

	cea_mode = drm_match_cea_mode(mode);

	if (priv->sink_is_hdmi) {
		quant_range = drm_default_rgb_quant_range(mode);

		switch (cea_mode) {
		case 2 ... 3:
		case 6 ... 7:
		case 17 ... 18:
		case 21 ... 22:
			colorimetry = HDMI_COLORIMETRY_ITU_601;
			break;

		default:
			colorimetry = HDMI_COLORIMETRY_ITU_709;
			break;
		}

		meson_txc_hdmi_set_avi_infoframe(priv, connector, mode,
						 conn_state,
						 bridge_state->output_bus_cfg.format,
						 quant_range, colorimetry);
		meson_txc_hdmi_set_vendor_infoframe(priv, connector, mode);
		meson_txc_hdmi_set_spd_infoframe(priv);
	} else {
		quant_range = HDMI_QUANTIZATION_RANGE_FULL;
		colorimetry = HDMI_COLORIMETRY_NONE;
	}

	meson_txc_hdmi_sys5_reset_assert(priv);

	meson_txc_hdmi_config_hdcp_registers(priv);

	if (cea_mode == 39)
		regmap_write(priv->regmap, TX_VIDEO_DTV_TIMING, 0x0);
	else
		regmap_write(priv->regmap, TX_VIDEO_DTV_TIMING,
			     TX_VIDEO_DTV_TIMING_DISABLE_VIC39_CORRECTION);

	regmap_write(priv->regmap, TX_CORE_DATA_CAPTURE_2,
		     TX_CORE_DATA_CAPTURE_2_INTERNAL_PACKET_ENABLE);
	regmap_write(priv->regmap, TX_CORE_DATA_MONITOR_1,
		     TX_CORE_DATA_MONITOR_1_LANE0 |
		     FIELD_PREP(TX_CORE_DATA_MONITOR_1_SELECT_LANE0, 0x7));
	regmap_write(priv->regmap, TX_CORE_DATA_MONITOR_2,
		     FIELD_PREP(TX_CORE_DATA_MONITOR_2_MONITOR_SELECT, 0x2));

	if (priv->sink_is_hdmi)
		regmap_write(priv->regmap, TX_TMDS_MODE,
			     TX_TMDS_MODE_FORCED_HDMI |
			     TX_TMDS_MODE_HDMI_CONFIG);
	else
		regmap_write(priv->regmap, TX_TMDS_MODE,
			     TX_TMDS_MODE_FORCED_HDMI);

	regmap_write(priv->regmap, TX_SYS4_CONNECT_SEL_1, 0x0);

	/*
	 * Set tmds_clk pattern to be "0000011111" before being sent to AFE
	 * clock channel.
	 */
	regmap_write(priv->regmap, TX_SYS4_CK_INV_VIDEO,
		     TX_SYS4_CK_INV_VIDEO_TMDS_CLK_PATTERN);

	regmap_write(priv->regmap, TX_SYS5_FIFO_CONFIG,
		     TX_SYS5_FIFO_CONFIG_CLK_CHANNEL3_OUTPUT_ENABLE |
		     TX_SYS5_FIFO_CONFIG_AFE_FIFO_CHANNEL2_ENABLE |
		     TX_SYS5_FIFO_CONFIG_AFE_FIFO_CHANNEL1_ENABLE |
		     TX_SYS5_FIFO_CONFIG_AFE_FIFO_CHANNEL0_ENABLE);

	meson_txc_hdmi_config_color_space(priv,
					  bridge_state->input_bus_cfg.format,
					  bridge_state->output_bus_cfg.format,
					  quant_range, colorimetry);

	meson_txc_hdmi_sys5_reset_deassert(priv);

	meson_txc_hdmi_config_serializer_clock(priv, colorimetry);
	meson_txc_hdmi_reconfig_packet_setting(priv, cea_mode);

	/* all resets need to be applied twice */
	for (i = 0; i < 2; i++) {
		regmap_write(priv->regmap, TX_SYS5_TX_SOFT_RESET_1,
			     TX_SYS5_TX_SOFT_RESET_1_TX_PIXEL_RSTN |
			     TX_SYS5_TX_SOFT_RESET_1_TX_TMDS_RSTN |
			     TX_SYS5_TX_SOFT_RESET_1_TX_AUDIO_MASTER_RSTN |
			     TX_SYS5_TX_SOFT_RESET_1_TX_AUDIO_RESAMPLE_RSTN |
			     TX_SYS5_TX_SOFT_RESET_1_TX_I2S_RESET_RSTN |
			     TX_SYS5_TX_SOFT_RESET_1_TX_DIG_RESET_N_CH2 |
			     TX_SYS5_TX_SOFT_RESET_1_TX_DIG_RESET_N_CH1 |
			     TX_SYS5_TX_SOFT_RESET_1_TX_DIG_RESET_N_CH0);
		regmap_write(priv->regmap, TX_SYS5_TX_SOFT_RESET_2,
			     TX_SYS5_TX_SOFT_RESET_2_HDMI_CH3_RST_IN |
			     TX_SYS5_TX_SOFT_RESET_2_HDMI_CH2_RST_IN |
			     TX_SYS5_TX_SOFT_RESET_2_HDMI_CH1_RST_IN |
			     TX_SYS5_TX_SOFT_RESET_2_HDMI_CH0_RST_IN |
			     TX_SYS5_TX_SOFT_RESET_2_HDMI_SR_RST |
			     TX_SYS5_TX_SOFT_RESET_2_TX_DDC_HDCP_RSTN |
			     TX_SYS5_TX_SOFT_RESET_2_TX_DDC_EDID_RSTN |
			     TX_SYS5_TX_SOFT_RESET_2_TX_DIG_RESET_N_CH3);
		usleep_range(5000, 10000);
		regmap_write(priv->regmap, TX_SYS5_TX_SOFT_RESET_1, 0x00);
		regmap_write(priv->regmap, TX_SYS5_TX_SOFT_RESET_2, 0x00);
		usleep_range(5000, 10000);
	}

	if (!priv->phy_is_on) {
		int ret;

		ret = phy_power_on(priv->phy);
		if (ret)
			drm_err(bridge->dev, "Failed to turn on PHY\n");
		else
			priv->phy_is_on = true;
	}
}

static void meson_txc_hdmi_bridge_atomic_disable(struct drm_bridge *bridge,
						 struct drm_bridge_state *old_bridge_state)
{
	struct meson_txc_hdmi *priv = bridge_to_meson_txc_hdmi(bridge);

	priv->current_connector = NULL;

	if (priv->phy_is_on) {
		int ret;

		ret = phy_power_off(priv->phy);
		if (ret)
			drm_err(bridge->dev, "Failed to turn off PHY\n");
		else
			priv->phy_is_on = false;
	}

	meson_txc_hdmi_disable_infoframe(priv, TX_PKT_REG_AUDIO_INFO_BASE_ADDR);
	meson_txc_hdmi_disable_infoframe(priv, TX_PKT_REG_AVI_INFO_BASE_ADDR);
	meson_txc_hdmi_disable_infoframe(priv, TX_PKT_REG_EXCEPT0_BASE_ADDR);
	meson_txc_hdmi_disable_infoframe(priv, TX_PKT_REG_VEND_INFO_BASE_ADDR);
}

static enum drm_mode_status
meson_txc_hdmi_bridge_mode_valid(struct drm_bridge *bridge,
				 const struct drm_display_info *info,
				 const struct drm_display_mode *mode)
{
	return MODE_OK;
}

static enum drm_connector_status meson_txc_hdmi_bridge_detect(struct drm_bridge *bridge)
{
	struct meson_txc_hdmi *priv = bridge_to_meson_txc_hdmi(bridge);
	enum drm_connector_status status;
	unsigned int val;

	regmap_read(priv->regmap, TX_HDCP_ST_EDID_STATUS, &val);
	if (val & TX_HDCP_ST_EDID_STATUS_HPD_STATUS)
		status = connector_status_connected;
	else
		status = connector_status_disconnected;

	mutex_lock(&priv->codec_mutex);
	if (priv->last_connector_status != status) {
		priv->last_connector_status = status;
		meson_txc_hdmi_handle_plugged_change(priv);
	}
	mutex_unlock(&priv->codec_mutex);

	return status;
}

static int meson_txc_hdmi_get_edid_block(void *data, u8 *buf, unsigned int block,
					 size_t len)
{
	unsigned int i, regval, start = block * EDID_LENGTH;
	struct meson_txc_hdmi *priv = data;
	int ret;

	/* Start the DDC transaction */
	regmap_update_bits(priv->regmap, TX_HDCP_EDID_CONFIG,
			   TX_HDCP_EDID_CONFIG_SYS_TRIGGER_CONFIG, 0);
	regmap_update_bits(priv->regmap, TX_HDCP_EDID_CONFIG,
			   TX_HDCP_EDID_CONFIG_SYS_TRIGGER_CONFIG,
			   TX_HDCP_EDID_CONFIG_SYS_TRIGGER_CONFIG);

	ret = regmap_read_poll_timeout(priv->regmap,
				       TX_HDCP_ST_EDID_STATUS,
				       regval,
				       (regval & TX_HDCP_ST_EDID_STATUS_EDID_DATA_READY),
				       1000, 200000);

	regmap_update_bits(priv->regmap, TX_HDCP_EDID_CONFIG,
			   TX_HDCP_EDID_CONFIG_SYS_TRIGGER_CONFIG, 0);

	if (ret)
		return ret;

	for (i = 0; i < len; i++) {
		regmap_read(priv->regmap, TX_RX_EDID_OFFSET + start + i,
			    &regval);
		buf[i] = regval;
	}

	return 0;
}

static struct edid *meson_txc_hdmi_bridge_get_edid(struct drm_bridge *bridge,
						   struct drm_connector *connector)
{
	struct meson_txc_hdmi *priv = bridge_to_meson_txc_hdmi(bridge);
	struct edid *edid;

	edid = drm_do_get_edid(connector, meson_txc_hdmi_get_edid_block, priv);
	if (!edid) {
		drm_dbg(priv->bridge.dev, "Failed to get EDID\n");
		return NULL;
	}

	priv->sink_is_hdmi = drm_detect_hdmi_monitor(edid);

	return edid;
}

static const struct drm_bridge_funcs meson_txc_hdmi_bridge_funcs = {
	.atomic_duplicate_state = drm_atomic_helper_bridge_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_bridge_destroy_state,
	.atomic_reset = drm_atomic_helper_bridge_reset,
	.attach = meson_txc_hdmi_bridge_attach,
	.atomic_get_output_bus_fmts = meson_txc_hdmi_bridge_atomic_get_output_bus_fmts,
	.atomic_get_input_bus_fmts = meson_txc_hdmi_bridge_atomic_get_input_bus_fmts,
	.atomic_enable = meson_txc_hdmi_bridge_atomic_enable,
	.atomic_disable = meson_txc_hdmi_bridge_atomic_disable,
	.mode_valid = meson_txc_hdmi_bridge_mode_valid,
	.detect = meson_txc_hdmi_bridge_detect,
	.get_edid = meson_txc_hdmi_bridge_get_edid,
};

static int meson_txc_hdmi_parse_dt(struct meson_txc_hdmi *priv)
{
	struct device_node *endpoint, *remote;

	endpoint = of_graph_get_endpoint_by_regs(priv->dev->of_node, 1, -1);
	if (!endpoint) {
		dev_err(priv->dev, "Missing endpoint in port@1\n");
		return -ENODEV;
	}

	remote = of_graph_get_remote_port_parent(endpoint);
	of_node_put(endpoint);
	if (!remote) {
		dev_err(priv->dev, "Endpoint in port@1 unconnected\n");
		return -ENODEV;
	}

	if (!of_device_is_available(remote)) {
		dev_err(priv->dev, "port@1 remote device is disabled\n");
		of_node_put(remote);
		return -ENODEV;
	}

	priv->next_bridge = of_drm_find_bridge(remote);
	of_node_put(remote);
	if (!priv->next_bridge)
		return -EPROBE_DEFER;

	return 0;
}

static int meson_txc_hdmi_hw_init(struct meson_txc_hdmi *priv)
{
	unsigned long ddc_i2c_bus_clk_hz = 500 * 1000;
	unsigned long sys_clk_hz = 24 * 1000 * 1000;
	int ret;

	ret = phy_init(priv->phy);
	if (ret) {
		dev_err(priv->dev, "Failed to initialize the PHY: %d\n", ret);
		return ret;
	}

	ret = clk_set_rate(priv->sys_clk, sys_clk_hz);
	if (ret) {
		dev_err(priv->dev, "Failed to set HDMI system clock to 24MHz\n");
		goto err_phy_exit;
	}

	ret = clk_prepare_enable(priv->sys_clk);
	if (ret) {
		dev_err(priv->dev, "Failed to enable the sys clk\n");
		goto err_phy_exit;
	}

	regmap_update_bits(priv->regmap, HDMI_OTHER_CTRL1,
			   HDMI_OTHER_CTRL1_POWER_ON,
			   HDMI_OTHER_CTRL1_POWER_ON);

	regmap_write(priv->regmap, TX_HDMI_PHY_CONFIG0,
		     TX_HDMI_PHY_CONFIG0_HDMI_COMMON_B7_B0);

	regmap_write(priv->regmap, TX_HDCP_MODE, 0x40);

	/*
	 * The vendor driver comments that this is a setting for "Band-gap and
	 * main-bias". 0x1d = power-up, 0x00 = power-down.
	 */
	regmap_write(priv->regmap, TX_SYS1_AFE_TEST, 0x1d);

	meson_txc_hdmi_config_serializer_clock(priv, HDMI_COLORIMETRY_NONE);

	/*
	 * The vendor driver has a comment with the following information for
	 * the magic value:
	 * bit[2:0]=011: CK channel output TMDS CLOCK
	 * bit[2:0]=101, ck channel output PHYCLCK
	 */
	regmap_write(priv->regmap, TX_SYS1_AFE_CONNECT, 0xfb);

	/* Termination resistor calib value */
	regmap_write(priv->regmap, TX_CORE_CALIB_VALUE, 0x0f);

	/* HPD glitch filter */
	regmap_write(priv->regmap, TX_HDCP_HPD_FILTER_L, 0xa0);
	regmap_write(priv->regmap, TX_HDCP_HPD_FILTER_H, 0xa0);

	/* Disable MEM power-down */
	regmap_write(priv->regmap, TX_MEM_PD_REG0, 0x0);

	regmap_write(priv->regmap, TX_HDCP_CONFIG3,
		     FIELD_PREP(TX_HDCP_CONFIG3_DDC_I2C_BUS_CLOCK_TIME_DIVIDER,
				(sys_clk_hz / ddc_i2c_bus_clk_hz) - 1));

	/* Enable software controlled DDC transaction */
	regmap_write(priv->regmap, TX_HDCP_EDID_CONFIG,
		     TX_HDCP_EDID_CONFIG_FORCED_MEM_COPY_DONE |
		     TX_HDCP_EDID_CONFIG_MEM_COPY_DONE_CONFIG);
	regmap_write(priv->regmap, TX_CORE_EDID_CONFIG_MORE,
		     TX_CORE_EDID_CONFIG_MORE_SYS_TRIGGER_CONFIG_SEMI_MANU);

	/* mask (= disable) all interrupts */
	regmap_write(priv->regmap, HDMI_OTHER_INTR_MASKN, 0x0);

	/* clear any pending interrupt */
	regmap_write(priv->regmap, HDMI_OTHER_INTR_STAT_CLR,
		     HDMI_OTHER_INTR_STAT_CLR_EDID_RISING |
		     HDMI_OTHER_INTR_STAT_CLR_HPD_FALLING |
		     HDMI_OTHER_INTR_STAT_CLR_HPD_RISING);

	return 0;

err_phy_exit:
	phy_exit(priv->phy);
	return 0;
}

static void meson_txc_hdmi_hw_exit(struct meson_txc_hdmi *priv)
{
	int ret;

	/* mask (= disable) all interrupts */
	regmap_write(priv->regmap, HDMI_OTHER_INTR_MASKN,
		     HDMI_OTHER_INTR_MASKN_TX_EDID_INT_RISE |
		     HDMI_OTHER_INTR_MASKN_TX_HPD_INT_FALL |
		     HDMI_OTHER_INTR_MASKN_TX_HPD_INT_RISE);

	regmap_update_bits(priv->regmap, HDMI_OTHER_CTRL1,
			   HDMI_OTHER_CTRL1_POWER_ON, 0);

	clk_disable_unprepare(priv->sys_clk);

	ret = phy_exit(priv->phy);
	if (ret)
		dev_err(priv->dev, "Failed to exit the PHY: %d\n", ret);
}

static u32 meson_txc_hdmi_hdmi_codec_calc_audio_n(struct hdmi_codec_params *hparms)
{
	u32 audio_n;

	if ((hparms->sample_rate % 44100) == 0)
		audio_n = (128 * hparms->sample_rate) / 900;
	else
		audio_n = (128 * hparms->sample_rate) / 1000;

	if (hparms->cea.coding_type == HDMI_AUDIO_CODING_TYPE_EAC3 ||
	    hparms->cea.coding_type == HDMI_AUDIO_CODING_TYPE_DTS_HD)
		audio_n *= 4;

	return audio_n;
}

static u8 meson_txc_hdmi_hdmi_codec_coding_type(struct hdmi_codec_params *hparms)
{
	switch (hparms->cea.coding_type) {
	case HDMI_AUDIO_CODING_TYPE_MLP:
		return TX_AUDIO_CONTROL_AUDIO_PACKET_TYPE_HBR_AUDIO_PACKET;
	case HDMI_AUDIO_CODING_TYPE_DSD:
		return TX_AUDIO_CONTROL_AUDIO_PACKET_TYPE_ONE_BIT_AUDIO;
	case HDMI_AUDIO_CODING_TYPE_DST:
		return TX_AUDIO_CONTROL_AUDIO_PACKET_TYPE_DST_AUDIO_PACKET;
	default:
		return TX_AUDIO_CONTROL_AUDIO_PACKET_TYPE_AUDIO_SAMPLE_PACKET;
	}
}

static int meson_txc_hdmi_hdmi_codec_hw_params(struct device *dev, void *data,
					       struct hdmi_codec_daifmt *fmt,
					       struct hdmi_codec_params *hparms)
{
	u8 buf[HDMI_INFOFRAME_SIZE(AUDIO)];
	struct meson_txc_hdmi *priv = data;
	u16 audio_tx_format;
	u32 audio_n;
	int len, i;

	if (hparms->cea.coding_type == HDMI_AUDIO_CODING_TYPE_MLP) {
		/*
		 * TODO: fixed CTS is not supported yet, it needs special
		 * TX_SYS1_ACR_N_* settings
		 */
		return -EINVAL;
	}

	switch (hparms->sample_width) {
	case 16:
		audio_tx_format = FIELD_PREP(TX_AUDIO_FORMAT_BIT_WIDTH_MASK,
					     TX_AUDIO_FORMAT_BIT_WIDTH_16);
		break;

	case 20:
		audio_tx_format = FIELD_PREP(TX_AUDIO_FORMAT_BIT_WIDTH_MASK,
					     TX_AUDIO_FORMAT_BIT_WIDTH_20);
		break;

	case 24:
		audio_tx_format = FIELD_PREP(TX_AUDIO_FORMAT_BIT_WIDTH_MASK,
					     TX_AUDIO_FORMAT_BIT_WIDTH_24);
		break;

	default:
		return -EINVAL;
	}

	switch (fmt->fmt) {
	case HDMI_I2S:
		regmap_update_bits(priv->regmap, HDMI_OTHER_CTRL1,
				   HDMI_OTHER_CTRL1_HDMI_AUDIO_CLOCK_ON,
				   HDMI_OTHER_CTRL1_HDMI_AUDIO_CLOCK_ON);

		audio_tx_format |= TX_AUDIO_FORMAT_SPDIF_OR_I2S |
				   TX_AUDIO_FORMAT_I2S_ONE_BIT_OR_I2S |
				   FIELD_PREP(TX_AUDIO_FORMAT_I2S_FORMAT, 0x2);

		if (hparms->channels > 2)
			audio_tx_format |= TX_AUDIO_FORMAT_I2S_2_OR_8_CH;

		regmap_write(priv->regmap, TX_AUDIO_FORMAT, audio_tx_format);

		regmap_write(priv->regmap, TX_AUDIO_I2S, TX_AUDIO_I2S_ENABLE);
		regmap_write(priv->regmap, TX_AUDIO_SPDIF, 0x0);
		break;

	case HDMI_SPDIF:
		regmap_update_bits(priv->regmap, HDMI_OTHER_CTRL1,
				   HDMI_OTHER_CTRL1_HDMI_AUDIO_CLOCK_ON, 0x0);

		if (hparms->cea.coding_type == HDMI_AUDIO_CODING_TYPE_STREAM)
			audio_tx_format |= TX_AUDIO_FORMAT_SPDIF_CHANNEL_STATUS_FROM_DATA_OR_REG;

		regmap_write(priv->regmap, TX_AUDIO_FORMAT, audio_tx_format);

		regmap_write(priv->regmap, TX_AUDIO_I2S, 0x0);
		regmap_write(priv->regmap, TX_AUDIO_SPDIF, TX_AUDIO_SPDIF_ENABLE);
		break;

	default:
		return -EINVAL;
	}

	if (hparms->channels > 2)
		regmap_write(priv->regmap, TX_AUDIO_HEADER,
			     TX_AUDIO_HEADER_AUDIO_SAMPLE_PACKET_HEADER_LAYOUT1);
	else
		regmap_write(priv->regmap, TX_AUDIO_HEADER, 0x0);

	regmap_write(priv->regmap, TX_AUDIO_SAMPLE,
		     FIELD_PREP(TX_AUDIO_SAMPLE_CHANNEL_VALID,
				BIT(hparms->channels) - 1));

	audio_n = meson_txc_hdmi_hdmi_codec_calc_audio_n(hparms);

	regmap_write(priv->regmap, TX_SYS1_ACR_N_0,
		     FIELD_PREP(TX_SYS1_ACR_N_0_N_BYTE0,
				(audio_n >> 0) & 0xff));
	regmap_write(priv->regmap, TX_SYS1_ACR_N_1,
		     FIELD_PREP(TX_SYS1_ACR_N_1_N_BYTE1,
				(audio_n >> 8) & 0xff));
	regmap_update_bits(priv->regmap, TX_SYS1_ACR_N_2,
			   TX_SYS1_ACR_N_2_N_UPPER_NIBBLE,
			   FIELD_PREP(TX_SYS1_ACR_N_2_N_UPPER_NIBBLE,
				      (audio_n >> 16) & 0xf));

	regmap_write(priv->regmap, TX_SYS0_ACR_CTS_0, 0x0);
	regmap_write(priv->regmap, TX_SYS0_ACR_CTS_1, 0x0);
	regmap_write(priv->regmap, TX_SYS0_ACR_CTS_2,
		     TX_SYS0_ACR_CTS_2_FORCE_ARC_STABLE);

	regmap_write(priv->regmap, TX_AUDIO_CONTROL,
		     TX_AUDIO_CONTROL_AUTO_AUDIO_FIFO_CLEAR |
		     FIELD_PREP(TX_AUDIO_CONTROL_AUDIO_PACKET_TYPE_MASK,
				meson_txc_hdmi_hdmi_codec_coding_type(hparms)) |
		     TX_AUDIO_CONTROL_AUDIO_SAMPLE_PACKET_FLAT);

	len = hdmi_audio_infoframe_pack(&hparms->cea, buf, sizeof(buf));
	if (len < 0)
		return len;

	meson_txc_hdmi_write_infoframe(priv->regmap,
				       TX_PKT_REG_AUDIO_INFO_BASE_ADDR,
				       buf, len, true);

	for (i = 0; i < ARRAY_SIZE(hparms->iec.status); i++) {
		unsigned char sub1, sub2;

		sub1 = sub2 = hparms->iec.status[i];

		if (i == 2) {
			sub1 |= FIELD_PREP(IEC958_AES2_CON_CHANNEL, 1);
			sub2 |= FIELD_PREP(IEC958_AES2_CON_CHANNEL, 2);
		}

		regmap_write(priv->regmap, TX_IEC60958_SUB1_OFFSET + i, sub1);
		regmap_write(priv->regmap, TX_IEC60958_SUB2_OFFSET + i, sub2);
	}

	return 0;
}

static int meson_txc_hdmi_hdmi_codec_audio_startup(struct device *dev,
						   void *data)
{
	struct meson_txc_hdmi *priv = data;

	regmap_update_bits(priv->regmap, TX_PACKET_CONTROL_2,
			   TX_PACKET_CONTROL_2_AUDIO_REQUEST_DISABLE, 0x0);

	/* reset audio master and sample */
	regmap_write(priv->regmap, TX_SYS5_TX_SOFT_RESET_1,
		     TX_SYS5_TX_SOFT_RESET_1_TX_AUDIO_RESAMPLE_RSTN |
		     TX_SYS5_TX_SOFT_RESET_1_TX_AUDIO_MASTER_RSTN);
	regmap_write(priv->regmap, TX_SYS5_TX_SOFT_RESET_1, 0x0);

	regmap_write(priv->regmap, TX_AUDIO_CONTROL_MORE,
		     TX_AUDIO_CONTROL_MORE_ENABLE);

	regmap_write(priv->regmap, TX_AUDIO_FIFO,
		     FIELD_PREP(TX_AUDIO_FIFO_FIFO_DEPTH_MASK,
				TX_AUDIO_FIFO_FIFO_DEPTH_512) |
		     FIELD_PREP(TX_AUDIO_FIFO_CRITICAL_THRESHOLD_MASK,
			        TX_AUDIO_FIFO_CRITICAL_THRESHOLD_DEPTH_DIV16) |
		     FIELD_PREP(TX_AUDIO_FIFO_NORMAL_THRESHOLD_MASK,
			        TX_AUDIO_FIFO_NORMAL_THRESHOLD_DEPTH_DIV8));

	regmap_write(priv->regmap, TX_AUDIO_LIPSYNC, 0x0);

	regmap_write(priv->regmap, TX_SYS1_ACR_N_2,
		     FIELD_PREP(TX_SYS1_ACR_N_2_N_MEAS_TOLERANCE, 0x3));

	return 0;
}

static void meson_txc_hdmi_hdmi_codec_audio_shutdown(struct device *dev,
						     void *data)
{
	struct meson_txc_hdmi *priv = data;

	meson_txc_hdmi_disable_infoframe(priv, TX_PKT_REG_AUDIO_INFO_BASE_ADDR);

	regmap_write(priv->regmap, TX_AUDIO_CONTROL_MORE, 0x0);
	regmap_update_bits(priv->regmap, HDMI_OTHER_CTRL1,
			   HDMI_OTHER_CTRL1_HDMI_AUDIO_CLOCK_ON, 0x0);

	regmap_update_bits(priv->regmap, TX_PACKET_CONTROL_2,
			   TX_PACKET_CONTROL_2_AUDIO_REQUEST_DISABLE,
			   TX_PACKET_CONTROL_2_AUDIO_REQUEST_DISABLE);
}

static int meson_txc_hdmi_hdmi_codec_mute_stream(struct device *dev,
						 void *data,
						 bool enable, int direction)
{
	struct meson_txc_hdmi *priv = data;

	regmap_write(priv->regmap, TX_AUDIO_PACK,
		     enable ? 0 : TX_AUDIO_PACK_AUDIO_SAMPLE_PACKETS_ENABLE);

	return 0;
}

static int meson_txc_hdmi_hdmi_codec_get_eld(struct device *dev, void *data,
					     uint8_t *buf, size_t len)
{
	struct meson_txc_hdmi *priv = data;

	if (priv->current_connector)
		memcpy(buf, priv->current_connector->eld,
		       min_t(size_t, MAX_ELD_BYTES, len));
	else
		memset(buf, 0, len);

	return 0;
}

static int meson_txc_hdmi_hdmi_codec_get_dai_id(struct snd_soc_component *component,
						struct device_node *endpoint)
{
	struct of_endpoint of_ep;
	int ret;

	ret = of_graph_parse_endpoint(endpoint, &of_ep);
	if (ret < 0)
		return ret;

	/*
	 * HDMI sound should be located as reg = <2>
	 * Then, it is sound port 0
	 */
	if (of_ep.port == 2)
		return 0;

	return -EINVAL;
}

static int meson_txc_hdmi_hdmi_codec_hook_plugged_cb(struct device *dev,
						     void *data,
						     hdmi_codec_plugged_cb fn,
						     struct device *codec_dev)
{
	struct meson_txc_hdmi *priv = data;

	mutex_lock(&priv->codec_mutex);
	priv->codec_plugged_cb = fn;
	priv->codec_dev = codec_dev;
	meson_txc_hdmi_handle_plugged_change(priv);
	mutex_unlock(&priv->codec_mutex);

	return 0;
}

static struct hdmi_codec_ops meson_txc_hdmi_hdmi_codec_ops = {
	.hw_params		= meson_txc_hdmi_hdmi_codec_hw_params,
	.audio_startup		= meson_txc_hdmi_hdmi_codec_audio_startup,
	.audio_shutdown		= meson_txc_hdmi_hdmi_codec_audio_shutdown,
	.mute_stream		= meson_txc_hdmi_hdmi_codec_mute_stream,
	.get_eld		= meson_txc_hdmi_hdmi_codec_get_eld,
	.get_dai_id		= meson_txc_hdmi_hdmi_codec_get_dai_id,
	.hook_plugged_cb	= meson_txc_hdmi_hdmi_codec_hook_plugged_cb,
};

static int meson_txc_hdmi_hdmi_codec_init(struct meson_txc_hdmi *priv)
{
	struct hdmi_codec_pdata pdata = {
		.ops			= &meson_txc_hdmi_hdmi_codec_ops,
		.i2s			= 1,
		.spdif			= 1,
		.max_i2s_channels	= 8,
		.data			= priv,
	};

	priv->hdmi_codec_pdev = platform_device_register_data(priv->dev,
							      HDMI_CODEC_DRV_NAME,
							      PLATFORM_DEVID_AUTO,
							      &pdata, sizeof(pdata));
	return PTR_ERR_OR_ZERO(priv->hdmi_codec_pdev);
}

static int meson_txc_hdmi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct meson_txc_hdmi *priv;
	void __iomem *base;
	u32 regval;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;

	mutex_init(&priv->codec_mutex);

	platform_set_drvdata(pdev, priv);

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	priv->regmap = devm_regmap_init(dev, NULL, base,
					&meson_txc_hdmi_regmap_config);
	if (IS_ERR(priv->regmap))
		return PTR_ERR(priv->regmap);

	priv->pclk = devm_clk_get(dev, "pclk");
	if (IS_ERR(priv->pclk)) {
		ret = PTR_ERR(priv->pclk);
		return dev_err_probe(dev, ret, "Failed to get the pclk\n");
	}

	priv->sys_clk = devm_clk_get(dev, "sys");
	if (IS_ERR(priv->sys_clk)) {
		ret = PTR_ERR(priv->sys_clk);
		return dev_err_probe(dev, ret,
				     "Failed to get the sys clock\n");
	}

	priv->phy = devm_phy_get(dev, "hdmi");
	if (IS_ERR(priv->phy)) {
		ret = PTR_ERR(priv->phy);
		return dev_err_probe(dev, ret, "Failed to get the HDMI PHY\n");
	}

	ret = meson_txc_hdmi_parse_dt(priv);
	if (ret)
		return ret;

	ret = clk_prepare_enable(priv->pclk);
	if (ret) {
		dev_err_probe(dev, ret, "Failed to enable the pclk\n");
		return ret;
	}

	regval = readl(base + HDMI_CTRL_PORT);
	regval |= HDMI_CTRL_PORT_APB3_ERR_EN;
	writel(regval, base + HDMI_CTRL_PORT);

	ret = meson_txc_hdmi_hw_init(priv);
	if (ret)
		goto err_disable_clk;

	ret = meson_txc_hdmi_hdmi_codec_init(priv);
	if (ret)
		goto err_hw_exit;

	priv->bridge.driver_private = priv;
	priv->bridge.funcs = &meson_txc_hdmi_bridge_funcs;
	priv->bridge.ops = DRM_BRIDGE_OP_DETECT | DRM_BRIDGE_OP_EDID;
	priv->bridge.of_node = dev->of_node;
	priv->bridge.interlace_allowed = true;

	drm_bridge_add(&priv->bridge);

	return 0;

err_hw_exit:
	meson_txc_hdmi_hw_exit(priv);
err_disable_clk:
	clk_disable_unprepare(priv->pclk);
	return ret;
}

static void meson_txc_hdmi_remove(struct platform_device *pdev)
{
	struct meson_txc_hdmi *priv = platform_get_drvdata(pdev);

	platform_device_unregister(priv->hdmi_codec_pdev);

	drm_bridge_remove(&priv->bridge);

	meson_txc_hdmi_hw_exit(priv);

	clk_disable_unprepare(priv->pclk);
}

static const struct of_device_id meson_txc_hdmi_of_table[] = {
	{ .compatible = "amlogic,meson8-hdmi-tx" },
	{ .compatible = "amlogic,meson8b-hdmi-tx" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, meson_txc_hdmi_of_table);

static struct platform_driver meson_txc_hdmi_platform_driver = {
	.probe		= meson_txc_hdmi_probe,
	.remove_new	= meson_txc_hdmi_remove,
	.driver		= {
		.name		= "meson-transwitch-hdmi",
		.of_match_table	= meson_txc_hdmi_of_table,
	},
};
module_platform_driver(meson_txc_hdmi_platform_driver);

MODULE_AUTHOR("Martin Blumenstingl <martin.blumenstingl@googlemail.com>");
MODULE_DESCRIPTION("Amlogic Meson8 and Meson8b TranSwitch HDMI 1.4 TX driver");
MODULE_LICENSE("GPL v2");
