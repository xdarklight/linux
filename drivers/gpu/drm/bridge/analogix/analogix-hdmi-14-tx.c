// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2020 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 *
 * All registers and magic values are taken from Amlogic's GPL kernel sources:
 *   Copyright (C) 2010 Amlogic, Inc.
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include <drm/bridge/analogix_hdmi_14_tx.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_device.h>
#include <drm/drm_edid.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_print.h>

#include <media/cec-notifier.h>

#include <uapi/linux/media-bus-format.h>
#include <uapi/linux/videodev2.h>

#include "analogix-hdmi-14-tx.h"

struct anx_hdmi_14_tx {
	struct device			*dev;
	struct regmap			*regmap;
	struct clk			*sys_clk;

	struct mutex			cec_notifier_mutex;
	struct cec_notifier		*cec_notifier;

	struct drm_connector		connector;
	struct drm_bridge		bridge;

	int				cea_mode;
	enum hdmi_colorimetry		colorimetry;
	unsigned int			input_bus_format;
	unsigned int			output_bus_format;
	bool				limited_rgb_quant_range;
	bool				sink_is_hdmi;
	bool				sink_has_audio;
};

#define to_anx_hdmi_14_tx(x) container_of(x, struct anx_hdmi_14_tx, x)

static void anx_hdmi_14_tx_write_infoframe(struct anx_hdmi_14_tx *priv,
					   unsigned int tx_pkt_reg,
					   u8 *buf, unsigned int len,
					   bool enable)
{
	unsigned int i;

	/* write the payload starting register offset 1 and skip the header */
	for (i = HDMI_INFOFRAME_HEADER_SIZE; i < len; i++)
		regmap_write(priv->regmap,
			     tx_pkt_reg + i - HDMI_INFOFRAME_HEADER_SIZE + 1,
			     buf[i]);

	/* zero the remaining payload bytes */
	for (; i < 0x1c; i++)
		regmap_write(priv->regmap, tx_pkt_reg + i, 0x00);

	/* write the header */
	regmap_write(priv->regmap, tx_pkt_reg + 0x00, buf[3]);
	regmap_write(priv->regmap, tx_pkt_reg + 0x1c, buf[0]);
	regmap_write(priv->regmap, tx_pkt_reg + 0x1d, buf[1]);
	regmap_write(priv->regmap, tx_pkt_reg + 0x1e, buf[2]);

	regmap_write(priv->regmap, tx_pkt_reg + 0x1f, enable ? 0xff : 0x00);
}

static void anx_hdmi_14_tx_disable_infoframe(struct anx_hdmi_14_tx *priv,
					     unsigned int tx_pkt_reg)
{
	u8 buf[HDMI_INFOFRAME_HEADER_SIZE] = { 0 };

	anx_hdmi_14_tx_write_infoframe(priv, tx_pkt_reg, buf,
				       HDMI_INFOFRAME_HEADER_SIZE, false);
}

static void anx_hdmi_14_tx_sys5_reset_assert(struct anx_hdmi_14_tx *priv)
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

static void anx_hdmi_14_tx_sys5_reset_deassert(struct anx_hdmi_14_tx *priv)
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

static void anx_hdmi_14_tx_config_hdcp_registers(struct anx_hdmi_14_tx *priv)
{
	regmap_write(priv->regmap, TX_HDCP_CONFIG0,
		     FIELD_PREP(TX_HDCP_CONFIG0_ROM_ENCRYPT_OFF, 0x3));
	regmap_write(priv->regmap, TX_HDCP_MEM_CONFIG, 0x0);
	regmap_write(priv->regmap, TX_HDCP_ENCRYPT_BYTE, 0x0);

	regmap_write(priv->regmap, TX_HDCP_MODE, TX_HDCP_MODE_CLEAR_AVMUTE);

	regmap_write(priv->regmap, TX_HDCP_MODE, TX_HDCP_MODE_ESS_CONFIG);
}

static u8 anx_hdmi_14_tx_bus_fmt_color_depth(unsigned int bus_format)
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
anx_hdmi_14_tx_bus_fmt_hdmi_colorspace(unsigned int bus_format)
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

static u8 anx_hdmi_14_tx_bus_fmt_color_format(unsigned int bus_format)
{
	switch (anx_hdmi_14_tx_bus_fmt_hdmi_colorspace(bus_format)) {
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

static void anx_hdmi_14_tx_config_color_space(struct anx_hdmi_14_tx *priv)
{
	unsigned int regval;

	regmap_write(priv->regmap, TX_VIDEO_DTV_MODE,
		     FIELD_PREP(TX_VIDEO_DTV_MODE_COLOR_DEPTH,
				anx_hdmi_14_tx_bus_fmt_color_depth(
					priv->output_bus_format)));

	regmap_write(priv->regmap, TX_VIDEO_DTV_OPTION_L,
		     FIELD_PREP(TX_VIDEO_DTV_OPTION_L_OUTPUT_COLOR_FORMAT,
				anx_hdmi_14_tx_bus_fmt_color_format(
					priv->output_bus_format)) |
		     FIELD_PREP(TX_VIDEO_DTV_OPTION_L_INPUT_COLOR_FORMAT,
				anx_hdmi_14_tx_bus_fmt_color_format(
					priv->input_bus_format)) |
		     FIELD_PREP(TX_VIDEO_DTV_OPTION_L_OUTPUT_COLOR_DEPTH,
				anx_hdmi_14_tx_bus_fmt_color_depth(
					priv->output_bus_format)) |
		     FIELD_PREP(TX_VIDEO_DTV_OPTION_L_INPUT_COLOR_DEPTH,
				anx_hdmi_14_tx_bus_fmt_color_depth(
					priv->input_bus_format)));

	if (priv->limited_rgb_quant_range)
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

	if (priv->colorimetry == HDMI_COLORIMETRY_ITU_601) {
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

static void anx_hdmi_14_tx_config_serializer_clock(struct anx_hdmi_14_tx *priv)
{
	/* Serializer Internal clock setting */
	if (priv->colorimetry == HDMI_COLORIMETRY_ITU_601)
		regmap_write(priv->regmap, 0x0018, 0x24);
	else
		regmap_write(priv->regmap, 0x0018, 0x22);

#if 0
	// TODO: not ported yet
	if ((param->VIC==HDMI_1080p60)&&(param->color_depth==COLOR_30BIT)&&(hdmi_rd_reg(0x018)==0x22)) {
		regmap_write(priv->regmap, 0x0018,0x12);
	}
#endif
}

static void anx_hdmi_14_tx_reconfig_packet_setting(struct anx_hdmi_14_tx *priv)
{
	unsigned int alloc_active2, alloc_eof[2], alloc_vsync[3], alloc_sof[2];

	switch (priv->cea_mode) {
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
	regmap_write(priv->regmap, TX_PACKET_CONTROL_1,
		     TX_PACKET_CONTROL_1_PACKET_ALLOC_MODE |
		     FIELD_PREP(TX_PACKET_CONTROL_1_PACKET_START_LATENCY,
				26));
	regmap_write(priv->regmap, TX_PACKET_ALLOC_ACTIVE_1, 0x01);
	regmap_write(priv->regmap, TX_PACKET_ALLOC_ACTIVE_2, alloc_active2);
	regmap_write(priv->regmap, TX_PACKET_ALLOC_EOF_1, alloc_eof[0]);
	regmap_write(priv->regmap, TX_PACKET_ALLOC_EOF_2, alloc_eof[1]);
	regmap_write(priv->regmap, TX_CORE_ALLOC_VSYNC_0, alloc_vsync[0]);
	regmap_write(priv->regmap, TX_CORE_ALLOC_VSYNC_1, alloc_vsync[1]);
	regmap_write(priv->regmap, TX_CORE_ALLOC_VSYNC_2, alloc_vsync[2]);
	regmap_write(priv->regmap, TX_PACKET_ALLOC_SOF_1, alloc_sof[0]);
	regmap_write(priv->regmap, TX_PACKET_ALLOC_SOF_2, alloc_sof[1]);
	regmap_write(priv->regmap, TX_PACKET_CONTROL_1,
		     TX_PACKET_CONTROL_1_FORCE_PACKET_TIMING |
		     FIELD_PREP(TX_PACKET_CONTROL_1_PACKET_START_LATENCY,
				58));
}

static void anx_hdmi_14_tx_set_avi_infoframe(struct anx_hdmi_14_tx *priv,
					     const struct drm_display_mode *mode)
{
	u8 buf[HDMI_INFOFRAME_SIZE(AVI)], *video_code;
	struct hdmi_avi_infoframe frame;
	int ret;

	ret = drm_hdmi_avi_infoframe_from_display_mode(&frame,
						       &priv->connector, mode);
	if (ret < 0) {
		drm_err(priv->bridge.dev,
			"Failed to setup AVI infoframe: %d\n", ret);
		return;
	}

	/* fixed infoframe configuration not linked to the mode */
	frame.colorspace =
		anx_hdmi_14_tx_bus_fmt_hdmi_colorspace(priv->output_bus_format);
	frame.colorimetry = priv->colorimetry;

	drm_hdmi_avi_infoframe_quant_range(&frame,
					   &priv->connector, mode,
					   priv->limited_rgb_quant_range ?
					   HDMI_QUANTIZATION_RANGE_LIMITED :
					   HDMI_QUANTIZATION_RANGE_FULL);

	ret = hdmi_avi_infoframe_pack(&frame, buf, sizeof(buf));
	if (ret < 0) {
		drm_err(priv->bridge.dev,
			"Failed to pack AVI infoframe: %d\n", ret);
		return;
	}

	video_code = &buf[HDMI_INFOFRAME_HEADER_SIZE + 3];
	if (*video_code > 109) {
		regmap_write(priv->regmap, TX_PKT_REG_EXCEPT0_BASE_ADDR,
			     *video_code);
		*video_code = 0x00;
	} else {
		regmap_write(priv->regmap, TX_PKT_REG_EXCEPT0_BASE_ADDR, 0x00);
	}

	anx_hdmi_14_tx_write_infoframe(priv, TX_PKT_REG_AVI_INFO_BASE_ADDR,
				       buf, sizeof(buf), true);
}

static void anx_hdmi_14_tx_set_vendor_infoframe(struct anx_hdmi_14_tx *priv,
						const struct drm_display_mode *mode)
{
	u8 buf[HDMI_INFOFRAME_HEADER_SIZE + 6];
	struct hdmi_vendor_infoframe frame;
	int ret;

	ret = drm_hdmi_vendor_infoframe_from_display_mode(&frame,
							  &priv->connector,
							  mode);
	if (ret) {
		drm_err(priv->bridge.dev,
			"Failed to setup vendor infoframe: %d\n", ret);
		return;
	}

	ret = hdmi_vendor_infoframe_pack(&frame, buf, sizeof(buf));
	if (ret < 0) {
		drm_err(priv->bridge.dev,
			"Failed to pack vendor infoframe: %d\n", ret);
		return;
	}

	anx_hdmi_14_tx_write_infoframe(priv, TX_PKT_REG_VEND_INFO_BASE_ADDR,
				       buf, sizeof(buf), true);
}

static void anx_hdmi_14_tx_set_spd_infoframe(struct anx_hdmi_14_tx *priv)
{
	u8 buf[HDMI_INFOFRAME_SIZE(SPD)];
	struct hdmi_spd_infoframe frame;
	int ret;

	ret = hdmi_spd_infoframe_init(&frame, "Analogix", "HDMI 1.4 TX IP");
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

	anx_hdmi_14_tx_write_infoframe(priv, TX_PKT_REG_SPD_INFO_BASE_ADDR,
				       buf, sizeof(buf), true);
}

static int anx_hdmi_14_tx_get_edid_block(void *data, u8 *buf,
					unsigned int block, size_t len)
{
	unsigned int i, regval, start = block * EDID_LENGTH;
	struct anx_hdmi_14_tx *priv = data;
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

static int anx_hdmi_14_tx_connector_get_modes(struct drm_connector *connector)
{
	struct anx_hdmi_14_tx *priv = to_anx_hdmi_14_tx(connector);
	struct edid *edid;
	int ret;

	edid = drm_do_get_edid(connector, anx_hdmi_14_tx_get_edid_block, priv);
	if (!edid)
		return 0;

	priv->sink_is_hdmi = drm_detect_hdmi_monitor(edid);
	priv->sink_has_audio = drm_detect_monitor_audio(edid);

	drm_connector_update_edid_property(connector, edid);

	mutex_lock(&priv->cec_notifier_mutex);
	cec_notifier_set_phys_addr_from_edid(priv->cec_notifier, edid);
	mutex_unlock(&priv->cec_notifier_mutex);

	ret = drm_add_edid_modes(connector, edid);

	kfree(edid);

	return ret;
}

static struct drm_connector_helper_funcs anx_hdmi_14_tx_connector_helper_funcs = {
	.get_modes = anx_hdmi_14_tx_connector_get_modes,
};

static enum drm_connector_status
anx_hdmi_14_tx_connector_detect(struct drm_connector *connector, bool force)
{
	struct anx_hdmi_14_tx *priv = to_anx_hdmi_14_tx(connector);
	enum drm_connector_status status;
	unsigned int val;

	regmap_read(priv->regmap, TX_HDCP_ST_EDID_STATUS, &val);
	if (val & TX_HDCP_ST_EDID_STATUS_HPD_STATUS)
		status = connector_status_connected;
	else
		status = connector_status_disconnected;

	if (status == connector_status_disconnected) {
		mutex_lock(&priv->cec_notifier_mutex);
		cec_notifier_phys_addr_invalidate(priv->cec_notifier);
		mutex_unlock(&priv->cec_notifier_mutex);
	}

	return status;
}

static const struct drm_connector_funcs anx_hdmi_14_tx_connector_funcs = {
	.fill_modes = drm_helper_probe_single_connector_modes,
	.detect = anx_hdmi_14_tx_connector_detect,
	.destroy = drm_connector_cleanup,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static int anx_hdmi_14_tx_bridge_attach(struct drm_bridge *bridge,
					enum drm_bridge_attach_flags flags)
{
	struct anx_hdmi_14_tx *priv = bridge->driver_private;
	struct drm_connector *connector = &priv->connector;
	struct drm_encoder *encoder = bridge->encoder;
	struct cec_connector_info conn_info;
	struct cec_notifier *notifier;
	int ret;

	if (flags & DRM_BRIDGE_ATTACH_NO_CONNECTOR) {
		drm_err(priv->bridge.dev,
			"Fix bridge driver to make connector optional!");
		return -EINVAL;
	}

	connector->interlace_allowed = 1;
	connector->polled = DRM_CONNECTOR_POLL_HPD;

	drm_connector_helper_add(connector,
				 &anx_hdmi_14_tx_connector_helper_funcs);

	ret = drm_connector_init(bridge->dev, &priv->connector,
				 &anx_hdmi_14_tx_connector_funcs,
				 DRM_MODE_CONNECTOR_HDMIA);
	if (ret)
		return ret;

	/*
	 * drm_connector_attach_max_bpc_property() requires the
	 * connector to have a state.
	 */
	drm_atomic_helper_connector_reset(connector);

	drm_connector_attach_max_bpc_property(connector, 8, 16);

	ret = drm_connector_attach_encoder(connector, encoder);
	if (ret)
		return ret;

	cec_fill_conn_info_from_drm(&conn_info, connector);
	notifier = cec_notifier_conn_register(priv->dev, NULL, &conn_info);
	if (!notifier)
		return -ENOMEM;

	mutex_lock(&priv->cec_notifier_mutex);
	priv->cec_notifier = notifier;
	mutex_unlock(&priv->cec_notifier_mutex);

	return 0;
}

static void anx_hdmi_14_tx_bridge_detach(struct drm_bridge *bridge)
{
	struct anx_hdmi_14_tx *priv = bridge->driver_private;

	mutex_lock(&priv->cec_notifier_mutex);
	cec_notifier_conn_unregister(priv->cec_notifier);
	priv->cec_notifier = NULL;
	mutex_unlock(&priv->cec_notifier_mutex);
}

static int
anx_hdmi_14_tx_bridge_atomic_check(struct drm_bridge *bridge,
				   struct drm_bridge_state *bridge_state,
				   struct drm_crtc_state *crtc_state,
				   struct drm_connector_state *conn_state)
{
	struct anx_hdmi_14_tx *priv = bridge->driver_private;

	priv->output_bus_format = bridge_state->output_bus_cfg.format;
	priv->input_bus_format = bridge_state->input_bus_cfg.format;

	drm_dbg(bridge->dev, "input format 0x%04x, output format 0x%04x\n",
		priv->output_bus_format, priv->input_bus_format);

	return 0;
}

/* Can return a maximum of 11 possible output formats for a mode/connector */
#define MAX_OUTPUT_SEL_FORMATS	11

static u32 *
anx_hdmi_14_tx_bridge_atomic_get_output_bus_fmts(struct drm_bridge *bridge,
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
		if (info->color_formats & DRM_COLOR_FORMAT_YCRCB444)
			output_fmts[i++] = MEDIA_BUS_FMT_YUV16_1X48;

		output_fmts[i++] = MEDIA_BUS_FMT_RGB161616_1X48;
	}

	if (max_bpc >= 12 && info->bpc >= 12) {
		if (info->color_formats & DRM_COLOR_FORMAT_YCRCB422)
			output_fmts[i++] = MEDIA_BUS_FMT_UYVY12_1X24;

		if (info->color_formats & DRM_COLOR_FORMAT_YCRCB444)
			output_fmts[i++] = MEDIA_BUS_FMT_YUV12_1X36;

		output_fmts[i++] = MEDIA_BUS_FMT_RGB121212_1X36;
	}

	if (max_bpc >= 10 && info->bpc >= 10) {
		if (info->color_formats & DRM_COLOR_FORMAT_YCRCB422)
			output_fmts[i++] = MEDIA_BUS_FMT_UYVY10_1X20;

		if (info->color_formats & DRM_COLOR_FORMAT_YCRCB444)
			output_fmts[i++] = MEDIA_BUS_FMT_YUV10_1X30;

		output_fmts[i++] = MEDIA_BUS_FMT_RGB101010_1X30;
	}

	if (info->color_formats & DRM_COLOR_FORMAT_YCRCB422)
		output_fmts[i++] = MEDIA_BUS_FMT_UYVY8_1X16;

	if (info->color_formats & DRM_COLOR_FORMAT_YCRCB444)
		output_fmts[i++] = MEDIA_BUS_FMT_YUV8_1X24;

	/* Default 8bit RGB fallback */
	output_fmts[i++] = MEDIA_BUS_FMT_RGB888_1X24;

	*num_output_fmts = i;

	return output_fmts;
}

/* Can return a maximum of 3 possible input formats for an output format */
#define MAX_INPUT_SEL_FORMATS	3

static u32 *
anx_hdmi_14_tx_bridge_atomic_get_input_bus_fmts(struct drm_bridge *bridge,
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
static void anx_hdmi_14_tx_bridge_enable(struct drm_bridge *bridge)
{
	struct anx_hdmi_14_tx *priv = to_anx_hdmi_14_tx(bridge);
	unsigned int i;

	anx_hdmi_14_tx_sys5_reset_assert(priv);

	regmap_write(priv->regmap, TX_PACKET_ALLOC_ACTIVE_1, 0x0);
	regmap_write(priv->regmap, TX_PACKET_CONTROL_1,
		     FIELD_PREP(TX_PACKET_CONTROL_1_PACKET_START_LATENCY, 58));
	regmap_write(priv->regmap, TX_PACKET_CONTROL_2,
		     TX_PACKET_CONTROL_2_HORIZONTAL_GC_PACKET_TRANSPORT_EN);

	anx_hdmi_14_tx_config_hdcp_registers(priv);

	regmap_write(priv->regmap, TX_AUDIO_CONTROL_MORE, 0x1);

	if (priv->cea_mode == 39)
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
			FIELD_PREP(TX_CORE_DATA_MONITOR_2_MONITOR_SELECT,
				   0x2));

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
	 * clock channel
	 */
	regmap_write(priv->regmap, TX_SYS4_CK_INV_VIDEO,
		     TX_SYS4_CK_INV_VIDEO_TMDS_CLK_PATTERN);

	regmap_write(priv->regmap, TX_SYS5_FIFO_CONFIG,
		     TX_SYS5_FIFO_CONFIG_CLK_CHANNEL3_OUTPUT_ENABLE |
		     TX_SYS5_FIFO_CONFIG_AFE_FIFO_CHANNEL2_ENABLE |
		     TX_SYS5_FIFO_CONFIG_AFE_FIFO_CHANNEL1_ENABLE |
		     TX_SYS5_FIFO_CONFIG_AFE_FIFO_CHANNEL0_ENABLE);

	anx_hdmi_14_tx_config_color_space(priv);

	anx_hdmi_14_tx_sys5_reset_deassert(priv);

	anx_hdmi_14_tx_config_serializer_clock(priv);
	anx_hdmi_14_tx_reconfig_packet_setting(priv);

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
}

static void anx_hdmi_14_tx_bridge_disable(struct drm_bridge *bridge)
{
	struct anx_hdmi_14_tx *priv = to_anx_hdmi_14_tx(bridge);

	anx_hdmi_14_tx_disable_infoframe(priv, TX_PKT_REG_AUDIO_INFO_BASE_ADDR);
	anx_hdmi_14_tx_disable_infoframe(priv, TX_PKT_REG_AVI_INFO_BASE_ADDR);
	anx_hdmi_14_tx_disable_infoframe(priv, TX_PKT_REG_EXCEPT0_BASE_ADDR);
	anx_hdmi_14_tx_disable_infoframe(priv, TX_PKT_REG_VEND_INFO_BASE_ADDR);
}

static void anx_hdmi_14_tx_bridge_mode_set(struct drm_bridge *bridge,
					   const struct drm_display_mode *mode,
					   const struct drm_display_mode *adj)
{
	struct anx_hdmi_14_tx *priv = to_anx_hdmi_14_tx(bridge);

	if (priv->input_bus_format = MEDIA_BUS_FMT_FIXED)
		priv->input_bus_format = MEDIA_BUS_FMT_RGB888_1X24;

	priv->cea_mode = drm_match_cea_mode(mode);

	if (priv->sink_is_hdmi) {
		enum hdmi_quantization_range quant_range;

		quant_range = drm_default_rgb_quant_range(mode);
		priv->limited_rgb_quant_range =
				quant_range == HDMI_QUANTIZATION_RANGE_LIMITED;

		switch (priv->cea_mode) {
		case 2 ... 3:
		case 6 ... 7:
		case 17 ... 18:
		case 21 ... 22:
			priv->colorimetry = HDMI_COLORIMETRY_ITU_601;
			break;

		default:
			priv->colorimetry = HDMI_COLORIMETRY_ITU_709;
			break;
		}

		anx_hdmi_14_tx_set_avi_infoframe(priv, mode);
		anx_hdmi_14_tx_set_vendor_infoframe(priv, mode);
		anx_hdmi_14_tx_set_spd_infoframe(priv);
	} else {
		priv->limited_rgb_quant_range = false;
		priv->colorimetry = HDMI_COLORIMETRY_NONE;
	}
}

static enum drm_mode_status
anx_hdmi_14_tx_bridge_mode_valid(struct drm_bridge *bridge,
				 const struct drm_display_mode *mode)
{
	return MODE_OK;
}

static const struct drm_bridge_funcs anx_hdmi_14_tx_bridge_funcs = {
	.atomic_duplicate_state = drm_atomic_helper_bridge_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_bridge_destroy_state,
	.atomic_reset = drm_atomic_helper_bridge_reset,
	.attach = anx_hdmi_14_tx_bridge_attach,
	.detach = anx_hdmi_14_tx_bridge_detach,
	.atomic_check = anx_hdmi_14_tx_bridge_atomic_check,
	.atomic_get_output_bus_fmts = anx_hdmi_14_tx_bridge_atomic_get_output_bus_fmts,
	.atomic_get_input_bus_fmts = anx_hdmi_14_tx_bridge_atomic_get_input_bus_fmts,
	.enable = anx_hdmi_14_tx_bridge_enable,
	.disable = anx_hdmi_14_tx_bridge_disable,
	.mode_set = anx_hdmi_14_tx_bridge_mode_set,
	.mode_valid = anx_hdmi_14_tx_bridge_mode_valid,
};

static int anx_hdmi_14_tx_init(struct anx_hdmi_14_tx *priv)
{
	unsigned long ddc_i2c_bus_clk_hz = 500 * 1000;
	unsigned long sys_clk_hz = 24 * 1000 * 1000;
	int ret;

	ret = clk_set_rate(priv->sys_clk, sys_clk_hz);
	if (ret) {
		dev_err(priv->dev,
			"Failed to set ANX HDMI system clock to 24MHz\n");
		return ret;
	}

	ret = clk_prepare_enable(priv->sys_clk);
	if (ret) {
		dev_err(priv->dev, "Failed to enable the ANX sys clk\n");
		return ret;
	}

	regmap_write(priv->regmap, 0x0010, 0xff);

	regmap_write(priv->regmap, TX_HDCP_MODE, 0x40);

	/* Band-gap and main-bias. 0x1d = power-up, 0x00 = power-down */
	regmap_write(priv->regmap, 0x0017, 0x1d);

	anx_hdmi_14_tx_config_serializer_clock(priv);

	/*
	 * bit[2:0]=011: CK channel output TMDS CLOCK
	 * bit[2:0]=101, ck channel output PHYCLCK
	 */
	regmap_write(priv->regmap, 0x001a, 0xfb);

	/* Termination resistor calib value */
	regmap_write(priv->regmap, TX_CORE_CALIB_VALUE, 0x0f);

	/* HPD glitch filter */
	regmap_write(priv->regmap, TX_HDCP_HPD_FILTER_L, 0xa0);
	regmap_write(priv->regmap, TX_HDCP_HPD_FILTER_H, 0xa0);

	/* Disable MEM power-down */
	regmap_write(priv->regmap, TX_MEM_PD_REG0, 0);

	regmap_write(priv->regmap, TX_HDCP_CONFIG3,
		     FIELD_PREP(TX_HDCP_CONFIG3_DDC_I2C_BUS_CLOCK_TIME_DIVIDER,
				(sys_clk_hz / ddc_i2c_bus_clk_hz) - 1));

	/* Enable software controlled DDC transaction */
	regmap_write(priv->regmap, TX_HDCP_EDID_CONFIG,
		     TX_HDCP_EDID_CONFIG_FORCED_MEM_COPY_DONE |
		     TX_HDCP_EDID_CONFIG_MEM_COPY_DONE_CONFIG);
	regmap_write(priv->regmap, TX_CORE_EDID_CONFIG_MORE,
		     TX_CORE_EDID_CONFIG_MORE_SYS_TRIGGER_CONFIG_SEMI_MANU);

	return 0;
}

static void anx_hdmi_14_tx_exit(struct anx_hdmi_14_tx *priv)
{
	clk_disable_unprepare(priv->sys_clk);
}

struct anx_hdmi_14_tx *anx_hdmi_14_tx_probe(struct device *dev)
{
	struct anx_hdmi_14_tx *priv;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return ERR_PTR(-ENOMEM);

	priv->dev = dev;

	priv->input_bus_format = MEDIA_BUS_FMT_FIXED;
	priv->output_bus_format = MEDIA_BUS_FMT_FIXED;

	mutex_init(&priv->cec_notifier_mutex);

	priv->regmap = dev_get_regmap(dev, "bridge");
	if (IS_ERR(priv->regmap))
		return ERR_CAST(priv->regmap);

	priv->sys_clk = devm_clk_get(dev, "sys");
	if (IS_ERR(priv->sys_clk)) {
		dev_err(dev, "Failed to get the ANX sys clock\n");
		return ERR_CAST(priv->sys_clk);
	}

	ret = anx_hdmi_14_tx_init(priv);
	if (ret)
		return ERR_PTR(ret);

	priv->bridge.driver_private = priv;
	priv->bridge.funcs = &anx_hdmi_14_tx_bridge_funcs;
#ifdef CONFIG_OF
	priv->bridge.of_node = dev->of_node;
#endif

	drm_bridge_add(&priv->bridge);

	return priv;
}
EXPORT_SYMBOL_GPL(anx_hdmi_14_tx_probe);

void anx_hdmi_14_tx_remove(struct anx_hdmi_14_tx *priv)
{
	drm_bridge_remove(&priv->bridge);

	anx_hdmi_14_tx_exit(priv);
}
EXPORT_SYMBOL_GPL(anx_hdmi_14_tx_remove);

MODULE_AUTHOR("Martin Blumenstingl <martin.blumenstingl@googlemail.com>");
MODULE_DESCRIPTION("Analogix HDMI 1.4 Transmitter IP driver");
MODULE_LICENSE("GPL v2");
