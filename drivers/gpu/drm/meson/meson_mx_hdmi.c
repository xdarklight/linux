// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2020 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 *
 * All registers and magic values are taken from Amlogic's GPL kernel sources:
 *   Copyright (C) 2010 Amlogic, Inc.
 *
 * The driver structure is loosely based on zx_hdmi.c:
 *   Copyright 2016 Linaro Ltd.
 *   Copyright 2016 ZTE Corporation.
 */

#include <linux/clk.h>
#include <linux/component.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/hdmi.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_device.h>
#include <linux/phy/phy.h>
#include <linux/regmap.h>
#include <media/cec-notifier.h>

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
	struct i2c_adapter		*ddc;
	struct cec_notifier		*cec_notifier;
	struct drm_connector		connector;
	struct drm_encoder		encoder;
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

static int meson_mx_hdmi_bus_fmt_color_depth(unsigned int bus_format)
{
	switch (bus_format) {
	case MEDIA_BUS_FMT_RGB101010_1X30:
	case MEDIA_BUS_FMT_YUV10_1X30:
	case MEDIA_BUS_FMT_UYVY10_1X20:
	case MEDIA_BUS_FMT_UYYVYY10_0_5X30:
		return 0x1;

	case MEDIA_BUS_FMT_RGB121212_1X36:
	case MEDIA_BUS_FMT_YUV12_1X36:
	case MEDIA_BUS_FMT_UYVY12_1X24:
	case MEDIA_BUS_FMT_UYYVYY12_0_5X36:
		return 0x2;

	case MEDIA_BUS_FMT_RGB161616_1X48:
	case MEDIA_BUS_FMT_YUV16_1X48:
	case MEDIA_BUS_FMT_UYYVYY16_0_5X48:
		return 0x3;

	case MEDIA_BUS_FMT_RGB888_1X24:
	case MEDIA_BUS_FMT_YUV8_1X24:
	case MEDIA_BUS_FMT_UYVY8_1X16:
	case MEDIA_BUS_FMT_UYYVYY8_0_5X24:
	default:
		return 0x0;
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
		     /* HACK:
		     TX_SYS5_TX_SOFT_RESET_2_HDMI_CH2_RST_IN |
		     TX_SYS5_TX_SOFT_RESET_2_HDMI_CH1_RST_IN |*/ 0x0);
	usleep_range(10, 20);
}

static void meson_mx_hdmi_encoder_disable(struct drm_encoder *encoder)
{
	struct meson_mx_hdmi *hdmi = to_meson_mx_hdmi(encoder);
	struct meson_drm *priv = hdmi->drm->dev_private;
	int ret;

	meson_venc_hdmi_encoder_disable(priv);

	regmap_update_bits(hdmi->regmap, HDMI_OTHER_CTRL1,
			   HDMI_OTHER_CTRL1_POWER_ON, 0);

	// TODO: assert resets?

	if (hdmi->phy_is_on) {
		ret = phy_power_off(hdmi->phy);
		if (ret)
			DRM_DEV_ERROR(hdmi->dev, "Failed to turn off PHY\n");
		else
			hdmi->phy_is_on = false;
	}
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

	regmap_write(hdmi->regmap, TX_HDCP_CONFIG0,
		     FIELD_PREP(TX_HDCP_CONFIG0_ROM_ENCRYPT_OFF_MASK, 0x3));
	regmap_write(hdmi->regmap, TX_HDCP_MEM_CONFIG, 0x0);
	regmap_write(hdmi->regmap, TX_HDCP_ENCRYPT_BYTE, 0x0);

	regmap_write(hdmi->regmap, TX_PACKET_CONTROL_1,
		     FIELD_PREP(TX_PACKET_CONTROL_1_PACKET_START_LATENCY, 58));

	regmap_write(hdmi->regmap, TX_PACKET_CONTROL_2,
		     TX_PACKET_CONTROL_2_HORIZONTAL_GC_PACKET_TRANSPORT_EN);

	regmap_write(hdmi->regmap, TX_HDCP_MODE, TX_HDCP_MODE_CLEAR_AVMUTE);

	regmap_write(hdmi->regmap, TX_HDCP_MODE, TX_HDCP_MODE_ESS_CONFIG);

	regmap_write(hdmi->regmap, TX_AUDIO_CONTROL_MORE, 0x1);

	if (drm_match_cea_mode(&encoder->crtc->state->adjusted_mode) == 39)
		regmap_write(hdmi->regmap, TX_VIDEO_DTV_TIMING, 0x0);
	else
		regmap_write(hdmi->regmap, TX_VIDEO_DTV_TIMING,
			     TX_VIDEO_DTV_TIMING_DISABLE_VIC39_CORRECTION);

	regmap_write(hdmi->regmap, TX_VIDEO_DTV_MODE,
		     FIELD_PREP(TX_VIDEO_DTV_MODE_COLOR_DEPTH_MASK,
				meson_mx_hdmi_bus_fmt_color_depth(0))); // TODO: see hdmi_bus_fmt_color_depth in dw-hdmi.c for the correct parameter

	regmap_write(hdmi->regmap, TX_CORE_DATA_CAPTURE_2,
		     TX_CORE_DATA_CAPTURE_2_INTERNAL_PACKET_ENABLE);

	regmap_write(hdmi->regmap, TX_CORE_DATA_MONITOR_1,
			TX_CORE_DATA_MONITOR_1_LANE0 |
			FIELD_PREP(TX_CORE_DATA_MONITOR_1_SELECT_LANE0_MASK, 0x7));

	regmap_write(hdmi->regmap, TX_CORE_DATA_MONITOR_2,
			FIELD_PREP(TX_CORE_DATA_MONITOR_2_MONITOR_SELECT_MASK, 0x2));

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

	regmap_write(hdmi->regmap, TX_VIDEO_DTV_OPTION_L,
		     FIELD_PREP(TX_VIDEO_DTV_OPTION_L_OUTPUT_COLOR_FORMAT_MASK,
				0x0) | // TODO
		     FIELD_PREP(TX_VIDEO_DTV_OPTION_L_INPUT_COLOR_FORMAT_MASK,
				0x1) | // TODO
		     FIELD_PREP(TX_VIDEO_DTV_OPTION_L_OUTPUT_COLOR_DEPTH,
				0x0) | // TODO
		     FIELD_PREP(TX_VIDEO_DTV_OPTION_L_INPUT_COLOR_DEPTH,
				0x0)); // TODO

	/* TODO: color range mapping: 0=16-235/240; 1=16-240; 2=1-254; 3=0-255 */
	regmap_write(hdmi->regmap, TX_VIDEO_DTV_OPTION_H,
		     FIELD_PREP(TX_VIDEO_DTV_OPTION_H_OUTPUT_COLOR_RANGE_MASK,
				0x0) |
		     FIELD_PREP(TX_VIDEO_DTV_OPTION_H_INPUT_COLOR_RANGE_MASK,
				0x0));
 
	if (true) { // TODO: param->cc == CC_ITU709
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

	meson_mx_hdmi_sys5_reset_deassert(hdmi);

#if 0
	// TODO, not ported yet:
	if ((param->VIC==HDMI_480p60)||(param->VIC==HDMI_480p60_16x9)
		||(param->VIC==HDMI_576p50)||(param->VIC==HDMI_576p50_16x9)
		||(param->VIC==HDMI_480i60)||(param->VIC==HDMI_480i60_16x9)
		||(param->VIC==HDMI_576i50)||(param->VIC==HDMI_576i50_16x9)) {
		regmap_write(hdmi->regmap, 0x018, 0x24);   
	} else {
		regmap_write(hdmi->regmap, 0x018, 0x22);   
	}

	if ((param->VIC==HDMI_1080p60)&&(param->color_depth==COLOR_30BIT)&&(hdmi_rd_reg(0x018)==0x22)) {
		regmap_write(hdmi->regmap, 0x018,0x12);
	}

	hdmi_reconfig_packet_setting(param->VIC);
#endif

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
			DRM_DEV_ERROR(hdmi->dev, "Failed to turn on PHY\n");
		else
			hdmi->phy_is_on = true;

		// TODO: phy_set_mode(hdmi->phy, ???); - maybe use phy_configure?
	}

	meson_venc_hdmi_encoder_enable(priv);
}

static void meson_mx_hdmi_encoder_mode_set(struct drm_encoder *encoder,
					   struct drm_display_mode *mode,
					   struct drm_display_mode *adj_mode)
{
	struct meson_mx_hdmi *hdmi = to_meson_mx_hdmi(encoder);
	struct meson_drm *priv = hdmi->drm->dev_private;
	int vic = drm_match_cea_mode(mode);
	unsigned int vclk_freq, venc_freq, hdmi_freq;

	/* VENC + VENC-DVI Mode setup */
	meson_venc_hdmi_mode_set(priv, drm_match_cea_mode(mode), mode);

	vclk_freq = mode->clock;

	if (!vic) {
		meson_vclk_setup(priv, MESON_VCLK_TARGET_DMT, vclk_freq,
				 vclk_freq, vclk_freq, false);
		return;
	}

	if (mode->flags & DRM_MODE_FLAG_DBLCLK)
		vclk_freq *= 2;

	venc_freq = vclk_freq;
	hdmi_freq = vclk_freq;

	if (meson_venc_hdmi_venc_repeat(vic))
		venc_freq *= 2;

	vclk_freq = max(venc_freq, hdmi_freq);

	if (mode->flags & DRM_MODE_FLAG_DBLCLK)
		venc_freq /= 2;

	DRM_DEBUG_DRIVER("vclk:%d venc=%d hdmi=%d enci=%d\n",
		vclk_freq, venc_freq, hdmi_freq,
		priv->venc.hdmi_use_enci);

	meson_vclk_setup(priv, MESON_VCLK_TARGET_HDMI, vclk_freq,
			 venc_freq, hdmi_freq, priv->venc.hdmi_use_enci);
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

	if (hdmi->ddc)
		edid = drm_get_edid(connector, hdmi->ddc);
	else
		edid = drm_do_get_edid(connector, meson_mx_hdmi_get_edid_block,
				       hdmi);

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
		dev_warn(hdmi->dev, "IRQ status has unknown bit set: 0x%04x\n",
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
		DRM_DEV_ERROR(hdmi->dev, "Failed to enable the pclk: %d\n",
			      ret);
		return ret;
	}

	ret = clk_set_rate(hdmi->hdmi_clk, hdmi_clk_hz);
	if (ret) {
		DRM_DEV_ERROR(hdmi->dev,
			      "Failed to set HDMI controller clock to 24MHz: %d\n",
			      ret);
		goto err_disable_pclk;
	}

	ret = clk_prepare_enable(hdmi->hdmi_clk);
	if (ret) {
		DRM_DEV_ERROR(hdmi->dev, "Failed to enable the HDMI clk: %d\n",
			      ret);
		goto err_disable_pclk;
	}

	regval = readl(base + HDMI_CTRL_PORT);
	regval |= HDMI_CTRL_PORT_APB3_ERR_EN;
	writel(regval, base + HDMI_CTRL_PORT);

	regmap_write(hdmi->regmap, 0x0010, 0xff);

	regmap_write(hdmi->regmap, TX_HDCP_MODE, 0x40);

	ret = phy_init(hdmi->phy);
	if (ret) {
		DRM_DEV_ERROR(hdmi->dev, "Failed to initialize the PHY: %d\n",
			      ret);
		goto err_disable_hdmi_clk;
	}

	/* Band-gap and main-bias. 0x1d = power-up, 0x00 = power-down */
	regmap_write(hdmi->regmap, 0x0017, 0x1d);

	/* Serializer Internal clock setting */
	regmap_write(hdmi->regmap, 0x0018, 0x22); // TODO: 0x22 vs 0x24

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
	struct device_node *ddc_node;
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
		DRM_DEV_ERROR(dev, "Failed to get the pclk: %d\n", ret);
		return ret;
	}

	hdmi->hdmi_clk = devm_clk_get(hdmi->dev, "hdmi");
	if (IS_ERR(hdmi->hdmi_clk)) {
		ret = PTR_ERR(hdmi->hdmi_clk);
		DRM_DEV_ERROR(dev, "Failed to get the hdmi clock: %d\n", ret);
		return ret;
	}

	hdmi->phy = devm_phy_get(hdmi->dev, "hdmi");
	if (IS_ERR(hdmi->phy)) {
		ret = PTR_ERR(hdmi->phy);
		DRM_DEV_ERROR(dev, "Failed to get the PHY: %d\n", ret);
		return ret;
	}

	ddc_node = of_parse_phandle(dev_of_node(dev), "ddc-i2c-bus", 0);
	if (ddc_node) {
		hdmi->ddc = of_get_i2c_adapter_by_node(ddc_node);
		of_node_put(ddc_node);
		if (!hdmi->ddc) {
			dev_dbg(hdmi->dev, "Failed to find the ddc-i2c-bus\n");
			return -EPROBE_DEFER;
		}
	}

	ret = meson_mx_hdmi_init(hdmi, base);
	if (ret)
		goto err_put_ddc;

	ret = meson_mx_hdmi_register(drm, hdmi);
	if (ret) {
		DRM_DEV_ERROR(dev, "Failed to register HDMI: %d\n", ret);
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
		DRM_DEV_ERROR(dev, "Failed to request threaded irq: %d\n",
			      ret);
		goto err_hdmi_exit;
	}

	return 0;

err_hdmi_exit:
	meson_mx_hdmi_exit(hdmi);
err_put_ddc:
	i2c_put_adapter(hdmi->ddc);
	return ret;
}

static void meson_mx_hdmi_unbind(struct device *dev, struct device *master,
				 void *data)
{
	struct meson_mx_hdmi *hdmi = dev_get_drvdata(dev);

	hdmi->connector.funcs->destroy(&hdmi->connector);
	hdmi->encoder.funcs->destroy(&hdmi->encoder);

	i2c_put_adapter(hdmi->ddc);

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
