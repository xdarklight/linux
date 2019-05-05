// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2019 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
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
#include <linux/gpio/consumer.h>
#include <linux/hdmi.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_device.h>
#include <linux/phy/phy.h>
#include <linux/regmap.h>
#include <media/cec-notifier.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_edid.h>
#include <drm/drm_of.h>
#include <drm/drm_probe_helper.h>
#include <drm/drmP.h>

#include "meson_mx_hdmi.h"

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
	struct gpio_desc		*hpd_gpiod;
	struct cec_notifier		*cec_notifier;
	struct drm_connector		connector;
	struct drm_encoder		encoder;
	bool				sink_is_hdmi;
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

static const struct regmap_config meson_mx_hdmi_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.reg_stride = 1,
	.reg_read = meson_mx_hdmi_reg_read,
	.reg_write = meson_mx_hdmi_reg_write,
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
		     TX_SYS5_TX_SOFT_RESET_2_UNKNOWN_BIT7 |
		     TX_SYS5_TX_SOFT_RESET_2_HDMI_CH0_RST_IN);
	udelay(10);

	regmap_write(hdmi->regmap, TX_SYS5_TX_SOFT_RESET_2,
		     TX_SYS5_TX_SOFT_RESET_2_UNKNOWN_BIT6 |
		     TX_SYS5_TX_SOFT_RESET_2_UNKNOWN_BIT5);
	udelay(10);

	regmap_write(hdmi->regmap, TX_SYS5_TX_SOFT_RESET_1,
		     TX_SYS5_TX_SOFT_RESET_1_TX_PIXEL_RSTN |
		     TX_SYS5_TX_SOFT_RESET_1_TX_TMDS_RSTN |
		     TX_SYS5_TX_SOFT_RESET_1_TX_AUDIO_MASTER_RSTN |
		     TX_SYS5_TX_SOFT_RESET_1_TX_AUDIO_RESAMPLE_RSTN |
		     TX_SYS5_TX_SOFT_RESET_1_TX_I2S_RESET_RSTN |
		     TX_SYS5_TX_SOFT_RESET_1_TX_DIG_RESET_N_CH2 |
		     TX_SYS5_TX_SOFT_RESET_1_TX_DIG_RESET_N_CH1 |
		     TX_SYS5_TX_SOFT_RESET_1_TX_DIG_RESET_N_CH0);
	udelay(10);
}

static void meson_mx_hdmi_sys5_reset_deassert(struct meson_mx_hdmi *hdmi)
{
	/* Release the resets except tmds_clk */
	regmap_write(hdmi->regmap, TX_SYS5_TX_SOFT_RESET_1,
		     TX_SYS5_TX_SOFT_RESET_1_TX_TMDS_RSTN);
	udelay(10);

	/* Release the tmds_clk reset as well */
	regmap_write(hdmi->regmap, TX_SYS5_TX_SOFT_RESET_1, 0x0);
	udelay(10);

	regmap_write(hdmi->regmap, TX_SYS5_TX_SOFT_RESET_2,
		     TX_SYS5_TX_SOFT_RESET_2_UNKNOWN_BIT6 |
		     TX_SYS5_TX_SOFT_RESET_2_UNKNOWN_BIT5 |
		     TX_SYS5_TX_SOFT_RESET_2_UNKNOWN_BIT3);
	udelay(10);

	regmap_write(hdmi->regmap, TX_SYS5_TX_SOFT_RESET_2,
		     TX_SYS5_TX_SOFT_RESET_2_UNKNOWN_BIT6 |
		     TX_SYS5_TX_SOFT_RESET_2_UNKNOWN_BIT5);
	udelay(10);
}

static void meson_mx_hdmi_encoder_mode_set(struct drm_encoder *encoder,
					   struct drm_display_mode *mode,
					   struct drm_display_mode *adj_mode)
{
	struct meson_mx_hdmi *hdmi = to_meson_mx_hdmi(encoder);

	if (drm_match_cea_mode(mode))
		regmap_write(hdmi->regmap, TX_VIDEO_DTV_TIMING, 0x0);
	else
		regmap_write(hdmi->regmap, TX_VIDEO_DTV_TIMING,
			     TX_VIDEO_DTV_TIMING_UNKNOWN_BIT4);

	regmap_write(hdmi->regmap, TX_VIDEO_DTV_MODE,
		     FIELD_PREP(TX_VIDEO_DTV_MODE_COLOR_DEPTH_MASK,
				meson_mx_hdmi_bus_fmt_color_depth(0))); // TODO: see hdmi_bus_fmt_color_depth in dw-hdmi.c for the correct parameter

#if 1
	// TODO: move most of this to _enable?

	regmap_write(hdmi->regmap, TX_CORE_DATA_CAPTURE_2,
		     TX_CORE_DATA_CAPTURE_2_INTERNAL_PACKET_ENABLE);

	regmap_write(hdmi->regmap, TX_CORE_DATA_MONITOR_1,
			TX_CORE_DATA_MONITOR_1_LANE0 |
			FIELD_PREP(TX_CORE_DATA_MONITOR_1_SELECT_LANE0_MASK, 0x3));

	regmap_write(hdmi->regmap, TX_CORE_DATA_MONITOR_2,
			FIELD_PREP(TX_CORE_DATA_MONITOR_2_MONITOR_SELECT_MASK, 0x2));

	regmap_write(hdmi->regmap, TX_TMDS_MODE,
		     TX_TMDS_MODE_FORCED_HDMI | TX_TMDS_MODE_HDMI_CONFIG);

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
				0x0) | // TODO
		     FIELD_PREP(TX_VIDEO_DTV_OPTION_L_OUTPUT_COLOR_DEPTH,
				0x0) | // TODO
		     FIELD_PREP(TX_VIDEO_DTV_OPTION_L_INPUT_COLOR_DEPTH,
				0x0)); // TODO

	/* TODO: color range mapping: 0=16-235/240; 1=16-240; 2=1-254; 3=0-255 */
	regmap_write(hdmi->regmap, TX_VIDEO_DTV_OPTION_H,
		     FIELD_PREP(TX_VIDEO_DTV_OPTION_H_OUTPUT_COLOR_RANGE_MASK,
				0x2) |
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
#endif

#if 0
	// TODO: why is this needed? also this is incomplete compared to the Amlogic driver!
	/* all resets need to be applied twice */
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
		     TX_SYS5_TX_SOFT_RESET_2_UNKNOWN_BIT7 |
		     TX_SYS5_TX_SOFT_RESET_2_UNKNOWN_BIT6 |
		     TX_SYS5_TX_SOFT_RESET_2_UNKNOWN_BIT5 |
		     TX_SYS5_TX_SOFT_RESET_2_HDMI_CH0_RST_IN |
		     TX_SYS5_TX_SOFT_RESET_2_UNKNOWN_BIT3 |
		     TX_SYS5_TX_SOFT_RESET_2_TX_DDC_HDCP_RSTN |
		     TX_SYS5_TX_SOFT_RESET_2_TX_DDC_EDID_RSTN |
		     TX_SYS5_TX_SOFT_RESET_2_TX_DIG_RESET_N_CH3);
	usleep_range(5000, 7500);
	regmap_write(hdmi->regmap, TX_SYS5_TX_SOFT_RESET_1, 0x00);
	regmap_write(hdmi->regmap, TX_SYS5_TX_SOFT_RESET_2, 0x00);
	usleep_range(5000, 7500);
#endif
	// TODO: phy_set_mode(hdmi->phy, ???); - maybe use phy_configure?
}

static void meson_mx_hdmi_hw_enable(struct meson_mx_hdmi *hdmi)
{
	regmap_write(hdmi->regmap, 0x0010, 0xff);

	regmap_write(hdmi->regmap, TX_HDCP_MODE, 0x40);

	// TODO: hdmi_phy_suspend();

	/* Band-gap and main-bias. 0x1d = power-up, 0x00 = power-down */
	regmap_write(hdmi->regmap, 0x0017, 0x1d);

	/* Serializer Internal clock setting */
	regmap_write(hdmi->regmap, 0x0018, 0x24);

	/*
	 * bit[2:0]=011: CK channel output TMDS CLOCK
	 * bit[2:0]=101, ck channel output PHYCLCK
	 */
	regmap_write(hdmi->regmap, 0x001a, 0xfb);

	/* Termination resistor calib value */
	regmap_write(hdmi->regmap, TX_CORE_CALIB_VALUE, 0x0f);

	/* HPD glitch filter */
	regmap_write(hdmi->regmap, TX_HDCP_HPD_FILTER_L, 0xa0);
	regmap_write(hdmi->regmap, TX_HDCP_HPD_FILTER_H, 0xa0);

	meson_mx_hdmi_sys5_reset_assert(hdmi);

	/* Enable software controlled DDC transaction */
	regmap_write(hdmi->regmap, TX_HDCP_EDID_CONFIG,
		     TX_HDCP_EDID_CONFIG_MEM_COPY_DONE_CONFIG |
		     TX_HDCP_EDID_CONFIG_FORCED_MEM_COPY_DONE);
	regmap_write(hdmi->regmap, TX_CORE_EDID_CONFIG_MORE,
		     TX_CORE_EDID_CONFIG_MORE_KEEP_EDID_ERROR);

	regmap_write(hdmi->regmap, TX_PACKET_ALLOC_ACTIVE_1, 0x0);
	regmap_write(hdmi->regmap, TX_PACKET_CONTROL_2,
		     TX_PACKET_CONTROL_2_HORIZONTAL_GC_PACKET_TRANSPORT_EN);

	regmap_write(hdmi->regmap, TX_HDCP_CONFIG0,
		     TX_HDCP_CONFIG0_ROM_ENCRYPT_OFF_MASK);
	regmap_write(hdmi->regmap, TX_HDCP_MEM_CONFIG, 0x0);
	regmap_write(hdmi->regmap, TX_HDCP_ENCRYPT_BYTE, 0x0);

	/* Ensure that the first HDCP succeeds */
	regmap_write(hdmi->regmap, TX_PACKET_CONTROL_1,
		     FIELD_PREP(TX_PACKET_CONTROL_1_PACKET_START_LATENCY, 58));

	regmap_write(hdmi->regmap, TX_HDCP_MODE, 0x0);

	// TODO: 0x18 - 1 but set_dispmode uses 0x30 - 1
	regmap_write(hdmi->regmap, TX_HDCP_CONFIG3,
		     FIELD_PREP(TX_HDCP_CONFIG3_DDC_I2C_BUS_CLOCK_TIME_DIVIDER,
				0x17));

	regmap_write(hdmi->regmap, TX_HDCP_MODE, TX_HDCP_MODE_ESS_CONFIG);
	
	regmap_write(hdmi->regmap, TX_AUDIO_CONTROL_MORE, 0x1);

	meson_mx_hdmi_sys5_reset_deassert(hdmi);
}

static void meson_mx_hdmi_hw_disable(struct meson_mx_hdmi *hdmi)
{
}

static void meson_mx_hdmi_encoder_enable(struct drm_encoder *encoder)
{
	struct meson_mx_hdmi *hdmi = to_meson_mx_hdmi(encoder);

	phy_power_on(hdmi->phy);

	meson_mx_hdmi_hw_enable(hdmi);
}

static void meson_mx_hdmi_encoder_disable(struct drm_encoder *encoder)
{
	struct meson_mx_hdmi *hdmi = to_meson_mx_hdmi(encoder);

	meson_mx_hdmi_hw_disable(hdmi);

	phy_power_off(hdmi->phy);
}

static const struct drm_encoder_helper_funcs meson_mx_hdmi_encoder_helper_funcs = {
	.enable	= meson_mx_hdmi_encoder_enable,
	.disable = meson_mx_hdmi_encoder_disable,
	.mode_set = meson_mx_hdmi_encoder_mode_set,
};

static const struct drm_encoder_funcs meson_mx_hdmi_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static int meson_mx_get_edid_block(void *data, u8 *buf, unsigned int block,
				   size_t len)
{
	struct meson_mx_hdmi *hdmi = data;
	unsigned int i, val, start = block * EDID_LENGTH;

	for (i = 0; i < len; i++) {
		regmap_read(hdmi->regmap, TX_RX_EDID_OFFSET + start + i, &val);
		buf[i] = val;
	}

	return 0;
}

static int meson_mx_hdmi_connector_get_modes(struct drm_connector *connector)
{
	struct meson_mx_hdmi *hdmi = to_meson_mx_hdmi(connector);
	struct edid *edid;
	int ret;

	edid = drm_do_get_edid(connector, meson_mx_get_edid_block, hdmi);
	if (!edid)
		return 0;

	hdmi->sink_is_hdmi = drm_detect_hdmi_monitor(edid);
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

	if (hdmi->hpd_gpiod) {
		if (gpiod_get_value_cansleep(hdmi->hpd_gpiod))
			status = connector_status_connected;
		else
			status = connector_status_disconnected;
	} else {
		regmap_read(hdmi->regmap, TX_HDCP_ST_EDID_STATUS, &val);
		if (val & TX_HDCP_ST_EDID_STATUS_HPD_STATUS)
			status = connector_status_connected;
		else
			status = connector_status_disconnected;
	}

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

	encoder->possible_crtcs = BIT(0);

	drm_encoder_init(drm, encoder, &meson_mx_hdmi_encoder_funcs,
			 DRM_MODE_ENCODER_TMDS, NULL);
	drm_encoder_helper_add(encoder, &meson_mx_hdmi_encoder_helper_funcs);

	hdmi->connector.polled = DRM_CONNECTOR_POLL_HPD;

	drm_connector_init(drm, &hdmi->connector,
			   &meson_mx_hdmi_connector_funcs,
			   DRM_MODE_CONNECTOR_HDMIA);
	drm_connector_helper_add(&hdmi->connector,
				 &meson_mx_hdmi_connector_helper_funcs);

	drm_connector_attach_encoder(&hdmi->connector, encoder);

	return 0;
}

static irqreturn_t meson_mx_hdmi_irq_thread(int irq, void *dev_id)
{
	struct meson_mx_hdmi *hdmi = dev_id;

	drm_helper_hpd_irq_event(hdmi->connector.dev);

	if (hdmi->connector.status != connector_status_connected)
		cec_notifier_set_phys_addr(hdmi->cec_notifier,
					   CEC_PHYS_ADDR_INVALID);

	return IRQ_HANDLED;
}

static irqreturn_t meson_mx_hdmi_irq_handler(int irq, void *dev_id)
{
	struct meson_mx_hdmi *hdmi = dev_id;
	u32 all_mask, regval;

	all_mask = HDMI_OTHER_INTR_STAT_EDID_RISING |
		   HDMI_OTHER_INTR_STAT_HPD_FALLING |
		   HDMI_OTHER_INTR_STAT_HPD_RISING;

	regmap_read(hdmi->regmap, HDMI_OTHER_INTR_STAT, &regval);
	if (regval & all_mask) {
		regmap_write(hdmi->regmap, HDMI_OTHER_INTR_STAT_CLR, all_mask);

		return IRQ_WAKE_THREAD;
	}

	return IRQ_HANDLED;
}

static int meson_mx_hdmi_init(struct meson_mx_hdmi *hdmi, void __iomem *base)
{
	u32 regval;
	int ret;

	regval = readl(base + HDMI_CTRL_PORT);
	regval |= HDMI_CTRL_PORT_APB3_ERR_EN;
	writel(regval, base + HDMI_CTRL_PORT);

	ret = clk_prepare_enable(hdmi->pclk);
	if (ret) {
		DRM_DEV_ERROR(hdmi->dev, "Failed to enable the pclk: %d\n",
			      ret);
		return ret;
	}

	devm_add_action_or_reset(hdmi->dev,
				 (void(*)(void *))clk_disable_unprepare,
				 hdmi->pclk);

	ret = clk_prepare_enable(hdmi->hdmi_clk);
	if (ret) {
		DRM_DEV_ERROR(hdmi->dev, "Failed to enable the HDMI clk: %d\n",
			      ret);
		return ret;
	}

	devm_add_action_or_reset(hdmi->dev,
				 (void(*)(void *))clk_disable_unprepare,
				 hdmi->hdmi_clk);

	ret = phy_init(hdmi->phy);
	if (ret) {
		DRM_DEV_ERROR(hdmi->dev, "Failed to initialize the PHY: %d\n",
			      ret);
		return ret;
	}


	devm_add_action_or_reset(hdmi->dev,
				 (void(*)(void *))phy_exit,
				 hdmi->phy);

	/* unmask = enable all interrupts - TODO: why aren't any interrupts fired? */
	regmap_write(hdmi->regmap, HDMI_OTHER_INTR_MASKN, 0x0);

	return 0;
}

static int meson_mx_hdmi_bind(struct device *dev, struct device *master,
			      void *data)
{
	struct platform_device *pdev = to_platform_device(dev);
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

	hdmi->hpd_gpiod = devm_gpiod_get_optional(hdmi->dev, "hpd", GPIOD_IN);
	if (IS_ERR(hdmi->hpd_gpiod)) {
		ret = PTR_ERR(hdmi->hpd_gpiod);
		DRM_DEV_ERROR(dev, "Failed to get HPD GPIO: %d\n", ret);
		return ret;
	}

	hdmi->cec_notifier = cec_notifier_get(dev);
	if (!hdmi->cec_notifier)
		return -ENOMEM;

	ret = meson_mx_hdmi_init(hdmi, base);
	if (ret)
		return ret;

	ret = meson_mx_hdmi_register(drm, hdmi);
	if (ret) {
		DRM_DEV_ERROR(dev, "Failed to register HDMI: %d\n", ret);
		return ret;
	}

	ret = devm_request_threaded_irq(dev, irq, meson_mx_hdmi_irq_handler,
					meson_mx_hdmi_irq_thread, 0, NULL,
					hdmi);
	if (ret) {
		DRM_DEV_ERROR(dev, "Failed to request threaded irq: %d\n",
			      ret);
		return ret;
	}

	return 0;
}

static void meson_mx_hdmi_unbind(struct device *dev, struct device *master,
				 void *data)
{
	struct meson_mx_hdmi *hdmi = dev_get_drvdata(dev);

	hdmi->connector.funcs->destroy(&hdmi->connector);
	hdmi->encoder.funcs->destroy(&hdmi->encoder);
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
	{ /* end */ },
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
