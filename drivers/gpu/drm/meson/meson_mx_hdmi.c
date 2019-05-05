// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2020 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 *
 * All registers and magic values are taken from Amlogic's GPL kernel sources:
 *   Copyright (C) 2010 Amlogic, Inc.
 */

#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/component.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/regmap.h>

#include <uapi/linux/media-bus-format.h>

#include <drm/bridge/txccq_txc_48352.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>

#include "meson_drv.h"
#include "meson_registers.h"
#include "meson_vclk.h"
#include "meson_venc.h"

#define HDMI_ADDR_PORT					0x0
#define HDMI_DATA_PORT					0x4
#define HDMI_CTRL_PORT					0x8
	#define HDMI_CTRL_PORT_APB3_ERR_EN		BIT(15)

struct meson_mx_hdmi {
	struct device			*dev;
	struct drm_device		*drm;
	struct txc_48352		*txc_48352;

	struct clk			*pclk;

	struct drm_encoder		encoder;
	struct drm_bridge		bridge;

	unsigned int			output_bus_format;
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

static const struct regmap_config meson_mx_hdmi_bus_regmap_config = {
	.name = "bus",
	.reg_bits = 16,
	.val_bits = 16,
	.reg_stride = 1,
	.reg_read = meson_mx_hdmi_reg_read,
	.reg_write = meson_mx_hdmi_reg_write,
	.fast_io = true,
};

static int meson_mx_hdmi_child_regmap_read(void *context, unsigned int addr,
					   unsigned int *data)
{
	struct regmap *bus_regmap = context;

	return regmap_read(bus_regmap, addr, data);
}

static int meson_mx_hdmi_child_regmap_write(void *context, unsigned int addr,
					    unsigned int data)
{
	struct regmap *bus_regmap = context;

	return regmap_write(bus_regmap, addr, data);
}

static const struct regmap_config meson_mx_hdmi_bridge_regmap_config = {
	.name = "bridge",
	.reg_bits = 16,
	.val_bits = 8,
	.reg_stride = 1,
	.reg_read = meson_mx_hdmi_child_regmap_read,
	.reg_write = meson_mx_hdmi_child_regmap_write,
	.max_register = 0x7ff,
	.fast_io = true,
};

static const struct regmap_range meson_mx_hdmi_other_regmap_ranges[] = {
	regmap_reg_range(0x8000, 0x800c),
};

static const struct regmap_access_table meson_mx_hdmi_other_regmap_access = {
	.yes_ranges = meson_mx_hdmi_other_regmap_ranges,
	.n_yes_ranges = ARRAY_SIZE(meson_mx_hdmi_other_regmap_ranges),
};

static const struct regmap_config meson_mx_hdmi_other_regmap_config = {
	.name = "other",
	.reg_bits = 16,
	.val_bits = 16,
	.reg_stride = 1,
	.reg_read = meson_mx_hdmi_child_regmap_read,
	.reg_write = meson_mx_hdmi_child_regmap_write,
	.rd_table = &meson_mx_hdmi_other_regmap_access,
	.wr_table = &meson_mx_hdmi_other_regmap_access,
	.max_register = 0xffff,
	.fast_io = true,
};

static int meson_mx_hdmi_bridge_atomic_check(struct drm_bridge *bridge,
					struct drm_bridge_state *bridge_state,
					struct drm_crtc_state *crtc_state,
					struct drm_connector_state *conn_state)
{
	struct meson_mx_hdmi *hdmi = to_meson_mx_hdmi(bridge);

	hdmi->output_bus_format = bridge_state->output_bus_cfg.format;

	return 0;
}

static void meson_mx_hdmi_bridge_enable(struct drm_bridge *bridge)
{
	struct meson_mx_hdmi *hdmi = to_meson_mx_hdmi(bridge);
	struct meson_drm *priv = hdmi->drm->dev_private;

	meson_venc_hdmi_bridge_reset(priv);

	meson_venc_hdmi_encoder_enable(priv);
}

static void meson_mx_hdmi_bridge_disable(struct drm_bridge *bridge)
{
	struct meson_mx_hdmi *hdmi = to_meson_mx_hdmi(bridge);
	struct meson_drm *priv = hdmi->drm->dev_private;

	meson_venc_hdmi_encoder_disable(priv);
}

static void meson_mx_hdmi_bridge_mode_set(struct drm_bridge *bridge,
				const struct drm_display_mode *mode,
				const struct drm_display_mode *adj_mode)
{
	unsigned int vclk_freq, venc_freq, hdmi_freq, phy_freq, ycrcb_map;
	struct meson_mx_hdmi *hdmi = to_meson_mx_hdmi(bridge);
	struct meson_drm *priv = hdmi->drm->dev_private;
	int cea_mode = drm_match_cea_mode(mode);

	switch (hdmi->output_bus_format) {
	case MEDIA_BUS_FMT_YUV8_1X24:
	case MEDIA_BUS_FMT_YUV10_1X30:
	case MEDIA_BUS_FMT_YUV12_1X36:
	case MEDIA_BUS_FMT_YUV16_1X48:
	case MEDIA_BUS_FMT_UYVY8_1X16:
	case MEDIA_BUS_FMT_UYVY10_1X20:
	case MEDIA_BUS_FMT_UYVY12_1X24:
		ycrcb_map = VPU_HDMI_OUTPUT_CRYCB;
		break;

	case MEDIA_BUS_FMT_RGB888_1X24:
	case MEDIA_BUS_FMT_RGB101010_1X30:
	case MEDIA_BUS_FMT_RGB121212_1X36:
	case MEDIA_BUS_FMT_RGB161616_1X48:
	default:
		ycrcb_map = VPU_HDMI_OUTPUT_YCBCR;
		break;

	}

	/* VENC + VENC-DVI Mode setup */
	meson_venc_hdmi_mode_set(priv, cea_mode, ycrcb_map, false, mode);

	vclk_freq = mode->clock;
	phy_freq = vclk_freq * 10;

	if (!cea_mode) {
		meson_vclk_setup(priv, MESON_VCLK_TARGET_DMT, phy_freq,
				 vclk_freq, vclk_freq, vclk_freq, false);
	} else {
		if (mode->flags & DRM_MODE_FLAG_DBLCLK)
			vclk_freq *= 2;

		venc_freq = vclk_freq;
		hdmi_freq = vclk_freq;

		if (meson_venc_hdmi_venc_repeat(cea_mode))
			venc_freq *= 2;

		vclk_freq = max(venc_freq, hdmi_freq);

		if (mode->flags & DRM_MODE_FLAG_DBLCLK)
			venc_freq /= 2;

		drm_dbg(hdmi->drm, "vclk:%d venc=%d hdmi=%d enci=%d\n",
			vclk_freq, venc_freq, hdmi_freq,
			priv->venc.hdmi_use_enci);

		meson_vclk_setup(priv, MESON_VCLK_TARGET_HDMI, phy_freq,
				 vclk_freq, venc_freq, hdmi_freq,
				 priv->venc.hdmi_use_enci);
	}
}

static const struct drm_bridge_funcs meson_mx_hdmi_bridge_funcs = {
	.atomic_duplicate_state	= drm_atomic_helper_bridge_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_bridge_destroy_state,
	.atomic_reset		= drm_atomic_helper_bridge_reset,
	.atomic_check		= meson_mx_hdmi_bridge_atomic_check,
	.enable			= meson_mx_hdmi_bridge_enable,
	.disable		= meson_mx_hdmi_bridge_disable,
	.mode_set		= meson_mx_hdmi_bridge_mode_set,
};

static const struct drm_encoder_funcs meson_mx_hdmi_encoder_funcs = {
	.destroy	= drm_encoder_cleanup,
};

static void meson_mx_hdmi_exit(struct meson_mx_hdmi *hdmi)
{
	if (hdmi->txc_48352)
		txc_48352_unbind(hdmi->txc_48352);

	drm_encoder_cleanup(&hdmi->encoder);

	clk_disable_unprepare(hdmi->pclk);
}

static int meson_mx_hdmi_init(struct meson_mx_hdmi *hdmi, void __iomem *base)
{
	struct drm_bridge *next_bridge;
	u32 regval;
	int ret;

	ret = clk_prepare_enable(hdmi->pclk);
	if (ret) {
		drm_err(hdmi->drm, "Failed to enable the pclk: %d\n", ret);
		return ret;
	}

	regval = readl(base + HDMI_CTRL_PORT);
	regval |= HDMI_CTRL_PORT_APB3_ERR_EN;
	writel(regval, base + HDMI_CTRL_PORT);

	hdmi->encoder.possible_crtcs = BIT(0);
	ret = drm_encoder_init(hdmi->drm, &hdmi->encoder,
			       &meson_mx_hdmi_encoder_funcs,
			       DRM_MODE_ENCODER_TMDS, NULL);
	if (ret) {
		return ret;
		goto err_disable_clk;
	}

	hdmi->bridge.funcs = &meson_mx_hdmi_bridge_funcs;
	ret = drm_bridge_attach(&hdmi->encoder, &hdmi->bridge, NULL, 0);
	if (ret) {
		return ret;
		goto err_cleanup_encoder;
	}

	hdmi->txc_48352 = txc_48352_bind(&hdmi->encoder, hdmi->dev);
	if (IS_ERR(hdmi->txc_48352)) {
		ret = PTR_ERR(hdmi->txc_48352);
		goto err_cleanup_encoder;
	}

	next_bridge = of_drm_find_bridge(hdmi->dev->of_node);
	if (next_bridge)
		drm_bridge_attach(&hdmi->encoder, next_bridge,
				  &hdmi->bridge, 0);

	return 0;

err_disable_clk:
	clk_disable_unprepare(hdmi->pclk);
err_cleanup_encoder:
	drm_encoder_cleanup(&hdmi->encoder);
	return ret;
}

static int meson_mx_hdmi_bind(struct device *dev, struct device *master,
			      void *data)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct regmap *bus_regmap, *child_regmap;
	struct drm_device *drm = data;
	struct meson_mx_hdmi *hdmi;
	struct resource *res;
	void __iomem *base;
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

	bus_regmap = devm_regmap_init(dev, NULL, base,
				      &meson_mx_hdmi_bus_regmap_config);
	if (IS_ERR(bus_regmap))
		return PTR_ERR(bus_regmap);

	child_regmap = devm_regmap_init(dev, NULL, bus_regmap,
					&meson_mx_hdmi_bridge_regmap_config);
	if (IS_ERR(child_regmap))
		return PTR_ERR(child_regmap);

	child_regmap = devm_regmap_init(dev, NULL, bus_regmap,
					&meson_mx_hdmi_other_regmap_config);
	if (IS_ERR(child_regmap))
		return PTR_ERR(child_regmap);

	hdmi->pclk = devm_clk_get(hdmi->dev, "pclk");
	if (IS_ERR(hdmi->pclk)) {
		ret = PTR_ERR(hdmi->pclk);
		drm_err(drm, "Failed to get the pclk: %d\n", ret);
		return ret;
	}

	ret = meson_mx_hdmi_init(hdmi, base);
	if (ret)
		return ret;

	return 0;
}

static void meson_mx_hdmi_unbind(struct device *dev, struct device *master,
				 void *data)
{
	struct meson_mx_hdmi *hdmi = dev_get_drvdata(dev);

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
