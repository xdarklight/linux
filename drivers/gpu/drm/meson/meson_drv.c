// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2016 BayLibre, SAS
 * Author: Neil Armstrong <narmstrong@baylibre.com>
 * Copyright (C) 2014 Endless Mobile
 *
 * Written by:
 *     Jasper St. Pierre <jstpierre@mecheye.net>
 */

#include <linux/component.h>
#include <linux/module.h>
#include <linux/of_graph.h>
#include <linux/sys_soc.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/soc/amlogic/meson-canvas.h>

#include <drm/drm_aperture.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_fbdev_dma.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_modeset_helper_vtables.h>
#include <drm/drm_module.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_vblank.h>

#include "meson_crtc.h"
#include "meson_drv.h"
#include "meson_overlay.h"
#include "meson_plane.h"
#include "meson_osd_afbcd.h"
#include "meson_registers.h"
#include "meson_encoder_cvbs.h"
#include "meson_encoder_hdmi.h"
#include "meson_encoder_dsi.h"
#include "meson_viu.h"
#include "meson_vpp.h"
#include "meson_rdma.h"

#define DRIVER_NAME "meson"
#define DRIVER_DESC "Amlogic Meson DRM driver"

/**
 * DOC: Video Processing Unit
 *
 * VPU Handles the Global Video Processing, it includes management of the
 * clocks gates, blocks reset lines and power domains.
 *
 * What is missing :
 *
 * - Full reset of entire video processing HW blocks
 * - Scaling and setup of the VPU clock
 * - Bus clock gates
 * - Powering up video processing HW blocks
 * - Powering Up HDMI controller and PHY
 */

static const struct drm_mode_config_funcs meson_mode_config_funcs = {
	.atomic_check        = drm_atomic_helper_check,
	.atomic_commit       = drm_atomic_helper_commit,
	.fb_create           = drm_gem_fb_create,
};

static const struct drm_mode_config_helper_funcs meson_mode_config_helpers = {
	.atomic_commit_tail = drm_atomic_helper_commit_tail_rpm,
};

static irqreturn_t meson_irq(int irq, void *arg)
{
	struct drm_device *dev = arg;
	struct meson_drm *priv = dev->dev_private;

	(void)readl_relaxed(priv->io_base + _REG(VENC_INTFLAG));

	meson_crtc_irq(priv);

	return IRQ_HANDLED;
}

static int meson_dumb_create(struct drm_file *file, struct drm_device *dev,
			     struct drm_mode_create_dumb *args)
{
	/*
	 * We need 64bytes aligned stride, and PAGE aligned size
	 */
	args->pitch = ALIGN(DIV_ROUND_UP(args->width * args->bpp, 8), SZ_64);
	args->size = PAGE_ALIGN(args->pitch * args->height);

	return drm_gem_dma_dumb_create_internal(file, dev, args);
}

DEFINE_DRM_GEM_DMA_FOPS(fops);

static const struct drm_driver meson_driver = {
	.driver_features	= DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,

	/* DMA Ops */
	DRM_GEM_DMA_DRIVER_OPS_WITH_DUMB_CREATE(meson_dumb_create),

	/* Misc */
	.fops			= &fops,
	.name			= DRIVER_NAME,
	.desc			= DRIVER_DESC,
	.date			= "20161109",
	.major			= 1,
	.minor			= 0,
};

static bool meson_vpu_has_available_connectors(struct device *dev)
{
	struct device_node *ep, *remote;

	/* Parses each endpoint and check if remote exists */
	for_each_endpoint_of_node(dev->of_node, ep) {
		/* If the endpoint node exists, consider it enabled */
		remote = of_graph_get_remote_port(ep);
		if (remote) {
			of_node_put(remote);
			of_node_put(ep);
			return true;
		}
	}

	return false;
}

static struct regmap_config meson_regmap_config = {
	.reg_bits       = 32,
	.val_bits       = 32,
	.reg_stride     = 4,
	.max_register   = 0x1000,
};

static int meson_cvbs_dac_phy_init(struct meson_drm *priv)
{
	struct platform_device *pdev;
	const char *platform_id_name;

	priv->cvbs_dac = devm_phy_optional_get(priv->dev, "cvbs-dac");
	if (IS_ERR(priv->cvbs_dac))
		return dev_err_probe(priv->dev, PTR_ERR(priv->cvbs_dac),
				     "Failed to get the 'cvbs-dac' PHY\n");
	else if (priv->cvbs_dac)
		return 0;

	switch (priv->compat) {
	case VPU_COMPATIBLE_GXBB:
		platform_id_name = "meson-gxbb-cvbs-dac";
		break;
	case VPU_COMPATIBLE_GXL:
	case VPU_COMPATIBLE_GXM:
		platform_id_name = "meson-gxl-cvbs-dac";
		break;
	case VPU_COMPATIBLE_G12A:
		platform_id_name = "meson-g12a-cvbs-dac";
		break;
	default:
		return dev_err_probe(priv->dev, -EINVAL,
				     "No CVBS DAC platform ID found\n");
	}

	pdev = platform_device_register_data(priv->dev, platform_id_name,
					     PLATFORM_DEVID_AUTO, NULL, 0);
	if (IS_ERR(pdev))
		return dev_err_probe(priv->dev, PTR_ERR(pdev),
				     "Failed to register fallback CVBS DAC PHY platform device\n");

	priv->cvbs_dac = platform_get_drvdata(pdev);
	if (IS_ERR(priv->cvbs_dac)) {
		platform_device_unregister(pdev);
		return dev_err_probe(priv->dev, PTR_ERR(priv->cvbs_dac),
				     "Failed to get the 'cvbs-dac' PHY from it's platform device\n");
	}

	dev_warn(priv->dev, "Using fallback for old .dtbs without CVBS DAC\n");

	priv->cvbs_dac_pdev = pdev;

	return 0;
}

static void meson_cvbs_dac_phy_exit(struct meson_drm *priv)
{
	platform_device_unregister(priv->cvbs_dac_pdev);
}

static void meson_vpu_init(struct meson_drm *priv)
{
	if (meson_vpu_is_compatible(priv, VPU_COMPATIBLE_M8) ||
	    meson_vpu_is_compatible(priv, VPU_COMPATIBLE_M8B) ||
	    meson_vpu_is_compatible(priv, VPU_COMPATIBLE_M8M2)) {
		writel(0x0, priv->io_base + _REG(VPU_MEM_PD_REG0));
		writel(0x0, priv->io_base + _REG(VPU_MEM_PD_REG1));
	} else {
		u32 value;

		/*
		* Slave dc0 and dc5 connected to master port 1.
		* By default other slaves are connected to master port 0.
		*/
		value = VPU_RDARB_SLAVE_TO_MASTER_PORT(0, 1) |
			VPU_RDARB_SLAVE_TO_MASTER_PORT(5, 1);
		writel_relaxed(value,
			       priv->io_base + _REG(VPU_RDARB_MODE_L1C1));

		/* Slave dc0 connected to master port 1 */
		value = VPU_RDARB_SLAVE_TO_MASTER_PORT(0, 1);
		writel_relaxed(value,
			       priv->io_base + _REG(VPU_RDARB_MODE_L1C2));

		/* Slave dc4 and dc7 connected to master port 1 */
		value = VPU_RDARB_SLAVE_TO_MASTER_PORT(4, 1) |
			VPU_RDARB_SLAVE_TO_MASTER_PORT(7, 1);
		writel_relaxed(value,
			       priv->io_base + _REG(VPU_RDARB_MODE_L2C1));

		/* Slave dc1 connected to master port 1 */
		value = VPU_RDARB_SLAVE_TO_MASTER_PORT(1, 1);
		writel_relaxed(value,
			       priv->io_base + _REG(VPU_WRARB_MODE_L2C1));
	}
}

static int meson_video_clock_init(struct meson_drm *priv)
{
	int ret;

	ret = clk_bulk_prepare(VPU_VID_CLK_NUM, priv->vid_clks);
	if (ret)
		return dev_err_probe(priv->dev, ret,
				     "Failed to prepare the video clocks\n");

	ret = clk_bulk_prepare(priv->num_intr_clks, priv->intr_clks);
	if (ret)
		return dev_err_probe(priv->dev, ret,
				     "Failed to prepare the interrupt clocks\n");

	return 0;
}

static void meson_video_clock_exit(struct meson_drm *priv)
{
	if (priv->clk_dac_enabled)
		clk_disable(priv->clk_dac);

	if (priv->clk_venc_enabled)
		clk_disable(priv->clk_venc);

	clk_bulk_unprepare(priv->num_intr_clks, priv->intr_clks);
	clk_bulk_unprepare(VPU_VID_CLK_NUM, priv->vid_clks);
}

static void meson_fbdev_setup(struct meson_drm *priv)
{
	unsigned int preferred_bpp;

	/*
	 * All SoC generations before GXBB don't have a way to configure the
	 * alpha value for DRM_FORMAT_XRGB8888 and DRM_FORMAT_XBGR8888. These
	 * formats have an X component instead of an alpha component. On
	 * Meson8/8b/8m2 there is no way to configure the alpha value to use
	 * instead of the X component. This results in the fact that the
	 * formats with X component are only supported on GXBB and newer. Use
	 * 24 bits per pixel and therefore DRM_FORMAT_RGB888 to get a
	 * working framebuffer console.
	 */
	if (meson_vpu_is_compatible(priv, VPU_COMPATIBLE_M8) ||
	    meson_vpu_is_compatible(priv, VPU_COMPATIBLE_M8B) ||
	    meson_vpu_is_compatible(priv, VPU_COMPATIBLE_M8M2))
		preferred_bpp = 24;
	else
		preferred_bpp = 32;

	drm_fbdev_dma_setup(priv->drm, preferred_bpp);
}

struct meson_drm_soc_attr {
	struct meson_drm_soc_limits limits;
	const struct soc_device_attribute *attrs;
};

static const struct meson_drm_soc_attr meson_drm_soc_attrs[] = {
	/* The maximum frequency of HDMI PHY on Meson8 and Meson8m2 is ~3GHz */
	{
		.limits = {
			.max_hdmi_phy_freq = 2976000,
		},
		.attrs = (const struct soc_device_attribute []) {
			{ .soc_id = "Meson8 (S802)", },
			{ .soc_id = "Meson8m2 (S812)", },
			{ /* sentinel */ },
		}
	},
	/*
	 * GXL S805X/S805Y HDMI PLL won't lock for HDMI PHY freq > 1,65GHz.
	 * Meson8b (S805) only supports "1200p@60 max resolution" according to
	 * the public datasheet.
	 */
	{
		.limits = {
			.max_hdmi_phy_freq = 1650000,
		},
		.attrs = (const struct soc_device_attribute []) {
			{ .soc_id = "GXL (S805*)", },
			{ .soc_id = "Meson8b (S805)", },
			{ /* sentinel */ }
		}
	},
};

static int meson_drv_bind_master(struct device *dev, bool has_components)
{
	struct platform_device *pdev = to_platform_device(dev);
	const struct meson_drm_match_data *match;
	struct meson_drm *priv;
	struct drm_device *drm;
	struct resource *res;
	void __iomem *regs;
	int ret, i;

	/* Checks if an output connector is available */
	if (!meson_vpu_has_available_connectors(dev)) {
		dev_err(dev, "No output connector available\n");
		return -ENODEV;
	}

	match = of_device_get_match_data(dev);
	if (!match)
		return -ENODEV;

	drm = drm_dev_alloc(&meson_driver, dev);
	if (IS_ERR(drm))
		return PTR_ERR(drm);

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		ret = -ENOMEM;
		goto free_drm;
	}
	drm->dev_private = priv;
	priv->drm = drm;
	priv->dev = dev;
	priv->compat = match->compat;
	priv->afbcd.ops = match->afbcd_ops;

	if (meson_vpu_is_compatible(priv, VPU_COMPATIBLE_M8) ||
	    meson_vpu_is_compatible(priv, VPU_COMPATIBLE_M8B) ||
	    meson_vpu_is_compatible(priv, VPU_COMPATIBLE_M8M2)) {
		priv->vid_pll_resets[VPU_RESET_VID_PLL_PRE].id = "vid_pll_pre";
		priv->vid_pll_resets[VPU_RESET_VID_PLL_POST].id = "vid_pll_post";
		priv->vid_pll_resets[VPU_RESET_VID_PLL_SOFT_PRE].id = "vid_pll_soft_pre";
		priv->vid_pll_resets[VPU_RESET_VID_PLL_SOFT_POST].id = "vid_pll_soft_post";

		ret = devm_reset_control_bulk_get_exclusive(dev,
							    VPU_RESET_VID_PLL_NUM,
							    priv->vid_pll_resets);
		if (ret)
			goto free_drm;

		priv->intr_clks[0].id = "vpu_intr";
		priv->intr_clks[1].id = "hdmi_intr_sync";
		priv->intr_clks[2].id = "venci_int";
		priv->num_intr_clks = 3;

		ret = devm_clk_bulk_get(dev, priv->num_intr_clks,
					priv->intr_clks);
		if (ret)
			goto free_drm;

		priv->vid_clks[VPU_VID_CLK_TMDS].id = "tmds";
		priv->vid_clks[VPU_VID_CLK_HDMI_TX_PIXEL].id = "hdmi_tx_pixel";
		priv->vid_clks[VPU_VID_CLK_CTS_ENCP].id = "cts_encp";
		priv->vid_clks[VPU_VID_CLK_CTS_ENCI].id = "cts_enci";
		priv->vid_clks[VPU_VID_CLK_CTS_ENCT].id = "cts_enct";
		priv->vid_clks[VPU_VID_CLK_CTS_ENCL].id = "cts_encl";
		priv->vid_clks[VPU_VID_CLK_CTS_VDAC0].id = "cts_vdac0";

		ret = devm_clk_bulk_get(dev, VPU_VID_CLK_NUM, priv->vid_clks);
		if (ret)
			goto free_drm;
	} else {
		priv->intr_clks[0].id = "vpu_intr";
		priv->num_intr_clks = 1;

		ret = devm_clk_bulk_get_optional(dev, priv->num_intr_clks,
						 priv->intr_clks);
		if (ret)
			goto free_drm;
	}

	ret = meson_video_clock_init(priv);
	if (ret)
		goto free_drm;

	regs = devm_platform_ioremap_resource_byname(pdev, "vpu");
	if (IS_ERR(regs)) {
		ret = PTR_ERR(regs);
		goto video_clock_exit;
	}

	priv->io_base = regs;

	/*
	 * The HHI resource is optional because it contains the clocks and CVBS
	 * encoder registers. These are managed by separate drivers though.
	 */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "hhi");
	if (res) {
		/* Simply ioremap since it may be a shared register zone */
		regs = devm_ioremap(dev, res->start, resource_size(res));
		if (!regs) {
			ret = -EADDRNOTAVAIL;
			goto video_clock_exit;
		}

		priv->hhi = devm_regmap_init_mmio(dev, regs,
						  &meson_regmap_config);
		if (IS_ERR(priv->hhi)) {
			dev_err(&pdev->dev,
				"Couldn't create the HHI regmap\n");
			ret = PTR_ERR(priv->hhi);
			goto video_clock_exit;
		}
	}

	priv->canvas = meson_canvas_get(dev);
	if (IS_ERR(priv->canvas)) {
		ret = PTR_ERR(priv->canvas);
		goto video_clock_exit;
	}

	ret = meson_canvas_alloc(priv->canvas, &priv->canvas_id_osd1);
	if (ret)
		goto video_clock_exit;
	ret = meson_canvas_alloc(priv->canvas, &priv->canvas_id_vd1_0);
	if (ret) {
		meson_canvas_free(priv->canvas, priv->canvas_id_osd1);
		goto video_clock_exit;
	}
	ret = meson_canvas_alloc(priv->canvas, &priv->canvas_id_vd1_1);
	if (ret) {
		meson_canvas_free(priv->canvas, priv->canvas_id_osd1);
		meson_canvas_free(priv->canvas, priv->canvas_id_vd1_0);
		goto video_clock_exit;
	}
	ret = meson_canvas_alloc(priv->canvas, &priv->canvas_id_vd1_2);
	if (ret) {
		meson_canvas_free(priv->canvas, priv->canvas_id_osd1);
		meson_canvas_free(priv->canvas, priv->canvas_id_vd1_0);
		meson_canvas_free(priv->canvas, priv->canvas_id_vd1_1);
		goto video_clock_exit;
	}

	ret = meson_cvbs_dac_phy_init(priv);
	if (ret)
		goto free_drm;

	priv->vsync_irq = platform_get_irq(pdev, 0);

	ret = drm_vblank_init(drm, 1);
	if (ret)
		goto exit_cvbs_dac_phy;

	/* Assign limits per soc revision/package */
	for (i = 0 ; i < ARRAY_SIZE(meson_drm_soc_attrs) ; ++i) {
		if (soc_device_match(meson_drm_soc_attrs[i].attrs)) {
			priv->limits = &meson_drm_soc_attrs[i].limits;
			break;
		}
	}

	/*
	 * Remove early framebuffers (ie. simplefb). The framebuffer can be
	 * located anywhere in RAM
	 */
	ret = drm_aperture_remove_framebuffers(&meson_driver);
	if (ret)
		goto exit_cvbs_dac_phy;

	ret = drmm_mode_config_init(drm);
	if (ret)
		goto exit_cvbs_dac_phy;
	drm->mode_config.max_width = 3840;
	drm->mode_config.max_height = 2160;
	drm->mode_config.funcs = &meson_mode_config_funcs;
	drm->mode_config.helper_private	= &meson_mode_config_helpers;

	/* Hardware Initialization */

	meson_vpu_init(priv);
	meson_venc_init(priv);
	meson_vpp_init(priv);
	meson_viu_init(priv);
	if (priv->afbcd.ops) {
		ret = priv->afbcd.ops->init(priv);
		if (ret)
			goto exit_cvbs_dac_phy;
	}

	/* Encoder Initialization */

	ret = meson_encoder_cvbs_init(priv);
	if (ret)
		goto exit_afbcd;

	if (has_components) {
		ret = component_bind_all(dev, drm);
		if (ret) {
			dev_err(drm->dev, "Couldn't bind all components\n");
			/* Do not try to unbind */
			has_components = false;
			goto exit_cvbs_dac_phy;
		}
	}

	ret = meson_encoder_hdmi_init(priv);
	if (ret)
		goto exit_afbcd;

	if (meson_vpu_is_compatible(priv, VPU_COMPATIBLE_G12A)) {
		ret = meson_encoder_dsi_init(priv);
		if (ret)
			goto exit_afbcd;
	}

	ret = meson_plane_create(priv);
	if (ret)
		goto exit_afbcd;

	ret = meson_overlay_create(priv);
	if (ret)
		goto exit_afbcd;

	ret = meson_crtc_create(priv);
	if (ret)
		goto exit_afbcd;

	ret = request_irq(priv->vsync_irq, meson_irq, 0, drm->driver->name, drm);
	if (ret)
		goto exit_afbcd;

	drm_mode_config_reset(drm);

	drm_kms_helper_poll_init(drm);

	platform_set_drvdata(pdev, priv);

	ret = drm_dev_register(drm, 0);
	if (ret)
		goto uninstall_irq;

	meson_fbdev_setup(priv);

	return 0;

uninstall_irq:
	free_irq(priv->vsync_irq, drm);
exit_afbcd:
	if (priv->afbcd.ops)
		priv->afbcd.ops->exit(priv);
exit_cvbs_dac_phy:
	meson_cvbs_dac_phy_exit(priv);
video_clock_exit:
	meson_video_clock_exit(priv);
free_drm:
	drm_dev_put(drm);

	meson_encoder_dsi_remove(priv);
	meson_encoder_hdmi_remove(priv);
	meson_encoder_cvbs_remove(priv);

	if (has_components)
		component_unbind_all(dev, drm);

	return ret;
}

static int meson_drv_bind(struct device *dev)
{
	return meson_drv_bind_master(dev, true);
}

static void meson_drv_unbind(struct device *dev)
{
	struct meson_drm *priv = dev_get_drvdata(dev);
	struct drm_device *drm = priv->drm;

	if (priv->canvas) {
		meson_canvas_free(priv->canvas, priv->canvas_id_osd1);
		meson_canvas_free(priv->canvas, priv->canvas_id_vd1_0);
		meson_canvas_free(priv->canvas, priv->canvas_id_vd1_1);
		meson_canvas_free(priv->canvas, priv->canvas_id_vd1_2);
	}

	drm_dev_unregister(drm);
	drm_kms_helper_poll_fini(drm);
	drm_atomic_helper_shutdown(drm);
	free_irq(priv->vsync_irq, drm);
	drm_dev_put(drm);

	meson_encoder_dsi_remove(priv);
	meson_encoder_hdmi_remove(priv);
	meson_encoder_cvbs_remove(priv);

	component_unbind_all(dev, drm);

	if (priv->afbcd.ops)
		priv->afbcd.ops->exit(priv);

	meson_cvbs_dac_phy_exit(priv);

	meson_video_clock_exit(priv);
}

static const struct component_master_ops meson_drv_master_ops = {
	.bind	= meson_drv_bind,
	.unbind	= meson_drv_unbind,
};

static int __maybe_unused meson_drv_pm_suspend(struct device *dev)
{
	struct meson_drm *priv = dev_get_drvdata(dev);

	if (!priv)
		return 0;

	// TODO: video clock suspend

	return drm_mode_config_helper_suspend(priv->drm);
}

static int __maybe_unused meson_drv_pm_resume(struct device *dev)
{
	struct meson_drm *priv = dev_get_drvdata(dev);

	if (!priv)
		return 0;

	meson_video_clock_init(priv);
	meson_vpu_init(priv);
	meson_venc_init(priv);
	meson_vpp_init(priv);
	meson_viu_init(priv);
	if (priv->afbcd.ops)
		priv->afbcd.ops->init(priv);

	return drm_mode_config_helper_resume(priv->drm);
}

static void meson_drv_shutdown(struct platform_device *pdev)
{
	struct meson_drm *priv = dev_get_drvdata(&pdev->dev);

	if (!priv)
		return;

	drm_kms_helper_poll_fini(priv->drm);
	drm_atomic_helper_shutdown(priv->drm);
}

/*
 * Only devices to use as components
 * TOFIX: get rid of components when we can finally
 * get meson_dx_hdmi to stop using the meson_drm
 * private structure for HHI registers.
 */
static const struct of_device_id components_dev_match[] = {
	{ .compatible = "amlogic,meson-gxbb-dw-hdmi" },
	{ .compatible = "amlogic,meson-gxl-dw-hdmi" },
	{ .compatible = "amlogic,meson-gxm-dw-hdmi" },
	{ .compatible = "amlogic,meson-g12a-dw-hdmi" },
	{}
};

static int meson_drv_probe(struct platform_device *pdev)
{
	struct component_match *match = NULL;
	struct device_node *np = pdev->dev.of_node;
	struct device_node *ep, *remote;
	int count = 0;

	for_each_endpoint_of_node(np, ep) {
		remote = of_graph_get_remote_port_parent(ep);
		if (!remote || !of_device_is_available(remote)) {
			of_node_put(remote);
			continue;
		}

		if (of_match_node(components_dev_match, remote)) {
			component_match_add(&pdev->dev, &match, component_compare_of, remote);

			dev_dbg(&pdev->dev, "parent %pOF remote match add %pOF parent %s\n",
				np, remote, dev_name(&pdev->dev));
		}

		of_node_put(remote);

		++count;
	}

	if (count && !match)
		return meson_drv_bind_master(&pdev->dev, false);

	/* If some endpoints were found, initialize the nodes */
	if (count) {
		dev_info(&pdev->dev, "Queued %d outputs on vpu\n", count);

		return component_master_add_with_match(&pdev->dev,
						       &meson_drv_master_ops,
						       match);
	}

	/* If no output endpoints were available, simply bail out */
	return 0;
};

static void meson_drv_remove(struct platform_device *pdev)
{
	component_master_del(&pdev->dev, &meson_drv_master_ops);
}

static struct meson_drm_match_data meson_drm_m8_data = {
	.compat = VPU_COMPATIBLE_M8,
};

static struct meson_drm_match_data meson_drm_m8b_data = {
	.compat = VPU_COMPATIBLE_M8B,
};

static struct meson_drm_match_data meson_drm_m8m2_data = {
	.compat = VPU_COMPATIBLE_M8M2,
};

static struct meson_drm_match_data meson_drm_gxbb_data = {
	.compat = VPU_COMPATIBLE_GXBB,
};

static struct meson_drm_match_data meson_drm_gxl_data = {
	.compat = VPU_COMPATIBLE_GXL,
};

static struct meson_drm_match_data meson_drm_gxm_data = {
	.compat = VPU_COMPATIBLE_GXM,
	.afbcd_ops = &meson_afbcd_gxm_ops,
};

static struct meson_drm_match_data meson_drm_g12a_data = {
	.compat = VPU_COMPATIBLE_G12A,
	.afbcd_ops = &meson_afbcd_g12a_ops,
};

static const struct of_device_id dt_match[] = {
	{ .compatible = "amlogic,meson8-vpu",
	  .data       = (void *)&meson_drm_m8_data },
	{ .compatible = "amlogic,meson8b-vpu",
	  .data       = (void *)&meson_drm_m8b_data },
	{ .compatible = "amlogic,meson8m2-vpu",
	  .data       = (void *)&meson_drm_m8m2_data },
	{ .compatible = "amlogic,meson-gxbb-vpu",
	  .data       = (void *)&meson_drm_gxbb_data },
	{ .compatible = "amlogic,meson-gxl-vpu",
	  .data       = (void *)&meson_drm_gxl_data },
	{ .compatible = "amlogic,meson-gxm-vpu",
	  .data       = (void *)&meson_drm_gxm_data },
	{ .compatible = "amlogic,meson-g12a-vpu",
	  .data       = (void *)&meson_drm_g12a_data },
	{}
};
MODULE_DEVICE_TABLE(of, dt_match);

static const struct dev_pm_ops meson_drv_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(meson_drv_pm_suspend, meson_drv_pm_resume)
};

static struct platform_driver meson_drm_platform_driver = {
	.probe      = meson_drv_probe,
	.remove_new = meson_drv_remove,
	.shutdown   = meson_drv_shutdown,
	.driver     = {
		.name	= "meson-drm",
		.of_match_table = dt_match,
		.pm = &meson_drv_pm_ops,
	},
};

drm_module_platform_driver(meson_drm_platform_driver);

MODULE_AUTHOR("Jasper St. Pierre <jstpierre@mecheye.net>");
MODULE_AUTHOR("Neil Armstrong <narmstrong@baylibre.com>");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
