/*
 * Copyright (C) 2016 BayLibre, SAS
 * Author: Neil Armstrong <narmstrong@baylibre.com>
 * Copyright (C) 2014 Endless Mobile
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 *
 * Written by:
 *     Jasper St. Pierre <jstpierre@mecheye.net>
 */

#include <linux/clk.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/component.h>
#include <linux/of_graph.h>

#include <drm/drmP.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_flip_work.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_plane_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_rect.h>
#include <drm/drm_fb_helper.h>

#include "meson_drv.h"
#include "meson_plane.h"
#include "meson_crtc.h"
#include "meson_venc_cvbs.h"

#include "meson_vpp.h"
#include "meson_viu.h"
#include "meson_venc.h"
#include "meson_canvas.h"
#include "meson_registers.h"

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

static void meson_fb_output_poll_changed(struct drm_device *dev)
{
	struct meson_drm *priv = dev->dev_private;

	drm_fbdev_cma_hotplug_event(priv->fbdev);
}

static const struct drm_mode_config_funcs meson_mode_config_funcs = {
	.output_poll_changed = meson_fb_output_poll_changed,
	.atomic_check        = drm_atomic_helper_check,
	.atomic_commit       = drm_atomic_helper_commit,
	.fb_create           = drm_gem_fb_create,
};

static irqreturn_t meson_irq(int irq, void *arg)
{
	struct drm_device *dev = arg;
	struct meson_drm *priv = dev->dev_private;

	(void)readl_relaxed(priv->io_base + _REG(VENC_INTFLAG));

	meson_crtc_irq(priv);

	return IRQ_HANDLED;
}

DEFINE_DRM_GEM_CMA_FOPS(fops);

static struct drm_driver meson_driver = {
	.driver_features	= DRIVER_HAVE_IRQ | DRIVER_GEM |
				  DRIVER_MODESET | DRIVER_PRIME |
				  DRIVER_ATOMIC,

	/* IRQ */
	.irq_handler		= meson_irq,

	/* PRIME Ops */
	.prime_handle_to_fd	= drm_gem_prime_handle_to_fd,
	.prime_fd_to_handle	= drm_gem_prime_fd_to_handle,
	.gem_prime_import	= drm_gem_prime_import,
	.gem_prime_export	= drm_gem_prime_export,
	.gem_prime_get_sg_table	= drm_gem_cma_prime_get_sg_table,
	.gem_prime_import_sg_table = drm_gem_cma_prime_import_sg_table,
	.gem_prime_vmap		= drm_gem_cma_prime_vmap,
	.gem_prime_vunmap	= drm_gem_cma_prime_vunmap,
	.gem_prime_mmap		= drm_gem_cma_prime_mmap,

	/* GEM Ops */
	.dumb_create		= drm_gem_cma_dumb_create,
	.gem_free_object_unlocked = drm_gem_cma_free_object,
	.gem_vm_ops		= &drm_gem_cma_vm_ops,

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
		if (remote)
			return true;
	}

	return false;
}

static struct regmap_config meson_regmap_config = {
	.reg_bits       = 32,
	.val_bits       = 32,
	.reg_stride     = 4,
	.max_register   = 0x1000,
};

static void meson_vpu_init(struct meson_drm *priv)
{
	if (meson_vpu_is_compatible(priv, "amlogic,meson-gxbb-vpu") ||
	    meson_vpu_is_compatible(priv, "amlogic,meson-gxl-vpu") ||
	    meson_vpu_is_compatible(priv, "amlogic,meson-gxm-vpu")) {
		writel_relaxed(0x210000,
			       priv->io_base + _REG(VPU_RDARB_MODE_L1C1));
		writel_relaxed(0x10000,
			       priv->io_base + _REG(VPU_RDARB_MODE_L1C2));
		writel_relaxed(0x900000,
			       priv->io_base + _REG(VPU_RDARB_MODE_L2C1));
		writel_relaxed(0x20000,
			       priv->io_base + _REG(VPU_WRARB_MODE_L2C1));
	}
}

static int meson_vpu_genpd_dev_init(struct device *dev, struct meson_drm *priv)
{
	static char *pd_names[] = { "vpu_hdmi",
				    "hdmi_memory",
				    "viu_osd1_memory",
				    "viu_osd2_memory",
				    "viu_vd1_memory",
				    "viu_vd2_memory",
				    "viu_chroma",
				    "viu_output_fifo",
				    "viu_scaler_memory",
				    "viu_osd_scaler_memory",
				    "viu_vdin0_memory",
				    "viu_vdin1_memory",
				    "pic_rot1",
				    "pic_rot2",
				    "pic_rot3",
				    "deinterlacer_pre",
				    "deinterlacer_post",
				    "sharp",
				    "viu2_osd1",
				    "viu2_osd2",
				    "viu2_vd1",
				    "viu2_chroma",
				    "viu2_output_fifo",
				    "viu2_scaler_memory",
				    "viu2_osd_scaler_memory",
				    "arb",
				    "afbc_dec",
				    "vpuarb2_am_async",
				    "vencp",
				    "vencl",
				    "venci",
				    "isp",
				    "ldim_stts",
				    "xvycc_lut",
				    "viu1_wm" };
	struct device *genpd_dev;
	int ret, i;

	priv->num_vpu_genpd_devs = ARRAY_SIZE(pd_names);

	priv->vpu_genpd_devs = devm_kcalloc(dev, priv->num_vpu_genpd_devs,
					    sizeof(*priv->vpu_genpd_devs),
					    GFP_KERNEL);
	if (!priv->vpu_genpd_devs)
		return -ENOMEM;

	for (i = 0; i < priv->num_vpu_genpd_devs; i++) {
		genpd_dev = genpd_dev_pm_attach_by_name(dev, pd_names[i]);
		if (IS_ERR_OR_NULL(genpd_dev)) {
			dev_err(dev, "Failed to attach %s power-domain\n",
				pd_names[i]);
			ret = PTR_ERR(genpd_dev);
			goto err;
		}

		priv->vpu_genpd_devs[i] = genpd_dev;
	}

	genpd_dev = genpd_dev_pm_attach_by_name(dev, "vpu_hdmi_iso");
	if (IS_ERR_OR_NULL(genpd_dev)) {
		dev_err(dev, "Failed to attach vpu_hdmi_iso power-domain\n");
		ret = PTR_ERR(genpd_dev);
		goto err;
	}

	priv->vpu_hdmi_iso_genpd_dev = genpd_dev;

	return 0;

err:
	while (--i >= 0)
		dev_pm_domain_detach(priv->vpu_genpd_devs[i], false);

	return ret;
}

static int meson_vpu_genpd_add_links(struct device *dev,
				     struct meson_drm *priv)
{
	int i;

	// TODO: error checking
	for (i = 0; i < priv->num_vpu_genpd_devs; i++)
		device_link_add(dev, priv->vpu_genpd_devs[i],
				DL_FLAG_STATELESS | DL_FLAG_PM_RUNTIME | DL_FLAG_RPM_ACTIVE);

	udelay(20);

	return 0;
}

static int meson_drv_bind_master(struct device *dev, bool has_components)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct meson_drm *priv;
	struct drm_device *drm;
	struct resource *res;
	void __iomem *regs;
	int ret;

	/* Checks if an output connector is available */
	if (!meson_vpu_has_available_connectors(dev)) {
		dev_err(dev, "No output connector available\n");
		return -ENODEV;
	}

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

	priv->resets = devm_reset_control_array_get(&pdev->dev, false, false);
	if (IS_ERR(priv->resets)) {
		dev_err(&pdev->dev, "Failed to get the reset lines\n");
		return PTR_ERR(priv->resets);
	}

	priv->vpu_clk = devm_clk_get(&pdev->dev, "vpu");
	if (IS_ERR(priv->vpu_clk)) {
		dev_err(&pdev->dev, "vpu clock request failed\n");
		return PTR_ERR(priv->vpu_clk);
	}

	priv->vapb_clk = devm_clk_get(&pdev->dev, "vapb");
	if (IS_ERR(priv->vapb_clk)) {
		dev_err(&pdev->dev, "vapb clock request failed\n");
		return PTR_ERR(priv->vapb_clk);
	}

	ret = meson_vpu_genpd_dev_init(&pdev->dev, priv);
	if (ret)
		return ret;

	// TODO: error check
	meson_vpu_genpd_add_links(&pdev->dev, priv);

	// TODO: error check
	reset_control_assert(priv->resets);

	// TODO: error check
	device_link_add(dev, priv->vpu_hdmi_iso_genpd_dev,
			DL_FLAG_STATELESS | DL_FLAG_PM_RUNTIME | DL_FLAG_RPM_ACTIVE);

	// TODO: error check
	reset_control_deassert(priv->resets);

	// TODO:
	clk_prepare_enable(priv->vpu_clk);
	clk_prepare_enable(priv->vapb_clk);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "vpu");
	regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(regs)) {
		ret = PTR_ERR(regs);
		goto free_drm;
	}

	priv->io_base = regs;

	priv->hhi = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
						    "amlogic,hhi-sysctrl");
	if (IS_ERR(priv->hhi)) {
		dev_warn(dev, "Falling back to parsing the 'hhi' registers\n");

		res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						   "hhi");
		if (!res) {
			ret = -EINVAL;
			goto free_drm;
		}
		/* Simply ioremap since it may be a shared register zone */
		regs = devm_ioremap(dev, res->start, resource_size(res));
		if (!regs) {
			ret = -EADDRNOTAVAIL;
			goto free_drm;
		}

		priv->hhi = devm_regmap_init_mmio(dev, regs,
						  &meson_regmap_config);
		if (IS_ERR(priv->hhi)) {
			dev_err(&pdev->dev,
				"Couldn't create the HHI regmap\n");
			ret = PTR_ERR(priv->hhi);
			goto free_drm;
		}
	}

	priv->canvas = meson_canvas_get(dev);
	if (!IS_ERR(priv->canvas)) {
		ret = meson_canvas_alloc(priv->canvas, &priv->canvas_id_osd1);
		if (ret)
			goto free_drm;
	} else {
		priv->canvas = NULL;

		res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dmc");
		if (!res) {
			ret = -EINVAL;
			goto free_drm;
		}
		/* Simply ioremap since it may be a shared register zone */
		regs = devm_ioremap(dev, res->start, resource_size(res));
		if (!regs) {
			ret = -EADDRNOTAVAIL;
			goto free_drm;
		}

		priv->dmc = devm_regmap_init_mmio(dev, regs,
						  &meson_regmap_config);
		if (IS_ERR(priv->dmc)) {
			dev_err(&pdev->dev, "Couldn't create the DMC regmap\n");
			ret = PTR_ERR(priv->dmc);
			goto free_drm;
		}
	}

	priv->vsync_irq = platform_get_irq(pdev, 0);

	ret = drm_vblank_init(drm, 1);
	if (ret)
		goto free_drm;

	drm_mode_config_init(drm);
	drm->mode_config.max_width = 3840;
	drm->mode_config.max_height = 2160;
	drm->mode_config.funcs = &meson_mode_config_funcs;

	/* Hardware Initialization */

	meson_vpu_init(priv);
	meson_venc_init(priv);
	meson_vpp_init(priv);
	meson_viu_init(priv);

	/* Encoder Initialization */

	ret = meson_venc_cvbs_create(priv);
	if (ret)
		goto free_drm;

	if (has_components) {
		ret = component_bind_all(drm->dev, drm);
		if (ret) {
			dev_err(drm->dev, "Couldn't bind all components\n");
			goto free_drm;
		}
	}

	ret = meson_plane_create(priv);
	if (ret)
		goto free_drm;

	ret = meson_crtc_create(priv);
	if (ret)
		goto free_drm;

	ret = drm_irq_install(drm, priv->vsync_irq);
	if (ret)
		goto free_drm;

	drm_mode_config_reset(drm);

	priv->fbdev = drm_fbdev_cma_init(drm, 32,
					 drm->mode_config.num_connector);
	if (IS_ERR(priv->fbdev)) {
		ret = PTR_ERR(priv->fbdev);
		goto free_drm;
	}

	drm_kms_helper_poll_init(drm);

	platform_set_drvdata(pdev, priv);

	ret = drm_dev_register(drm, 0);
	if (ret)
		goto free_drm;

	return 0;

free_drm:
	drm_dev_put(drm);

	return ret;
}

static int meson_drv_bind(struct device *dev)
{
	return meson_drv_bind_master(dev, true);
}

static void meson_drv_unbind(struct device *dev)
{
	struct drm_device *drm = dev_get_drvdata(dev);
	struct meson_drm *priv = drm->dev_private;

	if (priv->canvas)
		meson_canvas_free(priv->canvas, priv->canvas_id_osd1);

	drm_dev_unregister(drm);
	drm_kms_helper_poll_fini(drm);
	drm_fbdev_cma_fini(priv->fbdev);
	drm_mode_config_cleanup(drm);
	drm_dev_put(drm);

	// TODO: disable PDs, HDMI_ISO first

	reset_control_assert(priv->resets);

	clk_disable_unprepare(priv->vpu_clk);
	clk_disable_unprepare(priv->vapb_clk);
}

static const struct component_master_ops meson_drv_master_ops = {
	.bind	= meson_drv_bind,
	.unbind	= meson_drv_unbind,
};

static int compare_of(struct device *dev, void *data)
{
	DRM_DEBUG_DRIVER("Comparing of node %pOF with %pOF\n",
			 dev->of_node, data);

	return dev->of_node == data;
}

/* Possible connectors nodes to ignore */
static const struct of_device_id connectors_match[] = {
	{ .compatible = "composite-video-connector" },
	{ .compatible = "svideo-connector" },
	{ .compatible = "hdmi-connector" },
	{ .compatible = "dvi-connector" },
	{}
};

static int meson_probe_remote(struct platform_device *pdev,
			      struct component_match **match,
			      struct device_node *parent,
			      struct device_node *remote)
{
	struct device_node *ep, *remote_node;
	int count = 1;

	/* If node is a connector, return and do not add to match table */
	if (of_match_node(connectors_match, remote))
		return 1;

	component_match_add(&pdev->dev, match, compare_of, remote);

	for_each_endpoint_of_node(remote, ep) {
		remote_node = of_graph_get_remote_port_parent(ep);
		if (!remote_node ||
		    remote_node == parent || /* Ignore parent endpoint */
		    !of_device_is_available(remote_node))
			continue;

		count += meson_probe_remote(pdev, match, remote, remote_node);

		of_node_put(remote_node);
	}

	return count;
}

static int meson_drv_probe(struct platform_device *pdev)
{
	struct component_match *match = NULL;
	struct device_node *np = pdev->dev.of_node;
	struct device_node *ep, *remote;
	int count = 0;

	for_each_endpoint_of_node(np, ep) {
		remote = of_graph_get_remote_port_parent(ep);
		if (!remote || !of_device_is_available(remote))
			continue;

		count += meson_probe_remote(pdev, &match, np, remote);
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

static const struct of_device_id dt_match[] = {
	{ .compatible = "amlogic,meson8-vpu" },
	{ .compatible = "amlogic,meson8b-vpu" },
	{ .compatible = "amlogic,meson8m2-vpu" },
	{ .compatible = "amlogic,meson-gxbb-vpu" },
	{ .compatible = "amlogic,meson-gxl-vpu" },
	{ .compatible = "amlogic,meson-gxm-vpu" },
	{}
};
MODULE_DEVICE_TABLE(of, dt_match);

static struct platform_driver meson_drm_platform_driver = {
	.probe      = meson_drv_probe,
	.driver     = {
		.name	= "meson-drm",
		.of_match_table = dt_match,
	},
};

module_platform_driver(meson_drm_platform_driver);

MODULE_AUTHOR("Jasper St. Pierre <jstpierre@mecheye.net>");
MODULE_AUTHOR("Neil Armstrong <narmstrong@baylibre.com>");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
