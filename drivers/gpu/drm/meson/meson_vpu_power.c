#include <linux/clk.h>
#include <linux/kernel.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>

#include "meson_drv.h"

static int meson_vpu_pd_init(struct meson_drm *priv)
{
	static char *pd_names[] = { "vpu_hdmi",
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
	struct device *genpd_dev, *dev = priv->dev;
	int ret, i;

	if (meson_vpu_is_compatible(priv, "amlogic,meson-gxl-dw-hdmi") ||
	    meson_vpu_is_compatible(priv, "amlogic,meson-gxm-dw-hdmi"))
		priv->num_vpu_pds = ARRAY_SIZE(pd_names);
	else
		priv->num_vpu_pds = ARRAY_SIZE(pd_names) - 1;

	priv->vpu_pds = devm_kcalloc(dev, priv->num_vpu_pds,
				     sizeof(*priv->vpu_pds), GFP_KERNEL);
	if (!priv->vpu_pds)
		return -ENOMEM;

	for (i = 0; i < priv->num_vpu_pds; i++) {
		genpd_dev = genpd_dev_pm_attach_by_name(dev, pd_names[i]);
		if (IS_ERR_OR_NULL(genpd_dev)) {
			dev_err(dev, "Failed to attach %s power-domain\n",
				pd_names[i]);
			ret = PTR_ERR(genpd_dev);
			goto err;
		}

		priv->vpu_pds[i].genpd_dev = genpd_dev;
	}

	genpd_dev = genpd_dev_pm_attach_by_name(dev, "vpu_hdmi_iso");
	if (IS_ERR_OR_NULL(genpd_dev)) {
		dev_err(dev, "Failed to attach vpu_hdmi_iso power-domain\n");
		ret = PTR_ERR(genpd_dev);
		goto err;
	}

	priv->vpu_hdmi_iso_pd.genpd_dev = genpd_dev;

	return 0;

err:
	while (--i >= 0)
		dev_pm_domain_detach(priv->vpu_pds[i].genpd_dev, false);

	return ret;
}

static void meson_vpu_pd_remove(struct meson_drm *priv)
{
	int i;

	for (i = (priv->num_vpu_pds - 1); i >= 0; i--)
		dev_pm_domain_detach(priv->vpu_pds[i].genpd_dev, true);
}

static int meson_vpu_genpd_add_link(struct device *dev,
				    struct meson_drm_pd *pd)
{
	pd->pd_link = device_link_add(dev, pd->genpd_dev,
				      DL_FLAG_STATELESS |
				      DL_FLAG_PM_RUNTIME |
				      DL_FLAG_RPM_ACTIVE);
	if (IS_ERR(pd->pd_link))
		return PTR_ERR(pd->pd_link);

	return 0;
}

static int meson_vpu_genpd_add_links(struct meson_drm *priv)
{
	struct meson_drm_pd *pd;
	int ret, i;

	for (i = 0; i < priv->num_vpu_pds; i++) {
		pd = &priv->vpu_pds[i];

		ret = meson_vpu_genpd_add_link(priv->dev, pd);
		if (ret)
			goto err;
	}

	udelay(20);

	return 0;

err:
	while (--i >= 0)
		device_link_del(priv->vpu_pds[i].pd_link);

	return ret;
}

static void meson_vpu_genpd_del_links(struct meson_drm *priv)
{
	int i;

	for (i = 0; i < priv->num_vpu_pds; i++)
		device_link_del(priv->vpu_pds[i].pd_link);
}

int meson_vpu_power_init(struct meson_drm *priv)
{
	int pd_count, ret;

	pd_count = of_property_count_strings(priv->dev->of_node,
					     "power-domain-names");
	if (pd_count <= 1) {
		dev_info(priv->dev,
			 "Power is managed by the legacy power-domain\n");
		return 0;
	}

	priv->resets = devm_reset_control_array_get(priv->dev, false, false);
	if (IS_ERR(priv->resets)) {
		dev_err(priv->dev, "Failed to get the reset lines\n");
		return PTR_ERR(priv->resets);
	}

	priv->vpu_clk = devm_clk_get(priv->dev, "vpu");
	if (IS_ERR(priv->vpu_clk)) {
		dev_err(priv->dev, "vpu clock request failed\n");
		return PTR_ERR(priv->vpu_clk);
	}

	if (meson_vpu_is_compatible(priv, "amlogic,meson-gxbb-dw-hdmi") ||
	    meson_vpu_is_compatible(priv, "amlogic,meson-gxl-dw-hdmi") ||
	    meson_vpu_is_compatible(priv, "amlogic,meson-gxm-dw-hdmi")) {
		priv->vapb_clk = devm_clk_get(priv->dev, "vapb");
		if (IS_ERR(priv->vapb_clk)) {
			dev_err(priv->dev, "vapb clock request failed\n");
			return PTR_ERR(priv->vapb_clk);
		}
	}

	ret = meson_vpu_pd_init(priv);
	if (ret)
		return ret;

	ret = meson_vpu_genpd_add_links(priv);
	if (ret)
		goto pds_remove;

	ret = reset_control_assert(priv->resets);
	if (ret) {
		dev_err(priv->dev, "Failed to assert reset lines\n");
		goto vpu_pd_del_links;
	}

	/* take the VPU HDMI out of isolation: */
	ret = meson_vpu_genpd_add_link(priv->dev, &priv->vpu_hdmi_iso_pd);
	if (ret)
		goto vpu_pd_del_links;

	ret = reset_control_deassert(priv->resets);
	if (ret) {
		dev_err(priv->dev, "Failed to deassert reset lines\n");
		goto hdmi_iso_del_link;
	}

	ret = clk_prepare_enable(priv->vpu_clk);
	if (ret) {
		dev_err(priv->dev, "Failed to enable 'vpu' clock\n");
		goto assert_resets;
	}

	ret = clk_prepare_enable(priv->vapb_clk);
	if (ret) {
		dev_err(priv->dev, "Failed to enable 'vapb' clock\n");
		goto disable_vpu_clk;
	}

	return 0;

disable_vpu_clk:
	clk_disable_unprepare(priv->vpu_clk);
assert_resets:
	reset_control_assert(priv->resets);
hdmi_iso_del_link:
	device_link_del(priv->vpu_hdmi_iso_pd.pd_link);
vpu_pd_del_links:
	meson_vpu_genpd_del_links(priv);
pds_remove:
	meson_vpu_pd_remove(priv);

	return ret;
}

void meson_vpu_power_exit(struct meson_drm *priv)
{
	clk_disable_unprepare(priv->vapb_clk);
	clk_disable_unprepare(priv->vpu_clk);

	reset_control_assert(priv->resets);

	device_link_del(priv->vpu_hdmi_iso_pd.pd_link);

	meson_vpu_genpd_del_links(priv);

	meson_vpu_pd_remove(priv);
}
