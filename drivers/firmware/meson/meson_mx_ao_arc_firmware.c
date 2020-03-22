// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2020 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 */

#include <linux/kernel.h>
#include <linux/mailbox_client.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_platform.h>
#include <linux/remoteproc.h>
#include <linux/slab.h>
#include <linux/suspend.h>

#include <asm/suspend.h>

struct meson_mx_ao_arc_fw_data {
	struct rproc		*rproc;
	struct mbox_client	mbox_cl;
	struct mbox_chan	*mbox_chan0;
	struct mbox_chan	*mbox_chan1;
};

static struct platform_device *meson_mx_ao_arc_fw_pdev;

static int meson_mx_ao_arc_fw_pm_suspend(unsigned long arg)
{
	struct meson_mx_ao_arc_fw_data *priv;
	u32 message = 't';
	int ret;

#if 0
	flush_cache_all();
	outer_flush_all();
#endif

	priv = platform_get_drvdata(meson_mx_ao_arc_fw_pdev);

	ret = mbox_send_message(priv->mbox_chan0, &message);
	if (ret) {
		pr_err("Failed to send suspend message\n");
		return ret;
	}

	ret = mbox_flush(priv->mbox_chan0, 20);
	if (ret) {
		pr_err("Failed to flush suspend message\n");
		return ret;
	}

	return 0;
}

static int meson_mx_ao_arc_fw_suspend_enter(suspend_state_t state)
{
	dev_err(&meson_mx_ao_arc_fw_pdev->dev, "%s\n", __func__);

	switch (state) {
	case PM_SUSPEND_MEM:
		return cpu_suspend(0, meson_mx_ao_arc_fw_pm_suspend);

	default:
		return -EINVAL;
	}
}

static const struct platform_suspend_ops meson_mx_ao_arc_fw_suspend_ops = {
	.valid		= suspend_valid_only_mem,
	.enter		= meson_mx_ao_arc_fw_suspend_enter,
};

static int meson_mx_ao_arc_fw_probe(struct platform_device *pdev)
{
	struct meson_mx_ao_arc_fw_data *priv;
	struct device *dev = &pdev->dev;
	phandle rproc_phandle;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	ret = of_property_read_u32(dev->of_node, "amlogic,rproc",
				   &rproc_phandle);
	if (ret) {
		dev_err(dev, "Failed to get the amlogic,rproc phandle\n");
		return ret;
	}

	priv->rproc = rproc_get_by_phandle(rproc_phandle);
	if (!priv->rproc) {
		dev_dbg(dev, "Failed to get the rproc\n");
		return -EPROBE_DEFER;
	}

	priv->mbox_cl.dev = dev;
	priv->mbox_cl.tx_block = false;
	priv->mbox_cl.knows_txdone = true;

	priv->mbox_chan0 = mbox_request_channel(&priv->mbox_cl, 0);
	if (IS_ERR(priv->mbox_chan0)) {
		dev_err(dev, "Failed to request mbox channel #0\n");
		goto err_put_rproc;
	}

	priv->mbox_chan1 = mbox_request_channel(&priv->mbox_cl, 1);
	if (IS_ERR(priv->mbox_chan1)) {
		dev_err(dev, "Failed to request mbox channel #1\n");
		goto err_free_mbox_chan0;
	}

	if (rproc_boot(priv->rproc)) {
		dev_err(dev, "Failed to boot rproc\n");
		goto err_free_mbox_chan1;
	}

	platform_set_drvdata(pdev, priv);

	meson_mx_ao_arc_fw_pdev = pdev;

	suspend_set_ops(&meson_mx_ao_arc_fw_suspend_ops);

	return 0;

err_free_mbox_chan1:
	mbox_free_channel(priv->mbox_chan1);
err_free_mbox_chan0:
	mbox_free_channel(priv->mbox_chan0);
err_put_rproc:
	rproc_put(priv->rproc);
	return ret;
}

static int meson_mx_ao_arc_fw_remove(struct platform_device *pdev)
{
	struct meson_mx_ao_arc_fw_data *priv = platform_get_drvdata(pdev);

	suspend_set_ops(NULL);

	mbox_free_channel(priv->mbox_chan1);
	mbox_free_channel(priv->mbox_chan0);

	rproc_put(priv->rproc);

	return 0;
}

static const struct of_device_id meson_mx_ao_arc_fw_of_match[] = {
	{ .compatible = "amlogic,meson8-ao-arc-firmware" },
	{ .compatible = "amlogic,meson8b-ao-arc-firmware" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, meson_mx_ao_arc_fw_of_match);

static struct platform_driver meson_mx_ao_arc_fw_driver = {
	.probe = meson_mx_ao_arc_fw_probe,
	.remove = meson_mx_ao_arc_fw_remove,
	.driver = {
		.name = "meson-mx-ao-arc-firmware",
		.of_match_table = meson_mx_ao_arc_fw_of_match,
	},
};
module_platform_driver(meson_mx_ao_arc_fw_driver);

MODULE_DESCRIPTION("Amlogic Meson6/8/8b/8m2 AO ARC firmware driver");
MODULE_AUTHOR("Martin Blumenstingl <martin.blumenstingl@googlemail.com>");
MODULE_LICENSE("GPL v2");
