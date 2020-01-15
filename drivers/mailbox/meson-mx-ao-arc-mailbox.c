// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2020 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/mailbox_controller.h>

#define MESON_MX_AO_ARC_NUM_CHANS	3

struct meson_mx_ao_arc_mbox {
	struct mbox_chan		chans[MESON_MX_AO_ARC_NUM_CHANS];
	struct mbox_controller		mbox;
};

static void* __iomem meson_mx_ao_arc_mbox_iomem(struct mbox_chan *chan)
{
	return (void* __iomem)chan->con_priv;
}

static bool meson_mx_ao_arc_mbox_rts(struct mbox_chan *chan)
{
	return readl_relaxed(meson_mx_ao_arc_mbox_iomem(chan)) == 0;
}

static int meson_mx_ao_arc_mbox_send_data(struct mbox_chan *chan, void *data)
{

	u32 *arg = data;

	if (!meson_mx_ao_arc_mbox_rts(chan))
		return -EBUSY;

	writel_relaxed(*arg, meson_mx_ao_arc_mbox_iomem(chan));

	return 0;
}

static int meson_mx_ao_arc_mbox_flush(struct mbox_chan *chan,
				      unsigned long timeout)
{
	timeout = jiffies + msecs_to_jiffies(timeout);

	while (time_before(jiffies, timeout)) {
		if (meson_mx_ao_arc_mbox_rts(chan))
			return 0;

		udelay(5);
	}

	return -ETIMEDOUT;
}

static int meson_mx_ao_arc_mbox_startup(struct mbox_chan *chan)
{
	u32 data = 0;

	return meson_mx_ao_arc_mbox_send_data(chan, &data);
}

static bool meson_mx_ao_arc_mbox_last_tx_done(struct mbox_chan *chan)
{
	return meson_mx_ao_arc_mbox_rts(chan);
}

static const struct mbox_chan_ops meson_mx_ao_arc_mbox_ops = {
	.send_data	= meson_mx_ao_arc_mbox_send_data,
	.flush		= meson_mx_ao_arc_mbox_flush,
	.startup	= meson_mx_ao_arc_mbox_startup,
	.last_tx_done	= meson_mx_ao_arc_mbox_last_tx_done,
};

static int meson_mx_ao_arc_mbox_probe(struct platform_device *pdev)
{
	struct meson_mx_ao_arc_mbox *priv;
	struct device *dev = &pdev->dev;
	void __iomem *base;
	int i, ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	for (i = 0; i < MESON_MX_AO_ARC_NUM_CHANS; i++)
		priv->chans[i].con_priv = base + (i * sizeof(u32));

	priv->mbox.dev = dev;
	priv->mbox.ops = &meson_mx_ao_arc_mbox_ops;
	priv->mbox.txpoll_period = 5;
	priv->mbox.txdone_poll = true;
	priv->mbox.chans = priv->chans;
	priv->mbox.num_chans = MESON_MX_AO_ARC_NUM_CHANS;

	ret = devm_mbox_controller_register(dev, &priv->mbox);
	if (ret) {
		dev_err(dev, "Failed to register AO ARC mailbox controller\n");
		return ret;
	}

	return 0;
}

static const struct of_device_id meson_mx_ao_arc_mbox_of_match[] = {
	{ .compatible = "amlogic,meson-mx-ao-arc-mbox" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, meson_mx_ao_arc_mbox_of_match);

static struct platform_driver meson_mx_ao_arc_mbox_driver = {
	.probe = meson_mx_ao_arc_mbox_probe,
	.driver = {
		.name = "meson-mx-ao-arc-mbox",
		.of_match_table = meson_mx_ao_arc_mbox_of_match,
	},
};
module_platform_driver(meson_mx_ao_arc_mbox_driver);

MODULE_DESCRIPTION("Amlogic Meson6/8/8b/8m2 AO ARC mailbox driver");
MODULE_AUTHOR("Martin Blumenstingl <martin.blumenstingl@googlemail.com>");
MODULE_LICENSE("GPL v2");
