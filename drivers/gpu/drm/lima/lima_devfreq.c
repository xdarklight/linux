// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2019 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 *
 * Based on panfrost_devfreq.c:
 *   Copyright 2019 Collabora ltd.
 */
#include <linux/clk.h>
#include <linux/devfreq.h>
#include <linux/devfreq_cooling.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>

#include "lima_device.h"
#include "lima_devfreq.h"

static void lima_devfreq_update_utilization(struct lima_device *ldev)
{
	ktime_t now;
	ktime_t last;

	if (!ldev->devfreq.devfreq)
		return;

	write_lock(&ldev->devfreq.rwlock);

	now = ktime_get();
	last = ldev->devfreq.time_last_update;

	if (atomic_read(&ldev->devfreq.busy_count) > 0)
		ldev->devfreq.busy_time += ktime_sub(now, last);
	else
		ldev->devfreq.idle_time += ktime_sub(now, last);

	ldev->devfreq.time_last_update = now;

	write_unlock(&ldev->devfreq.rwlock);
}

static int lima_devfreq_target(struct device *dev, unsigned long *freq,
			       u32 flags)
{
	struct dev_pm_opp *opp;
	int err;

	opp = devfreq_recommended_opp(dev, freq, flags);
	if (IS_ERR(opp))
		return PTR_ERR(opp);
	dev_pm_opp_put(opp);

	err = dev_pm_opp_set_rate(dev, *freq);
	if (err)
		return err;

	return 0;
}

static void lima_devfreq_reset(struct lima_device *ldev)
{
	write_lock(&ldev->devfreq.rwlock);

	ldev->devfreq.busy_time = 0;
	ldev->devfreq.idle_time = 0;
	ldev->devfreq.time_last_update = ktime_get();

	write_unlock(&ldev->devfreq.rwlock);
}

static int lima_devfreq_get_dev_status(struct device *dev,
				       struct devfreq_dev_status *status)
{
	struct lima_device *ldev = dev_get_drvdata(dev);

	lima_devfreq_update_utilization(ldev);

	status->current_frequency = clk_get_rate(ldev->clk_gpu);

	read_lock(&ldev->devfreq.rwlock);

	status->total_time = ktime_to_ns(ktime_add(ldev->devfreq.busy_time,
						   ldev->devfreq.idle_time));
	status->busy_time = ktime_to_ns(ldev->devfreq.busy_time);

	read_unlock(&ldev->devfreq.rwlock);

	lima_devfreq_reset(ldev);

	dev_dbg(ldev->dev, "busy %lu total %lu %lu %% freq %lu MHz\n",
		status->busy_time, status->total_time,
		status->busy_time / (status->total_time / 100),
		status->current_frequency / 1000 / 1000);

	return 0;
}

static struct devfreq_dev_profile lima_devfreq_profile = {
	.polling_ms = 50, /* ~3 frames */
	.target = lima_devfreq_target,
	.get_dev_status = lima_devfreq_get_dev_status,
};

int lima_devfreq_init(struct lima_device *ldev)
{
	struct thermal_cooling_device *cooling;
	struct device *dev = &ldev->pdev->dev;
	struct devfreq *devfreq;
	struct dev_pm_opp *opp;
	unsigned long cur_freq;
	int ret;

	rwlock_init(&ldev->devfreq.rwlock);

	ldev->devfreq.opp_table = dev_pm_opp_set_clkname(dev, "core");
	if (IS_ERR(ldev->devfreq.opp_table))
		return PTR_ERR(ldev->devfreq.opp_table);

	ret = dev_pm_opp_of_add_table(dev);
	if (ret == -ENODEV) /* Optional, continue without devfreq */
		return 0;
	else if (ret)
		return ret;

	lima_devfreq_reset(ldev);

	cur_freq = clk_get_rate(ldev->clk_gpu);

	opp = devfreq_recommended_opp(dev, &cur_freq, 0);
	if (IS_ERR(opp))
		return PTR_ERR(opp);

	lima_devfreq_profile.initial_freq = cur_freq;
	dev_pm_opp_put(opp);

	devfreq = devm_devfreq_add_device(dev, &lima_devfreq_profile,
					  DEVFREQ_GOV_SIMPLE_ONDEMAND, NULL);
	if (IS_ERR(devfreq)) {
		dev_err(dev, "Couldn't initialize GPU devfreq\n");
		dev_pm_opp_of_remove_table(dev);
		return PTR_ERR(devfreq);
	}

	ldev->devfreq.devfreq = devfreq;

	cooling = of_devfreq_cooling_register(dev->of_node, devfreq);
	if (IS_ERR(cooling))
		dev_info(dev, "Failed to register cooling device\n");
	else
		ldev->devfreq.cooling = cooling;

	return 0;
}

void lima_devfreq_fini(struct lima_device *ldev)
{
	if (ldev->devfreq.cooling)
		devfreq_cooling_unregister(ldev->devfreq.cooling);

	if (ldev->devfreq.opp_table) {
		dev_pm_opp_put_clkname(ldev->devfreq.opp_table);
		ldev->devfreq.opp_table = NULL;
	}

	dev_pm_opp_of_remove_table(&ldev->pdev->dev);
}

void lima_devfreq_record_busy(struct lima_device *ldev)
{
	lima_devfreq_update_utilization(ldev);
	atomic_inc(&ldev->devfreq.busy_count);
}

void lima_devfreq_record_idle(struct lima_device *ldev)
{
	int count;

	lima_devfreq_update_utilization(ldev);
	count = atomic_dec_if_positive(&ldev->devfreq.busy_count);
	WARN_ON(count < 0);
}
