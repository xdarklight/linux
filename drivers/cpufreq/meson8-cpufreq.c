// SPDX-License-Identifier: GPL-2.0
/*
 * Amlogic Meson8/8b/8m2 SoCs have to switch to an intermediate clock to ensure
 * stable operation while changing the CPU clock tree.
 *
 * Copyright (C) 2026 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 */

#include <linux/clk.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include "cpufreq-dt.h"

static struct platform_device *meson8_cpufreq_pdev, *cpufreq_dt_pdev;
static struct clk *cpu_clk, *xtal_clk;

static unsigned int meson8_cpufreq_get_intermediate(struct cpufreq_policy *policy,
						    unsigned int index)
{
	return clk_get_rate(xtal_clk);
}

static int meson8_cpufreq_target_intermediate(struct cpufreq_policy *policy,
					      unsigned int index)
{
	int ret;

	ret = clk_set_parent(cpu_clk, xtal_clk);
	if (ret)
		return ret;

	udelay(100);

	return 0;
}

static struct cpufreq_dt_platform_data meson8_cpufreq_dt_data = {
	.get_intermediate = meson8_cpufreq_get_intermediate,
	.target_intermediate = meson8_cpufreq_target_intermediate,
};

static void meson8_cpufreq_put_clks(void *data)
{
	clk_put(cpu_clk);
	clk_put(xtal_clk);
}

static void meson8_cpufreq_dt_pdev_unregister(void *data)
{
	platform_device_unregister(cpufreq_dt_pdev);
}

static int meson8_cpufreq_probe(struct platform_device *pdev)
{
	struct device *cpu_dev;
	int ret;

	cpu_dev = get_cpu_device(0);
	if (!cpu_dev)
		return -ENODEV;

	cpu_clk = clk_get(cpu_dev, NULL);
	if (IS_ERR(cpu_clk))
		return dev_err_probe(&pdev->dev, PTR_ERR(cpu_clk),
				     "Failed to get the 'cpu' clock\n");

	ret = devm_add_action_or_reset(&pdev->dev, meson8_cpufreq_put_clks,
				       NULL);
	if (ret)
		return ret;

	xtal_clk = clk_get(cpu_dev, "xtal");
	if (IS_ERR(xtal_clk))
		return dev_err_probe(&pdev->dev, PTR_ERR(xtal_clk),
				     "Failed to get the 'xtal' clock\n");

	cpufreq_dt_pdev = platform_device_register_data(NULL, "cpufreq-dt",
							-1,
							&meson8_cpufreq_dt_data,
							sizeof(meson8_cpufreq_dt_data));
	if (IS_ERR(cpufreq_dt_pdev))
		return dev_err_probe(&pdev->dev, PTR_ERR(cpufreq_dt_pdev),
				     "Failed to register cpufreq-dt platform device\n");

	return devm_add_action_or_reset(&pdev->dev,
					meson8_cpufreq_dt_pdev_unregister, NULL);
}

static const struct of_device_id meson8_cpufreq_of_match_list[] __initconst = {
	{ .compatible = "amlogic,meson8", },
	{ .compatible = "amlogic,meson8b", },
	{ .compatible = "amlogic,meson8m2", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, meson8_cpufreq_of_match_list);

static struct platform_driver meson8_cpufreq_driver = {
	.probe = meson8_cpufreq_probe,
	.driver = {
		.name = "meson8-cpufreq",
	},
};

static int __init meson8_cpufreq_init(void)
{
	int ret;

	if (!of_machine_device_match(meson8_cpufreq_of_match_list))
		return -ENODEV;

	ret = platform_driver_register(&meson8_cpufreq_driver);
	if (ret)
		return ret;

	meson8_cpufreq_pdev = platform_device_register_simple("meson8-cpufreq",
							      -1, NULL, 0);
	ret = PTR_ERR_OR_ZERO(meson8_cpufreq_pdev);
	if (ret)
		platform_driver_unregister(&meson8_cpufreq_driver);

	return ret;
}
module_init(meson8_cpufreq_init);

static void __exit meson8_cpufreq_exit(void)
{
	platform_device_unregister(meson8_cpufreq_pdev);
	platform_driver_unregister(&meson8_cpufreq_driver);
}
module_exit(meson8_cpufreq_exit);

MODULE_AUTHOR("Martin Blumenstingl <martin.blumenstingl@googlemail.com>");
MODULE_DESCRIPTION("Amlogic Meson8/8b/8m2 cpufreq driver");
MODULE_LICENSE("GPL v2");
