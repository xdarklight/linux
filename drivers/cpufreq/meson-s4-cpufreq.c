// SPDX-License-Identifier: GPL-2.0
/*
 * Amlogic SoCs typically have to update to an intermediate clock for stable
 * operation when changing the CPU clocks.
 * Meson-S4 SoCs additionally uses secure monitor firmware to get binning
 * information (which means that better quality silicon can run a given
 * frequency at a lower voltage).
 *
 * Copyright (C) 2026 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 */

#include <linux/clk.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/firmware/meson/meson_sm.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/slab.h>

#include "cpufreq-dt.h"

static struct platform_device *meson_s4_cpufreq_pdev, *cpufreq_dt_pdev;

enum meson_s4_cpufreq_meson_clocks {
	MESON_S4_CPUFREQ_CPU_CLOCK = 0,
	MESON_S4_CPUFREQ_XTAL_CLOCK,
	MESON_S4_CPUFREQ_CPU_DYN_CLOCK,
	MESON_S4_CPUFREQ_SYS_PLL_CLOCK,
	MESON_S4_CPUFREQ_NUM_CLOCKS,
};

struct meson_s4_cpufreq_priv {
	struct clk_bulk_data clks[MESON_S4_CPUFREQ_NUM_CLOCKS];
	int cpu_opp_tokens[];
};

static unsigned int meson_s4_cpufreq_get_intermediate(struct cpufreq_policy *policy,
						      unsigned int index)
{
	unsigned int new_rate = policy->freq_table[index].frequency;
	struct meson_s4_cpufreq_priv *priv;
	struct clk *cpu_dyn_clk;

	priv = platform_get_drvdata(meson_s4_cpufreq_pdev);
	cpu_dyn_clk = priv->clks[MESON_S4_CPUFREQ_CPU_DYN_CLOCK].clk;

	return min_t(unsigned int, new_rate, clk_round_rate(cpu_dyn_clk, ~0));
}

static int meson_s4_cpufreq_target_intermediate(struct cpufreq_policy *policy,
						unsigned int index)
{
	unsigned int new_rate = policy->freq_table[index].frequency;
	struct meson_s4_cpufreq_priv *priv;
	struct clk *cpu_dyn_clk;
	int ret;

	priv = platform_get_drvdata(meson_s4_cpufreq_pdev);

	cpu_dyn_clk = priv->clks[MESON_S4_CPUFREQ_CPU_DYN_CLOCK].clk;

	ret = clk_set_rate(cpu_dyn_clk, new_rate);
	if (ret)
		return ret;

	return clk_set_parent(priv->clks[MESON_S4_CPUFREQ_CPU_CLOCK].clk,
			      cpu_dyn_clk);
}

static struct cpufreq_dt_platform_data meson_s4_cpufreq_dt_pdata = {
	.get_intermediate = meson_s4_cpufreq_get_intermediate,
	.target_intermediate = meson_s4_cpufreq_target_intermediate,
};

static void meson_s4_cpufreq_clear_opp_tokens(void *data)
{
	struct meson_s4_cpufreq_priv *priv = data;
	unsigned int cpu;

	for_each_present_cpu(cpu)
		dev_pm_opp_clear_config(priv->cpu_opp_tokens[cpu]);
}

static int meson_s4_cpufreq_init_opp_tokens(struct meson_s4_cpufreq_priv *priv,
					    s32 dvfs_table_index)
{
	struct dev_pm_opp_config config = {};
	char name[] = "binXXXXXXXXXXX"; /* Integers can take 11 chars max */
	struct device *cpu_dev;
	unsigned int cpu;
	int ret;

	config.supported_hw = &dvfs_table_index;
	config.supported_hw_count = 1;

	snprintf(name, sizeof(name), "bin%d", dvfs_table_index);
	config.prop_name = name;

	for_each_present_cpu(cpu) {
		cpu_dev = get_cpu_device(cpu);
		if (!cpu_dev)
			return -ENODEV;

		ret = dev_pm_opp_set_config(cpu_dev, &config);
		if (ret < 0)
			return ret;

		priv->cpu_opp_tokens[cpu] = ret;
	}

	return 0;
}

static void meson_s4_cpufreq_put_clks(void *data)
{
	struct meson_s4_cpufreq_priv *priv = data;

	clk_bulk_put(MESON_S4_CPUFREQ_NUM_CLOCKS, priv->clks);
}

static void meson_s4_cpufreq_dt_pdev_unregister(void *data)
{
	platform_device_unregister(cpufreq_dt_pdev);
}

static int meson_s4_cpufreq_probe(struct platform_device *pdev)
{
	struct meson_s4_cpufreq_priv *priv;
	struct meson_sm_firmware *fw;
	struct device *cpu_dev;
	s32 dvfs_table_index;
	int ret;

	cpu_dev = get_cpu_device(0);
	if (!cpu_dev)
		return -ENODEV;

	struct device_node *opp_np __free(device_node) =
		dev_pm_opp_of_get_opp_desc_node(cpu_dev);
	if (!opp_np)
		return -ENOENT;

	if (!of_device_is_compatible(opp_np,
				     "amlogic,meson-s4-cpu-operating-points"))
		return 0;

	struct device_node *sm_np __free(device_node) =
			of_parse_phandle(opp_np, "secure-monitor", 0);
	if (!sm_np)
		return dev_err_probe(&pdev->dev, -ENODEV,
				     "Could not get secure-monitor phandle\n");

	fw = meson_sm_get(sm_np);
	if (!fw)
		return -EPROBE_DEFER;

	ret = meson_sm_call(fw, SM_S4_GET_DVFS_TABLE_INDEX, &dvfs_table_index,
			    0, 0, 0, 0, 0);
	if (ret)
		return ret;

	priv = devm_kzalloc(&pdev->dev,
			    struct_size(priv, cpu_opp_tokens, num_possible_cpus()),
			    GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	platform_set_drvdata(pdev, priv);

	priv->clks[MESON_S4_CPUFREQ_CPU_CLOCK].id = "cpu";
	priv->clks[MESON_S4_CPUFREQ_XTAL_CLOCK].id = "xtal";
	priv->clks[MESON_S4_CPUFREQ_CPU_DYN_CLOCK].id = "cpu_dyn";
	priv->clks[MESON_S4_CPUFREQ_SYS_PLL_CLOCK].id = "sys_pll";
	ret = clk_bulk_get(cpu_dev, MESON_S4_CPUFREQ_NUM_CLOCKS, priv->clks);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "Failed to get CPU clocks\n");

	ret = devm_add_action_or_reset(&pdev->dev, meson_s4_cpufreq_put_clks,
				       priv);
	if (ret)
		return ret;

	ret = devm_add_action(&pdev->dev, meson_s4_cpufreq_clear_opp_tokens,
			      priv);
	if (ret)
		return ret;

	ret = meson_s4_cpufreq_init_opp_tokens(priv, dvfs_table_index);
	if (ret)
		return ret;

	cpufreq_dt_pdev = platform_device_register_data(NULL, "cpufreq-dt", -1,
							&meson_s4_cpufreq_dt_pdata,
							sizeof(meson_s4_cpufreq_dt_pdata));
	if (IS_ERR(cpufreq_dt_pdev))
		return dev_err_probe(&pdev->dev, PTR_ERR(cpufreq_dt_pdev),
				     "Failed to register cpufreq-dt platform device\n");

	return devm_add_action_or_reset(&pdev->dev,
					meson_s4_cpufreq_dt_pdev_unregister,
					NULL);
}

static const struct of_device_id meson_s4_cpufreq_of_match_list[] = {
	{ .compatible = "amlogic,meson-s4", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, meson_s4_cpufreq_of_match_list);

static struct platform_driver meson_s4_cpufreq_driver = {
	.probe = meson_s4_cpufreq_probe,
	.driver = {
		.name = "meson-s4-cpufreq",
	},
};

static int __init meson_s4_cpufreq_init(void)
{
	const char *platform_id;
	int ret;

	platform_id = of_machine_get_match_data(meson_s4_cpufreq_of_match_list);
	if (!platform_id)
		return -ENODEV;

	ret = platform_driver_register(&meson_s4_cpufreq_driver);
	if (ret)
		return ret;

	meson_s4_cpufreq_pdev = platform_device_register_simple("meson-s4-cpufreq",
								-1, NULL, 0);
	ret = PTR_ERR_OR_ZERO(meson_s4_cpufreq_pdev);
	if (ret)
		platform_driver_unregister(&meson_s4_cpufreq_driver);

	return ret;
}
module_init(meson_s4_cpufreq_init);

static void __exit meson_s4_cpufreq_exit(void)
{
	platform_device_unregister(meson_s4_cpufreq_pdev);
	platform_driver_unregister(&meson_s4_cpufreq_driver);
}
module_exit(meson_s4_cpufreq_exit);

MODULE_AUTHOR("Martin Blumenstingl <martin.blumenstingl@googlemail.com>");
MODULE_DESCRIPTION("Amlogic Meson-S4 cpufreq driver");
MODULE_LICENSE("GPL v2");
