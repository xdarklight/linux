/*
 * Copyright (c) 2018 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 *
 * SPDX-License-Identifier: GPL-2.0+
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/platform_device.h>

struct meson_cpu_clk {
	struct clk_hw	hw;
	struct clk	*cpu_clk_sel;
	struct clk	*xtal;
};

static struct meson_cpu_clk *to_meson_cpu_clk(struct clk_hw *hw)
{
	return container_of(hw, struct meson_cpu_clk, hw);
}

static unsigned long meson_cpu_clk_recalc_rate(struct clk_hw *hw,
					       unsigned long parent_rate)
{
	struct meson_cpu_clk *clk_cpu = to_meson_cpu_clk(hw);

	return clk_get_rate(clk_cpu->cpu_clk_sel);
}

static long meson_cpu_clk_round_rate(struct clk_hw *hw, unsigned long rate,
				     unsigned long *parent_rate)
{
	struct meson_cpu_clk *clk_cpu = to_meson_cpu_clk(hw);

	return clk_round_rate(clk_cpu->cpu_clk_sel, rate);
}

static int meson_cpu_clk_set_rate(struct clk_hw *hw, unsigned long rate,
				  unsigned long parent_rate)
{
	struct meson_cpu_clk *clk_cpu = to_meson_cpu_clk(hw);
	int ret;

#if 0
	unsigned long old_rate;
	u64 start;

	old_rate = clk_get_rate(clk_cpu->cpu_clk_sel);
	start = ktime_get_ns();
#endif

	/*
	 * temporarily switch CPU_CLK_SEL to XTAL so we can safely change any
	 * of the clocks in the hierarchy of CPU_SCALE_OUT_SEL (which also
	 * includes SYS_PLL) if necessary. This is needed because otherwise the
	 * whole system can lock up when changing any clock in the hierarchy of
	 * CPU_SCALE_OUT_SEL.
	 */
	ret = clk_set_parent(clk_cpu->cpu_clk_sel, clk_cpu->xtal);
	if (ret)
		return ret;

	udelay(100);

	/*
	 * let the common clock framework calculate all clocks in the
	 * CPU_SCALE_OUT_SEL tree. this will also switch CPU_CLK_SEL back to
	 * CPU_SCALE_OUT_SEL.
	 */
	ret = clk_set_rate(clk_cpu->cpu_clk_sel, rate);

#if 0
	printk("%s(%lu) from %lu took %llu ns\n", __func__, rate, old_rate, (ktime_get_ns() - start));
#endif

	return ret;
}

const struct clk_ops meson_cpu_clk_ops = {
	.recalc_rate = meson_cpu_clk_recalc_rate,
	.round_rate = meson_cpu_clk_round_rate,
	.set_rate = meson_cpu_clk_set_rate,
};

static int meson_cpu_clk_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct meson_cpu_clk *clk_cpu;
	struct clk_init_data init_data;
	int ret;

	clk_cpu = devm_kzalloc(dev, sizeof(*clk_cpu), GFP_KERNEL);
	if (!clk_cpu)
		return -ENOMEM;

	clk_cpu->cpu_clk_sel = devm_clk_get(dev, "cpu_clk_sel");
	if (IS_ERR(clk_cpu->cpu_clk_sel))
		return PTR_ERR(clk_cpu->cpu_clk_sel);

	clk_cpu->xtal = devm_clk_get(dev, "xtal");
	if (IS_ERR(clk_cpu->xtal))
		return PTR_ERR(clk_cpu->xtal);

	init_data.name = "cpu_clk";
	init_data.num_parents = 0;
	init_data.ops = &meson_cpu_clk_ops;

	clk_cpu->hw.init = &init_data;

	ret = devm_clk_hw_register(dev, &clk_cpu->hw);
	if (ret)
		return ret;

	return of_clk_add_hw_provider(np, of_clk_hw_simple_get, &clk_cpu->hw);
}

static const struct of_device_id meson_cpu_clk_match_table[] = {
	{ .compatible = "amlogic,meson8-cpu-clk" },
	{ .compatible = "amlogic,meson8b-cpu-clk" },
	{ }
};

static struct platform_driver meson_cpu_clk_driver = {
	.probe		= meson_cpu_clk_probe,
	.driver		= {
		.name	= "meson-cpu-clk",
		.of_match_table = meson_cpu_clk_match_table,
	},
};
builtin_platform_driver(meson_cpu_clk_driver);
