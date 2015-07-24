/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 * Copyright (C) 2015 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 */

#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/types.h>

#include "clk-xway.h"

/* pre-xRX300 PMU registers */
#define XWAY_PMU_PWDCR0		0x1C
#define XWAY_PMU_PWDSR0		0x20
#define XWAY_PMU_PWDCR1		0x24
#define XWAY_PMU_PWDSR1		0x28

/* xRX300 PMU registers */
#define XRX300_PMU_CLKGSR1		0x20
#define XRX300_PMU_CLKGCR1_A		0x24
#define XRX300_PMU_CLKGCR1_B		0x28
#define XRX300_PMU_CLKGSR2		0x30
#define XRX300_PMU_CLKGCR2_A		0x34
#define XRX300_PMU_CLKGCR2_B		0x38
#define XRX300_PMU_ANALOG_SR		0x40
#define XRX300_PMU_ANALOGCR_A		0x44
#define XRX300_PMU_ANALOGCR_B		0x48

#define NUM_GATES_PER_MODULE	32

#define IS_XRX300_CLKGSR1_FUNCTION(_func)	\
					(_func < NUM_GATES_PER_MODULE)
#define IS_XRX300_CLKGSR2_FUNCTION(_func)	\
					(_func < (2 * NUM_GATES_PER_MODULE))

static struct clk_onecell_data pmu_clk_data;

static inline u32 __function_to_register_bit(struct ltq_pmu_clk_gate *gate)
{
	u8 bit_idx = gate->function % NUM_GATES_PER_MODULE;

	return (1 << bit_idx);
}

static void xway_pmu_clk_endisable(struct ltq_pmu_clk_gate *gate,
					  bool enable)
{
	u32 reg, val, function_mask;

	if (gate->function < NUM_GATES_PER_MODULE)
		reg = XWAY_PMU_PWDCR0;
	else
		reg = XWAY_PMU_PWDCR1;

	val = __raw_readl(gate->reg_base + reg);
	function_mask = __function_to_register_bit(gate);

	/* In hardware: 0 = enabled, 1 = disabled. */
	if (enable)
		val &= ~function_mask;
	else
		val |= function_mask;

	__raw_writel(val, gate->reg_base + reg);
}

static int xway_pmu_clk_gate_is_enabled(struct clk_hw *hw)
{
	u32 val;
	struct ltq_pmu_clk_gate *gate = to_ltq_pmu_clk_gate(hw);

	if (gate->function < NUM_GATES_PER_MODULE)
		val = __raw_readl(gate->reg_base + XWAY_PMU_PWDSR0);
	else
		val = __raw_readl(gate->reg_base + XWAY_PMU_PWDSR1);

	/* In hardware: 0 = enabled, 1 = disabled. */
	return (val & __function_to_register_bit(gate)) == 0;
}

static int ltq_xway_pmu_clk_gate_enable(struct clk_hw *hw)
{
	struct ltq_pmu_clk_gate *gate = to_ltq_pmu_clk_gate(hw);
	int retry = 1000000;

	pr_debug("%s: Enabling PMU function %d\n", __func__, gate->function);

	xway_pmu_clk_endisable(gate, true);

	do {} while (--retry && !xway_pmu_clk_gate_is_enabled(hw));
	if (!retry) {
		pr_err("%s: Failed to enable PMU function %d\n",
			 __func__, gate->function);
		return -EIO;
	}

	return 0;
}

static void xway_pmu_clk_gate_disable(struct clk_hw *hw)
{
	struct ltq_pmu_clk_gate *gate = to_ltq_pmu_clk_gate(hw);
	int retry = 1000000;

	pr_debug("%s: Disabling PMU function %d\n", __func__, gate->function);

	xway_pmu_clk_endisable(gate, false);

	do {} while (--retry && xway_pmu_clk_gate_is_enabled(hw));
	if (!retry)
		pr_warn("%s: Failed to disable PMU function %d\n",
			 __func__, gate->function);
}

const struct clk_ops xway_pmu_clk_gate_ops = {
	.enable = ltq_xway_pmu_clk_gate_enable,
	.disable = xway_pmu_clk_gate_disable,
	.is_enabled = xway_pmu_clk_gate_is_enabled,
};

static int xrx300_pmu_clk_gate_is_enabled(struct clk_hw *hw)
{
	u32 val;
	struct ltq_pmu_clk_gate *gate = to_ltq_pmu_clk_gate(hw);

	if (IS_XRX300_CLKGSR1_FUNCTION(gate->function))
		val = __raw_readl(gate->reg_base + XRX300_PMU_CLKGSR1);
	else if (IS_XRX300_CLKGSR2_FUNCTION(gate->function))
		val = __raw_readl(gate->reg_base + XRX300_PMU_CLKGSR2);
	else
		val = __raw_readl(gate->reg_base + XRX300_PMU_ANALOG_SR);

	return val & __function_to_register_bit(gate);
}

static int xrx300_pmu_clk_gate_enable(struct clk_hw *hw)
{
	struct ltq_pmu_clk_gate *gate = to_ltq_pmu_clk_gate(hw);
	int retry = 1000000;
	u32 reg;

	pr_debug("%s: Enabling PMU function %d\n", __func__, gate->function);

	/* AR10 has a dedicated registers to enable a gate. */
	if (IS_XRX300_CLKGSR1_FUNCTION(gate->function))
		reg = XRX300_PMU_CLKGCR1_A;
	else if (IS_XRX300_CLKGSR2_FUNCTION(gate->function))
		reg = XRX300_PMU_CLKGCR2_A;
	else
		reg = XRX300_PMU_ANALOGCR_A;

	__raw_writel(__function_to_register_bit(gate), gate->reg_base + reg);

	do {} while (--retry && !xrx300_pmu_clk_gate_is_enabled(hw));
	if (!retry) {
		pr_err("%s: Failed to enable PMU function %d\n",
			 __func__, gate->function);
		return -EIO;
	}

	return 0;
}

static void xrx300_pmu_clk_gate_disable(struct clk_hw *hw)
{
	struct ltq_pmu_clk_gate *gate = to_ltq_pmu_clk_gate(hw);
	int retry = 1000000;
	u32 reg;

	pr_debug("%s: Disabling PMU function %d\n", __func__, gate->function);

	/* AR10 has a dedicated registers to disable a gate. */
	if (IS_XRX300_CLKGSR1_FUNCTION(gate->function))
		reg = XRX300_PMU_CLKGCR1_B;
	else if (IS_XRX300_CLKGSR2_FUNCTION(gate->function))
		reg = XRX300_PMU_CLKGCR2_B;
	else
		reg = XRX300_PMU_ANALOGCR_B;

	__raw_writel(__function_to_register_bit(gate), gate->reg_base + reg);

	do {} while (--retry && xrx300_pmu_clk_gate_is_enabled(hw));
	if (!retry)
		pr_warn("%s: Failed to disable PMU function %d\n",
			 __func__, gate->function);
}

const struct clk_ops xrx300_pmu_clk_gate_ops = {
	.enable = xrx300_pmu_clk_gate_enable,
	.disable = xrx300_pmu_clk_gate_disable,
	.is_enabled = xrx300_pmu_clk_gate_is_enabled,
};

struct clk *__ltq_xway_pmu_register_gate(struct ltq_xway_pmu_gate *cfg,
					 const struct clk_ops *gate_ops,
					 void __iomem *ltq_pmu_reg_base)
{
	struct ltq_pmu_clk_gate *gate;
	struct clk *clk;
	struct clk_init_data init;

	gate = kzalloc(sizeof(*gate), GFP_KERNEL);
	if (!gate)
		return ERR_PTR(-ENOMEM);

	init.name = cfg->name;
	init.ops = gate_ops;
	init.flags = cfg->flags;

	if (cfg->parent_name) {
		init.parent_names = &cfg->parent_name;
		init.num_parents = 1;
	} else {
		init.flags |= CLK_IS_ROOT;
		init.parent_names = NULL;
		init.num_parents = 0;
	}

	gate->function = cfg->function;
	gate->reg_base = ltq_pmu_reg_base;
	gate->hw.init = &init;

	clk = clk_register(NULL, &gate->hw);

	if (IS_ERR(clk)) {
		kfree(gate);
		return clk;
	}

	if (clk_register_clkdev(clk, cfg->name, NULL))
		pr_err("%s: Failed to register lookup for %s\n",
			__func__, cfg->name);

	if (cfg->dev_id || cfg->con_id)
		if (clk_register_clkdev(clk, cfg->con_id, cfg->dev_id))
			pr_err("%s: Failed to register alias lookup for %s\n",
				__func__, cfg->name);

	return clk;
}

void ltq_xway_pmu_register_gates(struct device_node *np,
				 struct ltq_xway_pmu_gate *gates,
				 u32 num_gates,
				 const struct clk_ops *gate_ops)
{
	struct resource res_pmu;
	struct clk *clk;
	struct clk **clk_table;
	struct ltq_xway_pmu_gate *gate;
	void __iomem *ltq_pmu_reg_base;
	int i;

	pr_debug("%s: Initializing XWay PMU gates\n", __func__);

	if (of_address_to_resource(np, 0, &res_pmu))
		panic("%s: could not determine PMU base address\n", __func__);

	ltq_pmu_reg_base = ioremap_nocache(res_pmu.start,
					  resource_size(&res_pmu));
	if (!ltq_pmu_reg_base)
		panic("%s: Failed to remap PMU resource\n", __func__);

	clk_table = kcalloc(num_gates, sizeof(struct clk *), GFP_KERNEL);
	if (!clk_table) {
		panic("%s: Failed to allocate PMU gate memory\n", __func__);
		return;
	}

	for (i = 0; i < num_gates; i++) {
		gate = &gates[i];
		clk = __ltq_xway_pmu_register_gate(gate, gate_ops,
						   ltq_pmu_reg_base);

		/* entries might not be sorted, thus use their id */
		clk_table[gate->id] = clk;
	}

	pmu_clk_data.clk_num = num_gates;
	pmu_clk_data.clks = clk_table;
	of_clk_add_provider(np, of_clk_src_onecell_get, &pmu_clk_data);

	pr_debug("%s: Registered %d XWay PMU gates\n", __func__, num_gates);
}
