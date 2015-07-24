/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 * Copyright (C) 2015 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 */

#include <dt-bindings/clock/lantiq-xway-pmu.h>
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

#define NUM_GATES_PER_MODULE	32

#define PMU_PWDCR0		0x1C
#define PMU_PWDSR0		0x20

#define PMU_PWDCR1		0x24
#define PMU_PWDSR1		0x28

static struct clk_onecell_data pmu_clk_data;

static inline u32 __to_function_bitmask(struct ltq_pmu_clk_gate *gate)
{
	u8 bit_idx = gate->function % NUM_GATES_PER_MODULE;

	return (1 << bit_idx);
}

static inline void ltq_xway_pmu_set_function(struct ltq_pmu_clk_gate *gate,
					     bool enable)
{
	u32 reg, val, function_mask;

	if (gate->function < NUM_GATES_PER_MODULE)
		reg = PMU_PWDCR0;
	else
		reg = PMU_PWDCR1;

	val = __raw_readl(gate->reg_base + reg);
	function_mask = __to_function_bitmask(gate);

	/* In hardware: 0 = enabled, 1 = disabled. */
	if (enable)
		val &= ~function_mask;
	else
		val |= function_mask;

	__raw_writel(val, gate->reg_base + reg);
}

static int ltq_xway_pmu_clk_gate_is_enabled(struct clk_hw *hw)
{
	u32 val;
	struct ltq_pmu_clk_gate *gate = to_ltq_pmu_clk_gate(hw);

	if (gate->function < NUM_GATES_PER_MODULE)
		val = __raw_readl(gate->reg_base + PMU_PWDSR0);
	else
		val = __raw_readl(gate->reg_base + PMU_PWDSR1);

	/* In hardware: 0 = enabled, 1 = disabled. */
	return (val & __to_function_bitmask(gate)) == 0;
}

static int ltq_xway_pmu_clk_gate_enable(struct clk_hw *hw)
{
	struct ltq_pmu_clk_gate *gate = to_ltq_pmu_clk_gate(hw);
	int err = 1000000;

	pr_debug("%s: Enabling PMU function %d\n", __func__, gate->function);

	ltq_xway_pmu_set_function(gate, 1);

	do {} while (--err && !ltq_xway_pmu_clk_gate_is_enabled(hw));
	if (!err) {
		pr_debug("%s: Failed to enable PMU function %d\n",
			 __func__, gate->function);
		return -EIO;
	}

	return 0;
}

static void ltq_xway_pmu_clk_gate_disable(struct clk_hw *hw)
{
	struct ltq_pmu_clk_gate *gate = to_ltq_pmu_clk_gate(hw);

	pr_debug("%s: Disabling PMU function %d\n", __func__, gate->function);

	ltq_xway_pmu_set_function(gate, 0);
}

const struct clk_ops xway_pmu_clk_gate_ops = {
	.enable = ltq_xway_pmu_clk_gate_enable,
	.disable = ltq_xway_pmu_clk_gate_disable,
	.is_enabled = ltq_xway_pmu_clk_gate_is_enabled,
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

	clk_table = kcalloc(NUM_PMU_GATES, sizeof(struct clk *), GFP_KERNEL);
	if (!clk_table) {
		panic("%s: Failed to allocate PMU gate memory\n", __func__);
		return;
	}

	for (i = 0; i < num_gates; i++) {
		gate = &gates[i];
		clk = __ltq_xway_pmu_register_gate(gate, gate_ops,
						   ltq_pmu_reg_base);
		clk_table[gate->id] = clk;
	}

	pmu_clk_data.clk_num = NUM_PMU_GATES;
	pmu_clk_data.clks = clk_table;
	of_clk_add_provider(np, of_clk_src_onecell_get, &pmu_clk_data);

	pr_debug("%s: Registered %d XWay PMU gates\n", __func__, num_gates);
}
