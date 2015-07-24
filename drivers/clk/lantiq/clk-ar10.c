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
#include <linux/of.h>
#include <linux/types.h>

#include "clk-xway.h"

#define CGU_SYS_VR9		0x0c
#define CGU_IFCCR_VR9		0x24

#define PMU_CLKGSR1		0x20
#define PMU_CLKGCR1_A		0x24
#define PMU_CLKGCR1_B		0x28
#define PMU_CLKGSR2		0x30
#define PMU_CLKGCR2_A		0x34
#define PMU_CLKGCR2_B		0x38
#define PMU_ANALOG_SR		0x40
#define PMU_ANALOGCR_A		0x44
#define PMU_ANALOGCR_B		0x48

#define NUM_GATES_PER_MODULE	32
#define IS_CLKGSR1_FUNCTION(_func)	(_func < NUM_GATES_PER_MODULE)
#define IS_CLKGSR2_FUNCTION(_func)	(_func < (2 * NUM_GATES_PER_MODULE))

static inline u32 __to_function_bitmask(struct ltq_pmu_clk_gate *gate)
{
	return (1 << (gate->function % NUM_GATES_PER_MODULE));
}

static unsigned long ltq_ar10_sys_recalc_rate(struct clk_hw *hw,
					      unsigned long parent_rate)
{
	struct ltq_cgu_clk *cgu_clk = to_ltq_cgu_clk(hw);
	unsigned long rate;

	if ((ltq_cgu_clk_read(cgu_clk, CGU_SYS_VR9) >> 8) & 0x1)
		rate = CLOCK_600M;
	else
		rate = CLOCK_500M;

	return rate;
}

static unsigned long ltq_ar10_cpu_recalc_rate(struct clk_hw *hw,
					      unsigned long parent_rate)
{
	struct ltq_cgu_clk *cgu_clk = to_ltq_cgu_clk(hw);
	unsigned int cpu_div;
	unsigned long rate;

	cpu_div = ltq_cgu_clk_read(cgu_clk, CGU_SYS_VR9) >> 4 & 0x7;

	switch (cpu_div) {
	case 0:
		rate = parent_rate;
		break;
	case 1:
		rate = parent_rate / 2;
		break;
	case 2:
		rate = parent_rate / 4;
		break;
	default:
		BUG();
	}

	return rate;
}

unsigned long ltq_ar10_fpi_recalc_rate(struct clk_hw *hw,
				       unsigned long parent_rate)
{
	struct ltq_cgu_clk *cgu_clk = to_ltq_cgu_clk(hw);
	unsigned int io_sel;
	unsigned long rate;

	io_sel = (ltq_cgu_clk_read(cgu_clk, 0x24) >> 25) & 0xf;

	switch (io_sel) {
	case 1:
		rate = CLOCK_300M;
		break;
	case 2:
		/* TODO: check BSP_MPS_ID_CFG, if bit 17 is set,
			* treat FPI clock as 125 instead of 150 */
		rate = CLOCK_150M;
		break;
	case 5:
		rate = CLOCK_250M;
		break;
	case 6:
		rate = CLOCK_125M;
		break;
	default:
		BUG();
	}

	return rate;
}

unsigned long ltq_ar10_pp32_recalc_rate(struct clk_hw *hw,
					unsigned long parent_rate)
{
	struct ltq_cgu_clk *cgu_clk = to_ltq_cgu_clk(hw);
	unsigned int ppe_sel;
	unsigned long rate;

	ppe_sel = (ltq_cgu_clk_read(cgu_clk, CGU_SYS_VR9) >> 16) & 0x7;

	switch (ppe_sel) {
	case 4:
		rate = CLOCK_400M;
		break;
	default:
		rate = CLOCK_250M;
		break;
	}

	return rate;
}

const struct clk_ops ar10_sys_clk_ops = {
	.recalc_rate = ltq_ar10_sys_recalc_rate,
};

const struct clk_ops ar10_cpu_clk_ops = {
	.recalc_rate = ltq_ar10_cpu_recalc_rate,
};

const struct clk_ops ar10_fpi_clk_ops = {
	.recalc_rate = ltq_ar10_fpi_recalc_rate,
};

const struct clk_ops ar10_pp32_clk_ops = {
	.recalc_rate = ltq_ar10_pp32_recalc_rate,
};

static void __init ar10_cgu_clocks_init_dt(struct device_node *np)
{
	struct ltq_cgu_clocks cgu_clocks = {};
	struct clk *sys_clk;

	ltq_xway_cgu_init_dt(np);

	ltq_cgu_select_gphy_clk_input(CGU_IFCCR_VR9);

	sys_clk = ltq_cgu_register_clk("sys", NULL, &ar10_sys_clk_ops);
	if (clk_register_clkdev(sys_clk, "sys", NULL))
		pr_err("%s: Failed to register the sys clkdev\n", __func__);

	cgu_clocks.cpu_clk = ltq_cgu_register_clk("cpu", "sys",
						  &ar10_cpu_clk_ops);
	cgu_clocks.fpi_clk = ltq_cgu_register_clk("fpi", NULL,
						  &ar10_fpi_clk_ops);
	cgu_clocks.io_clk = cgu_clocks.fpi_clk; /* FPI clock is used for IO */
	cgu_clocks.pp32_clk = ltq_cgu_register_clk("pp32", NULL,
						   &ar10_pp32_clk_ops);

	ltq_cgu_clk_of_add_provider(np, &cgu_clocks);
}

CLK_OF_DECLARE(cgu_ar10, "lantiq,cgu-ar10", ar10_cgu_clocks_init_dt);

static int ltq_ar10_pmu_clk_gate_is_enabled(struct clk_hw *hw)
{
	u32 val;
	struct ltq_pmu_clk_gate *gate = to_ltq_pmu_clk_gate(hw);

	if (IS_CLKGSR1_FUNCTION(gate->function))
		val = __raw_readl(gate->reg_base + PMU_CLKGSR1);
	else if (IS_CLKGSR2_FUNCTION(gate->function))
		val = __raw_readl(gate->reg_base + PMU_CLKGSR2);
	else
		val = __raw_readl(gate->reg_base + PMU_ANALOG_SR);

	return val & __to_function_bitmask(gate);
}

static int ltq_ar10_pmu_clk_gate_enable(struct clk_hw *hw)
{
	struct ltq_pmu_clk_gate *gate = to_ltq_pmu_clk_gate(hw);
	int err = 1000000;
	u32 reg;

	pr_debug("%s: Enabling PMU function %d\n", __func__, gate->function);

	/* AR10 has a dedicated registers to enable a gate. */
	if (IS_CLKGSR1_FUNCTION(gate->function))
		reg = PMU_CLKGCR1_A;
	else if (IS_CLKGSR2_FUNCTION(gate->function))
		reg = PMU_CLKGCR2_A;
	else
		reg = PMU_ANALOGCR_A;

	__raw_writel(__to_function_bitmask(gate), gate->reg_base + reg);

	do {} while (--err && !ltq_ar10_pmu_clk_gate_is_enabled(hw));

	if (!err) {
		pr_debug("%s: Failed to enable PMU function %d\n",
			 __func__, gate->function);
		return -EIO;
	}

	return 0;
}

static void ltq_ar10_pmu_clk_gate_disable(struct clk_hw *hw)
{
	struct ltq_pmu_clk_gate *gate = to_ltq_pmu_clk_gate(hw);
	u32 reg;

	pr_debug("%s: Disabling PMU function %d\n", __func__, gate->function);

	/* AR10 has a dedicated registers to disable a gate. */
	if (IS_CLKGSR1_FUNCTION(gate->function))
		reg = PMU_CLKGCR1_B;
	else if (IS_CLKGSR2_FUNCTION(gate->function))
		reg = PMU_CLKGCR2_B;
	else
		reg = PMU_ANALOGCR_B;

	__raw_writel(__to_function_bitmask(gate), gate->reg_base + reg);
}

/* AR10's PMU module is still controlling gates, but it's registers are
 * almost completely different compared to the other xway PMUs. */
const struct clk_ops ar10_pmu_clk_gate_ops = {
	.enable = ltq_ar10_pmu_clk_gate_enable,
	.disable = ltq_ar10_pmu_clk_gate_disable,
	.is_enabled = ltq_ar10_pmu_clk_gate_is_enabled,
};

static struct ltq_xway_pmu_gate ar10_pmu_gates[] __initdata = {
	PMU_GATE(PMU_GATE_DFEV0, "pmu_dfev0", 2, NULL, 0, NULL, NULL),
	PMU_GATE(PMU_GATE_DFEV1, "pmu_dfev1", 3, NULL, 0, NULL, NULL),
	PMU_GATE(PMU_GATE_DMA, "pmu_dma", 5, NULL, 0, "1e104100.dma", NULL),
	PMU_GATE(PMU_GATE_USB0_CTRL, "pmu_usb0_ctrl", 6, NULL, 0, NULL, NULL),
	PMU_GATE(PMU_GATE_USIF, "pmu_usif", 7, NULL, 0, NULL, NULL),
	PMU_GATE(PMU_GATE_SPI, "pmu_spi", 8, NULL, 0, "1e100800.spi", NULL),
	PMU_GATE(PMU_GATE_DSL_DFE, "pmu_dsl_dfe", 9, NULL, 0, NULL, NULL),
	PMU_GATE(PMU_GATE_EBU, "pmu_ebu", 10, NULL, CLK_IGNORE_UNUSED,
		 "1e105300.ebu", NULL),
	PMU_GATE(PMU_GATE_STP, "pmu_stp", 11, NULL, 0, "1e100bb0.stp", NULL),
	PMU_GATE(PMU_GATE_GPTC, "pmu_gptu", 12, NULL, 0, "1e100a00.gptu",
		 NULL),
	PMU_GATE(PMU_GATE_UART1, "pmu_serial1", 17, NULL, 0, "1e100c00.serial",
		 NULL),
	PMU_GATE(PMU_GATE_DEU, "pmu_deu", 20, NULL, 0, NULL, NULL),
	PMU_GATE(PMU_GATE_PPE_TC, "pmu_ppe_tc", 21, NULL, 0, NULL, NULL),
	PMU_GATE(PMU_GATE_PPE_EMA, "pmu_ppe_ema", 22, NULL, 0, NULL, NULL),
	PMU_GATE(PMU_GATE_PPE_DPLUS, "pmu_ppe_dplus", 23, NULL, 0, NULL, NULL),
	PMU_GATE(PMU_GATE_TDM, "pmu_tdm", 25, NULL, 0, NULL, NULL),
	PMU_GATE(PMU_GATE_USB1_CTRL, "pmu_usb1_ctrl", 27, NULL, 0, NULL, NULL),
	PMU_GATE(PMU_GATE_SWITCH, "pmu_switch", 28, NULL, 0, NULL, NULL),
	PMU_GATE(PMU_GATE_PCIE0_CTRL, "pmu_pcie0_ctrl", 33, NULL, 0, NULL,
		 NULL),
	PMU_GATE(PMU_GATE_PCIE1_CTRL, "pmu_pcie1_ctrl", 34, NULL, 0, NULL,
		 NULL),
	PMU_GATE(PMU_GATE_PCIE0_PDI, "pmu_pcie0_pdi", 36, NULL, 0, NULL, NULL),
	PMU_GATE(PMU_GATE_PCIE0_MSI, "pmu_pcie0_msi", 37, NULL, 0, NULL, NULL),
	PMU_GATE(PMU_GATE_PCIE1_PDI, "pmu_pcie1_pdi", 52, NULL, 0, NULL, NULL),
	PMU_GATE(PMU_GATE_PCIE1_MSI, "pmu_pcie1_phy", 53, NULL, 0, NULL, NULL),
	PMU_GATE(PMU_GATE_PCIE2_CTRL, "pmu_pcie2_ctrl", 57, NULL, 0, NULL,
		 NULL),
	PMU_GATE(PMU_GATE_PCIE2_PDI, "pmu_pcie2_pdi", 58, NULL, 0, NULL, NULL),
	PMU_GATE(PMU_GATE_PCIE2_MSI, "pmu_pcie2_msi", 59, NULL, 0, NULL, NULL),
	PMU_GATE(PMU_GATE_USB0_PHY, "pmu_usb0_phy", 64, NULL, 0, NULL, NULL),
	PMU_GATE(PMU_GATE_USB1_PHY, "pmu_usb1_phy", 65, NULL, 0, NULL, NULL),
	PMU_GATE(PMU_GATE_PCIE0_PHY, "pmu_pcie0_phy", 72, NULL, 0, NULL, NULL),
	PMU_GATE(PMU_GATE_PCIE1_PHY, "pmu_pcie1_phy", 73, NULL, 0, NULL, NULL),
	PMU_GATE(PMU_GATE_PCIE2_PHY, "pmu_pcie2_phy", 74, NULL, 0, NULL, NULL),
	PMU_GATE(PMU_GATE_ADSL_AFE, "pmu_adsl_afe", 80, NULL, 0, NULL, NULL),
	PMU_GATE(PMU_GATE_DCDC_2V5, "pmu_dcdc_2v5", 81, NULL,
		 CLK_IGNORE_UNUSED, NULL, NULL),
	PMU_GATE(PMU_GATE_DCDC_1VX, "pmu_dcdc_1vx", 82, NULL,
		 CLK_IGNORE_UNUSED, NULL, NULL),
	PMU_GATE(PMU_GATE_DCDC_1V0, "pmu_dcdc_1v0", 83, NULL,
		 CLK_IGNORE_UNUSED, NULL, NULL),
};

static void __init ar10_pmu_clk_gates_init_dt(struct device_node *np)
{
	ltq_xway_pmu_register_gates(np,
				    ar10_pmu_gates,
				    ARRAY_SIZE(ar10_pmu_gates),
				    &ar10_pmu_clk_gate_ops);
}

CLK_OF_DECLARE(pmu_ar10, "lantiq,pmu-ar10", ar10_pmu_clk_gates_init_dt);
