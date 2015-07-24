/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 * Copyright (C) 2015 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 */

#include <dt-bindings/clock/lantiq-pmu-arx300.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/types.h>

#include "clk-xway.h"

#define ARX300_PMU(_id, _name, _func, _parent, _flags, _dev, _con) \
	PMU_GATE(ARX300_PMU_GATE_##_id, _name, _func, _parent, _flags, _dev, _con)

static unsigned long ltq_arx300_sys_recalc_rate(struct clk_hw *hw,
						unsigned long parent_rate)
{
	struct ltq_cgu_clk *cgu_clk = to_ltq_cgu_clk(hw);
	unsigned long rate;

	if ((ltq_cgu_clk_read(cgu_clk, CGU_SYS_XRX) >> 8) & 0x1)
		rate = CLOCK_600M;
	else
		rate = CLOCK_500M;

	return rate;
}

static unsigned long ltq_arx300_cpu_recalc_rate(struct clk_hw *hw,
						unsigned long parent_rate)
{
	struct ltq_cgu_clk *cgu_clk = to_ltq_cgu_clk(hw);
	unsigned int cpu_div;
	unsigned long rate;

	cpu_div = ltq_cgu_clk_read(cgu_clk, CGU_SYS_XRX) >> 4 & 0x7;

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
		pr_warn("%s: Unknown cpu div: %u\n", __func__, cpu_div);
		rate = parent_rate;
		break;
	}

	return rate;
}

unsigned long ltq_arx300_fpi_recalc_rate(struct clk_hw *hw,
					 unsigned long parent_rate)
{
	struct ltq_cgu_clk *cgu_clk = to_ltq_cgu_clk(hw);
	unsigned int io_sel;
	unsigned long rate;

	io_sel = (ltq_cgu_clk_read(cgu_clk, CGU_IFCCR_XRX) >> 25) & 0xf;

	switch (io_sel) {
	case 1:
		rate = CLOCK_300M;
		break;
	case 2:
		rate = CLOCK_150M;
		break;
	case 5:
		rate = CLOCK_250M;
		break;
	case 6:
		rate = CLOCK_125M;
		break;
	default:
		pr_warn("%s: Unknown io sel: %u\n", __func__, io_sel);
		rate = CLOCK_125M;
		break;
	}

	return rate;
}

unsigned long ltq_arx300_pp32_recalc_rate(struct clk_hw *hw,
					  unsigned long parent_rate)
{
	struct ltq_cgu_clk *cgu_clk = to_ltq_cgu_clk(hw);
	unsigned int ppe_sel;
	unsigned long rate;

	ppe_sel = (ltq_cgu_clk_read(cgu_clk, CGU_SYS) >> 16) & 0x7;

	switch (ppe_sel) {
	case 1:
		rate = CLOCK_250M;
		break;
	case 4:
		rate = CLOCK_400M;
		break;
	default:
		pr_warn("%s: Unknown ppe sel: %u\n", __func__, ppe_sel);
		rate = CLOCK_250M;
		break;
	}

	return rate;
}

const struct clk_ops arx300_sys_clk_ops = {
	.recalc_rate = ltq_arx300_sys_recalc_rate,
};

const struct clk_ops arx300_cpu_clk_ops = {
	.recalc_rate = ltq_arx300_cpu_recalc_rate,
};

const struct clk_ops arx300_fpi_clk_ops = {
	.recalc_rate = ltq_arx300_fpi_recalc_rate,
};

const struct clk_ops arx300_pp32_clk_ops = {
	.recalc_rate = ltq_arx300_pp32_recalc_rate,
};

static void __init arx300_cgu_clocks_init_dt(struct device_node *np)
{
	struct ltq_cgu_clocks cgu_clocks = {};
	struct clk *sys_clk;

	ltq_xway_cgu_init_dt(np);

	ltq_xrx_cgu_select_gphy_clk_input(np);

	sys_clk = ltq_cgu_register_clk("sys", NULL, &arx300_sys_clk_ops);
	if (clk_register_clkdev(sys_clk, "sys", NULL))
		pr_err("%s: Failed to register the sys clkdev\n", __func__);

	cgu_clocks.cpu_clk = ltq_cgu_register_clk("cpu", "sys",
						  &arx300_cpu_clk_ops);
	cgu_clocks.fpi_clk = ltq_cgu_register_clk("fpi", NULL,
						  &arx300_fpi_clk_ops);
	cgu_clocks.io_clk = cgu_clocks.fpi_clk; /* FPI clock is used for IO */
	cgu_clocks.pp32_clk = ltq_cgu_register_clk("pp32", NULL,
						   &arx300_pp32_clk_ops);

	ltq_cgu_clk_of_add_provider(np, &cgu_clocks);
}

CLK_OF_DECLARE(cgu_arx300, "lantiq,arx300-cgu", arx300_cgu_clocks_init_dt);

void arx300_cgu_clocks_init_compat(struct device_node *np)
{
	arx300_cgu_clocks_init_dt(np);
}

static struct ltq_xway_pmu_gate arx300_pmu_gates[] __initdata = {
	ARX300_PMU(DFEV0, "dfev0", 2, NULL, 0, NULL, NULL),
	ARX300_PMU(DFEV1, "dfev1", 3, NULL, 0, NULL, NULL),
	ARX300_PMU(DMA, "dma", 5, NULL, 0, "1e104100.dma", NULL),
	ARX300_PMU(USB0_CTRL, "usb0_ctrl", 6, NULL, 0, "1e101000.usb", "ctl"),
	ARX300_PMU(USIF, "usif", 7, NULL, 0, "1da00000.usif", NULL),
	ARX300_PMU(SPI, "spi", 8, NULL, 0, "1e100800.spi", NULL),
	ARX300_PMU(DSL_DFE, "dsl_dfe", 9, NULL, 0, "1e116000.mei", "dfe"),
	ARX300_PMU(EBU, "ebu", 10, NULL, CLK_IGNORE_UNUSED, "1e105300.ebu", NULL),
	ARX300_PMU(STP, "ledc", 11, NULL, 0, "1e100bb0.stp", NULL),
	ARX300_PMU(GPTC, "gptu", 12, NULL, 0, "1e100a00.gptu", NULL),
	ARX300_PMU(UART1, "serial1", 17, NULL, CLK_IGNORE_UNUSED, "1e100c00.serial", NULL),
	ARX300_PMU(DEU, "deu", 20, NULL, 0, "1e103100.deu", NULL),
	ARX300_PMU(PPE_TC, "ppe_tc", 21, NULL, 0, "1e108000.eth", "ppe_tc"),
	ARX300_PMU(PPE_EMA, "ppe_ema", 22, NULL, 0, NULL, NULL),
	ARX300_PMU(PPE_DPLUS, "ppe_dplus", 23, NULL, 0, "1e108000.eth", "ppe_dplus"),
	ARX300_PMU(TDM, "tdm", 25, NULL, 0, NULL, NULL),
	ARX300_PMU(USB1_CTRL, "usb1_ctrl", 27, NULL, 0, "1e106000.usb", "ctl"),
	ARX300_PMU(SWITCH, "switch", 28, NULL, 0, "1e108000.eth", "switch"),
	ARX300_PMU(PCIE0_CTRL, "pcie0_ctrl", 33, NULL, 0, NULL, NULL),
	ARX300_PMU(PCIE1_CTRL, "pcie1_ctrl", 34, NULL, 0, NULL, NULL),
	ARX300_PMU(PCIE0_PDI, "pcie0_pdi", 36, NULL, 0, NULL, NULL),
	ARX300_PMU(PCIE0_MSI, "pcie0_msi", 37, NULL, 0, NULL, NULL),
	ARX300_PMU(DDR_CKE, "ddr_cke", 38, NULL, CLK_IGNORE_UNUSED, NULL, NULL),
	ARX300_PMU(PCIE1_PDI, "pcie1_pdi", 52, NULL, 0, NULL, NULL),
	ARX300_PMU(PCIE1_MSI, "pcie1_phy", 53, NULL, 0, NULL, NULL),
	ARX300_PMU(USB0_PHY, "usb0_phy", 64, NULL, 0, "1e101000.usb", "phy"),
	ARX300_PMU(USB1_PHY, "usb1_phy", 65, NULL, 0, "1e106000.usb", "phy"),
	ARX300_PMU(PCIE0_PHY, "pcie0_phy", 72, NULL, 0, NULL, NULL),
	ARX300_PMU(PCIE1_PHY, "pcie1_phy", 73, NULL, 0, NULL, NULL),
	ARX300_PMU(ADSL_AFE, "adsl_afe", 80, NULL, 0, "1e116000.mei", "afe"),
	ARX300_PMU(DCDC_2V5, "dcdc_2v5", 81, NULL, CLK_IGNORE_UNUSED, NULL, NULL),
	ARX300_PMU(DCDC_1VX, "dcdc_1vx", 82, NULL, CLK_IGNORE_UNUSED, NULL, NULL),
	ARX300_PMU(DCDC_1V0, "dcdc_1v0", 83, NULL, CLK_IGNORE_UNUSED, NULL, NULL),
};

static void __init arx300_pmu_clk_gates_init_dt(struct device_node *np)
{
	ltq_xway_pmu_register_gates(np,
				    arx300_pmu_gates,
				    ARRAY_SIZE(arx300_pmu_gates),
				    &xrx300_pmu_clk_gate_ops);
}

CLK_OF_DECLARE(pmu_arx300, "lantiq,arx300-pmu", arx300_pmu_clk_gates_init_dt);

void arx300_pmu_clocks_init_compat(struct device_node *np)
{
	arx300_pmu_clk_gates_init_dt(np);
}
