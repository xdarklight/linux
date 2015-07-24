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

#define CGU_SYS			0x10
#define CGU_IFCCR		0x18
#define CGU_PCICR		0x34

unsigned long ltq_ar9_sys_hz(struct ltq_cgu_clk *cgu_clk)
{
	if (((ltq_cgu_clk_read(cgu_clk, CGU_SYS) >> 3) & 0x3) == 0x2)
		return CLOCK_393M;

	return CLOCK_333M;
}

unsigned long ltq_ar9_fpi_hz(struct ltq_cgu_clk *cgu_clk)
{
	unsigned long sys_rate = ltq_ar9_sys_hz(cgu_clk);
	unsigned long rate;

	if (ltq_cgu_clk_read(cgu_clk, CGU_SYS) & BIT(0))
		rate = sys_rate / 3;
	else
		rate = sys_rate / 2;

	return rate;
}

static unsigned long ltq_ar9_cpu_recalc_rate(struct clk_hw *hw,
					     unsigned long parent_rate)
{
	struct ltq_cgu_clk *cgu_clk = to_ltq_cgu_clk(hw);
	unsigned long rate;

	if (ltq_cgu_clk_read(cgu_clk, CGU_SYS) & BIT(2))
		rate = ltq_ar9_fpi_hz(cgu_clk);
	else
		rate = ltq_ar9_sys_hz(cgu_clk);

	return rate;
}

static unsigned long ltq_ar9_fpi_recalc_rate(struct clk_hw *hw,
				     unsigned long parent_rate)
{
	struct ltq_cgu_clk *cgu_clk = to_ltq_cgu_clk(hw);

	return ltq_ar9_fpi_hz(cgu_clk);
}

const struct clk_ops ar9_cpu_clk_ops = {
	.recalc_rate = ltq_ar9_cpu_recalc_rate,
};

const struct clk_ops ar9_fpi_clk_ops = {
	.recalc_rate = ltq_ar9_fpi_recalc_rate,
};

static void __init ar9_cgu_clocks_init_dt(struct device_node *np)
{
	struct ltq_cgu_clocks cgu_clocks = {};

	ltq_xway_cgu_init_dt(np);

	cgu_clocks.cpu_clk = ltq_cgu_register_clk("cpu", NULL,
						  &ar9_cpu_clk_ops);
	cgu_clocks.fpi_clk = ltq_cgu_register_clk("fpi", NULL,
						  &ar9_fpi_clk_ops);
	cgu_clocks.io_clk = cgu_clocks.fpi_clk; /* FPI clock is used for IO */
	cgu_clocks.pp32_clk = clk_register_fixed_rate(NULL, "pp32", NULL,
						      CLK_IS_ROOT, CLOCK_250M);
	cgu_clocks.pci_clk = ltq_xway_register_cgu_pci_clk(
		CGU_IFCCR, CGU_PCICR, "pmu_pci");
	cgu_clocks.pci_ext_clk = ltq_xway_register_cgu_pci_ext_clk(
		CGU_IFCCR, CGU_PCICR);

	ltq_cgu_clk_of_add_provider(np, &cgu_clocks);
}

CLK_OF_DECLARE(cgu_ar9, "lantiq,cgu-ar9", ar9_cgu_clocks_init_dt);

static struct ltq_xway_pmu_gate ar9_pmu_gates[] __initdata = {
	PMU_GATE(PMU_GATE_USB0_PHY, "pmu_usb0_phy", 0, NULL, 0, NULL, NULL),
	PMU_GATE(PMU_GATE_DFEV0, "pmu_dfev0", 2, NULL, 0, NULL, NULL),
	PMU_GATE(PMU_GATE_DFEV1, "pmu_dfev1", 3, NULL, 0, NULL, NULL),
	PMU_GATE(PMU_GATE_PCI, "pmu_pci", 4, NULL, 0, NULL, NULL),
	PMU_GATE(PMU_GATE_DMA, "pmu_dma", 5, NULL, 0, "1e104100.dma", NULL),
	PMU_GATE(PMU_GATE_USB0_CTRL, "pmu_usb0_ctrl", 6, NULL, 0, NULL, NULL),
	PMU_GATE(PMU_GATE_UART0, "pmu_serial0", 7, NULL, 0,
		 "1e100400.serial", NULL),
	PMU_GATE(PMU_GATE_SPI, "pmu_spi", 8, NULL, 0, "1e100800.spi", NULL),
	PMU_GATE(PMU_GATE_DSL_DFE, "pmu_dsl_dfe", 9, NULL, 0, NULL, NULL),
	PMU_GATE(PMU_GATE_EBU, "pmu_ebu", 10, NULL, CLK_IGNORE_UNUSED,
		 "1e105300.ebu", NULL),
	PMU_GATE(PMU_GATE_STP, "pmu_stp", 11, NULL, 0, "1e100bb0.stp", NULL),
	PMU_GATE(PMU_GATE_GPTC, "pmu_gptu", 12, NULL, 0,
		 "1e100a00.gptu", NULL),
	PMU_GATE(PMU_GATE_ETOP, "pmu_etop", 13, NULL, CLK_IGNORE_UNUSED,
		 "1e180000.etop", NULL),
	PMU_GATE(PMU_GATE_FPI0, "pmu_fpi0", 14, NULL, CLK_IGNORE_UNUSED,
		 "10000000.fpi", NULL),
	PMU_GATE(PMU_GATE_AHB, "pmu_ahb", 15, NULL, 0, NULL, NULL),
	PMU_GATE(PMU_GATE_SDIO, "pmu_sdio", 16, NULL, 0, NULL, NULL),
	PMU_GATE(PMU_GATE_UART1, "pmu_serial1", 17, NULL, 0,
		 "1e100c00.serial", NULL),
	PMU_GATE(PMU_GATE_DEU, "pmu_deu", 20, NULL, 0, NULL, NULL),
	PMU_GATE(PMU_GATE_PPE_TC, "pmu_ppe_tc", 21, NULL, 0, NULL, NULL),
	PMU_GATE(PMU_GATE_PPE_EMA, "pmu_ppe_ema", 22, NULL, 0, NULL, NULL),
	PMU_GATE(PMU_GATE_PPE_DPLUS, "pmu_ppe_dplus", 23, NULL, 0, NULL, NULL),
	PMU_GATE(PMU_GATE_TDM, "pmu_tdm", 25, NULL, 0, NULL, NULL),
	PMU_GATE(PMU_GATE_USB1_PHY, "pmu_usb1_phy", 26, NULL, 0, NULL, NULL),
	PMU_GATE(PMU_GATE_USB1_CTRL, "pmu_usb1_ctrl", 27, NULL, 0, NULL, NULL),
	PMU_GATE(PMU_GATE_SWITCH, "pmu_switch", 28, NULL, 0,
		 "1e180000.etop", "switch"),
};

static void __init ar9_pmu_clk_gates_init_dt(struct device_node *np)
{
	ltq_xway_pmu_register_gates(np,
				    ar9_pmu_gates,
				    ARRAY_SIZE(ar9_pmu_gates),
				    &xway_pmu_clk_gate_ops);
}

CLK_OF_DECLARE(pmu_ar9, "lantiq,pmu-ar9", ar9_pmu_clk_gates_init_dt);
