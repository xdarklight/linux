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

static unsigned long ltq_danube_ddr_hz(struct ltq_cgu_clk *cgu_clk)
{
	unsigned ddr_sel = ltq_cgu_clk_read(cgu_clk, CGU_SYS) & 0x3;
	unsigned long rate;

	switch (ddr_sel) {
	case 0:
		rate = CLOCK_167M;
		break;
	case 1:
		rate = CLOCK_133M;
		break;
	case 2:
		rate = CLOCK_111M;
		break;
	case 3:
		rate = CLOCK_83M;
		break;
	}

	return rate;
}

unsigned long ltq_danube_cpu_recalc_rate(struct clk_hw *hw,
					 unsigned long parent_rate)
{
	struct ltq_cgu_clk *cgu_clk = to_ltq_cgu_clk(hw);
	unsigned long rate;

	switch (ltq_cgu_clk_read(cgu_clk, CGU_SYS) & 0xc) {
	case 0:
		rate = CLOCK_333M;
		break;
	case 4:
		rate = ltq_danube_ddr_hz(cgu_clk);
		break;
	case 8:
		rate = ltq_danube_ddr_hz(cgu_clk) << 1;
		break;
	default:
		rate = ltq_danube_ddr_hz(cgu_clk) >> 1;
		break;
	}

	return rate;
}

unsigned long ltq_danube_fpi_recalc_rate(struct clk_hw *hw,
					 unsigned long parent_rate)
{
	struct ltq_cgu_clk *cgu_clk = to_ltq_cgu_clk(hw);
	unsigned long ddr_clock = ltq_danube_ddr_hz(cgu_clk);
	unsigned long rate;

	if (ltq_cgu_clk_read(cgu_clk, CGU_SYS) & 0x40)
		rate = ddr_clock >> 1;
	else
		rate = ddr_clock;

	return rate;
}

unsigned long ltq_danube_pp32_recalc_rate(struct clk_hw *hw,
					  unsigned long parent_rate)
{
	struct ltq_cgu_clk *cgu_clk = to_ltq_cgu_clk(hw);
	unsigned int clksys = (ltq_cgu_clk_read(cgu_clk, CGU_SYS) >> 7) & 3;
	unsigned long rate;

	switch (clksys) {
	case 1:
		rate = CLOCK_240M;
		break;
	case 2:
		rate = CLOCK_222M;
		break;
	case 3:
		rate = CLOCK_133M;
		break;
	default:
		rate = CLOCK_266M;
		break;
	}

	return rate;
}

const struct clk_ops danube_cpu_clk_ops = {
	.recalc_rate = ltq_danube_cpu_recalc_rate,
};

const struct clk_ops danube_fpi_clk_ops = {
	.recalc_rate = ltq_danube_fpi_recalc_rate,
};

const struct clk_ops danube_pp32_clk_ops = {
	.recalc_rate = ltq_danube_pp32_recalc_rate,
};

static void __init danube_cgu_clocks_init_dt(struct device_node *np)
{
	struct ltq_cgu_clocks cgu_clocks = {};

	ltq_xway_cgu_init_dt(np);

	cgu_clocks.cpu_clk = ltq_cgu_register_clk("cpu", NULL,
						  &danube_cpu_clk_ops);
	cgu_clocks.fpi_clk = ltq_cgu_register_clk("fpi", NULL,
						  &danube_fpi_clk_ops);
	cgu_clocks.io_clk = cgu_clocks.fpi_clk; /* FPI clock is used for IO */
	cgu_clocks.pp32_clk = ltq_cgu_register_clk("pp32", NULL,
						   &danube_pp32_clk_ops);
	cgu_clocks.pci_clk = ltq_xway_register_cgu_pci_clk(
		CGU_IFCCR, CGU_PCICR, "cgu_pci");
	cgu_clocks.pci_ext_clk = ltq_xway_register_cgu_pci_ext_clk(
		CGU_IFCCR, CGU_PCICR);

	ltq_cgu_clk_of_add_provider(np, &cgu_clocks);
}

CLK_OF_DECLARE(cgu_danube, "lantiq,danube-cgu", danube_cgu_clocks_init_dt);

static struct ltq_xway_pmu_gate danube_pmu_gates[] __initdata = {
	PMU_GATE(USB0_PHY, "usb0_phy", 0, NULL, 0, "1e101000.usb", "phy"),
	PMU_GATE(VO_MIPS, "vo_mips", 2, NULL, 0, NULL, NULL),
	PMU_GATE(VODEC, "vodec", 3, NULL, 0, NULL, NULL),
	PMU_GATE(PCI, "pci", 4, NULL, CLK_IGNORE_UNUSED, "1e105400.pci", NULL),
	PMU_GATE(DMA, "dma", 5, NULL, 0, "1e104100.dma", NULL),
	PMU_GATE(USB0_CTRL, "usb0_ctrl", 6, NULL, 0, "1e101000.usb", "ctl"),
	PMU_GATE(UART0, "serial0", 7, NULL, 0, "1e100400.serial", NULL),
	PMU_GATE(SPI, "spi", 8, NULL, 0, "1e100800.spi", NULL),
	PMU_GATE(DSL_DFE, "dsl_dfe", 9, NULL, 0, "1e116000.mei", "dfe"),
	PMU_GATE(EBU, "ebu", 10, NULL, CLK_IGNORE_UNUSED, "1e105300.ebu", NULL),
	PMU_GATE(STP, "ledc", 11, NULL, 0, "1e100bb0.stp", NULL),
	PMU_GATE(GPTC, "gptu", 12, NULL, 0, "1e100a00.gptu", NULL),
	PMU_GATE(PPE_TPE, "ppe_tpe", 13, NULL, 0, "1e180000.etop", "ppe"),
	PMU_GATE(FPI0, "fpi_master", 14, NULL, CLK_IGNORE_UNUSED, "10000000.fpi", NULL),
	PMU_GATE(AHBM, "ahbm", 15, NULL, 0, NULL, NULL),
	PMU_GATE(SDIO, "sdio", 16, NULL, 0, "1e103000.sdio", NULL),
	PMU_GATE(UART1, "serial1", 17, NULL, CLK_IGNORE_UNUSED, "1e100c00.serial", NULL),
	PMU_GATE(WDT0, "wdt0", 18, NULL, 0, NULL, NULL),
	PMU_GATE(WDT1, "wdt1", 19, NULL, 0, NULL, NULL),
	PMU_GATE(DEU, "deu", 20, NULL, 0, "1e103100.deu", NULL),
	PMU_GATE(PPE_TC, "ppe_tc", 21, NULL, 0, NULL, NULL),
	PMU_GATE(PPE_EMA, "ppe_enet1", 22, NULL, 0, NULL, NULL),
	PMU_GATE(PPE_DPLUM, "ppe_enet0", 23, NULL, 0, NULL, NULL),
	PMU_GATE(TDM, "tdm", 25, NULL, 0, NULL, NULL),
};

static void __init danube_pmu_clk_gates_init_dt(struct device_node *np)
{
	ltq_xway_pmu_register_gates(np,
				    danube_pmu_gates,
				    ARRAY_SIZE(danube_pmu_gates),
				    &xway_pmu_clk_gate_ops);
}

CLK_OF_DECLARE(pmu_danube, "lantiq,danube-pmu", danube_pmu_clk_gates_init_dt);
