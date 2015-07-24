/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 * Copyright (C) 2015 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 */

#include <dt-bindings/clock/lantiq-pmu-danube.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/types.h>

#include "clk-xway.h"

#define DANUBE_PMU(_id, _name, _func, _parent, _flags, _dev, _con) \
	PMU_GATE(DANUBE_PMU_GATE_##_id, _name, _func, _parent, _flags, _dev, _con)

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
	default:
		pr_warn("%s: invalid ddr_sel %u\n", __func__, ddr_sel);
		rate = 0;
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
		CGU_IFCCR, CGU_PCICR);
	cgu_clocks.pci_ext_clk = ltq_xway_register_cgu_pci_ext_clk(
		CGU_IFCCR, CGU_PCICR);

	ltq_cgu_clk_of_add_provider(np, &cgu_clocks);
}

CLK_OF_DECLARE(cgu_danube, "lantiq,danube-cgu", danube_cgu_clocks_init_dt);

void danube_cgu_clocks_init_compat(struct device_node *np)
{
	danube_cgu_clocks_init_dt(np);
}

static struct ltq_xway_pmu_gate danube_pmu_gates[] __initdata = {
	DANUBE_PMU(USB0_PHY, "usb0_phy", 0, NULL, 0, "1e101000.usb", "phy"),
	DANUBE_PMU(VO_MIPS, "vo_mips", 2, NULL, 0, NULL, NULL),
	DANUBE_PMU(VODEC, "vodec", 3, NULL, 0, NULL, NULL),
	DANUBE_PMU(PCI, "pci", 4, NULL, CLK_IGNORE_UNUSED, "1e105400.pci", NULL),
	DANUBE_PMU(DMA, "dma", 5, NULL, 0, "1e104100.dma", NULL),
	DANUBE_PMU(USB0_CTRL, "usb0_ctrl", 6, NULL, 0, "1e101000.usb", "ctl"),
	DANUBE_PMU(UART0, "serial0", 7, NULL, 0, "1e100400.serial", NULL),
	DANUBE_PMU(SPI, "spi", 8, NULL, 0, "1e100800.spi", NULL),
	DANUBE_PMU(DSL_DFE, "dsl_dfe", 9, NULL, 0, "1e116000.mei", "dfe"),
	DANUBE_PMU(EBU, "ebu", 10, NULL, CLK_IGNORE_UNUSED, "1e105300.ebu", NULL),
	DANUBE_PMU(STP, "ledc", 11, NULL, 0, "1e100bb0.stp", NULL),
	DANUBE_PMU(GPTC, "gptu", 12, NULL, 0, "1e100a00.gptu", NULL),
	DANUBE_PMU(PPE_TPE, "ppe_tpe", 13, NULL, 0, "1e180000.etop", "ppe"),
	DANUBE_PMU(FPI0, "fpi_master", 14, NULL, CLK_IGNORE_UNUSED, "10000000.fpi", NULL),
	DANUBE_PMU(AHBM, "ahbm", 15, NULL, 0, NULL, NULL),
	DANUBE_PMU(SDIO, "sdio", 16, NULL, 0, "1e103000.sdio", NULL),
	DANUBE_PMU(UART1, "serial1", 17, NULL, CLK_IGNORE_UNUSED, "1e100c00.serial", NULL),
	DANUBE_PMU(WDT0, "wdt0", 18, NULL, 0, NULL, NULL),
	DANUBE_PMU(WDT1, "wdt1", 19, NULL, 0, NULL, NULL),
	DANUBE_PMU(DEU, "deu", 20, NULL, 0, "1e103100.deu", NULL),
	DANUBE_PMU(PPE_TC, "ppe_tc", 21, NULL, 0, NULL, NULL),
	DANUBE_PMU(PPE_EMA, "ppe_enet1", 22, NULL, 0, NULL, NULL),
	DANUBE_PMU(PPE_DPLUM, "ppe_enet0", 23, NULL, 0, NULL, NULL),
	DANUBE_PMU(TDM, "tdm", 25, NULL, 0, NULL, NULL),
};

static void __init danube_pmu_clk_gates_init_dt(struct device_node *np)
{
	ltq_xway_pmu_register_gates(np,
				    danube_pmu_gates,
				    ARRAY_SIZE(danube_pmu_gates),
				    &xway_pmu_clk_gate_ops);
}

CLK_OF_DECLARE(pmu_danube, "lantiq,danube-pmu", danube_pmu_clk_gates_init_dt);

void danube_pmu_clocks_init_compat(struct device_node *np)
{
	danube_pmu_clk_gates_init_dt(np);
}
