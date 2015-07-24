/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 * Copyright (C) 2015 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 */

#include <dt-bindings/clock/lantiq-pmu-xrx100.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/types.h>

#include "clk-xway.h"

#define XRX100_PMU(_id, _name, _func, _parent, _flags, _dev, _con) \
	PMU_GATE(XRX100_PMU_GATE_##_id, _name, _func, _parent, _flags, _dev, _con)

unsigned long ltq_xrx100_sys_hz(struct ltq_cgu_clk *cgu_clk)
{
	if (((ltq_cgu_clk_read(cgu_clk, CGU_SYS) >> 3) & 0x3) == 0x2)
		return CLOCK_393M;

	return CLOCK_333M;
}

unsigned long ltq_xrx100_fpi_hz(struct ltq_cgu_clk *cgu_clk)
{
	unsigned long sys_rate = ltq_xrx100_sys_hz(cgu_clk);
	unsigned long rate;

	if (ltq_cgu_clk_read(cgu_clk, CGU_SYS) & BIT(0))
		rate = sys_rate / 3;
	else
		rate = sys_rate / 2;

	return rate;
}

static unsigned long ltq_xrx100_cpu_recalc_rate(struct clk_hw *hw,
					     unsigned long parent_rate)
{
	struct ltq_cgu_clk *cgu_clk = to_ltq_cgu_clk(hw);
	unsigned long rate;

	if (ltq_cgu_clk_read(cgu_clk, CGU_SYS) & BIT(2))
		rate = ltq_xrx100_fpi_hz(cgu_clk);
	else
		rate = ltq_xrx100_sys_hz(cgu_clk);

	return rate;
}

static unsigned long ltq_xrx100_fpi_recalc_rate(struct clk_hw *hw,
				     unsigned long parent_rate)
{
	struct ltq_cgu_clk *cgu_clk = to_ltq_cgu_clk(hw);

	return ltq_xrx100_fpi_hz(cgu_clk);
}

const struct clk_ops xrx100_cpu_clk_ops = {
	.recalc_rate = ltq_xrx100_cpu_recalc_rate,
};

const struct clk_ops xrx100_fpi_clk_ops = {
	.recalc_rate = ltq_xrx100_fpi_recalc_rate,
};

static void __init xrx100_cgu_clocks_init_dt(struct device_node *np)
{
	struct ltq_cgu_clocks cgu_clocks = {};

	ltq_xway_cgu_init_dt(np);

	cgu_clocks.cpu_clk = ltq_cgu_register_clk("cpu", NULL,
						  &xrx100_cpu_clk_ops);
	cgu_clocks.fpi_clk = ltq_cgu_register_clk("fpi", NULL,
						  &xrx100_fpi_clk_ops);
	cgu_clocks.io_clk = cgu_clocks.fpi_clk; /* FPI clock is used for IO */
	cgu_clocks.pp32_clk = clk_register_fixed_rate(NULL, "pp32", NULL,
						      CLK_IS_ROOT, CLOCK_250M);
	cgu_clocks.pci_clk = ltq_xway_register_cgu_pci_clk(
		CGU_IFCCR, CGU_PCICR);
	cgu_clocks.pci_ext_clk = ltq_xway_register_cgu_pci_ext_clk(
		CGU_IFCCR, CGU_PCICR);

	ltq_cgu_clk_of_add_provider(np, &cgu_clocks);
}

CLK_OF_DECLARE(cgu_xrx100, "lantiq,xrx100-cgu", xrx100_cgu_clocks_init_dt);

void __init xrx100_cgu_clocks_init_compat(struct device_node *np)
{
	xrx100_cgu_clocks_init_dt(np);
}

static struct ltq_xway_pmu_gate xrx100_pmu_gates[] __initdata = {
	XRX100_PMU(USB0_PHY, "usb0_phy", 0, NULL, 0, "1e101000.usb", "phy"),
	XRX100_PMU(DFEV0, "dfev0", 2, NULL, 0, NULL, NULL),
	XRX100_PMU(DFEV1, "dfev1", 3, NULL, 0, NULL, NULL),
	XRX100_PMU(PCI, "pci", 4, NULL, CLK_IGNORE_UNUSED, "1e105400.pci", NULL),
	XRX100_PMU(DMA, "dma", 5, NULL, 0, "1e104100.dma", NULL),
	XRX100_PMU(USB0_CTRL, "usb0_ctrl", 6, "pmu_ahbm", 0, "1e101000.usb", "ctl"),
	XRX100_PMU(UART0, "serial0", 7, NULL, 0, "1e100400.serial", NULL),
	XRX100_PMU(SPI, "spi", 8, NULL, 0, "1e100800.spi", NULL),
	XRX100_PMU(DSL_DFE, "dsl_dfe", 9, NULL, 0, "1e116000.mei", "dfe"),
	XRX100_PMU(EBU, "ebu", 10, NULL, CLK_IGNORE_UNUSED, "1e105300.ebu", NULL),
	XRX100_PMU(STP, "ledc", 11, NULL, 0, "1e100bb0.stp", NULL),
	XRX100_PMU(GPTC, "gptu", 12, NULL, 0, "1e100a00.gptu", NULL),
	XRX100_PMU(VLYNQ, "vlynq", 13, "pmu_ahbm", 0, "1e180000.etop", NULL),
	XRX100_PMU(FPI0, "fpi_master", 14, NULL, CLK_IGNORE_UNUSED, "10000000.fpi", NULL),
	XRX100_PMU(AHBM, "ahbm", 15, NULL, CLK_IGNORE_UNUSED, NULL, NULL),
	XRX100_PMU(SDIO, "sdio", 16, NULL, 0, "1e103000.sdio", NULL),
	XRX100_PMU(UART1, "serial1", 17, NULL, CLK_IGNORE_UNUSED, "1e100c00.serial", NULL),
	XRX100_PMU(DEU, "deu", 20, NULL, 0, "1e103100.deu", NULL),
	XRX100_PMU(PPE_TC, "ppe_tc", 21, NULL, 0, NULL, NULL),
	XRX100_PMU(PPE_EMA, "ppe_ema", 22, NULL, 0, NULL, NULL),
	XRX100_PMU(PPE_DPLUS, "ppe_dplus", 23, NULL, 0, NULL, NULL),
	XRX100_PMU(TDM, "tdm", 25, NULL, 0, NULL, NULL),
	XRX100_PMU(USB1_PHY, "usb1_phy", 26, NULL, 0, "1e106000.usb", "phy"),
	XRX100_PMU(USB1_CTRL, "usb1_ctrl", 27, "pmu_ahbm", 0, "1e106000.usb", "ctl"),
	XRX100_PMU(SWITCH, "switch", 28, NULL, 0, "1e180000.etop", "switch"),
};

static void __init xrx100_pmu_clk_gates_init_dt(struct device_node *np)
{
	ltq_xway_pmu_register_gates(np,
				    xrx100_pmu_gates,
				    ARRAY_SIZE(xrx100_pmu_gates),
				    &xway_pmu_clk_gate_ops);
}

CLK_OF_DECLARE(pmu_xrx100, "lantiq,xrx100-pmu", xrx100_pmu_clk_gates_init_dt);

void __init xrx100_pmu_clocks_init_compat(struct device_node *np)
{
	xrx100_pmu_clk_gates_init_dt(np);
}
