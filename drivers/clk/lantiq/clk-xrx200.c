/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 * Copyright (C) 2015 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 */

#include <dt-bindings/clock/lantiq-pmu-xrx200.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/types.h>

#include "clk-xway.h"

#define XRX200_PMU(_id, _name, _func, _parent, _flags, _dev, _con) \
	PMU_GATE(XRX200_PMU_GATE_##_id, _name, _func, _parent, _flags, _dev, _con)

unsigned long ltq_xrx200_cpu_recalc_rate(struct clk_hw *hw,
					 unsigned long parent_rate)
{
	struct ltq_cgu_clk *cgu_clk = to_ltq_cgu_clk(hw);
	u32 cpu_sel;
	unsigned long rate;

	cpu_sel = (ltq_cgu_clk_read(cgu_clk, CGU_SYS_XRX) >> 4) & 0xf;

	switch (cpu_sel) {
	case 0:
		rate = CLOCK_600M;
		break;
	case 1:
		rate = CLOCK_500M;
		break;
	case 2:
		rate = CLOCK_393M;
		break;
	case 3:
		rate = CLOCK_333M;
		break;
	case 5:
	case 6:
		rate = CLOCK_196_608M;
		break;
	case 7:
		rate = CLOCK_167M;
		break;
	case 4:
	case 8:
	case 9:
		rate = CLOCK_125M;
		break;
	default:
		pr_warn("%s: invalid cpu_sel %u\n", __func__, cpu_sel);
		rate = 0;
		break;
	}

	return rate;
}

unsigned long ltq_xrx200_fpi_recalc_rate(struct clk_hw *hw,
					 unsigned long parent_rate)
{
	struct ltq_cgu_clk *cgu_clk = to_ltq_cgu_clk(hw);
	u32 ocp_sel;
	unsigned long rate;

	ocp_sel = ltq_cgu_clk_read(cgu_clk, CGU_SYS_XRX) & 0x3;

	switch (ocp_sel) {
	case 0:
		/* OCP ratio 1 */
		rate = parent_rate;
		break;
	case 2:
		/* OCP ratio 2 */
		rate = parent_rate / 2;
		break;
	case 3:
		/* OCP ratio 2.5 */
		rate = (parent_rate * 2) / 5;
		break;
	case 4:
		/* OCP ratio 3 */
		rate = parent_rate / 3;
		break;
	default:
		pr_warn("%s: invalid ocp_sel %u\n", __func__, ocp_sel);
		rate = 0;
		break;
	}

	return rate;
}

unsigned long ltq_xrx200_pp32_recalc_rate(struct clk_hw *hw,
					  unsigned long parent_rate)
{
	struct ltq_cgu_clk *cgu_clk = to_ltq_cgu_clk(hw);
	u32 clksys;
	unsigned long rate;

	clksys = (ltq_cgu_clk_read(cgu_clk, CGU_SYS) >> 16) & 0x7;

	switch (clksys) {
	case 0:
		rate = CLOCK_500M;
		break;
	case 1:
		rate = CLOCK_432M;
		break;
	case 2:
		rate = CLOCK_288M;
		break;
	default:
		rate = CLOCK_500M;
		break;
	}

	return rate;
}

const struct clk_ops xrx200_cpu_clk_ops = {
	.recalc_rate = ltq_xrx200_cpu_recalc_rate,
};

const struct clk_ops xrx200_fpi_clk_ops = {
	.recalc_rate = ltq_xrx200_fpi_recalc_rate,
};

const struct clk_ops xrx200_pp32_clk_ops = {
	.recalc_rate = ltq_xrx200_pp32_recalc_rate,
};

static void __init xrx200_cgu_clocks_init_dt(struct device_node *np)
{
	struct ltq_cgu_clocks cgu_clocks = {};

	ltq_xway_cgu_init_dt(np);

	ltq_xrx_cgu_select_gphy_clk_input(np);

	cgu_clocks.cpu_clk = ltq_cgu_register_clk("cpu", NULL,
						  &xrx200_cpu_clk_ops);
	cgu_clocks.fpi_clk = ltq_cgu_register_clk("fpi", "cpu",
						  &xrx200_fpi_clk_ops);
	cgu_clocks.io_clk = cgu_clocks.fpi_clk; /* FPI clock is used for IO */
	cgu_clocks.pp32_clk = ltq_cgu_register_clk("pp32", NULL,
						   &xrx200_pp32_clk_ops);
	cgu_clocks.pci_clk = ltq_xway_register_cgu_pci_clk(
		CGU_IFCCR_XRX, CGU_PCICR_XRX);
	cgu_clocks.pci_ext_clk = ltq_xway_register_cgu_pci_ext_clk(
		CGU_IFCCR_XRX, CGU_PCICR_XRX);

	ltq_cgu_clk_of_add_provider(np, &cgu_clocks);
}

CLK_OF_DECLARE(cgu_xrx200, "lantiq,xrx200-cgu", xrx200_cgu_clocks_init_dt);

void __init xrx200_cgu_clocks_init_compat(struct device_node *np)
{
	xrx200_cgu_clocks_init_dt(np);
}

/*
 * NOTE: Hardware does not support reading bits 3, 4, 13 and 31.
 */
static struct ltq_xway_pmu_gate xrx200_pmu_gates[] __initdata = {
	XRX200_PMU(USB0_PHY, "usb0_phy", 0, NULL, 0, "1e101000.usb", "phy"),
	XRX200_PMU(DFEV0, "dfev0", 2, NULL, 0, NULL, NULL),
	XRX200_PMU(DFEV1, "dfev1", 3, NULL, 0, NULL, NULL),
	XRX200_PMU(PCI, "pci", 4, NULL, CLK_IGNORE_UNUSED, "1e105400.pci", NULL),
	XRX200_PMU(DMA, "dma", 5, NULL, 0, "1e104100.dma", NULL),
	XRX200_PMU(USB0_CTRL, "usb0_ctrl", 6, "pmu_ahbm", 0, "1e101000.usb", "ctl"),
	XRX200_PMU(USIF, "usif", 7, NULL, 0, "1da00000.usif", NULL),
	XRX200_PMU(SPI, "spi", 8, NULL, 0, "1e100800.spi", NULL),
	XRX200_PMU(DSL_DFE, "dsl_dfe", 9, NULL, 0, "1e116000.mei", "dfe"),
	XRX200_PMU(EBU, "ebu", 10, NULL, CLK_IGNORE_UNUSED, "1e105300.ebu", NULL),
	XRX200_PMU(STP, "ledc", 11, NULL, 0, "1e100bb0.stp", NULL),
	XRX200_PMU(GPTC, "gptu", 12, NULL, 0, "1e100a00.gptu", NULL),
	XRX200_PMU(FPI0, "fpi_master", 14, NULL, CLK_IGNORE_UNUSED, "10000000.fpi", NULL),
	XRX200_PMU(AHBM, "ahbm", 15, NULL, 0, NULL, "ahb"),
	XRX200_PMU(SDIO, "sdio", 16, NULL, 0, "1e103000.sdio", NULL),
	XRX200_PMU(UART1, "serial1", 17, NULL, CLK_IGNORE_UNUSED, "1e100c00.serial", NULL),
	XRX200_PMU(PPE_QSB, "ppe_qsb", 18, NULL, 0, "1e108000.eth", "ppe_qsb"),
	XRX200_PMU(PPE_SLL01, "ppe_sll01", 19, NULL, 0, "1e108000.eth", "ppe_sll01"),
	XRX200_PMU(DEU, "deu", 20, NULL, 0, "1e103100.deu", NULL),
	XRX200_PMU(PPE_TC, "ppe_tc", 21, NULL, 0, "1e108000.eth", "ppe_tc"),
	XRX200_PMU(PPE_EMA, "ppe_ema", 22, NULL, 0, "1e108000.eth", "ppe_ema"),
	XRX200_PMU(PPE_DPLUM, "ppe_dplum", 23, NULL, 0, "1e108000.eth", "ppe_dplum"),
	XRX200_PMU(PPE_DPLUS, "ppe_dplus", 24, NULL, 0, "1e108000.eth", "ppe_dplus"),
	XRX200_PMU(TDM, "tdm", 25, NULL, 0, NULL, NULL),
	XRX200_PMU(USB1_PHY, "usb1_phy", 26, NULL, 0, "1e106000.usb", "phy"),
	XRX200_PMU(USB1_CTRL, "usb1_ctrl", 27, "pmu_ahbm", 0, "1e106000.usb", "ctl"),
	XRX200_PMU(SWITCH, "switch", 28, NULL, 0, "1e108000.eth", "switch"),
	XRX200_PMU(PPE_TOP, "ppe_top", 29, NULL, 0, "1e108000.eth", "ppe_top"),
	XRX200_PMU(GPHY, "gphy", 30, NULL, 0, "1f203000.rcu", "gphy"),
	XRX200_PMU(PCIE0_CLK, "pcie0_clk", 31, NULL, 0, "1d900000.pcie", "bus"),
	XRX200_PMU(PCIE0_PHY, "pcie0_phy", 32, NULL, 0, "1d900000.pcie", "phy"),
	XRX200_PMU(PCIE0_CTRL, "pcie0_ctrl", 33, NULL, 0, "1d900000.pcie", "ctl"),
	XRX200_PMU(PCIE0_PDI, "pcie0_pdi", 36, NULL, 0, "1d900000.pcie", "pdi"),
	XRX200_PMU(PCIE0_MSI, "pcie0_msi", 37, NULL, 0, "1d900000.pcie", "msi"),
	/* not supported in v1.1: */
	XRX200_PMU(DDR_CKE, "ddr_cke", 38, NULL, CLK_IGNORE_UNUSED, NULL, NULL),
};

static void __init xrx200_pmu_clk_gates_init_dt(struct device_node *np)
{
	ltq_xway_pmu_register_gates(np,
				    xrx200_pmu_gates,
				    ARRAY_SIZE(xrx200_pmu_gates),
				    &xway_pmu_clk_gate_ops);
}

CLK_OF_DECLARE(pmu_xrx200, "lantiq,xrx200-pmu", xrx200_pmu_clk_gates_init_dt);

void __init xrx200_pmu_clocks_init_compat(struct device_node *np)
{
	xrx200_pmu_clk_gates_init_dt(np);
}
