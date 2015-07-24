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

#define PMU_PWDCR0		0x1C
#define PMU_PWDSR0		0x20

#define PMU_PWDCR1		0x24
#define PMU_PWDSR1		0x28

unsigned long ltq_vrx200_cpu_recalc_rate(struct clk_hw *hw,
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
		BUG();
	}

	return rate;
}

unsigned long ltq_vrx200_fpi_recalc_rate(struct clk_hw *hw,
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
		BUG();
	}

	return rate;
}

unsigned long ltq_vrx200_pp32_recalc_rate(struct clk_hw *hw,
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

const struct clk_ops vrx200_cpu_clk_ops = {
	.recalc_rate = ltq_vrx200_cpu_recalc_rate,
};

const struct clk_ops vrx200_fpi_clk_ops = {
	.recalc_rate = ltq_vrx200_fpi_recalc_rate,
};

const struct clk_ops vrx200_pp32_clk_ops = {
	.recalc_rate = ltq_vrx200_pp32_recalc_rate,
};

static void __init vrx200_cgu_clocks_init_dt(struct device_node *np)
{
	struct ltq_cgu_clocks cgu_clocks = {};

	ltq_xrx_cgu_select_gphy_clk_input(np);

	cgu_clocks.cpu_clk = ltq_cgu_register_clk("cpu", NULL,
						  &vrx200_cpu_clk_ops);
	cgu_clocks.fpi_clk = ltq_cgu_register_clk("fpi", "cpu",
						  &vrx200_fpi_clk_ops);
	cgu_clocks.io_clk = cgu_clocks.fpi_clk; /* FPI clock is used for IO */
	cgu_clocks.pp32_clk = ltq_cgu_register_clk("pp32", NULL,
						   &vrx200_pp32_clk_ops);
	cgu_clocks.pci_clk = ltq_xway_register_cgu_pci_clk(
		CGU_IFCCR_XRX, CGU_PCICR_XRX, "cgu_pci");
	cgu_clocks.pci_ext_clk = ltq_xway_register_cgu_pci_ext_clk(
		CGU_IFCCR_XRX, CGU_PCICR_XRX);

	ltq_cgu_clk_of_add_provider(np, &cgu_clocks);
}

CLK_OF_DECLARE(cgu_vrx200, "lantiq,vrx200-cgu", vrx200_cgu_clocks_init_dt);

/*
 * NOTE: Hardware does not support reading bits 3, 4, 13 and 31.
 */
static struct ltq_xway_pmu_gate vrx200_pmu_gates[] __initdata = {
	PMU_GATE(USB0_PHY, "usb0_phy", 0, NULL, 0, "1e101000.usb", "phy"),
	PMU_GATE(DFEV0, "dfev0", 2, NULL, 0, NULL, NULL),
	PMU_GATE(DFEV1, "dfev1", 3, NULL, 0, NULL, NULL),
	PMU_GATE(PCI, "pci", 4, NULL, CLK_IGNORE_UNUSED, "1e105400.pci", NULL),
	PMU_GATE(DMA, "dma", 5, NULL, 0, "1e104100.dma", NULL),
	PMU_GATE(USB0_CTRL, "usb0_ctrl", 6, "pmu_ahbm", 0, "1e101000.usb", "ctl"),
	PMU_GATE(USIF, "usif", 7, NULL, 0, "1da00000.usif", NULL),
	PMU_GATE(SPI, "spi", 8, NULL, 0, "1e100800.spi", NULL),
	PMU_GATE(DSL_DFE, "dsl_dfe", 9, NULL, 0, "1e116000.mei", "dfe"),
	PMU_GATE(EBU, "ebu", 10, NULL, CLK_IGNORE_UNUSED, "1e105300.ebu", NULL),
	PMU_GATE(STP, "ledc", 11, NULL, 0, "1e100bb0.stp", NULL),
	PMU_GATE(GPTC, "gptu", 12, NULL, 0, "1e100a00.gptu", NULL),
	PMU_GATE(FPI0, "fpi_master", 14, NULL, CLK_IGNORE_UNUSED, "10000000.fpi", NULL),
	PMU_GATE(AHBM, "ahbm", 15, NULL, 0, NULL, NULL),
	PMU_GATE(SDIO, "sdio", 16, NULL, 0, "1e103000.sdio", NULL),
	PMU_GATE(UART1, "serial1", 17, NULL, CLK_IGNORE_UNUSED, "1e100c00.serial", NULL),
	PMU_GATE(PPE_QSB, "ppe_qsb", 18, NULL, 0, "1e108000.eth", "ppe_qsb"),
	PMU_GATE(PPE_SLL01, "ppe_sll01", 19, NULL, 0, "1e108000.eth", "ppe_sll01"),
	PMU_GATE(DEU, "deu", 20, NULL, 0, "1e103100.deu", NULL),
	PMU_GATE(PPE_TC, "ppe_tc", 21, NULL, 0, "1e108000.eth", "ppe_tc"),
	PMU_GATE(PPE_EMA, "ppe_ema", 22, NULL, 0, "1e108000.eth", "ppe_ema"),
	PMU_GATE(PPE_DPLUM, "ppe_dplum", 23, NULL, 0, "1e108000.eth", "ppe_dplum"),
	PMU_GATE(PPE_DPLUS, "ppe_dplus", 24, NULL, 0, "1e108000.eth", "ppe_dplus"),
	PMU_GATE(TDM, "tdm", 25, NULL, 0, NULL, NULL),
	PMU_GATE(USB1_PHY, "usb1_phy", 26, NULL, 0, "1e106000.usb", "phy"),
	PMU_GATE(USB1_CTRL, "usb1_ctrl", 27, "pmu_ahbm", 0, "1e106000.usb", "ctl"),
	PMU_GATE(SWITCH, "switch", 28, NULL, 0, "1e108000.eth", "switch"),
	PMU_GATE(PPE_TOP, "ppe_top", 29, NULL, 0, "1e108000.eth", "ppe_top"),
	PMU_GATE(GPHY, "gphy", 30, NULL, 0, "1f203000.rcu", "gphy"),
	PMU_GATE(PCIE0_CLK, "pcie0_clk", 31, NULL, 0, "1d900000.pcie", "bus"),
	PMU_GATE(PCIE0_PHY, "pcie0_phy", 32, NULL, 0, "1d900000.pcie", "phy"),
	PMU_GATE(PCIE0_CTRL, "pcie0_ctrl", 33, NULL, 0, "1d900000.pcie", "ctl"),
	PMU_GATE(PCIE0_PDI, "pcie0_pdi", 36, NULL, 0, "1d900000.pcie", "pdi"),
	PMU_GATE(PCIE0_MSI, "pcie0_msi", 37, NULL, 0, "1d900000.pcie", "msi"),
	/* not supported in v1.1: */
	PMU_GATE(DDR_CKE, "ddr_cke", 38, NULL, CLK_IGNORE_UNUSED, NULL, NULL),
};

static void __init vrx200_pmu_clk_gates_init_dt(struct device_node *np)
{
	ltq_xway_pmu_register_gates(np,
				    vrx200_pmu_gates,
				    ARRAY_SIZE(vrx200_pmu_gates),
				    &xway_pmu_clk_gate_ops);
}

CLK_OF_DECLARE(pmu_vrx200, "lantiq,vrx200-pmu", vrx200_pmu_clk_gates_init_dt);
