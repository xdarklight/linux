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
#define CGU_SYS_VR9		0x0c
#define CGU_IFCCR_VR9		0x24
#define CGU_PCICR_VR9		0x38

#define PMU_PWDCR0		0x1C
#define PMU_PWDSR0		0x20

#define PMU_PWDCR1		0x24
#define PMU_PWDSR1		0x28

unsigned long ltq_vr9_cpu_recalc_rate(struct clk_hw *hw,
				      unsigned long parent_rate)
{
	struct ltq_cgu_clk *cgu_clk = to_ltq_cgu_clk(hw);
	u32 cpu_sel;
	unsigned long rate;

	cpu_sel = (ltq_cgu_clk_read(cgu_clk, CGU_SYS_VR9) >> 4) & 0xf;

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

unsigned long ltq_vr9_fpi_recalc_rate(struct clk_hw *hw,
				      unsigned long parent_rate)
{
	struct ltq_cgu_clk *cgu_clk = to_ltq_cgu_clk(hw);
	u32 ocp_sel;
	unsigned long rate;

	ocp_sel = ltq_cgu_clk_read(cgu_clk, CGU_SYS_VR9) & 0x3;

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

unsigned long ltq_vr9_pp32_recalc_rate(struct clk_hw *hw,
				       unsigned long parent_rate)
{
	struct ltq_cgu_clk *cgu_clk = to_ltq_cgu_clk(hw);
	u32 clksys;
	unsigned long rate;

	clksys = (ltq_cgu_clk_read(cgu_clk, CGU_SYS) >> 16) & 3;

	switch (clksys) {
	case 1:
		rate = CLOCK_450M;
		break;
	case 2:
		rate = CLOCK_300M;
		break;
	default:
		rate = CLOCK_500M;
		break;
	}

	return rate;
}

const struct clk_ops vr9_cpu_clk_ops = {
	.recalc_rate = ltq_vr9_cpu_recalc_rate,
};

const struct clk_ops vr9_fpi_clk_ops = {
	.recalc_rate = ltq_vr9_fpi_recalc_rate,
};

const struct clk_ops vr9_pp32_clk_ops = {
	.recalc_rate = ltq_vr9_pp32_recalc_rate,
};

static void __init vr9_cgu_clocks_init_dt(struct device_node *np)
{
	struct ltq_cgu_clocks cgu_clocks = {};

	ltq_xway_cgu_init_dt(np);

	ltq_cgu_select_gphy_clk_input(CGU_IFCCR_VR9);

	cgu_clocks.cpu_clk = ltq_cgu_register_clk("cpu", NULL,
						  &vr9_cpu_clk_ops);
	cgu_clocks.fpi_clk = ltq_cgu_register_clk("fpi", "cpu",
						  &vr9_fpi_clk_ops);
	cgu_clocks.io_clk = cgu_clocks.fpi_clk; /* FPI clock is used for IO */
	cgu_clocks.pp32_clk = ltq_cgu_register_clk("pp32", NULL,
						   &vr9_pp32_clk_ops);
	cgu_clocks.pci_clk = ltq_xway_register_cgu_pci_clk(
		CGU_IFCCR_VR9, CGU_PCICR_VR9, "pmu_pci");
	cgu_clocks.pci_ext_clk = ltq_xway_register_cgu_pci_ext_clk(
		CGU_IFCCR_VR9, CGU_PCICR_VR9);

	ltq_cgu_clk_of_add_provider(np, &cgu_clocks);
}

CLK_OF_DECLARE(cgu_vr9, "lantiq,cgu-vr9", vr9_cgu_clocks_init_dt);

static struct ltq_xway_pmu_gate vr9_pmu_gates[] __initdata = {
	PMU_GATE(PMU_GATE_USB0_PHY, "pmu_usb0_phy", 0, NULL, 0, NULL, NULL),
	PMU_GATE(PMU_GATE_DFEV0, "pmu_dfev0", 2, NULL, 0, NULL, NULL),
	PMU_GATE(PMU_GATE_DFEV1, "pmu_dfev1", 3, NULL, 0, NULL, NULL),
	PMU_GATE(PMU_GATE_PCI, "pmu_pci", 4, NULL, CLK_IGNORE_UNUSED, NULL,
		 NULL),
	PMU_GATE(PMU_GATE_DMA, "pmu_dma", 5, NULL, 0, "1e104100.dma", NULL),
	PMU_GATE(PMU_GATE_USB0_CTRL, "pmu_usb0_ctrl", 6, "pmu_ahbm", 0,
		 NULL, NULL),
	PMU_GATE(PMU_GATE_USIF, "pmu_usif", 7, NULL, 0, NULL, NULL),
	PMU_GATE(PMU_GATE_SPI, "pmu_spi", 8, NULL, 0, "1e100800.spi", NULL),
	PMU_GATE(PMU_GATE_DSL_DFE, "pmu_dsl_dfe", 9, NULL, 0, NULL, NULL),
	PMU_GATE(PMU_GATE_EBU, "pmu_ebu", 10, NULL, CLK_IGNORE_UNUSED,
		 "1e105300.ebu", NULL),
	PMU_GATE(PMU_GATE_STP, "pmu_stp", 11, NULL, 0, "1e100bb0.stp", NULL),
	PMU_GATE(PMU_GATE_GPTC, "pmu_gptu", 12, NULL, 0, "1e100a00.gptu",
		 NULL),
	PMU_GATE(PMU_GATE_AHBS, "pmu_ahbs", 13, "pmu_ahbm", CLK_IGNORE_UNUSED,
		 "1d900000.pcie", "ahb"),
	PMU_GATE(PMU_GATE_FPI0, "pmu_fpi0", 14, NULL, CLK_IGNORE_UNUSED,
		 "10000000.fpi", NULL),
	PMU_GATE(PMU_GATE_AHBM, "pmu_ahbm", 15, NULL, 0, NULL, NULL),
	PMU_GATE(PMU_GATE_SDIO, "pmu_sdio", 16, NULL, 0, NULL, NULL),
	PMU_GATE(PMU_GATE_UART1, "pmu_serial1", 17, NULL, 0,
		 "1e100c00.serial", NULL),
	PMU_GATE(PMU_GATE_PPE_QSB, "pmu_ppe_qsb", 18, NULL, 0,
		"1e108000.eth", "ppe_qsb"),
	PMU_GATE(PMU_GATE_PPE_SLL01, "pmu_ppe_sll01", 19, NULL, 0,
		"1e108000.eth", "ppe_sll01"),
	PMU_GATE(PMU_GATE_DEU, "pmu_deu", 20, NULL, 0, NULL, NULL),
	PMU_GATE(PMU_GATE_PPE_TC, "pmu_ppe_tc", 21, NULL, 0,
		"1e108000.eth", "ppe_tc"),
	PMU_GATE(PMU_GATE_PPE_EMA, "pmu_ppe_ema", 22, NULL, 0,
		"1e108000.eth", "ppe_ema"),
	PMU_GATE(PMU_GATE_PPE_DPLUM, "pmu_ppe_dplum", 23, NULL, 0,
		"1e108000.eth", "ppe_dplum"),
	PMU_GATE(PMU_GATE_PPE_DPLUS, "pmu_ppe_dplus", 24, NULL, 0,
		"1e108000.eth", "ppe_dplus"),
	PMU_GATE(PMU_GATE_TDM, "pmu_tdm", 25, NULL, 0, NULL, NULL),
	PMU_GATE(PMU_GATE_USB1_PHY, "pmu_usb1_phy", 26, NULL, 0, NULL, NULL),
	PMU_GATE(PMU_GATE_USB1_CTRL, "pmu_usb1_ctrl", 27, "pmu_ahbm", 0,
		 NULL, NULL),
	PMU_GATE(PMU_GATE_SWITCH, "pmu_switch", 28, NULL, 0,
		 "1e108000.eth", "switch"),
	PMU_GATE(PMU_GATE_PPE_TOP, "pmu_ppe_top", 29, NULL, 0,
		"1e108000.eth", "ppe_top"),
	PMU_GATE(PMU_GATE_GPHY, "pmu_gphy", 30, NULL, 0,
		 "1f203000.rcu", "gphy"),
	PMU_GATE(PMU_GATE_PCIE0_CLK, "pmu_pcie0_clk", 31, NULL, 0,
		 "1d900000.pcie", "bus"),
	PMU_GATE(PMU_GATE_PCIE0_PHY, "pmu_pcie0_phy", 32, NULL, 0,
		 "1d900000.pcie", "phy"),
	PMU_GATE(PMU_GATE_PCIE0_CTRL, "pmu_pcie0_ctrl", 33, NULL, 0,
		 "1d900000.pcie", "ctl"),
	PMU_GATE(PMU_GATE_PCIE0_PDI, "pmu_pcie0_pdi", 36, NULL, 0,
		 "1d900000.pcie", "pdi"),
	PMU_GATE(PMU_GATE_PCIE0_MSI, "pmu_pcie1_msi", 37, NULL, 0,
		"1d900000.pcie", "msi"),
};

static void __init vr9_pmu_clk_gates_init_dt(struct device_node *np)
{
	ltq_xway_pmu_register_gates(np,
				    vr9_pmu_gates,
				    ARRAY_SIZE(vr9_pmu_gates),
				    &xway_pmu_clk_gate_ops);
}

CLK_OF_DECLARE(pmu_vr9, "lantiq,pmu-vr9", vr9_pmu_clk_gates_init_dt);
