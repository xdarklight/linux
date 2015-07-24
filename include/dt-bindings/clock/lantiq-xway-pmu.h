/*
 * Copyright (c) 2015 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Device Tree binding constants for the PMU module on lantiq xway SoCs.
 */

#ifndef _DT_BINDINGS_CLOCK_LANTIQ_XWAY_PMU_H
#define _DT_BINDINGS_CLOCK_LANTIQ_XWAY_PMU_H

#define PMU_GATE_DFEV0		0
#define PMU_GATE_DFEV1		1
#define PMU_GATE_PCI		2
#define PMU_GATE_DMA		3
#define PMU_GATE_USB0_CTRL	4
#define PMU_GATE_USB1_CTRL	5
#define PMU_GATE_USIF		6
#define PMU_GATE_SPI		7
#define PMU_GATE_DSL_DFE	8
#define PMU_GATE_EBU		9
#define PMU_GATE_STP		10
#define PMU_GATE_GPTC		11
#define PMU_GATE_AHBS		12
#define PMU_GATE_FPI0		13
#define PMU_GATE_AHB		14
#define PMU_GATE_WDT0		15
#define PMU_GATE_WDT1		16
#define PMU_GATE_AHBM		17
#define PMU_GATE_SDIO		18
#define PMU_GATE_UART0		19
#define PMU_GATE_UART1		20
#define PMU_GATE_PPE_QSB	21
#define PMU_GATE_PPE_SLL01	22
#define PMU_GATE_DEU		23
#define PMU_GATE_PPE_TC		24
#define PMU_GATE_PPE_EMA	25
#define PMU_GATE_PPE_DPLUM	26
#define PMU_GATE_PPE_DPLUS	27
#define PMU_GATE_TDM		28
#define PMU_GATE_SWITCH		29
#define PMU_GATE_ETOP		30
#define PMU_GATE_PPE_TOP	31
#define PMU_GATE_EPHY		32
#define PMU_GATE_GPHY		33
#define PMU_GATE_VODEC		34
#define PMU_GATE_VO_MIPS	35
#define PMU_GATE_PCIE0_CTRL	36
#define PMU_GATE_PCIE1_CTRL	37
#define PMU_GATE_PCIE0_PDI	38
#define PMU_GATE_PCIE0_MSI	39
#define PMU_GATE_PCIE1_PDI	40
#define PMU_GATE_PCIE1_MSI	41
#define PMU_GATE_PCIE2_CTRL	42
#define PMU_GATE_PCIE2_PDI	43
#define PMU_GATE_PCIE2_MSI	44
#define PMU_GATE_USB0_PHY	45
#define PMU_GATE_USB1_PHY	46
#define PMU_GATE_PCIE0_PHY	47
#define PMU_GATE_PCIE1_PHY	48
#define PMU_GATE_PCIE2_PHY	49
#define PMU_GATE_PCIE0_CLK	50
#define PMU_GATE_ADSL_AFE	51
#define PMU_GATE_DCDC_2V5	52
#define PMU_GATE_DCDC_1VX	53
#define PMU_GATE_DCDC_1V0	54

#define NUM_PMU_GATES		(PMU_GATE_DCDC_1V0 + 1)

#endif /* _DT_BINDINGS_CLOCK_LANTIQ_XWAY_PMU_H */
