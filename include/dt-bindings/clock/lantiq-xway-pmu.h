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
#define PMU_GATE_WDT0		14
#define PMU_GATE_WDT1		15
#define PMU_GATE_AHBM		16
#define PMU_GATE_SDIO		17
#define PMU_GATE_UART0		18
#define PMU_GATE_UART1		19
#define PMU_GATE_PPE_QSB	20
#define PMU_GATE_PPE_SLL01	21
#define PMU_GATE_DEU		22
#define PMU_GATE_PPE_TC		23
#define PMU_GATE_PPE_EMA	24
#define PMU_GATE_PPE_DPLUM	25
#define PMU_GATE_PPE_DPLUS	26
#define PMU_GATE_TDM		27
#define PMU_GATE_SWITCH		28
#define PMU_GATE_PPE_TPE	29
#define PMU_GATE_PPE_TOP	30
#define PMU_GATE_EPHY		31
#define PMU_GATE_GPHY		32
#define PMU_GATE_VODEC		33
#define PMU_GATE_VO_MIPS	34
#define PMU_GATE_PCIE0_CTRL	35
#define PMU_GATE_PCIE1_CTRL	36
#define PMU_GATE_PCIE0_PDI	37
#define PMU_GATE_PCIE0_MSI	38
#define PMU_GATE_PCIE1_PDI	39
#define PMU_GATE_PCIE1_MSI	40
#define PMU_GATE_PCIE2_CTRL	41
#define PMU_GATE_PCIE2_PDI	42
#define PMU_GATE_PCIE2_MSI	43
#define PMU_GATE_USB0_PHY	44
#define PMU_GATE_USB1_PHY	45
#define PMU_GATE_PCIE0_PHY	46
#define PMU_GATE_PCIE1_PHY	47
#define PMU_GATE_PCIE2_PHY	48
#define PMU_GATE_PCIE0_CLK	49
#define PMU_GATE_ADSL_AFE	50
#define PMU_GATE_DCDC_2V5	51
#define PMU_GATE_DCDC_1VX	52
#define PMU_GATE_DCDC_1V0	53
#define PMU_GATE_DDR_CKE	54

#define NUM_PMU_GATES		(PMU_GATE_DDR_CKE + 1)

#endif /* _DT_BINDINGS_CLOCK_LANTIQ_XWAY_PMU_H */
