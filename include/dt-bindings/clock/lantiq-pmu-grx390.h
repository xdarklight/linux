/*
 * Copyright (c) 2015 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Device Tree binding constants for the PMU modules found on Lantiq XWAY
 * GRX390 SoCs.
 */

#ifndef _DT_BINDINGS_CLOCK_LANTIQ_GRX390_PMU_H
#define _DT_BINDINGS_CLOCK_LANTIQ_GRX390_PMU_H

#define GRX390_PMU_GATE_DFEV0		0
#define GRX390_PMU_GATE_DFEV1		1
#define GRX390_PMU_GATE_DMA		2
#define GRX390_PMU_GATE_USB0_CTRL	3
#define GRX390_PMU_GATE_USIF		4
#define GRX390_PMU_GATE_SPI		5
#define GRX390_PMU_GATE_EBU		6
#define GRX390_PMU_GATE_STP		7
#define GRX390_PMU_GATE_GPTC		8
#define GRX390_PMU_GATE_UART1		9
#define GRX390_PMU_GATE_DEU		10
#define GRX390_PMU_GATE_PPE_TC		11
#define GRX390_PMU_GATE_PPE_EMA		12
#define GRX390_PMU_GATE_PPE_DPLUS	13
#define GRX390_PMU_GATE_TDM		14
#define GRX390_PMU_GATE_USB1_CTRL	15
#define GRX390_PMU_GATE_SWITCH		16
#define GRX390_PMU_GATE_PCIE0_CTRL	17
#define GRX390_PMU_GATE_PCIE1_CTRL	18
#define GRX390_PMU_GATE_PCIE0_PDI	19
#define GRX390_PMU_GATE_PCIE0_MSI	20
#define GRX390_PMU_GATE_DDR_CKE		21
#define GRX390_PMU_GATE_PCIE1_PDI	22
#define GRX390_PMU_GATE_PCIE1_MSI	23
#define GRX390_PMU_GATE_PCIE2_CTRL	24
#define GRX390_PMU_GATE_PCIE2_PDI	25
#define GRX390_PMU_GATE_PCIE2_MSI	26
#define GRX390_PMU_GATE_USB0_PHY	27
#define GRX390_PMU_GATE_USB1_PHY	28
#define GRX390_PMU_GATE_PCIE0_PHY	29
#define GRX390_PMU_GATE_PCIE1_PHY	30
#define GRX390_PMU_GATE_PCIE2_PHY	31
#define GRX390_PMU_GATE_DCDC_2V5	32
#define GRX390_PMU_GATE_DCDC_1VX	33
#define GRX390_PMU_GATE_DCDC_1V0	34

#endif /* _DT_BINDINGS_CLOCK_LANTIQ_GRX390_PMU_H */
