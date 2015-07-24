/*
 * Copyright (c) 2015 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Device Tree binding constants for the PMU modules found on Lantiq XWAY
 * XRX200 (GRX200, VRX200, VRX220) SoCs.
 */

#ifndef _DT_BINDINGS_CLOCK_LANTIQ_XRX200_PMU_H
#define _DT_BINDINGS_CLOCK_LANTIQ_XRX200_PMU_H

#define XRX200_PMU_GATE_USB0_PHY	0
#define XRX200_PMU_GATE_DFEV0		1
#define XRX200_PMU_GATE_DFEV1		2
#define XRX200_PMU_GATE_PCI		3
#define XRX200_PMU_GATE_DMA		4
#define XRX200_PMU_GATE_USB0_CTRL	5
#define XRX200_PMU_GATE_USIF		6
#define XRX200_PMU_GATE_SPI		7
#define XRX200_PMU_GATE_DSL_DFE		8
#define XRX200_PMU_GATE_EBU		9
#define XRX200_PMU_GATE_STP		10
#define XRX200_PMU_GATE_GPTC		11
#define XRX200_PMU_GATE_FPI0		12
#define XRX200_PMU_GATE_AHBM		13
#define XRX200_PMU_GATE_SDIO		14
#define XRX200_PMU_GATE_UART1		15
#define XRX200_PMU_GATE_PPE_QSB		16
#define XRX200_PMU_GATE_PPE_SLL01	17
#define XRX200_PMU_GATE_DEU		18
#define XRX200_PMU_GATE_PPE_TC		19
#define XRX200_PMU_GATE_PPE_EMA		20
#define XRX200_PMU_GATE_PPE_DPLUM	21
#define XRX200_PMU_GATE_PPE_DPLUS	22
#define XRX200_PMU_GATE_TDM		23
#define XRX200_PMU_GATE_USB1_PHY	24
#define XRX200_PMU_GATE_USB1_CTRL	25
#define XRX200_PMU_GATE_SWITCH		26
#define XRX200_PMU_GATE_PPE_TOP		27
#define XRX200_PMU_GATE_GPHY		28
#define XRX200_PMU_GATE_PCIE0_CLK	29
#define XRX200_PMU_GATE_PCIE0_PHY	30
#define XRX200_PMU_GATE_PCIE0_CTRL	31
#define XRX200_PMU_GATE_PCIE0_PDI	32
#define XRX200_PMU_GATE_PCIE0_MSI	33
#define XRX200_PMU_GATE_DDR_CKE		34

#endif /* _DT_BINDINGS_CLOCK_LANTIQ_XRX200_PMU_H */
