// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * PCIe RC driver for Lantiq SoCs, based on the Synopsys DesignWare Core.
 *
 * Copyright (C) 2019 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 *
 * Based on the BSP (called "UGW") driver:
 * Copyright (C) 2009 Lei Chuanhua <chuanhua.lei@infineon.com>
 */
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/of_device.h>
#include <linux/pci.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/resource.h>
#include <linux/types.h>

#include "pcie-designware.h"

/* RC Core Control Register */
#define PCIE_APP_RC_CCR						0x10
#define PCIE_APP_RC_CCR_LTSSM_ENABLE				BIT(0)

/* RC Core Debug Register */
#define PCIE_APP_RC_DR						0x14
#define PCIE_APP_RC_DR_DLL_UP					BIT(0)

/* PHY Link Status Register */
#define PCIE_APP_PHY_SR						0x18
#define PCIE_APP_PHY_SR_PHY_LINK_UP				BIT(0)

/* AHB Control Register, fixed bus enumeration exception */
#define PCIE_APP_AHB_CTRL					0x78
#define PCIE_APP_AHB_CTRL_BUS_ERROR_SUPPRESS			BIT(0)

#define PCIE_APP_IRNEN						0xf4
#define PCIE_APP_IRNEN_AER_REPORT				BIT(0)
#define PCIE_APP_IRNEN_AER_MSIX					BIT(1)
#define PCIE_APP_IRNEN_PME					BIT(2)
#define PCIE_APP_IRNEN_HOTPLUG					BIT(3)
#define PCIE_APP_IRNEN_RX_VDM_MSG				BIT(4)
#define PCIE_APP_IRNEN_RX_CORRECTABLE_ERR_MSG			BIT(5)
#define PCIE_APP_IRNEN_RX_NON_FATAL_ERR_MSG			BIT(6)
#define PCIE_APP_IRNEN_RX_FATAL_ERR_MSG				BIT(7)
#define PCIE_APP_IRNEN_RX_PME_MSG				BIT(8)
#define PCIE_APP_IRNEN_RX_PME_TURNOFF_ACK			BIT(9)
#define PCIE_APP_IRNEN_AHB_BR_FATAL_ERR				BIT(10)
#define PCIE_APP_IRNEN_LINK_AUTO_BW_STATUS			BIT(11)
#define PCIE_APP_IRNEN_BW_MGT					BIT(12)
#define PCIE_APP_IRNEN_INTA					BIT(13)
#define PCIE_APP_IRNEN_INTB					BIT(14)
#define PCIE_APP_IRNEN_INTC					BIT(15)
#define PCIE_APP_IRNEN_INTD					BIT(16)
#define PCIE_APP_IRNEN_WAKEUP					BIT(17)

#define PCIE_APP_IRNCR						0xf8
#define PCIE_APP_IRNCR_AER_REPORT				BIT(0)
#define PCIE_APP_IRNCR_AER_MSIX					BIT(1)
#define PCIE_APP_IRNCR_PME					BIT(2)
#define PCIE_APP_IRNCR_HOTPLUG					BIT(3)
#define PCIE_APP_IRNCR_RX_VDM_MSG				BIT(4)
#define PCIE_APP_IRNCR_RX_CORRECTABLE_ERR_MSG			BIT(5)
#define PCIE_APP_IRNCR_RX_NON_FATAL_ERR_MSG			BIT(6)
#define PCIE_APP_IRNCR_RX_FATAL_ERR_MSG				BIT(7)
#define PCIE_APP_IRNCR_RX_PME_MSG				BIT(8)
#define PCIE_APP_IRNCR_RX_PME_TURNOFF_ACK			BIT(9)
#define PCIE_APP_IRNCR_AHB_BR_FATAL_ERR				BIT(10)
#define PCIE_APP_IRNCR_LINK_AUTO_BW_STATUS			BIT(11)
#define PCIE_APP_IRNCR_BW_MGT					BIT(12)
#define PCIE_APP_IRNCR_INTA					BIT(13)
#define PCIE_APP_IRNCR_INTB					BIT(14)
#define PCIE_APP_IRNCR_INTC					BIT(15)
#define PCIE_APP_IRNCR_INTD					BIT(16)
#define PCIE_APP_IRNCR_WAKEUP					BIT(17)

#define PCIE_APP_IRNICR						0xfc

#define PCIE_RC_PCICMDSTS					0x004

/* IO Base/Limit Register bits, RC only */
#define PCIE_RC_IOBLSECS					0x01c
#define PCIE_RC_IOBLSECS_32BIT_IO_ADDR				BIT(0)
#define PCIE_RC_IOBLSECS_IO_BASE_ADDR_MASK			GENMASK(7, 4)
#define PCIE_RC_IOBLSECS_32BIT_IOLIMT				BIT(8)
#define PCIE_RC_IOBLSECS_IO_LIMIT_ADDR_MASK			GENMASK(15, 12)

/* Bus Number Register bits, mandatory */
#define PCIE_RC_BNR						0x018
#define PCIE_RC_BNR_PRIMARY_BUS_NUM_MASK			GENMASK(7, 0)
#define PCIE_RC_BNR_SECONDARY_BUS_NUM_MASK			GENMASK(15, 8)
#define PCIE_RC_BNR_SUB_BUS_NUM_MASK				GENMASK(23, 16)

/* Non-prefetchable Memory Base/Limit Register bits, RC only */
#define PCIE_RC_MBML						0x020
#define PCIE_RC_MBML_MEM_BASE_ADDR_MASK				GENMASK(15, 4)
#define PCIE_RC_MBML_MEM_LIMIT_ADDR_MASK			GENMASK(31, 20)

/* Prefetchable Memory Base/Limit Register bits, RC only */
#define PCIE_RC_PMBL						0x024

/* I/O Base/Limit Upper 16 bits register, RC only */
#define PCIE_RC_IO_BANDL					0x030
#define PCIE_RC_IO_BANDL_UPPER_16BIT_IO_BASE_MASK		GENMASK(15, 0)
#define PCIE_RC_IO_BANDL_UPPER_16BIT_IO_LIMIT_MASK		GENMASK(31, 16)

/* Interrupt and Secondary Bridge Control Register */
#define PCIE_RC_INTRBCTRL					0x03c
#define PCIE_RC_INTRBCTRL_INT_LINE_MASK				GENMASK(7, 0)
#define PCIE_RC_INTRBCTRL_INT_PIN_MASK				GENMASK(15, 8)
#define PCIE_RC_INTRBCTRL_PARITY_ERR_RESP_ENABLE		BIT(16)
#define PCIE_RC_INTRBCTRL_SERR_ENABLE				BIT(17)
#define PCIE_RC_INTRBCTRL_ISA_ENABLE				BIT(18)
#define PCIE_RC_INTRBCTRL_VGA_ENABLE				BIT(19)
#define PCIE_RC_INTRBCTRL_VGA_16BIT_DECODE			BIT(20)
#define PCIE_RC_INTRBCTRL_RST_SECONDARY_BUS			BIT(22)

/* Power Management Control and Status Register */
#define PCIE_RC_PM_CSR						0x044
#define PCIE_RC_PM_CSR_POWER_STATE_MASK				GENMASK(1, 0)
#define PCIE_RC_PM_CSR_SW_RST					BIT(3)
#define PCIE_RC_PM_CSR_PME_ENABLE				BIT(8)
#define PCIE_RC_PM_CSR_PME_STATUS				BIT(15)

/* MSI Capability Register for EP */
#define PCIE_RC_MCAPR						0x050
#define PCIE_RC_MCAPR_MSI_CAP_ID_MASK				GENMASK(7, 0)
#define PCIE_RC_MCAPR_MSI_NEXT_CAP_PTR_MASK			GENMASK(15, 8)
#define PCIE_RC_MCAPR_MSI_ENABLE				BIT(16)
#define PCIE_RC_MCAPR_MULTI_MSG_CAP_MASK			GENMASK(19, 17)
#define PCIE_RC_MCAPR_MULTI_MSG_ENABLE_MASK			GENMASK(22, 20)
#define PCIE_RC_MCAPR_ADDR64_CAP				BIT(23)

/* Device Capability Register */
#define PCIE_RC_DCAP						0x074
#define PCIE_RC_DCAP_MAX_PAYLOAD_SIZE_MASK			GENMASK(2, 0)
#define PCIE_RC_DCAP_EXT_TAG					BIT(5)
#define PCIE_RC_DCAP_EP_L0S_LATENCY_MASK			GENMASK(8, 6)
#define PCIE_RC_DCAP_EP_L1_LATENCY_MASK				GENMASK(11, 9)
#define PCIE_RC_DCAP_ROLE_BASE_ERR_REPORT			BIT(15)

/* Device Control and Status Register */
#define PCIE_RC_DCTLSTS						0x078
#define PCIE_RC_DCTLSTS_CORRECTABLE_ERR_EN			BIT(0)
#define PCIE_RC_DCTLSTS_NONFATAL_ERR_EN				BIT(1)
#define PCIE_RC_DCTLSTS_FATAL_ERR_EN				BIT(2)
#define PCIE_DCTLSYS_UR_REQ_EN					BIT(3)
#define PCIE_RC_DCTLSTS_RELAXED_ORDERING_EN			BIT(4)
#define PCIE_RC_DCTLSTS_MAX_PAYLOAD_SIZE_MASK			GENMASK(7, 5)
#define PCIE_RC_DCTLSTS_EXT_TAG_EN				BIT(8)
#define PCIE_RC_DCTLSTS_PHANTOM_FUNC_EN				BIT(9)
#define PCIE_RC_DCTLSTS_AUX_PM_EN				BIT(10)
#define PCIE_RC_DCTLSTS_NO_SNOOP_EN				BIT(11)
#define PCIE_RC_DCTLSTS_MAX_READ_SIZE_MASK			GENMASK(14, 12)
#define PCIE_RC_DCTLSTS_CORRECTABLE_ERR				BIT(16)
#define PCIE_RC_DCTLSTS_NONFATAL_ERR				BIT(17)
#define PCIE_RC_DCTLSTS_FATAL_ER				BIT(18)
#define PCIE_RC_DCTLSTS_UNSUPPORTED_REQ				BIT(19)
#define PCIE_RC_DCTLSTS_AUX_POWER				BIT(20)
#define PCIE_RC_DCTLSTS_TRANSACT_PENDING			BIT(21)

/* Link Capability Register */
#define PCIE_RC_LCAP						0x07c
#define PCIE_RC_LCAP_MAX_LINK_SPEED				GENMASK(3, 0)
#define PCIE_RC_LCAP_MAX_LENGTH_WIDTH_MASK			GENMASK(9, 4)
#define PCIE_RC_LCAP_ASPM_LEVEL_MASK				GENMASK(11, 10)
#define PCIE_RC_LCAP_L0S_EIXT_LATENCY_MASK			GENMASK(14, 12)
#define PCIE_RC_LCAP_L1_EXIT_LATENCY_MASK			GENMASK(17, 15)
#define PCIE_RC_LCAP_CLK_PM					BIT(18)
#define PCIE_RC_LCAP_SDER					BIT(19)
#define PCIE_RC_LCAP_DLL_ACTIVE_REPORT				BIT(20)
#define PCIE_RC_LCAP_PORT_NUM_MASK				GENMASK(31, 24)

/* Link Control and Status Register */
#define PCIE_RC_LCTLSTS						0x080
#define PCIE_RC_LCTLSTS_ASPM_ENABLE_MASK			GENMASK(1, 0)
#define PCIE_RC_LCTLSTS_READ_COMPLETION_BOUNDARY_128		BIT(3)
#define PCIE_RC_LCTLSTS_LINK_DISABLE				BIT(4)
#define PCIE_RC_LCTLSTS_RETRAIN_LINK				BIT(5)
#define PCIE_RC_LCTLSTS_COMMON_CLK_CFG				BIT(6)
#define PCIE_RC_LCTLSTS_EXT_SYNC				BIT(7)
#define PCIE_RC_LCTLSTS_CLK_PM_EN				BIT(8)
#define PCIE_RC_LCTLSTS_LINK_SPEED_MASK				GENMASK(19, 16)
#define PCIE_RC_LCTLSTS_NEGOTIATED_LINK_WIDTH_MASK		GENMASK(25, 20)
#define PCIE_RC_LCTLSTS_RETRAIN_PENDING				BIT(27)
#define PCIE_RC_LCTLSTS_SLOT_CLK_CFG				BIT(28)
#define PCIE_RC_LCTLSTS_DLL_ACTIVE				BIT(29)

/* Slot Capabilities */
#define PCIE_RC_SLCTLSTS					0x088

/* Root Control and Capability Register */
#define PCIE_RC_RCTLCAP						0x08c
#define PCIE_RC_RCTLCAP_SERR_ON_CORRECTABLE_ERR			BIT(0)
#define PCIE_RC_RCTLCAP_SERR_ON_NONFATAL_ERR			BIT(1)
#define PCIE_RC_RCTLCAP_SERR_ON_FATAL_ERR			BIT(2)
#define PCIE_RC_RCTLCAP_PME_INT_EN				BIT(3)

/* Root Status Register */
#define PCIE_RC_RSTS						0x090

/* Uncorrectable Error Status Register */
#define PCIE_RC_UES_R						0x104

/* Uncorrectable Error Mask Register, Mask means no report */
#define PCIE_RC_UEMR						0x108
#define PCIE_RC_UEMR_DATA_LINK_PROTOCOL_ERR			BIT(4)
#define PCIE_RC_UEMR_SURPRISE_DOWN_ERROR			BIT(5)
#define PCIE_RC_UEMR_POISONED_TLP				BIT(12)
#define PCIE_RC_UEMR_FC_PROTOCOL_ERR				BIT(13)
#define PCIE_RC_UEMR_COMPLETION_TIMEOUT				BIT(14)
#define PCIE_RC_UEMR_COMPLETOR_ABORT				BIT(15)
#define PCIE_RC_UEMR_UNEXPECTED_COMPLETION			BIT(16)
#define PCIE_RC_UEMR_RECEIVER_OVERFLOW				BIT(17)
#define PCIE_RC_UEMR_MALFORNED_TLP				BIT(18)
#define PCIE_RC_UEMR_ECRC_ERR					BIT(19)
#define PCIE_RC_UEMR_UR_REQ					BIT(20)

/* Uncorrectable Error Severity Register */
#define PCIE_RC_UESR						0x10c
#define PCIE_RC_UESR_DATA_LINK_PROTOCOL_ERR			BIT(4)
#define PCIE_RC_UESR_SURPRISE_DOWN_ERROR			BIT(5)
#define PCIE_RC_UESR_POISONED_TLP				BIT(12)
#define PCIE_RC_UESR_FC_PROTOCOL_ERR				BIT(13)
#define PCIE_RC_UESR_COMPLETION_TIMEOUT				BIT(14)
#define PCIE_RC_UESR_COMPLETOR_ABORT				BIT(15)
#define PCIE_RC_UESR_UNEXPECTED_COMPLETION			BIT(16)
#define PCIE_RC_UESR_RECEIVER_OVERFLOW				BIT(17)
#define PCIE_RC_UESR_MALFORNED_TLP				BIT(18)
#define PCIE_RC_UESR_ECRC_ERR					BIT(19)
#define PCIE_RC_UESR_UR_REQ					BIT(20)

/* Correctable Error Status Register */
#define PCIE_RC_CESR						0x110
#define PCIE_RC_CESR_RX_ERR					BIT(0)
#define PCIE_RC_CESR_BAD_TLP					BIT(6)
#define PCIE_RC_CESR_BAD_DLLP					BIT(7)
#define PCIE_RC_CESR_REPLAY_NUM_ROLLOVER			BIT(8)
#define PCIE_RC_CESR_REPLAY_TIMER_TIMEOUT_ERR			BIT(12)
#define PCIE_RC_CESR_ADVISORY_NONFTAL_ERR			BIT(13)

/* Correctable Error Mask Register */
#define PCIE_RC_CEMR						0x114

/* Advanced Error Capabilities and Control Register */
#define PCIE_RC_AECCR						0x118
#define PCIE_RC_AECCR_FIRST_ERR_PTR_MASK			GENMASK(4, 0)
#define PCIE_RC_AECCR_ECRC_GEN_CAP				BIT(5)
#define PCIE_RC_AECCR_ECRC_GEN_EN				BIT(6)
#define PCIE_RC_AECCR_ECRC_CHECK_CAP				BIT(7)
#define PCIE_RC_AECCR_ECRC_CHECK_EN				BIT(8)

/* Root Error Command Register */
#define PCIE_RC_RECR						0x12c
#define PCIE_RC_RECR_CORRECTABLE_ERR_REPORT_EN			BIT(0)
#define PCIE_RC_RECR_NONFATAL_ERR_REPORT_EN			BIT(1)
#define PCIE_RC_RECR_FATAL_ERR_REPORT_EN			BIT(2)

/* Root Error Status Register */
#define PCIE_RC_RESR						0x130

/* Port VC Capability Register 2 */
#define PCIE_RC_PVC2						0x148
#define PCIE_RC_PVC2_VC_ARB_16P_FIXED_WRR			BIT(0)
#define PCIE_RC_PVC2_VC_ARB_32P_WRR				BIT(1)
#define PCIE_RC_PVC2_VC_ARB_64P_WRR				BIT(2)
#define PCIE_RC_PVC2_VC_ARB_128P_WRR				BIT(3)

/* Port VC Control and Status Register */
#define PCIE_RC_PVCCRSR						0x14c

/* VC0 Resource Capability Register */
#define PCIE_RC_VC0_RC						0x150
#define PCIE_RC_VC0_RC_REJECT_SNOOP				BIT(15)

/* Ack Frequency Register */
#define PCIE_RC_AFR						0x70c
#define PCIE_RC_AFR_AF_MASK					GENMASK(7, 0)
#define PCIE_RC_AFR_FTS_NUM_MASK				GENMASK(15, 8)
#define PCIE_RC_AFR_COM_FTS_NUM_MASK				GENMASK(23, 16)
#define PCIE_RC_AFR_L0S_ENTRY_LATENCY_MASK			GENMASK(26, 24)
#define PCIE_RC_AFR_L1_ENTRY_LATENCY_MASK			GENMASK(29, 27)

/* Lane Skew Register */
#define PCIE_RC_LSR						0x714
#define PCIE_RC_LSR_FC_DISABLE					BIT(24)
#define PCIE_RC_LSR_ACKNAK_DISABLE				BIT(25)
#define PCIE_RC_LSR_LANE_DESKEW_DISABLE				BIT(31)

/* Symbol Timer Register and Filter Mask Register 1 */
#define PCIE_RC_STRFMR						0x71c
#define PCIE_RC_STRFMR_SKP_INTERVAL_MASK			GENMASK(10, 0)
#define PCIE_RC_STRFMR_FC_WDT_DISABLE				BIT(15)
#define PCIE_RC_STRFMR_TLP_FUNC_MISMATCH_OK			BIT(16)
#define PCIE_RC_STRFMR_POISONED_TLP_OK				BIT(17)
#define PCIE_RC_STRFMR_BAR_MATCH_OK				BIT(18)
#define PCIE_RC_STRFMR_TYPE1_CFG_REQ_OK				BIT(19)
#define PCIE_RC_STRFMR_LOCKED_REQ_OK				BIT(20)
#define PCIE_RC_STRFMR_CPL_TAG_ERR_RULES_OK			BIT(21)
#define PCIE_RC_STRFMR_CPL_REQUESTOR_ID_MISMATCH_OK		BIT(22)
#define PCIE_RC_STRFMR_CPL_FUNC_MISMATCH_OK			BIT(23)
#define PCIE_RC_STRFMR_CPL_TC_MISMATCH_OK			BIT(24)
#define PCIE_RC_STRFMR_CPL_ATTR_MISMATCH_OK			BIT(25)
#define PCIE_RC_STRFMR_CPL_LENGTH_MISMATCH_OK			BIT(26)
#define PCIE_RC_STRFMR_TLP_ECRC_ERR_OK				BIT(27)
#define PCIE_RC_STRFMR_CPL_TLP_ECRC_OK				BIT(28)
#define PCIE_RC_STRFMR_RX_TLP_MSG_NO_DROP			BIT(29)
#define PCIE_RC_STRFMR_RX_IO_TRANS_ENABLE			BIT(30)
#define PCIE_RC_STRFMR_RX_CFG_TRANS_ENABLE			BIT(31)

/* Filter Masker Register 2 */
#define PCIE_RC_FMR2						0x720
#define PCIE_RC_FMR2_VENDOR_MSG0_PASSED_TO_TRGT1		BIT(0)
#define PCIE_RC_FMR2_VENDOR_MSG1_PASSED_TO_TRGT1		BIT(1)

/* Transmit Posted FC Credit Status Register */
#define PCIE_RC_TPFCS						0x730

/* Transmit Non-Posted FC Credit Status */
#define PCIE_RC_TNPFCS						0x734

/* Transmit Complete FC Credit Status Register */
#define PCIE_RC_TCFCS						0x738

/* Queue Status Register */
#define PCIE_RC_QSR						0x73c

/* VC0 Completion Receive Queue Control */
#define PCIE_RC_VC0_CRQCR					0x750
#define PCIE_RC_VC0_CRQCR_CPL_DATA_CREDITS_MASK			GENMASK(11, 0)
#define PCIE_RC_VC0_CRQCR_CPL_HDR_CREDITS_MASK			GENMASK(19, 12)
#define PCIE_RC_VC0_CRQCR_CPL_TLP_QUEUE_MODE_MASK		GENMASK(23, 21)
#define PCIE_RC_VC0_CRQCR_CPL_TLP_QUEUE_MODE_STORE_FORWARD	0x1
#define PCIE_RC_VC0_CRQCR_CPL_TLP_QUEUE_MODE_CUT_THROUGH	0x2
#define PCIE_RC_VC0_CRQCR_CPL_TLP_QUEUE_MODE_BYPASS		0x4

#define PCIE_RC_PHYCR						0x814

#define RCU_AHB_ENDIAN					0x4c
#define RCU_AHB_ENDIAN_BE_PCIE_M			BIT(0)
#define RCU_AHB_ENDIAN_BE_XBAR_M			BIT(1)
#define RCU_AHB_ENDIAN_BE_PCIE_S			BIT(4)

enum lantiq_pcie_max_payload_size {
	LANTIQ_PCIE_MAX_PAYLOAD_SIZE_128	= 0x0,
	LANTIQ_PCIE_MAX_PAYLOAD_SIZE_256	= 0x1,
	LANTIQ_PCIE_MAX_PAYLOAD_SIZE_512	= 0x2,
	LANTIQ_PCIE_MAX_PAYLOAD_SIZE_1024	= 0x3,
	LANTIQ_PCIE_MAX_PAYLOAD_SIZE_2048	= 0x4,
	LANTIQ_PCIE_MAX_PAYLOAD_SIZE_4096	= 0x5,
};

struct lantiq_pcie {
	struct dw_pcie			pci;
	struct regmap			*app_regmap;
	struct regmap			*rcu_regmap;
	struct clk			*ahb_clk;
	struct clk			*bus_clk;
	struct clk			*pcie_clk;
	struct reset_control		*pcie_reset;
	struct phy			*phy;
	struct gpio_desc		*reset_gpio;
};

#define to_lantiq_pcie(x)		dev_get_drvdata((x)->dev)

static void lantiq_pcie_app_irq_clear(struct dw_pcie *pci)
{
	struct lantiq_pcie *ltq_pcie = to_lantiq_pcie(pci);
	u32 mask = PCIE_APP_IRNCR_AER_REPORT |
		   PCIE_APP_IRNCR_AER_MSIX |
		   PCIE_APP_IRNCR_PME |
		   PCIE_APP_IRNCR_HOTPLUG |
		   PCIE_APP_IRNCR_RX_VDM_MSG |
		   PCIE_APP_IRNCR_RX_CORRECTABLE_ERR_MSG |
		   PCIE_APP_IRNCR_RX_NON_FATAL_ERR_MSG |
		   PCIE_APP_IRNCR_RX_FATAL_ERR_MSG |
		   PCIE_APP_IRNCR_RX_PME_MSG |
		   PCIE_APP_IRNCR_RX_PME_TURNOFF_ACK |
		   PCIE_APP_IRNCR_AHB_BR_FATAL_ERR |
		   PCIE_APP_IRNCR_LINK_AUTO_BW_STATUS |
		   PCIE_APP_IRNCR_BW_MGT;

	regmap_update_bits(ltq_pcie->app_regmap, PCIE_APP_IRNCR, mask, mask);
}

static void lantiq_pcie_app_irq_enable(struct dw_pcie *pci)
{
	struct lantiq_pcie *ltq_pcie = to_lantiq_pcie(pci);
	u32 mask = PCIE_APP_IRNEN_AER_REPORT |
		   PCIE_APP_IRNEN_AER_MSIX |
		   PCIE_APP_IRNEN_PME |
		   PCIE_APP_IRNEN_HOTPLUG |
		   PCIE_APP_IRNEN_RX_VDM_MSG |
		   PCIE_APP_IRNEN_RX_CORRECTABLE_ERR_MSG |
		   PCIE_APP_IRNEN_RX_NON_FATAL_ERR_MSG |
		   PCIE_APP_IRNEN_RX_FATAL_ERR_MSG |
		   PCIE_APP_IRNEN_RX_PME_MSG |
		   PCIE_APP_IRNEN_RX_PME_TURNOFF_ACK |
		   PCIE_APP_IRNEN_AHB_BR_FATAL_ERR |
		   PCIE_APP_IRNEN_LINK_AUTO_BW_STATUS |
		   PCIE_APP_IRNEN_BW_MGT;

	regmap_update_bits(ltq_pcie->app_regmap, PCIE_APP_IRNEN, mask, mask);

	lantiq_pcie_app_irq_clear(pci);
}

static irqreturn_t lantiq_pcie_app_irq_handler(int irq, void *dev_id)
{
	struct dw_pcie *pci = dev_id;

	lantiq_pcie_app_irq_clear(pci);

	return IRQ_HANDLED;
}

static void lantiq_pcie_rcu_init(struct dw_pcie *pci)
{
	struct lantiq_pcie *ltq_pcie = to_lantiq_pcie(pci);

	regmap_update_bits(ltq_pcie->rcu_regmap, RCU_AHB_ENDIAN,
			   RCU_AHB_ENDIAN_BE_PCIE_M,
			   RCU_AHB_ENDIAN_BE_PCIE_M);

	if (of_device_is_big_endian(pci->dev->of_node))
		regmap_update_bits(ltq_pcie->rcu_regmap, RCU_AHB_ENDIAN,
				   RCU_AHB_ENDIAN_BE_PCIE_S,
				   RCU_AHB_ENDIAN_BE_PCIE_S);
	else
		regmap_update_bits(ltq_pcie->rcu_regmap, RCU_AHB_ENDIAN,
				   RCU_AHB_ENDIAN_BE_PCIE_S, 0x0);

	regmap_update_bits(ltq_pcie->rcu_regmap, RCU_AHB_ENDIAN,
			   RCU_AHB_ENDIAN_BE_XBAR_M, 0x0);
}

static int lantiq_pcie_hw_init(struct dw_pcie *pci)
{
	struct lantiq_pcie *ltq_pcie = to_lantiq_pcie(pci);
	int ret;

	ret = clk_prepare_enable(ltq_pcie->ahb_clk);
	if (ret)
		goto err;

	ret = phy_init(ltq_pcie->phy);
	if (ret)
		goto err_disable_ahb_clk;

	gpiod_set_value_cansleep(ltq_pcie->reset_gpio, 1);

	udelay(1);

	ret = reset_control_deassert(ltq_pcie->pcie_reset);
	if (ret)
		goto err_phy_exit;

	ret = phy_power_on(ltq_pcie->phy);
	if (ret)
		goto err_assert_pcie_reset;

	ret = clk_prepare_enable(ltq_pcie->pcie_clk);
	if (ret)
		goto err_phy_power_off;

	ret = clk_prepare_enable(ltq_pcie->bus_clk);
	if (ret)
		goto err_disable_pcie_clk;

	return 0;

err_disable_pcie_clk:
	clk_disable_unprepare(ltq_pcie->pcie_clk);
err_phy_power_off:
	phy_power_off(ltq_pcie->phy);
err_assert_pcie_reset:
	reset_control_assert(ltq_pcie->pcie_reset);
err_phy_exit:
	phy_exit(ltq_pcie->phy);
err_disable_ahb_clk:
	clk_disable_unprepare(ltq_pcie->ahb_clk);
err:
	return ret;
}

static void lantiq_pcie_hw_exit(struct dw_pcie *pci)
{
	struct lantiq_pcie *ltq_pcie = to_lantiq_pcie(pci);

	gpiod_set_value_cansleep(ltq_pcie->reset_gpio, 1);
	clk_disable_unprepare(ltq_pcie->bus_clk);
	clk_disable_unprepare(ltq_pcie->pcie_clk);
	phy_power_off(ltq_pcie->phy);
	reset_control_assert(ltq_pcie->pcie_reset);
	phy_exit(ltq_pcie->phy);
}

static void lantiq_pcie_clear_status_registers(struct dw_pcie *pci)
{
	struct lantiq_pcie *ltq_pcie = to_lantiq_pcie(pci);

	regmap_write(ltq_pcie->app_regmap, PCIE_APP_RC_DR, 0);
	dw_pcie_writel_dbi(pci, PCIE_RC_PCICMDSTS, 0);
	dw_pcie_writel_dbi(pci, PCIE_RC_DCTLSTS, 0);
	dw_pcie_writel_dbi(pci, PCIE_RC_LCTLSTS, 0);
	dw_pcie_writel_dbi(pci, PCIE_RC_SLCTLSTS, 0);
	dw_pcie_writel_dbi(pci, PCIE_RC_RSTS, 0);
	dw_pcie_writel_dbi(pci, PCIE_RC_UES_R, 0);
	dw_pcie_writel_dbi(pci, PCIE_RC_UESR, 0);
	dw_pcie_writel_dbi(pci, PCIE_RC_UESR, 0);
	dw_pcie_writel_dbi(pci, PCIE_RC_CESR, 0);
	dw_pcie_writel_dbi(pci, PCIE_RC_CEMR, 0);
	dw_pcie_writel_dbi(pci, PCIE_RC_RESR, 0);
	dw_pcie_writel_dbi(pci, PCIE_RC_PVCCRSR, 0);
	dw_pcie_writel_dbi(pci, PCIE_RC_VC0_RC, 0);
	dw_pcie_writel_dbi(pci, PCIE_RC_TPFCS, 0);
	dw_pcie_writel_dbi(pci, PCIE_RC_TNPFCS, 0);
	dw_pcie_writel_dbi(pci, PCIE_RC_TCFCS, 0);
	dw_pcie_writel_dbi(pci, PCIE_RC_QSR, 0);
	dw_pcie_writel_dbi(pci, PCIE_RC_IOBLSECS, 0);
}

static void lantiq_pcie_mem_io_setup(struct dw_pcie *pci)
{
	u32 tmp;

	/*
	 * BAR[0:1] readonly register
	 * RC contains only minimal BARs for packets mapped to this device
	 * Mem/IO filters defines a range of memory occupied by memory mapped
	 * IO devices that reside on the downstream side fo the bridge.
	 */
	tmp = 0;
	tmp |= FIELD_PREP(PCIE_RC_MBML_MEM_BASE_ADDR_MASK,
			  (pci->pp.mem_base >> 20));
	tmp |= FIELD_PREP(PCIE_RC_MBML_MEM_LIMIT_ADDR_MASK,
			  ((pci->pp.mem_base + pci->pp.mem_size - 1) >> 20));
	dw_pcie_writel_dbi(pci, PCIE_RC_MBML, tmp);
	dw_pcie_writel_dbi(pci, PCIE_RC_PMBL, tmp);

	tmp = 0;
	tmp |= PCIE_RC_IOBLSECS_32BIT_IO_ADDR;
	tmp |= FIELD_PREP(PCIE_RC_IOBLSECS_IO_BASE_ADDR_MASK,
			  (pci->pp.io_base >> 12));
	tmp |= FIELD_PREP(PCIE_RC_IOBLSECS_IO_LIMIT_ADDR_MASK,
			  ((pci->pp.io_base + pci->pp.io_size - 1) >> 12));
	dw_pcie_writel_dbi(pci, PCIE_RC_IOBLSECS, tmp);

	tmp = 0;
	tmp |= FIELD_PREP(PCIE_RC_IO_BANDL_UPPER_16BIT_IO_BASE_MASK,
			  (pci->pp.io_base >> 16));
	tmp |= FIELD_PREP(PCIE_RC_IO_BANDL_UPPER_16BIT_IO_LIMIT_MASK,
			  ((pci->pp.io_base + pci->pp.io_size - 1) >> 16));
	dw_pcie_writel_dbi(pci, PCIE_RC_IO_BANDL, tmp);
}

static void lantiq_pcie_rc_setup(struct dw_pcie *pci)
{
	u32 tmp;

	/*
	 * XXX, MSI settings should only apply to EP.
	 * MSI Capability: Only enable 32-bit addresses and disable multiple
	 * messages.
	 */
	tmp = dw_pcie_readl_dbi(pci, PCIE_RC_MCAPR);
	tmp |= PCIE_RC_MCAPR_MSI_ENABLE;
	tmp &= ~PCIE_RC_MCAPR_ADDR64_CAP;
	tmp &= ~PCIE_RC_MCAPR_MULTI_MSG_CAP_MASK;
	tmp &= ~PCIE_RC_MCAPR_MULTI_MSG_ENABLE_MASK;
	dw_pcie_writel_dbi(pci, PCIE_RC_MCAPR, tmp);

	/* Enable PME and soft reset */
	tmp = dw_pcie_readl_dbi(pci, PCIE_RC_PM_CSR);
	tmp |= PCIE_RC_PM_CSR_PME_ENABLE;
	tmp |= ~PCIE_RC_PM_CSR_SW_RST;
	dw_pcie_writel_dbi(pci, PCIE_RC_PM_CSR, tmp);

	/* setup the bus */
	tmp = 0;
	tmp |= FIELD_PREP(PCIE_RC_BNR_PRIMARY_BUS_NUM_MASK,
			  pci->pp.root_bus_nr);
	tmp |= FIELD_PREP(PCIE_RC_BNR_SECONDARY_BUS_NUM_MASK,
			  pci->pp.root_bus_nr + 1); // TODO
	tmp |= FIELD_PREP(PCIE_RC_BNR_SUB_BUS_NUM_MASK, 0xff);
	dw_pcie_writel_dbi(pci, PCIE_RC_BNR, tmp);
}

static void lantiq_pcie_device_setup(struct dw_pcie *pci)
{
	u32 tmp;

	/*
	 * set up maximum payload size, disable L0S/L1 latency as these are
	 * only available in EP mode.
	 */
	tmp = dw_pcie_readl_dbi(pci, PCIE_RC_DCAP);
	tmp &= ~PCIE_RC_DCAP_EP_L0S_LATENCY_MASK;
	tmp &= ~PCIE_RC_DCAP_EP_L1_LATENCY_MASK;
	tmp &= ~PCIE_RC_DCAP_MAX_PAYLOAD_SIZE_MASK;
	tmp |= FIELD_PREP(PCIE_RC_DCAP_MAX_PAYLOAD_SIZE_MASK,
			  LANTIQ_PCIE_MAX_PAYLOAD_SIZE_128);
	tmp |= PCIE_RC_DCAP_ROLE_BASE_ERR_REPORT;
	dw_pcie_writel_dbi(pci, PCIE_RC_DCAP, tmp);

	tmp = dw_pcie_readl_dbi(pci, PCIE_RC_DCTLSTS);

	/*
	 * Request size can be larger than the MPS used, but the completions
	 * returned for the read will be bounded by the MPS size.
	 * In our system, max request size depends on AHB burst size. It is 64
	 * bytes but we set it as 128 as minimum one.
	 */
	tmp &= ~PCIE_RC_DCTLSTS_MAX_READ_SIZE_MASK;
	tmp |= FIELD_PREP(PCIE_RC_DCTLSTS_MAX_READ_SIZE_MASK,
			  LANTIQ_PCIE_MAX_PAYLOAD_SIZE_128);
	tmp &= ~PCIE_RC_DCTLSTS_MAX_PAYLOAD_SIZE_MASK;
	tmp |= FIELD_PREP(PCIE_RC_DCTLSTS_MAX_PAYLOAD_SIZE_MASK,
			  LANTIQ_PCIE_MAX_PAYLOAD_SIZE_128);

	/* Enable relaxed ordering, no snoop, and all kinds of errors */
	tmp |= PCIE_RC_DCTLSTS_RELAXED_ORDERING_EN;
	tmp |= PCIE_RC_DCTLSTS_NO_SNOOP_EN;
	tmp |= PCIE_RC_DCTLSTS_CORRECTABLE_ERR_EN;
	tmp |= PCIE_RC_DCTLSTS_NONFATAL_ERR_EN;
	tmp |= PCIE_RC_DCTLSTS_FATAL_ERR_EN;
	tmp |= PCIE_DCTLSYS_UR_REQ_EN;

	dw_pcie_writel_dbi(pci, PCIE_RC_DCTLSTS, tmp);
}

static void lantiq_pcie_link_setup(struct dw_pcie *pci)
{
	u32 tmp;

	/* L0s is reported during link training via TS1 order set by N_FTS */
	tmp = dw_pcie_readl_dbi(pci, PCIE_RC_LCAP);
	tmp &= ~PCIE_RC_LCAP_L0S_EIXT_LATENCY_MASK;
	tmp |= FIELD_PREP(PCIE_RC_LCAP_L0S_EIXT_LATENCY_MASK, 0x3);
	dw_pcie_writel_dbi(pci, PCIE_RC_LCAP, tmp);

	/*
	 * 128 byte read completion boundary may cause multiple split
	 * transactions, use 64 byte RCB instead.
	 */
	tmp = dw_pcie_readl_dbi(pci, PCIE_RC_LCTLSTS);
	tmp &= ~PCIE_RC_LCTLSTS_LINK_DISABLE;
	tmp &= ~PCIE_RC_LCTLSTS_ASPM_ENABLE_MASK;
	tmp &= ~PCIE_RC_LCTLSTS_READ_COMPLETION_BOUNDARY_128;
	dw_pcie_writel_dbi(pci, PCIE_RC_LCTLSTS, tmp);
}

static void lantiq_pcie_error_setup(struct dw_pcie *pci)
{
	u32 tmp;

	/*
	 * Forward ERR_COR, ERR_NONFATAL, ERR_FATAL to the backbone.
	 * Poisoned write TLPs and completions indicating poisoned TLPs will
	 * set the PCIe_PCICMDSTS.MDPE.
	 */

	tmp = dw_pcie_readl_dbi(pci, PCIE_RC_INTRBCTRL);
	tmp |= PCIE_RC_INTRBCTRL_SERR_ENABLE;
	tmp |= PCIE_RC_INTRBCTRL_PARITY_ERR_RESP_ENABLE;
	dw_pcie_writel_dbi(pci, PCIE_RC_INTRBCTRL, tmp);

	/* Unmask (= enable) all uncorrectable error bits */
	tmp = dw_pcie_readl_dbi(pci, PCIE_RC_UEMR);
	tmp &= ~PCIE_RC_UEMR_DATA_LINK_PROTOCOL_ERR;
	tmp &= ~PCIE_RC_UEMR_SURPRISE_DOWN_ERROR;
	tmp &= ~PCIE_RC_UEMR_POISONED_TLP;
	tmp &= ~PCIE_RC_UEMR_FC_PROTOCOL_ERR;
	tmp &= ~PCIE_RC_UEMR_COMPLETION_TIMEOUT;
	tmp &= ~PCIE_RC_UEMR_COMPLETOR_ABORT;
	tmp &= ~PCIE_RC_UEMR_UNEXPECTED_COMPLETION;
	tmp &= ~PCIE_RC_UEMR_RECEIVER_OVERFLOW;
	tmp &= ~PCIE_RC_UEMR_MALFORNED_TLP;
	tmp &= ~PCIE_RC_UEMR_ECRC_ERR;
	tmp &= ~PCIE_RC_UEMR_UR_REQ;
	dw_pcie_writel_dbi(pci, PCIE_RC_UEMR, tmp);

	/* ALL uncorrectable errors are FATAL */
	tmp = dw_pcie_readl_dbi(pci, PCIE_RC_UESR);
	tmp |= PCIE_RC_UESR_DATA_LINK_PROTOCOL_ERR;
	tmp |= PCIE_RC_UESR_SURPRISE_DOWN_ERROR;
	tmp |= PCIE_RC_UESR_POISONED_TLP;
	tmp |= PCIE_RC_UESR_FC_PROTOCOL_ERR;
	tmp |= PCIE_RC_UESR_COMPLETION_TIMEOUT;
	tmp |= PCIE_RC_UESR_COMPLETOR_ABORT;
	tmp |= PCIE_RC_UESR_UNEXPECTED_COMPLETION;
	tmp |= PCIE_RC_UESR_RECEIVER_OVERFLOW;
	tmp |= PCIE_RC_UESR_MALFORNED_TLP;
	tmp |= PCIE_RC_UESR_ECRC_ERR;
	tmp |= PCIE_RC_UESR_UR_REQ;
	dw_pcie_writel_dbi(pci, PCIE_RC_UESR, tmp);

	/* Unmask (= enable) all correctable error bits */
	tmp = dw_pcie_readl_dbi(pci, PCIE_RC_CEMR);
	tmp &= ~PCIE_RC_CESR_RX_ERR;
	tmp &= ~PCIE_RC_CESR_BAD_TLP;
	tmp &= ~PCIE_RC_CESR_BAD_DLLP;
	tmp &= ~PCIE_RC_CESR_REPLAY_NUM_ROLLOVER;
	tmp &= ~PCIE_RC_CESR_REPLAY_TIMER_TIMEOUT_ERR;
	tmp &= ~PCIE_RC_CESR_ADVISORY_NONFTAL_ERR;
	dw_pcie_writel_dbi(pci, PCIE_RC_CEMR, tmp);

	tmp = dw_pcie_readl_dbi(pci, PCIE_RC_AECCR);
	tmp |= PCIE_RC_AECCR_ECRC_GEN_EN;
	tmp |= PCIE_RC_AECCR_ECRC_CHECK_EN;
	dw_pcie_writel_dbi(pci, PCIE_RC_AECCR, tmp);

	/* Report all types of errors */
	tmp = dw_pcie_readl_dbi(pci, PCIE_RC_RECR);
	tmp |= PCIE_RC_RECR_CORRECTABLE_ERR_REPORT_EN;
	tmp |= PCIE_RC_RECR_NONFATAL_ERR_REPORT_EN;
	tmp |= PCIE_RC_RECR_FATAL_ERR_REPORT_EN;
	dw_pcie_writel_dbi(pci, PCIE_RC_RECR, tmp);

	/* Clear the Root status register */
	dw_pcie_writel_dbi(pci, PCIE_RC_RESR,
			   dw_pcie_readl_dbi(pci, PCIE_RC_RESR));
}

static void lantiq_pcie_capabilities_setup(struct dw_pcie *pci)
{
	u32 tmp;

	tmp = dw_pcie_readl_dbi(pci, PCIE_RC_RCTLCAP);
	tmp |= PCIE_RC_RCTLCAP_SERR_ON_CORRECTABLE_ERR;
	tmp |= PCIE_RC_RCTLCAP_SERR_ON_NONFATAL_ERR;
	tmp |= PCIE_RC_RCTLCAP_SERR_ON_FATAL_ERR;
	tmp |= PCIE_RC_RCTLCAP_PME_INT_EN;
	dw_pcie_writel_dbi(pci, PCIE_RC_RCTLCAP, tmp);

	tmp = dw_pcie_readl_dbi(pci, PCIE_RC_PVC2);
	tmp |= PCIE_RC_PVC2_VC_ARB_16P_FIXED_WRR;
	tmp &= ~PCIE_RC_PVC2_VC_ARB_32P_WRR;
	tmp &= ~PCIE_RC_PVC2_VC_ARB_64P_WRR;
	tmp &= ~PCIE_RC_PVC2_VC_ARB_128P_WRR;
	dw_pcie_writel_dbi(pci, PCIE_RC_PVC2, tmp);

	tmp = dw_pcie_readl_dbi(pci, PCIE_RC_VC0_RC);
	tmp &= ~PCIE_RC_VC0_RC_REJECT_SNOOP;
	dw_pcie_writel_dbi(pci, PCIE_RC_VC0_RC, tmp);
}

static void lantiq_pcie_port_logic_setup(struct dw_pcie *pci)
{
	u32 tmp;

	tmp = dw_pcie_readl_dbi(pci, PCIE_RC_AFR);

	/*
	 * FTS number (default 12) increase to 63. This may increase time
	 * from/to L0s to L0.
	 */
	tmp &= ~PCIE_RC_AFR_FTS_NUM_MASK;
	tmp |= FIELD_PREP(PCIE_RC_AFR_FTS_NUM_MASK, 32);
	tmp &= ~PCIE_RC_AFR_COM_FTS_NUM_MASK;
	tmp |= FIELD_PREP(PCIE_RC_AFR_COM_FTS_NUM_MASK, 32);

	/* L0s and L1 entry latency */
	tmp &= ~PCIE_RC_AFR_L0S_ENTRY_LATENCY_MASK;
	tmp |= FIELD_PREP(PCIE_RC_AFR_L0S_ENTRY_LATENCY_MASK, 7);
	tmp &= ~PCIE_RC_AFR_L1_ENTRY_LATENCY_MASK;
	tmp |= FIELD_PREP(PCIE_RC_AFR_L1_ENTRY_LATENCY_MASK, 5);

	dw_pcie_writel_dbi(pci, PCIE_RC_AFR, tmp);

	tmp = dw_pcie_readl_dbi(pci, PCIE_PORT_LINK_CONTROL);
	tmp |= PORT_LINK_DLL_LINK_ENABLE;
	dw_pcie_writel_dbi(pci, PCIE_PORT_LINK_CONTROL, tmp);

	tmp = dw_pcie_readl_dbi(pci, PCIE_RC_LSR);
	tmp |= PCIE_RC_LSR_FC_DISABLE;
	tmp |= PCIE_RC_LSR_ACKNAK_DISABLE;
	dw_pcie_writel_dbi(pci, PCIE_RC_LSR, tmp);

	/*
	 * The default SKP interval is very accurate already, 5us.
	 * Enable IO/CFG transaction and disable FC WDT.
	 */
	tmp = dw_pcie_readl_dbi(pci, PCIE_RC_STRFMR);
	tmp |= PCIE_RC_STRFMR_RX_IO_TRANS_ENABLE;
	tmp |= PCIE_RC_STRFMR_RX_CFG_TRANS_ENABLE;
	tmp &= ~PCIE_RC_STRFMR_FC_WDT_DISABLE;
	dw_pcie_writel_dbi(pci, PCIE_RC_STRFMR, tmp);

	tmp = dw_pcie_readl_dbi(pci, PCIE_RC_FMR2);
	tmp |= PCIE_RC_FMR2_VENDOR_MSG0_PASSED_TO_TRGT1;
	tmp |= PCIE_RC_FMR2_VENDOR_MSG1_PASSED_TO_TRGT1;
	dw_pcie_writel_dbi(pci, PCIE_RC_FMR2, tmp);

	tmp = dw_pcie_readl_dbi(pci, PCIE_RC_VC0_CRQCR);
	tmp &= ~PCIE_RC_VC0_CRQCR_CPL_TLP_QUEUE_MODE_MASK;
	tmp |= FIELD_PREP(PCIE_RC_VC0_CRQCR_CPL_TLP_QUEUE_MODE_MASK,
			  PCIE_RC_VC0_CRQCR_CPL_TLP_QUEUE_MODE_BYPASS);
	dw_pcie_writel_dbi(pci, PCIE_RC_VC0_CRQCR, tmp);
}

static int lantiq_pcie_host_init(struct pcie_port *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct lantiq_pcie *ltq_pcie = to_lantiq_pcie(pci);
	int ret, i;

	lantiq_pcie_rcu_init(pci);

	/*
	 * XXX: PCIe elastic buffer bug will result in the PCIe link not being
	 * detected. One more reset of the PCIe PHY will solve this issue.
	 */
	for (i = 0; i < 5; i++) {
		ret = lantiq_pcie_hw_init(pci);
		if (ret)
			return ret;

		lantiq_pcie_clear_status_registers(pci);

		regmap_write(ltq_pcie->app_regmap, PCIE_APP_RC_CCR, 0);

		lantiq_pcie_mem_io_setup(pci);
		lantiq_pcie_rc_setup(pci);
		lantiq_pcie_device_setup(pci);
		lantiq_pcie_link_setup(pci);
		lantiq_pcie_error_setup(pci);
		lantiq_pcie_capabilities_setup(pci);
		lantiq_pcie_port_logic_setup(pci);
		lantiq_pcie_app_irq_enable(pci);

		regmap_write(ltq_pcie->app_regmap, PCIE_APP_AHB_CTRL,
			PCIE_APP_AHB_CTRL_BUS_ERROR_SUPPRESS);

		dw_pcie_setup_rc(pp);

		gpiod_set_value_cansleep(ltq_pcie->reset_gpio, 0);

		msleep(200);

		/* Start LTSSM training between RC and EP */
		regmap_write(ltq_pcie->app_regmap, PCIE_APP_RC_CCR,
			PCIE_APP_RC_CCR_LTSSM_ENABLE);

		ret = dw_pcie_wait_for_link(pci);
		if (!ret)
			return 0;

		/*  */
		lantiq_pcie_hw_exit(pci);
	}

	return ret;
}

static const struct dw_pcie_host_ops lantiq_dw_pcie_host_ops = {
	.host_init = lantiq_pcie_host_init,
};

static int lantiq_pcie_link_up(struct dw_pcie *pci)
{
	struct lantiq_pcie *ltq_pcie = to_lantiq_pcie(pci);
	u32 tmp;

	tmp = dw_pcie_readl_dbi(pci, PCIE_RC_LCTLSTS);
	if (tmp & PCIE_RC_LCTLSTS_RETRAIN_PENDING)
		return 0;

	regmap_read(ltq_pcie->app_regmap, PCIE_APP_PHY_SR, &tmp);
	if (!(tmp & PCIE_APP_PHY_SR_PHY_LINK_UP))
		return 0;

	/* Check whether the data link is up */
	regmap_read(ltq_pcie->app_regmap, PCIE_APP_RC_DR, &tmp);
	if (!(tmp & PCIE_APP_RC_DR_DLL_UP))
		return 0;

	/* Check whether the data link is active */
	tmp = dw_pcie_readl_dbi(pci, PCIE_RC_LCTLSTS);
	if (tmp & PCIE_RC_LCTLSTS_DLL_ACTIVE)
		return 0;

	return 1;
}

static const struct dw_pcie_ops lantiq_dw_pcie_ops = {
	.link_up = lantiq_pcie_link_up,
};

static int lantiq_pcie_probe(struct platform_device *pdev)
{
	static const struct regmap_config app_regmap_config = {
		.name = "app",
		.reg_bits = 32,
		.val_bits = 32,
		.reg_stride = 4,
		.fast_io = true,
		.max_register = PCIE_APP_IRNICR,
	};
	struct device *dev = &pdev->dev;
	struct lantiq_pcie *ltq_pcie;
	struct resource *res;
	void __iomem *base;
	int ret, irq;

	ltq_pcie = devm_kzalloc(dev, sizeof(*ltq_pcie), GFP_KERNEL);
	if (!ltq_pcie)
		return -ENOMEM;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dbi");
	ltq_pcie->pci.dbi_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(ltq_pcie->pci.dbi_base))
		return PTR_ERR(ltq_pcie->pci.dbi_base);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "app");
	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	ltq_pcie->app_regmap = devm_regmap_init_mmio(&pdev->dev, base,
						     &app_regmap_config);
	if (IS_ERR(ltq_pcie->app_regmap))
		return PTR_ERR(ltq_pcie->app_regmap);

	ltq_pcie->rcu_regmap = syscon_regmap_lookup_by_phandle(dev->of_node,
							       "lantiq,rcu");
	if (IS_ERR(ltq_pcie->rcu_regmap))
		return PTR_ERR(ltq_pcie->rcu_regmap);

	ltq_pcie->pci.pp.irq = platform_get_irq_byname(pdev, "msi");
	if (ltq_pcie->pci.pp.irq < 0)
		return ltq_pcie->pci.pp.irq;

	irq = platform_get_irq_byname(pdev, "app");
	if (irq < 0)
		return irq;

	ret = devm_request_irq(dev, irq, lantiq_pcie_app_irq_handler, 0, NULL,
			       &ltq_pcie->pci);
	if (ret)
		return ret;

	ltq_pcie->ahb_clk = devm_clk_get(dev, "ahb");
	if (IS_ERR(ltq_pcie->ahb_clk))
		return PTR_ERR(ltq_pcie->ahb_clk);

	ltq_pcie->bus_clk = devm_clk_get(dev, "pcie_bus");
	if (IS_ERR(ltq_pcie->bus_clk))
		return PTR_ERR(ltq_pcie->bus_clk);

	ltq_pcie->pcie_clk = devm_clk_get(dev, "pcie");
	if (IS_ERR(ltq_pcie->pcie_clk))
		return PTR_ERR(ltq_pcie->pcie_clk);

	ltq_pcie->pcie_reset = devm_reset_control_get_shared(dev, NULL);
	if (IS_ERR(ltq_pcie->pcie_reset))
		return PTR_ERR(ltq_pcie->pcie_reset);

	ltq_pcie->phy = devm_phy_get(dev, "pcie");
	if (IS_ERR(ltq_pcie->phy))
		return PTR_ERR(ltq_pcie->phy);

	ltq_pcie->reset_gpio = devm_gpiod_get_optional(dev, NULL,
						       GPIOD_OUT_LOW);
	if (IS_ERR(ltq_pcie->reset_gpio))
		return PTR_ERR(ltq_pcie->reset_gpio);

	ltq_pcie->pci.dev = dev;
	ltq_pcie->pci.ops = &lantiq_dw_pcie_ops;
	ltq_pcie->pci.pp.ops = &lantiq_dw_pcie_host_ops;

	platform_set_drvdata(pdev, ltq_pcie);

	ret = dw_pcie_host_init(&ltq_pcie->pci.pp);
	if (ret) {
		dev_err(dev, "Failed to initialize host\n");
		return ret;
	}

	return 0;
}

static const struct of_device_id lantiq_pcie_of_match[] = {
	{ .compatible = "lantiq,xrx200-pcie", },
	{ /* sentinel */ },
};

static struct platform_driver lantiq_pcie_driver = {
	.driver = {
		.name	= "lantiq-pcie",
		.of_match_table = lantiq_pcie_of_match,
		.suppress_bind_attrs = true,
	},
	.probe = lantiq_pcie_probe,
};
builtin_platform_driver(lantiq_pcie_driver);
