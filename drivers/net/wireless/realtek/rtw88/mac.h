/* SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause */
/* Copyright(c) 2018-2019  Realtek Corporation
 */

#ifndef __RTW_MAC_H__
#define __RTW_MAC_H__

#define RTW_HW_PORT_NUM		5
#define cut_version_to_mask(cut) (0x1 << ((cut) + 1))

#define SDIO_LOCAL_OFFSET	0x10250000
/* HCI Current Power Mode */
#define REG_SDIO_HCPWM		(SDIO_LOCAL_OFFSET + 0x0019)
/* RXDMA Request Length */
#define REG_SDIO_RX0_REQ_LEN	(SDIO_LOCAL_OFFSET + 0x001C)
/* OQT Free Page */
#define REG_SDIO_OQT_FREE_PG	(SDIO_LOCAL_OFFSET + 0x001E)
/* Free Tx Buffer Page */
#define REG_SDIO_FREE_TXPG	(SDIO_LOCAL_OFFSET + 0x0020)
/* HCI Current Power Mode 1 */
#define REG_SDIO_HCPWM1		(SDIO_LOCAL_OFFSET + 0x0024)
/* HCI Current Power Mode 2 */
#define REG_SDIO_HCPWM2		(SDIO_LOCAL_OFFSET + 0x0026)
/* Free Tx Page Sequence */
#define REG_SDIO_FREE_TXPG_SEQ	(SDIO_LOCAL_OFFSET + 0x0028)
/* HTSF Informaion */
#define REG_SDIO_HTSFR_INFO	(SDIO_LOCAL_OFFSET + 0x0030)
/* H2C */
#define REG_SDIO_H2C		(SDIO_LOCAL_OFFSET + 0x0060)
/* HCI Request Power Mode 1 */
#define REG_SDIO_HRPWM1		(SDIO_LOCAL_OFFSET + 0x0080)
/* HCI Request Power Mode 2 */
#define REG_SDIO_HRPWM2		(SDIO_LOCAL_OFFSET + 0x0082)
/* HCI Power Save Clock */
#define REG_SDIO_HPS_CLKR	(SDIO_LOCAL_OFFSET + 0x0084)
/* SDIO HCI Suspend Control */
#define REG_SDIO_HSUS_CTRL	(SDIO_LOCAL_OFFSET + 0x0086)
/* SDIO Host Extension Interrupt Mask Always */
#define REG_SDIO_HIMR_ON	(SDIO_LOCAL_OFFSET + 0x0090)
/* SDIO Host Extension Interrupt Status Always */
#define REG_SDIO_HISR_ON	(SDIO_LOCAL_OFFSET + 0x0091)
#define REG_SDIO_HCPWM1_V2	(SDIO_LOCAL_OFFSET + 0x0038)

/* SDIO Tx Control */
#define REG_SDIO_TX_CTRL	(SDIO_LOCAL_OFFSET + 0x0000)
/*SDIO status timeout*/
#define REG_SDIO_TIMEOUT	(SDIO_LOCAL_OFFSET + 0x0002)

/* SDIO Host Interrupt Mask */
#define REG_SDIO_HIMR				(SDIO_LOCAL_OFFSET + 0x0014)
#define REG_SDIO_HIMR_RX_REQUEST		BIT(0)
#define REG_SDIO_HIMR_AVAL			BIT(1)
#define REG_SDIO_HIMR_TXERR			BIT(2)
#define REG_SDIO_HIMR_RXERR			BIT(3)
#define REG_SDIO_HIMR_TXFOVW			BIT(4)
#define REG_SDIO_HIMR_RXFOVW			BIT(5)
#define REG_SDIO_HIMR_TXBCNOK			BIT(6)
#define REG_SDIO_HIMR_TXBCNERR			BIT(7)
#define REG_SDIO_HIMR_BCNERLY_INT		BIT(16)
#define REG_SDIO_HIMR_C2HCMD			BIT(17)
#define REG_SDIO_HIMR_CPWM1			BIT(18)
#define REG_SDIO_HIMR_CPWM2			BIT(19)
#define REG_SDIO_HIMR_HSISR_IND			BIT(20)
#define REG_SDIO_HIMR_GTINT3_IND		BIT(21)
#define REG_SDIO_HIMR_GTINT4_IND		BIT(22)
#define REG_SDIO_HIMR_PSTIMEOUT			BIT(23)
#define REG_SDIO_HIMR_OCPINT			BIT(24)
#define REG_SDIO_HIMR_ATIMEND			BIT(25)
#define REG_SDIO_HIMR_ATIMEND_E			BIT(26)
#define REG_SDIO_HIMR_CTWEND			BIT(27)
/* the following two are RTL8188 SDIO Specific */
#define REG_SDIO_HIMR_MCU_ERR			BIT(28)
#define REG_SDIO_HIMR_TSF_BIT32_TOGGLE		BIT(29)

/* SDIO Host Interrupt Service Routine */
#define REG_SDIO_HISR				(SDIO_LOCAL_OFFSET + 0x0018)
#define REG_SDIO_HISR_RX_REQUEST		BIT(0)
#define REG_SDIO_HISR_AVAL			BIT(1)
#define REG_SDIO_HISR_TXERR			BIT(2)
#define REG_SDIO_HISR_RXERR			BIT(3)
#define REG_SDIO_HISR_TXFOVW			BIT(4)
#define REG_SDIO_HISR_RXFOVW			BIT(5)
#define REG_SDIO_HISR_TXBCNOK			BIT(6)
#define REG_SDIO_HISR_TXBCNERR			BIT(7)
#define REG_SDIO_HISR_BCNERLY_INT		BIT(16)
#define REG_SDIO_HISR_C2HCMD			BIT(17)
#define REG_SDIO_HISR_CPWM1			BIT(18)
#define REG_SDIO_HISR_CPWM2			BIT(19)
#define REG_SDIO_HISR_HSISR_IND			BIT(20)
#define REG_SDIO_HISR_GTINT3_IND		BIT(21)
#define REG_SDIO_HISR_GTINT4_IND		BIT(22)
#define REG_SDIO_HISR_PSTIMEOUT			BIT(23)
#define REG_SDIO_HISR_OCPINT			BIT(24)
#define REG_SDIO_HISR_ATIMEND			BIT(25)
#define REG_SDIO_HISR_ATIMEND_E			BIT(26)
#define REG_SDIO_HISR_CTWEND			BIT(27)
/* the following two are RTL8188 SDIO Specific */
#define REG_SDIO_HISR_MCU_ERR			BIT(28)
#define REG_SDIO_HISR_TSF_BIT32_TOGGLE		BIT(29)

#define REG_SDIO_INDIRECT_REG_CFG		(SDIO_LOCAL_OFFSET + 0x0040)
#define REG_SDIO_INDIRECT_REG_DATA		(SDIO_LOCAL_OFFSET + 0x0044)

#define DDMA_POLLING_COUNT	1000
#define C2H_PKT_BUF		256
#define REPORT_BUF		128
#define PHY_STATUS_SIZE		4
#define ILLEGAL_KEY_GROUP	0xFAAAAA00

/* HW memory address */
#define OCPBASE_TXBUF_88XX		0x18780000
#define OCPBASE_DMEM_88XX		0x00200000
#define OCPBASE_EMEM_88XX		0x00100000

#define RSVD_PG_DRV_NUM			16
#define RSVD_PG_H2C_EXTRAINFO_NUM	24
#define RSVD_PG_H2C_STATICINFO_NUM	8
#define RSVD_PG_H2CQ_NUM		8
#define RSVD_PG_CPU_INSTRUCTION_NUM	0
#define RSVD_PG_FW_TXBUF_NUM		4

void rtw_set_channel_mac(struct rtw_dev *rtwdev, u8 channel, u8 bw,
			 u8 primary_ch_idx);
int rtw_mac_power_on(struct rtw_dev *rtwdev);
void rtw_mac_power_off(struct rtw_dev *rtwdev);
int rtw_download_firmware(struct rtw_dev *rtwdev, struct rtw_fw_state *fw);
int rtw_mac_init(struct rtw_dev *rtwdev);
void rtw_mac_flush_queues(struct rtw_dev *rtwdev, u32 queues, bool drop);

static inline void rtw_mac_flush_all_queues(struct rtw_dev *rtwdev, bool drop)
{
	rtw_mac_flush_queues(rtwdev, BIT(rtwdev->hw->queues) - 1, drop);
}

#endif
