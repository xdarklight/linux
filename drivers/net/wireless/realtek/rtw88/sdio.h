/* SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause */
/* Copyright (C) 2020 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 */

#ifndef __REG_SDIO_H_
#define __REG_SDIO_H_

#include "mac.h"

/* I/O bus domain address mapping */
#define WLAN_IOREG_OFFSET			0x10260000
#define FIRMWARE_FIFO_OFFSET			0x10270000
#define TX_HIQ_OFFSET				0x10310000
#define TX_MIQ_OFFSET				0x10320000
#define TX_LOQ_OFFSET				0x10330000
#define TX_EPQ_OFFSET				0x10350000
#define RX_RX0FF_OFFSET				0x10340000

#define RTW_SDIO_BUS_MSK			0xffff0000
#define SDIO_LOCAL_REG_MSK			0x0000ffff
#define WLAN_IOREG_REG_MSK			0x00000fff

/* Sdio Address for SDIO Local Reg, TRX FIFO, MAC Reg */
#define REG_SDIO_CMD_ADDR_MSK			GENMASK(16, 13)
#define REG_SDIO_CMD_ADDR_SDIO_REG 		0
#define REG_SDIO_CMD_ADDR_MAC_REG		8
#define REG_SDIO_CMD_ADDR_TXFF_HIGH		4
#define REG_SDIO_CMD_ADDR_TXFF_LOW		6
#define REG_SDIO_CMD_ADDR_TXFF_NORMAL		5
#define REG_SDIO_CMD_ADDR_TXFF_EXTRA		7
#define REG_SDIO_CMD_ADDR_RXFF			7

#define REG_SDIO_ADDR_CMD52_BIT			BIT(17)
#define REG_SDIO_ADDR_F0_BIT			BIT(18)

#define RTW_SDIO_BLOCK_SIZE			512
#define RTW_SDIO_ADDR_RX_RX0FF_GEN(_id)		(0x0e000 | ((_id) & 0x3))

struct sdio_func;

struct rtw_sdio {
	struct sdio_func *sdio_func;

	/* Used for PCI TX queueing. */
	spinlock_t irq_lock;

	u32 irq_mask;
	u8 rx_addr;
	bool sdio3_bus_mode;

	bool is_powered_on;
};

#endif
