/*
 * Amlogic Meson NFC (NAND flash controller) driver
 *
 * Copyright (C) 2017 Amlogic, Inc. All rights reserved.
 * Copyright (C) 2017 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 *
 * Heavily based on oxnas_nand and plat_nand.c:
 * Copyright (C) 2016 Neil Armstrong <narmstrong@baylibre.com>
 * Author: Vitaly Wool <vitalywool@gmail.com>
 * Copyright (C) 2013 Ma Haijun <mahaijuns@gmail.com>
 * Copyright (C) 2012 John Crispin <blogic@openwrt.org>
 * as well as code from Amlogic's GPL kernel sources
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/of.h>

#define MESON_P_NAND_CMD					0x00
	/* CMD register write commands */
	#define MESON_P_NAND_CMD_RESET				BIT(31)
	#define MESON_P_NAND_CMD_CHIP_MASK			GENMASK(13, 10)
	#define MESON_P_NAND_CMD_CHIP0				0xe
	#define MESON_P_NAND_CMD_CHIP1				0xd
	#define MESON_P_NAND_CMD_CHIP2				0xb
	#define MESON_P_NAND_CMD_CHIP3				0x7
	#define MESON_P_NAND_CMD_CHIP_NOT_SEL			0xf
	#define MESON_P_NAND_CMD_STANDBY			GENMASK(13, 10)
	#define MESON_P_NAND_CMD_ECC_SHORT_MODE			BIT(13)
	#define MESON_P_NAND_CMD_DMA_UNIT_SIZE_MASK		GENMASK(12, 6)
	#define MESON_P_NAND_CMD_CTRL_MASK			GENMASK(17, 14)
	#define MESON_P_NAND_CMD_CTRL_CLE			0x5
	#define MESON_P_NAND_CMD_CTRL_ALE			0x6
	#define MESON_P_NAND_CMD_CTRL_DATA_DWR			0x4
	#define MESON_P_NAND_CMD_CTRL_DATA_DRD			0x8
	#define MESON_P_NAND_CMD_CTRL_IDLE			0xc
	/* ECC modes: */
	#define MESON_P_NAND_CMD_ECC_MODE_MASK			GENMASK(17, 14)
	#define MESON_P_NAND_CMD_ECC_MODE_NONE			0x0
	#define MESON_P_NAND_CMD_ECC_MODE_BCH8_512B		0x1
	#define MESON_P_NAND_CMD_ECC_MODE_BCH8_1K		0x2
	#define MESON_P_NAND_CMD_ECC_MODE_BCH24_1K		0x3
	#define MESON_P_NAND_CMD_ECC_MODE_BCH30_1K		0x4
	#define MESON_P_NAND_CMD_ECC_MODE_BCH40_1K		0x5
	#define MESON_P_NAND_CMD_ECC_MODE_BCH50_1K		0x6
	#define MESON_P_NAND_CMD_ECC_MODE_BCH60_1K		0x7
	/* DMA page read/write bit: */
	#define MESON_P_NAND_CMD_DMA_READ			BIT(17)
	/* DMA page read/write "ran mode" bit: */
	#define MESON_P_NAND_CMD_DMA_RAN			BIT(19)
	/* DMA page read/write seed (enable/configure) bit: */
	#define MESON_P_NAND_CMD_DMA_SEED			BIT(20)
	/* DMA transfer bit (not documented, used with N2M_* / M2N_*): */
	#define MESON_P_NAND_CMD_DMA_XFER			BIT(21)
	/* CMD register read commands (different meanings than writing) */
	#define MESON_P_NAND_CMD_INFO_RB_STATUS_MASK		GENMASK(31, 28)
	#define MESON_P_NAND_CMD_INFO_FIFO_CNT_MASK		GENMASK(26, 22) /* TODO: datasheet says 20:24? */

#define NFC_SEND_CMD_M2N_RAW(host, ran, len) \
	NFC_SEND_CMD(host, (ran?M2N:M2N_NORAN)|(len&0x3fff))
#define NFC_SEND_CMD_N2M_RAW(host, ran, len) \
	NFC_SEND_CMD(host, (ran?N2M:N2M_NORAN)|(len&0x3fff))

#define M2N_NORAN	0x00200000 /* BIT(21) */
#define N2M_NORAN	0x00220000 /* BIT(21) | BIT(17) */

#define MESON_P_NAND_CFG					0x04
	#define MESON_P_NAND_CFG_BUS_CYCLE_MASK			GENMASK(4, 0)
	#define MESON_P_NAND_CFG_BUS_TIMING_MASK		GENMASK(9, 5)
	#define MESON_P_NAND_CFG_SYNC_MODE_MASK			GENMASK(11, 10)
	#define MESON_P_NAND_CFG_SYNC_MODE_ASYNC		0x0
	#define MESON_P_NAND_CFG_SYNC_MODE_MICRON_SYNC		0x1
	#define MESON_P_NAND_CFG_SYNC_MODE_TOGGLE		0x2
	#define MESON_P_NAND_CFG_OOB_ON				BIT(26)
	#define MESON_P_NAND_CFG_NEW_OOB_MODE			BIT(27)

#define MESON_P_NAND_DADR					0x08

#define MESON_P_NAND_IADR					0x0c

#define MESON_P_NAND_BUF					0x10

#define MESON_P_NAND_INFO					0x14
	#define MESON_P_NAND_INFO_BYTE0_MASK			GENMASK(7, 0)
	#define MESON_P_NAND_INFO_BYTE1_MASK			GENMASK(15, 8)
	#define MESON_P_NAND_INFO_ZERO_COUNT_MASK		GENMASK(21, 16)
	#define MESON_P_NAND_INFO_PAGE_ERR_CNT_MASK		GENMASK(28, 24)
	#define MESON_P_NAND_INFO_PAGE_UNCORRECTABLE		BIT(29)
	#define MESON_P_NAND_INFO_ECC_ON			BIT(30)
	#define MESON_P_NAND_INFO_DONE				BIT(31)

#define MESON_P_NAND_VER					0x34

#define MESON_NAND_SEED_OFFSET					0xc2

#define MESON_NAND_INFO_NEW_OOB_OFFSET				16
/* magic??? */
#define MESON_NAND_INFO_USER_MODE				2
/* MESON_P_NAND_INFO_BYTE0_MASK and MESON_P_NAND_INFO_BYTE1_MASK: */
// TODO: #define MESON_NAND_INFO_BYTES					2

#define MESON_NAND_SHORT_SIZE					(SZ_128 + SZ_256)

#define MESON_NAND_TWB_TIME_CYCLE				10
#define MESON_NAND_TWHR_TIME_CYCLE				20

#define MESON_NAND_CLK_RATE_MHZ_DEFAULT				212 // TODO: 200 on GXBB
#define MESON_NAND_CLK_RATE_MHZ_MODE5				255 // TODO: 250 on GXBB

#define MESON_NAND_MAX_CHIP_SELECTS				4

#warning "TODO: verify usages of NAND_ECC_NONE vs NAND_ECC_SOFT" // TODO

struct meson_nfc {
	struct nand_chip		*nand_chip;
	void __iomem			*io_base;
	int				selected_chip;
	dma_addr_t			info_dma_addr;
	size_t				info_dma_size;
	u32				*info_dma_buf;
	struct device			*dev;
	struct nand_hw_control		base;
	struct clk			*clk;
};

static const u8 chip_cmd_table[] = {
	[0] = MESON_P_NAND_CMD_CHIP0,
	[1] = MESON_P_NAND_CMD_CHIP1,
	[2] = MESON_P_NAND_CMD_CHIP2,
	[3] = MESON_P_NAND_CMD_CHIP3,
};

static void meson_nand_send_chip_cmd(struct meson_nfc *nfc, u16 cmd, u8 ctrl)
{
	u32 regval;

	if (WARN_ON(nfc->selected_chip >= MESON_NAND_MAX_CHIP_SELECTS))
		return;

	if (WARN_ON(nfc->selected_chip < 0))
		return;

	regval = cmd;
	regval |= FIELD_PREP(MESON_P_NAND_CMD_CTRL_MASK, ctrl);
	regval |= FIELD_PREP(MESON_P_NAND_CMD_CHIP_MASK,
			     chip_cmd_table[nfc->selected_chip]);

	writel(regval, nfc->io_base + MESON_P_NAND_CMD);
}

static u8 meson_nand_get_fifo_count(struct meson_nfc *nfc)
{
	u32 regval = readl(nfc->io_base + MESON_P_NAND_CMD);

	return FIELD_GET(MESON_P_NAND_CMD_INFO_FIFO_CNT_MASK, regval);
}

static void meson_nand_wait_fifo_empty(struct meson_nfc *nfc)
{
	u8 fifo_count;

	do {
		fifo_count = meson_nand_get_fifo_count(nfc);
	} while (fifo_count > 0);
}

static void meson_nand_send_idle(struct meson_nfc *nfc, u16 idle_cycles)
{
	u16 cmd = (idle_cycles & 0x3ff);

	meson_nand_wait_fifo_empty(nfc);

	meson_nand_send_chip_cmd(nfc, cmd, MESON_P_NAND_CMD_CTRL_IDLE);
}

static void meson_nand_wait_data_ready(struct meson_nfc *nfc)
{
	meson_nand_send_idle(nfc, 0);
	meson_nand_send_idle(nfc, 0);

	meson_nand_wait_fifo_empty(nfc);
}

static int meson_nand_dev_ready(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct meson_nfc *nfc = nand_get_controller_data(chip);
	unsigned long timeout = jiffies + msecs_to_jiffies(50);
	u32 regval, ready;

	if (nfc->selected_chip < 0)
		return 0;

	do {
		regval = readl(nfc->io_base + MESON_P_NAND_CMD);

		ready = FIELD_GET(MESON_P_NAND_CMD_INFO_RB_STATUS_MASK,
				  regval) & chip_cmd_table[nfc->selected_chip];

		udelay(2);
	} while (time_before(jiffies, timeout));

	meson_nand_send_idle(nfc, 5);

	return !!ready;
}

static uint8_t meson_nand_read_byte(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct meson_nfc *nfc = nand_get_controller_data(chip);

	meson_nand_send_chip_cmd(nfc, 0, MESON_P_NAND_CMD_CTRL_DATA_DRD);
	meson_nand_send_idle(nfc, MESON_NAND_TWB_TIME_CYCLE);

	meson_nand_wait_data_ready(nfc);

	return readl(nfc->io_base + MESON_P_NAND_BUF);
}

static void meson_nand_write_byte(struct mtd_info *mtd, uint8_t byte)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct meson_nfc *nfc = nand_get_controller_data(chip);

	meson_nand_send_idle(nfc, MESON_NAND_TWB_TIME_CYCLE);
	meson_nand_send_chip_cmd(nfc, byte, MESON_P_NAND_CMD_CTRL_DATA_DWR);
	meson_nand_send_idle(nfc, MESON_NAND_TWB_TIME_CYCLE);

	meson_nand_wait_data_ready(nfc);
}

static void meson_nand_read_buf(struct mtd_info *mtd, uint8_t *buf, int len)
{
	int i;

	for (i = 0; i < len; i++)
		buf[i] = meson_nand_read_byte(mtd);

	print_hex_dump(KERN_INFO, "meson_nand read buf: ", DUMP_PREFIX_NONE, 32, 4, buf, len, 1);
}

static void meson_nand_write_buf(struct mtd_info *mtd, const uint8_t *buf,
				 int len)
{
	int i;

	print_hex_dump(KERN_INFO, "meson_nand write buf: ", DUMP_PREFIX_NONE, 32, 4, buf, len, 1);

	// FIXME
	if (1) {
		printk("%s NOOP!\n", __func__);
		return;
	}

	for (i = 0; i < len; i++)
		meson_nand_write_byte(mtd, buf[i]);
}

static void meson_nand_cmd_ctrl(struct mtd_info *mtd, int cmd,
				unsigned int ctrl)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct meson_nfc *nfc = nand_get_controller_data(chip);

	if (cmd == NAND_CMD_NONE)
		return;

	if (ctrl & NAND_CLE)
		meson_nand_send_chip_cmd(nfc, cmd, MESON_P_NAND_CMD_CTRL_CLE);
	else if (ctrl & NAND_ALE)
		meson_nand_send_chip_cmd(nfc, cmd, MESON_P_NAND_CMD_CTRL_ALE);
	else
		printk("%s: not sending CMD%d (CTRL = %u)\n", __func__, cmd, ctrl);
}

static void meson_nand_select_chip(struct mtd_info *mtd, int ce)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct meson_nfc *nfc = nand_get_controller_data(chip);

	if (WARN_ON(ce >= MESON_NAND_MAX_CHIP_SELECTS)) {
		nfc->selected_chip = -1;
		return;
	}

	nfc->selected_chip = ce;

	if (ce >= 0)
		meson_nand_send_idle(nfc, 0);
}

static u8 meson_nand_get_ecc_mode(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd_to_nand(mtd);

	if (chip->ecc.mode == NAND_ECC_SOFT || chip->ecc.mode == NAND_ECC_NONE)
		return MESON_P_NAND_CMD_ECC_MODE_NONE;

	switch (chip->ecc.strength) {
	case 8:
		if (chip->ecc_step_ds == SZ_512)
			return MESON_P_NAND_CMD_ECC_MODE_BCH8_512B;
		else
			return MESON_P_NAND_CMD_ECC_MODE_BCH8_1K;
	case 24:
		return MESON_P_NAND_CMD_ECC_MODE_BCH24_1K;
	case 30:
		return MESON_P_NAND_CMD_ECC_MODE_BCH30_1K;
	case 40:
		return MESON_P_NAND_CMD_ECC_MODE_BCH40_1K;
	case 50:
		return MESON_P_NAND_CMD_ECC_MODE_BCH50_1K;
	case 60:
		return MESON_P_NAND_CMD_ECC_MODE_BCH60_1K;
	default:
		WARN_ON(1);
		return MESON_P_NAND_CMD_ECC_MODE_NONE;
	}
}

static int meson_nand_is_new_oob_mode(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct meson_nfc *nfc = nand_get_controller_data(chip);
	u32 cfg;

	cfg = readl(nfc->io_base + MESON_P_NAND_CFG);

	return !!(cfg & MESON_P_NAND_CFG_NEW_OOB_MODE);
}

static u16 meson_nand_get_dma_size(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd_to_nand(mtd);

	if (chip->ecc.mode == NAND_ECC_NONE)
		return 1;
	if (chip->ecc.mode == NAND_ECC_SOFT)
		return clamp_val(mtd->writesize - 1, 0x0, 0x3fff);
	else
		return chip->ecc.steps;
}

static u16 meson_nand_get_oob_fill_count(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	u16 oob_fill_count, tmp;

	if (chip->ecc.mode == NAND_ECC_SOFT || chip->ecc.mode == NAND_ECC_NONE)
		oob_fill_count = chip->ecc.steps * MESON_NAND_INFO_USER_MODE;
	else if (chip->ecc_step_ds == SZ_512) {
		oob_fill_count = mtd->writesize + mtd->oobsize;
		oob_fill_count -= SZ_512; // TODO: 512 or MESON_NAND_SHORT_SIZE?
	} else {
		tmp = chip->ecc.bytes + MESON_NAND_INFO_USER_MODE;
		oob_fill_count = mtd->oobsize - (chip->ecc.steps * tmp);
	}

	return oob_fill_count;
}

static size_t meson_nand_get_oob_size(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd_to_nand(mtd);

	if (chip->ecc.mode == NAND_ECC_SOFT || chip->ecc.mode == NAND_ECC_NONE)
		return mtd->oobsize;
	else
		return chip->ecc.steps * MESON_NAND_INFO_USER_MODE;
}

static int meson_nand_wait_dma_read_complete(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct meson_nfc *nfc = nand_get_controller_data(chip);
	unsigned long timeout;
	u32 info_data, info_index;

	if (chip->ecc.mode == NAND_ECC_SOFT || chip->ecc.mode == NAND_ECC_NONE)
		info_index = 0;
	else {
		info_index = meson_nand_get_dma_size(mtd);
		if (meson_nand_is_new_oob_mode(mtd))
			info_index += (MESON_NAND_INFO_NEW_OOB_OFFSET /
					BITS_PER_BYTE);

		info_index = (info_index - 1) * (BITS_PER_BYTE / sizeof(u32));
	}

	timeout = jiffies + msecs_to_jiffies(1000);

	do {
		smp_rmb(); rmb();

		info_data = nfc->info_dma_buf[info_index];

//print_hex_dump(KERN_INFO, "meson_nand info buf: ", DUMP_PREFIX_NONE, 32, 4, nfc->info_dma_buf, nfc->info_dma_size, 1);
//		printk(KERN_INFO "meson nand info@%d: 0x%08x\n", info_index, info_data);

		if (time_after(jiffies, timeout)) {
			dev_warn(nfc->dev, "timeout waiting for DMA read\n");
			return -ETIMEDOUT;
		}
	} while ((info_data & MESON_P_NAND_INFO_DONE) == 0);

	smp_wmb(); wmb();

	return 0;
}

static void meson_nand_submit_dma_xfer(struct mtd_info *mtd, int page,
				       enum dma_data_direction direction)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct meson_nfc *nfc = nand_get_controller_data(chip);
	u32 cmd;

	/* set the page seed for RAN mode: ((8<<16) | (3<<20)) */
	cmd = MESON_P_NAND_CMD_DMA_XFER;
	cmd |= MESON_P_NAND_CMD_DMA_SEED;
	cmd |= MESON_P_NAND_CMD_DMA_RAN;
	cmd |= (MESON_NAND_SEED_OFFSET + (page & 0x7fff));
	writel(cmd, nfc->io_base + MESON_P_NAND_CMD);

	/* configure the actual DMA transfer: */
	cmd = MESON_P_NAND_CMD_DMA_XFER;
	cmd |= FIELD_PREP(MESON_P_NAND_CMD_ECC_MODE_MASK,
			  meson_nand_get_ecc_mode(mtd));
	cmd |= meson_nand_get_dma_size(mtd);

	if (direction == DMA_FROM_DEVICE)
		cmd |= MESON_P_NAND_CMD_DMA_READ;

	if (chip->ecc.mode == NAND_ECC_HW)
		cmd |= MESON_P_NAND_CMD_DMA_RAN;

	if (chip->ecc.mode == NAND_ECC_HW &&
	    chip->ecc_step_ds == MESON_NAND_SHORT_SIZE) {
		cmd |= FIELD_PREP(MESON_P_NAND_CMD_DMA_UNIT_SIZE_MASK,
				  chip->ecc_step_ds >> 3);
		cmd |= MESON_P_NAND_CMD_ECC_SHORT_MODE;
	}

	/* (((1<<17) | (2<<20) | (1<<19))|(ecc_mode<<14)|(sho<<13)|((pgsz&0x7f)<<6)|(pag&0x3f)) */
	writel(cmd, nfc->io_base + MESON_P_NAND_CMD);

	if (direction == DMA_TO_DEVICE) {
		// TODO: different logic for NAND_ECC_NONE?
		cmd = MESON_P_NAND_CMD_DMA_XFER;
		cmd |= meson_nand_get_oob_fill_count(mtd);

		if (chip->ecc.mode == NAND_ECC_HW)
			cmd |= MESON_P_NAND_CMD_DMA_RAN;

		writel(cmd, nfc->io_base + MESON_P_NAND_CMD);
	}
}

static int meson_nand_dma_xfer_page(struct mtd_info *mtd, const uint8_t *buf,
				    int page,
				    enum dma_data_direction direction)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct meson_nfc *nfc = nand_get_controller_data(chip);
	dma_addr_t data_dma_addr;
	size_t data_dma_size;
	int err;

//printk("%s: %s page %d\n", __func__, direction == DMA_FROM_DEVICE ? "read" : "write", page);

	/* clear the info buffer before reading data */
	if (direction == DMA_FROM_DEVICE)
		memset(nfc->info_dma_buf, 0, nfc->info_dma_size);

	data_dma_size = mtd->writesize + mtd->oobsize;
	data_dma_addr = dma_map_single(nfc->dev, (uint8_t *)buf, data_dma_size,
				       direction);
	err = dma_mapping_error(nfc->dev, data_dma_addr);
	if (err) {
		dev_err(nfc->dev, "failed to map DMA buffer\n");
		return err;
	}

	smp_wmb(); wmb();

	/* data and info DMA addresses need to be set before *each* transfer */
	writel(data_dma_addr, nfc->io_base + MESON_P_NAND_DADR);
	writel(nfc->info_dma_addr, nfc->io_base + MESON_P_NAND_IADR);

	meson_nand_submit_dma_xfer(mtd, page, direction);

	meson_nand_wait_data_ready(nfc);

	if (direction == DMA_FROM_DEVICE)
		err = meson_nand_wait_dma_read_complete(mtd);
	else
		err = 0;

	dma_unmap_single(nfc->dev, data_dma_addr, data_dma_size, direction);

	return err;
}

static int meson_nand_read_page(struct mtd_info *mtd, struct nand_chip *chip,
				uint8_t *buf, int oob_required, int page)
{
	struct meson_nfc *nfc = nand_get_controller_data(chip);
	size_t oob_size = meson_nand_get_oob_size(mtd);
	uint8_t *oob_buf = buf + mtd->writesize;
	u32 ecc_step, info_index, info_data, bitflips, max_bitflips = 0;
	int i, err, print_err = 0;

	err = meson_nand_dma_xfer_page(mtd, buf, page, DMA_FROM_DEVICE);
	if (err < 0)
		return err;

	if (meson_nand_is_new_oob_mode(mtd))
		memcpy(oob_buf, nfc->info_dma_buf, oob_size);
	else {
		for (i = 0; i < oob_size; i += 2) {
			info_index = i / 2 * BITS_PER_BYTE / sizeof(u32);

			oob_buf[i] =
				FIELD_GET(MESON_P_NAND_INFO_BYTE0_MASK,
					  nfc->info_dma_buf[info_index]);
			oob_buf[i + 1] =
				FIELD_GET(MESON_P_NAND_INFO_BYTE1_MASK,
					  nfc->info_dma_buf[info_index]);
		}
	}

	for (ecc_step = 0; ecc_step < chip->ecc.steps; ecc_step++) {
		info_index = (ecc_step * BITS_PER_BYTE / sizeof(u32));
		if (meson_nand_is_new_oob_mode(mtd))
			info_index += 4;

		info_data = nfc->info_dma_buf[info_index];

		bitflips = FIELD_GET(MESON_P_NAND_INFO_PAGE_ERR_CNT_MASK,
				     info_data);

		if (info_data & MESON_P_NAND_INFO_PAGE_UNCORRECTABLE &&
		    bitflips == 0x1f) {
			printk("dma read info for page %d step %d = %d (0x%08x)\n", page, ecc_step, bitflips, info_data);
			print_err = page < 100;

#if 0
			if (FIELD_GET(MESON_P_NAND_INFO_ZERO_COUNT_MASK,
				      info_data) < chip->ecc.strength) {
				// TODO: verify this:
				memset(buf, 0xff, mtd->writesize);
				memset(chip->oob_poi, 0xff, mtd->oobsize);
			} else
				mtd->ecc_stats.failed++;
#else
			mtd->ecc_stats.failed++;
#endif
			break;
		} else {
			mtd->ecc_stats.corrected += bitflips;
			max_bitflips = max(bitflips, max_bitflips);
		}
	}

	if (print_err)
		print_hex_dump(KERN_INFO, "meson_nand info buf: ", DUMP_PREFIX_NONE, 32, 4, nfc->info_dma_buf, nfc->info_dma_size, 1);

	return max_bitflips;
}

static int meson_nand_write_page(struct mtd_info *mtd, struct nand_chip *chip,
				 const uint8_t *buf, int oob_required,
				 int page)
{
	// FIXME:
	if (1) {
		printk("%s NOOP!\n", __func__);
		return 0;
	}

	return meson_nand_dma_xfer_page(mtd, buf, page, DMA_TO_DEVICE);
}

static int meson_nand_read_oob(struct mtd_info *mtd, struct nand_chip *chip,
			       int page)
{
	chip->cmdfunc(mtd, NAND_CMD_READ0, 0x0, page);

	/* invalidate the cached page buffer */
	chip->pagebuf = -1;

	return chip->ecc.read_page(mtd, chip, chip->buffers->databuf, 1, page);

#if 0
	struct meson_nfc *nfc = nand_get_controller_data(chip);
	size_t oob_size = meson_nand_get_oob_size(mtd);
	int i, info_index, err;

	// NOTE: moved to read_page for now
	if (meson_nand_is_new_oob_mode(mtd))
		memcpy(chip->oob_poi, nfc->info_dma_buf, oob_size);
	else {
		for (i = 0; i < oob_size; i += 2) {
			info_index = i / 2 * BITS_PER_BYTE / sizeof(u32);

			chip->oob_poi[i] =
				FIELD_GET(MESON_P_NAND_INFO_BYTE0_MASK,
					  nfc->info_dma_buf[info_index]);
			chip->oob_poi[i + 1] =
				FIELD_GET(MESON_P_NAND_INFO_BYTE1_MASK,
					  nfc->info_dma_buf[info_index]);
		}
	}

	// FIXME: why? memset(chip->oob_poi + oob_size, 0, mtd->oobsize - oob_size);

	//print_hex_dump(KERN_INFO, "meson_nand user buf: ", DUMP_PREFIX_NONE, 32, 1, nfc->info_dma_buf, nfc->info_dma_size, 1);

	return 0;
#endif
}

static int meson_nand_write_oob(struct mtd_info *mtd, struct nand_chip *chip,
				int page)
{
	struct meson_nfc *nfc = nand_get_controller_data(chip);
	size_t oob_size = meson_nand_get_oob_size(mtd);
	int i, info_index, info_data, err;

	// FIXME:
	if (1) {
		printk("%s NOOP!\n", __func__);
		return 0;
	}

	chip->cmdfunc(mtd, NAND_CMD_SEQIN, 0x0, page);

	/* invalidate the cached page buffer */
	chip->pagebuf = -1;

	memset(chip->buffers->databuf, 0xff, mtd->writesize);

	err = chip->ecc.write_page(mtd, chip, chip->buffers->databuf, 1, page);
	if (err < 0)
		return err;

	/* Send command to program the OOB data */
	chip->cmdfunc(mtd, NAND_CMD_PAGEPROG, -1, -1);

	err = chip->waitfunc(mtd, chip);
	if (err & NAND_STATUS_FAIL)
		return -EIO;

	meson_nand_send_idle(nfc, MESON_NAND_TWB_TIME_CYCLE);

	if (meson_nand_is_new_oob_mode(mtd))
		memcpy(nfc->info_dma_buf, chip->oob_poi, oob_size);
	else {
		for (i = 0; i < oob_size; i += 2) {
			info_index = i / 2 * BITS_PER_BYTE / sizeof(u32);

			info_data = FIELD_PREP(MESON_P_NAND_INFO_BYTE0_MASK,
					       chip->oob_poi[i]);
			info_data |= FIELD_PREP(MESON_P_NAND_INFO_BYTE1_MASK,
						chip->oob_poi[i + 1]);

			nfc->info_dma_buf[info_index] = info_data;
		}
	}

	return 0;
}

static int meson_nand_oob_ecc(struct mtd_info *mtd, int section,
			      struct mtd_oob_region *oobecc)
{
	if (section)
		return -ERANGE;

	switch (mtd->oobsize) {
	case 64:
		oobecc->length = 56;
		oobecc->offset = 0;
		break;

	case 128:
		oobecc->length = 120;
		oobecc->offset = 0;
		break;

	case 218:
		oobecc->length = 200;
		oobecc->offset = 0;
		break;

	case 224:
		oobecc->length = 208;
		oobecc->offset = 0;
		break;

	case 256:
		oobecc->length = 240;
		oobecc->offset = 0;
		break;

	case 376:
		/* fall-through: */
	case 436:
		oobecc->length = 352;
		oobecc->offset = 0;
		break;

	case 448:
		oobecc->length = 416;
		oobecc->offset = 0;
		break;

	case 640:
		oobecc->length = 608;
		oobecc->offset = 0;
		break;

	case 744:
		oobecc->length = 700;
		oobecc->offset = 0;
		break;

	case 1280:
		oobecc->length = 1200;
		oobecc->offset = 0;
		break;

	case 1664:
		oobecc->length = 1584;
		oobecc->offset = 0;
		break;

	default:
		dev_err(&mtd->dev, "unsupported OOB size %u\n", mtd->oobsize);
		return -EINVAL;
	}

	return 0;
}

static int meson_nand_oob_free(struct mtd_info *mtd, int section,
			       struct mtd_oob_region *oobfree)
{
	if (section)
		return -ERANGE;

	switch (mtd->oobsize) {
	case 64:
	case 128:
	case 218:
	case 224:
		oobfree->length = 8;
		oobfree->offset = 0;
		break;

	case 256:
	case 376:
	case 436:
	case 448:
	case 640:
	case 744:
		oobfree->length = 16;
		oobfree->offset = 0;
		break;

	case 1280:
	case 1664:
		oobfree->length = 32;
		oobfree->offset = 0;
		break;

	default:
		dev_err(&mtd->dev, "unsupported OOB size %u\n", mtd->oobsize);
		return -EINVAL;
	}

	return 0;
}

static const struct mtd_ooblayout_ops meson_nand_ooblayout_ops = {
	.ecc	= meson_nand_oob_ecc,
	.free	= meson_nand_oob_free,
};

static int meson_nand_apply_timings(struct mtd_info *mtd, int chipnr,
				    const struct nand_data_interface *conf)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct meson_nfc *nfc = nand_get_controller_data(chip);
	u32 cfg;
	unsigned long rate_mhz;
	int err;

	/* clear any existing configuration */
	cfg = 0;
	writel(cfg, nfc->io_base + MESON_P_NAND_CFG);

	/* set bus cycle and timing like the vendor driver */
	cfg |= FIELD_PREP(MESON_P_NAND_CFG_BUS_CYCLE_MASK, 5);
	cfg |= FIELD_PREP(MESON_P_NAND_CFG_BUS_TIMING_MASK, 8);
	cfg |= FIELD_PREP(MESON_P_NAND_CFG_SYNC_MODE_MASK,
			  MESON_P_NAND_CFG_SYNC_MODE_ASYNC);

	if (mtd->writesize > SZ_4K) {
		cfg |= MESON_P_NAND_CFG_OOB_ON;
#if 0
		// FIXME: un-setting this allows us to read 3x16k
		// of the bootloader without corruption (when it's set
		// then we can only read 1x16k without corruption).
		cfg |= MESON_P_NAND_CFG_NEW_OOB_MODE;
#endif
	}

	writel(cfg, nfc->io_base + MESON_P_NAND_CFG);

	/* trigger a reset of the NAND core */
	writel(MESON_P_NAND_CMD_RESET, nfc->io_base + MESON_P_NAND_CMD);

	if (conf->timings.sdr.tREA_max > 16000)
		rate_mhz = MESON_NAND_CLK_RATE_MHZ_DEFAULT;
	else
		rate_mhz = MESON_NAND_CLK_RATE_MHZ_MODE5;

	err = 0; // FIXME HACK: clk_set_rate(nfc->clk, rate_mhz * 1000 * 1000);
	if (err) {
		dev_err(nfc->dev, "Failed to setup NAND clock for chip %d\n",
			nfc->selected_chip);
		return err;
	}

	return 0;
}

static int meson_nand_enable_hw(struct platform_device *pdev)
{
	struct meson_nfc *nfc = platform_get_drvdata(pdev);

	return clk_prepare_enable(nfc->clk);
}

static void meson_nand_disable_hw(struct platform_device *pdev)
{
	struct meson_nfc *nfc = platform_get_drvdata(pdev);

	clk_disable_unprepare(nfc->clk);
}

static int meson_nand_validate_ecc_mode(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct meson_nfc *nfc = nand_get_controller_data(chip);

	if (chip->ecc.mode == NAND_ECC_NONE || chip->ecc.mode == NAND_ECC_SOFT)
		return 0;

	if (chip->ecc.algo != NAND_ECC_BCH) {
		dev_err(nfc->dev, "hardware ECC only supports the BCH algo\n");
		return -EINVAL;
	}

	if (!chip->ecc.size || !chip->ecc.strength) {
		dev_err(nfc->dev,
			"unsupported strength or size for hardware ECC\n");
		return -EINVAL;
	}

	switch (chip->ecc.strength) {
	case 8:
	case 24:
	case 30:
	case 40:
	case 50:
	case 60:
		break;

	default:
		dev_err(nfc->dev, "unsupported hardware ECC strength: %d\n",
			chip->ecc.strength);
		return -EINVAL;
	}

	return 0;
}

static int meson_nand_dma_init(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct meson_nfc *nfc = nand_get_controller_data(chip);

	nfc->info_dma_size = (mtd->writesize / chip->ecc_step_ds) * BITS_PER_BYTE;
	nfc->info_dma_size += MESON_NAND_INFO_NEW_OOB_OFFSET;
	nfc->info_dma_buf = dma_alloc_coherent(nfc->dev, nfc->info_dma_size,
					       &nfc->info_dma_addr,
					       GFP_KERNEL);
	if (!nfc->info_dma_buf)
		return -ENOMEM;

	return 0;
}

static void meson_nand_dma_free(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct meson_nfc *nfc = nand_get_controller_data(chip);

	dma_free_coherent(nfc->dev, nfc->info_dma_size, nfc->info_dma_buf,
			  nfc->info_dma_addr);
}

static int meson_nand_init_chips(struct meson_nfc *nfc)
{
	struct device_node *np = nfc->dev->of_node;
	struct nand_chip *chip;
	struct mtd_info *mtd;
	int err;

	if (of_get_available_child_count(np) != 1)
		return -EINVAL;

	chip = devm_kzalloc(nfc->dev, sizeof(struct nand_chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->controller = &nfc->base;

	nand_set_flash_node(chip, of_get_next_available_child(np, NULL));
	nand_set_controller_data(chip, nfc);

	mtd = nand_to_mtd(chip);
	mtd->dev.parent = nfc->dev;
	mtd->priv = chip;

	chip->dev_ready = meson_nand_dev_ready;
	chip->read_byte = meson_nand_read_byte;
	chip->write_byte = meson_nand_write_byte;
	chip->read_buf = meson_nand_read_buf;
	chip->write_buf = meson_nand_write_buf;
	chip->cmd_ctrl = meson_nand_cmd_ctrl;
	chip->select_chip = meson_nand_select_chip;
	chip->setup_data_interface = meson_nand_apply_timings;
	chip->chip_delay = 100;
	chip->ecc.read_page = meson_nand_read_page;
	chip->ecc.write_page = meson_nand_write_page;
	chip->ecc.read_oob = meson_nand_read_oob;
	chip->ecc.write_oob = meson_nand_write_oob;
	chip->options = NAND_NO_SUBPAGE_WRITE | NAND_SKIP_BBTSCAN;

	mtd_set_ooblayout(mtd, &meson_nand_ooblayout_ops);

	/* Scan to find existence of the device */
	err = nand_scan_ident(mtd, MESON_NAND_MAX_CHIP_SELECTS, NULL);
	if (err)
		return err;

	err = meson_nand_validate_ecc_mode(mtd);
	if (err)
		return err;

	err = meson_nand_dma_init(mtd);
	if (err)
		return err;

	err = nand_scan_tail(mtd);
	if (err)
		goto err_cleanup_dma;

	err = mtd_device_register(mtd, NULL, 0);
	if (err) {
		dev_err(nfc->dev, "failed to register mtd device: %d\n", err);
		goto err_release_nand;
	}

	nfc->nand_chip = chip;

	return 0;

err_release_nand:
	nand_release(mtd);
err_cleanup_dma:
	meson_nand_dma_free(mtd);

	return err;
}

static int meson_nand_probe(struct platform_device *pdev)
{
	struct meson_nfc *nfc;
	struct resource *res;
	int err;

	err = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	if (err) {
		dev_err(&pdev->dev, "No suitable DMA mask available\n");
		return err;
	}

	nfc = devm_kzalloc(&pdev->dev, sizeof(struct nand_chip), GFP_KERNEL);
	if (!nfc)
		return -ENOMEM;

	nand_hw_control_init(&nfc->base);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	nfc->io_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(nfc->io_base))
		return PTR_ERR(nfc->io_base);

	nfc->clk = devm_clk_get(&pdev->dev, "nand");
	if (IS_ERR(nfc->clk))
		return PTR_ERR(nfc->clk);

	nfc->dev = &pdev->dev;
	nfc->selected_chip = -1;

	platform_set_drvdata(pdev, nfc);

	err = meson_nand_enable_hw(pdev);
	if (err)
		return err;

	dev_info(&pdev->dev, "core version 0x%08x\n",
		 readl(nfc->io_base + MESON_P_NAND_VER));

	err = meson_nand_init_chips(nfc);
	if (err) {
		meson_nand_disable_hw(pdev);
		return err;
	}

	return 0;
}

static int meson_nand_remove(struct platform_device *pdev)
{
	struct meson_nfc *nfc = platform_get_drvdata(pdev);
	struct mtd_info *mtd = nand_to_mtd(nfc->nand_chip);

	nand_release(mtd);
	meson_nand_disable_hw(pdev);
	meson_nand_dma_free(mtd);

	return 0;
}

static const struct of_device_id meson_nand_match[] = {
	{ .compatible = "amlogic,meson8-nand" },
	{ .compatible = "amlogic,meson8b-nand" },
	{},
};
MODULE_DEVICE_TABLE(of, meson_nand_match);

static struct platform_driver meson_nand_driver = {
	.probe	= meson_nand_probe,
	.remove	= meson_nand_remove,
	.driver	= {
		.name		= "meson_nand",
		.of_match_table	= meson_nand_match,
	},
};

module_platform_driver(meson_nand_driver);

MODULE_LICENSE("GPLv2");
MODULE_AUTHOR("Martin Blumenstingl <martin.blumenstingl@googlemail.com>");
MODULE_DESCRIPTION("Amlogic Meson NAND driver");
