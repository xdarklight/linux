/*
 * Amlogic Meson NFC (NAND flash controller) driver
 *
 * Copyright (C) 2017 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 *
 * Heavily based on oxnas_nand and plat_nand.c:
 * Copyright (C) 2016 Neil Armstrong <narmstrong@baylibre.com>
 * Author: Vitaly Wool <vitalywool@gmail.com>
 * Copyright (C) 2013 Ma Haijun <mahaijuns@gmail.com>
 * Copyright (C) 2012 John Crispin <blogic@openwrt.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
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
	#define MESON_P_NAND_CMD_CTRL_MASK			GENMASK(17, 14)
	#define MESON_P_NAND_CMD_CTRL_CLE			0x5
	#define MESON_P_NAND_CMD_CTRL_ALE			0x6
	#define MESON_P_NAND_CMD_CTRL_DATA_DWR			0x4
	#define MESON_P_NAND_CMD_CTRL_DATA_DRD			0x8
	#define MESON_P_NAND_CMD_CTRL_IDLE			0xc
	/* unset to write: */
	#define MESON_P_NAND_CMD_DMA_READ			BIT(17)
	#define MESON_P_NAND_CMD_DMA_RAN			BIT(19)
	#define MESON_P_NAND_CMD_DMA_SEED			BIT(20)
	#define MESON_P_NAND_CMD_DMA_XFER			BIT(21) /* not documented, always used in DMA mode with RAN / NO_RAN */
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
	#define MESON_P_NAND_INFO_PAGE_ERR_CNT_MASK		GENMASK(28, 24)
	#define MESON_P_NAND_INFO_PAGE_UNCORRECTABLE		BIT(29)
	#define MESON_P_NAND_INFO_ECC_ON			BIT(30)
	#define MESON_P_NAND_INFO_DONE				BIT(31)

#define MESON_P_NAND_VER					0x34

#define MESON_NAND_TWB_TIME_CYCLE				10
#define MESON_NAND_TWHR_TIME_CYCLE				20

#define MESON_NAND_CLK_RATE_MHZ_DEFAULT				212 // TODO: 200 on GXBB
#define MESON_NAND_CLK_RATE_MHZ_MODE5				255 // TODO: 250 on GXBB

#define MESON_NAND_MAX_CHIPS					4

struct meson_nfc {
	struct device			*dev;
	struct nand_hw_control		base;
	void __iomem			*io_base;
	struct clk			*clk;
	struct nand_chip		*chips[MESON_NAND_MAX_CHIPS];
	int				selected_chip;
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

	if (WARN_ON(nfc->selected_chip >= MESON_NAND_MAX_CHIPS))
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

static u32 meson_nand_read_dword(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct meson_nfc *nfc = nand_get_controller_data(chip);

	meson_nand_send_chip_cmd(nfc, 0, MESON_P_NAND_CMD_CTRL_DATA_DRD);
	meson_nand_send_idle(nfc, MESON_NAND_TWB_TIME_CYCLE);

	meson_nand_wait_data_ready(nfc);

	return readl(nfc->io_base + MESON_P_NAND_BUF);
}

static u16 meson_nand_read_word(struct mtd_info *mtd)
{
	return meson_nand_read_dword(mtd);
}

static uint8_t meson_nand_read_byte(struct mtd_info *mtd)
{
	return meson_nand_read_word(mtd) & 0xff;
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
	int i, j;
	uint8_t tmp[4];
	u32 dword;

	for (i = 0; i < len;) {
		dword = meson_nand_read_dword(mtd);
		tmp[0] = (dword >> 24) & 0xff;
		tmp[1] = (dword >> 16) & 0xff;
		tmp[2] = (dword >> 8) & 0xff;
		tmp[3] = (dword >> 0) & 0xff;

		for (j = 0; j < sizeof(u32) && i < len; j++)
			buf[i++] = tmp[j];
	}
}

static void meson_nand_write_buf(struct mtd_info *mtd, const uint8_t *buf,
				 int len)
{
	int i;

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
}

static void meson_nand_select_chip(struct mtd_info *mtd, int ce)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct meson_nfc *nfc = nand_get_controller_data(chip);

	if (ce >= MESON_NAND_MAX_CHIPS) {
		nfc->selected_chip = -1;
		return;
	}

	nfc->selected_chip = ce;
}

#if 0
static void meson_nand_dma_xfer_prepare(struct mtd_info *mtd)
{
	struct meson_chip *chip = to_meson_chip(mtd);
	struct meson_nfc *nfc = nand_get_controller_data(&chip->nand_chip);
	u32 cfg;

	cfg = readl(nfc->io_base + MESON_P_NAND_CFG);

	if (mtd->writesize > SZ_4K) {
		cfg |= MESON_P_NAND_CFG_OOB_ON;
		cfg |= MESON_P_NAND_CFG_NEW_OOB_MODE;
	} else {
		cfg &= ~MESON_P_NAND_CFG_OOB_ON;
		cfg &= ~MESON_P_NAND_CFG_NEW_OOB_MODE;
	}

	writel(cfg, nfc->io_base + MESON_P_NAND_CFG);
}

static void meson_nand_dma_xfer_page(struct mtd_info *mtd, int page,
				     size_t len, int read)
{
	struct meson_chip *chip = to_meson_chip(mtd);
	struct meson_nfc *nfc = nand_get_controller_data(&chip->nand_chip);
	u32 cmd;

	meson_nand_dma_xfer_prepare(mtd);

printk("%s(%d, %d)\n", __func__, len, read);

	/* #define SEED		((8<<16) | (3<<20)) */
	/* #define NFC_CMD_SEED(seed)	(SEED|(SEED_OFFSET + (seed&0x7fff))) */
	cmd = MESON_P_NAND_CMD_DMA_XFER;
	cmd |= MESON_P_NAND_CMD_DMA_SEED;
	cmd |= MESON_P_NAND_CMD_DMA_RAN;
	cmd |= (MESON_NAND_SEED_OFFSET + (page & 0x7fff));
	writel(cmd, nfc->io_base + MESON_P_NAND_CMD);

	if (mtd->ecc.mode == NAND_ECC_NONE || NAND_ECC_SOFT) {
		/* NFC_SEND_CMD(host, (ran?N2M:N2M_NORAN)|(len&0x3fff)) */
		/* ran = 0 for writes, probably 1 for reads */
		cmd = MESON_P_NAND_CMD_DMA_XFER;
		cmd |= (read ? MESON_P_NAND_CMD_DMA_RAN : 0);
		cmd |= ((len - 1) & 0x3fff);
		writel(cmd, nfc->io_base + MESON_P_NAND_CMD);
	}

	meson_nand_wait_data_ready(nfc);
}

static int meson_nand_read_page_raw(struct mtd_info *mtd,
				    struct nand_chip *chip, uint8_t *buf,
				    int oob_required, int page)
{
	struct meson_chip *mchip = to_meson_chip(mtd);
	struct meson_nfc *nfc = nand_get_controller_data(chip);
	unsigned long timeout;
	u32 status_info;

	meson_nand_dma_xfer_page(mtd, page, mtd->writesize, 1);

	timeout = jiffies + msecs_to_jiffies(1000);

	do {
		status_info = *mchip->info_dma_buf;
		printk("%s - status = 0x%08x\n", __func__, status_info);

		if (time_after(jiffies, timeout)) {
			dev_warn(nfc->dev, "timeout waiting for DMA read\n");
			return -ETIMEDOUT;
		}
	} while ((status_info & MESON_P_NAND_INFO_DONE) == 0);

	memcpy(buf, mchip->data_dma_buf, mtd->writesize);

	return 0;
}

static int meson_nand_write_page_raw(struct mtd_info *mtd,
				     struct nand_chip *chip,
				     const uint8_t *buf,
				     int oob_required, int page)
{
	struct meson_chip *mchip = to_meson_chip(mtd);

	memcpy(mchip->data_dma_buf, buf, mtd->writesize);

	meson_nand_dma_xfer_page(mtd, page, mtd->writesize, 0);

	return 0;
}
#endif

static int meson_nand_setup_data_interface(struct mtd_info *mtd,
					const struct nand_data_interface *conf,
					bool check_only)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct meson_nfc *nfc = nand_get_controller_data(chip);
	u32 cfg;
	unsigned long rate_mhz;
	int err;

	if (check_only)
		return 0;

	/* clear any existing configuration */
	cfg = 0;
	writel(cfg, nfc->io_base + MESON_P_NAND_CFG);

	/* set bus cycle and timing like the vendor driver */
	cfg |= FIELD_PREP(MESON_P_NAND_CFG_BUS_CYCLE_MASK, 6);
	cfg |= FIELD_PREP(MESON_P_NAND_CFG_BUS_TIMING_MASK, 8);
	cfg |= FIELD_PREP(MESON_P_NAND_CFG_SYNC_MODE_MASK,
			  MESON_P_NAND_CFG_SYNC_MODE_ASYNC);
	writel(cfg, nfc->io_base + MESON_P_NAND_CFG);

	/* trigger a reset of the NAND core */
	writel(MESON_P_NAND_CMD_RESET, nfc->io_base + MESON_P_NAND_CMD);

	if (conf->timings.sdr.tREA_max > 16000)
		rate_mhz = MESON_NAND_CLK_RATE_MHZ_DEFAULT;
	else
		rate_mhz = MESON_NAND_CLK_RATE_MHZ_MODE5;

	err = clk_set_rate(nfc->clk, rate_mhz * 1000 * 1000);
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

static int meson_nand_init_chips(struct meson_nfc *nfc)
{
	struct device_node *np = nfc->dev->of_node;
	struct device_node *nand_np;
	struct nand_chip *chip;
	struct mtd_info *mtd;
	int err, nchips = 0;

	if (of_get_child_count(np) > MESON_NAND_MAX_CHIPS)
		return -EINVAL;

	for_each_child_of_node(np, nand_np) {
		chip = devm_kzalloc(nfc->dev, sizeof(struct nand_chip),
				    GFP_KERNEL);
		if (!chip)
			return -ENOMEM;

		chip->controller = &nfc->base;

		nand_set_flash_node(chip, nand_np);
		nand_set_controller_data(chip, nfc);

		mtd = nand_to_mtd(chip);
		mtd->dev.parent = nfc->dev;
		mtd->priv = chip;

		chip->dev_ready = meson_nand_dev_ready;
		chip->read_word = meson_nand_read_word;
		chip->read_byte = meson_nand_read_byte;
		chip->write_byte = meson_nand_write_byte;
		chip->read_buf = meson_nand_read_buf;
		chip->write_buf = meson_nand_write_buf;
		chip->cmd_ctrl = meson_nand_cmd_ctrl;
		chip->select_chip = meson_nand_select_chip;
		chip->setup_data_interface = meson_nand_setup_data_interface;
		chip->chip_delay = 0;

		/* Scan to find existence of the device */
		err = nand_scan(mtd, 1);
		if (err)
			return err;

		err = mtd_device_register(mtd, NULL, 0);
		if (err) {
			nand_release(mtd);
			return err;
		}

		nfc->chips[nchips] = chip;
		++nchips;
	}

	return 0;
}

static int meson_nand_probe(struct platform_device *pdev)
{
	struct meson_nfc *nfc;
	struct resource *res;
	int err;

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
	int i;

	for (i = 0; i < MESON_NAND_MAX_CHIPS; i++) {
		if (!nfc->chips[i])
			continue;

		
		nand_release(nand_to_mtd(nfc->chips[i]));
	}

	meson_nand_disable_hw(pdev);

	return 0;
}

static const struct of_device_id meson_nand_match[] = {
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
