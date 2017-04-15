/*
 * meson-mx-mmc.c - Meson8 and Meson8b SDH Controller
 *
 * Copyright (C) 2015 Endless Mobile, Inc.
 * Author: Carlo Caione <carlo@endlessm.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>

#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/slot-gpio.h>

#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>

#define MESON_MX_SDIO_ARGU					0x00

#define MESON_MX_SDIO_SEND					0x04
	#define MESON_MX_SDIO_SEND_CMD_COMMAND_MASK		GENMASK(7, 0)
	#define MESON_MX_SDIO_SEND_CMD_RESP_BITS_MASK		GENMASK(15, 8)
	#define MESON_MX_SDIO_SEND_RESP_WITHOUT_CRC7		BIT(16)
	#define MESON_MX_SDIO_SEND_RESP_HAS_DATA		BIT(17)
	#define MESON_MX_SDIO_SEND_RESP_CRC7_FROM_8		BIT(18)
	#define MESON_MX_SDIO_SEND_CHECK_DAT0_BUSY		BIT(19)
	#define MESON_MX_SDIO_SEND_DATA				BIT(20)
	#define MESON_MX_SDIO_SEND_USE_INT_WINDOW		BIT(21)
	#define MESON_MX_SDIO_SEND_REPEAT_PACKAGE_TIMES_MASK	GENMASK(31, 24)

#define MESON_MX_SDIO_CONF					0x08
	#define MESON_MX_SDIO_CONF_CMD_CLK_DIV_SHIFT		0
	#define MESON_MX_SDIO_CONF_CMD_CLK_DIV_WIDTH		10
	#define MESON_MX_SDIO_CONF_CMD_DISABLE_CRC		BIT(10)
	#define MESON_MX_SDIO_CONF_CMD_OUT_AT_POSITIVE_EDGE	BIT(11)
	#define MESON_MX_SDIO_CONF_CMD_ARGUMENT_BITS_MASK	GENMASK(17, 12)
	#define MESON_MX_SDIO_CONF_RESP_LATCH_AT_NEGATIVE_EDGE	BIT(18)
	#define MESON_MX_SDIO_CONF_DATA_LATCH_AT_NEGATIVE_EDGE	BIT(19)
	#define MESON_MX_SDIO_CONF_BUS_WIDTH			BIT(20)
	#define MESON_MX_SDIO_CONF_M_ENDIAN_MASK		GENMASK(22, 21)
	#define MESON_MX_SDIO_CONF_WRITE_NWR_MASK		GENMASK(28, 23)
	#define MESON_MX_SDIO_CONF_WRITE_CRC_OK_STATUS_MASK	GENMASK(31, 29)

#define MESON_MX_SDIO_IRQS					0x0c
	#define MESON_MX_SDIO_IRQS_CMD_BUSY			BIT(4)
	#define MESON_MX_SDIO_IRQS_RESP_CRC7_OK			BIT(5)
	#define MESON_MX_SDIO_IRQS_DATA_READ_CRC16_OK		BIT(6)
	#define MESON_MX_SDIO_IRQS_DATA_WRITE_CRC16_OK		BIT(7)
	#define MESON_MX_SDIO_IRQS_IF_INT			BIT(8)
	#define MESON_MX_SDIO_IRQS_CMD_INT			BIT(9)
	#define MESON_MX_SDIO_IRQS_STATUS_INFO_MASK		GENMASK(15, 12)
	#define MESON_MX_SDIO_IRQS_TIMING_OUT_INT		BIT(16)
	#define MESON_MX_SDIO_IRQS_AMRISC_TIMING_OUT_INT_EN	BIT(17)
	#define MESON_MX_SDIO_IRQS_ARC_TIMING_OUT_INT_EN	BIT(18)
	#define MESON_MX_SDIO_IRQS_TIMING_OUT_COUNT_MASK	GENMASK(31, 19)

#define MESON_MX_SDIO_IRQC					0x10
	#define MESON_MX_SDIO_IRQC_ARC_CMD_INT_EN		BIT(4)
	#define MESON_MX_SDIO_IRQC_IF_CONFIG_MASK		GENMASK(7, 6)
	#define MESON_MX_SDIO_IRQC_SOFT_RESET			BIT(15)
	#define MESON_MX_SDIO_IRQC_FORCE_HALT			BIT(30)
	#define MESON_MX_SDIO_IRQC_HALT_HOLE			BIT(31)

#define MESON_MX_SDIO_MULT					0x14
	#define MESON_MX_SDIO_MULT_PORT_SEL_MASK		GENMASK(1, 0)
	#define MESON_MX_SDIO_MULT_MEMORY_STICK_ENABLE		BIT(2)
	#define MESON_MX_SDIO_MULT_MEMORY_STICK_SCLK_ALWAYS	BIT(3)
	#define MESON_MX_SDIO_MULT_STREAM_ENABLE		BIT(4)
	#define MESON_MX_SDIO_MULT_STREAM_8BITS_MODE		BIT(5)
	#define MESON_MX_SDIO_MULT_WR_RD_OUT_INDEX		BIT(8)
	#define MESON_MX_SDIO_MULT_DAT0_DAT1_SWAPPED		BIT(10)
	#define MESON_MX_SDIO_MULT_DAT1_DAT0_SWAPPED		BIT(11)
	#define MESON_MX_SDIO_MULT_RESP_READ_INDEX_MASK		GENMASK(15, 12)

#define MESON_MX_SDIO_ADDR					0x18

#define MESON_MX_SDIO_EXT					0x1c
	#define MESON_MX_SDIO_EXT_DATA_RW_NUMBER_MASK		GENMASK(29, 16)

#define MESON_MX_SDIO_BOUNCE_REQ_SIZE				(128 * 1024)
#define MESON_MX_SDIO_MAX_SLOTS					3

struct meson_mx_mmc_slot {
	struct mmc_host			*mmc;
	struct mmc_request		*mrq;

	struct delayed_work     	timeout_work;

	int				slot_id;

	bool				cmd_is_stop;

	int				ios_error;

	struct meson_mx_mmc_hw		*controller;
};

struct meson_mx_mmc_hw {
	struct clk			*parent_clk;
	struct clk			*core_clk;

	struct clk_divider		cfg_div;
	struct clk			*cfg_div_clk;
	struct clk_fixed_factor		fixed_factor;
	struct clk			*fixed_factor_clk;

	spinlock_t			lock;

	void __iomem			*base;
	int				irq;

	bool				dying;

	struct meson_mx_mmc_slot	*mmc_slots[MESON_MX_SDIO_MAX_SLOTS];
	struct device			slot_devices[MESON_MX_SDIO_MAX_SLOTS];
};

static u32 meson_mx_mmc_readl(struct mmc_host *mmc, char reg)
{
	struct meson_mx_mmc_slot *slot = mmc_priv(mmc);

	return readl(slot->controller->base + reg);
}

static void meson_mx_mmc_writel(struct mmc_host *mmc, u32 val, char reg)
{
	struct meson_mx_mmc_slot *slot = mmc_priv(mmc);

	writel(val, slot->controller->base + reg);
}

static void meson_mx_mmc_request_done(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct pinctrl *pinctrl;

	pinctrl = devm_pinctrl_get_select_default(mmc->parent);
	if (IS_ERR(pinctrl))
		dev_warn(mmc_dev(mmc),
			 "failed to select default pinctrl state\n");

	mmc_request_done(mmc, mrq);
}

static int meson_mx_mmc_clk_set_rate(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct meson_mx_mmc_slot *slot = mmc_priv(mmc);
	unsigned long clk_rate = ios->clock;
	int ret;

	if (clk_rate == 0)
		return 0;

	if (WARN_ON(clk_rate > mmc->f_max))
		clk_rate = mmc->f_max;
	else if (WARN_ON(clk_rate < mmc->f_min))
		clk_rate = mmc->f_min;

	ret = clk_set_rate(slot->controller->cfg_div_clk, ios->clock);
	if (ret) {
		dev_warn(mmc_dev(mmc), "failed to set MMC clock to %lu: %d\n",
			 clk_rate, ret);
		return ret;
	}

	slot->mmc->actual_clock = clk_get_rate(slot->controller->cfg_div_clk);

	return 0;
}

static void meson_mx_mmc_soft_reset(struct mmc_host *mmc)
{
	meson_mx_mmc_writel(mmc, MESON_MX_SDIO_IRQC_SOFT_RESET,
			    MESON_MX_SDIO_IRQC);

	/* re-write the CONFIG register after a soft-reset */
	meson_mx_mmc_writel(mmc, meson_mx_mmc_readl(mmc, MESON_MX_SDIO_CONF),
			    MESON_MX_SDIO_CONF);
}

static void meson_mx_mmc_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct meson_mx_mmc_slot *slot = mmc_priv(mmc);
	struct pinctrl *pinctrl;
	unsigned long flags;
	u32 reg;

	spin_lock_irqsave(&slot->controller->lock, flags);

	pinctrl = devm_pinctrl_get_select(mmc->parent, "active");
	if (IS_ERR(pinctrl)) {
		spin_unlock_irqrestore(&slot->controller->lock, flags);

		dev_warn(mmc_dev(mmc),
			 "failed to select 'active' pinctrl state\n");
		slot->ios_error = PTR_ERR(pinctrl);

		return;
	}

	reg = meson_mx_mmc_readl(mmc, MESON_MX_SDIO_CONF);

	switch (ios->bus_width) {
	case MMC_BUS_WIDTH_1:
		reg &= ~MESON_MX_SDIO_CONF_BUS_WIDTH;
		break;

	case MMC_BUS_WIDTH_4:
		reg |= MESON_MX_SDIO_CONF_BUS_WIDTH;
		break;

	case MMC_BUS_WIDTH_8:
	default:
		dev_err(mmc_dev(mmc), "unsupported bus width: %d\n",
			ios->bus_width);
		slot->ios_error = -EINVAL;

		spin_unlock_irqrestore(&slot->controller->lock, flags);

		return;
	}

	meson_mx_mmc_writel(mmc, reg, MESON_MX_SDIO_CONF);

	spin_unlock_irqrestore(&slot->controller->lock, flags);

	switch (ios->power_mode) {
	case MMC_POWER_OFF:
		if (!IS_ERR(mmc->supply.vmmc))
			mmc_regulator_set_ocr(mmc, mmc->supply.vmmc, 0);
		break;

	case MMC_POWER_UP:
		if (!IS_ERR(mmc->supply.vmmc))
			mmc_regulator_set_ocr(mmc, mmc->supply.vmmc, ios->vdd);

		slot->ios_error = meson_mx_mmc_clk_set_rate(mmc, ios);
		break;

	case MMC_POWER_ON:
		slot->ios_error = meson_mx_mmc_clk_set_rate(mmc, ios);
		break;
	}
}

static void meson_mx_mmc_start_cmd(struct mmc_host *mmc,
				   struct mmc_command *cmd)
{
	struct meson_mx_mmc_slot *slot = mmc_priv(mmc);
	unsigned int pack_size;
	u32 irqc, irqs, mult, send = 0, ext = 0;

	switch (mmc_resp_type(cmd)) {
	case MMC_RSP_R1:
	case MMC_RSP_R1B:
	case MMC_RSP_R3:
		/* 7 (CMD) + 32 (response) + 7 (CRC) -1 */
		send |= FIELD_PREP(MESON_MX_SDIO_SEND_CMD_RESP_BITS_MASK, 45);
		break;
	case MMC_RSP_R2:
		/* 7 (CMD) + 120 (response) + 7 (CRC) -1 */
		send |= FIELD_PREP(MESON_MX_SDIO_SEND_CMD_RESP_BITS_MASK, 133);
		send |= MESON_MX_SDIO_SEND_RESP_CRC7_FROM_8;
		break;
	default:
		break;
	}

	if (!(cmd->flags & MMC_RSP_CRC))
		send |= MESON_MX_SDIO_SEND_RESP_WITHOUT_CRC7;

	if (cmd->flags & MMC_RSP_BUSY)
		send |= MESON_MX_SDIO_SEND_CHECK_DAT0_BUSY;

	if (cmd->data) {
		send |= FIELD_PREP(MESON_MX_SDIO_SEND_REPEAT_PACKAGE_TIMES_MASK,
				   (cmd->data->blocks - 1));

		pack_size = cmd->data->blksz * BITS_PER_BYTE + (16 - 1);
		if (mmc->ios.bus_width)
			pack_size *= 4;

		ext |= FIELD_PREP(MESON_MX_SDIO_EXT_DATA_RW_NUMBER_MASK,
				  pack_size);

		if (cmd->data->flags & MMC_DATA_WRITE)
			send |= MESON_MX_SDIO_SEND_DATA;
		else
			send |= MESON_MX_SDIO_SEND_RESP_HAS_DATA;
	}

	send |= FIELD_PREP(MESON_MX_SDIO_SEND_CMD_COMMAND_MASK,
			   (0x40 | cmd->opcode));

	meson_mx_mmc_soft_reset(mmc);

	irqc = meson_mx_mmc_readl(mmc, MESON_MX_SDIO_IRQC);
	irqc |= MESON_MX_SDIO_IRQC_ARC_CMD_INT_EN;

	irqs = meson_mx_mmc_readl(mmc, MESON_MX_SDIO_IRQS);
	irqs |= MESON_MX_SDIO_IRQS_CMD_INT;

	mult = meson_mx_mmc_readl(mmc, MESON_MX_SDIO_MULT);
	mult &= ~MESON_MX_SDIO_MULT_PORT_SEL_MASK;
	mult |= FIELD_PREP(MESON_MX_SDIO_MULT_PORT_SEL_MASK, slot->slot_id);
	mult |= BIT(31);

	meson_mx_mmc_writel(mmc, mult, MESON_MX_SDIO_MULT);
	meson_mx_mmc_writel(mmc, irqs, MESON_MX_SDIO_IRQS);
	meson_mx_mmc_writel(mmc, irqc, MESON_MX_SDIO_IRQC);

	wmb();

	meson_mx_mmc_writel(mmc, cmd->arg, MESON_MX_SDIO_ARGU);
	meson_mx_mmc_writel(mmc, ext, MESON_MX_SDIO_EXT);
	meson_mx_mmc_writel(mmc, send, MESON_MX_SDIO_SEND);
}

static int meson_mx_mmc_map_dma(struct meson_mx_mmc_slot *slot,
				struct mmc_data *data)
{
	u32 dma_len;
	struct scatterlist *sg = data->sg;

	if (sg->offset & 3 || sg->length & 3) {
		dev_err(mmc_dev(slot->mmc),
			"unaligned scatterlist: os %x length %d\n",
			sg->offset, sg->length);
		return -EINVAL;
	}

	dma_len = dma_map_sg(mmc_dev(slot->mmc), data->sg, data->sg_len,
			     mmc_get_dma_dir(data));
	if (dma_len == 0) {
		dev_err(mmc_dev(slot->mmc), "dma_map_sg failed\n");
		return -ENOMEM;
	}

	return 0;
}

static void meson_mx_mmc_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct meson_mx_mmc_slot *slot = mmc_priv(mmc);
	struct mmc_command *cmd = mrq->cmd;
	struct mmc_data *data = mrq->data;
	unsigned long flags;
	int ret;

	if (slot->ios_error) {
		cmd->error = slot->ios_error;
		meson_mx_mmc_request_done(mmc, mrq);
		return;
	}

	if (data) {
		ret = meson_mx_mmc_map_dma(slot, data);
		if (ret < 0) {
			dev_err(mmc_dev(mmc), "map DMA failed\n");
			cmd->error = ret;
			data->error = ret;
			meson_mx_mmc_request_done(mmc, mrq);
			return;
		}
	}

	spin_lock_irqsave(&slot->controller->lock, flags);

	if (data)
		meson_mx_mmc_writel(mmc, sg_dma_address(data->sg),
				    MESON_MX_SDIO_ADDR);

	slot->mrq = mrq;
	meson_mx_mmc_start_cmd(mmc, mrq->cmd);

	spin_unlock_irqrestore(&slot->controller->lock, flags);

	schedule_delayed_work(&slot->timeout_work, msecs_to_jiffies(10000));
}

static irqreturn_t meson_mx_mmc_irq(int irq, void *data)
{
	struct meson_mx_mmc_hw *hw = (void *) data;
	unsigned long flags;
	u32 irqs;

	spin_lock_irqsave(&hw->lock, flags);
	irqs = readl(hw->base + MESON_MX_SDIO_IRQS);
	spin_unlock_irqrestore(&hw->lock, flags);

	if (irqs & MESON_MX_SDIO_IRQS_CMD_INT)
		return IRQ_WAKE_THREAD;

	return IRQ_HANDLED;
}

void meson_mx_mmc_read_response(struct mmc_host *mmc)
{
	struct meson_mx_mmc_slot *slot = mmc_priv(mmc);
	struct mmc_command *cmd = slot->mrq->cmd;
	u32 mult;
	int i, resp[4] = { 0 };

	mult = meson_mx_mmc_readl(mmc, MESON_MX_SDIO_MULT);
	mult |= MESON_MX_SDIO_MULT_WR_RD_OUT_INDEX;
	mult &= ~MESON_MX_SDIO_MULT_RESP_READ_INDEX_MASK;
	meson_mx_mmc_writel(mmc, mult, MESON_MX_SDIO_MULT);

	if (cmd->flags & MMC_RSP_136) {
		for (i = 0; i <= 3; i++)
			resp[3 - i] = meson_mx_mmc_readl(mmc,
							 MESON_MX_SDIO_ARGU);
		cmd->resp[0] = (resp[0] << 8) | ((resp[1] >> 24) & 0xff);
		cmd->resp[1] = (resp[1] << 8) | ((resp[2] >> 24) & 0xff);
		cmd->resp[2] = (resp[2] << 8) | ((resp[3] >> 24) & 0xff);
		cmd->resp[3] = (resp[3] << 8);
	} else if (cmd->flags & MMC_RSP_PRESENT) {
		cmd->resp[0] = meson_mx_mmc_readl(mmc, MESON_MX_SDIO_ARGU);
	}
}

static irqreturn_t meson_mx_mmc_irq_thread(int irq, void *irq_data)
{
	struct meson_mx_mmc_hw *hw = (void *) irq_data;
	struct meson_mx_mmc_slot *slot;
	struct mmc_host *mmc;
	struct mmc_data *data;
	unsigned long flags;
	struct mmc_request *mrq;
	unsigned slot_id;
	u32 irqs, send;

	spin_lock_irqsave(&hw->lock, flags);

	slot_id = FIELD_GET(MESON_MX_SDIO_MULT_PORT_SEL_MASK,
			    readl(hw->base + MESON_MX_SDIO_MULT));

	slot = hw->mmc_slots[slot_id];

	mmc = slot->mmc;
	mrq = slot->mrq;
	data = mrq->data;

	cancel_delayed_work_sync(&slot->timeout_work);

	if (!mrq) {
		spin_unlock_irqrestore(&hw->lock, flags);
		return IRQ_HANDLED;
	}

	if (slot->cmd_is_stop)
		goto out;

	irqs = meson_mx_mmc_readl(mmc, MESON_MX_SDIO_IRQS);
	send = meson_mx_mmc_readl(mmc, MESON_MX_SDIO_SEND);

	mrq->cmd->error = 0;

	if (!data) {
		if (!((irqs & MESON_MX_SDIO_IRQS_RESP_CRC7_OK) ||
		      (send & MESON_MX_SDIO_SEND_RESP_WITHOUT_CRC7)))
			mrq->cmd->error = -EILSEQ;
		else
			meson_mx_mmc_read_response(mmc);
	} else {
		if (!((irqs & MESON_MX_SDIO_IRQS_DATA_READ_CRC16_OK) ||
		      (irqs & MESON_MX_SDIO_IRQS_DATA_WRITE_CRC16_OK))) {
			mrq->cmd->error = -EILSEQ;
		} else {
			data->bytes_xfered = data->blksz * data->blocks;
			dma_unmap_sg(mmc_dev(mmc), data->sg, data->sg_len,
				     mmc_get_dma_dir(data));
		}
	}

	if (mrq->stop) {
		slot->cmd_is_stop = true;
		meson_mx_mmc_start_cmd(mmc, mrq->stop);
		spin_unlock_irqrestore(&hw->lock, flags);
		return IRQ_HANDLED;
	}

out:
	slot->cmd_is_stop = false;
	slot->mrq = NULL;

	spin_unlock_irqrestore(&hw->lock, flags);

	meson_mx_mmc_request_done(mmc, mrq);

	return IRQ_HANDLED;
}

static void meson_mx_mmc_timeout(struct work_struct *work)
{
	struct meson_mx_mmc_slot *slot = container_of(work,
						      struct meson_mx_mmc_slot,
						      timeout_work.work);
	struct mmc_host *mmc = slot->mmc;
	struct mmc_request *mrq = slot->mrq;
	unsigned long flags;
	u32 irqc;

	/* Do not run after meson_mx_mmc_remove() */
	if (slot->controller->dying)
		return;

	dev_err(mmc_dev(slot->mmc),
		"Timeout on CMD%u (IRQS = 0x%08x, ARGU = 0x%08x)\n",
		mrq->cmd->opcode, meson_mx_mmc_readl(mmc, MESON_MX_SDIO_IRQS),
		meson_mx_mmc_readl(mmc, MESON_MX_SDIO_ARGU));

	spin_lock_irqsave(&slot->controller->lock, flags);

	irqc = meson_mx_mmc_readl(mmc, MESON_MX_SDIO_IRQC);
	irqc &= ~MESON_MX_SDIO_IRQC_ARC_CMD_INT_EN;
	meson_mx_mmc_writel(mmc, irqc, MESON_MX_SDIO_IRQC);

	mrq->cmd->error = -ETIMEDOUT;
	slot->mrq = NULL;

	spin_unlock_irqrestore(&slot->controller->lock, flags);

	meson_mx_mmc_request_done(slot->mmc, mrq);
}

static struct mmc_host_ops meson_mx_mmc_ops = {
	.request	= meson_mx_mmc_request,
	.set_ios	= meson_mx_mmc_set_ios,
	.get_cd		= mmc_gpio_get_cd,
	.get_ro		= mmc_gpio_get_ro,
};

static int meson_mx_mmc_add_slot(struct device *slot_dev, unsigned slot_id,
				 struct meson_mx_mmc_hw *hw)
{
	struct meson_mx_mmc_slot *slot;
	struct mmc_host *mmc;
	int ret;

	if (!of_device_is_available(slot_dev->of_node))
		return 0;

	mmc = mmc_alloc_host(sizeof(struct meson_mx_mmc_slot), slot_dev);
	if (!mmc) {
		dev_err(slot_dev, "mmc alloc host failed\n");
		ret = -ENOMEM;
		goto error;
	}

	slot = mmc_priv(mmc);
	slot->mmc = mmc;
	slot->slot_id = slot_id;
	slot->controller = hw;

	hw->mmc_slots[slot_id] = slot;

	INIT_DELAYED_WORK(&slot->timeout_work, meson_mx_mmc_timeout);

	/* Get regulators and the supported OCR mask */
	ret = mmc_regulator_get_supply(mmc);
	if (ret == -EPROBE_DEFER)
		goto error_free_host;

	/* we do not support scatter lists in hardware */
	mmc->max_segs = 1;
	mmc->max_req_size = MESON_MX_SDIO_BOUNCE_REQ_SIZE;
	mmc->max_seg_size = mmc->max_req_size;
	mmc->max_blk_count = 256;
	mmc->max_blk_size = mmc->max_req_size / mmc->max_blk_count;

	/* Get the min and max supported clock rates */
	mmc->f_min = clk_round_rate(hw->cfg_div_clk, 1);
	mmc->f_max = clk_round_rate(hw->cfg_div_clk,
				    clk_get_rate(hw->parent_clk));

	mmc->caps2 |= MMC_CAP2_NO_SDIO;
	mmc->ops = &meson_mx_mmc_ops;

	ret = mmc_of_parse(mmc);
	if (ret)
		goto error_free_host;

	ret = mmc_add_host(mmc);
	if (ret)
		goto error_free_host;

	return 0;

error_free_host:
	mmc_free_host(mmc);
error:
	return ret;
}

static int meson_mx_mmc_probe_slots(struct platform_device *pdev)
{
	struct meson_mx_mmc_hw *hw = platform_get_drvdata(pdev);
	struct device *parent_dev = &pdev->dev;
	struct device_node *slot_node, *controller_node;
	struct device *slot_dev;
	unsigned slot_id;
	int num_slots, ret;

	controller_node = parent_dev->of_node;

	num_slots = of_get_available_child_count(controller_node);
	if (num_slots > MESON_MX_SDIO_MAX_SLOTS) {
		dev_err(parent_dev, "more slots configured than supported\n");
		return -EINVAL;
	}

	for_each_child_of_node(controller_node, slot_node) {
		if (of_property_read_u32(slot_node, "reg", &slot_id)) {
			dev_err(parent_dev, "missing 'reg' property for %s\n",
				of_node_full_name(slot_node));
			return -EINVAL;
		}

		if (slot_id >= MESON_MX_SDIO_MAX_SLOTS) {
			dev_err(parent_dev, "invalid 'reg' property value of %s\n",
				of_node_full_name(slot_node));
			return -EINVAL;
		}

		slot_dev = &hw->slot_devices[slot_id];

		device_initialize(slot_dev);
		dev_set_name(slot_dev, "%s.%d", dev_name(parent_dev), slot_id);

		slot_dev->parent = parent_dev;
		slot_dev->of_node = slot_node;

		ret = device_add(slot_dev);
		if (ret)
			return ret;

		ret = meson_mx_mmc_add_slot(slot_dev, slot_id, hw);
		if (ret) {
			device_unregister(slot_dev);
			return ret;
		}
	}

	return ret;
}

static int meson_mx_mmc_register_clks(struct platform_device *pdev)
{
	struct meson_mx_mmc_hw *hw = platform_get_drvdata(pdev);
	struct clk_init_data init;
	const char *clk_div_parents[1], *clk_fixed_factor_parents[1];

	clk_fixed_factor_parents[0] = __clk_get_name(hw->parent_clk);
	init.name = devm_kasprintf(&pdev->dev, GFP_KERNEL, "%s#fixed_factor",
				   dev_name(&pdev->dev));
	init.ops = &clk_fixed_factor_ops;
	init.flags = 0;
	init.parent_names = clk_fixed_factor_parents;
	init.num_parents = 1;
	hw->fixed_factor.div = 2;
	hw->fixed_factor.mult = 1;
	hw->fixed_factor.hw.init = &init;

	hw->fixed_factor_clk = devm_clk_register(&pdev->dev,
						 &hw->fixed_factor.hw);
	if (WARN_ON(PTR_ERR_OR_ZERO(hw->fixed_factor_clk)))
		return PTR_ERR(hw->fixed_factor_clk);

	clk_div_parents[0] = __clk_get_name(hw->fixed_factor_clk);
	init.name = devm_kasprintf(&pdev->dev, GFP_KERNEL, "%s#div",
				   dev_name(&pdev->dev));
	init.ops = &clk_divider_ops;
	init.flags = CLK_SET_RATE_PARENT;
	init.parent_names = clk_div_parents;
	init.num_parents = 1;
	hw->cfg_div.reg = hw->base + MESON_MX_SDIO_CONF;
	hw->cfg_div.shift = MESON_MX_SDIO_CONF_CMD_CLK_DIV_SHIFT;
	hw->cfg_div.width = MESON_MX_SDIO_CONF_CMD_CLK_DIV_WIDTH;
	hw->cfg_div.lock = &hw->lock;
	hw->cfg_div.hw.init = &init;
	hw->cfg_div.flags = CLK_DIVIDER_ALLOW_ZERO;

	hw->cfg_div_clk = devm_clk_register(&pdev->dev, &hw->cfg_div.hw);
	if (WARN_ON(PTR_ERR_OR_ZERO(hw->cfg_div_clk)))
		return PTR_ERR(hw->fixed_factor_clk);

	return 0;
}

static int meson_mx_mmc_probe(struct platform_device *pdev)
{
	struct meson_mx_mmc_hw *hw;
	struct resource *res;
	int ret, irq;
	u32 conf;

	hw = devm_kzalloc(&pdev->dev, sizeof(*hw), GFP_KERNEL);
	if (!hw)
		return -ENOMEM;

	spin_lock_init(&hw->lock);

	platform_set_drvdata(pdev, hw);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	hw->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(hw->base)) {
		ret = PTR_ERR(hw->base);
		goto error_out;
	}

	irq = platform_get_irq(pdev, 0);
	ret = devm_request_threaded_irq(&pdev->dev, irq, meson_mx_mmc_irq,
					meson_mx_mmc_irq_thread, 0, NULL,
					hw);
	if (ret)
		goto error_out;

	hw->core_clk = devm_clk_get(&pdev->dev, "core");
	if (IS_ERR(hw->core_clk)) {
		ret = PTR_ERR(hw->core_clk);
		goto error_out;
	}

	hw->parent_clk = devm_clk_get(&pdev->dev, "clkin");
	if (IS_ERR(hw->parent_clk)) {
		ret = PTR_ERR(hw->parent_clk);
		goto error_disable_core_clk;
	}

	ret = meson_mx_mmc_register_clks(pdev);
	if (ret)
		goto error_disable_clks;

	ret = clk_prepare_enable(hw->core_clk);
	if (ret) {
		dev_err(&pdev->dev, "Failed to enable core clock\n");
		goto error_disable_clks;
	}

	ret = clk_prepare_enable(hw->cfg_div_clk);
	if (ret) {
		dev_err(&pdev->dev, "Failed to enable MMC clock\n");
		goto error_disable_clks;
	}

	ret = meson_mx_mmc_probe_slots(pdev);
	if (ret)
		goto error_disable_clks;

	conf = 0;
	conf |= FIELD_PREP(MESON_MX_SDIO_CONF_CMD_ARGUMENT_BITS_MASK, 39);
	conf |= FIELD_PREP(MESON_MX_SDIO_CONF_M_ENDIAN_MASK, 0x3);
	conf |= FIELD_PREP(MESON_MX_SDIO_CONF_WRITE_NWR_MASK, 0x2);
	conf |= FIELD_PREP(MESON_MX_SDIO_CONF_WRITE_CRC_OK_STATUS_MASK, 0x2);
	writel(conf, hw->base + MESON_MX_SDIO_CONF);

	return 0;

error_disable_clks:
	clk_disable_unprepare(hw->cfg_div_clk);
error_disable_core_clk:
	clk_disable_unprepare(hw->core_clk);
error_out:
	return ret;
}

static int meson_mx_mmc_remove(struct platform_device *pdev)
{
	struct meson_mx_mmc_hw *hw = platform_get_drvdata(pdev);
	struct meson_mx_mmc_slot *slot;
	int i;

	hw->dying = true;

	for (i = 0; i < MESON_MX_SDIO_MAX_SLOTS; i++) {
		slot = hw->mmc_slots[i];

		if (!slot->mmc)
			continue;

		cancel_delayed_work_sync(&slot->timeout_work);

		mmc_remove_host(slot->mmc);

		mmc_free_host(slot->mmc);
	}

	for (i = 0; i < MESON_MX_SDIO_MAX_SLOTS; i++) {
		if (!device_is_registered(&hw->slot_devices[i]))
			continue;

		device_unregister(&hw->slot_devices[i]);
	}

	clk_disable_unprepare(hw->core_clk);
	clk_disable_unprepare(hw->cfg_div_clk);

	return 0;
}

static const struct of_device_id meson_mx_mmc_of_match[] = {
	{ .compatible = "amlogic,meson8-mmc", },
	{ .compatible = "amlogic,meson8b-mmc", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, meson_mx_mmc_of_match);

static struct platform_driver meson_mx_mmc_driver = {
	.probe   = meson_mx_mmc_probe,
	.remove  = meson_mx_mmc_remove,
	.driver  = {
		.name = "meson-mx-mmc",
		.of_match_table = of_match_ptr(meson_mx_mmc_of_match),
	},
};

module_platform_driver(meson_mx_mmc_driver);

MODULE_DESCRIPTION("Meson8/Meson8b Secure Digital Host Driver");
MODULE_AUTHOR("Carlo Caione <carlo@endlessm.com>");
MODULE_LICENSE("GPLv2");
