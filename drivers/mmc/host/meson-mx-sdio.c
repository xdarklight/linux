/*
 * meson-mx-sdio.c - Meson6, Meson8 and Meson8b SDIO/MMC Host Controller
 *
 * Copyright (C) 2015 Endless Mobile, Inc.
 * Author: Carlo Caione <carlo@endlessm.com>
 * Copyright (C) 2017 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
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
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/semaphore.h>
#include <linux/timer.h>
#include <linux/types.h>

#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/slot-gpio.h>

#define MESON_MX_SDIO_ARGU					0x00

#define MESON_MX_SDIO_SEND					0x04
	#define MESON_MX_SDIO_SEND_COMMAND_INDEX_MASK		GENMASK(7, 0)
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
	#define MESON_MX_SDIO_IRQS_STATUS_STATE_MACHINE_MASK	GENMASK(3, 0)
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
	#define MESON_MX_SDIO_IRQC_ARC_IF_INT_EN		BIT(3)
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
#define MESON_MX_SDIO_RESPONSE_CRC16_BITS			(16 - 1)
#define MESON_MX_SDIO_MAX_SLOTS					3

enum meson_mx_mmc_host_status {
	MESON_MX_MMC_STATUS_IDLE,
	MESON_MX_MMC_STATUS_BUSY,
	MESON_MX_MMC_STATUS_SHUTTING_DOWN,
};

struct meson_mx_mmc_slot {
	struct mmc_host			*mmc;
	struct meson_mx_mmc_host	*host;

	struct mmc_request		*mrq;
	struct mmc_command		*cmd;
	int				error;

	unsigned int			id;
	struct list_head		queue_node;
};

struct meson_mx_mmc_host {
	struct device			*dev;

	struct clk			*parent_clk;
	struct clk			*core_clk;
	struct clk_divider		cfg_div;
	struct clk			*cfg_div_clk;
	struct clk_fixed_factor		fixed_factor;
	struct clk			*fixed_factor_clk;

	void __iomem			*base;
	int				irq;
	spinlock_t			lock;
	spinlock_t			irq_lock;

	enum meson_mx_mmc_host_status	status;
	struct list_head		queue;
	struct timer_list		cmd_timeout;

	struct device			*slot_devices[MESON_MX_SDIO_MAX_SLOTS];
	struct meson_mx_mmc_slot	*slots[MESON_MX_SDIO_MAX_SLOTS];
	struct meson_mx_mmc_slot	*current_cmd_slot;
	struct meson_mx_mmc_slot	*sdio_irq_slot;
};

static void meson_mx_mmc_mask_bits(struct mmc_host *mmc, char reg, u32 mask,
				   u32 val)
{
	struct meson_mx_mmc_slot *slot = mmc_priv(mmc);
	u32 regval;

	regval = readl(slot->host->base + reg);
	regval &= ~mask;
	regval |= (val & mask);

	writel(regval, slot->host->base + reg);
}

static void meson_mx_mmc_soft_reset(struct meson_mx_mmc_host *host)
{
	writel(MESON_MX_SDIO_IRQC_SOFT_RESET, host->base + MESON_MX_SDIO_IRQC);
	udelay(2);
}

static struct mmc_command *meson_mx_mmc_get_next_cmd(struct mmc_command *cmd)
{
	if (cmd->opcode == MMC_SET_BLOCK_COUNT && !cmd->error)
		return cmd->mrq->cmd;
	else if (mmc_op_multi(cmd->opcode) &&
		 (!cmd->mrq->sbc || cmd->error || cmd->data->error))
		return cmd->mrq->stop;
	else
		return NULL;
}

static void meson_mx_mmc_apply_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct meson_mx_mmc_slot *slot = mmc_priv(mmc);
	unsigned long clk_rate = ios->clock;
	int ret;

	switch (ios->bus_width) {
	case MMC_BUS_WIDTH_1:
		meson_mx_mmc_mask_bits(mmc, MESON_MX_SDIO_CONF,
				       MESON_MX_SDIO_CONF_BUS_WIDTH, 0);
		break;

	case MMC_BUS_WIDTH_4:
		meson_mx_mmc_mask_bits(mmc, MESON_MX_SDIO_CONF,
				       MESON_MX_SDIO_CONF_BUS_WIDTH,
				       MESON_MX_SDIO_CONF_BUS_WIDTH);
		break;

	case MMC_BUS_WIDTH_8:
	default:
		dev_err(mmc_dev(mmc), "unsupported bus width: %d\n",
			ios->bus_width);
		slot->error = -EINVAL;
		return;
	}

	if (WARN_ON(clk_rate > mmc->f_max))
		clk_rate = mmc->f_max;

	ret = clk_set_rate(slot->host->cfg_div_clk, ios->clock);
	if (ret) {
		dev_warn(mmc_dev(mmc),
				"failed to set MMC clock to %lu: %d\n",
			clk_rate, ret);
		slot->error = ret;
		return;
	}

	mmc->actual_clock = clk_get_rate(slot->host->cfg_div_clk);
}

static void meson_mx_mmc_start_cmd(struct mmc_host *mmc,
				   struct mmc_command *cmd)
{
	struct meson_mx_mmc_slot *slot = mmc_priv(mmc);
	unsigned int pack_size;
	unsigned long irqflags, timeout;
	u32 mult, send = 0, ext = 0;

	slot->cmd = cmd;

	spin_lock_irqsave(&slot->host->irq_lock, irqflags);

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

		pack_size = cmd->data->blksz * BITS_PER_BYTE;
		if (mmc->ios.bus_width == MMC_BUS_WIDTH_4)
			pack_size += MESON_MX_SDIO_RESPONSE_CRC16_BITS * 4;
		else
			pack_size += MESON_MX_SDIO_RESPONSE_CRC16_BITS * 1;

		ext |= FIELD_PREP(MESON_MX_SDIO_EXT_DATA_RW_NUMBER_MASK,
				  pack_size);

		if (cmd->data->flags & MMC_DATA_WRITE)
			send |= MESON_MX_SDIO_SEND_DATA;
		else
			send |= MESON_MX_SDIO_SEND_RESP_HAS_DATA;

		cmd->data->bytes_xfered = 0;
	}

	send |= FIELD_PREP(MESON_MX_SDIO_SEND_COMMAND_INDEX_MASK,
			   (0x40 | cmd->opcode));

	/*
	 * soft-reset the MMC core and re-apply the IOS to make sure that these
	 * are correct for the slot which is selected below (as the IOS
	 * registers are shared across all slots).
	 */
	meson_mx_mmc_soft_reset(slot->host);
	meson_mx_mmc_apply_ios(mmc, &mmc->ios);

	mult = readl(slot->host->base + MESON_MX_SDIO_MULT);
	mult &= ~MESON_MX_SDIO_MULT_PORT_SEL_MASK;
	mult |= FIELD_PREP(MESON_MX_SDIO_MULT_PORT_SEL_MASK, slot->id);
	mult |= BIT(31);
	writel(mult, slot->host->base + MESON_MX_SDIO_MULT);

	meson_mx_mmc_mask_bits(mmc, MESON_MX_SDIO_IRQC,
			       MESON_MX_SDIO_IRQC_ARC_CMD_INT_EN,
			       MESON_MX_SDIO_IRQC_ARC_CMD_INT_EN);

	/* clear pending interrupts */
	meson_mx_mmc_mask_bits(mmc, MESON_MX_SDIO_IRQS,
			       MESON_MX_SDIO_IRQS_CMD_INT,
			       MESON_MX_SDIO_IRQS_CMD_INT);

	writel(cmd->arg, slot->host->base + MESON_MX_SDIO_ARGU);
	writel(ext, slot->host->base + MESON_MX_SDIO_EXT);
	writel(send, slot->host->base + MESON_MX_SDIO_SEND);

	spin_unlock_irqrestore(&slot->host->irq_lock, irqflags);

	if (cmd->busy_timeout)
		timeout = msecs_to_jiffies(cmd->busy_timeout);
	else
		timeout = msecs_to_jiffies(1000);

	mod_timer(&slot->host->cmd_timeout, jiffies + timeout);
}

static void meson_mx_mmc_start_request(struct mmc_host *mmc,
				       struct mmc_request *mrq)
{
	struct meson_mx_mmc_slot *slot = mmc_priv(mmc);
	struct meson_mx_mmc_host *host = slot->host;

	host->status = MESON_MX_MMC_STATUS_BUSY;
	host->current_cmd_slot = slot;

	if (mrq->data)
		writel(sg_dma_address(mrq->data->sg),
		       host->base + MESON_MX_SDIO_ADDR);

	if (mrq->sbc)
		meson_mx_mmc_start_cmd(mmc, mrq->sbc);
	else
		meson_mx_mmc_start_cmd(mmc, mrq->cmd);
}

static void meson_mx_mmc_request_done(struct meson_mx_mmc_host *host,
				      struct meson_mx_mmc_slot *current_slot)
	__releases(&host->lock)
	__acquires(&host->lock)
{
	struct meson_mx_mmc_slot *next_slot;
	struct mmc_request *mrq = current_slot->mrq;

	current_slot->mrq = NULL;
	current_slot->cmd = NULL;

	if (list_empty(&host->queue)) {
		dev_dbg(host->dev, "slot queue is empty\n");

		host->status = MESON_MX_MMC_STATUS_IDLE;
		host->current_cmd_slot = NULL;
	} else {
		next_slot = list_entry(host->queue.next,
				       struct meson_mx_mmc_slot, queue_node);
		list_del(&next_slot->queue_node);

		dev_dbg(host->dev, "next slot from queue is %s\n",
			mmc_hostname(next_slot->mmc));

		meson_mx_mmc_start_request(next_slot->mmc, next_slot->mrq);
	}

	spin_unlock(&host->lock);
	mmc_request_done(current_slot->mmc, mrq);
	spin_lock(&host->lock);
}

static void meson_mx_mmc_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct meson_mx_mmc_slot *slot = mmc_priv(mmc);

	if (spin_trylock(&slot->host->lock)) {
		/*
		 * only apply the mmc_ios if we are idle to not break any
		 * ongoing transfer. in case we are busy meson_mx_mmc_start_cmd
		 * will take care of applying the mmc_ios later on.
		 */
		if (slot->host->status == MESON_MX_MMC_STATUS_IDLE)
			meson_mx_mmc_apply_ios(mmc, ios);

		spin_unlock(&slot->host->lock);
	}

	switch (ios->power_mode) {
	case MMC_POWER_OFF:
		if (!IS_ERR(mmc->supply.vmmc))
			mmc_regulator_set_ocr(mmc, mmc->supply.vmmc, 0);
		break;

	case MMC_POWER_UP:
		if (!IS_ERR(mmc->supply.vmmc))
			mmc_regulator_set_ocr(mmc, mmc->supply.vmmc, ios->vdd);
		break;
	}
}

static void meson_mx_mmc_enable_sdio_irq(struct mmc_host *mmc, int enable)
{
	struct meson_mx_mmc_slot *slot = mmc_priv(mmc);
	unsigned long irqflags;

	spin_lock_irqsave(&slot->host->irq_lock, irqflags);

	meson_mx_mmc_mask_bits(mmc, MESON_MX_SDIO_MULT,
			       MESON_MX_SDIO_MULT_PORT_SEL_MASK,
			       FIELD_PREP(MESON_MX_SDIO_MULT_PORT_SEL_MASK,
					  slot->id));

	/* ACK pending interrupt */
	meson_mx_mmc_mask_bits(mmc, MESON_MX_SDIO_IRQS,
			       MESON_MX_SDIO_IRQS_IF_INT,
			       MESON_MX_SDIO_IRQS_IF_INT);

	meson_mx_mmc_mask_bits(mmc, MESON_MX_SDIO_IRQC,
			       MESON_MX_SDIO_IRQC_ARC_IF_INT_EN,
			       enable ? MESON_MX_SDIO_IRQC_ARC_IF_INT_EN : 0);

	if (enable)
		slot->host->sdio_irq_slot = slot;
	else
		slot->host->sdio_irq_slot = NULL;

	spin_unlock_irqrestore(&slot->host->irq_lock, irqflags);
}

static int meson_mx_mmc_map_dma(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct mmc_data *data = mrq->data;
	int dma_len;
	struct scatterlist *sg;

	if (!data)
		return 0;

	sg = data->sg;
	if (sg->offset & 3 || sg->length & 3) {
		dev_err(mmc_dev(mmc),
			"unaligned scatterlist: offset %x length %d\n",
			sg->offset, sg->length);
		return -EINVAL;
	}

	dma_len = dma_map_sg(mmc_dev(mmc), data->sg, data->sg_len,
			     mmc_get_dma_dir(data));
	if (dma_len <= 0) {
		dev_err(mmc_dev(mmc), "dma_map_sg failed\n");
		return -ENOMEM;
	}

	return 0;
}

static void meson_mx_mmc_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct meson_mx_mmc_slot *slot = mmc_priv(mmc);
	struct meson_mx_mmc_host *host = slot->host;
	struct mmc_command *cmd = mrq->cmd;

	if (!slot->error)
		slot->error = meson_mx_mmc_map_dma(mmc, mrq);

	if (slot->error) {
		cmd->error = slot->error;
		mmc_request_done(mmc, mrq);
		return;
	}

	spin_lock(&host->lock);

	slot->mrq = mrq;

	if (host->status == MESON_MX_MMC_STATUS_IDLE)
		meson_mx_mmc_start_request(mmc, mrq);
	else
		list_add_tail(&slot->queue_node, &host->queue);

	spin_unlock(&host->lock);
}

static void meson_mx_mmc_read_response(struct mmc_host *mmc,
				       struct mmc_command *cmd)
{
	struct meson_mx_mmc_slot *slot = mmc_priv(mmc);
	struct meson_mx_mmc_host *host = slot->host;
	u32 mult;
	int i, resp[4];

	mult = readl(host->base + MESON_MX_SDIO_MULT);
	mult |= MESON_MX_SDIO_MULT_WR_RD_OUT_INDEX;
	mult &= ~MESON_MX_SDIO_MULT_RESP_READ_INDEX_MASK;
	mult |= FIELD_PREP(MESON_MX_SDIO_MULT_RESP_READ_INDEX_MASK, 0);
	writel(mult, host->base + MESON_MX_SDIO_MULT);

	if (cmd->flags & MMC_RSP_136) {
		for (i = 0; i <= 3; i++)
			resp[3 - i] = readl(host->base + MESON_MX_SDIO_ARGU);
		cmd->resp[0] = (resp[0] << 8) | ((resp[1] >> 24) & 0xff);
		cmd->resp[1] = (resp[1] << 8) | ((resp[2] >> 24) & 0xff);
		cmd->resp[2] = (resp[2] << 8) | ((resp[3] >> 24) & 0xff);
		cmd->resp[3] = (resp[3] << 8);
	} else if (cmd->flags & MMC_RSP_PRESENT) {
		cmd->resp[0] = readl(host->base + MESON_MX_SDIO_ARGU);
	}
}

static irqreturn_t meson_mx_mmc_process_cmd_irq(struct meson_mx_mmc_slot *slot,
						u32 irqs, u32 send)
{
	struct mmc_command *cmd;

	if (WARN_ON(!slot))
		return IRQ_HANDLED;

	cmd = slot->cmd;
	if (WARN_ON(!cmd))
		return IRQ_HANDLED;

	cmd->error = 0;
	meson_mx_mmc_read_response(slot->mmc, cmd);

	if (cmd->data) {
		if (!((irqs & MESON_MX_SDIO_IRQS_DATA_READ_CRC16_OK) ||
		      (irqs & MESON_MX_SDIO_IRQS_DATA_WRITE_CRC16_OK)))
			cmd->error = -EILSEQ;
	} else {
		if (!((irqs & MESON_MX_SDIO_IRQS_RESP_CRC7_OK) ||
		      (send & MESON_MX_SDIO_SEND_RESP_WITHOUT_CRC7)))
			cmd->error = -EILSEQ;
	}

	return IRQ_WAKE_THREAD;
}

static void meson_mx_mmc_process_sdio_irq(struct meson_mx_mmc_slot *slot)
{
	/*
	 * ignore SDIO interrupts without corresponding slot as the SDIO
	 * interrupt seems to enable itself automatically - in this case we
	 * didn't assign a slot for this IRQ so we simply ignore it.
	 */
	if (!slot)
		return;

	mmc_signal_sdio_irq(slot->mmc);
}

static irqreturn_t meson_mx_mmc_irq(int irq, void *data)
{
	struct meson_mx_mmc_host *host = (void *) data;
	struct meson_mx_mmc_slot *cmd_slot, *sdio_irq_slot;
	bool sdio_irq, cmd_irq;
	u32 irqs, send;
	irqreturn_t ret;

	spin_lock(&host->irq_lock);

	irqs = readl(host->base + MESON_MX_SDIO_IRQS);
	send = readl(host->base + MESON_MX_SDIO_SEND);

	cmd_slot = host->current_cmd_slot;
	sdio_irq_slot = host->sdio_irq_slot;

	sdio_irq = !!(irqs & MESON_MX_SDIO_IRQS_IF_INT);
	cmd_irq = !!(irqs & MESON_MX_SDIO_IRQS_CMD_INT);

	if (cmd_irq) {
		del_timer(&host->cmd_timeout);

		ret = meson_mx_mmc_process_cmd_irq(cmd_slot, irqs, send);
	} else
		ret = IRQ_HANDLED;

	/* and finally ACK all pending interrupts */
	writel(irqs, host->base + MESON_MX_SDIO_IRQS);

	spin_unlock(&host->irq_lock);

	if (sdio_irq)
		meson_mx_mmc_process_sdio_irq(sdio_irq_slot);

	return ret;
}

static irqreturn_t meson_mx_mmc_irq_thread(int irq, void *irq_data)
{
	struct meson_mx_mmc_host *host = (void *) irq_data;
	struct meson_mx_mmc_slot *slot;
	struct mmc_command *cmd, *next_cmd;

	spin_lock(&host->lock);

	slot = host->current_cmd_slot;
	if (WARN_ON(!slot)) {
		spin_unlock(&host->lock);
		return IRQ_HANDLED;
	}

	cmd = slot->cmd;

	if (cmd->data) {
		dma_unmap_sg(mmc_dev(slot->mmc), cmd->data->sg,
				cmd->data->sg_len,
				mmc_get_dma_dir(cmd->data));

		cmd->data->bytes_xfered = cmd->data->blksz * cmd->data->blocks;
	}

	next_cmd = meson_mx_mmc_get_next_cmd(cmd);
	if (next_cmd)
		meson_mx_mmc_start_cmd(slot->mmc, next_cmd);
	else
		meson_mx_mmc_request_done(host, slot);

	spin_unlock(&host->lock);

	return IRQ_HANDLED;
}

static void meson_mx_mmc_timeout(unsigned long arg)
{
	struct meson_mx_mmc_host *host = (void *) arg;
	struct meson_mx_mmc_slot *slot;
	unsigned long irqflags;
	u32 irqc;

	spin_lock(&host->lock);

	/* Do not run after meson_mx_mmc_remove() */
	if (host->status == MESON_MX_MMC_STATUS_SHUTTING_DOWN)
		goto unlock;

	/* request was completed in the meantime */
	if (host->status == MESON_MX_MMC_STATUS_IDLE)
		goto unlock;

	spin_lock_irqsave(&host->irq_lock, irqflags);

	/* disable the CMD interrupt */
	irqc = readl(host->base + MESON_MX_SDIO_IRQC);
	irqc &= ~MESON_MX_SDIO_IRQC_ARC_CMD_INT_EN;
	writel(irqc, host->base + MESON_MX_SDIO_IRQC);

	spin_unlock_irqrestore(&host->irq_lock, irqflags);

	slot = host->current_cmd_slot;

	if (WARN_ON(!slot))
		goto unlock;

	dev_dbg(mmc_dev(slot->mmc),
		"Timeout on CMD%u (IRQS = 0x%08x, ARGU = 0x%08x)\n",
		slot->cmd->opcode, readl(host->base + MESON_MX_SDIO_IRQS),
		readl(host->base + MESON_MX_SDIO_ARGU));

	slot->cmd->error = -ETIMEDOUT;

	meson_mx_mmc_request_done(host, slot);

unlock:
	spin_unlock(&host->lock);
}

static struct mmc_host_ops meson_mx_mmc_ops = {
	.request		= meson_mx_mmc_request,
	.set_ios		= meson_mx_mmc_set_ios,
	.enable_sdio_irq	= meson_mx_mmc_enable_sdio_irq,
	.get_cd			= mmc_gpio_get_cd,
	.get_ro			= mmc_gpio_get_ro,
};

static int meson_mx_mmc_slot_probe(struct device *slot_dev,
				   struct meson_mx_mmc_host *host)
{
	struct meson_mx_mmc_slot *slot;
	struct mmc_host *mmc;
	unsigned int id;
	int ret;

	if (of_property_read_u32(slot_dev->of_node, "reg", &id)) {
		dev_err(slot_dev, "missing 'reg' property\n");
		return -EINVAL;
	}

	if (id >= MESON_MX_SDIO_MAX_SLOTS) {
		dev_err(slot_dev, "invalid 'reg' property value %d\n", id);
		return -EINVAL;
	}

	mmc = mmc_alloc_host(sizeof(*slot), slot_dev);
	if (!mmc)
		return -ENOMEM;

	slot = mmc_priv(mmc);
	slot->mmc = mmc;
	slot->id = id;
	slot->host = host;

	host->slots[id] = slot;
	host->slot_devices[id] = slot_dev;

	/* Get regulators and the supported OCR mask */
	ret = mmc_regulator_get_supply(mmc);
	if (ret == -EPROBE_DEFER)
		goto error_free_host;

	mmc->max_req_size = MESON_MX_SDIO_BOUNCE_REQ_SIZE;
	mmc->max_seg_size = mmc->max_req_size;
	mmc->max_blk_count =
		FIELD_GET(MESON_MX_SDIO_SEND_REPEAT_PACKAGE_TIMES_MASK,
			  0xffffffff);
	mmc->max_blk_size = FIELD_GET(MESON_MX_SDIO_EXT_DATA_RW_NUMBER_MASK,
				      0xffffffff);
	mmc->max_blk_size -= (4 * MESON_MX_SDIO_RESPONSE_CRC16_BITS);
	mmc->max_blk_size /= BITS_PER_BYTE;

	/* Get the min and max supported clock rates */
	mmc->f_min = clk_round_rate(host->cfg_div_clk, 1);
	mmc->f_max = clk_round_rate(host->cfg_div_clk,
				    clk_get_rate(host->parent_clk));

	mmc->caps |= MMC_CAP_CMD23;
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
	return ret;
}

static int meson_mx_mmc_probe_slots(struct meson_mx_mmc_host *host)
{
	struct device_node *slot_node;
	struct platform_device *slot_pdev;
	int ret;

	for_each_child_of_node(host->dev->of_node, slot_node) {
		if (!of_device_is_compatible(slot_node, "mmc-slot"))
			continue;

		slot_pdev = of_platform_device_create(slot_node, NULL,
						      host->dev);
		if (!slot_pdev)
			continue;

		ret = meson_mx_mmc_slot_probe(&slot_pdev->dev, host);
		if (ret) {
			of_platform_device_destroy(&slot_pdev->dev, NULL);
			return ret;
		}
	}

	return 0;
}

static int meson_mx_mmc_register_clks(struct meson_mx_mmc_host *host)
{
	struct clk_init_data init;
	const char *clk_div_parent, *clk_fixed_factor_parent;

	clk_fixed_factor_parent = __clk_get_name(host->parent_clk);
	init.name = devm_kasprintf(host->dev, GFP_KERNEL, "%s#fixed_factor",
				   dev_name(host->dev));
	init.ops = &clk_fixed_factor_ops;
	init.flags = 0;
	init.parent_names = &clk_fixed_factor_parent;
	init.num_parents = 1;
	host->fixed_factor.div = 2;
	host->fixed_factor.mult = 1;
	host->fixed_factor.hw.init = &init;

	host->fixed_factor_clk = devm_clk_register(host->dev,
						 &host->fixed_factor.hw);
	if (WARN_ON(PTR_ERR_OR_ZERO(host->fixed_factor_clk)))
		return PTR_ERR(host->fixed_factor_clk);

	clk_div_parent = __clk_get_name(host->fixed_factor_clk);
	init.name = devm_kasprintf(host->dev, GFP_KERNEL, "%s#div",
				   dev_name(host->dev));
	init.ops = &clk_divider_ops;
	init.flags = CLK_SET_RATE_PARENT;
	init.parent_names = &clk_div_parent;
	init.num_parents = 1;
	host->cfg_div.reg = host->base + MESON_MX_SDIO_CONF;
	host->cfg_div.shift = MESON_MX_SDIO_CONF_CMD_CLK_DIV_SHIFT;
	host->cfg_div.width = MESON_MX_SDIO_CONF_CMD_CLK_DIV_WIDTH;
	host->cfg_div.hw.init = &init;
	host->cfg_div.flags = CLK_DIVIDER_ALLOW_ZERO;

	host->cfg_div_clk = devm_clk_register(host->dev, &host->cfg_div.hw);
	if (WARN_ON(PTR_ERR_OR_ZERO(host->cfg_div_clk)))
		return PTR_ERR(host->fixed_factor_clk);

	return 0;
}

static int meson_mx_mmc_probe(struct platform_device *pdev)
{
	struct meson_mx_mmc_host *host;
	struct resource *res;
	int ret, irq;
	u32 conf;

	host = devm_kzalloc(&pdev->dev, sizeof(*host), GFP_KERNEL);
	if (!host)
		return -ENOMEM;

	spin_lock_init(&host->lock);
	spin_lock_init(&host->irq_lock);
	setup_timer(&host->cmd_timeout, meson_mx_mmc_timeout,
		    (unsigned long)host);
	INIT_LIST_HEAD(&host->queue);
	host->dev = &pdev->dev;

	platform_set_drvdata(pdev, host);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	host->base = devm_ioremap_resource(host->dev, res);
	if (IS_ERR(host->base)) {
		ret = PTR_ERR(host->base);
		goto error_out;
	}

	irq = platform_get_irq(pdev, 0);
	ret = devm_request_threaded_irq(host->dev, irq, meson_mx_mmc_irq,
					meson_mx_mmc_irq_thread, 0, NULL,
					host);
	if (ret)
		goto error_out;

	host->core_clk = devm_clk_get(host->dev, "core");
	if (IS_ERR(host->core_clk)) {
		ret = PTR_ERR(host->core_clk);
		goto error_out;
	}

	host->parent_clk = devm_clk_get(host->dev, "clkin");
	if (IS_ERR(host->parent_clk)) {
		ret = PTR_ERR(host->parent_clk);
		goto error_out;
	}

	ret = meson_mx_mmc_register_clks(host);
	if (ret)
		goto error_out;

	ret = clk_prepare_enable(host->core_clk);
	if (ret) {
		dev_err(host->dev, "Failed to enable core clock\n");
		goto error_out;
	}

	ret = clk_prepare_enable(host->cfg_div_clk);
	if (ret) {
		dev_err(host->dev, "Failed to enable MMC clock\n");
		goto error_disable_core_clk;
	}

	conf = 0;
	conf |= FIELD_PREP(MESON_MX_SDIO_CONF_CMD_ARGUMENT_BITS_MASK, 39);
	conf |= FIELD_PREP(MESON_MX_SDIO_CONF_M_ENDIAN_MASK, 0x3);
	conf |= FIELD_PREP(MESON_MX_SDIO_CONF_WRITE_NWR_MASK, 0x2);
	conf |= FIELD_PREP(MESON_MX_SDIO_CONF_WRITE_CRC_OK_STATUS_MASK, 0x2);
	writel(conf, host->base + MESON_MX_SDIO_CONF);

	meson_mx_mmc_soft_reset(host);

	ret = meson_mx_mmc_probe_slots(host);
	if (ret)
		goto error_disable_clks;

	return 0;

error_disable_clks:
	clk_disable_unprepare(host->cfg_div_clk);
error_disable_core_clk:
	clk_disable_unprepare(host->core_clk);
error_out:
	return ret;
}

static int meson_mx_mmc_remove(struct platform_device *pdev)
{
	struct meson_mx_mmc_host *host = platform_get_drvdata(pdev);
	struct meson_mx_mmc_slot *slot;
	int i;

	del_timer_sync(&host->cmd_timeout);

	host->status = MESON_MX_MMC_STATUS_SHUTTING_DOWN;

	for (i = 0; i < MESON_MX_SDIO_MAX_SLOTS; i++) {
		slot = host->slots[i];

		if (!slot->mmc)
			continue;

		mmc_remove_host(slot->mmc);
		mmc_free_host(slot->mmc);
	}

	for (i = 0; i < MESON_MX_SDIO_MAX_SLOTS; i++) {
		if (!host->slot_devices[i])
			continue;

		of_platform_device_destroy(host->slot_devices[i], NULL);
	}

	clk_disable_unprepare(host->cfg_div_clk);
	clk_disable_unprepare(host->core_clk);

	return 0;
}

static const struct of_device_id meson_mx_mmc_of_match[] = {
	{ .compatible = "amlogic,meson8-sdio", },
	{ .compatible = "amlogic,meson8b-sdio", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, meson_mx_mmc_of_match);

static struct platform_driver meson_mx_mmc_driver = {
	.probe   = meson_mx_mmc_probe,
	.remove  = meson_mx_mmc_remove,
	.driver  = {
		.name = "meson-mx-sdio",
		.of_match_table = of_match_ptr(meson_mx_mmc_of_match),
	},
};

module_platform_driver(meson_mx_mmc_driver);

MODULE_DESCRIPTION("Meson6, Meson8 and Meson8b SDIO/MMC Host Driver");
MODULE_AUTHOR("Carlo Caione <carlo@endlessm.com>");
MODULE_LICENSE("GPL v2");
