// SPDX-License-Identifier: GPL-2.0+
/*
 * Amlogic Meson6/Meson8/Meson8b/Meson8m2 SDHC MMC host controller driver.
 *
 * Copyright (C) 2019 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/iopoll.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/timer.h>
#include <linux/types.h>

#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/slot-gpio.h>

#define MESON_SDHC_ARGU						0x00

#define MESON_SDHC_SEND						0x04
	#define MESON_SDHC_SEND_CMD_INDEX			GENMASK(5, 0)
	#define MESON_SDHC_SEND_CMD_HAS_RESP			BIT(6)
	#define MESON_SDHC_SEND_CMD_HAS_DATA			BIT(7)
	#define MESON_SDHC_SEND_RESP_LEN			BIT(8)
	#define MESON_SDHC_SEND_RESP_NO_CRC			BIT(9)
	#define MESON_SDHC_SEND_DATA_DIR			BIT(10)
	#define MESON_SDHC_SEND_DATA_STOP			BIT(11)
	#define MESON_SDHC_SEND_R1B				BIT(12)
	#define MESON_SDHC_SEND_TOTAL_PACK			GENMASK(31, 16)

#define MESON_SDHC_CTRL						0x08
	#define MESON_SDHC_CTRL_DAT_TYPE			GENMASK(1, 0)
	#define MESON_SDHC_CTRL_DDR_MODE			BIT(2)
	#define MESON_SDHC_CTRL_TX_CRC_NOCHECK			BIT(3)
	#define MESON_SDHC_CTRL_PACK_LEN			GENMASK(12, 4)
	#define MESON_SDHC_CTRL_RX_TIMEOUT			GENMASK(19, 13)
	#define MESON_SDHC_CTRL_RX_PERIOD			GENMASK(23, 20)
	#define MESON_SDHC_CTRL_RX_ENDIAN			GENMASK(26, 24)
	#define MESON_SDHC_CTRL_SDIO_IRQ_MODE			BIT(27)
	#define MESON_SDHC_CTRL_DAT0_IRQ_SEL			BIT(28)
	#define MESON_SDHC_CTRL_TX_ENDIAN			GENMASK(31, 29)

#define MESON_SDHC_STAT						0x0c
	#define MESON_SDHC_STAT_CMD_BUSY			BIT(0)
	#define MESON_SDHC_STAT_DAT3_0				GENMASK(4, 1)
	#define MESON_SDHC_STAT_CMD				BIT(5)
	#define MESON_SDHC_STAT_RXFIFO_CNT			GENMASK(12, 6)
	#define MESON_SDHC_STAT_TXFIFO_CNT			GENMASK(19, 13)
	#define MESON_SDHC_STAT_DAT7_4				GENMASK(23, 20)

#define MESON_SDHC_CLKC						0x10
	#define MESON_SDHC_CLKC_CLK_DIV				GENMASK(11, 0)
	#define MESON_SDHC_CLKC_TX_CLK_ON			BIT(12)
	#define MESON_SDHC_CLKC_RX_CLK_ON			BIT(13)
	#define MESON_SDHC_CLKC_SD_CLK_ON			BIT(14)
	#define MESON_SDHC_CLKC_MOD_CLK_ON			BIT(15)
	#define MESON_SDHC_CLKC_CLK_SRC_SEL			GENMASK(18, 16)
	#define MESON_SDHC_CLKC_CLK_JIC				BIT(24)
	#define MESON_SDHC_CLKC_MEM_PWR_OFF			GENMASK(26, 25)

#define MESON_SDHC_ADDR						0x14

#define MESON_SDHC_PDMA						0x18
	#define MESON_SDHC_PDMA_DMA_MODE			BIT(0)
	#define MESON_SDHC_PDMA_PIO_RDRESP			GENMASK(3, 1)
	#define MESON_SDHC_PDMA_DMA_URGENT			BIT(4)
	#define MESON_SDHC_PDMA_WR_BURST			GENMASK(9, 5)
	#define MESON_SDHC_PDMA_RD_BURST			GENMASK(14, 10)
	#define MESON_SDHC_PDMA_RXFIFO_TH			GENMASK(21, 15)
	#define MESON_SDHC_PDMA_TXFIFO_TH			GENMASK(28, 22)
	#define MESON_SDHC_PDMA_RXFIFO_MANUAL_FLUSH		GENMASK(30, 29)
	#define MESON_SDHC_PDMA_TXFIFO_FILL			BIT(31)

#define MESON_SDHC_MISC						0x1c
	#define MESON_SDHC_MISC_WCRC_ERR_PATT			GENMASK(6, 4)
	#define MESON_SDHC_MISC_WCRC_OK_PATT			GENMASK(9, 7)
	#define MESON_SDHC_MISC_BURST_NUM			GENMASK(21, 16)
	#define MESON_SDHC_MISC_THREAD_ID			GENMASK(27, 22)
	#define MESON_SDHC_MISC_MANUAL_STOP			BIT(28)
	#define MESON_SDHC_MISC_TXSTART_THRES			GENMASK(31, 29)

#define MESON_SDHC_DATA						0x20

#define MESON_SDHC_ICTL						0x24
	#define MESON_SDHC_ICTL_RESP_OK				BIT(0)
	#define MESON_SDHC_ICTL_RESP_TIMEOUT			BIT(1)
	#define MESON_SDHC_ICTL_RESP_ERR_CRC			BIT(2)
	#define MESON_SDHC_ICTL_RESP_OK_NOCLEAR			BIT(3)
	#define MESON_SDHC_ICTL_DATA_1PACK_OK			BIT(4)
	#define MESON_SDHC_ICTL_DATA_TIMEOUT			BIT(5)
	#define MESON_SDHC_ICTL_DATA_ERR_CRC			BIT(6)
	#define MESON_SDHC_ICTL_DATA_XFER_OK			BIT(7)
	#define MESON_SDHC_ICTL_RX_HIGHER			BIT(8)
	#define MESON_SDHC_ICTL_RX_LOWER			BIT(9)
	#define MESON_SDHC_ICTL_DAT1_IRQ			BIT(10)
	#define MESON_SDHC_ICTL_DMA_DONE			BIT(11)
	#define MESON_SDHC_ICTL_RXFIFO_FULL			BIT(12)
	#define MESON_SDHC_ICTL_TXFIFO_EMPTY			BIT(13)
	#define MESON_SDHC_ICTL_ADDI_DAT1_IRQ			BIT(14)
	#define MESON_SDHC_ICTL_ALL_IRQS			GENMASK(14, 0)
	#define MESON_SDHC_ICTL_DAT1_IRQ_DELAY			GENMASK(17, 16)

#define MESON_SDHC_ISTA						0x28
	#define MESON_SDHC_ISTA_RESP_OK				BIT(0)
	#define MESON_SDHC_ISTA_RESP_TIMEOUT			BIT(1)
	#define MESON_SDHC_ISTA_RESP_ERR_CRC			BIT(2)
	#define MESON_SDHC_ISTA_RESP_OK_NOCLEAR			BIT(3)
	#define MESON_SDHC_ISTA_DATA_1PACK_OK			BIT(4)
	#define MESON_SDHC_ISTA_DATA_TIMEOUT			BIT(5)
	#define MESON_SDHC_ISTA_DATA_ERR_CRC			BIT(6)
	#define MESON_SDHC_ISTA_DATA_XFER_OK			BIT(7)
	#define MESON_SDHC_ISTA_RX_HIGHER			BIT(8)
	#define MESON_SDHC_ISTA_RX_LOWER			BIT(9)
	#define MESON_SDHC_ISTA_DAT1_IRQ			BIT(10)
	#define MESON_SDHC_ISTA_DMA_DONE			BIT(11)
	#define MESON_SDHC_ISTA_RXFIFO_FULL			BIT(12)
	#define MESON_SDHC_ISTA_TXFIFO_EMPTY			BIT(13)
	#define MESON_SDHC_ISTA_ADDI_DAT1_IRQ			BIT(14)
	#define MESON_SDHC_ISTA_ALL_IRQS			GENMASK(14, 0)

#define MESON_SDHC_SRST						0x2c
	#define MESON_SDHC_SRST_MAIN_CTRL			BIT(0)
	#define MESON_SDHC_SRST_RXFIFO				BIT(1)
	#define MESON_SDHC_SRST_TXFIFO				BIT(2)
	#define MESON_SDHC_SRST_DPHY_RX				BIT(3)
	#define MESON_SDHC_SRST_DPHY_TX				BIT(4)
	#define MESON_SDHC_SRST_DMA_IF				BIT(5)

#define MESON_SDHC_ESTA						0x30

#define MESON_SDHC_ENHC						0x34
	#define MESON_SDHC_ENHC_MESON8M2_WRRSP_MODE		BIT(0)
	#define MESON_SDHC_ENHC_MESON8M2_CHK_WRRSP		BIT(1)
	#define MESON_SDHC_ENHC_MESON8M2_CHK_DMA		BIT(2)
	#define MESON_SDHC_ENHC_MESON8M2_DEBUG			GENMASK(5, 3)
	#define MESON_SDHC_ENHC_MESON6_RX_TIMEOUT		GENMASK(7, 0)
	#define MESON_SDHC_ENHC_MESON6_DMA_RD_RESP		BIT(16)
	#define MESON_SDHC_ENHC_MESON6_DMA_WR_RESP		BIT(17)
	#define MESON_SDHC_ENHC_SDIO_IRQ_PERIOD			GENMASK(15, 8)
	#define MESON_SDHC_ENHC_RXFIFO_TH			GENMASK(24, 18)
	#define MESON_SDHC_ENHC_TXFIFO_TH			GENMASK(31, 25)

#define MESON_SDHC_CLK2						0x38
	#define MESON_SDHC_CLK2_RX_CLK_PHASE			GENMASK(11, 0)
	#define MESON_SDHC_CLK2_SD_CLK_PHASE			GENMASK(23, 12)

#define MESON_SDHC_PARENT_CLKS					4
#define MESON_SDHC_BOUNCE_REQ_SIZE				(128 * 1024)
#define MESON_SDHC_NUM_TUNING_TRIES				10

#define to_meson_mx_sdhc_phase_clk(_hw)				\
			container_of(_hw, struct meson_mx_sdhc_phase_clk, hw)

struct meson_mx_sdhc_data {
	void (*init_hw)(struct mmc_host *mmc);
	void (*set_pdma)(struct mmc_host *mmc);
	bool hardware_flush_all_cmds;
};

struct meson_mx_sdhc_phase_clk {
	struct clk_hw			hw;
	void __iomem			*reg;
	u32				mask;
	int				value_offset;
};

struct meson_mx_sdhc_host {
	struct mmc_host			*mmc;

	struct mmc_request		*mrq;
	struct mmc_command		*cmd;
	int				error;

	void __iomem			*base;

	spinlock_t			irq_lock;

	struct clk_divider		clkc_clk_div;
	struct clk_gate			clkc_tx_clk_on;
	struct clk_gate			clkc_rx_clk_on;
	struct clk_gate			clkc_sd_clk_on;
	struct clk_gate			clkc_mod_clk_on;
	struct clk_mux			clkc_clk_src_sel;
	struct meson_mx_sdhc_phase_clk	clk2_rx_clk_phase;
	struct meson_mx_sdhc_phase_clk	clk2_sd_clk_phase;

	struct clk_bulk_data		parent_clks[MESON_SDHC_PARENT_CLKS];
	struct clk			*pclk;

	struct clk			*tx_clk;
	struct clk			*rx_clk;
	struct clk			*sd_clk;
	struct clk			*mod_clk;
	bool				clocks_enabled;
	unsigned long			current_clk_rate;

	const struct meson_mx_sdhc_data	*platform;
};

static void meson_mx_sdhc_mask_bits(struct mmc_host *mmc, u8 reg, u32 mask,
				   u32 val)
{
	struct meson_mx_sdhc_host *host = mmc_priv(mmc);
	u32 regval;

	regval = readl(host->base + reg);
	regval &= ~mask;
	regval |= (val & mask);

	writel(regval, host->base + reg);
}

void meson_mx_sdhc_reset(struct mmc_host *mmc)
{
	struct meson_mx_sdhc_host *host = mmc_priv(mmc);

	writel(MESON_SDHC_SRST_MAIN_CTRL |  MESON_SDHC_SRST_RXFIFO |
	       MESON_SDHC_SRST_TXFIFO | MESON_SDHC_SRST_DPHY_RX |
	       MESON_SDHC_SRST_DPHY_TX | MESON_SDHC_SRST_DMA_IF,
	       host->base + MESON_SDHC_SRST);
	udelay(5);

	writel(0, host->base + MESON_SDHC_SRST);
	udelay(10);
}

static struct mmc_command *meson_mx_sdhc_get_next_cmd(struct mmc_command *cmd)
{
	if (cmd->opcode == MMC_SET_BLOCK_COUNT && !cmd->error)
		return cmd->mrq->cmd;
	else if (mmc_op_multi(cmd->opcode) &&
		 (!cmd->mrq->sbc || cmd->error || cmd->data->error))
		return cmd->mrq->stop;
	else
		return NULL;
}

static void meson_mx_sdhc_wait_cmd_ready(struct mmc_host *mmc)
{
	struct meson_mx_sdhc_host *host = mmc_priv(mmc);
	u32 stat, esta;
	int ret;

	ret = readl_poll_timeout(host->base + MESON_SDHC_STAT, stat,
				 !(stat & MESON_SDHC_STAT_CMD_BUSY), 1, 10000);
	if (ret)
		dev_warn(mmc_dev(mmc), "Failed to poll for CMD_BUSY\n");

	ret = readl_poll_timeout(host->base + MESON_SDHC_ESTA, esta,
				 !(esta & BIT(11)), 1, 10000);
	if (ret)
		dev_warn(mmc_dev(mmc), "Failed to poll for ESTA[11]\n");

	if ((stat & MESON_SDHC_STAT_CMD_BUSY) || (esta & BIT(11)))
		meson_mx_sdhc_reset(mmc);
}

static void meson_mx_sdhc_start_cmd(struct mmc_host *mmc,
				    struct mmc_command *cmd)
{
	struct meson_mx_sdhc_host *host = mmc_priv(mmc);
	unsigned long irqflags;
	u32 ictl, send;
	int pack_len;

	host->cmd = cmd;

	ictl = MESON_SDHC_ICTL_DATA_TIMEOUT | MESON_SDHC_ICTL_DATA_ERR_CRC |
	       MESON_SDHC_ICTL_RXFIFO_FULL | MESON_SDHC_ICTL_TXFIFO_EMPTY |
	       MESON_SDHC_ICTL_RESP_TIMEOUT | MESON_SDHC_ICTL_RESP_ERR_CRC;

	send = FIELD_PREP(MESON_SDHC_SEND_CMD_INDEX, cmd->opcode);

	if (cmd->data) {
#if 0
        vstat = readl(host->base + SDHC_STAT);
        if(stat->txfifo_cnt || stat->rxfifo_cnt){
            // sdhc_err("cmd%d: txfifo_cnt:%d, rxfifo_cnt:%d\n", 
                // mrq->cmd->opcode, stat->txfifo_cnt, stat->rxfifo_cnt);

            if(aml_sdhc_wait_ready(host, STAT_POLL_TIMEOUT)){ /*Wait command busy*/
            	sdhc_err("aml_sdhc_wait_ready error before start cmd fifo\n");
            }
            vsrst = readl(host->base + SDHC_SRST);        
            srst->rxfifo = 1;
            srst->txfifo = 1;
            srst->main_ctrl = 1;
            writel(vsrst, host->base+SDHC_SRST);
            udelay(5);
            writel(vsrst, host->base+SDHC_SRST);
        }
        vstat = readl(host->base + SDHC_STAT);
        if(stat->txfifo_cnt || stat->rxfifo_cnt){
            sdhc_err("FAIL to clear FIFO, cmd%d: txfifo_cnt:%d, rxfifo_cnt:%d\n", 
                mrq->cmd->opcode, stat->txfifo_cnt, stat->rxfifo_cnt);
        }
#endif

		send |= MESON_SDHC_SEND_CMD_HAS_DATA;
		send |= FIELD_PREP(MESON_SDHC_SEND_TOTAL_PACK,
				   cmd->data->blocks - 1);

		if (cmd->data->blksz < 512)
			pack_len = cmd->data->blksz;
		else
			pack_len = 0;

		if (cmd->data->flags & MMC_DATA_WRITE)
			send |= MESON_SDHC_SEND_DATA_DIR;

		/*
		 * If command with no data, just wait response done
		 * interrupt(int[0]), and if command with data transfer, just
		 * wait dma done interrupt(int[11]), don't need care about
		 * dat0 busy or not.
		 */
		if (host->platform->hardware_flush_all_cmds ||
		    cmd->data->flags & MMC_DATA_WRITE)
			ictl |= MESON_SDHC_ICTL_DMA_DONE;
		else
			ictl |= MESON_SDHC_ICTL_DATA_XFER_OK;

		if ((cmd->opcode == SD_IO_RW_DIRECT ||
		     cmd->opcode == SD_IO_RW_EXTENDED) &&
		    cmd->data->blocks > 1)
			meson_mx_sdhc_mask_bits(mmc, MESON_SDHC_MISC,
						MESON_SDHC_MISC_MANUAL_STOP,
						MESON_SDHC_MISC_MANUAL_STOP);
		else
			meson_mx_sdhc_mask_bits(mmc, MESON_SDHC_MISC,
						MESON_SDHC_MISC_MANUAL_STOP,
						0);
	} else {
		ictl |= MESON_SDHC_ICTL_RESP_OK;

		pack_len = 0;
	}

	if (cmd->opcode == MMC_STOP_TRANSMISSION)
		send |= MESON_SDHC_SEND_DATA_STOP;

	if (cmd->flags & MMC_RSP_PRESENT)
		send |= MESON_SDHC_SEND_CMD_HAS_RESP;

	if (cmd->flags & MMC_RSP_136) {
		send |= MESON_SDHC_SEND_RESP_LEN;
		send |= MESON_SDHC_SEND_RESP_NO_CRC; // HACK WTF
	}

#if 0
	if (cmd->flags & MMC_RSP_R1B)
		send |= MESON_SDHC_SEND_R1B;
#endif

	if (!(cmd->flags & MMC_RSP_CRC))
		send |= MESON_SDHC_SEND_RESP_NO_CRC;

	/* enable the new IRQs and mask all pending ones */
	writel(ictl, host->base + MESON_SDHC_ICTL);
	writel(MESON_SDHC_ISTA_ALL_IRQS, host->base + MESON_SDHC_ISTA);

	writel(cmd->arg, host->base + MESON_SDHC_ARGU);

	meson_mx_sdhc_mask_bits(mmc, MESON_SDHC_CTRL, MESON_SDHC_CTRL_PACK_LEN,
				FIELD_PREP(MESON_SDHC_CTRL_PACK_LEN, pack_len));

	if (cmd->data)
		writel(sg_dma_address(cmd->data->sg),
		       host->base + MESON_SDHC_ADDR);

	meson_mx_sdhc_wait_cmd_ready(mmc);

	spin_lock_irqsave(&host->irq_lock, irqflags);

	if (cmd->data)
		host->platform->set_pdma(mmc);

#if 0
    if(!IS_MESON_M8M2_CPU){
	    loop_limit = 100;
	    for (i = 0; i < loop_limit; i++) {
	        vesta = readl(host->base + SDHC_ESTA);
	        if (vesta == 0) {
	            // sdhc_err("ok: %s: cmd%d, SDHC_ESTA=%#x, i=%d\n", 
	                    // mmc_hostname(host->mmc), mrq->cmd->opcode, vesta, i);
	            break;
	        }
	        if (i > 50) {
	            sdhc_err("udelay\n");
	            udelay(1);
	        }
	    }
	    if (i >= loop_limit) {
	        sdhc_err("Warning: %s: cmd%d, SDHC_ESTA=%#x\n", 
	                mmc_hostname(host->mmc), mrq->cmd->opcode, vesta);
	    }

	    if (mrq->data && (mrq->data->flags & MMC_DATA_WRITE)) {
	        for (i = 0; i < loop_limit; i++) {
	            vstat = readl(host->base + SDHC_STAT);
	            if (stat->txfifo_cnt != 0) {
	                // sdhc_err("OK: %s: cmd%d, txfifo_cnt=%d, i=%d\n", 
	                        // mmc_hostname(host->mmc), mrq->cmd->opcode, stat->txfifo_cnt, i);
	                break;
	            }
	            udelay(1);
	        }
	        if (i >= loop_limit) {
	            sdhc_err("Warning: %s: cmd%d, txfifo_cnt=%d\n", 
	                    mmc_hostname(host->mmc), mrq->cmd->opcode, stat->txfifo_cnt);
	        }

	    }
    }
#endif

#if 0
	dev_err(mmc_dev(mmc), "SEND = 0x%08x, PDMA = 0x%08x", send, readl(host->base + MESON_SDHC_PDMA));
	dev_err(mmc_dev(mmc), "STAT = 0x%08x, CTRL = 0x%08x", readl(host->base + MESON_SDHC_STAT), readl(host->base + MESON_SDHC_CTRL));
	dev_err(mmc_dev(mmc), "MISC = 0x%08x, ICTL = 0x%08x", readl(host->base + MESON_SDHC_MISC), ictl);
	dev_err(mmc_dev(mmc), "SRST = 0x%08x, ISTA = 0x%08x", readl(host->base + MESON_SDHC_SRST), readl(host->base + MESON_SDHC_ISTA));
	dev_err(mmc_dev(mmc), "ESTA = 0x%08x, ENHC = 0x%08x", readl(host->base + MESON_SDHC_ESTA), readl(host->base + MESON_SDHC_ENHC));
#endif

	writel(send, host->base + MESON_SDHC_SEND);

	spin_unlock_irqrestore(&host->irq_lock, irqflags);
}

static void meson_mx_sdhc_disable_clks(struct mmc_host *mmc)
{
	struct meson_mx_sdhc_host *host = mmc_priv(mmc);

	if (!host->clocks_enabled)
		return;

	clk_disable_unprepare(host->tx_clk);
	clk_disable_unprepare(host->rx_clk);
	clk_disable_unprepare(host->sd_clk);

	clk_disable_unprepare(host->mod_clk);

	host->clocks_enabled = false;
}

static int meson_mx_sdhc_enable_clks(struct mmc_host *mmc)
{
	struct meson_mx_sdhc_host *host = mmc_priv(mmc);
	struct clk *clocks[] = {
		host->mod_clk,
		host->sd_clk,
		host->tx_clk,
		host->rx_clk,
	};
	int i, ret;

	if (host->clocks_enabled)
		return 0;

	for (i = 0; i < ARRAY_SIZE(clocks); i++) {
		ret = clk_prepare_enable(clocks[i]);
		if (ret) {
			dev_err(mmc_dev(mmc), "Failed to enable clock %s\n",
				__clk_get_name(clocks[i]));
			goto err;
		}
	}

	host->clocks_enabled = true;

	return 0;

err:
	while (--i >= 0)
		clk_disable_unprepare(clocks[i]);

	return ret;
}

static int meson_mx_sdhc_set_clk(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct meson_mx_sdhc_host *host = mmc_priv(mmc);
	int ret, rx_clk_phase;

	if (ios->clock == host->current_clk_rate)
		return 0;

	meson_mx_sdhc_disable_clks(mmc);

	if (ios->clock) {
		ret = clk_set_rate(host->sd_clk, ios->clock);
		if (ret) {
			dev_warn(mmc_dev(mmc),
				"Failed to set MMC clock to %uHz: %d\n",
				ios->clock, host->error);
			return ret;
		}

		ret = meson_mx_sdhc_enable_clks(mmc);
		if (ret)
			return ret;

		mmc->actual_clock = clk_get_rate(host->sd_clk);

		if (mmc->actual_clock > 100000000) {
			rx_clk_phase = 1;
		} else if (mmc->actual_clock > 45000000) {
			if (ios->signal_voltage == MMC_SIGNAL_VOLTAGE_330)
				rx_clk_phase = 15;
			else
				rx_clk_phase = 11;
		} else if (mmc->actual_clock >= 25000000) {
			rx_clk_phase = 15;
		} else if (mmc->actual_clock > 5000000) {
			rx_clk_phase = 23;
		} else if (mmc->actual_clock > 1000000) {
			rx_clk_phase = 55;
		} else {
			rx_clk_phase = 1061;
		}

		ret = clk_set_phase(host->rx_clk, rx_clk_phase);
		if (ret) {
			dev_warn(mmc_dev(mmc),
				 "Failed to set RX clock phase to %d deg\n",
				 rx_clk_phase);
			return ret;
		}
	} else {
		mmc->actual_clock = 0;
	}

	host->current_clk_rate = ios->clock;

	return 0;
}

static void meson_mx_sdhc_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct meson_mx_sdhc_host *host = mmc_priv(mmc);
	unsigned short vdd = ios->vdd;
	u32 dat_type;

#if 0
		dev_err(mmc_dev(mmc), "%s(%u): CLKC = 0x%08x, CLK2 = 0x%08x\n", __func__, ios->clock, readl(host->base + MESON_SDHC_CLKC), readl(host->base + MESON_SDHC_CLK2));
#endif

	// TODO: vqmmc
	switch (ios->power_mode) {
	case MMC_POWER_OFF:
		vdd = 0;
		/* fall through */
	case MMC_POWER_UP:
		if (!IS_ERR(mmc->supply.vmmc)) {
			host->error = mmc_regulator_set_ocr(mmc,
							    mmc->supply.vmmc,
							    vdd);
			if (host->error)
				return;
		}
		break;
	}

	host->error = meson_mx_sdhc_set_clk(mmc, ios);
	if (host->error)
		return;

	switch (ios->bus_width) {
	case MMC_BUS_WIDTH_1:
		dat_type = FIELD_PREP(MESON_SDHC_CTRL_DAT_TYPE, 0);
		break;

	case MMC_BUS_WIDTH_4:
		dat_type = FIELD_PREP(MESON_SDHC_CTRL_DAT_TYPE, 1);
		break;

	case MMC_BUS_WIDTH_8:
		dat_type = FIELD_PREP(MESON_SDHC_CTRL_DAT_TYPE, 2);
		break;

	default:
		dev_err(mmc_dev(mmc), "unsupported bus width: %d\n",
			ios->bus_width);
		host->error = -EINVAL;
		return;
	}

	meson_mx_sdhc_mask_bits(mmc, MESON_SDHC_CTRL, MESON_SDHC_CTRL_DAT_TYPE,
				dat_type);
}

static int meson_mx_sdhc_map_dma(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct mmc_data *data = mrq->data;
	int dma_len;

	if (!data)
		return 0;

	dma_len = dma_map_sg(mmc_dev(mmc), data->sg, data->sg_len,
			     mmc_get_dma_dir(data));
	if (dma_len <= 0) {
		dev_err(mmc_dev(mmc), "dma_map_sg failed\n");
		return -ENOMEM;
	}

	return 0;
}

static void meson_mx_sdhc_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct meson_mx_sdhc_host *host = mmc_priv(mmc);
	struct mmc_command *cmd = mrq->cmd;

	if (!host->error)
		host->error = meson_mx_sdhc_map_dma(mmc, mrq);

	if (host->error) {
		cmd->error = host->error;
		mmc_request_done(mmc, mrq);
		return;
	}

	host->mrq = mrq;

	if (mrq->sbc)
		meson_mx_sdhc_start_cmd(mmc, mrq->sbc);
	else
		meson_mx_sdhc_start_cmd(mmc, mrq->cmd);
}

static int meson_mx_sdhc_card_busy(struct mmc_host *mmc)
{
	struct meson_mx_sdhc_host *host = mmc_priv(mmc);
	u32 stat = readl(host->base + MESON_SDHC_STAT);

	return !!(stat & MESON_SDHC_STAT_DAT3_0);
}

static bool meson_mx_sdhc_tuning_point_matches(struct mmc_host *mmc,
					       u32 opcode)
{
	unsigned int i, num_matches = 0;
	int ret;

	for (i = 0; i < MESON_SDHC_NUM_TUNING_TRIES; i++) {
		ret = mmc_send_tuning(mmc, opcode, NULL);
		if (!ret)
			num_matches++;
	}

	return num_matches == MESON_SDHC_NUM_TUNING_TRIES;
}

static int meson_mx_sdhc_execute_tuning(struct mmc_host *mmc, u32 opcode)
{
	struct meson_mx_sdhc_host *host = mmc_priv(mmc);
	int ret, rx_clk_phase, old_phase, new_phase;
	int div, start, len, best_start, best_len;

	len = 0;
	start = 0;
	best_len = 0;

	old_phase = clk_get_phase(host->rx_clk);

	div = FIELD_GET(MESON_SDHC_CLKC_CLK_DIV,
			readl(host->base + MESON_SDHC_CLKC));

	for (rx_clk_phase = 0; rx_clk_phase <= div; rx_clk_phase++) {
		ret = clk_set_phase(host->rx_clk, rx_clk_phase);
		if (ret) {
			dev_warn(mmc_dev(mmc),
				 "Failed to set RX clock phase to %u deg\n",
				 rx_clk_phase);
			continue;
		}

		if (meson_mx_sdhc_tuning_point_matches(mmc, opcode)) {
			if (!len) {
				start = rx_clk_phase;

				dev_dbg(mmc_dev(mmc),
					"New RX phase window starts at %u\n",
					start);
			}

			len++;
		} else {
			if (len > best_len) {
				best_start = start;
				best_len = len;

				dev_dbg(mmc_dev(mmc),
					"New best RX phase window: %u - %u\n",
					best_start, best_start + best_len);
			}

			/* reset the current window */
			len = 0;
		}
	}

	if (len > best_len)
		/* the last window is the best (or possibly only) window */
		new_phase = start + (len / 2);
	else if (best_len)
		/* there was a better window than the last */
		new_phase = best_start + (best_len / 2);
	else
		/* no window was found at all, reset to the original phase */
		new_phase = old_phase;

	ret = clk_set_phase(host->rx_clk, rx_clk_phase);
	if (ret) {
		dev_warn(mmc_dev(mmc),
			 "Failed to set RX clock phase to %u deg\n",
		         new_phase);
		return ret;
	}

	if (!len && !best_len)
		return -EIO;

	dev_dbg(mmc_dev(mmc), "Tuned RX clock phase to %u deg\n", new_phase);

	return 0;
}

static int meson_mx_sdhc_signal_voltage_switch(struct mmc_host *mmc,
					       struct mmc_ios *ios)
{
	/* vqmmc regulator is available */
	if (!IS_ERR(mmc->supply.vqmmc)) {
		/*
		 * The usual amlogic setup uses a GPIO to switch from one
		 * regulator to the other. While the voltage ramp up is
		 * pretty fast, care must be taken when switching from 3.3v
		 * to 1.8v. Please make sure the regulator framework is aware
		 * of your own regulator constraints
		 */
		return mmc_regulator_set_vqmmc(mmc, ios);
	}

	/* no vqmmc regulator, assume fixed regulator at 3/3.3V */
	if (ios->signal_voltage == MMC_SIGNAL_VOLTAGE_330)
		return 0;

	return -EINVAL;
}

static struct mmc_host_ops meson_mx_sdhc_ops = {
	.request			= meson_mx_sdhc_request,
	.set_ios			= meson_mx_sdhc_set_ios,
// HACK	.card_busy			= meson_mx_sdhc_card_busy,
	.execute_tuning			= meson_mx_sdhc_execute_tuning,
	.start_signal_voltage_switch	= meson_mx_sdhc_signal_voltage_switch,
	.get_cd				= mmc_gpio_get_cd,
	.get_ro				= mmc_gpio_get_ro,
};

static void meson_mx_sdhc_request_done(struct meson_mx_sdhc_host *host)
{
	struct mmc_request *mrq = host->mrq;
	struct mmc_host *mmc = host->mmc;

	host->mrq = NULL;
	host->cmd = NULL;

	meson_mx_sdhc_mask_bits(mmc, MESON_SDHC_ICTL,
				MESON_SDHC_ICTL_ALL_IRQS, 0);
	meson_mx_sdhc_mask_bits(mmc, MESON_SDHC_ISTA, MESON_SDHC_ISTA_ALL_IRQS,
				MESON_SDHC_ISTA_ALL_IRQS);

	mmc_request_done(host->mmc, mrq);
}

static u32 meson_mx_sdhc_read_response(struct meson_mx_sdhc_host *host, u8 idx)
{
	meson_mx_sdhc_mask_bits(host->mmc, MESON_SDHC_PDMA,
				MESON_SDHC_PDMA_DMA_MODE, 0);

	meson_mx_sdhc_mask_bits(host->mmc, MESON_SDHC_PDMA,
				MESON_SDHC_PDMA_PIO_RDRESP,
				FIELD_PREP(MESON_SDHC_PDMA_PIO_RDRESP, idx));

	return readl(host->base + MESON_SDHC_ARGU);
}

static irqreturn_t meson_mx_sdhc_irq(int irq, void *data)
{
	struct meson_mx_sdhc_host *host = data;
	unsigned long irqflags;
	u32 ictl, ista;

	spin_lock_irqsave(&host->irq_lock, irqflags);

	ictl = readl(host->base + MESON_SDHC_ICTL);
	ista = readl(host->base + MESON_SDHC_ISTA);

	spin_unlock_irqrestore(&host->irq_lock, irqflags);
#if 0
	printk("ICTL = 0x%08x, ISTA = 0x%08x, STAT = 0x%08x\n", ictl, ista, readl(host->base + MESON_SDHC_STAT));
#endif
	if (!(ictl & ista))
		return IRQ_NONE;

	if (ista & MESON_SDHC_ISTA_RXFIFO_FULL ||
		ista & MESON_SDHC_ISTA_TXFIFO_EMPTY)
		host->cmd->error = -EIO;
	else if (ista & MESON_SDHC_ISTA_RESP_ERR_CRC ||
		 ista & MESON_SDHC_ISTA_DATA_ERR_CRC)
		host->cmd->error = -EILSEQ;
	else if (ista & MESON_SDHC_ISTA_RESP_TIMEOUT ||
		 ista & MESON_SDHC_ISTA_DATA_TIMEOUT)
		host->cmd->error = -ETIMEDOUT;

	return IRQ_WAKE_THREAD;
}

static irqreturn_t meson_mx_sdhc_irq_thread(int irq, void *irq_data)
{
	struct meson_mx_sdhc_host *host = irq_data;
	struct mmc_command *cmd, *next_cmd;
	unsigned long irqflags;
	u32 pdma;

	cmd = host->cmd;
	if (WARN_ON(!cmd))
		return IRQ_HANDLED;

	meson_mx_sdhc_wait_cmd_ready(host->mmc);

	spin_lock_irqsave(&host->irq_lock, irqflags);

	if (cmd->flags & MMC_RSP_136) {
		cmd->resp[0] = meson_mx_sdhc_read_response(host, 4);
		cmd->resp[1] = meson_mx_sdhc_read_response(host, 3);
		cmd->resp[2] = meson_mx_sdhc_read_response(host, 2);
		cmd->resp[3] = meson_mx_sdhc_read_response(host, 1);
	} else {
		cmd->resp[0] = meson_mx_sdhc_read_response(host, 0);
	}

	spin_unlock_irqrestore(&host->irq_lock, irqflags);

	if (cmd->error == -EIO || cmd->error == -ETIMEDOUT) {
		meson_mx_sdhc_reset(host->mmc);
	} else if (cmd->error == -EILSEQ) {
		mmc_retune_needed(host->mmc);
	} else if (cmd->data) {
		if (!host->platform->hardware_flush_all_cmds &&
		    cmd->flags & MMC_DATA_READ) {
			pdma = readl(host->base + MESON_SDHC_PDMA);
			pdma |= FIELD_PREP(MESON_SDHC_PDMA_RXFIFO_MANUAL_FLUSH,
					   2);
			writel(pdma, host->base);
		}

		dma_unmap_sg(mmc_dev(host->mmc), cmd->data->sg,
			     cmd->data->sg_len, mmc_get_dma_dir(cmd->data));

		cmd->data->bytes_xfered = cmd->data->blksz * cmd->data->blocks;
	}

	next_cmd = meson_mx_sdhc_get_next_cmd(cmd);
	if (next_cmd)
		meson_mx_sdhc_start_cmd(host->mmc, next_cmd);
	else
		meson_mx_sdhc_request_done(host);

	return IRQ_HANDLED;
}

static int meson_mx_sdhc_clk_get_phase(struct clk_hw *hw)
{
	struct meson_mx_sdhc_phase_clk *phase_clk =
		to_meson_mx_sdhc_phase_clk(hw);
	u32 val;

	val = readl(phase_clk->reg) & phase_clk->mask;
	val >>= __ffs(phase_clk->mask);

	return phase_clk->value_offset + val;
}

static int meson_mx_sdhc_clk_set_phase(struct clk_hw *hw, int degrees)
{
	struct meson_mx_sdhc_phase_clk *phase_clk =
		to_meson_mx_sdhc_phase_clk(hw);
	u32 reg_val, phase_val;

	phase_val = degrees - phase_clk->value_offset;
	phase_val <<= __ffs(phase_clk->mask);

	if (phase_val > phase_clk->mask)
		return -EINVAL;

	reg_val = readl(phase_clk->reg);
	reg_val &= ~phase_clk->mask;
	reg_val |= phase_val;
	writel(reg_val, phase_clk->reg);

	return 0;
}

static const struct clk_ops meson_mx_sdhc_phase_clk_ops = {
	.get_phase	= meson_mx_sdhc_clk_get_phase,
	.set_phase	= meson_mx_sdhc_clk_set_phase,
};

static struct clk *meson_mx_sdhc_register_clk(struct device *dev,
					      struct clk_hw *hw,
					      const char *name,
					      int num_parents,
					      struct clk **parents,
					      unsigned long flags,
					      const struct clk_ops *ops)
{
	const char *parent_names[MESON_SDHC_PARENT_CLKS];
	struct clk_init_data init;
	int i;

	for (i = 0; i < num_parents; i++)
		parent_names[i] = __clk_get_name(parents[i]);

	init.name = devm_kasprintf(dev, GFP_KERNEL, "%s#%s", dev_name(dev),
				   name);
	if (!init.name)
		return ERR_PTR(-ENOMEM);

	init.num_parents = num_parents;
	init.parent_names = parent_names;
	init.flags = CLK_SET_RATE_PARENT | flags;
	init.ops = ops;

	hw->init = &init;

	return devm_clk_register(dev, hw);
}

static int meson_mx_sdhc_register_clks(struct meson_mx_sdhc_host *host)
{
	struct clk *mux_parents[MESON_SDHC_PARENT_CLKS];
	struct clk *mux_clk, *div_clk, *sd_clk, *rx_clk;
	struct clk_div_table *div_table;
	int max_divider, i, j;

	for (i = 0; i < MESON_SDHC_PARENT_CLKS; i++)
		mux_parents[i] = host->parent_clks[i].clk;

	max_divider = FIELD_GET(MESON_SDHC_CLKC_CLK_DIV, 0xffffffff);

#if 1
	/* only odd dividers >= 3 (register value >= 2) are supported */
	div_table = devm_kcalloc(mmc_dev(host->mmc), max_divider / 2,
				 sizeof(*div_table), GFP_KERNEL);
	for (i = 0, j = 2; j <= max_divider; i++, j += 2) {
		div_table[i].val = j;
		div_table[i].div = j + 1;
	}
#else
	/* only even dividers >= 2 (register value >= 1) are supported */
	div_table = devm_kcalloc(mmc_dev(host->mmc), max_divider / 2,
				 sizeof(*div_table), GFP_KERNEL);
	for (i = 0, j = 1; j <= max_divider; i++, j += 2) {
		div_table[i].val = j;
		div_table[i].div = j + 1;
	}
#endif

	host->clkc_clk_src_sel.reg = host->base + MESON_SDHC_CLKC;
	host->clkc_clk_src_sel.shift = __ffs(MESON_SDHC_CLKC_CLK_SRC_SEL);
	host->clkc_clk_src_sel.mask = MESON_SDHC_CLKC_CLK_SRC_SEL >>
				      host->clkc_clk_src_sel.shift;
	mux_clk = meson_mx_sdhc_register_clk(mmc_dev(host->mmc),
					     &host->clkc_clk_src_sel.hw,
					     "clk_src_sel",
					     MESON_SDHC_PARENT_CLKS,
					     mux_parents, 0, &clk_mux_ops);
	if (IS_ERR(mux_clk))
		return PTR_ERR(mux_clk);

	host->clkc_clk_div.reg = host->base + MESON_SDHC_CLKC;
	host->clkc_clk_div.shift = __ffs(MESON_SDHC_CLKC_CLK_DIV);
	host->clkc_clk_div.width = fls(MESON_SDHC_CLKC_CLK_DIV) -
				   host->clkc_clk_div.shift;
	host->clkc_clk_div.table = div_table;
	host->clkc_clk_div.flags = CLK_DIVIDER_ALLOW_ZERO;
	div_clk = meson_mx_sdhc_register_clk(mmc_dev(host->mmc),
					     &host->clkc_clk_div.hw,
					     "clk_div", 1, &mux_clk, 
					     CLK_SET_RATE_PARENT,
					     &clk_divider_ops);
	if (IS_ERR(div_clk))
		return PTR_ERR(div_clk);

	host->clkc_mod_clk_on.reg = host->base + MESON_SDHC_CLKC;
	host->clkc_mod_clk_on.bit_idx = __ffs(MESON_SDHC_CLKC_MOD_CLK_ON);
	host->mod_clk = meson_mx_sdhc_register_clk(mmc_dev(host->mmc),
						  &host->clkc_mod_clk_on.hw,
						  "mod_clk_on", 1, &div_clk, 0,
						  &clk_gate_ops);
	if (IS_ERR(host->mod_clk))
		return PTR_ERR(host->mod_clk);

	host->clkc_tx_clk_on.reg = host->base + MESON_SDHC_CLKC;
	host->clkc_tx_clk_on.bit_idx = __ffs(MESON_SDHC_CLKC_TX_CLK_ON);
	host->tx_clk = meson_mx_sdhc_register_clk(mmc_dev(host->mmc),
						  &host->clkc_tx_clk_on.hw,
						  "tx_clk_on", 1, &div_clk,
						  CLK_SET_RATE_GATE |
						  CLK_SET_RATE_PARENT,
						  &clk_gate_ops);
	if (IS_ERR(host->tx_clk))
		return PTR_ERR(host->tx_clk);

	host->clkc_rx_clk_on.reg = host->base + MESON_SDHC_CLKC;
	host->clkc_rx_clk_on.bit_idx = __ffs(MESON_SDHC_CLKC_RX_CLK_ON);
	rx_clk = meson_mx_sdhc_register_clk(mmc_dev(host->mmc),
					    &host->clkc_rx_clk_on.hw,
					    "rx_clk_on", 1, &div_clk,
					    CLK_SET_RATE_GATE |
					    CLK_SET_RATE_PARENT,
					    &clk_gate_ops);
	if (IS_ERR(rx_clk))
		return PTR_ERR(rx_clk);

	host->clkc_sd_clk_on.reg = host->base + MESON_SDHC_CLKC;
	host->clkc_sd_clk_on.bit_idx = __ffs(MESON_SDHC_CLKC_SD_CLK_ON);
	sd_clk = meson_mx_sdhc_register_clk(mmc_dev(host->mmc),
					    &host->clkc_sd_clk_on.hw,
					    "sd_clk_on", 1, &div_clk,
					    CLK_SET_RATE_GATE |
					    CLK_SET_RATE_PARENT,
					    &clk_gate_ops);
	if (IS_ERR(sd_clk))
		return PTR_ERR(sd_clk);

	host->clk2_rx_clk_phase.reg = host->base + MESON_SDHC_CLK2;
	host->clk2_rx_clk_phase.mask = MESON_SDHC_CLK2_RX_CLK_PHASE;
	host->clk2_rx_clk_phase.value_offset = 0;
	host->rx_clk =
		meson_mx_sdhc_register_clk(mmc_dev(host->mmc),
					   &host->clk2_rx_clk_phase.hw,
					   "rx_phase", 1, &rx_clk,
					   CLK_SET_RATE_PARENT,
					   &meson_mx_sdhc_phase_clk_ops);
	if (IS_ERR(host->rx_clk))
		return PTR_ERR(host->rx_clk);

	host->clk2_sd_clk_phase.reg = host->base + MESON_SDHC_CLK2;
	host->clk2_sd_clk_phase.mask = MESON_SDHC_CLK2_SD_CLK_PHASE;
	host->clk2_sd_clk_phase.value_offset = 180;
	host->sd_clk =
		meson_mx_sdhc_register_clk(mmc_dev(host->mmc),
					   &host->clk2_sd_clk_phase.hw,
					   "sd_phase", 1, &sd_clk,
					   CLK_SET_RATE_PARENT,
					   &meson_mx_sdhc_phase_clk_ops);
	if (IS_ERR(host->sd_clk))
		return PTR_ERR(host->sd_clk);

	return 0;
}

static void meson_mx_sdhc_init_hw_meson8(struct mmc_host *mmc)
{
	struct meson_mx_sdhc_host *host = mmc_priv(mmc);
	u32 val;

	val = FIELD_PREP(MESON_SDHC_MISC_TXSTART_THRES, 7) |
	      FIELD_PREP(MESON_SDHC_MISC_WCRC_ERR_PATT, 5) |
	      FIELD_PREP(MESON_SDHC_MISC_WCRC_OK_PATT, 2);
	writel(val, host->base + MESON_SDHC_MISC);

	val = FIELD_PREP(MESON_SDHC_ENHC_RXFIFO_TH, 63) |
	      MESON_SDHC_ENHC_MESON6_DMA_WR_RESP |
	      FIELD_PREP(MESON_SDHC_ENHC_MESON6_RX_TIMEOUT, 255) |
	      FIELD_PREP(MESON_SDHC_ENHC_SDIO_IRQ_PERIOD, 12);
	writel(val, host->base + MESON_SDHC_ENHC);
};

static void meson_mx_sdhc_set_pdma_meson8(struct mmc_host *mmc)
{
	struct meson_mx_sdhc_host *host = mmc_priv(mmc);
	u32 pdma;

	pdma = readl(host->base + MESON_SDHC_PDMA);

	pdma |= MESON_SDHC_PDMA_DMA_MODE;

	if (host->cmd->flags & MMC_DATA_WRITE) {
		pdma &= ~MESON_SDHC_PDMA_RD_BURST;
		pdma |= FIELD_PREP(MESON_SDHC_PDMA_RD_BURST, 31);

		pdma |= MESON_SDHC_PDMA_TXFIFO_FILL;
	} else {
		pdma |= FIELD_PREP(MESON_SDHC_PDMA_RXFIFO_MANUAL_FLUSH, 1);
	}

	writel(pdma, host->base + MESON_SDHC_PDMA);

	if (host->cmd->flags & MMC_DATA_WRITE) {
		pdma &= ~MESON_SDHC_PDMA_RD_BURST;
		pdma |= FIELD_PREP(MESON_SDHC_PDMA_RD_BURST, 15);

		writel(pdma, host->base + MESON_SDHC_PDMA);
	}
}

static void meson_mx_sdhc_init_hw_meson8m2(struct mmc_host *mmc)
{
	struct meson_mx_sdhc_host *host = mmc_priv(mmc);
	u32 val;

	val = FIELD_PREP(MESON_SDHC_MISC_TXSTART_THRES, 6) |
	      FIELD_PREP(MESON_SDHC_MISC_WCRC_ERR_PATT, 5) |
	      FIELD_PREP(MESON_SDHC_MISC_WCRC_OK_PATT, 2);
	writel(val, host->base + MESON_SDHC_MISC);

	val = FIELD_PREP(MESON_SDHC_ENHC_RXFIFO_TH, 64) |
	      FIELD_PREP(MESON_SDHC_ENHC_MESON8M2_DEBUG, 1) |
	      MESON_SDHC_ENHC_MESON8M2_WRRSP_MODE |
	      FIELD_PREP(MESON_SDHC_ENHC_SDIO_IRQ_PERIOD, 12),
	writel(val, host->base + MESON_SDHC_ENHC);
}

static void meson_mx_sdhc_set_pdma_meson8m2(struct mmc_host *mmc)
{
	meson_mx_sdhc_mask_bits(mmc, MESON_SDHC_PDMA,
				MESON_SDHC_PDMA_DMA_MODE,
				MESON_SDHC_PDMA_DMA_MODE);
}

static void meson_mx_sdhc_init_hw(struct mmc_host *mmc)
{
	struct meson_mx_sdhc_host *host = mmc_priv(mmc);
	u32 val;

	meson_mx_sdhc_reset(mmc);

	val = 0;
	val |= FIELD_PREP(MESON_SDHC_CTRL_RX_PERIOD, 0xf);
	val |= FIELD_PREP(MESON_SDHC_CTRL_RX_TIMEOUT, 0x7f);
	val |= FIELD_PREP(MESON_SDHC_CTRL_RX_ENDIAN, 0x7);
	val |= FIELD_PREP(MESON_SDHC_CTRL_TX_ENDIAN, 0x7);
	writel(val, host->base + MESON_SDHC_CTRL);

	writel(0, host->base + MESON_SDHC_CLKC);

	val = 0;
	val |= MESON_SDHC_PDMA_DMA_URGENT;
	val |= FIELD_PREP(MESON_SDHC_PDMA_WR_BURST, 7);
	val |= FIELD_PREP(MESON_SDHC_PDMA_TXFIFO_TH, 49);
	val |= FIELD_PREP(MESON_SDHC_PDMA_RD_BURST, 15);
	val |= FIELD_PREP(MESON_SDHC_PDMA_RXFIFO_TH, 7);
	writel(val, host->base + MESON_SDHC_PDMA);

	/* some initialization bits depend on the SoC: */
	host->platform->init_hw(mmc);

	/* disable and mask all interrupts: */
	writel(0, host->base + MESON_SDHC_ICTL);
	writel(MESON_SDHC_ISTA_ALL_IRQS, host->base + MESON_SDHC_ISTA);
}

static int meson_mx_sdhc_probe(struct platform_device *pdev)
{
	struct meson_mx_sdhc_host *host;
	struct mmc_host *mmc;
	struct resource *res;
	int ret, irq;

	mmc = mmc_alloc_host(sizeof(*host), &pdev->dev);
	if (!mmc)
		return -ENOMEM;

	host = mmc_priv(mmc);
	host->mmc = mmc;

	spin_lock_init(&host->irq_lock);

	platform_set_drvdata(pdev, host);

	host->platform = device_get_match_data(&pdev->dev);
	if (!host->platform) {
		ret = -EINVAL;
		goto error_free_mmc;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	host->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(host->base)) {
		ret = PTR_ERR(host->base);
		goto error_free_mmc;
	}

	host->pclk = devm_clk_get(&pdev->dev, "pclk");
	if (IS_ERR(host->pclk)) {
		ret = PTR_ERR(host->pclk);
		goto error_free_mmc;
	}

	host->parent_clks[0].id = "clkin0";
	host->parent_clks[1].id = "clkin1";
	host->parent_clks[2].id = "clkin2";
	host->parent_clks[3].id = "clkin3";
	ret = devm_clk_bulk_get(&pdev->dev, MESON_SDHC_PARENT_CLKS,
				host->parent_clks);
	if (ret)
		goto error_free_mmc;

	ret = meson_mx_sdhc_register_clks(host);
	if (ret)
		goto error_free_mmc;

	irq = platform_get_irq(pdev, 0);
	ret = devm_request_threaded_irq(&pdev->dev, irq,
					meson_mx_sdhc_irq,
					meson_mx_sdhc_irq_thread,
					0, NULL, host);
	if (ret)
		goto error_free_mmc;

	ret = clk_prepare_enable(host->pclk);
	if (ret) {
		dev_err(&pdev->dev, "Failed to enable 'pclk' clock\n");
		goto error_free_mmc;
	}

	ret = clk_set_phase(host->sd_clk, 181);
	if (ret) {
		dev_warn(mmc_dev(mmc),
			 "Failed to set SD clock phase to 181 deg\n");
		return ret;
	}

	meson_mx_sdhc_init_hw(mmc);

	/* Get regulators and the supported OCR mask */
	ret = mmc_regulator_get_supply(mmc);
	if (ret)
		goto error_disable_pclk;

	mmc->max_req_size = MESON_SDHC_BOUNCE_REQ_SIZE;
	mmc->max_seg_size = mmc->max_req_size;
	mmc->max_blk_count = FIELD_GET(MESON_SDHC_SEND_TOTAL_PACK, 0xffffffff);
	mmc->max_blk_size = FIELD_GET(MESON_SDHC_CTRL_PACK_LEN, 0xffffffff);
	mmc->max_blk_size += 1; /* MESON_SDHC_CTRL_PACK_LEN 0 means 512 */
	mmc->max_busy_timeout = 1000; // FIXME
	mmc->f_min = clk_round_rate(host->sd_clk, 1);
	mmc->f_max = clk_round_rate(host->sd_clk, ~0UL);
	mmc->max_current_180 = 300;
	mmc->max_current_330 = 300;
	mmc->caps |= MMC_CAP_ERASE | MMC_CAP_CMD23;
	mmc->ops = &meson_mx_sdhc_ops;

	ret = mmc_of_parse(mmc);
	if (ret)
		goto error_disable_pclk;

	ret = mmc_add_host(mmc);
	if (ret)
		goto error_disable_pclk;

	return 0;

error_disable_pclk:
	clk_disable_unprepare(host->pclk);
error_free_mmc:
	mmc_free_host(mmc);

	return ret;
}

static int meson_mx_sdhc_remove(struct platform_device *pdev)
{
	struct meson_mx_sdhc_host *host = platform_get_drvdata(pdev);

	mmc_remove_host(host->mmc);

	meson_mx_sdhc_disable_clks(host->mmc);

	clk_disable_unprepare(host->pclk);

	mmc_free_host(host->mmc);

	return 0;
}

static struct meson_mx_sdhc_data meson_mx_sdhc_data_meson8 = {
	.init_hw			= &meson_mx_sdhc_init_hw_meson8,
	.set_pdma			= &meson_mx_sdhc_set_pdma_meson8,
	.hardware_flush_all_cmds	= false,
};

static struct meson_mx_sdhc_data meson_mx_sdhc_data_meson8m2 = {
	.init_hw			= &meson_mx_sdhc_init_hw_meson8m2,
	.set_pdma			= &meson_mx_sdhc_set_pdma_meson8m2,
	.hardware_flush_all_cmds	= true,
};

static const struct of_device_id meson_mx_sdhc_of_match[] = {
	{
		.compatible = "amlogic,meson8-sdhc",
		.data = &meson_mx_sdhc_data_meson8
	},
	{
		.compatible = "amlogic,meson8b-sdhc",
		.data = &meson_mx_sdhc_data_meson8
	},
	{
		.compatible = "amlogic,meson8m2-sdhc", 
		.data = &meson_mx_sdhc_data_meson8m2
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, meson_mx_sdhc_of_match);

static struct platform_driver meson_mx_sdhc_driver = {
	.probe   = meson_mx_sdhc_probe,
	.remove  = meson_mx_sdhc_remove,
	.driver  = {
		.name = "meson-mx-sdhc",
		.of_match_table = of_match_ptr(meson_mx_sdhc_of_match),
	},
};

module_platform_driver(meson_mx_sdhc_driver);

MODULE_DESCRIPTION("Meson6, Meson8, Meson8b and Meson8m2 SDHC Host Driver");
MODULE_AUTHOR("Martin Blumenstingl <martin.blumenstingl@googlemail.com>");
MODULE_LICENSE("GPL v2");
