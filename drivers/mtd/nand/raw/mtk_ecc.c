// SPDX-License-Identifier: GPL-2.0 OR MIT
/*
 * MTK ECC controller driver.
 * Copyright (C) 2016  MediaTek Inc.
 * Authors:	Xiaolei Li		<xiaolei.li@mediatek.com>
 *		Jorge Ramirez-Ortiz	<jorge.ramirez-ortiz@linaro.org>
 */

#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/iopoll.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/mutex.h>

#include "mtk_ecc.h"

#define ECC_IDLE_MASK		BIT(0)
#define ECC_IRQ_EN		BIT(0)
#define ECC_PG_IRQ_SEL		BIT(1)
#define ECC_OP_ENABLE		(1)
#define ECC_OP_DISABLE		(0)

#define ECC_ENCCON		(0x00)
#define ECC_ENCCNFG		(0x04)
#define		ECC_MS_SHIFT		(16)
#define ECC_ENCDIADDR		(0x08)
#define ECC_ENCIDLE		(0x0C)
#define ECC_DECCON		(0x100)
#define ECC_DECCNFG		(0x104)
#define		DEC_EMPTY_EN		BIT(31)
#define		DEC_CNFG_CORRECT_MASK	GENMASK(13, 12)
#define ECC_DECIDLE		(0x10C)
#define ECC_DECENUM0		(0x114)
#define ECC_DECEL0		(0x11C)
#define		DEC_EL_ODD_BYTE_POS_MASK	GENMASK(31, 19)
#define		DEC_EL_ODD_BIT_POS_MASK		GENMASK(18, 16)
#define		DEC_EL_EVEN_BYTE_POS_MASK	GENMASK(15, 3)
#define		DEC_EL_EVEN_BIT_POS_MASK	GENMASK(2, 0)

#define ECC_TIMEOUT		(500000)

#define ECC_IDLE_REG(op)	((op) == ECC_ENCODE ? ECC_ENCIDLE : ECC_DECIDLE)
#define ECC_CTL_REG(op)		((op) == ECC_ENCODE ? ECC_ENCCON : ECC_DECCON)

struct mtk_ecc_caps {
	u32 err_mask;
	u32 err_shift;
	u32 num_err_sect;
	const u8 *ecc_strength;
	const u32 *ecc_regs;
	u8 num_ecc_strength;
	u8 ecc_mode_shift;
	u32 parity_bits;
	int pg_irq_sel;
	bool has_dma_support;
};

struct mtk_ecc {
	struct device *dev;
	const struct mtk_ecc_caps *caps;
	void __iomem *regs;
	struct clk *clk;

	struct completion done;
	struct mutex lock;
	u32 sectors;

	u8 *eccdata;
};

/* ecc strength that each IP supports */
static const u8 ecc_strength_mt2701[] = {
	4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 28, 32, 36,
	40, 44, 48, 52, 56, 60
};

static const u8 ecc_strength_mt2712[] = {
	4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 28, 32, 36,
	40, 44, 48, 52, 56, 60, 68, 72, 80
};

static const u8 ecc_strength_mt7622[] = {
	4, 6, 8, 10, 12
};

enum mtk_ecc_regs {
	ECC_ENCPAR00,
	ECC_ENCIRQ_EN,
	ECC_ENCIRQ_STA,
	ECC_DECDONE,
	ECC_DECIRQ_EN,
	ECC_DECIRQ_STA,
	ECC_FDMADDR,
};

static int mt2701_ecc_regs[] = {
	[ECC_ENCPAR00] =        0x10,
	[ECC_ENCIRQ_EN] =       0x80,
	[ECC_ENCIRQ_STA] =      0x84,
	[ECC_DECDONE] =         0x124,
	[ECC_DECIRQ_EN] =       0x200,
	[ECC_DECIRQ_STA] =      0x204,
	[ECC_FDMADDR] =         -1,
};

static int mt2712_ecc_regs[] = {
	[ECC_ENCPAR00] =        0x300,
	[ECC_ENCIRQ_EN] =       0x80,
	[ECC_ENCIRQ_STA] =      0x84,
	[ECC_DECDONE] =         0x124,
	[ECC_DECIRQ_EN] =       0x200,
	[ECC_DECIRQ_STA] =      0x204,
	[ECC_FDMADDR] =         -1,
};

static int mt7621_ecc_regs[] = {
	[ECC_ENCPAR00] =        -1,
	[ECC_ENCIRQ_EN] =       -1,
	[ECC_ENCIRQ_STA] =      -1,
	[ECC_DECDONE] =         0x118,
	[ECC_DECIRQ_EN] =       -1,
	[ECC_DECIRQ_STA] =      -1,
	[ECC_FDMADDR] =         0x13c,
};

static int mt7622_ecc_regs[] = {
	[ECC_ENCPAR00] =        0x10,
	[ECC_ENCIRQ_EN] =       0x30,
	[ECC_ENCIRQ_STA] =      0x34,
	[ECC_DECDONE] =         0x11c,
	[ECC_DECIRQ_EN] =       0x140,
	[ECC_DECIRQ_STA] =      0x144,
	[ECC_FDMADDR] =         -1,
};

static inline void mtk_ecc_wait_idle(struct mtk_ecc *ecc,
				     enum mtk_ecc_operation op)
{
	struct device *dev = ecc->dev;
	u32 val;
	int ret;

	ret = readl_poll_timeout_atomic(ecc->regs + ECC_IDLE_REG(op), val,
					val & ECC_IDLE_MASK,
					10, ECC_TIMEOUT);
	if (ret)
		dev_warn(dev, "%s NOT idle\n",
			 op == ECC_ENCODE ? "encoder" : "decoder");
}

static irqreturn_t mtk_ecc_irq(int irq, void *id)
{
	struct mtk_ecc *ecc = id;
	u32 dec, enc;

	dec = readw(ecc->regs + ecc->caps->ecc_regs[ECC_DECIRQ_STA])
		    & ECC_IRQ_EN;
	if (dec) {
		dec = readw(ecc->regs + ecc->caps->ecc_regs[ECC_DECDONE]);
		if (dec & ecc->sectors) {
			/*
			 * Clear decode IRQ status once again to ensure that
			 * there will be no extra IRQ.
			 */
			readw(ecc->regs + ecc->caps->ecc_regs[ECC_DECIRQ_STA]);
			ecc->sectors = 0;
			complete(&ecc->done);
		} else {
			return IRQ_HANDLED;
		}
	} else {
		enc = readl(ecc->regs + ecc->caps->ecc_regs[ECC_ENCIRQ_STA])
		      & ECC_IRQ_EN;
		if (enc)
			complete(&ecc->done);
		else
			return IRQ_NONE;
	}

	return IRQ_HANDLED;
}

static int mtk_ecc_config(struct mtk_ecc *ecc, struct mtk_ecc_config *config)
{
	u32 ecc_bit, dec_sz, enc_sz;
	u32 reg, i;

	for (i = 0; i < ecc->caps->num_ecc_strength; i++) {
		if (ecc->caps->ecc_strength[i] == config->strength)
			break;
	}

	if (i == ecc->caps->num_ecc_strength) {
		dev_err(ecc->dev, "invalid ecc strength %d\n",
			config->strength);
		return -EINVAL;
	}

	ecc_bit = i;

	if (config->op == ECC_ENCODE) {
		/* configure ECC encoder (in bits) */
		enc_sz = config->len << 3;

		reg = ecc_bit | (config->mode << ecc->caps->ecc_mode_shift);
		reg |= (enc_sz << ECC_MS_SHIFT);
		writel(reg, ecc->regs + ECC_ENCCNFG);

		if (ecc->caps->has_dma_support && config->mode != ECC_NFI_MODE)
			writel(lower_32_bits(config->addr),
			       ecc->regs + ECC_ENCDIADDR);

	} else {
		/* configure ECC decoder (in bits) */
		dec_sz = (config->len << 3) +
			 config->strength * ecc->caps->parity_bits;

		reg = ecc_bit | (config->mode << ecc->caps->ecc_mode_shift);
		reg |= (dec_sz << ECC_MS_SHIFT);
		reg |= DEC_EMPTY_EN;

		if (ecc->caps->has_dma_support)
			reg |= FIELD_PREP(DEC_CNFG_CORRECT_MASK, 3);
		else
			reg |= FIELD_PREP(DEC_CNFG_CORRECT_MASK, 2);

		writel(reg, ecc->regs + ECC_DECCNFG);

		if (config->sectors)
			ecc->sectors = 1 << (config->sectors - 1);
	}

	return 0;
}

static u32 mtc_ecc_get_num_error_bits(struct mtk_ecc *ecc, u32 reg_offset,
				      u8 sector)
{
	u32 err;

	err = readl(ecc->regs + ECC_DECENUM0 + reg_offset);
	err >>= ((sector % ecc->caps->num_err_sect) * ecc->caps->err_shift);
	err &= ecc->caps->err_mask;

	return err;
}

void mtk_ecc_get_stats(struct mtk_ecc *ecc, struct mtk_ecc_stats *stats,
		       int sectors)
{
	u32 offset, i, err;
	u32 bitflips = 0;

	if (!ecc->caps->has_dma_support) {
		dev_err(ecc->dev, "%s requires DMA support\n", __func__);
		return;
	}

	stats->corrected = 0;
	stats->failed = 0;

	for (i = 0; i < sectors; i++) {
		offset = (i >> 2) << 2;
		err = mtc_ecc_get_num_error_bits(ecc, offset, i);
		if (err == ecc->caps->err_mask) {
			/* uncorrectable errors */
			stats->failed++;
			continue;
		}

		stats->corrected += err;
		bitflips = max_t(u32, bitflips, err);
	}

	stats->bitflips = bitflips;
}
EXPORT_SYMBOL(mtk_ecc_get_stats);

void mtk_ecc_correct_sector(struct mtk_ecc *ecc, struct mtk_ecc_stats *stats,
			    u32 ecc_size, u8 *sector_buf, u32 fdm_reg_size,
			    u8 *fdm_buf, u8 sector)
{
	unsigned int i;

	stats->corrected = 0;
	stats->failed = 0;

	stats->bitflips = mtc_ecc_get_num_error_bits(ecc, 0, sector);
	if (stats->bitflips == ecc->caps->err_mask) {
		/* uncorrectable errors */
		stats->bitflips = 0;
		stats->failed++;
		return;
	}

	for (i = 0; i < stats->bitflips; i++) {
		u32 error_locations, error_byte_pos, error_bit_pos;
		u8 *error_byte = NULL;

		/* Two (even and odd) sectors per 32-bit/4-byte register */
		error_locations = readl(ecc->regs + ECC_DECEL0 + ((i / 2) * 4));

		if (i & 1) {
			error_bit_pos = FIELD_GET(DEC_EL_ODD_BIT_POS_MASK,
						  error_locations);
			error_byte_pos = FIELD_GET(DEC_EL_ODD_BYTE_POS_MASK,
						   error_locations);
		} else {
			error_bit_pos = FIELD_GET(DEC_EL_EVEN_BIT_POS_MASK,
						  error_locations);
			error_byte_pos = FIELD_GET(DEC_EL_EVEN_BYTE_POS_MASK,
						   error_locations);
		}

		if (error_byte_pos < ecc_size)
			error_byte = &sector_buf[error_byte_pos];
		else if (error_byte_pos - ecc_size < fdm_reg_size)
			error_byte = &fdm_buf[error_byte_pos - ecc_size];

		if (error_byte) {
			*error_byte ^= (1 << error_bit_pos);
			stats->corrected++;
		}
	}
}
EXPORT_SYMBOL(mtk_ecc_correct_sector);

void mtk_ecc_release(struct mtk_ecc *ecc)
{
	clk_disable_unprepare(ecc->clk);
	put_device(ecc->dev);
}
EXPORT_SYMBOL(mtk_ecc_release);

static void mtk_ecc_hw_init(struct mtk_ecc *ecc)
{
	mtk_ecc_wait_idle(ecc, ECC_ENCODE);
	writew(ECC_OP_DISABLE, ecc->regs + ECC_ENCCON);

	mtk_ecc_wait_idle(ecc, ECC_DECODE);
	writel(ECC_OP_DISABLE, ecc->regs + ECC_DECCON);
}

static struct mtk_ecc *mtk_ecc_get(struct device_node *np)
{
	struct platform_device *pdev;
	struct mtk_ecc *ecc;

	pdev = of_find_device_by_node(np);
	if (!pdev)
		return ERR_PTR(-EPROBE_DEFER);

	ecc = platform_get_drvdata(pdev);
	if (!ecc) {
		put_device(&pdev->dev);
		return ERR_PTR(-EPROBE_DEFER);
	}

	clk_prepare_enable(ecc->clk);
	mtk_ecc_hw_init(ecc);

	return ecc;
}

struct mtk_ecc *of_mtk_ecc_get(struct device_node *of_node)
{
	struct mtk_ecc *ecc = NULL;
	struct device_node *np;

	np = of_parse_phandle(of_node, "ecc-engine", 0);
	if (np) {
		ecc = mtk_ecc_get(np);
		of_node_put(np);
	}

	return ecc;
}
EXPORT_SYMBOL(of_mtk_ecc_get);

int mtk_ecc_enable(struct mtk_ecc *ecc, struct mtk_ecc_config *config)
{
	enum mtk_ecc_operation op = config->op;
	u16 reg_val;
	int ret;

	ret = mutex_lock_interruptible(&ecc->lock);
	if (ret) {
		dev_err(ecc->dev, "interrupted when attempting to lock\n");
		return ret;
	}

	mtk_ecc_wait_idle(ecc, op);

	ret = mtk_ecc_config(ecc, config);
	if (ret) {
		mutex_unlock(&ecc->lock);
		return ret;
	}

	if (!ecc->caps->has_dma_support) {
		if (!config->fdma_addr) {
			dev_err(ecc->dev,
				"FMDA addr is missing in mtk_ecc_config\n");
			mutex_unlock(&ecc->lock);
			return -EINVAL;
		}

		writel(lower_32_bits(config->fdma_addr),
		       ecc->regs + ecc->caps->ecc_regs[ECC_FDMADDR]);
	} else if (config->mode != ECC_NFI_MODE || op != ECC_ENCODE) {
		init_completion(&ecc->done);
		reg_val = ECC_IRQ_EN;
		/*
		 * For ECC_NFI_MODE, if ecc->caps->pg_irq_sel is 1, then it
		 * means this chip can only generate one ecc irq during page
		 * read / write. If is 0, generate one ecc irq each ecc step.
		 */
		if (ecc->caps->pg_irq_sel && config->mode == ECC_NFI_MODE)
			reg_val |= ECC_PG_IRQ_SEL;
		if (op == ECC_ENCODE)
			writew(reg_val, ecc->regs +
			       ecc->caps->ecc_regs[ECC_ENCIRQ_EN]);
		else
			writew(reg_val, ecc->regs +
			       ecc->caps->ecc_regs[ECC_DECIRQ_EN]);
	}

	writew(ECC_OP_ENABLE, ecc->regs + ECC_CTL_REG(op));

	dev_err(ecc->dev, "%s(): ECC_ENCCNFG = 0x%08x, ECC_DECCNFG = 0x%08x\n", __func__, readl(ecc->regs + ECC_ENCCNFG), readl(ecc->regs + ECC_DECCNFG));

	return 0;
}
EXPORT_SYMBOL(mtk_ecc_enable);

void mtk_ecc_disable(struct mtk_ecc *ecc)
{
	enum mtk_ecc_operation op = ECC_ENCODE;

	/* find out the running operation */
	if (readw(ecc->regs + ECC_CTL_REG(op)) != ECC_OP_ENABLE)
		op = ECC_DECODE;

	/* disable it */
	mtk_ecc_wait_idle(ecc, op);
	if (op == ECC_DECODE)
		/*
		 * Clear decode IRQ status in case there is a timeout to wait
		 * decode IRQ.
		 */
		readw(ecc->regs + ecc->caps->ecc_regs[ECC_DECDONE]);

	if (ecc->caps->has_dma_support) {
		if (op == ECC_DECODE)
			writew(0, ecc->regs + ecc->caps->ecc_regs[ECC_DECIRQ_EN]);
		else
			writew(0, ecc->regs + ecc->caps->ecc_regs[ECC_ENCIRQ_EN]);
	}

	writew(ECC_OP_DISABLE, ecc->regs + ECC_CTL_REG(op));

	mutex_unlock(&ecc->lock);
}
EXPORT_SYMBOL(mtk_ecc_disable);

int mtk_ecc_wait_done(struct mtk_ecc *ecc, enum mtk_ecc_operation op)
{
	int ret;

	if (!ecc->caps->has_dma_support) {
		dev_err(ecc->dev,
			"Cannot wait for DMA based ECC done without DMA support\n");
		return -EINVAL;
	}

	ret = wait_for_completion_timeout(&ecc->done,
					  usecs_to_jiffies(ECC_TIMEOUT));
	if (!ret) {
		dev_err(ecc->dev, "%s timeout - interrupt did not arrive)\n",
			(op == ECC_ENCODE) ? "encoder" : "decoder");
		return -ETIMEDOUT;
	}

	return 0;
}
EXPORT_SYMBOL(mtk_ecc_wait_done);

int mtk_ecc_wait_sector_done(struct mtk_ecc *ecc, enum mtk_ecc_operation op,
			     u8 sector)
{
	void __iomem *eccdone = ecc->regs + ecc->caps->ecc_regs[ECC_DECDONE];
	u32 val;
	int ret;

	ret = readw_poll_timeout_atomic(eccdone, val,
					val & (1 << sector), 10,
					ECC_TIMEOUT);
	if (ret) {
		dev_err(ecc->dev,
			"%s timeout while waiting for sector %d ECC done\n",
			(op == ECC_ENCODE) ? "encoder" : "decoder", sector);
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL(mtk_ecc_wait_sector_done);

int mtk_ecc_encode(struct mtk_ecc *ecc, struct mtk_ecc_config *config,
		   u8 *data, u32 bytes)
{
	dma_addr_t addr;
	u32 len;
	int ret;

	if (!ecc->caps->has_dma_support) {
		dev_err(ecc->dev, "Cannot encode data without DMA support\n");
		return -EINVAL;
	}

	addr = dma_map_single(ecc->dev, data, bytes, DMA_TO_DEVICE);
	ret = dma_mapping_error(ecc->dev, addr);
	if (ret) {
		dev_err(ecc->dev, "dma mapping error\n");
		return -EINVAL;
	}

	config->op = ECC_ENCODE;
	config->addr = addr;
	ret = mtk_ecc_enable(ecc, config);
	if (ret) {
		dma_unmap_single(ecc->dev, addr, bytes, DMA_TO_DEVICE);
		return ret;
	}

	ret = mtk_ecc_wait_done(ecc, ECC_ENCODE);
	if (ret)
		goto timeout;

	mtk_ecc_wait_idle(ecc, ECC_ENCODE);

	/* Program ECC bytes to OOB: per sector oob = FDM + ECC + SPARE */
	len = (config->strength * ecc->caps->parity_bits + 7) >> 3;

	/* write the parity bytes generated by the ECC back to temp buffer */
	__ioread32_copy(ecc->eccdata,
			ecc->regs + ecc->caps->ecc_regs[ECC_ENCPAR00],
			round_up(len, 4));

	/* copy into possibly unaligned OOB region with actual length */
	memcpy(data + bytes, ecc->eccdata, len);
timeout:

	dma_unmap_single(ecc->dev, addr, bytes, DMA_TO_DEVICE);
	mtk_ecc_disable(ecc);

	return ret;
}
EXPORT_SYMBOL(mtk_ecc_encode);

void mtk_ecc_adjust_strength(struct mtk_ecc *ecc, u32 *p)
{
	const u8 *ecc_strength = ecc->caps->ecc_strength;
	int i;

	for (i = 0; i < ecc->caps->num_ecc_strength; i++) {
		if (*p <= ecc_strength[i]) {
			if (!i)
				*p = ecc_strength[i];
			else if (*p != ecc_strength[i])
				*p = ecc_strength[i - 1];
			return;
		}
	}

	*p = ecc_strength[ecc->caps->num_ecc_strength - 1];
}
EXPORT_SYMBOL(mtk_ecc_adjust_strength);

unsigned int mtk_ecc_get_parity_bits(struct mtk_ecc *ecc)
{
	return ecc->caps->parity_bits;
}
EXPORT_SYMBOL(mtk_ecc_get_parity_bits);

static const struct mtk_ecc_caps mtk_ecc_caps_mt2701 = {
	.err_mask = 0x3f,
	.err_shift = 8,
	.num_err_sect = 4,
	.ecc_strength = ecc_strength_mt2701,
	.num_ecc_strength = ARRAY_SIZE(ecc_strength_mt2701),
	.ecc_regs = mt2701_ecc_regs,
	.ecc_mode_shift = 5,
	.parity_bits = 14,
	.pg_irq_sel = 0,
	.has_dma_support = true,
};

static const struct mtk_ecc_caps mtk_ecc_caps_mt2712 = {
	.err_mask = 0x7f,
	.err_shift = 8,
	.num_err_sect = 4,
	.ecc_strength = ecc_strength_mt2712,
	.num_ecc_strength = ARRAY_SIZE(ecc_strength_mt2712),
	.ecc_regs = mt2712_ecc_regs,
	.ecc_mode_shift = 5,
	.parity_bits = 14,
	.pg_irq_sel = 1,
	.has_dma_support = true,
};

static const struct mtk_ecc_caps mtk_ecc_caps_mt7621 = {
	.err_mask = 0xf,
	.err_shift = 4,
	.num_err_sect = 8,
	.ecc_strength = ecc_strength_mt7622,
	.num_ecc_strength = ARRAY_SIZE(ecc_strength_mt7622),
	.ecc_regs = mt7621_ecc_regs,
	.ecc_mode_shift = 4,
	.parity_bits = 13,
	.pg_irq_sel = 0,
	.has_dma_support = false,
};

static const struct mtk_ecc_caps mtk_ecc_caps_mt7622 = {
	.err_mask = 0x1f,
	.err_shift = 5,
	.num_err_sect = 4,
	.ecc_strength = ecc_strength_mt7622,
	.num_ecc_strength = ARRAY_SIZE(ecc_strength_mt7622),
	.ecc_regs = mt7622_ecc_regs,
	.ecc_mode_shift = 4,
	.parity_bits = 13,
	.pg_irq_sel = 0,
	.has_dma_support = true,
};

static const struct of_device_id mtk_ecc_dt_match[] = {
	{
		.compatible = "mediatek,mt2701-ecc",
		.data = &mtk_ecc_caps_mt2701,
	}, {
		.compatible = "mediatek,mt2712-ecc",
		.data = &mtk_ecc_caps_mt2712,
	}, {
		.compatible = "mediatek,mt7621-ecc",
		.data = &mtk_ecc_caps_mt7621,
	}, {
	}, {
		.compatible = "mediatek,mt7622-ecc",
		.data = &mtk_ecc_caps_mt7622,
	},
	{},
};

static int mtk_ecc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mtk_ecc *ecc;
	u32 max_eccdata_size;
	int irq, ret;

	ecc = devm_kzalloc(dev, sizeof(*ecc), GFP_KERNEL);
	if (!ecc)
		return -ENOMEM;

	ecc->caps = of_device_get_match_data(dev);

	max_eccdata_size = ecc->caps->num_ecc_strength - 1;
	max_eccdata_size = ecc->caps->ecc_strength[max_eccdata_size];
	max_eccdata_size = (max_eccdata_size * ecc->caps->parity_bits + 7) >> 3;
	max_eccdata_size = round_up(max_eccdata_size, 4);
	ecc->eccdata = devm_kzalloc(dev, max_eccdata_size, GFP_KERNEL);
	if (!ecc->eccdata)
		return -ENOMEM;

	ecc->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ecc->regs))
		return PTR_ERR(ecc->regs);

	ecc->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(ecc->clk)) {
		dev_err(dev, "failed to get clock: %ld\n", PTR_ERR(ecc->clk));
		return PTR_ERR(ecc->clk);
	}

	if (ecc->caps->has_dma_support) {
		irq = platform_get_irq(pdev, 0);
		if (irq < 0)
			return irq;

		ret = dma_set_mask(dev, DMA_BIT_MASK(32));
		if (ret) {
			dev_err(dev, "failed to set DMA mask\n");
			return ret;
		}

		ret = devm_request_irq(dev, irq, mtk_ecc_irq, 0x0, "mtk-ecc",
				       ecc);
		if (ret) {
			dev_err(dev, "failed to request irq\n");
			return -EINVAL;
		}
	}

	ecc->dev = dev;
	mutex_init(&ecc->lock);
	platform_set_drvdata(pdev, ecc);
	dev_info(dev, "probed\n");

	dev_err(dev, "ECC_ENCCON = 0x%08x\n", readl(ecc->regs + ECC_ENCCON));
	dev_err(dev, "ECC_ENCCNFG = 0x%08x\n", readl(ecc->regs + ECC_ENCCNFG));
	dev_err(dev, "ECC_ENCDIADDR = 0x%08x\n", readl(ecc->regs + ECC_ENCDIADDR));
	dev_err(dev, "ECC_ENCIDLE = 0x%08x\n", readl(ecc->regs + ECC_ENCIDLE));
	dev_err(dev, "ECC_DECCON = 0x%08x\n", readl(ecc->regs + ECC_DECCON));
	dev_err(dev, "ECC_DECCNFG = 0x%08x\n", readl(ecc->regs + ECC_DECCNFG));
	dev_err(dev, "ECC_DECIDLE = 0x%08x\n", readl(ecc->regs + ECC_DECIDLE));
	dev_err(dev, "ECC_DECENUM0 = 0x%08x\n", readl(ecc->regs + ECC_DECENUM0));
	dev_err(dev, "ECC_DECEL0 = 0x%08x\n", readl(ecc->regs + ECC_DECEL0));
	dev_err(dev, "ECC_DECDONE = 0x%08x\n", readl(ecc->regs + 0x118));
	dev_err(dev, "ECC_FDMADDR = 0x%08x\n", readl(ecc->regs + 0x13c));

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int mtk_ecc_suspend(struct device *dev)
{
	struct mtk_ecc *ecc = dev_get_drvdata(dev);

	clk_disable_unprepare(ecc->clk);

	return 0;
}

static int mtk_ecc_resume(struct device *dev)
{
	struct mtk_ecc *ecc = dev_get_drvdata(dev);
	int ret;

	ret = clk_prepare_enable(ecc->clk);
	if (ret) {
		dev_err(dev, "failed to enable clk\n");
		return ret;
	}

	return 0;
}

static SIMPLE_DEV_PM_OPS(mtk_ecc_pm_ops, mtk_ecc_suspend, mtk_ecc_resume);
#endif

MODULE_DEVICE_TABLE(of, mtk_ecc_dt_match);

static struct platform_driver mtk_ecc_driver = {
	.probe  = mtk_ecc_probe,
	.driver = {
		.name  = "mtk-ecc",
		.of_match_table = mtk_ecc_dt_match,
#ifdef CONFIG_PM_SLEEP
		.pm = &mtk_ecc_pm_ops,
#endif
	},
};

module_platform_driver(mtk_ecc_driver);

MODULE_AUTHOR("Xiaolei Li <xiaolei.li@mediatek.com>");
MODULE_DESCRIPTION("MTK Nand ECC Driver");
MODULE_LICENSE("Dual MIT/GPL");
