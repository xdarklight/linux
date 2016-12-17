/*
 * Amlogic Meson Successive Approximation Register (SAR) A/D Converter
 *
 * Copyright (C) 2016 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/reset.h>
#include <linux/regmap.h>
#include <linux/iio/iio.h>

#define SAR_ADC_REG0						0x00
	#define SAR_ADC_REG0_PANEL_DETECT			BIT(31)
	#define SAR_ADC_REG0_DELTA_BUSY				BIT(30)
	#define SAR_ADC_REG0_AVG_BUSY				BIT(29)
	#define SAR_ADC_REG0_SAMPLE_BUSY			BIT(28)
	#define SAR_ADC_REG0_FIFO_FULL				BIT(27)
	#define SAR_ADC_REG0_FIFO_EMPTY				BIT(26)
	#define SAR_ADC_REG0_FIFO_COUNT_SHIFT			25
	#define SAR_ADC_REG0_FIFO_COUNT_MASK			GENMASK(25, 21)
	#define SAR_ADC_REG0_ADC_BIAS_CTRL_SHIFT		19
	#define SAR_ADC_REG0_ADC_BIAS_CTRL_MASK			GENMASK(20, 19)
	#define SAR_ADC_REG0_CURR_CHAN_ID_SHIFT			16
	#define SAR_ADC_REG0_CURR_CHAN_ID_MASK			GENMASK(18, 16)
	#define SAR_ADC_REG0_ADC_TEMP_SEN_SEL			BIT(15)
	#define SAR_ADC_REG0_SAMPLING_STOP			BIT(14)
	#define SAR_ADC_REG0_CHAN_DELTA_EN_SHIFT		12
	#define SAR_ADC_REG0_CHAN_DELTA_EN_MASK			GENMASK(13, 12)
	#define SAR_ADC_REG0_DETECT_IRQ_POL			BIT(10)
	#define SAR_ADC_REG0_DETECT_IRQ_EN			BIT(9)
	#define SAR_ADC_REG0_FIFO_CNT_IRQ_SHIFT			4
	#define SAR_ADC_REG0_FIFO_CNT_IRQ_MASK			GENMASK(8, 4)
	#define SAR_ADC_REG0_FIFO_IRQ_EN			BIT(3)
	#define SAR_ADC_REG0_SAMPLE_START			BIT(2)
	#define SAR_ADC_REG0_CONTINUOUS_EN			BIT(1)
	#define SAR_ADC_REG0_SAMPLING_ENABLE			BIT(0)

#define SAR_ADC_CHAN_LIST					0x04
	#define SAR_ADC_CHAN_LIST_NUM_CHANNELS_SHIFT		24
	#define SAR_ADC_CHAN_LIST_NUM_CHANNELS_MASK		GENMASK(26, 24)
	#define SAR_ADC_CHAN_LIST_8TH_CHAN_SHIFT		21
	#define SAR_ADC_CHAN_LIST_8TH_CHAN_MASK			GENMASK(23, 21)
	#define SAR_ADC_CHAN_LIST_7TH_CHAN_SHIFT		18
	#define SAR_ADC_CHAN_LIST_7TH_CHAN_MASK			GENMASK(20, 18)
	#define SAR_ADC_CHAN_LIST_6TH_CHAN_SHIFT		15
	#define SAR_ADC_CHAN_LIST_6TH_CHAN_MASK			GENMASK(17, 15)
	#define SAR_ADC_CHAN_LIST_5TH_CHAN_SHIFT		12
	#define SAR_ADC_CHAN_LIST_5TH_CHAN_MASK			GENMASK(14, 12)
	#define SAR_ADC_CHAN_LIST_4TH_CHAN_SHIFT		9
	#define SAR_ADC_CHAN_LIST_4TH_CHAN_MASK			GENMASK(11, 9)
	#define SAR_ADC_CHAN_LIST_3RD_CHAN_SHIFT		6
	#define SAR_ADC_CHAN_LIST_3RD_CHAN_MASK			GENMASK(8, 6)
	#define SAR_ADC_CHAN_LIST_2ND_CHAN_SHIFT		3
	#define SAR_ADC_CHAN_LIST_2ND_CHAN_MASK			GENMASK(5, 3)
	#define SAR_ADC_CHAN_LIST_1ST_CHAN_SHIFT		0
	#define SAR_ADC_CHAN_LIST_1ST_CHAN_MASK			GENMASK(2, 0)

#define SAR_ADC_AVG_CNTL					0x08
	#define SAR_ADC_AVG_CNTL_AVG_MODE_CHAN7_SHIFT		30
	#define SAR_ADC_AVG_CNTL_AVG_MODE_CHAN7_MASK		GENMASK(31, 30)
	#define SAR_ADC_AVG_CNTL_AVG_MODE_CHAN6_SHIFT		28
	#define SAR_ADC_AVG_CNTL_AVG_MODE_CHAN6_MASK		GENMASK(29, 28)
	#define SAR_ADC_AVG_CNTL_AVG_MODE_CHAN5_SHIFT		26
	#define SAR_ADC_AVG_CNTL_AVG_MODE_CHAN5_MASK		GENMASK(27, 26)
	#define SAR_ADC_AVG_CNTL_AVG_MODE_CHAN4_SHIFT		24
	#define SAR_ADC_AVG_CNTL_AVG_MODE_CHAN4_MASK		GENMASK(25, 24)
	#define SAR_ADC_AVG_CNTL_AVG_MODE_CHAN3_SHIFT		22
	#define SAR_ADC_AVG_CNTL_AVG_MODE_CHAN3_MASK		GENMASK(23, 22)
	#define SAR_ADC_AVG_CNTL_AVG_MODE_CHAN2_SHIFT		20
	#define SAR_ADC_AVG_CNTL_AVG_MODE_CHAN2_MASK		GENMASK(21, 20)
	#define SAR_ADC_AVG_CNTL_AVG_MODE_CHAN1_SHIFT		18
	#define SAR_ADC_AVG_CNTL_AVG_MODE_CHAN1_MASK		GENMASK(19, 18)
	#define SAR_ADC_AVG_CNTL_AVG_MODE_CHAN0_SHIFT		16
	#define SAR_ADC_AVG_CNTL_AVG_MODE_CHAN0_MASK		GENMASK(17, 16)
	#define SAR_ADC_AVG_CNTL_NUM_SAMPLES_CHAN7_SHIFT	14
	#define SAR_ADC_AVG_CNTL_NUM_SAMPLES_CHAN7_MASK		GENMASK(15, 14)
	#define SAR_ADC_AVG_CNTL_NUM_SAMPLES_CHAN6_SHIFT	12
	#define SAR_ADC_AVG_CNTL_NUM_SAMPLES_CHAN6_MASK		GENMASK(13, 12)
	#define SAR_ADC_AVG_CNTL_NUM_SAMPLES_CHAN5_SHIFT	10
	#define SAR_ADC_AVG_CNTL_NUM_SAMPLES_CHAN5_MASK		GENMASK(11, 10)
	#define SAR_ADC_AVG_CNTL_NUM_SAMPLES_CHAN4_SHIFT	8
	#define SAR_ADC_AVG_CNTL_NUM_SAMPLES_CHAN4_MASK		GENMASK(9, 8)
	#define SAR_ADC_AVG_CNTL_NUM_SAMPLES_CHAN3_SHIFT	6
	#define SAR_ADC_AVG_CNTL_NUM_SAMPLES_CHAN3_MASK		GENMASK(7, 6)
	#define SAR_ADC_AVG_CNTL_NUM_SAMPLES_CHAN2_SHIFT	4
	#define SAR_ADC_AVG_CNTL_NUM_SAMPLES_CHAN2_MASK		GENMASK(5, 4)
	#define SAR_ADC_AVG_CNTL_NUM_SAMPLES_CHAN1_SHIFT	2
	#define SAR_ADC_AVG_CNTL_NUM_SAMPLES_CHAN1_MASK		GENMASK(3, 2)
	#define SAR_ADC_AVG_CNTL_NUM_SAMPLES_CHAN0_SHIFT	0
	#define SAR_ADC_AVG_CNTL_NUM_SAMPLES_CHAN0_MASK		GENMASK(1, 0)

#define SAR_ADC_REG3						0x0c
	#define SAR_ADC_REG3_CNTL_USE_SC_DLY			BIT(31)
	#define SAR_ADC_REG3_CLK_EN				BIT(30)
	#define SAR_ADC_REG3_BL30_INITIALIZED			BIT(28)
	#define SAR_ADC_REG3_CTRL_CONT_RING_COUNTER_EN		BIT(27)
	#define SAR_ADC_REG3_CTRL_SAMPLING_CLOCK_PHASE		BIT(26)
	#define SAR_ADC_REG3_CTRL_CHAN7_MUX_SEL_SHIFT		23
	#define SAR_ADC_REG3_CTRL_CHAN7_MUX_SEL_MASK		GENMASK(25, 23)
	#define SAR_ADC_REG3_DETECT_EN				BIT(22)
	#define SAR_ADC_REG3_ADC_EN				BIT(21)
	#define SAR_ADC_REG3_PANEL_DETECT_COUNT_SHIFT		18
	#define SAR_ADC_REG3_PANEL_DETECT_COUNT_MASK		GENMASK(20, 18)
	#define SAR_ADC_REG3_PANEL_DETECT_FILTER_TB_SHIFT	16
	#define SAR_ADC_REG3_PANEL_DETECT_FILTER_TB_MASK	GENMASK(17, 16)
	#define SAR_ADC_REG3_ADC_CLK_DIV_SHIFT			10
	#define SAR_ADC_REG3_ADC_CLK_DIV_MASK			GENMASK(15, 10)
	#define SAR_ADC_REG3_BLOCK_DLY_SEL_SHIFT		8
	#define SAR_ADC_REG3_BLOCK_DLY_SEL_MASK			GENMASK(9, 8)
	#define SAR_ADC_REG3_BLOCK_DLY_SHIFT			7
	#define SAR_ADC_REG3_BLOCK_DLY_MASK			GENMASK(7, 0)

#define SAR_ADC_DELAY						0x10
	#define SAR_ADC_DELAY_INPUT_DLY_SEL_SHIFT		24
	#define SAR_ADC_DELAY_INPUT_DLY_SEL_MASK		GENMASK(25, 24)
	#define SAR_ADC_DELAY_INPUT_DLY_CNT_SHIFT		16
	#define SAR_ADC_DELAY_BL30_BUSY				BIT(15)
	#define SAR_ADC_DELAY_KERNEL_BUSY			BIT(14)
	#define SAR_ADC_DELAY_INPUT_DLY_CNT_MASK		GENMASK(23, 16)
	#define SAR_ADC_DELAY_SAMPLE_DLY_SEL_SHIFT		8
	#define SAR_ADC_DELAY_SAMPLEDLY_SEL_MASK		GENMASK(9, 8)
	#define SAR_ADC_DELAY_SAMPLE_DLY_CNT_SHIFT		0
	#define SAR_ADC_DELAY_SAMPLE_DLY_CNT_MASK		GENMASK(7, 0)

#define SAR_ADC_LAST_RD						0x14
	#define SAR_ADC_LAST_RD_LAST_CHANNEL1_SHIFT		16
	#define SAR_ADC_LAST_RD_LAST_CHANNEL1_MASK		GENMASK(23, 16)
	#define SAR_ADC_LAST_RD_LAST_CHANNEL0_SHIFT		0
	#define SAR_ADC_LAST_RD_LAST_CHANNEL0_MASK		GENMASK(9, 0)

#define SAR_ADC_FIFO_RD						0x18
	#define SAR_ADC_FIFO_RD_CHAN_ID_SHIFT			12
	#define SAR_ADC_FIFO_RD_CHAN_ID_MASK			GENMASK(14, 12)
	#define SAR_ADC_FIFO_RD_SAMPLE_VALUE_SHIFT		0
	#define SAR_ADC_FIFO_RD_SAMPLE_VALUE_MASK		GENMASK(9, 0)

#define SAR_ADC_AUX_SW						0x1c
	#define SAR_ADC_AUX_SW_MUX_SEL_CHAN7_SHIFT		23
	#define SAR_ADC_AUX_SW_MUX_SEL_CHAN7_MASK		GENMASK(25, 23)
	#define SAR_ADC_AUX_SW_MUX_SEL_CHAN6_SHIFT		20
	#define SAR_ADC_AUX_SW_MUX_SEL_CHAN6_MASK		GENMASK(22, 20)
	#define SAR_ADC_AUX_SW_MUX_SEL_CHAN5_SHIFT		17
	#define SAR_ADC_AUX_SW_MUX_SEL_CHAN5_MASK		GENMASK(19, 17)
	#define SAR_ADC_AUX_SW_MUX_SEL_CHAN4_SHIFT		14
	#define SAR_ADC_AUX_SW_MUX_SEL_CHAN4_MASK		GENMASK(16, 14)
	#define SAR_ADC_AUX_SW_MUX_SEL_CHAN3_SHIFT		11
	#define SAR_ADC_AUX_SW_MUX_SEL_CHAN3_MASK		GENMASK(13, 11)
	#define SAR_ADC_AUX_SW_MUX_SEL_CHAN2_SHIFT		8
	#define SAR_ADC_AUX_SW_MUX_SEL_CHAN2_MASK		GENMASK(10, 8)
	#define SAR_ADC_AUX_SW_VREF_P_MUX			BIT(6)
	#define SAR_ADC_AUX_SW_VREF_N_MUX			BIT(5)
	#define SAR_ADC_AUX_SW_MODE_SEL				BIT(4)
	#define SAR_ADC_AUX_SW_YP_DRIVE_SW			BIT(3)
	#define SAR_ADC_AUX_SW_XP_DRIVE_SW			BIT(2)
	#define SAR_ADC_AUX_SW_YM_DRIVE_SW			BIT(1)
	#define SAR_ADC_AUX_SW_XM_DRIVE_SW			BIT(0)

#define SAR_ADC_CHAN_10_SW					0x20
	#define SAR_ADC_CHAN_10_SW_CHAN1_MUX_SEL_SHIFT		23
	#define SAR_ADC_CHAN_10_SW_CHAN1_MUX_SEL_MASK		GENMASK(25, 23)
	#define SAR_ADC_CHAN_10_SW_CHAN1_VREF_P_MUX		BIT(22)
	#define SAR_ADC_CHAN_10_SW_CHAN1_VREF_N_MUX		BIT(21)
	#define SAR_ADC_CHAN_10_SW_CHAN1_MODE_SEL		BIT(20)
	#define SAR_ADC_CHAN_10_SW_CHAN1_YP_DRIVE_SW		BIT(19)
	#define SAR_ADC_CHAN_10_SW_CHAN1_XP_DRIVE_SW		BIT(18)
	#define SAR_ADC_CHAN_10_SW_CHAN1_YM_DRIVE_SW		BIT(17)
	#define SAR_ADC_CHAN_10_SW_CHAN1_XM_DRIVE_SW		BIT(16)
	#define SAR_ADC_CHAN_10_SW_CHAN0_MUX_SEL_SHIFT		7
	#define SAR_ADC_CHAN_10_SW_CHAN0_MUX_SEL_MASK		GENMASK(9, 7)
	#define SAR_ADC_CHAN_10_SW_CHAN0_VREF_P_MUX		BIT(6)
	#define SAR_ADC_CHAN_10_SW_CHAN0_VREF_N_MUX		BIT(5)
	#define SAR_ADC_CHAN_10_SW_CHAN0_MODE_SEL		BIT(4)
	#define SAR_ADC_CHAN_10_SW_CHAN0_YP_DRIVE_SW		BIT(3)
	#define SAR_ADC_CHAN_10_SW_CHAN0_XP_DRIVE_SW		BIT(2)
	#define SAR_ADC_CHAN_10_SW_CHAN0_YM_DRIVE_SW		BIT(1)
	#define SAR_ADC_CHAN_10_SW_CHAN0_XM_DRIVE_SW		BIT(0)

#define SAR_ADC_DETECT_IDLE_SW					0x24
	#define SAR_ADC_DETECT_IDLE_SW_DETECT_SW_EN		BIT(26)
	#define SAR_ADC_DETECT_IDLE_SW_DETECT_MODE_SHIFT	23
	#define SAR_ADC_DETECT_IDLE_SW_DETECT_MODE_MASK		GENMASK(25, 23)
	#define SAR_ADC_DETECT_IDLE_SW_DETECT_MODE_VREF_P_MUX	BIT(22)
	#define SAR_ADC_DETECT_IDLE_SW_DETECT_MODE_VREF_N_MUX	BIT(21)
	#define SAR_ADC_DETECT_IDLE_SW_DETECT_MODE_SEL		BIT(20)
	#define SAR_ADC_DETECT_IDLE_SW_DETECT_MODE_YP_DRIVE_SW	BIT(19)
	#define SAR_ADC_DETECT_IDLE_SW_DETECT_MODE_XP_DRIVE_SW	BIT(18)
	#define SAR_ADC_DETECT_IDLE_SW_DETECT_MODE_YM_DRIVE_SW	BIT(17)
	#define SAR_ADC_DETECT_IDLE_SW_DETECT_MODE_XM_DRIVE_SW	BIT(16)
	#define SAR_ADC_DETECT_IDLE_SW_IDLE_MODE_MUX_SEL_SHIFT	7
	#define SAR_ADC_DETECT_IDLE_SW_IDLE_MODE_MUX_SEL_MASK	GENMASK(9, 7)
	#define SAR_ADC_DETECT_IDLE_SW_IDLE_MODE_VREF_P_MUX	BIT(6)
	#define SAR_ADC_DETECT_IDLE_SW_IDLE_MODE_VREF_N_MUX	BIT(5)
	#define SAR_ADC_DETECT_IDLE_SW_IDLE_MODE_SEL		BIT(4)
	#define SAR_ADC_DETECT_IDLE_SW_IDLE_MODE_YP_DRIVE_SW	BIT(3)
	#define SAR_ADC_DETECT_IDLE_SW_IDLE_MODE_XP_DRIVE_SW	BIT(2)
	#define SAR_ADC_DETECT_IDLE_SW_IDLE_MODE_YM_DRIVE_SW	BIT(1)
	#define SAR_ADC_DETECT_IDLE_SW_IDLE_MODE_XM_DRIVE_SW	BIT(0)

#define SAR_ADC_DELTA_10					0x28
	#define SAR_ADC_DELTA_10_TEMP_SEL			BIT(27)
	#define SAR_ADC_DELTA_10_TS_REVE1			BIT(26)
	#define SAR_ADC_DELTA_10_CHAN1_DELTA_VALUE_SHIFT	16
	#define SAR_ADC_DELTA_10_CHAN1_DELTA_VALUE_MASK		GENMASK(25, 16)
	#define SAR_ADC_DELTA_10_TS_REVE0			BIT(15)
	#define SAR_ADC_DELTA_10_TS_C_SHIFT			11
	#define SAR_ADC_DELTA_10_TS_C_MASK			GENMASK(14, 11)
	#define SAR_ADC_DELTA_10_TS_VBG_EN			BIT(10)
	#define SAR_ADC_DELTA_10_CHAN0_DELTA_VALUE_SHIFT	0
	#define SAR_ADC_DELTA_10_CHAN0_DELTA_VALUE_MASK		GENMASK(9, 0)

#define SAR_ADC_CHAN_TEMP_SENSOR	6

#define SAR_ADC_NUM_CHANNELS		ARRAY_SIZE(meson_saradc_iio_channels)

#define SAR_ADC_NOMINAL_SHIFT		12

#define MESON_SAR_ADC_CHAN(_num) {				\
	.type = IIO_VOLTAGE,					\
	.indexed = true,					\
	.channel = _num,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
	.datasheet_name = "CHAN"#_num,				\
}

static const struct iio_chan_spec meson_saradc_iio_channels[] = {
	MESON_SAR_ADC_CHAN(0),
	MESON_SAR_ADC_CHAN(1),
	MESON_SAR_ADC_CHAN(2),
	MESON_SAR_ADC_CHAN(3),
	MESON_SAR_ADC_CHAN(4),
	MESON_SAR_ADC_CHAN(5),
	MESON_SAR_ADC_CHAN(6),
	MESON_SAR_ADC_CHAN(7),
};

enum meson_saradc_avg_mode {
	NO_AVERAGING = 0x0,
	SIMPLE_AVERAGING = 0x1,
	MEDIAN_AVERAGING = 0x2,
};

enum meson_saradc_chan7_mux_sel {
	VSS = 0x0,
	VDD_DIV4 = 0x1,
	VDD_DIV2 = 0x2,
	VDD_MUL3_DIV4 = 0x3,
	VDD = 0x4,
	CHAN7_NUM_MUXES
};

enum meson_saradc_type {
	SAR_ADC_10_BIT,
	SAR_ADC_12_BIT,
};

struct meson_saradc_chan_config {
	int				vref_p_mux:1;
	int				vref_n_mux:1;
	int				mode_sel:1;
	int				yp_drive_sw:1;
	int				xp_drive_sw:1;
	int				ym_drive_sw:1;
	int				xm_drive_sw:1;
	enum meson_saradc_avg_mode	avg_mode;
	uint8_t				avg_num_samples;
};

struct meson_saradc_priv {
	struct regmap				*regmap;
	struct clk				*clk;
	struct clk				*refclk;
	struct completion			completion;
	enum meson_saradc_type			type;
	struct meson_saradc_chan_config		channels[SAR_ADC_NUM_CHANNELS];
	int					ref_val;
	int					ref_nominal;
	int					coef;
};

static const struct regmap_config meson_saradc_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = SAR_ADC_DELTA_10,
};

static int meson_saradc_get_calibrated_value(struct iio_dev *indio_dev,
					     int val)
{
	struct meson_saradc_priv *priv = iio_priv(indio_dev);
	int nominal;

	if (priv->coef > 0 && val > 0) {
		/* coef = ((val - ref_nominal) << 10) / (val - ref_val)
		 * nominal = ((val - ref_val) * coef >> 10) + ref_nominal */
		nominal = (val - priv->ref_val) * priv->coef;
		nominal >>= SAR_ADC_NOMINAL_SHIFT;
		nominal += priv->ref_nominal;
	} else {
		nominal = val;
	}

	if (nominal < 0)
		return 0;
	else if (priv->type == SAR_ADC_10_BIT && nominal > 1023)
		return 1023;
	else if (priv->type == SAR_ADC_12_BIT && nominal > 4095)
		return 4095;
	else
		return nominal;
}

static int meson_saradc_read_sample(struct iio_dev *indio_dev)
{
	return meson_saradc_get_calibrated_value(indio_dev, 0);
}

static int meson_saradc_iio_info_read_raw(struct iio_dev *indio_dev,
					  struct iio_chan_spec const *chan,
					  int *val, int *val2, long mask)
{
	struct meson_saradc_priv *priv = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&indio_dev->mlock);

		reinit_completion(&priv->completion);

		// TODO

		if (!wait_for_completion_timeout(&priv->completion,
						 0xffffff)) {
//			writel_relaxed(0, priv->regs + SARADC_CTRL);
			mutex_unlock(&indio_dev->mlock);
			return -ETIMEDOUT;
		}

		*val = 0xffffff;
		mutex_unlock(&indio_dev->mlock);
		return IIO_VAL_INT;

	default:
		return -EINVAL;
	}
}

static irqreturn_t meson_saradc_isr(int irq, void *data)
{
	struct meson_saradc_priv *priv = data;

	/* Read value */

	/* Clear irq & power down adc */

	complete(&priv->completion);

	return IRQ_HANDLED;
}

static int meson_saradc_init(struct iio_dev *indio_dev)
{
	struct meson_saradc_priv *priv = iio_priv(indio_dev);
	int ret;
	u32 val;

	ret = regmap_read(priv->regmap, SAR_ADC_REG3, &val);
	if (ret)
		return ret;

	if (val & SAR_ADC_REG3_BL30_INITIALIZED) {
		dev_info(&indio_dev->dev, "already initialized by BL30\n");
	} else {
		/* in-kernel initialization not supported yet */
		return -ENOTSUPP;
	}

	/* enable the temperature sensor
	regmap_update_bits(priv->regmap, SAR_ADC_CHAN_10_SW,
			   SAR_ADC_DELTA_10_TEMP_SEL,
			   SAR_ADC_DELTA_10_TEMP_SEL);
	 */

	return 0;
}

static void meson_saradc_calibrate(struct iio_dev *indio_dev)
{
	struct meson_saradc_priv *priv = iio_priv(indio_dev);
	static int nominal[CHAN7_NUM_MUXES] = { 0, 256, 512, 768, 1023 };
	int i, val[CHAN7_NUM_MUXES];

	for (i = 0; i < CHAN7_NUM_MUXES; i++) {
		regmap_update_bits(priv->regmap, SAR_ADC_REG3,
				   SAR_ADC_REG3_CTRL_CHAN7_MUX_SEL_MASK,
				   i << SAR_ADC_REG3_CTRL_CHAN7_MUX_SEL_SHIFT);

		val[i] = 0; // TODO: read sample

		if (val[i] < 0)
			goto cal_end;
	}

	priv->ref_val = val[2];
	priv->ref_nominal = nominal[2];
	if (val[3] > val[1]) {
		priv->coef = nominal[3] - nominal[1];
		priv->coef <<= SAR_ADC_NOMINAL_SHIFT;
		priv->coef /= val[3] - val[1];
	}

cal_end:
	/* clear mux selection */
	regmap_update_bits(priv->regmap, SAR_ADC_REG3,
			   SAR_ADC_REG3_CTRL_CHAN7_MUX_SEL_MASK,
			   SAR_ADC_REG3_CTRL_CHAN7_MUX_SEL_MASK);

	dev_info(&indio_dev->dev, "calibration end: coef=%d\n", priv->coef); // TODO: dbg
}

static const struct iio_info meson_saradc_iio_info = {
	.read_raw = meson_saradc_iio_info_read_raw,
	.driver_module = THIS_MODULE,
};

static const struct of_device_id meson_saradc_of_match[] = {
	{
		.compatible = "amlogic,meson8b-saradc",
		.data = (void *)SAR_ADC_10_BIT,
	}, {
		.compatible = "amlogic,meson-gxbb-saradc",
		.data = (void *)SAR_ADC_10_BIT,
	}, {
		.compatible = "amlogic,meson-gxl-saradc",
		.data = (void *)SAR_ADC_12_BIT,
	},
	{},
};
MODULE_DEVICE_TABLE(of, meson_saradc_of_match);

static int meson_saradc_probe(struct platform_device *pdev)
{
	struct meson_saradc_priv *priv;
	struct iio_dev *indio_dev;
	struct resource *res;
	void __iomem *base;
	const struct of_device_id *match;
	int ret;
	int irq;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*priv));
	if (!indio_dev) {
		dev_err(&pdev->dev, "failed allocating iio device\n");
		return -ENOMEM;
	}

	priv = iio_priv(indio_dev);

	match = of_match_device(meson_saradc_of_match, &pdev->dev);
	priv->type = (enum meson_saradc_type)match->data;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	priv->regmap = devm_regmap_init_mmio(&pdev->dev, base,
					     &meson_saradc_regmap_config);
	if (IS_ERR(priv->regmap))
		return PTR_ERR(priv->regmap);

	init_completion(&priv->completion);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "no irq resource?\n");
		return irq;
	}

	ret = devm_request_irq(&pdev->dev, irq, meson_saradc_isr, 0,
			       dev_name(&pdev->dev), priv);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed requesting irq %d\n", irq);
		return ret;
	}

	priv->refclk = devm_clk_get(&pdev->dev, "refclk");
	if (IS_ERR(priv->refclk)) {
		dev_err(&pdev->dev, "failed to get refclk\n");
		return PTR_ERR(priv->refclk);
	}

	priv->clk = devm_clk_get(&pdev->dev, "saradc");
	if (IS_ERR(priv->clk)) {
		dev_err(&pdev->dev, "failed to get adc clock\n");
		return PTR_ERR(priv->clk);
	}

	ret = clk_prepare_enable(priv->refclk);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to enable refclk\n");
		return ret;
	}

	ret = clk_prepare_enable(priv->clk);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to enable converter clock\n");
		goto err_refclk;
	}

	ret = meson_saradc_init(indio_dev);
	if (ret)
		return ret;

	meson_saradc_calibrate(indio_dev);

	platform_set_drvdata(pdev, indio_dev);

	indio_dev->name = dev_name(&pdev->dev);
	indio_dev->dev.parent = &pdev->dev;
	indio_dev->dev.of_node = pdev->dev.of_node;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &meson_saradc_iio_info;

	indio_dev->channels = meson_saradc_iio_channels;
	indio_dev->num_channels = SAR_ADC_NUM_CHANNELS;

	ret = iio_device_register(indio_dev);
	if (ret)
		goto err_clk;

	return 0;

err_clk:
	clk_disable_unprepare(priv->clk);
err_refclk:
	clk_disable_unprepare(priv->refclk);
	return ret;
}

static int meson_saradc_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	struct meson_saradc_priv *priv = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);
	clk_disable_unprepare(priv->clk);

	return 0;
}

static struct platform_driver meson_saradc_driver = {
	.probe		= meson_saradc_probe,
	.remove		= meson_saradc_remove,
	.driver		= {
		.name	= "meson-saradc",
		.of_match_table = meson_saradc_of_match,
	},
};

module_platform_driver(meson_saradc_driver);

MODULE_AUTHOR("Martin Blumenstingl <martin.blumenstingl@googlemail.com>");
MODULE_DESCRIPTION("Amlogic Meson SARADC driver");
MODULE_LICENSE("GPL v2");
