// SPDX-License-Identifier: GPL-2.0-only
/*
 * Driver for the Cirrus CS4334 - or functionally compatible - I2S Stereo DACs.
 *
 * Copyright (C) 2021 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 *
 * Based on es7134.c:
 *   Copyright (c) 2017 BayLibre, SAS.
 *   Author: Jerome Brunet <jbrunet@baylibre.com>
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <sound/soc.h>

struct cs4334_clock_mode {
	unsigned int rate_min;
	unsigned int rate_max;
	unsigned int *mclk_fs;
	unsigned int mclk_fs_num;
};

struct cs4334_chip {
	struct snd_soc_dai_driver *dai_drv;
	const struct cs4334_clock_mode *modes;
	unsigned int mode_num;
};

struct cs4334_data {
	unsigned int mclk;
	const struct cs4334_chip *chip;
};

static int cs4334_hw_params(struct snd_pcm_substream *substream,
			    struct snd_pcm_hw_params *params,
			    struct snd_soc_dai *dai)
{
	struct cs4334_data *priv = snd_soc_dai_get_drvdata(dai);
	unsigned int mfs, rate = params_rate(params);
	int i, j;

	/* mclk has not been provided, assume it is OK */
	if (!priv->mclk)
		return 0;

	mfs = priv->mclk / rate;

	for (i = 0; i < priv->chip->mode_num; i++) {
		const struct cs4334_clock_mode *mode = &priv->chip->modes[i];

		if (rate < mode->rate_min || rate > mode->rate_max)
			continue;

		for (j = 0; j < mode->mclk_fs_num; j++) {
			if (mode->mclk_fs[j] == mfs)
				return 0;
		}

		dev_err(dai->dev, "unsupported mclk_fs %u for rate %u\n",
			mfs, rate);
		return -EINVAL;
	}

	/* should not happen */
	dev_err(dai->dev, "unsupported rate: %u\n", rate);
	return -EINVAL;
}

static int cs4334_set_sysclk(struct snd_soc_dai *dai, int clk_id,
			     unsigned int freq, int dir)
{
	struct cs4334_data *priv = snd_soc_dai_get_drvdata(dai);

	if (dir == SND_SOC_CLOCK_IN && clk_id == 0) {
		priv->mclk = freq;
		return 0;
	}

	return -ENOTSUPP;
}

static int cs4334_set_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{
	fmt &= (SND_SOC_DAIFMT_FORMAT_MASK | SND_SOC_DAIFMT_INV_MASK |
		SND_SOC_DAIFMT_MASTER_MASK);

	if (fmt != (SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
		    SND_SOC_DAIFMT_CBC_CFC)) {
		dev_err(codec_dai->dev, "Invalid DAI format\n");
		return -EINVAL;
	}

	return 0;
}

static const struct snd_soc_dai_ops cs4334_dai_ops = {
	.set_fmt = cs4334_set_fmt,
	.hw_params = cs4334_hw_params,
	.set_sysclk = cs4334_set_sysclk,
};

static struct snd_soc_dai_driver cs4334_dai = {
	.name = "cs4334-hifi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_32000 |
			 SNDRV_PCM_RATE_44100 |
			 SNDRV_PCM_RATE_48000 |
			 SNDRV_PCM_RATE_64000 |
			 SNDRV_PCM_RATE_88200 |
			 SNDRV_PCM_RATE_96000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE |
			   SNDRV_PCM_FMTBIT_S24_LE,
	},
	.ops = &cs4334_dai_ops,
};

static const struct cs4334_clock_mode cs4334_modes[] = {
	{
		/* Base Rate Mode (BRM) */
		.rate_min = 32000,
		.rate_max = 48000,
		.mclk_fs = (unsigned int[]) { 256, 384, 512 },
		.mclk_fs_num = 3,
	},
	{
		/* High Rate Mode (HRM) */
		.rate_min = 32000,
		.rate_max = 96000,
		.mclk_fs = (unsigned int[]) { 128, 192 },
		.mclk_fs_num = 2,
	},
};

static const struct cs4334_chip cs4334_chip = {
	.dai_drv = &cs4334_dai,
	.modes = cs4334_modes,
	.mode_num = ARRAY_SIZE(cs4334_modes),
};

static const struct snd_soc_dapm_widget cs4334_dapm_widgets[] = {
	SND_SOC_DAPM_REGULATOR_SUPPLY("VA", 0, 0),
};

static const struct snd_soc_component_driver cs4334_component_driver = {
	.dapm_widgets		= cs4334_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(cs4334_dapm_widgets),
	.idle_bias_on		= 1,
	.use_pmdown_time	= 1,
	.endianness		= 1,
};

static int cs4334_probe(struct platform_device *pdev)
{
	struct cs4334_data *priv;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	platform_set_drvdata(pdev, priv);

	priv->chip = of_device_get_match_data(&pdev->dev);
	if (!priv->chip) {
		dev_err(&pdev->dev, "Failed to find OF match data\n");
		return -ENODEV;
	}

	return devm_snd_soc_register_component(&pdev->dev,
					       &cs4334_component_driver,
					       priv->chip->dai_drv, 1);
}

static const struct of_device_id cs4334_of_match[] = {
	{ .compatible = "cirrus,cs4334", .data = &cs4334_chip },
	{ .compatible = "mxtronics,mxt8234", .data = &cs4334_chip },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, cs4334_of_match);

static struct platform_driver cs4334_codec_driver = {
	.probe = cs4334_probe,
	.driver = {
		.name = "cs4334-codec",
		.of_match_table = cs4334_of_match,
	},
};

module_platform_driver(cs4334_codec_driver);

MODULE_DESCRIPTION("ASoC Cirrus CS4334 codec driver");
MODULE_AUTHOR("Martin Blumenstingl <martin.blumenstingl@googlemail.com>");
MODULE_LICENSE("GPL v2");
