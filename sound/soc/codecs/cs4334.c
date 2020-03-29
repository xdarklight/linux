// SPDX-License-Identifier: GPL-2.0-only
/*
 * Driver for the Cirrus CS4334 - or functionally compatible - I2S Stereo DACs.
 *
 * Copyright (C) 2019 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 *
 * Based on pcm5102a:
 *   Copyright (C) 2013 Florian Meier <florian.meier@koalo.de>
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <sound/soc.h>

static struct snd_soc_dai_driver cs4334_dai = {
	.name = "cs4334",
	.playback = {
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_96000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE |
			   SNDRV_PCM_FMTBIT_S24_LE
	},
};

static const struct snd_soc_dapm_widget cs4334_dapm_widgets[] = {
	SND_SOC_DAPM_OUTPUT("AOUTL"),
	SND_SOC_DAPM_OUTPUT("AOUTR"),
	SND_SOC_DAPM_DAC("DAC", "Playback", SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_REGULATOR_SUPPLY("VDD", 0, 0),
};

static const struct snd_soc_dapm_route cs4334_dapm_routes[] = {
	{ "AOUTL", NULL, "DAC" },
	{ "AOUTR", NULL, "DAC" },
	{ "DAC", NULL, "VDD" },
};

static struct snd_soc_component_driver soc_component_dev_cs4334 = {
	.dapm_widgets		= cs4334_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(cs4334_dapm_widgets),
	.dapm_routes		= cs4334_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(cs4334_dapm_routes),
	.idle_bias_on		= 1,
	.use_pmdown_time	= 1,
	.endianness		= 1,
	.non_legacy_dai_naming	= 1,
};

static int cs4334_probe(struct platform_device *pdev)
{
	return devm_snd_soc_register_component(&pdev->dev,
					       &soc_component_dev_cs4334,
					       &cs4334_dai, 1);
}

static const struct of_device_id cs4334_of_match[] = {
	{ .compatible = "cirrus,cs4334" },
	{ .compatible = "mxtronics,mxt8234" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, cs4334_of_match);

static struct platform_driver cs4334_codec_driver = {
	.probe		= cs4334_probe,
	.driver		= {
		.name	= "cs4334-codec",
		.of_match_table = cs4334_of_match,
	},
};

module_platform_driver(cs4334_codec_driver);

MODULE_DESCRIPTION("ASoC Cirrus CS4334 codec driver");
MODULE_AUTHOR("Martin Blumenstingl <martin.blumenstingl@googlemail.com>");
MODULE_LICENSE("GPL v2");
