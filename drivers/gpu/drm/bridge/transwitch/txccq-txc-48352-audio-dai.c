// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 *
 * Based on dw-hdmi-i2s-audio.c:
 *   Copyright (c) 2017 Renesas Solutions Corp.
 *   Kuninori Morimoto <kuninori.morimoto.gx@renesas.com>
 */

#include <linux/bitfield.h>
#include <linux/module.h>
#include <linux/regmap.h>

#include <sound/hdmi-codec.h>

#include "txccq-txc-48352.h"
#include "txccq-txc-48352-audio-dai.h"

#define DRIVER_NAME "txccq-txc-48352-audio-dai"

static u32 txc_48352_audio_dai_calc_audio_n(struct hdmi_codec_params *hparms)
{
	u32 audio_n;

	if (hparms->sample_rate && (hparms->sample_rate % 44100) == 0)
		audio_n = 6272 * (hparms->sample_rate / 44100);
	else
		audio_n = 128 * hparms->sample_rate / 1000;

	if (hparms->cea.coding_type == HDMI_AUDIO_CODING_TYPE_EAC3 ||
	    hparms->cea.coding_type == HDMI_AUDIO_CODING_TYPE_DTS_HD)
		audio_n *= 4;

	return audio_n;
}

static u8 txc_48352_audio_dai_coding_type(struct hdmi_codec_params *hparms)
{
	switch (hparms->cea.coding_type) {
	case HDMI_AUDIO_CODING_TYPE_MLP:
		return TX_AUDIO_CONTROL_AUDIO_PACKET_TYPE_HBR_AUDIO_PACKET;
	case HDMI_AUDIO_CODING_TYPE_DSD:
		return TX_AUDIO_CONTROL_AUDIO_PACKET_TYPE_ONE_BIT_AUDIO;
	case HDMI_AUDIO_CODING_TYPE_DST:
		return TX_AUDIO_CONTROL_AUDIO_PACKET_TYPE_DST_AUDIO_PACKET;
	default:
		return TX_AUDIO_CONTROL_AUDIO_PACKET_TYPE_AUDIO_SAMPLE_PACKET;
	}
}

static int txc_48352_audio_dai_hw_params(struct device *dev, void *data,
					 struct hdmi_codec_daifmt *fmt,
					 struct hdmi_codec_params *hparms)
{
	u8 audio_spdif, audio_i2s, audio_format;
	struct txc_48352_audio_dai *audio = data;
	u8 buf[HDMI_INFOFRAME_SIZE(AUDIO)];
	u32 audio_n;
	int len, i;

	if (hparms->cea.coding_type == HDMI_AUDIO_CODING_TYPE_MLP) {
		/*
		 * TODO: fixed CTS is not supported yet, it needs special
		 * TX_SYS1_ACR_N_* settings
		 */
		return -EINVAL;
	}

	audio_n = txc_48352_audio_dai_calc_audio_n(hparms);

	switch (fmt->fmt) {
	case HDMI_I2S:
		audio_format = TX_AUDIO_FORMAT_SPDIF_OR_I2S |
			       FIELD_PREP(TX_AUDIO_FORMAT_I2S_FORMAT, 0x2) |
			       TX_AUDIO_FORMAT_I2S_ONE_BIT_OR_I2S;

		if (hparms->channels > 2)
			audio_format |= TX_AUDIO_FORMAT_I2S_2_OR_8_CH;

		audio_i2s = TX_AUDIO_I2S_ENABLE;
		audio_spdif = 0x0;
		break;

	case HDMI_SPDIF:
		if (hparms->cea.coding_type == HDMI_AUDIO_CODING_TYPE_PCM)
			audio_format = 0x0;
		else
			audio_format = TX_AUDIO_FORMAT_SPDIF_CHANNEL_STATUS_FROM_DATA_OR_REG;

		audio_i2s = 0x0;
		audio_spdif = TX_AUDIO_SPDIF_ENABLE;
		break;

	default:
		return -EINVAL;
	}

	switch (hparms->sample_width) {
	case 16:
		audio_format |= FIELD_PREP(TX_AUDIO_FORMAT_BIT_WIDTH_MASK,
					   TX_AUDIO_FORMAT_BIT_WIDTH_16);
		break;

	case 20:
		audio_format |= FIELD_PREP(TX_AUDIO_FORMAT_BIT_WIDTH_MASK,
					   TX_AUDIO_FORMAT_BIT_WIDTH_20);
		break;

	case 24:
		audio_format |= FIELD_PREP(TX_AUDIO_FORMAT_BIT_WIDTH_MASK,
					   TX_AUDIO_FORMAT_BIT_WIDTH_24);
		break;

	default:
		return -EINVAL;
	}

	regmap_write(audio->regmap, TX_AUDIO_FORMAT, audio_format);

	if (hparms->channels > 2)
		regmap_write(audio->regmap, TX_AUDIO_HEADER,
			     TX_AUDIO_HEADER_AUDIO_SAMPLE_PACKET_HEADER_LAYOUT);
	else
		regmap_write(audio->regmap, TX_AUDIO_HEADER, 0x0);

	regmap_write(audio->regmap, TX_AUDIO_SAMPLE,
		     FIELD_PREP(TX_AUDIO_SAMPLE_CHANNEL_VALID,
				BIT(hparms->channels) - 1));

	regmap_write(audio->regmap, TX_SYS1_ACR_N_0,
		     FIELD_PREP(TX_SYS1_ACR_N_0_N_BYTE0,
				(audio_n >> 0) & 0xff));
	regmap_write(audio->regmap, TX_SYS1_ACR_N_1,
		     FIELD_PREP(TX_SYS1_ACR_N_1_N_BYTE1,
				(audio_n >> 8) & 0xff));
	regmap_update_bits(audio->regmap, TX_SYS1_ACR_N_2,
			   TX_SYS1_ACR_N_2_N_UPPER_NIBBLE,
			   FIELD_PREP(TX_SYS1_ACR_N_2_N_UPPER_NIBBLE,
				      (audio_n >> 16) & 0xf));

	regmap_write(audio->regmap, TX_SYS0_ACR_CTS_0, 0x0);
	regmap_write(audio->regmap, TX_SYS0_ACR_CTS_1, 0x0);
	regmap_write(audio->regmap, TX_SYS0_ACR_CTS_2,
		     TX_SYS0_ACR_CTS_2_FORCE_ARC_STABLE);

	regmap_write(audio->regmap, TX_AUDIO_CONTROL,
		     TX_AUDIO_CONTROL_AUTO_AUDIO_FIFO_CLEAR |
		     FIELD_PREP(TX_AUDIO_CONTROL_AUDIO_PACKET_TYPE_MASK,
			        txc_48352_audio_dai_coding_type(hparms)) |
		     TX_AUDIO_CONTROL_AUDIO_SAMPLE_PACKET_FLAT);

	regmap_write(audio->regmap, TX_AUDIO_I2S, audio_i2s);
	regmap_write(audio->regmap, TX_AUDIO_SPDIF, audio_spdif);

	len = hdmi_audio_infoframe_pack(&hparms->cea, buf, sizeof(buf));
	if (len < 0)
		return len;

	audio->write_infoframe(audio->regmap, TX_PKT_REG_AUDIO_INFO_BASE_ADDR,
			       buf, len, true);

	for (i = 0; i < ARRAY_SIZE(hparms->iec.status); i++) {
		if (i == 3) {
			regmap_write(audio->regmap,
				     TX_IEC60958_SUB1_OFFSET + i,
				     hparms->iec.status[i] |
				     FIELD_PREP(IEC958_AES2_CON_CHANNEL, 1));
			regmap_write(audio->regmap,
				     TX_IEC60958_SUB2_OFFSET + i,
				     hparms->iec.status[i] |
				     FIELD_PREP(IEC958_AES2_CON_CHANNEL, 2));
		} else {
			regmap_write(audio->regmap,
				     TX_IEC60958_SUB1_OFFSET + i,
				     hparms->iec.status[i]);
			regmap_write(audio->regmap,
				     TX_IEC60958_SUB2_OFFSET + i,
				     hparms->iec.status[i]);
		}
	}

	return 0;
}

static int txc_48352_audio_dai_audio_startup(struct device *dev, void *data)
{
	struct txc_48352_audio_dai *audio = data;

	regmap_write(audio->regmap, TX_AUDIO_CONTROL_MORE,
		     TX_AUDIO_CONTROL_MORE_ENABLE);

	regmap_write(audio->regmap, TX_AUDIO_FIFO,
		     FIELD_PREP(TX_AUDIO_FIFO_FIFO_DEPTH_MASK,
				TX_AUDIO_FIFO_FIFO_DEPTH_512) |
		     FIELD_PREP(TX_AUDIO_FIFO_CRITICAL_THRESHOLD_MASK,
			        TX_AUDIO_FIFO_CRITICAL_THRESHOLD_DEPTH_DIV16) |
		     FIELD_PREP(TX_AUDIO_FIFO_NORMAL_THRESHOLD_MASK,
			        TX_AUDIO_FIFO_NORMAL_THRESHOLD_DEPTH_DIV8));

	regmap_write(audio->regmap, TX_AUDIO_LIPSYNC, 0x0);

	regmap_write(audio->regmap, TX_SYS1_ACR_N_2,
		     FIELD_PREP(TX_SYS1_ACR_N_2_N_MEAS_TOLERANCE, 0x3));

	return 0;
}

static void txc_48352_audio_dai_audio_shutdown(struct device *dev, void *data)
{
	struct txc_48352_audio_dai *audio = data;

	regmap_write(audio->regmap, TX_AUDIO_CONTROL_MORE, 0x0);
}

static int txc_48352_audio_dai_mute_stream(struct device *dev, void *data,
					   bool enable, int direction)
{
	struct txc_48352_audio_dai *audio = data;

	regmap_write(audio->regmap, TX_AUDIO_PACK,
		     enable ? 0 : TX_AUDIO_PACK_AUDIO_SAMPLE_PACKETS_ENABLE);

	return 0;
}

static int txc_48352_audio_dai_get_dai_id(struct snd_soc_component *component,
					  struct device_node *endpoint)
{
	struct of_endpoint of_ep;
	int ret;

	ret = of_graph_parse_endpoint(endpoint, &of_ep);
	if (ret < 0)
		return ret;

	/*
	 * HDMI sound should be located as reg = <2>
	 * Then, it is sound port 0
	 */
	if (of_ep.port == 2)
		return 0;

	return -EINVAL;
}

static struct hdmi_codec_ops txc_48352_audio_dai_ops = {
	.hw_params	= txc_48352_audio_dai_hw_params,
	.audio_startup  = txc_48352_audio_dai_audio_startup,
	.audio_shutdown	= txc_48352_audio_dai_audio_shutdown,
	.mute_stream	= txc_48352_audio_dai_mute_stream,
	.get_dai_id	= txc_48352_audio_dai_get_dai_id,
};

static int txc_48352_audio_dai_probe(struct platform_device *pdev)
{
	struct txc_48352_audio_dai *audio = dev_get_platdata(&pdev->dev);
	struct platform_device *hdmi_codec_pdev;
	struct hdmi_codec_pdata pdata = {
		.ops			= &txc_48352_audio_dai_ops,
		.i2s			= 1,
		.spdif			= 1,
		.max_i2s_channels	= 8,
		.data			= audio,
	};
	struct platform_device_info pdevinfo = {
		.parent		= pdev->dev.parent,
		.id		= PLATFORM_DEVID_AUTO,
		.name		= HDMI_CODEC_DRV_NAME,
		.data		= &pdata,
		.size_data	= sizeof(pdata)
	};

	hdmi_codec_pdev = platform_device_register_full(&pdevinfo);
	if (IS_ERR(hdmi_codec_pdev))
		return PTR_ERR(hdmi_codec_pdev);

	dev_set_drvdata(&pdev->dev, hdmi_codec_pdev);

	return 0;
}

static int txc_48352_audio_dai_remove(struct platform_device *pdev)
{
	struct platform_device *hdmi_codec_pdev = dev_get_drvdata(&pdev->dev);

	platform_device_unregister(hdmi_codec_pdev);

	return 0;
}

static struct platform_driver txc_48352_audio_dai_driver = {
	.probe	= txc_48352_audio_dai_probe,
	.remove	= txc_48352_audio_dai_remove,
	.driver	= {
		.name = DRIVER_NAME,
	},
};
module_platform_driver(txc_48352_audio_dai_driver);

MODULE_AUTHOR("Martin Blumenstingl <martin.blumenstingl@googlemail.com>");
MODULE_DESCRIPTION("TranSwitch TXC-48352 HDMI 1.4 Transmitter audio driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRIVER_NAME);
