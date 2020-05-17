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

static bool txc_48352_audio_dai_is_hdmi14_4k_mode(void)
{
	// TODO: not supported yet
	return false;
}

static u32 txc_48352_audio_dai_calc_audio_n(struct hdmi_codec_daifmt *fmt,
					    struct hdmi_codec_params *hparms)
{
	switch (hparms->sample_rate) {
	case 32000:
		return 4096;
	case 44100:
		if (txc_48352_audio_dai_is_hdmi14_4k_mode())
			return 4704;
		return 6272 * 2;

	case 48000:
		if (txc_48352_audio_dai_is_hdmi14_4k_mode())
			return 5120;
		return 6144 * 2;

	case 88200:
		if (txc_48352_audio_dai_is_hdmi14_4k_mode())
			return 9408;

		return 12544;

	case 96000:
		if (txc_48352_audio_dai_is_hdmi14_4k_mode())
			return 10240;
		return 12288;

	case 176400:
		if (txc_48352_audio_dai_is_hdmi14_4k_mode())
			return 18816;
		return 25088;

	case 192000:
		if (txc_48352_audio_dai_is_hdmi14_4k_mode())
			return 20480;
		return 24576;

	default:
		return 6144 * 2;
	}
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
	u8 audio_spdif, audio_i2s, audio_format, audio_header = 0x0;
	struct txc_48352_audio_dai *audio = data;
	u8 buf[HDMI_INFOFRAME_SIZE(AUDIO)];
	u32 audio_n;
	int len, i;

	switch (fmt->fmt) {
	case HDMI_I2S:
		audio_format = TX_AUDIO_FORMAT_I2S_OR_SPDIF |
				  FIELD_PREP(TX_AUDIO_FORMAT_I2S_FORMAT, 0x2);
		audio_i2s = TX_AUDIO_I2S_ENABLE;
		audio_spdif = 0x0;
		break;

	case HDMI_SPDIF:
		audio_format = 0x0;
		audio_i2s = 0x0;
		audio_spdif = TX_AUDIO_SPDIF_ENABLE;
		break;

	default:
		return -EINVAL;
	}

	if (hparms->channels > 2) {
		audio_format |= TX_AUDIO_FORMAT_I2S_2_OR_8_CH;
		audio_header |= TX_AUDIO_HEADER_AUDIO_SAMPLE_PACKET_HEADER_LAYOUT;
	}

	audio_format |= FIELD_PREP(TX_AUDIO_FORMAT_BIT_WIDTH,
				      hparms->sample_width / 8);
	audio_format |= TX_AUDIO_FORMAT_SPDIF_CHANNEL_STATUS_FROM_DATA_OR_REG;
	regmap_write(audio->regmap, TX_AUDIO_FORMAT, audio_format);

	regmap_write(audio->regmap, TX_AUDIO_HEADER, audio_header);

	regmap_write(audio->regmap, TX_AUDIO_SAMPLE,
		     FIELD_PREP(TX_AUDIO_SAMPLE_CHANNEL_VALID,
				BIT(hparms->channels) - 1));

	if (hparms->cea.coding_type == HDMI_AUDIO_CODING_TYPE_MLP) {
		/*
		 * TODO: fixed CTS is not supported yet, needs special
		 * TX_SYS1_ACR_N_* settings
		 */
		return -ENOTSUPP;
	} else {
		audio_n = txc_48352_audio_dai_calc_audio_n(fmt, hparms);

		regmap_write(audio->regmap, TX_SYS1_ACR_N_0,
			     FIELD_PREP(TX_SYS1_ACR_N_0_N_BYTE0,
					(audio_n >> 0) & 0xff));
		regmap_write(audio->regmap, TX_SYS1_ACR_N_1,
			     FIELD_PREP(TX_SYS1_ACR_N_1_N_BYTE1,
					(audio_n >> 8) & 0xff));
		regmap_update_bits(audio->regmap, TX_SYS1_ACR_N_2,
				   TX_SYS1_ACR_N_2_N_UPPER_NIBBLE,
				   FIELD_PREP(TX_SYS1_ACR_N_2_N_UPPER_NIBBLE,
					      (audio_n >> 12) & 0xf));

		regmap_write(audio->regmap, TX_SYS0_ACR_CTS_0, 0x0);
		regmap_write(audio->regmap, TX_SYS0_ACR_CTS_1, 0x0);
		regmap_write(audio->regmap, TX_SYS0_ACR_CTS_2,
			     TX_SYS0_ACR_CTS_2_FORCE_ARC_STABLE);
	}

	regmap_write(audio->regmap, TX_AUDIO_CONTROL,
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

	for (i = 0; i < 24; i++) {
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
		     enable ? TX_AUDIO_PACK_AUDIO_SAMPLE_PACKETS_ENABLE : 0);

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

static int snd_dw_hdmi_probe(struct platform_device *pdev)
{
	struct txc_48352_audio_dai *audio = platform_get_drvdata(pdev);
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

static int snd_dw_hdmi_remove(struct platform_device *pdev)
{
	struct platform_device *hdmi_codec_pdev = dev_get_drvdata(&pdev->dev);

	platform_device_unregister(hdmi_codec_pdev);

	return 0;
}

static struct platform_driver snd_dw_hdmi_driver = {
	.probe	= snd_dw_hdmi_probe,
	.remove	= snd_dw_hdmi_remove,
	.driver	= {
		.name = DRIVER_NAME,
	},
};
module_platform_driver(snd_dw_hdmi_driver);

MODULE_AUTHOR("Martin Blumenstingl <martin.blumenstingl@googlemail.com>");
MODULE_DESCRIPTION("TranSwitch TXC-48352 HDMI 1.4 Transmitter audio driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRIVER_NAME);
