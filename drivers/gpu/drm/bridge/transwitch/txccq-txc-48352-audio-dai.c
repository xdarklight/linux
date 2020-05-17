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

static u32 txc_48352_audio_dai_calc_audio_n(struct hdmi_codec_daifmt *fmt,
					    struct hdmi_codec_params *hparms)
{
	u32 audio_n = 6272;
#if 0
	switch(audio_param->sample_rate) {
	case FS_48K:
		audio_N_para = 6144 * 2;
		if((hdmitx_device->cur_VIC == HDMI_4k2k_24) || (hdmitx_device->cur_VIC == HDMI_4k2k_25)
		|| (hdmitx_device->cur_VIC == HDMI_4k2k_30) || (hdmitx_device->cur_VIC == HDMI_4k2k_smpte_24)) {
		audio_N_para = 5120;
		}
		break;
	case FS_32K:
		audio_N_para = 4096;
		break;
	case FS_44K1:
		audio_N_para = 6272 * 2;
		if((hdmitx_device->cur_VIC == HDMI_4k2k_24) || (hdmitx_device->cur_VIC == HDMI_4k2k_25)
		|| (hdmitx_device->cur_VIC == HDMI_4k2k_30) || (hdmitx_device->cur_VIC == HDMI_4k2k_smpte_24)) {
		audio_N_para = 4704;
		}
		break;
	case FS_88K2:
		audio_N_para = 12544;
		if((hdmitx_device->cur_VIC == HDMI_4k2k_24) || (hdmitx_device->cur_VIC == HDMI_4k2k_25)
		|| (hdmitx_device->cur_VIC == HDMI_4k2k_30) || (hdmitx_device->cur_VIC == HDMI_4k2k_smpte_24)) {
		audio_N_para = 9408;
		}
		break;
	case FS_176K4:
		audio_N_para = 25088;
		if((hdmitx_device->cur_VIC == HDMI_4k2k_24) || (hdmitx_device->cur_VIC == HDMI_4k2k_25)
		|| (hdmitx_device->cur_VIC == HDMI_4k2k_30) || (hdmitx_device->cur_VIC == HDMI_4k2k_smpte_24)) {
		audio_N_para = 18816;
		}
		break;
	case FS_96K:
		audio_N_para = 12288;
		if((hdmitx_device->cur_VIC == HDMI_4k2k_24) || (hdmitx_device->cur_VIC == HDMI_4k2k_25)
		|| (hdmitx_device->cur_VIC == HDMI_4k2k_30) || (hdmitx_device->cur_VIC == HDMI_4k2k_smpte_24)) {
		audio_N_para = 10240;
		}
		break;
	case FS_192K:
		audio_N_para = 24576;
		if((hdmitx_device->cur_VIC == HDMI_4k2k_24) || (hdmitx_device->cur_VIC == HDMI_4k2k_25)
		|| (hdmitx_device->cur_VIC == HDMI_4k2k_30) || (hdmitx_device->cur_VIC == HDMI_4k2k_smpte_24)) {
		audio_N_para = 20480;
		}
		break;
	default:
		audio_N_para = 6144 * 2;
		break;
	}
#endif

	return audio_n;
}

static int txc_48352_audio_dai_hw_params(struct device *dev, void *data,
					 struct hdmi_codec_daifmt *fmt,
					 struct hdmi_codec_params *hparms)
{
	u8 audio_spdif, audio_i2s, audio_format, audio_header = 0x0;
	struct txc_48352_audio_dai *audio = data;

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

	// TODO: hdmitx_set_aud_pkt_type (@TX_AUDIO_CONTROL[5:4])
	// TODO: audio_n

	regmap_write(audio->regmap, TX_AUDIO_I2S, audio_i2s);
	regmap_write(audio->regmap, TX_AUDIO_SPDIF, audio_spdif);

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

	regmap_write(audio->regmap, TX_SYS0_ACR_CTS_0, 0x0);
        regmap_write(audio->regmap, TX_SYS0_ACR_CTS_1, 0x0);
	regmap_write(audio->regmap, TX_SYS0_ACR_CTS_2,
		     TX_SYS0_ACR_CTS_2_FORCE_ARC_STABLE);

	regmap_write(audio->regmap, TX_SYS1_ACR_N_2,
		     FIELD_PREP(TX_SYS1_ACR_N_2_N_MEAS_TOLERANCE, 0x3));

	regmap_update_bits(audio->regmap, TX_AUDIO_CONTROL,
			   TX_AUDIO_CONTROL_AUDIO_SAMPLE_PACKET_FLAT,
			   TX_AUDIO_CONTROL_AUDIO_SAMPLE_PACKET_FLAT);

	return 0;
}

static void txc_48352_audio_dai_audio_shutdown(struct device *dev, void *data)
{
	struct txc_48352_audio_dai *audio = data;

	regmap_write(audio->regmap, TX_AUDIO_CONTROL_MORE, 0x0);
}

static int txc_48352_audio_dai_digital_mute(struct device *dev, void *data,
					    bool enable)
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
	.digital_mute	= txc_48352_audio_dai_digital_mute,
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
