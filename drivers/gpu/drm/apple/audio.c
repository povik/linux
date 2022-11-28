// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * DCP Audio Bits
 *
 * Copyright (C) The Asahi Linux Contributors
 *
 * TODO:
 *  - figure some nice identification of the sound card (in case
 *    there's many DCP instances)
 */

#include <linux/device.h>
#include <linux/platform_device.h>
#include <sound/hdmi-codec.h>
#include <sound/soc.h>

#include "dcp-internal.h"

int dcp_audio_prepare(struct device *dev, void *data, struct hdmi_codec_daifmt *fmt,
		      struct hdmi_codec_params *hparms)
{
	/* TODO: fill me! */

	dev_info(dev, "%s\n", __func__);
	return 0;
}

void dcp_audio_shutdown(struct device *dev, void *data)
{
	/* TODO: fill me! */

	dev_info(dev, "%s\n", __func__);
}

static const struct hdmi_codec_ops dcp_hdmi_codec_ops = {
	.prepare = dcp_audio_prepare,
	.audio_shutdown = dcp_audio_shutdown,
	/* TODO: fill me! */
};

static struct hdmi_codec_pdata dcp_hdmi_codec_pdata = {
	.ops = &dcp_hdmi_codec_ops,
	.max_i2s_channels = 8,
	.i2s = 1,
};

SND_SOC_DAILINK_DEFS(dcp_dpaudio,
	DAILINK_COMP_ARRAY(COMP_EMPTY()), // CPU
	DAILINK_COMP_ARRAY({ .dai_name = "i2s-hifi", }), // CODEC
	DAILINK_COMP_ARRAY(COMP_EMPTY())); // platform (filled at runtime)

static struct snd_soc_dai_link dcp_dai_link[] = {
	{
		.name = "DP Audio",
		.stream_name = "DP Audio PCM",
		SND_SOC_DAILINK_REG(dcp_dpaudio),
	},
};

int dcp_probe_audio(struct apple_dcp *dcp)
{
	struct device *dev = dcp->dev;
	struct device_node *dpaudio_node;
	struct platform_device *codec_pdev;
	struct snd_soc_dai_link *dai_link;
	struct snd_soc_card *card;
	int ret;

	dpaudio_node = of_parse_phandle(dev->of_node, "apple,audio-xmitter", 0);
	if (!dpaudio_node) {
		dev_info(dev, "No audio support\n");
		return 0;
	}

	codec_pdev = platform_device_register_data(dev, HDMI_CODEC_DRV_NAME,
						   PLATFORM_DEVID_AUTO,
						   &dcp_hdmi_codec_pdata,
						   sizeof(dcp_hdmi_codec_pdata));
	if (IS_ERR(codec_pdev))
		return dev_err_probe(dev, PTR_ERR(codec_pdev), "registering HDMI codec\n");

	dcp->audio.codec_pdev = codec_pdev;

	dai_link = devm_kmemdup(dev, dcp_dai_link, sizeof(*dai_link), GFP_KERNEL);
	if (!dai_link)
		return -ENOMEM;

	dai_link->cpus = devm_kmemdup(dev, dai_link->cpus,
				sizeof(*dai_link->cpus) * dai_link->num_cpus,
				GFP_KERNEL);
	dai_link->codecs = devm_kmemdup(dev, dai_link->codecs,
				sizeof(*dai_link->codecs) * dai_link->num_codecs,
				GFP_KERNEL);
	dai_link->platforms = devm_kmemdup(dev, dai_link->platforms,
				sizeof(*dai_link->platforms) * dai_link->num_platforms,
				GFP_KERNEL);

	if (!dai_link->cpus || !dai_link->codecs || !dai_link->platforms)
		return -ENOMEM;

	/*
	 * ASoC will consider this an I2S connection of CPU and CODEC DAI.
	 * That's not the case, but still -- we want to apply the ASoC scheme
	 * of a CPU/CODEC split as that models the situation rather nicely.
	 * Only the nature of the connection between the two parts is
	 * something opaque to us rather than being I2S.
	 *
	 * The CPU side here is the 'dpaudio' device that's a platform device
	 * of its own bound to a DT node of its own. It is where we hand off
	 * the samples for shipping, and behind the scenes they get to their
	 * destination.
	 *
	 * The CODEC side is roughly the DCP. It's where we deal with the plug
	 * detection, channel mapping, format negotiation and such. It's closer
	 * to the final consument of the samples. We spawn an off-the-shelf ASoC
	 * 'HDMI codec' as a platform device under DCP, and supply it with a bunch
	 * of callbacks. Then we pass the codec to ASoC to be the CODEC side.
	 *
	 * Below, we are linking the two sides up in a 'DAI link' and this
	 * single DAI link then forms an ASoC sound card.
	 */
	dai_link->codecs->name = dev_name(&codec_pdev->dev);
	dai_link->cpus->of_node = dpaudio_node;
	dai_link->platforms->of_node = dpaudio_node;

	card = devm_kzalloc(dev, sizeof(*card), GFP_KERNEL);
	card->dai_link = dai_link;
	card->num_links = 1;
	card->name = "Apple DCP DisplayPort";
	card->driver_name = "apple-dcp";
	card->dev = dev;
	card->owner = THIS_MODULE;

	snd_soc_card_set_drvdata(card, dcp);
	ret = devm_snd_soc_register_card(dev, card);
	if (ret)
		return dev_err_probe(dev, ret, "registering sound card\n");

	/*
	 * This angers me a bit. ASoC wants to take over the device's drvdata.
	 * Let's see if we can get away with resetting it back.
	 */
	platform_set_drvdata(to_platform_device(dcp->dev), dcp);

	return 0;
}
