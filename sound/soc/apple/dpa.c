// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Driver for DisplayPort audio transmitter circuitry 
 * on Apple SoCs (?)
 *
 * Copyright (C) The Asahi Linux Contributors
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/dmaengine.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/dmaengine_pcm.h>

struct apple_dpa_data {
	struct snd_dmaengine_dai_dma_data playback;
};

static int apple_dpa_dai_probe(struct snd_soc_dai *dai)
{
	struct apple_dpa_data *dpa = snd_soc_dai_get_drvdata(dai);

	snd_soc_dai_init_dma_data(dai, &dpa->playback, NULL);

	return 0;
}

static const struct snd_soc_dai_ops apple_dpa_dai_ops = {
};

static struct snd_soc_dai_driver apple_dpa_dai = {
	.probe = apple_dpa_dai_probe,
	.name = "DPA",
	.playback = {
		.channels_min = 1,
		.channels_max = 8,
		.rates = SNDRV_PCM_RATE_8000_192000,
		.formats = SNDRV_PCM_FMTBIT_S24_LE | \
			   SNDRV_PCM_FMTBIT_S32_LE,
	},
	.ops = &apple_dpa_dai_ops,
};

static const struct snd_soc_component_driver apple_dpa_component = {
	.name = "apple-dpa",
};

static int apple_dpa_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct apple_dpa_data *data;
	int ret;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	platform_set_drvdata(pdev, data);

	ret = devm_snd_soc_register_component(&pdev->dev, &apple_dpa_component,
					      &apple_dpa_dai, 1);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "registering ASoC component\n");

	ret = devm_snd_dmaengine_pcm_register(&pdev->dev, NULL, 0);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "registering PCM\n");

 	return 0;
}

static const struct of_device_id dpa_of_match[] = {
    { .compatible = "apple,dpa", },
    { }
};
MODULE_DEVICE_TABLE(of, dpa_of_match);

static struct platform_driver apple_dpa_driver = {
    .driver = {
	.name = "apple-dpa",
	.of_match_table = dpa_of_match,
    },
    .probe = apple_dpa_probe,
};
module_platform_driver(apple_dpa_driver);

MODULE_AUTHOR("Martin Povišer <povik+lin@cutebit.org>");
MODULE_DESCRIPTION("Driver for DPA on Apple SoCs");
MODULE_LICENSE("Dual MIT/GPL");
