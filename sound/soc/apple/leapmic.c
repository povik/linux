// SPDX-License-Identifier: GPL-2.0-only OR MIT
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/io.h>

#include <sound/pcm.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/soc-topology.h>
#include <sound/dmaengine_pcm.h>

#include "leap.h"
#include "leapshim.h"
#include "leapmic.h"

static char *force_firmware = NULL;
module_param(force_firmware, charp, 0660);

struct leapmic_data {
	struct device *dev;
	struct device_node *link_node;
	struct leap_cluster *leap;
	struct leapshim_data *shim;

	const struct firmware *fw;
};

static int leapmic_set_runtime_hwparams(struct snd_soc_component *component,
				    struct snd_pcm_substream *substream,
				    struct dma_chan *chan)
{
	struct device *dma_dev = chan->device->dev;
	struct snd_dmaengine_dai_dma_data dma_data = {};
	struct snd_pcm_hardware hw = {
		.info = SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID |
			SNDRV_PCM_INFO_INTERLEAVED,
		.periods_min = 2,
		.periods_max = UINT_MAX,
		.period_bytes_min = 256,
		.period_bytes_max = dma_get_max_seg_size(dma_dev),
		.buffer_bytes_max = SIZE_MAX,
		.fifo_size = 16,
	};
	int ret;

	ret = snd_dmaengine_pcm_refine_runtime_hwparams(substream, &dma_data,
							&hw, chan);
	if (ret)
		return ret;

	return snd_soc_set_runtime_hwparams(substream, &hw);
}

static int leapmic_pcm_open(struct snd_soc_component *component,
			    struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = asoc_substream_to_rtd(substream);
	bool is_tx = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;
	struct snd_soc_dai *dai = asoc_rtd_to_cpu(rtd, 0);
	struct dma_chan *chan = is_tx ? dai->playback_dma_data : dai->capture_dma_data;
	int ret;

	if (rtd->dai_link->no_pcm)
		return 0;

	ret = leapmic_set_runtime_hwparams(component, substream, chan);
	if (ret)
		return ret;

	return snd_dmaengine_pcm_open(substream, chan);
}

static int leapmic_pcm_close(struct snd_soc_component *component,
			 struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = asoc_substream_to_rtd(substream);

	if (rtd->dai_link->no_pcm)
		return 0;

	return snd_dmaengine_pcm_close(substream);
}

static int leapmic_trigger(struct snd_soc_component *component,
			   struct snd_pcm_substream *substream, int cmd)
{
	struct snd_soc_pcm_runtime *rtd = asoc_substream_to_rtd(substream);
	struct leapmic_data *lm = snd_soc_component_get_drvdata(component);
	u32 takeout;
	int ret;

	if (rtd->dai_link->no_pcm)
		return 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		ret = snd_dmaengine_pcm_trigger(substream, cmd);
		if (ret)
			return ret;
		leap_io_put(lm->leap, 0x60, 0x80000000);
		return 0;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		leap_io_take(lm->leap, 0x60, &takeout);
		ret = snd_dmaengine_pcm_trigger(substream, cmd);
		if (ret)
			return ret;
		return 0;

	default:
		return snd_dmaengine_pcm_trigger(substream, cmd);
	}
}

static snd_pcm_uframes_t leapmic_pointer(struct snd_soc_component *component,
					 struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = asoc_substream_to_rtd(substream);

	if (rtd->dai_link->no_pcm)
		return -ENOTSUPP;

	return snd_dmaengine_pcm_pointer(substream);
}

static void leapmic_pcm_free(struct snd_soc_component *component,
			     struct snd_pcm *pcm)
{
	struct snd_soc_pcm_runtime *rtd = snd_pcm_chip(pcm);
	struct snd_soc_dai *dai = asoc_rtd_to_cpu(rtd, 0);
	struct leapmic_data *lm = snd_soc_dai_get_drvdata(dai);
	int  i;

	if (rtd->dai_link->no_pcm)
		return;

	for_each_pcm_streams(i) {
		struct snd_pcm_substream *substream =
			rtd->pcm->streams[i].substream;
		bool is_tx = i == SNDRV_PCM_STREAM_PLAYBACK;
		struct dma_chan *chan = is_tx ?
			dai->playback_dma_data : dai->capture_dma_data;

		if (!substream)
			continue;

		leapshim_disable_dma(lm->shim, dai->id);

		if (!chan)
			continue;

		dma_release_channel(chan);
		if (is_tx)
			dai->playback_dma_data = NULL;
		else
			dai->capture_dma_data = NULL;
	}
}

static int leapmic_dai_pcm_new(struct snd_soc_pcm_runtime *rtd,
			       struct snd_soc_dai *dai)
{
	struct leapmic_data *lm = snd_soc_dai_get_drvdata(dai);
	int i;

	if (rtd->dai_link->no_pcm)
		return 0;

	for_each_pcm_streams(i) {
		struct snd_pcm_substream *substream =
			rtd->pcm->streams[i].substream;
		struct dma_chan *chan;

		if (!substream)
			continue;

		leapshim_enable_dma(lm->shim, dai->id);

		chan = leapshim_request_dma_channel(lm->shim, dai->id);
		if (IS_ERR_OR_NULL(chan)) {
			dev_err(lm->dev, "dai %d: failed to obtain DMA channel: %pe\n",
				dai->id, chan);
			leapmic_pcm_free(dai->component, rtd->pcm);
			return -EINVAL;
		}

		if (i == SNDRV_PCM_STREAM_PLAYBACK)
			dai->playback_dma_data = chan;
		else
			dai->capture_dma_data = chan;

		snd_pcm_set_managed_buffer(substream, SNDRV_DMA_TYPE_DEV_IRAM,
					   chan->device->dev, 512 * 1024, SIZE_MAX);
	};

	return 0;
}

static int leapmic_dai_load(struct snd_soc_component *component, int index,
			    struct snd_soc_dai_driver *dai_drv,
			    struct snd_soc_tplg_pcm *pcm, struct snd_soc_dai *dai)
{
	dai_drv->pcm_new = leapmic_dai_pcm_new;

	return 0;
}

#if 0
static int leapmic_link_load(struct snd_soc_component *, int index,
			     struct snd_soc_dai_link *link,
			     struct snd_soc_tplg_link_config *cfg)
{
}
#endif

static int leapmic_tplg_manifest(struct snd_soc_component *component, int index,
				 struct snd_soc_tplg_manifest *manifest)
{
	struct leapmic_data *lm = snd_soc_component_get_drvdata(component);
	u32 size = le32_to_cpu(manifest->priv.size);
	u8 *image = manifest->priv.data;

	leap_load_image(leap_get_core(lm->leap, 1), image, size);

	return 0;
}

static struct snd_soc_tplg_ops leapmic_tplg_ops = {
	.dai_load = leapmic_dai_load,
//	.link_load = leapmic_link_load,
	.manifest = leapmic_tplg_manifest,
};

static const struct firmware *leapmic_request_firmware(struct leapmic_data *lm)
{
	const char *compatible;
	const struct firmware *fw;
	char filename[64];
	int ret, i;

	if (force_firmware) {
		ret = request_firmware(&fw, force_firmware, lm->dev);

		if (ret)
			return NULL;

		return fw;
	}

	/*
	 * Look at the compatibles of the physical mic link (it's under the 'sound'
	 * node) to generate candidate firmware names.
	 */
	for (i = 0;; i++) {
		if (of_property_read_string_index(lm->link_node, "compatible", i,
						  &compatible))
			break;

		snprintf(filename, sizeof(filename) - 1, "leapmic-%s.bin", compatible);
		ret = request_firmware(&fw, filename, lm->dev);

		if (!ret) {
			dev_info(lm->dev, "obtained topology firmware '%s'\n", filename);
			return fw;
		}

		dev_info(lm->dev, "requesting topology firmware '%s': %d\n", filename, ret);
	}

	return NULL;
}

static int leapmic_component_probe(struct snd_soc_component *component)
{
	struct leapmic_data *lm = snd_soc_component_get_drvdata(component);
	const struct firmware *fw;
	int ret;

	fw = leapmic_request_firmware(lm);
	if (!fw) {
		dev_warn(lm->dev, "internal microphone unavailable due to not having loaded any topology firmware\n");
		return 0;
	}

	ret = snd_soc_tplg_component_load(component, &leapmic_tplg_ops, fw);
	if (ret < 0) {
		release_firmware(fw);
		return ret;
	}

	lm->fw = fw;
	return 0;
}

static void leapmic_component_remove(struct snd_soc_component *component)
{
	struct leapmic_data *lm = snd_soc_component_get_drvdata(component);

	if (lm->fw) {
		release_firmware(lm->fw);
		lm->fw = NULL;
	}
}

int leapmic_prepare(struct snd_soc_component *component,
		    struct snd_pcm_substream *substream)
{
	struct leapmic_data *lm = snd_soc_component_get_drvdata(component);

	leap_enable(leap_get_core(lm->leap, 1));

	return 0;
}

int leapmic_hw_free(struct snd_soc_component *component,
		    struct snd_pcm_substream *substream)
{
	struct leapmic_data *lm = snd_soc_component_get_drvdata(component);

	leap_disable(leap_get_core(lm->leap, 1));

	return 0;
}

static const struct snd_soc_component_driver apple_leapmic_component = {
	.name = "apple-leapmic",
	.probe = leapmic_component_probe,
	.remove = leapmic_component_remove,
	.open = leapmic_pcm_open,
	.close = leapmic_pcm_close,
	.pcm_destruct = leapmic_pcm_free,
	.trigger = leapmic_trigger,
	.pointer = leapmic_pointer,
	.prepare = leapmic_prepare,
	.hw_free = leapmic_hw_free,
};

static void leapmic_release(struct leapmic_data *lm)
{
}

static int leapmic_probe(struct platform_device *pdev)
{
	struct platform_device *leapshim_pdev;
	struct device_link *leapshim_link;
	struct leapmic_config *lmc = pdev->dev.platform_data;
	struct leapmic_data *lm;
	int ret;

	lm = devm_kzalloc(&pdev->dev, sizeof(*lm), GFP_KERNEL);
	if (!lm)
		return -ENOMEM;
	lm->dev = &pdev->dev;
	lm->link_node = lmc->link_node;

	leapshim_pdev = of_find_device_by_node(lmc->leapshim_node);
	if (!leapshim_pdev)
		return -ENODEV;

	leapshim_link = device_link_add(&pdev->dev, &leapshim_pdev->dev,
					DL_FLAG_AUTOREMOVE_CONSUMER);
	if (!leapshim_link) {
		dev_err(&pdev->dev, "failed to link to LEAP shim\n");
		return -EINVAL;
	}

	if (leapshim_link->supplier->links.status != DL_DEV_DRIVER_BOUND)
		return -EPROBE_DEFER;

	lm->shim = platform_get_drvdata(leapshim_pdev);
	lm->leap = platform_get_drvdata(leapshim_find_leap_device(lm->shim));

	platform_set_drvdata(pdev, lm);
	ret = snd_soc_register_component(&pdev->dev, &apple_leapmic_component,
					 NULL, 0);
	if (ret) {
		dev_err(&pdev->dev, "unable to register ASoC component: %d\n",
			ret);
		goto err_release;
	}

	return 0;

err_release:
	leapmic_release(lm);
	return ret;
}

static int leapmic_remove(struct platform_device *pdev)
{
	struct leapmic_data *lm = platform_get_drvdata(pdev);

	leapmic_release(lm);
	return 0;
}

static struct platform_driver apple_leapmic_driver = {
	.driver = {
		.name = LEAPMIC_PLATFORM_DRV_NAME,
	},
	.probe = leapmic_probe,
	.remove = leapmic_remove,
};
module_platform_driver(apple_leapmic_driver);

MODULE_AUTHOR("Martin Povišer <povik+lin@cutebit.org>");
MODULE_DESCRIPTION("ASoC Apple LEAP mic platform driver");
MODULE_LICENSE("Dual MIT/GPL");
MODULE_ALIAS("platform:" LEAPMIC_PLATFORM_DRV_NAME);
