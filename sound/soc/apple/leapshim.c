// SPDX-License-Identifier: GPL-2.0-only OR MIT
#include <linux/of.h>
#include <linux/of_dma.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/io.h>

#include <sound/core.h>
#include <sound/soc.h>

#include "leap.h"

#define REG_DMA_CONTROL(id)	(0x690 + 0x4 * (id)) /* TODO: id */

struct leapshim_data {
	struct device *dev;
	__iomem void *base;
};

#define LEAPSHIM_NUM_PDM_DRIVERS 8

int apple_leapshim_pdm_of_xlate_dai_name(struct snd_soc_component *component,
					 const struct of_phandle_args *args,
					 const char **dai_name)
{
	struct snd_soc_dai *dai;
	int id;

	if (args->args_count != 2)
		return -EINVAL;

	id = args->args[0] * 2 + args->args[1];

	for_each_component_dais(component, dai) {
		if (dai->id == id) {
			*dai_name = dai->driver->name;
			return 0;
		}
	}

	return -EINVAL;
}

static const struct snd_soc_dai_ops apple_leapshim_pdm_ops = {
};

static const struct snd_soc_component_driver apple_leapshim_pdm_component = {
	.name = "apple-leap-shim-pdm",
	.of_xlate_dai_name = apple_leapshim_pdm_of_xlate_dai_name,
};

static int apple_leapshim_pdm_probe(struct platform_device *pdev)
{
	struct snd_soc_dai_driver *dai_drivers;
	int ret, i;

	dai_drivers = devm_kzalloc(
		&pdev->dev, sizeof(*dai_drivers) * LEAPSHIM_NUM_PDM_DRIVERS, GFP_KERNEL);

	for (i = 0; i < LEAPSHIM_NUM_PDM_DRIVERS; i++) {
		struct snd_soc_dai_driver *drv = &dai_drivers[i];

		drv->id = i;
		drv->name = devm_kasprintf(&pdev->dev, GFP_KERNEL, "leapshim-pdm-%d-%s", 
					   i / 2, (i % 2 == 0 ? "left" : "right"));
		if (!drv->name) {
			ret = -ENOMEM;
			goto err_release;
		}
		drv->ops = &apple_leapshim_pdm_ops;
		drv->capture.channels_min = 1;
		drv->capture.channels_max = 1;
		drv->capture.rates = SNDRV_PCM_RATE_8000_192000;
		drv->capture.formats = SNDRV_PCM_FMTBIT_FLOAT_LE; /* TODO: n/a */

		drv->capture.stream_name =
			devm_kasprintf(&pdev->dev, GFP_KERNEL, "LEAP PDM%d%s RX", i / 2,
				       (i % 2 == 0 ? "L" : "R"));

		if (!drv->capture.stream_name) {
			ret = -ENOMEM;
			goto err_release;
		}
	}

	ret = snd_soc_register_component(&pdev->dev, &apple_leapshim_pdm_component,
					 dai_drivers, LEAPSHIM_NUM_PDM_DRIVERS);
	if(ret) {
		dev_err(&pdev->dev, "unable to register ASoC component: %d\n",
			ret);
		goto err_release;
	}

	dev_info(&pdev->dev, "leapshim pdm probed!\n");

	return 0;

err_release:
	return ret;
}

static int apple_leapshim_pdm_remove(struct platform_device *pdev)
{
	snd_soc_unregister_component(&pdev->dev);
	return 0;
}

static const struct of_device_id apple_leapshim_pdm_of_match[] = {
	{ .compatible = "apple,t8103-leap-pdm-shim", },
	{}
};
MODULE_DEVICE_TABLE(of, apple_leapshim_pdm_of_match);

static struct platform_driver apple_leapshim_pdm_driver = {
	.driver = {
		.name = "apple-leapshim-pdm",
		.of_match_table = apple_leapshim_pdm_of_match,
	},
	.probe = apple_leapshim_pdm_probe,
	.remove = apple_leapshim_pdm_remove,
};
module_platform_driver(apple_leapshim_pdm_driver);

struct dma_chan *leapshim_request_dma_channel(struct leapshim_data *shim, int id)
{
	struct device_node *node;
	const char *name;

	node = of_get_compatible_child(shim->dev->of_node, "apple,t8103-leap-dma-shim");

	if (!node)
		return ERR_PTR(-ENODEV);

	if (of_property_read_string_index(node, "dma-names", id, &name))
		return ERR_PTR(-EINVAL);

	return of_dma_request_slave_channel(node, name);
}

void leapshim_enable_dma(struct leapshim_data *shim, int id)
{
	writel_relaxed(0x42, shim->base + REG_DMA_CONTROL(id)); /* TODO */
}

void leapshim_disable_dma(struct leapshim_data *shim, int id)
{
	writel_relaxed(0x0, shim->base + REG_DMA_CONTROL(id));
}

static void leapshim_release(struct leapshim_data *shim)
{
}

struct platform_device *leapshim_find_leap_device(struct leapshim_data *shim)
{
	struct device_node *np = NULL;

	for_each_available_child_of_node(shim->dev->of_node, np) {
		struct of_phandle_args args;
		int ret;

		ret = of_parse_phandle_with_args(np, "apple,leap-io-ports", "#apple,leap-io-ports-cells",
						 0, &args);
		if (ret)
			continue;

		return of_find_device_by_node(args.np);
	}

	return NULL;
}

static int leapshim_probe(struct platform_device *pdev)
{
	struct leapshim_data *shim;
	int ret;

	shim = devm_kzalloc(&pdev->dev, sizeof(*shim), GFP_KERNEL);
	if (!shim)
		return -ENOMEM;
	shim->dev = &pdev->dev;
	shim->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(shim->base))
		return PTR_ERR(shim->base);
	platform_set_drvdata(pdev, shim);

	of_platform_populate(pdev->dev.of_node, NULL, NULL, &pdev->dev);

	return 0;

//err_release:
	leapshim_release(shim);
	return ret;
}

static int leapshim_remove(struct platform_device *pdev)
{
	struct leapshim_data *shim = platform_get_drvdata(pdev);

	leapshim_release(shim);
	return 0;
}

static const struct of_device_id apple_leapshim_of_match[] = {
	{ .compatible = "apple,t8103-leap-shim", },
	{}
};
MODULE_DEVICE_TABLE(of, apple_leapshim_of_match);

static struct platform_driver apple_leapshim_driver = {
	.driver = {
		.name = "apple-leapshim",
		.of_match_table = apple_leapshim_of_match,
	},
	.probe = leapshim_probe,
	.remove = leapshim_remove,
};
module_platform_driver(apple_leapshim_driver);

MODULE_AUTHOR("Martin Povišer <povik+lin@cutebit.org>");
MODULE_DESCRIPTION("Apple LEAP shim driver");
MODULE_LICENSE("Dual MIT/GPL");
