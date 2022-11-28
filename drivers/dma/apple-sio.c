// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Driver for SIO coprocessor on t8103 (M1) and other Apple SoCs
 *
 * Copyright (C) The Asahi Linux Contributors
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/completion.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_dma.h>
#include <linux/soc/apple/rtkit.h>

#include "dmaengine.h"

#define NCHANNELS_MAX	0x80

#define REG_CPU_CONTROL	0x44
#define CPU_CONTROL_RUN BIT(4)

#define SIOMSG_DATA	GENMASK(63, 32)
#define SIOMSG_TYPE	GENMASK(23, 16)
#define SIOMSG_PARAM	GENMASK(31, 24)
#define SIOMSG_TAG	GENMASK(13, 8)
#define SIOMSG_EP	GENMASK(7, 0)

#define EP_SIO		0x20

#define MSG_START	0x2
#define MSG_SETUP	0x3
#define MSG_CONFIGURE	0x5
#define MSG_ISSUE	0x6
#define MSG_TERMINATE	0x6
#define MSG_ACK		0x65
#define MSG_NACK	0x66
#define MSG_STARTED	0x67
#define MSG_REPORT	0x68

#define SIO_CALL_TIMEOUT_MS	100
#define SIO_SHMEM_SIZE		0x1000

struct sio_data;
struct sio_tx;

struct sio_chan {
	unsigned int no;
	struct sio_data *host;
	struct dma_chan chan;

	spinlock_t lock;
	struct sio_tx *current_tx;
};

#define USABLE_TAGS	0x10

struct sio_data {
	void __iomem *base;
	struct dma_device dma;
	struct device *dev;
	struct apple_rtkit *rtk;
	void *shmem;

	struct sio_tagdata {
		u32 allocated;
		struct completion completions[USABLE_TAGS];
		bool acked[USABLE_TAGS];
	} tags;

	int nchannels;
	struct sio_chan channels[];
};

struct sio_tx {
	struct dma_async_tx_descriptor tx;

	struct list_head node;
};

struct sio_chan_config {
	u32 datashape;
	u32 timeout;
	u32 fifo;
	u32 threshold;
	u32 limit;
} __attribute__((packed));;

static struct sio_chan *to_sio_chan(struct dma_chan *chan)
{
	return container_of(chan, struct sio_chan, chan);
}

static struct sio_tx *to_sio_tx(struct dma_async_tx_descriptor *tx)
{
	return container_of(tx, struct sio_tx, tx);
}

static enum dma_transfer_direction sio_chan_direction(int channo)
{
	/* Channel directions are fixed based on channel number */
	return (channo & 1) ? DMA_DEV_TO_MEM : DMA_MEM_TO_DEV;
}

static dma_cookie_t sio_tx_submit(struct dma_async_tx_descriptor *tx)
{
	struct sio_tx *siotx = to_sio_tx(tx);
	struct sio_chan *siochan = to_sio_chan(tx->chan);

	/* TODO */

	return 0; // cookie;
}

static int sio_desc_free(struct dma_async_tx_descriptor *tx)
{
	kfree(to_sio_tx(tx));

	return 0;
}

static struct dma_async_tx_descriptor *sio_prep_dma_cyclic(
		struct dma_chan *chan, dma_addr_t buf_addr, size_t buf_len,
		size_t period_len, enum dma_transfer_direction direction,
		unsigned long flags)
{
	struct sio_chan *siochan = container_of(chan, struct sio_chan, chan);
	struct sio_tx *siotx;

	if (direction != sio_chan_direction(siochan->no))
		return NULL;

	siotx = kzalloc(sizeof(*siotx), GFP_NOWAIT);
	if (!siotx)
		return NULL;

	/* TODO */

	dma_async_tx_descriptor_init(&siotx->tx, chan);
	siotx->tx.tx_submit = sio_tx_submit;
	siotx->tx.desc_free = sio_desc_free;

	return &siotx->tx;
}

static enum dma_status sio_tx_status(struct dma_chan *chan, dma_cookie_t cookie,
				     struct dma_tx_state *txstate)
{
	struct sio_chan *siochan = to_sio_chan(chan);
	struct sio_data *sio = siochan->host;
	struct sio_tx *siotx;

	enum dma_status ret;

	ret = dma_cookie_status(chan, cookie, txstate);
	if (ret == DMA_COMPLETE || !txstate)
		return ret;

	/* TODO */

	return ret;
}

static void sio_issue_pending(struct dma_chan *chan)
{
	struct sio_chan *siochan = to_sio_chan(chan);
	struct sio_tx *tx;
	unsigned long flags;

	/* TODO */
}

static int sio_terminate_all(struct dma_chan *chan)
{
	struct sio_chan *siochan = to_sio_chan(chan);
	unsigned long flags;

	/* TODO */
	siochan->current_tx = NULL;

	return 0;
}

static void sio_synchronize(struct dma_chan *chan)
{
	struct sio_chan *siochan = to_sio_chan(chan);

	/* TODO */
}

static int sio_alloc_chan_resources(struct dma_chan *chan)
{
	struct sio_chan *siochan = to_sio_chan(chan);
	struct sio_data *sio = siochan->host;
	int ret;

	dma_cookie_init(&siochan->chan);

	/* TODO */

	return 0;
}

static void sio_free_chan_resources(struct dma_chan *chan)
{
	/* TODO */
}

static struct dma_chan *sio_dma_of_xlate(struct of_phandle_args *dma_spec,
					 struct of_dma *ofdma)
{
	struct sio_data *sio = (struct sio_data *) ofdma->of_dma_data;
	unsigned int index = dma_spec->args[0];

	if (dma_spec->args_count != 1 || index >= sio->nchannels)
		return ERR_PTR(-EINVAL);

	return dma_get_slave_channel(&sio->channels[index].chan);
}


static void sio_rtk_crashed(void *cookie)
{
	struct sio_data *sio = cookie;

	dev_err(sio->dev, "SIO down (crashed)");
}

static void sio_recv_msg(void *cookie, u8 ep, u64 msg)
{
	struct sio_data *sio = cookie;
	u32 data;
	u8 param, type, tag, sioep;

	if (ep != EP_SIO)
		goto unknown;

	data = FIELD_GET(SIOMSG_DATA, msg);
	param = FIELD_GET(SIOMSG_PARAM, msg);
	type = FIELD_GET(SIOMSG_TYPE, msg);
	tag = FIELD_GET(SIOMSG_TAG, msg);
	sioep = FIELD_GET(SIOMSG_EP, msg);

	switch (type) {
	case MSG_STARTED:
		dev_info(sio->dev, "SIO protocol v%u\n", data);
		fallthrough;
	case MSG_ACK:
		if (!WARN_ON(tag >= USABLE_TAGS)) {
			sio->tags.acked[tag] = true;
			complete(&sio->tags.completions[tag]);
		}
		break;

	case MSG_NACK:
		if (!WARN_ON(tag >= USABLE_TAGS)) {
			sio->tags.acked[tag] = false;
			complete(&sio->tags.completions[tag]);
		}
		break;

	default:
		goto unknown;
	}

	return;

unknown:
	dev_warn(sio->dev, "received unknown message: ep %x data %016llx\n",
		 ep, msg);
}

static int sio_allocate_tag(struct sio_data *sio)
{
	struct sio_tagdata *tags = &sio->tags;
	int i;

	/*
	 * TODO: Make SMP-ready
	 */

	for (i = 0; i < USABLE_TAGS; i++)
		if (!(tags->allocated & BIT(i)))
			break;

	if (i < USABLE_TAGS) {
		reinit_completion(&tags->completions[i]);
		tags->allocated |= BIT(i);
		return i;
	} else {
		return -EBUSY;
	}
}

static void sio_free_tag(struct sio_data *sio, int tag)
{
	sio->tags.allocated &= ~BIT(tag);
}

static int sio_send_siomsg(struct sio_data *sio, u64 msg)
{
	int tag, ret;

	tag = sio_allocate_tag(sio);
	if (tag < 0)
		return tag;

	msg &= ~SIOMSG_TAG;
	msg |= FIELD_PREP(SIOMSG_TAG, tag);
	ret = apple_rtkit_send_message(sio->rtk, EP_SIO, msg, NULL, false);
	if (ret < 0)
		return ret;

	return tag;
}

static int sio_call(struct sio_data *sio, u64 msg)
{
	int tag, ret;

	tag = sio_send_siomsg(sio, msg);
	if (tag < 0)
		return tag;

	ret = wait_for_completion_timeout(&sio->tags.completions[tag],
					  msecs_to_jiffies(SIO_CALL_TIMEOUT_MS));
	if (!ret) {
		dev_warn(sio->dev, "call %8x timed out\n", msg);
		sio_free_tag(sio, tag);
		return -ETIME;
	}

	ret = sio->tags.acked[tag];
	sio_free_tag(sio, tag);

	return ret;
}

static const struct apple_rtkit_ops sio_rtkit_ops = {
	.crashed = sio_rtk_crashed,
	.recv_message = sio_recv_msg,
};

static int sio_device_config(struct dma_chan *chan,
			     struct dma_slave_config *config)
{
	struct sio_chan *siochan = to_sio_chan(chan);
	struct sio_data *sio = siochan->host;
	bool is_tx = sio_chan_direction(siochan->no) == DMA_MEM_TO_DEV;
	struct sio_chan_config *cfg = sio->shmem;
	int ret;

	switch (is_tx ? config->dst_addr_width : config->src_addr_width) {
	case DMA_SLAVE_BUSWIDTH_1_BYTE:
		cfg->datashape = 2;
		break;
	case DMA_SLAVE_BUSWIDTH_2_BYTES:
		cfg->datashape = 1;
		break;
	case DMA_SLAVE_BUSWIDTH_4_BYTES:
		cfg->datashape = 0;
		break;
	default:
		return -EINVAL;
	}

	cfg->fifo = 0x800;
	cfg->limit = 0x800;
	cfg->threshold = 0x800;

	ret = sio_call(sio, FIELD_PREP(SIOMSG_TYPE, MSG_CONFIGURE));

	if (ret == 1)
		ret = 0;
	else if (ret == 0)
		ret = -EINVAL;
	return 0;
}

static int sio_alloc_shmem(struct sio_data *sio)
{
	dma_addr_t iova;
	int err;

	sio->shmem = dma_alloc_coherent(sio->dev, SIO_SHMEM_SIZE, &iova, GFP_KERNEL);
	if (!sio->shmem)
		return -ENOMEM;

	err = sio_call(sio, FIELD_PREP(SIOMSG_TYPE, MSG_SETUP) \
			    | FIELD_PREP(SIOMSG_PARAM, 1) \
			    | FIELD_PREP(SIOMSG_DATA, iova >> 12));
	if (err != 1) {
		if (err == 0)
			err = -EINVAL;
		return err;
	}

	err = sio_call(sio, FIELD_PREP(SIOMSG_TYPE, MSG_SETUP) \
			    | FIELD_PREP(SIOMSG_PARAM, 2) \
			    | FIELD_PREP(SIOMSG_DATA, SIO_SHMEM_SIZE));
	if (err != 1) {
		if (err == 0)
			err = -EINVAL;
		return err;
	}

	return 0;
}

static int sio_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct sio_data *sio;
	struct dma_device *dma;
	int nchannels, nparams;
	int err, ret, i;

	err = of_property_read_u32(np, "dma-channels", &nchannels);
	if (err || nchannels > NCHANNELS_MAX)
		return dev_err_probe(&pdev->dev, -EINVAL,
				     "missing or invalid dma-channels property\n");

	sio = devm_kzalloc(&pdev->dev, struct_size(sio, channels, nchannels), GFP_KERNEL);
	if (!sio)
		return -ENOMEM;

	platform_set_drvdata(pdev, sio);
	sio->dev = &pdev->dev;
	sio->nchannels = nchannels;

	sio->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(sio->base))
		return PTR_ERR(sio->base);

	sio->rtk = devm_apple_rtkit_init(&pdev->dev, sio, NULL, 0, &sio_rtkit_ops);
	if (IS_ERR(sio->rtk))
		return dev_err_probe(&pdev->dev, PTR_ERR(sio->rtk),
				     "couldn't initialize rtkit\n");
	for (i = 0; i < USABLE_TAGS; i++)
		init_completion(&sio->tags.completions[i]);

	dma = &sio->dma;
	dma_cap_set(DMA_PRIVATE, dma->cap_mask);
	dma_cap_set(DMA_CYCLIC, dma->cap_mask);

	dma->dev = &pdev->dev;
	dma->device_alloc_chan_resources = sio_alloc_chan_resources;
	dma->device_free_chan_resources = sio_free_chan_resources;
	dma->device_tx_status = sio_tx_status;
	dma->device_issue_pending = sio_issue_pending;
	dma->device_terminate_all = sio_terminate_all;
	dma->device_synchronize = sio_synchronize;
	dma->device_prep_dma_cyclic = sio_prep_dma_cyclic;
	dma->device_config = sio_device_config;

	dma->directions = BIT(DMA_MEM_TO_DEV) | BIT(DMA_DEV_TO_MEM);
	dma->residue_granularity = DMA_RESIDUE_GRANULARITY_BURST;
	dma->src_addr_widths = BIT(DMA_SLAVE_BUSWIDTH_1_BYTE) |
			BIT(DMA_SLAVE_BUSWIDTH_2_BYTES) |
			BIT(DMA_SLAVE_BUSWIDTH_4_BYTES);
	dma->dst_addr_widths = BIT(DMA_SLAVE_BUSWIDTH_1_BYTE) |
			BIT(DMA_SLAVE_BUSWIDTH_2_BYTES) |
			BIT(DMA_SLAVE_BUSWIDTH_4_BYTES);

	INIT_LIST_HEAD(&dma->channels);
	for (i = 0; i < nchannels; i++) {
		struct sio_chan *siochan = &sio->channels[i];

		siochan->host = sio;
		siochan->no = i;
		siochan->chan.device = &sio->dma;
		spin_lock_init(&siochan->lock);
		list_add_tail(&siochan->chan.device_node, &dma->channels);
	}


	writel(CPU_CONTROL_RUN, sio->base + REG_CPU_CONTROL);

	err = apple_rtkit_boot(sio->rtk);
	if (err)
		return dev_err_probe(&pdev->dev, err, "SIO did not boot\n");

	err = apple_rtkit_start_ep(sio->rtk, EP_SIO);
	if (err)
		return dev_err_probe(&pdev->dev, err, "starting SIO endpoint\n");

	err = sio_call(sio, FIELD_PREP(SIOMSG_TYPE, MSG_START));
	if (err < 1) {
		if (err == 0)
			err = -ENXIO;
		return dev_err_probe(&pdev->dev, err, "starting SIO service\n");
	}

	err = sio_alloc_shmem(sio);
	if (err < 0)
		return 

	nparams = of_property_count_u32_elems(np, "apple,sio-fw-params");
	if (nparams < 0)
		return dev_err_probe(&pdev->dev, nparams, "bad apple,sio-fw-params\n");

	for (i = 0; i < nparams / 2; i++) {
		u32 key, val;

		err = of_property_read_u32_index(np, "apple,sio-fw-params", 2 * i, &key);
		if (err)
			return dev_err_probe(&pdev->dev, err, "bad apple,sio-fw-params\n");

		err = of_property_read_u32_index(np, "apple,sio-fw-params", 2 * i + 1, &val);
		if (err)
			return dev_err_probe(&pdev->dev, err, "bad apple,sio-fw-params\n");

		err = sio_call(sio, FIELD_PREP(SIOMSG_TYPE, MSG_SETUP) \
				    | FIELD_PREP(SIOMSG_PARAM, key & 0xff) \
				    | FIELD_PREP(SIOMSG_EP, key >> 8) \
				    | FIELD_PREP(SIOMSG_DATA, val));
		if (err < 1) {
			if (err == 0)
				err = -ENXIO;
			return dev_err_probe(&pdev->dev, err, "setting SIO parameter %x to %x\n",
					     key, val);
		}
	}

	err = dma_async_device_register(&sio->dma);
	if (err)
		return dev_err_probe(&pdev->dev, err, "failed to register DMA device\n");

	err = of_dma_controller_register(pdev->dev.of_node, sio_dma_of_xlate, sio);
	if (err) {
		dma_async_device_unregister(&sio->dma);
		return dev_err_probe(&pdev->dev, err, "failed to register with OF\n");
	}


	return 0;
}

static int sio_remove(struct platform_device *pdev)
{
	struct sio_data *sio = platform_get_drvdata(pdev);

	of_dma_controller_free(pdev->dev.of_node);
	dma_async_device_unregister(&sio->dma);

	return 0;
}

static const struct of_device_id sio_of_match[] = {
	{ .compatible = "apple,sio", },
	{ }
};
MODULE_DEVICE_TABLE(of, sio_of_match);

static struct platform_driver apple_sio_driver = {
	.driver = {
		.name = "apple-sio",
		.of_match_table = sio_of_match,
	},
	.probe = sio_probe,
	.remove = sio_remove,
};
module_platform_driver(apple_sio_driver);

MODULE_AUTHOR("Martin Povišer <povik+lin@cutebit.org>");
MODULE_DESCRIPTION("Driver for SIO coprocessor on Apple SoCs");
MODULE_LICENSE("Dual MIT/GPL");
