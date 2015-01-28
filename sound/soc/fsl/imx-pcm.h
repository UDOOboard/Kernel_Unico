/*
 * Copyright 2009 Sascha Hauer <s.hauer@pengutronix.de>
 *
 * This code is based on code copyrighted by Freescale,
 * Liam Girdwood, Javier Martin and probably others.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef _IMX_PCM_H
#define _IMX_PCM_H

#include <linux/platform_data/dma-imx.h>

#include <linux/mxc_asrc.h>

/*
 * Do not change this as the FIQ handler depends on this size
 */
#define IMX_DEFAULT_DMABUF_SIZE (256 * 1024)
#define IMX_SSI_DMABUF_SIZE	(64 * 1024)
#define IMX_SPDIF_DMABUF_SIZE	(64 * 1024)
#define IMX_ESAI_DMABUF_SIZE	(256 * 1024)
#define IMX_ASRC_DMABUF_SIZE	(256 * 1024)

static inline void
imx_pcm_dma_params_init_data(struct imx_dma_data *dma_data,
	int dma, bool shared)
{
	dma_data->dma_request0 = dma;
	dma_data->priority = DMA_PRIO_HIGH;
	if (shared)
		dma_data->peripheral_type = IMX_DMATYPE_SSI_SP;
	else
		dma_data->peripheral_type = IMX_DMATYPE_SSI;
}

///////////////////////////////////////////////////////////////////////////
struct imx_pcm_runtime_data {
	int period_bytes;
	int periods;
	int dma;
	unsigned long offset;
	unsigned long size;
	void *buf;
	int period_time;
	struct dma_async_tx_descriptor *desc;
	struct dma_chan *dma_chan;
	struct imx_dma_data dma_data;
	int asrc_enable;
	struct asrc_p2p_ops *asrc_pcm_p2p_ops_ko;

#if defined(CONFIG_MXC_ASRC) || defined(CONFIG_IMX_HAVE_PLATFORM_IMX_ASRC)
	enum asrc_pair_index asrc_index;
	struct dma_async_tx_descriptor *asrc_desc;
	struct dma_chan *asrc_dma_chan;
	struct imx_dma_data asrc_dma_data;
	struct dma_async_tx_descriptor *asrc_p2p_desc;
	struct dma_chan *asrc_p2p_dma_chan;
	struct imx_dma_data asrc_p2p_dma_data;
	struct asrc_p2p_params *p2p;
#endif
};
///////////////////////////////////////////////////////////////////////////

struct imx_pcm_fiq_params {
        int irq;
        void __iomem *base;

        /* Pointer to original ssi driver to setup tx rx sizes */
        struct snd_dmaengine_dai_dma_data *dma_params_rx;
        struct snd_dmaengine_dai_dma_data *dma_params_tx;
};

#ifdef CONFIG_SND_SOC_IMX_PCM_DMA
int imx_pcm_dma_init(struct platform_device *pdev);
void imx_pcm_dma_exit(struct platform_device *pdev);
#else
static inline int imx_pcm_dma_init(struct platform_device *pdev,
				   unsigned int flags)
{
	return -ENODEV;
}

static inline void imx_pcm_dma_exit(struct platform_device *pdev)
{
}
#endif

#ifdef CONFIG_SND_SOC_IMX_PCM_FIQ
int imx_pcm_fiq_init(struct platform_device *pdev,
				struct imx_pcm_fiq_params *params);
void imx_pcm_fiq_exit(struct platform_device *pdev);
#else
static inline int imx_pcm_fiq_init(struct platform_device *pdev,
				struct imx_pcm_fiq_params *params)
{
	return -ENODEV;
}

static inline void imx_pcm_fiq_exit(struct platform_device *pdev)
{
}
#endif

#endif /* _IMX_PCM_H */
