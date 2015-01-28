/*
 * sound/soc/imx/imx-ac97-vt1613.c --  SoC audio for i.MX Seco Q7 boards with
 *                                     AC97 vt1613 codec
 *
 * Copyright 2009 Sascha Hauer, Pengutronix <s.hauer@pengutronix.de>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/moduleparam.h>
#include <linux/device.h>
#include <linux/fsl_devices.h>
#include <linux/clk.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <asm/mach-types.h>
//#include <mach/audmux.h>


#include "../codecs/vt1613.h"
#include "imx-audmux.h"
#include "imx-ssi.h"

#define DAI_NAME_SIZE	32

struct imx_vt1613_data {
	struct snd_soc_dai_link dai;
	struct snd_soc_card card;
	char codec_dai_name[DAI_NAME_SIZE];
	char platform_name[DAI_NAME_SIZE];
	unsigned int clk_frequency;
};

struct imx_priv {
        int reset_gpio;
        int sysclk;
        int hw;
	bool amic_mono;
	bool dmic_mono;
	struct snd_soc_codec *codec;
	struct platform_device *pdev;
	struct snd_pcm_substream *first_stream;
	struct snd_pcm_substream *second_stream;
};
static struct imx_priv card_priv;
static struct snd_soc_card imx_vt1613;

static int vt1613_params(struct snd_pcm_substream *substream,
        struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	u32 dai_format;
	int ret;
	unsigned int channels = params_channels(params);

	snd_soc_dai_set_sysclk(codec_dai, VT1613_SYSCLK, 48000000, 0);

        snd_soc_dai_set_sysclk(codec_dai, VT1613_LRCLK, params_rate(params), 0);

	dai_format = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
            SND_SOC_DAIFMT_CBM_CFM;

	/* set codec DAI configuration */
        ret = snd_soc_dai_set_fmt(codec_dai, dai_format);
printk("vt1613_params %d\n", ret);
        if (ret < 0)
                return ret;

// Following settings stop AC97 flow in case of MX6 soc. So, exiting now. [gp000q7]
return 0;

	/* TODO: The SSI driver should figure this out for us */
	switch (channels) {
	case 2:
		snd_soc_dai_set_tdm_slot(cpu_dai, 0xfffffffc, 0xfffffffc, 2, 0);
		break;
	case 1:
		snd_soc_dai_set_tdm_slot(cpu_dai, 0xfffffffe, 0xfffffffe, 1, 0);
		break;
	default:
		return -EINVAL;
	}

	/* set cpu DAI configuration */
	dai_format = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_IF |
            SND_SOC_DAIFMT_CBM_CFM;
        ret = snd_soc_dai_set_fmt(cpu_dai, dai_format);
        if (ret < 0)
                return ret;

	return 0;
}

static struct snd_soc_ops imx_vt1613_hifi_ops = {
	.hw_params = vt1613_params,
};

static int vt1613_jack_func;
static int vt1613_spk_func;
static int vt1613_line_in_func;

static const char *jack_function[] = { "off", "on"};

static const char *spk_function[] = { "off", "on" };

static const char *line_in_function[] = { "off", "on" };

static const struct soc_enum vt1613_enum[] = {
	SOC_ENUM_SINGLE_EXT(2, jack_function),
	SOC_ENUM_SINGLE_EXT(2, spk_function),
	SOC_ENUM_SINGLE_EXT(2, line_in_function),
};

static int vt1613_get_jack(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.enumerated.item[0] = vt1613_jack_func;
	return 0;
}

static int vt1613_set_jack(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	if (vt1613_jack_func == ucontrol->value.enumerated.item[0])
		return 0;

	vt1613_jack_func = ucontrol->value.enumerated.item[0];
	if (vt1613_jack_func)
		snd_soc_dapm_enable_pin(&codec->dapm, "Headphone Jack");
	else
		snd_soc_dapm_disable_pin(&codec->dapm, "Headphone Jack");

//	if (vt1613_jack_func)
//		snd_soc_dapm_enable_pin(codec, "Headphone Jack");
//	else
//		snd_soc_dapm_disable_pin(codec, "Headphone Jack");
//	snd_soc_dapm_sync(codec);
	snd_soc_dapm_sync(&codec->dapm);
	return 1;
}

static int vt1613_get_spk(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.enumerated.item[0] = vt1613_spk_func;
	return 0;
}

static int vt1613_set_spk(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	if (vt1613_spk_func == ucontrol->value.enumerated.item[0])
		return 0;

	vt1613_spk_func = ucontrol->value.enumerated.item[0];
	if (vt1613_spk_func)
		snd_soc_dapm_enable_pin(&codec->dapm, "Ext Spk");
	else
		snd_soc_dapm_disable_pin(&codec->dapm, "Ext Spk");

//	if (vt1613_spk_func)
//		snd_soc_dapm_enable_pin(codec, "Ext Spk");
//	else
//		snd_soc_dapm_disable_pin(codec, "Ext Spk");
//	snd_soc_dapm_sync(codec);
	snd_soc_dapm_sync(&codec->dapm);
	return 1;
}

static int vt1613_get_line_in(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.enumerated.item[0] = vt1613_line_in_func;
	return 0;
}

static int vt1613_set_line_in(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	if (vt1613_line_in_func == ucontrol->value.enumerated.item[0])
		return 0;

	vt1613_line_in_func = ucontrol->value.enumerated.item[0];
	if (vt1613_line_in_func)
		snd_soc_dapm_enable_pin(&codec->dapm, "Line In Jack");
	else
		snd_soc_dapm_disable_pin(&codec->dapm, "Line In Jack");

//	if (vt1613_line_in_func)
//		snd_soc_dapm_enable_pin(codec, "Line In Jack");
//	else
//		snd_soc_dapm_disable_pin(codec, "Line In Jack");
//	snd_soc_dapm_sync(codec);
	snd_soc_dapm_sync(&codec->dapm);
	return 1;
}

/* card dapm widgets */
static const struct snd_soc_dapm_widget imx_vt1613_dapm_widgets[] = {
	SND_SOC_DAPM_MIC("Mic Jack", NULL),
	SND_SOC_DAPM_LINE("Line In Jack", NULL),
	SND_SOC_DAPM_SPK("Ext Spk", NULL),
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
};

static const struct snd_kcontrol_new vt1613_machine_controls[] = {
	SOC_ENUM_EXT("Jack Function", vt1613_enum[0], vt1613_get_jack,
		     vt1613_set_jack),
	SOC_ENUM_EXT("Speaker Function", vt1613_enum[1], vt1613_get_spk,
		     vt1613_set_spk),
	SOC_ENUM_EXT("Line In Function", vt1613_enum[1], vt1613_get_line_in,
		     vt1613_set_line_in),
};

/* imx_3stack machine connections to the codec pins */
static const struct snd_soc_dapm_route audio_map[] = {

	/* Mic Jack --> MIC_IN (with automatic bias) */
	{"MIC_IN", NULL, "Mic Jack"},

	/* Line in Jack --> LINE_IN */
	{"LINE_IN", NULL, "Line In Jack"},

	/* HP_OUT --> Headphone Jack */
	{"Headphone Jack", NULL, "HP_OUT"},

	/* LINE_OUT --> Ext Speaker */
	{"Ext Spk", NULL, "LINE_OUT"},
};

static int imx_seco_q7_vt1613_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;

	int ret;

	ret = snd_soc_add_codec_controls(codec, vt1613_machine_controls,
                                ARRAY_SIZE(vt1613_machine_controls));
	if (ret)
		return ret;

	/* Add imx_vt1613 specific widgets */
	snd_soc_dapm_new_controls(dapm, imx_vt1613_dapm_widgets,
				  ARRAY_SIZE(imx_vt1613_dapm_widgets));

	/* Set up imx_vt1613 specific audio path audio_map */
	snd_soc_dapm_add_routes(dapm, audio_map, ARRAY_SIZE(audio_map));

	snd_soc_dapm_disable_pin(dapm, "Line In Jack");
	snd_soc_dapm_enable_pin(dapm, "Headphone Jack");

	snd_soc_dapm_sync(dapm);

	return 0;
}

static struct snd_soc_dai_link imx_vt1613_dai[] = {
	{
		.name		= "HiFi",
		.stream_name	= "HiFi",
		.codec_dai_name	= "vt1613",
		.codec_name	= "vt1613-ac97.0",
		.cpu_dai_name	= "imx-ssi.0",
		.platform_name	= "imx-pcm-audio.0",
		.init		= imx_seco_q7_vt1613_init,
		.ops		= &imx_vt1613_hifi_ops,
	},
};

/* Audio machine driver */
static struct snd_soc_card imx_vt1613 = {
	.name		= "vt1613-audio",
	.dai_link	= imx_vt1613_dai,
	.num_links	= ARRAY_SIZE(imx_vt1613_dai),
};

static struct platform_device *imx_vt1613_snd_device;

static int imx_audmux_ac97_config(struct platform_device *pdev, int slave, int master)
{
	int ret;
        unsigned int ptcr, pdcr;

	/*
	 * The port numbering in the hardware manual starts at 1, while
	 * the audmux API expects it starts at 0.
	 */
        --slave;
        --master;

	ptcr = IMX_AUDMUX_V2_PTCR_SYN |
                        IMX_AUDMUX_V2_PTCR_TCLKDIR |
                        IMX_AUDMUX_V2_PTCR_TCSEL(master);
	pdcr = IMX_AUDMUX_V2_PDCR_RXDSEL(master);

        ret = imx_audmux_v2_configure_port(slave, ptcr, pdcr);
	if (ret) {
		dev_err(&pdev->dev, "audmux internal port setup failed\n");
		return ret;
	}

	ptcr = IMX_AUDMUX_V2_PTCR_SYN |
                        IMX_AUDMUX_V2_PTCR_TFSDIR |
                        IMX_AUDMUX_V2_PTCR_TFSEL(slave);
	pdcr = IMX_AUDMUX_V2_PDCR_RXDSEL(slave);

        ret = imx_audmux_v2_configure_port(master, ptcr, pdcr);
	if (ret) {
		dev_err(&pdev->dev, "audmux external port setup failed\n");
		return ret;
	}

        return 0;
}

/*
static int imx_vt1613_late_probe(struct snd_soc_card *card)
{
	struct snd_soc_dai *codec_dai = card->rtd[0].codec_dai;
	struct imx_priv *priv = &card_priv;
	struct imx_vt1613_data *data = snd_soc_card_get_drvdata(card);
	struct device *dev = &priv->pdev->dev;
	int ret;

	ret = -1;
//	ret = snd_soc_dai_set_sysclk(codec_dai, WM8962_SYSCLK_MCLK,
//			data->clk_frequency, SND_SOC_CLOCK_IN);
	if (ret < 0)
		dev_err(dev, "failed to set sysclk in %s\n", __func__);

	return ret;
}
*/

static int imx_vt1613_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct device_node *ssi_np, *codec_np;
	struct platform_device *ssi_pdev;
	struct imx_priv *priv = &card_priv;
	struct i2c_client *codec_dev;
	struct imx_vt1613_data *data;
	struct clk *codec_clk = NULL;
	int int_port, ext_port;
	int ret;
//        struct mxc_audio_platform_data *plat = pdev->dev.platform_data;
//        card_priv.pdev = pdev;

	priv->pdev = pdev;

	ret = of_property_read_u32(np, "mux-int-port", &int_port);
	if (ret) {
		dev_err(&pdev->dev, "mux-int-port missing or invalid\n");
		return ret;
	}
	ret = of_property_read_u32(np, "mux-ext-port", &ext_port);
	if (ret) {
		dev_err(&pdev->dev, "mux-ext-port missing or invalid\n");
		return ret;
	}

	ret = imx_audmux_ac97_config(pdev, int_port, ext_port);
	if (ret) {
		dev_err(&pdev->dev, "audmux port setup failed\n");
		return ret;
	}

	ssi_np = of_parse_phandle(pdev->dev.of_node, "ssi-controller", 0);
	codec_np = of_parse_phandle(pdev->dev.of_node, "audio-codec", 0);
	if (!ssi_np || !codec_np) {
		dev_err(&pdev->dev, "phandle missing or invalid\n");
		ret = -EINVAL;
		goto fail;
	}

	ssi_pdev = of_find_device_by_node(ssi_np);
	if (!ssi_pdev) {
		dev_err(&pdev->dev, "failed to find SSI platform device\n");
		ret = -EINVAL;
		goto fail;
	}
	codec_dev = of_find_device_by_node(codec_np);
	if (!codec_dev || !codec_dev->driver) {
		dev_err(&pdev->dev, "failed to find codec platform device\n");
		ret = -EINVAL;
		goto fail;
	}

	priv->first_stream = NULL;
	priv->second_stream = NULL;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto fail;
	}

//	codec_clk = devm_clk_get(&codec_dev->dev, NULL);
//return 0;
//	if (IS_ERR(codec_clk)) {
//		ret = PTR_ERR(codec_clk);
//		dev_err(&codec_dev->dev, "failed to get codec clk: %d\n", ret);
////		goto fail;
//	}
//
//return 0;
//	data->clk_frequency = clk_get_rate(codec_clk);

	priv->amic_mono = of_property_read_bool(codec_np, "amic-mono");
	priv->dmic_mono = of_property_read_bool(codec_np, "dmic-mono");

	data->dai.name = "HiFi";
	data->dai.stream_name = "HiFi";
	data->dai.codec_dai_name = "vt1613";
	data->dai.codec_of_node = codec_np;
	data->dai.cpu_dai_name = dev_name(&ssi_pdev->dev);
	data->dai.platform_of_node = ssi_np;
	data->dai.ops = &imx_vt1613_hifi_ops;
//	data->dai.init = &imx_wm8962_gpio_init;
	data->dai.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			    SND_SOC_DAIFMT_CBM_CFM;
	data->card.dev = &pdev->dev;
	ret = snd_soc_of_parse_card_name(&data->card, "model");
	if (ret)
		goto fail;
	ret = snd_soc_of_parse_audio_routing(&data->card, "audio-routing");
	if (ret)
		goto fail;
	data->card.num_links = 1;
	data->card.dai_link = &data->dai;
	data->card.dapm_widgets = imx_vt1613_dapm_widgets;
	data->card.num_dapm_widgets = ARRAY_SIZE(imx_vt1613_dapm_widgets);

//	data->card.late_probe = imx_vt1613_late_probe;

	platform_set_drvdata(pdev, &data->card);
	snd_soc_card_set_drvdata(&data->card, data);

	ret = snd_soc_register_card(&data->card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n", ret);
		goto fail;
	}


//printk("WWWWWWWWWWWWWWW   imx_vt1613_probe Done.\n");
//printk("tttt [0x%08x] \n", readl(0xc0a28010));
return 0;
//fail_mic:
//	driver_remove_file(pdev->dev.driver, &driver_attr_headphone);
fail_hp:
	snd_soc_unregister_card(&data->card);
fail:
	if (ssi_np)
		of_node_put(ssi_np);
	if (codec_np)
		of_node_put(codec_np);

	return ret;
//        imx_audmux_ac97_config(plat->src_port, plat->ext_port);
}


static int imx_vt1613_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	snd_soc_unregister_card(card);
//        struct mxc_audio_platform_data *plat = pdev->dev.platform_data;

//        if (plat->finit)
//                plat->finit();
        return 0;
}

static const struct of_device_id imx_vt1613_dt_ids[] = {
	{ .compatible = "fsl,imx-audio-vt1613", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_vt1613_dt_ids);

static struct platform_driver imx_vt1613_driver = {
	.driver = {
		.name = "imx-vt1613",
		.owner = THIS_MODULE,
		.of_match_table = imx_vt1613_dt_ids,
	},
	.probe = imx_vt1613_probe,
	.remove = imx_vt1613_remove,
};
module_platform_driver(imx_vt1613_driver);

MODULE_AUTHOR("Seco <info@seco.it>");
MODULE_DESCRIPTION("Freescale i.MX VT16163 ASoC machine driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:imx-vt1613");
