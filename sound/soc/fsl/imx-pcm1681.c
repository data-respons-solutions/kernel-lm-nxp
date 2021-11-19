/*
 * Copyright (C) 2015 DATA RESPONS AS.
 *
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 * Copyright (C) 2012 Linaro Ltd.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/power_supply.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <sound/control.h>
#include <sound/pcm_params.h>
#include <sound/soc-dapm.h>
#include <linux/pinctrl/consumer.h>
#include "fsl_esai.h"

#define DAI_NAME_SIZE	32

struct pcm_gpio_info {
	int gpio_nr;
	enum of_gpio_flags flags;
};

struct imx_pcm1681_data {
	struct snd_soc_dai_link dai;
	struct snd_soc_card card;
	struct snd_soc_dai_link_component comp[3];
	char codec_dai_name[DAI_NAME_SIZE];
	char platform_name[DAI_NAME_SIZE];
	struct pcm_gpio_info shutdown_gpios[8];
	int num_shutdown_gpios;
	u32 pll_freq;
	struct clk *input_clk;
	struct power_supply *power[2];
	int nr_power;
};

static const struct snd_soc_dapm_widget imx_pcm1681_dapm_widgets[] = {
	SND_SOC_DAPM_HP("OUT1", NULL),
	SND_SOC_DAPM_HP("OUT2", NULL),
	SND_SOC_DAPM_HP("OUT3", NULL),
	SND_SOC_DAPM_HP("OUT4", NULL),
	SND_SOC_DAPM_HP("OUT5", NULL),
	SND_SOC_DAPM_HP("OUT6", NULL),
	SND_SOC_DAPM_HP("OUT7", NULL),
	SND_SOC_DAPM_HP("OUT8", NULL),
};

static const struct snd_soc_dapm_route audio_map_1681[] = {
	{"OUT1", NULL, "VOUT1"},
	{"OUT2", NULL, "VOUT2"},
	{"OUT3", NULL, "VOUT3"},
	{"OUT4", NULL, "VOUT4"},
	{"OUT5", NULL, "VOUT5"},
	{"OUT6", NULL, "VOUT6"},
	{"OUT7", NULL, "VOUT7"},
	{"OUT8", NULL, "VOUT8"},

};

static int imx_pcm1681_hw_param(struct snd_pcm_substream *substream,
				     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = asoc_rtd_to_codec(rtd, 0);
	struct snd_soc_dai *cpu_dai = asoc_rtd_to_cpu(rtd, 0);
	struct imx_pcm1681_data *data = snd_soc_card_get_drvdata(rtd->card);
	unsigned int sample_rate = params_rate(params);
	unsigned ch = params_channels(params);
	snd_pcm_format_t sample_format = params_format(params);
	int ret = 0;
	int slotw=32;
	u32 width = snd_pcm_format_width(params_format(params));
	unsigned int clock_freq=0;
	u32 codec_dai_format = SND_SOC_DAIFMT_LEFT_J | SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS;
	snd_soc_dai_set_sysclk(cpu_dai, ESAI_HCKT_EXTAL, data->pll_freq, SND_SOC_CLOCK_IN);
	snd_soc_dai_set_sysclk(codec_dai, 0, data->pll_freq, SND_SOC_CLOCK_IN);
	ret = snd_soc_dai_set_fmt(codec_dai, codec_dai_format);
	if (ret) {
		dev_err(rtd->dev, "failed to set codec dai fmt: %d\n", ret);
		return ret;
	}
	ret = snd_soc_dai_set_fmt(cpu_dai, codec_dai_format);
	if (ret) {
		dev_err(rtd->dev, "failed to set cpu dai fmt: %d\n", ret);
		return ret;
	}

	switch (ch) {
	case 2:
	case 4:
	case 6:
		switch (sample_format) {
		case SNDRV_PCM_FORMAT_S16_LE:
			clock_freq = sample_rate * 32;
			slotw = 16;
			break;
		case SNDRV_PCM_FORMAT_S24_LE:
		case SNDRV_PCM_FORMAT_S32_LE:
			clock_freq = sample_rate * 64;
			break;
		default:
			return -EINVAL;
			break;
		}

		break;
	case 8:
		if (width >= 24) {
			clock_freq = sample_rate * 256;
		}
		else {
			dev_err(rtd->dev, "%s: only S24_LE and S32_LE supported for TDM, was %d\n", __func__, sample_format);
			return -EINVAL;
		}
		break;
	default:

		dev_err(rtd->dev, "%s: 2,4,6,8 channels must be used\n", __func__);
		return -EINVAL;
		break;
	}

	dev_dbg(rtd->dev, "%s: ESAI clock is %d\n", __func__, clock_freq);
	ret = snd_soc_dai_set_tdm_slot(cpu_dai, (1 << ch)-1, 0x0, ch, slotw);
	if (ret) {
		dev_err(rtd->dev, "%s: failed to set cpu tdm fmt: %d\n", __func__, ret);
		return ret;
	}

	ret = snd_soc_dai_set_tdm_slot(codec_dai, (1 << ch)-1, 0x0, ch, slotw);
	if (ret) {
		dev_err(rtd->dev, "%s: failed to set codec tdm fmt: %d\n", __func__, ret);
		return ret;
	}


	return 0;
}



static void imx_pcm1681_set_amps(struct imx_pcm1681_data *priv, bool off)
{
	int n;
	dev_dbg(priv->card.dev, "%s off = %d\n", __func__, off);
	for (n=0; n < priv->num_shutdown_gpios; n++) {
		if (priv->shutdown_gpios[n].flags & OF_GPIO_ACTIVE_LOW)
			gpio_set_value(priv->shutdown_gpios[n].gpio_nr, off ? 0 : 1 );
		else
			gpio_set_value(priv->shutdown_gpios[n].gpio_nr, off ? 1 : 0 );
	}
}

static int imx_pcm1681_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct imx_pcm1681_data *data = snd_soc_card_get_drvdata(rtd->card);
	imx_pcm1681_set_amps(data, false);
	return 0;
}

static void imx_pcm1681_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct imx_pcm1681_data *data = snd_soc_card_get_drvdata(rtd->card);
	imx_pcm1681_set_amps(data, true);
}

static struct snd_soc_ops imx_hifi_ops = {
	.hw_params = imx_pcm1681_hw_param,
	.startup = imx_pcm1681_startup,
	.shutdown = imx_pcm1681_shutdown,
};


static int imx_pcm1681_probe(struct platform_device *pdev)
{
	struct device_node *cpu_np=0;
	struct device_node *codec_np=0;
	struct platform_device *cpu_pdev;
	struct i2c_client *codec_dev;
	struct imx_pcm1681_data *data;
	int ret;
	int n, sd_gpios;
	enum of_gpio_flags flags;
	int gpio_nr;
	int nr_supplies;
	const char *supply_name;


	cpu_np = of_parse_phandle(pdev->dev.of_node, "cpu-dai", 0);
	if (!cpu_np) {
		dev_err(&pdev->dev, "cpu dai phandle missing or invalid\n");
		ret = -EINVAL;
		goto fail;
	}

	if (!strstr(cpu_np->name, "esai")) {
		dev_err(&pdev->dev, "Supports only esai, not %s", cpu_np->name);
		ret = -EINVAL;
		goto fail;
	}


	codec_np = of_parse_phandle(pdev->dev.of_node, "audio-codec", 0);
	if (!codec_np) {
		dev_err(&pdev->dev, "phandle missing or invalid\n");
		ret = -EINVAL;
		goto fail;
	}

	cpu_pdev = of_find_device_by_node(cpu_np);
	if (!cpu_pdev) {
		dev_err(&pdev->dev, "failed to find CPU platform device\n");
		ret = -EINVAL;
		goto fail;
	}
	codec_dev = of_find_i2c_device_by_node(codec_np);
	if (!codec_dev || !codec_dev->dev.driver) {
		dev_err(&pdev->dev, "failed to find codec platform device\n");
		ret = -EINVAL;
		goto fail;
	}


	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto fail;
	}
	data->pll_freq = 8192000;
	data->input_clk = devm_clk_get(&codec_dev->dev, NULL);
	if (IS_ERR(data->input_clk)) {
		data->input_clk = 0;
		dev_warn(&pdev->dev, "failed to get input clock from codec DT\n");
	}

	of_property_read_u32(pdev->dev.of_node, "hfclk-freq", &data->pll_freq);
	dev_info(&pdev->dev, "%s: PLL (HFTXC) = %d\n", __func__, data->pll_freq);
	/* clk_set_rate(data->input_clk, data->pll_freq); */

	sd_gpios = of_gpio_named_count(pdev->dev.of_node, "amp-shutdown-gpios");
	for (n=0; n < sd_gpios; n++) {
		ret = of_get_named_gpio(pdev->dev.of_node, "amp-shutdown-gpios", n);
		if (ret < 0 || !gpio_is_valid(ret))
			dev_warn(&pdev->dev, "%s: Bad DT for gpio %d [%d]\n", __func__, n, ret);
		else {
			gpio_nr = ret;
			of_get_named_gpio_flags(pdev->dev.of_node, "amp-shutdown-gpios", n, &flags);

			ret = gpio_request_one(gpio_nr,
					flags == OF_GPIO_ACTIVE_LOW ? GPIOF_OUT_INIT_LOW : GPIOF_OUT_INIT_HIGH,
					"imx-pcm1681");
			if (ret < 0)
				dev_warn(&pdev->dev, "%s: Unable to allocate gpio %d [%d]\n", __func__, n, ret);
			else {
				dev_info(&pdev->dev, "%s: Using gpio %d with flag %d\n", __func__, gpio_nr, flags);
				data->shutdown_gpios[n].gpio_nr = gpio_nr;
				data->shutdown_gpios[n].flags = flags;
				data->num_shutdown_gpios++;
			}
		}
	}
	data->dai.name = "HiFi";
	data->dai.stream_name = "HiFi";
	data->dai.num_cpus	= 1;
	data->dai.num_codecs	= 1;
	data->dai.num_platforms	= 1;

	data->dai.codecs = &data->comp[1];
	data->dai.codecs->dai_name = "pcm1681-hifi";
	data->dai.codecs->of_node = codec_np;

	data->dai.cpus = &data->comp[0];
	data->dai.cpus->of_node = cpu_np;

	data->dai.platforms = &data->comp[2];
	data->dai.platforms->of_node = cpu_np;

	data->dai.ops = &imx_hifi_ops;
	data->dai.dai_fmt = SND_SOC_DAIFMT_LEFT_J | SND_SOC_DAIFMT_NB_IF |
			    SND_SOC_DAIFMT_CBS_CFS;

	data->card.dev = &pdev->dev;
	ret = snd_soc_of_parse_card_name(&data->card, "model");
	if (ret) {
		dev_err(&pdev->dev, "DT must supply model attribute");
		goto fail;
	}
	ret = snd_soc_of_parse_audio_routing(&data->card, "audio-routing");
	if (ret)
		goto fail;
	data->card.num_links = 1;
	data->card.dai_link = &data->dai;
	data->card.dapm_widgets = imx_pcm1681_dapm_widgets;
	data->card.num_dapm_widgets = ARRAY_SIZE(imx_pcm1681_dapm_widgets);

	platform_set_drvdata(pdev, &data->card);
	snd_soc_card_set_drvdata(&data->card, data);

	/* Power supplies */
	nr_supplies = of_property_count_strings(pdev->dev.of_node, "extra-supply");
	if ( nr_supplies > 0) {
		dev_info(&pdev->dev, "Found %d power supplies (MAX is 2)\n", nr_supplies);
		if (nr_supplies > 2)
			nr_supplies = 2;
		data->nr_power = nr_supplies;
		for (nr_supplies=0; nr_supplies < data->nr_power; nr_supplies++) {
			ret = of_property_read_string_index(pdev->dev.of_node, "extra-supply", nr_supplies, &supply_name);
			if (ret == 0) {
				data->power[nr_supplies] = power_supply_get_by_name(supply_name);
				if (data->power[nr_supplies] == NULL)
					dev_err(&pdev->dev, "Unable to get power %s\n", supply_name);
			}

		}

	}
	ret = snd_soc_register_card(&data->card);
	if (ret) {
		if (ret != -EPROBE_DEFER)
			dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n", ret);
		goto fail;
	}

fail:
	if (cpu_np)
		of_node_put(cpu_np);
	if (codec_np)
		of_node_put(codec_np);

	return ret;
}

static int imx_pcm1681_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct imx_pcm1681_data *data = snd_soc_card_get_drvdata(card);
	if (data->power[0])
		power_supply_put(data->power[0]);
	if (data->power[1])
		power_supply_put(data->power[1]);
	snd_soc_unregister_card(card);

	return 0;
}

static const struct of_device_id imx_pcm1681_dt_ids[] = {
	{ .compatible = "fsl,imx-audio-pcm1681", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_pcm1681_dt_ids);

static struct platform_driver imx_pcm1681_driver = {
	.driver = {
		.name = "imx-pcm1681",
		.owner = THIS_MODULE,
		.pm = &snd_soc_pm_ops,
		.of_match_table = imx_pcm1681_dt_ids,
	},
	.probe = imx_pcm1681_probe,
	.remove = imx_pcm1681_remove,
};
module_platform_driver(imx_pcm1681_driver);

MODULE_AUTHOR("DATA RESPONS AS");
MODULE_DESCRIPTION("Freescale i.MX pcm1681 ASoC machine driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:imx-pcm1681");
