// SPDX-License-Identifier: GPL-2.0
//
// mt6768-mt6358.c  --  MT6768 with the MT6358 PMIC codec, ALSA SoC machine driver
//
// Minimal card: the SoC's audio front end connected to the PMIC codec over
// the internal ADDA (MTKAIF) link. External amplifiers are taken from the
// device tree ("aux-devs", "widgets", "audio-routing"); jack detection is
// not handled yet.

#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include "../common/mtk-afe-platform-driver.h"
#include "mt6768-afe-common.h"

SND_SOC_DAILINK_DEFS(playback1,
	DAILINK_COMP_ARRAY(COMP_CPU("DL1")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(playback2,
	DAILINK_COMP_ARRAY(COMP_CPU("DL2")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(capture1,
	DAILINK_COMP_ARRAY(COMP_CPU("UL1")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(capture2,
	DAILINK_COMP_ARRAY(COMP_CPU("UL2")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(primary_codec,
	DAILINK_COMP_ARRAY(COMP_CPU("ADDA")),
	DAILINK_COMP_ARRAY(COMP_CODEC("mt6358-sound", "mt6358-snd-codec-aif1")),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

static struct snd_soc_dai_link mt6768_mt6358_dai_links[] = {
	/* Front ends */
	{
		.name = "Playback_1",
		.stream_name = "Playback_1",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE, SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.playback_only = 1,
		SND_SOC_DAILINK_REG(playback1),
	},
	{
		.name = "Playback_2",
		.stream_name = "Playback_2",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE, SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.playback_only = 1,
		SND_SOC_DAILINK_REG(playback2),
	},
	{
		.name = "Capture_1",
		.stream_name = "Capture_1",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE, SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.capture_only = 1,
		SND_SOC_DAILINK_REG(capture1),
	},
	{
		.name = "Capture_2",
		.stream_name = "Capture_2",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE, SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.capture_only = 1,
		SND_SOC_DAILINK_REG(capture2),
	},
	/* Back end */
	{
		.name = "Primary Codec",
		.no_pcm = 1,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(primary_codec),
	},
};

static struct snd_soc_card mt6768_mt6358_card = {
	.name = "mt6768-mt6358",
	.owner = THIS_MODULE,
	.dai_link = mt6768_mt6358_dai_links,
	.num_links = ARRAY_SIZE(mt6768_mt6358_dai_links),
};

static int mt6768_mt6358_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &mt6768_mt6358_card;
	struct device_node *platform_node;
	struct snd_soc_dai_link *dai_link;
	int ret, i;

	card->dev = &pdev->dev;

	platform_node = of_parse_phandle(pdev->dev.of_node, "mediatek,platform", 0);
	if (!platform_node)
		return dev_err_probe(&pdev->dev, -EINVAL,
				     "Property 'mediatek,platform' missing or invalid\n");

	for_each_card_prelinks(card, i, dai_link) {
		if (!dai_link->platforms->name)
			dai_link->platforms->of_node = platform_node;
	}

	ret = snd_soc_of_parse_card_name(card, "model");
	if (ret)
		goto out;

	/* Board specific parts: external amplifiers and how they are wired */
	if (of_property_present(pdev->dev.of_node, "aux-devs")) {
		ret = snd_soc_of_parse_aux_devs(card, "aux-devs");
		if (ret)
			goto out;
	}
	if (of_property_present(pdev->dev.of_node, "widgets")) {
		ret = snd_soc_of_parse_audio_simple_widgets(card, "widgets");
		if (ret)
			goto out;
	}
	if (of_property_present(pdev->dev.of_node, "audio-routing")) {
		ret = snd_soc_of_parse_audio_routing(card, "audio-routing");
		if (ret)
			goto out;
	}

	ret = devm_snd_soc_register_card(&pdev->dev, card);
	if (ret)
		dev_err_probe(&pdev->dev, ret, "registering the sound card failed\n");
out:
	of_node_put(platform_node);
	return ret;
}

static const struct of_device_id mt6768_mt6358_dt_match[] = {
	{ .compatible = "mediatek,mt6768-mt6358-sound" },
	{ }
};
MODULE_DEVICE_TABLE(of, mt6768_mt6358_dt_match);

static struct platform_driver mt6768_mt6358_driver = {
	.driver = {
		.name = "mt6768-mt6358",
		.of_match_table = mt6768_mt6358_dt_match,
		.pm = &snd_soc_pm_ops,
	},
	.probe = mt6768_mt6358_probe,
};
module_platform_driver(mt6768_mt6358_driver);

MODULE_DESCRIPTION("MT6768-MT6358 ALSA SoC machine driver");
MODULE_LICENSE("GPL");
