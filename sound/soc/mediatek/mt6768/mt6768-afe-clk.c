// SPDX-License-Identifier: GPL-2.0
//
// mt6768-afe-clk.c  --  MediaTek 6768 afe clock ctrl
//
// Based on mt8183-afe-clk.c, Copyright (c) 2018 MediaTek Inc.
//
// Only the clocks needed for the internal ADDA path (PMIC codec) are handled;
// the audio PLL and the I2S master clocks are not wired up yet.

#include <linux/clk.h>

#include "mt6768-afe-common.h"
#include "mt6768-afe-clk.h"

enum {
	CLK_AFE = 0,
	CLK_INFRA_AXI,
	CLK_INFRA_26M,
	CLK_MUX_AUDIO,
	CLK_MUX_AUDIOINTBUS,
	CLK_TOP_SYSPLL1_D4,
	CLK_CLK26M,
	CLK_NUM
};

static const char *aud_clks[CLK_NUM] = {
	[CLK_AFE] = "aud_afe_clk",
	[CLK_INFRA_AXI] = "aud_infra_axi_clk",
	[CLK_INFRA_26M] = "aud_infra_26m_clk",
	[CLK_MUX_AUDIO] = "top_mux_audio",
	[CLK_MUX_AUDIOINTBUS] = "top_mux_audio_int",
	[CLK_TOP_SYSPLL1_D4] = "top_sys_pll1_d4",
	[CLK_CLK26M] = "top_clk26m_clk",
};

int mt6768_init_clock(struct mtk_base_afe *afe)
{
	struct mt6768_afe_private *afe_priv = afe->platform_priv;
	int i;

	afe_priv->clk = devm_kcalloc(afe->dev, CLK_NUM, sizeof(*afe_priv->clk),
				     GFP_KERNEL);
	if (!afe_priv->clk)
		return -ENOMEM;

	for (i = 0; i < CLK_NUM; i++) {
		afe_priv->clk[i] = devm_clk_get(afe->dev, aud_clks[i]);
		if (IS_ERR(afe_priv->clk[i]))
			return dev_err_probe(afe->dev, PTR_ERR(afe_priv->clk[i]),
					     "no clock %s\n", aud_clks[i]);
	}

	return 0;
}

int mt6768_afe_enable_clock(struct mtk_base_afe *afe)
{
	struct mt6768_afe_private *afe_priv = afe->platform_priv;
	int ret;

	ret = clk_prepare_enable(afe_priv->clk[CLK_INFRA_AXI]);
	if (ret)
		goto err;

	ret = clk_prepare_enable(afe_priv->clk[CLK_INFRA_26M]);
	if (ret)
		goto err_infra_26m;

	ret = clk_prepare_enable(afe_priv->clk[CLK_MUX_AUDIO]);
	if (ret)
		goto err_mux_audio;

	ret = clk_set_parent(afe_priv->clk[CLK_MUX_AUDIO],
			     afe_priv->clk[CLK_CLK26M]);
	if (ret)
		goto err_mux_audio_parent;

	ret = clk_prepare_enable(afe_priv->clk[CLK_MUX_AUDIOINTBUS]);
	if (ret)
		goto err_mux_audio_parent;

	ret = clk_set_parent(afe_priv->clk[CLK_MUX_AUDIOINTBUS],
			     afe_priv->clk[CLK_TOP_SYSPLL1_D4]);
	if (ret)
		goto err_intbus_parent;

	ret = clk_prepare_enable(afe_priv->clk[CLK_AFE]);
	if (ret)
		goto err_afe;

	return 0;

err_afe:
	clk_set_parent(afe_priv->clk[CLK_MUX_AUDIOINTBUS],
		       afe_priv->clk[CLK_CLK26M]);
err_intbus_parent:
	clk_disable_unprepare(afe_priv->clk[CLK_MUX_AUDIOINTBUS]);
err_mux_audio_parent:
	clk_disable_unprepare(afe_priv->clk[CLK_MUX_AUDIO]);
err_mux_audio:
	clk_disable_unprepare(afe_priv->clk[CLK_INFRA_26M]);
err_infra_26m:
	clk_disable_unprepare(afe_priv->clk[CLK_INFRA_AXI]);
err:
	dev_err(afe->dev, "enabling the audio clocks failed: %d\n", ret);
	return ret;
}

int mt6768_afe_disable_clock(struct mtk_base_afe *afe)
{
	struct mt6768_afe_private *afe_priv = afe->platform_priv;

	clk_disable_unprepare(afe_priv->clk[CLK_AFE]);
	clk_set_parent(afe_priv->clk[CLK_MUX_AUDIOINTBUS],
		       afe_priv->clk[CLK_CLK26M]);
	clk_disable_unprepare(afe_priv->clk[CLK_MUX_AUDIOINTBUS]);
	clk_disable_unprepare(afe_priv->clk[CLK_MUX_AUDIO]);
	clk_disable_unprepare(afe_priv->clk[CLK_INFRA_26M]);
	clk_disable_unprepare(afe_priv->clk[CLK_INFRA_AXI]);

	return 0;
}
