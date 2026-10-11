/* SPDX-License-Identifier: GPL-2.0 */
/*
 * mt6768-afe-clk.h  --  MediaTek 6768 afe clock ctrl definition
 *
 * Based on mt8183-afe-clk.h, Copyright (c) 2018 MediaTek Inc.
 */

#ifndef _MT6768_AFE_CLK_H_
#define _MT6768_AFE_CLK_H_

struct mtk_base_afe;

int mt6768_init_clock(struct mtk_base_afe *afe);
int mt6768_afe_enable_clock(struct mtk_base_afe *afe);
int mt6768_afe_disable_clock(struct mtk_base_afe *afe);

#endif
