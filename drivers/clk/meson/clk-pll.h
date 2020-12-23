/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 BayLibre, SAS.
 * Author: Jerome Brunet <jbrunet@baylibre.com>
 */

#ifndef __MESON_CLK_PLL_H
#define __MESON_CLK_PLL_H

#include <linux/clk-provider.h>
#include <linux/regmap.h>
#include "parm.h"

struct pll_params_table {
	unsigned int	m;
	unsigned int	n;
};

struct pll_mult_range {
	unsigned int	min;
	unsigned int	max;
};

#define PLL_PARAMS(_m, _n)						\
	{								\
		.m		= (_m),					\
		.n		= (_n),					\
	}

#define CLK_MESON_PLL_ROUND_CLOSEST	BIT(0)

struct meson_clk_pll_data {
	struct parm en;
	struct parm m;
	struct parm n;
	struct parm frac;
	struct parm l;
	struct parm rst;
	struct parm div_mode;
	struct parm filter_acq1;
	struct parm filter_acq2;
	struct parm iir_bypass_en;
	struct parm lm_w;
	struct parm reve;
	struct parm sdm_pr_en;
	struct parm tdc_cal_en;
	struct parm tdc_cal_ig;
	struct parm tdc_cal_off;
	struct parm tdc_en;
	struct parm tdc_off_c;
	const struct reg_sequence *init_regs;
	unsigned int init_count;
	const struct pll_params_table *table;
	const struct pll_mult_range *range;
	u8 flags;
};

extern const struct clk_ops meson_clk_pll_ro_ops;
extern const struct clk_ops meson_clk_pll_ops;
extern const struct clk_ops meson_clk_pcie_pll_ops;
extern const struct clk_ops meson_clk_hdmi_pll_m8_ops;

#endif /* __MESON_CLK_PLL_H */
