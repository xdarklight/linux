// SPDX-License-Identifier: GPL-2.0+
/*
 * Amlogic Meson SDHC clock controller
 *
 * Copyright (C) 2020 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 */

#include <dt-bindings/clock/meson-mx-sdhc-clkc.h>

#include <linux/clk-provider.h>
#include <linux/device.h>
#include <linux/platform_device.h>

#include "meson-mx-sdhc.h"

#define MESON_SDHC_NUM_BUILTIN_CLKS	6

struct meson_mx_sdhc_clkc {
	struct clk_mux			src_sel;
	struct clk_divider		div;
	struct clk_gate			mod_clk_en;
	struct clk_gate			tx_clk_en;
	struct clk_gate			rx_clk_en;
	struct clk_gate			sd_clk_en;
	struct clk_hw_onecell_data	hw_onecell_data;
};

static const struct clk_div_table meson_mx_sdhc_div_table[] = {
	{ .div = 6, .val = 5, },
	{ .div = 8, .val = 7, },
	{ .div = 9, .val = 8, },
	{ .div = 10, .val = 9, },
	{ .div = 12, .val = 11, },
	{ .div = 16, .val = 15, },
	{ .div = 18, .val = 17, },
	{ .div = 34, .val = 33, },
	{ .div = 142, .val = 141, },
	{ .div = 850, .val = 849, },
	{ .div = 2126, .val = 2125, },
	{ .div = 4096, .val = 4095, },
	{ /* sentinel */ }
};

static const struct meson_mx_sdhc_clkc meson_mx_sdhc_clkc_data = {
	.src_sel = {
		.mask = 0x3,
		.shift = 16,
	},
	.div = {
		.shift = 0,
		.width = 12,
		.table = meson_mx_sdhc_div_table,
	},
	.mod_clk_en = {
		.bit_idx = 15,
	},
	.tx_clk_en = {
		.bit_idx = 14,
	},
	.rx_clk_en = {
		.bit_idx = 13,
	},
	.sd_clk_en = {
		.bit_idx = 12,
	},
};

static const struct clk_init_data meson_mx_sdhc_clkc_init_data[] = {
	[SDHC_CLKID_SRC_SEL] = {
		.name = "sdhc_src_sel",
		.ops = &clk_mux_ops,
		.parent_data = (const struct clk_parent_data[]) {
			{ .fw_name = "clkin0" },
			{ .fw_name = "clkin1" },
			{ .fw_name = "clkin2" },
			{ .fw_name = "clkin3" },
		},
		.num_parents = 4,
	},
	[SDHC_CLKID_DIV] = {
		.name = "sdhc_div",
		.ops = &clk_divider_ops,
		.parent_hws = (const struct clk_hw *[]) {
			&meson_mx_sdhc_clkc_data.src_sel.hw,
		},
		.num_parents = 1,
		.flags = CLK_SET_RATE_PARENT,
	},
	[SDHC_CLKID_MOD_CLK] = {
		.name = "sdhc_mod_clk_on",
		.ops = &clk_gate_ops,
		.parent_hws = (const struct clk_hw *[]) {
			&meson_mx_sdhc_clkc_data.div.hw,
		},
		.num_parents = 1,
		.flags = CLK_SET_RATE_PARENT,
	},
	[SDHC_CLKID_SD_CLK] = {
		.name = "sdhc_tx_clk_on",
		.ops = &clk_gate_ops,
		.parent_hws = (const struct clk_hw *[]) {
			&meson_mx_sdhc_clkc_data.div.hw,
		},
		.num_parents = 1,
		.flags = CLK_SET_RATE_PARENT,
	},
	[SDHC_CLKID_TX_CLK] = {
		.name = "sdhc_rx_clk_on",
		.ops = &clk_gate_ops,
		.parent_hws = (const struct clk_hw *[]) {
			&meson_mx_sdhc_clkc_data.div.hw,
		},
		.num_parents = 1,
		.flags = CLK_SET_RATE_PARENT,
	},
	[SDHC_CLKID_RX_CLK] = {
		.name = "sdhc_sd_clk_on",
		.ops = &clk_gate_ops,
		.parent_hws = (const struct clk_hw *[]) {
			&meson_mx_sdhc_clkc_data.div.hw,
		},
		.num_parents = 1,
		.flags = CLK_SET_RATE_PARENT,
	},
};

int meson_mx_sdhc_register_clkc(struct device *dev, void __iomem *base)
{
	const struct clk_hw *div_parents[1], *gate_parents[1];
	struct clk_hw_onecell_data *onecell_data;
	struct meson_mx_sdhc_clkc *clkc_data;
	struct clk_init_data init_data;
	int i, ret;

	clkc_data = devm_kzalloc(dev, struct_size(clkc_data,
						  hw_onecell_data.hws,
						  MESON_SDHC_NUM_BUILTIN_CLKS),
				 GFP_KERNEL);
	if (!clkc_data)
		return -ENOMEM;

	memcpy(clkc_data, &meson_mx_sdhc_clkc_data,
	       sizeof(meson_mx_sdhc_clkc_data));

	div_parents[0] = &clkc_data->src_sel.hw;
	gate_parents[0] = &clkc_data->div.hw;

	clkc_data->src_sel.reg = base + MESON_SDHC_CLKC;
	clkc_data->div.reg = base + MESON_SDHC_CLKC;
	clkc_data->mod_clk_en.reg = base + MESON_SDHC_CLKC;
	clkc_data->sd_clk_en.reg = base + MESON_SDHC_CLKC;
	clkc_data->tx_clk_en.reg = base + MESON_SDHC_CLKC;
	clkc_data->rx_clk_en.reg = base + MESON_SDHC_CLKC;

	onecell_data = &clkc_data->hw_onecell_data;
	onecell_data->hws[SDHC_CLKID_SRC_SEL] = &clkc_data->src_sel.hw;
	onecell_data->hws[SDHC_CLKID_DIV] = &clkc_data->div.hw;
	onecell_data->hws[SDHC_CLKID_MOD_CLK] = &clkc_data->mod_clk_en.hw;
	onecell_data->hws[SDHC_CLKID_SD_CLK] = &clkc_data->sd_clk_en.hw;
	onecell_data->hws[SDHC_CLKID_TX_CLK] = &clkc_data->tx_clk_en.hw;
	onecell_data->hws[SDHC_CLKID_RX_CLK] = &clkc_data->rx_clk_en.hw;

	for (i = 0; i < MESON_SDHC_NUM_BUILTIN_CLKS; i++) {
		init_data = meson_mx_sdhc_clkc_init_data[i];

		if (i == SDHC_CLKID_SRC_SEL)
			/* uses clk_parent_data instead */
			init_data.parent_hws = NULL;
		else if (i == SDHC_CLKID_DIV)
			init_data.parent_hws = div_parents;
		else
			init_data.parent_hws = gate_parents;

		onecell_data->hws[i]->init = &init_data;

		ret = devm_clk_hw_register(dev, onecell_data->hws[i]);
		if (ret) {
			dev_err(dev, "Registration of SDHC clock %d failed\n",
				i);
			return ret;
		}
	}

	onecell_data->num = MESON_SDHC_NUM_BUILTIN_CLKS;

	return devm_of_clk_add_hw_provider(dev, of_clk_hw_onecell_get,
					   onecell_data);
}
