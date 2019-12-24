// SPDX-License-Identifier: GPL-2.0+
/*
 * Amlogic Meson SDHC clock controller
 *
 * Copyright (C) 2019 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 */

#include <dt-bindings/clock/meson-mx-sdhc-clkc.h>

#include <linux/clk-provider.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include "clk-regmap.h"
#include "clk-pll.h"

#define MESON_SDHC_CLKC			0x10

static const struct clk_regmap meson_mx_sdhc_src_sel = {
	.data = &(struct clk_regmap_mux_data){
		.offset = MESON_SDHC_CLKC,
		.mask = 0x3,
		.shift = 16,
	},
	.hw.init = &(struct clk_init_data){
		.name = "sdhc_src_sel",
		.ops = &clk_regmap_mux_ops,
		.parent_data = (const struct clk_parent_data[]) {
			{ .fw_name = "clkin0", .index = -1, },
			{ .fw_name = "clkin1", .index = -1, },
			{ .fw_name = "clkin2", .index = -1, },
			{ .fw_name = "clkin3", .index = -1, },
		},
		.num_parents = 4,
	},
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

static const struct clk_regmap meson_mx_sdhc_div = {
	.data = &(struct clk_regmap_div_data){
		.offset = MESON_SDHC_CLKC,
		.shift = 0,
		.width = 12,
		.table = meson_mx_sdhc_div_table,
	},
	.hw.init = &(struct clk_init_data){
		.name = "sdhc_div",
		.ops = &clk_regmap_divider_ops,
		.parent_data = (const struct clk_parent_data[]) {
			{ .name = "sdhc_src_sel", .index = -1, },
		},
		.num_parents = 1,
		.flags = CLK_SET_RATE_PARENT,
	},
};

static const struct clk_regmap meson_mx_sdhc_mod_clk_en = {
	.data = &(struct clk_regmap_gate_data){
		.offset = MESON_SDHC_CLKC,
		.bit_idx = 15,
	},
	.hw.init = &(struct clk_init_data){
		.name = "sdhc_mod_clk_on",
		.ops = &clk_regmap_gate_ops,
		.parent_data = (const struct clk_parent_data[]) {
			{ .name = "sdhc_div", .index = -1, },
		},
		.num_parents = 1,
		.flags = CLK_SET_RATE_GATE,
	},
};

static const struct clk_regmap meson_mx_sdhc_tx_clk_en = {
	.data = &(struct clk_regmap_gate_data){
		.offset = MESON_SDHC_CLKC,
		.bit_idx = 14,
	},
	.hw.init = &(struct clk_init_data){
		.name = "sdhc_tx_clk_on",
		.ops = &clk_regmap_gate_ops,
		.parent_data = (const struct clk_parent_data[]) {
			{ .name = "sdhc_div", .index = -1, },
		},
		.num_parents = 1,
		.flags = CLK_SET_RATE_GATE,
	},
};

static const struct clk_regmap meson_mx_sdhc_rx_clk_en = {
	.data = &(struct clk_regmap_gate_data){
		.offset = MESON_SDHC_CLKC,
		.bit_idx = 13,
	},
	.hw.init = &(struct clk_init_data){
		.name = "sdhc_rx_clk_on",
		.ops = &clk_regmap_gate_ops,
		.parent_data = (const struct clk_parent_data[]) {
			{ .name = "sdhc_div", .index = -1, },
		},
		.num_parents = 1,
		.flags = CLK_SET_RATE_PARENT | CLK_SET_RATE_GATE,
	},
};

static const struct clk_regmap meson_mx_sdhc_sd_clk_en = {
	.data = &(struct clk_regmap_gate_data){
		.offset = MESON_SDHC_CLKC,
		.bit_idx = 12,
	},
	.hw.init = &(struct clk_init_data){
		.name = "sdhc_sd_clk_on",
		.ops = &clk_regmap_gate_ops,
		.parent_data = (const struct clk_parent_data[]) {
			{ .name = "sdhc_div", .index = -1, },
		},
		.num_parents = 1,
		.flags = CLK_SET_RATE_PARENT | CLK_SET_RATE_GATE,
	},
};

static const struct clk_regmap *meson_mx_sdhc_clk_regmaps[] = {
	[SDHC_CLKID_SRC_SEL]	= &meson_mx_sdhc_src_sel,
	[SDHC_CLKID_DIV]	= &meson_mx_sdhc_div,
	[SDHC_CLKID_MOD_CLK]	= &meson_mx_sdhc_mod_clk_en,
	[SDHC_CLKID_SD_CLK]	= &meson_mx_sdhc_sd_clk_en,
	[SDHC_CLKID_TX_CLK]	= &meson_mx_sdhc_tx_clk_en,
	[SDHC_CLKID_RX_CLK]	= &meson_mx_sdhc_rx_clk_en,
};

#define MESON_MX_SDHC_NUM_CLKS ARRAY_SIZE(meson_mx_sdhc_clk_regmaps)

static int meson_mx_sdhc_clkc_probe(struct platform_device *pdev)
{
	struct device *parent = pdev->dev.parent;
	struct clk_hw_onecell_data *onecell_data;
	struct clk_regmap *clk_regmap;
	struct regmap *regmap;
	int i, ret;

	regmap = dev_get_regmap(parent, NULL);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	clk_regmap = devm_kcalloc(parent, sizeof(*clk_regmap),
				  MESON_MX_SDHC_NUM_CLKS, GFP_KERNEL);
	if (!clk_regmap)
		return -ENOMEM;

	onecell_data = devm_kzalloc(parent,
				    struct_size(onecell_data, hws,
						MESON_MX_SDHC_NUM_CLKS),
				    GFP_KERNEL);
	if (!onecell_data)
		return -ENOMEM;

	for (i = 0; i < MESON_MX_SDHC_NUM_CLKS; i++) {
		memcpy(&clk_regmap[i], meson_mx_sdhc_clk_regmaps[i],
		       sizeof(*clk_regmap));

		clk_regmap[i].map = regmap;
		onecell_data->hws[i] = &clk_regmap[i].hw;

		ret = devm_clk_hw_register(parent, onecell_data->hws[i]);
		if (ret) {
			dev_err(parent,
				"Registration of SDHC clock %d failed\n", i);
			return ret;
		}
	}

	onecell_data->num = MESON_MX_SDHC_NUM_CLKS;

	return devm_of_clk_add_hw_provider(parent, of_clk_hw_onecell_get,
					   onecell_data);
}

static const struct platform_device_id meson_mx_sdhc_clkc_ids[] = {
	{ "meson8-sdhc-clkc" },
	{ "meson8b-sdhc-clkc" },
	{ "meson8m2-sdhc-clkc" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(platform, meson_mx_sdhc_clkc_ids);

static struct platform_driver meson_mx_sdhc_clkc_driver = {
	.id_table	= meson_mx_sdhc_clkc_ids,
	.probe		= meson_mx_sdhc_clkc_probe,
	.driver		= {
		.name	= "meson-mx-sdhc-clkc",
	},
};
module_platform_driver(meson_mx_sdhc_clkc_driver);

MODULE_DESCRIPTION("Amlogic Meson8/8b/8m2 SDHC clock controller driver");
MODULE_AUTHOR("Martin Blumenstingl <martin.blumenstingl@googlemail.com>");
MODULE_LICENSE("GPL v2");
