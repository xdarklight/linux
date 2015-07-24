/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 * Copyright (C) 2015 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 */

#include <dt-bindings/clock/lantiq-xway-cgu.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/types.h>

#include "clk-xway.h"

#define NUM_GPIO_CLKS		4
#define NUM_CLKOUT_RATES	4

#define CGU_GPHY_CLK_SHIFT	2
#define CGU_GPHY_CLK_MASK	(0x4 << CGU_GPHY_CLK_SHIFT)

static void __iomem *ltq_cgu_membase;

struct ltq_cgu_gpio_clk {
	struct clk_hw	hw;
	unsigned long	*valid_rates;
	u8		module_idx;
	u32		ifccr;
};

struct ltq_cgu_pci_clk {
	struct clk_hw	hw;
	u32		ifccr;
	u32		pcicr;
	unsigned long	rate;
};

/* xway socs can generate clocks on gpio pins */
static unsigned long valid_gpio_clk_rates[NUM_GPIO_CLKS][NUM_CLKOUT_RATES] = {
	{CLOCK_32_768K, CLOCK_1_536M, CLOCK_2_5M, CLOCK_12M},
	{CLOCK_40M, CLOCK_12M, CLOCK_24M, CLOCK_48M},
	{CLOCK_25M, CLOCK_40M, CLOCK_30M, CLOCK_60M},
	{CLOCK_12M, CLOCK_50M, CLOCK_32_768K, CLOCK_25M},
};

static inline struct ltq_cgu_gpio_clk *to_ltq_cgu_gpio_clk(struct clk_hw *hw)
{
	return container_of(hw, struct ltq_cgu_gpio_clk, hw);
}

static inline struct ltq_cgu_pci_clk *to_ltq_cgu_pci_clk(struct clk_hw *hw)
{
	return container_of(hw, struct ltq_cgu_pci_clk, hw);
}

inline u32 ltq_xway_cgu_r32(u32 reg)
{
	return __raw_readl(ltq_cgu_membase + reg);
}

inline void ltq_xway_cgu_w32(u32 val, u32 reg)
{
	__raw_writel(val, ltq_cgu_membase + reg);
}

static void ltq_xway_cgu_gpio_clk_set(struct clk_hw *hw, int enable)
{
	struct ltq_cgu_gpio_clk *gpio_clk = to_ltq_cgu_gpio_clk(hw);
	unsigned int val, enable;

	enable_bits = 7 - gpio_clk->module_idx;
	val = ltq_xway_cgu_r32(gpio_clk->ifccr);

	if (enable)
		val |= enable_bits;
	else
		val &= ~(enable_bits);

	ltq_xway_cgu_w32(val, gpio_clk->ifccr);
}

static int xway_cgu_gpio_clk_enable(struct clk_hw *hw)
{
	ltq_xway_cgu_gpio_clk_set(hw, 1);

	return 0;
}

static void xway_cgu_gpio_clk_disable(struct clk_hw *hw)
{
	ltq_xway_cgu_gpio_clk_set(hw, 1);
}

static long xway_cgu_gpio_clk_round_rate(struct clk_hw *hw,
					 unsigned long rate,
					 unsigned long *parent_rate)
{
	struct ltq_cgu_gpio_clk *gpio_clk = to_ltq_cgu_gpio_clk(hw);
	int i;

	for (i = NUM_CLKOUT_RATES - 1; i >= 0; i++) {
		if (gpio_clk->valid_rates[i] >= rate)
			return gpio_clk->valid_rates[i];
	}

	return gpio_clk->valid_rates[0];
}

static unsigned long xway_cgu_gpio_clk_recalc_rate(struct clk_hw *hw,
						   unsigned long parent_rate)
{
	return 0;
}

static int xway_cgu_gpio_clk_set_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long parent_rate)
{
	struct ltq_cgu_gpio_clk *gpio_clk = to_ltq_cgu_gpio_clk(hw);
	int i;

	for (i = 0; i < NUM_CLKOUT_RATES; i++) {
		if (gpio_clk->valid_rates[i] == rate) {
			int shift = 14 - (2 * gpio_clk->module_idx);
			unsigned int val = ltq_xway_cgu_r32(gpio_clk->ifccr);

			val &= ~(3 << shift);
			val |= i << shift;
			ltq_xway_cgu_w32(val, gpio_clk->ifccr);
			return 0;
		}
	}

	return -EINVAL;
}

const struct clk_ops xway_cgu_gpio_clk_ops = {
	.enable = xway_cgu_gpio_clk_enable,
	.disable = xway_cgu_gpio_clk_disable,
	.round_rate = xway_cgu_gpio_clk_round_rate,
	.recalc_rate = xway_cgu_gpio_clk_recalc_rate,
	.set_rate = xway_cgu_gpio_clk_set_rate,
};

struct clk *__ltq_xway_register_cgu_clkout(u8 module_idx)
{
	struct ltq_cgu_gpio_clk *clkout;
	struct clk *clk;
	struct clk_init_data init;
	char *name;

	clkout = kzalloc(sizeof(*clkout), GFP_KERNEL);
	if (!clkout)
		return ERR_PTR(-ENOMEM);

	name = kzalloc(sizeof("clkout0"), GFP_KERNEL);
	if (!name)
		return ERR_PTR(-ENOMEM);

	sprintf(name, "clkout%d", module_idx);

	init.name = name;
	init.ops = &xway_cgu_gpio_clk_ops;
	init.flags = CLK_IS_ROOT;
	init.parent_names = NULL;
	init.num_parents = 0;

	clkout->valid_rates = valid_gpio_clk_rates[module_idx];
	clkout->module_idx = module_idx;
	clkout->hw.init = &init;

	clk = clk_register(NULL, &clkout->hw);

	if (IS_ERR(clk)) {
		kfree(clkout);
		return clk;
	}

	if (clk_register_clkdev(clk, name, "1f103000.cgu"))
		pr_err("%s: Failed to register lookup for %s\n",
		       __func__, name);

	return clk;
}

void ltq_xway_cgu_init_dt(struct device_node *np)
{
	struct resource res_cgu;

	pr_debug("%s: Initializing XWay CGU clocks\n", __func__);

	if (of_address_to_resource(np, 0, &res_cgu))
		panic("%s: could not determine CGU base address\n", __func__);

	ltq_cgu_membase = ioremap_nocache(res_cgu.start,
					  resource_size(&res_cgu));
	if (!ltq_cgu_membase)
		panic("%s: Failed to remap CGU resource\n", __func__);
}

static int xway_cgu_pci_xr9_clk_enable(struct clk_hw *hw)
{
	struct ltq_cgu_pci_clk *pci_clk = to_ltq_cgu_pci_clk(hw);
	u32 val;

	val = ltq_xway_cgu_r32(pci_clk->ifccr);
	val &= ~0x1f00000;

	if (pci_clk->rate == CLOCK_33M)
		val |= 0xe00000;
	else if (pci_clk->rate == CLOCK_62_5M)
		val |= 0x700000;
	else
		return -EINVAL;

	ltq_xway_cgu_w32(val, pci_clk->ifccr);

	return 0;
}

static int xway_cgu_pci_danube_clk_enable(struct clk_hw *hw)
{
	struct ltq_cgu_pci_clk *pci_clk = to_ltq_cgu_pci_clk(hw);
	u32 val;

	val = ltq_xway_cgu_r32(pci_clk->ifccr);
	val &= ~0xf00000;

	if (pci_clk->rate == CLOCK_33M)
		val |= 0x800000;
	else if (pci_clk->rate == CLOCK_62_5M)
		val |= 0x400000;
	else
		return -EINVAL;

	ltq_xway_cgu_w32(val, pci_clk->ifccr);

	return 0;
}

static long xway_cgu_pci_clk_round_rate(struct clk_hw *hw,
					 unsigned long rate,
					 unsigned long *parent_rate)
{
	if (rate >= CLOCK_62_5M)
		return CLOCK_62_5M;

	return CLOCK_33M;
}

static int xway_cgu_pci_clk_set_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long parent_rate)
{
	struct ltq_cgu_pci_clk *pci_clk = to_ltq_cgu_pci_clk(hw);

	pci_clk->rate = rate;

	return 0;
}

static unsigned long xway_cgu_pci_clk_recalc_rate(struct clk_hw *hw,
						   unsigned long parent_rate)
{
	struct ltq_cgu_pci_clk *pci_clk = to_ltq_cgu_pci_clk(hw);

	return pci_clk->rate;
}

const struct clk_ops xway_cgu_pci_xr9_clk_ops = {
	.enable = xway_cgu_pci_xr9_clk_enable,
	.round_rate = xway_cgu_pci_clk_round_rate,
	.recalc_rate = xway_cgu_pci_clk_recalc_rate,
	.set_rate = xway_cgu_pci_clk_set_rate,
};

const struct clk_ops xway_cgu_pci_danube_clk_ops = {
	.enable = xway_cgu_pci_danube_clk_enable,
	.round_rate = xway_cgu_pci_clk_round_rate,
	.recalc_rate = xway_cgu_pci_clk_recalc_rate,
	.set_rate = xway_cgu_pci_clk_set_rate,
};

struct clk *ltq_xway_register_cgu_pci_clk(u32 ifccr_reg_offset,
					  u32 pcicr_reg_offset,
					  const char *parent_name)
{
	struct ltq_cgu_pci_clk *pci_clk;
	struct clk *clk;
	struct clk_init_data init;

	pci_clk = kzalloc(sizeof(*pci_clk), GFP_KERNEL);
	if (!pci_clk)
		return ERR_PTR(-ENOMEM);

	init.name = "cgu_pci";

	if (of_machine_is_compatible("lantiq,ar9") ||
		of_machine_is_compatible("lantiq,vr9"))
		init.ops = &xway_cgu_pci_xr9_clk_ops;
	else
		init.ops = &xway_cgu_pci_danube_clk_ops;

	init.flags = 0;
	init.parent_names = &parent_name;
	init.num_parents = 1;

	pci_clk->ifccr = ifccr_reg_offset;
	pci_clk->pcicr = pcicr_reg_offset;
	pci_clk->rate = CLOCK_33M;
	pci_clk->hw.init = &init;

	clk = clk_register(NULL, &pci_clk->hw);
	if (IS_ERR(clk)) {
		kfree(pci_clk);
		return clk;
	}

	if (clk_register_clkdev(clk, NULL, "17000000.pci"))
		pr_err("%s: Failed to register lookup\n", __func__);

	return clk;
}

static int xway_cgu_pci_ext_clk_is_enabled(struct clk_hw *hw)
{
	struct ltq_cgu_pci_clk *pci_clk;

	pci_clk = container_of(hw, struct ltq_cgu_pci_clk, hw);

	return (ltq_xway_cgu_r32(pci_clk->ifccr) & (1 << 16)) == 0;
}

static int xway_cgu_pci_ext_clk_enable(struct clk_hw *hw)
{
	struct ltq_cgu_pci_clk *pci_clk;
	u32 ifccr_val;

	pci_clk = container_of(hw, struct ltq_cgu_pci_clk, hw);

	ifccr_val = ltq_xway_cgu_r32(pci_clk->ifccr);
	ltq_xway_cgu_w32(ifccr_val & ~(1 << 16), pci_clk->ifccr);
	ltq_xway_cgu_w32((1 << 30), pci_clk->pcicr);

	return 0;
}

static void xway_cgu_pci_ext_clk_disable(struct clk_hw *hw)
{
	struct ltq_cgu_pci_clk *pci_clk;
	u32 ifccr_val;

	pci_clk = container_of(hw, struct ltq_cgu_pci_clk, hw);

	ifccr_val = ltq_xway_cgu_r32(pci_clk->ifccr);
	ltq_xway_cgu_w32(ifccr_val | (1 << 16), pci_clk->ifccr);
	ltq_xway_cgu_w32((1 << 31) | (1 << 30), pci_clk->pcicr);
}

static void xway_cgu_pci_ext_clk_init(struct clk_hw *hw)
{
	/*
	 * Some chips have the external PCI clock enabled by default, while
	 * almost all (real world) devices use the internal PCI clock.
	 * pci-lantiq also only has a property to enable the external PCI
	 * clock, but none to disable it.
	 */
	xway_cgu_pci_ext_clk_disable(hw);
}

const struct clk_ops xway_cgu_pci_ext_clk_ops = {
	.enable = xway_cgu_pci_ext_clk_enable,
	.disable = xway_cgu_pci_ext_clk_disable,
	.is_enabled = xway_cgu_pci_ext_clk_is_enabled,
	.init = xway_cgu_pci_ext_clk_init,
};

struct clk *ltq_xway_register_cgu_pci_ext_clk(u32 ifccr_reg_offset,
					      u32 pcicr_reg_offset)
{
	struct ltq_cgu_pci_clk *pci_clk;
	struct clk *clk;
	struct clk_init_data init;

	pci_clk = kzalloc(sizeof(*pci_clk), GFP_KERNEL);
	if (!pci_clk)
		return ERR_PTR(-ENOMEM);

	init.name = "cgu_pci_ext";
	init.ops = &xway_cgu_pci_ext_clk_ops;
	init.flags = CLK_IS_ROOT | CLK_SET_RATE_GATE;
	init.parent_names = NULL;
	init.num_parents = 0;

	pci_clk->ifccr = ifccr_reg_offset;
	pci_clk->pcicr = pcicr_reg_offset;
	pci_clk->hw.init = &init;

	clk = clk_register(NULL, &pci_clk->hw);
	if (IS_ERR(clk)) {
		kfree(pci_clk);
		return clk;
	}

	if (clk_register_clkdev(clk, "external", "17000000.pci"))
		pr_err("%s: Failed to register lookup\n", __func__);

	return clk;
}

struct clk *ltq_cgu_register_clk(const char *name, const char *parent_name,
				 const struct clk_ops *ops)
{
	struct ltq_cgu_clk *cgu_clk;
	struct clk *clk;
	struct clk_init_data init;

	cgu_clk = kzalloc(sizeof(*cgu_clk), GFP_KERNEL);
	if (!cgu_clk)
		return ERR_PTR(-ENOMEM);

	init.name = name;
	init.ops = ops;
	init.flags = CLK_IS_ROOT;
	init.parent_names = (parent_name ? &parent_name : NULL);
	init.num_parents = (parent_name ? 1 : 0);

	cgu_clk->reg_base = ltq_cgu_membase;
	cgu_clk->hw.init = &init;

	clk = clk_register(NULL, &cgu_clk->hw);
	if (IS_ERR(clk)) {
		kfree(cgu_clk);
		return clk;
	}

	return clk;
}

void ltq_cgu_clk_of_add_provider(struct device_node *np,
				 struct ltq_cgu_clocks *cgu_clocks)
{
	struct clk_onecell_data *clk_data;

	clk_data = kmalloc(sizeof(struct clk_onecell_data), GFP_KERNEL);
	if (!clk_data)
		return;

	clk_data->clks = kcalloc(NUM_CGU_CLOCKS, sizeof(struct clk *),
				 GFP_KERNEL);
	if (!clk_data->clks) {
		kfree(clk_data);
		return;
	}

	/* Add lookups for the system clocks. */
	ltq_register_cpu_clkdev(cgu_clocks->cpu_clk);
	ltq_register_fpi_clkdev(cgu_clocks->fpi_clk);
	ltq_register_io_clkdev(cgu_clocks->io_clk);
	ltq_register_pp32_clkdev(cgu_clocks->pp32_clk);

	clk_data->clk_num = NUM_CGU_CLOCKS;
	clk_data->clks[CGU_CLK_CPU] = cgu_clocks->cpu_clk;
	clk_data->clks[CGU_CLK_FPI] = cgu_clocks->fpi_clk;
	clk_data->clks[CGU_CLK_IO] = cgu_clocks->io_clk;
	clk_data->clks[CGU_CLK_PP32] = cgu_clocks->pp32_clk;
	clk_data->clks[CGU_CLK_CKOUT0] = __ltq_xway_register_cgu_clkout(0);
	clk_data->clks[CGU_CLK_CKOUT1] = __ltq_xway_register_cgu_clkout(1);
	clk_data->clks[CGU_CLK_CKOUT2] = __ltq_xway_register_cgu_clkout(2);
	clk_data->clks[CGU_CLK_CKOUT3] = __ltq_xway_register_cgu_clkout(3);
	clk_data->clks[CGU_CLK_PCI] = cgu_clocks->pci_clk;
	clk_data->clks[CGU_CLK_PCI_EXT] = cgu_clocks->pci_ext_clk;
	clk_data->clks[CGU_CLK_EPHY] = cgu_clocks->ephy_clk;
	of_clk_add_provider(np, of_clk_src_onecell_get, clk_data);
}

int ltq_cgu_select_gphy_clk_input(struct device_node *np)
{
	u8 gphy_sel, val;

	/* Do nothing when the property is missing. */
	if (of_property_read_u8(np, "lantiq,gphy-clk-select", &gphy_sel))
		return 0;

	val = ltq_xway_cgu_r32(CGU_IFCCR_XRX);
	val &= ~(CGU_GPHY_CLK_MASK);

	switch (gphy_sel) {
	case CGU_GPHY_CLK_36MHZ_XTAL:
		val |= 0x1;
		break;
	case CGU_GPHY_CLK_25MHZ_PLL0:
		val |= 0x2;
		break;
	case CGU_GPHY_CLK_24MHZ_PLL2:
		val |= 0x3;
		break;
	case CGU_GPHY_CLK_25MHZ_GPIO3:
		val |= 0x4;
		break;
	default:
		pr_err("%s: Unknown GPHY clk select %x", __func__, gphy_sel);
		return -EINVAL;
	}

	ltq_xway_cgu_w32(val, CGU_IFCCR_XRX);

	return 0;
}
