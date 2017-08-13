/*
 * Copyright (c) 2015 Endless Mobile, Inc.
 * Author: Carlo Caione <carlo@endlessm.com>
 * Copyright (C) 2017 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * CPU clock path:
 *
 *                                      MUX1
 *              +-------------[/DIV*2]--|3|
 *              |    +--[/3]------------|2| MUX2
 * [sys_pll]----+----+--[/2]------------|1|-|1|
 *                   +------------------|0| | |----- [cpu_clk]
 *                                          | |
 * [xtal]---+-------------------------------|0|
 *
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>

#include "clkc.h"

#define to_meson_clk_cpu_hw(_hw) container_of(_hw, struct meson_clk_cpu, hw)

static int meson_clk_cpu_determine_rate(struct clk_hw *hw,
					struct clk_rate_request *req)
{
	struct meson_clk_cpu *clk_cpu = to_meson_clk_cpu_hw(hw);

	return __clk_mux_determine_rate(clk_cpu->sys_pll_scale_out_sel, req);
}

static int meson_clk_cpu_set_rate(struct clk_hw *hw, unsigned long rate,
				  unsigned long parent_rate)
{
	struct meson_clk_cpu *clk_cpu = to_meson_clk_cpu_hw(hw);
	struct clk_rate_request req = {};
	int ret;

	req.rate = rate;

	ret = meson_clk_cpu_determine_rate(hw, &req);
	if (ret)
		return ret;

	/* switch to XTAL before modifying any of the CPU related clocks */
	clk_hw_reparent(clk_cpu->cpu_clk_sel, clk_cpu->xtal);
#if 0
	/*
	 * set the divider first to ensure that whenever we switch to it that
	 * the divider is already programmed. if we leave the divider untouched
	 * the system may lock up when trying to switch to the divider.
	 */
	if (req.best_parent_hw == clk_cpu->sys_pll_scale_div) {
		struct clk_hw *divider_parent =
			clk_hw_get_parent(clk_cpu->sys_pll_scale_div);
		unsigned long divider_parent_rate = clk_hw_get_rate(divider_parent);

		ret = clk_divider_ops.set_rate(clk_cpu->sys_pll_scale_div,
					       req.rate, divider_parent_rate);
		if (ret)  {
			printk("%s: failed to set divider: %d\n", __func__,
			       ret);
			return ret;
		}
	}
#endif

	/* select the new best parent clock */
	clk_hw_reparent(clk_cpu->sys_pll_scale_out_sel, req.best_parent_hw);

	/* switch back to the output of the clocks which we just modified */
	clk_hw_reparent(clk_cpu->cpu_clk_sel, clk_cpu->sys_pll_scale_out_sel);
printk("%s: tried to set new rate to %lu MHz (%s) -> result = %lu MHz\n", __func__, req.rate / 1000 / 1000, req.best_parent_hw->init->name, clk_hw_get_rate(clk_cpu->sys_pll_scale_out_sel) / 1000 / 1000); // FIXME
	return 0;
}

static unsigned long meson_clk_cpu_recalc_rate(struct clk_hw *hw,
					       unsigned long parent_rate)
{
	struct meson_clk_cpu *clk_cpu = to_meson_clk_cpu_hw(hw);

	return clk_hw_get_rate(clk_cpu->sys_pll_scale_out_sel);
}

const struct clk_ops meson_clk_cpu_ops = {
	.recalc_rate	= meson_clk_cpu_recalc_rate,
	.determine_rate	= meson_clk_cpu_determine_rate,
	.set_rate	= meson_clk_cpu_set_rate,
};
