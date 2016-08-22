/*
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright (c) 2016 BayLibre, SAS.
 * Author: Neil Armstrong <narmstrong@baylibre.com>
 * Copyright (C) 2014 Amlogic, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 * The full GNU General Public License is included in this distribution
 * in the file called COPYING.
 *
 * BSD LICENSE
 *
 * Copyright (c) 2016 BayLibre, SAS.
 * Author: Neil Armstrong <narmstrong@baylibre.com>
 * Copyright (C) 2014 Amlogic, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in
 *     the documentation and/or other materials provided with the
 *     distribution.
 *   * Neither the name of Intel Corporation nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/clk-provider.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/pwm.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/of.h>

#define REG_PWM_A	0x0
#define REG_PWM_B	0x4
#define PWM_HIGH_SHIFT	16

#define REG_MISC_AB	0x8
#define MISC_B_CLK_EN	BIT(23)
#define MISC_A_CLK_EN	BIT(15)
#define MISC_CLK_DIV_MASK	0x7f
#define MISC_B_CLK_DIV_SHIFT	16
#define MISC_A_CLK_DIV_SHIFT	8
#define MISC_B_CLK_SEL_SHIFT	6
#define MISC_A_CLK_SEL_SHIFT	4
#define MISC_CLK_SEL_WIDTH	2
#define MISC_B_EN	BIT(1)
#define MISC_A_EN	BIT(0)

#define PWM_NUM		2

static const unsigned int mux_reg_shifts[PWM_NUM] = {
	MISC_A_CLK_SEL_SHIFT, MISC_B_CLK_SEL_SHIFT
};

enum pwm_channel {
	PWM_A = 0,
	PWM_B,
};

struct meson_pwm_channel {
	unsigned int pwm_hi;
	unsigned int pwm_lo;
	u8 pwm_pre_div;
	int period;
	int duty;
	struct pwm_state state;
};

struct meson_pwm_chip {
	struct platform_device *pdev;
	struct pwm_chip chip;
	void __iomem *base;
	u8 inverter_mask;
	spinlock_t lock;
	struct clk *clk_parents[PWM_NUM];
	struct clk_mux mux[PWM_NUM];
	struct clk *clk[PWM_NUM];
};

struct meson_pwm_data {
	const char *const *parent_names;
};

#define to_meson_pwm_chip(chip) \
	container_of(chip, struct meson_pwm_chip, chip)

static int meson_pwm_calc(struct meson_pwm_chip *pwm_data,
			  struct meson_pwm_channel *pwm_chan,
			  unsigned int id,
			  unsigned int duty_ns,
			  unsigned int period_ns)
{
	unsigned int pwm_pre_div;
	unsigned int pwm_cnt;
	unsigned int pwm_duty_cnt;
	unsigned long fin_freq = -1;
	unsigned long fin_ns;
	unsigned int i = 0;

	if (duty_ns > period_ns)
		return -EINVAL;

	if ((~(pwm_data->inverter_mask >> id) & 0x1))
		duty_ns = period_ns - duty_ns;

	if (period_ns == pwm_chan->period && duty_ns == pwm_chan->duty)
		return 0;

	switch (id) {
	case PWM_A:
		fin_freq = clk_get_rate(pwm_data->clk[0]);
		break;
	case PWM_B:
		fin_freq = clk_get_rate(pwm_data->clk[1]);
		break;
	}
	if (fin_freq <= 0) {
		dev_err(pwm_data->chip.dev, "invalid source clock frequency\n");
		return -EINVAL;
	}
	dev_dbg(pwm_data->chip.dev, "fin_freq: %luHz\n", fin_freq);
	fin_ns = NSEC_PER_SEC / fin_freq;

	/* Calc pre_div with the period */
	for (i = 0; i < MISC_CLK_DIV_MASK; i++) {
		pwm_pre_div = i;
		pwm_cnt = DIV_ROUND_CLOSEST(period_ns,
				fin_ns * (pwm_pre_div + 1));
		dev_dbg(pwm_data->chip.dev, "fin_ns=%lu pre_div=%d cnt=%d\n",
				fin_ns, pwm_pre_div, pwm_cnt);
		if (pwm_cnt <= 0xffff)
			break;
	}
	if (i == MISC_CLK_DIV_MASK) {
		dev_err(pwm_data->chip.dev, "Unable to get period pre_div");
		return -EINVAL;
	}
	dev_dbg(pwm_data->chip.dev, "period_ns=%d pre_div=%d pwm_cnt=%d\n",
		period_ns, pwm_pre_div, pwm_cnt);

	if (duty_ns == period_ns) {
		pwm_chan->pwm_pre_div = pwm_pre_div;
		pwm_chan->pwm_hi = pwm_cnt;
		pwm_chan->pwm_lo = 0;
	} else if (duty_ns == 0) {
		pwm_chan->pwm_pre_div = pwm_pre_div;
		pwm_chan->pwm_hi = 0;
		pwm_chan->pwm_lo = pwm_cnt;
	} else {
		/* Then check is we can have the duty with the same pre_div */
		pwm_duty_cnt = DIV_ROUND_CLOSEST(duty_ns,
					fin_ns * (pwm_pre_div + 1));
		if (pwm_cnt > 0xffff) {
			dev_err(pwm_data->chip.dev, "Unable to get duty period, differences are too high");
			return -EINVAL;
		}
		dev_dbg(pwm_data->chip.dev, "duty_ns=%d pre_div=%d pwm_cnt=%d\n",
			duty_ns, pwm_pre_div, pwm_duty_cnt);

		pwm_chan->pwm_pre_div = pwm_pre_div;
		pwm_chan->pwm_hi = pwm_duty_cnt;
		pwm_chan->pwm_lo = pwm_cnt - pwm_chan->pwm_hi;
	}

	return 0;
}

static void meson_pwm_get_state(struct pwm_chip *chip,
				struct pwm_device *pwm,
				struct pwm_state *state)
{
	struct meson_pwm_chip *pwm_data = to_meson_pwm_chip(chip);
	unsigned int id = pwm->hwpwm;

	if (!state)
		return;

	switch (id) {
	case PWM_A:
		if ((readl(pwm_data->base + REG_MISC_AB) & MISC_A_EN))
			state->enabled = true;
		break;
	case PWM_B:
		if ((readl(pwm_data->base + REG_MISC_AB) & MISC_B_EN))
			state->enabled = true;
		break;
	}
}

static int meson_pwm_request(struct pwm_chip *chip,
			     struct pwm_device *pwm)
{
	struct meson_pwm_channel *pwm_chan;

	pwm_chan = devm_kzalloc(chip->dev, sizeof(*pwm_chan), GFP_KERNEL);
	if (!pwm_chan)
		return -ENOMEM;

	meson_pwm_get_state(chip, pwm, &pwm_chan->state);

	pwm_set_chip_data(pwm, pwm_chan);

	return 0;
}

static void meson_pwm_free(struct pwm_chip *chip,
			   struct pwm_device *pwm)
{
	devm_kfree(chip->dev, pwm_get_chip_data(pwm));
	pwm_set_chip_data(pwm, NULL);
}

static void meson_pwm_enable(struct meson_pwm_chip *pwm_data,
			     struct meson_pwm_channel *pwm_chan,
			     unsigned int id)
{
	switch (id) {
	case PWM_A:
		writel((readl(pwm_data->base + REG_MISC_AB) &
			~(MISC_CLK_DIV_MASK << MISC_A_CLK_DIV_SHIFT)) |
			((pwm_chan->pwm_pre_div << MISC_A_CLK_DIV_SHIFT) |
			 MISC_A_CLK_EN),
			pwm_data->base + REG_MISC_AB);

		writel((pwm_chan->pwm_hi << PWM_HIGH_SHIFT) |
		       (pwm_chan->pwm_lo),
		       pwm_data->base + REG_PWM_A);

		writel(readl(pwm_data->base + REG_MISC_AB) | MISC_A_EN,
			pwm_data->base + REG_MISC_AB);
		break;

	case PWM_B:
		writel((readl(pwm_data->base + REG_MISC_AB) &
			~(MISC_CLK_DIV_MASK << MISC_B_CLK_DIV_SHIFT)) |
			((pwm_chan->pwm_pre_div << MISC_B_CLK_DIV_SHIFT) |
			 MISC_B_CLK_EN),
			pwm_data->base + REG_MISC_AB);

		writel((pwm_chan->pwm_hi << PWM_HIGH_SHIFT) |
		       (pwm_chan->pwm_lo),
		       pwm_data->base + REG_PWM_B);

		writel(readl(pwm_data->base + REG_MISC_AB) | MISC_B_EN,
			pwm_data->base + REG_MISC_AB);
		break;

	default:
		break;
	}
}

static void meson_pwm_disable(struct meson_pwm_chip *pwm_data, unsigned int id)
{
	switch (id) {
	case PWM_A:
		writel(readl(pwm_data->base + REG_MISC_AB) & ~MISC_A_EN,
			pwm_data->base + REG_MISC_AB);
		break;

	case PWM_B:
		writel(readl(pwm_data->base + REG_MISC_AB) & ~MISC_B_EN,
			pwm_data->base + REG_MISC_AB);
		break;

	default:
		break;
	}
}

static int meson_pwm_apply(struct pwm_chip *chip,
			   struct pwm_device *pwm,
			   struct pwm_state *state)
{
	struct meson_pwm_chip *pwm_data = to_meson_pwm_chip(chip);
	struct meson_pwm_channel *pwm_chan = pwm_get_chip_data(pwm);
	unsigned int id = pwm->hwpwm;
	unsigned long flags;
	int ret = 0;

	if (!state)
		return -EINVAL;

	spin_lock_irqsave(&pwm_data->lock, flags);

	if (!state->enabled) {
		meson_pwm_disable(pwm_data, id);
		pwm_chan->state.enabled = false;

		goto out;
	}

	if (state->period != pwm_chan->state.period ||
	    state->duty_cycle != pwm_chan->state.duty_cycle ||
	    state->polarity != pwm_chan->state.polarity) {
		if (pwm_chan->state.enabled) {
			meson_pwm_disable(pwm_data, id);
			pwm_chan->state.enabled = false;
		}

		if (state->polarity != pwm_chan->state.polarity) {
			if (state->polarity == PWM_POLARITY_NORMAL)
				pwm_data->inverter_mask |= BIT(id);
			else
				pwm_data->inverter_mask &= ~BIT(id);
		}

		ret = meson_pwm_calc(pwm_data, pwm_chan, id,
				     state->duty_cycle,
				     state->period);
		if (ret)
			goto out;

		pwm_chan->state.polarity = state->polarity;
		pwm_chan->state.period = state->period;
		pwm_chan->state.duty_cycle = state->duty_cycle;
	}

	if (state->enabled && !pwm_chan->state.enabled) {
		meson_pwm_enable(pwm_data, pwm_chan, id);
		pwm_chan->state.enabled = true;
	}

out:
	spin_unlock_irqrestore(&pwm_data->lock, flags);

	return ret;
}

static const struct pwm_ops meson_pwm_ops = {
	.request	= meson_pwm_request,
	.free		= meson_pwm_free,
	.apply		= meson_pwm_apply,
	.get_state	= meson_pwm_get_state,
	.owner		= THIS_MODULE,
};

static const char *const pwm_meson8b_parent_names[] = {
	"xtal", "vid_pll", "fclk_div4", "fclk_div3", NULL
};

static const struct meson_pwm_data pwm_meson8b_data = {
	.parent_names = pwm_meson8b_parent_names,
};

static const char *const pwm_gxbb_parent_names[] = {
	"xtal", "hdmi_pll", "fclk_div4", "fclk_div3", NULL
};

static const struct meson_pwm_data pwm_gxbb_data = {
	.parent_names = pwm_gxbb_parent_names,
};

static const struct of_device_id meson_pwm_matches[] = {
	{ .compatible = "amlogic,meson8b-pwm", .data = &pwm_meson8b_data },
	{ .compatible = "amlogic,meson-gxbb-pwm", .data = &pwm_gxbb_data },
	{},
};
MODULE_DEVICE_TABLE(of, meson_pwm_matches);

static int meson_pwm_mux_init(struct meson_pwm_chip *chip,
			      const struct meson_pwm_data *data)
{
	struct device *dev = &chip->pdev->dev;
	struct clk_init_data init;
	char clk_name[255];
	int ret;
	int i;

	for (i = 0 ; i < PWM_NUM ; ++i) {
		sprintf(clk_name, "%s#mux%d",
				of_node_full_name(dev->of_node), i);

		init.name = devm_kstrdup(dev, clk_name, GFP_KERNEL);
		init.ops = &clk_mux_ops;
		init.flags = CLK_IS_BASIC;
		init.parent_names = data->parent_names;
		init.num_parents = 1 << MISC_CLK_SEL_WIDTH;

		chip->mux[i].reg = chip->base + REG_MISC_AB;
		chip->mux[i].shift = mux_reg_shifts[i];
		chip->mux[i].mask = BIT(MISC_CLK_SEL_WIDTH) - 1;
		chip->mux[i].flags = 0;
		chip->mux[i].lock = &chip->lock;
		chip->mux[i].table = NULL;
		chip->mux[i].hw.init = &init;

		chip->clk[i] = devm_clk_register(dev, &chip->mux[i].hw);
		if (IS_ERR(chip->clk[i])) {
			dev_err(dev, "Failed to register %s\n", clk_name);
			return PTR_ERR(chip->clk[i]);
		}

		if (chip->clk_parents[i]) {
			ret = clk_set_parent(chip->clk[i],
					     chip->clk_parents[i]);
			if (ret) {
				dev_err(dev, "Failed to set %s parent\n",
					clk_name);
				return ret;
			}
		}
	}

	return 0;
}

static int meson_pwm_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct device *dev = &pdev->dev;
	struct meson_pwm_chip *chip;
	struct resource *res;
	int ret;

	match = of_match_node(meson_pwm_matches, dev->of_node);
	if (!match)
		return -ENODEV;

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (chip == NULL)
		return -ENOMEM;

	chip->pdev = pdev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	chip->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(chip->base))
		return PTR_ERR(chip->base);

	chip->clk_parents[0] = devm_clk_get(dev, "clkin0");
	if (IS_ERR(chip->clk_parents[0])) {
		if (PTR_ERR(chip->clk_parents[0]) == -EPROBE_DEFER)
			return PTR_ERR(chip->clk_parents[0]);
		chip->clk_parents[0] = NULL;
	}

	chip->clk_parents[1] = devm_clk_get(dev, "clkin1");
	if (IS_ERR(chip->clk_parents[1])) {
		if (PTR_ERR(chip->clk_parents[1]) == -EPROBE_DEFER)
			return PTR_ERR(chip->clk_parents[1]);
		chip->clk_parents[1] = NULL;
	}

	ret = meson_pwm_mux_init(chip, match->data);
	if (ret)
		return ret;

	clk_prepare_enable(chip->clk[0]);
	clk_prepare_enable(chip->clk[1]);

	chip->chip.dev = dev;
	chip->chip.ops = &meson_pwm_ops;
	chip->chip.base = -1;
	chip->chip.npwm = PWM_NUM;
	chip->chip.of_xlate = of_pwm_xlate_with_flags;
	chip->chip.of_pwm_n_cells = 3;
	chip->inverter_mask = BIT(PWM_NUM) - 1;

	ret = pwmchip_add(&chip->chip);
	if (ret < 0) {
		dev_err(dev, "failed to register PWM chip\n");
		return ret;
	}

	platform_set_drvdata(pdev, chip);

	return 0;
}

static int meson_pwm_remove(struct platform_device *pdev)
{
	struct meson_pwm_chip *chip = platform_get_drvdata(pdev);

	return pwmchip_remove(&chip->chip);
}

static struct platform_driver meson_pwm_driver = {
	.driver		= {
		.name	= "meson-pwm",
		.of_match_table = of_match_ptr(meson_pwm_matches),
	},
	.probe		= meson_pwm_probe,
	.remove		= meson_pwm_remove,
};
module_platform_driver(meson_pwm_driver);

MODULE_ALIAS("platform:meson-pwm");
MODULE_DESCRIPTION("Amlogic Meson PWM Generator driver");
MODULE_AUTHOR("Neil Armstrong <narmstrong@baylibre.com>");
MODULE_LICENSE("Dual BSD/GPL");
