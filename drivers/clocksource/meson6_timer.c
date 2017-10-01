/*
 * Amlogic Meson6 SoCs timer handling.
 *
 * Copyright (C) 2014 Carlo Caione <carlo@caione.org>
 *
 * Based on code from Amlogic, Inc
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/clockchips.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqreturn.h>
#include <linux/sched_clock.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>

#include "timer-of.h"

struct meson6_timer_clkevt {
	void __iomem		*mux_base;
	struct timer_of		of_timer;
	u32			enable_mask;
	u32			mode_mask;
	u32			input_clock_mask;
};

enum meson6_timera_input_clock {
	MESON_TIMERA_CLOCK_1US = 0x0,
	MESON_TIMERA_CLOCK_10US = 0x1,
	MESON_TIMERA_CLOCK_100US = 0x2,
	MESON_TIMERA_CLOCK_1MS = 0x3,
};

enum meson6_timere_input_clock {
	MESON_TIMERE_CLOCK_SYSTEM_CLOCK = 0x0,
	MESON_TIMERE_CLOCK_1US = 0x1,
	MESON_TIMERE_CLOCK_10US = 0x2,
	MESON_TIMERE_CLOCK_100US = 0x3,
	MESON_TIMERE_CLOCK_1MS = 0x4,
};

#define MESON_ISA_TIMER_MUX					0x00
#define MESON_ISA_TIMER_MUX_TIMERD_EN				BIT(19)
#define MESON_ISA_TIMER_MUX_TIMERC_EN				BIT(18)
#define MESON_ISA_TIMER_MUX_TIMERB_EN				BIT(17)
#define MESON_ISA_TIMER_MUX_TIMERA_EN				BIT(16)
#define MESON_ISA_TIMER_MUX_TIMERD_MODE				BIT(15)
#define MESON_ISA_TIMER_MUX_TIMERC_MODE				BIT(14)
#define MESON_ISA_TIMER_MUX_TIMERB_MODE				BIT(13)
#define MESON_ISA_TIMER_MUX_TIMERA_MODE				BIT(12)
#define MESON_ISA_TIMER_MUX_TIMERE_INPUT_CLOCK_MASK		GENMASK(10, 8)
#define MESON_ISA_TIMER_MUX_TIMERD_INPUT_CLOCK_MASK		GENMASK(7, 6)
#define MESON_ISA_TIMER_MUX_TIMERC_INPUT_CLOCK_MASK		GENMASK(5, 4)
#define MESON_ISA_TIMER_MUX_TIMERB_INPUT_CLOCK_MASK		GENMASK(3, 2)
#define MESON_ISA_TIMER_MUX_TIMERA_INPUT_CLOCK_MASK		GENMASK(1, 0)

#define MESON_ISA_TIMERA					0x04
#define MESON_ISA_TIMERB					0x08
#define MESON_ISA_TIMERC					0x0c
#define MESON_ISA_TIMERD					0x10
#define MESON_ISA_TIMERE					0x14

static void __iomem *timer_base;

static struct meson6_timer_clkevt *to_meson6_timer_clkevt(
	struct clock_event_device *evt)
{
	struct timer_of *of_timer = to_timer_of(evt);

	return container_of(of_timer, struct meson6_timer_clkevt, of_timer);
}

static u64 notrace meson6_timer_sched_read(void)
{
	return (u64)readl(timer_base + MESON_ISA_TIMERE);
}

static void meson6_clkevt_time_stop(struct meson6_timer_clkevt *meson_timer)
{
	u32 val = readl(meson_timer->mux_base);

	writel(val & ~meson_timer->enable_mask, meson_timer->mux_base);
}

static void meson6_clkevt_time_setup(struct meson6_timer_clkevt *meson_timer,
				     unsigned long delay)
{
	writel(delay, timer_of_base(&meson_timer->of_timer));
}

static void meson6_clkevt_time_start(struct meson6_timer_clkevt *meson_timer,
				     bool periodic)
{
	u32 val = readl(meson_timer->mux_base);

	if (periodic)
		val |= meson_timer->mode_mask;
	else
		val &= ~meson_timer->mode_mask;

	writel(val | meson_timer->enable_mask, meson_timer->mux_base);
}

static int meson6_shutdown(struct clock_event_device *evt)
{
	struct meson6_timer_clkevt *meson_timer = to_meson6_timer_clkevt(evt);

	meson6_clkevt_time_stop(meson_timer);
	return 0;
}

static int meson6_set_oneshot(struct clock_event_device *evt)
{
	struct meson6_timer_clkevt *meson_timer = to_meson6_timer_clkevt(evt);

	meson6_clkevt_time_stop(meson_timer);
	meson6_clkevt_time_start(meson_timer, false);
	return 0;
}

static int meson6_set_periodic(struct clock_event_device *evt)
{
	struct meson6_timer_clkevt *meson_timer = to_meson6_timer_clkevt(evt);

	meson6_clkevt_time_stop(meson_timer);
	meson6_clkevt_time_setup(meson_timer, USEC_PER_SEC / HZ - 1);
	meson6_clkevt_time_start(meson_timer, true);
	return 0;
}

static int meson6_clkevt_next_event(unsigned long delta,
				    struct clock_event_device *evt)
{
	struct meson6_timer_clkevt *meson_timer = to_meson6_timer_clkevt(evt);

	meson6_clkevt_time_stop(meson_timer);
	meson6_clkevt_time_setup(meson_timer, delta);
	meson6_clkevt_time_start(meson_timer, false);

	return 0;
}

static irqreturn_t meson6_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evt = (struct clock_event_device *)dev_id;

	evt->event_handler(evt);

	return IRQ_HANDLED;
}

static int __init meson6_timer_init_clockevt(struct device_node *node,
					     u8 timer_reg,
					     u32 enable_mask,
					     u32 mode_mask,
					     u32 input_clock_mask)
{
	struct meson6_timer_clkevt *meson_timer;
	u32 timer_mux;
	int ret;

	meson_timer = kzalloc(sizeof(*meson_timer), GFP_KERNEL);
	if (!meson_timer)
		return -ENOMEM;

	meson_timer->mux_base = timer_base + MESON_ISA_TIMER_MUX;
	meson_timer->enable_mask = enable_mask;
	meson_timer->mode_mask = mode_mask;
	meson_timer->input_clock_mask = input_clock_mask;

	meson_timer->of_timer.flags = TIMER_OF_IRQ;

	meson_timer->of_timer.of_base.base = timer_base + timer_reg;

	meson_timer->of_timer.of_irq.handler = meson6_timer_interrupt;
	meson_timer->of_timer.of_irq.flags = IRQF_TIMER | IRQF_IRQPOLL;
	meson_timer->of_timer.of_irq.index = 0;

	meson_timer->of_timer.clkevt.name = "meson6_tick";
	meson_timer->of_timer.clkevt.rating = 400;
	meson_timer->of_timer.clkevt.features = CLOCK_EVT_FEAT_PERIODIC |
						CLOCK_EVT_FEAT_ONESHOT;
	meson_timer->of_timer.clkevt.set_state_shutdown = meson6_shutdown;
	meson_timer->of_timer.clkevt.set_state_periodic = meson6_set_periodic;
	meson_timer->of_timer.clkevt.set_state_oneshot = meson6_set_oneshot;
	meson_timer->of_timer.clkevt.tick_resume = meson6_shutdown;
	meson_timer->of_timer.clkevt.set_next_event = meson6_clkevt_next_event;
	meson_timer->of_timer.clkevt.cpumask = cpu_possible_mask;

	/* stop the timer before initializing / registering it */
	meson6_clkevt_time_stop(meson_timer);

	timer_mux = readl(meson_timer->mux_base);
	timer_mux &= ~meson_timer->input_clock_mask;
	timer_mux |= MESON_TIMERA_CLOCK_1US << (ffs(input_clock_mask) - 1);
	writel(timer_mux, meson_timer->mux_base);

	ret = timer_of_init(node, &meson_timer->of_timer);
	if (ret)
		return ret;

	clockevents_config_and_register(&meson_timer->of_timer.clkevt,
					USEC_PER_SEC, 1, 0xfffe);

	return 0;
}

static int __init meson6_timer_init(struct device_node *node)
{
	u32 val;

	timer_base = of_io_request_and_map(node, 0, "meson6-timer");
	if (IS_ERR(timer_base)) {
		pr_err("Can't map registers\n");
		return -ENXIO;
	}

	/* Set 1us for timer E */
	val = readl(timer_base + MESON_ISA_TIMER_MUX);
	val &= ~MESON_ISA_TIMER_MUX_TIMERE_INPUT_CLOCK_MASK;
	val |= FIELD_PREP(MESON_ISA_TIMER_MUX_TIMERE_INPUT_CLOCK_MASK,
			  MESON_TIMERE_CLOCK_1US);
	writel(val, timer_base + MESON_ISA_TIMER_MUX);

	sched_clock_register(meson6_timer_sched_read, 32, USEC_PER_SEC);
	clocksource_mmio_init(timer_base + MESON_ISA_TIMERE, node->name,
			      1000 * 1000, 300, 32, clocksource_mmio_readl_up);

	return meson6_timer_init_clockevt(node, MESON_ISA_TIMERA,
				MESON_ISA_TIMER_MUX_TIMERA_EN,
				MESON_ISA_TIMER_MUX_TIMERA_MODE,
				MESON_ISA_TIMER_MUX_TIMERA_INPUT_CLOCK_MASK);
}

TIMER_OF_DECLARE(meson6, "amlogic,meson6-timer", meson6_timer_init);
