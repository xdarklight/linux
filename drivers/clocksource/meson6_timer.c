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
#include <linux/cpuhotplug.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqreturn.h>
#include <linux/sched_clock.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>

struct meson6_timer_clkevt {
	struct clock_event_device	clkevt;
	void __iomem			*base;
	u32				enable_mask;
	u32				mode_mask;
	u32				timer_offset;
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

static struct meson6_timer_clkevt meson_timers[] = {
	{
		.enable_mask = MESON_ISA_TIMER_MUX_TIMERA_EN,
		.mode_mask = MESON_ISA_TIMER_MUX_TIMERA_MODE,
		.timer_offset = MESON_ISA_TIMERA,
	},
	{
		.enable_mask = MESON_ISA_TIMER_MUX_TIMERB_EN,
		.mode_mask = MESON_ISA_TIMER_MUX_TIMERB_MODE,
		.timer_offset = MESON_ISA_TIMERB,
	},
	{
		.enable_mask = MESON_ISA_TIMER_MUX_TIMERC_EN,
		.mode_mask = MESON_ISA_TIMER_MUX_TIMERC_MODE,
		.timer_offset = MESON_ISA_TIMERC,
	},
	{
		.enable_mask = MESON_ISA_TIMER_MUX_TIMERD_EN,
		.mode_mask = MESON_ISA_TIMER_MUX_TIMERD_MODE,
		.timer_offset = MESON_ISA_TIMERD,
	},
};

static void __iomem *timer_base;

static struct meson6_timer_clkevt *to_meson6_timer_clkevt(
	struct clock_event_device *evt)
{
	return container_of(evt, struct meson6_timer_clkevt, clkevt);
}

static void meson6_timer_mux_mask_bits(struct meson6_timer_clkevt *meson_timer,
				       u32 mask, u32 set)
{
	u32 val;

	val = readl(meson_timer->base + MESON_ISA_TIMER_MUX);
	val &= ~mask;
	val |= (set & mask);
	writel(val, meson_timer->base + MESON_ISA_TIMER_MUX);
}

static u64 notrace meson6_timer_sched_read(void)
{
	return (u64)readl(timer_base + MESON_ISA_TIMERE);
}

static void meson6_clkevt_time_setup(struct clock_event_device *evt,
				     unsigned long delay)
{
	struct meson6_timer_clkevt *meson_timer = to_meson6_timer_clkevt(evt);

	writel(delay, meson_timer->base + meson_timer->timer_offset);
}

static void meson6_clkevt_time_start(struct clock_event_device *evt,
				     bool periodic)
{
	struct meson6_timer_clkevt *meson_timer = to_meson6_timer_clkevt(evt);

	if (periodic)
		meson6_timer_mux_mask_bits(meson_timer, meson_timer->mode_mask,
					   meson_timer->mode_mask);
	else
		meson6_timer_mux_mask_bits(meson_timer, meson_timer->mode_mask,
					   0);

	meson6_timer_mux_mask_bits(meson_timer, meson_timer->enable_mask,
				   meson_timer->enable_mask);
}

static int meson6_shutdown(struct clock_event_device *evt)
{
	struct meson6_timer_clkevt *meson_timer = to_meson6_timer_clkevt(evt);

	meson6_timer_mux_mask_bits(meson_timer, meson_timer->enable_mask, 0);
	return 0;
}

static int meson6_set_oneshot(struct clock_event_device *evt)
{
	meson6_shutdown(evt);
	meson6_clkevt_time_start(evt, false);
	return 0;
}

static int meson6_set_periodic(struct clock_event_device *evt)
{
	meson6_shutdown(evt);
	meson6_clkevt_time_setup(evt, USEC_PER_SEC / HZ - 1);
	meson6_clkevt_time_start(evt, true);
	return 0;
}

static int meson6_clkevt_next_event(unsigned long delta,
				    struct clock_event_device *evt)
{
	meson6_shutdown(evt);
	meson6_clkevt_time_setup(evt, delta);
	meson6_clkevt_time_start(evt, false);

	return 0;
}

static irqreturn_t meson6_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evt = (struct clock_event_device *)dev_id;
#if 0
	if (printk_ratelimit())
		printk("%s(%d) @ %d\n", __func__, irq, smp_processor_id());
#endif
	evt->event_handler(evt);

	return IRQ_HANDLED;
}

static int __init meson6_timer_init_clockevt(struct device_node *node, int idx)
{
	struct meson6_timer_clkevt *meson_timer = &meson_timers[idx];

	meson_timer->clkevt.rating = 400;
	meson_timer->clkevt.cpumask = cpu_possible_mask;
	meson_timer->clkevt.features = CLOCK_EVT_FEAT_PERIODIC |
				       CLOCK_EVT_FEAT_ONESHOT;
	meson_timer->clkevt.set_state_shutdown = meson6_shutdown;
	meson_timer->clkevt.set_state_periodic = meson6_set_periodic;
	meson_timer->clkevt.set_state_oneshot = meson6_set_oneshot;
	meson_timer->clkevt.tick_resume = meson6_shutdown;
	meson_timer->clkevt.set_next_event = meson6_clkevt_next_event;
	meson_timer->clkevt.name = kasprintf(GFP_KERNEL, "%pOF#timer%c",
					     node, 'A' + idx);

	meson_timer->clkevt.irq = irq_of_parse_and_map(node, idx);
	if (!meson_timer->clkevt.irq) {
		pr_err("Failed to map interrupt #%d for %pOF\n", idx, node);
		return -EINVAL;
	}

	return request_irq(meson_timer->clkevt.irq, meson6_timer_interrupt,
			   IRQF_TIMER | IRQF_IRQPOLL, meson_timer->clkevt.name,
			   &meson_timer->clkevt);
}

static void meson6_timer_register(struct meson6_timer_clkevt *meson_timer)
{
	clockevents_config_and_register(&meson_timer->clkevt, USEC_PER_SEC, 1,
					0xfffe);
}

static int meson6_timer_starting_cpu(unsigned int cpu)
{
	struct meson6_timer_clkevt *meson_timer;
	struct clock_event_device *evt;

	if (cpu >= ARRAY_SIZE(meson_timers))
		return -EINVAL;

	meson_timer = &meson_timers[cpu];
	evt = &meson_timer->clkevt;

//	evt->cpumask = cpumask_of(cpu);

	if (cpu)
		irq_force_affinity(evt->irq, cpumask_of(cpu));

	enable_percpu_irq(evt->irq, IRQ_TYPE_NONE);

	meson6_timer_register(meson_timer);

	return 0;
}

static int meson6_timer_dying_cpu(unsigned int cpu)
{
	struct meson6_timer_clkevt *meson_timer;
	struct clock_event_device *evt;

	if (cpu >= ARRAY_SIZE(meson_timers))
		return -EINVAL;

	meson_timer = &meson_timers[cpu];
	evt = &meson_timer->clkevt;

	evt->set_state_shutdown(evt);
	disable_percpu_irq(evt->irq);

	return 0;
}

static int __init meson6_timer_init(struct device_node *node)
{
	u32 val;
	int ret, num_timers, i;

	timer_base = of_io_request_and_map(node, 0, "meson6-timer");
	if (IS_ERR(timer_base)) {
		pr_err("Can't map registers\n");
		return -ENXIO;
	}

	/* disable all timers */
	val = 0;

	/* set 1us input clock for all timers */
	val |= FIELD_PREP(MESON_ISA_TIMER_MUX_TIMERA_INPUT_CLOCK_MASK,
			  MESON_TIMERA_CLOCK_1US);
	val |= FIELD_PREP(MESON_ISA_TIMER_MUX_TIMERB_INPUT_CLOCK_MASK,
			  MESON_TIMERA_CLOCK_1US);
	val |= FIELD_PREP(MESON_ISA_TIMER_MUX_TIMERC_INPUT_CLOCK_MASK,
			  MESON_TIMERA_CLOCK_1US);
	val |= FIELD_PREP(MESON_ISA_TIMER_MUX_TIMERD_INPUT_CLOCK_MASK,
			  MESON_TIMERA_CLOCK_1US);
	val |= FIELD_PREP(MESON_ISA_TIMER_MUX_TIMERE_INPUT_CLOCK_MASK,
			  MESON_TIMERE_CLOCK_1US);

	writel(val, timer_base + MESON_ISA_TIMER_MUX);

	sched_clock_register(meson6_timer_sched_read, 32, USEC_PER_SEC);
	clocksource_mmio_init(timer_base + MESON_ISA_TIMERE, node->name,
			      1000 * 1000, 300, 32, clocksource_mmio_readl_up);

	/* old DTs only specify one IRQ so we only support one timer on them */
	num_timers = min_t(int, of_irq_count(node), ARRAY_SIZE(meson_timers));

	for (i = 0; i < num_timers; i++) {
		meson_timers[i].base = timer_base;

		ret = meson6_timer_init_clockevt(node, i);
		if (ret)
			return ret;
	}

	if (num_timers == 1)
		meson6_timer_register(&meson_timers[0]);
	else {
		ret = cpuhp_setup_state(CPUHP_AP_MESON6_TIMER_STARTING,
					"clockevents/meson6_timer:starting",
					meson6_timer_starting_cpu,
					meson6_timer_dying_cpu);
		if (ret) {
			pr_err("Failed to setup hotplug state and timer\n");
			return ret;
		}
	}

	return 0;
}

TIMER_OF_DECLARE(meson6, "amlogic,meson6-timer", meson6_timer_init);
