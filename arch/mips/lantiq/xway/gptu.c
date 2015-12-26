/*
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 *
 *  Copyright (C) 2012 John Crispin <blogic@openwrt.org>
 *  Copyright (C) 2012 Lantiq GmbH
 */

#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_irq.h>

#include <lantiq_soc.h>

/* the magic ID byte of the core */
#define GPTU_MAGIC	0x59
/* clock control register */
#define GPTU_CLC	0x00
/* id register */
#define GPTU_ID		0x08
/* interrupt node enable */
#define GPTU_IRNEN	0xf4
/* interrupt control register */
#define GPTU_IRCR	0xf8
/* interrupt capture register */
#define GPTU_IRNCR	0xfc
/* there are 3 identical blocks of 2 timers. calculate register offsets */
#define GPTU_SHIFT(x)	(x % 2 ? 4 : 0)
#define GPTU_BASE(x)	(((x >> 1) * 0x20) + 0x10)
/* timer control register */
#define GPTU_CON(x)	(GPTU_BASE(x) + GPTU_SHIFT(x) + 0x00)
/* timer auto reload register */
#define GPTU_RUN(x)	(GPTU_BASE(x) + GPTU_SHIFT(x) + 0x08)
/* timer manual reload register */
#define GPTU_RLD(x)	(GPTU_BASE(x) + GPTU_SHIFT(x) + 0x10)
/* timer count register */
#define GPTU_CNT(x)	(GPTU_BASE(x) + GPTU_SHIFT(x) + 0x18)

/* GPTU_CON(x) */
#define CON_CNT		BIT(2)
#define CON_EDGE_ANY	(BIT(7) | BIT(6))
#define CON_SYNC	BIT(8)
#define CON_CLK_INT	BIT(10)

/* GPTU_RUN(x) */
#define RUN_SEN		BIT(0)
#define RUN_RL		BIT(2)

/* set clock to runmode */
#define CLC_RMC		BIT(8)
/* bring core out of suspend */
#define CLC_SUSPEND	BIT(4)
/* the disable bit */
#define CLC_DISABLE	BIT(0)

#define gptu_w32(x, y)	ltq_w32((x), gptu_membase + (y))
#define gptu_r32(x)	ltq_r32(gptu_membase + (x))

enum gptu_timer {
	TIMER1A = 0,
	TIMER1B,
	TIMER2A,
	TIMER2B,
	TIMER3A,
	TIMER3B,
	NUM_GPTU /* Must be last */
};

static void __iomem *gptu_membase;
static struct resource irqres[NUM_GPTU];

struct gptu_clk {
	struct clk_hw	hw;
	enum gptu_timer	timer;
};

static inline struct gptu_clk *to_gptu_clk(struct clk_hw *hw)
{
	return container_of(hw, struct gptu_clk, hw);
}

static irqreturn_t timer_irq_handler(int irq, void *priv)
{
	int timer = irq - irqres[0].start;
	gptu_w32(1 << timer, GPTU_IRNCR);
	return IRQ_HANDLED;
}

static void gptu_hwinit(void)
{
	gptu_w32(0x00, GPTU_IRNEN);
	gptu_w32(0xff, GPTU_IRNCR);
	gptu_w32(CLC_RMC | CLC_SUSPEND, GPTU_CLC);
}

static void gptu_hwexit(void)
{
	gptu_w32(0x00, GPTU_IRNEN);
	gptu_w32(0xff, GPTU_IRNCR);
	gptu_w32(CLC_DISABLE, GPTU_CLC);
}

static int gptu_enable(struct clk_hw *hw)
{
	int ret;
	struct gptu_clk *clk = to_gptu_clk(hw);

	ret = request_irq(irqres[clk->timer].start, timer_irq_handler,
		IRQF_TIMER, "gtpu", NULL);
	if (ret) {
		pr_err("gptu: failed to request irq\n");
		return ret;
	}

	gptu_w32(CON_CNT | CON_EDGE_ANY | CON_SYNC | CON_CLK_INT,
		GPTU_CON(clk->timer));
	gptu_w32(1, GPTU_RLD(clk->timer));
	gptu_w32(gptu_r32(GPTU_IRNEN) | BIT(clk->timer), GPTU_IRNEN);
	gptu_w32(RUN_SEN | RUN_RL, GPTU_RUN(clk->timer));

	return 0;
}

static void gptu_disable(struct clk_hw *hw)
{
	struct gptu_clk *clk = to_gptu_clk(hw);

	gptu_w32(0, GPTU_RUN(clk->timer));
	gptu_w32(0, GPTU_CON(clk->timer));
	gptu_w32(0, GPTU_RLD(clk->timer));
	gptu_w32(gptu_r32(GPTU_IRNEN) & ~BIT(clk->timer), GPTU_IRNEN);

	free_irq(irqres[clk->timer].start, NULL);
}

const struct clk_ops gptu_timer_clk_ops = {
	.enable = gptu_enable,
	.disable = gptu_disable,
};

static struct clk *gptu_register_timer_clk(struct device *dev,
						const char *name,
						enum gptu_timer timer)
{
	struct gptu_clk *gptu_timer;
	struct clk *clk;
	struct clk_init_data init;

	gptu_timer = kzalloc(sizeof(*gptu_timer), GFP_KERNEL);
	if (!gptu_timer)
		return ERR_PTR(-ENOMEM);

	init.name = name;
	init.ops = &gptu_timer_clk_ops;
	init.flags = CLK_IS_ROOT;
	init.parent_names = NULL;
	init.num_parents = 0;

	gptu_timer->timer = timer;
	gptu_timer->hw.init = &init;

	clk = clk_register(NULL, &gptu_timer->hw);
	if (IS_ERR(clk)) {
		kfree(gptu_timer);
		return clk;
	}

	if (clk_register_clkdev(clk, name, dev_name(dev)))
		pr_err("%s: Failed to register lookup for %s\n",
		       __func__, name);

	return clk;
}

static void gptu_register_clks(struct platform_device *pdev)
{
	gptu_register_timer_clk(&pdev->dev, "timer1a", TIMER1A);
	gptu_register_timer_clk(&pdev->dev, "timer1b", TIMER1B);
	gptu_register_timer_clk(&pdev->dev, "timer2a", TIMER2A);
	gptu_register_timer_clk(&pdev->dev, "timer2b", TIMER2B);
	gptu_register_timer_clk(&pdev->dev, "timer3a", TIMER3A);
	gptu_register_timer_clk(&pdev->dev, "timer3b", TIMER3B);
}

static int gptu_probe(struct platform_device *pdev)
{
	struct clk *clk;
	struct resource *res;
	int ret;

	ret = of_irq_to_resource_table(pdev->dev.of_node, irqres, NUM_GPTU);
	if (ret != NUM_GPTU) {
		dev_err(&pdev->dev, "Failed to get IRQ list\n");
		return -EINVAL;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	/* remap gptu register range */
	gptu_membase = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(gptu_membase))
		return PTR_ERR(gptu_membase);

	/* enable our clock */
	clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "Failed to get clock\n");
		return -ENOENT;
	}
	clk_prepare_enable(clk);

	/* power up the core */
	gptu_hwinit();

	/* the gptu has a ID register */
	if (((gptu_r32(GPTU_ID) >> 8) & 0xff) != GPTU_MAGIC) {
		dev_err(&pdev->dev, "Failed to find magic\n");
		gptu_hwexit();
		clk_disable(clk);
		clk_put(clk);
		return -ENAVAIL;
	}

	gptu_register_clks(pdev);

	dev_info(&pdev->dev, "gptu: %d timers loaded\n", NUM_GPTU);

	return 0;
}

static const struct of_device_id gptu_match[] = {
	{ .compatible = "lantiq,gptu-xway" },
	{},
};
MODULE_DEVICE_TABLE(of, dma_match);

static struct platform_driver dma_driver = {
	.probe = gptu_probe,
	.driver = {
		.name = "gptu-xway",
		.of_match_table = gptu_match,
	},
};

int __init gptu_init(void)
{
	int ret = platform_driver_register(&dma_driver);

	if (ret)
		pr_err("gptu: Error registering platform driver\n");
	return ret;
}

arch_initcall(gptu_init);
