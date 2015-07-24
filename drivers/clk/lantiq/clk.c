/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 * Copyright (C) 2015 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 */

#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/types.h>

#include "clk.h"

static int __ltq_register_clkdev(struct clk *clk, const char *name)
{
	int ret;

	ret = clk_register_clkdev(clk, name, NULL);
	if (ret)
		pr_err("%s: could not register clkdev %s\n", __func__, name);

	return ret;
}

inline int ltq_register_cpu_clkdev(struct clk *clk)
{
	return __ltq_register_clkdev(clk, "cpu");
}

inline int ltq_register_fpi_clkdev(struct clk *clk)
{
	return __ltq_register_clkdev(clk, "fpi");
}

inline int ltq_register_io_clkdev(struct clk *clk)
{
	return __ltq_register_clkdev(clk, "io");
}

inline int ltq_register_pp32_clkdev(struct clk *clk)
{
	return __ltq_register_clkdev(clk, "pp32");
}
