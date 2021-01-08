// SPDX-License-Identifier: GPL-2.0-only
/*
 *
 * Copyright (C) 2010 John Crispin <john@phrozen.org>
 * Copyright (C) 2010 Thomas Langer <thomas.langer@lantiq.com>
 */

#include <linux/irqchip.h>

#include <asm/irq.h>

unsigned int get_c0_compare_int(void)
{
	return CP0_LEGACY_COMPARE_IRQ;
}

void __init arch_init_irq(void)
{
	irqchip_init();
}
