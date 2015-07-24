/*
 * Copyright (c) 2015 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Device Tree binding constants for the SYS clocks and gates on SoCs of
 * lantiq's "falcon" family.
 */

#ifndef _DT_BINDINGS_CLOCK_LANTIQ_FALCON_H
#define _DT_BINDINGS_CLOCK_LANTIQ_FALCON_H

#define SYSCTL_CLK_CPU			0
#define SYSCTL_CLK_FPI			1
#define SYSCTL_CLK_IO			2
#define SYSCTL_GATE_GPIO0		3
#define SYSCTL_GATE_GPIO1		4
#define SYSCTL_GATE_GPIO2		5
#define SYSCTL_GATE_GPIO3		6
#define SYSCTL_GATE_GPIO4		7
#define SYSCTL_GATE_PADCTRL0		8
#define SYSCTL_GATE_PADCTRL1		9
#define SYSCTL_GATE_PADCTRL2		10
#define SYSCTL_GATE_PADCTRL3		11
#define SYSCTL_GATE_PADCTRL4		12
#define SYSCTL_GATE_SERIAL1		13
#define SYSCTL_GATE_SERIAL0		14
#define SYSCTL_GATE_SPI			15
#define SYSCTL_GATE_I2C			16

#define NUM_SYSCTL_CLOCKS		(SYSCTL_GATE_I2C + 1)

#endif /* _DT_BINDINGS_CLOCK_LANTIQ_FALCON_H */
