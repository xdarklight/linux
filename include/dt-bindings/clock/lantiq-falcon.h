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

#define SYS1_GATE_SERIAL1	0
#define SYS1_GATE_SERIAL0	1
#define SYS1_GATE_SPI		2
#define SYS1_GATE_I2C		3
#define SYS1_GATE_GPTC		4
#define SYS1_GATE_GPIO1		5
#define SYS1_GATE_GPIO3		6
#define SYS1_GATE_GPIO4		7
#define SYS1_GATE_PADCTRL1	8
#define SYS1_GATE_PADCTRL3	9
#define SYS1_GATE_PADCTRL4	10

#define SYSETH_GATE_GPIO0	0
#define SYSETH_GATE_GPIO2	1
#define SYSETH_GATE_PADCTRL0	2
#define SYSETH_GATE_PADCTRL2	3

#endif /* _DT_BINDINGS_CLOCK_LANTIQ_FALCON_H */
