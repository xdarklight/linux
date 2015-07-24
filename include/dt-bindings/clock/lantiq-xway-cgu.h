/*
 * Copyright (c) 2015 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Device Tree binding constants for the CGU module on lantiq xway SoCs.
 */

#ifndef _DT_BINDINGS_CLOCK_LANTIQ_XWAY_CGU_H
#define _DT_BINDINGS_CLOCK_LANTIQ_XWAY_CGU_H

#define CGU_CLK_CPU		0
#define CGU_CLK_FPI		1
#define CGU_CLK_IO		2
#define CGU_CLK_PP32		3
#define CGU_CLK_CKOUT0		4
#define CGU_CLK_CKOUT1		5
#define CGU_CLK_CKOUT2		6
#define CGU_CLK_CKOUT3		7
#define CGU_CLK_PCI		8	/* Only on SoCs with PCI support */
#define CGU_CLK_PCI_EXT		9	/* Only on SoCs with PCI support */
#define CGU_CLK_EPHY		10	/* Only on Amazon SE SoCs */

#define NUM_CGU_CLOCKS		(CGU_CLK_EPHY + 1)

/* These are only supported by SoCs with GPHY (VR9 and newer): */
/* XTAL 36 MHz input */
#define CGU_GPHY_CLK_36MHZ_XTAL		0
/* 25 MHz from PLL0 with divider */
#define CGU_GPHY_CLK_25MHZ_PLL0		1
/* derived from PLL2 output (XTAL is 36 MHz) */
#define CGU_GPHY_CLK_24MHZ_PLL2		2
/* 25 MHz Clock from Pin GPIO3 */
#define CGU_GPHY_CLK_25MHZ_GPIO3	3

#endif /* _DT_BINDINGS_CLOCK_LANTIQ_XWAY_CGU_H */
