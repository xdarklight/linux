// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2021 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 */

#include <linux/errno.h>
#include <linux/types.h>

#if defined(CONFIG_ARM) && defined(CONFIG_ARCH_MESON)

bool meson_mx_trustzone_firmware_available(void);

int meson_mx_trustzone_firmware_efuse_read(unsigned int offset,
					   unsigned int bytes, void *buf);

unsigned int meson_mx_trustzone_read_soc_rev1(void);

int meson_mx_trustzone_firmware_auxcoreboot_addr(unsigned int cpu,
						 unsigned int addr);
int meson_mx_trustzone_firmware_modify_corectrl(unsigned int cpu,
						bool power_on);

#else

static inline bool meson_mx_trustzone_firmware_available(void)
{
	return false;
}

static inline int meson_mx_trustzone_firmware_efuse_read(unsigned int offset,
							 unsigned int bytes,
							 void *buf)
{
	return -EINVAL;
}

static inline unsigned int meson_mx_trustzone_read_soc_rev1(void)
{
	return 0;
}

#endif
