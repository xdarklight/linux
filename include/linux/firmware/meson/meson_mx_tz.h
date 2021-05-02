// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2021 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 */

#include <linux/errno.h>

#if defined(CONFIG_ARM) && defined(CONFIG_ARCH_MESON)

bool meson_secure_firmware_available(void);

int meson_secure_firmware_efuse_read(unsigned int offset, unsigned int bytes,
				     void *buf);

int meson_secure_firmware_auxcoreboot_addr(u8 cpu, u32 addr);
int meson_secure_firmware_modify_corectrl(u8 cpu, bool power_on);

#else

static inline bool meson_secure_firmware_available(void)
{
	return false;
}

static inline int meson_secure_firmware_efuse_read(unsigned int offset,
						   unsigned int bytes,
						   void *buf)
{
	return -EINVAL;
}

#endif
