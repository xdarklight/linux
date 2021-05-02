// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2021 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 */

#include <linux/arm-smccc.h>
#include <linux/firmware/meson/meson_mx_tz.h>
#include <linux/memblock.h>
#include <linux/of.h>

#include <asm/firmware.h>
#include <asm/hardware/cache-l2x0.h>
#include <asm/outercache.h>

#include "firmware.h"

static bool meson_has_secure_firmware = false;
static struct meson_secure_firmware_memconfig {
	unsigned char name[64];
	unsigned int start_phy_addr;
	unsigned int end_phy_addr;
} memconfig[2];

static int meson_secure_firmware_hal_api(unsigned int cmd, u32 *args)
{
	struct arm_smccc_res res;

	arm_smccc_smc(MESON_CALL_TRUSTZONE_HAL_API, cmd, __pa_symbol(args), 0,
		      0, 0, 0, 0, &res);

	return res.a0;
}

static unsigned long meson_secure_firmware_mon(unsigned int cmd,
					       unsigned int arg0,
					       unsigned int arg1)
{
	struct arm_smccc_res res;

	arm_smccc_smc(MESON_CALL_TRUSTZONE_MON, cmd, arg0, arg1, 0, 0, 0, 0,
		      &res);

	return res.a0;
}

static int __init meson_secure_firmware_memconfig_init(void)
{
	u32 args[2] = {
		__pa_symbol(memconfig),
		ARRAY_SIZE(memconfig),
	};

	return meson_secure_firmware_hal_api(MESON_TRUSTZONE_HAL_API_MEMCONFIG,
					     args);
}

static void meson_secure_firmware_l2x0_write_sec(unsigned long val,
						 unsigned int reg)
{
	u32 fn;

	switch (reg) {
	case L2X0_CTRL:
		fn = MESON_TRUSTZONE_MON_L2X0_CTRL_INDEX;
		break;

	case L2X0_AUX_CTRL:
		fn = MESON_TRUSTZONE_MON_L2X0_AUXCTRL_INDEX;
		break;

	case L310_TAG_LATENCY_CTRL:
		fn = MESON_TRUSTZONE_MON_L2X0_TAGLATENCY_INDEX;
		break;

	case L310_DATA_LATENCY_CTRL:
		fn = MESON_TRUSTZONE_MON_L2X0_DATALATENCY_INDEX;
		break;

	case L310_ADDR_FILTER_START:
		fn = MESON_TRUSTZONE_MON_L2X0_FILTERSTART_INDEX;
		break;

	case L310_ADDR_FILTER_END:
		fn = MESON_TRUSTZONE_MON_L2X0_FILTEREND_INDEX;
		break;

	case L2X0_DEBUG_CTRL:
		fn = MESON_TRUSTZONE_MON_L2X0_DEBUG_INDEX;
		break;

	case L310_POWER_CTRL:
		fn = MESON_TRUSTZONE_MON_L2X0_POWER_INDEX;
		break;

	default:
		pr_warn("%s: unknown reg 0x%08x\n", __func__, reg);
		return;
	}

	WARN_ON(meson_secure_firmware_mon(fn, val, 0));
}

bool meson_secure_firmware_available(void)
{
	return meson_has_secure_firmware;
}

int meson_secure_firmware_auxcoreboot_addr(u8 cpu, u32 addr)
{
	return meson_secure_firmware_mon(MESON_TRUSTZONE_MON_CORE_BOOTADDR_INDEX,
					 cpu, addr);
}

int meson_secure_firmware_modify_corectrl(u8 cpu, bool power_on)
{
	u32 corectrl;

	corectrl = meson_secure_firmware_mon(MESON_TRUSTZONE_MON_CORE_RD_CTRL_INDEX,
					     0, 0);

	if (power_on)
		corectrl |= BIT(cpu);
	else
		corectrl &= ~BIT(cpu);

	return meson_secure_firmware_mon(MESON_TRUSTZONE_MON_CORE_WR_CTRL_INDEX,
					 corectrl, 0);
}

int meson_secure_firmware_efuse_read(unsigned int offset, unsigned int bytes,
				     void *buf)
{
	unsigned int read_bytes;
	u32 args[5] = {
		MESON_TRUSTZONE_HAL_API_EFUSE_CMD_READ,
		offset,
		bytes,
		__pa_symbol(buf),
		__pa(&read_bytes)
	};
	int ret;

	ret = meson_secure_firmware_hal_api(MESON_TRUSTZONE_HAL_API_EFUSE,
					    args);
	if (ret)
		return -EIO;

	if (read_bytes != bytes)
		return -EINVAL;

	return 0;
}

void __init meson_secure_firmware_init(void)
{
	struct device_node *np;
	unsigned int i, len;
	int ret;

	np = of_find_compatible_node(NULL, NULL,
				     "amlogic,meson-mx-secure-firmware");
	if (!np)
		return;

	of_node_put(np);
	meson_has_secure_firmware = true;

	pr_info("Running under secure (TrustZone) firmware.\n");

	if (IS_ENABLED(CONFIG_CACHE_L2X0))
		outer_cache.write_sec = meson_secure_firmware_l2x0_write_sec;

	ret = meson_secure_firmware_memconfig_init();
	if (ret) {
		pr_err("Failed to initialize memconfig: %d\n", ret);
		return;
	}

	for (i = 0; i < ARRAY_SIZE(memconfig); i++) {
		len = memconfig[i].end_phy_addr - memconfig[i].start_phy_addr;

		pr_debug("\tTrustZone memblock[%d]: %s (%u bytes)\n", i,
			 memconfig[i].name, len);

		memblock_reserve(memconfig[i].start_phy_addr, len);
	}
}
