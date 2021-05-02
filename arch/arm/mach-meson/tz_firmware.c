// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2021 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 */

#include <linux/arm-smccc.h>
#include <linux/firmware/meson/meson_mx_trustzone.h>
#include <linux/memblock.h>
#include <linux/of.h>

#include <asm/firmware.h>
#include <asm/hardware/cache-l2x0.h>
#include <asm/outercache.h>

#include "tz_firmware.h"

struct meson_mx_trustzone_firmware_memconfig {
	unsigned char name[64];
	unsigned int start_phy_addr;
	unsigned int end_phy_addr;
} __packed;

static struct meson_mx_trustzone_firmware_memconfig meson_firmware_memconfig[2];

static int meson_mx_trustzone_firmware_hal_api(unsigned int cmd, u32 *args)
{
	struct arm_smccc_res res;

	arm_smccc_smc(MESON_CALL_TRUSTZONE_HAL_API, cmd, virt_to_phys(args), 0,
		      0, 0, 0, 0, &res);

	return res.a0;
}

static u32 meson_mx_trustzone_firmware_mon(unsigned int cmd, unsigned int arg0,
					   unsigned int arg1)
{
	struct arm_smccc_res res;

	arm_smccc_smc(MESON_CALL_TRUSTZONE_MON, cmd, arg0, arg1, 0, 0, 0, 0,
		      &res);

	return res.a0;
}

static int meson_mx_trustzone_firmware_set_cpu_boot_addr(int cpu,
							 unsigned long boot_addr){
	return meson_mx_trustzone_firmware_mon(MESON_TRUSTZONE_MON_CORE_BOOTADDR_INDEX,
					       cpu, boot_addr);
}

static int meson_mx_trustzone_firmware_cpu_boot(int cpu)
{
	u32 ret, corectrl;

	corectrl = meson_mx_trustzone_firmware_mon(MESON_TRUSTZONE_MON_CORE_RD_CTRL_INDEX,
						   0, 0);

	corectrl |= BIT(cpu);

	ret = meson_mx_trustzone_firmware_mon(MESON_TRUSTZONE_MON_CORE_WR_CTRL_INDEX,
					      corectrl, 0);
	if (ret != corectrl)
		return -EINVAL;

	return 0;
}

static void meson_mx_trustzone_firmware_l2x0_write_sec(unsigned long val,
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

	case L310_PREFETCH_CTRL:
		fn = MESON_TRUSTZONE_MON_L2X0_PREFETCH_INDEX;
		break;

	case L310_POWER_CTRL:
		fn = MESON_TRUSTZONE_MON_L2X0_POWER_INDEX;
		break;

	default:
		pr_warn("Amlogic Meson TrustZone - unsupported L2X0 register 0x%08x\n",
			reg);
		return;
	}

	WARN_ON(meson_mx_trustzone_firmware_mon(fn, val, 0));
}

static int __maybe_unused meson_mx_trustzone_firmware_l2x0_init(void)
{
	if (IS_ENABLED(CONFIG_CACHE_L2X0))
		outer_cache.write_sec = meson_mx_trustzone_firmware_l2x0_write_sec;

	return 0;
}

static const struct firmware_ops meson_mx_trustzone_firmware_ops = {
	.set_cpu_boot_addr	= meson_mx_trustzone_firmware_set_cpu_boot_addr,
	.cpu_boot		= meson_mx_trustzone_firmware_cpu_boot,
	.l2x0_init		= meson_mx_trustzone_firmware_l2x0_init,
};

void __init meson_mx_trustzone_firmware_init(void)
{
	if (!meson_mx_trustzone_firmware_available())
		return;

	pr_info("Running under Amlogic Meson TrustZone secure firmware.\n");

	register_firmware_ops(&meson_mx_trustzone_firmware_ops);

	call_firmware_op(l2x0_init);
}

static void __init meson_mx_trustzone_firmware_memconfig_init(void)
{
	unsigned int i, size;
	u32 args[2] = {
		__pa_symbol(meson_firmware_memconfig),
		ARRAY_SIZE(meson_firmware_memconfig),
	};
	int ret;

	ret = meson_mx_trustzone_firmware_hal_api(MESON_TRUSTZONE_HAL_API_MEMCONFIG,
						  args);
	if (ret) {
		pr_err("Amlogic Meson TrustZone memconfig failed: %d\n", ret);
		return;
	}

	for (i = 0; i < ARRAY_SIZE(meson_firmware_memconfig); i++) {
		size = meson_firmware_memconfig[i].end_phy_addr -
		       meson_firmware_memconfig[i].start_phy_addr;

		pr_debug("\tAmlogic Meson TrustZone memblock[%d]: %s (%u bytes)\n",
			 i, meson_firmware_memconfig[i].name, size);

		ret = memblock_mark_nomap(meson_firmware_memconfig[i].start_phy_addr,
					  size);
		if (ret)
			pr_err("Failed to reserve %u bytes for Amlogic Meson TrustZone memblock[%d] (%s): %d\n",
			       size, i, meson_firmware_memconfig[i].name, ret);
	}
}

static void __init meson_mx_trustzone_firmware_monitor_memory_init(void)
{
	u32 base, size;
	int ret;

	base = meson_mx_trustzone_firmware_mon(MESON_TRUSTZONE_MON_MEM_BASE,
					       0, 0);
	WARN_ON(!base);

	size = meson_mx_trustzone_firmware_mon(MESON_TRUSTZONE_MON_MEM_TOTAL_SIZE,
					       0, 0);
	WARN_ON(!size);

	ret = memblock_mark_nomap(base, size);
	if (ret)
		pr_err("Failed to reserve %u bytes of Amlogic Meson TrustZone monitor memory: %d\n",
		       size, ret);
}

void __init meson_mx_trustzone_firmware_reserve_mem(void)
{
	if (!meson_mx_trustzone_firmware_available())
		return;

	meson_mx_trustzone_firmware_monitor_memory_init();
	meson_mx_trustzone_firmware_memconfig_init();
}

bool meson_mx_trustzone_firmware_available(void)
{
	struct device_node *np;

	np = of_find_compatible_node(NULL, NULL,
				     "amlogic,meson-mx-trustzone-firmware");
	if (!np)
		return false;

	of_node_put(np);

	return true;
}
EXPORT_SYMBOL_GPL(meson_mx_trustzone_firmware_available);

int meson_mx_trustzone_firmware_efuse_read(unsigned int offset,
					   unsigned int bytes, void *buf)
{
	unsigned int read_bytes;
	u32 args[5] = {
		MESON_TRUSTZONE_HAL_API_EFUSE_CMD_READ,
		offset,
		bytes,
		__pa_symbol(buf),
		virt_to_phys(&read_bytes)
	};
	int ret;

	ret = meson_mx_trustzone_firmware_hal_api(MESON_TRUSTZONE_HAL_API_EFUSE,
						  args);
	if (ret)
		return -EIO;

	if (read_bytes != bytes)
		return -EINVAL;

	return 0;
}
EXPORT_SYMBOL_GPL(meson_mx_trustzone_firmware_efuse_read);

u32 meson_mx_trustzone_read_soc_rev1(void)
{
	return meson_mx_trustzone_firmware_mon(MESON_TRUSTZONE_MON_CORE_RD_SOC_REV1,
					       0, 0);
}
EXPORT_SYMBOL_GPL(meson_mx_trustzone_read_soc_rev1);
