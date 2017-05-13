/*
 * Copyright (C) 2015 Carlo Caione <carlo@endlessm.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */

#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/smp.h>
#include <linux/mfd/syscon.h>
#include <asm/smp_scu.h>
#include <asm/smp_plat.h>

#define MESON_CPU_CTRL_REG			(0x00)
#define MESON_CPU_CTRL_ADDR_REG(c)		(0x04 + ((c - 1) << 2))

#define MESON_CPU_AO_RTI_PWR_A9_CNTL0		(0x00)
#define MESON_CPU_AO_RTI_PWR_A9_CNTL1		(0x04)
#define MESON_CPU_AO_RTI_PWR_A9_MEM_PD0		(0x14)

#define MESON_CPU_PWR_A9_CNTL0_M(c)		(0x03 << ((c * 2) + 16))
#define MESON_CPU_PWR_A9_CNTL1_M(c)		(0x03 << ((c + 1) << 1))
#define MESON_CPU_PWR_A9_MEM_PD0_M(c)		(0x0f << (32 - (c * 4)))
#define MESON_CPU_PWR_A9_CNTL1_ST(c)		(0x01 << (c + 16))

static void __iomem *sram_base;
static void __iomem *scu_base;
static struct regmap *pmu;

static struct reset_control *meson_smp_get_core_reset(int cpu)
{
	struct device_node *np = of_get_cpu_node(cpu, 0);

	return of_reset_control_get_exclusive(np, NULL);
}

static void meson_smp_set_cpu_ctrl(int cpu, bool on_off)
{
	u32 val = readl(sram_base + MESON_CPU_CTRL_REG) | BIT(0);

	if (on_off)
		val |= BIT(cpu);
	else
		val &= BIT(cpu);

	writel(val, sram_base + MESON_CPU_CTRL_REG);
}

static void __init meson_smp_prepare_cpus(const char *scu_compatible,
					  const char *pmu_compatible)
{
	static struct device_node *node;

	/* SMP SRAM */
	node = of_find_compatible_node(NULL, NULL, "amlogic,meson-mx-smp-sram");
	if (!node) {
		pr_err("Missing SRAM node\n");
		return;
	}

	sram_base = of_iomap(node, 0);
	if (!sram_base) {
		pr_err("Couldn't map SRAM registers\n");
		return;
	}

	/* PMU */
	pmu = syscon_regmap_lookup_by_compatible(pmu_compatible);
	if (IS_ERR(pmu)) {
		pr_err("Couldn't map PMU registers\n");
		return;
	}

	/* SCU */
	node = of_find_compatible_node(NULL, NULL, scu_compatible);
	if (!node) {
		pr_err("Missing SCU node\n");
		return;
	}

	scu_base = of_iomap(node, 0);
	if (!scu_base) {
		pr_err("Couln't map SCU registers\n");
		return;
	}

	scu_enable(scu_base);
}

static void __init meson8b_smp_prepare_cpus(unsigned int max_cpus)
{
	meson_smp_prepare_cpus("arm,cortex-a5-scu", "amlogic,meson8b-pmu");
}

static void __init meson8_smp_prepare_cpus(unsigned int max_cpus)
{
	meson_smp_prepare_cpus("arm,cortex-a9-scu", "amlogic,meson8-pmu");
}

static int meson8_smp_boot_secondary(unsigned int cpu,
				     struct task_struct *idle)
{
	struct reset_control *rstc;
	unsigned long timeout;
	int ret;
	u32 val;

	rstc = meson_smp_get_core_reset(cpu);
	if (IS_ERR(rstc)) {
		pr_err("Couldn't get the reset controller for CPU%d\n", cpu);
		return PTR_ERR(rstc);
	}

	/* CPU power UP */
	ret = regmap_update_bits(pmu, MESON_CPU_AO_RTI_PWR_A9_CNTL0,
				 MESON_CPU_PWR_A9_CNTL0_M(cpu), 0);
	if (ret < 0) {
		pr_err("Couldn't power up CPU%d\n", cpu);
		return ret;
	}

	udelay(5);

	/* Reset enable */
	reset_control_assert(rstc);

	/* Memory power UP */
	ret = regmap_update_bits(pmu, MESON_CPU_AO_RTI_PWR_A9_MEM_PD0,
				 MESON_CPU_PWR_A9_MEM_PD0_M(cpu), 0);
	if (ret < 0) {
		pr_err("Couldn't power up the memory for CPU%d\n", cpu);
		return ret;
	}

	/* Wake up CPU */
	ret = regmap_update_bits(pmu, MESON_CPU_AO_RTI_PWR_A9_CNTL1,
				 MESON_CPU_PWR_A9_CNTL1_M(cpu), 0);
	if (ret < 0) {
		pr_err("Couldn't wake up CPU%d\n", cpu);
		return ret;
	}

	udelay(10);

	do {
		ret = regmap_read(pmu, MESON_CPU_AO_RTI_PWR_A9_CNTL1, &val);
		if (ret < 0) {
			pr_err("CPU%d wake up failed\n", cpu);
			return ret;
		}
	} while (!(val & MESON_CPU_PWR_A9_CNTL1_ST(cpu)));

	/* Isolation disable */
	ret = regmap_update_bits(pmu, MESON_CPU_AO_RTI_PWR_A9_CNTL0, BIT(cpu),
				 0);
	if (ret < 0) {
		pr_err("Error when disabling isolation of CPU%d\n", cpu);
		return ret;
	}

	/* Reset disable */
	reset_control_deassert(rstc);

	timeout = jiffies + (10 * HZ);
	while (readl(sram_base + MESON_CPU_CTRL_ADDR_REG(cpu))) {
		if (!time_before(jiffies, timeout))
			return -EPERM;
	}

	udelay(100);
	writel(virt_to_phys(secondary_startup),
	       sram_base + MESON_CPU_CTRL_ADDR_REG(cpu));

	meson_smp_set_cpu_ctrl(cpu, true);

	return 0;
}

static void meson8_smp_secondary_init(unsigned int cpu)
{
	scu_power_mode(scu_base, SCU_PM_NORMAL);
}

#ifdef CONFIG_HOTPLUG_CPU
static void meson8_smp_cpu_die(unsigned int cpu)
{
	meson_smp_set_cpu_ctrl(cpu, false);

// TODO:	flush_cache_all();
	dsb();
	dmb();

// TODO:	meson_cleanup();
	scu_power_mode(scu_base, SCU_PM_POWEROFF);

	dsb();
	wfi();

	BUG();
}

static int meson8_smp_cpu_kill(unsigned int cpu)
{
	int ret;
#if 0
	u32 val;
	unsigned int offset=(cpu<<3);

	do {
		udelay(10);
		value=aml_read_reg32(MESON_CPU_POWER_CTRL_REG);
	} while((value&(3<<offset)) != (3<<offset));
#endif
	udelay(10);

	/* CPU power DOWN */
	ret = regmap_update_bits(pmu, MESON_CPU_AO_RTI_PWR_A9_CNTL0,
				 MESON_CPU_PWR_A9_CNTL0_M(cpu), 0x3);
	if (ret < 0) {
		pr_err("Couldn't power down CPU%d\n", cpu);
		return ret;
	}

	/* Isolation enable */
	ret = regmap_update_bits(pmu, MESON_CPU_AO_RTI_PWR_A9_CNTL0, BIT(cpu),
				 0x3);
	if (ret < 0) {
		pr_err("Error when enabling isolation for CPU%d\n", cpu);
		return ret;
	}

	udelay(10);

	/* Sleep status */
	ret = regmap_update_bits(pmu, MESON_CPU_AO_RTI_PWR_A9_CNTL1,
				 MESON_CPU_PWR_A9_CNTL1_M(cpu), 0x3);
	if (ret < 0) {
		pr_err("Couldn't change sleep status of CPU%d\n", cpu);
		return ret;
	}

	/* Memory power DOWN */
	ret = regmap_update_bits(pmu, MESON_CPU_AO_RTI_PWR_A9_MEM_PD0,
				 MESON_CPU_PWR_A9_MEM_PD0_M(cpu), 0xf);
	if (ret < 0) {
		pr_err("Couldn't power down the memory of CPU%d\n", cpu);
		return ret;
	}

	return 0;
}
#endif

static struct smp_operations meson8_smp_ops __initdata = {
	.smp_prepare_cpus	= meson8_smp_prepare_cpus,
	.smp_boot_secondary	= meson8_smp_boot_secondary,
	.smp_secondary_init	= meson8_smp_secondary_init,
#ifdef CONFIG_HOTPLUG_CPU
	.cpu_die		= meson8_smp_cpu_die,
	.cpu_kill		= meson8_smp_cpu_kill,
#endif
};

static struct smp_operations meson8b_smp_ops __initdata = {
	.smp_prepare_cpus	= meson8b_smp_prepare_cpus,
	.smp_boot_secondary	= meson8_smp_boot_secondary,
	.smp_secondary_init	= meson8_smp_secondary_init,
#ifdef CONFIG_HOTPLUG_CPU
	.cpu_die		= meson8_smp_cpu_die,
	.cpu_kill		= meson8_smp_cpu_kill,
#endif
};

CPU_METHOD_OF_DECLARE(meson8_smp, "amlogic,meson8-smp", &meson8_smp_ops);
CPU_METHOD_OF_DECLARE(meson8b_smp, "amlogic,meson8b-smp", &meson8b_smp_ops);
