#include <dt-bindings/clock/lantiq-falcon.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/types.h>

#include "clk.h"

/* SYSCTL - start/stop/restart/configure/... different parts of the Soc */
#define SYSCTL_SYS1		0
#define SYSCTL_SYSETH		1
#define SYSCTL_SYSGPE		2

/* infrastructure control register */
#define SYS1_INFRAC		0x00bc
/* Configuration fuses for drivers and pll */
#define STATUS_CONFIG		0x0040

/* GPE frequency selection */
#define GPPC_OFFSET		24
#define GPEFREQ_MASK		0x00000C0
#define GPEFREQ_OFFSET		10
/* Clock status register */
#define SYSCTL_CLKS		0x0000
/* Clock enable register */
#define SYSCTL_CLKEN		0x0004
/* Clock clear register */
#define SYSCTL_CLKCLR		0x0008
/* Activation Status Register */
#define SYSCTL_ACTS		0x0020
/* Activation Register */
#define SYSCTL_ACT		0x0024
/* Deactivation Register */
#define SYSCTL_DEACT		0x0028
/* reboot Register */
#define SYSCTL_RBT		0x002c
/* CPU0 Clock Control Register */
#define SYS1_CPU0CC		0x0040
/* HRST_OUT_N Control Register */
#define SYS1_HRSTOUTC		0x00c0
/* clock divider bit */
#define CPU0CC_CPUDIV		0x0001

/* Activation Status Register bits */
#define ACTS_ASC0_ACT		12
#define ACTS_SSC0		13
#define ACTS_ASC1_ACT		11
#define ACTS_I2C_ACT		14
#define ACTS_P0			16
#define ACTS_P1			16
#define ACTS_P2			17
#define ACTS_P3			17
#define ACTS_P4			18
#define ACTS_PADCTRL0		20
#define ACTS_PADCTRL1		20
#define ACTS_PADCTRL2		21
#define ACTS_PADCTRL3		21
#define ACTS_PADCTRL4		22

#define sysctl_w32(m, x, y)	__raw_writel((x), sysctl_membase[m] + (y))
#define sysctl_r32(m, x)	__raw_readl(sysctl_membase[m] + (x))
#define sysctl_w32_mask(m, clear, set, reg)	\
		sysctl_w32(m, (sysctl_r32(m, reg) & ~(clear)) | (set), reg)

#define status_r32(x)		__raw_readl(status_membase + (x))

static void __iomem *sysctl_membase[3], *status_membase;
void __iomem *ltq_sys1_membase;

static struct clk_onecell_data sysclk_data;

struct falcon_clk_gate {
	struct clk_hw	hw;
	void __iomem	*reg_base;
	u8		bit_idx;
};

static inline struct falcon_clk_gate *to_falcon_clk_gate(struct clk_hw *hw)
{
	 return container_of(hw, struct falcon_clk_gate, hw);
}

/* enable the ONU core */
static void falcon_gpe_enable(void)
{
	unsigned int freq;
	unsigned int status;

	/* if if the clock is already enabled */
	status = sysctl_r32(SYSCTL_SYS1, SYS1_INFRAC);
	if (status & (1 << (GPPC_OFFSET + 1)))
		return;

	freq = (status_r32(STATUS_CONFIG) &
		GPEFREQ_MASK) >>
		GPEFREQ_OFFSET;
	if (freq == 0)
		freq = 1; /* use 625MHz on unfused chip */

	/* apply new frequency */
	sysctl_w32_mask(SYSCTL_SYS1, 7 << (GPPC_OFFSET + 1),
			freq << (GPPC_OFFSET + 2), SYS1_INFRAC);
	udelay(1);

	/* enable new frequency */
	sysctl_w32_mask(SYSCTL_SYS1, 0, 1 << (GPPC_OFFSET + 1), SYS1_INFRAC);
	udelay(1);
}

static void __ltq_falcon_register_system_clks(void)
{
	if (sysctl_r32(SYSCTL_SYS1, SYS1_CPU0CC) & CPU0CC_CPUDIV)
		sysclk_data.clks[SYSCTL_CLK_CPU] = clk_register_fixed_rate(
			NULL, "cpu", NULL, CLK_IS_ROOT, CLOCK_200M);
	else
		sysclk_data.clks[SYSCTL_CLK_CPU] = clk_register_fixed_rate(
			NULL, "cpu", NULL, CLK_IS_ROOT, CLOCK_400M);

	sysclk_data.clks[SYSCTL_CLK_FPI] = clk_register_fixed_rate(
		NULL, "fpi", NULL, CLK_IS_ROOT, CLOCK_100M);
	sysclk_data.clks[SYSCTL_CLK_IO] = clk_register_fixed_rate(
		NULL, "io", NULL, CLK_IS_ROOT, CLOCK_200M);

	/* Add lookups for the system clocks. */
	ltq_register_cpu_clkdev(sysclk_data.clks[SYSCTL_CLK_CPU]);
	ltq_register_fpi_clkdev(sysclk_data.clks[SYSCTL_CLK_FPI]);
	ltq_register_io_clkdev(sysclk_data.clks[SYSCTL_CLK_IO]);
}

static inline void sysctl_wait(struct falcon_clk_gate *gate, u32 reg,
			       unsigned int test)
{
	int err = 1000000;
	u8 bits = (1 << gate->bit_idx);
	u32 val;

	do {
		val = __raw_readl(gate->reg_base + reg) & bits;
	} while (--err && (val != test));

	if (!err)
		pr_err("%s: module de/activation failed %08x %08X %08X %08X\n",
			__func__, reg, bits, test, val);
}

static inline void ltq_falcon_sysctl_set_bits(struct falcon_clk_gate *gate,
					      u32 reg_offset)
{
	__raw_writel((1 << gate->bit_idx), gate->reg_base + reg_offset);
}

static int falcon_sys_clk_gate_enable(struct clk_hw *hw)
{
	struct falcon_clk_gate *gate = to_falcon_clk_gate(hw);

	ltq_falcon_sysctl_set_bits(gate, SYSCTL_CLKEN);
	ltq_falcon_sysctl_set_bits(gate, SYSCTL_ACT);
	sysctl_wait(gate->reg_base, SYSCTL_CLKS, (1 << gate->bit_idx));

	return 0;
}

static void falcon_sys_clk_gate_disable(struct clk_hw *hw)
{
	struct falcon_clk_gate *gate = to_falcon_clk_gate(hw);

	ltq_falcon_sysctl_set_bits(gate, SYSCTL_CLKCLR);
	sysctl_wait(gate, SYSCTL_CLKS, 0);
}

const struct clk_ops falcon_sys_clk_gate_ops = {
	.enable = falcon_sys_clk_gate_enable,
	.disable = falcon_sys_clk_gate_disable,
};

struct falcon_sysclk_gate {
	u8	id;
	char	*devid;
	u8	module;
	u8	bit_idx;
} ltq_falcon_sysctl_gates[] __initdata = {
	{SYSCTL_GATE_GPIO0, "1d810000.gpio", SYSCTL_SYSETH, ACTS_P0},
	{SYSCTL_GATE_GPIO2, "1d810100.gpio", SYSCTL_SYSETH, ACTS_P2},
	{SYSCTL_GATE_GPIO1, "1e800100.gpio", SYSCTL_SYS1, ACTS_P1},
	{SYSCTL_GATE_GPIO3, "1e800200.gpio", SYSCTL_SYS1, ACTS_P3},
	{SYSCTL_GATE_GPIO4, "1e800300.gpio", SYSCTL_SYS1, ACTS_P4},
	{SYSCTL_GATE_PADCTRL0, "1db01000.pad", SYSCTL_SYSETH, ACTS_PADCTRL0},
	{SYSCTL_GATE_PADCTRL2, "1db02000.pad", SYSCTL_SYSETH, ACTS_PADCTRL2},
	{SYSCTL_GATE_PADCTRL1, "1e800400.pad", SYSCTL_SYS1, ACTS_PADCTRL1},
	{SYSCTL_GATE_PADCTRL3, "1e800500.pad", SYSCTL_SYS1, ACTS_PADCTRL3},
	{SYSCTL_GATE_PADCTRL4, "1e800600.pad", SYSCTL_SYS1, ACTS_PADCTRL4},
	{SYSCTL_GATE_SERIAL1, "1e100b00.serial", SYSCTL_SYS1, ACTS_ASC1_ACT},
	{SYSCTL_GATE_SERIAL0, "1e100c00.serial", SYSCTL_SYS1, ACTS_ASC0_ACT},
	{SYSCTL_GATE_SPI, "1e100d00.spi", SYSCTL_SYS1, ACTS_SSC0},
	{SYSCTL_GATE_I2C, "1e200000.i2c", SYSCTL_SYS1, ACTS_I2C_ACT},
};

static struct clk *clkdev_add_sys(struct falcon_sysclk_gate *gate)
{
	struct falcon_clk_gate *falcon_clk_gate;
	struct clk *clk;
	struct clk_init_data init;

	falcon_clk_gate = kzalloc(sizeof(*falcon_clk_gate), GFP_KERNEL);
	if (!falcon_clk_gate)
		return ERR_PTR(-ENOMEM);

	init.name = gate->devid;
	init.ops = &falcon_sys_clk_gate_ops;
	init.flags = CLK_IS_ROOT;
	init.parent_names = NULL;
	init.num_parents = 0;

	falcon_clk_gate->reg_base = sysctl_membase[gate->module];
	falcon_clk_gate->bit_idx = gate->bit_idx;
	falcon_clk_gate->hw.init = &init;

	clk = clk_register(NULL, &falcon_clk_gate->hw);
	if (IS_ERR(clk)) {
		pr_err("%s: Failed to register gate %s\n",
		       __func__, gate->devid);
		kfree(falcon_clk_gate);
		return clk;
	}

	if (clk_register_clkdev(clk, NULL, gate->devid))
		pr_err("%s: Failed to register lookup for %s\n",
		       __func__, gate->devid);

	return clk;
}

static void __init ltq_falcon_sys_init_dt(struct device_node *np_sys1)
{
	struct device_node *np_status =
		of_find_compatible_node(NULL, NULL, "lantiq,status-falcon");
	struct device_node *np_syseth =
		of_find_compatible_node(NULL, NULL, "lantiq,syseth-falcon");
	struct device_node *np_sysgpe =
		of_find_compatible_node(NULL, NULL, "lantiq,sysgpe-falcon");
	struct resource res_status, res_sys[3];
	int i;

	/* check if all the core register ranges are available */
	if (!np_status || !np_sys1 || !np_syseth || !np_sysgpe)
		panic("%s: Failed to load core DT nodes", __func__);

	if (of_address_to_resource(np_status, 0, &res_status) ||
			of_address_to_resource(np_sys1, 0, &res_sys[0]) ||
			of_address_to_resource(np_syseth, 0, &res_sys[1]) ||
			of_address_to_resource(np_sysgpe, 0, &res_sys[2]))
		panic("%s: Failed to get core resources", __func__);

	if ((request_mem_region(res_status.start, resource_size(&res_status),
				res_status.name) < 0) ||
		(request_mem_region(res_sys[0].start,
				resource_size(&res_sys[0]),
				res_sys[0].name) < 0) ||
		(request_mem_region(res_sys[1].start,
				resource_size(&res_sys[1]),
				res_sys[1].name) < 0) ||
		(request_mem_region(res_sys[2].start,
				resource_size(&res_sys[2]),
				res_sys[2].name) < 0))
		pr_err("%s: Failed to request core resources", __func__);

	status_membase = ioremap_nocache(res_status.start,
					resource_size(&res_status));

	if (!status_membase)
		panic("%s: Failed to remap core resources", __func__);

	for (i = 0; i < 3; i++) {
		sysctl_membase[i] = ioremap_nocache(res_sys[i].start,
						resource_size(&res_sys[i]));
		if (!sysctl_membase[i])
			panic("Failed to remap sysctrl resources");
	}
	ltq_sys1_membase = sysctl_membase[0];

	falcon_gpe_enable();

	sysclk_data.clk_num = NUM_SYSCTL_CLOCKS;
	sysclk_data.clks = kcalloc(sysclk_data.clk_num, sizeof(struct clk *),
				   GFP_KERNEL);
	if (!sysclk_data.clks) {
		panic("%s: Failed to allocate SYSCTL gate memory\n", __func__);
		return;
	}

	__ltq_falcon_register_system_clks();

	/* add our clock domains */
	for (i = 0; i < ARRAY_SIZE(ltq_falcon_sysctl_gates); i++) {
		struct falcon_sysclk_gate *gate = &ltq_falcon_sysctl_gates[i];

		sysclk_data.clks[gate->id] = clkdev_add_sys(gate);
	}

	of_clk_add_provider(np_sys1, of_clk_src_onecell_get, &sysclk_data);
}

CLK_OF_DECLARE(sysclk_falcon, "lantiq,sys1-falcon", ltq_falcon_sys_init_dt);
