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
#include <lantiq_soc.h>

#include "clk.h"

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

#define status_r32(x)		__raw_readl(status_membase + (x))

static void __iomem *syseth_membase, *status_membase;

static struct clk_onecell_data sys1_data, syseth_data;

struct ltq_falcon_sys_gate {
	u8		id;
	u8		bit_idx;
	const char	*name;
	const char	*parent_name;
	const char	*devid;
};

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
	status = ltq_sys1_r32(SYS1_INFRAC);
	if (status & (1 << (GPPC_OFFSET + 1)))
		return;

	freq = (status_r32(STATUS_CONFIG) &
		GPEFREQ_MASK) >>
		GPEFREQ_OFFSET;
	if (freq == 0)
		freq = 1; /* use 625MHz on unfused chip */

	/* apply new frequency */
	ltq_sys1_w32_mask(7 << (GPPC_OFFSET + 1),
			freq << (GPPC_OFFSET + 2), SYS1_INFRAC);
	udelay(1);

	/* enable new frequency */
	ltq_sys1_w32_mask(0, 1 << (GPPC_OFFSET + 1), SYS1_INFRAC);
	udelay(1);
}

static void falcon_register_system_clks(void)
{
	unsigned long cpu_rate;
	struct clk *cpu_clk, *fpi_clk, *io_clk;

	if (ltq_sys1_r32(SYS1_CPU0CC) & CPU0CC_CPUDIV)
		cpu_rate = CLOCK_200M;
	else
		cpu_rate = CLOCK_400M;

	cpu_clk = clk_register_fixed_rate(NULL, "cpu", NULL, CLK_IS_ROOT,
						cpu_rate);
	fpi_clk = clk_register_fixed_rate(NULL, "fpi", NULL, CLK_IS_ROOT,
						CLOCK_100M);
	io_clk = clk_register_fixed_rate(NULL, "io", NULL, CLK_IS_ROOT,
						CLOCK_200M);

	/* Add lookups for the system clocks. */
	ltq_register_cpu_clkdev(cpu_clk);
	ltq_register_fpi_clkdev(fpi_clk);
	ltq_register_io_clkdev(io_clk);
}

static inline void ltq_falcon_sysctl_set_bits(struct falcon_clk_gate *gate,
					      u32 reg_offset)
{
	__raw_writel((1 << gate->bit_idx), gate->reg_base + reg_offset);
}

static int falcon_sys_clk_gate_is_enabled(struct clk_hw *hw)
{
	struct falcon_clk_gate *gate = to_falcon_clk_gate(hw);
	u32 bits = (1 << gate->bit_idx);

	return __raw_readl(gate->reg_base + SYSCTL_CLKS) & bits;
}

static int falcon_sys_clk_gate_enable(struct clk_hw *hw)
{
	struct falcon_clk_gate *gate = to_falcon_clk_gate(hw);
	int retry = 1000000;

	ltq_falcon_sysctl_set_bits(gate, SYSCTL_CLKEN);
	ltq_falcon_sysctl_set_bits(gate, SYSCTL_ACT);

	do {} while (--retry && !falcon_sys_clk_gate_is_enabled(hw));
	if (!retry) {
		pr_err("%s: Failed to enable SYS gate %d\n", __func__,
			gate->bit_idx);
		return -EIO;
	}

	return 0;
}

static void falcon_sys_clk_gate_disable(struct clk_hw *hw)
{
	struct falcon_clk_gate *gate = to_falcon_clk_gate(hw);
	int retry = 1000000;

	ltq_falcon_sysctl_set_bits(gate, SYSCTL_CLKCLR);

	do {} while (--retry && falcon_sys_clk_gate_is_enabled(hw));
	if (!retry)
		pr_err("%s: Failed to disable SYS gate %d\n", __func__,
			gate->bit_idx);
}

const struct clk_ops falcon_sys_clk_gate_ops = {
	.enable = falcon_sys_clk_gate_enable,
	.disable = falcon_sys_clk_gate_disable,
	.is_enabled = falcon_sys_clk_gate_is_enabled,
};

static struct ltq_falcon_sys_gate falcon_sys1_gates[] __initdata = {
	{SYS1_GATE_SERIAL1, 11, "serial1", "fpi", "1e100b00.serial"},
	{SYS1_GATE_SERIAL0, 12, "serial0", "fpi", "1e100c00.serial"},
	{SYS1_GATE_SPI, 13, "spi", "fpi", "1e100d00.spi"},
	{SYS1_GATE_I2C, 14, "i2c", "fpi", "1e200000.i2c"},
	{SYS1_GATE_GPTC, 15, "gptc", "fpi", "1e100e00.gptc"},
	{SYS1_GATE_GPIO1, 16, "gpio1", NULL, "1e800100.gpio"},
	{SYS1_GATE_GPIO3, 17, "gpio3", NULL, "1e800200.gpio"},
	{SYS1_GATE_GPIO4, 18, "gpio4", NULL, "1e800300.gpio"},
	{SYS1_GATE_PADCTRL1, 20, "pad1", NULL, "1e800400.pad"},
	{SYS1_GATE_PADCTRL3, 21, "pad3", NULL, "1e800500.pad"},
	{SYS1_GATE_PADCTRL4, 22, "pad4", NULL, "1e800600.pad"},
};

static struct ltq_falcon_sys_gate falcon_syseth_gates[] __initdata = {
	{SYSETH_GATE_GPIO0, 16, "gpio0", NULL, "1d810000.gpio"},
	{SYSETH_GATE_GPIO2, 17, "gpio2", NULL, "1d810100.gpio"},
	{SYSETH_GATE_PADCTRL0, 20, "pad0", NULL, "1db01000.pad"},
	{SYSETH_GATE_PADCTRL2, 21, "pad2", NULL, "1db02000.pad"},
};

static struct clk *falcon_register_sys_gate(struct ltq_falcon_sys_gate *gate,
					    void __iomem *reg_base)
{
	struct falcon_clk_gate *falcon_clk_gate;
	struct clk *clk;
	struct clk_init_data init;

	falcon_clk_gate = kzalloc(sizeof(*falcon_clk_gate), GFP_KERNEL);
	if (!falcon_clk_gate)
		return ERR_PTR(-ENOMEM);

	init.name = gate->devid;
	init.ops = &falcon_sys_clk_gate_ops;

	if (gate->parent_name) {
		init.flags = 0;
		init.parent_names = &gate->parent_name;
		init.num_parents = 1;
	} else {
		init.flags = CLK_IS_ROOT;
		init.parent_names = NULL;
		init.num_parents = 0;
	}

	falcon_clk_gate->reg_base = reg_base;
	falcon_clk_gate->bit_idx = gate->bit_idx;
	falcon_clk_gate->hw.init = &init;

	clk = clk_register(NULL, &falcon_clk_gate->hw);
	if (IS_ERR(clk)) {
		pr_err("%s: Failed to register gate %s\n",
		       __func__, gate->devid);
		kfree(falcon_clk_gate);
		return clk;
	}

	if (clk_register_clkdev(clk, gate->name, NULL))
		pr_err("%s: Failed to register lookup for %s\n",
			__func__, gate->name);

	if (clk_register_clkdev(clk, NULL, gate->devid))
		pr_err("%s: Failed to register devid lookup for %s\n",
		       __func__, gate->devid);

	return clk;
}

static void __init falcon_sys_gates_init(struct device_node *np,
					 struct clk_onecell_data *clk_data,
					 void __iomem *reg_base,
					 struct ltq_falcon_sys_gate *gates,
					 unsigned int nr_gates)
{
	struct clk **clk_table;
	int i;

	clk_table = kcalloc(nr_gates, sizeof(struct clk *), GFP_KERNEL);
	if (!clk_table) {
		panic("%s: Failed to allocate PMU gate memory\n", __func__);
		return;
	}

	for (i = 0; i < nr_gates; ++i)
		clk_table[i] = falcon_register_sys_gate(&gates[i], reg_base);

	clk_data->clk_num = nr_gates;
	clk_data->clks = clk_table;
	of_clk_add_provider(np, of_clk_src_onecell_get, clk_data);
}

static void __init falcon_sys1_clk_init_dt(struct device_node *np)
{
	struct resource res_sys1;

	if (of_address_to_resource(np, 0, &res_sys1))
		panic("%s: could not determine the sys1 base address\n",
			__func__);

	ltq_sys1_membase = ioremap_nocache(res_sys1.start,
					resource_size(&res_sys1));
	if (!ltq_sys1_membase)
		panic("%s: Failed to remap the sys1 resource\n", __func__);

	falcon_register_system_clks();

	falcon_sys_gates_init(np, &sys1_data, ltq_sys1_membase,
				falcon_sys1_gates,
				ARRAY_SIZE(falcon_sys1_gates));
}
CLK_OF_DECLARE(falcon_sys1_clk, "lantiq,sys1-falcon", falcon_sys1_clk_init_dt);

static void __init falcon_syseth_clk_init_dt(struct device_node *np)
{
	struct resource res_syseth;

	if (of_address_to_resource(np, 0, &res_syseth))
		panic("%s: could not determine the syseth base address\n",
			__func__);

	syseth_membase = ioremap_nocache(res_syseth.start,
					resource_size(&res_syseth));
	if (!syseth_membase)
		panic("%s: Failed to remap the syseth resource\n", __func__);

	falcon_sys_gates_init(np, &syseth_data, syseth_membase,
				falcon_syseth_gates,
				ARRAY_SIZE(falcon_syseth_gates));
}
CLK_OF_DECLARE(falcon_syseth_clk, "lantiq,syseth-falcon", falcon_syseth_clk_init_dt);

static int __init falcon_sysstatus_init(void)
{
	struct resource res_sysstatus;
	struct device_node *np_status =
		of_find_compatible_node(NULL, NULL, "lantiq,status-falcon");

	if (!np_status)
		panic("%s: Failed to load status DT node", __func__);

	if (of_address_to_resource(np_status, 0, &res_sysstatus))
		panic("%s: could not determine the status base address\n",
			__func__);

	status_membase = ioremap_nocache(res_sysstatus.start,
					resource_size(&res_sysstatus));
	if (!status_membase)
		panic("%s: Failed to remap the status resource\n", __func__);

	falcon_gpe_enable();

	return 0;
}

subsys_initcall(falcon_sysstatus_init);
