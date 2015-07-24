#ifndef __DRV_CLK_LANTIQ_XWAY_H
#define __DRV_CLK_LANTIQ_XWAY_H

#include "clk.h"

#define NUM_GPIO_CLKS		4

/* CGU clock registers */
#define CGU_SYS			0x10
#define CGU_IFCCR		0x0018
#define CGU_PCICR		0x34

/* xRX CGU clock registers (vrx200 and newer) */
#define CGU_SYS_XRX		0x0c
#define CGU_IFCCR_XRX		0x24
#define CGU_PCICR_XRX		0x38

struct device_node;

/*
 * The clk_ops (using one register to enable/disable functions) for the PMU
 * gates on Amazon-SE to VRX200.
 */
extern const struct clk_ops xway_pmu_clk_gate_ops;

/*
 * The clk_ops for ARX300 and newer SoCs (using separate registers to
 * enable/disable functions) for the PMU gates.
 */
extern const struct clk_ops xrx300_pmu_clk_gate_ops;

/* Describes a PMU gate for the clk framework. */
struct ltq_pmu_clk_gate {
	struct clk_hw	hw;
	u8		function;
	void __iomem	*reg_base;
};

struct ltq_xway_pmu_gate {
	u8		id;
	const char	*name;
	u8		function;
	const char	*parent_name;
	unsigned long	flags;
	const char	*dev_id;
	const char	*con_id;
};

#define PMU_GATE(_gate_id, _name, _func, _parent, _flags, _dev, _con)	\
	{								\
		.id		= _gate_id,				\
		.name		= "pmu_"_name,				\
		.function	= _func,				\
		.parent_name	= _parent,				\
		.flags		= _flags,				\
		.dev_id		= _dev,					\
		.con_id		= _con,					\
	}

/* Describes a clock provided by the CGU module for the clk framework. */
struct ltq_cgu_clk {
	struct clk_hw	hw;
	void __iomem	*reg_base;
};

struct ltq_cgu_clocks {
	struct clk	*cpu_clk;
	struct clk	*fpi_clk;
	struct clk	*io_clk;
	struct clk	*pp32_clk;
	struct clk	*pci_clk;
	struct clk	*pci_ext_clk;
	struct clk	*ephy_clk;
};

static inline struct ltq_pmu_clk_gate *to_ltq_pmu_clk_gate(struct clk_hw *hw)
{
	return container_of(hw, struct ltq_pmu_clk_gate, hw);
}

static inline struct ltq_cgu_clk *to_ltq_cgu_clk(struct clk_hw *hw)
{
	return container_of(hw, struct ltq_cgu_clk, hw);
}

static inline u32 ltq_cgu_clk_read(struct ltq_cgu_clk *cgu_clk, u32 reg)
{
	return __raw_readl(cgu_clk->reg_base + reg);
}

/**
 * Registers a list of PMU gates. The resulting clks will be registered with
 * the given DT node.
 *
 * @np The PMU device node pointer
 * @gate_cfgs An array of ltq_xway_pmu_gate which describe the PMU gates
 * @num_gates The number of elements in the gate_cfgs array
 * @gate_ops The clk_ops that will be used to control the PMU bits
 */
void ltq_xway_pmu_register_gates(struct device_node *np,
				 struct ltq_xway_pmu_gate *gate_cfgs,
				 u32 num_gates,
				 const struct clk_ops *gate_ops);

void ltq_xway_cgu_init_dt(struct device_node *np);

struct clk *ltq_xway_register_cgu_pci_clk(u32 ifccr_reg_offset,
					  u32 pcicr_reg_offset);
struct clk *ltq_xway_register_cgu_pci_ext_clk(u32 ifccr_reg_offset,
					      u32 pcicr_reg_offset);

struct clk *ltq_cgu_register_clk(const char *name, const char *parent_name,
				 const struct clk_ops *ops);

void ltq_cgu_clk_of_add_provider(struct device_node *np,
				 struct ltq_cgu_clocks *cgu_clocks);

/**
 * Select the GPHY clock input by evaluating the "lantiq,gphy-clk-select" DT
 * property. This is only valid for xRX200 and newer SoCs.
 *
 * @np The CGU device node pointer
 */
int ltq_xrx_cgu_select_gphy_clk_input(struct device_node *np);

#endif /* __DRV_CLK_LANTIQ_XWAY_H */
