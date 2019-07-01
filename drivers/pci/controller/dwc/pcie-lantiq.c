// SPDX-License-Identifier: GPL-2.0-only
/*
 * PCIe RC driver for Lantiq SoCs, based on the Synopsys DesignWare Core.
 *
 * Copyright (C) 2019 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 *
 * Based on the BSP (called "UGW") driver:
 * Copyright (C) 2009 Lei Chuanhua <chuanhua.lei@infineon.com>
 */
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/pci.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/resource.h>
#include <linux/types.h>

#include "pcie-designware.h"

/* RC Core Control Register */
#define PCIE_APP_RC_CCR						0x10
#define PCIE_APP_RC_CCR_LTSSM_ENABLE				BIT(0)

/* RC Core Debug Register */
#define PCIE_APP_RC_DR						0x14
#define PCIE_APP_RC_DR_DLL_UP					BIT(0)

/* PHY Link Status Register */
#define PCIE_APP_PHY_SR						0x18
#define PCIE_APP_PHY_SR_PHY_LINK_UP				BIT(0)

/* Electromechanical Control Register */
#define PCIE_APP_EM_CR						0x1c

/* Interrupt Status Register */
#define PCIE_APP_IR_SR						0x20

/* Message Control Register */
#define PCIE_APP_MSG_CR						0x30

#define PCIE_APP_VDM_DR						0x34

/* Vendor-Defined Message Requester ID Register */
#define PCIE_APP_VDM_RID					0x38

/* ASPM Control Register */
#define PCIE_APP_ASPM_CR					0x40

/* Vendor Message DW0 Register */
#define PCIE_APP_VM_MSG_DW0					0x50

/* Vendor Message DW1 Register */
#define PCIE_APP_VM_MSG_DW1					0x54

#define PCIE_APP_VM_MSG_DW2					0x58
#define PCIE_APP_VM_MSG_DW3					0x5c

/* Vendor Message Request Register */
#define PCIE_APP_VM_MSG_REQR					0x60

/* AHB Slave Side Band Control Register */
#define PCIE_APP_AHB_SSB					0x70

/* AHB Master SideBand Ctrl Register */
#define PCIE_APP_AHB_MSB					0x74

/* AHB Control Register, fixed bus enumeration exception */
#define PCIE_APP_AHB_CTRL					0x78
#define PCIE_APP_AHB_CTRL_BUS_ERROR_SUPPRESS			BIT(0)

/* Interrupt Enable Register */
#define PCIE_APP_IRNEN						0xf4
#define PCIE_APP_IRNEN_AER_REPORT				BIT(0)
#define PCIE_APP_IRNEN_AER_MSIX					BIT(1)
#define PCIE_APP_IRNEN_PME					BIT(2)
#define PCIE_APP_IRNEN_HOTPLUG					BIT(3)
#define PCIE_APP_IRNEN_RX_VDM_MSG				BIT(4)
#define PCIE_APP_IRNEN_RX_CORRECTABLE_ERR_MSG			BIT(5)
#define PCIE_APP_IRNEN_RX_NON_FATAL_ERR_MSG			BIT(6)
#define PCIE_APP_IRNEN_RX_FATAL_ERR_MSG				BIT(7)
#define PCIE_APP_IRNEN_RX_PME_MSG				BIT(8)
#define PCIE_APP_IRNEN_RX_PME_TURNOFF_ACK			BIT(9)
#define PCIE_APP_IRNEN_AHB_BR_FATAL_ERR				BIT(10)
#define PCIE_APP_IRNEN_LINK_AUTO_BW_STATUS			BIT(11)
#define PCIE_APP_IRNEN_BW_MGT					BIT(12)
#define PCIE_APP_IRNEN_INTA					BIT(13)
#define PCIE_APP_IRNEN_INTB					BIT(14)
#define PCIE_APP_IRNEN_INTC					BIT(15)
#define PCIE_APP_IRNEN_INTD					BIT(16)
#define PCIE_APP_IRNEN_WAKEUP					BIT(17)

/* Interrupt Clear Register */
#define PCIE_APP_IRNCR						0xf8
#define PCIE_APP_IRNCR_AER_REPORT				BIT(0)
#define PCIE_APP_IRNCR_AER_MSIX					BIT(1)
#define PCIE_APP_IRNCR_PME					BIT(2)
#define PCIE_APP_IRNCR_HOTPLUG					BIT(3)
#define PCIE_APP_IRNCR_RX_VDM_MSG				BIT(4)
#define PCIE_APP_IRNCR_RX_CORRECTABLE_ERR_MSG			BIT(5)
#define PCIE_APP_IRNCR_RX_NON_FATAL_ERR_MSG			BIT(6)
#define PCIE_APP_IRNCR_RX_FATAL_ERR_MSG				BIT(7)
#define PCIE_APP_IRNCR_RX_PME_MSG				BIT(8)
#define PCIE_APP_IRNCR_RX_PME_TURNOFF_ACK			BIT(9)
#define PCIE_APP_IRNCR_AHB_BR_FATAL_ERR				BIT(10)
#define PCIE_APP_IRNCR_LINK_AUTO_BW_STATUS			BIT(11)
#define PCIE_APP_IRNCR_BW_MGT					BIT(12)
#define PCIE_APP_IRNCR_INTA					BIT(13)
#define PCIE_APP_IRNCR_INTB					BIT(14)
#define PCIE_APP_IRNCR_INTC					BIT(15)
#define PCIE_APP_IRNCR_INTD					BIT(16)
#define PCIE_APP_IRNCR_WAKEUP					BIT(17)

#define PCIE_APP_IRNICR						0xfc

#define PCIE_APP_INTC_NUM_IRQS				(PCI_NUM_INTX + 1)

struct lantiq_pcie {
	struct dw_pcie			pci;
	struct regmap			*app_regmap;
	struct regmap			*rcu_regmap;
	struct clk			*ahb_clk;
	struct clk			*bus_clk;
	struct clk			*ctl_clk;
	struct reset_control		*pcie_reset;
	struct phy			*phy;
	struct gpio_desc		*reset_gpio;
	unsigned int			mapped_irqs[PCIE_APP_INTC_NUM_IRQS];
	u32				rcu_ahb_endian_offset;
	u32				rcu_ahb_endian_big_endian_pciem_mask;
	u32				rcu_ahb_endian_big_endian_xbarm_mask;
	u32				rcu_ahb_endian_big_endian_pcies_mask;
	bool				big_endian;
};

#define to_lantiq_pcie(x)		dev_get_drvdata((x)->dev)

static const struct regmap_range lantiq_dw_pcie_app_regmap_rd_ranges[] = {
	regmap_reg_range(PCIE_APP_RC_CCR, PCIE_APP_RC_CCR),
	regmap_reg_range(PCIE_APP_RC_DR, PCIE_APP_RC_DR),
	regmap_reg_range(PCIE_APP_PHY_SR, PCIE_APP_PHY_SR),
	regmap_reg_range(PCIE_APP_EM_CR, PCIE_APP_EM_CR),
	regmap_reg_range(PCIE_APP_IR_SR, PCIE_APP_IR_SR),
	regmap_reg_range(PCIE_APP_MSG_CR, PCIE_APP_MSG_CR),
	regmap_reg_range(PCIE_APP_VDM_DR, PCIE_APP_VDM_DR),
	regmap_reg_range(PCIE_APP_VDM_RID, PCIE_APP_VDM_RID),
	regmap_reg_range(PCIE_APP_ASPM_CR, PCIE_APP_ASPM_CR),
	regmap_reg_range(PCIE_APP_VM_MSG_DW0, PCIE_APP_VM_MSG_DW0),
	regmap_reg_range(PCIE_APP_VM_MSG_DW1, PCIE_APP_VM_MSG_DW1),
	regmap_reg_range(PCIE_APP_VM_MSG_DW2, PCIE_APP_VM_MSG_DW2),
	regmap_reg_range(PCIE_APP_VM_MSG_DW3, PCIE_APP_VM_MSG_DW3),
	regmap_reg_range(PCIE_APP_VM_MSG_REQR, PCIE_APP_VM_MSG_REQR),
	regmap_reg_range(PCIE_APP_AHB_SSB, PCIE_APP_AHB_SSB),
	regmap_reg_range(PCIE_APP_AHB_MSB, PCIE_APP_AHB_MSB),
	regmap_reg_range(PCIE_APP_AHB_CTRL, PCIE_APP_AHB_CTRL),
	regmap_reg_range(PCIE_APP_IRNEN, PCIE_APP_IRNEN),
	regmap_reg_range(PCIE_APP_IRNCR, PCIE_APP_IRNCR),
	regmap_reg_range(PCIE_APP_IRNICR, PCIE_APP_IRNICR),
};

static const struct regmap_access_table lantiq_dw_pcie_app_regmap_rd_table = {
	.yes_ranges = lantiq_dw_pcie_app_regmap_rd_ranges,
	.n_yes_ranges = ARRAY_SIZE(lantiq_dw_pcie_app_regmap_rd_ranges),
};

static const struct regmap_config lantiq_dw_pcie_app_regmap_config = {
	.name = "app",
	.reg_bits = 8,
	.val_bits = 32,
	.reg_stride = 4,
	.fast_io = true,
	.max_register = PCIE_APP_IRNICR,
	.rd_table = &lantiq_dw_pcie_app_regmap_rd_table,
};

static void lantiq_pcie_app_irq_clear(struct dw_pcie *pci)
{
	struct lantiq_pcie *ltq_pcie = to_lantiq_pcie(pci);
	u32 mask = PCIE_APP_IRNCR_AER_REPORT |
		   PCIE_APP_IRNCR_AER_MSIX |
		   PCIE_APP_IRNCR_PME |
		   PCIE_APP_IRNCR_HOTPLUG |
		   PCIE_APP_IRNCR_RX_VDM_MSG |
		   PCIE_APP_IRNCR_RX_CORRECTABLE_ERR_MSG |
		   PCIE_APP_IRNCR_RX_NON_FATAL_ERR_MSG |
		   PCIE_APP_IRNCR_RX_FATAL_ERR_MSG |
		   PCIE_APP_IRNCR_RX_PME_MSG |
		   PCIE_APP_IRNCR_RX_PME_TURNOFF_ACK |
		   PCIE_APP_IRNCR_AHB_BR_FATAL_ERR |
		   PCIE_APP_IRNCR_LINK_AUTO_BW_STATUS |
		   PCIE_APP_IRNCR_BW_MGT;

	regmap_update_bits(ltq_pcie->app_regmap, PCIE_APP_IRNCR, mask, mask);
}

static void lantiq_pcie_app_irq_enable(struct dw_pcie *pci)
{
	struct lantiq_pcie *ltq_pcie = to_lantiq_pcie(pci);
	u32 mask = PCIE_APP_IRNEN_AER_REPORT |
		   PCIE_APP_IRNEN_AER_MSIX |
		   PCIE_APP_IRNEN_PME |
		   PCIE_APP_IRNEN_HOTPLUG |
		   PCIE_APP_IRNEN_RX_VDM_MSG |
		   PCIE_APP_IRNEN_RX_CORRECTABLE_ERR_MSG |
		   PCIE_APP_IRNEN_RX_NON_FATAL_ERR_MSG |
		   PCIE_APP_IRNEN_RX_FATAL_ERR_MSG |
		   PCIE_APP_IRNEN_RX_PME_MSG |
		   PCIE_APP_IRNEN_RX_PME_TURNOFF_ACK |
		   PCIE_APP_IRNEN_AHB_BR_FATAL_ERR |
		   PCIE_APP_IRNEN_LINK_AUTO_BW_STATUS |
		   PCIE_APP_IRNEN_BW_MGT;

	regmap_update_bits(ltq_pcie->app_regmap, PCIE_APP_IRNEN, mask, mask);

	lantiq_pcie_app_irq_clear(pci);
}

static irqreturn_t lantiq_pcie_app_irq_handler(int irq, void *dev_id)
{
	struct dw_pcie *pci = dev_id;

	lantiq_pcie_app_irq_clear(pci);

	return IRQ_HANDLED;
}

static void lantiq_pcie_rcu_init(struct dw_pcie *pci)
{
	struct lantiq_pcie *ltq_pcie = to_lantiq_pcie(pci);
	u32 pcies_mask;

	regmap_update_bits(ltq_pcie->rcu_regmap,
			   ltq_pcie->rcu_ahb_endian_offset,
			   ltq_pcie->rcu_ahb_endian_big_endian_pciem_mask,
			   ltq_pcie->rcu_ahb_endian_big_endian_pciem_mask);

	if (ltq_pcie->big_endian)
		pcies_mask = ltq_pcie->rcu_ahb_endian_big_endian_pcies_mask;
	else
		pcies_mask = 0x0;

	regmap_update_bits(ltq_pcie->rcu_regmap,
			   ltq_pcie->rcu_ahb_endian_offset,
			   ltq_pcie->rcu_ahb_endian_big_endian_pcies_mask,
			   pcies_mask);

	regmap_update_bits(ltq_pcie->rcu_regmap,
			   ltq_pcie->rcu_ahb_endian_offset,
			   ltq_pcie->rcu_ahb_endian_big_endian_xbarm_mask,
			   0x0);
}

static int lantiq_pcie_hw_init(struct dw_pcie *pci)
{
	struct lantiq_pcie *ltq_pcie = to_lantiq_pcie(pci);
	int ret;

	ret = clk_prepare_enable(ltq_pcie->ahb_clk);
	if (ret)
		goto err;

	ret = phy_init(ltq_pcie->phy);
	if (ret)
		goto err_disable_ahb_clk;

	gpiod_set_value_cansleep(ltq_pcie->reset_gpio, 1);

	udelay(1);

	ret = reset_control_deassert(ltq_pcie->pcie_reset);
	if (ret)
		goto err_phy_exit;

	ret = phy_power_on(ltq_pcie->phy);
	if (ret)
		goto err_assert_pcie_reset;

	ret = clk_prepare_enable(ltq_pcie->ctl_clk);
	if (ret)
		goto err_phy_power_off;

	ret = clk_prepare_enable(ltq_pcie->bus_clk);
	if (ret)
		goto err_disable_ctl_clk;

	return 0;

err_disable_ctl_clk:
	clk_disable_unprepare(ltq_pcie->ctl_clk);
err_phy_power_off:
	phy_power_off(ltq_pcie->phy);
err_assert_pcie_reset:
	reset_control_assert(ltq_pcie->pcie_reset);
err_phy_exit:
	phy_exit(ltq_pcie->phy);
err_disable_ahb_clk:
	clk_disable_unprepare(ltq_pcie->ahb_clk);
err:
	return ret;
}

static void lantiq_pcie_hw_exit(struct dw_pcie *pci)
{
	struct lantiq_pcie *ltq_pcie = to_lantiq_pcie(pci);

	gpiod_set_value_cansleep(ltq_pcie->reset_gpio, 1);
	clk_disable_unprepare(ltq_pcie->bus_clk);
	clk_disable_unprepare(ltq_pcie->ctl_clk);
	phy_power_off(ltq_pcie->phy);
	reset_control_assert(ltq_pcie->pcie_reset);
	phy_exit(ltq_pcie->phy);
}

static int lantiq_pcie_host_init(struct pcie_port *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct lantiq_pcie *ltq_pcie = to_lantiq_pcie(pci);
	int ret, i;

	lantiq_pcie_rcu_init(pci);

	/*
	 * XXX: PCIe elastic buffer bug will result in the PCIe link not being
	 * detected. One more reset of the PCIe PHY will solve this issue.
	 */
	for (i = 0; i < 5; i++) {
		ret = lantiq_pcie_hw_init(pci);
		if (ret)
			return ret;

		dw_pcie_setup_rc(pp);

		regmap_write(ltq_pcie->app_regmap, PCIE_APP_RC_DR, 0);

		/* Disable LTSSM */
		regmap_write(ltq_pcie->app_regmap, PCIE_APP_RC_CCR, 0);

		lantiq_pcie_app_irq_enable(pci);

		regmap_write(ltq_pcie->app_regmap, PCIE_APP_AHB_CTRL,
			     PCIE_APP_AHB_CTRL_BUS_ERROR_SUPPRESS);

		msleep(100);

		gpiod_set_value_cansleep(ltq_pcie->reset_gpio, 0);

		/* Start LTSSM training between RC and EP */
		regmap_write(ltq_pcie->app_regmap, PCIE_APP_RC_CCR,
			     PCIE_APP_RC_CCR_LTSSM_ENABLE);

		ret = dw_pcie_wait_for_link(pci);
		if (ret) {
			lantiq_pcie_hw_exit(pci);

			/* try again due to the issue described above */
			continue;
		}

		return 0;
	}

	return ret;
}

static int lantiq_pcie_msi_host_init(struct pcie_port *pp)
{
	/*
	 * MSI is available through a dedicated MSI PIC (Programmable
	 * Interrupt Controller) outside of the PCIe registers.
	 */
	return 0;
}

static void __iomem *lantiq_pcie_conf_addr(void __iomem *base, int where)
{
	return base + ALIGN_DOWN(where, 4);
}

static u32 lantiq_pcie_conf_mask(int size)
{
	return BIT_ULL_MASK(size * 8) - 1;
}

static u8 lantiq_pcie_conf_shift(int where)
{
	return (where - ALIGN_DOWN(where, 4)) * 8;
}

static int lantiq_pcie_rd_conf(void __iomem *base, int where, int size,
			       u32 *val, bool endian_swap)
{
	void __iomem *addr;
	int ret;

	/* only 4-byte reads are supported */
	addr = lantiq_pcie_conf_addr(base, where);

	ret = dw_pcie_read(addr, 4, val);
	if (ret != PCIBIOS_SUCCESSFUL)
		return ret;

	if (endian_swap)
		*val = __le32_to_cpu(*val);

	*val >>= lantiq_pcie_conf_shift(where);
	*val &= lantiq_pcie_conf_mask(size);

	return PCIBIOS_SUCCESSFUL;
}

static int lantiq_pcie_wr_conf(void __iomem *base, int where, int size,
			       u32 val, bool endian_swap)
{
	void __iomem *addr;
	u32 tmp;
	int ret;

	/* only 4-byte writes are supported */
	addr = lantiq_pcie_conf_addr(base, where);

	ret = dw_pcie_read(addr, 4, &tmp);
	if (ret != PCIBIOS_SUCCESSFUL)
		return ret;

	if (endian_swap)
		tmp = __le32_to_cpu(tmp);

	tmp &= ~(lantiq_pcie_conf_mask(size) << lantiq_pcie_conf_shift(where));
	tmp |= val << lantiq_pcie_conf_shift(where);

	if (endian_swap)
		tmp = __cpu_to_le32(tmp);

	return dw_pcie_write(addr, 4, tmp);
}

static int lantiq_pcie_rd_own_conf(struct pcie_port *pp, int where, int size,
				   u32 *val)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct lantiq_pcie *ltq_pcie = to_lantiq_pcie(pci);

	/* the own config space requires endianness swapping */
	return lantiq_pcie_rd_conf(pci->dbi_base, where, size, val,
				   ltq_pcie->big_endian);
}

static int lantiq_pcie_wr_own_conf(struct pcie_port *pp, int where,
				   int size, u32 val)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct lantiq_pcie *ltq_pcie = to_lantiq_pcie(pci);

	/* the own config space requires endianness swapping */
	return lantiq_pcie_wr_conf(pci->dbi_base, where, size, val,
				   ltq_pcie->big_endian);
}

static u32 lantiq_pcie_other_conf_bus_addr(struct pcie_port *pp,
					   struct pci_bus *bus,
					   unsigned int devfn, int where)
{
	u32 addr = where;

	addr |= (PCI_SLOT(devfn) & 0x1f) << 15;
	addr |= (PCI_FUNC(devfn) & 0x7) << 12;

	if (bus->parent->number != pp->root_bus_nr)
		/* type 1, only support 8 buses */
		addr |= (bus->number & 0x7) << 20;

	return addr;
}

static int lantiq_pcie_rd_other_conf(struct pcie_port *pp, struct pci_bus *bus,
				     unsigned int devfn, int where, int size,
				     u32 *val)
{
	u32 bus_addr = lantiq_pcie_other_conf_bus_addr(pp, bus, devfn, where);

	/* the other config space does not require endianness swapping */
	return lantiq_pcie_rd_conf(pp->va_cfg0_base, bus_addr, size, val,
				   false);
}

static int lantiq_pcie_wr_other_conf(struct pcie_port *pp, struct pci_bus *bus,
				     unsigned int devfn, int where, int size,
				     u32 val)
{
	u32 bus_addr = lantiq_pcie_other_conf_bus_addr(pp, bus, devfn, where);

	/* the other config space does not require endianness swapping */
	return lantiq_pcie_wr_conf(pp->va_cfg0_base, bus_addr, size, val,
				   false);
}

static const struct dw_pcie_host_ops lantiq_dw_pcie_host_ops = {
	.host_init = lantiq_pcie_host_init,
	.msi_host_init = lantiq_pcie_msi_host_init,
	.rd_other_conf = lantiq_pcie_rd_other_conf,
	.wr_other_conf = lantiq_pcie_wr_other_conf,
	.rd_own_conf = lantiq_pcie_rd_own_conf,
	.wr_own_conf = lantiq_pcie_wr_own_conf,
};

static int lantiq_pcie_link_up(struct dw_pcie *pci)
{
	struct lantiq_pcie *ltq_pcie = to_lantiq_pcie(pci);
	u32 tmp;
	int ret;

	ret = regmap_read_poll_timeout(ltq_pcie->app_regmap, PCIE_APP_PHY_SR,
				       tmp, tmp & PCIE_APP_PHY_SR_PHY_LINK_UP,
				       100, 300000);
	if (ret)
		return 0;

	msleep(100);

	/* Check whether the data link is up */
	regmap_read(ltq_pcie->app_regmap, PCIE_APP_RC_DR, &tmp);
	if (!(tmp & PCIE_APP_RC_DR_DLL_UP))
		return 0;

	tmp = dw_pcie_readl_dbi(pci, PCIE_PORT_DEBUG1);
	if (tmp & PCIE_PORT_DEBUG1_LINK_IN_TRAINING ||
	    !(tmp & PCIE_PORT_DEBUG1_LINK_UP))
		return 0;

	return 1;
}

static u32 lantiq_pcie_read_dbi(struct dw_pcie *pci, void __iomem *base,
				u32 reg, size_t size)
{
	struct lantiq_pcie *ltq_pcie = to_lantiq_pcie(pci);
	u32 tmp;

	dw_pcie_read(base + reg, size, &tmp);

	if (ltq_pcie->big_endian) {
		switch (size) {
		case 1:
			return tmp;
		case 2:
			return __le16_to_cpu(tmp);
		case 4:
			return __le32_to_cpu(tmp);
		default:
			return PCIBIOS_BAD_REGISTER_NUMBER;
		}
	}

	return tmp;
}

static void lantiq_pcie_write_dbi(struct dw_pcie *pci, void __iomem *base,
				  u32 reg, size_t size, u32 val)
{
	struct lantiq_pcie *ltq_pcie = to_lantiq_pcie(pci);
	u32 tmp;

	if (ltq_pcie->big_endian) {
		switch (size) {
		case 1:
			tmp = val;
			break;
		case 2:
			tmp = __cpu_to_le16(val);
			break;
		case 4:
			tmp = __cpu_to_le32(val);
			break;
		default:
			return;
		}
	} else {
		tmp = val;
	}

	dw_pcie_write(base + reg, size, tmp);
}

static const struct dw_pcie_ops lantiq_dw_pcie_ops = {
	.link_up = lantiq_pcie_link_up,
	.read_dbi = lantiq_pcie_read_dbi,
	.write_dbi = lantiq_pcie_write_dbi,
};

static void lantiq_dw_pcie_ack_irq(struct irq_data *d)
{
	struct lantiq_pcie *ltq_pcie = irq_data_get_irq_chip_data(d);

	regmap_update_bits(ltq_pcie->app_regmap, PCIE_APP_IRNCR,
			   PCIE_APP_IRNCR_INTA << d->hwirq,
			   PCIE_APP_IRNCR_INTA << d->hwirq);
}

static void lantiq_dw_pcie_mask_irq(struct irq_data *d)
{
	struct lantiq_pcie *ltq_pcie = irq_data_get_irq_chip_data(d);

	regmap_update_bits(ltq_pcie->app_regmap, PCIE_APP_IRNEN,
			   PCIE_APP_IRNEN_INTA << d->hwirq, 0);
}

static void lantiq_dw_pcie_unmask_irq(struct irq_data *d)
{
	struct lantiq_pcie *ltq_pcie = irq_data_get_irq_chip_data(d);

	regmap_update_bits(ltq_pcie->app_regmap, PCIE_APP_IRNEN,
			   PCIE_APP_IRNEN_INTA << d->hwirq,
			   PCIE_APP_IRNEN_INTA << d->hwirq);
}

static void lantiq_dw_pcie_irq_handler(struct irq_desc *desc)
{
	unsigned int *mapped_irq = irq_desc_get_handler_data(desc);
	struct irq_chip *irqchip = irq_desc_get_chip(desc);

	chained_irq_enter(irqchip, desc);

	generic_handle_irq(*mapped_irq);

	chained_irq_exit(irqchip, desc);
}

static struct irq_chip lantiq_dw_pcie_irq_chip = {
	.name = "LANTIQ-PCIe",
	.irq_ack = lantiq_dw_pcie_ack_irq,
	.irq_mask = lantiq_dw_pcie_mask_irq,
	.irq_unmask = lantiq_dw_pcie_unmask_irq,
};

static int lantiq_dw_pcie_irq_map(struct irq_domain *domain, unsigned int irq,
				  irq_hw_number_t hwirq)
{
	irq_set_chip_and_handler(irq, &lantiq_dw_pcie_irq_chip,
				 handle_level_irq);
	irq_set_chip_data(irq, domain->host_data);

	return 0;
}

static const struct irq_domain_ops lantiq_dw_pcie_irqdomain_ops = {
	.map = lantiq_dw_pcie_irq_map,
};

static void lantiq_dw_pcie_remove_irq_domain(void *data)
{
	struct irq_domain *domain = data;

	irq_domain_remove(domain);
}

static int lantiq_dw_pcie_setup_cascaded_irq(struct lantiq_pcie *ltq_pcie)
{
	int irqs[PCIE_APP_INTC_NUM_IRQS], ret, i;
	struct device *dev = ltq_pcie->pci.dev;
	struct device_node *intc_np;
	struct irq_domain *domain;

	intc_np = of_get_child_by_name(dev->of_node, "interrupt-controller");
	if (!intc_np) {
		dev_err(dev, "Missing child node 'interrupt-controller'\n");
		ret = -EINVAL;
		goto err;
	}

	if (of_irq_count(intc_np) != PCIE_APP_INTC_NUM_IRQS) {
		dev_err(dev, "There must be five interrupt inputs\n");
		ret = -EINVAL;
		goto err_put_intc_np;
	}

	domain = irq_domain_add_linear(intc_np, PCIE_APP_INTC_NUM_IRQS,
				       &lantiq_dw_pcie_irqdomain_ops,
				       ltq_pcie);
	if (!domain) {
		dev_err(dev, "Failed to create PCI IRQ domain\n");
		ret = -EINVAL;
		goto err_put_intc_np;
	}

	ret = devm_add_action_or_reset(dev, lantiq_dw_pcie_remove_irq_domain,
				       domain);
	if (ret)
		goto err_put_intc_np;

	for (i = 0; i < PCIE_APP_INTC_NUM_IRQS; i++) {
		irqs[i] = irq_of_parse_and_map(intc_np, i);
		if (!irqs[i]) {
			dev_err(dev, "Failed to map interrupt %d\n", i);
			goto err_dispose_irq_mapping;
		}

		ltq_pcie->mapped_irqs[i] = irq_create_mapping(domain, i);

		irq_set_chained_handler_and_data(irqs[i],
						 lantiq_dw_pcie_irq_handler,
						 &ltq_pcie->mapped_irqs[i]);
	}

	of_node_put(intc_np);

	return 0;

err_dispose_irq_mapping:
	while (--i >= 0) {
		irq_dispose_mapping(irqs[i]);
		irq_dispose_mapping(ltq_pcie->mapped_irqs[i]);
	}
err_put_intc_np:
	of_node_put(intc_np);
err:
	return ret;
}

static int lantiq_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct lantiq_pcie *ltq_pcie;
	struct resource *res;
	void __iomem *base;
	int ret, irq;

	ltq_pcie = devm_kzalloc(dev, sizeof(*ltq_pcie), GFP_KERNEL);
	if (!ltq_pcie)
		return -ENOMEM;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dbi");
	ltq_pcie->pci.dbi_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(ltq_pcie->pci.dbi_base))
		return PTR_ERR(ltq_pcie->pci.dbi_base);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "app");
	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	ltq_pcie->app_regmap =
		devm_regmap_init_mmio(&pdev->dev, base,
				      &lantiq_dw_pcie_app_regmap_config);
	if (IS_ERR(ltq_pcie->app_regmap))
		return PTR_ERR(ltq_pcie->app_regmap);

	ltq_pcie->rcu_regmap = syscon_regmap_lookup_by_phandle(dev->of_node,
							       "lantiq,rcu");
	if (IS_ERR(ltq_pcie->rcu_regmap))
		return PTR_ERR(ltq_pcie->rcu_regmap);

	irq = platform_get_irq_byname(pdev, "ir");
	if (irq < 0)
		return irq;

	ret = devm_request_irq(dev, irq, lantiq_pcie_app_irq_handler, 0, NULL,
			       &ltq_pcie->pci);
	if (ret)
		return ret;

	ltq_pcie->ahb_clk = devm_clk_get(dev, "ahb");
	if (IS_ERR(ltq_pcie->ahb_clk))
		return PTR_ERR(ltq_pcie->ahb_clk);

	ltq_pcie->bus_clk = devm_clk_get(dev, "pcie_bus");
	if (IS_ERR(ltq_pcie->bus_clk))
		return PTR_ERR(ltq_pcie->bus_clk);

	ltq_pcie->ctl_clk = devm_clk_get(dev, "ctl");
	if (IS_ERR(ltq_pcie->ctl_clk))
		return PTR_ERR(ltq_pcie->ctl_clk);

	ltq_pcie->pcie_reset = devm_reset_control_get_shared(dev, NULL);
	if (IS_ERR(ltq_pcie->pcie_reset))
		return PTR_ERR(ltq_pcie->pcie_reset);

	ltq_pcie->phy = devm_phy_get(dev, "pcie");
	if (IS_ERR(ltq_pcie->phy))
		return PTR_ERR(ltq_pcie->phy);

	ltq_pcie->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						       GPIOD_OUT_LOW);
	if (IS_ERR(ltq_pcie->reset_gpio))
		return PTR_ERR(ltq_pcie->reset_gpio);

	ltq_pcie->big_endian = of_device_is_big_endian(dev->of_node);

	/* TODO: move these to devicetree like in the PHY driver */
	ltq_pcie->rcu_ahb_endian_offset = 0x4c;
	ltq_pcie->rcu_ahb_endian_big_endian_pciem_mask = BIT(0);
	ltq_pcie->rcu_ahb_endian_big_endian_xbarm_mask = BIT(1);
	ltq_pcie->rcu_ahb_endian_big_endian_pcies_mask = BIT(4);

	ltq_pcie->pci.dev = dev;
	ltq_pcie->pci.ops = &lantiq_dw_pcie_ops;
	ltq_pcie->pci.pp.ops = &lantiq_dw_pcie_host_ops;

	platform_set_drvdata(pdev, ltq_pcie);

	ret = lantiq_dw_pcie_setup_cascaded_irq(ltq_pcie);
	if (ret)
		return ret;

	ret = dw_pcie_host_init(&ltq_pcie->pci.pp);
	if (ret) {
		dev_err(dev, "Failed to initialize host\n");
		return ret;
	}

	return 0;
}

static const struct of_device_id lantiq_pcie_of_match[] = {
	{ .compatible = "lantiq,dw-pcie", },
	{ /* sentinel */ },
};

static struct platform_driver lantiq_pcie_driver = {
	.driver = {
		.name	= "lantiq-pcie",
		.of_match_table = lantiq_pcie_of_match,
		.suppress_bind_attrs = true,
	},
	.probe = lantiq_pcie_probe,
};
builtin_platform_driver(lantiq_pcie_driver);
