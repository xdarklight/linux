// SPDX-License-Identifier: GPL-2.0
/*
 *
 * Copyright (C) 2010 John Crispin <john@phrozen.org>
 * Copyright (C) 2010 Thomas Langer <thomas.langer@lantiq.com>
 * Copyright (C) 2022 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 */

#include <linux/cpu.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/kernel.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/slab.h>

/* register definitions - internal irqs */
#define LTQ_ICU_ISR			0x00
#define LTQ_ICU_IER			0x08
#define LTQ_ICU_IOSR			0x10
#define LTQ_ICU_IRSR			0x18
#define LTQ_ICU_IMR			0x20

#define LTQ_ICU_IM_SIZE			0x28

#define LTQ_ICU_NUM_IR_PER_IM		32
#define LTQ_ICU_HWIRQ_TO_BITMASK(hwirq)	BIT((hwirq) % LTQ_ICU_NUM_IR_PER_IM)

/* the performance counter */
#define LTQ_ICU_PERF_IRQ		((4 * LTQ_ICU_NUM_IR_PER_IM) + 31)

static int ltq_perfcount_irq;

#if defined(CONFIG_MIPS)
int get_c0_perfcount_int(void)
{
	return ltq_perfcount_irq;
}
EXPORT_SYMBOL_GPL(get_c0_perfcount_int);
#endif

struct ltq_icu_priv {
	void __iomem *base[NR_CPUS];
	struct irq_domain *domain;
	unsigned int im;
	raw_spinlock_t lock;
};

static void ltq_icu_irq_set_masked(struct irq_data *d, unsigned int vpe,
				   bool masked)
{
	struct ltq_icu_priv *priv = irq_data_get_irq_chip_data(d);
	u32 temp;

	temp = ioread32be(priv->base[vpe] + LTQ_ICU_IER);
	if (masked)
		temp &= ~LTQ_ICU_HWIRQ_TO_BITMASK(d->hwirq);
	else
		temp |= LTQ_ICU_HWIRQ_TO_BITMASK(d->hwirq);

	iowrite32be(temp, priv->base[vpe] + LTQ_ICU_IER);
}

static void ltq_icu_irq_ack(struct irq_data *d)
{
	struct ltq_icu_priv *priv = irq_data_get_irq_chip_data(d);
	unsigned long flags;
	unsigned int vpe;

	raw_spin_lock_irqsave(&priv->lock, flags);
	for_each_present_cpu(vpe) {
		iowrite32be(LTQ_ICU_HWIRQ_TO_BITMASK(d->hwirq),
			    priv->base[vpe] + LTQ_ICU_ISR);
	}
	raw_spin_unlock_irqrestore(&priv->lock, flags);
}

static void ltq_icu_irq_mask(struct irq_data *d)
{
	struct ltq_icu_priv *priv = irq_data_get_irq_chip_data(d);
	unsigned long flags;
	unsigned int vpe;

	raw_spin_lock_irqsave(&priv->lock, flags);
	for_each_present_cpu(vpe) {
		ltq_icu_irq_set_masked(d, vpe, true);
	}
	raw_spin_unlock_irqrestore(&priv->lock, flags);
}

static void ltq_icu_irq_unmask(struct irq_data *d)
{
	struct ltq_icu_priv *priv = irq_data_get_irq_chip_data(d);
	unsigned long flags;
	unsigned int vpe;

	vpe = cpumask_first_and(irq_data_get_affinity_mask(d),
				cpu_online_mask);

	/* This shouldn't be even possible, maybe during CPU hotplug spam */
	if (unlikely(vpe >= nr_cpu_ids))
		vpe = smp_processor_id();

	raw_spin_lock_irqsave(&priv->lock, flags);
	ltq_icu_irq_set_masked(d, vpe, false);
	raw_spin_unlock_irqrestore(&priv->lock, flags);
}

static int ltq_icu_irq_set_affinity(struct irq_data *d,
				    const struct cpumask *cpumask, bool force)
{
	struct ltq_icu_priv *priv = irq_data_get_irq_chip_data(d);
	unsigned int new_vpe, vpe;
	unsigned long flags;

	if (force)
		new_vpe = cpumask_first(cpumask);
	else
		new_vpe = cpumask_any_and(cpumask, cpu_online_mask);

	raw_spin_lock_irqsave(&priv->lock, flags);
	for_each_present_cpu(vpe) {
		ltq_icu_irq_set_masked(d, vpe,
				       vpe != new_vpe || irqd_irq_masked(d));
	}
	raw_spin_unlock_irqrestore(&priv->lock, flags);

	irq_data_update_effective_affinity(d, cpumask_of(new_vpe));

	return IRQ_SET_MASK_OK;
}

static int ltq_icu_irq_retrigger(struct irq_data *d)
{
	struct ltq_icu_priv *priv = irq_data_get_irq_chip_data(d);
	unsigned long flags;
	unsigned int vpe;
	u32 temp;

	vpe = cpumask_first_and(irq_data_get_affinity_mask(d),
				cpu_online_mask);

	raw_spin_lock_irqsave(&priv->lock, flags);
	temp = ioread32be(priv->base[vpe] + LTQ_ICU_IRSR);
	temp |= LTQ_ICU_HWIRQ_TO_BITMASK(d->hwirq);
	iowrite32be(temp, priv->base[vpe] + LTQ_ICU_ISR);
	raw_spin_unlock_irqrestore(&priv->lock, flags);

	return irq_chip_retrigger_hierarchy(d);
}

static struct irq_chip ltq_icu_irq_chip = {
	.name = "ICU",
	.irq_ack = ltq_icu_irq_ack,
	.irq_mask = ltq_icu_irq_mask,
	.irq_unmask = ltq_icu_irq_unmask,
	.irq_set_affinity = ltq_icu_irq_set_affinity,
	.irq_retrigger = ltq_icu_irq_retrigger,
};

static void ltq_icu_irq_handler(struct irq_desc *desc)
{
	struct ltq_icu_priv *priv = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	unsigned int vpe = smp_processor_id();
	irq_hw_number_t hwirq;
	u32 iosr;

	chained_irq_enter(chip, desc);

	iosr = ioread32be(priv->base[vpe] + LTQ_ICU_IOSR);
	if (iosr) {
		/*
		 * silicon bug causes only the msb set to 1 to be valid, all
		 * other bits might be bogus.
		*/
		hwirq = __fls(iosr) + (LTQ_ICU_NUM_IR_PER_IM * priv->im);

		generic_handle_domain_irq(priv->domain, hwirq);
	}

	chained_irq_exit(chip, desc);
}

static int ltq_icu_alloc(struct irq_domain *domain, unsigned int virq,
			 unsigned int nr_irqs, void *arg)
{
	struct ltq_icu_priv *priv = domain->host_data;
	struct irq_fwspec *fwspec = arg;
	unsigned int i, im, type;
	unsigned long hwirq;
	int ret;

	ret = irq_domain_translate_onecell(domain, fwspec, &hwirq, &type);
	if (ret)
		return ret;

	for (i = 0; i < nr_irqs; i++) {
		im = (hwirq + i) / LTQ_ICU_NUM_IR_PER_IM;
		irq_domain_set_info(domain, virq + i, hwirq + i,
				    &ltq_icu_irq_chip, &priv[im],
				    handle_level_irq, NULL, NULL);
	}

	return 0;
}

static const struct irq_domain_ops ltq_icu_irq_domain_ops = {
	.translate	= irq_domain_translate_onecell,
	.alloc		= ltq_icu_alloc,
	.free		= irq_domain_free_irqs_parent,
};

static int __init ltq_icu_of_init(struct device_node *node,
				  struct device_node *parent)
{
	struct irq_domain *icu_domain;
	struct ltq_icu_priv *priv;
	unsigned int im, vpe;
	void __iomem *base;
	int irq, num_im;

	num_im = of_irq_count(node);

	priv = kcalloc(num_im, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	/* load register regions of available ICUs */
	for_each_possible_cpu(vpe) {
		base = of_io_request_and_map(node, vpe, of_node_full_name(node));
		if (!base) {
			pr_err("%pOF: Failed to remap ICU%i memory\n", node,
			       vpe);
			kfree(priv);
			return -ENXIO;
		}

		for (im = 0; im < num_im; im++) {
			priv[im].base[vpe] = base + (LTQ_ICU_IM_SIZE * im);
			raw_spin_lock_init(&priv[im].lock);
			priv[im].im = im;

			/* make sure all irqs are turned off by default */
			iowrite32be(0, priv[im].base[vpe] + LTQ_ICU_IER);

			/* clear all possibly pending interrupts */
			iowrite32be(~0, priv[im].base[vpe] + LTQ_ICU_ISR);

			/* clear resend */
			iowrite32be(0, priv[im].base[vpe] + LTQ_ICU_IRSR);
		}
	}

	icu_domain = irq_domain_add_linear(node,
					   num_im * LTQ_ICU_NUM_IR_PER_IM,
					   &ltq_icu_irq_domain_ops, priv);
	if (!icu_domain) {
		kfree(priv);
		return -ENOMEM;
	}

	for (im = 0; im < num_im; im++) {
		irq = of_irq_get(node, im);
		if (irq < 0) {
			pr_err("%pOF: Failed to get interrupt for IM%u\n",
			       node, im);
			irq_domain_remove(icu_domain);
			kfree(priv);
			return irq;
		}

		priv[im].domain = icu_domain;

		irq_set_chained_handler_and_data(irq, ltq_icu_irq_handler,
						 &priv[im]);
	}

	/* tell oprofile which irq to use */
	ltq_perfcount_irq = irq_create_mapping(icu_domain, LTQ_ICU_PERF_IRQ);

	return 0;
}
IRQCHIP_DECLARE(ltq_icu_irq, "lantiq,icu", ltq_icu_of_init);
