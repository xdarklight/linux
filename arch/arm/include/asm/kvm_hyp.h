/*
 * Copyright (C) 2015 - ARM Ltd
 * Author: Marc Zyngier <marc.zyngier@arm.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __ARM_KVM_HYP_H__
#define __ARM_KVM_HYP_H__

#include <linux/compiler.h>
#include <linux/kvm_host.h>
#include <asm/cp15.h>
#include <asm/vfp.h>

#define __hyp_text __section(.hyp.text) notrace

#define __ACCESS_VFP(CRn)			\
	"mrc", "mcr", __stringify(p10, 7, %0, CRn, cr0, 0), u32

#define write_special(v, r)					\
	asm volatile("msr " __stringify(r) ", %0" : : "r" (v))
#define read_special(r) ({					\
	u32 __val;						\
	asm volatile("mrs %0, " __stringify(r) : "=r" (__val));	\
	__val;							\
})

#define VFP_FPEXC	__ACCESS_VFP(FPEXC)

/* AArch64 compatibility macros, only for the timer so far */
#define read_sysreg_el0(r)		read_sysreg(r##_el0)
#define write_sysreg_el0(v, r)		write_sysreg(v, r##_el0)

#define cntp_ctl_el0			CNTP_CTL
#define cntp_cval_el0			CNTP_CVAL
#define cntv_ctl_el0			CNTV_CTL
#define cntv_cval_el0			CNTV_CVAL
#define cntvoff_el2			CNTVOFF
#define cnthctl_el2			CNTHCTL

void __timer_enable_traps(struct kvm_vcpu *vcpu);
void __timer_disable_traps(struct kvm_vcpu *vcpu);

void __vgic_v2_save_state(struct kvm_vcpu *vcpu);
void __vgic_v2_restore_state(struct kvm_vcpu *vcpu);

void __sysreg_save_state(struct kvm_cpu_context *ctxt);
void __sysreg_restore_state(struct kvm_cpu_context *ctxt);

void __vgic_v3_save_state(struct kvm_vcpu *vcpu);
void __vgic_v3_restore_state(struct kvm_vcpu *vcpu);
void __vgic_v3_activate_traps(struct kvm_vcpu *vcpu);
void __vgic_v3_deactivate_traps(struct kvm_vcpu *vcpu);
void __vgic_v3_save_aprs(struct kvm_vcpu *vcpu);
void __vgic_v3_restore_aprs(struct kvm_vcpu *vcpu);

asmlinkage void __vfp_save_state(struct vfp_hard_struct *vfp);
asmlinkage void __vfp_restore_state(struct vfp_hard_struct *vfp);
static inline bool __vfp_enabled(void)
{
	return !(read_sysreg(HCPTR) & (HCPTR_TCP(11) | HCPTR_TCP(10)));
}

void __hyp_text __banked_save_state(struct kvm_cpu_context *ctxt);
void __hyp_text __banked_restore_state(struct kvm_cpu_context *ctxt);

asmlinkage int __guest_enter(struct kvm_vcpu *vcpu,
			     struct kvm_cpu_context *host);
asmlinkage int __hyp_do_panic(const char *, int, u32);

#endif /* __ARM_KVM_HYP_H__ */
