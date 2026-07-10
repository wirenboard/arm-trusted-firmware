/*
 * Copyright (c) 2014-2023, Arm Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <asm_macros.S>
#include <assert_macros.S>
#include <lib/psci/psci.h>
#include <platform_def.h>

	.globl	psci_do_pwrdown_cache_maintenance
	.globl	psci_do_pwrup_cache_maintenance
	.globl	psci_power_down_wfi

/* -----------------------------------------------------------------------
 * void psci_do_pwrdown_cache_maintenance(unsigned int power level);
 *
 * This function performs cache maintenance for the specified power
 * level. The levels of cache affected are determined by the power
 * level which is passed as the argument i.e. level 0 results
 * in a flush of the L1 cache. Both the L1 and L2 caches are flushed
 * for a higher power level.
 *
 * Additionally, this function also ensures that stack memory is correctly
 * flushed out to avoid coherency issues due to a change in its memory
 * attributes after the data cache is disabled.
 * -----------------------------------------------------------------------
 */
/* -----------------------------------------------------------------------
 * WB8 (H616) defensive mitigation, WB-fork-local. A rare corruption of a
 * callee-saved register inside this cache-maintenance window (measured live
 * as x21 taking x18's value) later faulted as an alignment data abort
 * dereferencing psci_plat_pm_ops in psci_do_cpu_off. Spill x21-x28 alongside
 * the existing x19/x20 save (the slots sit above SP and are covered by the
 * same stack flush below) and reload them after the window: the reloads hit
 * the explicitly-flushed stack, repairing any in-window corruption before the
 * caller uses those registers. The resume-window under-volt that produced it
 * is fixed in the allwinner off-resume path; this stays as cheap
 * defense-in-depth while that fix accumulates soak history.
 * TODO: remove once the resume-window residual soak-confirms gone (post-P3).
 * -----------------------------------------------------------------------
 */
func psci_do_pwrdown_cache_maintenance
	stp     x29, x30, [sp,#-16]!
	stp     x19, x20, [sp,#-16]!

	/* WB8: spill the rest of the callee-saved set (see the note above);
	 * the slots sit above SP, so the stack flush below covers them
	 * exactly like the x19/x20 spills. x0 (power level) is untouched. */
	stp	x21, x22, [sp,#-16]!
	stp	x23, x24, [sp,#-16]!
	stp	x25, x26, [sp,#-16]!
	stp	x27, x28, [sp,#-16]!

	/* ---------------------------------------------
	 * Invoke CPU-specific power down operations for
	 * the appropriate level
	 * ---------------------------------------------
	 */
	bl	prepare_cpu_pwr_dwn

	/* ---------------------------------------------
	 * Do stack maintenance by flushing the used
	 * stack to the main memory and invalidating the
	 * remainder.
	 * ---------------------------------------------
	 */
	bl	plat_get_my_stack

	/* ---------------------------------------------
	 * Calculate and store the size of the used
	 * stack memory in x1.
	 * ---------------------------------------------
	 */
	mov	x19, x0
	mov	x1, sp
	sub	x1, x0, x1
	mov	x0, sp
	bl	flush_dcache_range

	/* ---------------------------------------------
	 * Calculate and store the size of the unused
	 * stack memory in x1. Calculate and store the
	 * stack base address in x0.
	 * ---------------------------------------------
	 */
	sub	x0, x19, #PLATFORM_STACK_SIZE
	sub	x1, sp, x0
	bl	inv_dcache_range

	/* WB8: reload the callee-saved set from the just-flushed stack
	 * slots, repairing any in-window corruption before the caller
	 * dereferences them (e.g. psci_plat_pm_ops in psci_do_cpu_off). */
	ldp	x27, x28, [sp], #16
	ldp	x25, x26, [sp], #16
	ldp	x23, x24, [sp], #16
	ldp	x21, x22, [sp], #16
	ldp	x19, x20, [sp], #16
	ldp	x29, x30, [sp], #16
	ret
endfunc psci_do_pwrdown_cache_maintenance


/* -----------------------------------------------------------------------
 * void psci_do_pwrup_cache_maintenance(void);
 *
 * This function performs cache maintenance after this cpu is powered up.
 * Currently, this involves managing the used stack memory before turning
 * on the data cache.
 * -----------------------------------------------------------------------
 */
func psci_do_pwrup_cache_maintenance
	stp	x29, x30, [sp,#-16]!

	/* ---------------------------------------------
	 * Ensure any inflight stack writes have made it
	 * to main memory.
	 * ---------------------------------------------
	 */
	dmb	st

	/* ---------------------------------------------
	 * Calculate and store the size of the used
	 * stack memory in x1. Calculate and store the
	 * stack base address in x0.
	 * ---------------------------------------------
	 */
	bl	plat_get_my_stack
	mov	x1, sp
	sub	x1, x0, x1
	mov	x0, sp
	bl	inv_dcache_range

	/* ---------------------------------------------
	 * Enable the data cache.
	 * ---------------------------------------------
	 */
	mrs	x0, sctlr_el3
	orr	x0, x0, #SCTLR_C_BIT
	msr	sctlr_el3, x0
	isb

	ldp	x29, x30, [sp], #16
	ret
endfunc psci_do_pwrup_cache_maintenance

/* -----------------------------------------------------------------------
 * void psci_power_down_wfi(void);
 * This function is called to indicate to the power controller that it
 * is safe to power down this cpu. It should not exit the wfi and will
 * be released from reset upon power up.
 * -----------------------------------------------------------------------
 */
func psci_power_down_wfi
#if ERRATA_A510_2684597
	bl apply_cpu_pwr_dwn_errata
#endif
	dsb	sy		// ensure write buffer empty
1:
	wfi
	b	1b
endfunc psci_power_down_wfi
