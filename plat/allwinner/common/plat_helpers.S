/*
 * Copyright (c) 2017-2018, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <arch.h>
#include <asm_macros.S>
#include <sunxi_def.h>
#include <sunxi_mmap.h>

	.globl	plat_crash_console_init
	.globl	plat_crash_console_putc
	.globl	plat_crash_console_flush
	.globl	plat_my_core_pos
	.globl	platform_mem_init
	.globl	plat_report_exception

func plat_crash_console_init
	mov_imm x0, SUNXI_UART0_BASE
	mov_imm x1, SUNXI_UART0_CLK_IN_HZ
	mov_imm x2, SUNXI_UART0_BAUDRATE
	b	console_16550_core_init
endfunc plat_crash_console_init

func plat_crash_console_putc
	mov_imm x1, SUNXI_UART0_BASE
	b	console_16550_core_putc
endfunc plat_crash_console_putc

func plat_crash_console_flush
	ret
endfunc plat_crash_console_flush

func plat_my_core_pos
	mrs	x0, mpidr_el1
	and	x1, x0, #MPIDR_CLUSTER_MASK
	and	x0, x0, #MPIDR_CPU_MASK
	add	x0, x0, x1, LSR #6
	ret
endfunc plat_my_core_pos

func platform_mem_init
	ret
endfunc platform_mem_init

func plat_report_exception
	ret
endfunc plat_report_exception

	.globl	sunxi_offresume_entrypoint

	/*
	 * Suspend-to-off resume entry for CPU0, branched into by the SPL
	 * with a raw jump (no cache maintenance on the SPL side) after the
	 * whole cluster lost power. WARMBOOT_ENABLE_DCACHE_EARLY turns the
	 * D-cache on early in bl31_warm_entrypoint, so any stale/garbage
	 * lines inherited from POR/BROM/SPL become live data — the suspect
	 * behind rare post-resume garbage-pointer faults. Invalidate the
	 * whole hierarchy first (INVALIDATE ONLY — the suspend entry path
	 * already flushed everything real to DRAM; a clean here would write
	 * garbage back). Safe as CPU0-exclusive: every secondary is still
	 * powered off at this point.
	 */
func sunxi_offresume_entrypoint
	mov	x0, #DCISW
	bl	dcsw_op_all
	ic	iallu
	tlbi	alle3
	dsb	sy
	isb
	b	bl31_warm_entrypoint
endfunc sunxi_offresume_entrypoint
