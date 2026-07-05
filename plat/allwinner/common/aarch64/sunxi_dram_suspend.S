/*
 * Copyright (c) 2026, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * DRAM self-refresh retention wait for the H616/T507 SYSTEM_SUSPEND
 * implementation.
 *
 * This code is copied to SRAM A1 and executed there with the MMU off,
 * because BL31 itself lives in DRAM on these SoCs and DRAM must not be
 * touched between self-refresh entry and exit. It is fully position
 * independent (immediate materialisation only, local relative branches
 * only), uses no stack and no memory besides the device registers, and
 * only clobbers caller-saved registers.
 *
 * Progress breadcrumbs (stage numbers) are written to RTC data reg 2
 * (0x07000108) so that a stall can be located after the EC watchdog
 * recovers the board — the RTC domain survives warm resets.
 *
 * Register use:
 *   x1  = DRAM CTL base       x2 = CCU base    x8 = RTC data reg 2
 *   x9  = saved mbus_cfg      x10 = saved dram_gate_reset
 *   x11 = saved pll5_cfg      x12 = saved cpu_axi_cfg
 *   x13 = saved pll_cpux      x15 = wait start (CNTPCT)
 *   x3, x4, x5 = scratch
 *
 * Returns: x0 = wait duration in milliseconds.
 */

#include <asm_macros.S>

#define SUNXI_CCU_BASE_L	0x03001000
#define SUNXI_DRAM_CTL_BASE_L	0x047fb000
#define SUNXI_RTC_DATA2_L	0x07000108

#define MCTL_PWRCTL		0x30
#define MCTL_STATR		0x04

#define CCU_PLL_CPUX		0x000
#define CCU_PLL_DDR0		0x010
#define CCU_CPU_AXI_CFG		0x500
#define CCU_MBUS_CFG		0x540
#define CCU_DRAM_BGR		0x80c

	.macro	stage num
	mov	w3, #\num
	str	w3, [x8]
	.endm

	.globl	sunxi_dram_suspend_blob
	.globl	sunxi_dram_suspend_blob_end

func sunxi_dram_suspend_blob
	mov_imm	x1, SUNXI_DRAM_CTL_BASE_L
	mov_imm	x2, SUNXI_CCU_BASE_L
	mov_imm	x8, SUNXI_RTC_DATA2_L

	stage	0xa1

	/*
	 * 1. Enter software self-refresh with the DRAM clock stopped by
	 * the controller itself (en_dfi_dram_clk_disable, PHY-coordinated
	 * — a manual PLL_DDR0/BGR shutdown kills the PHY calibration and
	 * DRAM reads return garbage afterwards, found the hard way).
	 * Wait for operating_mode = 3.
	 */
	ldr	w3, [x1, #MCTL_PWRCTL]
	orr	w3, w3, #0x20		/* selfref_sw */
	orr	w3, w3, #0x200		/* en_dfi_dram_clk_disable */
	str	w3, [x1, #MCTL_PWRCTL]
1:	ldr	w3, [x1, #MCTL_STATR]
	and	w3, w3, #3
	cmp	w3, #3
	b.ne	1b

	stage	0xa3

	/* 3. CPU cluster to 24 MHz, PLL_CPUX off. */
	ldr	w12, [x2, #CCU_CPU_AXI_CFG]
	and	w3, w12, #0xf8ffffff	/* mux to HOSC */
	str	w3, [x2, #CCU_CPU_AXI_CFG]
	dsb	sy
	isb
	ldr	w13, [x2, #CCU_PLL_CPUX]
	and	w3, w13, #0x7fffffff
	str	w3, [x2, #CCU_PLL_CPUX]
	dsb	sy

	stage	0xa4

	/* 4. Wait for a pending interrupt; cap at 600 s (backstop). */
	mrs	x15, cntpct_el0
	mov_imm	x5, (24000000 * 600)
2:	mrs	x3, isr_el1
	cbnz	x3, 3f
	mrs	x4, cntpct_el0
	sub	x4, x4, x15
	cmp	x4, x5
	b.lo	2b
3:
	stage	0xa5

	/*
	 * 5a. PLL_CPUX back on. Force LOCK_EN (bit 29) for the lock poll:
	 * the OS may run with lock detection disabled, in which case the
	 * LOCK flag would never assert and the poll would hang. The clean
	 * saved value is written back once the PLL has locked.
	 */
	orr	w3, w13, #(1 << 29)
	orr	w3, w3, #(1 << 31)
	str	w3, [x2, #CCU_PLL_CPUX]
4:	ldr	w3, [x2, #CCU_PLL_CPUX]
	tbz	w3, #28, 4b
	str	w13, [x2, #CCU_PLL_CPUX]
	str	w12, [x2, #CCU_CPU_AXI_CFG]
	dsb	sy
	isb

	stage	0xa7

	/* 6. Leave self-refresh, wait for operating_mode = normal. */
	ldr	w3, [x1, #MCTL_PWRCTL]
	and	w3, w3, #0xffffffdf	/* clear selfref_sw */
	and	w3, w3, #0xfffffdff	/* clear en_dfi_dram_clk_disable */
	str	w3, [x1, #MCTL_PWRCTL]
7:	ldr	w3, [x1, #MCTL_STATR]
	and	w3, w3, #3
	cmp	w3, #1
	b.ne	7b

	stage	0xa8

	/* Return the wait duration in ms. */
	mrs	x0, cntpct_el0
	sub	x0, x0, x15
	mov_imm	x3, 24000
	udiv	x0, x0, x3
	ret
endfunc sunxi_dram_suspend_blob
sunxi_dram_suspend_blob_end:
