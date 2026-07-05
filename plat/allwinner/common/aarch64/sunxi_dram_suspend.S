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
#define SUNXI_PIO_BASE_L	0x0300b000
#define SUNXI_DRAM_CTL_BASE_L	0x047fb000
#define SUNXI_RTC_DATA2_L	0x07000108

#define PIO_PE_DAT		0x0a0
#define PIO_PG_DAT		0x0e8

/*
 * Carrier load switches driven by T507 GPIOs (WB 8.5.3 schematic):
 *   PE15 = WIFI_EN   (U8  -> WiFi+BT module 3V3)
 *   PE9  = USB_PWR_EN (U13 -> external USB port VBUS)
 *   PE4  = MODEM_ON  (U11 -> 5V_MODEM)
 *   PG12 = ON_PERIPH (U10/U5 -> 5V_PERIPH: MOD slots, RS-485, 5V_IN/OUT)
 *   PG13 = 5V-ON     (U12 -> 5VOUT terminal)
 * All are cut for the duration of the suspend and restored on wakeup;
 * the WiFi/BT module re-enumerates on resume.
 */
#define PE_CUT_MASK		((1 << 15) | (1 << 9) | (1 << 4))
#define PG_CUT_MASK		((1 << 12) | (1 << 13))

#define MCTL_PWRCTL		0x30
#define MCTL_STATR		0x04

#define CCU_PLL_CPUX		0x000
#define CCU_PLL_DDR0		0x010
#define CCU_PLL_PERIPH0		0x020
#define CCU_PLL_PERIPH1		0x028
#define CCU_CPU_AXI_CFG		0x500
#define CCU_PSI_AHB1_AHB2	0x510
#define CCU_AHB3		0x51c
#define CCU_APB1		0x520
#define CCU_APB2		0x524
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

	/*
	 * 1b. Cut the carrier peripheral rails (GPIO-driven load
	 * switches), saving the previous GPIO data values.
	 */
	mov_imm	x7, SUNXI_PIO_BASE_L
	ldr	w6, [x7, #PIO_PE_DAT]
	mov	w3, #PE_CUT_MASK
	bic	w3, w6, w3
	str	w3, [x7, #PIO_PE_DAT]
	ldr	w16, [x7, #PIO_PG_DAT]
	mov	w3, #PG_CUT_MASK
	bic	w3, w16, w3
	str	w3, [x7, #PIO_PG_DAT]

	/*
	 * 2. Buses to 24 MHz, then stop the big PLLs: PERIPH0/1 (buses
	 * are on HOSC now), DDR0 (DRAM clock is already parked by the
	 * DFI handshake, the controller state just freezes). The RTC,
	 * R_INTC and the GIC keep working — that is the wakeup path.
	 */
	ldr	w9,  [x2, #CCU_PSI_AHB1_AHB2]
	ldr	w10, [x2, #CCU_AHB3]
	ldr	w11, [x2, #CCU_APB1]
	ldr	w14, [x2, #CCU_APB2]
	and	w3, w9, #0xf8ffffff	/* PSI/AHB1/AHB2 to HOSC */
	str	w3, [x2, #CCU_PSI_AHB1_AHB2]
	and	w3, w10, #0xf8ffffff	/* AHB3 to HOSC */
	str	w3, [x2, #CCU_AHB3]
	and	w3, w11, #0xf8ffffff	/* APB1 to HOSC */
	str	w3, [x2, #CCU_APB1]
	and	w3, w14, #0xf8ffffff	/* APB2 to HOSC */
	str	w3, [x2, #CCU_APB2]
	dsb	sy
	isb
	/*
	 * PLL_DDR0 must stay running: the DDR PHY depends on it even
	 * with the DRAM clock parked via the DFI handshake — stopping
	 * it makes all DRAM reads return garbage after self-refresh
	 * exit (operating_mode reports "normal" regardless).
	 *
	 * PLL_PERIPH0 must stay running too: it clocks the USB
	 * hosts/PHYs, and suspended USB devices drop off the bus when
	 * it stops (the rtl8733bu WiFi module dies until power-cycled).
	 */
	ldr	w17, [x2, #CCU_PLL_PERIPH1]
	and	w3, w17, #0x7fffffff
	str	w3, [x2, #CCU_PLL_PERIPH1]

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

	/* Restore the peripheral rails first — they need time to ramp. */
	str	w6, [x7, #PIO_PE_DAT]
	str	w16, [x7, #PIO_PG_DAT]

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

	stage	0xa6

	/* 5b. PLL_PERIPH1 back on (same LOCK_EN dance). */
	orr	w3, w17, #(1 << 29)
	orr	w3, w3, #(1 << 31)
	str	w3, [x2, #CCU_PLL_PERIPH1]
9:	ldr	w3, [x2, #CCU_PLL_PERIPH1]
	tbz	w3, #28, 9b
	str	w17, [x2, #CCU_PLL_PERIPH1]

	/* 5c. Bus muxes back. */
	str	w9,  [x2, #CCU_PSI_AHB1_AHB2]
	str	w10, [x2, #CCU_AHB3]
	str	w11, [x2, #CCU_APB1]
	str	w14, [x2, #CCU_APB2]
	dsb	sy
	isb

	/* ~20 us settle time. */
	mrs	x4, cntpct_el0
	add	x4, x4, #480
6:	mrs	x3, cntpct_el0
	cmp	x3, x4
	b.lo	6b

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
