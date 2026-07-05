/*
 * Copyright (c) 2017-2021, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <string.h>

#include <arch_helpers.h>
#include <common/debug.h>
#include <drivers/arm/gicv2.h>
#include <drivers/delay_timer.h>
#include <drivers/mentor/mi2cv.h>
#include <lib/mmio.h>
#include <lib/psci/psci.h>

#include <sunxi_cpucfg.h>
#include <sunxi_mmap.h>
#include <sunxi_private.h>

/*
 * Scratch area in SRAM A1 for the DRAM self-refresh retention blob.
 * SRAM A1 is only used by the BROM/SPL during boot; at runtime it is
 * free (with BL31 in DRAM, even the NOBITS sections live elsewhere).
 * The first page is skipped to stay clear of BROM leftovers.
 */
#define SUNXI_SUSPEND_SRAM_BASE		(SUNXI_SRAM_A1_BASE + 0x1000U)

extern char sunxi_dram_suspend_blob[], sunxi_dram_suspend_blob_end[];

/*
 * Peripheral power for the suspend window.
 *
 * Carrier (WB 8.5.3): SY6280 load switches with T507 GPIO enables —
 * PE15 WiFi/BT 3V3, PE9 USB VBUS, PE4 modem 5V, PG12 peripheral
 * 5V/RS-485/MOD, PG13 5VOUT terminal. Module: the two DP83825 PHYs
 * have their INTR/PWRDN pins on PE1/PE8 (unused as interrupts, the
 * MDIO bus is polled) — driven low they enter hardware power-down.
 *
 * PMIC (AXP853T ≈ AXP858/AXP15060 at 0x36 on R_I2C): DCDC4 (VDD-GPU,
 * confirmed powered and unused) and BLDO1 (LVDS/HDMI/MCSI, unused)
 * off; DCDC2 (VDD-CPU) from its runtime point down to 0.85 V — only
 * ever while the cluster runs at 24 MHz.
 */
#define SUNXI_PIO_PE_CFG0	(SUNXI_PIO_BASE + 0x90U)
#define SUNXI_PIO_PE_CFG1	(SUNXI_PIO_BASE + 0x94U)
#define SUNXI_PIO_PE_DAT	(SUNXI_PIO_BASE + 0xa0U)
#define SUNXI_PIO_PG_DAT	(SUNXI_PIO_BASE + 0xe8U)

#define PE_RAIL_MASK		((1U << 15) | (1U << 9) | (1U << 4))
#define PE_PHY_MASK		((1U << 1) | (1U << 8))
#define PG_RAIL_MASK		((1U << 12) | (1U << 13))

#define AXP_I2C_ADDR		0x36
#define AXP_REG_OUT_CTRL1	0x10	/* bit3 = DCDC4 */
#define AXP_REG_OUT_CTRL2	0x11	/* bit5 = BLDO1 */
#define AXP_REG_DCDC2_V		0x14	/* 0.5 V + 10 mV steps (<= 1.2 V) */

#define AXP_DCDC2_SUSPEND_V	35U	/* 0.5 V + 35 * 10 mV = 0.85 V */

static struct {
	uint32_t pe_cfg0, pe_cfg1, pe_dat, pg_dat;
	uint32_t cpu_axi, pll_cpux;
	int	 pmic_ok;
	uint8_t	 out_ctrl1, out_ctrl2, dcdc2_v;
} sus;

static int axp_rd(uint8_t reg, uint8_t *val)
{
	return i2c_read(AXP_I2C_ADDR, reg, 1, val, 1);
}

static int axp_wr(uint8_t reg, uint8_t val)
{
	return i2c_write(AXP_I2C_ADDR, reg, 1, &val, 1);
}

static void sunxi_suspend_periph_cut(void)
{
	sus.pe_cfg0 = mmio_read_32(SUNXI_PIO_PE_CFG0);
	sus.pe_cfg1 = mmio_read_32(SUNXI_PIO_PE_CFG1);
	sus.pe_dat  = mmio_read_32(SUNXI_PIO_PE_DAT);
	sus.pg_dat  = mmio_read_32(SUNXI_PIO_PG_DAT);

	/* Rail switches: outputs already, just drive low. */
	mmio_write_32(SUNXI_PIO_PE_DAT,
		      sus.pe_dat & ~(PE_RAIL_MASK | PE_PHY_MASK));
	mmio_write_32(SUNXI_PIO_PG_DAT, sus.pg_dat & ~PG_RAIL_MASK);

	/* PHY PWRDN pins are inputs at runtime: make them outputs (low). */
	mmio_write_32(SUNXI_PIO_PE_CFG0,
		      (sus.pe_cfg0 & ~(0xfU << 4)) | (0x1U << 4));  /* PE1 */
	mmio_write_32(SUNXI_PIO_PE_CFG1,
		      (sus.pe_cfg1 & ~0xfU) | 0x1U);		    /* PE8 */
}

static void sunxi_suspend_periph_restore(void)
{
	mmio_write_32(SUNXI_PIO_PE_CFG0, sus.pe_cfg0);
	mmio_write_32(SUNXI_PIO_PE_CFG1, sus.pe_cfg1);
	mmio_write_32(SUNXI_PIO_PE_DAT, sus.pe_dat);
	mmio_write_32(SUNXI_PIO_PG_DAT, sus.pg_dat);
}

static void sunxi_suspend_pmic_enter(void)
{
	sus.pmic_ok = 0;

	if (sunxi_init_platform_r_twi(sunxi_read_soc_id(), false) != 0) {
		WARN("PSCI: suspend: R_TWI init failed, skipping PMIC\n");
		return;
	}
	i2c_init((void *)SUNXI_R_I2C_BASE);
	mmio_write_32(0x07000108U, 0xb6U);

	if (axp_rd(AXP_REG_OUT_CTRL1, &sus.out_ctrl1) != 0 ||
	    axp_rd(AXP_REG_OUT_CTRL2, &sus.out_ctrl2) != 0 ||
	    axp_rd(AXP_REG_DCDC2_V, &sus.dcdc2_v) != 0) {
		WARN("PSCI: suspend: PMIC read failed, skipping PMIC\n");
		return;
	}

	mmio_write_32(0x07000108U, 0xb7U);
	/* Never raise the voltage: only step down to the suspend point. */
	if ((sus.dcdc2_v & 0x7fU) > AXP_DCDC2_SUSPEND_V) {
		axp_wr(AXP_REG_DCDC2_V,
		       (sus.dcdc2_v & 0x80U) | AXP_DCDC2_SUSPEND_V);
	}
	/*
	 * DCDC1 (the whole 3V3 domain: SoC IO banks, eMMC VCC, carrier
	 * 3V3) is cut too. Electrically safe by design: the PL-bank
	 * pads carrying the PMIC I2C and the bus pull-ups are in the
	 * VCC-RTC domain, the PIO registers are in VDD-SYS (only pads
	 * lose power), the carrier load-switch enables have pull-downs,
	 * and the eMMC sleeps with VCCQ (ALDO1) retained — JEDEC allows
	 * VCC removal in Sleep, the kernel re-initializes the card on
	 * resume. It does, however, look exactly like a dead PMIC to
	 * the EC's 3.3V monitor: the OS MUST announce the sleep window
	 * to the EC (SUSPEND_CTRL regmap register, EC firmware with
	 * suspend-mode support) before suspending, or the EC
	 * hard-cycles the board mid-suspend ("PMIC is unexpectedly
	 * off").
	 */
	axp_wr(AXP_REG_OUT_CTRL1,
	       sus.out_ctrl1 & ~((1U << 3) | (1U << 0))); /* DCDC4, DCDC1 */
	axp_wr(AXP_REG_OUT_CTRL2, sus.out_ctrl2 & ~(1U << 5)); /* BLDO1 */

	sus.pmic_ok = 1;
}

static void sunxi_suspend_pmic_exit(void)
{
	if (sus.pmic_ok == 0) {
		return;
	}

	/* 3V3 domain back first; DCDC soft-start needs a moment. */
	axp_wr(AXP_REG_OUT_CTRL1, sus.out_ctrl1);
	udelay(2000);
	axp_wr(AXP_REG_DCDC2_V, sus.dcdc2_v);
	axp_wr(AXP_REG_OUT_CTRL2, sus.out_ctrl2);

	/* DCDC2 slews at ~2.5 mV/us: give the CPU rail time to rise. */
	udelay(100);
}

static void sunxi_suspend_cpu_slow(void)
{
	sus.cpu_axi  = mmio_read_32(SUNXI_CCU_BASE + 0x500U);
	sus.pll_cpux = mmio_read_32(SUNXI_CCU_BASE + 0x000U);

	mmio_write_32(SUNXI_CCU_BASE + 0x500U,
		      sus.cpu_axi & ~(0x7U << 24));	/* CPUX <- HOSC */
	dsbsy();
	isb();
	udelay(2);
	mmio_write_32(SUNXI_CCU_BASE + 0x000U,
		      sus.pll_cpux & ~BIT_32(31));	/* PLL_CPUX off */
}

static void sunxi_suspend_cpu_fast(void)
{
	/* Re-enable PLL_CPUX with lock detect forced for the poll. */
	mmio_write_32(SUNXI_CCU_BASE + 0x000U,
		      sus.pll_cpux | BIT_32(31) | BIT_32(29));
	while ((mmio_read_32(SUNXI_CCU_BASE + 0x000U) & BIT_32(28)) == 0U) {
	}
	mmio_write_32(SUNXI_CCU_BASE + 0x000U, sus.pll_cpux | BIT_32(31));
	mmio_write_32(SUNXI_CCU_BASE + 0x500U, sus.cpu_axi);
	dsbsy();
	isb();
}

#define SUNXI_WDOG0_CTRL_REG		(SUNXI_R_WDOG_BASE + 0x0010)
#define SUNXI_WDOG0_CFG_REG		(SUNXI_R_WDOG_BASE + 0x0014)
#define SUNXI_WDOG0_MODE_REG		(SUNXI_R_WDOG_BASE + 0x0018)

static int sunxi_pwr_domain_on(u_register_t mpidr)
{
	sunxi_cpu_on(mpidr);

	return PSCI_E_SUCCESS;
}

static void sunxi_pwr_domain_off(const psci_power_state_t *target_state)
{
	gicv2_cpuif_disable();

	sunxi_cpu_power_off_self();
}

static void sunxi_pwr_domain_on_finish(const psci_power_state_t *target_state)
{
	gicv2_pcpu_distif_init();
	gicv2_cpuif_enable();
}

static int sunxi_validate_power_state(unsigned int power_state,
				      psci_power_state_t *req_state)
{
	/* No CPU_SUSPEND (cpuidle) states are implemented. */
	return PSCI_E_INVALID_PARAMS;
}

static void sunxi_pwr_domain_suspend(const psci_power_state_t *target_state)
{
	/*
	 * SYSTEM_SUSPEND, prototype level: nothing is gated or powered down
	 * here yet, so DRAM contents and all device state are trivially
	 * preserved. The GIC is left exactly as the rich OS configured it:
	 * every interrupt it kept enabled remains able to terminate the WFI
	 * in sunxi_pwr_domain_pwr_down_wfi() below.
	 */
	NOTICE("PSCI: System suspend: entering WFI retention\n");
}

static void sunxi_pwr_domain_suspend_finish(const psci_power_state_t *target_state)
{
	gicv2_pcpu_distif_init();
	gicv2_cpuif_enable();

	NOTICE("PSCI: System suspend: wakeup\n");
}

static void __dead2
sunxi_pwr_domain_pwr_down_wfi(const psci_power_state_t *target_state)
{
	if (is_local_state_off(target_state->pwr_domain_state[PLAT_MAX_PWR_LVL])) {
		/*
		 * SYSTEM_SUSPEND: the core keeps its power and context is
		 * gone anyway (the generic code prepared for powerdown),
		 * so wait here until a wakeup interrupt is pending, then
		 * emulate the power-up: caches were cleaned and disabled
		 * by psci_pwrdown_cpu(), so turn off the MMU and re-enter
		 * BL31 through the warm boot entrypoint, exactly as a
		 * physically reset core entering through RVBAR would.
		 * This restores the saved NS context and returns to the
		 * kernel's resume vector.
		 *
		 * DEBUG instrumentation: the EL3 secure physical timer
		 * (PPI 29) is used as a heartbeat so the wait loop can
		 * report the GIC state every few seconds, and to force a
		 * resume after ~3 minutes so a missed wakeup does not end
		 * in an EC watchdog reset.
		 */
		uint64_t beats;
		size_t blob_size = (size_t)(sunxi_dram_suspend_blob_end -
					    sunxi_dram_suspend_blob);

		/*
		 * With the other cores hardware-closed, WFI on this core
		 * never wakes (the CPUIDLE hardware gates the core; GIC
		 * interrupts go pending but are never observed — verified
		 * on WB 8.5.1), so the retention wait polls ISR_EL1.
		 *
		 * The wait runs from SRAM A1 with the MMU off: it puts the
		 * DRAM into self-refresh, stops MBUS/DRAM clocks and
		 * PLL_DDR0, and drops the CPU to the 24 MHz oscillator
		 * with PLL_CPUX off. BL31 lives in DRAM on H616, so
		 * nothing may touch DRAM until the blob returns.
		 */
		mmio_write_32(0x07000108U, 0xb0U);
		memcpy((void *)SUNXI_SUSPEND_SRAM_BASE,
		       sunxi_dram_suspend_blob, blob_size);
		__asm__ volatile("ic iallu" : : : "memory");
		dsbsy();
		isb();

		/* Verify the copy actually landed in SRAM (paranoia:
		 * catches a gated/read-as-zero SRAM before jumping into
		 * it). */
		if (mmio_read_32(SUNXI_SUSPEND_SRAM_BASE) !=
		    *(uint32_t *)sunxi_dram_suspend_blob) {
			ERROR("PSCI: SRAM copy mismatch: %x != %x\n",
			      mmio_read_32(SUNXI_SUSPEND_SRAM_BASE),
			      *(uint32_t *)sunxi_dram_suspend_blob);
			mmio_write_32(0x07000108U, 0xbeU);
			((void (*)(void))sunxi_sec_entrypoint)();
		}

		/*
		 * Cut peripheral power (carrier load switches, PHY
		 * power-down pins) and trim the PMIC (GPU rail and display
		 * LDO off, VDD-CPU lowered) — all while DRAM and the full
		 * CPU clock are still up. The CPU is switched down to
		 * 24 MHz afterwards, so the lowered VDD-CPU is never
		 * exposed to full-speed execution.
		 */
		mmio_write_32(0x07000108U, 0xb3U);
		sunxi_suspend_periph_cut();
		mmio_write_32(0x07000108U, 0xb4U);
		/*
		 * 24 MHz first, PMIC after: the VDD-CPU trim to 0.85 V is
		 * only safe once the cluster runs at HOSC speed. Suspend
		 * entry can catch the CPU at a high OPP (~1.1 V) — cutting
		 * the voltage under full-speed execution browns the cores
		 * out mid-I2C (found as a ~50% entry hang, stage 0xb7).
		 * R_I2C lives in the R clock domain, so the CPUX mux
		 * switch does not affect bus timing.
		 */
		sunxi_suspend_cpu_slow();
		mmio_write_32(0x07000108U, 0xb5U);
		sunxi_suspend_pmic_enter();

		/*
		 * Measurement hack: a magic in RTC data0 (written by the
		 * wb-suspend-off helper) turns this suspend into a one-way
		 * rail kill — after self-refresh entry the SRAM blob
		 * switches off every DCDC except those in the mask
		 * (DCDC5/VCC-DRAM), so only LPDDR4-in-self-refresh + LDOs
		 * + EC remain for the meter. No resume; the EC suspend
		 * deadline reset-recovers the board.
		 */
		{
			uint64_t kill = 0U;

			if (mmio_read_32(0x07000100U) == 0x0ff51ee9U) {
				mmio_write_32(0x07000100U, 0U);
				kill = 0x10U;	/* keep DCDC5 only */
				NOTICE("PSCI: suspend: one-way rail kill armed\n");
			}

			mmio_write_32(0x07000108U, 0xb1U);
			disable_mmu_el3();
			mmio_write_32(0x07000108U, 0xb2U);
			beats = ((uint64_t (*)(uint64_t))SUNXI_SUSPEND_SRAM_BASE)(kill);
		}

		/* Reverse order: rails/voltage back first, then CPU speed
		 * (the CPU may only return to full speed after VDD-CPU is
		 * restored and has settled). */
		sunxi_suspend_pmic_exit();
		sunxi_suspend_periph_restore();
		sunxi_suspend_cpu_fast();

		NOTICE("PSCI: system resume after %llu ms, ISR=%lx\n",
		       (unsigned long long)beats, read_isr_el1());

		/*
		 * DEBUG breadcrumb, readable from Linux after resume even
		 * when the serial console capture is unavailable:
		 * RTC data reg 3 @ 0x0700010c =
		 *   [31:24] poll count  [23:16] HPPIR low byte
		 *   [15:8]  PEND4 bits 15:8 (bit8 = RTC SPI 136)
		 *   [7:0]   ISR_EL1 low byte
		 */
		mmio_write_32(0x0700010cU,
			      ((uint32_t)((beats / 1000U) & 0xffU) << 24) |
			      ((mmio_read_32(SUNXI_GICC_BASE + 0x018) & 0xffU) << 16) |
			      (((mmio_read_32(SUNXI_GICD_BASE + 0x210) >> 8) & 0xffU) << 8) |
			      (read_isr_el1() & 0xffU));

		((void (*)(void))sunxi_sec_entrypoint)();
		/* Not reached. */
	}

	/*
	 * CPU_OFF: power-off was armed in sunxi_cpu_power_off_self(),
	 * the CPUIDLE hardware removes power once this core hits WFI.
	 */
	while (true) {
		dsb();
		wfi();
	}
}

static void sunxi_get_sys_suspend_power_state(psci_power_state_t *req_state)
{
	unsigned int i;

	for (i = 0U; i <= PLAT_MAX_PWR_LVL; i++) {
		req_state->pwr_domain_state[i] = PLAT_MAX_OFF_STATE;
	}
}

static void __dead2 sunxi_system_off(void)
{
	gicv2_cpuif_disable();

	/* Attempt to power down the board (may not return) */
	sunxi_power_down();

	/* Turn off all CPUs */
	sunxi_cpu_power_off_others();
	sunxi_cpu_power_off_self();
	psci_power_down_wfi();
}

static void __dead2 sunxi_system_reset(void)
{
	gicv2_cpuif_disable();

	/* Reset the whole system when the watchdog times out */
	mmio_write_32(SUNXI_WDOG0_CFG_REG, 1);
	/* Enable the watchdog with the shortest timeout (0.5 seconds) */
	mmio_write_32(SUNXI_WDOG0_MODE_REG, (0 << 4) | 1);
	/* Wait for twice the watchdog timeout before panicking */
	mdelay(1000);

	ERROR("PSCI: System reset failed\n");
	panic();
}

static const plat_psci_ops_t sunxi_native_psci_ops = {
	.pwr_domain_on			= sunxi_pwr_domain_on,
	.pwr_domain_off			= sunxi_pwr_domain_off,
	.pwr_domain_on_finish		= sunxi_pwr_domain_on_finish,
	.validate_power_state		= sunxi_validate_power_state,
	.pwr_domain_suspend		= sunxi_pwr_domain_suspend,
	.pwr_domain_suspend_finish	= sunxi_pwr_domain_suspend_finish,
	.pwr_domain_pwr_down_wfi	= sunxi_pwr_domain_pwr_down_wfi,
	.get_sys_suspend_power_state	= sunxi_get_sys_suspend_power_state,
	.system_off			= sunxi_system_off,
	.system_reset			= sunxi_system_reset,
	.validate_ns_entrypoint		= sunxi_validate_ns_entrypoint,
};

void sunxi_set_native_psci_ops(const plat_psci_ops_t **psci_ops)
{
	*psci_ops = &sunxi_native_psci_ops;
}
