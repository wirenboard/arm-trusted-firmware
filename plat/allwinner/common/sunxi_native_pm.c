/*
 * Copyright (c) 2017-2021, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <arch_helpers.h>
#include <common/debug.h>
#include <drivers/arm/gicv2.h>
#include <drivers/delay_timer.h>
#include <lib/mmio.h>
#include <lib/psci/psci.h>

#include <sunxi_cpucfg.h>
#include <sunxi_mmap.h>
#include <sunxi_private.h>

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
		unsigned int beats = 0;

		NOTICE("PSCI: GICD_CTLR=%x ISEN0=%x ISEN4=%x IGRP4=%x PRIO136=%x TGT136=%x GICC_CTLR=%x PMR=%x\n",
		       mmio_read_32(SUNXI_GICD_BASE + 0x000),
		       mmio_read_32(SUNXI_GICD_BASE + 0x100),
		       mmio_read_32(SUNXI_GICD_BASE + 0x110),
		       mmio_read_32(SUNXI_GICD_BASE + 0x090),
		       mmio_read_32(SUNXI_GICD_BASE + 0x488),
		       mmio_read_32(SUNXI_GICD_BASE + 0x888),
		       mmio_read_32(SUNXI_GICC_BASE + 0x000),
		       mmio_read_32(SUNXI_GICC_BASE + 0x004));

		/*
		 * The CPUIDLE hardware was enabled when the other cores
		 * were hot-unplugged on the way into suspend (CPU_OFF).
		 * With it enabled, this core's WFI is hardware-managed and
		 * never wakes (interrupts go pending at the GIC but the
		 * gated core does not observe them — verified by polling).
		 * Disable it for the retention wait, re-enable afterwards.
		 */
		mmio_write_32(SUNXI_CPUIDLE_EN_REG, 0x16aa0000U);
		mmio_write_32(SUNXI_CPUIDLE_EN_REG, 0xaa160000U);

		/* Secure timer PPI 29 as heartbeat/backstop wake. */
		mmio_write_32(SUNXI_GICD_BASE + 0x100, BIT_32(29));

		while (read_isr_el1() == 0U && beats < 36U) {
			write_cntps_tval_el1(read_cntfrq_el0() * 5U);
			write_cntps_ctl_el1(1U);
			dsb();
			wfi();
			write_cntps_ctl_el1(0U);
			beats++;
		}

		NOTICE("PSCI: resuming after %u wfi wakes, ISR=%lx\n",
		       beats, read_isr_el1());

		/* Heartbeat PPI off again. */
		mmio_write_32(SUNXI_GICD_BASE + 0x180, BIT_32(29));

		/* Re-enable the CPUIDLE hardware (CPU_ON/OFF rely on it). */
		mmio_write_32(SUNXI_CPUIDLE_EN_REG, 0x16aa0000U);
		mmio_write_32(SUNXI_CPUIDLE_EN_REG, 0xaa160001U);

		/*
		 * DEBUG breadcrumb, readable from Linux after resume even
		 * when the serial console capture is unavailable:
		 * RTC data reg 3 @ 0x0700010c =
		 *   [31:24] poll count  [23:16] HPPIR low byte
		 *   [15:8]  PEND4 bits 15:8 (bit8 = RTC SPI 136)
		 *   [7:0]   ISR_EL1 low byte
		 */
		mmio_write_32(0x0700010cU,
			      ((beats & 0xffU) << 24) |
			      ((mmio_read_32(SUNXI_GICC_BASE + 0x018) & 0xffU) << 16) |
			      (((mmio_read_32(SUNXI_GICD_BASE + 0x210) >> 8) & 0xffU) << 8) |
			      (read_isr_el1() & 0xffU));

		disable_mmu_el3();
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
