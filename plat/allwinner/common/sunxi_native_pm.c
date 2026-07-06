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
	int	 off_resume;
	int	 crc;		/* verify variant: run the DRAM CRC bracket */
	uint8_t	 out_ctrl1, out_ctrl2, dcdc2_v;
} sus;

/*
 * SoC register-file snapshot for suspend-to-off: VDD-SYS dies, so the
 * CCU and the pin controllers lose everything the kernel configured
 * and will not re-program on resume (pinctrl-sunxi and the clk
 * framework both treat hardware state as retained). The buffers live
 * in BL31 .bss — DRAM, preserved by self-refresh.
 */
/*
 * DRAM retention checksum bracket for suspend-to-off: one sum per
 * 256 MiB chunk, computed after the kernel is quiesced (full CPU
 * speed, before the 24 MHz switch) and recomputed at warm re-entry
 * before the kernel resumes. Chunk 0 skips the first 2 MiB (BL31
 * itself lives and works there); coverage ends at 3 GiB (32-bit VA).
 */
#define DRAM_SUM_CHUNK		0x10000000ULL
#define DRAM_SUM_CHUNKS		12U
static uint64_t dram_sums[DRAM_SUM_CHUNKS];

static uint64_t sunxi_dram_sum_range(uintptr_t base, size_t len)
{
	const uint64_t *p = (const uint64_t *)base;
	const uint64_t *end = (const uint64_t *)(base + len);
	uint64_t acc = 0;

	while (p < end) {
		acc ^= *p++;
		acc = (acc << 13) | (acc >> 51);
	}
	return acc;
}

static void sunxi_dram_sums_compute(uint64_t *out)
{
	uint64_t t0 = read_cntpct_el0();
	uint32_t i;

	for (i = 0U; i < DRAM_SUM_CHUNKS; i++) {
		uintptr_t base = SUNXI_DRAM_BASE + i * DRAM_SUM_CHUNK;
		size_t len = DRAM_SUM_CHUNK;

		if (i == 0U) {
			base += 0x200000U;
			len -= 0x200000U;
		}
		out[i] = sunxi_dram_sum_range(base, len);
	}
	NOTICE("PSCI: DRAM sums in %llu ms:\n",
	       (read_cntpct_el0() - t0) / 24000ULL);
	for (i = 0U; i < DRAM_SUM_CHUNKS; i += 4U) {
		NOTICE("  %u: %lx %lx %lx %lx\n", i,
		       out[i], out[i + 1U], out[i + 2U], out[i + 3U]);
	}
}

static uint32_t soc_ccu[0x1000 / 4];
/* SPI1 (the EC link, watchdog-critical): the controller resets to
 * SLAVE mode with VDD-SYS and the sun6i driver only re-programs it in
 * runtime-PM resume, which system resume does not re-run. */
static uint32_t soc_spi1[0x40 / 4];
static uint32_t soc_pio[0x300 / 4];
static uint32_t soc_rpio[0x60 / 4];

/*
 * GIC-400 distributor state: the kernel's GIC driver assumes the
 * distributor is retained across "deep" suspend; after suspend-to-off
 * it comes back with every SPI disabled and unrouted, so all
 * IRQ-driven peripherals silently die. 10 words cover 320 interrupt
 * lines — more than the H616 has.
 */
#define GICD_WORDS	10U
static uint32_t gicd_ctlr;
static uint32_t gicd_igroup[GICD_WORDS];
static uint32_t gicd_isenable[GICD_WORDS];
static uint32_t gicd_ipriority[GICD_WORDS * 8];
static uint32_t gicd_itarget[GICD_WORDS * 8];
static uint32_t gicd_icfg[GICD_WORDS * 2];

static void sunxi_gicd_state_save(void)
{
	uint32_t i;

	gicd_ctlr = mmio_read_32(SUNXI_GICD_BASE + 0x000U);
	for (i = 0U; i < GICD_WORDS; i++) {
		gicd_igroup[i]   = mmio_read_32(SUNXI_GICD_BASE + 0x080U + i * 4U);
		gicd_isenable[i] = mmio_read_32(SUNXI_GICD_BASE + 0x100U + i * 4U);
	}
	for (i = 0U; i < GICD_WORDS * 8U; i++) {
		gicd_ipriority[i] = mmio_read_32(SUNXI_GICD_BASE + 0x400U + i * 4U);
		gicd_itarget[i]   = mmio_read_32(SUNXI_GICD_BASE + 0x800U + i * 4U);
	}
	for (i = 0U; i < GICD_WORDS * 2U; i++)
		gicd_icfg[i] = mmio_read_32(SUNXI_GICD_BASE + 0xc00U + i * 4U);
}

static void sunxi_gicd_state_restore(void)
{
	uint32_t i;

	for (i = 0U; i < GICD_WORDS; i++)
		mmio_write_32(SUNXI_GICD_BASE + 0x080U + i * 4U, gicd_igroup[i]);
	for (i = 0U; i < GICD_WORDS * 8U; i++) {
		mmio_write_32(SUNXI_GICD_BASE + 0x400U + i * 4U, gicd_ipriority[i]);
		mmio_write_32(SUNXI_GICD_BASE + 0x800U + i * 4U, gicd_itarget[i]);
	}
	for (i = 0U; i < GICD_WORDS * 2U; i++)
		mmio_write_32(SUNXI_GICD_BASE + 0xc00U + i * 4U, gicd_icfg[i]);
	for (i = 0U; i < GICD_WORDS; i++)
		mmio_write_32(SUNXI_GICD_BASE + 0x100U + i * 4U, gicd_isenable[i]);
	mmio_write_32(SUNXI_GICD_BASE + 0x000U, gicd_ctlr);
	dsbsy();
}

static void sunxi_soc_state_save(void)
{
	uint32_t i;

	for (i = 0U; i < ARRAY_SIZE(soc_ccu); i++)
		soc_ccu[i] = mmio_read_32(SUNXI_CCU_BASE + i * 4U);
	for (i = 0U; i < ARRAY_SIZE(soc_pio); i++)
		soc_pio[i] = mmio_read_32(SUNXI_PIO_BASE + i * 4U);
	for (i = 0U; i < ARRAY_SIZE(soc_rpio); i++)
		soc_rpio[i] = mmio_read_32(SUNXI_R_PIO_BASE + i * 4U);
	for (i = 0U; i < ARRAY_SIZE(soc_spi1); i++)
		soc_spi1[i] = mmio_read_32(0x05011000U + i * 4U);
}

static void sunxi_soc_state_restore(void)
{
	/*
	 * PLL restore list. PLL_CPUX (0x000) is handled separately with
	 * the CPU parked on HOSC; PLL_DDR0 (0x010) and PLL_PERIPH0
	 * (0x020) are LIVE (DRAM and buses run on them, SPL programmed
	 * them to the same values) — rewriting a live PLL glitches its
	 * consumers fatally, so they are skipped.
	 */
	static const uint16_t plls[] = {
		0x028U, 0x030U,
		0x040U, 0x048U, 0x060U, 0x078U, 0x088U,
	};
	uint32_t i, v;

	/* 0. PLL_CPUX via the proven park-on-HOSC dance. */
	mmio_write_32(SUNXI_CCU_BASE + 0x500U,
		      mmio_read_32(SUNXI_CCU_BASE + 0x500U) & ~(0x7U << 24));
	dsbsy();
	isb();
	udelay(2);
	v = soc_ccu[0];
	if ((v & BIT_32(31)) != 0U) {
		mmio_write_32(SUNXI_CCU_BASE + 0x000U, v | BIT_32(29));
		for (uint32_t t = 0U; t < 10000U; t++) {
			if ((mmio_read_32(SUNXI_CCU_BASE + 0x000U) &
			     BIT_32(28)) != 0U)
				break;
		}
	}
	mmio_write_32(SUNXI_CCU_BASE + 0x000U, v);
	udelay(2);
	mmio_write_32(SUNXI_CCU_BASE + 0x500U, soc_ccu[0x500U / 4U]);
	dsbsy();
	isb();

	/* 1. Remaining PLLs: enable with forced lock detect, wait,
	 * then drop back to the saved value. */
	for (i = 0U; i < ARRAY_SIZE(plls); i++) {
		v = soc_ccu[plls[i] / 4U];
		if ((v & BIT_32(31)) == 0U) {
			mmio_write_32(SUNXI_CCU_BASE + plls[i], v);
			continue;
		}
		mmio_write_32(SUNXI_CCU_BASE + plls[i], v | BIT_32(29));
		for (uint32_t t = 0U; t < 10000U; t++) {
			if ((mmio_read_32(SUNXI_CCU_BASE + plls[i]) &
			     BIT_32(28)) != 0U)
				break;
		}
		mmio_write_32(SUNXI_CCU_BASE + plls[i], v);
	}
	udelay(20);

	/* 2. Everything else in the CCU: dividers, muxes, gates and
	 * resets, in address order (gates/resets come after their
	 * mux/divider registers within each peripheral's group). */
	for (i = 0x504U / 4U; i < ARRAY_SIZE(soc_ccu); i++) {
		/* DRAM/MBUS clock registers: SPL has already configured
		 * the live controller — rewriting them (SDRCLK update
		 * bits) would glitch the running DRAM. */
		if (i >= 0x800U / 4U && i < 0x810U / 4U)
			continue;
		mmio_write_32(SUNXI_CCU_BASE + i * 4U, soc_ccu[i]);
	}
	udelay(10);

	/* 3. Pin controllers, now that their clocks are back. */
	for (i = 0U; i < ARRAY_SIZE(soc_pio); i++)
		mmio_write_32(SUNXI_PIO_BASE + i * 4U, soc_pio[i]);
	for (i = 0U; i < ARRAY_SIZE(soc_rpio); i++)
		mmio_write_32(SUNXI_R_PIO_BASE + i * 4U, soc_rpio[i]);

	/* SPI1 controller: GCR (master mode!), clock, format, wait
	 * cycles, IRQ enables. Status/FIFO registers are skipped.
	 *
	 * DISABLED for kernel-PM validation: the spi-sun6i system-sleep
	 * PM ops (pm_runtime_force_suspend/resume) now reprogram the
	 * controller (GCR master mode etc.) on the kernel side, so this
	 * firmware restore is redundant. Kept #if 0 (not deleted) so it
	 * can be restored instantly if the kernel path proves insufficient. */
#if 0
	mmio_write_32(0x05011004U, soc_spi1[0x04U / 4U]);
	mmio_write_32(0x05011024U, soc_spi1[0x24U / 4U]);
	mmio_write_32(0x05011008U, soc_spi1[0x08U / 4U]);
	mmio_write_32(0x05011020U, soc_spi1[0x20U / 4U]);
	mmio_write_32(0x05011018U, soc_spi1[0x18U / 4U]);
	mmio_write_32(0x05011010U, soc_spi1[0x10U / 4U]);
#endif
	dsbsy();
}

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
	/*
	 * Suspend-to-off: checksum the retained image here — the
	 * caches are still on (pwr_down_wfi runs after the generic
	 * code disables them: 3 GiB takes 2 s cached vs 74 s not).
	 * Only the verify variant (magic 0x0ff51eeb) computes it; the
	 * fast default (0x0ff51eea) skips the ~74 s bracket. This runs
	 * before the magic is cleared in pwr_down_wfi, so reading it
	 * here is fine.
	 */
	if (mmio_read_32(0x07000100U) == 0x0ff51eebU) {
		sunxi_dram_sums_compute(dram_sums);
	}

	NOTICE("PSCI: System suspend: entering WFI retention\n");
}

static void sunxi_pwr_domain_suspend_finish(const psci_power_state_t *target_state)
{
	/*
	 * After suspend-to-off the GIC lost power with VDD-SYS: bring
	 * the distributor back up before the per-cpu parts. The kernel
	 * re-applies its own interrupt configuration in its GIC
	 * syscore resume.
	 */
	if (sus.off_resume != 0) {
		sus.off_resume = 0;
		NOTICE("off-resume pre: ctlr=%x en1=%x en2=%x spi1clk=%x spi1bgr=%x\n",
		       mmio_read_32(SUNXI_GICD_BASE + 0x000U),
		       mmio_read_32(SUNXI_GICD_BASE + 0x104U),
		       mmio_read_32(SUNXI_GICD_BASE + 0x108U),
		       mmio_read_32(SUNXI_CCU_BASE + 0x944U),
		       mmio_read_32(SUNXI_CCU_BASE + 0x96cU));
		sunxi_soc_state_restore();
		gicv2_distif_init();
		sunxi_gicd_state_restore();

		/* Kernel-configured CPU clock back (also speeds up the
		 * checksum verify below ~3x over the SPL clock). */
		sunxi_suspend_cpu_fast();

		/*
		 * Verify variant only (sus.crc, armed from magic
		 * 0x0ff51eeb). The magic at 0x07000100 is already 0 by
		 * resume time — it is cleared in pwr_down_wfi — so the
		 * gate MUST be the persisted flag, never a re-read.
		 */
		if (sus.crc != 0) {
			static uint64_t verify[DRAM_SUM_CHUNKS];
			uint32_t i, bad = 0U;

			sus.crc = 0;
			sunxi_dram_sums_compute(verify);
			for (i = 0U; i < DRAM_SUM_CHUNKS; i++) {
				if (verify[i] != dram_sums[i]) {
					bad++;
					NOTICE("PSCI: DRAM sum MISMATCH chunk %u: %lx != %lx\n",
					       i, verify[i], dram_sums[i]);
				}
			}
			NOTICE("PSCI: DRAM retention verdict: %s (%u/%u chunks bad)\n",
			       (bad == 0U) ? "CLEAN" : "CORRUPTED", bad,
			       DRAM_SUM_CHUNKS);
		}
		NOTICE("off-resume post: ctlr=%x en1=%x en2=%x spi1clk=%x spi1bgr=%x saved_en1=%x\n",
		       mmio_read_32(SUNXI_GICD_BASE + 0x000U),
		       mmio_read_32(SUNXI_GICD_BASE + 0x104U),
		       mmio_read_32(SUNXI_GICD_BASE + 0x108U),
		       mmio_read_32(SUNXI_CCU_BASE + 0x944U),
		       mmio_read_32(SUNXI_CCU_BASE + 0x96cU),
		       gicd_isenable[1]);
		/*
		 * The PIO block lost its state with VDD-SYS; the sus
		 * snapshot survived in DRAM — bring the carrier rails
		 * and PHY pins back to their pre-suspend configuration.
		 */
		sunxi_suspend_periph_restore();
	}

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
			uint32_t magic = mmio_read_32(0x07000100U);

			if (magic == 0x0ff51ee9U) {
				mmio_write_32(0x07000100U, 0U);
				kill = 0x10U;	/* keep DCDC5 only */
				NOTICE("PSCI: suspend: one-way rail kill armed\n");
			} else if (magic == 0x0ff51eeaU ||
				   magic == 0x0ff51eebU) {
				/*
				 * 0x0ff51eea = suspend-to-off, fast (no CRC).
				 * 0x0ff51eeb = suspend-to-off, with the DRAM
				 * CRC bracket (sus.crc set below). Both take
				 * the identical suspend-to-off path.
				 */
				/*
				 * Suspend-to-off: the PMIC sleeps with only the
				 * DRAM rails alive, the EC wakes it by PWRON at
				 * the RTC alarm, and the SoC boots through SPL,
				 * which finds the resume vector in RTC data1
				 * and jumps back into this (DRAM-resident,
				 * self-refresh-preserved) BL31 instead of
				 * loading U-Boot.
				 *
				 * PMIC preparation: enable POK-negedge as a
				 * sleep wake source (REG41 bit3), then REG31
				 * bit3 = "start sleep, record REG10/11/12" —
				 * the rail cut the blob performs next is what
				 * wake undoes.
				 */
				uint8_t v;

				mmio_write_32(0x07000100U, 0U);
				/*
				 * The sleep-wake restores the rail enables
				 * RECORDED at the REG31 write. The trim path
				 * above already cut DCDC1/DCDC4 and lowered
				 * VDD-CPU — recording that would leave 3V3
				 * dead forever after wake. Put the runtime
				 * configuration back first (a millisecond
				 * rail bounce, harmless), record THAT, then
				 * let the blob kill everything.
				 */
				if (sus.pmic_ok != 0) {
					axp_wr(AXP_REG_OUT_CTRL1, sus.out_ctrl1);
					axp_wr(AXP_REG_OUT_CTRL2, sus.out_ctrl2);
					axp_wr(AXP_REG_DCDC2_V, sus.dcdc2_v);
					sunxi_soc_state_save();
					sunxi_gicd_state_save();
				}
				/*
				 * Only the POK (PWRON) negative edge may wake
				 * the sleeping PMIC: mask every other IRQ
				 * enable and ack all pending statuses, or the
				 * PMIC's own IRQ line (rail-off events, stale
				 * POK edges) wakes it right back up through
				 * the global REG1F[7] gate.
				 */
				if (sus.pmic_ok != 0 &&
				    axp_wr(0x40U, 0x00U) == 0 &&	/* INTEN1 off */
				    axp_wr(0x41U, 0x08U) == 0 &&	/* only POK negedge */
				    axp_wr(0x48U, 0xffU) == 0 &&	/* ack INTSTS1 */
				    axp_wr(0x49U, 0xffU) == 0 &&	/* ack INTSTS2 */
				    axp_rd(0x1fU, &v) == 0 &&
				    axp_wr(0x1fU, v | 0x80U) == 0 &&	/* global IRQ wakeup en */
				    axp_rd(0x31U, &v) == 0 &&
				    axp_wr(0x31U, v | 0x08U) == 0) {	/* record + sleep */
					kill = 0x10U;	/* keep DCDC5 only */
					sus.off_resume = 1;
					sus.crc = (magic == 0x0ff51eebU) ? 1 : 0;
					mmio_write_32(0x07000104U,
						      (uint32_t)sunxi_sec_entrypoint);
					dsbsy();
					NOTICE("PSCI: suspend-to-off armed, resume via 0x%x\n",
					       (uint32_t)sunxi_sec_entrypoint);
				} else {
					WARN("PSCI: suspend-to-off: PMIC prep failed, normal suspend\n");
				}
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
