/* SPDX-License-Identifier: MIT */

/*
 * Dialog Semiconductor "baku" PMU register map for Apple t8140 (A18 Pro, Mac17G)
 *
 * The baku PMU is a Dialog Semiconductor PMIC accessed via SPMI at:
 *   Bus:   nub-spmi0  (MMIO base 0xF8714000)
 *   Addr:  0xE        (SPMI slave address)
 *
 * This header was derived by reverse-engineering the AppleDialogSPMIPMU kext
 * from the mac17g kernelcache (kernelcache.mac17g.bin, 121MB MH_FILESET arm64e)
 * and decoding live ADT properties via ioreg on Mac17,5 running macOS 26.3.2.
 *
 * Register addresses are 16-bit SPMI extended (EXT_READL/EXT_WRITEL).
 *
 * ADT property → object field → SPMI register address mapping:
 *   info-scrpad[0]         → obj+0x92  → BAKU_SCRPAD_BASE     = 0x8000
 *   info-scrpad[4]         → obj+0x94  → (BAKU_SCRPAD_SIZE      = 0x1000 bytes)
 *   info-leg_scrpad        → obj+0x98  → BAKU_LEG_SCRPAD       = 0xF700
 *   info-fault_log[0]      → obj+0x9a  → BAKU_FAULT_LOG_BASE   = 0x1000
 *   info-fault_log[4]      → obj+0x9c  → (BAKU_FAULT_LOG_COUNT = 5)
 *   info-off2wake_source[0]→ obj+0x9e  → BAKU_OFF2WAKE_BASE    = 0x2413
 *   info-off2wake_source[4]→ obj+0xa0  → (BAKU_OFF2WAKE_COUNT  = 2)
 *   info-scrpad_lpm_log    → obj+0xa2  → BAKU_LPM_LOG_BASE     = 0x8F80
 *   info-scrpad_lpm_ctrl   → obj+0xa4  → BAKU_LPM_CTRL_BASE    = 0x8FDC
 *   info-scrpad_socd[0]    → obj+0xa6  → BAKU_SOCD_BASE        = 0x8800
 *   info-pm_setting        → obj+0xa8  → BAKU_PM_SETTING       = 0xF801
 *   info-ulpm_enable_offset→ obj+0xaa  → NOT present (BAKU_ULPM_EN_OFF  = 0)
 *   info-ulpm_enable_mask  → obj+0xac  → NOT present (BAKU_ULPM_EN_MASK = 0)
 *   info-fw-sram           → obj+0xae  → NOT present (_sramAddrLoReg    = 0)
 *   info-fw-sram           → obj+0xb0  → NOT present (_sramAddrHiReg    = 0)
 *   info-fw-sram           → obj+0xb2  → NOT present (_sramWrBaseReg    = 0)
 *   info-fw-sram           → obj+0xb4  → NOT present (_sramRdBaseReg    = 0)
 *   info-fw-sram           → obj+0xb6  → NOT present (_sramPageSize     = 0)
 *   info-scrpad_socd[4]    → obj+0xb8  → (BAKU_SOCD_COUNT      = 0x700)
 *   info-has_lpem          → NOT present (BAKU_HAS_LPEM = 0)
 *   info-has_slpsmc        → present, value=1 (BAKU_HAS_SLPSMC = 1)
 *   info-has_sysrst        → NOT present (BAKU_HAS_SYSRST = 0)
 *   info-has_aapf          → NOT present (BAKU_HAS_AAPF = 0)
 */

#ifndef BAKU_PMU_H
#define BAKU_PMU_H

/*
 * SPMI bus and device address
 */
#define BAKU_SPMI_NODE  "/arm-io/nub-spmi0"
#define BAKU_SPMI_ADDR  0xE

/*
 * Scratchpad registers (always accessible, no firmware unlock needed)
 */
#define BAKU_SCRPAD_BASE     0x8000  /* info-scrpad[0]: base of 0x1000-byte scratchpad */
#define BAKU_SCRPAD_SIZE     0x1000  /* info-scrpad[4]: size in bytes */
#define BAKU_LEG_SCRPAD      0xF700  /* info-leg_scrpad: legacy scratchpad (panic counters) */

/*
 * Fault log registers
 */
#define BAKU_FAULT_LOG_BASE  0x1000  /* info-fault_log[0]: base of fault log area */
#define BAKU_FAULT_LOG_COUNT 5       /* info-fault_log[4]: number of log entries */

/*
 * Off-to-wake source registers
 */
#define BAKU_OFF2WAKE_BASE   0x2413  /* info-off2wake_source[0]: off-to-wake source reg */
#define BAKU_OFF2WAKE_COUNT  2       /* info-off2wake_source[4]: number of source bytes */

/*
 * LPM (Low Power Mode) control/log registers
 * Used by the SLPSMC (Sleep SMC) feature, which IS enabled on mac17g.
 * The SLPSMC init sequence reads/writes these registers to coordinate with
 * the Dialog PMU firmware's low-power state machine.
 */
#define BAKU_LPM_LOG_BASE    0x8F80  /* info-scrpad_lpm_log: LPM event log base */
#define BAKU_LPM_CTRL_BASE   0x8FDC  /* info-scrpad_lpm_ctrl: LPM control register */

/*
 * SOCD (System-On-Chip Debug?) scratchpad registers
 */
#define BAKU_SOCD_BASE       0x8800  /* info-scrpad_socd[0]: SOCD scratchpad base */
#define BAKU_SOCD_COUNT      0x700   /* info-scrpad_socd[4]: SOCD region size in bytes */

/*
 * PM settings register
 * This is the main "is the PMU alive?" health register.
 * A successful EXT_READL from 0xF801 confirms the PMU SPMI link is active.
 */
#define BAKU_PM_SETTING      0xF801  /* info-pm_setting: PMU power management settings */

/*
 * SRAM access registers - NOT PRESENT on mac17g (info-fw-sram absent in ADT)
 * These would be used to access the Dialog PMU's internal firmware SRAM.
 * Without these, _writeMem() and _readMem() are no-ops.
 */
/* #define BAKU_SRAM_ADDR_LO  (not present) */
/* #define BAKU_SRAM_ADDR_HI  (not present) */
/* #define BAKU_SRAM_WR_BASE  (not present) */
/* #define BAKU_SRAM_RD_BASE  (not present) */

/*
 * Feature flags (from ADT, specific to mac17g / t8140 / A18 Pro)
 * LPEM  = Low Power Event Manager
 * SLPSMC= Sleep SMC (power-domain coordination with pmgr) — ENABLED
 * SYSRST= System Reset capability
 * AAPF  = (unknown, likely Apple-specific power feature)
 */
#define BAKU_HAS_LPEM    0  /* info-has_lpem absent in mac17g ADT */
#define BAKU_HAS_SLPSMC  1  /* info-has_slpsmc = 1: SLPSMC is active */
#define BAKU_HAS_SYSRST  0  /* info-has_sysrst absent in mac17g ADT */
#define BAKU_HAS_AAPF    0  /* info-has_aapf absent in mac17g ADT */

/*
 * ptmu-region power domain map (SPMI 0x6000 - 0x6FFF)
 *
 * These 16 regions cover the Dialog PMU's internal power domain control space.
 * They are managed by the PMU FIRMWARE, not directly by the OS driver.
 * Access from the host (EXT_READL) to these addresses NAKs until the PMU
 * firmware has completed its initialization sequence.
 *
 * iBoot starts the PMU firmware before handoff, so these regions should be
 * accessible after a successful SPMI WAKEUP if iBoot ran normally.
 *
 * Layout (from pmu-main@E ADT ptmu-region-N-data properties):
 *   Region 0:  SPMI 0x6000 - 0x603F  (0x40 bytes)
 *   Region 1:  SPMI 0x6040 - 0x607F  (0x40 bytes)
 *   Region 2:  SPMI 0x6080 - 0x60BF  (0x40 bytes)
 *   Region 3:  SPMI 0x60C0 - 0x60FF  (0x40 bytes)
 *   Region 4:  SPMI 0x6100 - 0x613F  (0x40 bytes)
 *   Region 5:  SPMI 0x6140 - 0x617F  (0x40 bytes)
 *   Region 6:  SPMI 0x6180 - 0x61BF  (0x40 bytes)
 *   Region 7:  SPMI 0x61C0 - 0x61FF  (0x40 bytes)
 *   Region 8:  SPMI 0x6200 - 0x627F  (0x80 bytes)
 *   Region 9:  SPMI 0x6280 - 0x62FF  (0x80 bytes)
 *   Region 10: SPMI 0x6300 - 0x637F  (0x80 bytes)
 *   Region 11: SPMI 0x6380 - 0x63FF  (0x80 bytes)
 *   Region 12: SPMI 0x6400 - 0x65FF  (0x200 bytes)
 *   Region 13: SPMI 0x6600 - 0x67FF  (0x200 bytes)
 *   Region 14: SPMI 0x6800 - 0x6BFF  (0x400 bytes)
 *   Region 15: SPMI 0x6C00 - 0x6FFF  (0x400 bytes)
 *
 * ATC0_USB_AON power control is somewhere within 0x6000-0x6FFF.
 * The specific bit/register is unknown; it is managed by the PMU firmware
 * and is NOT directly accessible from the host SPMI bus under normal macOS.
 */
#define BAKU_PTMU_BASE   0x6000
#define BAKU_PTMU_END    0x6FFF

#endif /* BAKU_PMU_H */
