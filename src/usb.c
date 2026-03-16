/* SPDX-License-Identifier: MIT */

#include "usb.h"
#include "adt.h"
#include "baku_pmu.h"
#include "dart.h"
#include "i2c.h"
#include "iodev.h"
#include "malloc.h"
#include "pmgr.h"
#include "spmi.h"
#include "string.h"
#include "tps6598x.h"
#include "types.h"
#include "usb_dwc3.h"
#include "usb_dwc3_regs.h"
#include "utils.h"
#include "vsprintf.h"

struct usb_drd_regs {
    uintptr_t drd_regs;
    uintptr_t drd_regs_unk3;
    uintptr_t atc;
};

#if USB_IODEV_COUNT > 100
#error "USB_IODEV_COUNT is limited to 100 to prevent overflow in ADT path names"
#endif

// length of the format string is is used as buffer size
// limits the USB instance numbers to reasonable 2 digits
#define FMT_DART_PATH        "/arm-io/dart-usb%u"
#define FMT_DART_MAPPER_PATH "/arm-io/dart-usb%u/mapper-usb%u"
#define FMT_ATC_PATH         "/arm-io/atc-phy%u"
#define FMT_DRD_PATH         "/arm-io/usb-drd%u"
// HPM_PATH string is at most
// "/arm-io/i2cX" (12) + "/" + hpmBusManagerX (14) + "/" + "hpmX" (4) + '\0'
#define MAX_HPM_PATH_LEN 40

static tps6598x_irq_state_t tps6598x_irq_state[USB_IODEV_COUNT];
static bool usb_is_initialized = false;

#define PIPEHANDLER_MUX_CTRL             0x0c
#define PIPEHANDLER_MUX_CTRL_USB3        0x08
#define PIPEHANDLER_MUX_CTRL_USB4_TUNNEL 0x11
#define PIPEHANDLER_MUX_CTRL_DUMMY       0x22

#define PIPEHANDLER_LOCK_REQ 0x10
#define PIPEHANDLER_LOCK_ACK 0x14
#define PIPEHANDLER_LOCK_EN  BIT(0)

#define PIPEHANDLER_AON_GEN                     0x1C
#define PIPEHANDLER_AON_GEN_DWC3_FORCE_CLAMP_EN BIT(4)
#define PIPEHANDLER_AON_GEN_DWC3_RESET_N        BIT(0)

#define PIPEHANDLER_NONSELECTED_OVERRIDE 0x20
#define PIPEHANDLER_NATIVE_RESET         BIT(12)
#define PIPEHANDLER_DUMMY_PHY_EN         BIT(15)
#define PIPEHANDLER_NATIVE_POWER_DOWN    GENMASK(3, 0)

static dart_dev_t *usb_dart_init(u32 idx)
{
    int mapper_offset;
    char path[sizeof(FMT_DART_MAPPER_PATH)];
    bool indexed = true;

    snprintf(path, sizeof(path), FMT_DART_MAPPER_PATH, idx, idx);
    mapper_offset = adt_path_offset(adt, path);
    if (mapper_offset < 0 && idx == 0) {
        // t8140 (A18 Pro) uses un-indexed node names: dart-usb, mapper-usb
        strncpy(path, "/arm-io/dart-usb/mapper-usb", sizeof(path) - 1);
        path[sizeof(path) - 1] = '\0';
        mapper_offset = adt_path_offset(adt, path);
        if (mapper_offset >= 0)
            ;
        indexed = false;
    }
    if (mapper_offset < 0)
        return NULL;

    u32 dart_idx;
    if (ADT_GETPROP(adt, mapper_offset, "reg", &dart_idx) < 0) {
        printf("usb: Error getting DART %s device index\n", path);
        return NULL;
    }

    if (indexed)
        snprintf(path, sizeof(path), FMT_DART_PATH, idx);
    else {
        strncpy(path, "/arm-io/dart-usb", sizeof(path) - 1);
        path[sizeof(path) - 1] = '\0';
    }
    return dart_init_adt(path, 1, dart_idx, false);
}

static int usb_drd_get_regs(u32 idx, struct usb_drd_regs *regs)
{
    int adt_drd_path[8];
    int adt_drd_offset;
    int adt_phy_path[8];
    int adt_phy_offset;
    char phy_path[sizeof(FMT_ATC_PATH)];
    char drd_path[sizeof(FMT_DRD_PATH)];

    snprintf(drd_path, sizeof(drd_path), FMT_DRD_PATH, idx);
    adt_drd_offset = adt_path_offset_trace(adt, drd_path, adt_drd_path);
    if (adt_drd_offset < 0 && idx == 0) {
        // t8140 (A18 Pro) uses un-indexed node names: usb-drd, dart-usb
        strncpy(drd_path, "/arm-io/usb-drd", sizeof(drd_path) - 1);
        drd_path[sizeof(drd_path) - 1] = '\0';
        adt_drd_offset = adt_path_offset_trace(adt, drd_path, adt_drd_path);
        if (adt_drd_offset >= 0)
            ;
    }
    if (adt_drd_offset < 0) {
        // Nonexistent device
        return -1;
    }

    snprintf(phy_path, sizeof(phy_path), FMT_ATC_PATH, idx);
    adt_phy_offset = adt_path_offset_trace(adt, phy_path, adt_phy_path);
    if (adt_phy_offset < 0) {
        printf("usb: Error getting phy node %s\n", phy_path);
        return -1;
    }

    if (adt_get_reg(adt, adt_phy_path, "reg", 0, &regs->atc, NULL) < 0) {
        printf("usb: Error getting reg with index 0 for %s.\n", phy_path);
        return -1;
    }
    if (adt_get_reg(adt, adt_drd_path, "reg", 0, &regs->drd_regs, NULL) < 0) {
        printf("usb: Error getting reg with index 0 for %s.\n", drd_path);
        return -1;
    }
    if (adt_get_reg(adt, adt_drd_path, "reg", 3, &regs->drd_regs_unk3, NULL) < 0) {
        printf("usb: FAIL reg[3] missing for %s (A18 Pro may have fewer regs)\n", drd_path);
        return -1;
    }

    return 0;
}

int usb_phy_bringup(u32 idx)
{
    char path[24];

    if (idx >= USB_IODEV_COUNT)
        return -1;

    struct usb_drd_regs usb_regs;
    if (usb_drd_get_regs(idx, &usb_regs) < 0)
        return -1;

    snprintf(path, sizeof(path), FMT_ATC_PATH, idx);
    if (pmgr_adt_power_enable(path) < 0)
        printf("usb: pmgr_adt_power_enable(%s) failed (AON domain may already be on) — continuing\n",
               path);

    // dart-usb: on t8140 the node is un-indexed. Its clock-gates include ATC0_USB_AON
    // which always times out, failing the pmgr call even though iBoot already left the
    // dart powered. Try indexed then un-indexed; treat failure as non-fatal since the
    // hardware may already be on.
    snprintf(path, sizeof(path), FMT_DART_PATH, idx);
    if (pmgr_adt_power_enable(path) < 0) {
        if (idx == 0) {
            if (pmgr_adt_power_enable("/arm-io/dart-usb") < 0)
                printf("usb: dart pmgr failed (ATC0_USB_AON timeout expected on t8140) — continuing\n");
        } else {
            printf("usb: pmgr_adt_power_enable(%s) failed\n", path);
            return -1;
        }
    }

    // usb-drd: same ATC0_USB_AON timeout issue. Try indexed then un-indexed and
    // continue — partial pmgr success may still enable the DWC3 clock gate.
    snprintf(path, sizeof(path), FMT_DRD_PATH, idx);
    if (pmgr_adt_power_enable(path) < 0) {
        if (idx == 0) {
            if (pmgr_adt_power_enable("/arm-io/usb-drd") < 0)
                printf("usb: drd pmgr failed (ATC0_USB_AON timeout expected on t8140) — continuing\n");
        } else {
            printf("usb: pmgr_adt_power_enable(%s) failed\n", path);
            return -1;
        }
    }

    write32(usb_regs.atc + 0x08, 0x01c1000f);
    write32(usb_regs.atc + 0x04, 0x00000003);
    write32(usb_regs.atc + 0x04, 0x00000000);
    write32(usb_regs.atc + 0x1c, 0x008c0813);
    write32(usb_regs.atc + 0x00, 0x00000002);

    write32(usb_regs.drd_regs_unk3 + PIPEHANDLER_MUX_CTRL, PIPEHANDLER_MUX_CTRL_DUMMY);
    write32(usb_regs.drd_regs_unk3 + PIPEHANDLER_AON_GEN, PIPEHANDLER_AON_GEN_DWC3_RESET_N);
    write32(usb_regs.drd_regs_unk3 + PIPEHANDLER_NONSELECTED_OVERRIDE, 0x9332);

    return 0;
}

dwc3_dev_t *usb_iodev_bringup(u32 idx)
{
    dart_dev_t *usb_dart = usb_dart_init(idx);
    if (!usb_dart)
        return NULL;

    struct usb_drd_regs usb_reg;
    if (usb_drd_get_regs(idx, &usb_reg) < 0)
        return NULL;

    return usb_dwc3_init(usb_reg.drd_regs, usb_dart);
}

#define USB_IODEV_WRAPPER(name, pipe)                                                              \
    static ssize_t usb_##name##_can_read(void *dev)                                                \
    {                                                                                              \
        return usb_dwc3_can_read(dev, pipe);                                                       \
    }                                                                                              \
                                                                                                   \
    static bool usb_##name##_can_write(void *dev)                                                  \
    {                                                                                              \
        return usb_dwc3_can_write(dev, pipe);                                                      \
    }                                                                                              \
                                                                                                   \
    static ssize_t usb_##name##_read(void *dev, void *buf, size_t count)                           \
    {                                                                                              \
        return usb_dwc3_read(dev, pipe, buf, count);                                               \
    }                                                                                              \
                                                                                                   \
    static ssize_t usb_##name##_write(void *dev, const void *buf, size_t count)                    \
    {                                                                                              \
        return usb_dwc3_write(dev, pipe, buf, count);                                              \
    }                                                                                              \
                                                                                                   \
    static ssize_t usb_##name##_queue(void *dev, const void *buf, size_t count)                    \
    {                                                                                              \
        return usb_dwc3_queue(dev, pipe, buf, count);                                              \
    }                                                                                              \
                                                                                                   \
    static void usb_##name##_handle_events(void *dev)                                              \
    {                                                                                              \
        usb_dwc3_handle_events(dev);                                                               \
    }                                                                                              \
                                                                                                   \
    static void usb_##name##_flush(void *dev)                                                      \
    {                                                                                              \
        usb_dwc3_flush(dev, pipe);                                                                 \
    }

USB_IODEV_WRAPPER(0, CDC_ACM_PIPE_0)
USB_IODEV_WRAPPER(1, CDC_ACM_PIPE_1)

static struct iodev_ops iodev_usb_ops = {
    .can_read = usb_0_can_read,
    .can_write = usb_0_can_write,
    .read = usb_0_read,
    .write = usb_0_write,
    .queue = usb_0_queue,
    .flush = usb_0_flush,
    .handle_events = usb_0_handle_events,
};

static struct iodev_ops iodev_usb_sec_ops = {
    .can_read = usb_1_can_read,
    .can_write = usb_1_can_write,
    .read = usb_1_read,
    .write = usb_1_write,
    .queue = usb_1_queue,
    .flush = usb_1_flush,
    .handle_events = usb_1_handle_events,
};

struct iodev iodev_usb_vuart = {
    .ops = &iodev_usb_sec_ops,
    .usage = 0,
    .lock = SPINLOCK_INIT,
};

static tps6598x_dev_t *hpm_init(i2c_dev_t *i2c, const char *hpm_path)
{
    tps6598x_dev_t *tps = tps6598x_init(hpm_path, i2c);
    if (!tps) {
        printf("usb: tps6598x_init failed for %s.\n", hpm_path);
        return NULL;
    }

    if (tps6598x_powerup(tps) < 0) {
        printf("usb: tps6598x_powerup failed for %s.\n", hpm_path);
        tps6598x_shutdown(tps);
        return NULL;
    }

    return tps;
}

void usb_spmi_init(void)
{
    // On t8140 (A18 Pro), ATC0_USB_AON is power-gated at boot and its pmgr register
    // times out because the Dialog "baku" PMU (not the USB HPM) must be active before
    // pmgr power-state transitions on this domain complete. Send WAKEUP to the Dialog
    // PMU (pmu-main, SPMI addr=0xE) on the system PMU bus (nub-spmi0@F8714000).
    // NOTE: hpm0 on nub-spmi-a0 is the TI SN2012xx USB-C controller — separate bus,
    // different function, does NOT control ATC0_USB_AON.
    //
    // See src/baku_pmu.h for the full Dialog baku PMU register map derived from
    // mac17g kernelcache AppleDialogSPMIPMU reverse engineering.
    spmi_dev_t *pmu_spmi = spmi_init(BAKU_SPMI_NODE);
    if (pmu_spmi) {
        u8 buf[8];

        printf("usb: sending SPMI wakeup to Dialog baku PMU (addr=0x%x) on %s\n",
               BAKU_SPMI_ADDR, BAKU_SPMI_NODE);
        if (spmi_send_wakeup(pmu_spmi, BAKU_SPMI_ADDR) < 0)
            printf("usb: SPMI wakeup Dialog PMU failed\n");
        else
            printf("usb: SPMI wakeup Dialog PMU ok\n");

        // Probe BAKU_PM_SETTING (0xF801) — the main PMU health register.
        // A successful read confirms the SPMI EXT_READL link to the PMU is active.
        // This register always responds; it does not require PMU firmware unlock.
        if (spmi_ext_read_long(pmu_spmi, BAKU_SPMI_ADDR, BAKU_PM_SETTING, buf, 1) == 0)
            printf("usb: PMU 0x%04x(pm_setting)=%02x — PMU SPMI link OK\n",
                   BAKU_PM_SETTING, buf[0]);
        else
            printf("usb: PMU 0x%04x(pm_setting) read FAILED — PMU not responding\n",
                   BAKU_PM_SETTING);

        // Probe BAKU_LEG_SCRPAD (0xF700) — legacy scratchpad / panic counter area.
        // This area is directly mapped and always readable without firmware involvement.
        if (spmi_ext_read_long(pmu_spmi, BAKU_SPMI_ADDR, BAKU_LEG_SCRPAD, buf, 8) == 0)
            printf("usb: PMU 0x%04x(leg_scrpad): %02x %02x %02x %02x %02x %02x %02x %02x\n",
                   BAKU_LEG_SCRPAD,
                   buf[0], buf[1], buf[2], buf[3],
                   buf[4], buf[5], buf[6], buf[7]);
        else
            printf("usb: PMU 0x%04x(leg_scrpad) read FAILED\n", BAKU_LEG_SCRPAD);

        // Probe BAKU_LPM_CTRL_BASE (0x8FDC) — LPM control register.
        // SLPSMC (Sleep SMC) is enabled on mac17g (info-has_slpsmc=1).
        // The AppleDialogSPMIPMU driver writes this register during SLPSMC init.
        // If SLPSMC init has run (i.e., iBoot ran it), this should be readable.
        if (spmi_ext_read_long(pmu_spmi, BAKU_SPMI_ADDR, BAKU_LPM_CTRL_BASE, buf, 4) == 0)
            printf("usb: PMU 0x%04x(lpm_ctrl): %02x %02x %02x %02x\n",
                   BAKU_LPM_CTRL_BASE, buf[0], buf[1], buf[2], buf[3]);
        else
            printf("usb: PMU 0x%04x(lpm_ctrl) read FAILED (SLPSMC not init'd?)\n",
                   BAKU_LPM_CTRL_BASE);

        // Probe first 8 bytes of BAKU_PTMU_BASE (0x6000) — power domain region 0.
        // These are managed by the PMU firmware. They NAK until the firmware has
        // completed its init sequence. iBoot should have run the firmware; if so
        // this will succeed and give us the current power domain state.
        if (spmi_ext_read_long(pmu_spmi, BAKU_SPMI_ADDR, BAKU_PTMU_BASE, buf, 8) == 0)
            printf("usb: PMU 0x%04x(ptmu[0]):  %02x %02x %02x %02x %02x %02x %02x %02x\n",
                   BAKU_PTMU_BASE,
                   buf[0], buf[1], buf[2], buf[3],
                   buf[4], buf[5], buf[6], buf[7]);
        else
            printf("usb: PMU 0x%04x(ptmu[0]) read FAILED (PMU fw not init'd or locked)\n",
                   BAKU_PTMU_BASE);

        spmi_shutdown(pmu_spmi);
    } else {
        printf("usb: %s init failed, continuing without Dialog PMU wakeup\n", BAKU_SPMI_NODE);
    }

    for (int idx = 0; idx < USB_IODEV_COUNT; ++idx)
        usb_phy_bringup(idx); /* Fails on missing devices, just continue */

    usb_is_initialized = true;
}

static int usb_init_i2c(const char *i2c_path)
{
    char hpm_path[MAX_HPM_PATH_LEN];

    int node = adt_path_offset(adt, i2c_path);
    if (node < 0)
        return 0;

    node = adt_first_child_offset(adt, node);
    if (node < 0)
        return 0;

    if (!adt_is_compatible(adt, node, "usbc,manager"))
        return 0;

    const char *hpm_mngr_name = adt_get_name(adt, node);
    if (!hpm_mngr_name || strnlen(hpm_mngr_name, 16) >= 16)
        return 0;

    i2c_dev_t *i2c = i2c_init(i2c_path);
    if (!i2c) {
        printf("usb: i2c init failed for %s\n", i2c_path);
        return -1;
    }

    ADT_FOREACH_CHILD(adt, node)
    {
        const char *name = adt_get_name(adt, node);
        if (!name || memcmp(name, "hpm", 3) || name[4] != '\0')
            continue; // unexpected hpm node name
        u32 idx = name[3] - '0';
        if (idx >= USB_IODEV_COUNT)
            continue; // unexpected hpm index

        snprintf(hpm_path, sizeof(hpm_path), "%s/%s/%s", i2c_path, hpm_mngr_name, name);

        tps6598x_dev_t *tps = hpm_init(i2c, hpm_path);
        if (!tps) {
            printf("usb: failed to init %s\n", name);
            continue;
        }

        if (tps6598x_disable_irqs(tps, &tps6598x_irq_state[idx]))
            printf("usb: unable to disable IRQ masks for %s\n", name);

        tps6598x_shutdown(tps);
    }

    i2c_shutdown(i2c);

    return 0;
}

void usb_init(void)
{
    if (usb_is_initialized)
        return;

    /*
     * M3/M4 models do not use i2c, but instead SPMI with a new controller.
     * We can get USB going for now by just bringing up the phys.
     */
    if (adt_path_offset(adt, "/arm-io/nub-spmi-a0/hpm0") > 0) {
        usb_spmi_init();
        return;
    }

    /*
     * A7-A11 uses a custom internal otg controller with the peripheral part
     * being dwc2.
     */
    if (adt_path_offset(adt, "/arm-io/otgphyctrl") > 0 &&
        adt_path_offset(adt, "/arm-io/usb-complex") > 0) {
        /* We do not support the custom controller and dwc2 (yet). */
        return;
    }

    if (adt_is_compatible(adt, 0, "J180dAP") && usb_init_i2c("/arm-io/i2c3") < 0)
        return;
    if (usb_init_i2c("/arm-io/i2c0") < 0)
        return;

    for (int idx = 0; idx < USB_IODEV_COUNT; ++idx)
        usb_phy_bringup(idx); /* Fails on missing devices, just continue */

    usb_is_initialized = true;
}

void usb_i2c_restore_irqs(const char *i2c_path, bool force)
{
    char hpm_path[MAX_HPM_PATH_LEN];

    int node = adt_path_offset(adt, i2c_path);
    if (node < 0)
        return;

    node = adt_first_child_offset(adt, node);
    if (node < 0)
        return;

    if (!adt_is_compatible(adt, node, "usbc,manager"))
        return;

    const char *hpm_mngr_name = adt_get_name(adt, node);
    if (!hpm_mngr_name || strnlen(hpm_mngr_name, 16) >= 16)
        return;

    i2c_dev_t *i2c = i2c_init(i2c_path);
    if (!i2c) {
        printf("usb: i2c init failed.\n");
        return;
    }

    ADT_FOREACH_CHILD(adt, node)
    {
        const char *name = adt_get_name(adt, node);
        if (!name || memcmp(name, "hpm", 3) || name[4] != '\0')
            continue; // unexpected hpm node name
        u32 idx = name[3] - '0';
        if (idx >= USB_IODEV_COUNT)
            continue; // unexpected hpm index

        if (iodev_get_usage(IODEV_USB0 + idx) && !force)
            continue;

        if (tps6598x_irq_state[idx].valid) {
            snprintf(hpm_path, sizeof(hpm_path), "%s/%s/%s", i2c_path, hpm_mngr_name, name);
            tps6598x_dev_t *tps = hpm_init(i2c, hpm_path);
            if (!tps)
                continue;

            if (tps6598x_restore_irqs(tps, &tps6598x_irq_state[idx]))
                printf("usb: unable to restore IRQ masks for %s\n", name);

            tps6598x_shutdown(tps);
        }
    }

    i2c_shutdown(i2c);
}

void usb_hpm_restore_irqs(bool force)
{
    /*
     * Do not try to restore irqs on M3/M4 which don't use i2c
     */
    if (adt_path_offset(adt, "/arm-io/nub-spmi-a0/hpm0") > 0)
        return;

    /*
     * Do not try to restore irqs on A7-A11 which don't use i2c
     */
    if (adt_path_offset(adt, "/arm-io/otgphyctrl") > 0 &&
        adt_path_offset(adt, "/arm-io/usb-complex") > 0)
        return;

    if (adt_is_compatible(adt, 0, "J180dAP"))
        usb_i2c_restore_irqs("/arm-io/i2c3", force);
    usb_i2c_restore_irqs("/arm-io/i2c0", force);
}

void usb_iodev_init(void)
{
    for (int i = 0; i < USB_IODEV_COUNT; i++) {
        dwc3_dev_t *opaque;
        struct iodev *usb_iodev;

        opaque = usb_iodev_bringup(i);
        if (!opaque)
            continue;

        usb_iodev = memalign(SPINLOCK_ALIGN, sizeof(*usb_iodev));
        if (!usb_iodev)
            continue;

        usb_iodev->ops = &iodev_usb_ops;
        usb_iodev->opaque = opaque;
        usb_iodev->usage = USAGE_CONSOLE | USAGE_UARTPROXY;
        spin_init(&usb_iodev->lock);

        iodev_register_device(IODEV_USB0 + i, usb_iodev);
        printf("USB%d: initialized at %p\n", i, opaque);
    }
}

void usb_iodev_shutdown(void)
{
    for (int i = 0; i < USB_IODEV_COUNT; i++) {
        struct iodev *usb_iodev = iodev_unregister_device(IODEV_USB0 + i);
        if (!usb_iodev)
            continue;

        printf("USB%d: shutdown\n", i);
        usb_dwc3_shutdown(usb_iodev->opaque);
        free(usb_iodev);
    }
}

void usb_iodev_vuart_setup(iodev_id_t iodev)
{
    if (iodev < IODEV_USB0 || iodev >= IODEV_USB0 + USB_IODEV_COUNT)
        return;

    iodev_usb_vuart.opaque = iodev_get_opaque(iodev);
}
