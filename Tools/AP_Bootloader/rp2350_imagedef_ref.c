/* Board-specific defines (PICO2, etc.) are generated per build into hwdef.h */
#include "hwdef.h"

/*
 * rp2350_imagedef_ref.c - Force RP2350 PICOBIN IMAGE_DEF block into flash,
 *                          and provide minimal ChibiOS init for the bootloader.
 *
 * The RP2350 bootrom requires a PICOBIN IMAGE_DEF block (identified by magic
 * 0xffffded3) within the first 4KB of flash. Without it the bootrom falls
 * through to USB ROM boot mode and never executes code from XIP flash.
 *
 * The block is defined in ChibiOS rp2350_imagedef.S (placed in .embedded_block
 * section) and compiled into libch.a as rp2350_imagedef.o.  The linker only
 * extracts archive members that satisfy unresolved symbol references, so the
 * object must be forced by referencing its exported symbol __embedded_block.
 *
 * The linker script common.ld includes KEEP(*(.embedded_block)) and the
 * embedded_block output section is placed before the ARM vector table so the
 * bootrom can locate it at the very start of flash.
 *
 * This file is compiled as a direct object (not archived) in the AP_Bootloader
 * wscript, so its undefined reference to __embedded_block unconditionally
 * forces rp2350_imagedef.o out of libch.a during the link step.
 *
 * On non-RP2350 boards __embedded_block does not exist, and the #ifdef guard
 * means this file compiles to an empty translation unit — no cost at all.
 */

#ifdef PICO2

#include <stdint.h>
#include "hal.h"

/* Defined in modules/ChibiOS/os/common/startup/ARMCMx/devices/RP2350/rp2350_imagedef.S */
extern const char __embedded_block[];

/*
 * The volatile write prevents the compiler from treating this as dead code.
 * __attribute__((used)) ensures the variable survives even with -O2/LTO.
 * The linker sees an undefined reference to __embedded_block and resolves it
 * by pulling rp2350_imagedef.o from libch.a.
 */
__attribute__((used)) static const void *const _rp2350_imagedef_anchor =
    (const void *)__embedded_block;

/*
 * Minimal ChibiOS init for the RP2350 bootloader.
 *
 * The normal ArduPilot full build links board.c (board.o in libch.a) which
 * provides strong overrides for __early_init, __late_init, and boardInit.
 * That object also contains pico2_gpio_init, setup_usb_strings, malloc_init,
 * efl init etc. — too large to fit within the 16 kB bootloader flash budget.
 *
 * halInit() (in hal.o) calls boardInit() which is the board-specific hook.
 * By defining boardInit here we satisfy that reference without extracting
 * board.o from libch.a at all.
 *
 * We also provide strong __early_init and __late_init to override the weak
 * bx-lr stubs in crt0_v8m-ml.S and to avoid a multiple-definition error if
 * board.o is ever pulled in by some other dependency.
 *
 * Symbol resolution order (this .o appears before -lch in the link command):
 *   __early_init  → our stub (does nothing; GPIO not needed for bootloader)
 *   __late_init   → our minimal version (calls halInit + chSysInit only)
 *   boardInit     → our stub (prevents hal.o from pulling board.o)
 *   halInit       → hal.o   (small HAL initalizer)
 *   chSysInit     → chsys.o (RTOS kernel initializer)
 *   board.o       → NOT extracted; stays out of the bootloader flash image
 *
 * VTOR fix:
 * The RP2350 HAL low-level init (hal_lld_init via halInit) resets SCB->VTOR
 * to the image load base (0x10000000) as part of the RP2350 system init.
 * Our ARM vector table starts at 0x10000020 (after the 32-byte IMAGE_DEF
 * PICOBIN block).  Without correcting VTOR, every peripheral IRQ dispatches
 * to the wrong entry and hits _unhandled_exception.
 * We restore VTOR to _vectors (= __vectors_base__ = 0x10000020) at the end
 * of __late_init, after halInit() has run.
 */
extern void halInit(void);
extern void chSysInit(void);
/* _vectors is defined by the startup section in the linker script */
extern const uint32_t _vectors[];

/*
 * These are strong overrides for the ChibiOS startup hooks. Declare them before
 * defining them to keep -Wmissing-declarations builds warning-free.
 */
void __early_init(void);
void boardInit(void);
void __late_init(void);

/*
 * setup_usb_strings() is defined in hwdef/common/usbcfg.c (inside
 * #if defined(HAL_USB_PRODUCT_ID)).  It populates vcom_strings[1..3]
 * (manufacturer, product, serial) from HAL_USB_STRING_* defines so that
 * the USB host receives valid string descriptors during enumeration.
 *
 * Without this call vcom_strings[1..3] remain {0, NULL} (static-init default)
 * and the USB device returns a zero-length IN packet for every string request.
 * Linux treats a ZLP as an error, waits ~5 seconds, and retries three times —
 * adding ~15 s to enumeration.  The bootloader times out before USB becomes
 * usable, so uploader.py never gets a chance to connect.
 */
#if defined(HAL_USB_PRODUCT_ID)
extern void setup_usb_strings(void);
#endif

/* SCB->VTOR register */
#define SCB_VTOR_REG   (*(volatile uint32_t *)0xE000ED08U)
/* NVIC external IRQ disable/clear registers. */
#define NVIC_ICER0_REG (*(volatile uint32_t *)0xE000E180U)
#define NVIC_ICER1_REG (*(volatile uint32_t *)0xE000E184U)
#define NVIC_ICPR0_REG (*(volatile uint32_t *)0xE000E280U)
#define NVIC_ICPR1_REG (*(volatile uint32_t *)0xE000E284U)
/*
 * RP2350 has external IRQs 0..51 spread across NVIC banks 0 and 1.
 * Keep bootloader-critical lines enabled:
 * - TIMER0 IRQ0 (scheduler tick setup path)
 * - USBCTRL IRQ (USB CDC bootloader transport)
 * - UART0/UART1 IRQs (defensive keep for board variants / debug transport)
 *
 * Note: XIP flash execution does not depend on an NVIC IRQ line, so there is
 * no dedicated "flash interrupt" that must be preserved here.
 */
#define RP2350_NVIC_KEEP_BANK0_MASK (uint32_t)((1U << 0U) | (1U << 14U))
#define RP2350_NVIC_KEEP_BANK1_MASK (uint32_t)((1U << (33U - 32U)) | (1U << (34U - 32U)))
#define RP2350_NVIC_EXT_IRQS_BANK0_DISABLE_MASK (uint32_t)(0xFFFFFFFFU & ~RP2350_NVIC_KEEP_BANK0_MASK)
#define RP2350_NVIC_EXT_IRQS_BANK1_DISABLE_MASK (uint32_t)(0x000FFFFFU & ~RP2350_NVIC_KEEP_BANK1_MASK)
/* Crash breadcrumb values mirrored with app-side diagnostics. */
#define RP2350_RESET_DIAG_UNHANDLED_EXCEPTION 0x55484E44U /* 'UHND' */

void __attribute__((noreturn)) _unhandled_exception(void);

void __attribute__((noreturn)) _unhandled_exception(void)
{
    /*
     * Persist trap context for SWD post-mortem:
     *   SCRATCH[2] = xPSR  (IPSR low bits = active exception number)
     *   SCRATCH[3] = ICSR  (active/pending exception state)
     *   SCRATCH[4] = VTOR  (vector table base at trap time)
     *   SCRATCH[7] = 'UHND' marker
     */
    WATCHDOG->SCRATCH[2] = __get_xPSR();
    WATCHDOG->SCRATCH[3] = SCB->ICSR;
    WATCHDOG->SCRATCH[4] = SCB->VTOR;
    WATCHDOG->SCRATCH[7] = RP2350_RESET_DIAG_UNHANDLED_EXCEPTION;
    __DSB();
    __ISB();

    NVIC_SystemReset();
    while (1) {}
}

/* Called from crt0 before BSS/data init.  No GPIO setup needed in bootloader,
 * but we do need to reset peripherals and start the PLLs.
 *
 * ChibiOS commit a7485da935 ("Fix RP Peripheral Init and move clock setup
 * early to match STM32 convention") removed rp_peripheral_reset() and
 * rp_clock_init() from hal_lld_init() and requires the board-level
 * __early_init() to call them instead.  Without this, after any NVIC_SystemReset
 * (including the phase-1 reset in jump_to_app()), the system stays on the ROM
 * ROSC clock (~12 MHz), USB PLL never locks, and USB CDC never enumerates. */
void __early_init(void) {
#if RP_NO_INIT == FALSE
    /* Reset all peripherals except what is needed to keep XIP flash running
     * and the PLLs (rp_clock_init() below handles PLL_SYS/PLL_USB after
     * switching CLK_SYS to the safe ROSC glitchless mux). */
    rp_peripheral_reset(~(RESETS_ALLREG_IO_QSPI  | RESETS_ALLREG_PADS_QSPI |
                           RESETS_ALLREG_PLL_SYS  | RESETS_ALLREG_PLL_USB));

    /* Configure PLL_SYS → 150 MHz system clock and PLL_USB → 48 MHz USB
     * reference.  Must happen before halInit() so USB CDC can enumerate and
     * so the ChibiOS scheduler tick period is correct. */
    rp_clock_init();
#endif /* RP_NO_INIT */
}

/* stub for the boardInit() hook called inside halInit() */
void boardInit(void) {}

/* Initialise the HAL and RTOS kernel.  Must run before any ChibiOS service. */
void __late_init(void)
{
    /*
     * Set VTOR to our vector table base BEFORE calling halInit().
     *
     * The RP2350 bootrom sets VTOR to the image load address (0x10000000)
     * when launching our image.  Our vector table starts at 0x10000020
     * (after the 32-byte PICOBIN IMAGE_DEF block), so any IRQ that fires
     * during halInit() would dispatch to wrong/IMAGE_DEF data and crash.
     *
     * halInit() → stInit() → st_lld_init() arms TIMER0 and enables IRQ0.
     * We must correct VTOR before that happens so IRQ0 dispatches to the
     * real ChibiOS TIMER0 handler in our vector table.
     */
    SCB_VTOR_REG = (uint32_t)_vectors;

    /*
     * Defensive IRQ scrub for RP2350 bootloader isolation:
     * after SYSRESETREQ handoff from app, we can inherit stale enable/pending
     * state on unrelated peripheral lines. Clear and mask only non-essential
     * vectors so scheduler timing and USB bootloader transport remain intact.
     */
    NVIC_ICER0_REG = RP2350_NVIC_EXT_IRQS_BANK0_DISABLE_MASK;
    NVIC_ICER1_REG = RP2350_NVIC_EXT_IRQS_BANK1_DISABLE_MASK;
    NVIC_ICPR0_REG = RP2350_NVIC_EXT_IRQS_BANK0_DISABLE_MASK;
    NVIC_ICPR1_REG = RP2350_NVIC_EXT_IRQS_BANK1_DISABLE_MASK;

    halInit();

#if defined(HAL_USB_PRODUCT_ID)
    /*
     * Populate USB string descriptors AFTER halInit() so that ChibiOS HAL
     * state is fully set up before we access OTP (for the serial number) and
     * write to vcom_strings[].  This must happen before chSysInit() starts
     * the USB driver and the host begins enumeration.
     */
    setup_usb_strings();
#endif

    chSysInit();
}

#endif /* PICO2 */
