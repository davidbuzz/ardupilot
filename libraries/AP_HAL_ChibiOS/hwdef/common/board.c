/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 * Modified for use in AP_HAL by Andrew Tridgell and Siddharth Bharat Purohit
 */

#include "hal.h"
#include "usbcfg.h"
#include "stm32_util.h"
#include "flash.h"
#include "watchdog.h"
#include "board.h"
#include "hrt.h"

#if defined(PIC02) || defined(RP2350) || PIC02_AVAILABLE
#define STM32_AVAILABLE FALSE 
#endif

#if PIC02_AVAILABLE == TRUE
/*
 * RP2350 reset-cause breadcrumb register used for live SWD diagnosis.
 * SCRATCH[0] and SCRATCH[1] are used by fastboot/bootloader handoff,
 * SCRATCH[6] is used by watchdog-reason detection (see watchdog.c/h),
 * so use SCRATCH[7] (RP2350_RESET_DIAG_SCRATCH_IDX) for the sentinel.
 * SCRATCH[2..5] are reserved here for exception context capture.
 * The sentinel constants are defined in watchdog.h — include it for consistency.
 */
#include "watchdog.h"

/*
 * Override the ChibiOS weak _unhandled_exception stub.
 *
 * The default ChibiOS implementation is an infinite loop (.stay: b .stay).
 * On RP2350 any unhandled IRQ — whether from a floating pin glitching a
 * disabled peripheral, a spurious SIO interrupt, or any future vector not
 * explicitly registered — would permanently lock the CPU in that loop with
 * the watchdog paused (PAUSE_DBG=1) when GDB is attached, or silently
 * wedged in production.
 *
 * Instead, trigger an immediate SYSRESETREQ so the board self-resets and
 * recovers.  The watchdog will also catch any case where NVIC_SystemReset
 * itself is somehow delayed.
 *
 * This override is a strong symbol and therefore replaces the weak one.
 * It must be in a C translation unit (not assembly), so the linker picks
 * this definition preferentially.
 */
static volatile uint8_t etype = 0;
static volatile uint8_t e_number = 0;
void __attribute__((noreturn)) _unhandled_exception(void)
{
  /*
   * Persist breadcrumbs so SYSRESETREQ loops can be identified post-reset.
   *
   * SCRATCH[0] = SCB->VTOR at trap time — tells us which vector table was
   *              active. Should be &_vectors (flash, 0x10010080 for Laurel).
   *              If VTOR ≠ expected, that is why the wrong handler fired.
   * SCRATCH[2] = xPSR at trap (IPSR bits[8:0] = active exception number;
   *              subtract 16 to get external IRQ number, e.g. 52-16=36=I2C0)
   * SCRATCH[3] = ICSR snapshot (VECTACTIVE, VECTPENDING, NMIPENDSET, etc.)
   * SCRATCH[4] = NVIC ISPR0 (pending IRQ 0..31)
   * SCRATCH[5] = NVIC ISPR1 (pending IRQ 32..51 — covers I2C0=36, I2C1=37,
   *              UART0=33, UART1=34, SPI0=31(in ISPR0), SPI1=32, USB=28)
   * SCRATCH[6] = LR at trap entry (EXC_RETURN pattern 0xFFFFxx..)
   * SCRATCH[7] = 'UHND' sentinel (0x55484E44)
   */
  WATCHDOG->SCRATCH[0] = SCB->VTOR;       /* vector table base at trap time */
  WATCHDOG->SCRATCH[2] = e_number = (__get_xPSR() & 0x1FFU) - 16U;
  WATCHDOG->SCRATCH[3] = etype = SCB->ICSR;
  WATCHDOG->SCRATCH[4] = NVIC->ISPR[0];
  WATCHDOG->SCRATCH[5] = NVIC->ISPR[1];  /* IRQ32..51: I2C0=bit4, I2C1=bit5, SPI1=bit0 */
  /* Capture EXC_RETURN from LR — tells us which stack and FPU state was active */
  __asm volatile ("str lr, %0" : "=m"(WATCHDOG->SCRATCH[6]) :: "memory");
  WATCHDOG->SCRATCH[RP2350_RESET_DIAG_SCRATCH_IDX] = RP2350_RESET_DIAG_UNHANDLED_EXCEPTION;
  __DSB();
  __ISB();

    NVIC_SystemReset();
    /* NOTREACHED — NVIC_SystemReset() does not return, but the noreturn   */
    /* attribute requires an infinite loop to satisfy the compiler.        */
    while (1) {}
}


/*
 * VectorD4 = IRQ 37 = I2C1_IRQ.
 *
 * When RP_I2C_USE_I2C1 is FALSE the ChibiOS I2C LLD does not register a strong
 * VectorD4 handler, leaving the weak stub active.  If the ROM Core-1 launch
 * sequence fires IRQ 37 at Core-0 (observed during the descending IRQ sweep),
 * the weak stub falls through to _unhandled_exception.
 *
 * Guard with RP_I2C_USE_I2C1 == FALSE so that boards that do enable I2C1 keep
 * the real driver handler and do not get a linker duplicate-symbol error.
 */
#if RP_I2C_USE_I2C1 == FALSE
OSAL_IRQ_HANDLER(VectorD4) {
    OSAL_IRQ_PROLOGUE();
    nvicDisableVector(37U);   /* I2C1_IRQ — spurious ROM launch IRQ, I2C1 not in use */
    nvicClearPending(37U);
    OSAL_IRQ_EPILOGUE();
}
#endif /* RP_I2C_USE_I2C1 == FALSE */

// TODO add more conditional blocks like teh above that disable other ROM-spawned IRQs that we don't use, e.g. USB=28, SPI1=32, I2C0=36 

/*
 * No-op handlers for the IRQs we dont really care about
 *
 * The RP2350 ROM Core-1 multicore launch sequence (inside start_core1() in hal_lld.c)
 * powers up Core-1 via the POWMAN domain sequencer and fires a descending sequence of
 * IRQs at Core-0 .
 *
 * All of these fire *after* nvicInit() has run, so the NVIC scrub in the bootloader
 * cannot prevent them.  Without strong handler symbols the .weak VectorXX stubs in
 * vectors.S fall through to _unhandled_exception → NVIC_SystemReset() crash loop.
 * We cover the full range as each of these either needs to be handled by a LLD or handled here.
 *
 */

 // timer0 is the system timer.
 //
// Vector40	0	TIMER0_IRQ0
// Vector44	1	TIMER0_IRQ1
// Vector48	2	TIMER0_IRQ2
// Vector4C	3	TIMER0_IRQ3

void turn_off_timer(int timerid) { 
  switch (timerid) {
    case 0:
      TIMER0->INTE  = 0U;
      TIMER0->INTR  = 0xFU;   /* W1C: clear all 4 alarm pending bits */
      TIMER0->ARMED = 0xFU;   /* W1C: disarm all 4 alarms */
      break;
    case 1:
      TIMER1->INTE  = 0U;
      TIMER1->INTR  = 0xFU;   /* W1C: clear all 4 alarm pending bits */
      TIMER1->ARMED = 0xFU;   /* W1C: disarm all 4 alarms */
      break;
    default:
      break;
  }
}

// Vector50	4	TIMER1_IRQ0
// todo work out why _WFI in idle bring us to Vector50 constatly.
// Vector50-5C: TIMER1_IRQ0-3 (IRQ 4-7)
//
// TIMER1 alarms are level-triggered: TIMER->INTS asserts the IRQ line
// continuously while any alarm fires.  We must:
//   1. Silence the peripheral (INTE=0 drives INTS=0, de-asserting the line)
//   2. DSB so the APB write propagates before the NVIC sees it
//   3. nvicClearPending — safe now that the input line is LOW
//   4. nvicDisableVector — belt-and-suspenders so this never fires again
// Reversing steps 1/3 or omitting __DSB causes the NVIC to immediately
// re-assert ISPR (level input still HIGH) and loop forever in idle.
OSAL_IRQ_HANDLER(Vector50) {
    OSAL_IRQ_PROLOGUE();
    turn_off_timer(1);           /* silence peripheral FIRST (INTE=0→INTS=0) */
    __DSB();                     /* ensure APB writes reach TIMER1 before NVIC op */
    nvicClearPending(4U);        /* TIMER1_IRQ0 — safe now that INTS is de-asserted */
    nvicDisableVector(4U);       /* never fire again */
    OSAL_IRQ_EPILOGUE();
}
// Vector54	5	TIMER1_IRQ1
OSAL_IRQ_HANDLER(Vector54) {
    OSAL_IRQ_PROLOGUE();
    turn_off_timer(1);           /* peripheral silence first */
    __DSB();
    nvicClearPending(5U);        /* TIMER1_IRQ1 */
    nvicDisableVector(5U);
    OSAL_IRQ_EPILOGUE();
}
// Vector58	6	TIMER1_IRQ2
OSAL_IRQ_HANDLER(Vector58) {
    OSAL_IRQ_PROLOGUE();
    turn_off_timer(1);           /* peripheral silence first */
    __DSB();
    nvicClearPending(6U);        /* TIMER1_IRQ2 */
    nvicDisableVector(6U);
    OSAL_IRQ_EPILOGUE();
}
// Vector5C	7	TIMER1_IRQ3
OSAL_IRQ_HANDLER(Vector5C) {
    OSAL_IRQ_PROLOGUE();
    turn_off_timer(1);           /* peripheral silence first */
    __DSB();
    nvicClearPending(7U);        /* TIMER1_IRQ3 */
    nvicDisableVector(7U);
    OSAL_IRQ_EPILOGUE();
}
// Vector60	8	PWM_IRQ_WRAP_0
// Vector64	9	PWM_IRQ_WRAP_1
OSAL_IRQ_HANDLER(Vector64) {
    OSAL_IRQ_PROLOGUE();
    nvicDisableVector(9U);   /* PWM_IRQ_WRAP_1  */
    nvicClearPending(9U);
    OSAL_IRQ_EPILOGUE();
}
// Vector68	10	DMA_IRQ_0
// Vector6C	11	DMA_IRQ_1
 //Vector70	12	DMA_IRQ_2
OSAL_IRQ_HANDLER(Vector70) {
    OSAL_IRQ_PROLOGUE();
    nvicDisableVector(12U);   /* DMA_IRQ_2  */
    nvicClearPending(12U);
    OSAL_IRQ_EPILOGUE();
}
 //Vector74	13	DMA_IRQ_3
OSAL_IRQ_HANDLER(Vector74) {
    OSAL_IRQ_PROLOGUE();
    nvicDisableVector(13U);   /* DMA_IRQ_3  */
    nvicClearPending(13U);
    OSAL_IRQ_EPILOGUE();
}
// 14 is 
//Vector78	14	USBCTRL_IRQ

// 15&16 is PIO0
//Vector7C	15	PIO0_IRQ_0
//Vector80	16	PIO0_IRQ_1
// 17&18 is PIO1
//Vector84	17	PIO1_IRQ_0
//Vector88	18	PIO1_IRQ_1
// 19&20 is PIO2
//Vector8C	19	PIO2_IRQ_0
//Vector90	20	PIO2_IRQ_1
// OSAL_IRQ_HANDLER(Vector88) {
//     OSAL_IRQ_PROLOGUE();
//     nvicDisableVector(18U);   /* PIO1_IRQ_1  */
//     nvicClearPending(18U);
//     OSAL_IRQ_EPILOGUE();
// }
OSAL_IRQ_HANDLER(Vector8C) {
    OSAL_IRQ_PROLOGUE();
    nvicDisableVector(19U);   /* PIO2_IRQ_0  */
    nvicClearPending(19U);
    OSAL_IRQ_EPILOGUE();
}
OSAL_IRQ_HANDLER(Vector90) {
    OSAL_IRQ_PROLOGUE();
    nvicDisableVector(20U);   /* PIO2_IRQ_1  */
    nvicClearPending(20U);
    OSAL_IRQ_EPILOGUE();
}
// 21 is IO_IRQ_BANK0, and we use that, so defined in the LLD, not here.
//Vector94	IRQ21	IO_IRQ_BANK0
 OSAL_IRQ_HANDLER(Vector98) {
    OSAL_IRQ_PROLOGUE();
    nvicDisableVector(22U);   /* IO_IRQ_BANK0_NS  */
    nvicClearPending(22U);
    OSAL_IRQ_EPILOGUE();
}
OSAL_IRQ_HANDLER(Vector9C) {
    OSAL_IRQ_PROLOGUE();
    nvicDisableVector(23U);   /* IO_IRQ_QSPI  */
    nvicClearPending(23U);
    OSAL_IRQ_EPILOGUE();
}
OSAL_IRQ_HANDLER(VectorA0) {
    OSAL_IRQ_PROLOGUE();
    nvicDisableVector(24U);   /* IO_IRQ_QSPI_NS  */
    nvicClearPending(24U);
    OSAL_IRQ_EPILOGUE();
}
OSAL_IRQ_HANDLER(VectorA4) {
    OSAL_IRQ_PROLOGUE();
    nvicDisableVector(25U);   /* SIO_IRQ_FIFO  */
    nvicClearPending(25U);
    OSAL_IRQ_EPILOGUE();
}
// caution. if the ChibiOS SIO LLD registers a strong VectorA8, since the SIO bell is used for inter-core signalling in some ChibiOS configurations.
OSAL_IRQ_HANDLER(VectorA8) {
    OSAL_IRQ_PROLOGUE();
    nvicDisableVector(26U);   /* SIO_IRQ_BELL  */
    nvicClearPending(26U);
    OSAL_IRQ_EPILOGUE();
}

OSAL_IRQ_HANDLER(VectorAC) {
    OSAL_IRQ_PROLOGUE();
    nvicDisableVector(27U);   /* SIO_IRQ_FIFO_NS */
    nvicClearPending(27U);
    OSAL_IRQ_EPILOGUE();
}
OSAL_IRQ_HANDLER(VectorB0) {
    OSAL_IRQ_PROLOGUE();
    nvicDisableVector(28U);   /* SIO_IRQ_BELL_NS */
    nvicClearPending(28U);
    OSAL_IRQ_EPILOGUE();
}
OSAL_IRQ_HANDLER(VectorB4) {
    OSAL_IRQ_PROLOGUE();
    nvicDisableVector(29U);   /* SIO_IRQ_MTIMECMP */
    nvicClearPending(29U);
    OSAL_IRQ_EPILOGUE();
}
OSAL_IRQ_HANDLER(VectorB8) {
    OSAL_IRQ_PROLOGUE();
    nvicDisableVector(30U);   /* CLOCKS_IRQ */
    nvicClearPending(30U);
    OSAL_IRQ_EPILOGUE();
}
OSAL_IRQ_HANDLER(VectorBC) {
    OSAL_IRQ_PROLOGUE();
    nvicDisableVector(31U);   /* SPI0_IRQ  */
    nvicClearPending(31U);
    OSAL_IRQ_EPILOGUE();
}
 //#if RP_SPI_USE_SPI1 == FALSE - currently not reqd as SPI1 LLD doesnt use VectorC0
 OSAL_IRQ_HANDLER(VectorC0) {
    OSAL_IRQ_PROLOGUE();
    nvicDisableVector(32U);   /* SPI1_IRQ */
    nvicClearPending(32U);
    OSAL_IRQ_EPILOGUE();
}
//#endif
// 33 and 34 are UART0_IRQ and UART1_IRQ,  and in-use, so defined elsewhere in lld.
//VectorC4	33	UART0_IRQ
//VectorC8	34	UART1_IRQ
OSAL_IRQ_HANDLER(VectorCC) {
    OSAL_IRQ_PROLOGUE();
    nvicDisableVector(35U);   /* ADC_IRQ_FIFO */
    nvicClearPending(35U);
    OSAL_IRQ_EPILOGUE();
}
// 36 and 37 are I2C0_IRQ and I2C1_IRQ, and I2C1 is conditionally in-use, so defined elsewhere in lld.
//the board uses I2C1 (GPIO15/18) for the compass — not I2C0
//VectorD0	36	I2C0_IRQ
#if RP_I2C_USE_I2C0 == FALSE
OSAL_IRQ_HANDLER(VectorD0) {
    OSAL_IRQ_PROLOGUE();
    nvicDisableVector(36U);   /* I2C0_IRQ */
    nvicClearPending(36U);
    OSAL_IRQ_EPILOGUE();
}
#endif 

//VectorD4	37	I2C1_IRQ
OSAL_IRQ_HANDLER(VectorD8) {
    OSAL_IRQ_PROLOGUE();
    nvicDisableVector(38U);   /* OTP_IRQ */
    nvicClearPending(38U);
    OSAL_IRQ_EPILOGUE();
}
OSAL_IRQ_HANDLER(VectorDC) {
    OSAL_IRQ_PROLOGUE();
    nvicDisableVector(39U);   /* TRNG_IRQ */
    nvicClearPending(39U);
    OSAL_IRQ_EPILOGUE();
}
OSAL_IRQ_HANDLER(VectorE0) {
    OSAL_IRQ_PROLOGUE();
    nvicDisableVector(40U);   /* PROC0_IRQ_CTI */
    nvicClearPending(40U);
    OSAL_IRQ_EPILOGUE();
}
OSAL_IRQ_HANDLER(VectorE4) {
    OSAL_IRQ_PROLOGUE();
    nvicDisableVector(41U);   /* PROC1_IRQ_CTI */
    nvicClearPending(41U);
    OSAL_IRQ_EPILOGUE();
}
OSAL_IRQ_HANDLER(VectorE8) {
    OSAL_IRQ_PROLOGUE();
    nvicDisableVector(42U);   /* PLL_SYS_IRQ */
    nvicClearPending(42U);
    OSAL_IRQ_EPILOGUE();
}
OSAL_IRQ_HANDLER(VectorEC) {
    OSAL_IRQ_PROLOGUE();
    nvicDisableVector(43U);   /* PLL_USB_IRQ */
    nvicClearPending(43U);
    OSAL_IRQ_EPILOGUE();
}
OSAL_IRQ_HANDLER(VectorF0) {
    OSAL_IRQ_PROLOGUE();
    nvicDisableVector(44U);   /* POWMAN_IRQ_POW */
    nvicClearPending(44U);
    OSAL_IRQ_EPILOGUE();
}
OSAL_IRQ_HANDLER(VectorF4) {
    OSAL_IRQ_PROLOGUE();
    nvicDisableVector(45U);   /* POWMAN_IRQ_TIMER */
    nvicClearPending(45U);
    OSAL_IRQ_EPILOGUE();
}
OSAL_IRQ_HANDLER(VectorF8) {
    OSAL_IRQ_PROLOGUE();
    /* Disable and clear du — no further action needed. */
    nvicDisableVector(46U);
    nvicClearPending(46U);
    OSAL_IRQ_EPILOGUE();
}
OSAL_IRQ_HANDLER(VectorFC) {
    OSAL_IRQ_PROLOGUE();
    /* Disable and clear SPARE_IRQ_1 (IRQ 47) — no further action needed. */
    nvicDisableVector(47U);
    nvicClearPending(47U);
    OSAL_IRQ_EPILOGUE();
}
OSAL_IRQ_HANDLER(Vector100) {
    OSAL_IRQ_PROLOGUE();
    nvicDisableVector(48U);   /* SPAREIRQ_IRQ_2 */
    nvicClearPending(48U);
    OSAL_IRQ_EPILOGUE();
}
OSAL_IRQ_HANDLER(Vector104) {
    OSAL_IRQ_PROLOGUE();
    nvicDisableVector(49U);   /* SPAREIRQ_IRQ_3 */
    nvicClearPending(49U);
    OSAL_IRQ_EPILOGUE();
}
OSAL_IRQ_HANDLER(Vector108) {
    OSAL_IRQ_PROLOGUE();
    nvicDisableVector(50U);   /* SPAREIRQ_IRQ_4 */
    nvicClearPending(50U);
    OSAL_IRQ_EPILOGUE();
}
OSAL_IRQ_HANDLER(Vector10C) {
    OSAL_IRQ_PROLOGUE();
    nvicDisableVector(51U);   /* SPAREIRQ_IRQ_5 */
    nvicClearPending(51U);
    OSAL_IRQ_EPILOGUE();
}

#endif /* PIC02_AVAILABLE */

/*
 * RP2350 (Pico2) note on XIP cache coherency after SWD flashing:
 *
 * When we program flash via SWD/OpenOCD then issue a SYSRESETREQ reset, the
 * RP2350 XIP cache can retain stale lines across the reset. If those stale
 * lines include the vector table, an external IRQ can dispatch to the *old*
 * handler address even though flash now contains the correct ISR entry.
 *
 * This shows up as (for example) I2C1 IRQ37 repeatedly landing in the default
 * DebugMon/_unhandled_exception handler, even though VectorD4 is present and
 * correctly placed in the vector table in flash.
 *
 * The ChibiOS RP2350 EFL driver invalidates the XIP cache by writing through
 * the XIP "maintenance" alias then re-enabling the cache. We reuse that same
 * mechanism here, but we must do it after C runtime init so the helper can
 * execute from RAM (the .ramtext section is copied into RAM as part of .data).
 */
#if PIC02_AVAILABLE == TRUE
#define RP2350_RAMFUNC __attribute__((noinline, section(".ramtext")))
static RP2350_RAMFUNC void rp2350_invalidate_xip_cache(void)
{
    /* Constants mirror modules/ChibiOS/os/hal/ports/RP/RP2350/hal_efl_lld.c */
    volatile uint8_t *maint = (volatile uint8_t *)0x18000000U;   /* RP_XIP_MAINTENANCE_BASE */
    volatile uint32_t *xip = (volatile uint32_t *)0x400C8000U;    /* RP_XIP_CTRL_BASE */

    /* 16KB cache, 8-byte cache line size. Writing any value invalidates the line. */
    for (uint32_t offset = 0U; offset < (16U * 1024U); offset += 8U) {
        maint[offset] = 0U;
    }

    __DSB();
    __ISB();

    /* Re-enable both Secure and Non-secure cache views. */
    xip[0] = (1U << 0) | (1U << 1); /* EN_SECURE | EN_NONSECURE */
}
#endif // PIC02_AVAILABLE

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

#if STM32_AVAILABLE == TRUE
    /**
    * @brief   STM32 GPIO static initialization data.
    */
    #if defined(STM32F100_MCUCONF) || defined(STM32F103_MCUCONF) || defined(STM32F105_MCUCONF)
        const PALConfig pal_default_config =
        {
          {VAL_GPIOA_ODR, VAL_GPIOA_CRL, VAL_GPIOA_CRH},
          {VAL_GPIOB_ODR, VAL_GPIOB_CRL, VAL_GPIOB_CRH},
          {VAL_GPIOC_ODR, VAL_GPIOC_CRL, VAL_GPIOC_CRH},
          {VAL_GPIOD_ODR, VAL_GPIOD_CRL, VAL_GPIOD_CRH},
          {VAL_GPIOE_ODR, VAL_GPIOE_CRL, VAL_GPIOE_CRH},
        };

    #else //Other than STM32F1/F3 series, and excluding non-stm32 now too.

        /**
        * @brief   Type of STM32 GPIO port setup.
        */
        typedef struct {
          uint32_t              moder;
          uint32_t              otyper;
          uint32_t              ospeedr;
          uint32_t              pupdr;
          uint32_t              odr;
          uint32_t              afrl;
          uint32_t              afrh;
        } gpio_setup_t;

        /**
        * @brief   Type of STM32 GPIO initialization data.
        */
        typedef struct {
            #if STM32_HAS_GPIOA || defined(__DOXYGEN__)
              gpio_setup_t          PAData;
            #endif
            #if STM32_HAS_GPIOB || defined(__DOXYGEN__)
              gpio_setup_t          PBData;
            #endif
            #if STM32_HAS_GPIOC || defined(__DOXYGEN__)
              gpio_setup_t          PCData;
            #endif
            #if STM32_HAS_GPIOD || defined(__DOXYGEN__)
              gpio_setup_t          PDData;
            #endif
            #if STM32_HAS_GPIOE || defined(__DOXYGEN__)
              gpio_setup_t          PEData;
            #endif
            #if STM32_HAS_GPIOF || defined(__DOXYGEN__)
              gpio_setup_t          PFData;
            #endif
            #if STM32_HAS_GPIOG || defined(__DOXYGEN__)
              gpio_setup_t          PGData;
            #endif
            #if STM32_HAS_GPIOH || defined(__DOXYGEN__)
              gpio_setup_t          PHData;
            #endif
            #if STM32_HAS_GPIOI || defined(__DOXYGEN__)
              gpio_setup_t          PIData;
            #endif
            #if STM32_HAS_GPIOJ || defined(__DOXYGEN__)
              gpio_setup_t          PJData;
            #endif
            #if STM32_HAS_GPIOK || defined(__DOXYGEN__)
              gpio_setup_t          PKData;
            #endif
        } gpio_config_t;

        /**
        * @brief   STM32 GPIO static initialization data.
        */
        static const gpio_config_t gpio_default_config = {
            #if STM32_HAS_GPIOA
              {VAL_GPIOA_MODER, VAL_GPIOA_OTYPER, VAL_GPIOA_OSPEEDR, VAL_GPIOA_PUPDR,
              VAL_GPIOA_ODR,   VAL_GPIOA_AFRL,   VAL_GPIOA_AFRH},
            #endif
            #if STM32_HAS_GPIOB
              {VAL_GPIOB_MODER, VAL_GPIOB_OTYPER, VAL_GPIOB_OSPEEDR, VAL_GPIOB_PUPDR,
              VAL_GPIOB_ODR,   VAL_GPIOB_AFRL,   VAL_GPIOB_AFRH},
            #endif
            #if STM32_HAS_GPIOC
              {VAL_GPIOC_MODER, VAL_GPIOC_OTYPER, VAL_GPIOC_OSPEEDR, VAL_GPIOC_PUPDR,
              VAL_GPIOC_ODR,   VAL_GPIOC_AFRL,   VAL_GPIOC_AFRH},
            #endif
            #if STM32_HAS_GPIOD
              {VAL_GPIOD_MODER, VAL_GPIOD_OTYPER, VAL_GPIOD_OSPEEDR, VAL_GPIOD_PUPDR,
              VAL_GPIOD_ODR,   VAL_GPIOD_AFRL,   VAL_GPIOD_AFRH},
            #endif
            #if STM32_HAS_GPIOE
              {VAL_GPIOE_MODER, VAL_GPIOE_OTYPER, VAL_GPIOE_OSPEEDR, VAL_GPIOE_PUPDR,
              VAL_GPIOE_ODR,   VAL_GPIOE_AFRL,   VAL_GPIOE_AFRH},
            #endif
            #if STM32_HAS_GPIOF
              {VAL_GPIOF_MODER, VAL_GPIOF_OTYPER, VAL_GPIOF_OSPEEDR, VAL_GPIOF_PUPDR,
              VAL_GPIOF_ODR,   VAL_GPIOF_AFRL,   VAL_GPIOF_AFRH},
            #endif
            #if STM32_HAS_GPIOG
              {VAL_GPIOG_MODER, VAL_GPIOG_OTYPER, VAL_GPIOG_OSPEEDR, VAL_GPIOG_PUPDR,
              VAL_GPIOG_ODR,   VAL_GPIOG_AFRL,   VAL_GPIOG_AFRH},
            #endif
            #if STM32_HAS_GPIOH
              {VAL_GPIOH_MODER, VAL_GPIOH_OTYPER, VAL_GPIOH_OSPEEDR, VAL_GPIOH_PUPDR,
              VAL_GPIOH_ODR,   VAL_GPIOH_AFRL,   VAL_GPIOH_AFRH},
            #endif
            #if STM32_HAS_GPIOI
              {VAL_GPIOI_MODER, VAL_GPIOI_OTYPER, VAL_GPIOI_OSPEEDR, VAL_GPIOI_PUPDR,
              VAL_GPIOI_ODR,   VAL_GPIOI_AFRL,   VAL_GPIOI_AFRH},
            #endif
            #if STM32_HAS_GPIOJ
              {VAL_GPIOJ_MODER, VAL_GPIOJ_OTYPER, VAL_GPIOJ_OSPEEDR, VAL_GPIOJ_PUPDR,
              VAL_GPIOJ_ODR,   VAL_GPIOJ_AFRL,   VAL_GPIOJ_AFRH},
            #endif
            #if STM32_HAS_GPIOK
              {VAL_GPIOK_MODER, VAL_GPIOK_OTYPER, VAL_GPIOK_OSPEEDR, VAL_GPIOK_PUPDR,
              VAL_GPIOK_ODR,   VAL_GPIOK_AFRL,   VAL_GPIOK_AFRH}
            #endif
        };
#endif //#if STM32_AVAILABLE

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/
#if STM32_AVAILABLE == TRUE
    static void gpio_init(stm32_gpio_t *gpiop, const gpio_setup_t *config) {

      gpiop->OTYPER  = config->otyper;
      gpiop->OSPEEDR = config->ospeedr;
      gpiop->PUPDR   = config->pupdr;
      gpiop->ODR     = config->odr;
      gpiop->AFRL    = config->afrl;
      gpiop->AFRH    = config->afrh;
      gpiop->MODER   = config->moder;
    }
#endif

#if STM32_AVAILABLE == TRUE
    static void stm32_gpio_init(void) {

      /* Enabling GPIO-related clocks, the mask comes from the
        registry header file.*/
    #if defined(STM32H7)
    #if !EXT_FLASH_SIZE_MB // if we have external flash resetting GPIO might disable all comms with it
      rccResetAHB4(STM32_GPIO_EN_MASK);
    #endif
      rccEnableAHB4(STM32_GPIO_EN_MASK, true);
    #elif defined(STM32F3)
      rccResetAHB(STM32_GPIO_EN_MASK);
      rccEnableAHB(STM32_GPIO_EN_MASK, true);
    #elif defined(STM32G4) || defined(STM32L4) || defined(STM32L4PLUS)
      rccResetAHB2(STM32_GPIO_EN_MASK);
      rccEnableAHB2(STM32_GPIO_EN_MASK, true);
    #else
      rccResetAHB1(STM32_GPIO_EN_MASK);
      rccEnableAHB1(STM32_GPIO_EN_MASK, true);
    #endif

      /* Initializing all the defined GPIO ports.*/
    #if STM32_HAS_GPIOA
      gpio_init(GPIOA, &gpio_default_config.PAData);
    #endif
    #if STM32_HAS_GPIOB
      gpio_init(GPIOB, &gpio_default_config.PBData);
    #endif
    #if STM32_HAS_GPIOC
      gpio_init(GPIOC, &gpio_default_config.PCData);
    #endif
    #if STM32_HAS_GPIOD
      gpio_init(GPIOD, &gpio_default_config.PDData);
    #endif
    #if STM32_HAS_GPIOE
      gpio_init(GPIOE, &gpio_default_config.PEData);
    #endif
    #if STM32_HAS_GPIOF
      gpio_init(GPIOF, &gpio_default_config.PFData);
    #endif
    #if STM32_HAS_GPIOG
      gpio_init(GPIOG, &gpio_default_config.PGData);
    #endif
    #if STM32_HAS_GPIOH
      gpio_init(GPIOH, &gpio_default_config.PHData);
    #endif
    #if STM32_HAS_GPIOI
      gpio_init(GPIOI, &gpio_default_config.PIData);
    #endif
    #if STM32_HAS_GPIOJ
      gpio_init(GPIOJ, &gpio_default_config.PJData);
    #endif
    #if STM32_HAS_GPIOK
      gpio_init(GPIOK, &gpio_default_config.PKData);
    #endif
    }
#endif //#if STM32_AVAILABLE

#endif //!STM32F100_MCUCONF

#if PIC02_AVAILABLE == TRUE
void pico2_gpio_init(void) {
    /* Configure PWM GPIO pins to PWM alternate function (function 4).
     * HAL_PWM_GPIO_LINES is generated by chibios_hwdef.py when PWM pins are defined.
     * Use a local array so PAL_LINE (comma-operator form) is allowed as an initializer. */
    #if defined(HAL_PWM_GPIO_LINES)
    ioline_t lines[] = {HAL_PWM_GPIO_LINES};
    for (unsigned i = 0; i < sizeof(lines)/sizeof(lines[0]); i++) {
        palSetLineMode(lines[i], PAL_MODE_ALTERNATE_PWM);
    }
    #endif

    /* Configure SPI GPIO pins to SPI alternate function (FUNCSEL=1).
     * The ChibiOS RP2350 SPIv1 LLD (hal_spi_lld.c) does not set GPIO FUNCSEL —
     * it only programs SPI peripheral registers and DMA.  Without FUNCSEL=1 the
     * SPI peripheral cannot drive SCK/MOSI or receive MISO, so all sensor probes
     * would silently return 0xFF.
     * Additionally, SPIDevice.cpp reads sck_mode = palReadLineMode(sck_line) at
     * SPIBus construction time (HAL_SPI_SCK_SAVE_RESTORE path) and restores that
     * mode between transactions.  If we do not set FUNCSEL=1 here first, sck_mode
     * is saved as FUNCSEL=5 (SIO) and the restore actively defeats any SPI routing. */
#if defined(HAL_GPIO_PIN_SPI0_SCK)
    palSetLineMode(HAL_GPIO_PIN_SPI0_SCK, PAL_MODE_ALTERNATE_SPI);
    palSetLineMode(HAL_GPIO_PIN_SPI0_RX,  PAL_MODE_ALTERNATE_SPI);
    palSetLineMode(HAL_GPIO_PIN_SPI0_TX,  PAL_MODE_ALTERNATE_SPI);
#endif
#if defined(HAL_GPIO_PIN_SPI1_SCK)
    palSetLineMode(HAL_GPIO_PIN_SPI1_SCK,  PAL_MODE_ALTERNATE_SPI);
    palSetLineMode(HAL_GPIO_PIN_SPI1_MISO, PAL_MODE_ALTERNATE_SPI);
    palSetLineMode(HAL_GPIO_PIN_SPI1_MOSI, PAL_MODE_ALTERNATE_SPI);
#endif

    /* Configure I2C GPIO pins to I2C alternate function (FUNCSEL=3).
     * The ChibiOS RP2350 I2Cv1 LLD (hal_i2c_lld.c) does not configure GPIO
     * FUNCSEL — it only programs the I2C peripheral controller registers.
     * Without FUNCSEL=3 the I2C hardware peripheral cannot drive/sample the
     * SCL and SDA pads: all I2C probe transfers time out silently and sensors
     * are not detected (DPS310, external compass, etc.).
     * This mirrors the SPI pin setup block above and must run after halInit()
     * so that _pal_lld_init()'s reset-unreset of IO_BANK0/PADS_BANK0 does
     * not clobber the FUNCSEL settings. */
#if defined(HAL_GPIO_PIN_I2C0_SCL)
    palSetLineMode(HAL_GPIO_PIN_I2C0_SCL, PAL_MODE_ALTERNATE_I2C);
    palSetLineMode(HAL_GPIO_PIN_I2C0_SDA, PAL_MODE_ALTERNATE_I2C);
#endif
#if defined(HAL_GPIO_PIN_I2C1_SCL)
    palSetLineMode(HAL_GPIO_PIN_I2C1_SCL, PAL_MODE_ALTERNATE_I2C);
    palSetLineMode(HAL_GPIO_PIN_I2C1_SDA, PAL_MODE_ALTERNATE_I2C);
#endif

    /* Configure SPI CS pins as output-HIGH (deasserted).
     * On RP2350, GPIO pads reset to FUNCSEL=31 (NULL / high-Z).  The ChibiOS
     * SPI_SELECT_MODE_PAD path drives CS via palClearPad/palSetPad, which writes
     * to the SIO output register.  For those writes to appear on the pad, the pin
     * must be:
     *   (a) FUNCSEL=5 (SIO)  — routes SIO output to the GPIO pad, AND
     *   (b) GPIO_OE=1        — enables the pad driver.
     * PAL_MODE_OUTPUT_PUSHPULL provides both.  SIO_OUT is written HIGH first
     * (palSetLine) to pre-arm the latch before OE is asserted, preventing a
     * brief low-going glitch on CS that would falsely select a device.
     * If a CS macro is not defined (sensor not fitted), the block is omitted. */
#if defined(HAL_GPIO_PIN_MAG_CS)
    palSetLine(HAL_GPIO_PIN_MAG_CS);
    palSetLineMode(HAL_GPIO_PIN_MAG_CS, PAL_MODE_OUTPUT_PUSHPULL);
#endif
#if defined(HAL_GPIO_PIN_MPU_CS)
    palSetLine(HAL_GPIO_PIN_MPU_CS);
    palSetLineMode(HAL_GPIO_PIN_MPU_CS, PAL_MODE_OUTPUT_PUSHPULL);
#endif
#if defined(HAL_GPIO_PIN_BARO_EXT_CS)
    palSetLine(HAL_GPIO_PIN_BARO_EXT_CS);
    palSetLineMode(HAL_GPIO_PIN_BARO_EXT_CS, PAL_MODE_OUTPUT_PUSHPULL);
#endif
#if defined(HAL_GPIO_PIN_GYRO_EXT_CS)
    palSetLine(HAL_GPIO_PIN_GYRO_EXT_CS);
    palSetLineMode(HAL_GPIO_PIN_GYRO_EXT_CS, PAL_MODE_OUTPUT_PUSHPULL);
#endif

  /* Configure board status LEDs as SIO outputs.
   * Some RP2350 bring-up paths can leave GPIOs in a non-SIO function after
   * reset-domain churn; explicitly restoring push-pull output mode here keeps
   * board LED behavior deterministic for AP_Notify and hardware diagnostics.
   * We do not force output level in this block so existing hwdef/AP_Notify
   * polarity and state ownership remain unchanged. */
#if defined(HAL_GPIO_PIN_LED_BLUE)
  palSetLineMode(HAL_GPIO_PIN_LED_BLUE, PAL_MODE_OUTPUT_PUSHPULL);
#endif
#if defined(HAL_GPIO_PIN_LED_GREEN)
  palSetLineMode(HAL_GPIO_PIN_LED_GREEN, PAL_MODE_OUTPUT_PUSHPULL);
#endif

  /* Configure board regulator-enable lines as SIO push-pull outputs at the
   * correct initial levels declared in hwdef.dat (OUTPUT HIGH / OUTPUT LOW).
   *
   * These pins revert to FUNCSEL=31 (NULL, Hi-Z) with PAD_BANK0 reset
   * default PDE=1 (pull-down) after _pal_lld_init() resets the IO_BANK0
   * and PADS_BANK0 peripherals.  Without this block BEC_5V_EN (GPIO14) is
   * left floating LOW, which disables the 5 V BEC at boot and cuts power to
   * any load on that rail before ArduPilot's runtime code can assert it.
   *
   * Set the output latch (palSetLine / palClearLine) before asserting OE via
   * palSetLineMode so the pad instantaneously shows the correct level the
   * moment the driver is enabled, with no unwanted transient glitch. */
#if defined(HAL_GPIO_PIN_BEC_5V_EN)
  palSetLine(HAL_GPIO_PIN_BEC_5V_EN);                /* pre-arm latch HIGH (5 V BEC enabled) */
  palSetLineMode(HAL_GPIO_PIN_BEC_5V_EN, PAL_MODE_OUTPUT_PUSHPULL);  /* FUNCSEL=5 + OE */
#endif
#if defined(HAL_GPIO_PIN_BEC_9V_EN)
  palClearLine(HAL_GPIO_PIN_BEC_9V_EN);              /* pre-arm latch LOW (9 V BEC disabled at boot) */
  palSetLineMode(HAL_GPIO_PIN_BEC_9V_EN, PAL_MODE_OUTPUT_PUSHPULL);  /* FUNCSEL=5 + OE */
#endif
}
#endif //PIC02_AVAILABLE

/**
 * @brief   Early initialization code.
 * @details This initialization must be performed just after stack setup
 *          and before any other initialization.
 *
 * You must not rely on: 1) BSS variables being cleared 2) DATA Variables being initialized 3) RAM functions to be in RAM
 * You can rely on: 1) const variables or tables 2) flash code 3) automatic variables
 */
void __early_init(void) {
    #if STM32_AVAILABLE == TRUE && !defined(STM32F1)
      stm32_gpio_init();
    #endif
    #if STM32_AVAILABLE == TRUE && !defined(HAL_XIP_ENABLED) || defined(HAL_FORCE_CLOCK_INIT)
      // if running from external flash then the clocks must not be reset - instead rely on the bootloader to setup
      stm32_clock_init();
    #endif
    #if defined(HAL_DISABLE_DCACHE)
      SCB_DisableDCache();
    #endif
    #if defined(STM32H7)
      // ensure ITCM and DTCM are enabled. These could be disabled by the px4
      // bootloader
      SCB->ITCMCR |= 1; // ITCM enable
      SCB->DTCMCR |= 1; // DTCM enable

        #ifdef STM32_NOCACHE_MPU_REGION_1
          // disable cache on configured regions so they can be used for DMA
          // this requires some coordination with the memory map in the MCU configuration script
          mpuConfigureRegion(STM32_NOCACHE_MPU_REGION_1,
                            STM32_NOCACHE_MPU_REGION_1_BASE,
                            MPU_RASR_ATTR_AP_RW_RW |
                            MPU_RASR_ATTR_NON_CACHEABLE |
                            STM32_NOCACHE_MPU_REGION_1_SIZE |
                            MPU_RASR_ENABLE);
        #endif
        #ifdef STM32_NOCACHE_MPU_REGION_2
          mpuConfigureRegion(STM32_NOCACHE_MPU_REGION_2,
                            STM32_NOCACHE_MPU_REGION_2_BASE,
                            MPU_RASR_ATTR_AP_RW_RW |
                            MPU_RASR_ATTR_NON_CACHEABLE |
                            STM32_NOCACHE_MPU_REGION_2_SIZE |
                            MPU_RASR_ENABLE);
        #endif
      #if defined(DUAL_CORE)
        stm32_disable_cm4_core(); // disable second core
      #endif
    #endif
    #if PIC02_AVAILABLE == TRUE
      /*
       * RP2350 note (Pico2/ArduPilot bootloader):
       * The application vector table lives at __vectors_base__ in flash
       * (typically app_flash_base + 0x80). If we don't explicitly point
       * VTOR at that table then external IRQs (for example I2C1 IRQ37)
       * are taken via the boot ROM vector table and can end up in the
       * default _unhandled_exception handler even though the real ISR
       * (for example VectorD4 for I2C1) is linked into the image.
       *
       * Set VTOR as early as possible so all peripheral IRQs land in the
       * correct ChibiOS handlers.
       */
      extern uint32_t __vectors_base__[];

      SCB->VTOR = (uint32_t)__vectors_base__;
      __DSB();
      __ISB();

#if RP_NO_INIT == FALSE
      /*
       * Reset all peripherals except those needed for XIP flash and the PLLs.
       * IO_QSPI / PADS_QSPI must stay unreset because the CPU is executing
       * from flash via XIP.  PLL_SYS / PLL_USB are handled below by
       * rp_clock_init() after it first switches CLK_SYS to the safe ROSC.
       *
       * This call was previously made inside hal_lld_init() but ChibiOS
       * commit a7485da935 ("Fix RP Peripheral Init and move clock setup early")
       * moved it here.  Without it, the RP2350 peripheral reset state is
       * whatever the ROM or bootloader left it — often some peripherals are
       * still held in reset — and rp_clock_init() may not have a clean
       * RESETS_DONE handshake.
       */
      rp_peripheral_reset(~(RESETS_ALLREG_IO_QSPI  | RESETS_ALLREG_PADS_QSPI |
                             RESETS_ALLREG_PLL_SYS  | RESETS_ALLREG_PLL_USB));

      /*
       * Configure PLL_SYS (→ 150 MHz system clock) and PLL_USB (→ 48 MHz).
       *
       * Also previously in hal_lld_init() and moved to __early_init() in the
       * same ChibiOS commit.  Without this call the RP2350 runs on the ROSC
       * (~12 MHz), the USB PLL never locks, and USB CDC never enumerates.
       * ChibiOS still starts (ROSC is stable), but all timing is ~12× off
       * and any USB-dependent path (MAVLink, uploader.py) silently freezes.
       */
      rp_clock_init();
#endif /* RP_NO_INIT */

      /* Configure SPI CS pins and PWM GPIOs now that IO_BANK0 is out of reset. */
      pico2_gpio_init();
    #endif
}

#if defined(RP_CORE1_START) && RP_CORE1_START == TRUE
/*
 * Startup verification: ensure the bare-metal core1 FIFO dispatcher
 * round-trip works before any real dispatches are attempted.
 *
 * c1_startup_result:
 *   0xDEADC1C1 — set by core1 when it executes c1_startup_ping()
 *   0xBADC1BAD — timeout (core1 did not reach idle in time)
 */
volatile uint32_t c1_startup_result = 0U;

static void c1_startup_ping(void)
{
    c1_startup_result = 0xDEADC1C1U;
}
#endif /* RP_CORE1_START */

void __late_init(void) {

#if PIC02_AVAILABLE == TRUE
  /*
   * The bootloader leaves USB running (NVIC enabled, controller active).
   * Before halInit() zeroes the USB driver struct (USBD1.config=NULL),
   * we must prevent the USB ISR from firing, otherwise the first SOF
   * from the host hits usb_lld_serve_interrupt() with a NULL config
   * pointer, which reads ROM[12] (the ROM HardFault vector) and jumps
   * there, ending in the ROM debug BKPT trap at 0x000002f8.
   *
   * Fix: disable the USB NVIC IRQ and hold USB in peripheral reset
   * right here. usb_lld_start() will unreset and re-enable it later.
   */
  nvicDisableVector(RP_USBCTRL_IRQ_NUMBER);
  rp_peripheral_reset(RESETS_ALLREG_USBCTRL);

  /*
   * Clear all external IRQ pending bits before halInit() fires ChibiOS drivers.
   *
   * RP2350 NVIC pending bits (NVIC_ISPR) are NOT guaranteed cleared by
   * SYSRESETREQ — they survive the reset and live across boots.  This means
   * any IRQ that fired as unhandled in a previous boot (e.g. I2C0=IRQ36,
   * SPI1=IRQ32) still has its ISPR pending bit set at the start of the next
   * boot.  The first chSysInit() CPSIE immediately dispatches those stale
   * pending IRQs before ChibiOS drivers have registered their handlers,
   * landing in _unhandled_exception → NVIC_SystemReset → reset loop.
   *
   * Writing all-ones to NVIC_ICPR (interrupt-clear-pending) clears every
   * pending bit that exists.  halInit() and each ChibiOS peripheral driver
   * will re-arm only the IRQs they actually own.
   *
   * RP2350 has 52 external IRQs (0..51), so ICPR[0] covers 0..31 and
   * ICPR[1] covers 32..51 (upper bits 52..63 are unimplemented, writes ignored).
   */
  NVIC->ICPR[0] = 0xFFFFFFFFU;
  NVIC->ICPR[1] = 0xFFFFFFFFU;

  /*
   * Proactively silence TIMER1 at the peripheral level before halInit().
   *
   * TIMER1->INTS is a level-driven output (= TIMER1->INTR & TIMER1->INTE).
   * Setting INTE=0 forces INTS=0 regardless of INTR, which de-asserts the
   * NVIC input lines for IRQ4-7 (TIMER1_IRQ0-3).  Without this, the NVIC
   * ICPR scrub above clears the pending bits momentarily, but the peripheral
   * immediately re-asserts them (the ROM/bootloader leaves alarms armed with
   * past target times), causing halInit() CPSIE to immediately take Vector50.
   */
  TIMER1->INTE  = 0U;           /* de-assert IRQ4-7 NVIC input lines */
  TIMER1->INTR  = 0xFU;         /* W1C: clear all 4 raw alarm pending bits */
  TIMER1->ARMED = 0xFU;         /* W1C: disarm all 4 alarms */
  __DSB();                       /* ensure APB writes reach peripheral */

  /*
   * Ensure the XIP cache does not hold stale vector table lines (or other
   * early-startup code) when the firmware was reflashed via SWD and reset
   * using SYSRESETREQ.
   *
   * This must run after the C runtime has initialized .data, because the
   * helper is located in .ramtext and executes from RAM.
   */
  rp2350_invalidate_xip_cache();

  /*
   * Relocate the vector table into SRAM.
   *
   * On the RP2350 we have observed external IRQs occasionally dispatching to
   * stale/default handlers after SWD reflashing + SYSRESETREQ reset, despite
   * the flash vector table containing the correct ISR entry.
   *
   * Moving the vector table into SRAM makes IRQ dispatch deterministic and
   * avoids any dependency on XIP flash caching behavior for vector fetches.
   *
   * The VTOR hardware enforces bits[6:0]=0 (128-byte minimum), but the ARM
   * Cortex-M33 architecture spec requires the table to be aligned to the next
   * power-of-two >= table size.  With 72 entries (16 system + 56 external) the
   * table is 288 bytes, so the required alignment is 512 bytes.  Using only
   * 128-byte alignment leaves bits[8:7] of the VTOR base potentially non-zero,
   * which violates the spec and may cause the CPU to fetch wrong vector entries
   * for high-numbered IRQs.
   *
   * The RP2350 port defines CORTEX_NUM_VECTORS=56 (see devices/RP2350/cmparams.h),
   * so the full table is 16 system exceptions + 56 external IRQ vectors = 288 bytes.
   * Next power-of-2 >= 288 = 512.
   */
  extern uint32_t __vectors_base__[];
  enum { RP2350_VECTOR_WORDS = 16U + 56U };
  static uint32_t rp2350_vectors[RP2350_VECTOR_WORDS] __attribute__((aligned(512)));
  const uint32_t *flash_vectors = __vectors_base__;

  for (uint32_t i = 0; i < RP2350_VECTOR_WORDS; i++) {
      rp2350_vectors[i] = flash_vectors[i];
  }

  /*
   * NOTE: SCB->VTOR is set to rp2350_vectors AFTER halInit() below.
   * hal_lld_init() (called inside halInit) resets VTOR to &_vectors (the
   * flash address), so any VTOR write here would be immediately overwritten.
   * The assignment is deferred to the post-halInit() #if PIC02_AVAILABLE
   * block so the RAM vector table is the final, active table.
   */

  /*
   * RP2350 UART RX pads: at chip reset PADS_BANK0 has ISO=1 + PDE=1 + IE=0.
   * Setting 0x5A (PUE+IE+SCHMITT) prevents any early glitch on the RX lines
   * before UARTDriver::_begin() runs.  Neither halInit() / pal_lld_init()
   * nor sio_lld_start() configure per-pin GPIO CTRL or PADS registers; the
   * actual FUNCSEL=2 (UART) and permanent pull-up are applied later in
   * UARTDriver::_begin() via palSetLineMode(PAL_MODE_ALTERNATE_UART) followed
   * by palLineSetPushPull(PULLUP) before sioStart().
   */
  PADS_BANK0->GPIO[13] = 0x5AU;  /* GPIO13 = UART0_RX — early PUE+IE+SCHMITT */
  PADS_BANK0->GPIO[11] = 0x5AU;  /* GPIO11 = UART1_RX — early PUE+IE+SCHMITT */

#if defined(RP_CORE1_START) && RP_CORE1_START == TRUE
  /*
   * Drain the SIO inter-core FIFO before core1 is started.
   *
   * The RP2350 ROM core1 launch handshake leaves residual data in the FIFO.
   * Drain it so the first real dispatch message from c1_run_sync() is not
   * lost or misinterpreted by core1's bare-metal dispatcher loop.
   */
  SIO->FIFO_ST = SIO_FIFO_ST_ROE | SIO_FIFO_ST_WOF;   /* clear error flags */
  while ((SIO->FIFO_ST & SIO_FIFO_ST_VLD) != 0U) {
    (void)SIO->FIFO_RD;                                 /* drain stale data  */
  }
#endif  /* RP_CORE1_START */
#endif  /* PIC02_AVAILABLE */

  halInit();

#if PIC02_AVAILABLE == TRUE
  /*
   * Re-apply GPIO configuration after halInit().
   *
   * pico2_gpio_init() is also called from __early_init() but SIO GPIO_OE
   * is cleared somewhere during the RP2350 startup reset sequence before
   * hal_lld_init() completes.  Calling it again here, after halInit() has
   * finished initialising all ChibiOS HAL drivers (including pal_lld_init),
   * ensures SPI CS pins are driven HIGH (output mode) by the time the
   * application starts.
   */
  pico2_gpio_init();

  /*
   * Activate the RAM vector table that was populated from flash before
   * halInit() above.  We do this here — after halInit() / hal_lld_init() —
   * because hal_lld_init() unconditionally resets SCB->VTOR to &_vectors
   * (the flash alias, 0x10010080 for Laurel).  Setting VTOR now overrides
   * that and makes all subsequent IRQ dispatches use the SRAM copy instead.
   *
   * Why RAM is better than flash for the vector table on RP2350:
   *   - The XIP cache can serve stale lines after SWD reflash + SYSRESETREQ.
   *     A stale cache entry at a vector slot makes the CPU jump to the old
   *     (pre-flash) handler address, causing an _unhandled_exception crash.
   *   - SRAM accesses are deterministic; no cache coherency concern.
   *   - rp2350_vectors[] was filled from flash after rp2350_invalidate_xip_cache()
   *     flushed stale lines, so the content is guaranteed correct.
   *
   * The Cortex-M33 TBLOFF requires 128-byte alignment; rp2350_vectors is
   * __attribute__((aligned(128))) so this write is always valid.
   */
  SCB->VTOR = (uint32_t)rp2350_vectors;
  __DSB();
  __ISB();
#endif

#ifdef HAL_USB_PRODUCT_ID
  /*
   * Populate USB string descriptors AFTER halInit() so HAL state is fully
   * set up before we access OTP (serial number) and write vcom_strings[].
   * Must happen BEFORE chSysInit() starts the USB driver so the host
   * receives valid string descriptors during the first enumeration attempt.
   * This mirrors the approach used in the bootloader __late_init().
   */
  setup_usb_strings();
#endif

  chSysInit();

#if defined(RP_CORE1_START) && RP_CORE1_START == TRUE
  /*
   * Verify core1 bare-metal FIFO dispatcher round-trip at startup.
   * Poll c1_boot_stage until core1 reaches idle (0x4D) with a 200 ms
   * timeout, then dispatch c1_startup_ping() and check the result.
   * c1_startup_result is readable via OpenOCD: should be 0xDEADC1C1.
   */
  {
      extern volatile uint32_t c1_boot_stage;
      /* Wait for core1 to reach its idle loop (stage 0x4D) before
       * dispatching.  Worst case is the stack fill (~8 KB at 250 MHz ≈
       * ~60 µs).  Limit to 1 M iterations (~4 ms at 250 MHz) so we
       * never block long enough to trigger the watchdog. */
      for (volatile uint32_t i = 0U; i < 1000000UL && c1_boot_stage != 0x4DU; i++) {}
      if (c1_boot_stage == 0x4DU) {
          c1_run_sync(c1_startup_ping);
      } else {
          c1_startup_result = 0xBADC1BADU;
      }
  }
#endif /* RP_CORE1_START */

  /*
   * Start EFL driver for RP2350 QSPI flash read/write/erase support.
   * Must be called after halInit() which runs eflInit().
   */
  #if PIC02_AVAILABLE == TRUE && HAL_USE_EFL == TRUE
    eflStart(&EFLD1, NULL);
  #endif

  /*
   * Initialize RNG
   */
  #if HAL_USE_HW_RNG && defined(RNG)
    rccEnableAHB2(RCC_AHB2ENR_RNGEN, 0);
    RNG->CR |= RNG_CR_IE;
    RNG->CR |= RNG_CR_RNGEN;
  #endif

  #if STM32_AVAILABLE == TRUE
    stm32_watchdog_save_reason();
    #ifndef HAL_BOOTLOADER_BUILD
      stm32_watchdog_clear_reason();
    #endif
  #endif
  #if CH_CFG_USE_HEAP == TRUE
    malloc_init();
  #endif

  #if defined(STM32_AVAILABLE) && defined(HAL_FLASH_SET_NRST_MODE)
    // ensure NRST_MODE is set correctly
    stm32_flash_set_NRST_MODE(HAL_FLASH_SET_NRST_MODE);
  #endif
}

#if HAL_USE_SDC || defined(__DOXYGEN__)
    /**
    * @brief   SDC card detection.
    */
    bool sdc_lld_is_card_inserted(SDCDriver *sdcp) {
        (void)sdcp;
        return true;
    }

    /**
    * @brief   SDC card write protection detection.
    */
    bool sdc_lld_is_write_protected(SDCDriver *sdcp) {

      (void)sdcp;
      return false;
    }
#endif /* HAL_USE_SDC */

#if HAL_USE_MMC_SPI || defined(__DOXYGEN__)
    /**
    * @brief   MMC_SPI card detection.
    */
    bool mmc_lld_is_card_inserted(MMCDriver *mmcp) {
      (void)mmcp;
      /* TODO: Fill the implementation.*/
      return true;
    }

    /**
    * @brief   MMC_SPI card write protection detection.
    */
    bool mmc_lld_is_write_protected(MMCDriver *mmcp) {
      (void)mmcp;
      /* TODO: Fill the implementation.*/
      return false;
    }
#endif

/**
 * @brief   Board-specific initialization code.
 * @todo    Add your board-specific code, if any.
 */
void boardInit(void) {
#if defined(RP2350B_QFN80) && (HAL_USE_ADC == TRUE)
  /*
   * RP2350B QFN-80 analog inputs are on GPIO40-47 (AINSEL 0-7).
   * Laurel uses GPIO40/41/42 for battery voltage/current and RSSI, so force
   * those pads into analog mode early to disable digital input and pulls.
   */
  adcRPGpioInit(40U);
  adcRPGpioInit(41U);
  adcRPGpioInit(42U);
#endif
  HAL_BOARD_INIT_HOOK_CALL;
}

#if defined(RP_CORE1_START) && RP_CORE1_START == TRUE
/*
 * c1_run_sync() — dispatch a function to core1 and block until done.
 *
 * Core1 runs a bare-metal FIFO dispatcher loop (c1_main.c).  It waits for
 * a 32-bit function pointer from core0's SIO FIFO_WR, calls the function,
 * then writes 1 back to signal completion.
 *
 * Protocol:
 *   core0 → SIO FIFO_WR: (uint32_t)fn   (fn must not use ChibiOS blocking calls)
 *   core1 → SIO FIFO_WR: 1              (done signal)
 *
 * Memory ordering: DMB barriers ensure all stores made by core0 before the
 * call are visible to core1, and all stores made by core1 (computation
 * results) are visible to core0 after the call returns.
 *
 * Must only be called after core1 has started (i.e. after hal_lld_init()
 * has run start_core1()).  Safe to call from any ChibiOS thread on core0.
 */
/* Count of c1_run_sync() fallbacks due to core1 timeout. Visible via OpenOCD. */
volatile uint32_t c1_timeout_count;

/* Per-type dispatch counts for the 10-second dual-core utilisation report.
 * Updated by c1_run_sync_locked() for EKF and c1_try_run_sync() for PID. */
volatile uint32_t c1_ekf_c1_count;  /* EKF covariance dispatches that ran on Core1  */
volatile uint32_t c1_ekf_c0_count;  /* EKF covariance dispatches that fell back to Core0 */
volatile uint32_t c1_pid_c1_count;  /* PID rate-loop dispatches that ran on Core1  */
volatile uint32_t c1_pid_c0_count;  /* PID rate-loop dispatches that fell back to Core0 */

/*
 * Dispatch serialisation: multiple ChibiOS threads (main scheduler for EKF
 * covariance, high-rate rate_thread for PID) may both want core1 at once.
 * Without serialisation the SIO FIFO write + completion-read pair of one
 * caller can interleave with the other, causing each to consume the wrong
 * "done = 1" signal.
 *
 * Strategy:
 *   • c1_run_sync_locked()  — used by EKF: acquires the mutex, dispatches,
 *                             releases.  The ~3000 µs wait with chThdSleep
 *                             inside c1_run_sync() still yields 10 µs slices
 *                             to lower-priority threads (GCS, USB TX).
 *   • c1_try_run_sync()     — used by rate_thread: non-blocking TryLock.
 *                             If EKF already holds the mutex, PID computation
 *                             falls back to core0 immediately rather than
 *                             stalling the 1 kHz control loop for 3000 µs.
 */
static MUTEX_DECL(c1_dispatch_mtx);

/*
 * c1_run_sync_locked() — serialised c1_run_sync().
 * Blocks until the dispatch mutex is free, then dispatches fn to core1.
 * Use for callers that must have core1 results eventually (e.g. EKF).
 */
void c1_run_sync_locked(void (*fn)(void))
{
    chMtxLock(&c1_dispatch_mtx);
    bool on_c1 = c1_run_sync(fn);
    chMtxUnlock(&c1_dispatch_mtx);
    /* Count whether EKF covariance ran on Core1 or fell back to Core0. */
    if (on_c1) { c1_ekf_c1_count++; } else { c1_ekf_c0_count++; }
}

/*
 * c1_try_run_sync() — best-effort core1 dispatch with immediate fallback.
 * Attempts a non-blocking TryLock on the dispatch mutex.  If the mutex is
 * already held (e.g. EKF covariance is running), fn() executes on core0
 * instead and the function returns false.  This avoids stalling a
 * high-priority caller (rate_thread) for the full ~3000 µs EKF covariance
 * window.  Returns true if core1 was used, false if core0 fallback was used.
 */
bool c1_try_run_sync(void (*fn)(void))
{
    if (!chMtxTryLock(&c1_dispatch_mtx)) {
        /* Core1 is busy (EKF holding mutex) — run fn on core0 immediately. */
        fn();
        __DMB();
        c1_pid_c0_count++;  /* mutex-contention fallback */
        return false;
    }
    bool on_c1 = c1_run_sync(fn);
    chMtxUnlock(&c1_dispatch_mtx);
    /* Count whether PID ran on Core1 or timed out and fell back to Core0. */
    if (on_c1) { c1_pid_c1_count++; } else { c1_pid_c0_count++; }
    return on_c1;
}

// this is the core0 handler to talk with core1
bool c1_run_sync(void (*fn)(void))
{
    /* Release barrier: ensure preceding stores are visible to core1 before
     * we write the function pointer to the FIFO. */
    __DMB();

    /* Wait for TX FIFO space.  If core1 has crashed and returned to ROM, it
     * will no longer consume FIFO entries; after 4 calls the TX FIFO fills
     * and RDY goes low.  Time out after 1 ms and fall back to core0.
     * Use chThdSleep(10µs) on each retry so ChibiOS can schedule
     * lower-priority threads (main loop, GCS) while we wait. */
    const uint32_t t0 = hrt_micros32();
    // what is SIO->FIFO_ST ? its bit 0 is RDY (ready for write), bit 1 is VLD (valid data to read), bit 2 is ROE (read overrun error), bit 3 is WOF (write overrun error)

    while (!(SIO->FIFO_ST & SIO_FIFO_ST_RDY)) {
        if (hrt_micros32() - t0 > 1000U) {
            c1_timeout_count++;
            fn();
            __DMB();
            return false;  /* Core0 fallback — Core1 not consuming FIFO */
        }
        chThdSleep(TIME_US2I(10));
    }
    // execute it on core 1. write the function pointer to the FIFO
    SIO->FIFO_WR = (uint32_t)fn;

    /* Core1 sits in a bare-metal WFE loop with no NVIC setup (IRQ26 is not
     * enabled), so the FIFO write alone does not wake it.  An explicit SEV
     * is required — the same pattern used by ChibiOS fifoBlockingWrite(). */
    __SEV();

    /* Wait for core1's done signal (value = 1).  Time out after 1 ms and
     * fall back to running fn() on core0 so the caller is not stalled
     * forever if core1 has faulted (e.g. HardFault → ROM LockUp).
     * chThdSleep(10µs) between retries yields the CPU in 10µs chunks so the
     * main loop and GCS threads can run while core1 computes (prevents
     * priority-182 starvation of the 350 Hz main loop).  10µs balances
     * low context-switch overhead (~10 switches per 100µs core1 slice) with
     * giving lower-priority threads adequate CPU time. */
    const uint32_t t1 = hrt_micros32();
    while (!(SIO->FIFO_ST & SIO_FIFO_ST_VLD)) {
        if (hrt_micros32() - t1 > 1000U) {
            c1_timeout_count++;
            fn();
            __DMB();
            return false;  /* Core0 fallback — Core1 did not signal completion in time */
        }
        chThdSleep(TIME_US2I(10));
    }
    (void)SIO->FIFO_RD;

    /* Acquire barrier: ensure core1's stores are visible before we read
     * any data updated by fn() (e.g. PID outputs in attitude_control). */
    __DMB();
    return true;  /* fn() ran and completed on Core1 */
}
#endif  /* RP_CORE1_START */
