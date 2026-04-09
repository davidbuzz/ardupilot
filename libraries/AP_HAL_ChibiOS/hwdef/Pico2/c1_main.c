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
 */

/**
 * @file    Pico2/c1_main.c
 * @brief   RP2350 core1 bare-metal function-pointer dispatcher for ArduPilot.
 *
 * Architecture: "Core0 schedules, Core1 runs"
 * ============================================
 * Core0 runs ChibiOS (CH_CFG_SMP_MODE=FALSE) and owns all scheduling.
 * Core1 runs bare-metal — no ChibiOS instance, no scheduler, no NVIC setup.
 *
 * Protocol (SIO FIFO, 32-bit words):
 *   Core0 → Core1  (core1's RX FIFO):  function pointer cast to uint32_t
 *   Core1 → Core0  (core0's RX FIFO):  1 = task completed, 0 = idle ping
 *
 * Core1 polls its RX FIFO (SIO->FIFO_ST bit VLD), reads the function
 * pointer, calls it, then writes a completion word back so core0 can
 * optionally synchronize.  WFE is used when the FIFO is empty so core1
 * does not busy-spin and waste power.
 *
 * Boot canary (c1_boot_stage):
 *   0xDEAD0000 = never reached c1_main (stuck in CRT0)
 *   1           = __c1_cpu_init ran
 *   2           = __c1_late_init ran
 *   3           = c1_main entered
 *   0x4D        = idle — waiting for next function pointer from core0
 *   0x50        = running task
 *   0x51        = task returned, sending done signal
 */

#include <stdint.h>

/*
 * This translation unit is only added by the Pico2 board makefile and the
 * generated CRT0 trampoline always expects c1_main() to exist when that
 * build path is selected. Keep the dispatcher symbols unconditional here so
 * link success does not depend on external preprocessor state or stale build
 * dependencies.
 */

/*
 * RP2350 SIO register block (section 2.3.1 of the RP2350 datasheet).
 * Only the FIFO registers needed here.
 */
#define SIO_BASE        0xD0000000UL
#define SIO_FIFO_ST     (*(volatile uint32_t *)(SIO_BASE + 0x050))
#define SIO_FIFO_WR     (*(volatile uint32_t *)(SIO_BASE + 0x054))
#define SIO_FIFO_RD     (*(volatile uint32_t *)(SIO_BASE + 0x058))

#define FIFO_ST_VLD     (1u << 0)   /* RX FIFO has data */
#define FIFO_ST_RDY     (1u << 1)   /* TX FIFO has space */
#define FIFO_ST_ROE     (1u << 3)   /* RX overflow error */
#define FIFO_ST_WOF     (1u << 2)   /* TX underflow error */

/*
 * TIMER0 TIMERAWL — free-running 1 MHz counter, safe for multi-core reads.
 * Unlike TIMELR, reading TIMERAWL does NOT latch TIMERAWH so it is safe
 * to read from Core1 concurrently with Core0 without coordination.
 * Used to measure Core1 busy time in microseconds for CPU% reporting.
 */
#define TIMER0_TIMERAWL  (*(volatile uint32_t *)0x400B0028U)

/*
 * Core-Private NVIC registers (PPB, banked per-core on RP2350).
 * RP2350 has 52 external IRQs → two 32-bit words.
 */
#define NVIC_ICER0  (*(volatile uint32_t *)0xE000E180U)
#define NVIC_ICER1  (*(volatile uint32_t *)0xE000E184U)
#define NVIC_ICPR0  (*(volatile uint32_t *)0xE000E280U)
#define NVIC_ICPR1  (*(volatile uint32_t *)0xE000E284U)

/* Boot-stage canary — visible to OpenOCD/GDB after halt. */
volatile uint32_t c1_boot_stage = 0xDEAD0000U;

/*
 * Accumulated microseconds Core1 has spent executing dispatched functions.
 * Stamped on entry/exit of every fn() call using TIMER0_TIMERAWL.
 * Core0 reads this every 10 s to compute Core1 CPU utilisation.
 */
volatile uint32_t c1_busy_us __attribute__((used, externally_visible)) = 0U;

/*
 * Override the weak __c1_cpu_init stub.
 * Called from _crt0_c1_entry after MSP/PSP/VTOR/FPU are set up.
 */
void __c1_cpu_init(void)
{
    /*
     * Scrub Core 1's own NVIC immediately on entry.
     *
     * The RP2350 ROM Core 1 launch protocol leaves SPARE_IRQ_1 (IRQ 47,
     * VectorFC) enabled in Core 1's banked NVIC after the 6-step FIFO
     * handshake. Without this scrub, the first WFE wakeup fires VectorFC →
     * weak _unhandled_exception → NVIC_SystemReset() and the board resets.
     *
     * Core 1 is bare-metal (WFE/SEV + SIO FIFO polling) and uses no NVIC
     * IRQs, so disable and clear all 52 external IRQ lines.
     */
    NVIC_ICER0 = 0xFFFFFFFFU;  /* disable IRQs  0..31 */
    NVIC_ICER1 = 0xFFFFFFFFU;  /* disable IRQs 32..51 (SPARE_IRQ_1=47 here) */
    NVIC_ICPR0 = 0xFFFFFFFFU;  /* clear pending 0..31 */
    NVIC_ICPR1 = 0xFFFFFFFFU;  /* clear pending 32..51 */
    __asm volatile ("dsb sy\n isb" ::: "memory");

    c1_boot_stage = 1U;
}

/*
 * Override the weak __c1_late_init stub.
 * Called from _crt0_c1_entry after CONTROL=PSP and stack fill.
 */
void __c1_late_init(void)
{
    c1_boot_stage = 2U;
}

/**
 * Core1 entry point — called from _crt0_c1_entry after stack/FPU init.
 *
 * Runs entirely bare-metal. Core1 has NO ChibiOS instance: no
 * chInstanceObjectInit, no scheduler, no PendSV, no NVIC setup.
 * All scheduling decisions remain on core0's ChibiOS instance (ch0).
 *
 * Core1 sits in a WFE loop waiting for core0 to push a function pointer
 * via the SIO inter-core FIFO.  On receipt it calls the function and
 * signals done.  The calling convention is:
 *
 *   typedef void (*c1_task_fn)(void);
 *   // core0 side:
 *   SIO->FIFO_WR = (uint32_t)(c1_task_fn)my_function;
 *   // core1 will call my_function() then write 1 back.
 */
void c1_main(void)
{
    c1_boot_stage = 3U;

    /* Clear any stale FIFO error flags and drain residual data left by
     * the ROM core1 launch handshake before we start using the FIFO. */
    SIO_FIFO_ST = FIFO_ST_ROE | FIFO_ST_WOF;
    while (SIO_FIFO_ST & FIFO_ST_VLD) {
        (void)SIO_FIFO_RD;
    }

    while (1) {
        c1_boot_stage = 0x4DU;  /* idle — waiting for work */

        /* Sleep until core0 signals us (SEV from SIO write wakes WFE). */
        while (!(SIO_FIFO_ST & FIFO_ST_VLD)) {
            __asm volatile ("wfe");
        }

        /* Read the function pointer sent by core0. */
        uint32_t msg = SIO_FIFO_RD;

        if (msg == 0U) {
            /* Null pointer = keepalive ping, just acknowledge. */
            while (!(SIO_FIFO_ST & FIFO_ST_RDY)) { /* spin */ }
            SIO_FIFO_WR = 0U;
            continue;
        }

        c1_boot_stage = 0x50U;  /* running task */

        /* Call the function.  It must run to completion (no ChibiOS
         * blocking calls allowed on core1). */
        typedef void (*c1_task_fn)(void);
        const uint32_t _c1_t0 = TIMER0_TIMERAWL;
        ((c1_task_fn)msg)();
        c1_busy_us += TIMER0_TIMERAWL - _c1_t0;  /* accumulate Core1 busy time in µs */

        c1_boot_stage = 0x51U;  /* task returned, sending done */

        /* DMB: ensure fn()'s stores are visible to core0 before the done
         * signal is observed.  Without this, the Cortex-M33 store buffer
         * could allow core0 to read the FIFO word before seeing the result
         * of fn() (e.g. c1_startup_result written by c1_startup_ping). */
        __asm volatile ("dmb sy" ::: "memory");

        /* Signal completion to core0.  Spin-wait if TX FIFO is full
         * (should never happen in normal operation). */
        while (!(SIO_FIFO_ST & FIFO_ST_RDY)) { /* spin */ }
        SIO_FIFO_WR = 1U;  /* done */

        /* SEV: wake core0 if it is ever changed to use WFE instead of
         * spinning.  Harmless with the current spin-loop implementation. */
        __asm volatile ("sev");
    }
}

