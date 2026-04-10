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
 * @file    Laurel/c1_main.c
 * @brief   RP2350 core1 bare-metal function-pointer dispatcher for Laurel.
 *
 * Architecture: "Core0 schedules, Core1 runs"
 * ============================================
 * Core0 runs ChibiOS (CH_CFG_SMP_MODE=FALSE) and owns all scheduling.
 * Core1 runs bare-metal — no ChibiOS instance, no scheduler, no NVIC setup.
 *
 * Protocol (SIO FIFO, 32-bit words):
 *   Core0 → Core1  (core1's RX FIFO):  function pointer cast to uint32_t
 *   Core1 → Core0  (core0's RX FIFO):  1 = task completed, 0 = idle ping
 */

#include <stdint.h>

/*
 * This translation unit is added directly by the Laurel board makefile and
 * the generated CRT0 trampoline expects c1_main() to exist whenever the
 * extra-core startup path is selected.
 */

#define SIO_BASE        0xD0000000UL
#define SIO_FIFO_ST     (*(volatile uint32_t *)(SIO_BASE + 0x050))
#define SIO_FIFO_WR     (*(volatile uint32_t *)(SIO_BASE + 0x054))
#define SIO_FIFO_RD     (*(volatile uint32_t *)(SIO_BASE + 0x058))

#define FIFO_ST_VLD     (1u << 0)
#define FIFO_ST_RDY     (1u << 1)
#define FIFO_ST_ROE     (1u << 3)
#define FIFO_ST_WOF     (1u << 2)

/*
 * Core-Private NVIC registers (PPB address space, banked per-core on RP2350).
 * Accessing these from Core 1 clears Core 1's own NVIC state, not Core 0's.
 * RP2350 has 52 external IRQs, covered by two 32-bit words (ICER0/ICER1).
 */
#define NVIC_ICER0  (*(volatile uint32_t *)0xE000E180U)
#define NVIC_ICER1  (*(volatile uint32_t *)0xE000E184U)
#define NVIC_ICPR0  (*(volatile uint32_t *)0xE000E280U)
#define NVIC_ICPR1  (*(volatile uint32_t *)0xE000E284U)

/*
 * SCB registers used in __c1_cpu_init() to configure Core 1's exception handling.
 * All PPB addresses are per-core on Cortex-M33 / RP2350.
 */
#define SCB_VTOR    (*(volatile uint32_t *)0xE000ED08U)  /* Vector Table Offset Register       */
#define SCB_SHCSR   (*(volatile uint32_t *)0xE000ED24U)  /* System Handler Control and State   */
/* SHCSR enable bits for configurable-priority system exceptions */
#define SHCSR_MEMFAULTENA  (1u << 16)  /* enable MemManage fault handler     */
#define SHCSR_BUSFAULTENA  (1u << 17)  /* enable BusFault handler            */
#define SHCSR_USGFAULTENA  (1u << 18)  /* enable UsageFault handler          */

/*
 * TIMER0 TIMERAWL — free-running 1 MHz counter, safe for multi-core reads.
 * Unlike TIMELR, reading TIMERAWL does NOT latch TIMERAWH so it is safe to
 * read from Core1 concurrently with Core0 without coordination.
 * Used to measure Core1 busy time in microseconds for CPU% reporting.
 */
#define TIMER0_TIMERAWL  (*(volatile uint32_t *)0x400B0028U)

/* Diagnostic: captures Core 1's VTOR at init time — readable via OpenOCD/GDB. */
volatile uint32_t c1_vtor_at_boot = 0U;

/*
 * Accumulated microseconds Core1 has spent executing dispatched functions.
 * Stamped on entry/exit of every fn() call using TIMER0_TIMERAWL.
 * Core0 reads this every 10 s to compute Core1 CPU utilisation.
 */
volatile uint32_t c1_busy_us __attribute__((used, externally_visible)) = 0U;

volatile uint32_t c1_boot_stage = 0xDEAD0000U;
/*
 * Side-channel async attitude dispatch.
 *
 * These two variables implement a zero-mutex, zero-FIFO-protocol mechanism
 * for the attitude controller's async Core1 dispatch.  Unlike the main SIO
 * FIFO path (used by EKF covariance and PID rate control), the side-channel
 * does NOT produce a FIFO done token.  Instead Core1 signals completion via
 * the c1_att_sidechan_done volatile flag, which Core0 polls in c1_att_barrier.
 *
 * Protocol (Core0 → Core1):
 *   Core0 writes a non-zero function pointer to c1_att_fn_sidechan + calls SEV.
 *   Core1 detects the non-zero value in its WFE idle loop, clears the pointer,
 *   calls the function, then sets c1_att_sidechan_done = 1 and calls SEV.
 *   Core0 c1_att_barrier() spins on c1_att_sidechan_done (2 ms timeout).
 *
 * This avoids sharing the SIO FIFO done-token stream with the sync dispatchers
 * (c1_run_sync / c1_run_sync_locked / c1_try_run_sync), eliminating the
 * protocol-corruption deadlock that occurs when mutex ownership spans cycles.
 */
volatile uint32_t c1_att_fn_sidechan  __attribute__((used, externally_visible)) = 0U;
volatile uint8_t  c1_att_sidechan_done __attribute__((used, externally_visible)) = 0U;
void __c1_cpu_init(void)
{
    /*
     * Scrub Core 1's NVIC before anything else runs.
     *
     * The RP2350 ROM's Core 1 launch protocol (6-step FIFO handshake) leaves
     * SPARE_IRQ_1 (IRQ 47, VectorFC) enabled in Core 1's banked NVIC registers.
     * If left enabled, the first WFE wakeup or any peripheral event causes
     * VectorFC to fire → weak _unhandled_exception → NVIC_SystemReset() crash.
     *
     * Core 1 runs bare-metal (WFE/SEV + SIO FIFO polling) and needs no NVIC
     * IRQs whatsoever, so disable and clear all 52 pending/enabled IRQs.
     */
    NVIC_ICER0 = 0xFFFFFFFFU;  /* disable IRQs  0..31 on Core 1 */
    NVIC_ICER1 = 0xFFFFFFFFU;  /* disable IRQs 32..51 on Core 1 (covers SPARE_IRQ_1=47) */
    NVIC_ICPR0 = 0xFFFFFFFFU;  /* clear pending IRQs  0..31 on Core 1 */
    NVIC_ICPR1 = 0xFFFFFFFFU;  /* clear pending IRQs 32..51 on Core 1 */
    /* DSB+ISB ensure the writes complete before any instruction that could
     * re-enable IRQs or take an exception sees the updated NVIC state. */
    __asm volatile ("dsb sy\n isb" ::: "memory");

    /*
     * Re-assert VTOR explicitly on Core 1.
     *
     * The ChibiOS CRT1 startup (crt0_v8m-ml.S extra-core path) already sets
     * VTOR = &_vectors when CRT0_VTOR_INIT=TRUE.  We repeat it here
     * belt-and-suspenders style to guard against any ROM or bootloader
     * artefact that may have left VTOR at an invalid address on Core 1.
     *
     * Without a valid VTOR, any fault causes a vector-table bus error
     * (HFSR.VECTTBL = 1) → immediate LOCKUP (double fault) with no
     * handler running.  Explicitly setting VTOR here breaks that cycle.
     */
    extern uint32_t _vectors[];
    SCB_VTOR = (uint32_t)_vectors;
    c1_vtor_at_boot = (uint32_t)_vectors;  /* readable via OpenOCD for diagnosis */

    /*
     * Enable configurable-priority fault handlers on Core 1.
     *
     * By default SHCSR.MEMFAULTENA/BUSFAULTENA/USGFAULTENA are 0, which
     * means any MemManage, BusFault, or UsageFault fault on Core 1 silently
     * escalates to HardFault (HFSR.FORCED=1).  The HardFault handler on
     * Core 1 can then itself fault (e.g. VECTTBL), causing a double-fault
     * lockup that is opaque to the debugger.
     *
     * With these bits set, each fault type invokes its own handler directly.
     * The handlers (at VTOR[4..6]) loop safely via save_fault_watchdog,
     * and OpenOCD can halt Core 1 to read CFSR/MMFAR/BFAR for diagnosis.
     */
    SCB_SHCSR |= SHCSR_MEMFAULTENA | SHCSR_BUSFAULTENA | SHCSR_USGFAULTENA;
    __asm volatile ("dsb sy\n isb" ::: "memory");

    c1_boot_stage = 1U;
}

void __c1_late_init(void)
{
    c1_boot_stage = 2U;
}

void c1_main(void)
{
    c1_boot_stage = 3U;

    /*
     * Re-enable maskable interrupts on Core 1.
     *
     * The ChibiOS CRT1 startup sets PRIMASK=1 (cpsid i) at entry to give
     * the startup code a stable environment.  With PRIMASK=1, configurable
     * priority faults (BusFault, MemManage, UsageFault) cannot be delivered
     * directly — they escalate to HardFault (HFSR.FORCED=1).  If the
     * HardFault handler then itself encounters a fault (e.g. bad VTOR →
     * HFSR.VECTTBL), the core goes to lockup with no recoverable state.
     *
     * We have already disabled all 52 NVIC IRQs via NVIC_ICER, so clearing
     * PRIMASK here is safe: no peripheral interrupt can fire.  But clearing
     * PRIMASK allows MemManage/BusFault/UsageFault to fire directly into
     * their respective handlers (save_fault_watchdog + infinite loop) rather
     * than escalating.  This survives a single fault without double-faulting.
     */
    __asm volatile ("cpsie i" ::: "memory");

    SIO_FIFO_ST = FIFO_ST_ROE | FIFO_ST_WOF;
    while (SIO_FIFO_ST & FIFO_ST_VLD) {
        (void)SIO_FIFO_RD;
    }

    while (1) {
        c1_boot_stage = 0x4DU;

        // wait for a function pointer from core0, execute it, then signal completion by writing 1 back to core0.  If we receive 0 instead of a valid pointer, it's a ping from core0 — write 0 back and wait for the next message.
        while (!(SIO_FIFO_ST & FIFO_ST_VLD)) {
            /*
             * Side-channel: check for an async attitude job queued by Core0.
             * Core0 sets c1_att_fn_sidechan (non-zero) and calls SEV to wake us.
             * We consume the pointer (clear before calling to prevent re-entry),
             * run it, then signal done via c1_att_sidechan_done flag (NOT via the
             * SIO FIFO, to avoid corrupting the sync-dispatch done-token stream).
             */
            uint32_t att_fn_raw = c1_att_fn_sidechan;
            if (att_fn_raw != 0U) {
                c1_att_fn_sidechan = 0U;   /* consume before calling (prevents re-entry) */
                __asm volatile ("dsb sy\n isb" ::: "memory");
                typedef void (*c1_task_fn)(void);
                const uint32_t t0 = TIMER0_TIMERAWL;
                ((c1_task_fn)att_fn_raw)();
                c1_busy_us += TIMER0_TIMERAWL - t0;
                __asm volatile ("dmb sy" ::: "memory");
                c1_att_sidechan_done = 1U; /* signal completion to Core0 */
                __asm volatile ("sev");    /* wake Core0 if it is WFEing in c1_att_barrier */
            }
            __asm volatile ("wfe"); // wait for event — low-power sleep until core0 writes to the FIFO and signals with SEV
            // small delay.
            for (volatile int i = 0; i < 100; i++) {
                __asm volatile ("nop");
            }
        }

        uint32_t msg = SIO_FIFO_RD;

        // if msg is 0, it's a ping from core0.  Write 0 back and wait for the next message.
        if (msg == 0U) {
            while (!(SIO_FIFO_ST & FIFO_ST_RDY)) { }
            SIO_FIFO_WR = 0U;
            // small delay
            for (volatile int i = 0; i < 100; i++) {
                __asm volatile ("nop");
            }
            continue;
        }

        c1_boot_stage = 0x50U;

        // treat the message as a function pointer and call it.  The function is responsible for doing its own DMB if it needs to ensure memory visibility of its actions to core0 before signaling completion.
        typedef void (*c1_task_fn)(void);
        const uint32_t _c1_t0 = TIMER0_TIMERAWL;
        ((c1_task_fn)msg)();
        c1_busy_us += TIMER0_TIMERAWL - _c1_t0;  /* accumulate Core1 busy time in µs */

        c1_boot_stage = 0x51U;

        // notify core0 that task is complete
        __asm volatile ("dmb sy" ::: "memory");

        while (!(SIO_FIFO_ST & FIFO_ST_RDY)) { }
        SIO_FIFO_WR = 1U;

        // 'sev' is a ARM instruction that signals an event to all cores. It sets the internal "event register" on every core in the system.
        // It's the counterpart to WFE (Wait For Event), which puts a core to sleep until its event register is set 
        __asm volatile ("sev");
    }
}