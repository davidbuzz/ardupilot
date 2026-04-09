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

volatile uint32_t c1_boot_stage = 0xDEAD0000U;

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

    c1_boot_stage = 1U;
}

void __c1_late_init(void)
{
    c1_boot_stage = 2U;
}

void c1_main(void)
{
    c1_boot_stage = 3U;

    SIO_FIFO_ST = FIFO_ST_ROE | FIFO_ST_WOF;
    while (SIO_FIFO_ST & FIFO_ST_VLD) {
        (void)SIO_FIFO_RD;
    }

    while (1) {
        c1_boot_stage = 0x4DU;

        // wait for a function pointer from core0, execute it, then signal completion by writing 1 back to core0.  If we receive 0 instead of a valid pointer, it's a ping from core0 — write 0 back and wait for the next message.
        while (!(SIO_FIFO_ST & FIFO_ST_VLD)) {
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
        ((c1_task_fn)msg)();

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