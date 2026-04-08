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

volatile uint32_t c1_boot_stage = 0xDEAD0000U;

void __c1_cpu_init(void)
{
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

        while (!(SIO_FIFO_ST & FIFO_ST_VLD)) {
            __asm volatile ("wfe");
            // small delay.
            for (volatile int i = 0; i < 100; i++) {
                __asm volatile ("nop");
            }
        }

        uint32_t msg = SIO_FIFO_RD;

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