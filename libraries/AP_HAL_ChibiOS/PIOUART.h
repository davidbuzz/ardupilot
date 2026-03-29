/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * RP2350 PIO-based pseudo-UART driver for ArduPilot/ChibiOS.
 *
 * Implements up to 4 additional serial ports via RP2350 PIO state machines:
 *   PIOUART0: PIO0 SM0 (TX) + SM1 (RX)  — RX IRQ via PIO0 IRQ0
 *   PIOUART1: PIO0 SM2 (TX) + SM3 (RX)  — RX IRQ via PIO0 IRQ1
 *   PIOUART2: PIO1 SM0 (TX) + SM1 (RX)  — RX IRQ via PIO1 IRQ0
 *   PIOUART3: PIO1 SM2 (TX) + SM3 (RX)  — RX IRQ via PIO1 IRQ1
 * 
 * TX: PIO side-set. RX: PIO IN + autopush at 8 bits → ISR ring buffer.
 * Baud clock: sys_clk / (8 cycles_per_bit * baud_rate).
 *
 * Instruction words derived from pico-sdk uart_tx.pio / uart_rx.pio.
 * Source in hwdef/Pico2/pico_pio_uart.pio.
 *
 * WIP: skeleton — verify on real RP2350 hardware.
 */
#pragma once

#include "AP_HAL_ChibiOS.h"

#if defined(HAL_HAVE_PIO_UARTS) && HAL_HAVE_PIO_UARTS > 0

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/RingBuffer.h>
#include <hal.h>

// ---------------------------------------------------------------------------
// PIO UART protocol constants
// ---------------------------------------------------------------------------

#define PIO_UART_CYCLES_PER_BIT  8U

// Instruction memory layout (12 of 32 words used per PIO):
//   [0..4]  TX program
//   [5..13] RX program
#define PIO_UART_TX_PROG_OFFSET  0U
#define PIO_UART_TX_PROG_LEN     5U
#define PIO_UART_RX_PROG_OFFSET  5U
#define PIO_UART_RX_PROG_LEN     9U

// ---------------------------------------------------------------------------
// Pre-assembled PIO UART programs
// ---------------------------------------------------------------------------

// TX: side_set 1 opt, 8 cycles/bit, loaded at offset 0
//   0xE081: set    pindirs, 1      ; one-time pin direction init
//   0x9FA0: pull   block            side 1 [7]
//   0xF727: set    x, 7             side 0 [7]
//   0x6001: out    pins, 1
//   0x0643: jmp    x--, 3           [6]
static const uint16_t k_pio_uart_tx_pgm[PIO_UART_TX_PROG_LEN] = {
    0xE081u, 0x9FA0u, 0xF727u, 0x6001u, 0x0643u,
};

// RX: 8 cycles/bit, pre-relocated for offset 5
//   0x2020: wait  0 pin, 0
//   0xE227: set   x, 7             [2]
//   0x4001: in    pins, 1
//   0x0647: jmp   x--, 7           [6]   (target=7 = offset+2)
//   0x00CD: jmp   pin, 13                (target=13 = offset+8)
//   0xC014: irq   4 rel
//   0x20A0: wait  1 pin, 0
//   0x0005: jmp   5                      (target=5 = offset+0)
//   0x8020: push  noblock
static const uint16_t k_pio_uart_rx_pgm[PIO_UART_RX_PROG_LEN] = {
    0x2020u, 0xE227u, 0x4001u, 0x0647u, 0x00CDu,
    0xC014u, 0x20A0u, 0x0005u, 0x8020u,
};

// ---------------------------------------------------------------------------
// PIO register bit-field constants
// ---------------------------------------------------------------------------

#define PIO_CTRL_SM_ENABLE_LSB        0u
#define PIO_CTRL_SM_RESTART_LSB       8u
#define PIO_CTRL_CLKDIV_RESTART_LSB  12u

// RP2350 PIO FSTAT layout (datasheet Table 983):
// RXFULL[3:0], RXEMPTY[11:8], TXFULL[19:16], TXEMPTY[27:24].
#define PIO_FSTAT_RXFULL_LSB          0u
#define PIO_FSTAT_RXEMPTY_LSB         8u
#define PIO_FSTAT_TXFULL_LSB         16u
#define PIO_FSTAT_TXEMPTY_LSB        24u

#define PIO_CLKDIV_FRAC_LSB           8u
#define PIO_CLKDIV_INT_LSB           16u

#define PIO_EXECCTRL_WRAP_BOT_LSB     7u
#define PIO_EXECCTRL_WRAP_TOP_LSB    12u
#define PIO_EXECCTRL_JMP_PIN_LSB     24u

#define PIO_SHIFTCTRL_AUTOPUSH       (1u << 16)
#define PIO_SHIFTCTRL_AUTOPULL       (1u << 17)
#define PIO_SHIFTCTRL_IN_SHIFTDIR    (1u << 18)
#define PIO_SHIFTCTRL_OUT_SHIFTDIR   (1u << 19)
#define PIO_SHIFTCTRL_PUSH_THRESH_LSB 26u
#define PIO_SHIFTCTRL_PULL_THRESH_LSB 20u

#define PIO_PINCTRL_OUT_BASE_LSB      0u
#define PIO_PINCTRL_SET_BASE_LSB      5u
#define PIO_PINCTRL_SIDESET_BASE_LSB 10u
#define PIO_PINCTRL_IN_BASE_LSB      15u
#define PIO_PINCTRL_OUT_COUNT_LSB    20u
#define PIO_PINCTRL_SET_COUNT_LSB    26u
#define PIO_PINCTRL_SIDESET_COUNT_LSB 29u

// RX FIFO not-empty interrupt bit for SM sm (in IRQx_INTE/INTS on RP2350).
// RP2350 maps SM0..SM3 RXNEMPTY to bits 0..3.
#define PIO_INTE_RX_NOTEMPTY(sm)  (1u << (sm))

#ifdef HAL_HAVE_PIO_UARTS
#define PIO_NUM_INSTANCES  HAL_HAVE_PIO_UARTS
#else
#define PIO_NUM_INSTANCES  4U
#endif

// ---------------------------------------------------------------------------
// PIORXDriver class — inherits AP_HAL::UARTDriver
// ---------------------------------------------------------------------------

namespace ChibiOS {

class PIORXDriver final : public AP_HAL::UARTDriver {
public:
    explicit PIORXDriver(uint8_t instance);
    CLASS_NO_COPY(PIORXDriver);

    // ---- Public virtual overrides required by AP_HAL::UARTDriver ----
    bool is_initialized() override { return _initialized; }
    bool tx_pending() override;
    uint32_t txspace() override;
    bool wait_timeout(uint16_t n, uint32_t timeout_ms) override;

    // ---- ISR dispatch handlers ----
    static void _irq_pio0_0();
    static void _irq_pio0_1();
    static void _irq_pio1_0();
    static void _irq_pio1_1();

    // ---- ISR worker ----
    void _service_rx_fifo();

protected:
    // ---- Protected pure-virtual overrides from AP_HAL::UARTDriver ----
    void _begin(uint32_t baud, uint16_t rxSpace, uint16_t txSpace) override;
    size_t _write(const uint8_t *buffer, size_t size) override;
    ssize_t _read(uint8_t *buffer, uint16_t count) override WARN_IF_UNUSED;
    void _end() override;
    void _flush() override;
    uint32_t _available() override;
    bool _discard_input() override;

private:
    struct InstanceConfig {
        PIO_TypeDef  *pio;
        uint8_t       sm_tx;
        uint8_t       sm_rx;
        uint8_t       tx_pin;
        uint8_t       rx_pin;
        uint8_t       irq_num;
    };

    static const InstanceConfig _cfg_table[PIO_NUM_INSTANCES];
    static PIORXDriver          *_instances[PIO_NUM_INSTANCES];
    static bool                  _pgm_loaded[2];   // [0]=PIO0, [1]=PIO1

    const uint8_t _instance;
    bool  _initialized;
    uint32_t _active_baud;
    ByteBuffer *_readbuf;
    ByteBuffer *_writebuf;

    const InstanceConfig &cfg() const { return _cfg_table[_instance]; }

    void _upload_programs();
    void _start_tx_sm(uint32_t clkdiv_int, uint32_t clkdiv_frac);
    void _start_rx_sm(uint32_t clkdiv_int, uint32_t clkdiv_frac);
    void _configure_gpio(uint8_t pin, bool is_output);
    void _enable_rx_irq();
    void _drain_tx_fifo();

    static void _calc_clkdiv(uint32_t baud, uint32_t &int_div, uint32_t &frac_div);
};

} // namespace ChibiOS

#endif // HAL_HAVE_PIO_UARTS
