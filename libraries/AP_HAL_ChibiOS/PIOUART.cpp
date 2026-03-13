/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * RP2350 PIO UART driver — 8N1, no-DMA, IRQ-driven RX.
 * WIP: skeleton — verify on real RP2350 hardware.
 */

#include "PIOUART.h"

#if defined(HAL_HAVE_PIO_UARTS)

#include <AP_HAL/AP_HAL.h>
#include <hal.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS

using namespace ChibiOS;

extern const AP_HAL::HAL &hal;

// System clock comes from ChibiOS RP2350 rp_clocks.h derived constant.
#ifndef RP_SYS_CLK_HZ
#define RP_SYS_CLK_HZ  RP_PLL_SYS_CLK
#endif

// GPIO funcsel for PIO0=6, PIO1=7
#define RP_GPIO_FUNCSEL_PIO0  6U
#define RP_GPIO_FUNCSEL_PIO1  7U

// Default RX ring-buffer size (TX goes direct to FIFO — no staging buffer needed)
static const uint16_t PIO_UART_RX_BUF = 512;

// NVIC priority for PIO UART RX IRQ
#define PIO_UART_IRQ_PRIO  5

// ---------------------------------------------------------------------------
// Static members
// ---------------------------------------------------------------------------

PIORXDriver *PIORXDriver::_instances[PIO_NUM_INSTANCES];
bool         PIORXDriver::_pgm_loaded[2];

// InstanceConfig: pin numbers from hwdef.h PIORXDRIVERn_TX/RX_PIN defines
const PIORXDriver::InstanceConfig PIORXDriver::_cfg_table[PIO_NUM_INSTANCES] = {
#if PIO_NUM_INSTANCES >= 1
    { PIO0, 0, 1, PIOUART0_TX_PIN, PIOUART0_RX_PIN, RP_PIO0_IRQ_0_NUMBER },
#endif
#if PIO_NUM_INSTANCES >= 2
    { PIO0, 2, 3, PIOUART1_TX_PIN, PIOUART1_RX_PIN, RP_PIO0_IRQ_1_NUMBER },
#endif
#if PIO_NUM_INSTANCES >= 3
    { PIO1, 0, 1, PIOUART2_TX_PIN, PIOUART2_RX_PIN, RP_PIO1_IRQ_0_NUMBER },
#endif
#if PIO_NUM_INSTANCES >= 4
    { PIO1, 2, 3, PIOUART3_TX_PIN, PIOUART3_RX_PIN, RP_PIO1_IRQ_1_NUMBER },
#endif
};

// ---------------------------------------------------------------------------
// ChibiOS IRQ handlers (C linkage, vector table entries)
// ---------------------------------------------------------------------------

extern "C" {

CH_IRQ_HANDLER(RP_PIO0_IRQ_0_HANDLER)
{
    CH_IRQ_PROLOGUE();
    PIORXDriver::_irq_pio0_0();
    CH_IRQ_EPILOGUE();
}

CH_IRQ_HANDLER(RP_PIO0_IRQ_1_HANDLER)
{
    CH_IRQ_PROLOGUE();
    PIORXDriver::_irq_pio0_1();
    CH_IRQ_EPILOGUE();
}

CH_IRQ_HANDLER(RP_PIO1_IRQ_0_HANDLER)
{
    CH_IRQ_PROLOGUE();
    PIORXDriver::_irq_pio1_0();
    CH_IRQ_EPILOGUE();
}

CH_IRQ_HANDLER(RP_PIO1_IRQ_1_HANDLER)
{
    CH_IRQ_PROLOGUE();
    PIORXDriver::_irq_pio1_1();
    CH_IRQ_EPILOGUE();
}

} // extern "C"

void PIORXDriver::_irq_pio0_0() { if (_instances[0]) { _instances[0]->_service_rx_fifo(); } }
void PIORXDriver::_irq_pio0_1() { if (_instances[1]) { _instances[1]->_service_rx_fifo(); } }
void PIORXDriver::_irq_pio1_0() { if (_instances[2]) { _instances[2]->_service_rx_fifo(); } }
void PIORXDriver::_irq_pio1_1() { if (_instances[3]) { _instances[3]->_service_rx_fifo(); } }

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

PIORXDriver::PIORXDriver(uint8_t instance)
    : _instance(instance)
    , _initialized(false)
    , _readbuf(nullptr)
{
    if (instance < PIO_NUM_INSTANCES) {
        _instances[instance] = this;
    }
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

void PIORXDriver::_calc_clkdiv(uint32_t baud, uint32_t &int_div, uint32_t &frac_div)
{
    const uint32_t sys_clk  = RP_SYS_CLK_HZ;
    const uint32_t divisor  = PIO_UART_CYCLES_PER_BIT * baud;
    int_div  = sys_clk / divisor;
    frac_div = ((sys_clk % divisor) * 256U + divisor / 2U) / divisor;
    if (int_div  < 1U)   { int_div  = 1U; }
    if (int_div  > 65535U){ int_div  = 65535U; }
    if (frac_div > 255U) { frac_div = 255U; }
}

void PIORXDriver::_configure_gpio(uint8_t pin, bool is_output)
{
    const uint32_t funcsel = (cfg().pio == PIO0) ? RP_GPIO_FUNCSEL_PIO0
                                                 : RP_GPIO_FUNCSEL_PIO1;
    iomode_t mode;
    if (is_output) {
        mode = PAL_RP_IOCTRL_FUNCSEL(funcsel)
             | PAL_RP_PAD_DRIVE4
             | PAL_RP_PAD_IE
             | PAL_RP_PAD_SCHMITT;
    } else {
        mode = PAL_RP_IOCTRL_FUNCSEL(funcsel)
             | PAL_RP_PAD_OD
             | PAL_RP_PAD_IE
             | PAL_RP_PAD_PUE
             | PAL_RP_PAD_SCHMITT;
    }
    palSetPadMode(IOPORT1, pin, mode);
}

void PIORXDriver::_upload_programs()
{
    const uint8_t pio_idx = (cfg().pio == PIO0) ? 0U : 1U;
    if (_pgm_loaded[pio_idx]) {
        return;
    }
    PIO_TypeDef *const pio = cfg().pio;

    pio->CTRL = 0U; // stop all SMs

    for (uint8_t i = 0; i < PIO_UART_TX_PROG_LEN; i++) {
        pio->INSTR_MEM[PIO_UART_TX_PROG_OFFSET + i] = k_pio_uart_tx_pgm[i];
    }
    for (uint8_t i = 0; i < PIO_UART_RX_PROG_LEN; i++) {
        pio->INSTR_MEM[PIO_UART_RX_PROG_OFFSET + i] = k_pio_uart_rx_pgm[i];
    }

    _pgm_loaded[pio_idx] = true;
}

void PIORXDriver::_start_tx_sm(uint32_t int_div, uint32_t frac_div)
{
    PIO_TypeDef *const pio   = cfg().pio;
    const uint8_t      sm    = cfg().sm_tx;
    const uint8_t      tx_pin = cfg().tx_pin;

    pio->CTRL &= ~(1u << (PIO_CTRL_SM_ENABLE_LSB + sm));

    pio->SM[sm].CLKDIV = (int_div  << PIO_CLKDIV_INT_LSB)
                       | (frac_div << PIO_CLKDIV_FRAC_LSB);

    pio->SM[sm].EXECCTRL =
          ((uint32_t)(PIO_UART_TX_PROG_OFFSET + PIO_UART_TX_PROG_LEN - 1)
                       << PIO_EXECCTRL_WRAP_TOP_LSB)
        | ((uint32_t)PIO_UART_TX_PROG_OFFSET << PIO_EXECCTRL_WRAP_BOT_LSB);

    pio->SM[sm].SHIFTCTRL = PIO_SHIFTCTRL_OUT_SHIFTDIR;

    pio->SM[sm].PINCTRL =
          (2u               << PIO_PINCTRL_SIDESET_COUNT_LSB)
        | ((uint32_t)tx_pin << PIO_PINCTRL_SIDESET_BASE_LSB)
        | ((uint32_t)tx_pin << PIO_PINCTRL_OUT_BASE_LSB)
        | (1u               << PIO_PINCTRL_OUT_COUNT_LSB);

    pio->CTRL |= (1u << (PIO_CTRL_CLKDIV_RESTART_LSB + sm))
              |  (1u << (PIO_CTRL_SM_RESTART_LSB      + sm));

    pio->SM[sm].INSTR = (uint32_t)k_pio_uart_tx_pgm[0];

    pio->CTRL |= (1u << (PIO_CTRL_SM_ENABLE_LSB + sm));
}

void PIORXDriver::_start_rx_sm(uint32_t int_div, uint32_t frac_div)
{
    PIO_TypeDef *const pio   = cfg().pio;
    const uint8_t      sm    = cfg().sm_rx;
    const uint8_t      rx_pin = cfg().rx_pin;

    pio->CTRL &= ~(1u << (PIO_CTRL_SM_ENABLE_LSB + sm));

    pio->SM[sm].CLKDIV = (int_div  << PIO_CLKDIV_INT_LSB)
                       | (frac_div << PIO_CLKDIV_FRAC_LSB);

    pio->SM[sm].EXECCTRL =
          ((uint32_t)(PIO_UART_RX_PROG_OFFSET + PIO_UART_RX_PROG_LEN - 1)
                       << PIO_EXECCTRL_WRAP_TOP_LSB)
        | ((uint32_t)PIO_UART_RX_PROG_OFFSET << PIO_EXECCTRL_WRAP_BOT_LSB)
        | ((uint32_t)rx_pin << PIO_EXECCTRL_JMP_PIN_LSB);

    pio->SM[sm].SHIFTCTRL = PIO_SHIFTCTRL_AUTOPUSH
                          | PIO_SHIFTCTRL_IN_SHIFTDIR
                          | (8u << PIO_SHIFTCTRL_PUSH_THRESH_LSB);

    pio->SM[sm].PINCTRL = ((uint32_t)rx_pin << PIO_PINCTRL_IN_BASE_LSB);

    pio->CTRL |= (1u << (PIO_CTRL_CLKDIV_RESTART_LSB + sm))
              |  (1u << (PIO_CTRL_SM_RESTART_LSB      + sm));

    pio->CTRL |= (1u << (PIO_CTRL_SM_ENABLE_LSB + sm));
}

void PIORXDriver::_enable_rx_irq()
{
    PIO_TypeDef *const pio   = cfg().pio;
    const uint8_t      sm_rx = cfg().sm_rx;

    if (sm_rx <= 1U) {
        pio->IRQ0_INTE |= PIO_INTE_RX_NOTEMPTY(sm_rx);
    } else {
        pio->IRQ1_INTE |= PIO_INTE_RX_NOTEMPTY(sm_rx);
    }

    nvicEnableVector(cfg().irq_num, PIO_UART_IRQ_PRIO);
}

// ---------------------------------------------------------------------------
// ISR: drain RX FIFO into ring buffer
// ---------------------------------------------------------------------------

void PIORXDriver::_service_rx_fifo()
{
    if (!_readbuf || !_initialized) {
        return;
    }
    PIO_TypeDef *const pio = cfg().pio;
    const uint8_t      sm  = cfg().sm_rx;

    while (!(pio->FSTAT & (1u << (PIO_FSTAT_RXEMPTY_LSB + sm)))) {
        // Data is in MSB (right-shift ISR, 8-bit autopush)
        const uint8_t byte = (uint8_t)(pio->RXF[sm] >> 24U);
        _readbuf->write(&byte, 1);
    }
}

// ---------------------------------------------------------------------------
// AP_HAL::UARTDriver protected virtual overrides
// ---------------------------------------------------------------------------

void PIORXDriver::_begin(uint32_t b, uint16_t rxSpace, uint16_t txSpace)
{
    if (_instance >= PIO_NUM_INSTANCES) {
        return;
    }
    if (b == 0) {
        b = 57600;
    }
    if (rxSpace == 0) {
        rxSpace = PIO_UART_RX_BUF;
    }

    if (_readbuf == nullptr) {
        _readbuf = new ByteBuffer(rxSpace);
        if (!_readbuf) {
            return;
        }
    }

    _upload_programs();
    _configure_gpio(cfg().tx_pin, true);
    _configure_gpio(cfg().rx_pin, false);

    uint32_t int_div, frac_div;
    _calc_clkdiv(b, int_div, frac_div);

    _start_tx_sm(int_div, frac_div);
    _start_rx_sm(int_div, frac_div);
    _enable_rx_irq();

    _initialized = true;
}

void PIORXDriver::_end()
{
    if (!_initialized) {
        return;
    }
    PIO_TypeDef *const pio   = cfg().pio;
    const uint8_t      sm_tx = cfg().sm_tx;
    const uint8_t      sm_rx = cfg().sm_rx;

    pio->CTRL &= ~((1u << (PIO_CTRL_SM_ENABLE_LSB + sm_tx))
                 | (1u << (PIO_CTRL_SM_ENABLE_LSB + sm_rx)));

    nvicDisableVector(cfg().irq_num);
    if (sm_rx <= 1U) {
        pio->IRQ0_INTE &= ~PIO_INTE_RX_NOTEMPTY(sm_rx);
    } else {
        pio->IRQ1_INTE &= ~PIO_INTE_RX_NOTEMPTY(sm_rx);
    }

    _initialized = false;
}

void PIORXDriver::_flush()
{
    // TX goes direct to PIO FIFO — nothing to flush in software
}

uint32_t PIORXDriver::_available()
{
    if (!_initialized || !_readbuf) {
        return 0;
    }
    return _readbuf->available();
}

bool PIORXDriver::_discard_input()
{
    if (!_initialized || !_readbuf) {
        return false;
    }
    _readbuf->clear();
    return true;
}

ssize_t PIORXDriver::_read(uint8_t *buffer, uint16_t count)
{
    if (!_initialized || !_readbuf || !buffer) {
        return -1;
    }
    return (ssize_t)_readbuf->read(buffer, count);
}

size_t PIORXDriver::_write(const uint8_t *buffer, size_t size)
{
    if (!_initialized || !buffer || size == 0) {
        return 0;
    }
    PIO_TypeDef *const pio = cfg().pio;
    const uint8_t      sm  = cfg().sm_tx;

    size_t written = 0;
    for (size_t i = 0; i < size; i++) {
        if (!(pio->FSTAT & (1u << (PIO_FSTAT_TXFULL_LSB + sm)))) {
            pio->TXF[sm] = (uint32_t)buffer[i];
            written++;
        }
        // Drop byte if TX FIFO full (non-blocking)
    }
    return written;
}

// ---------------------------------------------------------------------------
// AP_HAL::UARTDriver public virtual overrides
// ---------------------------------------------------------------------------

uint32_t PIORXDriver::txspace()
{
    if (!_initialized) {
        return 0;
    }
    // PIO TX FIFO has 4 words; count free slots in the TX FIFO
    PIO_TypeDef *const pio = cfg().pio;
    const uint8_t      sm  = cfg().sm_tx;
    // FLEVEL[sm*2 .. sm*2+3] gives TX FIFO level (0-4); space = 4 - level
    // Simpler: check TXFULL bit per slot (crude but works for skeleton)
    return (pio->FSTAT & (1u << (PIO_FSTAT_TXFULL_LSB + sm))) ? 0U : 1U;
}

bool PIORXDriver::tx_pending()
{
    if (!_initialized) {
        return false;
    }
    // TX FIFO not empty = pending
    PIO_TypeDef *const pio = cfg().pio;
    const uint8_t      sm  = cfg().sm_tx;
    return !(pio->FSTAT & (1u << (PIO_FSTAT_TXEMPTY_LSB + sm)));
}

bool PIORXDriver::wait_timeout(uint16_t n, uint32_t timeout_ms)
{
    const uint32_t start = AP_HAL::millis();
    while (AP_HAL::millis() - start < timeout_ms) {
        if (_available() >= n) {
            return true;
        }
        hal.scheduler->delay_microseconds(100);
    }
    return _available() >= n;
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#endif // HAL_HAVE_PIO_UARTS
