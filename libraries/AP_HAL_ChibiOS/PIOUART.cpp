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

#if defined(HAL_HAVE_PIO_UARTS) && HAL_HAVE_PIO_UARTS > 0

#include <AP_HAL/AP_HAL.h>
#include <hal.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS

using namespace ChibiOS;

extern const AP_HAL::HAL &hal;

// GPIO funcsel for PIO0=6, PIO1=7
#define RP_GPIO_FUNCSEL_PIO0  6U
#define RP_GPIO_FUNCSEL_PIO1  7U

// Default RX ring-buffer size (TX goes direct to FIFO — no staging buffer needed)
static const uint16_t PIO_UART_RX_BUF = 512;
static const uint16_t PIO_UART_TX_BUF = 512;

// TX FIFO depth per state machine (not joined)
static const uint8_t PIO_TX_FIFO_DEPTH = 4U;

// Extract TX fill level for SM sm from PIO FLEVEL register.
// FLEVEL layout: bits [sm*8+3:sm*8] = TX fill, bits [sm*8+7:sm*8+4] = RX fill
static inline uint32_t pio_tx_level(PIO_TypeDef *pio, uint8_t sm)
{
    return (pio->FLEVEL >> (sm * 8U)) & 0xFU;
}

// NVIC priority for PIO UART RX IRQ
#define PIO_UART_IRQ_PRIO  5

// ---------------------------------------------------------------------------
// Static members
// ---------------------------------------------------------------------------

PIORXDriver *PIORXDriver::_instances[PIO_NUM_INSTANCES];
bool         PIORXDriver::_pgm_loaded[2];

// Bring-up debug counters read via SWD when diagnosing silent TX/RX paths.
volatile uint32_t pio_uart_dbg_begin_count[PIO_NUM_INSTANCES];
volatile uint32_t pio_uart_dbg_ctor_count[PIO_NUM_INSTANCES];
volatile uint32_t pio_uart_dbg_write_calls[PIO_NUM_INSTANCES];
volatile uint32_t pio_uart_dbg_write_bytes[PIO_NUM_INSTANCES];
volatile uint32_t pio_uart_dbg_rx_service_calls[PIO_NUM_INSTANCES];
volatile uint32_t pio_uart_dbg_rx_bytes[PIO_NUM_INSTANCES];

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

// Forward declarations suppress -Wmissing-declarations on the IRQ handlers below.
// CH_IRQ_HANDLER(x) expands to 'extern "C" void x(void)'; the trailing
// semicolons produce declarations rather than definitions.
extern "C" {
CH_IRQ_HANDLER(RP_PIO0_IRQ_0_HANDLER);
CH_IRQ_HANDLER(RP_PIO0_IRQ_1_HANDLER);
CH_IRQ_HANDLER(RP_PIO1_IRQ_0_HANDLER);
CH_IRQ_HANDLER(RP_PIO1_IRQ_1_HANDLER);
} // extern "C" (declarations)

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

void PIORXDriver::_irq_pio0_0()
{
#if PIO_NUM_INSTANCES > 0
    if (_instances[0]) {
        _instances[0]->_service_rx_fifo();
    }
#endif
}

void PIORXDriver::_irq_pio0_1()
{
#if PIO_NUM_INSTANCES > 1
    if (_instances[1]) {
        _instances[1]->_service_rx_fifo();
    }
#endif
}

void PIORXDriver::_irq_pio1_0()
{
#if PIO_NUM_INSTANCES > 2
    if (_instances[2]) {
        _instances[2]->_service_rx_fifo();
    }
#endif
}

void PIORXDriver::_irq_pio1_1()
{
#if PIO_NUM_INSTANCES > 3
    if (_instances[3]) {
        _instances[3]->_service_rx_fifo();
    }
#endif
}

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

PIORXDriver::PIORXDriver(uint8_t instance)
    : _instance(instance)
    , _initialized(false)
    , _active_baud(0)
    , _readbuf(nullptr)
    , _writebuf(nullptr)
{
    if (instance < PIO_NUM_INSTANCES) {
        _instances[instance] = this;
        pio_uart_dbg_ctor_count[instance]++;
    }
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

void PIORXDriver::_calc_clkdiv(uint32_t baud, uint32_t &int_div, uint32_t &frac_div)
{
    // Use the runtime-configured clk_sys value so PIO UART baud generation
    // stays aligned with the actual RP2350 clock tree, even if the board
    // overrides PLL/post-divider settings from the rp_clocks.h defaults.
    uint32_t sys_clk = rp_clock_get_hz(RP_CLK_SYS);
    if (sys_clk == 0U) {
        sys_clk = RP_CLK_SYS_FREQ;
    }
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
        // RX pin should be a plain peripheral input with pull-up.
        // Open-drain mode here can distort idle/high levels and produce
        // framing noise on loopback or externally-driven UART lines.
        mode = PAL_RP_IOCTRL_FUNCSEL(funcsel)
             | PAL_RP_PAD_IE
             | PAL_RP_PAD_PUE
             | PAL_RP_PAD_SCHMITT;
    }
    palSetPadMode(IOPORT1, pin, mode);

    // Keep GPIO direction sane even when routed to PIO function.
    // On RP2350 bring-up, explicitly setting SIO OE avoids silent TX pins
    // staying as inputs when PIO pin-direction state is not latched yet.
    if (pin < 32U) {
        if (is_output) {
            SIO->GPIO_OE_SET = (1u << pin);
        } else {
            SIO->GPIO_OE_CLR = (1u << pin);
        }
    } else {
        const uint32_t bit = 1u << (pin - 32U);
        if (is_output) {
            SIO->GPIO_HI_OE_SET = bit;
        } else {
            SIO->GPIO_HI_OE_CLR = bit;
        }
    }
}

void PIORXDriver::_upload_programs()
{
    const uint8_t pio_idx = (cfg().pio == PIO0) ? 0U : 1U;
    if (_pgm_loaded[pio_idx]) {
        return;
    }
    PIO_TypeDef *const pio = cfg().pio;

    // RP2350 keeps many peripherals asserted in reset until explicitly
    // released. Ensure the selected PIO block is live before touching any
    // PIO registers, otherwise register writes are ignored.
    hal_lld_peripheral_unreset((pio_idx == 0U) ? RESETS_ALLREG_PIO0
                                                : RESETS_ALLREG_PIO1);

    pio->CTRL = 0U; // stop all SMs
    // Start from a known IRQ mask state; RX polling is used during bring-up.
    pio->IRQ0_INTE = 0U;
    pio->IRQ1_INTE = 0U;

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
        | ((uint32_t)(PIO_UART_TX_PROG_OFFSET + 1U) << PIO_EXECCTRL_WRAP_BOT_LSB)
        | (1u << 30); // SIDE_EN: enable optional sideset bit used by uart_tx

    pio->SM[sm].SHIFTCTRL = PIO_SHIFTCTRL_OUT_SHIFTDIR;

        pio->SM[sm].PINCTRL =
            // SIDE_EN consumes one bit in Delay/Side-set, so one actual
            // side-set data bit requires SIDESET_COUNT=2 (enable+data).
            (2u               << PIO_PINCTRL_SIDESET_COUNT_LSB)
                | ((uint32_t)tx_pin << PIO_PINCTRL_SIDESET_BASE_LSB)
                | ((uint32_t)tx_pin << PIO_PINCTRL_OUT_BASE_LSB)
                | (1u               << PIO_PINCTRL_OUT_COUNT_LSB)
                | ((uint32_t)tx_pin << PIO_PINCTRL_SET_BASE_LSB)
                | (1u               << PIO_PINCTRL_SET_COUNT_LSB);

    pio->CTRL |= (1u << (PIO_CTRL_CLKDIV_RESTART_LSB + sm))
              |  (1u << (PIO_CTRL_SM_RESTART_LSB      + sm));

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

    // RX program uses explicit 'push noblock' after stop-bit validation,
    // so AUTOPUSH must remain disabled.
    pio->SM[sm].SHIFTCTRL = PIO_SHIFTCTRL_IN_SHIFTDIR;

    pio->SM[sm].PINCTRL = ((uint32_t)rx_pin << PIO_PINCTRL_IN_BASE_LSB);

    pio->CTRL |= (1u << (PIO_CTRL_CLKDIV_RESTART_LSB + sm))
              |  (1u << (PIO_CTRL_SM_RESTART_LSB      + sm));

    // Important: force the RX SM PC to the RX program base.
    // The TX program is loaded at IMEM[0..3] and RX at IMEM[4..11].
    // After restart, SM PC may be 0; if so, RX can execute the TX entry
    // instruction (pull block) and stall forever with an empty TX FIFO.
    // Execute an unconditional JMP to PIO_UART_RX_PROG_OFFSET so RX always
    // starts in the correct program.
    const uint32_t jmp_rx_prog = (PIO_UART_RX_PROG_OFFSET & 0x1FU);
    pio->SM[sm].INSTR = jmp_rx_prog;

    pio->CTRL |= (1u << (PIO_CTRL_SM_ENABLE_LSB + sm));
}

void PIORXDriver::_enable_rx_irq()
{
    PIO_TypeDef *const pio   = cfg().pio;
    const uint8_t      sm_rx = cfg().sm_rx;
    const uint32_t rx_mask = PIO_INTE_RX_NOTEMPTY(sm_rx);

    // Drop any stale pending data before enabling IRQ-driven RX. During
    // bring-up, random FIFO residue can otherwise cause an immediate IRQ
    // retrigger loop before the driver is fully initialized.
    while (!(pio->FSTAT & (1u << (PIO_FSTAT_RXEMPTY_LSB + sm_rx)))) {
        (void)pio->RXF[sm_rx];
    }

    if (sm_rx <= 1U) {
        // Enable exactly one RXNEMPTY source for this instance.
        // Preserving previous bits can leave unrelated sources enabled,
        // causing immediate interrupt storms during early init.
        pio->IRQ0_INTF = 0U;
        pio->IRQ0_INTE = rx_mask;
    } else {
        pio->IRQ1_INTF = 0U;
        pio->IRQ1_INTE = rx_mask;
    }

    nvicEnableVector(cfg().irq_num, PIO_UART_IRQ_PRIO);
}

// ---------------------------------------------------------------------------
// ISR: drain RX FIFO into ring buffer
// ---------------------------------------------------------------------------

void PIORXDriver::_service_rx_fifo()
{
    pio_uart_dbg_rx_service_calls[_instance]++;

    PIO_TypeDef *const pio = cfg().pio;
    const uint8_t      sm  = cfg().sm_rx;
    volatile uint8_t *const rxfifo_byte = ((volatile uint8_t *)&pio->RXF[sm]) + 3;

    // Always drain hardware FIFO if data is present. If this runs before
    // normal initialization has completed, discarding bytes here prevents
    // an IRQ retrigger storm that can starve the main loop.
    while (!(pio->FSTAT & (1u << (PIO_FSTAT_RXEMPTY_LSB + sm)))) {
        // For right-shifted UART RX, the received byte is left-justified in
        // RXF bits [31:24]. The RP2350 datasheet's UART RX example reads the
        // FIFO as an 8-bit access at RXF+3, which pops the FIFO and returns
        // that upper byte directly.
        const uint8_t byte = *rxfifo_byte;
        pio_uart_dbg_rx_bytes[_instance]++;
        if (_readbuf && _initialized) {
            _readbuf->write(&byte, 1);
        }
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

    // SERIAL_CONTROL commonly calls begin() repeatedly with unchanged
    // parameters. Reinitializing PIO SMs on each packet disrupts RX/TX and
    // can inject framing noise into loopback tests.
    if (_initialized && _active_baud == b) {
        return;
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

    if (txSpace == 0) {
        txSpace = PIO_UART_TX_BUF;
    }
    if (_writebuf == nullptr) {
        _writebuf = new ByteBuffer(txSpace);
        if (!_writebuf) {
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

    // Start each session from a clean RX state. During clock/pin bring-up,
    // the RX SM can capture transient bits; purge both hardware FIFO and
    // software ring buffer before userspace traffic starts.
    {
        PIO_TypeDef *const pio = cfg().pio;
        const uint8_t sm = cfg().sm_rx;
        while (!(pio->FSTAT & (1u << (PIO_FSTAT_RXEMPTY_LSB + sm)))) {
            (void)pio->RXF[sm];
        }
        if (_readbuf) {
            _readbuf->clear();
        }
    }

    // Mark initialized before enabling RX IRQ so ISR writes can safely append
    // into the software ring buffer as soon as bytes start arriving.
    _initialized = true;
    _active_baud = b;
    _enable_rx_irq();
    pio_uart_dbg_begin_count[_instance]++;
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
    _active_baud = 0;
}

void PIORXDriver::_flush()
{
    if (!_initialized) {
        return;
    }

    // Wait for the hardware FIFO/shift engine to become idle so callers that
    // require synchronous transmission semantics can force completion.
    const uint32_t start_ms = AP_HAL::millis();
    while (tx_pending() && (AP_HAL::millis() - start_ms) < 50U) {
        hal.scheduler->delay_microseconds(50);
    }
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

void PIORXDriver::_drain_tx_fifo()
{
    if (!_writebuf || !_initialized) {
        return;
    }
    PIO_TypeDef *const pio = cfg().pio;
    const uint8_t      sm  = cfg().sm_tx;

    // Move as many bytes as the TX FIFO has free slots
    uint8_t byte;
    while (pio_tx_level(pio, sm) < PIO_TX_FIFO_DEPTH
           && _writebuf->read_byte(&byte)) {
        pio->TXF[sm] = (uint32_t)byte;
    }
}

size_t PIORXDriver::_write(const uint8_t *buffer, size_t size)
{
    if (!_initialized || !buffer || size == 0) {
        return 0;
    }

    pio_uart_dbg_write_calls[_instance]++;

    PIO_TypeDef *const pio = cfg().pio;
    const uint8_t sm = cfg().sm_tx;
    size_t written = 0;

    // Direct FIFO writes avoid dependence on a periodic TX refill callback.
    // This is important for SERIAL_CONTROL where one write() call may enqueue
    // the full payload and no subsequent write occurs to trigger draining.
    for (size_t i = 0; i < size; i++) {
        const uint32_t wait_start_us = AP_HAL::micros();
        while (pio_tx_level(pio, sm) >= PIO_TX_FIFO_DEPTH) {
            if ((AP_HAL::micros() - wait_start_us) > 20000U) {
                return written;
            }
            hal.scheduler->delay_microseconds(20);
        }
        pio->TXF[sm] = (uint32_t)buffer[i];
        written++;
    }

    pio_uart_dbg_write_bytes[_instance] += written;

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
    PIO_TypeDef *const pio = cfg().pio;
    const uint8_t sm = cfg().sm_tx;
    const uint32_t level = pio_tx_level(pio, sm);
    return (level < PIO_TX_FIFO_DEPTH) ? (PIO_TX_FIFO_DEPTH - level) : 0U;
}

bool PIORXDriver::tx_pending()
{
    if (!_initialized) {
        return false;
    }
    // Pending while the PIO TX FIFO has bytes not yet shifted out.
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
