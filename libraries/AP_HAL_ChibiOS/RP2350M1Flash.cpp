/*
 * Inspired by modules/ChibiOS/os/hal/ports/RP/RP2350/hal_efl_lld.c and
 * libraries/AP_HAL_ChibiOS/hwdef/common/flash.c, but cannot use them because:
 *   - hal_efl_lld.c only drives M0 (boot flash, CS0n).  Its exit-XIP sequence
 *     sends QPI-exit commands to CS0 and its CS force functions are hardcoded to
 *     QMI_DIRECT_CSR_ASSERT_CS0N.  There is no CS1 path.
 *   - hal_efl_lld.c unconditionally flushes the XIP cache on re-entry, which is
 *     correct for M0 (content changed) but unnecessary and wasteful for M1 (M0
 *     content is unchanged, so cached code lines remain valid).
 *   - flash.c / hal.flash wrap the ChibiOS EFL API which is M0-only by design.
 * This driver reuses the same JEDEC SPI command set (WREN/PP/SE/RDSR/RDID) but
 * targets QMI CS1n (GPIO0) and skips the XIP cache flush on exit.
 */
#include "RP2350M1Flash.h"

#if defined(RP2350)

#include <string.h>
#include "hal.h"

#define RAMFUNC __attribute__((noinline, section(".ramtext")))

/* JEDEC commands */
#define M1_CMD_WREN          0x06U
#define M1_CMD_RDSR          0x05U
#define M1_CMD_PP            0x02U   /* Page Program */
#define M1_CMD_SE            0x20U   /* Sector Erase 4 KB */
#define M1_CMD_RDID          0x9FU   /* Read JEDEC ID */

#define M1_SR_BUSY           (1U << 0)

/*
 * Enter QMI direct mode.  Saves nothing — caller must not access M0 or M1
 * via XIP while direct mode is active.  M0 XIP cache lines remain valid
 * (M0 content is not modified), so code already in cache continues to run.
 */
RAMFUNC static void m1_enter_direct(void)
{
    QMI_TypeDef *qmi = QMI;
    while (qmi->DIRECT_CSR & QMI_DIRECT_CSR_BUSY) {}
    qmi->DIRECT_CSR = QMI_DIRECT_CSR_EN | QMI_DIRECT_CSR_CLKDIV(3U);
    __asm volatile ("dsb sy" ::: "memory");
}

/*
 * Exit QMI direct mode.  Restores M0 and M1 XIP by clearing DIRECT_CSR_EN.
 * Does NOT flush the XIP cache — M0 content is unchanged so cached lines
 * remain valid.  This is the key difference from M0 writes.
 */
RAMFUNC static void m1_exit_direct(void)
{
    QMI_TypeDef *qmi = QMI;
    while (qmi->DIRECT_CSR & QMI_DIRECT_CSR_BUSY) {}
    qmi->DIRECT_CSR = 0U;
    __asm volatile ("dsb sy" ::: "memory");
    __asm volatile ("isb"    ::: "memory");
}

/* Assert CS1n low (select M1 flash). */
RAMFUNC static void m1_cs_low(void)
{
    QMI->DIRECT_CSR |= QMI_DIRECT_CSR_ASSERT_CS1N;
    (void)QMI->DIRECT_CSR;
}

/* Deassert CS1n high (deselect M1 flash). */
RAMFUNC static void m1_cs_high(void)
{
    QMI->DIRECT_CSR &= ~QMI_DIRECT_CSR_ASSERT_CS1N;
    (void)QMI->DIRECT_CSR;
}

/* Transfer one byte via QMI direct mode (M1 must be selected by caller). */
RAMFUNC static uint8_t m1_xfer_byte(uint8_t tx)
{
    QMI_TypeDef *qmi = QMI;
    while (qmi->DIRECT_CSR & QMI_DIRECT_CSR_TXFULL) {}
    qmi->DIRECT_TX = tx;
    while (qmi->DIRECT_CSR & QMI_DIRECT_CSR_RXEMPTY) {}
    return (uint8_t)qmi->DIRECT_RX;
}

/* Send command byte and optional tx bytes, capture optional rx bytes. */
RAMFUNC static void m1_cmd(uint8_t cmd,
                            const uint8_t *tx, size_t tx_len,
                            uint8_t *rx, size_t rx_len)
{
    m1_cs_low();
    m1_xfer_byte(cmd);
    for (size_t i = 0; i < tx_len; i++) {
        m1_xfer_byte(tx[i]);
    }
    for (size_t i = 0; i < rx_len; i++) {
        uint8_t b = m1_xfer_byte(0U);
        if (rx) rx[i] = b;
    }
    m1_cs_high();
}

/* Wait until M1 flash SR.BUSY clears. */
RAMFUNC static void m1_wait_ready(void)
{
    uint8_t sr;
    do {
        m1_cs_low();
        m1_xfer_byte(M1_CMD_RDSR);
        sr = m1_xfer_byte(0U);
        m1_cs_high();
    } while (sr & M1_SR_BUSY);
}

/* Send 24-bit big-endian address. */
RAMFUNC static void m1_send_addr(uint32_t offset)
{
    m1_xfer_byte((uint8_t)(offset >> 16));
    m1_xfer_byte((uint8_t)(offset >> 8));
    m1_xfer_byte((uint8_t)(offset));
}

/* ---- Public API --------------------------------------------------------- */

bool rp2350_m1_flash_init(void)
{
    /* Read JEDEC ID to verify M1 is alive.  We do this in direct mode so
     * it works even before M1_RFMT/M1_RCMD are confirmed by a real read. */
    m1_enter_direct();
    uint8_t id[3] = {0, 0, 0};
    m1_cmd(M1_CMD_RDID, NULL, 0, id, 3);
    m1_exit_direct();

    /* W25Q128JVPIM: manufacturer=0xEF, memory type=0x70, capacity=0x18 */
    bool ok = (id[0] == 0xEFU && id[1] == 0x70U && id[2] == 0x18U);
    hal.console->printf("M1 flash JEDEC: %02X %02X %02X — %s\n",
                        id[0], id[1], id[2], ok ? "OK" : "UNEXPECTED");
    return ok;
}

void rp2350_m1_flash_read(uint32_t offset, void *buf, size_t len)
{
    memcpy(buf, (const void *)(RP2350_M1_FLASH_BASE + offset), len);
}

bool rp2350_m1_flash_erase_sector(uint32_t offset)
{
    if (offset & (RP2350_M1_SECTOR_SIZE - 1U)) {
        return false;
    }

    m1_enter_direct();

    /* WREN */
    m1_cmd(M1_CMD_WREN, NULL, 0, NULL, 0);

    /* Sector Erase */
    m1_cs_low();
    m1_xfer_byte(M1_CMD_SE);
    m1_send_addr(offset);
    m1_cs_high();

    m1_wait_ready();
    m1_exit_direct();
    return true;
}

bool rp2350_m1_flash_program_page(uint32_t offset, const uint8_t *data, uint16_t len)
{
    /* Must not cross a 256-byte page boundary. */
    if (len == 0U || len > RP2350_M1_PAGE_SIZE) {
        return false;
    }
    if (((offset & (RP2350_M1_PAGE_SIZE - 1U)) + len) > RP2350_M1_PAGE_SIZE) {
        return false;
    }

    m1_enter_direct();

    /* WREN */
    m1_cmd(M1_CMD_WREN, NULL, 0, NULL, 0);

    /* Page Program */
    m1_cs_low();
    m1_xfer_byte(M1_CMD_PP);
    m1_send_addr(offset);
    for (uint16_t i = 0; i < len; i++) {
        m1_xfer_byte(data[i]);
    }
    m1_cs_high();

    m1_wait_ready();
    m1_exit_direct();
    return true;
}

#endif /* RP2350 */
