/*
 * RP2350 QMI M1 flash driver — W25Q128JVPIM on QSPI CS1n.
 *
 * Async write architecture
 * ========================
 * All write/erase operations issue the JEDEC command and return immediately.
 * The chip programs or erases internally while M0 XIP remains fully active.
 * Callers poll rp2350_m1_flash_busy() to detect completion before issuing
 * the next operation.
 *
 * Direct-mode windows are therefore bounded by command clocking time only:
 *   RDSR poll:      ~5 µs   (WREN + cmd + 1 byte at 62.5 MHz)
 *   Sector erase:   ~5 µs   (WREN + SE + 3-byte addr)
 *   Block erase:    ~5 µs   (WREN + D8h + 3-byte addr)
 *   Chip erase:     ~5 µs   (WREN + C7h)
 *   Page program:   ~35 µs  (WREN + PP + 3-byte addr + 256 bytes)
 *
 * During these windows Core1 is parked via the SIO doorbell protocol
 * (rpEflBeforeXipOff / rpEflAfterXipOn) and Core0 IRQs are masked via
 * PRIMASK for the duration of the command transfer only.  The chip then
 * operates internally with both XIP and IRQs fully restored — so the 50 ms
 * sector erase or 1 ms page program is fully concurrent with M0 code
 * execution.  The 5–35 µs IRQ blackout per command is negligible.
 *
 * Relationship to the M0 EFL driver
 * ===================================
 * hal_efl_lld.c only drives M0 (CS0n).  Its CS-force and QPI-exit sequences
 * are hardcoded to CS0.  The M0 driver also flushes the XIP cache on exit —
 * correct for M0 (content changed) but wasteful for M1 (M0 content unchanged,
 * so all cached M0 lines remain valid).
 */
#include "RP2350M1Flash.h"

#if defined(RP2350)

#include <string.h>
#include <stdio.h>
#include "hal.h"

/* All functions that execute while DIRECT_CSR_EN=1 must live in SRAM. */
#define RAMFUNC __attribute__((noinline, section(".ramtext")))

/*
 * Core1 XIP lockout — park Core1 before disabling XIP, unpark after.
 * No-ops before Core1 has armed IRQ26 (c1_xip_lock_ready=0), so storage
 * init (which runs before Core1 starts) is unaffected.
 */
#if defined(RP_CORE1_START) && RP_CORE1_START == TRUE
extern "C" void rpEflBeforeXipOff(void);
extern "C" void rpEflAfterXipOn(void);
#else
static inline void rpEflBeforeXipOff(void) {}
static inline void rpEflAfterXipOn(void)   {}
#endif

/* JEDEC commands */
#define M1_CMD_WREN  0x06U  /* Write Enable */
#define M1_CMD_RDSR  0x05U  /* Read Status Register */
#define M1_CMD_PP    0x02U  /* Page Program */
#define M1_CMD_SE    0x20U  /* Sector Erase 4 KB */
#define M1_CMD_BE64  0xD8U  /* Block Erase 64 KB */
#define M1_CMD_CE    0xC7U  /* Chip Erase */
#define M1_CMD_RDID  0x9FU  /* Read JEDEC ID */

#define M1_SR_BUSY   (1U << 0)

/* ---- Low-level QMI helpers (all RAMFUNC — execute during direct mode) ---- */

/*
 * PRIMASK saved here across m1_enter_direct / m1_exit_direct.
 * IRQs are masked for the duration of the XIP-off window (~5–35 µs) to
 * prevent an uncached IRQ handler from triggering a BusFault while the bus
 * is in direct mode.  Nesting is not needed: enter/exit are always paired
 * on the same thread without re-entry.
 */
static uint32_t m1_saved_primask;

RAMFUNC static void m1_enter_direct(void)
{
    rpEflBeforeXipOff();   /* park Core1 before bus goes to direct mode */
    /* Mask all IRQs so no handler can fetch from flash while XIP is off. */
    uint32_t pm;
    __asm volatile ("mrs %0, primask" : "=r" (pm) :: "memory");
    m1_saved_primask = pm;
    __asm volatile ("cpsid i" ::: "memory");
    QMI_TypeDef *qmi = QMI;
    while (qmi->DIRECT_CSR & QMI_DIRECT_CSR_BUSY) {}
    qmi->DIRECT_CSR = QMI_DIRECT_CSR_EN | QMI_DIRECT_CSR_CLKDIV(3U);
    __asm volatile ("dsb sy" ::: "memory");
}

RAMFUNC static void m1_exit_direct(void)
{
    QMI_TypeDef *qmi = QMI;
    while (qmi->DIRECT_CSR & QMI_DIRECT_CSR_BUSY) {}
    qmi->DIRECT_CSR = 0U;
    __asm volatile ("dsb sy" ::: "memory");
    __asm volatile ("isb"    ::: "memory");
    rpEflAfterXipOn();     /* unpark Core1 — M0 XIP is back on */
    /* Restore IRQ mask — XIP is live again, handlers can run safely. */
    __asm volatile ("msr primask, %0" :: "r" (m1_saved_primask) : "memory");
}

RAMFUNC static void m1_cs_low(void)
{
    QMI->DIRECT_CSR |= QMI_DIRECT_CSR_ASSERT_CS1N;
    (void)QMI->DIRECT_CSR;
}

RAMFUNC static void m1_cs_high(void)
{
    QMI->DIRECT_CSR &= ~QMI_DIRECT_CSR_ASSERT_CS1N;
    (void)QMI->DIRECT_CSR;
}

RAMFUNC static uint8_t m1_xfer_byte(uint8_t tx)
{
    QMI_TypeDef *qmi = QMI;
    while (qmi->DIRECT_CSR & QMI_DIRECT_CSR_TXFULL) {}
    qmi->DIRECT_TX = tx;
    while (qmi->DIRECT_CSR & QMI_DIRECT_CSR_RXEMPTY) {}
    return (uint8_t)qmi->DIRECT_RX;
}

/* Single CS transaction: send cmd, optional tx bytes, optional rx bytes. */
RAMFUNC static void m1_cmd(uint8_t cmd,
                            const uint8_t *tx, size_t tx_len,
                            uint8_t *rx,       size_t rx_len)
{
    m1_cs_low();
    m1_xfer_byte(cmd);
    for (size_t i = 0; i < tx_len; i++) { m1_xfer_byte(tx[i]); }
    for (size_t i = 0; i < rx_len; i++) {
        uint8_t b = m1_xfer_byte(0U);
        if (rx) { rx[i] = b; }
    }
    m1_cs_high();
}

/* Send 24-bit big-endian byte address (after cmd byte, CS already low). */
RAMFUNC static void m1_send_addr(uint32_t offset)
{
    m1_xfer_byte((uint8_t)(offset >> 16));
    m1_xfer_byte((uint8_t)(offset >>  8));
    m1_xfer_byte((uint8_t)(offset));
}

/* ---- Public API ---------------------------------------------------------- */

/*
 * Verify M1 is alive by reading its JEDEC ID.
 * RAMFUNC: code between m1_enter_direct and m1_exit_direct must be in SRAM.
 * printf() is called only after m1_exit_direct (XIP re-enabled).
 * Called once at boot, before Core1 starts — rpEflBeforeXipOff is a no-op.
 */
RAMFUNC bool rp2350_m1_flash_init(void)
{
    m1_enter_direct();
    uint8_t id[3];
    id[0] = 0; id[1] = 0; id[2] = 0;
    m1_cmd(M1_CMD_RDID, nullptr, 0, id, 3);
    m1_exit_direct();

    /* W25Q128JVPIM: 0xEF 0x70 0x18 */
    bool ok = (id[0] == 0xEFU && id[1] == 0x70U && id[2] == 0x18U);
    printf("M1 flash JEDEC: %02X %02X %02X - %s\n",
           id[0], id[1], id[2], ok ? "OK" : "UNEXPECTED");
    return ok;
}

/* Read bytes from M1 via memory-mapped XIP — no direct mode needed. */
RAMFUNC void rp2350_m1_flash_read(uint32_t offset, void *buf, size_t len)
{
    const uint8_t *src = (const uint8_t *)(RP2350_M1_FLASH_BASE + offset);
    uint8_t *dst = (uint8_t *)buf;
    for (size_t i = 0; i < len; i++) {
        dst[i] = src[i];
    }
}

/*
 * Poll SR.BUSY — one short direct-mode window (~5 µs).
 * Returns true while chip is busy erasing or programming.
 * #async — call repeatedly between operations; M0 XIP active between calls.
 */
RAMFUNC bool rp2350_m1_flash_busy(void)
{
    m1_enter_direct();
    m1_cs_low();
    m1_xfer_byte(M1_CMD_RDSR);
    uint8_t sr = m1_xfer_byte(0U);
    m1_cs_high();
    m1_exit_direct();
    return (sr & M1_SR_BUSY) != 0;
}

/*
 * Poll until SR.BUSY clears.  Each iteration is one ~5 µs direct-mode window;
 * M0 XIP is active between iterations.  #async cooperative wait — M0 is never
 * locked out for more than ~5 µs at a time, regardless of erase duration.
 */
void rp2350_m1_flash_wait_ready(void)
{
    while (rp2350_m1_flash_busy()) {}
}

/*
 * Issue a 4 KB sector erase and return immediately. #async
 * Direct-mode window: ~5 µs.  Chip erases internally (~50 ms).
 * Caller must poll rp2350_m1_flash_busy() before next operation.
 */
RAMFUNC bool rp2350_m1_flash_erase_sector(uint32_t offset)
{
    if (offset & (RP2350_M1_SECTOR_SIZE - 1U)) { return false; }
    m1_enter_direct();
    m1_cmd(M1_CMD_WREN, nullptr, 0, nullptr, 0);
    m1_cs_low();
    m1_xfer_byte(M1_CMD_SE);
    m1_send_addr(offset);
    m1_cs_high();
    m1_exit_direct();
    return true;
}

/*
 * Issue a 64 KB block erase and return immediately. #async
 * Direct-mode window: ~5 µs.  Chip erases internally (~150 ms).
 * Caller must poll rp2350_m1_flash_busy() before next operation.
 */
RAMFUNC bool rp2350_m1_flash_erase_block64(uint32_t offset)
{
    if (offset & (65536U - 1U)) { return false; }
    m1_enter_direct();
    m1_cmd(M1_CMD_WREN, nullptr, 0, nullptr, 0);
    m1_cs_low();
    m1_xfer_byte(M1_CMD_BE64);
    m1_send_addr(offset);
    m1_cs_high();
    m1_exit_direct();
    return true;
}

/*
 * Issue a whole-chip erase and return immediately. #async
 * Direct-mode window: ~5 µs.  Chip erases internally (~20–60 s).
 * Caller must poll rp2350_m1_flash_busy() before next operation.
 */
RAMFUNC bool rp2350_m1_flash_bulk_erase(void)
{
    m1_enter_direct();
    m1_cmd(M1_CMD_WREN, nullptr, 0, nullptr, 0);
    m1_cmd(M1_CMD_CE,   nullptr, 0, nullptr, 0);
    m1_exit_direct();
    return true;
}

/*
 * Issue a page program and return immediately. #async
 * Direct-mode window: ~35 µs for a full 256-byte page (260 bytes at 62.5 MHz).
 * Chip programs internally (~1 ms).
 * Caller must poll rp2350_m1_flash_busy() before next operation or read.
 * offset + len must not cross a 256-byte page boundary.
 */
RAMFUNC bool rp2350_m1_flash_program_page(uint32_t offset, const uint8_t *data, uint16_t len)
{
    if (len == 0U || len > RP2350_M1_PAGE_SIZE) { return false; }
    if (((offset & (RP2350_M1_PAGE_SIZE - 1U)) + len) > RP2350_M1_PAGE_SIZE) { return false; }
    m1_enter_direct();
    m1_cmd(M1_CMD_WREN, nullptr, 0, nullptr, 0);
    m1_cs_low();
    m1_xfer_byte(M1_CMD_PP);
    m1_send_addr(offset);
    for (uint16_t i = 0; i < len; i++) { m1_xfer_byte(data[i]); }
    m1_cs_high();
    m1_exit_direct();
    return true;
}

#endif /* RP2350 */
