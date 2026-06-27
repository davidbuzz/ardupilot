#pragma once

/* hwdef.h defines RP2350 (and other board macros) — must come first. */
#if __has_include(<hwdef.h>)
#include <hwdef.h>
#endif

/*
 * RP2350 QMI M1 flash driver — W25Q128JVPIM blackbox flash on QSPI CS1n.
 *
 * The RP2350 QMI supports two flash devices on the shared QSPI bus:
 *   M0: boot/XIP flash (W25Q64)  — CS = QSPI_SS_N (dedicated pin 75)
 *   M1: blackbox flash (W25Q128) — CS = QSPI_CS1n  (GPIO0, function 9)
 *
 * M1 is memory-mapped at 0x11000000 (read via XIP cache).
 * Erase/write use QMI direct mode with ASSERT_CS1N.
 *
 * Key difference from M0 writes: M0 content is not modified, so the XIP
 * cache does NOT need to be flushed after M1 writes.  Cached M0 code keeps
 * executing from the 16 KB XIP cache during the brief direct-mode window,
 * eliminating the cold-cache stall that follows every M0 param write.
 *
 * Initialisation (GPIO0 FUNCSEL=9 + M1_TIMING) is done in rp_clocks.c
 * before the sys_clk ramp so M1 is ready as early as M0.
 */

#if defined(RP2350)

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define RP2350_M1_FLASH_BASE      0x11000000U
#define RP2350_M1_SECTOR_SIZE     4096U        /* 4 KB erase sector */
#define RP2350_M1_PAGE_SIZE       256U         /* 256 byte program page */

#ifdef __cplusplus
extern "C" {
#endif

/* Call once at runtime to verify M1 is alive (reads JEDEC ID). */
bool rp2350_m1_flash_init(void);

/* Read bytes from M1 via memory-mapped XIP at 0x11000000. */
void rp2350_m1_flash_read(uint32_t offset, void *buf, size_t len);

/* Erase a 4 KB sector.  offset must be 4 KB-aligned.
 * Runs with XIP in direct mode; M0 XIP cache is NOT flushed on exit. */
bool rp2350_m1_flash_erase_sector(uint32_t offset);

/* Erase a 64 KB block.  offset must be 64 KB-aligned.
 * Runs with XIP in direct mode; M0 XIP cache is NOT flushed on exit. */
bool rp2350_m1_flash_erase_block64(uint32_t offset);

/* Start a whole-chip bulk erase (0xC7).  Returns immediately; chip is busy
 * erasing.  Poll rp2350_m1_flash_busy() to detect completion. */
bool rp2350_m1_flash_bulk_erase(void);

/* Non-blocking busy check — returns true if chip is still erasing/programming.
 * Each call is one ~5 µs direct-mode window; M0 XIP active between calls. */
bool rp2350_m1_flash_busy(void);

/* Block until SR.BUSY clears, yielding M0 XIP between each ~5 µs poll.
 * Use this after async erase/program when the caller must wait for completion
 * before issuing the next command (e.g. Sector4kErase → BufferToPage). */
void rp2350_m1_flash_wait_ready(void);

/* Program up to 256 bytes.  offset+len must not cross a 256-byte page.
 * Runs with XIP in direct mode; M0 XIP cache is NOT flushed on exit. */
bool rp2350_m1_flash_program_page(uint32_t offset, const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* RP2350 */
