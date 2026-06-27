#include "AP_Logger_Flash_RP2350M1.h"

#if HAL_LOGGING_FLASH_RP2350M1_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

/*
 * Page layout on M1 (W25Q128, 16 MB total):
 *   0x00000–0x07FFF  32 KB  — parameter storage (AP_STORAGE_M1_FLASH, not touched here)
 *   0x08000–0xFFFFF ~15.97 MB — log region (this driver)
 *
 * AP_Logger_Block page numbering is 1-based; physical M1 offset =
 *   M1_LOG_START + (pageNum - 1) * df_PageSize
 *
 * Geometry (matches W25Q128):
 *   df_PageSize     = 256 bytes   (program page)
 *   df_PagePerBlock = 256 pages   (= 64 KB block / 256 B)
 *   df_PagePerSector= 16  pages   (=  4 KB sector / 256 B)
 *   df_NumPages     = 65408       (= (16 MB - 32 KB) / 256)
 *
 * Async write architecture
 * ========================
 * All M1 write operations issue the command and return immediately.
 * The W25Q128 programs/erases internally while M0 XIP stays active.
 * rp2350_m1_flash_wait_ready() polls SR.BUSY in short (~5 µs) direct-mode
 * bursts with M0 XIP active between polls — so even a 50 ms sector erase
 * never locks M0 for more than ~5 µs at a time.
 *
 * AP_Logger_Block expects Sector4kErase() and SectorErase() to be synchronous
 * (calls them in tight loops), so those call wait_ready() after issue.
 * StartErase() (chip erase) is truly async — InErase() is polled by the IO
 * thread, each poll being one ~5 µs direct-mode window.
 */

void AP_Logger_Flash_RP2350M1::Init()
{
    df_PageSize      = 256U;
    df_PagePerBlock  = 256U;
    df_PagePerSector = 16U;
    /* Total pages in the log region: (16MB - 32KB) / 256 */
    df_NumPages = (16U * 1024U * 1024U - M1_LOG_START) / df_PageSize;

    printf("M1 logger: %u pages (~%u KB) at 0x%08X\n",
           df_NumPages,
           (unsigned)((uint32_t)df_NumPages * df_PageSize / 1024U),
           (unsigned)(RP2350_M1_FLASH_BASE + M1_LOG_START));

    AP_Logger_Block::Init();
}

void AP_Logger_Flash_RP2350M1::PageToBuffer(uint32_t pageNum)
{
    if (pageNum == 0 || pageNum > df_NumPages + 1U) {
        printf("M1 logger: invalid page read %u\n", pageNum);
        memset(buffer, 0xFF, df_PageSize);
        df_Read_PageAdr = pageNum;
        return;
    }
    if (pageNum == df_Read_PageAdr) {
        return;
    }
    df_Read_PageAdr = pageNum;
    const uint32_t off = M1_LOG_START + (pageNum - 1U) * df_PageSize;
    rp2350_m1_flash_read(off, buffer, df_PageSize);
}

void AP_Logger_Flash_RP2350M1::BufferToPage(uint32_t pageNum)
{
    if (pageNum == 0 || pageNum > df_NumPages + 1U) {
        printf("M1 logger: invalid page write %u\n", pageNum);
        return;
    }
    if (pageNum != df_Read_PageAdr) {
        df_Read_PageAdr = 0;
    }
    /* Wait for any in-progress page program or erase to complete.
     * Each poll is ~5 µs in direct mode; M0 XIP active between polls. */
    rp2350_m1_flash_wait_ready();
    const uint32_t off = M1_LOG_START + (pageNum - 1U) * df_PageSize;
    rp2350_m1_flash_program_page(off, buffer, (uint16_t)df_PageSize);
}

/*
 * Erase one 64 KB block.  SectorAdr is the block number (0-based).
 * Synchronous: waits for chip completion in cooperative ~5 µs poll bursts.
 */
void AP_Logger_Flash_RP2350M1::SectorErase(uint32_t SectorAdr)
{
    const uint32_t off = M1_LOG_START + SectorAdr * (uint32_t)df_PagePerBlock * df_PageSize;
    rp2350_m1_flash_erase_block64(off);   /* async: command issued, returns ~5 µs */
    rp2350_m1_flash_wait_ready();         /* cooperative poll; M0 XIP active between polls */
}

/*
 * Erase one 4 KB sector.  SectorAdr is the sector number (0-based).
 * Synchronous: waits for chip completion in cooperative ~5 µs poll bursts.
 */
void AP_Logger_Flash_RP2350M1::Sector4kErase(uint32_t SectorAdr)
{
    const uint32_t off = M1_LOG_START + SectorAdr * (uint32_t)df_PagePerSector * df_PageSize;
    rp2350_m1_flash_erase_sector(off);    /* async: command issued, returns ~5 µs */
    rp2350_m1_flash_wait_ready();         /* cooperative poll; M0 XIP active between polls */
}

/*
 * Whole-chip erase — truly async.  InErase() is polled by the IO thread.
 * Each InErase() call is one ~5 µs direct-mode window.
 */
void AP_Logger_Flash_RP2350M1::StartErase()
{
    rp2350_m1_flash_bulk_erase();
    erase_start_ms = AP_HAL::millis();
    printf("M1 logger: bulk erase started\n");
}

bool AP_Logger_Flash_RP2350M1::InErase()
{
    if (erase_start_ms == 0) {
        return false;
    }
    if (!rp2350_m1_flash_busy()) {
        printf("M1 logger: bulk erase done (%u ms)\n",
               AP_HAL::millis() - erase_start_ms);
        erase_start_ms = 0;
        return false;
    }
    if (AP_HAL::millis() - erase_start_ms > 60000U) {
        printf("M1 logger: bulk erase timeout\n");
        erase_start_ms = 0;
        return false;
    }
    return true;
}

#endif  /* HAL_LOGGING_FLASH_RP2350M1_ENABLED */
