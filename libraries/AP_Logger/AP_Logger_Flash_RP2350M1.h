#pragma once

/*
 * AP_Logger backend for the RP2350 QMI M1 flash (W25Q128, 16 MB).
 *
 * The first 32 KB of M1 is reserved for parameter storage
 * (AP_STORAGE_M1_FLASH).  This logger uses the remainder (~15.97 MB)
 * starting at offset M1_LOG_START (0x8000).
 *
 * Transport: RP2350 QMI direct mode via rp2350_m1_flash_* (RP2350M1Flash.h).
 * No SPI device needed — M1 is accessed directly through QMI CS1n.
 */

#include "AP_Logger_config.h"

#if HAL_LOGGING_FLASH_RP2350M1_ENABLED

#include "AP_Logger_Block.h"
#include <AP_HAL_ChibiOS/RP2350M1Flash.h>

class AP_Logger_Flash_RP2350M1 : public AP_Logger_Block {
public:
    AP_Logger_Flash_RP2350M1(AP_Logger &front, LoggerMessageWriter_DFLogStart *writer) :
        AP_Logger_Block(front, writer) {}

    static AP_Logger_Backend *probe(AP_Logger &front,
                                    LoggerMessageWriter_DFLogStart *ls) {
        return NEW_NOTHROW AP_Logger_Flash_RP2350M1(front, ls);
    }

    void Init(void) override;
    bool CardInserted() const override { return true; }

private:
    /* 32 KB reserved at the start of M1 for parameter storage. */
    static constexpr uint32_t M1_LOG_START = 32768U;

    void BufferToPage(uint32_t PageAdr) override;
    void PageToBuffer(uint32_t PageAdr) override;
    void SectorErase(uint32_t SectorAdr) override;
    void Sector4kErase(uint32_t SectorAdr) override;
    void StartErase() override;
    bool InErase() override;

    uint32_t erase_start_ms;
};

#endif  /* HAL_LOGGING_FLASH_RP2350M1_ENABLED */
