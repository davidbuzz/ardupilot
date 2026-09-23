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
 *
 */
#pragma once

#include <hal.h>

namespace AP_HAL {
class SPIDevice;
}

bool sdcard_init();
bool sdcard_init_raw(uint8_t slowdown, uint8_t tries);
BaseBlockDevice *sdcard_get_block_device();
void sdcard_stop();
bool sdcard_retry();
AP_HAL::SPIDevice *sdcard_get_spi_device();

#if defined(RP2350)
bool sdcard_init_raw_mmc_rp2350(uint8_t sd_slowdown);
void sdcard_retry_rp2350(void);
#endif
