# Pico2 (RP2350) vs CubeBlack (STM32F427) — Feature Gap Analysis

**Branch:** `buzz-rp2350-chibios-v2`  
**Date:** 2025-07  
**Purpose:** Track what is implemented, what is partially working, and what still needs work on the Pico2 RP2350 port.

---

## Legend
- ✅ Working / complete
- ⚠️  Partially implemented or has known limitations
- ❌ Not implemented
- 🚫 Not feasible on RP2350 hardware
- 💡 ChibiOS LLD driver EXISTS — just needs enabling in hwdef.dat

---

## 1. Serial / UART

| Feature | CubeBlack | Pico2 | Status / Notes |
|---------|-----------|-------|----------------|
| USB serial (SERIAL0) | OTG1 | OTG1 | ✅ Working. `HAL_USE_USB TRUE`, `HAL_USE_SERIAL_USB TRUE`, `USBv1` ChibiOS driver. |
| Hardware UART0 (SERIAL1) | USART2 | UART0 (GPIO 12/13) | ✅ Working via `HAL_USE_SIO TRUE` → `SIODriver` (UARTDriver.cpp has `HAL_USE_SIO` paths). |
| Hardware UART1 (SERIAL2) | USART3 | UART1 (GPIO 10/11) | ✅ Working via SIO. |
| Additional hardware UARTs | UART4, UART7, UART8, USART6 (IOMCU) | — | 🚫 RP2350 only has 2 hardware UARTs. Covered instead by PIOUART. |
| PIOUART0 (SERIAL3) | N/A | GPIO 14/17 | ⚠️ RX via PIO ISR → ring buffer works. TX implemented but `txspace()` returns 0 or 1 only (not real FIFO count). `_write()` silently drops bytes if PIO TX FIFO full — no software TX ring buffer. |
| PIOUART1 (SERIAL4) | N/A | GPIO 19/20 | ⚠️ Same issues as PIOUART0. |
| PIOUART2 (SERIAL5) | N/A | GPIO 21/27 | ⚠️ Same issues as PIOUART0. |
| PIOUART3 (SERIAL6) | N/A | GPIO 28/29 | ⚠️ Same issues as PIOUART0. |
| RTS/CTS hardware flow control | USART2, USART3 | — | ❌ Not defined for any port. PIOUART has no flow control support at all. |
| UART DMA | Yes (STM32 DMA streams) | UART0/UART1 | ✅ DMAv1 driver in use for SIO UART DMA. |

**PIOUART TX fix needed:** `txspace()` in `PIOUART.cpp` should read the `FLEVEL` register from the PIO TX FIFO to return actual free slot count (0–8), not just `!pio_sm_is_tx_fifo_full()` clamped to 0/1. Also needs a software TX ring buffer so `_write()` does not discard bytes when FIFO is full.

---

## 2. RC Input

| Feature | CubeBlack | Pico2 | Status / Notes |
|---------|-----------|-------|----------------|
| RC input method | EICU timer (STM32-specific) | GPIO PAL callback | ✅ `SoftSigReaderRP2350.cpp` (89 lines) uses `palSetLineCallbackI` / `palEnableLineEventI` / `chVTGetSystemTimeX()`. Both edges captured, pulse widths decoded. GPIO 16 (PA16), `PULLDOWN`. |
| SBUS / inverted input | Dedicated invert pin | — | ❌ No invert pin defined. Needs hardware inverter or bit-bang approach. |
| DSM / SRXL | Shares UART | Possible via PIOUART | ⚠️ Would work once PIOUART TX is fixed (these use half-duplex UART protocols). |

---

## 3. RC Output (PWM / ESC)

| Feature | CubeBlack | Pico2 | Status / Notes |
|---------|-----------|-------|----------------|
| Standard PWM servo output | TIM1/TIM4, 6 outputs | GPIO 0–7, PWM slices 0–3 | ✅ 8 outputs. ChibiOS `PWMv1` driver. `chibios_hwdef.py` extended for 2-channel RP2350 PWM groups. |
| DShot (timer DMA) | ✅ via IOMCU | — | ❌ `HAL_DSHOT_ENABLED 0`. RP2350 has no timer DMA (no DMAR register). Would require PIO-based DShot (significant custom work). |
| BLHeli / SerialLED | ✅ | — | ❌ `HAL_SERIALLED_ENABLED 0`. Same dependency on timer DMA. |
| Serial ESC comms | ✅ | — | ❌ `HAL_SERIAL_ESC_COMM_ENABLED 0`. |
| IOMCU (servo MCU) | ✅ STM32F100 | — | 🚫 No IOMCU — Pico2 drives servos directly. |
| Maximum PWM outputs | 6 (IOMCU) + 6 (FMU TIM1) | 8 | Pico2 has 8 without IOMCU (RP2350 has 8 slices × 2ch = 16 possible total). |

---

## 4. SPI

| Feature | CubeBlack | Pico2 | Status / Notes |
|---------|-----------|-------|----------------|
| SPI0 bus | SPI4 | GPIO 22/32/35 (SCK/MISO/MOSI) | ✅ Enabled. `HAL_USE_SPI TRUE`, `RP_SPI_USE_SPI0 TRUE`. ChibiOS `SPIv1` LLD (PL022). SSPCR0/SSPCPSR config paths added to `SPIDevice.cpp`. |
| SPI1 bus | SPI1 | GPIO 42/40/43 (SCK/MISO/MOSI) | ✅ Enabled. `RP_SPI_USE_SPI1 TRUE`. Same as SPI0. |
| SPI CS pins | Multiple | PA23 MAG_CS, PA24 MPU_CS, PA25 BARO_EXT_CS, PA26 GYRO_EXT_CS | ⚠️ Defined, SPI enabled. Sensor `IMU`/`BARO`/`COMPASS` lines still commented out in hwdef.dat — need enabling and testing. |

**SPI is the single biggest blocker** — it gates IMU, barometer, compass, RAMTRON storage, and any external SPI sensors.

---

## 5. I2C

| Feature | CubeBlack | Pico2 | Status / Notes |
|---------|-----------|-------|----------------|
| I2C1 bus | I2C1 (external) | GPIO 15/18 (SCL/SDA) | ❌ Pins defined. `HAL_USE_I2C FALSE`. ChibiOS `I2Cv1` LLD **exists** in `RP/LLD/I2Cv1/` and is in `platform.mk`. 💡 Enable `HAL_USE_I2C TRUE`. |
| I2C2 bus | I2C2 (internal) | — | ❌ Not pinned out. |

**I2C needed for:** external GPS (most modern GPS modules), compass, airspeed sensors, rangefinders.

---

## 6. ADC / Battery Monitoring

| Feature | CubeBlack | Pico2 | Status / Notes |
|---------|-----------|-------|----------------|
| ADC | PA2/PA3 (voltage/current), PA4 (VDD) | HAL_BATT_VOLT_PIN 2, HAL_BATT_CURR_PIN 3 | ❌ Pins and scaling defined in hwdef.dat but `HAL_USE_ADC FALSE`. ChibiOS `ADCv1` LLD **exists** in `RP/LLD/ADCv1/` and is in `platform.mk`. 💡 Enable `HAL_USE_ADC TRUE`. |
| Board voltage sensing | `HAL_HAVE_BOARD_VOLTAGE 1` | — | ❌ Blocked by ADC disabled. |

---

## 7. IMU / Barometer / Compass (Sensor Stack)

| Feature | CubeBlack | Pico2 | Status / Notes |
|---------|-----------|-------|----------------|
| IMU MPU9250 | SPI `mpu9250` | SPIDEV entry exists | ❌ `HAL_USE_SPI FALSE` blocks all SPI sensors. No `IMU` lines in hwdef.dat. |
| IMU LSM9DS0 | SPI `lsm9ds0_*` | SPIDEV entry exists | ❌ Same. |
| Barometer MS5611 | SPI `ms5611_ext` | SPIDEV entry exists | ❌ Same. |
| Compass LSM303D | SPI `lsm9ds0_ext_am` | SPIDEV entry exists | ❌ Same. |
| Compass AK8963 | In MPU9250 | SPIDEV entries reference it | ❌ Same. |

**Root cause:** All sensor lines (`IMU`, `BARO`, `COMPASS`) are commented out in `hwdef.dat`. They can be uncommented once `HAL_USE_SPI TRUE` is enabled and tested.

---

## 8. Storage / FRAM

| Feature | CubeBlack | Pico2 | Status / Notes |
|---------|-----------|-------|----------------|
| Flash parameter storage | RAMTRON (FRAM) SPI preferred | EFL flash sectors 1020–1023 | ⚠️ Flash storage works. 8KB (RP2350 4KB sector constraint × 2 with `AP_FLASH_STORAGE_DOUBLE_PAGE 1`). Wear is higher than FRAM. |
| RAMTRON FRAM | SPI DEVID10 `FRAM_CS` | `ramtron` SPIDEV commented out | ❌ `HAL_WITH_RAMTRON 1` defined but SPIDEV line commented out and `HAL_USE_SPI FALSE`. If an external FRAM is wired it could work once SPI is enabled. |
| Storage size | 16384 bytes | 8192 bytes | ⚠️ Half the CubeBlack. Constrained by RP2350 4KB flash page size. |
| microSD card | SDIO-based | No SDIO on Pico2 | 🚫 No hardware SDIO pins. `HAL_OS_FATFS_IO 0`. Could add SPI-mode SD card via `HAL_USE_MMC_SPI` once SPI works, but not currently planned. |
| ROMFS | IO firmware embedded | Binary data embedded | ✅ `AP_FILESYSTEM_ROMFS_ENABLED 1` works. No IO firmware needed (no IOMCU). |

---

## 9. CAN

| Feature | CubeBlack | Pico2 | Status / Notes |
|---------|-----------|-------|----------------|
| CAN1 | ✅ via IOMCU | — | 🚫 RP2350 has no hardware CAN controller. `HAL_USE_CAN FALSE`. PIO-based CAN is theoretically possible but not in ChibiOS and would be a major custom effort. |
| SLCAN (USB CAN) | OTG2 | — | 🚫 No CAN, no SLCAN. |
| DroneCAN | ✅ | — | 🚫 No CAN. |

---

## 10. System / RTOS Features

| Feature | CubeBlack | Pico2 | Status / Notes |
|---------|-----------|-------|----------------|
| Watchdog | ✅ | — | ❌ `HAL_USE_WDG FALSE`. ChibiOS `WDGv1` LLD **exists** in `RP/LLD/WDGv1/` and is in `platform.mk`. 💡 One-line enable. |
| RTC | ✅ | — | ❌ `HAL_USE_RTC FALSE`. ChibiOS `RTCv1` LLD **exists** in `RP/LLD/RTCv1/` and is in `platform.mk`. RP2350 has an AOSC (always-on) oscillator for RTC. 💡 Enable + configure. |
| GPT (general purpose timers) | ✅ | — | ❌ `HAL_USE_GPT FALSE`. ChibiOS `TIMERv1` LLD **exists** in `RP/LLD/TIMERv1/` and is in `platform.mk`. 💡 Enable if needed for tone/beeper or other timer uses. |
| Crash dump | ✅ STM32-style crash registers | — | 🚫 `AP_CRASHDUMP_ENABLED 0`. RP2350 has no equivalent to STM32 crash dump registers. Would need custom fault handler to log to flash. Not planned. |
| Gyro FFT / DSP | ✅ `HAL_WITH_DSP TRUE` | — | 🚫 `HAL_GYROFFT_ENABLED 0`, `HAL_WITH_DSP FALSE`. RP2350 Cortex-M33 has `CMSIS-DSP` library support, but the ArduPilot STM32-oriented DSP library integration is not adapted. Feasible but deferred. |
| IMU heater | ✅ `HAL_HAVE_IMU_HEATER 1` | — | ❌ Not defined. Could be added as a PWM output + temperature control loop once ADC and IMU work. |

---

## 11. APJ Board ID

| Feature | CubeBlack | Pico2 | Status / Notes |
|---------|-----------|-------|----------------|
| APJ_BOARD_ID | `TARGET_HW_CUBE_F4` | `TARGET_HW_CUBE_F4` | ⚠️ **PLACEHOLDER** — Pico2 uses the same board ID as CubeBlack. A dedicated ID needs to be registered in `Tools/AP_Bootloader/board_types.txt` before production use. |

---

## 12. CPU / Architecture

| Feature | CubeBlack | Pico2 | Notes |
|---------|-----------|-------|-------|
| MCU | STM32F427 Cortex-M4F @ 168MHz | RP2350 Cortex-M33 @ 150MHz | RP2350 has 2 cores; ArduPilot uses 1. |
| FPU | FPv4-SP (M4F) | FPv5-SP (M33) | Both single-precision hardware float. |
| Double precision | Software only | Software only | Neither has hardware double. |
| Flash | 2MB internal | 4MB external QSPI | Pico2 flash access is slightly slower (XIP cache mitigates). |
| RAM | ~256KB | ~520KB (SRAM0+SRAM1) | Pico2 has substantially more RAM. |
| Unique hardware ID | 96-bit device ID | 8-byte QSPI flash UID | Both can provide a hardware ID for MAVLink. |

---

## Summary: Priority Work Items

### High priority (blocks flight)

1. ~~**SPI driver** (`HAL_USE_SPI TRUE`)~~ — ✅ **DONE** (`SPIv1` LLD enabled; `SPIDevice.cpp` has RP2350 SSPCR0/SSPCPSR paths)
2. **Sensor lines in hwdef.dat** — Uncomment `IMU`, `BARO`, `COMPASS` lines now that SPI works. Test sensor detection.
3. **PIOUART TX ring buffer + accurate `txspace()`** — Current implementation silently drops bytes; GPS comms need reliable TX.

### Medium priority (important for functionality)

4. **I2C driver** (`HAL_USE_I2C TRUE`) — `I2Cv1` LLD exists. Needed for external GPS, some sensors, airspeed.
5. **ADC driver** (`HAL_USE_ADC TRUE`) — `ADCv1` LLD exists. Needed for battery voltage/current monitoring.
6. **Battery monitor pins** — `HAL_BATT_VOLT_PIN`/`HAL_BATT_CURR_PIN` already defined; will work once ADC enabled.
7. **Watchdog** (`HAL_USE_WDG TRUE`) — `WDGv1` LLD exists. One-line change + small init code. Safety-critical.
8. **APJ_BOARD_ID** — Register a real ID in `board_types.txt`.

### Lower priority (nice to have)

9. **RTC** (`HAL_USE_RTC TRUE`) — `RTCv1` LLD exists.
10. **GPT timers** (`HAL_USE_GPT TRUE`) — `TIMERv1` LLD exists; may be needed for tone/buzzer.
11. **SBUS invert** — Need hardware inverter or investigate RP2350 PIO for UART inversion.
12. **Gyro FFT** — Requires adapting `CMSIS-DSP` for RP2350; deferred.
13. **DShot via PIO** — Complex custom PIO code needed; no ChibiOS driver.
14. **SPI-mode SD card** (`HAL_USE_MMC_SPI`) — Possible once SPI works, but no SD slot on standard Pico2.

### Not feasible on RP2350

- **Hardware CAN** — RP2350 has no CAN peripheral.
- **IOMCU** — Not applicable (Pico2 drives servos directly).
- **STM32 crash dump** — No equivalent hardware.

---

## Files to Edit for SPI/I2C/ADC Enabling

When enabling SPI, I2C, and ADC, the changes needed are:

1. **`hwdef/Pico2/hwdef.dat`** — Change `HAL_USE_SPI FALSE → TRUE`, `HAL_USE_I2C FALSE → TRUE`, `HAL_USE_ADC FALSE → TRUE`.
2. **`hwdef/Pico2/mcuconf.h`** (generated or hand-written) — Set `RP_SPI_USE_SPI0 TRUE`, `RP_SPI_USE_SPI1 TRUE`, `RP_I2C_USE_I2C1 TRUE`, `RP_ADC_USE_ADC1 TRUE` etc. Check `modules/ChibiOS/os/hal/ports/RP/RP2350/mcuconf.h` template.
3. **`hwdef.dat`** — Uncomment `IMU`, `BARO`, `COMPASS` lines (they are already present at bottom of file, commented out).
4. **Verify `SPIDevice.cpp`** has no `PIC02_AVAILABLE`-gated stubs that would swallow SPI transactions silently.
5. **Verify `I2CDevice.cpp`** similarly.
6. **Verify `AnalogIn.cpp`** has RP2350 ADC read paths.

