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



| PIOUART0 (SERIAL3) | N/A | GPIO 14/17 | ✅ RX via PIO ISR → ring buffer works. TX now uses a 512-byte software ring buffer; `_write()` enqueues to ring buffer and drains opportunistically; `txspace()` returns ring buffer free space. `_drain_tx_fifo()` uses `FLEVEL` register for accurate TX FIFO fill level. |
| PIOUART1 (SERIAL4) | N/A | GPIO 19/20 | ✅ Same as PIOUART0. |
| PIOUART2 (SERIAL5) | N/A | GPIO 21/27 | ✅ Same as PIOUART0. |
| PIOUART3 (SERIAL6) | N/A | GPIO 28/29 | ⚠️ GPIO 28/29 are ADC channels 2/3; removed from SERIAL_ORDER to avoid conflict. Same TX improvements as PIOUART0 if re-enabled on non-ADC pins. |
| RTS/CTS hardware flow control | USART2, USART3 | — | ❌ Not defined for any port. PIOUART has no flow control support at all. |
| UART DMA | Yes (STM32 DMA streams) | UART0/UART1 | ✅ DMAv1 driver in use for SIO UART DMA. |

**PIOUART TX fixed:** `txspace()` returns the 512-byte software ring buffer free space. `_drain_tx_fifo()` reads `FLEVEL` for accurate TX FIFO fill level and drains the ring buffer on every `_write()` and `_flush()` call.

---

## 2. RC Input

| Feature | CubeBlack | Pico2 | Status / Notes |
|---------|-----------|-------|----------------|
| RC input method | EICU timer (STM32-specific) | GPIO PAL callback | ✅ `SoftSigReaderRP2350.cpp` (89 lines) uses `palSetLineCallbackI` / `palEnableLineEventI` / `chVTGetSystemTimeX()`. Both edges captured, pulse widths decoded. GPIO 16 (PA16), `PULLDOWN`. |
| SBUS / inverted input | Dedicated invert pin | — | ✅ GPIO INOVER via `IO_BANK0->GPIO[pad].CTRL`. `set_options(OPTION_RXINV)` sets `PAL_RP_IOCTRL_INOVER_INV` on the HW UART RX pad; `OPTION_TXINV` sets `PAL_RP_IOCTRL_OUTOVER_DRVINVPERI` on TX. `configure_parity(2)` and `set_stop_bits(2)` now stored and applied to `SIOConfig.UARTLCR_H` (`PEN|EPS|STP2`) in `_begin()`. SBUS works on UART0/1 with no external inverter. PIOUART-based SBUS (8E2 PIO program) not implemented. |
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
| SPI CS pins | Multiple | PA23 MAG_CS, PA24 MPU_CS, PA25 BARO_EXT_CS, PA26 GYRO_EXT_CS | ✅ Defined and active. `IMU Invensense`, `BARO MS5611`, `AP_COMPASS_PROBING_ENABLED` lines all enabled in hwdef.dat. |

**SPI is the single biggest blocker** — it gates IMU, barometer, compass, RAMTRON storage, and any external SPI sensors.

---

## 5. I2C

| Feature | CubeBlack | Pico2 | Status / Notes |
|---------|-----------|-------|----------------|
| I2C1 bus | I2C1 (external) | GPIO 15/18 (SCL/SDA) | ✅ `HAL_USE_I2C TRUE`, `RP_I2C_USE_I2C1 TRUE`. `I2CDevice.cpp` RP2350 branch sets `baudrate` directly (I2Cv1 LLD, no TIMINGR). `chibios_hwdef.py` emits `SHARED_DMA_NONE` config (no DMA in RP2350 I2C LLD). Builds clean. |
| I2C2 bus | I2C2 (internal) | — | ❌ Not pinned out on Pico2. |

**I2C needed for:** external GPS (most modern GPS modules), compass, airspeed sensors, rangefinders.

---

## 6. ADC / Battery Monitoring

| Feature | CubeBlack | Pico2 | Status / Notes |
|---------|-----------|-------|----------------|
| ADC | PA2/PA3 (voltage/current), PA4 (VDD) | PA28/PA29 (GPIO 28/29, ADC ch2/3) | ✅ `HAL_USE_ADC TRUE`, `RP_ADC_USE_ADC1 TRUE`. `PA28 BATT_VOLTAGE_SENS ADC1`, `PA29 BATT_CURRENT_SENS ADC1`. AnalogIn.cpp RP2350 branch uses round-robin rrobin bitmask (ADCv1, no STM32-style SQR/SMPR). |
| Board voltage sensing | `HAL_HAVE_BOARD_VOLTAGE 1` | — | ⚠️ No VDD_5V_SENS pin on Pico2. `HAL_HAVE_BOARD_VOLTAGE` not set. |

---

## 7. IMU / Barometer / Compass (Sensor Stack)

| Feature | CubeBlack | Pico2 | Status / Notes |
|---------|-----------|-------|----------------|
| IMU Invensense (mpu9250/icm20948/mpu6000) | SPI `mpu9250` | SPI1/MPU_CS | ✅ `IMU Invensense SPI:mpu9250 ROTATION_NONE`. Driver WHOAMI-probes for mpu6000/mpu9250/icm20948 at same CS. |
| Barometer MS5611 | SPI `ms5611_ext` | SPI0/BARO_EXT_CS | ✅ `BARO MS5611 SPI:ms5611_ext`. |
| Compass (external) | I2C probe | I2C1 (GPIO 15/18) | ✅ `AP_COMPASS_PROBING_ENABLED 1`. ArduPilot probes I2C1 for HMC5883/IST8310/QMC5883 etc. |
| IMU LSM9DS0 | SPI `lsm9ds0_g` | SPI1/GYRO_EXT_CS | ⚠️ SPIDEV entry exists; no `IMU LSM9DS0` line added yet (secondary sensor, add if hardware attached). |

**Status:** Primary sensor lines active. ArduPilot skips sensors whose WHOAMI probe fails — safe to run on hardware that only has a subset of sensors wired.

---

## 8. Storage / FRAM

| Feature | CubeBlack | Pico2 | Status / Notes |
|---------|-----------|-------|----------------|
| Flash parameter storage | RAMTRON (FRAM) SPI preferred | EFL flash sectors 1020–1023 | ⚠️ Flash storage works. 8KB (RP2350 4KB sector constraint × 2 with `AP_FLASH_STORAGE_DOUBLE_PAGE 1`). Wear is higher than FRAM. |
| RAMTRON FRAM | SPI DEVID10 `FRAM_CS` | `ramtron` SPIDEV commented out | ❌ `HAL_WITH_RAMTRON 1` defined but SPIDEV line commented out. SPI is now enabled (`HAL_USE_SPI TRUE`); if an external FRAM is wired, uncomment the `ramtron` SPIDEV line. |
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
| Watchdog | ✅ | ✅ | ✅ `HAL_USE_WDG TRUE`. `rp2350_watchdog_init/pat/was_watchdog_reset()` in `watchdog.c` using ChibiOS `wdgStart`/`wdgReset` HAL API. 2s timeout. Guarded with `#elif defined(RP2350)` in `HAL_ChibiOS_Class.cpp`, `Scheduler.cpp`, `Util.cpp`. |
| RTC | ✅ | — | ⚠️ `HAL_USE_RTC FALSE`. RP2350 removed the dedicated RTC peripheral (replaced by POWMAN_TIMER); ChibiOS `RTCv1` LLD is RP2040-only and does not apply. ArduPilot RTC via `stm32_set_utc_usec/get_utc_usec` uses `hrt_micros64()` + GPS-provided UTC offset — no hardware RTC needed. Does not persist across power-off. |
| GPT (general purpose timers) | ✅ | — | ⚠️ `HAL_USE_GPT FALSE`. ChibiOS `TIMERv1` LLD (`RP/LLD/TIMERv1/`) contains the system-tick driver (`hal_st_lld`), not a GPT driver. No GPT LLD exists for RP2350. GPT not currently used by ArduPilot on this target. |
| Crash dump | ✅ STM32-style crash registers | — | 🚫 `AP_CRASHDUMP_ENABLED 0`. RP2350 has no equivalent to STM32 crash dump registers. Would need custom fault handler to log to flash. Not planned. |
| Gyro FFT / DSP | ✅ `HAL_WITH_DSP TRUE` | — | 🚫 `HAL_GYROFFT_ENABLED 0`, `HAL_WITH_DSP FALSE`. RP2350 Cortex-M33 has `CMSIS-DSP` library support, but the ArduPilot STM32-oriented DSP library integration is not adapted. Feasible but deferred. |
| IMU heater | ✅ `HAL_HAVE_IMU_HEATER 1` | — | ❌ Not defined. Could be added as a PWM output + temperature control loop once ADC and IMU work. |

---

## 11. APJ Board ID

| Feature | CubeBlack | Pico2 | Status / Notes |
|---------|-----------|-------|----------------|
| APJ_BOARD_ID | `TARGET_HW_CUBE_F4` | `AP_HW_RASPBERRYPI_PICO2` | ✅ `AP_HW_RASPBERRYPI_PICO2` (ID 189) registered in `Tools/AP_Bootloader/board_types.txt`. Needs formal ArduPilot PR before production merge. |

---

## 12. CPU / Architecture

| Feature | CubeBlack | Pico2 | Notes |
|---------|-----------|-------|-------|
| MCU | STM32F427 Cortex-M4F @ 168MHz | RP2350 Cortex-M33 @ 150MHz | RP2350 has 2 cores; ArduPilot uses 1. |
| FPU | FPv4-SP (M4F) | FPv5-SP (M33) | Both single-precision hardware float. |
| Double precision | Software only | Software only | Neither has hardware double. |
| Flash | 2MB internal | 4MB external QSPI | Pico2 flash access is slightly slower (XIP cache mitigates). |
| RAM | ~256KB | ~520KB (SRAM0+SRAM1) | Pico2 has substantially more RAM. |
| Unique hardware ID | 96-bit device ID | 8-byte QSPI flash UID | ⚠️ `UDID_START = 0x1FFF7A10` in `PICO2.py` (XIP flash mirror, not guaranteed unique). Proper fix: use RP2350 boot ROM `sys_info` or read OTP rows 0x00-0x03 (`0x401C0000`). WIP placeholder is functional but may not be unique across boards. |

---

## Summary: Priority Work Items

### High priority (blocks flight)

1. ~~**SPI driver** (`HAL_USE_SPI TRUE`)~~ — ✅ **DONE** (`SPIv1` LLD enabled; `SPIDevice.cpp` has RP2350 SSPCR0/SSPCPSR paths)
2. ~~**Sensor lines in hwdef.dat**~~ — ✅ **DONE** (`IMU Invensense SPI:mpu9250 ROTATION_NONE`, `BARO MS5611 SPI:ms5611_ext`, `AP_COMPASS_PROBING_ENABLED 1`)
3. ~~**PIOUART TX ring buffer + accurate `txspace()`**~~ — ✅ **DONE** (`ByteBuffer _writebuf` added; `_drain_tx_fifo()` uses `FLEVEL` register; `txspace()` returns ring buffer free space)

### Medium priority (important for functionality)

4. ~~**I2C driver** (`HAL_USE_I2C TRUE`)~~ — ✅ **DONE** (`I2Cv1` LLD enabled; `I2CDevice.cpp` RP2350 baudrate path; `chibios_hwdef.py` PICO2 SHARED_DMA_NONE config)
5. ~~**ADC driver** (`HAL_USE_ADC TRUE`)~~ — ✅ **DONE** (`ADCv1` LLD enabled; `PA28/PA29` as battery volt/curr; `AnalogIn.cpp` RP2350 round-robin path)
6. ~~**Battery monitor pins**~~ — ✅ **DONE** (`HAL_BATT_VOLT_PIN 2`, `HAL_BATT_CURR_PIN 3` active now that ADC is enabled)
7. ~~**Watchdog** (`HAL_USE_WDG TRUE`)~~ — ✅ **DONE** (`WDGv1` LLD enabled; `rp2350_watchdog_init/pat/was_watchdog_reset()` via ChibiOS WDG API; 2s timeout)
8. **APJ_BOARD_ID** — Register a real ID in `board_types.txt`.

### Lower priority (nice to have)

9. **RTC** (`HAL_USE_RTC TRUE`) — Not applicable: RTCv1 LLD is RP2040-only; RP2350 uses POWMAN_TIMER. ArduPilot RTC via GPS time + `hrt_micros64()` offset works without hardware RTC.
10. **GPT timers** (`HAL_USE_GPT TRUE`) — No GPT LLD for RP2350; TIMERv1 is the system-tick driver only.
11. ~~**SBUS invert**~~ — ✅ **DONE** (hardware UART). GPIO INOVER bits set via `IO_BANK0->GPIO[pad].CTRL` in `set_options()`. Parity/stop bits applied in `_begin()` SIOConfig. SBUS on UART0/1 at 100kbps 8E2 inverted. PIOUART 8E2 deferred.
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

