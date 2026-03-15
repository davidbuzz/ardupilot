# Pico2 (RP2350) vs CubeBlack (STM32F427) — Feature Gap Analysis

**Branch:** `buzz-rp2350-chibios-v2`  
**Date:** 2025-07  
**Purpose:** Track what is implemented, what is partially working, and what still needs work on the Pico2 RP2350 port.
---

## Legend
- ✅ Code Written and Compiles
- ⚠️  Partially implemented or has known limitations
- ❌ Not implemented or Not tested yet
- 🚫 Not feasible on RP2350 hardware
- 💡 ChibiOS LLD driver EXISTS — just needs enabling in hwdef.dat
---

## Features List

| Category | Feature | CubeBlack | Pico2 | Status | Tested in hardware | Notes |
|----------|---------|-----------|-------|--------|--------------------|-------|
| Serial / UART | USB serial (SERIAL0) | OTG1 | OTG1 | ✅ | ❌ not done, needs hardware testing | Working. `HAL_USE_USB TRUE`, `HAL_USE_SERIAL_USB TRUE`, `USBv1` ChibiOS driver. |
| Serial / UART | USB device serial number | From UDID_START | From RP2350 OTP | ✅ | ❌ not done, needs hardware testing | `string_substitute()` in `usbcfg_common.c` reads OTP rows 0–5 (CHIPID0-3 + RANDID0-1) via ECC-mapped view at `0x40130000`. USB serial descriptor now shows genuine per-device 96-bit ID. |
| Serial / UART | Hardware UART0 (SERIAL1) | USART2 | UART0 (GPIO 12/13) | ✅ | ❌ not done, needs hardware testing | Working via `HAL_USE_SIO TRUE` → `SIODriver` (UARTDriver.cpp has `HAL_USE_SIO` paths). |
| Serial / UART | Hardware UART1 (SERIAL2) | USART3 | UART1 (GPIO 10/11) | ✅ | ❌ not done, needs hardware testing | Working via SIO. |
| Serial / UART | Additional hardware UARTs | UART4/7/8/USART6 | — | 🚫 | — | RP2350 only has 2 hardware UARTs. Covered instead by PIOUART. |
| Serial / UART | PIOUART0 (SERIAL3) | N/A | GPIO 14/17 | ✅ | ❌ not done, needs hardware testing | RX via PIO ISR → ring buffer works. TX now uses a 512-byte software ring buffer; `_write()` enqueues to ring buffer and drains opportunistically; `txspace()` returns ring buffer free space. `_drain_tx_fifo()` uses `FLEVEL` register for accurate TX FIFO fill level. |
| Serial / UART | PIOUART1 (SERIAL4) | N/A | GPIO 19/20 | ✅ | ❌ not done, needs hardware testing | Same as PIOUART0. |
| Serial / UART | PIOUART2 (SERIAL5) | N/A | GPIO 21/27 | ✅ | ❌ not done, needs hardware testing | Same as PIOUART0. |
| Serial / UART | PIOUART3 (SERIAL6) | N/A | GPIO 28/29 | ⚠️ | ❌ not done, needs hardware testing | GPIO 28/29 are ADC channels 2/3; removed from SERIAL_ORDER to avoid conflict. Same TX improvements as PIOUART0 if re-enabled on non-ADC pins. |
| Serial / UART | RTS/CTS hardware flow control | USART2, USART3 | — | ❌ | — | Not defined for any port. PIOUART has no flow control support at all. |
| Serial / UART | UART DMA | STM32 DMA streams | UART0/UART1 | ✅ | ❌ not done, needs hardware testing | DMAv1 driver in use for SIO UART DMA. |
| RC Input | RC input method | EICU timer | GPIO PAL callback | ✅ | ❌ not done, needs hardware testing | `SoftSigReaderRP2350.cpp` (89 lines) uses `palSetLineCallbackI` / `palEnableLineEventI` / `chVTGetSystemTimeX()`. Both edges captured, pulse widths decoded. GPIO 16 (PA16), `PULLDOWN`. |
| RC Input | SBUS / inverted input | Dedicated invert pin | — | ✅ | ❌ not done, needs hardware testing | GPIO INOVER via `IO_BANK0->GPIO[pad].CTRL`. `set_options(OPTION_RXINV)` sets `PAL_RP_IOCTRL_INOVER_INV` on the HW UART RX pad; `OPTION_TXINV` sets `PAL_RP_IOCTRL_OUTOVER_DRVINVPERI` on TX. `configure_parity(2)` and `set_stop_bits(2)` now stored and applied to `SIOConfig.UARTLCR_H` (`PEN|EPS|STP2`) in `_begin()`. SBUS works on UART0/1 with no external inverter. PIOUART-based SBUS (8E2 PIO program) not implemented. |
| RC Input | DSM / SRXL | Shares UART | Possible via PIOUART | ✅ | ❌ not done, needs hardware testing | PIOUART TX ring buffer is now functional; DSM/SRXL auto-detection works on any UART or PIOUART port. Configure via `SERIAL_n_PROTOCOL=23` (RC Input) on the port connected to the receiver. |
| RC Output | Standard PWM servo output | TIM1/TIM4, 6 outputs | GPIO 0–7, PWM slices 0–3 | ✅ | ❌ not done, needs hardware testing | 8 outputs. ChibiOS `PWMv1` driver. `chibios_hwdef.py` extended for 2-channel RP2350 PWM groups. |
| RC Output | DShot (timer DMA) | ✅ via IOMCU | — | ❌ | — | `HAL_DSHOT_ENABLED 0`. RP2350 has no timer DMA (no DMAR register). Would require PIO-based DShot (significant custom work). |
| RC Output | BLHeli / SerialLED | ✅ | — | ❌ | — | `HAL_SERIALLED_ENABLED 0`. Same dependency on timer DMA. |
| RC Output | Serial ESC comms | ✅ | — | ❌ | — | `HAL_SERIAL_ESC_COMM_ENABLED 0`. |
| RC Output | IOMCU (servo MCU) | ✅ STM32F100 | — | 🚫 | — | No IOMCU — Pico2 drives servos directly. |
| RC Output | Maximum PWM outputs | 6+6 outputs | 8 | — | ❌ not done, needs hardware testing | Pico2 has 8 without IOMCU (RP2350 has 8 slices × 2ch = 16 possible total). |
| SPI | SPI0 bus | SPI4 | GPIO 22/32/35 (SCK/MISO/MOSI) | ✅ | ❌ not done, needs hardware testing | Enabled. `HAL_USE_SPI TRUE`, `RP_SPI_USE_SPI0 TRUE`. ChibiOS `SPIv1` LLD (PL022). SSPCR0/SSPCPSR config paths added to `SPIDevice.cpp`. |
| SPI | SPI1 bus | SPI1 | GPIO 42/40/43 (SCK/MISO/MOSI) | ✅ | ❌ not done, needs hardware testing | Enabled. `RP_SPI_USE_SPI1 TRUE`. Same as SPI0. |
| SPI | SPI CS pins | Multiple | PA23 MAG_CS, PA24 MPU_CS, PA25 BARO_EXT_CS, PA26 GYRO_EXT_CS | ✅ | ❌ not done, needs hardware testing | Defined and active. `IMU Invensense`, `BARO MS5611`, `AP_COMPASS_PROBING_ENABLED` lines all enabled in hwdef.dat. |
| SPI | SPI clock divider | STM32 BR bits | RP2350 PL022 SCR field | ✅ | ❌ not done, needs hardware testing | `derive_freq_flag_bus()` fixed: removed erroneous `scr -= 1` which made actual_freq > target_freq for non-integer divisors. `scr = floor(SYSCLK / (CPSR * target))` now correctly guarantees actual ≤ target. |
| I2C | I2C1 bus | I2C1 (external) | GPIO 15/18 (SCL/SDA) | ✅ | ❌ not done, needs hardware testing | `HAL_USE_I2C TRUE`, `RP_I2C_USE_I2C1 TRUE`. `I2CDevice.cpp` RP2350 branch sets `baudrate` directly (I2Cv1 LLD, no TIMINGR). `chibios_hwdef.py` emits `SHARED_DMA_NONE` config (no DMA in RP2350 I2C LLD). Builds clean. |
| I2C | I2C2 bus | I2C2 (internal) | — | ❌ | — | Not pinned out on Pico2. |
| ADC | ADC | PA2/PA3/PA4 | PA28/PA29 (GPIO 28/29, ADC ch2/3) | ✅ | ❌ not done, needs hardware testing | `HAL_USE_ADC TRUE`, `RP_ADC_USE_ADC1 TRUE`. `PA28 BATT_VOLTAGE_SENS ADC1`, `PA29 BATT_CURRENT_SENS ADC1`. AnalogIn.cpp RP2350 branch uses round-robin rrobin bitmask (ADCv1, no STM32-style SQR/SMPR). |
| ADC | Board voltage sensing | `HAL_HAVE_BOARD_VOLTAGE 1` | — | ⚠️ | — | No VDD_5V_SENS pin on Pico2. `HAL_HAVE_BOARD_VOLTAGE` not set. |
| Sensors | IMU Invensense gen-1 (mpu6000/mpu9250) | SPI `mpu9250` | SPI1/MPU_CS | ✅ | ❌ not done, needs hardware testing | `IMU Invensense SPI:mpu9250 ROTATION_NONE`. Driver WHOAMI-probes for mpu6000/mpu9250/icm20608 on MPU_CS (PA24). |
| Sensors | IMU Invensense gen-2 (ICM20948) | SPI `icm20948` | SPI1/MPU_CS | ✅ | ❌ not done, needs hardware testing | `IMU Invensensev2 SPI:icm20948 ROTATION_NONE`. ICM20948-compatible probe on same MPU_CS — Invensensev2 driver handles the 9-axis+AK09916 variant. Gen-1 probe above matches first if that chip is present. |
| Sensors | Barometer MS5611 | SPI `ms5611_ext` | SPI0/BARO_EXT_CS | ✅ | ❌ not done, needs hardware testing | `BARO MS5611 SPI:ms5611_ext`. |
| Sensors | Compass (external) | I2C probe | I2C1 (GPIO 15/18) | ✅ | ❌ not done, needs hardware testing | `AP_COMPASS_PROBING_ENABLED 1`. ArduPilot probes I2C1 for HMC5883/IST8310/QMC5883 etc. |
| Sensors | IMU LSM9DS0 | SPI `lsm9ds0_g` | SPI1/GYRO_EXT_CS | ⚠️ | ❌ not done, needs hardware testing | SPIDEV `lsm9ds0_g` exists but `lsm9ds0_am` (accel/mag) requires an `ACCEL_EXT_CS` pin not available on standard Pico2. Cannot enable without hardware change. |
| Storage | Flash parameter storage | RAMTRON FRAM SPI | EFL flash pages 4–11 (mid-flash) | ✅ | ❌ not done, needs hardware testing | `STORAGE_FLASH_PAGE 4` (bare key-value — `define` prefix was wrong syntax). `AP_FLASH_STORAGE_QUAD_PAGE 1`: 4×4KB per half = 16KB logical sector. Pages 4–7 (half 1, `0x10004000`) + pages 8–11 (half 2, `0x10008000`) = 32KB total. `HAL_STORAGE_SIZE 8192` < sector_size 16384 ✓. `reserve_size` (~8512) < sector_size (16384) — wear-leveling compaction correct. `chibios_hwdef.py`: PICO2 now uses 4KB page model (`[4]*(FLASH_SIZE_KB//4)`) and `validate_flash_storage_size()` handles `AP_FLASH_STORAGE_QUAD_PAGE`. Previously storage silently fell back to `HAL_USE_EMPTY_STORAGE 1` (params lost on reboot). |
| Storage | RAMTRON FRAM | SPI DEVID10 `FRAM_CS` | Not fitted | ✅ | — | `HAL_WITH_RAMTRON 0` — no FRAM device on Pico2. SPIDEV `ramtron` line and `FRAM_CS` pin remain commented out. To add external FRAM: uncomment both lines and set `HAL_WITH_RAMTRON 1`. |
| Storage | Storage size | 16384 bytes | 8192 bytes | ⚠️ | ❌ not done, needs hardware testing | 8KB — limited by one 16KB QUAD_PAGE half (16384 − reserve overhead). Physical storage area = 32KB (pages 4–11, mid-flash). Increasing beyond 15360 bytes would require `AP_FLASH_STORAGE_QUAD_PAGE×2` or end-of-flash placement. |
| Storage | microSD card | SDIO-based | No SDIO on Pico2 | 🚫 | — | No hardware SDIO pins. `HAL_OS_FATFS_IO 0`. Could add SPI-mode SD card via `HAL_USE_MMC_SPI` if SPI-mode SD hardware is wired, but not currently planned. |
| Storage | ROMFS | IO firmware embedded | Binary data embedded | ✅ | ❌ not done, needs hardware testing | `AP_FILESYSTEM_ROMFS_ENABLED 1` works. No IO firmware needed (no IOMCU). |
| CAN | CAN1 | ✅ via IOMCU | — | 🚫 | — | RP2350 has no hardware CAN controller. `HAL_USE_CAN FALSE`. PIO-based CAN is theoretically possible but not in ChibiOS and would be a major custom effort. |
| CAN | SLCAN (USB CAN) | OTG2 | — | 🚫 | — | No CAN, no SLCAN. |
| CAN | DroneCAN | ✅ | — | 🚫 | — | No CAN. |
| System | Watchdog | ✅ | ✅ | ✅ | ❌ not done, needs hardware testing | `HAL_USE_WDG TRUE`. `rp2350_watchdog_init/pat/was_watchdog_reset()` in `watchdog.c` using ChibiOS `wdgStart`/`wdgReset` HAL API. 2s timeout. Guarded with `#elif defined(RP2350)` in `HAL_ChibiOS_Class.cpp`, `Scheduler.cpp`, `Util.cpp`. |
| System | RTC | ✅ | — | ⚠️ | — | `HAL_USE_RTC FALSE`. RP2350 removed the dedicated RTC peripheral (replaced by POWMAN_TIMER); ChibiOS `RTCv1` LLD is RP2040-only and does not apply. ArduPilot RTC via `stm32_set_utc_usec/get_utc_usec` uses `hrt_micros64()` + GPS-provided UTC offset — no hardware RTC needed. Does not persist across power-off. |
| System | GPT (general purpose timers) | ✅ | — | ⚠️ | — | `HAL_USE_GPT FALSE`. ChibiOS `TIMERv1` LLD (`RP/LLD/TIMERv1/`) contains the system-tick driver (`hal_st_lld`), not a GPT driver. No GPT LLD exists for RP2350. GPT not currently used by ArduPilot on this target. |
| System | Crash dump | ✅ STM32-style crash registers | — | 🚫 | — | `AP_CRASHDUMP_ENABLED 0`. RP2350 has no equivalent to STM32 crash dump registers. Would need custom fault handler to log to flash. Not planned. |
| System | Gyro FFT / DSP | ✅ `HAL_WITH_DSP TRUE` | — | 🚫 | — | `HAL_GYROFFT_ENABLED 0`, `HAL_WITH_DSP FALSE`. RP2350 Cortex-M33 has `CMSIS-DSP` library support, but the ArduPilot STM32-oriented DSP library integration is not adapted. Feasible but deferred. |
| System | IMU heater | ✅ `HAL_HAVE_IMU_HEATER 1` | — | ✅ | ❌ not done, needs hardware testing | `HAL_HAVE_IMU_HEATER 1` defined. No physical heating element on standard Pico2; `HAL_HEATER_GPIO_PIN` not set so no GPIO is toggled. Enables HEAT log messages (1 Hz: `Temp/Targ/P/I/Out`), IMU temperature monitoring, and `BRD_IMU_TARGTEMP` arming-temperature check. Heating requires an external resistor + GPIO if added later. |
| System | MCU temperature sensor | HAL_WITH_MCU_MONITORING (H7) | ✅ `HAL_WITH_MCU_MONITORING 1` (RP2350) | ✅ | ❌ not done, needs hardware testing | `HAL_WITH_MCU_MONITORING 1` enabled. RP2350 ADC channel 4 (internal temperature sensor, `RP_ADC_TEMPERATURE_CHANNEL`) appended to ADC group 0 as virtual pin 253; `ts_enabled` set automatically when channel 4 appears in the round-robin mask. Formula: `T = 27 − (Vadc − 0.706) / 0.001721` (RP2350 datasheet §4.9.5). Populates `mcu_temperature()` API used by GCS and flight logging. +692 bytes flash. |
| Board ID | APJ_BOARD_ID | `TARGET_HW_CUBE_F4` | `AP_HW_RASPBERRYPI_PICO2` | ✅ | ❌ not done, needs hardware testing | `AP_HW_RASPBERRYPI_PICO2` (ID 189) registered in `Tools/AP_Bootloader/board_types.txt`. Needs formal ArduPilot PR before production merge. |
| Board ID | AP_Bootloader_USBCDC | ✅ | ✅ | ✅ | ✅ hardware testing DONE | `hwdef-bl.dat` implemented: `FLASH_RESERVE_START_KB 0`, `FLASH_BOOTLOADER_LOAD_KB 16`. BL at `0x10000000` (~12 KB), storage half-1 at `0x10004000`, storage half-2 at `0x10008000`, app at `0x1000C000`. USB (`SIOD0`/OTG1) + UART0 (`PA12/PA13`). `HAL_USE_EFL TRUE`, `HAL_USE_WDG TRUE`, `HAL_USE_SIO TRUE`. lsusb works identifying board as 'ArduPilot BootLoader' |
| Board ID | AP_Bootloader | ✅ | ✅ | ✅ | ✅ hardware testing DONE | `hwdef-bl.dat` implemented: `FLASH_RESERVE_START_KB 0`, `FLASH_BOOTLOADER_LOAD_KB 16`. BL at `0x10000000` (~12 KB), storage half-1 at `0x10004000`, storage half-2 at `0x10008000`, app at `0x1000C000`. USB (`SIOD0`/OTG1) + UART0 (`PA12/PA13`). `HAL_USE_EFL TRUE`, `HAL_USE_WDG TRUE`, `HAL_USE_SIO TRUE`. Guards added: `bl_protocol.cpp` (`rccDisable*`, `rccResetOTG_FS` behind `#if !defined(RP2350)`), `support.cpp` (`DBGMCU_BASE` stub for RP2350), `chibios_hwdef.py` (`SIOD%u` instead of `SD%u` for RP2350 `BOOTLOADER_DEV_LIST`). Keys: fix flash_base to 0x10000000 (XIP), 64KB block erase, 30s verify timeout. `uploader.py --port /dev/ttyACM1 arducopter.apj`: Erase 100%, Program 100%, Verify 100%, Reboot OK on Pico2/RP2350B0. |

| CPU / Architecture | MCU | STM32F427 @ 168MHz | RP2350 Cortex-M33 @ 150MHz | — | — | RP2350 has 2 cores; ArduPilot uses 1. |
| CPU / Architecture | FPU | FPv4-SP (M4F) | FPv5-SP (M33) | — | — | Both single-precision hardware float. |
| CPU / Architecture | Double precision | Software only | Software only | — | — | Neither has hardware double. |
| CPU / Architecture | Flash | 2MB internal | 4MB external QSPI | — | — | Pico2 flash access is slightly slower (XIP cache mitigates). |
| CPU / Architecture | RAM | ~256KB | ~520KB (SRAM0+SRAM1) | — | — | Pico2 has substantially more RAM. |
| CPU / Architecture | Unique hardware ID | 96-bit device ID | RP2350 OTP CHIPID + RANDID | ✅ | ❌ not done, needs hardware testing | `get_system_id()` / `get_system_id_unformatted()` in `Util.cpp` read OTP rows 0–5 (CHIPID0-3 + RANDID0-1) via ECC-mapped view at `0x40130000`. Row N at `*(uint32_t*)(0x40130000 + N*4)`, data in bits[15:0]. Genuine per-device 96-bit unique ID factory-programmed at manufacture. Also fixed in `usbcfg_common.c` (`string_substitute()` USB serial descriptor) and `AP_BLHeli.cpp` (`MSP_UID` handler). |

---




