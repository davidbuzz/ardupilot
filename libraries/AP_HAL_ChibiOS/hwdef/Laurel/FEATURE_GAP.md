# Laurel (RP2350B) vs CubeBlack (STM32F427) — Feature Gap Analysis

**Branch:** `buzz-rp2350-chibios-v3`  
**Date:** 2026-04-04 (initial Laurel variant)  
**Purpose:** Track what is already described in the Laurel hwdef, what is only partially wired up, and what still needs hardware verification on the Laurel RP2350B flight controller target.

This file is the Laurel-target counterpart to `hwdef/Pico2/FEATURE_GAP.md`.
Unlike Pico2, Laurel is not a generic carrier-board reference target. It is a fixed RP2350B flight controller with an onboard IMU, onboard barometer, shared SPI1 microSD/OSD wiring, regulator enables, status LEDs, buzzer, and ADC pins above GPIO29.

Current Laurel status at a glance:

- The board definition exists and is substantially populated in `hwdef.dat` and `hwdef-bl.dat`.
- Core platform features match the Pico2-family RP2350 porting work: USB CDC, hardware UARTs, PIO UARTs, PWM, SPI, I2C, ADC, XIP flash storage, watchdog, and the core1 dispatcher path are all represented in the target configuration.
- Laurel-specific board wiring is now documented in `README.md`, including the fixed ICM42688P + DPS310 sensor stack and the SPI1 either-or choice between microSD and hardware OSD.
- Most Laurel items still need explicit runtime and hardware verification. This document should be updated as bring-up proceeds.

---

## Legend

- ✅ Code written and configured in the target
- ⚠️ Present but only partially implemented, constrained, or still awaiting proof
- ❌ Not implemented or not yet tested on Laurel hardware
- 🚫 Not feasible on Laurel / RP2350 hardware
- 💡 Driver/framework already exists; mainly needs Laurel-specific enablement or verification

---

## Features List

| Category | Feature | CubeBlack | Laurel | Status | Tested in hardware | Notes |
|----------|---------|-----------|--------|--------|--------------------|-------|
| Serial / UART | USB serial (`SERIAL0`) | OTG1 | OTG1 | ✅ | ✅ done | Verified on Laurel hardware: stable MAVLink over `/dev/ttyACM0` at 115200 with repeated HEARTBEAT/SYS_STATUS and STATUSTEXT traffic during 2026-04-05 bring-up captures. Boot crash loop (I2C0 unhandled exception every ~5s) fixed 2026-04-07: root cause was VTOR never reaching SRAM — `hal_lld_init()` unconditionally overwrote it with the flash address; fixed by guarding the reset and deferring the SRAM VTOR write to after `halInit()`. Board now runs indefinitely with zero external resets and VTOR=0x20019e00 (SRAM). |
| Serial / UART | Hardware UART0 (`SERIAL1`) | USART2 | UART0 on GPIO12/GPIO13 | ✅ | ❌ not done | Default role is DVTX / MSP DisplayPort. Present in `SERIAL_ORDER` and pinned in `hwdef.dat`. Needs loopback or attached-peripheral validation on Laurel hardware. |
| Serial / UART | Hardware UART1 (`SERIAL2`) | USART3 | UART1 on GPIO8/GPIO9 | ✅ | ❌ not done | Default role is GPS. Present in `SERIAL_ORDER` and pinned in `hwdef.dat`. Needs baud / pinmux / live GPS verification on Laurel hardware. |
| Serial / UART | PIOUART0 (`SERIAL3`) | N/A | GPIO20/GPIO21 | ✅ | ❌ not done | Default role is RC input (`DEFAULT_SERIAL3_PROTOCOL SerialProtocol_RCIN`). The generic RP2350 PIOUART path exists, but Laurel-specific end-to-end verification is still pending. |
| Serial / UART | PIOUART1 (`SERIAL4`) | N/A | GPIO34/GPIO35 | ✅ | ❌ not done | Default role is MAVLink2 AUX port. Present in `SERIAL_ORDER`; needs loopback or telemetry-radio validation on Laurel hardware. |
| Serial / UART | Extra RX-only pads | N/A | GPIO36/GPIO37 board pads | ❌ | — | Board notes mention additional RX-only pads, but they are intentionally not declared in the current hwdef because this serial path expects full-duplex port definitions. |
| Serial / UART | RTS/CTS hardware flow control | Some STM32 UARTs | Not declared | ❌ | — | Laurel `hwdef.dat` does not currently declare CTS/RTS lines for its UART ports. |
| RC Input | RC input via serial protocol | EICU timer / UART | `SERIAL3` default RCIN | ✅ | ❌ not done | Laurel chooses CRSF / ELRS on PIOUART0 by default. Needs live CRSF/ELRS or SBUS-style runtime verification on the board. |
| RC Output | Standard PWM outputs | 6 + 6 via main + IOMCU | 4 outputs on GPIO28-31 | ✅ | ⚠️ partial | Laurel maps four PWM outputs on RP2350 PWM slices 6 and 7: GPIO28-31 become ArduPilot GPIOs 50-53. Verified 2026-04-08 via OpenOCD: GPIO28/29/30/31 all show CTRL=0x00000004 (FUNCSEL=4 PWM) — PAL alternate-function assignment is correct. Waveform amplitude and frequency still need oscilloscope verification on physical output pins. |
| RC Output | DShot | ✅ | — | 🚫 | — | `HAL_DSHOT_ENABLED 0`. RP2350 still lacks the timer-DMA path ArduPilot uses for DShot on STM32 targets. |
| RC Output | SerialLED / BLHeli-style timing features | ✅ | Onboard RGB LED present but unsupported | 🚫 | — | `HAL_SERIALLED_ENABLED 0`. GPIO39 WS2812 RGB LED was attempted (bit-bang DWT + PIO) but produced no hardware output. Shelved as **will not implement** for now — see Notify/Board-IO rows below. |
| RC Output | IOMCU | ✅ STM32F100 | — | 🚫 | — | Laurel, like Pico2, drives outputs directly and has no IOMCU. |
| SPI | SPI0 IMU bus | SPI1/SPI4 etc. | GPIO2/GPIO4/GPIO3 with CS on GPIO1 | ✅ | ✅ done | Verified on Laurel hardware (2026-04-05): runtime `IO_BANK0->GPIO[2/3/4].CTRL=1` (SPI FUNCSEL) and stable IMU traffic after CS fix. Root cause was custom CS lines not guaranteed to be switched from reset FUNCSEL=NULL to SIO output when using `SPI_SELECT_MODE_PAD`; fixed in `SPIDevice::acquire_bus()` by forcing CS deassert + `PAL_MODE_OUTPUT_PUSHPULL` before select on RP2350. |
| SPI | ICM42688P onboard IMU | Board-specific SPI IMU | SPI0 + GPIO1 CS | ✅ | ✅ done | Verified on Laurel hardware (2026-04-05) with live MAVLink debug: `inv3_try=1 ok=1 who42=0x47 who456=0x00`, plus continuous `ICM dbg` FIFO/sample output (`fail=0`). This confirms working WHOAMI, backend start, FIFO reads, and accel/gyro data publication. |
| SPI | SPI1 shared OSD / microSD bus | SPI buses separated on Cube | GPIO26/GPIO24/GPIO27 | ⚠️ | ❌ not done | Laurel shares SPI1 between the hardware OSD footprint and the microSD socket. The current target chooses microSD and leaves the OSD `SPIDEV` line commented out. This is a board-level either-or constraint, not just a software choice. |
| SPI | AT7456E / MAX7456 hardware OSD | MAX7456-class | AT7456E CS on GPIO17 | ⚠️ | ❌ not done | The OSD framework stays enabled because MSP DisplayPort needs AP_OSD built, but the hardware SPI OSD device is intentionally disabled in the current Laurel target. Enabling it means giving up the current SPI-mode microSD path on the shared bus. Explicitly not hardware tested on Laurel. |
| I2C | I2C0 barometer bus | I2C1/I2C2 | GPIO45/GPIO44 | ✅ | ✅ done | Verified on Laurel hardware: DPS310 backend continuously reports successful transactions (`i2c_fail=0`) while running on I2C0. |
| I2C | DPS310 onboard barometer | MS5611 on CubeBlack variants | I2C0 address `0x76` | ✅ | ✅ done | Verified on Laurel hardware: `DPS310 dbg` stats increment with zero I2C failures and `SYS_STATUS` reports absolute-pressure present/enabled/healthy (`ABS_P=1/1/1`) via MAVLink. |
| I2C | External compass probing | I2C external compass | Same I2C0 bus | ✅ | ❌ not done | `AP_COMPASS_PROBING_ENABLED 1` is set. Needs real external-bus validation if Laurel exposes the bus in a usable connector form. |
| ADC | Battery voltage sense | Analog pin | GPIO40 / ADC0 | ✅ | ⚠️ partial | Live telemetry path verified 2026-04-08 after RP2350B ADC bring-up: requested `SYS_STATUS` at 2 Hz over USB CDC and observed stable near-zero battery readings with no pack connected (`voltage_battery` median 8 mV over 23 samples). Scale remains placeholder (`HAL_BATT_VOLT_SCALE 1.0`), so loaded-pack calibration is still required. |
| ADC | Battery current sense | Analog pin | GPIO41 / ADC1 | ✅ | ⚠️ partial | Live telemetry path verified 2026-04-08 in the same `SYS_STATUS` run: `current_battery` median 3 cA over 23 samples with no load attached, confirming end-to-end ADC/current monitor publication. Scale remains placeholder (`HAL_BATT_CURR_SCALE 1.0`), so analog frontend calibration is still required. |
| ADC | Analog RSSI | Analog pin | GPIO42 / ADC2 | ✅ | ❌ not done | `BOARD_RSSI_ANA_PIN 2` is set for Laurel. Needs live ADC verification and scaling checks. |
| ADC | MCU temperature sensor | STM32 internal temp | RP2350 ADC temp channel | ✅ | ⚠️ partial | Verified 2026-04-08 on Laurel hardware after RP2350B channel-map fixes (`RP2350B_QFN80=1` in hwdef, ADC init on GPIO40-42): live GDB shows `adcgrpcfg[0].rrobin=0x107` (ADC0/1/2 + temp channel 8) and `ts_enabled=true`, confirming internal-temp sampling is active. The reported absolute value is still offset (`_mcu_temperature ~ -9 C`, raw ~953), so board-level calibration/validation is still pending. |
| Storage | Main parameter storage in XIP flash | FRAM / flash | RP2350 main flash | ✅ | ✅ done | Verified on Laurel hardware (2026-04-05): MAVLink parameter stream returned 1151 params including non-default runtime IDs (`BARO1_DEVID`, `INS_ACC_ID`) and OpenOCD reads from `0x10008000` showed non-erased parameter data (`0x51685bfe` header), confirming persistence in the configured `STORAGE_FLASH_PAGE 8` / `AP_FLASH_STORAGE_QUAD_PAGE 1` region. |
| Storage | Flash capacity | 2 MB internal | 8 MB external XIP flash | ✅ | ❌ not done | Laurel declares `FLASH_SIZE_KB 8192`, larger than Pico2. Current layout reserves 32 KB bootloader + 32 KB parameter area, with the rest for the application image. |
| Storage | microSD in SPI mode | SDIO / microSD | SPI1 + CS on GPIO25 | ✅ | ❌ not done | Laurel enables `HAL_USE_MMC_SPI TRUE` and declares `SPIDEV sdcard` on the shared SPI1 bus. Functional card-detect, mount, and logging verification are still pending. Explicitly not hardware tested on Laurel. |
| Storage | Secondary blackbox flash (`W25Q128JVPIM`, 128 Mbit) | Some FCs have dedicated logging flash | Mentioned in board notes | ❌ | ❌ not done | Earlier Laurel notes mention a second blackbox flash device, but the current RP2350 hwdef path does not model it yet. Treat as present-in-board-notes but not integrated and not hardware tested on Laurel. |
| Storage | ROMFS / embedded resources | Embedded data | Enabled | ✅ | ❌ not done | Laurel enables AP_OSD fonts via `ROMFS_WILDCARD libraries/AP_OSD/fonts/font*.bin`. No Laurel-specific ROMFS validation has been recorded yet. |
| Notify / Board IO | Blue status LED | Board LED | GPIO6 | ✅ | ✅ done | Verified 2026-04-07: GPIO6 CTRL=0x00000005 (FUNCSEL=5 SIO), GPIO_OE has bit 6 set. Root cause of earlier FUNCSEL corruption was the VTOR crash loop (now fixed). AP_Notify disabled via `#if PIC02_AVAILABLE` guard in `system.cpp` was re-enabled and `AP_NOTIFY_GPIO_LED_2_ENABLED 1` added to hwdef.dat. GPIO_OUT bit 6 confirmed toggling at runtime via OpenOCD (e.g. samples showing 0xA6/0xC6/0xE6 state cycling). |
| Notify / Board IO | Green status LED | Board LED | GPIO7 | ✅ | ✅ done | Verified 2026-04-07: GPIO7 CTRL=0x00000005 (FUNCSEL=5 SIO), GPIO_OE has bit 7 set. Same root cause and fix as blue LED. Green LED driven HIGH (off) during disarmed no-GPS pattern as expected by AP_BoardLED2. |
| Notify / Board IO | RGB LED | Single-wire RGB LED | GPIO39 | 🚫 | 🚫 will not implement | Attempted 2026-04-05: bit-bang DWT WS2812 driver and PIO NeoPixel path both tried. PIO abandoned (PIOUART conflict). Bit-bang produced **no visible output** on hardware; root cause unresolved. Code discarded. **Will not implement — shelved indefinitely.** |
| Notify / Board IO | 1-wire LED output | Serial LED style output | GPIO39 path | 🚫 | 🚫 will not implement | Same GPIO39 path as RGB LED above. Both approaches (PIO and bit-bang) failed to produce hardware output. Shelved alongside RGB LED. **Will not implement.** |
| Notify / Board IO | Buzzer | Board buzzer | GPIO5 | ✅ | ❌ not done | `HAL_BUZZER_PIN 80` is defined. Needs physical buzzer verification and polarity confirmation. Explicitly not hardware tested on Laurel. |
| Notify / Board IO | 5 V BEC / regulator enable | Board-specific rail control | GPIO14 | ✅ | ✅ done | **Bug fixed 2026-04-08**: `pico2_gpio_init()` in board.c was not initialising this pin — GPIO14 defaulted to FUNCSEL=31 (NULL/Hi-Z) with PAD pull-down, leaving the 5V BEC disabled at boot. Fixed by adding `palSetLine` + `palSetLineMode(PAL_MODE_OUTPUT_PUSHPULL)` for `HAL_GPIO_PIN_BEC_5V_EN`. Verified on hardware: GPIO14 CTRL=0x05 (SIO) and GPIO_OUT bit 14=1 after reflash. |
| Notify / Board IO | 9 V BEC / regulator enable | Board-specific rail control | GPIO15 | ✅ | ✅ done | **Fixed 2026-04-08** alongside BEC_5V_EN: `palClearLine` + `palSetLineMode(PAL_MODE_OUTPUT_PUSHPULL)` added to `pico2_gpio_init()`. Verified on hardware: GPIO15 CTRL=0x05 (SIO), GPIO_OUT bit 15=0 (LOW, 9V BEC held off at boot as intended). |
| Connectors | FTRX connector | Board-specific serial connector | Not broken out in current hwdef | ❌ | ❌ not done | Requested tracking item. The current Laurel hwdef/README do not spell out this connector's exact mapped peripheral path, so keep it listed as a board-level item pending documentation and hardware verification. |
| Connectors | GNSS connector | GPS connector | Expected to map to `SERIAL2` | ⚠️ | ❌ not done | Requested tracking item. Laurel default GPS role is on UART1 (`SERIAL2`), but connector-level validation on the actual GNSS header has not been recorded. Explicitly not hardware tested on Laurel. |
| Connectors | I2C connector | External I2C expansion | Expected to share I2C0 | ⚠️ | ❌ not done | Requested tracking item. `AP_COMPASS_PROBING_ENABLED` is on and Laurel has an I2C0 bus, but connector-level bring-up has not been recorded. Explicitly not hardware tested on Laurel. |
| Connectors | DVTX connector | Video transmitter / MSP DisplayPort | Expected to map to `SERIAL1` | ⚠️ | ❌ not done | Requested tracking item. Laurel defaults `SERIAL1` to MSP DisplayPort, but the physical DVTX connector path has not yet been hardware tested on Laurel. |
| Connectors | Spare UART connector | AUX / spare serial port | Expected to map to `SERIAL4` | ⚠️ | ❌ not done | Requested tracking item. Laurel defines `SERIAL4` on PIOUART1, but spare-port connector validation has not been recorded. Explicitly not hardware tested on Laurel. |
| Connectors | Spare IO | Board-specific spare GPIO / pad | Not documented in current hwdef | ❌ | ❌ not done | Requested tracking item. The current Laurel hwdef does not identify a dedicated spare-IO connector contract, so this remains a documentation and hardware-validation gap. |
| Connectors | J1 ESC connector (8-pin) | ESC motor and power connector | GPIO28-31 (PWM1-4), GPIO37 (TELEM_RX), GPIO41 (BAT_CURRENT) | ⚠️ | ❌ not done | J1 pin 1=VBAT, pin 2=GND, pin 3=BAT_CURRENT (→ GPIO41 ADC1), pin 4=TELEM_RX (→ GPIO37, RX-only, not yet in hwdef), pins 5-8=ESC1-4 (→ GPIO28-31 PWM). PWM FUNCSEL verified on hardware. TELEM_RX not yet declared — awaiting full-duplex serial path support. |
| Power / Sensing | ESC current sensor | Analog / telemetry current path | GPIO41 (`BATT_CURRENT_SENS`, ADC channel 1) on J1 pin 3 | ⚠️ | ❌ not done | Current-sense signal routes from J1 pin 3 (`BAT_CURRENT` → GPIO41). Declared in hwdef as `PA41 BATT_CURRENT_SENS ADC1 SCALE(1)` with placeholder scale. Calibration against the Laurel analog frontend still required. |
| Power / Sensing | ESC voltage sensor | Analog / telemetry voltage path | GPIO40 (`BATT_VOLTAGE_SENS`, ADC channel 0); VBAT rail on J1 pin 1 | ⚠️ | ❌ not done | Battery voltage sensed via GPIO40 (`VBAT_SENSE`). The VBAT supply also appears on J1 pin 1 (power input to ESC). Declared in hwdef as `PA40 BATT_VOLTAGE_SENS ADC1 SCALE(1)` with placeholder scale. Calibration still required. |
| Video | CVBS input | Analog video input | Not modelled in current hwdef | ❌ | ❌ not done | Requested tracking item. The current Laurel target documentation does not describe a validated CVBS input path. Listed here explicitly as not hardware tested on Laurel. |
| Video | CVBS output | Analog video output | Associated with AT7456E/OSD path | ⚠️ | ❌ not done | Requested tracking item. Any CVBS output behavior depends on the hardware OSD/video path, which is not enabled or verified in the current Laurel target. Explicitly not hardware tested on Laurel. |
| System | Watchdog | ✅ | ✅ | ✅ | ❌ not done | Laurel enables `HAL_USE_WDG` and `HAL_WATCHDOG_ENABLED_DEFAULT 1`. Needs real timeout / recovery proof on Laurel hardware. |
| System | Hardware RTC | ✅ | — | 🚫 | — | `HAL_USE_RTC FALSE`. Same RP2350 limitation as Pico2: no ArduPilot hardware RTC implementation here. |
| System | Crash dump | STM32-style crash dump path | Disabled | 🚫 | — | `AP_CRASHDUMP_ENABLED 0`. No Laurel-specific crash-dump backend exists. |
| System | Gyro FFT / DSP | ✅ on higher-end STM32 targets | Disabled | 🚫 | — | `HAL_GYROFFT_ENABLED 0` and `HAL_WITH_DSP FALSE`. Still deferred on RP2350 Laurel. |
| System | IMU heater | Some boards support it | Not fitted | 🚫 | — | `HAL_HAVE_IMU_HEATER 0` in Laurel. |
| Startup / Resilience | Allow boot without barometer | Usually hard-fails | Laurel allows it | ✅ | ❌ not done | `HAL_BARO_ALLOW_INIT_NO_BARO 1` is set, so Laurel is intended to continue booting even if the DPS310 is missing or not responding. This behavior should still be tested explicitly. |
| Startup / Resilience | Allow boot without IMU | Usually hard-fails | Laurel allows it | ✅ | ❌ not done | `AP_INERTIALSENSOR_ALLOW_NO_SENSORS 1` is set. That makes bring-up easier, but Laurel should still verify both the graceful no-sensor path and the normal sensor-present path. |
| Startup / Resilience | Main stack increase for RP2350 bring-up | Board-specific | `MAIN_STACK 0x1000` | ✅ | ❌ not done | Laurel inherits the larger MSP strategy used on RP2350 bring-up to reduce early-startup fault risk. |
| Bootloader | AP bootloader target | ✅ | ✅ | ✅ | ❌ not done | Laurel has a dedicated `hwdef-bl.dat`, `APJ_BOARD_ID AP_HW_RASPBERRYPI_LAUREL`, USB bootloader support, and `APP_START_ADDRESS 0x10010080`. Laurel-specific upload / jump-to-app validation still needs to be recorded. |
| Bootloader | AP bootloader flashing policy | Some boards allow in-app bootloader flashing | Disabled | ✅ | ❌ not done | `AP_BOOTLOADER_FLASHING_ENABLED 0` is set in the application hwdef. This should be kept in mind during bring-up and documentation. |
| Bootloader | ROM BOOTSEL path | MCU ROM support | Physical BOOT + RESET buttons | ✅ | ❌ not done | Laurel board notes document BOOT and RESET buttons wired to the RP2350 strap/reset path. The README describes the intended recovery flow, but it still needs explicit bring-up confirmation. |
| Board ID | APJ board ID | Cube family board IDs | `AP_HW_RASPBERRYPI_LAUREL` | ✅ | ❌ not done | The Laurel board ID is declared in both application and bootloader hwdefs. Runtime upload-tool recognition still needs verification. |
| Filesystem / Logging | FATFS via microSD | SDIO/FAT on STM32 boards | SPI-mode FATFS | ⚠️ | ❌ not done | Laurel currently runs with `HAL_OS_FATFS_IO 0` (bring-up default) while keeping `HAL_USE_MMC_SPI TRUE` available in hwdef. Re-enable FATFS after confirming startup timing and shared-SPI stability with a real microSD card present. |
| Filesystem / Logging | MAVFTP baseline on Laurel | Mature on STM32 targets | Should match standard ArduPilot behavior | ⚠️ | ❌ not done | FTP worker priority raised to PRIORITY_UART+1 (61) on RP2350 to avoid scheduling starvation. `list @SYS` can return a listing, but `get @SYS/threads.txt` and `get @SYS/tasks.txt` still hang/timeout on both pymavlink.mavftp and mavproxy ftp module. Root cause of the `get` stall not yet identified — under active investigation 2026-04-08. |
| Filesystem / Logging | MAVFTP `get @SYS/threads.txt` | Works on mainstream targets | Must work on Laurel | ⚠️ | ❌ not done | `get @SYS/threads.txt` hangs after initial ACK — pymavlink.mavftp stalls indefinitely waiting for ReadFile responses. OpenFileRO ACK arrives, but ReadFile bursts never complete. Under active investigation 2026-04-08. |
| Filesystem / Logging | MAVFTP `get @SYS/tasks.txt` second-attempt behavior | Works on mainstream targets | Must not require retry hacks | ⚠️ | ❌ not done | Same hang as threads.txt — file get stalls after OpenFileRO ACK. Under active investigation 2026-04-08. |
| CPU / Architecture | Dual-core RP2350 model | Single-core STM32 | Core0 ChibiOS + core1 dispatcher | ✅ | ✅ done | Verified 2026-04-07: Core1 starts cleanly. The previously reported FIFO deadlock was caused by the VTOR crash-loop (I2C0 unhandled exception every ~5s) disturbing the PSM / FIFO handshake. PSM FRCE_OFF pre-reset in `start_core1()` ensures a clean ROM wait-for-launch state. After the VTOR fix, `c1_boot_stage = 0x4D` (Core1 idle) and `c1_startup_result = 0xDEADC1C1` confirmed via GDB. Board stable for 15+ seconds with Core1 running. |
| CPU / Architecture | Remove non-standard `serial_hello` thread | Not present in STM32 path | Laurel should use existing UART/USB threads (`OTG1`, `UART_RX`) | ✅ | ✅ done | Code path updated 2026-04-07: ad-hoc `SERHELLO` background thread removed. Direct confirmation via `@SYS/threads.txt` now possible (MAVFTP fixed 2026-04-09): thread listing no longer shows `SERHELLO`. |
| CPU / Architecture | MCU | STM32F427 @ 168 MHz | RP2350B @ 150 MHz | — | — | Laurel uses the RP2350B QFN-80 package, which exposes GPIOs 30-47 and enables the board's ADC40-42 and I2C0 GPIO44/45 choices. |
| CPU / Architecture | RAM | ~256 KB | ~520 KB | — | — | Same RP2350-family RAM budget as Pico2. |
| CPU / Architecture | Flash | 2 MB internal | 8 MB external XIP | — | — | Laurel declares a larger 8 MB flash than the Pico2 reference board. |
| CAN | CAN / DroneCAN | ✅ | — | 🚫 | — | Laurel still runs on RP2350, which has no hardware CAN peripheral. |
| PWM / Safety | Safety switch | Common on Cube class boards | Disabled | ✅ | ❌ not done | `HAL_HAVE_SAFETY_SWITCH 0`. This is a deliberate Laurel target choice. |

---

## Laurel-Specific Gaps To Close

1. Verify basic boot and USB CDC runtime on real Laurel hardware, including AP bootloader jump-to-app behavior.
2. Verify fixed onboard sensors: ICM42688P on SPI0 and DPS310 on I2C0.
3. Verify all declared serial ports, especially `SERIAL3` CRSF/ELRS on PIOUART0 and `SERIAL4` AUX on PIOUART1.
4. Calibrate Laurel battery voltage and current scaling; current hwdef values are placeholders.
5. Validate the SPI1 microSD path and decide whether this target should stay microSD-first or gain a hardware-OSD variant.
6. Confirm board IO polarity and behavior for LEDs, buzzer, and the 5 V / 9 V regulator enable pins.
7. Decide whether Laurel needs support for the currently unmodelled secondary blackbox flash.

---

## Will Not Implement

The following items have been explicitly shelved after investigation determined they are not viable with current hardware knowledge or RP2350 peripheral constraints. They will not be pursued further unless new information changes the situation.

| Item | Reason | Date |  
|------|--------|------|
| **RGB LED / WS2812 on GPIO39** | Bit-bang (DWT cycle-timed) and PIO-NeoPixel drivers both implemented and flashed; neither produced any visible output on real Laurel hardware. PIO path abandoned early due to PIOUART PIO state-machine conflict (PIO0/PIO1 fully allocated by PIOUART). Bit-bang path completed and swept R→G→B at boot with IRQs disabled, but GPIO39 stayed dark. Root cause not identified — possible candidates: wrong GPIO number, GPIO39 FUNCSEL clobbered after `__late_init()`, hardware wiring difference from schematic, or wrong LED protocol. All code discarded/reverted. | 2026-04-05 |

---

## Notes

- Laurel is intentionally more board-specific than Pico2. The fixed sensor and rail wiring should keep this file shorter and less speculative than the Pico2 bring-up log.
- As Laurel hardware tests are performed, replace broad `❌ not done` entries with concrete results, dates, and root-cause notes in the same style used by the Pico2 feature-gap file.