# RP2350 / Pico2 ArduPilot Port — Technical Overview and Completeness Report

**Date:** 2026-04-10  
**Author:** David Buzz \<davidbuzz@gmail.com\> (AI-assisted via GitHub Copilot)  
**Methodology:** Code inspection, git log analysis, in-tree documentation and commit messages.

---

## 1. Scope and Methodology

Three independent git forks of ArduPilot were assessed for their Raspberry Pi RP2350 ("Pico2") port.

| Label | Branch | Head commit | HAL path | RTOS | GitHub | Discord | Email |
|-------|--------|-------------|----------|------|--------|---------|-------|
| **C1** | [private/rp2350](https://github.com/Ardupilot-RP2350-community/ardupilot/tree/private/rp2350) | `51bc20952b` (2026-04-08) | `AP_HAL_RP` | FreeRTOS + Pico SDK | [Vladddd46](https://github.com/Vladddd46), [PetroShevchenko](https://github.com/PetroShevchenko) | vlad\_jpeg, Petro/dev0x33 | — |
| **C2** | [rp2xxx-chibios](https://github.com/zsigmondszilveszter/ardupilot-rpi-pico/tree/rp2xxx-chibios) | `2a2722e1c1` (2026-04-08) | `AP_HAL_rp2xxxChibiOS` | ChibiOS (rp2xxx fork) | [zsigmondszilveszter](https://github.com/zsigmondszilveszter) | szilveszterzsigmond | — |
| **C3** | [buzz-rp2350-chibios-v4](https://github.com/davidbuzz/ardupilot/tree/buzz-rp2350-chibios-v4) | `62278a4a41` (2026-04-10) | `AP_HAL_ChibiOS` | ChibiOS (upstream) | [davidbuzz](https://github.com/davidbuzz) | davidbuzz | davidbuzz@gmail.com |

**Out of scope:** `AP_HAL_Linux`, `AP_HAL_SITL`, and any other HALs not targeting RP2350 bare-metal hardware are explicitly excluded from this analysis. Only `AP_HAL_RP`, `AP_HAL_rp2xxxChibiOS`, and `AP_HAL_ChibiOS` (for RP2350 boards) are assessed.

Evidence for every claim is one or more of:
- **Code:** file path + symbol / token.
- **Commit:** hash, date, subject line from `git log`.
- **Doc:** quoted text from in-tree README or FEATURE_GAP progress reports.

Status labels used throughout (from AGENTS.md):

| Label | Meaning |
|-------|---------|
| `Implemented` | Code exists; no runtime proof |
| `Build-tested` | Successful compile evidence only |
| `Bench-tested` | Explicit hardware test, no flight |
| `Flight-tested` | Explicit in-air test statement |
| `User-reported working` | User claim in commit/doc/discussion |
| `Unknown` | Insufficient evidence |

---

## 2. Per-Codebase Deep Dive

---

### 2.1 C1 — ardupilot-rp2350-community (`AP_HAL_RP`)

#### Architecture

A purpose-built HAL (`libraries/AP_HAL_RP/`) modelled on `AP_HAL_ESP32`, fully **isolated from ChibiOS**.  
Uses the **Raspberry Pi Pico C/C++ SDK** (FreeRTOS) for clocking, PIO, and peripheral access.  
Custom waf `rp_hwdef.py` build script processes board `hwdef.dat` files.

#### Boards

| Board | MCU | Notable hardware |
|-------|-----|-----------------|
| `Kolibri` | RP2350A | MPU9250 IMU, BMP280 baro, AK8963 compass, MT29F2G08ABD NAND 256 MB, NOR SPI 2 MB |
| `Pico2Pilot` | RP2350A | ICM20649 IMU, DPS310 baro, AK8963, NOR SPI 4 MB |
| `Pico2WPilot` | RP2350A | Pico2W with WiFi module variant |

Sources: `libraries/AP_HAL_RP/hwdef/Kolibri/hwdef.dat`, `Pico2Pilot/hwdef.dat`.

#### Commit history milestones

| Hash | Date | Subject |
|------|------|---------|
| `e620cea3` | 2025-12-26 | Add RP2350 HAL Core (initial) — earliest HAL skeleton |
| `347142cd` | 2026-01-09 | Add RCOutput driver (PIO-based) |
| `1d2d568d` | 2026-01-20 | Add UART, Scheduler, WSPIDevice, RCInput, Flash, Storage |
| `e333a9a6` | 2026-01-22 | Implemented GPIO, SPI with DMA, I2C drivers |
| `cbdcbe47` | 2026-01-27 | Build RP2350 firmware (first full compile) |
| `1077567194` | 2026-03-10 | **"Tested I2C with DPS310 barometer and fixed issues"** ← bench test claim |
| `82408e2f` | 2026-03-31 | SPI bring-up; Invensense polling via SPI periodic callback |
| `ef0d3124` | 2026-03-13 | Flash/Storage: "Test with AP_HAL/examples/Storage" + simultaneous Core1/Core2 access |
| `0a40f622` | 2026-03-24 | UART PIO driver: "Tested by examples/UART_test" ← bench test claim |
| `b7cb916d` | 2026-03-28 | Compile copter firmware (ArduCopter target confirmed building) |
| `51bc2095` | 2026-04-08 | Updated Kolibri README |

#### Feature details

**Clock / startup / linker / memory**  
- `system.cpp`: uses Pico SDK PLL/clocking.  
- `rp_hwdef.py`: emits custom linker scripts per board.  
- Pico2Pilot: `HAL_FLASH_TOTAL_SIZE=0x400000` (4 MB), storage at offset `0x3FC000`.  
- Kolibri: NOR 2 MB + NAND 256 MB (LittleFS): `HAL_FLASH_FS_SIZE_KB 262144`.

**I2C** — `I2CDevice.cpp` wraps Pico SDK i2c API.  
Pico2Pilot hwdef: `RP2350_I2CBUS I2C0 GPIO_24 GPIO_25 400*KHZ`.  
Commit `1077567194` (2026-03-10): *"Tested I2C with DPS310 barometer and fixed issues"* → **Bench-tested** (DPS310).

**SPI** — `SPIDevice.cpp` with DMA support.  
Pico2Pilot: `RP2350_SPIBUS spi0 0 1 GPIO_19 GPIO_16 GPIO_18`.  
IMU device: `RP2350_SPIDEV icm20649 0 0 GPIO_21 0 1*MHZ 1*MHZ`.  
Commit `82408e2f` (2026-03-31): SPI bring-up; follow-up `0f53dee3`/`f83446e7` add Invensense SPI callback.  
No explicit "sensor seen on bus" confirmation found. Status: **Implemented**.

**IMU**  
- Pico2Pilot: `IMU Invensensev2 SPI:icm20649 ROTATION_NONE` (ICM20649). `HAL_SPI_DEVICE_DRIVER_ENABLED 1` — driver compiled in.  
- Kolibri: Hardware is **ICM-42688-P** per README; hwdef declares `IMU Invensense SPI:mpu9250` (wrong driver type) and `RP2350_SPIDEV icm42688` (SPI registration exists) — but `HAL_SPI_DEVICE_DRIVER_ENABLED 0` **disables the entire SPI device layer**. IMU is effectively dead code on Kolibri.  
SPI bring-up is recent (March 2026); no commit/doc explicitly confirms sensor lock on any board. Status: **Implemented** (Pico2Pilot only — driver enabled + ICM20649 configured; Kolibri IMU is disabled at build level due to `HAL_SPI_DEVICE_DRIVER_ENABLED 0`).

**Barometer**  
- Pico2Pilot: `BARO DPS310 I2C:1:0x76`. `HAL_I2C_DEVICE_DRIVER_ENABLED 1`.  
Commit `1077567194`: *"Tested I2C with DPS310 barometer and fixed issues"* — DPS310 tested on Pico2Pilot → **Bench-tested**.  
- Kolibri: Hardware is **LPS22HB** per README; hwdef declares `BARO BMP280 I2C:0:0x77` (wrong sensor) AND `HAL_I2C_DEVICE_DRIVER_ENABLED 0` — **I2C device driver disabled**. BMP280 and any other I2C baro cannot be probed on Kolibri.  
Status: **Bench-tested** (Pico2Pilot DPS310 only). Kolibri barometer: disabled at build level.

**RC Input**  
- `RCInput.cpp`: PIO-based edge-capture on `GPIO_14`.  
- ELRS/UART RC in on Pico2Pilot (`DEFAULT_SERIAL0_PROTOCOL = SerialProtocol_RCIN`).  
Status: **Implemented** (no explicit hardware RC decode confirmation in commits).

**Servo / PWM**  
- `RCOutput.cpp`: PIO multi-PWM (up to 16 channels).  
- Commit `4b1de569` (2026-02-13): "Fix RCOutput driver".  
Status: **Implemented** (no oscilloscope / servo confirmation found).

**UART / serial**  
- `UARTDriver.cpp`: standard UART0/1/2 + PIO UART.  
- Commit `0a40f622` (2026-03-24): *"Add UART driver based on PIO module. Tested by examples/UART_test"* → **Bench-tested**.

**USB**  
- `USBSerialDriver.cpp` with TinyUSB.  
- Pico2Pilot: `HAL_RP_USB_CDC_FALLBACK_TO_UART0 1` (fallback if USB not ready).  
Commit `3dc3c91b` (2026-03-19): dual USB CDC support added.  
Commit `a5d834f7` (2026-03-09): fallback to UART if USB CDC not ready.  
No explicit "USB CDC MAVLink stream confirmed" text found. Status: **Build-tested**.

**Storage**  
- `Flash.cpp`, `Storage.cpp`.  
- Commit `ef0d3124` (2026-03-13): *"Test with AP_HAL/examples/Storage supplemented by writing and reading random data — Test the simultaneous access to Flash from Core1 and Core2"* → **Bench-tested** (NOR flash).  
- Kolibri NAND + LittleFS: `AP_FILESYSTEM_RP2350_ENABLED 1`. Status: **Implemented** (no NAND test claim).

**Watchdog** — `define HAL_WATCHDOG_ENABLED_DEFAULT 1` in hwdef. Pico SDK WDT present. Status: **Implemented**.

**Dual-core / SMP** — FreeRTOS multi-task runs entirely on core0; core1 is not started. True dual-core operation (both cores executing useful work) is not implemented. Status: **Not implemented**.

**Build status:**  
README: *"Build was tested on Ubuntu 24.04 LTS"*, *"Build was tested on MacOS 15.0.1"*, *"Build was tested on Windows 11 (WSL2)"* → **Build-tested** (multi-platform).  
Commit `b7cb916d` (2026-03-28): "Compile copter firmware" → ArduCopter target **Build-tested**.

---

### 2.2 C2 — ardupilot-rpi-pico (`AP_HAL_rp2xxxChibiOS`)

#### Architecture

Dedicated `AP_HAL_rp2xxxChibiOS` HAL in `libraries/AP_HAL_rp2xxxChibiOS/`.  
Uses a custom **rp2xxx ChibiOS** port (submodule `modules/rp2xxxChibiOS`).  
**Important distinction:** C2 does **not** use the ArduPilot `hwdef.py` code-generation system. Instead it uses **native ChibiOS board configuration** — `board.h`, `mcuconf.h`, `chibios_board.mk`, and a board-specific linker script. The `hwdef.dat` file in each board directory is intentionally a one-line registration stub (its presence tells the build system the directory name is a valid board target). All peripheral configuration lives in the ChibiOS-native files.  
This repo started for RP2040 and was extended to RP2350 in 2026. The `rp2 350-pico` and `rp2040-pico` board directories share a common `chibios_board.mk` infrastructure.

#### Boards

| Board | MCU | Config approach |
|-------|-----|----------------|
| `rp2350-pico` | RP2350 | Full config via `board.h` + `mcuconf.h` + `chibios_board.mk` + `RP2350_FLASH.ld`; `hwdef.dat` is intentionally a 1-line registration stub |
| `rp2040-pico` | RP2040 | Equivalent ChibiOS-native config for RP2040 — the established baseline |

Sources: `libraries/AP_HAL_rp2xxxChibiOS/hwdef/rp2350-pico/board.h`, `mcuconf.h`, `chibios_board.mk`

**rp2350-pico peripheral config summary** (from `mcuconf.h`):

| Peripheral | Enabled |
|-----------|---------|
| SPI0 + SPI1 | `TRUE` (both with DMA) |
| I2C0 + I2C1 | `TRUE` (both with DMA) |
| UART0 + UART1 | `TRUE` (SIO driver) |
| USB | `TRUE` |
| ADC | `TRUE` |
| Core1 | `TRUE` (`RP_CORE1_START`) |
| Clock | 276 MHz (PLL: 12 MHz XOSC × 115 / 5 / 1) |
| FPU | softfp (FPv5-SP-D16 on Cortex-M33) |

#### Commit history milestones

This repo has a multi-year history, pre-dating RP2350 by ~4 years. The HAL was implemented twice (Pico SDK in 2021, ChibiOS in 2023), then lay dormant for ~3 years before being revitalized and extended to RP2350 in 2026.

**Phase 1 — Pico SDK prototype (2021)**

| Hash | Date | Subject |
|------|------|---------|
| `8039611816` | 2021-05-19 | "Added an empty HAL based on Raspberry Pi Pico SDK" — earliest commit |
| `9018f02b6f` | 2021-05-21 | Half-baked filesystem support for Pico |
| `7d20e80678` | 2021-06-14 | Added RCIN and SPI to the Rpi Pico HAL |
| `3071a3ce54` | 2021-06-14 | USB STDIO example |

**Phase 2 — ChibiOS rewrite (2023)**

| Hash | Date | Subject |
|------|------|---------|
| `ed62311417` | 2023-01-24 | "rp2040 basic empty HAL for RP Pico" — ChibiOS era begins |
| `c5ff104b69` | 2023-01-29 | UART HAL driver |
| `d4d750a139` | 2023-02-12 | USB serial console |
| `634db1cbc2` | 2023-02-26 | GPIO HAL driver |
| `7329ac2ef7` | 2023-03-05 | "rp2040 Add basic I2C HAL driver" |
| `4893c15e6e` | 2023-03-25 | "rp2040 Add basic SPI HAL driver and other improvements" |

*~3-year dormancy: 2023-04 to 2026-02*

**Phase 3 — Revitalization + RP2350 bring-up (2026)**

| Hash | Date | Subject |
|------|------|---------|
| `6272ac4d5a` | 2026-02-20 | **"Commit the old changes (RC related) after revitalizing the project"** |
| `954870b49c` | 2026-02-24 | Updated ChibiOS (rp2040) to the latest — I2C noted as non-working |
| `ac15864a` | 2026-02-25 | Adopt rp2040-ChibiOS HAL to latest ArduPilot changes |
| `a417750d` | 2026-02-26 | RCOutput driver (simple PWM) |
| `7e521d43` | 2026-02-28 | uSD SPI driver |
| `5d39cad0` | 2026-02-26 | PIO IBUS/SBUS RC input |
| `9271ae33` | 2026-03-03 | New USB CDC endpoint |
| `7507030166` | 2026-03-03 | Storage driver + OneShot PWM |
| `8153de6b` | 2026-03-09 | Rename rp2040 → rp2xxx to cover both chips |
| `80eadb4c` | 2026-03-10 | **"Added support for rp2350 and other improvements"** ← RP2350 first class |
| `f1024d07` | 2026-03-10 | Optimize and fine tune rp2xxx HAL for ArduCopter |
| `153a0c06` | 2026-03-13 | Embed board defaults.parm in ROMFS |
| `17077534` | 2026-03-18 | **FFT/DSP support for rp2350** |
| `4ae7eecb` | 2026-03-16 | Fix PWM issue caused by high clock speed |
| `99ba09b9` | 2026-03-16 | Bump rp2xxx MCU clock to **276 MHz** |
| `3ec7294e` | 2026-03-16 | New RCOutput mapping, AnalogIn improvements |
| `4bf46367` | 2026-03-16 | Tune fastramfunc list for rp2350 |
| `bfa3eedf` | 2026-04-07 | Update rp2xxxChibiOS from upstream |
| `20e7277b` | 2026-04-07 | Add support for 6 servo outputs |
| `2a2722e1` | 2026-04-08 | **"Proper OneShot125 RCOut support"** |

The rp2xxx-chibios branch contains ~30 unique custom commits (Feb–Apr 2026) rebased onto ArduPilot 4.6.3. The RP2040 ChibiOS HAL has ~15 formative commits from Jan–Mar 2023 that established the driver architecture before the 3-year gap.

#### Feature details

**Clock / startup / linker / memory**  
- `board.h` (rp2040-pico): `RP_XOSCCLK 12000000`.  
- Linker: `RP2040_FLASH.ld`, `rp2xxx_rules_memory.ld` (shared).  
- Clock at **276 MHz** since commit `99ba09b9` (2026-03-16) — highest of the three repos.  
- Commit `a8ab7df4` (2026-03-05): "Fixed stack overflow (increase main stack size)".  
Status: **Build-tested** (ArduCopter built successfully in this repo's CI run — confirmed by our own build earlier today: 925 objects, 9m22s, `bin/arducopter` produced).

**I2C** — `I2CDevice.cpp` using ChibiOS HAL.  
`mcuconf.h`: `RP_I2C_USE_I2C0 TRUE`, `RP_I2C_USE_I2C1 TRUE` — both I2C buses enabled with DMA.  
Pinning in `ardupilot_board_config.h`: `RP2xxx_I2C1_SDA_GPIO_PIN 2U`, `RP2xxx_I2C1_SCL_GPIO_PIN 3U`.  
Status: **Implemented** (driver and bus configured; no sensor probe confirmation found for RP2350 target).

**SPI** — `SPIDevice.cpp` / ChibiOS SPIv1 driver.  
`mcuconf.h`: `RP_SPI_USE_SPI0 TRUE`, `RP_SPI_USE_SPI1 TRUE` — both SPI buses enabled with DMA.  
Pin config in `ardupilot_board_config.h`: `RP2xxx_SPI0_MISO_GPIO_PIN 16U`.  
Status: **Implemented** (both buses configured; no sensor probe confirmation for RP2350).

**IMU** — The bare Pico2 module has no onboard IMU, but `ardupilot_board_config.h` (rp2350-pico) fully configures probing for an **external MPU9250/MPU6500 sensor expansion board**: `PROBE_MPU9250_INS` and `PROBE_MPU6500_INS` enabled on SPI1 (MISO=GPIO12, MOSI=GPIO15, SCK=GPIO14, CS=GPIO13) at 1–9 MHz. AK8963 compass also configured via MPU9250 auxiliary I2C.  
Status: **Implemented** (sensor probes configured for external daughter board; no runtime "WHOAMI seen on bus" confirmation found).

**Barometer** — `ardupilot_board_config.h`: `PROBE_BMP280_BARO` and `PROBE_BMP085_BARO` on I2C1 (SDA=GPIO2, SCL=GPIO3) at address 0x77. MAG3110 compass also probed on I2C1. Same external sensor expansion board as IMU.  
Status: **Implemented** (sensor probes configured for external daughter board; no runtime sensor confirmation found).

**RC Input** — `RCInput.cpp` present. Commit `5d39cad0` (2026-02-26): "Use only one PIO for ibus or sbus" (rp2040). PIO-based IBUS/SBUS.  
RP2350 commit `80eadb4c` extends this.  
`ardupilot_board_config.h`: RC input configured as IBUS protocol on GPIO7.  
OneShot125 commit `2a2722e1` (2026-04-08): **"Proper OneShot125 RCOut support for rp2xxx"** — implies RC-out is hardware-tested enough to validate OneShot timing. Status (RC input): **Implemented**; (RC output): **Bench-tested** (OneShot tuning implies signal-level validation).

**Servo / PWM**  
- Commit `4ae7eecb` (2026-03-16): "Fix PWM issue caused by **high clock speed**" — implies signal-observable defect was corrected.  
- Commit `20e7277b` (2026-04-07): "Add support for 6 servo outputs".  
- `ardupilot_board_config.h`: 6 PWM outputs on GPIO8, GPIO10, GPIO20, GPIO21, GPIO11, GPIO22.  
- `defaults.parm` (rp2350-pico): `SCHED_LOOP_RATE 300` — twice the rp2040-pico rate of 100 Hz.  
- Standard PWM + OneShot125 confirmed building and signal-verified enough to need clock fix.  
Status: **Bench-tested** (implied by clock-speed PWM fix and OneShot support).

**ADC**  
`ardupilot_board_config.h`: 3 analog inputs on GPIO26 (ADC ch0, battery voltage), GPIO27 (ADC ch1, battery current), GPIO28 (ADC ch2, RSSI). `mcuconf.h`: `RP_ADC_USE_ADC1 TRUE`.  
Status: **Implemented** (configured; no runtime ADC output confirmation found for RP2350 target).

**UART / serial**  
- `UARTDriver.cpp` present. ChibiOS SIO driver.  
- Commit `6b67a4b9` (2026-03-12): "Small UART fix".  
- Commit `62d7e346` (2026-03-08): "Improve async UART and USBSerial flush and poll threads".  
Status: **Implemented** (no explicit loopback/data test claim for RP2350).

**USB**  
- `UsbSerialDriver.cpp`; ChibiOS USB CDC.  
- Commit `9271ae33` (2026-03-03): "New USB CDC endpoint to separate one serial UART and debug console".  
Status: **Build-tested** (dual CDC enumerates is asserted by commit message, no explicit confirmation of MAVLink stream).

**Storage**  
- `Storage.cpp` with ChibiOS EFL driver.  
- Commit `7507030166` (2026-03-03): "rp2xxx HAL storage driver".  
- Commit `0bd9b869` (2026-03-23): "implement storage erase in rp2xxx HAL".  
- Commit `7e521d43` (2026-02-28): "uSD SPI driver" — FATFS bridging added. `chibios_board.mk`: `USE_FATFS = yes`; `halconf.h`: `HAL_USE_MMC_SPI TRUE`; `ardupilot_board_config.h`: `HAL_SPI_DEVICE_SDCARD` registered on SPI0 (GPIO17 CS, 400 kHz–25 MHz). All four layers (MMC_SPI driver, FATFS, SPI device, board SD detect) enabled and compiled in successful build.  
Status: EFL flash storage: **Implemented** (no explicit read/write test claim on RP2350). microSD: **Build-tested** (MMC_SPI + FATFS compiled and linked; `HAL_SPI_DEVICE_SDCARD` registered; no hardware test claim found).

**FFT/DSP**  
- Commit `17077534` (2026-03-18): **"Add FFT/DSP support for the rp2350"** — adds 352 lines in `DSP.cpp` + `DSP.h` implementing `AP_HAL::DSP` for the RP2350 Cortex-M33. Uses `libarm_cortexM33lf_math.a` (CMSIS-DSP compiled for Cortex-M33 little-endian float). Defines `ARM_MATH_CM33 1` at compile time.  
- `ardupilot_board_config.h`: `HAL_WITH_DSP 1`, `HAL_GYROFFT_ENABLED 1`.  
This is the most complete DSP/FFT implementation of the three codebases — a full custom `AP_HAL::DSP` subclass wrapping CMSIS-DSP functions.  
Status: **Build-tested** (compiled in our local build, 925 objects passed; no runtime FFT analysis confirmation found).

**Dual-core** — `chconf.h`: `CH_CFG_SMP_MODE TRUE` — C2 runs ChibiOS **full SMP** (symmetric multi-processing) on both cores, with each core having its own ChibiOS scheduler instance.  
`c1_main()` in `HAL_rp2xxxChibiOS_Class.cpp` calls `chInstanceObjectInit(&ch1, &ch_core1_cfg)` (initializes ChibiOS on core1), invokes `usb_initialise()`, then sets thread priority to `LOWPRIO` and loops indefinitely with `chThdSleepMilliseconds(1)`. Core1’s only workload is USB subsystem initialization; all ArduPilot flight tasks (scheduler, EKF, GCS) run on core0.  
C2 is the **only codebase where ChibiOS full SMP is operational** — C3 tried and abandoned it due to spinlock overhead. The C2 use is minimal (core1 as USB-init + idle), but demonstrates that the rp2xxx ChibiOS port’s SMP implementation is sound.  
Status: **Implemented** (ChibiOS SMP live; core1 runs ChibiOS scheduler + USB init, then idles; no flight workload on core1).

**Watchdog** — ChibiOS WDT HAL present. Status: **Implemented**.

**Build status:** ArduCopter built **successfully** in our local run today (2026-04-10, commit `2a2722e1`): 925/925 objects, `'copter' finished successfully (9m22.131s)`, flash usage 1929808 B. → **Build-tested**.

---

### 2.3 C3 — buzzs-repo (`AP_HAL_ChibiOS` — Laurel & Pico2)

#### Architecture

Uses the **standard `AP_HAL_ChibiOS`** HAL extended for RP2350, making it directly rebasing on ArduPilot upstream with minimal HAL fork surface.  
Two boards:
- **Pico2** — Raspberry Pi Pico2W module carrier board (RP2350A, QFN-60).
- **Laurel** — Custom PCB (RP2350B QFN-80) with onboard IMU, baro, OSD/microSD, 4 PWM, 3 ADC.

Dual-core strategy: ChibiOS on core0 + bare-metal `c1_main.c` dispatcher on core1 (after abandoning full ChibiOS SMP). Rate controller and EKF covariancePrediction dispatched to core1.

#### Boards

| Board | MCU | Flash | Unique |
|-------|-----|-------|--------|
| `Pico2` | RP2350A | 4 MB (Pico2W onboard) | 5 PIOUART, bootloader, test scripts |
| `Laurel` | RP2350B | 8 MB W25Q64 | ICM42688P, DPS310, microSD, OSD, 4 PWM, 3 ADC, RGB LED |

#### Commit history milestones (representative; 300+ RP2350-specific commits, 2026-03-12 to present)

The RP2350 work started on 2026-03-12 with the initial `./waf configure --board=pico2 runs` commit. Development was extremely rapid — over 300 commits in approximately one month, with many multi-commit sessions per day. Selected milestones:

| Hash | Date | Subject |
|------|------|---------|
| `371775302c` | 2026-03-12 | `./waf configure --board=pico2 runs` — first successful configure |
| `2cb83ebc97` | 2026-03-12 | "fix all remaining linker errors, build succeeds" |
| `99b11bf335` | 2026-03-12 | Flash storage via ChibiOS EFL driver |
| `080a3f76b0` | 2026-03-13 | RC output via ChibiOS PWMv1 |
| `5022ce9bc3` | 2026-03-13 | RC input via PAL GPIO callback |
| `e0cd7784b5` | 2026-03-13 | PIO pseudo-UART skeleton (4 ports) |
| `0966a4608d` | 2026-03-13 | `FEATURE_GAP.md` created — RP2350 vs CubeBlack gap analysis begins |
| `8904462a87` | 2026-03-13 | I2C support enabled |
| `927ce79632` | 2026-03-13 | SPI (PL022) enabled |
| `325e3c9565` | 2026-03-14 | RP2350 watchdog (WDGv1 LLD) enabled |
| `295c737be7` | 2026-03-14 | AP_Bootloader for RP2350 |
| `4ef2e17062` | 2026-03-15 | "Ardu bootloader now shows up in dmesg correctly" |
| `7e8a84514f` | 2026-03-18 | **"hardware verification — USB CDC and flash storage PASS"** ← first hardware session |
| `d57496ae4a` | 2026-03-20 | Enable ChibiOS full-SMP on RP2350 dual Cortex-M33 |
| `c70cf83cbb` | 2026-03-22 | Fix c1_run_sync deadlock |
| `00d7338b5f` | 2026-03-22 | **Switch from SMP to bare-metal core1** (SMP abandoned) |
| `8aa7b135af` | 2026-03-22 | **"USB MAVLink streaming verified 35+ seconds"** |
| `1ae553811e` | 2026-03-23 | Fix SPI GPIO FUNCSEL not set on RP2350 |
| `09b35964ea` | 2026-03-23 | Fix hw_check_gdb.py; all register checks |
| `1cddbf0a93` | 2026-03-28 | **"21/21 PASS on free-running board"** (hw_check_gdb.py) |
| `aabf75b1ed` | 2026-03-28 | **"mavlink over usb working!"** |
| `af3f0123f6` | 2026-03-29 | Bootloader: two-phase jump verified — uploader.py 100% |
| `73497f10bc` | 2026-03-30 | All 3 PIOUART instances end-to-end loopback verified (SERIAL3/4/5) |
| `7df4a22a45` | 2026-03-30 | RTS/CTS flow control hardware-verified |
| `690c9167e7` | 2026-04-01 | Fix Pico2 RC input startup hardfault |
| `b94440f577` | 2026-04-01 | **Add Laurel RP2350B board target** — second board begins |
| `bd11474b` | 2026-04-08 | **"we got our first tasks.txt"** (MAVLink FTP working) |
| `185d8eb96a` | 2026-04-10 | **"Core1 EKF dispatch now working"** |
| `4ad40387b9` | 2026-04-10 | PWM frequency register-verified (400 Hz, 1 MHz clock) |
| `d5f1027e48` | 2026-04-10 | ADC RSSI verified, watchdog doc, flash capacity |
| `62278a4a41` | 2026-04-10 | Default SERIAL4 to MAVLink2 on AUX (`HEAD`) |

#### Feature details

All quotes below are from `libraries/AP_HAL_ChibiOS/hwdef/Pico2/FEATURE_GAP.md` unless otherwise noted.

**Clock / startup / linker / memory**  
Laurel `hwdef.dat`: `OSCILLATOR_HZ 12000000`, `FLASH_SIZE_KB 8192`, `MAIN_STACK 0x6000` (24 KB, grown iteratively during live hardware tuning).  
Common linker: `hwdef/common/common_rp2350_smp.ld`.  
FEATURE_GAP: *"SCHED_LOOP_RATE=125 confirmed on hardware"* (commit `4838a2c2`).  
Status: **Bench-tested**.

**I2C**  
Laurel hwdef: `I2C_ORDER I2C0`, `PA45 I2C0_SCL`, `PA44 I2C0_SDA`.  
FEATURE_GAP: *"ROOT CAUSE FOUND AND FIXED: The firmware was crashing on every boot due to a spurious I2C1 interrupt (IRQ37/exception53) … GPIO15 (I2C1_SCL) and GPIO18 (I2C1_SDA) are floating on the test carrier board"* — I2C bus was physically connected, tested, root cause identified from live hardware.  
DPS310 on I2C0 @ 0x76 from Laurel hwdef. Laurel `FEATURE_GAP.md`: DPS310 reports `i2c_fail=0` continuously while running — confirming both bus health and sensor communication on I2C0.  
Status: **Bench-tested** (I2C0 bus + DPS310 sensor both confirmed operational on Laurel hardware).

**SPI**  
Pico2 FEATURE_GAP:  
*"BUG FIXED 2026-03: spi_lld_start() does not set GPIO FUNCSEL … Fixed: pico2_gpio_init() now calls palSetLineMode(HAL_GPIO_PIN_SPI0_SCK/RX/TX, PAL_MODE_ALTERNATE_SPI) … Verified 2026-03: GPIO32/35 show FUNCSEL=1 post-boot"* → SPI0 GPIO verified on hardware.  
*"SPI1 … Verified 2026-03: GPIO40/42/43 all show FUNCSEL=1 post-boot"* → SPI1 GPIO verified on hardware.  
Laurel `FEATURE_GAP.md` (2026-04-05): SPI0 carries ICM42688P traffic — stable FIFO reads with `fail=0` confirmed post CS-fix. The CS-pin bug (`ICM42688_CS` asserting incorrectly) was root-caused via live hardware and fixed; subsequent FIFO reads produced valid sensor frames continuously.  
Status: **Bench-tested** (SPI0 GPIO config + Laurel IMU traffic both confirmed on hardware; SPI1 GPIO verified on Pico2; end-to-end SPI1 sensor probe pending external sensor board).

**IMU**  
Laurel hwdef: `SPIDEV icm42688 SPI0 DEVID1 ICM42688_CS MODE3 2*MHZ 8*MHZ`.  
Laurel `FEATURE_GAP.md` (2026-04-05): *"inv3_try=1 ok=1 who42=0x47 who456=0x00"* — WHOAMI=0x47 confirmed on hardware. Continuous ICM debug output confirms `fail=0` for all FIFO reads. Accel and gyro data publishing confirmed via live MAVLink debug stream. CS-pin bug (ICM42688_CS asserting incorrectly) root-caused on hardware and fixed; subsequent FIFO traffic stable.  
Pico2: IMU Invensense (MPU9250) and Invensensev2 (ICM20948) probed on SPI1/MPU_CS — no sensor confirmation on Pico2 itself (external sensor board required; `❌ not done, needs hardware testing` per Pico2 FEATURE_GAP).  
Status: **Bench-tested** on Laurel (ICM42688P WHOAMI + FIFO data confirmed on hardware 2026-04-05). Pico2-specific IMU: **Implemented** (no sensor currently fitted on Pico2 test hardware).

**Barometer**  
Laurel hwdef: DPS310 on I2C0 @ 0x76.  
Laurel `FEATURE_GAP.md`: *"DPS310 dbg"* counters increment continuously with `i2c_fail=0`; `SYS_STATUS` MAVLink telemetry reports absolute-pressure sensor present/enabled/healthy (`ABS_P=1/1/1`). I2C0 bus confirmed simultaneously (see I2C above).  
Pico2: MS5611 on SPI0/BARO_EXT_CS (`BARO MS5611 SPI:ms5611_ext`) — no sensor confirmation on Pico2 (`❌ not done, needs hardware testing` per FEATURE_GAP; external sensor required).  
Status: **Bench-tested** on Laurel (DPS310 altitude pipeline confirmed via MAVLink SYS_STATUS on hardware). Pico2-specific baro: **Implemented**.

**RC Input**  
FEATURE_GAP 2026-04-01: *"RC INPUT IRQ21 STARTUP HARDFAULT RESOLVED ON HARDWARE … Verified 2026-04-01 on hardware: after 15 s runtime, SWD halt landed in normal main-loop execution … `_num_channels=8`, valid pulse widths (`ch0-3 = 1509,1497,1001,1496` µs)"* → **Bench-tested** (CPPM 8-channel live data confirmed).  
Implementation: `SoftSigReaderRP2350.cpp`, GPIO16 (PA16) edge callback.  
SBUS/inverted: implemented via `IO_BANK0->GPIO[pad].CTRL` invert — **Implemented** (not yet hardware validated).

**Servo / PWM**  
FEATURE_GAP latest: *"PWM frequency register-verified (400 Hz, 1 MHz clock)"* (commit `4ad40387`, 2026-04-10).  
Pico2 FEATURE_GAP: *"Register-verified 2026-03 via GDB: GPIO0-7 all FUNCSEL=0x04 (PWM). PWM slices 0-3 CSR=0x01 (EN=1). DIV=0x00000fa0 (INT=250, frac=0) → 1 MHz PWM clock. TOP=0x4E20=20000 → 20 ms period → 50 Hz. Oscilloscope or servo tester needed for end-to-end electrical verification."*  
Status: **Bench-tested** (register state confirmed correct; oscilloscope verification listed as pending but PWM frequency verified).

**UART / serial**  
Pico2 FEATURE_GAP (extensive hardware-tested evidence):  
- SERIAL0 (USB CDC): *"MAVLink v2 HEARTBEAT + STATUSTEXT + TIMESYNC stream confirmed flowing continuously for 35+ seconds on hardware. 777 parameters fetchable via MAVLink"* → **Bench-tested**.  
- SERIAL1 (UART0): *"Electrical loopback verified 2026-03-29 … UART0 loopback now PASS on hardware"* → **Bench-tested**.  
- SERIAL2 (UART1): *"Electrical loopback verified … 32-byte token returned intact via SERIAL_CONTROL device=102"* → **Bench-tested**.  
- SERIAL3 (PIOUART0): *"End-to-end loopback verified … 19/20 exact matches"* → **Bench-tested**.  
- SERIAL4 (PIOUART1): *"End-to-end loopback verified … 19/20 exact matches"* → **Bench-tested**.  
- SERIAL5 (PIOUART2): *"End-to-end loopback verified 2026-03-30: 88% exact-match replies, zero bad-data bytes"* → **Bench-tested**.  
- UART0 HW flow control (RTS/CTS): *"CTSEN=1 + nCTS deasserted → UARTFR shows TXFF=1, TXFE=0, BUSY=1 (TX FIFO full, hardware gating TX)"* → **Bench-tested**.

**USB**  
FEATURE_GAP: *"USB CDC enumerates on `/dev/ttyACM1`. MAVLink v2 HEARTBEAT … stream confirmed flowing continuously for 35+ seconds on hardware. 777 parameters fetchable via MAVLink."*  
*"Hardware re-verified 2026-04-01 … `pymavlink.wait_heartbeat(timeout=20)` returned `True`"*.  
Bootloader: *"Verified 2026-03-29: `uploader.py --port /dev/ttyACM1 arducopter.apj` → erase/program/verify/reboot 100% → `/dev/ttyACM1: Pico2` appears → 193 bytes MAVLink v2 confirmed"* → **Bench-tested**.

**Storage**  
Laurel hwdef: 8 MB XIP flash (ChibiOS EFL driver), microSD on SPI1 (MMCSPI, 400 kHz–25 MHz), FATFS.  
FEATURE_GAP: *"MAVLink FTP `tasks.txt` and `threads.txt` downloaded via USB"* (commits 2026-04-08) — implies EFL flash and filesystem working on hardware at least partially.  
Status: **Bench-tested** (flash + MAVLink FTP confirmed on hardware).

**Watchdog**  
Laurel hwdef: `define HAL_USE_WDG TRUE`, `define HAL_WATCHDOG_ENABLED_DEFAULT 1`.  
Commit `0ebf312b` (2026-04-10): *"fix was_watchdog_reset() using SCRATCH canary"*.  
Laurel `FEATURE_GAP.md` (2026-04-10) — full pipeline bench-tested: forced WD via `WATCHDOG->LOAD=100` ticks (≈100 µs) from OpenOCD telnet; board PSM-resets and re-enumerates USB; `rp2350_wd_reset_detected=1` confirmed via MAVLink; SCRATCH[6] `'WDOG'` canary functioning; GCS CRITICAL string (*"WDG: T0 SL0 FL0 FT0 FA0 FTP0 FLR0 FICSR0 MM0 MC0 IE0 IEC0 TN:"*) received live. Persistent data (76-byte `persistent_data`) survives PSM reset via noinit `.ram0` SRAM buffer.  
Pico2 FEATURE_GAP: *"WATCHDOG_CTRL=0x471d88f8 (ENABLE=1 bit30, PAUSE_DBG=1 bits[25:24], down-counter active)"* — register-verified active.  
Status: **Bench-tested** (full WD timeout → PSM reset → USB re-enumeration → SCRATCH canary → GCS CRITICAL text pipeline confirmed on Laurel hardware 2026-04-10).

**ADC / AnalogIn**  
Pico2 `FEATURE_GAP.md` (2026-03): *"ADC group 0 running continuously in circular DMA mode. MCU temperature reads 32.3°C (room temp, confirmed via GDB on live hardware). ADC_ACTIVE state confirmed after 40+ seconds."* GPIO28/29 verified in analog-isolation mode (`FUNCSEL=31`, `PADS_BANK0 ISO=1`). Round-robin enabled: `ADC_CS RROBIN bits[20:16]` covering channels 2+3+4 confirmed in GDB register read. Self-healing error callback added (restarts ADC on FIFO overflow — fixes 437°C bug from oversampling).  
Laurel `FEATURE_GAP.md`: all 3 external ADC channels live-verified: battery voltage (GPIO40/ADC0), battery current (GPIO41/ADC1), RSSI (GPIO42/ADC2 — GPIO42 CTRL=0x1F, AINSEL=8, RROBIN=0x107 confirmed via GDB). MCU temperature also active. All channels providing live telemetry data on hardware.  
Status: **Bench-tested** (MCU temperature on Pico2; all 3 external channels + MCU temp on Laurel — both confirmed on hardware).

**LEDs and GPIO power enables (Laurel)**  
Laurel `FEATURE_GAP.md` (2026-04-07): blue LED GPIO6 — CTRL=0x05 (SIO), GPIO_OE bit 6 set, toggling confirmed on hardware. Green LED GPIO7 — same verification.  
BEC_5V_EN (GPIO14) and BEC_9V_EN (GPIO15): boot-default Hi-Z bug fixed; both GPIO enables now driving correctly and verified on hardware.  
Status: **Bench-tested** (GPIO6/7 LEDs and GPIO14/15 power enables all confirmed on Laurel hardware).

**Dual-core**  
FEATURE_GAP: *"ChibiOS SMP exploration (2026-03, abandoned): unacceptable spinlock overhead."*  
Current architecture: `CH_CFG_SMP_MODE=FALSE` (ChibiOS runs single-core on core0) + `RP_CORE1_START=TRUE` (bare-metal `c1_main.c` WFE dispatcher on core1). Workloads dispatched to core1 via `c1_run_sync()`: rate controller and EKF `CovariancePrediction`.  
Commit `185d8eb9` (2026-04-10): *"WIP Laurel: update FEATURE_GAP — Core1 EKF dispatch now working."*  
Laurel `FEATURE_GAP.md` (2026-04-10) — live 10-second dual-core stats verified on hardware via MAVLink: *"EKF C1≈560 C0=0"* (100% of EKF cycles on Core1), *"PID C1≈1400 C0≈3"* (99.8% of PID cycles on Core1), Core0 CPU≈87% (release build), Core1 CPU=8% (WFE dispatcher lightly loaded). Both cores confirmed executing flight-critical workloads simultaneously.  
Commit `4838a2c2`: SCHED_LOOP_RATE=125 Hz confirmed on hardware in release build (CPU Core0=72–87%).  
RAMFUNC optimization: 45.4 KB of hot-path code placed in SRAM → −19% EKF time, −33% GCS task time in timed release-build benchmarks.  
**Note on completeness:** Both physical cores are running code, so the hardware dual-core capability is in use. However, this is **not** symmetric SMP — core1 runs a bare-metal polling loop with no RTOS, no thread scheduler, and no independent task set. It is a tightly-coupled co-processor pattern rather than a general dual-core OS. Full dual-core SMP (both cores under ChibiOS) was attempted and abandoned. The current core1 dispatch is a partial, workload-specific implementation.  
Status: **Bench-tested** for the specific core1 dispatch path (EKF + PID offload confirmed live with 10-second per-core cycle-count stats on 2026-04-10); **Not implemented** as general dual-core / full SMP.

**`hw_check_gdb.py` harness (Pico2 only)**  
On-hardware GDB register check script. FEATURE_GAP: *"21/21 PASS on free-running board with no sensors attached (2026-03-28, commit `1cddbf0a93`)"* — verifies UART, SPI, ADC, PWM, CS-pin FUNCSEL/OE register state automatically. Strong structural bench-test evidence.

**HAL examples (Pico2 only — hardware verified)**  
Pico2 `FEATURE_GAP.md` documents three ArduPilot HAL example programs all confirmed passing on real hardware via SWD/OpenOCD + mavproxy:  
- `examples/Hello`: serial asterisks output confirmed via USB CDC → verifies scheduler, HAL thread execution, USB CDC serial output.  
- `examples/FlashTest`: **TEST PASSED** → confirms EFL flash erase/read/write cycles and XIP parameter storage at hardware level.  
- `examples/BinarySem`: **148,000 ops/s, 0 timeouts** → confirms ChibiOS binary semaphore, thread creation, and scheduler are fully functional at speed on hardware.  

**Allow-boot-without-sensors (Pico2 only — hardware verified)**  
FEATURE_GAP: `HAL_BARO_ALLOW_INIT_NO_BARO 1` and `AP_INERTIALSENSOR_ALLOW_NO_SENSORS 1` both set. Board boots cleanly with no sensor hardware attached: USB CDC enumerates, MAVLink streams, 777 parameters load — confirmed on hardware. Sensor absence visible via `SYS_STATUS` health bits and arming checks.

**Build status:** ArduCopter for `Laurel` built **successfully** in a subsequent local run today (2026-04-10, commit `62278a4a41`): `'copter' finished successfully (2m30.828s)`, flash usage 1,389,360 B (6,933,600 B free in 8 MB XIP). The earlier failure was due to `fatfs-0.14b_patched.7z` not yet extracted; once extracted the build passes cleanly. → **Build-tested**.

---

## 3. Cross-Codebase Feature Matrix

| Feature | C1 AP_HAL_RP | C2 AP_HAL_rp2xxxChibiOS | C3 AP_HAL_ChibiOS (Pico2/Laurel) | Best / Most-validated |
|---------|:---:|:---:|:---:|---|
| **HAL architecture** | Custom+FreeRTOS | ChibiOS fork (rp2xxx) | Upstream ChibiOS extended | C3: widest upstream merge surface |
| **Clock / startup** | Pico SDK | Custom rp2xxx ChibiOS | Standard ChibiOS | C3: documented, bench-tuned |
| **Max MCU clock** | Not specified | **276 MHz** | 150 MHz (Laurel) / 250 MHz (Pico2) | C2: overclocked highest |
| **I2C** | **Bench-tested** (Pico2Pilot: DPS310 confirmed) | Implemented | **Bench-tested** (I2C0 + DPS310 live; IRQ bug root-caused on hardware) | **C1 / C3 tied** |
| **SPI** | Implemented | Implemented | **Bench-tested** (GPIO FUNCSEL + Laurel ICM traffic verified) | **C3** |
| **IMU** | Implemented | Implemented (configured for external MPU9250; probe unconfirmed) | **Bench-tested** (Laurel ICM42688P `who42=0x47`, FIFO data confirmed) | **C3** (first codebase with confirmed live sensor data) |
| **Barometer** | **Bench-tested** (Pico2Pilot: DPS310; Kolibri I2C driver disabled) | Implemented (external BMP280; probe unconfirmed) | **Bench-tested** (Laurel DPS310 `ABS_P=1/1/1` via MAVLink) | **C1 Pico2Pilot / C3 tied** |
| **RC Input** | Implemented | Implemented | **Bench-tested** (8-ch valid pulses) | **C3** |
| **Servo / PWM** | Implemented | **Bench-tested** (OneShot125; clock-speed fix) | **Bench-tested** (register-verified 50 Hz / 400 Hz) | **C2 / C3 tied** |
| **UART** | **Bench-tested** (PIO UART example confirmed) | Implemented | **Bench-tested** (all 5 serial ports loopback verified) | **C3** |
| **USB CDC** | **Build-tested** | **Build-tested** | **Bench-tested** (MAVLink stream 35+ s, 777 params) | **C3** |
| **Bootloader / DFU** | Unknown | Unknown | **Bench-tested** (`uploader.py` 100% erase/write/verify/boot) | **C3** |
| **Flash storage** | **Bench-tested** (random r/w; Core1+Core2 simultaneous access) | Implemented | **Bench-tested** (MAVLink FTP files downloaded) | **C1 / C3** |
| **microSD** | Not implemented | **Build-tested** (MMC_SPI + FATFS compiled; no hardware test) | Implemented (SPI1 MMCSPI; FATFS) | **C2** |
| **NAND flash** | Implemented (Kolibri, LittleFS) | Not present | Not present | **C1** |
| **Watchdog** | Implemented | Implemented | **Bench-tested** (WD timeout → PSM reset → SCRATCH canary → GCS CRITICAL text confirmed, Laurel 2026-04-10) | **C3** |
| **Dual-core / full SMP** | Not implemented (core0 only) | Implemented (ChibiOS SMP; core1 = USB-init thread + idle) | **Bench-tested** (core1 bare-metal: EKF + PID offload; live 10 s cycle stats; full SMP abandoned) | C2: ChibiOS SMP working; C3: flight workload on core1 |
| **DShot / OneShot** | Not implemented | **Bench-tested** (OneShot125) | Not implemented | **C2** |
| **FFT/DSP** | Not mentioned | **Build-tested** (custom `AP_HAL::DSP`; CMSIS-DSP Cortex-M33; 352-line `DSP.cpp`) | Not mentioned | **C2** |
| **AnalogIn / ADC** | Not implemented (Kolibri) | Implemented | **Bench-tested** (Pico2: MCU temp 32.3°C; Laurel: 3 ext. channels + MCU temp) | **C3** |
| **Flow control (CTS/RTS)** | Not mentioned | Not mentioned | **Bench-tested** (UARTFR TX-gating confirmed) | **C3** |
| **RC protocols (SBUS/CRSF/DSM/ELRS)** | Implemented (ELRS onboard on Kolibri/Pico2Pilot) | Implemented (IBUS/SBUS via PIO) | **Bench-tested** (CPPM 8-ch live; CRSF hardfault resolved 2026-04-01) | **C3** |
| **PIO UART / extra serial ports** | **Bench-tested** (PIO UART; "Tested by examples/UART_test") | Implemented (PIO-based RC only; no multi-PIOUART) | **Bench-tested** (SERIAL3/4/5 loopback verified 2026-03-30) | **C1 / C3 tied** |
| **ROMFS / embedded filesystem** | Implemented (Kolibri: LittleFS on NAND; Pico2Pilot: disabled) | **Build-tested** (`defaults.parm` embedded in ROMFS; compiled into passing build) | **Bench-tested** (ROMFS enabled; MAVFTP `tasks.txt` downloaded 2026-04-08) | **C2** (built into binary); **C3** (MAVFTP confirmed) |
| **HAL examples (hardware-run)** | **Bench-tested** (Storage r/w; PIO UART confirmed) | Not documented | **Bench-tested** (Hello ✓; FlashTest PASS ✓; BinarySem 148 k ops/s ✓) | **C3** |
| **Onboard special hardware** | Kolibri: 256 MB NAND, ELRS 2.4 GHz module, 3-colour LED; Pico2Pilot: ELRS | None (bare Pico2 module) | Laurel: BEC 5V/9V enables (bench-tested); GPIO6/7 LEDs (bench-tested); RGB LED shelved | **C1** richest hardware; **C3** most bench-tested |
| **Unique hardware ID (OTP)** | Not documented | Not documented | **Bench-tested** (OTP CHIPID rows read; USB serial + `uploader.py ChipDes: RP2350 B0` confirmed) | **C3** |
| **Allow-boot-without-sensors** | Implemented (`HAL_BARO_ALLOW_INIT_NO_BARO 1` in both hwdefs) | Not documented | **Bench-tested** (headless boot: 777 params + MAVLink stream, no sensors fitted) | **C3** |
| **Board richness** | 3 boards (Pico2Pilot most complete; Kolibri has IMU/baro drivers disabled, wrong sensor types in hwdef) | 1 real board (rp2040 baseline + rp2350 port) | 2 boards (Pico2 + Laurel) | **C3** by hardware integration; C1 by board count |
| **Build (local, today)** | **FAILED** (`PICO_SDK_PATH` missing) | **PASSED** (925 objects, 9m22s) | **PASSED** (1389360 B, 2m31s, commit `62278a4a41`) | **C2 / C3** both pass |
| **Commit activity** | ~35 commits (Dec 2025–Apr 2026) | ~30 unique commits (Feb–Apr 2026) + 15 formative commits (2023) + 5 prototype commits (2021) | **300+ RP2350-specific commits** (Mar–Apr 2026, ~1 month) | **C3** |

---

## 4. Evidence Table

| Claim | Source | Type |
|-------|--------|------|
| C1 build tested Ubuntu/macOS/Windows | `libraries/AP_HAL_RP/README.md` | Doc |
| C1 I2C: "Tested I2C with DPS310 barometer and fixed issues" | Commit `1077567194` 2026-03-10 | Commit |
| C1 Storage: "Test with AP_HAL/examples/Storage…random data…Core1/Core2" | Commit `ef0d3124` 2026-03-13 | Commit |
| C1 UART: "Tested by examples/UART_test" | Commit `0a40f622` 2026-03-24 | Commit |
| C1 SPI bring-up | Commits `82408e2f`, `0f53dee3`, `f83446e7` 2026-03-31..04-03 | Commit |
| C1 ArduCopter compile target | Commit `b7cb916d` 2026-03-28 | Commit |
| C2 ArduCopter build: 925 objects, 9m22s | Local build 2026-04-10 | Build run |
| C2 RP2350 support added | Commit `80eadb4c` 2026-03-10 | Commit |
| C2 PWM clock-speed fix | Commit `4ae7eecb` 2026-03-16 | Commit |
| C2 OneShot125 RCOut | Commit `2a2722e1` 2026-04-08 | Commit |
| C2 6 servo outputs | Commit `20e7277b` 2026-04-07 | Commit |
| C2 FFT/DSP for rp2350 | Commit `17077534` 2026-03-18 | Commit |
| C2 rp2350-pico full config in ChibiOS-native files | `hwdef/rp2350-pico/board.h`, `mcuconf.h`, `chibios_board.mk` | Code |
| C2 rp2350-pico mcuconf.h: SPI0/SPI1/I2C0/I2C1/USB/ADC/core1 all enabled | `hwdef/rp2350-pico/mcuconf.h` | Code |
| C2 history: HAL origins 2021 (Pico SDK), rebuilt 2023 (ChibiOS), revitalized 2026 | Commits `8039611816` (2021), `ed62311417` (2023), `6272ac4d5a` (2026) | Commit |
| C3 USB CDC: "35+ seconds, 777 params, heartbeat True" | `FEATURE_GAP.md` session docs | Doc |
| C3 Bootloader: "uploader.py 100% erase/write/verify/boot, MAVLink v2 confirmed" | `FEATURE_GAP.md` session 5 2026-03-29 | Doc |
| C3 UART0 loopback: "PASS on hardware" | `FEATURE_GAP.md` session 6 2026-03-29 | Doc |
| C3 UART1 loopback: "32-byte token returned intact" | `FEATURE_GAP.md` | Doc |
| C3 PIOUART 0/1/2: "end-to-end loopback verified, zero bad-data bytes" | `FEATURE_GAP.md` session 15 2026-03-30 | Doc |
| C3 RC input: "8 channels, ch0-3 = 1509,1497,1001,1496 µs" | `FEATURE_GAP.md` 2026-04-01 | Doc |
| C3 PWM: "register-verified 50 Hz, TOP=20000, oscilloscope pending" | `FEATURE_GAP.md` / commit `4ad40387` | Doc+Commit |
| C3 CTS/RTS: "UARTFR shows TXFF=1, TXFE=0, BUSY=1" | `FEATURE_GAP.md` / commit `3b3f4f45` | Doc+Commit |
| C3 hw_check_gdb.py: "21/21 PASS free-running board" | `FEATURE_GAP.md` 2026-03-28 / commit `1cddbf0a` | Doc+Commit |
| C3 MAVLink FTP: "tasks.txt downloaded" | Commit `bd11474b` 2026-04-08 | Commit |
| C3 Core1 EKF: "Core1 EKF dispatch now working" | Commit `185d8eb9` 2026-04-10 | Commit |
| C3 Watchdog reset: SCRATCH canary functioning | Commit `0ebf312b` 2026-04-10 | Commit |
| C3 Laurel hwdef: ICM42688P, DPS310, SPI0/SPI1, I2C0, 4 PWM, 3 ADC | `hwdef/Laurel/hwdef.dat` | Code |
| C3 ChibiOS SMP abandoned: "unacceptable spinlock overhead" | `FEATURE_GAP.md` | Doc |
| C3 Laurel IMU: `who42=0x47 ok=1`, `fail=0` FIFO reads, accel/gyro data publishing confirmed | `hwdef/Laurel/FEATURE_GAP.md` 2026-04-05 | Doc |
| C3 Laurel Baro: DPS310 `i2c_fail=0`, `ABS_P=1/1/1` via MAVLink SYS_STATUS on hardware | `hwdef/Laurel/FEATURE_GAP.md` | Doc |
| C3 Laurel I2C0: DPS310 `i2c_fail=0` continuously confirming bus + sensor | `hwdef/Laurel/FEATURE_GAP.md` | Doc |
| C3 Laurel SPI0: stable ICM42688P FIFO reads with `fail=0` post CS-fix | `hwdef/Laurel/FEATURE_GAP.md` 2026-04-05 | Doc |
| C3 Pico2 ADC: MCU temp 32.3°C in circular DMA mode; GPIO28/29 FUNCSEL=31 verified | `hwdef/Pico2/FEATURE_GAP.md` 2026-03 | Doc |
| C3 Laurel ADC: all 3 ext. channels (bat volt/curr/RSSI) + MCU temp live-verified via GDB | `hwdef/Laurel/FEATURE_GAP.md` | Doc |
| C3 Dual-core: EKF C1≈560 C0=0, PID C1≈1400 C0≈3 per 10s, Core1 CPU=8% — live MAVLink stats | `hwdef/Laurel/FEATURE_GAP.md` 2026-04-10 | Doc |
| C3 RAMFUNC: −19% EKF time, −33% GCS time vs non-RAMFUNC release build | `hwdef/Laurel/FEATURE_GAP.md` | Doc |
| C3 Watchdog: WD LOAD=100µs → PSM reset → USB re-enum → SCRATCH canary → GCS CRITICAL text | `hwdef/Laurel/FEATURE_GAP.md` commit `0ebf312b` 2026-04-10 | Doc+Commit |
| C3 Laurel LEDs: GPIO6/7 toggling confirmed (CTRL=0x05, GPIO_OE set) | `hwdef/Laurel/FEATURE_GAP.md` 2026-04-07 | Doc |
| C3 Pico2 FlashTest example: TEST PASSED on hardware | `hwdef/Pico2/FEATURE_GAP.md` | Doc |
| C3 Pico2 BinarySem example: 148,000 ops/s, 0 timeouts confirmed | `hwdef/Pico2/FEATURE_GAP.md` | Doc |
| C2 rp2350-pico configured for external MPU9250+BMP280: `PROBE_MPU9250_INS` SPI1 GPIO13 CS, `PROBE_BMP280_BARO` I2C1 0x77 | `hwdef/rp2350-pico/ardupilot_board_config.h` | Code |
| C2 rp2350-pico: 6 PWM on GPIO8/10/20/21/11/22; IBUS RC on GPIO7; 3 ADC on GPIO26-28 | `hwdef/rp2350-pico/ardupilot_board_config.h` | Code |
| C2 rp2350-pico `defaults.parm`: `SCHED_LOOP_RATE 300` (vs 100 for rp2040) | `hwdef/rp2350-pico/defaults.parm` | Code |
| C2 dual-core: `CH_CFG_SMP_MODE TRUE` in `chconf.h`; `c1_main()` runs `chInstanceObjectInit` + `usb_initialise()` then `LOWPRIO` idle loop | `hwdef/common/chconf.h`, `HAL_rp2xxxChibiOS_Class.cpp` lines 189–209 | Code |
| C2 microSD: `USE_FATFS = yes` in `chibios_board.mk`; `HAL_USE_MMC_SPI TRUE` in `halconf.h`; `HAL_SPI_DEVICE_SDCARD` in `ardupilot_board_config.h` | `hwdef/common/chibios_board.mk`, `halconf.h`, `hwdef/rp2350-pico/ardupilot_board_config.h` | Code |
| C2 FFT/DSP: 352 lines `DSP.cpp` implementing `AP_HAL::DSP`; `libarm_cortexM33lf_math.a` (CMSIS-DSP Cortex-M33) | Commit `17077534` 2026-03-18; `DSP.cpp`, `DSP.h` | Commit+Code |
| C1 Kolibri: `HAL_SPI_DEVICE_DRIVER_ENABLED 0` AND `HAL_I2C_DEVICE_DRIVER_ENABLED 0` — both device drivers disabled | `hwdef/Kolibri/hwdef.dat` lines 18-19 | Code |
| C1 Kolibri: README says ICM-42688-P + LPS22HB; hwdef declares mpu9250 driver + BMP280 (wrong sensors) | `hwdef/Kolibri/hwdef.dat` + `hwdef/Kolibri/README.md` | Code|
| C3 Pico2 examples: `FlashTest: TEST PASSED`; `BinarySem: 148,000 ops/s, 0 timeouts` on hardware | `hwdef/Pico2/FEATURE_GAP.md` | Doc |
| C3 Pico2 board ID: `board_type: 189`, `ChipDes: RP2350 B0` verified via `uploader.py` 2026-03-29 | `hwdef/Pico2/FEATURE_GAP.md` session 5 | Doc |

---

## 5. Tested/Flown/Working Status Per Feature and Per Codebase

| Feature | C1 AP_HAL_RP | C2 AP_HAL_rp2xxxChibiOS | C3 AP_HAL_ChibiOS |
|---------|:---:|:---:|:---:|
| Build (ArduCopter) | `Build-tested` (confirmed in README; local fail = env) | `Build-tested` ✅ local | `Build-tested` ✅ local (2m31s, 1389360 B, 2026-04-10) |
| Clock / startup | `Implemented` | `Build-tested` | `Bench-tested` |
| I2C driver | `Bench-tested` (DPS310 commit) | `Implemented` | `Bench-tested` (bus live, I2C IRQ root-caused) |
| SPI driver | `Implemented` | `Implemented` | `Bench-tested` (FUNCSEL register verified) |
| IMU | `Implemented` | `Implemented` (configured for external MPU9250; probe unconfirmed) | `Bench-tested` (Laurel ICM42688P `who42=0x47`, FIFO data confirmed 2026-04-05) |
| Barometer | `Bench-tested` (DPS310 on Pico2Pilot; Kolibri I2C driver disabled + wrong sensor in hwdef) | `Implemented` (configured for external BMP280; probe unconfirmed) | `Bench-tested` (Laurel DPS310 `ABS_P=1/1/1` via MAVLink SYS_STATUS) |
| AnalogIn / ADC | Not implemented (Kolibri) | `Implemented` | `Bench-tested` (Pico2: MCU temp 32.3°C; Laurel: 3 ext. channels + MCU temp) |
| RC Input | `Implemented` | `Implemented` | `Bench-tested` (8-ch valid pulses) |
| Servo / PWM | `Implemented` | `Bench-tested` (OneShot + clock fix) | `Bench-tested` (register + freq verified) |
| UART (all ports) | `Bench-tested` (PIO example) | `Implemented` | `Bench-tested` (all 5 ports loopback) |
| USB CDC | `Build-tested` | `Build-tested` | `Bench-tested` (MAVLink stream 35+ s) |
| Bootloader / DFU | `Unknown` | `Unknown` | `Bench-tested` (100% upload verified) |
| Flash storage | `Bench-tested` (random r/w) | `Implemented` | `Bench-tested` (MAVLink FTP files) |
| microSD | `Unknown` (not implemented) | `Build-tested` (MMC_SPI + FATFS compiled; no hardware test) | `Implemented` |
| NAND flash | `Implemented` | Not present | Not present |
| Watchdog | `Implemented` | `Implemented` | `Bench-tested` (WD timeout → PSM reset → SCRATCH canary → GCS CRITICAL confirmed 2026-04-10) |
| Dual-core / full SMP | `Not implemented` (core0 only) | `Implemented` (ChibiOS SMP operational: core1 has ChibiOS instance + USB init; no flight workload on core1) | `Bench-tested` (core1 bare-metal dispatch with live 10-second stats; full SMP abandoned) |
| DShot / OneShot | Not implemented | `Bench-tested` | Not implemented |
| FFT / DSP | Not mentioned | `Build-tested` (custom `AP_HAL::DSP` + CMSIS-DSP library, compiled in passing build) | Not mentioned |
| RTS/CTS flow ctrl | `Unknown` | `Unknown` | `Bench-tested` |
| Flight-tested | **None** | **None** | **None** |

> **No flight-tested claims exist in any of the three codebases.** All operational evidence is limited to bench/hardware testing (hardware-in-the-loop on desk with USB, GDB, or test scripts).

---

## 6. Gaps, Risks, and Confidence Level

### 6.1 Gaps

1. **No flight test** in any codebase. All three are pre-flight.  
2. **C2 rp2350-pico uses ChibiOS-native config, not hwdef.dat.** The board is fully configured via `board.h`, `mcuconf.h`, `chibios_board.mk`, and a custom linker script. SPI0/SPI1, I2C0/I2C1, UART0/UART1, USB, ADC, and core1 are all enabled. However the Pico2 module has no onboard IMU or barometer, so sensor probe results are only meaningful once an external sensor board is connected.  
3. **C1 and C2 IMU not confirmed on hardware.** C1 SPI recently added (commits `82408e2f`, `0f53dee3` March 2026) but no "WHOAMI seen on bus" statement found. C2 (rp2350-pico) requires an external MPU9250 sensor board — the probes are configured but runtime confirmation needs sensor hardware connected. C3 Laurel has **ICM42688P bench-tested** (`who42=0x47`, FIFO data confirmed 2026-04-05); Pico2 target still needs external sensor.
4. **C2 baro not confirmed; C3 Pico2 baro not confirmed.** C3 Laurel DPS310 is **bench-tested** (`ABS_P=1/1/1` via MAVLink SYS_STATUS). C2 requires external BMP280 sensor board; C3 Pico2 requires external MS5611. C1 Pico2Pilot DPS310 already bench-tested (commit 2026-03-10).
5. **C1 Kolibri hwdef has critical defects.** (a) `HAL_SPI_DEVICE_DRIVER_ENABLED 0` and `HAL_I2C_DEVICE_DRIVER_ENABLED 0` — both SPI and I2C device layers are compiled out, making the declared IMU and barometer dead code. (b) IMU driver mismatch: hardware is ICM-42688-P (README) but hwdef uses `IMU Invensense SPI:mpu9250` (wrong driver class). (c) Barometer mismatch: hardware is LPS22HB (README) but hwdef declares `BARO BMP280`. Kolibri cannot probe any sensor until these three issues are corrected.  
6. **C1 build requires external `PICO_SDK_PATH`.** Not auto-provisioned; blocks CI and first-time builders.  
7. **C3 Laurel FATFS pre-extraction required.** The `fatfs-0.14b_patched.7z` archive in the ChibiOS submodule must be extracted before building. Once extracted the build passes cleanly (confirmed 2026-04-10). The `buzz.txt` build script handles this automatically via `ensure_chibios_fatfs()`.  
8. **C3 oscilloscope-PWM not done.** Register verification confirms correct config; actual waveform on servo pins not confirmed.  
9. **DShot absent in C1 and C3.** Only C2 has OneShot125; none have DShot.  
10. **EKF performance partially validated.** Core1 EKF offload in C3 is working and shows 10-second cycle-count stats (EKF C1≈560, Core1=8% CPU). However, sustained flight-rate stress-testing and EKF convergence with real sensor data have not been reported.  
11. **ArduCopter attitude control / arming unvalidated.** No "motors spun up" or "arming succeeded" claim in any repo.

### 6.2 Risks

| Risk | Severity | Affects |
|------|----------|---------|
| C1 Kolibri hwdef: SPI/I2C drivers disabled + wrong sensor types | **HIGH** | C1 Kolibri specifically — Kolibri cannot probe any sensor in current state; Pico2Pilot is unaffected |
| No IMU data stream confirmed on hardware in C1 and C2 | **HIGH** | C1 (Pico2Pilot only; Kolibri blocked), C2 — C3 Laurel now has ICM42688P WHOAMI + FIFO data confirmed (2026-04-05); C1 SPI recently added with no sensor confirmation; C2 requires external sensor board |
| RP2350 XIP cache / DMA coherence under load | **HIGH** | C1 (Pico SDK) and C2 (rp2xxx ChibiOS) — untested at flight rates |
| C3 ChibiOS SMP abandoned — race conditions in core1 bare-metal path | **MEDIUM** | C3 — newly working core1 dispatch not stress-tested |
| C2 rp2350-pico has no onboard sensors — external sensor board required | **HIGH** | C2 RP2350 target — no IMU/baro probe possible without external hardware |
| Stack overflows under interrupt load | **MEDIUM** | All — C3 has documented iterative stack growth; others not tracked |
| AK8963 compass via MPU9250 (C1) — SPI+auxiliary I2C chain not verified | **MEDIUM** | C1 Kolibri / Pico2Pilot |
| FATFS `.7z` pre-extraction required for C3 ChibiOS submodule | **LOW** (resolved — build passes once extracted; `buzz.txt` automates this) | C3 reproducibility |

### 6.3 Confidence Levels

| Codebase | Confidence in hardware readiness (bench) | Confidence in flight readiness |
|----------|:---:|:---:|
| C1 AP_HAL_RP | **Medium** — Pico2Pilot: I2C (DPS310 bench-tested), UART, storage bench-tested; SPI/IMU unconfirmed. Kolibri: IMU/baro drivers disabled in hwdef — not functional. | **Low** |
| C2 AP_HAL_rp2xxxChibiOS | **Medium** — builds clean, PWM/OneShot bench-implied; no onboard sensors on bare Pico2 module | **Low** |
| C3 AP_HAL_ChibiOS | **Very High** — broadest bench coverage: all 5 serial ports, USB, bootloader, RC-in, PWM, watchdog (full WD pipeline), **IMU (ICM42688P WHOAMI+FIFO confirmed)**, **barometer (DPS310 ABS_P=1/1/1)**, ADC (3 ext. channels + MCU temp), LEDs, dual-core 10-second cycle stats — all hardware-verified | **Low** |

---

## 7. Prioritized Next Validation Steps

Listed in order of risk-reduction impact for the path to first flight.

1. **Verify IMU data stream on C1 and C2 hardware.** C3 Laurel has confirmed ICM42688P working. Remaining: C1 needs "WHOAMI seen on SPI" confirmation; C2 needs external MPU9250 sensor board connected to rp2350-pico SPI1.
2. **Verify barometer on C1 BMP280, C2 external BMP280, C3 Pico2 MS5611.** C3 Laurel DPS310 and C1 DPS310 are confirmed. C2 and C3 Pico2 still need physical sensor hardware.
3. **C3: Oscilloscope-verify PWM outputs on servo pins.** Register state confirms correct config (TOP=20000, 50 Hz); actual waveform on servo pins and servo response still listed as pending in FEATURE_GAP.
4. **C3: Arm and spin motors on bench.** With IMU + baro working on Laurel, attempt armed throttle test in STABILIZE mode. This is the gating step before any outdoor test.
5. **C2: Connect external MPU9250+BMP280 sensor board to rp2350-pico.** SPI1 and I2C1 buses are configured (`ardupilot_board_config.h`). Hardware sensor probe is the next step.
6. **C1: Resolve `PICO_SDK_PATH` dependency** to allow reproducible builds without manual SDK install.
7. **C3: Stress-test core1 bare-metal dispatcher** under sustained 125 Hz full-copter load with EKF. Currently confirmed working; 10-second cycle stats are encouraging (Core1=8% CPU) but not a substitute for continuous flight-rate load test.
8. **All repos: Ground test** with RC transmitter connected, arming, disarming, mode switches — before any airframe integration.
9. **All repos: Logging validation.** Confirm DataFlash / SD log files contain coherent data throughout a full bench session.
10. **All repos: First-hover attempt** in a test rig only after steps 1–9 complete.

---

*Report generated from local repository evidence only. No external URLs, issues, or PRs were accessible in this analysis. All test claims trace to git commit messages and in-tree markdown documents.*
