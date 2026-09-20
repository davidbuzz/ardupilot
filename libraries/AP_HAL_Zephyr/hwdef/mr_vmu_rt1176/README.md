# MR-VMU-RT1176 Flight Controller (Zephyr)

The MR-VMU-RT1176 is NXP's open-source reference design implementing the
Pixhawk FMUv6X-RT standard: an FMU module on an NXP carrier board. This is the
primary target of the ArduPilot-on-Zephyr port.

- waf board: `mr_vmu_rt1176`
- Zephyr board target: `mr_vmu_rt1176/mimxrt1176/cm7`
- HAL board: `HAL_BOARD_ZEPHYR`, HAL class `HAL_Zephyr`, namespace `Zephyr::`
- `APJ_BOARD_ID` `AP_HW_MR_VMU_RT1176` = 1253 (`Tools/AP_Bootloader/board_types.txt`; the PX4 `px4_fmu-v6xrt` bootloader the board ships with reports 35, so flash AP_Bootloader first)

## Features

- NXP MIMXRT1176 (Cortex-M7 at 1 GHz, plus an unused Cortex-M4)
- 64 MB external FlexSPI Octal-DDR NOR (MX25UM51345G), XIP, no internal program
  flash
- Three SPI IMU slots: one onboard, two on a shock-mounted daughterboard over
  FPC. Fitted parts vary by board revision, so the hwdef probes rather than
  assumes
- Two BMP388-family barometers (onboard on I2C1, offboard on I2C2)
- BMM150 compass on the offboard daughterboard
- 9 serial ports including USB CDC, 3 I2C buses, 3 SPI buses, 2 CAN buses
- 12 FMU-direct PWM outputs, 8 of them DShot-capable pads. No IOMCU is fitted
- Dual-path RC input: single-wire SBUS/CRSF on LPUART6 plus PPM-SUM pulse
  capture
- microSD slot on USDHC1, FatFs logging
- Two SMBus smart-battery power inputs (POWER1, POWER2). There is no analog
  battery sense on this board
- Two USB CDC-ACM interfaces: MAVLink and mcumgr SMP
- 100Base-T1 Ethernet, not implemented

There is no safety switch (`HAL_HAVE_SAFETY_SWITCH 0`).

## Documentation in the wiki

Two parts of this board's documentation now live in the ArduPilot wiki:

- [Loading firmware onto Zephyr boards](https://ardupilot.org/ardupilot/docs/common-loading-firmware-onto-zephyr-boards.html) — building the firmware, loading it over USB or SWD, and replacing the bootloader.
- [MR-VMU-RT1176 bring-up reference](https://ardupilot.org/dev/docs/zephyr-rt1176-bringup.html) — processor and memory layout, where the board's configuration actually lives, current status, and the bring-up checklist behind each subsystem.

## UART mapping

`SERIAL_ORDER` in `hwdef.dat` is
`OTG1 USART4 USART8 USART3 USART5 USART10 USART11 USART6 LPUART1`, generating
`HAL_UART_DT_DEVICE_LOOKUP` with `HAL_UART_NUM_SERIAL_PORTS 9`:

| ArduPilot | Device         | Function                                                        |
| --------- | -------------- | --------------------------------------------------------------- |
| SERIAL0   | `usb_cdc_acm0` | USB CDC ACM, MAVLink (GCS)                                      |
| SERIAL1   | `lpuart4`      | TELEM1                                                          |
| SERIAL2   | `lpuart8`      | TELEM2                                                          |
| SERIAL3   | `lpuart3`      | GPS1 (`DEFAULT_SERIAL3_PROTOCOL 5`)                             |
| SERIAL4   | `lpuart5`      | GPS2                                                            |
| SERIAL5   | `lpuart10`     | TELEM3                                                          |
| SERIAL6   | `lpuart11`     | External, DTS node not enabled                                  |
| SERIAL7   | `lpuart6`      | RC input, single-wire (`RCInput.cpp` hardcodes `hal.serial(7)`) |
| SERIAL8   | `lpuart1`      | Debug connector, also `zephyr,console` and `zephyr,shell-uart`  |

`lpuart11` has no enabled node and no pinctrl group in the board DTS, so SERIAL6
resolves to nothing at runtime even though the slot exists.

Every enabled LPUART carries `dmas`/`dma-names` and runs the async eDMA UART
API.

`zephyr,console = &lpuart1`. [PROCESS.md](../../PROCESS.md) records a standing
policy (2026-08-15) that consoles move to USB CDC on all boards and lists this
board as not yet compliant. `CONFIG_LOG=n` here comes from `prj.nxprt1176.conf`
as a sanctioned case-by-case disable under the same policy, with the
cbprintf-hang rationale in that file.

## RC input

Two paths, as on Pixhawk-standard boards:

- Serial protocols (SBUS/CRSF and the rest) on `&lpuart6`: `single-wire`,
  `current-speed = <100000>`, eDMA. Reaches ArduPilot as SERIAL7 and is scanned
  by `AP_RCProtocol`.
- PPM-SUM (CPPM) by pulse capture: `rcin-gpios = <&gpio2 8 GPIO_ACTIVE_HIGH>` in
  the DTS `zephyr,user` node, with hardware capture via `pwms = <&qtmr1 0 0 0>`
  (`CONFIG_AP_RCIN_PWM_CAPTURE=y`, `CONFIG_PWM_CAPTURE=y`).

Both paths land on the same pad, `EMC_B1_40`, so `hwdef.dat` sets
`RCIN_PULSE_GPIO_SHARES_UART_PAD 1` and `RCInput.cpp` runs a runtime MUX_MODE
arbiter between them. Register-level detail is in
[RC input pad arbitration](#rc-input-pad-arbitration) below.

The board does have a dedicated PPM pad, `FMU_PPM_INPUT` = ball M2 =
`GPIO_EMC_B2_12` = `GPIO_MUX2_IO22` = gpio2 pin 22. It is not fed by this
carrier's RCIN position. Measured 2026-08-13 with the receiver in the carrier's
RCIN 3-pin servo-rail position, the CPPM pulse train arrived on `EMC_B1_40`
(about 470 transitions/s, 9 pulses at 50 Hz) and `EMC_B2_12` showed nothing. A
carrier that routes RC-IN conditioning to both nets could use pin 22 with
`RCIN_PULSE_GPIO_SHARES_UART_PAD` removed, in which case `RCInput.cpp` takes its
plain `gpio_pin_configure_dt()` path and no arbitration happens.

## PWM output

12 FMU-direct channels (`HAL_PWM_COUNT 12`, `NUM_SERVO_CHANNELS 16`). No IOMCU
is fitted (`HAL_WITH_IO_MCU 0`); the schematic makes it an optional part and our
hardware does not populate it. Channel map from `RCOutput.cpp`:

| Output | Zephyr PWM node | Pad         |
| ------ | --------------- | ----------- |
| 1      | `flexpwm1_pwm0` | `EMC_B1_23` |
| 2      | `flexpwm1_pwm1` | `EMC_B1_25` |
| 3      | `flexpwm1_pwm2` | `EMC_B1_27` |
| 4      | `flexpwm2_pwm0` | `EMC_B1_06` |
| 5      | `flexpwm2_pwm1` | `EMC_B1_08` |
| 6      | `flexpwm2_pwm2` | `EMC_B1_10` |
| 7      | `flexpwm2_pwm3` | `EMC_B1_19` |
| 8      | `flexpwm3_pwm0` | `EMC_B1_29` |
| 9      | `flexpwm3_pwm1` | `EMC_B1_31` |
| 10     | `flexpwm3_pwm3` | `EMC_B1_21` |
| 11     | `flexpwm4_pwm0` | `EMC_B1_00` |
| 12     | `flexpwm4_pwm1` | `EMC_B1_02` |

DShot is not implemented, though pads for channels 1-8 have a FlexIO pinctrl
state declared (`pinmux_dshot_fmu_ch1_8`) and `ap_rcout_mux` switches to it, see
[The DShot pinctrl precedent](#the-dshot-pinctrl-precedent).

## SPI devices

`&lpspi1`, `&lpspi2` and `&lpspi3` are enabled, each with `dmas`/`dma-names` and
`clock-frequency = <DT_FREQ_M(10)>`. `&lpspi4` is `status = "disabled"`. There
is no LPSPI5 on this board.

`SPIDEV` entries in `hwdef.dat`, generated into `HAL_SPI_DT_SPEC_DECLS` and
`HAL_SPI_DT_SPEC_LOOKUP`, all at 8 MHz, SPI mode 3:

| Device name        | Bus      | CS index     |
| ------------------ | -------- | ------------ |
| `imu_sensor1`      | `lpspi1` | `cs-gpios` 0 |
| `imu_sensor2`      | `lpspi2` | `cs-gpios` 0 |
| `imu_sensor3`      | `lpspi3` | `cs-gpios` 0 |
| `imu_sensor3_gyro` | `lpspi3` | `cs-gpios` 1 |

Devices are named by bus, not by part, because the fitted silicon varies by
board revision (`WIRING.md`, "Revision variance").

The generated `HAL_INS_PROBE_LIST` offers each bus to `Invensensev3` and then
`Invensense`:

```text
IMU Invensensev3 SPI:imu_sensor1 ROTATION_NONE
IMU Invensense   SPI:imu_sensor1 ROTATION_NONE
IMU Invensensev3 SPI:imu_sensor2 ROTATION_NONE
IMU Invensense   SPI:imu_sensor2 ROTATION_NONE
IMU Invensensev3 SPI:imu_sensor3 ROTATION_NONE
IMU Invensense   SPI:imu_sensor3 ROTATION_NONE
```

Detection decides what binds. Four constraints on that list:

- **Probes are not side-effect free.** `Invensensev2`'s whoami check writes a
  bank-select register, which corrupts a live ICM-426xx and makes the working
  IMU disappear. Only add a backend that is known non-destructive on the fitted
  parts.
- **`lpspi1`'s onboard IMU is an ICM-42686-P**, WHO_AM_I `0x44`, identified
  2026-08-09. ArduPilot's `Invensensev3` has no `0x44` entry, so it does not
  bind. Supporting it means adding `0x44` plus its scale table to
  `AP_InertialSensor_Invensensev3`, not adding hwdef probe lines.
- **`lpspi2` carries the ICM-42688-P** (`0x47`) and is the IMU the loop-rate
  numbers are measured on.
- **BMI088 on `lpspi3` is declarable but disabled.** The two-device probe line
  exists, commented out in `hwdef.dat`, and both chip selects are wired. It was
  disabled 2026-08-09 because its DeviceBus thread measured 25-26% of a core for
  a redundant second IMU instance.

Fast sampling is off by default (`HAL_DEFAULT_INS_FAST_SAMPLE 0`) on
bus-saturation grounds; the measurement is in the `hwdef.dat` comment.

`CONFIG_AP_SPI_PROBE_DIAG` is a read-only boot scan for "what is on this bus"
questions. It is `=n` in `prj.mr_vmu_rt1176.conf`.

## I2C devices

`&lpi2c1`, `&lpi2c2` and `&lpi2c3` are enabled at `I2C_BITRATE_FAST` (400 kHz),
each with `dmas`. `I2C_ORDER I2C1 I2C2 I2C3` gives `HAL_I2C_BUS_COUNT 3`:

| AP bus | Zephyr node | Role                                              |
| ------ | ----------- | ------------------------------------------------- |
| 0      | `lpi2c1`    | external / GPS1 connector, POWER1 SMBus           |
| 1      | `lpi2c2`    | onboard baro                                      |
| 2      | `lpi2c3`    | offboard daughtercard (baro plus compass, shared) |

Declared devices (`HAL_I2C_DEVICES_LIST`):

| Driver   | Bus | Address | Notes                                                   |
| -------- | --- | ------- | ------------------------------------------------------- |
| `BMP388` | 1   | `0x76`  | onboard baro                                            |
| `BMP388` | 2   | `0x77`  | offboard baro; the AP BMP388 driver also handles BMP390 |
| `BMM150` | 2   | `0x10`  | compass, marked external, `ROTATION_NONE`               |

`CONFIG_AP_I2C_PROBE_DIAG=n`. The full I2C probe scan ate most of the CPU and
stretched boot to over 170 s; with it off, boot is 4 s.

## Analog inputs and power

- `&lpadc1` is enabled, `&lpadc2` is `status = "disabled"`. `AnalogIn.cpp`
  compiles each under its own `DT_NODE_HAS_STATUS(...)` guard, so only LPADC1 is
  live.
- `hwdef.dat` sets `HAL_HAVE_BOARD_VOLTAGE 1` and `HAL_HAVE_SERVO_VOLTAGE 1`.
- There is no analog battery sense. POWER1 and POWER2 are SMBus smart-battery
  connectors: the BMS reports voltage and current over I2C and is configured at
  runtime with `BATT_MONITOR` parameters. The analog nets that exist are
  internal rail monitors. The DTS `aliases` node states this, having previously
  carried wrong `adc-batt0`/`adc-batt1` aliases.

## USB

`&usb1` (nxp,ehci) is enabled with `phy-handle = <&usbphy1>`. `&usbphy1` must
stay enabled or enumeration fails with "device not accepting address".
`CONFIG_USB_DEVICE_STACK_NEXT=y` with `CONFIG_USBD_CDC_ACM_CLASS=y`; VID/PID and
descriptor strings come from `USBD_DEVICE_DEFINE` and `USBD_DESC_*` in
`../../zephyr/src/main.cpp`. The board enumerates high-speed as
`27b1:0004 ArduPilot mr_vmu_rt1176` with the OCOTP UID as its serial.

| Node            | Interface label | Use                                             |
| --------------- | --------------- | ----------------------------------------------- |
| `usb_cdc_acm0`  | `MAVLink`       | SERIAL0 (GCS link), and `GPIO::usb_connected()` |
| `cdc_acm_uart1` | `SMP`           | `zephyr,uart-mcumgr` transport                  |

`Tools/scripts/61-ardupilot-zephyr.rules` renames the two host CDC nodes to
`-if-mavlink` and `-if-smp`, plus short `/dev/serial/by-ap/{mavlink,smp}` forms,
using the DTS interface `label` strings.

## CAN

`&flexcan1` and `&flexcan2` are enabled at `bitrate = <1000000>`. `hwdef.dat`
sets `HAL_NUM_CAN_IFACES 2` and `CAN_ORDER 1 2`; `boards.py` sets
`with_can = True` with `CANARD_IFACE_ALL = 0x3` and `CANARD_ENABLE_CANFD = 0`.

`CANIface.cpp` is a classic-CAN driver. CAN-FD is not supported. DroneCAN is the
only CAN protocol in scope: `AP_PICCOLOCAN_ENABLED 0` and
`AP_FETTEC_ONEWIRE_ENABLED 0` in `hwdef.dat`, each with its compile-failure
reason recorded there.

`HAL_STORAGE_SIZE 16384` is load-bearing for CAN. The 8 KB default gives
StorageManager only 10 areas with no `StorageCANDNA` slot, and AP_DroneCAN's DNA
server then fails to init.

