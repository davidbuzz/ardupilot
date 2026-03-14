# Raspberry Pi Pico 2 (RP2350) ArduPilot Port

The Pico2 target runs ArduPilot on the Raspberry Pi Pico 2 module
(RP2350 Cortex-M33 @ 150 MHz). The hwdef.dat is designed for a
carrier board that provides SPI/I2C sensors and exposes the RP2350's
full GPIO range including pins above GPIO29 (available on the Pico2
castellated edges).

## Features

- RP2350 dual-core Cortex-M33 @ 150 MHz (ArduPilot uses one core)
- 520 KB SRAM
- 4 MB external QSPI flash (parameter storage in last 32 KB)
- USB CDC serial (SERIAL0)
- 2 hardware UARTs + 3 PIO UARTs (5 telemetry/GPS ports)
- 8 PWM outputs (GPIO 0–7, 4 PWM slices × 2 channels)
- SPI0 and SPI1 buses for IMU, barometer, and compass sensors
- I2C1 bus (GPIO 15/18) for external compass / GPS
- 2 ADC channels for battery voltage and current sensing
- RC input via GPIO PAL callback (GPIO 16, PULLDOWN)
- SBUS/inverted UART on UART0/UART1 via hardware INOVER
- Watchdog (2 s timeout)
- IMU temperature monitoring (HEAT log messages)

## UART Mapping

| Serial port | Function | GPIO (Pico2 physical pin) |
|-------------|----------|--------------------------|
| SERIAL0 | USB / console | USB connector |
| SERIAL1 | Telem1 / GPS | TX=GPIO12 (pin 16), RX=GPIO13 (pin 17) |
| SERIAL2 | Telem2 | TX=GPIO10 (pin 14), RX=GPIO11 (pin 15) |
| SERIAL3 | GPS / spare (PIOUART0) | TX=GPIO14 (pin 19), RX=GPIO17 (pin 22) |
| SERIAL4 | spare (PIOUART1) | TX=GPIO19 (pin 25), RX=GPIO20 (pin 26) |
| SERIAL5 | spare (PIOUART2) | TX=GPIO21 (pin 27), RX=GPIO27 (pin 32) |

SBUS (100 kbps, 8E2, inverted) is supported on SERIAL1/SERIAL2 using
the RP2350 GPIO INOVER bit — no external inverter required. Set
`SERIAL_n_PROTOCOL=23` (RC Input) and `SERIAL_n_OPTIONS=3` on the
connected port.

## RC Input

Connect RC receiver signal to **GPIO 16** (pin 21). The pin is
PULLDOWN; for PWM/PPM receivers a 10 kΩ pull-up to 3.3 V may be
needed. SBUS can be wired to **GPIO 13** (UART0 RX, SERIAL1).

## PWM / Servo Outputs

| Output | GPIO | Pico2 pin |
|--------|------|-----------|
| PWM1 | GPIO 0 | pin 1 |
| PWM2 | GPIO 1 | pin 2 |
| PWM3 | GPIO 2 | pin 4 |
| PWM4 | GPIO 3 | pin 5 |
| PWM5 | GPIO 4 | pin 6 |
| PWM6 | GPIO 5 | pin 7 |
| PWM7 | GPIO 6 | pin 9 |
| PWM8 | GPIO 7 | pin 10 |

Standard 50 Hz PWM servo output. DShot is not supported (no
timer DMA on RP2350). PWM outputs can also be used as GPIOs by
setting `BRD_PWM_COUNT` < 8; GPIO numbers start at 50 (PWM1=50 ...
PWM8=57).

## SPI Buses (carrier board only)

Pins above GPIO29 are not accessible on the standard Pico2 breakout
header — they appear as castellated-edge pads used by a flight
controller carrier board.

**SPI0** (barometer bus)

| Signal | GPIO |
|--------|------|
| SCK | GPIO 22 |
| MISO (RX) | GPIO 32 |
| MOSI (TX) | GPIO 35 |
| BARO_EXT_CS | GPIO 25 |

**SPI1** (IMU bus)

| Signal | GPIO |
|--------|------|
| SCK | GPIO 42 |
| MISO | GPIO 40 |
| MOSI | GPIO 43 |
| MPU_CS | GPIO 24 |
| MAG_CS | GPIO 23 |
| GYRO_EXT_CS | GPIO 26 |

## I2C Bus

| Signal | GPIO | Pico2 pin |
|--------|------|-----------|
| I2C1 SCL | GPIO 15 | pin 20 |
| I2C1 SDA | GPIO 18 | pin 24 |

`AP_COMPASS_PROBING_ENABLED 1`: ArduPilot auto-probes for HMC5883,
IST8310, QMC5883 and other common I2C compasses.

## Battery Monitoring

| Signal | GPIO | Pico2 pin | Default scale |
|--------|------|-----------|---------------|
| BATT_VOLTAGE_SENS | GPIO 28 | pin 34 | 10.1 V/V |
| BATT_CURRENT_SENS | GPIO 29 | pin 35 | 17.0 A/V |

## Sensor Stack

The hwdef probes the following sensors; missing sensors are silently
skipped via WHOAMI check:

| Sensor | Bus | CS pin |
|--------|-----|--------|
| Invensense gen-1 IMU (mpu6000 / mpu9250) | SPI1 | MPU_CS |
| Invensense gen-2 IMU (ICM20948 / AK09916) | SPI1 | MPU_CS |
| MS5611 barometer | SPI0 | BARO_EXT_CS |
| Compass (I2C auto-probe) | I2C1 | — |
| LSM9DS0 gyro-only | SPI1 | GYRO_EXT_CS |

Note: LSM9DS0 requires two CS lines (gyro + accel/mag). The accel/mag
CS (`ACCEL_EXT_CS`) is not mapped on this carrier; only the gyro
channel can be detected.

## IMU Temperature Monitoring

`HAL_HAVE_IMU_HEATER 1` is enabled. No physical heating element
exists on the standard Pico2; `HAL_HEATER_GPIO_PIN` is not defined.
The subsystem still logs IMU temperature to the `HEAT` log message
(1 Hz) and exposes the `BRD_IMU_TARGTEMP` parameter for arming
temperature checks. An external heater resistor + MOSFET can be added
by defining `HAL_HEATER_GPIO_PIN` in a local hwdef override.

## Parameter Storage

Parameters are stored in the last 32 KB of the 4 MB QSPI flash
(sectors 1016–1023). `AP_FLASH_STORAGE_QUAD_PAGE 1` aggregates
4 × 4 KB physical pages into one 16 KB logical sector for correct
wear-levelling. Storage capacity is 8 KB (vs 16 KB on CubeBlack).

If an external SPI FRAM (ramtron-compatible) is wired to a free
SPI bus, uncomment the `#SPIDEV ramtron` and `#PA_n FRAM_CS CS`
lines in hwdef.dat to use it instead of flash.

## Firmware Building

```bash
./waf configure --board=Pico2
./waf copter        # (or plane, rover, sub, heli, etc.)
```

Flash `build/Pico2/bin/arducopter.elf` via SWD (SWCLK=GPIO0,
SWDIO=GPIO1 via PC0/PC1 in hwdef) or the RP2350 UF2 bootloader.

## setting up hardware debugging, using *two* Pico2Ws, one running debugprobe_on_pico2.uf2
    https://github.com/raspberrypi/debugprobe/releases/download/debugprobe-v2.3.0/debugprobe_on_pico2.uf2
    as the debugger, and the other running ardupilot as the "target"

    BOOTSEL flash the above file to a Pico2w, label it "debugger", and ..

    holding a 2w with the cpu facing you, and the usb plug up, the top-left pins starting at the corner, we're interested, on the debugger in skipping two pins , then putting a wire on the next 5 pins.... 
    pin1/GP0 skip
    pin2/GP1 skip
    pin3/GND use/GND
    pin4/GP2 use/SWCLK
    pin5/GP3 use/SWDIO
    pin6/GP4 use/UART0RX to GP1
    pin7/GP5 use/UART0TX to GP0

    Debugger(Probe) | Target-Pico-2w       | 	Notes
    pin4/GP2	    |       SWCLK          |	 Clock signal
    pin5/GP3	    |       SWDIO          |	 Data I/O
    pin3/GND        |   	GND	           |	Common ground is mandatory
    pin6/GP4        |   UART0RX/GP1/pin2   |	
    pin7/GP5        |   UART0TX/GP0/pin1   |	

    SWCLK/GND/SWDIO are found on the three pin header in the middle of the board, and if held in the orientation as described above, and left-to-right in that order...  SWCLK(left)/GND(middle)/SWDIO(right)

# Get a compatible openocd... eg get premade binaries here:
    https://github.com/raspberrypi/pico-sdk-tools/releases/tag/v2.1.0-0

    cd ~/Downloads
    wget https://github.com/raspberrypi/pico-sdk-tools/releases/download/v2.1.0-0/openocd-0.12.0+dev-x86_64-lin.tar.gz
    mkdir ~/openocd-pico
    mv openocd-0.12.0+dev-x86_64-lin.tar.gz ~/openocd-pico
    cd ~/openocd-pico
    tar -zxvf openocd-0.12.0+dev-x86_64-lin.tar.gz
    ~/openocd-pico/openocd
        Open On-Chip Debugger 0.12.0+dev-gebec950-dirty (2024-11-25-10:19)

# start openocd and if it finds your usb connected hardware/debugger/picoprobe, leave it running.
    ~/openocd-pico/openocd -c "gdb_port 50000" -c "tcl_port 50001" -c "telnet_port 50002" -s ~/openocd-pico/scripts  -f interface/cmsis-dap.cfg -f target/rp2350.cfg -c "adapter speed 5000"
    Error: Error connecting DP: cannot read IDR
    # if u get this, you didnt plug the second usb cable in, this setup needs both, the target isnt booted/powered.
    # if it stays running without errors, mentions 'Cortex-M33 r1p0 processor detected' and is last line says ..
    'Info : Listening on port 50000 for gdb connections', then its working perfectly.

    good example output: 
        Open On-Chip Debugger 0.12.0+dev-gebec950-dirty (2024-11-25-10:19)
        Licensed under GNU GPL v2
        For bug reports, read
            http://openocd.org/doc/doxygen/bugs.html
        Info : Hardware thread awareness created
        Info : Hardware thread awareness created
        Info : Hardware thread awareness created
        Info : Hardware thread awareness created
        cortex_m reset_config sysresetreq
        adapter speed: 5000 kHz
        Info : Listening on port 50001 for tcl connections
        Info : Listening on port 50002 for telnet connections
        Info : Using CMSIS-DAPv2 interface with VID:PID=0x2e8a:0x000c, serial=E5CC2F739DCD6475
        Info : CMSIS-DAP: SWD supported
        Info : CMSIS-DAP: Atomic commands supported
        Info : CMSIS-DAP: Test domain timer supported
        Info : CMSIS-DAP: FW Version = 2.0.0
        Info : CMSIS-DAP: Interface Initialised (SWD)
        Info : SWCLK/TCK = 0 SWDIO/TMS = 0 TDI = 0 TDO = 0 nTRST = 0 nRESET = 0
        Info : CMSIS-DAP: Interface ready
        Info : clock speed 5000 kHz
        Info : SWD DPIDR 0x4c013477
        Info : [rp2350.dap.core0] Cortex-M33 r1p0 processor detected
        Info : [rp2350.dap.core0] target has 8 breakpoints, 4 watchpoints
        Info : [rp2350.dap.core0] Examination succeed
        Info : [rp2350.dap.core1] Cortex-M33 r1p0 processor detected
        Info : [rp2350.dap.core1] target has 8 breakpoints, 4 watchpoints
        Info : [rp2350.dap.core1] Examination succeed
        Info : starting gdb server for rp2350.dap.core0 on 50000
        Info : Listening on port 50000 for gdb connections



# make a .gdbinit file.
    cd ~/ardupilot/
    echo "# this sets up gdb to use openocd. You must start openocd first" > .gdbinit
    echo "target extended-remote :50000">> .gdbinit
    echo "mon reset halt">> .gdbinit
    echo "set confirm off">> .gdbinit

# locate files and run your debugger, pointing it at the *elf*, not the uf2 or aything.
    which arm-none-eabi-gdb
        /opt/gcc-arm-none-eabi-10-2020-q4-major/bin//arm-none-eabi-gdb 
    file build/Pico2/bin/arducopter
        build/Pico2/bin/arducopter: ELF 32-bit LSB executable, ARM, EABI5 version 1 (SYSV), statically linked, with debug_info, not stripped
    # run it...
    arm-none-eabi-gdb --quiet ./build/Pico2/bin/arducopter

       #example output of what it should look like:
            arm-none-eabi-gdb --quiet ./build/Pico2/bin/arducopter 
            Reading symbols from ./build/Pico2/bin/arducopter...
            warning: multi-threaded target stopped without sending a thread-id, using first non-exited thread
            0x00000088 in ?? ()
            [rp2350.dap.core1] VECTRESET is not supported on this Cortex-M core, using SYSRESETREQ instead.
            [rp2350.dap.core1] Set 'cortex_m reset_config sysresetreq'.
            [rp2350.dap.core0] halted due to debug-request, current mode: Thread 
            xPSR: 0xf9000000 pc: 0x00000088 msp: 0xf0000000
            [rp2350.dap.core1] halted due to debug-request, current mode: Thread 
            xPSR: 0xf9000000 pc: 0x00000088 msp: 0xf0000000
            (gdb) 

        # yay! 
    
    # recompile it all with --debug for a better gdb experience. ( add it to both waf commands)

# eg, possible three terminal setup...
    terminal 1 for build and upload:
    make clean ; 
    make -d  > make3.debug.log 2>&1 ; 
    make upload
    [then hold button and re-plug usb within 10 seconds to upload]

    terminal 2 just runs openocd, most of the time:
    started openocd:
    ~/openocd-pico/openocd -c "gdb_port 50000" -c "tcl_port 50001" -c "telnet_port 50002" -s ~/openocd-pico/scripts  -f interface/cmsis-dap.cfg -f target/rp2350.cfg -c "adapter speed 5000"

    terminal 3 for debugger/gdb etc:
    cd /home/buzz2/Chibi/ChibiOS-RT/demos/RP/RT-RP2040-PICO
    gdb connected, and poked around - seems to not crash immediatly any more:
    /opt/gcc-arm-none-eabi-10-2020-q4-major/bin//arm-none-eabi-gdb ./build/ch.elf


todo document use of openocd here.

## Firmware Flashing

```bash
./waf configure --board=Pico2 --bootloader
[ Hold the BOOTSEL button on the Pico2 while plugging in USB,
then release. The board will appear as a mass-storage device.]
./waf bootloader --upload
[ .uf2 bootloader will be uploaded for you and it will reboot, probably staying as a mass-storage device. ]

./waf configure --board=Pico2
./waf copter --upload
[ If the board does not respond within 1-2 seconds, unplug and re-plug the USB connector ]
   
./waf copter        # (or plane, rover, sub, heli, etc.)
```

Flash `build/Pico2/bin/arducopter.elf` via SWD (SWCLK=GPIO0,
SWDIO=GPIO1 via PC0/PC1 in hwdef) or the RP2350 UF2 bootloader.

## Known Limitations

| Feature | Status |
|---------|--------|
| DShot / BLHeli / SerialLED | Not supported — no timer DMA on RP2350 |
| UART RTS/CTS flow control | Not supported — PIOUART has no flow control |
| CAN / DroneCAN | Not feasible — RP2350 has no CAN peripheral |
| IOMCU | Not applicable — Pico2 drives servos directly |
| microSD / FAT logging | Not wired — no SDIO, SPI-mode SD not included |
| I2C2 | Not available — not pinned out on the standard carrier |
| Hardware RTC | GPS time used; RP2350 removed hardware RTC (POWMAN_TIMER) |
| Storage | 8 KB parameter storage (CubeBlack has 16 KB) |
| Gyro FFT / DSP | Deferred — CMSIS-DSP needs ArduPilot integration for RP2350 |

