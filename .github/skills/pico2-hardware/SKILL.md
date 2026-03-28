---
name: pico2-hardware
description: "Raspberry Pi Pico2 / RP2350 hardware interaction for the ArduPilot port. Use when: flashing firmware via SWD or UF2; starting or restarting OpenOCD; connecting GDB to live hardware; live inspecting memory/registers; diagnosing USB CDC serial silence; diagnosing UART/GPIO FUNCSEL issues; halting vs resetting the board; using --nx with gdb; reading /dev/ttyACM* output; recovering from lost comms."
argument-hint: "What do you need to do? (flash / gdb / openocd / uart-debug / monitor-serial)"
---

# Pico2 / RP2350 Hardware Skill

## Hardware Setup

Two Pico2W boards are used:
- **Debugger** (labeled): flashed with `debugprobe_on_pico2.uf2` — provides CMSIS-DAP SWD + UART bridge
- **Target**: runs ArduPilot firmware

### Wiring (debugger → target)

| Debugger pin | Target pin | Signal |
|---|---|---|
| GP2 (pin 4) | SWCLK | SWD clock |
| GP3 (pin 5) | SWDIO | SWD data |
| GND (pin 3) | GND | Common ground — **mandatory** |
| GP4 (pin 6) / UART0RX | GP1 / pin 2 | UART console RX←TX |
| GP5 (pin 7) / UART0TX | GP0 / pin 1 | UART console TX→RX |

SWCLK/GND/SWDIO are also on the 3-pin debug header in the centre of the Pico2 board (left→right).

---

## OpenOCD

### Start (always background this)

```bash
~/openocd-pico/openocd \
  -c "gdb_port 50000" \
  -c "tcl_port 50001" \
  -c "telnet_port 50002" \
  -s ~/openocd-pico/scripts \
  -f interface/cmsis-dap.cfg \
  -f target/rp2350.cfg \
  -c "adapter speed 5000" &
```

### Good output (means it's working)

```
Info : SWD DPIDR 0x4c013477
Info : [rp2350.dap.core0] Cortex-M33 r1p0 processor detected
Info : Listening on port 50000 for gdb connections
```

### "Error connecting DP: cannot read IDR"
Target is not powered. Plug in the target USB cable.

### Lost comms / hung OpenOCD
```bash
pkill -f openocd          # kill existing process
# re-run the start command above
```

---

## Flash Workflows

There are **two separate flash paths** — bootloader (one-time) and app firmware (everyday update).

Before any step that requires touching the target board, the agent must stop and
explicitly prompt the human for that action. Do not assume the human will unplug,
re-plug, or hold BOOTSEL without being asked.

---

### 1. Bootloader — one-time BOOTSEL flash

The bootloader is built once and loaded via BOOTSEL/UF2. It is NOT updated by `--upload`.

```bash
# Build the bootloader
./waf configure --board=Pico2 --bootloader --debug
./waf bootloader -j12
```

Then ask the human:
> "Please hold the BOOTSEL button on the Pico2 while plugging in the USB cable, then release. Tell me when the device appears as a mass-storage drive."

Do not start the bootloader upload until the human confirms the board is in BOOTSEL mode.

Once in BOOTSEL mode, flash via `--upload` (triggers picotool UF2 path automatically):
```bash
./waf configure --board=Pico2 --bootloader
./waf bootloader --upload
```

---

### 2. App Firmware — `./waf copter --upload` (everyday path)

This uses `uploader.py` which speaks the ArduPilot MAVLink bootloader protocol over USB CDC.
**Requires the AP_Bootloader to already be installed (see above).**

**IMPORTANT: This path requires a human to unplug and re-plug the USB cable.**

Before running `--upload`, always tell the human:
> "Please unplug the Pico2 USB cable and plug it back in now."

Do not run `uploader.py` or `./waf ... --upload` until the human confirms they have done the re-plug.

Wait for `/dev/ttyACM*` to re-appear, then run:
```bash
./waf copter --upload
# or manually:
python3 Tools/scripts/uploader.py \
    --port /dev/ttyACM1,/dev/ttyACM0 \
    build/Pico2/bin/arducopter.apj
```

Expected output:
```
Found board bd,0 bootloader rev 5 on /dev/ttyACM1
Bootloader Protocol: 5
ChipDes:
  family: RP2350
Erase  : [====================] 100.0%
Program: [====================] 100.0%
Verify : [====================] 100.0%
Rebooting.
EXIT: 0
```

The full flash takes about 60–90 seconds. Use `timeout 120` if calling manually.

**Port locking error** (`[Errno 11] Could not exclusively lock port`): another process (mavproxy, cat, etc.) has the port open. Kill it first.

---

### 3. App Firmware via SWD (developer/debug path — no unplug required)

Use this when OpenOCD is already running and you don't want to disturb the running system,
or when the AP_Bootloader is not installed.

```bash
gdb-multiarch --nx --batch \
  -ex "target extended-remote :50000" \
  -ex "mon halt" \
  -ex "mon reset halt" \
  -ex "load build/Pico2/bin/arducopter" \
  -ex "mon reset run" \
  -ex "quit" \
  build/Pico2/bin/arducopter
```

Expected: `Transfer rate: ~113 KB/sec` — ~15 seconds for a 1.4 MB image.

Always flash the `.elf` / no-extension ELF (not `.uf2` or `.bin`).

**Note:** `arm-none-eabi-gdb` is broken on Ubuntu 24.04 (missing libncurses.so.5). Use `gdb-multiarch` instead.

---

## GDB Live Diagnosis

### CRITICAL: always use `--nx`

`/home/buzz/ardupilot/.gdbinit` contains `mon reset halt`.  
**This resets the board on every standard GDB connect**, masking the real live state.  
Always use `--nx` for diagnostics:

```bash
arm-none-eabi-gdb --nx build/Pico2/bin/arducopter
# then manually:
(gdb) target extended-remote :50000
(gdb) mon halt          # halt without reset
```

### Halt without reset via telnet (no GDB needed)

```bash
nc localhost 50002      # openocd telnet
> halt
> targets              # show state
```

### Useful GDB inspection commands

```gdb
# General state
info threads
bt                          # backtrace current thread

# Memory: read a struct field
p serial0Driver._writebuf.available()
p serial1Driver._device_initialised
p serial1Driver.uart_thread_ctx

# Read raw memory address
x/4wx 0x2000db18

# Show USB CDC state
p SDU1.state              # SDU_READY = good
p SDU1.config->usbp->state  # USB_ACTIVE = good
```

---

## Serial / USB Monitor

ArduPilot maps:
| Port | Device | GPIO |
|---|---|---|
| SERIAL0 / USB | `/dev/ttyACM0` or `ttyACM1` | USB connector |
| SERIAL1 | UART0 | TX=GP12, RX=GP13 |
| SERIAL2 | UART1 | TX=GP10, RX=GP11 |
| SERIAL3 | PIOUART0 | TX=GP14, RX=GP17 |

```bash
# Detect USB CDC port
ls /dev/ttyACM*

# Quick health check — should see "Init ArduCopter" text
timeout 5 cat /dev/ttyACM1 | head -c 200 | xxd

# MAVLink stream (human readable)
mavproxy.py --master=/dev/ttyACM1 --baudrate=115200
```

---

## MAVLink SERIAL_CONTROL — Correct Flag Values

**Pymavlink constants** (confirmed from `pymavlink.dialects.v20.common`):

| Flag | Value | Meaning |
|---|---|---|
| `REPLY` | 1 | FC-originated response — **if set, FC returns immediately without acting** |
| `RESPOND` | 2 | FC echoes received data back |
| `EXCLUSIVE` | 4 | Lock port exclusively |
| `BLOCKING` | 8 | Block waiting for data |
| `MULTI` | 16 | Multi-packet response |

**Common mistake:** `EXCLUSIVE=1<<0=1` sets the REPLY bit → `handle_serial_control()` hits early return at line ~40 of `GCS_MAVLink/GCS_serial_control.cpp`:
```cpp
if (packet.flags & SERIAL_CONTROL_FLAG_REPLY) { return; }
```
This means `begin()` is **never called** on the target UART — completely silent failure.

**Correct flags for open+respond**: `EXCLUSIVE | RESPOND = 4 | 2 = 6`

**Correct device IDs** (from `common.xml`):
```python
SERIAL_CONTROL_SERIAL0 = 100   # SERIAL0/USB
SERIAL_CONTROL_SERIAL1 = 101   # SERIAL1/UART0
SERIAL_CONTROL_SERIAL2 = 102   # SERIAL2/UART1
```

---

## Build Commands

```bash
cd /home/buzz/ardupilot

# Configure (first time or after hwdef changes)
./waf configure --board=Pico2 --debug

# Build copter
./waf copter -j12

# Bootloader only
./waf configure --board=Pico2 --debug --bootloader
./waf bootloader -j12
```

Build output: `build/Pico2/bin/arducopter` (ELF, ~1.5 MB)

---

## UART GPIO FUNCSEL Fix (UARTDriver.cpp)

RP2350 requires explicit `FUNCSEL=2` (UART) on GPIO pads before calling `sioStart()`.
Without it, the pad stays in default GPIO mode and UART is silent.

Fix location: `libraries/AP_HAL_ChibiOS/UARTDriver.cpp`, in `_begin()`, SIO path:
```cpp
// Set GPIO function to UART (FUNCSEL=2) before sioStart — RP2350 requirement
if (sdef.tx_line != 0) {
    palSetLineMode(sdef.tx_line, PAL_MODE_ALTERNATE_UART);
}
if (sdef.rx_line != 0) {
    palSetLineMode(sdef.rx_line, PAL_MODE_ALTERNATE_UART);
    palLineSetPushPull(sdef.rx_line, PAL_PUSHPULL_PULLUP);
}
sioStart((SIODriver*)sdef.serial, &siocfg);
```

---

## Memory Map (confirmed addresses)

| Symbol | Address |
|---|---|
| `serial0Driver` (SERIAL0/USB) | `0x2000d9a0` |
| `serial1Driver` (SERIAL1/UART0) | `0x2000db18` |
| `serial2Driver` (SERIAL2/UART1) | `0x2000dc90` |
| `_serial_tab` (ROM) | `0x1017c890` |

---

## Test Script

`Tools/debug/test_uart_pico2.py` — MAVLink SERIAL_CONTROL exerciser.

Usage:
```bash
source zephyrproject/.venv/bin/activate
python3 Tools/debug/test_uart_pico2.py --port /dev/ttyACM1
python3 Tools/debug/test_uart_pico2.py --port /dev/ttyACM1 --loopback  # needs GPIO loopback wire
```

Loopback wiring for hardware verification:
- UART0: GP12 → GP13
- UART1: GP10 → GP11

---

## Common Pitfalls

| Symptom | Cause | Fix |
|---|---|---|
| USB CDC goes silent after SERIAL_CONTROL | Wrong REPLY flag set (bit0=1) triggers early return | Use `flags = EXCLUSIVE\|RESPOND = 6` |
| GDB connect resets board unexpectedly | `.gdbinit` has `mon reset halt` | Use `gdb --nx` for live diagnosis |
| OpenOCD "cannot read IDR" | Target not powered | Plug in target USB |
| UART completely silent | FUNCSEL not set to UART before sioStart | Add `palSetLineMode(ALTERNATE_UART)` in `_begin()` |
| `serial1Driver._device_initialised = false` | `begin()` was never called | SERIAL_CONTROL flags wrong, or device ID wrong |
| No heartbeats on USB after test | Write buffer empty (not a freeze) | Board is running fine; check MAVLink connection |
