# ArduPilot / Pico2 — Claude Code Instructions

## IMPORTANT: Also read AGENTS.md

**Read [`AGENTS.md`](AGENTS.md) at the start of every session.** It contains mandatory rules for this project including:
- Commit message format (`Subsystem: description`, WIP prefix for local work)
- Git commit every ~5 minutes during active coding
- TODO list must always end with: git-commit + review FEATURE_GAP.md for next task
- `git worktree` commands require explicit prior user/operator/admin consent after stating the intended path/name, target branch or commit, and purpose
- Coding style (4 spaces, K&R braces, `#pragma once`, snake_case methods)
- What AI must NOT do (no fabrication, no safety-critical guessing, no submodule edits except ChibiOS RP2350)
- After fixing one hardware fault, always continue to the next item in FEATURE_GAP.md

## Skills

This project has a hardware interaction skill for the Raspberry Pi Pico2 / RP2350 target.

Use the **`pico2-hardware`** skill (`.github/skills/pico2-hardware/SKILL.md`) whenever:
- Flashing new firmware to the Pico2 (always use GDB+OpenOCD, never `--upload`)
- Starting or checking OpenOCD (`~/openocd-pico/openocd`)
- Reading memory/registers on live hardware (OpenOCD telnet on port 50002)
- Verifying boot-time canary values (e.g. `c1_boot_stage`, `c1_startup_result`)
- Diagnosing USB CDC or UART silence
- Any "did it work on hardware?" question

**Always test on hardware after a firmware change** — build success is not enough.

## Hardware Setup

- OpenOCD: `~/openocd-pico/openocd`, scripts in `~/openocd-pico/scripts`
- Interface: CMSIS-DAP (`interface/cmsis-dap.cfg`), target: `target/rp2350.cfg`
- GDB port: 50000, telnet: 50002
- Debug probe: Pico2W with `debugprobe_on_pico2.uf2`
- **Always use `gdb --nx`** for live diagnosis (`.gdbinit` resets board on connect)

## Build

```bash
cd /home/buzz2/ardupilot
python waf build --target bin/arducopter   # incremental
./waf configure --board=Pico2 --debug      # after hwdef changes
```

## Feature Gap / Work Queue

[`libraries/AP_HAL_ChibiOS/hwdef/Pico2/FEATURE_GAP.md`](libraries/AP_HAL_ChibiOS/hwdef/Pico2/FEATURE_GAP.md) is the AI-maintained task list and status tracker for this port. Read it and keep it up to date:

- **After completing any task**: update the relevant row's status column and notes
- **When choosing next work**: pick the highest-priority `❌ not done` item
- **After every git commit**: review FEATURE_GAP.md to decide what to work on next (this is also mandated by AGENTS.md)

Legend: ✅ done · ⚠️ partial · ❌ not done · 🚫 not feasible · 💡 driver exists, just needs enabling

## Reference Material

**Always read these before making hardware-related changes:**

- [`libraries/AP_HAL_ChibiOS/hwdef/Pico2/README.md`](libraries/AP_HAL_ChibiOS/hwdef/Pico2/README.md) — canonical reference for pin assignments, UART/SPI/I2C/PWM mapping, sensor stack, known limitations, and build/flash instructions. If a question involves GPIO numbers, serial ports, sensor buses, or peripheral capabilities, check here first.

- [`libraries/AP_HAL_ChibiOS/hwdef/Pico2/datasheet/rp2350_datasheet-no-images.md`](libraries/AP_HAL_ChibiOS/hwdef/Pico2/datasheet/rp2350_datasheet-no-images.md) — local RP2350 datasheet (text, LLM-readable). Use for register layouts, peripheral details (SIO FIFO, PIO, DMA, clocks, etc.).

Key facts from README:
- SERIAL0=USB, SERIAL1=UART0 (GP12/13), SERIAL2=UART1 (GP10/11), SERIAL3-5=PIO UARTs
- PWM outputs: GPIO 0–7 (8 channels, 50 Hz, no DShot)
- SPI1=IMU (GP42/40/43), SPI0=baro (GP22/32/35)
- I2C1: SCL=GP15, SDA=GP18
- RC input: GPIO 16
- No CAN, no SD card, no hardware RTC, no DShot

## Branch

Active branch: `buzz-rp2350-chibios-v2` — RP2350/Pico2 ChibiOS HAL port.
