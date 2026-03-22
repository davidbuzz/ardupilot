#!/usr/bin/env python3
"""
hw_check_gdb.py — Pico2/RP2350 ArduPilot hardware register verification.

Connects to OpenOCD via GDB port (50000) and reads peripheral registers
to verify the hardware is configured correctly. Use this after flashing
to confirm the firmware has initialised all peripherals correctly.

Usage:
  # Start OpenOCD first:
  #   ~/openocd-pico/openocd -c "gdb_port 50000" -c "tcl_port 50001" \
  #     -c "telnet_port 50002" -s ~/openocd-pico/scripts \
  #     -f interface/cmsis-dap.cfg -f target/rp2350.cfg \
  #     -c "adapter speed 5000" &
  python3 hw_check_gdb.py [--elf /path/to/arducopter] [--port 50000]
  python3 hw_check_gdb.py --elf build/Pico2/bin/arducopter

Requires: arm-none-eabi-gdb in PATH, OpenOCD running on gdb_port.
"""

import argparse
import subprocess
import sys
import re
import os

DEFAULT_ELF = os.path.expanduser(
    '/home/buzz2/ardupilot/build/Pico2/bin/arducopter'
)

# ---------------------------------------------------------------------------
# RP2350 peripheral base addresses
# ---------------------------------------------------------------------------
SIO_BASE        = 0xd0000000
IO_BANK0_BASE   = 0x40028000
PADS_BANK0_BASE = 0x40038000
ADC_BASE        = 0x400a0000  # RP2350 canonical; 0x400a4000 also aliases
WDG_BASE        = 0x400d8000
OTP_BASE        = 0x40130000
PIO0_BASE       = 0x50200000
PIO1_BASE       = 0x50300000
# SPI (RP2350 addresses — differ from RP2040)
SPI0_BASE       = 0x40080000
SPI1_BASE       = 0x40088000
# PWM
PWM_BASE        = 0x400a8000

SYSCLK_HZ       = 250_000_000


def rd32(addr: int) -> str:
    return f"*(unsigned int*)0x{addr:08x}"


def build_gdb_script(elf_path: str) -> list[str]:
    """Build list of GDB commands that print register values."""
    cmds = []
    if elf_path and os.path.exists(elf_path):
        cmds += [f'file "{elf_path}"']

    def p(label, addr):
        cmds.append(f'printf "{label} = 0x%08x\\n", {rd32(addr)}')

    def pf(label, expr):
        cmds.append(f'printf "{label}\\n", {expr}')

    # ---- Core1 status ----
    cmds.append('printf "\\n=== Core1 status ===\\n"')
    p('c1_boot_stage', 0x20005408)
    if elf_path and os.path.exists(elf_path):
        cmds.append('printf "c1_startup_result = 0x%08x\\n", c1_startup_result')
    p('SIO_FIFO_ST', SIO_BASE + 0x054)

    # ---- GPIO CTRL (FUNCSEL) for key pins — diagnose peripheral routing ----
    # IO_BANK0->GPIO[n].CTRL is at IO_BANK0_BASE + n*8 + 4
    # FUNCSEL field is bits[4:0]:  1=SPI, 2=UART, 4=PWM, 5=SIO, 31=NULL
    cmds.append('printf "\\n=== GPIO CTRL / FUNCSEL (key pins) ===\\n"')
    p('GPIO0  CTRL (PWM slice0A, expect 4=PWM)', IO_BANK0_BASE + 0*8 + 4)
    p('GPIO10 CTRL (UART1 TX,   expect 2=UART)', IO_BANK0_BASE + 10*8 + 4)
    p('GPIO12 CTRL (UART0 TX,   expect 2=UART)', IO_BANK0_BASE + 12*8 + 4)
    p('GPIO22 CTRL (SPI0 SCK,   expect 1=SPI or 5=SIO-idle)',  IO_BANK0_BASE + 22*8 + 4)
    p('GPIO32 CTRL (SPI0 MISO,  expect 1=SPI)',  IO_BANK0_BASE + 32*8 + 4)
    p('GPIO35 CTRL (SPI0 MOSI,  expect 1=SPI)',  IO_BANK0_BASE + 35*8 + 4)
    p('GPIO40 CTRL (SPI1 MISO,  expect 1=SPI)',  IO_BANK0_BASE + 40*8 + 4)
    p('GPIO42 CTRL (SPI1 SCK,   expect 1=SPI or 5=SIO-idle)',  IO_BANK0_BASE + 42*8 + 4)
    p('GPIO43 CTRL (SPI1 MOSI,  expect 1=SPI)',  IO_BANK0_BASE + 43*8 + 4)

    # ---- SIO GPIO ----
    cmds.append('printf "\\n=== SIO GPIO (GPIO 0-31) ===\\n"')
    p('GPIO_OUT (0x010)', SIO_BASE + 0x010)
    p('GPIO_OE  (0x030)', SIO_BASE + 0x030)

    # ---- PWM (slices 0-3, GPIO0-7) ----
    cmds.append('printf "\\n=== PWM (slices 0-3, GPIO0-7, expect 50Hz) ===\\n"')
    for sl in range(4):
        base = PWM_BASE + sl * 0x14
        cmds.append(
            f'printf "Slice{sl}: CSR=0x%04x DIV=0x%06x TOP=0x%04x CC=0x%08x\\n",'
            f' {rd32(base)}&0xffff, ({rd32(base+0x4)})&0xffffff,'
            f' {rd32(base+0x8)}&0xffff, {rd32(base+0xc)}'
        )

    # ---- UART0 GPIO and registers ----
    # RP2350 UART addresses differ from RP2040: UART0=0x40070000, UART1=0x40078000
    UART0_BASE = 0x40070000
    UART1_BASE = 0x40078000
    cmds.append('printf "\\n=== UART0 (GPIO12=TX, GPIO13=RX) ===\\n"')
    p('GPIO12 FUNCSEL (expect 2=UART)', IO_BANK0_BASE + 12*8 + 4)
    p('GPIO13 FUNCSEL (expect 2=UART)', IO_BANK0_BASE + 13*8 + 4)
    p('UART0 IBRD', UART0_BASE + 0x24)
    p('UART0 FBRD', UART0_BASE + 0x28)
    p('UART0 LCR_H', UART0_BASE + 0x2c)
    p('UART0 CR', UART0_BASE + 0x30)

    cmds.append('printf "\\n=== UART1 (GPIO10=TX, GPIO11=RX) ===\\n"')
    p('GPIO10 FUNCSEL (expect 2=UART)', IO_BANK0_BASE + 10*8 + 4)
    p('GPIO11 FUNCSEL (expect 2=UART)', IO_BANK0_BASE + 11*8 + 4)
    p('UART1 IBRD', UART1_BASE + 0x24)
    p('UART1 FBRD', UART1_BASE + 0x28)
    p('UART1 CR',   UART1_BASE + 0x30)

    # ---- SPI0 (GPIO22=SCK, GPIO32=MISO, GPIO35=MOSI) ----
    cmds.append('printf "\\n=== SPI0 (GPIO22=SCK/35=MOSI/32=MISO) ===\\n"')
    p('GPIO22 FUNCSEL (SCK, expect 5=SIO at idle)', IO_BANK0_BASE + 22*8 + 4)
    p('GPIO32 FUNCSEL (MISO, expect 1=SPI)', IO_BANK0_BASE + 32*8 + 4)
    p('GPIO35 FUNCSEL (MOSI, expect 1=SPI)', IO_BANK0_BASE + 35*8 + 4)
    p('SPI0 SSPCPSR (clock prescale)', SPI0_BASE + 0x010)
    p('SPI0 SSPCR0 (frame format)', SPI0_BASE + 0x000)
    p('SPI0 SSPCR1 (SSE=enabled)', SPI0_BASE + 0x004)

    # ---- SPI1 (GPIO42=SCK, GPIO40=MISO, GPIO43=MOSI) ----
    cmds.append('printf "\\n=== SPI1 (GPIO42=SCK/43=MOSI/40=MISO) ===\\n"')
    p('GPIO40 FUNCSEL (MISO, expect 1=SPI)', IO_BANK0_BASE + 40*8 + 4)
    p('GPIO42 FUNCSEL (SCK, expect 5=SIO at idle)', IO_BANK0_BASE + 42*8 + 4)
    p('GPIO43 FUNCSEL (MOSI, expect 1=SPI)', IO_BANK0_BASE + 43*8 + 4)
    p('SPI1 SSPCPSR', SPI1_BASE + 0x010)

    # ---- SPI CS pins ----
    cmds.append('printf "\\n=== SPI CS pins (GPIO23-26, expect OE=1 OUT=1) ===\\n"')
    p('GPIO_OE  bits[26:23]', SIO_BASE + 0x030)
    p('GPIO_OUT bits[26:23]', SIO_BASE + 0x010)
    for gpio, name in [(23, 'MAG_CS'), (24, 'MPU_CS'), (25, 'BARO_EXT_CS'), (26, 'GYRO_EXT_CS')]:
        p(f'GPIO{gpio} ({name}) FUNCSEL', IO_BANK0_BASE + gpio*8 + 4)

    # ---- ADC ----
    cmds.append('printf "\\n=== ADC (GPIO28=BATT_V, GPIO29=BATT_I, ch4=temp) ===\\n"')
    p('ADC_CS (EN=1,TS_EN=1,START_MANY=1)', ADC_BASE + 0x000)
    p('ADC_RESULT', ADC_BASE + 0x004)
    p('GPIO28 FUNCSEL (expect 31=NULL/analog)', IO_BANK0_BASE + 28*8 + 4)
    p('PADS GPIO28 (expect ISO=1,IE=0)', PADS_BANK0_BASE + 0x074)

    # ---- Watchdog ----
    cmds.append('printf "\\n=== Watchdog ===\\n"')
    p('WATCHDOG_CTRL (ENABLE=bit30, PAUSE_DBG=bits25:24)', WDG_BASE)
    p('WATCHDOG_REASON (0=clean boot, 1=WDT reset)', WDG_BASE + 0x008)

    # ---- OTP unique ID ----
    cmds.append('printf "\\n=== OTP unique device ID ===\\n"')
    for i, name in enumerate(['CHIPID0', 'CHIPID1', 'CHIPID2', 'CHIPID3', 'RANDID0', 'RANDID1']):
        p(f'OTP row{i} ({name})', OTP_BASE + i*4)

    return cmds


def run_gdb(gdb_cmds: list[str], port: int) -> str:
    args = [
        'arm-none-eabi-gdb', '--nx',
        '-ex', 'set confirm off',
        '-ex', f'target extended-remote :{port}',
    ]
    for cmd in gdb_cmds:
        args += ['-ex', cmd]
    args += ['-ex', 'quit']

    result = subprocess.run(args, capture_output=True, text=True, timeout=30)
    return result.stdout + result.stderr


def parse_and_report(output: str):
    """Parse GDB output and print a human-readable report with PASS/FAIL."""
    lines = output.splitlines()

    # Extract register values
    regs = {}
    for line in lines:
        m = re.match(r'(.+?)\s*=\s*(0x[0-9a-fA-F]+)', line)
        if m:
            regs[m.group(1).strip()] = int(m.group(2), 16)

    print(output)  # Raw output

    # --- Quick PASS/FAIL checks ---
    print('\n' + '='*60)
    print('QUICK CHECK SUMMARY')
    print('='*60)

    checks = []

    def check(name, val, expected, mask=0xFFFFFFFF, info=''):
        actual = val & mask
        ok = (actual == (expected & mask))
        sym = 'PASS' if ok else 'FAIL'
        checks.append(ok)
        exp_str = f'0x{(expected&mask):x}'
        act_str = f'0x{actual:x}'
        note = f'  ({info})' if info else ''
        print(f'  [{sym}] {name}: got {act_str}, expected {exp_str}{note}')

    def reg(key):
        return regs.get(key, None)

    # Core1
    if (v := reg('c1_boot_stage')) is not None:
        check('Core1 boot_stage (idle=0x4d)', v, 0x4d, info='0x4d=idle loop running')
    if (v := reg('c1_startup_result')) is not None:
        check('Core1 startup_result (0xDEADC1C1)', v, 0xDEADC1C1, info='ping succeeded')

    # PWM: expect CSR bit0=1 (EN), TOP=0x4E20=50Hz, DIV INT=250
    for sl in range(4):
        key = f'Slice{sl}'  # raw printf output, skip
    # (PWM check via raw output — harder to parse)

    # GPIO CTRL / FUNCSEL key pins
    if (v := reg('GPIO0  CTRL (PWM slice0A, expect 4=PWM)')) is not None:
        check('GPIO0 FUNCSEL (PWM=4)', v, 0x4, mask=0x1f)
    if (v := reg('GPIO12 CTRL (UART0 TX,   expect 2=UART)')) is not None:
        check('GPIO12 FUNCSEL (UART=2)', v, 0x2, mask=0x1f)
    # SCK pins: FUNCSEL=1 (SPI, mid-transaction) or FUNCSEL=5 (SIO, idle) are both correct.
    # FUNCSEL=31 means pico2_gpio_init() never ran and SPI will be silent.
    if (v := reg('GPIO22 CTRL (SPI0 SCK,   expect 1=SPI or 5=SIO-idle)')) is not None:
        fs = v & 0x1f
        ok = (fs == 1 or fs == 5)
        sym = 'PASS' if ok else 'FAIL'
        print(f'  [{sym}] GPIO22 FUNCSEL (SPI=1 or SIO-idle=5): got 0x{fs:x}'
              f'{", idle=correct" if fs==5 else ", in-transaction" if fs==1 else ", NULL=BROKEN"}')
        checks.append(ok)
    if (v := reg('GPIO32 CTRL (SPI0 MISO,  expect 1=SPI)')) is not None:
        check('GPIO32 FUNCSEL (SPI=1)', v, 0x1, mask=0x1f)
    if (v := reg('GPIO42 CTRL (SPI1 SCK,   expect 1=SPI or 5=SIO-idle)')) is not None:
        fs = v & 0x1f
        ok = (fs == 1 or fs == 5)
        sym = 'PASS' if ok else 'FAIL'
        print(f'  [{sym}] GPIO42 FUNCSEL (SPI=1 or SIO-idle=5): got 0x{fs:x}'
              f'{", idle=correct" if fs==5 else ", in-transaction" if fs==1 else ", NULL=BROKEN"}')
        checks.append(ok)

    # UART0
    if (v := reg('GPIO12 FUNCSEL (expect 2=UART)')) is not None:
        check('UART0 GPIO12 FUNCSEL (UART=2)', v, 0x2, mask=0x1f)
    if (v := reg('GPIO13 FUNCSEL (expect 2=UART)')) is not None:
        check('UART0 GPIO13 FUNCSEL (UART=2)', v, 0x2, mask=0x1f)
    if (v := reg('UART0 IBRD')) is not None:
        check('UART0 IBRD (~271=57600baud@250MHz)', v, 271, info='271→57608 baud')
    if (v := reg('UART0 CR')) is not None:
        check('UART0 CR (TX+RX+EN=0x301)', v, 0x301, mask=0x301)

    # SPI0
    if (v := reg('GPIO32 FUNCSEL (MISO, expect 1=SPI)')) is not None:
        check('SPI0 GPIO32 FUNCSEL (SPI=1)', v, 0x1, mask=0x1f)
    if (v := reg('GPIO35 FUNCSEL (MOSI, expect 1=SPI)')) is not None:
        check('SPI0 GPIO35 FUNCSEL (SPI=1)', v, 0x1, mask=0x1f)
    if (v := reg('SPI0 SSPCPSR (clock prescale)')) is not None:
        check('SPI0 SSPCPSR non-zero (SPI clock enabled)', v > 0, True, info=f'val=0x{v:x}')

    # SPI CS pins
    if (v := reg('GPIO_OE  bits[26:23]')) is not None:
        mask_26_23 = 0x7800000  # bits 23-26
        check('SPI CS pins GPIO_OE bits[26:23]=HIGH', v & mask_26_23, mask_26_23,
              info='all 4 CS pins in output mode')
    if (v := reg('GPIO_OUT bits[26:23]')) is not None:
        mask_26_23 = 0x7800000
        check('SPI CS pins GPIO_OUT bits[26:23]=HIGH', v & mask_26_23, mask_26_23,
              info='all 4 CS pins deasserted (HIGH)')

    # ADC
    if (v := reg('ADC_CS (EN=1,TS_EN=1,START_MANY=1)')) is not None:
        check('ADC EN=1', v & 1, 1)
        check('ADC TS_EN=1', (v >> 1) & 1, 1, info='temperature sensor enabled')
        check('ADC START_MANY=1', (v >> 3) & 1, 1, info='free-running mode')

    # Watchdog
    if (v := reg('WATCHDOG_CTRL (ENABLE=bit30, PAUSE_DBG=bits25:24)')) is not None:
        check('Watchdog ENABLE (bit30)', (v >> 30) & 1, 1)
        check('Watchdog PAUSE_DBG (bits[25:24]≠0)', (v >> 24) & 3, 3,
              info='pauses during SWD debug — correct')
    if (v := reg('WATCHDOG_REASON (0=clean boot, 1=WDT reset)')) is not None:
        check('Watchdog REASON=0 (no WDT reset)', v, 0, info='clean power-on boot')

    # OTP
    if (v := reg('OTP row0 (CHIPID0)')) is not None:
        check('OTP CHIPID0 non-zero (factory programmed)', v != 0, True,
              info=f'val=0x{v:08x}')

    total = len(checks)
    passed = sum(checks)
    print(f'\n{passed}/{total} checks passed.')
    if passed == total:
        print('All checks PASSED. ✓')
    else:
        print(f'{total - passed} check(s) FAILED.')


def main():
    parser = argparse.ArgumentParser(description='Pico2 RP2350 hardware register health check')
    parser.add_argument('--port', type=int, default=50000, help='GDB port (default: 50000)')
    parser.add_argument('--elf', default=DEFAULT_ELF, help='Path to ELF for symbol resolution')
    args = parser.parse_args()

    if not os.path.exists(args.elf):
        print(f'Warning: ELF not found at {args.elf} — symbol names will not be resolved')
        args.elf = None

    print(f'Connecting to OpenOCD GDB server on port {args.port}...')
    cmds = build_gdb_script(args.elf)
    output = run_gdb(cmds, args.port)
    parse_and_report(output)


if __name__ == '__main__':
    main()
