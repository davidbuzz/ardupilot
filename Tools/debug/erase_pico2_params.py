#!/usr/bin/env python3
"""
erase_pico2_params.py — Erase the Pico2 / RP2350 ArduPilot parameter flash
sectors via SWD / OpenOCD telnet.

Background
----------
On the Pico2 the 4 MB QSPI flash is laid out as:

    0x10000000 – 0x10007FFF   32 KB  Bootloader
    0x10008000 – 0x1000FFFF   32 KB  Parameter storage (AP_FlashStorage)
    0x10010000 – 0x103FFFFF 4032 KB  Application firmware

ArduPilot reads saved parameters from the storage region on every boot.
Erasing it forces ArduPilot to fall back to the compiled-in defaults (from
defaults.parm embedded in the ROMFS).  This is useful when:

  • Updating firmware with a new default loop-rate (SCHED_LOOP_RATE) or other
    compile-time-overridden defaults that must survive a parameter reset.
  • Recovering a board stuck in a bad parameter state.
  • Clearing accumulated cruft (e.g. old compass calibration) before testing.

Prerequisites
-------------
  • OpenOCD must already be running and connected to the Pico2 via the
    DebugProbe / CMSIS-DAP adapter, listening on its telnet port.
  • Start OpenOCD with:
      ~/openocd-pico/openocd \\
          -c "gdb_port 50000" -c "tcl_port 50001" -c "telnet_port 50002" \\
          -s ~/openocd-pico/scripts \\
          -f interface/cmsis-dap.cfg -f target/rp2350.cfg \\
          -c "adapter speed 5000"

Usage
-----
    # Erase params using defaults (OpenOCD at localhost:50002):
    python3 Tools/debug/erase_pico2_params.py

    # Erase without resuming (leave CPU halted for GDB):
    python3 Tools/debug/erase_pico2_params.py --no-resume

    # Custom OpenOCD host/port:
    python3 Tools/debug/erase_pico2_params.py --host 192.168.1.5 --port 50002

    # Override the flash region (address and size in hex or decimal):
    python3 Tools/debug/erase_pico2_params.py --addr 0x10008000 --size 0x8000
"""
# AP_FLAKE8_CLEAN

import argparse
import socket
import sys
import time


# --------------------------------------------------------------------------- #
# Pico2 flash layout constants (matches hwdef.dat)
# --------------------------------------------------------------------------- #
DEFAULT_PARAM_ADDR = 0x10008000   # start of parameter storage region
DEFAULT_PARAM_SIZE = 0x00008000   # 32 KB (two 16 KB AP_FlashStorage halves)

OPENOCD_HOST = "localhost"
OPENOCD_TELNET_PORT = 50002


def openocd_command(sock: socket.socket, cmd: str, timeout: float = 10.0) -> str:
    """Send a single OpenOCD telnet command and return the response text.

    OpenOCD telnet doesn't use a real telnet control protocol on this
    interface — it is a plain TCP stream with a prompt character sequence.
    We send the command followed by a newline and wait for the prompt
    ("> ") to signal end-of-response.
    """
    payload = (cmd + "\n").encode()
    sock.sendall(payload)

    # Accumulate response until we see the ">" prompt that ends each reply.
    response = b""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            sock.settimeout(0.5)
            chunk = sock.recv(4096)
        except socket.timeout:
            chunk = b""
        response += chunk
        # OpenOCD telnet prompt is "> " or just ">" after a response.
        if b"> " in response or response.endswith(b">"):
            break
    return response.decode(errors="replace")


def connect(host: str, port: int, timeout: float = 5.0) -> socket.socket:
    """Open a plain TCP connection to the OpenOCD telnet interface."""
    sock = socket.create_connection((host, port), timeout=timeout)
    # Drain the banner/greeting that OpenOCD sends on connect.
    sock.settimeout(2.0)
    try:
        sock.recv(4096)
    except socket.timeout:
        pass
    return sock


def do_erase(host: str, port: int, addr: int, size: int, resume: bool) -> bool:
    """Halt the target, erase the parameter region, optionally resume.

    Returns True on apparent success, False if an error was detected.
    """
    print(f"Connecting to OpenOCD at {host}:{port} …")
    try:
        sock = connect(host, port)
    except (ConnectionRefusedError, OSError) as exc:
        print(f"ERROR: Cannot connect to OpenOCD: {exc}")
        print("  Is OpenOCD running?  Start it with:")
        print("    ~/openocd-pico/openocd \\")
        print("        -c \"gdb_port 50000\" -c \"tcl_port 50001\" -c \"telnet_port 50002\" \\")
        print("        -s ~/openocd-pico/scripts \\")
        print("        -f interface/cmsis-dap.cfg -f target/rp2350.cfg \\")
        print("        -c \"adapter speed 5000\"")
        return False

    ok = True
    try:
        # Halt both cores so the flash controller is idle.
        print("  Halting target …")
        resp = openocd_command(sock, "halt")
        if "error" in resp.lower() or "failed" in resp.lower():
            print(f"  WARNING: halt response: {resp.strip()!r}")

        # Erase the parameter storage region.
        erase_cmd = f"flash erase_address 0x{addr:08x} 0x{size:x}"
        print(f"  Erasing: {erase_cmd}")
        resp = openocd_command(sock, erase_cmd, timeout=30.0)
        print(f"  OpenOCD says: {resp.strip()!r}")

        # Detect failure strings in the response.
        lower = resp.lower()
        if "error" in lower or "failed" in lower or "invalid" in lower:
            print("ERROR: Flash erase appears to have failed (see response above).")
            ok = False
        else:
            print(f"  OK — {size // 1024} KB at 0x{addr:08x} erased.")

        if resume:
            print("  Resuming target …")
            openocd_command(sock, "resume")
            print("  Target running.")
        else:
            print("  Target left halted (--no-resume specified).")

    finally:
        sock.close()

    return ok


def parse_addr(s: str) -> int:
    """Accept hex (0x…) or decimal integer strings."""
    return int(s, 0)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Erase Pico2/RP2350 ArduPilot parameter flash via OpenOCD telnet.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "--host", default=OPENOCD_HOST,
        help=f"OpenOCD telnet host (default: {OPENOCD_HOST})",
    )
    parser.add_argument(
        "--port", type=int, default=OPENOCD_TELNET_PORT,
        help=f"OpenOCD telnet port (default: {OPENOCD_TELNET_PORT})",
    )
    parser.add_argument(
        "--addr", type=parse_addr, default=DEFAULT_PARAM_ADDR,
        help=f"Start address of parameter flash region (default: 0x{DEFAULT_PARAM_ADDR:08x})",
    )
    parser.add_argument(
        "--size", type=parse_addr, default=DEFAULT_PARAM_SIZE,
        help=f"Size in bytes of parameter flash region (default: 0x{DEFAULT_PARAM_SIZE:x} = {DEFAULT_PARAM_SIZE // 1024} KB)",
    )
    parser.add_argument(
        "--no-resume", dest="resume", action="store_false", default=True,
        help="Leave the CPU halted after erasing (useful if GDB is about to connect)",
    )
    args = parser.parse_args()

    print(f"Pico2 parameter flash erase")
    print(f"  Region : 0x{args.addr:08x} – 0x{args.addr + args.size - 1:08x} ({args.size // 1024} KB)")
    print(f"  OpenOCD: {args.host}:{args.port}")
    print(f"  Resume : {'yes' if args.resume else 'no (--no-resume)'}")
    print()

    success = do_erase(args.host, args.port, args.addr, args.size, args.resume)
    if success:
        print()
        print("Done.  On next boot ArduPilot will load compiled-in defaults")
        print("(from defaults.parm embedded in firmware ROMFS).")
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
