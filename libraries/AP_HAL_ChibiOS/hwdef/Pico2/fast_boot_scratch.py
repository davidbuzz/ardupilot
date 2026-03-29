#!/usr/bin/env python3
"""
fast_boot_scratch.py -- request a clean RP2350 bootloader handoff into app.

This talks to a running OpenOCD telnet server and writes the RP2350
bootloader scratch flag value that ArduPilot's Pico2 bootloader watches
for. It is useful after SWD programming when the target has reset back
into the bootloader USB port instead of starting the freshly flashed app.

Usage:
  # Start OpenOCD first, then run:
  python3 fast_boot_scratch.py

  # Non-default telnet port:
  python3 fast_boot_scratch.py --port 57002

  # Write a different scratch value without resetting the target:
  python3 fast_boot_scratch.py --value 0xB007CA11 --no-reset
"""

import argparse
import re
import socket
import sys
import time


DEFAULT_HOST = "127.0.0.1"
DEFAULT_TELNET_PORT = 50002
DEFAULT_SCRATCH_ADDR = 0x400D8010
DEFAULT_SCRATCH_VALUE = 0xB007CAFE


def parse_args():
    parser = argparse.ArgumentParser(
        description="Write the RP2350 fast-boot scratch flag via OpenOCD telnet"
    )
    parser.add_argument(
        "--host",
        default=DEFAULT_HOST,
        help=f"OpenOCD telnet host (default: {DEFAULT_HOST})",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=DEFAULT_TELNET_PORT,
        help=f"OpenOCD telnet port (default: {DEFAULT_TELNET_PORT})",
    )
    parser.add_argument(
        "--address",
        type=lambda value: int(value, 0),
        default=DEFAULT_SCRATCH_ADDR,
        help=f"Scratch register address (default: 0x{DEFAULT_SCRATCH_ADDR:08X})",
    )
    parser.add_argument(
        "--value",
        type=lambda value: int(value, 0),
        default=DEFAULT_SCRATCH_VALUE,
        help=f"Scratch value to write (default: 0x{DEFAULT_SCRATCH_VALUE:08X})",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=2.0,
        help="Socket timeout in seconds (default: 2.0)",
    )
    parser.add_argument(
        "--no-halt",
        action="store_true",
        help="Skip the initial halt command before writing scratch",
    )
    parser.add_argument(
        "--no-reset",
        action="store_true",
        help="Do not issue reset run after writing the scratch flag",
    )
    parser.add_argument(
        "--target",
        default="rp2350.dap.core0",
        help="OpenOCD target name to select before writing scratch",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Print raw OpenOCD command responses",
    )
    return parser.parse_args()


class OpenOCDTelnet:
    def __init__(self, host: str, port: int, timeout: float, verbose: bool):
        self._host = host
        self._port = port
        self._timeout = timeout
        self._verbose = verbose
        self._sock = None

    def __enter__(self):
        self._sock = socket.create_connection((self._host, self._port), timeout=self._timeout)
        self._sock.settimeout(0.2)
        banner = self._read_until_prompt()
        if self._verbose and banner:
            print(banner, end="")
        return self

    def __exit__(self, exc_type, exc, tb):
        if self._sock is not None:
            try:
                self.command("exit", expect_prompt=False)
            except OSError:
                pass
            self._sock.close()
            self._sock = None

    def _read_until_prompt(self) -> str:
        deadline = time.time() + self._timeout
        chunks = []
        while time.time() < deadline:
            try:
                data = self._sock.recv(4096)
            except socket.timeout:
                continue
            if not data:
                break
            chunks.append(data)
            joined = b"".join(chunks)
            if joined.endswith(b"> ") or joined.endswith(b"\n> ") or joined.endswith(b">"):
                break
        return b"".join(chunks).decode("utf-8", errors="replace")

    def command(self, text: str, expect_prompt: bool = True) -> str:
        self._sock.sendall(text.encode("ascii") + b"\n")
        if not expect_prompt:
            return ""
        response = self._read_until_prompt()
        if self._verbose and response:
            print(response, end="")
        return response


def ensure_no_error(response: str, command: str):
    lowered = response.lower()
    if "error:" in lowered or "failed" in lowered:
        raise RuntimeError(f"OpenOCD reported a failure for '{command}':\n{response.strip()}")


def parse_mdw_value(response: str) -> int:
    match = re.search(r"0x[0-9a-fA-F]+:\s*([0-9a-fA-F]+)", response)
    if match is None:
        raise RuntimeError(f"Could not parse mdw response:\n{response.strip()}")
    return int(match.group(1), 16)


def main():
    args = parse_args()

    print(
        f"Connecting to OpenOCD telnet on {args.host}:{args.port} and writing "
        f"0x{args.value:08X} to 0x{args.address:08X}"
    )

    try:
        with OpenOCDTelnet(args.host, args.port, args.timeout, args.verbose) as session:
            target_response = session.command(f"targets {args.target}")
            ensure_no_error(target_response, f"targets {args.target}")

            if not args.no_halt:
                halt_response = session.command("halt")
                ensure_no_error(halt_response, "halt")

            write_response = session.command(f"mww 0x{args.address:08X} 0x{args.value:08X}")
            ensure_no_error(write_response, "mww")

            readback_response = session.command(f"mdw 0x{args.address:08X}")
            ensure_no_error(readback_response, "mdw")
            if parse_mdw_value(readback_response) != args.value:
                raise RuntimeError(
                    "Scratch register readback did not match the requested value:\n"
                    f"{readback_response.strip()}"
                )

            if args.no_reset:
                print("Scratch value written and verified. Target left halted/running as requested.")
            else:
                reset_response = session.command("reset run")
                ensure_no_error(reset_response, "reset run")
                print("Scratch value written, verified, and target reset into fast-boot handoff.")

    except (OSError, RuntimeError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())