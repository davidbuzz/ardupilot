#!/usr/bin/env python3
"""
set_param_ram.py — Write ArduPilot parameter values directly into RAM via
GDB + OpenOCD without rebooting the target.

Background
----------
ArduPilot parameters are C++ objects (AP_Int8, AP_Float, AP_Int16 …) whose
internal ``_value`` member holds the live value that the running code reads.
By writing that member through GDB's „set variable" command we can change
a parameter on the fly — no reboot, no MAVLink connection required.

This is useful when:
  • The board has no GCS connection (pre-arming, USB not working).
  • You want to test a parameter change that normally needs a defaults.parm
    reload or flash erase (e.g. FSTRATE_ENABLE=1 to trigger rate_thread).
  • You need to force a value that the running code would otherwise reject.

Usage
-----
    # Set a single symbol value (GDB expression path):
    python3 Tools/debug/set_param_ram.py copter.g2.att_enable._value=1

    # Set multiple values:
    python3 Tools/debug/set_param_ram.py \\
            copter.g2.att_enable._value=1 \\
            copter.g2.att_decimation._value=4

    # Print current value only (no write):
    python3 Tools/debug/set_param_ram.py --read copter.g2.att_enable._value

    # Use a non-default ELF or GDB port:
    python3 Tools/debug/set_param_ram.py \\
            --elf build/Pico2/bin/arducopter \\
            --port 50000 \\
            copter.g2.att_enable._value=1

    # Halt the CPU while setting, then resume automatically (default):
    python3 Tools/debug/set_param_ram.py copter.g2.att_enable._value=1

    # Leave the CPU halted after the operation (useful for follow-up GDB):
    python3 Tools/debug/set_param_ram.py --no-resume copter.g2.att_enable._value=1

Prerequisites
-------------
  • OpenOCD must be running and connected to the board, listening on the
    GDB port (default 50000).  Start it with:
      ~/openocd-pico/openocd \\
          -c "gdb_port 50000" -c "tcl_port 50001" -c "telnet_port 50002" \\
          -s ~/openocd-pico/scripts \\
          -f interface/cmsis-dap.cfg -f target/rp2350.cfg \\
          -c "adapter speed 5000"

  • arm-none-eabi-gdb must be on your PATH, or pass --gdb to override.

  • The ELF file must match the firmware running on the board (symbols must
    match).  The default path is build/Pico2/bin/arducopter.

Limitations
-----------
  • The board is briefly halted while GDB does the write.  On a running
    system the halt lasts ~50 ms; the scheduler and rate thread will miss
    some cycles but recover immediately on resume.
  • Values written this way survive only until the next reboot.  To make
    a change permanent, either update defaults.parm or use a MAVLink GCS
    to set and save the parameter.
  • GDB symbol names are NOT the same as MAVLink parameter names.  Use
    GDB's ``ptype`` or ``info variables`` commands to discover them, or
    consult the ArduPilot parameter C++ source.
"""
# AP_FLAKE8_CLEAN

import argparse
import os
import re
import subprocess
import sys
import textwrap


# --------------------------------------------------------------------------- #
# Defaults
# --------------------------------------------------------------------------- #
DEFAULT_ELF = "build/Pico2/bin/arducopter"
DEFAULT_GDB = "arm-none-eabi-gdb"
DEFAULT_GDB_HOST = "localhost"
DEFAULT_GDB_PORT = 50000


# --------------------------------------------------------------------------- #
# Helpers
# --------------------------------------------------------------------------- #

def find_elf(path: str) -> str:
    """Return an absolute path to the ELF, searching common build dirs."""
    if os.path.isfile(path):
        return os.path.abspath(path)
    # Try relative to the repo root (script lives in Tools/debug/)
    repo_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    candidate = os.path.join(repo_root, path)
    if os.path.isfile(candidate):
        return candidate
    return path   # return as-is; GDB will report the error


def build_gdb_script_flat(assignments: list, reads: list, host: str, port: int,
                          resume: bool) -> list:
    """Build flat list of -ex args (each -ex takes exactly one command)."""
    cmds = []
    cmds += ["-ex", f"target extended-remote {host}:{port}"]
    cmds += ["-ex", "set confirm off"]

    # Print before values first (reads and symbols about to be written).
    for sym in reads:
        cmds += ["-ex", f"print {sym}"]
    for sym, _ in assignments:
        cmds += ["-ex", f"print {sym}"]

    # Apply writes.
    for sym, val in assignments:
        cmds += ["-ex", f"set variable {sym} = {val}"]

    # Confirm writes.
    for sym, _ in assignments:
        cmds += ["-ex", f"print {sym}"]

    if resume:
        # "detach" resumes the target AND releases the GDB remote connection so
        # GDB exits cleanly in --batch mode.  "continue" would block forever
        # waiting for the next stop event over extended-remote.
        cmds += ["-ex", "detach"]

    return cmds


def parse_assignment(token: str):
    """Parse ``symbol=value`` → (symbol, value).  Raises ValueError on error."""
    if "=" not in token:
        raise ValueError(f"Expected 'symbol=value', got: {token!r}")
    sym, _, val = token.partition("=")
    sym = sym.strip()
    val = val.strip()
    if not sym:
        raise ValueError(f"Empty symbol name in: {token!r}")
    if not val:
        raise ValueError(f"Empty value in: {token!r}")
    return sym, val


def run_gdb(gdb_bin: str, elf: str, extra_args: list, timeout: int = 30) -> str:
    """Run arm-none-eabi-gdb in batch mode and return stdout."""
    cmd = [
        gdb_bin,
        "--batch",   # non-interactive: exit when commands are done
        "--nx",      # do not read ~/.gdbinit (avoids duplicate remote connect)
    ] + extra_args + [elf]

    try:
        result = subprocess.run(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,   # merge stderr so errors surface
            timeout=timeout,
            text=True,
        )
    except FileNotFoundError:
        sys.exit(f"ERROR: GDB binary not found: {gdb_bin!r}\n"
                 "Install arm-none-eabi-gdb or pass --gdb <path>.")
    except subprocess.TimeoutExpired:
        sys.exit(f"ERROR: GDB timed out after {timeout}s.  Is OpenOCD running?")

    return result.stdout


def display_output(raw: str, assignments: list, reads: list) -> bool:
    """Parse and pretty-print GDB output.  Returns True on success."""
    # GDB prints values as:  $1 = <value>
    # We also capture any error lines.
    lines = raw.splitlines()
    value_lines = [l for l in lines if re.match(r'^\$\d+\s*=', l)]
    error_lines = [l for l in lines if re.search(r'error|no symbol|cannot|not found',
                                                   l, re.IGNORECASE)]

    total_reads = len(reads) + len(assignments)  # before reads
    total_confirms = len(assignments)             # after-write reads

    # Map GDB result indices to labels.
    labels = []
    # First batch: pure reads.
    for sym in reads:
        labels.append(("READ  ", sym))
    # Second batch: before-write reads for each assignment symbol.
    for sym, _ in assignments:
        labels.append(("BEFORE", sym))
    # Third batch: after-write confirms.
    for sym, val in assignments:
        labels.append(("AFTER ", f"{sym}  (set to {val})"))

    print()
    for i, vline in enumerate(value_lines):
        if i < len(labels):
            prefix, desc = labels[i]
            # Extract the value portion after "$N = "
            m = re.match(r'^\$\d+\s*=\s*(.*)', vline)
            val_str = m.group(1) if m else vline
            if prefix == "AFTER ":
                print(f"  \033[92m✓ {prefix} {desc} → {val_str}\033[0m")
            else:
                print(f"  {prefix} {desc} = {val_str}")
        else:
            print(f"  {vline}")

    if error_lines:
        print()
        for e in error_lines:
            print(f"  \033[91m⚠ {e}\033[0m")
        print()
        return False

    print()
    return True


# --------------------------------------------------------------------------- #
# Main
# --------------------------------------------------------------------------- #

def main():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description=textwrap.dedent("""\
            Write ArduPilot parameter values into RAM via GDB + OpenOCD.

            Each positional argument is a GDB symbol assignment:
                symbol_path=value
            e.g.:
                copter.g2.att_enable._value=1
                copter.g.sched_loop_rate._value=400

            Use --read to inspect values without writing.
        """),
    )
    parser.add_argument(
        "targets",
        nargs="*",
        metavar="symbol=value",
        help="GDB symbol assignments (e.g. copter.g2.att_enable._value=1)",
    )
    parser.add_argument(
        "--read", "-r",
        nargs="+",
        metavar="symbol",
        default=[],
        help="Print current value of symbol(s) without writing",
    )
    parser.add_argument(
        "--elf",
        default=DEFAULT_ELF,
        metavar="PATH",
        help=f"ELF file matching the running firmware (default: {DEFAULT_ELF})",
    )
    parser.add_argument(
        "--gdb",
        default=DEFAULT_GDB,
        metavar="PATH",
        help=f"arm-none-eabi-gdb binary (default: {DEFAULT_GDB})",
    )
    parser.add_argument(
        "--host",
        default=DEFAULT_GDB_HOST,
        help=f"OpenOCD GDB server host (default: {DEFAULT_GDB_HOST})",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=DEFAULT_GDB_PORT,
        help=f"OpenOCD GDB server port (default: {DEFAULT_GDB_PORT})",
    )
    parser.add_argument(
        "--no-resume",
        action="store_true",
        default=False,
        help="Leave CPU halted after operation (default: resume automatically)",
    )
    parser.add_argument(
        "--timeout",
        type=int,
        default=30,
        metavar="SECS",
        help="GDB subprocess timeout in seconds (default: 30)",
    )
    parser.add_argument(
        "--raw",
        action="store_true",
        default=False,
        help="Print raw GDB output instead of formatted output",
    )

    args = parser.parse_args()

    # Require at least one thing to do.
    if not args.targets and not args.read:
        parser.error("Provide at least one symbol=value assignment or --read symbol.")

    # Parse assignments.
    assignments = []
    for tok in args.targets:
        try:
            sym, val = parse_assignment(tok)
        except ValueError as e:
            parser.error(str(e))
        assignments.append((sym, val))

    # Resolve ELF path.
    elf = find_elf(args.elf)
    if not os.path.isfile(elf):
        print(f"WARNING: ELF not found at {elf!r} — GDB symbol resolution will fail.",
              file=sys.stderr)

    resume = not args.no_resume

    # Build GDB command.
    extra_args = build_gdb_script_flat(
        assignments, args.read, args.host, args.port, resume
    )

    # Announce what we're about to do.
    print(f"Connecting to OpenOCD at {args.host}:{args.port} …")
    if assignments:
        for sym, val in assignments:
            print(f"  Setting {sym} = {val}")
    if args.read:
        for sym in args.read:
            print(f"  Reading {sym}")
    print(f"  ELF: {elf}")
    print(f"  Resume after: {resume}")

    raw_output = run_gdb(args.gdb, elf, extra_args, timeout=args.timeout)

    if args.raw:
        print(raw_output)
        return

    ok = display_output(raw_output, assignments, args.read)

    if not ok:
        print("One or more errors were reported by GDB.  "
              "Check symbol names with: arm-none-eabi-gdb --batch --nx "
              f"-ex \"target extended-remote {args.host}:{args.port}\" "
              "-ex \"info variables att_enable\" "
              f"{elf}")
        sys.exit(1)


if __name__ == "__main__":
    main()
