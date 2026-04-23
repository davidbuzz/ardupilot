#!/usr/bin/env python3
"""
test_uart_pico2.py — UART hardware loopback test for Pico2 / RP2350 ArduPilot port.

Tests SERIAL1 = UART0 (GPIO12 TX / GPIO13 RX) and
      SERIAL2 = UART1 (GPIO10 TX / GPIO11 RX).

Two test modes:

  MODE A — MAVLink SERIAL_CONTROL tunnel (no extra hardware needed):
      Uses the MAVLink SERIAL_CONTROL message to send raw bytes through a
      serial port from the GCS and read them back from the ArduPilot side.
      This only tests the firmware's ability to forward data — it does not
      confirm the physical GPIO pins are toggling.

  MODE B — Physical loopback (wire TX pin to RX pin):
      Wire GPIO12→GPIO13 for UART0, or GPIO10→GPIO11 for UART1, then run
      this script.  A test string is sent via SERIAL_CONTROL and the echo
      is expected back.  This confirms both the TX driver produces pulses
      and the RX driver correctly receives them.

Usage:
    # Mode A (no wiring needed — software test):
    python3 Tools/debug/test_uart_pico2.py --port /dev/ttyACM1

    # Mode B (GPIO12→GPIO13 loopback wire fitted):
    python3 Tools/debug/test_uart_pico2.py --port /dev/ttyACM1 --loopback

Hardware pin reference (Pico2 Pico-W pinout, standard 3.3V):
    UART0_TX = GPIO12 = physical pin 16
    UART0_RX = GPIO13 = physical pin 17
    UART1_TX = GPIO10 = physical pin 14
    UART1_RX = GPIO11 = physical pin 15
    GND              = pin 38 (or any GND pin)

    For Mode B loopback: connect pin 16 → pin 17  (UART0)
                          connect pin 14 → pin 15  (UART1)
    Do NOT do both at the same time without isolating GND currents.

Pre-conditions:
    - Pico2 running ArduPilot (arducopter) firmware with SERIAL1 and SERIAL2
      configured as None or GPS (any protocol is fine for this test,
      SERIAL_CONTROL overrides it temporarily).
    - DEFAULT_SERIAL1_BAUD must be reachable (default 57600 in hwdef).
      The script defaults to 57600 for both ports.
    - BRD_SAFETYENABLE = 0  (recommended — avoids safety-on lockout).
"""

import argparse
import sys
import time
import random
import string

try:
    from pymavlink import mavutil
except ImportError:
    sys.exit("pymavlink not found.  Run: pip install pymavlink")


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

GCS_SYSID  = 255
GCS_COMPID = 0

# MAVLink SERIAL_CONTROL device IDs for direct SERIALn access.
# From pymavlink common.xml: SERIAL_CONTROL_SERIAL0=100, SERIAL1=101, SERIAL2=102.
# Using SERIAL_CONTROL_SERIAL1 (101) and SERIAL_CONTROL_SERIAL2 (102) here;
# TELEM1/TELEM2 (0/1) map through gcs().chan(1/2) which returns nullptr when
# the port protocol is SerialProtocol_None, causing the command to be silently
# ignored instead of calling _begin() on the underlying UART driver.
SERIAL_CTRL_UART0 = 101  # SERIAL_CONTROL_SERIAL1 → SERIAL1 → UART0 (GPIO12/13)
SERIAL_CTRL_UART1 = 102  # SERIAL_CONTROL_SERIAL2 → SERIAL2 → UART1 (GPIO10/11)

TEST_BAUD    = 57600
TIMEOUT_SECS = 2.0       # wait for echo

# Flags for SERIAL_CONTROL (confirmed from pymavlink common.xml).
# CRITICAL: bit 0 (REPLY=1) must NOT be set — the FC handler returns immediately
# when REPLY is set (that flag means the packet is a FC-originated response).
SERIAL_CONTROL_FLAG_REPLY      = 1   # do NOT set this when sending from GCS
SERIAL_CONTROL_FLAG_RESPOND    = 2   # ask FC to send back any data it received
SERIAL_CONTROL_FLAG_EXCLUSIVE  = 4   # lock port exclusively to this SERIAL_CONTROL session
SERIAL_CONTROL_FLAG_BLOCKING   = 8   # FC blocks waiting for data before responding
SERIAL_CONTROL_FLAG_MULTI      = 16  # FC response spans multiple packets


# ---------------------------------------------------------------------------
# Connection helper
# ---------------------------------------------------------------------------

def connect(port: str, baud: int) -> mavutil.mavfile:
    print(f"Connecting to {port} @ {baud} baud ...")
    mav = mavutil.mavlink_connection(port, baud=baud,
                                     source_system=GCS_SYSID,
                                     source_component=GCS_COMPID)
    mav.wait_heartbeat(timeout=10)
    print(f"  Heartbeat sysid={mav.target_system} compid={mav.target_component}")
    return mav


# ---------------------------------------------------------------------------
# SERIAL_CONTROL helpers
# ---------------------------------------------------------------------------

def serial_control_send(mav: mavutil.mavfile, dev: int, baud: int, data: bytes) -> None:
    """Send raw bytes to a serial port on the flight controller via MAVLink."""
    # SERIAL_CONTROL data field is 70 bytes, pad with zeros
    payload = list(data[:70]) + [0] * (70 - len(data))
    # EXCLUSIVE (4) | RESPOND (2) = 6.  Must NOT include REPLY (1) — the FC
    # handler returns immediately if REPLY bit is set (it treats it as a
    # FC-originated echo packet, not a GCS command).
    flags = SERIAL_CONTROL_FLAG_EXCLUSIVE | SERIAL_CONTROL_FLAG_RESPOND
    mav.mav.serial_control_send(
        dev,                          # device (101=SERIAL1, 102=SERIAL2)
        flags,                        # 6 = EXCLUSIVE|RESPOND
        0,                            # timeout (ms)
        baud,                         # baudrate
        len(data),                    # count of valid bytes
        payload                       # data[70]
    )


def serial_control_recv(mav: mavutil.mavfile, deadline: float) -> bytes:
    """
    Collect all SERIAL_CONTROL response packets until deadline.
    Returns the concatenated data bytes received.
    """
    received = b""
    while time.time() < deadline:
        msg = mav.recv_match(type='SERIAL_CONTROL', blocking=True,
                             timeout=0.2)
        if msg is not None and msg.count > 0:
            received += bytes(msg.data[:msg.count])
    return received


# ---------------------------------------------------------------------------
# Test: software loopback (Mode A)
# ---------------------------------------------------------------------------

def test_software_loopback(mav: mavutil.mavfile, dev: int, dev_name: str,
                            baud: int) -> bool:
    """
    Send bytes to a UART port via SERIAL_CONTROL and read back whatever the
    firmware echoes.  Without a physical loopback wire, nothing comes back —
    this test just confirms the firmware accepts the command without crashing.
    """
    print(f"\n  [{dev_name}] Software (no-wire) test @ {baud} baud ...")

    test_str = (b"PICO2UART_TEST_" +
                 ''.join(random.choices(string.ascii_uppercase, k=8)).encode())

    serial_control_send(mav, dev, baud, test_str)

    # Small delay then check for any unsolicited data (e.g. GPS NMEA, etc.)
    received = serial_control_recv(mav, time.time() + 0.5)

    if received:
        print(f"    Received {len(received)} bytes (unexpected data on port — "
              f"port may already be in use by another subsystem).")
    else:
        print(f"    Sent {len(test_str)} bytes, no echo (expected without loopback wire).")

    # Allow the UART port time to fully open.  This also implicitly tests the
    # FUNCSEL fix: before palSetLineMode(PAL_MODE_ALTERNATE_UART) was added in
    # UARTDriver::_begin(), a floating RX line would sit LOW and trigger a
    # continuous UART BREAK IRQ storm that starved the USB ISR and killed the
    # CDC link.  If the FC is still sending heartbeats after begin(), the FUNCSEL
    # and PUE pull-up are working correctly.
    time.sleep(0.5)
    hb = mav.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
    if hb is None:
        print("    ✗  FC stopped sending heartbeats after SERIAL_CONTROL!")
        return False
    print("    ✓  FC alive after SERIAL_CONTROL send.")
    return True


# ---------------------------------------------------------------------------
# Test: physical loopback (Mode B — TX wired to RX)
# ---------------------------------------------------------------------------

def test_physical_loopback(mav: mavutil.mavfile, dev: int, dev_name: str,
                            baud: int) -> bool:
    """
    Send a known test pattern and expect to receive it back via the loopback wire.
    TX pin must be wired to RX pin externally.
    """
    print(f"\n  [{dev_name}] Physical loopback test @ {baud} baud ...")

    # Use a short recognisable string with known framing pattern
    test_payload = b"\xAA\x55" + b"LOOPBACK_OK_PICO2" + b"\x55\xAA"

    serial_control_send(mav, dev, baud, test_payload)
    deadline = time.time() + TIMEOUT_SECS
    received = serial_control_recv(mav, deadline)

    if test_payload in received:
        print(f"    ✓  Echo matched ({len(received)} bytes received).")
        return True
    elif received:
        print(f"    ✗  Received {len(received)} bytes but pattern not found.")
        print(f"       Sent:     {test_payload.hex()}")
        print(f"       Received: {received.hex()}")
        return False
    else:
        print(f"    ✗  No data received. Is TX wired to RX?")
        print(f"       Expected: GPIO10→GPIO11 (UART1) or GPIO12→GPIO13 (UART0)")
        return False


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> int:
    parser = argparse.ArgumentParser(
        description="Pico2 UART hardware test",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM1",
                        help="USB MAVLink port (default: /dev/ttyACM1)")
    parser.add_argument("--baud", type=int, default=115200,
                        help="USB baud (ignored for CDC, default 115200)")
    parser.add_argument("--uart-baud", type=int, default=TEST_BAUD,
                        help=f"Baud rate for the UART under test (default {TEST_BAUD})")
    parser.add_argument("--loopback", action="store_true",
                        help="Run physical loopback test (requires TX→RX wire)")
    args = parser.parse_args()

    mav = connect(args.port, args.baud)

    results = {}

    ports = [
        (SERIAL_CTRL_UART0, "SERIAL1/UART0 (GPIO12-TX, GPIO13-RX)"),
        (SERIAL_CTRL_UART1, "SERIAL2/UART1 (GPIO10-TX, GPIO11-RX)"),
    ]

    for dev, name in ports:
        if args.loopback:
            ok = test_physical_loopback(mav, dev, name, args.uart_baud)
        else:
            ok = test_software_loopback(mav, dev, name, args.uart_baud)
        results[name] = ok

    # -----------------------------------------------------------------------
    # Summary
    # -----------------------------------------------------------------------
    print("\n" + "=" * 55)
    print("UART TEST SUMMARY" + (" — PHYSICAL LOOPBACK" if args.loopback else " — SOFTWARE (no wire)"))
    print("=" * 55)
    any_fail = False
    for name, ok in results.items():
        icon = "✓" if ok else "✗"
        status = "PASS" if ok else "FAIL"
        print(f"  {icon}  {name}: {status}")
        if not ok:
            any_fail = True

    print()
    if any_fail:
        print("RESULT: SOME TESTS FAILED")
        if not args.loopback:
            print("  Re-run with --loopback after wiring TX→RX for definitive test.")
    else:
        print("RESULT: ALL TESTS PASSED")
        if not args.loopback:
            print()
            print("  NOTE: Software test only confirms the FC accepts SERIAL_CONTROL")
            print("  without crashing. For full electrical verification:")
            print("    1. Wire GPIO12 → GPIO13 (UART0 loopback)")
            print("    2. Wire GPIO10 → GPIO11 (UART1 loopback)")
            print("    3. Re-run with --loopback")

    return 1 if any_fail else 0


if __name__ == "__main__":
    sys.exit(main())
