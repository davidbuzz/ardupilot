#!/usr/bin/env python3
"""
test_uart_pico2.py — UART loopback test for Pico2 (RP2350) ArduPilot port.

Tests UART0 (SERIAL1, GPIO12/13) and UART1 (SERIAL2, GPIO10/11) via the
SERIAL_CONTROL MAVLink message in MODE_EXCLUSIVE | MODE_NOFORWARD.

Usage
-----
  # Wire GPIO12 → GPIO13 (UART0 loopback), GPIO10 → GPIO11 (UART1 loopback).
  # Connect Pico2 USB to /dev/ttyACM1 (or pass --port).
  # Run:
  python3 test_uart_pico2.py --loopback

Arguments
---------
  --port PORT       Serial device to use (default: /dev/ttyACM1)
  --baud BAUD       MAVLink baud (default: 115200)
  --loopback        Run the loopback test (required to actually test)
  --uart {0,1,all}  Which UART to test (default: all)

Requirements
------------
  pip install pymavlink pyserial
"""

import argparse
import sys
import time
import os
import random
import string

try:
    from pymavlink import mavutil
except ImportError:
    print("pymavlink not found. Install with: pip install pymavlink")
    sys.exit(1)


# ---------------------------------------------------------------------------
# MAVLink SERIAL_CONTROL port numbers
# ---------------------------------------------------------------------------
SERIAL_CONTROL_PORT_UART0 = 0   # SERIAL1 on ArduPilot = UART0 on Pico2
SERIAL_CONTROL_PORT_UART1 = 1   # SERIAL2 on ArduPilot = UART1 on Pico2

SERIAL_CONTROL_FLAG_EXCLUSIVE  = (1 << 4)  # MAV_SERIAL_CONTROL_FLAG_EXCLUSIVE
SERIAL_CONTROL_FLAG_RESPOND    = (1 << 1)  # MAV_SERIAL_CONTROL_FLAG_RESPOND
SERIAL_CONTROL_FLAG_BLOCKING   = (1 << 3)  # MAV_SERIAL_CONTROL_FLAG_BLOCKING

BAUD_57600 = 57600


def connect(port, baud):
    print(f"Connecting to {port} at {baud} baud …")
    mav = mavutil.mavlink_connection(port, baud=baud, source_system=255, source_component=190)
    mav.wait_heartbeat(timeout=10)
    print(f"  Heartbeat from sysid={mav.target_system} compid={mav.target_component}")
    return mav


def serial_control_send(mav, port_id, data_bytes, flags=0, timeout_ms=100, baud=BAUD_57600):
    """Send a SERIAL_CONTROL message with up to 70 bytes of data."""
    assert len(data_bytes) <= 70, "SERIAL_CONTROL payload limited to 70 bytes"
    payload = list(data_bytes) + [0] * (70 - len(data_bytes))
    mav.mav.serial_control_send(
        port_id,
        flags | SERIAL_CONTROL_FLAG_RESPOND,
        timeout_ms,
        baud,
        len(data_bytes),
        payload,
    )


def serial_control_recv(mav, port_id, timeout=2.0):
    """Receive a SERIAL_CONTROL message on the given port, return bytes or None."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = mav.recv_match(type='SERIAL_CONTROL', blocking=True, timeout=0.2)
        if msg and msg.port == port_id and msg.count > 0:
            return bytes(msg.data[:msg.count])
    return None


def flush_port(mav, port_id):
    """Read and discard any pending SERIAL_CONTROL data on the port."""
    deadline = time.time() + 0.5
    while time.time() < deadline:
        msg = mav.recv_match(type='SERIAL_CONTROL', blocking=False)
        if msg is None:
            break


def test_loopback(mav, port_id, port_name, baud=BAUD_57600):
    """Run a loopback test on one UART port. Returns True on pass."""
    print(f"\n--- {port_name} loopback test (baud={baud}) ---")

    flush_port(mav, port_id)

    # Generate a unique test payload
    test_data = (''.join(random.choices(string.ascii_letters + string.digits, k=32))).encode()
    print(f"  Sending  : {test_data!r}")

    serial_control_send(mav, port_id, test_data, baud=baud)

    # Allow time for TX → loopback wire → RX
    time.sleep(0.2)

    received = serial_control_recv(mav, port_id, timeout=2.0)

    if received is None:
        print(f"  FAIL: no response received (timeout)")
        return False

    print(f"  Received : {received!r}")

    if received == test_data:
        print(f"  PASS: data matches ✓")
        return True
    elif test_data in received or received in test_data:
        print(f"  PARTIAL: data partially matched (framing issue?)")
        return False
    else:
        print(f"  FAIL: data mismatch")
        return False


def run_loopback_tests(port, baud, which_uart):
    mav = connect(port, baud)

    results = {}

    if which_uart in ('0', 'all'):
        results['UART0 (SERIAL1, GPIO12/13)'] = test_loopback(
            mav, SERIAL_CONTROL_PORT_UART0, 'UART0 (SERIAL1, GPIO12/13)'
        )

    if which_uart in ('1', 'all'):
        results['UART1 (SERIAL2, GPIO10/11)'] = test_loopback(
            mav, SERIAL_CONTROL_PORT_UART1, 'UART1 (SERIAL2, GPIO10/11)'
        )

    print("\n=== Results ===")
    all_pass = True
    for name, ok in results.items():
        status = "PASS ✓" if ok else "FAIL ✗"
        print(f"  {name}: {status}")
        if not ok:
            all_pass = False

    if all_pass:
        print("\nAll tests PASSED.")
        return 0
    else:
        print("\nSome tests FAILED.")
        return 1


def main():
    parser = argparse.ArgumentParser(description='Pico2 UART loopback test via MAVLink SERIAL_CONTROL')
    parser.add_argument('--port', default='/dev/ttyACM1', help='Serial device (default: /dev/ttyACM1)')
    parser.add_argument('--baud', type=int, default=115200, help='MAVLink connection baud (default: 115200)')
    parser.add_argument('--loopback', action='store_true', help='Run loopback test (requires GPIO12→13, GPIO10→11 wires)')
    parser.add_argument('--uart', choices=['0', '1', 'all'], default='all', help='Which UART to test (default: all)')
    args = parser.parse_args()

    if not args.loopback:
        print("Specify --loopback to run the test.")
        print("Hardware required:")
        print("  Wire GPIO12 → GPIO13  (UART0 TX→RX loopback)")
        print("  Wire GPIO10 → GPIO11  (UART1 TX→RX loopback)")
        sys.exit(0)

    sys.exit(run_loopback_tests(args.port, args.baud, args.uart))


if __name__ == '__main__':
    main()
