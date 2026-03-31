#!/usr/bin/env python3
"""
Pico2 startup console capture
==============================
Captures serial output from the ArduPilot Pico2 target from power-on /
reset until the specified duration (default 60 s).  Prints a human-readable
view to stdout AND saves the raw bytes to a timestamped log file.

How it works
------------
* Prefers the stable /dev/serial/by-id/ symlink so the ACM number changing
  on each USB reconnect is not a problem.
* Asserts DTR immediately on open so the ChibiOS SDU driver releases its
  TX hold-off (the board will not send a single byte until DTR is high).
* Distinguishes printable text from binary MAVLink frames in the display:
    - Printable UTF-8 runs are printed as-is (early boot DEV_PRINTF output
      appears here before MAVLink initialises).
    - Binary blobs are summarised as  [42 bin]  so the terminal is not
      flooded with garbage, but the raw bytes are preserved in the .log file.
* MAVLink STATUSTEXT messages (msgid 253, both v1 0xFE and v2 0xFD framing)
  are decoded inline and printed with a  [MSG]  prefix so they stand out.

Usage
-----
    # auto-detect port, 60 s capture
    python3 libraries/AP_HAL_ChibiOS/hwdef/Pico2/capture_startup.py

    # wait for the board to appear (useful after a power-cycle)
    python3 libraries/AP_HAL_ChibiOS/hwdef/Pico2/capture_startup.py --wait

    # explicit port and longer capture
    python3 libraries/AP_HAL_ChibiOS/hwdef/Pico2/capture_startup.py \\
            --port /dev/ttyACM1 --duration 120

    # show raw hex instead of [N bin] for binary chunks
    python3 libraries/AP_HAL_ChibiOS/hwdef/Pico2/capture_startup.py --hex

    # suppress log file
    python3 libraries/AP_HAL_ChibiOS/hwdef/Pico2/capture_startup.py --no-log

Dependencies: pyserial  (pip install pyserial)
"""

import argparse
import glob
import os
import struct
import sys
import time

# ---------------------------------------------------------------------------
# Port discovery
# ---------------------------------------------------------------------------

# Stable by-id path for the ArduPilot Pico2 USB CDC port.
# This does not change when the ACM number is reassigned on reconnect.
BY_ID_GLOB = '/dev/serial/by-id/usb-ArduPilot_Pico2_*-if00'

def _find_port_once():
    """Return the first candidate port path, or None if nothing found."""
    # Prefer the stable symlink
    matches = sorted(glob.glob(BY_ID_GLOB))
    if matches:
        return matches[0]
    # Fall back to ttyACM* — take the highest-numbered one (debugprobe
    # usually gets ACM0; the target gets ACM1 or higher)
    acm = sorted(glob.glob('/dev/ttyACM*'))
    if acm:
        return acm[-1]
    return None


def find_port(wait=False, timeout=120):
    """Find the Pico2 serial port.

    If *wait* is True, polls until the device appears or *timeout* seconds
    elapses.  Used after a power-cycle where the device is briefly absent.
    """
    deadline = time.time() + timeout
    while True:
        p = _find_port_once()
        if p:
            return p
        if not wait or time.time() > deadline:
            return None
        print("  [waiting for device…]", end='\r', flush=True)
        time.sleep(0.2)


# ---------------------------------------------------------------------------
# MAVLink frame parser (minimal — extracts STATUSTEXT only)
# ---------------------------------------------------------------------------

MAVLINK_V1_STX = 0xFE
MAVLINK_V2_STX = 0xFD
STATUSTEXT_MSGID = 253   # same in v1 and v2

# MAVLink v1 STATUSTEXT: severity(1) + text(50) = 51 bytes payload
# MAVLink v2 STATUSTEXT: same layout (no extensions in common usage)

def _try_parse_statustext(frame_bytes):
    """Attempt to decode a raw MAVLink frame as STATUSTEXT.

    Returns the text string on success, or None if it is not a STATUSTEXT.
    *frame_bytes* starts at the STX byte.
    """
    if len(frame_bytes) < 8:
        return None
    stx = frame_bytes[0]
    if stx == MAVLINK_V1_STX:
        # v1: STX LEN SEQ SYS COMP MSGID PAYLOAD… CRC1 CRC2
        if len(frame_bytes) < 8:
            return None
        length = frame_bytes[1]
        msgid = frame_bytes[5]
        if msgid != STATUSTEXT_MSGID:
            return None
        expected_len = 6 + length + 2   # header + payload + 2-byte CRC
        if len(frame_bytes) < expected_len:
            return None
        payload = frame_bytes[6:6 + length]
    elif stx == MAVLINK_V2_STX:
        # v2: STX LEN INC VER SEQ SYS COMP MSGID(3) PAYLOAD… CRC1 CRC2
        if len(frame_bytes) < 12:
            return None
        length = frame_bytes[1]
        msgid = struct.unpack_from('<I', bytes(frame_bytes[7:10]) + b'\x00')[0]
        if msgid != STATUSTEXT_MSGID:
            return None
        expected_len = 10 + length + 2   # header(10) + payload + CRC(2)
        if len(frame_bytes) < expected_len:
            return None
        payload = frame_bytes[10:10 + length]
    else:
        return None

    if len(payload) < 1:
        return None
    # payload[0] = severity (MAV_SEVERITY), payload[1:] = text (null-padded)
    text_bytes = payload[1:] if len(payload) > 1 else b''
    text = text_bytes.rstrip(b'\x00').decode('utf-8', errors='replace').strip()
    return text if text else None


# ---------------------------------------------------------------------------
# Output formatting
# ---------------------------------------------------------------------------

def _print_chunk(data, show_hex, log_fh):
    """Print *data* bytes to stdout (and write raw to log_fh).

    Tries to decode runs of printable bytes as UTF-8 text; summarises
    non-printable runs.  STATUSTEXT frames are decoded inline with [MSG].
    """
    if log_fh:
        log_fh.write(data)
        log_fh.flush()

    if not data:
        return

    # Walk through bytes looking for MAVLink frame starts and printable runs.
    i = 0
    printable_buf = bytearray()

    def flush_printable():
        nonlocal printable_buf
        if printable_buf:
            sys.stdout.write(printable_buf.decode('utf-8', errors='replace'))
            sys.stdout.flush()
            printable_buf = bytearray()

    while i < len(data):
        b = data[i]

        # MAVLink frame start?
        if b in (MAVLINK_V1_STX, MAVLINK_V2_STX) and i + 4 < len(data):
            # Grab enough bytes to attempt parse (max STATUSTEXT frame ~65 B)
            chunk = data[i:i + 256]
            text = _try_parse_statustext(chunk)
            if text is not None:
                flush_printable()
                # Compute frame length to skip past it
                if b == MAVLINK_V1_STX:
                    frame_len = 6 + data[i + 1] + 2
                else:
                    frame_len = 10 + data[i + 1] + 2
                print(f"\033[32m[MSG] {text}\033[0m")  # green
                i += max(frame_len, 1)
                continue
            # Not a STATUSTEXT — treat this byte as binary below
            # (fall through to binary handling)

        # Printable ASCII or common whitespace → accumulate for display
        if 0x20 <= b <= 0x7E or b in (0x09, 0x0A, 0x0D):
            flush_printable()   # flush any pending before switching mode
            printable_buf.append(b)
            i += 1
            continue

        # Binary byte — flush text buffer, then emit summary or hex
        flush_printable()
        # Collect a run of binary bytes
        j = i
        while j < len(data) and not (
            0x20 <= data[j] <= 0x7E or data[j] in (0x09, 0x0A, 0x0D) or
            data[j] in (MAVLINK_V1_STX, MAVLINK_V2_STX)
        ):
            j += 1
        bin_chunk = data[i:j]
        if show_hex:
            hex_str = ' '.join(f'{x:02x}' for x in bin_chunk)
            print(f"\033[90m[{len(bin_chunk):3d} bin: {hex_str[:80]}]\033[0m")
        else:
            print(f"\033[90m[{len(bin_chunk):3d} bin]\033[0m", end='')
        i = j

    flush_printable()


# ---------------------------------------------------------------------------
# Main capture loop
# ---------------------------------------------------------------------------

def capture(port_path, duration, show_hex, log_path):
    try:
        import serial
    except ImportError:
        print("ERROR: pyserial not installed.  Run: pip install pyserial")
        sys.exit(1)

    print(f"Opening {port_path} …", flush=True)
    try:
        ser = serial.Serial(port_path, baudrate=115200, timeout=0.1)
    except serial.SerialException as e:
        print(f"ERROR: could not open {port_path}: {e}")
        sys.exit(1)

    # Assert DTR so ChibiOS SDU driver releases its TX hold-off.
    # Without this the board will not send a single byte.
    ser.dtr = True

    log_fh = None
    if log_path:
        log_fh = open(log_path, 'wb')
        print(f"Logging raw bytes → {log_path}")

    t_start = time.time()
    t_end = t_start + duration
    print(f"Capturing for {duration} s  (Ctrl-C to stop early)\n"
          f"{'─' * 60}", flush=True)

    # Write a timestamp header to the log file for later reference
    if log_fh:
        header = f"# Pico2 startup capture  port={port_path}  duration={duration}s\n"
        header += f"# Started: {time.strftime('%Y-%m-%d %H:%M:%S')}\n"
        log_fh.write(header.encode())

    total_bytes = 0
    try:
        while time.time() < t_end:
            chunk = ser.read(256)   # non-blocking read (timeout=0.1 s)
            if chunk:
                _print_chunk(bytearray(chunk), show_hex, log_fh)
                total_bytes += len(chunk)
    except KeyboardInterrupt:
        print("\n[interrupted]")
    finally:
        elapsed = time.time() - t_start
        print(f"\n{'─' * 60}")
        print(f"Captured {total_bytes} bytes in {elapsed:.1f} s")
        if log_path:
            print(f"Raw log: {log_path}")
        ser.close()
        if log_fh:
            log_fh.close()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description='Capture Pico2 startup console output for ~60 seconds.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument('--port', '-p',
                        help='Serial port to use (default: auto-detect)')
    parser.add_argument('--duration', '-d', type=int, default=60,
                        help='Capture duration in seconds (default: 60)')
    parser.add_argument('--wait', '-w', action='store_true',
                        help='Wait up to 120 s for the device to appear '
                             '(useful when run before power-cycling the board)')
    parser.add_argument('--hex', action='store_true',
                        help='Show hex dump of binary chunks (default: [N bin])')
    parser.add_argument('--no-log', action='store_true',
                        help='Do not write a raw log file')
    args = parser.parse_args()

    # Resolve port
    if args.port:
        port_path = args.port
    else:
        port_path = find_port(wait=args.wait)
        if not port_path:
            print("ERROR: Pico2 not found.  Is the USB cable connected?\n"
                  "       Use --wait to poll until the device appears, or\n"
                  "       specify --port /dev/ttyACMx explicitly.")
            sys.exit(1)
        print(f"Auto-detected port: {port_path}")

    # Build log file path (alongside this script, timestamp in name)
    log_path = None
    if not args.no_log:
        ts = time.strftime('%Y%m%d_%H%M%S')
        script_dir = os.path.dirname(os.path.abspath(__file__))
        log_path = os.path.join(script_dir, f'startup_{ts}.log')

    capture(port_path, args.duration, args.hex, log_path)


if __name__ == '__main__':
    main()
