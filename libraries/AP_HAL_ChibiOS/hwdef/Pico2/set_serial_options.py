#!/usr/bin/env python3
"""
set_serial_options.py — Set SERIAL*_OPTIONS parameters on Pico2/RP2350 via MAVLink.

Usage:
    python3 set_serial_options.py [--port /dev/ttyACM1] [--serial 1] [--options 8192]

Common option values:
    8192 (bit 13)  OPTION_RTSCTS  — enable hardware RTS/CTS flow control
    1              OPTION_RXINV   — invert RX line (e.g. SBUS)
    2              OPTION_TXINV   — invert TX line
    3              OPTION_RXINV + OPTION_TXINV

After setting, the firmware applies the option immediately via set_options();
no reboot is required for most options.  OPTION_RTSCTS is applied live.

Dependencies:
    pip install pymavlink pyserial
"""

import argparse
import sys
import time
import threading
from pymavlink import mavutil
from serial import SerialException


def parse_args():
    p = argparse.ArgumentParser(description="Set SERIAL*_OPTIONS on Pico2 via MAVLink")
    p.add_argument("--port",    default="/dev/ttyACM1", help="Serial port (default /dev/ttyACM1)")
    p.add_argument("--baud",    type=int, default=115200)
    p.add_argument("--serial",  type=int, default=1,    help="Serial port number (1=SERIAL1, 2=SERIAL2, …)")
    p.add_argument("--options", type=int, default=8192, help="Options bitmask (default 8192 = OPTION_RTSCTS)")
    p.add_argument("--drain-timeout", type=float, default=300.0,
                   help="Max seconds to wait while draining params (default 300)")
    p.add_argument("--progress-interval", type=float, default=1.0,
                   help="Seconds between progress prints during param drain (default 1.0)")
    p.add_argument("--save",    action="store_true",    help="Send PREFLIGHT_STORAGE to write params to flash")
    p.add_argument("--reboot",  action="store_true",    help="Reboot the vehicle after setting params")
    return p.parse_args()


def connect(port: str, baud: int) -> mavutil.mavserial:
    """Open the USB CDC port with DTR asserted (required by ChibiOS SDU driver)."""
    mav = mavutil.mavlink_connection(
        device=port,
        baud=baud,
        source_system=255,
        source_component=190,   # MAV_COMP_ID_MISSIONPLANNER
        dialect="ardupilotmega",
        serial_mode=True,
    )
    # ChibiOS SDU driver will NOT transmit until DTR is high.
    mav.port.dtr = True
    return mav


def wait_for_link(mav: mavutil.mavserial, timeout: float = 12.0) -> bool:
    """Wait until we receive any non-BAD_DATA MAVLink message."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            msg = mav.recv_match(blocking=True, timeout=0.5)
        except SerialException:
            # USB CDC can briefly disappear/re-enumerate during reset.
            return False
        if msg and msg.get_type() != "BAD_DATA":
            return True
    return False


def connect_with_retries(port: str,
                         baud: int,
                         attempts: int = 8,
                         retry_delay: float = 1.0) -> mavutil.mavserial:
    """
    Repeatedly try to open MAVLink and receive first valid packet.

    RP2350 USB CDC can drop momentarily while the target resets; retrying here
    makes the tool resilient instead of failing immediately.
    """
    last_error = None
    for attempt in range(1, attempts + 1):
        print(f"Connect attempt {attempt}/{attempts}…")
        try:
            mav = connect(port, baud)
        except Exception as e:
            last_error = e
            print(f"  open failed: {e}")
            time.sleep(retry_delay)
            continue

        if wait_for_link(mav):
            print("  link up")
            return mav

        print("  no link yet, retrying…")
        try:
            mav.close()
        except Exception:
            pass
        time.sleep(retry_delay)

    raise RuntimeError(f"failed to connect after {attempts} attempts: {last_error}")


def send_heartbeats(mav: mavutil.mavserial, stop_event: threading.Event):
    """Background thread: send GCS HEARTBEAT at 1 Hz."""
    while not stop_event.is_set():
        mav.mav.heartbeat_send(
            type=mavutil.mavlink.MAV_TYPE_GCS,
            autopilot=mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            base_mode=0,
            custom_mode=0,
            system_status=mavutil.mavlink.MAV_STATE_ACTIVE,
        )
        stop_event.wait(1.0)


def decode_pid(pid) -> str:
    """Decode a MAVLink param_id which may be bytes or str."""
    if hasattr(pid, "decode"):
        pid = pid.decode("utf-8", errors="ignore")
    return pid.rstrip("\x00")


def drain_params(mav: mavutil.mavserial,
                 timeout: float = 300.0,
                 progress_interval: float = 1.0) -> dict:
    """
    Download all vehicle parameters from the stream.

    Sends param_request_list and collects PARAM_VALUE messages until the
    stream completes (last index reached) or timeout. Progress is printed
    continuously (default once per second) so the operator can see params
    arriving even when unique param count changes slowly.

    At ~4 params/sec the full 956-param set takes ~4 minutes; timeout
    defaults to 300 s to cover that comfortably.

    Returns {name: (value, type, index)} dict.
    """
    params: dict = {}
    param_count = None
    last_index = -1
    rx_msgs = 0
    t_start = time.time()
    next_progress = t_start + progress_interval

    # Always request a fresh stream from index 0.
    print("  sending param_request_list…")
    mav.mav.param_request_list_send(1, 1)

    deadline = time.time() + timeout
    last_recv = time.time()
    while time.time() < deadline:
        # Long recv timeout to survive the 2-3 s restart gap after request_list.
        msg = mav.recv_match(type="PARAM_VALUE", blocking=True, timeout=5.0)
        if msg is None:
            elapsed_idle = time.time() - last_recv
            if param_count and len(params) >= param_count:
                break  # stream already finished
            if len(params) > 0 and elapsed_idle > 15.0:
                print(f"  stream stalled at {len(params)}/{param_count or '?'} after {elapsed_idle:.0f}s idle")
                break
            continue

        rx_msgs += 1
        pid = decode_pid(msg.param_id)
        params[pid] = (msg.param_value, msg.param_type, msg.param_index)
        param_count = msg.param_count
        last_index = msg.param_index
        last_recv = time.time()

        now = time.time()
        if now >= next_progress or msg.param_index >= msg.param_count - 1:
            elapsed = now - t_start or 0.001
            unique_n = len(params)
            msg_rate = rx_msgs / elapsed
            if param_count:
                pct = (100.0 * unique_n) / param_count
                print(f"  rx={rx_msgs:5d} unique={unique_n:4d}/{param_count} ({pct:5.1f}%) "
                      f"last_idx={last_index:4d} rate={msg_rate:4.1f} msg/s")
            else:
                print(f"  rx={rx_msgs:5d} unique={unique_n:4d} "
                      f"last_idx={last_index:4d} rate={msg_rate:4.1f} msg/s")
            next_progress = now + progress_interval

        if msg.param_index >= msg.param_count - 1:
            print(f"  stream complete in {time.time() - t_start:.1f}s")
            break

    return params


def set_param(mav: mavutil.mavserial, name: str, value: float, retries: int = 3) -> bool:
    """
    Set a parameter and confirm it was accepted.

    Strategy: param_set_send always triggers an immediate PARAM_VALUE echo
    back for just that parameter — we do not need to drain the full 956-param
    stream first (which would take ~4 minutes at 4 params/sec).

    We send the request up to `retries` times and watch for a PARAM_VALUE
    whose param_id matches `name`.  The echo typically arrives within 1-2 s.
    """
    name_bytes = name.encode("ascii").ljust(16, b"\x00")

    for attempt in range(retries):
        if attempt > 0:
            print(f"  retry {attempt}…")

        print(f"  → param_set {name} = {value:.0f}")
        mav.mav.param_set_send(1, 1, name_bytes, value, mavutil.mavlink.MAV_PARAM_TYPE_INT32)

        # The vehicle echoes the param immediately; scan PARAM_VALUEs for a match.
        deadline = time.time() + 8.0
        while time.time() < deadline:
            msg = mav.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
            if msg is None:
                continue
            pid = decode_pid(msg.param_id)
            if pid == name:
                match = "✓" if msg.param_value == value else "✗ MISMATCH"
                print(f"  {match} echo: {pid} = {msg.param_value:.0f}  (expected {value:.0f})")
                return msg.param_value == value

    print(f"  ✗ no echo for {name} after {retries} attempts")
    return False


def main():
    args = parse_args()
    param_name = f"SERIAL{args.serial}_OPTIONS"

    print(f"Connecting to {args.port} at {args.baud} baud…")
    try:
        mav = connect_with_retries(args.port, args.baud)
    except Exception as e:
        print(f"ERROR: cannot open {args.port}: {e}")
        sys.exit(1)

    stop_event = threading.Event()
    hb_thread = threading.Thread(target=send_heartbeats, args=(mav, stop_event), daemon=True)
    hb_thread.start()

    # connect_with_retries() already validated that MAVLink packets are flowing.

    # Drain the full param stream — shows all params arriving, lets the
    # operator see the current value before changing it.
    print(f"Downloading param stream to find {param_name} (can take ~4 min at 4 params/sec)…")
    all_params = drain_params(mav,
                              timeout=args.drain_timeout,
                              progress_interval=args.progress_interval)
    print(f"  Got {len(all_params)} params total.")

    if param_name in all_params:
        cur, _, idx = all_params[param_name]
        print(f"  current {param_name} = {cur:.0f}  (index {idx})")
    else:
        print(f"  WARNING: {param_name} not seen in stream, will set anyway")

    print(f"\nSetting {param_name} = {args.options}…")
    ok = set_param(mav, param_name, float(args.options))

    if args.save:
        print("\nSaving parameters to flash (PREFLIGHT_STORAGE)…")
        mav.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE,
                                  0, 1, 0, 0, 0, 0, 0, 0)
        time.sleep(1.0)

    if args.reboot:
        print("Rebooting vehicle (PREFLIGHT_REBOOT_SHUTDOWN)…")
        mav.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                                  0, 1, 0, 0, 0, 0, 0, 0)
        time.sleep(0.5)

    stop_event.set()
    print(f"\n{'SUCCESS' if ok else 'FAILED'}: {param_name} = {args.options}")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
