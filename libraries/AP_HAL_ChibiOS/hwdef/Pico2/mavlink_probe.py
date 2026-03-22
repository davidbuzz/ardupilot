#!/usr/bin/env python3
"""
mavlink_probe.py — MAVLink USB CDC probe for RP2350/Pico2 bring-up

Usage:
    python3 mavlink_probe.py [--port /dev/ttyACM1] [--baud 115200]

What this does:
  1. Opens the USB CDC port with DTR=True asserted.
     ChibiOS SDU driver will NOT transmit unless DTR is high — this
     is the most common reason ttyACM1 appears but produces zero bytes.
  2. Sends MAVLink HEARTBEAT packets (GCS role, sysid=255, compid=190)
     at 1 Hz so ArduPilot's GCS routing learns the GCS sysid and
     enables full status streaming on this connection.
  3. Prints every MAVLink packet received, decoded where possible.
  4. Reports a summary line every 5 s so you can see if ArduPilot is
     sending anything at all.

Dependencies:
    pip install pymavlink pyserial
"""

import argparse
import sys
import time
import threading
from pymavlink import mavutil

def parse_args():
    p = argparse.ArgumentParser(description="MAVLink USB probe for Pico2/RP2350")
    p.add_argument("--port",  default="/dev/ttyACM1", help="Serial port (default /dev/ttyACM1)")
    p.add_argument("--baud",  type=int, default=115200, help="Baud rate (default 115200, ignored for USB CDC)")
    p.add_argument("--sysid", type=int, default=255,    help="This GCS system ID (default 255)")
    p.add_argument("--quiet", action="store_true",      help="Only print summary lines, not every packet")
    return p.parse_args()

def sender_thread(mav, sysid: int, stop_event: threading.Event):
    """Send HEARTBEAT at 1 Hz until stop_event is set."""
    while not stop_event.is_set():
        mav.mav.heartbeat_send(
            type       = mavutil.mavlink.MAV_TYPE_GCS,
            autopilot  = mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            base_mode  = 0,
            custom_mode= 0,
            system_status = mavutil.mavlink.MAV_STATE_ACTIVE,
        )
        stop_event.wait(1.0)

def main():
    args = parse_args()

    print(f"Connecting MAVLink on {args.port} (baud={args.baud}, GCS sysid={args.sysid})...")

    # Let mavutil own the port — opening it manually first causes a double-open
    # hang. Set DTR immediately after mavutil opens it.
    try:
        mav = mavutil.mavlink_connection(
            device   = args.port,
            baud     = args.baud,
            source_system  = args.sysid,
            source_component = 190,   # MAV_COMP_ID_MISSIONPLANNER
            dialect  = "ardupilotmega",
            serial_mode = True,
        )
    except Exception as e:
        print(f"ERROR: cannot open {args.port}: {e}")
        sys.exit(1)

    # Assert DTR=True. ChibiOS SDU driver keeps its TX path disabled until DTR is high.
    mav.port.dtr = True
    print(f"Port open. DTR asserted.")

    stop_event = threading.Event()
    t = threading.Thread(target=sender_thread, args=(mav, args.sysid, stop_event), daemon=True)
    t.start()

    print("Sending HEARTBEAT at 1 Hz. Waiting for packets from ArduPilot...")
    print("Press Ctrl-C to stop.\n")

    rx_count   = 0
    hb_count   = 0
    last_summary = time.time()

    try:
        while True:
            msg = mav.recv_match(blocking=True, timeout=0.5)
            if msg is None:
                # timeout — still alive, just nothing received
                pass
            else:
                rx_count += 1
                if msg.get_type() == "HEARTBEAT":
                    hb_count += 1
                if not args.quiet:
                    ts = time.strftime("%H:%M:%S")
                    print(f"[{ts}] {msg.get_type()}: {msg}")

            now = time.time()
            if now - last_summary >= 5.0:
                print(f"--- 5s summary: rx={rx_count} pkts, heartbeats={hb_count} ---")
                rx_count = hb_count = 0
                last_summary = now

    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        stop_event.set()
        mav.close()

if __name__ == "__main__":
    main()
