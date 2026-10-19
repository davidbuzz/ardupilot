#!/usr/bin/env python3
'''
Smoke test for a copter flying on the Godot physics backend: arm in GUIDED,
take off, hold altitude, fly 20m north, then land and wait for disarm.

Start SITL (with --model JSON:127.0.0.1) and the Godot backend first, then:
  ./test_flight.py --connect tcp:127.0.0.1:5760

AP_FLAKE8_CLEAN
'''

import argparse
import math
import sys
import time

from pymavlink import mavutil

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--connect", default="tcp:127.0.0.1:5760", help="MAVLink connection string")
parser.add_argument("--alt", type=float, default=10.0, help="takeoff altitude (m)")
parser.add_argument("--timeout", type=float, default=300.0, help="overall timeout (s)")
args = parser.parse_args()

state = {}
failures = []


def check(ok, what):
    print("%s: %s" % ("PASS" if ok else "FAIL", what))
    if not ok:
        failures.append(what)


def pump(mav, seconds):
    '''process incoming messages for a while, keeping the latest of each type'''
    end = time.time() + seconds
    while time.time() < end:
        m = mav.recv_match(blocking=True, timeout=0.1)
        if m is None:
            continue
        t = m.get_type()
        state[t] = m
        if t == "STATUSTEXT":
            print("  AP: %s" % m.text)


def wait_until(mav, cond, timeout, what):
    end = time.time() + timeout
    while time.time() < end:
        pump(mav, 0.2)
        if cond():
            return True
    print("timed out waiting for %s" % what)
    return False


def armed():
    hb = state.get("HEARTBEAT")
    return hb is not None and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0


def rel_alt():
    gpi = state.get("GLOBAL_POSITION_INT")
    return gpi.relative_alt * 0.001 if gpi else 0.0


def local_ne():
    lp = state.get("LOCAL_POSITION_NED")
    return (lp.x, lp.y) if lp else (0.0, 0.0)


mav = mavutil.mavlink_connection(args.connect, source_system=255)
mav.wait_heartbeat(timeout=60)
print("connected to system %u" % mav.target_system)
mav.mav.request_data_stream_send(mav.target_system, mav.target_component,
                                 mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
deadline = time.time() + args.timeout

# arming succeeds once the EKF has converged and GPS has a fix
while not armed():
    if time.time() > deadline:
        check(False, "arm")
        sys.exit(1)
    mav.set_mode("GUIDED")
    mav.mav.command_long_send(mav.target_system, mav.target_component,
                              mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    pump(mav, 2)
check(True, "armed in GUIDED")

mav.mav.command_long_send(mav.target_system, mav.target_component,
                          mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, args.alt)
check(wait_until(mav, lambda: rel_alt() > args.alt - 0.5, 60, "takeoff"), "takeoff to %.0fm" % args.alt)

# hold position and record how steady it is
alts = []
tilt = 0.0
end = time.time() + 10
while time.time() < end:
    pump(mav, 0.1)
    alts.append(rel_alt())
    att = state.get("ATTITUDE")
    if att:
        tilt = max(tilt, math.degrees(max(abs(att.roll), abs(att.pitch))))
print("hover: alt %.2f..%.2fm, max tilt %.1f deg" % (min(alts), max(alts), tilt))
check(max(alts) - min(alts) < 1.0 and abs(sum(alts) / len(alts) - args.alt) < 1.0, "altitude hold within 1m")
check(tilt < 5.0, "hover tilt under 5 degrees")

# fly 20m north at the same altitude
n0, e0 = local_ne()
mav.mav.set_position_target_local_ned_send(
    0, mav.target_system, mav.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
    0b0000110111111000, n0 + 20, e0, -args.alt, 0, 0, 0, 0, 0, 0, 0, 0)
arrived = wait_until(mav, lambda: math.hypot(local_ne()[0] - n0 - 20, local_ne()[1] - e0) < 1.0, 60,
                     "position target")
check(arrived, "flew 20m north to within 1m")

mav.set_mode("LAND")
check(wait_until(mav, lambda: not armed(), 90, "landing and disarm"), "landed and disarmed")

print("%d failure(s)" % len(failures))
sys.exit(1 if failures else 0)
