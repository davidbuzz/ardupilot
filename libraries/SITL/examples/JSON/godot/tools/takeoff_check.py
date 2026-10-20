#!/usr/bin/env python3
'''
Fly the ArduPlane autotest suite's FBWA takeoff on the Godot plane and print
the ground roll and climb, to see how a model copes with the suite's stick
inputs: rudder and up elevator from a standstill, then more throttle at 6
and 12 m/s, as in takeoff_in_FBWA() in Tools/autotest/arduplane.py.

  ./takeoff_check.py [--model plane] [--instance 1] [--duration 30]

It starts its own backend and SITL on the given instance, so it can run
beside an autotest on instance 0. Build ArduPlane first.

AP_FLAKE8_CLEAN
'''

import argparse
import time

from pymavlink import mavutil

from sim_pair import SimPair

# RC overrides for each stage of the takeoff, and the groundspeed which ends it
STAGES = [
    ({2: 1200, 3: 1300, 4: 1700}, 6, "moving: throttle 1300, rudder and up elevator"),
    ({2: 1200, 3: 1600, 4: 1500}, 12, "6 m/s: throttle 1600, rudder centred"),
    ({2: 1100, 3: 2000, 4: 1500}, None, "12 m/s: full throttle, more elevator"),
]


def rc_override(mav, channels):
    values = [65535] * 8
    for ch, pwm in channels.items():
        values[ch - 1] = pwm
    mav.mav.rc_channels_override_send(1, 1, *values)


def arm_in_fbwa(mav, timeout=90):
    deadline = time.time() + timeout
    while time.time() < deadline:
        mav.set_mode("FBWA")
        rc_override(mav, {3: 1000})
        mav.arducopter_arm()
        msg = mav.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if (msg is not None and msg.get_srcSystem() == 1 and
                msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            return
    raise RuntimeError("could not arm in FBWA")


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--model", default="plane", help="model in the project's models directory")
    parser.add_argument("--instance", type=int, default=1, help="SITL instance")
    parser.add_argument("--duration", type=float, default=30, help="seconds of wall time to watch after arming")
    args = parser.parse_args()

    with SimPair("plane", model=args.model, instance=args.instance) as sim:
        # RC overrides are accepted from the GCS system id, 255
        mav = mavutil.mavlink_connection(sim.mavlink, source_system=255, retries=60)
        mav.wait_heartbeat()
        mav.mav.request_data_stream_send(1, 1, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
        arm_in_fbwa(mav)
        print("armed in FBWA")
        print(STAGES[0][2])
        state = {}
        stage = 0
        last_ms = 0
        deadline = time.time() + args.duration
        while time.time() < deadline:
            msg = mav.recv_match(blocking=True, timeout=1)
            if msg is None or msg.get_srcSystem() != 1:
                continue
            state[msg.get_type()] = msg
            if msg.get_type() == "STATUSTEXT":
                print("    AP: %s" % msg.text)
            if not all(k in state for k in ("VFR_HUD", "SERVO_OUTPUT_RAW", "ATTITUDE", "GLOBAL_POSITION_INT")):
                continue
            vfr = state["VFR_HUD"]
            servo = state["SERVO_OUTPUT_RAW"]
            att = state["ATTITUDE"]
            pos = state["GLOBAL_POSITION_INT"]
            end_speed = STAGES[stage][1]
            if end_speed is not None and vfr.groundspeed > end_speed:
                stage += 1
                print(STAGES[stage][2])
            rc_override(mav, STAGES[stage][0])
            if att.time_boot_ms - last_ms >= 500:
                last_ms = att.time_boot_ms
                print("t=%6.1f gs=%5.2f thr=%3d servo2=%d servo4=%d pitch=%5.1f roll=%5.1f yaw=%6.1f alt=%6.2f" % (
                    att.time_boot_ms * 1e-3, vfr.groundspeed, vfr.throttle, servo.servo2_raw, servo.servo4_raw,
                    att.pitch * 57.3, att.roll * 57.3, att.yaw * 57.3, pos.relative_alt * 1e-3))


if __name__ == "__main__":
    main()
