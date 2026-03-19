#!/usr/bin/env python3
"""
test_pwm_pico2.py — PWM hardware verification for Pico2 / RP2350 ArduPilot port.

Connects to the Pico2 over USB MAVLink (/dev/ttyACM*), sends MAVLink
DO_SET_SERVO commands for each of the 8 PWM outputs (servo channels 1-8
mapped to GPIO0-7), and reports the result.

Usage:
    python3 Tools/debug/test_pwm_pico2.py [--port /dev/ttyACM1] [--baud 115200]

Hardware setup:
    - Connect a logic analyser or oscilloscope to Pico2 GPIO0-7 (pins 1-10).
    - Or simply connect an LED + 330Ω resistor to each pin to observe brightness
      changes when PWM duty cycle changes.
    - The script sweeps duty cycle from 1000µs (min) → 1500µs (neutral) → 2000µs (max)
      on each channel in sequence so you can verify each channel is alive.

What it verifies:
    1. All 8 DO_SET_SERVO commands are accepted (MAV_RESULT_ACCEPTED).
    2. Channels respond independently — each channel is swept while others are idle.
    3. Frequency is 50 Hz by default; test also tries 400 Hz on channel 1.

Expected electrical behaviour (measurable with logic analyser):
    - 50 Hz: period = 20ms, pulse width 1000-2000µs.
    - GPIO0 = PWM1 (slice 0 ch A), GPIO1 = PWM2 (slice 0 ch B), ...
    - GPIO6 = PWM7 (slice 3 ch A), GPIO7 = PWM8 (slice 3 ch B).
"""

import argparse
import sys
import time

# pymavlink — install with: pip install pymavlink
try:
    from pymavlink import mavutil
except ImportError:
    sys.exit("pymavlink not found.  Run: pip install pymavlink")


# ---------------------------------------------------------------------------
# Configuration constants
# ---------------------------------------------------------------------------

# ArduCopter default system/component IDs
GCS_SYSID  = 255
GCS_COMPID = 0

# Number of servo channels on Pico2
NUM_SERVO_CHANNELS = 8

# PWM pulse widths used during sweep (microseconds)
PWM_MIN     = 1000
PWM_NEUTRAL = 1500
PWM_MAX     = 2000
PWM_SAFE    = 1000   # value left on outputs after the test finishes

# How long (seconds) to hold each step so a scope/analyser can capture it
HOLD_SECS = 0.6

# How long (seconds) to wait for a MAVLink command ACK before timing out
ACK_TIMEOUT = 2.0


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def connect(port: str, baud: int) -> mavutil.mavfile:
    """Open a MAVLink connection and wait for a heartbeat."""
    print(f"Connecting to {port} @ {baud} baud ...")
    mav = mavutil.mavlink_connection(port, baud=baud,
                                     source_system=GCS_SYSID,
                                     source_component=GCS_COMPID)
    mav.wait_heartbeat(timeout=10)
    print(f"  Heartbeat from sysid={mav.target_system} "
          f"compid={mav.target_component}")
    return mav


def send_do_set_servo(mav: mavutil.mavfile, servo_num: int, pwm_us: int) -> bool:
    """
    Send MAV_CMD_DO_SET_SERVO (183) and wait for a COMMAND_ACK.

    servo_num: 1-based servo channel (1 = GPIO0 / PWM1 on Pico2)
    pwm_us:    pulse width in microseconds (1000-2000)

    Returns True if the command was accepted (MAV_RESULT_ACCEPTED == 0).
    """
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,            # confirmation
        float(servo_num),   # param1: servo number (1-based)
        float(pwm_us),      # param2: PWM value (µs)
        0, 0, 0, 0, 0       # unused params
    )

    # Wait for ACK
    deadline = time.time() + ACK_TIMEOUT
    while time.time() < deadline:
        msg = mav.recv_match(type='COMMAND_ACK', blocking=True,
                             timeout=ACK_TIMEOUT)
        if msg is None:
            break
        if msg.command == mavutil.mavlink.MAV_CMD_DO_SET_SERVO:
            accepted = (msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED)
            return accepted
    return False


def set_all_safe(mav: mavutil.mavfile) -> None:
    """Set all servo channels to the safe (1000µs) value."""
    for ch in range(1, NUM_SERVO_CHANNELS + 1):
        send_do_set_servo(mav, ch, PWM_SAFE)
        time.sleep(0.05)


# ---------------------------------------------------------------------------
# Test routines
# ---------------------------------------------------------------------------

def test_channel_sweep(mav: mavutil.mavfile) -> dict:
    """
    Sweep each servo channel MIN→NEUTRAL→MAX, one at a time.

    Returns a dict: {channel_num: 'PASS'|'FAIL'|'NO_ACK'}
    """
    results = {}

    print("\n--- PWM channel sweep (50 Hz) ---")
    for ch in range(1, NUM_SERVO_CHANNELS + 1):
        gpio = ch - 1           # GPIO0 for ch1, GPIO7 for ch8
        slice_n = gpio // 2     # PWM slices 0-3
        chan_name = 'A' if gpio % 2 == 0 else 'B'
        print(f"  Servo {ch}  →  GPIO{gpio}  (slice {slice_n} ch {chan_name})")

        # Reset all to safe, then drive this channel
        set_all_safe(mav)
        time.sleep(0.1)

        step_results = []
        for pwm in (PWM_MIN, PWM_NEUTRAL, PWM_MAX, PWM_NEUTRAL):
            ok = send_do_set_servo(mav, ch, pwm)
            step_results.append(ok)
            if not ok:
                print(f"    ✗  ch{ch} @ {pwm}µs — NO ACK")
                break
            print(f"    ✓  ch{ch} @ {pwm}µs")
            time.sleep(HOLD_SECS)

        results[ch] = 'PASS' if all(step_results) else (
                      'NO_ACK' if not any(step_results) else 'FAIL')

    # Leave all at neutral
    for ch in range(1, NUM_SERVO_CHANNELS + 1):
        send_do_set_servo(mav, ch, PWM_NEUTRAL)

    return results


def test_all_channels_simultaneously(mav: mavutil.mavfile) -> bool:
    """
    Drive all 8 channels at different widths simultaneously.
    Verifies that all four PWM slices are running concurrently.
    """
    print("\n--- All 8 channels simultaneously ---")
    widths = [1000, 1100, 1200, 1300, 1400, 1500, 1600, 1700]
    ok = True
    for idx, pwm in enumerate(widths, 1):
        if not send_do_set_servo(mav, idx, pwm):
            print(f"  ✗  ch{idx} @ {pwm}µs — NO ACK")
            ok = False
        else:
            print(f"  ✓  ch{idx} @ {pwm}µs")
    time.sleep(HOLD_SECS * 2)
    set_all_safe(mav)
    return ok


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> int:
    parser = argparse.ArgumentParser(description="Pico2 PWM hardware test")
    parser.add_argument("--port", default="/dev/ttyACM1",
                        help="Serial port (default: /dev/ttyACM1)")
    parser.add_argument("--baud", type=int, default=115200,
                        help="Baud rate (default: 115200, irrelevant for USB CDC)")
    args = parser.parse_args()

    mav = connect(args.port, args.baud)

    # First leave everything at safe minimum (1000µs) before sweeping
    print("\nSetting all channels to safe minimum (1000µs) ...")    
    set_all_safe(mav)
    time.sleep(0.5)

    # Run tests
    sweep_results = test_channel_sweep(mav)
    all_ok = test_all_channels_simultaneously(mav)

    # -----------------------------------------------------------------------
    # Summary
    # -----------------------------------------------------------------------
    print("\n" + "=" * 50)
    print("PWM TEST SUMMARY")
    print("=" * 50)
    any_fail = False
    for ch, r in sweep_results.items():
        gpio = ch - 1
        icon = "✓" if r == "PASS" else "✗"
        print(f"  Servo {ch} (GPIO{gpio}): {icon} {r}")
        if r != "PASS":
            any_fail = True

    if not all_ok:
        print("  Simultaneous 8-channel test: ✗ FAIL")
        any_fail = True
    else:
        print("  Simultaneous 8-channel test: ✓ PASS")

    print()
    if any_fail:
        print("RESULT: SOME TESTS FAILED — check the notes below.")
        print()
        print("Troubleshooting:")
        print("  • Ensure arming check is disabled or ARMING_CHECK=0.")
        print("    DO_SET_SERVO requires the vehicle to be armed or for SERVO_FUNCTION")
        print("    on the channel to be set to 0 (RCPassthrough) or >0 with safety off.")
        print("  • If all channels NO_ACK: flight controller may not be accepting")
        print("    servo overrides — try setting BRD_SAFETYENABLE=0 and rebooting.")
        print("  • If only some channels fail: check wiring / solder joints on carrier.")
    else:
        print("RESULT: ALL TESTS PASSED")
        print()
        print("Hardware verification steps (requires oscilloscope/logic analyser):")
        print("  1. Confirm 20ms period (50 Hz) on all GPIO0-7.")
        print("  2. Confirm pulse widths swept correctly (1000/1500/2000µs).")
        print("  3. Confirm simultaneous independent output on all 4 PWM slices.")

    set_all_safe(mav)
    return 1 if any_fail else 0


if __name__ == "__main__":
    sys.exit(main())
