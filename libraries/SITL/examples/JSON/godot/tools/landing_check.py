#!/usr/bin/env python3
'''
Fly test_flight.py's plane mission, with its runway takeoff, circuit and
automatic landing, on a Godot model set up as the ArduPlane autotest suite's
default frame, plane-elevrev. The copter test can be flown the same way.

  ./landing_check.py [--vehicle plane|copter] [--model plane] [--instance 1]

It starts its own backend and SITL on the given instance, so it can run
beside an autotest on instance 0. Build the vehicle first.

AP_FLAKE8_CLEAN
'''

import argparse
import os
import subprocess
import sys

from sim_pair import PROJECT, SimPair


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--vehicle", default="plane", choices=["plane", "copter"])
    parser.add_argument("--model", default=None, help="model in the project's models directory")
    parser.add_argument("--instance", type=int, default=1, help="SITL instance")
    args = parser.parse_args()

    with SimPair(args.vehicle, model=args.model, instance=args.instance) as sim:
        result = subprocess.run([sys.executable, os.path.join(PROJECT, "test_flight.py"),
                                 "--vehicle", args.vehicle, "--connect", sim.mavlink])
    sys.exit(result.returncode)


if __name__ == "__main__":
    main()
