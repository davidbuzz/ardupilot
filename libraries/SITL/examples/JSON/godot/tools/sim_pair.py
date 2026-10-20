'''
Start the Godot backend and SITL together for the checks in this directory,
with the parameters the autotest suite gives the same frame, and stop both
afterwards. SITL runs in a temporary directory, so its logs and eeprom do
not land in the source tree.

AP_FLAKE8_CLEAN
'''

import os
import shutil
import signal
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT = os.path.dirname(HERE)
TOP = os.path.abspath(os.path.join(PROJECT, "..", "..", "..", "..", ".."))
AUTOTEST = os.path.join(TOP, "Tools", "autotest")

# CMAC, where the autotest suite starts
HOME = "-35.362938,149.165085,585,354"

VEHICLES = {
    # ArduPlane's default autotest frame, plane-elevrev, reverses the
    # elevator and rudder outputs, which the backend reverses back
    "plane": {
        "binary": "arduplane",
        "model": "plane",
        "reverse": "2,4",
        "defaults": ["models/plane.parm",
                     "models/plane-reverse-servo-pitch-yaw.parm",
                     "models/plane-reverse-rc-pitch-yaw.parm"],
    },
    "copter": {
        "binary": "arducopter",
        "model": "quadx",
        "reverse": "",
        "defaults": ["default_params/copter.parm"],
    },
}


def find_godot():
    '''the Godot binary, from $GODOT or the PATH'''
    for candidate in [os.environ.get("GODOT"), "godot", "godot4"]:
        if candidate:
            path = shutil.which(candidate)
            if path is not None:
                return path
    sys.exit("Godot 4.4 or later not found: put it on the PATH as 'godot' or set GODOT")


class SimPair(object):
    '''the backend and SITL for one vehicle on one SITL instance'''

    def __init__(self, vehicle="plane", model=None, instance=1, speedup=10, reverse=None):
        self.info = VEHICLES[vehicle]
        self.model = model or self.info["model"]
        self.instance = instance
        self.speedup = speedup
        self.reverse = self.info["reverse"] if reverse is None else reverse
        self.backend = None
        self.sitl = None
        self.workdir = None

    @property
    def mavlink(self):
        '''SITL's first MAVLink port'''
        return "tcp:127.0.0.1:%u" % (5760 + 10 * self.instance)

    def __enter__(self):
        # let SIGTERM, as from timeout(1), still stop both processes
        signal.signal(signal.SIGTERM, lambda *args: sys.exit(1))
        self.workdir = tempfile.mkdtemp(prefix="godot-check-")
        print("logs in %s" % self.workdir)
        backend_log = os.path.join(self.workdir, "godot.log")
        cmd = [find_godot(), "--headless", "--fixed-fps", "50", "--path", PROJECT, "--",
               "--vehicle=res://models/%s.gltf" % self.model,
               "--port=%u" % (9002 + 10 * self.instance)]
        if self.reverse:
            cmd.append("--reverse=" + self.reverse)
        with open(backend_log, "w") as log:
            self.backend = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT)
        deadline = time.time() + 60
        while "waiting for SITL" not in open(backend_log).read():
            if self.backend.poll() is not None or time.time() > deadline:
                self.__exit__(None, None, None)
                sys.exit("the Godot backend did not start; see %s" % backend_log)
            time.sleep(0.1)

        defaults = [os.path.join(AUTOTEST, p) for p in self.info["defaults"]]
        defaults.append(os.path.join(PROJECT, "models", self.model + ".parm"))
        binary = os.path.join(TOP, "build", "sitl", "bin", self.info["binary"])
        with open(os.path.join(self.workdir, "sitl.log"), "w") as log:
            self.sitl = subprocess.Popen([binary, "-w", "-I%u" % self.instance, "--home", HOME,
                                          "--model", "JSON:127.0.0.1", "--speedup", str(self.speedup),
                                          "--defaults", ",".join(defaults)],
                                         cwd=self.workdir, stdout=log, stderr=subprocess.STDOUT)
        return self

    def __exit__(self, exc_type, exc, tb):
        if self.sitl is not None and self.sitl.poll() is None:
            self.sitl.kill()
            self.sitl.wait()
        if self.backend is not None and self.backend.poll() is None:
            self.backend.terminate()
            try:
                self.backend.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.backend.kill()
                self.backend.wait()
        return False
