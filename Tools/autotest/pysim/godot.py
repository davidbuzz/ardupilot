'''
Run the Godot and Jolt physics backend in libraries/SITL/examples/JSON/godot,
which provides a vehicle's physics to SITL through SITL's JSON model.

AP_FLAKE8_CLEAN
'''

from __future__ import annotations

import atexit
import os
import shutil
import subprocess
import sys
import time

from pysim import util

PROJECT = util.reltopdir("libraries/SITL/examples/JSON/godot")

# the SITL model which talks to the backend
SITL_MODEL = "JSON:127.0.0.1"

# vehicleinfo.json vehicle -> {frame: model in the Godot project's models directory}
MODELS = {
    "ArduCopter": {"+": "quadx", "quad": "quadx", "X": "quadx"},
    "ArduPlane": {"plane": "plane", "plane-3d": "plane3d"},
}


def model_for(vehicle, frame):
    '''the Godot model for a vehicle's frame, or None if there isn't one'''
    return MODELS.get(vehicle, {}).get(frame)


def frames_for(vehicle):
    return sorted(MODELS.get(vehicle, {}).keys())


def check_supported(vehicle):
    if vehicle not in MODELS:
        raise ValueError("no Godot physics model for %s; there are models for %s" %
                         (vehicle, ", ".join(sorted(MODELS.keys()))))


def params_filepath(model):
    '''parameters for a Godot model, to apply after the frame's defaults'''
    return os.path.join(PROJECT, "models", model + ".parm")


def find_godot():
    '''the Godot binary, from $GODOT or the PATH'''
    for candidate in [os.environ.get("GODOT"), "godot", "godot4", "Godot",
                      "/Applications/Godot.app/Contents/MacOS/Godot"]:
        if candidate:
            path = shutil.which(candidate)
            if path is not None:
                return path
    raise FileNotFoundError("Godot 4.4 or later not found: put it on the PATH as 'godot' or set GODOT")


class Backend(object):
    '''the backend process. It keeps running across SITL restarts, resetting
    the vehicle each time SITL starts again, and is restarted only to change
    model.'''

    def __init__(self, logfile, instance=0, headless=True):
        self.logfile = logfile
        # SITL's JSON model sends to port 9002 + 10 x instance
        self.port = 9002 + 10 * instance
        self.headless = headless
        self.process = None
        self.model = None
        atexit.register(self.stop)

    def is_running(self):
        return self.process is not None and self.process.poll() is None

    def start(self, model, timeout=60):
        if self.is_running():
            if model == self.model:
                return
            self.stop()
        headless = self.headless
        if (not headless and sys.platform.startswith("linux") and
                not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY")):
            print("No display, so running the Godot backend without its window")
            headless = True
        # a fixed frame rate lets the physics run faster than real time
        # when SITL is sped up; SITL sets the pace, so V-Sync must not
        cmd = [find_godot(), "--path", PROJECT, "--fixed-fps", "50"]
        if headless:
            cmd.append("--headless")
        else:
            cmd.append("--disable-vsync")
        cmd.extend(["--",
                    "--vehicle=res://models/%s.gltf" % model,
                    "--port=%u" % self.port])
        with open(self.logfile, "w") as log:
            self.process = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT)
        self.model = model
        deadline = time.time() + timeout
        while time.time() < deadline:
            with open(self.logfile) as f:
                if "waiting for SITL" in f.read():
                    return
            if self.process.poll() is not None:
                raise RuntimeError("the Godot backend exited; see %s" % self.logfile)
            time.sleep(0.1)
        self.stop()
        raise RuntimeError("the Godot backend did not start; see %s" % self.logfile)

    def stop(self):
        if not self.is_running():
            return
        self.process.terminate()
        try:
            self.process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait()
