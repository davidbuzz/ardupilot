#!/usr/bin/env python3
'''
Write models/plane.gltf: a generic 2 kg aeroplane with tricycle gear and
nose wheel steering, sized like SITL's plane so that the ArduPlane tests'
airspeeds, throttle settings and takeoff technique suit it.

  ./make_plane.py [output.gltf]

AP_FLAKE8_CLEAN
'''

import json
import math
import os
import sys


def qaxis(axis, deg):
    h = math.radians(deg) / 2
    s = math.sin(h)
    return [round(axis[0] * s, 8), round(axis[1] * s, 8), round(axis[2] * s, 8), round(math.cos(h), 8)]


X = (1, 0, 0)
Z = (0, 0, 1)
nodes = [None]  # body goes first
shapes = []


def shape(d):
    shapes.append(d)
    return len(shapes) - 1


def add(node):
    nodes.append(node)
    return len(nodes) - 1


def collider(name, size, translation):
    s = shape({"type": "box", "box": {"size": size}})
    return add({"name": name, "translation": translation,
                "extensions": {"KHR_physics_rigid_bodies": {"collider": {"geometry": {"shape": s}, "physicsMaterial": 0}}}})


def part(name, kind, params, translation, rotation=None, children=None):
    n = {"name": name, "translation": translation, "extensions": {"ARDUPILOT_sitl": {kind: params}}}
    if rotation:
        n["rotation"] = rotation
    if children:
        n["children"] = children
    return add(n)


children = []
# glTF axes: +Z forward, +X left, +Y up. The body origin is the centre of
# mass, at the wing's quarter chord.
children.append(collider("fuselage", [0.1, 0.1, 1.0], [0, 0, -0.1]))
children.append(collider("wing_box", [1.6, 0.02, 0.25], [0, 0.06, -0.0625]))
children.append(collider("tail_box", [0.5, 0.01, 0.16], [0, 0.05, -0.69]))
children.append(collider("fin_box", [0.01, 0.25, 0.18], [0, 0.175, -0.70]))

wing = {"span": 0.4, "chord": 0.25, "aspectRatio": 6.4, "cl0": 0.25, "cm0": -0.04, "cd0": 0.012, "stallAngle": 14}
dihedral = 3.0
for side, sign in (("left", 1), ("right", -1)):
    for panel, x in (("inner", 0.2), ("outer", 0.6)):
        params = dict(wing)
        if panel == "outer":
            # SERVO1 above trim rolls right: left aileron trailing edge down
            params.update({"flapServo": 1, "flapAngleMin": -20 * sign, "flapAngleMax": 20 * sign,
                           "flapChordFraction": 0.25})
        y = 0.06 + x * math.tan(math.radians(dihedral))
        children.append(part("wing_%s_%s" % (side, panel), "surface", params,
                             [round(sign * x, 4), round(y, 4), 0.0], qaxis(Z, sign * dihedral)))

# tailplane at -1.7 deg incidence; SERVO2 above trim pitches up: trailing edge up
children.append(part("tailplane", "surface",
                     {"span": 0.5, "chord": 0.16, "cd0": 0.015, "stallAngle": 15, "downwash": 0.45,
                      "flapServo": 2, "flapAngleMin": 15, "flapAngleMax": -15, "flapChordFraction": 0.35},
                     [0, 0.05, -0.65], qaxis(X, 1.7)))
# fin: span up, lift towards the right; SERVO4 above trim yaws right, pushing the tail left
children.append(part("fin", "surface",
                     {"span": 0.25, "chord": 0.18, "aspectRatio": 2.5, "cd0": 0.015, "stallAngle": 18,
                      "flapServo": 4, "flapAngleMin": 25, "flapAngleMax": -25, "flapChordFraction": 0.35},
                     [0, 0.175, -0.66], qaxis(Z, 90)))

children.append(part("propeller", "rotor",
                     {"servo": 3, "maxThrust": 34.0, "thrustExpo": 0.3, "torqueRatio": 0.012,
                      "direction": "ccw", "timeConstant": 0.1, "diameter": 0.3, "pitchSpeed": 90.0},
                     [0, 0, 0.45], qaxis(X, 90)))

# Names avoid Godot's import hint suffixes (-col, -rigid, -wheel, ...).
# nose wheel on a steering pivot with a vertical axis, driven by the rudder channel
nose_wheel = part("wheel_nose", "wheel",
                  {"radius": 0.035, "travel": 0.04, "spring": 400, "damping": 15, "grip": 0.9,
                   "rollingResistance": 0.12},
                  [-0.12, 0, 0], qaxis(Z, -90))
children.append(part("steering_nose", "hinge",
                     {"servo": 4, "angleMin": 10, "angleMax": -10, "maxRate": 200},
                     [0, -0.05, 0.30], qaxis(Z, 90), [nose_wheel]))
for side, x in (("left", 0.15), ("right", -0.15)):
    children.append(part("wheel_" + side, "wheel",
                         {"radius": 0.04, "travel": 0.04, "spring": 700, "damping": 25, "grip": 0.8,
                          "rollingResistance": 0.12},
                         [x, -0.17, -0.12]))

nodes[0] = {
    "name": "plane",
    "translation": [0, 0.22, 0],
    "children": children,
    "extensions": {
        "KHR_physics_rigid_bodies": {"motion": {
            "mass": 2.0,
            "inertiaDiagonal": [0.16, 0.26, 0.12],
            "centerOfMass": [0, 0, 0],
        }},
        "ARDUPILOT_sitl": {"drag": {"area": [0.05, 0.05, 0.028]}},
    },
}

gltf = {
    "asset": {"version": "2.0", "generator": "ArduPilot SITL Godot example"},
    "extensionsUsed": ["KHR_physics_rigid_bodies", "KHR_implicit_shapes", "ARDUPILOT_sitl"],
    "extensions": {
        "KHR_implicit_shapes": {"shapes": shapes},
        "KHR_physics_rigid_bodies": {"physicsMaterials": [
            {"staticFriction": 0.6, "dynamicFriction": 0.5, "restitution": 0.1},
        ]},
        "ARDUPILOT_sitl": {"vehicle": 0},
    },
    "scene": 0,
    "scenes": [{"nodes": [0]}],
    "nodes": nodes,
}
out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "models", "plane.gltf")
with open(out, "w") as f:
    json.dump(gltf, f, indent=2)
