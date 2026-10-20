#!/usr/bin/env python3
'''
Write models/plane3d.gltf: an aerobatic plane matching SITL's plane-3d,
2 kg with a 1.88 m span and 0.45 m^2 of wing, for 3D and scripted
aerobatics.

  ./make_plane3d.py [output.gltf]

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
nodes = [None]
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
# mass, 3 cm behind the wing's quarter chord for near-neutral stability.
children.append(collider("fuselage", [0.1, 0.12, 1.1], [0, 0, -0.1]))
children.append(collider("wing_box", [1.88, 0.03, 0.24], [0, 0, -0.03]))
children.append(collider("tail_box", [0.6, 0.01, 0.167], [0, 0, -0.78]))
children.append(collider("fin_box", [0.01, 0.3, 0.2], [0, 0.15, -0.79]))

# symmetric aerofoil, full span ailerons
wing = {"span": 0.47, "chord": 0.24, "aspectRatio": 7.85, "cl0": 0.0, "cd0": 0.02, "stallAngle": 14,
        "flapServo": 1, "flapChordFraction": 0.3}
for side, sign in (("left", 1), ("right", -1)):
    for panel, x in (("inner", 0.235), ("outer", 0.705)):
        params = dict(wing)
        # SERVO1 above trim rolls right: left aileron trailing edge down
        params.update({"flapAngleMin": -30 * sign, "flapAngleMax": 30 * sign})
        children.append(part("wing_%s_%s" % (side, panel), "surface", params,
                             [round(sign * x, 4), 0, 0.03]))

# tailplane at -0.5 deg incidence; SERVO2 above trim pitches up: trailing edge up
children.append(part("tailplane", "surface",
                     {"span": 0.6, "chord": 0.167, "cd0": 0.02, "stallAngle": 15, "downwash": 0.45,
                      "flapServo": 2, "flapAngleMin": 30, "flapAngleMax": -30, "flapChordFraction": 0.5},
                     [0, 0, -0.75], qaxis(X, 0.5)))
# fin: span up, lift to the right; SERVO4 above trim yaws right
children.append(part("fin", "surface",
                     {"span": 0.3, "chord": 0.2, "aspectRatio": 2.5, "cd0": 0.02, "stallAngle": 18,
                      "flapServo": 4, "flapAngleMin": 35, "flapAngleMax": -35, "flapChordFraction": 0.5},
                     [0, 0.15, -0.76], qaxis(Z, 90)))

children.append(part("propeller", "rotor",
                     {"servo": 3, "maxThrust": 50.0, "thrustExpo": 0.5, "torqueRatio": 0.012,
                      "direction": "ccw", "timeConstant": 0.05, "diameter": 0.3, "pitchSpeed": 100.0},
                     [0, 0, 0.5], qaxis(X, 90)))

# Names avoid Godot's import hint suffixes (-col, -rigid, -wheel, ...).
nose_wheel = part("wheel_nose", "wheel",
                  {"radius": 0.035, "travel": 0.04, "spring": 400, "damping": 15, "grip": 0.9,
                   "rollingResistance": 0.03},
                  [-0.14, 0, 0], qaxis(Z, -90))
children.append(part("steering_nose", "hinge",
                     {"servo": 4, "angleMin": 25, "angleMax": -25, "maxRate": 200},
                     [0, -0.06, 0.32], qaxis(Z, 90), [nose_wheel]))
for side, x in (("left", 0.16), ("right", -0.16)):
    children.append(part("wheel_" + side, "wheel",
                         {"radius": 0.04, "travel": 0.04, "spring": 700, "damping": 25, "grip": 0.8,
                          "rollingResistance": 0.03},
                         [x, -0.2, -0.07]))

nodes[0] = {
    "name": "plane3d",
    "translation": [0, 0.25, 0],
    "children": children,
    "extensions": {
        "KHR_physics_rigid_bodies": {"motion": {
            "mass": 2.0,
            "inertiaDiagonal": [0.18, 0.30, 0.13],
            "centerOfMass": [0, 0, 0],
        }},
        "ARDUPILOT_sitl": {"drag": {"area": [0.06, 0.06, 0.008]}},
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
    os.path.dirname(os.path.abspath(__file__)), "..", "models", "plane3d.gltf")
with open(out, "w") as f:
    json.dump(gltf, f, indent=2)
