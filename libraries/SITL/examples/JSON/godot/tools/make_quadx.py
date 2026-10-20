#!/usr/bin/env python3
'''
Write models/quadx.gltf: a generic 1.5 kg X quadcopter for ArduCopter's
default frame.

  ./make_quadx.py [output.gltf]

AP_FLAKE8_CLEAN
'''

import json
import math
import os
import sys

s45 = math.sin(math.radians(22.5))
c45 = math.cos(math.radians(22.5))


def r(v):
    return [round(x, 8) for x in v]


nodes = []


def add(node):
    nodes.append(node)
    return len(nodes) - 1


def collider(shape, material=0):
    return {"KHR_physics_rigid_bodies": {"collider": {"geometry": {"shape": shape}, "physicsMaterial": material}}}


children = []
children.append(add({"name": "frame", "extensions": collider(0)}))
children.append(add({"name": "arm_fr_bl", "rotation": r([0, s45, 0, c45]), "extensions": collider(1)}))
children.append(add({"name": "arm_fl_br", "rotation": r([0, -s45, 0, c45]), "extensions": collider(1)}))
for name, x, z in [("fl", 0.08, 0.08), ("fr", -0.08, 0.08), ("bl", 0.08, -0.08), ("br", -0.08, -0.08)]:
    children.append(add({"name": "leg_" + name, "translation": [x, -0.06, z], "extensions": collider(2)}))
    children.append(add({"name": "foot_" + name, "translation": [x, -0.12, z], "extensions": collider(3)}))

# ArduPilot quad X motor order. glTF axes: +Z forward, -X right, +Y up
arm = 0.12
motors = [
    (1, -arm, arm, "ccw"),   # front right
    (2, arm, -arm, "ccw"),   # back left
    (3, arm, arm, "cw"),     # front left
    (4, -arm, -arm, "cw"),   # back right
]
for servo, x, z, direction in motors:
    children.append(add({
        "name": "motor%d" % servo,
        "translation": [x, 0.025, z],
        "extensions": {"ARDUPILOT_sitl": {"rotor": {
            "servo": servo,
            "pwmMin": 1000,
            "pwmMax": 2000,
            "maxThrust": 9.8,
            "thrustExpo": 0.65,
            "torqueRatio": 0.016,
            "direction": direction,
            "timeConstant": 0.03,
            "diameter": 0.254,
            "pitchSpeed": 20.0,
            "inplaneDrag": 0.02,
        }}},
    }))

body = {
    "name": "quadx",
    "translation": [0, 0.135, 0],
    "children": [],
    "extensions": {
        "KHR_physics_rigid_bodies": {"motion": {
            "mass": 1.5,
            "inertiaDiagonal": [0.015, 0.028, 0.015],
            "centerOfMass": [0, 0, 0],
        }},
        "ARDUPILOT_sitl": {"drag": {"area": [0.05, 0.08, 0.05]}},
    },
}
nodes.insert(0, body)
# shift child indices for the inserted root
body["children"] = [c + 1 for c in children]

gltf = {
    "asset": {"version": "2.0", "generator": "ArduPilot SITL Godot example"},
    "extensionsUsed": ["KHR_physics_rigid_bodies", "KHR_implicit_shapes", "ARDUPILOT_sitl"],
    "extensions": {
        "KHR_implicit_shapes": {"shapes": [
            {"type": "box", "box": {"size": [0.12, 0.05, 0.16]}},
            {"type": "box", "box": {"size": [0.34, 0.015, 0.02]}},
            {"type": "cylinder", "cylinder": {"height": 0.12, "radiusTop": 0.006, "radiusBottom": 0.006}},
            {"type": "sphere", "sphere": {"radius": 0.012}},
        ]},
        "KHR_physics_rigid_bodies": {"physicsMaterials": [
            {"staticFriction": 0.9, "dynamicFriction": 0.8, "restitution": 0.05},
        ]},
        "ARDUPILOT_sitl": {"vehicle": 0},
    },
    "scene": 0,
    "scenes": [{"nodes": [0]}],
    "nodes": nodes,
}
out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "models", "quadx.gltf")
with open(out, "w") as f:
    json.dump(gltf, f, indent=2)
