# Copyright (C) 2026 David Buzz
# SPDX-License-Identifier: GPL-3.0-or-later
# Part of ArduPilot and released under its licence: see COPYING.txt.

extends SceneTree # godot --headless --path .. -s native/check.gd
const GLTFPhysics := preload("res://scripts/gltf_physics.gd")
const Vehicle := preload("res://scripts/ap_vehicle.gd")
const TICKS := 200 # one second: tumbling magnifies rounding
const RATE := 200
const TOLERANCE := 0.01 # m, m/s: scripted versus C++

var pairs: Array = [] # [name, scripted, native, starts]
var tick := 0

func _find(n: Node) -> Vehicle:
    if n is Vehicle:
        return n
    for c in n.get_children():
        var v := _find(c)
        if v != null:
            return v
    return null

func _initialize() -> void:
    Engine.physics_ticks_per_second = RATE
    if not ClassDB.class_exists(&"ArduPilotNative"):
        print("no native library: run native/build.sh")
        quit(1)
        return
    var x := 0.0
    for f in DirAccess.get_files_at("res://models"):
        if not f.ends_with(".gltf"):
            continue
        var pair := [f]
        for k in 2:
            var scene: Node3D = GLTFPhysics.new().load_gltf(ProjectSettings.globalize_path("res://models/" + f))
            scene.position = Vector3(x, 100.0, 50.0 * k) # airborne, clear of each other
            root.add_child(scene)
            pair.append(_find(scene))
        x += 30.0
        pairs.append(pair)
    physics_frame.connect(_step)

func _pair_up() -> void: # once they are in the tree
    var flown: Array = []
    for p in pairs:
        var a: Vehicle = p[1]
        var b: Vehicle = p[2]
        if b._native == null:
            print("%-22s scripted only: no rotor or surface, or one moves on the body" % p[0])
            continue
        a._native = null # this one stays in script
        a._native_rotors.clear()
        a._native_surfaces.clear()
        a._scripted = a._components
        for v in [a, b]:
            v.linear_velocity = Vector3(0, 0, 15)
        flown.append([p[0], a, b, a.global_position, b.global_position])
    pairs = flown

func _step() -> void:
    if tick == 0:
        _pair_up()
    var pwm := PackedInt32Array()
    pwm.resize(16)
    for i in 16:
        pwm[i] = 1500 + int(350.0 * sin(0.03 * tick + i))
    for p in pairs:
        p[1].apply_actuators(pwm, 1.0 / RATE)
        p[2].apply_actuators(pwm, 1.0 / RATE)
    tick += 1
    if tick < TICKS:
        return
    var bad := 0
    for p in pairs:
        var a: Vehicle = p[1]
        var b: Vehicle = p[2]
        var moved: float = (a.global_position - p[3]).length()
        var d_pos: float = ((a.global_position - p[3]) - (b.global_position - p[4])).length()
        var d_vel := (a.linear_velocity - b.linear_velocity).length()
        var d_spin := (a.angular_velocity - b.angular_velocity).length()
        var ok: bool = d_pos < TOLERANCE and d_vel < TOLERANCE and d_spin < TOLERANCE
        bad += 0 if ok else 1
        print("%-22s %s  moved %6.1f m at %5.1f m/s; apart by %.5f m, %.5f m/s, %.5f rad/s" % [
                p[0], "same" if ok else "DIFFERS", moved, a.linear_velocity.length(), d_pos, d_vel, d_spin])
    quit(1 if bad > 0 else 0)
