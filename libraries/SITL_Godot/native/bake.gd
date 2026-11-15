# Copyright (C) 2026 David Buzz
# SPDX-License-Identifier: GPL-3.0-or-later
# Part of ArduPilot and released under its licence: see COPYING.txt.

extends SceneTree # godot --headless --path .. -s native/bake.gd
const GLTFPhysics := preload("res://scripts/gltf_physics.gd")
const Vehicle := preload("res://scripts/ap_vehicle.gd")
const Rotor := preload("res://scripts/ap_rotor.gd")
const Surface := preload("res://scripts/ap_surface.gd")
const Hinge := preload("res://scripts/ap_hinge.gd")
const Wheel := preload("res://scripts/ap_wheel.gd")
const Floater := preload("res://scripts/ap_floater.gd")
const Streamer := preload("res://scripts/ap_streamer.gd")
const HeliRotor := preload("res://scripts/ap_heli_rotor.gd")
const OUT := "res://native/build/bakes" # loader's result, plain records

var loaded: Array = [] # [name, scene]

func _find(n: Node) -> Vehicle:
    if n is Vehicle:
        return n
    for c in n.get_children():
        var v := _find(c)
        if v != null:
            return v
    return null

func _initialize() -> void:
    for f in DirAccess.get_files_at("res://models"):
        if not f.ends_with(".gltf"):
            continue
        var scene: Node3D = GLTFPhysics.new().load_gltf(ProjectSettings.globalize_path("res://models/" + f))
        root.add_child(scene) # read before any step
        loaded.append([f.get_basename(), scene])
    physics_frame.connect(_bake)

func num(v: float) -> String:
    return String.num(v, 9)

func nums(values: Array) -> String:
    return " ".join(values.map(func(v): return num(float(v))))

func pose(t: Transform3D) -> String: # position, then quaternion xyzw
    var q := t.basis.orthonormalized().get_rotation_quaternion()
    return nums([t.origin.x, t.origin.y, t.origin.z, q.x, q.y, q.z, q.w])

func _shapes(body: Node3D, index: int, n: Node, lines: PackedStringArray) -> void:
    for c in n.get_children():
        if c is CollisionShape3D and c.shape != null:
            var at := pose(body.global_transform.affine_inverse() * (c as Node3D).global_transform)
            var s: Shape3D = c.shape
            if s is BoxShape3D:
                lines.append("shape %d box %s %s %s" % [index, nums([0.5 * s.size.x, 0.5 * s.size.y, 0.5 * s.size.z]), num(s.margin), at])
            elif s is SphereShape3D:
                lines.append("shape %d sphere %s %s" % [index, num(s.radius), at])
            elif s is CapsuleShape3D:
                lines.append("shape %d capsule %s %s" % [index, nums([s.radius, s.height]), at])
            elif s is CylinderShape3D:
                lines.append("shape %d cylinder %s %s %s" % [index, nums([s.radius, s.height]), num(s.margin), at])
            else:
                lines.append("unsupported a %s collider on %s" % [s.get_class(), c.name])
        if not (c is RigidBody3D): # a nested body owns its own
            _shapes(body, index, c, lines)

func _body(b: RigidBody3D, index: int, scene: Node3D, lines: PackedStringArray) -> void:
    var st := PhysicsServer3D.body_get_direct_state(b.get_rid())
    var inertia := Vector3.ONE / st.inverse_inertia
    var com := st.center_of_mass_local
    var mat := b.physics_material_override
    lines.append("body %s %s %s %s %s %s" % [String(b.name).replace(" ", "_"), num(1.0 / st.inverse_mass),
            nums([inertia.x, inertia.y, inertia.z, com.x, com.y, com.z]),
            pose(scene.global_transform.affine_inverse() * b.global_transform),
            num(mat.friction if mat != null else 1.0), num(mat.bounce if mat != null else 0.0)])
    _shapes(b, index, b, lines)

func _hinge_above(c: Node, v: Vehicle, hinges: Array) -> int: # index; -1 body; -2 elsewhere
    var n := c.get_parent()
    while n != null and n != v:
        if n is Hinge:
            return hinges.find(n)
        if n is RigidBody3D:
            return -2
        n = n.get_parent()
    return -1 if n == v else -2

func _write(model: String, scene: Node3D) -> String:
    var v := _find(scene)
    var lines := PackedStringArray(["sitlbake 1", "model %s" % model])
    if v == null:
        return ""
    lines.append("env %s" % nums([v.air_density, float(v.water_enabled), v.water_level, v.water_density]))
    lines.append("drag %s" % nums([v.drag_area.x, v.drag_area.y, v.drag_area.z]))
    _body(v, 0, scene, lines)
    var bodies: Array = [v] # vehicle first, then limbs
    for limb in v._limbs:
        bodies.append(limb[0])
        _body(limb[0], bodies.size() - 1, scene, lines)
    var joints: Array = scene.find_children("*", "HingeJoint3D", true, false)
    for j in joints:
        var a := bodies.find(j.get_node_or_null(j.node_a))
        var b := bodies.find(j.get_node_or_null(j.node_b))
        if a < 0 or b < 0:
            lines.append("unsupported joint %s to something which is not a body" % j.name)
            continue
        lines.append("joint %d %d %s" % [a, b, pose(scene.global_transform.affine_inverse() * (j as Node3D).global_transform)])
    var hinges: Array = v._components.filter(func(c): return c is Hinge and not c.has_meta(&"ap_joint"))
    for c in v._components:
        if c is Streamer: # drawn only
            continue
        if c is Hinge and c.has_meta(&"ap_joint"): # servo on a joint
            var j := joints.find(c.get_node_or_null(c.get_meta(&"ap_joint")))
            if j < 0:
                lines.append("unsupported hinge %s (its joint is missing)" % c.name)
                continue
            lines.append("jointhinge %d %s" % [j, nums([c.servo, c.pwm_min, c.pwm_max, c.angle_min, c.angle_max,
                    c.max_rate, c.max_torque, float(c.continuous), c.built_angle])])
            continue
        var above := _hinge_above(c, v, hinges)
        if above == -2:
            lines.append("unsupported part %s on a jointed body" % c.name)
            continue
        var parent: Node3D = hinges[above] if above >= 0 else v
        var at := pose(parent.global_transform.affine_inverse() * (c as Node3D).global_transform)
        if c is Hinge:
            at = pose(parent.global_transform.affine_inverse() * (c.get_parent() as Node3D).global_transform * c.rest)
            lines.append("hinge %d %s %s" % [above, at, nums([c.servo, c.pwm_min, c.pwm_max, c.angle_min, c.angle_max,
                    c.max_rate, float(c.sheet)])])
        elif c is Rotor:
            lines.append("rotor %d %s %s" % [above, at, nums(Array(c.native_params()))])
        elif c is Surface:
            lines.append("surface %d %s %s" % [above, at, nums(Array(c.native_params()))])
        elif c is Wheel:
            lines.append("wheel %d %s %s" % [above, at, nums([c.radius, c.travel, c.spring, c.damping, c.rolling_resistance,
                    c.grip, c.drive_servo, c.drive_pwm_min, c.drive_pwm_max, c.max_drive_force, c.free_speed])])
        elif c is Floater:
            lines.append("floater %d %s %s" % [above, at, nums([float(c.fluid == "air"), c.volume, c.height, c.damping,
                    c.drag_area.x, c.drag_area.y, c.drag_area.z])])
        elif c is HeliRotor:
            lines.append("helirotor %d %s %s" % [above, at, nums([c.swash_servo_1, c.swash_servo_2, c.swash_servo_3, c.rsc_servo,
                    c.pwm_min, c.pwm_max, c.collective_min, c.collective_max, c.cyclic_scale, c.nominal_rpm, c.spool_time,
                    c.thrust_scale, c.torque_ratio, c.hub_stiffness, c.flap_time_constant,
                    -1.0 if c.direction == "ccw" else 1.0, c.diameter])])
        else:
            lines.append("unsupported part %s (%s)" % [c.name, c.gltf_key()])
    return "\n".join(lines) + "\n"

func _bake() -> void:
    var dir := ProjectSettings.globalize_path(OUT)
    DirAccess.make_dir_recursive_absolute(dir)
    for entry in loaded:
        var text := _write(entry[0], entry[1])
        var f := FileAccess.open(dir.path_join(entry[0] + ".bake"), FileAccess.WRITE)
        f.store_string(text)
        f.close()
        var why := Array(text.split("\n")).filter(func(l): return l.begins_with("unsupported"))
        print("%-18s %s" % [entry[0], "baked" if why.is_empty() else "baked, not yet flyable: " + "; ".join(why.map(func(l): return l.substr(12)))])
    quit()
