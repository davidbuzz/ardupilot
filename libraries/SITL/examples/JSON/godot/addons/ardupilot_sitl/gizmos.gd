@tool
extends EditorNode3DGizmoPlugin
## Draws vehicle parts in the 3D editor: rotor discs with thrust arrows and
## spin direction, lifting surface outlines with flap hinge lines, hinge
## axes with their travel, wheels with their suspension, and the vehicle's
## forward axis and centre of mass.

const Vehicle := preload("res://scripts/ap_vehicle.gd")
const Rotor := preload("res://scripts/ap_rotor.gd")
const Hinge := preload("res://scripts/ap_hinge.gd")
const Surface := preload("res://scripts/ap_surface.gd")
const Wheel := preload("res://scripts/ap_wheel.gd")

## arrow length per newton of thrust (m/N)
const THRUST_SCALE := 0.02


func _init() -> void:
    create_material("rotor", Color(1.0, 0.45, 0.1))
    create_material("surface", Color(0.3, 0.75, 1.0))
    create_material("flap", Color(1.0, 0.85, 0.1))
    create_material("hinge", Color(1.0, 0.25, 0.85))
    create_material("wheel", Color(0.35, 1.0, 0.35))
    create_material("vehicle", Color(1.0, 1.0, 1.0))


func _get_gizmo_name() -> String:
    return "ArduPilot vehicle parts"


func _has_gizmo(node: Node3D) -> bool:
    return node is Vehicle or node is Rotor or node is Hinge or node is Surface or node is Wheel


static func _circle(out: PackedVector3Array, centre: Vector3, u: Vector3, v: Vector3, r: float,
        from := 0.0, to := TAU, segments := 32) -> void:
    for i in segments:
        var a0 := lerpf(from, to, float(i) / segments)
        var a1 := lerpf(from, to, float(i + 1) / segments)
        out.append(centre + (u * cos(a0) + v * sin(a0)) * r)
        out.append(centre + (u * cos(a1) + v * sin(a1)) * r)


static func _arrow(out: PackedVector3Array, from: Vector3, to: Vector3, side: Vector3) -> void:
    out.append(from)
    out.append(to)
    var back := (from - to).normalized() * minf(0.03, from.distance_to(to) * 0.3)
    out.append(to)
    out.append(to + back + side * back.length() * 0.6)
    out.append(to)
    out.append(to + back - side * back.length() * 0.6)


func _redraw(gizmo: EditorNode3DGizmo) -> void:
    gizmo.clear()
    var n := gizmo.get_node_3d()
    var lines := PackedVector3Array()

    if n is Rotor:
        var r: float = maxf(n.diameter * 0.5, 0.02)
        _circle(lines, Vector3.ZERO, Vector3.RIGHT, Vector3.BACK, r)
        _arrow(lines, Vector3.ZERO, Vector3.UP * maxf(n.max_thrust * THRUST_SCALE, 0.02), Vector3.RIGHT)
        # spin direction: arrowheads on the rim, ccw seen from the tip of +Y
        var spin := 1.0 if n.direction == "ccw" else -1.0
        for k in 3:
            var a := TAU * k / 3.0
            var p := Vector3(cos(a), 0, -sin(a)) * r
            var t := Vector3(-sin(a), 0, -cos(a)) * spin
            _arrow(lines, p - t * 0.04, p, Vector3.UP)
        gizmo.add_lines(lines, get_material("rotor", gizmo))

    elif n is Surface:
        var hs: float = n.span * 0.5
        var le: float = 0.25 * n.chord
        var te: float = -0.75 * n.chord
        var corners := [Vector3(hs, 0, le), Vector3(-hs, 0, le), Vector3(-hs, 0, te), Vector3(hs, 0, te)]
        for i in 4:
            lines.append(corners[i])
            lines.append(corners[(i + 1) % 4])
        # lift direction from the aerodynamic centre
        _arrow(lines, Vector3.ZERO, Vector3.UP * minf(0.5 * n.chord + 0.05, 0.15), Vector3.RIGHT)
        gizmo.add_lines(lines, get_material("surface", gizmo))
        if n.flap_servo > 0:
            var hinge_z: float = te + n.flap_chord_fraction * n.chord
            var flap := PackedVector3Array([Vector3(hs, 0, hinge_z), Vector3(-hs, 0, hinge_z)])
            # flap travel at mid span
            var depth: float = n.flap_chord_fraction * n.chord
            for deg in [n.flap_angle_min, n.flap_angle_max]:
                var d := deg_to_rad(deg)
                flap.append(Vector3(0, 0, hinge_z))
                flap.append(Vector3(0, -sin(d) * depth, hinge_z - cos(d) * depth))
            gizmo.add_lines(flap, get_material("flap", gizmo))

    elif n is Hinge:
        lines.append(Vector3.RIGHT * -0.08)
        lines.append(Vector3.RIGHT * 0.08)
        # travel arc, drawn in the parent's frame so it doesn't move with the angle
        var to_rest: Transform3D = n.transform.affine_inverse() * n.rest_transform()
        var r := 0.05
        var a0 := deg_to_rad(n.angle_min)
        var a1 := deg_to_rad(n.angle_max)
        var arc := PackedVector3Array()
        # a pointer aft (-Z) turned about X sweeps (0, r sin a, -r cos a)
        _circle(arc, Vector3.ZERO, Vector3.FORWARD, Vector3.UP, r, a0, a1, 16)
        for i in arc.size():
            arc[i] = to_rest * arc[i]
        lines.append_array(arc)
        lines.append(Vector3.ZERO)
        lines.append(Vector3(0, 0, -r * 1.3))
        gizmo.add_lines(lines, get_material("hinge", gizmo))

    elif n is Wheel:
        _circle(lines, Vector3.ZERO, Vector3.BACK, Vector3.UP, n.radius)
        lines.append(Vector3.ZERO)
        lines.append(Vector3.UP * n.travel)
        lines.append(Vector3(-0.02, n.travel, 0))
        lines.append(Vector3(0.02, n.travel, 0))
        gizmo.add_lines(lines, get_material("wheel", gizmo))

    elif n is Vehicle:
        var com: Vector3 = n.center_of_mass if n.center_of_mass_mode == RigidBody3D.CENTER_OF_MASS_MODE_CUSTOM else Vector3.ZERO
        for axis in [Vector3.RIGHT, Vector3.UP, Vector3.BACK]:
            lines.append(com - axis * 0.04)
            lines.append(com + axis * 0.04)
        # forward is +Z, the direction the vehicle faces at yaw 0 (north)
        _arrow(lines, Vector3.ZERO, Vector3.BACK * 0.3, Vector3.RIGHT)
        gizmo.add_lines(lines, get_material("vehicle", gizmo))
