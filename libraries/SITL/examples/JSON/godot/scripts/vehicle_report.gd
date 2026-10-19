extends RefCounted
## A summary of a vehicle's parts and the figures that matter when
## designing one: thrust to weight, wing loading, stall speed and static
## margin. The figures come from the same models the simulation uses.

const Vehicle := preload("res://scripts/ap_vehicle.gd")
const Rotor := preload("res://scripts/ap_rotor.gd")
const Hinge := preload("res://scripts/ap_hinge.gd")
const Surface := preload("res://scripts/ap_surface.gd")
const Wheel := preload("res://scripts/ap_wheel.gd")

const G := 9.80665


static func _parts(n: Node, out: Array) -> void:
    for c in n.get_children():
        if c is Rotor or c is Hinge or c is Surface or c is Wheel:
            out.append(c)
        if not (c is RigidBody3D):
            _parts(c, out)


## transform of n relative to the vehicle, valid outside the SceneTree
static func _relative(n: Node, vehicle: Node) -> Transform3D:
    var t := Transform3D.IDENTITY
    while n != null and n != vehicle:
        if n is Node3D:
            t = n.transform * t
        n = n.get_parent()
    return t


static func report(vehicle: RigidBody3D) -> String:
    var lines := PackedStringArray()
    var mass := vehicle.mass
    var com := vehicle.center_of_mass if vehicle.center_of_mass_mode == RigidBody3D.CENTER_OF_MASS_MODE_CUSTOM else Vector3.ZERO
    lines.append("vehicle '%s': mass %.3f kg, inertia %s kg.m^2 (glTF X, Y, Z)" % [vehicle.name, mass, vehicle.inertia])
    var parts: Array = []
    _parts(vehicle, parts)

    var lift_thrust := 0.0
    var fwd_thrust := 0.0
    var area := 0.0
    var cl_area := 0.0
    var slope_area := 0.0
    var slope_area_z := 0.0
    var chord_area := 0.0
    for p in parts:
        var t := _relative(p, vehicle)
        var pos := Vehicle.TO_NED * t.origin
        var where := "FRD (%.3f, %.3f, %.3f)" % [pos.x, pos.y, pos.z]
        if p is Rotor:
            var axis := t.basis.y.normalized()
            lift_thrust += p.max_thrust * maxf(axis.y, 0.0)
            fwd_thrust += p.max_thrust * maxf(axis.z, 0.0)
            lines.append("  rotor   %-14s servo %2d at %s %s, %.1f N" % [p.name, p.servo, where, p.direction, p.max_thrust])
        elif p is Hinge:
            lines.append("  hinge   %-14s servo %2d at %s, %.0f..%.0f deg" % [p.name, p.servo, where, p.angle_min, p.angle_max])
        elif p is Wheel:
            lines.append("  wheel   %-14s at %s, radius %.3f m" % [p.name, where, p.radius])
        elif p is Surface:
            var s: float = p.span * p.chord
            var flap := "flap servo %d" % p.flap_servo if p.flap_servo > 0 else "no flap"
            lines.append("  surface %-14s at %s, %.3f m^2, %s" % [p.name, where, s, flap])
            # surfaces that lift upwards set the wing loading and pitch stability
            var up := t.basis.y.normalized().y
            if up > 0.7:
                var a: float = p.lift_slope() * up * up * (1.0 - p.downwash)
                area += s
                cl_area += s * (p.cl0 + p.lift_slope() * deg_to_rad(p.stall_angle))
                slope_area += a * s
                slope_area_z += a * s * t.origin.z
                chord_area += s * p.chord

    var weight := mass * G
    if lift_thrust > 0.0:
        lines.append("hover thrust/weight %.2f" % (lift_thrust / weight))
    if fwd_thrust > 0.0:
        lines.append("forward thrust/weight %.2f" % (fwd_thrust / weight))
    if area > 0.0:
        lines.append("lifting area %.3f m^2, wing loading %.1f kg/m^2" % [area, mass / area])
        lines.append("stall speed about %.1f m/s" % sqrt(2.0 * weight / (1.225 * cl_area)))
        var neutral := slope_area_z / slope_area
        var mean_chord := chord_area / area
        lines.append("neutral point %.3f m ahead of the origin, centre of mass %.3f m ahead" % [neutral, com.z])
        lines.append("static margin %.0f%% of mean chord %.3f m (positive is stable)" % [
                100.0 * (com.z - neutral) / mean_chord, mean_chord])
    return "\n".join(lines)
