@tool
@icon("res://icons/wheel.svg")
class_name ArduPilotWheel
extends "res://scripts/ap_component.gd"
## A wheel on a spring-damper suspension, found by a ray cast.
##
## The node's origin is the wheel centre with the suspension fully extended;
## the suspension compresses along local +Y and the wheel rolls along local
## +Z. Put the node under an ArduPilotHinge (axis vertical) to steer it.
## Don't also give the wheel a collider.

@export var radius: float = 0.05:
    set(v):
        radius = v
        update_gizmos()
## suspension travel (m)
@export var travel: float = 0.05:
    set(v):
        travel = v
        update_gizmos()
## spring rate (N/m)
@export var spring: float = 500.0
## damping (N.s/m)
@export var damping: float = 20.0
## rolling resistance coefficient
@export var rolling_resistance: float = 0.02
## sideways friction coefficient
@export var grip: float = 0.9
## servo driving the wheel, 0 for none; pwm_min = full reverse, pwm_max = full forward
@export_range(0, 32) var drive_servo: int = 0
@export var drive_pwm_min: float = 1000.0
@export var drive_pwm_max: float = 2000.0
## driving force at full throttle (N)
@export var max_drive_force: float = 0.0

## suspension compression (m), 0 when the wheel is off the ground
var compression: float = 0.0
var on_ground := false
var _spin: float = 0.0


func gltf_key() -> String:
    return "wheel"


func gltf_fields() -> Dictionary:
    return {
        "radius": "radius", "travel": "travel", "spring": "spring",
        "damping": "damping", "rollingResistance": "rolling_resistance",
        "grip": "grip", "driveServo": "drive_servo", "drivePwmMin": "drive_pwm_min",
        "drivePwmMax": "drive_pwm_max", "maxDriveForce": "max_drive_force",
    }


func physics_update(vehicle: RigidBody3D, pwm: PackedInt32Array, dt: float) -> void:
    var xform := global_transform
    var up := xform.basis.y.normalized()
    var top := xform.origin + up * travel
    var length := travel + radius
    var query := PhysicsRayQueryParameters3D.create(top, top - up * length)
    query.exclude = [vehicle.get_rid()]
    var hit := vehicle.get_world_3d().direct_space_state.intersect_ray(query)
    var last := compression
    if hit.is_empty():
        compression = 0.0
        on_ground = false
        _place_visuals(dt, 0.0)
        return
    on_ground = true
    compression = length - top.distance_to(hit.position)
    var normal: Vector3 = hit.normal
    var rate := (compression - last) / dt if dt > 0.0 else 0.0
    var load := maxf(spring * compression + damping * rate, 0.0)

    var contact: Vector3 = hit.position
    var v: Vector3 = vehicle.velocity_at(contact)
    var fwd := xform.basis.z
    fwd = (fwd - normal * fwd.dot(normal)).normalized()
    var side := normal.cross(fwd)
    var v_fwd := v.dot(fwd)
    var v_side := v.dot(side)

    # friction is smoothed near zero speed so the solver stays stable
    var force := up * load
    force -= side * grip * load * clampf(v_side / 0.2, -1.0, 1.0)
    force -= fwd * rolling_resistance * load * clampf(v_fwd / 0.2, -1.0, 1.0)
    if drive_servo > 0 and max_drive_force > 0.0:
        var f := servo_fraction(pwm, drive_servo, drive_pwm_min, drive_pwm_max)
        if f >= 0.0:
            var drive := clampf((2.0 * f - 1.0) * max_drive_force, -grip * load, grip * load)
            force += fwd * drive
    vehicle.apply_force(force, contact - vehicle.global_position)
    _place_visuals(dt, v_fwd)


func _place_visuals(dt: float, v_fwd: float) -> void:
    if radius > 0.0:
        _spin = fmod(_spin + v_fwd / radius * dt, TAU)
    var t := Transform3D(Basis(Vector3.RIGHT, _spin), Vector3.UP * compression)
    for child in get_children():
        if child is Node3D:
            if not child.has_meta("ap_rest"):
                child.set_meta("ap_rest", child.transform)
            child.transform = t * (child.get_meta("ap_rest") as Transform3D)
