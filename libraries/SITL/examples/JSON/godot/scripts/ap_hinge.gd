@tool
@icon("res://icons/hinge.svg")
class_name ArduPilotHinge
extends "res://scripts/ap_component.gd"
## A pivot driven by one servo output. Everything below this node (meshes,
## lifting surfaces, rotors, wheels) rotates about the node's local X axis,
## by the right-hand rule. Use it for control surfaces, tilt rotors,
## steerable wheels or gimbals.
##
## The pivot stays part of its rigid body: its mass is not modelled as
## moving, and colliders do not follow it.

## SERVOn output driving this pivot (1-based)
@export_range(1, 32) var servo: int = 1
@export var pwm_min: float = 1000.0
@export var pwm_max: float = 2000.0
## angle at pwm_min (deg)
@export var angle_min: float = -30.0:
    set(v):
        angle_min = v
        update_gizmos()
## angle at pwm_max (deg)
@export var angle_max: float = 30.0:
    set(v):
        angle_max = v
        update_gizmos()
## servo slew rate (deg/s), 0 for instant
@export var max_rate: float = 300.0
## deflection shown in the editor (deg). Does not affect the simulation.
@export_range(-180.0, 180.0) var preview_angle: float = 0.0:
    set(v):
        preview_angle = v
        if Engine.is_editor_hint() and is_inside_tree():
            _set_angle(v)

## current angle (deg)
var angle: float = 0.0
## transform at zero angle
var rest: Transform3D
var _applying := false
var _initialised := false


func gltf_key() -> String:
    return "hinge"


func gltf_fields() -> Dictionary:
    return {
        "servo": "servo", "pwmMin": "pwm_min", "pwmMax": "pwm_max",
        "angleMin": "angle_min", "angleMax": "angle_max", "maxRate": "max_rate",
    }


func _enter_tree() -> void:
    set_notify_local_transform(true)
    var first := not _initialised
    if first:
        # a saved transform includes the preview deflection
        _initialised = true
        angle = preview_angle
    rest = transform * _rotation(-angle)
    if first and not Engine.is_editor_hint():
        _set_angle(0.0)


func _notification(what: int) -> void:
    if what == NOTIFICATION_LOCAL_TRANSFORM_CHANGED and not _applying:
        # moved in the editor: keep the deflection, move the pivot
        rest = transform * _rotation(-angle)


static func _rotation(deg: float) -> Transform3D:
    return Transform3D(Basis(Vector3.RIGHT, deg_to_rad(deg)), Vector3.ZERO)


func _set_angle(deg: float) -> void:
    angle = deg
    _applying = true
    transform = rest * _rotation(deg)
    _applying = false


func physics_update(_vehicle: RigidBody3D, pwm: PackedInt32Array, dt: float) -> void:
    var f := servo_fraction(pwm, servo, pwm_min, pwm_max)
    if f < 0.0:
        return
    var target := lerpf(angle_min, angle_max, f)
    if max_rate > 0.0:
        var step := max_rate * dt
        target = clampf(target, angle - step, angle + step)
    if target != angle:
        _set_angle(target)
