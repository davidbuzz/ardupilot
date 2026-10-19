@tool
@icon("res://icons/rotor.svg")
class_name ArduPilotRotor
extends "res://scripts/ap_component.gd"
## A propeller and motor driven by one servo output.
##
## Thrust acts along the node's local +Y axis at the node's origin. Put the
## node under an ArduPilotHinge to tilt it.

## SERVOn output driving this rotor (1-based, as in SERVOn_FUNCTION)
@export_range(1, 32) var servo: int = 1
@export var pwm_min: float = 1000.0
@export var pwm_max: float = 2000.0
## static thrust at full throttle (N)
@export var max_thrust: float = 10.0:
    set(v):
        max_thrust = v
        update_gizmos()
## thrust curve: 0 = linear in throttle, 1 = quadratic. Matches MOT_THST_EXPO.
@export_range(0.0, 1.0) var thrust_expo: float = 0.65
## reaction torque per unit thrust (N.m/N)
@export var torque_ratio: float = 0.016
## spin direction seen from the tip of the thrust axis
@export_enum("ccw", "cw") var direction: String = "ccw":
    set(v):
        direction = v
        update_gizmos()
## first-order motor spin-up time constant (s)
@export var time_constant: float = 0.03
## propeller diameter (m), for display
@export var diameter: float = 0.25:
    set(v):
        diameter = v
        update_gizmos()
## inflow speed at which full-throttle thrust falls to zero (m/s), roughly
## pitch x rpm. Thrust falls linearly with inflow; 0 disables the effect.
@export var pitch_speed: float = 0.0
## rotor drag in edgewise flow: force = -coefficient x thrust x speed (s/m)
@export var inplane_drag: float = 0.0

## static thrust after the motor lag (N)
var static_thrust: float = 0.0
## thrust after inflow effects (N)
var thrust: float = 0.0


func gltf_key() -> String:
    return "rotor"


func gltf_fields() -> Dictionary:
    return {
        "servo": "servo", "pwmMin": "pwm_min", "pwmMax": "pwm_max",
        "maxThrust": "max_thrust", "thrustExpo": "thrust_expo",
        "torqueRatio": "torque_ratio", "direction": "direction",
        "timeConstant": "time_constant", "diameter": "diameter",
        "pitchSpeed": "pitch_speed", "inplaneDrag": "inplane_drag",
    }


func physics_update(vehicle: RigidBody3D, pwm: PackedInt32Array, dt: float) -> void:
    var t := maxf(servo_fraction(pwm, servo, pwm_min, pwm_max), 0.0)
    var demand := max_thrust * ((1.0 - thrust_expo) * t + thrust_expo * t * t)
    if time_constant > 0.0:
        static_thrust += (demand - static_thrust) * clampf(dt / time_constant, 0.0, 1.0)
    else:
        static_thrust = demand

    var xform := global_transform
    var axis := xform.basis.y.normalized()
    var v_air: Vector3 = vehicle.air_velocity_at(xform.origin)
    var inflow := v_air.dot(axis)
    thrust = static_thrust
    if pitch_speed > 0.0 and max_thrust > 0.0:
        # the propeller's pitch speed scales with rpm, i.e. sqrt(thrust)
        var v_pitch := pitch_speed * sqrt(static_thrust / max_thrust)
        if v_pitch > 0.01:
            thrust *= clampf(1.0 - inflow / v_pitch, 0.0, 1.5)

    var force := axis * thrust
    if inplane_drag > 0.0:
        force -= inplane_drag * thrust * (v_air - axis * inflow)
    var offset := xform.origin - vehicle.global_position
    vehicle.apply_force(force, offset)
    # a ccw rotor's reaction torque on the airframe is clockwise
    var spin := 1.0 if direction == "ccw" else -1.0
    vehicle.apply_torque(-axis * (spin * torque_ratio * thrust))

    # cosmetic: spin any attached propeller meshes
    if max_thrust > 0.0:
        var rate := spin * 200.0 * sqrt(static_thrust / max_thrust)
        for child in get_children():
            if child is Node3D:
                child.rotate_object_local(Vector3.UP, rate * dt)
