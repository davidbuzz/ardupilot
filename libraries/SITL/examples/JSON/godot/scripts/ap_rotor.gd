@tool
@icon("res://icons/rotor.svg")
class_name ArduPilotRotor
extends Marker3D
## A propeller/motor driven by one ArduPilot servo output.
##
## Thrust acts along the node's local +Y axis (glTF "up"), at the node's
## origin, on the nearest RigidBody3D ancestor. Serialised to glTF as the
## "rotor" member of the ARDUPILOT_sitl node extension.

## SERVOn output driving this rotor (1-based, as in SERVOn_FUNCTION)
@export_range(1, 32) var servo: int = 1
@export var pwm_min: float = 1000.0
@export var pwm_max: float = 2000.0
## static thrust at full throttle (N)
@export var max_thrust: float = 10.0
## thrust curve: 0 = linear in throttle, 1 = quadratic. Matches MOT_THST_EXPO.
@export_range(0.0, 1.0) var thrust_expo: float = 0.65
## reaction torque per unit thrust (N.m/N)
@export var torque_ratio: float = 0.016
## spin direction viewed from above (looking down the thrust axis)
@export_enum("ccw", "cw") var direction: String = "ccw"
## first-order motor spin-up time constant (s)
@export var time_constant: float = 0.03

var thrust: float = 0.0


func throttle_from_pwm(pwm: float) -> float:
    if pwm_max <= pwm_min:
        return 0.0
    return clampf((pwm - pwm_min) / (pwm_max - pwm_min), 0.0, 1.0)


## advance the motor model by dt and apply its force and torque to body
func update(body: RigidBody3D, pwm: float, dt: float) -> void:
    var t := throttle_from_pwm(pwm)
    var demand := max_thrust * ((1.0 - thrust_expo) * t + thrust_expo * t * t)
    if time_constant > 0.0:
        thrust += (demand - thrust) * clampf(dt / time_constant, 0.0, 1.0)
    else:
        thrust = demand

    var xform := global_transform
    var axis := xform.basis.y.normalized()
    body.apply_force(axis * thrust, xform.origin - body.global_position)
    # a ccw rotor's reaction torque on the airframe is clockwise from above
    var spin := 1.0 if direction == "ccw" else -1.0
    body.apply_torque(-axis * (spin * torque_ratio * thrust))

    # cosmetic: spin any attached propeller meshes
    if max_thrust > 0.0:
        var rate := spin * 200.0 * sqrt(thrust / max_thrust)
        for child in get_children():
            if child is Node3D:
                child.rotate_object_local(Vector3.UP, rate * dt)


func to_gltf() -> Dictionary:
    return {
        "servo": servo,
        "pwmMin": pwm_min,
        "pwmMax": pwm_max,
        "maxThrust": max_thrust,
        "thrustExpo": thrust_expo,
        "torqueRatio": torque_ratio,
        "direction": direction,
        "timeConstant": time_constant,
    }


func from_gltf(d: Dictionary) -> void:
    servo = int(d.get("servo", servo))
    pwm_min = float(d.get("pwmMin", pwm_min))
    pwm_max = float(d.get("pwmMax", pwm_max))
    max_thrust = float(d.get("maxThrust", max_thrust))
    thrust_expo = float(d.get("thrustExpo", thrust_expo))
    torque_ratio = float(d.get("torqueRatio", torque_ratio))
    direction = str(d.get("direction", direction))
    time_constant = float(d.get("timeConstant", time_constant))
