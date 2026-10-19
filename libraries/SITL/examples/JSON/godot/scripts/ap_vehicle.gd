@tool
@icon("res://icons/vehicle.svg")
class_name ArduPilotVehicle
extends RigidBody3D
## The rigid body whose state is reported to ArduPilot SITL.
##
## The IMU is taken to be at this node's origin, with the glTF axis
## convention: +Z forward, -X right, +Y up. The world frame uses the same
## mapping, so +Z is north, -X is east and +Y is up.

const Rotor := preload("res://scripts/ap_rotor.gd")

## maps a glTF/Godot vector (x, y, z) to NED/FRD (z, -x, -y)
const TO_NED := Basis(Vector3(0, -1, 0), Vector3(0, 0, -1), Vector3(1, 0, 0))

## quadratic drag area (Cd * A, m^2) along the body's local X, Y and Z axes
@export var drag_area: Vector3 = Vector3.ZERO
@export var air_density: float = 1.225

var _rotors: Array[Rotor] = []
var _initial_transform: Transform3D
var _last_velocity: Vector3
var _have_last_velocity := false


func _ready() -> void:
    if Engine.is_editor_hint():
        return
    can_sleep = false
    linear_damp_mode = DAMP_MODE_REPLACE
    angular_damp_mode = DAMP_MODE_REPLACE
    linear_damp = 0.0
    angular_damp = 0.0
    _initial_transform = global_transform
    _rotors.clear()
    _find_rotors(self)


func _find_rotors(n: Node) -> void:
    for c in n.get_children():
        if c is Rotor:
            _rotors.append(c)
        # a nested rigid body owns its own actuators
        if not (c is RigidBody3D):
            _find_rotors(c)


func get_rotors() -> Array[Rotor]:
    return _rotors


## apply actuator and aerodynamic forces for the coming physics step
func apply_actuators(pwm: PackedInt32Array, dt: float) -> void:
    for r in _rotors:
        var idx := r.servo - 1
        r.update(self, float(pwm[idx]) if idx < pwm.size() else 0.0, dt)

    if drag_area != Vector3.ZERO:
        var b := global_basis.orthonormalized()
        var v_local := b.transposed() * linear_velocity
        var f_local := -0.5 * air_density * drag_area * v_local * v_local.abs()
        apply_central_force(b * f_local)


## put the vehicle back where it started, at rest
func reset() -> void:
    var rid := get_rid()
    PhysicsServer3D.body_set_state(rid, PhysicsServer3D.BODY_STATE_TRANSFORM, _initial_transform)
    PhysicsServer3D.body_set_state(rid, PhysicsServer3D.BODY_STATE_LINEAR_VELOCITY, Vector3.ZERO)
    PhysicsServer3D.body_set_state(rid, PhysicsServer3D.BODY_STATE_ANGULAR_VELOCITY, Vector3.ZERO)
    for r in _rotors:
        r.thrust = 0.0
    _have_last_velocity = false


## vehicle state in ArduPilot's frames after the last physics step of length dt
func sitl_state(dt: float) -> Dictionary:
    var st := PhysicsServer3D.body_get_direct_state(get_rid())
    var b := st.transform.basis.orthonormalized()
    var w := st.angular_velocity
    # linear_velocity is that of the centre of mass; move it to the IMU
    var v := st.linear_velocity + w.cross(-st.center_of_mass)

    var accel_world := Vector3.ZERO
    if _have_last_velocity and dt > 0.0:
        accel_world = (v - _last_velocity) / dt
    _last_velocity = v
    _have_last_velocity = true
    var specific_force := accel_world - st.total_gravity

    var dcm := TO_NED * b * TO_NED.transposed()
    return {
        "gyro": TO_NED * (b.transposed() * w),
        "accel_body": TO_NED * (b.transposed() * specific_force),
        "position": TO_NED * st.transform.origin,
        "velocity": TO_NED * v,
        "quaternion": dcm.get_rotation_quaternion(),
    }
