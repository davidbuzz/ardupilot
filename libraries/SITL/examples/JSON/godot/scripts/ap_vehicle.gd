@tool
@icon("res://icons/vehicle.svg")
class_name ArduPilotVehicle
extends RigidBody3D
## The rigid body whose state is reported to ArduPilot SITL.
##
## The IMU is taken to be at this node's origin, with the glTF axis
## convention: +Z forward, -X right, +Y up. The world frame uses the same
## mapping, so +Z is north, -X is east and +Y is up.

const Component := preload("res://scripts/ap_component.gd")
const Rotor := preload("res://scripts/ap_rotor.gd")
const Hinge := preload("res://scripts/ap_hinge.gd")
const Surface := preload("res://scripts/ap_surface.gd")

## maps a glTF/Godot vector (x, y, z) to NED/FRD (z, -x, -y)
const TO_NED := Basis(Vector3(0, -1, 0), Vector3(0, 0, -1), Vector3(1, 0, 0))

## quadratic drag area (Cd * A, m^2) along the body's local X, Y and Z axes
@export var drag_area: Vector3 = Vector3.ZERO
@export var air_density: float = 1.225

## wind velocity in the world frame (m/s, Godot axes)
var wind: Vector3 = Vector3.ZERO
## angle of attack of the body (rad), updated every step
var alpha: float = 0.0

var _components: Array[Component] = []
var _initial_transform: Transform3D
var _initial_velocity: Vector3
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
    _initial_velocity = linear_velocity
    _components.clear()
    _find_components(self)
    # pivots first, so parts below them use this step's pose
    _components.sort_custom(func(a, b): return (a is Hinge) and not (b is Hinge))
    for c in _components:
        if c is Surface:
            c.air_density = air_density


func _find_components(n: Node) -> void:
    for c in n.get_children():
        if c is Component:
            _components.append(c)
        # a nested rigid body owns its own parts
        if not (c is RigidBody3D):
            _find_components(c)


func get_components() -> Array[Component]:
    return _components


func get_rotors() -> Array:
    return _components.filter(func(c): return c is Rotor)


func has_surfaces() -> bool:
    return _components.any(func(c): return c is Surface)


## velocity of a point fixed to the body, in the world frame
func velocity_at(point: Vector3) -> Vector3:
    var st := PhysicsServer3D.body_get_direct_state(get_rid())
    var com := st.transform.origin + st.center_of_mass
    return st.linear_velocity + st.angular_velocity.cross(point - com)


## velocity of a point fixed to the body relative to the air
func air_velocity_at(point: Vector3) -> Vector3:
    return velocity_at(point) - wind


## extra airflow over a surface from the slipstreams of the vehicle's
## rotors, as a velocity relative to the air; see ArduPilotRotor.propwash_at
func propwash_at(point: Vector3, v_air: Vector3, span_dir := Vector3.ZERO, span := 0.0) -> Vector3:
    var wash := Vector3.ZERO
    for c in _components:
        if c is Rotor:
            wash += c.propwash_at(point, v_air, span_dir, span)
    return wash


## apply actuator and aerodynamic forces for the coming physics step
func apply_actuators(pwm: PackedInt32Array, dt: float) -> void:
    var v_body := global_basis.orthonormalized().transposed() * (linear_velocity - wind)
    alpha = atan2(-v_body.y, v_body.z) if absf(v_body.z) + absf(v_body.y) > 0.1 else 0.0
    for c in _components:
        c.physics_update(self, pwm, dt)

    if drag_area != Vector3.ZERO:
        var b := global_basis.orthonormalized()
        var v_local := b.transposed() * (linear_velocity - wind)
        var f_local := -0.5 * air_density * drag_area * v_local * v_local.abs()
        apply_central_force(b * f_local)


func set_initial_velocity(v: Vector3) -> void:
    _initial_velocity = v
    linear_velocity = v


## put the vehicle back where it started
func reset() -> void:
    var rid := get_rid()
    PhysicsServer3D.body_set_state(rid, PhysicsServer3D.BODY_STATE_TRANSFORM, _initial_transform)
    PhysicsServer3D.body_set_state(rid, PhysicsServer3D.BODY_STATE_LINEAR_VELOCITY, _initial_velocity)
    PhysicsServer3D.body_set_state(rid, PhysicsServer3D.BODY_STATE_ANGULAR_VELOCITY, Vector3.ZERO)
    for r in get_rotors():
        r.static_thrust = 0.0
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
    var s := {
        "gyro": TO_NED * (b.transposed() * w),
        "accel_body": TO_NED * (b.transposed() * specific_force),
        "position": TO_NED * st.transform.origin,
        "velocity": TO_NED * v,
        "quaternion": dcm.get_rotation_quaternion(),
    }
    if has_surfaces():
        # a pitot tube along the body's forward axis
        s["airspeed"] = maxf((b.transposed() * (v - wind)).z, 0.0)
    return s
