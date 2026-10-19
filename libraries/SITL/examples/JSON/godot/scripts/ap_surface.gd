@tool
@icon("res://icons/surface.svg")
class_name ArduPilotSurface
extends "res://scripts/ap_component.gd"
## A lifting surface (wing panel, tailplane, fin) using strip theory.
##
## Local axes: span along X, chord along Z with the leading edge towards
## +Z, lift positive towards +Y. The node's origin is the aerodynamic centre
## (quarter chord). Forces come from the airflow at that point, including
## the body's rotation, so roll, pitch and yaw damping arise naturally.
## Split a wing into several panels for better damping and stall behaviour.
##
## Lift: finite-wing lift slope blended into flat-plate lift past the stall
## (Beard & McLain, Small Unmanned Aircraft, 2012, sec 4.2). An optional
## trailing-edge flap (aileron, elevator, rudder) driven by a servo shifts
## the effective angle of attack by its flap effectiveness.

## span (m), along local X
@export var span: float = 1.0:
    set(v):
        span = v
        update_gizmos()
## chord (m), along local Z
@export var chord: float = 0.2:
    set(v):
        chord = v
        update_gizmos()
## aspect ratio of the whole wing this panel belongs to; 0 to use span/chord
@export var aspect_ratio: float = 0.0
## lift coefficient at zero angle of attack (camber)
@export var cl0: float = 0.0
## zero-lift drag coefficient
@export var cd0: float = 0.015
## pitching moment coefficient about the aerodynamic centre
@export var cm0: float = 0.0
## Oswald efficiency factor for induced drag
@export var oswald: float = 0.85
## angle of attack where the lift stops increasing (deg)
@export var stall_angle: float = 15.0
## downwash from the wing ahead, as d(epsilon)/d(alpha) of the vehicle's
## angle of attack: about 2 CL_alpha / (pi AR) of the wing, 0.4 to 0.5 for
## a typical tailplane, 0 for a wing
@export var downwash: float = 0.0
@export_group("Flap")
## servo driving the trailing-edge flap; 0 for none
@export_range(0, 32) var flap_servo: int = 0
@export var flap_pwm_min: float = 1000.0
@export var flap_pwm_max: float = 2000.0
## flap deflection at pwm_min and pwm_max (deg, positive trailing edge down)
@export var flap_angle_min: float = -20.0
@export var flap_angle_max: float = 20.0
## flap chord as a fraction of the panel's chord
@export_range(0.05, 0.95) var flap_chord_fraction: float = 0.25:
    set(v):
        flap_chord_fraction = v
        update_gizmos()
@export_group("")

## a child with this name is turned to show the flap deflection
const FLAP_VISUAL := "flap_visual"

var air_density: float = 1.225
## state from the last step, for display and logging
var alpha: float = 0.0
var flap: float = 0.0
var lift: float = 0.0


func gltf_key() -> String:
    return "surface"


func gltf_fields() -> Dictionary:
    return {
        "span": "span", "chord": "chord", "aspectRatio": "aspect_ratio",
        "cl0": "cl0", "cd0": "cd0", "cm0": "cm0", "oswald": "oswald",
        "stallAngle": "stall_angle", "downwash": "downwash", "flapServo": "flap_servo",
        "flapPwmMin": "flap_pwm_min", "flapPwmMax": "flap_pwm_max",
        "flapAngleMin": "flap_angle_min", "flapAngleMax": "flap_angle_max",
        "flapChordFraction": "flap_chord_fraction",
    }


func effective_aspect_ratio() -> float:
    if aspect_ratio > 0.0:
        return aspect_ratio
    return span / chord if chord > 0.0 else 1.0


## finite wing lift slope per radian (Helmbold)
func lift_slope() -> float:
    var ar := effective_aspect_ratio()
    return TAU * ar / (2.0 + sqrt(ar * ar + 4.0))


## change in angle of attack per radian of flap (thin aerofoil theory)
func flap_effectiveness() -> float:
    var theta := acos(2.0 * flap_chord_fraction - 1.0)
    return 1.0 - (theta - sin(theta)) / PI


## lift and drag coefficients at angle of attack a with flap deflection d
func coefficients(a: float, d: float) -> Vector2:
    var a_eff := a + flap_effectiveness() * d
    var a0 := deg_to_rad(stall_angle)
    var m := 50.0
    var e1 := exp(-m * (a_eff - a0))
    var e2 := exp(m * (a_eff + a0))
    var sigma := (1.0 + e1 + e2) / ((1.0 + e1) * (1.0 + e2))
    var cl_linear := cl0 + lift_slope() * a_eff
    var s := sin(a_eff)
    var c := cos(a_eff)
    var cl_plate := 2.0 * signf(a_eff) * s * s * c
    var cl := (1.0 - sigma) * cl_linear + sigma * cl_plate
    var cd_induced := cl_linear * cl_linear / (PI * oswald * effective_aspect_ratio())
    var cd := cd0 + (1.0 - sigma) * cd_induced + sigma * 2.0 * s * s
    var sd := sin(d)
    cd += 0.9 * pow(flap_chord_fraction, 1.38) * sd * sd
    return Vector2(cl, cd)


func physics_update(vehicle: RigidBody3D, pwm: PackedInt32Array, _dt: float) -> void:
    if flap_servo > 0:
        var f := servo_fraction(pwm, flap_servo, flap_pwm_min, flap_pwm_max)
        if f >= 0.0:
            flap = deg_to_rad(lerpf(flap_angle_min, flap_angle_max, f))
            var pivot := get_node_or_null(FLAP_VISUAL) as Node3D
            if pivot:
                # trailing edge down turns -Z towards -Y, which is about -X
                pivot.basis = Basis(Vector3.RIGHT, -flap)

    var xform := global_transform
    var b := xform.basis.orthonormalized()
    var v_local: Vector3 = b.transposed() * vehicle.air_velocity_at(xform.origin)
    # strip theory: spanwise flow does not contribute
    v_local.x = 0.0
    var speed := v_local.length()
    if speed < 0.1:
        lift = 0.0
        return
    alpha = atan2(-v_local.y, v_local.z)
    if downwash != 0.0:
        # downwash collapses once the wing stalls, so limit its growth
        alpha -= downwash * clampf(vehicle.alpha, -0.3, 0.3)
    var k := coefficients(alpha, flap)
    var q := 0.5 * air_density * speed * speed
    var area := span * chord
    var wind_dir := -v_local / speed
    var lift_dir := Vector3.RIGHT.cross(wind_dir)
    lift = q * area * k.x
    var force := lift_dir * lift + wind_dir * (q * area * k.y)
    vehicle.apply_force(b * force, xform.origin - vehicle.global_position)
    if cm0 != 0.0:
        # nose-up moment turns +Z towards +Y, which is about -X
        vehicle.apply_torque(b * Vector3(-q * area * chord * cm0, 0.0, 0.0))
