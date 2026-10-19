@tool
extends Node3D
## Base class for vehicle parts: rotors, lifting surfaces, hinges and wheels.
##
## Each part is serialised to glTF as one member of the ARDUPILOT_sitl node
## extension; gltf_key() names the member and gltf_fields() maps its JSON
## keys to the script's exported properties.


## name of this part's member in the ARDUPILOT_sitl node extension
func gltf_key() -> String:
    return ""


## JSON key -> property name
func gltf_fields() -> Dictionary:
    return {}


## advance the part by dt, applying any forces to vehicle
func physics_update(_vehicle: RigidBody3D, _pwm: PackedInt32Array, _dt: float) -> void:
    pass


## position of pwm on the servo's range, 0 to 1, or -1 with no output
static func servo_fraction(pwm: PackedInt32Array, servo: int, pwm_min: float, pwm_max: float) -> float:
    if servo < 1 or servo > pwm.size() or pwm_max <= pwm_min:
        return -1.0
    var v := float(pwm[servo - 1])
    if v < 500.0:
        # ArduPilot outputs 0 on a channel that is disabled
        return -1.0
    return clampf((v - pwm_min) / (pwm_max - pwm_min), 0.0, 1.0)


func to_gltf() -> Dictionary:
    var d := {}
    for key in gltf_fields():
        var v = get(gltf_fields()[key])
        d[key] = [v.x, v.y, v.z] if v is Vector3 else v
    return d


func from_gltf(d: Dictionary) -> void:
    var fields := gltf_fields()
    for key in d:
        if not fields.has(key):
            push_warning("ARDUPILOT_sitl.%s: unknown field '%s'" % [gltf_key(), key])
            continue
        var prop: String = fields[key]
        var cur = get(prop)
        var v = d[key]
        if cur is Vector3 and v is Array and v.size() == 3:
            set(prop, Vector3(v[0], v[1], v[2]))
        elif cur is int:
            set(prop, int(v))
        elif cur is float:
            set(prop, float(v))
        else:
            set(prop, v)
