extends Node3D
## ArduPilot SITL physics backend using Godot's built-in Jolt physics.
##
## Usage (arguments after "--" are ours):
##   godot --path . -- --vehicle=res://models/quadx.gltf --port=9002
##
## Each servo packet from SITL advances the physics by exactly one step, so
## the simulation runs in lockstep with ArduPilot. Physics is paused while
## SITL is not sending. Run Godot with --fixed-fps <n> to let it go faster
## than real time when SITL is run with --speedup.

const Bridge := preload("res://scripts/sitl_bridge.gd")
const Vehicle := preload("res://scripts/ap_vehicle.gd")
const GLTFPhysics := preload("res://scripts/gltf_physics.gd")

var options := {
    "vehicle": "res://models/quadx.gltf",
    "world": "",
    "port": "9002",
    "spawn-height": "0.0",
}

var bridge := Bridge.new()
var vehicle: Vehicle
var sim_time := 0.0
var last_dt := 0.0
var awaiting_reply := false
var connected := false
var camera: Camera3D
var hud: Label


func _ready() -> void:
    for arg in OS.get_cmdline_user_args():
        var kv := arg.trim_prefix("--").split("=", true, 1)
        if kv.size() == 2:
            options[kv[0]] = kv[1]

    # physics must stand still until SITL drives it
    PhysicsServer3D.set_active(false)
    Engine.max_physics_steps_per_frame = 64

    if options["world"] != "":
        var world := _load_model(options["world"])
        if world:
            add_child(world)
    else:
        _build_default_world()

    var model := _load_model(options["vehicle"])
    if model == null:
        get_tree().quit(1)
        return
    model.position.y += float(options["spawn-height"])
    add_child(model)
    vehicle = _find_vehicle(model)
    if vehicle == null:
        push_error("%s has no ArduPilotVehicle body" % options["vehicle"])
        get_tree().quit(1)
        return
    print("vehicle '%s': mass %.3f kg, %d rotors" % [vehicle.name, vehicle.mass, vehicle.get_rotors().size()])
    for r in vehicle.get_rotors():
        var p := Vehicle.TO_NED * (vehicle.global_transform.affine_inverse() * r.global_position)
        print("  servo %d at FRD (%.3f, %.3f, %.3f) %s, max %.1f N" % [r.servo, p.x, p.y, p.z, r.direction, r.max_thrust])

    var err := bridge.listen(int(options["port"]))
    if err != OK:
        push_error("cannot listen on UDP port %s: %s" % [options["port"], error_string(err)])
        get_tree().quit(1)
        return
    print("waiting for SITL on UDP port %s" % options["port"])

    if DisplayServer.get_name() != "headless":
        _build_view()


func _load_model(path: String) -> Node3D:
    var ext := path.get_extension().to_lower()
    if ext == "gltf" or ext == "glb":
        # read the source file directly, so editing it needs no re-import
        var abs_path := ProjectSettings.globalize_path(path)
        return GLTFPhysics.new().load_gltf(abs_path)
    var res := load(path)
    if res is PackedScene:
        return res.instantiate() as Node3D
    push_error("cannot load %s" % path)
    return null


func _find_vehicle(n: Node) -> Vehicle:
    if n is Vehicle:
        return n
    for c in n.get_children():
        var v := _find_vehicle(c)
        if v:
            return v
    return null


func _physics_process(delta: float) -> void:
    if vehicle == null:
        return
    if awaiting_reply:
        bridge.send_state(sim_time, vehicle.sitl_state(last_dt))
        awaiting_reply = false

    var timeout_us := 1000000 if connected else 1000
    if not bridge.wait_for_servos(timeout_us):
        PhysicsServer3D.set_active(false)
        if connected:
            print("SITL stopped sending, physics paused")
            connected = false
        return

    if not connected:
        print("SITL connected from %s:%d, frame rate %d Hz" % [bridge.sitl_ip, bridge.sitl_port, bridge.frame_rate])
        connected = true
    if bridge.frame_count == 0 and sim_time > 0.0:
        print("SITL restarted, resetting vehicle")
        vehicle.reset()
        sim_time = 0.0
    if bridge.frame_rate > 0 and bridge.frame_rate != Engine.physics_ticks_per_second:
        # takes effect from the next frame; sim_time follows the real step
        Engine.physics_ticks_per_second = bridge.frame_rate

    vehicle.apply_actuators(bridge.pwm, delta)
    PhysicsServer3D.set_active(true)
    sim_time += delta
    last_dt = delta
    awaiting_reply = true


func _process(_delta: float) -> void:
    if camera == null or vehicle == null:
        return
    var target := vehicle.global_position
    var back := vehicle.global_basis.z
    back.y = 0.0
    back = back.normalized() if back.length() > 0.1 else Vector3.FORWARD
    var want := target - back * 3.0 + Vector3.UP * 1.2
    camera.global_position = camera.global_position.lerp(want, 0.1)
    camera.look_at(target)
    var ned := Vehicle.TO_NED * target
    hud.text = "%s  t=%.2fs  alt=%.2fm  N=%.2f E=%.2f" % [
            "SITL connected" if connected else "waiting for SITL", sim_time, -ned.z, ned.x, ned.y]


func _build_default_world() -> void:
    var ground := StaticBody3D.new()
    ground.name = "ground"
    var cs := CollisionShape3D.new()
    var box := BoxShape3D.new()
    box.size = Vector3(2000, 1, 2000)
    cs.shape = box
    cs.position.y = -0.5
    ground.add_child(cs)
    var mat := PhysicsMaterial.new()
    mat.friction = 0.8
    ground.physics_material_override = mat
    add_child(ground)


func _build_view() -> void:
    var env := WorldEnvironment.new()
    env.environment = Environment.new()
    var sky := Sky.new()
    sky.sky_material = ProceduralSkyMaterial.new()
    env.environment.background_mode = Environment.BG_SKY
    env.environment.sky = sky
    env.environment.ambient_light_source = Environment.AMBIENT_SOURCE_SKY
    add_child(env)

    var sun := DirectionalLight3D.new()
    sun.rotation_degrees = Vector3(-50, 30, 0)
    sun.shadow_enabled = true
    add_child(sun)

    # chequered ground, 1 m squares, for a sense of motion
    var img := Image.create(2, 2, false, Image.FORMAT_RGB8)
    img.set_pixel(0, 0, Color(0.35, 0.5, 0.3))
    img.set_pixel(1, 1, Color(0.35, 0.5, 0.3))
    img.set_pixel(1, 0, Color(0.3, 0.42, 0.25))
    img.set_pixel(0, 1, Color(0.3, 0.42, 0.25))
    var tex := ImageTexture.create_from_image(img)
    var mat := StandardMaterial3D.new()
    mat.albedo_texture = tex
    mat.texture_filter = BaseMaterial3D.TEXTURE_FILTER_NEAREST
    mat.uv1_scale = Vector3(500, 500, 1)
    var plane := PlaneMesh.new()
    plane.size = Vector2(2000, 2000)
    plane.material = mat
    var mi := MeshInstance3D.new()
    mi.mesh = plane
    add_child(mi)

    camera = Camera3D.new()
    camera.position = Vector3(0, 1.5, -4)
    add_child(camera)
    camera.make_current()

    var layer := CanvasLayer.new()
    hud = Label.new()
    hud.position = Vector2(10, 10)
    layer.add_child(hud)
    add_child(layer)
