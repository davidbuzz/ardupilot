@tool
extends EditorPlugin
## Drives the Godot editor through the vehicle editing tutorial, with
## captions and a moving mouse pointer, for screen recording. Not part of
## the project: record_tutorial.sh installs it as an editor plugin in a
## copy of the project. res://scratch/mode.txt says what to do: "prep"
## converts and saves the plane scene then quits, so the project is
## imported; "film" waits for res://scratch/go, runs the steps, then
## writes res://scratch/done.

const GLTFPhysics := preload("res://scripts/gltf_physics.gd")
const Report := preload("res://scripts/vehicle_report.gd")
const Rotor := preload("res://scripts/ap_rotor.gd")
const Hinge := preload("res://scripts/ap_hinge.gd")

const SCENE := "res://models/plane.tscn"

var caption: Label
var panel: PanelContainer
var ring: Panel
var step_label: Label


func _ready() -> void:
    var mode := FileAccess.get_file_as_string("res://scratch/mode.txt").strip_edges()
    await wait(4.0)
    if mode == "prep":
        convert_plane()
        await wait(3.0)
        EditorInterface.open_scene_from_path(SCENE)
        await wait(3.0)
        EditorInterface.save_scene()
        await wait(1.0)
        get_tree().quit()
        return
    if mode != "film":
        return
    EditorInterface.get_editor_settings().set_setting("interface/editor/update_continuously", true)
    while not FileAccess.file_exists("res://scratch/go"):
        await wait(0.2)
    build_overlay()
    await film()
    FileAccess.open("res://scratch/done", FileAccess.WRITE).store_string("done")


func wait(seconds: float) -> void:
    await get_tree().create_timer(seconds).timeout


# ------------------------------------------------------------- overlay

func build_overlay() -> void:
    var base := EditorInterface.get_base_control()
    panel = PanelContainer.new()
    var sb := StyleBoxFlat.new()
    sb.bg_color = Color(0.05, 0.05, 0.1, 0.88)
    sb.border_color = Color(1.0, 0.55, 0.15)
    sb.set_border_width_all(2)
    sb.set_corner_radius_all(8)
    sb.content_margin_left = 18
    sb.content_margin_right = 18
    sb.content_margin_top = 10
    sb.content_margin_bottom = 10
    panel.add_theme_stylebox_override("panel", sb)
    var vb := VBoxContainer.new()
    step_label = Label.new()
    step_label.add_theme_font_size_override("font_size", 17)
    step_label.add_theme_color_override("font_color", Color(1.0, 0.65, 0.25))
    caption = Label.new()
    caption.add_theme_font_size_override("font_size", 23)
    caption.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
    caption.custom_minimum_size = Vector2(1100, 0)
    vb.add_child(step_label)
    vb.add_child(caption)
    panel.add_child(vb)
    panel.z_index = 4096
    panel.top_level = true
    base.add_child(panel)

    ring = Panel.new()
    var rs := StyleBoxFlat.new()
    rs.bg_color = Color(1, 0.4, 0.1, 0.0)
    rs.border_color = Color(1.0, 0.45, 0.1, 0.95)
    rs.set_border_width_all(4)
    rs.set_corner_radius_all(30)
    ring.add_theme_stylebox_override("panel", rs)
    ring.size = Vector2(60, 60)
    ring.z_index = 4096
    ring.top_level = true
    ring.visible = false
    base.add_child(ring)


func say(step: String, text: String) -> void:
    step_label.text = step
    caption.text = text
    await get_tree().process_frame
    var base_size := EditorInterface.get_base_control().size
    panel.reset_size()
    panel.position = Vector2((base_size.x - panel.size.x) * 0.5, base_size.y - panel.size.y - 70)


func point(target: Vector2, click := true) -> void:
    var start := Vector2(DisplayServer.mouse_get_position())
    var steps := 18
    for i in range(1, steps + 1):
        var p := start.lerp(target, float(i) / steps)
        DisplayServer.warp_mouse(Vector2i(p))
        await wait(0.03)
    if click:
        ring.position = target - ring.size * 0.5
        ring.visible = true
        await wait(0.5)
        ring.visible = false


# ------------------------------------------------------------- finding things on screen

func find_first(root: Node, cls: String) -> Node:
    var found := root.find_children("*", cls, true, false)
    return found[0] if found.size() > 0 else null


func tree_item_pos(node: Node) -> Vector2:
    var ste := find_first(EditorInterface.get_base_control(), "SceneTreeEditor")
    if ste == null:
        return Vector2(150, 200)
    var tree := find_first(ste, "Tree") as Tree
    var item := tree.get_root()
    while item != null:
        if item.get_metadata(0) == node.get_path():
            tree.scroll_to_item(item)
            var r := tree.get_item_area_rect(item)
            return tree.global_position + r.position + Vector2(60, r.size.y * 0.5) - Vector2(0, tree.get_scroll().y)
        item = item.get_next_in_tree()
    return tree.global_position + Vector2(100, 40)


func property_pos(prop: String) -> Vector2:
    var inspector := EditorInterface.get_inspector()
    for c in inspector.find_children("*", "EditorProperty", true, false):
        var ep := c as EditorProperty
        if ep.get_edited_property() == StringName(prop) and not ep.is_visible_in_tree():
            var n: Node = ep.get_parent()
            while n != null and n != inspector:
                if n.get_class() == "EditorInspectorSection" and n.has_method("unfold"):
                    n.call("unfold")
                n = n.get_parent()
    await wait(0.3)
    for c in inspector.find_children("*", "EditorProperty", true, false):
        var ep := c as EditorProperty
        if ep.get_edited_property() == StringName(prop) and ep.is_visible_in_tree():
            inspector.ensure_control_visible(ep)
            await get_tree().process_frame
            return ep.global_position + Vector2(ep.size.x * 0.75, ep.size.y * 0.5)
    return inspector.global_position + Vector2(150, 150)


func viewport_pos(world: Vector3) -> Vector2:
    var vp := EditorInterface.get_editor_viewport_3d(0)
    var cam := vp.get_camera_3d()
    var container := vp.get_parent() as Control
    var p := cam.unproject_position(world)
    return container.global_position + p * (container.size / Vector2(vp.size))


func menu_pos(title: String) -> Vector2:
    var bar := find_first(EditorInterface.get_base_control(), "MenuBar") as MenuBar
    if bar == null:
        return Vector2(90, 20)
    var x := 0.0
    for i in bar.get_menu_count():
        var t := bar.get_menu_title(i)
        var w := bar.get_theme_font("font").get_string_size(t, HORIZONTAL_ALIGNMENT_LEFT, -1,
                bar.get_theme_font_size("font_size")).x + 16
        if t == title:
            return bar.global_position + Vector2(x + w * 0.5, bar.size.y * 0.5)
        x += w
    return bar.global_position + Vector2(90, bar.size.y * 0.5)


## a left click, through the editor's own input handling
func click_at(pos: Vector2) -> void:
    for pressed in [true, false]:
        var ev := InputEventMouseButton.new()
        ev.button_index = MOUSE_BUTTON_LEFT
        ev.pressed = pressed
        ev.position = pos
        ev.global_position = pos
        Input.parse_input_event(ev)
        await get_tree().process_frame


## slide the pointer from one point towards another until a menu highlights an item
func slide_to_item(menu: PopupMenu, idx: int, from: Vector2, to: Vector2) -> void:
    var steps := 60
    for i in range(1, steps + 1):
        DisplayServer.warp_mouse(Vector2i(from.lerp(to, float(i) / steps)))
        await wait(0.02)
        if menu.get_focused_item() == idx:
            break
    # settle in the middle of the item
    for i in 3:
        var m := Vector2(DisplayServer.mouse_get_position()) + Vector2(0, 3)
        DisplayServer.warp_mouse(Vector2i(m))
        await wait(0.03)
        if menu.get_focused_item() != idx:
            DisplayServer.warp_mouse(Vector2i(m - Vector2(0, 3)))
            break


## open Project > Tools as a user would and rest the pointer on one of our commands
func show_tools_menu(item_text: String) -> void:
    var tools: PopupMenu = null
    var item_idx := 0
    for pm in EditorInterface.get_base_control().find_children("*", "PopupMenu", true, false):
        var m := pm as PopupMenu
        for i in m.item_count:
            if m.get_item_text(i) == item_text:
                tools = m
                item_idx = i
    if tools == null:
        return
    var project := tools.get_parent() as PopupMenu
    var title := menu_pos("Project")
    await point(title)
    await click_at(title)
    await wait(0.6)
    if project == null or not project.visible:
        tools.popup(Rect2i(Vector2i(title) + Vector2i(-30, 14), Vector2i(10, 10)))
    else:
        var tools_idx := 0
        for i in project.item_count:
            if project.get_item_text(i) == "Tools":
                tools_idx = i
        var x := float(project.position.x) + project.size.x * 0.35
        await slide_to_item(project, tools_idx, Vector2(x, project.position.y + 4),
                Vector2(x, project.position.y + project.size.y - 4))
        var t := 0.0
        while not tools.visible and t < 2.0:
            await wait(0.1)
            t += 0.1
    await wait(0.4)
    var here := Vector2(DisplayServer.mouse_get_position())
    var tx := float(tools.position.x) + tools.size.x * 0.4
    # across into the submenu at the same height, then down to the item
    await point(Vector2(tx, here.y), false)
    await slide_to_item(tools, item_idx, Vector2(tx, here.y),
            Vector2(tx, tools.position.y + tools.size.y - 4))
    await point(Vector2(DisplayServer.mouse_get_position()))
    await wait(1.2)
    tools.hide()
    if project:
        project.hide()


# ------------------------------------------------------------- the tutorial

func convert_plane() -> void:
    var ext := GLTFPhysics.new()
    var root := ext.load_gltf(ProjectSettings.globalize_path("res://models/plane.gltf"))
    var scene := PackedScene.new()
    scene.pack(root)
    ResourceSaver.save(scene, SCENE)
    root.free()
    EditorInterface.get_resource_filesystem().scan()


func scene_node(name: String) -> Node3D:
    return EditorInterface.get_edited_scene_root().find_child(name, true, false) as Node3D


func select(n: Node) -> void:
    EditorInterface.get_selection().clear()
    EditorInterface.edit_node(n)
    EditorInterface.get_selection().add_node(n)


func film() -> void:
    await say("ArduPilot SITL with Godot physics",
            "How to edit a vehicle model in the Godot editor: parts, pivots, thrust and surfaces.")
    await wait(5.0)

    await say("Step 1 - Open the project",
            "Open libraries/SITL/examples/JSON/godot in Godot 4.4 or later (Project Manager > Import, "
            + "or: godot --editor --path libraries/SITL/examples/JSON/godot). The models folder holds the vehicles.")
    var fs := find_first(EditorInterface.get_base_control(), "FileSystemDock") as Control
    if fs:
        var ftree := find_first(fs, "Tree") as Tree
        var item := ftree.get_root() if ftree else null
        while item != null:
            if item.get_text(0) == "models":
                item.collapsed = false
                await wait(0.3)
                var r := ftree.get_item_area_rect(item)
                await point(ftree.global_position + r.position + Vector2(40, r.size.y * 0.5) - Vector2(0, ftree.get_scroll().y))
                break
            item = item.get_next_in_tree()
    await wait(5.0)

    await say("Step 2 - Make an editable copy of a vehicle",
            "Project > Tools > ArduPilot: convert glTF vehicle to scene..., then pick models/plane.gltf. "
            + "It saves models/plane.tscn and opens it.")
    await show_tools_menu("ArduPilot: convert glTF vehicle to scene...")
    convert_plane()
    await wait(1.5)
    EditorInterface.open_scene_from_path(SCENE)
    await wait(5.0)

    var body := scene_node("plane")
    await say("Step 3 - The parts of the vehicle",
            "The Scene tree shows the body (ArduPilotVehicle: mass, inertia, drag) and its parts: "
            + "lifting surfaces, the propeller, a steering pivot and wheels. Colliders give the shape for contact.")
    await point(tree_item_pos(body))
    select(body)
    await wait(2.0)
    await point(await property_pos("mass"))
    await wait(4.0)

    var prop := scene_node("propeller")
    await say("Step 4 - Select a part",
            "Click the propeller. The Inspector shows its ArduPilotRotor settings: servo output, "
            + "maximum thrust, thrust curve, spin direction, diameter and pitch speed.")
    await point(tree_item_pos(prop))
    select(prop)
    await wait(4.0)

    await say("Step 5 - Change its thrust",
            "Type new values in the Inspector: here Max Thrust 34 -> 40 N and Pitch Speed 90 -> 95 m/s. "
            + "The orange thrust arrow in the 3D view grows with it.")
    await point(await property_pos("max_thrust"))
    prop.set("max_thrust", 40.0)
    await wait(2.0)
    await point(await property_pos("pitch_speed"))
    prop.set("pitch_speed", 95.0)
    await wait(4.0)

    await say("Step 6 - Move a part",
            "Drag the gizmo arrows in the 3D view, or type in Transform > Position. "
            + "+Z is forward, +Y is up and +X is left. Here the propeller moves 10 cm forward.")
    await point(await property_pos("position"))
    await wait(1.0)
    await point(viewport_pos(prop.global_position))
    for i in 20:
        prop.position.z += 0.005
        await wait(0.1)
    await wait(3.0)

    var aileron := scene_node("wing_left_outer")
    await say("Step 7 - Control surfaces",
            "Select a wing panel (ArduPilotSurface). Span and chord size it; Flap Servo, Flap Angle Min/Max "
            + "and Flap Chord Fraction make it an aileron on SERVO1. The gizmo draws the outline and hinge line.")
    await point(tree_item_pos(aileron))
    select(aileron)
    await wait(2.0)
    await point(await property_pos("flap_servo"))
    await wait(2.0)
    await point(await property_pos("flap_angle_max"))
    aileron.set("flap_angle_max", 25.0)
    await wait(4.0)

    await say("Step 8 - Add parts",
            "Select the body, click + (Add Child Node), search for ArduPilot and pick a part. Here an "
            + "ArduPilotHinge goes on the left wing tip, then an ArduPilotRotor under it: a tilt rotor.")
    await point(tree_item_pos(body))
    select(body)
    await wait(1.0)
    var plus := find_first(find_first(EditorInterface.get_base_control(), "SceneTreeDock"), "Button") as Control
    if plus:
        await point(plus.global_position + plus.size * 0.5)
    await wait(1.0)
    var root := EditorInterface.get_edited_scene_root()
    var hinge := Hinge.new()
    hinge.name = "tilt_hinge"
    hinge.position = Vector3(0.8, 0.12, 0.05)
    body.add_child(hinge)
    hinge.owner = root
    hinge.set("servo", 6)
    hinge.set("angle_min", 0.0)
    hinge.set("angle_max", 90.0)
    await wait(1.0)
    await point(tree_item_pos(hinge))
    select(hinge)
    await wait(1.5)
    if plus:
        await point(plus.global_position + plus.size * 0.5)
    await wait(0.5)
    var motor := Rotor.new()
    motor.name = "tilt_motor"
    hinge.add_child(motor)
    motor.owner = root
    motor.position = Vector3(0, 0.3, 0)
    motor.set("servo", 5)
    motor.set("max_thrust", 10.0)
    motor.set("diameter", 0.3)
    # something to see: a nacelle up to the rotor and its disc
    var nacelle := MeshInstance3D.new()
    var box := BoxMesh.new()
    box.size = Vector3(0.05, 0.3, 0.05)
    nacelle.mesh = box
    nacelle.name = "nacelle"
    nacelle.position = Vector3(0, 0.15, 0)
    hinge.add_child(nacelle)
    nacelle.owner = root
    var disc := MeshInstance3D.new()
    var cyl := CylinderMesh.new()
    cyl.top_radius = 0.15
    cyl.bottom_radius = 0.15
    cyl.height = 0.01
    disc.mesh = cyl
    disc.name = "disc"
    motor.add_child(disc)
    disc.owner = root
    await wait(1.0)
    await point(tree_item_pos(motor))
    select(motor)
    await wait(1.5)
    await point(await property_pos("servo"))
    await wait(3.0)

    await say("Step 9 - Pivoting segments",
            "Everything under a hinge pivots about the hinge's X axis, driven by its servo (here SERVO6, "
            + "0 to 90 degrees). Drag Preview Angle to see it move; the nose wheel steering works the same way.")
    await point(tree_item_pos(hinge))
    select(hinge)
    await wait(1.5)
    await point(await property_pos("angle_max"))
    await wait(1.5)
    await point(await property_pos("preview_angle"))
    for a in range(0, 91, 5):
        hinge.set("preview_angle", float(a))
        await wait(0.08)
    await wait(1.0)
    for a in range(90, -1, -5):
        hinge.set("preview_angle", float(a))
        await wait(0.08)
    await wait(1.0)
    for a in range(0, 91, 5):
        hinge.set("preview_angle", float(a))
        await wait(0.08)
    await wait(2.0)
    hinge.set("preview_angle", 0.0)
    await wait(1.0)

    await say("Step 10 - Check the design",
            "Project > Tools > ArduPilot: vehicle report lists every part and estimates thrust/weight, "
            + "wing loading, stall speed and static margin from the same models the simulation uses.")
    await show_tools_menu("ArduPilot: vehicle report")
    var dlg := AcceptDialog.new()
    dlg.title = "Vehicle report"
    dlg.dialog_text = Report.report(body)
    EditorInterface.get_base_control().add_child(dlg)
    dlg.popup_centered()
    await wait(8.0)
    dlg.hide()
    dlg.queue_free()

    await say("Step 11 - Save and fly",
            "Save with Ctrl+S, then fly it: godot --path . -- --vehicle=res://models/plane.tscn with SITL, "
            + "or export it with Project > Tools > ArduPilot: export vehicle as glTF... to share it.")
    EditorInterface.save_scene()
    await show_tools_menu("ArduPilot: export vehicle as glTF...")
    await wait(5.0)

    await say("Done",
            "Models are glTF files: KHR_physics_rigid_bodies for mass and colliders, ARDUPILOT_sitl for parts. "
            + "See README.md in the project for every parameter.")
    await wait(5.0)
