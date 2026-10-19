@tool
extends EditorPlugin
## Makes the Godot editor a vehicle model editor for ArduPilot SITL:
## - .gltf/.glb files using KHR_physics_rigid_bodies and ARDUPILOT_sitl are
##   imported with their bodies, colliders and rotors
## - ArduPilotVehicle and ArduPilotRotor appear in the Add Node dialog
## - Project > Tools > "Export ArduPilot vehicle glTF..." writes the open
##   scene back out as a self-describing glTF file

const GLTFPhysics := preload("res://scripts/gltf_physics.gd")
const MENU_EXPORT := "Export ArduPilot vehicle glTF..."

var _ext: GLTFPhysics
var _dialog: EditorFileDialog


func _enter_tree() -> void:
    _ext = GLTFPhysics.new()
    if not GLTFPhysics._register(_ext):
        push_warning("ArduPilot glTF physics extension already registered")
    add_tool_menu_item(MENU_EXPORT, _on_export)

    _dialog = EditorFileDialog.new()
    _dialog.file_mode = EditorFileDialog.FILE_MODE_SAVE_FILE
    _dialog.access = EditorFileDialog.ACCESS_FILESYSTEM
    _dialog.filters = PackedStringArray(["*.gltf ; glTF JSON", "*.glb ; glTF binary"])
    _dialog.file_selected.connect(_on_file_selected)
    EditorInterface.get_base_control().add_child(_dialog)


func _exit_tree() -> void:
    remove_tool_menu_item(MENU_EXPORT)
    if GLTFPhysics.registered == _ext:
        GLTFPhysics._unregister(_ext)
    _dialog.queue_free()


func _on_export() -> void:
    if EditorInterface.get_edited_scene_root() == null:
        push_error("open the vehicle scene to export first")
        return
    _dialog.popup_file_dialog()


func _on_file_selected(path: String) -> void:
    var err := _ext.save_gltf(EditorInterface.get_edited_scene_root(), path)
    if err != OK:
        push_error("export to %s failed: %s" % [path, error_string(err)])
    else:
        print("exported ", path)
