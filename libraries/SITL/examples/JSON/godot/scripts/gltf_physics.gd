@tool
class_name ArduPilotGLTFPhysics
extends GLTFDocumentExtension
## glTF import/export of rigid body physics and ArduPilot actuators.
##
## Import: KHR_physics_rigid_bodies + KHR_implicit_shapes become Godot
## physics bodies and collision shapes, and the ARDUPILOT_sitl node extension
## becomes ArduPilotRotor nodes and ArduPilotVehicle drag settings.
## Export: the reverse, so a model edited in the Godot editor can be written
## back to a self-describing glTF file.
##
## The same instance is used at runtime (see load_gltf()) and by the editor
## import pipeline (registered by the addon in addons/ardupilot_sitl).
##
## Not yet supported: joints, triggers, collision filters, non-identity
## inertiaOrientation and animated colliders. These are reported, not
## silently ignored.

const KHR_RB := "KHR_physics_rigid_bodies"
const KHR_SHAPES := "KHR_implicit_shapes"
const AP_EXT := "ARDUPILOT_sitl"
const Vehicle := preload("res://scripts/ap_vehicle.gd")
const Rotor := preload("res://scripts/ap_rotor.gd")
const OMI_EXTS := ["OMI_physics_body", "OMI_physics_shape", "OMI_collider"]

## the instance registered with GLTFDocument, if any. The editor addon
## registers one for the lifetime of the editor; otherwise one is registered
## just for the duration of each load or save.
static var registered: GLTFDocumentExtension = null

## create visual meshes from collision shapes when the file has no meshes
var visualise_colliders := true

# export state, reset by _export_preflight
var _export_shapes: Array = []
var _export_materials: Array = []
var _export_vehicle: GLTFNode = null


static func _register(ext: GLTFDocumentExtension) -> bool:
    if registered != null:
        return false
    GLTFDocument.register_gltf_document_extension(ext)
    registered = ext
    return true


static func _unregister(ext: GLTFDocumentExtension) -> void:
    GLTFDocument.unregister_gltf_document_extension(ext)
    registered = null


## load a .gltf/.glb, returning the generated scene with physics built
func load_gltf(path: String) -> Node3D:
    var temporary := _register(self)
    var doc := GLTFDocument.new()
    var state := GLTFState.new()
    var err := doc.append_from_file(path, state)
    var root: Node3D = null
    if err != OK:
        push_error("failed to load %s: %s" % [path, error_string(err)])
    else:
        root = doc.generate_scene(state) as Node3D
        if root == null:
            push_error("no scene generated from %s" % path)
    if temporary:
        _unregister(self)
    return root


## write a scene containing physics bodies and actuators to a .gltf/.glb
func save_gltf(root: Node, path: String) -> Error:
    var temporary := _register(self)
    var doc := GLTFDocument.new()
    var state := GLTFState.new()
    var err := doc.append_from_scene(root, state)
    if err == OK:
        err = doc.write_to_filesystem(state, path)
    if temporary:
        _unregister(self)
    return err


# ---------------------------------------------------------------- import

func _get_supported_extensions() -> PackedStringArray:
    return PackedStringArray([KHR_RB, KHR_SHAPES, AP_EXT])


func _import_preflight(_state: GLTFState, extensions: PackedStringArray) -> Error:
    if KHR_RB in extensions or AP_EXT in extensions:
        return OK
    return ERR_SKIP


func _import_post(state: GLTFState, root: Node) -> Error:
    build_physics(state, root)
    return OK


## convert the plain scene generated from state into physics bodies
func build_physics(state: GLTFState, root: Node) -> void:
    var json: Dictionary = state.json
    var gltf_nodes: Array = json.get("nodes", [])
    var doc_ext: Dictionary = json.get("extensions", {})
    var shapes: Array = doc_ext.get(KHR_SHAPES, {}).get("shapes", [])
    var khr_doc: Dictionary = doc_ext.get(KHR_RB, {})
    var materials: Array = khr_doc.get("physicsMaterials", [])
    var ap_doc: Dictionary = doc_ext.get(AP_EXT, {})
    var visualise: bool = visualise_colliders and json.get("meshes", []).is_empty()

    if not khr_doc.get("physicsJoints", []).is_empty():
        push_warning("%s: joints are not supported yet and are ignored" % KHR_RB)

    var parent_of := {}
    for i in gltf_nodes.size():
        for c in gltf_nodes[i].get("children", []):
            parent_of[int(c)] = i

    var khr := {}
    var ap := {}
    for i in gltf_nodes.size():
        var ext: Dictionary = gltf_nodes[i].get("extensions", {})
        if ext.has(KHR_RB):
            khr[i] = ext[KHR_RB]
        if ext.has(AP_EXT):
            ap[i] = ext[AP_EXT]

    var vehicle_idx := _choose_vehicle(ap_doc, khr, ap, parent_of)
    # glTF node index -> node that replaced the generated one
    var replaced := {}

    # bodies first, parents before children, so colliders can find them
    var motion_nodes: Array = khr.keys().filter(func(i): return khr[i].has("motion"))
    motion_nodes.sort_custom(func(a, b): return _depth(a, parent_of) < _depth(b, parent_of))
    for i in motion_nodes:
        var n := state.get_scene_node(i) as Node3D
        if n == null:
            continue
        var body: RigidBody3D = Vehicle.new() if i == vehicle_idx else RigidBody3D.new()
        body = _replace_node(n, body, root) as RigidBody3D
        replaced[i] = body
        _apply_motion(body, khr[i]["motion"])
        if ap.has(i) and ap[i].has("drag") and body is Vehicle:
            var d: Dictionary = ap[i]["drag"]
            var area: Array = d.get("area", [0, 0, 0])
            body.drag_area = Vector3(area[0], area[1], area[2])
            body.air_density = float(d.get("airDensity", body.air_density))

    for i in khr.keys():
        var k: Dictionary = khr[i]
        if k.has("trigger"):
            push_warning("%s: trigger on node %d is not supported yet" % [KHR_RB, i])
        if k.has("joint"):
            push_warning("%s: joint on node %d is not supported yet" % [KHR_RB, i])
        if not k.has("collider"):
            continue
        var n := replaced.get(i, state.get_scene_node(i)) as Node3D
        if n == null:
            continue
        _add_collider(state, root, n, k["collider"], shapes, materials, visualise)

    for i in ap.keys():
        if not ap[i].has("rotor"):
            continue
        var n := replaced.get(i, state.get_scene_node(i)) as Node3D
        if n == null:
            continue
        if n is PhysicsBody3D:
            push_warning("%s: rotor on body node %s ignored; put it on a child node" % [AP_EXT, n.name])
            continue
        var rotor := _replace_node(n, Rotor.new(), root) as Rotor
        rotor.from_gltf(ap[i]["rotor"])
        if visualise:
            var disc := MeshInstance3D.new()
            var m := CylinderMesh.new()
            var radius := 0.12 * sqrt(maxf(rotor.max_thrust, 0.1) / 10.0)
            m.top_radius = radius
            m.bottom_radius = radius
            m.height = 0.005
            disc.mesh = m
            disc.name = "disc"
            rotor.add_child(disc)
            disc.owner = root


func _choose_vehicle(ap_doc: Dictionary, khr: Dictionary, ap: Dictionary, parent_of: Dictionary) -> int:
    if ap_doc.has("vehicle"):
        return int(ap_doc["vehicle"])
    # otherwise the body that carries the first rotor
    for i in ap.keys():
        if not ap[i].has("rotor"):
            continue
        var j: int = i
        while true:
            if khr.has(j) and khr[j].has("motion"):
                return j
            if not parent_of.has(j):
                break
            j = parent_of[j]
    for i in khr.keys():
        if khr[i].has("motion"):
            return i
    return -1


func _depth(i: int, parent_of: Dictionary) -> int:
    var d := 0
    while parent_of.has(i):
        i = parent_of[i]
        d += 1
    return d


## transform of n relative to root, valid outside the SceneTree
static func _xform_to_root(n: Node, root: Node) -> Transform3D:
    var t := Transform3D.IDENTITY
    while n != null and n != root:
        if n is Node3D:
            t = n.transform * t
        n = n.get_parent()
    return t


## put replacement where n is. An empty node is replaced outright; a node
## with content (a mesh, say) is kept as a child of the replacement. The
## replacement gets no scale, which Jolt does not support on bodies.
func _replace_node(n: Node3D, replacement: Node3D, root: Node) -> Node3D:
    var parent := n.get_parent()
    if parent == null:
        push_error("cannot replace scene root %s" % n.name)
        return n
    var local := n.transform
    var node_name := n.name
    replacement.transform = Transform3D(local.basis.orthonormalized(), local.origin)
    var residual := replacement.transform.affine_inverse() * local
    var is_empty := n.get_class() == "Node3D" and n.get_script() == null

    var idx := n.get_index()
    parent.remove_child(n)
    parent.add_child(replacement)
    parent.move_child(replacement, idx)
    replacement.name = node_name
    replacement.owner = root

    if is_empty:
        for c in n.get_children():
            _set_owner_recursive(c, null)
            n.remove_child(c)
            replacement.add_child(c)
            if c is Node3D:
                c.transform = residual * c.transform
            _set_owner_recursive(c, root)
        n.queue_free()
    else:
        _set_owner_recursive(n, null)
        n.name = String(n.name) + "_visual"
        replacement.add_child(n)
        n.transform = residual
        _set_owner_recursive(n, root)
    return replacement


static func _set_owner_recursive(n: Node, owner: Node) -> void:
    n.owner = owner
    for c in n.get_children():
        _set_owner_recursive(c, owner)


static func _v3(a, default: Vector3) -> Vector3:
    if a is Array and a.size() == 3:
        return Vector3(a[0], a[1], a[2])
    return default


func _apply_motion(body: RigidBody3D, m: Dictionary) -> void:
    body.linear_damp_mode = RigidBody3D.DAMP_MODE_REPLACE
    body.angular_damp_mode = RigidBody3D.DAMP_MODE_REPLACE
    body.linear_damp = 0.0
    body.angular_damp = 0.0

    var mass := float(m.get("mass", -1.0))
    if m.get("isKinematic", false) or mass == 0.0:
        # glTF uses zero mass to mean infinite
        body.freeze = true
        body.freeze_mode = RigidBody3D.FREEZE_MODE_KINEMATIC
    elif mass > 0.0:
        body.mass = mass
    else:
        push_warning("%s: no mass given for %s, using %.1f kg" % [KHR_RB, body.name, body.mass])

    if m.has("inertiaDiagonal"):
        var inertia := _v3(m["inertiaDiagonal"], Vector3.ZERO)
        # zero means infinite: lock rotation about that axis
        body.axis_lock_angular_x = inertia.x == 0.0
        body.axis_lock_angular_y = inertia.y == 0.0
        body.axis_lock_angular_z = inertia.z == 0.0
        body.inertia = Vector3(
                inertia.x if inertia.x > 0.0 else 1.0,
                inertia.y if inertia.y > 0.0 else 1.0,
                inertia.z if inertia.z > 0.0 else 1.0)
    if m.has("inertiaOrientation"):
        var q: Array = m["inertiaOrientation"]
        if not Quaternion(q[0], q[1], q[2], q[3]).is_equal_approx(Quaternion.IDENTITY):
            push_warning("%s: inertiaOrientation is not supported, using principal axes = body axes" % KHR_RB)
    if m.has("centerOfMass"):
        body.center_of_mass_mode = RigidBody3D.CENTER_OF_MASS_MODE_CUSTOM
        body.center_of_mass = _v3(m["centerOfMass"], Vector3.ZERO)
    body.gravity_scale = float(m.get("gravityFactor", 1.0))
    var b := body.transform.basis
    body.linear_velocity = b * _v3(m.get("linearVelocity"), Vector3.ZERO)
    body.angular_velocity = b * _v3(m.get("angularVelocity"), Vector3.ZERO)


func _find_body(n: Node, root: Node) -> PhysicsBody3D:
    while n != null and n != root:
        if n is PhysicsBody3D:
            return n
        n = n.get_parent()
    return null


func _add_collider(state: GLTFState, root: Node, n: Node3D, collider: Dictionary,
        shapes: Array, materials: Array, visualise: bool) -> void:
    var body := _find_body(n, root)
    if body == null:
        # a collider without a moving ancestor is static scenery
        var sb := StaticBody3D.new()
        sb.name = String(n.name) + "_static"
        root.add_child(sb)
        sb.owner = root
        var t := _xform_to_root(n, root)
        sb.transform = Transform3D(t.basis.orthonormalized(), t.origin)
        body = sb

    var rel := _xform_to_root(body, root).affine_inverse() * _xform_to_root(n, root)
    var scale := rel.basis.get_scale().abs()
    var geometry: Dictionary = collider.get("geometry", {})
    var shape: Shape3D = null
    var shape_mesh: Mesh = null
    if geometry.has("shape"):
        var s: Dictionary = shapes[int(geometry["shape"])]
        var r := _implicit_shape(s, scale)
        shape = r[0]
        shape_mesh = r[1]
    elif geometry.has("mesh"):
        shape = _mesh_shape(state, int(geometry["mesh"]), scale,
                geometry.get("convexHull", false) or body is RigidBody3D)
    if shape == null:
        push_warning("%s: unsupported collider geometry on %s" % [KHR_RB, n.name])
        return

    var cs := CollisionShape3D.new()
    cs.name = String(n.name) + "_collider"
    cs.shape = shape
    cs.set_meta("gltf_shape", geometry)
    body.add_child(cs)
    cs.owner = root
    cs.transform = Transform3D(rel.basis.orthonormalized(), rel.origin)

    if collider.has("physicsMaterial") and body.physics_material_override == null:
        var pm: Dictionary = materials[int(collider["physicsMaterial"])]
        var mat := PhysicsMaterial.new()
        mat.friction = float(pm.get("dynamicFriction", pm.get("staticFriction", 0.5)))
        mat.bounce = float(pm.get("restitution", 0.0))
        body.physics_material_override = mat

    if visualise and shape_mesh != null:
        var mi := MeshInstance3D.new()
        mi.name = String(n.name) + "_mesh"
        mi.mesh = shape_mesh
        cs.add_child(mi)
        mi.owner = root


## returns [Shape3D, Mesh] for a KHR_implicit_shapes shape scaled by scale
func _implicit_shape(s: Dictionary, scale: Vector3) -> Array:
    var radial := maxf(scale.x, scale.z)
    match s.get("type", ""):
        "box":
            var size := _v3(s.get("box", {}).get("size"), Vector3.ONE) * scale
            var shape := BoxShape3D.new()
            shape.size = size
            var mesh := BoxMesh.new()
            mesh.size = size
            return [shape, mesh]
        "sphere":
            var r := float(s.get("sphere", {}).get("radius", 0.5)) * maxf(radial, scale.y)
            var shape := SphereShape3D.new()
            shape.radius = r
            var mesh := SphereMesh.new()
            mesh.radius = r
            mesh.height = 2.0 * r
            return [shape, mesh]
        "capsule", "cylinder":
            var type: String = s["type"]
            var p: Dictionary = s.get(type, {})
            var h := float(p.get("height", 0.5)) * scale.y
            var rt := float(p.get("radiusTop", 0.25)) * radial
            var rb := float(p.get("radiusBottom", 0.25)) * radial
            if type == "capsule":
                var r := maxf(rt, rb)
                if not is_equal_approx(rt, rb):
                    push_warning("%s: tapered capsule approximated with radius %.3f" % [KHR_SHAPES, r])
                var shape := CapsuleShape3D.new()
                shape.radius = r
                shape.height = h + 2.0 * r
                var capsule := CapsuleMesh.new()
                capsule.radius = r
                capsule.height = h + 2.0 * r
                return [shape, capsule]
            var mesh := CylinderMesh.new()
            mesh.top_radius = rt
            mesh.bottom_radius = rb
            mesh.height = h
            if is_equal_approx(rt, rb):
                var shape := CylinderShape3D.new()
                shape.radius = rt
                shape.height = h
                return [shape, mesh]
            # cone or frustum
            var hull := ConvexPolygonShape3D.new()
            hull.points = mesh.get_mesh_arrays()[Mesh.ARRAY_VERTEX]
            return [hull, mesh]
        "plane":
            var p: Dictionary = s.get("plane", {})
            if p.has("sizeX") and p.has("sizeZ"):
                var size := Vector3(float(p["sizeX"]) * scale.x, 0.001, float(p["sizeZ"]) * scale.z)
                var shape := BoxShape3D.new()
                shape.size = size
                var mesh := PlaneMesh.new()
                mesh.size = Vector2(size.x, size.z)
                return [shape, mesh]
            var shape := WorldBoundaryShape3D.new()
            var mesh := PlaneMesh.new()
            mesh.size = Vector2(200, 200)
            return [shape, mesh]
    push_warning("%s: unknown shape type '%s'" % [KHR_SHAPES, s.get("type", "")])
    return [null, null]


func _mesh_shape(state: GLTFState, mesh_idx: int, scale: Vector3, convex: bool) -> Shape3D:
    var meshes := state.get_meshes()
    if mesh_idx < 0 or mesh_idx >= meshes.size():
        return null
    var mesh := meshes[mesh_idx].mesh.get_mesh()
    if mesh == null:
        return null
    if convex:
        var hull := mesh.create_convex_shape(true, false) as ConvexPolygonShape3D
        var pts := hull.points
        for i in pts.size():
            pts[i] *= scale
        hull.points = pts
        return hull
    var tri := mesh.create_trimesh_shape() as ConcavePolygonShape3D
    var faces := tri.get_faces()
    for i in faces.size():
        faces[i] *= scale
    tri.set_faces(faces)
    return tri


# ---------------------------------------------------------------- export

func _export_preflight(_state: GLTFState, root: Node) -> Error:
    _export_shapes.clear()
    _export_materials.clear()
    _export_vehicle = null
    return OK


func _export_node(state: GLTFState, gltf_node: GLTFNode, json: Dictionary, node: Node) -> Error:
    var ext: Dictionary = json.get("extensions", {})
    # Godot's own physics exporter writes the older OMI extensions; this
    # file uses KHR_physics_rigid_bodies instead, so don't describe it twice
    for omi in OMI_EXTS:
        ext.erase(omi)
    var khr := {}
    var ap := {}

    if node is RigidBody3D:
        khr["motion"] = _motion_to_gltf(node)
        if node is Vehicle:
            _export_vehicle = gltf_node
            if node.drag_area != Vector3.ZERO:
                var a: Vector3 = node.drag_area
                ap["drag"] = {"area": [a.x, a.y, a.z], "airDensity": node.air_density}
    elif node is CollisionShape3D and node.shape != null:
        var collider := _collider_to_gltf(node)
        if not collider.is_empty():
            khr["collider"] = collider
    elif node is Rotor:
        ap["rotor"] = node.to_gltf()

    if not khr.is_empty():
        ext[KHR_RB] = khr
        state.add_used_extension(KHR_RB, false)
    if not ap.is_empty():
        ext[AP_EXT] = ap
        state.add_used_extension(AP_EXT, false)
    if ext.is_empty():
        json.erase("extensions")
    else:
        json["extensions"] = ext
    return OK


func _export_post(state: GLTFState) -> Error:
    var json: Dictionary = state.json
    var ext: Dictionary = json.get("extensions", {})
    if not _export_shapes.is_empty():
        ext[KHR_SHAPES] = {"shapes": _export_shapes}
        if not KHR_SHAPES in json.get("extensionsUsed", []):
            json["extensionsUsed"] = json.get("extensionsUsed", []) + [KHR_SHAPES]
    if not _export_materials.is_empty():
        ext[KHR_RB] = {"physicsMaterials": _export_materials}
    var vehicle_idx := state.get_nodes().find(_export_vehicle) if _export_vehicle != null else -1
    if vehicle_idx >= 0:
        ext[AP_EXT] = {"vehicle": vehicle_idx}
    for omi in OMI_EXTS:
        ext.erase(omi)
    if not ext.is_empty():
        json["extensions"] = ext
    var used: Array = json.get("extensionsUsed", [])
    json["extensionsUsed"] = used.filter(func(e): return not e in OMI_EXTS)
    return OK


func _motion_to_gltf(body: RigidBody3D) -> Dictionary:
    var m := {}
    if body.freeze:
        m["isKinematic"] = true
    m["mass"] = body.mass
    if body.inertia != Vector3.ZERO:
        var i := body.inertia
        m["inertiaDiagonal"] = [
            0.0 if body.axis_lock_angular_x else i.x,
            0.0 if body.axis_lock_angular_y else i.y,
            0.0 if body.axis_lock_angular_z else i.z]
    if body.center_of_mass_mode == RigidBody3D.CENTER_OF_MASS_MODE_CUSTOM:
        var c := body.center_of_mass
        m["centerOfMass"] = [c.x, c.y, c.z]
    if body.gravity_scale != 1.0:
        m["gravityFactor"] = body.gravity_scale
    return m


func _collider_to_gltf(cs: CollisionShape3D) -> Dictionary:
    var s := cs.shape
    var d := {}
    if s is BoxShape3D:
        d = {"type": "box", "box": {"size": [s.size.x, s.size.y, s.size.z]}}
    elif s is SphereShape3D:
        d = {"type": "sphere", "sphere": {"radius": s.radius}}
    elif s is CapsuleShape3D:
        d = {"type": "capsule", "capsule": {"height": s.height - 2.0 * s.radius,
                "radiusTop": s.radius, "radiusBottom": s.radius}}
    elif s is CylinderShape3D:
        d = {"type": "cylinder", "cylinder": {"height": s.height,
                "radiusTop": s.radius, "radiusBottom": s.radius}}
    elif s is WorldBoundaryShape3D:
        d = {"type": "plane", "plane": {}}
    else:
        push_warning("%s: %s on %s not exported" % [KHR_SHAPES, s.get_class(), cs.name])
        return {}
    var collider := {"geometry": {"shape": _export_shapes.size()}}
    _export_shapes.append(d)

    var body := cs.get_parent() as PhysicsBody3D
    var pm: PhysicsMaterial = null
    if body is RigidBody3D or body is StaticBody3D:
        pm = body.physics_material_override
    if pm != null:
        var md := {"staticFriction": pm.friction, "dynamicFriction": pm.friction,
                "restitution": pm.bounce}
        var idx := _export_materials.find(md)
        if idx < 0:
            idx = _export_materials.size()
            _export_materials.append(md)
        collider["physicsMaterial"] = idx
    return collider
