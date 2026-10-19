# Godot and Jolt physics backend for SITL

A physics backend for the [SITL JSON interface](../readme.md) built on
[Godot](https://godotengine.org) 4.4 or later and its bundled
[Jolt](https://github.com/jrouwe/JoltPhysics) rigid body engine. Vehicles
are glTF files: mass, inertia and collision shapes use the Khronos
`KHR_physics_rigid_bodies` and `KHR_implicit_shapes` extensions, and the
parts ArduPilot drives use an `ARDUPILOT_sitl` extension. The Godot editor
is the vehicle editor.

It flies a generic quadcopter and a generic aeroplane with ArduCopter and
ArduPlane, using:

- full rigid body dynamics and contacts from Jolt
- per-rotor thrust with motor lag, inflow (thrust falls with airspeed
  through the disc), rotor drag and reaction torque
- propwash: surfaces behind a propeller feel its slipstream, so the tail
  keeps authority at low airspeed
- strip-theory lifting surfaces using the local airflow at each surface,
  so rotational damping, dihedral effect and weathercock stability arise
  from the geometry rather than from stability derivatives
- servo-driven pivots (control surfaces, tilt rotors, steering, gimbals)
- ray cast wheels with spring-damper suspension for runway operations

Tested with Godot 4.7.

## Running

Install Godot 4.4 or later. Jolt is built in and selected by
`project.godot`; no other add-ons are needed. Build SITL as usual, for
example with `./waf configure --board sitl && ./waf copter plane`.

`run_sim.sh` starts the backend and `sim_vehicle.py` together and stops
both on Ctrl-C:

```sh
libraries/SITL/examples/JSON/godot/run_sim.sh copter --map --console
libraries/SITL/examples/JSON/godot/run_sim.sh plane --map --console
```

It finds Godot on the PATH as `godot`, or from `$GODOT`; run it without
arguments for its other modes. To start the two by hand instead, start the
backend, which waits for SITL on UDP port 9002:

```sh
godot --path libraries/SITL/examples/JSON/godot -- --vehicle=res://models/quadx.gltf
```

then SITL:

```sh
sim_vehicle.py -v ArduCopter -f quad --model JSON:127.0.0.1 \
    --add-param-file=Tools/autotest/default_params/copter.parm \
    --add-param-file=libraries/SITL/examples/JSON/godot/models/quadx.parm --map --console
```

For the aeroplane, use `--vehicle=res://models/plane.gltf` and:

```sh
sim_vehicle.py -v ArduPlane -f plane --model JSON:127.0.0.1 \
    --add-param-file=Tools/autotest/models/plane.parm \
    --add-param-file=libraries/SITL/examples/JSON/godot/models/plane.parm --map --console
```

SITL looks up a frame's standard parameters by the model name, which for
`JSON` matches no frame, so they are given explicitly before the model's
own.

The plane faces north on the ground at home and takes off from there.

Each servo packet from SITL advances the physics by exactly one step, so
the backend runs in lockstep with ArduPilot and physics pauses when SITL
stops. Godot normally paces its physics against the wall clock; to go
faster than real time with SITL's `--speedup`, run Godot with a fixed
frame rate, and headless if no view is needed:

```sh
godot --headless --fixed-fps 50 --path libraries/SITL/examples/JSON/godot -- --vehicle=res://models/plane.gltf
```

Options, after `--`:

| Option | Default | Meaning |
| --- | --- | --- |
| `--vehicle=` | `res://models/quadx.gltf` | vehicle: `.gltf`, `.glb` or a Godot scene |
| `--world=` | flat ground | scenery: a `.gltf`/`.glb` whose colliders have no `motion`, or a scene |
| `--port=` | `9002` | UDP port to listen on |
| `--wind=` | `0,0,0` | wind velocity, north, east, down (m/s) |
| `--spawn-height=` | `0` | start this far above the position in the file (m) |
| `--spawn-speed=` | `0` | start moving forward at this speed (m/s), for air starts |
| `--reverse=` | none | servo outputs to reverse about 1500 µs, such as `2,4` |

`test_flight.py` checks a vehicle end to end through MAVLink. The copter
test arms, takes off, holds altitude, flies 20 m and lands; the plane test
flies an AUTO mission with a runway takeoff, a circuit and an automatic
landing.

```sh
./test_flight.py --vehicle plane --connect tcp:127.0.0.1:5762
```

SITL's first port, 5760, is taken by MAVProxy when it is running; without
MAVProxy, connect to 5760 instead.

### With sim_vehicle.py and the autotest suite

`sim_vehicle.py` and `autotest.py` take a `--godot` option which starts
the backend, with its window, and uses it in place of SITL's built-in
physics for frames that have a Godot model:

| Vehicle | Frame | Model |
| --- | --- | --- |
| ArduCopter | `+`, `quad`, `X` | `quadx` |
| ArduPlane | `plane`, `plane-elevrev` | `plane` |
| ArduPlane | `plane-3d` | `plane3d`, an aerobatic plane |

```sh
Tools/autotest/sim_vehicle.py -v ArduCopter --godot --map --console
Tools/autotest/sim_vehicle.py -v ArduPlane -f plane-3d --godot --map --console
Tools/autotest/autotest.py --godot build.Copter test.Copter.TakeoffAlt
Tools/autotest/autotest.py --godot test.Plane.AerobaticsScripting
```

Set `$GODOT` if Godot is not on the PATH as `godot`. `--godot-headless`
runs the autotest backend without its window. A test which asks for a
frame with no Godot model falls back to SITL's own physics and says so.
`plane-elevrev`, the default for the ArduPlane tests, reverses the
elevator and rudder outputs in its parameters; the backend's
`--reverse=2,4` option reverses them back, as SITL's model does. The backend's log is
`godot-<vehicle>.log` in the autotest's build logs directory, `buildlogs`
beside the ArduPilot tree unless `$BUILDLOGS` says otherwise.

The backend runs about twice as fast as real time, well below the
autotest's usual speedup, so a full suite takes much longer than with the
built-in physics. Tests tuned against SITL's own models can fail on a
different airframe: `test.Plane.AerobaticsScripting`, for one, flies the
whole schedule but its largest path error is about 18 m against a limit
of 15 m, where SITL's `plane-3d` manages about 10 m.

### In VS Code

Open `godot-sitl.code-workspace` (File > Open Workspace from File). It
opens the ArduPilot tree and this project, and recommends the Godot Tools,
C/C++ and Python extensions. If Godot is not on the PATH as `godot`, set
its path in the workspace setting `godotTools.editorPath.godot4`, which
both the tasks and the GDScript debugger use. Then, from Terminal > Run
Task:

- **Godot SITL: build ArduCopter** or **build ArduPlane**, once
- **Godot SITL: fly quadcopter** or **fly aeroplane**: the backend's
  window, SITL, and MAVProxy with its map and console
- **Godot SITL: flight test**: the smoke test, against a running sim
- **Godot SITL: open vehicle editor**

In Run and Debug, **ArduCopter in gdb** and **ArduPlane in gdb** start the
backend and debug SITL itself; connect a ground station with the
**MAVProxy** task. **backend in GDScript debugger** runs the backend under
Godot Tools' debugger, with SITL started by a **SITL only** task. Because
the two run in lockstep, a breakpoint on either side pauses the whole
simulation.

## Frames

glTF axes are used throughout: +Y up, +Z forward and +X to the left. A
vehicle's origin is its IMU and should be its centre of mass. In the world,
+Z is north and -X is east, so a vehicle with no rotation faces north.

## Vehicle files

### Rigid bodies

From `KHR_physics_rigid_bodies` and `KHR_implicit_shapes` the backend
supports:

- `motion`: `mass`, `inertiaDiagonal`, `centerOfMass`, `isKinematic`,
  `gravityFactor`, initial velocities. Zero mass or inertia means infinite.
- `collider` with implicit `box`, `sphere`, `capsule`, `cylinder`
  (including cones) and `plane` shapes, or a `mesh` (convex hull on moving
  bodies)
- `physicsMaterials`: friction and restitution

Joints, triggers, collision filters and a non-identity `inertiaOrientation`
are not supported yet; the loader warns when it meets them. Export writes
colliders with implicit shapes only, not convex hulls or meshes. The body with
`motion` that carries the parts is the vehicle; the document-level
`ARDUPILOT_sitl.vehicle` node index can name it explicitly.

### Parts: the ARDUPILOT_sitl extension

A node becomes a part by having one of these members in its
`ARDUPILOT_sitl` node extension. Parts act on the nearest rigid body above
them. Servo numbers are 1-based, as in `SERVOn_FUNCTION`.

`rotor`: a propeller and motor. Thrust acts along the node's +Y.

| Field | Default | Meaning |
| --- | --- | --- |
| `servo` | 1 | servo output |
| `pwmMin`, `pwmMax` | 1000, 2000 | PWM for zero and full throttle |
| `maxThrust` | 10 | static thrust at full throttle (N) |
| `thrustExpo` | 0.65 | 0 = thrust linear in throttle, 1 = quadratic, as `MOT_THST_EXPO` |
| `torqueRatio` | 0.016 | reaction torque per unit thrust (N.m/N) |
| `direction` | `ccw` | spin seen from the tip of the thrust axis, `ccw` or `cw` |
| `timeConstant` | 0.03 | motor lag (s) |
| `diameter` | 0.25 | propeller diameter (m) |
| `pitchSpeed` | 0 | inflow at which full-throttle thrust falls to zero (m/s); 0 disables |
| `inplaneDrag` | 0 | rotor drag in edgewise flow, force per unit thrust per m/s |

`surface`: a lifting surface. Span along +X, chord along Z with the leading
edge forward (+Z), lift towards +Y; the origin is the quarter chord.

| Field | Default | Meaning |
| --- | --- | --- |
| `span`, `chord` | 1, 0.2 | size (m) |
| `aspectRatio` | span/chord | aspect ratio of the whole wing a panel belongs to |
| `cl0` | 0 | lift coefficient at zero angle of attack |
| `cd0` | 0.015 | zero-lift drag coefficient |
| `cm0` | 0 | pitching moment coefficient |
| `oswald` | 0.85 | span efficiency for induced drag |
| `stallAngle` | 15 | angle of attack at the stall (deg) |
| `downwash` | 0 | dε/dα from the wing ahead; about 0.45 for a tailplane |
| `flapServo` | 0 | servo driving a trailing-edge flap, 0 for none |
| `flapPwmMin`, `flapPwmMax` | 1000, 2000 | PWM range of the flap servo |
| `flapAngleMin`, `flapAngleMax` | -20, 20 | flap angle at those PWMs (deg, positive trailing edge down) |
| `flapChordFraction` | 0.25 | flap chord as a fraction of the chord |

`hinge`: a servo-driven pivot. The node and everything below it rotate
about its local X axis by the right-hand rule. Colliders do not follow it.

| Field | Default | Meaning |
| --- | --- | --- |
| `servo` | 1 | servo output |
| `pwmMin`, `pwmMax` | 1000, 2000 | PWM range |
| `angleMin`, `angleMax` | -30, 30 | angle at those PWMs (deg) |
| `maxRate` | 300 | slew rate (deg/s), 0 for instant |

`wheel`: a wheel on a suspension. The origin is the wheel centre with the
suspension fully extended; it compresses along +Y and rolls along +Z. Put
it under a hinge with a vertical axis to steer it.

| Field | Default | Meaning |
| --- | --- | --- |
| `radius` | 0.05 | wheel radius (m) |
| `travel` | 0.05 | suspension travel (m) |
| `spring`, `damping` | 500, 20 | suspension rate (N/m) and damping (N.s/m) |
| `rollingResistance` | 0.02 | rolling resistance coefficient |
| `grip` | 0.9 | sideways friction coefficient |
| `driveServo` | 0 | servo driving the wheel, 0 for none; mid PWM is stopped |
| `drivePwmMin`, `drivePwmMax` | 1000, 2000 | PWM for full reverse and full forward |
| `maxDriveForce` | 0 | driving force at full throttle (N) |

`drag`, on the vehicle's body: `area` is the quadratic drag area (Cd × A,
m²) along the body's X, Y and Z axes, and `airDensity` defaults to 1.225.

A rotor, for example:

```json
{
  "name": "motor1",
  "translation": [-0.12, 0.025, 0.12],
  "extensions": {
    "ARDUPILOT_sitl": {
      "rotor": {"servo": 1, "maxThrust": 9.8, "direction": "ccw", "diameter": 0.254}
    }
  }
}
```

## Editing vehicles

Open this directory as a project in the Godot editor. The ArduPilot SITL
add-on, enabled in `project.godot`, makes it a vehicle editor:

- `.gltf` and `.glb` files in the project are imported with their bodies,
  colliders and parts
- `ArduPilotVehicle`, `ArduPilotRotor`, `ArduPilotSurface`,
  `ArduPilotHinge` and `ArduPilotWheel` are in the Add Node dialog, with
  every parameter in the inspector
- gizmos draw rotor discs with thrust and spin direction, surface outlines
  with their flap hinge line and travel, hinge axes with their travel,
  wheels with their suspension, and the vehicle's forward axis and centre
  of mass
- a hinge's `preview_angle` shows its deflection in the editor without
  changing the model
- Project > Tools has three commands:
  - **ArduPilot: vehicle report** lists the parts and estimates thrust to
    weight, wing loading, stall speed and static margin from the same
    models the simulation uses
  - **ArduPilot: convert glTF vehicle to scene** turns an imported glTF,
    which can only be inherited, into a scene you can change freely
  - **ArduPilot: export vehicle as glTF** writes the open scene back to a
    self-describing glTF file

A typical workflow is to model and texture the airframe in Blender,
optionally adding colliders and mass with the
[glTF physics add-on](https://github.com/eoineoineoin/glTF_Physics_Blender_Exporter),
export glTF, convert it to a scene here, add and place the parts, check the
report, run it, and export glTF to share. A scene can be flown directly
with `--vehicle=res://path/to/vehicle.tscn`.

### Step by step

1. Open the project: `godot --editor --path libraries/SITL/examples/JSON/godot`,
   or Import it in the Project Manager. The vehicles are in `models`.
2. Project > Tools > **ArduPilot: convert glTF vehicle to scene...** and pick,
   say, `models/plane.gltf`. It saves `models/plane.tscn` beside it and opens
   it.
3. The Scene tree shows the vehicle body, an `ArduPilotVehicle` with its mass,
   inertia and drag, and under it the parts and colliders. Select a node to
   see its settings in the Inspector.
4. Change a part by typing in the Inspector. For the propeller, Max Thrust
   and Pitch Speed set its static thrust and how it falls off with speed; the
   thrust arrow in the 3D view follows.
5. Move a part with the gizmo arrows in the 3D view or with Transform >
   Position, in metres from the vehicle origin: +Z forward, +Y up, +X left.
6. A wing panel is an `ArduPilotSurface`. Span and chord size it; set Flap
   Servo and the flap angles and chord fraction, in the Flap group, to make
   it a control surface.
7. Parts under an `ArduPilotHinge` pivot about the hinge's X axis, driven by
   its servo. Drag Preview Angle to see the movement; the simulation and
   the exported glTF start from the hinge at rest.
8. To add a part, select its parent, click + (Add Child Node), search for
   ArduPilot and pick a rotor, surface, hinge or wheel. Give it a servo and
   set that output's `SERVOn_FUNCTION` in ArduPilot.
9. Project > Tools > **ArduPilot: vehicle report** checks the design: thrust
   to weight, wing loading, stall speed and static margin.
10. Save with Ctrl+S and fly the scene with
    `--vehicle=res://models/plane.tscn`, or use **ArduPilot: export vehicle
    as glTF...** to write a glTF file. To fly it with `--godot`, export it
    over the model in `models` named in `Tools/autotest/pysim/godot.py`.

Godot's importer treats some node name suffixes as import hints, such as
`-col`, `-rigid`, `-vehicle` and `-wheel` (also with `_`), and renames
those nodes; avoid them in vehicle files.

## Physics models

Rotor: throttle t from the PWM gives a static thrust
`maxThrust × ((1 - expo) t + expo t²)` through a first-order lag. With a
pitch speed, thrust is scaled by `1 - inflow / (pitchSpeed × √(T/maxThrust))`,
the propeller's pitch speed scaling with rpm. Rotor drag opposes the
edgewise flow in proportion to thrust. The reaction torque is
`torqueRatio × thrust` about the thrust axis.

Surface: the air velocity at the aerodynamic centre, including the body's
rotation and the wind, is resolved into the surface's chord plane. The flap
adds `τ δ` to the angle of attack, where τ is the flap effectiveness from
thin aerofoil theory. Lift uses the finite wing lift slope
`2π AR / (2 + √(AR² + 4))`, blended into flat plate lift past the stall
with the sigmoid of Beard and McLain's *Small Unmanned Aircraft*. Drag is
`cd0` plus induced drag before the stall and flat plate drag after it,
plus a flap drag increment. Tailplanes subtract `downwash × α` of the
vehicle from their angle of attack.

Wheel: a ray from the top of the suspension travel finds the ground. The
spring and damper force acts along the suspension; side friction and
rolling resistance act in the ground plane, smoothed near zero speed.

Propwash: momentum theory gives the induced velocity at the disc from the
thrust and the inflow, and the fully developed slipstream is twice that.
It is applied along the thrust axis to surfaces behind the rotor, over the
part of their span inside a cylinder of half the disc's area, the fully
developed slipstream, giving the span's mean dynamic pressure.

Not modelled yet: ground effect, propeller gyroscopic and P-factor
effects, motor and battery electrics, and moving mass in hinges.

## Ideas for further work

- joints from `KHR_physics_rigid_bodies` as Jolt constraints, for slung
  loads, articulated landing gear and tilting parts with mass
- rangefinder, optical flow and camera sensors from ray casts and rendering
- motor and battery electrical models feeding the JSON `battery` fields
- glTF 2.1 external asset references, so a world file can place vehicles
  and scenery from separate files
- the same glTF vehicles in a native SITL backend linking Jolt directly,
  for use without Godot
