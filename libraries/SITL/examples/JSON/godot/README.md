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
- strip-theory lifting surfaces using the local airflow at each surface,
  so rotational damping, dihedral effect and weathercock stability arise
  from the geometry rather than from stability derivatives
- servo-driven pivots (control surfaces, tilt rotors, steering, gimbals)
- ray cast wheels with spring-damper suspension for runway operations

Tested with Godot 4.7.

## Running

Install Godot 4.4 or later. Jolt is built in and selected by
`project.godot`; no other add-ons are needed.

Start the backend, which waits for SITL on UDP port 9002:

```sh
godot --path libraries/SITL/examples/JSON/godot -- --vehicle=res://models/quadx.gltf
```

then SITL:

```sh
sim_vehicle.py -v ArduCopter -f quad --model JSON:127.0.0.1 \
    --add-param-file=libraries/SITL/examples/JSON/godot/models/quadx.parm --map --console
```

For the aeroplane, use `--vehicle=res://models/plane.gltf` and:

```sh
sim_vehicle.py -v ArduPlane -f plane --model JSON:127.0.0.1 \
    --add-param-file=libraries/SITL/examples/JSON/godot/models/plane.parm --map --console
```

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

`test_flight.py` checks a vehicle end to end through MAVLink. The copter
test arms, takes off, holds altitude, flies 20 m and lands; the plane test
flies an AUTO mission with a runway takeoff, a circuit and an automatic
landing.

```sh
./test_flight.py --vehicle plane --connect tcp:127.0.0.1:5760
```

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

Not modelled yet: propwash over surfaces, ground effect, propeller
gyroscopic and P-factor effects, motor and battery electrics, and moving
mass in hinges.

## Ideas for further work

- joints from `KHR_physics_rigid_bodies` as Jolt constraints, for slung
  loads, articulated landing gear and tilting parts with mass
- rangefinder, optical flow and camera sensors from ray casts and rendering
- motor and battery electrical models feeding the JSON `battery` fields
- propwash, so that a tail behind a propeller keeps authority on the ground
- glTF 2.1 external asset references, so a world file can place vehicles
  and scenery from separate files
- the same glTF vehicles in a native SITL backend linking Jolt directly,
  for use without Godot
