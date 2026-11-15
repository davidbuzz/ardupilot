# SITL 2.0, option B: Godot bakes the vehicle, C++ steps it

Status: **the chosen plan** as of 2026-11-15, after option A was tried and
set aside. First implementation flying (see "Progress"). Experimental and
local; not part of PR #35174. Updated as work proceeds.

The alternatives it was weighed against: option A, Godot embedded in SITL as
a library, and option C, a C++ glTF loader with no Godot at all.

## What option B is

The work is split by when it happens.

- **Once, ahead of time, in Godot:** the existing `.gd` loader reads a glTF
  vehicle and writes out a flattened description of it, the "bake": every
  body, shape, joint and part with its numbers already worked out.
- **Every step, in SITL:** a C++ model reads the bake, builds the bodies in
  Jolt and steps them in-process. No Godot runs during a flight.

Godot is still the vehicle editor and the viewer. The glTF file stays the
source of truth; the bake is a build product of it.

## Why it is possible

The loader's job ends before the first step. Everything it decides - which
node is a body, where a wheel sits, what a hinge turns - can be written down
as plain numbers. `scripts/ap_vehicle.gd` already does this on a small scale
for `native/`: it hands each rotor and surface to C++ as a transform in the
body's frame plus a list of parameters.

## Why bother

Timing of one plane step on 2026-11-15 (about 485 us in total, one Godot
worker thread, `--speedup=100` asked, about 21x achieved):

| Stage | Time |
| --- | --- |
| Waiting for SITL's reply over UDP | 268 us |
| Outside the scripts: Jolt, engine, drawing | 85 us |
| Per-part forces | 50 us |
| Building and sending the reply | 46 us |
| Everything else | about 35 us |

ArduPilot's own work is about 140 us of the 268 us wait. Option B removes
the round trip, Godot's main loop and all per-step script, leaving
ArduPilot's work plus a Jolt step and the part models in C++.

## How it compares

Option A's figure is measured. B and C are bounded by the stock SITL
ceiling below; what they achieve depends on how cheap the C++ step is.

| | A. Godot embedded | B. Bake, then C++ | C. Full C++ |
| --- | --- | --- | --- |
| `.gd` code reused | nearly all | loader only | none |
| New C++ | small: a SITL model that hosts Godot | the per-step logic and a bake reader | everything |
| Plane, 100 Hz | 26 to 33x headless (measured); no gain with a window | up to 100x (stock SITL's ceiling) | up to 100x |
| Dependency at run time | a Godot library build | Jolt | Jolt |
| Godot needed to fly | yes, as a library | no, if bakes are committed | no |
| Risk of physics differing from today | none: same engine, same scripts | real | real |

Against option C, B saves writing a glTF loader in C++: about half of the
1,094 lines of `scripts/gltf_physics.gd` is physics set-up that would
otherwise be re-expressed. It does not save the per-step code: about 1,000
lines of part and vehicle scripts (`scripts/ap_*.gd`), plus the launcher,
water and impact logic in `scripts/main.gd`, still move to C++.

## Progress

### Built and flying (2026-11-15)

| Piece | Where | State |
| --- | --- | --- |
| Part models with no engine types: rotor, surface, hinge, wheel, floater, body drag | `native/sitl_parts.{h,cpp}` | Done. Rotor and surface checked against the scripts (`native/check.gd`); the rest not yet compared |
| Bake writer | `native/bake.gd` | Done: all 21 models bake and are flyable |
| Jolt stepper with a plain C interface | `native/jolt_sim.{h,cpp}` | Done: jointed bodies, hinge motors, lake, launchers |
| One static library: Jolt 5.5.0 (the version Godot 4.7 bundles) plus the above | `native/build_jolt.sh` | Done: 53 seconds at `-j6` |
| SITL model, `--model jolt:vehicle.bake` | `libraries/SITL/SIM_Jolt.{h,cpp}` | Done: servos in and state out by direct call |
| Build hook | `Tools/ardupilotwaf/boards.py` | With `SITL_JOLT_LIB` set at configure, the library is linked into the SITL binary |
| Harness | `Tools/autotest/pysim/godot.py`, `vehicle_test_suite.py` | With `SITL_JOLT_LIB` set, `--godot` starts SITL on the baked model and no backend; servo reversals, wind, lake depth and launcher travel in the model string (`jolt:vehicle.bake,reverse=2+4,wind=3+0+0,lake=60,launcher=sled+15+5+15`) |
| External viewer | `SIM_Jolt` (sender), `scripts/main.gd` `--watch=port` (viewer) | Working: SITL sends each body's pose over UDP fifty times a second of real time and never waits; Godot, as a separate process, freezes the vehicle and places it there. The harness starts it unless headless is asked for |

To build and fly:

```sh
libraries/SITL_Godot/native/build_jolt.sh
godot --headless --path libraries/SITL_Godot -s native/bake.gd
export SITL_JOLT_LIB=$PWD/libraries/SITL_Godot/native/build/libsitl_jolt.a
Tools/autotest/autotest.py build.Copter
Tools/autotest/autotest.py --godot --speedup=100 test.Copter.TakeoffAlt
```

### Measured

The stepper on its own, no SITL around it:

| Vehicle | Cost per step |
| --- | --- |
| Quadcopter (4 rotors) | 3.6 us |
| Plane rolling on its wheels (surfaces, 3 wheels) | 32 us |

In SITL, one binary, no Godot running, `--speedup=100` asked:

| Test | Model, physics rate | Result | Speed | Godot backend, same day |
| --- | --- | --- | --- | --- |
| `Copter.TakeoffAlt` | quadx, 200 Hz | passed | too short to read (2 to 3 s of wall time in all) | 6.6x |
| `Copter.ModeAltHold` | quadx | passed | about 60x | |
| `Copter.ModeLoiter` | quadx | passed | about 60x | |
| `Copter.ModeCircle` | quadx | passed | about 61x | |
| `Copter.RTLYaw` | quadx | passed, once the start heading was honoured | not read | |
| `Plane.LOITER` | plane, 100 Hz | passed, once servo reversals were passed on | 70 to 77x; 7 s of wall time in all | about 21x |
| `Plane.AHRS2Logging` | plane | passed | about 65x | |
| `Plane.AerobaticsScripting` | plane3d, 100 Hz | passed | held at 5x by the test itself | 5x |
| `QuadPlane.GUIDEDToAUTO` | quadplane, 200 Hz | passed | about 54x | about 11x |
| `QuadPlane.Mission` | quadplane | passed | about 64x; 37 s of wall time | 6.3x |

With the external viewer open and `--speedup=10` asked, `QuadPlane.Mission`
passed at exactly 10x: the viewer costs the physics nothing.

### The whole test list (2026-11-15)

`test-all.sh` on this backend, `SPEEDUP=50`, with the external viewer open:
14 of the 15 tests pass, in one SITL binary with no Godot in the loop.

| Test | Result | Speed |
| --- | --- | --- |
| `Copter.TakeoffAlt` | passed | 48x |
| `Helicopter.PosHoldTakeOff` | passed | 49x |
| `Plane.TakeoffTakeoff1` (catapult) | passed | 41x |
| `Plane.AerobaticsScripting` | **failed**: "Missing trick HumptyBump" | 51x |
| `Plane.TakeoffBungee` | passed | 41x |
| `Plane.GliderDrop` | passed | not read |
| `QuadPlane.Mission` | passed | 44x |
| `Rover.DriveRTL` | passed | 50x |
| `Rover.DriveMotorBoat` | passed | not read |
| `BalanceBot.DriveRTL` | passed | 48x |
| `Sailboat.DriveRTL` | passed | 49x |
| `Hexapod.Walk` | passed | 22x |
| `Sub.DiveMission` | passed | 14x |
| `Tracker.TrackPlane` | passed | 50x |
| `Blimp.FourMotorHold` | passed | 50x |

- **The one failure is not the backend.** `Plane.AerobaticsScripting` carried
  `SIM_SPEEDUP: 5` with the comment "need to give some cycles to lua". That
  cap was removed locally, along with every other per-test speedup override
  in the autotests; at 50x the test reports a missing trick, twice running.
  It passes at 10x and 20x, and passed once and failed once at 40x. It is
  now capped at 40x in the test, which is not yet reliable: unresolved.
- **Two ran well short of the 50x asked:** the Sub at 14x and the hexapod at
  22x. The Sub is fixed (see "What held the Sub back"); the hexapod has not
  been re-measured since.
- All 21 models now bake as flyable; none is refused.

What it took after the first flights: jointed bodies and hinge joints with
motors (hexapod, tracker), the sail sheet, wind and wind vane (sailboat), a
lake for anything that floats (boats, Sub), the launchers (catapult, bungee,
drop, toss), and the helicopter rotor. One fault found on the way: SITL
traps floating point faults and Jolt's vector code raises harmless ones, so
the stepper now holds the traps off while Jolt runs.

### What held the Sub back (2026-11-15)

The Sub's Jolt step costs about 5 us, so the physics was never the limit.
Three things were, none of them in this backend:

| Cause | Fix | Sub.DiveMission, 50x asked |
| --- | --- | --- |
| As first flown: 400 Hz loop, 400 Hz physics, debug build | | 14 to 17x |
| ArduSub runs its control loop at 400 Hz, four times the copter model's | `models/sub.parm`: `SCHED_LOOP_RATE 100`, `SIM_RATE_HZ 200`, as `quadx.parm` | 14 to 24x on its own |
| SITL stops whenever its MAVLink output backs up, to let the harness read: about 3,000 stops of 1 ms or more in one mission | `Tools/autotest/ardusub.py`: stream rate 5 Hz, as Copter's suite uses, which halves the stops | 38 to 42x |
| `autotest.py` builds debug binaries (`-O0`) unless told not to | `test-all.sh` now passes `--no-debug` | 50.8x, three runs |

- **The debug build affects every number above this section.** All speeds
  measured before this point, on every backend and for stock SITL, were
  against an unoptimised ArduPilot. Comparisons between backends still hold.
- **Not fixed:** `autotest.py --uds` (the unix socket link, whose output
  queue is far larger) fails to start with the Godot harness code, which
  expects a port number. It would remove most of the remaining stops.

### Each test's fastest speed (2026-11-15)

A search raised each test's `--speedup` from 50 in steps of 1.2 until it
failed or fell behind, then stepped down by 0.85 until one value passed
twice running and was held (measured speed at least 0.93 of the speed
asked). Optimised builds, the viewer open, a 16-thread desktop, nothing
else running. `test-all.sh` now asks each test for its own figure when
`SITL_JOLT_LIB` is set; `SPEEDUP=n` still overrides them all.

| Test | Fastest held | Measured, two runs | One step faster |
| --- | --- | --- | --- |
| `Copter.TakeoffAlt` | 75x | 76.1x, 77.0x | see below |
| `Helicopter.PosHoldTakeOff` | 85x | 84.0x, 85.5x | see below |
| `Plane.TakeoffTakeoff1` | 40x | 40.0x, 39.8x | 50x: passed, ran at 46x |
| `Plane.AerobaticsScripting` | 40x | 40.0x, 40.0x | 50x: failed, a trick missed |
| `Plane.TakeoffBungee` | 30x | 30.0x, 30.0x | 40x: passed, ran at 38x |
| `Plane.GliderDrop` | 40x | 37.5x, 38.3x | 50x: passed, ran at 41x |
| `QuadPlane.Mission` | 50x | 50.0x, 49.9x | 60x: failed |
| `Rover.DriveRTL` | 70x | 70.3x, 70.3x | 80x: failed |
| `Rover.DriveMotorBoat` | 80x | 80.6x, 80.1x | 95x: stalled, stopped after 25 s |
| `BalanceBot.DriveRTL` | 40x | 40.2x, 40.2x | 50x: failed |
| `Sailboat.DriveRTL` | 60x | 64.8x, 62.1x | 70x: stalled, stopped after 21 s |
| `Hexapod.Walk` | 30x | 28.0x, 30.4x | 40x: passed, ran at 18x |
| `Sub.DiveMission` | 50x | 49.8x, 48.5x | 60x: passed, ran at 46x |
| `Tracker.TrackPlane` | 50x | 50.0x, 50.0x | 60x: failed |
| `Blimp.FourMotorHold` | 70x | 68.9x, 68.8x | 80x: passed, ran at 68x |

- **Copter and helicopter** were found by an earlier form of the search,
  which asked for far more than could be had and took 0.9 of what was
  reached. They were not searched again by the stepping rule.
- **Asking for more can give less.** The hexapod ran at 18x when asked for
  40x or 50x, the Sub at 46x when asked for 60x: SITL's MAVLink queue
  fills, SITL pauses, and the Python harness falls behind the vehicle.
  Asked for 1000x a plane sat on the ground and the test stalled without
  failing. The hexapod is also the one model whose physics is costly
  (19 bodies, about 55 us a step).
- **Requests above 100x are given 100x** (`MAX_SPEEDUP` in
  `vehicle_test_suite.py`) for that reason.
- `Plane.AerobaticsScripting` held 40x twice here but has failed at 40x
  before: fixed since, see the next section.
- The lake's shore (below) was added part way through: tests from
  `Rover.DriveMotorBoat` on were measured with it, those before it without.
  The earlier tests use no water.

### What held six tests under 50x (2026-11-15)

Six tests in the table above were under 50x. None was held back by the
Jolt step itself; each cause was found by measurement before anything was
changed.

| Test | Was | Cause | Fix | Now, 50x asked |
| --- | --- | --- | --- | --- |
| `Plane.TakeoffTakeoff1`, `Plane.TakeoffBungee`, `Plane.GliderDrop` | 40x, 30x, 40x | The Python harness: at 10 Hz streams it spent 85% of a core reading MAVLink, and SITL stopped for it about ten times a simulated second | Plane and Rover suites ask for 5 Hz streams on these backends, as Copter, Sub and Blimp do; the harness drains its pexpects every 5 ms, not every message | 50.0x, 10 of 10 passed |
| `BalanceBot.DriveRTL` | 40x | The same: the harness fell behind and read `NAV_CONTROLLER_OUTPUT` after the drive was over | The same | 50.0x, 4 of 4 |
| `Hexapod.Walk` | 30x | SITL's own core was full: nineteen bodies cost 46 us a step, at 400 Hz | `hexapod.parm`: `SIM_RATE_HZ 100` | 50x, 3 of 3, core at 60 to 70% |
| `Plane.AerobaticsScripting` | 40x, marginal | SITL's clock ran on while the Lua script computed: 0.3 ms of wall time is 15 ms of flight at 50x, so the script got 40 to 50 ms between updates in place of 25 | `AP_HAL_SITL` holds the clock while the scripting thread runs | 50.0x; update interval 30 ms; 40x cap removed |

- **The harness profile** (`perf`, Python's perf trampolines): of its time
  in a test, 21% parsing MAVLink, 21% in its per-message hook, 12% of that
  draining pexpects.
- **The clock hold is in SITL, not in this backend**, and changes stock
  SITL too: a script now runs at one instant at any speedup. Its first form
  cost speed (19x): the main thread kept the scheduler's lock while waiting
  for a script which was waiting for that lock. The clock now runs while the
  script waits on a lock, and waits for the script again when that lock is
  given up.
- **The glider is close to the limit.** Its tuning needs a 200 Hz loop and
  so 400 Hz physics, which takes 70 to 75% of a core at 50x. In one full
  run its flight ran at 24x; in nine repeats it held 50x. This machine
  is a laptop which throttles when hot, which is the likely cause and is
  not confirmed.
- **`Tracker.TrackPlane` failed about half its runs at 50x**, for a reason
  which is not speed alone: the aeroplane circled from boot, and with the
  dish turning the EKF took 100 to 140 simulated seconds to use GPS, once
  never (313 s). The slower the harness's setup landed in simulated time,
  the worse. The aeroplane now starts once the tracker is ready to arm, as
  the stock tracker test does, and the EKF uses GPS at the same simulated
  second every run. The dish gets one lap to lock on before it is scored
  (it can hunt for 36 s), and the suite asks for 5 Hz streams here too:
  10 of 10 passed at 50x, worst yaw error 3 degrees.

### Against stock SITL (2026-11-15)

The same tests on SITL's own physics, no Godot and no Jolt, `--speedup=100`
asked, one run each. Speed is the median of half-second readings of SITL's
clock against the wall, taken from the harness's telemetry log; the method
read 50.0x on a Jolt run which reported 50.0x itself.

| Test | Jolt model, default asked | Jolt model, 100x asked: achieved | Stock SITL, 100x asked: achieved |
| --- | --- | --- | --- |
| `Copter.TakeoffAlt` | 75x | 95x | 46x |
| `Helicopter.PosHoldTakeOff` | 85x | 84x | 48x |
| `Plane.TakeoffTakeoff1` | 50x | 60x | 47x |
| `Plane.AerobaticsScripting` | 50x | 73x | 35x |
| `Plane.TakeoffBungee` | 50x | 90x | 39x |
| `Plane.GliderDrop` | 50x | 80x | cannot run: nothing holds the glider up |
| `QuadPlane.Mission` | 50x | 74x | 32x |
| `Rover.DriveRTL` | 70x | 88x | 94x |
| `Rover.DriveMotorBoat` | 80x | 97x | 100x |
| `BalanceBot.DriveRTL` | 50x | 70x | 90x |
| `Sailboat.DriveRTL` | 60x | 65x | 86x |
| `Hexapod.Walk` | 50x | 33x | 91x, on SITL's rover: it has no legs |
| `Sub.DiveMission` | 50x | 49x | 38x |
| `Tracker.TrackPlane` | 50x | 59x | 95x, and the test failed |
| `Blimp.FourMotorHold` | 70x | 57x | 78x |

- All fifteen passed on the Jolt model at 100x asked, one run each, after
  the fixes above. The defaults in `test-all.sh` were set before them and
  have not been raised to match.
- **Asking for more still costs some tests speed:** the hexapod holds 50x
  asked for 50x and ran at 33x asked for 100x; the blimp 70x and 57x.
- The two columns do not step alike: stock SITL's default physics rate is
  1200 Hz, the Jolt models' 100 to 400 Hz. Why stock's aircraft are its
  slow rows was not looked into.
- Several tests last a second or two of wall time, so a figure can rest on
  one to four readings.

### Not done yet

- **Comparing whole flights against the Godot backend.** Tests passing shows
  the vehicles fly acceptably, not that they fly the same.
- **The lake's shore is a bowl, not main.gd's lake.** The bed rises from
  the lake's depth to the ground between 0.55 and 1.0 of its radius
  (`lake=depth+radius` in the model string). Before this the bed was flat
  everywhere, and a speedboat run too fast drove off across "land" for
  ever without failing.
- **The catapult's shuttle** is not modelled; the push goes straight to the
  aircraft, as it does in the script.
- **Scenery and impact detection.**
- In the viewer: propellers turning, surfaces and hinges moving (only body
  poses are sent so far), the ground speed readout, and SITL's reason when
  it refuses a model.
- `sim_vehicle.py`, fleets.

## The bake

A text file per vehicle of plain records, one a line, each a keyword and
numbers: no JSON, so the stepper needs no parser to speak of. `native/bake.gd`
writes them into `native/build/bakes/` by running the loader and reading
what it built, so the bake holds what Godot actually made. It holds:

- **Bodies:** the vehicle and anything jointed to it: mass, inertia, centre
  of mass, start pose, friction, restitution (from the physics server, not
  from the file).
- **Shapes:** kind (box, sphere, capsule, cylinder), size, margin, pose on
  the body.
- **Parts:** kind, the hinge it rides on if any, its pose, its parameters.
- **Vehicle-wide values:** drag areas, air and water densities.
- **`unsupported` records** naming anything the stepper cannot fly yet, so
  it refuses the vehicle with the reason instead of flying it wrongly.

- **Joints:** the two bodies and the hinge's pose, and for a servo on a
  joint its limits, rate and torque.

Two ways to produce it, to be decided:

1. **Committed beside the models** (`models/*.bake`), written by the
   generator scripts or an export step, with a test that fails when a bake
   is older than its glTF. Flying then needs no Godot at all.
2. **Made at start-up** by the harness, which already runs Godot once to
   import the project. Nothing extra is committed, but Godot must be
   installed.

## The ceiling: stock SITL with no Godot (measured 2026-11-15)

Stock `arduplane` on SITL's own plane physics, `--speedup=100` asked, sitting
disarmed on the ground, on the same machine:

| Physics rate (`SIM_RATE_HZ`) | Achieved | Physics steps a second |
| --- | --- | --- |
| 100 Hz | 100x (all that was asked) | 10,000 |
| 400 Hz | 68x | 27,000 |
| 1200 Hz (SITL's default) | 34x | 40,000 |

The quadplane at 1200 Hz reached 19x.

So ArduPilot itself costs about 25 to 40 us a step, not the 140 us estimated
earlier from the round-trip timing: that estimate was wrong. With physics
in-process and cheap, 100x at 100 Hz and about 70x at 400 Hz are within
reach. The best any Godot-in-the-loop arrangement has managed is about 33x.

## Phases

| Phase | Deliverable | State |
| --- | --- | --- |
| 1. Bake writer | A `.gd` export that walks a loaded vehicle and writes the bake; run over all 21 models | Done |
| 2. Jolt in the build | Jolt compiled as a static library for the SITL board only, behind a build flag that defaults off | Done, as an opt-in at configure |
| 3. Bake reader and backend skeleton | A `SIM_Jolt` class registered as a SITL model: bodies, shapes and joints built from the bake, ground plane, gravity, state out to ArduPilot | Done |
| 4. Parts | Rotor, surface, wheel (Jolt ray cast), floater, drag, hinge, hinge with its joint motor, sail sheet and heli rotor are ported. The streamer is drawing only | Done |
| 5. World | Water and lake, wind, launchers (catapult, bungee, drop), scenery colliders, impact detection, reset on reboot | Wind, start heading, lake and launchers done (SITL's own `SIM_WIND_*` now reach the physics); scenery and impact detection to do |
| 6. Viewer | External: a small UDP stream of body poses and part angles from SITL, and a view-only mode in the Godot project, as a separate process | Body poses done and working; part angles to do |
| 7. Harness and tests | An `autotest.py` and `sim_vehicle.py` option, the frame-to-model map reused, `test-all.sh` run against the new backend | `autotest.py` done; the rest to do |

Phases 2 and 4 to 7 are the same work as in option C. Only phases 1 and 3
differ: a bake writer and reader in place of a C++ glTF loader.

## Design decisions

- **The glTF stays the source of truth.** Nobody edits a bake by hand.
- **No Godot at run time.** SITL needs only the bake; Godot is the editor,
  the bake writer and the optional viewer.
- **The viewer is external.** It runs as its own process and only listens,
  so the physics never waits for a picture.
- **One binary.** Jolt and the stepper are linked into the SITL executable
  as a static library; nothing is loaded at run time.
- **C++ standard.** ArduPilot builds with `-std=gnu++11`; Jolt needs C++17.
  Jolt and the backend's own source files compile at C++17 behind a
  C++11-clean header, as in option C.
- **Godot's backend is the reference.** Both use Jolt, so for the same servo
  outputs the two should agree closely. A comparison check like
  `native/check.gd`, extended to whole flights, is the main correctness
  tool.
- **Part models stay engine-free.** `native/sitl_parts` has no Godot or Jolt
  types in it, so the same code serves the Godot extension and SITL.
- **Fleets stay on Godot at first.** Each SITL instance is its own process,
  so vehicles in separate processes cannot collide inside an in-process
  Jolt.

## Risks

- **Two things to keep in step.** The bake format is a contract between the
  `.gd` writer and the C++ reader. A new part kind or field has to be added
  on both sides, and a stale bake flies the wrong vehicle.
- **Matching Godot's physics exactly.** Contact and joint behaviour depend
  on engine settings Godot sets internally. The hexapod (18 jointed bodies)
  and the wheeled vehicles are where differences would show.
- **The Jolt dependency.** A new submodule under `modules/`, MIT-licensed;
  whether ArduPilot accepts it is the maintainers' call.
- **Scope.** Smaller than option C by the loader, but the per-step rewrite
  is the larger half and is still all here.

## Open decisions

1. Bakes committed beside the models, or made at start-up.
2. Whether option B is a destination or a step on the way from option A:
   the bake writer is small and useful either way.
3. Where the experimental work lives: suggested, a separate local branch.
