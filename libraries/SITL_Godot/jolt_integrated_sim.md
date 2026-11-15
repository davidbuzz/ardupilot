# The Jolt integrated sim (`--jolt`) and its limits

`--jolt` steps a baked copy of a Godot model inside the SITL binary, with
the Jolt engine linked in. Godot is only a viewer: SITL sends it body
poses and it draws them. It is experimental and local to this branch.

`--godot` is the other way to run the same models: Godot steps the physics
in a second process and SITL talks to it a packet a step. Where a limit
below says "`--godot` does this", that is the one to use instead.

How to build and run it is in `README.md`; how it was built and measured
is in `sitl-2.0-optionB-godot-bakes-c++-steps.md`; what could lift the
fleet limits is in `future-work.md`. This file is the list of what it
cannot do.

Written 2026-11-15.

## Several vehicles

Several `--jolt` vehicles can be flown and seen together
(`autotest.py --godot-vehicles 2`, `sim_vehicle.py -n 2`). Each is a SITL
process of its own, and one viewer draws them all.

- **They cannot touch.** Each vehicle is alone in its own Jolt world, so
  two of them pass through each other. Nothing one does is felt by
  another: no collisions, no landing on each other, no towing.
  `--godot` puts up to five in one world, where they do collide.
- **Their clocks are not tied together.** Each SITL keeps to the speedup
  it was asked for and no more. One which falls behind is not waited for,
  so two vehicles drawn side by side may be at different simulated times.
  The time and rate the viewer shows are those of the vehicle in focus.
- **Homes are ignored for placement.** The viewer draws vehicle `i` at
  `i` x `--spacing` from the first (5 m east unless told otherwise), as
  `--godot` spawns a fleet. Instances given different home locations are
  still drawn that far apart, so the picture does not match the map.
- **At most five.** The viewer takes five vehicles and ignores the rest.
- **Only two were tried, both copters.** Three to five, vehicles of
  different models, and `test-all.sh` with a fleet are untested.
- **One viewer, on the same machine.** Each SITL sends its poses to
  `127.0.0.1`, port 9003 + 10 x instance. A viewer on another machine
  gets nothing.
- **No sign of a dead vehicle but "waiting".** A vehicle whose SITL sends
  nothing for two seconds goes back to waiting; it is not removed.

## What the viewer shows

- **Body poses only.** Each rigid body is drawn where it is. Nothing on a
  body moves: propellers do not turn, control surfaces and tilting motors
  do not move. `--godot` shows all of these.
- **No ground speed readout**, and no message when SITL refuses a model.
- **It is not the simulation.** SITL never waits for the viewer, and what
  it draws is a 50 Hz (wall time) sample of what SITL did.

## The world

- **One vehicle and flat ground.** No scenery, and impacts are not
  detected.
- **The lake is a bowl.** The bed rises from the lake's depth to the
  ground between 0.55 and 1.0 of its radius. It is not the lake `--godot`
  draws.
- **The catapult's shuttle is not modelled.** The push goes straight to
  the aircraft.

## Building and starting

- **The firmware must be built with `--jolt`.** That is what links the
  stepper in. A binary built without it is refused, with a message.
- **Godot is still needed once.** Models are baked by Godot the first
  time, and again when a model or the loader changes, even for
  `--jolt-headless`.
- **`--jolt` with `--godot` is refused.** A run is one or the other.

## Speed

- **Not every test reaches 100x.** Each test in `test-all.sh` passes at 50x
  or more, and asks for the fastest it was measured to hold. Asked for
  100x, the hexapod reached 33x and the Sub 49x.
- **Some run slower than stock SITL.** The rovers, the balance bot, the
  sailboat and the blimp were faster on SITL's own physics when both were
  asked for 100x.
- **The test harness is often the limit**, not the physics: it has to read
  every message SITL sends.
- **Measured on one laptop, one run a test.** That laptop slows itself
  when hot, and tests which keep a core busy can dip.

## Fidelity

- **Not compared against `--godot` flight for flight.** The tests passing
  shows the vehicles fly acceptably, not that they fly the same as on the
  Godot backend.
