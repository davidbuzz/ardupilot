# Future work

Things worth doing which nobody has started. Each entry says what it is for,
what it rests on, and what was found out when it was looked into.

## TODO: `--jolt` vehicles which see and hit each other

Recorded 2026-11-15.

Each `--jolt` SITL process owns its own Jolt world. Several can be flown
and drawn together in one viewer, but they cannot see or touch each other.
`--godot` can fly five in one world, at about a fifth of the speed. This is
how `--jolt` could get fleets which touch without giving up the one-process
design.

### What it rests on

[PR #34019](https://github.com/ArduPilot/ardupilot/pull/34019), "Sitl shared
memory", read at head `cdec5df12d`. It supplies two of the three things
collisions between separate SITL instances need:

- **A shared clock.** Instances barrier-sync through a shared-memory segment
  so their simulated time stays together. Without it a pose read from a peer
  belongs to a different instant.
- **A shared state channel.** Each instance has a 4 KB slot guarded by a
  mutex. Every instance already publishes `AP_SITL_SwarmInfo` there (sysid,
  latitude, longitude, altitude, velocity, heading) and there is a getter
  for peers.

It does not do collisions. The swarm information is written every frame and
no physics model reads it.

### What is left to do

- [x] **One viewer for several `--jolt` instances.** Done 2026-11-15, with
  nothing from #34019: each instance sends its poses on its own port
  (9003 + 10 x instance), and the viewer listens on one port a vehicle and
  draws each `--spacing` from the last.
- [ ] **Publish full poses.** Collision needs every body's position and
  attitude, not a heading: nineteen bodies for the hexapod. The slot has
  room if the 512-byte swarm region is enlarged, or the poses go in as
  floats.
- [ ] **Publish the model's name**, so a peer can load that model's bake and
  have its colliders.
- [ ] **Ghost bodies.** Each instance adds its peers to its own Jolt world
  as kinematic bodies and moves them to the published poses every step. Its
  own vehicle then collides with them.
- [ ] **A tight clock barrier for this use.** See the limits below.
- [x] **Allow more than one vehicle with `--jolt`** in `autotest.py` and
  `sim_vehicle.py`. Done 2026-11-15, for flying and seeing them together.
- [ ] **Draw each vehicle where its home is.** The viewer places vehicle i
  at i x `--spacing`, as `--godot` spawns them; instances given different
  homes are still drawn that far apart.
- [ ] **A test**: two rovers driven at each other, and a tracker following a
  real aeroplane instance in place of the harness's made-up one.

### What it would and would not be good for

- **Good for "did they hit" and for bumping.** A mid-air, or one rover
  driving into another, would be found and both would react.
- **Poor for sustained contact.** Each side treats the other as infinitely
  heavy and sees it a step late, so landing one vehicle on another, towing
  and docking would jitter. Only one shared world does those properly, which
  is what `--godot` is for.
- **Only as accurate as the clock skew.** The PR's description gives about
  2.5 ms across five instances, which is ample. Its review found the skew
  allowed grows with the speedup, to 2 s at 100x; at 20 m/s that is 40 m.
  Collisions need the tight barrier, and that costs speed: every instance
  then runs at the pace of the slowest.

### State of #34019 when this was written

- Open, changes requested, +5271 lines. Its author stepped back from it on
  2026-09-02 and left it to tridge to carry forward.
- The part under dispute is the larger time steps at high speedups, which
  its author had already suggested dropping. The shared memory and the
  barrier are the parts this work needs.
- It changes the same SITL files as the clock hold for Lua scripts on this
  branch (`AP_HAL_SITL/Scheduler.cpp`, `SITL_State`). Both add a wait before
  the clock steps, so they should live together, but they will conflict when
  merged.

### When

Not before #34019, or its shared-memory core, is merged: this branch is
already unmerged work on top of the unmerged Godot backend
([PR #35174](https://github.com/ArduPilot/ardupilot/pull/35174)), and this
would be a third layer. The viewer item was the exception and is done.
