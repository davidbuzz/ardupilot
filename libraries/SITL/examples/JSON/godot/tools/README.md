# Tools for the Godot backend

These are not part of the Godot project; `.gdignore` keeps the editor out
of this directory.

## Model generators

The vehicles in `models` were written by these scripts, which set out
every part's position and parameters in one place. Each writes its model
over the one in `models`, or to a file given as its argument:

```sh
./make_plane.py
./make_plane3d.py
./make_quadx.py [output.gltf]
```

Editing a model in the Godot editor and exporting it works as well; a
model changed that way no longer matches its script.

## Flight checks

`takeoff_check.py` and `landing_check.py` start their own backend and SITL,
set up as the autotest suite sets up ArduPlane's default frame,
`plane-elevrev`, on SITL instance 1 by default, so they can run beside an
autotest on instance 0. Build the vehicle first; they find Godot on the
PATH as `godot` or from `$GODOT`, and need pymavlink.

- `takeoff_check.py` flies the autotest suite's FBWA takeoff, holding
  rudder and up elevator from a standstill as `takeoff_in_FBWA()` does,
  and prints the ground roll and climb twice a second
- `landing_check.py` flies `test_flight.py`'s plane mission, with a runway
  takeoff, a circuit and an automatic landing; `--vehicle copter` flies the
  copter test instead
- `tlog_landing.py` prints each landing in a telemetry log from its flare
  until the vehicle disarms. The autotest suite leaves a `.tlog` for each
  test at the top of the tree, which this reads after a landing fails.

```sh
./takeoff_check.py --model plane --duration 30
./landing_check.py
./tlog_landing.py ../../../../../../autotest-1234.tlog
```

## Tutorial video

`tutorial/godot_vehicle_editor_tutorial.mp4` is a captioned walk through
editing a vehicle in the Godot editor, the steps listed under "Editing
vehicles" in the project's README. `tutorial/record_tutorial.sh` records it
again, for instance after the editor or the plane changes. It needs Xvfb
and ffmpeg as well as Godot, and works on a copy of the project:

```sh
tutorial/record_tutorial.sh tutorial/godot_vehicle_editor_tutorial.mp4
```

`tutorial/director.gd` drives the editor: it moves the pointer, opens
menus, selects nodes, sets values and shows the captions. Change the steps
in its `film()` function.
