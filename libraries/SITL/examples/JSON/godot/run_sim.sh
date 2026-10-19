#!/usr/bin/env bash
# Start the Godot physics backend and SITL together, or either on its own.
#
# usage: run_sim.sh [--headless] copter|plane [sim_vehicle.py options...]
#        run_sim.sh --backend-only [--headless] copter|plane
#        run_sim.sh --sitl-only copter|plane [sim_vehicle.py options...]
#        run_sim.sh --editor
#
#   --headless      run the backend without a window, so that SITL's
#                   --speedup can go faster than real time
#   --backend-only  run just the backend, e.g. to start SITL under a debugger
#   --sitl-only     run just SITL, e.g. with the backend in Godot's debugger
#   --editor        open the project in the Godot editor
#
# The Godot binary is taken from $GODOT, or found on the PATH. Extra backend
# options (see README.md) can be given in $GODOT_SIM_ARGS, for example
#   GODOT_SIM_ARGS="--wind=0,5,0" ./run_sim.sh plane
# When both are started, the backend is stopped when SITL exits.

set -e

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ARDUPILOT="$(cd "$HERE/../../../../.." && pwd)"

usage() {
    sed -n '4,7p' "$0" | sed 's/^# //' >&2
    exit 1
}

HEADLESS=0
BACKEND=1
SITL=1
EDITOR=0
while [ $# -gt 0 ]; do
    case "$1" in
        --headless) HEADLESS=1 ;;
        --backend-only) SITL=0 ;;
        --sitl-only) BACKEND=0 ;;
        --editor) EDITOR=1 ;;
        *) break ;;
    esac
    shift
done

find_godot() {
    local candidate
    for candidate in "$GODOT" godot godot4 Godot /Applications/Godot.app/Contents/MacOS/Godot; do
        if [ -n "$candidate" ] && command -v "$candidate" > /dev/null 2>&1; then
            GODOT="$(command -v "$candidate")"
            break
        fi
        if [ -n "$GODOT" ] && [ "$candidate" = "$GODOT" ]; then
            echo "Godot not found at '$GODOT', looking on the PATH" >&2
        fi
        GODOT=""
    done
    if [ -z "$GODOT" ]; then
        echo "Godot 4.4 or later not found: put it on the PATH as 'godot' or set GODOT" >&2
        exit 1
    fi
    local version major minor
    version="$("$GODOT" --version 2>/dev/null | head -n 1)"
    major="$(echo "$version" | cut -d. -f1)"
    minor="$(echo "$version" | cut -d. -f2)"
    if [ "${major:-0}" -lt 4 ] || { [ "$major" -eq 4 ] && [ "${minor:-0}" -lt 4 ]; }; then
        echo "$GODOT is version '$version'; Godot 4.4 or later is needed for Jolt" >&2
        exit 1
    fi
}

if [ $EDITOR = 1 ]; then
    find_godot
    exec "$GODOT" --editor --path "$HERE"
fi

case "$1" in
    copter)
        VEHICLE=ArduCopter; FRAME=quad; MODEL=quadx
        STANDARD_PARAMS=Tools/autotest/default_params/copter.parm ;;
    plane)
        VEHICLE=ArduPlane; FRAME=plane; MODEL=plane
        STANDARD_PARAMS=Tools/autotest/models/plane.parm ;;
    *)
        usage ;;
esac
shift

GODOT_ARGS=(--path "$HERE")
if [ $HEADLESS = 1 ]; then
    GODOT_ARGS+=(--headless --fixed-fps 50)
fi

if [ $BACKEND = 1 ] && [ $SITL = 0 ]; then
    find_godot
    # shellcheck disable=SC2086
    exec "$GODOT" "${GODOT_ARGS[@]}" -- --vehicle="res://models/$MODEL.gltf" $GODOT_SIM_ARGS
fi

if [ $BACKEND = 1 ]; then
    find_godot
    LOG="$(mktemp -t godot-sitl.XXXXXX)"
    # shellcheck disable=SC2086
    "$GODOT" "${GODOT_ARGS[@]}" -- --vehicle="res://models/$MODEL.gltf" $GODOT_SIM_ARGS > "$LOG" 2>&1 &
    GODOT_PID=$!
    TAIL_PID=""
    trap 'kill $GODOT_PID $TAIL_PID 2> /dev/null; rm -f "$LOG"' EXIT

    # wait for the backend to load the vehicle and listen for SITL
    for _ in $(seq 1 300); do
        if grep -q "waiting for SITL" "$LOG"; then
            break
        fi
        if ! kill -0 $GODOT_PID 2> /dev/null; then
            cat "$LOG" >&2
            echo "the Godot backend exited" >&2
            exit 1
        fi
        sleep 0.1
    done
    tail -n +1 -f "$LOG" &
    TAIL_PID=$!
fi

# SITL finds a frame's standard parameters by model name, which for JSON
# matches no frame, so give them explicitly, before the model's own
cd "$ARDUPILOT"
Tools/autotest/sim_vehicle.py -v "$VEHICLE" -f "$FRAME" --model JSON:127.0.0.1 \
    --add-param-file="$STANDARD_PARAMS" --add-param-file="$HERE/models/$MODEL.parm" "$@"
