#!/bin/bash
# Record the vehicle editing tutorial video: run the Godot editor on a
# virtual display, have director.gd drive it through the steps with
# captions, and capture the screen with ffmpeg.
#
#   tools/tutorial/record_tutorial.sh [output.mp4]
#
# Needs Godot 4.4 or later ($GODOT, or godot on the PATH), Xvfb and ffmpeg.
# It works on a copy of the project, so the tree is left untouched. Set
# DISPLAY_NUM to use another X display number than 91.

set -e

HERE=$(cd "$(dirname "$0")" && pwd)
PROJECT=$(cd "$HERE/../.." && pwd)
OUT=$(realpath -m "${1:-godot_vehicle_editor_tutorial.mp4}")
GODOT=${GODOT:-$(command -v godot || command -v godot4 || true)}
DISPLAY_NUM=${DISPLAY_NUM:-91}
SIZE=1600x900

if [ -z "$GODOT" ]; then
    echo "Godot 4.4 or later not found: put it on the PATH as 'godot' or set GODOT" >&2
    exit 1
fi
for tool in Xvfb ffmpeg; do
    command -v $tool > /dev/null || { echo "$tool not found" >&2; exit 1; }
done

WORK=$(mktemp -d)
P=$WORK/project
XP=
EP=
FP=
cleanup() {
    for pid in $FP $EP $XP; do
        kill $pid 2> /dev/null || true
    done
    sleep 1
    [ -n "$EP" ] && kill -9 $EP 2> /dev/null || true
    rm -rf "$WORK"
}
trap cleanup EXIT

editor() {
    DISPLAY=:$DISPLAY_NUM "$GODOT" --editor --single-window --path "$P" --rendering-driver opengl3 \
        --resolution $SIZE --position 0,0 res://main.tscn > "$WORK/editor-$1.log" 2>&1 &
    EP=$!
}

# a copy of the project, without its cache, with the director installed
mkdir -p "$P"
(cd "$PROJECT" && tar cf - --exclude=./.godot --exclude=./tools --exclude='*.import' --exclude='*.uid' .) |
    (cd "$P" && tar xf -)
mkdir -p "$P/addons/director" "$P/scratch"
cp "$HERE/director.gd" "$P/addons/director/plugin.gd"
cp "$HERE/plugin.cfg" "$P/addons/director/plugin.cfg"
sed -i 's|^enabled=PackedStringArray(\(.*\))|enabled=PackedStringArray(\1, "res://addons/director/plugin.cfg")|' \
    "$P/project.godot"
grep -q addons/director "$P/project.godot" || { echo "could not enable the director plugin" >&2; exit 1; }

Xvfb :$DISPLAY_NUM -screen 0 ${SIZE}x24 > /dev/null 2>&1 &
XP=$!
sleep 1

# import the project and make the plane scene once, then start from the
# glTF again so the video shows the conversion
echo "importing the project"
echo prep > "$P/scratch/mode.txt"
editor prep
for i in $(seq 1 180); do
    kill -0 $EP 2> /dev/null || break
    sleep 1
done
kill $EP 2> /dev/null && { echo "the editor did not finish preparing; see $WORK/editor-prep.log" >&2; exit 1; }
EP=
rm -f "$P/models/plane.tscn"

# the 3D view's camera for the plane scene, kept by the editor in a file
# named after the md5 of the scene's path
mkdir -p "$P/.godot/editor"
cat > "$P/.godot/editor/plane.tscn-editstate-$(echo -n res://models/plane.tscn | md5sum | cut -d' ' -f1).cfg" << 'EOF'
[editor_states]

3D={
"viewport_mode": 1,
"viewports": [{
"distance": 2.6,
"orthogonal": false,
"position": Vector3(0, 0.22, 0),
"transform_gizmo": true,
"view_type": 0,
"x_rotation": 0.45,
"y_rotation": 2.3
}]
}
EOF

echo "recording"
echo film > "$P/scratch/mode.txt"
editor film
sleep 20
ffmpeg -loglevel error -y -f x11grab -draw_mouse 1 -video_size $SIZE -framerate 15 -i :$DISPLAY_NUM \
    -c:v libx264 -preset veryfast -crf 18 -pix_fmt yuv420p "$WORK/raw.mp4" < /dev/null &
FP=$!
sleep 1
touch "$P/scratch/go"
for i in $(seq 1 600); do
    [ -f "$P/scratch/done" ] && break
    kill -0 $EP 2> /dev/null || { echo "the editor exited; see $WORK/editor-film.log" >&2; exit 1; }
    sleep 1
done
[ -f "$P/scratch/done" ] || { echo "the tutorial did not finish" >&2; exit 1; }
sleep 1
kill -INT $FP
wait $FP || true
FP=

ffmpeg -loglevel error -y -i "$WORK/raw.mp4" -c:v libx264 -preset slow -crf 22 -pix_fmt yuv420p \
    -movflags +faststart "$OUT"
echo "wrote $OUT"
