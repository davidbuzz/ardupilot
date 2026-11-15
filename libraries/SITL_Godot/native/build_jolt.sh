#!/bin/bash
# Copyright (C) 2026 David Buzz
# SPDX-License-Identifier: GPL-3.0-or-later
# Part of ArduPilot and released under its licence: see COPYING.txt.

# Build build/libsitl_jolt.a: Jolt, the part models and the baked-vehicle
# stepper, for SITL to link (see sitl-2.0-optionB-*.md). Experimental.
# usage: native/build_jolt.sh
set -e
cd "$(dirname "${BASH_SOURCE[0]}")"
JOLT=build/JoltPhysics                  # the version Godot 4.7 bundles
OBJ=build/jolt-obj
FLAGS="-std=c++17 -O2 -DNDEBUG -fPIC -I$JOLT -I."
mkdir -p build $OBJ && touch build/.gdignore
[ -d $JOLT ] || git clone -q --depth 1 --branch v5.5.0 https://github.com/jrouwe/JoltPhysics.git $JOLT
compile() { # one source, when it is newer than its object
    o=$OBJ/$(echo "$1" | tr '/' '_').o
    [ "$o" -nt "$1" ] || g++ $FLAGS -c "$1" -o "$o"
}
export -f compile
export FLAGS OBJ
(find $JOLT/Jolt -name '*.cpp'; echo sitl_parts.cpp; echo jolt_sim.cpp) | xargs -P "${BUILD_JOBS:-6}" -I{} bash -c 'compile {}'
rm -f build/libsitl_jolt.a
ar rcs build/libsitl_jolt.a $OBJ/*.o
echo "built native/build/libsitl_jolt.a"
