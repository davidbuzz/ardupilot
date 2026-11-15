#!/bin/bash
# Copyright (C) 2026 David Buzz
# SPDX-License-Identifier: GPL-3.0-or-later
# Part of ArduPilot and released under its licence: see COPYING.txt.

# Build the optional C++ rotor and surface models as a Godot extension.
# Without it the backend runs the same models from its scripts, slower.
# Linux only so far. usage: native/build.sh
set -e
cd "$(dirname "${BASH_SOURCE[0]}")"
CPP=build/godot-cpp                     # Godot's C++ bindings, fetched once
LIB=$CPP/bin/libgodot-cpp.linux.template_release.x86_64.a
mkdir -p build && touch build/.gdignore  # keeps Godot out of the bindings' sources
[ -d $CPP ] || git clone -q --depth 1 --branch godot-4.5-stable https://github.com/godotengine/godot-cpp.git $CPP
[ -f $LIB ] || (cd $CPP && "${SCONS:-scons}" platform=linux target=template_release -j"${BUILD_JOBS:-6}")
g++ -std=c++17 -O2 -fPIC -shared -fvisibility=hidden -I$CPP/include -I$CPP/gen/include -I$CPP/gdextension \
    sitl_parts.cpp godot_binding.cpp $LIB -o libardupilot_native.so
cat > ardupilot_native.gdextension <<'CFG'
[configuration]

entry_symbol = "ardupilot_native_init"
compatibility_minimum = "4.5"

[libraries]

linux.x86_64 = "res://native/libardupilot_native.so"
CFG
"${GODOT:-godot}" --headless --path .. --import > build/import.log 2>&1  # Godot lists its extensions on import
echo "built native/libardupilot_native.so"
