/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  A glTF vehicle of libraries/SITL_Godot, baked and stepped in this
  process by Jolt: --model jolt:path/to/vehicle.bake. Experimental: see
  libraries/SITL_Godot/sitl-2.0-optionB-godot-bakes-c++-steps.md
*/

#pragma once

#include "SIM_config.h"

#ifndef AP_SIM_JOLT_ENABLED
#define AP_SIM_JOLT_ENABLED 0 // needs native/build/libsitl_jolt.a linked in
#endif

#if AP_SIM_JOLT_ENABLED

#include "SIM_Aircraft.h"
#include <AP_HAL/utility/Socket_native.h>

struct JoltSim;

namespace SITL {

class Jolt : public Aircraft {
public:
    Jolt(const char *frame_str);

    void update(const struct sitl_input &input) override;

    static Aircraft *create(const char *frame_str) {
        return NEW_NOTHROW Jolt(frame_str);
    }

private:
    JoltSim *sim;
    char bake[256];
    uint32_t reversed; // reversed outputs, a bit each
    Vector3f steady_wind; // blown whatever SIM_WIND_* say (NED, m/s)
    float lake_depth, lake_radius; // 0 for the usual one
    char launcher[8]; // sled, ramp, drop or toss
    float launch[3];
    SocketAPM_native view_sock{true}; // poses for a viewer
    uint64_t view_wall_us;
    void send_poses(uint64_t wall_us);
    uint64_t report_wall_us; // for the speed achieved
    uint64_t report_sim_us;
};

} // namespace SITL

#endif // AP_SIM_JOLT_ENABLED
