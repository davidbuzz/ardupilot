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
  A baked glTF vehicle stepped by Jolt inside SITL: no second process,
  no packets, no text. The physics is libraries/SITL_Godot/native.
*/

#include "SIM_Jolt.h"

#if AP_SIM_JOLT_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <SITL_Godot/native/jolt_sim.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

using namespace SITL;

Jolt::Jolt(const char *frame_str) :
    Aircraft(frame_str)
{
    const char *colon = strchr(frame_str, ':'); // jolt:bake[,reverse=2+4][,wind=n+e+d][,lake=m][,launcher=kind+a+b+c]
    strncpy(bake, colon != nullptr ? colon + 1 : "", sizeof(bake) - 1);
    char *save = nullptr;
    strtok_r(bake, ",", &save);
    for (char *option = strtok_r(nullptr, ",", &save); option != nullptr; option = strtok_r(nullptr, ",", &save)) {
        if (strncmp(option, "reverse=", 8) == 0) {
            for (const char *p = option + 8; *p != 0; p = (strchr(p, '+') != nullptr) ? strchr(p, '+') + 1 : "") {
                const int channel = atoi(p);
                if (channel >= 1 && channel <= 32) {
                    reversed |= 1U << (channel - 1);
                }
            }
        } else if (strncmp(option, "wind=", 5) == 0) {
            float w[3] {};
            sscanf(option + 5, "%f+%f+%f", &w[0], &w[1], &w[2]);
            steady_wind = Vector3f(w[0], w[1], w[2]);
        } else if (strncmp(option, "lake=", 5) == 0) {
            lake_depth = atof(option + 5);
        } else if (strncmp(option, "launcher=", 9) == 0) {
            const char *plus = strchr(option + 9, '+');
            strncpy(launcher, option + 9, MIN(sizeof(launcher) - 1, size_t(plus != nullptr ? plus - (option + 9) : 7)));
            if (plus != nullptr) {
                sscanf(plus + 1, "%f+%f+%f", &launch[0], &launch[1], &launch[2]);
            }
        } else {
            printf("SIM_Jolt: unknown option %s\n", option);
        }
    }
    printf("Starting SITL: Jolt, %s\n", bake);
}

void Jolt::send_poses(uint64_t wall_us) // 50Hz, never waits
{
    if (wall_us - view_wall_us < 20000ULL) {
        return;
    }
    view_wall_us = wall_us;
    struct PACKED {
        uint32_t magic;
        uint32_t count;
        double time_s;
        double poses[7 * 32]; // position, quaternion xyzw: glTF axes
    } pkt;
    pkt.magic = 0x544C4F4A; // "JOLT"
    pkt.count = jolt_sim_poses(sim, pkt.poses, 32);
    pkt.time_s = time_now_us * 1.0e-6;
    view_sock.sendto(&pkt, 16 + pkt.count * 7 * sizeof(double), "127.0.0.1", 9003 + 10 * instance);
}

void Jolt::update(const struct sitl_input &input)
{
    if (sim == nullptr) {
        char why[256] {};
        sim = jolt_sim_create(bake, lake_depth, why, sizeof(why));
        if (sim == nullptr) {
            AP_HAL::panic("SIM_Jolt: %s", why);
        }
        jolt_sim_set_heading(sim, home_yaw);
        if (launcher[0] != 0) {
            jolt_sim_set_launcher(sim, launcher, launch[0], launch[1], launch[2]);
        }
    }
    if (sitl != nullptr && sitl->loop_rate_hz > 0) {
        adjust_frame_time(sitl->loop_rate_hz); // SIM_RATE_HZ is the physics rate
    }
    update_wind(input);
    wind_ef += steady_wind;
    jolt_sim_set_wind(sim, wind_ef.x, wind_ef.y, wind_ef.z);

    uint16_t servos[ARRAY_SIZE(input.servos)];
    for (uint8_t i = 0; i < ARRAY_SIZE(servos); i++) {
        const bool turn = (reversed & (1U << i)) != 0 && input.servos[i] >= 500; // a disabled output stays 0
        servos[i] = turn ? 3000 - input.servos[i] : input.servos[i];
    }
    JoltSimState s;
    jolt_sim_step(sim, servos, ARRAY_SIZE(servos), frame_time_us * 1.0e-6, &s);

    gyro = Vector3f(s.gyro[0], s.gyro[1], s.gyro[2]);
    accel_body = Vector3f(s.accel_body[0], s.accel_body[1], s.accel_body[2]);
    velocity_ef = Vector3f(s.velocity[0], s.velocity[1], s.velocity[2]);
    position = Vector3d(s.position[0], s.position[1], s.position[2]);
    position.xy() += origin.get_distance_NE_double(home);
    dcm = Matrix3f(Vector3f(s.dcm[0], s.dcm[1], s.dcm[2]),
                   Vector3f(s.dcm[3], s.dcm[4], s.dcm[5]),
                   Vector3f(s.dcm[6], s.dcm[7], s.dcm[8]));
    velocity_air_ef = velocity_ef - wind_ef;
    velocity_air_bf = dcm.transposed() * velocity_air_ef;
    if (s.has_airspeed) {
        airspeed = airspeed_pitot = s.airspeed;
    } else {
        update_eas_airspeed();
    }
    if (s.has_vane) {
        wind_vane_apparent.direction = s.vane_direction;
        wind_vane_apparent.speed = s.vane_speed;
    }

    update_position();
    time_advance();
    update_mag_field_bf();

    const uint64_t wall_us = get_wall_time_us();
    send_poses(wall_us);
    if (report_wall_us == 0 || wall_us - report_wall_us > 500000ULL) {
        if (report_wall_us != 0) {
            printf("Jolt: rate %.1fx real time (asked %.1fx)\n",
                   (time_now_us - report_sim_us) / double(wall_us - report_wall_us), double(target_speedup));
        }
        report_wall_us = wall_us;
        report_sim_us = time_now_us;
    }
}

#endif // AP_SIM_JOLT_ENABLED
