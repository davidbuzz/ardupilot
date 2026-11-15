// Copyright (C) 2026 David Buzz
// SPDX-License-Identifier: GPL-3.0-or-later
// Part of ArduPilot and released under its licence: see COPYING.txt.
#pragma once // plain C: SITL links this
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct JoltSim JoltSim;

typedef struct { // ArduPilot's frames: NED, body forward-right-down
    double gyro[3]; // rad/s
    double accel_body[3]; // specific force (m/s^2)
    double position[3]; // from the start (m)
    double velocity[3]; // m/s
    double dcm[9]; // body to NED, by rows
    double airspeed; // along the nose (m/s)
    int has_airspeed;
    double vane_direction, vane_speed; // apparent wind, where from (rad), m/s
    int has_vane;
} JoltSimState;

JoltSim *jolt_sim_create(const char *bake_path, double lake_depth, char *error, int error_size); // null and reason on failure
void jolt_sim_destroy(JoltSim *sim);
void jolt_sim_reset(JoltSim *sim); // back to where it starts
void jolt_sim_set_heading(JoltSim *sim, double degrees); // start heading from north
void jolt_sim_set_wind(JoltSim *sim, double north, double east, double down);
void jolt_sim_set_launcher(JoltSim *sim, const char *kind, double a, double b, double c); // sled, ramp, drop, toss; servo 7
void jolt_sim_step(JoltSim *sim, const uint16_t *pwm, int channels, double dt, JoltSimState *state);
int jolt_sim_poses(JoltSim *sim, double *out, int max_bodies); // 7 doubles a body: position, xyzw

#ifdef __cplusplus
}
#endif
