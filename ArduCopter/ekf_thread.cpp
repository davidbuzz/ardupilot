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
#include "Copter.h"
#include <AP_InertialSensor/AP_InertialSensor_rate_config.h>

#if defined(RP2350)

#pragma GCC optimize("O2")

/*
  EKF thread — runs NavEKF3::UpdateFilter() on core1, below the rate thread
  in priority, so PID math is never starved.

  Design:
  - core0 calls signal_ekf_thread() immediately after AP_InertialSensor::update()
    each scheduler tick, waking this thread with fresh delta-angle/velocity data.
  - This thread calls AP_AHRS::update_EKF3_from_thread() which:
      1. Runs ekf3.update() (~2ms) without holding _rsem.
      2. Briefly acquires _rsem to copy results into the shared AHRS state.
  - read_AHRS() on core0 calls ahrs.update(true) which skips ekf3.update()
    (because _ekf_runs_in_thread is set) and reads the cached state directly.
  - One-tick latency is introduced: read_AHRS sees the EKF result from the
    previous tick's INS data, not the current tick. At 200-300 Hz this is
    3-5 ms, acceptable for the outer attitude loop.
*/
void Copter::ekf_thread()
{
    while (true) {
        _ekf_notifier.wait_blocking();
        const uint32_t t0 = AP_HAL::micros();
        AP::ahrs().update_EKF3_from_thread();
        const uint32_t dur = AP_HAL::micros() - t0;
        _ekf_last_duration_us   = dur;
        _ekf_total_duration_us += dur;
    }
}

/*
  Called as a FAST_TASK immediately after AP_InertialSensor::update() so the
  EKF thread on core1 wakes with the freshest possible delta-angle data.
*/
void Copter::signal_ekf_thread()
{
    if (started_ekf_thread) {
        _ekf_notifier.signal();
    }
}

#endif  // defined(RP2350)
