# Laurel / RP2350 Known Bugs and Flight-Blocking Issues

Last updated: 2026-06-24 (tlog analysis). Branch: `rp2350-v5-etc-dual-core`.

---

## FIXED

### BUG-002 — SD card logging disabled; logs never written
**Commit:** (pending — defaults.parm change, LOG_BACKEND_TYPE 1 @READONLY)
**Symptom:** SD card present and formatted but all log files missing after flight.
Mission Planner log download returns empty list.
**Root cause:** `LOG_BACKEND_TYPE 0` in defaults.parm explicitly disabled the logging
backend. Set during early bring-up to avoid pre-arm failures when SD card path was
unstable.
**Fix:** `LOG_BACKEND_TYPE 1 @READONLY` in defaults.parm. `HAL_LOGGING_BACKEND_DEFAULT_ENABLED 1`
and `HAL_SDCARD_SPI_INIT_TRIES 3` (was 1 — one failed probe meant 30-second retry delay)
in hwdef.dat.

---

## OPEN — TOP PRIORITY (fix before next flight)

### PERF-001 / FLY-002 — EKF CPU overload → auto-decimates to 8× (123 Hz) → all GPS modes blocked
**Observed:** 2026-06-24. Tlog: `laurel_2026-06-24_20-36-19.tlog`.
**Symptom (boot):** Every boot, EKF auto-decimation cascade fires within the first 40s:
`EKF CPU 99% (>50), decim->4` → `decim->5` → `decim->6` → `decim->7` → `decim->8`
Settles at: `C1: rate=988Hz ekf=123Hz ekf_duty=94% decim=8`. Core1 at 99%. Does not
improve once airborne.
**Symptom (flight):** With EKF barely converging at 123 Hz and saturated CPU, the EKF
solution is unreliable. Even with valid accel cal and excellent GPS (HDOP 0.58, 19 sats),
all GPS/altitude modes are blocked:
- Loiter, Auto, PosHold → `requires position`
- AltHold → `need alt estimate`
**Context:** EKF runs on Core1 (375 MHz Cortex-M33). Each EKF tick costs 1400–31000 µs
(min = prediction-only, max = GPS+baro measurement update). The slow ticks (~15 ms avg)
exceed the 8 ms slot at decim=8 → sustained 93–96% duty → CPU saturated.
Single EKF core (`EK3_IMU_MASK=1`). 24×24 float state matrix.
**Investigation path:**
  1. **Check FPU compiler flags** — Cortex-M33 has hardware FPU but it must be enabled:
     `-mfpu=fpv5-sp-d16 -mfloat-abi=hard`. If soft-float is used, all matrix ops are
     emulated in SW → 10–20× slower. Check `build/Laurel/compile_commands.json`.
  2. **Check EKF memory placement** — if state matrices land in slow SRAM or uncached XIP
     region, cache misses dominate. Check linker map for `NavEKF3` / `NavEKF3_core` symbols.
  3. **Try lower GPS measurement rate** — ublox at 5 Hz instead of 10 Hz halves expensive
     measurement-update ticks. Set `GPS_DRV_OPTIONS` bit to limit output rate.
  4. **Try `EK3_LOOP_RATE`** — pre-set decim to avoid the boot cascade of warnings.
**Not yet investigated.**

---

### BUG-004 — EKF completely dead; all GPS-assisted modes blocked
**Observed:** 2026-06-24 outdoor flight session. Tlog: `laurel_2026-06-24_20-36-19.tlog`.
**Symptom:** EKF_STATUS_REPORT flags = 0x0 throughout every armed interval (298 samples
across 4 arm events). No attitude solution, no velocity, no position, no altitude from
EKF. Vehicle stuck in STABILIZE. All mode-change attempts during arm #3 (5-minute armed
session) were rejected:
- `Mode change to Auto failed: requires position` (+578s)
- `Mode change to Loiter failed: requires position` (+591s, +602s, +604s, +605s)
- `Mode change to Position Hold failed: requires position` (+632s, +637s, +639s)
- `Mode change to Altitude Hold failed: need alt estimate` (+656s)
GPS was excellent throughout (HDOP avg 0.58, 17–19 sats, 100% 3D fix) — GPS is not the
problem.
**Root cause confirmed:** Accel calibration values not persisting (CAL-001). The
6-position calibration was performed multiple times (2026-06-23 and twice on 2026-06-24)
but `INS_ACCOFFS_*` are reverting to zero on each reboot or reflash. Without valid offsets
the accelerometer has a persistent ~0.4 m/s² lateral bias → DCM attitude error ~2–4° →
EKF3 detects DCM/EKF divergence (`AHRS: DCM Roll/Pitch inconsistent`, 10–25° reported)
and refuses to initialise → EKF flags stay 0x0 → all GPS/altitude modes blocked.
**`AHRS_ORIENTATION 8` (ROLL_180) — confirmed CORRECT** (2026-06-24): hand-hold test
showed corrections in the right direction. Board is mounted upside-down. Not the issue.
**Resolution:** Fix CAL-001 (parameter storage not persisting). Everything else unblocks.

---

## OPEN — Pre-flight Calibration (not code bugs, but block flight)

### CAL-001 — 6-position accel calibration not persisting across reboot (BUG)
**Symptom:** `PreArm: 3D Accel calibration needed` returns after every reboot or reflash
despite the full 6-position calibration having been completed. User performed the
6-position "Calibrate Accel" procedure at least twice on 2026-06-24 and once on
2026-06-23; the error reappeared each subsequent session. Correct behaviour is: do it
once, it persists in flash, never needed again until hardware changes.
**Root cause — two mechanisms, likely both contributing:**
  1. **Firmware reflash wipes parameter storage.** A firmware flash via OpenOCD
     (`program arducopter.bin ... 0x10010000`) erases all flash sectors the binary
     occupies. If the StorageFlash parameter area is within or adjacent to the flashed
     region, calibration values stored the previous day are wiped on reflash. This
     explains yesterday's cal being gone today.
  2. **INS_ACCOFFS_* may not be persisting even without reflash.** The two cals done
     on the evening of 2026-06-24 (before the flight in the tlog) also did not take.
     No firmware was reflashed between those cal attempts and the flight. This points to
     a separate StorageFlash write/read fault on RP2350: params are written but either
     not flushed to flash or the wrong flash area is being used.
**Investigation needed:**
  - After 6-position cal completes in Mission Planner, immediately fetch full param list
    and verify `INS_ACCOFFS_X/Y/Z` are non-zero and `INS_ACC_CALTEMP` is non-zero.
  - Reboot (without reflashing) and fetch params again — if INS_ACCOFFS_* revert to 0,
    the StorageFlash backend is broken.
  - Check `HAL_STORAGE_SIZE` and StorageFlash page layout in hwdef.dat vs the actual
    binary size to confirm the parameter area is not being overwritten by OpenOCD.
  - Check `AP_Param::save_io_worker()` / `StorageFlash::write()` paths on RP2350 for
    any early-return or failed erase condition.
**Workaround (none available):** Until fixed, calibration must be redone after every
reflash. The flight blocks remain in place until this is resolved. See BUG-004.

### CAL-002 — RC not calibrated / not found
**Symptom:** `PreArm: RC not found`. Mode changes via RC switches impossible. Prevents
arming.
**Action:** Connect RC receiver to SERIAL3 (PIOUART0, GPIO20/21) with SERIAL3_PROTOCOL=23
(RCIN/CRSF). Calibrate RC in Mission Planner → Initial Setup → Mandatory Hardware →
Radio Calibration. For PPM: GPIO21 edge-capture via SoftSigReaderRP2350.

### CAL-003 — Battery voltage/current not calibrated
**Symptom:** `PreArm: Battery 1 low voltage failsafe`. Prevents arming.
**Action:** Measure actual battery voltage, set `BATT_VOLT_MULT` and `BATT_AMP_PERVLT`
for the Laurel hardware voltage divider and current sense resistor values. Set
`BATT_LOW_VOLT` and `BATT_CRT_VOLT` to appropriate values for the battery chemistry.

---

## OPEN — Performance / EKF

---

## OPEN — Flight / Tuning

### FLY-001 — Stabilize mode: vehicle tips severely; cannot self-hold level
**Observed:** 2026-06-24 outdoor flight session. Tlog: `laurel_2026-06-24_20-36-19.tlog`.
**Symptom:** Vehicle was airborne in Stabilize across 4 arm events (57s, 10s, 290s, 90s).
During the 5-minute arm (#3), raw IMU data recorded three severe tilt spikes:
- +538s: `ax=8.50 m/s²` → ~60° tilt
- +718s: `ax=7.06 m/s²` → ~46° tilt
- +728s: `ax=8.14 m/s²` → ~56° tilt
The vehicle was tipping to near-horizontal and being caught. AHRS attitude estimate
(DCM) reported Roll ±11°, Pitch ±15° (average across all armed samples) — these are
the corrected attitude values; the raw IMU spikes show the vehicle was physically much
further over at those moments. Yawrate reached ±100 °/s, confirming dynamic instability.
**Root causes (two, independent):**
  1. **CAL-001 (accel calibration missing)** — EKF has no solution (flags 0x0). Stabilize
     falls back to DCM only, which itself has a biased attitude from the uncalibrated accel.
     The rate controller is working from a wrong reference frame.
  2. **Default PIDs not matched to Laurel** — default ArduCopter PIDs are sized for a
     generic ~450mm quad. Laurel frame geometry, motor KV, prop size, and all-up weight
     are not accounted for. Rate P/I/D gains need tuning.
**Action items (in order):**
  1. CAL-001: 6-position accel calibration — eliminates DCM attitude bias and allows EKF
     to initialise, giving the rate controller a correct attitude reference.
  2. Verify motor/ESC calibration — unequal thrust is indistinguishable from wrong PIDs.
  3. First flight attempt post-cal: fly conservatively in Stabilize at low altitude with
     spotter. If oscillating: reduce `ATC_RAT_RLL_P` / `ATC_RAT_PIT_P` by 20%.
     If sluggish/sagging: increase by 20%.
  4. Once able to hover, use AutoTune (`AUTOTUNE_AXES 3` for roll+pitch first).

### FLY-002 — Loiter / position / altitude modes unavailable
**Observed:** 2026-06-24 outdoor session. Tlog: `laurel_2026-06-24_20-36-19.tlog`.
**Symptom:** During arm #3 (5-minute armed session), while GPS was excellent (HDOP 0.58,
17–19 sats, 100% 3D fix), every GPS-assisted and altitude-assisted mode was rejected:
- Loiter, Auto, PosHold → `requires position`
- AltHold → `need alt estimate`
**Root cause:** EKF flags 0x0 throughout — no position, no altitude, no velocity estimate
from EKF (see BUG-004). GPS quality is not the problem; the EKF is simply not running.
**Resolution:** Fix CAL-001. Once accel calibration is done, EKF initialises, and GPS
lock (which was already present and healthy) provides the position solution. All these
modes should become available within ~30s of arming outdoors after cal is done.

