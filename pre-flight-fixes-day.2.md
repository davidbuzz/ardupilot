# mr_vmu_rt1176 — day 2: why the HUD was upside down

Companion to `pre-flight-fixes.md`. That file is the pre-flight parameter record
and its sections 1-7 are not to be edited to match a later opinion. This file is
the reasoning and the sources behind one specific bug found on 2026-09-27.

Convention as in day 1: **verified** = measured on this board or read from source
that was actually opened; **inferred** = reasoned and not yet confirmed. Nothing
in the conclusion below has been confirmed on hardware yet.

---

## 1. The bug

With the committed hwdef (per-socket rotations copied from PX4) and
`AHRS_ORIENTATION 0`:

- `ATTITUDE` roll **+179.8 deg**, `RAW_IMU` zacc **+988**, where a level
  right-way-up board reads about **-988**. HUD upside down, ground where sky
  should be. **Verified** (bench, 2026-09-27).

The board is **not** physically inverted (maintainer, definitive).

`ROTATION_YAW_270` maps (x,y,z) -> (y,-x,z) and so cannot change Z, and the
board rotation was 0, so nothing in that configuration applied the 180 degree
flip the board needs. The rotation does reach the driver - the generated
`build/mr_vmu_rt1176/hwdef.h` contains
`HAL_INS_PROBE3 ... get_device("imu_sensor2"), ROTATION_YAW_270` - so this was
never a plumbing fault. **Verified.**

---

## 2. Root cause: PX4 rotates in the driver, ArduPilot does not

**PX4** negates two axes inside the driver, before any `-R` rotation is applied.
`src/drivers/imu/invensense/icm42688p/ICM42688P.cpp`:

```c
739:  accel.y[i] = (accel.y[i] == INT16_MIN) ? INT16_MAX : -accel.y[i];
740:  accel.z[i] = (accel.z[i] == INT16_MIN) ? INT16_MAX : -accel.z[i];
...
788:  gyro.x[i] = gyro.x[i];
789:  gyro.y[i] = (gyro.y[i] == INT16_MIN) ? INT16_MAX : -gyro.y[i];
790:  gyro.z[i] = (gyro.z[i] == INT16_MIN) ? INT16_MAX : -gyro.z[i];
```

Accel and gyro are treated identically, so there is no accel/gyro frame
mismatch to worry about. **Verified** (read via `gh api`).

**ArduPilot** takes the axes as read.
`libraries/AP_InertialSensor/AP_InertialSensor_Invensensev3.cpp:533`:

```c
Vector3f accel{float(d.accel[0]), float(d.accel[1]), float(d.accel[2])};
Vector3f gyro{float(d.gyro[0]), float(d.gyro[1]), float(d.gyro[2])};
```

and hands them straight to `_rotate_and_correct_accel()`. **Verified.**

Negating y and z is exactly `ROTATION_ROLL_180`: (x,y,z) -> (x,-y,-z).

### The translation rule

    ArduPilot hwdef rotation  =  PX4's -R  composed with  ROLL_180 applied FIRST

i.e. `R_ap = R_px4 ∘ ROLL_180`. **Copying a PX4 `-R` value straight into an
ArduPilot hwdef is wrong by 180 degrees of roll, every time.** That is what was
done here, and it is the whole bug.

---

## 3. The rule validated on hardware neither of us configured

Holybro Pixhawk 6X REV6 exists in both autopilots, so both values are known
independently. ArduPilot from
`libraries/AP_HAL_ChibiOS/hwdef/Pixhawk6X/hwdef.dat` (`BOARD_MATCH(FMUV6_BOARD_HOLYBRO_6X_REV6)`),
PX4 from `boards/px4/fmu-v6x/init/rc.board_sensors`:

| Part | PX4 `-R` | `-R` composed with ROLL_180 | ArduPilot hwdef says | |
|---|---|---|---|---|
| `iim42652` | `-R 6` = `YAW_270` | `ROLL_180_YAW_270` | `ROTATION_ROLL_180_YAW_270` | match |
| `icm45686` | `-R 10` = `ROLL_180_YAW_90` | `YAW_90` | `ROTATION_YAW_90` | match |

Two independent matches. Computed numerically against the actual case bodies in
`libraries/AP_Math/vector3.cpp`, not from the enum names. **Verified.**

(`adis16470` is a different driver with its own convention and is not evidence
either way for the Invensense rule.)

---

## 4. What that makes the correct value for this board

PX4's facts for FMUv6X-RT, verified via `gh api` against
`boards/px4/fmu-v6xrt/`:

`src/spi.cpp`, for hardware types V6XRT000 and V6XRT001:

```
LPSPI1: DRV_IMU_DEVTYPE_ICM42686P
LPSPI2: DRV_IMU_DEVTYPE_ICM42688P
LPSPI3: DRV_GYR_DEVTYPE_BMI088 + DRV_ACC_DEVTYPE_BMI088
```

`init/rc.board_sensors`:

```
icm42688p -6 -R 12 -b 1 -s start      # bus 1
bmi088 -A -R 4 -s start ; bmi088 -G -R 4 -s start
icm42688p    -R  6 -b 2 -s start      # bus 2
bmm150 -I start                        # INTERNAL, no -R
bmp388 -I -b 3 -a 0x77 ; bmp388 -I -b 2
```

PX4 uses **no global board rotation at all**. Our board is a revision PX4
supports, and the `0x44` WHOAMI on lpspi1 is the ICM-42686P that PX4 expects
there - an earlier doubt about the board revision was unfounded.

**Only socket 2 is live for us.** `check_whoami()` in
`AP_InertialSensor_Invensensev3.cpp` accepts ICM40609, ICM42688 (P and V),
ICM42605, ICM40605, IIM42652, IIM42653, ICM42670, ICM45686 and ICM56686 - and
**no ICM-42686** - so lpspi1 never initialises. `INS_ACC_ID` is set with no
`INS_ACC2_ID`/`ACC3_ID`, confirming one IMU as instance 0. **Verified.**

Applying the rule:

| Socket | PX4 | AP equivalent |
|---|---|---|
| `imu_sensor2` (LPSPI2, the live one) | `-R 6` `YAW_270` | **`ROTATION_ROLL_180_YAW_270`** |
| `imu_sensor1` (LPSPI1) | `-R 12` `PITCH_180` | no single-enum match; inert anyway, no AP driver |
| BMI088 (LPSPI3) | `-R 4` `YAW_180` | no single-enum match; currently undeclared in our hwdef |

**Proposed, NOT yet verified on hardware:**

```
IMU Invensensev3 SPI:imu_sensor2 ROTATION_ROLL_180_YAW_270
AHRS_ORIENTATION = 0
```

---

## 5. Why the correction must be per-IMU and not AHRS_ORIENTATION

`AP_Compass_Backend::rotate_field()`:

```c
if (!state.external) { mag.rotate(_compass._board_orientation); }  // AHRS_ORIENTATION
else                 { mag.rotate(state.orientation.get()); }       // COMPASS_ORIENT
```

and `libraries/AP_AHRS/AP_AHRS_Backend.cpp:86-87` feeds `AHRS_ORIENTATION` into
**both** the INS and the compass - but `rotate_field()` only uses it when that
compass is **internal**. Our hwdef declares the BMM150 **external**
(`COMPASS BMM150 I2C:2:0x10 true ROTATION_NONE`), where PX4 starts it `-I`
internal. So a non-zero `AHRS_ORIENTATION` rotates the IMU and leaves the
compass behind, putting the two frames 180 degrees apart - the constant compass
error and the `AngErr=178`. A per-IMU rotation is applied inside the backend,
before offsets, so the compass never sees the discrepancy. **Verified** from
source.

---

## 6. The day-1 warning was correct, and this is the proof

`pre-flight-fixes.md` section 1 says:

> `AHRS_ORIENT=8` corrects the Z axis, so a level board reads a convincing
> `accel z = -1 g` while the horizontal frame is yawed 180 degrees. It looks
> right on the bench and diverges on takeoff.

Computed against `vector3.cpp`:

| Configuration | Net rotation | Single-enum equivalent |
|---|---|---|
| Bench-level: `YAW_270` in hwdef + `AHRS_ORIENTATION 8` | `ROLL_180` after `YAW_270` | `ROTATION_ROLL_180_YAW_90` |
| Correct PX4 equivalent | `YAW_270` after `ROLL_180` | `ROTATION_ROLL_180_YAW_270` |

Those differ by **`YAW_180`** - forward is backward. The bench-level
configuration is exactly the trap described above: correct Z, 180 degrees of
yaw error, and a stationary board cannot reveal it because gravity has no
heading. **Verified by arithmetic.**

Two consequences worth stating plainly. A level roll/pitch reading is **not**
evidence that an orientation is right. And `ROTATION_ROLL_180_YAW_90`, which was
proposed earlier in the session from that bench reading, is the wrong-by-180
value; it was composed in the wrong order (board-rotation-after instead of
driver-flip-before) and would have flown backwards.

---

## 7. Still open

- **Nothing in section 4 is confirmed on hardware.** Needs a build, a flash, and
  then the physical check: nose down pitches down, roll right rolls right, yaw
  right increases heading.
- `imu_sensor1` and the BMI088 composed rotations have no single-enum match, so
  if either is ever driven they need separate handling.
- The BMI088 on LPSPI3 is wired and started by PX4 with `-R 4` but is not
  declared in our hwdef.
- The compass external/internal divergence from PX4 remains. It is not needed to
  fix this bug once the rotation is per-IMU, but it is a real difference.
- Accelerometer, compass and RC calibration have still never been done on this
  board, and all of them must be redone after any rotation change.
