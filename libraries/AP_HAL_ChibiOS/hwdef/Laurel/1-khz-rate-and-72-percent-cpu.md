# Laurel 1 kHz Backend and ~72% CPU Report (Current Branch State)

Date: 2026-05-12
Scope: exact settings currently present in this branch and currently observed runtime after flash + param-storage erase.

## 1. Current Laurel defaults (source of truth in this branch)

From `libraries/AP_HAL_ChibiOS/hwdef/Laurel/defaults.parm`:

- `SCHED_LOOP_RATE 100`
- `INS_FAST_SAMPLE 1`
- `INS_GYRO_RATE 0`
- `FSTRATE_DIV 3`
- `FSTRATE_ENABLE 0`
- `BRD_OPTIONS 1`
- `LOG_BACKEND_TYPE 0`
- `FRAME_CLASS 1`
- `FRAME_TYPE 1`
- `SERIAL3_PROTOCOL 23`
- `SERIAL4_PROTOCOL 2`
- `EK3_IMU_MASK 1`

Meaning of the active rate/scheduler combination now:

- `INS_GYRO_RATE=0` keeps the Invensense backend at 1 kHz (not 2 kHz)
- `FSTRATE_ENABLE=0` disables the dedicated fast rate thread path
- `SCHED_LOOP_RATE=100` sets main scheduler target to 100 Hz

## 2. Current Laurel compile-time defines in this branch

From `libraries/AP_HAL_ChibiOS/hwdef/Laurel/hwdef.dat`:

- `define HAL_EKF_IMU_MASK_DEFAULT 1`
- `define HAL_FS_MOUNT_RETRY_MS 30000`
- `define HAL_SDCARD_RETRY_INTERVAL_MS 30000`

Related runtime behavior change in this branch:

From `libraries/AP_HAL_ChibiOS/Storage.cpp`:

- RP2350 path uses `healthy_timeout_ms = 30000U`
- non-RP2350 path uses `healthy_timeout_ms = 2000U`

This was added to reduce false early storage-health failure conditions during startup pressure.

## 3. Parameter persistence rule (critical)

This board persists non-default params in flash storage:

- param storage region: `0x10008000 .. 0x10009FFF` (8 KB)

So after reflashing, runtime params may still come from saved storage, not from `defaults.parm`.
Defaults are re-applied only when storage is erased/uninitialized.

## 4. Currently observed runtime profile

After flash + explicit erase of param storage (`monitor flash erase_address pad 0x10008000 0x2000`), observed perf lines were in this range:

- `Perf: main~112-117Hz rate~988Hz load~65-71%`

Representative lines seen:

- `AP: Perf: main=117Hz rate=988Hz load=65%`
- `AP: Perf: main=116Hz rate=988Hz load=65%`
- `AP: Perf: main=112Hz rate=988Hz load=70%`
- `AP: Perf: main=111Hz rate=988Hz load=69%`

This is the basis for the file name "1-khz-rate-and-72-percent-cpu".

## 5. Other active branch edits that may affect diagnostics/behavior

Current modified files also include:

- `libraries/AP_HAL_ChibiOS/Scheduler.cpp` (uses `HAL_FS_MOUNT_RETRY_MS` for mount retry timing)
- `libraries/AP_InertialSensor/AP_InertialSensor_Invensensev3.cpp` (fast-sampling gating/debug changes)
- `libraries/AP_NavEKF3/AP_NavEKF3.cpp` (allocation diagnostics)
- `libraries/GCS_MAVLink/GCS_FTP.cpp` (ENOMEM diagnostics)

These are branch-local changes and are part of the current behavior surface.

## 6. One-line current state

Current branch Laurel defaults are a 1 kHz backend with 100 Hz scheduler and fast-rate thread disabled, producing approximately mid-60s to low-70s CPU load after a clean param-storage erase.
