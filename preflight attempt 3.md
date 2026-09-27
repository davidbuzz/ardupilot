# Pre-flight attempt 3 — mr_vmu_rt1176 (AP_HAL_Zephyr)

Board: NXP MR-VMU-RT1176, i.MX RT1176 Cortex-M7 @996 MHz, XIP from 64 MB external
NOR. Branch `buzz-zephyr-v3`. All figures below are measured on hardware unless
marked otherwise.

**Outcome: did not fly. Hardware damaged during a bench motor test (section 7).**

---

## 1. Fixes landed, with evidence

### 1.1 I2C completion wait: flat 100 ms -> ChibiOS-style computed bound

`modules/zephyr/drivers/i2c/i2c_mcux_lpi2c.c`, eDMA transfer path.

The eDMA completion wait was `k_sem_take(&e->done, K_MSEC(100))` - a flat 100 ms.
ArduPilot retries a failed transfer 3 times (`_retries = 2`) **while holding the
per-bus `DeviceBus` semaphore**, so one NAKing device stalled the whole bus for
300 ms. `AP_HAL_ChibiOS/I2CDevice.cpp:374-376` instead bounds every attempt:

    uint32_t timeout_ms = 1+2*(((8*1000000UL/bus.busclock)*(send_len+recv_len))/1000);
    timeout_ms = MAX(timeout_ms, _timeout_ms);   // _timeout_ms defaults to 4

i.e. twice the expected transfer time, floored at 4 ms -> ~12 ms for three
attempts, against our 300 ms. **25x divergence from the source of truth.**

Replaced with a computed bound: bytes costed at the slowest legal I2C rate
(100 kHz, 9 bit-times/byte), 4 ms floor, **100 ms cap so it can only ever
shorten**. No bitrate lookup needed in the driver.

### 1.2 I2C retry budget (AP side)

`libraries/AP_HAL_Zephyr/I2CDevice.cpp` - `transfer()` now computes the ChibiOS
timeout and stops retrying once that budget is spent, rather than starting another
attempt that could block for the driver's whole internal wait.

### 1.3 Main-loop yield 100 us -> 400 us

`libraries/AP_HAL_Zephyr/HAL_Zephyr_Class.cpp:68`, applied at `:293`.

See section 3.1 for why this is the only lever that feeds the below-main threads.
Stepped 100 -> 250 -> 400 us with measurements at each step (section 2).

### 1.4 AP_InternalError + ARM cache ops into ITCM

`libraries/AP_HAL_Zephyr/zephyr/itcm_hot_code.ld`.

In-firmware PC profiling (~1 kHz, 10052 samples) measured `itcm=42% xip=57%`, and
the single hottest 64-byte bucket in the whole system was `0x300b4740` at
523/10052 = **5.2%**, resolving to the `AP_InternalError` region
(`errors_as_string` at `0x300b4718+100`, `AP::internalerror()` at `0x300b477c`).

That cannot be algorithmic work: `errors_as_string()` has exactly one caller,
`AP_Arming.cpp:1203`, at roughly 1 Hz. A 100-byte function called once a second is
not 5% of a 996 MHz CPU. **The samples are instruction-fetch stalls** - the CPU
parked at that PC waiting on FlexSPI. Corroborated by two long-branch veneers also
appearing hot (`__k_heap_init_veneer`, `__k_queue_append_veneer`); veneers exist
only because ITCM-resident code branches to XIP targets.

AP_InternalError.cpp is 681 B of text. Added with the Zephyr ARM cache ops
(`arch_dcache_flush_range` is called on every SPI DMA transfer, ~850/s from
`SPIDevice.cpp:45`, and its bucket was ~2%; 464 B for all `arch_dcache`/
`arch_icache` entries). ITCM went 459,976 -> 461,640 B of 491,520.

### 1.5 1 kHz PC sampler off by default

`libraries/AP_HAL_Zephyr/zephyr/src/ap_pcprofile.c` - now gated behind
`AP_ZEPHYR_PCPROFILE_ENABLED`, default 0. On a `CONFIG_TICKLESS_KERNEL` with a
1 MHz tick, every `k_timer` expiry reprograms the hardware timer, so a 1 kHz
sampler costs far more than the sample itself. Switching it off paid for the
250 -> 400 us yield step outright.

### 1.6 MAVFTP — upstream commit d90898ca

Applied as 6 hunks (FTPDIAG counters preserved). Two parts:

- the worker waits on a `HAL_BinarySemaphore` signalled at push, instead of
  polling `hal.scheduler->delay(2)`. At ~0.2% of CPU a polling thread must win the
  CPU repeatedly just to *discover* work; a semaphore needs one wake per request.
- `push_reply()` is bounded by `FTP_SESSION_KILL_TIMEOUT` instead of retrying
  `send_reply()` **forever**. The old unbounded retry hung the worker *and the
  session reaper* until reboot - exactly the observed signature: one `OpenFileRO`
  succeeds, then permanent silence.

Verified on hardware, all three paths:

| test | result |
|---|---|
| `OpenFileRO` x6, 2 s apart | 6 ACK / 0 silent |
| sequential `ReadFile` | 10,185 B to EOF, 45 reads, 1 retry |
| `BurstReadFile` (what GCSes use) | 42 packets, clean EOF |
| payload decode | `magic=0x671b`, header reports **910 params** |
| send counters | `dr0 tb0 ns0`, `en`=`ok` |

The 910 matches what the classic `PARAM_REQUEST_LIST` protocol enumerated
independently, so the container is valid and complete - a byte count alone proves
nothing (see 4.2).

---

## 2. Sensor health progression (all 70 s windows, disarmed)

`Bad Baro Health` / `Bad Compass Health` in the HUD. `SYS_STATUS` unhealthy
fraction, not snapshots - a single sample cannot distinguish "cleared" from "not
yet reappeared".

| build | MAG bad | ABS_PRESSURE bad | LoopRate | I2C1 per 10 s | I2C1 mean xfer | worst stall |
|---|---|---|---|---|---|---|
| start | 46% | 51% | 329 Hz | - | - | - |
| + I2C driver timeout | 13% | 11% | 380-399 Hz | - | - | 438 ms |
| + yield 250 + ITCM | 4% | 4% | 357-416 Hz | 427-690 | 6.0-9.7 ms | 306-582 ms |
| + yield 400, sampler off | **0%** | **0%** | **423-443 Hz** | 864-983 | 4.1-5.3 ms | 58-144 ms |
| later, clean boot | 2% | 0% | 307-336 Hz | 52-61 | 91-111 ms | see section 5.2 |

Devices: `BARO BMP388 I2C:1:0x76`, `BARO BMP388 I2C:2:0x77`,
`COMPASS BMM150 I2C:2:0x10`. BMM150 registers at `MEASURE_TIME_USEC 16667`
(60 Hz), BMP388 at 20 ms (50 Hz), so bus 2 needs 110 callbacks/s.

---

## 3. Mechanisms established

### 3.1 The below-main threads' CPU is `yield_us x achieved_loop_rate`

`AP_SCHEDULER_LOOP_YIELD_US` is the **only** CPU any thread below main receives.
Proof: with the machine 100% busy and 0% idle, main released **4.48%** of the CPU
with its boost dropped, against `100 us x 466 loops/s = 4.66%` predicted.

Income is therefore a product, and falls when either term falls:

- 443 Hz x 400 us = **17.7%** of the machine
- 290 Hz x 400 us = **11.6%**

Consequence, which inverts the usual intuition: **on this HAL a lower loop rate
hands the IO band LESS, not more**, because there are fewer yields per second.
Everything below main is affected - storage, log_io, AP_io, compasscal, the MAVFTP
worker, and the I2C sensor buses at prio 7.

Distribution measured: prio 7-9 received 1.71%, the whole prio>=10 band 0.97%
shared five ways.

Corollary, also measured: **raising a starved thread to a higher-but-still-below-
main priority creates no CPU** - it only reorders who wins the same scraps.
Promoting FTP 11 -> 8 would roughly 4x its share entirely at UART/storage/logger's
expense. This was tried and reverted.

### 3.2 Wall time is not work time - the I2C case

I2C bus threads run at prio 7, below main(3), SPI(2), timer(2), rate(1), rcin(6),
which together consume ~96%. A ~200 us transfer took **6-21 ms of wall time**,
worst case **3.19 seconds**, because the thread is preempted mid-transfer.

Proved by instrumenting the driver's phases: `lk=0-55us` (the `K_FOREVER` lock),
`bb=0` (busy-bus rejections), `bbok=38us` (busy check) - only ~63 us of a 15,765 us
transfer is inside the driver. **The rest is preemption, not blocking.**

The rank order is exact ChibiOS parity and is asserted in
`AP_HAL_Zephyr/Scheduler.h:174`: `"ChibiOS: main 180 > CAN 178 > rcin 177 >
I2C 176"`. ChibiOS survives this at ~32% load with 68% idle; at 100% we do not.

### 3.3 Motor output path is correct

`RCOUT` instrumentation records the pulse actually handed to `pwm_set()`, which
`SERVO_OUTPUT_RAW` cannot show (it reports `SRV_Channels` intent):

    RCOUT sf2 p1885,1950,1889,1931 rc0
    RCOUT sf2 p1684,1948,1950,1748 rc0

- `sf2` = `SAFETY_ARMED`, safety released, pulses not being zeroed
- pulses up to **1950 us** written to the FlexPWM
- `rc0` on every write - the Zephyr PWM driver accepted all of them
- values track RC throttle and carry per-motor attitude differential

Chain verified end to end: vehicle -> `SRV_Channels` -> HAL -> FlexPWM driver.

**IMPORTANT LIMIT ON THIS CLAIM.** `RCOUT` proves the HAL wrote the value and the
driver *accepted* it (`rc0`). It does **not** prove the electrical waveform on
every pin is correct. In the test of section 7, **two of four motors ran correctly
and two cycled/pulsed** - that is a 50% failure, and it is evidence that the pin
output is NOT correct on all channels. Two motors working is not a confirmation;
the two that failed are the data point.

Channel mapping for motors 1-4, and what has actually been verified:

| motor | FMU ch | FlexPWM node | instance / submodule | ever verified? |
|---|---|---|---|---|
| 1 | CH1 | `flexpwm1_pwm0` | 1 / SM0 | yes (2026-08-11 servo sweep) |
| 2 | CH2 | `flexpwm1_pwm1` | 1 / SM1 | **no** |
| 3 | CH3 | `flexpwm1_pwm2` | 1 / SM2 | **no** |
| 4 | CH4 | `flexpwm2_pwm0` | 2 / SM0 | yes (2026-08-11 servo sweep) |

The 2026-08-11 verification covered CH1/4/8/11 - i.e. **`pwm0` of each FlexPWM
instance, one per instance**. Submodules 1 and 2 of any instance have never been
verified on hardware. If the two misbehaving motors were 2 and 3, that is exactly
the unverified set and the fault is almost certainly per-submodule.

What the driver does do correctly (checked): `pwm_mcux.c` initialises each
submodule with its own `clockSource = kPWM_BusClock` and starts each submodule's
timer individually via `PWM_StartTimer(base, 1U << config->index)`. So the obvious
"submodules 1-3 never clocked" failure is not present.

What remains suspect: `pwm_mcux.c:183-191` is a **local modification** (CLDOK
cancel-pending-load, added because busy-waiting for LDOK cost ~350 us per 400 Hz
update cycle on this board). It changes reload timing on the repeated-update path,
which is the path every motor output takes in flight. It should be reviewed against
per-submodule behaviour before trusting any channel.

**Open question that decides this: which two motors ran correctly?** If 1 and 4,
the fault is per-submodule. If any other pair, the mapping theory is wrong.

---

## 4. Traps worth knowing

### 4.1 `Scheduler.h`'s hwdef priority override CANNOT work

`AP_HAL_Zephyr/Scheduler.h:121` documents overriding `APM_*_PRIORITY` from
`hwdef.dat`. On this HAL that **silently does nothing**: `Scheduler.h` includes
only `AP_HAL/Scheduler.h` and `AP_HAL/AP_HAL_Boards.h`, neither pulls in
`hwdef.h`, and the only force-includes are `autoconf.h` and `zephyr_stdint.h` -
there are no `-DAPM_*` flags. The comment is misleading and should be fixed.

### 4.2 `@PARAM/param.pck`'s advertised size is a deliberate over-estimate

`AP_Filesystem_Param.cpp:480`:

    // give size estimation to avoid needing to scan entire file
    stbuf->st_size = AP_Param::count_parameters() * 12;

`10920 = 910 x 12`. **EOF short of that is a COMPLETE file, not a truncated one.**
A fetch ending at 10,185 B was wrongly called incomplete because of this.

### 4.3 MAVFTP test harness traps

- FTP `seq` is uint16. A seq of 70000 throws in `struct.pack`.
- Another consumer on the MAVProxy fanout issues its own FTP traffic. A test client
  **must** filter replies on its own seq, or it reads someone else's ACK as its
  own. Observed: a `req_op=15` (BurstReadFile) reply to a request never sent.

### 4.4 Never call `GCS_SEND_TEXT()` from a low-priority thread

`GCS_SEND_TEXT` takes a semaphore. A thread getting 2-6 wakes/second holding a
lock main also wants is a priority inversion - and Zephyr's `k_mutex_unlock()`
restores the owner's *lock-time* priority, so a boost taken across a HAL semaphore
is undone at `give()`. Putting a diagnostic `GCS_SEND_TEXT` inside
`AP_Logger_File::io_timer()` (log_io, prio 10) is the likeliest cause of the loop
rate regression in 5.2.

### 4.5 `printk` diagnostics are invisible to a GCS

USB CDC is the console *and* the MAVLink port. Anything that must be observable
while a GCS is attached has to go out as a `STATUSTEXT` (50 char limit), not a
`printk`. Several diagnostics had to be re-routed for this reason.

---

## 5. Open defects

### 5.1 `AP_Logger: stuck thread ()` — starvation, not a blocked call

Recurs every ~32 s. `AP_Logger_File.cpp:167` fires when `_io_timer_heartbeat` is
>10 s stale (Zephyr/ESP32 timeout; 5 s elsewhere).

Instrumented `io_timer()` phases (`LOGDIAG gap/snl/iot/n`):

    LOGDIAG gap4579 snl0/0 iot314  n23
    LOGDIAG gap3957 snl0/0 iot1007 n25
    LOGDIAG gap581  snl0/0 iot581  n47

- `snl0/0` - **`start_new_log()` never called.** Not the log-open path.
- `n` = 23-58 per 10 s. `io_thread()` loops on `delay_microseconds(250..1000)`, so
  ~10000/10 s is expected. **200x too slow.**
- `iot` up to **1007 ms** for a function that returns immediately when disarmed
  (`_write_fd == -1`). That is wall time - the thread is preempted mid-call.

`log_io` is created `PRIORITY_IO, 1` -> `11 - 1 =` prio 10 in the Zephyr mapping
(ChibiOS 58+1=59; parity correct).

**Note a reasoning error to avoid repeating:** it is tempting to argue from the
code that this *cannot* be starvation, because `_io_timer_heartbeat = tnow` is the
second statement in `io_timer()` - so a stale heartbeat "must" mean the thread is
stuck inside a previous call. That is wrong. At 2-6 wakes/second a starved thread
produces >10 s gaps without being blocked in anything.

The empty `()` is `last_io_operation`, which every named operation (`write`,
`fsync`, `close`, `disk_space_avail`) sets before blocking. Blank means none of
them - consistent with starvation rather than a blocked filesystem call.

Fix direction: this is not a change *in* AP_Logger. The below-main band needs real
CPU, which on a 100%-busy machine means the yield (3.1) or reducing load.

### 5.2 Loop rate ~310 Hz, unexplained

Was 423-443 Hz; a later clean disarmed boot measures 307-336 Hz with **no new CPU
consumer**: `main3=53-55% SPI2=10-11% AP_timer2=11% AP_rcin6=10-12%` - main
unchanged. Boost count also fell from `b=1054-1360` to `b=276-400` per 10 s, i.e.
fewer `wait_for_sample()` cycles - fewer loops completed, not merely slower ones.

`SCHED_LOOP_RATE` is **not** the cause. A two-point ratio (600 -> 423-443, 0.74;
400 -> 277-310, 0.73) suggested the loop achieves ~73% of commanded, but restoring
600 gave **307-336 Hz, ratio 0.52** - the theory failed its own confirming test.

Leading candidate: the priority inversion in 4.4. A fix (moving the LOGDIAG
`GCS_SEND_TEXT` to the monitor thread) is built and flashed but **never measured**.
An `ACCT thr/idle/isr` statustext was added to separate idle from ISR time in the
missing ~25% of the top-3 CPU line; also unmeasured.

### 5.3 `MOT_SPIN_ARM = 0.07` gives 1070 us

Bench note records 0.20 as correct for these ESCs (they start ~1200 us). Measured
idle output while armed was exactly 1070 us. Not the cause of the no-spin symptom
(section 7 explains that), but wrong for this hardware.

### 5.4 Branch would fail CI

`Tools/scripts/check_branch_conventions.py --base-branch a3b190a800`:

- `ddc972d182 wip` - missing subsystem prefix (should be `AP_HAL_Zephyr:`)
- `7afea56c07 wip` - missing prefix (`AP_Compass:`)
- `9b1bf2d395 wip` - missing prefix (`docs:`)
- `0f3e8d41c1` - subject is a bare GitHub URL, so the checker reads the prefix as
  `pulled as expiment from https`, not in `allowed_subsystems.py`
- `14915bfbe387` - bumps `modules/zephyr` **and** modifies
  `libraries/GCS_MAVLink/GCS_FTP.cpp`/`.h`. Needs splitting; the submodule bump
  belongs alone (`625eefad5ff7` is a clean example in the same history).

Everything else passes: subject lengths, author emails, board IDs in range,
trailing newlines, markdown.

---

## 6. Hypotheses refuted — do not re-propose

| hypothesis | how it died |
|---|---|
| Sensors unhealthy from NAKs / unpowered rail | `nak=0` across all windows |
| I2C completion timeouts | `to=0`, no FAIL line ever emitted |
| `-EBUSY` retry storm | `oth=0` once the bucket was actually displayed |
| Sensor callbacks sleeping | BMM150/BMP388 `delay()` calls are init-only |
| `CONFIG_PM_DEVICE_RUNTIME` clock churn per transfer | not set in the build |
| XIP in the I2C/IMU path | 100% ITCM, verified by symbol address |
| eDMA channel collision | 23 distinct claims of 32 channels |
| Divide-by-zero in the burst throttle | `bw_in_bytes_per_second()` guarded 3 ways |
| Driver `K_FOREVER` lock contention | `lk=0-55us` |
| MAVFTP starved (as root cause) | it was the unbounded `push_reply()` hang |
| MAVFTP fails via UDP but works on serial | confounded by board state, not transport |
| `start_new_log()` blocking the logger | `snl0/0` - never called |
| `SCHED_LOOP_RATE` caused the loop regression | restoring 600 gave 0.52 ratio |
| `MOT_SPIN_ARM` caused no-spin | irrelevant at raised throttle |
| Spool-up block / `GROUND_IDLE` | outputs ramped to 1950 us with differential |
| CPU-load gate in `takeoff_check()` | our HAL never implements `get_system_load()` |
| ESC-telemetry gate | `TKOFF_RPM_MIN=0` -> returns true immediately |
| `MOT_PWM_TYPE` set to DShot | reads 0 (analogue PWM) |
| Only one channel per FlexPWM instance | all 12 mapped to distinct submodules |
| RC not reaching the FC | throttle reads 992 -> 1940 |
| Safety flag zeroing pulses | `sf2` = `SAFETY_ARMED` |
| PWM driver rejecting writes | `rc0` on every write |

---

## 7. Hardware failure during bench motor test

**What happened.** With a fresh battery installed, the vehicle was armed on the
bench and throttle raised, while a 170 s telemetry capture ran. Two of four motors
ran in sync and appeared healthy; the other two cycled/pulsed. After roughly 10 s
of that, a component failed - described as a capacitor letting go, with smoke.

**Cause, from the capture.** At least one output was pinned at **1950 us - full
throttle - continuously from ~62 s to ~160 s**:

    42.0s  *** ARMED ***
    52.7s  SERVO=(1923, 1950, 1923, 1942)
    65.7s  SERVO=(1444, 1858, 1548, 1950)
    89.6s  SERVO=(1798, 1950, 1943, 1914)
    134.0s SERVO=(1798, 1950, 1540, 1358)
    160.5s SERVO=(1811, 1950, ...

A stationary multirotor cannot respond to its own attitude corrections, so the
controller's integrators wind up against a frame that will not move and the
outputs saturate. That is why one channel went to maximum and stayed while the
others hunted. Motors at high throttle with no airflow and no load authority is
where ESCs desync; "cycling/pulsing" on two motors is the textbook description of
desync, and a desynced ESC draws very large current. The capacitor failure is the
normal end of that sequence.

**This was avoidable.** The saturated channel was visible in the live capture for
~100 s and was treated as data rather than a stop condition. A saturated output on
a stationary airframe must be a stop condition.

**Procedure for future bench motor tests:**

- props off, short bursts only
- stop immediately if any output saturates (>~1900 us) or holds high
- never hold a stationary multirotor armed at high throttle while collecting
  telemetry
- watch for per-motor divergence: it means the controller is integrating against
  a fixed frame

**Inspection before re-powering:**

- assume the failed ESC's FETs are shorted; do not energise the rail to test it -
  a shorted ESC can take the BEC and the flight controller with it
- inspect the other three ESCs for discoloured caps or scorching; desync stresses
  all of them
- check the motor that ran at full for ~100 s for burnt windings (smell, free
  rotation)
- inspect the PDB and the FC servo rail for collateral damage before trusting the
  board

---

## 8. Parameters as measured (via MAVFTP param.pck, 910 params)

    FRAME_CLASS      1        FRAME_TYPE       1        (Quad X)
    MOT_PWM_TYPE     0        (analogue PWM, not DShot)
    MOT_PWM_MIN      1000     MOT_PWM_MAX      2000
    MOT_SPIN_ARM     0.07     MOT_SPIN_MIN     0.25     MOT_THST_EXPO 0.65
    MOT_SAFE_DISARM  0
    SERVO1..4_FUNCTION 33/34/35/36   (Motor 1-4)
    SERVO_RATE       50       SERVO_DSHOT_RATE 0        SERVO_DSHOT_ESC 0
    TKOFF_RPM_MIN    0
    BRD_SAFETY_DEFLT 0        BRD_SAFETYOPTION 3
    SCHED_LOOP_RATE  600      FSTRATE_ENABLE   1        INS_GYRO_RATE 0
    EK3_ENABLE       1        AHRS_EKF_TYPE    3

Other verified state: IMU rate `RATELOOP gyro_hz=1011 measured_hz=1011`; EKF3
`EKF_STATUS flags=0x033f` (ATTITUDE, HORIZ_VEL, VERT_VEL, HORIZ_POS_REL,
HORIZ_POS_ABS, VERT_POS, both PRED bits; `CONST_POS_MODE` clear), variances
0.016/0.001/0.011; free heap ~205 KB (`freemem16` saturated at 65535,
`freemem32` hand-decoded from the raw payload).

Note: `Warning: Arming Checks Disabled` appears at boot. With `ARMING_CHECK` off,
the vehicle arms regardless of sensor or config problems - "it armed fine" has not
been evidence of health at any point.

---

## 9. Diagnostics currently in the tree

Must come out before a PR. Several are in shared upstream files.

| what | where | notes |
|---|---|---|
| I2C `ok/nak/to/oth/reset/xfer/max` counters | `AP_HAL_Zephyr/I2CDevice.cpp` | + STATUSTEXT in Scheduler.cpp |
| LPI2C phase split (`lk/bb/bbok`) | `modules/zephyr/.../i2c_mcux_lpi2c.c` | submodule |
| `BUSCB` per-callback timing | `AP_HAL_Zephyr/DeviceBus.cpp` | |
| `RCOUT sf/p1-4/rc` | `AP_HAL_Zephyr/RCOutput.cpp` | proves hardware writes |
| `CPU`/`ACCT` per-thread + idle/ISR | `AP_HAL_Zephyr/Scheduler.cpp` | |
| `LOGDIAG gap/snl/iot/n` | `AP_Logger/AP_Logger_File.cpp` | **shared upstream file** |
| `FTPDIAG`/`FTPS`/`FTP` counters | `GCS_MAVLink/GCS_FTP.cpp`/`.h` | **shared upstream file** |
| `UARTSTAT async=` flag | `AP_HAL_Zephyr/Scheduler.cpp` | |

`UARTSTAT` note for future readers: `queued/dma/done/fail` are **TX-only**
counters, so a port that never transmits shows zeros and that says nothing about
whether RX is on DMA. `rx`/`rxev` come only from `_async_cb`'s `UART_RX_RDY`, so
non-zero `rx` *proves* the async/DMA path. s2 (LPUART8) does use DMA; one byte per
event is the 1 ms idle flush firing between genuinely isolated arrivals (~450-580
events/s, and isolated bytes cap at 1000/s with a 1 ms timeout).
