# ArduPilot on Zephyr RTOS

AP_HAL_Zephyr runs ArduPilot on top of the [Zephyr RTOS](https://www.zephyrproject.org/).
Instead of driving STM32 peripherals directly the way AP_HAL_ChibiOS does, it
sits on Zephyr's device drivers, so any SoC with decent Zephyr support becomes
a candidate ArduPilot target.

Bringing up a new board is then mostly devicetree and Kconfig work rather than
writing peripheral drivers from scratch. ArduPilot already reaches Cortex-M7
through AP_HAL_ChibiOS and Xtensa through AP_HAL_ESP32, so neither is new
ground on its own; what is different here is that one HAL covers both families
and a host build from one set of sources, because the per-peripheral driver
work belongs to Zephyr.

If you already know AP_HAL_ChibiOS, start with [Zephyr compared to ChibiOS](https://ardupilot.org/dev/docs/zephyr-vs-chibios.html).

## Boards

| waf board          | SoC                | Architecture | Zephyr board target             |
| ------------------ | ------------------ | ------------ | ------------------------------- |
| `mr_vmu_rt1176`    | NXP i.MX RT1176    | Cortex-M7    | `mr_vmu_rt1176/mimxrt1176/cm7`  |
| `CubeOrangeZephyr` | ST STM32H743       | Cortex-M7    | `cube_orange_zephyr`            |
| `ESP32S3Zephyr`    | Espressif ESP32-S3 | Xtensa LX7   | `esp32s3_zephyr/esp32s3/procpu` |
| `native_sim`       | host               | x86-64 Linux | `native_sim/native/64`          |

CubeOrangeZephyr is the same hardware as the ChibiOS CubeOrange target, on
purpose. Flashing both to one board is the only honest way to compare the two
backends, and it's where most of the numbers in COMPARED_TO_CHIBIOS.md came
from.

native_sim builds the lot as a Linux executable. It runs the real HAL code
path with no hardware attached, which makes it the cheapest way to check we
haven't broken boot, storage or the main loop. It is not SITL - SITL replaces
the HAL, native_sim exercises it.

## Documentation

The Zephyr HAL is documented in the ArduPilot wiki. Start at
[Zephyr Based Autopilots](https://ardupilot.org/dev/docs/zephyr-autopilots.html),
which indexes every page below.

| Page | What is there |
| ---- | ------------- |
| [Building for Zephyr](https://ardupilot.org/dev/docs/zephyr-building.html) | Getting the prerequisites and the SDK, the waf board targets, and how the Kconfig fragments and devicetree overlays are layered. |
| [Loading firmware onto Zephyr boards](https://ardupilot.org/ardupilot/docs/common-loading-firmware-onto-zephyr-boards.html) | Loading a build over USB or SWD, and replacing the bootloader. |
| [Architecture and porting notes](https://ardupilot.org/dev/docs/zephyr-hal-architecture.html) | How the HAL is put together and the rules a new board port has to follow. |
| [Zephyr compared to ChibiOS](https://ardupilot.org/dev/docs/zephyr-vs-chibios.html) | What differs for a ChibiOS developer, and which board does what today. |
| [Feature parity detail](https://ardupilot.org/dev/docs/zephyr-hal-parity.html) | The per-subsystem audit against AP_HAL_ChibiOS, one row per HAL call. |
| [Debugging](https://ardupilot.org/dev/docs/zephyr-debugging.html) | SWD and GDB, the readable globals, the fault record, and Renode emulation. |
| [Development process](https://ardupilot.org/dev/docs/zephyr-development-process.html) | How a change is verified before it is committed. |
| [Bootloader and security](https://ardupilot.org/dev/docs/zephyr-bootloader-security.html) | How the bootloader is built and signed, and what the update path protects. |
| [Running on native_sim](https://ardupilot.org/dev/docs/zephyr-native-sim.html) | Running the real HAL as a Linux process. |
| [MR-VMU-RT1176 bring-up](https://ardupilot.org/dev/docs/zephyr-rt1176-bringup.html) | Memory layout, configuration, status and the bring-up checklist for that board. |

The `.md` files beside this one are stubs that each point at their wiki page.

## What lives where

| Path              | Contents                                                       |
| ----------------- | -------------------------------------------------------------- |
| `*.cpp`, `*.h`    | the HAL itself                                                  |
| `hwdef/<board>/`  | sensors, buses and serial config per board                      |
| `zephyr/`         | the Zephyr application: CMake, Kconfig, `prj*.conf`             |
| `zephyr/src/`     | Zephyr-side C glue and SoC fixups                               |
| `zephyr/boards/`  | devicetree, pinctrl and defconfigs for boards not upstream in Zephyr |
| `Tools/renode/`   | emulator board platforms, the C# peripheral models these boards need, and the flight harness CI runs them under |

The HAL class is `HAL_Zephyr`, namespace `Zephyr::`. File layout and naming
track AP_HAL_ChibiOS deliberately closely so you can diff the two when
something behaves differently.

`Tools/renode/` matters more here than on a ChibiOS board: CubeOrangeZephyr
and mr_vmu_rt1176 both boot under Renode, and
`.github/workflows/test_renode_zephyr.yml` flies a copter mission on each of
them, alongside a ChibiOS reference flight. Three flights cost about 100
runner-minutes, so both its push and its pull-request triggers are
path-filtered to `libraries/AP_HAL_Zephyr/` and the workflow itself; a change
anywhere else - the waf backend, the Renode harness, a shared library - runs
it on demand through `workflow_dispatch`. A failed flight or boot check fails the job: every flight step ends
in `exit $rc` with an `::error` annotation naming the verdict, so a green
job means the missions passed, not merely that they ran. The step summary
carries the verdict line and the flight GIF. See
[Tools/renode/README.md](../../Tools/renode/README.md).

Per-board wiring and connector detail is in `hwdef/<board>/README.md` where
there is one - today that is `mr_vmu_rt1176` and `native_sim` only.
Debugging, tooling and crash dumps are in [DEBUGGING.md](DEBUGGING.md).
