# RP2350 ChibiOS Full-SMP Bring-up: Status & Progress Report

**Date:** 2026-03-20
**Branch:** `buzz-rp2350-chibios-v2`
**Target hardware:** Raspberry Pi Pico2 (RP2350, dual Cortex-M33)

---

## 1. Goal

Enable ChibiOS full-SMP (`CH_CFG_SMP_MODE=TRUE`, `RP_CORE1_START=TRUE`) on the RP2350 so that the ArduPilot "rate" (main flight-control) thread can be pinned to core1 via `thread_create_alloc_on_core()`, leaving core0 free for I/O and peripheral handling.

---

## 2. Architecture Overview

### ChibiOS SMP Model

ChibiOS uses a **symmetric multi-processing** model where each core runs its own OS *instance*:

- **Core0:** `ch0` (`os_instance_t`) — initialized by `chSysInit()`, handles IRQs, USB, MAVLink, etc.
- **Core1:** `ch1` (`os_instance_t`) — initialized by `chInstanceObjectInit(&ch1, &ch_core1_cfg)` from `c1_main()`.

The global `ch_system` structure at `0x20018eb0` tracks both instances.

### Inter-core synchronization

ChibiOS SMP uses **SIO spinlock 31** (`0xd000017c`) as the kernel lock:
- `port_lock()` / `chSysLock()` → acquire spinlock 31 + set `BASEPRI=16`
- `port_unlock()` / `chSysUnlock()` → release spinlock 31 + set `BASEPRI=0`

BASEPRI=16 blocks ARM NVIC priorities ≥ 16 (numerically). Priority 2 (timer IRQs) can still fire through BASEPRI=16.

### Core1 Boot Sequence

```
RP2350 boot ROM  →  multicore_launch_core1()  →  _crt0_c1_entry
    ↓
Sets: MSP=0x20002800, MSPLIM=0x20002200, PSP=0x20004400, PSPLIM=0x20002800
Sets: CONTROL=2 (use PSP), FPCCR=0xC0000000 (lazy FP), VTOR=0x10010080
Fills: MSP stack (0x20002200–0x20002800) and PSP stack (0x20002800–0x20004400) with 0x55555555
Calls: __c1_cpu_init() → c1_boot_stage=1
Calls: __c1_late_init() → c1_boot_stage=2
Calls: c1_main()
    ↓
c1_main():
  c1_boot_stage = 3
  chSysWaitSystemState(ch_sys_running)  ← busy-wait for core0 to finish chSysInit()
  c1_boot_stage = 4
  chInstanceObjectInit(&ch1, &ch_core1_cfg)  ← CRASHES HERE
  c1_boot_stage = 5  (never reached)
```

### Key Memory Addresses

| Symbol | Address | Description |
|--------|---------|-------------|
| `c1_boot_stage` | `0x20005410` | Core1 top-level boot canary |
| `c1_inst_stage` | `0x2000540c` | Sub-stage canary inside chInstanceObjectInit |
| `ch1` | `0x20018a00` | Core1 os_instance_t |
| `ch_system` | `0x20018eb0` | Global ChibiOS system structure |
| `ch1.vtlist` | `0x20018a10` | Virtual timer list (ch1 + 16) |

---

## 3. Canary Instrumentation Added

Two layers of canary writes have been added to precisely locate crashes:

### `c1_boot_stage` (in `c1_main.c`)
```
0 = never reached __c1_cpu_init (ROM crash during CRT0)
1 = __c1_cpu_init ran (VTOR/FPU/stacks set up)
2 = __c1_late_init ran (stacks filled, about to call c1_main)
3 = c1_main entered
4 = chSysWaitSystemState returned (ch_sys_running seen)
5 = chInstanceObjectInit completed  ← NEVER REACHED
6 = idle loop entered
```

### `c1_inst_stage` (in `chinstances.c` and `chcore.c`)
```
0x4A = just before bl port_init
0x4F = first instruction of port_init (before port_suspend)
0x50 = after port_suspend (BASEPRI=16 set, cpsie i done)
0x51 = after FPU init
0x52 = after NVIC_SetPriorityGrouping (AIRCR write)
0x53 = after CoreDebug/DWT enable
0x54 = after port_smp_init (timer+SIO FIFO IRQs armed)
0x55 = after SVCall/PendSV priority set
0x56 = after MPU regions programmed
0x57 = after mpuEnable (port_init about to return)
0x41 = after port_init returned
0x42 = after ch_pqueue_init returned ← LAST VALUE SEEN
0x43 = after __vt_object_init returned ← NEVER REACHED
```

**Observed state after hardware reset + 5 seconds:**
- `c1_boot_stage = 4` (chInstanceObjectInit entered but not completed)
- `c1_inst_stage = 0x42` (stuck after ch_pqueue_init, before __vt_object_init returns)
- Core1 PC = `0xDA` (RP2350 boot ROM — core1 has reset to ROM state)
- Core1 MSP = `0xF0000000` (ROM's initial MSP after core1 reset)

---

## 4. What Has Been Confirmed Working

1. **port_init completes successfully**: All canaries 0x4F, 0x50–0x57, 0x41 are written. BASEPRI=16 is set, FPU initialized, NVIC priority grouping set, DWT enabled, SMP init (timer + SIO FIFO IRQs), SVCall/PendSV priorities, MPU regions, MPU enabled.

2. **ch_pqueue_init completes**: Stage 0x42 is written, meaning the ready-list priority queue is initialized correctly.

3. **port_init details (all verified via objdump):**
   - BASEPRI set to 16
   - `cpsie i` — PRIMASK cleared (interrupts unmasked)
   - FPU: FPCCR = 0xC0000000 (ASPEN+LSPEN), CONTROL = 2 (PSP, lazy FP)
   - NVIC priority grouping: AIRCR written correctly
   - DWT: CoreDebug->DEMCR bit TRCENA set, DWT->CTRL CYCCNTENA set
   - `__port_smp_init`: stBindAlarmN(1) → nvicEnableVector(IRQ1, priority=2)
   - `__port_smp_init`: SIO->FIFO_ST error flags cleared, IRQ25 priority = 0xF0, NVIC_ISER bit 25 enabled
   - SVCall priority = 0 (highest), PendSV priority = 0xF0 (lowest)
   - MPU MAIR0 = 0x0444bbff, MAIR1 = 0
   - MPU regions 0–7: all RLAR bit 0 = 0 → all regions **disabled**
   - MPU_CTRL = 5 (ENABLE | PRIVDEFENA) — background memory map active

4. **Stack configuration is valid:**
   - Core1 PSP: 0x20004400 → ~0x200043B0 inside __vt_object_init (well above PSPLIM = 0x20002800)
   - Core1 MSP: 0x20002800, MSPLIM = 0x20002200 (exception handler stack = 1536 bytes)

5. **Interrupt state analysis:**
   - TIMER0->INTE = 0x00000001 (only alarm 0 = core0's tick; alarm 1 NOT enabled)
   - TIMER0_IRQ_1 (IRQ1, priority 2) is enabled in NVIC but has NO hardware trigger
   - IRQ25 (SIO FIFO) is enabled in NVIC at priority 0xF0 = blocked by BASEPRI=16

---

## 5. The Crash: Deep Analysis

### Crash Location
The crash occurs **inside `__vt_object_init`**, which initializes core1's virtual timer list (`ch1.vtlist` at `0x20018a10`).

### `__vt_object_init` Disassembly (0x1014eff0)
```asm
1014eff0:  push  {r3, r4, r5, lr}          ; SP -= 16
1014eff2:  mov   r4, r0                    ; r4 = vtlist = 0x20018a10
1014eff4:  str   r0, [r0, #0]              ; vtlist->next = self
1014eff6:  str   r0, [r0, #4]              ; vtlist->prev = self
1014eff8:  mov.w r3, #0xffffffff
1014effc:  str   r3, [r0, #8]              ; vtlist->delta = -1
1014effe:  movs  r5, #0
1014f000:  str   r5, [r0, #12]             ; vtlist->lasttime = 0
1014f002:  movs  r3, #10
1014f004:  str   r3, [r0, #16]             ; vtlist->lastdelta = 10 (CH_CFG_ST_TIMEDELTA)
1014f006:  bl    1014b354 <stGetCounter>   ; read TIMER0->TIMERAWL (0x400b0028)
1014f00a:  mov   r2, r0                    ; r2 = timer value
1014f00c:  mov   r3, r5                    ; r3 = 0
1014f00e:  strd  r2, r3, [r4, #24]         ; vtlist->laststamp = {timer, 0}
1014f012:  pop   {r3, r4, r5, pc}          ; return
```

`stGetCounter` (0x1014b354) is a trivial leaf function:
```asm
1014b354:  ldr   r3, [pc, #4]   ; r3 = 0x400b0000 (TIMER0 base)
1014b356:  ldr   r0, [r3, #40]  ; r0 = TIMER0->TIMERAWL (0x400b0028)
1014b358:  bx    lr
```

### Memory Accesses in `__vt_object_init`
All stores are to `ch1.vtlist` = `0x20018a10`–`0x20018a2b`, which is in BSS (zeroed by core0 at startup). All addresses are valid RP2350 SRAM. TIMER0 at `0x400b0000` is a standard peripheral accessible from both cores.

### What Has Been Ruled Out

| Hypothesis | Status | Reason |
|------------|--------|--------|
| MPU fault | **RULED OUT** | All 8 MPU regions disabled (RLAR bit 0 = 0). PRIVDEFENA=1 allows full background access. |
| Stack overflow / PSPLIM | **RULED OUT** | PSP ≈ 0x200043B0 at crash, PSPLIM = 0x20002800. Gap = 6KB. |
| TIMER0_IRQ_1 firing (hardware) | **RULED OUT** | TIMER0->INTE bit 1 = 0. Hardware cannot assert IRQ1. |
| IRQ25 (SIO FIFO) firing during init | **RULED OUT** | Priority 0xF0 = 240 ≥ 16, blocked by BASEPRI=16. |
| Unaligned `strd` access | **RULED OUT** | 0x20018a28 is 8-byte aligned (0x28 % 8 = 0). |
| Wrong vtlist address | **RULED OUT** | r0 = ch1 + 16 = 0x20018a00 + 16 = 0x20018a10, confirmed from nm and disassembly. |
| Invalid return address in `pop {pc}` | **LOW PROBABILITY** | No stack corruption path identified between push and pop (only one leaf call). |
| Core1 BSS not initialized | **RULED OUT** | Core0's `_crt0_entry` zeros BSS including ch1 at 0x20018a00. |

### Key Findings from Full Disassembly

#### `port_init` does `cpsie i` after setting BASEPRI
```asm
1014e062:  msr   BASEPRI, r3   ; BASEPRI = 16
1014e066:  cpsie i             ; ← PRIMASK = 0 (INTERRUPTS UNMASKED)
```
The CRT0 sets PRIMASK=1 (`cpsid i`). `port_init` then sets BASEPRI=16 and explicitly clears PRIMASK via `cpsie i`. From this point on, any IRQ with priority < 16 **can** fire. With TIMER0_IRQ_1 at priority 2, it is not blocked by BASEPRI=16 — but requires hardware assertion (TIMER0->INTS bit 1 = 1), which cannot happen while TIMER0->INTE bit 1 = 0.

#### `__port_smp_init` enables IRQ1 at priority 2
```asm
stBindAlarmN(core_id=1)
  → st_lld_bind_alarm_n(1)
    → nvicEnableVector(IRQ=1, priority=2)
      1. NVIC_IP[1] = 0x20    (priority 2 in 4-bit left-shifted format)
      2. NVIC_ICPR[0] bit 1   (clear pending)
      3. NVIC_ISER[0] bit 1   (enable)
```
IRQ1 pending is explicitly cleared before enabling. IRQ1 is enabled in core1's NVIC at priority 2, but hardware assertion requires TIMER0->INTE bit 1 = 1.

#### `st_lld_start_alarm_n` enables TIMER0->INTE
When the first virtual timer is inserted on core1 (via `vt_insert_first`), `st_lld_start_alarm_n` runs:
```asm
str r1, [r2, r3, lsl #2]  ; TIMER0->ALARM[alarm_n] = value  (arms alarm)
str r3, [r2, #60]          ; TIMER0->INTR = (1<<alarm_n)     (clear flag)
str r3, [r1, #64]          ; TIMER0->INTE SET = (1<<alarm_n) (ENABLE alarm IRQ!)
```
**This is not called during chInstanceObjectInit.** It only fires when the first VT is inserted on core1, which happens after init is complete.

#### Vector table (VTOR = 0x10010080)
Both cores share VTOR = 0x10010080. The first 128 bytes (0x10010000–0x1001007f) are the RP2350 image header. The vector table starts at 0x10010080:
- IRQ0 (TIMER0_IRQ_0, Vector40): 0x1014e1d4
- IRQ1 (TIMER0_IRQ_1, Vector44): 0x1014e21c
- IRQ25 (SIO FIFO, VectorA4): 0x1014e14c

The SIO FIFO handler (VectorA4) reads SIO->FIFO_ST and calls `port_local_halt` if FIFO is empty — this is the intercore reschedule handler.

### Remaining Hypothesis

The crash has not yet been pinpointed to a specific instruction inside `__vt_object_init`. The next diagnostic step is to add sub-stage canary writes **inside** `__vt_object_init` (by inlining the initialization in `chinstances.c` with intermediate canary writes) to determine exactly which of the 7 memory operations fails:

1. `str r0, [r0, #0]` — vtlist->next = self
2. `str r0, [r0, #4]` — vtlist->prev = self
3. `str r3, [r0, #8]` — vtlist->delta = -1
4. `str r5, [r0, #12]` — vtlist->lasttime = 0
5. `str r3, [r0, #16]` — vtlist->lastdelta = 10
6. `bl stGetCounter` — call into leaf function
7. `strd r2, r3, [r4, #24]` — vtlist->laststamp = {timer, 0}
8. `pop {r3, r4, r5, pc}` — return (might expose stack corruption)

A plausible remaining hypothesis: **there is a subtle hardware issue or race on the RP2350 that causes core1 to be reset** when certain memory regions are written to for the first time after the MPU is enabled. The MPU is enabled just before `port_init` returns (stage 0x57), and `__vt_object_init` is the first "normal" SRAM write after that. Even though all MPU regions are "disabled" (RLAR bit 0 = 0), it is possible that:

- The MPU MAIR attributes (0x0444bbff for MAIR0) combined with PRIVDEFENA interact unexpectedly on this chip revision.
- OR: there is a silicon-level erratum on early RP2350 silicon that affects certain SRAM regions.
- OR: the SIO FIFO has leftover data from the core1 launch sequence, and when IRQ25 is enabled, a stale pending signal causes issues that manifest later.
- OR: the `cpsie i` in `port_init` and the subsequent `strd` (double-word store) interact with an ARM Cortex-M33 erratum or RP2350 memory-ordering issue.

---

## 6. Files Modified

| File | Change |
|------|--------|
| `modules/ChibiOS/os/rt/src/chinstances.c` | Added `c1_inst_stage` canaries (0x40–0x48, 0x4A) |
| `modules/ChibiOS/os/rt/src/chsys.c` | Added `ch_core1_cfg`, `ch1` instance, SMP init call |
| `modules/ChibiOS/os/common/ports/ARMv8-M-ML-ALT/chcore.c` | Added `c1_inst_stage` canaries in `port_init` (0x4F–0x57) |
| `libraries/AP_HAL_ChibiOS/hwdef/Pico2/c1_main.c` | Core1 entry point: `c1_boot_stage`, `__c1_cpu_init`, `__c1_late_init`, `c1_main` |
| `libraries/AP_HAL_ChibiOS/hwdef/Pico2/hwdef.dat` | Enabled `CH_CFG_SMP_MODE`, `RP_CORE1_START`, include c1_main.c |
| `libraries/AP_HAL_ChibiOS/Scheduler.cpp` | Thread creation with core affinity (WIP) |

---

## 7. Build & Debug Commands

```bash
# Build
rm -f build/Pico2/modules/ChibiOS/libch.a
./waf build --target=bin/arducopter --board=Pico2

# Flash
openocd -f interface/cmsis-dap.cfg -f target/rp2350.cfg \
  -c "program build/Pico2/bin/arducopter.elf verify reset exit"

# Attach GDB (two terminals):
# Terminal 1:
openocd -f interface/cmsis-dap.cfg -f target/rp2350.cfg \
  -c "bindto 127.0.0.1"
# Terminal 2:
arm-none-eabi-gdb build/Pico2/bin/arducopter
  target remote :3333
  monitor reset halt
  # Read canaries:
  x/1xw 0x20005410   # c1_boot_stage
  x/1xw 0x2000540c   # c1_inst_stage
  # Read timer/interrupt state:
  x/1xw 0x400b003c   # TIMER0->INTR
  x/1xw 0x400b0040   # TIMER0->INTE
  x/4xw 0xe000e200   # NVIC_ISPR
  # MPU state:
  x/1xw 0xe000ed94   # MPU_CTRL
  x/1xw 0xe000ed28   # SCB->CFSR (fault status)
  x/1xw 0xe000ed2c   # SCB->HFSR (hard fault status)
  x/1xw 0xe000ed34   # SCB->MMFAR (MemManage fault addr)
```

### Diagnostic Address Map

| Address | Register | Purpose |
|---------|----------|---------|
| `0x20005410` | `c1_boot_stage` | Core1 top-level progress (0–6) |
| `0x2000540c` | `c1_inst_stage` | Init sub-stage (0x40–0x57) |
| `0x400b003c` | TIMER0->INTR | Raw interrupt status (alarms 0–3) |
| `0x400b0040` | TIMER0->INTE | Interrupt enable (alarms 0–3) |
| `0xe000ed28` | SCB->CFSR | Configurable Fault Status |
| `0xe000ed2c` | SCB->HFSR | Hard Fault Status |
| `0xe000ed34` | SCB->MMFAR | MemManage Fault Address |
| `0xe000e200` | NVIC_ISPR[0] | Active/pending IRQs |
| `0xe000ed94` | MPU->CTRL | MPU enabled/PRIVDEFENA status |
| `0xd000017c` | SIO->SPINLOCK[31] | Kernel spinlock state |

---

## 8. Next Steps

### Immediate (Active Debug)

1. **Add sub-stage canaries inside `__vt_object_init`**: Inline the vtlist initialization in `chinstances.c` with intermediate `c1_inst_stage` writes (0x42a, 0x42b, 0x42c, ...) to pinpoint the exact faulting instruction.

2. **Read SCB->CFSR and HFSR on core1**: After the crash, use OpenOCD's `targets rp2350.dap.core1` to read the fault status registers. This will tell us the fault type (MemManage, BusFault, UsageFault, HardFault) and the faulting address.

3. **Try disabling MPU**: Comment out `mpuEnable()` in `port_init` temporarily. If the crash disappears, the MPU MAIR configuration is the culprit even though all regions are disabled.

4. **Check for SIO FIFO stale data**: Drain the SIO RX FIFO completely in `__port_smp_init` before enabling IRQ25, not just clearing error flags.

### Short Term

5. **Rate thread pinning**: Once core1 is stable, use `thread_create_alloc_on_core()` in `Scheduler.cpp` to pin the ArduPilot rate thread to core1.

6. **MAVLink over USB verification**: Confirm telemetry/MAVLink continues to work under SMP (core0 handles USB).

7. **SMP stress test**: Run with deliberate inter-core load to verify spinlock and scheduler correctness.

### Longer Term

8. **WFE/WFI idle optimization**: Replace the busy `b.n` loop in c1_main with `port_wait_for_interrupt()` to reduce power consumption on core1's idle thread.

9. **Performance measurement**: Measure latency improvement from pinning the rate loop to a dedicated core.

---

## 9. Summary Assessment

The SMP infrastructure is substantially correct — core1 successfully boots, synchronizes with core0, enters `chInstanceObjectInit`, completes `port_init` (including MPU, FPU, NVIC, SMP timer binding), and initializes the ready-list queue. The crash is isolated to a single small function (`__vt_object_init`, 35 bytes, 8 memory operations) that initializes the virtual timer delta-list.

The crash results in core1 resetting to the RP2350 boot ROM — indicating an unrecoverable exception (most likely HardFault or MemManage fault escalation). The root cause has not yet been identified from static analysis alone. Hardware diagnostic readout of the Configurable Fault Status Register (CFSR) on core1 immediately after the fault is the critical next step.
