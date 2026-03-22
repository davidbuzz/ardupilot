# RP2350 Dual-Core Bring-up: Status & Progress Report

**Date:** 2026-03-22 (updated)
**Branch:** `buzz-rp2350-chibios-v2`
**Target hardware:** Raspberry Pi Pico2 (RP2350, dual Cortex-M33)

---

## 1. Goal (Revised)

Run the ArduPilot rate (flight-control) controller on RP2350 core1, leaving core0 free for I/O, EKF, GCS and peripheral handling. The original approach used ChibiOS full-SMP (`CH_CFG_SMP_MODE=TRUE`); after extensive debugging (see Section 5) this has been **abandoned in favour of a bare-metal core1 dispatcher** (see Section 11).

---

## 2. Architecture Overview

### ChibiOS SMP Model

ChibiOS SMP (`CH_CFG_SMP_MODE=TRUE`) is an **AMP-like model** — each core has its own independent scheduler instance with **hard thread affinity** (threads are created on a specific core and never migrate). This is NOT symmetric load-balancing SMP as in Linux. Each core's instance maintains its own ready-list and virtual timer list, and inter-core communication is limited to wakeup signals via SIO FIFO.

- **Core0:** `ch0` (`os_instance_t`) — initialized by `chSysInit()`, handles IRQs, USB, MAVLink, etc.
- **Core1:** `ch1` (`os_instance_t`) — initialized by `chInstanceObjectInit(&ch1, &ch_core1_cfg)` from `c1_main()`.

The global `ch_system` structure at `0x20015150` (current build) coordinates instance state and the global thread registry.

### Inter-core synchronization

ChibiOS SMP uses **SIO spinlock 31** (`0xd0000100 + 31*4 = 0xd000017c`) as the kernel lock:
- `port_lock()` / `chSysLock()` → acquire spinlock 31 + set `BASEPRI=16`
- `port_unlock()` / `chSysUnlock()` → release spinlock 31 + set `BASEPRI=0`

BASEPRI=16 blocks ARM NVIC priorities ≥ 16 (numerically). Priority 2 (timer IRQs) can still fire through BASEPRI=16.

### Core1 Boot Sequence

```
RP2350 boot ROM  →  start_core1() ROM handshake  →  _crt0_c1_entry (0x1001136c)
    ↓
Sets: MSP=__c1_main_stack_end__=0x20002800, MSPLIM=__c1_main_stack_base__=0x20002200
Sets: PSP=__c1_process_stack_end__=0x20004400, PSPLIM=__c1_process_stack_base__=0x20002800
Sets: CONTROL=2 (Thread uses PSP), FPCCR=0xC0000000 (ASPEN+LSPEN), VTOR=0x10010080
Fills: MSP stack (0x20002200–0x20002800) and PSP stack (0x20002800–0x20004400) with 0x55555555
Calls: __c1_cpu_init() → c1_boot_stage=1  (0x20004b4c)
Calls: __c1_late_init() → c1_boot_stage=2
Calls: c1_main()
    ↓
c1_main():
  c1_boot_stage = 3
  chSysWaitSystemState(ch_sys_running)  ← busy-wait for core0 to finish chSysInit()
  c1_boot_stage = 4
  chInstanceObjectInit(&ch1, &ch_core1_cfg)  ← CRASHES HERE (per older debug session)
  c1_boot_stage = 0x4A  (never reached in crash state)
```

### Key Memory Addresses (current build, 2026-03-21)

| Symbol | Address | Description |
|--------|---------|-------------|
| `c1_fifo_last_msg` | `0x20004b40` | Last SIO FIFO message received by core1 ISR |
| `c1_inst_stage` | `0x20004b48` | Sub-stage canary inside chInstanceObjectInit |
| `c1_boot_stage` | `0x20004b4c` | Core1 top-level boot canary |
| `ch1` | `0x20014ca0` | Core1 os_instance_t (size 0x1b0) |
| `ch0` | `0x20014fa0` | Core0 os_instance_t (size 0x1b0) |
| `ch_system` | `0x20015150` | Global ChibiOS system structure |
| `c1_fifo_msg_count` | `0x20015aa0` | Count of SIO FIFO messages received by core1 ISR |
| `c1_fifo_isr_stage` | `0x20015aa4` | Stage canary inside VectorA4 (FIFO ISR) |
| `ch1.vtlist` | `0x20014cb0` | Virtual timer list (ch1 + 16) |

**Note:** These addresses changed significantly from the prior debug session. Always confirm from the current `Linker.map`.

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

**Observed state (older debug session — addresses below are from a prior build):**
- `c1_boot_stage = 4` (chInstanceObjectInit entered but not completed)
- `c1_inst_stage = 0x42` (stuck after ch_pqueue_init, before __vt_object_init returns)
- Core1 PC = `0xDA` (RP2350 boot ROM — core1 has reset to ROM state)
- Core1 MSP = `0xF0000000` (ROM's initial MSP after core1 reset)

**Observed state (latest debug session, freshly flashed binary, ~600ms after boot):**
- Core0 is in lockup: PC = `0xeffffffe`, xPSR = `0x28000003` (IPSR=3 = HardFault active)
- Core0 MSP = `0x4c12f228` — **massively corrupted** (not in SRAM range 0x20000000–0x20080000)
- Core1 is at ROM (`0xEC`) — either never started yet, or has already crashed and reset
- OpenOCD reports "clearing lockup after double fault" — indicates a second fault inside the HardFault handler
- The double fault is consistent with HardFault_Handler doing `memcpy(&ctx, __get_PSP(), ...)`: if PSP is invalid when the first fault fires, the memcpy causes a second fault → lockup
- **Root cause not yet identified** — a hardware breakpoint debug session targeting HardFault_Handler is the next step (see Section 7)

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

### Crash Location (older debug session, prior build)
The crash was observed **inside `__vt_object_init`**, which initializes core1's virtual timer list. In the current build `ch1.vtlist` is at `0x20014cb0` (ch1 = 0x20014ca0, vtlist offset = 16).

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
- IRQ0 (TIMER0_IRQ_0, Vector40): 0x100eefcc (thumb: 0x100eefcd)
- IRQ1 (TIMER0_IRQ_1, Vector44): 0x100ef014 (thumb: 0x100ef015)
- IRQ25 (SIO FIFO, VectorA4): 0x100eeef8 (thumb: 0x100eeef9)
- HardFault: 0x100a77f0 (thumb: 0x100a77f1, at flash offset [VTOR+0xC] = 0x1001008C)

The SIO FIFO handler (VectorA4) reads SIO->FIFO_ST and calls `port_local_halt` on PANIC — this is the intercore reschedule handler.

**Note:** All vector addresses changed compared to the prior build because the binary layout is different.

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

### Diagnostic Address Map (current build, 2026-03-21)

| Address | Symbol/Register | Purpose |
|---------|----------|---------|
| `0x20004b40` | `c1_fifo_last_msg` | Last SIO FIFO message received by core1 ISR |
| `0x20004b48` | `c1_inst_stage` | chInstanceObjectInit sub-stage (0x40–0x57) |
| `0x20004b4c` | `c1_boot_stage` | Core1 top-level progress (0–6, 0x4A–0x4D) |
| `0x20014ca0` | `ch1` | Core1 os_instance_t |
| `0x20014cb0` | `ch1.vtlist` | Core1 virtual timer delta-list |
| `0x20014fa0` | `ch0` | Core0 os_instance_t |
| `0x20015150` | `ch_system` | Global ch_system (state, instances[]) |
| `0x20015aa0` | `c1_fifo_msg_count` | Count of FIFO messages received by core1 ISR |
| `0x20015aa4` | `c1_fifo_isr_stage` | Sub-stage inside VectorA4 (FIFO ISR) |
| `0x400b003c` | TIMER0->INTR | Raw interrupt status (alarms 0–3) |
| `0x400b0040` | TIMER0->INTE | Interrupt enable (alarms 0–3) |
| `0xe000ed28` | SCB->CFSR | Configurable Fault Status |
| `0xe000ed2c` | SCB->HFSR | Hard Fault Status |
| `0xe000ed34` | SCB->MMFAR | MemManage Fault Address |
| `0xe000ed38` | SCB->BFAR | Bus Fault Address |
| `0xe000e200` | NVIC_ISPR[0] | Pending IRQs |
| `0xe000ed94` | MPU->CTRL | MPU enabled/PRIVDEFENA status |
| `0xd000017c` | SIO->SPINLOCK[31] | Kernel spinlock state |

### Key Function Addresses (current build)

| Address | Symbol | Notes |
|---------|--------|-------|
| `0x100a77f0` | `HardFault_Handler` | Sets hw bp here to catch first fault |
| `0x100ed5c8` | `chSysHalt` | Called on OS assertions |
| `0x100eee04` | `port_init` | Per-core low-level init |
| `0x100eeecc` | `port_local_halt` | Called on PANIC SIO FIFO message |
| `0x100eeef8` | `VectorA4` | SIO FIFO IRQ25 handler (inter-core reschedule) |
| `0x100eef70` | `__port_smp_init` | Arms timer + SIO FIFO IRQs on this core |
| `0x100eefcc` | `Vector40` | TIMER0_IRQ_0 (core0 tick) |
| `0x100ef014` | `Vector44` | TIMER0_IRQ_1 (core1 tick) |
| `0x100efde8` | `chInstanceObjectInit` | Attaches a core to the ChibiOS kernel |
| `0x1001136c` | `_crt0_c1_entry` | Core1 CRT0 trampoline |

---

## 8. Next Steps

### Immediate (Active Debug)

0. **Run `/tmp/catch_first_fault.tcl`** — sets a hardware breakpoint at `HardFault_Handler` (0x100a77f0), resets, and catches the first fault before it escalates to lockup. Script already written. Inspect PSP frame to get the faulting PC and fault registers:
   ```bash
   /home/buzz/openocd-pico/openocd \
     -f interface/cmsis-dap.cfg -f target/rp2350.cfg \
     -c "tcl_port 0" -c "telnet_port 0" -c "gdb_port 0" \
     -f /tmp/catch_first_fault.tcl
   ```
   **This is the most direct path to finding the root cause.**

1. **Understand the double-fault root cause**: The HardFault handler (`system.cpp:181`) does:
   ```c
   memcpy(&ctx, (void*)__get_PSP(), sizeof(struct port_extctx));
   ```
   If PSP is invalid when the first HardFault fires, this memcpy causes a second HardFault → lockup. The bp at 0x100a77f0 catches the handler *before* the memcpy.

2. **Add sub-stage canaries inside `__vt_object_init`**: Inline the vtlist initialization in `chinstances.c` with intermediate `c1_inst_stage` writes (0x42a, 0x42b, 0x42c, ...) to pinpoint the exact faulting instruction.

3. **Read SCB->CFSR and HFSR**: After the crash, use OpenOCD `targets rp2350.dap.core0` and `targets rp2350.dap.core1` to read fault status registers on both cores.

4. **Try disabling MPU**: Comment out `mpuEnable()` in `port_init` temporarily. If the crash disappears, the MPU MAIR configuration is the culprit even though all regions are disabled.

5. **Check for SIO FIFO stale data**: Drain the SIO RX FIFO completely in `__port_smp_init` before enabling IRQ25, not just clearing error flags.

### Short Term

5. **Rate thread pinning**: Once core1 is stable, use `thread_create_alloc_on_core()` in `Scheduler.cpp` to pin the ArduPilot rate thread to core1.

6. **MAVLink over USB verification**: Confirm telemetry/MAVLink continues to work under SMP (core0 handles USB).

7. **SMP stress test**: Run with deliberate inter-core load to verify spinlock and scheduler correctness.

### Longer Term

8. **WFE/WFI idle optimization**: Replace the busy `b.n` loop in c1_main with `port_wait_for_interrupt()` to reduce power consumption on core1's idle thread.

9. **Performance measurement**: Measure latency improvement from pinning the rate loop to a dedicated core.

---

## 9. Additional Bug: IRQ25 Dispatch to DebugMon_Handler on Core1

A second distinct bug was observed in an intermediate debug session (before the core0 lockup regression appeared):

- When core1's SIO FIFO ISR fires (IRQ25), it dispatches to `DebugMon_Handler` (0x1001128a, the default ChibiOS handler) instead of `VectorA4` (0x100eeef8).
- The vector table entry at [VTOR+0x124] = [0x10010124] correctly contains 0x100eeef9 (VectorA4 thumb address).
- This means the CPU is fetching the wrong vector at exception entry, OR the FPB is redirecting the fetch.
- **OpenOCD FPB breakpoints** were found at addresses 0x1001128a and 0x1001128e — these caused the CPU to halt when the vector table was fetched (OpenOCD's software bp at DebugMon_Handler caused the IRQ dispatch to appear to go there).
- After removing the FPB comparators, it needs to be verified whether IRQ25 correctly dispatches to VectorA4.
- This issue was not definitively resolved before the core0 lockup regression appeared.

**Current status**: Likely not a real dispatch bug but an artifact of OpenOCD hardware breakpoints. Needs re-verification once core0 stability is restored.

---

## 10. Summary Assessment

Two active crash bugs are blocking SMP operation:

**Bug A (core1 crash — older build observation):** Core1 successfully boots, synchronizes with core0, enters `chInstanceObjectInit`, completes `port_init` (including MPU, FPU, NVIC, SMP timer binding), and initializes the ready-list queue. The crash was isolated to `__vt_object_init` (c1_inst_stage stops at 0x42). Core1 resets to the RP2350 boot ROM. Root cause undetermined.

**Bug B (core0 lockup — current build):** Core0 crashes within ~600ms of boot with a double-fault lockup (PC=0xeffffffe, MSP=0x4c12f228 corrupted). The `HardFault_Handler` in `system.cpp` does `memcpy(&ctx, PSP, ...)` as its first action — if PSP is invalid when the first fault fires, this memcpy causes a second HardFault → lockup. Root cause undetermined.

**Immediate priority:** Run `catch_first_fault.tcl` (hw bp at HardFault_Handler 0x100a77f0) to capture the first fault's exception frame and CFSR/HFSR before the double-fault escalation.

---

## 11. Architectural Decision: Abandon ChibiOS SMP, Use Bare-Metal Core1 Dispatcher

### Decision (2026-03-22)

After spending significant time debugging `chInstanceObjectInit` crashes on core1 (Bug A: stuck at `__vt_object_init`, c1_inst_stage=0x42) and a core0 double-fault lockup (Bug B: PC=0xeffffffe), and after observing that core1's IRQ25 (SIO FIFO) consistently dispatches to the default handler instead of VectorA4, the decision was made to **abandon `CH_CFG_SMP_MODE=TRUE`** and replace it with a simpler, more robust architecture.

### New Architecture: "Core0 Schedules, Core1 Runs"

```
Core0 (ChibiOS, CH_CFG_SMP_MODE=FALSE)
  ├── All ChibiOS scheduling, IRQs, USB, MAVLink, EKF, etc.
  ├── Rate tick fires → writes function pointer to SIO->FIFO_WR
  └── Optionally waits for completion signal from core1

Core1 (bare-metal, no ChibiOS at all)
  ├── Starts via RP_CORE1_START=TRUE → _crt0_c1_entry → c1_main()
  ├── Polls SIO->FIFO_RD for function pointer
  ├── Calls fn() (the rate controller body)
  └── Writes done-signal to SIO->FIFO_WR → loops
```

Core0 remains the single source of scheduling truth. Core1 is a pure compute engine: it receives a function pointer, runs it to completion, and signals done. No ChibiOS instance on core1, no chInstanceObjectInit, no virtual timer list, no PendSV, no SIO IRQ handlers.

### Tradeoffs

| | ChibiOS SMP (`CH_CFG_SMP_MODE=TRUE`) | Bare-metal core1 dispatcher |
|---|---|---|
| Core1 thread sleep/delay | ✅ Full ChibiOS primitives | ❌ Cannot block — rate body must run to completion each tick |
| Multiple threads on core1 | ✅ Full scheduler | ❌ One function at a time |
| Scheduler complexity | ❌ Very high — two scheduler instances, spinlock, FIFO IRQ, cross-core wakeup | ✅ Minimal — SIO FIFO poll loop |
| Stability | ❌ Currently crashing (Bugs A+B) | ✅ Expected to be stable immediately |
| Rate controller requirement | Any ChibiOS-aware thread | Body must be a non-blocking function call |
| Implementation risk | ❌ High | ✅ Low |
| Performance gain | Same | Same (core1 dedicated to rate controller) |

The rate controller body in ArduPilot (`fast_update()` / `AP_Vehicle::update()` inner loop) is a periodic computation — it does not block within a single iteration. It is suitable for the bare-metal dispatch model.

### To-Do Items (Implementation Checklist)

- [x] **Decision documented** in SMP_Status_progress_report.md
- [ ] **`chconf_rp2350.h`**: Revert `CH_CFG_SMP_MODE` from `TRUE` → `FALSE`
- [ ] **`c1_main.c`**: Rewrite as bare-metal SIO FIFO dispatcher:
  - Poll `SIO->FIFO_ST.VLD`, read function pointer from `SIO->FIFO_RD`
  - Call the function (rate controller body or test stub)
  - Write done-signal to `SIO->FIFO_WR`
  - WFE when idle
- [ ] **`stm32_util.h`**: Remove `thread_create_alloc_on_core()` declaration (SMP helper, no longer needed)
- [ ] **`malloc.c`**: Remove `thread_create_alloc_on_core()` implementation
- [ ] **`Scheduler.cpp`**: Replace the `thread_create_alloc_on_core()` SMP block with a SIO FIFO dispatch hook for the rate thread. The rate thread wrapper on core0 sends the function body to core1 each tick.
- [ ] **`board.c`**: Keep SIO FIFO drain in `__late_init()` (still correct — drain any ROM-handshake residue before use). Remove `#if CH_CFG_SMP_MODE == TRUE` guard since the drain is useful regardless.
- [ ] **Linker script** (`common_rp2350_smp.ld`): Keep core1 MSP/PSP stack sections — `_crt0_c1_entry` still needs them. Rename to `common_rp2350_c1.ld` to reflect new non-SMP purpose.
- [ ] **Build** and confirm zero compile errors
- [ ] **Flash and verify**: `c1_boot_stage` reaches `0x4D` (idle loop entered), core1 is alive
- [ ] **Test rate dispatch**: core0 sends test function pointer, core1 executes it, signals done
- [ ] **Hook rate controller**: dispatch actual `fast_update()` body to core1 each tick

### What Is NOT Needed Anymore

- `CH_CFG_SMP_MODE=TRUE` — remove
- `chInstanceObjectInit` on core1 — remove from c1_main.c
- `chSysWaitSystemState` on core1 — remove
- `ch1` os_instance_t — still declared in ChibiOS but never initialized (harmless)
- `VectorA4` SIO FIFO ISR on core1 — core1 never enables its NVIC, so IRQ25 never fires
- `thread_create_alloc_on_core()` — remove entirely
