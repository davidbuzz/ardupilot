# ArduPilot Pico2 USB CDC UART Comms Failure — Deep Technical Report (Rev 2)

Date: 2026-03-31  
Repository: /home/buzz/ardupilot  
Branch family: buzz-rp2350-chibios-v3 (baseline check done at commit fed1d8d1d0)  
Target board: Raspberry Pi Pico2 (RP2350, Cortex-M33 dual-core, ChibiOS running single-core mode)  
Debugger transport: CMSIS-DAP debugprobe + OpenOCD + GDB  
Observed host OS: Linux  

---

## 1. Scope And Objective

This report documents the complete technical investigation into a persistent communication failure where ArduPilot firmware on Pico2 enumerates as USB CDC ACM, but host reads return zero bytes and MAVProxy never receives heartbeat traffic.

Primary objective:
- Determine whether the fault is in firmware transmit path, USB device controller state, ChibiOS USB serial driver, or host-side USB polling behavior.

Secondary objective:
- Validate whether recent UARTDriver partial-flush experiments introduced a regression.

Key conclusion preview:
- The failure is not caused by the partial-flush patch.
- Firmware reaches normal scheduler runtime and writes data into the USB TX path.
- USB CDC software and hardware show queued IN data awaiting host fetch.
- Host appears not to issue IN tokens to consume endpoint 1 IN payload, causing obqueue saturation and apparent serial silence.

---

## 2. Symptom Statement

### 2.1 User-visible behavior
- Device appears as /dev/ttyACM* and enumerates with expected VID:PID.
- Python serial reads return 0 bytes repeatedly.
- MAVProxy exits by timeout without heartbeat detection.
- Manual DTR/RTS manipulation does not produce readable output.

### 2.2 Why this is important
- SERIAL0 on Pico2 is the primary USB console/MAVLink channel.
- Lack of USB RX on host blocks flight stack observability and blocks normal bench validation.

---

## 3. Test Environment And Known Addresses

### 3.1 OpenOCD/GDB ports used in this campaign
- OpenOCD GDB server: port 50010
- OpenOCD telnet-style memory access: port 50012

### 3.2 Key symbols and runtime addresses verified
- serial0Driver base: 0x20012900
- serial0Driver._total_written: 0x200129fc
- SDU1 base (ChibiOS SerialUSBDriver): 0x20019120
- USBD1 base: 0x20019900
- USBD1.transmitting field observed at/around: 0x20019928 (halfword)
- EP1 IN DPRAM buffer control register: 0x50100088
- USB SIESTATUS register: 0x50110050
- USB BUFSTATUS register: 0x50110058
- USB INTE register: 0x50110090

### 3.3 Host-side identity checks
- Device path observed: /sys/bus/usb/devices/1-3.1.2/
- VID:PID observed: 1209:5741
- runtime_status observed: active

---

## 4. Chronological Investigation Log (Condensed But Complete)

1. Regression check run against baseline firmware (no new patch): host still received 0 bytes.
2. Therefore issue classified as pre-existing, not introduced by in-progress UART flush work.
3. USB status register interpretation corrected (earlier confusion with SOFRD vs SIESTATUS fixed).
4. GDB runtime verified that firmware is alive and executing AP scheduler-loop work (example stack in AP_AHRS::update_DCM).
5. serial0Driver structure fully inspected in RAM; USB binding and init flags were valid.
6. Pending byte accumulation and transmit counters confirmed data is being written by firmware.
7. Deep USB queue/endpoint state dump showed TX queue full, endpoint buffer marked available/full, but no completion activity.
8. USB interrupt enable set confirmed DEV_SOF interrupt bit active.
9. Dominant fault model shifted from device-not-transmitting to host-not-consuming.
10. usbmon capture started to confirm whether Linux schedules IN transactions for the ACM data endpoint.

---

## 5. Firmware Runtime Health Verification

### 5.1 Core scheduler not deadlocked
Evidence from GDB thread snapshots and backtraces showed normal thread activity and main control-loop execution, not a startup stall.

Interpretation:
- This is not the earlier setup-not-complete failure mode.
- The system has reached a state where periodic MAVLink generation is plausible.

### 5.2 serial0Driver state proved operational
Extracted observations:
- uart_thread_name = OTG1
- sdef.is_usb = true
- sdef.serial points to SDU1
- _tx_initialised = true
- _rx_initialised = true
- _device_initialised = true
- _flow_control = FLOW_CONTROL_ENABLE

Interpretation:
- USB-backed UART driver is fully initialised and connected to ChibiOS SerialUSB backend.
- The code path for write attempts is active.

---

## 6. Data-Path Evidence: Bytes Are Generated But Stop Progressing

### 6.1 Write buffer and total-written counters
Observed at one sampling point:
- _writebuf.size = 2048
- _writebuf.head = 512
- _writebuf.tail = 487
- pending in ring = 25 bytes
- _total_written had reached values including 2560, later 8448 bytes

Then:
- _total_written stopped increasing.

Interpretation:
- Firmware successfully wrote a nontrivial amount of outgoing data.
- Transmission ceased making forward progress after downstream queue saturation.

### 6.2 Critical queue saturation snapshot
Observed:
- SDU1.obqueue.bn = 2
- SDU1.obqueue.bcounter = 0

Meaning:
- Two total TX buffers exist in obqueue.
- Zero free buffers means both are occupied.

Impact:
- chnWriteTimeout(..., TIME_IMMEDIATE) returns 0 when no free obqueue buffer exists.
- UARTDriver write loop stops advancing write throughput despite active producer.

---

## 7. USB Controller State Evidence

### 7.1 SIE and connection status
Observed SIESTATUS around:
- 0x40051005 / 0x40051009 variants during snapshots

Decoded flags indicate:
- CONNECTED = 1
- VBUS detected = 1
- full-speed negotiated
- not in suspended state at sampled points

### 7.2 Endpoint transmission state
Observed:
- USBD1.state = USB_ACTIVE
- USBD1.transmitting = 2 (endpoint 1 IN is software-marked transmitting)
- EP1 IN BUF_CTRL at 0x50100088 contained 0xc437a440

For this investigation, that value was interpreted as:
- FULL asserted
- AVAILABLE asserted
- payload of 64 bytes prepared in DPRAM and handed to controller

### 7.3 Completion status mismatch
Observed simultaneously:
- BUFSTATUS = 0

Interpretation:
- Device-side completion path is not firing for this IN payload.
- Controller has a prepared buffer, but host has not consumed/acknowledged it in a way that advances buffer lifecycle.

---

## 8. Why This Looks Like Host IN-Token Starvation

The specific signature is highly consistent:
1. Device firmware writes bytes into ChibiOS stack.
2. ChibiOS queue becomes full.
3. Endpoint IN buffer is primed and marked available/full.
4. No endpoint completion events arrive.
5. Host user-space sees no bytes.

If the device path were fundamentally broken before queueing, obqueue would not fill and endpoint buffer would not show pending IN payload.

If the device were suspended/disconnected, SIE status and runtime activity would show that. They did not.

Therefore the dominant explanation is:
- Linux host side is not issuing periodic IN transactions for this ACM data endpoint (or not under current line-state/driver conditions).

---

## 9. SOF Interrupt Debate Resolution

There was an intermediate ambiguity about whether SOF interrupt was enabled.

Final verified state:
- INTE register = 0x0003d010 at snapshot.
- Bit 17 (0x20000, DEV_SOF) set.

Conclusion:
- SOF interrupts are enabled; lack of forward TX progress is not explained by missing SOF enable.

---

## 10. DTR/RTS/Flow-Control Considerations

### 10.1 Why this remains relevant
- serial0Driver field showed FLOW_CONTROL_ENABLE even for USB-backed UART abstraction.
- Several host tests toggled DTR/RTS manually and still saw no payload.
- Some CDC ACM stacks gate polling behavior on control line state transitions.

### 10.2 Current confidence level
- DTR gating remains a plausible contributor but is not yet proven root cause.
- The stronger direct evidence remains absence of host consumption despite queued endpoint data.

---

## 11. Regression Assessment

Status: cleared.

Evidence:
- Baseline firmware from current branch head reproduced identical 0-byte behavior.
- Partial obqueue-flush change was stashed and not present in baseline retest.

Conclusion:
- Existing fault predates new UARTDriver experimentation.

---

## 12. Rejected Or De-prioritized Hypotheses

1. Firmware boot crash or hard-fault loop.
- Rejected by normal thread activity and scheduler backtraces.

2. setup() never completes.
- De-prioritized for this campaign by runtime evidence in normal loop work.

3. USB not enumerated / cable issue / suspended bus.
- Rejected by SIE/host active status and valid USB state.

4. No data generated by firmware.
- Rejected by _total_written progression to multi-kilobyte values before stall.

5. SOF callback disabled.
- Rejected by INTE bit verification.

---

## 13. Root-Cause Statement (Current Best)

Current best root-cause model:

The RP2350 device side successfully enqueues CDC ACM TX data and primes EP1 IN buffers, but Linux host behavior does not progress IN transfer completion for that endpoint. This prevents buffer release in ChibiOS obqueue, causes non-blocking writes (TIME_IMMEDIATE) to return zero, and results in persistent user-space reads of zero bytes.

Confidence level:
- High for device-side queueing and stalling mechanism.
- Medium-high for host IN-token starvation as primary external cause (usbmon confirmation still required for final closure).

---

## 14. High-Value Evidence Table

| Layer | Observation | Value/State | Diagnostic meaning |
|---|---|---|---|
| ArduPilot UART | serial0Driver init flags | tx/rx/device initialised true | Driver is up |
| ArduPilot UART | _total_written | reaches 8448 then stops | Data produced then stalled |
| ChibiOS SDU | obqueue.bn | 2 | Two TX queue buffers |
| ChibiOS SDU | obqueue.bcounter | 0 | No free TX buffers |
| USB stack | USBD1.state | USB_ACTIVE | Enumerated and configured |
| USB stack | USBD1.transmitting | 2 | EP1 IN marked in-flight |
| USB DPRAM | EP1 IN BUF_CTRL | 0xc437a440 | 64B payload primed |
| USB regs | BUFSTATUS | 0 | No completion indication |
| USB regs | INTE bit17 | set | SOF interrupt enabled |
| Host sysfs | runtime_status | active | Link not runtime-suspended |

---

## 15. USBMON Workstream Status

usbmon infrastructure was loaded and capture started on bus monitor endpoint.

Status at this report revision:
- Capture sequence initiated.
- Final parsed trace proving presence/absence of IN submissions for the exact device/endpoint was still in progress at handoff boundary.

Required closure artifact:
- A short parsed usbmon extract mapping device address and endpoint 0x81 IN transaction cadence over 2 to 5 seconds.

---

## 16. Practical Implication For UARTDriver Code Path

Because writes use immediate timeout semantics in this path:
- Once obqueue is full, writes fail fast.
- Producer traffic appears to stop even if system is otherwise healthy.

This behavior is expected under downstream starvation and does not by itself imply firmware logic fault.

Potential resilience improvement (separate from root cause):
- Add instrumentation counters for USB write zero-return events and queue occupancy transitions.
- Optionally allow bounded wait/yield policy under sustained full queue to reduce abrupt starvation behavior.

---

## 17. Recommended Next Diagnostic Sequence (Deterministic)

1. Collect usbmon filtered trace tied to target devnum and endpoint.
2. Verify whether host sends IN submits/completions on ACM data endpoint.
3. Capture control-transfer traffic, especially SET_CONTROL_LINE_STATE (DTR/RTS changes).
4. Compare behavior across:
   - pyserial with explicit DTR true before read
   - mavproxy open sequence
   - minicom/screen (known to assert line state reliably)
5. If host submits IN but no completion, inspect endpoint descriptors and max packet consistency from lsusb -v capture.
6. If host does not submit IN at all, focus on Linux cdc_acm state machine and line-state requirements for this device profile.

---

## 18. Suggested Instrumentation Additions For Next Pass

Add temporary debug counters/log hooks in UARTDriver USB path and/or ChibiOS glue:
- Count chnWriteTimeout return values by status (full queue vs partial write).
- Track transitions of obqueue.bcounter.
- Track each call to usbStartTransmitI and associated endpoint payload length.
- Track data-transmitted callback count.

These metrics can be read via GDB symbols to avoid serial dependency while serial itself is under test.

---

## 19. Risk Assessment

### Functional risk
- USB telemetry/console path unreliable or unusable in current host/firmware interaction.

### Safety/testing risk
- Reduced bench visibility may mask other initialization or runtime faults.
- Alternate serial path (UART1/UART2) may be required for temporary observability.

### Regression risk from current findings
- Low, because issue reproduces on baseline and no mandatory functional code change has yet been merged as a fix.

---

## 20. Executive Summary

The communication failure is not due to firmware startup crash, missing USB initialization, or absence of MAVLink generation attempts. Device-side software successfully writes USB TX payloads into ChibiOS and primes endpoint buffers, but transfer completion does not occur. The resulting queue saturation causes immediate non-blocking writes to fail and manifests as persistent zero-byte reads on Linux.

Most probable fault domain is host-side CDC ACM polling/control-line behavior (or host-device interaction at that layer), not core UARTDriver TX generation logic.

Final closure requires usbmon endpoint-level confirmation and then either host-side workaround (line-state policy) or device descriptor/control-path adjustment if needed.

---

## 21. Appendix A — Representative Command Set Used

OpenOCD/telnet style snapshots:
- halt
- reg pc
- mdw 0x20012900 40
- mdw 0x20019120 24
- mdw 0x20019920 4
- mdw 0x50100088 2
- mdw 0x50110050 1
- mdw 0x50110058 1
- mdw 0x50110090 1
- resume

Host checks:
- timeout-based pyserial readers on /dev/ttyACM*
- MAVProxy setup/heartbeat checks with 12 to 15 second windows
- usbmon enabled via modprobe and capture from /sys/kernel/debug/usb/usbmon/1u

---

## 22. Appendix B — Status Of In-Progress Code Work

- Partial USB queue-flush code experiment exists in stash and was intentionally not active during baseline revalidation.
- No claim is made here that this stash fixes the root issue.
- The diagnostic findings in this report should be treated as independent of that pending code experiment.

---

## 23. Forensic Evidence Ledger (High Detail)

This section records the strongest evidence items in a logbook style. Time is relative to each local experiment window and is included to preserve ordering confidence even where absolute timestamps were not captured.

### 23.1 Host symptom observations

Observation H1:
- Repeated short-window reads from /dev/ttyACM1 and /dev/ttyACM2 returned zero payload bytes.
- Both pyserial and mavproxy windows ended by timeout.

Observation H2:
- Explicit DTR and RTS toggles were attempted from userspace.
- No stable data flow followed those control-line changes in the sampled windows.

Observation H3:
- Linux runtime power state showed device active.
- No evidence of runtime autosuspend at the time of the critical queue-stall snapshots.

Inference from H1 through H3:
- A host-visible ACM node exists and remains openable.
- User-space access itself is not denied.
- The failure mode is data-flow specific, not enumeration failure.

### 23.2 Firmware liveness observations

Observation F1:
- GDB backtraces showed normal scheduler execution and active flight-code paths, including AP_AHRS work.

Observation F2:
- No persistent hard-fault signature was observed in the main loop during the key sampling period.

Observation F3:
- serial0Driver state showed initialized transmit and receive paths and valid USB backend binding.

Inference from F1 through F3:
- Firmware is alive and capable of generating outbound bytes.
- Failure is downstream of MAVLink generation and UART write submission.

### 23.3 Queue and endpoint observations

Observation Q1:
- serial0Driver._total_written advanced into multi-kilobyte range and then stopped.

Observation Q2:
- SDU1 obqueue had bn equals 2 and bcounter equals 0 at critical snapshot.

Observation Q3:
- USBD1.transmitting indicated endpoint 1 IN busy state.

Observation Q4:
- EP1 IN DPRAM buffer control indicated a full 64-byte packet marked available.

Observation Q5:
- BUFSTATUS remained zero at matching snapshots.

Inference from Q1 through Q5:
- Producer writes succeeded initially.
- Queue resources became exhausted.
- Endpoint payload was armed but completion did not retire queued buffers.

### 23.4 Interrupt and line-state observations

Observation I1:
- INTE register included DEV_SOF enable bit.

Observation I2:
- SIESTATUS indicated connected, VBUS present, full-speed, and not suspended at sampled points.

Inference from I1 and I2:
- No support for the theory that SOF was disabled or bus was functionally detached.

### 23.5 Consolidated failure signature

The sequence that repeated is:
- Outbound data generated.
- Data enqueued.
- Queue fills completely.
- Endpoint marked transmitting with pending packet.
- Completion path does not progress.
- Host receives zero bytes.

That signature is the central evidence for host polling starvation or equivalent host-device transaction dead-zone on the ACM data endpoint.

---

## 24. Register Decoding Notes (Detailed)

This section captures the semantic meaning of critical registers used in the investigation and how each value informed diagnosis.

### 24.1 SIESTATUS at 0x50110050

Representative observed values included 0x40051005 and 0x40051009 in different snapshots.

Interpretive summary used in this campaign:
- Link connected.
- VBUS detected.
- Full-speed negotiation active.
- Not in suspended state during critical captures.

Diagnostic impact:
- Rules out simple unplugged, unattached, or suspended explanations for zero-byte host reads.

### 24.2 INTE at 0x50110090

Representative observed value: 0x0003d010.

Key bit for this campaign:
- Bit 17 set, interpreted as DEV_SOF interrupt enable.

Diagnostic impact:
- Refutes missing-SOF-enable hypothesis.

### 24.3 BUFSTATUS at 0x50110058

Representative observed value at critical points: 0x00000000.

Diagnostic impact:
- No completed endpoint buffer retirement indicated while queue remained full.
- Reinforces transaction completion starvation model.

### 24.4 EP1 IN BUF_CTRL at 0x50100088

Representative observed value: 0xc437a440.

Interpreted as:
- Packet length reflects 64-byte primed TX payload.
- FULL and AVAILABLE state bits indicate controller has data prepared for host fetch.

Diagnostic impact:
- Device prepared payload is present; failure is after preparation stage.

---

## 25. ChibiOS And ArduPilot State-Machine Walkthrough

This section narrates control flow to show exactly where progress halts.

### 25.1 Producer side

1. MAVLink generation in ArduPilot emits bytes to UART abstraction.
2. ChibiOS UARTDriver USB path accepts data into ring buffer.
3. write_pending_bytes_NODMA attempts chnWriteTimeout with immediate timeout.

### 25.2 Queue handoff side

4. ChibiOS serial USB obqueue consumes data until all available queue buffers are occupied.
5. usbStartTransmitI is used to launch endpoint IN transfer when idle.

### 25.3 Completion side

6. Normal path requires host IN token and successful completion callback.
7. Completion callback frees queue buffers and chains further data.

### 25.4 Failure condition in this incident

8. Completion callback progression is absent while endpoint remains armed.
9. Queue never frees.
10. Immediate-timeout writes return zero when queue has no free buffer.
11. Producer appears stalled despite active firmware.

This explains both internal counters and external zero-byte reads without requiring a firmware crash.

---

## 26. Hypothesis Matrix With Confidence Scoring

Scoring convention:
- Evidence fit scored 0 to 5.
- Higher score means better fit to observed facts.

| Hypothesis | Evidence fit | Notes |
|---|---|---|
| Host not issuing IN tokens for ACM data endpoint | 5 | Best fit to armed EP buffer plus no completion plus host zero-read behavior |
| Device-side endpoint stuck due to controller bug despite host polling | 3 | Plausible but weaker without usbmon proof of IN traffic |
| MAVLink not generated by firmware | 1 | Contradicted by _total_written growth into multi-kilobyte range |
| USB not active or not configured | 0 | Contradicted by SIESTATUS and USBD1 active state |
| SOF callback disabled | 0 | Contradicted by INTE bit state |
| Regression from partial flush patch | 0 | Contradicted by baseline reproduction with patch removed |

Current best explanation remains host-side polling starvation or host-device ACM control mismatch.

---

## 27. Reproducibility Protocol (Step-By-Step)

This protocol is designed so another engineer can reproduce the exact incident signature.

### 27.1 Preconditions

- OpenOCD running and attached to target.
- Firmware built and loaded on Pico2.
- Host has pyserial and mavproxy installed.

### 27.2 Reproduce host symptom

1. Open ACM port with short timeout reads for 5 to 12 seconds.
2. Confirm total bytes read remains zero.
3. Repeat with mavproxy setup mode and observe timeout with no heartbeat.

Expected signature:
- Port opens.
- No payload arrives.

### 27.3 Capture internal queue signature

1. Halt target briefly through OpenOCD.
2. Read serial0Driver and SDU1 queue fields.
3. Read USBD1 transmitting state.
4. Read EP1 IN BUF_CTRL and BUFSTATUS.
5. Resume target.

Expected signature:
- _total_written nonzero and may plateau.
- obqueue free-count zero.
- endpoint buffer armed.
- no completion indication.

### 27.4 Confirm SOF enable and link health

1. Read INTE.
2. Read SIESTATUS.

Expected signature:
- DEV_SOF enabled.
- connected and active link state.

### 27.5 Optional host side confirmation

1. Enable usbmon.
2. Capture short trace on correct bus.
3. Filter by target devnum and endpoint.
4. Check for IN submission cadence.

Closure criteria:
- If IN submissions are absent, host polling starvation is confirmed.
- If IN submissions exist but completion absent, investigate descriptor-level or controller-level path.

---

## 28. Detailed Command Transcript Catalogue

This catalogue lists command classes used, including purpose and expected diagnostic output shape.

### 28.1 Host payload checks

Class A commands:
- short pyserial loops with timeout 0.05 to 0.3 seconds
- total byte counters over 5 to 12 second windows

Use:
- Fast yes/no check for visible payload.

Typical outcome in this incident:
- total equals zero.

### 28.2 MAVProxy checks

Class B commands:
- timeout-wrapped mavproxy setup and heartbeat scans

Use:
- Verify if MAVLink heartbeat is observable over ACM.

Typical outcome in this incident:
- command times out with no usable heartbeat lines.

### 28.3 OpenOCD register snapshots

Class C commands:
- halt, mdw, reg pc, resume sequences

Use:
- Snapshot queue, endpoint, and controller state under failure.

Typical outcomes in this incident:
- queue full and endpoint armed with no completion progression.

### 28.4 Multi-port write counter comparisons

Class D commands:
- read _total_written for SERIAL0, SERIAL1, SERIAL2 at T0 and T5.

Use:
- Compare relative port activity and isolate whether only USB path is affected.

Typical outcomes in this incident:
- SERIAL0 showed growth then stall behavior under queue saturation model.

---

## 29. Quantitative Interpretation Of Throughput Stall

Given:
- obqueue has two transmit buffers.
- endpoint packet payload observed as 64 bytes.

Practical consequence:
- Effective queued payload capacity on the active USB path is small.
- If host consumption pauses even briefly, queue fills rapidly.

Once queue full:
- Immediate-timeout writes return zero.
- Producer does not block; it simply cannot enqueue additional bytes.

So the observed plateau in _total_written is expected once downstream retirement stops, and it can happen abruptly.

---

## 30. Why DTR Testing Was Inconclusive But Still Important

DTR and RTS manipulations were attempted from host userspace and did not recover flow in the sampled attempts.

Why this does not fully eliminate control-line involvement:
- Different host applications may apply line-state transitions at different times.
- Some stacks require specific open order, not merely final DTR level.
- Control requests may race with endpoint readiness during reconnect cycles.

Therefore DTR hypothesis is downgraded but not mathematically eliminated until usbmon confirms control transfer and endpoint polling sequence together.

---

## 31. Decision Log: Why No Immediate Firmware Patch Was Declared Final

No definitive fix commit was produced in this incident window because:
- Baseline reproduced failure unchanged.
- Device-side evidence strongly implicated host transaction behavior.
- Without usbmon closure, patching device code would be speculative and high-risk for regressions.

Engineering decision:
- Preserve investigative integrity.
- Gather endpoint-level host traffic proof before asserting a device-side corrective patch.

---

## 32. Closure Criteria For Incident Resolution

The incident should be considered closed only when all criteria below are met:

1. Host receives sustained nonzero ACM payload from SERIAL0 over at least 60 seconds.
2. MAVProxy detects heartbeat consistently without special ad hoc toggles.
3. _total_written continues to advance without hard plateau under normal heartbeat traffic.
4. obqueue free count oscillates as expected and does not remain stuck at zero.
5. Endpoint completion activity is visible, directly or via indirect queue retirement evidence.
6. Re-test after disconnect/reconnect cycle shows same healthy behavior.
7. Baseline and patched behavior comparison is documented.

---

## 33. Immediate Next Work Package (Actionable)

Work package WP-USBMON-01:
- Capture 3 to 5 second usbmon traces with target devnum known.
- Produce parsed table with columns: timestamp, endpoint, direction, submit/complete, status.
- Explicitly count IN submits to ACM data endpoint.

Work package WP-CTRL-02:
- Capture control transfers for SET_CONTROL_LINE_STATE and line coding interactions.
- Correlate control transfer timing against first attempted data reads.

Work package WP-DIFF-03:
- Compare host behavior between pyserial, mavproxy, and one terminal utility using same device node in separate runs.
- Record whether endpoint polling behavior differs by application.

Work package WP-INSTR-04:
- Add temporary counters for usbStartTransmitI calls and data-transmitted callbacks.
- Read counters over GDB while host reads are active.

---

## 34. Additional Risk Notes For Ongoing Development

Risk R1:
- Developers may misinterpret zero-byte ACM reads as firmware deadlock and make unrelated code changes.

Mitigation:
- Check queue and endpoint state first before changing startup logic.

Risk R2:
- Applying aggressive retry loops in firmware may hide host-side starvation while increasing CPU load.

Mitigation:
- Keep instrumentation lightweight and time-bounded.

Risk R3:
- Incomplete evidence could lead to premature blame of either host driver or RP2350 USB path.

Mitigation:
- Require usbmon proof for final attribution.

---

## 35. Summary Delta From Previous Report Revision

Compared to the prior revision, this revision adds:
- A formal forensic evidence ledger.
- Register semantics and diagnostic impact commentary.
- State-machine stage breakdown from producer to completion callback.
- Hypothesis scoring matrix.
- Full reproducibility protocol and closure criteria.
- Work-package structure for next execution cycle.

This creates a full engineering incident package rather than a short diagnostic note.

---

## 36. Formal Failure Tree Analysis

This section frames the incident using a hierarchical failure tree so future investigations can prune branches systematically.

Top event:
- Host reads from ACM device return zero bytes over repeated windows while firmware is expected to emit MAVLink heartbeat traffic.

Branch A: Firmware does not generate payload.
- A1: Main loop not running.
- A2: MAVLink scheduler task suppressed.
- A3: UART write path disabled.

Branch B: Firmware generates payload but fails before USB endpoint arm.
- B1: Ring buffer logic incorrect.
- B2: chnWriteTimeout always returns zero due to local bug.
- B3: SDU not active.

Branch C: Endpoint armed but not consumed.
- C1: Host never submits IN tokens.
- C2: Host submits IN but transaction never completes.
- C3: Descriptor/control mismatch causes host-side read starvation.

Branch D: Host reads consumed data from wrong interface node.
- D1: Wrong ttyACM node selected after reconnect.
- D2: Alternate process holding intended device node.

Branch E: Observability artifact (false negative).
- E1: Reader timeout too short for bursty output.
- E2: Binary MAVLink discarded by text-only filters.

Tree-pruning status from current evidence:
- Branch A mostly pruned by runtime activity and write counters.
- Branch B mostly pruned by queue fill and endpoint arm evidence.
- Branch C remains dominant and unresolved.
- Branch D partially controlled but should be continuously guarded with by-id paths.
- Branch E partially mitigated, but binary framing should always be considered when parsing output.

---

## 37. Queue Dynamics Model

The observed stall can be represented as a small finite-capacity queue under non-blocking producer semantics.

Definitions:
- Q: available obqueue capacity in bytes.
- lambda_p: producer byte rate from firmware into USB path.
- lambda_c: consumer byte rate retired by host IN transactions.
- W: write function behavior with TIME_IMMEDIATE.

Dynamics:
- If lambda_p > lambda_c for sufficient time, Q approaches 0 free capacity.
- Once free capacity is exhausted, W returns 0 for additional writes rather than blocking.

Practical implication:
- The producer appears stalled despite active firmware execution.
- _total_written plateaus and remains stable until consumer retirement resumes.

Boundary condition relevant here:
- Very small effective queued payload allows short host polling disruptions to force saturation rapidly.

---

## 38. Interrupt-to-Completion Causality Chain

Expected healthy causal chain:
1. Firmware queues bytes.
2. USB endpoint IN is armed.
3. Host sends IN token.
4. Controller transfers payload.
5. Endpoint completion interrupt fires.
6. ChibiOS callback retires one queue buffer.
7. Next queued payload may be armed.

Observed broken chain:
1. Steps 1 and 2 observed.
2. Steps 3 through 7 not evidenced by buffer retirement behavior at critical snapshots.

Diagnostic meaning:
- Breakpoint likely between host IN scheduling and completion interrupt progression.

---

## 39. Host-Side Transaction Expectations (ACM Data Endpoint)

For each open and active read flow, host is expected to:
- Maintain periodic IN requests to data IN endpoint.
- Process returned payloads into tty buffer.
- Continue polling even during low throughput.

Failure indicators in host traces:
- No IN submit entries for endpoint 0x81.
- IN submits present but no completes or repeated error status.
- Control requests that leave line-state in unexpected condition for this device implementation.

Investigation consequence:
- usbmon parsing is the single highest-value next artifact to settle branch C1 vs C2.

---

## 40. Evidence Confidence Calibration

Confidence bands used in this report:
- Strong: directly observed in registers or counters.
- Moderate: inferred from consistent multi-source observations.
- Weak: plausible but unproven without additional capture.

Examples:
- Strong: obqueue free count equals zero.
- Strong: endpoint buffer control indicates primed packet.
- Moderate: host IN-token starvation as dominant cause.
- Weak: exact role of DTR timing in this specific failure.

This calibration is important so future edits do not accidentally elevate hypotheses to facts.

---

## 41. Controlled Experiment Matrix

The matrix below defines experiments required to isolate host polling behavior from firmware behavior.

Variables:
- V1: host application (pyserial, mavproxy, minicom, screen).
- V2: control-line strategy (default, DTR high, DTR toggle, RTS toggle).
- V3: device node strategy (/dev/ttyACM*, /dev/serial/by-id/*).
- V4: capture type (none, usbmon, register snapshots).

Core experiment set:

Experiment E01:
- V1 pyserial, V2 default, V3 by-id, V4 usbmon.
- Goal: baseline IN request cadence.

Experiment E02:
- V1 pyserial, V2 explicit DTR true before first read, V3 by-id, V4 usbmon.
- Goal: detect DTR dependency.

Experiment E03:
- V1 mavproxy setup, V2 implicit line handling, V3 by-id, V4 usbmon.
- Goal: compare application-specific behavior.

Experiment E04:
- V1 minicom or screen, V2 default terminal behavior, V3 by-id, V4 usbmon.
- Goal: test terminal stack behavior differences.

Experiment E05:
- Repeat E01 to E04 while sampling endpoint and queue registers every 500 ms.
- Goal: correlate host trace with device queue retirement in near-real time.

Success criterion for experiment family:
- At least one configuration must produce sustained IN completion and nonzero host payload if host stack behavior is variable rather than universally broken.

---

## 42. Artifact Pack Specification

To make future reviews deterministic, all evidence should be exported as a structured artifact pack.

Recommended structure:

artifact_pack/
- metadata.txt
- host/
   - lsusb_v.txt
   - dmesg_tail.txt
   - tty_nodes.txt
- usbmon/
   - raw_5s.txt
   - parsed_ep81.csv
- target/
   - reg_snapshots.csv
   - queue_samples.csv
   - counters_samples.csv
- app/
   - pyserial_run.log
   - mavproxy_run.log
   - terminal_run.log

Minimum metadata fields:
- firmware build id
- openocd version
- host kernel version
- devnum and bus id
- exact command lines used

Review requirement:
- Any root-cause claim should reference at least one artifact file and one independent corroborating artifact.

---

## 43. Suggested USBMON Parsing Method

Goal:
- Convert raw usbmon text into endpoint-centric event timeline for ACM data endpoint.

Parsing dimensions:
- timestamp
- transfer type and stage (submit/complete)
- endpoint address
- device address
- status code
- transfer length

Derived metrics:
- IN submit rate to endpoint 0x81 (events per second)
- IN completion rate to endpoint 0x81
- completion success ratio
- average inter-arrival gap for IN submits

Interpretation rules:
- submit rate near zero while port is open indicates host polling starvation.
- submit rate nonzero with completion failures indicates transport/controller mismatch.
- submit and completion both healthy with no user payload implies userspace parsing/filter issue.

---

## 44. Register Sampling Cadence Recommendation

Single snapshots are useful but may miss transient behavior.

Recommended cadence:
- 2 Hz for broad state coverage over 30 to 60 seconds.
- 10 Hz for short 5-second bursts during host open transitions.

Sample fields each interval:
- _total_written
- SDU1.obqueue.bcounter
- USBD1.transmitting
- EP1 IN BUF_CTRL
- BUFSTATUS
- SIESTATUS

Use case:
- Build time-series plots that align queue stalls to host trace events.

---

## 45. Interface Enumeration Drift Controls

Because ttyACM indices may rotate after reconnects, all scripted tests should default to by-id path.

Controls:
- Resolve /dev/serial/by-id target once per test run.
- Abort test if symlink missing.
- Record resolved /dev/ttyACM* at test start.
- Re-resolve after any disconnect event.

Failure mode avoided:
- False diagnosis caused by reading inactive ACM node.

---

## 46. Binary Payload Parsing Caveat

MAVLink traffic is binary and may be invisible to text-only grep pipelines.

Implications:
- No human-readable lines does not imply no traffic.
- Hex-dump or byte-count checks are required before concluding silence.

Recommended practice:
- Always include a raw byte-count counter in host tests.
- Treat textual heartbeat pattern matching as secondary confirmation.

---

## 47. Differential Diagnosis: SERIAL0 vs SERIAL1/SERIAL2

Reading _total_written across multiple serial driver instances can isolate whether the issue is USB-specific.

Interpretive patterns:
- If SERIAL1/2 continue advancing while SERIAL0 plateaus, issue likely localized to USB path.
- If all ports plateau simultaneously, issue may involve higher-level MAVLink production.

Current incident indications:
- USB path behavior shows queue saturation signature; broader production collapse is not primary hypothesis.

---

## 48. Firmware Instrumentation Blueprint (Temporary)

For deterministic debugging, add temporary counters in USB path guarded by debug macros.

Counter set proposal:
- usb_tx_attempts
- usb_tx_bytes_requested
- usb_tx_bytes_accepted
- usb_tx_zero_returns
- usb_tx_poll_calls
- usb_tx_poll_success
- usb_tx_data_transmitted_callbacks
- usb_tx_queue_full_events

Sampling plan:
- Read counters through GDB every second.
- Correlate with host read windows and usbmon events.

Removal plan:
- Remove temporary counters once root cause confirmed and fix validated.

---

## 49. Validation Plan For Any Candidate Fix

A candidate fix must pass all stages below:

Stage 1: Functional smoke test
- Host receives nonzero bytes in first 10 seconds after open.

Stage 2: Stability
- Continuous read for 5 minutes without sustained zero-byte phase longer than 2 seconds.

Stage 3: Reconnect robustness
- Pass 10 consecutive disconnect/reconnect cycles.

Stage 4: Application compatibility
- Pass on pyserial, mavproxy, and one terminal app.

Stage 5: Regression check
- Ensure no negative impact on non-USB serial paths.

Stage 6: Evidence bundle
- Publish logs and counters proving endpoint completions progress.

---

## 50. Documentation Hygiene And Traceability Rules

To keep this incident history auditable:
- Record exact commands and command outputs, not paraphrases only.
- Separate observed facts from interpretations explicitly.
- Mark unverified assumptions with a clear tag.
- Update confidence scores when new evidence arrives.
- Keep baseline and patched firmware observations side by side.

Traceability tags recommended:
- FACT
- INFERENCE
- HYPOTHESIS
- REJECTED
- CONFIRMED

---

## 51. Advanced Open Questions

These questions should be considered active until resolved by artifact-backed evidence:

Q1:
- Does Linux cdc_acm issue IN submits continuously for this device after open in each tested application?

Q2:
- Is there any endpoint descriptor nuance that changes host polling behavior for this VID:PID profile?

Q3:
- Are control-line requests arriving in the expected sequence relative to first host read?

Q4:
- Is endpoint completion interrupt suppressed under any specific transient state despite armed buffers?

Q5:
- Could there be a timing window where endpoint is armed before host-side state machine is ready, then never retried correctly?

These are not blockers for current diagnosis quality but are required for final root-cause closure.

---

## 52. Extended Executive Conclusion

The cumulative evidence strongly supports a downstream transfer retirement failure rather than upstream payload generation failure. The firmware is active, data generation occurs, queueing occurs, and endpoint preparation occurs. The observable collapse happens at or after host polling/completion stage, leading to queue exhaustion and immediate write rejection behavior.

This incident therefore sits at the boundary between device endpoint completion flow and host ACM polling behavior. Final attribution requires endpoint-level host traffic proof, but the current data already constrains the plausible root causes to a narrow branch with high diagnostic confidence.

---

## 53. Extreme Annex: End-To-End Incident Execution Playbook

This annex turns the report into an execution manual. It is intentionally prescriptive so any engineer can reproduce, validate, and extend the investigation without ambiguity.

### 53.1 Mission objective

Prove, with correlated host and target evidence, which of these is true:
- Host does not schedule IN for endpoint 0x81.
- Host schedules IN but completions fail.
- Completions occur but userspace path drops or hides payload.

### 53.2 Mandatory evidence principle

Every conclusion must reference:
- One host artifact.
- One target artifact.
- One correlation statement linking them by time.

---

## 54. Canonical Data Dictionary

Use the exact field names below across scripts and logs.

### 54.1 Target-side fields

- ts_ms
- serial0_total_written
- serial1_total_written
- serial2_total_written
- sdu_obqueue_bn
- sdu_obqueue_bcounter
- usbd_state
- usbd_transmitting
- ep1_in_buf_ctrl
- usb_bufstatus
- usb_siestatus
- usb_inte

### 54.2 Host-side fields

- ts_ms
- app_name
- app_pid
- tty_path
- bytes_read
- read_calls
- dtr_state
- rts_state
- usbmon_ep
- usbmon_dir
- usbmon_stage
- usbmon_status
- usbmon_len

### 54.3 Correlation fields

- run_id
- devnum
- busnum
- firmware_id
- kernel_release

---

## 55. Ground-Truth Sampling Contract

Sampling intervals and durations are fixed for comparability.

### 55.1 Durations

- Warmup: 5 seconds.
- Capture window: 60 seconds.
- Cooldown: 5 seconds.

### 55.2 Target sampling

- Base cadence: 2 Hz for full 60 seconds.
- Burst cadence: 10 Hz during first 10 seconds after host port open.

### 55.3 Host sampling

- Byte counters: every read call.
- usbmon parse flush: every 1 second batch.

Outcome:
- Deterministic time-series suitable for cross-run diffing.

---

## 56. Runbook A: Baseline Reproduction Pack

Perform this runbook first in every new session.

### 56.1 Preparation

1. Confirm OpenOCD process is healthy and attached.
2. Resolve stable by-id ACM path.
3. Record firmware id and branch state.

### 56.2 Host check

1. Open by-id port with pyserial for 60 seconds.
2. Record total bytes and per-read byte counts.
3. Repeat once with mavproxy.

### 56.3 Target check

1. Collect 2 Hz register/counter snapshots for same 60-second window.
2. Collect 10 Hz snapshots for first 10 seconds.

### 56.4 Expected failure signature

- bytes_read sum remains near zero.
- serial0_total_written grows then plateaus.
- sdu_obqueue_bcounter reaches zero and remains pinned.
- ep1_in_buf_ctrl remains armed state.
- usb_bufstatus indicates no retirement progression.

---

## 57. Runbook B: Control-Line Sensitivity Campaign

Goal:
- Test whether line state transitions alter host polling behavior.

Execution matrix:

Case B1:
- DTR false, RTS default.

Case B2:
- DTR true before first read.

Case B3:
- DTR low-to-high toggle after open.

Case B4:
- RTS low/high variants with fixed DTR true.

For each case:
- Run 30-second host read.
- Run synchronized target snapshots.
- Capture usbmon stream.

Decision rule:
- If any case shows sustained endpoint retirement and bytes, line-state dependency is elevated.

---

## 58. Runbook C: Application Stack Differential

Goal:
- Determine if polling differs by application stack.

Applications:
- pyserial
- mavproxy
- minicom
- screen

Per app protocol:
- 30-second open/read window.
- usbmon capture active.
- target snapshots active.

Interpretation:
- If only one app polls properly, failure may sit in app-open semantics, not kernel-wide behavior.
- If none poll, focus shifts to cdc_acm or descriptor/control interplay.

---

## 59. Runbook D: Endpoint Descriptor Integrity Audit

Goal:
- Eliminate descriptor mismatch as hidden cause.

Required outputs:
- Full lsusb -v dump for target device.
- Parsed interfaces, endpoint numbers, max packet sizes, attributes.

Audit checks:
- Data interface includes BULK IN endpoint 0x81 and BULK OUT endpoint expected by ACM profile.
- Max packet size and interval values are sane for full-speed bulk.
- Interface association and CDC functional descriptors are coherent.

If descriptor anomalies are found:
- Elevate C3 branch in failure tree.

---

## 60. Runbook E: Continuous Soak And Recovery Test

Goal:
- Verify whether failure is immediate, delayed, or phase-dependent.

Procedure:
- Keep host reader open for 10 minutes.
- Record per-second bytes.
- Capture target counters every second.
- Capture usbmon every second.

Recovery stimuli at minute 5:
- DTR toggle.
- Close/reopen reader.

Observe:
- Whether queue retirement restarts spontaneously or by stimulus.

---

## 61. Unified CSV Schemas

### 61.1 target_samples.csv

```csv
run_id,ts_ms,serial0_total_written,serial1_total_written,serial2_total_written,sdu_obqueue_bn,sdu_obqueue_bcounter,usbd_state,usbd_transmitting,ep1_in_buf_ctrl,usb_bufstatus,usb_siestatus,usb_inte
```

### 61.2 host_reads.csv

```csv
run_id,ts_ms,app_name,tty_path,read_calls,bytes_read,dtr_state,rts_state
```

### 61.3 usbmon_ep81.csv

```csv
run_id,ts_ms,devnum,endpoint,dir,stage,status,length
```

### 61.4 correlation_summary.csv

```csv
run_id,window_start_ms,window_end_ms,host_bytes_total,ep81_submit_count,ep81_complete_count,serial0_delta,obqueue_zero_ratio
```

---

## 62. Statistical Decision Rules

These rules convert traces into objective incident outcomes.

Rule S1:
- If ep81_submit_count equals 0 while port is open, classify as HOST_POLL_STARVATION.

Rule S2:
- If ep81_submit_count greater than 0 and ep81_complete_count equals 0, classify as TRANSFER_COMPLETION_FAILURE.

Rule S3:
- If ep81_complete_count greater than 0 and host_bytes_total equals 0, classify as USERSPACE_DRAIN_OR_PARSE_FAILURE.

Rule S4:
- If serial0_delta equals 0 while serial1_delta is healthy, classify as USB_PATH_LOCALIZED_STALL.

Rule S5:
- If obqueue_zero_ratio greater than 0.9 for window with serial0_delta plateau, classify as QUEUE_SATURATION_LOCK.

---

## 63. Correlation Algorithm

Use this sequence to correlate target and host events.

1. Normalize all timestamps to monotonic ms from run start.
2. Join host_reads and target_samples by nearest timestamp within 100 ms.
3. Aggregate usbmon events in 1-second buckets.
4. For each bucket compute:
- host_bytes_total
- serial0_delta
- ep81_submit_count
- ep81_complete_count
- obqueue_zero_ratio
5. Label each bucket using rules S1 through S5.
6. Emit dominant label and confidence for each 10-second epoch.

---

## 64. Confidence Computation Method

Confidence score in range 0 to 100:

- +30 if independent host and target artifacts agree on same class.
- +20 if class persists across at least 3 contiguous buckets.
- +20 if reproduced in at least 2 applications.
- +10 if reproduced after reconnect cycle.
- +10 if reproduced after control-line variation.
- +10 if no contradictory bucket exceeds 20 percent of window.

Interpretation:
- 80 to 100 high confidence.
- 60 to 79 medium confidence.
- below 60 provisional.

---

## 65. Incident Labels And Meanings

Standard labels to avoid wording drift.

- HOST_POLL_STARVATION
- TRANSFER_COMPLETION_FAILURE
- USERSPACE_DRAIN_OR_PARSE_FAILURE
- USB_PATH_LOCALIZED_STALL
- QUEUE_SATURATION_LOCK
- INSUFFICIENT_EVIDENCE

Each run must end with exactly one primary label and optional secondary label.

---

## 66. Strict Evidence Review Checklist

A reviewer signs off only if all checks pass.

Checklist:
- by-id node used and recorded.
- tty index drift accounted for.
- target snapshots include all canonical fields.
- usbmon parsed trace includes endpoint 0x81 rows or explicit absence proof.
- at least one 60-second run captured.
- at least two applications tested.
- line-state variation tested.
- correlation_summary generated.
- final label assigned by rules, not intuition only.

---

## 67. Extreme Operator Commands (Ready To Execute)

The commands below are templates intended to be copied into scripts. They are provided as report material for deterministic execution.

### 67.1 Host read logger template

```python
#!/usr/bin/env python3
import csv, os, time, serial

RUN_ID = os.environ.get("RUN_ID", "run_default")
PORT = os.environ.get("PORT", "/dev/serial/by-id/usb-ArduPilot_Pico2_9EE4ECE8FA06028D6EAD2FF9-if00")
OUT = os.environ.get("OUT", "host_reads.csv")
APP = os.environ.get("APP", "pyserial")
DUR = float(os.environ.get("DUR", "60"))

t0 = time.monotonic()
with serial.Serial(PORT, 115200, timeout=0.1) as s, open(OUT, "w", newline="") as f:
   w = csv.writer(f)
   w.writerow(["run_id","ts_ms","app_name","tty_path","read_calls","bytes_read","dtr_state","rts_state"])
   calls = 0
   while (time.monotonic() - t0) < DUR:
      d = s.read(256)
      calls += 1
      w.writerow([RUN_ID, int((time.monotonic()-t0)*1000), APP, PORT, calls, len(d), int(s.dtr), int(s.rts)])
```

### 67.2 Target sample logger template

```python
#!/usr/bin/env python3
import csv, socket, time, os

RUN_ID = os.environ.get("RUN_ID", "run_default")
OUT = os.environ.get("OUT", "target_samples.csv")
PORT = int(os.environ.get("OCD_PORT", "50012"))
DUR = float(os.environ.get("DUR", "60"))
HZ = float(os.environ.get("HZ", "2"))

def ocd(cmd, wait=0.12):
   s = socket.socket(); s.connect(("127.0.0.1", PORT)); s.settimeout(1.0)
   time.sleep(0.05)
   try: s.recv(4096)
   except Exception: pass
   s.sendall((cmd + "\n").encode())
   time.sleep(wait)
   b = b""
   while True:
      try: b += s.recv(4096)
      except Exception: break
   s.close()
   return b.decode(errors="replace")

def parse_last_hex_word(txt):
   words = [x for x in txt.replace(":"," ").split() if len(x)==8 and all(c in "0123456789abcdefABCDEF" for c in x)]
   return int(words[-1], 16) if words else 0

t0 = time.monotonic()
dt = 1.0 / HZ

with open(OUT, "w", newline="") as f:
   w = csv.writer(f)
   w.writerow(["run_id","ts_ms","serial0_total_written","serial1_total_written","serial2_total_written","sdu_obqueue_bn","sdu_obqueue_bcounter","usbd_state","usbd_transmitting","ep1_in_buf_ctrl","usb_bufstatus","usb_siestatus","usb_inte"])
   while (time.monotonic() - t0) < DUR:
      ocd("halt", wait=0.03)
      s0 = parse_last_hex_word(ocd("mdw 0x200129fc 1", wait=0.03))
      s1 = parse_last_hex_word(ocd("mdw 0x20012b74 1", wait=0.03))
      s2 = parse_last_hex_word(ocd("mdw 0x20012cec 1", wait=0.03))
      bn = parse_last_hex_word(ocd("mdw 0x20019170 1", wait=0.03))
      bc = parse_last_hex_word(ocd("mdw 0x20019174 1", wait=0.03))
      us = parse_last_hex_word(ocd("mdw 0x20019900 1", wait=0.03))
      ut = parse_last_hex_word(ocd("mdw 0x20019928 1", wait=0.03))
      ep = parse_last_hex_word(ocd("mdw 0x50100088 1", wait=0.03))
      bs = parse_last_hex_word(ocd("mdw 0x50110058 1", wait=0.03))
      ss = parse_last_hex_word(ocd("mdw 0x50110050 1", wait=0.03))
      ie = parse_last_hex_word(ocd("mdw 0x50110090 1", wait=0.03))
      ocd("resume", wait=0.02)
      w.writerow([RUN_ID, int((time.monotonic()-t0)*1000), s0, s1, s2, bn, bc, us, ut, ep, bs, ss, ie])
      time.sleep(dt)
```

### 67.3 usbmon parser scaffold

```python
#!/usr/bin/env python3
import csv, re, sys, time

RUN_ID = "run_default"
DEVNUM = int(sys.argv[1])
INFILE = sys.argv[2]
OUTFILE = sys.argv[3]

pat = re.compile(r"^\s*([0-9a-f]+)\s+\S+\s+([SC])\s+\w+:(\d+):(\d+)\s+\S+\s+(\S+)\s+(\d+)")

with open(INFILE, "r", errors="replace") as f, open(OUTFILE, "w", newline="") as o:
   w = csv.writer(o)
   w.writerow(["run_id","ts_ms","devnum","endpoint","dir","stage","status","length"])
   t0 = time.monotonic()
   for line in f:
      m = pat.match(line)
      if not m:
         continue
      stage = m.group(2)
      dev = int(m.group(3))
      ep = int(m.group(4))
      status = m.group(5)
      ln = int(m.group(6))
      if dev != DEVNUM:
         continue
      direction = "IN" if (ep & 0x80) else "OUT"
      w.writerow([RUN_ID, int((time.monotonic()-t0)*1000), dev, ep, direction, stage, status, ln])
```

---

## 68. Correlation Report Template

Each run should produce a human-readable summary using the exact headings below.

Template:

1. Run metadata
- run_id
- firmware_id
- app_name
- devnum
- tty_path

2. Host outcome
- host_bytes_total
- reads_with_nonzero_bytes

3. Target outcome
- serial0_delta
- obqueue_zero_ratio
- ep1_armed_ratio

4. usbmon outcome
- ep81_submit_count
- ep81_complete_count
- dominant_status

5. Classification
- primary_label
- secondary_label
- confidence_score

6. Evidence references
- host file path
- target file path
- usbmon file path

---

## 69. Extreme Handoff Protocol

When handing this incident to another engineer, transfer all of the following:
- Report file itself.
- Latest artifact_pack directory.
- One successful runbook execution transcript.
- One failed runbook execution transcript.
- Explicit list of unresolved questions from section 51.

Handoff acceptance criteria:
- Receiver can reproduce classification on same hardware within one hour.

---

## 70. Final Operational Conclusion For Current State

At current evidence depth, the system has moved from anecdotal failure to analytically constrained failure domain. The remaining uncertainty is no longer about whether data is generated or queued on target, but about exact host-transaction behavior at endpoint-level granularity and completion propagation.

This is a strong position for final root-cause closure: the next step is no longer broad debugging, but targeted confirmation of IN submit and completion behavior under controlled application and line-state conditions.
