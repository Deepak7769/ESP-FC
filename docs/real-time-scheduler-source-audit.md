# Real-time flight-control scheduling: cross-firmware source audit (RTA-001)

**Audit date:** 2026-11-21  
**Scope:** One subsystem only — gyro sampling, control-loop scheduling, actuator cadence, background work and overload instrumentation.  
**Status:** `SOURCE-REVIEWED / DESIGN PROPOSED / NOT IMPLEMENTED`  
**Safety:** No flight behavior changed by this audit. Existing build/test success is not proof of flight safety.

## 1. Primary-source register (pinned versions)

This review inspected the *actual source* of five external firmware families, and
our current repository. These are selected representative designs, **not all
flight-controller firmware worldwide**. References are pinned so line numbers
and behavior can be independently reproduced.

| Firmware | Pinned Git revision | Inspected implementation |
|---|---|---|
| Our ESP-FC development | `b9d59cb5788f0b88457943c59a69530ed446e1df` | [main.cpp](https://github.com/Deepak7769/ESP-FC/blob/b9d59cb5788f0b88457943c59a69530ed446e1df/src/main.cpp#L24-L165), [Espfc.cpp](https://github.com/Deepak7769/ESP-FC/blob/b9d59cb5788f0b88457943c59a69530ed446e1df/lib/Espfc/src/Espfc.cpp#L47-L228), [Timer.cpp](https://github.com/Deepak7769/ESP-FC/blob/b9d59cb5788f0b88457943c59a69530ed446e1df/lib/Espfc/src/Utils/Timer.cpp#L90-L132), [Model.h](https://github.com/Deepak7769/ESP-FC/blob/b9d59cb5788f0b88457943c59a69530ed446e1df/lib/Espfc/src/Model.h#L1119-L1201), [Stats.cpp](https://github.com/Deepak7769/ESP-FC/blob/b9d59cb5788f0b88457943c59a69530ed446e1df/lib/Espfc/src/Utils/Stats.cpp#L23-L144) |
| Betaflight | `1b94a3871529bbb68d8dbb7d33a99d1101d3f2d4` | [task registry](https://github.com/betaflight/betaflight/blob/1b94a3871529bbb68d8dbb7d33a99d1101d3f2d4/src/main/fc/tasks.c#L391-L540), [scheduler](https://github.com/betaflight/betaflight/blob/1b94a3871529bbb68d8dbb7d33a99d1101d3f2d4/src/main/scheduler/scheduler.c#L526-L586), [background budget](https://github.com/betaflight/betaflight/blob/1b94a3871529bbb68d8dbb7d33a99d1101d3f2d4/src/main/scheduler/scheduler.c#L735-L813) |
| ArduPilot | `dc9c4e25eab47e4df7ce431f6837e6e1d10949de` | [AP_Scheduler.cpp](https://github.com/ArduPilot/ardupilot/blob/dc9c4e25eab47e4df7ce431f6837e6e1d10949de/libraries/AP_Scheduler/AP_Scheduler.cpp#L191-L310) |
| PX4 | `0e0f3ce8a241441de5aa59f479cb49b2d8652f5d` | [MC rate-control callback](https://github.com/PX4/PX4-Autopilot/blob/0e0f3ce8a241441de5aa59f479cb49b2d8652f5d/src/modules/mc_rate_control/MulticopterRateControl.cpp#L64-L145), [work-item scheduling interface](https://github.com/PX4/PX4-Autopilot/blob/0e0f3ce8a241441de5aa59f479cb49b2d8652f5d/platforms/common/include/px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp#L43-L85) |
| INAV | `21adf64440bb0d9f0bdc56569098fbaa0e39d24d` | [task registry](https://github.com/iNavFlight/inav/blob/21adf64440bb0d9f0bdc56569098fbaa0e39d24d/src/main/fc/fc_tasks.c#L528-L592), [scheduler](https://github.com/iNavFlight/inav/blob/21adf64440bb0d9f0bdc56569098fbaa0e39d24d/src/main/scheduler/scheduler.c#L237-L352) |
| Quicksilver | `d75b85d9d7cbfc92be0a6bad7784834454e84c41` | [priority queue and time budget](https://github.com/BossHobby/QUICKSILVER/blob/d75b85d9d7cbfc92be0a6bad7784834454e84c41/src/core/scheduler.c#L31-L146) |

**Version caution:** These are revisions retrieved on the audit date, not
necessarily the official latest stable release of each project. Findings apply
to the cited files at the cited revisions. We did not benchmark these
platforms or validate their flight behavior.

## 2. Observed design patterns (source evidence vs. proposed changes)

| System | Verified scheduling mechanism | Strong point to study | Important boundary |
|---|---|---|---|
| Betaflight | Gyro, filtering and PID tasks have real-time priority. The scheduler tracks an intended gyro boundary, catches up by full cycles on gross overruns, and runs lower-priority tasks opportunistically when there is time. | Explicit timing boundary and budget for best-effort work. | Its precise gyro-aware timing and peripheral support are platform-specific; do not copy GPL code. |
| ArduPilot | Priority-ordered fast-loop and scheduled tasks; lower-priority tasks have expected duration budgets, task slips and overruns are recorded. | Task accounting, rate flexibility, and vehicle-class adaptability. | Fast tasks may still overrun. Broad architecture has different target constraints than a lightweight ESP32 controller. |
| PX4 | Multicopter rate controller registers a callback for angular-velocity updates, uses the sample timestamp to compute bounded `dt`; other activities can run on scheduled work queues. | Measurement-driven execution and explicit timestamps between modules. | Work queue execution has operating-system overhead; no guarantee of universal 4-kHz rate on all targets. |
| INAV | Priority-based cooperative scheduler, overdue real-time tasks, age-based lower-priority selection, execution-time statistics; SITL-specific sleeping. | Clear task aging and real-time precedence with navigation workload. | Cooperative tasks can still block later tasks if they overrun. |
| Quicksilver | Sorted priority task queue; non-real-time tasks denied when estimated worst-case duration exceeds remaining loop time; task execution statistics. | Compact budget admission suitable for resource-constrained targets. | Worst-case estimates are adaptive; they are not mathematically guaranteed WCET bounds. |
| Current ESP-FC | ESP32 multicore: timer ISR notifies a high-priority gyro task; another task drains application queue. Other targets use periodic timer checks. Gyro, control, mixer and safety cadence are set by dividers. | Existing ISR-to-task separation, stale-sample gating and decoupled failsafe servicing. | No evidence yet that all tasks meet deadlines under load; noncritical operations remain inside the high-priority update call. |

These are observations about cited source, not claims that a complete
firmware has been vetted or universally outperforms the others.

## 3. Source-confirmed findings in **our** firmware

### F-RT-01 — Timer overrun counts exist but lack a verified reporting/test path

`src/main.cpp:39,97-120` increments `gyroTaskMissedDeadlines` if multiple
timer notifications accumulated before a high-priority task wakes. This
is useful but currently insufficient as proof of timely control: we need
sample-to-output latency, overrun duration, missed-sample counts,
and telemetry/readback with tests. An increment records delayed notification
consumption, **not** necessarily a measured IMU or motor deadline miss.

**Proposal:** expose independent, resettable and saturating counters for
ISR releases, missed releases, sensor timestamps, control completion, actuator
commit, and per-task overruns.

### F-RT-02 — High-priority flight update still contains background operations

The ESP32 gyro task calls `espfc.update(true)` from
`src/main.cpp:97-120`. Even in its multicore branch,
`Espfc.cpp:198-205` calls serial, buzzer, LED and statistics work
at the end of that same update. `Espfc::updateOther()` at
`Espfc.cpp:211-225` merely drains a queue; that does not establish
that every non-critical subsystem has been isolated on the other core.

**Risk (not yet benchmarked):** serial bursts or maintenance tasks could
inflate worst-case sample-to-output latency despite acceptable *average*
reported CPU usage.

**Proposal:** instrument before relocating work. Any move to another core
must preserve MSP/CLI synchronization, thread safety, arming-state
transitions and configuration persistence.

### F-RT-03 — Software timer re-anchors to *actual* execution time

`Utils/Timer.cpp:101-130` performs `next = now + interval`.
After a delayed task, the next software release is based on completion
time rather than the original phase. This can accumulate timing drift
for software-periodic tasks. The gyro ISR path is a **separate mechanism**,
so this observation must not be incorrectly reported as proof of gyro
timer drift on ESP32.

**Proposal:** for tasks requiring a stable phase, maintain
`next_deadline += k * interval` with explicitly bounded missed-release
handling. Keep phase-free event-driven tasks separate. Add wraparound tests.

### F-RT-04 — Periods are configurable and hardware dependent

`Model.h:1119-1201` sets gyro rate from detected device/bus,
`loopTimer` from `config.loopSync`, mixer from `config.mixerSync`,
input roughly aligned to a maximum of 1000 Hz, and actuator supervision at
50 Hz. `Model.h:1008-1054` imposes protocol-specific motor-output
limits, including sync PWM restrictions. This is **not** a universal
4-kHz gyro/PID + 400-Hz motor guarantee for every board/protocol.

**Proposal:** describe 4-kHz control / 400-Hz PWM as a requested **target
profile**, not as an unconditionally supported hardware capability.

### F-RT-05 — Current CPU-load summary cannot establish deadline safety

`Stats.cpp:126-142` combines `COUNTER_CPU_0` and `COUNTER_CPU_1`
with a weighted formula. `Stats.cpp:34-45` smooths loop time.
Neither value alone describes the maximum latency distribution of gyro,
control or motor output. **Average 43%, 73% or any other load is not
sufficient evidence that a 250-µs loop meets its deadline**.

**Proposal:** report CPU occupancy *per core* separately from timing
measurements. Include peak latency, histogram/percentile, skipped-release
counter and longest uninterrupted noncritical task. Any optional load
shedding must never suppress necessary attitude/rate control or essential
failsafe supervision.

### F-RT-06 — Sensor-failure handling is partly decoupled, but needs fault injection

`Espfc.cpp:132-196` advances receiver and actuator/failsafe supervision
even if a gyro transaction fails, and skips controller execution when its
sample is invalid. The multicore branch also gates controller work on
`SENSOR_READ_CONTROL`. This is a useful safety design, but no timing
guarantee is established until we inject delayed, missing and corrupted
samples on each target.

## 4. Our proposed original scheduler contract (DESIGN-RT-001)

We will not port another firmware's scheduling source.

**A. Three execution classes**

1. **Hard-priority control:** sensor acquisition, timestamp validation,
   flight state, attitude/rate/vertical control as needed, mixer and output
   safety. **Never shed the essential control chain.** Safety-disarm
   propagation must not wait for the next optional task.
2. **Bounded safety and navigation:** receiver/failsafe checks, estimator
   health, battery alarms, rescue and LAND handovers, navigation updates.
   These have guaranteed service cadence and bounded execution budget
   appropriate to the active mode, not optional shedding.
3. **Opportunistic services:** telemetry, configurable MSP polling,
   OSD, LED, nonessential logging and diagnostics. May be deferred,
   throttled or disabled when real-time slack is insufficient.

Note that a safety-critical class may still require multiple rates; one
global priority number cannot represent every necessary deadline.

**B. End-to-end control-loop pipeline**

`sample IRQ / timer release -> acquire -> validate timestamp -> filters ->
estimator -> controllers -> mixer -> output commit`.

Each stage records monotonic timestamps and a failure reason if skipped.
Controller `dt` comes from validated sample timestamps, with explicit
handling for duplicates, stale samples and unreasonable intervals.
Do **not** replay multiple controller iterations using one old gyro sample.

**C. Deadline behavior**

- ESP32 4-kHz profile: nominal control period `250 µs`.
- Conventional PWM 400-Hz profile: motor output period `2500 µs`;
  this is a *separate actuator cadence*, not a promise about latency to
  the most recent control sample.
- For a 4-kHz target, 85% of one *single-core period* is 212.5 µs and
  95% is 237.5 µs. These are **illustrative budget values only**;
  the user-requested 85%/95% CPU thresholds are CPU **load-policy**
  requirements, not per-release execution-time thresholds.
- Missed release: increment a counter; do not execute a backlog of
  stale control steps; validate newest sensor data and preserve safety
  supervision. A missed deadline is a measured error even if the
  next cycle recovers.
- Timer wraparound, reboot/reset, and clock-domain consistency must be
  tested explicitly.

**D. Overload policy (NOT YET IMPLEMENTED)**

- `CPU > 85%`: classify as emergency capacity and reduce
  **optional** activity, never drop attitude/rate, necessary vertical
  control or safety supervision. Require a defined observation window,
  hysteresis and per-core evidence before turning the threshold into an
  arming/flight decision.
- `CPU > 95%`: treat as a requested terminal LAND-failsafe policy,
  **but only if the LAND estimator and actuator chain remains healthy**.
  An unconditional LAND command with failed control-loop timing is
  *not a defensible safety guarantee*; define the state-dependent
  fallback through hazard analysis and fault-injection tests.
- Flight-phase priority changes should alter only budgets of
  nonessential tasks and safe navigation jobs. A navigation service
  needed for active Rescue must not be dropped by the service shedder.
- No mid-air motor-stop when valid automatic thrust control owns output.

**E. Hardware abstraction**

A board declares timer resolution, sensor data-ready support, bus latency,
interrupt/RTOS model, actuator protocol/cadence, available DMA and safe
worst-case constraints. The scheduler selects profiles only when their
prerequisites pass. **No hardcoded universal ESP32 / SPI / IBUS board.**

## 5. Verification gates before modifying runtime behavior

| Gate | Test and expected evidence | Status |
|---|---|---|
| RT-T01 | Native unit tests for phase-locked deadline arithmetic, wraparound, saturation and missed-release accounting | ISOLATED COMPONENT + NATIVE TESTS ADDED; CI and runtime integration tracked separately |
| RT-T02 | Instrument baseline flight pipeline with separate per-core CPU, loop jitter and sensor-to-motor latency | NOT STARTED |
| RT-T03 | Stress serial MSP bursts, configuration saves and logging; demonstrate worst-case flight-loop and failsafe servicing | NOT STARTED |
| RT-T04 | Inject gyro timeouts, duplicate timestamps, stalled IMU buses and delayed interrupts | NOT STARTED |
| RT-T05 | Validate independent 4-kHz control and 400-Hz PWM **on hardware that supports it** | NOT STARTED |
| RT-T06 | SITL/HITL: overload above 85% and above 95%, GPS failure, sensor loss, LAND handover, no motor-stop during valid vertical control | NOT STARTED |
| RT-T07 | Compare baseline vs. modified builds across available MCU targets; record target-by-target limitations | NOT STARTED |

**Update (2026-11-21):** RT-T01 now has an original header-only \`DeadlineTracker\` and isolated native tests at \`test/test_deadline/\`. The component is not connected to any runtime flight-control or motor path. Test pass/fail is determined by GitHub CI.\n\n**Explicit decision:** RTA-001 is a *research-only* milestone. We will not
rewrite the scheduler or claim performance improvements until baseline
instrumentation and its native tests are green. The next **one** change is
RT-T01: implement and test a small, target-independent deadline accounting
primitive, without connecting it to live motor outputs yet.

## 6. What this audit does **not** establish

- This is not an exhaustive audit of all worldwide firmware.
- No source was copied into our production code.
- No benchmark was run on the referenced third-party controllers.
- No 4-kHz or 400-Hz stability, load percentage, fault recovery or real
  flight safety claim has been established by this review.
- Compiler success does not mean a board's hardware support is verified.
