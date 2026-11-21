# Worldwide flight-controller firmware research — independent design charter

**Date:** 2026-11-21  
**Scope:** Original, hardware-independent flight-controller engineering.  
**Project branch:** `development/betaflight-full-compatibility`

## Change of direction

The owner does **not** want an imported Betaflight/PX4/ArduPilot codebase or a
repackaged copy of an existing flight stack. The experimental upstream
Betaflight *git submodule* from the preceding step has therefore been removed.

We will examine many existing flight-control systems as **engineering
references** and make independently justified implementation decisions for
this project. There is **no chosen upstream firmware as the foundation**.

**Honesty:** The repository already starts from rtlopez's MIT-licensed
ESP-FC project and contains existing third-party integrations; declaring the
*existing repository* entirely original would be inaccurate. A future
independent implementation requires a code-provenance/license audit and,
where needed, original replacement modules. Research alone does not erase
the history of the current fork.

## Source coverage, without falsely claiming exhaustiveness

The companion `firmware-research-catalog.csv` identifies **31**
distinct named projects or families. It spans multi-rotor racing firmware,
navigation/autopilots, helicopter-specific controllers, small-whoop systems,
ESP32 implementations, historical platforms, research firmware, and
commercial products with limited public implementation details.

**Status:** `source-identified-only` means we found a primary project/source
location. It does **not** mean its entire codebase has been read, every
feature verified, current hardware support audited, or the project
bench-tested. These tasks remain open. No one can credibly claim to
enumerate every firmware ever built worldwide, especially inaccessible,
private, lost, and unpublished implementations.

Projects are not assumed independent: several are forks/derivatives.
Product firmware, boards, configurators, companion computers, receivers,
ESC firmware, and flight-controller firmware are distinct objects.
Don't count a board or a receiver protocol as a new flight stack.

## Learning map: research **by subsystem**, not by brand

| Research area | Priority sources | What we must evaluate |
|---|---|---|
| Real-time gyro/attitude loops | Betaflight, Quicksilver, EmuFlight, ESP-FC | Scheduling jitter, DMA/interrupts, filter latency, aliasing, loop overrun response |
| Navigation and autonomous safety | ArduPilot, PX4, INAV, Paparazzi | EKF health, GPS loss, geofence, mode transitions, RTL/LAND, estimator fallback |
| Lightweight/ESP architecture | ESP-Drone, ESP-FC, Crazyflie, Silverware, Yufeng | Memory footprint, CPU isolation, sensor bus performance, power stability |
| Helicopter/fixed-wing specialization | Rotorflight, ArduPilot Plane, INAV, MatrixPilot | Vehicle-specific actuators, mixers and controllers |
| Protocols and configurator | Betaflight MSP, MAVLink, PX4, INAV, LibrePilot | Typed parameter schema, capabilities, persistence, error reporting, compatibility |
| Test and diagnostics | PX4 SITL/HITL, ArduPilot SITL/autotest, ROSflight | Fault injection, regression gates, telemetry, Blackbox/flight logs |
| History and design alternatives | MultiWii, Baseflight, Cleanflight, dRonin, Tau Labs, AeroQuad, AutoQuad | Design trade-offs, regressions, maintainability |
| Closed implementations | KISS, FlightOne | Publicly observable behavior only; do not assert knowledge of proprietary source |

## Cross-check sources and standards

- Research survey: https://www.mdpi.com/2079-9292/13/23/4785
- Research project directory: https://github.com/C-PANATORN/Open-UAV-Platforms
- MAVLink microservices: https://mavlink.io/en/services/index.html
- Betaflight MSP reference: https://betaflight.com/docs/development/MSP-Protocol-Reference-Dev
- PX4 architecture: https://docs.px4.io/main/en/concept/architecture
- PX4 hardware simulation: https://docs.px4.io/main/en/simulation/hardware
- ArduPilot SITL test methods: https://ardupilot.org/dev/docs/using-sitl-for-ardupilot-testing.html
- DroneCAN peripherals: https://docs.ardupilot.org/plane/docs/common-uavcan-peripherals.html
- Betaflight hardware support: https://betaflight.com/docs/wiki/getting-started/hardware-support

## Worldwide discovery method

1. Expand from primary projects into dependency graphs, forks,
   releases, archived projects, academic papers and vendor manuals.
2. Search multiple regional languages and local code forges, including
   ESP32/STM32 designs that are not indexed by English-only terms.
3. Deduplicate derivatives and record explicit upstream relationships.
4. Record source language, maintained/archived status, documentation,
   hardware list, licensing and tests **only after verifying them**.
5. For every claimed feature, trace its code path and actual driver/
   estimator/controller effect. Use issue trackers and release notes to
   identify known limitations.
6. Keep a dated evidence trail with pinned version/commit, file paths,
   source links, and reproducible tests. Revisit changing projects.

This is a **repeatable research method**, not a claim that all possible
languages, countries or closed firmware have already been searched.

## Architectural commitments and acceptance gates

- Platform-independent flight logic and board-specific HAL/targets;
  never hardcode one MPU, receiver protocol, motor layout, or ESP32 board.
- Capabilities are declared by **actual target implementation and tests**,
  not because a configurable UI element exists.
- Configuration path: input -> validation -> apply -> runtime impact ->
  readback -> durable storage -> power-cycle verification.
- Sensor integrity: timeouts, calibrated units, orientation, fresh samples,
  bus recovery and estimator confidence must be explicit.
- Controller invariants: bounded outputs, finite inputs, anti-windup,
  deterministic timing and safe handovers between modes.
- Safety: priority of attitude stabilization, arming gates, receiver-loss,
  GPS-loss, battery and CPU overload scenarios; controlled landing only
  when verified sensors, controllers and actuators can support it.
- Support levels: `researched`, `driver-present`, `compiled`,
  `SITL-tested`, `bench-validated`, `flight-validated`.
  Never equate one level with the next.
- New features require native tests, simulator tests and physical validation
  where hardware effects cannot be proven by software alone.

## License discipline

- Existing ESP-FC repository root uses MIT; included components may carry
  separate licenses and must be audited individually.
- Betaflight and ArduPilot are GPL-family projects; PX4 uses BSD-3-Clause.
  Do not paste GPL-covered implementations into MIT-labeled code and
  claim the result remains purely MIT.
- Reading architectural ideas/specifications is different from copying code.
  Keep code provenance records and get qualified licensing review for any
  actual imports, especially proprietary materials.

## Next **single** engineering milestone

Create a verifiable **hardware capability and configuration contract**
in our own codebase before adding new hardware:
a board descriptor for MCU/pins/buses/timers/motor outputs/sensors,
typed capability responses, and readback/persistence regression tests.
Keep all legacy firmware behavior unchanged until the new contract passes.
