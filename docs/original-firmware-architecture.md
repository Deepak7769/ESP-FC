# Original flight-controller architecture — design hypotheses

This document defines the *proposed* direction and its verification gates.
It is **not** an implementation report.

## Layering

```text
Flight configurator / CLI / test harness
            |
      Schema + capability service <-- persistent versioned config
            |
   Flight state machine and safety manager
            |
Estimator --> navigation --> guidance --> attitude/rate/vertical controllers
    ^                                  |
    |                                  v
Sensor HAL / timestamping         Mixer / actuator safety
    ^                                  |
    +---- Board HAL / DMA / buses -----+--> ESC / servos
                  |
      Diagnostics / logging / timing
```

### Portability model

- **MCU family**: platform abstraction for clocks, DMA, interrupts,
  concurrency, storage, watchdog, USB/UART and timers.
- **Board target**: resource/pin map, voltage limitations, bus wiring,
  available timers, flash layout and hardware capabilities.
- **Device driver**: protocol and device ID probe, timing, calibration,
  resolution and health; do not infer presence from a compiled driver.
- **Vehicle definition**: mixer, output mapping, motor/servo behavior,
  actuator constraints, validated arming requirements.
- **Runtime feature**: versioned config + demonstrated implementation +
  prerequisites. Unsupported features return an explicit error.

### Safety-critical controls

- Do not disable attitude/rate control as CPU-shedding work.
- Keep automatic thrust ownership explicit during AltHold, Rescue and LAND;
  no opportunistic motor-stop of active vertical control.
- CPU emergency and LAND transitions are **requirements to verify**, not
  proven behavior in a multi-platform firmware.
- LAND touchdown and disarm conditions require a separately tested
  state machine with stale-sensor handling.
- GPS Rescue failing GPS-health checks should use a tested LAND handover
  if a safe altitude estimator remains; otherwise a separate hazard
  analysis is mandatory. Never claim unconditional safe landing without
  reliable measurements and control authority.

### Initial acceptance tests

1. Compile each declared target; check code size, RAM and watchdog.
2. Inject missing/incorrect IMU and serial receiver; no false readiness.
3. Test configuration roundtrip, invalid values, reboot, brownout and
   configuration migration.
4. Exercise GPIO/timer resource conflicts and unsupported features.
5. Measure 4-kHz control-loop deadline distribution separately from
   400-Hz PWM output, where the board supports those rates.
6. SITL failsafe cases: RX loss, GPS outage, barometer drift, IMU freezes,
   non-finite controller inputs and scheduler overruns.
7. Bench-test motor directions/mapping with propellers removed; escalate
   to restrained hardware testing only after analysis.
8. Record evidence per board; compilation alone is not hardware support.

## Feature completeness definition

A configurator control passes only if all applicable checks pass:
`visible or discoverable`, `request accepted`, `validated`, `readback`,
`saved`, `runtime effect`, `bench-tested where required`, and
`accurately unavailable on unsupported hardware`.
