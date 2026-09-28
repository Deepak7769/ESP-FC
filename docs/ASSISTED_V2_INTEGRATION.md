# Assisted V2 integration contract

This document defines the radio/channel contract and staged activation policy for
Angle V2, AltHold V2, and failsafe LAND V2.

## Channel contract

ESP-FC uses logical AETR input indices after receiver mapping:

| Logical index | Meaning | Assisted V2 use |
| ---: | --- | --- |
| 0 | Roll | normal pilot roll |
| 1 | Pitch | normal pilot pitch |
| 2 | Yaw | normal pilot yaw |
| 3 | Throttle | accumulated/manual throttle |
| 4 | AUX1 | arm/mode switch as configured |
| 5 | AUX2 | available/legacy project switch |
| 6 | AUX3 | **raw spring-centered vertical stick for AltHold** |

The production-policy validation environments compile with:

```
-DESPFC_ALTHOLD_V2_CENTERED_STICK_CHANNEL=6
```

For the custom LoRa/iBUS radio, iBUS channel 7 (zero-based channel index 6)
therefore carries the raw left-stick Y position mapped to 1000..2000 us with
approximately 1500 us at physical stick center.

The ordinary throttle channel remains independent. It may keep the existing
stateful/accumulated throttle behavior for manual flight. AltHold must not use
that accumulated value as a climb-rate request.

Expected receiver output while the vertical stick is released:

```
CH3 / logical throttle (index 3): previous accumulated manual throttle
CH7 / logical AUX3    (index 6):  1500 us
```

Expected AUX3 values:

```
full down   -> about 1000 us -> descent request
center      -> about 1500 us -> zero pilot climb/descent request
full up     -> about 2000 us -> climb request
```

The FC applies its own deadband and climb/descent scaling. The transmitter
should therefore send the raw centered stick rather than another integrated
or rate-shaped throttle value.

## Failsafe LAND ownership

Once Stage-2 AUTO_LAND begins, LAND remains authoritative for that armed flight.
A recovered receiver is still qualified, but pilot throttle is not handed back
during the descent. LAND ends only through estimator failure fallback,
touchdown confirmation, or the bounded LAND timeout, all of which disarm.

This avoids a thrust discontinuity when the custom transmitter's manual
throttle channel contains an accumulated value unrelated to the current
spring-stick position.

## Build policy

`ESPFC_ASSISTED_V2_ACTIVE` is the common Angle + AltHold + LAND activation
switch. A motor-driving build is rejected at compile time unless
`ESPFC_ASSISTED_V2_OUTPUT_ACK` is also explicitly defined.

The repository keeps two non-hardware validation paths:

- `esp32_assisted_v2_candidate`: real ESP32 compilation with
  `ESPFC_SAFE_BENCH_BUILD`, so the ESC driver is never attached.
- `native_assisted_v2_active`: exercises the production activation policy in
  native unit tests, with AUX3 selected as the centered vertical-stick input.

The ordinary `esp32` environment is intentionally not converted into an
Assisted V2 motor-driving target by these changes. Physical-actuator activation
is a separate hardware-validation milestone rather than an accidental side
effect of compiling the default target.

GitHub Actions also publishes the `esp32_assisted_v2_candidate` firmware as an
`esp32_assisted_v2_validation_<commit>` artifact. It is intentionally
non-actuating: the controller, estimator, mixer math, mode logic, and Blackbox
paths execute, but `ESPFC_SAFE_BENCH_BUILD` prevents ESC/servo drivers from
being attached.

## Blackbox validation views

Use the existing debug modes to identify which layer generated an unexpected
command during non-actuating hardware validation.

### `AUTOPILOT_ALTITUDE`

| debug field | value |
| --- | --- |
| debug[0] | fused altitude, cm |
| debug[1] | altitude target, cm |
| debug[2] | fused vertical rate, cm/s |
| debug[3] | vertical-rate target, cm/s |
| debug[4] | pilot vertical-rate request, cm/s |
| debug[5] | barometer innovation, cm |
| debug[6] | altitude estimator healthy (0/1) |
| debug[7] | barometer sample accepted (0/1) |

### `AUTOPILOT_PID`

| debug field | value |
| --- | --- |
| debug[0] | vertical-rate target, cm/s |
| debug[1] | measured vertical rate, cm/s |
| debug[2] | vertical-rate error, cm/s |
| debug[3] | vertical PID P term x1000 |
| debug[4] | vertical PID I term x1000 |
| debug[5] | vertical PID D term x1000 |
| debug[6] | requested normalized thrust x1000 |
| debug[7] | packed Assisted V2 status bits |

`debug[7]` status bits:

| bit | meaning |
| ---: | --- |
| 0 | ALTHOLD mode active |
| 1 | assisted altitude controller active |
| 2 | altitude estimator healthy |
| 3 | barometer sample accepted |
| 4 | Assisted V2 currently owns vertical output |
| 5 | LAND requested |
| 6 | LAND supervisor active |
| 7 | LAND output blocked |
| 8 | receiver channels valid |

These fields are intended to make the complete command chain observable before
physical actuator authority is enabled.

## Configuration prerequisites

Before Assisted V2 can be considered available at runtime, the FC still
requires its saved configuration to provide the actual hardware settings:
a detected gyro/accelerometer, a detected and calibrated barometer, a valid
motor protocol, correct receiver channel mapping, and mode conditions for
ARM/ANGLE/ALTHOLD as desired.

AUTO_LAND is not the source-code default. The default remains DROP so a saved
configuration must explicitly select AUTO_LAND after the non-actuating
validation path has been checked on the actual hardware.

## LAND termination

LAND V2 uses a build-selectable descent rate. The non-actuating ESP32
validation environments currently select **0.10 m/s downward**. The ordinary
production-policy default remains 0.50 m/s until the slower behavior has been
validated. Touchdown confirmation uses near-ground altitude, low vertical
speed, descent evidence, and a dwell period.

The timeout uses the same selected descent-rate constant:

```
timeout = clamp(entry_height / selected_descent_rate + 10 s, 15 s, 60 s)
```

If the estimator becomes invalid or the timeout expires, LAND disarms through
the failsafe path rather than remaining armed indefinitely.
