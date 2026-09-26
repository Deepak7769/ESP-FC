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

LAND uses a nominal -0.50 m/s descent request. Touchdown confirmation uses
near-ground altitude, low vertical speed, descent evidence, and a dwell period.
A second independent termination path bounds LAND duration:

```
timeout = clamp(entry_height / 0.50 m/s + 10 s, 15 s, 60 s)
```

If the estimator becomes invalid or the timeout expires, LAND disarms through
the failsafe path rather than remaining armed indefinitely.
