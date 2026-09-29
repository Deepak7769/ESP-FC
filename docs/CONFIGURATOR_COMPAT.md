# Configurator compatibility persistence

ESP-FC intentionally exposes a Betaflight-compatible MSP surface that is wider
than its active flight-control implementation. Starting with storage format
`0x03`, Configurator fields that previously reset to constants can round-trip
through a dedicated `ConfiguratorCompatConfig` tail.

## Authority rule

Persisting a Configurator value is **not** equivalent to enabling controller
behavior. Compatibility-only fields are not consumed by the existing PID,
Angle V2, Acro/rate, mixer, arming, or failsafe control paths unless a separate
source file explicitly documents a safe consumer.

This keeps Configurator honest about what the user saved while avoiding hidden
changes to established flight-control logic.

## Fields now persisted

The compatibility tail covers the previously discarded portions of:

- MSP PID advanced: battery PID flag, feedforward-transition metadata,
  acceleration limits, I-term-rotation flag, smart-feedforward flag,
  I-term-relax type, absolute-control gain, throttle boost, Acro-Trainer angle
  limit, D-Max fields, integrated-yaw fields, automatic-profile cell count,
  dynamic-idle field, modern feedforward metadata, voltage-sag compensation
  and thrust-linearization metadata.
- Advanced configuration: 32-kHz gyro flag, PWM inversion, gyro-selection/high
  FSR metadata, gyro calibration threshold/duration/yaw offset and overflow
  metadata.
- Receiver configuration: Spektrum-bind value, interpolation metadata, SPI-RX
  identity/channel metadata, camera angle, smoothing selector metadata, USB
  type, ELRS UID and model ID.
- Failsafe UI: off-delay, fixed-throttle and throttle-low-delay values.
- RC tuning: throttle midpoint, expo and hover values.
- Reversible/3D UI values and 3D deadband.
- Arming auto-disarm/first-arm-calibration metadata.
- Separate yaw/position-hold deadbands.
- D-term dynamic-LPF exponent.
- MSP-set software RTC timestamp.

## What remains non-active

These persisted values do not add unsupported hardware or algorithms. In
particular they do not create D-Max, Absolute Control, integrated yaw, 3D motor
reversal, SPI ELRS/Spektrum drivers, a hardware RTC, dynamic idle, thrust
linearization, or a new PID controller.

## EEPROM migration

Storage format is now `0x03`. A valid `0x02` image is migrated by copying
its unchanged legacy prefix and leaving the appended compatibility tail at
deterministic defaults. Mode rows using IDs that were invalid before the new
shadow-mode append are scrubbed during this migration so stale bytes cannot
activate a newly visible shadow request.


## CLI visibility

Compatibility values are exposed with a `compat_` prefix so a CLI dump does
not imply that a stored Betaflight field is an active ESP-FC controller. The
existing Anti-Gravity feature bit and gain now also have named CLI entries.

The `shadow` command prints Horizon, Headfree, Acro-Trainer and horizontal
navigation diagnostic state and always reports the hard output-authority
boundary.


## Protocol robustness completion

`MSP_RTC` now returns the same software timestamp accepted by
`MSP_SET_RTC`, completing that metadata round-trip without claiming a
battery-backed clock.

MSPv2 text writes now validate the declared payload length before modifying the
craft name. A truncated packet is rejected and leaves the previous name
unchanged; unsupported text-slot types remain explicitly unsupported rather
than being silently presented as implemented features.
