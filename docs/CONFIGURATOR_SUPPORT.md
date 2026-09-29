# Betaflight Configurator compatibility matrix

This file describes the current **ESP-FC master** behavior. Betaflight
Configurator exposes more fields than ESP-FC has physical hardware or runtime
algorithms for, so a visible control is not by itself evidence of an active
feature.

## Status vocabulary

- **ACTIVE** — an ESP-FC runtime path consumes the setting or mode.
- **SHADOW** — calculation/state-machine support exists, but it is explicitly
  blocked from setpoints, PID, mixer, arming and outputs.
- **PERSISTED ONLY** — the Configurator/MSP value round-trips through storage,
  but no active controller or hardware path consumes it.
- **FIXED CAPABILITY** — ESP-FC has a real implementation, but that option is
  fixed by the driver rather than freely selectable.
- **UNSUPPORTED** — the required runtime or hardware subsystem is absent.

## Flight-mode and navigation surface

| Configurator concept | ESP-FC status | Current behavior |
| --- | --- | --- |
| Acro / rate mode | ACTIVE | Existing ESP-FC rate path; unchanged by compatibility work |
| Angle | ACTIVE | Existing Angle V2 path; unchanged by compatibility work |
| AltHold | ACTIVE | Existing Assisted V2 path |
| LAND | ACTIVE | Existing failsafe LAND V2 path |
| Anti-Gravity | ACTIVE on standard ESP32 build | Existing production path; compatibility work does not alter its controller math |
| Horizon | SHADOW | Betaflight-style inclination/stick blend strength and diagnostic rate suggestion only |
| GPS Rescue / RTH | SHADOW | Home validity, N/E error, range, bearing and desired horizontal velocity diagnostics only |
| Position Hold | SHADOW | Captures a GPS hold point and calculates horizontal error/velocity demand only |
| Waypoints | SHADOW | Target-coordinate calculation API and horizontal error/velocity demand only; no Configurator mission upload or autonomous waypoint execution |
| Headfree | SHADOW | Betaflight-style yaw-reference roll/pitch transform only |
| Acro Trainer | SHADOW | Betaflight-style projected-angle limiting suggestion only |
| 3D reversible flight | PERSISTED ONLY | UI deadband/neutral values persist; no reversible motor-control path |
| GPS-driven motor authority | UNSUPPORTED | No shadow navigation value is connected to attitude/thrust/mixer output |

The shadow implementation has a hard source-level boundary: it may update only
`ModelState::shadow` and debug telemetry. It does not write
`state.setpoint`, `innerPid`, `angleV2`, `assistedMode`, mixer/output
state, arming or failsafe state.

## PID and tuning compatibility

The existing PID, Angle V2 and Acro/rates execution paths are intentionally
unchanged. The following Configurator fields now persist so they no longer
silently reset, but **remain non-authoritative metadata**:

- Absolute Control gain
- I-term rotation
- I-term Relax type selector (the existing ESP-FC I-term Relax magnitude/path
  remains separate)
- Smart Feedforward
- Feedforward transition
- modern FF averaging, smoothing, boost, jitter and max-rate metadata
- D-Max axis values, gain and advance
- integrated-yaw and yaw-relax metadata
- throttle boost
- battery-PID-compensation flag
- voltage-sag-compensation value
- thrust-linearization value
- dynamic-idle minimum-RPM value
- automatic profile cell count
- roll/pitch and yaw acceleration-limit metadata
- Acro-Trainer angle-limit value (consumed only by the SHADOW calculation)

ESP-FC still has one real PID/rate configuration set; storing profile-related
metadata does not create multiple active profiles. `MSP_PID_CONTROLLER`
continues to report the single supported controller and its selector remains
non-switchable.

## Receiver and input compatibility

| Field | Status |
| --- | --- |
| Serial PPM/SBUS/iBUS/CRSF and ESP-NOW paths | ACTIVE where supported by the selected target/port |
| Spektrum bind value | PERSISTED ONLY; no Spektrum bind/driver added |
| SPI-RX protocol, ID and channel-count fields | PERSISTED ONLY; no native SPI ELRS/FrSky driver added |
| ELRS UID/model ID | PERSISTED ONLY; serial ELRS over CRSF is a separate active path |
| FPV camera-angle field | PERSISTED ONLY |
| Legacy RC interpolation selectors | PERSISTED ONLY; ESP-FC's existing RC filters remain authoritative |
| Separate yaw/position-hold deadbands | PERSISTED ONLY |
| Throttle midpoint/expo/hover fields | PERSISTED ONLY; existing Acro/rate throttle handling is unchanged |

## Gyro and filter compatibility

32-kHz mode, high-FSR selection, multi-gyro selector, Configurator gyro
calibration threshold/duration/yaw-offset/overflow metadata, PWM inversion and
D-term dynamic-LPF exponent now round-trip as **PERSISTED ONLY** metadata where
there is no matching runtime consumer. Existing sensor detection, calibration,
filtering and single-gyro operation remain authoritative.

The legacy sensor-selection setter is atomic: rangefinder and optical-flow
selectors are rejected when non-zero because ESP-FC has no matching runtime
drivers. The alignment setter likewise accepts the existing enum-based
gyro/magnetometer alignment and single-gyro enable mask, while rejecting
custom per-gyro Euler offsets that ESP-FC cannot represent.

## GPS

- UBLOX is a **FIXED CAPABILITY** provider.
- Auto configuration is a **FIXED CAPABILITY** and is reported ON.
- Auto baud scanning is a **FIXED CAPABILITY** and is reported ON.
- SBAS AUTO/NONE now persists and the GPS sensor configuration path respects
  the resulting enable flag.
- Galileo configuration persists for receivers that support the modern GNSS
  configuration path.
- The u-blox M6/NEO-6M path uses legacy GPS-only UBX messages and does not gain
  newer constellations, dual-band or L5 capability from Configurator fields.
- GPS position, velocity, satellites, fix quality, home distance/bearing and
  home validity are real sensing/telemetry features.
- RTH/POSHOLD/waypoint actuator authority remains deliberately absent; those
  calculations are SHADOW only.

## Failsafe and arming UI

Failsafe delay, kill-switch and DROP/AUTO-LAND selection are active ESP-FC
settings. Configurator off-delay, fixed-throttle and throttle-low-delay values,
plus auto-disarm and gyro-calibration-on-first-arm metadata, now persist but do
not replace ESP-FC's existing failsafe/LAND/arming supervision.

## Battery/current

ADC voltage/current and MSP companion current are active. Virtual and ESC
current-meter source IDs remain unsupported and are sanitized instead of being
misrepresented as working sensors.

Betaflight's MSPv2 battery-profile surface is bridged to ESP-FC's single
battery configuration. Profile index 0 reports the existing cell-warning
threshold and the same fixed min/max/full-cell values already exposed by
`MSP_BATTERY_CONFIG`. The warning threshold may be written through the
profile command. Multiple profiles, capacity tracking, forced cell count and
consumption-percentage warnings remain unsupported; writes that attempt to
change those unsupported fields are rejected rather than acknowledged and
discarded.

## Blackbox and storage

Serial and target-supported flash Blackbox paths are active. Selecting an
SD-card enum does not create an SD-card driver, so SD Blackbox remains
unsupported on the standard target.

## VTX, telemetry, OSD and LEDs

- SmartAudio band/channel/power/low-power control remains the implemented VTX
  subset. `MSP2_GET_VTX_DEVICE_STATUS` now reports detected SmartAudio
  readiness plus the configured band/channel/power using Betaflight's common
  device-status field order; unavailable frequency/status/table/custom fields
  are explicitly marked unavailable.
- Full Betaflight VTX-table/custom-frequency/pit-frequency behavior is not
  claimed unless a real runtime consumer exists.
- FrSky serial selection currently routes to ESP-FC text telemetry rather than
  a complete FrSky encoder.
- HoTT selection has no normal telemetry encoder/runtime handler.
- The existing iBUS output is not advertised as a complete FlySky sensor
  telemetry service.
- Status LED / WS2812 support exists, but a full Betaflight LED-strip layout
  and effects editor is not implemented.
- A general Betaflight OSD/DisplayPort subsystem is not present.
- MSP requests for OSD warnings, the Betaflight LED-strip value editor,
  optical-flow telemetry and VTX-table rows remain explicit protocol errors.
  Native regressions lock this behavior so unsupported Configurator pages are
  not accidentally made to look functional by zero-filled placeholder data.

These interfaces stay explicit rather than reporting a stored value as a
hardware implementation.

## Miscellaneous compatibility

- `MSP_SET_RTC` stores a software timestamp, but does not imply a battery-backed
  hardware RTC. `MSP_TX_INFO` therefore reports RTC support as unavailable
  even after a software timestamp has been supplied.
- Normal firmware reboot is supported. Reboot-to-bootloader remains target/
  bootloader dependent and is not falsely acknowledged.
- Craft name is implemented and writable. Build key and release name are
  exposed as read-only firmware metadata. Unsupported text slots are rejected
  on write instead of falsely acknowledging a value that cannot be preserved.
- Debug-menu names are not capability declarations; a debug selection is
  meaningful only when ESP-FC has a producer for it.

## Reference implementations used for shadow design

The non-actuating calculations intentionally follow established architecture
rather than inventing a new flight stack:

- Betaflight: Horizon blend/strength, Acro-Trainer projected-angle limiting and
  Headfree frame transformation.
- INAV: separate POSHOLD, RTH and waypoint navigation-state architecture.
- ArduPilot: separation between position/waypoint navigation and the attitude
  controller.

No source from those projects is used to bypass ESP-FC's hard non-actuating
boundary.


## CLI capability summary

The read-only `capabilities` CLI command prints the current high-level
Configurator contract as ACTIVE, SHADOW, PERSISTED ONLY, FIXED and UNSUPPORTED
groups. It also prints `shadow_output_authority: NONE`, making the
non-actuating navigation boundary visible without requiring source inspection.
