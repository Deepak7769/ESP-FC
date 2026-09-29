# Non-actuating compatibility shadow features

This subsystem implements the *calculation and state-machine layer* for several
Configurator-visible concepts that ESP-FC previously did not implement. It is
deliberately isolated from actuator authority.

## Hard authority boundary

`Control/ShadowFeatures.cpp` may read receiver, attitude and GPS state and may
write only `ModelState::shadow` plus debug slots. It does **not** write:

- `ModelState::setpoint`
- `ModelState::innerPid`
- `ModelState::angleV2`
- `ModelState::assistedMode`
- mixer state
- motor/servo outputs
- arming/failsafe state

The ordinary PID controller, Angle V2 controller and Acro/rate path are not
modified by this integration.

## Reference behavior

The implementation is based on the architecture and math patterns used by real
flight-control projects, but stops before actuator authority:

- **Betaflight Horizon**: leveling strength is reduced by both craft inclination
  and stick deflection, then used only to calculate a diagnostic Acro/level
  blend.
- **Betaflight Acro Trainer**: current angle plus a gyro-rate-scaled lookahead is
  checked against an angle limit and a diagnostic limiting-rate suggestion is
  calculated.
- **Betaflight Headfree**: a yaw reference is captured on entry and roll/pitch
  commands are rotated into the current body frame for diagnostics.
- **INAV navigation architecture**: RTH, position hold and waypoint processing
  are represented as separate navigation states. ESP-FC computes target
  north/east error, distance, bearing and bounded desired horizontal velocity,
  but never converts these values into attitude or thrust commands.
- **ArduPilot architecture**: position/waypoint navigation is kept separate from
  the underlying attitude controller. ESP-FC follows the same separation in
  this shadow subsystem.

Reference revisions used during implementation:

- Betaflight: commit 744f95fa31542c4c906f18072348a366ab11b6b7
- INAV: current navigation state-machine structure reviewed 2026-09-29
- ArduPilot: current AC_WPNav / AC_Loiter separation reviewed 2026-09-29

## Configurator modes

The following modes are appended without renumbering existing ESP-FC modes:

- HORIZON SHADOW
- GPS RESCUE SHADOW
- POSHOLD SHADOW
- HEADFREE SHADOW
- ACRO TRAINER SHADOW
- WAYPOINT SHADOW

The word **SHADOW** is intentional: selecting these modes does not grant flight
authority.

## Diagnostics

`DEBUG_POSITION_NAV` exposes target north/east error, target distance and the
shadow navigation phase. Horizon strength is also available through the
existing angle debug channel.

The shadow state contains the full calculation outputs and can be inspected in
native tests or future telemetry without changing the existing controller.


## Authority regression coverage

Native tests exercise the shadow subsystem with valid GPS/home data and a
non-zero return-to-home navigation demand while seeding rate setpoints, angle
targets, assisted-altitude state, PID terms, normalized output channels and
PWM output values with sentinel values. The test requires all authoritative
state to remain byte-for-byte equivalent at the field level after the shadow
navigation update.

This is intentionally stronger than checking the `authorityBlocked` flag:
a future change that accidentally writes a setpoint, PID term or output will
fail CI even if the flag still says that authority is blocked.


## Coordinate robustness

Horizontal shadow-navigation deltas use 64-bit longitude subtraction and
shortest-path wrapping across the international date line before converting to
local north/east meters. This avoids signed 32-bit overflow at the
+180/-180-degree boundary and keeps the diagnostic vector local rather than
accidentally spanning nearly a full Earth circumference.


## GPS qualification

Shadow horizontal navigation now requires a fresh GPS solution, an accepted
2D-or-better fix type, and latitude/longitude inside the physical WGS-84
degree bounds before capturing or using a position. Diagnostic waypoints
outside ±90 degrees latitude or ±180 degrees longitude are rejected rather
than being propagated into navigation math.


## CI authority guard

In addition to runtime regression tests, CI runs
`tools/check_shadow_authority.py`. The check rejects direct assignments from
the shadow subsystem into setpoints, PID state, Angle V2, Assisted V2,
failsafe, mixer or output state; it also rejects authority-changing Model
calls and direct control/output includes. A second scan rejects every
shadow-mode identifier from the authoritative Control/Output source trees, so
a future controller cannot silently consume a shadow mode without failing CI.
Debug telemetry remains permitted.


## Waypoint interface scope

The waypoint shadow has a bounded in-memory target API used by native
regressions and future diagnostic integrations. There is currently no
Configurator mission-upload protocol, persistent mission store, path planner,
or autonomous waypoint executor. The visible WAYPOINT SHADOW mode therefore
does not imply a working mission system.


Global shadow-navigation deltas now use great-circle distance and initial
bearing, then project that diagnostic vector into local north/east components.
The calculation remains non-actuating but no longer relies on a short-baseline
flat-Earth approximation for high-latitude or long-distance targets.
