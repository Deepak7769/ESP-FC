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
