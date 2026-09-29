# NEO-6M GPS integration on ESP32

This project supports u-blox 6 / NEO-6M as a **position and telemetry sensor**.
It does not grant GPS any motor, mixer, attitude, altitude, LAND, or autonomous
navigation authority.

## ESP32 UART pins

The project wiring uses the ESP32 hardware UART on:

- TX: GPIO17 (D17)
- RX: GPIO16 (D16)

On the current target these pins are also the default serial receiver UART.
A GPS byte stream and an iBUS/SBUS/CRSF receiver byte stream cannot occupy the
same UART at the same time. Firmware therefore preserves SERIAL_RX and disables
GPS on that UART if both functions are configured simultaneously, while setting
`state.gps.serialConflict` for diagnostics.

## NEO-6M protocol path

The u-blox 6 path uses legacy UBX messages supported by GPS-only NEO-6M
firmware:

- NAV-POSLLH for geodetic position and horizontal/vertical accuracy
- NAV-SOL for fix type, satellites used, pDOP, and solution accuracy
- NAV-VELNED for N/E/D velocity, ground speed, course, and speed accuracy
- NAV-SVINFO for per-satellite signal/usage information

The receiver is kept on its stable 5 Hz navigation rate. NMEA output is disabled
after detection and UBX is used for the runtime data path. Generation-9+
CFG-VALSET/L5/constellation configuration is skipped for NEO-6M.

## Runtime validity

A received solution is explicitly marked fresh. If no current navigation
solution arrives for 1.5 seconds, the fix and satellite-used count are
invalidated while the last coordinates remain available only as diagnostics.
Satellite summary fields expose satellites used and maximum C/N0.

This integration is intentionally limited to sensing, telemetry, diagnostics,
and home-vector math. It does not implement RTH, waypoint following, position
hold, or any other GPS-driven actuator control.


## Betaflight Configurator / MSP behavior

The firmware advertises the GPS build capability to Betaflight Configurator and
returns the UART's real baud index in the MSP, GPS, and telemetry baud fields.
When Configurator writes a GPS-only port, ESP-FC now consumes the GPS baud field
instead of discarding it.

`MSP_GPS_CONFIG` is mapped to the settings ESP-FC actually implements:

- provider is fixed to UBLOX
- auto configuration is fixed ON
- auto baud detection is fixed ON
- set-home-once is persisted
- Galileo maps to `gps_enable_galileo`
- SBAS AUTO/NONE maps to `gps_enable_sbas`

Regional SBAS selections are intentionally represented as AUTO because this
driver does not persist a regional SBAS selector.

The CLI `gps` command reports whether GPS is configured, detected, fresh,
assigned to a UART, or blocked by a GPS/SERIAL_RX conflict. This makes a
configuration problem distinguishable from a receiver that is detected but has
not obtained a navigation fix.

The GPIO16/GPIO17 resource conflict is not bypassed: if SERIAL_RX and GPS are
both assigned to that same UART, receiver input retains priority and GPS is
removed from the effective serial configuration. A raw receiver stream and a
raw GPS stream cannot be decoded simultaneously on one UART.


## CLI diagnostic corrections

The `gps` CLI status now prints `distanceToHome` directly in meters instead
of applying an angular radians-to-degrees conversion. GPS update rate also
prints `N/A` until a non-zero message interval has been measured, avoiding
an infinite/undefined diagnostic during startup or a cold start.
