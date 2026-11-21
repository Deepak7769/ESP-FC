# Dare Devil setup

## Connect and identify the board

Remove propellers and connect the ESP32 over USB. Open the serial port in Betaflight Configurator. In the CLI, run `status`, `logs` and `dump`. Save a copy of the dump before changing settings. On the bench-tested build, the version line includes `ESPF ESP32 validation 950bbdf`.

## Project quad wiring

| Function | Classic ESP32 GPIO |
|---|---|
| MPU6500 SPI clock, MOSI, MISO, chip select | 18, 23, 19, 5 |
| BMP280 and QMC5883P I²C SDA, SCL | 21, 22 |
| iBUS receiver input, UART2 RX | 16 |
| Quad X motor outputs 1 through 4 | 25, 27, 32, 33 |

Check the actual board against its CLI pin map. UART1 defaults can overlap motor outputs on this installation. Check for conflicts before enabling another serial function. [Other wiring examples](wiring.md)

## Receiver and modes

In the Receiver tab, verify channel order, direction and switch ranges. The project radio supplies manual throttle on CH3 and a separate spring-centered vertical command on CH7 (AUX3). With the stick released, AUX3 should read about 1500 µs. Assign ARM to a verified switch range. Assign Angle and AltHold so that Angle can be tested independently. An active AltHold request blocks initial arming. [Assisted controller behavior](ASSISTED_V2_INTEGRATION.md)

## Check before first flight

1. On a level frame, calibrate the accelerometer and confirm that roll and pitch on the Setup model follow the frame.
2. Move the frame vertically and confirm relative altitude responds in the correct direction.
3. With propellers removed, verify motor order, motor direction and the disarm switch.
4. Verify receiver loss and recovery on the bench. With the radio linked, confirm the RX_FAILSAFE flag clears.
5. Save and reconnect. Verify the selected modes, receiver mapping and motor settings persisted.

## Optional peripherals and tuning

Use a dedicated serial port for GPS or serial Blackbox recording. On the project quad UART2 already carries iBUS. Verify the selected feature has a runtime driver for the target before wiring additional devices. The coverage guide records the supported settings and outstanding work by configurator tab. [Configurator coverage](CONFIGURATOR_COVERAGE.md)
