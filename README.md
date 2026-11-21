# Dare Devil

Dare Devil is flight-controller firmware for ESP32 multicopters. The classic ESP32 build runs a gyro rate controller, Angle stabilization, barometric AltHold and failsafe LAND. Betaflight Configurator connects over MSP. The firmware reports the identifier `ESPF` and API 1.48.

## Bench-validated firmware

Commit `950bbdf` is the source revision flashed to the project quad. The ESP32 detected an MPU6500 over SPI, a BMP280 and a QMC5883P over I²C, and an iBUS receiver on UART2. The configured PID loop runs at 4 kHz and the motor outputs use 400 Hz PWM. Physical flight validation remains open.

## Build and flash

From the repository root, use PlatformIO to build for a classic ESP32 with 4 MB flash.

```sh
pio run -e esp32
```

For a new board, flash `.pio/build/esp32/firmware_0x00.bin` at `0x0000`. Export your CLI configuration first because the merged image overwrites stored settings. For an update on the matching partition layout, flash `.pio/build/esp32/firmware.bin` at `0x10000` with full erase disabled. Select the matching environment for other chips. [Partition layout](partitions_4M_nota.csv) · [Build configuration](platformio.ini)

## Configure and test

- [Setup and first bench checks](docs/setup.md)
- [Wiring examples](docs/wiring.md) and [CLI reference](docs/cli.md)
- [Angle, AltHold and LAND behavior](docs/ASSISTED_V2_INTEGRATION.md)
- [Anti-Gravity controller and bench tests](docs/ANTI_GRAVITY_BENCH.md)
- [Configurator feature coverage and known gaps](docs/CONFIGURATOR_COVERAGE.md)
- [Wireless configuration](docs/wireless.md) and [companion current meter](docs/MSP_CURRENT_METER.md)
- [Development and testing](docs/development.md)

## Credits and licensing

Dare Devil builds on ESP-FC and includes components from Betaflight and other upstream projects. Source files retain their original notices. [Project license](LICENSE)
