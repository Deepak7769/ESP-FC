# Virtual configurator coverage

The recording shows Betaflight App 2026.6.1, virtual firmware 2025.12.0 and MSP API 1.46. The hardware scope includes every listed option, including disabled choices. The firmware under test is ESP-FC and its target settings determine the physical outputs available. [App source](https://github.com/betaflight/betaflight-configurator/blob/14a057ffc58417c5128199fc1233284982a64be3/src/js/VirtualFC.js)

## Recording index

Times indicate the first sampled second with that tab selected. Presets appears for about one second.

| Time | Tab | Firmware or host behavior |
|---|---|---|
| 0:00 | [Setup](https://github.com/betaflight/betaflight-configurator/blob/14a057ffc58417c5128199fc1233284982a64be3/src/components/tabs/SetupTab.vue) | Calibration, reset, status and live attitude need a connected-board comparison. |
| 0:09 | [Sensors](https://github.com/betaflight/betaflight-configurator/blob/14a057ffc58417c5128199fc1233284982a64be3/src/components/tabs/SensorsTab.vue) | HC-SR04 uses configurable trigger and echo pins. TFmini, TF02 and TFnova use the LIDAR_TF serial port. The four MT models send range and optical flow over MSP. The firmware reports fresh range through MSP and raw flow through debug channels. Custom compass mounting angles are applied before board alignment. Saving sensor alignment preserves an enabled, explicitly selected gyro. The barometer and compass driver checks are listed below. Sensor fusion for assisted flight and physical verification remain open. |
| 0:24 | [Ports](https://github.com/betaflight/betaflight-configurator/blob/14a057ffc58417c5128199fc1233284982a64be3/src/components/tabs/PortsTab.vue) | The code reads and writes distinct GPS and telemetry rates, uses the serial Blackbox rate on dedicated ports, and emits LTM frames. The remaining port roles have different driver requirements. |
| 0:46 | [Configuration](https://github.com/betaflight/betaflight-configurator/blob/14a057ffc58417c5128199fc1233284982a64be3/src/components/tabs/ConfigurationTab.vue) | Several advanced configuration and beeper options still need runtime consumers. |
| 1:04 | [Power and Battery](https://github.com/betaflight/betaflight-configurator/blob/14a057ffc58417c5128199fc1233284982a64be3/src/components/tabs/PowerTab.vue) | Native tests cover the battery fields, source IDs, virtual-meter calibration, charge integration and fresh extended DShot readings. UART ESC sensor devices still need a serial driver. |
| 1:08 | [Failsafe](https://github.com/betaflight/betaflight-configurator/blob/14a057ffc58417c5128199fc1233284982a64be3/src/components/tabs/FailsafeTab.vue) | The flight suite covers the existing LAND and DROP behavior. Each remaining visible policy and per-channel action needs verification. |
| 1:23 | [Presets](https://github.com/betaflight/betaflight-configurator/blob/14a057ffc58417c5128199fc1233284982a64be3/src/components/tabs/PresetsTab.vue) | The app downloads and applies presets using the firmware CLI. Matching each command, range and default in a preset is a separate firmware dependency. |
| 1:24 | [PID Tuning](https://github.com/betaflight/betaflight-configurator/blob/14a057ffc58417c5128199fc1233284982a64be3/src/components/tabs/PidTuningTab.vue) | Manual throttle uses the curve saved by the app, including mid, expo and hover on newer APIs. The flight suite covers several control loops. Four selectable rate profiles retain their names, stick rates, limits and throttle curves across saves. Three PID profiles support selection, copying, naming and resetting the current profile. D-Max ceilings, gain and advance change the active D term, follow the selected PID profile and appear in D_MAX debug channels. Native tests cover the slider preview, saved settings, motion response and upper bound. |
| 1:53 | [Receiver](https://github.com/betaflight/betaflight-configurator/blob/14a057ffc58417c5128199fc1233284982a64be3/src/components/tabs/ReceiverTab.vue) | SBUS, IBUS and CRSF already have drivers. New tests exercise SUMD, SUMH, XBus Mode B and RJ01. The serial and SPI lists below track the remaining choices. |
| 2:30 | [Modes](https://github.com/betaflight/betaflight-configurator/blob/14a057ffc58417c5128199fc1233284982a64be3/src/components/tabs/AuxiliaryTab.vue) | The simulator exposes more mode IDs than the ESP-FC runtime. Each mode needs both its permanent-ID mapping and its action. |
| 3:17 | [Adjustments](https://github.com/betaflight/betaflight-configurator/blob/14a057ffc58417c5128199fc1233284982a64be3/src/components/tabs/AdjustmentsTab.vue) | The app expects adjustment ranges, adjustable functions and live AUX behavior. These Betaflight MSP range commands are missing. |
| 3:45 | [Servos](https://github.com/betaflight/betaflight-configurator/blob/14a057ffc58417c5128199fc1233284982a64be3/src/components/tabs/ServosTab.vue) | Native tests exercise the tab load request, min/middle/max, signed mixer rate, AUX forwarding, lost-input fallback and PWM command calculation. Bench verification is open. |
| 3:51 | [GPS](https://github.com/betaflight/betaflight-configurator/blob/14a057ffc58417c5128199fc1233284982a64be3/src/components/tabs/GpsTab.vue) | NMEA, u-blox and MSP input paths are present. Tests cover NMEA input, fix timeout, both MSP input formats and config readback. SBAS and Galileo need a receiver check. Map tiles and controls run in the app. |
| 4:11 | [Motors](https://github.com/betaflight/betaflight-configurator/blob/14a057ffc58417c5128199fc1233284982a64be3/src/components/tabs/MotorsTab.vue) | Extended DShot voltage/current units are tested. Protocol choice, ESC operations and airframe mixer coverage vary by target. |
| 4:43 | [OSD](https://github.com/betaflight/betaflight-configurator/blob/14a057ffc58417c5128199fc1233284982a64be3/src/components/tabs/OsdTab.vue) | The firmware lacks the Betaflight OSD configuration handler and the corresponding renderers for the complete layout editor. |
| 5:30 | [Video Transmitter](https://github.com/betaflight/betaflight-configurator/blob/14a057ffc58417c5128199fc1233284982a64be3/src/components/tabs/VtxTab.vue) | SmartAudio sends channel and power changes and applies both low-power-disarm policies. VTX device confirmation, VTX tables, additional protocols and their power/band behavior are open. |
| 5:42 | [LED Strip](https://github.com/betaflight/betaflight-configurator/blob/14a057ffc58417c5128199fc1233284982a64be3/src/components/tabs/LedStripTab.vue) | The virtual editor has 256 layout entries. The programmable strip layout, palette, overlays and controller are open. |
| 6:18 | [Tethered Logging](https://github.com/betaflight/betaflight-configurator/blob/14a057ffc58417c5128199fc1233284982a64be3/src/components/tabs/LoggingTab.vue) | The app samples MSP telemetry and owns the host file. Real sample values and sustained transport rates need a connected-board run. |
| 6:32 | [Blackbox](https://github.com/betaflight/betaflight-configurator/blob/14a057ffc58417c5128199fc1233284982a64be3/src/components/tabs/OnboardLoggingTab.vue) | The firmware has serial and flash logging. Storage operations, field masks, debug data and all selectable media need separate end-to-end checks. |
| 6:44 | [Autotune](https://github.com/betaflight/betaflight-configurator/blob/14a057ffc58417c5128199fc1233284982a64be3/src/components/tabs/AutotuneTab.vue) | The app imports a chirp log and analyzes the response. Firmware chirp generation and matching log fields are open. |
| 6:47 | [Blackbox Viewer](https://github.com/betaflight/betaflight-configurator/blob/14a057ffc58417c5128199fc1233284982a64be3/src/components/tabs/BlackboxViewerTab.vue) | The app renders a log file. A recording from each supported firmware logging backend is needed for an end-to-end check. |

## Sensor verification

| Configurator choice | Firmware behavior exercised in native tests | Physical checks still open |
|---|---|---|
| BMP085 | Reads the factory coefficients in big-endian order, runs temperature and pressure conversions, and rejects incomplete reads. | Compare pressure against a reference while changing height. |
| MS5611 | Validates the PROM CRC before reset, compensates warm and cold readings, and rejects failed transfers. The driver uses full SPI commands and supports both I²C addresses. | Exercise I²C and SPI on a physical device and check timing and first valid altitude. |
| LPS | Detects three register layouts covering LPS25HB, LPS22HB and LPS22DF. Tests check control registers, negative temperature, pressure units, missing data and reset timeout. | Verify both buses, including single-byte output reads with block data update enabled. |
| DPS310 / SPL06-001 | Both chips report product ID 0x10. Explicit DPS310 selection reads the factory temperature-source bit. AUTO keeps the previous SPL06-001 interpretation. Direct I²C and SPI check data-ready status before publishing. | Verify the fitted model, calibration source and conversion timing. For a known DPS310 select DPS310 explicitly. |
| QMC5883L | Configures 100 Hz continuous measurement, checks configuration readback, and applies the scale for the selected ±2 gauss range. | Compare output with a known field and verify axis orientation on the installed board. |
| Sensors behind an MPU auxiliary I²C master | Splits long calibration reads into hardware-sized transfers and rejects auxiliary-bus NACK and arbitration loss. When a compass owns the single configured read window, the barometer probe leaves it in place. | Check disconnect and recovery on a physical MPU bridge. Simultaneous streaming of two sensors on that bridge requires additional master slots. |

The LPS driver follows ST's register and output conventions. The DPS310 driver uses the factory calibration-source bit and conversion-time limits. The QMC5883L uses the manufacturer's scale and set/reset initialization. [ST LPS22HB](https://www.st.com/resource/en/datasheet/lps22hb.pdf) · [ST LPS25HB](https://www.st.com/resource/en/datasheet/lps25hb.pdf) · [Infineon DPS310](https://www.infineon.com/dgdl/Infineon-DPS310-DataSheet-v01_02-EN.pdf?fileId=5546d462576f34750157750826c42242) · [QST QMC5883L](https://www.qstcorp.com/upload/pdf/202512/13-52-04%20QMC5883L%20Datasheet%20Rev.%20B.pdf)

BMP388 and BMP390 use the Bosch factory trim block, forced conversions and a dummy byte on SPI reads. QMP6988 and 2SMPB_02B share a chip ID and require explicit model selection on direct I²C or SPI so the firmware applies the correct compensation. Native tests use synthetic trim and pressure frames. Physical checks on both buses remain open. The VIRTUAL barometer still needs a simulated data source. [Bosch BMP388](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp388-ds001.pdf) · [QST QMP6988](https://robu-prod-media.s3.ap-south-1.amazonaws.com/uploads/2022/09/1359730-QMP6988-Datasheet.pdf) · [Omron 2SMPB-02B](https://components.omron.com/sites/default/files/datasheet_pdf/A255-E1.pdf)

The remaining control and peripheral work is identified by tab above.

## Checking profiles with the configurator

With a disarmed board connected, open the **Rates** subtab of **PID Tuning**. Select the first profile, give it a name, change its roll rate or throttle curve, and save. Select another profile and confirm that it shows its own values. Copy the first profile over the second, reconnect after saving, and check the active index, name and curve. The four profiles share receiver channel mapping and calibration. A firmware upgrade from storage version 2 or 3 starts on profile one and seeds all four profiles with the previous active rates.

The PID and Filter subtabs select one of three PID profiles. Switching while disarmed reloads the controller gains and D-term filters. Copy and reset operate on the selected profile. A reset keeps the other profiles and receiver configuration. On upgrade, versions 2 through 4 seed the three PID profiles with the previous tuning.

## Hardware and repeated options

These are configurator option names. Physical support still requires a matching backend and transport. AUTO, NONE, simulator and target-defined choices have their own behavior.

**Accelerometers**

`AUTO`, `NONE`, `ADXL345`, `MPU6050`, `MMA8452`, `BMA280`, `LSM303DLHC`, `MPU6000`, `MPU6500`, `MPU9250`, `ICM20601`, `ICM20602`, `ICM20608G`, `ICM20649`, `ICM20689`, `ICM42605`, `ICM42688P`, `BMI160`, `BMI270`, `LSM6DSO`, `LSM6DSV16X`, `VIRTUAL`. [Definition](https://github.com/betaflight/betaflight-configurator/blob/14a057ffc58417c5128199fc1233284982a64be3/src/js/sensor_types.js)

**Gyroscopes**

`AUTO`, `NONE`, `MPU6050`, `L3G4200D`, `MPU3050`, `L3GD20`, `MPU6000`, `MPU6500`, `MPU9250`, `ICM20601`, `ICM20602`, `ICM20608G`, `ICM20649`, `ICM20689`, `ICM42605`, `ICM42688P`, `BMI160`, `BMI270`, `LSM6DSO`, `LSM6DSV16X`, `VIRTUAL`. [Definition](https://github.com/betaflight/betaflight-configurator/blob/14a057ffc58417c5128199fc1233284982a64be3/src/js/sensor_types.js)

**Barometers**

`DEFAULT`, `NONE`, `BMP085`, `MS5611`, `BMP280`, `LPS`, `QMP6988`, `BMP388`, `DPS310`, `2SMPB_02B`, `VIRTUAL`. [Definition](https://github.com/betaflight/betaflight-configurator/blob/14a057ffc58417c5128199fc1233284982a64be3/src/js/sensor_types.js)

**Magnetometers**

`DEFAULT`, `NONE`, `HMC5883`, `AK8975`, `AK8963`, `QMC5883`, `LIS2MDL`, `LIS3MDL`, `MPU925X_AK8963`, `IST8310`. [Definition](https://github.com/betaflight/betaflight-configurator/blob/14a057ffc58417c5128199fc1233284982a64be3/src/js/sensor_types.js)

**Rangefinders**

`NONE`, `HCSR04`, `TFMINI`, `TF02`, `MTF01`, `MTF02`, `MTF01P`, `MTF02P`, `TFNOVA`. [Definition](https://github.com/betaflight/betaflight-configurator/blob/14a057ffc58417c5128199fc1233284982a64be3/src/js/sensor_types.js)

**Optical flow**

`NONE`, `MT`. [Definition](https://github.com/betaflight/betaflight-configurator/blob/14a057ffc58417c5128199fc1233284982a64be3/src/js/sensor_types.js)

**Serial receivers**

`NONE`, `SPEKTRUM2048`, `SBUS`, `SUMD`, `SUMH`, `XBUS_MODE_B`, `XBUS_MODE_B_RJ01`, `IBUS`, `JETIEXBUS`, `CRSF`, `SPEKTRUM2048/SRXL`, `TARGET_CUSTOM`, `FPORT`, `SPEKTRUM SRXL2`, `IRC GHOST`, `SPEKTRUM1024`. [Definition](https://github.com/betaflight/betaflight-configurator/blob/14a057ffc58417c5128199fc1233284982a64be3/src/js/fc.js)

**SPI receivers**

`NRF24_V202_250K`, `NRF24_V202_1M`, `NRF24_SYMA_X`, `NRF24_SYMA_X5C`, `NRF24_CX10`, `CX10A`, `NRF24_H8_3D`, `NRF24_INAV`, `FRSKY_D`, `FRSKY_X`, `A7105_FLYSKY`, `A7105_FLYSKY_2A`, `NRF24_KN`, `SFHSS`, `SPEKTRUM`, `FRSKY_X_LBT`, `REDPINE`, `FRSKY_X_V2`, `FRSKY_X_LBT_V2`, `EXPRESSLRS`. [Definition](https://github.com/betaflight/betaflight-configurator/blob/14a057ffc58417c5128199fc1233284982a64be3/src/components/tabs/ReceiverTab.vue)

**Port functions**

`MSP`, `GPS`, `TELEMETRY_FRSKY`, `TELEMETRY_HOTT`, `TELEMETRY_SMARTPORT`, `RX_SERIAL`, `BLACKBOX`, `TELEMETRY_LTM`, `TELEMETRY_MAVLINK`, `IRC_TRAMP`, `ESC_SENSOR`, `TBS_SMARTAUDIO`, `TELEMETRY_IBUS`, `RUNCAM_DEVICE_CONTROL`, `LIDAR_TF`, `FRSKY_OSD`, `VTX_MSP`. [Definition](https://github.com/betaflight/betaflight-configurator/blob/14a057ffc58417c5128199fc1233284982a64be3/src/composables/ports/usePortsRules.js)

**Airframe mixers**

`Tricopter`, `Quad +`, `Quad X`, `Bicopter`, `Gimbal`, `Y6`, `Hex +`, `Flying Wing`, `Y4`, `Hex X`, `Octo X8`, `Octo Flat +`, `Octo Flat X`, `Airplane`, `Heli 120`, `Heli 90`, `V-tail Quad`, `Hex H`, `PPM to SERVO`, `Dualcopter`, `Singlecopter`, `A-tail Quad`, `Custom`, `Custom Airplane`, `Custom Tricopter`, `Quad X 1234`, `Octo X8 +`. [Definition](https://github.com/betaflight/betaflight-configurator/blob/14a057ffc58417c5128199fc1233284982a64be3/src/js/model.js)

**Virtual mode assignments at API 1.46**

`ARM`, `ANGLE`, `HORIZON`, `ANTI GRAVITY`, `MAG`, `HEADFREE`, `HEADADJ`, `CAMSTAB`, `PASSTHRU`, `BEEPERON`, `LEDLOW`, `CALIB`, `OSD DISABLE`, `TELEMETRY`, `SERVO1`, `SERVO2`, `SERVO3`, `BLACKBOX`, `FAILSAFE`, `AIR MODE`, `3D DISABLE`, `FPV ANGLE MIX`, `BLACKBOX ERASE`, `CAMERA CONTROL 1`, `CAMERA CONTROL 2`, `CAMERA CONTROL 3`, `FLIP OVER AFTER CRASH`, `BOXPREARM`, `BEEP GPS SATELLITE COUNT`, `VTX PIT MODE`, `USER1`, `USER2`, `USER3`, `USER4`, `PID AUDIO`, `PARALYZE`, `GPS RESCUE`, `ACRO TRAINER`, `VTX CONTROL DISABLE`, `LAUNCH CONTROL`, `MSP OVERRIDE`, `STICK COMMANDS DISABLE`, `BEEPER MUTE`, `READY`, `LAP TIMER RESET`. [Definition](https://github.com/betaflight/betaflight-configurator/blob/14a057ffc58417c5128199fc1233284982a64be3/src/js/VirtualFC.js)

**Adjustment selector at API 1.46**

`No changes`, `RC Rate Adjustment`, `RC Expo Adjustment`, `Throttle Expo Adjustment`, `Pitch & Roll Rate Adjustment`, `Yaw Rate Adjustment`, `Pitch & Roll P Adjustment`, `Pitch & Roll I Adjustment`, `Pitch & Roll D Adjustment`, `Yaw P Adjustment`, `Yaw I Adjustment`, `Yaw D Adjustment`, `Rate Profile Selection`, `Pitch Rate`, `Roll Rate`, `Pitch P Adjustment`, `Pitch I Adjustment`, `Pitch D Adjustment`, `Roll P Adjustment`, `Roll I Adjustment`, `Roll D Adjustment`, `RC Rate Yaw`, `Pitch & Roll F Adjustment`, `Feedforward Transition`, `Horizon Strength Adjustment`, `PID-Audio Selection`, `Pitch F Adjustment`, `Roll F Adjustment`, `Yaw F Adjustment`, `OSD Profile Selection`, `LED Profile Selection`, `LED Brightness Adjust`, `Slider Master Multiplier`. [Definition](https://github.com/betaflight/betaflight-configurator/blob/14a057ffc58417c5128199fc1233284982a64be3/src/composables/adjustments/useAdjustmentsData.js)

**OSD display elements**

`RSSI_VALUE`, `MAIN_BATT_VOLTAGE`, `CROSSHAIRS`, `ARTIFICIAL_HORIZON`, `HORIZON_SIDEBARS`, `TIMER_1`, `TIMER_2`, `FLYMODE`, `CRAFT_NAME`, `THROTTLE_POSITION`, `VTX_CHANNEL`, `CURRENT_DRAW`, `MAH_DRAWN`, `GPS_SPEED`, `GPS_SATS`, `ALTITUDE`, `PID_ROLL`, `PID_PITCH`, `PID_YAW`, `POWER`, `PID_RATE_PROFILE`, `WARNINGS`, `AVG_CELL_VOLTAGE`, `GPS_LON`, `GPS_LAT`, `DEBUG`, `PITCH_ANGLE`, `ROLL_ANGLE`, `MAIN_BATT_USAGE`, `DISARMED`, `HOME_DIR`, `HOME_DIST`, `NUMERICAL_HEADING`, `NUMERICAL_VARIO`, `COMPASS_BAR`, `ESC_TEMPERATURE`, `ESC_RPM`, `REMAINING_TIME_ESTIMATE`, `RTC_DATE_TIME`, `ADJUSTMENT_RANGE`, `CORE_TEMPERATURE`, `ANTI_GRAVITY`, `G_FORCE`, `MOTOR_DIAG`, `LOG_STATUS`, `FLIP_ARROW`, `LINK_QUALITY`, `FLIGHT_DIST`, `STICK_OVERLAY_LEFT`, `STICK_OVERLAY_RIGHT`, `PILOT_NAME`, `DISPLAY_NAME`, `ESC_RPM_FREQ`, `RATE_PROFILE_NAME`, `PID_PROFILE_NAME`, `OSD_PROFILE_NAME`, `RSSI_DBM_VALUE`, `RC_CHANNELS`, `CAMERA_FRAME`, `OSD_EFFICIENCY`, `TOTAL_FLIGHTS`, `OSD_UP_DOWN_REFERENCE`, `OSD_TX_UPLINK_POWER`, `WH_DRAWN`, `AUX_VALUE`, `READY_MODE`, `RSNR_VALUE`, `SYS_GOGGLE_VOLTAGE`, `SYS_VTX_VOLTAGE`, `SYS_BITRATE`, `SYS_DELAY`, `SYS_DISTANCE`, `SYS_LQ`, `SYS_GOGGLE_DVR`, `SYS_VTX_DVR`, `SYS_WARNINGS`, `SYS_VTX_TEMP`, `SYS_FAN_SPEED`, `GPS_LAP_TIME_CURRENT`, `GPS_LAP_TIME_PREVIOUS`, `GPS_LAP_TIME_BEST3`. [Definition](https://github.com/betaflight/betaflight-configurator/blob/14a057ffc58417c5128199fc1233284982a64be3/src/components/tabs/osd/osd.js)

**OSD post-flight statistics**

`RTC_DATE_TIME`, `TIMER_1`, `TIMER_2`, `MAX_SPEED`, `MAX_DISTANCE`, `MIN_BATTERY`, `END_BATTERY`, `STAT_BATTERY`, `MIN_RSSI`, `MAX_CURRENT`, `USED_MAH`, `MAX_ALTITUDE`, `BLACKBOX`, `BLACKBOX_LOG_NUMBER`, `MAX_G_FORCE`, `MAX_ESC_TEMP`, `MAX_ESC_RPM`, `MIN_LINK_QUALITY`, `FLIGHT_DISTANCE`, `MAX_FFT`, `STAT_TOTAL_FLIGHTS`, `STAT_TOTAL_FLIGHT_TIME`, `STAT_TOTAL_FLIGHT_DIST`, `MIN_RSSI_DBM`, `USED_WH`, `MIN_RSNR`, `STAT_BEST_3_CONSEC_LAPS`, `STAT_BEST_LAP`, `STAT_FULL_THROTTLE_TIME`, `STAT_FULL_THROTTLE_COUNTER`, `STAT_AVG_THROTTLE`. [Definition](https://github.com/betaflight/betaflight-configurator/blob/14a057ffc58417c5128199fc1233284982a64be3/src/components/tabs/osd/osd.js)

**OSD warnings**

`ARMING_DISABLED`, `BATTERY_NOT_FULL`, `BATTERY_WARNING`, `BATTERY_CRITICAL`, `VISUAL_BEEPER`, `CRASH_FLIP_MODE`, `ESC_FAIL`, `CORE_TEMPERATURE`, `RC_SMOOTHING_FAILURE`, `FAILSAFE`, `LAUNCH_CONTROL`, `GPS_RESCUE_UNAVAILABLE`, `GPS_RESCUE_DISABLED`, `RSSI`, `LINK_QUALITY`, `RSSI_DBM`, `OVER_CAP`, `RSNR`, `LOAD`. [Definition](https://github.com/betaflight/betaflight-configurator/blob/14a057ffc58417c5128199fc1233284982a64be3/src/components/tabs/osd/osd.js)

## Control index

The following index contains 490 distinct input or action bindings from the 21 selected tab templates and their tab-specific components at the pinned revision. The app shares templates across firmware versions. Conditional or hidden rows can therefore appear here. A binding can generate several controls, as with servo rows and AUX ranges. A listed binding records UI coverage only.

### Setup

[SetupTab.vue](https://github.com/betaflight/betaflight-configurator/blob/14a057ffc58417c5128199fc1233284982a64be3/src/components/tabs/SetupTab.vue)

- `resetZaxis`
- `showConfirmReset`
- `onRebootBootloader`
- `confirmResetOpen`
- `cancelConfirmReset`
- `confirmReset`
- `buildInfoOpen`
- `closeBuildInfo`

### Sensors

[SensorsTab.vue](https://github.com/betaflight/betaflight-configurator/blob/14a057ffc58417c5128199fc1233284982a64be3/src/components/tabs/SensorsTab.vue)

- `(checked) => toggleGyro(gyro.index, checked)`
- GYRO/ACCEL — `sensorAlignment.gyro_to_use`
- Accelerometer — `accHardwareEnabled`
- Magnetometer — `magHardwareEnabled`
- Barometer — `baroHardwareEnabled`
- Rangefinder — `sonarHardwareEnabled`
- Rangefinder — `sensorConfig.sonar_hardware`
- Optical Flow — `opticalFlowHardwareEnabled`
- Optical Flow — `sensorConfig.opticalflow_hardware`
- `openBoardAlignmentWizard`
- First GYRO — `sensorAlignment.gyro_1_align`
- Second GYRO — `sensorAlignment.gyro_2_align`
- `onCalibrateAccel`
- Accelerometer Roll Trim — `accelTrims.roll`
- Accelerometer Pitch Trim — `accelTrims.pitch`
- `resetYaw`
- Magnetometer Alignment — `sensorAlignment.align_mag`
- Magnetometer Alignment — `startAlignDetection`
- `cancelAlignDetection`
- `applyAlignDetection`
- `resetAlignDetection`
- `dismissDeclinationNote`
- `dismissDeclinationWarning`
- `magDeclination`
- `autoSetDeclination`
- `magInclination !== null ? magInclination + '°' : '—'`
- `magFieldStrength !== null ? magFieldStrength + ' nT' : '—'`
- `startCheckMode()`
- `startGuidedCal()`
- `cancelMagCal()`
- `clearMagCalSamples()`
- `acceptGuidedMagCal()`
- `magVizMode = m.value`
- `copyFullCalCli`
- `applyFullCal`
- `exportFullCalModel`
- `retryAndStartMagCal()`
- `retryMagCal()`
- `startLegacyFirmwareCal()`
- `showLiveSensors = true`
- `saveConfig`

[LiveSensorPanel.vue](https://github.com/betaflight/betaflight-configurator/blob/14a057ffc58417c5128199fc1233284982a64be3/src/components/tabs/sensors/LiveSensorPanel.vue)

- `checkboxes[0]`
- `checkboxes[1]`
- `checkboxes[2]`
- `checkboxes[3]`
- `checkboxes[4]`
- `checkboxes[5]`
- `debugModeName`
- `updateGlobalRate(Number($event))`

[SensorGraph.vue](https://github.com/betaflight/betaflight-configurator/blob/14a057ffc58417c5128199fc1233284982a64be3/src/components/tabs/sensors/SensorGraph.vue)

- `$emit('update:rate', Number($event))`
- `$emit('update:scale', Number($event))`

### Ports

[PortsTab.vue](https://github.com/betaflight/betaflight-configurator/blob/14a057ffc58417c5128199fc1233284982a64be3/src/components/tabs/PortsTab.vue)

- `port.msp`
- `port.msp_baudrate`
- `port.rxSerial`
- `portFieldSet(port, 'telemetry', $event); onTelemetryChange(port);`
- `port.telemetry_baudrate`
- `portFieldSet(port, 'sensor', $event)`
- `port.gps_baudrate`
- `portFieldSet(port, 'peripheral', $event); onPeripheralChange(port);`
- `port.blackbox_baudrate`
- `saveConfig`

### Configuration

[ConfigurationTab.vue](https://github.com/betaflight/betaflight-configurator/blob/14a057ffc58417c5128199fc1233284982a64be3/src/components/tabs/ConfigurationTab.vue)

- PID loop frequency — `pidAdvancedConfig.pid_process_denom`
- Aircraft name — `craftName`
- Pilot name — `pilotName`
- FPV Camera Angle [degrees] — `fpvCamAngleDegrees`
- Maximum ARM Angle [degrees] — `armingConfig.small_angle`
- Calibrate Gyro on first arm — `armingConfig.gyro_cal_on_first_arm_bool`
- Disarm motors after delay [seconds] — `armingConfig.auto_disarm_delay`
- `(checked) => toggleFeature(feature, checked)`
- Beacon Tone — `dshotBeaconTone`
- `enableAllDshot`
- `disableAllDshot`
- `(checked) => toggleDshotCondition(cond, checked)`
- `enableAllBeepers`
- `disableAllBeepers`
- `(checked) => toggleBeeper(beeper, checked)`
- `saveConfig`

### Power and Battery

[PowerTab.vue](https://github.com/betaflight/betaflight-configurator/blob/14a057ffc58417c5128199fc1233284982a64be3/src/components/tabs/PowerTab.vue)

- Battery Profile — `onBatteryProfileChange`
- Profile Name — `batteryProfileName`
- Voltage Meter Source — `batteryConfig.voltageMeterSource`
- Current Meter Source — `batteryConfig.currentMeterSource`
- Minimum Cell Voltage — `batteryConfig.vbatmincellvoltage`
- Maximum Cell Voltage — `batteryConfig.vbatmaxcellvoltage`
- Warning Cell Voltage — `batteryConfig.vbatwarningcellvoltage`
- Capacity (mAh) — `batteryConfig.capacity`
- Scale — `voltageConfigs[index].vbatscale`
- Divider Value — `voltageConfigs[index].vbatresdivval`
- Multiplier Value — `voltageConfigs[index].vbatresdivmultiplier`
- Scale [1/10th mV/A] — `currentConfigs[index].scale`
- Offset [mA] — `currentConfigs[index].offset`
- `openCalibrationManager`
- `handleSave`
- `showCalibrationManagerDialog`
- Measured Voltage — `vbatcalibrationValue`
- Measured Amperage — `amperagecalibrationValue`
- `showCalibrationConfirmDialog`

### Failsafe

[FailsafeTab.vue](https://github.com/betaflight/betaflight-configurator/blob/14a057ffc58417c5128199fc1233284982a64be3/src/components/tabs/FailsafeTab.vue)

- Minimum length — `rxConfig.rx_min_usec`
- Maximum length — `rxConfig.rx_max_usec`
- `rxFailConfig[index].mode`
- `rxFailConfig[index].value`
- Failsafe Switch Action — `failsafeConfig.failsafe_switch_mode`
- Period of time in Stage 1 failsafe after signal loss [seconds] — `failsafeDelay`
- Failsafe Throttle Low Delay [seconds] — `failsafeThrottleLowDelay`
- `failsafeConfig.failsafe_procedure`
- Throttle value used while landing — `failsafeConfig.failsafe_throttle`
- Delay for turning off the Motors during Failsafe [seconds] — `failsafeOffDelay`
- Altitude mode — `gpsRescue.altitudeMode`
- Return altitude (meters) — `gpsRescue.returnAltitudeM`
- Initial climb (meters) — `gpsRescue.initialClimbM`
- Ascend rate (meters/second) — `gpsRescueAscendRate`
- Return ground speed (meters/second) — `gpsRescueGroundSpeed`
- Maximum pitch angle — `gpsRescue.angle`
- Descent distance (meters) — `gpsRescue.descentDistanceM`
- Descend rate (meters/second) — `gpsRescueDescendRate`
- Throttle minimum — `gpsRescue.throttleMin`
- Throttle maximum — `gpsRescue.throttleMax`
- Throttle hover - <span class="message-negative">IMPORTANT: set this value accurately</span> — `gpsRescue.throttleHover`
- Minimum distance to home (meters) — `gpsRescue.minStartDistM`
- Minimum satellites — `gpsRescue.minSats`
- Allow arming without fix - <span class="message-negative">WARNING: No fix = disarm on failsafe!</span> — `gpsRescueAllowArmingWithoutFix`
- Sanity checks — `gpsRescue.sanityChecks`
- `saveConfig`

### Presets

[PresetsTab.vue](https://github.com/betaflight/betaflight-configurator/blob/14a057ffc58417c5128199fc1233284982a64be3/src/components/tabs/PresetsTab.vue)

- `openSourcesDialog`
- `loadConfigBackup`
- `saveConfigBackup`
- `hideBackupWarning`
- `reloadPresets`
- `store.filters.categories`
- `store.filters.keywords`
- `store.filters.authors`
- `store.filters.firmwareVersions`
- `store.filters.status`
- `store.setSearchString($event)`
- `store.clearPickedPresets()`
- `applyPickedPresets()`
- `closeCliErrorsWithoutSaving`
- `saveAnywayAfterCliErrors`

[PresetFilterSelect.vue](https://github.com/betaflight/betaflight-configurator/blob/14a057ffc58417c5128199fc1233284982a64be3/src/components/tabs/presets/PresetFilterSelect.vue)

- `selectedItems`

[PresetCard.vue](https://github.com/betaflight/betaflight-configurator/blob/14a057ffc58417c5128199fc1233284982a64be3/src/components/tabs/presets/PresetCard.vue)

- `handleOpen`

[PresetDetailsDialog.vue](https://github.com/betaflight/betaflight-configurator/blob/14a057ffc58417c5128199fc1233284982a64be3/src/components/tabs/presets/PresetDetailsDialog.vue)

- `emit('toggle-cli-visible', true)`
- `emit('toggle-cli-visible', false)`
- `handleApply`
- `requestClose`

[PresetSourcesDialog.vue](https://github.com/betaflight/betaflight-configurator/blob/14a057ffc58417c5128199fc1233284982a64be3/src/components/tabs/presets/PresetSourcesDialog.vue)

- `handleAddSource`
- `emit('close')`

[PresetSourceCard.vue](https://github.com/betaflight/betaflight-configurator/blob/14a057ffc58417c5128199fc1233284982a64be3/src/components/tabs/presets/PresetSourceCard.vue)

- `!selected && emit('select')`
- `draft.name`
- `draft.url`
- `draft.gitHubBranch`
- `handleActivate`
- `handleDeactivate`
- `handleSave`
- `resetDraft`
- `emit('delete')`

### PID Tuning

[PidTuningTab.vue](https://github.com/betaflight/betaflight-configurator/blob/14a057ffc58417c5128199fc1233284982a64be3/src/components/tabs/PidTuningTab.vue)

- Profile — `currentProfile`
- Rateprofile — `currentRateProfile`
- PID profile name — `localProfileName`
- Rate profile name — `localRateProfileName`
- `copyProfile`
- `copyRateProfile`
- `resetProfile`
- `toggleShowAllPids`
- `activeSubtab`
- `onFormChanged`
- `refresh`
- `save`

[PidSubTab.vue](https://github.com/betaflight/betaflight-configurator/blob/14a057ffc58417c5128199fc1233284982a64be3/src/components/tabs/pid-tuning/PidSubTab.vue)

- `sliderPidsMode`
- `sliderDGain`
- `sliderPIGain`
- `sliderFeedforwardGain`
- `sliderDMaxGain`
- `sliderIGain`
- `sliderRollPitchRatio`
- `sliderPitchPIGain`
- `sliderMasterMultiplier`
- `pidRollP`
- `pidRollI`
- `pidRollD`
- `advancedTuning.dMaxRoll`
- `advancedTuning.feedforwardRoll`
- `pidPitchP`
- `pidPitchI`
- `pidPitchD`
- `advancedTuning.dMaxPitch`
- `advancedTuning.feedforwardPitch`
- `pidYawP`
- `pidYawI`
- `pidYawD`
- `advancedTuning.dMaxYaw`
- `advancedTuning.feedforwardYaw`
- `pidAltP`
- `pidAltI`
- `pidAltD`
- `pidVelP`
- `pidVelI`
- `pidVelD`
- `pidMagP`
- `pidPosP`
- `pidPosRP`
- `pidPosRI`
- `pidPosRD`
- `pidNavRP`
- `pidNavRI`
- `pidNavRD`
- `pidLevelAngle`
- `pidLevelHorizon`
- `pidLevelTransition`
- `advancedTuning.levelAngleLimit`
- `advancedTuning.feedforward_jitter_factor`
- `advancedTuning.feedforward_smooth_factor`
- `advancedTuning.feedforward_averaging`
- `advancedTuning.feedforward_boost`
- `advancedTuning.feedforward_max_rate_limit`
- `feedforwardTransitionValue`
- `tpaMode`
- `tpaRate`
- `tpaBreakpoint`
- I Term Relax — `itermRelaxEnabled`
- `advancedTuning.itermRelax`
- `advancedTuning.itermRelaxType`
- `advancedTuning.itermRelaxCutoff`
- Anti Gravity — `antiGravityEnabled`
- `advancedTuning.antiGravityMode`
- `antiGravityGainValue`
- `advancedTuning.itermThrottleThreshold`
- I Term Rotation — `itermRotationEnabled`
- `advancedTuning.dMaxGain`
- `advancedTuning.dMaxAdvance`
- Throttle Boost — `advancedTuning.throttleBoost`
- Motor Output Limit — `advancedTuning.motorOutputLimit`
- Dynamic Idle Value [* 100 RPM] — `advancedTuning.idleMinRpm`
- Vbat Sag Compensation % — `vbatSagEnabled`
- `advancedTuning.vbat_sag_compensation`
- Thrust Linearization % — `thrustLinearEnabled`
- `advancedTuning.thrustLinearization`
- Cell Count - for auto Profile switching — `advancedTuning.autoProfileCellCount`
- Acro Trainer Angle Limit — `advancedTuning.acroTrainerAngleLimit`
- Integrated Yaw — `integratedYawEnabled`
- Absolute Control — `advancedTuning.absoluteControlGain`

[RatesSubTab.vue](https://github.com/betaflight/betaflight-configurator/blob/14a057ffc58417c5128199fc1233284982a64be3/src/components/tabs/pid-tuning/RatesSubTab.vue)

- Rates Type — `ratesType`
- `rcRate = $event`
- `rollRate = $event`
- `rcExpo = $event`
- `rcRatePitch = $event`
- `pitchRate = $event`
- `rcPitchExpo = $event`
- `rcRateYaw = $event`
- `yawRate = $event`
- `rcYawExpo = $event`
- `throttleLimitType`
- `throttleLimitPercent`
- `throttleMid = $event`
- `throttleHover = $event`
- `throttleExpo = $event`

[FilterSubTab.vue](https://github.com/betaflight/betaflight-configurator/blob/14a057ffc58417c5128199fc1233284982a64be3/src/components/tabs/pid-tuning/FilterSubTab.vue)

- `gyroSliderEnabled`
- `gyroFilterMultiplier`
- `dtermSliderEnabled`
- `dtermFilterMultiplier`
- Gyro Lowpass 1 — `gyroLowpassEnabled`
- `gyroLowpassMode`
- `gyro_lowpass_hz`
- `gyro_lowpass_dyn_min_hz`
- `gyro_lowpass_dyn_max_hz`
- `gyro_lowpass_type`
- Gyro Lowpass 2 — `gyroLowpass2Enabled`
- `gyro_lowpass2_hz`
- `gyro_lowpass2_type`
- Gyro Notch Filter 1 — `gyroNotch1Enabled`
- `gyro_notch_hz`
- `gyro_notch_cutoff`
- Gyro Notch Filter 2 — `gyroNotch2Enabled`
- `gyro_notch2_hz`
- `gyro_notch2_cutoff`
- Gyro RPM Filter — `rpmFilterEnabled`
- `gyro_rpm_notch_harmonics`
- `gyro_rpm_notch_min_hz`
- `gyro_rpm_notch_q`
- `setRpmWeight(i - 1, $event)`
- `gyro_rpm_notch_fade_range_hz`
- Dynamic Notch Filter — `dynamicNotchEnabled`
- `dyn_notch_count`
- `dyn_notch_q`
- `dyn_notch_min_hz`
- `dyn_notch_max_hz`
- D Term Lowpass 1 — `dtermLowpassEnabled`
- `dtermLowpassMode`
- `dterm_lowpass_hz`
- `dterm_lowpass_dyn_min_hz`
- `dterm_lowpass_dyn_max_hz`
- `dyn_lpf_curve_expo`
- `dterm_lowpass_type`
- D Term Lowpass 2 — `dtermLowpass2Enabled`
- `dterm_lowpass2_hz`
- `dterm_lowpass2_type`
- D Term Notch Filter — `dtermNotchEnabled`
- `dterm_notch_hz`
- `dterm_notch_cutoff`
- Yaw Lowpass Filter — `yawLowpassEnabled`
- `yaw_lowpass_hz`

### Receiver

[ReceiverTab.vue](https://github.com/betaflight/betaflight-configurator/blob/14a057ffc58417c5128199fc1233284982a64be3/src/components/tabs/ReceiverTab.vue)

- Receiver Mode — `selectedRxMode`
- Serial Receiver Provider — `rxConfig.serialrx_provider`
- SPI Bus Receiver Provider — `rxConfig.rxSpiProtocol`
- Binding phrase — `elrsBindingPhrase`
- Model Match ID — `rxConfig.elrsModelId`
- Telemetry output — `(checked) => toggleTelemetry(checked)`
- Analog RSSI input — `(checked) => toggleRssiAdc(checked)`
- RSSI Channel — `rssiConfig.channel`
- `channelMapString`
- `channelMapPresetValue`
- `rxConfig.stick_min`
- `rxConfig.stick_center`
- `rxConfig.stick_max`
- `rcDeadbandConfig.deadband`
- `rcDeadbandConfig.yaw_deadband`
- `rcDeadbandConfig.deadband3d_throttle`
- RC Smoothing — `(on) => (rxConfig.rcSmoothing = on ? 1 : 0)`
- Setpoint Cutoff Type — `setpointManualMode`
- Setpoint Cutoff Frequency — `rxConfig.rcSmoothingSetpointCutoff`
- Setpoint Auto Factor — `rxConfig.rcSmoothingAutoFactor`
- Throttle Cutoff Type — `throttleManualMode`
- Throttle Cutoff Frequency — `rxConfig.rcSmoothingThrottleCutoff`
- Throttle Auto Factor — `rxConfig.rcSmoothingAutoFactorThrottle`
- Feedforward Cutoff Type — `feedforwardManualMode`
- Feedforward Cutoff Frequency — `rxConfig.rcSmoothingFeedforwardCutoff`
- `resetRefreshRate`
- `refreshRate`
- `openSticksWindow`
- `sendBind`
- `saveConfig(needReboot)`

### Modes

[AuxiliaryTab.vue](https://github.com/betaflight/betaflight-configurator/blob/14a057ffc58417c5128199fc1233284982a64be3/src/components/tabs/AuxiliaryTab.vue)

- Hide unused modes — `hideUnused`
- `addLink(mode)`
- `addRange(mode)`
- `entry.auxChannelIndex`
- `entry.modeLogic`
- `entry.sliderRange`
- `removeEntry(mode, entry.uid)`
- `entry.linkedTo`
- `saveModes`

### Adjustments

[AdjustmentsTab.vue](https://github.com/betaflight/betaflight-configurator/blob/14a057ffc58417c5128199fc1233284982a64be3/src/components/tabs/AdjustmentsTab.vue)

- `showAllSlots = !showAllSlots`
- `adjustment.enabled`
- `adjustment.auxChannelIndex`
- `adjustment.rangeArray`
- `adjustment.adjustmentFunction`
- `adjustment.auxSwitchChannelIndex`
- `onModeChange(adjustment, $event)`
- `adjustment.adjustmentCenter`
- `adjustment.adjustmentScale`
- `saveAdjustments`

### Servos

[ServosTab.vue](https://github.com/betaflight/betaflight-configurator/blob/14a057ffc58417c5128199fc1233284982a64be3/src/components/tabs/ServosTab.vue)

- `servo.min`
- `servo.middle`
- `servo.max`
- `setChannelForward(index, ch - 1, $event)`
- `servo.rate`
- `liveMode`
- `saveServoConfig`

### GPS

[GpsTab.vue](https://github.com/betaflight/betaflight-configurator/blob/14a057ffc58417c5128199fc1233284982a64be3/src/components/tabs/GpsTab.vue)

- `toggleFeature(feature, $event)`
- Protocol — `gpsConfig.provider`
- Auto Baud — `autoBaudChecked`
- Auto Config — `autoConfigChecked`
- Use Galileo — `ubloxGalileoChecked`
- Set Home Point Once — `homeOnceChecked`
- Ground Assistance Type — `gpsConfig.ublox_sbas`
- `checkConnectivity`
- `setLayer('satellite')`
- `setLayer('hybrid')`
- `setLayer('street')`
- `zoomIn`
- `zoomOut`
- `toggleFullscreen`
- `saveConfig`

### Motors

[MotorsTab.vue](https://github.com/betaflight/betaflight-configurator/blob/14a057ffc58417c5128199fc1233284982a64be3/src/components/tabs/MotorsTab.vue)

- `fcStore.mixerConfig.mixer`
- Motor direction is reversed — `reverseMotorDir`
- `openMotorOutputReorderDialog()`
- `openEscDshotDirectionDialog()`
- ESC/Motor protocol — `selectedEscProtocol`
- Motor PWM speed Separated from PID speed — `useUnsyncedPwm`
- Motor PWM frequency — `fcStore.pidAdvancedConfig.motor_pwm_rate`
- `toggleFeature('MOTOR_STOP', $event)`
- `toggleFeature('ESC_SENSOR', $event)`
- Bidirectional DShot (requires supported ESC firmware) — `useDshotTelemetry`
- $t(configurationMotorPoles.message) (number of magnets on the motor bell) — `fcStore.motorConfig.motor_poles`
- Motor Idle (%) — `fcStore.pidAdvancedConfig.motorIdle`
- Dynamic Idle Value [* 100 RPM] — `fcStore.advancedTuning.idleMinRpm`
- Minimum Command (ESC value when disarmed) — `fcStore.motorConfig.mincommand`
- Minimum Throttle (Lowest ESC value when armed) — `fcStore.motorConfig.minthrottle`
- Maximum Throttle (Highest ESC value when armed) — `fcStore.motorConfig.maxthrottle`
- `toggleFeature('3D', $event)`
- 3D Deadband Low — `fcStore.motor3dConfig.deadband3d_low`
- 3D Deadband High — `fcStore.motor3dConfig.deadband3d_high`
- 3D Neutral — `fcStore.motor3dConfig.neutral`
- `resetMaxValues`
- `sensorType`
- `sensorRate`
- `sensorScale`
- `onMotorValueUpdate(i - 1, $event)`
- `onMasterValueUpdate($event)`
- <strong>I understand the risks</strong>, the propellers are removed - enable motor control and arming, and disable Runaway Takeoff Prevention. — `motorsTestingEnabled`
- `settingsChangedOpen`
- `closeWarningDialog`
- `dialogDynFiltersOpen`
- `closeDynFiltersDialog`
- `applyDynFiltersChange`
- `stopMotors()`
- `handleSave(true)`

### OSD

[OsdTab.vue](https://github.com/betaflight/betaflight-configurator/blob/14a057ffc58417c5128199fc1233284982a64be3/src/components/tabs/OsdTab.vue)

- `elementSearchQuery`
- `toggleFieldVisibility(field.index, profileIdx - 1, $event)`
- `openPresetMenu(field, $event)`
- `showPresetSubmenu = !showPresetSubmenu`
- `showRulers`
- Active OSD Profile — `activeProfile`
- Font — `selectedFont`
- Alarm: — `timer.alarm`
- `entry.alarm.value`
- `selectedFontPreset`
- `loadCustomFontFile()`
- `replaceLogoImage()`
- `flashFont()`
- `openFontManager()`
- `saveConfig()`

### Video Transmitter

[VtxTab.vue](https://github.com/betaflight/betaflight-configurator/blob/14a057ffc58417c5128199fc1233284982a64be3/src/components/tabs/VtxTab.vue)

- Enter frequency directly — `frequencyMode`
- Band — `vtxConfig.vtx_band`
- Channel — `vtxConfig.vtx_channel`
- Frequency — `vtxConfig.vtx_frequency`
- Power — `vtxConfig.vtx_power`
- Pit Mode — `vtxConfig.vtx_pit_mode`
- Pit Mode frequency — `vtxConfig.vtx_pit_mode_frequency`
- Low Power Disarm — `vtxConfig.vtx_low_power_disarm`
- `setTableBands`
- `setTableChannels`
- `setBandName(bandIdx, String($event).toUpperCase())`
- `setBandLetter(bandIdx, String($event).toUpperCase())`
- `setBandFactory(bandIdx, $event)`
- `setBandChannelFreq(bandIdx, chIdx, $event)`
- `setTablePowerLevels`
- `setPowerLevelValue(i, $event)`
- `setPowerLevelLabel(i, String($event).toUpperCase())`
- `loadJsonFile`
- `saveJsonFile`
- `handleSave`

### LED Strip

[LedStripTab.vue](https://github.com/betaflight/betaflight-configurator/blob/14a057ffc58417c5128199fc1233284982a64be3/src/components/tabs/LedStripTab.vue)

- `clearSelected`
- `clearAll`
- `selectedFunction`
- `modifiers.throttleHue`
- `auxChannelValue`
- `modifiers.larsonScanner`
- `modifiers.blink`
- `modifiers.rainbow`
- `rainbowDelta`
- `rainbowFreq`
- `overlayStates.warnings`
- `overlayStates.indicator`
- `overlayStates.vtx`
- `modeColorsMode`
- `handleModeColorClick(modeColorsMode, i - 1)`
- `toggleDirection(dir)`
- `colorHSV.h`
- `colorHSV.s`
- `colorHSV.v`
- `handleColorClick(i - 1, $event)`
- `handleModeColorClick(6, config.direction)`
- `brightness`
- `toggleWireMode`
- `clearWiresSelected`
- `clearWiresAll`
- `save`

### Tethered Logging

[LoggingTab.vue](https://github.com/betaflight/betaflight-configurator/blob/14a057ffc58417c5128199fc1233284982a64be3/src/components/tabs/LoggingTab.vue)

- `toggleProperty(prop.code, $event)`
- Sampling Interval: — `samplingInterval`
- `selectLogFile`
- `toggleLogging`

### Blackbox

[OnboardLoggingTab.vue](https://github.com/betaflight/betaflight-configurator/blob/14a057ffc58417c5128199fc1233284982a64be3/src/components/tabs/OnboardLoggingTab.vue)

- Blackbox logging device — `blackboxDevice`
- Blackbox logging rate — `blackboxRate`
- Blackbox debug mode — `debugMode`
- `updateDebugField(index, $event)`
- `flashEraseCancel`
- `flashErase`
- `flashSaveCancel`
- `dismissSavingDialog`
- `saveSettings`

### Autotune

[AutotuneImport.vue](https://github.com/betaflight/betaflight-configurator/blob/14a057ffc58417c5128199fc1233284982a64be3/src/components/tabs/autotune/AutotuneImport.vue)

- `importAndAnalyze`

[BodePlot.vue](https://github.com/betaflight/betaflight-configurator/blob/14a057ffc58417c5128199fc1233284982a64be3/src/components/tabs/autotune/BodePlot.vue)

- `toggleAxis(axis.key, $event)`

[GainRecommendation.vue](https://github.com/betaflight/betaflight-configurator/blob/14a057ffc58417c5128199fc1233284982a64be3/src/components/tabs/autotune/GainRecommendation.vue)

- `selectedAxisKey`
- `onApply`

### Blackbox Viewer

The controls are generated by the host view. [Tab source](https://github.com/betaflight/betaflight-configurator/blob/14a057ffc58417c5128199fc1233284982a64be3/src/components/tabs/BlackboxViewerTab.vue)
