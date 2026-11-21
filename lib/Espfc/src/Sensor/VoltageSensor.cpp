#include "VoltageSensor.hpp"
#include "Hal/Time.hpp"

#include <algorithm>

namespace Espfc::Sensor {

VoltageSensor::VoltageSensor(Model& model): _model(model) {}

int VoltageSensor::begin()
{
  _model.state.battery.timer.setRate(100);
  _model.state.battery.samples = 50;
  _model.state.battery.cells = 0;
  _model.state.battery.integratedMah = 0;
  _integrationSource = CURRENT_METER_NONE;

  _model.state.battery.mspCurrentCentiAmps = 0;
  _model.state.battery.mspMahDrawn = 0;
  _model.state.battery.mspCurrentLastUpdateUs = 0;
  _model.state.battery.mspCurrentValid = false;

  reload(MODEL_CHANGE_FILTER);

  _state = VBAT;

  return 1;
}

int VoltageSensor::reload(ModelChangeEvent event)
{
  switch (event)
  {
    case MODEL_CHANGE_FILTER:
      _vFilterFast.begin(FilterConfig(FILTER_PT1, 20), _model.state.battery.timer.rate);
      _vFilter.begin(FilterConfig(FILTER_PT2, 2), _model.state.battery.timer.rate);
      _iFilterFast.begin(FilterConfig(FILTER_PT1, 20), _model.state.battery.timer.rate);
      _iFilter.begin(FilterConfig(FILTER_PT2, 2), _model.state.battery.timer.rate);
      break;
    default:
      break;
  }
  return 1;
}

int VoltageSensor::update()
{
  if (!_model.state.battery.timer.check()) return 0;

  Utils::Stats::Measure measure(_model.state.stats, COUNTER_BATTERY);

  switch (_state)
  {
    case VBAT:
      _state = IBAT;
      return readVbat();
    case IBAT:
      _state = VBAT;
      return readIbat();
  }

  return 0;
}

float VoltageSensor::escMeter(bool voltage) const
{
  if (!_model.config.output.dshotTelemetry) return 0.f;
  const uint32_t now = millis();
  const auto& telemetry = _model.state.output.telemetry;
  float sum = 0.f;
  unsigned count = 0;
  for (size_t i = 0; i < OUTPUT_CHANNELS; ++i)
  {
    if (_model.config.pin[PIN_OUTPUT_0 + i] < 0 || _model.config.output.channel[i].servo) continue;
    const uint32_t stamp = voltage ? telemetry.voltageTimeMs[i] : telemetry.currentTimeMs[i];
    if (!stamp || now - stamp > 500) continue;
    sum += voltage ? telemetry.voltage[i] * 0.25f : telemetry.current[i];
    ++count;
  }
  return voltage && count ? sum / count : sum;
}

int VoltageSensor::readVbat()
{
  float volts = 0.f;
  if (_model.config.vbat.source == 2)
    volts = escMeter(true);
#ifdef ESPFC_ADC_0
  else if (_model.config.vbat.source == 1 && _model.config.pin[PIN_INPUT_ADC_0] >= 0 && _model.config.vbat.resDiv)
  {
    _model.state.battery.rawVoltage = analogRead(_model.config.pin[PIN_INPUT_ADC_0]);
    volts = _vFilterFast.update(_model.state.battery.rawVoltage * ESPFC_ADC_SCALE);
    volts *= _model.config.vbat.scale * 0.1f * _model.config.vbat.resMult / _model.config.vbat.resDiv;
  }
#endif
  _model.state.battery.voltageUnfiltered = volts;
  _model.state.battery.voltage = _vFilter.update(_model.state.battery.voltageUnfiltered);

  auto& battery = _model.state.battery;
  const auto& profile = _model.config.batteryProfile;
  if (battery.voltage < 2.f)
  {
    if (battery.cells && !battery.samples) battery.integratedMah = 0.f;
    battery.cells = 0;
    battery.samples = 50;
    battery.cellVoltage = battery.percentage = 0.f;
  }
  else
  {
    if (battery.samples > 0)
    {
      battery.cells = std::clamp<int>(std::ceil(battery.voltage / (profile.cellMax * 0.01f)), 1, 32);
      --battery.samples;
    }
    battery.cellVoltage = battery.voltage / battery.cells;
    const int range = profile.cellMax - profile.cellMin;
    battery.percentage = range > 0
      ? std::clamp((battery.cellVoltage * 100.f - profile.cellMin) * 100.f / range, 0.f, 100.f)
      : 0.f;
  }

  if (_model.config.debug.mode == DEBUG_BATTERY)
  {
    _model.state.debug[0] = std::clamp<long>(lrintf(_model.state.battery.voltageUnfiltered * 100.0f), 0L, 32000L);
    _model.state.debug[1] = std::clamp<long>(lrintf(_model.state.battery.voltage * 100.0f), 0L, 32000L);
  }
  return 1;
}

void VoltageSensor::integrateCurrent(float amps, uint32_t now)
{
  const uint32_t deltaUs = now - _lastCurrentUs;
  if (_integrationSource == _model.config.ibat.source && deltaUs <= 500000)
    _model.state.battery.integratedMah += std::max(amps, 0.f) * deltaUs / 3600000.f;
  _lastCurrentUs = now;
  _integrationSource = _model.config.ibat.source;
}

int VoltageSensor::readIbat()
{
  auto& battery =
      _model.state.battery;

  if (_model.config.ibat.source == CURRENT_METER_ESC)
  {
    battery.current = battery.currentUnfiltered = escMeter(false);
    integrateCurrent(battery.current, micros());
    return 1;
  }
  if (_model.config.ibat.source == CURRENT_METER_VIRTUAL)
  {
    const auto& profile = _model.config.batteryProfile;
    float centiAmps = profile.virtualOffset;
    if (_model.isModeActive(MODE_ARMED) &&
        !(_model.isFeatureActive(FEATURE_MOTOR_STOP) && _model.isThrottleLow() && !_model.state.output.managedThrust))
    {
      const float throttle = std::clamp(_model.state.input.us[AXIS_THRUST] - 1000.f, 0.f, 1000.f);
      centiAmps += (throttle + throttle * throttle / 50.f) * profile.virtualScale * 0.001f;
    }
    battery.current = battery.currentUnfiltered = std::max(0.f, centiAmps * 0.01f);
    integrateCurrent(battery.current, micros());
    return 1;
  }
  if (_model.config.ibat.source != CURRENT_METER_ADC) _integrationSource = CURRENT_METER_NONE;

  if (_model.config.ibat.source == CURRENT_METER_NONE)
  {
    battery.current = battery.currentUnfiltered = 0.f;
    return 0;
  }
  if (_model.config.ibat.source ==
      CURRENT_METER_MSP)
  {
    constexpr uint32_t MSP_CURRENT_STALE_US =
        500000;

    const uint32_t now =
        micros();

    const bool fresh =
        battery.mspCurrentValid &&
        static_cast<uint32_t>(
            now -
            battery.mspCurrentLastUpdateUs) <
            MSP_CURRENT_STALE_US;

    if (!fresh)
    {
      // Never keep publishing an old companion current indefinitely. Current
      // telemetry is deliberately fail-silent and has no control authority.
      battery.mspCurrentValid =
          false;

      battery.currentUnfiltered =
          0.0f;

      battery.current =
          0.0f;

      _iFilter.prime(
          0.0f);

      return 0;
    }

    const float amps =
        static_cast<float>(
            battery.mspCurrentCentiAmps) *
        0.01f;

    battery.rawCurrent =
        battery.mspCurrentCentiAmps;

    battery.currentUnfiltered =
        amps;

    battery.current =
        _iFilter.update(
            amps);

    if (_model.config.debug.mode ==
        DEBUG_CURRENT_SENSOR)
    {
      _model.state.debug[0] =
          battery.mspCurrentCentiAmps;

      _model.state.debug[1] =
          std::clamp<long>(
              lrintf(
                  battery.current *
                  100.0f),
              -32000L,
              32000L);

      _model.state.debug[2] =
          static_cast<uint16_t>(
              battery.mspMahDrawn);
    }

    return 1;
  }

#ifdef ESPFC_ADC_1
  if (_model.config.ibat.source !=
          CURRENT_METER_ADC ||
      _model.config.pin[
          PIN_INPUT_ADC_1] == -1)
  {
    return 0;
  }

  battery.rawCurrent =
      analogRead(
          _model.config.pin[
              PIN_INPUT_ADC_1]);

  float volts =
      _iFilterFast.update(
          battery.rawCurrent *
          ESPFC_ADC_SCALE);

  const float milivolts =
      volts *
      1000.0f;

  volts +=
      _model.config.ibat.offset *
      0.001f;

  volts *=
      _model.config.ibat.scale *
      0.1f;

  battery.currentUnfiltered =
      volts;

  battery.current =
      _iFilter.update(
          battery.currentUnfiltered);

  integrateCurrent(battery.current, micros());

  if (_model.config.debug.mode ==
      DEBUG_CURRENT_SENSOR)
  {
    _model.state.debug[0] =
        lrintf(
            milivolts);

    _model.state.debug[1] =
        std::clamp<long>(
            lrintf(
                battery.currentUnfiltered *
                100.0f),
            -32000L,
            32000L);

    _model.state.debug[2] =
        battery.rawCurrent;
  }

  return 1;
#else
  return 0;
#endif
}

} // namespace Espfc::Sensor
