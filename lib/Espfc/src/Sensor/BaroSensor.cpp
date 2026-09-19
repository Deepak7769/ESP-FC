#include "Sensor/BaroSensor.hpp"
#include "Hal/Time.hpp"
#include <cmath>
#include <algorithm>

namespace Espfc::Sensor {

BaroSensor::BaroSensor(Model& model):
    _model(model)
{
}
int BaroSensor::begin()
{
  auto& baro =
      _model.state.baro;

  // Deterministic runtime reset.
  _baro = nullptr;

  _state =
      BARO_STATE_INIT;

  _wait =
      0;

  _counter =
      0;

  _biasAlpha =
      0.0f;

  _first =
      true;

  _lastAltitudeUs =
      0;

  baro.sampleValid =
      false;

  baro.lastUpdateUs =
      0;

  baro.vario =
      0.0f;

  baro.altitudePrev =
      0.0f;

  baro.altitudeBias =
      0.0f;

  baro.altitudeBiasSamples =
      0;

  if (!_model.baroActive() ||
      !baro.dev)
  {
    return 0;
  }

  _baro =
      baro.dev;

const int gyroInterval =
    std::max<int>(
        _model.state.gyro.timer.interval,
        1);

const int delay =
    std::max<int>(
        _baro->getDelay(
            BARO_MODE_TEMP) +
        _baro->getDelay(
            BARO_MODE_PRESS),
        1);

const int toGyroRate =
    (delay /
     gyroInterval) +
    1;

const int interval =
    std::max(
        gyroInterval *
            toGyroRate,
        1);

const int rate =
    std::max(
        1000000 /
            interval,
        1);

  baro.rate =
      rate;

  const float dt =
      1.0f /
      static_cast<float>(rate);

  const float tau =
      0.8f;

  _biasAlpha =
      1.0f -
      expf(-dt / tau);

  baro.altitudeBiasSamples =
      3 * rate;

  reload(MODEL_CHANGE_FILTER);

  _model.logger.info()
      .log("BARO INIT")
      .log(
          Device::BaroDevice::getName(
              _baro->getType()))
      .logln(rate);

  _baro->setMode(
      BARO_MODE_TEMP);

  return 1;
}

int BaroSensor::reload(ModelChangeEvent event)
{
  switch (event)
  {
    case MODEL_CHANGE_FILTER: {
      const int rate =
    std::max<int>(
        _model.state.baro.rate,
        1);
      const auto internalFilter = FILTER_PT1;
      const auto internalCutoff = std::max((rate + 2) / 4, 1);
      _temperatureFilter.begin(FilterConfig(internalFilter, internalCutoff), rate);
      _pressureFilter.begin(FilterConfig(internalFilter, internalCutoff), rate);
      _varioFilter.begin(FilterConfig(internalFilter, internalCutoff), rate);
      break;
    }
    default:
      break;
  }
  return 1;
}

int BaroSensor::update()
{
  int status = read();

  return status;
}

int BaroSensor::read()
{
  if (!_baro || !_model.baroActive()) return 0;

 const uint32_t now =
    micros();

if ((int32_t)(now - _wait) < 0)
{
  return 0;
}

  Utils::Stats::Measure measure(_model.state.stats, COUNTER_BARO);

  // if(_model.config.debug.mode == DEBUG_BARO)
  // {
  //   _model.state.debug[0] = _state;
  // }

  switch (_state)
  {
    case BARO_STATE_INIT:
      _baro->setMode(BARO_MODE_TEMP);
      _state = BARO_STATE_TEMP_GET;
      _wait = micros() + _baro->getDelay(BARO_MODE_TEMP);
      return 0;
    case BARO_STATE_TEMP_GET:
      readTemperature();
      _baro->setMode(BARO_MODE_PRESS);
      _state = BARO_STATE_PRESS_GET;
      _wait = micros() + _baro->getDelay(BARO_MODE_PRESS);
      _counter = 1;
      return 1;
case BARO_STATE_PRESS_GET:
{
  const bool pressureValid =
      readPressure();

  if (pressureValid)
  {
    updateAltitude();
  }

  if (--_counter > 0)
  {
    _baro->setMode(
        BARO_MODE_PRESS);

    _state =
        BARO_STATE_PRESS_GET;

    _wait =
        micros() +
        _baro->getDelay(
            BARO_MODE_PRESS);
  }
  else
  {
    _baro->setMode(
        BARO_MODE_TEMP);

    _state =
        BARO_STATE_TEMP_GET;

    _wait =
        micros() +
        _baro->getDelay(
            BARO_MODE_TEMP);
  }

  return pressureValid ? 1 : 0;
}
    default:
      _state = BARO_STATE_INIT;
      break;
  }

  return 0;
}

void BaroSensor::readTemperature()
{
  float temp = _model.state.baro.temperatureRaw = _baro->readTemperature();
  _model.state.baro.temperature = _temperatureFilter.update(temp);
}

bool BaroSensor::readPressure()
{
  constexpr float
      MIN_PRESSURE_PA =
          1000.0f;

  constexpr float
      MAX_PRESSURE_PA =
          120000.0f;

  const float press =
      _baro->readPressure();

  if (!std::isfinite(press) ||
      press < MIN_PRESSURE_PA ||
      press > MAX_PRESSURE_PA)
  {
    // Keep the previous valid state.
    // Freshness timeout handles persistent failure.
    return false;
  }

  const float filteredPressure =
      _pressureFilter.update(
          press);

  if (!std::isfinite(
          filteredPressure) ||
      filteredPressure <
          MIN_PRESSURE_PA ||
      filteredPressure >
          MAX_PRESSURE_PA)
  {
    return false;
  }

  _model.state.baro.pressureRaw =
      press;

  _model.state.baro.pressure =
      filteredPressure;

  _model.state.baro.sampleValid =
      true;

  return true;
}
void BaroSensor::updateAltitude()
{
  auto& baro = _model.state.baro;

  baro.altitudeRaw = Utils::toAltitude(baro.pressure);
  float altitude = baro.altitudeRaw;

  if (baro.altitudeBiasSamples > 0)
  {
    baro.altitudeBiasSamples--;
    baro.altitudeBias += (altitude - baro.altitudeBias) * _biasAlpha;
  }
  else if (baro.altitudeBiasSamples == 0)
  {
    _model.logger.info().log("BARO BIAS").logln(baro.altitudeBias);
    baro.altitudeBiasSamples--;
  }

  baro.altitudeGround = altitude - baro.altitudeBias;
  baro.altitude = altitude;
const uint32_t altitudeNow =
    micros();
  
  baro.lastUpdateUs = altitudeNow;

if (_first)
{
  baro.altitudePrev =
      altitude;

  _lastAltitudeUs =
      altitudeNow;

  baro.vario =
      0.f;

  _first =
      false;
}
else
{
  const uint32_t elapsedUs =
      altitudeNow -
      _lastAltitudeUs;

  _lastAltitudeUs =
      altitudeNow;

  if (elapsedUs > 0)
  {
    const float dt =
        elapsedUs * 0.000001f;

    const float rawVario =
        (altitude -
         baro.altitudePrev) /
        dt;

    baro.vario =
        _varioFilter.update(
            rawVario);
  }

  baro.altitudePrev =
      altitude;
}

  if (_model.config.debug.mode == DEBUG_BARO)
  {
    _model.state.debug[0] = lrintf(baro.vario * 100.0f);     // cm/s
    _model.state.debug[1] = lrintf(baro.pressureRaw * 0.1f); // hPa x 10
    //_model.state.debug[1] = lrintf(baro.pressureRaw - 100000.0f); // Pa - 100000
    _model.state.debug[2] = lrintf(baro.temperatureRaw * 100.f); // deg C x 100
    _model.state.debug[3] = lrintf(baro.altitudeGround * 100.f); // cm
  }
}

} // namespace Espfc::Sensor
