#include "Sensor/SurfaceSensor.hpp"

#include <Arduino.h>
#if defined(ESP32)
#include <driver/gpio.h>
#endif
#include <algorithm>
#include <cstring>

namespace Espfc::Sensor {
namespace {

constexpr uint32_t TIMEOUT_US = 200000;
constexpr uint16_t MAX_CM[] = {0, 400, 1200, 2200, 800, 250, 1200, 600, 1400};

bool isTf(uint8_t type)
{
  return type == RANGE_TFMINI || type == RANGE_TF02 || type == RANGE_TFNOVA;
}

int16_t debugValue(int32_t value)
{
  return std::clamp<int32_t>(value, INT16_MIN, INT16_MAX);
}

} // namespace

bool SurfaceSensor::rangeActive(const Model& model)
{
  const auto& surface = model.state.surface;
  return surface.source != RANGE_NONE && surface.source == model.config.surface.rangefinder &&
         static_cast<uint32_t>(micros() - surface.rangeTimeUs) <= TIMEOUT_US;
}

bool SurfaceSensor::flowActive(const Model& model)
{
  const auto& surface = model.state.surface;
  return model.config.surface.opticalFlow && surface.flowPresent &&
         static_cast<uint32_t>(micros() - surface.flowTimeUs) <= TIMEOUT_US;
}

void SurfaceSensor::recordRange(Model& model, uint8_t type, int32_t mm, uint32_t now)
{
  if (type == RANGE_NONE || type > RANGE_TFNOVA || type != model.config.surface.rangefinder) return;
  auto& surface = model.state.surface;
  surface.source = type;
  surface.rangeTimeUs = now;
  surface.distanceMm = mm >= 0 && mm <= MAX_CM[type] * 10 ? mm : -1;
  const int16_t cm = surface.distanceMm < 0 ? -1 : surface.distanceMm / 10;
  model.setDebug(DEBUG_RANGEFINDER, 1, cm);
  model.setDebug(DEBUG_RANGEFINDER_QUALITY, 0, cm);
}

void SurfaceSensor::recordFlow(Model& model, uint8_t quality, int32_t x, int32_t y, uint32_t now)
{
  auto& surface = model.state.surface;
  // MT reports flow in milliradians/s. A fresh distance of at least 80 mm
  // is required for a usable measurement on this family of sensors.
  const auto type = model.config.surface.rangefinder;
  const uint32_t maxAge = (type == RANGE_MTF02 || type == RANGE_MTF02P) ? 100000 : 50000;
  const bool validRange = surface.source == type && surface.distanceMm >= 80 &&
                          now - surface.rangeTimeUs <= maxAge;
  const uint32_t delta = surface.flowPresent ? now - surface.flowTimeUs : 0;
  surface.flowPresent = true;
  surface.flowTimeUs = now;
  surface.flowQuality = validRange ? quality * 100u / 255u : 0;
  surface.flowX = x;
  surface.flowY = y;
  model.setDebug(DEBUG_OPTICALFLOW, 0, surface.flowQuality);
  model.setDebug(DEBUG_OPTICALFLOW, 1, debugValue(x));
  model.setDebug(DEBUG_OPTICALFLOW, 2, debugValue(y));
  model.setDebug(DEBUG_OPTICALFLOW, 5, std::min<uint32_t>(delta, INT16_MAX));
}

void SurfaceSensor::beginUltrasonic()
{
  const auto& cfg = _model.config.surface;
  if (cfg.rangefinder != RANGE_HCSR04 || cfg.triggerPin < 0 || cfg.echoPin < 0 ||
      cfg.triggerPin == cfg.echoPin || !_model.validatePinResources()) return;
#if defined(ESP32)
  if (!GPIO_IS_VALID_OUTPUT_GPIO(cfg.triggerPin) || !GPIO_IS_VALID_GPIO(cfg.echoPin)) return;
#elif defined(ESP8266)
  if (cfg.triggerPin > 16 || cfg.echoPin > 15) return;
#elif defined(ARCH_RP2040)
  if (cfg.triggerPin >= NUM_BANK0_GPIOS || cfg.echoPin >= NUM_BANK0_GPIOS) return;
#endif
  _trigger = cfg.triggerPin;
  _echo = cfg.echoPin;
  digitalWrite(_trigger, LOW);
  pinMode(_trigger, OUTPUT);
  pinMode(_echo, INPUT);
#if defined(ARCH_RP2040)
  attachInterruptParam(_echo, echoInterrupt, CHANGE, this);
#elif !defined(UNIT_TEST)
  attachInterruptArg(_echo, echoInterrupt, this, CHANGE);
#endif
}

void IRAM_ATTR SurfaceSensor::echoInterrupt(void* arg)
{
  auto& sensor = *static_cast<SurfaceSensor*>(arg);
  const uint32_t now = micros();
  if (digitalRead(sensor._echo))
  {
    sensor._rise = now;
    sensor._rising = true;
  }
  else if (sensor._rising)
  {
    sensor._pulseWidth = now - sensor._rise;
    sensor._rising = false;
  }
}

int32_t SurfaceSensor::echoDistanceMm(uint32_t widthUs)
{
  // HC-SR04 timing is 58 microseconds/cm; minimum measuring distance is 2 cm.
  if (widthUs < 116 || widthUs > 23200) return -1;
  return (widthUs * 10 + 29) / 58;
}

void SurfaceSensor::beginSerial(Stream::ReadWritable* port)
{
  _port = port;
  _length = 0;
  configureSerial(micros());
}

void SurfaceSensor::configureSerial(uint32_t now)
{
  const auto type = _model.config.surface.rangefinder;
  if (_port && (type == RANGE_TFMINI || type == RANGE_TF02))
  {
    constexpr uint8_t sample100Hz[] = {0x42,0x57,0x02,0x00,0x00,0x00,0x01,0x06};
    _port->write(sample100Hz, sizeof(sample100Hz));
  }
  _lastConfigure = now;
}

void SurfaceSensor::parse(uint8_t byte, uint32_t now)
{
  if (_length < 2 && byte != 0x59) { _length = 0; return; }
  _frame[_length++] = byte;
  if (_length < sizeof(_frame)) return;
  uint8_t sum = 0;
  for (size_t i = 0; i < sizeof(_frame) - 1; ++i) sum += _frame[i];
  if (sum != _frame[8])
  {
    // A partial/bad packet may already contain the next packet's header.
    size_t pos = 1;
    while (pos < 9 && !(_frame[pos] == 0x59 && (pos == 8 || _frame[pos + 1] == 0x59))) ++pos;
    _length = sizeof(_frame) - pos;
    std::memmove(_frame, _frame + pos, _length);
    return;
  }
  _length = 0;
  const uint8_t type = _model.config.surface.rangefinder;
  int32_t cm = _frame[2] | (_frame[3] << 8);
  _model.setDebug(DEBUG_LIDAR_TF, 0, debugValue(cm));
  _model.setDebug(DEBUG_LIDAR_TF, 1, debugValue(_frame[4] | (_frame[5] << 8)));
  _model.setDebug(DEBUG_LIDAR_TF, 2, _frame[6]);
  _model.setDebug(DEBUG_LIDAR_TF, 3, _frame[7]);
  if (cm < (type == RANGE_TFNOVA ? 10 : 40) || cm > MAX_CM[type] ||
      (type == RANGE_TFNOVA && _frame[7] < 90) || (type == RANGE_TF02 && _frame[6] < 7))
    cm = -1;
  else if (type == RANGE_TFMINI && _frame[6] == 7)
    cm -= 13;
  recordRange(_model, type, cm * 10, now);
}

void SurfaceSensor::update(uint32_t now)
{
  auto& surface = _model.state.surface;
  if (surface.source != RANGE_NONE && now - surface.rangeTimeUs > TIMEOUT_US)
  {
    surface.source = RANGE_NONE;
    surface.distanceMm = -1;
    _model.setDebug(DEBUG_RANGEFINDER, 1, -1);
  }
  if (surface.flowPresent && now - surface.flowTimeUs > TIMEOUT_US)
  {
    surface.flowPresent = false;
    surface.flowQuality = 0;
    _model.setDebug(DEBUG_OPTICALFLOW, 0, 0);
  }
  if (_trigger >= 0 && _model.config.surface.rangefinder == RANGE_HCSR04 && now - _lastTrigger >= 60000)
  {
    // Read the previous ping before starting the next. The echo ISR never waits.
    const uint32_t width = _pulseWidth;
    _pulseWidth = 0;
    _rising = false;
    if (width) recordRange(_model, RANGE_HCSR04, echoDistanceMm(width), now);
    digitalWrite(_trigger, HIGH);
    delayMicroseconds(10);
    digitalWrite(_trigger, LOW);
    _lastTrigger = now;
  }
  if (!_port || !isTf(_model.config.surface.rangefinder)) return;
  uint8_t bytes[64];
  const int available = _port->available();
  if (available > 0)
  {
    const auto count = _port->readMany(bytes, std::min<size_t>(available, sizeof(bytes)));
    for (size_t i = 0; i < count; ++i) parse(bytes[i], now);
  }
  if (surface.source == RANGE_NONE && now - _lastConfigure >= 500000) configureSerial(now);
}

} // namespace Espfc::Sensor
