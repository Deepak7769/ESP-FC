#include "BaroBMP085.hpp"
#include "Hal/Time.hpp"
#include <cmath>

#define BMP085_DEFAULT_ADDRESS 0x77
#define BMP085_WHOAMI_ID 0x55

#define BMP085_CALIB_REG 0xAA
#define BMP085_WHOAMI_REG 0xD0
#define BMP085_RESET_REG 0xE0
#define BMP085_CONTROL_REG 0xF4
#define BMP085_MEASUREMENT_REG 0xF6

#define BMP085_MEASURE_T 0x2E  // temperature measurent
#define BMP085_MEASURE_P0 0x34 // pressure measurement no oversampling
#define BMP085_MEASURE_P1 0x74 // pressure measurement oversampling x2
#define BMP085_MEASURE_P2 0xB4 // pressure measurement oversampling x4
#define BMP085_MEASURE_P3 0xF4 // pressure measurement oversampling x8

namespace Espfc::Device::Baro {

int BaroBMP085::begin(BusDevice* bus)
{
  return begin(bus, BMP085_DEFAULT_ADDRESS);
}

int BaroBMP085::begin(BusDevice* bus, uint8_t addr)
{
  setBus(bus, addr);

  if (!testConnection()) return 0;

  _temperatureValid = false;
  _conversionStarted = false;
  if (!_bus->writeByte(_addr, BMP085_RESET_REG, 0xB6)) return 0;
  delay(10);

  uint8_t data[22];
  if (_bus->read(_addr, BMP085_CALIB_REG, sizeof(data), data) != sizeof(data)) return 0;
  uint16_t words[11];
  for (unsigned i = 0; i < 11; ++i)
  {
    words[i] = static_cast<uint16_t>(data[2*i]) << 8 | data[2*i+1];
    if (words[i] == 0 || words[i] == 0xffff) return 0;
  }
  _cal = {static_cast<int16_t>(words[0]), static_cast<int16_t>(words[1]), static_cast<int16_t>(words[2]),
          words[3], words[4], words[5], static_cast<int16_t>(words[6]), static_cast<int16_t>(words[7]),
          static_cast<int16_t>(words[8]), static_cast<int16_t>(words[9]), static_cast<int16_t>(words[10])};

  return 1;
}

BaroDeviceType BaroBMP085::getType() const
{
  return BARO_BMP085;
}

float BaroBMP085::readTemperature()
{
  _temperatureValid = false;
  uint8_t buffer[2];
  if (!_conversionStarted || _mode != BMP085_MEASURE_T ||
      _bus->read(_addr, BMP085_MEASUREMENT_REG, 2, buffer) != 2) return NAN;
  const int32_t ut = (buffer[0] << 8) | buffer[1];
  if (ut == 0 || ut == 0xffff) return NAN;
  const int64_t x1 = (static_cast<int64_t>(ut - _cal.ac6) * _cal.ac5) >> 15;
  if (x1 + _cal.md == 0) return NAN;
  const int64_t fine = x1 + (static_cast<int64_t>(_cal.mc) * 2048) / (x1 + _cal.md);
  const float temperature = ((fine + 8) >> 4) * 0.1f;
  if (temperature < -40.f || temperature > 85.f) return NAN;
  _t_fine = fine;
  _temperatureValid = true;
  return temperature;
}

float BaroBMP085::readPressure()
{
  uint8_t buffer[3];
  if (!_temperatureValid || !_conversionStarted || _mode == BMP085_MEASURE_T ||
      _bus->read(_addr, BMP085_MEASUREMENT_REG, 3, buffer) != 3) return NAN;
  const unsigned oss = _mode >> 6;
  const int64_t up = ((buffer[0] << 16) | (buffer[1] << 8) | buffer[2]) >> (8 - oss);
  const int64_t b6 = _t_fine - 4000;
  const int64_t squared = (b6 * b6) >> 12;
  int64_t x1 = (_cal.b2 * squared) >> 11;
  int64_t x2 = (_cal.ac2 * b6) >> 11;
  const int64_t b3 = (((_cal.ac1 * 4 + x1 + x2) * (1 << oss)) + 2) >> 2;
  x1 = (_cal.ac3 * b6) >> 13;
  x2 = (_cal.b1 * squared) >> 16;
  const int64_t b4 = (_cal.ac4 * (((x1 + x2 + 2) >> 2) + 32768)) >> 15;
  if (b4 <= 0 || up <= b3) return NAN;
  const int64_t b7 = (up - b3) * (50000 >> oss);
  int64_t pressure = b7 < 0x80000000LL ? b7 * 2 / b4 : b7 / b4 * 2;
  if (pressure <= 0 || pressure > 150000) return NAN;
  x1 = ((pressure >> 8) * (pressure >> 8) * 3038) >> 16;
  x2 = (-7357 * pressure) >> 16;
  return pressure + ((x1 + x2 + 3791) >> 4);
}

void BaroBMP085::setMode(BaroDeviceMode mode)
{
  _mode = mode == BARO_MODE_TEMP ? BMP085_MEASURE_T : BMP085_MEASURE_P3;
  _conversionStarted = _bus->writeByte(_addr, BMP085_CONTROL_REG, _mode);
}

int BaroBMP085::getDelay(BaroDeviceMode mode) const
{
  switch (mode)
  {
    case BARO_MODE_TEMP:
      return 4550; // temp
    default:
      // return 4550;  // press_0
      // return 7550;  // press_1
      // return 13550; // press_2
      return 25550; // press_3
  }
}

bool BaroBMP085::testConnection()
{
  uint8_t whoami = 0;
  if (_bus->readByte(_addr, BMP085_WHOAMI_REG, &whoami) != 1) return false;
  setChipId(whoami);
  if (whoami != BMP085_WHOAMI_ID) return false;
  return true;
}

} // namespace Espfc::Device::Baro
