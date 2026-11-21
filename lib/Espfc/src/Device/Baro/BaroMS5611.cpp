#include "BaroMS5611.hpp"
#include "Hal/Time.hpp"
#include <cmath>

namespace Espfc::Device::Baro {

int BaroMS5611::begin(BusDevice* bus)
{
  return begin(bus, 0x77) || begin(bus, 0x76);
}

int BaroMS5611::begin(BusDevice* bus, uint8_t addr)
{
  setBus(bus, addr);
  _temperatureValid = _ready = false;
  // Verify the PROM before issuing reset on a bus shared with other chips.
  if (!bus || bus->getType() == BUS_SLV || !testConnection() || !bus->write(addr, 0x1e, 0, nullptr)) return 0;
  delay(3);
  return testConnection();
}

bool BaroMS5611::testConnection()
{
  uint16_t remainder = 0;
  for (unsigned i = 0; i < 8; ++i)
  {
    uint8_t bytes[2];
    if (_bus->readCommand(_addr, 0xa0 + 2*i, 2, bytes) != 2) return false;
    _prom[i] = (bytes[0] << 8) | bytes[1];
    if (i >= 1 && i <= 6 && (_prom[i] == 0 || _prom[i] == 0xffff)) return false;
    if (i == 7) bytes[1] = 0;
    for (uint8_t byte : bytes)
    {
      remainder ^= byte;
      for (unsigned bit = 0; bit < 8; ++bit)
        remainder = (remainder & 0x8000) ? (remainder << 1) ^ 0x3000 : remainder << 1;
    }
  }
  return (remainder >> 12) == (_prom[7] & 0x0f);
}

void BaroMS5611::setMode(BaroDeviceMode mode)
{
  _mode = mode;
  _ready = _bus->write(_addr, mode == BARO_MODE_TEMP ? 0x58 : 0x48, 0, nullptr);
}

uint32_t BaroMS5611::readAdc(BaroDeviceMode mode)
{
  uint8_t bytes[3];
  if (!_ready || _mode != mode || _bus->readCommand(_addr, 0, 3, bytes) != 3) return 0;
  const uint32_t value = (bytes[0] << 16) | (bytes[1] << 8) | bytes[2];
  return value == 0xffffff ? 0 : value;
}

float BaroMS5611::readTemperature()
{
  _temperatureValid = false;
  const auto raw = readAdc(BARO_MODE_TEMP);
  if (!raw) return NAN;
  const int64_t dt = static_cast<int64_t>(raw) - (_prom[5] * 256);
  int64_t temp = 2000 + ((dt * _prom[6]) >> 23);
  _offset = (static_cast<int64_t>(_prom[2]) << 16) + ((dt * _prom[4]) >> 7);
  _sensitivity = (static_cast<int64_t>(_prom[1]) << 15) + ((dt * _prom[3]) >> 8);
  if (temp < 2000)
  {
    const int64_t cold = 5 * (temp - 2000) * (temp - 2000);
    _offset -= cold >> 1;
    _sensitivity -= cold >> 2;
    if (temp < -1500)
    {
      const int64_t veryCold = (temp + 1500) * (temp + 1500);
      _offset -= 7 * veryCold;
      _sensitivity -= (11 * veryCold) >> 1;
    }
    temp -= (dt * dt) >> 31;
  }
  if (temp < -4000 || temp > 8500) return NAN;
  _temperatureValid = true;
  return temp * 0.01f;
}

float BaroMS5611::readPressure()
{
  const auto raw = readAdc(BARO_MODE_PRESS);
  if (!raw || !_temperatureValid) return NAN;
  return (((raw * _sensitivity) >> 21) - _offset) >> 15;
}

} // namespace Espfc::Device::Baro
