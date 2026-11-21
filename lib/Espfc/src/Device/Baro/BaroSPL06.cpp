#include "BaroSPL06.hpp"
#include "Hal/Time.hpp"
#include <algorithm>
#include <cmath>

namespace Espfc::Device::Baro {
namespace {
constexpr uint8_t RESULT = 0x00;
constexpr uint8_t STATUS = 0x08;
constexpr float SCALE_8X = 7864320.f;

int32_t signed24(const uint8_t* data)
{
  const int32_t raw = (data[0] << 16) | (data[1] << 8) | data[2];
  return raw & 0x800000 ? raw - 0x1000000 : raw;
}
}

int BaroSPL06::begin(BusDevice* bus)
{
  return begin(bus, 0x76) || begin(bus, 0x77);
}

int BaroSPL06::begin(BusDevice* bus, uint8_t addr)
{
  setBus(bus, addr);
  _running = _haveTemperature = false;
  if (!bus || !testConnection() || !bus->writeByte(addr, 0x0c, 0x89)) return 0;
  delay(50);
  uint8_t status;
  if (bus->readByte(addr, STATUS, &status) != 1 || (status & 0xc0) != 0xc0) return 0;

  uint8_t buffer[18];
  // Short bursts also fit the MPU auxiliary I2C master transfer count.
  if (bus->read(addr, 0x10, 9, buffer) != 9 || bus->read(addr, 0x19, 9, buffer + 9) != 9 ||
      std::all_of(buffer, buffer+18, [](uint8_t b){ return b == 0; }) ||
      std::all_of(buffer, buffer+18, [](uint8_t b){ return b == 0xff; })) return 0;

  _cal.c0 = (int16_t)buffer[0] << 4 | buffer[1] >> 4;
  _cal.c0 = (_cal.c0 & 0x0800) ? (_cal.c0 | 0xF000) : _cal.c0;

  _cal.c1 = (int16_t)(buffer[1] & 0x0F) << 8 | buffer[2];
  _cal.c1 = (_cal.c1 & 0x0800) ? (_cal.c1 | 0xF000) : _cal.c1;

  _cal.c00 = (int32_t)buffer[3] << 12 | (int32_t)buffer[4] << 4 | (int32_t)buffer[5] >> 4;
  _cal.c00 = (_cal.c00 & 0x080000) ? (_cal.c00 | 0xFFF00000) : _cal.c00;

  _cal.c10 = (int32_t)(buffer[5] & 0x0F) << 16 | (int32_t)buffer[6] << 8 | (int32_t)buffer[7];
  _cal.c10 = (_cal.c10 & 0x080000) ? (_cal.c10 | 0xFFF00000) : _cal.c10;

  _cal.c01 = (int16_t)buffer[8] << 8 | buffer[9];
  _cal.c11 = (int16_t)buffer[10] << 8 | buffer[11];
  _cal.c20 = (int16_t)buffer[12] << 8 | buffer[13];
  _cal.c21 = (int16_t)buffer[14] << 8 | buffer[15];
  _cal.c30 = (int16_t)buffer[16] << 8 | buffer[17];

  uint8_t source = 0x80; // The SPL06-001 is calibrated for the MEMS temperature sensor.
  if (_type == BARO_DPS310 && bus->readByte(addr, 0x28, &source) != 1) return 0;
  // Both conversions at 8x oversampling fit within a 32 Hz measurement cycle.
  if (!bus->writeByte(addr, 0x06, 0x53) || !bus->writeByte(addr, 0x07, 0x53 | (source & 0x80)) ||
      !bus->writeByte(addr, 0x09, 0) || !bus->writeByte(addr, STATUS, 7)) return 0;

  if (bus->getType() == BUS_SLV && bus->read(addr, RESULT, 6, buffer) != 6) return 0;
  _running = true;
  return 1;
}

bool BaroSPL06::testConnection()
{
  uint8_t id;
  if (!_bus || _bus->readByte(_addr, 0x0d, &id) != 1) return false;
  setChipId(id);
  return id == 0x10;
}

float BaroSPL06::readTemperature()
{
  return _haveTemperature ? _temperature : NAN;
}

float BaroSPL06::readPressure()
{
  _haveTemperature = false;
  if (!_running) return NAN;
  const bool bridge = _bus->getType() == BUS_SLV;
  uint8_t status;
  if (!bridge && (_bus->readByte(_addr, STATUS, &status) != 1 || (status & 0x30) != 0x30)) return NAN;

  uint8_t data[6];
  const int count = bridge ? _bus->readFast(_addr, RESULT, 6, data) : _bus->read(_addr, RESULT, 6, data);
  if (count != 6) return NAN;
  const float p = signed24(data) / SCALE_8X;
  const float t = signed24(data+3) / SCALE_8X;
  _temperature = _cal.c0 * 0.5f + _cal.c1 * t;
  if (_temperature < -40.f || _temperature > 85.f) return NAN;
  _haveTemperature = true;
  return _cal.c00 + p * (_cal.c10 + p * (_cal.c20 + p * _cal.c30)) +
         t * (_cal.c01 + p * (_cal.c11 + p * _cal.c21));
}

void BaroSPL06::setMode(BaroDeviceMode) {}

int BaroSPL06::getDelay(BaroDeviceMode mode) const
{
  return mode == BARO_MODE_PRESS ? 31250 : 1;
}

} // namespace Espfc::Device::Baro
