#include "BaroLPS.hpp"
#include "Hal/Time.hpp"
#include <cmath>

namespace Espfc::Device::Baro {
namespace {
constexpr uint8_t LPS25HB = 0xbd;
constexpr uint8_t LPS22HB = 0xb1;
constexpr uint8_t LPS22DF = 0xb4;
}

bool BaroLPS::testConnection()
{
  uint8_t id;
  if (!_bus || _bus->readByte(_addr, 0x0f, &id) != 1) return false;
  setChipId(id);
  return id == LPS25HB || id == LPS22HB || id == LPS22DF;
}

int BaroLPS::begin(BusDevice* bus)
{
  return begin(bus, 0x5d) || begin(bus, 0x5c);
}

int BaroLPS::begin(BusDevice* bus, uint8_t addr)
{
  _running = false;
  setBus(bus, addr);
  if (!bus || (bus->getType() != BUS_I2C && !bus->isSPI()) || !testConnection()) return 0;
  const auto id = *getChipId();
  const uint8_t ctrl1 = id == LPS25HB ? 0x20 : 0x10;
  const uint8_t ctrl2 = ctrl1 + 1;
  if (!bus->writeByte(addr, ctrl2, 0x04)) return 0;
  uint8_t value = 0x04;
  for (unsigned retry = 0; retry < 10 && (value & 0x04); ++retry)
  {
    delay(1);
    if (bus->readByte(addr, ctrl2, &value) != 1) return 0;
  }
  if (value & 0x04) return 0;

  // Block data update freezes each result until its most significant byte is read.
  // LPS22HB requires single-register I2C reads with BDU and auto-increment disabled.
  uint8_t config = 0xc4; // LPS25HB, active, 25 Hz, BDU
  if (id == LPS22HB)
  {
    if (!bus->writeByte(addr, ctrl2, 0)) return 0;
    config = 0x32; // 25 Hz, BDU
  }
  else if (id == LPS22DF)
  {
    if (!bus->writeByte(addr, ctrl2, 0x08)) return 0; // BDU
    config = 0x22; // 25 Hz, average 16
  }
  _running = bus->writeByte(addr, ctrl1, config) &&
             bus->readByte(addr, ctrl1, &value) == 1 && value == config;
  return _running;
}

bool BaroLPS::readOutput(uint8_t readyBit, uint8_t reg, uint8_t* bytes, unsigned count)
{
  uint8_t status;
  if (!_running || _bus->readByte(_addr, 0x27, &status) != 1 || !(status & readyBit)) return false;
  for (unsigned i = 0; i < count; ++i)
    if (_bus->readByte(_addr, reg + i, &bytes[i]) != 1) return false;
  return true;
}

float BaroLPS::readTemperature()
{
  uint8_t bytes[2];
  if (!readOutput(2, 0x2b, bytes, 2)) return NAN;
  const auto raw = static_cast<int16_t>(bytes[0] | (bytes[1] << 8));
  return *getChipId() == LPS25HB ? 42.5f + raw / 480.f : raw * 0.01f;
}

float BaroLPS::readPressure()
{
  uint8_t bytes[3];
  if (!readOutput(1, 0x28, bytes, 3) || (bytes[2] & 0x80)) return NAN;
  const int32_t raw = bytes[0] | (bytes[1] << 8) | (bytes[2] << 16);
  return raw ? raw * (100.f / 4096.f) : NAN;
}

} // namespace Espfc::Device::Baro
