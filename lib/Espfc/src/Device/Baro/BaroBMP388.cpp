#include "BaroBMP388.hpp"
#include <algorithm>

namespace Espfc::Device::Baro {

bool BaroBMP388::read(uint8_t reg, uint8_t size, uint8_t* dest)
{
  if (!_bus) return false;
  if (!_bus->isSPI()) return _bus->read(_addr, reg, size, dest) == size;
  uint8_t buffer[22]; // the 21-byte trim block plus the SPI dummy byte
  if (size >= sizeof(buffer) || _bus->read(_addr, reg, size + 1, buffer) != size + 1) return false;
  std::copy_n(buffer + 1, size, dest);
  return true;
}

bool BaroBMP388::testConnection()
{
  uint8_t id;
  if (!read(0x00, 1, &id)) return false;
  setChipId(id);
  return id == 0x50 || id == 0x60;
}

int BaroBMP388::begin(BusDevice* bus)
{
  return begin(bus, 0x76) || begin(bus, 0x77);
}

int BaroBMP388::begin(BusDevice* bus, uint8_t addr)
{
  ready = pending = false;
  pressure = NAN;
  setBus(bus, addr);
  if (!bus || (!bus->isSPI() && bus->getType() != BUS_I2C)) return 0;
  uint8_t value;
  if (bus->isSPI()) read(0, 1, &value); // first CS transition selects SPI after power-up
  if (!testConnection()) return 0;
  uint8_t trim[21];
  if (!read(0x31, sizeof(trim), trim) ||
      std::all_of(trim, trim + sizeof(trim), [](uint8_t b) { return b == 0; }) ||
      std::all_of(trim, trim + sizeof(trim), [](uint8_t b) { return b == 0xff; })) return 0;
  auto word = [&](int index) -> uint16_t { return trim[index] | (trim[index + 1] << 8); };
  // Bosch BMP388 datasheet, calibration coefficient signedness and normalization.
  t1 = word(0) * 256.f;
  t2 = std::ldexp(float(word(2)), -30);
  t3 = std::ldexp(float(int8_t(trim[4])), -48);
  p1 = std::ldexp(float(int16_t(word(5)) - 16384), -20);
  p2 = std::ldexp(float(int16_t(word(7)) - 16384), -29);
  p3 = std::ldexp(float(int8_t(trim[9])), -32);
  p4 = std::ldexp(float(int8_t(trim[10])), -37);
  p5 = word(11) * 8.f;
  p6 = std::ldexp(float(word(13)), -6);
  p7 = std::ldexp(float(int8_t(trim[15])), -8);
  p8 = std::ldexp(float(int8_t(trim[16])), -15);
  p9 = std::ldexp(float(int16_t(word(17))), -48);
  p10 = std::ldexp(float(int8_t(trim[19])), -48);
  p11 = std::ldexp(float(int8_t(trim[20])), -65);
  ready = bus->writeByte(addr, 0x1b, 0) && bus->writeByte(addr, 0x1c, 3) &&
          read(0x1c, 1, &value) && value == 3;
  return ready;
}

void BaroBMP388::setMode(BaroDeviceMode mode)
{
  if (mode != BARO_MODE_TEMP) return;
  pressure = NAN;
  pending = ready && _bus->writeByte(_addr, 0x1b, 0x13);
}

float BaroBMP388::readTemperature()
{
  if (!pending) return NAN;
  pending = false;
  uint8_t status, frame[6];
  if (!read(0x03, 1, &status) || (status & 0xef) != 0x60 || !read(0x04, 6, frame)) return NAN;
  auto adc = [&](int i) { return float(frame[i] | (frame[i + 1] << 8) | (frame[i + 2] << 16)); };
  const float dt = adc(3) - t1;
  const float t = dt * (t2 + dt * t3);
  const float raw = adc(0);
  const float offset = p5 + t * (p6 + t * (p7 + t * p8));
  const float sensitivity = p1 + t * (p2 + t * (p3 + t * p4));
  pressure = offset + raw * (sensitivity + raw * (p9 + t * p10 + raw * p11));
  return t;
}

float BaroBMP388::readPressure()
{
  const float sample = pressure;
  pressure = NAN;
  return sample;
}

} // namespace Espfc::Device::Baro
