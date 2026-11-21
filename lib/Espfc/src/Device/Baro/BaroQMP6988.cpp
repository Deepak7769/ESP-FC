#include "BaroQMP6988.hpp"
#include "Hal/Time.hpp"
#include <algorithm>

namespace Espfc::Device::Baro {
namespace {
int32_t big24(const uint8_t* data)
{
  return (int32_t(data[0]) << 16) | (data[1] << 8) | data[2];
}

int32_t signedValue(int32_t raw, int32_t sign)
{
  return (raw ^ sign) - sign;
}
}

bool BaroQMP6988::testConnection()
{
  uint8_t id;
  if (!_bus || _bus->readByte(_addr, 0xd1, &id) != 1) return false;
  setChipId(id);
  return id == 0x5c;
}

int BaroQMP6988::begin(BusDevice* bus)
{
  return begin(bus, 0x70) || begin(bus, 0x56);
}

int BaroQMP6988::begin(BusDevice* bus, uint8_t addr)
{
  ready = pending = false;
  pressure = NAN;
  setBus(bus, addr);
  if (!bus || (!bus->isSPI() && bus->getType() != BUS_I2C) || !testConnection()) return 0;
  uint8_t status = 1;
  for (unsigned attempt = 0; attempt < 20 && (status & 1); ++attempt)
  {
    if (bus->readByte(addr, 0xf3, &status) != 1) return 0;
    if (status & 1) delay(1);
  }
  if (status & 1) return 0;
  const uint8_t size = _type == BARO_QMP6988 ? 25 : 21;
  uint8_t trim[25];
  if (bus->read(addr, 0xa0, size, trim) != size ||
      std::all_of(trim, trim + size, [](uint8_t b) { return b == 0; }) ||
      std::all_of(trim, trim + size, [](uint8_t b) { return b == 0xff; })) return 0;
  loadTrim(trim);
  // Leave chip filtering off. The flight controller filters each complete sample.
  ready = bus->writeByte(addr, 0xf4, 0) && bus->writeByte(addr, 0xf1, 0) &&
          bus->readByte(addr, 0xf1, &status) == 1 && (status & 7) == 0;
  return ready;
}

void BaroQMP6988::loadTrim(const uint8_t* trim)
{
  auto word = [&](int i) { return (trim[i] << 8) | trim[i + 1]; };
  auto coefficient = [&](int i) { return signedValue(word(i), 0x8000) / 32767.f; };
  if (_type == BARO_QMP6988)
  {
    qmp.b00 = signedValue((word(0) << 4) | (trim[24] >> 4), 0x80000) / 16.f;
    qmp.a0 = signedValue((word(18) << 4) | (trim[24] & 15), 0x80000) / 16.f;
    qmp.a1 = -0.00630f + 0.000430f * coefficient(20);
    qmp.a2 = -1.9e-11f + 1.2e-10f * coefficient(22);
    qmp.bt1 = 0.1f + 0.091f * coefficient(2);
    qmp.bt2 = 1.2e-8f + 1.2e-6f * coefficient(4);
    qmp.bp1 = 0.033f + 0.019f * coefficient(6);
    qmp.b11 = 2.1e-7f + 1.4e-7f * coefficient(8);
    qmp.bp2 = -6.3e-10f + 3.5e-10f * coefficient(10);
    qmp.b12 = 2.9e-13f + 7.6e-13f * coefficient(12);
    qmp.b21 = 2.1e-15f + 1.2e-14f * coefficient(14);
    qmp.bp3 = 1.3e-16f + 7.9e-17f * coefficient(16);
  }
  else
  {
    omron.cp = signedValue(big24(trim), 0x800000);
    omron.bp = 30.f + 10.f * coefficient(3);
    omron.ap = 3e-5f * coefficient(5);
    omron.ct = 0.04f + 0.0085f * coefficient(7);
    omron.bt = -6.6e-6f + 1.6e-6f * coefficient(9);
    omron.at = 8e-11f * coefficient(11);
    omron.ca = signedValue(big24(trim + 13), 0x800000);
    omron.ba = -160.f + 8.f * coefficient(17);
    omron.aa = 0.00042f * coefficient(19);
  }
}

float BaroQMP6988::compensate(float rawP, float rawT)
{
  if (_type == BARO_QMP6988)
  {
    const float t = qmp.a0 + rawT * (qmp.a1 + rawT * qmp.a2);
    pressure = qmp.b00 + t * (qmp.bt1 + t * qmp.bt2) +
        rawP * (qmp.bp1 + t * (qmp.b11 + t * qmp.b12) +
                rawP * (qmp.bp2 + t * qmp.b21 + rawP * qmp.bp3));
    return t / 256.f;
  }

  // Rationalized quadratic roots retain precision when a trim coefficient is zero.
  const float dt = rawT - omron.ca;
  const float t = 2.f * dt / (omron.ba - std::sqrt(omron.ba * omron.ba + 4.f * omron.aa * dt));
  const float dp = rawP - omron.cp;
  const float linear = 2.f * dp / (omron.bp + std::sqrt(omron.bp * omron.bp + 4.f * omron.ap * dp));
  pressure = linear / (1.f + omron.ct + t * (omron.bt + t * omron.at));
  return t / 256.f;
}

int BaroQMP6988::getDelay(BaroDeviceMode mode) const
{
  if (mode == BARO_MODE_PRESS) return 1;
  return _type == BARO_QMP6988 ? 24000 : 35000;
}

void BaroQMP6988::setMode(BaroDeviceMode mode)
{
  if (mode != BARO_MODE_TEMP) return;
  pressure = NAN;
  pending = ready && _bus->writeByte(_addr, 0xf4, _type == BARO_QMP6988 ? 0x31 : 0x75);
}

float BaroQMP6988::readTemperature()
{
  if (!pending) return NAN;
  pending = false;
  uint8_t status, frame[6];
  if (_bus->readByte(_addr, 0xf3, &status) != 1 || (status & 9) ||
      _bus->read(_addr, 0xf7, 6, frame) != 6) return NAN;
  return compensate(big24(frame) - 0x800000, big24(frame + 3) - 0x800000);
}

float BaroQMP6988::readPressure()
{
  const float sample = pressure;
  pressure = NAN;
  return sample;
}

} // namespace Espfc::Device::Baro
