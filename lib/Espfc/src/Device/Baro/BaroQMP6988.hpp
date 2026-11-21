#pragma once

#include "Device/BaroDevice.hpp"
#include <cmath>

namespace Espfc::Device::Baro {

// QMP6988 and 2SMPB-02B use the same measurement protocol and chip ID.
// The user must specify the part because the factory trim formats differ.
class BaroQMP6988 : public BaroDevice
{
public:
  void setType(BaroDeviceType type) { _type = type; }
  BaroDeviceType getType() const override { return _type; }
  int begin(BusDevice* bus) override;
  int begin(BusDevice* bus, uint8_t addr) override;
  bool testConnection() override;
  float readTemperature() override;
  float readPressure() override;
  void setMode(BaroDeviceMode mode) override;
  int getDelay(BaroDeviceMode mode) const override;

private:
  void loadTrim(const uint8_t* trim);
  float compensate(float rawP, float rawT);
  BaroDeviceType _type = BARO_QMP6988;
  struct { float a0, a1, a2, b00, bt1, bt2, bp1, b11, bp2, b12, b21, bp3; } qmp;
  struct { float cp, bp, ap, ct, bt, at, ca, ba, aa; } omron;
  float pressure = NAN;
  bool ready = false;
  bool pending = false;
};

} // namespace Espfc::Device::Baro
