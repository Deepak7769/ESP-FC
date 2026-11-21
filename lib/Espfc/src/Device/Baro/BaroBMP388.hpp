#pragma once

#include "Device/BaroDevice.hpp"
#include <cmath>

namespace Espfc::Device::Baro {

class BaroBMP388 : public BaroDevice
{
public:
  int begin(BusDevice* bus) override;
  int begin(BusDevice* bus, uint8_t addr) override;
  BaroDeviceType getType() const override { return BARO_BMP388; }
  bool testConnection() override;
  float readTemperature() override;
  float readPressure() override;
  void setMode(BaroDeviceMode mode) override;
  int getDelay(BaroDeviceMode mode) const override { return mode == BARO_MODE_TEMP ? 25000 : 1; }

private:
  bool read(uint8_t reg, uint8_t size, uint8_t* dest);
  float t1, t2, t3, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11;
  float pressure = NAN;
  bool ready = false;
  bool pending = false;
};

} // namespace Espfc::Device::Baro
