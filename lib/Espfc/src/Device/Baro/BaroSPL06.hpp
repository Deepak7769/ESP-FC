#pragma once

#include "Debug_Espfc.h"
#include "Device/BaroDevice.hpp"

namespace Espfc::Device::Baro {

class BaroSPL06 : public BaroDevice
{
public:
  struct CalibrationData
  {
    int16_t c0;
    int16_t c1;
    int32_t c00;
    int32_t c10;
    int16_t c01;
    int16_t c11;
    int16_t c20;
    int16_t c21;
    int16_t c30;
  } __attribute__((__packed__));

  int begin(BusDevice* bus) final;
  int begin(BusDevice* bus, uint8_t addr) final;

  void setType(BaroDeviceType type) { _type = type; }
  BaroDeviceType getType() const final { return _type; }

  float readTemperature() final;
  float readPressure() final;

  void setMode(BaroDeviceMode mode);
  virtual int getDelay(BaroDeviceMode mode) const final;

  bool testConnection() final;

private:
  BaroDeviceType _type = BARO_SPL06;
  CalibrationData _cal = {};
  float _temperature = 0;
  bool _haveTemperature = false;
  bool _running = false;
};

} // namespace Espfc::Device::Baro
