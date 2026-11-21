#pragma once

#include "Debug_Espfc.h"
#include "Device/BaroDevice.hpp"

namespace Espfc::Device::Baro {

class BaroBMP085 : public BaroDevice
{
public:
  struct CalibrationData
  {
    int16_t ac1;
    int16_t ac2;
    int16_t ac3;
    uint16_t ac4;
    uint16_t ac5;
    uint16_t ac6;
    int16_t b1;
    int16_t b2;
    int16_t mb;
    int16_t mc;
    int16_t md;
  };

  int begin(BusDevice* bus) final;
  int begin(BusDevice* bus, uint8_t addr) final;
  BaroDeviceType getType() const final;

  float readTemperature() final;
  float readPressure() final;

  void setMode(BaroDeviceMode mode);
  int getDelay(BaroDeviceMode mode) const final;

  bool testConnection() final;

protected:
  uint8_t _mode = 0;
  int32_t _t_fine = 0;
  bool _temperatureValid = false;
  bool _conversionStarted = false;
  CalibrationData _cal;
};

} // namespace Espfc::Device::Baro
