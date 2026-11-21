#pragma once

#include "Device/BaroDevice.hpp"

namespace Espfc::Device::Baro {

class BaroMS5611 : public BaroDevice
{
public:
  int begin(BusDevice* bus) override;
  int begin(BusDevice* bus, uint8_t addr) override;
  BaroDeviceType getType() const override { return BARO_MS5611; }
  bool testConnection() override;
  float readTemperature() override;
  float readPressure() override;
  void setMode(BaroDeviceMode mode) override;
  int getDelay(BaroDeviceMode) const override { return 10000; }

private:
  uint32_t readAdc(BaroDeviceMode mode);
  uint16_t _prom[8] = {};
  int64_t _offset = 0;
  int64_t _sensitivity = 0;
  BaroDeviceMode _mode = BARO_MODE_TEMP;
  bool _ready = false;
  bool _temperatureValid = false;
};

} // namespace Espfc::Device::Baro
