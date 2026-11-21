#pragma once

#include "Device/BaroDevice.hpp"

namespace Espfc::Device::Baro {

class BaroLPS : public BaroDevice
{
public:
  int begin(BusDevice* bus) override;
  int begin(BusDevice* bus, uint8_t addr) override;
  BaroDeviceType getType() const override { return BARO_LPS; }
  bool testConnection() override;
  float readTemperature() override;
  float readPressure() override;
  // In continuous mode both measurements become available every 40 ms.
  void setMode(BaroDeviceMode) override {}
  int getDelay(BaroDeviceMode mode) const override { return mode == BARO_MODE_TEMP ? 40000 : 1; }

private:
  bool readOutput(uint8_t readyBit, uint8_t reg, uint8_t* bytes, unsigned count);
  bool _running = false;
};

} // namespace Espfc::Device::Baro
