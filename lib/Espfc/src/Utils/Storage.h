#pragma once

namespace Espfc {

enum StorageResult
{

  STORAGE_NONE,
  STORAGE_LOAD_SUCCESS,
  STORAGE_SAVE_SUCCESS,
  STORAGE_SAVE_ERROR,
  STORAGE_ERR_BAD_MAGIC,
  STORAGE_ERR_BAD_VERSION,
  STORAGE_ERR_BAD_SIZE,
};

}

#include "ModelConfig.h"
#include <cstddef>

namespace Espfc::Utils {

class Storage
{
public:
  // The stored schemas share an append-only prefix. Each board reads its own native
  // layout, including its target-specific channel and serial port counts.
  template <typename Reader>
  static StorageResult readConfig(ModelConfig& config, Reader read)
  {
    if (read(0) != EEPROM_MAGIC) return STORAGE_ERR_BAD_MAGIC;
    const uint8_t version = read(1);
    if (version < 2 || version > EEPROM_VERSION) return STORAGE_ERR_BAD_VERSION;
    const size_t size = read(2) | (static_cast<unsigned>(read(3)) << 8);
    size_t expected = sizeof(ModelConfig);
    if (version == 2) expected = offsetof(ModelConfig, batteryProfile);
    if (version == 3) expected = offsetof(ModelConfig, rateProfiles);
    if (version == 4) expected = offsetof(ModelConfig, pidProfiles);
    if (version == 5) expected = offsetof(ModelConfig, dMaxProfiles);
    if (size != expected) return STORAGE_ERR_BAD_SIZE;
    auto* bytes = reinterpret_cast<uint8_t*>(&config);
    for (size_t i = 0; i < size; ++i) bytes[i] = read(i + 4);
    if (version == 2)
    {
      config.batteryProfile = {};
      config.gpsReceiver = {};
      config.surface = {};
      config.throttleCurve = {};
      for (auto& routing : config.servoRouting) routing = {};
      for (auto& rates : config.serialSensorRates) rates = {};
    }
    if (version < 4 || config.rateProfiles.selected >= RateProfileBank::COUNT) config.initializeRateProfiles();
    for (auto& name : config.rateProfiles.names) name[sizeof(name) - 1] = 0;
    if (version < 5 || config.pidProfiles.selected >= PidProfileBank::COUNT) config.initializePidProfiles();
    for (auto& name : config.pidProfiles.names) name[sizeof(name) - 1] = 0;
    if (version < 6)
    {
      for (auto& configDMax : config.dMaxProfiles) configDMax = {};
      for (auto& angle : config.magAlignment) angle = 0;
    }
    return STORAGE_LOAD_SUCCESS;
  }

  int begin();
  StorageResult load(ModelConfig& config) const;
  StorageResult save(const ModelConfig& config);

private:
  static constexpr uint8_t EEPROM_MAGIC = 0xA5;
  static constexpr uint8_t EEPROM_VERSION = 0x06;

  static constexpr size_t EEPROM_SIZE = 2048;
};

} // namespace Espfc::Utils
