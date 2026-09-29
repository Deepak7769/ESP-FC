#ifndef UNIT_TEST

#include "Utils/Storage.h"
#include "ModelConfig.h"
#include <Arduino.h>
#include <cstddef>
#include <EEPROM.h>

#if defined(NO_GLOBAL_INSTANCES) || defined(NO_GLOBAL_EEPROM)
static EEPROMClass EEPROM;
#endif

namespace Espfc::Utils {

int Storage::begin()
{
  EEPROM.begin(EEPROM_SIZE);
  static_assert(sizeof(ModelConfig) <= EEPROM_SIZE, "ModelConfig Size too big");
  return 1;
}

StorageResult Storage::load(ModelConfig& config) const
{
  // return STORAGE_ERR_BAD_MAGIC;

  int addr = 0;
  uint8_t magic = EEPROM.read(addr++);
  if (EEPROM_MAGIC != magic)
  {
    return STORAGE_ERR_BAD_MAGIC;
  }

  const uint8_t version = EEPROM.read(addr++);

  uint16_t size = 0;
  size = EEPROM.read(addr++);
  size |= EEPROM.read(addr++) << 8;

  if (version == EEPROM_VERSION)
  {
    if (size != sizeof(ModelConfig))
    {
      return STORAGE_ERR_BAD_SIZE;
    }

    EEPROM.get(addr, config);
    return STORAGE_LOAD_SUCCESS;
  }

  if (version == EEPROM_VERSION_V3)
  {
    // v0x04 appends GPS-rescue, LED-strip and VTX-table compatibility state
    // to ConfiguratorCompatConfig. Load the exact v0x03 prefix and retain the
    // deterministic constructor defaults for the newly appended fields.
    constexpr size_t V4_ADDED_BYTES =
        sizeof(CompatGpsRescueConfig) +
        sizeof(CompatLedStripConfig) +
        sizeof(CompatVtxTableConfig);

    constexpr size_t LEGACY_V3_SIZE =
        sizeof(ModelConfig) -
        V4_ADDED_BYTES;

    if (size != LEGACY_V3_SIZE)
    {
      return STORAGE_ERR_BAD_SIZE;
    }

    uint8_t* dst =
        reinterpret_cast<uint8_t*>(&config);

    for (size_t i = 0; i < LEGACY_V3_SIZE; ++i)
    {
      dst[i] = EEPROM.read(addr + i);
    }

    return STORAGE_LOAD_SUCCESS;
  }

  if (version == EEPROM_VERSION_V2)
  {
    // v0x03 appends ConfiguratorCompatConfig to the legacy layout. Loading
    // exactly the prefix keeps every previous field/offset intact and leaves
    // the new tail at its constructor defaults.
    constexpr size_t LEGACY_V2_SIZE =
        offsetof(ModelConfig, compat);

    if (size != LEGACY_V2_SIZE)
    {
      return STORAGE_ERR_BAD_SIZE;
    }

    uint8_t* dst =
        reinterpret_cast<uint8_t*>(&config);

    for (size_t i = 0; i < LEGACY_V2_SIZE; ++i)
    {
      dst[i] = EEPROM.read(addr + i);
    }

    // IDs added after MODE_ANTI_GRAVITY were invalid/ignored in v0x02.
    // Scrub those stale rows so arbitrary old bytes cannot become newly-live
    // shadow mode requests after migration.
    for (size_t i = 0; i < ACTUATOR_CONDITIONS; ++i)
    {
      auto& condition = config.conditions[i];

      if ((condition.id >= MODE_HORIZON_SHADOW &&
           condition.id < MODE_COUNT) ||
          (condition.linkId >= MODE_HORIZON_SHADOW &&
           condition.linkId < MODE_COUNT))
      {
        condition = ActuatorCondition{};
      }
    }

    return STORAGE_LOAD_SUCCESS;
  }

  return STORAGE_ERR_BAD_VERSION;
}

StorageResult Storage::save(const ModelConfig& config)
{
  int addr = 0;
  uint16_t size = sizeof(ModelConfig);
  EEPROM.write(addr++, EEPROM_MAGIC);
  EEPROM.write(addr++, EEPROM_VERSION);
  EEPROM.write(addr++, size & 0xFF);
  EEPROM.write(addr++, (size >> 8) & 0xFF);
  EEPROM.put(addr, config);
  bool ok = EEPROM.commit();
  if (!ok) return STORAGE_SAVE_ERROR;
  return STORAGE_SAVE_SUCCESS;
}

} // namespace Espfc::Utils

#endif
