#pragma once

#include "Device/InputDevice.h"
#include "Device/SerialDevice.h"
#include "Stream/Readable.hpp"

namespace Espfc::Device {

// Graupner SUM and JR XBus are receive-only, big-endian serial channel frames.
class InputSumXbus : public InputDevice
{
public:
  void begin(Stream::Readable* serial, SerialRXProvider provider);
  InputStatus update() override;
  uint16_t get(uint8_t channel) const override;
  void get(uint16_t* channels, size_t count) const override;
  size_t getChannelCount() const override { return _channelCount; }
  bool needAverage() const override { return false; }

private:
  InputStatus accept(uint8_t byte);
  InputStatus decode() const;
  Stream::Readable* _serial = nullptr;
  SerialRXProvider _provider = SERIALRX_NONE;
  // Largest SUMD frame is a 3-byte header, 32 words and a CRC16.
  uint8_t _frame[69] = {};
  uint16_t _channels[32] = {};
  size_t _used = 0;
  size_t _length = 0;
  size_t _channelCount = 0;
  uint32_t _lastReadUs = 0;
};

} // namespace Espfc::Device
