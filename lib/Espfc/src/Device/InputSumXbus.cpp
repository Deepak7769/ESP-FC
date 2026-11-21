#include "Device/InputSumXbus.hpp"
#include "Hal/Time.hpp"
#include <algorithm>

namespace Espfc::Device {

static uint16_t bigEndianWord(const uint8_t* bytes)
{
  return static_cast<uint16_t>(bytes[0]) << 8 | bytes[1];
}

static bool validCrc16(const uint8_t* data, size_t size)
{
  uint16_t crc = 0;
  for (size_t i = 0; i < size - 2; ++i)
  {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (unsigned bit = 0; bit < 8; ++bit)
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
  }
  return crc == bigEndianWord(data + size - 2);
}

static bool validRj01Crc(const uint8_t* data, size_t size)
{
  uint8_t crc = 0;
  for (size_t i = 0; i < size - 1; ++i)
  {
    crc ^= data[i];
    for (unsigned bit = 0; bit < 8; ++bit)
      crc = (crc & 1) ? (crc >> 1) ^ 0x8c : crc >> 1;
  }
  return crc == data[size - 1];
}

void InputSumXbus::begin(Stream::Readable* serial, SerialRXProvider provider)
{
  _serial = serial;
  _provider = provider;
  _used = _length = _channelCount = 0;
}

InputStatus InputSumXbus::update()
{
  if (!_serial || _serial->available() <= 0) return INPUT_IDLE;
  const uint32_t now = micros();
  const unsigned gapUs = _provider == SERIALRX_SUMD ? 4000 :
                         _provider == SERIALRX_SUMH ? 5000 : 8000;
  if (now - _lastReadUs > gapUs) _used = 0;
  _lastReadUs = now;

  // Stop at the first complete frame so a queued good frame cannot mask a
  // receiver failsafe indication. Bound work if the serial line is noisy.
  const int available = std::min(_serial->available(), static_cast<int>(sizeof(_frame)));
  for (int i = 0; i < available; ++i)
  {
    const int byte = _serial->read();
    if (byte < 0) break;
    const InputStatus status = accept(static_cast<uint8_t>(byte));
    if (status != INPUT_IDLE) return status;
  }
  return INPUT_IDLE;
}

InputStatus InputSumXbus::accept(uint8_t byte)
{
  if (_used == 0)
  {
    switch (_provider)
    {
      case SERIALRX_SUMD:
      case SERIALRX_SUMH:
        if (byte != 0xa8) return INPUT_IDLE;
        _length = _provider == SERIALRX_SUMH ? 21 : sizeof(_frame);
        break;
      case SERIALRX_XBUS_MODE_B:
        if (byte != 0xa1 && byte != 0xa2) return INPUT_IDLE;
        _length = byte == 0xa1 ? 27 : 35;
        break;
      case SERIALRX_XBUS_MODE_B_RJ01:
        if (byte != 0xa1) return INPUT_IDLE;
        _length = 33;
        break;
      default:
        return INPUT_IDLE;
    }
  }
  _frame[_used++] = byte;
  if (_provider == SERIALRX_SUMD && _used == 3)
  {
    if (byte < 4 || byte > 32)
    {
      _used = 0;
      return INPUT_IDLE;
    }
    _length = 5 + 2 * byte;
  }
  if (_used < _length) return INPUT_IDLE;
  _used = 0;
  const InputStatus status = decode();
  if (status != INPUT_RECEIVED) return status;

  const bool isSum = _provider == SERIALRX_SUMD || _provider == SERIALRX_SUMH;
  const size_t offset = isSum ? 3 : _provider == SERIALRX_XBUS_MODE_B_RJ01 ? 4 : 1;
  _channelCount = isSum ? (_provider == SERIALRX_SUMD ? _frame[2] : 8) :
                          (_frame[0] == 0xa2 ? 16 : 12);
  for (size_t i = 0; i < _channelCount; ++i)
  {
    const uint16_t word = bigEndianWord(_frame + offset + 2 * i);
    if (_provider == SERIALRX_SUMD) _channels[i] = word / 8;
    else if (_provider == SERIALRX_SUMH) _channels[i] = std::max(0, word * 5 / 32 - 375);
    else _channels[i] = 800 + (static_cast<uint32_t>(word) * 1400 >> 12);
  }
  return INPUT_RECEIVED;
}

InputStatus InputSumXbus::decode() const
{
  switch (_provider)
  {
    case SERIALRX_SUMD:
      if (!validCrc16(_frame, _length)) return INPUT_IDLE;
      if (_frame[1] == 0x81) return INPUT_FAILSAFE;
      return (_frame[1] == 1 || _frame[1] == 3) ? INPUT_RECEIVED : INPUT_IDLE;
    case SERIALRX_SUMH:
      return _frame[19] == 0 ? INPUT_RECEIVED : INPUT_IDLE;
    case SERIALRX_XBUS_MODE_B:
      return validCrc16(_frame, _length) ? INPUT_RECEIVED : INPUT_IDLE;
    case SERIALRX_XBUS_MODE_B_RJ01:
      return _frame[1] == 30 && _frame[3] == 0xa1 &&
             validRj01Crc(_frame, _length) && validCrc16(_frame + 3, 27)
               ? INPUT_RECEIVED : INPUT_IDLE;
    default:
      return INPUT_IDLE;
  }
}

uint16_t InputSumXbus::get(uint8_t channel) const
{
  return channel < _channelCount ? _channels[channel] : 0;
}

void InputSumXbus::get(uint16_t* channels, size_t count) const
{
  if (channels) std::copy_n(_channels, std::min(count, _channelCount), channels);
}

} // namespace Espfc::Device
