#include "Vtx.hpp"
#include "Utils/Crc.hpp"

static const uint8_t dummyByte[] = {0x00};

namespace Espfc::Connect {

int Vtx::begin(Stream::ReadWritable* serial)
{
  _serial = serial;
  _timer.setInterval(300000);
  _wasArmed = _model.isModeActive(MODE_ARMED);
  _lastChannel = _lastBand = _lastPower = 0xff;

  _state = State::INIT;
  return 1;
}

int Vtx::update()
{
  if (!_serial) return 0;
  _wasArmed |= _model.isModeActive(MODE_ARMED);
  if (!_timer.check()) return 1;
  const auto& cfg = _model.config.vtx;
  if (_state == State::INIT)
  {
    _model.state.vtx.active = true;
    _state = State::IDLE;
  }
  if (cfg.band != _lastBand || cfg.channel != _lastChannel)
  {
    _lastBand = cfg.band;
    _lastChannel = cfg.channel;
    setChannel();
  }
  else if (_lastPower != desiredPower())
  {
    _lastPower = desiredPower();
    setPower();
  }

  return 1;
}

int Vtx::setChannel()
{
  if (!_serial || !_model.config.vtx.band || _model.config.vtx.band > 5 ||
      !_model.config.vtx.channel || _model.config.vtx.channel > 8) return 0;
  uint8_t band = _model.config.vtx.band - 1;
  uint8_t channel = _model.config.vtx.channel - 1;
  uint8_t code = static_cast<uint8_t>(band * 8 + channel);
  uint8_t vtxCommand[6] = {0xAA, 0x55, 0x07, 0x01, code};
  vtxCommand[5] = Utils::crc8_dvb_s2(0, vtxCommand, 5);
  _serial->write(dummyByte, 1);
  _serial->write(vtxCommand, 6);
  _serial->flush();

  return 1;
}

uint8_t Vtx::desiredPower() const
{
  const auto& cfg = _model.config.vtx;
  const bool reduce = !_model.isModeActive(MODE_ARMED) &&
                      (cfg.lowPowerDisarm == 1 || (cfg.lowPowerDisarm == 2 && !_wasArmed));
  return reduce ? 1 : cfg.power;
}

int Vtx::setPower()
{
  const uint8_t power = desiredPower();
  if (!_serial || !power) return 0;
  const uint8_t code = power - 1;
  uint8_t vtxCommand[6] = {0xAA, 0x55, 0x05, 0x01, code};
  vtxCommand[5] = Utils::crc8_dvb_s2(0, vtxCommand, 5);
  _serial->write(dummyByte, 1);
  _serial->write(vtxCommand, 6);
  _serial->flush();

  return 1;
}

} // namespace Espfc::Connect
