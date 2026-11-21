#pragma once

#include "Model.h"
#include "Utils/Timer.h"

namespace Espfc::Connect {

enum VtxDeviceType
{
  VTXDEV_UNSUPPORTED = 0, // reserved for MSP
  VTXDEV_RTC6705 = 1,
  // 2 reserved
  VTXDEV_SMARTAUDIO = 3,
  VTXDEV_TRAMP = 4,
  VTXDEV_MSP = 5,
  VTXDEV_UNKNOWN = 0xFF,
};

enum State
{
  INACTIVE,
  INIT,
  IDLE,
};

class Vtx
{
public:
  Vtx(Model& model): _serial(nullptr), _model(model) {}

  int begin(Stream::ReadWritable* serial);
  int update();
  int setChannel();
  int setPower();
  Connect::VtxDeviceType type;

private:
  Stream::ReadWritable* _serial;
  Model& _model;
  State _state = State::INACTIVE;
  uint8_t desiredPower() const;
  bool _wasArmed = false;
  uint8_t _lastBand = 0xff;
  uint8_t _lastChannel = 0xff;
  uint8_t _lastPower = 0xff;
  Utils::Timer _timer;
};

} // namespace Espfc::Connect
