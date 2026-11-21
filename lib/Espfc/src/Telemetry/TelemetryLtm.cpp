#include "Telemetry/TelemetryLtm.hpp"
#include "Hal/Time.hpp"
#include <algorithm>
#include <cmath>

namespace Espfc::Telemetry {
namespace {

class LtmFrame
{
public:
  explicit LtmFrame(char type) : _data{'$', 'T', static_cast<uint8_t>(type)} {}
  void byte(uint8_t value) { _data[_length++] = value; _checksum ^= value; }
  void word(uint16_t value) { byte(value); byte(value >> 8); }
  void dword(uint32_t value) { word(value); word(value >> 16); }
  void send(Stream::Writable& port)
  {
    _data[_length++] = _checksum;
    if (port.availableForWrite() >= static_cast<int>(_length)) port.write(_data, _length);
  }
private:
  uint8_t _data[18]; // G and O frames are the largest (14 payload bytes)
  size_t _length = 3;
  uint8_t _checksum = 0;
};

} // namespace

void TelemetryLtm::update(const Model& model, Stream::Writable& port)
{
  const uint32_t now = millis();
  if (now - _lastMs < 100) return;
  _lastMs = now;
  const auto& attitude = model.state.attitude.euler;
  LtmFrame angles('A');
  angles.word(lrintf(Utils::toDeg(attitude.y)));
  angles.word(lrintf(Utils::toDeg(attitude.x)));
  angles.word((lrintf(Utils::toDeg(-attitude.z)) + 360) % 360);
  angles.send(port);

  const auto& gps = model.state.gps;
  if (_slot % 2 == 0)
  {
    const auto& battery = model.state.battery;
    unsigned mode = 1; // rate
    if (model.state.failsafe.landingActive) mode = 15;
    else if (model.isModeActive(MODE_ALTHOLD)) mode = 8;
    else if (model.isModeActive(MODE_ANGLE)) mode = 2;
    LtmFrame status('S');
    status.word(std::clamp<long>(lrintf(battery.voltage * 1000.f), 0, 65535));
    status.word(battery.mahDrawn(model.config.ibat.source));
    status.byte(model.getRssi() * 255u / 1023);
    status.byte(0); // no airspeed sensor
    status.byte(mode << 2 | (model.isModeActive(MODE_FAILSAFE) << 1) | model.isModeActive(MODE_ARMED));
    status.send(port);
  }
  else if (gps.present)
  {
    LtmFrame position('G');
    position.dword(gps.location.raw.lat);
    position.dword(gps.location.raw.lon);
    position.byte(std::clamp<int32_t>(gps.velocity.raw.groundSpeed / 1000, 0, 255));
    position.dword(lrintf(model.state.altitude.height * 100.f));
    const unsigned fix = !gps.fix ? 1 : (gps.fixType >= 3 ? 3 : 2);
    position.byte(std::min<unsigned>(gps.numSats, 63) << 2 | fix);
    position.send(port);
  }
  if (_slot == 0)
  {
    LtmFrame origin('O');
    origin.dword(gps.location.home.lat);
    origin.dword(gps.location.home.lon);
    origin.dword(gps.location.home.height / 10);
    origin.byte(1);
    origin.byte(gps.isHomeValid());
    origin.send(port);
  }
  _slot = (_slot + 1) % 10;
}

} // namespace Espfc::Telemetry
