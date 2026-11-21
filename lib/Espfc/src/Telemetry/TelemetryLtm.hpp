#pragma once

#include "Model.h"

namespace Espfc::Telemetry {

class TelemetryLtm
{
public:
  void update(const Model& model, Stream::Writable& port);

private:
  uint32_t _lastMs = 0;
  uint8_t _slot = 0;
};

} // namespace Espfc::Telemetry
