#pragma once

#include "BaseSensor.hpp"
#include "Model.h"
#include "Utils/Filter.h"

namespace Espfc::Sensor {

class VoltageSensor : public BaseSensor
{
public:
  enum State
  {
    VBAT,
    IBAT
  };
  VoltageSensor(Model& model);
  int begin();
  int reload(ModelChangeEvent event);
  int update();
  int readVbat();
  int readIbat();

private:
  float escMeter(bool voltage) const;
  void integrateCurrent(float amps, uint32_t now);
  uint32_t _lastCurrentUs = 0;
  int _integrationSource = CURRENT_METER_NONE;
  Model& _model;
  Utils::Filter _vFilterFast;
  Utils::Filter _vFilter;
  Utils::Filter _iFilterFast;
  Utils::Filter _iFilter;
  State _state;
};

} // namespace Espfc::Sensor
