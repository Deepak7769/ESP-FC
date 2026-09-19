#pragma once

#include "Control/Rates.h"
#include "Model.h"

namespace Espfc::Control {

class Controller
{
public:
  Controller(Model& model);
  int begin();
  int reload(ModelChangeEvent event);
  int update();

  void outerLoopRobot();
  void innerLoopRobot();
  void outerLoop();
  void innerLoop();

  inline float getTpaFactor() const;
  inline void resetIterm();
  float calculateSetpointRate(int axis, float input) const;
  float calcualteAltHoldSetpoint() const;


private:
  void reloadFilter();
  void reloadPid();

  // V2 assisted-mode controller.
  // Initially runs in shadow mode for verification.
  void updateAssistedModesShadow();
  float calculatePilotClimbRateShadow() const;

  Model& _model;
  Rates _rates;
  Utils::Filter _speedFilter;

  bool _shadowAngleWasActive = false;
  bool _shadowAltWasActive = false;

  float _shadowAngleTarget[AXIS_COUNT_RP] =
      {0.0f, 0.0f};

  float _shadowAltitudeTarget = 0.0f;
  float _shadowVzTarget = 0.0f;
};

} // namespace Espfc::Control
