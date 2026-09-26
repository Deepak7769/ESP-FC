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

  // Tracks ownership of the thrust output so the V2
  // velocity controller can enter without a thrust step.
  bool _altHoldV2OutputWasActive = false;

  float _shadowAngleTarget[AXIS_COUNT_RP] =
      {0.0f, 0.0f};

  float _shadowAltitudeTarget = 0.0f;
  float _shadowVzTarget = 0.0f;

uint32_t _shadowLastUpdateUs = 0;
};

} // namespace Espfc::Control
