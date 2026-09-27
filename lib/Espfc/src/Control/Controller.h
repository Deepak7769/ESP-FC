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
void updateAssistedModes();

float calculatePilotClimbRateShadow() const;

bool _angleV2WasActive =
    false;

bool _shadowAltWasActive =
    false;

// Tracks ownership of the thrust output so AltHold V2
// can enter without a thrust discontinuity.
bool _altHoldV2OutputWasActive =
    false;

float _shadowAltitudeTarget =
    0.0f;

float _shadowVzTarget =
    0.0f;

uint32_t _assistedLastUpdateUs =
    0;
};

} // namespace Espfc::Control
