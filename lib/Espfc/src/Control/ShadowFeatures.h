#pragma once

#include "Model.h"

namespace Espfc::Control {

// Diagnostic implementation of Configurator-visible flight concepts that are
// not part of ESP-FC's actuator-authority path. Reference behavior is derived
// from Betaflight's Horizon/Acro-Trainer/Headfree math and INAV's separation
// of POSHOLD/RTH navigation state from attitude control.
//
// HARD CONTRACT:
//   * never writes ModelState::setpoint
//   * never writes innerPid / angleV2 / assistedMode
//   * never writes mixer or output state
//   * never arms/disarms the craft
class ShadowFeatures
{
public:
  explicit ShadowFeatures(Model& model): _model(model) {}

  int begin();
  int update();

  static float horizonStrength(float rollDeg, float pitchDeg, float rollStick, float pitchStick);
  static float acroTrainerSuggestion(
      float requestedRateDegS,
      float currentAngleDeg,
      float gyroRateDegS,
      float angleLimitDeg = 20.0f,
      float gain = 7.5f,
      float lookaheadSeconds = 0.05f);

  static void geoDeltaMeters(
      int32_t originLat,
      int32_t originLon,
      int32_t targetLat,
      int32_t targetLon,
      float& northM,
      float& eastM);

  void setWaypoint(int32_t lat, int32_t lon, uint8_t index = 0);
  void clearWaypoint();

private:
  void updateHorizon();
  void updateHeadfree();
  void updateAcroTrainer();
  void updateNavigation();

  Model& _model;
};

} // namespace Espfc::Control
