#include "Control/AdvancedModesBench.h"

#if defined(ESPFC_ADVANCED_MODES_ACTIVE_TEST)

#include "Control/Controller.h"
#include "Control/ShadowFeatures.h"
#include "Utils/Math.hpp"

#include <algorithm>
#include <cmath>

namespace Espfc::Control {

void applyAdvancedModesBench(Model& model, const Controller& controller)
{
  ShadowFeatures shadow(model);
  shadow.update();

  if (!model.state.input.channelsValid) return;

  if (model.isModeActive(MODE_HEADFREE_SHADOW) &&
      model.state.shadow.headfreeReferenceValid &&
      model.state.attitude.healthy)
  {
    model.state.setpoint.rate[AXIS_ROLL] =
        controller.calculateSetpointRate(
            AXIS_ROLL,
            std::clamp(model.state.shadow.headfreeInput[AXIS_ROLL], -1.0f, 1.0f));

    model.state.setpoint.rate[AXIS_PITCH] =
        controller.calculateSetpointRate(
            AXIS_PITCH,
            std::clamp(model.state.shadow.headfreeInput[AXIS_PITCH], -1.0f, 1.0f));
  }

  if (model.isModeActive(MODE_HORIZON_SHADOW) &&
      model.state.attitude.healthy)
  {
    const float rollDeg = Utils::toDeg(model.state.attitude.euler[AXIS_ROLL]);
    const float pitchDeg = Utils::toDeg(model.state.attitude.euler[AXIS_PITCH]);

    const float strength =
        ShadowFeatures::horizonStrength(
            rollDeg,
            pitchDeg,
            model.state.input.ch[AXIS_ROLL],
            model.state.input.ch[AXIS_PITCH]);

    const float angleGain =
        static_cast<float>(model.config.pid[FC_PID_LEVEL].P) * 0.1f;

    const float anglesDeg[AXIS_COUNT_RP] = {rollDeg, pitchDeg};

    for (size_t axis = 0; axis < AXIS_COUNT_RP; ++axis)
    {
      const float acroRateDegS =
          Utils::toDeg(model.state.setpoint.rate[axis]);

      const float levelRateDegS =
          -anglesDeg[axis] * angleGain;

      model.state.setpoint.rate[axis] =
          Utils::toRad(
              acroRateDegS * (1.0f - strength) +
              levelRateDegS * strength);
    }
  }

  if (model.isModeActive(MODE_ACRO_TRAINER_SHADOW) &&
      model.state.attitude.healthy)
  {
    const float angleLimit =
        std::max(
            1.0f,
            static_cast<float>(model.config.compat.acroTrainerAngleLimit));

    const float anglesDeg[AXIS_COUNT_RP] = {
        Utils::toDeg(model.state.attitude.euler[AXIS_ROLL]),
        Utils::toDeg(model.state.attitude.euler[AXIS_PITCH])};

    for (size_t axis = 0; axis < AXIS_COUNT_RP; ++axis)
    {
      const float requestedDegS =
          Utils::toDeg(model.state.setpoint.rate[axis]);

      const float gyroDegS =
          Utils::toDeg(model.state.gyro.adc[axis]);

      model.state.setpoint.rate[axis] =
          Utils::toRad(
              ShadowFeatures::acroTrainerSuggestion(
                  requestedDegS,
                  anglesDeg[axis],
                  gyroDegS,
                  angleLimit));
    }
  }
}

} // namespace Espfc::Control

#endif
