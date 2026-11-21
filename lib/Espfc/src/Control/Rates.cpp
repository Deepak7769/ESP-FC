#include "Rates.h"
#include "Utils/MemoryHelper.h"

namespace Espfc::Control {

constexpr float SETPOINT_RATE_LIMIT = 1998.0f;
constexpr float RC_RATE_INCREMENTAL = 14.54f;

float FAST_CODE_ATTR Rates::manualThrottle(const ThrottleCurveConfig& curve, float stick)
{
  stick = std::clamp(stick, -1.f, 1.f);
  if ((curve.mid == curve.hover && curve.expo == 0) || stick == -1.f || stick == 1.f)
    return stick;
  const float x = (stick + 1.f) * 0.5f;
  const float mid = std::min<uint8_t>(curve.mid, 100) * 0.01f;
  const float hover = std::min<uint8_t>(curve.hover, 100) * 0.01f;
  const float expo = std::min<uint8_t>(curve.expo, 100) * 0.01f;
  // Each of the configurator's two quadratic segments has its control point
  // halfway between its endpoints in X, so its Bezier parameter is linear in X.
  if (x <= mid)
  {
    const float t = x / mid;
    return 2.f * hover * t * (1.f + expo * (1.f - t)) - 1.f;
  }
  const float t = (x - mid) / (1.f - mid);
  const float y = hover + (1.f - hover) * t * (1.f - expo * (1.f - t));
  return 2.f * y - 1.f;
}

void Rates::begin(const InputConfig& config)
{
  rateType = (RateType)config.rateType;
  for (size_t i = 0; i < 3; i++)
  {
    rcExpo[i] = config.expo[i];
    rcRates[i] = config.rate[i];
    rates[i] = config.superRate[i];
    rateLimit[i] = config.rateLimit[i];
  }
}

float FAST_CODE_ATTR Rates::getSetpoint(const int axis, float input) const
{
  input = std::clamp(input, -0.995f, 0.995f); // limit input
  const float inputAbs = fabsf(input);
  float result = 0;
  switch (rateType)
  {
    case RATES_TYPE_BETAFLIGHT:
      result = betaflight(axis, input, inputAbs);
      break;
    case RATES_TYPE_RACEFLIGHT:
      result = raceflight(axis, input, inputAbs);
      break;
    case RATES_TYPE_KISS:
      result = kiss(axis, input, inputAbs);
      break;
    case RATES_TYPE_ACTUAL:
      result = actual(axis, input, inputAbs);
      break;
    case RATES_TYPE_QUICK:
      result = quick(axis, input, inputAbs);
      break;
  }
  return Utils::toRad(std::clamp<float>(result, -rateLimit[axis], rateLimit[axis]));
}

float FAST_CODE_ATTR Rates::betaflight(const int axis, float rcCommandf, const float rcCommandfAbs) const
{
  if (this->rcExpo[axis])
  {
    const float expof = this->rcExpo[axis] / 100.0f;
    rcCommandf = rcCommandf * power3(rcCommandfAbs) * expof + rcCommandf * (1 - expof);
  }

  float rcRate = this->rcRates[axis] / 100.0f;
  if (rcRate > 2.0f)
  {
    rcRate += RC_RATE_INCREMENTAL * (rcRate - 2.0f);
  }
  float angleRate = 200.0f * rcRate * rcCommandf;
  if (this->rates[axis])
  {
    const float rcSuperfactor =
        1.0f / (constrainf(1.0f - (rcCommandfAbs * (this->rates[axis] / 100.0f)), 0.01f, 1.00f));
    angleRate *= rcSuperfactor;
  }

  return angleRate;
}

float FAST_CODE_ATTR Rates::raceflight(const int axis, float rcCommandf, const float rcCommandfAbs) const
{
  // -1.0 to 1.0 ranged and curved
  rcCommandf = ((1.0f + 0.01f * this->rcExpo[axis] * (rcCommandf * rcCommandf - 1.0f)) * rcCommandf);
  // convert to -2000 to 2000 range using acro+ modifier
  float angleRate = 10.0f * this->rcRates[axis] * rcCommandf;
  angleRate = angleRate * (1 + rcCommandfAbs * (float)this->rates[axis] * 0.01f);

  return angleRate;
}

float FAST_CODE_ATTR Rates::kiss(const int axis, float rcCommandf, const float rcCommandfAbs) const
{
  const float rcCurvef = this->rcExpo[axis] / 100.0f;

  float kissRpyUseRates = 1.0f / (constrainf(1.0f - (rcCommandfAbs * (this->rates[axis] / 100.0f)), 0.01f, 1.00f));
  float kissRcCommandf =
      (power3(rcCommandf) * rcCurvef + rcCommandf * (1 - rcCurvef)) * (this->rcRates[axis] / 1000.0f);
  float kissAngle =
      constrainf(((2000.0f * kissRpyUseRates) * kissRcCommandf), -SETPOINT_RATE_LIMIT, SETPOINT_RATE_LIMIT);

  return kissAngle;
}

float FAST_CODE_ATTR Rates::actual(const int axis, float rcCommandf, const float rcCommandfAbs) const
{
  float expof = this->rcExpo[axis] / 100.0f;
  expof = rcCommandfAbs * (power5(rcCommandf) * expof + rcCommandf * (1 - expof));

  const float centerSensitivity = this->rcRates[axis] * 10.0f;
  const float stickMovement = std::max(0.f, this->rates[axis] * 10.0f - centerSensitivity);
  const float angleRate = rcCommandf * centerSensitivity + stickMovement * expof;

  return angleRate;
}

float FAST_CODE_ATTR Rates::quick(const int axis, float rcCommandf, const float rcCommandfAbs) const
{
  const float rcRate = this->rcRates[axis] * 2;
  if (rcRate <= 0.f)
{
  return 0.f;
}
  const float maxDPS = std::max(this->rates[axis] * 10.f, rcRate);
  const float linearity = this->rcExpo[axis] / 100.0f;
  const float superFactorConfig = (maxDPS / rcRate - 1) / (maxDPS / rcRate);

  float curve = power3(rcCommandfAbs) * linearity + rcCommandfAbs * (1 - linearity);
  float superfactor = 1.0f / (constrainf(1.0f - (curve * superFactorConfig), 0.01f, 1.00f));
  float angleRate = constrainf(rcCommandf * rcRate * superfactor, -SETPOINT_RATE_LIMIT, SETPOINT_RATE_LIMIT);

  return angleRate;
}

} // namespace Espfc::Control
