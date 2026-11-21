#include "Pid.h"
#include "Utils/Math.hpp"
#include "Utils/MemoryHelper.h"
#include <algorithm>

namespace Espfc::Control {

Pid::Pid()
    : rate(1.0f), dt(1.0f), Kp(0.1), Ki(0.f), Kd(0.f), Kf(0.0f), iLimitLow(-0.3f), iLimitHigh(0.3f), iReset(0.0f),
      oLimitLow(-1.f), oLimitHigh(1.f), pScale(1.f), iScale(1.f), dScale(1.f), fScale(1.f), error(0.f), iTermError(0.f),
      pTerm(0.f), iTerm(0.f), dTerm(0.f), fTerm(0.f), prevMeasurement(0.f), prevError(0.f), prevSetpoint(0.f),
      ftermDerivative(true), itermRelax(ITERM_RELAX_OFF), itermRelaxFactor(1.0f),
      itermRelaxBase(0.f)
{
}

void Pid::begin()
{
  dt = 1.f / rate;
}

void Pid::configureDMax(uint8_t base, uint8_t ceiling, uint8_t gain, uint8_t advance)
{
  _dMaxRatio = base && ceiling > base ? static_cast<float>(ceiling) / base : 1.f;
  // D-Max gains operate on angular acceleration in degrees/s²; ESP-FC uses radians.
  constexpr float gainScale = Utils::toDeg(1.f) * 0.00008f / 35.f;
  _dMaxGyroGain = gain * gainScale;
  _dMaxSetpointGain = advance * gainScale;
  _dMaxRange.init(std::max(rate, 1.f), 85.f);
  _dMaxBoost.init(std::max(rate, 1.f), 35.f);
  _dMaxRange.reset();
  _dMaxBoost.reset();
  dMaxMultiplier = 1.f;
  dMaxGyroDemand = dMaxStickDemand = 0.f;
}

void Pid::resetIterm()
{
  iTerm = iReset;
}

float FAST_CODE_ATTR Pid::update(
    float setpoint,
    float measurement,
    float tpaFactor,
    bool tpaP,
    float pMultiplier,
    float iGainAdd)
{
  error = setpoint - measurement;
   const float tpa =
    std::clamp(tpaFactor, 0.0f, 1.0f);
    
  // P-term
  pTerm = Kp * error * pScale;
    
  pTerm = ptermFilter.update(pTerm);
    
  if (tpaP)
{
  pTerm *= tpa;
}

  // Optional P boost used by the guarded Anti-Gravity bench path.
  pTerm *= std::max(pMultiplier, 0.0f);

  // I-term
  iTermError = error;

  // Anti-Gravity uses an additive I coefficient, matching the
  // (Ki + itermAccelerator) structure used by Betaflight.
  const float effectiveIGain =
      Ki * iScale +
      std::max(iGainAdd, 0.0f);

  if (effectiveIGain > 0.f)
  {
    if (!(iTermError > 0.f && blockPositiveI) && !(iTermError < 0.f && blockNegativeI))
    {
      // I-term relax
      if (itermRelax)
      {
        const bool increasing = (iTerm > 0 && iTermError > 0) || (iTerm < 0 && iTermError < 0);
        const bool incrementOnly = itermRelax == ITERM_RELAX_RP_INC || itermRelax == ITERM_RELAX_RPY_INC;
        itermRelaxBase = setpoint - itermRelaxFilter.update(setpoint);
        itermRelaxFactor = std::max(0.0f, 1.0f - std::abs(Utils::toDeg(itermRelaxBase)) * 0.025f);
        if (!incrementOnly || increasing) iTermError *= itermRelaxFactor;
      }
      iTerm += effectiveIGain * iTermError * dt;
      iTerm = std::clamp(iTerm, iLimitLow, iLimitHigh);
    }
  }
  else
  {
    iTerm = 0; // zero integral
  }

  // D-term
  if (Kd > 0.f && dScale > 0.f)
  {
    // dTerm = (Kd * dScale * (((error - prevError) * dGamma) + (prevMeasurement - measure) * (1.f - dGamma)) / dt);
    dTerm = Kd * dScale * ((prevMeasurement - measurement) * rate);
    dTerm = dtermNotchFilter.update(dTerm);
    dTerm = dtermFilter.update(dTerm);
    dTerm = dtermFilter2.update(dTerm);
    if (_dMaxRatio > 1.f)
    {
      const float acceleration = dTerm / (Kd * dScale);
      dMaxGyroDemand = std::abs(_dMaxRange.update(acceleration)) * _dMaxGyroGain;
      dMaxStickDemand = std::abs((setpoint - prevSetpoint) * rate) * _dMaxSetpointGain;
      const float boost = _dMaxBoost.update(std::max(dMaxGyroDemand, dMaxStickDemand));
      dMaxMultiplier = 1.f + (_dMaxRatio - 1.f) * std::clamp(boost, 0.f, 1.f);
      dTerm *= dMaxMultiplier;
    }
    dTerm *= tpa;
  }
  else
  {
    dTerm = 0;
    dMaxMultiplier = 1.f;
    dMaxGyroDemand = dMaxStickDemand = 0.f;
  }

  // F-term
  if (Kf > 0.f && fScale > 0.f)
  {
    if (ftermDerivative)
    {
      fTerm = Kf * fScale * (setpoint - prevSetpoint) * rate;
    }
    else
    {
      fTerm = Kf * fScale * setpoint;
    }
    fTerm = ftermFilter.update(fTerm);
  }
  else
  {
    fTerm = 0;
  }

  prevMeasurement = measurement;
  prevError = error;
  prevSetpoint = setpoint;

  return std::clamp(pTerm + iTerm + dTerm + fTerm, oLimitLow, oLimitHigh);
}

} // namespace Espfc::Control
