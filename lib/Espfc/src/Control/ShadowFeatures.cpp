#include "Control/ShadowFeatures.h"

#include "Utils/Math.hpp"

#include <algorithm>
#include <cmath>

namespace Espfc::Control {

namespace {

constexpr float PI_F = 3.14159265358979323846f;
constexpr double PI_D = 3.14159265358979323846;
constexpr double DEG_TO_RAD_D = PI_D / 180.0;
constexpr double EARTH_RADIUS_M_D = 6371000.0;

float wrapPi(float angle)
{
  while (angle > PI_F) angle -= 2.0f * PI_F;
  while (angle < -PI_F) angle += 2.0f * PI_F;
  return angle;
}

float safeInput(const Model& model, size_t axis)
{
  if (axis >= AXIS_COUNT || !model.state.input.channelsValid) return 0.0f;
  const float value = model.state.input.ch[axis];
  return std::isfinite(value) ? std::clamp(value, -1.0f, 1.0f) : 0.0f;
}

} // namespace

int ShadowFeatures::begin()
{
  _model.state.shadow = {};
  _model.state.shadow.authorityBlocked = true;
  return 1;
}

float ShadowFeatures::horizonStrength(
    float rollDeg,
    float pitchDeg,
    float rollStick,
    float pitchStick)
{
  // Betaflight calcHorizonLevelStrength() attenuates leveling with both
  // inclination and stick deflection. Use the same two-factor structure while
  // keeping fixed shadow-only limits so no existing PID/Angle setting changes.
  constexpr float HORIZON_LIMIT_DEG = 60.0f;
  constexpr float HORIZON_STICK_LIMIT = 1.0f;

  const float inclination =
      std::max(std::fabs(rollDeg), std::fabs(pitchDeg));

  const float stick =
      std::max(std::fabs(rollStick), std::fabs(pitchStick));

  const float attitudeFactor =
      std::clamp(
          (HORIZON_LIMIT_DEG - inclination) /
              HORIZON_LIMIT_DEG,
          0.0f,
          1.0f);

  const float stickFactor =
      std::clamp(
          (HORIZON_STICK_LIMIT - stick) /
              HORIZON_STICK_LIMIT,
          0.0f,
          1.0f);

  return attitudeFactor * stickFactor;
}

float ShadowFeatures::acroTrainerSuggestion(
    float requestedRateDegS,
    float currentAngleDeg,
    float gyroRateDegS,
    float angleLimitDeg,
    float gain,
    float lookaheadSeconds)
{
  // Mirrors the structure of Betaflight applyAcroTrainer(): project angle
  // using a gyro-rate-scaled lookahead, then return a limiting rate only when
  // the projected/current angle exceeds the configured boundary in the same
  // direction as the requested rotation. This return value is diagnostic only.
  const float requestSign =
      requestedRateDegS > 0.0f
          ? 1.0f
          : requestedRateDegS < 0.0f
              ? -1.0f
              : 0.0f;

  const float angleSign =
      currentAngleDeg > 0.0f
          ? 1.0f
          : currentAngleDeg < 0.0f
              ? -1.0f
              : 0.0f;

  if (requestSign == 0.0f)
  {
    return requestedRateDegS;
  }

  if (std::fabs(currentAngleDeg) > angleLimitDeg &&
      angleSign == requestSign)
  {
    return std::clamp(
        (angleLimitDeg * angleSign - currentAngleDeg) * gain,
        -1000.0f,
        1000.0f);
  }

  const float scaledLookahead =
      std::clamp(
          std::fabs(gyroRateDegS) / 500.0f,
          0.0f,
          1.0f) *
      lookaheadSeconds;

  const float projected =
      currentAngleDeg +
      gyroRateDegS * scaledLookahead;

  const float projectedSign =
      projected > 0.0f
          ? 1.0f
          : projected < 0.0f
              ? -1.0f
              : 0.0f;

  if (std::fabs(projected) > angleLimitDeg &&
      projectedSign == requestSign)
  {
    return std::clamp(
        (angleLimitDeg * projectedSign - projected) * gain,
        -1000.0f,
        1000.0f);
  }

  return requestedRateDegS;
}

void ShadowFeatures::geoDeltaMeters(
    int32_t originLat,
    int32_t originLon,
    int32_t targetLat,
    int32_t targetLon,
    float& northM,
    float& eastM)
{
  const double lat0 =
      static_cast<double>(originLat) *
      1.0e-7 *
      DEG_TO_RAD_D;

  const double lat1 =
      static_cast<double>(targetLat) *
      1.0e-7 *
      DEG_TO_RAD_D;

  const double dLat =
      lat1 - lat0;

  // Work in 64-bit space before subtraction. Longitudes are stored as
  // degrees * 1e7, so crossing +180/-180 can otherwise overflow int32_t and
  // also select the long way around the globe.
  int64_t dLonE7 =
      static_cast<int64_t>(targetLon) -
      static_cast<int64_t>(originLon);

  constexpr int64_t HALF_TURN_E7 =
      1800000000LL;

  constexpr int64_t FULL_TURN_E7 =
      3600000000LL;

  if (dLonE7 >
      HALF_TURN_E7)
  {
    dLonE7 -=
        FULL_TURN_E7;
  }
  else if (dLonE7 <
           -HALF_TURN_E7)
  {
    dLonE7 +=
        FULL_TURN_E7;
  }

  const double dLon =
      static_cast<double>(dLonE7) *
      1.0e-7 *
      DEG_TO_RAD_D;

  const double meanLat =
      0.5 *
      (lat0 + lat1);

  northM =
      static_cast<float>(
          dLat *
          EARTH_RADIUS_M_D);

  eastM =
      static_cast<float>(
          dLon *
          std::cos(meanLat) *
          EARTH_RADIUS_M_D);
}

void ShadowFeatures::setWaypoint(
    int32_t lat,
    int32_t lon,
    uint8_t index)
{
  auto& shadow = _model.state.shadow;
  shadow.waypointLocation.lat = lat;
  shadow.waypointLocation.lon = lon;
  shadow.waypointLocation.height = 0;
  shadow.waypointLocationValid = true;
  shadow.waypointIndex = index;
}

void ShadowFeatures::clearWaypoint()
{
  auto& shadow = _model.state.shadow;
  shadow.waypointLocation = {};
  shadow.waypointLocationValid = false;
  shadow.waypointIndex = 0;
}

void ShadowFeatures::updateHorizon()
{
  auto& shadow = _model.state.shadow;
  shadow.horizonActive =
      _model.isModeActive(
          MODE_HORIZON_SHADOW);

  shadow.horizonValid =
      shadow.horizonActive &&
      _model.state.attitude.healthy;

  if (!shadow.horizonValid)
  {
    shadow.horizonStrength = 0.0f;
    shadow.horizonRateSuggestion[0] = 0.0f;
    shadow.horizonRateSuggestion[1] = 0.0f;
    return;
  }

  const float rollDeg =
      Utils::toDeg(
          _model.state.attitude.euler.x);

  const float pitchDeg =
      Utils::toDeg(
          _model.state.attitude.euler.y);

  shadow.horizonStrength =
      horizonStrength(
          rollDeg,
          pitchDeg,
          safeInput(_model, AXIS_ROLL),
          safeInput(_model, AXIS_PITCH));

  const float angleGain =
      static_cast<float>(
          _model.config.pid[
              FC_PID_LEVEL].P) *
      0.1f;

  const float currentAngles[AXIS_COUNT_RP] =
      {rollDeg, pitchDeg};

  for (size_t axis = 0;
       axis < AXIS_COUNT_RP;
       ++axis)
  {
    const float acroRate =
        Utils::toDeg(
            _model.state.setpoint.rate[
                axis]);

    const float levelRate =
        -currentAngles[axis] *
        angleGain;

    const float blended =
        acroRate *
            (1.0f -
             shadow.horizonStrength) +
        levelRate *
            shadow.horizonStrength;

    shadow.horizonRateSuggestion[axis] =
        Utils::toRad(
            blended);
  }

  _model.setDebug(
      DEBUG_ANGLE_MODE,
      6,
      static_cast<int16_t>(
          std::clamp<long>(
              lrintf(
                  shadow.horizonStrength *
                  1000.0f),
              -32768L,
              32767L)));
}

void ShadowFeatures::updateHeadfree()
{
  auto& shadow = _model.state.shadow;

  const bool active =
      _model.isModeActive(
          MODE_HEADFREE_SHADOW);

  if (active &&
      (!shadow.headfreeActive ||
       !shadow.headfreeReferenceValid) &&
      _model.state.attitude.healthy)
  {
    shadow.headfreeReferenceYaw =
        _model.state.attitude.euler.z;

    shadow.headfreeReferenceValid =
        true;
  }

  shadow.headfreeActive =
      active;

  if (!active ||
      !shadow.headfreeReferenceValid ||
      !_model.state.attitude.healthy)
  {
    shadow.headfreeInput[0] = 0.0f;
    shadow.headfreeInput[1] = 0.0f;
    shadow.headingError = 0.0f;

    if (!active)
    {
      shadow.headfreeReferenceValid =
          false;
    }

    return;
  }

  const float deltaYaw =
      wrapPi(
          _model.state.attitude.euler.z -
          shadow.headfreeReferenceYaw);

  const float c =
      std::cos(deltaYaw);

  const float s =
      std::sin(deltaYaw);

  const float roll =
      safeInput(
          _model,
          AXIS_ROLL);

  const float pitch =
      safeInput(
          _model,
          AXIS_PITCH);

  // Earth-reference command rotated into the current body frame, matching
  // Betaflight's headfree transformation concept without modifying RC input.
  shadow.headfreeInput[0] =
      c * roll +
      s * pitch;

  shadow.headfreeInput[1] =
      -s * roll +
      c * pitch;

  shadow.headingError =
      -deltaYaw;
}

void ShadowFeatures::updateAcroTrainer()
{
  auto& shadow = _model.state.shadow;

  shadow.acroTrainerActive =
      _model.isModeActive(
          MODE_ACRO_TRAINER_SHADOW);

  if (!shadow.acroTrainerActive ||
      !_model.state.attitude.healthy)
  {
    shadow.acroTrainerProjectedAngle[0] = 0.0f;
    shadow.acroTrainerProjectedAngle[1] = 0.0f;
    shadow.acroTrainerRateSuggestion[0] = 0.0f;
    shadow.acroTrainerRateSuggestion[1] = 0.0f;
    return;
  }

  const float angleDeg[AXIS_COUNT_RP] = {
      Utils::toDeg(
          _model.state.attitude.euler.x),
      Utils::toDeg(
          _model.state.attitude.euler.y)};

  for (size_t axis = 0;
       axis < AXIS_COUNT_RP;
       ++axis)
  {
    const float rateDegS =
        Utils::toDeg(
            _model.state.gyro.adc[
                axis]);

    const float requestedDegS =
        Utils::toDeg(
            _model.state.setpoint.rate[
                axis]);

    const float lookahead =
        std::clamp(
            std::fabs(rateDegS) /
                500.0f,
            0.0f,
            1.0f) *
        0.05f;

    shadow.acroTrainerProjectedAngle[
        axis] =
        angleDeg[axis] +
        rateDegS *
            lookahead;

    shadow.acroTrainerRateSuggestion[
        axis] =
        Utils::toRad(
            acroTrainerSuggestion(
                requestedDegS,
                angleDeg[axis],
                rateDegS,
                std::max(
                    1.0f,
                    static_cast<float>(
                        _model.config.compat.acroTrainerAngleLimit))));
  }
}

void ShadowFeatures::updateNavigation()
{
  auto& shadow =
      _model.state.shadow;

  shadow.gpsNavigationValid =
      _model.state.gps.isHomeValid();

  const bool rth =
      _model.isModeActive(
          MODE_GPS_RESCUE_SHADOW);

  const bool posHold =
      _model.isModeActive(
          MODE_POSHOLD_SHADOW);

  const bool waypoint =
      _model.isModeActive(
          MODE_WAYPOINT_SHADOW);

  if (posHold &&
      (!shadow.holdLocationValid ||
       _model.hasChanged(
           MODE_POSHOLD_SHADOW)) &&
      _model.state.gps.solutionFresh &&
      _model.state.gps.fix)
  {
    shadow.holdLocation =
        _model.state.gps.location.raw;

    shadow.holdLocationValid =
        true;
  }

  if (!posHold)
  {
    shadow.holdLocationValid =
        false;
  }

  GpsCoordinate<int32_t> target{};
  bool targetValid = false;

  if (rth)
  {
    shadow.navPhase =
        shadow.gpsNavigationValid
            ? SHADOW_NAV_RETURN_HOME
            : SHADOW_NAV_WAIT_HOME;

    if (shadow.gpsNavigationValid)
    {
      target =
          _model.state.gps.location.home;

      targetValid =
          true;
    }
  }
  else if (posHold)
  {
    shadow.navPhase =
        shadow.holdLocationValid
            ? SHADOW_NAV_POSITION_HOLD
            : SHADOW_NAV_WAIT_HOME;

    if (shadow.holdLocationValid)
    {
      target =
          shadow.holdLocation;

      targetValid =
          true;
    }
  }
  else if (waypoint)
  {
    shadow.navPhase =
        shadow.waypointLocationValid
            ? SHADOW_NAV_WAYPOINT
            : SHADOW_NAV_WAIT_HOME;

    if (shadow.waypointLocationValid)
    {
      target =
          shadow.waypointLocation;

      targetValid =
          true;
    }
  }
  else
  {
    shadow.navPhase =
        SHADOW_NAV_IDLE;
  }

  if (!targetValid ||
      !_model.state.gps.solutionFresh ||
      !_model.state.gps.fix)
  {
    shadow.targetNorthM = 0.0f;
    shadow.targetEastM = 0.0f;
    shadow.targetDistanceM = 0.0f;
    shadow.targetBearingRad = 0.0f;
    shadow.desiredNorthMs = 0.0f;
    shadow.desiredEastMs = 0.0f;
    return;
  }

  geoDeltaMeters(
      _model.state.gps.location.raw.lat,
      _model.state.gps.location.raw.lon,
      target.lat,
      target.lon,
      shadow.targetNorthM,
      shadow.targetEastM);

  shadow.targetDistanceM =
      std::hypot(
          shadow.targetNorthM,
          shadow.targetEastM);

  shadow.targetBearingRad =
      std::atan2(
          shadow.targetEastM,
          shadow.targetNorthM);

  if (shadow.targetDistanceM <=
      1.0f)
  {
    shadow.navPhase =
        SHADOW_NAV_ARRIVED;

    shadow.desiredNorthMs =
        0.0f;

    shadow.desiredEastMs =
        0.0f;
  }
  else
  {
    // Reference-style outer navigation demand only: proportional distance
    // error with a bounded horizontal speed. Never converted to angle,
    // attitude, PID or mixer commands.
    const float speed =
        std::clamp(
            shadow.targetDistanceM *
                0.5f,
            0.0f,
            5.0f);

    shadow.desiredNorthMs =
        std::cos(
            shadow.targetBearingRad) *
        speed;

    shadow.desiredEastMs =
        std::sin(
            shadow.targetBearingRad) *
        speed;
  }

  _model.setDebug(
      DEBUG_POSITION_NAV,
      0,
      static_cast<int16_t>(
          std::clamp<long>(
              lrintf(
                  shadow.targetNorthM *
                  10.0f),
              -32768L,
              32767L)));

  _model.setDebug(
      DEBUG_POSITION_NAV,
      1,
      static_cast<int16_t>(
          std::clamp<long>(
              lrintf(
                  shadow.targetEastM *
                  10.0f),
              -32768L,
              32767L)));

  _model.setDebug(
      DEBUG_POSITION_NAV,
      2,
      static_cast<int16_t>(
          std::clamp<long>(
              lrintf(
                  shadow.targetDistanceM *
                  10.0f),
              -32768L,
              32767L)));

  _model.setDebug(
      DEBUG_POSITION_NAV,
      3,
      static_cast<int16_t>(
          shadow.navPhase));
}

int ShadowFeatures::update()
{
  // Reassert the non-actuating invariant every cycle so diagnostics clearly
  // expose that these features have no output authority.
  _model.state.shadow.authorityBlocked =
      true;

  updateHorizon();
  updateHeadfree();
  updateAcroTrainer();
  updateNavigation();

  return 1;
}

} // namespace Espfc::Control
