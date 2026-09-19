#pragma once

#include "Model.h"
#include "Utils/Filter.h"
#include "Hal/Time.hpp"

#include <Complementary.hpp>

#include <algorithm>
#include <cmath>

namespace Espfc::Control {

class Altitude
{
public:
Altitude(Model& model):
    _model(model),
    _heightInitialized(false),
    _filteredBaroValid(false),
    _estimatorTimeValid(false),
    _baroTimeValid(false),
    _acceptedBaroTimeValid(false),
    _lastEstimatorUpdateUs(0),
    _lastBaroUpdateUs(0),
    _lastAcceptedBaroUs(0),
    _filteredBaroAlt(0.0f),
    _filteredBaroVario(0.0f)
{
}
  int begin()
  {
    auto& altitude =
        _model.state.altitude;

    altitude.height =
        0.0f;

    altitude.vario =
        0.0f;

    altitude.baroInnovation =
        0.0f;

    altitude.healthy =
        false;

    altitude.baroAccepted =
        false;

    _heightInitialized =
        false;

    _filteredBaroValid =
        false;
    _estimatorTimeValid =
    false;

_baroTimeValid =
    false;

_acceptedBaroTimeValid =
    false;

_lastEstimatorUpdateUs =
    0;
    _lastBaroUpdateUs =
        0;

    _lastAcceptedBaroUs =
        0;

    _filteredBaroAlt =
        0.0f;

    _filteredBaroVario =
        0.0f;

    reload(
        MODEL_CHANGE_FILTER);

    return 1;
  }

  int reload(ModelChangeEvent event)
  {
    if (event != MODEL_CHANGE_FILTER)
    {
      return 1;
    }

    const int accelRate =
        std::max<int>(
            _model.state.accel.timer.rate,
            1);

    const int baroRate =
        std::max<int>(
            _model.state.baro.rate,
            1);

    // IMPORTANT:
    // These filters consume BAROMETER samples,
    // therefore their rate must be the barometer rate,
    // not the IMU rate.
    _altitudeFilter.begin(
        FilterConfig(
            FILTER_PT3,
            5),
        baroRate);

    _varioFilter.begin(
        FilterConfig(
            FILTER_PT3,
            5),
        baroRate);

    _varioFusion.begin(
        accelRate,
        _model.config.altHold.baroTau *
            0.1f);

    return 1;
  }

  int update()
  {
    Utils::Stats::Measure measure(
        _model.state.stats,
        COUNTER_IMU_FUSION2);

    auto& altitude =
        _model.state.altitude;

    const auto& baro =
        _model.state.baro;

    const int accelRate =
        std::max<int>(
            _model.state.accel.timer.rate,
            1);

const float nominalDt =
    1.0f /
    static_cast<float>(
        accelRate);

const uint32_t now =
    micros();

float dt =
    nominalDt;

if (_estimatorTimeValid)
{
  const uint32_t elapsedUs =
      static_cast<uint32_t>(
          now -
          _lastEstimatorUpdateUs);

  if (elapsedUs > 0)
  {
    const float measuredDt =
        static_cast<float>(
            elapsedUs) *
        0.000001f;

    dt =
        std::clamp(
            measuredDt,
            nominalDt * 0.25f,
            nominalDt * 4.0f);
  }
}

_lastEstimatorUpdateUs =
    now;

_estimatorTimeValid =
    true;

    // --------------------------------------------------
    // BAROMETER FRESHNESS
    // --------------------------------------------------

    constexpr uint32_t BARO_STALE_US =
        350000;

const bool baroFresh =
    baro.sampleValid &&
    static_cast<uint32_t>(
        now -
        baro.lastUpdateUs) <
        BARO_STALE_US;

    // Do not allow altitude hold during startup
    // pressure-zero calibration.
    const bool baroBiasReady =
        baro.altitudeBiasSamples < 0;

const bool newBaroSample =
    baro.sampleValid &&
    (!_baroTimeValid ||
     baro.lastUpdateUs !=
         _lastBaroUpdateUs);

    float baroDt =
        1.0f /
        static_cast<float>(
            std::max<int>(
                baro.rate,
                1));

    // --------------------------------------------------
    // PROCESS EACH BAROMETER SAMPLE EXACTLY ONCE
    // --------------------------------------------------

    if (newBaroSample)
    {
    if (_baroTimeValid)
      {
        baroDt =
            static_cast<float>(
                static_cast<uint32_t>(
                    baro.lastUpdateUs -
                    _lastBaroUpdateUs)) *
            0.000001f;

        baroDt =
            std::clamp(
                baroDt,
                0.001f,
                0.250f);
      }

      _lastBaroUpdateUs =
          baro.lastUpdateUs;
        _baroTimeValid =
    true;

      if (std::isfinite(
              baro.altitudeGround) &&
          std::isfinite(
              baro.vario))
      {
        _filteredBaroAlt =
            _altitudeFilter.update(
                baro.altitudeGround);

        _filteredBaroVario =
            _varioFilter.update(
                baro.vario);

        _filteredBaroValid =
            std::isfinite(
                _filteredBaroAlt) &&
            std::isfinite(
                _filteredBaroVario);
      }
    }
const auto& attitude =
    _model.state.attitude;
    // --------------------------------------------------
    // FAST VERTICAL VELOCITY ESTIMATION
    // --------------------------------------------------
const float accZ =
    _model.state.accel.world.z;

const bool accelFinite =
    std::isfinite(accZ);

// accel.world is only updated after a successful AHRS
// solution. Do not repeatedly integrate an old value.
const uint32_t nominalPeriodUs =
    static_cast<uint32_t>(
        nominalDt *
        1000000.0f);

const uint32_t projectionMaxAgeUs =
    std::max<uint32_t>(
        nominalPeriodUs * 3u,
        5000u);

const bool accelProjectionFresh =
    attitude.healthy &&
    static_cast<uint32_t>(
        now -
        attitude.lastUpdateUs) <=
        projectionMaxAgeUs;

const float safeAccZ =
    (accelFinite &&
     accelProjectionFresh)
        ? accZ
        : 0.0f;

// Previous externally visible state must match the
// complementary filter's previous state.
const float previousVario =
    std::isfinite(altitude.vario)
        ? altitude.vario
        : 0.0f;

// IMU-only prediction for this cycle.
const float predictedVario =
    previousVario +
    safeAccZ * dt;

// A stale barometer must NOT continuously pull Vz
// toward its last value.
const bool baroVarioUsable =
    baroFresh &&
    _filteredBaroValid;

const float varioMeasurement =
    baroVarioUsable
        ? _filteredBaroVario
        : predictedVario;

altitude.vario =
    _varioFusion.update(
        safeAccZ,
        varioMeasurement);

    // --------------------------------------------------
    // INITIALIZE ABSOLUTE HEIGHT
    // --------------------------------------------------

    if (!_heightInitialized &&
        newBaroSample &&
        baroFresh &&
        baroBiasReady &&
        _filteredBaroValid)
    {
altitude.height =
    _filteredBaroAlt;

// Do NOT set altitude.vario to zero here.
//
// _varioFusion has already been running during the
// barometer-bias phase. Resetting only altitude.vario
// would make the public estimator state disagree with
// the complementary filter's internal state.

if (!std::isfinite(altitude.vario))
{
  altitude.vario =
      0.0f;

  _varioFusion.begin(
      accelRate,
      _model.config.altHold.baroTau *
          0.1f,
      0.0f);
}

altitude.baroInnovation =
    0.0f;

      altitude.baroAccepted =
          true;

      _heightInitialized =
          true;

_lastAcceptedBaroUs =
    baro.lastUpdateUs;

_acceptedBaroTimeValid =
    true;
    }
    else
    {
      altitude.baroAccepted =
          false;
    }

    // --------------------------------------------------
    // HEIGHT PREDICTION + BAROMETER CORRECTION
    // --------------------------------------------------

    if (_heightInitialized)
    {
      const float safeVario =
          std::isfinite(
              altitude.vario)
              ? altitude.vario
              : 0.0f;

      const float predictedHeight =
          altitude.height +
          safeVario * dt;

      bool acceptedThisSample =
          false;

      if (newBaroSample &&
          baroFresh &&
          baroBiasReady &&
          _filteredBaroValid)
      {
        altitude.baroInnovation =
            _filteredBaroAlt -
            predictedHeight;

        constexpr float
            BARO_INNOVATION_GATE_M =
                1.5f;

        acceptedThisSample =
            std::isfinite(
                altitude.baroInnovation) &&
            std::fabs(
                altitude.baroInnovation) <
                BARO_INNOVATION_GATE_M;
      }

      if (acceptedThisSample)
      {
        constexpr float
            HEIGHT_CORRECTION_TAU_S =
                1.0f;

        const float alpha =
            std::clamp(
                baroDt /
                    (HEIGHT_CORRECTION_TAU_S +
                     baroDt),
                0.0f,
                1.0f);

        altitude.height =
            predictedHeight +
            alpha *
                altitude.baroInnovation;

altitude.baroAccepted =
    true;

_lastAcceptedBaroUs =
    baro.lastUpdateUs;

_acceptedBaroTimeValid =
    true;
      }
      else
      {
        altitude.height =
            predictedHeight;
      }
    }

    // --------------------------------------------------
    // ACCEPTED-BARO HEALTH
    // --------------------------------------------------

    constexpr uint32_t
        ACCEPTED_BARO_STALE_US =
            500000;

bool acceptedBaroFresh =
    _acceptedBaroTimeValid &&
    static_cast<uint32_t>(
        now -
        _lastAcceptedBaroUs) <
        ACCEPTED_BARO_STALE_US;

    // --------------------------------------------------
    // SAFE RE-ACQUISITION
    //
    // If the estimator has lost absolute reference because
    // many barometer samples were rejected, only rebase while
    // DISARMED. Never jump the altitude reference while armed.
    // --------------------------------------------------

    if (_heightInitialized &&
        !acceptedBaroFresh &&
        baroFresh &&
        baroBiasReady &&
        newBaroSample &&
        _filteredBaroValid &&
        !_model.isModeActive(
            MODE_ARMED))
    {
      altitude.height =
          _filteredBaroAlt;

      altitude.vario =
          0.0f;

      altitude.baroInnovation =
          0.0f;

      altitude.baroAccepted =
          true;

_lastAcceptedBaroUs =
    baro.lastUpdateUs;

_acceptedBaroTimeValid =
    true;

acceptedBaroFresh =
    true;

      _varioFusion.begin(
          accelRate,
          _model.config.altHold.baroTau *
              0.1f,
          0.0f);
    }
 
    // --------------------------------------------------
    // FINAL ESTIMATOR HEALTH
    // --------------------------------------------------
 constexpr uint32_t
    ATTITUDE_STALE_US =
        100000;



const bool attitudeFresh =
    attitude.healthy &&
    static_cast<uint32_t>(
        now -
        attitude.lastUpdateUs) <
        ATTITUDE_STALE_US;
    
altitude.healthy =
    _heightInitialized &&
    _filteredBaroValid &&
    baroBiasReady &&
    baroFresh &&
    acceptedBaroFresh &&
    attitudeFresh &&
    accelFinite &&
    std::isfinite(
        altitude.height) &&
    std::isfinite(
        altitude.vario);
    // --------------------------------------------------
    // DEBUG
    // --------------------------------------------------

    if (_model.config.debug.mode ==
        DEBUG_ALTITUDE)
    {
      _model.state.debug[0] =
          std::clamp(
              lrintf(
                  baro.altitudeGround *
                  100.0f),
              -32000l,
              32000l);

      _model.state.debug[1] =
          std::clamp(
              lrintf(
                  baro.vario *
                  100.0f),
              -32000l,
              32000l);

      _model.state.debug[2] =
          std::clamp(
              lrintf(
                  altitude.height *
                  100.0f),
              -32000l,
              32000l);

      _model.state.debug[3] =
          std::clamp(
              lrintf(
                  altitude.vario *
                  100.0f),
              -32000l,
              32000l);

      _model.state.debug[4] =
          std::clamp(
              lrintf(
                  altitude.baroInnovation *
                  100.0f),
              -32000l,
              32000l);

      _model.state.debug[5] =
          altitude.healthy
              ? 1
              : 0;

      _model.state.debug[6] =
          altitude.baroAccepted
              ? 1
              : 0;

const uint32_t baroAgeMs =
    baro.sampleValid
        ? static_cast<uint32_t>(
              now -
              baro.lastUpdateUs) /
              1000u
        : 32000u;

      _model.state.debug[7] =
          static_cast<int16_t>(
              std::min<uint32_t>(
                  baroAgeMs,
                  32000u));
    }

    return 1;
  }

private:
  Model& _model;

  Utils::Filter _altitudeFilter;
  Utils::Filter _varioFilter;

  Complementary _varioFusion;

  // Estimator state
  bool _heightInitialized;
  bool _filteredBaroValid;

  // Timestamp validity flags.
  // These avoid treating micros()==0 as "invalid"
  // after the 32-bit timer wraps.
  bool _estimatorTimeValid;
  bool _baroTimeValid;
  bool _acceptedBaroTimeValid;

  // Timing
  uint32_t _lastEstimatorUpdateUs;
  uint32_t _lastBaroUpdateUs;
  uint32_t _lastAcceptedBaroUs;

  // Filtered barometer state
  float _filteredBaroAlt;
  float _filteredBaroVario;
};

} // namespace Espfc::Control
