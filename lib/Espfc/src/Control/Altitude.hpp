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

    const float dt =
        1.0f /
        static_cast<float>(
            accelRate);

    const uint32_t now =
        micros();

    // --------------------------------------------------
    // BAROMETER FRESHNESS
    // --------------------------------------------------

    constexpr uint32_t BARO_STALE_US =
        350000;

    const bool baroFresh =
        baro.sampleValid &&
        baro.lastUpdateUs != 0 &&
        static_cast<uint32_t>(
            now -
            baro.lastUpdateUs) <
            BARO_STALE_US;

    // Do not allow altitude hold during startup
    // pressure-zero calibration.
    const bool baroBiasReady =
        baro.altitudeBiasSamples < 0;

    const bool newBaroSample =
        baro.lastUpdateUs != 0 &&
        baro.lastUpdateUs !=
            _lastBaroUpdateUs;

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
      if (_lastBaroUpdateUs != 0)
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

    // --------------------------------------------------
    // FAST VERTICAL VELOCITY ESTIMATION
    // --------------------------------------------------

    const float accZ =
        _model.state.accel.world.z;

    const bool accelFinite =
        std::isfinite(accZ);

    const float safeAccZ =
        accelFinite
            ? accZ
            : 0.0f;

    const float baroVarioMeasurement =
        _filteredBaroValid
            ? _filteredBaroVario
            : 0.0f;

    altitude.vario =
        _varioFusion.update(
            safeAccZ,
            baroVarioMeasurement);

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

      altitude.vario =
          0.0f;

      altitude.baroInnovation =
          0.0f;

      altitude.baroAccepted =
          true;

      _heightInitialized =
          true;

      _lastAcceptedBaroUs =
          baro.lastUpdateUs;
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
        _lastAcceptedBaroUs != 0 &&
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

    altitude.healthy =
        _heightInitialized &&
        _filteredBaroValid &&
        baroBiasReady &&
        baroFresh &&
        acceptedBaroFresh &&
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
          baro.lastUpdateUs != 0
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

  bool _heightInitialized;
  bool _filteredBaroValid;

  uint32_t _lastBaroUpdateUs;
  uint32_t _lastAcceptedBaroUs;

  float _filteredBaroAlt;
  float _filteredBaroVario;
};

} // namespace Espfc::Control
