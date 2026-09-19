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
      _lastBaroUpdateUs(0)
  {
  }

  int begin()
  {
    auto& altitude = _model.state.altitude;

    altitude.height = 0.0f;
    altitude.vario = 0.0f;
    altitude.baroInnovation = 0.0f;
    altitude.healthy = false;
    altitude.baroAccepted = false;

    _heightInitialized = false;
    _lastBaroUpdateUs = 0;

    reload(MODEL_CHANGE_FILTER);

    return 1;
  }

  int reload(ModelChangeEvent event)
  {
    switch (event)
    {
      case MODEL_CHANGE_FILTER:
      {
        const int rate =
            std::max<int>(_model.state.accel.timer.rate, 1);

        // These filters are deliberately not extremely slow.
        // The estimator itself performs the long-term correction.
        _altitudeFilter.begin(
            FilterConfig(FILTER_PT3, 5),
            rate);

        _varioFilter.begin(
            FilterConfig(FILTER_PT3, 5),
            rate);

        _varioFusion.begin(
            rate,
            _model.config.altHold.baroTau * 0.1f);

        break;
      }

      default:
        break;
    }

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

    const int rate =
        std::max<int>(
            _model.state.accel.timer.rate,
            1);

    const float dt =
        1.0f / static_cast<float>(rate);

    // Upsample filtered barometer information to the IMU update rate.
    const float baroAlt =
        _altitudeFilter.update(
            baro.altitudeGround);

    const float baroVario =
        _varioFilter.update(
            baro.vario);

    // Existing fast accel/baro vertical-speed fusion.
    const float accZ =
        _model.state.accel.world.z;

    altitude.vario =
        _varioFusion.update(
            accZ,
            baroVario);

    const uint32_t now =
        micros();

    // A barometer object existing is not enough.
    // Data must also be valid and recent.
    constexpr uint32_t BARO_STALE_US =
        350000;

    const bool baroFresh =
        baro.sampleValid &&
        baro.lastUpdateUs != 0 &&
        static_cast<uint32_t>(
            now - baro.lastUpdateUs) <
            BARO_STALE_US;

float baroDt =
    1.0f /
    static_cast<float>(
        std::max<int>(
            baro.rate,
            1));

const bool newBaroSample =
    baro.lastUpdateUs != 0 &&
    baro.lastUpdateUs !=
        _lastBaroUpdateUs;

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

    // Guard against corrupted or unexpectedly long timing.
    baroDt =
        std::clamp(
            baroDt,
            0.001f,
            0.250f);
  }

  _lastBaroUpdateUs =
      baro.lastUpdateUs;
}

    if (!_heightInitialized &&
        baroFresh &&
        std::isfinite(baroAlt))
    {
      altitude.height =
          baroAlt;

      altitude.vario =
          0.0f;

      altitude.baroInnovation =
          0.0f;

      _heightInitialized =
          true;
    }

    if (_heightInitialized)
    {
      // Fast prediction:
      // height(k+1) = height(k) + Vz * dt
      const float predictedHeight =
          altitude.height +
          altitude.vario * dt;

      altitude.baroInnovation =
          baroAlt -
          predictedHeight;

      // Reject physically implausible sudden barometer steps.
      // Diagnostic starting value; tune from logs, not by making
      // the barometer graph artificially flat.
      constexpr float BARO_INNOVATION_GATE_M =
          1.5f;

      altitude.baroAccepted =
          newBaroSample &&
          baroFresh &&
          std::isfinite(baroAlt) &&
          std::isfinite(altitude.vario) &&
          std::fabs(
              altitude.baroInnovation) <
              BARO_INNOVATION_GATE_M;

      if (altitude.baroAccepted)
      {
        // Slow absolute-height correction while the IMU/vario
        // path handles fast motion.
        constexpr float HEIGHT_CORRECTION_TAU_S =
            1.0f;

const float alpha =
    std::clamp(
        baroDt /
        (HEIGHT_CORRECTION_TAU_S + baroDt),
        0.0f,
        1.0f);

        altitude.height =
            predictedHeight +
            alpha *
            altitude.baroInnovation;
      }
      else
      {
        altitude.height =
            predictedHeight;
      }
    }

    altitude.healthy =
        _heightInitialized &&
        baroFresh &&
        std::isfinite(altitude.height) &&
        std::isfinite(altitude.vario);

    if (_model.config.debug.mode ==
        DEBUG_ALTITUDE)
    {
      _model.state.debug[0] =
          std::clamp(
              lrintf(baro.altitudeGround * 100.0f),
              -32000l,
              32000l);

      _model.state.debug[1] =
          std::clamp(
              lrintf(baro.vario * 100.0f),
              -32000l,
              32000l);

      _model.state.debug[2] =
          std::clamp(
              lrintf(altitude.height * 100.0f),
              -32000l,
              32000l);

      _model.state.debug[3] =
          std::clamp(
              lrintf(altitude.vario * 100.0f),
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
          altitude.healthy ? 1 : 0;

      _model.state.debug[6] =
          altitude.baroAccepted ? 1 : 0;

      _model.state.debug[7] =
          static_cast<int16_t>(
              std::min<uint32_t>(
                  (now - baro.lastUpdateUs) /
                      1000u,
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
  uint32_t _lastBaroUpdateUs;
};

} // namespace Espfc::Control
