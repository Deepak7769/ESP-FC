#include "Complementary.hpp"

#include <cmath>

void Complementary::begin(
    float sampleRate,
    float tau,
    float state)
{
  _dt =
      1.0f / sampleRate;

  _tau =
      tau;

  _alpha =
      _tau /
      (_tau + _dt);

  _state =
      state;
}

float Complementary::update(
    float rate,
    float position)
{
  return update(
      rate,
      position,
      _dt);
}

float Complementary::update(
    float rate,
    float position,
    float dt)
{
  if (!std::isfinite(dt) ||
      dt <= 0.0f)
  {
    dt =
        _dt;
  }

  const float alpha =
      _tau /
      (_tau + dt);

  _state =
      alpha *
          (_state + rate * dt) +
      (1.0f - alpha) *
          position;

  return _state;
}
