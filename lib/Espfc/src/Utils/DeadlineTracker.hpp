#pragma once

#include <cstdint>

namespace Espfc::Utils {

// Result of observing one periodic release. One poll can authorize at most
// one execution: skipped release slots are counted, never replayed.
struct DeadlineRelease
{
  bool due = false;
  uint32_t scheduledAtUs = 0;
  uint32_t nextAtUs = 0;
  uint32_t skippedReleases = 0;
  uint32_t overdueUs = 0;       // From the first outstanding deadline.
  uint32_t phaseLatenessUs = 0; // From the most recent due release.
};

struct DeadlineStatistics
{
  uint32_t servicedReleases = 0;
  uint32_t missedReleases = 0;
  uint32_t rejectedObservations = 0;
  uint32_t worstOverdueUs = 0;
  uint32_t worstPhaseLatenessUs = 0;
  uint32_t worstMissedBurst = 0;
};

// Pure arithmetic; no clocks, interrupts, allocations or HAL dependencies.
//
// Contract:
//  - arm() schedules the FIRST release startUs + periodUs, not startUs.
//  - periodUs must be in [1, 0x7fffffff] microseconds.
//  - poll() timestamps must be monotonically increasing modulo 2^32, with
//    less than 2^31 us between observations; longer gaps are ambiguous and
//    require explicit re-arm. A rejected sample never moves the schedule.
//  - Deadlines remain phase-locked to the initial start time; late callbacks
//    do not re-anchor the schedule. No control/backlog replay is performed.
//  - Counters saturate instead of wrapping. Not thread-safe: callers must
//    provide single-owner access or external synchronization.
class DeadlineTracker
{
public:
  bool arm(uint32_t startUs, uint32_t periodUs)
  {
    clear();
    if (periodUs == 0 || periodUs >= HALF_RANGE_US) return false;
    _armed = true;
    _periodUs = periodUs;
    _lastObservedUs = startUs;
    _nextAtUs = startUs + periodUs;
    return true;
  }

  void clear()
  {
    _armed = false;
    _periodUs = 0;
    _lastObservedUs = 0;
    _nextAtUs = 0;
    _stats = DeadlineStatistics{};
  }

  DeadlineRelease poll(uint32_t nowUs)
  {
    DeadlineRelease result{};
    if (!_armed) return result;

    // The caller may re-supply the same timestamp, but must not supply
    // backwards or modulo-ambiguous timestamps.
    if (static_cast<uint32_t>(nowUs - _lastObservedUs) >= HALF_RANGE_US)
    {
      _stats.rejectedObservations =
          saturatedAdd(_stats.rejectedObservations, 1);
      return result;
    }
    _lastObservedUs = nowUs;

    const uint32_t sinceDeadline = nowUs - _nextAtUs;
    if (sinceDeadline >= HALF_RANGE_US) return result; // Not due.

    // Skip whole periods without executing stale controller iterations.
    const uint32_t skipped = sinceDeadline / _periodUs;
    const uint32_t scheduledAt = _nextAtUs + skipped * _periodUs;
    const uint32_t phaseLateness = nowUs - scheduledAt;

    _nextAtUs = scheduledAt + _periodUs;
    _stats.servicedReleases = saturatedAdd(_stats.servicedReleases, 1);
    _stats.missedReleases = saturatedAdd(_stats.missedReleases, skipped);
    if (sinceDeadline > _stats.worstOverdueUs) _stats.worstOverdueUs = sinceDeadline;
    if (phaseLateness > _stats.worstPhaseLatenessUs)
    {
      _stats.worstPhaseLatenessUs = phaseLateness;
    }
    if (skipped > _stats.worstMissedBurst) _stats.worstMissedBurst = skipped;

    result.due = true;
    result.scheduledAtUs = scheduledAt;
    result.nextAtUs = _nextAtUs;
    result.skippedReleases = skipped;
    result.overdueUs = sinceDeadline;
    result.phaseLatenessUs = phaseLateness;
    return result;
  }

  bool isArmed() const { return _armed; }
  uint32_t periodUs() const { return _periodUs; }
  uint32_t nextDeadlineUs() const { return _nextAtUs; }
  const DeadlineStatistics& statistics() const { return _stats; }

private:
  static constexpr uint32_t HALF_RANGE_US = 0x80000000u;

  static uint32_t saturatedAdd(uint32_t a, uint32_t b)
  {
    const uint32_t maximum = 0xffffffffu;
    return (b > maximum - a) ? maximum : a + b;
  }

  bool _armed = false;
  uint32_t _periodUs = 0;
  uint32_t _lastObservedUs = 0;
  uint32_t _nextAtUs = 0;
  DeadlineStatistics _stats{};
};

} // namespace Espfc::Utils
