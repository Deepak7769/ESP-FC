#include "Utils/DeadlineTracker.hpp"
#include <unity.h>

using Espfc::Utils::DeadlineRelease;
using Espfc::Utils::DeadlineTracker;

void test_deadline_first_release_occurs_after_period()
{
  DeadlineTracker tracker;
  TEST_ASSERT_TRUE(tracker.arm(100, 250));
  TEST_ASSERT_TRUE(tracker.isArmed());
  TEST_ASSERT_EQUAL_UINT32(250, tracker.periodUs());
  TEST_ASSERT_EQUAL_UINT32(350, tracker.nextDeadlineUs());
  TEST_ASSERT_FALSE(tracker.poll(100).due);
  TEST_ASSERT_FALSE(tracker.poll(349).due);

  const DeadlineRelease onTime = tracker.poll(350);
  TEST_ASSERT_TRUE(onTime.due);
  TEST_ASSERT_EQUAL_UINT32(350, onTime.scheduledAtUs);
  TEST_ASSERT_EQUAL_UINT32(600, onTime.nextAtUs);
  TEST_ASSERT_EQUAL_UINT32(0, onTime.skippedReleases);
  TEST_ASSERT_EQUAL_UINT32(0, onTime.phaseLatenessUs);
  TEST_ASSERT_FALSE(tracker.poll(350).due);
  TEST_ASSERT_EQUAL_UINT32(1, tracker.statistics().servicedReleases);
  TEST_ASSERT_EQUAL_UINT32(0, tracker.statistics().missedReleases);
}

void test_deadline_late_callback_keeps_original_phase()
{
  DeadlineTracker tracker;
  TEST_ASSERT_TRUE(tracker.arm(100, 250));
  const DeadlineRelease late = tracker.poll(375);
  TEST_ASSERT_TRUE(late.due);
  TEST_ASSERT_EQUAL_UINT32(350, late.scheduledAtUs);
  TEST_ASSERT_EQUAL_UINT32(600, late.nextAtUs);
  TEST_ASSERT_EQUAL_UINT32(25, late.overdueUs);
  TEST_ASSERT_EQUAL_UINT32(25, late.phaseLatenessUs);
  TEST_ASSERT_EQUAL_UINT32(25, tracker.statistics().worstOverdueUs);
  TEST_ASSERT_FALSE(tracker.poll(599).due);
  TEST_ASSERT_TRUE(tracker.poll(600).due);
  TEST_ASSERT_EQUAL_UINT32(850, tracker.nextDeadlineUs());
}

void test_deadline_skips_missed_slots_without_replaying_backlog()
{
  DeadlineTracker tracker;
  TEST_ASSERT_TRUE(tracker.arm(100, 250));

  const DeadlineRelease late = tracker.poll(1105);
  TEST_ASSERT_TRUE(late.due);
  TEST_ASSERT_EQUAL_UINT32(3, late.skippedReleases); // 350, 600, 850 skipped.
  TEST_ASSERT_EQUAL_UINT32(1100, late.scheduledAtUs);
  TEST_ASSERT_EQUAL_UINT32(1350, late.nextAtUs);
  TEST_ASSERT_EQUAL_UINT32(755, late.overdueUs);
  TEST_ASSERT_EQUAL_UINT32(5, late.phaseLatenessUs);
  TEST_ASSERT_EQUAL_UINT32(1, tracker.statistics().servicedReleases);
  TEST_ASSERT_EQUAL_UINT32(3, tracker.statistics().missedReleases);
  TEST_ASSERT_EQUAL_UINT32(3, tracker.statistics().worstMissedBurst);
  TEST_ASSERT_FALSE(tracker.poll(1105).due);

  const DeadlineRelease second = tracker.poll(1600);
  TEST_ASSERT_TRUE(second.due);
  TEST_ASSERT_EQUAL_UINT32(1, second.skippedReleases);
  TEST_ASSERT_EQUAL_UINT32(1600, second.scheduledAtUs);
  TEST_ASSERT_EQUAL_UINT32(1850, second.nextAtUs);
  TEST_ASSERT_EQUAL_UINT32(2, tracker.statistics().servicedReleases);
  TEST_ASSERT_EQUAL_UINT32(4, tracker.statistics().missedReleases);
  TEST_ASSERT_EQUAL_UINT32(755, tracker.statistics().worstOverdueUs);
}

void test_deadline_monotonic_32bit_wraparound()
{
  DeadlineTracker tracker;
  TEST_ASSERT_TRUE(tracker.arm(0xfffffff0u, 32));
  TEST_ASSERT_EQUAL_UINT32(0x00000010u, tracker.nextDeadlineUs());
  TEST_ASSERT_FALSE(tracker.poll(0x00000005u).due);

  const DeadlineRelease exact = tracker.poll(0x00000010u);
  TEST_ASSERT_TRUE(exact.due);
  TEST_ASSERT_EQUAL_UINT32(0x00000010u, exact.scheduledAtUs);
  TEST_ASSERT_EQUAL_UINT32(0x00000030u, tracker.nextDeadlineUs());

  const DeadlineRelease delayed = tracker.poll(0x00000051u);
  TEST_ASSERT_TRUE(delayed.due);
  TEST_ASSERT_EQUAL_UINT32(1, delayed.skippedReleases);
  TEST_ASSERT_EQUAL_UINT32(0x00000050u, delayed.scheduledAtUs);
  TEST_ASSERT_EQUAL_UINT32(0x00000070u, delayed.nextAtUs);
  TEST_ASSERT_EQUAL_UINT32(33, delayed.overdueUs);
  TEST_ASSERT_EQUAL_UINT32(1, delayed.phaseLatenessUs);
}

void test_deadline_rejects_backward_observations()
{
  DeadlineTracker tracker;
  TEST_ASSERT_TRUE(tracker.arm(1000, 250));
  TEST_ASSERT_FALSE(tracker.poll(1100).due);
  TEST_ASSERT_FALSE(tracker.poll(1050).due);
  TEST_ASSERT_EQUAL_UINT32(1, tracker.statistics().rejectedObservations);
  TEST_ASSERT_EQUAL_UINT32(1250, tracker.nextDeadlineUs());
  TEST_ASSERT_TRUE(tracker.poll(1250).due);
  TEST_ASSERT_EQUAL_UINT32(1500, tracker.nextDeadlineUs());
  TEST_ASSERT_EQUAL_UINT32(0, tracker.statistics().missedReleases);
}

void test_deadline_rejects_ambiguous_gaps_and_bad_periods()
{
  DeadlineTracker tracker;
  TEST_ASSERT_FALSE(tracker.arm(100, 0));
  TEST_ASSERT_FALSE(tracker.isArmed());
  TEST_ASSERT_FALSE(tracker.poll(350).due);
  TEST_ASSERT_FALSE(tracker.arm(100, 0x80000000u));
  TEST_ASSERT_FALSE(tracker.isArmed());

  TEST_ASSERT_TRUE(tracker.arm(0, 0x7fffffffu));
  TEST_ASSERT_FALSE(tracker.poll(0x80000000u).due); // Ambiguous > half-range gap.
  TEST_ASSERT_EQUAL_UINT32(1, tracker.statistics().rejectedObservations);
  TEST_ASSERT_EQUAL_UINT32(0x7fffffffu, tracker.nextDeadlineUs());
  TEST_ASSERT_TRUE(tracker.poll(0x7fffffffu).due);
}

void test_deadline_missed_counter_saturates()
{
  DeadlineTracker tracker;
  TEST_ASSERT_TRUE(tracker.arm(0, 1));

  TEST_ASSERT_TRUE(tracker.poll(0x7fffffffu).due);
  TEST_ASSERT_TRUE(tracker.poll(0xfffffffeu).due);
  TEST_ASSERT_TRUE(tracker.poll(0x7ffffffdu).due);

  TEST_ASSERT_EQUAL_UINT32(3, tracker.statistics().servicedReleases);
  TEST_ASSERT_EQUAL_UINT32(0xffffffffu, tracker.statistics().missedReleases);
  TEST_ASSERT_EQUAL_UINT32(0x7ffffffeu, tracker.statistics().worstMissedBurst);
}

void test_deadline_reset_clears_history()
{
  DeadlineTracker tracker;
  TEST_ASSERT_TRUE(tracker.arm(0, 250));
  TEST_ASSERT_TRUE(tracker.poll(1000).due);
  TEST_ASSERT_EQUAL_UINT32(3, tracker.statistics().missedReleases);

  tracker.clear();
  TEST_ASSERT_FALSE(tracker.isArmed());
  TEST_ASSERT_EQUAL_UINT32(0, tracker.statistics().servicedReleases);
  TEST_ASSERT_EQUAL_UINT32(0, tracker.statistics().missedReleases);
  TEST_ASSERT_FALSE(tracker.poll(1500).due);

  TEST_ASSERT_TRUE(tracker.arm(5000, 100));
  TEST_ASSERT_EQUAL_UINT32(5100, tracker.nextDeadlineUs());
  TEST_ASSERT_EQUAL_UINT32(0, tracker.statistics().worstOverdueUs);
}

int main(int argc, char** argv)
{
  (void)argc;
  (void)argv;
  UNITY_BEGIN();
  RUN_TEST(test_deadline_first_release_occurs_after_period);
  RUN_TEST(test_deadline_late_callback_keeps_original_phase);
  RUN_TEST(test_deadline_skips_missed_slots_without_replaying_backlog);
  RUN_TEST(test_deadline_monotonic_32bit_wraparound);
  RUN_TEST(test_deadline_rejects_backward_observations);
  RUN_TEST(test_deadline_rejects_ambiguous_gaps_and_bad_periods);
  RUN_TEST(test_deadline_missed_counter_saturates);
  RUN_TEST(test_deadline_reset_clears_history);
  return UNITY_END();
}
