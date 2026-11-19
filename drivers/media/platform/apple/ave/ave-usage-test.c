// SPDX-License-Identifier: GPL-2.0-only
#include <kunit/test.h>
#include <linux/module.h>

#include "ave-usage.h"

static void ave_usage_single_frame_test(struct kunit *test)
{
	struct ave_usage u;

	ave_usage_init(&u);
	KUNIT_EXPECT_EQ(test, ave_usage_busy_ns_at(&u, 1000), 0ULL);
	KUNIT_EXPECT_EQ(test, ave_usage_active_sessions(&u), 0U);
	KUNIT_EXPECT_EQ(test, ave_usage_frames(&u), 0ULL);

	ave_usage_session_begin(&u);
	KUNIT_EXPECT_EQ(test, ave_usage_active_sessions(&u), 1U);

	ave_usage_frame_begin_at(&u, 1000);
	/* Live read while frame is in flight includes elapsed time so far. */
	KUNIT_EXPECT_EQ(test, ave_usage_busy_ns_at(&u, 1500), 500ULL);
	ave_usage_frame_done_at(&u, 2200);
	KUNIT_EXPECT_EQ(test, ave_usage_busy_ns_at(&u, 3000), 1200ULL);
	KUNIT_EXPECT_EQ(test, ave_usage_frames(&u), 1ULL);

	ave_usage_session_end_at(&u, 0, 3100);
	KUNIT_EXPECT_EQ(test, ave_usage_active_sessions(&u), 0U);
	KUNIT_EXPECT_EQ(test, ave_usage_busy_ns_at(&u, 5000), 1200ULL);
}

/* Concurrent frames or sessions count union wall time, never double-count. */
static void ave_usage_overlap_union_test(struct kunit *test)
{
	struct ave_usage u;

	ave_usage_init(&u);
	ave_usage_session_begin(&u);
	ave_usage_session_begin(&u);
	KUNIT_EXPECT_EQ(test, ave_usage_active_sessions(&u), 2U);

	ave_usage_frame_begin_at(&u, 1000);
	ave_usage_frame_begin_at(&u, 1200);
	KUNIT_EXPECT_EQ(test, ave_usage_busy_ns_at(&u, 1500), 500ULL);

	ave_usage_frame_done_at(&u, 1800);
	KUNIT_EXPECT_EQ(test, ave_usage_busy_ns_at(&u, 2000), 1000ULL);
	KUNIT_EXPECT_EQ(test, ave_usage_frames(&u), 1ULL);

	ave_usage_frame_done_at(&u, 2500);
	KUNIT_EXPECT_EQ(test, ave_usage_busy_ns_at(&u, 4000), 1500ULL);
	KUNIT_EXPECT_EQ(test, ave_usage_frames(&u), 2ULL);
}

/* Aborting or closing a session with frames still out closes the busy span. */
static void ave_usage_abandon_and_reset_test(struct kunit *test)
{
	struct ave_usage u;

	ave_usage_init(&u);
	ave_usage_session_begin(&u);
	ave_usage_frame_begin_at(&u, 1000);
	ave_usage_frame_begin_at(&u, 1100);
	/* Session ends with 2 abandoned frames, even clamped safely above inflight. */
	ave_usage_session_end_at(&u, 5, 1900);
	KUNIT_EXPECT_EQ(test, ave_usage_active_sessions(&u), 0U);
	KUNIT_EXPECT_EQ(test, ave_usage_busy_ns_at(&u, 5000), 900ULL);

	/* Monotonicity when clock does not advance and cancel_all on power loss. */
	ave_usage_frame_begin_at(&u, 6000);
	KUNIT_EXPECT_EQ(test, ave_usage_busy_ns_at(&u, 5500), 900ULL);
	ave_usage_cancel_all_at(&u, 7000);
	KUNIT_EXPECT_EQ(test, ave_usage_busy_ns_at(&u, 9000), 1900ULL);
}

static struct kunit_case ave_usage_cases[] = {
	KUNIT_CASE(ave_usage_single_frame_test),
	KUNIT_CASE(ave_usage_overlap_union_test),
	KUNIT_CASE(ave_usage_abandon_and_reset_test),
	{}
};

static struct kunit_suite ave_usage_suite = {
	.name = "apple-ave-usage",
	.test_cases = ave_usage_cases,
};

kunit_test_suite(ave_usage_suite);

MODULE_DESCRIPTION("Apple AVE usage accounting tests");
MODULE_LICENSE("GPL");
