/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _APPLE_AVE_USAGE_H_
#define _APPLE_AVE_USAGE_H_

#include <linux/ktime.h>
#include <linux/minmax.h>
#include <linux/spinlock.h>
#include <linux/types.h>

/**
 * struct ave_usage - per-device encoder usage counters
 * @lock: serializes counter updates and live busy_ns reads
 * @busy_ns: completed cumulative nanoseconds with >= 1 frame in flight
 * @busy_start_ns: ktime_get_ns() timestamp when inflight went 0 -> 1
 * @frames: cumulative completed frames answered by firmware
 * @inflight: number of frames currently in flight across all sessions
 * @active_sessions: number of active firmware sessions on this encoder
 */
struct ave_usage {
	spinlock_t lock;	/* protects counters and busy_start_ns */
	u64 busy_ns;
	u64 busy_start_ns;
	u64 frames;
	unsigned int inflight;
	unsigned int active_sessions;
};

static inline void ave_usage_init(struct ave_usage *u)
{
	spin_lock_init(&u->lock);
	u->busy_ns = 0;
	u->busy_start_ns = 0;
	u->frames = 0;
	u->inflight = 0;
	u->active_sessions = 0;
}

static inline void ave_usage_frame_begin_at(struct ave_usage *u, u64 now_ns)
{
	unsigned long flags;

	spin_lock_irqsave(&u->lock, flags);
	if (!u->inflight++)
		u->busy_start_ns = now_ns;
	spin_unlock_irqrestore(&u->lock, flags);
}

static inline void ave_usage_frame_begin(struct ave_usage *u)
{
	ave_usage_frame_begin_at(u, ktime_get_ns());
}

static inline void ave_usage_frame_done_at(struct ave_usage *u, u64 now_ns)
{
	unsigned long flags;

	spin_lock_irqsave(&u->lock, flags);
	u->frames++;
	if (u->inflight && !--u->inflight && now_ns > u->busy_start_ns)
		u->busy_ns += now_ns - u->busy_start_ns;
	spin_unlock_irqrestore(&u->lock, flags);
}

static inline void ave_usage_frame_done(struct ave_usage *u)
{
	ave_usage_frame_done_at(u, ktime_get_ns());
}

static inline void ave_usage_session_begin(struct ave_usage *u)
{
	unsigned long flags;

	spin_lock_irqsave(&u->lock, flags);
	u->active_sessions++;
	spin_unlock_irqrestore(&u->lock, flags);
}

static inline void ave_usage_session_end_at(struct ave_usage *u,
					    unsigned int abandoned, u64 now_ns)
{
	unsigned long flags;
	unsigned int n;

	spin_lock_irqsave(&u->lock, flags);
	if (u->active_sessions)
		u->active_sessions--;
	n = min(abandoned, u->inflight);
	if (n && u->inflight == n && now_ns > u->busy_start_ns)
		u->busy_ns += now_ns - u->busy_start_ns;
	u->inflight -= n;
	spin_unlock_irqrestore(&u->lock, flags);
}

static inline void ave_usage_session_end(struct ave_usage *u,
					 unsigned int abandoned)
{
	ave_usage_session_end_at(u, abandoned, ktime_get_ns());
}

static inline void ave_usage_cancel_all_at(struct ave_usage *u, u64 now_ns)
{
	unsigned long flags;

	spin_lock_irqsave(&u->lock, flags);
	if (u->inflight && now_ns > u->busy_start_ns)
		u->busy_ns += now_ns - u->busy_start_ns;
	u->inflight = 0;
	spin_unlock_irqrestore(&u->lock, flags);
}

static inline void ave_usage_cancel_all(struct ave_usage *u)
{
	ave_usage_cancel_all_at(u, ktime_get_ns());
}

static inline u64 ave_usage_busy_ns_at(struct ave_usage *u, u64 now_ns)
{
	unsigned long flags;
	u64 total;

	spin_lock_irqsave(&u->lock, flags);
	total = u->busy_ns;
	if (u->inflight && now_ns > u->busy_start_ns)
		total += now_ns - u->busy_start_ns;
	spin_unlock_irqrestore(&u->lock, flags);
	return total;
}

static inline u64 ave_usage_busy_ns(struct ave_usage *u)
{
	return ave_usage_busy_ns_at(u, ktime_get_ns());
}

static inline unsigned int ave_usage_active_sessions(struct ave_usage *u)
{
	unsigned long flags;
	unsigned int n;

	spin_lock_irqsave(&u->lock, flags);
	n = u->active_sessions;
	spin_unlock_irqrestore(&u->lock, flags);
	return n;
}

static inline u64 ave_usage_frames(struct ave_usage *u)
{
	unsigned long flags;
	u64 n;

	spin_lock_irqsave(&u->lock, flags);
	n = u->frames;
	spin_unlock_irqrestore(&u->lock, flags);
	return n;
}

#endif
