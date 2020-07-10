// Copyright 2020, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Shared frame timing code.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup aux_util
 */

#include "util/u_misc.h"
#include "util/u_logging.h"
#include "util/u_render_timing.h"

#include <stdio.h>
#include <assert.h>
#include <inttypes.h>


/*
 *
 * Helpers
 *
 */

#if 0
#define DEBUG_PRINT_FRAME_ID() U_LOG_RAW("%" PRIi64, frame_id)
#else
#define DEBUG_PRINT_FRAME_ID()                                                 \
	do {                                                                   \
	} while (false)
#endif

static uint64_t
get_last_input_plus_period_at_least_greater_then(struct u_rt_helper *urth,
                                                 uint64_t then_ns)
{
	uint64_t val = urth->last_input;

	while (val <= then_ns) {
		val += urth->period;
		assert(val != 0);
	}

	return val;
}


/*
 *
 * 'Exported' functions.
 *
 */

void
u_rt_helper_client_clear(struct u_rt_helper *urth)
{
	for (size_t i = 0; i < ARRAY_SIZE(urth->frames); i++) {
		urth->frames[i].state = U_RT_READY;
		urth->frames[i].frame_id = -1;
	}
}

void
u_rt_helper_init(struct u_rt_helper *urth)
{
	U_ZERO(urth);
	u_rt_helper_client_clear(urth);
}

void
u_rt_helper_predict(struct u_rt_helper *urth,
                    int64_t *out_frame_id,
                    uint64_t *predicted_display_time,
                    uint64_t *wake_up_time,
                    uint64_t *predicted_display_period,
                    uint64_t *min_display_period)
{
	int64_t frame_id = ++urth->frame_counter;
	*out_frame_id = frame_id;

	DEBUG_PRINT_FRAME_ID();

	uint64_t at_least_ns = os_monotonic_get_ns();

	// Don't return a time before the last returned type.
	if (at_least_ns < urth->last_returned) {
		at_least_ns = urth->last_returned;
	}

	uint64_t predict_ns =
	    get_last_input_plus_period_at_least_greater_then(urth, at_least_ns);

	urth->last_returned = predict_ns;

	*wake_up_time = predict_ns - urth->period;
	*predicted_display_time = predict_ns;
	*predicted_display_period = urth->period;
	*min_display_period = urth->period;

	size_t index = (uint64_t)frame_id % ARRAY_SIZE(urth->frames);
	assert(urth->frames[index].frame_id == -1);
	assert(urth->frames[index].state == U_RT_READY);

	urth->frames[index].predicted = os_monotonic_get_ns();
	urth->frames[index].state = U_RT_PREDICTED;
	urth->frames[index].frame_id = frame_id;
}

void
u_rt_helper_mark_wait_woke(struct u_rt_helper *urth, int64_t frame_id)
{
	size_t index = (uint64_t)frame_id % ARRAY_SIZE(urth->frames);
	assert(urth->frames[index].frame_id == frame_id);
	assert(urth->frames[index].state == U_RT_PREDICTED);

	urth->frames[index].wait_woke = os_monotonic_get_ns();
	urth->frames[index].state = U_RT_WAIT_LEFT;
}

void
u_rt_helper_mark_begin(struct u_rt_helper *urth, int64_t frame_id)
{
	DEBUG_PRINT_FRAME_ID();
	DEBUG_PRINT_FRAME_ID();

	size_t index = (uint64_t)frame_id % ARRAY_SIZE(urth->frames);
	assert(urth->frames[index].frame_id == frame_id);
	assert(urth->frames[index].state == U_RT_WAIT_LEFT);

	urth->frames[index].begin = os_monotonic_get_ns();
	urth->frames[index].state = U_RT_BEGUN;
}

void
u_rt_helper_mark_discarded(struct u_rt_helper *urth, int64_t frame_id)
{
	DEBUG_PRINT_FRAME_ID();

	size_t index = (uint64_t)frame_id % ARRAY_SIZE(urth->frames);
	assert(urth->frames[index].frame_id == frame_id);
	assert(urth->frames[index].state == U_RT_WAIT_LEFT ||
	       urth->frames[index].state == U_RT_BEGUN);

	urth->frames[index].end_frame = os_monotonic_get_ns();
	urth->frames[index].state = U_RT_READY;
	urth->frames[index].frame_id = -1;
}

void
u_rt_helper_mark_delivered(struct u_rt_helper *urth, int64_t frame_id)
{
	DEBUG_PRINT_FRAME_ID();

	size_t index = (uint64_t)frame_id % ARRAY_SIZE(urth->frames);
	assert(urth->frames[index].frame_id == frame_id);
	assert(urth->frames[index].state == U_RT_BEGUN);

	uint64_t now_ns = os_monotonic_get_ns();

	urth->frames[index].end_frame = now_ns;
	urth->frames[index].state = U_RT_READY;
	urth->frames[index].frame_id = -1;

#if 0
	uint64_t then_ns = urth->frames[index].wait_woke;
	uint64_t diff_ns = now_ns - then_ns;
	uint64_t ms100 = diff_ns / (1000 * 10);

	U_LOG_RAW("Diff %i.%02ims", (int)ms100 / 100, (int)ms100 % 100);
#endif
}

void
u_rt_helper_new_sample(struct u_rt_helper *urth,
                       uint64_t predict,
                       uint64_t extra,
                       uint64_t min_period)
{
	urth->last_input = predict;
	urth->extra = extra;
	urth->period = min_period;

	if (urth->last_returned == 0) {
		return;
	}
}
