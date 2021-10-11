// Copyright 2020-2021, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  For generating a fake timing.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup aux_util
 */

#include "os/os_time.h"

#include "util/u_time.h"
#include "util/u_misc.h"
#include "util/u_debug.h"
#include "util/u_timing.h"
#include "util/u_logging.h"
#include "util/u_trace_marker.h"

#include <stdio.h>
#include <assert.h>
#include <inttypes.h>


/*
 *
 * Structs and defines.
 *
 */


struct fake_timing
{
	struct u_frame_timing base;

	/*!
	 * The periodicity of the display.
	 */
	uint64_t frame_period_ns;

	/*!
	 * When the last frame was made.
	 */
	uint64_t last_display_time_ns;

	/*!
	 * Very often the present time that we get from the system is only when
	 * the display engine starts scanning out from the buffers we provided,
	 * and not when the pixels turned into photons that the user sees.
	 */
	uint64_t present_offset_ns;

	// The amount of time that the application needs to render frame.
	uint64_t app_time_ns;

	int64_t frame_id_generator;
};


/*
 *
 * Helper functions.
 *
 */

static inline struct fake_timing *
fake_timing(struct u_frame_timing *uft)
{
	return (struct fake_timing *)uft;
}

static uint64_t
predict_next_frame(struct fake_timing *ft)
{
	uint64_t time_needed_ns = ft->present_offset_ns + ft->app_time_ns;
	uint64_t now_ns = os_monotonic_get_ns();
	uint64_t predicted_display_time_ns = ft->last_display_time_ns + ft->frame_period_ns;

	while (now_ns + time_needed_ns > predicted_display_time_ns) {
		predicted_display_time_ns += ft->frame_period_ns;
	}

	return predicted_display_time_ns;
}

static uint64_t
get_percent_of_time(uint64_t time_ns, uint32_t fraction_percent)
{
	double fraction = (double)fraction_percent / 100.0;
	return time_s_to_ns(time_ns_to_s(time_ns) * fraction);
}


/*
 *
 * Member functions.
 *
 */

static void
ft_predict(struct u_frame_timing *uft,
           int64_t *out_frame_id,
           uint64_t *out_wake_up_time_ns,
           uint64_t *out_desired_present_time_ns,
           uint64_t *out_present_slop_ns,
           uint64_t *out_predicted_display_time_ns,
           uint64_t *out_predicted_display_period_ns,
           uint64_t *out_min_display_period_ns)
{
	struct fake_timing *ft = fake_timing(uft);

	int64_t frame_id = ft->frame_id_generator++;
	uint64_t predicted_display_time_ns = predict_next_frame(ft);
	uint64_t desired_present_time_ns = predicted_display_time_ns - ft->present_offset_ns;
	uint64_t wake_up_time_ns = desired_present_time_ns - ft->app_time_ns;
	uint64_t present_slop_ns = U_TIME_HALF_MS_IN_NS;
	uint64_t predicted_display_period_ns = ft->frame_period_ns;
	uint64_t min_display_period_ns = ft->frame_period_ns;

	*out_frame_id = frame_id;
	*out_wake_up_time_ns = wake_up_time_ns;
	*out_desired_present_time_ns = desired_present_time_ns;
	*out_present_slop_ns = present_slop_ns;
	*out_predicted_display_time_ns = predicted_display_time_ns;
	*out_predicted_display_period_ns = predicted_display_period_ns;
	*out_min_display_period_ns = min_display_period_ns;
}

static void
ft_mark_point(struct u_frame_timing *uft, enum u_timing_point point, int64_t frame_id, uint64_t when_ns)
{
	// To help validate calling code.
	switch (point) {
	case U_TIMING_POINT_WAKE_UP: break;
	case U_TIMING_POINT_BEGIN: break;
	case U_TIMING_POINT_SUBMIT: break;
	default: assert(false);
	}
}

static void
ft_info(struct u_frame_timing *uft,
        int64_t frame_id,
        uint64_t desired_present_time_ns,
        uint64_t actual_present_time_ns,
        uint64_t earliest_present_time_ns,
        uint64_t present_margin_ns)
{
	/*
	 * The compositor might call this function because it selected the
	 * fake timing code even tho displaying timing is available.
	 */
}

static void
ft_destroy(struct u_frame_timing *uft)
{
	struct fake_timing *ft = fake_timing(uft);
	free(ft);
}


/*
 *
 * 'Exported' functions.
 *
 */

xrt_result_t
u_ft_fake_create(uint64_t estimated_frame_period_ns, struct u_frame_timing **out_uft)
{
	struct fake_timing *ft = U_TYPED_CALLOC(struct fake_timing);
	ft->base.predict = ft_predict;
	ft->base.mark_point = ft_mark_point;
	ft->base.info = ft_info;
	ft->base.destroy = ft_destroy;
	ft->frame_period_ns = estimated_frame_period_ns;

	// To make sure the code can start from a non-zero frame id.
	ft->frame_id_generator = 5;

	// Just a wild guess.
	ft->present_offset_ns = U_TIME_1MS_IN_NS * 4;

	// 20% of the frame time.
	ft->app_time_ns = get_percent_of_time(estimated_frame_period_ns, 20);

	// Make the next display time be in the future.
	ft->last_display_time_ns = os_monotonic_get_ns() + U_TIME_1MS_IN_NS * 50.0;

	// Return value.
	*out_uft = &ft->base;

	U_LOG_I("Created fake timing");

	return XRT_SUCCESS;
}
