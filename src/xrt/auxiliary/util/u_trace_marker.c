// Copyright 2020-2021, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Tracing support code, see @ref tracing.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup aux_util
 */

#include "xrt/xrt_compiler.h"
#include "xrt/xrt_config_os.h"
#include "xrt/xrt_config_have.h"

#include "os/os_time.h"

#include "util/u_trace_marker.h"

#include <inttypes.h>


#ifdef XRT_FEATURE_TRACING

PERCETTO_CATEGORY_DEFINE(U_TRACE_CATEGORIES)

PERCETTO_TRACK_DEFINE(rt_cpu, PERCETTO_TRACK_EVENTS);
PERCETTO_TRACK_DEFINE(rt_allotted, PERCETTO_TRACK_EVENTS);
PERCETTO_TRACK_DEFINE(rt_gpu, PERCETTO_TRACK_EVENTS);
PERCETTO_TRACK_DEFINE(rt_margin, PERCETTO_TRACK_EVENTS);
PERCETTO_TRACK_DEFINE(rt_error, PERCETTO_TRACK_EVENTS);
PERCETTO_TRACK_DEFINE(rt_info, PERCETTO_TRACK_EVENTS);
PERCETTO_TRACK_DEFINE(rt_present, PERCETTO_TRACK_EVENTS);
PERCETTO_TRACK_DEFINE(ft_cpu, PERCETTO_TRACK_EVENTS);
PERCETTO_TRACK_DEFINE(ft_draw, PERCETTO_TRACK_EVENTS);


static enum u_trace_which static_which;
static bool static_inited = false;

void
u_trace_marker_setup(enum u_trace_which which)
{
	static_which = which;

	I_PERCETTO_TRACK_PTR(rt_cpu)->name = "RT 1 Sleep";
	I_PERCETTO_TRACK_PTR(rt_allotted)->name = "RT 2 Allotted time";
	I_PERCETTO_TRACK_PTR(rt_gpu)->name = "RT 3 GPU";
	I_PERCETTO_TRACK_PTR(rt_margin)->name = "RT 4 Margin";
	I_PERCETTO_TRACK_PTR(rt_error)->name = "RT 5 Error";
	I_PERCETTO_TRACK_PTR(rt_info)->name = "RT 6 Info";
	I_PERCETTO_TRACK_PTR(rt_present)->name = "RT 7 Present";

	I_PERCETTO_TRACK_PTR(ft_cpu)->name = "FT 1 App";
	I_PERCETTO_TRACK_PTR(ft_draw)->name = "FT 2 Draw";
}

void
u_trace_marker_init(void)
{
	if (static_inited) {
		return;
	}
	static_inited = true;

	int ret = PERCETTO_INIT(PERCETTO_CLOCK_MONOTONIC);
	if (ret != 0) {
		return;
	}

	if (static_which == U_TRACE_WHICH_SERVICE) {
		PERCETTO_REGISTER_TRACK(rt_cpu);
		PERCETTO_REGISTER_TRACK(rt_allotted);
		PERCETTO_REGISTER_TRACK(rt_gpu);
		PERCETTO_REGISTER_TRACK(rt_margin);
		PERCETTO_REGISTER_TRACK(rt_error);
		PERCETTO_REGISTER_TRACK(rt_info);
		PERCETTO_REGISTER_TRACK(rt_present);

		PERCETTO_REGISTER_TRACK(ft_cpu);
		PERCETTO_REGISTER_TRACK(ft_draw);
	}
}

#else /* XRT_FEATURE_TRACING */

void
u_trace_marker_setup(enum u_trace_which which)
{
	(void)which;

	// Noop
}

void
u_trace_marker_init(void)
{
	// Noop
}

#endif /* XRT_FEATURE_TRACING */
