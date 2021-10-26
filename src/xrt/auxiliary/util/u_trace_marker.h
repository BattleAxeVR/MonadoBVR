// Copyright 2020-2021, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Tracing support code, see @ref tracing.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup aux_util
 */

#pragma once

#include "xrt/xrt_compiler.h"
#include "xrt/xrt_config_os.h"
#include "xrt/xrt_config_have.h"
#include "xrt/xrt_config_build.h"

#include <stdio.h>

#ifdef XRT_FEATURE_TRACING
#include <percetto.h>
#endif


#ifdef __cplusplus
extern "C" {
#endif

/*!
 * Should the extra tracks be enabled, see @ref tracing.
 *
 * @ingroup aux_util
 */
enum u_trace_which
{
	U_TRACE_WHICH_SERVICE,
	U_TRACE_WHICH_OPENXR,
};

/*!
 * Internal setup function, use @ref U_TRACE_TARGET_SETUP, see @ref tracing.
 *
 * @ingroup aux_util
 */
void
u_trace_marker_setup(enum u_trace_which which);

/*!
 * Must be called from a non-static/global constructor context.
 *
 * @ingroup aux_util
 */
void
u_trace_marker_init(void);

#define VK_TRACE_IDENT(IDENT) U_TRACE_EVENT(vk, #IDENT)
#define SINK_TRACE_IDENT(IDENT) U_TRACE_EVENT(sink, #IDENT)
#define XRT_TRACE_MARKER() U_TRACE_EVENT(xrt, __func__)
#define IPC_TRACE_MARKER() U_TRACE_EVENT(ipc, __func__)
#define OXR_TRACE_MARKER() U_TRACE_EVENT(oxr, __func__)
#define COMP_TRACE_MARKER() U_TRACE_EVENT(comp, __func__)
#define SINK_TRACE_MARKER() U_TRACE_EVENT(sink, __func__)


/*
 *
 * When enabled.
 *
 */

#ifdef XRT_FEATURE_TRACING

#ifndef XRT_OS_LINUX
#error "Tracing only supported on Linux"
#endif

#ifndef XRT_HAVE_PERCETTO
#error "Need to have Percetto/Perfetto"
#endif



#define U_TRACE_CATEGORIES(C, G)                                                                                       \
	C(vk, "vk")         /* Vulkan calls */                                                                         \
	C(xrt, "xrt")       /* Misc XRT calls */                                                                       \
	C(sink, "sink")     /* Sink/frameserver calls */                                                               \
	C(oxr, "st/oxr")    /* OpenXR State Tracker calls */                                                           \
	C(comp, "comp")     /* Compositor calls  */                                                                    \
	C(ipc, "ipc")       /* IPC calls */                                                                            \
	C(timing, "timing") /* Timing calls */

PERCETTO_CATEGORY_DECLARE(U_TRACE_CATEGORIES)

PERCETTO_TRACK_DECLARE(rt_cpu);
PERCETTO_TRACK_DECLARE(rt_allotted);
PERCETTO_TRACK_DECLARE(rt_gpu);
PERCETTO_TRACK_DECLARE(rt_margin);
PERCETTO_TRACK_DECLARE(rt_error);
PERCETTO_TRACK_DECLARE(rt_info);
PERCETTO_TRACK_DECLARE(rt_present);
PERCETTO_TRACK_DECLARE(ft_cpu);
PERCETTO_TRACK_DECLARE(ft_draw);

#define U_TRACE_EVENT(CATEGORY, NAME) TRACE_EVENT(CATEGORY, NAME)
#define U_TRACE_EVENT_BEGIN_ON_TRACK(CATEGORY, TRACK, TIME, NAME)                                                      \
	TRACE_EVENT_BEGIN_ON_TRACK(CATEGORY, TRACK, TIME, NAME)
#define U_TRACE_EVENT_BEGIN_ON_TRACK_DATA(CATEGORY, TRACK, TIME, NAME, ...)                                            \
	TRACE_EVENT_BEGIN_ON_TRACK_DATA(CATEGORY, TRACK, TIME, NAME, __VA_ARGS__)
#define U_TRACE_EVENT_END_ON_TRACK(CATEGORY, TRACK, TIME) TRACE_EVENT_END_ON_TRACK(CATEGORY, TRACK, TIME)
#define U_TRACE_CATEGORY_IS_ENABLED(CATEGORY) PERCETTO_CATEGORY_IS_ENABLED(CATEGORY)
#define U_TRACE_INSTANT_ON_TRACK(CATEGORY, TRACK, TIME, NAME)                                                          \
	TRACE_ANY_WITH_ARGS(PERCETTO_EVENT_INSTANT, CATEGORY, &g_percetto_track_##TRACK, TIME, NAME, 0)
#define U_TRACE_DATA(fd, type, data) u_trace_data(fd, type, (void *)&(data), sizeof(data))

#define U_TRACE_TARGET_SETUP(WHICH)                                                                                    \
	void __attribute__((constructor(101))) u_trace_marker_constructor(void);                                       \
                                                                                                                       \
	void u_trace_marker_constructor(void)                                                                          \
	{                                                                                                              \
		u_trace_marker_setup(WHICH);                                                                           \
	}


#else // XRT_FEATURE_TRACING


/*
 *
 * When disabled.
 *
 */

#define U_TRACE_EVENT(CATEGORY, NAME)                                                                                  \
	do {                                                                                                           \
	} while (false)

#define U_TRACE_EVENT_BEGIN_ON_TRACK(CATEGORY, TRACK, TIME, NAME)                                                      \
	do {                                                                                                           \
	} while (false)

#define U_TRACE_EVENT_BEGIN_ON_TRACK_DATA(CATEGORY, TRACK, TIME, NAME, ...)                                            \
	do {                                                                                                           \
	} while (false)

#define U_TRACE_EVENT_END_ON_TRACK(CATEGORY, TRACK, TIME)                                                              \
	do {                                                                                                           \
	} while (false)

#define U_TRACE_INSTANT_ON_TRACK(CATEGORY, TRACK, TIME, NAME)                                                          \
	do {                                                                                                           \
	} while (false)

#define U_TRACE_CATEGORY_IS_ENABLED(_) (false)

/*!
 * Add to target c file to enable tracing, see @ref tracing.
 *
 * @ingroup aux_util
 */
#define U_TRACE_TARGET_SETUP(WHICH)

#endif // XRT_FEATURE_TRACING


#ifdef __cplusplus
}
#endif
