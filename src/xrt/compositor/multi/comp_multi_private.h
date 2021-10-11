// Copyright 2021, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Multi-client compositor internal structs.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup comp_multi
 */

#pragma once

#include "xrt/xrt_compiler.h"
#include "xrt/xrt_defines.h"
#include "xrt/xrt_compositor.h"

#include "os/os_time.h"
#include "os/os_threading.h"

#include "util/u_timing.h"

#ifdef __cplusplus
extern "C" {
#endif


#define MULTI_MAX_CLIENTS 64
#define MULTI_MAX_LAYERS 16


/*
 *
 * Native compositor.
 *
 */

/*!
 * Data for a single composition layer.
 *
 * Similar in function to @ref comp_layer
 *
 * @ingroup comp_multi
 */
struct multi_layer_entry
{
	/*!
	 * Device to get pose from.
	 */
	struct xrt_device *xdev;

	/*!
	 * Pointers to swapchains.
	 *
	 * How many are actually used depends on the value of @p data.type
	 */
	struct xrt_swapchain *xscs[4];

	/*!
	 * All basic (trivially-serializable) data associated with a layer,
	 * aside from which swapchain(s) are used.
	 */
	struct xrt_layer_data data;
};

/*!
 * Render state for a single client, including all layers.
 *
 * @ingroup comp_multi
 */
struct multi_layer_slot
{
	uint64_t display_time_ns; //!< When should this be shown, @see XrFrameEndInfo::displayTime.
	enum xrt_blend_mode env_blend_mode;
	uint32_t num_layers;
	struct multi_layer_entry layers[MULTI_MAX_LAYERS];
	bool active;
};

/*!
 * Render state for a single client, including all layers.
 *
 * @ingroup comp_multi
 */
struct multi_event
{
	struct multi_event *next;
	union xrt_compositor_event xce;
};

/*!
 * A single compositor.
 *
 * @ingroup comp_multi
 */
struct multi_compositor
{
	struct xrt_compositor_native base;

	// Client info.
	struct xrt_session_info xsi;

	//! Owning system compositor.
	struct multi_system_compositor *msc;

	//! Only matters for Windows and in process.
	struct os_precise_sleeper sleeper;

	struct
	{
		struct os_mutex mutex;
		struct multi_event *next;
	} event;

	struct
	{
		struct
		{
			bool visible;
			bool focused;
		} sent;
		struct
		{
			bool visible;
			bool focused;
		} current;

		int64_t z_order;
	} state;

	//! Lock for all of the slots.
	struct os_mutex slot_lock;

	/*!
	 * Currently being transferred or waited on.
	 * Not protected by the slot lock as it is only touched by the client thread.
	 */
	struct multi_layer_slot progress;

	//! Scheduled frames for a future timepoint.
	struct multi_layer_slot scheduled;

	/*!
	 * Fully ready to be used.
	 * Not protected by the slot lock as it is only touched by the main render loop thread.
	 */
	struct multi_layer_slot delivered;

	struct u_render_timing *urt;
};

static inline struct multi_compositor *
multi_compositor(struct xrt_compositor *xc)
{
	return (struct multi_compositor *)xc;
}

/*!
 * Create a multi client wrapper compositor.
 *
 * @ingroup comp_multi
 */
xrt_result_t
multi_compositor_create(struct multi_system_compositor *msc,
                        const struct xrt_session_info *xsi,
                        struct xrt_compositor_native **out_xcn);

/*!
 * Push a event to be delivered to the client.
 *
 * @ingroup comp_multi
 */
void
multi_compositor_push_event(struct multi_compositor *mc, const union xrt_compositor_event *xce);

/*!
 * Deliver any scheduled frames at that is to be display at or after the given @p display_time_ns. Called by the render
 * thread and copies data from multi_compositor::scheduled to multi_compositor::delivered while holding the slot_lock.
 *
 * @ingroup comp_multi
 */
void
multi_compositor_deliver_any_frames(struct multi_compositor *mc, uint64_t display_time_ns);


/*
 *
 * System compositor.
 *
 */

struct multi_system_compositor
{
	struct xrt_system_compositor base;

	//! Extra functions to handle multi client.
	struct xrt_multi_compositor_control xmcc;

	//! Real native compositor.
	struct xrt_compositor_native *xcn;

	//! Render loop thread.
	struct os_thread_helper oth;

	/*!
	 * This mutex protects the list of client compositor
	 * and the rendering timings on it.
	 */
	struct os_mutex list_and_timing_lock;

	struct
	{
		uint64_t predicted_display_time_ns;
		uint64_t predicted_display_period_ns;
		uint64_t diff_ns;
	} last_timings;

	struct multi_compositor *clients[MULTI_MAX_CLIENTS];
};

static inline struct multi_system_compositor *
multi_system_compositor(struct xrt_system_compositor *xsc)
{
	return (struct multi_system_compositor *)xsc;
}



#ifdef __cplusplus
}
#endif
