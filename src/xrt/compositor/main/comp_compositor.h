// Copyright 2019-2020, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Main compositor written using Vulkan header.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @author Lubosz Sarnecki <lubosz.sarnecki@collabora.com>
 * @ingroup comp_main
 */

#pragma once

#include "xrt/xrt_gfx_vk.h"

#include "util/u_threading.h"
#include "util/u_index_fifo.h"

#include "main/comp_settings.h"
#include "main/comp_window.h"
#include "main/comp_renderer.h"


#ifdef __cplusplus
extern "C" {
#endif

#define NUM_FRAME_TIMES 50
#define COMP_MAX_LAYERS 16

/*
 *
 * Structs
 *
 */

/*!
 * A single swapchain image, holds the needed state for tracking image usage.
 *
 * @ingroup comp_main
 * @see comp_swapchain
 */
struct comp_swapchain_image
{
	//! Vulkan image to create view from.
	VkImage image;
	//! Exported memory backing the image.
	VkDeviceMemory memory;
	//! Sampler used by the renderer and distortion code.
	VkSampler sampler;
	//! Views used by the renderer and distortion code, for each array
	//! layer.
	struct
	{
		VkImageView *alpha;
		VkImageView *no_alpha;
	} views;
	//! The number of array slices in a texture, 1 == regular 2D texture.
	size_t array_size;
};

/*!
 * A swapchain that is almost a one to one mapping to a OpenXR swapchain.
 *
 * Not used by the window backend that uses the vk_swapchain to render to.
 *
 * @ingroup comp_main
 * @implements xrt_swapchain_fd
 * @see comp_compositor
 */
struct comp_swapchain
{
	struct xrt_swapchain_fd base;

	struct comp_compositor *c;

	struct comp_swapchain_image images[XRT_MAX_SWAPCHAIN_IMAGES];

	/*!
	 * This fifo is used to always give out the oldest image to acquire
	 * image, this should probably be made even smarter.
	 */
	struct u_index_fifo fifo;
};

/*!
 * A single layer.
 *
 * @ingroup comp_main
 * @see comp_layer_slot
 */
struct comp_layer
{
	/*!
	 * Up to two compositor swapchains referenced per layer.
	 *
	 * Unused elements should be set to null.
	 */
	struct comp_swapchain *scs[2];

	/*!
	 * All basic (trivially-serializable) data associated with a layer.
	 */
	struct xrt_layer_data data;
};

/*!
 * A stack of layers.
 *
 * @ingroup comp_main
 * @see comp_compositor
 */
struct comp_layer_slot
{
	enum xrt_blend_mode env_blend_mode;

	struct comp_layer layers[COMP_MAX_LAYERS];

	uint32_t num_layers;
};

/*!
 * State to emulate state transitions correctly.
 *
 * @ingroup comp_main
 */
enum comp_state
{
	COMP_STATE_READY = 0,
	COMP_STATE_PREPARED = 1,
	COMP_STATE_WAITED = 2,
	COMP_STATE_VISIBLE = 3,
	COMP_STATE_FOCUSED = 4,
};

/*!
 * Main compositor struct tying everything in the compositor together.
 *
 * @ingroup comp_main
 * @implements xrt_compositor_fd
 */
struct comp_compositor
{
	struct xrt_compositor_fd base;

	//! A link back to the compositor we are presenting to the client.
	struct xrt_compositor *client;

	//! Renderer helper.
	struct comp_renderer *r;

	//! The window or display we are using.
	struct comp_window *window;

	//! The device we are displaying to.
	struct xrt_device *xdev;

	//! The settings.
	struct comp_settings settings;

	//! Vulkan bundle of things.
	struct vk_bundle vk;

	//! Timestamp of last-rendered (immersive) frame.
	int64_t last_frame_time_ns;

	//! State for generating the correct set of events.
	enum comp_state state;

	//! Triple buffered layer stacks.
	struct comp_layer_slot slots[3];

	/*!
	 * @brief Data exclusive to the begin_frame/end_frame for computing an
	 * estimate of the app's needs.
	 */
	struct
	{
		int64_t last_begin;
		int64_t last_end;
	} app_profiling;

	//! The time our compositor needs to do rendering
	int64_t frame_overhead_ns;

	struct
	{
		//! Current Index for times_ns.
		int index;

		//! Timestamps of last-rendered (immersive) frames.
		int64_t times_ns[NUM_FRAME_TIMES];

		//! Frametimes between last-rendered (immersive) frames.
		float timings_ms[NUM_FRAME_TIMES];

		//! Average FPS of last NUM_FRAME_TIMES rendered frames.
		float fps;

		struct u_var_timing *debug_var;
	} compositor_frame_times;

	/*!
	 * @brief Estimated rendering time per frame of the application.
	 *
	 * Set by the begin_frame/end_frame code.
	 *
	 * @todo make this atomic.
	 */
	int64_t expected_app_duration_ns;
	//! The last time we provided in the results of wait_frame
	int64_t last_next_display_time;

	/*!
	 * The current state we are tracking.
	 *
	 * Settings is supposed to be read only.
	 */
	struct
	{
		uint32_t width;
		uint32_t height;
	} current;

	struct
	{
		//! Thread object for safely destroying swapchain.
		struct u_threading_stack destroy_swapchains;
	} threading;
};


/*
 *
 * Functions and helpers.
 *
 */

/*!
 * Convenience function to convert a xrt_swapchain to a comp_swapchain.
 *
 * @private @memberof comp_swapchain
 */
static inline struct comp_swapchain *
comp_swapchain(struct xrt_swapchain *xsc)
{
	return (struct comp_swapchain *)xsc;
}

/*!
 * Convenience function to convert a xrt_compositor to a comp_compositor.
 *
 * @private @memberof comp_compositor
 */
static inline struct comp_compositor *
comp_compositor(struct xrt_compositor *xc)
{
	return (struct comp_compositor *)xc;
}

/*!
 * Do garbage collection, destroying any resources that has been scheduled for
 * destruction from other threads.
 *
 * @public @memberof comp_compositor
 */
void
comp_compositor_garbage_collect(struct comp_compositor *c);

/*!
 * A compositor function that is implemented in the swapchain code.
 *
 * @public @memberof comp_compositor
 */
struct xrt_swapchain *
comp_swapchain_create(struct xrt_compositor *xc,
                      struct xrt_swapchain_create_info *info);

/*!
 * Swapchain destruct is delayed until it is safe to destroy them, this function
 * does the actual destruction and is called from @ref
 * comp_compositor_garbage_collect.
 *
 * @private @memberof comp_swapchain
 */
void
comp_swapchain_really_destroy(struct comp_swapchain *sc);

/*!
 * Printer helper.
 *
 * @public @memberof comp_compositor
 */
void
comp_compositor_print(struct comp_compositor *c,
                      const char *func,
                      const char *fmt,
                      ...) XRT_PRINTF_FORMAT(3, 4);

/*!
 * Spew level logging.
 *
 * @relates comp_compositor
 */
#define COMP_SPEW(c, ...)                                                      \
	do {                                                                   \
		if (c->settings.print_spew) {                                  \
			comp_compositor_print(c, __func__, __VA_ARGS__);       \
		}                                                              \
	} while (false)

/*!
 * Debug level logging.
 *
 * @relates comp_compositor
 */
#define COMP_DEBUG(c, ...)                                                     \
	do {                                                                   \
		if (c->settings.print_debug) {                                 \
			comp_compositor_print(c, __func__, __VA_ARGS__);       \
		}                                                              \
	} while (false)

/*!
 * Mode printing.
 *
 * @relates comp_compositor
 */
#define COMP_PRINT_MODE(c, ...)                                                \
	do {                                                                   \
		if (c->settings.print_modes) {                                 \
			comp_compositor_print(c, __func__, __VA_ARGS__);       \
		}                                                              \
	} while (false)

/*!
 * Error level logging.
 *
 * @relates comp_compositor
 */
#define COMP_ERROR(c, ...)                                                     \
	do {                                                                   \
		comp_compositor_print(c, __func__, __VA_ARGS__);               \
	} while (false)


#ifdef __cplusplus
}
#endif
