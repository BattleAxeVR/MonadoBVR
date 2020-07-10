// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Header defining a XRT graphics provider.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup xrt_iface
 */

#pragma once

#include "xrt/xrt_device.h"
#include "xrt/xrt_compositor.h"
#include "xrt/xrt_vulkan_includes.h"


#ifdef __cplusplus
extern "C" {
#endif

struct time_state;


/*!
 * @ingroup xrt_iface
 */
extern const char *xrt_gfx_vk_instance_extensions;

/*!
 * @ingroup xrt_iface
 */
extern const char *xrt_gfx_vk_device_extensions;

/*!
 * @ingroup xrt_iface
 */
void
xrt_gfx_vk_get_versions(struct xrt_api_requirements *ver);

/*!
 * Create a Vulkan compositor client.
 *
 * @ingroup xrt_iface
 * @public @memberof xrt_compositor_fd
 */
struct xrt_compositor_vk *
xrt_gfx_vk_provider_create(struct xrt_compositor_fd *xcfd,
                           VkInstance instance,
                           PFN_vkGetInstanceProcAddr get_instance_proc_addr,
                           VkPhysicalDevice physical_device,
                           VkDevice device,
                           uint32_t queue_family_index,
                           uint32_t queue_index);


#ifdef __cplusplus
}
#endif
