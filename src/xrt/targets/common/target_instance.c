// Copyright 2020, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Shared default implementation of the instance with compositor.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 */

#include "target_instance_parts.h"

#include "xrt/xrt_gfx_fd.h"

static int
t_instance_create_fd_compositor(struct xrt_instance *xinst,
                                struct xrt_device *xdev,
                                bool flip_y,
                                struct xrt_compositor_fd **out_xcfd)
{
	struct xrt_compositor_fd *xcfd =
	    xrt_gfx_provider_create_fd(xdev, flip_y);

	if (xcfd == NULL) {
		return -1;
	}

	*out_xcfd = xcfd;

	return 0;
}

/*
 *
 * Exported function(s).
 *
 */

int
xrt_instance_create(struct xrt_instance_info *i_info,
                    struct xrt_instance **out_xinst)
{
	struct xrt_prober *xp = NULL;

	int ret = xrt_prober_create_with_lists(&xp, &target_lists);
	if (ret < 0) {
		return ret;
	}

	struct t_instance *tinst = U_TYPED_CALLOC(struct t_instance);
	tinst->base.select = t_instance_select;
	tinst->base.create_fd_compositor = t_instance_create_fd_compositor;
	tinst->base.get_prober = t_instance_get_prober;
	tinst->base.destroy = t_instance_destroy;
	tinst->xp = xp;

	*out_xinst = &tinst->base;

	return 0;
}
