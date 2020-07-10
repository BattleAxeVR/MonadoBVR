// Copyright 2018-2020, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Holds OpenGL-specific session functions.
 * @author Drew DeVault <sir@cmpwn.com>
 * @author Simon Ser <contact@emersion.fr>
 * @ingroup oxr_main
 * @ingroup comp_client
 */

#include <stdlib.h>

#include "util/u_misc.h"

#include "oxr_objects.h"
#include "oxr_logger.h"
#include "oxr_two_call.h"
#include "oxr_handle.h"

#include "xrt/xrt_instance.h"

#ifdef XR_USE_PLATFORM_EGL
#define EGL_NO_X11              // libglvnd
#define MESA_EGL_NO_X11_HEADERS // mesa
#include <EGL/egl.h>
#include "xrt/xrt_gfx_egl.h"


// Not forward declared by mesa
typedef EGLBoolean(EGLAPIENTRYP PFNEGLQUERYCONTEXTPROC)(EGLDisplay dpy,
                                                        EGLContext ctx,
                                                        EGLint attribute,
                                                        EGLint *value);
#endif

#ifdef XR_USE_PLATFORM_EGL

XrResult
oxr_session_populate_egl(struct oxr_logger *log,
                         struct oxr_system *sys,
                         XrGraphicsBindingEGLMNDX const *next,
                         struct oxr_session *sess)
{
	EGLint egl_client_type;

	PFNEGLQUERYCONTEXTPROC eglQueryContext =
	    (PFNEGLQUERYCONTEXTPROC)next->getProcAddress("eglQueryContext");
	if (!eglQueryContext) {
		return oxr_error(
		    log, XR_ERROR_INITIALIZATION_FAILED,
		    "Call to getProcAddress(eglQueryContext) failed");
	}

	if (!eglQueryContext(next->display, next->context,
	                     EGL_CONTEXT_CLIENT_TYPE, &egl_client_type)) {
		return oxr_error(
		    log, XR_ERROR_INITIALIZATION_FAILED,
		    "Call to eglQueryContext(EGL_CONTEXT_CLIENT_TYPE) failed");
	}

	if (egl_client_type != EGL_OPENGL_API &&
	    egl_client_type != EGL_OPENGL_ES_API) {
		return oxr_error(log, XR_ERROR_INITIALIZATION_FAILED,
		                 "Unsupported EGL client type");
	}

	struct xrt_compositor_fd *xcfd = NULL;
	int ret = xrt_instance_create_fd_compositor(sys->inst->xinst, sys->head,
	                                            true, &xcfd);
	if (ret < 0 || xcfd == NULL) {
		return oxr_error(log, XR_ERROR_INITIALIZATION_FAILED,
		                 "Failed to create an fd compositor '%i'", ret);
	}

	struct xrt_compositor_gl *xcgl =
	    xrt_gfx_provider_create_gl_egl(xcfd, next->display, next->config,
	                                   next->context, next->getProcAddress);

	if (xcgl == NULL) {
		xcfd->base.destroy(&xcfd->base);
		return oxr_error(log, XR_ERROR_INITIALIZATION_FAILED,
		                 "Failed to create an egl client compositor");
	}

	sess->compositor = &xcgl->base;
	sess->create_swapchain = oxr_swapchain_gl_create;

	return XR_SUCCESS;
}

#endif
