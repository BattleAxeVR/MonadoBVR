// Copyright 2020, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Code to handle distortion parameters and fov.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup aux_distortion
 */

#include "xrt/xrt_device.h"

#include "math/m_mathinclude.h"

#include "util/u_misc.h"
#include "util/u_device.h"
#include "util/u_distortion.h"

#include "cardboard_device.pb.h"
#include "pb_decode.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "util/u_logging.h"

void
u_distortion_cardboard_calculate(const struct u_cardboard_distortion_arguments *args,
                                 struct xrt_hmd_parts *parts,
                                 struct u_cardboard_distortion *out_dist)
{
	/*
	 * HMD parts
	 */

	uint32_t w_pixels = args->screen.w_pixels / 2;
	uint32_t h_pixels = args->screen.h_pixels;

	// Base assumption, the driver can change afterwards.
	if (parts->blend_mode_count == 0) {
		size_t idx = 0;
		parts->blend_modes[idx++] = XRT_BLEND_MODE_OPAQUE;
		parts->blend_mode_count = idx;
	}

	// Use the full screen.
	parts->screens[0].w_pixels = args->screen.w_pixels;
	parts->screens[0].h_pixels = args->screen.h_pixels;

	parts->views[0].viewport.x_pixels = 0;
	parts->views[0].viewport.y_pixels = 0;
	parts->views[0].viewport.w_pixels = w_pixels;
	parts->views[0].viewport.h_pixels = h_pixels;
	parts->views[0].display.w_pixels = w_pixels;
	parts->views[0].display.h_pixels = h_pixels;
	parts->views[0].rot = u_device_rotation_ident;
	parts->distortion.fov[0] = args->fov;

	parts->views[1].viewport.x_pixels = w_pixels;
	parts->views[1].viewport.y_pixels = 0;
	parts->views[1].viewport.w_pixels = w_pixels;
	parts->views[1].viewport.h_pixels = h_pixels;
	parts->views[1].display.w_pixels = w_pixels;
	parts->views[1].display.h_pixels = h_pixels;
	parts->views[1].rot = u_device_rotation_ident;
	parts->distortion.fov[1] = args->fov;


	/*
	 * Left values
	 */

	// clang-format off
	struct u_cardboard_distortion_values l_values = {0};
	l_values.distortion_k[0] = args->distortion_k[0];
	l_values.distortion_k[1] = args->distortion_k[1];
	l_values.distortion_k[2] = args->distortion_k[2];
	l_values.distortion_k[3] = args->distortion_k[3];
	l_values.distortion_k[4] = args->distortion_k[4];
	l_values.screen.size.x = args->screen.w_meters;
	l_values.screen.size.y = args->screen.h_meters;
	l_values.screen.offset.x = (args->screen.w_meters - args->inter_lens_distance_meters) / 2.0f;
	l_values.screen.offset.y = args->lens_y_center_on_screen_meters;
	// clang-format on

	// Turn into tanangles
	l_values.screen.size.x /= args->screen_to_lens_distance_meters;
	l_values.screen.size.y /= args->screen_to_lens_distance_meters;
	l_values.screen.offset.x /= args->screen_to_lens_distance_meters;
	l_values.screen.offset.y /= args->screen_to_lens_distance_meters;

	// Tan-angle to texture coordinates
	// clang-format off
	l_values.texture.size.x = tanf(-args->fov.angle_left) + tanf(args->fov.angle_right);
	l_values.texture.size.y = tanf(args->fov.angle_up) + tanf(-args->fov.angle_down);
	l_values.texture.offset.x = tanf(-args->fov.angle_left);
	l_values.texture.offset.y = tanf(-args->fov.angle_down);
	// clang-format on

	// Fix up views not covering the entire screen.
	l_values.screen.size.x /= 2.0;


	/*
	 * Right values
	 */

	// clang-format off
	struct u_cardboard_distortion_values r_values = {0};
	r_values.distortion_k[0] = args->distortion_k[0];
	r_values.distortion_k[1] = args->distortion_k[1];
	r_values.distortion_k[2] = args->distortion_k[2];
	r_values.distortion_k[3] = args->distortion_k[3];
	r_values.distortion_k[4] = args->distortion_k[4];
	r_values.screen.size.x = args->screen.w_meters;
	r_values.screen.size.y = args->screen.h_meters;
	r_values.screen.offset.x = (args->screen.w_meters + args->inter_lens_distance_meters) / 2.0f;
	r_values.screen.offset.y = args->lens_y_center_on_screen_meters;
	// clang-format on

	// Turn into tanangles
	r_values.screen.size.x /= args->screen_to_lens_distance_meters;
	r_values.screen.size.y /= args->screen_to_lens_distance_meters;
	r_values.screen.offset.x /= args->screen_to_lens_distance_meters;
	r_values.screen.offset.y /= args->screen_to_lens_distance_meters;

	// Tanangle to texture coordinates
	// clang-format off
	r_values.texture.size.x = tanf(-args->fov.angle_left) + tanf(args->fov.angle_right);
	r_values.texture.size.y = tanf(args->fov.angle_up) + tanf(-args->fov.angle_down);
	r_values.texture.offset.x = tanf(-args->fov.angle_left);
	r_values.texture.offset.y = tanf(-args->fov.angle_down);
	// clang-format on

	// Fix up views not covering the entire screen.
	r_values.screen.size.x /= 2.0;
	r_values.screen.offset.x -= r_values.screen.size.x;


	/*
	 * Write results.
	 */

	// Copy the arguments.
	out_dist->args = *args;

	// Save the results.
	out_dist->values[0] = l_values;
	out_dist->values[1] = r_values;
}

bool
u_cardboard_distortion_arguments_read(const char *proto_file, struct u_cardboard_distortion_arguments *out_dist)
{
    if (proto_file == NULL || out_dist == NULL) {
        return false;
    }

    FILE *file = fopen(proto_file, "rb");
    if (file == NULL) {
        U_LOG_E("Failed to load calibration file: %s", proto_file);
        return false;
    }

    // get total fize size.
    fseek(file, 0L, SEEK_END);
    const size_t buffer_size = ftell(file);
    fseek(file, 0L, SEEK_SET);

    uint8_t *device_params_buffer = (uint8_t *)malloc(buffer_size * sizeof(uint8_t));
    if (device_params_buffer == NULL) {
        fclose(file);
        return false;
    }

    const size_t read_size = fread(device_params_buffer, sizeof(uint8_t), buffer_size, file);
    fclose(file);
    if (read_size != buffer_size) {
        U_LOG_E("Failed to get file size");
        free(device_params_buffer);
        return false;
    }

    // cardboard sdk writes current_device_params file with a 8-byte header.
    const size_t offset = sizeof(uint32_t) * 2;
    const size_t proto_size = buffer_size - offset;
    const uint8_t *proto = device_params_buffer + offset;

    DeviceParams msg = DeviceParams_init_default;
    pb_istream_t stream = pb_istream_from_buffer(proto, proto_size);
    const bool result = pb_decode(&stream, DeviceParams_fields, &msg);
    free(device_params_buffer);

    if (result) {
        memcpy(out_dist->distortion_k, msg.distortion_coefficients, sizeof(out_dist->distortion_k));
        if (msg.has_inter_lens_distance) {
            out_dist->inter_lens_distance_meters = msg.inter_lens_distance;
        }
        if (msg.has_screen_to_lens_distance) {
            out_dist->screen_to_lens_distance_meters = msg.screen_to_lens_distance;
        }
#define XR_DEG_TO_RAD(x) (float)(x * M_PI / 180.0)
        const float *const device_fov = msg.left_eye_field_of_view_angles;
        const struct xrt_fov fov = {.angle_left = -XR_DEG_TO_RAD(device_fov[0]),
                .angle_right = XR_DEG_TO_RAD(device_fov[1]),
                .angle_down = -XR_DEG_TO_RAD(device_fov[2]),
                .angle_up = XR_DEG_TO_RAD(device_fov[3])};
#undef XR_DEG_TO_RAD
        out_dist->fov = fov;
        U_LOG_I("Successfully loaded calibration: vendor: \"%s\" model: \"%s\"",
                msg.has_vendor ? msg.vendor : "Unknown", msg.has_model ? msg.model : "Unknown");
    } else
        U_LOG_E("Failed to load calibration file: %s", proto_file);
    return result;
}
