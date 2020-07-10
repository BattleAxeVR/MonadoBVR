// Copyright 2020, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Compositor quad rendering.
 * @author Lubosz Sarnecki <lubosz.sarnecki@collabora.com>
 * @ingroup comp_main
 */

#pragma once

#include "vk/vk_helpers.h"
#include "comp_compositor.h"

struct layer_transformation
{
	struct xrt_matrix_4x4 mvp;
	bool flip_y;
};

struct comp_render_layer
{
	struct vk_bundle *vk;

	enum xrt_layer_eye_visibility visibility;
	bool view_space;

	enum xrt_layer_type type;

	struct layer_transformation transformation[2];
	struct vk_buffer transformation_ubos[2];

	VkDescriptorPool descriptor_pool;
	VkDescriptorSet descriptor_sets[2];

	struct xrt_matrix_4x4 model_matrix;
};

struct comp_render_layer *
comp_layer_create(struct vk_bundle *vk,
                  enum xrt_layer_type type,
                  VkDescriptorSetLayout *layout);

void
comp_layer_draw(struct comp_render_layer *self,
                uint32_t eye,
                VkPipeline pipeline,
                VkPipelineLayout pipeline_layout,
                VkCommandBuffer cmd_buffer,
                const struct vk_buffer *vertex_buffer,
                const struct xrt_matrix_4x4 *vp_world,
                const struct xrt_matrix_4x4 *vp_eye);

void
comp_layer_set_model_matrix(struct comp_render_layer *self,
                            const struct xrt_matrix_4x4 *m);

void
comp_layer_destroy(struct comp_render_layer *self);

void
comp_layer_update_descriptors(struct comp_render_layer *self,
                              VkSampler sampler,
                              VkImageView image_view);

void
comp_layer_update_stereo_descriptors(struct comp_render_layer *self,
                                     VkSampler left_sampler,
                                     VkSampler right_sampler,
                                     VkImageView left_image_view,
                                     VkImageView right_image_view);

void
comp_layer_set_flip_y(struct comp_render_layer *self, bool flip_y);
