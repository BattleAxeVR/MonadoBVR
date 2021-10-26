// Copyright 2019-2021, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  The compositor compute based rendering code.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup comp_main
 */

#include "math/m_api.h"
#include "math/m_matrix_4x4_f64.h"

#include "main/comp_compositor.h"
#include "render/comp_render.h"

#include <stdio.h>


/*
 *
 * Defines
 *
 */

#define C(c)                                                                                                           \
	do {                                                                                                           \
		VkResult ret = c;                                                                                      \
		if (ret != VK_SUCCESS) {                                                                               \
			return false;                                                                                  \
		}                                                                                                      \
	} while (false)

#define D(TYPE, thing)                                                                                                 \
	if (thing != VK_NULL_HANDLE) {                                                                                 \
		vk->vkDestroy##TYPE(vk, vk->device, thing, NULL);                                                      \
		thing = VK_NULL_HANDLE;                                                                                \
	}

#define DD(pool, thing)                                                                                                \
	if (thing != VK_NULL_HANDLE) {                                                                                 \
		free_descriptor_set(vk, pool, thing);                                                                  \
		thing = VK_NULL_HANDLE;                                                                                \
	}


/*
 *
 * Helper functions.
 *
 */

/*
 * For dispatching compute to the view, calculate the number of groups.
 */
static void
calc_dispatch_dims(const struct comp_viewport_data views[2], uint32_t *out_w, uint32_t *out_h)
{
#define IMAX(a, b) ((a) > (b) ? (a) : (b))
	uint32_t w = IMAX(views[0].w, views[1].w);
	uint32_t h = IMAX(views[0].h, views[1].h);
#undef IMAX

	// Power of two divide and round up.
#define P2_DIVIDE_ROUND_UP(v, div) ((v + (div - 1)) / div)
	w = P2_DIVIDE_ROUND_UP(w, 8);
	h = P2_DIVIDE_ROUND_UP(h, 8);
#undef P2_DIVIDE_ROUND_UP

	*out_w = w;
	*out_h = h;
}

/*!
 * Create a simplified projection matrix for timewarp.
 */
static void
calc_projection(const struct xrt_fov *fov, struct xrt_matrix_4x4_f64 *result)
{
	const double tan_left = tan(fov->angle_left);
	const double tan_right = tan(fov->angle_right);

	const double tan_down = tan(fov->angle_down);
	const double tan_up = tan(fov->angle_up);

	const double tan_width = tan_right - tan_left;
	const double tan_height = tan_up - tan_down;

	const double near = 0.5;
	const double far = 1.5;

	const double a11 = 2 / tan_width;
	const double a22 = 2 / tan_height;

	const double a31 = (tan_right + tan_left) / tan_width;
	const double a32 = (tan_up + tan_down) / tan_height;

	const float a33 = -far / (far - near);
	const float a43 = -(far * near) / (far - near);


#if 1
	// We skip a33 & a43 because we don't have depth.
	(void)a33;
	(void)a43;

	// clang-format off
	*result = (struct xrt_matrix_4x4_f64){
		{
			      a11,         0,  0,  0,
			        0,       a22,  0,  0,
			      a31,       a32, -1,  0,
			        0,         0,  0,  1,
		}
	};
	// clang-format on
#else
	// clang-format off
	*result = (struct xrt_matrix_4x4_f64) {
		.v = {
			a11, 0, 0, 0,
			0, a22, 0, 0,
			a31, a32, a33, -1,
			0, 0, a43, 0,
		}
	};
	// clang-format on
#endif
}

static void
calc_time_warp_matrix(struct comp_rendering_compute *crc,
                      const struct xrt_pose *src_pose,
                      const struct xrt_fov *src_fov,
                      const struct xrt_pose *new_pose,
                      struct xrt_matrix_4x4 *matrix)
{
	// Src projection matrix.
	struct xrt_matrix_4x4_f64 src_proj;
	calc_projection(src_fov, &src_proj);

	// Src rotation matrix.
	struct xrt_matrix_4x4_f64 src_rot_inv;
	struct xrt_quat src_q = src_pose->orientation;
	m_mat4_f64_orientation(&src_q, &src_rot_inv); // This is a model matrix, a inverted view matrix.

	// New rotation matrix.
	struct xrt_matrix_4x4_f64 new_rot, new_rot_inv;
	struct xrt_quat new_q = new_pose->orientation;
	m_mat4_f64_orientation(&new_q, &new_rot_inv); // This is a model matrix, a inverted view matrix.
	m_mat4_f64_invert(&new_rot_inv, &new_rot);    // Invert to make it a view matrix.

	// Combine both rotation matricies to get difference.
	struct xrt_matrix_4x4_f64 delta_rot, delta_rot_inv;
	m_mat4_f64_multiply(&new_rot, &src_rot_inv, &delta_rot);
	m_mat4_f64_invert(&delta_rot, &delta_rot_inv);

	// Combine the source projection matrix and
	struct xrt_matrix_4x4_f64 result;
	m_mat4_f64_multiply(&src_proj, &delta_rot_inv, &result);

	// Reset if timewarp is off.
	if (crc->c->debug.atw_off) {
		result = src_proj;
	}

	// Convert from f64 to f32.
	for (int i = 0; i < 16; i++) {
		matrix->v[i] = result.v[i];
	}
}


/*
 *
 * Vulkan helpers.
 *
 */

static VkResult
create_command_buffer(struct vk_bundle *vk, VkCommandBuffer *out_cmd)
{
	VkResult ret;

	VkCommandBufferAllocateInfo cmd_buffer_info = {
	    .sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO,
	    .commandPool = vk->cmd_pool,
	    .level = VK_COMMAND_BUFFER_LEVEL_PRIMARY,
	    .commandBufferCount = 1,
	};

	VkCommandBuffer cmd = VK_NULL_HANDLE;

	os_mutex_lock(&vk->cmd_pool_mutex);

	ret = vk->vkAllocateCommandBuffers( //
	    vk->device,                     //
	    &cmd_buffer_info,               //
	    &cmd);                          //

	os_mutex_unlock(&vk->cmd_pool_mutex);

	if (ret != VK_SUCCESS) {
		VK_ERROR(vk, "vkCreateFramebuffer failed: %s", vk_result_string(ret));
		return ret;
	}

	*out_cmd = cmd;

	return VK_SUCCESS;
}

static void
destroy_command_buffer(struct vk_bundle *vk, VkCommandBuffer command_buffer)
{
	os_mutex_lock(&vk->cmd_pool_mutex);

	vk->vkFreeCommandBuffers( //
	    vk->device,           // device
	    vk->cmd_pool,         // commandPool
	    1,                    // commandBufferCount
	    &command_buffer);     // pCommandBuffers

	os_mutex_unlock(&vk->cmd_pool_mutex);
}

static VkResult
begin_command_buffer(struct vk_bundle *vk, VkCommandBuffer command_buffer)
{
	VkResult ret;

	VkCommandBufferBeginInfo command_buffer_info = {
	    .sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO,
	};

	ret = vk->vkBeginCommandBuffer( //
	    command_buffer,             //
	    &command_buffer_info);      //
	if (ret != VK_SUCCESS) {
		VK_ERROR(vk, "vkBeginCommandBuffer failed: %s", vk_result_string(ret));
		return ret;
	}

	return VK_SUCCESS;
}

static VkResult
end_command_buffer(struct vk_bundle *vk, VkCommandBuffer command_buffer)
{
	VkResult ret;

	// End the command buffer.
	ret = vk->vkEndCommandBuffer( //
	    command_buffer);          //
	if (ret != VK_SUCCESS) {
		VK_ERROR(vk, "vkEndCommandBuffer failed: %s", vk_result_string(ret));
		return ret;
	}

	return VK_SUCCESS;
}

static VkResult
create_descriptor_set(struct vk_bundle *vk,
                      VkDescriptorPool descriptor_pool,
                      VkDescriptorSetLayout descriptor_layout,
                      VkDescriptorSet *out_descriptor_set)
{
	VkResult ret;

	VkDescriptorSetAllocateInfo alloc_info = {
	    .sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO,
	    .descriptorPool = descriptor_pool,
	    .descriptorSetCount = 1,
	    .pSetLayouts = &descriptor_layout,
	};

	VkDescriptorSet descriptor_set = VK_NULL_HANDLE;
	ret = vk->vkAllocateDescriptorSets( //
	    vk->device,                     //
	    &alloc_info,                    //
	    &descriptor_set);               //
	if (ret != VK_SUCCESS) {
		VK_DEBUG(vk, "vkAllocateDescriptorSets failed: %s", vk_result_string(ret));
		return ret;
	}

	*out_descriptor_set = descriptor_set;

	return VK_SUCCESS;
}

XRT_MAYBE_UNUSED static void
update_compute_discriptor_set(struct vk_bundle *vk,
                              uint32_t src_binding,
                              VkSampler src_samplers[2],
                              VkImageView src_image_views[2],
                              uint32_t distortion_binding,
                              VkSampler distortion_samplers[6],
                              VkImageView distortion_image_views[6],
                              uint32_t target_binding,
                              VkImageView target_image_view,
                              uint32_t ubo_binding,
                              VkBuffer ubo_buffer,
                              VkDeviceSize ubo_size,
                              VkDescriptorSet descriptor_set)
{
	VkDescriptorImageInfo src_image_info[2] = {
	    {
	        .sampler = src_samplers[0],
	        .imageView = src_image_views[0],
	        .imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
	    },
	    {
	        .sampler = src_samplers[1],
	        .imageView = src_image_views[1],
	        .imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
	    },
	};

	VkDescriptorImageInfo distortion_image_info[6] = {
	    {
	        .sampler = distortion_samplers[0],
	        .imageView = distortion_image_views[0],
	        .imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
	    },
	    {
	        .sampler = distortion_samplers[1],
	        .imageView = distortion_image_views[1],
	        .imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
	    },
	    {
	        .sampler = distortion_samplers[2],
	        .imageView = distortion_image_views[2],
	        .imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
	    },
	    {
	        .sampler = distortion_samplers[3],
	        .imageView = distortion_image_views[3],
	        .imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
	    },
	    {
	        .sampler = distortion_samplers[4],
	        .imageView = distortion_image_views[4],
	        .imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
	    },
	    {
	        .sampler = distortion_samplers[5],
	        .imageView = distortion_image_views[5],
	        .imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
	    },
	};

	VkDescriptorImageInfo target_image_info = {
	    .imageView = target_image_view,
	    .imageLayout = VK_IMAGE_LAYOUT_GENERAL,
	};

	VkDescriptorBufferInfo buffer_info = {
	    .buffer = ubo_buffer,
	    .offset = 0,
	    .range = ubo_size,
	};

	VkWriteDescriptorSet write_descriptor_sets[4] = {
	    {
	        .sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET,
	        .dstSet = descriptor_set,
	        .dstBinding = src_binding,
	        .descriptorCount = ARRAY_SIZE(src_image_info),
	        .descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
	        .pImageInfo = src_image_info,
	    },
	    {
	        .sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET,
	        .dstSet = descriptor_set,
	        .dstBinding = distortion_binding,
	        .descriptorCount = ARRAY_SIZE(distortion_image_info),
	        .descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
	        .pImageInfo = distortion_image_info,
	    },
	    {
	        .sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET,
	        .dstSet = descriptor_set,
	        .dstBinding = target_binding,
	        .descriptorCount = 1,
	        .descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,
	        .pImageInfo = &target_image_info,
	    },
	    {
	        .sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET,
	        .dstSet = descriptor_set,
	        .dstBinding = ubo_binding,
	        .descriptorCount = 1,
	        .descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
	        .pBufferInfo = &buffer_info,
	    },
	};

	vk->vkUpdateDescriptorSets(            //
	    vk->device,                        //
	    ARRAY_SIZE(write_descriptor_sets), // descriptorWriteCount
	    write_descriptor_sets,             // pDescriptorWrites
	    0,                                 // descriptorCopyCount
	    NULL);                             // pDescriptorCopies
}

XRT_MAYBE_UNUSED static void
update_compute_discriptor_set_target(struct vk_bundle *vk,
                                     uint32_t target_binding,
                                     VkImageView target_image_view,
                                     uint32_t ubo_binding,
                                     VkBuffer ubo_buffer,
                                     VkDeviceSize ubo_size,
                                     VkDescriptorSet descriptor_set)
{
	VkDescriptorImageInfo target_image_info = {
	    .imageView = target_image_view,
	    .imageLayout = VK_IMAGE_LAYOUT_GENERAL,
	};

	VkDescriptorBufferInfo buffer_info = {
	    .buffer = ubo_buffer,
	    .offset = 0,
	    .range = ubo_size,
	};

	VkWriteDescriptorSet write_descriptor_sets[2] = {
	    {
	        .sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET,
	        .dstSet = descriptor_set,
	        .dstBinding = target_binding,
	        .descriptorCount = 1,
	        .descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,
	        .pImageInfo = &target_image_info,
	    },
	    {
	        .sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET,
	        .dstSet = descriptor_set,
	        .dstBinding = ubo_binding,
	        .descriptorCount = 1,
	        .descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
	        .pBufferInfo = &buffer_info,
	    },
	};

	vk->vkUpdateDescriptorSets(            //
	    vk->device,                        //
	    ARRAY_SIZE(write_descriptor_sets), // descriptorWriteCount
	    write_descriptor_sets,             // pDescriptorWrites
	    0,                                 // descriptorCopyCount
	    NULL);                             // pDescriptorCopies
}


/*
 *
 * 'Exported' functions.
 *
 */

bool
comp_rendering_compute_init(struct comp_compositor *c, struct comp_resources *r, struct comp_rendering_compute *crc)
{
	assert(crc->c == NULL);
	assert(crc->r == NULL);

	struct vk_bundle *vk = &c->vk;
	crc->c = c;
	crc->r = r;

	C(create_command_buffer(vk, &crc->cmd));

	C(create_descriptor_set(              //
	    vk,                               //
	    r->compute.descriptor_pool,       // descriptor_pool
	    r->compute.descriptor_set_layout, // descriptor_set_layout
	    &crc->clear_descriptor_set));     // descriptor_set

	return true;
}

bool
comp_rendering_compute_begin(struct comp_rendering_compute *crc)
{
	struct vk_bundle *vk = &crc->c->vk;

	C(begin_command_buffer(vk, crc->cmd));

	return true;
}

bool
comp_rendering_compute_end(struct comp_rendering_compute *crc)
{
	struct vk_bundle *vk = &crc->c->vk;

	C(end_command_buffer(vk, crc->cmd));

	return true;
}

void
comp_rendering_compute_close(struct comp_rendering_compute *crc)
{
	assert(crc->c != NULL);
	assert(crc->r != NULL);

	struct vk_bundle *vk = &crc->c->vk;

	destroy_command_buffer(vk, crc->cmd);

	// Reclaimed by vkResetDescriptorPool.
	crc->clear_descriptor_set = VK_NULL_HANDLE;

	vk->vkResetDescriptorPool(vk->device, crc->r->compute.descriptor_pool, 0);

	crc->c = NULL;
	crc->r = NULL;
}

void
comp_rendering_compute_projection_timewarp(struct comp_rendering_compute *crc,
                                           VkSampler src_samplers[2],
                                           VkImageView src_image_views[2],
                                           const struct xrt_normalized_rect src_norm_rects[2],
                                           const struct xrt_pose src_poses[2],
                                           const struct xrt_fov src_fovs[2],
                                           const struct xrt_pose new_poses[2],
                                           VkImage target_image,
                                           VkImageView target_image_view,
                                           const struct comp_viewport_data views[2])
{
	assert(crc->c != NULL);
	assert(crc->r != NULL);

	struct vk_bundle *vk = &crc->c->vk;
	struct comp_resources *r = crc->r;


	/*
	 * UBO
	 */

	struct xrt_matrix_4x4 time_warp_matrix[2];
	calc_time_warp_matrix(     //
	    crc,                   //
	    &src_poses[0],         //
	    &src_fovs[0],          //
	    &new_poses[0],         //
	    &time_warp_matrix[0]); //
	calc_time_warp_matrix(     //
	    crc,                   //
	    &src_poses[1],         //
	    &src_fovs[1],          //
	    &new_poses[1],         //
	    &time_warp_matrix[1]); //

	struct comp_ubo_compute_data *data = (struct comp_ubo_compute_data *)r->compute.ubo.mapped;
	data->views[0] = views[0];
	data->views[1] = views[1];
	data->pre_transforms[0] = r->distortion.uv_to_tanangle[0];
	data->pre_transforms[1] = r->distortion.uv_to_tanangle[1];
	data->transforms[0] = time_warp_matrix[0];
	data->transforms[1] = time_warp_matrix[1];
	data->post_transforms[0] = src_norm_rects[0];
	data->post_transforms[1] = src_norm_rects[1];


	/*
	 * Source, target and distortion images.
	 */

	VkImageSubresourceRange subresource_range = {
	    .aspectMask = VK_IMAGE_ASPECT_COLOR_BIT,
	    .baseMipLevel = 0,
	    .levelCount = VK_REMAINING_MIP_LEVELS,
	    .baseArrayLayer = 0,
	    .layerCount = VK_REMAINING_ARRAY_LAYERS,
	};

	vk_set_image_layout(            //
	    vk,                         //
	    crc->cmd,                   //
	    target_image,               //
	    0,                          //
	    VK_ACCESS_SHADER_WRITE_BIT, //
	    VK_IMAGE_LAYOUT_UNDEFINED,  //
	    VK_IMAGE_LAYOUT_GENERAL,    //
	    subresource_range);         //

	VkSampler sampler = r->compute.default_sampler;
	VkSampler distortion_samplers[6] = {
	    sampler, sampler, sampler, sampler, sampler, sampler,
	};

	update_compute_discriptor_set(     //
	    vk,                            //
	    r->compute.src_binding,        //
	    src_samplers,                  //
	    src_image_views,               //
	    r->compute.distortion_binding, //
	    distortion_samplers,           //
	    r->distortion.image_views,     //
	    r->compute.target_binding,     //
	    target_image_view,             //
	    r->compute.ubo_binding,        //
	    r->compute.ubo.buffer,         //
	    VK_WHOLE_SIZE,                 //
	    crc->clear_descriptor_set);    //

	vk->vkCmdBindPipeline(                        //
	    crc->cmd,                                 // commandBuffer
	    VK_PIPELINE_BIND_POINT_COMPUTE,           // pipelineBindPoint
	    r->compute.distortion_timewarp_pipeline); // pipeline

	vk->vkCmdBindDescriptorSets(        //
	    crc->cmd,                       // commandBuffer
	    VK_PIPELINE_BIND_POINT_COMPUTE, // pipelineBindPoint
	    r->compute.pipeline_layout,     // layout
	    0,                              // firstSet
	    1,                              // descriptorSetCount
	    &crc->clear_descriptor_set,     // pDescriptorSets
	    0,                              // dynamicOffsetCount
	    NULL);                          // pDynamicOffsets


	uint32_t w = 0, h = 0;
	calc_dispatch_dims(views, &w, &h);
	assert(w != 0 && h != 0);

	vk->vkCmdDispatch( //
	    crc->cmd,      // commandBuffer
	    w,             // groupCountX
	    h,             // groupCountY
	    2);            // groupCountZ

	VkImageMemoryBarrier memoryBarrier = {
	    .sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER,
	    .srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT,
	    .dstAccessMask = VK_ACCESS_MEMORY_READ_BIT,
	    .oldLayout = VK_IMAGE_LAYOUT_GENERAL,
	    .newLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR,
	    .srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
	    .dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
	    .image = target_image,
	    .subresourceRange = subresource_range,
	};

	vk->vkCmdPipelineBarrier(                 //
	    crc->cmd,                             //
	    VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, //
	    VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,    //
	    0,                                    //
	    0,                                    //
	    NULL,                                 //
	    0,                                    //
	    NULL,                                 //
	    1,                                    //
	    &memoryBarrier);                      //
}

void
comp_rendering_compute_projection(struct comp_rendering_compute *crc,
                                  VkSampler src_samplers[2],
                                  VkImageView src_image_views[2],
                                  const struct xrt_normalized_rect src_norm_rects[2],
                                  VkImage target_image,
                                  VkImageView target_image_view,
                                  const struct comp_viewport_data views[2])
{
	assert(crc->c != NULL);
	assert(crc->r != NULL);

	struct vk_bundle *vk = &crc->c->vk;
	struct comp_resources *r = crc->r;


	/*
	 * UBO
	 */

	struct comp_ubo_compute_data *data = (struct comp_ubo_compute_data *)r->compute.ubo.mapped;
	data->views[0] = views[0];
	data->views[1] = views[1];
	data->post_transforms[0] = src_norm_rects[0];
	data->post_transforms[1] = src_norm_rects[1];


	/*
	 * Source, target and distortion images.
	 */

	VkImageSubresourceRange subresource_range = {
	    .aspectMask = VK_IMAGE_ASPECT_COLOR_BIT,
	    .baseMipLevel = 0,
	    .levelCount = VK_REMAINING_MIP_LEVELS,
	    .baseArrayLayer = 0,
	    .layerCount = VK_REMAINING_ARRAY_LAYERS,
	};

	vk_set_image_layout(            //
	    vk,                         //
	    crc->cmd,                   //
	    target_image,               //
	    0,                          //
	    VK_ACCESS_SHADER_WRITE_BIT, //
	    VK_IMAGE_LAYOUT_UNDEFINED,  //
	    VK_IMAGE_LAYOUT_GENERAL,    //
	    subresource_range);         //

	VkSampler sampler = r->compute.default_sampler;
	VkSampler distortion_samplers[6] = {
	    sampler, sampler, sampler, sampler, sampler, sampler,
	};

	update_compute_discriptor_set(     //
	    vk,                            //
	    r->compute.src_binding,        //
	    src_samplers,                  //
	    src_image_views,               //
	    r->compute.distortion_binding, //
	    distortion_samplers,           //
	    r->distortion.image_views,     //
	    r->compute.target_binding,     //
	    target_image_view,             //
	    r->compute.ubo_binding,        //
	    r->compute.ubo.buffer,         //
	    VK_WHOLE_SIZE,                 //
	    crc->clear_descriptor_set);    //

	vk->vkCmdBindPipeline(               //
	    crc->cmd,                        // commandBuffer
	    VK_PIPELINE_BIND_POINT_COMPUTE,  // pipelineBindPoint
	    r->compute.distortion_pipeline); // pipeline

	vk->vkCmdBindDescriptorSets(        //
	    crc->cmd,                       // commandBuffer
	    VK_PIPELINE_BIND_POINT_COMPUTE, // pipelineBindPoint
	    r->compute.pipeline_layout,     // layout
	    0,                              // firstSet
	    1,                              // descriptorSetCount
	    &crc->clear_descriptor_set,     // pDescriptorSets
	    0,                              // dynamicOffsetCount
	    NULL);                          // pDynamicOffsets


	uint32_t w = 0, h = 0;
	calc_dispatch_dims(views, &w, &h);
	assert(w != 0 && h != 0);

	vk->vkCmdDispatch( //
	    crc->cmd,      // commandBuffer
	    w,             // groupCountX
	    h,             // groupCountY
	    2);            // groupCountZ

	VkImageMemoryBarrier memoryBarrier = {
	    .sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER,
	    .srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT,
	    .dstAccessMask = VK_ACCESS_MEMORY_READ_BIT,
	    .oldLayout = VK_IMAGE_LAYOUT_GENERAL,
	    .newLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR,
	    .srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
	    .dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
	    .image = target_image,
	    .subresourceRange = subresource_range,
	};

	vk->vkCmdPipelineBarrier(                 //
	    crc->cmd,                             //
	    VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, //
	    VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,    //
	    0,                                    //
	    0,                                    //
	    NULL,                                 //
	    0,                                    //
	    NULL,                                 //
	    1,                                    //
	    &memoryBarrier);                      //
}

void
comp_rendering_compute_clear(struct comp_rendering_compute *crc,       //
                             VkImage target_image,                     //
                             VkImageView target_image_view,            //
                             const struct comp_viewport_data views[2]) //
{
	assert(crc->c != NULL);
	assert(crc->r != NULL);

	struct vk_bundle *vk = &crc->c->vk;
	struct comp_resources *r = crc->r;


	/*
	 * UBO
	 */

	// Calculate transforms.
	struct xrt_matrix_4x4 transforms[2];
	for (uint32_t i = 0; i < 2; i++) {
		math_matrix_4x4_identity(&transforms[i]);
	}

	struct comp_ubo_compute_data *data = (struct comp_ubo_compute_data *)r->compute.ubo.mapped;
	data->views[0] = views[0];
	data->views[1] = views[1];


	/*
	 * Source, target and distortion images.
	 */

	VkImageSubresourceRange subresource_range = {
	    .aspectMask = VK_IMAGE_ASPECT_COLOR_BIT,
	    .baseMipLevel = 0,
	    .levelCount = VK_REMAINING_MIP_LEVELS,
	    .baseArrayLayer = 0,
	    .layerCount = VK_REMAINING_ARRAY_LAYERS,
	};

	vk_set_image_layout(            //
	    vk,                         //
	    crc->cmd,                   //
	    target_image,               //
	    0,                          //
	    VK_ACCESS_SHADER_WRITE_BIT, //
	    VK_IMAGE_LAYOUT_UNDEFINED,  //
	    VK_IMAGE_LAYOUT_GENERAL,    //
	    subresource_range);         //

	VkSampler sampler = r->compute.default_sampler;
	VkSampler src_samplers[2] = {sampler, sampler};
	VkImageView src_image_views[2] = {VK_NULL_HANDLE, VK_NULL_HANDLE};
	VkSampler distortion_samplers[6] = {sampler, sampler, sampler, sampler, sampler, sampler};
	VkImageView distortion_image_views[6] = {VK_NULL_HANDLE, VK_NULL_HANDLE, VK_NULL_HANDLE,
	                                         VK_NULL_HANDLE, VK_NULL_HANDLE, VK_NULL_HANDLE};

	update_compute_discriptor_set(     //
	    vk,                            //
	    r->compute.src_binding,        //
	    src_samplers,                  //
	    src_image_views,               //
	    r->compute.distortion_binding, //
	    distortion_samplers,           //
	    distortion_image_views,        //
	    r->compute.target_binding,     //
	    target_image_view,             //
	    r->compute.ubo_binding,        //
	    r->compute.ubo.buffer,         //
	    VK_WHOLE_SIZE,                 //
	    crc->clear_descriptor_set);    //

	vk->vkCmdBindPipeline(              //
	    crc->cmd,                       // commandBuffer
	    VK_PIPELINE_BIND_POINT_COMPUTE, // pipelineBindPoint
	    r->compute.clear_pipeline);     // pipeline

	vk->vkCmdBindDescriptorSets(        //
	    crc->cmd,                       // commandBuffer
	    VK_PIPELINE_BIND_POINT_COMPUTE, // pipelineBindPoint
	    r->compute.pipeline_layout,     // layout
	    0,                              // firstSet
	    1,                              // descriptorSetCount
	    &crc->clear_descriptor_set,     // pDescriptorSets
	    0,                              // dynamicOffsetCount
	    NULL);                          // pDynamicOffsets


	uint32_t w = 0, h = 0;
	calc_dispatch_dims(views, &w, &h);
	assert(w != 0 && h != 0);

	vk->vkCmdDispatch( //
	    crc->cmd,      // commandBuffer
	    w,             // groupCountX
	    h,             // groupCountY
	    2);            // groupCountZ

	VkImageMemoryBarrier memoryBarrier = {
	    .sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER,
	    .srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT,
	    .dstAccessMask = VK_ACCESS_MEMORY_READ_BIT,
	    .oldLayout = VK_IMAGE_LAYOUT_GENERAL,
	    .newLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR,
	    .srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
	    .dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
	    .image = target_image,
	    .subresourceRange = subresource_range,
	};

	vk->vkCmdPipelineBarrier(                 //
	    crc->cmd,                             //
	    VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, //
	    VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,    //
	    0,                                    //
	    0,                                    //
	    NULL,                                 //
	    0,                                    //
	    NULL,                                 //
	    1,                                    //
	    &memoryBarrier);                      //
}
