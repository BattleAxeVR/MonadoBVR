// Copyright 2019-2020, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Shared resources for rendering.
 * @author Lubosz Sarnecki <lubosz.sarnecki@collabora.com>
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup comp_main
 */


#include "main/comp_compositor.h"
#include "render/comp_render.h"
#include "math/m_vec2.h"

#include <stdio.h>


#define C(c)                                                                                                           \
	do {                                                                                                           \
		VkResult ret = c;                                                                                      \
		if (ret != VK_SUCCESS) {                                                                               \
			return false;                                                                                  \
		}                                                                                                      \
	} while (false)

#define D(TYPE, thing)                                                                                                 \
	if (thing != VK_NULL_HANDLE) {                                                                                 \
		vk->vkDestroy##TYPE(vk->device, thing, NULL);                                                          \
		thing = VK_NULL_HANDLE;                                                                                \
	}

#define DF(TYPE, thing)                                                                                                \
	if (thing != VK_NULL_HANDLE) {                                                                                 \
		vk->vkFree##TYPE(vk->device, thing, NULL);                                                             \
		thing = VK_NULL_HANDLE;                                                                                \
	}


static VkResult
create_pipeline_cache(struct vk_bundle *vk, VkPipelineCache *out_pipeline_cache)
{
	VkResult ret;

	VkPipelineCacheCreateInfo pipeline_cache_info = {
	    .sType = VK_STRUCTURE_TYPE_PIPELINE_CACHE_CREATE_INFO,
	};

	VkPipelineCache pipeline_cache;
	ret = vk->vkCreatePipelineCache(vk->device,           //
	                                &pipeline_cache_info, //
	                                NULL,                 //
	                                &pipeline_cache);     //
	if (ret != VK_SUCCESS) {
		VK_ERROR(vk, "vkCreatePipelineCache failed: %s", vk_result_string(ret));
		return ret;
	}

	*out_pipeline_cache = pipeline_cache;

	return VK_SUCCESS;
}

static VkResult
create_pipeline_layout(struct vk_bundle *vk,
                       VkDescriptorSetLayout descriptor_set_layout,
                       VkPipelineLayout *out_pipeline_layout)
{
	VkResult ret;

	VkPipelineLayoutCreateInfo pipeline_layout_info = {
	    .sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO,
	    .setLayoutCount = 1,
	    .pSetLayouts = &descriptor_set_layout,
	};

	VkPipelineLayout pipeline_layout = VK_NULL_HANDLE;
	ret = vk->vkCreatePipelineLayout(vk->device,            //
	                                 &pipeline_layout_info, //
	                                 NULL,                  //
	                                 &pipeline_layout);     //
	if (ret != VK_SUCCESS) {
		VK_ERROR(vk, "vkCreatePipelineLayout failed: %s", vk_result_string(ret));
		return ret;
	}

	*out_pipeline_layout = pipeline_layout;

	return VK_SUCCESS;
}

static VkResult
create_descriptor_pool(struct vk_bundle *vk,
                       uint32_t num_uniform_per_desc,
                       uint32_t num_sampler_per_desc,
                       uint32_t num_storage_per_desc,
                       uint32_t num_descs,
                       bool freeable,
                       VkDescriptorPool *out_descriptor_pool)
{
	VkResult ret;


	uint32_t count = 0;
	VkDescriptorPoolSize pool_sizes[3] = {0};

	if (num_uniform_per_desc > 0) {
		pool_sizes[count].type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
		pool_sizes[count].descriptorCount = num_uniform_per_desc * num_descs;
		count++;
	}

	if (num_sampler_per_desc > 0) {
		pool_sizes[count].type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
		pool_sizes[count].descriptorCount = num_sampler_per_desc * num_descs;
		count++;
	}

	if (num_storage_per_desc > 0) {
		pool_sizes[count].type = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
		pool_sizes[count].descriptorCount = num_storage_per_desc * num_descs;
		count++;
	}

	assert(count > 0 && count <= ARRAY_SIZE(pool_sizes));

	VkDescriptorPoolCreateFlags flags = 0;

	if (freeable) {
		flags |= VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;
	}

	VkDescriptorPoolCreateInfo descriptor_pool_info = {
	    .sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO,
	    .flags = flags,
	    .maxSets = num_descs,
	    .poolSizeCount = count,
	    .pPoolSizes = pool_sizes,
	};

	VkDescriptorPool descriptor_pool = VK_NULL_HANDLE;
	ret = vk->vkCreateDescriptorPool(vk->device,            //
	                                 &descriptor_pool_info, //
	                                 NULL,                  //
	                                 &descriptor_pool);     //
	if (ret != VK_SUCCESS) {
		VK_ERROR(vk, "vkCreateRenderPass failed: %s", vk_result_string(ret));
		return ret;
	}

	*out_descriptor_pool = descriptor_pool;

	return VK_SUCCESS;
}


/*
 *
 * Mesh
 *
 */

static VkResult
create_mesh_descriptor_set_layout(struct vk_bundle *vk,
                                  uint32_t src_binding,
                                  uint32_t ubo_binding,
                                  VkDescriptorSetLayout *out_descriptor_set_layout)
{
	VkResult ret;

	VkDescriptorSetLayoutBinding set_layout_bindings[2] = {
	    {
	        .binding = src_binding,
	        .descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
	        .descriptorCount = 1,
	        .stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT,
	    },
	    {
	        .binding = ubo_binding,
	        .descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
	        .descriptorCount = 1,
	        .stageFlags = VK_SHADER_STAGE_VERTEX_BIT,
	    },
	};

	VkDescriptorSetLayoutCreateInfo set_layout_info = {
	    .sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO,
	    .bindingCount = ARRAY_SIZE(set_layout_bindings),
	    .pBindings = set_layout_bindings,
	};

	VkDescriptorSetLayout descriptor_set_layout = VK_NULL_HANDLE;
	ret = vk->vkCreateDescriptorSetLayout(vk->device,              //
	                                      &set_layout_info,        //
	                                      NULL,                    //
	                                      &descriptor_set_layout); //
	if (ret != VK_SUCCESS) {
		VK_ERROR(vk, "vkCreateDescriptorSetLayout failed: %s", vk_result_string(ret));
		return ret;
	}

	*out_descriptor_set_layout = descriptor_set_layout;

	return VK_SUCCESS;
}


static bool
init_mesh_vertex_buffers(struct vk_bundle *vk,
                         struct comp_buffer *vbo,
                         struct comp_buffer *ibo,
                         uint32_t num_vertices,
                         uint32_t stride,
                         void *vertices,
                         uint32_t num_indices,
                         void *indices)
{
	// Using the same flags for all vbos.
	VkBufferUsageFlags vbo_usage_flags = VK_BUFFER_USAGE_VERTEX_BUFFER_BIT;
	VkBufferUsageFlags ibo_usage_flags = VK_BUFFER_USAGE_INDEX_BUFFER_BIT;
	VkMemoryPropertyFlags memory_property_flags =
	    VK_MEMORY_PROPERTY_HOST_COHERENT_BIT | VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT;

	// Distortion vbo and ibo sizes.
	VkDeviceSize vbo_size = stride * num_vertices;
	VkDeviceSize ibo_size = sizeof(int) * num_indices;


	// Don't create vbo if size is zero.
	if (vbo_size == 0) {
		return true;
	}

	C(comp_buffer_init(vk,                    // vk_bundle
	                   vbo,                   // buffer
	                   vbo_usage_flags,       // usage_flags
	                   memory_property_flags, // memory_property_flags
	                   vbo_size));            // size

	C(comp_buffer_write(vk,         // vk_bundle
	                    vbo,        // buffer
	                    vertices,   // data
	                    vbo_size)); // size


	// Don't create index buffer if size is zero.
	if (ibo_size == 0) {
		return true;
	}

	C(comp_buffer_init(vk,                    // vk_bundle
	                   ibo,                   // buffer
	                   ibo_usage_flags,       // usage_flags
	                   memory_property_flags, // memory_property_flags
	                   ibo_size));            // size

	C(comp_buffer_write(vk,         // vk_bundle
	                    ibo,        // buffer
	                    indices,    // data
	                    ibo_size)); // size

	return true;
}


/*
 *
 * Compute
 *
 */

static VkResult
create_compute_descriptor_set_layout(struct vk_bundle *vk,
                                     uint32_t src_binding,
                                     uint32_t distortion_binding,
                                     uint32_t target_binding,
                                     uint32_t ubo_binding,
                                     VkDescriptorSetLayout *out_descriptor_set_layout)
{
	VkResult ret;

	VkDescriptorSetLayoutBinding set_layout_bindings[4] = {
	    {
	        .binding = src_binding,
	        .descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
	        .descriptorCount = 2,
	        .stageFlags = VK_SHADER_STAGE_COMPUTE_BIT,
	    },
	    {
	        .binding = distortion_binding,
	        .descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
	        .descriptorCount = 6,
	        .stageFlags = VK_SHADER_STAGE_COMPUTE_BIT,
	    },
	    {
	        .binding = target_binding,
	        .descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,
	        .descriptorCount = 1,
	        .stageFlags = VK_SHADER_STAGE_COMPUTE_BIT,
	    },
	    {
	        .binding = ubo_binding,
	        .descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
	        .descriptorCount = 1,
	        .stageFlags = VK_SHADER_STAGE_COMPUTE_BIT,
	    },
	};

	VkDescriptorSetLayoutCreateInfo set_layout_info = {
	    .sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO,
	    .bindingCount = ARRAY_SIZE(set_layout_bindings),
	    .pBindings = set_layout_bindings,
	};

	VkDescriptorSetLayout descriptor_set_layout = VK_NULL_HANDLE;
	ret = vk->vkCreateDescriptorSetLayout( //
	    vk->device,                        //
	    &set_layout_info,                  //
	    NULL,                              //
	    &descriptor_set_layout);           //
	if (ret != VK_SUCCESS) {
		VK_ERROR(vk, "vkCreateDescriptorSetLayout failed: %s", vk_result_string(ret));
		return ret;
	}

	*out_descriptor_set_layout = descriptor_set_layout;

	return VK_SUCCESS;
}

static VkResult
create_compute_pipeline(struct vk_bundle *vk,
                        VkPipelineCache pipeline_cache,
                        VkShaderModule shader,
                        VkPipelineLayout pipeline_layout,
                        VkPipeline *out_compute_pipeline)
{
	VkResult ret;

	VkPipelineShaderStageCreateInfo shader_stage_info = {
	    .sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO,
	    .pNext = NULL,
	    .stage = VK_SHADER_STAGE_COMPUTE_BIT,
	    .module = shader,
	    .pName = "main",
	};

	VkComputePipelineCreateInfo pipeline_info = {
	    .sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO,
	    .pNext = NULL,
	    .flags = 0,
	    .stage = shader_stage_info,
	    .layout = pipeline_layout,
	};

	VkPipeline pipeline = VK_NULL_HANDLE;
	ret = vk->vkCreateComputePipelines( //
	    vk->device,                     //
	    pipeline_cache,                 //
	    1,                              //
	    &pipeline_info,                 //
	    NULL,                           //
	    &pipeline);                     //
	if (ret != VK_SUCCESS) {
		VK_DEBUG(vk, "vkCreateComputePipelines failed: %s", vk_result_string(ret));
		return ret;
	}

	*out_compute_pipeline = pipeline;

	return VK_SUCCESS;
}

static VkResult
create_distortion_image_and_view(struct vk_bundle *vk,
                                 VkExtent2D extent,
                                 VkDeviceMemory *out_device_memory,
                                 VkImage *out_image,
                                 VkImageView *out_image_view)
{
	VkFormat format = VK_FORMAT_R32G32_SFLOAT;
	VkImage image = VK_NULL_HANDLE;
	VkDeviceMemory device_memory = VK_NULL_HANDLE;
	VkImageView image_view = VK_NULL_HANDLE;

	C(vk_create_image_simple(                                         //
	    vk,                                                           // vk_bundle
	    extent,                                                       // extent
	    format,                                                       // format
	    VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT, // usage
	    &device_memory,                                               // out_device_memory
	    &image));                                                     // out_image

	VkImageSubresourceRange subresource_range = {
	    .aspectMask = VK_IMAGE_ASPECT_COLOR_BIT,
	    .baseMipLevel = 0,
	    .levelCount = VK_REMAINING_MIP_LEVELS,
	    .baseArrayLayer = 0,
	    .layerCount = VK_REMAINING_ARRAY_LAYERS,
	};

	C(vk_create_view(      //
	    vk,                // vk_bundle
	    image,             // image
	    format,            // format
	    subresource_range, // subresource_range
	    &image_view));     // out_image_view

	*out_device_memory = device_memory;
	*out_image = image;
	*out_image_view = image_view;

	return VK_SUCCESS;
}

static VkResult
queue_upload_for_first_level_and_layer(
    struct vk_bundle *vk, VkCommandBuffer cmd, VkBuffer src, VkImage dst, VkExtent2D extent)
{
	VkImageSubresourceRange subresource_range = {
	    .aspectMask = VK_IMAGE_ASPECT_COLOR_BIT,
	    .baseMipLevel = 0,
	    .levelCount = VK_REMAINING_MIP_LEVELS,
	    .baseArrayLayer = 0,
	    .layerCount = VK_REMAINING_ARRAY_LAYERS,
	};

	C(vk_set_image_layout(                    //
	    vk,                                   //
	    cmd,                                  //
	    dst,                                  //
	    0,                                    //
	    VK_ACCESS_TRANSFER_WRITE_BIT,         //
	    VK_IMAGE_LAYOUT_UNDEFINED,            //
	    VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, //
	    subresource_range));                  //

	VkImageSubresourceLayers subresource_layers = {
	    .aspectMask = VK_IMAGE_ASPECT_COLOR_BIT,
	    .mipLevel = 0,
	    .baseArrayLayer = 0,
	    .layerCount = 1,
	};

	VkBufferImageCopy region = {
	    .bufferOffset = 0,
	    .bufferRowLength = 0,
	    .bufferImageHeight = 0,
	    .imageSubresource = subresource_layers,
	    .imageOffset = {0, 0, 0},
	    .imageExtent = {extent.width, extent.height, 1},
	};

	vk->vkCmdCopyBufferToImage(               //
	    cmd,                                  // commandBuffer
	    src,                                  // srcBuffer
	    dst,                                  // dstImage
	    VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, // dstImageLayout
	    1,                                    // regionCount
	    &region);                             // pRegions

	C(vk_set_image_layout(                        //
	    vk,                                       //
	    cmd,                                      //
	    dst,                                      //
	    0,                                        //
	    VK_ACCESS_SHADER_READ_BIT,                //
	    VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,     //
	    VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, //
	    subresource_range));                      //

	return VK_SUCCESS;
}

static VkResult
create_and_queue_upload(struct vk_bundle *vk,
                        VkCommandBuffer cmd,
                        VkBuffer src_buffer,
                        VkDeviceMemory *out_image_device_memory,
                        VkImage *out_image,
                        VkImageView *out_image_view)
{
	VkExtent2D extent = {COMP_DISTORTION_IMAGE_DIMENSIONS, COMP_DISTORTION_IMAGE_DIMENSIONS};

	VkDeviceMemory device_memory = VK_NULL_HANDLE;
	VkImage image = VK_NULL_HANDLE;
	VkImageView image_view = VK_NULL_HANDLE;

	C(create_distortion_image_and_view( //
	    vk,                             // vk_bundle
	    extent,                         // extent
	    &device_memory,                 // out_device_memory
	    &image,                         // out_image
	    &image_view));                  // out_image_view

	C(queue_upload_for_first_level_and_layer( //
	    vk,                                   // vk_bundle
	    cmd,                                  // cmd
	    src_buffer,                           // src
	    image,                                // dst
	    extent));                             // extent

	*out_image_device_memory = device_memory;
	*out_image = image;
	*out_image_view = image_view;

	return VK_SUCCESS;
}

/*!
 * Helper struct to make code easier to read.
 */
struct texture
{
	struct xrt_vec2 pixels[COMP_DISTORTION_IMAGE_DIMENSIONS][COMP_DISTORTION_IMAGE_DIMENSIONS];
};

struct tan_angles_transforms
{
	struct xrt_vec2 offset;
	struct xrt_vec2 scale;
};

static void
calc_uv_to_tanangle(struct xrt_device *xdev, uint32_t view, struct xrt_normalized_rect *out_rect)
{
	const struct xrt_fov fov = xdev->hmd->views[view].fov;
	const double tan_left = tan(fov.angle_left);
	const double tan_right = tan(fov.angle_right);

	const double tan_down = tan(fov.angle_down);
	const double tan_up = tan(fov.angle_up);

	const double tan_width = tan_right - tan_left;
	const double tan_height = tan_up - tan_down;

	const double tan_offset_x = (tan_right + tan_left) - tan_width / 2;
	const double tan_offset_y = (tan_up + tan_down) - tan_height / 2;

	struct xrt_normalized_rect transform = {
	    .x = tan_offset_x,
	    .y = tan_offset_y,
	    .w = tan_width,
	    .h = tan_height,
	};

	*out_rect = transform;
}

static XRT_MAYBE_UNUSED VkResult
create_and_file_in_distortion_buffer_for_view(struct vk_bundle *vk,
                                              struct xrt_device *xdev,
                                              struct comp_buffer *r_buffer,
                                              struct comp_buffer *g_buffer,
                                              struct comp_buffer *b_buffer,
                                              uint32_t view)
{
	VkBufferUsageFlags usage_flags = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
	VkMemoryPropertyFlags properties = VK_MEMORY_PROPERTY_HOST_COHERENT_BIT | VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT;


	VkDeviceSize size = sizeof(struct texture);

	C(comp_buffer_init(vk, r_buffer, usage_flags, properties, size));
	C(comp_buffer_init(vk, g_buffer, usage_flags, properties, size));
	C(comp_buffer_init(vk, b_buffer, usage_flags, properties, size));

	C(comp_buffer_map(vk, r_buffer));
	C(comp_buffer_map(vk, g_buffer));
	C(comp_buffer_map(vk, b_buffer));

	struct texture *r = r_buffer->mapped;
	struct texture *g = g_buffer->mapped;
	struct texture *b = b_buffer->mapped;

	for (int row = 0; row < COMP_DISTORTION_IMAGE_DIMENSIONS; row++) {
		// This goes from 0 to 1.0 inclusive.
		float v = (double)row / (double)COMP_DISTORTION_IMAGE_DIMENSIONS;

		for (int col = 0; col < COMP_DISTORTION_IMAGE_DIMENSIONS; col++) {
			// This goes from 0 to 1.0 inclusive.
			float u = (double)col / (double)COMP_DISTORTION_IMAGE_DIMENSIONS;

			struct xrt_uv_triplet result;
			xrt_device_compute_distortion(xdev, view, u, v, &result);


			r->pixels[row][col] = result.r;
			g->pixels[row][col] = result.g;
			b->pixels[row][col] = result.b;
		}
	}

	comp_buffer_unmap(vk, r_buffer);
	comp_buffer_unmap(vk, g_buffer);
	comp_buffer_unmap(vk, b_buffer);

	return VK_SUCCESS;
}

/*
 *
 * 'Exported' renderer functions.
 *
 */

bool
comp_resources_init(struct comp_compositor *c, struct comp_resources *r)
{
	struct vk_bundle *vk = &c->vk;
	struct xrt_device *xdev = c->xdev;

	/*
	 * Constants
	 */

	r->mesh.src_binding = 0;
	r->mesh.ubo_binding = 1;
	struct xrt_hmd_parts *parts = xdev->hmd;
	r->mesh.num_vertices = parts->distortion.mesh.num_vertices;
	r->mesh.stride = parts->distortion.mesh.stride;
	r->mesh.num_indices[0] = parts->distortion.mesh.num_indices[0];
	r->mesh.num_indices[1] = parts->distortion.mesh.num_indices[1];
	r->mesh.total_num_indices = parts->distortion.mesh.total_num_indices;
	r->mesh.offset_indices[0] = parts->distortion.mesh.offset_indices[0];
	r->mesh.offset_indices[1] = parts->distortion.mesh.offset_indices[1];

	r->compute.src_binding = 0;
	r->compute.distortion_binding = 1;
	r->compute.target_binding = 2;
	r->compute.ubo_binding = 3;


	/*
	 * Shared
	 */

	C(create_pipeline_cache(vk, &r->pipeline_cache));


	/*
	 * Mesh static.
	 */

	C(create_descriptor_pool(vk,                         // vk_bundle
	                         1,                          // num_uniform_per_desc
	                         1,                          // num_sampler_per_desc
	                         0,                          // num_storage_per_desc
	                         16 * 2,                     // num_descs
	                         true,                       // freeable
	                         &r->mesh_descriptor_pool)); // out_descriptor_pool

	C(create_mesh_descriptor_set_layout(vk,                               // vk_bundle
	                                    r->mesh.src_binding,              // src_binding
	                                    r->mesh.ubo_binding,              // ubo_binding
	                                    &r->mesh.descriptor_set_layout)); // out_mesh_descriptor_set_layout

	C(create_pipeline_layout(vk,                            // vk_bundle
	                         r->mesh.descriptor_set_layout, // descriptor_set_layout
	                         &r->mesh.pipeline_layout));    // out_pipeline_layout

	if (!init_mesh_vertex_buffers(vk,                                //
	                              &r->mesh.vbo,                      //
	                              &r->mesh.ibo,                      //
	                              r->mesh.num_vertices,              //
	                              r->mesh.stride,                    //
	                              parts->distortion.mesh.vertices,   //
	                              r->mesh.total_num_indices,         //
	                              parts->distortion.mesh.indices)) { //
		return false;
	}


	/*
	 * Compute static.
	 */

	C(vk_create_sampler(                       //
	    vk,                                    // vk_bundle
	    VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE, // clamp_mode
	    &r->compute.default_sampler));         // out_sampler

	C(create_descriptor_pool(          //
	    vk,                            // vk_bundle
	    1,                             // num_uniform_per_desc
	    8,                             // num_sampler_per_desc
	    1,                             // num_storage_per_desc
	    1,                             // num_descs
	    false,                         // freeable
	    &r->compute.descriptor_pool)); // out_descriptor_pool

	C(create_compute_descriptor_set_layout(  //
	    vk,                                  // vk_bundle
	    r->compute.src_binding,              // src_binding,
	    r->compute.distortion_binding,       // distortion_binding,
	    r->compute.target_binding,           // target_binding,
	    r->compute.ubo_binding,              // ubo_binding,
	    &r->compute.descriptor_set_layout)); // out_descriptor_set_layout

	C(create_pipeline_layout(             //
	    vk,                               // vk_bundle
	    r->compute.descriptor_set_layout, // descriptor_set_layout
	    &r->compute.pipeline_layout));    // out_pipeline_layout

	C(create_compute_pipeline(        //
	    vk,                           // vk_bundle
	    r->pipeline_cache,            // pipeline_cache
	    c->shaders.clear_comp,        // shader
	    r->compute.pipeline_layout,   // pipeline_layout
	    &r->compute.clear_pipeline)); // out_compute_pipeline

	C(create_compute_pipeline(             //
	    vk,                                // vk_bundle
	    r->pipeline_cache,                 // pipeline_cache
	    c->shaders.distortion_comp,        // shader
	    r->compute.pipeline_layout,        // pipeline_layout
	    &r->compute.distortion_pipeline)); // out_compute_pipeline

	C(create_compute_pipeline(                      //
	    vk,                                         // vk_bundle
	    r->pipeline_cache,                          // pipeline_cache
	    c->shaders.distortion_timewarp_comp,        // shader
	    r->compute.pipeline_layout,                 // pipeline_layout
	    &r->compute.distortion_timewarp_pipeline)); // out_compute_pipeline


	VkBufferUsageFlags ubo_usage_flags = VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;
	VkMemoryPropertyFlags memory_property_flags = VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT |
	                                              VK_MEMORY_PROPERTY_HOST_COHERENT_BIT |
	                                              VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT;
	size_t ubo_size = sizeof(struct comp_ubo_compute_data);

	C(comp_buffer_init(        //
	    vk,                    // vk_bundle
	    &r->compute.ubo,       // buffer
	    ubo_usage_flags,       // usage_flags
	    memory_property_flags, // memory_property_flags
	    ubo_size));            // size
	C(comp_buffer_map(         //
	    vk,                    // vk_bundle
	    &r->compute.ubo));     // buffer


	struct comp_buffer buffers[COMP_DISTORTION_NUM_IMAGES];

	calc_uv_to_tanangle(c->xdev, 0, &r->distortion.uv_to_tanangle[0]);
	calc_uv_to_tanangle(c->xdev, 1, &r->distortion.uv_to_tanangle[1]);

	create_and_file_in_distortion_buffer_for_view(vk, c->xdev, &buffers[0], &buffers[2], &buffers[4], 0);
	create_and_file_in_distortion_buffer_for_view(vk, c->xdev, &buffers[1], &buffers[3], &buffers[5], 1);

	VkCommandBuffer upload_buffer = VK_NULL_HANDLE;
	C(vk_init_cmd_buffer(vk, &upload_buffer));

	for (uint32_t i = 0; i < COMP_DISTORTION_NUM_IMAGES; i++) {
		C(create_and_queue_upload(             //
		    vk,                                // vk_bundle
		    upload_buffer,                     // cmd
		    buffers[i].buffer,                 // src_buffer
		    &r->distortion.device_memories[i], // out_image_device_memory
		    &r->distortion.images[i],          // out_image
		    &r->distortion.image_views[i]));   // out_image_view
	}

	C(vk_submit_cmd_buffer(vk, upload_buffer));

	os_mutex_lock(&vk->queue_mutex);
	vk->vkDeviceWaitIdle(vk->device);
	os_mutex_unlock(&vk->queue_mutex);

	for (uint32_t i = 0; i < ARRAY_SIZE(buffers); i++) {
		comp_buffer_close(vk, &buffers[i]);
	}


	/*
	 * Done
	 */

	U_LOG_I("New renderer initialized!");

	return true;
}

void
comp_resources_close(struct comp_compositor *c, struct comp_resources *r)
{
	struct vk_bundle *vk = &c->vk;

	D(DescriptorSetLayout, r->mesh.descriptor_set_layout);
	D(PipelineLayout, r->mesh.pipeline_layout);
	D(PipelineCache, r->pipeline_cache);
	D(DescriptorPool, r->mesh_descriptor_pool);
	comp_buffer_close(vk, &r->mesh.vbo);
	comp_buffer_close(vk, &r->mesh.ibo);

	D(DescriptorPool, r->compute.descriptor_pool);
	D(DescriptorSetLayout, r->compute.descriptor_set_layout);
	D(Pipeline, r->compute.clear_pipeline);
	D(Pipeline, r->compute.distortion_pipeline);
	D(Pipeline, r->compute.distortion_timewarp_pipeline);
	D(PipelineLayout, r->compute.pipeline_layout);
	D(Sampler, r->compute.default_sampler);
	for (uint32_t i = 0; i < ARRAY_SIZE(r->distortion.image_views); i++) {
		D(ImageView, r->distortion.image_views[i]);
	}
	for (uint32_t i = 0; i < ARRAY_SIZE(r->distortion.images); i++) {
		D(Image, r->distortion.images[i]);
	}
	for (uint32_t i = 0; i < ARRAY_SIZE(r->distortion.images); i++) {
		DF(Memory, r->distortion.device_memories[i]);
	}
	comp_buffer_close(vk, &r->compute.ubo);
}
