#include "particles.h"

#include "compute/vk_compute.h"
#include "core/assets/manager.h"
#include <core/assets/ktx_loader.h>
#include "core/context.h"
#include "core/descriptor/descriptors.h"
#include "core/descriptor/manager.h"
#include "core/device/device.h"
#include "core/device/resource.h"
#include "core/device/swapchain.h"
#include "core/frame/resources.h"
#include "core/pipeline/manager.h"
#include "core/pipeline/sampler.h"
#include "render/graph/graph.h"
#include "render/pipelines.h"
#include "scene/vk_scene.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace
{
    struct ParticleGPU
    {
        glm::vec4 pos_age;
        glm::vec4 vel_life;
        glm::vec4 color;
        glm::vec4 misc;
    };
    static_assert(sizeof(ParticleGPU) == 64);

    constexpr uint32_t k_local_size_x = 256;
    constexpr uint32_t k_block_size = 256;
    constexpr uint32_t k_max_blocks = ParticlePass::k_max_particles / k_block_size;
    static_assert(k_max_blocks == 512);

    struct ParticleUpdatePushConstants
    {
        glm::uvec4 header;              // x=base, y=count, z=reset, w=unused
        glm::vec4 sim;                  // x=dt, y=time, z=drag, w=gravity
        glm::vec4 origin_delta;         // xyz origin delta local
        glm::vec4 emitter_pos_radius;   // xyz emitter pos local, w=spawn radius
        glm::vec4 emitter_dir_cone;     // xyz emitter dir local, w=cone angle (radians)
        glm::vec4 ranges;               // x=minSpeed, y=maxSpeed, z=minLife, w=maxLife
        glm::vec4 size_range;           // x=minSize, y=maxSize, z/w unused
        glm::vec4 color;                // rgba
    };
    static_assert(sizeof(ParticleUpdatePushConstants) == 128);

    struct ParticleSortPushConstants
    {
        glm::uvec4 header; // x=base, y=count, z/w unused
        glm::mat4 view;
    };
    static_assert(sizeof(ParticleSortPushConstants) == 80);

    struct ParticleBuildIndicesPushConstants
    {
        glm::uvec4 header; // x=base, y=count, z=flags (bit0=identity), w unused
    };
    static_assert(sizeof(ParticleBuildIndicesPushConstants) == 16);

    struct ParticleRenderPushConstants
    {
        glm::vec4 screen;   // x=invW, y=invH, z=softDepthDistance, w=timeSeconds
        glm::vec4 flipbook; // x=cols, y=rows, z=fps, w=intensity
        glm::vec4 noise;    // x=scale, y=strength, z=scrollX, w=scrollY
    };
    static_assert(sizeof(ParticleRenderPushConstants) == 48);

    static void cmd_compute_memory_barrier(VkCommandBuffer cmd, VkAccessFlags2 srcAccess, VkAccessFlags2 dstAccess)
    {
        VkMemoryBarrier2 mb{};
        mb.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER_2;
        mb.srcStageMask = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT;
        mb.srcAccessMask = srcAccess;
        mb.dstStageMask = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT;
        mb.dstAccessMask = dstAccess;

        VkDependencyInfo dep{};
        dep.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
        dep.memoryBarrierCount = 1;
        dep.pMemoryBarriers = &mb;

        vkCmdPipelineBarrier2(cmd, &dep);
    }

    static glm::vec3 safe_normalize(const glm::vec3 &v, const glm::vec3 &fallback)
    {
        const float len2 = glm::dot(v, v);
        if (len2 <= 1e-10f || !std::isfinite(len2))
        {
            return fallback;
        }
        return v * (1.0f / std::sqrt(len2));
    }

    static float clamp_nonnegative(float v)
    {
        if (!std::isfinite(v)) return 0.0f;
        return std::max(0.0f, v);
    }
}

void ParticlePass::preload_vfx_texture(const std::string &assetName)
{
    (void)get_or_load_vfx_texture(assetName);
}

void ParticlePass::preload_needed_textures()
{
    if (!_context) return;
    for (const auto &sys : _systems)
    {
        if (!sys.flipbook_texture.empty())
        {
            (void)get_or_load_vfx_texture(sys.flipbook_texture);
        }
        if (!sys.noise_texture.empty())
        {
            (void)get_or_load_vfx_texture(sys.noise_texture);
        }
    }
}

AllocatedImage *ParticlePass::find_vfx_texture(std::string_view assetName)
{
    if (assetName.empty() || !_context || !_context->getAssets())
    {
        return nullptr;
    }

    std::string resolved = _context->getAssets()->assetPath(assetName);
    if (resolved.empty())
    {
        return nullptr;
    }

    auto it = _vfx_texture_lookup.find(resolved);
    if (it == _vfx_texture_lookup.end()) return nullptr;
    const uint32_t idx = it->second;
    if (idx >= _vfx_textures.size()) return nullptr;
    return &_vfx_textures[idx].image;
}

AllocatedImage *ParticlePass::get_or_load_vfx_texture(std::string_view assetName)
{
    if (assetName.empty() || !_context || !_context->getAssets() || !_context->getResources())
    {
        return nullptr;
    }

    std::string resolved = _context->getAssets()->assetPath(assetName);
    if (resolved.empty())
    {
        return nullptr;
    }

    auto it = _vfx_texture_lookup.find(resolved);
    if (it != _vfx_texture_lookup.end())
    {
        const uint32_t idx = it->second;
        if (idx < _vfx_textures.size())
        {
            return &_vfx_textures[idx].image;
        }
    }

    if (_vfx_texture_failures.find(resolved) != _vfx_texture_failures.end())
    {
        return nullptr;
    }

    ktxutil::Ktx2D ktx{};
    if (!ktxutil::load_ktx2_2d(resolved.c_str(), ktx))
    {
        _vfx_texture_failures.insert(resolved);
        return nullptr;
    }

    AllocatedImage img = _context->getResources()->create_image_compressed_layers(
        ktx.bytes.data(),
        ktx.bytes.size(),
        ktx.fmt,
        ktx.mipLevels,
        1,
        ktx.copies,
        VK_IMAGE_USAGE_SAMPLED_BIT);

    if (img.image == VK_NULL_HANDLE || img.imageView == VK_NULL_HANDLE)
    {
        _vfx_texture_failures.insert(resolved);
        return nullptr;
    }

    const uint32_t idx = static_cast<uint32_t>(_vfx_textures.size());
    VfxTexture rec{};
    rec.resolvedPath = resolved;
    rec.image = img;
    _vfx_textures.push_back(std::move(rec));
    _vfx_texture_lookup.emplace(resolved, idx);
    return &_vfx_textures[idx].image;
}

void ParticlePass::init(EngineContext *context)
{
    _context = context;
    if (!_context || !_context->getDevice() || !_context->getResources() || !_context->getAssets() ||
        !_context->pipelines || !_context->getDescriptorLayouts() || !_context->getSamplers())
    {
        return;
    }

    _free_ranges.clear();
    _free_ranges.push_back(FreeRange{0u, k_max_particles});

    _particle_pool_size = VkDeviceSize(sizeof(ParticleGPU)) * VkDeviceSize(k_max_particles);
    _particle_pool = _context->getResources()->create_buffer(
        _particle_pool_size,
        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
        VMA_MEMORY_USAGE_GPU_ONLY);

    _draw_indices_size = VkDeviceSize(sizeof(uint32_t)) * VkDeviceSize(k_max_particles);
    _draw_indices = _context->getResources()->create_buffer(
        _draw_indices_size,
        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
        VMA_MEMORY_USAGE_GPU_ONLY);

    _sorted_blocks_size = VkDeviceSize(sizeof(uint32_t)) * VkDeviceSize(k_max_blocks);
    _sorted_blocks = _context->getResources()->create_buffer(
        _sorted_blocks_size,
        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
        VMA_MEMORY_USAGE_GPU_ONLY);

    // Fallback 1x1 textures (uncompressed RGBA8).
    // - flipbook: white (no change)
    // - noise: neutral 0.5 (no UV distortion when remapped to [-1,1])
    {
        const uint32_t white = 0xFFFFFFFFu;
        _fallback_flipbook = _context->getResources()->create_image(&white,
                                                                    VkExtent3D{1, 1, 1},
                                                                    VK_FORMAT_R8G8B8A8_UNORM,
                                                                    VK_IMAGE_USAGE_SAMPLED_BIT,
                                                                    false);
        const uint32_t neutral = 0x80808080u;
        _fallback_noise = _context->getResources()->create_image(&neutral,
                                                                 VkExtent3D{1, 1, 1},
                                                                 VK_FORMAT_R8G8B8A8_UNORM,
                                                                 VK_IMAGE_USAGE_SAMPLED_BIT,
                                                                 false);
    }

    // Zero the pool once so all particles start "dead" and get respawned deterministically by the compute update.
    if (_particle_pool.buffer != VK_NULL_HANDLE)
    {
        VkBuffer buf = _particle_pool.buffer;
        VkDeviceSize size = _particle_pool_size;
        _context->getResources()->immediate_submit([buf, size](VkCommandBuffer cmd) {
            vkCmdFillBuffer(cmd, buf, 0, size, 0u);
        });
    }

    VkDevice device = _context->getDevice()->device();

    // Set=1 layout for graphics: particle pool SSBO + optional draw indices indirection.
    {
        DescriptorLayoutBuilder builder;
        builder.add_binding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER);
        builder.add_binding(1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER);
        _particle_set_layout = builder.build(device, VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT);
    }

    // Set=2 layout for graphics: sampled G-buffer position (for soft particles).
    {
        DescriptorLayoutBuilder builder;
        builder.add_binding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        _input_set_layout = builder.build(
            device,
            VK_SHADER_STAGE_FRAGMENT_BIT,
            nullptr,
            VK_DESCRIPTOR_SET_LAYOUT_CREATE_UPDATE_AFTER_BIND_POOL_BIT);
    }

    // Set=3 layout for graphics: flipbook + noise textures.
    {
        DescriptorLayoutBuilder builder;
        builder.add_binding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        builder.add_binding(1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        _vfx_set_layout = builder.build(
            device,
            VK_SHADER_STAGE_FRAGMENT_BIT,
            nullptr,
            VK_DESCRIPTOR_SET_LAYOUT_CREATE_UPDATE_AFTER_BIND_POOL_BIT);
    }

    // Compute update pipeline + instance.
    {
        ComputePipelineCreateInfo ci{};
        ci.shaderPath = _context->getAssets()->shaderPath("particles_update.comp.spv");
        ci.descriptorTypes = {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER};
        ci.pushConstantSize = sizeof(ParticleUpdatePushConstants);
        ci.pushConstantStages = VK_SHADER_STAGE_COMPUTE_BIT;
        _context->pipelines->createComputePipeline("particles.update", ci);

        _context->pipelines->createComputeInstance("particles.update", "particles.update");
        _context->pipelines->setComputeInstanceBuffer("particles.update", 0, _particle_pool.buffer, _particle_pool_size,
                                                      VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 0);
    }

    // Compute block sort + index build pipelines (alpha block-sorting).
    {
        ComputePipelineCreateInfo sortCi{};
        sortCi.shaderPath = _context->getAssets()->shaderPath("particles_sort_blocks.comp.spv");
        sortCi.descriptorTypes = {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER};
        sortCi.pushConstantSize = sizeof(ParticleSortPushConstants);
        sortCi.pushConstantStages = VK_SHADER_STAGE_COMPUTE_BIT;
        _context->pipelines->createComputePipeline("particles.sort_blocks", sortCi);
        _context->pipelines->createComputeInstance("particles.sort_blocks", "particles.sort_blocks");
        _context->pipelines->setComputeInstanceBuffer("particles.sort_blocks", 0, _particle_pool.buffer, _particle_pool_size,
                                                      VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 0);
        _context->pipelines->setComputeInstanceBuffer("particles.sort_blocks", 1, _sorted_blocks.buffer, _sorted_blocks_size,
                                                      VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 0);

        ComputePipelineCreateInfo buildCi{};
        buildCi.shaderPath = _context->getAssets()->shaderPath("particles_build_indices.comp.spv");
        buildCi.descriptorTypes = {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER};
        buildCi.pushConstantSize = sizeof(ParticleBuildIndicesPushConstants);
        buildCi.pushConstantStages = VK_SHADER_STAGE_COMPUTE_BIT;
        _context->pipelines->createComputePipeline("particles.build_indices", buildCi);
        _context->pipelines->createComputeInstance("particles.build_indices", "particles.build_indices");
        _context->pipelines->setComputeInstanceBuffer("particles.build_indices", 0, _sorted_blocks.buffer, _sorted_blocks_size,
                                                      VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 0);
        _context->pipelines->setComputeInstanceBuffer("particles.build_indices", 1, _draw_indices.buffer, _draw_indices_size,
                                                      VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 0);
    }

    // Graphics pipelines for render (additive + optional alpha).
    {
        const std::string vert = _context->getAssets()->shaderPath("particles.vert.spv");
        const std::string frag = _context->getAssets()->shaderPath("particles.frag.spv");

        GraphicsPipelineCreateInfo base{};
        base.vertexShaderPath = vert;
        base.fragmentShaderPath = frag;
        base.setLayouts = {
            _context->getDescriptorLayouts()->gpuSceneDataLayout(), // set = 0
            _particle_set_layout,                                   // set = 1
            _input_set_layout,                                      // set = 2
            _vfx_set_layout,                                        // set = 3
        };

        VkPushConstantRange pcr{};
        pcr.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
        pcr.offset = 0;
        pcr.size = sizeof(ParticleRenderPushConstants);
        base.pushConstants = { pcr };

        base.configure = [this](PipelineBuilder &b) {
            b.set_input_topology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST);
            b.set_polygon_mode(VK_POLYGON_MODE_FILL);
            b.set_cull_mode(VK_CULL_MODE_NONE, VK_FRONT_FACE_CLOCKWISE);
            b.set_multisampling_none();
            b.enable_depthtest(false, VK_COMPARE_OP_GREATER_OR_EQUAL);
            if (_context && _context->getSwapchain())
            {
                b.set_color_attachment_format(_context->getSwapchain()->drawImage().imageFormat);
                b.set_depth_format(_context->getSwapchain()->depthImage().imageFormat);
            }
        };

        GraphicsPipelineCreateInfo additive = base;
        additive.configure = [baseCfg = base.configure](PipelineBuilder &b) {
            baseCfg(b);
            b.enable_blending_additive();
        };
        _context->pipelines->createGraphicsPipeline("particles.additive", additive);

        GraphicsPipelineCreateInfo alpha = base;
        alpha.configure = [baseCfg = base.configure](PipelineBuilder &b) {
            baseCfg(b);
            b.enable_blending_alphablend();
        };
        _context->pipelines->createGraphicsPipeline("particles.alpha", alpha);
    }
}

void ParticlePass::cleanup()
{
    if (_context && _context->pipelines)
    {
        _context->pipelines->destroyComputeInstance("particles.update");
        _context->pipelines->destroyComputeInstance("particles.sort_blocks");
        _context->pipelines->destroyComputeInstance("particles.build_indices");

        _context->pipelines->destroyComputePipeline("particles.update");
        _context->pipelines->destroyComputePipeline("particles.sort_blocks");
        _context->pipelines->destroyComputePipeline("particles.build_indices");
    }

    if (_context && _context->getDevice())
    {
        if (_particle_set_layout != VK_NULL_HANDLE)
        {
            vkDestroyDescriptorSetLayout(_context->getDevice()->device(), _particle_set_layout, nullptr);
            _particle_set_layout = VK_NULL_HANDLE;
        }
        if (_input_set_layout != VK_NULL_HANDLE)
        {
            vkDestroyDescriptorSetLayout(_context->getDevice()->device(), _input_set_layout, nullptr);
            _input_set_layout = VK_NULL_HANDLE;
        }
        if (_vfx_set_layout != VK_NULL_HANDLE)
        {
            vkDestroyDescriptorSetLayout(_context->getDevice()->device(), _vfx_set_layout, nullptr);
            _vfx_set_layout = VK_NULL_HANDLE;
        }
    }

    if (_context && _context->getResources())
    {
        for (auto &t : _vfx_textures)
        {
            if (t.image.image != VK_NULL_HANDLE)
            {
                _context->getResources()->destroy_image(t.image);
            }
        }
        _vfx_textures.clear();
        _vfx_texture_lookup.clear();
        _vfx_texture_failures.clear();

        if (_fallback_flipbook.image != VK_NULL_HANDLE)
        {
            _context->getResources()->destroy_image(_fallback_flipbook);
            _fallback_flipbook = {};
        }
        if (_fallback_noise.image != VK_NULL_HANDLE)
        {
            _context->getResources()->destroy_image(_fallback_noise);
            _fallback_noise = {};
        }

        if (_particle_pool.buffer != VK_NULL_HANDLE)
        {
            _context->getResources()->destroy_buffer(_particle_pool);
            _particle_pool = {};
        }
        if (_draw_indices.buffer != VK_NULL_HANDLE)
        {
            _context->getResources()->destroy_buffer(_draw_indices);
            _draw_indices = {};
        }
        if (_sorted_blocks.buffer != VK_NULL_HANDLE)
        {
            _context->getResources()->destroy_buffer(_sorted_blocks);
            _sorted_blocks = {};
        }
    }

    _systems.clear();
    _free_ranges.clear();
    _context = nullptr;
}

void ParticlePass::execute(VkCommandBuffer)
{
    // Executed via RenderGraph.
}

uint32_t ParticlePass::free_particles() const
{
    uint64_t sum = 0;
    for (const auto &r : _free_ranges) sum += r.count;
    if (sum > k_max_particles) sum = k_max_particles;
    return static_cast<uint32_t>(sum);
}

uint32_t ParticlePass::allocated_particles() const
{
    return k_max_particles - free_particles();
}

ParticlePass::System *ParticlePass::find_system(uint32_t id)
{
    for (auto &s : _systems)
    {
        if (s.id == id) return &s;
    }
    return nullptr;
}

bool ParticlePass::allocate_range(uint32_t count, uint32_t &out_base)
{
    if (count == 0) return false;
    for (auto it = _free_ranges.begin(); it != _free_ranges.end(); ++it)
    {
        if (it->count < count) continue;

        out_base = it->base;
        it->base += count;
        it->count -= count;
        if (it->count == 0) _free_ranges.erase(it);
        return true;
    }
    return false;
}

void ParticlePass::merge_free_ranges()
{
    if (_free_ranges.size() < 2) return;

    std::sort(_free_ranges.begin(), _free_ranges.end(), [](const FreeRange &a, const FreeRange &b) {
        return a.base < b.base;
    });

    std::vector<FreeRange> merged;
    merged.reserve(_free_ranges.size());

    FreeRange cur = _free_ranges.front();
    for (size_t i = 1; i < _free_ranges.size(); ++i)
    {
        const FreeRange &n = _free_ranges[i];
        const uint32_t cur_end = cur.base + cur.count;
        if (n.base <= cur_end)
        {
            const uint32_t n_end = n.base + n.count;
            cur.count = std::max(cur_end, n_end) - cur.base;
        }
        else
        {
            merged.push_back(cur);
            cur = n;
        }
    }
    merged.push_back(cur);
    _free_ranges = std::move(merged);
}

void ParticlePass::free_range(uint32_t base, uint32_t count)
{
    if (count == 0) return;
    _free_ranges.push_back(FreeRange{base, count});
    merge_free_ranges();
}

uint32_t ParticlePass::create_system(uint32_t count)
{
    if (count == 0) return 0;
    count = std::min(count, free_particles());

    uint32_t base = 0;
    if (!allocate_range(count, base)) return 0;

    System s{};
    s.id = _next_system_id++;
    s.base = base;
    s.count = count;
    s.enabled = true;
    s.reset = true;
    s.blend = BlendMode::Additive;
    s.params = Params{};

    _systems.push_back(s);
    // Load default flipbook/noise if configured. Happens during UI before draw(), so uploads can be captured this frame.
    preload_vfx_texture(_systems.back().flipbook_texture);
    preload_vfx_texture(_systems.back().noise_texture);
    return s.id;
}

bool ParticlePass::destroy_system(uint32_t id)
{
    for (size_t i = 0; i < _systems.size(); ++i)
    {
        if (_systems[i].id != id) continue;
        free_range(_systems[i].base, _systems[i].count);
        _systems.erase(_systems.begin() + static_cast<std::ptrdiff_t>(i));
        return true;
    }
    return false;
}

bool ParticlePass::resize_system(uint32_t id, uint32_t new_count)
{
    System *s = find_system(id);
    if (!s) return false;
    if (new_count == s->count) return true;
    if (new_count == 0) return destroy_system(id);

    const uint32_t old_base = s->base;
    const uint32_t old_count = s->count;

    // Shrink in place: keep base and return the tail to the freelist.
    if (new_count < old_count)
    {
        const uint32_t tail_base = old_base + new_count;
        const uint32_t tail_count = old_count - new_count;
        s->count = new_count;
        free_range(tail_base, tail_count);
        return true;
    }

    // Try to grow in place if the next range is free.
    const uint32_t extra = new_count - old_count;
    const uint32_t want_base = old_base + old_count;
    for (auto it = _free_ranges.begin(); it != _free_ranges.end(); ++it)
    {
        if (it->base != want_base) continue;
        if (it->count < extra) break;

        it->base += extra;
        it->count -= extra;
        if (it->count == 0) _free_ranges.erase(it);

        s->count = new_count;
        s->reset = true;
        return true;
    }

    // Fallback: allocate a new range and recycle the old one.
    uint32_t base = 0;
    if (!allocate_range(new_count, base)) return false;

    s->base = base;
    s->count = new_count;
    s->reset = true;

    free_range(old_base, old_count);
    return true;
}

void ParticlePass::register_graph(RenderGraph *graph, RGImageHandle hdrTarget, RGImageHandle depthHandle,
                                  RGImageHandle gbufferPosition)
{
    if (!graph || !_context || !_context->pipelines || _particle_pool.buffer == VK_NULL_HANDLE ||
        _draw_indices.buffer == VK_NULL_HANDLE || _sorted_blocks.buffer == VK_NULL_HANDLE ||
        _particle_set_layout == VK_NULL_HANDLE || _input_set_layout == VK_NULL_HANDLE || _vfx_set_layout == VK_NULL_HANDLE ||
        !hdrTarget.valid() || !depthHandle.valid() || !gbufferPosition.valid())
    {
        return;
    }

    // Per-frame dt/time and floating-origin delta
    _dt_sec = 0.0f;
    if (_context->scene)
    {
        _dt_sec = _context->scene->getDeltaTime();
    }
    if (!std::isfinite(_dt_sec)) _dt_sec = 0.0f;
    _dt_sec = std::clamp(_dt_sec, 0.0f, 0.1f);
    _time_sec += _dt_sec;

    _origin_delta_local = glm::vec3(0.0f);
    if (_context->scene)
    {
        WorldVec3 origin_world = _context->scene->get_world_origin();
        if (_has_prev_origin)
        {
            const WorldVec3 delta_world = origin_world - _prev_origin_world;
            _origin_delta_local = glm::vec3(delta_world);
        }
        _prev_origin_world = origin_world;
        _has_prev_origin = true;
    }

    const float origin_delta_len2 = glm::dot(_origin_delta_local, _origin_delta_local);
    const bool origin_delta_valid = std::isfinite(origin_delta_len2) && origin_delta_len2 > 0.0f;
    if (origin_delta_valid)
    {
        for (auto &sys : _systems)
        {
            sys.params.emitter_pos_local -= _origin_delta_local;
        }
    }

    bool any_active = false;
    for (const auto &s : _systems)
    {
        if (s.enabled && s.count > 0)
        {
            any_active = true;
            break;
        }
    }
    if (!any_active) return;

    VkBuffer pool = _particle_pool.buffer;
    VkDeviceSize poolSize = _particle_pool_size;
    VkBuffer indices = _draw_indices.buffer;
    VkDeviceSize indicesSize = _draw_indices_size;
    VkBuffer sortedBlocks = _sorted_blocks.buffer;
    VkDeviceSize sortedBlocksSize = _sorted_blocks_size;

    graph->add_pass(
        "Particles.Update",
        RGPassType::Compute,
        [pool, poolSize, indices, indicesSize, sortedBlocks, sortedBlocksSize](RGPassBuilder &builder, EngineContext *) {
            builder.write_buffer(pool, RGBufferUsage::StorageReadWrite, poolSize, "particles.pool");
            builder.write_buffer(indices, RGBufferUsage::StorageReadWrite, indicesSize, "particles.indices");
            builder.write_buffer(sortedBlocks, RGBufferUsage::StorageReadWrite, sortedBlocksSize, "particles.sorted_blocks");
        },
        [this](VkCommandBuffer cmd, const RGPassResources &, EngineContext *ctx) {
            EngineContext *ctxLocal = ctx ? ctx : _context;
            if (!ctxLocal || !ctxLocal->pipelines) return;

            for (auto &sys : _systems)
            {
                if (!sys.enabled || sys.count == 0) continue;

                ParticleUpdatePushConstants pc{};
                pc.header = glm::uvec4(sys.base, sys.count, sys.reset ? 1u : 0u, 0u);

                float minSpeed = sys.params.min_speed;
                float maxSpeed = sys.params.max_speed;
                if (!std::isfinite(minSpeed)) minSpeed = 0.0f;
                if (!std::isfinite(maxSpeed)) maxSpeed = 0.0f;
                if (maxSpeed < minSpeed) std::swap(minSpeed, maxSpeed);

                float minLife = sys.params.min_life;
                float maxLife = sys.params.max_life;
                if (!std::isfinite(minLife)) minLife = 0.1f;
                if (!std::isfinite(maxLife)) maxLife = 0.1f;
                if (maxLife < minLife) std::swap(minLife, maxLife);

                float minSize = sys.params.min_size;
                float maxSize = sys.params.max_size;
                if (!std::isfinite(minSize)) minSize = 0.01f;
                if (!std::isfinite(maxSize)) maxSize = 0.01f;
                if (maxSize < minSize) std::swap(minSize, maxSize);

                const float radius = clamp_nonnegative(sys.params.spawn_radius);
                const float drag = clamp_nonnegative(sys.params.drag);
                const float gravity = sys.params.gravity;

                pc.sim = glm::vec4(_dt_sec, _time_sec, drag, gravity);
                pc.origin_delta = glm::vec4(_origin_delta_local, 0.0f);
                pc.emitter_pos_radius = glm::vec4(sys.params.emitter_pos_local, radius);

                glm::vec3 dir = safe_normalize(sys.params.emitter_dir_local, glm::vec3(0.0f, 1.0f, 0.0f));
                const float coneRad = glm::radians(sys.params.cone_angle_degrees);
                pc.emitter_dir_cone = glm::vec4(dir, coneRad);

                pc.ranges = glm::vec4(minSpeed, maxSpeed, minLife, maxLife);
                pc.size_range = glm::vec4(minSize, maxSize, 0.0f, 0.0f);
                pc.color = sys.params.color;

                ComputeDispatchInfo di{};
                di.groupCountX = ComputeManager::calculateGroupCount(sys.count, k_local_size_x);
                di.groupCountY = 1;
                di.groupCountZ = 1;
                di.pushConstants = &pc;
                di.pushConstantSize = sizeof(pc);

                ctxLocal->pipelines->dispatchComputeInstance(cmd, "particles.update", di);

                if (sys.blend == BlendMode::Alpha)
                {
                    // Ensure particle writes are visible before block depth reads.
                    cmd_compute_memory_barrier(cmd,
                                               VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT,
                                               VK_ACCESS_2_SHADER_STORAGE_READ_BIT);

                    ParticleSortPushConstants spc{};
                    spc.header = glm::uvec4(sys.base, sys.count, 0u, 0u);
                    spc.view = ctxLocal->getSceneData().view;

                    ComputeDispatchInfo sdi{};
                    sdi.groupCountX = 1;
                    sdi.groupCountY = 1;
                    sdi.groupCountZ = 1;
                    sdi.pushConstants = &spc;
                    sdi.pushConstantSize = sizeof(spc);
                    ctxLocal->pipelines->dispatchComputeInstance(cmd, "particles.sort_blocks", sdi);

                    cmd_compute_memory_barrier(cmd,
                                               VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT,
                                               VK_ACCESS_2_SHADER_STORAGE_READ_BIT);

                    ParticleBuildIndicesPushConstants ipc{};
                    ipc.header = glm::uvec4(sys.base, sys.count, 0u /*flags*/, 0u);

                    ComputeDispatchInfo idi{};
                    idi.groupCountX = ComputeManager::calculateGroupCount(sys.count, k_local_size_x);
                    idi.groupCountY = 1;
                    idi.groupCountZ = 1;
                    idi.pushConstants = &ipc;
                    idi.pushConstantSize = sizeof(ipc);
                    ctxLocal->pipelines->dispatchComputeInstance(cmd, "particles.build_indices", idi);
                }
                else
                {
                    ParticleBuildIndicesPushConstants ipc{};
                    ipc.header = glm::uvec4(sys.base, sys.count, 1u /*identity*/, 0u);

                    ComputeDispatchInfo idi{};
                    idi.groupCountX = ComputeManager::calculateGroupCount(sys.count, k_local_size_x);
                    idi.groupCountY = 1;
                    idi.groupCountZ = 1;
                    idi.pushConstants = &ipc;
                    idi.pushConstantSize = sizeof(ipc);
                    ctxLocal->pipelines->dispatchComputeInstance(cmd, "particles.build_indices", idi);
                }

                sys.reset = false;
            }
        });

    graph->add_pass(
        "Particles.Render",
        RGPassType::Graphics,
        [pool, poolSize, indices, indicesSize, hdrTarget, depthHandle, gbufferPosition](RGPassBuilder &builder, EngineContext *) {
            builder.read_buffer(pool, RGBufferUsage::StorageRead, poolSize, "particles.pool");
            builder.read_buffer(indices, RGBufferUsage::StorageRead, indicesSize, "particles.indices");
            builder.read(gbufferPosition, RGImageUsage::SampledFragment);
            builder.write_color(hdrTarget);
            builder.write_depth(depthHandle, false /*load existing depth*/);
        },
        [this, gbufferPosition](VkCommandBuffer cmd, const RGPassResources &res, EngineContext *ctx) {
            EngineContext *ctxLocal = ctx ? ctx : _context;
            if (!ctxLocal || !ctxLocal->currentFrame) return;

            ResourceManager *rm = ctxLocal->getResources();
            DeviceManager *dev = ctxLocal->getDevice();
            DescriptorManager *layouts = ctxLocal->getDescriptorLayouts();
            PipelineManager *pipes = ctxLocal->pipelines;
            SamplerManager *samplers = ctxLocal->getSamplers();
            if (!rm || !dev || !layouts || !pipes || !samplers ||
                _particle_set_layout == VK_NULL_HANDLE || _input_set_layout == VK_NULL_HANDLE)
            {
                return;
            }

            VkImageView posView = res.image_view(gbufferPosition);
            if (posView == VK_NULL_HANDLE) return;

            // Per-frame SceneData UBO (set=0 binding=0)
            AllocatedBuffer sceneBuf = rm->create_buffer(sizeof(GPUSceneData),
                                                         VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
                                                         VMA_MEMORY_USAGE_CPU_TO_GPU);
            ctxLocal->currentFrame->_deletionQueue.push_function([rm, sceneBuf]() { rm->destroy_buffer(sceneBuf); });

            VmaAllocationInfo ai{};
            vmaGetAllocationInfo(dev->allocator(), sceneBuf.allocation, &ai);
            *reinterpret_cast<GPUSceneData *>(ai.pMappedData) = ctxLocal->getSceneData();
            vmaFlushAllocation(dev->allocator(), sceneBuf.allocation, 0, sizeof(GPUSceneData));

            VkDescriptorSet globalSet = ctxLocal->currentFrame->_frameDescriptors.allocate(
                dev->device(), layouts->gpuSceneDataLayout());
            {
                DescriptorWriter w;
                w.write_buffer(0, sceneBuf.buffer, sizeof(GPUSceneData), 0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER);
                w.update_set(dev->device(), globalSet);
            }

            // Particle pool descriptor (set=1 binding=0)
            VkDescriptorSet particleSet = ctxLocal->currentFrame->_frameDescriptors.allocate(
                dev->device(), _particle_set_layout);
            {
                DescriptorWriter w;
                w.write_buffer(0, _particle_pool.buffer, _particle_pool_size, 0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER);
                w.write_buffer(1, _draw_indices.buffer, _draw_indices_size, 0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER);
                w.update_set(dev->device(), particleSet);
            }

            // Inputs (set=2): G-buffer position texture for soft particles.
            VkDescriptorSet inputSet = ctxLocal->currentFrame->_frameDescriptors.allocate(
                dev->device(), _input_set_layout);
            {
                DescriptorWriter w;
                w.write_image(0, posView, samplers->defaultNearest(),
                              VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                              VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
                w.update_set(dev->device(), inputSet);
            }

            VkExtent2D extent = ctxLocal->getDrawExtent();
            VkViewport vp{0.0f, 0.0f, float(extent.width), float(extent.height), 0.0f, 1.0f};
            VkRect2D sc{{0, 0}, extent};
            vkCmdSetViewport(cmd, 0, 1, &vp);
            vkCmdSetScissor(cmd, 0, 1, &sc);

            VkPipelineLayout layout = VK_NULL_HANDLE;
            VkPipeline pipeline = VK_NULL_HANDLE;
            BlendMode boundBlend = BlendMode::Additive;
            bool hasBound = false;

            auto bind_pipeline = [&](BlendMode blend) {
                const char *name = (blend == BlendMode::Alpha) ? "particles.alpha" : "particles.additive";
                if (!pipes->getGraphics(name, pipeline, layout)) return false;
                vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);
                vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, layout, 0, 1, &globalSet, 0, nullptr);
                vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, layout, 1, 1, &particleSet, 0, nullptr);
                vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, layout, 2, 1, &inputSet, 0, nullptr);
                boundBlend = blend;
                hasBound = true;
                return true;
            };

            for (const auto &sys : _systems)
            {
                if (!sys.enabled || sys.count == 0) continue;
                if (!hasBound || sys.blend != boundBlend)
                {
                    if (!bind_pipeline(sys.blend)) continue;
                }

                // VFX textures (set=3): flipbook + noise (per-system).
                VkDescriptorSet vfxSet = ctxLocal->currentFrame->_frameDescriptors.allocate(
                    dev->device(), _vfx_set_layout);
                {
                    AllocatedImage *flipImg = find_vfx_texture(sys.flipbook_texture);
                    if (!flipImg || flipImg->imageView == VK_NULL_HANDLE)
                    {
                        flipImg = &_fallback_flipbook;
                    }
                    AllocatedImage *noiseImg = find_vfx_texture(sys.noise_texture);
                    if (!noiseImg || noiseImg->imageView == VK_NULL_HANDLE)
                    {
                        noiseImg = &_fallback_noise;
                    }

                    DescriptorWriter w;
                    w.write_image(0, flipImg->imageView, samplers->defaultLinear(),
                                  VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                                  VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
                    w.write_image(1, noiseImg->imageView, samplers->defaultLinear(),
                                  VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                                  VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
                    w.update_set(dev->device(), vfxSet);
                }
                vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, layout, 3, 1, &vfxSet, 0, nullptr);

                ParticleRenderPushConstants pc{};
                const float invW = (extent.width > 0) ? (1.0f / float(extent.width)) : 0.0f;
                const float invH = (extent.height > 0) ? (1.0f / float(extent.height)) : 0.0f;
                const float softDist = clamp_nonnegative(sys.params.soft_depth_distance);
                pc.screen = glm::vec4(invW, invH, softDist, _time_sec);

                const uint32_t colsU = std::max(sys.params.flipbook_cols, 1u);
                const uint32_t rowsU = std::max(sys.params.flipbook_rows, 1u);
                float fps = sys.params.flipbook_fps;
                if (!std::isfinite(fps)) fps = 0.0f;
                fps = std::clamp(fps, 0.0f, 240.0f);
                float intensity = sys.params.flipbook_intensity;
                if (!std::isfinite(intensity)) intensity = 0.0f;
                intensity = std::max(0.0f, intensity);
                pc.flipbook = glm::vec4(float(colsU), float(rowsU), fps, intensity);

                float noiseScale = sys.params.noise_scale;
                if (!std::isfinite(noiseScale)) noiseScale = 0.0f;
                noiseScale = std::max(0.0f, noiseScale);
                float noiseStrength = sys.params.noise_strength;
                if (!std::isfinite(noiseStrength)) noiseStrength = 0.0f;
                noiseStrength = std::max(0.0f, noiseStrength);
                glm::vec2 scroll = sys.params.noise_scroll;
                if (!std::isfinite(scroll.x)) scroll.x = 0.0f;
                if (!std::isfinite(scroll.y)) scroll.y = 0.0f;
                pc.noise = glm::vec4(noiseScale, noiseStrength, scroll.x, scroll.y);

                vkCmdPushConstants(cmd, layout, VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(pc), &pc);

                // Instanced quad draw. gl_InstanceIndex includes firstInstance, so it becomes the particle index.
                vkCmdDraw(cmd, 6, sys.count, 0, sys.base);
                if (ctxLocal->stats)
                {
                    ctxLocal->stats->drawcall_count += 1;
                    ctxLocal->stats->triangle_count += static_cast<int>(sys.count) * 2;
                }
            }
        });
}
