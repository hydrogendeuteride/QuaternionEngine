#include "particles.h"

#include "compute/vk_compute.h"
#include "core/assets/manager.h"
#include "core/context.h"
#include "core/descriptor/descriptors.h"
#include "core/descriptor/manager.h"
#include "core/device/device.h"
#include "core/device/resource.h"
#include "core/device/swapchain.h"
#include "core/frame/resources.h"
#include "core/pipeline/manager.h"
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

void ParticlePass::init(EngineContext *context)
{
    _context = context;
    if (!_context || !_context->getDevice() || !_context->getResources() || !_context->getAssets() ||
        !_context->pipelines || !_context->getDescriptorLayouts())
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

    // Set=1 layout for graphics: particle pool SSBO.
    {
        DescriptorLayoutBuilder builder;
        builder.add_binding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER);
        _particle_set_layout = builder.build(device, VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT);
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
        };

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
    if (_context && _context->getDevice())
    {
        if (_particle_set_layout != VK_NULL_HANDLE)
        {
            vkDestroyDescriptorSetLayout(_context->getDevice()->device(), _particle_set_layout, nullptr);
            _particle_set_layout = VK_NULL_HANDLE;
        }
    }

    if (_context && _context->getResources())
    {
        if (_particle_pool.buffer != VK_NULL_HANDLE)
        {
            _context->getResources()->destroy_buffer(_particle_pool);
            _particle_pool = {};
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

void ParticlePass::register_graph(RenderGraph *graph, RGImageHandle hdrTarget, RGImageHandle depthHandle)
{
    if (!graph || !_context || !_context->pipelines || _particle_pool.buffer == VK_NULL_HANDLE ||
        !hdrTarget.valid() || !depthHandle.valid())
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

    graph->add_pass(
        "Particles.Update",
        RGPassType::Compute,
        [pool, poolSize](RGPassBuilder &builder, EngineContext *) {
            builder.write_buffer(pool, RGBufferUsage::StorageReadWrite, poolSize, "particles.pool");
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

                sys.reset = false;
            }
        });

    graph->add_pass(
        "Particles.Render",
        RGPassType::Graphics,
        [pool, poolSize, hdrTarget, depthHandle](RGPassBuilder &builder, EngineContext *) {
            builder.read_buffer(pool, RGBufferUsage::StorageRead, poolSize, "particles.pool");
            builder.write_color(hdrTarget);
            builder.write_depth(depthHandle, false /*load existing depth*/);
        },
        [this](VkCommandBuffer cmd, const RGPassResources &, EngineContext *ctx) {
            EngineContext *ctxLocal = ctx ? ctx : _context;
            if (!ctxLocal || !ctxLocal->currentFrame) return;

            ResourceManager *rm = ctxLocal->getResources();
            DeviceManager *dev = ctxLocal->getDevice();
            DescriptorManager *layouts = ctxLocal->getDescriptorLayouts();
            PipelineManager *pipes = ctxLocal->pipelines;
            if (!rm || !dev || !layouts || !pipes || _particle_set_layout == VK_NULL_HANDLE) return;

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
                w.update_set(dev->device(), particleSet);
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
