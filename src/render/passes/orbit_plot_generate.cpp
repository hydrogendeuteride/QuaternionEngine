#include "orbit_plot_generate.h"

#include <compute/vk_compute.h>
#include <core/assets/manager.h>
#include <core/context.h>
#include <core/device/device.h>
#include <core/device/resource.h>
#include <core/orbit_plot/orbit_plot.h>
#include <core/pipeline/manager.h>
#include <render/graph/graph.h>
#include <scene/vk_scene.h>

#include <algorithm>
#include <chrono>
#include <cfloat>
#include <cstring>
#include <limits>
#include <cmath>

namespace
{
    constexpr const char *k_pipeline_name = "orbit_plot.generate";
    constexpr const char *k_instance_name = "orbit_plot.generate.instance";
    constexpr const char *k_shader_name = "orbit_plot_generate.comp.spv";

    constexpr std::size_t k_min_input_capacity_lines = 128;
    constexpr std::size_t k_min_output_capacity_segments = 512;
    constexpr uint32_t k_max_subdivision_depth = 24;
    constexpr float k_min_subdivision_dt_s = 1.0e-6f;
    constexpr double k_upload_frustum_margin_ratio = 0.05;
    constexpr double k_min_upload_clip_dt_s = 1.0e-6;

    struct OrbitPlotGeneratePushConstants
    {
        glm::vec4 camera_local_tan_half_fov{0.0f};
        glm::vec4 subdivision_error_px{0.0f};
        glm::vec4 viewport_height_min_dt{0.0f};
        glm::vec4 dash_on_off_period{0.0f};
        glm::uvec4 count_limits{0u};
    };
    static_assert(sizeof(OrbitPlotGeneratePushConstants) == 80);

    static std::size_t grow_capacity(std::size_t required, std::size_t current, std::size_t min_value)
    {
        std::size_t cap = std::max(current, min_value);
        while (cap < required)
        {
            const std::size_t next = cap * 2ull;
            if (next <= cap)
            {
                break;
            }
            cap = next;
        }
        return std::max(cap, required);
    }

    static bool pack_vec3_finite(const glm::dvec3 &src, glm::vec3 &dst)
    {
        if (!std::isfinite(src.x) || !std::isfinite(src.y) || !std::isfinite(src.z))
        {
            return false;
        }

        if (std::abs(src.x) > static_cast<double>(FLT_MAX) ||
            std::abs(src.y) > static_cast<double>(FLT_MAX) ||
            std::abs(src.z) > static_cast<double>(FLT_MAX))
        {
            return false;
        }

        dst = glm::vec3(static_cast<float>(src.x),
                        static_cast<float>(src.y),
                        static_cast<float>(src.z));
        return std::isfinite(dst.x) && std::isfinite(dst.y) && std::isfinite(dst.z);
    }

    static glm::dvec3 eval_root_local_position(const OrbitPlotSystem::GpuRootSegment &root,
                                               double u,
                                               const glm::dvec3 &reference_offset_local_d)
    {
        u = std::clamp(u, 0.0, 1.0);
        const double u2 = u * u;
        const double u3 = u2 * u;

        const double h00 = (2.0 * u3) - (3.0 * u2) + 1.0;
        const double h10 = u3 - (2.0 * u2) + u;
        const double h01 = (-2.0 * u3) + (3.0 * u2);
        const double h11 = u3 - u2;

        const glm::dvec3 p0 = reference_offset_local_d + root.p0_bci;
        const glm::dvec3 p1 = reference_offset_local_d + root.p1_bci;
        const glm::dvec3 m0 = root.v0_bci * root.dt_s;
        const glm::dvec3 m1 = root.v1_bci * root.dt_s;
        return (h00 * p0) + (h10 * m0) + (h01 * p1) + (h11 * m1);
    }

    static bool frustum_contains_local_margin(const glm::mat4 &viewproj,
                                              const glm::dvec3 &point_local_d,
                                              const double margin_ratio)
    {
        glm::vec3 point_local{};
        if (!pack_vec3_finite(point_local_d, point_local))
        {
            return false;
        }

        const glm::vec4 clip = viewproj * glm::vec4(point_local, 1.0f);
        if (!std::isfinite(clip.x) || !std::isfinite(clip.y) || !std::isfinite(clip.z) || !std::isfinite(clip.w))
        {
            return false;
        }

        const double w = static_cast<double>(clip.w);
        const double aw = std::abs(w);
        if (!(aw > 1.0e-6))
        {
            return false;
        }

        const double safe_margin = std::isfinite(margin_ratio) ? std::max(0.0, margin_ratio) : 0.0;
        const double x = static_cast<double>(clip.x);
        const double y = static_cast<double>(clip.y);
        const double z = static_cast<double>(clip.z);

        const double xy_bound = (1.0 + safe_margin) * aw;
        const double z_min = -safe_margin * aw;
        const double z_max = (1.0 + safe_margin) * aw;

        return (x >= -xy_bound && x <= xy_bound) &&
               (y >= -xy_bound && y <= xy_bound) &&
               (z >= z_min && z <= z_max);
    }

    static bool frustum_accept_root_margin(const glm::mat4 &viewproj,
                                           const glm::dvec3 &a_local_d,
                                           const glm::dvec3 &b_local_d,
                                           const double margin_ratio)
    {
        if (frustum_contains_local_margin(viewproj, a_local_d, margin_ratio) ||
            frustum_contains_local_margin(viewproj, b_local_d, margin_ratio))
        {
            return true;
        }

        const glm::dvec3 mid_local_d = 0.5 * (a_local_d + b_local_d);
        return frustum_contains_local_margin(viewproj, mid_local_d, margin_ratio);
    }
} // namespace

void OrbitPlotGenerate::init(EngineContext *context)
{
    _context = context;
    _pipeline_ready = false;
    if (!_context || !_context->pipelines || !_context->getAssets())
    {
        return;
    }

    ComputePipelineCreateInfo ci{};
    ci.shaderPath = _context->getAssets()->shaderPath(k_shader_name);
    ci.descriptorTypes = {
            VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
            VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
            VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
            VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
    };
    ci.pushConstantSize = sizeof(OrbitPlotGeneratePushConstants);
    ci.pushConstantStages = VK_SHADER_STAGE_COMPUTE_BIT;

    const bool pipeline_ok = _context->pipelines->createComputePipeline(k_pipeline_name, ci);
    const bool instance_ok = pipeline_ok && _context->pipelines->createComputeInstance(k_instance_name, k_pipeline_name);
    _pipeline_ready = pipeline_ok && instance_ok;

    if (!_pipeline_ready && _context->pipelines)
    {
        _context->pipelines->destroyComputeInstance(k_instance_name);
        _context->pipelines->destroyComputePipeline(k_pipeline_name);
    }
}

void OrbitPlotGenerate::cleanup()
{
    if (_context && _context->getResources())
    {
        for (Slot &slot : _slots)
        {
            destroy_slot_resources(slot);
        }
    }

    if (_context && _context->pipelines)
    {
        _context->pipelines->destroyComputeInstance(k_instance_name);
        _context->pipelines->destroyComputePipeline(k_pipeline_name);
    }

    _input_staging.clear();
    _pipeline_ready = false;
    _context = nullptr;
}

bool OrbitPlotGenerate::prepare_and_register(RenderGraph *graph,
                                             const OrbitPlotSystem &plot,
                                             const WorldVec3 &origin_world,
                                             const uint32_t frame_index,
                                             std::size_t max_segments_gpu,
                                             PreparedFrame &out_frame)
{
    out_frame = {};
    if (!_pipeline_ready || !_context || !graph || !_context->getResources() || !_context->getDevice() || !_context->pipelines)
    {
        return false;
    }

    const std::span<const OrbitPlotSystem::GpuRootSegment> active = plot.active_gpu_root_segments();
    if (active.empty())
    {
        return false;
    }

    glm::vec3 camera_local{0.0f, 0.0f, 0.0f};
    float tan_half_fov = std::tan(glm::radians(70.0f) * 0.5f);
    if (_context->scene)
    {
        const Camera &cam = _context->scene->getMainCamera();
        camera_local = world_to_local(cam.position_world, origin_world);
        tan_half_fov = std::tan(glm::radians(cam.fovDegrees) * 0.5f);
    }

    const WorldVec3 reference_offset_world = plot.gpu_reference_body_world() + plot.gpu_align_delta_world();
    const glm::dvec3 reference_offset_local_d = world_to_local_d(reference_offset_world, origin_world);

    const bool frustum_valid = (_context->scene != nullptr);
    const glm::mat4 frustum_viewproj = _context->getSceneData().viewproj;

    max_segments_gpu = std::max<std::size_t>(1ull, max_segments_gpu);

    std::size_t upload_budget_bytes = plot.settings().upload_budget_bytes;
    const std::size_t min_upload_budget = sizeof(GenerateInputLine);
    if (upload_budget_bytes < min_upload_budget)
    {
        upload_budget_bytes = min_upload_budget;
    }

    std::size_t max_upload_roots = upload_budget_bytes / sizeof(GenerateInputLine);
    max_upload_roots = std::max<std::size_t>(1ull, max_upload_roots);

    struct CulledRoot
    {
        const OrbitPlotSystem::GpuRootSegment *root = nullptr;
        double u0 = 0.0;
        double u1 = 1.0;
    };

    std::vector<CulledRoot> culled_roots{};
    culled_roots.reserve(active.size());
    for (const OrbitPlotSystem::GpuRootSegment &root : active)
    {
        if (!(root.dt_s > 0.0) || !std::isfinite(root.dt_s))
        {
            continue;
        }

        const double u0 = std::clamp(root.clip_u0, 0.0, 1.0);
        const double u1 = std::clamp(root.clip_u1, 0.0, 1.0);
        if (!(u1 > u0))
        {
            continue;
        }

        const double clip_dt_s = (u1 - u0) * root.dt_s;
        if (!std::isfinite(clip_dt_s) || !(clip_dt_s > k_min_upload_clip_dt_s))
        {
            continue;
        }

        if (frustum_valid)
        {
            const glm::dvec3 a_local_d = eval_root_local_position(root, u0, reference_offset_local_d);
            const glm::dvec3 b_local_d = eval_root_local_position(root, u1, reference_offset_local_d);
            if (!frustum_accept_root_margin(frustum_viewproj, a_local_d, b_local_d, k_upload_frustum_margin_ratio))
            {
                continue;
            }
        }

        culled_roots.push_back(CulledRoot{
                .root = &root,
                .u0 = u0,
                .u1 = u1,
        });
    }

    if (culled_roots.empty())
    {
        return false;
    }

    const std::size_t input_root_count = std::min<std::size_t>(culled_roots.size(), max_upload_roots);
    const bool upload_cap_hit = culled_roots.size() > input_root_count;
    if (input_root_count == 0)
    {
        return false;
    }

    _input_staging.clear();
    _input_staging.reserve(input_root_count);

    const std::size_t capped_input_count = std::min<std::size_t>(input_root_count, max_segments_gpu);
    uint32_t depth_estimate = 0;
    uint32_t overlay_estimate = 0;

    for (std::size_t i = 0; i < input_root_count; ++i)
    {
        const CulledRoot &root_entry = culled_roots[i];
        if (!root_entry.root)
        {
            continue;
        }

        const OrbitPlotSystem::GpuRootSegment &root = *root_entry.root;
        const float dt_s = static_cast<float>(root.dt_s);
        const float u0 = static_cast<float>(root_entry.u0);
        const float u1 = static_cast<float>(root_entry.u1);
        if (!std::isfinite(dt_s) || !(dt_s > 0.0f) || !std::isfinite(u0) || !std::isfinite(u1) || !(u1 > u0))
        {
            continue;
        }

        const glm::dvec3 p0_local_d = reference_offset_local_d + root.p0_bci;
        const glm::dvec3 p1_local_d = reference_offset_local_d + root.p1_bci;
        if (!std::isfinite(p0_local_d.x) || !std::isfinite(p0_local_d.y) || !std::isfinite(p0_local_d.z) ||
            !std::isfinite(p1_local_d.x) || !std::isfinite(p1_local_d.y) || !std::isfinite(p1_local_d.z))
        {
            continue;
        }

        const glm::dvec3 anchor_local_d = 0.5 * (p0_local_d + p1_local_d);
        const glm::dvec3 p0_rel_d = p0_local_d - anchor_local_d;
        const glm::dvec3 p1_rel_d = p1_local_d - anchor_local_d;

        glm::vec3 anchor_local{};
        glm::vec3 p0_rel{};
        glm::vec3 p1_rel{};
        glm::vec3 v0{};
        glm::vec3 v1{};
        if (!pack_vec3_finite(anchor_local_d, anchor_local) ||
            !pack_vec3_finite(p0_rel_d, p0_rel) ||
            !pack_vec3_finite(p1_rel_d, p1_rel) ||
            !pack_vec3_finite(root.v0_bci, v0) ||
            !pack_vec3_finite(root.v1_bci, v1))
        {
            continue;
        }

        GenerateInputLine gpu_line{};
        gpu_line.anchor_dt = glm::vec4(anchor_local, dt_s);
        gpu_line.p0_rel_u0 = glm::vec4(p0_rel, u0);
        gpu_line.p1_rel_u1 = glm::vec4(p1_rel, u1);
        gpu_line.v0_depth = glm::vec4(v0, root.depth == OrbitPlotDepth::DepthTested ? 0.0f : 1.0f);
        gpu_line.v1_dashed = glm::vec4(v1, root.dashed ? 1.0f : 0.0f);
        gpu_line.dash_phase_dashed = glm::vec4(std::max(0.0f, root.dash_phase_start_px),
                                               0.0f,
                                               0.0f,
                                               0.0f);
        gpu_line.color = root.color;
        _input_staging.push_back(gpu_line);

        if (_input_staging.size() <= capped_input_count)
        {
            if (root.depth == OrbitPlotDepth::DepthTested)
            {
                ++depth_estimate;
            }
            else
            {
                ++overlay_estimate;
            }
        }
    }
    if (_input_staging.empty())
    {
        return false;
    }
    const std::size_t upload_input_count = _input_staging.size();

    const std::size_t ring_slot = static_cast<std::size_t>(frame_index) % k_ring_slots;
    Slot &slot = _slots[ring_slot];
    if (!ensure_slot_resources(slot, upload_input_count, max_segments_gpu))
    {
        return false;
    }

    const auto upload_start = std::chrono::steady_clock::now();
    DeviceManager *dm = _context->getDevice();

    const std::size_t input_bytes = upload_input_count * sizeof(GenerateInputLine);
    std::memcpy(slot.input.info.pMappedData, _input_staging.data(), input_bytes);
    vmaFlushAllocation(dm->allocator(), slot.input.allocation, 0, input_bytes);

    IndirectPayload payload{};
    payload.depth.vertexCount = 6;
    payload.depth.instanceCount = 0;
    payload.depth.firstVertex = 0;
    payload.depth.firstInstance = 0;
    payload.overlay.vertexCount = 6;
    payload.overlay.instanceCount = 0;
    payload.overlay.firstVertex = 0;
    payload.overlay.firstInstance = 0;
    payload.total_count = 0;
    payload.cap_hit = 0;

    std::memcpy(slot.indirect.info.pMappedData, &payload, sizeof(payload));
    vmaFlushAllocation(dm->allocator(), slot.indirect.allocation, 0, sizeof(payload));

    const double upload_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - upload_start)
                                     .count();

    const VkBuffer input_buffer = slot.input.buffer;
    const VkBuffer depth_buffer = slot.depth_vertices.buffer;
    const VkBuffer overlay_buffer = slot.overlay_vertices.buffer;
    const VkBuffer indirect_buffer = slot.indirect.buffer;
    const std::size_t depth_buffer_size = slot.output_capacity_segments * 2ull * sizeof(OrbitPlotVertex);
    const std::size_t overlay_buffer_size = slot.output_capacity_segments * 2ull * sizeof(OrbitPlotVertex);
    const std::size_t indirect_buffer_size = sizeof(IndirectPayload);

    const float safe_error_px = static_cast<float>(std::clamp(
            (std::isfinite(plot.settings().render_error_px) && plot.settings().render_error_px > 0.0)
                ? plot.settings().render_error_px
                : 0.75,
            0.01,
            16.0));
    const float viewport_height_px = static_cast<float>(std::max(1u, _context->getDrawExtent().height));
    constexpr float k_dash_on_px = 14.0f;
    constexpr float k_dash_off_px = 9.0f;
    const float dash_period_px = std::max(1.0f, k_dash_on_px + k_dash_off_px);

    OrbitPlotGeneratePushConstants pc{};
    pc.camera_local_tan_half_fov = glm::vec4(camera_local, std::max(1.0e-6f, tan_half_fov));
    pc.subdivision_error_px = glm::vec4(safe_error_px, 0.0f, 0.0f, 0.0f);
    pc.viewport_height_min_dt = glm::vec4(viewport_height_px, k_min_subdivision_dt_s, 0.0f, 0.0f);
    pc.dash_on_off_period = glm::vec4(k_dash_on_px, k_dash_off_px, dash_period_px, 0.0f);
    pc.count_limits = glm::uvec4(
            static_cast<uint32_t>(
                    std::min<std::size_t>(upload_input_count, static_cast<std::size_t>(std::numeric_limits<uint32_t>::max()))),
            static_cast<uint32_t>(
                    std::min<std::size_t>(max_segments_gpu, static_cast<std::size_t>(std::numeric_limits<uint32_t>::max()))),
            k_max_subdivision_depth,
            0u);

    graph->add_pass(
            "OrbitPlot.Generate",
            RGPassType::Compute,
            [input_buffer,
             input_bytes,
             depth_buffer,
             depth_buffer_size,
             overlay_buffer,
             overlay_buffer_size,
             indirect_buffer,
             indirect_buffer_size](RGPassBuilder &builder, EngineContext *) {
                builder.read_buffer(input_buffer, RGBufferUsage::StorageRead, input_bytes, "orbit_plot.generate.input");
                builder.write_buffer(depth_buffer,
                                     RGBufferUsage::StorageReadWrite,
                                     depth_buffer_size,
                                     "orbit_plot.generate.depth_vertices");
                builder.write_buffer(overlay_buffer,
                                     RGBufferUsage::StorageReadWrite,
                                     overlay_buffer_size,
                                     "orbit_plot.generate.overlay_vertices");
                builder.write_buffer(indirect_buffer,
                                     RGBufferUsage::StorageReadWrite,
                                     indirect_buffer_size,
                                     "orbit_plot.generate.indirect");
            },
            [this,
             input_buffer,
             input_bytes,
             depth_buffer,
             depth_buffer_size,
             overlay_buffer,
             overlay_buffer_size,
             indirect_buffer,
             indirect_buffer_size,
             pc](VkCommandBuffer cmd, const RGPassResources &, EngineContext *ctx) {
                EngineContext *ctx_local = ctx ? ctx : _context;
                if (!ctx_local || !ctx_local->pipelines)
                {
                    return;
                }

                ctx_local->pipelines->setComputeInstanceBuffer(k_instance_name,
                                                               0,
                                                               input_buffer,
                                                               input_bytes,
                                                               VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                                               0);
                ctx_local->pipelines->setComputeInstanceBuffer(k_instance_name,
                                                               1,
                                                               depth_buffer,
                                                               depth_buffer_size,
                                                               VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                                               0);
                ctx_local->pipelines->setComputeInstanceBuffer(k_instance_name,
                                                               2,
                                                               overlay_buffer,
                                                               overlay_buffer_size,
                                                               VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                                               0);
                ctx_local->pipelines->setComputeInstanceBuffer(k_instance_name,
                                                               3,
                                                               indirect_buffer,
                                                               indirect_buffer_size,
                                                               VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                                               0);

                ComputeDispatchInfo di{};
                di.groupCountX = ComputeManager::calculateGroupCount(pc.count_limits.x, k_local_size_x);
                di.groupCountY = 1;
                di.groupCountZ = 1;
                di.pushConstants = &pc;
                di.pushConstantSize = sizeof(pc);
                ctx_local->pipelines->dispatchComputeInstance(cmd, k_instance_name, di);
            });

    out_frame.valid = true;
    out_frame.gpu_cap_hit = upload_input_count > max_segments_gpu;
    out_frame.upload_cap_hit = upload_cap_hit;
    out_frame.input_root_count = static_cast<uint32_t>(upload_input_count);
    out_frame.depth_segment_count_estimate = depth_estimate;
    out_frame.overlay_segment_count_estimate = overlay_estimate;
    out_frame.upload_bytes = input_bytes;
    out_frame.upload_budget_bytes = upload_budget_bytes;
    out_frame.upload_ms = upload_ms;
    out_frame.depth_vertex_buffer = depth_buffer;
    out_frame.overlay_vertex_buffer = overlay_buffer;
    out_frame.indirect_buffer = indirect_buffer;
    out_frame.depth_vertex_buffer_size = depth_buffer_size;
    out_frame.overlay_vertex_buffer_size = overlay_buffer_size;
    out_frame.indirect_buffer_size = indirect_buffer_size;
    return true;
}

bool OrbitPlotGenerate::ensure_slot_resources(Slot &slot,
                                              const std::size_t input_lines,
                                              const std::size_t max_segments_gpu)
{
    if (!_context || !_context->getResources())
    {
        return false;
    }

    ResourceManager *rm = _context->getResources();

    const std::size_t required_input_capacity =
            grow_capacity(input_lines, slot.input_capacity_lines, k_min_input_capacity_lines);
    if (slot.input.buffer == VK_NULL_HANDLE || slot.input.info.pMappedData == nullptr ||
        slot.input_capacity_lines < required_input_capacity)
    {
        if (slot.input.buffer != VK_NULL_HANDLE)
        {
            rm->destroy_buffer(slot.input);
            slot.input = {};
            slot.input_capacity_lines = 0;
        }

        const std::size_t input_bytes = required_input_capacity * sizeof(GenerateInputLine);
        slot.input = rm->create_buffer(
                input_bytes,
                VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                VMA_MEMORY_USAGE_CPU_TO_GPU);
        slot.input_capacity_lines = required_input_capacity;
    }

    const std::size_t required_output_capacity =
            grow_capacity(max_segments_gpu, slot.output_capacity_segments, k_min_output_capacity_segments);
    if (slot.depth_vertices.buffer == VK_NULL_HANDLE || slot.output_capacity_segments < required_output_capacity)
    {
        if (slot.depth_vertices.buffer != VK_NULL_HANDLE)
        {
            rm->destroy_buffer(slot.depth_vertices);
            slot.depth_vertices = {};
        }

        const std::size_t out_bytes = required_output_capacity * 2ull * sizeof(OrbitPlotVertex);
        slot.depth_vertices = rm->create_buffer(
                out_bytes,
                VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
                VMA_MEMORY_USAGE_GPU_ONLY);
    }

    if (slot.overlay_vertices.buffer == VK_NULL_HANDLE || slot.output_capacity_segments < required_output_capacity)
    {
        if (slot.overlay_vertices.buffer != VK_NULL_HANDLE)
        {
            rm->destroy_buffer(slot.overlay_vertices);
            slot.overlay_vertices = {};
        }

        const std::size_t out_bytes = required_output_capacity * 2ull * sizeof(OrbitPlotVertex);
        slot.overlay_vertices = rm->create_buffer(
                out_bytes,
                VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
                VMA_MEMORY_USAGE_GPU_ONLY);
    }

    slot.output_capacity_segments = required_output_capacity;

    if (slot.indirect.buffer == VK_NULL_HANDLE || slot.indirect.info.pMappedData == nullptr)
    {
        if (slot.indirect.buffer != VK_NULL_HANDLE)
        {
            rm->destroy_buffer(slot.indirect);
            slot.indirect = {};
        }

        slot.indirect = rm->create_buffer(
                sizeof(IndirectPayload),
                VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_INDIRECT_BUFFER_BIT,
                VMA_MEMORY_USAGE_CPU_TO_GPU);
    }

    if (slot.input.buffer == VK_NULL_HANDLE || slot.input.info.pMappedData == nullptr ||
        slot.depth_vertices.buffer == VK_NULL_HANDLE || slot.overlay_vertices.buffer == VK_NULL_HANDLE ||
        slot.indirect.buffer == VK_NULL_HANDLE || slot.indirect.info.pMappedData == nullptr)
    {
        destroy_slot_resources(slot);
        return false;
    }

    return true;
}

void OrbitPlotGenerate::destroy_slot_resources(Slot &slot)
{
    if (!_context || !_context->getResources())
    {
        slot = {};
        return;
    }

    ResourceManager *rm = _context->getResources();
    if (slot.input.buffer != VK_NULL_HANDLE)
    {
        rm->destroy_buffer(slot.input);
    }
    if (slot.depth_vertices.buffer != VK_NULL_HANDLE)
    {
        rm->destroy_buffer(slot.depth_vertices);
    }
    if (slot.overlay_vertices.buffer != VK_NULL_HANDLE)
    {
        rm->destroy_buffer(slot.overlay_vertices);
    }
    if (slot.indirect.buffer != VK_NULL_HANDLE)
    {
        rm->destroy_buffer(slot.indirect);
    }
    slot = {};
}
