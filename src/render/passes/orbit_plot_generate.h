#pragma once

#include <core/types.h>
#include <core/world.h>

#include <glm/vec4.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

class EngineContext;
class OrbitPlotSystem;
class RenderGraph;

class OrbitPlotGenerate
{
public:
    struct PreparedFrame
    {
        bool valid = false;
        bool gpu_cap_hit = false;
        bool upload_cap_hit = false;

        uint32_t input_root_count = 0;
        uint32_t depth_segment_count_estimate = 0;
        uint32_t overlay_segment_count_estimate = 0;

        std::size_t upload_bytes = 0;
        std::size_t upload_budget_bytes = 0;
        double upload_ms = 0.0;

        VkBuffer depth_vertex_buffer = VK_NULL_HANDLE;
        VkBuffer overlay_vertex_buffer = VK_NULL_HANDLE;
        VkBuffer indirect_buffer = VK_NULL_HANDLE;
        std::size_t depth_vertex_buffer_size = 0;
        std::size_t overlay_vertex_buffer_size = 0;
        std::size_t indirect_buffer_size = 0;
    };

    void init(EngineContext *context);
    void cleanup();

    bool can_generate() const { return _pipeline_ready; }

    bool prepare_and_register(RenderGraph *graph,
                              const OrbitPlotSystem &plot,
                              const WorldVec3 &origin_world,
                              uint32_t frame_index,
                              std::size_t max_segments_gpu,
                              PreparedFrame &out_frame);

private:
    static constexpr uint32_t k_local_size_x = 64;
    static constexpr std::size_t k_ring_slots = 3;

    struct GenerateInputLine
    {
        glm::vec4 anchor_dt{0.0f};
        glm::vec4 p0_rel_u0{0.0f};
        glm::vec4 p1_rel_u1{0.0f};
        glm::vec4 v0_depth{0.0f};
        glm::vec4 v1_dashed{0.0f};
        glm::vec4 dash_phase_dashed{0.0f};
        glm::vec4 color{1.0f};
    };
    static_assert(sizeof(GenerateInputLine) == 112);

    struct IndirectPayload
    {
        VkDrawIndirectCommand depth{};
        VkDrawIndirectCommand overlay{};
        uint32_t total_count = 0;
        uint32_t cap_hit = 0;
        uint32_t _pad0 = 0;
        uint32_t _pad1 = 0;
    };
    static_assert(sizeof(IndirectPayload) == 48);

    struct Slot
    {
        AllocatedBuffer input{};
        std::size_t input_capacity_lines = 0;

        AllocatedBuffer depth_vertices{};
        AllocatedBuffer overlay_vertices{};
        std::size_t output_capacity_segments = 0;

        AllocatedBuffer indirect{};
    };

    bool ensure_slot_resources(Slot &slot, std::size_t input_lines, std::size_t max_segments_gpu);
    void destroy_slot_resources(Slot &slot);

    EngineContext *_context = nullptr;
    bool _pipeline_ready = false;
    std::array<Slot, k_ring_slots> _slots{};
    std::vector<GenerateInputLine> _input_staging{};
};
