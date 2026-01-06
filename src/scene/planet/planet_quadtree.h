#pragma once

#include "cubesphere.h"

#include <core/types.h>
#include <core/world.h>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace planet
{
    struct PatchKey
    {
        CubeFace face = CubeFace::PosX;
        uint32_t level = 0;
        uint32_t x = 0;
        uint32_t y = 0;

        friend bool operator==(const PatchKey &, const PatchKey &) = default;
    };

    struct PatchKeyHash
    {
        size_t operator()(const PatchKey &k) const noexcept
        {
            const uint64_t f = static_cast<uint64_t>(k.face) & 0xFFull;
            const uint64_t l = static_cast<uint64_t>(k.level) & 0x3Full;
            const uint64_t x = static_cast<uint64_t>(k.x) & 0x1FFFFFull;
            const uint64_t y = static_cast<uint64_t>(k.y) & 0x1FFFFFull;

            // Simple stable packing: [face:8 | level:6 | x:21 | y:21]
            const uint64_t packed = (f << 56) | (l << 50) | (x << 29) | (y << 8);
            return std::hash<uint64_t>{}(packed);
        }
    };

    class PlanetQuadtree
    {
    public:
        struct Settings
        {
            uint32_t max_level = 14;
            float target_sse_px = 32.0f;    // screen space error pixel
            uint32_t max_patches_visible = 8192;
            bool frustum_cull = true;
            bool horizon_cull = true;

            // RT stability guardrail (only applied near-surface).
            bool rt_guardrail = false;  //ALWAYS DISABLE IT
            double max_patch_edge_rt_m = 5000.0;
            double rt_guardrail_max_altitude_m = 200000.0;
        };

        struct Stats
        {
            uint32_t visible_leaves = 0;
            uint32_t max_level_used = 0;
            uint32_t nodes_visited = 0;
            uint32_t nodes_culled = 0;
            uint32_t splits_budget_limited = 0;
        };

        void set_settings(const Settings &settings) { _settings = settings; }
        const Settings &settings() const { return _settings; }
        const Stats &stats() const { return _stats; }
        const std::vector<PatchKey> &visible_leaves() const { return _visible_leaves; }

        void update(const WorldVec3 &body_center_world,
                    double radius_m,
                    double max_height_m,
                    const WorldVec3 &camera_world,
                    const WorldVec3 &origin_world,
                    const GPUSceneData &scene_data,
                    VkExtent2D logical_extent,
                    uint32_t patch_resolution);

    private:
        Settings _settings{};
        Stats _stats{};
        std::vector<PatchKey> _visible_leaves;
    };
} // namespace planet
