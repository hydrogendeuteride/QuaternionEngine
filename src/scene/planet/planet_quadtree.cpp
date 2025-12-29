#include "planet_quadtree.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

#include <glm/gtc/constants.hpp>

namespace planet
{
    namespace
    {
        struct Node
        {
            PatchKey key{};
        };

        bool is_patch_visible_horizon(const WorldVec3 &body_center_world,
                                      double radius_m,
                                      const WorldVec3 &camera_world,
                                      const glm::dvec3 &patch_center_dir,
                                      double patch_edge_m)
        {
            const glm::dvec3 w = camera_world - body_center_world;
            const double d = glm::length(w);
            if (d <= radius_m || d <= 0.0)
            {
                return true;
            }

            const glm::dvec3 w_dir = w / d;
            const double cos_theta = glm::dot(patch_center_dir, w_dir);

            // Horizon angle: cos(theta_h) = R / d
            const double cos_h = glm::clamp(radius_m / d, 0.0, 1.0);
            const double sin_h = std::sqrt(glm::max(0.0, 1.0 - cos_h * cos_h));

            // Expand horizon by patch angular radius to avoid culling near silhouettes.
            const double half_diag_m = patch_edge_m * 0.7071067811865476; // sqrt(2)/2
            const double ang = glm::clamp(half_diag_m / radius_m, 0.0, glm::pi<double>());
            const double cos_a = std::cos(ang);
            const double sin_a = std::sin(ang);

            // Visible if theta <= theta_h + ang:
            // cos(theta) >= cos(theta_h + ang)
            const double cos_limit = cos_h * cos_a - sin_h * sin_a;
            return cos_theta >= cos_limit;
        }

        bool is_patch_visible_frustum(const glm::vec3 &center_local, float bound_radius_m, const glm::mat4 &viewproj)
        {
            if (!(bound_radius_m > 0.0f))
            {
                bound_radius_m = 1.0f;
            }

            // Conservative AABB-in-clip test for a cube around the patch center.
            const std::array<glm::vec3, 8> corners{
                glm::vec3{+1, +1, +1}, glm::vec3{+1, +1, -1}, glm::vec3{+1, -1, +1}, glm::vec3{+1, -1, -1},
                glm::vec3{-1, +1, +1}, glm::vec3{-1, +1, -1}, glm::vec3{-1, -1, +1}, glm::vec3{-1, -1, -1},
            };

            glm::vec4 clip[8];
            for (int i = 0; i < 8; ++i)
            {
                const glm::vec3 p = center_local + corners[i] * bound_radius_m;
                clip[i] = viewproj * glm::vec4(p, 1.0f);
            }

            auto all_out = [&](auto pred) {
                for (int i = 0; i < 8; ++i)
                {
                    if (!pred(clip[i])) return false;
                }
                return true;
            };

            // Clip volume in Vulkan (ZO): -w<=x<=w, -w<=y<=w, 0<=z<=w
            if (all_out([](const glm::vec4 &v) { return v.x < -v.w; })) return false; // left
            if (all_out([](const glm::vec4 &v) { return v.x >  v.w; })) return false; // right
            if (all_out([](const glm::vec4 &v) { return v.y < -v.w; })) return false; // bottom
            if (all_out([](const glm::vec4 &v) { return v.y >  v.w; })) return false; // top
            if (all_out([](const glm::vec4 &v) { return v.z <  0.0f; })) return false; // near (ZO)
            if (all_out([](const glm::vec4 &v) { return v.z >  v.w; })) return false; // far

            return true;
        }
    } // namespace

    void PlanetQuadtree::update(const WorldVec3 &body_center_world,
                                double radius_m,
                                const WorldVec3 &camera_world,
                                const WorldVec3 &origin_world,
                                const GPUSceneData &scene_data,
                                VkExtent2D logical_extent)
    {
        _visible_leaves.clear();
        _stats = {};

        if (radius_m <= 0.0)
        {
            return;
        }

        if (logical_extent.width == 0 || logical_extent.height == 0)
        {
            logical_extent = VkExtent2D{1920, 1080};
        }

        const bool rt_shadows_enabled = (scene_data.rtOptions.x != 0u) && (scene_data.rtOptions.z != 0u);
        const double cam_alt_m = glm::max(0.0, glm::length(camera_world - body_center_world) - radius_m);
        const bool rt_guardrail_active =
            _settings.rt_guardrail &&
            rt_shadows_enabled &&
            (_settings.max_patch_edge_rt_m > 0.0) &&
            (cam_alt_m <= _settings.rt_guardrail_max_altitude_m);

        const float proj_y = scene_data.proj[1][1];
        const float proj_scale = std::abs(proj_y) * (static_cast<float>(logical_extent.height) * 0.5f);
        if (!(proj_scale > 0.0f))
        {
            return;
        }

        thread_local std::vector<Node> stack;
        stack.clear();
        stack.reserve(256);

        const size_t max_visible_leaves =
            (_settings.max_patches_visible > 0u)
                ? static_cast<size_t>(std::max(_settings.max_patches_visible, 6u))
                : std::numeric_limits<size_t>::max();

        auto push_root = [&](CubeFace face)
        {
            Node n{};
            n.key.face = face;
            n.key.level = 0;
            n.key.x = 0;
            n.key.y = 0;
            stack.push_back(n);
        };

        // Push in reverse order so pop_back visits in +X,-X,+Y,-Y,+Z,-Z order.
        push_root(CubeFace::NegZ);
        push_root(CubeFace::PosZ);
        push_root(CubeFace::NegY);
        push_root(CubeFace::PosY);
        push_root(CubeFace::NegX);
        push_root(CubeFace::PosX);

        while (!stack.empty())
        {
            Node n = stack.back();
            stack.pop_back();
            _stats.nodes_visited++;

            const PatchKey &k = n.key;

            const double patch_edge_m = cubesphere_patch_edge_m(radius_m, k.level);
            const glm::dvec3 patch_dir = cubesphere_patch_center_direction(k.face, k.level, k.x, k.y);

            if (_settings.horizon_cull)
            {
                if (!is_patch_visible_horizon(body_center_world, radius_m, camera_world, patch_dir, patch_edge_m))
                {
                    _stats.nodes_culled++;
                    continue;
                }
            }

            const WorldVec3 patch_center_world =
                body_center_world + patch_dir * radius_m;

            if (_settings.frustum_cull)
            {
                const glm::vec3 patch_center_local = world_to_local(patch_center_world, origin_world);
                const float bound_r = static_cast<float>(patch_edge_m * 0.7071067811865476);
                if (!is_patch_visible_frustum(patch_center_local, bound_r, scene_data.viewproj))
                {
                    _stats.nodes_culled++;
                    continue;
                }
            }

            const double dist_m = glm::max(1.0, glm::length(camera_world - patch_center_world));

            // Screen-space error metric.
            const double error_m = 0.5 * patch_edge_m;
            const float sse_px = static_cast<float>((error_m / dist_m) * static_cast<double>(proj_scale));

            bool refine = (k.level < _settings.max_level) && (sse_px > _settings.target_sse_px);
            if (!refine && rt_guardrail_active && (k.level < _settings.max_level) && (patch_edge_m > _settings.max_patch_edge_rt_m))
            {
                refine = true;
            }

            if (refine)
            {
                // Budget check: splitting replaces this node with 4 children (adds +3 leaves minimum).
                // Keep a stable upper bound on the final leaf count: leaves_so_far + stack.size() + 4.
                const size_t min_leaves_if_split = _visible_leaves.size() + stack.size() + 4u;
                if (min_leaves_if_split > max_visible_leaves)
                {
                    refine = false;
                    _stats.splits_budget_limited++;
                }
            }

            if (refine)
            {
                // Child order: (0,0), (1,0), (0,1), (1,1) with y increasing downward.
                const uint32_t cl = k.level + 1u;
                const uint32_t cx = k.x * 2u;
                const uint32_t cy = k.y * 2u;

                stack.push_back(Node{PatchKey{k.face, cl, cx + 1u, cy + 1u}});
                stack.push_back(Node{PatchKey{k.face, cl, cx + 0u, cy + 1u}});
                stack.push_back(Node{PatchKey{k.face, cl, cx + 1u, cy + 0u}});
                stack.push_back(Node{PatchKey{k.face, cl, cx + 0u, cy + 0u}});
                continue;
            }

            _visible_leaves.push_back(k);
            _stats.max_level_used = std::max(_stats.max_level_used, k.level);
        }

        _stats.visible_leaves = static_cast<uint32_t>(_visible_leaves.size());

        // Keep deterministic order for stability (optional).
        // DFS already stable; sort is useful when culling changes traversal.
        std::sort(_visible_leaves.begin(), _visible_leaves.end(),
                  [](const PatchKey &a, const PatchKey &b)
                  {
                      if (a.face != b.face) return a.face < b.face;
                      if (a.level != b.level) return a.level < b.level;
                      if (a.x != b.x) return a.x < b.x;
                      return a.y < b.y;
                  });
    }
} // namespace planet
