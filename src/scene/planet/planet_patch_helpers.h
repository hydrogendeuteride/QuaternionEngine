#pragma once

#include <core/types.h>
#include <scene/planet/planet_heightmap.h>
#include <scene/planet/planet_quadtree.h>
#include <scene/planet/cubesphere.h>
#include <render/materials.h>

#include <array>
#include <cstdint>
#include <unordered_set>
#include <vector>

// Internal helpers shared across planet .cpp files. Not part of the public API.

namespace planet_helpers
{
    struct PatchBoundsData
    {
        glm::vec3 origin{0.0f};
        glm::vec3 extents{0.5f};
        float sphere_radius{0.5f};
    };

    PatchBoundsData compute_patch_bounds(const std::vector<Vertex> &vertices);

    GLTFMetallic_Roughness::MaterialConstants make_planet_constants(
        const glm::vec4 &base_color = glm::vec4(1.0f),
        float metallic = 0.0f,
        float roughness = 1.0f,
        const glm::vec3 &emission_factor = glm::vec3(0.0f));

    glm::vec4 debug_color_for_level(uint32_t level);

    void recompute_patch_normals(std::vector<Vertex> &vertices, uint32_t resolution);

    void refine_patch_edge_normals_from_height(std::vector<Vertex> &vertices,
                                               uint32_t resolution,
                                               const glm::dvec3 &patch_center_dir,
                                               double radius_m,
                                               uint32_t level,
                                               uint8_t edge_stitch_mask,
                                               double height_scale_m,
                                               const std::array<planet::HeightFace, 6> &height_faces);

    void reinforce_patch_skirts(std::vector<Vertex> &vertices,
                                uint32_t resolution,
                                const glm::dvec3 &patch_center_dir,
                                double radius_m,
                                uint32_t level);

    void stitch_patch_edges_to_parent_grid(std::vector<Vertex> &vertices,
                                           uint32_t resolution,
                                           uint8_t edge_mask);

    uint8_t compute_patch_edge_stitch_mask(const planet::PatchKey &key,
                                           const std::unordered_set<planet::PatchKey, planet::PatchKeyHash> &leaf_set,
                                           uint32_t max_level_in_set);
} // namespace planet_helpers
