#include "planet_system.h"

#include <core/context.h>
#include <core/device/resource.h>
#include <core/frame/resources.h>
#include <core/types.h>
#include <core/assets/manager.h>
#include <render/materials.h>
#include <render/primitives.h>
#include <core/pipeline/sampler.h>
#include <scene/planet/cubesphere.h>
#include <scene/planet/planet_heightmap.h>
#include <scene/tangent_space.h>
#include <scene/vk_scene.h>

#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <span>
#include <unordered_set>

#include "device.h"

namespace
{
    struct PatchBoundsData
    {
        glm::vec3 origin{0.0f};
        glm::vec3 extents{0.5f};
        float sphere_radius{0.5f};
    };

    PatchBoundsData compute_patch_bounds(const std::vector<Vertex> &vertices)
    {
        PatchBoundsData b{};
        if (vertices.empty())
        {
            return b;
        }

        glm::vec3 minpos = vertices[0].position;
        glm::vec3 maxpos = vertices[0].position;
        for (const auto &v: vertices)
        {
            minpos = glm::min(minpos, v.position);
            maxpos = glm::max(maxpos, v.position);
        }
        b.origin = (maxpos + minpos) * 0.5f;
        b.extents = (maxpos - minpos) * 0.5f;
        b.sphere_radius = glm::length(b.extents);
        return b;
    }

    GLTFMetallic_Roughness::MaterialConstants make_planet_constants(
        const glm::vec4 &base_color = glm::vec4(1.0f),
        float metallic = 0.0f,
        float roughness = 1.0f,
        const glm::vec3 &emission_factor = glm::vec3(0.0f))
    {
        GLTFMetallic_Roughness::MaterialConstants c{};
        c.colorFactors = base_color;
        c.metal_rough_factors = glm::vec4(metallic, roughness, 0.0f, 0.0f);
        // extra[1].rgb = emissive factor (sampled in mesh.frag)
        c.extra[1] = glm::vec4(emission_factor, 0.0f);
        // Mark planet materials so the deferred lighting pass can apply a special
        // shadowing path when RT-only shadows are enabled (avoid relying on TLAS
        // intersections with planet geometry).
        // Convention: extra[2].y > 0 => "force clipmap (shadow map) receiver"
        c.extra[2].y = 1.0f;
        return c;
    }

    glm::vec4 debug_color_for_level(uint32_t level)
    {
        const float t = static_cast<float>(level) * 0.37f;
        const float r = 0.35f + 0.65f * std::sin(t + 0.0f);
        const float g = 0.35f + 0.65f * std::sin(t + 2.1f);
        const float b = 0.35f + 0.65f * std::sin(t + 4.2f);
        return glm::vec4(r, g, b, 1.0f);
    }

    void recompute_patch_normals(std::vector<Vertex> &vertices, uint32_t resolution)
    {
        const uint32_t res = std::max(2u, resolution);
        const uint32_t base_count = res * res;
        if (vertices.size() < base_count)
        {
            return;
        }

        thread_local std::vector<glm::vec3> scratch_normals;
        scratch_normals.resize(static_cast<size_t>(base_count));

        // Blend width: boundary vertices (dist=0) keep radial normals, and the
        // next kBlendWidth rows gradually transition to fully computed normals.
        // This avoids a hard lighting discontinuity at patch edges.
        constexpr uint32_t kBlendWidth = 2u;

        for (uint32_t j = 0; j < res; ++j)
        {
            const uint32_t ju = (j > 0u) ? (j - 1u) : j;
            const uint32_t jd = (j + 1u < res) ? (j + 1u) : j;

            for (uint32_t i = 0; i < res; ++i)
            {
                const uint32_t il = (i > 0u) ? (i - 1u) : i;
                const uint32_t ir = (i + 1u < res) ? (i + 1u) : i;

                const uint32_t idx = j * res + i;
                const uint32_t dist_to_edge = std::min({i, res - 1u - i, j, res - 1u - j});

                if (dist_to_edge == 0u)
                {
                    // Keep radial normals on patch boundaries to avoid visible seams between
                    // neighboring patches/faces that do not share derivative samples.
                    scratch_normals[idx] = vertices[idx].normal;
                    continue;
                }

                const glm::vec3 pL = vertices[j * res + il].position;
                const glm::vec3 pR = vertices[j * res + ir].position;
                const glm::vec3 pU = vertices[ju * res + i].position;
                const glm::vec3 pD = vertices[jd * res + i].position;

                const glm::vec3 dx = pR - pL;
                const glm::vec3 dy = pD - pU;
                glm::vec3 n = glm::cross(dy, dx);
                const float len2 = glm::dot(n, n);
                if (len2 > 1e-12f)
                {
                    n *= (1.0f / std::sqrt(len2));
                }
                else
                {
                    n = vertices[idx].normal;
                }

                // Ensure outward orientation.
                if (glm::dot(n, vertices[idx].normal) < 0.0f)
                {
                    n = -n;
                }

                // Gradually blend from radial normal to computed normal near edges.
                if (dist_to_edge <= kBlendWidth)
                {
                    const float t = static_cast<float>(dist_to_edge) / static_cast<float>(kBlendWidth + 1u);
                    n = glm::normalize(glm::mix(vertices[idx].normal, n, t));
                }

                scratch_normals[idx] = n;
            }
        }

        for (uint32_t idx = 0; idx < base_count; ++idx)
        {
            vertices[idx].normal = scratch_normals[idx];
        }

        const uint32_t skirt_count = 4u * res;
        if (vertices.size() < base_count + skirt_count)
        {
            return;
        }

        const uint32_t top_skirt_start = base_count + 0u * res;
        const uint32_t right_skirt_start = base_count + 1u * res;
        const uint32_t bottom_skirt_start = base_count + 2u * res;
        const uint32_t left_skirt_start = base_count + 3u * res;

        // Top edge (j=0)
        for (uint32_t i = 0; i < res; ++i)
        {
            vertices[top_skirt_start + i].normal = vertices[0u * res + i].normal;
        }
        // Right edge (i=res-1)
        for (uint32_t j = 0; j < res; ++j)
        {
            vertices[right_skirt_start + j].normal = vertices[j * res + (res - 1u)].normal;
        }
        // Bottom edge (j=res-1)
        for (uint32_t i = 0; i < res; ++i)
        {
            vertices[bottom_skirt_start + i].normal = vertices[(res - 1u) * res + i].normal;
        }
        // Left edge (i=0)
        for (uint32_t j = 0; j < res; ++j)
        {
            vertices[left_skirt_start + j].normal = vertices[j * res + 0u].normal;
        }
    }

    float sample_height_from_direction(const std::array<planet::HeightFace, 6> &height_faces, const glm::dvec3 &dir)
    {
        planet::CubeFace face = planet::CubeFace::PosX;
        double u01 = 0.0;
        double v01 = 0.0;
        if (!planet::cubesphere_direction_to_face_uv(dir, face, u01, v01))
        {
            return 0.0f;
        }

        const uint32_t face_index = static_cast<uint32_t>(face);
        if (face_index >= height_faces.size())
        {
            return 0.0f;
        }

        const planet::HeightFace &hf = height_faces[face_index];
        if (hf.width == 0 || hf.height == 0 || hf.texels.empty())
        {
            return 0.0f;
        }

        return planet::sample_height(hf, static_cast<float>(u01), static_cast<float>(v01));
    }

    glm::dvec3 sample_surface_position_from_direction(const std::array<planet::HeightFace, 6> &height_faces,
                                                      const glm::dvec3 &dir,
                                                      double radius_m,
                                                      double height_scale_m)
    {
        glm::dvec3 d = dir;
        const double len2 = glm::dot(d, d);
        if (!(len2 > 0.0))
        {
            d = glm::dvec3(0.0, 0.0, 1.0);
        }
        else
        {
            d *= (1.0 / std::sqrt(len2));
        }

        const double h = static_cast<double>(sample_height_from_direction(height_faces, d)) * height_scale_m;
        return d * (radius_m + h);
    }

    void refine_patch_edge_normals_from_height(std::vector<Vertex> &vertices,
                                               uint32_t resolution,
                                               const glm::dvec3 &patch_center_dir,
                                               double radius_m,
                                               uint32_t level,
                                               uint8_t edge_stitch_mask,
                                               double height_scale_m,
                                               const std::array<planet::HeightFace, 6> &height_faces)
    {
        constexpr uint8_t kStitchedTop = 1u << 0u;
        constexpr uint8_t kStitchedRight = 1u << 1u;
        constexpr uint8_t kStitchedBottom = 1u << 2u;
        constexpr uint8_t kStitchedLeft = 1u << 3u;

        if (!(height_scale_m > 0.0) || !(radius_m > 0.0))
        {
            return;
        }

        bool has_height = false;
        for (const planet::HeightFace &hf: height_faces)
        {
            if (hf.width > 0 && hf.height > 0 && !hf.texels.empty())
            {
                has_height = true;
                break;
            }
        }
        if (!has_height)
        {
            return;
        }

        const uint32_t res = std::max(2u, resolution);
        const uint32_t base_count = res * res;
        if (vertices.size() < base_count)
        {
            return;
        }

        const double edge_m = planet::cubesphere_patch_edge_m(radius_m, level);
        const double seg_m = edge_m / static_cast<double>(res - 1u);
        if (!(seg_m > 0.0))
        {
            return;
        }

        constexpr uint32_t kRefineWidth = 2u;
        const double angle = glm::clamp(seg_m / radius_m, 1e-6, 5e-3);
        // Stitched edges border a coarser patch (2:1 LOD). Sample normals there with
        // coarse spacing so both sides converge to nearly identical edge normals.
        const double stitched_angle = glm::clamp((seg_m * 2.0) / radius_m, 1e-6, 5e-3);

        for (uint32_t j = 0; j < res; ++j)
        {
            for (uint32_t i = 0; i < res; ++i)
            {
                const uint32_t dist_to_edge = std::min({i, res - 1u - i, j, res - 1u - j});
                if (dist_to_edge > kRefineWidth)
                {
                    continue;
                }

                const uint32_t idx = j * res + i;
                const bool on_stitched_top = (j == 0u) && ((edge_stitch_mask & kStitchedTop) != 0u);
                const bool on_stitched_right = (i == res - 1u) && ((edge_stitch_mask & kStitchedRight) != 0u);
                const bool on_stitched_bottom = (j == res - 1u) && ((edge_stitch_mask & kStitchedBottom) != 0u);
                const bool on_stitched_left = (i == 0u) && ((edge_stitch_mask & kStitchedLeft) != 0u);
                const bool on_stitched_edge =
                        on_stitched_top || on_stitched_right || on_stitched_bottom || on_stitched_left;
                const double sample_angle = on_stitched_edge ? stitched_angle : angle;
                const glm::vec3 p_local = vertices[idx].position;
                glm::dvec3 dir = patch_center_dir * radius_m +
                                 glm::dvec3(static_cast<double>(p_local.x),
                                            static_cast<double>(p_local.y),
                                            static_cast<double>(p_local.z));
                const double len2 = glm::dot(dir, dir);
                if (!(len2 > 0.0))
                {
                    continue;
                }
                dir *= (1.0 / std::sqrt(len2));

                const glm::dvec3 up = (std::abs(dir.y) < 0.95) ? glm::dvec3(0.0, 1.0, 0.0) : glm::dvec3(1.0, 0.0, 0.0);
                glm::dvec3 t = glm::cross(up, dir);
                const double t_len2 = glm::dot(t, t);
                if (!(t_len2 > 1e-20))
                {
                    continue;
                }
                t *= (1.0 / std::sqrt(t_len2));
                glm::dvec3 b = glm::cross(dir, t);
                const double b_len2 = glm::dot(b, b);
                if (!(b_len2 > 1e-20))
                {
                    continue;
                }
                b *= (1.0 / std::sqrt(b_len2));

                const glm::dvec3 pL = sample_surface_position_from_direction(height_faces, dir - t * sample_angle, radius_m, height_scale_m);
                const glm::dvec3 pR = sample_surface_position_from_direction(height_faces, dir + t * sample_angle, radius_m, height_scale_m);
                const glm::dvec3 pU = sample_surface_position_from_direction(height_faces, dir - b * sample_angle, radius_m, height_scale_m);
                const glm::dvec3 pD = sample_surface_position_from_direction(height_faces, dir + b * sample_angle, radius_m, height_scale_m);

                glm::dvec3 n = glm::cross(pD - pU, pR - pL);
                const double n_len2 = glm::dot(n, n);
                if (!(n_len2 > 1e-20))
                {
                    continue;
                }
                n *= (1.0 / std::sqrt(n_len2));
                if (glm::dot(n, dir) < 0.0)
                {
                    n = -n;
                }

                const float t_blend =
                        1.0f - static_cast<float>(dist_to_edge) / static_cast<float>(kRefineWidth + 1u);
                const glm::vec3 refined_n(static_cast<float>(n.x), static_cast<float>(n.y), static_cast<float>(n.z));
                vertices[idx].normal = glm::normalize(glm::mix(vertices[idx].normal, refined_n, t_blend));
            }
        }

        // Keep stitched edge midpoints consistent with parent-grid normals.
        auto restitch_edge_normal = [&](uint32_t center, uint32_t prev, uint32_t next) {
            const glm::vec3 blended = 0.5f * (vertices[prev].normal + vertices[next].normal);
            const float len2 = glm::dot(blended, blended);
            if (len2 > 1e-12f)
            {
                // Keep midpoint normals unnormalized so coarse/fine LOD edges interpolate identically.
                vertices[center].normal = blended;
            }
        };

        if ((edge_stitch_mask & kStitchedTop) != 0u && res >= 3u)
        {
            for (uint32_t i = 1u; i + 1u < res; i += 2u)
            {
                const uint32_t c = 0u * res + i;
                restitch_edge_normal(c, c - 1u, c + 1u);
            }
        }
        if ((edge_stitch_mask & kStitchedBottom) != 0u && res >= 3u)
        {
            const uint32_t row = res - 1u;
            for (uint32_t i = 1u; i + 1u < res; i += 2u)
            {
                const uint32_t c = row * res + i;
                restitch_edge_normal(c, c - 1u, c + 1u);
            }
        }
        if ((edge_stitch_mask & kStitchedLeft) != 0u && res >= 3u)
        {
            for (uint32_t j = 1u; j + 1u < res; j += 2u)
            {
                const uint32_t c = j * res + 0u;
                restitch_edge_normal(c, c - res, c + res);
            }
        }
        if ((edge_stitch_mask & kStitchedRight) != 0u && res >= 3u)
        {
            const uint32_t col = res - 1u;
            for (uint32_t j = 1u; j + 1u < res; j += 2u)
            {
                const uint32_t c = j * res + col;
                restitch_edge_normal(c, c - res, c + res);
            }
        }

        const uint32_t skirt_count = 4u * res;
        if (vertices.size() < base_count + skirt_count)
        {
            return;
        }

        const uint32_t top_skirt_start = base_count + 0u * res;
        const uint32_t right_skirt_start = base_count + 1u * res;
        const uint32_t bottom_skirt_start = base_count + 2u * res;
        const uint32_t left_skirt_start = base_count + 3u * res;

        for (uint32_t ii = 0; ii < res; ++ii)
        {
            vertices[top_skirt_start + ii].normal = vertices[0u * res + ii].normal;
        }
        for (uint32_t jj = 0; jj < res; ++jj)
        {
            vertices[right_skirt_start + jj].normal = vertices[jj * res + (res - 1u)].normal;
        }
        for (uint32_t ii = 0; ii < res; ++ii)
        {
            vertices[bottom_skirt_start + ii].normal = vertices[(res - 1u) * res + ii].normal;
        }
        for (uint32_t jj = 0; jj < res; ++jj)
        {
            vertices[left_skirt_start + jj].normal = vertices[jj * res + 0u].normal;
        }
    }

    void reinforce_patch_skirts(std::vector<Vertex> &vertices,
                                uint32_t resolution,
                                const glm::dvec3 &patch_center_dir,
                                double radius_m,
                                uint32_t level)
    {
        const uint32_t res = std::max(2u, resolution);
        const uint32_t base_count = res * res;
        const uint32_t skirt_count = 4u * res;
        if (vertices.size() < base_count + skirt_count || !(radius_m > 0.0))
        {
            return;
        }

        const uint32_t top_skirt_start = base_count + 0u * res;
        const uint32_t right_skirt_start = base_count + 1u * res;
        const uint32_t bottom_skirt_start = base_count + 2u * res;
        const uint32_t left_skirt_start = base_count + 3u * res;

        const double edge_m = planet::cubesphere_patch_edge_m(radius_m, level);
        const double base_depth = planet::cubesphere_skirt_depth_m(radius_m, level);
        const double min_depth = std::max(base_depth * 2.5, edge_m * 0.06);
        const double max_depth = std::max(min_depth * 4.0, min_depth + 1500.0);

        auto absolute_from_local = [&](const glm::vec3 &p_local) -> glm::dvec3 {
            return patch_center_dir * radius_m +
                   glm::dvec3(static_cast<double>(p_local.x),
                              static_cast<double>(p_local.y),
                              static_cast<double>(p_local.z));
        };

        auto edge_depth_m = [&](uint32_t edge_index, uint32_t inner_index) -> double {
            const glm::dvec3 p_edge = absolute_from_local(vertices[edge_index].position);
            const glm::dvec3 p_inner = absolute_from_local(vertices[inner_index].position);
            const double r_edge = glm::length(p_edge);
            const double r_inner = glm::length(p_inner);
            const double radial_delta = std::abs(r_edge - r_inner);
            const double depth = min_depth + radial_delta * 3.0;
            return glm::clamp(depth, min_depth, max_depth);
        };

        auto place_skirt_vertex = [&](uint32_t edge_index, uint32_t inner_index, uint32_t skirt_index) {
            const glm::dvec3 edge_abs = absolute_from_local(vertices[edge_index].position);
            const double len2 = glm::dot(edge_abs, edge_abs);
            if (!(len2 > 0.0))
            {
                return;
            }

            const glm::dvec3 dir = edge_abs * (1.0 / std::sqrt(len2));
            const double depth = edge_depth_m(edge_index, inner_index);
            const glm::dvec3 skirt_abs = edge_abs - dir * depth;
            const glm::dvec3 skirt_local = skirt_abs - patch_center_dir * radius_m;

            vertices[skirt_index] = vertices[edge_index];
            vertices[skirt_index].position = glm::vec3(static_cast<float>(skirt_local.x),
                                                       static_cast<float>(skirt_local.y),
                                                       static_cast<float>(skirt_local.z));
            vertices[skirt_index].normal = vertices[edge_index].normal;
        };

        for (uint32_t i = 0; i < res; ++i)
        {
            const uint32_t top_edge = 0u * res + i;
            const uint32_t top_inner = ((res > 1u) ? 1u : 0u) * res + i;
            place_skirt_vertex(top_edge, top_inner, top_skirt_start + i);

            const uint32_t bottom_row = (res > 1u) ? (res - 1u) : 0u;
            const uint32_t bottom_inner_row = (res > 1u) ? (res - 2u) : bottom_row;
            const uint32_t bottom_edge = bottom_row * res + i;
            const uint32_t bottom_inner = bottom_inner_row * res + i;
            place_skirt_vertex(bottom_edge, bottom_inner, bottom_skirt_start + i);
        }

        for (uint32_t j = 0; j < res; ++j)
        {
            const uint32_t left_edge = j * res + 0u;
            const uint32_t left_inner = j * res + ((res > 1u) ? 1u : 0u);
            place_skirt_vertex(left_edge, left_inner, left_skirt_start + j);

            const uint32_t right_col = (res > 1u) ? (res - 1u) : 0u;
            const uint32_t right_inner_col = (res > 1u) ? (res - 2u) : right_col;
            const uint32_t right_edge = j * res + right_col;
            const uint32_t right_inner = j * res + right_inner_col;
            place_skirt_vertex(right_edge, right_inner, right_skirt_start + j);
        }
    }

    constexpr uint8_t kEdgeTop = 1u << 0u;
    constexpr uint8_t kEdgeRight = 1u << 1u;
    constexpr uint8_t kEdgeBottom = 1u << 2u;
    constexpr uint8_t kEdgeLeft = 1u << 3u;

    bool find_leaf_containing(const std::unordered_set<planet::PatchKey, planet::PatchKeyHash> &leaf_set,
                              planet::CubeFace face,
                              double u01,
                              double v01,
                              uint32_t max_level,
                              planet::PatchKey &out_key)
    {
        const double uu = glm::clamp(u01, 0.0, std::nextafter(1.0, 0.0));
        const double vv = glm::clamp(v01, 0.0, std::nextafter(1.0, 0.0));

        for (int32_t level = static_cast<int32_t>(max_level); level >= 0; --level)
        {
            const uint32_t l = static_cast<uint32_t>(level);
            const uint32_t tiles = (l < 31u) ? (1u << l) : 0u;
            if (tiles == 0u)
            {
                continue;
            }

            const uint32_t xi = std::min(tiles - 1u, static_cast<uint32_t>(uu * static_cast<double>(tiles)));
            const uint32_t yi = std::min(tiles - 1u, static_cast<uint32_t>(vv * static_cast<double>(tiles)));

            const planet::PatchKey key{face, l, xi, yi};
            if (leaf_set.contains(key))
            {
                out_key = key;
                return true;
            }
        }

        return false;
    }

    int32_t sample_neighbor_level_across_edge(const planet::PatchKey &key,
                                              double u_face,
                                              double v_face,
                                              const std::unordered_set<planet::PatchKey, planet::PatchKeyHash> &leaf_set,
                                              uint32_t max_level_in_set)
    {
        const glm::dvec3 dir = planet::cubesphere_unit_direction(key.face, u_face, v_face);
        planet::CubeFace sample_face = planet::CubeFace::PosX;
        double sample_u01 = 0.0;
        double sample_v01 = 0.0;
        if (!planet::cubesphere_direction_to_face_uv(dir, sample_face, sample_u01, sample_v01))
        {
            return -1;
        }

        planet::PatchKey neighbor{};
        if (!find_leaf_containing(leaf_set, sample_face, sample_u01, sample_v01, max_level_in_set, neighbor))
        {
            return -1;
        }

        return static_cast<int32_t>(neighbor.level);
    }

    uint8_t compute_patch_edge_stitch_mask(const planet::PatchKey &key,
                                           const std::unordered_set<planet::PatchKey, planet::PatchKeyHash> &leaf_set,
                                           uint32_t max_level_in_set)
    {
        if (key.level == 0u)
        {
            return 0u;
        }

        double u0 = 0.0, u1 = 0.0, v0 = 0.0, v1 = 0.0;
        planet::cubesphere_tile_uv_bounds(key.level, key.x, key.y, u0, u1, v0, v1);

        const double du = std::abs(u1 - u0);
        const double dv = std::abs(v1 - v0);
        const double eps_u = glm::max(1e-9, du * 5e-4);
        const double eps_v = glm::max(1e-9, dv * 5e-4);
        constexpr std::array<double, 3> samples{0.25, 0.5, 0.75};

        auto edge_has_coarser_neighbor = [&](double sample_u, double sample_v, bool horizontal) -> bool {
            for (const double t: samples)
            {
                const double uf = horizontal ? glm::mix(u0, u1, t) : sample_u;
                const double vf = horizontal ? sample_v : glm::mix(v0, v1, t);
                const int32_t level = sample_neighbor_level_across_edge(key, uf, vf, leaf_set, max_level_in_set);
                if (level >= 0 && level < static_cast<int32_t>(key.level))
                {
                    return true;
                }
            }
            return false;
        };

        uint8_t mask = 0u;
        if (edge_has_coarser_neighbor(0.0, v0 - eps_v, true))
        {
            mask |= kEdgeTop;
        }
        if (edge_has_coarser_neighbor(u1 + eps_u, 0.0, false))
        {
            mask |= kEdgeRight;
        }
        if (edge_has_coarser_neighbor(0.0, v1 + eps_v, true))
        {
            mask |= kEdgeBottom;
        }
        if (edge_has_coarser_neighbor(u0 - eps_u, 0.0, false))
        {
            mask |= kEdgeLeft;
        }
        return mask;
    }

    void stitch_patch_edges_to_parent_grid(std::vector<Vertex> &vertices, uint32_t resolution, uint8_t edge_mask)
    {
        if (edge_mask == 0u)
        {
            return;
        }

        const uint32_t res = std::max(2u, resolution);
        const uint32_t base_count = res * res;
        if (vertices.size() < base_count || res < 3u)
        {
            return;
        }

        auto blend_edge_vertex = [&](uint32_t center, uint32_t prev, uint32_t next) {
            vertices[center].position = 0.5f * (vertices[prev].position + vertices[next].position);
            const glm::vec3 blended = 0.5f * (vertices[prev].normal + vertices[next].normal);
            const float len2 = glm::dot(blended, blended);
            // Keep midpoint normals unnormalized so interpolation across 2:1 stitched edges matches
            // the coarser edge (shaders normalize per-fragment).
            vertices[center].normal = (len2 > 1e-12f) ? blended : vertices[prev].normal;
        };

        if ((edge_mask & kEdgeTop) != 0u)
        {
            for (uint32_t i = 1u; i + 1u < res; i += 2u)
            {
                const uint32_t c = 0u * res + i;
                blend_edge_vertex(c, c - 1u, c + 1u);
            }
        }

        if ((edge_mask & kEdgeBottom) != 0u)
        {
            const uint32_t row = res - 1u;
            for (uint32_t i = 1u; i + 1u < res; i += 2u)
            {
                const uint32_t c = row * res + i;
                blend_edge_vertex(c, c - 1u, c + 1u);
            }
        }

        if ((edge_mask & kEdgeLeft) != 0u)
        {
            for (uint32_t j = 1u; j + 1u < res; j += 2u)
            {
                const uint32_t c = j * res + 0u;
                blend_edge_vertex(c, c - res, c + res);
            }
        }

        if ((edge_mask & kEdgeRight) != 0u)
        {
            const uint32_t col = res - 1u;
            for (uint32_t j = 1u; j + 1u < res; j += 2u)
            {
                const uint32_t c = j * res + col;
                blend_edge_vertex(c, c - res, c + res);
            }
        }
    }
} // namespace

void PlanetSystem::init(EngineContext *context)
{
    _context = context;
}

void PlanetSystem::set_earth_debug_tint_patches_by_lod(bool enabled)
{
    if (_earth_debug_tint_patches_by_lod == enabled)
    {
        return;
    }
    _earth_debug_tint_patches_by_lod = enabled;
    _earth_patch_cache_dirty = true;
}

void PlanetSystem::set_earth_patch_resolution(uint32_t resolution)
{
    const uint32_t clamped = std::max(2u, resolution);
    if (_earth_patch_resolution == clamped)
    {
        return;
    }

    _earth_patch_resolution = clamped;
    _earth_patch_cache_dirty = true;
}

void PlanetSystem::cleanup()
{
    if (!_context)
    {
        return;
    }

    TextureCache *textures = _context->textures;
    if (textures)
    {
        for (auto &kv: _terrain_states)
        {
            TerrainState &state = *kv.second;
            for (MaterialInstance &mat: state.face_materials)
            {
                if (mat.materialSet != VK_NULL_HANDLE)
                {
                    textures->unwatchSet(mat.materialSet);
                }
            }
        }
    }

    ResourceManager *rm = _context->getResources();
    if (rm)
    {
        for (auto &kv: _terrain_states)
        {
            TerrainState &state = *kv.second;
            for (TerrainPatch &p: state.patches)
            {
                if (p.vertex_buffer.buffer != VK_NULL_HANDLE)
                {
                    rm->destroy_buffer(p.vertex_buffer);
                    p.vertex_buffer = {};
                    p.vertex_buffer_address = 0;
                }
            }

            state.patch_lookup.clear();
            state.patch_lru.clear();
            state.patch_free.clear();
            state.patches.clear();

            if (state.material_constants_buffer.buffer != VK_NULL_HANDLE)
            {
                rm->destroy_buffer(state.material_constants_buffer);
                state.material_constants_buffer = {};
            }
        }

        if (_earth_patch_index_buffer.buffer != VK_NULL_HANDLE)
        {
            rm->destroy_buffer(_earth_patch_index_buffer);
            _earth_patch_index_buffer = {};
        }
    }

    if (_earth_patch_material_allocator_initialized)
    {
        if (DeviceManager *device = _context->getDevice())
        {
            _earth_patch_material_allocator.destroy_pools(device->device());
        }
        _earth_patch_material_allocator_initialized = false;
    }

    if (_earth_patch_material_layout != VK_NULL_HANDLE)
    {
        if (DeviceManager *device = _context->getDevice())
        {
            vkDestroyDescriptorSetLayout(device->device(), _earth_patch_material_layout, nullptr);
        }
        _earth_patch_material_layout = VK_NULL_HANDLE;
    }

    _terrain_states.clear();

    _earth_patch_index_count = 0;
    _earth_patch_index_resolution = 0;

    _bodies.clear();
}

PlanetSystem::PlanetBody *PlanetSystem::find_body_by_name(std::string_view name)
{
    auto it = std::find_if(_bodies.begin(), _bodies.end(),
                           [&](const PlanetBody &b) { return b.name == name; });
    return it != _bodies.end() ? &(*it) : nullptr;
}

double PlanetSystem::sample_terrain_displacement_m(const PlanetBody &body, const glm::dvec3 &dir_from_center) const
{
    if (!body.terrain || !(body.terrain_height_max_m > 0.0))
    {
        return 0.0;
    }

    const TerrainState *state = find_terrain_state(body.name);
    if (!state)
    {
        return 0.0;
    }

    planet::CubeFace face = planet::CubeFace::PosX;
    double u01 = 0.0;
    double v01 = 0.0;
    if (!planet::cubesphere_direction_to_face_uv(dir_from_center, face, u01, v01))
    {
        return 0.0;
    }

    const uint32_t face_index = static_cast<uint32_t>(face);
    if (face_index >= state->height_faces.size())
    {
        return 0.0;
    }

    const planet::HeightFace &height_face = state->height_faces[face_index];
    if (height_face.width == 0 || height_face.height == 0 || height_face.texels.empty())
    {
        return 0.0;
    }

    const float h01 = planet::sample_height(height_face, static_cast<float>(u01), static_cast<float>(v01));
    return static_cast<double>(h01) * body.terrain_height_max_m;
}

const PlanetSystem::EarthDebugStats &PlanetSystem::terrain_debug_stats(std::string_view name) const
{
    const TerrainState *state = find_terrain_state(name);
    return state ? state->debug_stats : _empty_debug_stats;
}

const PlanetSystem::EarthDebugStats &PlanetSystem::earth_debug_stats() const
{
    const PlanetBody *body = find_terrain_body();
    return body ? terrain_debug_stats(body->name) : _empty_debug_stats;
}

PlanetSystem::TerrainState *PlanetSystem::get_or_create_terrain_state(std::string_view name)
{
    if (name.empty())
    {
        return nullptr;
    }

    std::string key{name};
    auto it = _terrain_states.find(key);
    if (it != _terrain_states.end())
    {
        return it->second.get();
    }

    auto state = std::make_unique<TerrainState>();
    TerrainState *ptr = state.get();
    _terrain_states.emplace(std::move(key), std::move(state));
    return ptr;
}

PlanetSystem::TerrainState *PlanetSystem::find_terrain_state(std::string_view name)
{
    auto it = _terrain_states.find(std::string{name});
    return (it != _terrain_states.end()) ? it->second.get() : nullptr;
}

const PlanetSystem::TerrainState *PlanetSystem::find_terrain_state(std::string_view name) const
{
    auto it = _terrain_states.find(std::string{name});
    return (it != _terrain_states.end()) ? it->second.get() : nullptr;
}

PlanetSystem::PlanetBody *PlanetSystem::create_mesh_planet(const MeshPlanetCreateInfo &info)
{
    if (info.name.empty() || find_body_by_name(info.name))
    {
        return nullptr;
    }

    PlanetBody body{};
    body.name = info.name;
    body.center_world = info.center_world;
    body.radius_m = info.radius_m;
    body.visible = info.visible;
    body.terrain = false;
    body.base_color = info.base_color;
    body.metallic = info.metallic;
    body.roughness = info.roughness;

    if (_context && _context->assets)
    {
        AssetManager *assets = _context->assets;

        const std::string asset_name = fmt::format("Planet_{}", info.name);
        const GLTFMetallic_Roughness::MaterialConstants mc =
                make_planet_constants(info.base_color, info.metallic, info.roughness);

        body.material = assets->createMaterialFromConstants(asset_name, mc, MaterialPass::MainColor);

        std::vector<Vertex> verts;
        std::vector<uint32_t> inds;
        primitives::buildSphere(verts, inds,
                                static_cast<int>(std::max(3u, info.sectors)),
                                static_cast<int>(std::max(2u, info.stacks)));
        geom::generate_tangents(verts, inds);

        body.mesh = assets->createMesh(asset_name,
                                       std::span<Vertex>(verts),
                                       std::span<uint32_t>(inds),
                                       body.material);
    }

    _bodies.push_back(std::move(body));
    return &_bodies.back();
}

PlanetSystem::PlanetBody *PlanetSystem::create_terrain_planet(const TerrainPlanetCreateInfo &info)
{
    if (info.name.empty() || find_body_by_name(info.name))
    {
        return nullptr;
    }

    PlanetBody body{};
    body.name = info.name;
    body.center_world = info.center_world;
    body.radius_m = info.radius_m;
    body.visible = info.visible;

    body.terrain = true;
    body.base_color = info.base_color;
    body.metallic = info.metallic;
    body.roughness = info.roughness;
    body.terrain_albedo_dir = info.albedo_dir;
    body.terrain_height_dir = info.height_dir;
    body.terrain_height_max_m = (!info.height_dir.empty()) ? std::max(0.0, info.height_max_m) : 0.0;
    body.terrain_emission_dir = info.emission_dir;
    body.emission_factor = info.emission_factor;

    if (_context && _context->assets)
    {
        AssetManager *assets = _context->assets;

        const std::string asset_name = fmt::format("Planet_{}_TerrainMaterial", info.name);
        const GLTFMetallic_Roughness::MaterialConstants mc =
                make_planet_constants(info.base_color, info.metallic, info.roughness, info.emission_factor);
        body.material = assets->createMaterialFromConstants(asset_name, mc, MaterialPass::MainColor);
    }

    _bodies.push_back(std::move(body));
    return &_bodies.back();
}

bool PlanetSystem::destroy_planet(std::string_view name)
{
    if (name.empty())
    {
        return false;
    }

    auto it = std::find_if(_bodies.begin(), _bodies.end(), [&](const PlanetBody &b) { return b.name == name; });
    if (it == _bodies.end())
    {
        return false;
    }

    // Clean up any terrain state for this planet (even if not currently marked as terrain).
    if (TerrainState *state = find_terrain_state(it->name))
    {
        clear_terrain_patch_cache(*state);
        clear_terrain_materials(*state);
        if (_context && state->material_constants_buffer.buffer != VK_NULL_HANDLE)
        {
            ResourceManager *rm = _context->getResources();
            FrameResources *frame = _context->currentFrame;
            const AllocatedBuffer buf = state->material_constants_buffer;
            if (rm)
            {
                if (frame)
                {
                    frame->_deletionQueue.push_function([rm, buf]() { rm->destroy_buffer(buf); });
                }
                else
                {
                    rm->destroy_buffer(buf);
                }
            }
            state->material_constants_buffer = {};
        }
    }
    _terrain_states.erase(std::string{name});

    if (it->mesh && _context && _context->assets)
    {
        AssetManager *assets = _context->assets;
        if (_context->currentFrame)
        {
            assets->removeMeshDeferred(it->mesh->name, _context->currentFrame->_deletionQueue);
        }
        else
        {
            assets->removeMesh(it->mesh->name);
        }
    }

    _bodies.erase(it);
    return true;
}

void PlanetSystem::clear_planets(bool destroy_mesh_assets)
{
    if (destroy_mesh_assets && _context && _context->assets)
    {
        AssetManager *assets = _context->assets;
        for (const PlanetBody &b: _bodies)
        {
            if (!b.mesh) continue;
            if (_context->currentFrame)
            {
                assets->removeMeshDeferred(b.mesh->name, _context->currentFrame->_deletionQueue);
            }
            else
            {
                assets->removeMesh(b.mesh->name);
            }
        }
    }

    // Terrain patches can be very large; clear them even if we keep the
    // shared index/material resources around.
    for (auto &kv: _terrain_states)
    {
        TerrainState &state = *kv.second;
        clear_terrain_patch_cache(state);
        clear_terrain_materials(state);
        if (_context && state.material_constants_buffer.buffer != VK_NULL_HANDLE)
        {
            ResourceManager *rm = _context->getResources();
            FrameResources *frame = _context->currentFrame;
            const AllocatedBuffer buf = state.material_constants_buffer;
            if (rm)
            {
                if (frame)
                {
                    frame->_deletionQueue.push_function([rm, buf]() { rm->destroy_buffer(buf); });
                }
                else
                {
                    rm->destroy_buffer(buf);
                }
            }
            state.material_constants_buffer = {};
        }
    }
    _terrain_states.clear();

    _bodies.clear();
}

bool PlanetSystem::set_planet_center(std::string_view name, const WorldVec3 &center_world)
{
    PlanetBody *b = find_body_by_name(name);
    if (!b) return false;
    b->center_world = center_world;
    return true;
}

bool PlanetSystem::set_planet_radius(std::string_view name, double radius_m)
{
    PlanetBody *b = find_body_by_name(name);
    if (!b) return false;

    const double safe_radius = std::max(1.0, radius_m);
    if (b->radius_m == safe_radius) return true;

    b->radius_m = safe_radius;
    if (b->terrain)
    {
        if (TerrainState *state = find_terrain_state(b->name))
        {
            clear_terrain_patch_cache(*state);
            state->debug_stats = {};
        }
    }
    return true;
}

bool PlanetSystem::set_planet_visible(std::string_view name, bool visible)
{
    PlanetBody *b = find_body_by_name(name);
    if (!b) return false;
    b->visible = visible;
    return true;
}

bool PlanetSystem::set_planet_terrain(std::string_view name, bool terrain)
{
    PlanetBody *target = find_body_by_name(name);
    if (!target) return false;

    if (target->terrain == terrain)
    {
        return true;
    }

    target->terrain = terrain;
    if (terrain)
    {
        if (TerrainState *state = get_or_create_terrain_state(target->name))
        {
            state->patch_cache_dirty = true;
        }
    }
    else
    {
        if (TerrainState *state = find_terrain_state(target->name))
        {
            clear_terrain_patch_cache(*state);
            clear_terrain_materials(*state);
            state->debug_stats = {};
        }
    }
    return true;
}

PlanetSystem::PlanetBody *PlanetSystem::find_terrain_body()
{
    auto it = std::find_if(_bodies.begin(), _bodies.end(),
                           [](const PlanetBody &b) { return b.terrain; });
    return it != _bodies.end() ? &(*it) : nullptr;
}

const PlanetSystem::PlanetBody *PlanetSystem::find_terrain_body() const
{
    auto it = std::find_if(_bodies.begin(), _bodies.end(),
                           [](const PlanetBody &b) { return b.terrain; });
    return it != _bodies.end() ? &(*it) : nullptr;
}

PlanetSystem::TerrainPatch *PlanetSystem::find_terrain_patch(TerrainState &state, const planet::PatchKey &key)
{
    auto it = state.patch_lookup.find(key);
    if (it == state.patch_lookup.end())
    {
        return nullptr;
    }
    const uint32_t idx = it->second;
    if (idx >= state.patches.size())
    {
        return nullptr;
    }
    return &state.patches[idx];
}

void PlanetSystem::clear_terrain_patch_cache(TerrainState &state)
{
    if (!_context)
    {
        return;
    }

    ResourceManager *rm = _context->getResources();
    FrameResources *frame = _context->currentFrame;

    if (rm)
    {
        for (TerrainPatch &p: state.patches)
        {
            if (p.vertex_buffer.buffer == VK_NULL_HANDLE)
            {
                continue;
            }

            const AllocatedBuffer vb = p.vertex_buffer;
            if (frame)
            {
                frame->_deletionQueue.push_function([rm, vb]() { rm->destroy_buffer(vb); });
            }
            else
            {
                rm->destroy_buffer(vb);
            }

            p.vertex_buffer = {};
            p.vertex_buffer_address = 0;
        }
    }

    state.patch_lookup.clear();
    state.patch_lru.clear();
    state.patch_free.clear();
    state.patches.clear();
}

void PlanetSystem::clear_all_terrain_patch_caches()
{
    for (auto &kv: _terrain_states)
    {
        if (!kv.second)
        {
            continue;
        }
        clear_terrain_patch_cache(*kv.second);
        kv.second->debug_stats = {};
    }
}

void PlanetSystem::clear_terrain_materials(TerrainState &state)
{
    if (_context && _context->textures)
    {
        TextureCache *textures = _context->textures;
        for (MaterialInstance &mat: state.face_materials)
        {
            if (mat.materialSet != VK_NULL_HANDLE)
            {
                textures->unwatchSet(mat.materialSet);
            }
        }
    }

    // Keep descriptor sets allocated so they can be reused if terrain is re-enabled,
    // but force a rebinding on next use.
    state.bound_albedo_dir.clear();
}

void PlanetSystem::ensure_earth_patch_index_buffer()
{
    if (_earth_patch_index_buffer.buffer != VK_NULL_HANDLE && _earth_patch_index_resolution == _earth_patch_resolution)
    {
        return;
    }

    if (!_context)
    {
        return;
    }

    ResourceManager *rm = _context->getResources();
    if (!rm)
    {
        return;
    }

    // Resolution changed (or first init): clear existing patch cache and shared index buffer.
    if (_earth_patch_index_buffer.buffer != VK_NULL_HANDLE)
    {
        FrameResources *frame = _context->currentFrame;

        clear_all_terrain_patch_caches();

        const AllocatedBuffer ib = _earth_patch_index_buffer;
        if (frame)
        {
            frame->_deletionQueue.push_function([rm, ib]() { rm->destroy_buffer(ib); });
        }
        else
        {
            rm->destroy_buffer(ib);
        }
        _earth_patch_index_buffer = {};
        _earth_patch_index_count = 0;
        _earth_patch_index_resolution = 0;
    }

    std::vector<uint32_t> indices;
    planet::build_cubesphere_patch_indices(indices, _earth_patch_resolution);
    _earth_patch_index_count = static_cast<uint32_t>(indices.size());
    _earth_patch_index_buffer =
            rm->upload_buffer(indices.data(),
                              indices.size() * sizeof(uint32_t),
                              VK_BUFFER_USAGE_INDEX_BUFFER_BIT |
                              VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT |
                              VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR);
    _earth_patch_index_resolution = _earth_patch_resolution;
}

void PlanetSystem::ensure_earth_patch_material_layout()
{
    if (_earth_patch_material_layout != VK_NULL_HANDLE)
    {
        return;
    }

    if (!_context)
    {
        return;
    }

    DeviceManager *device = _context->getDevice();
    if (!device)
    {
        return;
    }

    DescriptorLayoutBuilder layoutBuilder;
    layoutBuilder.add_binding(0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER);
    layoutBuilder.add_binding(1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    layoutBuilder.add_binding(2, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    layoutBuilder.add_binding(3, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    layoutBuilder.add_binding(4, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    layoutBuilder.add_binding(5, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);

    _earth_patch_material_layout = layoutBuilder.build(device->device(),
                                                       VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
                                                       nullptr,
                                                       VK_DESCRIPTOR_SET_LAYOUT_CREATE_UPDATE_AFTER_BIND_POOL_BIT);
}

void PlanetSystem::ensure_terrain_material_constants_buffer(TerrainState &state, const PlanetBody &body)
{
    if (state.material_constants_buffer.buffer != VK_NULL_HANDLE)
    {
        if (!_context)
        {
            return;
        }

        DeviceManager *device = _context->getDevice();
        if (!device)
        {
            return;
        }

        const bool same_constants =
                state.bound_base_color == body.base_color &&
                state.bound_metallic == body.metallic &&
                state.bound_roughness == body.roughness &&
                state.bound_emission_factor == body.emission_factor;
        if (same_constants)
        {
            return;
        }

        const GLTFMetallic_Roughness::MaterialConstants constants =
                make_planet_constants(body.base_color, body.metallic, body.roughness, body.emission_factor);

        VmaAllocationInfo allocInfo{};
        vmaGetAllocationInfo(device->allocator(), state.material_constants_buffer.allocation, &allocInfo);
        auto *mapped = static_cast<GLTFMetallic_Roughness::MaterialConstants *>(allocInfo.pMappedData);
        if (mapped)
        {
            *mapped = constants;
            vmaFlushAllocation(device->allocator(),
                               state.material_constants_buffer.allocation,
                               0,
                               sizeof(constants));

            state.bound_base_color = body.base_color;
            state.bound_metallic = body.metallic;
            state.bound_roughness = body.roughness;
            state.bound_emission_factor = body.emission_factor;
        }
        return;
    }

    if (!_context)
    {
        return;
    }

    ResourceManager *rm = _context->getResources();
    DeviceManager *device = _context->getDevice();
    if (!rm || !device)
    {
        return;
    }

    const GLTFMetallic_Roughness::MaterialConstants constants =
            make_planet_constants(body.base_color, body.metallic, body.roughness, body.emission_factor);

    state.material_constants_buffer =
            rm->create_buffer(sizeof(GLTFMetallic_Roughness::MaterialConstants),
                              VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
                              VMA_MEMORY_USAGE_CPU_TO_GPU);

    if (state.material_constants_buffer.buffer == VK_NULL_HANDLE)
    {
        return;
    }

    VmaAllocationInfo allocInfo{};
    vmaGetAllocationInfo(device->allocator(), state.material_constants_buffer.allocation, &allocInfo);
    auto *mapped = static_cast<GLTFMetallic_Roughness::MaterialConstants *>(allocInfo.pMappedData);
    if (mapped)
    {
        *mapped = constants;
        vmaFlushAllocation(device->allocator(), state.material_constants_buffer.allocation, 0, sizeof(constants));

        state.bound_base_color = body.base_color;
        state.bound_metallic = body.metallic;
        state.bound_roughness = body.roughness;
        state.bound_emission_factor = body.emission_factor;
    }
}

void PlanetSystem::ensure_terrain_face_materials(TerrainState &state, const PlanetBody &body)
{
    if (!_context || !body.material)
    {
        return;
    }

    DeviceManager *device = _context->getDevice();
    SamplerManager *samplers = _context->getSamplers();
    AssetManager *assets = _context->assets;
    TextureCache *textures = _context->textures;
    if (!device || !assets)
    {
        return;
    }

    ensure_earth_patch_material_layout();
    ensure_terrain_material_constants_buffer(state, body);

    if (_earth_patch_material_layout == VK_NULL_HANDLE ||
        state.material_constants_buffer.buffer == VK_NULL_HANDLE)
    {
        return;
    }

    if (!_earth_patch_material_allocator_initialized)
    {
        std::vector<DescriptorAllocatorGrowable::PoolSizeRatio> sizes = {
            {VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1},
            {VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 6},
        };
        _earth_patch_material_allocator.init(device->device(), 16, sizes);
        _earth_patch_material_allocator_initialized = true;
    }

    VkSampler tileSampler = samplers ? samplers->linearClampEdge() : VK_NULL_HANDLE;
    if (tileSampler == VK_NULL_HANDLE && samplers)
    {
        tileSampler = samplers->defaultLinear();
    }
    if (tileSampler == VK_NULL_HANDLE)
    {
        return;
    }

    VkImageView checker = assets->fallbackCheckerboardView();
    VkImageView white = assets->fallbackWhiteView();
    VkImageView flatNormal = assets->fallbackFlatNormalView();
    VkImageView black = assets->fallbackBlackView();

    if (checker == VK_NULL_HANDLE) checker = white;
    if (white == VK_NULL_HANDLE) white = checker;
    if (flatNormal == VK_NULL_HANDLE) flatNormal = white;
    if (black == VK_NULL_HANDLE) black = white;

    const std::string desired_albedo_dir = body.terrain_albedo_dir;
    const bool albedo_dir_changed = desired_albedo_dir != state.bound_albedo_dir;
    if (albedo_dir_changed)
    {
        state.bound_albedo_dir = desired_albedo_dir;
    }

    const std::string desired_emission_dir = body.terrain_emission_dir;
    const bool emission_dir_changed = desired_emission_dir != state.bound_emission_dir;
    if (emission_dir_changed)
    {
        state.bound_emission_dir = desired_emission_dir;
    }

    for (size_t face_index = 0; face_index < state.face_materials.size(); ++face_index)
    {
        MaterialInstance &mat = state.face_materials[face_index];

        mat.pipeline = body.material->data.pipeline;
        mat.passType = body.material->data.passType;

        if (mat.materialSet == VK_NULL_HANDLE)
        {
            mat.materialSet = _earth_patch_material_allocator.allocate(device->device(), _earth_patch_material_layout);

            DescriptorWriter writer;
            writer.write_buffer(0,
                                state.material_constants_buffer.buffer,
                                sizeof(GLTFMetallic_Roughness::MaterialConstants),
                                0,
                                VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER);
            writer.write_image(1,
                               checker,
                               tileSampler,
                               VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                               VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
            writer.write_image(2,
                               white,
                               tileSampler,
                               VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                               VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
            writer.write_image(3,
                               flatNormal,
                               tileSampler,
                               VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                               VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
            writer.write_image(4,
                               white,
                               tileSampler,
                               VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                               VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
            writer.write_image(5,
                               black,
                               tileSampler,
                               VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                               VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
            writer.update_set(device->device(), mat.materialSet);

            if (textures && tileSampler != VK_NULL_HANDLE && !desired_albedo_dir.empty())
            {
                const planet::CubeFace face = static_cast<planet::CubeFace>(face_index);
                const std::string rel = fmt::format("{}/{}.ktx2", desired_albedo_dir, planet::cube_face_name(face));

                TextureCache::TextureKey tk{};
                tk.kind = TextureCache::TextureKey::SourceKind::FilePath;
                tk.path = assets->assetPath(rel);
                tk.srgb = true;
                tk.mipmapped = true;

                TextureCache::TextureHandle h = textures->request(tk, tileSampler);
                textures->watchBinding(h, mat.materialSet, 1u, tileSampler, checker);
            }

            if (textures && tileSampler != VK_NULL_HANDLE && !desired_emission_dir.empty())
            {
                const planet::CubeFace face = static_cast<planet::CubeFace>(face_index);
                // Try .ktx2 first, then .png
                std::string rel = fmt::format("{}/{}.ktx2", desired_emission_dir, planet::cube_face_name(face));
                std::string abs_path = assets->assetPath(rel);
                if (!std::filesystem::exists(abs_path))
                {
                    rel = fmt::format("{}/{}.png", desired_emission_dir, planet::cube_face_name(face));
                    abs_path = assets->assetPath(rel);
                }

                TextureCache::TextureKey tk{};
                tk.kind = TextureCache::TextureKey::SourceKind::FilePath;
                tk.path = abs_path;
                tk.srgb = true;
                tk.mipmapped = true;

                TextureCache::TextureHandle h = textures->request(tk, tileSampler);
                textures->watchBinding(h, mat.materialSet, 5u, tileSampler, black);
            }
        }
        else if ((albedo_dir_changed || emission_dir_changed) && textures && tileSampler != VK_NULL_HANDLE)
        {
            textures->unwatchSet(mat.materialSet);

            DescriptorWriter writer;
            if (albedo_dir_changed)
            {
                writer.write_image(1,
                                   checker,
                                   tileSampler,
                                   VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                                   VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
            }
            if (emission_dir_changed)
            {
                writer.write_image(5,
                                   black,
                                   tileSampler,
                                   VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                                   VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
            }
            writer.update_set(device->device(), mat.materialSet);

            if (albedo_dir_changed && !desired_albedo_dir.empty())
            {
                const planet::CubeFace face = static_cast<planet::CubeFace>(face_index);
                const std::string rel = fmt::format("{}/{}.ktx2", desired_albedo_dir, planet::cube_face_name(face));

                TextureCache::TextureKey tk{};
                tk.kind = TextureCache::TextureKey::SourceKind::FilePath;
                tk.path = assets->assetPath(rel);
                tk.srgb = true;
                tk.mipmapped = true;

                TextureCache::TextureHandle h = textures->request(tk, tileSampler);
                textures->watchBinding(h, mat.materialSet, 1u, tileSampler, checker);
            }

            if (emission_dir_changed && !desired_emission_dir.empty())
            {
                const planet::CubeFace face = static_cast<planet::CubeFace>(face_index);
                // Try .ktx2 first, then .png
                std::string rel = fmt::format("{}/{}.ktx2", desired_emission_dir, planet::cube_face_name(face));
                std::string abs_path = assets->assetPath(rel);
                if (!std::filesystem::exists(abs_path))
                {
                    rel = fmt::format("{}/{}.png", desired_emission_dir, planet::cube_face_name(face));
                    abs_path = assets->assetPath(rel);
                }

                TextureCache::TextureKey tk{};
                tk.kind = TextureCache::TextureKey::SourceKind::FilePath;
                tk.path = abs_path;
                tk.srgb = true;
                tk.mipmapped = true;

                TextureCache::TextureHandle h = textures->request(tk, tileSampler);
                textures->watchBinding(h, mat.materialSet, 5u, tileSampler, black);
            }
        }
    }
}

void PlanetSystem::ensure_terrain_height_maps(TerrainState &state, const PlanetBody &body)
{
    if (!_context || !_context->assets)
    {
        return;
    }

    const std::string desired_dir = body.terrain_height_dir;
    const double desired_max_m = body.terrain_height_max_m;
    const bool changed =
            desired_dir != state.bound_height_dir ||
            desired_max_m != state.bound_height_max_m;

    const bool want_height = !desired_dir.empty() && (desired_max_m > 0.0);
    const bool have_height = [&]() -> bool {
        for (const planet::HeightFace &f: state.height_faces)
        {
            if (f.width == 0 || f.height == 0 || f.texels.empty())
            {
                return false;
            }
        }
        return true;
    }();

    const bool needs_load = changed || (want_height && !have_height);
    if (!needs_load)
    {
        return;
    }

    if (changed)
    {
        // Height affects vertex positions/normals; regenerate patch meshes if parameters change.
        clear_terrain_patch_cache(state);
        state.patch_cache_dirty = false;

        state.bound_height_dir = desired_dir;
        state.bound_height_max_m = desired_max_m;
        for (planet::HeightFace &f: state.height_faces)
        {
            f = {};
        }
    }

    if (!want_height)
    {
        for (planet::HeightFace &f: state.height_faces)
        {
            f = {};
        }
        return;
    }

    AssetManager *assets = _context->assets;
    std::array<planet::HeightFace, 6> loaded_faces{};
    bool ok = true;
    for (size_t face_index = 0; face_index < loaded_faces.size(); ++face_index)
    {
        const planet::CubeFace face = static_cast<planet::CubeFace>(face_index);
        const std::string rel = fmt::format("{}/{}.ktx2", desired_dir, planet::cube_face_name(face));
        const std::string abs = assets->assetPath(rel);

        planet::HeightFace face_data{};
        if (!planet::load_heightmap_bc4(abs, face_data))
        {
            fmt::println("[PlanetSystem] Failed to load height face '{}'", abs);
            ok = false;
            break;
        }
        loaded_faces[face_index] = std::move(face_data);
    }

    if (!ok)
    {
        // If this was a retry (parameters didn't change), keep existing geometry and retry later.
        // If parameters changed, we already cleared meshes/faces above.
        return;
    }

    state.height_faces = std::move(loaded_faces);

    if (!changed)
    {
        // Recovered height data after a previous failure; regenerate meshes so displacement applies.
        clear_terrain_patch_cache(state);
        state.patch_cache_dirty = false;
    }
}

PlanetSystem::TerrainPatch *PlanetSystem::get_or_create_terrain_patch(TerrainState &state,
                                                                      const PlanetBody &body,
                                                                      const planet::PatchKey &key,
                                                                      uint32_t frame_index,
                                                                      uint8_t edge_stitch_mask)
{
    uint32_t reuse_idx = UINT32_MAX;
    if (TerrainPatch *p = find_terrain_patch(state, key))
    {
        p->last_used_frame = frame_index;
        state.patch_lru.splice(state.patch_lru.begin(), state.patch_lru, p->lru_it);
        if (p->edge_stitch_mask == edge_stitch_mask)
        {
            return p;
        }

        reuse_idx = static_cast<uint32_t>(p - state.patches.data());
        if (_context && p->vertex_buffer.buffer != VK_NULL_HANDLE)
        {
            ResourceManager *rm_existing = _context->getResources();
            FrameResources *frame_existing = _context->currentFrame;
            if (rm_existing)
            {
                const AllocatedBuffer vb_old = p->vertex_buffer;
                if (frame_existing)
                {
                    frame_existing->_deletionQueue.push_function([rm_existing, vb_old]() { rm_existing->destroy_buffer(vb_old); });
                }
                else
                {
                    rm_existing->destroy_buffer(vb_old);
                }
            }
            p->vertex_buffer = {};
            p->vertex_buffer_address = 0;
        }
    }

    if (!_context)
    {
        return nullptr;
    }

    ResourceManager *rm = _context->getResources();
    DeviceManager *device = _context->getDevice();
    if (!rm || !device)
    {
        return nullptr;
    }

    if (_earth_patch_index_buffer.buffer == VK_NULL_HANDLE || _earth_patch_index_count == 0)
    {
        return nullptr;
    }

    const glm::vec4 vertex_color =
            _earth_debug_tint_patches_by_lod ? debug_color_for_level(key.level) : glm::vec4(1.0f);

    thread_local std::vector<Vertex> scratch_vertices;
    const uint32_t safe_res = std::max(2u, _earth_patch_resolution);
    scratch_vertices.reserve(
        static_cast<size_t>(safe_res) * static_cast<size_t>(safe_res) +
        static_cast<size_t>(4u) * static_cast<size_t>(safe_res));
    const glm::dvec3 patch_center_dir =
            planet::build_cubesphere_patch_vertices(scratch_vertices,
                                                    body.radius_m,
                                                    key.face,
                                                    key.level,
                                                    key.x,
                                                    key.y,
                                                    safe_res,
                                                    vertex_color);

    if (scratch_vertices.empty())
    {
        return nullptr;
    }

    if (body.terrain_height_max_m > 0.0)
    {
        const uint32_t face_index = static_cast<uint32_t>(key.face);
        if (face_index < state.height_faces.size())
        {
            const planet::HeightFace &height_face = state.height_faces[face_index];
            if (height_face.width > 0 && height_face.height > 0 && !height_face.texels.empty())
            {
                const float scale = static_cast<float>(body.terrain_height_max_m);
                constexpr float kFaceEdgeEpsilon = 1e-6f;
                for (Vertex &v: scratch_vertices)
                {
                    float h01 = 0.0f;

                    // On cube-face boundaries (u/v at 0 or 1), sample via direction mapping so
                    // both neighboring faces resolve to the same boundary samples.
                    const bool on_cube_face_edge =
                            (v.uv_x <= kFaceEdgeEpsilon) ||
                            (v.uv_x >= 1.0f - kFaceEdgeEpsilon) ||
                            (v.uv_y <= kFaceEdgeEpsilon) ||
                            (v.uv_y >= 1.0f - kFaceEdgeEpsilon);

                    if (on_cube_face_edge)
                    {
                        planet::CubeFace sample_face = key.face;
                        double sample_u = static_cast<double>(v.uv_x);
                        double sample_v = static_cast<double>(v.uv_y);
                        const glm::vec3 n = glm::normalize(v.normal);
                        const glm::dvec3 dir(static_cast<double>(n.x),
                                             static_cast<double>(n.y),
                                             static_cast<double>(n.z));
                        if (planet::cubesphere_direction_to_face_uv(dir, sample_face, sample_u, sample_v))
                        {
                            const uint32_t sample_face_index = static_cast<uint32_t>(sample_face);
                            if (sample_face_index < state.height_faces.size())
                            {
                                const planet::HeightFace &sample_height_face = state.height_faces[sample_face_index];
                                if (sample_height_face.width > 0 &&
                                    sample_height_face.height > 0 &&
                                    !sample_height_face.texels.empty())
                                {
                                    h01 = planet::sample_height(sample_height_face,
                                                                static_cast<float>(sample_u),
                                                                static_cast<float>(sample_v));
                                }
                                else
                                {
                                    h01 = planet::sample_height(height_face, v.uv_x, v.uv_y);
                                }
                            }
                            else
                            {
                                h01 = planet::sample_height(height_face, v.uv_x, v.uv_y);
                            }
                        }
                        else
                        {
                            h01 = planet::sample_height(height_face, v.uv_x, v.uv_y);
                        }
                    }
                    else
                    {
                        h01 = planet::sample_height(height_face, v.uv_x, v.uv_y);
                    }

                    const float h_m = h01 * scale;
                    v.position += v.normal * h_m;
                }

                stitch_patch_edges_to_parent_grid(scratch_vertices, safe_res, edge_stitch_mask);
                recompute_patch_normals(scratch_vertices, safe_res);
                refine_patch_edge_normals_from_height(scratch_vertices,
                                                      safe_res,
                                                      patch_center_dir,
                                                      body.radius_m,
                                                      key.level,
                                                      edge_stitch_mask,
                                                      body.terrain_height_max_m,
                                                      state.height_faces);
            }
        }
    }

    reinforce_patch_skirts(scratch_vertices,
                           safe_res,
                           patch_center_dir,
                           body.radius_m,
                           key.level);

    const PatchBoundsData bounds = compute_patch_bounds(scratch_vertices);

    AllocatedBuffer vb =
            rm->upload_buffer(scratch_vertices.data(),
                              scratch_vertices.size() * sizeof(Vertex),
                              VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                              VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT |
                              VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR);
    if (vb.buffer == VK_NULL_HANDLE)
    {
        return nullptr;
    }

    VkBufferDeviceAddressInfo addrInfo{.sType = VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO};
    addrInfo.buffer = vb.buffer;
    VkDeviceAddress addr = vkGetBufferDeviceAddress(device->device(), &addrInfo);

    uint32_t idx = 0;
    if (reuse_idx != UINT32_MAX)
    {
        idx = reuse_idx;
    }
    else if (!state.patch_free.empty())
    {
        idx = state.patch_free.back();
        state.patch_free.pop_back();
    }
    else
    {
        idx = static_cast<uint32_t>(state.patches.size());
        state.patches.emplace_back();
    }

    if (idx >= state.patches.size())
    {
        return nullptr;
    }

    TerrainPatch &p = state.patches[idx];
    p.key = key;
    p.state = TerrainPatchState::Ready;
    p.edge_stitch_mask = edge_stitch_mask;
    p.vertex_buffer = vb;
    p.vertex_buffer_address = addr;
    p.bounds_origin = bounds.origin;
    p.bounds_extents = bounds.extents;
    p.bounds_sphere_radius = bounds.sphere_radius;
    p.patch_center_dir = patch_center_dir;
    p.last_used_frame = frame_index;
    if (reuse_idx == UINT32_MAX)
    {
        state.patch_lru.push_front(idx);
        p.lru_it = state.patch_lru.begin();
        state.patch_lookup.emplace(key, idx);
    }
    return &p;
}

void PlanetSystem::trim_terrain_patch_cache(TerrainState &state)
{
    if (_earth_patch_cache_max == 0)
    {
        return;
    }

    if (state.patch_lookup.size() <= static_cast<size_t>(_earth_patch_cache_max))
    {
        return;
    }

    if (!_context)
    {
        return;
    }

    ResourceManager *rm = _context->getResources();
    if (!rm)
    {
        return;
    }

    FrameResources *frame = _context->currentFrame;
    const uint32_t now = state.patch_frame_stamp;

    size_t guard = 0;
    const size_t guard_limit = state.patch_lru.size();

    while (state.patch_lookup.size() > static_cast<size_t>(_earth_patch_cache_max) && !state.patch_lru.empty())
    {
        if (guard++ >= guard_limit)
        {
            // No evictable patches (all used this frame). Avoid thrashing.
            break;
        }

        const uint32_t idx = state.patch_lru.back();
        if (idx >= state.patches.size())
        {
            state.patch_lru.pop_back();
            continue;
        }

        TerrainPatch &p = state.patches[idx];
        if (p.last_used_frame == now)
        {
            state.patch_lru.splice(state.patch_lru.begin(), state.patch_lru, p.lru_it);
            continue;
        }

        guard = 0;

        state.patch_lru.erase(p.lru_it);
        state.patch_lookup.erase(p.key);

        if (p.vertex_buffer.buffer != VK_NULL_HANDLE)
        {
            const AllocatedBuffer vb = p.vertex_buffer;
            if (frame)
            {
                frame->_deletionQueue.push_function([rm, vb]() { rm->destroy_buffer(vb); });
            }
            else
            {
                rm->destroy_buffer(vb);
            }
        }

        p = TerrainPatch{};
        state.patch_free.push_back(idx);
    }
}

void PlanetSystem::update_and_emit(const SceneManager &scene, DrawContext &draw_context)
{
    if (!_enabled)
    {
        return;
    }

    const WorldVec3 origin_world = scene.get_world_origin();

    // Terrain bodies: quadtree patches (each terrain planet has independent cache/state).
    {
        using Clock = std::chrono::steady_clock;

        if (_context == nullptr)
        {
            return;
        }

        if (_earth_patch_cache_dirty)
        {
            clear_all_terrain_patch_caches();
            _earth_patch_cache_dirty = false;
        }

        const VkExtent2D logical_extent = _context->getLogicalRenderExtent();
        const WorldVec3 cam_world = scene.getMainCamera().position_world;

        ensure_earth_patch_index_buffer();

        for (PlanetBody &body: _bodies)
        {
            if (!body.terrain || !body.visible || !body.material)
            {
                continue;
            }

            TerrainState *state = get_or_create_terrain_state(body.name);
            if (!state)
            {
                continue;
            }

            if (state->patch_cache_dirty)
            {
                clear_terrain_patch_cache(*state);
                state->patch_cache_dirty = false;
            }

            const Clock::time_point t0 = Clock::now();

            state->quadtree.set_settings(_earth_quadtree_settings);

            ensure_terrain_height_maps(*state, body);

            const Clock::time_point t_q0 = Clock::now();
            state->quadtree.update(body.center_world,
                                   body.radius_m,
                                   body.terrain_height_max_m,
                                   cam_world,
                                   origin_world,
                                   scene.getSceneData(),
                                   logical_extent,
                                   _earth_patch_resolution);
            const Clock::time_point t_q1 = Clock::now();

            ensure_terrain_face_materials(*state, body);
            if (_context->textures)
            {
                for (const MaterialInstance &mat: state->face_materials)
                {
                    if (mat.materialSet != VK_NULL_HANDLE)
                    {
                        _context->textures->markSetUsed(mat.materialSet, _context->frameIndex);
                    }
                }
            }

            size_t desired_capacity =
                    static_cast<size_t>(state->patches.size()) +
                    static_cast<size_t>(_earth_patch_create_budget_per_frame) +
                    32u;
            if (_earth_patch_cache_max != 0)
            {
                desired_capacity = std::max(
                    desired_capacity,
                    static_cast<size_t>(_earth_patch_cache_max) +
                    static_cast<size_t>(_earth_patch_create_budget_per_frame) +
                    32u);
            }
            if (state->patches.capacity() < desired_capacity)
            {
                state->patches.reserve(desired_capacity);
            }

            uint32_t created_patches = 0;
            double ms_patch_create = 0.0;
            const uint32_t max_create = _earth_patch_create_budget_per_frame;
            const double max_create_ms =
                    (_earth_patch_create_budget_ms > 0.0f) ? static_cast<double>(_earth_patch_create_budget_ms) : 0.0;
            const uint32_t frame_index = ++state->patch_frame_stamp;

            const Clock::time_point t_emit0 = Clock::now();

            const std::vector<planet::PatchKey> &desired_leaves = state->quadtree.visible_leaves();
            std::unordered_set<planet::PatchKey, planet::PatchKeyHash> desired_leaf_set;
            desired_leaf_set.reserve(desired_leaves.size() * 2u);
            uint32_t max_level_in_set = 0u;
            for (const planet::PatchKey &k: desired_leaves)
            {
                desired_leaf_set.insert(k);
                max_level_in_set = std::max(max_level_in_set, k.level);
            }

            std::unordered_map<planet::PatchKey, uint8_t, planet::PatchKeyHash> edge_stitch_masks;
            edge_stitch_masks.reserve(desired_leaves.size());
            for (const planet::PatchKey &k: desired_leaves)
            {
                const uint8_t mask = compute_patch_edge_stitch_mask(k, desired_leaf_set, max_level_in_set);
                if (mask != 0u)
                {
                    edge_stitch_masks.emplace(k, mask);
                }
            }

            auto stitch_mask_for = [&](const planet::PatchKey &k) -> uint8_t {
                auto it = edge_stitch_masks.find(k);
                return (it != edge_stitch_masks.end()) ? it->second : 0u;
            };

            // Patch creation priority: create higher-LOD (smaller) patches first so we fill -> much better at rendering thousands of it
            // near-camera terrain before spending budget on far patches.
            std::vector<planet::PatchKey> create_queue = desired_leaves;
            std::sort(create_queue.begin(), create_queue.end(),
                      [](const planet::PatchKey &a, const planet::PatchKey &b) {
                          if (a.level != b.level) return a.level > b.level;
                          if (a.face != b.face) return a.face < b.face;
                          if (a.x != b.x) return a.x < b.x;
                          return a.y < b.y;
                      });

            for (const planet::PatchKey &k: create_queue)
            {
                const uint8_t stitch_mask = stitch_mask_for(k);
                if (TerrainPatch *existing = find_terrain_patch(*state, k);
                    existing && existing->edge_stitch_mask == stitch_mask)
                {
                    continue;
                }

                const bool hit_count_budget = (max_create != 0u) && (created_patches >= max_create);
                const bool hit_time_budget = (max_create_ms > 0.0) && (ms_patch_create >= max_create_ms);
                if (hit_count_budget || hit_time_budget)
                {
                    break;
                }

                const Clock::time_point t_c0 = Clock::now();
                TerrainPatch *patch = get_or_create_terrain_patch(*state, body, k, frame_index, stitch_mask);
                const Clock::time_point t_c1 = Clock::now();

                if (patch)
                {
                    created_patches++;
                }
                ms_patch_create += std::chrono::duration<double, std::milli>(t_c1 - t_c0).count();
            }

            auto is_patch_ready = [&](const planet::PatchKey &k) -> bool {
                TerrainPatch *p = find_terrain_patch(*state, k);
                if (!p)
                {
                    return false;
                }

                if (p->state != TerrainPatchState::Ready ||
                    p->vertex_buffer.buffer == VK_NULL_HANDLE ||
                    p->vertex_buffer_address == 0)
                {
                    return false;
                }

                return true;
            };

            // Compute a render cut that never renders holes: if a desired leaf patch isn't ready yet,
            // fall back to the nearest ready ancestor patch.
            auto compute_render_cut = [&](const std::vector<planet::PatchKey> &desired_leaves)
                -> std::vector<planet::PatchKey> {
                std::vector<planet::PatchKey> render_keys;
                if (desired_leaves.empty())
                {
                    return render_keys;
                }

                std::unordered_set<planet::PatchKey, planet::PatchKeyHash> leaf_set;
                leaf_set.reserve(desired_leaves.size() * 2u);

                std::unordered_map<planet::PatchKey, uint8_t, planet::PatchKeyHash> child_masks;
                child_masks.reserve(desired_leaves.size() * 2u);

                for (const planet::PatchKey &leaf: desired_leaves)
                {
                    leaf_set.insert(leaf);

                    planet::PatchKey child = leaf;
                    while (child.level > 0u)
                    {
                        const planet::PatchKey parent{child.face, child.level - 1u, child.x >> 1u, child.y >> 1u};
                        const uint32_t child_idx = (child.x & 1u) | ((child.y & 1u) << 1u);
                        child_masks[parent] |= static_cast<uint8_t>(1u << child_idx);
                        child = parent;
                    }
                }

                render_keys.reserve(desired_leaves.size());

                auto traverse = [&](auto &&self, const planet::PatchKey &k) -> bool {
                    if (leaf_set.contains(k))
                    {
                        if (is_patch_ready(k))
                        {
                            render_keys.push_back(k);
                            return true;
                        }
                        return false;
                    }

                    auto it = child_masks.find(k);
                    if (it == child_masks.end() || it->second == 0u)
                    {
                        return true;
                    }

                    const size_t checkpoint = render_keys.size();
                    bool ok = true;
                    const uint8_t mask = it->second;

                    for (uint32_t cy = 0; cy < 2u; ++cy)
                    {
                        for (uint32_t cx = 0; cx < 2u; ++cx)
                        {
                            const uint32_t child_idx = cx + cy * 2u;
                            if ((mask & static_cast<uint8_t>(1u << child_idx)) == 0u)
                            {
                                continue;
                            }

                            const planet::PatchKey child{
                                k.face,
                                k.level + 1u,
                                k.x * 2u + cx,
                                k.y * 2u + cy,
                            };
                            if (!self(self, child))
                            {
                                ok = false;
                            }
                        }
                    }

                    if (ok)
                    {
                        return true;
                    }

                    // One or more desired children are missing-> fall back to this node to avoid
                    render_keys.resize(checkpoint);
                    if (is_patch_ready(k))
                    {
                        render_keys.push_back(k);
                        return true;
                    }
                    return false;
                };

                // Roots in deterministic +X,-X,+Y,-Y,+Z,-Z order.
                const std::array<planet::CubeFace, 6> faces{
                    planet::CubeFace::PosX,
                    planet::CubeFace::NegX,
                    planet::CubeFace::PosY,
                    planet::CubeFace::NegY,
                    planet::CubeFace::PosZ,
                    planet::CubeFace::NegZ,
                };

                for (planet::CubeFace face: faces)
                {
                    const planet::PatchKey root{face, 0u, 0u, 0u};
                    if (leaf_set.contains(root) || child_masks.find(root) != child_masks.end())
                    {
                        traverse(traverse, root);
                    }
                }

                return render_keys;
            };

            std::vector<uint32_t> ready_patch_indices;
            const std::vector<planet::PatchKey> render_keys = compute_render_cut(desired_leaves);
            ready_patch_indices.reserve(render_keys.size());

            for (const planet::PatchKey &k: render_keys)
            {
                const uint8_t stitch_mask = stitch_mask_for(k);
                TerrainPatch *patch = find_terrain_patch(*state, k);
                if (patch && patch->edge_stitch_mask != stitch_mask)
                {
                    patch = get_or_create_terrain_patch(*state, body, k, frame_index, stitch_mask);
                }
                if (!patch)
                {
                    continue;
                }

                patch->last_used_frame = frame_index;
                state->patch_lru.splice(state->patch_lru.begin(), state->patch_lru, patch->lru_it); {
                    const uint32_t idx = static_cast<uint32_t>(patch - state->patches.data());
                    ready_patch_indices.push_back(idx);
                }
            }

            for (uint32_t idx: ready_patch_indices)
            {
                if (idx >= state->patches.size())
                {
                    continue;
                }

                TerrainPatch &patch = state->patches[idx];
                if (patch.state != TerrainPatchState::Ready ||
                    patch.vertex_buffer.buffer == VK_NULL_HANDLE ||
                    patch.vertex_buffer_address == 0 ||
                    _earth_patch_index_buffer.buffer == VK_NULL_HANDLE ||
                    _earth_patch_index_count == 0)
                {
                    continue;
                }

                const uint32_t face_index = static_cast<uint32_t>(patch.key.face);
                if (face_index >= state->face_materials.size())
                {
                    continue;
                }
                MaterialInstance *material = &state->face_materials[face_index];
                if (material->materialSet == VK_NULL_HANDLE || material->pipeline == nullptr)
                {
                    continue;
                }

                const WorldVec3 patch_center_world =
                        body.center_world + patch.patch_center_dir * body.radius_m;
                const glm::vec3 patch_center_local = world_to_local(patch_center_world, origin_world);
                const glm::mat4 transform = glm::translate(glm::mat4(1.0f), patch_center_local);

                Bounds b{};
                b.origin = patch.bounds_origin;
                b.extents = patch.bounds_extents;
                b.sphereRadius = patch.bounds_sphere_radius;
                b.type = BoundsType::Box;

                RenderObject obj{};
                obj.indexCount = _earth_patch_index_count;
                obj.firstIndex = 0;
                obj.indexBuffer = _earth_patch_index_buffer.buffer;
                obj.vertexBuffer = patch.vertex_buffer.buffer;
                obj.vertexBufferAddress = patch.vertex_buffer_address;
                obj.material = material;
                obj.bounds = b;
                obj.transform = transform;
                // Planet terrain patches are not meaningful RT occluders; skip BLAS/TLAS builds.
                obj.sourceMesh = nullptr;
                obj.surfaceIndex = 0;
                obj.objectID = draw_context.nextID++;
                obj.ownerType = RenderObject::OwnerType::MeshInstance;
                obj.ownerName = body.name;

                draw_context.OpaqueSurfaces.push_back(obj);
            }
            const Clock::time_point t_emit1 = Clock::now();

            trim_terrain_patch_cache(*state);

            const uint32_t visible_patches = static_cast<uint32_t>(state->quadtree.visible_leaves().size());
            const uint32_t n = _earth_patch_resolution;
            const uint32_t patch_tris = (n >= 2u) ? (2u * (n - 1u) * (n + 3u)) : 0u;
            const uint32_t estimated_tris = patch_tris * visible_patches;

            state->debug_stats = {};
            state->debug_stats.quadtree = state->quadtree.stats();
            state->debug_stats.visible_patches = visible_patches;
            state->debug_stats.rendered_patches = static_cast<uint32_t>(ready_patch_indices.size());
            state->debug_stats.created_patches = created_patches;
            state->debug_stats.patch_cache_size = static_cast<uint32_t>(state->patch_lookup.size());
            state->debug_stats.estimated_triangles = estimated_tris;
            state->debug_stats.ms_quadtree = static_cast<float>(std::chrono::duration<double, std::milli>(t_q1 - t_q0).
                count());
            state->debug_stats.ms_patch_create = static_cast<float>(ms_patch_create);
            const double ms_emit_total = std::chrono::duration<double, std::milli>(t_emit1 - t_emit0).count();
            state->debug_stats.ms_emit = static_cast<float>(std::max(0.0, ms_emit_total - ms_patch_create));
            state->debug_stats.ms_total = static_cast<float>(std::chrono::duration<double, std::milli>(
                Clock::now() - t0).count());
        }
    }

    // Other bodies (moon): regular mesh instances.
    for (size_t body_index = 0; body_index < _bodies.size(); ++body_index)
    {
        PlanetBody &b = _bodies[body_index];

        if (b.terrain)
        {
            continue;
        }

        if (!b.visible || !b.mesh || b.mesh->surfaces.empty())
        {
            continue;
        }

        const glm::vec3 t_local = world_to_local(b.center_world, origin_world);
        const float r = static_cast<float>(b.radius_m);
        const glm::vec3 s = glm::vec3(r * 2.0f); // primitive sphere radius is 0.5
        const glm::quat q = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
        const glm::mat4 transform = make_trs_matrix(t_local, q, s);

        uint32_t surface_index = 0;
        for (const GeoSurface &surf: b.mesh->surfaces)
        {
            RenderObject obj{};
            obj.indexCount = surf.count;
            obj.firstIndex = surf.startIndex;
            obj.indexBuffer = b.mesh->meshBuffers.indexBuffer.buffer;
            obj.vertexBuffer = b.mesh->meshBuffers.vertexBuffer.buffer;
            obj.vertexBufferAddress = b.mesh->meshBuffers.vertexBufferAddress;
            obj.material = surf.material ? &surf.material->data : nullptr;
            obj.bounds = surf.bounds;
            obj.transform = transform;
            obj.sourceMesh = b.mesh.get();
            obj.surfaceIndex = surface_index++;
            obj.objectID = draw_context.nextID++;
            obj.ownerType = RenderObject::OwnerType::MeshInstance;
            obj.ownerName = b.name;

            if (obj.material && obj.material->passType == MaterialPass::Transparent)
            {
                draw_context.TransparentSurfaces.push_back(obj);
            }
            else if (obj.material && obj.material->passType == MaterialPass::MeshVFX)
            {
                draw_context.MeshVfxSurfaces.push_back(obj);
            }
            else
            {
                draw_context.OpaqueSurfaces.push_back(obj);
            }
        }
    }
}
