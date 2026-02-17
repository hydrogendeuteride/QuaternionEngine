#include "planet_patch_helpers.h"

#include <scene/planet/cubesphere.h>
#include <scene/planet/planet_heightmap.h>

#include <algorithm>
#include <cmath>
#include <unordered_set>

namespace
{
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
} // namespace

namespace planet_helpers
{
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
        const glm::vec4 &base_color,
        float metallic,
        float roughness,
        const glm::vec3 &emission_factor)
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
} // namespace planet_helpers
