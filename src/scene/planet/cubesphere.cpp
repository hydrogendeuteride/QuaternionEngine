#include "cubesphere.h"

#include <scene/tangent_space.h>

#include <cmath>

#include <glm/gtc/constants.hpp>

namespace planet
{
    glm::dvec3 cubesphere_unit_direction(CubeFace face, double u, double v)
    {
        // Convention: u increases right, v increases down (image space).
        glm::dvec3 d(0.0);
        switch (face)
        {
            case CubeFace::PosX: d = glm::dvec3(1.0, -v, -u);
                break;
            case CubeFace::NegX: d = glm::dvec3(-1.0, -v, u);
                break;
            case CubeFace::PosY: d = glm::dvec3(u, 1.0, v);
                break;
            case CubeFace::NegY: d = glm::dvec3(u, -1.0, -v);
                break;
            case CubeFace::PosZ: d = glm::dvec3(u, -v, 1.0);
                break;
            case CubeFace::NegZ: d = glm::dvec3(-u, -v, -1.0);
                break;
        }

        const double len2 = glm::dot(d, d);
        if (len2 <= 0.0)
        {
            return glm::dvec3(0.0, 0.0, 1.0);
        }
        return d * (1.0 / std::sqrt(len2));
    }

    bool cubesphere_direction_to_face_uv(const glm::dvec3 &dir,
                                         CubeFace &out_face,
                                         double &out_u01,
                                         double &out_v01)
    {
        glm::dvec3 d = dir;
        const double len2 = glm::dot(d, d);
        if (!(len2 > 0.0))
        {
            return false;
        }
        d *= (1.0 / std::sqrt(len2));

        const double ax = std::abs(d.x);
        const double ay = std::abs(d.y);
        const double az = std::abs(d.z);

        double u = 0.0;
        double v = 0.0;
        double ma = 1.0;

        if (ax >= ay && ax >= az)
        {
            ma = ax;
            if (d.x >= 0.0)
            {
                out_face = CubeFace::PosX;
                u = -d.z / ma;
                v = -d.y / ma;
            }
            else
            {
                out_face = CubeFace::NegX;
                u = d.z / ma;
                v = -d.y / ma;
            }
        }
        else if (ay >= ax && ay >= az)
        {
            ma = ay;
            if (d.y >= 0.0)
            {
                out_face = CubeFace::PosY;
                u = d.x / ma;
                v = d.z / ma;
            }
            else
            {
                out_face = CubeFace::NegY;
                u = d.x / ma;
                v = -d.z / ma;
            }
        }
        else
        {
            ma = az;
            if (d.z >= 0.0)
            {
                out_face = CubeFace::PosZ;
                u = d.x / ma;
                v = -d.y / ma;
            }
            else
            {
                out_face = CubeFace::NegZ;
                u = -d.x / ma;
                v = -d.y / ma;
            }
        }

        out_u01 = glm::clamp((u + 1.0) * 0.5, 0.0, 1.0);
        out_v01 = glm::clamp((v + 1.0) * 0.5, 0.0, 1.0);
        return true;
    }

    void cubesphere_tile_uv_bounds(uint32_t level, uint32_t x, uint32_t y,
                                   double &out_u0, double &out_u1,
                                   double &out_v0, double &out_v1)
    {
        const uint32_t tiles_u = (level < 31u) ? (1u << level) : 0u;
        const double inv_tiles = (tiles_u > 0u) ? (1.0 / static_cast<double>(tiles_u)) : 1.0;

        const double u0_01 = static_cast<double>(x) * inv_tiles;
        const double u1_01 = static_cast<double>(x + 1u) * inv_tiles;
        const double v0_01 = static_cast<double>(y) * inv_tiles;
        const double v1_01 = static_cast<double>(y + 1u) * inv_tiles;

        out_u0 = u0_01 * 2.0 - 1.0;
        out_u1 = u1_01 * 2.0 - 1.0;
        out_v0 = v0_01 * 2.0 - 1.0;
        out_v1 = v1_01 * 2.0 - 1.0;
    }

    glm::dvec3 cubesphere_patch_center_direction(CubeFace face, uint32_t level, uint32_t x, uint32_t y)
    {
        double u0 = 0.0, u1 = 0.0, v0 = 0.0, v1 = 0.0;
        cubesphere_tile_uv_bounds(level, x, y, u0, u1, v0, v1);
        const double u_mid = 0.5 * (u0 + u1);
        const double v_mid = 0.5 * (v0 + v1);
        return cubesphere_unit_direction(face, u_mid, v_mid);
    }

    WorldVec3 cubesphere_patch_center_world(const WorldVec3 &center_world,
                                            double radius_m,
                                            CubeFace face,
                                            uint32_t level,
                                            uint32_t x,
                                            uint32_t y)
    {
        const glm::dvec3 dir = cubesphere_patch_center_direction(face, level, x, y);
        return center_world + dir * radius_m;
    }

    double cubesphere_patch_edge_m(double radius_m, uint32_t level)
    {
        // Each cube face spans 90 degrees. Use arc length per tile edge as a simple estimate.
        const double face_arc_m = (glm::pi<double>() * 0.5) * radius_m;
        const uint32_t safe_level = (level < 30u) ? level : 30u;
        const double tiles_per_axis = static_cast<double>(1u << safe_level);
        return face_arc_m / tiles_per_axis;
    }

    double cubesphere_skirt_depth_m(double radius_m, uint32_t level)
    {
        const double edge_m = cubesphere_patch_edge_m(radius_m, level);
        return glm::max(10.0, 0.02 * edge_m);
    }

    void build_cubesphere_patch_indices(std::vector<uint32_t> &out_indices, uint32_t resolution)
    {
        out_indices.clear();

        if (resolution < 2)
        {
            return;
        }

        const size_t grid_index_count =
                static_cast<size_t>(resolution - 1u) * static_cast<size_t>(resolution - 1u) * 6u;
        const size_t skirt_index_count = static_cast<size_t>(4u) * static_cast<size_t>(resolution - 1u) * 6u;
        out_indices.reserve(grid_index_count + skirt_index_count);

        // Base grid indices
        for (uint32_t j = 0; j + 1 < resolution; ++j)
        {
            for (uint32_t i = 0; i + 1 < resolution; ++i)
            {
                const uint32_t i0 = j * resolution + i;
                const uint32_t i1 = i0 + 1;
                const uint32_t i2 = i0 + resolution;
                const uint32_t i3 = i2 + 1;

                // CCW winding when viewed from outside the sphere.
                out_indices.push_back(i0);
                out_indices.push_back(i1);
                out_indices.push_back(i2);

                out_indices.push_back(i2);
                out_indices.push_back(i1);
                out_indices.push_back(i3);
            }
        }

        auto add_skirt_quads = [&](uint32_t base0, uint32_t base1, uint32_t skirt0, uint32_t skirt1) {
            out_indices.push_back(base0);
            out_indices.push_back(base1);
            out_indices.push_back(skirt0);

            out_indices.push_back(skirt0);
            out_indices.push_back(base1);
            out_indices.push_back(skirt1);
        };

        const uint32_t base_vertex_count = resolution * resolution;
        const uint32_t top_skirt_start = base_vertex_count + 0u * resolution;
        const uint32_t right_skirt_start = base_vertex_count + 1u * resolution;
        const uint32_t bottom_skirt_start = base_vertex_count + 2u * resolution;
        const uint32_t left_skirt_start = base_vertex_count + 3u * resolution;

        // Skirt indices: 4 edges, (N-1) segments each.
        for (uint32_t i = 0; i + 1 < resolution; ++i)
        {
            // Top edge
            add_skirt_quads(0u * resolution + i,
                            0u * resolution + (i + 1u),
                            top_skirt_start + i,
                            top_skirt_start + (i + 1u));
            // Bottom edge
            add_skirt_quads((resolution - 1u) * resolution + i,
                            (resolution - 1u) * resolution + (i + 1u),
                            bottom_skirt_start + i,
                            bottom_skirt_start + (i + 1u));
        }
        for (uint32_t j = 0; j + 1 < resolution; ++j)
        {
            // Left edge
            add_skirt_quads(j * resolution + 0u,
                            (j + 1u) * resolution + 0u,
                            left_skirt_start + j,
                            left_skirt_start + (j + 1u));
            // Right edge
            add_skirt_quads(j * resolution + (resolution - 1u),
                            (j + 1u) * resolution + (resolution - 1u),
                            right_skirt_start + j,
                            right_skirt_start + (j + 1u));
        }
    }

    glm::dvec3 build_cubesphere_patch_vertices(std::vector<Vertex> &out_vertices,
                                               double radius_m,
                                               CubeFace face,
                                               uint32_t level,
                                               uint32_t x,
                                               uint32_t y,
                                               uint32_t resolution,
                                               const glm::vec4 &vertex_color)
    {
        out_vertices.clear();

        if (resolution < 2)
        {
            return glm::dvec3(0.0, 0.0, 1.0);
        }

        const glm::dvec3 patch_center_dir = cubesphere_patch_center_direction(face, level, x, y);

        const double skirt_depth_m = cubesphere_skirt_depth_m(radius_m, level);
        const double skirt_radius_m = glm::max(0.0, radius_m - skirt_depth_m);

        double u0 = 0.0, u1 = 0.0, v0 = 0.0, v1 = 0.0;
        cubesphere_tile_uv_bounds(level, x, y, u0, u1, v0, v1);

        const uint32_t base_vertex_count = resolution * resolution;
        const uint32_t skirt_vertex_count = 4u * resolution;
        out_vertices.resize(static_cast<size_t>(base_vertex_count) + static_cast<size_t>(skirt_vertex_count));

        const uint32_t tiles_per_axis = (level < 31u) ? (1u << level) : 1u;
        const double inv_tiles = (tiles_per_axis > 0u) ? (1.0 / static_cast<double>(tiles_per_axis)) : 1.0;

        const double inv = 1.0 / static_cast<double>(resolution - 1u);
        const double du = (u1 - u0) * inv;
        const double dv = (v1 - v0) * inv;
        for (uint32_t j = 0; j < resolution; ++j)
        {
            const float t = static_cast<float>(static_cast<double>(j) * inv);
            const double v = v0 + dv * static_cast<double>(j);

            for (uint32_t i = 0; i < resolution; ++i)
            {
                const float s = static_cast<float>(static_cast<double>(i) * inv);
                const double u = u0 + du * static_cast<double>(i);

                const glm::dvec3 unit_dir = cubesphere_unit_direction(face, u, v);
                const glm::dvec3 delta_d = (unit_dir - patch_center_dir) * radius_m;

                Vertex vert{};
                vert.position = glm::vec3(static_cast<float>(delta_d.x),
                                          static_cast<float>(delta_d.y),
                                          static_cast<float>(delta_d.z));
                vert.normal = glm::vec3(static_cast<float>(unit_dir.x),
                                        static_cast<float>(unit_dir.y),
                                        static_cast<float>(unit_dir.z));

                // UVs cover the entire cube face (0..1) so all patches on this face
                // sample from a single per-face texture.
                const double u_face = (static_cast<double>(x) + static_cast<double>(s)) * inv_tiles;
                const double v_face = (static_cast<double>(y) + static_cast<double>(t)) * inv_tiles;
                vert.uv_x = static_cast<float>(u_face);
                vert.uv_y = static_cast<float>(v_face);
                vert.color = vertex_color;
                vert.tangent = glm::vec4(1.0f, 0.0f, 0.0f, 1.0f);

                const uint32_t idx = j * resolution + i;
                out_vertices[idx] = vert;
            }
        }

        auto add_skirt_vertex = [&](uint32_t base_index, uint32_t skirt_index) {
            const glm::vec3 n = out_vertices[base_index].normal;
            const glm::dvec3 unit_dir(static_cast<double>(n.x),
                                      static_cast<double>(n.y),
                                      static_cast<double>(n.z));
            const glm::dvec3 delta_d = unit_dir * skirt_radius_m - patch_center_dir * radius_m;

            Vertex vert = out_vertices[base_index];
            vert.position = glm::vec3(static_cast<float>(delta_d.x),
                                      static_cast<float>(delta_d.y),
                                      static_cast<float>(delta_d.z));
            vert.normal = glm::vec3(static_cast<float>(unit_dir.x),
                                    static_cast<float>(unit_dir.y),
                                    static_cast<float>(unit_dir.z));
            out_vertices[skirt_index] = vert;
        };

        const uint32_t top_skirt_start = base_vertex_count + 0u * resolution;
        const uint32_t right_skirt_start = base_vertex_count + 1u * resolution;
        const uint32_t bottom_skirt_start = base_vertex_count + 2u * resolution;
        const uint32_t left_skirt_start = base_vertex_count + 3u * resolution;

        // Top edge (j=0)
        for (uint32_t i = 0; i < resolution; ++i)
        {
            add_skirt_vertex(0u * resolution + i, top_skirt_start + i);
        }
        // Right edge (i=resolution-1)
        for (uint32_t j = 0; j < resolution; ++j)
        {
            add_skirt_vertex(j * resolution + (resolution - 1u), right_skirt_start + j);
        }
        // Bottom edge (j=resolution-1)
        for (uint32_t i = 0; i < resolution; ++i)
        {
            add_skirt_vertex((resolution - 1u) * resolution + i, bottom_skirt_start + i);
        }
        // Left edge (i=0)
        for (uint32_t j = 0; j < resolution; ++j)
        {
            add_skirt_vertex(j * resolution + 0u, left_skirt_start + j);
        }

        return patch_center_dir;
    }

    void build_cubesphere_patch_mesh(CubeSpherePatchMesh &out,
                                     const WorldVec3 &center_world,
                                     double radius_m,
                                     CubeFace face,
                                     uint32_t level,
                                     uint32_t x,
                                     uint32_t y,
                                     uint32_t resolution,
                                     const glm::vec4 &vertex_color,
                                     bool generate_tangents)
    {
        out.vertices.clear();
        out.indices.clear();
        out.patch_center_world = center_world;

        if (resolution < 2)
        {
            return;
        }

        const glm::dvec3 patch_center_dir =
                build_cubesphere_patch_vertices(out.vertices, radius_m, face, level, x, y, resolution, vertex_color);
        build_cubesphere_patch_indices(out.indices, resolution);

        out.patch_center_world = center_world + patch_center_dir * radius_m;

        if (generate_tangents)
        {
            geom::generate_tangents(out.vertices, out.indices);
        }
    }
} // namespace planet
