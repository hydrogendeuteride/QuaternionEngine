#pragma once

#include <core/types.h>
#include <core/world.h>

#include <cstdint>
#include <vector>

#include <glm/vec4.hpp>

namespace planet
{
    // Cube face ordering matches KTX/Vulkan cubemap face order:
    // +X, -X, +Y, -Y, +Z, -Z
    enum class CubeFace : uint8_t
    {
        PosX = 0,
        NegX = 1,
        PosY = 2,
        NegY = 3,
        PosZ = 4,
        NegZ = 5,
    };

    // Returns a short name for the cube face (e.g., "px", "nx", "py", ...).
    // Useful for constructing file paths like "{dir}/px.ktx2".
    inline const char *cube_face_name(CubeFace face)
    {
        switch (face)
        {
            case CubeFace::PosX: return "px";
            case CubeFace::NegX: return "nx";
            case CubeFace::PosY: return "py";
            case CubeFace::NegY: return "ny";
            case CubeFace::PosZ: return "pz";
            case CubeFace::NegZ: return "nz";
        }
        return "px";
    }

    // u,v are in [-1,+1] on the chosen face. Convention:
    // - u increases to the right
    // - v increases downward (image space)
    glm::dvec3 cubesphere_unit_direction(CubeFace face, double u, double v);

    // Map a direction to a cube face and face UVs in [0..1] range.
    // Convention matches cubesphere_unit_direction(): u increases right, v increases down.
    // Returns false if dir is degenerate.
    bool cubesphere_direction_to_face_uv(const glm::dvec3 &dir,
                                         CubeFace &out_face,
                                         double &out_u01,
                                         double &out_v01);

    // Tile bounds on a face in cube-face parametric space:
    // u,v in [-1,+1], where [0..1] maps to [-1..+1].
    void cubesphere_tile_uv_bounds(uint32_t level, uint32_t x, uint32_t y,
                                   double &out_u0, double &out_u1,
                                   double &out_v0, double &out_v1);

    glm::dvec3 cubesphere_patch_center_direction(CubeFace face, uint32_t level, uint32_t x, uint32_t y);

    WorldVec3 cubesphere_patch_center_world(const WorldVec3 &center_world,
                                            double radius_m,
                                            CubeFace face,
                                            uint32_t level,
                                            uint32_t x,
                                            uint32_t y);

    // Approximate world-space tile edge length on the sphere surface.
    double cubesphere_patch_edge_m(double radius_m, uint32_t level);

    // Skirt depth heuristic (meters).
    double cubesphere_skirt_depth_m(double radius_m, uint32_t level);

    struct CubeSpherePatchMesh
    {
        std::vector<Vertex> vertices;
        std::vector<uint32_t> indices;
        WorldVec3 patch_center_world{0.0, 0.0, 0.0};
    };

    // Build the shared index list for a patch grid with skirts. Indices are identical for all
    // patches as long as 'resolution' is constant.
    void build_cubesphere_patch_indices(std::vector<uint32_t> &out_indices, uint32_t resolution);

    // Build patch vertices (including skirts). Vertex positions are relative to the patch center on
    // the sphere surface (computed from face/level/x/y). Returns the patch center direction.
    glm::dvec3 build_cubesphere_patch_vertices(std::vector<Vertex> &out_vertices,
                                               double radius_m,
                                               CubeFace face,
                                               uint32_t level,
                                               uint32_t x,
                                               uint32_t y,
                                               uint32_t resolution,
                                               const glm::vec4 &vertex_color);

    // Build a cube-sphere patch mesh with skirts. Vertex positions are relative to patch_center_world.
    void build_cubesphere_patch_mesh(CubeSpherePatchMesh &out,
                                     const WorldVec3 &center_world,
                                     double radius_m,
                                     CubeFace face,
                                     uint32_t level,
                                     uint32_t x,
                                     uint32_t y,
                                     uint32_t resolution,
                                     const glm::vec4 &vertex_color,
                                     bool generate_tangents = true);
} // namespace planet
