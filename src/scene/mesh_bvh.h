#pragma once

#include <cstdint>
#include <memory>
#include <span>
#include <vector>

#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>

#include <core/types.h>
#include <bvh/BVH.h>

struct MeshAsset;

// For each BVH primitive, record which surface and which triangle
// (by starting index into the index buffer) it represents.
struct MeshBVHPrimitiveRef
{
    uint32_t surfaceIndex = 0;
    uint32_t firstIndex = 0;
};

// CPU-side BVH for a mesh, built in mesh-local space.
struct MeshBVH
{
    std::vector<PrimitiveF> primitives;
    std::vector<BVHNodeF> nodes;
    std::vector<MeshBVHPrimitiveRef> primitiveRefs;
};

// Build a mesh-local BVH for a triangle mesh.
// vertices/indices must match the GPU data uploaded for 'mesh'.
// Returns nullptr if no triangles are found.
std::unique_ptr<MeshBVH> build_mesh_bvh(const MeshAsset &mesh,
                                        std::span<const Vertex> vertices,
                                        std::span<const uint32_t> indices);

struct MeshBVHPickHit
{
    bool hit = false;

    float t = 0.0f;                 // world-space distance along ray
    glm::vec3 localPos{0.0f};       // hit position in mesh-local space
    glm::vec3 worldPos{0.0f};       // hit position in world space

    uint32_t surfaceIndex = 0;      // hit GeoSurface index
    uint32_t firstIndex = 0;        // index into mesh index buffer (triangle start)
};

// Ray–mesh BVH intersection in world space.
// Returns true on hit and fills outHit.
bool intersect_ray_mesh_bvh(const MeshBVH &bvh,
                            const glm::mat4 &worldTransform,
                            const glm::vec3 &rayOriginWorld,
                            const glm::vec3 &rayDirWorld,
                            MeshBVHPickHit &outHit);

