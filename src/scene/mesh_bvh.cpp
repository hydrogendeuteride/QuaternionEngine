#include "mesh_bvh.h"

#include <limits>
#include <thread>

#include <scene/vk_loader.h>

#include "glm/gtx/norm.hpp"

std::unique_ptr<MeshBVH> build_mesh_bvh(const MeshAsset &mesh,
                                        std::span<const Vertex> vertices,
                                        std::span<const uint32_t> indices)
{
    if (vertices.empty() || indices.size() < 3 || mesh.surfaces.empty())
    {
        return {};
    }

    size_t totalTriangles = 0;
    for (const GeoSurface &surf: mesh.surfaces)
    {
        totalTriangles += static_cast<size_t>(surf.count / 3);
    }
    if (totalTriangles == 0)
    {
        return {};
    }

    auto result = std::make_unique<MeshBVH>();
    result->primitives.reserve(totalTriangles);
    result->primitiveRefs.reserve(totalTriangles);

    for (uint32_t s = 0; s < mesh.surfaces.size(); ++s)
    {
        const GeoSurface &surf = mesh.surfaces[s];
        uint32_t start = surf.startIndex;
        uint32_t end = surf.startIndex + surf.count;
        if (start >= indices.size())
        {
            continue;
        }
        if (end > indices.size())
        {
            end = static_cast<uint32_t>(indices.size());
        }

        for (uint32_t idx = start; idx + 2 < end; idx += 3)
        {
            uint32_t i0 = indices[idx + 0];
            uint32_t i1 = indices[idx + 1];
            uint32_t i2 = indices[idx + 2];
            if (i0 >= vertices.size() || i1 >= vertices.size() || i2 >= vertices.size())
            {
                continue;
            }

            const glm::vec3 &p0 = vertices[i0].position;
            const glm::vec3 &p1 = vertices[i1].position;
            const glm::vec3 &p2 = vertices[i2].position;

            // BVH2 now expects triangle primitives with explicit vertices.
            // Store the triangle in mesh-local space and let the library
            // compute/update the AABB used for hierarchy construction.
            PrimitiveF prim{};
            prim.v0 = Vec3<float>(p0.x, p0.y, p0.z);
            prim.v1 = Vec3<float>(p1.x, p1.y, p1.z);
            prim.v2 = Vec3<float>(p2.x, p2.y, p2.z);
            prim.updateBounds();
            result->primitives.push_back(prim);

            MeshBVHPrimitiveRef ref{};
            ref.surfaceIndex = s;
            ref.firstIndex = idx;
            result->primitiveRefs.push_back(ref);
        }
    }

    if (result->primitives.empty())
    {
        return {};
    }

    unsigned int threadCount = std::thread::hardware_concurrency();
    if (threadCount == 0)
    {
        threadCount = 1;
    }

    tf::Executor executor{threadCount};
    result->nodes = buildLBVH<uint64_t>(executor, result->primitives, MortonSortMethod::RadixSort);

    return result;
}

bool intersect_ray_mesh_bvh(const MeshBVH &bvh,
                            const glm::mat4 &worldTransform,
                            const glm::vec3 &rayOriginWorld,
                            const glm::vec3 &rayDirWorld,
                            MeshBVHPickHit &outHit)
{
    outHit = {};

    if (bvh.nodes.empty() || bvh.primitives.empty())
    {
        return false;
    }

    if (glm::length2(rayDirWorld) < 1e-8f)
    {
        return false;
    }

    glm::mat4 invM = glm::inverse(worldTransform);
    glm::vec3 originLocal = glm::vec3(invM * glm::vec4(rayOriginWorld, 1.0f));
    glm::vec3 dirLocal = glm::vec3(invM * glm::vec4(rayDirWorld, 0.0f));

    if (glm::length2(dirLocal) < 1e-8f)
    {
        return false;
    }
    dirLocal = glm::normalize(dirLocal);

    Ray ray(Vec3<float>(originLocal.x, originLocal.y, originLocal.z),
            Vec3<float>(dirLocal.x, dirLocal.y, dirLocal.z));

    uint32_t primIdx = 0;
    float tLocal = 0.0f;
    if (!traverseBVHClosestHit<float>(bvh.nodes, bvh.primitives, ray, primIdx, tLocal))
    {
        return false;
    }
    if (primIdx >= bvh.primitiveRefs.size())
    {
        return false;
    }

    const MeshBVHPrimitiveRef &ref = bvh.primitiveRefs[primIdx];

    glm::vec3 localHit = originLocal + dirLocal * tLocal;
    glm::vec3 worldHit = glm::vec3(worldTransform * glm::vec4(localHit, 1.0f));

    outHit.hit = true;
    outHit.localPos = localHit;
    outHit.worldPos = worldHit;
    outHit.surfaceIndex = ref.surfaceIndex;
    outHit.firstIndex = ref.firstIndex;
    outHit.t = glm::length(worldHit - rayOriginWorld);

    return true;
}
