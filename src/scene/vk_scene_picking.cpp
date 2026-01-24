#include "vk_scene.h"

#include "core/device/swapchain.h"
#include "core/device/images.h"
#include "core/context.h"
#include "mesh_bvh.h"
#include "scene/planet/planet_system.h"

#include "glm/gtx/transform.hpp"
#include <glm/gtc/matrix_transform.hpp>
#include "glm/gtx/norm.inl"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace
{
    struct BoundsHitDebug
    {
        bool usedBVH = false;
        bool bvhHit = false;
        bool fallbackBox = false;
    };

    // Ray / oriented-box intersection in world space using object-local AABB.
    // Returns true when hit; outWorldHit is the closest hit point in world space.
    bool intersect_ray_box(const glm::vec3 &rayOrigin,
                           const glm::vec3 &rayDir,
                           const Bounds &bounds,
                           const glm::mat4 &worldTransform,
                           glm::vec3 &outWorldHit)
    {
        if (glm::length2(rayDir) < 1e-8f)
        {
            return false;
        }

        // Transform ray into local space of the bounds for precise box test.
        glm::mat4 invM = glm::inverse(worldTransform);
        glm::vec3 localOrigin = glm::vec3(invM * glm::vec4(rayOrigin, 1.0f));
        glm::vec3 localDir = glm::vec3(invM * glm::vec4(rayDir, 0.0f));

        // Note: localDir length depends on object scale. Large objects (planets)
        // can shrink the direction vector after inverse-transform. Only reject
        // truly degenerate (zero/NaN) directions.
        const float localDirLen2 = glm::dot(localDir, localDir);
        if (!(localDirLen2 > 0.0f))
        {
            return false;
        }
        localDir = glm::normalize(localDir);

        glm::vec3 minB = bounds.origin - bounds.extents;
        glm::vec3 maxB = bounds.origin + bounds.extents;

        float tMin = 0.0f;
        float tMax = std::numeric_limits<float>::max();

        for (int axis = 0; axis < 3; ++axis)
        {
            float o = localOrigin[axis];
            float d = localDir[axis];
            if (std::abs(d) < 1e-8f)
            {
                // Ray parallel to slab: must be inside to intersect.
                if (o < minB[axis] || o > maxB[axis])
                {
                    return false;
                }
            }
            else
            {
                float invD = 1.0f / d;
                float t1 = (minB[axis] - o) * invD;
                float t2 = (maxB[axis] - o) * invD;
                if (t1 > t2)
                {
                    std::swap(t1, t2);
                }

                tMin = std::max(tMin, t1);
                tMax = std::min(tMax, t2);

                if (tMax < tMin)
                {
                    return false;
                }
            }
        }

        if (tMax < 0.0f)
        {
            return false;
        }

        // Choose the closest intersection in front of the ray origin.
        // If the ray starts inside the box (tMin <= 0), use the exit point tMax.
        float tHit = tMin;
        if (tHit <= 0.0f)
        {
            tHit = tMax;
        }
        if (tHit <= 0.0f)
        {
            return false;
        }

        glm::vec3 localHit = localOrigin + tHit * localDir;
        glm::vec3 worldHit = glm::vec3(worldTransform * glm::vec4(localHit, 1.0f));

        outWorldHit = worldHit;
        return true;
    }

    // Quick conservative ray / bounding-sphere test in world space.
    // Returns false when the ray misses the sphere; on hit, outT is the
    // closest positive intersection distance along the ray direction.
    bool intersect_ray_sphere(const glm::vec3 &rayOrigin,
                              const glm::vec3 &rayDir,
                              const Bounds &bounds,
                              const glm::mat4 &worldTransform,
                              float &outT)
    {
        // Sphere center is bounds.origin transformed to world.
        glm::vec3 centerWorld = glm::vec3(worldTransform * glm::vec4(bounds.origin, 1.0f));

        // Approximate world-space radius by scaling with the maximum axis scale.
        glm::vec3 sx = glm::vec3(worldTransform[0]);
        glm::vec3 sy = glm::vec3(worldTransform[1]);
        glm::vec3 sz = glm::vec3(worldTransform[2]);
        float maxScale = std::max({glm::length(sx), glm::length(sy), glm::length(sz)});
        float radiusWorld = bounds.sphereRadius * maxScale;
        if (radiusWorld <= 0.0f)
        {
            return false;
        }

        glm::vec3 oc = rayOrigin - centerWorld;
        float b = glm::dot(oc, rayDir);
        float c = glm::dot(oc, oc) - radiusWorld * radiusWorld;
        float disc = b * b - c;
        if (disc < 0.0f)
        {
            return false;
        }
        float s = std::sqrt(disc);
        float t0 = -b - s;
        float t1 = -b + s;
        float t = t0 >= 0.0f ? t0 : t1;
        if (t < 0.0f)
        {
            return false;
        }
        outT = t;
        return true;
    }

    // Ray / capsule intersection in world space. Capsule is aligned with local Y axis
    // and reconstructed from Bounds.origin/extents, assuming extents.x/z ~= radius
    // and extents.y ~= halfHeight + radius (AABB center/half-size convention).
    bool intersect_ray_capsule(const glm::vec3 &rayOrigin,
                               const glm::vec3 &rayDir,
                               const Bounds &bounds,
                               const glm::mat4 &worldTransform,
                               glm::vec3 &outWorldHit)
    {
        if (glm::length2(rayDir) < 1e-8f)
        {
            return false;
        }

        // Transform ray into object-local space.
        glm::mat4 invM = glm::inverse(worldTransform);
        glm::vec3 localOrigin = glm::vec3(invM * glm::vec4(rayOrigin, 1.0f));
        glm::vec3 localDir = glm::vec3(invM * glm::vec4(rayDir, 0.0f));
        // See intersect_ray_box() note about object scale and inverse-transformed directions.
        const float localDirLen2 = glm::dot(localDir, localDir);
        if (!(localDirLen2 > 0.0f))
        {
            return false;
        }
        localDir = glm::normalize(localDir);

        // Work in capsule-local space where Bounds.origin is at (0,0,0).
        glm::vec3 ro = localOrigin - bounds.origin;
        glm::vec3 rd = localDir;

        float radius = std::max(bounds.extents.x, bounds.extents.z);
        if (radius <= 0.0f)
        {
            return false;
        }
        // extents.y is (halfCylinder + radius) for a symmetric capsule.
        float halfSegment = std::max(bounds.extents.y - radius, 0.0f);

        float tHit = std::numeric_limits<float>::max();
        bool hit = false;

        // 1) Cylinder part around Y axis: x^2 + z^2 = r^2, |y| <= halfSegment.
        float a = rd.x * rd.x + rd.z * rd.z;
        float b = 2.0f * (ro.x * rd.x + ro.z * rd.z);
        float c = ro.x * ro.x + ro.z * ro.z - radius * radius;

        if (std::abs(a) > 1e-8f)
        {
            float disc = b * b - 4.0f * a * c;
            if (disc >= 0.0f)
            {
                float s = std::sqrt(disc);
                float invDen = 0.5f / a;
                float t0 = (-b - s) * invDen;
                float t1 = (-b + s) * invDen;
                if (t0 > t1) std::swap(t0, t1);

                auto tryCylHit = [&](float t)
                {
                    if (t < 0.0f || t >= tHit)
                    {
                        return;
                    }
                    glm::vec3 p = ro + rd * t;
                    if (std::abs(p.y) <= halfSegment + 1e-4f)
                    {
                        tHit = t;
                        hit = true;
                    }
                };

                tryCylHit(t0);
                tryCylHit(t1);
            }
        }

        // 2) Spherical caps at y = +/- halfSegment.
        auto intersectCap = [&](float capY)
        {
            glm::vec3 center(0.0f, capY, 0.0f);
            glm::vec3 oc = ro - center;
            float b2 = glm::dot(oc, rd);
            float c2 = glm::dot(oc, oc) - radius * radius;
            float disc = b2 * b2 - c2;
            if (disc < 0.0f)
            {
                return;
            }
            float s = std::sqrt(disc);
            float t0 = -b2 - s;
            float t1 = -b2 + s;

            auto tryCapHit = [&](float t)
            {
                if (t < 0.0f || t >= tHit)
                {
                    return;
                }
                tHit = t;
                hit = true;
            };

            if (t0 >= 0.0f) tryCapHit(t0);
            if (t1 >= 0.0f) tryCapHit(t1);
        };

        intersectCap(+halfSegment);
        intersectCap(-halfSegment);

        if (!hit)
        {
            return false;
        }

        glm::vec3 localHit = ro + rd * tHit;
        glm::vec3 worldHit = glm::vec3(worldTransform * glm::vec4(localHit + bounds.origin, 1.0f));

        if (glm::dot(worldHit - rayOrigin, rayDir) <= 0.0f)
        {
            return false;
        }

        outWorldHit = worldHit;
        return true;
    }

    // Ray / oriented-bounds intersection in world space using object-local shape.
    // For non-mesh shapes we use a quick world-space bounding-sphere pretest;
    // for mesh bounds we go directly to the mesh BVH (which already has a root AABB).
    // Returns true when hit; outWorldHit is the closest hit point in world space.
    bool intersect_ray_bounds(const glm::vec3 &rayOrigin,
                              const glm::vec3 &rayDir,
                              const RenderObject &obj,
                              glm::vec3 &outWorldHit,
                              BoundsHitDebug *debug,
                              MeshBVHPickHit *outMeshHit)
    {
        const Bounds &bounds = obj.bounds;
        const glm::mat4 &worldTransform = obj.transform;

        if (outMeshHit)
        {
            *outMeshHit = {};
        }

        // Non-pickable object.
        if (bounds.type == BoundsType::None)
        {
            return false;
        }

        if (glm::length2(rayDir) < 1e-8f)
        {
            return false;
        }

        switch (bounds.type)
        {
            case BoundsType::Sphere:
            {
                // Early reject using bounding sphere in world space.
                float sphereT = 0.0f;
                if (!intersect_ray_sphere(rayOrigin, rayDir, bounds, worldTransform, sphereT))
                {
                    return false;
                }
                // We already have the hit distance along the ray from the sphere test.
                outWorldHit = rayOrigin + rayDir * sphereT;
                return true;
            }
            case BoundsType::Capsule:
            {
                float sphereT = 0.0f;
                if (!intersect_ray_sphere(rayOrigin, rayDir, bounds, worldTransform, sphereT))
                {
                    return false;
                }
                return intersect_ray_capsule(rayOrigin, rayDir, bounds, worldTransform, outWorldHit);
            }
            case BoundsType::Mesh:
            {
                // For mesh bounds we rely solely on the CPU mesh BVH.
                // If there is no BVH or the BVH misses, the object is
                // treated as not hit by this ray (no coarse box fallback).
                if (!obj.sourceMesh || !obj.sourceMesh->bvh)
                {
                    return false;
                }

                if (debug)
                {
                    debug->usedBVH = true;
                }

                MeshBVHPickHit meshHit{};
                if (!intersect_ray_mesh_bvh(*obj.sourceMesh->bvh, worldTransform, rayOrigin, rayDir, meshHit))
                {
                    if (debug)
                    {
                        // BVH was queried but produced no hit.
                        debug->fallbackBox = true;
                    }
                    return false;
                }

                if (debug)
                {
                    debug->bvhHit = true;
                }

                outWorldHit = meshHit.worldPos;
                if (outMeshHit)
                {
                    *outMeshHit = meshHit;
                }
                return true;
            }
            case BoundsType::Box:
            default:
            {
                float sphereT = 0.0f;
                if (!intersect_ray_sphere(rayOrigin, rayDir, bounds, worldTransform, sphereT))
                {
                    return false;
                }
                return intersect_ray_box(rayOrigin, rayDir, bounds, worldTransform, outWorldHit);
            }
        }
    }

    // Test whether the clip-space box corners of an object intersect a 2D NDC rectangle.
    // ndcMin/ndcMax are in [-1,1]x[-1,1]. Returns true if any visible corner projects inside.
    bool box_overlaps_ndc_rect(const RenderObject &obj,
                               const glm::mat4 &viewproj,
                               const glm::vec2 &ndcMin,
                               const glm::vec2 &ndcMax)
    {
        const glm::vec3 o = obj.bounds.origin;
        const glm::vec3 e = obj.bounds.extents;
        const glm::mat4 m = viewproj * obj.transform; // world -> clip

        const std::array<glm::vec3, 8> corners{
            glm::vec3{+1, +1, +1}, glm::vec3{+1, +1, -1}, glm::vec3{+1, -1, +1}, glm::vec3{+1, -1, -1},
            glm::vec3{-1, +1, +1}, glm::vec3{-1, +1, -1}, glm::vec3{-1, -1, +1}, glm::vec3{-1, -1, -1},
        };

        for (const glm::vec3 &c : corners)
        {
            glm::vec3 pLocal = o + c * e;
            glm::vec4 clip = m * glm::vec4(pLocal, 1.f);
            if (clip.w <= 0.0f)
            {
                continue;
            }
            float x = clip.x / clip.w;
            float y = clip.y / clip.w;
            float z = clip.z / clip.w; // Vulkan Z0: 0..1
            if (z < 0.0f || z > 1.0f)
            {
                continue;
            }
            if (x >= ndcMin.x && x <= ndcMax.x &&
                y >= ndcMin.y && y <= ndcMax.y)
            {
                return true;
            }
        }
        return false;
    }
} // namespace

bool SceneManager::pick(const glm::vec2 &mousePosPixels, RenderObject &outObject, WorldVec3 &outWorldPos)
{
    if (_context == nullptr)
    {
        return false;
    }

    SwapchainManager *swapchain = _context->getSwapchain();
    if (swapchain == nullptr)
    {
        return false;
    }

    VkExtent2D dstExtent = swapchain->swapchainExtent();
    if (dstExtent.width == 0 || dstExtent.height == 0)
    {
        return false;
    }

    VkExtent2D logicalExtent{ kRenderWidth, kRenderHeight };
    if (_context)
    {
        VkExtent2D ctxLogical = _context->getLogicalRenderExtent();
        if (ctxLogical.width > 0 && ctxLogical.height > 0)
        {
            logicalExtent = ctxLogical;
        }
    }

    glm::vec2 logicalPos{};
    if (!vkutil::map_window_to_letterbox_src(mousePosPixels, logicalExtent, dstExtent, logicalPos))
    {
        return false;
    }

    float width = static_cast<float>(logicalExtent.width);
    float height = static_cast<float>(logicalExtent.height);

    // Convert from logical view coordinates (top-left origin) to NDC in [-1, 1].
    float ndcX = (2.0f * logicalPos.x / width) - 1.0f;
    float ndcY = 1.0f - (2.0f * logicalPos.y / height);

    float fovRad = glm::radians(mainCamera.fovDegrees);
    float tanHalfFov = std::tan(fovRad * 0.5f);
    float aspect = width / height;

    // Build ray in camera space using -Z forward convention.
    glm::vec3 dirCamera(ndcX * aspect * tanHalfFov,
                        ndcY * tanHalfFov,
                        -1.0f);
    dirCamera = glm::normalize(dirCamera);

    const WorldVec3 origin_world = get_world_origin();
    glm::vec3 rayOrigin = world_to_local(mainCamera.position_world, origin_world);
    glm::mat4 camRotation = mainCamera.getRotationMatrix();
    glm::vec3 rayDir = glm::normalize(glm::vec3(camRotation * glm::vec4(dirCamera, 0.0f)));

    bool anyHit = false;
    float bestDist2 = std::numeric_limits<float>::max();
    glm::vec3 bestHitPos{};

    // Reset debug info for this pick.
    pickingDebug = {};

    PlanetSystem *planets = get_planet_system();
    auto intersect_terrain_planet_sphere = [&](const RenderObject &obj, glm::vec3 &outHitPos) -> bool
    {
        if (!planets)
        {
            return false;
        }

        // Terrain patches are emitted as MeshInstance-owned RenderObjects but don't have a MeshAsset.
        // CPU picking against their AABBs can return points that are noticeably above/below the base
        // sphere (hundreds of meters at Earth scale). When terrain displacement is disabled, use an
        // analytic ray/sphere intersection against the planet base radius for a stable result.
        if (obj.ownerType != RenderObject::OwnerType::MeshInstance ||
            obj.ownerName.empty() ||
            obj.sourceMesh != nullptr)
        {
            return false;
        }

        PlanetSystem::PlanetBody *body = planets->find_body_by_name(obj.ownerName);
        if (!body || !body->terrain || !(body->terrain_height_max_m <= 0.0))
        {
            return false;
        }

        const glm::dvec3 ro = glm::dvec3(rayOrigin);
        const glm::dvec3 rd = glm::dvec3(rayDir);
        const glm::dvec3 center_local = body->center_world - origin_world;
        const double r = std::max(0.0, body->radius_m);
        if (!(r > 0.0))
        {
            return false;
        }

        const glm::dvec3 oc = ro - center_local;
        const double b = glm::dot(oc, rd);
        const double c = glm::dot(oc, oc) - r * r;
        const double disc = b * b - c;
        if (!(disc >= 0.0))
        {
            return false;
        }

        const double s = std::sqrt(disc);
        const double t0 = -b - s;
        const double t1 = -b + s;
        const double t = (t0 >= 0.0) ? t0 : t1;
        if (!(t >= 0.0))
        {
            return false;
        }

        const glm::dvec3 hit = ro + rd * t;
        outHitPos = glm::vec3(static_cast<float>(hit.x),
                              static_cast<float>(hit.y),
                              static_cast<float>(hit.z));
        return true;
    };

    auto testList = [&](const std::vector<RenderObject> &list)
    {
        for (const RenderObject &obj: list)
        {
            glm::vec3 hitPos{};
            BoundsHitDebug localDebug{};
            MeshBVHPickHit localMeshHit{};
            const bool hit =
                intersect_terrain_planet_sphere(obj, hitPos) ||
                intersect_ray_bounds(rayOrigin, rayDir, obj, hitPos, &localDebug, &localMeshHit);
            if (!hit)
            {
                continue;
            }

            float d2 = glm::length2(hitPos - rayOrigin);
            if (d2 < bestDist2)
            {
                bestDist2 = d2;
                bestHitPos = hitPos;
                outObject = obj;

                // If we have a precise mesh BVH hit, refine the picked
                // primitive to the exact triangle instead of the whole surface.
                if (localMeshHit.hit && outObject.sourceMesh && outObject.sourceMesh->bvh)
                {
                    outObject.firstIndex = localMeshHit.firstIndex;
                    outObject.indexCount = 3;
                    outObject.surfaceIndex = localMeshHit.surfaceIndex;
                }

                anyHit = true;

                // Capture debug info for the best hit so far.
                pickingDebug.usedMeshBVH = localDebug.usedBVH;
                pickingDebug.meshBVHHit = localDebug.bvhHit;
                pickingDebug.meshBVHFallbackBox = localDebug.fallbackBox;
                if (obj.sourceMesh && obj.sourceMesh->bvh)
                {
                    pickingDebug.meshBVHPrimCount =
                        static_cast<uint32_t>(obj.sourceMesh->bvh->primitives.size());
                    pickingDebug.meshBVHNodeCount =
                        static_cast<uint32_t>(obj.sourceMesh->bvh->nodes.size());
                }
                else
                {
                    pickingDebug.meshBVHPrimCount = 0;
                    pickingDebug.meshBVHNodeCount = 0;
                }
            }
        }
    };

    testList(mainDrawContext.OpaqueSurfaces);
    testList(mainDrawContext.TransparentSurfaces);

    if (anyHit)
    {
        outWorldPos = local_to_world(bestHitPos, origin_world);
    }

    return anyHit;
}

bool SceneManager::resolveObjectID(uint32_t id, RenderObject &outObject) const
{
    if (id == 0)
    {
        return false;
    }

    auto findIn = [&](const std::vector<RenderObject> &list) -> bool
    {
        for (const RenderObject &obj : list)
        {
            if (obj.objectID == id)
            {
                outObject = obj;
                return true;
            }
        }
        return false;
    };

    if (findIn(mainDrawContext.OpaqueSurfaces))
    {
        return true;
    }
    if (findIn(mainDrawContext.TransparentSurfaces))
    {
        return true;
    }
    return false;
}

void SceneManager::selectRect(const glm::vec2 &p0, const glm::vec2 &p1, std::vector<RenderObject> &outObjects) const
{
    if (!_context || !_context->getSwapchain())
    {
        return;
    }

    SwapchainManager *swapchain = _context->getSwapchain();
    VkExtent2D dstExtent = swapchain->swapchainExtent();
    if (dstExtent.width == 0 || dstExtent.height == 0)
    {
        return;
    }

    VkExtent2D logicalExtent{ kRenderWidth, kRenderHeight };
    VkExtent2D ctxLogical = _context->getLogicalRenderExtent();
    if (ctxLogical.width > 0 && ctxLogical.height > 0)
    {
        logicalExtent = ctxLogical;
    }

    VkRect2D activeRect = vkutil::compute_letterbox_rect(logicalExtent, dstExtent);
    if (activeRect.extent.width == 0 || activeRect.extent.height == 0)
    {
        return;
    }

    glm::vec2 selMin = glm::min(p0, p1);
    glm::vec2 selMax = glm::max(p0, p1);

    glm::vec2 activeMin{static_cast<float>(activeRect.offset.x),
                        static_cast<float>(activeRect.offset.y)};
    glm::vec2 activeMax{activeMin.x + static_cast<float>(activeRect.extent.width),
                        activeMin.y + static_cast<float>(activeRect.extent.height)};

    glm::vec2 clipMin = glm::max(selMin, activeMin);
    glm::vec2 clipMax = glm::min(selMax, activeMax);
    if (clipMax.x <= clipMin.x || clipMax.y <= clipMin.y)
    {
        return;
    }

    auto toLogical = [&](const glm::vec2 &p) -> glm::vec2
    {
        float localX = p.x - activeMin.x;
        float localY = p.y - activeMin.y;
        float u = localX / static_cast<float>(activeRect.extent.width);
        float v = localY / static_cast<float>(activeRect.extent.height);
        return glm::vec2{u * static_cast<float>(logicalExtent.width),
                         v * static_cast<float>(logicalExtent.height)};
    };

    glm::vec2 logical0 = toLogical(clipMin);
    glm::vec2 logical1 = toLogical(clipMax);

    float width = static_cast<float>(logicalExtent.width);
    float height = static_cast<float>(logicalExtent.height);

    // Convert from logical view coordinates (top-left origin) to NDC in [-1, 1].
    auto toNdc = [&](const glm::vec2 &p) -> glm::vec2
    {
        float ndcX = (2.0f * p.x / width) - 1.0f;
        float ndcY = 1.0f - (2.0f * p.y / height);
        return glm::vec2{ndcX, ndcY};
    };

    glm::vec2 ndc0 = toNdc(logical0);
    glm::vec2 ndc1 = toNdc(logical1);
    glm::vec2 ndcMin = glm::min(ndc0, ndc1);
    glm::vec2 ndcMax = glm::max(ndc0, ndc1);

    const glm::mat4 vp = sceneData.viewproj;

    auto testList = [&](const std::vector<RenderObject> &list)
    {
        for (const RenderObject &obj : list)
        {
            if (box_overlaps_ndc_rect(obj, vp, ndcMin, ndcMax))
            {
                outObjects.push_back(obj);
            }
        }
    };

    testList(mainDrawContext.OpaqueSurfaces);
    testList(mainDrawContext.TransparentSurfaces);
}
