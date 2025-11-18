#include "vk_scene.h"

#include "vk_swapchain.h"
#include "core/engine_context.h"

#include "glm/gtx/transform.hpp"
#include <glm/gtc/matrix_transform.hpp>
#include "glm/gtx/norm.inl"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace
{
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

        if (glm::length2(localDir) < 1e-8f)
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

        float tHit = (tMin >= 0.0f) ? tMin : tMax;
        glm::vec3 localHit = localOrigin + tHit * localDir;
        glm::vec3 worldHit = glm::vec3(worldTransform * glm::vec4(localHit, 1.0f));

        if (glm::dot(worldHit - rayOrigin, rayDir) <= 0.0f)
        {
            return false;
        }

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

    // Ray / oriented-bounds intersection in world space using object-local shape.
    // Uses a quick sphere test first; on success refines based on BoundsType.
    // Returns true when hit; outWorldHit is the closest hit point in world space.
    bool intersect_ray_bounds(const glm::vec3 &rayOrigin,
                              const glm::vec3 &rayDir,
                              const Bounds &bounds,
                              const glm::mat4 &worldTransform,
                              glm::vec3 &outWorldHit)
    {
        // Non-pickable object.
        if (bounds.type == BoundsType::None)
        {
            return false;
        }

        if (glm::length2(rayDir) < 1e-8f)
        {
            return false;
        }

        // Early reject using bounding sphere in world space.
        float sphereT = 0.0f;
        if (!intersect_ray_sphere(rayOrigin, rayDir, bounds, worldTransform, sphereT))
        {
            return false;
        }

        // Shape-specific refinement after the conservative sphere test.
        switch (bounds.type)
        {
            case BoundsType::Sphere:
            {
                // We already have the hit distance along the ray from the sphere test.
                outWorldHit = rayOrigin + rayDir * sphereT;
                return true;
            }
            case BoundsType::Box:
            case BoundsType::Mesh: // TODO: replace with BVH/mesh query; box is a safe fallback.
            case BoundsType::Capsule:
            default:
            {
                // For Capsule and Mesh we currently fall back to the oriented box;
                // this still benefits from tighter AABBs if you author them.
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

bool SceneManager::pick(const glm::vec2 &mousePosPixels, RenderObject &outObject, glm::vec3 &outWorldPos)
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

    VkExtent2D extent = swapchain->windowExtent();
    if (extent.width == 0 || extent.height == 0)
    {
        return false;
    }

    float width = static_cast<float>(extent.width);
    float height = static_cast<float>(extent.height);

    // Convert from window coordinates (top-left origin) to NDC in [-1, 1].
    float ndcX = (2.0f * mousePosPixels.x / width) - 1.0f;
    float ndcY = 1.0f - (2.0f * mousePosPixels.y / height);

    float fovRad = glm::radians(mainCamera.fovDegrees);
    float tanHalfFov = std::tan(fovRad * 0.5f);
    float aspect = width / height;

    // Build ray in camera space using -Z forward convention.
    glm::vec3 dirCamera(ndcX * aspect * tanHalfFov,
                        ndcY * tanHalfFov,
                        -1.0f);
    dirCamera = glm::normalize(dirCamera);

    glm::vec3 rayOrigin = mainCamera.position;
    glm::mat4 camRotation = mainCamera.getRotationMatrix();
    glm::vec3 rayDir = glm::normalize(glm::vec3(camRotation * glm::vec4(dirCamera, 0.0f)));

    bool anyHit = false;
    float bestDist2 = std::numeric_limits<float>::max();
    glm::vec3 bestHitPos{};

    auto testList = [&](const std::vector<RenderObject> &list)
    {
        for (const RenderObject &obj: list)
        {
            glm::vec3 hitPos{};
            if (!intersect_ray_bounds(rayOrigin, rayDir, obj.bounds, obj.transform, hitPos))
            {
                continue;
            }

            float d2 = glm::length2(hitPos - rayOrigin);
            if (d2 < bestDist2)
            {
                bestDist2 = d2;
                bestHitPos = hitPos;
                outObject = obj;
                anyHit = true;
            }
        }
    };

    testList(mainDrawContext.OpaqueSurfaces);
    testList(mainDrawContext.TransparentSurfaces);

    if (anyHit)
    {
        outWorldPos = bestHitPos;
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

    VkExtent2D extent = _context->getSwapchain()->windowExtent();
    if (extent.width == 0 || extent.height == 0)
    {
        return;
    }

    float width = static_cast<float>(extent.width);
    float height = static_cast<float>(extent.height);

    // Convert from window coordinates (top-left origin) to NDC in [-1, 1].
    auto toNdc = [&](const glm::vec2 &p) -> glm::vec2
    {
        float ndcX = (2.0f * p.x / width) - 1.0f;
        float ndcY = 1.0f - (2.0f * p.y / height);
        return glm::vec2{ndcX, ndcY};
    };

    glm::vec2 ndc0 = toNdc(p0);
    glm::vec2 ndc1 = toNdc(p1);
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

