#include "vk_scene.h"

#include <utility>
#include <unordered_set>
#include <chrono>

#include "vk_swapchain.h"
#include "core/engine_context.h"
#include "core/config.h"
#include "glm/gtx/transform.hpp"
#include <glm/gtc/matrix_transform.hpp>
#include "glm/gtx/norm.inl"
#include "glm/gtx/compatibility.hpp"
#include <algorithm>
#include <limits>
#include <cmath>
#include "core/config.h"

namespace
{
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

    // Ray / oriented-bounds intersection in world space using object-local AABB.
    // Uses a quick sphere test first; on success refines with OBB slabs.
    // Returns true when hit; outWorldHit is the closest hit point in world space.
    bool intersect_ray_bounds(const glm::vec3 &rayOrigin,
                              const glm::vec3 &rayDir,
                              const Bounds &bounds,
                              const glm::mat4 &worldTransform,
                              glm::vec3 &outWorldHit)
    {
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

void SceneManager::init(EngineContext *context)
{
    _context = context;

    mainCamera.velocity = glm::vec3(0.f);
    mainCamera.position = glm::vec3(30.f, -00.f, 85.f);
    mainCamera.pitch = 0;
    mainCamera.yaw = 0;

    sceneData.ambientColor = glm::vec4(0.1f, 0.1f, 0.1f, 1.0f);
    sceneData.sunlightDirection = glm::vec4(-0.2f, -1.0f, -0.3f, 1.0f);
    sceneData.sunlightColor = glm::vec4(1.0f, 1.0f, 1.0f, 3.0f);
}

void SceneManager::update_scene()
{
    auto start = std::chrono::system_clock::now();

    mainDrawContext.OpaqueSurfaces.clear();
    mainDrawContext.TransparentSurfaces.clear();
    mainDrawContext.nextID = 1;

    mainCamera.update();

    // Simple per-frame dt (seconds) for animations
    static auto lastFrameTime = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    float dt = std::chrono::duration<float>(now - lastFrameTime).count();
    lastFrameTime = now;
    if (dt < 0.f)
    {
        dt = 0.f;
    }
    if (dt > 0.1f)
    {
        dt = 0.1f;
    }

    // Advance glTF animations once per unique LoadedGLTF
    if (dt > 0.f)
    {
        std::unordered_set<LoadedGLTF *> animatedScenes;

        auto updateSceneAnim = [&](std::shared_ptr<LoadedGLTF> &scene) {
            if (!scene) return;
            LoadedGLTF *ptr = scene.get();
            if (animatedScenes.insert(ptr).second)
            {
                ptr->updateAnimation(dt);
            }
        };

        for (auto &[name, scene] : loadedScenes)
        {
            updateSceneAnim(scene);
        }
        for (auto &[name, inst] : dynamicGLTFInstances)
        {
            updateSceneAnim(inst.scene);
        }
    }

    // Draw all loaded GLTF scenes (static world)
    for (auto &[name, scene] : loadedScenes)
    {
        if (scene)
        {
            scene->Draw(glm::mat4{1.f}, mainDrawContext);
        }
    }

    // dynamic GLTF instances
    for (const auto &kv: dynamicGLTFInstances)
    {
        const GLTFInstance &inst = kv.second;
        if (inst.scene)
        {
            inst.scene->Draw(inst.transform, mainDrawContext);
        }
    }

    // Default primitives are added as dynamic instances by the engine.

    // dynamic mesh instances
    for (const auto &kv: dynamicMeshInstances)
    {
        const MeshInstance &inst = kv.second;
        if (!inst.mesh || inst.mesh->surfaces.empty()) continue;
        uint32_t surfaceIndex = 0;
        for (const auto &surf: inst.mesh->surfaces)
        {
            RenderObject obj{};
            obj.indexCount = surf.count;
            obj.firstIndex = surf.startIndex;
            obj.indexBuffer = inst.mesh->meshBuffers.indexBuffer.buffer;
            obj.vertexBuffer = inst.mesh->meshBuffers.vertexBuffer.buffer;
            obj.vertexBufferAddress = inst.mesh->meshBuffers.vertexBufferAddress;
            obj.material = &surf.material->data;
            obj.bounds = surf.bounds;
            obj.transform = inst.transform;
            obj.sourceMesh = inst.mesh.get();
            obj.surfaceIndex = surfaceIndex++;
            obj.objectID = mainDrawContext.nextID++;
            if (obj.material->passType == MaterialPass::Transparent)
            {
                mainDrawContext.TransparentSurfaces.push_back(obj);
            }
            else
            {
                mainDrawContext.OpaqueSurfaces.push_back(obj);
            }
        }
    }

    glm::mat4 view = mainCamera.getViewMatrix();
    // Use reversed infinite-Z projection (right-handed, -Z forward) to avoid far-plane clipping
    // on very large scenes. Vulkan clip space is 0..1 (GLM_FORCE_DEPTH_ZERO_TO_ONE) and requires Y flip.
    auto makeReversedInfinitePerspective = [](float fovyRadians, float aspect, float zNear) {
        // Column-major matrix; indices are [column][row]
        float f = 1.0f / tanf(fovyRadians * 0.5f);
        glm::mat4 m(0.0f);
        m[0][0] = f / aspect;
        m[1][1] = f;
        m[2][2] = 0.0f;
        m[2][3] = -1.0f; // w = -z_eye (right-handed)
        m[3][2] = zNear; // maps near -> 1, far -> 0 (reversed-Z)
        return m;
    };

    const float fov = glm::radians(70.f);
    const float aspect = (float) _context->getSwapchain()->windowExtent().width /
                         (float) _context->getSwapchain()->windowExtent().height;
    const float nearPlane = 0.1f;
    glm::mat4 projection = makeReversedInfinitePerspective(fov, aspect, nearPlane);
    // Vulkan NDC has inverted Y.
    projection[1][1] *= -1.0f;

    sceneData.view = view;
    sceneData.proj = projection;
    sceneData.viewproj = projection * view;

    // Clipmap shadow setup (directional). Each level i covers a square region
    // around the camera in the light's XY plane with radius R_i = R0 * 2^i.
    // The region center is snapped to the light-space texel grid for stability.
    static const float kAheadBlend[kShadowCascadeCount] = {0.2f, 0.5f, 0.75f, 1.0f};
    {
        const glm::mat4 invView = glm::inverse(view);
        const glm::vec3 camPos = glm::vec3(invView[3]);
        const glm::vec3 camFwd  = -glm::vec3(invView[2]);

        glm::vec3 L = glm::normalize(-glm::vec3(sceneData.sunlightDirection));
        if (glm::length(L) < 1e-5f) L = glm::vec3(0.0f, -1.0f, 0.0f);
        const glm::vec3 worldUp(0.0f, 1.0f, 0.0f);
        glm::vec3 right = glm::cross(L, worldUp);
        if (glm::length2(right) < 1e-6f) right = glm::vec3(1, 0, 0);
        right = glm::normalize(right);
        glm::vec3 up = glm::normalize(glm::cross(right, L));

        auto level_radius = [](int level) {
            return kShadowClipBaseRadius * powf(2.0f, float(level));
        };

        sceneData.cascadeSplitsView = glm::vec4(
            level_radius(0), level_radius(1), level_radius(2), level_radius(3));

        for (int ci = 0; ci < kShadowCascadeCount; ++ci)
        {
            const float radius = level_radius(ci);
            const float cover = radius * kShadowCascadeRadiusScale + kShadowCascadeRadiusMargin;

            const float ahead = radius * 0.5;
            const float fu = glm::dot(camFwd, right);
            const float fv = glm::dot(camFwd, up);

            const glm::vec3 aheadXY = right * (fu * ahead) + up * (fv * ahead);

            const float u = glm::dot(camPos + aheadXY, right) + fu * ahead;
            const float v = glm::dot(camPos + aheadXY, up) + fv * ahead;

            const float texel = (2.0f * cover) / float(kShadowMapResolution);
            const float uSnapped = floorf(u / texel) * texel;
            const float vSnapped = floorf(v / texel) * texel;
            const float du = uSnapped - u;
            const float dv = vSnapped - v;

            const glm::vec3 center = camPos + aheadXY * kAheadBlend[ci] + right * du + up * dv;

            const float pullback = glm::max(kShadowClipPullbackMin, cover * kShadowClipPullbackFactor);
            const glm::vec3 eye = center - L * pullback;
            const glm::mat4 V = glm::lookAtRH(eye, center, up);

            const float zNear = 0.2f;
            const float zFar = pullback + cover * kShadowClipForwardFactor + kShadowClipZPadding;

            const glm::mat4 P = glm::orthoRH_ZO(-cover, cover, -cover, cover, zNear, zFar);
            const glm::mat4 lightVP = P * V;

            sceneData.lightViewProjCascades[ci] = lightVP;
            if (ci == 0)
            {
                sceneData.lightViewProj = lightVP;
            }
        }
    }

    // Publish shadow/RT settings to SceneData
    if (_context)
    {
        const auto &ss = _context->shadowSettings;
        const uint32_t rtEnabled = (ss.mode != 0) ? 1u : 0u;
        sceneData.rtOptions = glm::uvec4(rtEnabled, ss.hybridRayCascadesMask, ss.mode, 0u);
        sceneData.rtParams  = glm::vec4(ss.hybridRayNoLThreshold, 0.0f, 0.0f, 0.0f);
    }

    auto end = std::chrono::system_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    stats.scene_update_time = elapsed.count() / 1000.f;
}

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

void SceneManager::loadScene(const std::string &name, std::shared_ptr<LoadedGLTF> scene)
{
    if (scene)
    {
        scene->debugName = name;
    }
    loadedScenes[name] = std::move(scene);
}

std::shared_ptr<LoadedGLTF> SceneManager::getScene(const std::string &name)
{
    auto it = loadedScenes.find(name);
    return (it != loadedScenes.end()) ? it->second : nullptr;
}

void SceneManager::cleanup()
{
    // Explicitly clear dynamic instances first to drop any extra shared_ptrs
    // that could keep GPU resources alive.
    clearMeshInstances();
    clearGLTFInstances();

    // Drop our references to GLTF scenes. Their destructors call clearAll()
    // exactly once to release GPU resources.
    loadedScenes.clear();
    loadedNodes.clear();
}

void SceneManager::addMeshInstance(const std::string &name, std::shared_ptr<MeshAsset> mesh, const glm::mat4 &transform)
{
    if (!mesh) return;
    dynamicMeshInstances[name] = MeshInstance{std::move(mesh), transform};
}

bool SceneManager::removeMeshInstance(const std::string &name)
{
    return dynamicMeshInstances.erase(name) > 0;
}

void SceneManager::clearMeshInstances()
{
    dynamicMeshInstances.clear();
}

void SceneManager::addGLTFInstance(const std::string &name, std::shared_ptr<LoadedGLTF> scene,
                                   const glm::mat4 &transform)
{
    if (!scene) return;
    dynamicGLTFInstances[name] = GLTFInstance{std::move(scene), transform};
}

bool SceneManager::removeGLTFInstance(const std::string &name)
{
    return dynamicGLTFInstances.erase(name) > 0;
}

bool SceneManager::setGLTFInstanceTransform(const std::string &name, const glm::mat4 &transform)
{
    auto it = dynamicGLTFInstances.find(name);
    if (it == dynamicGLTFInstances.end()) return false;
    it->second.transform = transform;
    return true;
}

void SceneManager::clearGLTFInstances()
{
    dynamicGLTFInstances.clear();
}

bool SceneManager::setSceneAnimation(const std::string &sceneName, int animationIndex, bool resetTime)
{
    auto it = loadedScenes.find(sceneName);
    if (it == loadedScenes.end() || !it->second)
    {
        return false;
    }

    it->second->setActiveAnimation(animationIndex, resetTime);
    return true;
}

bool SceneManager::setSceneAnimation(const std::string &sceneName, const std::string &animationName, bool resetTime)
{
    auto it = loadedScenes.find(sceneName);
    if (it == loadedScenes.end() || !it->second)
    {
        return false;
    }

    it->second->setActiveAnimation(animationName, resetTime);
    return true;
}

bool SceneManager::setSceneAnimationLoop(const std::string &sceneName, bool loop)
{
    auto it = loadedScenes.find(sceneName);
    if (it == loadedScenes.end() || !it->second)
    {
        return false;
    }

    it->second->animationLoop = loop;
    return true;
}

bool SceneManager::setGLTFInstanceAnimation(const std::string &instanceName, int animationIndex, bool resetTime)
{
    auto it = dynamicGLTFInstances.find(instanceName);
    if (it == dynamicGLTFInstances.end() || !it->second.scene)
    {
        return false;
    }

    it->second.scene->setActiveAnimation(animationIndex, resetTime);
    return true;
}

bool SceneManager::setGLTFInstanceAnimation(const std::string &instanceName, const std::string &animationName, bool resetTime)
{
    auto it = dynamicGLTFInstances.find(instanceName);
    if (it == dynamicGLTFInstances.end() || !it->second.scene)
    {
        return false;
    }

    it->second.scene->setActiveAnimation(animationName, resetTime);
    return true;
}

bool SceneManager::setGLTFInstanceAnimationLoop(const std::string &instanceName, bool loop)
{
    auto it = dynamicGLTFInstances.find(instanceName);
    if (it == dynamicGLTFInstances.end() || !it->second.scene)
    {
        return false;
    }

    it->second.scene->animationLoop = loop;
    return true;
}
