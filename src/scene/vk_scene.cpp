#include "vk_scene.h"

#include <utility>
#include <unordered_set>
#include <chrono>

#include "core/device/swapchain.h"
#include "core/context.h"
#include "core/config.h"
#include "glm/gtx/transform.hpp"
#include <glm/gtc/matrix_transform.hpp>
#include "glm/gtx/norm.inl"
#include "glm/gtx/compatibility.hpp"
#include <algorithm>
#include <limits>
#include <cmath>

#include "core/frame/resources.h"
#include "core/config.h"
#include <fmt/core.h>

SceneManager::~SceneManager()
{
    fmt::println("[SceneManager] dtor: loadedScenes={} dynamicGLTFInstances={} pendingGLTFRelease={}",
                 loadedScenes.size(),
                 dynamicGLTFInstances.size(),
                 pendingGLTFRelease.size());
}

void SceneManager::addPointLight(const PointLight &light)
{
    pointLights.push_back(light);
}

void SceneManager::clearPointLights()
{
    pointLights.clear();
}

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

    // Seed a couple of default point lights for quick testing.
    PointLight warmKey{};
    warmKey.position = glm::vec3(0.0f, 0.0f, 0.0f);
    warmKey.radius = 25.0f;
    warmKey.color = glm::vec3(1.0f, 0.95f, 0.8f);
    warmKey.intensity = 15.0f;
    addPointLight(warmKey);

    PointLight coolFill{};
    coolFill.position = glm::vec3(-10.0f, 4.0f, 10.0f);
    coolFill.radius = 20.0f;
    coolFill.color = glm::vec3(0.6f, 0.7f, 1.0f);
    coolFill.intensity = 10.0f;
    addPointLight(coolFill);
}

void SceneManager::update_scene()
{
    auto start = std::chrono::system_clock::now();

    // Release any GLTF assets that were scheduled for safe destruction after GPU idle.
    // Defer actual destruction to the current frame's deletion queue so we wait
    // until the GPU has finished work that might still reference their resources.
    if (_context && _context->currentFrame)
    {
        if (!pendingGLTFRelease.empty())
        {
            fmt::println("[SceneManager] update_scene: scheduling {} pending GLTF releases (hasContext={}, hasFrame={})",
                         pendingGLTFRelease.size(),
                         true,
                         true);
        }
        for (auto &sp : pendingGLTFRelease)
        {
            auto keepAlive = sp; // copy to keep ref count in the lambda
            _context->currentFrame->_deletionQueue.push_function([keepAlive]() mutable { keepAlive.reset(); });
        }
    }
    pendingGLTFRelease.clear();

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

    auto tagOwner = [&](RenderObject::OwnerType type, const std::string &name,
                        size_t opaqueBegin, size_t transpBegin)
    {
        for (size_t i = opaqueBegin; i < mainDrawContext.OpaqueSurfaces.size(); ++i)
        {
            mainDrawContext.OpaqueSurfaces[i].ownerType = type;
            mainDrawContext.OpaqueSurfaces[i].ownerName = name;
        }
        for (size_t i = transpBegin; i < mainDrawContext.TransparentSurfaces.size(); ++i)
        {
            mainDrawContext.TransparentSurfaces[i].ownerType = type;
            mainDrawContext.TransparentSurfaces[i].ownerName = name;
        }
    };

    // Draw all loaded GLTF scenes (static world)
    for (auto &[name, scene] : loadedScenes)
    {
        if (scene)
        {
            const size_t opaqueStart = mainDrawContext.OpaqueSurfaces.size();
            const size_t transpStart = mainDrawContext.TransparentSurfaces.size();
            scene->Draw(glm::mat4{1.f}, mainDrawContext);
            tagOwner(RenderObject::OwnerType::StaticGLTF, name, opaqueStart, transpStart);
        }
    }

    // dynamic GLTF instances
    for (const auto &kv: dynamicGLTFInstances)
    {
        const GLTFInstance &inst = kv.second;
        if (inst.scene)
        {
            const size_t opaqueStart = mainDrawContext.OpaqueSurfaces.size();
            const size_t transpStart = mainDrawContext.TransparentSurfaces.size();
            inst.scene->Draw(inst.transform, mainDrawContext);
            tagOwner(RenderObject::OwnerType::GLTFInstance, kv.first, opaqueStart, transpStart);
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
            if (inst.boundsTypeOverride.has_value())
            {
                obj.bounds.type = *inst.boundsTypeOverride;
            }
            obj.transform = inst.transform;
            obj.sourceMesh = inst.mesh.get();
            obj.surfaceIndex = surfaceIndex++;
            obj.objectID = mainDrawContext.nextID++;
            obj.ownerType = RenderObject::OwnerType::MeshInstance;
            obj.ownerName = kv.first;
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

    // Keep projection FOV in sync with the camera so that CPU ray picking
    // matches what is rendered on-screen.
    const float fov = glm::radians(mainCamera.fovDegrees);
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

    // Fill punctual lights into GPUSceneData
    const uint32_t lightCount = static_cast<uint32_t>(std::min(pointLights.size(), static_cast<size_t>(kMaxPunctualLights)));
    for (uint32_t i = 0; i < lightCount; ++i)
    {
        const PointLight &pl = pointLights[i];
        sceneData.punctualLights[i].position_radius = glm::vec4(pl.position, pl.radius);
        sceneData.punctualLights[i].color_intensity = glm::vec4(pl.color, pl.intensity);
    }
    for (uint32_t i = lightCount; i < kMaxPunctualLights; ++i)
    {
        sceneData.punctualLights[i].position_radius = glm::vec4(0.0f);
        sceneData.punctualLights[i].color_intensity = glm::vec4(0.0f);
    }
    sceneData.lightCounts = glm::uvec4(lightCount, 0u, 0u, 0u);

    auto end = std::chrono::system_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    stats.scene_update_time = elapsed.count() / 1000.f;
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

    // On engine shutdown we know VulkanEngine::cleanup() has already called
    // vkDeviceWaitIdle(), so it is safe to destroy all remaining GLTF scenes
    // immediately instead of deferring them through pendingGLTFRelease.
    if (!pendingGLTFRelease.empty())
    {
        fmt::println("[SceneManager] cleanup: forcing {} pending GLTF releases before shutdown",
                     pendingGLTFRelease.size());
        pendingGLTFRelease.clear(); // drop strong refs → ~LoadedGLTF::clearAll() runs
    }

    // Drop our references to GLTF scenes. Their destructors call clearAll()
    // exactly once to release GPU resources.
    loadedScenes.clear();
    loadedNodes.clear();
}

void SceneManager::addMeshInstance(const std::string &name, std::shared_ptr<MeshAsset> mesh,
                                   const glm::mat4 &transform, std::optional<BoundsType> boundsType)
{
    if (!mesh) return;
    MeshInstance inst{};
    inst.mesh = std::move(mesh);
    inst.transform = transform;
    inst.boundsTypeOverride = boundsType;
    dynamicMeshInstances[name] = std::move(inst);
}

bool SceneManager::getMeshInstanceTransform(const std::string &name, glm::mat4 &outTransform)
{
    auto it = dynamicMeshInstances.find(name);
    if (it == dynamicMeshInstances.end())
    {
        return false;
    }
    outTransform = it->second.transform;
    return true;
}

bool SceneManager::setMeshInstanceTransform(const std::string &name, const glm::mat4 &transform)
{
    auto it = dynamicMeshInstances.find(name);
    if (it == dynamicMeshInstances.end())
    {
        return false;
    }
    it->second.transform = transform;
    return true;
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
    fmt::println("[SceneManager] addGLTFInstance '{}' (scene='{}')",
                 name,
                 scene->debugName.empty() ? "<unnamed>" : scene->debugName.c_str());
    dynamicGLTFInstances[name] = GLTFInstance{std::move(scene), transform};
}

bool SceneManager::removeGLTFInstance(const std::string &name)
{
    auto it = dynamicGLTFInstances.find(name);
    if (it == dynamicGLTFInstances.end()) return false;

    // Defer destruction until after the next frame fence (update_scene).
    if (it->second.scene)
    {
        if (_context && _context->currentFrame)
        {
            auto keepAlive = it->second.scene;
            fmt::println("[SceneManager] removeGLTFInstance '{}' scheduling deferred destroy (scene='{}')",
                         name,
                         keepAlive && !keepAlive->debugName.empty() ? keepAlive->debugName.c_str() : "<unnamed>");
            _context->currentFrame->_deletionQueue.push_function([keepAlive]() mutable { keepAlive.reset(); });
        }
        else
        {
            // Fallback: stash until we have a frame to attach the destruction to.
            pendingGLTFRelease.push_back(it->second.scene);
        }
    }

    dynamicGLTFInstances.erase(it);
    return true;
}

bool SceneManager::getGLTFInstanceTransform(const std::string &name, glm::mat4 &outTransform)
{
    auto it = dynamicGLTFInstances.find(name);
    if (it == dynamicGLTFInstances.end())
    {
        return false;
    }
    outTransform = it->second.transform;
    return true;
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
    fmt::println("[SceneManager] clearGLTFInstances: dynamicGLTFInstances={} pendingBefore={}",
                 dynamicGLTFInstances.size(),
                 pendingGLTFRelease.size());
    for (auto &kv : dynamicGLTFInstances)
    {
        if (kv.second.scene)
        {
            pendingGLTFRelease.push_back(kv.second.scene);
        }
    }
    dynamicGLTFInstances.clear();
    fmt::println("[SceneManager] clearGLTFInstances: pendingAfter={}",
                 pendingGLTFRelease.size());
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
