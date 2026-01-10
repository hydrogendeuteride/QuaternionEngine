#include "vk_scene.h"

#include <utility>
#include <unordered_set>
#include <chrono>

#include "scene/planet/planet_system.h"
#include "core/device/swapchain.h"
#include "core/context.h"
#include "core/config.h"
#include "glm/gtx/transform.hpp"
#include <glm/gtc/matrix_transform.hpp>
#include "glm/gtx/norm.inl"
#include "glm/gtx/compatibility.hpp"
#include <algorithm>
#include <cmath>

#include "core/frame/resources.h"
#include <fmt/core.h>

SceneManager::SceneManager() = default;

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

bool SceneManager::getPointLight(size_t index, PointLight &outLight) const
{
    if (index >= pointLights.size())
    {
        return false;
    }
    outLight = pointLights[index];
    return true;
}

bool SceneManager::setPointLight(size_t index, const PointLight &light)
{
    if (index >= pointLights.size())
    {
        return false;
    }
    pointLights[index] = light;
    return true;
}

bool SceneManager::removePointLight(size_t index)
{
    if (index >= pointLights.size())
    {
        return false;
    }
    pointLights.erase(pointLights.begin() + index);
    return true;
}

void SceneManager::addSpotLight(const SpotLight &light)
{
    spotLights.push_back(light);
}

void SceneManager::clearSpotLights()
{
    spotLights.clear();
}

bool SceneManager::getSpotLight(size_t index, SpotLight &outLight) const
{
    if (index >= spotLights.size())
    {
        return false;
    }
    outLight = spotLights[index];
    return true;
}

bool SceneManager::setSpotLight(size_t index, const SpotLight &light)
{
    if (index >= spotLights.size())
    {
        return false;
    }
    spotLights[index] = light;
    return true;
}

bool SceneManager::removeSpotLight(size_t index)
{
    if (index >= spotLights.size())
    {
        return false;
    }
    spotLights.erase(spotLights.begin() + index);
    return true;
}

void SceneManager::init(EngineContext *context)
{
    _context = context;

    mainCamera.position_world = WorldVec3(30.0, 0.0, 85.0);
    mainCamera.orientation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);

    sceneData.ambientColor = glm::vec4(0.1f, 0.1f, 0.1f, 1.0f);
    sceneData.sunlightDirection = glm::vec4(-0.2f, -1.0f, -0.3f, 1.0f);
    sceneData.sunlightColor = glm::vec4(1.0f, 1.0f, 1.0f, 3.0f);

    cameraRig.init(*this, mainCamera);

    _camera_position_local = world_to_local(mainCamera.position_world, _origin_world);

    _planetSystem = std::make_unique<PlanetSystem>();
    _planetSystem->init(_context);
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
    mainDrawContext.gltfNodeLocalOverrides = nullptr;

    // Simple per-frame dt (seconds) for animations and camera behavior.
    auto now = std::chrono::steady_clock::now();
    if (_lastFrameTime.time_since_epoch().count() == 0)
    {
        _lastFrameTime = now;
    }
    float dt = std::chrono::duration<float>(now - _lastFrameTime).count();
    _lastFrameTime = now;
    if (dt < 0.f)
    {
        dt = 0.f;
    }
    if (dt > 0.1f)
    {
        dt = 0.1f;
    }
    _deltaTime = dt;

    cameraRig.update(*this, mainCamera, dt);

    // Floating origin: keep render-local coordinates near (0,0,0) by shifting the origin
    // when the camera drifts too far in world space.
    if (_floating_origin_recenter_threshold > 0.0)
    {
        const WorldVec3 d = mainCamera.position_world - _origin_world;
        const double threshold2 = _floating_origin_recenter_threshold * _floating_origin_recenter_threshold;
        if (glm::length2(d) > threshold2)
        {
            const WorldVec3 newOrigin =
                (_floating_origin_snap_size > 0.0)
                    ? snap_world(mainCamera.position_world, _floating_origin_snap_size)
                    : mainCamera.position_world;
            _origin_world = newOrigin;
        }
    }

    _camera_position_local = world_to_local(mainCamera.position_world, _origin_world);

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

    // Root transform to shift "world space" float scenes into render-local space.
    // Any object that is authored in world coordinates (float) should be offset by -origin.
    const glm::mat4 world_to_local_root =
        glm::translate(glm::mat4{1.f}, world_to_local(WorldVec3(0.0, 0.0, 0.0), _origin_world));

    // Draw all loaded GLTF scenes (static world), advancing their independent animation states.
    for (auto &[name, scene] : loadedScenes)
    {
        if (!scene)
        {
            continue;
        }

        // Advance this scene's animation state (independent of instances).
        if (dt > 0.f)
        {
            auto &animState = sceneAnimations[name];
            scene->updateAnimation(dt, animState);
        }

        const size_t opaqueStart = mainDrawContext.OpaqueSurfaces.size();
        const size_t transpStart = mainDrawContext.TransparentSurfaces.size();
        mainDrawContext.gltfNodeLocalOverrides = nullptr;
        scene->Draw(world_to_local_root, mainDrawContext);
        mainDrawContext.gltfNodeLocalOverrides = nullptr;
        tagOwner(RenderObject::OwnerType::StaticGLTF, name, opaqueStart, transpStart);
    }

    // dynamic GLTF instances (each with its own animation state)
    for (auto &kv : dynamicGLTFInstances)
    {
        GLTFInstance &inst = kv.second;
        if (!inst.scene)
        {
            continue;
        }

        if (dt > 0.f)
        {
            inst.scene->updateAnimation(dt, inst.animation);
        }

        const size_t opaqueStart = mainDrawContext.OpaqueSurfaces.size();
        const size_t transpStart = mainDrawContext.TransparentSurfaces.size();
        // Enable per-instance node pose overrides while drawing this instance.
        if (!inst.nodeLocalOverrides.empty())
        {
            mainDrawContext.gltfNodeLocalOverrides = &inst.nodeLocalOverrides;
        }
        else
        {
            mainDrawContext.gltfNodeLocalOverrides = nullptr;
        }
        glm::vec3 tLocal = world_to_local(inst.translation_world, _origin_world);
        glm::mat4 instanceTransform = make_trs_matrix(tLocal, inst.rotation, inst.scale);
        inst.scene->Draw(instanceTransform, mainDrawContext);
        mainDrawContext.gltfNodeLocalOverrides = nullptr;
        tagOwner(RenderObject::OwnerType::GLTFInstance, kv.first, opaqueStart, transpStart);
    }

    // Default primitives are added as dynamic instances by the engine.

    // dynamic mesh instances
    for (const auto &kv: dynamicMeshInstances)
    {
        const MeshInstance &inst = kv.second;
        if (!inst.mesh || inst.mesh->surfaces.empty()) continue;
        glm::vec3 tLocal = world_to_local(inst.translation_world, _origin_world);
        glm::mat4 instanceTransform = make_trs_matrix(tLocal, inst.rotation, inst.scale);
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
            obj.transform = instanceTransform;
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

    glm::mat4 view = mainCamera.getViewMatrix(_camera_position_local);
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
    // matches what is rendered inside the fixed logical render area (letterboxed).
    const float fov = glm::radians(mainCamera.fovDegrees);

    // Derive aspect ratio from the fixed logical render size instead of the window/swapchain.
    VkExtent2D logicalExtent{ kRenderWidth, kRenderHeight };
    if (_context)
    {
        VkExtent2D ctxExtent = _context->getLogicalRenderExtent();
        if (ctxExtent.width > 0 && ctxExtent.height > 0)
        {
            logicalExtent = ctxExtent;
        }
    }

    const float aspect = static_cast<float>(logicalExtent.width) /
                         static_cast<float>(logicalExtent.height);
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
        const glm::vec3 camPos = _camera_position_local;
        const glm::mat4 camRot = mainCamera.getRotationMatrix();
        const glm::vec3 camFwd = -glm::vec3(camRot[2]); // -Z axis in world/local space

        glm::vec3 L = glm::normalize(-glm::vec3(sceneData.sunlightDirection));
        if (glm::length(L) < 1e-5f) L = glm::vec3(0.0f, -1.0f, 0.0f);
        const glm::vec3 worldUp(0.0f, 1.0f, 0.0f);
        const glm::vec3 refUp = (std::abs(glm::dot(L, worldUp)) > 0.99f)
                                    ? glm::vec3(0.0f, 0.0f, 1.0f)
                                    : worldUp;
        glm::vec3 right = glm::cross(L, refUp);
        if (glm::length2(right) < 1e-6f) right = glm::vec3(1, 0, 0);
        right = glm::normalize(right);
        glm::vec3 up = glm::normalize(glm::cross(right, L));

        auto level_radius = [](int level) {
            // exact power-of-two scaling (reduces drift vs powf)
            return std::ldexp(kShadowClipBaseRadius, level);
        };

        sceneData.cascadeSplitsView = glm::vec4(
            level_radius(0), level_radius(1), level_radius(2), level_radius(3));

        for (int ci = 0; ci < kShadowCascadeCount; ++ci)
        {
            const float radius = level_radius(ci);
            const float cover = radius * kShadowCascadeRadiusScale + kShadowCascadeRadiusMargin;

            const float ahead = radius * 0.5f;
            const float fu = glm::dot(camFwd, right);
            const float fv = glm::dot(camFwd, up);

            // Bias the clipmap slightly forward in the camera's view direction to spend more
            // resolution where we can actually see. The bias is applied in the light XY plane.
            const glm::vec3 aheadXY = (right * fu + up * fv) * (ahead * kAheadBlend[ci]);

            // Target center before stabilization
            const glm::vec3 centerTarget = camPos + aheadXY;
            const float u = glm::dot(centerTarget, right);
            const float v = glm::dot(centerTarget, up);

            const float texel = (2.0f * cover) / float(kShadowMapResolution);
            // Snap the center in light-space to texel increments for stable shadows (reduced shimmering).
            const float uSnapped = std::round(u / texel) * texel;
            const float vSnapped = std::round(v / texel) * texel;
            const float du = uSnapped - u;
            const float dv = vSnapped - v;

            const glm::vec3 center = centerTarget + right * du + up * dv;

            const float pullback = glm::max(kShadowClipPullbackMin, cover * kShadowClipPullbackFactor);
            const glm::vec3 eye = center - L * pullback;
            const glm::mat4 V = glm::lookAtRH(eye, center, up);

            const float zNear = 0.2f;
            const float zFar = pullback + cover * kShadowClipForwardFactor + kShadowClipZPadding;

            // Note: shadow maps intentionally use the same "forward" 0..1 depth mapping
            // as glm::orthoRH_ZO, paired with a GREATER depth test to keep the farthest
            // depth along the light ray (which corresponds to the closest occluder to the sun
            // for our light view setup).
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
        // RT shadows are considered active only when shadows are enabled and
        // a hybrid/RT shadow mode is selected (mode != 0).
        const uint32_t rtEnabled = (ss.enabled && ss.mode != 0u) ? 1u : 0u;
        const uint32_t reflMode = _context->reflectionMode;
        // rtOptions.x = RT shadows enabled, y = cascade mask, z = shadow mode, w = reflection mode (SSR/RT)
        sceneData.rtOptions = glm::uvec4(rtEnabled, ss.hybridRayCascadesMask, ss.mode, reflMode);

        // Planet receiver shadow-maps (RT-only mode): enable shadow map rendering/sampling only when a
        // visible planet surface point near the camera falls inside at least one shadow cascade.
        float planetReceiverShadowMaps = 0.0f;
        if (ss.enabled && ss.mode == 2u && _planetSystem && _planetSystem->enabled())
        {
            const WorldVec3 camW = mainCamera.position_world;
            const WorldVec3 originW = _origin_world;

            auto point_in_any_cascade = [&](const glm::vec3 &pLocal) -> bool
            {
                for (int ci = 0; ci < kShadowCascadeCount; ++ci)
                {
                    const glm::vec4 clip = sceneData.lightViewProjCascades[ci] * glm::vec4(pLocal, 1.0f);
                    const float w = clip.w;
                    if (std::abs(w) < 1.0e-6f)
                    {
                        continue;
                    }
                    const glm::vec3 ndc = glm::vec3(clip) * (1.0f / w);
                    if (std::abs(ndc.x) <= 1.0f && std::abs(ndc.y) <= 1.0f && ndc.z >= 0.0f && ndc.z <= 1.0f)
                    {
                        return true;
                    }
                }
                return false;
            };

            for (const PlanetSystem::PlanetBody &b : _planetSystem->bodies())
            {
                if (!b.visible || b.radius_m <= 0.0)
                {
                    continue;
                }

                const WorldVec3 toCam = camW - b.center_world;
                const double dist2 = glm::dot(toCam, toCam);
                if (!(dist2 > 1.0e-16))
                {
                    continue;
                }

                const double dist = std::sqrt(dist2);
                const WorldVec3 dir = toCam * (1.0 / dist);
                const WorldVec3 surfaceW = b.center_world + dir * b.radius_m;
                const glm::vec3 surfaceLocal = world_to_local(surfaceW, originW);

                if (point_in_any_cascade(surfaceLocal))
                {
                    planetReceiverShadowMaps = 1.0f;
                    break;
                }
            }
        }

        // rtParams.x = N·L threshold for hybrid shadows
        // rtParams.y = shadows enabled flag (1.0 = on, 0.0 = off)
        // rtParams.z = planet receiver clipmap shadow maps enabled flag (RT-only mode)
        // rtParams.w = sun angular radius (radians) for analytic planet shadow penumbra
        sceneData.rtParams  = glm::vec4(ss.hybridRayNoLThreshold,
                                        ss.enabled ? 1.0f : 0.0f,
                                        planetReceiverShadowMaps,
                                        glm::radians(std::max(0.0f, ss.planetSunAngularRadiusDeg)));
    }

    if (_planetSystem)
    {
        _planetSystem->update_and_emit(*this, mainDrawContext);
    }

    const uint32_t lightCount = static_cast<uint32_t>(std::min(pointLights.size(), static_cast<size_t>(kMaxPunctualLights)));
    for (uint32_t i = 0; i < lightCount; ++i)
    {
        const PointLight &pl = pointLights[i];
        glm::vec3 posLocal = world_to_local(pl.position_world, _origin_world);
        sceneData.punctualLights[i].position_radius = glm::vec4(posLocal, pl.radius);
        sceneData.punctualLights[i].color_intensity = glm::vec4(pl.color, pl.intensity);
    }
    for (uint32_t i = lightCount; i < kMaxPunctualLights; ++i)
    {
        sceneData.punctualLights[i].position_radius = glm::vec4(0.0f);
        sceneData.punctualLights[i].color_intensity = glm::vec4(0.0f);
    }

    const uint32_t spotCount = static_cast<uint32_t>(std::min(spotLights.size(), static_cast<size_t>(kMaxSpotLights)));
    for (uint32_t i = 0; i < spotCount; ++i)
    {
        const SpotLight &sl = spotLights[i];
        glm::vec3 posLocal = world_to_local(sl.position_world, _origin_world);

        glm::vec3 dir = sl.direction;
        const float dirLen2 = glm::length2(dir);
        if (dirLen2 > 1.0e-8f)
        {
            dir *= 1.0f / std::sqrt(dirLen2);
        }
        else
        {
            dir = glm::vec3(0.0f, -1.0f, 0.0f);
        }

        const float radius = std::max(sl.radius, 0.0001f);
        const float innerDeg = std::clamp(sl.inner_angle_deg, 0.0f, 89.0f);
        const float outerDeg = std::clamp(sl.outer_angle_deg, innerDeg, 89.9f);
        const float cosInner = glm::cos(glm::radians(innerDeg));
        const float cosOuter = glm::cos(glm::radians(outerDeg));

        sceneData.spotLights[i].position_radius = glm::vec4(posLocal, radius);
        sceneData.spotLights[i].direction_cos_outer = glm::vec4(dir, cosOuter);
        sceneData.spotLights[i].color_intensity = glm::vec4(sl.color, sl.intensity);
        sceneData.spotLights[i].cone = glm::vec4(cosInner, 0.0f, 0.0f, 0.0f);
    }
    for (uint32_t i = spotCount; i < kMaxSpotLights; ++i)
    {
        sceneData.spotLights[i].position_radius = glm::vec4(0.0f);
        sceneData.spotLights[i].direction_cos_outer = glm::vec4(0.0f);
        sceneData.spotLights[i].color_intensity = glm::vec4(0.0f);
        sceneData.spotLights[i].cone = glm::vec4(0.0f);
    }

    // Populate analytic planet shadow occluders (up to 4 planets with largest angular size from the camera).
    uint32_t planetOccluderCount = 0;
    for (int i = 0; i < 4; ++i)
    {
        sceneData.planetOccluders[i] = glm::vec4(0.0f);
    }

    if (_planetSystem && _planetSystem->enabled())
    {
        struct PlanetCandidate
        {
            const PlanetSystem::PlanetBody *body = nullptr;
            double score = 0.0; // radius / distance
        };

        std::vector<PlanetCandidate> candidates;
        candidates.reserve(_planetSystem->bodies().size());

        for (const PlanetSystem::PlanetBody &b : _planetSystem->bodies())
        {
            if (!b.visible || !(b.radius_m > 0.0))
            {
                continue;
            }

            const WorldVec3 toCam = mainCamera.position_world - b.center_world;
            const double dist2 = glm::dot(toCam, toCam);
            const double dist = (dist2 > 1.0e-16) ? std::sqrt(dist2) : 0.0;
            const double score = (dist > 0.0) ? (b.radius_m / dist) : 1.0e30;

            candidates.push_back(PlanetCandidate{&b, score});
        }

        std::sort(candidates.begin(), candidates.end(),
                  [](const PlanetCandidate &a, const PlanetCandidate &b)
                  {
                      return a.score > b.score;
                  });

        planetOccluderCount = static_cast<uint32_t>(std::min<size_t>(4, candidates.size()));
        for (uint32_t i = 0; i < planetOccluderCount; ++i)
        {
            const PlanetSystem::PlanetBody &b = *candidates[i].body;
            const glm::vec3 centerLocal = world_to_local(b.center_world, _origin_world);
            const float radiusM = static_cast<float>(std::max(0.0, b.radius_m));
            sceneData.planetOccluders[i] = glm::vec4(centerLocal, radiusM);
        }
    }

    sceneData.lightCounts = glm::uvec4(lightCount, spotCount, planetOccluderCount, 0u);

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
    loadedScenes[name] = scene;

    if (scene && !scene->animations.empty())
    {
        LoadedGLTF::AnimationState st{};
        st.activeAnimation = 0;
        st.animationTime = 0.0f;
        st.animationLoop = true;
        sceneAnimations[name] = st;
    }
    else
    {
        sceneAnimations.erase(name);
    }
}

std::shared_ptr<LoadedGLTF> SceneManager::getScene(const std::string &name)
{
    auto it = loadedScenes.find(name);
    return (it != loadedScenes.end()) ? it->second : nullptr;
}

void SceneManager::cleanup()
{
    if (_planetSystem)
    {
        _planetSystem->cleanup();
        _planetSystem.reset();
    }

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
    sceneAnimations.clear();
    loadedNodes.clear();
}

void SceneManager::addMeshInstance(const std::string &name, std::shared_ptr<MeshAsset> mesh,
                                   const glm::mat4 &transform, std::optional<BoundsType> boundsType)
{
    if (!mesh) return;
    MeshInstance inst{};
    inst.mesh = std::move(mesh);
    glm::vec3 t{};
    decompose_trs_matrix(transform, t, inst.rotation, inst.scale);
    inst.translation_world = WorldVec3(t);
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
    const MeshInstance &inst = it->second;
    outTransform = make_trs_matrix(glm::vec3(inst.translation_world), inst.rotation, inst.scale);
    return true;
}

bool SceneManager::setMeshInstanceTransform(const std::string &name, const glm::mat4 &transform)
{
    auto it = dynamicMeshInstances.find(name);
    if (it == dynamicMeshInstances.end())
    {
        return false;
    }
    MeshInstance &inst = it->second;
    glm::vec3 t{};
    decompose_trs_matrix(transform, t, inst.rotation, inst.scale);
    inst.translation_world = WorldVec3(t);
    return true;
}

bool SceneManager::getMeshInstanceTransformLocal(const std::string &name, glm::mat4 &outTransformLocal) const
{
    auto it = dynamicMeshInstances.find(name);
    if (it == dynamicMeshInstances.end())
    {
        return false;
    }

    const MeshInstance &inst = it->second;
    glm::vec3 tLocal = world_to_local(inst.translation_world, _origin_world);
    outTransformLocal = make_trs_matrix(tLocal, inst.rotation, inst.scale);
    return true;
}

bool SceneManager::setMeshInstanceTransformLocal(const std::string &name, const glm::mat4 &transformLocal)
{
    auto it = dynamicMeshInstances.find(name);
    if (it == dynamicMeshInstances.end())
    {
        return false;
    }

    MeshInstance &inst = it->second;
    glm::vec3 tLocal{};
    decompose_trs_matrix(transformLocal, tLocal, inst.rotation, inst.scale);
    inst.translation_world = local_to_world(tLocal, _origin_world);
    return true;
}

bool SceneManager::getMeshInstanceTRSWorld(const std::string &name,
                                          WorldVec3 &outTranslationWorld,
                                          glm::quat &outRotation,
                                          glm::vec3 &outScale) const
{
    auto it = dynamicMeshInstances.find(name);
    if (it == dynamicMeshInstances.end())
    {
        return false;
    }

    const MeshInstance &inst = it->second;
    outTranslationWorld = inst.translation_world;
    outRotation = inst.rotation;
    outScale = inst.scale;
    return true;
}

bool SceneManager::setMeshInstanceTRSWorld(const std::string &name,
                                          const WorldVec3 &translationWorld,
                                          const glm::quat &rotation,
                                          const glm::vec3 &scale)
{
    auto it = dynamicMeshInstances.find(name);
    if (it == dynamicMeshInstances.end())
    {
        return false;
    }

    MeshInstance &inst = it->second;
    inst.translation_world = translationWorld;
    inst.rotation = rotation;
    inst.scale = scale;
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
    GLTFInstance inst{};
    inst.scene = std::move(scene);
    glm::vec3 t{};
    decompose_trs_matrix(transform, t, inst.rotation, inst.scale);
    inst.translation_world = WorldVec3(t);
    if (inst.scene && !inst.scene->animations.empty())
    {
        inst.animation.activeAnimation = 0;
        inst.animation.animationTime = 0.0f;
        inst.animation.animationLoop = true;
    }
    dynamicGLTFInstances[name] = std::move(inst);
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

std::shared_ptr<LoadedGLTF> SceneManager::getGLTFInstanceScene(const std::string &instanceName) const
{
    auto it = dynamicGLTFInstances.find(instanceName);
    if (it != dynamicGLTFInstances.end())
    {
        return it->second.scene;
    }
    return nullptr;
}

bool SceneManager::getGLTFInstanceTransform(const std::string &name, glm::mat4 &outTransform)
{
    auto it = dynamicGLTFInstances.find(name);
    if (it == dynamicGLTFInstances.end())
    {
        return false;
    }
    const GLTFInstance &inst = it->second;
    outTransform = make_trs_matrix(glm::vec3(inst.translation_world), inst.rotation, inst.scale);
    return true;
}

bool SceneManager::setGLTFInstanceTransform(const std::string &name, const glm::mat4 &transform)
{
    auto it = dynamicGLTFInstances.find(name);
    if (it == dynamicGLTFInstances.end()) return false;
    GLTFInstance &inst = it->second;
    glm::vec3 t{};
    decompose_trs_matrix(transform, t, inst.rotation, inst.scale);
    inst.translation_world = WorldVec3(t);
    return true;
}

bool SceneManager::getGLTFInstanceTransformLocal(const std::string &name, glm::mat4 &outTransformLocal) const
{
    auto it = dynamicGLTFInstances.find(name);
    if (it == dynamicGLTFInstances.end())
    {
        return false;
    }

    const GLTFInstance &inst = it->second;
    glm::vec3 tLocal = world_to_local(inst.translation_world, _origin_world);
    outTransformLocal = make_trs_matrix(tLocal, inst.rotation, inst.scale);
    return true;
}

bool SceneManager::setGLTFInstanceTransformLocal(const std::string &name, const glm::mat4 &transformLocal)
{
    auto it = dynamicGLTFInstances.find(name);
    if (it == dynamicGLTFInstances.end())
    {
        return false;
    }

    GLTFInstance &inst = it->second;
    glm::vec3 tLocal{};
    decompose_trs_matrix(transformLocal, tLocal, inst.rotation, inst.scale);
    inst.translation_world = local_to_world(tLocal, _origin_world);
    return true;
}

bool SceneManager::getGLTFInstanceTRSWorld(const std::string &name,
                                          WorldVec3 &outTranslationWorld,
                                          glm::quat &outRotation,
                                          glm::vec3 &outScale) const
{
    auto it = dynamicGLTFInstances.find(name);
    if (it == dynamicGLTFInstances.end())
    {
        return false;
    }

    const GLTFInstance &inst = it->second;
    outTranslationWorld = inst.translation_world;
    outRotation = inst.rotation;
    outScale = inst.scale;
    return true;
}

bool SceneManager::setGLTFInstanceTRSWorld(const std::string &name,
                                          const WorldVec3 &translationWorld,
                                          const glm::quat &rotation,
                                          const glm::vec3 &scale)
{
    auto it = dynamicGLTFInstances.find(name);
    if (it == dynamicGLTFInstances.end())
    {
        return false;
    }

    GLTFInstance &inst = it->second;
    inst.translation_world = translationWorld;
    inst.rotation = rotation;
    inst.scale = scale;
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

bool SceneManager::setGLTFInstanceNodeOffset(const std::string &instanceName,
                                             const std::string &nodeName,
                                             const glm::mat4 &offset)
{
    auto it = dynamicGLTFInstances.find(instanceName);
    if (it == dynamicGLTFInstances.end())
    {
        return false;
    }
    GLTFInstance &inst = it->second;
    if (!inst.scene)
    {
        return false;
    }

    auto nodePtr = inst.scene->getNode(nodeName);
    if (!nodePtr)
    {
        return false;
    }

    inst.nodeLocalOverrides[nodePtr.get()] = offset;
    return true;
}

bool SceneManager::clearGLTFInstanceNodeOffset(const std::string &instanceName,
                                               const std::string &nodeName)
{
    auto it = dynamicGLTFInstances.find(instanceName);
    if (it == dynamicGLTFInstances.end())
    {
        return false;
    }
    GLTFInstance &inst = it->second;
    if (!inst.scene)
    {
        return false;
    }

    auto nodePtr = inst.scene->getNode(nodeName);
    if (!nodePtr)
    {
        return false;
    }

    auto ovIt = inst.nodeLocalOverrides.find(nodePtr.get());
    if (ovIt == inst.nodeLocalOverrides.end())
    {
        return false;
    }

    inst.nodeLocalOverrides.erase(ovIt);
    return true;
}

void SceneManager::clearGLTFInstanceNodeOffsets(const std::string &instanceName)
{
    auto it = dynamicGLTFInstances.find(instanceName);
    if (it == dynamicGLTFInstances.end())
    {
        return;
    }
    it->second.nodeLocalOverrides.clear();
}

bool SceneManager::setSceneAnimation(const std::string &sceneName, int animationIndex, bool resetTime)
{
    auto it = loadedScenes.find(sceneName);
    if (it == loadedScenes.end() || !it->second)
    {
        return false;
    }

    auto &animState = sceneAnimations[sceneName];
    it->second->setActiveAnimation(animState, animationIndex, resetTime);
    return true;
}

bool SceneManager::setSceneAnimation(const std::string &sceneName, const std::string &animationName, bool resetTime)
{
    auto it = loadedScenes.find(sceneName);
    if (it == loadedScenes.end() || !it->second)
    {
        return false;
    }

    auto &animState = sceneAnimations[sceneName];
    it->second->setActiveAnimation(animState, animationName, resetTime);
    return true;
}

bool SceneManager::setSceneAnimationLoop(const std::string &sceneName, bool loop)
{
    auto it = loadedScenes.find(sceneName);
    if (it == loadedScenes.end() || !it->second)
    {
        return false;
    }

    auto &animState = sceneAnimations[sceneName];
    animState.animationLoop = loop;
    return true;
}

bool SceneManager::setGLTFInstanceAnimation(const std::string &instanceName, int animationIndex, bool resetTime)
{
    auto it = dynamicGLTFInstances.find(instanceName);
    if (it == dynamicGLTFInstances.end() || !it->second.scene)
    {
        return false;
    }

    LoadedGLTF::AnimationState &animState = it->second.animation;
    it->second.scene->setActiveAnimation(animState, animationIndex, resetTime);
    return true;
}

bool SceneManager::setGLTFInstanceAnimation(const std::string &instanceName, const std::string &animationName, bool resetTime)
{
    auto it = dynamicGLTFInstances.find(instanceName);
    if (it == dynamicGLTFInstances.end() || !it->second.scene)
    {
        return false;
    }

    LoadedGLTF::AnimationState &animState = it->second.animation;
    it->second.scene->setActiveAnimation(animState, animationName, resetTime);
    return true;
}

bool SceneManager::setGLTFInstanceAnimationLoop(const std::string &instanceName, bool loop)
{
    auto it = dynamicGLTFInstances.find(instanceName);
    if (it == dynamicGLTFInstances.end() || !it->second.scene)
    {
        return false;
    }

    it->second.animation.animationLoop = loop;
    return true;
}

void SceneManager::setSunlightDirection(const glm::vec3& dir)
{
    const float len2 = glm::length2(dir);
    const glm::vec3 normalized = (len2 > 1.0e-8f)
                                     ? (dir * (1.0f / std::sqrt(len2)))
                                     : glm::vec3(0.0f, -1.0f, 0.0f);
    sceneData.sunlightDirection = glm::vec4(normalized, sceneData.sunlightDirection.w);
}

glm::vec3 SceneManager::getSunlightDirection() const
{
    return glm::vec3(sceneData.sunlightDirection);
}

void SceneManager::setSunlightColor(const glm::vec3& color, float intensity)
{
    sceneData.sunlightColor = glm::vec4(color, intensity);
}

glm::vec3 SceneManager::getSunlightColor() const
{
    return glm::vec3(sceneData.sunlightColor);
}

float SceneManager::getSunlightIntensity() const
{
    return sceneData.sunlightColor.w;
}
