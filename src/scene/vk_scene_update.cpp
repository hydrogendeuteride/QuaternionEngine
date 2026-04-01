#include "vk_scene.h"

#include "core/config.h"
#include "core/context.h"
#include "core/frame/resources.h"
#include "core/util/logger.h"
#include "scene/planet/planet_system.h"

#include <glm/gtc/matrix_transform.hpp>
#include "glm/gtx/norm.inl"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>

namespace
{
    glm::mat4 make_reversed_infinite_perspective(float fovy_radians, float aspect, float z_near)
    {
        const float f = 1.0f / std::tan(fovy_radians * 0.5f);
        glm::mat4 m(0.0f);
        m[0][0] = f / aspect;
        m[1][1] = f;
        m[2][2] = 0.0f;
        m[2][3] = -1.0f;
        m[3][2] = z_near;
        return m;
    }

    void tag_owner_surfaces(DrawContext &draw_context,
                            RenderObject::OwnerType type,
                            const std::string &name,
                            size_t opaque_begin,
                            size_t transparent_begin,
                            size_t mesh_vfx_begin)
    {
        for (size_t i = opaque_begin; i < draw_context.OpaqueSurfaces.size(); ++i)
        {
            draw_context.OpaqueSurfaces[i].ownerType = type;
            draw_context.OpaqueSurfaces[i].ownerName = name;
        }
        for (size_t i = transparent_begin; i < draw_context.TransparentSurfaces.size(); ++i)
        {
            draw_context.TransparentSurfaces[i].ownerType = type;
            draw_context.TransparentSurfaces[i].ownerName = name;
        }
        for (size_t i = mesh_vfx_begin; i < draw_context.MeshVfxSurfaces.size(); ++i)
        {
            draw_context.MeshVfxSurfaces[i].ownerType = type;
            draw_context.MeshVfxSurfaces[i].ownerName = name;
        }
    }

    glm::vec3 normalize_or_fallback(const glm::vec3 &v, const glm::vec3 &fallback)
    {
        const float len2 = glm::length2(v);
        if (len2 <= 1.0e-8f)
        {
            return fallback;
        }
        return v * (1.0f / std::sqrt(len2));
    }
}

void SceneManager::update_scene()
{
    const auto start = std::chrono::system_clock::now();

    releasePendingGLTFReleases();
    resetMainDrawContext();

    const float dt = updateFrameTiming();
    cameraRig.update(*this, mainCamera, dt);
    recenterFloatingOriginIfNeeded();

    const WorldVec3 origin_world = get_world_origin();
    _camera_position_local = world_to_local(mainCamera.position_world, origin_world);

    syncDynamicRootColliderBodies();
    emitDynamicGLTFInstances(origin_world, dt);
    syncColliders();
    emitDynamicMeshInstances(origin_world);
    emitDynamicDecals(origin_world);

    updateViewProjectionData();
    updateDirectionalShadowData();
    updateRayTracingSceneData(origin_world);

    sceneData.timeParams = glm::vec4(_elapsedTime, _deltaTime, 0.0f, 0.0f);
    if (_planetSystem)
    {
        _planetSystem->update_and_emit(*this, mainDrawContext);
    }

    const PunctualLightSummary punctual_summary = updatePunctualLightData(origin_world);
    const uint32_t planet_occluder_count = updatePlanetOccluders(origin_world);
    sceneData.lightCounts = glm::uvec4(punctual_summary.point_count, punctual_summary.spot_count, planet_occluder_count, 0u);

    const auto end = std::chrono::system_clock::now();
    const auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    stats.scene_update_time = elapsed.count() / 1000.0f;
}

void SceneManager::releasePendingGLTFReleases()
{
    if (_context && _context->currentFrame)
    {
        if (!pendingGLTFRelease.empty())
        {
            Logger::info("[SceneManager] update_scene: scheduling {} pending GLTF releases (hasContext={}, hasFrame={})",
                         pendingGLTFRelease.size(),
                         true,
                         true);
        }
        for (auto &sp : pendingGLTFRelease)
        {
            auto keep_alive = sp;
            _context->currentFrame->_deletionQueue.push_function([keep_alive]() mutable { keep_alive.reset(); });
        }
    }
    pendingGLTFRelease.clear();
}

void SceneManager::resetMainDrawContext()
{
    mainDrawContext.OpaqueSurfaces.clear();
    mainDrawContext.TransparentSurfaces.clear();
    mainDrawContext.MeshVfxSurfaces.clear();
    mainDrawContext.Decals.clear();
    mainDrawContext.nextID = 1;
    mainDrawContext.gltfNodeLocalOverrides = nullptr;
}

float SceneManager::updateFrameTiming()
{
    auto now = std::chrono::steady_clock::now();
    if (_lastFrameTime.time_since_epoch().count() == 0)
    {
        _lastFrameTime = now;
    }

    float dt = std::chrono::duration<float>(now - _lastFrameTime).count();
    _lastFrameTime = now;
    dt = std::clamp(dt, 0.0f, 0.1f);

    _deltaTime = dt;
    _elapsedTime = std::fmod(_elapsedTime + dt, 10000.0f);
    return dt;
}

void SceneManager::recenterFloatingOriginIfNeeded()
{
    if (_floating_origin_recenter_threshold <= 0.0)
    {
        return;
    }

    const WorldVec3 origin_before = get_world_origin();
    const WorldVec3 d = mainCamera.position_world - origin_before;
    const double threshold2 = _floating_origin_recenter_threshold * _floating_origin_recenter_threshold;
    if (glm::length2(d) <= threshold2)
    {
        return;
    }

    const WorldVec3 new_origin =
        (_floating_origin_snap_size > 0.0)
            ? snap_world(mainCamera.position_world, _floating_origin_snap_size)
            : mainCamera.position_world;

    if (_context)
    {
        (void)_context->set_origin_world(new_origin);
    }
}

void SceneManager::emitDynamicGLTFInstances(const WorldVec3 &origin_world, float dt)
{
    static int debug_gltf_emit_logs = 0;
    for (auto &kv : dynamicGLTFInstances)
    {
        GLTFInstance &inst = kv.second;
        if (!inst.scene)
        {
            continue;
        }

        if (dt > 0.0f)
        {
            inst.scene->updateAnimation(dt, inst.animation);
        }

        const size_t opaque_start = mainDrawContext.OpaqueSurfaces.size();
        const size_t transparent_start = mainDrawContext.TransparentSurfaces.size();
        const size_t mesh_vfx_start = mainDrawContext.MeshVfxSurfaces.size();

        mainDrawContext.gltfNodeLocalOverrides =
            inst.nodeLocalOverrides.empty() ? nullptr : &inst.nodeLocalOverrides;

        const glm::vec3 t_local = world_to_local(inst.translation_world, origin_world);
        const glm::mat4 instance_transform = make_trs_matrix(t_local, inst.rotation, inst.scale);
        inst.scene->Draw(instance_transform, mainDrawContext);
        mainDrawContext.gltfNodeLocalOverrides = nullptr;

        if (debug_gltf_emit_logs < 12)
        {
            const size_t opaque_added = mainDrawContext.OpaqueSurfaces.size() - opaque_start;
            const size_t transparent_added = mainDrawContext.TransparentSurfaces.size() - transparent_start;
            const size_t mesh_vfx_added = mainDrawContext.MeshVfxSurfaces.size() - mesh_vfx_start;
            Logger::info("[SceneManager] emitDynamicGLTFInstances '{}' scene='{}' topNodes={} meshes={} materials={} "
                         "localPos=({}, {}, {}) added opaque={} transparent={} vfx={}",
                         kv.first,
                         inst.scene->debugName.empty() ? "<unnamed>" : inst.scene->debugName.c_str(),
                         inst.scene->topNodes.size(),
                         inst.scene->meshes.size(),
                         inst.scene->materials.size(),
                         t_local.x,
                         t_local.y,
                         t_local.z,
                         opaque_added,
                         transparent_added,
                         mesh_vfx_added);
            ++debug_gltf_emit_logs;
        }

        tag_owner_surfaces(mainDrawContext,
                           RenderObject::OwnerType::GLTFInstance,
                           kv.first,
                           opaque_start,
                           transparent_start,
                           mesh_vfx_start);
    }
}

void SceneManager::emitDynamicMeshInstances(const WorldVec3 &origin_world)
{
    for (const auto &kv : dynamicMeshInstances)
    {
        const MeshInstance &inst = kv.second;
        if (!inst.mesh || inst.mesh->surfaces.empty())
        {
            continue;
        }

        const glm::vec3 t_local = world_to_local(inst.translation_world, origin_world);
        const glm::mat4 instance_transform = make_trs_matrix(t_local, inst.rotation, inst.scale);

        uint32_t surface_index = 0;
        for (const auto &surf : inst.mesh->surfaces)
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
            obj.transform = instance_transform;
            obj.sourceMesh = inst.mesh.get();
            obj.surfaceIndex = surface_index++;
            obj.objectID = mainDrawContext.nextID++;
            obj.ownerType = RenderObject::OwnerType::MeshInstance;
            obj.ownerName = kv.first;

            if (obj.material->passType == MaterialPass::Transparent)
            {
                mainDrawContext.TransparentSurfaces.push_back(obj);
            }
            else if (obj.material->passType == MaterialPass::MeshVFX)
            {
                mainDrawContext.MeshVfxSurfaces.push_back(obj);
            }
            else
            {
                mainDrawContext.OpaqueSurfaces.push_back(obj);
            }
        }
    }
}

void SceneManager::emitDynamicDecals(const WorldVec3 &origin_world)
{
    if (dynamicDecals.empty())
    {
        return;
    }

    std::vector<const std::pair<const std::string, DecalInstance> *> ordered;
    ordered.reserve(dynamicDecals.size());
    for (const auto &kv : dynamicDecals)
    {
        ordered.push_back(&kv);
    }

    std::sort(ordered.begin(), ordered.end(),
              [](const auto *a, const auto *b)
              {
                  if (a->second.sort_order != b->second.sort_order)
                  {
                      return a->second.sort_order < b->second.sort_order;
                  }
                  return a->first < b->first;
              });

    const size_t draw_count = std::min<size_t>(kMaxDecals, ordered.size());
    mainDrawContext.Decals.reserve(draw_count);
    for (size_t i = 0; i < draw_count; ++i)
    {
        const DecalInstance &src = ordered[i]->second;
        DecalDraw d{};
        d.shape = src.shape;
        d.center_local = world_to_local(src.center_world, origin_world);
        d.rotation = src.rotation;
        d.half_extents = src.half_extents;
        d.albedoHandle = src.albedoHandle;
        d.normalHandle = src.normalHandle;
        d.tint = src.tint;
        d.opacity = src.opacity;
        d.normalStrength = src.normalStrength;
        mainDrawContext.Decals.push_back(d);
    }
}

void SceneManager::updateViewProjectionData()
{
    const glm::mat4 view = mainCamera.getViewMatrix(_camera_position_local);
    const float fov = glm::radians(mainCamera.fovDegrees);

    VkExtent2D logical_extent{kRenderWidth, kRenderHeight};
    if (_context)
    {
        const VkExtent2D ctx_extent = _context->getLogicalRenderExtent();
        if (ctx_extent.width > 0 && ctx_extent.height > 0)
        {
            logical_extent = ctx_extent;
        }
    }

    const float aspect = static_cast<float>(logical_extent.width) /
                         static_cast<float>(logical_extent.height);
    glm::mat4 projection = make_reversed_infinite_perspective(fov, aspect, 0.1f);
    projection[1][1] *= -1.0f;

    sceneData.view = view;
    sceneData.proj = projection;
    sceneData.viewproj = projection * view;
}

void SceneManager::updateDirectionalShadowData()
{
    static const float kAheadBlend[kShadowCascadeCount] = {0.2f, 0.5f, 0.75f, 1.0f};

    const glm::vec3 cam_pos = _camera_position_local;
    const glm::mat4 cam_rot = mainCamera.getRotationMatrix();
    const glm::vec3 cam_fwd = -glm::vec3(cam_rot[2]);

    glm::vec3 light_dir = glm::normalize(-glm::vec3(sceneData.sunlightDirection));
    if (glm::length(light_dir) < 1.0e-5f)
    {
        light_dir = glm::vec3(0.0f, -1.0f, 0.0f);
    }

    const glm::vec3 world_up(0.0f, 1.0f, 0.0f);
    const glm::vec3 ref_up =
        (std::abs(glm::dot(light_dir, world_up)) > 0.99f) ? glm::vec3(0.0f, 0.0f, 1.0f) : world_up;
    glm::vec3 right = glm::cross(light_dir, ref_up);
    if (glm::length2(right) < 1.0e-6f)
    {
        right = glm::vec3(1.0f, 0.0f, 0.0f);
    }
    right = glm::normalize(right);
    const glm::vec3 up = glm::normalize(glm::cross(right, light_dir));

    auto level_radius = [](int level)
    {
        return std::ldexp(kShadowClipBaseRadius, level);
    };

    sceneData.cascadeSplitsView = glm::vec4(
        level_radius(0), level_radius(1), level_radius(2), level_radius(3));

    for (int ci = 0; ci < kShadowCascadeCount; ++ci)
    {
        const float radius = level_radius(ci);
        const float cover = radius * kShadowCascadeRadiusScale + kShadowCascadeRadiusMargin;

        const float ahead = radius * 0.5f;
        const float fu = glm::dot(cam_fwd, right);
        const float fv = glm::dot(cam_fwd, up);
        const glm::vec3 ahead_xy = (right * fu + up * fv) * (ahead * kAheadBlend[ci]);

        const glm::vec3 center_target = cam_pos + ahead_xy;
        const float u = glm::dot(center_target, right);
        const float v = glm::dot(center_target, up);

        const uint32_t shadow_res = _context ? _context->getShadowMapResolution() : kShadowMapResolution;
        const float texel = (2.0f * cover) / static_cast<float>(shadow_res);
        const float u_snapped = std::round(u / texel) * texel;
        const float v_snapped = std::round(v / texel) * texel;
        const glm::vec3 center = center_target + right * (u_snapped - u) + up * (v_snapped - v);

        const float pullback = glm::max(kShadowClipPullbackMin, cover * kShadowClipPullbackFactor);
        const glm::vec3 eye = center - light_dir * pullback;
        const glm::mat4 view = glm::lookAtRH(eye, center, up);
        const float z_far = pullback + cover * kShadowClipForwardFactor + kShadowClipZPadding;
        const glm::mat4 proj = glm::orthoRH_ZO(-cover, cover, -cover, cover, 0.2f, z_far);
        const glm::mat4 light_vp = proj * view;

        sceneData.lightViewProjCascades[ci] = light_vp;
        if (ci == 0)
        {
            sceneData.lightViewProj = light_vp;
        }
    }
}

void SceneManager::updateRayTracingSceneData(const WorldVec3 &origin_world)
{
    if (!_context)
    {
        return;
    }

    const auto &ss = _context->shadowSettings;
    const uint32_t rt_enabled = (ss.enabled && ss.mode != 0u) ? 1u : 0u;
    sceneData.rtOptions = glm::uvec4(rt_enabled, ss.hybridRayCascadesMask, ss.mode, _context->reflectionMode);

    float planet_receiver_shadow_maps = 0.0f;
    if (ss.enabled && ss.mode == 2u && _planetSystem && _planetSystem->enabled())
    {
        auto point_in_any_cascade = [&](const glm::vec3 &p_local) -> bool
        {
            for (int ci = 0; ci < kShadowCascadeCount; ++ci)
            {
                const glm::vec4 clip = sceneData.lightViewProjCascades[ci] * glm::vec4(p_local, 1.0f);
                if (std::abs(clip.w) < 1.0e-6f)
                {
                    continue;
                }

                const glm::vec3 ndc = glm::vec3(clip) * (1.0f / clip.w);
                if (std::abs(ndc.x) <= 1.0f &&
                    std::abs(ndc.y) <= 1.0f &&
                    ndc.z >= 0.0f &&
                    ndc.z <= 1.0f)
                {
                    return true;
                }
            }
            return false;
        };

        for (const PlanetSystem::PlanetBody &body : _planetSystem->bodies())
        {
            if (!body.visible || body.radius_m <= 0.0)
            {
                continue;
            }

            const WorldVec3 to_cam = mainCamera.position_world - body.center_world;
            const double dist2 = glm::dot(to_cam, to_cam);
            if (!(dist2 > 1.0e-16))
            {
                continue;
            }

            const double dist = std::sqrt(dist2);
            const WorldVec3 dir = to_cam * (1.0 / dist);
            const WorldVec3 surface_world = body.center_world + dir * body.radius_m;
            const glm::vec3 surface_local = world_to_local(surface_world, origin_world);

            if (point_in_any_cascade(surface_local))
            {
                planet_receiver_shadow_maps = 1.0f;
                break;
            }
        }
    }

    sceneData.rtParams = glm::vec4(ss.hybridRayNoLThreshold,
                                   ss.enabled ? 1.0f : 0.0f,
                                   planet_receiver_shadow_maps,
                                   glm::radians(std::max(0.0f, ss.planetSunAngularRadiusDeg)));

    sceneData.shadowTuning = glm::vec4(std::clamp(ss.shadowMinVisibility, 0.0f, 1.0f),
                                       0.0f, 0.0f, 0.0f);
}

SceneManager::PunctualLightSummary SceneManager::updatePunctualLightData(const WorldVec3 &origin_world)
{
    const ShadowSettings *ss_ptr = _context ? &_context->shadowSettings : nullptr;
    const uint32_t punctual_mode = (ss_ptr && ss_ptr->enabled) ? ss_ptr->punctualMode : 0u;
    const bool punctual_use_maps = (punctual_mode == 1u) || (punctual_mode == 3u);
    const bool punctual_use_rt = (punctual_mode == 2u) || (punctual_mode == 3u);
    const uint32_t max_shadowed_spots = ss_ptr ? std::min(ss_ptr->maxShadowedSpotLights, kMaxShadowedSpotLights) : 0u;
    const uint32_t max_shadowed_points = ss_ptr ? std::min(ss_ptr->maxShadowedPointLights, kMaxShadowedPointLights) : 0u;
    uint32_t spot_shadow_count = 0u;
    uint32_t point_shadow_count = 0u;

    const uint32_t point_count =
        static_cast<uint32_t>(std::min(pointLights.size(), static_cast<size_t>(kMaxPunctualLights)));
    _pointShadowOrder.clear();
    _pointShadowOrder.reserve(point_count);

    if (point_count > 0u)
    {
        auto point_shadow_score = [&](const PointLight &pl) -> double
        {
            const glm::vec3 c = glm::max(pl.color, glm::vec3(0.0f));
            const float luma = glm::dot(c, glm::vec3(0.2126f, 0.7152f, 0.0722f));
            const glm::dvec3 to_cam = mainCamera.position_world - pl.position_world;
            const double dist = std::sqrt(glm::dot(to_cam, to_cam));
            const double proximity = std::clamp(static_cast<double>(pl.radius) / (dist + 1.0), 0.0, 1.0);
            return static_cast<double>(std::max(pl.intensity, 0.0f) * luma) * proximity;
        };

        _shadowCandidates.clear();
        _shadowCandidates.reserve(point_count);
        for (uint32_t i = 0; i < point_count; ++i)
        {
            const PointLight &pl = pointLights[i];
            if (pl.cast_shadows && pl.radius > 0.0f && pl.intensity > 0.0f)
            {
                _shadowCandidates.push_back(i);
            }
        }

        std::sort(_shadowCandidates.begin(), _shadowCandidates.end(),
                  [&](uint32_t a, uint32_t b)
                  {
                      return point_shadow_score(pointLights[a]) > point_shadow_score(pointLights[b]);
                  });

        if (punctual_use_maps)
        {
            point_shadow_count = std::min<uint32_t>(max_shadowed_points, static_cast<uint32_t>(_shadowCandidates.size()));
        }

        _shadowSelected.clear();
        _shadowSelected.resize(point_count, 0u);
        for (uint32_t i = 0; i < point_shadow_count; ++i)
        {
            _pointShadowOrder.push_back(_shadowCandidates[i]);
            _shadowSelected[_shadowCandidates[i]] = 1u;
        }
        for (uint32_t i = 0; i < point_count; ++i)
        {
            if (!_shadowSelected[i])
            {
                _pointShadowOrder.push_back(i);
            }
        }
    }

    const uint32_t spot_count =
        static_cast<uint32_t>(std::min(spotLights.size(), static_cast<size_t>(kMaxSpotLights)));
    _spotShadowOrder.clear();
    _spotShadowOrder.reserve(spot_count);

    if (spot_count > 0u)
    {
        auto spot_shadow_score = [&](const SpotLight &sl) -> double
        {
            const glm::vec3 c = glm::max(sl.color, glm::vec3(0.0f));
            const float luma = glm::dot(c, glm::vec3(0.2126f, 0.7152f, 0.0722f));
            const glm::dvec3 to_cam = mainCamera.position_world - sl.position_world;
            const double dist = std::sqrt(glm::dot(to_cam, to_cam));
            const double proximity = std::clamp(static_cast<double>(sl.radius) / (dist + 1.0), 0.0, 1.0);
            const double cone_weight = std::clamp(static_cast<double>(sl.outer_angle_deg) / 90.0, 0.05, 1.0);
            return static_cast<double>(std::max(sl.intensity, 0.0f) * luma) * proximity * cone_weight;
        };

        _shadowCandidates.clear();
        _shadowCandidates.reserve(spot_count);
        for (uint32_t i = 0; i < spot_count; ++i)
        {
            const SpotLight &sl = spotLights[i];
            if (sl.cast_shadows && sl.radius > 0.0f && sl.intensity > 0.0f)
            {
                _shadowCandidates.push_back(i);
            }
        }

        std::sort(_shadowCandidates.begin(), _shadowCandidates.end(),
                  [&](uint32_t a, uint32_t b)
                  {
                      return spot_shadow_score(spotLights[a]) > spot_shadow_score(spotLights[b]);
                  });

        if (punctual_use_maps)
        {
            spot_shadow_count = std::min<uint32_t>(max_shadowed_spots, static_cast<uint32_t>(_shadowCandidates.size()));
        }

        _shadowSelected.clear();
        _shadowSelected.resize(spot_count, 0u);
        for (uint32_t i = 0; i < spot_shadow_count; ++i)
        {
            _spotShadowOrder.push_back(_shadowCandidates[i]);
            _shadowSelected[_shadowCandidates[i]] = 1u;
        }
        for (uint32_t i = 0; i < spot_count; ++i)
        {
            if (!_shadowSelected[i])
            {
                _spotShadowOrder.push_back(i);
            }
        }
    }

    for (uint32_t i = 0; i < point_count; ++i)
    {
        const PointLight &pl = pointLights[_pointShadowOrder[i]];
        const glm::vec3 pos_local = world_to_local(pl.position_world, origin_world);
        sceneData.punctualLights[i].position_radius = glm::vec4(pos_local, std::max(pl.radius, 0.0001f));
        sceneData.punctualLights[i].color_intensity = glm::vec4(pl.color, pl.intensity);
    }
    for (uint32_t i = point_count; i < kMaxPunctualLights; ++i)
    {
        sceneData.punctualLights[i].position_radius = glm::vec4(0.0f);
        sceneData.punctualLights[i].color_intensity = glm::vec4(0.0f);
    }

    for (uint32_t i = 0; i < spot_count; ++i)
    {
        const SpotLight &sl = spotLights[_spotShadowOrder[i]];
        const glm::vec3 pos_local = world_to_local(sl.position_world, origin_world);
        const glm::vec3 dir = normalize_or_fallback(sl.direction, glm::vec3(0.0f, -1.0f, 0.0f));

        const float radius = std::max(sl.radius, 0.0001f);
        const float inner_deg = std::clamp(sl.inner_angle_deg, 0.0f, 89.0f);
        const float outer_deg = std::clamp(sl.outer_angle_deg, inner_deg, 89.9f);
        const float cos_inner = glm::cos(glm::radians(inner_deg));
        const float cos_outer = glm::cos(glm::radians(outer_deg));

        sceneData.spotLights[i].position_radius = glm::vec4(pos_local, radius);
        sceneData.spotLights[i].direction_cos_outer = glm::vec4(dir, cos_outer);
        sceneData.spotLights[i].color_intensity = glm::vec4(sl.color, sl.intensity);
        sceneData.spotLights[i].cone = glm::vec4(cos_inner, 0.0f, 0.0f, 0.0f);
    }
    for (uint32_t i = spot_count; i < kMaxSpotLights; ++i)
    {
        sceneData.spotLights[i].position_radius = glm::vec4(0.0f);
        sceneData.spotLights[i].direction_cos_outer = glm::vec4(0.0f);
        sceneData.spotLights[i].color_intensity = glm::vec4(0.0f);
        sceneData.spotLights[i].cone = glm::vec4(0.0f);
    }

    for (glm::mat4 &m : sceneData.spotLightShadowViewProj)
    {
        m = glm::mat4(1.0f);
    }
    for (glm::mat4 &m : sceneData.pointLightShadowViewProj)
    {
        m = glm::mat4(1.0f);
    }

    for (uint32_t i = 0; i < spot_shadow_count; ++i)
    {
        const glm::vec3 pos_local = glm::vec3(sceneData.spotLights[i].position_radius);
        const glm::vec3 dir =
            normalize_or_fallback(glm::vec3(sceneData.spotLights[i].direction_cos_outer), glm::vec3(0.0f, -1.0f, 0.0f));

        glm::vec3 up(0.0f, 1.0f, 0.0f);
        if (std::abs(glm::dot(up, dir)) > 0.99f)
        {
            up = glm::vec3(0.0f, 0.0f, 1.0f);
        }

        const float radius = std::max(sceneData.spotLights[i].position_radius.w, 0.05f);
        const float cos_outer = std::clamp(sceneData.spotLights[i].direction_cos_outer.w, -1.0f, 1.0f);
        const float spot_fov = std::clamp(2.0f * std::acos(cos_outer), glm::radians(2.0f), glm::radians(170.0f));
        const glm::mat4 shadow_view = glm::lookAtRH(pos_local, pos_local + dir, up);
        const glm::mat4 shadow_proj = glm::perspectiveRH_ZO(spot_fov, 1.0f, 0.05f, radius);
        sceneData.spotLightShadowViewProj[i] = shadow_proj * shadow_view;
    }

    constexpr std::array<glm::vec3, kPointShadowFaceCount> kPointDirs = {
        glm::vec3(1.0f, 0.0f, 0.0f),
        glm::vec3(-1.0f, 0.0f, 0.0f),
        glm::vec3(0.0f, 1.0f, 0.0f),
        glm::vec3(0.0f, -1.0f, 0.0f),
        glm::vec3(0.0f, 0.0f, 1.0f),
        glm::vec3(0.0f, 0.0f, -1.0f),
    };
    constexpr std::array<glm::vec3, kPointShadowFaceCount> kPointUps = {
        glm::vec3(0.0f, -1.0f, 0.0f),
        glm::vec3(0.0f, -1.0f, 0.0f),
        glm::vec3(0.0f, 0.0f, 1.0f),
        glm::vec3(0.0f, 0.0f, -1.0f),
        glm::vec3(0.0f, -1.0f, 0.0f),
        glm::vec3(0.0f, -1.0f, 0.0f),
    };

    for (uint32_t i = 0; i < point_shadow_count; ++i)
    {
        const glm::vec3 pos_local = glm::vec3(sceneData.punctualLights[i].position_radius);
        const float radius = std::max(sceneData.punctualLights[i].position_radius.w, 0.05f);
        const glm::mat4 shadow_proj = glm::perspectiveRH_ZO(glm::radians(90.0f), 1.0f, 0.05f, radius);
        for (uint32_t face = 0; face < kPointShadowFaceCount; ++face)
        {
            const glm::mat4 shadow_view = glm::lookAtRH(pos_local, pos_local + kPointDirs[face], kPointUps[face]);
            sceneData.pointLightShadowViewProj[i * kPointShadowFaceCount + face] = shadow_proj * shadow_view;
        }
    }

    uint32_t rt_spot_budget = 0u;
    uint32_t rt_point_budget = 0u;
    if (punctual_use_rt)
    {
        if (punctual_mode == 2u)
        {
            rt_spot_budget = spot_count;
            rt_point_budget = point_count;
        }
        else
        {
            rt_spot_budget = ss_ptr ? std::min(ss_ptr->hybridRtMaxSpotLights, spot_count) : 0u;
            rt_point_budget = ss_ptr ? std::min(ss_ptr->hybridRtMaxPointLights, point_count) : 0u;
        }
    }

    const uint32_t mode_encoded = ss_ptr && ss_ptr->enabled ? ss_ptr->punctualMode : 0u;
    const uint32_t mode_flags = (punctual_use_maps ? 1u : 0u) | (punctual_use_rt ? 2u : 0u);
    sceneData.punctualShadowConfig = glm::uvec4(mode_encoded, spot_shadow_count, point_shadow_count, mode_flags);
    sceneData.punctualShadowRtBudget = glm::uvec4(rt_spot_budget, rt_point_budget, 0u, 0u);
    sceneData.punctualShadowParams = glm::vec4(
        ss_ptr ? glm::clamp(ss_ptr->punctualHybridRayNoLThreshold, 0.0f, 1.0f) : 0.25f,
        ss_ptr ? std::max(ss_ptr->spotShadowDepthBias, 0.0f) : 0.0009f,
        ss_ptr ? std::max(ss_ptr->pointShadowDepthBias, 0.0f) : 0.0025f,
        0.0f);

    return PunctualLightSummary{point_count, spot_count};
}

uint32_t SceneManager::updatePlanetOccluders(const WorldVec3 &origin_world)
{
    for (glm::vec4 &occluder : sceneData.planetOccluders)
    {
        occluder = glm::vec4(0.0f);
    }

    if (!_planetSystem || !_planetSystem->enabled())
    {
        return 0u;
    }

    struct PlanetCandidate
    {
        const PlanetSystem::PlanetBody *body = nullptr;
        double score = 0.0;
    };

    std::vector<PlanetCandidate> candidates;
    candidates.reserve(_planetSystem->bodies().size());

    for (const PlanetSystem::PlanetBody &body : _planetSystem->bodies())
    {
        if (!body.visible || !(body.radius_m > 0.0))
        {
            continue;
        }

        const WorldVec3 to_cam = mainCamera.position_world - body.center_world;
        const double dist2 = glm::dot(to_cam, to_cam);
        const double dist = (dist2 > 1.0e-16) ? std::sqrt(dist2) : 0.0;
        const double score = (dist > 0.0) ? (body.radius_m / dist) : 1.0e30;
        candidates.push_back(PlanetCandidate{&body, score});
    }

    std::sort(candidates.begin(), candidates.end(),
              [](const PlanetCandidate &a, const PlanetCandidate &b)
              {
                  return a.score > b.score;
              });

    const uint32_t occluder_count = static_cast<uint32_t>(std::min<size_t>(4, candidates.size()));
    for (uint32_t i = 0; i < occluder_count; ++i)
    {
        const PlanetSystem::PlanetBody &body = *candidates[i].body;
        const glm::vec3 center_local = world_to_local(body.center_world, origin_world);
        const float radius_m = static_cast<float>(std::max(0.0, body.radius_m));
        sceneData.planetOccluders[i] = glm::vec4(center_local, radius_m);
    }

    return occluder_count;
}
