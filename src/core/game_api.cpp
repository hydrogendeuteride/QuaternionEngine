#include "game_api.h"
#include "engine.h"
#include "context.h"
#include "core/assets/texture_cache.h"
#include "core/assets/ibl_manager.h"
#include "core/pipeline/manager.h"
#include "render/passes/tonemap.h"
#include "render/passes/fxaa.h"
#include "render/renderpass.h"
#include "scene/vk_scene.h"
#include "scene/camera.h"

#include <glm/gtx/matrix_decompose.hpp>
#include <glm/gtx/quaternion.hpp>

namespace GameAPI
{

// ============================================================================
// Transform helpers
// ============================================================================

glm::mat4 Transform::to_matrix() const
{
    glm::mat4 T = glm::translate(glm::mat4(1.0f), position);
    glm::mat4 R = glm::mat4_cast(rotation);
    glm::mat4 S = glm::scale(glm::mat4(1.0f), scale);
    return T * R * S;
}

Transform Transform::from_matrix(const glm::mat4& m)
{
    Transform t;
    glm::vec3 skew;
    glm::vec4 perspective;
    glm::decompose(m, t.scale, t.rotation, t.position, skew, perspective);
    return t;
}

glm::mat4 TransformD::to_matrix() const
{
    glm::mat4 T = glm::translate(glm::mat4(1.0f), glm::vec3(position));
    glm::mat4 R = glm::mat4_cast(rotation);
    glm::mat4 S = glm::scale(glm::mat4(1.0f), scale);
    return T * R * S;
}

TransformD TransformD::from_matrix(const glm::mat4& m)
{
    TransformD t;
    glm::vec3 skew;
    glm::vec4 perspective;
    glm::vec3 pos{};
    glm::decompose(m, t.scale, t.rotation, pos, skew, perspective);
    t.position = glm::dvec3(pos);
    return t;
}

// ============================================================================
// Engine Implementation
// ============================================================================

Engine::Engine(VulkanEngine* engine)
    : _engine(engine)
{
}

// ----------------------------------------------------------------------------
// Memory / Texture Streaming
// ----------------------------------------------------------------------------

size_t Engine::get_texture_budget() const
{
    return _engine->query_texture_budget_bytes();
}

void Engine::set_texture_loads_per_frame(int count)
{
    if (_engine->_textureCache)
    {
        _engine->_textureCache->set_max_loads_per_pump(count);
    }
}

int Engine::get_texture_loads_per_frame() const
{
    return _engine->_textureCache ? _engine->_textureCache->max_loads_per_pump() : 0;
}

void Engine::set_texture_upload_budget(size_t bytes)
{
    if (_engine->_textureCache)
    {
        _engine->_textureCache->set_max_bytes_per_pump(bytes);
    }
}

size_t Engine::get_texture_upload_budget() const
{
    return _engine->_textureCache ? _engine->_textureCache->max_bytes_per_pump() : 0;
}

void Engine::set_cpu_source_budget(size_t bytes)
{
    if (_engine->_textureCache)
    {
        _engine->_textureCache->set_cpu_source_budget(bytes);
    }
}

size_t Engine::get_cpu_source_budget() const
{
    return _engine->_textureCache ? _engine->_textureCache->cpu_source_budget() : 0;
}

void Engine::set_max_upload_dimension(uint32_t dim)
{
    if (_engine->_textureCache)
    {
        _engine->_textureCache->set_max_upload_dimension(dim);
    }
}

uint32_t Engine::get_max_upload_dimension() const
{
    return _engine->_textureCache ? _engine->_textureCache->max_upload_dimension() : 0;
}

void Engine::set_keep_source_bytes(bool keep)
{
    if (_engine->_textureCache)
    {
        _engine->_textureCache->set_keep_source_bytes(keep);
    }
}

bool Engine::get_keep_source_bytes() const
{
    return _engine->_textureCache ? _engine->_textureCache->keep_source_bytes() : false;
}

void Engine::evict_textures_to_budget()
{
    if (_engine->_textureCache)
    {
        size_t budget = _engine->query_texture_budget_bytes();
        _engine->_textureCache->evictToBudget(budget);
    }
}

// ----------------------------------------------------------------------------
// Shadows
// ----------------------------------------------------------------------------

void Engine::set_shadows_enabled(bool enabled)
{
    if (_engine->_context)
    {
        _engine->_context->shadowSettings.enabled = enabled;
    }
}

bool Engine::get_shadows_enabled() const
{
    return _engine->_context ? _engine->_context->shadowSettings.enabled : false;
}

void Engine::set_shadow_mode(ShadowMode mode)
{
    if (_engine->_context)
    {
        // Guard against requesting RT modes on unsupported hardware.
        if (mode != ShadowMode::ClipmapOnly)
        {
            if (!_engine->_deviceManager
                || !_engine->_deviceManager->supportsRayQuery()
                || !_engine->_deviceManager->supportsAccelerationStructure())
            {
                mode = ShadowMode::ClipmapOnly;
            }
        }

        _engine->_context->shadowSettings.mode = static_cast<uint32_t>(mode);
        _engine->_context->shadowSettings.hybridRayQueryEnabled =
            _engine->_context->shadowSettings.enabled && (mode != ShadowMode::ClipmapOnly);
    }
}

ShadowMode Engine::get_shadow_mode() const
{
    if (!_engine->_context) return ShadowMode::ClipmapOnly;
    return static_cast<ShadowMode>(_engine->_context->shadowSettings.mode);
}

void Engine::set_hybrid_ray_cascade_mask(uint32_t mask)
{
    if (_engine->_context)
    {
        _engine->_context->shadowSettings.hybridRayCascadesMask = mask & 0xF;
    }
}

uint32_t Engine::get_hybrid_ray_cascade_mask() const
{
    return _engine->_context ? _engine->_context->shadowSettings.hybridRayCascadesMask : 0;
}

void Engine::set_hybrid_ray_threshold(float threshold)
{
    if (_engine->_context)
    {
        _engine->_context->shadowSettings.hybridRayNoLThreshold = glm::clamp(threshold, 0.0f, 1.0f);
    }
}

float Engine::get_hybrid_ray_threshold() const
{
    return _engine->_context ? _engine->_context->shadowSettings.hybridRayNoLThreshold : 0.25f;
}

// ----------------------------------------------------------------------------
// IBL (Image-Based Lighting)
// ----------------------------------------------------------------------------

static ::IBLPaths to_internal_ibl_paths(const IBLPaths& p)
{
    ::IBLPaths out;
    out.specularCube = p.specularCube;
    out.diffuseCube = p.diffuseCube;
    out.brdfLut2D = p.brdfLut;
    out.background2D = p.background;
    return out;
}

static IBLPaths from_internal_ibl_paths(const ::IBLPaths& p)
{
    IBLPaths out;
    out.specularCube = p.specularCube;
    out.diffuseCube = p.diffuseCube;
    out.brdfLut = p.brdfLut2D;
    out.background = p.background2D;
    return out;
}

bool Engine::load_global_ibl(const IBLPaths& paths)
{
    if (!_engine->_iblManager) return false;

    ::IBLPaths internal = to_internal_ibl_paths(paths);
    _engine->_globalIBLPaths = internal;

    if (_engine->_iblManager->load_async(internal))
    {
        _engine->_pendingIBLRequest.active = true;
        _engine->_pendingIBLRequest.targetVolume = -1;
        _engine->_pendingIBLRequest.paths = internal;
        _engine->_hasGlobalIBL = false;
        return true;
    }
    return false;
}

IBLPaths Engine::get_global_ibl_paths() const
{
    return from_internal_ibl_paths(_engine->_globalIBLPaths);
}

void Engine::set_global_ibl_paths(const IBLPaths& paths)
{
    _engine->_globalIBLPaths = to_internal_ibl_paths(paths);
}

size_t Engine::add_ibl_volume(const IBLVolume& volume)
{
    VulkanEngine::IBLVolume v;
    v.center_world = WorldVec3(volume.center);
    v.halfExtents = volume.halfExtents;
    v.paths = to_internal_ibl_paths(volume.paths);
    v.enabled = volume.enabled;

    _engine->_iblVolumes.push_back(v);
    return _engine->_iblVolumes.size() - 1;
}

size_t Engine::add_ibl_volume(const IBLVolumeD& volume)
{
    VulkanEngine::IBLVolume v;
    v.center_world = WorldVec3(volume.center);
    v.halfExtents = volume.halfExtents;
    v.paths = to_internal_ibl_paths(volume.paths);
    v.enabled = volume.enabled;

    _engine->_iblVolumes.push_back(v);
    return _engine->_iblVolumes.size() - 1;
}

bool Engine::remove_ibl_volume(size_t index)
{
    if (index >= _engine->_iblVolumes.size()) return false;

    if (_engine->_activeIBLVolume == static_cast<int>(index))
    {
        _engine->_activeIBLVolume = -1;
    }
    else if (_engine->_activeIBLVolume > static_cast<int>(index))
    {
        _engine->_activeIBLVolume -= 1;
    }

    _engine->_iblVolumes.erase(_engine->_iblVolumes.begin() + index);
    return true;
}

bool Engine::get_ibl_volume(size_t index, IBLVolume& out) const
{
    if (index >= _engine->_iblVolumes.size()) return false;

    const auto& v = _engine->_iblVolumes[index];
    out.center = glm::vec3(v.center_world);
    out.halfExtents = v.halfExtents;
    out.paths = from_internal_ibl_paths(v.paths);
    out.enabled = v.enabled;
    return true;
}

bool Engine::get_ibl_volume(size_t index, IBLVolumeD& out) const
{
    if (index >= _engine->_iblVolumes.size()) return false;

    const auto& v = _engine->_iblVolumes[index];
    out.center = v.center_world;
    out.halfExtents = v.halfExtents;
    out.paths = from_internal_ibl_paths(v.paths);
    out.enabled = v.enabled;
    return true;
}

bool Engine::set_ibl_volume(size_t index, const IBLVolume& volume)
{
    if (index >= _engine->_iblVolumes.size()) return false;

    auto& v = _engine->_iblVolumes[index];
    v.center_world = WorldVec3(volume.center);
    v.halfExtents = volume.halfExtents;
    v.paths = to_internal_ibl_paths(volume.paths);
    v.enabled = volume.enabled;
    return true;
}

bool Engine::set_ibl_volume(size_t index, const IBLVolumeD& volume)
{
    if (index >= _engine->_iblVolumes.size()) return false;

    auto& v = _engine->_iblVolumes[index];
    v.center_world = WorldVec3(volume.center);
    v.halfExtents = volume.halfExtents;
    v.paths = to_internal_ibl_paths(volume.paths);
    v.enabled = volume.enabled;
    return true;
}

int Engine::get_active_ibl_volume() const
{
    return _engine->_activeIBLVolume;
}

size_t Engine::get_ibl_volume_count() const
{
    return _engine->_iblVolumes.size();
}

void Engine::clear_ibl_volumes()
{
    _engine->_iblVolumes.clear();
    _engine->_activeIBLVolume = -1;
}

// ----------------------------------------------------------------------------
// Objects / Instances
// ----------------------------------------------------------------------------

bool Engine::add_gltf_instance(const std::string& name,
                               const std::string& modelPath,
                               const Transform& transform,
                               bool preloadTextures)
{
    return _engine->addGLTFInstance(name, modelPath, transform.to_matrix(), preloadTextures);
}

bool Engine::add_gltf_instance(const std::string& name,
                               const std::string& modelPath,
                               const TransformD& transform,
                               bool preloadTextures)
{
    if (!_engine || !_engine->_sceneManager)
    {
        return false;
    }

    // Add the instance first (GPU resources), then apply the authoritative world transform in double.
    if (!_engine->addGLTFInstance(name, modelPath, glm::mat4(1.0f), preloadTextures))
    {
        return false;
    }

    return _engine->_sceneManager->setGLTFInstanceTRSWorld(name,
                                                          WorldVec3(transform.position),
                                                          transform.rotation,
                                                          transform.scale);
}

uint32_t Engine::add_gltf_instance_async(const std::string& name,
                                         const std::string& modelPath,
                                         const Transform& transform,
                                         bool preloadTextures)
{
    return _engine->loadGLTFAsync(name, modelPath, transform.to_matrix(), preloadTextures);
}

uint32_t Engine::add_gltf_instance_async(const std::string& name,
                                         const std::string& modelPath,
                                         const TransformD& transform,
                                         bool preloadTextures)
{
    return _engine->loadGLTFAsync(name,
                                 modelPath,
                                 WorldVec3(transform.position),
                                 transform.rotation,
                                 transform.scale,
                                 preloadTextures);
}

bool Engine::remove_gltf_instance(const std::string& name)
{
    return _engine->_sceneManager ? _engine->_sceneManager->removeGLTFInstance(name) : false;
}

bool Engine::get_gltf_instance_transform(const std::string& name, Transform& out) const
{
    if (!_engine->_sceneManager) return false;

    glm::mat4 m;
    if (_engine->_sceneManager->getGLTFInstanceTransform(name, m))
    {
        out = Transform::from_matrix(m);
        return true;
    }
    return false;
}

bool Engine::get_gltf_instance_transform(const std::string& name, TransformD& out) const
{
    if (!_engine->_sceneManager) return false;

    WorldVec3 t{};
    glm::quat r{};
    glm::vec3 s{};
    if (_engine->_sceneManager->getGLTFInstanceTRSWorld(name, t, r, s))
    {
        out.position = glm::dvec3(t);
        out.rotation = r;
        out.scale = s;
        return true;
    }
    return false;
}

bool Engine::set_gltf_instance_transform(const std::string& name, const Transform& transform)
{
    return _engine->_sceneManager
           ? _engine->_sceneManager->setGLTFInstanceTransform(name, transform.to_matrix())
           : false;
}

bool Engine::set_gltf_instance_transform(const std::string& name, const TransformD& transform)
{
    return _engine->_sceneManager
           ? _engine->_sceneManager->setGLTFInstanceTRSWorld(name,
                                                            WorldVec3(transform.position),
                                                            transform.rotation,
                                                            transform.scale)
           : false;
}

bool Engine::add_primitive_instance(const std::string& name,
                                    PrimitiveType type,
                                    const Transform& transform)
{
    AssetManager::MeshGeometryDesc::Type geomType;
    switch (type)
    {
    case PrimitiveType::Cube:    geomType = AssetManager::MeshGeometryDesc::Type::Cube; break;
    case PrimitiveType::Sphere:  geomType = AssetManager::MeshGeometryDesc::Type::Sphere; break;
    case PrimitiveType::Plane:   geomType = AssetManager::MeshGeometryDesc::Type::Plane; break;
    case PrimitiveType::Capsule: geomType = AssetManager::MeshGeometryDesc::Type::Capsule; break;
    default: return false;
    }

    return _engine->addPrimitiveInstance(name, geomType, transform.to_matrix());
}

bool Engine::add_primitive_instance(const std::string& name,
                                    PrimitiveType type,
                                    const TransformD& transform)
{
    AssetManager::MeshGeometryDesc::Type geomType;
    switch (type)
    {
    case PrimitiveType::Cube:    geomType = AssetManager::MeshGeometryDesc::Type::Cube; break;
    case PrimitiveType::Sphere:  geomType = AssetManager::MeshGeometryDesc::Type::Sphere; break;
    case PrimitiveType::Plane:   geomType = AssetManager::MeshGeometryDesc::Type::Plane; break;
    case PrimitiveType::Capsule: geomType = AssetManager::MeshGeometryDesc::Type::Capsule; break;
    default: return false;
    }

    if (!_engine->addPrimitiveInstance(name, geomType, glm::mat4(1.0f)))
    {
        return false;
    }
    return _engine->_sceneManager
           ? _engine->_sceneManager->setMeshInstanceTRSWorld(name,
                                                            WorldVec3(transform.position),
                                                            transform.rotation,
                                                            transform.scale)
           : false;
}

bool Engine::remove_mesh_instance(const std::string& name)
{
    return _engine->_sceneManager ? _engine->_sceneManager->removeMeshInstance(name) : false;
}

bool Engine::get_mesh_instance_transform(const std::string& name, Transform& out) const
{
    if (!_engine->_sceneManager) return false;

    glm::mat4 m;
    if (_engine->_sceneManager->getMeshInstanceTransform(name, m))
    {
        out = Transform::from_matrix(m);
        return true;
    }
    return false;
}

bool Engine::get_mesh_instance_transform(const std::string& name, TransformD& out) const
{
    if (!_engine->_sceneManager) return false;

    WorldVec3 t{};
    glm::quat r{};
    glm::vec3 s{};
    if (_engine->_sceneManager->getMeshInstanceTRSWorld(name, t, r, s))
    {
        out.position = glm::dvec3(t);
        out.rotation = r;
        out.scale = s;
        return true;
    }
    return false;
}

bool Engine::set_mesh_instance_transform(const std::string& name, const Transform& transform)
{
    return _engine->_sceneManager
           ? _engine->_sceneManager->setMeshInstanceTransform(name, transform.to_matrix())
           : false;
}

bool Engine::set_mesh_instance_transform(const std::string& name, const TransformD& transform)
{
    return _engine->_sceneManager
           ? _engine->_sceneManager->setMeshInstanceTRSWorld(name,
                                                            WorldVec3(transform.position),
                                                            transform.rotation,
                                                            transform.scale)
           : false;
}

void Engine::preload_instance_textures(const std::string& name)
{
    _engine->preloadInstanceTextures(name);
}

void Engine::clear_all_instances()
{
    if (_engine->_sceneManager)
    {
        _engine->_sceneManager->clearGLTFInstances();
        _engine->_sceneManager->clearMeshInstances();
    }
}

// ----------------------------------------------------------------------------
// Animation
// ----------------------------------------------------------------------------

bool Engine::set_instance_animation(const std::string& instanceName, int animationIndex, bool resetTime)
{
    return _engine->_sceneManager
           ? _engine->_sceneManager->setGLTFInstanceAnimation(instanceName, animationIndex, resetTime)
           : false;
}

bool Engine::set_instance_animation(const std::string& instanceName, const std::string& animationName, bool resetTime)
{
    return _engine->_sceneManager
           ? _engine->_sceneManager->setGLTFInstanceAnimation(instanceName, animationName, resetTime)
           : false;
}

bool Engine::set_instance_animation_loop(const std::string& instanceName, bool loop)
{
    return _engine->_sceneManager
           ? _engine->_sceneManager->setGLTFInstanceAnimationLoop(instanceName, loop)
           : false;
}

bool Engine::set_instance_node_offset(const std::string& instanceName, const std::string& nodeName, const glm::mat4& offset)
{
    return _engine->_sceneManager
           ? _engine->_sceneManager->setGLTFInstanceNodeOffset(instanceName, nodeName, offset)
           : false;
}

bool Engine::clear_instance_node_offset(const std::string& instanceName, const std::string& nodeName)
{
    return _engine->_sceneManager
           ? _engine->_sceneManager->clearGLTFInstanceNodeOffset(instanceName, nodeName)
           : false;
}

void Engine::clear_all_instance_node_offsets(const std::string& instanceName)
{
    if (_engine->_sceneManager)
    {
        _engine->_sceneManager->clearGLTFInstanceNodeOffsets(instanceName);
    }
}

// ----------------------------------------------------------------------------
// Lighting
// ----------------------------------------------------------------------------

size_t Engine::add_point_light(const PointLight& light)
{
    if (!_engine->_sceneManager) return 0;

    SceneManager::PointLight pl;
    pl.position_world = WorldVec3(light.position);
    pl.radius = light.radius;
    pl.color = light.color;
    pl.intensity = light.intensity;

    size_t idx = _engine->_sceneManager->getPointLightCount();
    _engine->_sceneManager->addPointLight(pl);
    return idx;
}

size_t Engine::add_point_light(const PointLightD& light)
{
    if (!_engine->_sceneManager) return 0;

    SceneManager::PointLight pl;
    pl.position_world = WorldVec3(light.position);
    pl.radius = light.radius;
    pl.color = light.color;
    pl.intensity = light.intensity;

    size_t idx = _engine->_sceneManager->getPointLightCount();
    _engine->_sceneManager->addPointLight(pl);
    return idx;
}

bool Engine::remove_point_light(size_t index)
{
    return _engine->_sceneManager ? _engine->_sceneManager->removePointLight(index) : false;
}

bool Engine::get_point_light(size_t index, PointLight& out) const
{
    if (!_engine->_sceneManager) return false;

    SceneManager::PointLight pl;
    if (_engine->_sceneManager->getPointLight(index, pl))
    {
        out.position = glm::vec3(pl.position_world);
        out.radius = pl.radius;
        out.color = pl.color;
        out.intensity = pl.intensity;
        return true;
    }
    return false;
}

bool Engine::get_point_light(size_t index, PointLightD& out) const
{
    if (!_engine->_sceneManager) return false;

    SceneManager::PointLight pl;
    if (_engine->_sceneManager->getPointLight(index, pl))
    {
        out.position = pl.position_world;
        out.radius = pl.radius;
        out.color = pl.color;
        out.intensity = pl.intensity;
        return true;
    }
    return false;
}

bool Engine::set_point_light(size_t index, const PointLight& light)
{
    if (!_engine->_sceneManager) return false;

    SceneManager::PointLight pl;
    pl.position_world = WorldVec3(light.position);
    pl.radius = light.radius;
    pl.color = light.color;
    pl.intensity = light.intensity;

    return _engine->_sceneManager->setPointLight(index, pl);
}

bool Engine::set_point_light(size_t index, const PointLightD& light)
{
    if (!_engine->_sceneManager) return false;

    SceneManager::PointLight pl;
    pl.position_world = WorldVec3(light.position);
    pl.radius = light.radius;
    pl.color = light.color;
    pl.intensity = light.intensity;

    return _engine->_sceneManager->setPointLight(index, pl);
}

size_t Engine::get_point_light_count() const
{
    return _engine->_sceneManager ? _engine->_sceneManager->getPointLightCount() : 0;
}

void Engine::clear_point_lights()
{
    if (_engine->_sceneManager)
    {
        _engine->_sceneManager->clearPointLights();
    }
}

// ----------------------------------------------------------------------------
// Post Processing - FXAA
// ----------------------------------------------------------------------------

void Engine::set_fxaa_enabled(bool enabled)
{
    if (!_engine->_renderPassManager) return;
    if (auto* fxaa = _engine->_renderPassManager->getPass<FxaaPass>())
    {
        fxaa->set_enabled(enabled);
    }
}

bool Engine::get_fxaa_enabled() const
{
    if (!_engine->_renderPassManager) return false;
    if (auto* fxaa = _engine->_renderPassManager->getPass<FxaaPass>())
    {
        return fxaa->enabled();
    }
    return false;
}

void Engine::set_fxaa_edge_threshold(float threshold)
{
    if (!_engine->_renderPassManager) return;
    if (auto* fxaa = _engine->_renderPassManager->getPass<FxaaPass>())
    {
        fxaa->set_edge_threshold(threshold);
    }
}

float Engine::get_fxaa_edge_threshold() const
{
    if (!_engine->_renderPassManager) return 0.125f;
    if (auto* fxaa = _engine->_renderPassManager->getPass<FxaaPass>())
    {
        return fxaa->edge_threshold();
    }
    return 0.125f;
}

void Engine::set_fxaa_edge_threshold_min(float threshold)
{
    if (!_engine->_renderPassManager) return;
    if (auto* fxaa = _engine->_renderPassManager->getPass<FxaaPass>())
    {
        fxaa->set_edge_threshold_min(threshold);
    }
}

float Engine::get_fxaa_edge_threshold_min() const
{
    if (!_engine->_renderPassManager) return 0.0312f;
    if (auto* fxaa = _engine->_renderPassManager->getPass<FxaaPass>())
    {
        return fxaa->edge_threshold_min();
    }
    return 0.0312f;
}

// ----------------------------------------------------------------------------
// Post Processing - SSR
// ----------------------------------------------------------------------------

void Engine::set_ssr_enabled(bool enabled)
{
    if (_engine->_context)
    {
        _engine->_context->enableSSR = enabled;
    }
}

bool Engine::get_ssr_enabled() const
{
    return _engine->_context ? _engine->_context->enableSSR : false;
}

void Engine::set_reflection_mode(ReflectionMode mode)
{
    if (_engine->_context)
    {
        // Guard against requesting RT reflection modes on unsupported hardware.
        if (mode != ReflectionMode::SSROnly)
        {
            if (!_engine->_deviceManager
                || !_engine->_deviceManager->supportsRayQuery()
                || !_engine->_deviceManager->supportsAccelerationStructure())
            {
                mode = ReflectionMode::SSROnly;
            }
        }

        _engine->_context->reflectionMode = static_cast<uint32_t>(mode);
    }
}

ReflectionMode Engine::get_reflection_mode() const
{
    if (!_engine->_context) return ReflectionMode::SSROnly;
    return static_cast<ReflectionMode>(_engine->_context->reflectionMode);
}

// ----------------------------------------------------------------------------
// Post Processing - Tonemapping
// ----------------------------------------------------------------------------

void Engine::set_exposure(float exposure)
{
    if (!_engine->_renderPassManager) return;
    if (auto* tonemap = _engine->_renderPassManager->getPass<TonemapPass>())
    {
        tonemap->setExposure(exposure);
    }
}

float Engine::get_exposure() const
{
    if (!_engine->_renderPassManager) return 1.0f;
    if (auto* tonemap = _engine->_renderPassManager->getPass<TonemapPass>())
    {
        return tonemap->exposure();
    }
    return 1.0f;
}

void Engine::set_tonemap_operator(TonemapOperator op)
{
    if (!_engine->_renderPassManager) return;
    if (auto* tonemap = _engine->_renderPassManager->getPass<TonemapPass>())
    {
        tonemap->setMode(static_cast<int>(op));
    }
}

TonemapOperator Engine::get_tonemap_operator() const
{
    if (!_engine->_renderPassManager) return TonemapOperator::ACES;
    if (auto* tonemap = _engine->_renderPassManager->getPass<TonemapPass>())
    {
        return static_cast<TonemapOperator>(tonemap->mode());
    }
    return TonemapOperator::ACES;
}

// ----------------------------------------------------------------------------
// Post Processing - Bloom
// ----------------------------------------------------------------------------

void Engine::set_bloom_enabled(bool enabled)
{
    if (!_engine->_renderPassManager) return;
    if (auto* tonemap = _engine->_renderPassManager->getPass<TonemapPass>())
    {
        tonemap->setBloomEnabled(enabled);
    }
}

bool Engine::get_bloom_enabled() const
{
    if (!_engine->_renderPassManager) return false;
    if (auto* tonemap = _engine->_renderPassManager->getPass<TonemapPass>())
    {
        return tonemap->bloomEnabled();
    }
    return false;
}

void Engine::set_bloom_threshold(float threshold)
{
    if (!_engine->_renderPassManager) return;
    if (auto* tonemap = _engine->_renderPassManager->getPass<TonemapPass>())
    {
        tonemap->setBloomThreshold(threshold);
    }
}

float Engine::get_bloom_threshold() const
{
    if (!_engine->_renderPassManager) return 1.0f;
    if (auto* tonemap = _engine->_renderPassManager->getPass<TonemapPass>())
    {
        return tonemap->bloomThreshold();
    }
    return 1.0f;
}

void Engine::set_bloom_intensity(float intensity)
{
    if (!_engine->_renderPassManager) return;
    if (auto* tonemap = _engine->_renderPassManager->getPass<TonemapPass>())
    {
        tonemap->setBloomIntensity(intensity);
    }
}

float Engine::get_bloom_intensity() const
{
    if (!_engine->_renderPassManager) return 0.7f;
    if (auto* tonemap = _engine->_renderPassManager->getPass<TonemapPass>())
    {
        return tonemap->bloomIntensity();
    }
    return 0.7f;
}

// ----------------------------------------------------------------------------
// Camera
// ----------------------------------------------------------------------------

void Engine::set_camera_position(const glm::vec3& position)
{
    if (_engine->_sceneManager)
    {
        _engine->_sceneManager->getMainCamera().position_world = WorldVec3(position);
    }
}

glm::vec3 Engine::get_camera_position() const
{
    if (_engine->_sceneManager)
    {
        return glm::vec3(_engine->_sceneManager->getMainCamera().position_world);
    }
    return glm::vec3(0.0f);
}

void Engine::set_camera_position(const glm::dvec3& position)
{
    if (_engine->_sceneManager)
    {
        _engine->_sceneManager->getMainCamera().position_world = position;
    }
}

glm::dvec3 Engine::get_camera_position_d() const
{
    if (_engine->_sceneManager)
    {
        return _engine->_sceneManager->getMainCamera().position_world;
    }
    return glm::dvec3(0.0);
}

void Engine::set_camera_rotation(float pitch, float yaw)
{
    if (_engine->_sceneManager)
    {
        Camera& cam = _engine->_sceneManager->getMainCamera();

        // Convert degrees to radians.
        float pitchRad = glm::radians(pitch);
        float yawRad = glm::radians(yaw);

        // -Z forward convention: yaw around +Y, then pitch around local +X.
        glm::quat yawQ = glm::angleAxis(yawRad, glm::vec3(0.0f, 1.0f, 0.0f));
        glm::quat pitchQ = glm::angleAxis(pitchRad, glm::vec3(1.0f, 0.0f, 0.0f));

        cam.orientation = glm::normalize(yawQ * pitchQ);
    }
}

void Engine::get_camera_rotation(float& pitch, float& yaw) const
{
    if (_engine->_sceneManager)
    {
        const Camera& cam = _engine->_sceneManager->getMainCamera();

        // Derive forward from orientation and convert to pitch/yaw (degrees).
        glm::vec3 forward = glm::rotate(cam.orientation, glm::vec3(0.0f, 0.0f, -1.0f));
        forward = glm::normalize(forward);

        pitch = glm::degrees(asinf(-forward.y));
        yaw = glm::degrees(atan2f(forward.x, forward.z));
    }
    else
    {
        pitch = 0.0f;
        yaw = 0.0f;
    }
}

void Engine::set_camera_fov(float fovDegrees)
{
    if (_engine->_sceneManager)
    {
        _engine->_sceneManager->getMainCamera().fovDegrees = fovDegrees;
    }
}

float Engine::get_camera_fov() const
{
    if (_engine->_sceneManager)
    {
        return _engine->_sceneManager->getMainCamera().fovDegrees;
    }
    return 70.0f;
}

void Engine::camera_look_at(const glm::vec3& target)
{
    if (!_engine->_sceneManager) return;

    Camera& cam = _engine->_sceneManager->getMainCamera();
    glm::vec3 dir = glm::normalize(target - glm::vec3(cam.position_world));

    // For a -Z forward convention, build a quaternion that rotates -Z into dir.
    // Use glm's lookAt-style helper via matrices, then convert to a quaternion.
    glm::vec3 up(0.0f, 1.0f, 0.0f);
    if (glm::length2(glm::cross(dir, up)) < 1e-6f)
    {
        up = glm::vec3(0.0f, 0.0f, 1.0f);
    }

    glm::vec3 f = dir;
    glm::vec3 r = glm::normalize(glm::cross(up, f));
    glm::vec3 u = glm::cross(f, r);

    glm::mat3 rot;
    rot[0] = r;
    rot[1] = u;
    rot[2] = -f; // -Z forward

    cam.orientation = glm::quat_cast(rot);
}

void Engine::camera_look_at(const glm::dvec3& target)
{
    if (!_engine->_sceneManager) return;

    Camera& cam = _engine->_sceneManager->getMainCamera();
    glm::dvec3 dirD = glm::normalize(target - cam.position_world);
    glm::vec3 dir = glm::normalize(glm::vec3(dirD));

    // For a -Z forward convention, build a quaternion that rotates -Z into dir.
    glm::vec3 up(0.0f, 1.0f, 0.0f);
    if (glm::length2(glm::cross(dir, up)) < 1e-6f)
    {
        up = glm::vec3(0.0f, 0.0f, 1.0f);
    }

    glm::vec3 f = dir;
    glm::vec3 r = glm::normalize(glm::cross(up, f));
    glm::vec3 u = glm::cross(f, r);

    glm::mat3 rot;
    rot[0] = r;
    rot[1] = u;
    rot[2] = -f; // -Z forward

    cam.orientation = glm::quat_cast(rot);
}

// ----------------------------------------------------------------------------
// Rendering
// ----------------------------------------------------------------------------

void Engine::set_render_scale(float scale)
{
    _engine->renderScale = glm::clamp(scale, 0.3f, 1.0f);
}

float Engine::get_render_scale() const
{
    return _engine->renderScale;
}

void Engine::set_pass_enabled(const std::string& passName, bool enabled)
{
    _engine->_rgPassToggles[passName] = enabled;
}

bool Engine::get_pass_enabled(const std::string& passName) const
{
    auto it = _engine->_rgPassToggles.find(passName);
    if (it != _engine->_rgPassToggles.end())
    {
        return it->second;
    }
    return true; // Default to enabled if not in map
}

void Engine::hot_reload_shaders()
{
    if (_engine->_pipelineManager)
    {
        _engine->_pipelineManager->hotReloadChanged();
    }
}

// ----------------------------------------------------------------------------
// Statistics
// ----------------------------------------------------------------------------

Stats Engine::get_stats() const
{
    Stats s;
    s.frametime = _engine->stats.frametime;
    s.drawTime = _engine->stats.mesh_draw_time;
    s.sceneUpdateTime = _engine->stats.scene_update_time;
    s.triangleCount = _engine->stats.triangle_count;
    s.drawCallCount = _engine->stats.drawcall_count;
    return s;
}

// ----------------------------------------------------------------------------
// Picking / Selection
// ----------------------------------------------------------------------------

Engine::PickResult Engine::get_last_pick() const
{
    PickResult r;
    r.valid = _engine->_lastPick.valid;
    r.ownerName = _engine->_lastPick.ownerName;
    r.worldPosition = glm::vec3(_engine->_lastPick.worldPos);
    return r;
}

Engine::PickResultD Engine::get_last_pick_d() const
{
    PickResultD r;
    r.valid = _engine->_lastPick.valid;
    r.ownerName = _engine->_lastPick.ownerName;
    r.worldPosition = _engine->_lastPick.worldPos;
    return r;
}

void Engine::set_use_id_buffer_picking(bool use)
{
    _engine->_useIdBufferPicking = use;
}

bool Engine::get_use_id_buffer_picking() const
{
    return _engine->_useIdBufferPicking;
}

} // namespace GameAPI
