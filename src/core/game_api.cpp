#include "game_api.h"
#include "engine.h"
#include "context.h"
#include "core/assets/texture_cache.h"
#include "core/assets/ibl_manager.h"
#include "core/pipeline/manager.h"
#include "core/debug_draw/debug_draw.h"
#include "render/passes/tonemap.h"
#include "render/passes/fxaa.h"
#include "render/passes/particles.h"
#include "render/renderpass.h"
#include "core/picking/picking_system.h"
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

bool Engine::add_textured_primitive(const std::string& name,
                                    PrimitiveType type,
                                    const PrimitiveMaterial& material,
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

    AssetManager::MeshMaterialDesc matDesc;
    matDesc.kind = AssetManager::MeshMaterialDesc::Kind::Textured;
    matDesc.options.albedoPath = material.albedoPath;
    matDesc.options.metalRoughPath = material.metalRoughPath;
    matDesc.options.normalPath = material.normalPath;
    matDesc.options.occlusionPath = material.occlusionPath;
    matDesc.options.emissivePath = material.emissivePath;
    matDesc.options.constants.colorFactors = material.colorFactor;
    matDesc.options.constants.metal_rough_factors = glm::vec4(material.metallic, material.roughness, 0.0f, 0.0f);

    return _engine->addPrimitiveInstance(name, geomType, transform.to_matrix(), matDesc);
}

bool Engine::add_textured_primitive(const std::string& name,
                                    PrimitiveType type,
                                    const PrimitiveMaterial& material,
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

    AssetManager::MeshMaterialDesc matDesc;
    matDesc.kind = AssetManager::MeshMaterialDesc::Kind::Textured;
    matDesc.options.albedoPath = material.albedoPath;
    matDesc.options.metalRoughPath = material.metalRoughPath;
    matDesc.options.normalPath = material.normalPath;
    matDesc.options.occlusionPath = material.occlusionPath;
    matDesc.options.emissivePath = material.emissivePath;
    matDesc.options.constants.colorFactors = material.colorFactor;
    matDesc.options.constants.metal_rough_factors = glm::vec4(material.metallic, material.roughness, 0.0f, 0.0f);

    if (!_engine->addPrimitiveInstance(name, geomType, glm::mat4(1.0f), matDesc))
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
// Lighting - Directional (Sunlight)
// ----------------------------------------------------------------------------

void Engine::set_sunlight_direction(const glm::vec3& dir)
{
    if (_engine->_sceneManager)
    {
        _engine->_sceneManager->setSunlightDirection(dir);
    }
}

glm::vec3 Engine::get_sunlight_direction() const
{
    if (_engine->_sceneManager)
    {
        return _engine->_sceneManager->getSunlightDirection();
    }
    return glm::vec3(0.0f, -1.0f, 0.0f);
}

void Engine::set_sunlight_color(const glm::vec3& color, float intensity)
{
    if (_engine->_sceneManager)
    {
        _engine->_sceneManager->setSunlightColor(color, intensity);
    }
}

glm::vec3 Engine::get_sunlight_color() const
{
    if (_engine->_sceneManager)
    {
        return _engine->_sceneManager->getSunlightColor();
    }
    return glm::vec3(1.0f);
}

float Engine::get_sunlight_intensity() const
{
    if (_engine->_sceneManager)
    {
        return _engine->_sceneManager->getSunlightIntensity();
    }
    return 1.0f;
}

// ----------------------------------------------------------------------------
// Lighting - Point Lights
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
// Lighting - Spot Lights
// ----------------------------------------------------------------------------

size_t Engine::add_spot_light(const SpotLight& light)
{
    if (!_engine->_sceneManager) return 0;

    SceneManager::SpotLight sl;
    sl.position_world = WorldVec3(light.position);
    sl.direction = (glm::length(light.direction) > 1.0e-6f)
        ? glm::normalize(light.direction)
        : glm::vec3(0.0f, -1.0f, 0.0f);
    sl.radius = light.radius;
    sl.color = light.color;
    sl.intensity = light.intensity;
    sl.inner_angle_deg = light.inner_angle_deg;
    sl.outer_angle_deg = light.outer_angle_deg;

    size_t idx = _engine->_sceneManager->getSpotLightCount();
    _engine->_sceneManager->addSpotLight(sl);
    return idx;
}

size_t Engine::add_spot_light(const SpotLightD& light)
{
    if (!_engine->_sceneManager) return 0;

    SceneManager::SpotLight sl;
    sl.position_world = WorldVec3(light.position);
    sl.direction = (glm::length(light.direction) > 1.0e-6f)
        ? glm::normalize(light.direction)
        : glm::vec3(0.0f, -1.0f, 0.0f);
    sl.radius = light.radius;
    sl.color = light.color;
    sl.intensity = light.intensity;
    sl.inner_angle_deg = light.inner_angle_deg;
    sl.outer_angle_deg = light.outer_angle_deg;

    size_t idx = _engine->_sceneManager->getSpotLightCount();
    _engine->_sceneManager->addSpotLight(sl);
    return idx;
}

bool Engine::remove_spot_light(size_t index)
{
    return _engine->_sceneManager ? _engine->_sceneManager->removeSpotLight(index) : false;
}

bool Engine::get_spot_light(size_t index, SpotLight& out) const
{
    if (!_engine->_sceneManager) return false;

    SceneManager::SpotLight sl;
    if (_engine->_sceneManager->getSpotLight(index, sl))
    {
        out.position = glm::vec3(sl.position_world);
        out.direction = sl.direction;
        out.radius = sl.radius;
        out.color = sl.color;
        out.intensity = sl.intensity;
        out.inner_angle_deg = sl.inner_angle_deg;
        out.outer_angle_deg = sl.outer_angle_deg;
        return true;
    }
    return false;
}

bool Engine::get_spot_light(size_t index, SpotLightD& out) const
{
    if (!_engine->_sceneManager) return false;

    SceneManager::SpotLight sl;
    if (_engine->_sceneManager->getSpotLight(index, sl))
    {
        out.position = sl.position_world;
        out.direction = sl.direction;
        out.radius = sl.radius;
        out.color = sl.color;
        out.intensity = sl.intensity;
        out.inner_angle_deg = sl.inner_angle_deg;
        out.outer_angle_deg = sl.outer_angle_deg;
        return true;
    }
    return false;
}

bool Engine::set_spot_light(size_t index, const SpotLight& light)
{
    if (!_engine->_sceneManager) return false;

    SceneManager::SpotLight sl;
    sl.position_world = WorldVec3(light.position);
    sl.direction = (glm::length(light.direction) > 1.0e-6f)
        ? glm::normalize(light.direction)
        : glm::vec3(0.0f, -1.0f, 0.0f);
    sl.radius = light.radius;
    sl.color = light.color;
    sl.intensity = light.intensity;
    sl.inner_angle_deg = light.inner_angle_deg;
    sl.outer_angle_deg = light.outer_angle_deg;

    return _engine->_sceneManager->setSpotLight(index, sl);
}

bool Engine::set_spot_light(size_t index, const SpotLightD& light)
{
    if (!_engine->_sceneManager) return false;

    SceneManager::SpotLight sl;
    sl.position_world = WorldVec3(light.position);
    sl.direction = (glm::length(light.direction) > 1.0e-6f)
        ? glm::normalize(light.direction)
        : glm::vec3(0.0f, -1.0f, 0.0f);
    sl.radius = light.radius;
    sl.color = light.color;
    sl.intensity = light.intensity;
    sl.inner_angle_deg = light.inner_angle_deg;
    sl.outer_angle_deg = light.outer_angle_deg;

    return _engine->_sceneManager->setSpotLight(index, sl);
}

size_t Engine::get_spot_light_count() const
{
    return _engine->_sceneManager ? _engine->_sceneManager->getSpotLightCount() : 0;
}

void Engine::clear_spot_lights()
{
    if (_engine->_sceneManager)
    {
        _engine->_sceneManager->clearSpotLights();
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
// Time
// ----------------------------------------------------------------------------

float Engine::get_delta_time() const
{
    if (_engine->_sceneManager)
    {
        return _engine->_sceneManager->getDeltaTime();
    }
    return 0.0f;
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
// Volumetrics (Cloud/Smoke/Flame)
// ----------------------------------------------------------------------------

void Engine::set_volumetrics_enabled(bool enabled)
{
    if (!_engine || !_engine->_context) return;
    _engine->_context->enableVolumetrics = enabled;
}

bool Engine::get_volumetrics_enabled() const
{
    if (!_engine || !_engine->_context) return false;
    return _engine->_context->enableVolumetrics;
}

bool Engine::get_voxel_volume(size_t index, VoxelVolumeSettings& out) const
{
    if (!_engine || !_engine->_context) return false;
    if (index >= EngineContext::MAX_VOXEL_VOLUMES) return false;

    const auto& src = _engine->_context->voxelVolumes[index];

    out.enabled = src.enabled;
    out.type = static_cast<VoxelVolumeType>(src.type);
    out.followCameraXZ = src.followCameraXZ;
    out.animateVoxels = src.animateVoxels;
    out.volumeCenterLocal = src.volumeCenterLocal;
    out.volumeHalfExtents = src.volumeHalfExtents;
    out.volumeVelocityLocal = src.volumeVelocityLocal;
    out.densityScale = src.densityScale;
    out.coverage = src.coverage;
    out.extinction = src.extinction;
    out.stepCount = src.stepCount;
    out.gridResolution = src.gridResolution;
    out.windVelocityLocal = src.windVelocityLocal;
    out.dissipation = src.dissipation;
    out.noiseStrength = src.noiseStrength;
    out.noiseScale = src.noiseScale;
    out.noiseSpeed = src.noiseSpeed;
    out.emitterUVW = src.emitterUVW;
    out.emitterRadius = src.emitterRadius;
    out.albedo = src.albedo;
    out.scatterStrength = src.scatterStrength;
    out.emissionColor = src.emissionColor;
    out.emissionStrength = src.emissionStrength;

    return true;
}

bool Engine::set_voxel_volume(size_t index, const VoxelVolumeSettings& settings)
{
    if (!_engine || !_engine->_context) return false;
    if (index >= EngineContext::MAX_VOXEL_VOLUMES) return false;

    auto& dst = _engine->_context->voxelVolumes[index];

    dst.enabled = settings.enabled;
    dst.type = static_cast<::VoxelVolumeType>(settings.type);
    dst.followCameraXZ = settings.followCameraXZ;
    dst.animateVoxels = settings.animateVoxels;
    dst.volumeCenterLocal = settings.volumeCenterLocal;
    dst.volumeHalfExtents = settings.volumeHalfExtents;
    dst.volumeVelocityLocal = settings.volumeVelocityLocal;
    dst.densityScale = settings.densityScale;
    dst.coverage = settings.coverage;
    dst.extinction = settings.extinction;
    dst.stepCount = settings.stepCount;
    dst.gridResolution = settings.gridResolution;
    dst.windVelocityLocal = settings.windVelocityLocal;
    dst.dissipation = settings.dissipation;
    dst.noiseStrength = settings.noiseStrength;
    dst.noiseScale = settings.noiseScale;
    dst.noiseSpeed = settings.noiseSpeed;
    dst.emitterUVW = settings.emitterUVW;
    dst.emitterRadius = settings.emitterRadius;
    dst.albedo = settings.albedo;
    dst.scatterStrength = settings.scatterStrength;
    dst.emissionColor = settings.emissionColor;
    dst.emissionStrength = settings.emissionStrength;

    return true;
}

size_t Engine::get_max_voxel_volumes() const
{
    return EngineContext::MAX_VOXEL_VOLUMES;
}

// ----------------------------------------------------------------------------
// Particle Systems
// ----------------------------------------------------------------------------

uint32_t Engine::create_particle_system(uint32_t particle_count)
{
    if (!_engine || !_engine->_renderPassManager) return 0;

    ParticlePass* particlePass = _engine->_renderPassManager->getPass<ParticlePass>();
    if (!particlePass) return 0;

    return particlePass->create_system(particle_count);
}

bool Engine::destroy_particle_system(uint32_t id)
{
    if (!_engine || !_engine->_renderPassManager) return false;

    ParticlePass* particlePass = _engine->_renderPassManager->getPass<ParticlePass>();
    if (!particlePass) return false;

    return particlePass->destroy_system(id);
}

bool Engine::resize_particle_system(uint32_t id, uint32_t new_count)
{
    if (!_engine || !_engine->_renderPassManager) return false;

    ParticlePass* particlePass = _engine->_renderPassManager->getPass<ParticlePass>();
    if (!particlePass) return false;

    return particlePass->resize_system(id, new_count);
}

bool Engine::get_particle_system(uint32_t id, ParticleSystem& out) const
{
    if (!_engine || !_engine->_renderPassManager) return false;

    ParticlePass* particlePass = _engine->_renderPassManager->getPass<ParticlePass>();
    if (!particlePass) return false;

    const auto& systems = particlePass->systems();
    for (const auto& sys : systems)
    {
        if (sys.id == id)
        {
            out.id = sys.id;
            out.particleCount = sys.count;
            out.enabled = sys.enabled;
            out.reset = sys.reset;
            out.blendMode = static_cast<ParticleBlendMode>(sys.blend);
            out.flipbookTexture = sys.flipbook_texture;
            out.noiseTexture = sys.noise_texture;

            // Copy parameters
            const auto& p = sys.params;
            out.params.emitterPosLocal = p.emitter_pos_local;
            out.params.spawnRadius = p.spawn_radius;
            out.params.emitterDirLocal = p.emitter_dir_local;
            out.params.coneAngleDegrees = p.cone_angle_degrees;
            out.params.minSpeed = p.min_speed;
            out.params.maxSpeed = p.max_speed;
            out.params.minLife = p.min_life;
            out.params.maxLife = p.max_life;
            out.params.minSize = p.min_size;
            out.params.maxSize = p.max_size;
            out.params.drag = p.drag;
            out.params.gravity = p.gravity;
            out.params.color = p.color;
            out.params.softDepthDistance = p.soft_depth_distance;
            out.params.flipbookCols = p.flipbook_cols;
            out.params.flipbookRows = p.flipbook_rows;
            out.params.flipbookFps = p.flipbook_fps;
            out.params.flipbookIntensity = p.flipbook_intensity;
            out.params.noiseScale = p.noise_scale;
            out.params.noiseStrength = p.noise_strength;
            out.params.noiseScroll = p.noise_scroll;

            return true;
        }
    }

    return false;
}

bool Engine::set_particle_system(uint32_t id, const ParticleSystem& system)
{
    if (!_engine || !_engine->_renderPassManager) return false;

    ParticlePass* particlePass = _engine->_renderPassManager->getPass<ParticlePass>();
    if (!particlePass) return false;

    auto& systems = particlePass->systems();
    for (auto& sys : systems)
    {
        if (sys.id == id)
        {
            sys.enabled = system.enabled;
            sys.reset = system.reset;
            sys.blend = static_cast<ParticlePass::BlendMode>(system.blendMode);
            sys.flipbook_texture = system.flipbookTexture;
            sys.noise_texture = system.noiseTexture;

            // Copy parameters
            auto& p = sys.params;
            p.emitter_pos_local = system.params.emitterPosLocal;
            p.spawn_radius = system.params.spawnRadius;
            p.emitter_dir_local = system.params.emitterDirLocal;
            p.cone_angle_degrees = system.params.coneAngleDegrees;
            p.min_speed = system.params.minSpeed;
            p.max_speed = system.params.maxSpeed;
            p.min_life = system.params.minLife;
            p.max_life = system.params.maxLife;
            p.min_size = system.params.minSize;
            p.max_size = system.params.maxSize;
            p.drag = system.params.drag;
            p.gravity = system.params.gravity;
            p.color = system.params.color;
            p.soft_depth_distance = system.params.softDepthDistance;
            p.flipbook_cols = system.params.flipbookCols;
            p.flipbook_rows = system.params.flipbookRows;
            p.flipbook_fps = system.params.flipbookFps;
            p.flipbook_intensity = system.params.flipbookIntensity;
            p.noise_scale = system.params.noiseScale;
            p.noise_strength = system.params.noiseStrength;
            p.noise_scroll = system.params.noiseScroll;

            // Preload textures if changed
            if (!sys.flipbook_texture.empty())
            {
                particlePass->preload_vfx_texture(sys.flipbook_texture);
            }
            if (!sys.noise_texture.empty())
            {
                particlePass->preload_vfx_texture(sys.noise_texture);
            }

            return true;
        }
    }

    return false;
}

std::vector<uint32_t> Engine::get_particle_system_ids() const
{
    std::vector<uint32_t> ids;

    if (!_engine || !_engine->_renderPassManager) return ids;

    ParticlePass* particlePass = _engine->_renderPassManager->getPass<ParticlePass>();
    if (!particlePass) return ids;

    const auto& systems = particlePass->systems();
    ids.reserve(systems.size());
    for (const auto& sys : systems)
    {
        ids.push_back(sys.id);
    }

    return ids;
}

uint32_t Engine::get_allocated_particles() const
{
    if (!_engine || !_engine->_renderPassManager) return 0;

    ParticlePass* particlePass = _engine->_renderPassManager->getPass<ParticlePass>();
    if (!particlePass) return 0;

    return particlePass->allocated_particles();
}

uint32_t Engine::get_free_particles() const
{
    if (!_engine || !_engine->_renderPassManager) return 0;

    ParticlePass* particlePass = _engine->_renderPassManager->getPass<ParticlePass>();
    if (!particlePass) return 0;

    return particlePass->free_particles();
}

uint32_t Engine::get_max_particles() const
{
    return ParticlePass::k_max_particles;
}

void Engine::preload_particle_texture(const std::string& assetPath)
{
    if (!_engine || !_engine->_renderPassManager) return;

    ParticlePass* particlePass = _engine->_renderPassManager->getPass<ParticlePass>();
    if (!particlePass) return;

    particlePass->preload_vfx_texture(assetPath);
}

// ----------------------------------------------------------------------------
// Picking / Selection
// ----------------------------------------------------------------------------

Engine::PickResult Engine::get_last_pick() const
{
    PickResult r;
    const PickingSystem *picking = _engine ? _engine->picking() : nullptr;
    if (picking)
    {
        const auto &pick = picking->last_pick();
        r.valid = pick.valid;
        r.ownerName = pick.ownerName;
        r.worldPosition = glm::vec3(pick.worldPos);
    }
    return r;
}

Engine::PickResultD Engine::get_last_pick_d() const
{
    PickResultD r;
    const PickingSystem *picking = _engine ? _engine->picking() : nullptr;
    if (picking)
    {
        const auto &pick = picking->last_pick();
        r.valid = pick.valid;
        r.ownerName = pick.ownerName;
        r.worldPosition = pick.worldPos;
    }
    return r;
}

void Engine::set_use_id_buffer_picking(bool use)
{
    if (!_engine) return;
    PickingSystem *picking = _engine->picking();
    if (!picking) return;
    picking->set_use_id_buffer_picking(use);
}

bool Engine::get_use_id_buffer_picking() const
{
    const PickingSystem *picking = _engine ? _engine->picking() : nullptr;
    if (!picking) return false;
    return picking->use_id_buffer_picking();
}

// ----------------------------------------------------------------------------
// Debug Drawing
// ----------------------------------------------------------------------------

void Engine::set_debug_draw_enabled(bool enabled)
{
    if (!_engine || !_engine->_debugDraw) return;
    _engine->_debugDraw->settings().enabled = enabled;
}

bool Engine::get_debug_draw_enabled() const
{
    if (!_engine || !_engine->_debugDraw) return false;
    return _engine->_debugDraw->settings().enabled;
}

void Engine::set_debug_layer_mask(uint32_t mask)
{
    if (!_engine || !_engine->_debugDraw) return;
    _engine->_debugDraw->settings().layer_mask = mask;
}

uint32_t Engine::get_debug_layer_mask() const
{
    if (!_engine || !_engine->_debugDraw) return 0;
    return _engine->_debugDraw->settings().layer_mask;
}

void Engine::set_debug_show_depth_tested(bool show)
{
    if (!_engine || !_engine->_debugDraw) return;
    _engine->_debugDraw->settings().show_depth_tested = show;
}

bool Engine::get_debug_show_depth_tested() const
{
    if (!_engine || !_engine->_debugDraw) return true;
    return _engine->_debugDraw->settings().show_depth_tested;
}

void Engine::set_debug_show_overlay(bool show)
{
    if (!_engine || !_engine->_debugDraw) return;
    _engine->_debugDraw->settings().show_overlay = show;
}

bool Engine::get_debug_show_overlay() const
{
    if (!_engine || !_engine->_debugDraw) return true;
    return _engine->_debugDraw->settings().show_overlay;
}

void Engine::set_debug_segments(int segments)
{
    if (!_engine || !_engine->_debugDraw) return;
    _engine->_debugDraw->settings().segments = segments;
}

int Engine::get_debug_segments() const
{
    if (!_engine || !_engine->_debugDraw) return 32;
    return _engine->_debugDraw->settings().segments;
}

void Engine::debug_draw_clear()
{
    if (!_engine || !_engine->_debugDraw) return;
    _engine->_debugDraw->clear();
}

void Engine::debug_draw_line(const glm::vec3& a, const glm::vec3& b,
                              const glm::vec4& color,
                              float duration_seconds,
                              bool depth_tested)
{
    if (!_engine || !_engine->_debugDraw) return;
    _engine->_debugDraw->add_line(WorldVec3(a), WorldVec3(b), color, duration_seconds,
                                   depth_tested ? DebugDepth::DepthTested : DebugDepth::AlwaysOnTop,
                                   DebugDrawLayer::Misc);
}

void Engine::debug_draw_line(const glm::dvec3& a, const glm::dvec3& b,
                              const glm::vec4& color,
                              float duration_seconds,
                              bool depth_tested)
{
    if (!_engine || !_engine->_debugDraw) return;
    _engine->_debugDraw->add_line(WorldVec3(a), WorldVec3(b), color, duration_seconds,
                                   depth_tested ? DebugDepth::DepthTested : DebugDepth::AlwaysOnTop,
                                   DebugDrawLayer::Misc);
}

void Engine::debug_draw_ray(const glm::vec3& origin, const glm::vec3& direction, float length,
                             const glm::vec4& color,
                             float duration_seconds,
                             bool depth_tested)
{
    if (!_engine || !_engine->_debugDraw) return;
    _engine->_debugDraw->add_ray(WorldVec3(origin), glm::dvec3(direction), length, color, duration_seconds,
                                  depth_tested ? DebugDepth::DepthTested : DebugDepth::AlwaysOnTop,
                                  DebugDrawLayer::Misc);
}

void Engine::debug_draw_ray(const glm::dvec3& origin, const glm::dvec3& direction, double length,
                             const glm::vec4& color,
                             float duration_seconds,
                             bool depth_tested)
{
    if (!_engine || !_engine->_debugDraw) return;
    _engine->_debugDraw->add_ray(WorldVec3(origin), direction, length, color, duration_seconds,
                                  depth_tested ? DebugDepth::DepthTested : DebugDepth::AlwaysOnTop,
                                  DebugDrawLayer::Misc);
}

void Engine::debug_draw_aabb(const glm::vec3& center, const glm::vec3& half_extents,
                              const glm::vec4& color,
                              float duration_seconds,
                              bool depth_tested)
{
    if (!_engine || !_engine->_debugDraw) return;
    _engine->_debugDraw->add_aabb(WorldVec3(center), half_extents, color, duration_seconds,
                                   depth_tested ? DebugDepth::DepthTested : DebugDepth::AlwaysOnTop,
                                   DebugDrawLayer::Misc);
}

void Engine::debug_draw_aabb(const glm::dvec3& center, const glm::vec3& half_extents,
                              const glm::vec4& color,
                              float duration_seconds,
                              bool depth_tested)
{
    if (!_engine || !_engine->_debugDraw) return;
    _engine->_debugDraw->add_aabb(WorldVec3(center), half_extents, color, duration_seconds,
                                   depth_tested ? DebugDepth::DepthTested : DebugDepth::AlwaysOnTop,
                                   DebugDrawLayer::Misc);
}

void Engine::debug_draw_sphere(const glm::vec3& center, float radius,
                                const glm::vec4& color,
                                float duration_seconds,
                                bool depth_tested)
{
    if (!_engine || !_engine->_debugDraw) return;
    _engine->_debugDraw->add_sphere(WorldVec3(center), radius, color, duration_seconds,
                                     depth_tested ? DebugDepth::DepthTested : DebugDepth::AlwaysOnTop,
                                     DebugDrawLayer::Misc);
}

void Engine::debug_draw_sphere(const glm::dvec3& center, float radius,
                                const glm::vec4& color,
                                float duration_seconds,
                                bool depth_tested)
{
    if (!_engine || !_engine->_debugDraw) return;
    _engine->_debugDraw->add_sphere(WorldVec3(center), radius, color, duration_seconds,
                                     depth_tested ? DebugDepth::DepthTested : DebugDepth::AlwaysOnTop,
                                     DebugDrawLayer::Misc);
}

void Engine::debug_draw_capsule(const glm::vec3& p0, const glm::vec3& p1, float radius,
                                 const glm::vec4& color,
                                 float duration_seconds,
                                 bool depth_tested)
{
    if (!_engine || !_engine->_debugDraw) return;
    _engine->_debugDraw->add_capsule(WorldVec3(p0), WorldVec3(p1), radius, color, duration_seconds,
                                      depth_tested ? DebugDepth::DepthTested : DebugDepth::AlwaysOnTop,
                                      DebugDrawLayer::Misc);
}

void Engine::debug_draw_capsule(const glm::dvec3& p0, const glm::dvec3& p1, float radius,
                                 const glm::vec4& color,
                                 float duration_seconds,
                                 bool depth_tested)
{
    if (!_engine || !_engine->_debugDraw) return;
    _engine->_debugDraw->add_capsule(WorldVec3(p0), WorldVec3(p1), radius, color, duration_seconds,
                                      depth_tested ? DebugDepth::DepthTested : DebugDepth::AlwaysOnTop,
                                      DebugDrawLayer::Misc);
}

void Engine::debug_draw_circle(const glm::vec3& center, const glm::vec3& normal, float radius,
                                const glm::vec4& color,
                                float duration_seconds,
                                bool depth_tested)
{
    if (!_engine || !_engine->_debugDraw) return;
    _engine->_debugDraw->add_circle(WorldVec3(center), glm::dvec3(normal), radius, color, duration_seconds,
                                     depth_tested ? DebugDepth::DepthTested : DebugDepth::AlwaysOnTop,
                                     DebugDrawLayer::Misc);
}

void Engine::debug_draw_circle(const glm::dvec3& center, const glm::dvec3& normal, float radius,
                                const glm::vec4& color,
                                float duration_seconds,
                                bool depth_tested)
{
    if (!_engine || !_engine->_debugDraw) return;
    _engine->_debugDraw->add_circle(WorldVec3(center), normal, radius, color, duration_seconds,
                                     depth_tested ? DebugDepth::DepthTested : DebugDepth::AlwaysOnTop,
                                     DebugDrawLayer::Misc);
}

void Engine::debug_draw_cone(const glm::vec3& apex, const glm::vec3& direction,
                              float length, float angle_degrees,
                              const glm::vec4& color,
                              float duration_seconds,
                              bool depth_tested)
{
    if (!_engine || !_engine->_debugDraw) return;
    _engine->_debugDraw->add_cone(WorldVec3(apex), glm::dvec3(direction), length, angle_degrees,
                                   color, duration_seconds,
                                   depth_tested ? DebugDepth::DepthTested : DebugDepth::AlwaysOnTop,
                                   DebugDrawLayer::Misc);
}

void Engine::debug_draw_cone(const glm::dvec3& apex, const glm::dvec3& direction,
                              float length, float angle_degrees,
                              const glm::vec4& color,
                              float duration_seconds,
                              bool depth_tested)
{
    if (!_engine || !_engine->_debugDraw) return;
    _engine->_debugDraw->add_cone(WorldVec3(apex), direction, length, angle_degrees,
                                   color, duration_seconds,
                                   depth_tested ? DebugDepth::DepthTested : DebugDepth::AlwaysOnTop,
                                   DebugDrawLayer::Misc);
}

} // namespace GameAPI
