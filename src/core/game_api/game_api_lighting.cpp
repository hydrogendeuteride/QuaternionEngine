#include "core/game_api.h"
#include "core/engine.h"
#include "core/context.h"
#include "scene/vk_scene.h"
#include "core/assets/ibl_manager.h"

namespace GameAPI
{

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
// IBL (Image-Based Lighting) - Helper functions
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

static VulkanEngine::IBLVolumeShape to_internal_ibl_volume_shape(IBLVolumeShape shape)
{
    switch (shape)
    {
    case IBLVolumeShape::Sphere:
        return VulkanEngine::IBLVolumeShape::Sphere;
    case IBLVolumeShape::Box:
    default:
        return VulkanEngine::IBLVolumeShape::Box;
    }
}

static IBLVolumeShape from_internal_ibl_volume_shape(VulkanEngine::IBLVolumeShape shape)
{
    switch (shape)
    {
    case VulkanEngine::IBLVolumeShape::Sphere:
        return IBLVolumeShape::Sphere;
    case VulkanEngine::IBLVolumeShape::Box:
    default:
        return IBLVolumeShape::Box;
    }
}

// ----------------------------------------------------------------------------
// IBL (Image-Based Lighting)
// ----------------------------------------------------------------------------

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
    v.shape = to_internal_ibl_volume_shape(volume.shape);
    v.radius = volume.radius;

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
    v.shape = to_internal_ibl_volume_shape(volume.shape);
    v.radius = volume.radius;

    _engine->_iblVolumes.push_back(v);
    return _engine->_iblVolumes.size() - 1;
}

size_t Engine::add_ibl_sphere_volume(const glm::vec3& center, float radius, const IBLPaths& paths, bool enabled)
{
    IBLVolume v{};
    v.center = center;
    v.paths = paths;
    v.enabled = enabled;
    v.shape = IBLVolumeShape::Sphere;
    v.radius = radius;
    return add_ibl_volume(v);
}

size_t Engine::add_ibl_sphere_volume(const glm::dvec3& center, float radius, const IBLPaths& paths, bool enabled)
{
    IBLVolumeD v{};
    v.center = center;
    v.paths = paths;
    v.enabled = enabled;
    v.shape = IBLVolumeShape::Sphere;
    v.radius = radius;
    return add_ibl_volume(v);
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
    out.shape = from_internal_ibl_volume_shape(v.shape);
    out.radius = v.radius;
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
    out.shape = from_internal_ibl_volume_shape(v.shape);
    out.radius = v.radius;
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
    v.shape = to_internal_ibl_volume_shape(volume.shape);
    v.radius = volume.radius;
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
    v.shape = to_internal_ibl_volume_shape(volume.shape);
    v.radius = volume.radius;
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

} // namespace GameAPI
