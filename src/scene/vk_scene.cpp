#include "vk_scene.h"

#include "core/context.h"
#include "core/util/logger.h"
#include "scene/planet/planet_system.h"

#include "glm/gtx/norm.inl"
#include <cmath>

SceneManager::SceneManager() = default;

SceneManager::~SceneManager()
{
    Logger::info("[SceneManager] dtor: dynamicGLTFInstances={} pendingGLTFRelease={}",
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
    sceneData.sunlightDirection = glm::vec4(glm::normalize(glm::vec3(-0.2f, -1.0f, -0.3f)), 1.0f);
    sceneData.sunlightColor = glm::vec4(1.0f, 1.0f, 1.0f, 3.0f);

    cameraRig.init(*this, mainCamera);

    _camera_position_local = world_to_local(mainCamera.position_world, get_world_origin());

    _planetSystem = std::make_unique<PlanetSystem>();
    _planetSystem->init(_context);
}

WorldVec3 SceneManager::get_world_origin() const
{
    return _context ? _context->origin_world : WorldVec3{0.0, 0.0, 0.0};
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
    clearDecals();
    clearGLTFInstances();

    // On engine shutdown we know VulkanEngine::cleanup() has already called
    // vkDeviceWaitIdle(), so it is safe to destroy all remaining GLTF scenes
    // immediately instead of deferring them through pendingGLTFRelease.
    if (!pendingGLTFRelease.empty())
    {
        Logger::info("[SceneManager] cleanup: forcing {} pending GLTF releases before shutdown",
                     pendingGLTFRelease.size());
        pendingGLTFRelease.clear(); // drop strong refs → ~LoadedGLTF::clearAll() runs
    }
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
