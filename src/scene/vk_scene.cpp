#include "vk_scene.h"

#include <utility>

#include "vk_swapchain.h"
#include "core/engine_context.h"
#include "core/config.h"
#include "glm/gtx/transform.hpp"
#include <glm/gtc/matrix_transform.hpp>

#include "glm/gtx/norm.inl"

void SceneManager::init(EngineContext *context)
{
    _context = context;

    mainCamera.velocity = glm::vec3(0.f);
    mainCamera.position = glm::vec3(30.f, -00.f, 85.f);
    mainCamera.pitch = 0;
    mainCamera.yaw = 0;

    sceneData.ambientColor = glm::vec4(0.1f, 0.1f, 0.1f, 1.0f);
    sceneData.sunlightDirection = glm::vec4(-1.0f, -1.0f, -1.0f, 1.0f);
    sceneData.sunlightColor = glm::vec4(1.0f, 1.0f, 1.0f, 3.0f);
}

void SceneManager::update_scene()
{
    auto start = std::chrono::system_clock::now();

    mainDrawContext.OpaqueSurfaces.clear();
    mainDrawContext.TransparentSurfaces.clear();

    mainCamera.update();

    if (loadedScenes.find("structure") != loadedScenes.end())
    {
        loadedScenes["structure"]->Draw(glm::mat4{1.f}, mainDrawContext);
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

    {
        const glm::vec3 camPos = glm::vec3(glm::inverse(view)[3]);
        glm::vec3 L = glm::normalize(-glm::vec3(sceneData.sunlightDirection));
        if (glm::length(L) < 1e-5f) L = glm::vec3(0.0f, -1.0f, 0.0f);

        const glm::vec3 worldUp(0.0f, 1.0f, 0.0f);
        glm::vec3 right = glm::normalize(glm::cross(worldUp, L));
        glm::vec3 up = glm::normalize(glm::cross(L, right));
        if (glm::length2(right) < 1e-6f)
        {
            right = glm::vec3(1, 0, 0);
            up = glm::normalize(glm::cross(L, right));
        }

        // Simple 2-stage CSM using existing shadow setup (no frustum box fitting)
        const float baseOrtho = tan(fov);
        const float lightNear = kShadowLightNear;

        // Configure split distances in view space from central config
        const float csmFar = kShadowCSMFar;
        const float split0 = kShadowSplitRatio * csmFar;
        sceneData.cascadeSplitsView = glm::vec4(split0, csmFar, csmFar, csmFar);

        // Scale XY coverage with aspect so wide screens don't clip (from central config)
        const float orthoY0 = baseOrtho * kShadowCascade0XYScale;
        const float orthoX0 = orthoY0 * aspect;
        const float orthoY1 = baseOrtho * kShadowCascade1XYScale;
        const float orthoX1 = orthoY1 * aspect;

        // Cascade 0 (near): larger XY footprint, push light back and extend far plane
        {
            const float lightFar0 = kShadowCascade0Far;
            const glm::vec3 lightPos0 = camPos - L * kShadowCascade0LightOffset;
            const glm::mat4 viewLight0 = glm::lookAtRH(lightPos0, camPos, up);
            const glm::mat4 projLight0 = glm::orthoRH_ZO(-orthoX0, orthoX0,
                                                         -orthoY0, orthoY0,
                                                         lightNear, lightFar0);
            sceneData.lightViewProjCascades[0] = projLight0 * viewLight0;
            sceneData.lightViewProj = sceneData.lightViewProjCascades[0];
        }

        // Cascade 1 (far): much larger XY footprint and longer depth range
        {
            const float lightFar1 = kShadowCascade1Far;     // much farther reach for distant shadows
            const glm::vec3 lightPos1 = camPos - L * kShadowCascade1LightOffset;
            const glm::mat4 viewLight1 = glm::lookAtRH(lightPos1, camPos, up);
            const glm::mat4 projLight1 = glm::orthoRH_ZO(-orthoX1, orthoX1,
                                                         -orthoY1, orthoY1,
                                                         lightNear, lightFar1);
            sceneData.lightViewProjCascades[1] = projLight1 * viewLight1;
        }

        // Fill remaining slots with the far cascade to keep data valid
        for (int c = 2; c < 4; ++c)
        {
            sceneData.lightViewProjCascades[c] = sceneData.lightViewProjCascades[1];
        }
    }


    auto end = std::chrono::system_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    stats.scene_update_time = elapsed.count() / 1000.f;
}

void SceneManager::loadScene(const std::string &name, std::shared_ptr<LoadedGLTF> scene)
{
    loadedScenes[name] = std::move(scene);
}

std::shared_ptr<LoadedGLTF> SceneManager::getScene(const std::string &name)
{
    auto it = loadedScenes.find(name);
    return (it != loadedScenes.end()) ? it->second : nullptr;
}

void SceneManager::cleanup()
{
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

void SceneManager::clearGLTFInstances()
{
    dynamicGLTFInstances.clear();
}
