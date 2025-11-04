#include "vk_scene.h"

#include <utility>

#include "vk_swapchain.h"
#include "core/engine_context.h"
#include "core/config.h"
#include "glm/gtx/transform.hpp"
#include <glm/gtc/matrix_transform.hpp>

#include "glm/gtx/norm.inl"
#include "glm/gtx/compatibility.hpp"
#include <algorithm>
#include "core/config.h"

void SceneManager::init(EngineContext *context)
{
    _context = context;

    mainCamera.velocity = glm::vec3(0.f);
    mainCamera.position = glm::vec3(30.f, -00.f, 85.f);
    mainCamera.pitch = 0;
    mainCamera.yaw = 0;

    sceneData.ambientColor = glm::vec4(0.1f, 0.1f, 0.1f, 1.0f);
    sceneData.sunlightDirection = glm::vec4(-1.0f, -1.0f, -0.1f, 1.0f);
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

void SceneManager::clearGLTFInstances()
{
    dynamicGLTFInstances.clear();
}
