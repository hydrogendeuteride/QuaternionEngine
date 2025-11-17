#pragma once
#include <core/vk_types.h>
#include <scene/camera.h>
#include <unordered_map>
#include <memory>

#include "scene/vk_loader.h"
class EngineContext;

struct RenderObject
{
    uint32_t indexCount;
    uint32_t firstIndex;
    VkBuffer indexBuffer;
    VkBuffer vertexBuffer; // for RG buffer tracking (device-address path still used in shader)

    MaterialInstance *material;
    Bounds bounds;

    glm::mat4 transform;
    VkDeviceAddress vertexBufferAddress;
};

struct DrawContext
{
    std::vector<RenderObject> OpaqueSurfaces;
    std::vector<RenderObject> TransparentSurfaces;
};

class SceneManager
{
public:
    void init(EngineContext *context);

    void cleanup();

    void update_scene();

    Camera &getMainCamera() { return mainCamera; }
    const GPUSceneData &getSceneData() const { return sceneData; }
    DrawContext &getMainDrawContext() { return mainDrawContext; }

    void loadScene(const std::string &name, std::shared_ptr<LoadedGLTF> scene);

    std::shared_ptr<LoadedGLTF> getScene(const std::string &name);

    // Dynamic renderables API
    struct MeshInstance
    {
        std::shared_ptr<MeshAsset> mesh;
        glm::mat4 transform{1.f};
    };

    void addMeshInstance(const std::string &name, std::shared_ptr<MeshAsset> mesh,
                         const glm::mat4 &transform = glm::mat4(1.f));
    bool removeMeshInstance(const std::string &name);
    void clearMeshInstances();

    // GLTF instances (runtime-spawned scenes with transforms)
    struct GLTFInstance
    {
        std::shared_ptr<LoadedGLTF> scene;
        glm::mat4 transform{1.f};
    };

    void addGLTFInstance(const std::string &name, std::shared_ptr<LoadedGLTF> scene,
                         const glm::mat4 &transform = glm::mat4(1.f));
    bool removeGLTFInstance(const std::string &name);
    bool setGLTFInstanceTransform(const std::string &name, const glm::mat4 &transform);
    void clearGLTFInstances();

    // Animation control helpers (glTF)
    // Note: a LoadedGLTF may be shared by multiple instances; changing
    // the active animation on a scene or instance affects all users
    // of that shared LoadedGLTF.
    bool setSceneAnimation(const std::string &sceneName, int animationIndex, bool resetTime = true);
    bool setSceneAnimation(const std::string &sceneName, const std::string &animationName, bool resetTime = true);
    bool setSceneAnimationLoop(const std::string &sceneName, bool loop);

    bool setGLTFInstanceAnimation(const std::string &instanceName, int animationIndex, bool resetTime = true);
    bool setGLTFInstanceAnimation(const std::string &instanceName, const std::string &animationName, bool resetTime = true);
    bool setGLTFInstanceAnimationLoop(const std::string &instanceName, bool loop);

    struct SceneStats
    {
        float scene_update_time = 0.f;
    } stats;

private:
    EngineContext *_context = nullptr;

    Camera mainCamera = {};
    GPUSceneData sceneData = {};
    DrawContext mainDrawContext;

    std::unordered_map<std::string, std::shared_ptr<LoadedGLTF> > loadedScenes;
    std::unordered_map<std::string, std::shared_ptr<Node> > loadedNodes;
    std::unordered_map<std::string, MeshInstance> dynamicMeshInstances;
    std::unordered_map<std::string, GLTFInstance> dynamicGLTFInstances;
};
