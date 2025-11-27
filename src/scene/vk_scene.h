#pragma once
#include <core/types.h>
#include <scene/camera.h>
#include <unordered_map>
#include <memory>
#include <optional>
#include <glm/vec2.hpp>
#include <string>

#include "scene/vk_loader.h"
class EngineContext;

struct RenderObject
{
    // Geometry and material binding
    uint32_t indexCount;
    uint32_t firstIndex;
    VkBuffer indexBuffer;
    VkBuffer vertexBuffer; // for RG buffer tracking (device-address path still used in shader)

    MaterialInstance *material;
    Bounds bounds;

    glm::mat4 transform;
    VkDeviceAddress vertexBufferAddress;

    // Optional debug/source information (may be null/unused for some objects).
    MeshAsset *sourceMesh = nullptr;
    uint32_t surfaceIndex = 0;
    // Unique per-draw identifier for ID-buffer picking (0 = none).
    uint32_t objectID = 0;
    // Optional logical owner for editor/picking (instance name etc.).
    enum class OwnerType : uint8_t
    {
        None = 0,
        StaticGLTF,     // loaded scene
        GLTFInstance,   // runtime glTF instance with transform
        MeshInstance    // dynamic primitive/mesh instance
    };
    OwnerType ownerType = OwnerType::None;
    std::string ownerName;
    // Optional owning glTF scene and node for this draw (null for procedural/dynamic meshes).
    LoadedGLTF *sourceScene = nullptr;
    Node *sourceNode = nullptr;
};

struct DrawContext
{
    std::vector<RenderObject> OpaqueSurfaces;
    std::vector<RenderObject> TransparentSurfaces;
    // Monotonic counter used to assign stable per-frame object IDs.
    uint32_t nextID = 1;
};

class SceneManager
{
public:
    ~SceneManager();
    void init(EngineContext *context);

    void cleanup();

    void update_scene();

    Camera &getMainCamera() { return mainCamera; }

    // Ray-pick against current DrawContext using per-surface Bounds.
    // mousePosPixels is in window coordinates (SDL), origin at top-left.
    // Returns true if any object was hit, filling outObject and outWorldPos.
    bool pick(const glm::vec2 &mousePosPixels, RenderObject &outObject, glm::vec3 &outWorldPos);

    // Resolve an object ID (from ID buffer) back to the RenderObject for
    // the most recently built DrawContext. Returns false if not found or id==0.
    bool resolveObjectID(uint32_t id, RenderObject &outObject) const;

    // Select all objects whose projected bounds intersect the given screen-space
    // rectangle (window coordinates, origin top-left). Results are appended to outObjects.
    void selectRect(const glm::vec2 &p0, const glm::vec2 &p1, std::vector<RenderObject> &outObjects) const;

    const GPUSceneData &getSceneData() const { return sceneData; }
    DrawContext &getMainDrawContext() { return mainDrawContext; }

    void loadScene(const std::string &name, std::shared_ptr<LoadedGLTF> scene);

    std::shared_ptr<LoadedGLTF> getScene(const std::string &name);

    // Dynamic renderables API
    struct MeshInstance
    {
        std::shared_ptr<MeshAsset> mesh;
        glm::mat4 transform{1.f};
        std::optional<BoundsType> boundsTypeOverride;
    };

    void addMeshInstance(const std::string &name, std::shared_ptr<MeshAsset> mesh,
                         const glm::mat4 &transform = glm::mat4(1.f),
                         std::optional<BoundsType> boundsType = {});
    bool getMeshInstanceTransform(const std::string &name, glm::mat4 &outTransform);
    bool setMeshInstanceTransform(const std::string &name, const glm::mat4 &transform);
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
    bool getGLTFInstanceTransform(const std::string &name, glm::mat4 &outTransform);
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

    struct PointLight
    {
        glm::vec3 position;
        float radius;
        glm::vec3 color;
        float intensity;
    };

    void addPointLight(const PointLight &light);
    void clearPointLights();
    const std::vector<PointLight> &getPointLights() const { return pointLights; }

    struct SceneStats
    {
        float scene_update_time = 0.f;
    } stats;

    struct PickingDebug
    {
        bool usedMeshBVH = false;
        bool meshBVHHit = false;
        bool meshBVHFallbackBox = false;
        uint32_t meshBVHPrimCount = 0;
        uint32_t meshBVHNodeCount = 0;
    };

    const PickingDebug &getPickingDebug() const { return pickingDebug; }

private:
    EngineContext *_context = nullptr;

    Camera mainCamera = {};
    GPUSceneData sceneData = {};
    DrawContext mainDrawContext;
    std::vector<PointLight> pointLights;

    std::unordered_map<std::string, std::shared_ptr<LoadedGLTF> > loadedScenes;
    std::unordered_map<std::string, std::shared_ptr<Node> > loadedNodes;
    std::unordered_map<std::string, MeshInstance> dynamicMeshInstances;
    std::unordered_map<std::string, GLTFInstance> dynamicGLTFInstances;
    // Keep GLTF assets alive until after the next frame fence to avoid destroying
    // GPU resources that might still be in-flight.
    std::vector<std::shared_ptr<LoadedGLTF>> pendingGLTFRelease;

    PickingDebug pickingDebug{};
};
