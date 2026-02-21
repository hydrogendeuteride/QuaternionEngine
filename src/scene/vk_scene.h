#pragma once
#include <core/types.h>
#include <core/world.h>
#include <scene/camera.h>
#include <scene/camera/camera_rig.h>
#include <unordered_map>
#include <memory>
#include <optional>
#include <chrono>
#include <glm/vec2.hpp>
#include <string>

#include "scene/vk_loader.h"
#include "physics/physics_body.h"

class EngineContext;
class PlanetSystem;

namespace Physics { class PhysicsWorld; }

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
        GLTFInstance,   // runtime glTF instance with transform
        MeshInstance    // dynamic primitive/mesh instance
    };
    OwnerType ownerType = OwnerType::None;
    std::string ownerName;
    // Optional owning glTF scene and node for this draw (null for procedural/dynamic meshes).
    LoadedGLTF *sourceScene = nullptr;
    Node *sourceNode = nullptr;
};

enum class DecalShape : uint8_t
{
    Box = 0,
    Sphere = 1
};

struct DecalDraw
{
    DecalShape shape = DecalShape::Box;
    glm::vec3 center_local{0.0f, 0.0f, 0.0f};
    glm::quat rotation{1.0f, 0.0f, 0.0f, 0.0f};
    glm::vec3 half_extents{0.5f, 0.5f, 0.5f};
    uint32_t albedoHandle = 0xFFFFFFFFu;
    uint32_t normalHandle = 0xFFFFFFFFu;
    glm::vec3 tint{1.0f, 1.0f, 1.0f};
    float opacity = 1.0f;
    float normalStrength = 1.0f;
};

struct DrawContext
{
    std::vector<RenderObject> OpaqueSurfaces;
    std::vector<RenderObject> TransparentSurfaces;
    std::vector<RenderObject> MeshVfxSurfaces;
    std::vector<DecalDraw> Decals;
    // Monotonic counter used to assign stable per-frame object IDs.
    uint32_t nextID = 1;
    // Optional per-instance glTF node local overrides (additive layer in local space).
    // When non-null, MeshNode::Draw will rebuild world transforms using these offsets.
    const std::unordered_map<const Node*, glm::mat4> *gltfNodeLocalOverrides = nullptr;
};

class SceneManager
{
public:
    static constexpr size_t kMaxDecals = 128;

    SceneManager();
    ~SceneManager();
    void init(EngineContext *context);

    void cleanup();

    void update_scene();

    Camera &getMainCamera() { return mainCamera; }
    const Camera &getMainCamera() const { return mainCamera; }
    CameraRig &getCameraRig() { return cameraRig; }
    const CameraRig &getCameraRig() const { return cameraRig; }

    WorldVec3 get_world_origin() const;
    glm::vec3 get_camera_local_position() const { return _camera_position_local; }

    // Ray-pick against current DrawContext using per-surface Bounds.
    // mousePosPixels is in window coordinates (SDL), origin at top-left.
    // Returns true if any object was hit, filling outObject and outWorldPos.
    bool pick(const glm::vec2 &mousePosPixels, RenderObject &outObject, WorldVec3 &outWorldPos);

    // Resolve an object ID (from ID buffer) back to the RenderObject for
    // the most recently built DrawContext. Returns false if not found or id==0.
    bool resolveObjectID(uint32_t id, RenderObject &outObject) const;

    // Select all objects whose projected bounds intersect the given screen-space
    // rectangle (window coordinates, origin top-left). Results are appended to outObjects.
    void selectRect(const glm::vec2 &p0, const glm::vec2 &p1, std::vector<RenderObject> &outObjects) const;

    const GPUSceneData &getSceneData() const { return sceneData; }

    // Sunlight (directional light) access
    void setSunlightDirection(const glm::vec3& dir);
    glm::vec3 getSunlightDirection() const;
    void setSunlightColor(const glm::vec3& color, float intensity);
    glm::vec3 getSunlightColor() const;
    float getSunlightIntensity() const;

    // Delta time (seconds) for the current frame
    float getDeltaTime() const { return _deltaTime; }
    DrawContext &getMainDrawContext() { return mainDrawContext; }

    // Dynamic renderables API
    struct MeshInstance
    {
        std::shared_ptr<MeshAsset> mesh;
        WorldVec3 translation_world{0.0, 0.0, 0.0};
        glm::quat rotation{1.0f, 0.0f, 0.0f, 0.0f};
        glm::vec3 scale{1.0f, 1.0f, 1.0f};
        std::optional<BoundsType> boundsTypeOverride;
    };

    void addMeshInstance(const std::string &name, std::shared_ptr<MeshAsset> mesh,
                         const glm::mat4 &transform = glm::mat4(1.f),
                         std::optional<BoundsType> boundsType = {});
    bool getMeshInstanceTransform(const std::string &name, glm::mat4 &outTransform);
    bool setMeshInstanceTransform(const std::string &name, const glm::mat4 &transform);
    bool getMeshInstanceTransformLocal(const std::string &name, glm::mat4 &outTransformLocal) const;
    bool setMeshInstanceTransformLocal(const std::string &name, const glm::mat4 &transformLocal);
    bool getMeshInstanceTRSWorld(const std::string &name, WorldVec3 &outTranslationWorld, glm::quat &outRotation, glm::vec3 &outScale) const;
    bool setMeshInstanceTRSWorld(const std::string &name, const WorldVec3 &translationWorld, const glm::quat &rotation, const glm::vec3 &scale);
    bool setMeshInstanceMaterial(const std::string &name, std::shared_ptr<GLTFMaterial> material);
    bool removeMeshInstance(const std::string &name);
    void clearMeshInstances();

    struct DecalInstance
    {
        DecalShape shape = DecalShape::Box;
        WorldVec3 center_world{0.0, 0.0, 0.0};
        glm::quat rotation{1.0f, 0.0f, 0.0f, 0.0f};
        glm::vec3 half_extents{0.5f, 0.5f, 0.5f};
        uint32_t albedoHandle = 0xFFFFFFFFu;
        uint32_t normalHandle = 0xFFFFFFFFu;
        glm::vec3 tint{1.0f, 1.0f, 1.0f};
        float opacity = 1.0f;
        float normalStrength = 1.0f;
        int32_t sort_order = 0;
    };

    bool setDecal(const std::string &name, const DecalInstance &decal);
    bool getDecal(const std::string &name, DecalInstance &outDecal) const;
    bool removeDecal(const std::string &name);
    void clearDecals();
    size_t getDecalCount() const { return dynamicDecals.size(); }

    // GLTF instances (runtime-spawned scenes with transforms)
    struct GLTFInstance
    {
        std::shared_ptr<LoadedGLTF> scene;
        WorldVec3 translation_world{0.0, 0.0, 0.0};
        glm::quat rotation{1.0f, 0.0f, 0.0f, 0.0f};
        glm::vec3 scale{1.0f, 1.0f, 1.0f};
        LoadedGLTF::AnimationState animation;
        // Per-instance local-space pose offsets for nodes in this glTF scene.
        // The offset matrix is post-multiplied onto the node's localTransform.
        std::unordered_map<const Node*, glm::mat4> nodeLocalOverrides;
    };

    void addGLTFInstance(const std::string &name, std::shared_ptr<LoadedGLTF> scene,
                         const glm::mat4 &transform = glm::mat4(1.f));
    bool removeGLTFInstance(const std::string &name);
    bool getGLTFInstanceTransform(const std::string &name, glm::mat4 &outTransform);
    bool setGLTFInstanceTransform(const std::string &name, const glm::mat4 &transform);
    bool getGLTFInstanceTransformLocal(const std::string &name, glm::mat4 &outTransformLocal) const;
    bool setGLTFInstanceTransformLocal(const std::string &name, const glm::mat4 &transformLocal);
    bool getGLTFInstanceTRSWorld(const std::string &name, WorldVec3 &outTranslationWorld, glm::quat &outRotation, glm::vec3 &outScale) const;
    bool setGLTFInstanceTRSWorld(const std::string &name, const WorldVec3 &translationWorld, const glm::quat &rotation, const glm::vec3 &scale);
    void clearGLTFInstances();
    // Per-instance glTF node pose overrides (local-space, layered on top of animation/base TRS).
    // 'offset' is post-multiplied onto the node's localTransform for this instance only.
    bool setGLTFInstanceNodeOffset(const std::string &instanceName,
                                   const std::string &nodeName,
                                   const glm::mat4 &offset);
    bool clearGLTFInstanceNodeOffset(const std::string &instanceName,
                                     const std::string &nodeName);
    void clearGLTFInstanceNodeOffsets(const std::string &instanceName);

    // Returns a glTF node's world transform
    // - World: uses the instance's stored world-space translation (float in the returned matrix).
    // - Local: uses the render-local (floating-origin shifted) translation for better precision near the camera.
    bool getGLTFInstanceNodeWorldTransform(const std::string &instanceName,
                                          const std::string &nodeName,
                                          glm::mat4 &outWorldTransform) const;
    glm::mat4 getGLTFInstanceNodeWorldTransform(const std::string &instanceName,
                                                const std::string &nodeName) const;

    bool getGLTFInstanceNodeWorldTransformLocal(const std::string &instanceName,
                                               const std::string &nodeName,
                                               glm::mat4 &outWorldTransformLocal) const;
    bool getGLTFInstanceNodeWorldTransformLocal(const std::string &instanceName,
                                               const std::string &nodeName,
                                               const WorldVec3 &origin_world,
                                               glm::mat4 &outWorldTransformLocal) const;
    glm::mat4 getGLTFInstanceNodeWorldTransformLocal(const std::string &instanceName,
                                                     const std::string &nodeName) const;
    glm::mat4 getGLTFInstanceNodeWorldTransformLocal(const std::string &instanceName,
                                                     const std::string &nodeName,
                                                     const WorldVec3 &origin_world) const;

    // Animation control helpers (glTF)
    // Note: a LoadedGLTF may be shared by multiple instances; changing
    // the active animation on a scene or instance affects all users
    // of that shared LoadedGLTF.
    bool setGLTFInstanceAnimation(const std::string &instanceName, int animationIndex, bool resetTime = true);
    bool setGLTFInstanceAnimation(const std::string &instanceName, const std::string &animationName, bool resetTime = true);
    bool setGLTFInstanceAnimationLoop(const std::string &instanceName, bool loop);
    bool setGLTFInstanceAnimationSpeed(const std::string &instanceName, float speed);
    bool transitionGLTFInstanceAnimation(const std::string &instanceName, int animationIndex, float blendDurationSeconds, bool resetTime = true);
    bool transitionGLTFInstanceAnimation(const std::string &instanceName, const std::string &animationName, float blendDurationSeconds, bool resetTime = true);

    struct PointLight
    {
        WorldVec3 position_world;
        float radius;
        glm::vec3 color;
        float intensity;
        bool cast_shadows = true;
    };

    void addPointLight(const PointLight &light);
    void clearPointLights();
    size_t getPointLightCount() const { return pointLights.size(); }
    bool getPointLight(size_t index, PointLight &outLight) const;
    bool setPointLight(size_t index, const PointLight &light);
    bool removePointLight(size_t index);
    const std::vector<PointLight> &getPointLights() const { return pointLights; }

    struct SpotLight
    {
        WorldVec3 position_world;
        glm::vec3 direction{0.0f, -1.0f, 0.0f}; // world-space unit vector
        float radius = 10.0f;
        glm::vec3 color{1.0f, 1.0f, 1.0f};
        float intensity = 1.0f;
        // Cone half-angles in degrees (inner <= outer).
        float inner_angle_deg = 15.0f;
        float outer_angle_deg = 25.0f;
        bool cast_shadows = true;
    };

    void addSpotLight(const SpotLight &light);
    void clearSpotLights();
    size_t getSpotLightCount() const { return spotLights.size(); }
    bool getSpotLight(size_t index, SpotLight &outLight) const;
    bool setSpotLight(size_t index, const SpotLight &light);
    bool removeSpotLight(size_t index);
    const std::vector<SpotLight> &getSpotLights() const { return spotLights; }

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

    PlanetSystem *get_planet_system() const { return _planetSystem.get(); }

    // Returns the LoadedGLTF scene for a named GLTF instance, or nullptr if not found.
    std::shared_ptr<LoadedGLTF> getGLTFInstanceScene(const std::string &instanceName) const;

    // =========================================================================
    // Physics collider synchronization
    // =========================================================================
    // Enables automatic physics body creation and transform sync for a glTF instance.
    // Creates one kinematic body per collider-owning node, updated each frame to match
    // the node's world transform (including animation). Returns number of bodies created.
    //
    // Parameters:
    //   instanceName  - Name of the glTF instance (must exist in dynamicGLTFInstances)
    //   world         - Physics world to create bodies in
    //   layer         - Collision layer for created bodies
    //   user_data     - Optional user data attached to all created bodies
    //
    // The instance must have collider_compounds defined (from COL_* markers or sidecar).
    // Mesh colliders (COL_MESH) from sidecars are also supported and will be created as static bodies.
    // If already enabled, this is a no-op and returns 0.
    size_t enableColliderSync(const std::string &instanceName,
                              Physics::PhysicsWorld *world,
                              uint32_t layer = 0,
                              uint64_t user_data = 0);

    // Disables collider sync for an instance, destroying all associated physics bodies.
    // Returns true if the instance had collider sync enabled.
    bool disableColliderSync(const std::string &instanceName);

    // Returns true if collider sync is enabled for the given instance.
    bool isColliderSyncEnabled(const std::string &instanceName) const;

    // Manually trigger collider sync update (called automatically in update_scene).
    void syncColliders();

    // Debug: get body IDs for a synced instance (empty if not synced).
    std::vector<Physics::BodyId> getColliderSyncBodies(const std::string &instanceName) const;

private:
    EngineContext *_context = nullptr;

    Camera mainCamera = {};
    CameraRig cameraRig{};
    GPUSceneData sceneData = {};
    DrawContext mainDrawContext;
    std::vector<PointLight> pointLights;
    std::vector<SpotLight> spotLights;
    glm::vec3 _camera_position_local{0.0f, 0.0f, 0.0f};
    double _floating_origin_recenter_threshold = 1000.0;
    double _floating_origin_snap_size = 100.0;
    float _deltaTime = 0.0f;
    float _elapsedTime = 0.0f;
    std::chrono::steady_clock::time_point _lastFrameTime{};

    std::unordered_map<std::string, MeshInstance> dynamicMeshInstances;
    std::unordered_map<std::string, DecalInstance> dynamicDecals;
    std::unordered_map<std::string, GLTFInstance> dynamicGLTFInstances;
    // Keep GLTF assets alive until after the next frame fence to avoid destroying
    // GPU resources that might still be in-flight.
    std::vector<std::shared_ptr<LoadedGLTF>> pendingGLTFRelease;

    std::unique_ptr<PlanetSystem> _planetSystem;

    PickingDebug pickingDebug{};

    // Reusable scratch vectors for punctual shadow sorting (avoid per-frame allocation)
    std::vector<uint32_t> _pointShadowOrder;
    std::vector<uint32_t> _spotShadowOrder;
    std::vector<uint32_t> _shadowCandidates;
    std::vector<uint8_t> _shadowSelected;

    // Collider sync state per glTF instance
    struct ColliderSyncEntry
    {
        Physics::PhysicsWorld *world{nullptr};
        // node name → physics body ID
        std::unordered_map<std::string, Physics::BodyId> node_bodies;
        // Static mesh collider bodies (do not get per-frame transform sync).
        std::vector<Physics::BodyId> mesh_bodies;
        uint32_t layer{0};
        uint64_t user_data{0};
    };
    std::unordered_map<std::string, ColliderSyncEntry> _colliderSyncEntries;

    void destroyColliderSyncEntry(ColliderSyncEntry &entry);
};
