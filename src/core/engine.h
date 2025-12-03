// vulkan_engine.h : Include file for standard system include files,
// or project specific include files.

#pragma once

#include <core/types.h>
#include <vector>
#include <string>
#include <unordered_map>
#include "vk_mem_alloc.h"
#include <deque>
#include <functional>
#include "descriptor/descriptors.h"
#include "scene/vk_loader.h"
#include "compute/vk_compute.h"
#include <scene/camera.h>

#include "device/device.h"
#include "render/renderpass.h"
#include "render/passes/background.h"
#include "device/resource.h"
#include "device/swapchain.h"
#include "scene/vk_scene.h"
#include "render/materials.h"

#include "frame/resources.h"
#include "descriptor/manager.h"
#include "pipeline/sampler.h"
#include "render/passes/ssr.h"
#include "core/context.h"
#include "core/pipeline/manager.h"
#include "core/assets/manager.h"
#include "core/assets/async_loader.h"
#include "render/graph/graph.h"
#include "core/raytracing/raytracing.h"
#include "core/assets/texture_cache.h"
#include "core/assets/ibl_manager.h"

// Number of frames-in-flight. Affects per-frame command buffers, fences,
// semaphores, and transient descriptor pools in FrameResources.
constexpr unsigned int FRAME_OVERLAP = 2;

// Compute push constants and effects are declared in compute/vk_compute.h now.


struct RenderPass
{
	std::string name;
	std::function<void(VkCommandBuffer)> execute;
};

struct MeshNode : public Node
{
    std::shared_ptr<MeshAsset> mesh;
    // Owning glTF scene (for picking/debug); may be null for non-gltf meshes.
    LoadedGLTF *scene = nullptr;

    virtual void Draw(const glm::mat4 &topMatrix, DrawContext &ctx) override;
};

class VulkanEngine
{
public:
	bool _isInitialized{false};
	int _frameNumber{0};

    std::shared_ptr<DeviceManager> _deviceManager;
    std::unique_ptr<SwapchainManager> _swapchainManager;
    std::shared_ptr<ResourceManager> _resourceManager;
    std::unique_ptr<RenderPassManager> _renderPassManager;
    std::unique_ptr<SceneManager> _sceneManager;
    std::unique_ptr<PipelineManager> _pipelineManager;
    std::unique_ptr<AssetManager> _assetManager;
    std::unique_ptr<AsyncAssetLoader> _asyncLoader;
    std::unique_ptr<RenderGraph> _renderGraph;
    std::unique_ptr<RayTracingManager> _rayManager;
    std::unique_ptr<TextureCache> _textureCache;
    std::unique_ptr<IBLManager> _iblManager;

	struct SDL_Window *_window{nullptr};

    FrameResources _frames[FRAME_OVERLAP];

    FrameResources &get_current_frame() { return _frames[_frameNumber % FRAME_OVERLAP]; };

	VkExtent2D _drawExtent;
	float renderScale = 1.f;

    std::unique_ptr<DescriptorManager> _descriptorManager;
    std::unique_ptr<SamplerManager> _samplerManager;
    ComputeManager compute;

    std::shared_ptr<EngineContext> _context;

	std::vector<VkFramebuffer> _framebuffers;

    DeletionQueue _mainDeletionQueue;

    VkPipelineLayout _meshPipelineLayout;
    VkPipeline _meshPipeline;

	std::shared_ptr<MeshAsset> cubeMesh;
	std::shared_ptr<MeshAsset> sphereMesh;

	AllocatedImage _whiteImage;
	AllocatedImage _blackImage;
	AllocatedImage _greyImage;
	AllocatedImage _errorCheckerboardImage;
    AllocatedImage _flatNormalImage; // 1x1 (0.5,0.5,1.0)

    MaterialInstance defaultData;

    GLTFMetallic_Roughness metalRoughMaterial;

    EngineStats stats;

    std::vector<RenderPass> renderPasses;

    // Debug helpers: track spawned IBL test meshes to remove them easily
    std::vector<std::string> _iblTestNames;

    // Simple world-space IBL reflection volumes (axis-aligned boxes).
    struct IBLVolume
    {
        glm::vec3 center{0.0f, 0.0f, 0.0f};
        glm::vec3 halfExtents{10.0f, 10.0f, 10.0f};
        IBLPaths paths{};   // HDRI paths for this volume
        bool enabled{true};
    };
    // Global/default IBL used when no volume contains the camera.
    IBLPaths _globalIBLPaths{};
    bool _hasGlobalIBL{false};
    // User-defined local IBL volumes and currently active index (-1 = global).
    std::vector<IBLVolume> _iblVolumes;
    int _activeIBLVolume{-1};

    struct PickInfo
    {
        MeshAsset *mesh = nullptr;
        LoadedGLTF *scene = nullptr;
        Node *node = nullptr;
        RenderObject::OwnerType ownerType = RenderObject::OwnerType::None;
        std::string ownerName;
        glm::vec3 worldPos{0.0f};
        glm::mat4 worldTransform{1.0f};
        uint32_t indexCount = 0;
        uint32_t firstIndex = 0;
        uint32_t surfaceIndex = 0;
        bool valid = false;
    } _lastPick;
    uint32_t _lastPickObjectID = 0;

    struct PickRequest
    {
        bool active = false;
        glm::vec2 windowPos{0.0f};
        glm::uvec2 idCoords{0, 0};
    } _pendingPick;
    bool _pickResultPending = false;
    AllocatedBuffer _pickReadbackBuffer{};

    // Hover and drag-selection state (raycast-based)
    PickInfo _hoverPick{};
    glm::vec2 _mousePosPixels{-1.0f, -1.0f};
    struct DragState
    {
        bool dragging = false;
        bool buttonDown = false;
        glm::vec2 start{0.0f};
        glm::vec2 current{0.0f};
    } _dragState;
    // Optional list of last drag-selected objects (for future editing UI)
    std::vector<PickInfo> _dragSelection;

    // Toggle to enable/disable ID-buffer picking in addition to raycast
    bool _useIdBufferPicking = false;
    // Debug: draw mesh BVH boxes for last pick
    bool _debugDrawBVH = false;

    // Last click selection (CPU ray or ID-buffer). Useful for game/editor code.
    const PickInfo &get_last_pick() const { return _lastPick; }

    // Debug: persistent pass enable overrides (by pass name)
    std::unordered_map<std::string, bool> _rgPassToggles;

	//initializes everything in the engine
	void init();

	//shuts down the engine
	void cleanup();

	//draw loop
	void draw();

	//run main loop
	void run();

    // Query a conservative streaming texture budget for the texture cache.
    size_t query_texture_budget_bytes() const;

    // Convenience helper: load a glTF from assets/models and add it as a runtime instance.
    // modelRelativePath is relative to the AssetManager model root.
    bool addGLTFInstance(const std::string &instanceName,
                         const std::string &modelRelativePath,
                         const glm::mat4 &transform = glm::mat4(1.f));

    // Asynchronous glTF load that reports progress via AsyncAssetLoader.
    // Returns a JobID that can be queried via AsyncAssetLoader.
    uint32_t loadGLTFAsync(const std::string &sceneName,
                           const std::string &modelRelativePath,
                           const glm::mat4 &transform = glm::mat4(1.f));

	bool resize_requested{false};
	bool freeze_rendering{false};

private:
    void init_frame_resources();

    void init_pipelines();

    void init_mesh_pipeline();

    void init_default_data();
};
