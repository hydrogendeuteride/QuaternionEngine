// vulkan_engine.h : Include file for standard system include files,
// or project specific include files.

#pragma once

#include <core/types.h>
#include <core/world.h>
#include <vector>
#include <string>
#include <unordered_map>
#include <cstdint>
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
#include "core/ui/imgui_system.h"
#include "core/picking/picking_system.h"
#include "core/input/input_system.h"

class DebugDrawSystem;

struct DebugDrawDeleter
{
    void operator()(DebugDrawSystem *p) const;
};

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
    enum class WindowMode : uint8_t
    {
        Windowed = 0,
        FullscreenDesktop = 1,  // borderless fullscreen ("fullscreen windowed")
        FullscreenExclusive = 2 // exclusive fullscreen (may change display mode)
    };

    ~VulkanEngine();

	bool _isInitialized{false};
	int _frameNumber{0};

	std::shared_ptr<DeviceManager> _deviceManager;
	std::unique_ptr<SwapchainManager> _swapchainManager;
	std::shared_ptr<ResourceManager> _resourceManager;
	std::unique_ptr<RenderPassManager> _renderPassManager;
	std::unique_ptr<ImGuiSystem> _ui;
	std::unique_ptr<SceneManager> _sceneManager;
    std::unique_ptr<PickingSystem> _picking;
    std::unique_ptr<InputSystem> _input;
    std::unique_ptr<DebugDrawSystem, DebugDrawDeleter> _debugDraw;
	std::unique_ptr<PipelineManager> _pipelineManager;
	std::unique_ptr<AssetManager> _assetManager;
	std::unique_ptr<AsyncAssetLoader> _asyncLoader;
    std::unique_ptr<RenderGraph> _renderGraph;
    std::unique_ptr<RayTracingManager> _rayManager;
    std::unique_ptr<TextureCache> _textureCache;
    std::unique_ptr<IBLManager> _iblManager;

	struct SDL_Window *_window{nullptr};

    WindowMode _windowMode{WindowMode::Windowed};
    int _windowDisplayIndex{0};

    // HiDPI: allow high-DPI drawables (Retina / Windows scaling).
    // Window coordinates from SDL events may be in "points" while the drawable/swapchain is in pixels.
    bool _hiDpiEnabled{true};

    FrameResources _frames[FRAME_OVERLAP];

    FrameResources &get_current_frame() { return _frames[_frameNumber % FRAME_OVERLAP]; };

	VkExtent2D _drawExtent;
    VkExtent2D _logicalRenderExtent{};
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

	    // Simple world-space IBL reflection volumes (boxes or spheres).
	    enum class IBLVolumeShape : uint8_t
	    {
	        Box = 0,
	        Sphere = 1
	    };
	    struct IBLVolume
	    {
	        WorldVec3 center_world{0.0, 0.0, 0.0};
	        glm::vec3 halfExtents{10.0f, 10.0f, 10.0f};
	        IBLPaths paths{};   // HDRI paths for this volume
	        bool enabled{true};
	        IBLVolumeShape shape{IBLVolumeShape::Box};
	        float radius{10.0f};
	    };
    // Global/default IBL used when no volume contains the camera.
    IBLPaths _globalIBLPaths{};
    bool _hasGlobalIBL{false};
    // User-defined local IBL volumes and currently active index (-1 = global).
    std::vector<IBLVolume> _iblVolumes;
    int _activeIBLVolume{-1};
    // Pending async IBL request (global or volume). targetVolume = -1 means global.
    struct PendingIBLRequest
    {
        bool active{false};
        int targetVolume{-1};
        IBLPaths paths{};
    } _pendingIBLRequest;

    ImGuiSystem *ui() { return _ui.get(); }
    const ImGuiSystem *ui() const { return _ui.get(); }

    PickingSystem *picking() { return _picking.get(); }
    const PickingSystem *picking() const { return _picking.get(); }

    InputSystem *input() { return _input.get(); }
    const InputSystem *input() const { return _input.get(); }

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

    // Window controls (runtime)
    void set_window_mode(WindowMode mode, int display_index);

    // Rendering resolution controls:
    // - logicalRenderExtent controls camera aspect and picking (letterboxed view).
    // - renderScale controls the internal render target pixel count (logical * scale).
    void set_logical_render_extent(VkExtent2D extent);
    void set_render_scale(float scale);

    // Query a conservative streaming texture budget for the texture cache.
    size_t query_texture_budget_bytes() const;

    // Convenience helper: load a glTF from assets/models and add it as a runtime instance.
    // modelRelativePath is relative to the AssetManager model root.
    // If preloadTextures is true, textures will be immediately marked for loading to VRAM.
    bool addGLTFInstance(const std::string &instanceName,
                         const std::string &modelRelativePath,
                         const glm::mat4 &transform = glm::mat4(1.f),
                         bool preloadTextures = false);

    // Spawn a runtime primitive mesh instance (cube/sphere/plane/capsule).
    // - instanceName is the unique key for this object in SceneManager.
    // - geomType selects which analytic primitive to build.
    // - material controls whether the primitive uses the default PBR material
    //   or a textured material (see AssetManager::MeshMaterialDesc).
    // - boundsTypeOverride can force a specific bounds type for picking.
    // The underlying mesh is cached in AssetManager using a per-primitive name,
    // so multiple instances share GPU buffers.
    bool addPrimitiveInstance(const std::string &instanceName,
                              AssetManager::MeshGeometryDesc::Type geomType,
                              const glm::mat4 &transform = glm::mat4(1.f),
                              const AssetManager::MeshMaterialDesc &material = {},
                              std::optional<BoundsType> boundsTypeOverride = {});

    // Asynchronous glTF load that reports progress via AsyncAssetLoader.
    // Returns a JobID that can be queried via AsyncAssetLoader.
    // If preloadTextures is true, textures will be immediately marked for loading to VRAM.
    uint32_t loadGLTFAsync(const std::string &sceneName,
                           const std::string &modelRelativePath,
                           const glm::mat4 &transform = glm::mat4(1.f),
                           bool preloadTextures = false);

    uint32_t loadGLTFAsync(const std::string &sceneName,
                           const std::string &modelRelativePath,
                           const WorldVec3 &translationWorld,
                           const glm::quat &rotation,
                           const glm::vec3 &scale,
                           bool preloadTextures = false);

    // Preload textures for an already-loaded scene instance so they are
    // available before the object becomes visible (visibility-driven loading).
    void preloadInstanceTextures(const std::string &instanceName);

	bool resize_requested{false};
	bool freeze_rendering{false};

private:
    struct WindowedRect
    {
        int x{0};
        int y{0};
        int w{0};
        int h{0};
        bool valid{false};
    } _windowedRect;

    void init_frame_resources();

    void init_pipelines();

    void init_mesh_pipeline();

    void init_default_data();

    // Debounce swapchain recreation during live window resizing.
    uint32_t _last_resize_event_ms{0};
    static constexpr uint32_t RESIZE_DEBOUNCE_MS = 150;
};
