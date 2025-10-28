// vulkan_engine.h : Include file for standard system include files,
// or project specific include files.

#pragma once

#include <core/vk_types.h>
#include <vector>
#include <string>
#include <unordered_map>
#include "vk_mem_alloc.h"
#include <deque>
#include <functional>
#include "vk_descriptors.h"
#include "scene/vk_loader.h"
#include "compute/vk_compute.h"
#include <scene/camera.h>

#include "vk_device.h"
#include "render/vk_renderpass.h"
#include "render/vk_renderpass_background.h"
#include "vk_resource.h"
#include "vk_swapchain.h"
#include "scene/vk_scene.h"
#include "render/vk_materials.h"

#include "frame_resources.h"
#include "vk_descriptor_manager.h"
#include "vk_sampler_manager.h"
#include "core/engine_context.h"
#include "core/vk_pipeline_manager.h"
#include "core/asset_manager.h"
#include "render/rg_graph.h"
#include "core/vk_raytracing.h"

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
    std::unique_ptr<RenderGraph> _renderGraph;
    std::unique_ptr<RayTracingManager> _rayManager;

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

    MaterialInstance defaultData;

    GLTFMetallic_Roughness metalRoughMaterial;

    EngineStats stats;

    std::vector<RenderPass> renderPasses;

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

	bool resize_requested{false};
	bool freeze_rendering{false};

private:
    void init_frame_resources();

    void init_pipelines();

    void init_mesh_pipeline();

    void init_default_data();
};
