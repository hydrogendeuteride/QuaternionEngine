// Engine bootstrap, frame loop, and render-graph wiring.
//
// Responsibilities
// - Initialize SDL + Vulkan managers (device, resources, descriptors, samplers, pipelines).
// - Create swapchain + default images and build the Render Graph each frame.
// - Publish an EngineContext so passes and subsystems access per‑frame state uniformly.
// - Drive ImGui + debug UIs and optional ray‑tracing TLAS rebuilds.
//
// See also:
//  - docs/EngineContext.md
//  - docs/RenderGraph.md
//  - docs/FrameResources.md
//  - docs/RayTracing.md
//
//> includes
#include "engine.h"

#include "core/input/input_system.h"

#include "SDL2/SDL.h"
#include "SDL2/SDL_vulkan.h"

#include <core/util/initializers.h>
#include <core/types.h>

#include "VkBootstrap.h"

#include <chrono>
#include <thread>
#include <span>
#include <array>

#include <filesystem>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <glm/gtx/transform.hpp>

#include "config.h"
#include "render/primitives.h"

#include "vk_mem_alloc.h"
#include "core/ui/imgui_system.h"
#include "core/picking/picking_system.h"
#include "core/debug_draw/debug_draw.h"
#include "render/passes/geometry.h"
#include "render/passes/imgui_pass.h"
#include "render/passes/lighting.h"
#include "render/passes/clouds.h"
#include "render/passes/particles.h"
#include "render/passes/transparent.h"
#include "render/passes/fxaa.h"
#include "render/passes/tonemap.h"
#include "render/passes/debug_draw.h"
#include "render/passes/shadow.h"
#include "scene/mesh_bvh.h"
#include "device/resource.h"
#include "device/images.h"
#include "context.h"
#include "core/pipeline/manager.h"
#include "core/assets/texture_cache.h"
#include "core/assets/ibl_manager.h"

// ImGui debug UI (tabs, inspectors, etc.) is implemented in core/vk_engine_ui.cpp.
void vk_engine_draw_debug_ui(VulkanEngine *eng);

VulkanEngine *loadedEngine = nullptr;

VulkanEngine::~VulkanEngine() = default;

void DebugDrawDeleter::operator()(DebugDrawSystem *p) const
{
    delete p;
}

static VkExtent2D clamp_nonzero_extent(VkExtent2D extent)
{
    if (extent.width == 0) extent.width = 1;
    if (extent.height == 0) extent.height = 1;
    return extent;
}

static VkExtent2D scaled_extent(VkExtent2D logicalExtent, float scale)
{
    logicalExtent = clamp_nonzero_extent(logicalExtent);
    if (!std::isfinite(scale)) scale = 1.0f;
    scale = std::clamp(scale, 0.1f, 4.0f);

    const float fw = static_cast<float>(logicalExtent.width) * scale;
    const float fh = static_cast<float>(logicalExtent.height) * scale;

    VkExtent2D out{};
    out.width = static_cast<uint32_t>(std::max(1.0f, std::floor(fw)));
    out.height = static_cast<uint32_t>(std::max(1.0f, std::floor(fh)));
    return out;
}

static bool file_exists_nothrow(const std::string &path)
{
    if (path.empty()) return false;
    std::error_code ec;
    return std::filesystem::exists(path, ec) && !ec;
}

static void print_vma_stats(DeviceManager* dev, const char* tag)
{
    if (!vmaDebugEnabled()) return;
    if (!dev) return;
    VmaAllocator alloc = dev->allocator();
    if (!alloc) return;
    VmaTotalStatistics stats{};
    vmaCalculateStatistics(alloc, &stats);
    const VmaStatistics &s = stats.total.statistics;
    fmt::print("[VMA][{}] Blocks:{} Allocs:{} BlockBytes:{} AllocBytes:{}\n",
               tag,
               (size_t)s.blockCount,
               (size_t)s.allocationCount,
               (unsigned long long)s.blockBytes,
               (unsigned long long)s.allocationBytes);
}

static void dump_vma_json(DeviceManager* dev, const char* tag)
{
    if (!vmaDebugEnabled()) return;
    if (!dev) return;
    VmaAllocator alloc = dev->allocator();
    if (!alloc) return;
    char* json = nullptr;
    vmaBuildStatsString(alloc, &json, VK_TRUE);
    if (json)
    {
        // Write to a small temp file beside the binary
        std::string fname = std::string("vma_") + tag + ".json";
        FILE* f = fopen(fname.c_str(), "wb");
        if (f)
        {
            fwrite(json, 1, strlen(json), f);
            fclose(f);
            fmt::print("[VMA] Wrote {}\n", fname);
        }
        vmaFreeStatsString(alloc, json);
    }
}

size_t VulkanEngine::query_texture_budget_bytes() const
{
    DeviceManager *dev = _deviceManager.get();
    if (!dev) return kTextureBudgetFallbackBytes; // fallback
    VmaAllocator alloc = dev->allocator();
    if (!alloc) return kTextureBudgetFallbackBytes;

    const VkPhysicalDeviceMemoryProperties *memProps = nullptr;
    vmaGetMemoryProperties(alloc, &memProps);
    if (!memProps) return kTextureBudgetFallbackBytes;

    VmaBudget budgets[VK_MAX_MEMORY_HEAPS] = {};
    vmaGetHeapBudgets(alloc, budgets);

    unsigned long long totalBudget = 0;
    unsigned long long totalUsage = 0;
    for (uint32_t i = 0; i < memProps->memoryHeapCount; ++i)
    {
        if (memProps->memoryHeaps[i].flags & VK_MEMORY_HEAP_DEVICE_LOCAL_BIT)
        {
            totalBudget += budgets[i].budget;
            totalUsage  += budgets[i].usage;
        }
    }
    if (totalBudget == 0) return kTextureBudgetFallbackBytes;

    unsigned long long cap = static_cast<unsigned long long>(double(totalBudget) * kTextureBudgetFraction);

    // If usage is already near the cap, still allow current textures to live; eviction will trim.
    // Clamp to at least a minimum budget, at most totalBudget.
    unsigned long long minCap = static_cast<unsigned long long>(kTextureBudgetMinBytes);
    if (cap < minCap) cap = minCap;
    if (cap > totalBudget) cap = totalBudget;
    return static_cast<size_t>(cap);
}

void VulkanEngine::init()
{
    // DPI awareness and HiDPI behavior must be configured before initializing the video subsystem.
#if defined(_WIN32)
#ifdef SDL_HINT_WINDOWS_DPI_AWARENESS
    SDL_SetHint(SDL_HINT_WINDOWS_DPI_AWARENESS, "permonitorv2");
#endif
#endif

    // We initialize SDL and create a window with it.
    SDL_Init(SDL_INIT_VIDEO);

    // Initialize default logical render resolution for the engine.
    _logicalRenderExtent.width = kRenderWidth;
    _logicalRenderExtent.height = kRenderHeight;
    _logicalRenderExtent = clamp_nonzero_extent(_logicalRenderExtent);
    _drawExtent = scaled_extent(_logicalRenderExtent, renderScale);

    SDL_WindowFlags window_flags = static_cast<SDL_WindowFlags>(SDL_WINDOW_VULKAN | SDL_WINDOW_RESIZABLE);
    if (_hiDpiEnabled)
    {
        window_flags = static_cast<SDL_WindowFlags>(window_flags | SDL_WINDOW_ALLOW_HIGHDPI);
    }

    _swapchainManager = std::make_unique<SwapchainManager>();

    _window = SDL_CreateWindow(
        "Vulkan Engine",
        SDL_WINDOWPOS_UNDEFINED,
        SDL_WINDOWPOS_UNDEFINED,
        _swapchainManager->windowExtent().width,
        _swapchainManager->windowExtent().height,
        window_flags
    );

    if (_swapchainManager)
    {
        _swapchainManager->set_window_extent_from_window(_window);
    }

    _windowMode = WindowMode::Windowed;
    _windowDisplayIndex = SDL_GetWindowDisplayIndex(_window);
    if (_windowDisplayIndex < 0) _windowDisplayIndex = 0;
    {
        int wx = 0, wy = 0, ww = 0, wh = 0;
        SDL_GetWindowPosition(_window, &wx, &wy);
        SDL_GetWindowSize(_window, &ww, &wh);
        if (ww > 0 && wh > 0)
        {
            _windowedRect.x = wx;
            _windowedRect.y = wy;
            _windowedRect.w = ww;
            _windowedRect.h = wh;
            _windowedRect.valid = true;
        }
    }

    _deviceManager = std::make_shared<DeviceManager>();
    _deviceManager->init_vulkan(_window);

    _resourceManager = std::make_shared<ResourceManager>();
    _resourceManager->init(_deviceManager.get());

    _descriptorManager = std::make_unique<DescriptorManager>();
    _descriptorManager->init(_deviceManager.get());

    _samplerManager = std::make_unique<SamplerManager>();
    _samplerManager->init(_deviceManager.get());

    // Build dependency-injection context
    _context = std::make_shared<EngineContext>();
    _input = std::make_unique<InputSystem>();
    _context->input = _input.get();

    _debugDraw.reset(new DebugDrawSystem());
    _context->debug_draw = _debugDraw.get();

    _context->device = _deviceManager;
    _context->resources = _resourceManager;
    _context->descriptors = std::make_shared<DescriptorAllocatorGrowable>(); {
        std::vector<DescriptorAllocatorGrowable::PoolSizeRatio> sizes = {
            {VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1},
            {VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1},
            {VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 4},
        };
        _context->descriptors->init(_deviceManager->device(), 10, sizes);
    }
    _context->logicalRenderExtent = _logicalRenderExtent;

    // Default voxel volumetric presets (up to 4 volumes for performance).
    // Slot 0 matches the old CloudSettings defaults.
    if (_context->voxelVolumes.size() >= 1)
    {
        VoxelVolumeSettings &v = _context->voxelVolumes[0];
        v.enabled = false;
        v.type = VoxelVolumeType::Clouds;
        v.followCameraXZ = true;
        v.animateVoxels = false;
        v.volumeCenterLocal = glm::vec3(0.0f, 80.0f, 0.0f);
        v.volumeHalfExtents = glm::vec3(128.0f, 20.0f, 128.0f);
        v.volumeVelocityLocal = glm::vec3(0.0f);
        v.densityScale = 1.25f;
        v.coverage = 0.45f;
        v.extinction = 1.0f;
        v.stepCount = 64;
        v.gridResolution = 64;
        v.windVelocityLocal = glm::vec3(6.0f, 0.0f, 2.0f);
        v.dissipation = 0.35f;
        v.noiseStrength = 1.0f;
        v.noiseScale = 6.0f;
        v.noiseSpeed = 0.15f;
        v.emitterUVW = glm::vec3(0.5f, 0.05f, 0.5f);
        v.emitterRadius = 0.18f;
        v.albedo = glm::vec3(1.0f);
        v.scatterStrength = 1.0f;
        v.emissionColor = glm::vec3(1.0f, 0.6f, 0.25f);
        v.emissionStrength = 0.0f;
    }
    if (_context->voxelVolumes.size() >= 2)
    {
        VoxelVolumeSettings &v = _context->voxelVolumes[1];
        v.enabled = false;
        v.type = VoxelVolumeType::Smoke;
        v.followCameraXZ = false;
        v.animateVoxels = true;
        v.volumeCenterLocal = glm::vec3(0.0f, 2.0f, 0.0f);
        v.volumeHalfExtents = glm::vec3(6.0f, 6.0f, 6.0f);
        v.volumeVelocityLocal = glm::vec3(0.0f);
        v.densityScale = 1.0f;
        v.coverage = 0.0f;
        v.extinction = 2.5f;
        v.stepCount = 48;
        v.gridResolution = 48;
        v.windVelocityLocal = glm::vec3(0.5f, 3.0f, 0.0f);
        v.dissipation = 1.5f;
        v.noiseStrength = 2.0f;
        v.noiseScale = 10.0f;
        v.noiseSpeed = 1.0f;
        v.emitterUVW = glm::vec3(0.5f, 0.08f, 0.5f);
        v.emitterRadius = 0.22f;
        v.albedo = glm::vec3(0.25f);
        v.scatterStrength = 0.25f;
        v.emissionStrength = 0.0f;
    }
    if (_context->voxelVolumes.size() >= 3)
    {
        VoxelVolumeSettings &v = _context->voxelVolumes[2];
        v.enabled = false;
        v.type = VoxelVolumeType::Flame;
        v.followCameraXZ = false;
        v.animateVoxels = true;
        v.volumeCenterLocal = glm::vec3(0.0f, 1.0f, 0.0f);
        v.volumeHalfExtents = glm::vec3(3.0f, 5.0f, 3.0f);
        v.volumeVelocityLocal = glm::vec3(0.0f);
        v.densityScale = 1.5f;
        v.coverage = 0.0f;
        v.extinction = 1.0f;
        v.stepCount = 56;
        v.gridResolution = 48;
        v.windVelocityLocal = glm::vec3(0.0f, 4.5f, 0.0f);
        v.dissipation = 3.0f;
        v.noiseStrength = 3.0f;
        v.noiseScale = 14.0f;
        v.noiseSpeed = 3.0f;
        v.emitterUVW = glm::vec3(0.5f, 0.10f, 0.5f);
        v.emitterRadius = 0.18f;
        v.albedo = glm::vec3(1.0f);
        v.scatterStrength = 0.0f;
        v.emissionColor = glm::vec3(1.0f, 0.5f, 0.1f);
        v.emissionStrength = 12.0f;
    }

    _swapchainManager->init(_deviceManager.get(), _resourceManager.get());
    _swapchainManager->set_render_extent(_drawExtent);
    _swapchainManager->init_swapchain();

    // Fill remaining context pointers now that managers exist
    _context->descriptorLayouts = _descriptorManager.get();
    _context->samplers = _samplerManager.get();
    _context->swapchain = _swapchainManager.get();

    // Create graphics pipeline manager (after swapchain is ready)
    _pipelineManager = std::make_unique<PipelineManager>();
    _pipelineManager->init(_context.get());
    _context->pipelines = _pipelineManager.get();

    // Create central AssetManager for paths and asset caching
    _assetManager = std::make_unique<AssetManager>();
    _assetManager->init(this);
    _context->assets = _assetManager.get();

    // Create texture cache (engine-owned, accessible via EngineContext)
    _textureCache = std::make_unique<TextureCache>();
    _textureCache->init(_context.get());
    _context->textures = _textureCache.get();
    // Conservative defaults to avoid CPU/RAM/VRAM spikes during heavy glTF loads.
    _textureCache->set_max_loads_per_pump(3);
    _textureCache->set_keep_source_bytes(false);
    _textureCache->set_cpu_source_budget(64ull * 1024ull * 1024ull); // 64 MiB
    _textureCache->set_max_bytes_per_pump(128ull * 1024ull * 1024ull); // 128 MiB/frame
    _textureCache->set_max_upload_dimension(4096);

    // Async asset loader for background glTF + texture jobs
    _asyncLoader = std::make_unique<AsyncAssetLoader>();
    _asyncLoader->init(this, _assetManager.get(), _textureCache.get(), 4);

    // Optional ray tracing manager if supported and extensions enabled
    if (_deviceManager->supportsRayQuery() && _deviceManager->supportsAccelerationStructure())
    {
        _rayManager = std::make_unique<RayTracingManager>();
        _rayManager->init(_deviceManager.get(), _resourceManager.get());
        _context->ray = _rayManager.get();
    }

    _sceneManager = std::make_unique<SceneManager>();
    _sceneManager->init(_context.get());
    _context->scene = _sceneManager.get();

    compute.init(_context.get());
    // Publish engine-owned subsystems into context for modules
    _context->compute = &compute;
    _context->window = _window;
    _context->stats = &stats;

    _picking = std::make_unique<PickingSystem>();
    _picking->init(_context.get());

    // Render graph skeleton
    _renderGraph = std::make_unique<RenderGraph>();
    _renderGraph->init(_context.get());
    _context->renderGraph = _renderGraph.get();

    // Create IBL manager early so set=3 layout exists before pipelines are built
    _iblManager = std::make_unique<IBLManager>();
    _iblManager->init(_context.get());
    if (_textureCache)
    {
        _iblManager->set_texture_cache(_textureCache.get());
    }
    // Publish to context for passes and pipeline layout assembly
    _context->ibl = _iblManager.get();

    // Try to load default IBL assets if present (async)
    {
        IBLPaths ibl{};
        ibl.specularCube = _assetManager->assetPath("ibl/docklands.ktx2");
        ibl.diffuseCube  = _assetManager->assetPath("ibl/docklands.ktx2"); // temporary: reuse if separate diffuse not provided
        ibl.brdfLut2D    = _assetManager->assetPath("ibl/brdf_lut.ktx2");
        // By default, use the same texture for lighting and background; users can point background2D
        // at a different .ktx2 to decouple them.
        ibl.background2D = ibl.specularCube;
        // Treat this as the global/fallback IBL used outside any local volume.
        _globalIBLPaths = ibl;
        _activeIBLVolume = -1;
        _hasGlobalIBL = false;
        if (_iblManager)
        {
            if (_iblManager->load_async(ibl))
            {
                _pendingIBLRequest.active = true;
                _pendingIBLRequest.targetVolume = -1;
                _pendingIBLRequest.paths = ibl;
            }
            else
            {
                fmt::println("[Engine] Warning: failed to enqueue default IBL load (specular='{}', brdfLut='{}'). IBL lighting will be disabled until a valid IBL is loaded.",
                             ibl.specularCube,
                             ibl.brdfLut2D);
            }
        }
    }

    init_frame_resources();

    // Build material pipelines early so materials can be created
    metalRoughMaterial.build_pipelines(this);

    init_default_data();

    _renderPassManager = std::make_unique<RenderPassManager>();
    _renderPassManager->init(_context.get());

    auto imguiPass = std::make_unique<ImGuiPass>();
    _renderPassManager->setImGuiPass(std::move(imguiPass));

    _ui = std::make_unique<ImGuiSystem>();
    _ui->init(_context.get());
    _ui->add_draw_callback([this]() { vk_engine_draw_debug_ui(this); });

    _resourceManager->set_deferred_uploads(true);

    _context->enableSSR = true;

    //everything went fine
    _isInitialized = true;
}

void VulkanEngine::set_window_mode(WindowMode mode, int display_index)
{
    if (!_window) return;

    const int num_displays = SDL_GetNumVideoDisplays();
    int current_display = SDL_GetWindowDisplayIndex(_window);
    if (current_display < 0) current_display = 0;

    if (num_displays <= 0)
    {
        display_index = 0;
    }
    else
    {
        if (display_index < 0) display_index = current_display;
        display_index = std::clamp(display_index, 0, num_displays - 1);
    }

    const WindowMode prev_mode = _windowMode;
    const bool entering_fullscreen = (prev_mode == WindowMode::Windowed) && (mode != WindowMode::Windowed);

    if (entering_fullscreen)
    {
        int wx = 0, wy = 0, ww = 0, wh = 0;
        SDL_GetWindowPosition(_window, &wx, &wy);
        SDL_GetWindowSize(_window, &ww, &wh);
        if (ww > 0 && wh > 0)
        {
            _windowedRect.x = wx;
            _windowedRect.y = wy;
            _windowedRect.w = ww;
            _windowedRect.h = wh;
            _windowedRect.valid = true;
        }
    }

    // If we are currently in any fullscreen mode, leave fullscreen first so we can safely
    // move the window to the target display before re-entering fullscreen.
    if (prev_mode != WindowMode::Windowed)
    {
        if (SDL_SetWindowFullscreen(_window, 0) != 0)
        {
            fmt::println("[Window] SDL_SetWindowFullscreen(0) failed: {}", SDL_GetError());
        }
    }

    // Move the window to the selected display (applies to both windowed and fullscreen).
    SDL_SetWindowPosition(_window,
                          SDL_WINDOWPOS_CENTERED_DISPLAY(display_index),
                          SDL_WINDOWPOS_CENTERED_DISPLAY(display_index));

    if (mode == WindowMode::Windowed)
    {
        SDL_SetWindowBordered(_window, SDL_TRUE);
        SDL_SetWindowResizable(_window, SDL_TRUE);

        int target_w = 0, target_h = 0;
        if (_windowedRect.valid)
        {
            target_w = _windowedRect.w;
            target_h = _windowedRect.h;
        }
        if (target_w <= 0 || target_h <= 0)
        {
            SDL_GetWindowSize(_window, &target_w, &target_h);
        }
        if (target_w > 0 && target_h > 0)
        {
            SDL_SetWindowSize(_window, target_w, target_h);
        }
    }
    else
    {
        SDL_SetWindowBordered(_window, SDL_FALSE);

        const Uint32 fullscreen_flag =
            (mode == WindowMode::FullscreenDesktop) ? SDL_WINDOW_FULLSCREEN_DESKTOP : SDL_WINDOW_FULLSCREEN;

        if (mode == WindowMode::FullscreenExclusive)
        {
            SDL_DisplayMode desktop{};
            if (SDL_GetDesktopDisplayMode(display_index, &desktop) == 0)
            {
                // Request desktop mode by default. Future UI can add explicit mode selection.
                SDL_SetWindowDisplayMode(_window, &desktop);
            }
        }

        if (SDL_SetWindowFullscreen(_window, fullscreen_flag) != 0)
        {
            fmt::println("[Window] SDL_SetWindowFullscreen({}) failed: {}",
                         static_cast<unsigned>(fullscreen_flag),
                         SDL_GetError());
        }
    }

    _windowMode = mode;
    _windowDisplayIndex = SDL_GetWindowDisplayIndex(_window);
    if (_windowDisplayIndex < 0) _windowDisplayIndex = display_index;

    // Make sure SDL has processed the fullscreen/move requests before we query drawable size for swapchain recreation.
    SDL_PumpEvents();

    // Recreate swapchain immediately so the very next draw uses correct extent.
    if (_swapchainManager)
    {
        _swapchainManager->resize_swapchain(_window);
        if (_ui)
        {
            _ui->on_swapchain_recreated();
        }
        if (_swapchainManager->resize_requested)
        {
            resize_requested = true;
            _last_resize_event_ms = 0;
        }
        else
        {
            resize_requested = false;
        }
    }
}

void VulkanEngine::set_logical_render_extent(VkExtent2D extent)
{
    extent = clamp_nonzero_extent(extent);
    if (_logicalRenderExtent.width == extent.width && _logicalRenderExtent.height == extent.height)
    {
        return;
    }

    _logicalRenderExtent = extent;
    if (_context)
    {
        _context->logicalRenderExtent = _logicalRenderExtent;
    }

    VkExtent2D newDraw = scaled_extent(_logicalRenderExtent, renderScale);
    if (_swapchainManager)
    {
        _swapchainManager->resize_render_targets(newDraw);
    }
}

void VulkanEngine::set_render_scale(float scale)
{
    if (!std::isfinite(scale)) scale = 1.0f;
    scale = std::clamp(scale, 0.1f, 4.0f);
    if (std::abs(renderScale - scale) < 1e-4f)
    {
        return;
    }

    renderScale = scale;

    VkExtent2D newDraw = scaled_extent(_logicalRenderExtent, renderScale);
    if (_swapchainManager)
    {
        _swapchainManager->resize_render_targets(newDraw);
    }
}

void VulkanEngine::init_default_data()
{
    //> default_img
    //3 default textures, white, grey, black. 1 pixel each
    uint32_t white = glm::packUnorm4x8(glm::vec4(1, 1, 1, 1));
    _whiteImage = _resourceManager->create_image((void *) &white, VkExtent3D{1, 1, 1}, VK_FORMAT_R8G8B8A8_UNORM,
                                                 VK_IMAGE_USAGE_SAMPLED_BIT);

    uint32_t grey = glm::packUnorm4x8(glm::vec4(0.66f, 0.66f, 0.66f, 1));
    _greyImage = _resourceManager->create_image((void *) &grey, VkExtent3D{1, 1, 1}, VK_FORMAT_R8G8B8A8_UNORM,
                                                VK_IMAGE_USAGE_SAMPLED_BIT);

    uint32_t black = glm::packUnorm4x8(glm::vec4(0, 0, 0, 0));
    _blackImage = _resourceManager->create_image((void *) &black, VkExtent3D{1, 1, 1}, VK_FORMAT_R8G8B8A8_UNORM,
                                                 VK_IMAGE_USAGE_SAMPLED_BIT);

    // Flat normal (0.5, 0.5, 1.0) for missing normal maps
    uint32_t flatN = glm::packUnorm4x8(glm::vec4(0.5f, 0.5f, 1.0f, 1.0f));
    _flatNormalImage = _resourceManager->create_image((void *) &flatN, VkExtent3D{1, 1, 1}, VK_FORMAT_R8G8B8A8_UNORM,
                                                      VK_IMAGE_USAGE_SAMPLED_BIT);

    //checkerboard image
    uint32_t magenta = glm::packUnorm4x8(glm::vec4(1, 0, 1, 1));
    std::array<uint32_t, 16 * 16> pixels{}; //for 16x16 checkerboard texture
    for (int x = 0; x < 16; x++)
    {
        for (int y = 0; y < 16; y++)
        {
            pixels[y * 16 + x] = ((x % 2) ^ (y % 2)) ? magenta : black;
        }
    }
    _errorCheckerboardImage = _resourceManager->create_image(pixels.data(), VkExtent3D{16, 16, 1},
                                                             VK_FORMAT_R8G8B8A8_UNORM,
                                                             VK_IMAGE_USAGE_SAMPLED_BIT);

    // build default primitive meshes via generic AssetManager API
    {
        AssetManager::MeshCreateInfo ci{};
        ci.name = "Cube";
        ci.geometry.type = AssetManager::MeshGeometryDesc::Type::Cube;
        ci.material.kind = AssetManager::MeshMaterialDesc::Kind::Default;
        cubeMesh = _assetManager->createMesh(ci);
    }
    {
        AssetManager::MeshCreateInfo ci{};
        ci.name = "Sphere";
        ci.geometry.type = AssetManager::MeshGeometryDesc::Type::Sphere;
        ci.geometry.sectors = 16;
        ci.geometry.stacks = 16;
        ci.material.kind = AssetManager::MeshMaterialDesc::Kind::Default;
        sphereMesh = _assetManager->createMesh(ci);
    }

    // Note: Default primitive meshes (cubeMesh, sphereMesh) are created above
    // but no longer spawned as scene instances. Use GameRuntime callbacks
    // or GameAPI to add objects to the scene at runtime.

    _mainDeletionQueue.push_function([&]() {
        _resourceManager->destroy_image(_whiteImage);
        _resourceManager->destroy_image(_greyImage);
        _resourceManager->destroy_image(_blackImage);
        _resourceManager->destroy_image(_errorCheckerboardImage);
        _resourceManager->destroy_image(_flatNormalImage);
    });
    //< default_img
}

bool VulkanEngine::addGLTFInstance(const std::string &instanceName,
                                   const std::string &modelRelativePath,
                                   const glm::mat4 &transform,
                                   bool preloadTextures)
{
    if (!_assetManager || !_sceneManager)
    {
        return false;
    }

    const std::string resolvedPath = _assetManager->modelPath(modelRelativePath);
    if (!file_exists_nothrow(resolvedPath))
    {
        fmt::println("[Engine] Failed to add glTF instance '{}' – model file not found (requested='{}', resolved='{}')",
                     instanceName,
                     modelRelativePath,
                     resolvedPath);
        return false;
    }

    auto gltf = _assetManager->loadGLTF(resolvedPath);
    if (!gltf.has_value() || !gltf.value())
    {
        fmt::println("[Engine] Failed to add glTF instance '{}' – AssetManager::loadGLTF('{}') returned empty scene",
                     instanceName,
                     resolvedPath);
        return false;
    }

    // Provide a readable debug name for UI/picking when missing.
    if ((*gltf)->debugName.empty())
    {
        (*gltf)->debugName = modelRelativePath;
    }

    _sceneManager->addGLTFInstance(instanceName, *gltf, transform);

    // Optionally preload textures for runtime-added instances
    if (preloadTextures && _textureCache && _resourceManager)
    {
        uint32_t frame = static_cast<uint32_t>(_frameNumber);
        uint32_t count = 0;

        for (const auto &[name, material] : (*gltf)->materials)
        {
            if (material && material->data.materialSet)
            {
                _textureCache->markSetUsed(material->data.materialSet, frame);
                ++count;
            }
        }

        if (count > 0)
        {
            fmt::println("[Engine] Marked {} materials for preloading in instance '{}'",
                         count, instanceName);

            // Trigger immediate texture loading pump to start upload
            _textureCache->pumpLoads(*_resourceManager, get_current_frame());
        }
    }

    return true;
}

bool VulkanEngine::addPrimitiveInstance(const std::string &instanceName,
                                        AssetManager::MeshGeometryDesc::Type geomType,
                                        const glm::mat4 &transform,
                                        const AssetManager::MeshMaterialDesc &material,
                                        std::optional<BoundsType> boundsTypeOverride)
{
    if (!_assetManager || !_sceneManager)
    {
        return false;
    }

    // Build a cache key for the primitive mesh so multiple instances
    // share the same GPU buffers.
    std::string meshName;
    switch (geomType)
    {
    case AssetManager::MeshGeometryDesc::Type::Cube:
        meshName = "Primitive.Cube";
        break;
    case AssetManager::MeshGeometryDesc::Type::Sphere:
        meshName = "Primitive.Sphere";
        break;
    case AssetManager::MeshGeometryDesc::Type::Plane:
        meshName = "Primitive.Plane";
        break;
    case AssetManager::MeshGeometryDesc::Type::Capsule:
        meshName = "Primitive.Capsule";
        break;
    case AssetManager::MeshGeometryDesc::Type::Provided:
    default:
        // Provided geometry requires explicit vertex/index data; not supported here.
        return false;
    }

    AssetManager::MeshCreateInfo ci{};
    ci.name = meshName;
    ci.geometry.type = geomType;
    ci.material = material;
    ci.boundsType = boundsTypeOverride;

    auto mesh = _assetManager->createMesh(ci);
    if (!mesh)
    {
        return false;
    }

    _sceneManager->addMeshInstance(instanceName, mesh, transform, boundsTypeOverride);
    return true;
}

uint32_t VulkanEngine::loadGLTFAsync(const std::string &sceneName,
                                     const std::string &modelRelativePath,
                                     const glm::mat4 &transform,
                                     bool preloadTextures)
{
    if (!_asyncLoader || !_assetManager || !_sceneManager)
    {
        return 0;
    }

    const std::string resolvedPath = _assetManager->modelPath(modelRelativePath);
    if (!file_exists_nothrow(resolvedPath))
    {
        fmt::println("[Engine] Failed to enqueue async glTF load for scene '{}' – model file not found (requested='{}', resolved='{}')",
                     sceneName,
                     modelRelativePath,
                     resolvedPath);
        return 0;
    }

    return _asyncLoader->load_gltf_async(sceneName, resolvedPath, transform, preloadTextures);
}

uint32_t VulkanEngine::loadGLTFAsync(const std::string &sceneName,
                                     const std::string &modelRelativePath,
                                     const WorldVec3 &translationWorld,
                                     const glm::quat &rotation,
                                     const glm::vec3 &scale,
                                     bool preloadTextures)
{
    if (!_asyncLoader || !_assetManager || !_sceneManager)
    {
        return 0;
    }

    const std::string resolvedPath = _assetManager->modelPath(modelRelativePath);
    if (!file_exists_nothrow(resolvedPath))
    {
        fmt::println("[Engine] Failed to enqueue async glTF load for scene '{}' – model file not found (requested='{}', resolved='{}')",
                     sceneName,
                     modelRelativePath,
                     resolvedPath);
        return 0;
    }

    return _asyncLoader->load_gltf_async(sceneName,
                                         resolvedPath,
                                         translationWorld,
                                         rotation,
                                         scale,
                                         preloadTextures);
}

void VulkanEngine::preloadInstanceTextures(const std::string &instanceName)
{
    if (!_textureCache || !_sceneManager)
    {
        return;
    }

    auto gltfScene = _sceneManager->getGLTFInstanceScene(instanceName);
    if (!gltfScene)
    {
        return;
    }

    uint32_t frame = static_cast<uint32_t>(_frameNumber);
    uint32_t count = 0;

    // Mark all materials in this glTF scene as used so TextureCache will
    // schedule their textures for upload before the object is visible.
    for (const auto &[name, material] : gltfScene->materials)
    {
        if (material && material->data.materialSet)
        {
            _textureCache->markSetUsed(material->data.materialSet, frame);
            ++count;
        }
    }

    fmt::println("[Engine] Preloaded {} material sets for instance '{}'", count, instanceName);
}

void VulkanEngine::cleanup()
{
    if (_asyncLoader)
    {
        _asyncLoader->shutdown();
        _asyncLoader.reset();
    }

    vkDeviceWaitIdle(_deviceManager->device());

    print_vma_stats(_deviceManager.get(), "begin");

    _sceneManager->cleanup();
    print_vma_stats(_deviceManager.get(), "after SceneManager");
    dump_vma_json(_deviceManager.get(), "after_SceneManager");

    if (_isInitialized)
    {
        //make sure the gpu has stopped doing its things
        vkDeviceWaitIdle(_deviceManager->device());

        if (_ui)
        {
            _ui->cleanup();
            _ui.reset();
        }
        if (_picking)
        {
            _picking->cleanup();
            _picking.reset();
        }
        if (_debugDraw)
        {
            if (_context)
            {
                _context->debug_draw = nullptr;
            }
            _debugDraw.reset();
        }

        // Flush all frame deletion queues first while VMA allocator is still alive
        for (int i = 0; i < FRAME_OVERLAP; i++)
        {
            _frames[i]._deletionQueue.flush();
        }
        for (int i = 0; i < FRAME_OVERLAP; i++)
        {
            _frames[i].cleanup(_deviceManager.get());
        }

        metalRoughMaterial.clear_resources(_deviceManager->device());

        _mainDeletionQueue.flush();
        print_vma_stats(_deviceManager.get(), "after MainDQ flush");
        dump_vma_json(_deviceManager.get(), "after_MainDQ");

        if (_textureCache) { _textureCache->cleanup(); }

        _renderPassManager->cleanup();
        print_vma_stats(_deviceManager.get(), "after RenderPassManager");
        dump_vma_json(_deviceManager.get(), "after_RenderPassManager");

        _pipelineManager->cleanup();
        print_vma_stats(_deviceManager.get(), "after PipelineManager");
        dump_vma_json(_deviceManager.get(), "after_PipelineManager");

        compute.cleanup();
        print_vma_stats(_deviceManager.get(), "after Compute");
        dump_vma_json(_deviceManager.get(), "after_Compute");

        // Ensure RenderGraph's timestamp query pool is destroyed before the device.
        if (_renderGraph)
        {
            _renderGraph->shutdown();
        }

        _swapchainManager->cleanup();
        print_vma_stats(_deviceManager.get(), "after Swapchain");
        dump_vma_json(_deviceManager.get(), "after_Swapchain");

        if (_assetManager) _assetManager->cleanup();
        print_vma_stats(_deviceManager.get(), "after AssetManager");
        dump_vma_json(_deviceManager.get(), "after_AssetManager");

        // Release IBL GPU resources (spec/diffuse cubes + BRDF LUT)
        if (_iblManager)
        {
            _iblManager->unload();
        }
        print_vma_stats(_deviceManager.get(), "after IBLManager");
        dump_vma_json(_deviceManager.get(), "after_IBLManager");

        // Ensure ray tracing resources (BLAS/TLAS/instance buffers) are freed before VMA is destroyed
        if (_rayManager) { _rayManager->cleanup(); }
        print_vma_stats(_deviceManager.get(), "after RTManager");
        dump_vma_json(_deviceManager.get(), "after_RTManager");

        _resourceManager->cleanup();
        print_vma_stats(_deviceManager.get(), "after ResourceManager");
        dump_vma_json(_deviceManager.get(), "after_ResourceManager");

        _samplerManager->cleanup();
        _descriptorManager->cleanup();
        print_vma_stats(_deviceManager.get(), "after Samplers+Descriptors");
        dump_vma_json(_deviceManager.get(), "after_Samplers_Descriptors");

        _context->descriptors->destroy_pools(_deviceManager->device());

        // Extra safety: flush frame deletion queues once more before destroying VMA
        for (int i = 0; i < FRAME_OVERLAP; i++)
        {
            _frames[i]._deletionQueue.flush();
        }

        print_vma_stats(_deviceManager.get(), "before DeviceManager");
        dump_vma_json(_deviceManager.get(), "before_DeviceManager");
        _deviceManager->cleanup();

        if (_input)
        {
            _input->set_cursor_mode(CursorMode::Normal);
            _input.reset();
        }
        if (_context)
        {
            _context->input = nullptr;
        }

        SDL_DestroyWindow(_window);
    }
}

void VulkanEngine::draw()
{
    // Integrate any completed async asset jobs into the scene before updating.
    if (_asyncLoader && _sceneManager)
    {
        _asyncLoader->pump_main_thread(*_sceneManager);
    }

    // Apply any completed async pipeline rebuilds before using pipelines this frame.
    if (_pipelineManager)
    {
        _pipelineManager->pump_main_thread();
    }

    _sceneManager->update_scene();

    if (_debugDraw && _sceneManager)
    {
        _debugDraw->begin_frame(_sceneManager->getDeltaTime());
    }

    // Update IBL based on camera position and user-defined reflection volumes.
    if (_iblManager && _sceneManager)
    {
        WorldVec3 camPosWorld = _sceneManager->getMainCamera().position_world;
        int newVolume = -1;
        for (size_t i = 0; i < _iblVolumes.size(); ++i)
        {
            const IBLVolume &v = _iblVolumes[i];
            if (!v.enabled) continue;
            WorldVec3 local = camPosWorld - v.center_world;
            if (std::abs(local.x) <= static_cast<double>(v.halfExtents.x) &&
                std::abs(local.y) <= static_cast<double>(v.halfExtents.y) &&
                std::abs(local.z) <= static_cast<double>(v.halfExtents.z))
            {
                newVolume = static_cast<int>(i);
                break;
            }
        }

        if (newVolume != _activeIBLVolume)
        {
            IBLPaths *paths = nullptr;
            if (newVolume >= 0)
            {
                paths = &_iblVolumes[newVolume].paths;
            }
            else if (_hasGlobalIBL)
            {
                paths = &_globalIBLPaths;
            }

            // Avoid enqueueing duplicate jobs for the same target volume.
            const bool alreadyPendingForTarget =
                _pendingIBLRequest.active && _pendingIBLRequest.targetVolume == newVolume;

            if (paths && !alreadyPendingForTarget)
            {
                IBLPaths resolved = *paths;
                if (_assetManager)
                {
                    if (!resolved.specularCube.empty()) resolved.specularCube = _assetManager->assetPath(resolved.specularCube);
                    if (!resolved.diffuseCube.empty()) resolved.diffuseCube = _assetManager->assetPath(resolved.diffuseCube);
                    if (!resolved.brdfLut2D.empty()) resolved.brdfLut2D = _assetManager->assetPath(resolved.brdfLut2D);
                    if (!resolved.background2D.empty()) resolved.background2D = _assetManager->assetPath(resolved.background2D);
                }
                *paths = resolved;

                if (_iblManager->load_async(resolved))
                {
                    _pendingIBLRequest.active = true;
                    _pendingIBLRequest.targetVolume = newVolume;
                    _pendingIBLRequest.paths = resolved;
                }
                else
                {
                    fmt::println("[Engine] Warning: failed to enqueue IBL load for {} (specular='{}')",
                                 (newVolume >= 0) ? "volume" : "global environment",
                                 resolved.specularCube);
                }
            }
        }
    }

    if (_picking)
    {
        _picking->update_hover();
    }

    // Compute desired internal render-target extent from logical extent + render scale.
    _drawExtent = scaled_extent(_logicalRenderExtent, renderScale);
    if (_swapchainManager)
    {
        VkExtent2D current = _swapchainManager->renderExtent();
        if (current.width != _drawExtent.width || current.height != _drawExtent.height)
        {
            _swapchainManager->resize_render_targets(_drawExtent);
        }
    }

    uint32_t swapchainImageIndex;

    VkResult e = vkAcquireNextImageKHR(_deviceManager->device(),
                                       _swapchainManager->swapchain(),
                                       1000000000,
                                       get_current_frame()._swapchainSemaphore,
                                       nullptr,
                                       &swapchainImageIndex);
    if (e == VK_ERROR_OUT_OF_DATE_KHR)
    {
        resize_requested = true;
        return;
    }
    if (e == VK_SUBOPTIMAL_KHR)
    {
        // Acquire succeeded and signaled the semaphore. Keep rendering this frame
        // so the semaphore gets waited on, but schedule a resize soon.
        resize_requested = true;
    }
    else
    {
        VK_CHECK(e);
    }

    VK_CHECK(vkResetFences(_deviceManager->device(), 1, &get_current_frame()._renderFence));

    //now that we are sure that the commands finished executing, we can safely reset the command buffer to begin recording again.
    VK_CHECK(vkResetCommandBuffer(get_current_frame()._mainCommandBuffer, 0));

    // Build or update TLAS for current frame now that the previous frame is idle.
    // TLAS is used for hybrid/full RT shadows and RT-assisted SSR reflections.
    // For reflections, only build TLAS when RT is actually enabled (reflectionMode != 0).
    // For shadows, only build TLAS when shadows are enabled and an RT shadow mode is selected.
    const bool rtShadowsActive =
        _context->shadowSettings.enabled && (_context->shadowSettings.mode != 0u);
    const bool rtReflectionsActive =
        _context->enableSSR && (_context->reflectionMode != 0u);
    if (_rayManager && (rtShadowsActive || rtReflectionsActive))
    {
        _rayManager->buildTLASFromDrawContext(_context->getMainDrawContext(), get_current_frame()._deletionQueue);
    }

    //naming it cmd for shorter writing
    VkCommandBuffer cmd = get_current_frame()._mainCommandBuffer;

    //begin the command buffer recording. We will use this command buffer exactly once, so we want to let vulkan know that
    VkCommandBufferBeginInfo cmdBeginInfo = vkinit::command_buffer_begin_info(
        VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT);

    //---------------------------
    VK_CHECK(vkBeginCommandBuffer(cmd, &cmdBeginInfo));

    // publish per-frame pointers and draw extent to context for passes
    _context->currentFrame = &get_current_frame();
    _context->frameIndex = static_cast<uint32_t>(_frameNumber);
    _context->drawExtent = _drawExtent;

    // Inform VMA of current frame for improved internal stats/aging (optional).
    vmaSetCurrentFrameIndex(_deviceManager->allocator(), _context->frameIndex);

    // Optional: check for shader changes and hot-reload pipelines
    if (_pipelineManager)
    {
        _pipelineManager->hotReloadChanged();
    }

    // --- RenderGraph frame build ---
    if (_renderGraph)
    {
        _renderGraph->clear();

        RGImageHandle hDraw = _renderGraph->import_draw_image();
        RGImageHandle hDepth = _renderGraph->import_depth_image();
        RGImageHandle hGBufferPosition = _renderGraph->import_gbuffer_position();
        RGImageHandle hGBufferNormal = _renderGraph->import_gbuffer_normal();
        RGImageHandle hGBufferAlbedo = _renderGraph->import_gbuffer_albedo();
        RGImageHandle hGBufferExtra = _renderGraph->import_gbuffer_extra();
        RGImageHandle hSwapchain = _renderGraph->import_swapchain_image(swapchainImageIndex);
        // For debug overlays (IBL volumes), re-use HDR draw image as a color target.
        RGImageHandle hDebugColor = hDraw;

        // Create transient depth targets for cascaded shadow maps (even if RT-only / disabled, to keep descriptors stable)
        const VkExtent2D shadowExtent{
            static_cast<uint32_t>(kShadowMapResolution),
            static_cast<uint32_t>(kShadowMapResolution)};
        std::array<RGImageHandle, kShadowCascadeCount> hShadowCascades{};
        for (int i = 0; i < kShadowCascadeCount; ++i)
        {
            std::string name = std::string("shadow.cascade.") + std::to_string(i);
            hShadowCascades[i] = _renderGraph->create_depth_image(name.c_str(), shadowExtent, VK_FORMAT_D32_SFLOAT);
        }

        // Prior to building passes, pump texture loads for this frame.
        if (_textureCache)
        {
            size_t budget = query_texture_budget_bytes();
            _textureCache->set_gpu_budget_bytes(budget);
            _textureCache->evictToBudget(budget);
            _textureCache->pumpLoads(*_resourceManager, get_current_frame());
        }

        // Allow passes to enqueue texture/image uploads before the upload pass snapshot.
        // Particles use this to preload flipbooks/noise referenced by systems.
        if (_renderPassManager)
        {
            if (auto *particles = _renderPassManager->getPass<ParticlePass>())
            {
                particles->preload_needed_textures();
            }
        }

        _resourceManager->register_upload_pass(*_renderGraph, get_current_frame());

        ImGuiPass *imguiPass = nullptr;
        RGImageHandle finalColor = hDraw; // by default, present HDR draw directly (copy)

        if (_renderPassManager)
        {
            if (auto *background = _renderPassManager->getPass<BackgroundPass>())
            {
                background->register_graph(_renderGraph.get(), hDraw, hDepth);
            }
            if (_context->shadowSettings.enabled && _context->shadowSettings.mode != 2u)
            {
                if (auto *shadow = _renderPassManager->getPass<ShadowPass>())
                {
                    shadow->register_graph(_renderGraph.get(), std::span<RGImageHandle>(hShadowCascades.data(), hShadowCascades.size()), shadowExtent);
                }
            }
            if (auto *geometry = _renderPassManager->getPass<GeometryPass>())
            {
                RGImageHandle hID = _renderGraph->import_id_buffer();
                geometry->register_graph(_renderGraph.get(), hGBufferPosition, hGBufferNormal, hGBufferAlbedo, hGBufferExtra, hID, hDepth);

                if (_picking && _swapchainManager)
                {
                    _picking->register_id_buffer_readback(*_renderGraph,
                                                          hID,
                                                          _drawExtent,
                                                          _swapchainManager->swapchainExtent());
                }
            }
            if (auto *lighting = _renderPassManager->getPass<LightingPass>())
            {
                lighting->register_graph(_renderGraph.get(), hDraw, hGBufferPosition, hGBufferNormal, hGBufferAlbedo, hGBufferExtra,
                                         std::span<RGImageHandle>(hShadowCascades.data(), hShadowCascades.size()));
            }

            // Optional Screen Space Reflections pass: consumes HDR draw + G-Buffer and
            // produces an SSR-augmented HDR image. Controlled by EngineContext::enableSSR.
            RGImageHandle hSSR{};
            SSRPass *ssr = _renderPassManager->getPass<SSRPass>();
            const bool ssrEnabled = (_context && _context->enableSSR && ssr != nullptr);
            if (ssrEnabled)
            {
                RGImageDesc ssrDesc{};
                ssrDesc.name = "hdr.ssr";
                ssrDesc.format = _swapchainManager->drawImage().imageFormat;
                ssrDesc.extent = _drawExtent;
                ssrDesc.usage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT
                                | VK_IMAGE_USAGE_SAMPLED_BIT
                                | VK_IMAGE_USAGE_STORAGE_BIT;
                hSSR = _renderGraph->create_image(ssrDesc);

                ssr->register_graph(_renderGraph.get(),
                                    hDraw,
                                    hGBufferPosition,
                                    hGBufferNormal,
                                    hGBufferAlbedo,
                                    hSSR);
            }

            // Downstream passes draw on top of either the SSR output or the raw HDR draw.
            RGImageHandle hdrTarget = (ssrEnabled && hSSR.valid()) ? hSSR : hDraw;

            // Optional voxel volumetrics pass: reads hdrTarget + gbufferPosition and outputs a new HDR target.
            if (_context && _context->enableVolumetrics)
            {
                if (auto *clouds = _renderPassManager->getPass<CloudPass>())
                {
                    hdrTarget = clouds->register_graph(_renderGraph.get(), hdrTarget, hGBufferPosition);
                }
            }

            if (auto *particles = _renderPassManager->getPass<ParticlePass>())
            {
                particles->register_graph(_renderGraph.get(), hdrTarget, hDepth, hGBufferPosition);
            }

            if (auto *transparent = _renderPassManager->getPass<TransparentPass>())
            {
                transparent->register_graph(_renderGraph.get(), hdrTarget, hDepth);
            }
            imguiPass = _renderPassManager->getImGuiPass();

            // Emit per-frame debug primitives (lights/particles/volumes/picking BVH) into the debug draw system.
            if (_context && _context->debug_draw && _context->debug_draw->settings().enabled && _sceneManager)
            {
                DebugDrawSystem *dd = _context->debug_draw;
                SceneManager *scene = _sceneManager.get();
                const WorldVec3 origin_world = scene ? scene->get_world_origin() : WorldVec3{0.0, 0.0, 0.0};

                const uint32_t layer_mask = dd->settings().layer_mask;
                auto layer_on = [layer_mask](DebugDrawLayer layer) {
                    return (layer_mask & static_cast<uint32_t>(layer)) != 0u;
                };

                // Picking: BVH root bounds + picked surface bounds (if available)
                if (layer_on(DebugDrawLayer::Picking) && _picking && _picking->debug_draw_bvh())
                {
                    const auto &pick = _picking->last_pick();
                    if (pick.valid && pick.mesh)
                    {
                        const glm::mat4 &M = pick.worldTransform;
                        auto obb_from_local_aabb = [&](const glm::vec3 &center_local, const glm::vec3 &half_extents) {
                            const glm::vec3 c = center_local;
                            const glm::vec3 e = glm::max(half_extents, glm::vec3(0.0f));
                            const glm::vec3 corners_local[8] = {
                                c + glm::vec3(-e.x, -e.y, -e.z),
                                c + glm::vec3(+e.x, -e.y, -e.z),
                                c + glm::vec3(-e.x, +e.y, -e.z),
                                c + glm::vec3(+e.x, +e.y, -e.z),
                                c + glm::vec3(-e.x, -e.y, +e.z),
                                c + glm::vec3(+e.x, -e.y, +e.z),
                                c + glm::vec3(-e.x, +e.y, +e.z),
                                c + glm::vec3(+e.x, +e.y, +e.z),
                            };
                            std::array<WorldVec3, 8> corners_world{};
                            for (int i = 0; i < 8; ++i)
                            {
                                const glm::vec3 p_local = glm::vec3(M * glm::vec4(corners_local[i], 1.0f));
                                corners_world[i] = local_to_world(p_local, origin_world);
                            }
                            return corners_world;
                        };

                        if (pick.surfaceIndex < pick.mesh->surfaces.size())
                        {
                            const Bounds &b = pick.mesh->surfaces[pick.surfaceIndex].bounds;
                            dd->add_obb_corners(obb_from_local_aabb(b.origin, b.extents),
                                                glm::vec4(1.0f, 1.0f, 0.0f, 0.75f),
                                                0.0f,
                                                DebugDepth::AlwaysOnTop,
                                                DebugDrawLayer::Picking);
                        }

                        if (pick.mesh->bvh && !pick.mesh->bvh->nodes.empty())
                        {
                            const auto &root = pick.mesh->bvh->nodes[0];
                            const glm::vec3 bmin(root.bounds.min.x, root.bounds.min.y, root.bounds.min.z);
                            const glm::vec3 bmax(root.bounds.max.x, root.bounds.max.y, root.bounds.max.z);
                            const glm::vec3 c = (bmin + bmax) * 0.5f;
                            const glm::vec3 e = (bmax - bmin) * 0.5f;
                            dd->add_obb_corners(obb_from_local_aabb(c, e),
                                                glm::vec4(0.0f, 1.0f, 1.0f, 0.75f),
                                                0.0f,
                                                DebugDepth::AlwaysOnTop,
                                                DebugDrawLayer::Picking);
                        }
                    }
                }

                // Lights: spheres + spot cones
                if (scene && layer_on(DebugDrawLayer::Lights))
                {
                    for (const auto &pl : scene->getPointLights())
                    {
                        dd->add_sphere(pl.position_world,
                                       pl.radius,
                                       glm::vec4(pl.color, 0.35f),
                                       0.0f,
                                       DebugDepth::AlwaysOnTop,
                                       DebugDrawLayer::Lights);
                    }
                    for (const auto &sl : scene->getSpotLights())
                    {
                        dd->add_cone(sl.position_world,
                                     glm::dvec3(sl.direction),
                                     sl.radius,
                                     sl.outer_angle_deg,
                                     glm::vec4(sl.color, 0.35f),
                                     0.0f,
                                     DebugDepth::AlwaysOnTop,
                                     DebugDrawLayer::Lights);
                        dd->add_sphere(sl.position_world,
                                       0.15f,
                                       glm::vec4(sl.color, 0.9f),
                                       0.0f,
                                       DebugDepth::AlwaysOnTop,
                                       DebugDrawLayer::Lights);
                    }
                }

                // Particles: emitter + spawn radius + emission cone
                if (layer_on(DebugDrawLayer::Particles))
                {
                    if (auto *particles = _renderPassManager->getPass<ParticlePass>())
                    {
                        for (const auto &sys : particles->systems())
                        {
                            if (!sys.enabled || sys.count == 0) continue;

                            const WorldVec3 emitter_world = local_to_world(sys.params.emitter_pos_local, origin_world);
                            glm::vec4 c = sys.params.color;
                            c.a = 0.5f;

                            dd->add_sphere(emitter_world,
                                           std::max(0.05f, sys.params.spawn_radius * 0.5f),
                                           c,
                                           0.0f,
                                           DebugDepth::AlwaysOnTop,
                                           DebugDrawLayer::Particles);

                            dd->add_circle(emitter_world,
                                           glm::dvec3(sys.params.emitter_dir_local),
                                           sys.params.spawn_radius,
                                           glm::vec4(1.0f, 0.6f, 0.1f, 0.35f),
                                           0.0f,
                                           DebugDepth::AlwaysOnTop,
                                           DebugDrawLayer::Particles);

                            dd->add_cone(emitter_world,
                                         glm::dvec3(sys.params.emitter_dir_local),
                                         std::max(0.5f, sys.params.spawn_radius * 3.0f),
                                         sys.params.cone_angle_degrees,
                                         glm::vec4(1.0f, 0.6f, 0.1f, 0.35f),
                                         0.0f,
                                         DebugDepth::AlwaysOnTop,
                                         DebugDrawLayer::Particles);
                        }
                    }
                }

                // Volumetrics: volume AABB + wind vector
                if (scene && _context->enableVolumetrics && layer_on(DebugDrawLayer::Volumetrics))
                {
                    const glm::vec3 cam_local = scene->get_camera_local_position();
                    for (uint32_t i = 0; i < EngineContext::MAX_VOXEL_VOLUMES; ++i)
                    {
                        const auto &vs = _context->voxelVolumes[i];
                        if (!vs.enabled) continue;

                        glm::vec3 center_local = vs.volumeCenterLocal;
                        if (vs.followCameraXZ)
                        {
                            center_local.x += cam_local.x;
                            center_local.z += cam_local.z;
                        }

                        const WorldVec3 center_world = local_to_world(center_local, origin_world);
                        dd->add_aabb(center_world,
                                     vs.volumeHalfExtents,
                                     glm::vec4(0.4f, 0.8f, 1.0f, 0.35f),
                                     0.0f,
                                     DebugDepth::AlwaysOnTop,
                                     DebugDrawLayer::Volumetrics);

                        const float wind_len = glm::length(vs.windVelocityLocal);
                        if (std::isfinite(wind_len) && wind_len > 1.0e-4f)
                        {
                            dd->add_ray(center_world,
                                        glm::dvec3(vs.windVelocityLocal),
                                        std::clamp(wind_len, 0.5f, 25.0f),
                                        glm::vec4(0.2f, 1.0f, 0.2f, 0.9f),
                                        0.0f,
                                        DebugDepth::AlwaysOnTop,
                                        DebugDrawLayer::Volumetrics);
                        }
                    }
                }
            }

            // Optional Tonemap pass: sample HDR draw -> LDR intermediate
            if (auto *tonemap = _renderPassManager->getPass<TonemapPass>())
            {
                finalColor = tonemap->register_graph(_renderGraph.get(), hdrTarget);

                // Optional FXAA pass: runs on LDR tonemapped output.
                if (auto *fxaa = _renderPassManager->getPass<FxaaPass>())
                {
                    finalColor = fxaa->register_graph(_renderGraph.get(), finalColor);
                }

                // Debug lines after tonemap/FXAA so they don't trigger bloom.
                if (auto *debugDraw = _renderPassManager->getPass<DebugDrawPass>())
                {
                    debugDraw->register_graph(_renderGraph.get(), finalColor, hDepth, true /*LDR*/);
                }
            }
            else
            {
                // If tonemapping is disabled, present whichever HDR buffer we ended up with.
                finalColor = hdrTarget;

                // Debug lines onto HDR target when tonemapping is disabled.
                if (auto *debugDraw = _renderPassManager->getPass<DebugDrawPass>())
                {
                    debugDraw->register_graph(_renderGraph.get(), finalColor, hDepth, false /*HDR*/);
                }
            }
        }

        auto appendPresentExtras = [imguiPass, hSwapchain](RenderGraph &graph)
        {
            if (imguiPass)
            {
                imguiPass->register_graph(&graph, hSwapchain);
            }
        };

        _renderGraph->add_present_chain(finalColor, hSwapchain, appendPresentExtras);

        // Apply persistent pass enable overrides
        for (size_t i = 0; i < _renderGraph->pass_count(); ++i)
        {
            const char* name = _renderGraph->pass_name(i);
            auto it = _rgPassToggles.find(name);
            if (it != _rgPassToggles.end())
            {
                _renderGraph->set_pass_enabled(i, it->second);
            }
        }

        if (_renderGraph->compile())
        {
            _renderGraph->execute(cmd);
        }
    }

    VK_CHECK(vkEndCommandBuffer(cmd));

    VkCommandBufferSubmitInfo cmdinfo = vkinit::command_buffer_submit_info(cmd);

    VkSemaphoreSubmitInfo waitInfo = vkinit::semaphore_submit_info(VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT_KHR,
                                                                   get_current_frame()._swapchainSemaphore);
    VkSemaphoreSubmitInfo signalInfo = vkinit::semaphore_submit_info(VK_PIPELINE_STAGE_2_ALL_GRAPHICS_BIT,
                                                                     get_current_frame()._renderSemaphore);

    VkSubmitInfo2 submit = vkinit::submit_info(&cmdinfo, &signalInfo, &waitInfo);
    //------------
    VK_CHECK(vkQueueSubmit2(_deviceManager->graphicsQueue(), 1, &submit, get_current_frame()._renderFence));

    VkPresentInfoKHR presentInfo = vkinit::present_info();

    VkSwapchainKHR swapchain = _swapchainManager->swapchain();
    presentInfo.pSwapchains = &swapchain;
    presentInfo.swapchainCount = 1;

    presentInfo.pWaitSemaphores = &get_current_frame()._renderSemaphore;
    presentInfo.waitSemaphoreCount = 1;

    presentInfo.pImageIndices = &swapchainImageIndex;

    VkResult presentResult = vkQueuePresentKHR(_deviceManager->graphicsQueue(), &presentInfo);
    if (_swapchainManager)
    {
        _swapchainManager->set_swapchain_image_layout(swapchainImageIndex, VK_IMAGE_LAYOUT_PRESENT_SRC_KHR);
    }
    if (presentResult == VK_ERROR_OUT_OF_DATE_KHR || presentResult == VK_SUBOPTIMAL_KHR)
    {
        resize_requested = true;
    }

    _frameNumber++;
}

namespace
{
    struct NativeEventDispatchCtx
    {
        VulkanEngine *engine = nullptr;
        bool ui_capture_mouse = false;
    };

    void dispatch_native_event(void *user, InputSystem::NativeEventView view)
    {
        auto *ctx = static_cast<NativeEventDispatchCtx *>(user);
        if (!ctx || !ctx->engine || view.backend != InputSystem::NativeBackend::SDL2 || view.data == nullptr)
        {
            return;
        }

        const SDL_Event &e = *static_cast<const SDL_Event *>(view.data);

        if (ctx->engine->ui())
        {
            ctx->engine->ui()->process_event(e);
        }
        if (ctx->engine->picking())
        {
            ctx->engine->picking()->process_event(e, ctx->ui_capture_mouse);
        }
    }
} // namespace

void VulkanEngine::run()
{
    bool bQuit = false;

    //main loop
    while (!bQuit)
    {
        auto start = std::chrono::system_clock::now();

        if (_input)
        {
            _input->begin_frame();
            _input->pump_events();

            if (_input->quit_requested())
            {
                bQuit = true;
            }

            freeze_rendering = _input->window_minimized();

            if (_input->resize_requested())
            {
                resize_requested = true;
                _last_resize_event_ms = _input->last_resize_event_ms();
                _input->clear_resize_request();
            }
        }

        const bool ui_capture_mouse = _ui && _ui->want_capture_mouse();
        const bool ui_capture_keyboard = _ui && _ui->want_capture_keyboard();

        if (_input)
        {
            NativeEventDispatchCtx ctx{};
            ctx.engine = this;
            ctx.ui_capture_mouse = ui_capture_mouse;
            _input->for_each_native_event(dispatch_native_event, &ctx);
        }

        if (_sceneManager && _input)
        {
            _sceneManager->getMainCamera().process_input(*_input, ui_capture_keyboard, ui_capture_mouse);
        }

        if (freeze_rendering)
        {
            //throttle the speed to avoid the endless spinning
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        if (resize_requested)
        {
            const uint32_t now_ms = SDL_GetTicks();
            if (now_ms - _last_resize_event_ms >= RESIZE_DEBOUNCE_MS)
            {
                _swapchainManager->resize_swapchain(_window);
                if (_ui)
                {
                    _ui->on_swapchain_recreated();
                }
                resize_requested = false;
            }
        }

        // Begin frame: wait for the GPU, resolve pending ID-buffer picks,
        // and clear per-frame resources before building UI and recording commands.
        VK_CHECK(vkWaitForFences(_deviceManager->device(), 1, &get_current_frame()._renderFence, true, 1000000000));

        // Safe to destroy any BLAS queued for deletion now that the previous frame is idle.
        if (_rayManager) { _rayManager->flushPendingDeletes(); }

        // Progress queued BLAS builds over multiple frames to avoid large
        // stalls when many meshes require ray tracing structures at once.
        if (_rayManager) { _rayManager->pump_blas_builds(1); }

        // Commit any completed async IBL load now that the GPU is idle.
        if (_iblManager && _pendingIBLRequest.active)
        {
            IBLManager::AsyncResult iblRes = _iblManager->pump_async();
            if (iblRes.completed)
            {
                if (iblRes.success)
                {
                    if (_pendingIBLRequest.targetVolume >= 0)
                    {
                        _activeIBLVolume = _pendingIBLRequest.targetVolume;
                    }
                    else
                    {
                        _activeIBLVolume = -1;
                        _hasGlobalIBL = true;
                    }
                }
                else
                {
                    fmt::println("[Engine] Warning: async IBL load failed (specular='{}')",
                                 _pendingIBLRequest.paths.specularCube);
                }
                _pendingIBLRequest.active = false;
            }
        }

        if (_picking)
        {
            _picking->begin_frame();
        }

        get_current_frame()._deletionQueue.flush();
        if (_renderGraph)
        {
            _renderGraph->resolve_timings();
        }
        get_current_frame()._frameDescriptors.clear_pools(_deviceManager->device());

        if (_ui)
        {
            _ui->begin_frame();
            _ui->end_frame();
        }
        draw();

        auto end = std::chrono::system_clock::now();

        //convert to microseconds (integer), and then come back to miliseconds
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        stats.frametime = elapsed.count() / 1000.f;
    }
}

void VulkanEngine::init_frame_resources()
{
    // descriptor pool sizes per-frame
    std::vector<DescriptorAllocatorGrowable::PoolSizeRatio> frame_sizes = {
        {VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 3},
        {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 3},
        {VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 3},
        {VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 4},
        {VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR, 1},
    };

    for (int i = 0; i < FRAME_OVERLAP; i++)
    {
        _frames[i].init(_deviceManager.get(), frame_sizes);
    }
}

void VulkanEngine::init_pipelines()
{
    metalRoughMaterial.build_pipelines(this);
}

namespace
{
    // Rebuild a node's world transform in glTF local space, layering per-instance
    // local offsets on top of the base localTransform at each node in the chain.
    glm::mat4 build_node_world_with_overrides(const Node *node,
                                              const std::unordered_map<const Node*, glm::mat4> &overrides)
    {
        if (!node)
        {
            return glm::mat4(1.0f);
        }

        std::vector<const Node*> chain;
        const Node *cur = node;
        while (cur)
        {
            chain.push_back(cur);
            std::shared_ptr<Node> parent = cur->parent.lock();
            cur = parent ? parent.get() : nullptr;
        }

        glm::mat4 world(1.0f);
        for (auto it = chain.rbegin(); it != chain.rend(); ++it)
        {
            const Node *n = *it;
            glm::mat4 local = n->localTransform;
            auto ovIt = overrides.find(n);
            if (ovIt != overrides.end())
            {
                // Layer the override in local space for this instance.
                local = local * ovIt->second;
            }
            world = world * local;
        }
        return world;
    }
}

void MeshNode::Draw(const glm::mat4 &topMatrix, DrawContext &ctx)
{
    glm::mat4 nodeMatrix;
    if (ctx.gltfNodeLocalOverrides && !ctx.gltfNodeLocalOverrides->empty())
    {
        glm::mat4 world = build_node_world_with_overrides(this, *ctx.gltfNodeLocalOverrides);
        nodeMatrix = topMatrix * world;
    }
    else
    {
        nodeMatrix = topMatrix * worldTransform;
    }

    if (!mesh)
    {
        Node::Draw(topMatrix, ctx);
        return;
    }

    for (uint32_t i = 0; i < mesh->surfaces.size(); ++i)
    {
        const auto &s = mesh->surfaces[i];

        RenderObject def{};
        def.indexCount = s.count;
        def.firstIndex = s.startIndex;
        def.indexBuffer = mesh->meshBuffers.indexBuffer.buffer;
        def.vertexBuffer = mesh->meshBuffers.vertexBuffer.buffer;
        def.bounds = s.bounds; // ensure culling uses correct mesh-local AABB
        def.material = &s.material->data;

        def.transform = nodeMatrix;
        def.vertexBufferAddress = mesh->meshBuffers.vertexBufferAddress;
        def.sourceMesh = mesh.get();
        def.surfaceIndex = i;
        def.objectID = ctx.nextID++;
        def.sourceScene = scene;
        def.sourceNode = this;

        if (s.material->data.passType == MaterialPass::Transparent)
        {
            ctx.TransparentSurfaces.push_back(def);
        }
        else
        {
            ctx.OpaqueSurfaces.push_back(def);
        }
    }

    // recurse down
    Node::Draw(topMatrix, ctx);
}
