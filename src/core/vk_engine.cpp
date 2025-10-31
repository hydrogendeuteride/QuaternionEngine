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
#include "vk_engine.h"
#include <core/vk_images.h>

#include "SDL2/SDL.h"
#include "SDL2/SDL_vulkan.h"

#include <core/vk_initializers.h>
#include <core/vk_types.h>

#include "VkBootstrap.h"

#include <chrono>
#include <thread>
#include <span>
#include <array>

#include "render/vk_pipelines.h"
#include <iostream>
#include <glm/gtx/transform.hpp>

#include "config.h"
#include "render/primitives.h"

#include "vk_mem_alloc.h"
#include "imgui.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_vulkan.h"
#include "render/vk_renderpass_geometry.h"
#include "render/vk_renderpass_imgui.h"
#include "render/vk_renderpass_lighting.h"
#include "render/vk_renderpass_transparent.h"
#include "render/vk_renderpass_tonemap.h"
#include "render/vk_renderpass_shadow.h"
#include "vk_resource.h"
#include "engine_context.h"
#include "core/vk_pipeline_manager.h"
#include "core/config.h"

VulkanEngine *loadedEngine = nullptr;

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

void VulkanEngine::init()
{
    // We initialize SDL and create a window with it.
    SDL_Init(SDL_INIT_VIDEO);

    constexpr auto window_flags = static_cast<SDL_WindowFlags>(SDL_WINDOW_VULKAN | SDL_WINDOW_RESIZABLE);

    _swapchainManager = std::make_unique<SwapchainManager>();

    _window = SDL_CreateWindow(
        "Vulkan Engine",
        SDL_WINDOWPOS_UNDEFINED,
        SDL_WINDOWPOS_UNDEFINED,
        _swapchainManager->windowExtent().width,
        _swapchainManager->windowExtent().height,
        window_flags
    );

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

    _swapchainManager->init(_deviceManager.get(), _resourceManager.get());
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

    // Render graph skeleton
    _renderGraph = std::make_unique<RenderGraph>();
    _renderGraph->init(_context.get());
    _context->renderGraph = _renderGraph.get();

    init_frame_resources();

    // Build material pipelines early so materials can be created
    metalRoughMaterial.build_pipelines(this);

    init_default_data();

    _renderPassManager = std::make_unique<RenderPassManager>();
    _renderPassManager->init(_context.get());

    auto imguiPass = std::make_unique<ImGuiPass>();
    _renderPassManager->setImGuiPass(std::move(imguiPass));

    const std::string structurePath = _assetManager->modelPath("seoul_high.glb");
    const auto structureFile = _assetManager->loadGLTF(structurePath);

    assert(structureFile.has_value());

    _sceneManager->loadScene("structure", *structureFile);

    _resourceManager->set_deferred_uploads(true);

    //everything went fine
    _isInitialized = true;
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

    // Register default primitives as dynamic scene instances
    if (_sceneManager)
    {
        _sceneManager->addMeshInstance("default.cube", cubeMesh,
                                       glm::translate(glm::mat4(1.f), glm::vec3(-2.f, 0.f, -2.f)));
        _sceneManager->addMeshInstance("default.sphere", sphereMesh,
                                       glm::translate(glm::mat4(1.f), glm::vec3(2.f, 0.f, -2.f)));
    }

    _mainDeletionQueue.push_function([&]() {
        _resourceManager->destroy_image(_whiteImage);
        _resourceManager->destroy_image(_greyImage);
        _resourceManager->destroy_image(_blackImage);
        _resourceManager->destroy_image(_errorCheckerboardImage);
    });
    //< default_img
}

void VulkanEngine::cleanup()
{
    vkDeviceWaitIdle(_deviceManager->device());

    print_vma_stats(_deviceManager.get(), "begin");

    _sceneManager->cleanup();
    print_vma_stats(_deviceManager.get(), "after SceneManager");
    dump_vma_json(_deviceManager.get(), "after_SceneManager");

    if (_isInitialized)
    {
        //make sure the gpu has stopped doing its things
        vkDeviceWaitIdle(_deviceManager->device());

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

    _renderPassManager->cleanup();
        print_vma_stats(_deviceManager.get(), "after RenderPassManager");
        dump_vma_json(_deviceManager.get(), "after_RenderPassManager");

        _pipelineManager->cleanup();
        print_vma_stats(_deviceManager.get(), "after PipelineManager");
        dump_vma_json(_deviceManager.get(), "after_PipelineManager");

        compute.cleanup();
        print_vma_stats(_deviceManager.get(), "after Compute");
        dump_vma_json(_deviceManager.get(), "after_Compute");

        _swapchainManager->cleanup();
        print_vma_stats(_deviceManager.get(), "after Swapchain");
        dump_vma_json(_deviceManager.get(), "after_Swapchain");

        if (_assetManager) _assetManager->cleanup();
        print_vma_stats(_deviceManager.get(), "after AssetManager");
        dump_vma_json(_deviceManager.get(), "after_AssetManager");

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

        SDL_DestroyWindow(_window);
    }
}

void VulkanEngine::draw()
{
    _sceneManager->update_scene();
    //> frame_clear
    //wait until the gpu has finished rendering the last frame. Timeout of 1 second
    VK_CHECK(vkWaitForFences(_deviceManager->device(), 1, &get_current_frame()._renderFence, true, 1000000000));

    get_current_frame()._deletionQueue.flush();
    // Resolve last frame's pass timings before we clear and rebuild the graph
    if (_renderGraph)
    {
        _renderGraph->resolve_timings();
    }
    get_current_frame()._frameDescriptors.clear_pools(_deviceManager->device());
    //< frame_clear

    uint32_t swapchainImageIndex;

    VkResult e = vkAcquireNextImageKHR(_deviceManager->device(), _swapchainManager->swapchain(), 1000000000,
                                       get_current_frame()._swapchainSemaphore,
                                       nullptr, &swapchainImageIndex);
    if (e == VK_ERROR_OUT_OF_DATE_KHR)
    {
        resize_requested = true;
        return;
    }

    _drawExtent.height = std::min(_swapchainManager->swapchainExtent().height,
                                  _swapchainManager->drawImage().imageExtent.height) * renderScale;
    _drawExtent.width = std::min(_swapchainManager->swapchainExtent().width,
                                 _swapchainManager->drawImage().imageExtent.width) * renderScale;

    VK_CHECK(vkResetFences(_deviceManager->device(), 1, &get_current_frame()._renderFence));

    //now that we are sure that the commands finished executing, we can safely reset the command buffer to begin recording again.
    VK_CHECK(vkResetCommandBuffer(get_current_frame()._mainCommandBuffer, 0));

    // Build or update TLAS for current frame now that the previous frame is idle
    if (_rayManager && _context->shadowSettings.mode != 0u)
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
    _context->drawExtent = _drawExtent;

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
        RGImageHandle hSwapchain = _renderGraph->import_swapchain_image(swapchainImageIndex);

        // Create transient depth targets for cascaded shadow maps (even if RT-only, to keep descriptors stable)
        const VkExtent2D shadowExtent{2048, 2048};
        std::array<RGImageHandle, kShadowCascadeCount> hShadowCascades{};
        for (int i = 0; i < kShadowCascadeCount; ++i)
        {
            std::string name = std::string("shadow.cascade.") + std::to_string(i);
            hShadowCascades[i] = _renderGraph->create_depth_image(name.c_str(), shadowExtent, VK_FORMAT_D32_SFLOAT);
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
            if (_context->shadowSettings.mode != 2u)
            {
                if (auto *shadow = _renderPassManager->getPass<ShadowPass>())
                {
                    shadow->register_graph(_renderGraph.get(), std::span<RGImageHandle>(hShadowCascades.data(), hShadowCascades.size()), shadowExtent);
                }
            }
            if (auto *geometry = _renderPassManager->getPass<GeometryPass>())
            {
                geometry->register_graph(_renderGraph.get(), hGBufferPosition, hGBufferNormal, hGBufferAlbedo, hDepth);
            }
            if (auto *lighting = _renderPassManager->getPass<LightingPass>())
            {
                lighting->register_graph(_renderGraph.get(), hDraw, hGBufferPosition, hGBufferNormal, hGBufferAlbedo,
                                         std::span<RGImageHandle>(hShadowCascades.data(), hShadowCascades.size()));
            }
            if (auto *transparent = _renderPassManager->getPass<TransparentPass>())
            {
                transparent->register_graph(_renderGraph.get(), hDraw, hDepth);
            }
            imguiPass = _renderPassManager->getImGuiPass();

            // Optional Tonemap pass: sample HDR draw -> LDR intermediate
            if (auto *tonemap = _renderPassManager->getPass<TonemapPass>())
            {
                finalColor = tonemap->register_graph(_renderGraph.get(), hDraw);
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

    VK_CHECK(vkQueueSubmit2(_deviceManager->graphicsQueue(), 1, &submit, get_current_frame()._renderFence));

    VkPresentInfoKHR presentInfo = vkinit::present_info();

    VkSwapchainKHR swapchain = _swapchainManager->swapchain();
    presentInfo.pSwapchains = &swapchain;
    presentInfo.swapchainCount = 1;

    presentInfo.pWaitSemaphores = &get_current_frame()._renderSemaphore;
    presentInfo.waitSemaphoreCount = 1;

    presentInfo.pImageIndices = &swapchainImageIndex;

    VkResult presentResult = vkQueuePresentKHR(_deviceManager->graphicsQueue(), &presentInfo);
    if (presentResult == VK_ERROR_OUT_OF_DATE_KHR)
    {
        resize_requested = true;
    }

    _frameNumber++;
}

void VulkanEngine::run()
{
    SDL_Event e;
    bool bQuit = false;

    //main loop
    while (!bQuit)
    {
        auto start = std::chrono::system_clock::now();
        //Handle events on queue
        while (SDL_PollEvent(&e) != 0)
        {
            //close the window when user alt-f4s or clicks the X button
            if (e.type == SDL_QUIT) bQuit = true;
            if (e.type == SDL_WINDOWEVENT)
            {
                if (e.window.event == SDL_WINDOWEVENT_MINIMIZED)
                {
                    freeze_rendering = true;
                }
                if (e.window.event == SDL_WINDOWEVENT_RESTORED)
                {
                    freeze_rendering = false;
                }
            }
            _sceneManager->getMainCamera().processSDLEvent(e);
            ImGui_ImplSDL2_ProcessEvent(&e);
        }

        if (freeze_rendering)
        {
            //throttle the speed to avoid the endless spinning
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        if (resize_requested)
        {
            _swapchainManager->resize_swapchain(_window);
        }


        // imgui new frame
        ImGui_ImplVulkan_NewFrame();
        ImGui_ImplSDL2_NewFrame();

        ImGui::NewFrame();

        if (ImGui::Begin("background"))
        {
            auto background_pass = _renderPassManager->getPass<BackgroundPass>();
            ComputeEffect &selected = background_pass->_backgroundEffects[background_pass->_currentEffect];

            ImGui::Text("Selected effect: %s", selected.name);

            ImGui::SliderInt("Effect Index", &background_pass->_currentEffect, 0,
                             background_pass->_backgroundEffects.size() - 1);

            ImGui::InputFloat4("data1", reinterpret_cast<float *>(&selected.data.data1));
            ImGui::InputFloat4("data2", reinterpret_cast<float *>(&selected.data.data2));
            ImGui::InputFloat4("data3", reinterpret_cast<float *>(&selected.data.data3));
            ImGui::InputFloat4("data4", reinterpret_cast<float *>(&selected.data.data4));

            ImGui::SliderFloat("Render Scale", &renderScale, 0.3f, 1.f);

            ImGui::End();
        }

        if (ImGui::Begin("Stats"))
        {
            ImGui::Text("frametime %f ms", stats.frametime);
            ImGui::Text("draw time %f ms", stats.mesh_draw_time);
            ImGui::Text("update time %f ms", _sceneManager->stats.scene_update_time);
            ImGui::Text("triangles %i", stats.triangle_count);
            ImGui::Text("draws %i", stats.drawcall_count);
            ImGui::End();
        }

        // Shadows / Ray Query settings
        if (ImGui::Begin("Shadows"))
        {
            const bool rq = _deviceManager->supportsRayQuery();
            const bool as = _deviceManager->supportsAccelerationStructure();
            ImGui::Text("RayQuery: %s", rq ? "supported" : "not available");
            ImGui::Text("AccelStruct: %s", as ? "supported" : "not available");
            ImGui::Separator();

            auto &ss = _context->shadowSettings;
            // Mode selection
            int mode = static_cast<int>(ss.mode);
            ImGui::TextUnformatted("Shadow Mode");
            ImGui::RadioButton("Clipmap only", &mode, 0); ImGui::SameLine();
            ImGui::RadioButton("Clipmap + RT", &mode, 1); ImGui::SameLine();
            ImGui::RadioButton("RT only", &mode, 2);
            // If device lacks RT support, force mode 0
            if (!(rq && as) && mode != 0) mode = 0;
            ss.mode = static_cast<uint32_t>(mode);
            ss.hybridRayQueryEnabled = (ss.mode != 0);

            // Hybrid controls (mode 1)
            ImGui::BeginDisabled(ss.mode != 1u);
            ImGui::TextUnformatted("Cascades using ray assist:");
            for (int i = 0; i < 4; ++i)
            {
                bool on = (ss.hybridRayCascadesMask >> i) & 1u;
                std::string label = std::string("C") + std::to_string(i);
                if (ImGui::Checkbox(label.c_str(), &on))
                {
                    if (on) ss.hybridRayCascadesMask |= (1u << i);
                    else    ss.hybridRayCascadesMask &= ~(1u << i);
                }
                if (i != 3) ImGui::SameLine();
            }
            ImGui::SliderFloat("N·L threshold", &ss.hybridRayNoLThreshold, 0.0f, 1.0f, "%.2f");
            ImGui::EndDisabled();

            ImGui::Separator();
            ImGui::TextWrapped("Clipmap only: raster PCF+RPDB. Clipmap+RT: PCF assisted by ray query at low N·L. RT only: skip shadow maps and use ray tests only.");

            ImGui::End();
        }

        // Render Graph debug window
        if (ImGui::Begin("Render Graph"))
        {
            if (_renderGraph)
            {
                auto &graph = *_renderGraph;
                std::vector<RenderGraph::RGDebugPassInfo> passInfos;
                graph.debug_get_passes(passInfos);
                if (ImGui::Button("Reload Pipelines")) { _pipelineManager->hotReloadChanged(); }
                ImGui::SameLine();
                ImGui::Text("%zu passes", passInfos.size());

                if (ImGui::BeginTable("passes", 8, ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp))
                {
                    ImGui::TableSetupColumn("Enable", ImGuiTableColumnFlags_WidthFixed, 70);
                    ImGui::TableSetupColumn("Name");
                    ImGui::TableSetupColumn("Type", ImGuiTableColumnFlags_WidthFixed, 80);
                    ImGui::TableSetupColumn("GPU ms", ImGuiTableColumnFlags_WidthFixed, 70);
                    ImGui::TableSetupColumn("CPU rec ms", ImGuiTableColumnFlags_WidthFixed, 90);
                    ImGui::TableSetupColumn("Imgs", ImGuiTableColumnFlags_WidthFixed, 55);
                    ImGui::TableSetupColumn("Bufs", ImGuiTableColumnFlags_WidthFixed, 55);
                    ImGui::TableSetupColumn("Attachments", ImGuiTableColumnFlags_WidthFixed, 100);
                    ImGui::TableHeadersRow();

                    auto typeName = [](RGPassType t){
                        switch (t) {
                            case RGPassType::Graphics: return "Graphics";
                            case RGPassType::Compute:  return "Compute";
                            case RGPassType::Transfer: return "Transfer";
                            default: return "?";
                        }
                    };

                    for (size_t i = 0; i < passInfos.size(); ++i)
                    {
                        auto &pi = passInfos[i];
                        ImGui::TableNextRow();
                        ImGui::TableSetColumnIndex(0);
                        bool enabled = true;
                        if (auto it = _rgPassToggles.find(pi.name); it != _rgPassToggles.end()) enabled = it->second;
                        std::string chkId = std::string("##en") + std::to_string(i);
                        if (ImGui::Checkbox(chkId.c_str(), &enabled))
                        {
                            _rgPassToggles[pi.name] = enabled;
                        }
                        ImGui::TableSetColumnIndex(1);
                        ImGui::TextUnformatted(pi.name.c_str());
                        ImGui::TableSetColumnIndex(2);
                        ImGui::TextUnformatted(typeName(pi.type));
                        ImGui::TableSetColumnIndex(3);
                        if (pi.gpuMillis >= 0.0f) ImGui::Text("%.2f", pi.gpuMillis); else ImGui::TextUnformatted("-");
                        ImGui::TableSetColumnIndex(4);
                        if (pi.cpuMillis >= 0.0f) ImGui::Text("%.2f", pi.cpuMillis); else ImGui::TextUnformatted("-");
                        ImGui::TableSetColumnIndex(5);
                        ImGui::Text("%u/%u", pi.imageReads, pi.imageWrites);
                        ImGui::TableSetColumnIndex(6);
                        ImGui::Text("%u/%u", pi.bufferReads, pi.bufferWrites);
                        ImGui::TableSetColumnIndex(7);
                        ImGui::Text("%u%s", pi.colorAttachmentCount, pi.hasDepth ? "+D" : "");
                    }
                    ImGui::EndTable();
                }

                if (ImGui::CollapsingHeader("Images", ImGuiTreeNodeFlags_DefaultOpen))
                {
                    std::vector<RenderGraph::RGDebugImageInfo> imgs;
                    graph.debug_get_images(imgs);
                    if (ImGui::BeginTable("images", 7, ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp))
                    {
                        ImGui::TableSetupColumn("Id", ImGuiTableColumnFlags_WidthFixed, 40);
                        ImGui::TableSetupColumn("Name");
                        ImGui::TableSetupColumn("Fmt", ImGuiTableColumnFlags_WidthFixed, 120);
                        ImGui::TableSetupColumn("Extent", ImGuiTableColumnFlags_WidthFixed, 120);
                        ImGui::TableSetupColumn("Imported", ImGuiTableColumnFlags_WidthFixed, 70);
                        ImGui::TableSetupColumn("Usage", ImGuiTableColumnFlags_WidthFixed, 80);
                        ImGui::TableSetupColumn("Life", ImGuiTableColumnFlags_WidthFixed, 80);
                        ImGui::TableHeadersRow();
                        for (const auto &im : imgs)
                        {
                            ImGui::TableNextRow();
                            ImGui::TableSetColumnIndex(0); ImGui::Text("%u", im.id);
                            ImGui::TableSetColumnIndex(1); ImGui::TextUnformatted(im.name.c_str());
                            ImGui::TableSetColumnIndex(2); ImGui::TextUnformatted(string_VkFormat(im.format));
                            ImGui::TableSetColumnIndex(3); ImGui::Text("%ux%u", im.extent.width, im.extent.height);
                            ImGui::TableSetColumnIndex(4); ImGui::TextUnformatted(im.imported ? "yes" : "no");
                            ImGui::TableSetColumnIndex(5); ImGui::Text("0x%x", (unsigned)im.creationUsage);
                            ImGui::TableSetColumnIndex(6); ImGui::Text("%d..%d", im.firstUse, im.lastUse);
                        }
                        ImGui::EndTable();
                    }
                }

                if (ImGui::CollapsingHeader("Buffers"))
                {
                    std::vector<RenderGraph::RGDebugBufferInfo> bufs;
                    graph.debug_get_buffers(bufs);
                    if (ImGui::BeginTable("buffers", 6, ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp))
                    {
                        ImGui::TableSetupColumn("Id", ImGuiTableColumnFlags_WidthFixed, 40);
                        ImGui::TableSetupColumn("Name");
                        ImGui::TableSetupColumn("Size", ImGuiTableColumnFlags_WidthFixed, 100);
                        ImGui::TableSetupColumn("Imported", ImGuiTableColumnFlags_WidthFixed, 70);
                        ImGui::TableSetupColumn("Usage", ImGuiTableColumnFlags_WidthFixed, 100);
                        ImGui::TableSetupColumn("Life", ImGuiTableColumnFlags_WidthFixed, 80);
                        ImGui::TableHeadersRow();
                        for (const auto &bf : bufs)
                        {
                            ImGui::TableNextRow();
                            ImGui::TableSetColumnIndex(0); ImGui::Text("%u", bf.id);
                            ImGui::TableSetColumnIndex(1); ImGui::TextUnformatted(bf.name.c_str());
                            ImGui::TableSetColumnIndex(2); ImGui::Text("%zu", (size_t)bf.size);
                            ImGui::TableSetColumnIndex(3); ImGui::TextUnformatted(bf.imported ? "yes" : "no");
                            ImGui::TableSetColumnIndex(4); ImGui::Text("0x%x", (unsigned)bf.usage);
                            ImGui::TableSetColumnIndex(5); ImGui::Text("%d..%d", bf.firstUse, bf.lastUse);
                        }
                        ImGui::EndTable();
                    }
                }
            }
            ImGui::End();
        }

        // Pipelines debug window (graphics)
        if (ImGui::Begin("Pipelines"))
        {
            if (_pipelineManager)
            {
                std::vector<PipelineManager::GraphicsPipelineDebugInfo> pipes;
                _pipelineManager->debug_get_graphics(pipes);
                if (ImGui::Button("Reload Changed")) { _pipelineManager->hotReloadChanged(); }
                ImGui::SameLine(); ImGui::Text("%zu graphics pipelines", pipes.size());
                if (ImGui::BeginTable("gfxpipes", 5, ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp))
                {
                    ImGui::TableSetupColumn("Name");
                    ImGui::TableSetupColumn("VS");
                    ImGui::TableSetupColumn("FS");
                    ImGui::TableSetupColumn("Valid", ImGuiTableColumnFlags_WidthFixed, 60);
                    ImGui::TableHeadersRow();
                    for (const auto &p : pipes)
                    {
                        ImGui::TableNextRow();
                        ImGui::TableSetColumnIndex(0); ImGui::TextUnformatted(p.name.c_str());
                        ImGui::TableSetColumnIndex(1); ImGui::TextUnformatted(p.vertexShaderPath.c_str());
                        ImGui::TableSetColumnIndex(2); ImGui::TextUnformatted(p.fragmentShaderPath.c_str());
                        ImGui::TableSetColumnIndex(3); ImGui::TextUnformatted(p.valid ? "yes" : "no");
                    }
                    ImGui::EndTable();
                }
            }
            ImGui::End();
        }

    // Draw targets window
        if (ImGui::Begin("Targets"))
        {
            ImGui::Text("Draw extent: %ux%u", _drawExtent.width, _drawExtent.height);
            auto scExt = _swapchainManager->swapchainExtent();
            ImGui::Text("Swapchain:   %ux%u", scExt.width, scExt.height);
            ImGui::Text("Draw fmt:    %s", string_VkFormat(_swapchainManager->drawImage().imageFormat));
            ImGui::Text("Swap fmt:    %s", string_VkFormat(_swapchainManager->swapchainImageFormat()));
            ImGui::End();
        }

        // PostFX window
        if (ImGui::Begin("PostFX"))
        {
            if (auto *tm = _renderPassManager->getPass<TonemapPass>())
            {
                float exp = tm->exposure();
                int mode = tm->mode();
                if (ImGui::SliderFloat("Exposure", &exp, 0.05f, 8.0f)) { tm->setExposure(exp); }
                ImGui::TextUnformatted("Operator");
                ImGui::SameLine();
                if (ImGui::RadioButton("Reinhard", mode == 0)) { mode = 0; tm->setMode(mode); }
                ImGui::SameLine();
                if (ImGui::RadioButton("ACES", mode == 1)) { mode = 1; tm->setMode(mode); }
            }
            else
            {
                ImGui::TextUnformatted("Tonemap pass not available");
            }
            ImGui::End();
        }

        // Scene window
        if (ImGui::Begin("Scene"))
        {
            const DrawContext &dc = _context->getMainDrawContext();
            ImGui::Text("Opaque draws: %zu", dc.OpaqueSurfaces.size());
            ImGui::Text("Transp draws: %zu", dc.TransparentSurfaces.size());
            ImGui::End();
        }
        ImGui::Render();
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

void MeshNode::Draw(const glm::mat4 &topMatrix, DrawContext &ctx)
{
    glm::mat4 nodeMatrix = topMatrix * worldTransform;

    for (auto &s: mesh->surfaces)
    {
        RenderObject def{};
        def.indexCount = s.count;
        def.firstIndex = s.startIndex;
        def.indexBuffer = mesh->meshBuffers.indexBuffer.buffer;
        def.vertexBuffer = mesh->meshBuffers.vertexBuffer.buffer;
        def.bounds = s.bounds; // ensure culling uses correct mesh-local AABB
        def.material = &s.material->data;

        def.transform = nodeMatrix;
        def.vertexBufferAddress = mesh->meshBuffers.vertexBufferAddress;

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
