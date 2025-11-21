// ImGui debug UI helpers for VulkanEngine.
//
// This file contains the immediate-mode ImGui widgets that expose engine
// statistics, render-graph inspection, texture streaming controls, etc.
// The main frame loop in vk_engine.cpp simply calls vk_engine_draw_debug_ui().

#include "vk_engine.h"

#include "imgui.h"
#include "ImGuizmo.h"

#include "render/primitives.h"
#include "vk_mem_alloc.h"
#include "render/vk_renderpass_tonemap.h"
#include "render/vk_renderpass_background.h"
#include <glm/gtx/euler_angles.hpp>
#include "render/rg_graph.h"
#include "core/vk_pipeline_manager.h"
#include "core/texture_cache.h"
#include "core/ibl_manager.h"
#include "engine_context.h"
#include <vk_types.h>

#include "mesh_bvh.h"

namespace
{
    // Background / compute playground
    static void ui_background(VulkanEngine *eng)
    {
        if (!eng || !eng->_renderPassManager) return;
        auto *background_pass = eng->_renderPassManager->getPass<BackgroundPass>();
        if (!background_pass)
        {
            ImGui::TextUnformatted("Background pass not available");
            return;
        }

        ComputeEffect &selected = background_pass->_backgroundEffects[background_pass->_currentEffect];

        ImGui::Text("Selected effect: %s", selected.name);
        ImGui::SliderInt("Effect Index", &background_pass->_currentEffect, 0,
                         (int) background_pass->_backgroundEffects.size() - 1);
        ImGui::InputFloat4("data1", reinterpret_cast<float *>(&selected.data.data1));
        ImGui::InputFloat4("data2", reinterpret_cast<float *>(&selected.data.data2));
        ImGui::InputFloat4("data3", reinterpret_cast<float *>(&selected.data.data3));
        ImGui::InputFloat4("data4", reinterpret_cast<float *>(&selected.data.data4));

        ImGui::Separator();
        ImGui::SliderFloat("Render Scale", &eng->renderScale, 0.3f, 1.f);
    }

    // IBL test grid spawner (spheres varying metallic/roughness)
    static void spawn_ibl_test(VulkanEngine *eng)
    {
        if (!eng || !eng->_assetManager || !eng->_sceneManager) return;
        using MC = GLTFMetallic_Roughness::MaterialConstants;

        std::vector<Vertex> verts;
        std::vector<uint32_t> inds;
        primitives::buildSphere(verts, inds, 24, 24);

        const float mVals[5] = {0.0f, 0.25f, 0.5f, 0.75f, 1.0f};
        const float rVals[5] = {0.04f, 0.25f, 0.5f, 0.75f, 1.0f};
        const float spacing = 1.6f;
        const glm::vec3 origin(-spacing * 2.0f, 0.0f, -spacing * 2.0f);

        for (int iy = 0; iy < 5; ++iy)
        {
            for (int ix = 0; ix < 5; ++ix)
            {
                MC c{};
                c.colorFactors = glm::vec4(0.82f, 0.82f, 0.82f, 1.0f);
                c.metal_rough_factors = glm::vec4(mVals[ix], rVals[iy], 0.0f, 0.0f);
                const std::string base = fmt::format("ibltest.m{}_r{}", ix, iy);
                auto mat = eng->_assetManager->createMaterialFromConstants(base + ".mat", c, MaterialPass::MainColor);

                auto mesh = eng->_assetManager->createMesh(base + ".mesh",
                                                           std::span<Vertex>(verts.data(), verts.size()),
                                                           std::span<uint32_t>(inds.data(), inds.size()),
                                                           mat);

                const glm::vec3 pos = origin + glm::vec3(ix * spacing, 0.5f, iy * spacing);
                glm::mat4 M = glm::translate(glm::mat4(1.0f), pos);
                eng->_sceneManager->addMeshInstance(base + ".inst", mesh, M, BoundsType::Sphere);
                eng->_iblTestNames.push_back(base + ".inst");
                eng->_iblTestNames.push_back(base + ".mesh");
                eng->_iblTestNames.push_back(base + ".mat");
            }
        }

        // Chrome and glass extras
        {
            MC chrome{};
            chrome.colorFactors = glm::vec4(0.9f, 0.9f, 0.9f, 1.0f);
            chrome.metal_rough_factors = glm::vec4(1.0f, 0.06f, 0, 0);
            auto mat = eng->_assetManager->createMaterialFromConstants("ibltest.chrome.mat", chrome,
                                                                       MaterialPass::MainColor);
            auto mesh = eng->_assetManager->createMesh("ibltest.chrome.mesh",
                                                       std::span<Vertex>(verts.data(), verts.size()),
                                                       std::span<uint32_t>(inds.data(), inds.size()),
                                                       mat);
            glm::mat4 M = glm::translate(glm::mat4(1.0f), origin + glm::vec3(5.5f, 0.5f, 0.0f));
            eng->_sceneManager->addMeshInstance("ibltest.chrome.inst", mesh, M, BoundsType::Sphere);
            eng->_iblTestNames.insert(eng->_iblTestNames.end(),
                                      {"ibltest.chrome.inst", "ibltest.chrome.mesh", "ibltest.chrome.mat"});
        } {
            MC glass{};
            glass.colorFactors = glm::vec4(0.9f, 0.95f, 1.0f, 0.25f);
            glass.metal_rough_factors = glm::vec4(0.0f, 0.02f, 0, 0);
            auto mat = eng->_assetManager->createMaterialFromConstants("ibltest.glass.mat", glass,
                                                                       MaterialPass::Transparent);
            auto mesh = eng->_assetManager->createMesh("ibltest.glass.mesh",
                                                       std::span<Vertex>(verts.data(), verts.size()),
                                                       std::span<uint32_t>(inds.data(), inds.size()),
                                                       mat);
            glm::mat4 M = glm::translate(glm::mat4(1.0f), origin + glm::vec3(5.5f, 0.5f, 2.0f));
            eng->_sceneManager->addMeshInstance("ibltest.glass.inst", mesh, M, BoundsType::Sphere);
            eng->_iblTestNames.insert(eng->_iblTestNames.end(),
                                      {"ibltest.glass.inst", "ibltest.glass.mesh", "ibltest.glass.mat"});
        }
    }

    static void clear_ibl_test(VulkanEngine *eng)
    {
        if (!eng || !eng->_sceneManager || !eng->_assetManager) return;
        for (size_t i = 0; i < eng->_iblTestNames.size(); ++i)
        {
            const std::string &n = eng->_iblTestNames[i];
            // Remove instances and meshes by prefix
            if (n.ends_with(".inst")) eng->_sceneManager->removeMeshInstance(n);
            else if (n.ends_with(".mesh")) eng->_assetManager->removeMesh(n);
        }
        eng->_iblTestNames.clear();
    }

    static void ui_ibl(VulkanEngine *eng)
    {
        if (!eng) return;
        if (ImGui::Button("Spawn IBL Test Grid")) { spawn_ibl_test(eng); }
        ImGui::SameLine();
        if (ImGui::Button("Clear IBL Test")) { clear_ibl_test(eng); }
        ImGui::TextUnformatted(
            "5x5 spheres: metallic across columns, roughness across rows.\nExtra: chrome + glass.");
    }

    // Quick stats & targets overview
    static void ui_overview(VulkanEngine *eng)
    {
        if (!eng) return;
        ImGui::Text("frametime %.2f ms", eng->stats.frametime);
        ImGui::Text("draw time %.2f ms", eng->stats.mesh_draw_time);
        ImGui::Text("update time %.2f ms", eng->_sceneManager->stats.scene_update_time);
        ImGui::Text("triangles %i", eng->stats.triangle_count);
        ImGui::Text("draws %i", eng->stats.drawcall_count);

        ImGui::Separator();
        ImGui::Text("Draw extent: %ux%u", eng->_drawExtent.width, eng->_drawExtent.height);
        auto scExt = eng->_swapchainManager->swapchainExtent();
        ImGui::Text("Swapchain:   %ux%u", scExt.width, scExt.height);
        ImGui::Text("Draw fmt:    %s", string_VkFormat(eng->_swapchainManager->drawImage().imageFormat));
        ImGui::Text("Swap fmt:    %s", string_VkFormat(eng->_swapchainManager->swapchainImageFormat()));
    }

    // Texture streaming + budget UI
    static const char *stateName(uint8_t s)
    {
        switch (s)
        {
            case 0: return "Unloaded";
            case 1: return "Loading";
            case 2: return "Resident";
            case 3: return "Evicted";
            default: return "?";
        }
    }

    static void ui_textures(VulkanEngine *eng)
    {
        if (!eng || !eng->_textureCache)
        {
            ImGui::TextUnformatted("TextureCache not available");
            return;
        }
        DeviceManager *dev = eng->_deviceManager.get();
        VmaAllocator alloc = dev ? dev->allocator() : VK_NULL_HANDLE;
        unsigned long long devLocalBudget = 0, devLocalUsage = 0;
        if (alloc)
        {
            const VkPhysicalDeviceMemoryProperties *memProps = nullptr;
            vmaGetMemoryProperties(alloc, &memProps);
            VmaBudget budgets[VK_MAX_MEMORY_HEAPS] = {};
            vmaGetHeapBudgets(alloc, budgets);
            if (memProps)
            {
                for (uint32_t i = 0; i < memProps->memoryHeapCount; ++i)
                {
                    if (memProps->memoryHeaps[i].flags & VK_MEMORY_HEAP_DEVICE_LOCAL_BIT)
                    {
                        devLocalBudget += budgets[i].budget;
                        devLocalUsage += budgets[i].usage;
                    }
                }
            }
        }

        const size_t texBudget = eng->query_texture_budget_bytes();
        eng->_textureCache->set_gpu_budget_bytes(texBudget);
        const size_t resBytes = eng->_textureCache->resident_bytes();
        const size_t cpuSrcBytes = eng->_textureCache->cpu_source_bytes();
        ImGui::Text("Device local: %.1f / %.1f MiB",
                    (double) devLocalUsage / 1048576.0,
                    (double) devLocalBudget / 1048576.0);
        ImGui::Text("Texture budget: %.1f MiB", (double) texBudget / 1048576.0);
        ImGui::Text("Resident textures: %.1f MiB", (double) resBytes / 1048576.0);
        ImGui::Text("CPU source bytes: %.1f MiB", (double) cpuSrcBytes / 1048576.0);
        ImGui::SameLine();
        if (ImGui::Button("Trim To Budget Now"))
        {
            eng->_textureCache->evictToBudget(texBudget);
        }

        // Controls
        static int loadsPerPump = 4;
        loadsPerPump = eng->_textureCache->max_loads_per_pump();
        if (ImGui::SliderInt("Loads/Frame", &loadsPerPump, 1, 16))
        {
            eng->_textureCache->set_max_loads_per_pump(loadsPerPump);
        }
        static int uploadBudgetMiB = 128;
        uploadBudgetMiB = (int) (eng->_textureCache->max_bytes_per_pump() / 1048576ull);
        if (ImGui::SliderInt("Upload Budget (MiB)", &uploadBudgetMiB, 16, 2048))
        {
            eng->_textureCache->set_max_bytes_per_pump((size_t) uploadBudgetMiB * 1048576ull);
        }
        static bool keepSources = false;
        keepSources = eng->_textureCache->keep_source_bytes();
        if (ImGui::Checkbox("Keep Source Bytes", &keepSources))
        {
            eng->_textureCache->set_keep_source_bytes(keepSources);
        }
        static int cpuBudgetMiB = 64;
        cpuBudgetMiB = (int) (eng->_textureCache->cpu_source_budget() / 1048576ull);
        if (ImGui::SliderInt("CPU Source Budget (MiB)", &cpuBudgetMiB, 0, 2048))
        {
            eng->_textureCache->set_cpu_source_budget((size_t) cpuBudgetMiB * 1048576ull);
        }
        static int maxUploadDim = 4096;
        maxUploadDim = (int) eng->_textureCache->max_upload_dimension();
        if (ImGui::SliderInt("Max Upload Dimension", &maxUploadDim, 0, 8192))
        {
            eng->_textureCache->set_max_upload_dimension((uint32_t) std::max(0, maxUploadDim));
        }

        TextureCache::DebugStats stats{};
        std::vector<TextureCache::DebugRow> rows;
        eng->_textureCache->debug_snapshot(rows, stats);
        ImGui::Text("Counts  R:%zu  U:%zu  E:%zu",
                    stats.countResident,
                    stats.countUnloaded,
                    stats.countEvicted);

        const int topN = 12;
        if (ImGui::BeginTable("texrows", 4, ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp))
        {
            ImGui::TableSetupColumn("MiB", ImGuiTableColumnFlags_WidthFixed, 80);
            ImGui::TableSetupColumn("State", ImGuiTableColumnFlags_WidthFixed, 90);
            ImGui::TableSetupColumn("LastUsed", ImGuiTableColumnFlags_WidthFixed, 90);
            ImGui::TableSetupColumn("Name");
            ImGui::TableHeadersRow();
            int count = 0;
            for (const auto &r: rows)
            {
                if (count++ >= topN) break;
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::Text("%.2f", (double) r.bytes / 1048576.0);
                ImGui::TableSetColumnIndex(1);
                ImGui::TextUnformatted(stateName(r.state));
                ImGui::TableSetColumnIndex(2);
                ImGui::Text("%u", r.lastUsed);
                ImGui::TableSetColumnIndex(3);
                ImGui::TextUnformatted(r.name.c_str());
            }
            ImGui::EndTable();
        }
    }

    // Shadows / Ray Query controls
    static void ui_shadows(VulkanEngine *eng)
    {
        if (!eng) return;
        const bool rq = eng->_deviceManager->supportsRayQuery();
        const bool as = eng->_deviceManager->supportsAccelerationStructure();
        ImGui::Text("RayQuery: %s", rq ? "supported" : "not available");
        ImGui::Text("AccelStruct: %s", as ? "supported" : "not available");
        ImGui::Separator();

        auto &ss = eng->_context->shadowSettings;
        int mode = static_cast<int>(ss.mode);
        ImGui::TextUnformatted("Shadow Mode");
        ImGui::RadioButton("Clipmap only", &mode, 0);
        ImGui::SameLine();
        ImGui::RadioButton("Clipmap + RT", &mode, 1);
        ImGui::SameLine();
        ImGui::RadioButton("RT only", &mode, 2);
        if (!(rq && as) && mode != 0) mode = 0; // guard for unsupported HW
        ss.mode = static_cast<uint32_t>(mode);
        ss.hybridRayQueryEnabled = (ss.mode != 0);

        ImGui::BeginDisabled(ss.mode != 1u);
        ImGui::TextUnformatted("Cascades using ray assist:");
        for (int i = 0; i < 4; ++i)
        {
            bool on = (ss.hybridRayCascadesMask >> i) & 1u;
            std::string label = std::string("C") + std::to_string(i);
            if (ImGui::Checkbox(label.c_str(), &on))
            {
                if (on) ss.hybridRayCascadesMask |= (1u << i);
                else ss.hybridRayCascadesMask &= ~(1u << i);
            }
            if (i != 3) ImGui::SameLine();
        }
        ImGui::SliderFloat("N·L threshold", &ss.hybridRayNoLThreshold, 0.0f, 1.0f, "%.2f");
        ImGui::EndDisabled();

        ImGui::Separator();
        ImGui::TextWrapped(
            "Clipmap only: raster PCF+RPDB. Clipmap+RT: PCF assisted by ray query at low N·L. RT only: skip shadow maps and use ray tests only.");
    }

    // Render Graph inspection (passes, images, buffers)
    static void ui_render_graph(VulkanEngine *eng)
    {
        if (!eng || !eng->_renderGraph)
        {
            ImGui::TextUnformatted("RenderGraph not available");
            return;
        }
        auto &graph = *eng->_renderGraph;

        std::vector<RenderGraph::RGDebugPassInfo> passInfos;
        graph.debug_get_passes(passInfos);
        if (ImGui::Button("Reload Pipelines")) { eng->_pipelineManager->hotReloadChanged(); }
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

            auto typeName = [](RGPassType t) {
                switch (t)
                {
                    case RGPassType::Graphics: return "Graphics";
                    case RGPassType::Compute: return "Compute";
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
                if (auto it = eng->_rgPassToggles.find(pi.name); it != eng->_rgPassToggles.end()) enabled = it->second;
                std::string chkId = std::string("##en") + std::to_string(i);
                if (ImGui::Checkbox(chkId.c_str(), &enabled))
                {
                    eng->_rgPassToggles[pi.name] = enabled;
                }
                ImGui::TableSetColumnIndex(1);
                ImGui::TextUnformatted(pi.name.c_str());
                ImGui::TableSetColumnIndex(2);
                ImGui::TextUnformatted(typeName(pi.type));
                ImGui::TableSetColumnIndex(3);
                if (pi.gpuMillis >= 0.0f) ImGui::Text("%.2f", pi.gpuMillis);
                else ImGui::TextUnformatted("-");
                ImGui::TableSetColumnIndex(4);
                if (pi.cpuMillis >= 0.0f) ImGui::Text("%.2f", pi.cpuMillis);
                else ImGui::TextUnformatted("-");
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
                for (const auto &im: imgs)
                {
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::Text("%u", im.id);
                    ImGui::TableSetColumnIndex(1);
                    ImGui::TextUnformatted(im.name.c_str());
                    ImGui::TableSetColumnIndex(2);
                    ImGui::TextUnformatted(string_VkFormat(im.format));
                    ImGui::TableSetColumnIndex(3);
                    ImGui::Text("%ux%u", im.extent.width, im.extent.height);
                    ImGui::TableSetColumnIndex(4);
                    ImGui::TextUnformatted(im.imported ? "yes" : "no");
                    ImGui::TableSetColumnIndex(5);
                    ImGui::Text("0x%x", (unsigned) im.creationUsage);
                    ImGui::TableSetColumnIndex(6);
                    ImGui::Text("%d..%d", im.firstUse, im.lastUse);
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
                for (const auto &bf: bufs)
                {
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::Text("%u", bf.id);
                    ImGui::TableSetColumnIndex(1);
                    ImGui::TextUnformatted(bf.name.c_str());
                    ImGui::TableSetColumnIndex(2);
                    ImGui::Text("%zu", (size_t) bf.size);
                    ImGui::TableSetColumnIndex(3);
                    ImGui::TextUnformatted(bf.imported ? "yes" : "no");
                    ImGui::TableSetColumnIndex(4);
                    ImGui::Text("0x%x", (unsigned) bf.usage);
                    ImGui::TableSetColumnIndex(5);
                    ImGui::Text("%d..%d", bf.firstUse, bf.lastUse);
                }
                ImGui::EndTable();
            }
        }
    }

    // Pipeline manager (graphics)
    static void ui_pipelines(VulkanEngine *eng)
    {
        if (!eng || !eng->_pipelineManager)
        {
            ImGui::TextUnformatted("PipelineManager not available");
            return;
        }
        std::vector<PipelineManager::GraphicsPipelineDebugInfo> pipes;
        eng->_pipelineManager->debug_get_graphics(pipes);
        if (ImGui::Button("Reload Changed")) { eng->_pipelineManager->hotReloadChanged(); }
        ImGui::SameLine();
        ImGui::Text("%zu graphics pipelines", pipes.size());
        if (ImGui::BeginTable("gfxpipes", 5, ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp))
        {
            ImGui::TableSetupColumn("Name");
            ImGui::TableSetupColumn("VS");
            ImGui::TableSetupColumn("FS");
            ImGui::TableSetupColumn("Valid", ImGuiTableColumnFlags_WidthFixed, 60);
            ImGui::TableHeadersRow();
            for (const auto &p: pipes)
            {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::TextUnformatted(p.name.c_str());
                ImGui::TableSetColumnIndex(1);
                ImGui::TextUnformatted(p.vertexShaderPath.c_str());
                ImGui::TableSetColumnIndex(2);
                ImGui::TextUnformatted(p.fragmentShaderPath.c_str());
                ImGui::TableSetColumnIndex(3);
                ImGui::TextUnformatted(p.valid ? "yes" : "no");
            }
            ImGui::EndTable();
        }
    }

    // Post-processing
    static void ui_postfx(VulkanEngine *eng)
    {
        if (!eng) return;
        if (auto *tm = eng->_renderPassManager ? eng->_renderPassManager->getPass<TonemapPass>() : nullptr)
        {
            float exp = tm->exposure();
            int mode = tm->mode();
            if (ImGui::SliderFloat("Exposure", &exp, 0.05f, 8.0f)) { tm->setExposure(exp); }
            ImGui::TextUnformatted("Operator");
            ImGui::SameLine();
            if (ImGui::RadioButton("Reinhard", mode == 0))
            {
                mode = 0;
                tm->setMode(mode);
            }
            ImGui::SameLine();
            if (ImGui::RadioButton("ACES", mode == 1))
            {
                mode = 1;
                tm->setMode(mode);
            }
        }
        else
        {
            ImGui::TextUnformatted("Tonemap pass not available");
        }
    }

    // Scene debug bits
    static void ui_scene(VulkanEngine *eng)
    {
        if (!eng) return;
        const DrawContext &dc = eng->_context->getMainDrawContext();
        ImGui::Text("Opaque draws: %zu", dc.OpaqueSurfaces.size());
        ImGui::Text("Transp draws: %zu", dc.TransparentSurfaces.size());
        ImGui::Checkbox("Use ID-buffer picking", &eng->_useIdBufferPicking);
        ImGui::Text("Picking mode: %s",
                    eng->_useIdBufferPicking ? "ID buffer (async, 1-frame latency)" : "CPU raycast");
        ImGui::Checkbox("Debug draw mesh BVH (last pick)", &eng->_debugDrawBVH);
        ImGui::Separator();

        // Spawn glTF instances (runtime)
        ImGui::TextUnformatted("Spawn glTF instance");
        static char gltfPath[256] = "mirage2000/scene.gltf";
        static char gltfName[128] = "gltf_01";
        static float gltfPos[3] = {0.0f, 0.0f, 0.0f};
        static float gltfRot[3] = {0.0f, 0.0f, 0.0f}; // pitch, yaw, roll (deg)
        static float gltfScale[3] = {1.0f, 1.0f, 1.0f};
        ImGui::InputText("Model path (assets/models/...)", gltfPath, IM_ARRAYSIZE(gltfPath));
        ImGui::InputText("Instance name", gltfName, IM_ARRAYSIZE(gltfName));
        ImGui::InputFloat3("Position", gltfPos);
        ImGui::InputFloat3("Rotation (deg XYZ)", gltfRot);
        ImGui::InputFloat3("Scale", gltfScale);
        if (ImGui::Button("Add glTF instance"))
        {
            glm::mat4 T = glm::translate(glm::mat4(1.0f), glm::vec3(gltfPos[0], gltfPos[1], gltfPos[2]));
            glm::mat4 R = glm::eulerAngleXYZ(glm::radians(gltfRot[0]),
                                             glm::radians(gltfRot[1]),
                                             glm::radians(gltfRot[2]));
            glm::mat4 S = glm::scale(glm::mat4(1.0f), glm::vec3(gltfScale[0], gltfScale[1], gltfScale[2]));
            glm::mat4 M = T * R * S;
            eng->addGLTFInstance(gltfName, gltfPath, M);
        }

        ImGui::Separator();
        // Spawn primitive mesh instances (cube/sphere)
        ImGui::TextUnformatted("Spawn primitive");
        static int primType = 0; // 0 = cube, 1 = sphere
        static char primName[128] = "prim_01";
        static float primPos[3] = {0.0f, 0.0f, 0.0f};
        static float primRot[3] = {0.0f, 0.0f, 0.0f};
        static float primScale[3] = {1.0f, 1.0f, 1.0f};
        ImGui::RadioButton("Cube", &primType, 0); ImGui::SameLine();
        ImGui::RadioButton("Sphere", &primType, 1);
        ImGui::InputText("Primitive name", primName, IM_ARRAYSIZE(primName));
        ImGui::InputFloat3("Prim Position", primPos);
        ImGui::InputFloat3("Prim Rotation (deg XYZ)", primRot);
        ImGui::InputFloat3("Prim Scale", primScale);
        if (ImGui::Button("Add primitive instance"))
        {
            std::shared_ptr<MeshAsset> mesh = (primType == 0) ? eng->cubeMesh : eng->sphereMesh;
            if (mesh)
            {
                glm::mat4 T = glm::translate(glm::mat4(1.0f), glm::vec3(primPos[0], primPos[1], primPos[2]));
                glm::mat4 R = glm::eulerAngleXYZ(glm::radians(primRot[0]),
                                                 glm::radians(primRot[1]),
                                                 glm::radians(primRot[2]));
                glm::mat4 S = glm::scale(glm::mat4(1.0f), glm::vec3(primScale[0], primScale[1], primScale[2]));
                glm::mat4 M = T * R * S;
                eng->_sceneManager->addMeshInstance(primName, mesh, M);
            }
        }

        ImGui::Separator();
        // Delete selected model/primitive (uses last pick if valid, otherwise hover)
        static std::string deleteStatus;
        if (ImGui::Button("Delete selected"))
        {
            deleteStatus.clear();
            const auto *pick = eng->_lastPick.valid ? &eng->_lastPick
                                                    : (eng->_hoverPick.valid ? &eng->_hoverPick : nullptr);
            if (!pick || pick->ownerName.empty())
            {
                deleteStatus = "No selection to delete.";
            }
            else if (pick->ownerType == RenderObject::OwnerType::MeshInstance)
            {
                bool ok = eng->_sceneManager->removeMeshInstance(pick->ownerName);
                deleteStatus = ok ? "Removed mesh instance: " + pick->ownerName
                                  : "Mesh instance not found: " + pick->ownerName;
            }
            else if (pick->ownerType == RenderObject::OwnerType::GLTFInstance)
            {
                bool ok = eng->_sceneManager->removeGLTFInstance(pick->ownerName);
                deleteStatus = ok ? "Removed glTF instance: " + pick->ownerName
                                  : "glTF instance not found: " + pick->ownerName;
            }
            else
            {
                deleteStatus = "Cannot delete this object type (static scene).";
            }
        }
        if (!deleteStatus.empty())
        {
            ImGui::TextUnformatted(deleteStatus.c_str());
        }
        ImGui::Separator();

        if (eng->_lastPick.valid)
        {
            const char *meshName = eng->_lastPick.mesh ? eng->_lastPick.mesh->name.c_str() : "<unknown>";
            const char *sceneName = "<none>";
            if (eng->_lastPick.scene && !eng->_lastPick.scene->debugName.empty())
            {
                sceneName = eng->_lastPick.scene->debugName.c_str();
            }
            ImGui::Text("Last pick scene: %s", sceneName);
            ImGui::Text("Last pick source: %s",
                        eng->_useIdBufferPicking ? "ID buffer" : "CPU raycast");
            ImGui::Text("Last pick object ID: %u", eng->_lastPickObjectID);
            ImGui::Text("Last pick mesh: %s (surface %u)", meshName, eng->_lastPick.surfaceIndex);
            ImGui::Text("World pos: (%.3f, %.3f, %.3f)",
                        eng->_lastPick.worldPos.x,
                        eng->_lastPick.worldPos.y,
                        eng->_lastPick.worldPos.z);
            const char *ownerTypeStr = "none";
            switch (eng->_lastPick.ownerType)
            {
                case RenderObject::OwnerType::MeshInstance: ownerTypeStr = "mesh instance"; break;
                case RenderObject::OwnerType::GLTFInstance: ownerTypeStr = "glTF instance"; break;
                case RenderObject::OwnerType::StaticGLTF: ownerTypeStr = "glTF scene"; break;
                default: break;
            }
            const char *ownerName = eng->_lastPick.ownerName.empty() ? "<unnamed>" : eng->_lastPick.ownerName.c_str();
            ImGui::Text("Owner: %s (%s)", ownerName, ownerTypeStr);
            ImGui::Text("Indices: first=%u count=%u",
                        eng->_lastPick.firstIndex,
                        eng->_lastPick.indexCount);

            if (eng->_sceneManager)
            {
                const SceneManager::PickingDebug &dbg = eng->_sceneManager->getPickingDebug();
                ImGui::Text("Mesh BVH used: %s, hit: %s, fallback box: %s",
                            dbg.usedMeshBVH ? "yes" : "no",
                            dbg.meshBVHHit ? "yes" : "no",
                            dbg.meshBVHFallbackBox ? "yes" : "no");
                if (dbg.meshBVHPrimCount > 0)
                {
                    ImGui::Text("Mesh BVH stats: prims=%u, nodes=%u",
                                dbg.meshBVHPrimCount,
                                dbg.meshBVHNodeCount);
                }
            }
        }
        else
        {
            ImGui::TextUnformatted("Last pick: <none>");
        }
        ImGui::Separator();
        if (eng->_hoverPick.valid)
        {
            const char *meshName = eng->_hoverPick.mesh ? eng->_hoverPick.mesh->name.c_str() : "<unknown>";
            ImGui::Text("Hover mesh: %s (surface %u)", meshName, eng->_hoverPick.surfaceIndex);
            const char *ownerTypeStr = "none";
            switch (eng->_hoverPick.ownerType)
            {
                case RenderObject::OwnerType::MeshInstance: ownerTypeStr = "mesh instance"; break;
                case RenderObject::OwnerType::GLTFInstance: ownerTypeStr = "glTF instance"; break;
                case RenderObject::OwnerType::StaticGLTF: ownerTypeStr = "glTF scene"; break;
                default: break;
            }
            const char *ownerName = eng->_hoverPick.ownerName.empty() ? "<unnamed>" : eng->_hoverPick.ownerName.c_str();
            ImGui::Text("Hover owner: %s (%s)", ownerName, ownerTypeStr);
        }
        else
        {
            ImGui::TextUnformatted("Hover: <none>");
        }
        if (!eng->_dragSelection.empty())
        {
            ImGui::Text("Drag selection: %zu objects", eng->_dragSelection.size());
        }

        ImGui::Separator();
        ImGui::TextUnformatted("Object Gizmo (ImGuizmo)");

        if (!eng->_sceneManager)
        {
            ImGui::TextUnformatted("SceneManager not available");
            return;
        }

        SceneManager *sceneMgr = eng->_sceneManager.get();
        const GPUSceneData &sceneData = sceneMgr->getSceneData();

        static ImGuizmo::OPERATION op = ImGuizmo::TRANSLATE;
        static ImGuizmo::MODE mode = ImGuizmo::LOCAL;

        ImGui::TextUnformatted("Operation");
        if (ImGui::RadioButton("Translate", op == ImGuizmo::TRANSLATE)) op = ImGuizmo::TRANSLATE;
        ImGui::SameLine();
        if (ImGui::RadioButton("Rotate", op == ImGuizmo::ROTATE)) op = ImGuizmo::ROTATE;
        ImGui::SameLine();
        if (ImGui::RadioButton("Scale", op == ImGuizmo::SCALE)) op = ImGuizmo::SCALE;

        ImGui::TextUnformatted("Mode");
        if (ImGui::RadioButton("Local", mode == ImGuizmo::LOCAL)) mode = ImGuizmo::LOCAL;
        ImGui::SameLine();
        if (ImGui::RadioButton("World", mode == ImGuizmo::WORLD)) mode = ImGuizmo::WORLD;

        // Resolve a dynamic instance transform for the current pick.
        glm::mat4 targetTransform(1.0f);
        enum class GizmoTarget
        {
            None,
            MeshInstance,
            GLTFInstance,
            Node
        };
        GizmoTarget target = GizmoTarget::None;

        ImGuiIO &io = ImGui::GetIO();
        ImGuizmo::SetOrthographic(false);
        ImGuizmo::SetDrawlist();
        ImGuizmo::SetRect(0.0f, 0.0f, io.DisplaySize.x, io.DisplaySize.y);

        glm::mat4 view = sceneData.view;
        glm::mat4 proj = sceneData.proj;

        proj[1][1] *= -1.0f;

        ImGuizmo::Manipulate(&view[0][0], &proj[0][0],
                             op, mode,
                             &targetTransform[0][0]);
    }
} // namespace

void vk_engine_draw_debug_ui(VulkanEngine *eng)
{
    if (!eng) return;

    ImGuizmo::BeginFrame();

    // Consolidated debug window with tabs
    if (ImGui::Begin("Debug"))
    {
        const ImGuiTabBarFlags tf =
                ImGuiTabBarFlags_Reorderable | ImGuiTabBarFlags_AutoSelectNewTabs;
        if (ImGui::BeginTabBar("DebugTabs", tf))
        {
            if (ImGui::BeginTabItem("Overview"))
            {
                ui_overview(eng);
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("Background"))
            {
                ui_background(eng);
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("Shadows"))
            {
                ui_shadows(eng);
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("Render Graph"))
            {
                ui_render_graph(eng);
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("Pipelines"))
            {
                ui_pipelines(eng);
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("IBL"))
            {
                ui_ibl(eng);
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("PostFX"))
            {
                ui_postfx(eng);
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("Scene"))
            {
                ui_scene(eng);
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("Textures"))
            {
                ui_textures(eng);
                ImGui::EndTabItem();
            }
            ImGui::EndTabBar();
        }
        ImGui::End();
    }
}

