// ImGui debug UI helpers for VulkanEngine.
//
// This file contains the immediate-mode ImGui widgets that expose engine
// statistics, render-graph inspection, texture streaming controls, etc.
// The main frame loop in vk_engine.cpp simply calls vk_engine_draw_debug_ui().

#include "engine.h"
#include "core/picking/picking_system.h"

#include "SDL2/SDL.h"
#include "SDL2/SDL_vulkan.h"

#include "imgui.h"
#include "ImGuizmo.h"

#include "render/primitives.h"
#include "vk_mem_alloc.h"
#include "render/passes/tonemap.h"
#include "render/passes/fxaa.h"
#include "render/passes/background.h"
#include "render/passes/particles.h"
#include <glm/gtx/euler_angles.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include "render/graph/graph.h"
#include "core/pipeline/manager.h"
#include "core/assets/texture_cache.h"
#include "core/assets/ibl_manager.h"
#include "device/images.h"
#include "context.h"
#include <core/types.h>
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <filesystem>

#include "mesh_bvh.h"

namespace
{
    static void ui_window(VulkanEngine *eng)
    {
        if (!eng || !eng->_window) return;

        int num_displays = SDL_GetNumVideoDisplays();
        if (num_displays <= 0)
        {
            ImGui::Text("No displays reported by SDL (%s)", SDL_GetError());
            return;
        }

        int current_display = SDL_GetWindowDisplayIndex(eng->_window);
        if (current_display < 0) current_display = eng->_windowDisplayIndex;
        current_display = std::clamp(current_display, 0, num_displays - 1);

        const char *cur_display_name = SDL_GetDisplayName(current_display);
        if (!cur_display_name) cur_display_name = "Unknown";

        ImGui::Text("Current: %s on display %d (%s)",
                    (eng->_windowMode == VulkanEngine::WindowMode::Windowed)
                        ? "Windowed"
                        : (eng->_windowMode == VulkanEngine::WindowMode::FullscreenDesktop)
                              ? "Borderless Fullscreen"
                              : "Exclusive Fullscreen",
                    current_display,
                    cur_display_name);

        static int pending_display = -1;
        static int pending_mode = -1; // 0 windowed, 1 borderless, 2 exclusive
        if (pending_display < 0) pending_display = current_display;
        if (pending_mode < 0) pending_mode = static_cast<int>(eng->_windowMode);

        ImGui::Separator();

        if (ImGui::BeginCombo("Monitor", cur_display_name))
        {
            for (int i = 0; i < num_displays; ++i)
            {
                const char *name = SDL_GetDisplayName(i);
                if (!name) name = "Unknown";
                const bool selected = (pending_display == i);
                if (ImGui::Selectable(name, selected))
                {
                    pending_display = i;
                }
                if (selected)
                {
                    ImGui::SetItemDefaultFocus();
                }
            }
            ImGui::EndCombo();
        }

        const char *mode_labels[] = {
            "Windowed",
            "Borderless (Fullscreen Desktop)",
            "Exclusive Fullscreen"
        };
        ImGui::Combo("Mode", &pending_mode, mode_labels, 3);

        ImGui::TextUnformatted("Apply triggers immediate swapchain recreation.");
        if (ImGui::Button("Apply"))
        {
            auto mode = static_cast<VulkanEngine::WindowMode>(std::clamp(pending_mode, 0, 2));
            eng->set_window_mode(mode, pending_display);

            // Re-sync pending selections with what SDL actually applied.
            pending_display = SDL_GetWindowDisplayIndex(eng->_window);
            if (pending_display < 0) pending_display = eng->_windowDisplayIndex;
            pending_display = std::clamp(pending_display, 0, num_displays - 1);
            pending_mode = static_cast<int>(eng->_windowMode);
        }
        ImGui::SameLine();
        if (ImGui::Button("Use Current"))
        {
            pending_display = current_display;
            pending_mode = static_cast<int>(eng->_windowMode);
        }

        ImGui::Separator();
        ImGui::TextUnformatted("HiDPI / Sizes");
        ImGui::Text("HiDPI enabled: %s", eng->_hiDpiEnabled ? "yes" : "no");

        int winW = 0, winH = 0;
        SDL_GetWindowSize(eng->_window, &winW, &winH);
        int drawW = 0, drawH = 0;
        SDL_Vulkan_GetDrawableSize(eng->_window, &drawW, &drawH);
        ImGui::Text("Window size: %d x %d", winW, winH);
        ImGui::Text("Drawable size: %d x %d", drawW, drawH);
        if (winW > 0 && winH > 0 && drawW > 0 && drawH > 0)
        {
            ImGui::Text("Drawable scale: %.3f x %.3f",
                        static_cast<float>(drawW) / static_cast<float>(winW),
                        static_cast<float>(drawH) / static_cast<float>(winH));
        }
        if (eng->_swapchainManager)
        {
            VkExtent2D sw = eng->_swapchainManager->swapchainExtent();
            ImGui::Text("Swapchain extent: %u x %u", sw.width, sw.height);
        }

        ImGui::Separator();
        ImGui::TextUnformatted("GPU");
        if (!eng->_deviceManager || !eng->_deviceManager->physicalDevice())
        {
            ImGui::TextUnformatted("No Vulkan device initialized.");
            return;
        }

        VkPhysicalDevice gpu = eng->_deviceManager->physicalDevice();
        VkPhysicalDeviceProperties props{};
        vkGetPhysicalDeviceProperties(gpu, &props);
        VkPhysicalDeviceMemoryProperties mem{};
        vkGetPhysicalDeviceMemoryProperties(gpu, &mem);

        auto type_str = [](VkPhysicalDeviceType t) -> const char*
        {
            switch (t)
            {
                case VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU: return "Discrete";
                case VK_PHYSICAL_DEVICE_TYPE_INTEGRATED_GPU: return "Integrated";
                case VK_PHYSICAL_DEVICE_TYPE_VIRTUAL_GPU: return "Virtual";
                case VK_PHYSICAL_DEVICE_TYPE_CPU: return "CPU";
                default: return "Other";
            }
        };

        uint64_t device_local_bytes = 0;
        for (uint32_t i = 0; i < mem.memoryHeapCount; ++i)
        {
            if (mem.memoryHeaps[i].flags & VK_MEMORY_HEAP_DEVICE_LOCAL_BIT)
            {
                device_local_bytes += mem.memoryHeaps[i].size;
            }
        }
        const double vram_gib = static_cast<double>(device_local_bytes) / (1024.0 * 1024.0 * 1024.0);

        const uint32_t api = props.apiVersion;
        ImGui::Text("Name: %s", props.deviceName);
        ImGui::Text("Type: %s", type_str(props.deviceType));
        ImGui::Text("Vendor: 0x%04x  Device: 0x%04x", props.vendorID, props.deviceID);
        ImGui::Text("Vulkan API: %u.%u.%u", VK_VERSION_MAJOR(api), VK_VERSION_MINOR(api), VK_VERSION_PATCH(api));
        ImGui::Text("Driver: %u (0x%08x)", props.driverVersion, props.driverVersion);
        ImGui::Text("Device-local memory: %.2f GiB", vram_gib);
        ImGui::Text("RayQuery: %s  AccelStruct: %s",
                    eng->_deviceManager->supportsRayQuery() ? "yes" : "no",
                    eng->_deviceManager->supportsAccelerationStructure() ? "yes" : "no");
    }

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
        ImGui::TextUnformatted("Render Resolution");

        static int pendingLogicalW = 0;
        static int pendingLogicalH = 0;
        if (pendingLogicalW <= 0 || pendingLogicalH <= 0)
        {
            pendingLogicalW = static_cast<int>(eng->_logicalRenderExtent.width);
            pendingLogicalH = static_cast<int>(eng->_logicalRenderExtent.height);
        }

        ImGui::InputInt("Logical Width", &pendingLogicalW);
        ImGui::InputInt("Logical Height", &pendingLogicalH);

        if (ImGui::Button("Apply Logical Resolution"))
        {
            uint32_t w = static_cast<uint32_t>(pendingLogicalW > 0 ? pendingLogicalW : 1);
            uint32_t h = static_cast<uint32_t>(pendingLogicalH > 0 ? pendingLogicalH : 1);
            eng->set_logical_render_extent(VkExtent2D{w, h});
        }
        ImGui::SameLine();
        if (ImGui::Button("720p"))
        {
            pendingLogicalW = 1280;
            pendingLogicalH = 720;
        }
        ImGui::SameLine();
        if (ImGui::Button("1080p"))
        {
            pendingLogicalW = 1920;
            pendingLogicalH = 1080;
        }
        ImGui::SameLine();
        if (ImGui::Button("1440p"))
        {
            pendingLogicalW = 2560;
            pendingLogicalH = 1440;
        }

        static float pendingScale = 1.0f;
        if (!ImGui::IsAnyItemActive())
        {
            pendingScale = eng->renderScale;
        }
        bool scaleChanged = ImGui::SliderFloat("Render Scale", &pendingScale, 0.25f, 2.0f);
        bool applyScale = scaleChanged && ImGui::IsItemDeactivatedAfterEdit();
        ImGui::SameLine();
        applyScale = ImGui::Button("Apply Scale") || applyScale;
        if (applyScale)
        {
            eng->set_render_scale(pendingScale);
        }
    }

    static void ui_particles(VulkanEngine *eng)
    {
        if (!eng || !eng->_renderPassManager) return;
        auto *pass = eng->_renderPassManager->getPass<ParticlePass>();
        if (!pass)
        {
            ImGui::TextUnformatted("Particle pass not available");
            return;
        }

        const uint32_t freeCount = pass->free_particles();
        const uint32_t allocCount = pass->allocated_particles();
        ImGui::Text("Pool: %u allocated / %u free (max %u)", allocCount, freeCount, ParticlePass::k_max_particles);

        ImGui::Separator();

        static int newCount = 32768;
        newCount = std::max(newCount, 1);
        ImGui::InputInt("New System Particles", &newCount);
        ImGui::SameLine();
        if (ImGui::Button("Create"))
        {
            const uint32_t want = static_cast<uint32_t>(std::max(1, newCount));
            pass->create_system(want);
        }
        ImGui::SameLine();
        if (ImGui::Button("Create 32k"))
        {
            pass->create_system(32768);
        }
        ImGui::SameLine();
        if (ImGui::Button("Create 128k"))
        {
            pass->create_system(ParticlePass::k_max_particles);
        }

        ImGui::Separator();

        auto &systems = pass->systems();
        if (systems.empty())
        {
            ImGui::TextUnformatted("No particle systems. Create one above.");
            return;
        }

        static int selected = 0;
        selected = std::clamp(selected, 0, (int)systems.size() - 1);

        if (ImGui::BeginListBox("Systems"))
        {
            for (int i = 0; i < (int)systems.size(); ++i)
            {
                const auto &s = systems[i];
                char label[128];
                std::snprintf(label, sizeof(label), "#%u base=%u count=%u %s",
                              s.id, s.base, s.count, s.enabled ? "on" : "off");
                const bool isSelected = (selected == i);
                if (ImGui::Selectable(label, isSelected))
                {
                    selected = i;
                }
                if (isSelected) ImGui::SetItemDefaultFocus();
            }
            ImGui::EndListBox();
        }

        selected = std::clamp(selected, 0, (int)systems.size() - 1);
        auto &s = systems[(size_t)selected];

        static std::vector<std::string> vfxKtx2;
        auto refresh_vfx_list = [&]() {
            vfxKtx2.clear();
            vfxKtx2.push_back(std::string{}); // None
            if (!eng || !eng->_assetManager) return;
            const auto &paths = eng->_assetManager->paths();
            if (paths.assets.empty()) return;
            std::error_code ec;
            std::filesystem::path vfxDir = paths.assets / "vfx";
            if (!std::filesystem::exists(vfxDir, ec) || ec) return;
            for (const auto &entry : std::filesystem::directory_iterator(vfxDir, ec))
            {
                if (ec) break;
                if (!entry.is_regular_file(ec) || ec) continue;
                const auto p = entry.path();
                if (p.extension() != ".ktx2" && p.extension() != ".KTX2") continue;
                vfxKtx2.push_back(std::string("vfx/") + p.filename().string());
            }
            std::sort(vfxKtx2.begin() + 1, vfxKtx2.end());
        };
        if (vfxKtx2.empty())
        {
            refresh_vfx_list();
        }

        ImGui::Separator();

        ImGui::Text("Selected: id=%u base=%u count=%u", s.id, s.base, s.count);
        ImGui::Checkbox("Enabled", &s.enabled);
        ImGui::SameLine();
        if (ImGui::Button("Reset (Respawn)"))
        {
            s.reset = true;
        }
        ImGui::SameLine();
        if (ImGui::Button("Destroy"))
        {
            const uint32_t id = s.id;
            pass->destroy_system(id);
            selected = 0;
            return;
        }

        const char *blendItems[] = {"Additive", "Alpha (block-sorted)"};
        int blend = (s.blend == ParticlePass::BlendMode::Alpha) ? 1 : 0;
        if (ImGui::Combo("Blend", &blend, blendItems, 2))
        {
            s.blend = (blend == 1) ? ParticlePass::BlendMode::Alpha : ParticlePass::BlendMode::Additive;
        }

        ImGui::Separator();

        static int pendingResizeCount = 0;
        if (!ImGui::IsAnyItemActive())
        {
            pendingResizeCount = (int)s.count;
        }
        ImGui::InputInt("Resize Count", &pendingResizeCount);
        ImGui::SameLine();
        if (ImGui::Button("Apply Resize"))
        {
            const uint32_t want = static_cast<uint32_t>(std::max(0, pendingResizeCount));
            pass->resize_system(s.id, want);
        }

        ImGui::Separator();
        ImGui::TextUnformatted("Emitter");
        ImGui::InputFloat3("Position (local)", reinterpret_cast<float *>(&s.params.emitter_pos_local));
        ImGui::SliderFloat("Spawn Radius", &s.params.spawn_radius, 0.0f, 10.0f, "%.3f");
        ImGui::InputFloat3("Direction (local)", reinterpret_cast<float *>(&s.params.emitter_dir_local));
        ImGui::SliderFloat("Cone Angle (deg)", &s.params.cone_angle_degrees, 0.0f, 89.0f, "%.1f");

        ImGui::Separator();
        ImGui::TextUnformatted("Motion");
        ImGui::InputFloat("Min Speed", &s.params.min_speed);
        ImGui::InputFloat("Max Speed", &s.params.max_speed);
        ImGui::InputFloat("Min Life (s)", &s.params.min_life);
        ImGui::InputFloat("Max Life (s)", &s.params.max_life);
        ImGui::InputFloat("Min Size", &s.params.min_size);
        ImGui::InputFloat("Max Size", &s.params.max_size);
        ImGui::SliderFloat("Drag", &s.params.drag, 0.0f, 10.0f, "%.3f");
        ImGui::SliderFloat("Gravity (m/s^2)", &s.params.gravity, 0.0f, 30.0f, "%.2f");

        ImGui::Separator();
        ImGui::TextUnformatted("Rendering");
        ImGui::SliderFloat("Soft Depth (m)", &s.params.soft_depth_distance, 0.0f, 2.0f, "%.3f");

        if (ImGui::Button("Refresh VFX List"))
        {
            refresh_vfx_list();
        }
        ImGui::SameLine();
        if (ImGui::Button("Use Flame Defaults"))
        {
            s.flipbook_texture = "vfx/flame.ktx2";
            s.noise_texture = "vfx/simplex.ktx2";
            s.params.flipbook_cols = 16;
            s.params.flipbook_rows = 4;
            s.params.flipbook_fps = 30.0f;
            s.params.flipbook_intensity = 1.0f;
            s.params.noise_scale = 6.0f;
            s.params.noise_strength = 0.05f;
            s.params.noise_scroll = glm::vec2(0.0f, 0.0f);
            pass->preload_vfx_texture(s.flipbook_texture);
            pass->preload_vfx_texture(s.noise_texture);
        }

        auto combo_vfx = [&](const char *label, std::string &path) {
            const char *preview = path.empty() ? "None" : path.c_str();
            if (ImGui::BeginCombo(label, preview))
            {
                for (const auto &opt : vfxKtx2)
                {
                    const bool isNone = opt.empty();
                    const bool isSelected = (path == opt) || (path.empty() && isNone);
                    const char *name = isNone ? "None" : opt.c_str();
                    if (ImGui::Selectable(name, isSelected))
                    {
                        path = opt;
                        if (!path.empty())
                        {
                            pass->preload_vfx_texture(path);
                        }
                    }
                    if (isSelected) ImGui::SetItemDefaultFocus();
                }
                ImGui::EndCombo();
            }
        };

        ImGui::Separator();
        ImGui::TextUnformatted("Flipbook");
        combo_vfx("Flipbook Texture", s.flipbook_texture);
        int cols = (int)s.params.flipbook_cols;
        int rows = (int)s.params.flipbook_rows;
        cols = std::max(cols, 1);
        rows = std::max(rows, 1);
        if (ImGui::InputInt("Flipbook Cols", &cols)) s.params.flipbook_cols = (uint32_t)std::max(cols, 1);
        if (ImGui::InputInt("Flipbook Rows", &rows)) s.params.flipbook_rows = (uint32_t)std::max(rows, 1);
        ImGui::SliderFloat("Flipbook FPS", &s.params.flipbook_fps, 0.0f, 120.0f, "%.1f");
        ImGui::SliderFloat("Flipbook Intensity", &s.params.flipbook_intensity, 0.0f, 8.0f, "%.3f");

        ImGui::Separator();
        ImGui::TextUnformatted("Noise");
        combo_vfx("Noise Texture", s.noise_texture);
        ImGui::SliderFloat("Noise Scale", &s.params.noise_scale, 0.0f, 32.0f, "%.3f");
        ImGui::SliderFloat("Noise Strength", &s.params.noise_strength, 0.0f, 1.0f, "%.3f");
        ImGui::InputFloat2("Noise Scroll", reinterpret_cast<float *>(&s.params.noise_scroll));

        ImGui::Separator();
        ImGui::TextUnformatted("Color");
        ImGui::ColorEdit4("Tint", reinterpret_cast<float *>(&s.params.color), ImGuiColorEditFlags_Float);
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

        ImGui::Separator();

        // Post-processing: FXAA
        if (auto *fx = eng->_renderPassManager ? eng->_renderPassManager->getPass<FxaaPass>() : nullptr)
        {
            bool fxaaEnabled = fx->enabled();
            if (ImGui::Checkbox("FXAA", &fxaaEnabled))
            {
                fx->set_enabled(fxaaEnabled);
            }
            float edgeTh = fx->edge_threshold();
            if (ImGui::SliderFloat("FXAA Edge Threshold", &edgeTh, 0.01f, 0.5f))
            {
                fx->set_edge_threshold(edgeTh);
            }
            float edgeThMin = fx->edge_threshold_min();
            if (ImGui::SliderFloat("FXAA Edge Threshold Min", &edgeThMin, 0.0f, 0.1f))
            {
                fx->set_edge_threshold_min(edgeThMin);
            }
        }
        else
        {
            ImGui::TextUnformatted("FXAA pass not available");
        }
        ImGui::TextUnformatted("IBL Volumes (reflection probes)");

        if (!eng->_iblManager)
        {
            ImGui::TextUnformatted("IBLManager not available");
            return;
        }

        if (eng->_activeIBLVolume < 0)
        {
            ImGui::TextUnformatted("Active IBL: Global");
        }
        else
        {
            ImGui::Text("Active IBL: Volume %d", eng->_activeIBLVolume);
        }

        if (ImGui::Button("Add IBL Volume"))
        {
            VulkanEngine::IBLVolume vol{};
            if (eng->_sceneManager)
            {
                vol.center_world = eng->_sceneManager->getMainCamera().position_world;
            }
            vol.halfExtents = glm::vec3(10.0f, 10.0f, 10.0f);
            vol.paths = eng->_globalIBLPaths;
            eng->_iblVolumes.push_back(vol);
        }

        for (size_t i = 0; i < eng->_iblVolumes.size(); ++i)
        {
            auto &vol = eng->_iblVolumes[i];
            ImGui::PushID(static_cast<int>(i));
            ImGui::Separator();
            ImGui::Text("Volume %zu", i);
            ImGui::Checkbox("Enabled", &vol.enabled);
            {
                double c[3] = {vol.center_world.x, vol.center_world.y, vol.center_world.z};
                if (ImGui::InputScalarN("Center (world)", ImGuiDataType_Double, c, 3, nullptr, nullptr, "%.3f"))
                {
                    vol.center_world = WorldVec3(c[0], c[1], c[2]);
                }
            }
            ImGui::InputFloat3("Half Extents", &vol.halfExtents.x);

            // Simple path editors; store absolute or engine-local paths.
            char specBuf[256]{};
            char diffBuf[256]{};
            char bgBuf[256]{};
            char brdfBuf[256]{};
            std::strncpy(specBuf, vol.paths.specularCube.c_str(), sizeof(specBuf) - 1);
            std::strncpy(diffBuf, vol.paths.diffuseCube.c_str(), sizeof(diffBuf) - 1);
            std::strncpy(bgBuf, vol.paths.background2D.c_str(), sizeof(bgBuf) - 1);
            std::strncpy(brdfBuf, vol.paths.brdfLut2D.c_str(), sizeof(brdfBuf) - 1);

            if (ImGui::InputText("Specular path", specBuf, IM_ARRAYSIZE(specBuf)))
            {
                vol.paths.specularCube = specBuf;
            }
            if (ImGui::InputText("Diffuse path", diffBuf, IM_ARRAYSIZE(diffBuf)))
            {
                vol.paths.diffuseCube = diffBuf;
            }
            if (ImGui::InputText("Background path", bgBuf, IM_ARRAYSIZE(bgBuf)))
            {
                vol.paths.background2D = bgBuf;
            }
            if (ImGui::InputText("BRDF LUT path", brdfBuf, IM_ARRAYSIZE(brdfBuf)))
            {
                vol.paths.brdfLut2D = brdfBuf;
            }

            if (ImGui::Button("Reload This Volume IBL"))
            {
                if (eng->_iblManager && vol.enabled)
                {
                    if (eng->_iblManager->load_async(vol.paths))
                    {
                        eng->_pendingIBLRequest.active = true;
                        eng->_pendingIBLRequest.targetVolume = static_cast<int>(i);
                        eng->_pendingIBLRequest.paths = vol.paths;
                    }
                }
            }
            ImGui::SameLine();
            if (ImGui::Button("Set As Global IBL"))
            {
                eng->_globalIBLPaths = vol.paths;
                if (eng->_iblManager)
                {
                    if (eng->_iblManager->load_async(eng->_globalIBLPaths))
                    {
                        eng->_pendingIBLRequest.active = true;
                        eng->_pendingIBLRequest.targetVolume = -1;
                        eng->_pendingIBLRequest.paths = eng->_globalIBLPaths;
                        eng->_hasGlobalIBL = false;
                    }
                }
            }

            ImGui::PopID();
        }
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

        if (eng->_sceneManager)
        {
            ImGui::Separator();
            WorldVec3 origin = eng->_sceneManager->get_world_origin();
            WorldVec3 camWorld = eng->_sceneManager->getMainCamera().position_world;
            glm::vec3 camLocal = eng->_sceneManager->get_camera_local_position();
            ImGui::Text("Origin (world): (%.3f, %.3f, %.3f)", origin.x, origin.y, origin.z);
            ImGui::Text("Camera (world): (%.3f, %.3f, %.3f)", camWorld.x, camWorld.y, camWorld.z);
            ImGui::Text("Camera (local): (%.3f, %.3f, %.3f)", camLocal.x, camLocal.y, camLocal.z);
        }
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

    static const char *job_state_name(AsyncAssetLoader::JobState s)
    {
        using JS = AsyncAssetLoader::JobState;
        switch (s)
        {
            case JS::Pending:   return "Pending";
            case JS::Running:   return "Running";
            case JS::Completed: return "Completed";
            case JS::Failed:    return "Failed";
            case JS::Cancelled: return "Cancelled";
            default:            return "?";
        }
    }

    static void ui_async_assets(VulkanEngine *eng)
    {
        if (!eng || !eng->_asyncLoader)
        {
            ImGui::TextUnformatted("AsyncAssetLoader not available");
            return;
        }

        std::vector<AsyncAssetLoader::DebugJob> jobs;
        eng->_asyncLoader->debug_snapshot(jobs);

        ImGui::Text("Active jobs: %zu", jobs.size());
        ImGui::Separator();

        if (!jobs.empty())
        {
            if (ImGui::BeginTable("async_jobs", 5, ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp))
            {
                ImGui::TableSetupColumn("ID", ImGuiTableColumnFlags_WidthFixed, 40);
                ImGui::TableSetupColumn("Scene");
                ImGui::TableSetupColumn("Model");
                ImGui::TableSetupColumn("State", ImGuiTableColumnFlags_WidthFixed, 90);
                ImGui::TableSetupColumn("Progress", ImGuiTableColumnFlags_WidthFixed, 180);
                ImGui::TableHeadersRow();

                for (const auto &j : jobs)
                {
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::Text("%u", j.id);
                    ImGui::TableSetColumnIndex(1);
                    ImGui::TextUnformatted(j.scene_name.c_str());
                    ImGui::TableSetColumnIndex(2);
                    ImGui::TextUnformatted(j.model_relative_path.c_str());
                    ImGui::TableSetColumnIndex(3);
                    ImGui::TextUnformatted(job_state_name(j.state));
                    ImGui::TableSetColumnIndex(4);
                    float p = j.progress;
                    ImGui::ProgressBar(p, ImVec2(-FLT_MIN, 0.0f));
                    if (j.texture_count > 0)
                    {
                        ImGui::SameLine();
                        ImGui::Text("(%zu/%zu tex)", j.textures_resident, j.texture_count);
                    }
                }
                ImGui::EndTable();
            }
        }
        else
        {
            ImGui::TextUnformatted("No async asset jobs currently running.");
        }

        ImGui::Separator();
        ImGui::TextUnformatted("Spawn async glTF instance");
        static char gltfPath[256] = "mirage2000/scene.gltf";
        static char gltfName[128] = "async_gltf_01";
        static float gltfPos[3] = {0.0f, 0.0f, 0.0f};
        static float gltfRot[3] = {0.0f, 0.0f, 0.0f};
        static float gltfScale[3] = {1.0f, 1.0f, 1.0f};
        ImGui::InputText("Model path (assets/models/...)", gltfPath, IM_ARRAYSIZE(gltfPath));
        ImGui::InputText("Instance name", gltfName, IM_ARRAYSIZE(gltfName));
        ImGui::InputFloat3("Position", gltfPos);
        ImGui::InputFloat3("Rotation (deg XYZ)", gltfRot);
        ImGui::InputFloat3("Scale", gltfScale);
        if (ImGui::Button("Load glTF async"))
        {
            glm::mat4 T = glm::translate(glm::mat4(1.0f), glm::vec3(gltfPos[0], gltfPos[1], gltfPos[2]));
            glm::mat4 R = glm::eulerAngleXYZ(glm::radians(gltfRot[0]),
                                             glm::radians(gltfRot[1]),
                                             glm::radians(gltfRot[2]));
            glm::mat4 S = glm::scale(glm::mat4(1.0f), glm::vec3(gltfScale[0], gltfScale[1], gltfScale[2]));
            glm::mat4 M = T * R * S;
            eng->loadGLTFAsync(gltfName, gltfPath, M);
            eng->preloadInstanceTextures(gltfName);
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

        // Global on/off toggle for all shadowing.
        ImGui::Checkbox("Enable Shadows", &ss.enabled);
        ImGui::Separator();

        ImGui::BeginDisabled(!ss.enabled);
        int mode = static_cast<int>(ss.mode);
        ImGui::TextUnformatted("Shadow Mode");
        ImGui::RadioButton("Clipmap only", &mode, 0);
        ImGui::SameLine();
        ImGui::RadioButton("Clipmap + RT", &mode, 1);
        ImGui::SameLine();
        ImGui::RadioButton("RT only", &mode, 2);
        if (!(rq && as) && mode != 0) mode = 0; // guard for unsupported HW
        ss.mode = static_cast<uint32_t>(mode);
        ss.hybridRayQueryEnabled = ss.enabled && (ss.mode != 0);

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
        if (!eng->_context) return;

        EngineContext *ctx = eng->_context.get();

        ImGui::TextUnformatted("Reflections");
        bool ssrEnabled = ctx->enableSSR;
        if (ImGui::Checkbox("Enable Screen-Space Reflections", &ssrEnabled))
        {
            ctx->enableSSR = ssrEnabled;
        }

        int reflMode = static_cast<int>(ctx->reflectionMode);
        ImGui::TextUnformatted("Reflection Mode");
        ImGui::RadioButton("SSR only", &reflMode, 0);
        ImGui::SameLine();
        ImGui::RadioButton("SSR + RT fallback", &reflMode, 1);
        ImGui::SameLine();
        ImGui::RadioButton("RT only", &reflMode, 2);

        const bool rq = eng->_deviceManager->supportsRayQuery();
        const bool as = eng->_deviceManager->supportsAccelerationStructure();
        if (!(rq && as) && reflMode != 0)
        {
            reflMode = 0; // guard for unsupported HW
        }
        ctx->reflectionMode = static_cast<uint32_t>(reflMode);

        ImGui::Separator();
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

            // Bloom controls
            bool bloomEnabled = tm->bloomEnabled();
            if (ImGui::Checkbox("Bloom", &bloomEnabled))
            {
                tm->setBloomEnabled(bloomEnabled);
            }
            float bloomThreshold = tm->bloomThreshold();
            if (ImGui::SliderFloat("Bloom Threshold", &bloomThreshold, 0.0f, 5.0f))
            {
                tm->setBloomThreshold(bloomThreshold);
            }
            float bloomIntensity = tm->bloomIntensity();
            if (ImGui::SliderFloat("Bloom Intensity", &bloomIntensity, 0.0f, 2.0f))
            {
                tm->setBloomIntensity(bloomIntensity);
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
        PickingSystem *picking = eng->picking();
        if (picking)
        {
            bool use_id = picking->use_id_buffer_picking();
            if (ImGui::Checkbox("Use ID-buffer picking", &use_id))
            {
                picking->set_use_id_buffer_picking(use_id);
            }
            ImGui::Text("Picking mode: %s",
                        use_id ? "ID buffer (async, 1-frame latency)" : "CPU raycast");

            bool debug_bvh = picking->debug_draw_bvh();
            if (ImGui::Checkbox("Debug draw mesh BVH (last pick)", &debug_bvh))
            {
                picking->set_debug_draw_bvh(debug_bvh);
            }
        }
        else
        {
            ImGui::TextUnformatted("Picking system not available");
        }
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

        // Point light editor
        if (eng->_sceneManager)
        {
            ImGui::Separator();
            ImGui::TextUnformatted("Point lights");

            SceneManager *sceneMgr = eng->_sceneManager.get();
            const auto &lights = sceneMgr->getPointLights();
            ImGui::Text("Active lights: %zu", lights.size());

            static int selectedLight = -1;
            if (selectedLight >= static_cast<int>(lights.size()))
            {
                selectedLight = static_cast<int>(lights.size()) - 1;
            }

            if (ImGui::BeginListBox("Light list"))
            {
                for (size_t i = 0; i < lights.size(); ++i)
                {
                    std::string label = fmt::format("Light {}", i);
                    const bool isSelected = (selectedLight == static_cast<int>(i));
                    if (ImGui::Selectable(label.c_str(), isSelected))
                    {
                        selectedLight = static_cast<int>(i);
                    }
                }
                ImGui::EndListBox();
            }

            // Controls for the selected light
            if (selectedLight >= 0 && selectedLight < static_cast<int>(lights.size()))
            {
                SceneManager::PointLight pl{};
                if (sceneMgr->getPointLight(static_cast<size_t>(selectedLight), pl))
                {
                    double pos[3] = {pl.position_world.x, pl.position_world.y, pl.position_world.z};
                    float col[3] = {pl.color.r, pl.color.g, pl.color.b};
                    bool changed = false;

                    changed |= ImGui::InputScalarN("Position (world)", ImGuiDataType_Double, pos, 3, nullptr, nullptr, "%.3f");
                    changed |= ImGui::SliderFloat("Radius", &pl.radius, 0.1f, 1000.0f);
                    changed |= ImGui::ColorEdit3("Color", col);
                    changed |= ImGui::SliderFloat("Intensity", &pl.intensity, 0.0f, 100.0f);

                    if (changed)
                    {
                        pl.position_world = WorldVec3(pos[0], pos[1], pos[2]);
                        pl.color = glm::vec3(col[0], col[1], col[2]);
                        sceneMgr->setPointLight(static_cast<size_t>(selectedLight), pl);
                    }

                    if (ImGui::Button("Remove selected light"))
                    {
                        sceneMgr->removePointLight(static_cast<size_t>(selectedLight));
                        selectedLight = -1;
                    }
                }
            }

            // Controls for adding a new light
            ImGui::Separator();
            ImGui::TextUnformatted("Add point light");
            static double newPos[3] = {0.0, 1.0, 0.0};
            static float newRadius = 10.0f;
            static float newColor[3] = {1.0f, 1.0f, 1.0f};
            static float newIntensity = 5.0f;

            ImGui::InputScalarN("New position (world)", ImGuiDataType_Double, newPos, 3, nullptr, nullptr, "%.3f");
            ImGui::SliderFloat("New radius", &newRadius, 0.1f, 1000.0f);
            ImGui::ColorEdit3("New color", newColor);
            ImGui::SliderFloat("New intensity", &newIntensity, 0.0f, 100.0f);

            if (ImGui::Button("Add point light"))
            {
                SceneManager::PointLight pl{};
                pl.position_world = WorldVec3(newPos[0], newPos[1], newPos[2]);
                pl.radius = newRadius;
                pl.color = glm::vec3(newColor[0], newColor[1], newColor[2]);
                pl.intensity = newIntensity;

                const size_t oldCount = sceneMgr->getPointLightCount();
                sceneMgr->addPointLight(pl);
                selectedLight = static_cast<int>(oldCount);
            }

            if (ImGui::Button("Clear all lights"))
            {
                sceneMgr->clearPointLights();
                selectedLight = -1;
            }

            // Spot light editor
            ImGui::Separator();
            ImGui::TextUnformatted("Spot lights");

            const auto &spotLights = sceneMgr->getSpotLights();
            ImGui::Text("Active spot lights: %zu", spotLights.size());

            static int selectedSpot = -1;
            if (selectedSpot >= static_cast<int>(spotLights.size()))
            {
                selectedSpot = static_cast<int>(spotLights.size()) - 1;
            }

            if (ImGui::BeginListBox("Spot light list##spot_list"))
            {
                for (size_t i = 0; i < spotLights.size(); ++i)
                {
                    std::string label = fmt::format("Spot {}", i);
                    const bool isSelected = (selectedSpot == static_cast<int>(i));
                    if (ImGui::Selectable(label.c_str(), isSelected))
                    {
                        selectedSpot = static_cast<int>(i);
                    }
                }
                ImGui::EndListBox();
            }

            if (selectedSpot >= 0 && selectedSpot < static_cast<int>(spotLights.size()))
            {
                SceneManager::SpotLight sl{};
                if (sceneMgr->getSpotLight(static_cast<size_t>(selectedSpot), sl))
                {
                    double pos[3] = {sl.position_world.x, sl.position_world.y, sl.position_world.z};
                    float dir[3] = {sl.direction.x, sl.direction.y, sl.direction.z};
                    float col[3] = {sl.color.r, sl.color.g, sl.color.b};
                    bool changed = false;

                    changed |= ImGui::InputScalarN("Position (world)##spot_pos", ImGuiDataType_Double, pos, 3, nullptr, nullptr, "%.3f");
                    changed |= ImGui::InputFloat3("Direction##spot_dir", dir, "%.3f");
                    changed |= ImGui::SliderFloat("Radius##spot_radius", &sl.radius, 0.1f, 1000.0f);
                    changed |= ImGui::SliderFloat("Inner angle (deg)##spot_inner", &sl.inner_angle_deg, 0.0f, 89.0f);
                    changed |= ImGui::SliderFloat("Outer angle (deg)##spot_outer", &sl.outer_angle_deg, 0.0f, 89.9f);
                    changed |= ImGui::ColorEdit3("Color##spot_color", col);
                    changed |= ImGui::SliderFloat("Intensity##spot_intensity", &sl.intensity, 0.0f, 100.0f);

                    if (changed)
                    {
                        sl.position_world = WorldVec3(pos[0], pos[1], pos[2]);
                        glm::vec3 d{dir[0], dir[1], dir[2]};
                        sl.direction = (glm::length(d) > 1.0e-6f) ? glm::normalize(d) : glm::vec3(0.0f, -1.0f, 0.0f);
                        sl.color = glm::vec3(col[0], col[1], col[2]);
                        sl.inner_angle_deg = std::clamp(sl.inner_angle_deg, 0.0f, 89.0f);
                        sl.outer_angle_deg = std::clamp(sl.outer_angle_deg, sl.inner_angle_deg, 89.9f);
                        sceneMgr->setSpotLight(static_cast<size_t>(selectedSpot), sl);
                    }

                    if (ImGui::Button("Remove selected spot light##spot_remove"))
                    {
                        sceneMgr->removeSpotLight(static_cast<size_t>(selectedSpot));
                        selectedSpot = -1;
                    }
                }
            }

            ImGui::Separator();
            ImGui::TextUnformatted("Add spot light");
            static double newSpotPos[3] = {0.0, 2.0, 0.0};
            static float newSpotDir[3] = {0.0f, -1.0f, 0.0f};
            static float newSpotRadius = 10.0f;
            static float newSpotInner = 15.0f;
            static float newSpotOuter = 25.0f;
            static float newSpotColor[3] = {1.0f, 1.0f, 1.0f};
            static float newSpotIntensity = 10.0f;

            ImGui::InputScalarN("New position (world)##spot_new_pos", ImGuiDataType_Double, newSpotPos, 3, nullptr, nullptr, "%.3f");
            ImGui::InputFloat3("New direction##spot_new_dir", newSpotDir, "%.3f");
            ImGui::SliderFloat("New radius##spot_new_radius", &newSpotRadius, 0.1f, 1000.0f);
            ImGui::SliderFloat("New inner angle (deg)##spot_new_inner", &newSpotInner, 0.0f, 89.0f);
            ImGui::SliderFloat("New outer angle (deg)##spot_new_outer", &newSpotOuter, 0.0f, 89.9f);
            if (newSpotInner > newSpotOuter)
            {
                newSpotOuter = newSpotInner;
            }
            ImGui::ColorEdit3("New color##spot_new_color", newSpotColor);
            ImGui::SliderFloat("New intensity##spot_new_intensity", &newSpotIntensity, 0.0f, 100.0f);

            if (ImGui::Button("Add spot light##spot_add"))
            {
                SceneManager::SpotLight sl{};
                sl.position_world = WorldVec3(newSpotPos[0], newSpotPos[1], newSpotPos[2]);
                glm::vec3 d{newSpotDir[0], newSpotDir[1], newSpotDir[2]};
                sl.direction = (glm::length(d) > 1.0e-6f) ? glm::normalize(d) : glm::vec3(0.0f, -1.0f, 0.0f);
                sl.radius = newSpotRadius;
                sl.color = glm::vec3(newSpotColor[0], newSpotColor[1], newSpotColor[2]);
                sl.intensity = newSpotIntensity;
                sl.inner_angle_deg = std::clamp(newSpotInner, 0.0f, 89.0f);
                sl.outer_angle_deg = std::clamp(newSpotOuter, sl.inner_angle_deg, 89.9f);

                const size_t oldCount = sceneMgr->getSpotLightCount();
                sceneMgr->addSpotLight(sl);
                selectedSpot = static_cast<int>(oldCount);
            }

            if (ImGui::Button("Clear all spot lights##spot_clear"))
            {
                sceneMgr->clearSpotLights();
                selectedSpot = -1;
            }
        }

        ImGui::Separator();
        // Delete selected model/primitive (uses last pick if valid, otherwise hover)
        static std::string deleteStatus;
        if (ImGui::Button("Delete selected"))
        {
            deleteStatus.clear();
            const PickingSystem::PickInfo *pick = nullptr;
            if (picking)
            {
                const auto &last = picking->last_pick();
                const auto &hover = picking->hover_pick();
                pick = last.valid ? &last : (hover.valid ? &hover : nullptr);
            }
            if (!pick || pick->ownerName.empty())
            {
                deleteStatus = "No selection to delete.";
            }
            else if (pick->ownerType == RenderObject::OwnerType::MeshInstance)
            {
                bool ok = eng->_sceneManager->removeMeshInstance(pick->ownerName);
                if (ok && picking)
                {
                    picking->clear_owner_picks(RenderObject::OwnerType::MeshInstance, pick->ownerName);
                }
                deleteStatus = ok ? "Removed mesh instance: " + pick->ownerName
                                  : "Mesh instance not found: " + pick->ownerName;
            }
            else if (pick->ownerType == RenderObject::OwnerType::GLTFInstance)
            {
                bool ok = eng->_sceneManager->removeGLTFInstance(pick->ownerName);
                if (ok)
                {
                    deleteStatus = "Removed glTF instance: " + pick->ownerName;
                    if (picking)
                    {
                        picking->clear_owner_picks(RenderObject::OwnerType::GLTFInstance, pick->ownerName);
                    }
                }
                else
                {
                    deleteStatus = "glTF instance not found: " + pick->ownerName;
                }
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

        if (picking && picking->last_pick().valid)
        {
            const auto &last = picking->last_pick();
            const char *meshName = last.mesh ? last.mesh->name.c_str() : "<unknown>";
            const char *sceneName = "<none>";
            if (last.scene && !last.scene->debugName.empty())
            {
                sceneName = last.scene->debugName.c_str();
            }
            ImGui::Text("Last pick scene: %s", sceneName);
            ImGui::Text("Last pick source: %s",
                        picking->use_id_buffer_picking() ? "ID buffer" : "CPU raycast");
            ImGui::Text("Last pick object ID: %u", picking->last_pick_object_id());
            ImGui::Text("Last pick mesh: %s (surface %u)", meshName, last.surfaceIndex);
            ImGui::Text("World pos: (%.3f, %.3f, %.3f)",
                        last.worldPos.x,
                        last.worldPos.y,
                        last.worldPos.z);
            const char *ownerTypeStr = "none";
            switch (last.ownerType)
            {
                case RenderObject::OwnerType::MeshInstance: ownerTypeStr = "mesh instance"; break;
                case RenderObject::OwnerType::GLTFInstance: ownerTypeStr = "glTF instance"; break;
                case RenderObject::OwnerType::StaticGLTF: ownerTypeStr = "glTF scene"; break;
                default: break;
            }
            const char *ownerName = last.ownerName.empty() ? "<unnamed>" : last.ownerName.c_str();
            ImGui::Text("Owner: %s (%s)", ownerName, ownerTypeStr);
            ImGui::Text("Indices: first=%u count=%u",
                        last.firstIndex,
                        last.indexCount);

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
        if (picking && picking->hover_pick().valid)
        {
            const auto &hover = picking->hover_pick();
            const char *meshName = hover.mesh ? hover.mesh->name.c_str() : "<unknown>";
            ImGui::Text("Hover mesh: %s (surface %u)", meshName, hover.surfaceIndex);
            const char *ownerTypeStr = "none";
            switch (hover.ownerType)
            {
                case RenderObject::OwnerType::MeshInstance: ownerTypeStr = "mesh instance"; break;
                case RenderObject::OwnerType::GLTFInstance: ownerTypeStr = "glTF instance"; break;
                case RenderObject::OwnerType::StaticGLTF: ownerTypeStr = "glTF scene"; break;
                default: break;
            }
            const char *ownerName = hover.ownerName.empty() ? "<unnamed>" : hover.ownerName.c_str();
            ImGui::Text("Hover owner: %s (%s)", ownerName, ownerTypeStr);
        }
        else
        {
            ImGui::TextUnformatted("Hover: <none>");
        }
        if (picking && !picking->drag_selection().empty())
        {
            ImGui::Text("Drag selection: %zu objects", picking->drag_selection().size());
        }

        ImGui::Separator();
        ImGui::TextUnformatted("Object Gizmo (ImGuizmo)");

        if (!eng->_sceneManager)
        {
            ImGui::TextUnformatted("SceneManager not available");
            return;
        }

        SceneManager *sceneMgr = eng->_sceneManager.get();

        // Choose a pick to edit: prefer last pick, then hover.
        PickingSystem::PickInfo *pick = nullptr;
        if (picking)
        {
            if (picking->last_pick().valid)
            {
                pick = picking->mutable_last_pick();
            }
            else if (picking->hover_pick().valid)
            {
                pick = picking->mutable_hover_pick();
            }
        }

        if (!pick || pick->ownerName.empty())
        {
            ImGui::TextUnformatted("No selection for gizmo (pick or hover an instance).");
            return;
        }

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
            GLTFInstance
        };
        GizmoTarget target = GizmoTarget::None;

        if (pick->ownerType == RenderObject::OwnerType::MeshInstance)
        {
            if (sceneMgr->getMeshInstanceTransformLocal(pick->ownerName, targetTransform))
            {
                target = GizmoTarget::MeshInstance;
                ImGui::Text("Editing mesh instance: %s", pick->ownerName.c_str());
            }
        }
        else if (pick->ownerType == RenderObject::OwnerType::GLTFInstance)
        {
            if (sceneMgr->getGLTFInstanceTransformLocal(pick->ownerName, targetTransform))
            {
                target = GizmoTarget::GLTFInstance;
                ImGui::Text("Editing glTF instance: %s", pick->ownerName.c_str());
            }
        }

        if (target == GizmoTarget::None)
        {
            ImGui::TextUnformatted("Gizmo only supports dynamic mesh/glTF instances.");
            return;
        }

        ImGuiIO &io = ImGui::GetIO();
        ImGuizmo::SetOrthographic(false);

        VkExtent2D swapExtent = eng->_swapchainManager
                                ? eng->_swapchainManager->swapchainExtent()
                                : VkExtent2D{1, 1};
        VkExtent2D drawExtent{
            static_cast<uint32_t>(static_cast<float>(eng->_logicalRenderExtent.width) * eng->renderScale),
            static_cast<uint32_t>(static_cast<float>(eng->_logicalRenderExtent.height) * eng->renderScale)
        };
        if (drawExtent.width == 0 || drawExtent.height == 0)
        {
            drawExtent = VkExtent2D{1, 1};
        }

        VkRect2D activeRect = vkutil::compute_letterbox_rect(drawExtent, swapExtent);
        const float fbScaleX = (io.DisplayFramebufferScale.x > 0.0f) ? io.DisplayFramebufferScale.x : 1.0f;
        const float fbScaleY = (io.DisplayFramebufferScale.y > 0.0f) ? io.DisplayFramebufferScale.y : 1.0f;
        const float rectX = static_cast<float>(activeRect.offset.x) / fbScaleX;
        const float rectY = static_cast<float>(activeRect.offset.y) / fbScaleY;
        const float rectW = static_cast<float>(activeRect.extent.width) / fbScaleX;
        const float rectH = static_cast<float>(activeRect.extent.height) / fbScaleY;

        ImGuizmo::SetDrawlist();
        ImGuizmo::SetRect(rectX, rectY, rectW, rectH);

        // Build a distance-based perspective projection for ImGuizmo instead of
        // using the engine's reversed-Z Vulkan projection.
        Camera &cam = sceneMgr->getMainCamera();
        float fovRad = glm::radians(cam.fovDegrees);
        float aspect = drawExtent.height > 0
                       ? static_cast<float>(drawExtent.width) / static_cast<float>(drawExtent.height)
                       : 1.0f;

        // Distance from camera to object; clamp to avoid degenerate planes.
        glm::vec3 camPos = sceneMgr->get_camera_local_position();
        glm::vec3 objPos = glm::vec3(targetTransform[3]);
        float dist = glm::length(objPos - camPos);
        if (!std::isfinite(dist) || dist <= 0.0f)
        {
            dist = 1.0f;
        }

        // Near/far based on distance: keep ratio reasonable for precision.
        float nearPlane = glm::max(0.05f, dist * 0.05f);
        float farPlane = glm::max(nearPlane * 50.0f, dist * 2.0f);

        glm::mat4 view = cam.getViewMatrix(sceneMgr->get_camera_local_position());
        glm::mat4 proj = glm::perspective(fovRad, aspect, nearPlane, farPlane);

        glm::mat4 before = targetTransform;

        ImDrawList* dl = ImGui::GetForegroundDrawList();
        ImGuizmo::SetDrawlist(dl);

        ImGuizmo::SetRect(rectX, rectY, rectW, rectH);
        ImGuizmo::Manipulate(&view[0][0], &proj[0][0],
                             op, mode,
                             &targetTransform[0][0]);

        bool changed = false;
        for (int c = 0; c < 4 && !changed; ++c)
        {
            for (int r = 0; r < 4; ++r)
            {
                if (before[c][r] != targetTransform[c][r])
                {
                    changed = true;
                    break;
                }
            }
        }

        if (changed)
        {
            switch (target)
            {
                case GizmoTarget::MeshInstance:
                    sceneMgr->setMeshInstanceTransformLocal(pick->ownerName, targetTransform);
                    break;
                case GizmoTarget::GLTFInstance:
                    sceneMgr->setGLTFInstanceTransformLocal(pick->ownerName, targetTransform);
                    break;
                default:
                    break;
            }

            // Keep pick debug info roughly in sync.
            pick->worldTransform = targetTransform;
            pick->worldPos = local_to_world(glm::vec3(targetTransform[3]), sceneMgr->get_world_origin());
        }
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
            if (ImGui::BeginTabItem("Window"))
            {
                ui_window(eng);
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("Background"))
            {
                ui_background(eng);
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("Particles"))
            {
                ui_particles(eng);
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
            if (ImGui::BeginTabItem("Async Assets"))
            {
                ui_async_assets(eng);
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
