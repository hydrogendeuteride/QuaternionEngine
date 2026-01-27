// ImGui debug UI helpers for VulkanEngine.
//
// This file contains the immediate-mode ImGui widgets that expose engine
// statistics, render-graph inspection, texture streaming controls, etc.
// The main frame loop in vk_engine.cpp simply calls vk_engine_draw_debug_ui().

#include "engine.h"
#include "core/picking/picking_system.h"
#include "core/debug_draw/debug_draw.h"

#include "SDL2/SDL.h"
#include "SDL2/SDL_vulkan.h"

#include "imgui.h"
#include "ImGuizmo.h"

#include "render/primitives.h"
#include "vk_mem_alloc.h"
#include "render/passes/tonemap.h"
#include "render/passes/auto_exposure.h"
#include "render/passes/fxaa.h"
#include "render/passes/background.h"
#include "render/passes/particles.h"
#include <glm/gtx/euler_angles.hpp>
#include <glm/gtx/quaternion.hpp>
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
#include <string>

#include "mesh_bvh.h"
#include "scene/planet/planet_system.h"

namespace
{
    static IBLPaths resolve_ibl_paths(VulkanEngine *eng, const IBLPaths &paths)
    {
        IBLPaths out = paths;
        if (!eng || !eng->_assetManager) return out;

        if (!out.specularCube.empty()) out.specularCube = eng->_assetManager->assetPath(out.specularCube);
        if (!out.diffuseCube.empty()) out.diffuseCube = eng->_assetManager->assetPath(out.diffuseCube);
        if (!out.brdfLut2D.empty()) out.brdfLut2D = eng->_assetManager->assetPath(out.brdfLut2D);
        if (!out.background2D.empty()) out.background2D = eng->_assetManager->assetPath(out.background2D);
        return out;
    }

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
            ImGui::SameLine();
            if (ImGui::Button("Delete"))
            {
                const int idx = static_cast<int>(i);
                if (eng->_activeIBLVolume == idx)
                {
                    eng->_activeIBLVolume = -1;
                }
                else if (eng->_activeIBLVolume > idx)
                {
                    eng->_activeIBLVolume -= 1;
                }

                if (eng->_pendingIBLRequest.active)
                {
                    if (eng->_pendingIBLRequest.targetVolume == idx)
                    {
                        eng->_pendingIBLRequest.active = false;
                    }
                    else if (eng->_pendingIBLRequest.targetVolume > idx)
                    {
                        eng->_pendingIBLRequest.targetVolume -= 1;
                    }
                }

                eng->_iblVolumes.erase(eng->_iblVolumes.begin() + idx);
                ImGui::PopID();
                break;
            }
            ImGui::Checkbox("Enabled", &vol.enabled);
            {
                double c[3] = {vol.center_world.x, vol.center_world.y, vol.center_world.z};
                if (ImGui::InputScalarN("Center (world)", ImGuiDataType_Double, c, 3, nullptr, nullptr, "%.3f"))
                {
                    vol.center_world = WorldVec3(c[0], c[1], c[2]);
                }
            }
            {
                const char* shape_items[] = {"Box", "Sphere"};
                int shape_idx = (vol.shape == VulkanEngine::IBLVolumeShape::Sphere) ? 1 : 0;
                if (ImGui::Combo("Shape", &shape_idx, shape_items, IM_ARRAYSIZE(shape_items)))
                {
                    VulkanEngine::IBLVolumeShape new_shape =
                        (shape_idx == 1) ? VulkanEngine::IBLVolumeShape::Sphere : VulkanEngine::IBLVolumeShape::Box;
                    if (new_shape != vol.shape)
                    {
                        if (new_shape == VulkanEngine::IBLVolumeShape::Sphere)
                        {
                            vol.radius = std::max({vol.halfExtents.x, vol.halfExtents.y, vol.halfExtents.z});
                        }
                        else
                        {
                            vol.halfExtents = glm::vec3(vol.radius);
                        }
                        vol.shape = new_shape;
                    }
                }
            }
            if (vol.shape == VulkanEngine::IBLVolumeShape::Sphere)
            {
                ImGui::InputFloat("Radius", &vol.radius);
                vol.radius = std::max(vol.radius, 0.0f);
            }
            else
            {
                ImGui::InputFloat3("Half Extents", &vol.halfExtents.x);
            }

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
                    vol.paths = resolve_ibl_paths(eng, vol.paths);
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
                vol.paths = resolve_ibl_paths(eng, vol.paths);
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
        float fps = (eng->stats.frametime > 0.0f) ? (1000.0f / eng->stats.frametime) : 0.0f;
        ImGui::Text("frametime %.2f ms (%.1f FPS)", eng->stats.frametime, fps);
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

    static void ui_camera(VulkanEngine *eng)
    {
        if (!eng || !eng->_sceneManager)
        {
            ImGui::TextUnformatted("SceneManager not available");
            return;
        }

        SceneManager *sceneMgr = eng->_sceneManager.get();
        CameraRig &rig = sceneMgr->getCameraRig();
        Camera &cam = sceneMgr->getMainCamera();

        // Mode switch
        static const char *k_mode_names[] = {"Free", "Orbit", "Follow", "Chase", "Fixed"};
        int mode = static_cast<int>(rig.mode());
        if (ImGui::Combo("Mode", &mode, k_mode_names, IM_ARRAYSIZE(k_mode_names)))
        {
            rig.set_mode(static_cast<CameraMode>(mode), *sceneMgr, cam);
            if (eng->_input)
            {
                eng->_input->set_cursor_mode(CursorMode::Normal);
            }
        }

        ImGui::Text("Active mode: %s", rig.mode_name());
        ImGui::Separator();

        // Camera state (world)
        double pos[3] = {cam.position_world.x, cam.position_world.y, cam.position_world.z};
        if (ImGui::InputScalarN("Position (world)", ImGuiDataType_Double, pos, 3, nullptr, nullptr, "%.3f"))
        {
            cam.position_world = WorldVec3(pos[0], pos[1], pos[2]);
        }
        float fov = cam.fovDegrees;
        if (ImGui::SliderFloat("FOV (deg)", &fov, 30.0f, 110.0f))
        {
            cam.fovDegrees = fov;
        }

        WorldVec3 origin = sceneMgr->get_world_origin();
        glm::vec3 camLocal = sceneMgr->get_camera_local_position();
        ImGui::Text("Origin (world): (%.3f, %.3f, %.3f)", origin.x, origin.y, origin.z);
        ImGui::Text("Camera (local): (%.3f, %.3f, %.3f)", camLocal.x, camLocal.y, camLocal.z);

        bool terrain_clamp_enabled = rig.terrain_surface_clamp_enabled();
        if (ImGui::Checkbox("Clamp camera above terrain", &terrain_clamp_enabled))
        {
            rig.set_terrain_surface_clamp_enabled(terrain_clamp_enabled);
        }

        ImGui::BeginDisabled(!terrain_clamp_enabled);
        double terrain_clearance_m = rig.terrain_surface_clearance_m();
        if (ImGui::InputDouble("Terrain surface clearance (m)", &terrain_clearance_m, 0.05, 0.5, "%.3f"))
        {
            rig.set_terrain_surface_clearance_m(terrain_clearance_m);
        }
        ImGui::EndDisabled();

        auto target_from_last_pick = [&](CameraTarget &target) -> bool {
            PickingSystem *picking = eng->picking();
            if (!picking) return false;
            const auto &pick = picking->last_pick();
            if (!pick.valid) return false;

            if (pick.ownerType == RenderObject::OwnerType::MeshInstance)
            {
                // Many procedural objects (planets etc.) tag draws as "MeshInstance" for picking,
                // but they don't exist in SceneManager::dynamicMeshInstances. Only use a
                // MeshInstance camera target if it resolves.
                WorldVec3 t{};
                glm::quat r{};
                glm::vec3 s{};
                if (sceneMgr->getMeshInstanceTRSWorld(pick.ownerName, t, r, s))
                {
                    target.type = CameraTargetType::MeshInstance;
                    target.name = pick.ownerName;
                }
                else if (PlanetSystem *planets = sceneMgr->get_planet_system())
                {
                    if (PlanetSystem::PlanetBody *body = planets->find_body_by_name(pick.ownerName))
                    {
                        target.type = CameraTargetType::MeshInstance;
                        target.name = body->name;
                    }
                    else
                    {
                        target.type = CameraTargetType::WorldPoint;
                        target.world_point = pick.worldPos;
                        target.name.clear();
                    }
                }
                else
                {
                    target.type = CameraTargetType::WorldPoint;
                    target.world_point = pick.worldPos;
                    target.name.clear();
                }
            }
            else if (pick.ownerType == RenderObject::OwnerType::GLTFInstance)
            {
                target.type = CameraTargetType::GLTFInstance;
                target.name = pick.ownerName;
            }
            else
            {
                target.type = CameraTargetType::WorldPoint;
                target.world_point = pick.worldPos;
                target.name.clear();
            }
            return true;
        };

        auto draw_target = [&](const char *id, CameraTarget &target, char *name_buf, size_t name_buf_size) {
            ImGui::PushID(id);
            static const char *k_target_types[] = {"None", "WorldPoint", "MeshInstance", "GLTFInstance"};
            int type = static_cast<int>(target.type);
            if (ImGui::Combo("Target type", &type, k_target_types, IM_ARRAYSIZE(k_target_types)))
            {
                target.type = static_cast<CameraTargetType>(type);
                if (target.type != CameraTargetType::MeshInstance && target.type != CameraTargetType::GLTFInstance)
                {
                    target.name.clear();
                    if (name_buf_size > 0)
                    {
                        name_buf[0] = '\0';
                    }
                }
            }

            if (target.type == CameraTargetType::WorldPoint)
            {
                double p[3] = {target.world_point.x, target.world_point.y, target.world_point.z};
                if (ImGui::InputScalarN("World point", ImGuiDataType_Double, p, 3, nullptr, nullptr, "%.3f"))
                {
                    target.world_point = WorldVec3(p[0], p[1], p[2]);
                }
            }
            else if (target.type == CameraTargetType::MeshInstance || target.type == CameraTargetType::GLTFInstance)
            {
                if (std::strncmp(name_buf, target.name.c_str(), name_buf_size) != 0)
                {
                    std::snprintf(name_buf, name_buf_size, "%s", target.name.c_str());
                }
                ImGui::InputText("Target name", name_buf, name_buf_size);
                target.name = name_buf;
            }

            WorldVec3 tpos{};
            glm::quat trot{};
            bool ok = rig.resolve_target(*sceneMgr, target, tpos, trot);
            ImGui::Text("Resolved: %s", ok ? "yes" : "no");
            if (ok)
            {
                ImGui::Text("Target world: (%.3f, %.3f, %.3f)", tpos.x, tpos.y, tpos.z);
            }
            ImGui::PopID();
        };

        // Free
        if (ImGui::CollapsingHeader("Free", ImGuiTreeNodeFlags_DefaultOpen))
        {
            auto &s = rig.free_settings();
            ImGui::InputFloat("Move speed (u/s)", &s.move_speed);
            s.move_speed = std::clamp(s.move_speed, 0.06f, 300.0f);
            ImGui::InputFloat("Look sensitivity", &s.look_sensitivity);
            ImGui::InputFloat("Roll speed (rad/s)", &s.roll_speed);
            ImGui::TextUnformatted("Roll keys: Q/E");
        }

        // Orbit
        if (ImGui::CollapsingHeader("Orbit"))
        {
            auto &s = rig.orbit_settings();
            static char orbitName[128] = "";
            draw_target("orbit_target", s.target, orbitName, IM_ARRAYSIZE(orbitName));
            if (ImGui::Button("Orbit target = Last Pick"))
            {
                target_from_last_pick(s.target);
            }
            ImGui::InputDouble("Distance", &s.distance, 0.1, 1.0, "%.3f");
            s.distance = std::clamp(s.distance,
                                    OrbitCameraSettings::kMinDistance,
                                    OrbitCameraSettings::kMaxDistance);
            float yawDeg = glm::degrees(s.yaw);
            float pitchDeg = glm::degrees(s.pitch);
            if (ImGui::SliderFloat("Yaw (deg)", &yawDeg, -180.0f, 180.0f))
            {
                s.yaw = glm::radians(yawDeg);
            }
            if (ImGui::SliderFloat("Pitch (deg)", &pitchDeg, -89.0f, 89.0f))
            {
                s.pitch = glm::radians(pitchDeg);
            }
            ImGui::InputFloat("Look sensitivity##orbit", &s.look_sensitivity);

            ImGui::Separator();
            ImGui::Text("Reference Up Vector");
            ImGui::InputFloat3("Up##orbit_up", &s.reference_up.x);
            if (ImGui::Button("Normalize Up"))
            {
                rig.set_orbit_reference_up(s.reference_up);
            }
            ImGui::SameLine();
            if (ImGui::Button("Reset to World Y"))
            {
                rig.set_orbit_reference_up(glm::vec3(0.0f, 1.0f, 0.0f));
            }
            if (ImGui::Button("Align Up to Target"))
            {
                rig.align_orbit_up_to_target();
            }
        }

        // Follow
        if (ImGui::CollapsingHeader("Follow"))
        {
            auto &s = rig.follow_settings();
            static char followName[128] = "";
            draw_target("follow_target", s.target, followName, IM_ARRAYSIZE(followName));
            if (ImGui::Button("Follow target = Last Pick"))
            {
                target_from_last_pick(s.target);
            }
            ImGui::InputFloat3("Position offset (local)", &s.position_offset_local.x);

            glm::vec3 rotDeg = glm::degrees(glm::eulerAngles(s.rotation_offset));
            float r[3] = {rotDeg.x, rotDeg.y, rotDeg.z};
            if (ImGui::InputFloat3("Rotation offset (deg XYZ)", r))
            {
                glm::mat4 R = glm::eulerAngleXYZ(glm::radians(r[0]), glm::radians(r[1]), glm::radians(r[2]));
                s.rotation_offset = glm::quat_cast(R);
            }
            if (ImGui::Button("Reset rotation offset"))
            {
                s.rotation_offset = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
            }
        }

        // Chase
        if (ImGui::CollapsingHeader("Chase"))
        {
            auto &s = rig.chase_settings();
            static char chaseName[128] = "";
            draw_target("chase_target", s.target, chaseName, IM_ARRAYSIZE(chaseName));
            if (ImGui::Button("Chase target = Last Pick"))
            {
                target_from_last_pick(s.target);
            }
            ImGui::InputFloat3("Position offset (local)##chase", &s.position_offset_local.x);

            glm::vec3 rotDeg = glm::degrees(glm::eulerAngles(s.rotation_offset));
            float r[3] = {rotDeg.x, rotDeg.y, rotDeg.z};
            if (ImGui::InputFloat3("Rotation offset (deg XYZ)##chase", r))
            {
                glm::mat4 R = glm::eulerAngleXYZ(glm::radians(r[0]), glm::radians(r[1]), glm::radians(r[2]));
                s.rotation_offset = glm::quat_cast(R);
            }

            ImGui::SliderFloat("Position lag (1/s)", &s.position_lag, 0.0f, 30.0f);
            ImGui::SliderFloat("Rotation lag (1/s)", &s.rotation_lag, 0.0f, 30.0f);
        }

        // Fixed
        if (ImGui::CollapsingHeader("Fixed"))
        {
            ImGui::TextUnformatted("Fixed mode does not modify the camera automatically.");
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
        ImGui::TextUnformatted("Analytic Planet Shadow");
        ImGui::SliderFloat("Sun angular radius (deg)", &ss.planetSunAngularRadiusDeg, 0.0f, 2.0f, "%.3f");
        ImGui::TextWrapped("Soft penumbra for planet->scene directional shadows. Set to 0 for a hard edge.");

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
        ImGui::TextUnformatted("Volumetrics");
        bool voxEnabled = ctx->enableVolumetrics;
        if (ImGui::Checkbox("Enable Voxel Volumetrics (Cloud/Smoke/Flame)", &voxEnabled))
        {
            ctx->enableVolumetrics = voxEnabled;
        }

        const char *typeLabels[] = {"Clouds", "Smoke", "Flame"};

        for (uint32_t i = 0; i < EngineContext::MAX_VOXEL_VOLUMES; ++i)
        {
            VoxelVolumeSettings &vs = ctx->voxelVolumes[i];

            std::string header = "Voxel Volume " + std::to_string(i);
            if (!ImGui::TreeNode(header.c_str()))
            {
                continue;
            }

            std::string id = "##vox" + std::to_string(i);
            ImGui::Checkbox(("Enabled" + id).c_str(), &vs.enabled);

            int type = static_cast<int>(vs.type);
            if (ImGui::Combo(("Type" + id).c_str(), &type, typeLabels, IM_ARRAYSIZE(typeLabels)))
            {
                type = std::clamp(type, 0, 2);
                vs.type = static_cast<VoxelVolumeType>(type);
            }

            ImGui::Checkbox(("Follow Camera XZ" + id).c_str(), &vs.followCameraXZ);
            ImGui::Checkbox(("Animate Voxels" + id).c_str(), &vs.animateVoxels);

            if (vs.followCameraXZ)
            {
                ImGui::InputFloat3(("Volume Offset (local)" + id).c_str(), &vs.volumeCenterLocal.x);
            }
            else
            {
                ImGui::InputFloat3(("Volume Center (local)" + id).c_str(), &vs.volumeCenterLocal.x);
            }
            ImGui::InputFloat3(("Volume Velocity (local)" + id).c_str(), &vs.volumeVelocityLocal.x);
            ImGui::InputFloat3(("Volume Half Extents" + id).c_str(), &vs.volumeHalfExtents.x);
            vs.volumeHalfExtents.x = std::max(vs.volumeHalfExtents.x, 0.01f);
            vs.volumeHalfExtents.y = std::max(vs.volumeHalfExtents.y, 0.01f);
            vs.volumeHalfExtents.z = std::max(vs.volumeHalfExtents.z, 0.01f);

            ImGui::SliderFloat(("Density Scale" + id).c_str(), &vs.densityScale, 0.0f, 6.0f);
            ImGui::SliderFloat(("Coverage" + id).c_str(), &vs.coverage, 0.0f, 0.95f);
            ImGui::SliderFloat(("Extinction" + id).c_str(), &vs.extinction, 0.0f, 8.0f);
            ImGui::SliderInt(("Steps" + id).c_str(), &vs.stepCount, 8, 256);

            int gridRes = static_cast<int>(vs.gridResolution);
            if (ImGui::SliderInt(("Grid Resolution" + id).c_str(), &gridRes, 16, 128))
            {
                vs.gridResolution = static_cast<uint32_t>(std::max(4, gridRes));
            }

            if (vs.animateVoxels)
            {
                ImGui::InputFloat3(("Wind Velocity (local)" + id).c_str(), &vs.windVelocityLocal.x);
                ImGui::SliderFloat(("Dissipation" + id).c_str(), &vs.dissipation, 0.0f, 6.0f);
                ImGui::SliderFloat(("Noise Strength" + id).c_str(), &vs.noiseStrength, 0.0f, 6.0f);
                ImGui::SliderFloat(("Noise Scale" + id).c_str(), &vs.noiseScale, 0.25f, 32.0f);
                ImGui::SliderFloat(("Noise Speed" + id).c_str(), &vs.noiseSpeed, 0.0f, 8.0f);

                if (vs.type != VoxelVolumeType::Clouds)
                {
                    ImGui::InputFloat3(("Emitter UVW" + id).c_str(), &vs.emitterUVW.x);
                    ImGui::SliderFloat(("Emitter Radius" + id).c_str(), &vs.emitterRadius, 0.01f, 0.5f);
                }
            }

            ImGui::ColorEdit3(("Albedo/Tint" + id).c_str(), &vs.albedo.x);
            ImGui::SliderFloat(("Scatter Strength" + id).c_str(), &vs.scatterStrength, 0.0f, 2.0f);

            if (vs.type == VoxelVolumeType::Flame)
            {
                ImGui::ColorEdit3(("Emission Color" + id).c_str(), &vs.emissionColor.x);
                ImGui::SliderFloat(("Emission Strength" + id).c_str(), &vs.emissionStrength, 0.0f, 25.0f);
            }
            else
            {
                vs.emissionStrength = 0.0f;
            }

            ImGui::TreePop();
        }

        ImGui::Separator();
        ImGui::TextUnformatted("Atmosphere");
        bool atmEnabled = ctx->enableAtmosphere;
        if (ImGui::Checkbox("Enable Atmosphere Scattering", &atmEnabled))
        {
            ctx->enableAtmosphere = atmEnabled;
        }

        AtmosphereSettings &atm = ctx->atmosphere;
        if (ImGui::BeginCombo("Planet", atm.bodyName.empty() ? "Auto (closest)" : atm.bodyName.c_str()))
        {
            const bool autoSelected = atm.bodyName.empty();
            if (ImGui::Selectable("Auto (closest)", autoSelected))
            {
                atm.bodyName.clear();
            }

            if (ctx->scene)
            {
                if (PlanetSystem *planets = ctx->scene->get_planet_system())
                {
                    for (const PlanetSystem::PlanetBody &b : planets->bodies())
                    {
                        const bool selected = (!atm.bodyName.empty() && atm.bodyName == b.name);
                        if (ImGui::Selectable(b.name.c_str(), selected))
                        {
                            atm.bodyName = b.name;
                        }
                    }
                }
            }

            ImGui::EndCombo();
        }

        if (ImGui::Button("Reset Earth Params"))
        {
            std::string keepName = atm.bodyName;
            atm = AtmosphereSettings{};
            atm.bodyName = std::move(keepName);
        }

        float atmHeightKm = atm.atmosphereHeightM / 1000.0f;
        float HrKm = atm.rayleighScaleHeightM / 1000.0f;
        float HmKm = atm.mieScaleHeightM / 1000.0f;

        if (ImGui::SliderFloat("Atmosphere Height (km)", &atmHeightKm, 1.0f, 200.0f, "%.2f"))
        {
            atm.atmosphereHeightM = std::max(0.0f, atmHeightKm * 1000.0f);
        }
        if (ImGui::SliderFloat("Rayleigh Scale Height (km)", &HrKm, 1.0f, 20.0f, "%.2f"))
        {
            atm.rayleighScaleHeightM = std::max(1.0f, HrKm * 1000.0f);
        }
        if (ImGui::SliderFloat("Mie Scale Height (km)", &HmKm, 0.1f, 10.0f, "%.2f"))
        {
            atm.mieScaleHeightM = std::max(1.0f, HmKm * 1000.0f);
        }

        ImGui::SliderFloat("Mie g", &atm.mieG, -0.2f, 0.99f, "%.3f");
        ImGui::SliderFloat("Intensity", &atm.intensity, 0.0f, 4.0f, "%.2f");
        ImGui::SliderFloat("Sun Disk", &atm.sunDiskIntensity, 0.0f, 10.0f, "%.2f");
        ImGui::SliderFloat("Sun Halo", &atm.sunHaloIntensity, 0.0f, 4.0f, "%.2f");
        ImGui::SliderFloat("Halo Radius (deg)", &atm.sunHaloRadiusDeg, 0.0f, 20.0f, "%.2f");
        ImGui::SliderFloat("Sun Starburst", &atm.sunStarburstIntensity, 0.0f, 4.0f, "%.2f");
        ImGui::SliderFloat("Starburst Radius (deg)", &atm.sunStarburstRadiusDeg, 0.0f, 30.0f, "%.2f");
        int spikes = std::clamp(atm.sunStarburstSpikes, 2, 64);
        if (ImGui::SliderInt("Starburst Spikes", &spikes, 2, 32))
        {
            // Keep it even by default to produce a symmetrical ray pattern.
            spikes = std::max(2, spikes);
            spikes &= ~1;
            atm.sunStarburstSpikes = spikes;
        }
        ImGui::SliderFloat("Starburst Sharpness", &atm.sunStarburstSharpness, 1.0f, 64.0f, "%.1f");
        ImGui::SliderFloat("Jitter", &atm.jitterStrength, 0.0f, 1.0f, "%.2f");
        ImGui::SliderFloat("Planet Snap (m)", &atm.planetSurfaceSnapM, 0.0f, 2000.0f, "%.1f");
        ImGui::SliderInt("View Steps", &atm.viewSteps, 4, 64);
        ImGui::SliderInt("Light Steps", &atm.lightSteps, 2, 32);

        ImGui::Separator();
        if (auto *tm = eng->_renderPassManager ? eng->_renderPassManager->getPass<TonemapPass>() : nullptr)
        {
            AutoExposurePass *ae = eng->_renderPassManager->getPass<AutoExposurePass>();
            bool autoExpEnabled = (ae != nullptr) ? ae->enabled() : false;
            if (ae)
            {
                if (ImGui::Checkbox("Auto Exposure", &autoExpEnabled))
                {
                    ae->set_enabled(autoExpEnabled, tm->exposure());
                }
                if (autoExpEnabled)
                {
                    float comp = ae->compensation();
                    if (ImGui::SliderFloat("Exposure Compensation", &comp, 0.05f, 8.0f, "%.3f"))
                    {
                        ae->set_compensation(comp);
                    }
                    float key = ae->key_value();
                    if (ImGui::SliderFloat("Key (middle grey)", &key, 0.02f, 0.5f, "%.3f"))
                    {
                        ae->set_key_value(key);
                    }
                    float minExp = ae->min_exposure();
                    if (ImGui::SliderFloat("Min Exposure", &minExp, 0.01f, 8.0f, "%.3f"))
                    {
                        ae->set_min_exposure(minExp);
                    }
                    float maxExp = ae->max_exposure();
                    if (ImGui::SliderFloat("Max Exposure", &maxExp, 0.01f, 16.0f, "%.3f"))
                    {
                        ae->set_max_exposure(maxExp);
                    }
                    float speedUp = ae->speed_up();
                    if (ImGui::SliderFloat("Speed Up", &speedUp, 0.0f, 10.0f, "%.2f"))
                    {
                        ae->set_speed_up(speedUp);
                    }
                    float speedDown = ae->speed_down();
                    if (ImGui::SliderFloat("Speed Down", &speedDown, 0.0f, 10.0f, "%.2f"))
                    {
                        ae->set_speed_down(speedDown);
                    }

                    ImGui::Text("Lavg %.4f", ae->last_luminance());
                    ImGui::Text("Exposure %.3f (target %.3f)", ae->exposure(), ae->target_exposure());
                }
            }

            float exp = tm->exposure();
            int mode = tm->mode();
            if (!autoExpEnabled)
            {
                if (ImGui::SliderFloat("Exposure", &exp, 0.05f, 8.0f)) { tm->setExposure(exp); }
            }
            else
            {
                ImGui::Text("Exposure (auto) %.3f", exp);
            }
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

    // Scene editor - spawn and delete instances
    static void ui_scene_editor(VulkanEngine *eng)
    {
        if (!eng || !eng->_sceneManager)
        {
            ImGui::TextUnformatted("SceneManager not available");
            return;
        }

        SceneManager *sceneMgr = eng->_sceneManager.get();
        PickingSystem *picking = eng->picking();

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
    }

    // Lights editor (Point + Spot lights)
    static void ui_lights(VulkanEngine *eng)
    {
        if (!eng || !eng->_sceneManager)
        {
            ImGui::TextUnformatted("SceneManager not available");
            return;
        }

        SceneManager *sceneMgr = eng->_sceneManager.get();

        // Sunlight editor (Directional light)
        ImGui::TextUnformatted("Sunlight (directional)");
        {
            // Shaders use Lsun = normalize(-sceneData.sunlightDirection.xyz).
            // Expose Lsun directly in the UI (direction *towards* the sun).
            glm::vec3 Lsun = -sceneMgr->getSunlightDirection();
            const float len2 = glm::length2(Lsun);
            Lsun = (len2 > 1.0e-8f) ? glm::normalize(Lsun) : glm::vec3(0.0f, 1.0f, 0.0f);

            float dir[3] = {Lsun.x, Lsun.y, Lsun.z};
            if (ImGui::InputFloat3("Direction (to sun)##sun_dir", dir, "%.3f"))
            {
                glm::vec3 v{dir[0], dir[1], dir[2]};
                const float vlen2 = glm::length2(v);
                v = (vlen2 > 1.0e-8f) ? glm::normalize(v) : glm::vec3(0.0f, 1.0f, 0.0f);
                sceneMgr->setSunlightDirection(-v);
            }

            if (ImGui::Button("Reset##sun_reset"))
            {
                sceneMgr->setSunlightDirection(glm::vec3(-0.2f, -1.0f, -0.3f));
            }
        }

        ImGui::Separator();

        // Point light editor
        ImGui::TextUnformatted("Point lights");
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

    // Picking & Gizmo - picking info and transform editor
    static void ui_picking_gizmo(VulkanEngine *eng)
    {
        if (!eng || !eng->_sceneManager)
        {
            ImGui::TextUnformatted("SceneManager not available");
            return;
        }

        SceneManager *sceneMgr = eng->_sceneManager.get();
        PickingSystem *picking = eng->picking();

        // Last pick info
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
            GLTFInstance,
            Planet
        };
        GizmoTarget target = GizmoTarget::None;

        if (pick->ownerType == RenderObject::OwnerType::MeshInstance)
        {
            if (sceneMgr->getMeshInstanceTransformLocal(pick->ownerName, targetTransform))
            {
                target = GizmoTarget::MeshInstance;
                ImGui::Text("Editing mesh instance: %s", pick->ownerName.c_str());
            }
            else if (PlanetSystem *planets = sceneMgr->get_planet_system())
            {
                PlanetSystem::PlanetBody *body = planets->find_body_by_name(pick->ownerName);
                if (body)
                {
                    glm::vec3 tLocal = world_to_local(body->center_world, sceneMgr->get_world_origin());
                    targetTransform = make_trs_matrix(tLocal,
                                                      glm::quat(1.0f, 0.0f, 0.0f, 0.0f),
                                                      glm::vec3(1.0f));
                    target = GizmoTarget::Planet;
                    ImGui::Text("Editing planet: %s", pick->ownerName.c_str());
                    ImGui::Text("Radius: %.3f km", body->radius_m / 1000.0);
                    if (op == ImGuizmo::ROTATE)
                    {
                        ImGui::TextUnformatted("Note: planet rotation is not supported (use Translate/Scale).");
                    }
                }
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
                case GizmoTarget::Planet:
                {
                    PlanetSystem *planets = sceneMgr->get_planet_system();
                    if (!planets)
                    {
                        break;
                    }

                    PlanetSystem::PlanetBody *body = planets->find_body_by_name(pick->ownerName);
                    if (!body)
                    {
                        break;
                    }

                    glm::vec3 tLocal{};
                    glm::quat r{};
                    glm::vec3 s{};
                    decompose_trs_matrix(targetTransform, tLocal, r, s);

                    if (op == ImGuizmo::TRANSLATE)
                    {
                        planets->set_planet_center(pick->ownerName, local_to_world(tLocal, sceneMgr->get_world_origin()));
                    }
                    else if (op == ImGuizmo::SCALE)
                    {
                        const double scale =
                            (static_cast<double>(std::abs(s.x)) +
                             static_cast<double>(std::abs(s.y)) +
                             static_cast<double>(std::abs(s.z))) / 3.0;
                        if (std::isfinite(scale) && scale > 0.0)
                        {
                            planets->set_planet_radius(pick->ownerName, body->radius_m * scale);
                        }
                    }
                    break;
                }
                default:
                    break;
            }

            // Keep pick debug info roughly in sync.
            pick->worldTransform = targetTransform;
            pick->worldPos = local_to_world(glm::vec3(targetTransform[3]), sceneMgr->get_world_origin());
        }
    }

    static void ui_planets(VulkanEngine *eng)
    {
        if (!eng || !eng->_sceneManager)
        {
            return;
        }

        SceneManager *scene = eng->_sceneManager.get();
        PlanetSystem *planets = scene->get_planet_system();
        if (!planets)
        {
            ImGui::TextUnformatted("Planet system not available");
            return;
        }

        bool enabled = planets->enabled();
        if (ImGui::Checkbox("Enable planet rendering", &enabled))
        {
            planets->set_enabled(enabled);
        }

        const WorldVec3 origin_world = scene->get_world_origin();
        const WorldVec3 cam_world = scene->getMainCamera().position_world;
        const glm::vec3 cam_local = scene->get_camera_local_position();

        ImGui::Separator();
        ImGui::Text("Camera world (m):  %.3f, %.3f, %.3f", cam_world.x, cam_world.y, cam_world.z);
        ImGui::Text("Camera local (m):  %.3f, %.3f, %.3f", cam_local.x, cam_local.y, cam_local.z);
        ImGui::Text("World origin (m):  %.3f, %.3f, %.3f", origin_world.x, origin_world.y, origin_world.z);

        auto look_at_world = [](Camera &cam, const WorldVec3 &target_world)
        {
            glm::dvec3 dirD = glm::normalize(target_world - cam.position_world);
            glm::vec3 dir = glm::normalize(glm::vec3(dirD));

            glm::vec3 up(0.0f, 1.0f, 0.0f);
            if (glm::length2(glm::cross(dir, up)) < 1e-6f)
            {
                up = glm::vec3(0.0f, 0.0f, 1.0f);
            }

            glm::vec3 f = dir;
            glm::vec3 r = glm::normalize(glm::cross(up, f));
            glm::vec3 u = glm::cross(f, r);

            glm::mat3 rot;
            rot[0] = r;
            rot[1] = u;
            rot[2] = -f; // -Z forward
            cam.orientation = glm::quat_cast(rot);
        };

        const PlanetSystem::PlanetBody *terrain_body = nullptr;
        for (const PlanetSystem::PlanetBody &b : planets->bodies())
        {
            if (b.terrain)
            {
                terrain_body = &b;
                break;
            }
        }

        ImGui::Separator();
        if (terrain_body)
        {
            const std::string label = "Terrain LOD / Perf (" + terrain_body->name + ")";
            if (ImGui::CollapsingHeader(label.c_str(), ImGuiTreeNodeFlags_DefaultOpen))
            {
                auto settings = planets->earth_quadtree_settings();
                bool changed = false;

                bool tint = planets->earth_debug_tint_patches_by_lod();
                if (ImGui::Checkbox("Debug: tint patches by LOD", &tint))
                {
                    planets->set_earth_debug_tint_patches_by_lod(tint);
                }

                int maxLevel = static_cast<int>(settings.max_level);
                if (ImGui::SliderInt("Max LOD level", &maxLevel, 0, 20))
                {
                    settings.max_level = static_cast<uint32_t>(std::max(0, maxLevel));
                    changed = true;
                }

                if (ImGui::SliderFloat("Target SSE (px)", &settings.target_sse_px, 4.0f, 128.0f, "%.1f"))
                {
                    settings.target_sse_px = std::max(settings.target_sse_px, 0.1f);
                    changed = true;
                }

                int maxPatches = static_cast<int>(settings.max_patches_visible);
                if (ImGui::SliderInt("Max visible patches", &maxPatches, 64, 20000))
                {
                    settings.max_patches_visible = static_cast<uint32_t>(std::max(6, maxPatches));
                    changed = true;
                }

                int createBudget = static_cast<int>(planets->earth_patch_create_budget_per_frame());
                if (ImGui::SliderInt("Patch create budget/frame", &createBudget, 0, 512))
                {
                    planets->set_earth_patch_create_budget_per_frame(static_cast<uint32_t>(std::max(0, createBudget)));
                }

                float createBudgetMs = planets->earth_patch_create_budget_ms();
                if (ImGui::DragFloat("Patch create budget (ms)", &createBudgetMs, 0.25f, 0.0f, 50.0f, "%.2f"))
                {
                    planets->set_earth_patch_create_budget_ms(std::max(0.0f, createBudgetMs));
                }

                int patchRes = static_cast<int>(planets->earth_patch_resolution());
                if (ImGui::SliderInt("Patch resolution (verts/edge)", &patchRes, 2, 129))
                {
                    planets->set_earth_patch_resolution(static_cast<uint32_t>(std::max(2, patchRes)));
                }

                int cacheMax = static_cast<int>(planets->earth_patch_cache_max());
                if (ImGui::SliderInt("Patch cache max", &cacheMax, 0, 50000))
                {
                    planets->set_earth_patch_cache_max(static_cast<uint32_t>(std::max(0, cacheMax)));
                }

                if (ImGui::Checkbox("Frustum cull", &settings.frustum_cull)) changed = true;
                if (ImGui::Checkbox("Horizon cull", &settings.horizon_cull)) changed = true;

                if (ImGui::Checkbox("RT guardrail (LOD floor)", &settings.rt_guardrail)) changed = true;
                if (settings.rt_guardrail)
                {
                    float maxEdge = static_cast<float>(settings.max_patch_edge_rt_m);
                    if (ImGui::DragFloat("RT max patch edge (m)", &maxEdge, 100.0f, 0.0f, 200000.0f, "%.0f"))
                    {
                        settings.max_patch_edge_rt_m = static_cast<double>(std::max(0.0f, maxEdge));
                        changed = true;
                    }

                    float maxAlt = static_cast<float>(settings.rt_guardrail_max_altitude_m);
                    if (ImGui::DragFloat("RT max altitude (m)", &maxAlt, 1000.0f, 0.0f, 2.0e6f, "%.0f"))
                    {
                        settings.rt_guardrail_max_altitude_m = static_cast<double>(std::max(0.0f, maxAlt));
                        changed = true;
                    }
                }

                if (changed)
                {
                    planets->set_earth_quadtree_settings(settings);
                }

                const PlanetSystem::EarthDebugStats &s = planets->terrain_debug_stats(terrain_body->name);
                ImGui::Separator();
                ImGui::Text("Visible patches: %u  (rendered: %u | est. tris: %u)",
                            s.visible_patches,
                            s.rendered_patches,
                            s.estimated_triangles);
                ImGui::Text("Cache size: %u  (created this frame: %u)", s.patch_cache_size, s.created_patches);
                ImGui::Text("Quadtree: max level used %u | visited %u | culled %u | budget-limited %u",
                            s.quadtree.max_level_used,
                            s.quadtree.nodes_visited,
                            s.quadtree.nodes_culled,
                            s.quadtree.splits_budget_limited);
                ImGui::Text("CPU ms: quadtree %.2f | create %.2f | emit %.2f | total %.2f",
                            s.ms_quadtree, s.ms_patch_create, s.ms_emit, s.ms_total);
            }
        }
        else
        {
            ImGui::TextUnformatted("No terrain planet active");
        }

        ImGui::Separator();
        if (ImGui::CollapsingHeader("Planet Tools", ImGuiTreeNodeFlags_DefaultOpen))
        {
            if (ImGui::Button("Clear all planets"))
            {
                planets->clear_planets(true);
            }

            static int selected_planet = 0;
            static std::string edit_target_name;
            static bool edit_visible = true;
            static bool edit_terrain = false;
            static double edit_center[3] = {0.0, 0.0, 0.0};
            static double edit_radius_km = 1.0;

            const auto &bodies = planets->bodies();
            if (selected_planet < 0) selected_planet = 0;
            if (!bodies.empty() && selected_planet >= static_cast<int>(bodies.size()))
            {
                selected_planet = static_cast<int>(bodies.size()) - 1;
            }

            if (ImGui::BeginListBox("Planets"))
            {
                for (int i = 0; i < static_cast<int>(bodies.size()); ++i)
                {
                    const PlanetSystem::PlanetBody &b = bodies[i];
                    const bool is_selected = (selected_planet == i);
                    const char *tag = b.terrain ? " (terrain)" : " (mesh)";
                    const std::string label = b.name + tag;
                    if (ImGui::Selectable(label.c_str(), is_selected))
                    {
                        selected_planet = i;
                    }
                }
                ImGui::EndListBox();
            }

            if (!bodies.empty())
            {
                const PlanetSystem::PlanetBody &b = bodies[static_cast<size_t>(selected_planet)];
                if (edit_target_name != b.name)
                {
                    edit_target_name = b.name;
                    edit_visible = b.visible;
                    edit_terrain = b.terrain;
                    edit_center[0] = b.center_world.x;
                    edit_center[1] = b.center_world.y;
                    edit_center[2] = b.center_world.z;
                    edit_radius_km = b.radius_m / 1000.0;
                }

                ImGui::Text("Selected: %s", b.name.c_str());

                if (ImGui::Checkbox("Visible##selected", &edit_visible))
                {
                    planets->set_planet_visible(edit_target_name, edit_visible);
                }
                if (ImGui::Checkbox("Terrain##selected", &edit_terrain))
                {
                    planets->set_planet_terrain(edit_target_name, edit_terrain);
                }

                if (ImGui::InputScalarN("Center (world m)##selected", ImGuiDataType_Double, edit_center, 3, nullptr, nullptr, "%.3f"))
                {
                    planets->set_planet_center(edit_target_name, WorldVec3(edit_center[0], edit_center[1], edit_center[2]));
                }
                if (ImGui::DragScalar("Radius (km)##selected", ImGuiDataType_Double, &edit_radius_km, 10.0f, nullptr, nullptr, "%.3f"))
                {
                    planets->set_planet_radius(edit_target_name, std::max(1.0, edit_radius_km * 1000.0));
                }

                const double dist = glm::length(cam_world - b.center_world);
                const double alt_m = dist - b.radius_m;
                ImGui::Text("Altitude above %s: %.3f km", b.name.c_str(), alt_m / 1000.0);

                if (ImGui::Button("Look at##selected"))
                {
                    look_at_world(scene->getMainCamera(), b.center_world);
                }
                ImGui::SameLine();
                if (ImGui::Button("Teleport: 10000 km above##selected"))
                {
                    scene->getMainCamera().position_world =
                        b.center_world + WorldVec3(0.0, 0.0, b.radius_m + 1.0e7);
                    look_at_world(scene->getMainCamera(), b.center_world);
                }

                if (ImGui::Button("Teleport: 1000 km orbit##selected"))
                {
                    scene->getMainCamera().position_world =
                        b.center_world + WorldVec3(0.0, 0.0, b.radius_m + 1.0e6);
                    look_at_world(scene->getMainCamera(), b.center_world);
                }
                ImGui::SameLine();
                if (ImGui::Button("Teleport: 10 km above##selected"))
                {
                    scene->getMainCamera().position_world =
                        b.center_world + WorldVec3(0.0, 0.0, b.radius_m + 1.0e4);
                    look_at_world(scene->getMainCamera(), b.center_world);
                }

                if (ImGui::Button("Destroy selected"))
                {
                    planets->destroy_planet(edit_target_name);
                    edit_target_name.clear();
                    selected_planet = 0;
                }
            }

            ImGui::Separator();
            ImGui::TextUnformatted("Create mesh planet");

            static char new_name[64] = "Mars";
            static double new_center[3] = {50000000.0, 0.0, 0.0};
            static double new_radius_km = 3390.0;
            static float new_color[4] = {0.8f, 0.35f, 0.25f, 1.0f};
            static float new_metallic = 0.0f;
            static float new_roughness = 1.0f;
            static int new_sectors = 48;
            static int new_stacks = 24;

            ImGui::InputText("Name", new_name, IM_ARRAYSIZE(new_name));
            ImGui::InputScalarN("Center (world m)", ImGuiDataType_Double, new_center, 3, nullptr, nullptr, "%.3f");
            ImGui::DragScalar("Radius (km)", ImGuiDataType_Double, &new_radius_km, 10.0f, nullptr, nullptr, "%.3f");
            ImGui::ColorEdit4("Base color", new_color);
            ImGui::DragFloat("Metallic", &new_metallic, 0.01f, 0.0f, 1.0f, "%.2f");
            ImGui::DragFloat("Roughness", &new_roughness, 0.01f, 0.0f, 1.0f, "%.2f");
            ImGui::SliderInt("Sectors", &new_sectors, 8, 256);
            ImGui::SliderInt("Stacks", &new_stacks, 4, 256);

            if (ImGui::Button("Create"))
            {
                PlanetSystem::MeshPlanetCreateInfo info{};
                info.name = new_name;
                info.center_world = WorldVec3(new_center[0], new_center[1], new_center[2]);
                info.radius_m = std::max(1.0, new_radius_km * 1000.0);
                info.visible = true;
                info.base_color = glm::vec4(new_color[0], new_color[1], new_color[2], new_color[3]);
                info.metallic = std::clamp(new_metallic, 0.0f, 1.0f);
                info.roughness = std::clamp(new_roughness, 0.0f, 1.0f);
                info.sectors = static_cast<uint32_t>(std::max(3, new_sectors));
                info.stacks = static_cast<uint32_t>(std::max(2, new_stacks));
                planets->create_mesh_planet(info);
            }

            ImGui::Separator();
            ImGui::TextUnformatted("Create terrain planet");

            static char terr_name[64] = "Earth2";
            static double terr_center[3] = {0.0, 0.0, 0.0};
            static double terr_radius_km = 6378.137;
            static float terr_color[4] = {1.0f, 1.0f, 1.0f, 1.0f};
            static float terr_metallic = 0.0f;
            static float terr_roughness = 1.0f;
            static char terr_albedo_dir[256] = "planets/earth/albedo/L0";

            ImGui::InputText("Name##terrain_create", terr_name, IM_ARRAYSIZE(terr_name));
            ImGui::InputScalarN("Center (world m)##terrain_create", ImGuiDataType_Double, terr_center, 3, nullptr, nullptr, "%.3f");
            ImGui::DragScalar("Radius (km)##terrain_create", ImGuiDataType_Double, &terr_radius_km, 10.0f, nullptr, nullptr, "%.3f");
            ImGui::ColorEdit4("Base color##terrain_create", terr_color);
            ImGui::DragFloat("Metallic##terrain_create", &terr_metallic, 0.01f, 0.0f, 1.0f, "%.2f");
            ImGui::DragFloat("Roughness##terrain_create", &terr_roughness, 0.01f, 0.0f, 1.0f, "%.2f");
            ImGui::InputText("Albedo dir##terrain_create", terr_albedo_dir, IM_ARRAYSIZE(terr_albedo_dir));

            if (ImGui::Button("Create##terrain_create"))
            {
                PlanetSystem::TerrainPlanetCreateInfo info{};
                info.name = terr_name;
                info.center_world = WorldVec3(terr_center[0], terr_center[1], terr_center[2]);
                info.radius_m = std::max(1.0, terr_radius_km * 1000.0);
                info.visible = true;
                info.base_color = glm::vec4(terr_color[0], terr_color[1], terr_color[2], terr_color[3]);
                info.metallic = std::clamp(terr_metallic, 0.0f, 1.0f);
                info.roughness = std::clamp(terr_roughness, 0.0f, 1.0f);
                info.albedo_dir = terr_albedo_dir;
                planets->create_terrain_planet(info);
            }
        }
    }

    static void ui_debug_draw(VulkanEngine *eng)
    {
        if (!eng || !eng->_context)
        {
            ImGui::TextUnformatted("Engine context not available");
            return;
        }

        DebugDrawSystem *dd = eng->_debugDraw.get();
        if (dd)
        {
            ImGui::SeparatorText("Debug Draw");

            bool enabled = dd->settings().enabled;
            if (ImGui::Checkbox("Enabled", &enabled))
            {
                dd->settings().enabled = enabled;
            }

            ImGui::SameLine();
            ImGui::Checkbox("Depth-tested", &dd->settings().show_depth_tested);
            ImGui::SameLine();
            ImGui::Checkbox("Overlay", &dd->settings().show_overlay);

            bool physics_layer = (dd->settings().layer_mask & static_cast<uint32_t>(DebugDrawLayer::Physics)) != 0u;
            if (ImGui::Checkbox("Layer: Physics", &physics_layer))
            {
                if (physics_layer)
                {
                    dd->settings().layer_mask |= static_cast<uint32_t>(DebugDrawLayer::Physics);
                }
                else
                {
                    dd->settings().layer_mask &= ~static_cast<uint32_t>(DebugDrawLayer::Physics);
                }
            }

            ImGui::SliderInt("Segments", &dd->settings().segments, 8, 128);
        }
    }
} // namespace

// Window visibility states for menu-bar toggles
namespace
{
    static bool g_show_debug_window = false;
} // namespace

void vk_engine_draw_debug_ui(VulkanEngine *eng)
{
    if (!eng) return;

    ImGuizmo::BeginFrame();

    // Main menu bar at the top
    if (ImGui::BeginMainMenuBar())
    {
        if (ImGui::BeginMenu("View"))
        {
            ImGui::MenuItem("Engine Debug", nullptr, &g_show_debug_window);
            ImGui::EndMenu();
        }

        // Quick stats in menu bar
        ImGui::Separator();
        ImGui::Text("%.1f ms | %d tris | %d draws",
                    eng->stats.frametime,
                    eng->stats.triangle_count,
                    eng->stats.drawcall_count);

        ImGui::EndMainMenuBar();
    }

    // Single consolidated debug window with tabs
    if (g_show_debug_window)
    {
        ImGui::SetNextWindowSize(ImVec2(800, 600), ImGuiCond_FirstUseEver);
        if (ImGui::Begin("Engine Debug", &g_show_debug_window))
        {
            if (ImGui::BeginTabBar("DebugTabs", ImGuiTabBarFlags_None))
            {
                if (ImGui::BeginTabItem("Overview"))
                {
                    ui_overview(eng);
                    ImGui::EndTabItem();
                }

                if (ImGui::BeginTabItem("Scene Editor"))
                {
                    ui_scene_editor(eng);
                    ImGui::EndTabItem();
                }

                if (ImGui::BeginTabItem("Lights"))
                {
                    ui_lights(eng);
                    ImGui::EndTabItem();
                }

                if (ImGui::BeginTabItem("Picking & Gizmo"))
                {
                    ui_picking_gizmo(eng);
                    ImGui::EndTabItem();
                }

                if (ImGui::BeginTabItem("Camera"))
                {
                    ui_camera(eng);
                    ImGui::EndTabItem();
                }

                if (ImGui::BeginTabItem("Planets"))
                {
                    ui_planets(eng);
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

                if (ImGui::BeginTabItem("Shadows"))
                {
                    ui_shadows(eng);
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

                if (ImGui::BeginTabItem("Debug Draw"))
                {
                    ui_debug_draw(eng);
                    ImGui::EndTabItem();
                }

                if (ImGui::BeginTabItem("Window"))
                {
                    ui_window(eng);
                    ImGui::EndTabItem();
                }

                if (ImGui::BeginTabItem("Textures"))
                {
                    ui_textures(eng);
                    ImGui::EndTabItem();
                }

                if (ImGui::BeginTabItem("Async Assets"))
                {
                    ui_async_assets(eng);
                    ImGui::EndTabItem();
                }

                ImGui::EndTabBar();
            }
        }
        ImGui::End();
    }
}
