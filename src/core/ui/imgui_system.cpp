#include "imgui_system.h"

#include "core/context.h"
#include "core/device/swapchain.h"

#include "SDL2/SDL.h"
#include "SDL2/SDL_vulkan.h"

#include "imgui.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_vulkan.h"

#include <algorithm>
#include <cmath>

#include "device.h"

namespace
{
    VkDescriptorPool create_imgui_descriptor_pool(VkDevice device)
    {
        VkDescriptorPoolSize pool_sizes[] = {
            {VK_DESCRIPTOR_TYPE_SAMPLER, 1000},
            {VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1000},
            {VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, 1000},
            {VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1000},
            {VK_DESCRIPTOR_TYPE_UNIFORM_TEXEL_BUFFER, 1000},
            {VK_DESCRIPTOR_TYPE_STORAGE_TEXEL_BUFFER, 1000},
            {VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1000},
            {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1000},
            {VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER_DYNAMIC, 1000},
            {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER_DYNAMIC, 1000},
            {VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT, 1000}
        };

        VkDescriptorPoolCreateInfo pool_info{};
        pool_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
        pool_info.flags = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;
        pool_info.maxSets = 1000;
        pool_info.poolSizeCount = static_cast<uint32_t>(std::size(pool_sizes));
        pool_info.pPoolSizes = pool_sizes;

        VkDescriptorPool pool = VK_NULL_HANDLE;
        VK_CHECK(vkCreateDescriptorPool(device, &pool_info, nullptr, &pool));
        return pool;
    }

    uint32_t clamp_imgui_image_count(uint32_t count)
    {
        if (count < 2) return 2;
        if (count > 8) return 8;
        return count;
    }
} // namespace

void ImGuiSystem::init(EngineContext *context)
{
    if (_initialized)
    {
        return;
    }

    _context = context;
    if (_context == nullptr || _context->getDevice() == nullptr || _context->getSwapchain() == nullptr || _context->window == nullptr)
    {
        fmt::println("[ImGuiSystem] init skipped (missing context/device/swapchain/window)");
        return;
    }

    VkDevice device = _context->getDevice()->device();

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();

    ImGuiIO &io = ImGui::GetIO();

    _swapchain_format = _context->getSwapchain()->swapchainImageFormat();

    _dpi_scale = std::clamp(compute_dpi_scale(), 0.5f, 4.0f);
    rebuild_fonts(_dpi_scale);

    _imgui_pool = create_imgui_descriptor_pool(device);

    ImGui_ImplSDL2_InitForVulkan(_context->window);

    ImGui_ImplVulkan_InitInfo init_info{};
    init_info.Instance = _context->getDevice()->instance();
    init_info.PhysicalDevice = _context->getDevice()->physicalDevice();
    init_info.Device = _context->getDevice()->device();
    init_info.QueueFamily = _context->getDevice()->graphicsQueueFamily();
    init_info.Queue = _context->getDevice()->graphicsQueue();
    init_info.DescriptorPool = _imgui_pool;

    const auto &images = _context->getSwapchain()->swapchainImages();
    uint32_t image_count = images.empty() ? 3u : clamp_imgui_image_count(static_cast<uint32_t>(images.size()));
    init_info.MinImageCount = image_count;
    init_info.ImageCount = image_count;

    init_info.UseDynamicRendering = true;
    init_info.PipelineRenderingCreateInfo = {};
    init_info.PipelineRenderingCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO;
    init_info.PipelineRenderingCreateInfo.colorAttachmentCount = 1;
    init_info.PipelineRenderingCreateInfo.pColorAttachmentFormats = &_swapchain_format;
    init_info.MSAASamples = VK_SAMPLE_COUNT_1_BIT;

    ImGui_ImplVulkan_Init(&init_info);

    if (!ImGui_ImplVulkan_CreateFontsTexture())
    {
        fmt::println("[ImGuiSystem] Warning: ImGui_ImplVulkan_CreateFontsTexture() failed");
    }

    io.FontGlobalScale = (_dpi_scale > 0.0f) ? (1.0f / _dpi_scale) : 1.0f;

    _initialized = true;
}

void ImGuiSystem::cleanup()
{
    if (!_initialized)
    {
        return;
    }

    ImGui_ImplVulkan_Shutdown();
    ImGui_ImplSDL2_Shutdown();

    if (_context && _context->getDevice() && _imgui_pool != VK_NULL_HANDLE)
    {
        vkDestroyDescriptorPool(_context->getDevice()->device(), _imgui_pool, nullptr);
    }
    _imgui_pool = VK_NULL_HANDLE;

    ImGui::DestroyContext();

    _draw_callbacks.clear();
    _context = nullptr;
    _initialized = false;
}

void ImGuiSystem::process_event(const SDL_Event &event)
{
    if (!_initialized)
    {
        return;
    }
    ImGui_ImplSDL2_ProcessEvent(const_cast<SDL_Event *>(&event));
}

void ImGuiSystem::begin_frame()
{
    if (!_initialized)
    {
        return;
    }

    ImGui_ImplVulkan_NewFrame();
    ImGui_ImplSDL2_NewFrame();

    update_framebuffer_scale();

    const float new_scale = std::clamp(compute_dpi_scale(), 0.5f, 4.0f);
    if (std::isfinite(new_scale) && std::abs(new_scale - _dpi_scale) > 0.05f)
    {
        rebuild_fonts(new_scale);
        _dpi_scale = new_scale;
    }

    ImGui::NewFrame();

    for (auto &cb : _draw_callbacks)
    {
        if (cb) cb();
    }
}

void ImGuiSystem::end_frame()
{
    if (!_initialized)
    {
        return;
    }
    ImGui::Render();
}

void ImGuiSystem::add_draw_callback(DrawCallback callback)
{
    _draw_callbacks.push_back(std::move(callback));
}

void ImGuiSystem::clear_draw_callbacks()
{
    _draw_callbacks.clear();
}

bool ImGuiSystem::want_capture_mouse() const
{
    if (!_initialized) return false;
    return ImGui::GetIO().WantCaptureMouse;
}

bool ImGuiSystem::want_capture_keyboard() const
{
    if (!_initialized) return false;
    return ImGui::GetIO().WantCaptureKeyboard;
}

void ImGuiSystem::on_swapchain_recreated()
{
    if (!_initialized || _context == nullptr || _context->getSwapchain() == nullptr)
    {
        return;
    }

    const auto &images = _context->getSwapchain()->swapchainImages();
    uint32_t image_count = images.empty() ? 3u : clamp_imgui_image_count(static_cast<uint32_t>(images.size()));
    ImGui_ImplVulkan_SetMinImageCount(image_count);

    update_framebuffer_scale();
}

float ImGuiSystem::compute_dpi_scale() const
{
    if (_context == nullptr || _context->window == nullptr || _context->getSwapchain() == nullptr)
    {
        return 1.0f;
    }

    int win_w = 0, win_h = 0;
    SDL_GetWindowSize(_context->window, &win_w, &win_h);
    if (win_w <= 0 || win_h <= 0)
    {
        return _dpi_scale > 0.0f ? _dpi_scale : 1.0f;
    }

    VkExtent2D swap = _context->getSwapchain()->swapchainExtent();
    if (swap.width == 0 || swap.height == 0)
    {
        return _dpi_scale > 0.0f ? _dpi_scale : 1.0f;
    }

    float sx = static_cast<float>(swap.width) / static_cast<float>(win_w);
    float sy = static_cast<float>(swap.height) / static_cast<float>(win_h);

    if (!std::isfinite(sx) || !std::isfinite(sy))
    {
        return 1.0f;
    }

    return 0.5f * (sx + sy);
}

void ImGuiSystem::update_framebuffer_scale()
{
    if (_context == nullptr || _context->window == nullptr || _context->getSwapchain() == nullptr)
    {
        return;
    }

    int win_w = 0, win_h = 0;
    SDL_GetWindowSize(_context->window, &win_w, &win_h);
    if (win_w <= 0 || win_h <= 0)
    {
        return;
    }

    VkExtent2D swap = _context->getSwapchain()->swapchainExtent();
    if (swap.width == 0 || swap.height == 0)
    {
        return;
    }

    ImGuiIO &io = ImGui::GetIO();
    io.DisplayFramebufferScale = ImVec2(static_cast<float>(swap.width) / static_cast<float>(win_w),
                                        static_cast<float>(swap.height) / static_cast<float>(win_h));

    const float scale = std::clamp(compute_dpi_scale(), 0.5f, 4.0f);
    io.FontGlobalScale = (scale > 0.0f) ? (1.0f / scale) : 1.0f;
}

void ImGuiSystem::rebuild_fonts(float dpi_scale)
{
    if (_initialized)
    {
        ImGui_ImplVulkan_DestroyFontsTexture();
    }

    ImGuiIO &io = ImGui::GetIO();
    io.Fonts->Clear();

    ImFontConfig cfg{};
    cfg.SizePixels = _base_font_size * dpi_scale;
    cfg.OversampleH = 3;
    cfg.OversampleV = 2;
    cfg.PixelSnapH = false;

    io.Fonts->AddFontDefault(&cfg);

    io.FontGlobalScale = (dpi_scale > 0.0f) ? (1.0f / dpi_scale) : 1.0f;

    if (_initialized)
    {
        if (!ImGui_ImplVulkan_CreateFontsTexture())
        {
            fmt::println("[ImGuiSystem] Warning: ImGui_ImplVulkan_CreateFontsTexture() failed after DPI change");
        }
    }
}
