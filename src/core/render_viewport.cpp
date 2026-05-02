#include "core/render_viewport.h"

#include "core/config.h"
#include "core/device/images.h"
#include "core/engine.h"

#include "SDL2/SDL.h"
#include "SDL2/SDL_vulkan.h"

#include <cmath>

namespace render
{
    bool query_render_viewport_metrics(const VulkanEngine &engine, RenderViewportMetrics &out_metrics)
    {
        out_metrics = {};
        if (!engine._window || !engine._swapchainManager)
        {
            return false;
        }

        SDL_GetWindowSize(engine._window, &out_metrics.window_width_px, &out_metrics.window_height_px);
        SDL_Vulkan_GetDrawableSize(engine._window,
                                   &out_metrics.drawable_width_px,
                                   &out_metrics.drawable_height_px);

        if (out_metrics.drawable_width_px > 0 && out_metrics.drawable_height_px > 0)
        {
            out_metrics.drawable_extent.width = static_cast<uint32_t>(out_metrics.drawable_width_px);
            out_metrics.drawable_extent.height = static_cast<uint32_t>(out_metrics.drawable_height_px);
        }
        else
        {
            out_metrics.drawable_extent = engine._swapchainManager->windowExtent();
            out_metrics.drawable_width_px = static_cast<int>(out_metrics.drawable_extent.width);
            out_metrics.drawable_height_px = static_cast<int>(out_metrics.drawable_extent.height);
        }

        out_metrics.swapchain_extent = engine._swapchainManager->swapchainExtent();
        if (out_metrics.drawable_extent.width == 0 || out_metrics.drawable_extent.height == 0 ||
            out_metrics.swapchain_extent.width == 0 || out_metrics.swapchain_extent.height == 0)
        {
            return false;
        }

        out_metrics.logical_extent = engine._logicalRenderExtent;
        if (out_metrics.logical_extent.width == 0 || out_metrics.logical_extent.height == 0)
        {
            out_metrics.logical_extent = VkExtent2D{kRenderWidth, kRenderHeight};
        }
        if (out_metrics.logical_extent.width == 0 || out_metrics.logical_extent.height == 0)
        {
            return false;
        }

        out_metrics.letterbox_rect =
                vkutil::compute_letterbox_rect(out_metrics.logical_extent, out_metrics.swapchain_extent);
        if (out_metrics.letterbox_rect.extent.width == 0 || out_metrics.letterbox_rect.extent.height == 0)
        {
            return false;
        }

        out_metrics.draw_from_swap_x =
                static_cast<double>(out_metrics.drawable_extent.width) /
                static_cast<double>(out_metrics.swapchain_extent.width);
        out_metrics.draw_from_swap_y =
                static_cast<double>(out_metrics.drawable_extent.height) /
                static_cast<double>(out_metrics.swapchain_extent.height);

        if (out_metrics.window_width_px > 0 && out_metrics.window_height_px > 0)
        {
            out_metrics.window_from_draw_x =
                    static_cast<double>(out_metrics.window_width_px) /
                    static_cast<double>(out_metrics.drawable_extent.width);
            out_metrics.window_from_draw_y =
                    static_cast<double>(out_metrics.window_height_px) /
                    static_cast<double>(out_metrics.drawable_extent.height);
        }

        return std::isfinite(out_metrics.draw_from_swap_x) &&
               std::isfinite(out_metrics.draw_from_swap_y) &&
               std::isfinite(out_metrics.window_from_draw_x) &&
               std::isfinite(out_metrics.window_from_draw_y);
    }

    bool window_to_swapchain_pixels(const VulkanEngine &engine,
                                    const glm::vec2 &window_pos,
                                    glm::vec2 &out_swapchain_pos,
                                    RenderViewportMetrics *out_metrics)
    {
        RenderViewportMetrics metrics{};
        if (!query_render_viewport_metrics(engine, metrics))
        {
            return false;
        }

        glm::vec2 scale{1.0f, 1.0f};
        if (metrics.window_width_px > 0 && metrics.window_height_px > 0 &&
            metrics.drawable_width_px > 0 && metrics.drawable_height_px > 0)
        {
            scale.x = static_cast<float>(metrics.drawable_width_px) /
                      static_cast<float>(metrics.window_width_px);
            scale.y = static_cast<float>(metrics.drawable_height_px) /
                      static_cast<float>(metrics.window_height_px);
        }

        const glm::vec2 drawable_pos{window_pos.x * scale.x, window_pos.y * scale.y};
        const float sx =
                static_cast<float>(metrics.swapchain_extent.width) /
                static_cast<float>(metrics.drawable_extent.width);
        const float sy =
                static_cast<float>(metrics.swapchain_extent.height) /
                static_cast<float>(metrics.drawable_extent.height);
        out_swapchain_pos = glm::vec2{drawable_pos.x * sx, drawable_pos.y * sy};

        if (out_metrics)
        {
            *out_metrics = metrics;
        }
        return true;
    }

    bool window_to_logical_render_pixels(const VulkanEngine &engine,
                                         const glm::vec2 &window_pos,
                                         glm::vec2 &out_logical_pos,
                                         RenderViewportMetrics *out_metrics)
    {
        RenderViewportMetrics metrics{};
        glm::vec2 swapchain_pos{};
        if (!window_to_swapchain_pixels(engine, window_pos, swapchain_pos, &metrics))
        {
            return false;
        }

        if (!vkutil::map_window_to_letterbox_src(swapchain_pos,
                                                 metrics.logical_extent,
                                                 metrics.swapchain_extent,
                                                 out_logical_pos))
        {
            return false;
        }

        if (out_metrics)
        {
            *out_metrics = metrics;
        }
        return true;
    }

    float query_window_content_scale_x(const VulkanEngine &engine, const float fallback_scale)
    {
        RenderViewportMetrics metrics{};
        if (!query_render_viewport_metrics(engine, metrics) ||
            metrics.window_width_px <= 0 ||
            metrics.drawable_width_px <= 0)
        {
            return fallback_scale;
        }

        const float scale =
                static_cast<float>(metrics.drawable_width_px) /
                static_cast<float>(metrics.window_width_px);
        return std::isfinite(scale) && scale > 0.0f ? scale : fallback_scale;
    }

    float query_window_display_dpi(const VulkanEngine &engine, const float fallback_dpi)
    {
        if (!engine._window)
        {
            return fallback_dpi;
        }

        const int display_idx = SDL_GetWindowDisplayIndex(engine._window);
        if (display_idx < 0)
        {
            return fallback_dpi;
        }

        float reported_dpi = 0.0f;
        if (SDL_GetDisplayDPI(display_idx, nullptr, &reported_dpi, nullptr) == 0 && reported_dpi > 0.0f)
        {
            return reported_dpi;
        }
        return fallback_dpi;
    }
} // namespace render
