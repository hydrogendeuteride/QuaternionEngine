#pragma once

#include "core/types.h"

#include <glm/vec2.hpp>

class VulkanEngine;

namespace render
{
    struct RenderViewportMetrics
    {
        int window_width_px{0};
        int window_height_px{0};
        int drawable_width_px{0};
        int drawable_height_px{0};
        VkExtent2D drawable_extent{};
        VkExtent2D swapchain_extent{};
        VkExtent2D logical_extent{};
        VkRect2D letterbox_rect{};
        double draw_from_swap_x{1.0};
        double draw_from_swap_y{1.0};
        double window_from_draw_x{1.0};
        double window_from_draw_y{1.0};
    };

    bool query_render_viewport_metrics(const VulkanEngine &engine, RenderViewportMetrics &out_metrics);
    bool window_to_swapchain_pixels(const VulkanEngine &engine,
                                    const glm::vec2 &window_pos,
                                    glm::vec2 &out_swapchain_pos,
                                    RenderViewportMetrics *out_metrics = nullptr);
    bool window_to_logical_render_pixels(const VulkanEngine &engine,
                                         const glm::vec2 &window_pos,
                                         glm::vec2 &out_logical_pos,
                                         RenderViewportMetrics *out_metrics = nullptr);
    float query_window_content_scale_x(const VulkanEngine &engine, float fallback_scale = 1.0f);
    float query_window_display_dpi(const VulkanEngine &engine, float fallback_dpi);
} // namespace render
