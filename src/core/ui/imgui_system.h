#pragma once

#include <core/types.h>

#include <functional>
#include <vector>

union SDL_Event;
class EngineContext;

class ImGuiSystem
{
public:
    using DrawCallback = std::function<void()>;

    void init(EngineContext *context);
    void cleanup();

    void process_event(const SDL_Event &event);

    void begin_frame();
    void end_frame();

    void add_draw_callback(DrawCallback callback);
    void clear_draw_callbacks();

    bool want_capture_mouse() const;
    bool want_capture_keyboard() const;

    void on_swapchain_recreated();

private:
    float compute_dpi_scale() const;
    void update_framebuffer_scale();
    void rebuild_fonts(float dpi_scale);

    EngineContext *_context = nullptr;
    std::vector<DrawCallback> _draw_callbacks;

    VkDescriptorPool _imgui_pool = VK_NULL_HANDLE;
    VkFormat _swapchain_format = VK_FORMAT_UNDEFINED;

    float _dpi_scale = 1.0f;
    float _base_font_size = 16.0f;
    bool _initialized = false;
};
