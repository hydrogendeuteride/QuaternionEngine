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

    void processEvent(const SDL_Event &event);

    void beginFrame();
    void endFrame();

    void addDrawCallback(DrawCallback callback);
    void clearDrawCallbacks();

    bool wantCaptureMouse() const;
    bool wantCaptureKeyboard() const;

    void onSwapchainRecreated();

private:
    float computeDpiScale() const;
    void updateFramebufferScale();
    void rebuildFonts(float dpi_scale);

    EngineContext *_context = nullptr;
    std::vector<DrawCallback> _draw_callbacks;

    VkDescriptorPool _imgui_pool = VK_NULL_HANDLE;
    VkFormat _swapchain_format = VK_FORMAT_UNDEFINED;

    float _dpi_scale = 1.0f;
    float _base_font_size = 16.0f;
    bool _initialized = false;
};
