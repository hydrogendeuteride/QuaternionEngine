## ImGui System: Immediate-Mode UI Integration

Manages Dear ImGui lifecycle, event processing, and rendering within the Vulkan engine. Provides DPI-aware font scaling and callback-based UI composition.

### Components

- `ImGuiSystem` (src/core/ui/imgui_system.h/.cpp)
  - Initializes ImGui context with Vulkan backend.
  - Processes SDL events for ImGui input.
  - Manages draw callback registration.
  - Handles DPI scaling and font rebuilding.

- `ImGuiPass` (src/render/passes/imgui_pass.h/.cpp)
  - RenderGraph pass that records ImGui draw commands.
  - Renders to swapchain image using dynamic rendering.

- `engine_ui.cpp`
  - Built-in debug UI widgets (stats, render graph, texture streaming, etc.).
  - Example of using the draw callback system.

### ImGuiSystem API

**Initialization:**

```cpp
void init(EngineContext *context);
void cleanup();
```

**Frame Lifecycle:**

```cpp
void beginFrame();  // NewFrame + invoke draw callbacks
void endFrame();    // ImGui::Render()
```

**Event Processing:**

```cpp
void processEvent(const SDL_Event &event);
```

**Draw Callbacks:**

```cpp
void addDrawCallback(DrawCallback callback);
void clearDrawCallbacks();

// DrawCallback type
using DrawCallback = std::function<void()>;
```

**Input Capture Queries:**

```cpp
bool wantCaptureMouse() const;
bool wantCaptureKeyboard() const;
```

**Swapchain Events:**

```cpp
void onSwapchainRecreated();  // Update image count after resize
```

### Usage Examples

**Basic Setup (Engine Internal):**

```cpp
// In VulkanEngine::init()
_imgui_system.init(&_context);

// Register debug UI callback
_imgui_system.addDrawCallback([this]() {
    vk_engine_draw_debug_ui(this);
});
```

**Event Processing:**

```cpp
// In event loop
for (const SDL_Event& event : events)
{
    _imgui_system.processEvent(event);
}
```

**Frame Integration:**

```cpp
// Start of frame (after input processing)
_imgui_system.beginFrame();

// ... game update, scene rendering ...

// End of frame (before RenderGraph execution)
_imgui_system.endFrame();
```

**Custom UI Callback:**

```cpp
void Game::init()
{
    engine.imgui_system().addDrawCallback([this]() {
        draw_game_ui();
    });
}

void Game::draw_game_ui()
{
    if (ImGui::Begin("Game Stats"))
    {
        ImGui::Text("Score: %d", _score);
        ImGui::Text("Health: %.0f%%", _health * 100.0f);

        if (ImGui::Button("Pause"))
        {
            toggle_pause();
        }
    }
    ImGui::End();
}
```

**Respecting Input Capture:**

```cpp
void Game::update()
{
    // Don't process game input when ImGui wants it
    if (!engine.imgui_system().wantCaptureMouse())
    {
        handle_mouse_input();
    }

    if (!engine.imgui_system().wantCaptureKeyboard())
    {
        handle_keyboard_input();
    }
}
```

### DPI Scaling

The system automatically handles HiDPI displays:

1. **DPI Detection**: Computed from swapchain extent vs window size ratio.
2. **Font Scaling**: Base font size (16px) scaled by DPI factor.
3. **Global Scale**: `FontGlobalScale` set to 1/DPI for proper sizing.
4. **Dynamic Updates**: Fonts rebuilt when DPI changes (e.g., monitor switch).

DPI scale range: 0.5x to 4.0x (clamped for stability).

### Vulkan Integration

ImGui is initialized with:
- **Dynamic Rendering**: No render pass objects, uses `VK_KHR_dynamic_rendering`.
- **Dedicated Descriptor Pool**: Separate pool with generous limits for ImGui textures.
- **Swapchain Format**: Renders directly to swapchain image format.

```cpp
ImGui_ImplVulkan_InitInfo init_info{};
init_info.Instance = device->instance();
init_info.PhysicalDevice = device->physicalDevice();
init_info.Device = device->device();
init_info.QueueFamily = device->graphicsQueueFamily();
init_info.Queue = device->graphicsQueue();
init_info.DescriptorPool = _imgui_pool;
init_info.MinImageCount = swapchain_image_count;
init_info.ImageCount = swapchain_image_count;
init_info.UseDynamicRendering = true;
init_info.PipelineRenderingCreateInfo.colorAttachmentCount = 1;
init_info.PipelineRenderingCreateInfo.pColorAttachmentFormats = &swapchain_format;
init_info.MSAASamples = VK_SAMPLE_COUNT_1_BIT;
```

### RenderGraph Integration

ImGui rendering is handled by `ImGuiPass`:

```cpp
// In RenderGraph build
_imgui_pass.register_graph(graph, swapchain_image_handle);

// Pass executes after all other rendering
// Renders ImGui draw data to swapchain image
```

The pass:
1. Begins dynamic rendering on swapchain image.
2. Calls `ImGui_ImplVulkan_RenderDrawData()`.
3. Ends rendering.

### Built-in Debug UI

The engine provides comprehensive debug widgets in `engine_ui.cpp`:

**Window Tab:**
- Monitor selection and fullscreen modes.
- HiDPI status and size information.
- GPU information display.

**Stats Tab:**
- Frame time and FPS.
- Draw call and triangle counts.
- Memory usage statistics.

**Scene Tab:**
- GLTF instance spawning.
- Primitive mesh spawning.
- Point light editor.
- Object transform manipulation (ImGuizmo).

**Picking & Gizmo Tab:**
- Last/hover picking details.
- glTF hierarchy navigation (parent/child node selection).
- Object transform manipulation with ImGuizmo.

**Render Graph Tab:**
- Pass list with toggle controls.
- Resource tracking visualization.
- Barrier inspection.

**Texture Streaming Tab:**
- VRAM budget and usage.
- Texture load queue status.
- Cache statistics.

**Shadows Tab:**
- Shadow mode selection.
- Cascade visualization.
- Ray-tracing hybrid controls.

**Post Processing Tab:**
- Tonemapping settings.
- Bloom controls.
- FXAA parameters.
- SSR configuration.

### Draw Callback Order

Callbacks are invoked in registration order during `beginFrame()`:

```cpp
void ImGuiSystem::beginFrame()
{
    ImGui_ImplVulkan_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();

    // Invoke all registered callbacks
    for (auto& cb : _draw_callbacks)
    {
        if (cb) cb();
    }
}
```

Register order-dependent callbacks carefully:
```cpp
// Engine debug UI first
imgui.addDrawCallback([]{ draw_engine_ui(); });

// Game UI on top
imgui.addDrawCallback([]{ draw_game_ui(); });

// Editor overlays last
imgui.addDrawCallback([]{ draw_editor_overlays(); });
```

### ImGuizmo Integration

The engine integrates ImGuizmo for 3D gizmo manipulation:

```cpp
#include "ImGuizmo.h"

void draw_object_gizmo(const glm::mat4& view, const glm::mat4& proj,
                       glm::mat4& object_transform)
{
    ImGuizmo::SetOrthographic(false);
    ImGuizmo::SetDrawlist();

    ImGuiIO& io = ImGui::GetIO();
    ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);

    static ImGuizmo::OPERATION op = ImGuizmo::TRANSLATE;
    static ImGuizmo::MODE mode = ImGuizmo::WORLD;

    ImGuizmo::Manipulate(
        glm::value_ptr(view),
        glm::value_ptr(proj),
        op,
        mode,
        glm::value_ptr(object_transform));
}
```

### Tips

- Always check `wantCaptureMouse()` before processing game mouse input.
- Use `wantCaptureKeyboard()` before processing game keyboard input.
- Register draw callbacks during initialization, not every frame.
- Call `onSwapchainRecreated()` after window resize/mode change.
- The descriptor pool is sized for 1000 sets of each type — sufficient for most debug UIs.
- For production games, consider conditionally compiling out debug UI.
- ImGui windows are persistent between frames — state is preserved automatically.

### Frame Flow

1. **Event Processing**: `processEvent()` for each SDL event.
2. **Begin Frame**: `beginFrame()` starts new ImGui frame and invokes callbacks.
3. **UI Building**: All `ImGui::*` calls happen inside draw callbacks.
4. **End Frame**: `endFrame()` calls `ImGui::Render()` to finalize draw data.
5. **RenderGraph**: `ImGuiPass` executes, recording draw commands to GPU.
6. **Present**: Swapchain presents the final image with ImGui overlay.
