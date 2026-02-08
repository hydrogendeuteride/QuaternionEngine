# UI

> ImGui integration layer for Vulkan dynamic rendering with DPI-aware font scaling.

## Purpose

Provides a self-contained wrapper around Dear ImGui that handles Vulkan backend
initialization (dynamic rendering, descriptor pools), SDL2 input forwarding,
DPI-aware font rebuilding, and a callback-based draw registration system — so the
rest of the engine can submit UI widgets without touching any ImGui internals.

## Directory Layout

```
ui/
├── imgui_system.h    — ImGuiSystem class declaration
└── imgui_system.cpp  — implementation
```

## Key Types

| Type | Role |
|------|------|
| `ImGuiSystem` | Central entry point — init/cleanup, frame lifecycle, draw callbacks, DPI handling |
| `ImGuiSystem::DrawCallback` | `std::function<void()>` alias for registered UI draw functions |

## Lifecycle

```
init(EngineContext*)
  ├─ creates ImGui context & dark style
  ├─ computes DPI scale from swapchain / window ratio
  ├─ rebuilds fonts at native DPI
  ├─ creates dedicated descriptor pool (1000 sets per type)
  ├─ initializes ImGui SDL2 + Vulkan backends (dynamic rendering)
  └─ uploads font texture

begin_frame()
  ├─ ImGui_ImplVulkan_NewFrame / ImGui_ImplSDL2_NewFrame
  ├─ updates framebuffer scale & detects DPI changes (rebuilds fonts if needed)
  ├─ ImGui::NewFrame()
  └─ invokes all registered draw callbacks

end_frame()
  └─ ImGui::Render()

on_swapchain_recreated()
  └─ updates min image count & framebuffer scale

cleanup()
  ├─ shuts down Vulkan & SDL2 ImGui backends
  ├─ destroys descriptor pool
  └─ destroys ImGui context
```

## Usage

### Registering draw callbacks

```cpp
engine->ui()->add_draw_callback([]() {
    ImGui::Begin("Debug");
    ImGui::Text("Hello from UI callback");
    ImGui::End();
});
```

### Input forwarding

```cpp
// In event loop
engine->ui()->process_event(sdl_event);

// Guard game input when ImGui wants focus
if (!engine->ui()->want_capture_mouse()) {
    // handle game mouse input
}
if (!engine->ui()->want_capture_keyboard()) {
    // handle game keyboard input
}
```

## Integration

`ImGuiSystem` is owned by `VulkanEngine` as `std::unique_ptr<ImGuiSystem>` and
accessed via `VulkanEngine::ui()`. It is **not** exposed through `EngineContext`.

During engine initialization, the built-in debug UI callback is registered:
```cpp
_ui->add_draw_callback([this]() { vk_engine_draw_debug_ui(this); });
```

The ImGui render pass records `ImGui_ImplVulkan_RenderDrawData` into the
render graph after all other passes (tonemap, FXAA, transparent, debug draw).

## Related Docs

- [docs/ImGuiSystem.md](../../../docs/ImGuiSystem.md) — detailed ImGui system documentation
