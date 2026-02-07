# Input

> Platform-agnostic input system providing keyboard, mouse, and window event handling via SDL2 backend.

## Purpose

Provides a unified input abstraction that polls platform events (SDL2), translates them
into engine-native types (`Key`, `MouseButton`, `InputEvent`), and maintains per-frame
state for both polling-based and event-based input consumption. Also tracks window
lifecycle (quit, minimize, resize) and supports cursor mode switching.

## Directory Layout

```
input/
├── input_system.h    — all enums, structs, InputState, and InputSystem classes
└── input_system.cpp  — SDL2 event polling and state management implementation
```

## Key Types

| Type | Role |
|------|------|
| `InputSystem` | Central entry point — event pumping, window state, cursor mode, native event forwarding |
| `InputState` | Per-frame snapshot of keyboard/mouse state (down/pressed/released queries) |
| `InputEvent` | Timestamped event record with type, key/button, position, delta, and modifiers |
| `Key` | Cross-platform key codes based on USB HID usage IDs (A–Z, 0–9, modifiers, etc.) |
| `MouseButton` | Mouse button enum (Left, Middle, Right, X1, X2) |
| `CursorMode` | Cursor visibility/capture mode (Normal, Hidden, Relative) |
| `InputModifiers` | Modifier key state (shift, ctrl, alt, super) |

## Lifecycle

```
InputSystem()
  └─ creates SDL2 backend (Impl)

begin_frame()
  └─ resets per-frame deltas (pressed/released arrays, mouse delta, wheel delta)

pump_events()
  └─ SDL_PollEvent loop
       ├─ tracks window state (quit, minimize, resize)
       ├─ translates SDL key/mouse events → InputEvent records
       └─ updates InputState (down/pressed/released arrays, mouse pos/delta)

state()     ← polling-based: query key_down(), mouse_pressed(), mouse_delta(), etc.
events()    ← event-based: iterate timestamped InputEvent span

set_cursor_mode(mode)
  └─ switches between Normal / Hidden / Relative (SDL relative mouse)

for_each_native_event(callback, user)
  └─ forwards raw SDL_Event pointers to consumers (e.g. ImGui backend)

~InputSystem()
  └─ destroys Impl (RAII)
```

## Usage

### Polling-based input

```cpp
const auto& input = ctx->input->state();

if (input.key_pressed(Key::Escape))
    request_quit();

if (input.mouse_down(MouseButton::Right))
{
    glm::vec2 delta = input.mouse_delta();
    camera.rotate(delta.x, delta.y);
}

float scroll = input.wheel_delta().y;
camera.zoom(scroll);
```

### Event-based input

```cpp
for (const auto& ev : ctx->input->events())
{
    if (ev.type == InputEvent::Type::KeyDown && ev.key == Key::F)
        toggle_flashlight();
}
```

### Cursor mode

```cpp
ctx->input->set_cursor_mode(CursorMode::Relative);  // FPS-style capture
ctx->input->set_cursor_mode(CursorMode::Normal);     // UI interaction
```

## Integration

`InputSystem` is owned by `VulkanEngine` and exposed via `EngineContext::input`.
Each frame the engine calls `begin_frame()` then `pump_events()` before any
gameplay or rendering logic. Window state flags (`quit_requested`, `resize_requested`,
`window_minimized`) are consumed by the engine's main loop. Native SDL events are
forwarded to the ImGui backend via `for_each_native_event()`.

## Related Docs

- [docs/InputSystem.md](../../../docs/InputSystem.md) — detailed input system documentation
