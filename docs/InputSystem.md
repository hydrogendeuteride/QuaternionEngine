## Input System: Cross-Platform Input Handling

Unified input abstraction layer that wraps SDL2 events into a clean, game-friendly API. Provides both polled state queries and event-based access.

### Components

- `InputSystem` (src/core/input/input_system.h/.cpp)
  - Main entry point for all input handling.
  - Pumps SDL2 events, tracks window state (quit, minimize, resize).
  - Maintains per-frame `InputState` and `InputEvent` list.

- `InputState`
  - Polled snapshot of keyboard and mouse state.
  - Distinguishes between "down" (held), "pressed" (just pressed this frame), and "released" (just released this frame).

- `InputEvent`
  - Discrete input event with timestamp and modifiers.
  - Types: `KeyDown`, `KeyUp`, `MouseButtonDown`, `MouseButtonUp`, `MouseMove`, `MouseWheel`.

### Key Codes

Cross-platform key codes based on USB HID usage IDs (compatible with SDL scancodes):

```cpp
enum class Key : uint16_t
{
    Unknown = 0,

    // Letters (A-Z): 4-29
    A = 4, B = 5, C = 6, D = 7, E = 8, F = 9, G = 10, H = 11, I = 12,
    J = 13, K = 14, L = 15, M = 16, N = 17, O = 18, P = 19, Q = 20,
    R = 21, S = 22, T = 23, U = 24, V = 25, W = 26, X = 27, Y = 28, Z = 29,

    // Numbers (0-9): 30-39
    Num1 = 30, Num2 = 31, Num3 = 32, Num4 = 33, Num5 = 34,
    Num6 = 35, Num7 = 36, Num8 = 37, Num9 = 38, Num0 = 39,

    // Special keys
    Enter = 40,
    Escape = 41,
    Backspace = 42,
    Tab = 43,
    Space = 44,

    // Modifiers
    LeftCtrl = 224,
    LeftShift = 225,
    LeftAlt = 226,
    LeftSuper = 227,
    RightCtrl = 228,
    RightShift = 229,
    RightAlt = 230,
    RightSuper = 231,
};
```

### Mouse Buttons

```cpp
enum class MouseButton : uint8_t
{
    Left = 0,
    Middle = 1,
    Right = 2,
    X1 = 3,  // Extra button 1
    X2 = 4,  // Extra button 2
};
```

### Cursor Modes

```cpp
enum class CursorMode : uint8_t
{
    Normal = 0,   // Default cursor, visible
    Hidden = 1,   // Cursor hidden but not captured
    Relative = 2, // FPS-style: cursor hidden, motion is relative delta only
};
```

### InputState API

Polled state for keyboard and mouse. Updated each frame before game logic runs.

**Keyboard:**

```cpp
bool key_down(Key key) const;     // True if key is currently held
bool key_pressed(Key key) const;  // True only on the frame key was pressed
bool key_released(Key key) const; // True only on the frame key was released
```

**Mouse:**

```cpp
bool mouse_down(MouseButton button) const;     // True if button is held
bool mouse_pressed(MouseButton button) const;  // True only on frame pressed
bool mouse_released(MouseButton button) const; // True only on frame released

glm::vec2 mouse_position() const; // Current cursor position (pixels)
glm::vec2 mouse_delta() const;    // Motion delta this frame (pixels)
glm::vec2 wheel_delta() const;    // Scroll wheel delta this frame
```

**Modifiers:**

```cpp
InputModifiers modifiers() const;

struct InputModifiers
{
    bool shift = false;
    bool ctrl = false;
    bool alt = false;
    bool super = false; // Windows key / Command key
};
```

### InputEvent Structure

For event-driven input handling:

```cpp
struct InputEvent
{
    enum class Type : uint8_t
    {
        KeyDown,
        KeyUp,
        MouseButtonDown,
        MouseButtonUp,
        MouseMove,
        MouseWheel,
    };

    Type type;
    uint32_t timestamp_ms;    // SDL timestamp
    InputModifiers mods;      // Active modifiers

    Key key;                  // Valid for KeyDown/KeyUp
    MouseButton mouse_button; // Valid for MouseButtonDown/Up

    glm::vec2 mouse_pos;      // Valid for mouse events
    glm::vec2 mouse_delta;    // Valid for MouseMove
    glm::vec2 wheel_delta;    // Valid for MouseWheel
};
```

### InputSystem API

**Frame Lifecycle:**

```cpp
void begin_frame();   // Clear per-frame state (pressed/released, deltas)
void pump_events();   // Poll and process all pending SDL events
```

**State Access:**

```cpp
const InputState& state() const;              // Get current polled state
std::span<const InputEvent> events() const;   // Get all events this frame
```

**Window State:**

```cpp
bool quit_requested() const;     // True if window close requested
bool window_minimized() const;   // True if window is minimized

bool resize_requested() const;   // True if resize/move occurred
uint32_t last_resize_event_ms() const;
void clear_resize_request();     // Clear after handling resize
```

**Cursor Control:**

```cpp
CursorMode cursor_mode() const;
void set_cursor_mode(CursorMode mode);
```

**Native Event Access (Engine Internal):**

```cpp
// For ImGui or other systems that need raw SDL events
void for_each_native_event(NativeEventCallback callback, void* user) const;
```

### Usage Example

**Polled State (Most Common):**

```cpp
void Game::update(InputSystem& input)
{
    const InputState& state = input.state();

    // Movement
    glm::vec3 move{0.0f};
    if (state.key_down(Key::W)) move.z -= 1.0f;
    if (state.key_down(Key::S)) move.z += 1.0f;
    if (state.key_down(Key::A)) move.x -= 1.0f;
    if (state.key_down(Key::D)) move.x += 1.0f;

    // Sprint (shift held)
    float speed = 5.0f;
    if (state.modifiers().shift) speed = 10.0f;

    // Camera look (relative mode)
    if (state.mouse_down(MouseButton::Right))
    {
        input.set_cursor_mode(CursorMode::Relative);
        glm::vec2 delta = state.mouse_delta();
        camera.rotate(delta.x * 0.1f, delta.y * 0.1f);
    }
    else
    {
        input.set_cursor_mode(CursorMode::Normal);
    }

    // Fire on click
    if (state.mouse_pressed(MouseButton::Left))
    {
        player.fire();
    }

    // Toggle inventory on I press
    if (state.key_pressed(Key::I))
    {
        ui.toggle_inventory();
    }
}
```

**Event-Driven (For UI / Text Input):**

```cpp
void TextBox::process_input(InputSystem& input)
{
    for (const InputEvent& ev : input.events())
    {
        if (ev.type == InputEvent::Type::KeyDown)
        {
            if (ev.key == Key::Backspace && !text.empty())
            {
                text.pop_back();
            }
            else if (ev.key == Key::Enter)
            {
                submit();
            }
        }
        else if (ev.type == InputEvent::Type::MouseWheel)
        {
            scroll_offset -= ev.wheel_delta.y * 20.0f;
        }
    }
}
```

### Frame Flow

1. Engine calls `InputSystem::begin_frame()` at frame start.
2. Engine calls `InputSystem::pump_events()` to process SDL events.
3. Game code queries `state()` for polled input or iterates `events()` for event-driven logic.
4. Render loop proceeds.

### Integration with ImGui

The engine provides native event access for ImGui integration:

```cpp
input.for_each_native_event(
    [](void* user, InputSystem::NativeEventView event)
    {
        if (event.backend == InputSystem::NativeBackend::SDL2)
        {
            ImGui_ImplSDL2_ProcessEvent(
                static_cast<const SDL_Event*>(event.data));
        }
    },
    nullptr);
```

### Tips

- Prefer polled state (`key_down`, `mouse_down`) for continuous actions like movement.
- Use pressed/released (`key_pressed`, `mouse_pressed`) for one-shot actions like firing or toggling.
- Use `CursorMode::Relative` for FPS-style camera control; mouse position becomes meaningless, only delta matters.
- Check `quit_requested()` each frame to handle window close gracefully.
- The `resize_requested()` flag includes window move events to trigger swapchain recreation on multi-monitor setups.