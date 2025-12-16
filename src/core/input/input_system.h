#pragma once

#include <core/types.h>

#include <array>
#include <cstdint>
#include <memory>
#include <span>
#include <vector>

// Cross-platform input codes loosely based on USB HID usage IDs (and SDL scancodes).
// Keep this list minimal and extend as needed.
enum class Key : uint16_t
{
    Unknown = 0,

    A = 4,
    B = 5,
    C = 6,
    D = 7,
    E = 8,
    F = 9,
    G = 10,
    H = 11,
    I = 12,
    J = 13,
    K = 14,
    L = 15,
    M = 16,
    N = 17,
    O = 18,
    P = 19,
    Q = 20,
    R = 21,
    S = 22,
    T = 23,
    U = 24,
    V = 25,
    W = 26,
    X = 27,
    Y = 28,
    Z = 29,

    Num1 = 30,
    Num2 = 31,
    Num3 = 32,
    Num4 = 33,
    Num5 = 34,
    Num6 = 35,
    Num7 = 36,
    Num8 = 37,
    Num9 = 38,
    Num0 = 39,

    Enter = 40,
    Escape = 41,
    Backspace = 42,
    Tab = 43,
    Space = 44,

    LeftCtrl = 224,
    LeftShift = 225,
    LeftAlt = 226,
    LeftSuper = 227,
    RightCtrl = 228,
    RightShift = 229,
    RightAlt = 230,
    RightSuper = 231,
};

enum class MouseButton : uint8_t
{
    Left = 0,
    Middle = 1,
    Right = 2,
    X1 = 3,
    X2 = 4,
};

enum class CursorMode : uint8_t
{
    Normal = 0,
    Hidden = 1,
    Relative = 2,
};

struct InputModifiers
{
    bool shift = false;
    bool ctrl = false;
    bool alt = false;
    bool super = false;
};

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

    Type type = Type::KeyDown;
    uint32_t timestamp_ms = 0;
    InputModifiers mods{};

    Key key = Key::Unknown;
    MouseButton mouse_button = MouseButton::Left;

    glm::vec2 mouse_pos{0.0f};
    glm::vec2 mouse_delta{0.0f};
    glm::vec2 wheel_delta{0.0f};
};

class InputState
{
public:
    static constexpr uint16_t kMaxKeys = 512;
    static constexpr uint8_t kMouseButtonCount = 5;

    void begin_frame();

    bool key_down(Key key) const;
    bool key_pressed(Key key) const;
    bool key_released(Key key) const;

    bool mouse_down(MouseButton button) const;
    bool mouse_pressed(MouseButton button) const;
    bool mouse_released(MouseButton button) const;

    glm::vec2 mouse_position() const { return _mouse_pos; }
    glm::vec2 mouse_delta() const { return _mouse_delta; }
    glm::vec2 wheel_delta() const { return _wheel_delta; }
    InputModifiers modifiers() const { return _mods; }

private:
    friend class InputSystem;

    static size_t key_index(Key key);
    static size_t mouse_index(MouseButton button);

    void set_key(Key key, bool down, bool repeat);
    void set_mouse_button(MouseButton button, bool down);
    void add_mouse_motion(const glm::vec2 &pos, const glm::vec2 &delta);
    void add_mouse_wheel(const glm::vec2 &delta);
    void set_modifiers(const InputModifiers &mods);

    std::array<uint8_t, kMaxKeys> _keys_down{};
    std::array<uint8_t, kMaxKeys> _keys_pressed{};
    std::array<uint8_t, kMaxKeys> _keys_released{};

    std::array<uint8_t, kMouseButtonCount> _mouse_down{};
    std::array<uint8_t, kMouseButtonCount> _mouse_pressed{};
    std::array<uint8_t, kMouseButtonCount> _mouse_released{};

    glm::vec2 _mouse_pos{0.0f};
    glm::vec2 _mouse_delta{0.0f};
    glm::vec2 _wheel_delta{0.0f};
    InputModifiers _mods{};
};

class InputSystem
{
public:
    enum class NativeBackend : uint8_t
    {
        SDL2 = 0,
    };

    struct NativeEventView
    {
        NativeBackend backend = NativeBackend::SDL2;
        const void *data = nullptr;
    };

    using NativeEventCallback = void (*)(void *user, NativeEventView event);

    InputSystem();
    ~InputSystem();

    InputSystem(const InputSystem &) = delete;
    InputSystem &operator=(const InputSystem &) = delete;

    void begin_frame();
    void pump_events();

    const InputState &state() const { return _state; }
    std::span<const InputEvent> events() const { return _events; }

    bool quit_requested() const { return _quit_requested; }
    bool window_minimized() const { return _window_minimized; }

    bool resize_requested() const { return _resize_requested; }
    uint32_t last_resize_event_ms() const { return _last_resize_event_ms; }
    void clear_resize_request() { _resize_requested = false; }

    CursorMode cursor_mode() const { return _cursor_mode; }
    void set_cursor_mode(CursorMode mode);

    // Engine-internal: dispatch native platform events (SDL events today).
    void for_each_native_event(NativeEventCallback callback, void *user) const;

private:
    struct Impl;
    std::unique_ptr<Impl> _impl;

    InputState _state{};
    std::vector<InputEvent> _events{};

    bool _quit_requested = false;
    bool _window_minimized = false;
    bool _resize_requested = false;
    uint32_t _last_resize_event_ms = 0;
    CursorMode _cursor_mode = CursorMode::Normal;
};

