#include "input_system.h"

#include "SDL2/SDL.h"

#include <algorithm>
#include <cstring>

struct InputSystem::Impl
{
    std::vector<SDL_Event> sdl_events{};
};

namespace
{
    InputModifiers mods_from_sdl(const SDL_Keymod mod)
    {
        InputModifiers out{};
        out.shift = (mod & KMOD_SHIFT) != 0;
        out.ctrl = (mod & KMOD_CTRL) != 0;
        out.alt = (mod & KMOD_ALT) != 0;
        out.super = (mod & KMOD_GUI) != 0;
        return out;
    }

    bool map_sdl_mouse_button(uint8_t sdl_button, MouseButton &out)
    {
        switch (sdl_button)
        {
            case SDL_BUTTON_LEFT:
                out = MouseButton::Left;
                return true;
            case SDL_BUTTON_MIDDLE:
                out = MouseButton::Middle;
                return true;
            case SDL_BUTTON_RIGHT:
                out = MouseButton::Right;
                return true;
            case SDL_BUTTON_X1:
                out = MouseButton::X1;
                return true;
            case SDL_BUTTON_X2:
                out = MouseButton::X2;
                return true;
            default:
                return false;
        }
    }

    glm::vec2 wheel_from_sdl(const SDL_MouseWheelEvent &wheel)
    {
        glm::vec2 delta(static_cast<float>(wheel.x), static_cast<float>(wheel.y));
        if (wheel.direction == SDL_MOUSEWHEEL_FLIPPED)
        {
            delta = -delta;
        }
        return delta;
    }
} // namespace

void InputState::begin_frame()
{
    std::fill(_keys_pressed.begin(), _keys_pressed.end(), 0);
    std::fill(_keys_released.begin(), _keys_released.end(), 0);

    std::fill(_mouse_pressed.begin(), _mouse_pressed.end(), 0);
    std::fill(_mouse_released.begin(), _mouse_released.end(), 0);

    _mouse_delta = glm::vec2(0.0f);
    _wheel_delta = glm::vec2(0.0f);
}

bool InputState::key_down(Key key) const
{
    const size_t idx = key_index(key);
    if (idx >= _keys_down.size()) return false;
    return _keys_down[idx] != 0;
}

bool InputState::key_pressed(Key key) const
{
    const size_t idx = key_index(key);
    if (idx >= _keys_pressed.size()) return false;
    return _keys_pressed[idx] != 0;
}

bool InputState::key_released(Key key) const
{
    const size_t idx = key_index(key);
    if (idx >= _keys_released.size()) return false;
    return _keys_released[idx] != 0;
}

bool InputState::mouse_down(MouseButton button) const
{
    const size_t idx = mouse_index(button);
    if (idx >= _mouse_down.size()) return false;
    return _mouse_down[idx] != 0;
}

bool InputState::mouse_pressed(MouseButton button) const
{
    const size_t idx = mouse_index(button);
    if (idx >= _mouse_pressed.size()) return false;
    return _mouse_pressed[idx] != 0;
}

bool InputState::mouse_released(MouseButton button) const
{
    const size_t idx = mouse_index(button);
    if (idx >= _mouse_released.size()) return false;
    return _mouse_released[idx] != 0;
}

size_t InputState::key_index(Key key)
{
    return static_cast<size_t>(static_cast<uint16_t>(key));
}

size_t InputState::mouse_index(MouseButton button)
{
    return static_cast<size_t>(static_cast<uint8_t>(button));
}

void InputState::set_key(Key key, bool down, bool repeat)
{
    const size_t idx = key_index(key);
    if (idx >= _keys_down.size()) return;

    const bool was_down = _keys_down[idx] != 0;
    if (down)
    {
        _keys_down[idx] = 1;
        if (!was_down && !repeat)
        {
            _keys_pressed[idx] = 1;
        }
    }
    else
    {
        _keys_down[idx] = 0;
        if (was_down)
        {
            _keys_released[idx] = 1;
        }
    }
}

void InputState::set_mouse_button(MouseButton button, bool down)
{
    const size_t idx = mouse_index(button);
    if (idx >= _mouse_down.size()) return;

    const bool was_down = _mouse_down[idx] != 0;
    if (down)
    {
        _mouse_down[idx] = 1;
        if (!was_down)
        {
            _mouse_pressed[idx] = 1;
        }
    }
    else
    {
        _mouse_down[idx] = 0;
        if (was_down)
        {
            _mouse_released[idx] = 1;
        }
    }
}

void InputState::add_mouse_motion(const glm::vec2 &pos, const glm::vec2 &delta)
{
    _mouse_pos = pos;
    _mouse_delta += delta;
}

void InputState::add_mouse_wheel(const glm::vec2 &delta)
{
    _wheel_delta += delta;
}

void InputState::set_modifiers(const InputModifiers &mods)
{
    _mods = mods;
}

InputSystem::InputSystem()
    : _impl(std::make_unique<Impl>())
{
}

InputSystem::~InputSystem() = default;

void InputSystem::begin_frame()
{
    _state.begin_frame();
    _events.clear();
    if (_impl)
    {
        _impl->sdl_events.clear();
    }
}

void InputSystem::pump_events()
{
    if (!_impl)
    {
        return;
    }

    SDL_Event e{};
    while (SDL_PollEvent(&e) != 0)
    {
        _impl->sdl_events.push_back(e);

        // Track app/window state
        if (e.type == SDL_QUIT)
        {
            _quit_requested = true;
        }
        if (e.type == SDL_WINDOWEVENT)
        {
            switch (e.window.event)
            {
                case SDL_WINDOWEVENT_MINIMIZED:
                    _window_minimized = true;
                    break;
                case SDL_WINDOWEVENT_RESTORED:
                    _window_minimized = false;
                    _resize_requested = true;
                    _last_resize_event_ms = SDL_GetTicks();
                    break;
                case SDL_WINDOWEVENT_SIZE_CHANGED:
                case SDL_WINDOWEVENT_RESIZED:
                    _resize_requested = true;
                    _last_resize_event_ms = SDL_GetTicks();
                    break;
                case SDL_WINDOWEVENT_MOVED:
                    _resize_requested = true;
                    _last_resize_event_ms = SDL_GetTicks();
                    break;
                default:
                    break;
            }
        }

        // Convert to engine input events + state
        if (e.type == SDL_KEYDOWN || e.type == SDL_KEYUP)
        {
            const bool down = (e.type == SDL_KEYDOWN);
            const bool repeat = down && (e.key.repeat != 0);
            const uint16_t code = static_cast<uint16_t>(e.key.keysym.scancode);
            const Key key = static_cast<Key>(code);
            const InputModifiers mods = mods_from_sdl(static_cast<SDL_Keymod>(e.key.keysym.mod));

            _state.set_modifiers(mods);
            _state.set_key(key, down, repeat);

            InputEvent ev{};
            ev.type = down ? InputEvent::Type::KeyDown : InputEvent::Type::KeyUp;
            ev.timestamp_ms = e.key.timestamp;
            ev.mods = mods;
            ev.key = key;
            _events.push_back(ev);
        }
        else if (e.type == SDL_MOUSEBUTTONDOWN || e.type == SDL_MOUSEBUTTONUP)
        {
            MouseButton btn{};
            if (!map_sdl_mouse_button(e.button.button, btn))
            {
                continue;
            }
            const bool down = (e.type == SDL_MOUSEBUTTONDOWN);
            const InputModifiers mods = mods_from_sdl(static_cast<SDL_Keymod>(SDL_GetModState()));

            _state.set_modifiers(mods);
            _state.set_mouse_button(btn, down);
            _state.add_mouse_motion(glm::vec2(static_cast<float>(e.button.x), static_cast<float>(e.button.y)),
                                    glm::vec2(0.0f));

            InputEvent ev{};
            ev.type = down ? InputEvent::Type::MouseButtonDown : InputEvent::Type::MouseButtonUp;
            ev.timestamp_ms = e.button.timestamp;
            ev.mods = mods;
            ev.mouse_button = btn;
            ev.mouse_pos = glm::vec2(static_cast<float>(e.button.x), static_cast<float>(e.button.y));
            _events.push_back(ev);
        }
        else if (e.type == SDL_MOUSEMOTION)
        {
            const InputModifiers mods = mods_from_sdl(static_cast<SDL_Keymod>(SDL_GetModState()));
            _state.set_modifiers(mods);
            _state.add_mouse_motion(glm::vec2(static_cast<float>(e.motion.x), static_cast<float>(e.motion.y)),
                                    glm::vec2(static_cast<float>(e.motion.xrel), static_cast<float>(e.motion.yrel)));

            InputEvent ev{};
            ev.type = InputEvent::Type::MouseMove;
            ev.timestamp_ms = e.motion.timestamp;
            ev.mods = mods;
            ev.mouse_pos = glm::vec2(static_cast<float>(e.motion.x), static_cast<float>(e.motion.y));
            ev.mouse_delta = glm::vec2(static_cast<float>(e.motion.xrel), static_cast<float>(e.motion.yrel));
            _events.push_back(ev);
        }
        else if (e.type == SDL_MOUSEWHEEL)
        {
            const InputModifiers mods = mods_from_sdl(static_cast<SDL_Keymod>(SDL_GetModState()));
            const glm::vec2 delta = wheel_from_sdl(e.wheel);

            _state.set_modifiers(mods);
            _state.add_mouse_wheel(delta);

            InputEvent ev{};
            ev.type = InputEvent::Type::MouseWheel;
            ev.timestamp_ms = e.wheel.timestamp;
            ev.mods = mods;
            ev.wheel_delta = delta;
            _events.push_back(ev);
        }
    }
}

void InputSystem::set_cursor_mode(CursorMode mode)
{
    if (_cursor_mode == mode)
    {
        return;
    }

    switch (mode)
    {
        case CursorMode::Normal:
            SDL_SetRelativeMouseMode(SDL_FALSE);
            SDL_ShowCursor(SDL_ENABLE);
            break;
        case CursorMode::Hidden:
            SDL_SetRelativeMouseMode(SDL_FALSE);
            SDL_ShowCursor(SDL_DISABLE);
            break;
        case CursorMode::Relative:
            SDL_ShowCursor(SDL_DISABLE);
            SDL_SetRelativeMouseMode(SDL_TRUE);
            break;
    }

    _cursor_mode = mode;
}

void InputSystem::for_each_native_event(NativeEventCallback callback, void *user) const
{
    if (!_impl || !callback)
    {
        return;
    }

    for (const SDL_Event &e: _impl->sdl_events)
    {
        NativeEventView view{};
        view.backend = NativeBackend::SDL2;
        view.data = &e;
        callback(user, view);
    }
}

