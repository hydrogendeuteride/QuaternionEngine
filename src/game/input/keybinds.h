#pragma once

#include "core/input/input_system.h"

#include <optional>
#include <string>

namespace Game
{
    // Per-subsystem keybind structs. Each field is a Key; Key::Unknown means "unbound".
    //
    // To add a new category:
    //   1. Declare a struct here with its default key assignments.
    //   2. Add a member to `Keybinds` below.
    //   3. In keybinds.cpp, add a field table and register it in the `kSections` list.
    //   4. (Optional) add defaults to assets/settings/keybinds.toml.

    struct ShipKeybinds
    {
        // Translation (local frame)
        Key thrust_forward{Key::W};
        Key thrust_back{Key::S};
        Key thrust_left{Key::A};
        Key thrust_right{Key::D};
        Key thrust_up{Key::Space};
        // Disabled by default: LeftCtrl collides with maneuver gizmo 10x modifier.
        Key thrust_down{Key::Unknown};

        // Rotation (local frame)
        Key pitch_up{Key::ArrowUp};
        Key pitch_down{Key::ArrowDown};
        Key yaw_left{Key::ArrowLeft};
        Key yaw_right{Key::ArrowRight};
        Key roll_left{Key::Q};
        Key roll_right{Key::E};

        // Flight assist
        Key sas_toggle{Key::T};
    };

    struct Keybinds
    {
        static constexpr int kSchemaVersion = 1;
        ShipKeybinds ship{};
    };

    std::optional<Keybinds> load_keybinds(const std::string &toml_path);
    bool save_keybinds(const std::string &toml_path, const Keybinds &kb);
} // namespace Game
