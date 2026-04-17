#include "core/input/keybinds.h"

#include <cctype>
#include <optional>
#include <string>

namespace
{
    struct KeyNameEntry
    {
        Key key;
        std::string_view name;
    };

    // Ordered aliases used for key_to_string (first name per key is canonical) and key_from_string.
    constexpr KeyNameEntry kKeyNames[] = {
        {Key::A, "A"}, {Key::B, "B"}, {Key::C, "C"}, {Key::D, "D"},
        {Key::E, "E"}, {Key::F, "F"}, {Key::G, "G"}, {Key::H, "H"},
        {Key::I, "I"}, {Key::J, "J"}, {Key::K, "K"}, {Key::L, "L"},
        {Key::M, "M"}, {Key::N, "N"}, {Key::O, "O"}, {Key::P, "P"},
        {Key::Q, "Q"}, {Key::R, "R"}, {Key::S, "S"}, {Key::T, "T"},
        {Key::U, "U"}, {Key::V, "V"}, {Key::W, "W"}, {Key::X, "X"},
        {Key::Y, "Y"}, {Key::Z, "Z"},

        {Key::Num1, "1"}, {Key::Num2, "2"}, {Key::Num3, "3"}, {Key::Num4, "4"},
        {Key::Num5, "5"}, {Key::Num6, "6"}, {Key::Num7, "7"}, {Key::Num8, "8"},
        {Key::Num9, "9"}, {Key::Num0, "0"},

        {Key::Enter, "Enter"}, {Key::Escape, "Escape"}, {Key::Backspace, "Backspace"},
        {Key::Tab, "Tab"}, {Key::Space, "Space"},
        {Key::LeftBracket, "LeftBracket"}, {Key::RightBracket, "RightBracket"},
        {Key::Comma, "Comma"}, {Key::Period, "Period"}, {Key::Slash, "Slash"},

        {Key::ArrowRight, "ArrowRight"}, {Key::ArrowLeft, "ArrowLeft"},
        {Key::ArrowDown, "ArrowDown"}, {Key::ArrowUp, "ArrowUp"},

        {Key::LeftCtrl, "LeftCtrl"}, {Key::LeftShift, "LeftShift"},
        {Key::LeftAlt, "LeftAlt"}, {Key::LeftSuper, "LeftSuper"},
        {Key::RightCtrl, "RightCtrl"}, {Key::RightShift, "RightShift"},
        {Key::RightAlt, "RightAlt"}, {Key::RightSuper, "RightSuper"},

        {Key::Unknown, "None"}, // canonical name for "no binding"

        // Compatibility aliases accepted by the parser.
        {Key::Num1, "Num1"}, {Key::Num2, "Num2"}, {Key::Num3, "Num3"}, {Key::Num4, "Num4"},
        {Key::Num5, "Num5"}, {Key::Num6, "Num6"}, {Key::Num7, "Num7"}, {Key::Num8, "Num8"},
        {Key::Num9, "Num9"}, {Key::Num0, "Num0"},
        {Key::Unknown, "Unknown"},
    };

    std::string to_lower(std::string_view s)
    {
        std::string out;
        out.reserve(s.size());
        for (char c : s)
        {
            out.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
        }
        return out;
    }
} // namespace

std::string_view key_to_string(const Key k)
{
    for (const auto &entry : kKeyNames)
    {
        if (entry.key == k)
        {
            return entry.name;
        }
    }
    return "None";
}

std::optional<Key> key_from_string(std::string_view name)
{
    if (name.empty())
    {
        return std::nullopt;
    }
    const std::string lowered = to_lower(name);
    for (const auto &entry : kKeyNames)
    {
        if (to_lower(entry.name) == lowered)
        {
            return entry.key;
        }
    }
    return std::nullopt;
}
