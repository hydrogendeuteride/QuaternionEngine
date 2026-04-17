#pragma once

#include "core/input/input_system.h"

#include <optional>
#include <string_view>

// Canonical string name for a Key used when serializing config. Returns "None" for Unknown.
std::string_view key_to_string(Key k);

// Parse a key name (case-insensitive). Accepts canonical names plus enum aliases like
// "Num1" and "Unknown". Returns std::nullopt if the token is empty or unrecognized.
std::optional<Key> key_from_string(std::string_view name);
