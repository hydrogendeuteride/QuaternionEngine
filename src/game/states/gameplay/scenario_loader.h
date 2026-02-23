#pragma once

#include "gameplay_state.h"
#include <string>
#include <optional>

namespace Game
{
    // Load a ScenarioConfig from a JSON file.
    // Returns std::nullopt on parse/IO failure (errors logged via Logger).
    std::optional<ScenarioConfig> load_scenario_config(const std::string &json_path);

    // Serialize a ScenarioConfig to a JSON string (for saving/debugging).
    std::string serialize_scenario_config(const ScenarioConfig &config);
} // namespace Game
