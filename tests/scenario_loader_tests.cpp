#include "game/states/gameplay/scenario_loader.h"

#include <nlohmann/json.hpp>
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>

namespace
{
    using json = nlohmann::json;

    std::filesystem::path make_temp_dir()
    {
        static std::atomic<uint64_t> counter{0};
        const auto tick = std::chrono::high_resolution_clock::now().time_since_epoch().count();
        const uint64_t seq = counter.fetch_add(1, std::memory_order_relaxed);
        std::filesystem::path dir = std::filesystem::temp_directory_path() /
                                    ("vulkan_engine_scenario_loader_" + std::to_string(tick) + "_" + std::to_string(seq));
        std::filesystem::create_directories(dir);
        return dir;
    }

    json make_valid_scenario()
    {
        return json{
                {"schema_version", 1},
                {"speed_scale", 1.0},
                {"mu_base", 3.986004418e14},
                {"system_center", {{"x", 0.0}, {"y", 0.0}, {"z", 0.0}}},
                {"celestials", json::array({
                                      {
                                              {"name", "earth"},
                                              {"mass_kg", 5.972e24},
                                              {"radius_m", 6371000.0},
                                              {"atmosphere_top_m", 100000.0},
                                              {"terrain_max_m", 0.0},
                                              {"soi_radius_m", 924000000.0},
                                              {"orbit_distance_m", 0.0},
                                              {"has_terrain", false},
                                              {"albedo_dir", ""},
                                              {"height_dir", ""},
                                              {"height_max_m", 0.0},
                                              {"emission_dir", ""},
                                              {"emission_factor", {{"x", 0.0}, {"y", 0.0}, {"z", 0.0}}},
                                              {"render_scale", 1.0},
                                      },
                              })},
                {"orbiters", json::array({
                                     {
                                             {"name", "ship"},
                                             {"orbit_altitude_m", 400000.0},
                                             {"offset_from_player", {{"x", 0.0}, {"y", 0.0}, {"z", 0.0}}},
                                             {"relative_velocity", {{"x", 0.0}, {"y", 0.0}, {"z", 0.0}}},
                                             {"primitive", "capsule"},
                                             {"render_scale", {{"x", 1.0}, {"y", 1.0}, {"z", 1.0}}},
                                             {"body_settings",
                                              {
                                                      {"shape", {{"type", "capsule"}, {"radius", 1.0}, {"half_height", 1.0}}},
                                                      {"user_data", 0},
                                                      {"position", {{"x", 0.0}, {"y", 0.0}, {"z", 0.0}}},
                                                      {"rotation", {{"w", 1.0}, {"x", 0.0}, {"y", 0.0}, {"z", 0.0}}},
                                                      {"motion_type", "dynamic"},
                                                      {"mass", 1.0},
                                                      {"friction", 0.2},
                                                      {"restitution", 0.0},
                                                      {"linear_damping", 0.0},
                                                      {"angular_damping", 0.0},
                                                      {"layer", 4},
                                                      {"is_sensor", false},
                                                      {"start_active", true},
                                                      {"allow_sleeping", true},
                                                      {"gravity_scale", 0.0},
                                              }},
                                             {"is_player", true},
                                             {"is_rebase_anchor", true},
                                     },
                             })},
        };
    }

    std::string write_json_file(const std::filesystem::path &dir, const std::string &filename, const json &doc)
    {
        const std::filesystem::path path = dir / filename;
        std::ofstream out(path);
        out << doc.dump(2);
        out.close();
        return path.string();
    }

    std::string write_raw_file(const std::filesystem::path &dir, const std::string &filename, const std::string &content)
    {
        const std::filesystem::path path = dir / filename;
        std::ofstream out(path);
        out << content;
        out.close();
        return path.string();
    }
} // namespace

TEST(ScenarioLoader, LoadsDefaultScenarioAsset)
{
    const std::string path = std::string(VULKAN_ENGINE_SOURCE_DIR) + "/assets/scenarios/default_gameplay.json";
    auto cfg = Game::load_scenario_config(path);
    ASSERT_TRUE(cfg.has_value());
    EXPECT_FALSE(cfg->celestials.empty());
    EXPECT_FALSE(cfg->orbiters.empty());
}

TEST(ScenarioLoader, RejectsMissingRequiredArray)
{
    const std::filesystem::path dir = make_temp_dir();
    json root = make_valid_scenario();
    root.erase("orbiters");
    auto cfg = Game::load_scenario_config(write_json_file(dir, "missing_orbiters.json", root));
    EXPECT_FALSE(cfg.has_value());
    std::error_code ec;
    std::filesystem::remove_all(dir, ec);
}

TEST(ScenarioLoader, RejectsTypeMismatch)
{
    const std::filesystem::path dir = make_temp_dir();
    json root = make_valid_scenario();
    root["speed_scale"] = "fast";
    auto cfg = Game::load_scenario_config(write_json_file(dir, "type_mismatch.json", root));
    EXPECT_FALSE(cfg.has_value());
    std::error_code ec;
    std::filesystem::remove_all(dir, ec);
}

TEST(ScenarioLoader, RejectsInvalidEnumValue)
{
    const std::filesystem::path dir = make_temp_dir();
    json root = make_valid_scenario();
    root["orbiters"][0]["primitive"] = "torus";
    auto cfg = Game::load_scenario_config(write_json_file(dir, "invalid_enum.json", root));
    EXPECT_FALSE(cfg.has_value());
    std::error_code ec;
    std::filesystem::remove_all(dir, ec);
}

TEST(ScenarioLoader, RejectsInvalidNumericRange)
{
    const std::filesystem::path dir = make_temp_dir();
    json root = make_valid_scenario();
    root["celestials"][0]["radius_m"] = -1.0;
    auto cfg = Game::load_scenario_config(write_json_file(dir, "invalid_range.json", root));
    EXPECT_FALSE(cfg.has_value());
    std::error_code ec;
    std::filesystem::remove_all(dir, ec);
}

TEST(ScenarioLoader, RejectsMalformedJson)
{
    const std::filesystem::path dir = make_temp_dir();
    auto cfg = Game::load_scenario_config(write_raw_file(dir, "malformed.json", "{ broken "));
    EXPECT_FALSE(cfg.has_value());
    std::error_code ec;
    std::filesystem::remove_all(dir, ec);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
