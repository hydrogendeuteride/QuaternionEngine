#include "game/states/gameplay/scenario_loader.h"

#include <nlohmann/json.hpp>
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <limits>
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

    struct TempDir
    {
        std::filesystem::path path = make_temp_dir();
        ~TempDir()
        {
            std::error_code ec;
            std::filesystem::remove_all(path, ec);
        }
    };

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

TEST(ScenarioLoader, LoadsValidScenarioDocument)
{
    const TempDir tmp;
    auto cfg = Game::load_scenario_config(write_json_file(tmp.path, "valid_scenario.json", make_valid_scenario()));
    ASSERT_TRUE(cfg.has_value());
    EXPECT_FALSE(cfg->celestials.empty());
    EXPECT_FALSE(cfg->orbiters.empty());
}

TEST(ScenarioLoader, RejectsMissingRequiredArray)
{
    const TempDir tmp;
    json root = make_valid_scenario();
    root.erase("orbiters");
    auto cfg = Game::load_scenario_config(write_json_file(tmp.path, "missing_orbiters.json", root));
    EXPECT_FALSE(cfg.has_value());
}

TEST(ScenarioLoader, RejectsTypeMismatch)
{
    const TempDir tmp;
    json root = make_valid_scenario();
    root["speed_scale"] = "fast";
    auto cfg = Game::load_scenario_config(write_json_file(tmp.path, "type_mismatch.json", root));
    EXPECT_FALSE(cfg.has_value());
}

TEST(ScenarioLoader, RejectsInvalidEnumValue)
{
    const TempDir tmp;
    json root = make_valid_scenario();
    root["orbiters"][0]["primitive"] = "torus";
    auto cfg = Game::load_scenario_config(write_json_file(tmp.path, "invalid_enum.json", root));
    EXPECT_FALSE(cfg.has_value());
}

TEST(ScenarioLoader, RejectsInvalidNumericRange)
{
    const TempDir tmp;
    json root = make_valid_scenario();
    root["celestials"][0]["radius_m"] = -1.0;
    auto cfg = Game::load_scenario_config(write_json_file(tmp.path, "invalid_range.json", root));
    EXPECT_FALSE(cfg.has_value());
}

TEST(ScenarioLoader, RejectsMalformedJson)
{
    const TempDir tmp;
    auto cfg = Game::load_scenario_config(write_raw_file(tmp.path, "malformed.json", "{ broken "));
    EXPECT_FALSE(cfg.has_value());
}

TEST(ScenarioLoader, RejectsUnsupportedSchemaVersion)
{
    const TempDir tmp;
    json root = make_valid_scenario();
    root["schema_version"] = 99;
    auto cfg = Game::load_scenario_config(write_json_file(tmp.path, "unsupported_schema.json", root));
    EXPECT_FALSE(cfg.has_value());
}

TEST(ScenarioLoader, RejectsNonPositiveMuBase)
{
    const TempDir tmp;
    json root = make_valid_scenario();
    root["mu_base"] = 0.0;
    auto cfg = Game::load_scenario_config(write_json_file(tmp.path, "invalid_mu_base.json", root));
    EXPECT_FALSE(cfg.has_value());
}

TEST(ScenarioLoader, RejectsEmptyCelestialsArray)
{
    const TempDir tmp;
    json root = make_valid_scenario();
    root["celestials"] = json::array();
    auto cfg = Game::load_scenario_config(write_json_file(tmp.path, "empty_celestials.json", root));
    EXPECT_FALSE(cfg.has_value());
}

TEST(ScenarioLoader, RejectsEmptyOrbitersArray)
{
    const TempDir tmp;
    json root = make_valid_scenario();
    root["orbiters"] = json::array();
    auto cfg = Game::load_scenario_config(write_json_file(tmp.path, "empty_orbiters.json", root));
    EXPECT_FALSE(cfg.has_value());
}

TEST(ScenarioLoader, RejectsTerrainWithoutRequiredDirectories)
{
    const TempDir tmp;
    json root = make_valid_scenario();
    root["celestials"][0]["has_terrain"] = true;
    root["celestials"][0]["albedo_dir"] = "";
    root["celestials"][0]["height_dir"] = "";
    auto cfg = Game::load_scenario_config(write_json_file(tmp.path, "terrain_missing_dirs.json", root));
    EXPECT_FALSE(cfg.has_value());
}

TEST(ScenarioLoader, SaveRejectsEmptyPath)
{
    Game::ScenarioConfig cfg{};
    EXPECT_FALSE(Game::save_scenario_config("", cfg));
}

TEST(ScenarioLoader, SaveCreatesParentDirectoriesAndRoundTrips)
{
    const TempDir tmp;
    const std::string input_path = write_json_file(tmp.path, "valid_for_roundtrip.json", make_valid_scenario());
    const auto loaded = Game::load_scenario_config(input_path);
    ASSERT_TRUE(loaded.has_value());

    const std::filesystem::path output_path = tmp.path / "nested" / "scenario" / "saved.json";
    ASSERT_TRUE(Game::save_scenario_config(output_path.string(), *loaded));
    ASSERT_TRUE(std::filesystem::exists(output_path));

    const auto reloaded = Game::load_scenario_config(output_path.string());
    ASSERT_TRUE(reloaded.has_value());

    EXPECT_EQ(reloaded->celestials.size(), loaded->celestials.size());
    EXPECT_EQ(reloaded->orbiters.size(), loaded->orbiters.size());
    EXPECT_DOUBLE_EQ(reloaded->mu_base, loaded->mu_base);
    EXPECT_DOUBLE_EQ(reloaded->speed_scale, loaded->speed_scale);
    EXPECT_EQ(reloaded->celestials.front().name, loaded->celestials.front().name);
    EXPECT_EQ(reloaded->orbiters.front().name, loaded->orbiters.front().name);
    EXPECT_EQ(reloaded->orbiters.front().body_settings.layer, loaded->orbiters.front().body_settings.layer);
    EXPECT_TRUE(reloaded->orbiters.front().body_settings.shape.is_capsule());
}

TEST(ScenarioLoader, SerializeIncludesExpectedTopLevelFields)
{
    const TempDir tmp;
    const std::string input_path = write_json_file(tmp.path, "valid_for_serialize.json", make_valid_scenario());
    const auto loaded = Game::load_scenario_config(input_path);
    ASSERT_TRUE(loaded.has_value());

    const std::string serialized = Game::serialize_scenario_config(*loaded);
    const json root = json::parse(serialized);

    EXPECT_EQ(root.at("schema_version").get<int>(), 1);
    EXPECT_TRUE(root.contains("speed_scale"));
    EXPECT_TRUE(root.contains("mu_base"));
    EXPECT_TRUE(root.contains("system_center"));
    EXPECT_TRUE(root.contains("celestials"));
    EXPECT_TRUE(root.contains("orbiters"));
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
