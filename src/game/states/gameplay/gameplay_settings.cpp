#include "game/states/gameplay/gameplay_settings.h"
#include "core/util/logger.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <filesystem>
#include <fstream>

namespace Game
{
    using json = nlohmann::json;

    namespace
    {
        template<typename T>
        T json_opt(const json &j, const char *key, T fallback)
        {
            auto it = j.find(key);
            if (it == j.end() || it->is_null())
            {
                return fallback;
            }
            try
            {
                return it->get<T>();
            }
            catch (...)
            {
                return fallback;
            }
        }
    } // anonymous namespace

    std::optional<GameplaySettings> load_gameplay_settings(const std::string &json_path)
    {
        std::ifstream file(json_path);
        if (!file.is_open())
        {
            Logger::info("No gameplay settings file: {}", json_path);
            return std::nullopt;
        }

        json root;
        try
        {
            file >> root;
        }
        catch (const json::parse_error &e)
        {
            Logger::error("Gameplay settings JSON parse error '{}': {}", json_path, e.what());
            return std::nullopt;
        }

        if (!root.is_object())
        {
            Logger::error("Gameplay settings root must be an object: {}", json_path);
            return std::nullopt;
        }

        const int version = json_opt<int>(root, "schema_version", 0);
        if (version != GameplaySettings::kSchemaVersion)
        {
            Logger::warn("Gameplay settings schema_version {} != {}, ignoring: {}",
                         version, GameplaySettings::kSchemaVersion, json_path);
            return std::nullopt;
        }

        GameplaySettings s{};

        s.prediction_draw_full_orbit = json_opt<bool>(root, "prediction_draw_full_orbit", s.prediction_draw_full_orbit);
        s.prediction_draw_future_segment = json_opt<bool>(root, "prediction_draw_future_segment", s.prediction_draw_future_segment);
        s.prediction_draw_velocity_ray = json_opt<bool>(root, "prediction_draw_velocity_ray", s.prediction_draw_velocity_ray);
        s.prediction_line_alpha_scale = json_opt<float>(root, "prediction_line_alpha_scale", s.prediction_line_alpha_scale);
        s.prediction_line_overlay_boost = json_opt<float>(root, "prediction_line_overlay_boost", s.prediction_line_overlay_boost);

        s.prediction_periodic_refresh_s = json_opt<double>(root, "prediction_periodic_refresh_s", s.prediction_periodic_refresh_s);
        s.prediction_thrust_refresh_s = json_opt<double>(root, "prediction_thrust_refresh_s", s.prediction_thrust_refresh_s);

        s.prediction_sampling_policy.orbiter_min_window_s = json_opt<double>(
                root,
                "prediction_sampling_orbiter_min_window_s",
                json_opt<double>(
                        root,
                        "prediction_future_window_orbiter_s",
                        s.prediction_sampling_policy.orbiter_min_window_s));
        s.prediction_sampling_policy.celestial_min_window_s = json_opt<double>(
                root,
                "prediction_sampling_celestial_min_window_s",
                json_opt<double>(
                        root,
                        "prediction_future_window_celestial_s",
                        s.prediction_sampling_policy.celestial_min_window_s));
        const double legacy_maneuver_preview_window_s = json_opt<double>(
                root,
                "maneuver_plan_preview_window_s",
                json_opt<double>(
                        root,
                        "prediction_future_window_planned_s",
                        s.maneuver_plan_horizon.horizon_s));
        const double legacy_maneuver_solve_margin_s = json_opt<double>(
                root,
                "maneuver_plan_solve_margin_s",
                legacy_maneuver_preview_window_s);
        s.maneuver_plan_horizon.horizon_s = json_opt<double>(
                root,
                "maneuver_plan_horizon_s",
                std::max(legacy_maneuver_preview_window_s, legacy_maneuver_solve_margin_s));

        s.orbit_plot_budget.render_error_px = json_opt<double>(
                root,
                "orbit_plot_budget_render_error_px",
                json_opt<double>(root, "orbit_plot_render_error_px", s.orbit_plot_budget.render_error_px));
        s.orbit_plot_budget.render_max_segments_cpu = json_opt<int>(
                root,
                "orbit_plot_budget_render_max_segments_cpu",
                json_opt<int>(root, "orbit_plot_render_max_segments_cpu", s.orbit_plot_budget.render_max_segments_cpu));
        s.orbit_plot_budget.pick_max_segments = json_opt<int>(
                root,
                "orbit_plot_budget_pick_max_segments",
                json_opt<int>(root, "orbit_plot_pick_max_segments", s.orbit_plot_budget.pick_max_segments));
        s.orbit_plot_budget.pick_frustum_margin_ratio = json_opt<double>(
                root,
                "orbit_plot_budget_pick_frustum_margin_ratio",
                s.orbit_plot_budget.pick_frustum_margin_ratio);

        s.debug_draw_enabled = json_opt<bool>(root, "debug_draw_enabled", s.debug_draw_enabled);
        s.runtime_orbiter_rails_enabled = json_opt<bool>(root, "runtime_orbiter_rails_enabled", s.runtime_orbiter_rails_enabled);
        s.runtime_orbiter_rails_distance_m = json_opt<double>(root, "runtime_orbiter_rails_distance_m", s.runtime_orbiter_rails_distance_m);
        s.contact_log_enabled = json_opt<bool>(root, "contact_log_enabled", s.contact_log_enabled);
        s.contact_log_print_console = json_opt<bool>(root, "contact_log_print_console", s.contact_log_print_console);

        Logger::info("Loaded gameplay settings: {}", json_path);
        return s;
    }

    bool save_gameplay_settings(const std::string &json_path, const GameplaySettings &s)
    {
        if (json_path.empty())
        {
            Logger::error("Gameplay settings save path is empty.");
            return false;
        }

        try
        {
            const std::filesystem::path out_path(json_path);
            const std::filesystem::path parent = out_path.parent_path();

            if (!parent.empty())
            {
                std::error_code ec;
                std::filesystem::create_directories(parent, ec);
                if (ec)
                {
                    Logger::error("Failed to create settings directory '{}': {}", parent.string(), ec.message());
                    return false;
                }
            }

            json root;
            root["schema_version"] = GameplaySettings::kSchemaVersion;

            root["prediction_draw_full_orbit"] = s.prediction_draw_full_orbit;
            root["prediction_draw_future_segment"] = s.prediction_draw_future_segment;
            root["prediction_draw_velocity_ray"] = s.prediction_draw_velocity_ray;
            root["prediction_line_alpha_scale"] = s.prediction_line_alpha_scale;
            root["prediction_line_overlay_boost"] = s.prediction_line_overlay_boost;

            root["prediction_periodic_refresh_s"] = s.prediction_periodic_refresh_s;
            root["prediction_thrust_refresh_s"] = s.prediction_thrust_refresh_s;

            root["prediction_sampling_orbiter_min_window_s"] = s.prediction_sampling_policy.orbiter_min_window_s;
            root["prediction_sampling_celestial_min_window_s"] = s.prediction_sampling_policy.celestial_min_window_s;
            root["maneuver_plan_horizon_s"] = s.maneuver_plan_horizon.horizon_s;

            root["orbit_plot_budget_render_error_px"] = s.orbit_plot_budget.render_error_px;
            root["orbit_plot_budget_render_max_segments_cpu"] = s.orbit_plot_budget.render_max_segments_cpu;
            root["orbit_plot_budget_pick_max_segments"] = s.orbit_plot_budget.pick_max_segments;
            root["orbit_plot_budget_pick_frustum_margin_ratio"] = s.orbit_plot_budget.pick_frustum_margin_ratio;

            // Legacy aliases kept for compatibility with older local settings files/builds.
            root["maneuver_plan_preview_window_s"] = s.maneuver_plan_horizon.horizon_s;
            root["maneuver_plan_solve_margin_s"] = s.maneuver_plan_horizon.horizon_s;
            root["prediction_future_window_planned_s"] = s.maneuver_plan_horizon.horizon_s;
            root["orbit_plot_render_error_px"] = s.orbit_plot_budget.render_error_px;
            root["orbit_plot_render_max_segments_cpu"] = s.orbit_plot_budget.render_max_segments_cpu;
            root["orbit_plot_pick_max_segments"] = s.orbit_plot_budget.pick_max_segments;

            root["debug_draw_enabled"] = s.debug_draw_enabled;
            root["runtime_orbiter_rails_enabled"] = s.runtime_orbiter_rails_enabled;
            root["runtime_orbiter_rails_distance_m"] = s.runtime_orbiter_rails_distance_m;
            root["contact_log_enabled"] = s.contact_log_enabled;
            root["contact_log_print_console"] = s.contact_log_print_console;

            std::ofstream file(out_path, std::ios::out | std::ios::trunc);
            if (!file.is_open())
            {
                Logger::error("Failed to open gameplay settings for writing: {}", json_path);
                return false;
            }

            file << root.dump(2);
            if (!file.good())
            {
                Logger::error("Failed to write gameplay settings: {}", json_path);
                return false;
            }

            Logger::info("Saved gameplay settings: {}", json_path);
            return true;
        }
        catch (const std::exception &e)
        {
            Logger::error("Gameplay settings save failed '{}': {}", json_path, e.what());
            return false;
        }
    }
} // namespace Game
