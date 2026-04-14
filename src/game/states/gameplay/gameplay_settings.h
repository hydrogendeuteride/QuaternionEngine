#pragma once

#include <optional>
#include <string>

namespace Game
{
    struct PredictionSamplingPolicy
    {
        double orbiter_min_window_s{600.0};
        double celestial_min_window_s{21600.0};
    };

    struct ManeuverPlanHorizonSettings
    {
        double horizon_s{600.0};
    };

    struct OrbitPlotBudgetSettings
    {
        double render_error_px{0.75};
        int render_max_segments_cpu{40'000};
        int pick_max_segments{32'000};
        double pick_frustum_margin_ratio{0.05};
    };

    struct GameplaySettings
    {
        static constexpr int kSchemaVersion = 1;

        // Prediction draw
        bool prediction_draw_full_orbit{true};
        bool prediction_draw_future_segment{true};
        bool prediction_draw_velocity_ray{false};
        float prediction_line_alpha_scale{1.0f};
        float prediction_line_overlay_boost{0.0f};

        // Prediction timing
        double prediction_periodic_refresh_s{0.0};
        double prediction_thrust_refresh_s{0.1};

        // Prediction sampling / plan windows
        PredictionSamplingPolicy prediction_sampling_policy{};
        ManeuverPlanHorizonSettings maneuver_plan_horizon{};

        // Orbit plot rendering
        OrbitPlotBudgetSettings orbit_plot_budget{};

        // Debug toggles
        bool debug_draw_enabled{true};
        bool runtime_orbiter_rails_enabled{true};
        double runtime_orbiter_rails_distance_m{10'000.0};
        bool contact_log_enabled{true};
        bool contact_log_print_console{false};
    };

    std::optional<GameplaySettings> load_gameplay_settings(const std::string &json_path);

    bool save_gameplay_settings(const std::string &json_path, const GameplaySettings &settings);
} // namespace Game
