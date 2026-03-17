#pragma once

#include <optional>
#include <string>

namespace Game
{
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

        // Prediction windows
        double prediction_future_window_orbiter_s{600.0};
        double prediction_future_window_celestial_s{21600.0};
        double prediction_future_window_planned_s{600.0};

        // Orbit plot rendering
        double orbit_plot_render_error_px{0.75};
        int orbit_plot_render_max_segments_cpu{4'000};
        int orbit_plot_pick_max_segments{8'000};

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
