#include "game/orbit/prediction/orbit_prediction_service_internal.h"

namespace Game
{
    void distribute_sample_budget(std::vector<MultiBandSampleWindow> &windows,
                                  const std::size_t total_sample_budget,
                                  const double start_time_s,
                                  const double end_time_s,
                                  const double horizon_s)
    {
        if (windows.empty())
        {
            return;
        }

        std::size_t total_intervals_budget =
                (total_sample_budget > 1) ? (total_sample_budget - 1) : 1;
        while (windows.size() > 1 && windows.size() > total_intervals_budget)
        {
            windows[windows.size() - 2].t1_s = windows.back().t1_s;
            windows.pop_back();
        }

        const std::size_t window_count = windows.size();
        std::vector<double> weights(window_count, 0.0);
        double total_weight = 0.0;
        for (std::size_t i = 0; i < window_count; ++i)
        {
            const double duration_s = windows[i].t1_s - windows[i].t0_s;
            weights[i] = std::max(0.0, duration_s) * windows[i].density;
            total_weight += weights[i];
        }

        if (!(total_weight > 0.0) || !std::isfinite(total_weight))
        {
            windows.assign(1, MultiBandSampleWindow{
                                  .t0_s = start_time_s,
                                  .t1_s = end_time_s,
                                  .interval_count = total_intervals_budget,
                                  .sample_dt_s = horizon_s / static_cast<double>(total_intervals_budget),
                          });
            return;
        }

        std::vector<std::size_t> intervals(window_count, 1);
        std::vector<double> fractional(window_count, 0.0);
        std::size_t remaining_intervals =
                (total_intervals_budget > window_count) ? (total_intervals_budget - window_count) : 0;
        if (total_intervals_budget <= window_count)
        {
            remaining_intervals = 0;
            intervals.assign(window_count, 1);
        }
        else
        {
            std::size_t assigned_extra = 0;
            for (std::size_t i = 0; i < window_count; ++i)
            {
                const double ideal_extra =
                        (static_cast<double>(remaining_intervals) * weights[i]) / total_weight;
                const std::size_t extra = static_cast<std::size_t>(std::floor(ideal_extra));
                intervals[i] += extra;
                assigned_extra += extra;
                fractional[i] = ideal_extra - static_cast<double>(extra);
            }

            std::size_t extras_left = remaining_intervals - assigned_extra;
            while (extras_left > 0)
            {
                std::size_t best_idx = 0;
                for (std::size_t i = 1; i < window_count; ++i)
                {
                    if (fractional[i] > fractional[best_idx])
                    {
                        best_idx = i;
                    }
                }
                ++intervals[best_idx];
                fractional[best_idx] = 0.0;
                --extras_left;
            }
        }

        for (std::size_t i = 0; i < window_count; ++i)
        {
            windows[i].interval_count = intervals[i];
            const double duration_s = windows[i].t1_s - windows[i].t0_s;
            windows[i].sample_dt_s =
                    (intervals[i] > 0) ? (duration_s / static_cast<double>(intervals[i])) : duration_s;
        }
    }

    std::vector<MultiBandSampleWindow> build_base_band_windows(const double start_time_s,
                                                                const double end_time_s)
    {
        std::vector<MultiBandSampleWindow> windows{};
        double cursor_s = start_time_s;
        for (const BandTemplate &band : kBandTemplates)
        {
            const double band_end_s = std::min(end_time_s, start_time_s + band.duration_cap_s);
            if (!(band_end_s > cursor_s))
            {
                continue;
            }

            windows.push_back(MultiBandSampleWindow{
                    .t0_s = cursor_s,
                    .t1_s = band_end_s,
                    .density = band.density,
            });
            cursor_s = band_end_s;
            if (!(end_time_s > cursor_s))
            {
                break;
            }
        }
        return windows;
    }

    std::vector<MultiBandSampleWindow> build_multi_band_sample_windows(const double start_time_s,
                                                                       const double horizon_s,
                                                                       const std::size_t total_sample_budget)
    {
        if (!(horizon_s > 0.0) || !std::isfinite(start_time_s))
        {
            return {};
        }

        const double end_time_s = start_time_s + horizon_s;
        std::vector<MultiBandSampleWindow> windows = build_base_band_windows(start_time_s, end_time_s);
        if (windows.empty())
        {
            return windows;
        }

        distribute_sample_budget(windows, total_sample_budget, start_time_s, end_time_s, horizon_s);
        return windows;
    }

    std::vector<MultiBandSampleWindow> build_multi_band_sample_windows(const double start_time_s,
                                                                       const double horizon_s,
                                                                       const std::size_t total_sample_budget,
                                                                       const std::vector<double> &node_times_s)
    {
        if (node_times_s.empty())
        {
            return build_multi_band_sample_windows(start_time_s, horizon_s, total_sample_budget);
        }

        if (!(horizon_s > 0.0) || !std::isfinite(start_time_s))
        {
            return {};
        }

        const double end_time_s = start_time_s + horizon_s;
        std::vector<MultiBandSampleWindow> windows = build_base_band_windows(start_time_s, end_time_s);
        if (windows.empty())
        {
            return windows;
        }

        // Insert high-density windows around each maneuver node.
        constexpr double half_w = OrbitPredictionTuning::kMultiBandNodeWindowHalfWidthS;
        constexpr double node_density = OrbitPredictionTuning::kMultiBandNodeDensity;

        for (const double t_node : node_times_s)
        {
            const double nt0 = std::max(start_time_s, t_node - half_w);
            const double nt1 = std::min(end_time_s, t_node + half_w);
            if (!(nt1 > nt0))
            {
                continue;
            }

            std::vector<MultiBandSampleWindow> split{};
            split.reserve(windows.size() + 2);
            bool node_inserted = false;

            for (const MultiBandSampleWindow &w : windows)
            {
                // No overlap: keep as-is.
                if (w.t1_s <= nt0 || w.t0_s >= nt1)
                {
                    split.push_back(w);
                    continue;
                }

                // Left remnant before node window.
                if (w.t0_s < nt0)
                {
                    split.push_back(MultiBandSampleWindow{
                            .t0_s = w.t0_s,
                            .t1_s = nt0,
                            .density = w.density,
                    });
                }

                // Insert or extend the node window (merge overlapping node windows).
                if (!node_inserted)
                {
                    split.push_back(MultiBandSampleWindow{
                            .t0_s = nt0,
                            .t1_s = nt1,
                            .density = node_density,
                    });
                    node_inserted = true;
                }
                else
                {
                    // Previous node window already covers [nt0, ...); extend if needed.
                    MultiBandSampleWindow &prev = split.back();
                    if (prev.density >= node_density)
                    {
                        prev.t1_s = std::max(prev.t1_s, nt1);
                    }
                }

                // Right remnant after node window.
                if (w.t1_s > nt1)
                {
                    split.push_back(MultiBandSampleWindow{
                            .t0_s = nt1,
                            .t1_s = w.t1_s,
                            .density = w.density,
                    });
                }
            }

            windows = std::move(split);
        }

        // Prune degenerate windows.
        windows.erase(std::remove_if(windows.begin(),
                                     windows.end(),
                                     [](const MultiBandSampleWindow &w) { return !(w.t1_s > w.t0_s); }),
                      windows.end());

        if (windows.empty())
        {
            return windows;
        }

        // Distribute sample budget across all windows.
        distribute_sample_budget(windows, total_sample_budget, start_time_s, end_time_s, horizon_s);
        return windows;
    }

    std::vector<orbitsim::TrajectorySample> sample_trajectory_segments_multi_band(
            const std::vector<orbitsim::TrajectorySegment> &segments,
            const std::vector<MultiBandSampleWindow> &windows)
    {
        if (segments.empty())
        {
            return {};
        }

        return sample_multi_band(windows, [&segments](const double t_s) -> std::optional<orbitsim::State> {
            orbitsim::State state{};
            if (!sample_trajectory_segment_state(segments, t_s, state))
            {
                return std::nullopt;
            }
            return state;
        });
    }

    std::vector<orbitsim::TrajectorySample> sample_body_ephemeris_multi_band(
            const orbitsim::CelestialEphemeris &ephemeris,
            const orbitsim::BodyId body_id,
            const std::vector<MultiBandSampleWindow> &windows)
    {
        if (body_id == orbitsim::kInvalidBodyId)
        {
            return {};
        }

        return sample_multi_band(windows, [&ephemeris, body_id](const double t_s) -> std::optional<orbitsim::State> {
            const orbitsim::State state = ephemeris.body_state_at_by_id(body_id, t_s);
            if (!finite_vec3(state.position_m) || !finite_vec3(state.velocity_mps))
            {
                return std::nullopt;
            }
            return state;
        });
    }
} // namespace Game
