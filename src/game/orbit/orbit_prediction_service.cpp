#include "game/orbit/orbit_prediction_service.h"
#include "game/orbit/orbit_prediction_math.h"
#include "game/orbit/orbit_prediction_tuning.h"

#include "orbitsim/coordinate_frames.hpp"
#include "orbitsim/detail/spacecraft_propagation.hpp"
#include "orbitsim/frame_utils.hpp"
#include "orbitsim/maneuvers.hpp"
#include "orbitsim/integrators.hpp"
#include "orbitsim/maneuvers_types.hpp"
#include "orbitsim/spacecraft_state_cache.hpp"
#include "orbitsim/trajectories.hpp"
#include "orbitsim/trajectory_transforms.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>

namespace Game
{
    namespace
    {
        constexpr double kEphemerisDurationEpsilonS = 1.0e-6;
        constexpr double kEphemerisDtEpsilonS = 1.0e-9;
        constexpr std::size_t kMaxCachedEphemerides = 64;
        using CancelCheck = std::function<bool()>;

        bool finite_vec3(const glm::dvec3 &v)
        {
            return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
        }

        bool build_maneuver_preview(const orbitsim::State &ship_state,
                                    const double node_time_s,
                                    OrbitPredictionService::ManeuverNodePreview &out_preview)
        {
            out_preview.valid = false;
            if (!std::isfinite(node_time_s))
            {
                return false;
            }

            if (!finite_vec3(ship_state.position_m) || !finite_vec3(ship_state.velocity_mps))
            {
                return false;
            }

            out_preview.inertial_position_m = ship_state.position_m;
            out_preview.inertial_velocity_mps = ship_state.velocity_mps;
            out_preview.valid = true;
            return true;
        }

        std::vector<orbitsim::TrajectorySegment> trajectory_segments_from_samples(
                const std::vector<orbitsim::TrajectorySample> &samples)
        {
            std::vector<orbitsim::TrajectorySegment> out;
            if (samples.size() < 2)
            {
                return out;
            }

            out.reserve(samples.size() - 1);
            for (std::size_t i = 1; i < samples.size(); ++i)
            {
                const orbitsim::TrajectorySample &a = samples[i - 1];
                const orbitsim::TrajectorySample &b = samples[i];
                const double dt_s = b.t_s - a.t_s;
                if (!(dt_s > 0.0) || !std::isfinite(dt_s))
                {
                    continue;
                }

                orbitsim::State start{};
                start.position_m = a.position_m;
                start.velocity_mps = a.velocity_mps;
                orbitsim::State end{};
                end.position_m = b.position_m;
                end.velocity_mps = b.velocity_mps;

                out.push_back(orbitsim::TrajectorySegment{
                        .t0_s = a.t_s,
                        .dt_s = dt_s,
                        .start = start,
                        .end = end,
                        .flags = 0u,
                });
            }

            return out;
        }

        struct PlannedSegmentBoundaryState
        {
            double t_s{0.0};
            orbitsim::State state_before{};
            orbitsim::State state_after{};
        };

        double segment_end_time(const orbitsim::TrajectorySegment &segment)
        {
            return segment.t0_s + segment.dt_s;
        }

        bool eval_segment_state(const orbitsim::TrajectorySegment &segment,
                                const double t_s,
                                orbitsim::State &out_state)
        {
            out_state = segment.start;
            if (!(segment.dt_s > 0.0) || !std::isfinite(segment.dt_s) || !std::isfinite(t_s))
            {
                return finite_vec3(out_state.position_m) && finite_vec3(out_state.velocity_mps);
            }

            double u = (t_s - segment.t0_s) / segment.dt_s;
            if (!std::isfinite(u))
            {
                u = 0.0;
            }
            u = std::clamp(u, 0.0, 1.0);

            const double u2 = u * u;
            const double u3 = u2 * u;
            const double h00 = (2.0 * u3) - (3.0 * u2) + 1.0;
            const double h10 = u3 - (2.0 * u2) + u;
            const double h01 = (-2.0 * u3) + (3.0 * u2);
            const double h11 = u3 - u2;
            const double dh00 = (6.0 * u2) - (6.0 * u);
            const double dh10 = (3.0 * u2) - (4.0 * u) + 1.0;
            const double dh01 = (-6.0 * u2) + (6.0 * u);
            const double dh11 = (3.0 * u2) - (2.0 * u);

            const glm::dvec3 p0 = glm::dvec3(segment.start.position_m);
            const glm::dvec3 p1 = glm::dvec3(segment.end.position_m);
            const glm::dvec3 m0 = glm::dvec3(segment.start.velocity_mps) * segment.dt_s;
            const glm::dvec3 m1 = glm::dvec3(segment.end.velocity_mps) * segment.dt_s;
            const glm::dvec3 pos = (h00 * p0) + (h10 * m0) + (h01 * p1) + (h11 * m1);
            const glm::dvec3 vel = ((dh00 * p0) + (dh10 * m0) + (dh01 * p1) + (dh11 * m1)) / segment.dt_s;
            if (!finite_vec3(pos) || !finite_vec3(vel))
            {
                return false;
            }

            out_state = orbitsim::make_state(pos, vel);
            return true;
        }

        bool sample_trajectory_segment_state(const std::vector<orbitsim::TrajectorySegment> &segments,
                                             const double t_s,
                                             orbitsim::State &out_state)
        {
            out_state = {};
            if (segments.empty() || !std::isfinite(t_s))
            {
                return false;
            }

            const auto it = std::lower_bound(segments.begin(),
                                             segments.end(),
                                             t_s,
                                             [](const orbitsim::TrajectorySegment &segment, const double query_t_s) {
                                                 return segment_end_time(segment) < query_t_s;
                                             });
            if (it == segments.end())
            {
                return eval_segment_state(segments.back(), segment_end_time(segments.back()), out_state);
            }

            return eval_segment_state(*it, t_s, out_state);
        }

        bool finite_state(const orbitsim::State &state)
        {
            return finite_vec3(state.position_m) && finite_vec3(state.velocity_mps);
        }

        void append_or_merge_planned_boundary_state(std::vector<PlannedSegmentBoundaryState> &states,
                                                    const double t_s,
                                                    const orbitsim::State &state_before,
                                                    const orbitsim::State &state_after)
        {
            if (!std::isfinite(t_s) || !finite_state(state_before) || !finite_state(state_after))
            {
                return;
            }

            constexpr double kBoundaryMergeToleranceS = 1.0e-9;
            if (!states.empty() && std::abs(states.back().t_s - t_s) <= kBoundaryMergeToleranceS)
            {
                states.back().state_after = state_after;
                return;
            }

            states.push_back(PlannedSegmentBoundaryState{
                    .t_s = t_s,
                    .state_before = state_before,
                    .state_after = state_after,
            });
        }

        std::vector<orbitsim::TrajectorySegment> split_trajectory_segments_at_known_boundaries(
                const std::vector<orbitsim::TrajectorySegment> &segments,
                const std::vector<PlannedSegmentBoundaryState> &boundaries)
        {
            if (segments.empty() || boundaries.empty())
            {
                return segments;
            }

            constexpr double kBoundaryEpsilonS = 1.0e-9;
            std::vector<orbitsim::TrajectorySegment> out;
            out.reserve(segments.size() + boundaries.size());

            std::size_t boundary_index = 0;
            for (const orbitsim::TrajectorySegment &segment : segments)
            {
                const double seg_t0_s = segment.t0_s;
                const double seg_t1_s = segment_end_time(segment);
                if (!(segment.dt_s > 0.0) || !std::isfinite(seg_t0_s) || !std::isfinite(seg_t1_s) || !(seg_t1_s > seg_t0_s))
                {
                    out.push_back(segment);
                    continue;
                }

                double cursor_t_s = seg_t0_s;
                orbitsim::State cursor_state = segment.start;

                while (boundary_index < boundaries.size() &&
                       boundaries[boundary_index].t_s <= (seg_t0_s + kBoundaryEpsilonS))
                {
                    if (std::abs(boundaries[boundary_index].t_s - seg_t0_s) <= kBoundaryEpsilonS)
                    {
                        cursor_state = boundaries[boundary_index].state_after;
                    }
                    ++boundary_index;
                }

                std::size_t local_boundary_index = boundary_index;
                while (local_boundary_index < boundaries.size())
                {
                    const PlannedSegmentBoundaryState &boundary = boundaries[local_boundary_index];
                    if (!(boundary.t_s < (seg_t1_s - kBoundaryEpsilonS)))
                    {
                        break;
                    }
                    if (!(boundary.t_s > (cursor_t_s + kBoundaryEpsilonS)))
                    {
                        cursor_state = boundary.state_after;
                        ++local_boundary_index;
                        continue;
                    }

                    const double part_dt_s = boundary.t_s - cursor_t_s;
                    if (part_dt_s > kBoundaryEpsilonS)
                    {
                        out.push_back(orbitsim::TrajectorySegment{
                                .t0_s = cursor_t_s,
                                .dt_s = part_dt_s,
                                .start = cursor_state,
                                .end = boundary.state_before,
                                .flags = segment.flags,
                        });
                    }

                    cursor_t_s = boundary.t_s;
                    cursor_state = boundary.state_after;
                    ++local_boundary_index;
                }

                const double tail_dt_s = seg_t1_s - cursor_t_s;
                if (tail_dt_s > kBoundaryEpsilonS)
                {
                    out.push_back(orbitsim::TrajectorySegment{
                            .t0_s = cursor_t_s,
                            .dt_s = tail_dt_s,
                            .start = cursor_state,
                            .end = segment.end,
                            .flags = segment.flags,
                    });
                }

                boundary_index = local_boundary_index;
            }

            return out;
        }

        struct MultiBandSampleWindow
        {
            double t0_s{0.0};
            double t1_s{0.0};
            double density{1.0};
            std::size_t interval_count{0};
            double sample_dt_s{0.0};
        };

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

        std::vector<MultiBandSampleWindow> build_multi_band_sample_windows(const double start_time_s,
                                                                           const double horizon_s,
                                                                           const std::size_t total_sample_budget)
        {
            std::vector<MultiBandSampleWindow> windows{};
            if (!(horizon_s > 0.0) || !std::isfinite(start_time_s))
            {
                return windows;
            }

            struct BandTemplate
            {
                double duration_cap_s;
                double density;
            };

            constexpr std::array<BandTemplate, 3> kBandTemplates{{
                    {OrbitPredictionTuning::kMultiBandNearDurationS, OrbitPredictionTuning::kMultiBandNearDensity},
                    {OrbitPredictionTuning::kMultiBandMidDurationS, OrbitPredictionTuning::kMultiBandMidDensity},
                    {std::numeric_limits<double>::infinity(), OrbitPredictionTuning::kMultiBandFarDensity},
            }};

            double cursor_s = start_time_s;
            const double end_time_s = start_time_s + horizon_s;
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
                        .interval_count = 0,
                        .sample_dt_s = 0.0,
                });
                cursor_s = band_end_s;
                if (!(end_time_s > cursor_s))
                {
                    break;
                }
            }

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

            std::vector<MultiBandSampleWindow> windows{};
            if (!(horizon_s > 0.0) || !std::isfinite(start_time_s))
            {
                return windows;
            }

            // Phase A: build base time-bands with density tags (no budget yet).
            struct BandTemplate
            {
                double duration_cap_s;
                double density;
            };

            constexpr std::array<BandTemplate, 3> kBandTemplates{{
                    {OrbitPredictionTuning::kMultiBandNearDurationS, OrbitPredictionTuning::kMultiBandNearDensity},
                    {OrbitPredictionTuning::kMultiBandMidDurationS, OrbitPredictionTuning::kMultiBandMidDensity},
                    {std::numeric_limits<double>::infinity(), OrbitPredictionTuning::kMultiBandFarDensity},
            }};

            const double end_time_s = start_time_s + horizon_s;
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

            if (windows.empty())
            {
                return windows;
            }

            // Phase B: insert high-density windows around each maneuver node.
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

            // Phase C: prune degenerate windows.
            windows.erase(std::remove_if(windows.begin(),
                                         windows.end(),
                                         [](const MultiBandSampleWindow &w) { return !(w.t1_s > w.t0_s); }),
                          windows.end());

            if (windows.empty())
            {
                return windows;
            }

            // Phase D: distribute sample budget across all windows.
            distribute_sample_budget(windows, total_sample_budget, start_time_s, end_time_s, horizon_s);
            return windows;
        }

        std::vector<orbitsim::TrajectorySample> sample_trajectory_segments_multi_band(
                const std::vector<orbitsim::TrajectorySegment> &segments,
                const std::vector<MultiBandSampleWindow> &windows)
        {
            std::vector<orbitsim::TrajectorySample> samples{};
            if (segments.empty() || windows.empty())
            {
                return samples;
            }

            std::size_t reserve_count = 1;
            for (const MultiBandSampleWindow &window : windows)
            {
                reserve_count += window.interval_count;
            }
            samples.reserve(reserve_count);

            for (std::size_t band_idx = 0; band_idx < windows.size(); ++band_idx)
            {
                const MultiBandSampleWindow &window = windows[band_idx];
                if (!(window.t1_s > window.t0_s) || window.interval_count == 0)
                {
                    continue;
                }

                for (std::size_t i = 0; i <= window.interval_count; ++i)
                {
                    if (band_idx > 0 && i == 0)
                    {
                        continue;
                    }

                    const double u =
                            static_cast<double>(i) / static_cast<double>(std::max<std::size_t>(1, window.interval_count));
                    const double t_s = std::clamp(window.t0_s + ((window.t1_s - window.t0_s) * u),
                                                  window.t0_s,
                                                  window.t1_s);

                    orbitsim::State state{};
                    if (!sample_trajectory_segment_state(segments, t_s, state))
                    {
                        continue;
                    }

                    samples.push_back(orbitsim::TrajectorySample{
                            .t_s = t_s,
                            .position_m = state.position_m,
                            .velocity_mps = state.velocity_mps,
                    });
                }
            }

            return samples;
        }

        std::vector<orbitsim::TrajectorySample> sample_body_ephemeris_multi_band(
                const orbitsim::CelestialEphemeris &ephemeris,
                const orbitsim::BodyId body_id,
                const std::vector<MultiBandSampleWindow> &windows)
        {
            std::vector<orbitsim::TrajectorySample> samples{};
            if (body_id == orbitsim::kInvalidBodyId || windows.empty())
            {
                return samples;
            }

            std::size_t reserve_count = 1;
            for (const MultiBandSampleWindow &window : windows)
            {
                reserve_count += window.interval_count;
            }
            samples.reserve(reserve_count);

            for (std::size_t band_idx = 0; band_idx < windows.size(); ++band_idx)
            {
                const MultiBandSampleWindow &window = windows[band_idx];
                if (!(window.t1_s > window.t0_s) || window.interval_count == 0)
                {
                    continue;
                }

                for (std::size_t i = 0; i <= window.interval_count; ++i)
                {
                    if (band_idx > 0 && i == 0)
                    {
                        continue;
                    }

                    const double u =
                            static_cast<double>(i) / static_cast<double>(std::max<std::size_t>(1, window.interval_count));
                    const double t_s = std::clamp(window.t0_s + ((window.t1_s - window.t0_s) * u),
                                                  window.t0_s,
                                                  window.t1_s);
                    const orbitsim::State state = ephemeris.body_state_at_by_id(body_id, t_s);
                    if (!finite_vec3(state.position_m) || !finite_vec3(state.velocity_mps))
                    {
                        continue;
                    }

                    samples.push_back(orbitsim::TrajectorySample{
                            .t_s = t_s,
                            .position_m = state.position_m,
                            .velocity_mps = state.velocity_mps,
                    });
                }
            }

            return samples;
        }

        bool finite_scalar(const double value)
        {
            return std::isfinite(value);
        }

        struct SpacecraftSamplingBudget
        {
            double target_samples_max{OrbitPredictionTuning::kSpacecraftTargetSamplesSoftNormal};
            int soft_max_steps{OrbitPredictionTuning::kSpacecraftMaxStepsSoftNormal};
            int hard_max_steps{OrbitPredictionTuning::kSpacecraftMaxStepsHardNormal};
        };

        bool request_uses_long_range_prediction_policy(const OrbitPredictionService::Request &request)
        {
            return std::isfinite(request.future_window_s) &&
                   request.future_window_s > OrbitPredictionTuning::kLongRangeHorizonThresholdS;
        }

        double resolve_prediction_horizon_cap_s(const OrbitPredictionService::Request &request)
        {
            return request_uses_long_range_prediction_policy(request)
                           ? OrbitPredictionTuning::kLongRangeHorizonCapS
                           : OrbitPredictionTuning::kMaxHorizonS;
        }

        double resolve_prediction_sample_dt_cap_s(const OrbitPredictionService::Request &request)
        {
            if (request.lagrange_sensitive)
            {
                return OrbitPredictionTuning::kMaxSampleDtS;
            }

            return request_uses_long_range_prediction_policy(request)
                           ? OrbitPredictionTuning::kLongRangeMaxSampleDtS
                           : OrbitPredictionTuning::kMaxSampleDtS;
        }

        std::size_t resolve_spacecraft_segment_budget(const OrbitPredictionService::Request &request,
                                                      const double horizon_s,
                                                      const std::size_t sample_budget)
        {
            std::size_t budget = std::max<std::size_t>(1, sample_budget);
            if (!request_uses_long_range_prediction_policy(request) || !(horizon_s > 0.0))
            {
                return budget;
            }

            const std::size_t desired_long_range_budget =
                    static_cast<std::size_t>(std::ceil(horizon_s / OrbitPredictionTuning::kLongRangeSegmentTargetDtS));
            budget = std::max(budget, desired_long_range_budget);
            return std::clamp<std::size_t>(budget, 1, OrbitPredictionTuning::kLongRangeMaxSegmentsHard);
        }

        template<typename BodyPositionAt>
        std::size_t select_primary_index_with_hysteresis(const std::vector<orbitsim::MassiveBody> &bodies,
                                                         const orbitsim::Vec3 &query_pos_m,
                                                         BodyPositionAt body_position_at,
                                                         const double softening_length_m,
                                                         const orbitsim::BodyId preferred_body_id)
        {
            if (bodies.empty())
            {
                return 0;
            }

            const double eps2 = softening_length_m * softening_length_m;
            std::size_t best_index = 0;
            double best_metric = -1.0;
            std::optional<std::size_t> preferred_index;
            double preferred_metric = -1.0;

            for (std::size_t i = 0; i < bodies.size(); ++i)
            {
                const double mass_kg = std::isfinite(bodies[i].mass_kg) ? bodies[i].mass_kg : 0.0;
                if (!(mass_kg >= 0.0))
                {
                    continue;
                }

                const orbitsim::Vec3 dr = body_position_at(i) - query_pos_m;
                const double r2 = glm::dot(dr, dr) + eps2;
                if (!(r2 > 0.0) || !std::isfinite(r2))
                {
                    continue;
                }

                const double metric = mass_kg / r2;
                if (metric > best_metric)
                {
                    best_metric = metric;
                    best_index = i;
                }

                if (bodies[i].id == preferred_body_id)
                {
                    preferred_index = i;
                    preferred_metric = metric;
                }
            }

            if (preferred_index.has_value() &&
                preferred_metric > 0.0 &&
                best_metric > 0.0 &&
                preferred_metric >= (best_metric * OrbitPredictionTuning::kPrimaryBodyHysteresisKeepRatio))
            {
                return *preferred_index;
            }

            return best_index;
        }

        void apply_lagrange_integrator_profile(orbitsim::GameSimulation::Config &sim_config)
        {
            orbitsim::DOPRI5Options &integrator = sim_config.spacecraft_integrator;
            if (!(integrator.max_step_s > 0.0) || integrator.max_step_s > OrbitPredictionTuning::kLagrangeIntegratorMaxStepS)
            {
                integrator.max_step_s = OrbitPredictionTuning::kLagrangeIntegratorMaxStepS;
            }
            integrator.abs_tol = std::min(integrator.abs_tol, OrbitPredictionTuning::kLagrangeIntegratorAbsTol);
            integrator.rel_tol = std::min(integrator.rel_tol, OrbitPredictionTuning::kLagrangeIntegratorRelTol);
        }

        std::size_t count_future_maneuver_impulses(const OrbitPredictionService::Request &request)
        {
            return static_cast<std::size_t>(std::count_if(
                    request.maneuver_impulses.begin(),
                    request.maneuver_impulses.end(),
                    [&request](const OrbitPredictionService::ManeuverImpulse &impulse) {
                        return std::isfinite(impulse.t_s) && impulse.t_s >= request.sim_time_s;
                    }));
        }

        bool request_needs_control_sensitive_prediction(const OrbitPredictionService::Request &request)
        {
            return request.thrusting || count_future_maneuver_impulses(request) > 0;
        }

        double resolve_prediction_integrator_max_step_s(const OrbitPredictionService::Request &request)
        {
            if (request_needs_control_sensitive_prediction(request))
            {
                return OrbitPredictionTuning::kPredictionIntegratorMaxStepControlledS;
            }

            return request_uses_long_range_prediction_policy(request)
                           ? OrbitPredictionTuning::kPredictionIntegratorMaxStepLongRangeS
                           : OrbitPredictionTuning::kPredictionIntegratorMaxStepS;
        }

        void apply_prediction_integrator_profile(orbitsim::GameSimulation::Config &sim_config,
                                                 const OrbitPredictionService::Request &request)
        {
            orbitsim::DOPRI5Options &integrator = sim_config.spacecraft_integrator;
            const double max_step_s = resolve_prediction_integrator_max_step_s(request);
            if (!(integrator.max_step_s > 0.0) || integrator.max_step_s > max_step_s)
            {
                integrator.max_step_s = max_step_s;
            }

            integrator.max_substeps =
                    std::max(integrator.max_substeps, OrbitPredictionTuning::kPredictionIntegratorMaxSubstepsSoft);
            integrator.max_substeps_hard =
                    std::max({integrator.max_substeps_hard,
                              integrator.max_substeps,
                              OrbitPredictionTuning::kPredictionIntegratorMaxSubstepsHard});
            integrator.max_interval_splits =
                    std::max(integrator.max_interval_splits, OrbitPredictionTuning::kPredictionIntegratorMaxIntervalSplits);
        }

        std::size_t resolve_prediction_ephemeris_max_segments(const OrbitPredictionService::Request &request)
        {
            if (request.lagrange_sensitive)
            {
                return OrbitPredictionTuning::kLagrangeEphemerisMaxSamples;
            }

            return OrbitPredictionTuning::kPredictionEphemerisMaxSegmentsHard;
        }

        double resolve_prediction_ephemeris_dt_cap_s(const OrbitPredictionService::Request &request)
        {
            double dt_cap_s = request_uses_long_range_prediction_policy(request)
                                      ? OrbitPredictionTuning::kPredictionEphemerisMaxDtLongRangeS
                                      : OrbitPredictionTuning::kPredictionEphemerisMaxDtS;
            if (request_needs_control_sensitive_prediction(request))
            {
                dt_cap_s = std::min(dt_cap_s, OrbitPredictionTuning::kPredictionEphemerisMaxDtControlledS);
            }
            if (request.lagrange_sensitive)
            {
                dt_cap_s = std::min(dt_cap_s, OrbitPredictionTuning::kLagrangeEphemerisMaxDtS);
            }
            return dt_cap_s;
        }

        double resolve_prediction_ephemeris_dt_s(const OrbitPredictionService::Request &request,
                                                 const OrbitPredictionService::EphemerisSamplingSpec &sampling_spec)
        {
            double dt_s = sampling_spec.sample_dt_s;
            dt_s = std::min(dt_s, resolve_prediction_ephemeris_dt_cap_s(request));
            if (request.celestial_ephemeris_dt_s > 0.0)
            {
                dt_s = std::min(dt_s, request.celestial_ephemeris_dt_s);
            }

            const std::size_t max_segments = std::max<std::size_t>(1, resolve_prediction_ephemeris_max_segments(request));
            const double min_dt_for_budget_s = sampling_spec.horizon_s / static_cast<double>(max_segments);
            if (std::isfinite(min_dt_for_budget_s) && min_dt_for_budget_s > 0.0)
            {
                dt_s = std::max(dt_s, min_dt_for_budget_s);
            }

            return std::max(1.0e-3, dt_s);
        }

        SpacecraftSamplingBudget build_spacecraft_sampling_budget(const OrbitPredictionService::Request &request)
        {
            SpacecraftSamplingBudget out{};
            const double future_maneuvers = static_cast<double>(count_future_maneuver_impulses(request));

            const double target_bonus = std::min(
                    future_maneuvers * OrbitPredictionTuning::kSpacecraftTargetSamplesBonusPerManeuver,
                    OrbitPredictionTuning::kSpacecraftTargetSamplesHardNormal -
                            OrbitPredictionTuning::kSpacecraftTargetSamplesSoftNormal);
            out.target_samples_max =
                    std::clamp(OrbitPredictionTuning::kSpacecraftTargetSamplesSoftNormal + target_bonus,
                               OrbitPredictionTuning::kSpacecraftTargetSamplesSoftNormal,
                               OrbitPredictionTuning::kSpacecraftTargetSamplesHardNormal);

            const double step_bonus = std::min(
                    future_maneuvers * static_cast<double>(OrbitPredictionTuning::kSpacecraftMaxStepsBonusPerManeuver),
                    static_cast<double>(OrbitPredictionTuning::kSpacecraftMaxStepsHardNormal -
                                        OrbitPredictionTuning::kSpacecraftMaxStepsSoftNormal));
            out.soft_max_steps =
                    std::clamp(OrbitPredictionTuning::kSpacecraftMaxStepsSoftNormal + static_cast<int>(std::lround(step_bonus)),
                               OrbitPredictionTuning::kSpacecraftMaxStepsSoftNormal,
                               OrbitPredictionTuning::kSpacecraftMaxStepsHardNormal);
            return out;
        }

        bool same_config_for_ephemeris(const orbitsim::GameSimulation::Config &a,
                                       const orbitsim::GameSimulation::Config &b)
        {
            return a.gravitational_constant == b.gravitational_constant &&
                   a.softening_length_m == b.softening_length_m;
        }

        bool same_spin_state(const orbitsim::SpinState &a, const orbitsim::SpinState &b)
        {
            return a.axis == b.axis &&
                   a.angle_rad == b.angle_rad &&
                   a.rate_rad_per_s == b.rate_rad_per_s;
        }

        bool same_state_for_ephemeris(const orbitsim::State &a, const orbitsim::State &b)
        {
            return a.position_m == b.position_m &&
                   a.velocity_mps == b.velocity_mps &&
                   same_spin_state(a.spin, b.spin);
        }

        bool same_massive_body_for_ephemeris(const orbitsim::MassiveBody &a, const orbitsim::MassiveBody &b)
        {
            return a.id == b.id &&
                   a.mass_kg == b.mass_kg &&
                   same_state_for_ephemeris(a.state, b.state);
        }

        bool same_massive_body_set_for_ephemeris(const std::vector<orbitsim::MassiveBody> &a,
                                                 const std::vector<orbitsim::MassiveBody> &b)
        {
            if (a.size() != b.size())
            {
                return false;
            }

            for (std::size_t i = 0; i < a.size(); ++i)
            {
                if (!same_massive_body_for_ephemeris(a[i], b[i]))
                {
                    return false;
                }
            }

            return true;
        }

        bool compatible_cached_ephemeris(const OrbitPredictionService::CachedEphemerisEntry &entry,
                                         const OrbitPredictionService::EphemerisBuildRequest &request)
        {
            if (!entry.ephemeris || entry.ephemeris->empty())
            {
                return false;
            }

            if (entry.sim_time_s != request.sim_time_s)
            {
                return false;
            }

            if (!same_config_for_ephemeris(entry.sim_config, request.sim_config))
            {
                return false;
            }

            if (!same_massive_body_set_for_ephemeris(entry.massive_bodies, request.massive_bodies))
            {
                return false;
            }

            if ((entry.duration_s + kEphemerisDurationEpsilonS) < request.duration_s)
            {
                return false;
            }

            if (entry.celestial_dt_s > (request.celestial_dt_s + kEphemerisDtEpsilonS))
            {
                return false;
            }

            return true;
        }

        OrbitPredictionService::SharedCelestialEphemeris build_ephemeris_from_request(
                const OrbitPredictionService::EphemerisBuildRequest &request,
                const CancelCheck &cancel_requested = {})
        {
            if (!finite_scalar(request.sim_time_s) ||
                !(request.duration_s > 0.0) ||
                !(request.celestial_dt_s > 0.0) ||
                request.max_samples == 0)
            {
                return {};
            }

            orbitsim::GameSimulation sim(request.sim_config);
            if (!sim.set_time_s(request.sim_time_s))
            {
                return {};
            }

            if (cancel_requested && cancel_requested())
            {
                return {};
            }

            for (const orbitsim::MassiveBody &body : request.massive_bodies)
            {
                const auto body_handle =
                        (body.id != orbitsim::kInvalidBodyId)
                                ? sim.create_body_with_id(body.id, body)
                                : sim.create_body(body);
                if (!body_handle.valid())
                {
                    return {};
                }
            }

            auto ephemeris = std::make_shared<orbitsim::CelestialEphemeris>();
            std::vector<orbitsim::BodyId> body_ids;
            body_ids.reserve(request.massive_bodies.size());
            for (const orbitsim::MassiveBody &body : request.massive_bodies)
            {
                body_ids.push_back(body.id);
            }
            ephemeris->set_body_ids(std::move(body_ids));

            std::vector<orbitsim::MassiveBody> massive = sim.massive_bodies();
            double t_s = request.sim_time_s;
            const double t_end_s = request.sim_time_s + request.duration_s;
            std::vector<orbitsim::State> start_states;
            std::vector<orbitsim::State> end_states;

            while (t_s < t_end_s && ephemeris->segments.size() < request.max_samples)
            {
                if (cancel_requested && cancel_requested())
                {
                    return {};
                }

                const double h_s = std::min(request.celestial_dt_s, t_end_s - t_s);
                orbitsim::detail::snapshot_states(massive, &start_states);
                orbitsim::symplectic4_step(
                        massive,
                        h_s,
                        request.sim_config.gravitational_constant,
                        request.sim_config.softening_length_m);
                orbitsim::detail::snapshot_states(massive, &end_states);

                ephemeris->segments.push_back(orbitsim::CelestialEphemerisSegment{
                        .t0_s = t_s,
                        .dt_s = h_s,
                        .start = start_states,
                        .end = end_states,
                });
                t_s += h_s;
            }

            if (!ephemeris || ephemeris->empty())
            {
                return {};
            }

            return ephemeris;
        }

        OrbitPredictionService::EphemerisBuildRequest build_ephemeris_build_request(
                const OrbitPredictionService::Request &request,
                const OrbitPredictionService::EphemerisSamplingSpec &sampling_spec)
        {
            OrbitPredictionService::EphemerisBuildRequest out{};
            out.sim_time_s = request.sim_time_s;
            out.sim_config = request.sim_config;
            out.massive_bodies = request.massive_bodies;
            out.duration_s = sampling_spec.horizon_s;
            out.celestial_dt_s = resolve_prediction_ephemeris_dt_s(request, sampling_spec);
            const std::size_t desired_segments =
                    static_cast<std::size_t>(std::ceil(out.duration_s / out.celestial_dt_s));
            out.max_samples = std::max<std::size_t>(sampling_spec.max_samples, desired_segments);
            out.max_samples = std::min(out.max_samples, resolve_prediction_ephemeris_max_segments(request));
            return out;
        }

        struct CelestialPredictionSamplingSpec
        {
            bool valid{false};
            glm::dvec3 rel_pos_m{0.0};
            glm::dvec3 rel_vel_mps{0.0};
            double horizon_s{0.0};
            double sample_dt_s{0.0};
            std::size_t max_samples{0};
        };

        CelestialPredictionSamplingSpec build_celestial_prediction_sampling_spec(
                const OrbitPredictionService::Request &request,
                const orbitsim::MassiveBody &subject_body)
        {
            CelestialPredictionSamplingSpec out{};
            if (request.massive_bodies.size() < 2)
            {
                return out;
            }

            std::vector<orbitsim::MassiveBody> candidate_bodies;
            candidate_bodies.reserve(request.massive_bodies.size() - 1);
            for (const orbitsim::MassiveBody &body : request.massive_bodies)
            {
                if (body.id != subject_body.id)
                {
                    candidate_bodies.push_back(body);
                }
            }
            if (candidate_bodies.empty())
            {
                return out;
            }

            const std::size_t primary_index = orbitsim::auto_select_primary_index(
                    candidate_bodies,
                    subject_body.state.position_m,
                    [&candidate_bodies](const std::size_t i) -> orbitsim::Vec3 { return candidate_bodies[i].state.position_m; },
                    request.sim_config.softening_length_m);
            const orbitsim::MassiveBody &reference_body = candidate_bodies[primary_index];

            out.rel_pos_m = glm::dvec3(subject_body.state.position_m - reference_body.state.position_m);
            out.rel_vel_mps = glm::dvec3(subject_body.state.velocity_mps - reference_body.state.velocity_mps);
            if (!finite_vec3(out.rel_pos_m) || !finite_vec3(out.rel_vel_mps))
            {
                return out;
            }

            const double mu_ref_m3_s2 = request.sim_config.gravitational_constant * reference_body.mass_kg;
            if (!(mu_ref_m3_s2 > 0.0) || !std::isfinite(mu_ref_m3_s2))
            {
                return out;
            }

            const auto [horizon_s_auto, dt_s_auto] =
                    OrbitPredictionMath::select_prediction_horizon_and_dt(mu_ref_m3_s2, out.rel_pos_m, out.rel_vel_mps);

            const double horizon_cap_s = resolve_prediction_horizon_cap_s(request);
            const double sample_dt_cap_s = resolve_prediction_sample_dt_cap_s(request);
            double horizon_s = std::clamp(horizon_s_auto, OrbitPredictionTuning::kMinHorizonS, horizon_cap_s);
            horizon_s = std::max(horizon_s, std::max(1.0, request.future_window_s));
            double dt_s = std::clamp(dt_s_auto, 0.01, sample_dt_cap_s);
            const int max_steps = OrbitPredictionTuning::kMaxStepsNormal;
            const double min_dt_for_step_budget = horizon_s / static_cast<double>(std::max(1, max_steps));
            if (std::isfinite(min_dt_for_step_budget) && min_dt_for_step_budget > 0.0)
            {
                dt_s = std::max(dt_s, min_dt_for_step_budget);
            }

            dt_s = std::clamp(dt_s, 0.01, sample_dt_cap_s);
            horizon_s = std::clamp(horizon_s, dt_s, horizon_cap_s);
            const int sample_count = std::clamp(static_cast<int>(std::ceil(horizon_s / dt_s)) + 1, 2, max_steps);
            if (!(horizon_s > 0.0) || !(dt_s > 0.0) || sample_count < 2)
            {
                return out;
            }

            out.valid = true;
            out.horizon_s = horizon_s;
            out.sample_dt_s = dt_s;
            out.max_samples = static_cast<std::size_t>(sample_count);
            return out;
        }
    } // namespace

    OrbitPredictionService::OrbitPredictionService()
    {
        // Use a small worker pool so independent visible tracks can solve in parallel.
        const unsigned int hw_threads = std::max(1u, std::thread::hardware_concurrency());
        const std::size_t worker_count = std::clamp<std::size_t>(static_cast<std::size_t>(hw_threads > 1 ? hw_threads - 1 : 1),
                                                                 1,
                                                                 4);
        _workers.reserve(worker_count);
        for (std::size_t i = 0; i < worker_count; ++i)
        {
            _workers.emplace_back(&OrbitPredictionService::worker_loop, this);
        }
    }

    OrbitPredictionService::~OrbitPredictionService()
    {
        {
            std::lock_guard<std::mutex> lock(_mutex);
            // Stop accepting/publishing work before waking the worker for shutdown.
            _running = false;
            _pending_jobs.clear();
            _completed.clear();
        }
        _cv.notify_all();

        for (std::thread &worker : _workers)
        {
            if (worker.joinable())
            {
                worker.join();
            }
        }
    }

    void OrbitPredictionService::request(Request request)
    {
        {
            std::lock_guard<std::mutex> lock(_mutex);
            const uint64_t generation_id = _next_generation_id++;
            const uint64_t track_id = request.track_id;
            const uint64_t request_epoch = _request_epoch;
            _latest_requested_generation_by_track[track_id] = generation_id;

            // Keep only the newest queued request per track to avoid backlogging stale previews.
            auto existing = std::find_if(_pending_jobs.begin(),
                                         _pending_jobs.end(),
                                         [track_id](const PendingJob &job) { return job.track_id == track_id; });
            if (existing != _pending_jobs.end())
            {
                existing->track_id = track_id;
                existing->request_epoch = request_epoch;
                existing->generation_id = generation_id;
                existing->request = std::move(request);
            }
            else
            {
                PendingJob job{};
                job.track_id = track_id;
                job.request_epoch = request_epoch;
                job.generation_id = generation_id;
                job.request = std::move(request);
                _pending_jobs.push_back(std::move(job));
            }
        }
        _cv.notify_one();
    }

    std::optional<OrbitPredictionService::Result> OrbitPredictionService::poll_completed()
    {
        std::lock_guard<std::mutex> lock(_mutex);
        if (_completed.empty())
        {
            return std::nullopt;
        }

        Result out = std::move(_completed.front());
        _completed.pop_front();
        return out;
    }

    void OrbitPredictionService::reset()
    {
        {
            std::lock_guard<std::mutex> lock(_mutex);
            // Bump the epoch so any in-flight jobs become stale even if they finish later.
            ++_request_epoch;
            _pending_jobs.clear();
            _completed.clear();
            _latest_requested_generation_by_track.clear();
        }

        {
            std::lock_guard<std::mutex> cache_lock(_ephemeris_mutex);
            _ephemeris_cache.clear();
            _next_ephemeris_use_serial = 1;
        }
    }

    OrbitPredictionService::SharedCelestialEphemeris OrbitPredictionService::get_or_build_ephemeris(
            const EphemerisBuildRequest &request)
    {
        return get_or_build_ephemeris(request, {});
    }

    OrbitPredictionService::SharedCelestialEphemeris OrbitPredictionService::get_or_build_ephemeris(
            const EphemerisBuildRequest &request,
            const std::function<bool()> &cancel_requested)
    {
        // Reuse an equal-or-better ephemeris when another prediction already paid the build cost.
        {
            std::lock_guard<std::mutex> lock(_ephemeris_mutex);
            for (CachedEphemerisEntry &entry : _ephemeris_cache)
            {
                if (!compatible_cached_ephemeris(entry, request))
                {
                    continue;
                }

                entry.last_use_serial = _next_ephemeris_use_serial++;
                return entry.ephemeris;
            }
        }

        SharedCelestialEphemeris built = build_ephemeris_from_request(request, cancel_requested);
        if (!built)
        {
            return {};
        }

        std::lock_guard<std::mutex> lock(_ephemeris_mutex);
        // Re-check after the build in case another worker filled the cache first.
        for (CachedEphemerisEntry &entry : _ephemeris_cache)
        {
            if (!compatible_cached_ephemeris(entry, request))
            {
                continue;
            }

            entry.last_use_serial = _next_ephemeris_use_serial++;
            return entry.ephemeris;
        }

        CachedEphemerisEntry entry{};
        entry.sim_time_s = request.sim_time_s;
        entry.duration_s = request.duration_s;
        entry.celestial_dt_s = request.celestial_dt_s;
        entry.sim_config = request.sim_config;
        entry.massive_bodies = request.massive_bodies;
        entry.ephemeris = std::move(built);
        entry.last_use_serial = _next_ephemeris_use_serial++;
        _ephemeris_cache.push_back(std::move(entry));

        if (_ephemeris_cache.size() > kMaxCachedEphemerides)
        {
            // Bound memory usage with a simple LRU eviction policy.
            auto lru_it = std::min_element(_ephemeris_cache.begin(),
                                           _ephemeris_cache.end(),
                                           [](const CachedEphemerisEntry &a, const CachedEphemerisEntry &b) {
                                               return a.last_use_serial < b.last_use_serial;
                                           });
            if (lru_it != _ephemeris_cache.end())
            {
                _ephemeris_cache.erase(lru_it);
            }
        }

        return _ephemeris_cache.back().ephemeris;
    }

    OrbitPredictionService::EphemerisSamplingSpec OrbitPredictionService::build_ephemeris_sampling_spec(const Request &request)
    {
        EphemerisSamplingSpec out{};
        if (!std::isfinite(request.sim_time_s) || request.massive_bodies.empty())
        {
            return out;
        }

        const std::size_t primary_index = select_primary_index_with_hysteresis(
                request.massive_bodies,
                request.ship_bary_position_m,
                [&request](const std::size_t i) -> orbitsim::Vec3 { return request.massive_bodies[i].state.position_m; },
                request.sim_config.softening_length_m,
                request.preferred_primary_body_id);
        if (primary_index >= request.massive_bodies.size())
        {
            return out;
        }

        const orbitsim::MassiveBody &primary_body = request.massive_bodies[primary_index];
        const double mu_ref_m3_s2 = request.sim_config.gravitational_constant * primary_body.mass_kg;
        if (!(mu_ref_m3_s2 > 0.0) || !std::isfinite(mu_ref_m3_s2))
        {
            return out;
        }

        const glm::dvec3 ship_rel_pos_m = request.ship_bary_position_m - glm::dvec3(primary_body.state.position_m);
        const glm::dvec3 ship_rel_vel_mps = request.ship_bary_velocity_mps - glm::dvec3(primary_body.state.velocity_mps);
        if (!finite_vec3(ship_rel_pos_m) || !finite_vec3(ship_rel_vel_mps))
        {
            return out;
        }

        const auto [horizon_s_auto, dt_s_auto] =
                OrbitPredictionMath::select_prediction_horizon_and_dt(mu_ref_m3_s2, ship_rel_pos_m, ship_rel_vel_mps);

        // Start from orbital-state-driven defaults, then clamp into service-wide budgets.
        const double horizon_cap_s = resolve_prediction_horizon_cap_s(request);
        double horizon_s = std::clamp(horizon_s_auto, OrbitPredictionTuning::kMinHorizonS, horizon_cap_s);
        double dt_s = std::clamp(dt_s_auto, 0.01, resolve_prediction_sample_dt_cap_s(request));
        const SpacecraftSamplingBudget sampling_budget = build_spacecraft_sampling_budget(request);
        int max_steps = sampling_budget.soft_max_steps;
        double dt_min_s = 0.01;
        double dt_max_s = resolve_prediction_sample_dt_cap_s(request);
        // Keep solver coverage aligned with the UI's requested future draw window.
        double min_horizon_s = std::max(1.0, request.future_window_s);
        horizon_s = std::max(horizon_s, min_horizon_s);

        if (request.thrusting)
        {
            // During active thrust we trade long horizons for denser near-term sampling.
            const double thrust_horizon_cap_s =
                    std::clamp(std::max(OrbitPredictionTuning::kThrustHorizonMinS,
                                        std::max(0.0, request.future_window_s) * OrbitPredictionTuning::kThrustHorizonWindowScale),
                               OrbitPredictionTuning::kThrustHorizonMinS,
                               OrbitPredictionTuning::kThrustHorizonMaxS);
            horizon_s = std::min(horizon_s, thrust_horizon_cap_s);
            horizon_s = std::max(horizon_s, min_horizon_s);

            const double target_samples = std::clamp(horizon_s / OrbitPredictionTuning::kTargetSamplesDivisorS,
                                                     OrbitPredictionTuning::kThrustTargetSamplesMin,
                                                     OrbitPredictionTuning::kThrustTargetSamplesMax);
            dt_min_s = OrbitPredictionTuning::kThrustMinSampleDtS;
            dt_max_s = OrbitPredictionTuning::kThrustMaxSampleDtS;
            dt_s = std::clamp(horizon_s / target_samples, dt_min_s, dt_max_s);
            max_steps = OrbitPredictionTuning::kMaxStepsThrust;
        }
        else
        {
            const double target_samples = std::clamp(horizon_s / OrbitPredictionTuning::kTargetSamplesDivisorS,
                                                     OrbitPredictionTuning::kTargetSamplesMin,
                                                     sampling_budget.target_samples_max);
            dt_s = std::clamp(horizon_s / target_samples, dt_min_s, dt_max_s);

            const int required_steps = std::clamp(static_cast<int>(std::ceil(horizon_s / dt_s)),
                                                  2,
                                                  sampling_budget.hard_max_steps);
            max_steps = std::clamp(std::max(sampling_budget.soft_max_steps, required_steps),
                                   2,
                                   sampling_budget.hard_max_steps);
        }

        const double min_dt_for_step_budget = horizon_s / static_cast<double>(std::max(1, max_steps));
        if (std::isfinite(min_dt_for_step_budget) && min_dt_for_step_budget > 0.0)
        {
            dt_s = std::max(dt_s, min_dt_for_step_budget);
        }

        dt_s = std::clamp(dt_s, dt_min_s, dt_max_s);
        horizon_s = std::clamp(horizon_s, dt_s, horizon_cap_s);
        const int steps = std::clamp(static_cast<int>(std::ceil(horizon_s / dt_s)), 2, max_steps);

        out.valid = (steps >= 2) && (horizon_s > 0.0) && (dt_s > 0.0);
        out.horizon_s = horizon_s;
        out.sample_dt_s = dt_s;
        out.max_samples = static_cast<std::size_t>(steps) + 1;
        return out;
    }

    bool OrbitPredictionService::should_publish_result(
            const PendingJob &job,
            const uint64_t current_request_epoch,
            const std::unordered_map<uint64_t, uint64_t> &latest_requested_generation_by_track)
    {
        // reset() invalidates every queued and in-flight job by moving to a new epoch.
        if (job.request_epoch != current_request_epoch)
        {
            return false;
        }

        // Only publish the newest generation for each track.
        const auto latest_it = latest_requested_generation_by_track.find(job.track_id);
        return latest_it == latest_requested_generation_by_track.end() ||
               job.generation_id >= latest_it->second;
    }

    bool OrbitPredictionService::should_continue_job(const uint64_t track_id,
                                                     const uint64_t generation_id,
                                                     const uint64_t request_epoch) const
    {
        std::lock_guard<std::mutex> lock(_mutex);
        if (!_running || request_epoch != _request_epoch)
        {
            return false;
        }

        const auto latest_it = _latest_requested_generation_by_track.find(track_id);
        return latest_it == _latest_requested_generation_by_track.end() ||
               generation_id >= latest_it->second;
    }

    void OrbitPredictionService::worker_loop()
    {
        while (true)
        {
            PendingJob job{};
            {
                std::unique_lock<std::mutex> lock(_mutex);
                _cv.wait(lock, [this]() {
                    return !_running || !_pending_jobs.empty();
                });

                if (!_running && _pending_jobs.empty())
                {
                    return;
                }

                if (_pending_jobs.empty())
                {
                    continue;
                }

                job = std::move(_pending_jobs.front());
                _pending_jobs.pop_front();
            }

            if (!should_continue_job(job.track_id, job.generation_id, job.request_epoch))
            {
                continue;
            }

            // Run the expensive simulation outside the lock so request() stays responsive.
            Result result = compute_prediction(job.generation_id, job.request, job.request_epoch);

            {
                std::lock_guard<std::mutex> lock(_mutex);
                if (!_running)
                {
                    return;
                }

                if (!should_publish_result(job, _request_epoch, _latest_requested_generation_by_track))
                {
                    continue;
                }

                _completed.push_back(std::move(result));
            }
        }
    }

    OrbitPredictionService::Result OrbitPredictionService::compute_prediction(const uint64_t generation_id,
                                                                              const Request &request,
                                                                              const uint64_t request_epoch)
    {
        Result out{};
        struct ScopedComputeTimer
        {
            Result *result{nullptr};
            std::chrono::steady_clock::time_point start{};
            ~ScopedComputeTimer()
            {
                if (!result)
                {
                    return;
                }
                result->compute_time_ms =
                        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
            }
        };
        ScopedComputeTimer timer{};
        timer.result = &out;
        timer.start = std::chrono::steady_clock::now();

        out.generation_id = generation_id;
        out.track_id = request.track_id;
        out.build_time_s = request.sim_time_s;
        const auto cancel_requested = [this,
                                       track_id = request.track_id,
                                       generation_id,
                                       request_epoch]() {
            return !should_continue_job(track_id, generation_id, request_epoch);
        };

        // Invalid inputs produce an invalid result rather than throwing on the worker thread.
        if (!std::isfinite(request.sim_time_s))
        {
            return out;
        }

        orbitsim::GameSimulation::Config sim_config = request.sim_config;
        apply_prediction_integrator_profile(sim_config, request);
        if (request.lagrange_sensitive)
        {
            apply_lagrange_integrator_profile(sim_config);
        }

        orbitsim::GameSimulation sim(sim_config);
        if (!sim.set_time_s(request.sim_time_s))
        {
            return out;
        }

        if (cancel_requested())
        {
            return out;
        }

        for (const orbitsim::MassiveBody &body : request.massive_bodies)
        {
            const auto body_handle =
                    (body.id != orbitsim::kInvalidBodyId)
                        ? sim.create_body_with_id(body.id, body)
                        : sim.create_body(body);
            if (!body_handle.valid())
            {
                return out;
            }
        }

        if (cancel_requested())
        {
            return out;
        }

        out.massive_bodies = sim.massive_bodies();

        // Celestial route: predict the body's inertial motion.
        if (request.kind == RequestKind::Celestial)
        {
            if (request.subject_body_id == orbitsim::kInvalidBodyId)
            {
                return out;
            }

            const orbitsim::MassiveBody *subject_sim = sim.body_by_id(request.subject_body_id);
            if (!subject_sim)
            {
                return out;
            }

            const CelestialPredictionSamplingSpec sampling_spec =
                    build_celestial_prediction_sampling_spec(request, *subject_sim);
            if (!sampling_spec.valid)
            {
                return out;
            }

            SharedCelestialEphemeris shared_ephemeris = request.shared_ephemeris;
            if (!shared_ephemeris ||
                shared_ephemeris->empty() ||
                (shared_ephemeris->t_end_s() + kEphemerisDurationEpsilonS) < (request.sim_time_s + sampling_spec.horizon_s))
            {
                // Keep ephemeris construction on the worker so gameplay only enqueues work.
                EphemerisSamplingSpec ephemeris_sampling_spec{};
                ephemeris_sampling_spec.valid = true;
                ephemeris_sampling_spec.horizon_s = sampling_spec.horizon_s;
                ephemeris_sampling_spec.sample_dt_s = sampling_spec.sample_dt_s;
                ephemeris_sampling_spec.max_samples = sampling_spec.max_samples;
                shared_ephemeris = get_or_build_ephemeris(build_ephemeris_build_request(request, ephemeris_sampling_spec),
                                                          cancel_requested);
            }
            if (!shared_ephemeris || shared_ephemeris->empty())
            {
                return out;
            }

            if (cancel_requested())
            {
                return out;
            }

            const std::vector<MultiBandSampleWindow> sample_windows =
                    build_multi_band_sample_windows(request.sim_time_s,
                                                    sampling_spec.horizon_s,
                                                    sampling_spec.max_samples);
            std::vector<orbitsim::TrajectorySample> traj_inertial =
                    sample_body_ephemeris_multi_band(*shared_ephemeris, subject_sim->id, sample_windows);
            if (traj_inertial.size() < 2)
            {
                return out;
            }

            out.shared_ephemeris = shared_ephemeris;
            out.trajectory_inertial = std::move(traj_inertial);
            out.trajectory_segments_inertial = trajectory_segments_from_samples(out.trajectory_inertial);
            out.valid = true;
            return out;
        }

        // Spacecraft route: build a transient ship state, then predict inertial baseline and planned trajectories.
        const glm::dvec3 ship_bary_pos_m = request.ship_bary_position_m;
        const glm::dvec3 ship_bary_vel_mps = request.ship_bary_velocity_mps;
        if (!finite_vec3(ship_bary_pos_m) || !finite_vec3(ship_bary_vel_mps))
        {
            return out;
        }

        const EphemerisSamplingSpec sampling_spec = build_ephemeris_sampling_spec(request);
        if (!sampling_spec.valid)
        {
            return out;
        }

        const double horizon_s = sampling_spec.horizon_s;
        orbitsim::Spacecraft ship_sc{};
        ship_sc.state = orbitsim::make_state(ship_bary_pos_m, ship_bary_vel_mps);
        ship_sc.dry_mass_kg = 1.0;

        const auto ship_h = sim.create_spacecraft(ship_sc);
        if (!ship_h.valid())
        {
            return out;
        }

        sim.maneuver_plan() = orbitsim::ManeuverPlan{};

        orbitsim::TrajectorySegmentOptions segment_opt{};
        segment_opt.duration_s = horizon_s;
        segment_opt.max_segments =
                resolve_spacecraft_segment_budget(request,
                                                 horizon_s,
                                                 (sampling_spec.max_samples > 0) ? (sampling_spec.max_samples - 1) : 0);
        segment_opt.include_start = true;
        segment_opt.include_end = true;
        segment_opt.stop_on_impact = false;
        // Keep trajectory segment dt driven by duration/max_segments. In orbitsim, lookup_dt_s also
        // overrides the emitted segment step, which silently truncates coverage to
        // max_segments * lookup_dt_s for long-range predictions.

        SharedCelestialEphemeris shared_ephemeris = request.shared_ephemeris;
        if (!shared_ephemeris ||
            shared_ephemeris->empty() ||
            (shared_ephemeris->t_end_s() + kEphemerisDurationEpsilonS) < (request.sim_time_s + horizon_s))
        {
            // Spacecraft requests follow the same rule: never build ephemeris on the gameplay thread.
            shared_ephemeris = get_or_build_ephemeris(build_ephemeris_build_request(request, sampling_spec),
                                                      cancel_requested);
        }
        if (!shared_ephemeris || shared_ephemeris->empty())
        {
            return out;
        }

        if (cancel_requested())
        {
            return out;
        }
        const orbitsim::CelestialEphemeris &eph = *shared_ephemeris;

        const std::vector<orbitsim::TrajectorySegment> traj_segments_inertial_baseline =
                orbitsim::predict_spacecraft_trajectory_segments(sim, eph, ship_h.id, segment_opt);
        if (traj_segments_inertial_baseline.empty())
        {
            return out;
        }

        if (cancel_requested())
        {
            return out;
        }

        const double baseline_end_s = segment_end_time(traj_segments_inertial_baseline.back());
        const std::vector<MultiBandSampleWindow> baseline_sample_windows =
                build_multi_band_sample_windows(request.sim_time_s,
                                                std::max(0.0, baseline_end_s - request.sim_time_s),
                                                sampling_spec.max_samples);
        const std::vector<orbitsim::TrajectorySample> traj_inertial_baseline =
                sample_trajectory_segments_multi_band(traj_segments_inertial_baseline, baseline_sample_windows);
        if (traj_inertial_baseline.size() < 2)
        {
            return out;
        }

        if (cancel_requested())
        {
            return out;
        }

        out.shared_ephemeris = shared_ephemeris;
        out.trajectory_segments_inertial = traj_segments_inertial_baseline;
        out.trajectory_inertial = traj_inertial_baseline;

        if (!request.maneuver_impulses.empty())
        {
            std::vector<ManeuverImpulse> maneuver_impulses = request.maneuver_impulses;
            std::stable_sort(maneuver_impulses.begin(),
                             maneuver_impulses.end(),
                             [](const ManeuverImpulse &a, const ManeuverImpulse &b) { return a.t_s < b.t_s; });

            orbitsim::ManeuverPlan plan{};
            plan.impulses.reserve(maneuver_impulses.size());
            out.maneuver_previews.reserve(maneuver_impulses.size());
            std::vector<PlannedSegmentBoundaryState> planned_boundary_states;
            planned_boundary_states.reserve(maneuver_impulses.size());
            orbitsim::Spacecraft preview_spacecraft = ship_sc;
            preview_spacecraft.id = ship_h.id;
            double preview_time_s = request.sim_time_s;
            const orbitsim::SpacecraftStateLookup empty_lookup{};

            // Propagate once from node-to-node so long plans do not repeatedly restart from t0.
            for (const ManeuverImpulse &src : maneuver_impulses)
            {
                if (cancel_requested())
                {
                    return out;
                }

                if (!std::isfinite(src.t_s) || !finite_vec3(src.dv_rtn_mps))
                {
                    continue;
                }

                if (src.t_s > preview_time_s)
                {
                    preview_spacecraft = orbitsim::detail::propagate_spacecraft_in_ephemeris(
                            preview_spacecraft,
                            out.massive_bodies,
                            eph,
                            orbitsim::ManeuverPlan{},
                            sim.config().gravitational_constant,
                            sim.config().softening_length_m,
                            sim.config().spacecraft_integrator,
                            preview_time_s,
                            src.t_s - preview_time_s,
                            empty_lookup);
                    preview_time_s = src.t_s;
                }

                ManeuverNodePreview preview{};
                preview.node_id = src.node_id;
                preview.t_s = src.t_s;

                const orbitsim::State pre_burn_state = preview_spacecraft.state;
                if (build_maneuver_preview(preview_spacecraft.state, src.t_s, preview))
                {
                    out.maneuver_previews.push_back(preview);
                }

                orbitsim::ImpulseSegment impulse{};
                impulse.t_s = src.t_s;
                impulse.primary_body_id = src.primary_body_id;
                impulse.dv_rtn_mps = src.dv_rtn_mps;
                impulse.spacecraft_id = ship_h.id;
                plan.impulses.push_back(impulse);

                if (preview.valid)
                {
                    std::optional<std::size_t> primary_index = orbitsim::body_index_for_id(out.massive_bodies, src.primary_body_id);
                    if (!primary_index.has_value())
                    {
                        primary_index = orbitsim::auto_select_primary_index(
                                out.massive_bodies,
                                preview.inertial_position_m,
                                [&eph, time_s = src.t_s](const std::size_t i) -> orbitsim::Vec3 {
                                    return eph.body_position_at(i, time_s);
                                },
                                sim.config().softening_length_m);
                    }

                    if (primary_index.has_value() && *primary_index < out.massive_bodies.size())
                    {
                        const orbitsim::Vec3 dv_inertial_mps = orbitsim::rtn_vector_to_inertial(
                                eph,
                                out.massive_bodies,
                                *primary_index,
                                src.t_s,
                                preview.inertial_position_m,
                                preview.inertial_velocity_mps,
                                src.dv_rtn_mps);
                        if (finite_vec3(dv_inertial_mps))
                        {
                            preview_spacecraft.state.velocity_mps += dv_inertial_mps;
                        }
                    }
                }

                append_or_merge_planned_boundary_state(
                        planned_boundary_states,
                        src.t_s,
                        pre_burn_state,
                        preview_spacecraft.state);
            }

            orbitsim::sort_impulses_by_time(plan);
            sim.maneuver_plan() = std::move(plan);

            // Planned output shows the same prediction window with all maneuver nodes applied.
            std::vector<orbitsim::TrajectorySegment> traj_segments_inertial_planned =
                    orbitsim::predict_spacecraft_trajectory_segments(sim, eph, ship_h.id, segment_opt);
            traj_segments_inertial_planned =
                    split_trajectory_segments_at_known_boundaries(traj_segments_inertial_planned, planned_boundary_states);
            if (!traj_segments_inertial_planned.empty())
            {
                if (cancel_requested())
                {
                    return out;
                }

                out.trajectory_segments_inertial_planned = traj_segments_inertial_planned;

                const double planned_end_s = segment_end_time(traj_segments_inertial_planned.back());
                std::vector<double> node_times;
                node_times.reserve(request.maneuver_impulses.size());
                for (const auto &imp : request.maneuver_impulses)
                {
                    node_times.push_back(imp.t_s);
                }
                const std::vector<MultiBandSampleWindow> planned_sample_windows =
                        build_multi_band_sample_windows(request.sim_time_s,
                                                        std::max(0.0, planned_end_s - request.sim_time_s),
                                                        sampling_spec.max_samples,
                                                        node_times);
                const std::vector<orbitsim::TrajectorySample> traj_inertial_planned =
                        sample_trajectory_segments_multi_band(traj_segments_inertial_planned, planned_sample_windows);
                if (traj_inertial_planned.size() >= 2)
                {
                    out.trajectory_inertial_planned = traj_inertial_planned;
                }
            }
        }

        out.valid = true;
        return out;
    }
} // namespace Game
