#pragma once

/// Shared types, constants, and inline helpers for orbit_prediction_service split files.
/// Used by orbit_prediction_service.cpp, orbit_prediction_service_trajectory.cpp,
/// orbit_prediction_service_sampling.cpp, and orbit_prediction_service_policy.cpp.

#include "game/orbit/orbit_prediction_service.h"
#include "game/orbit/orbit_prediction_math.h"
#include "game/orbit/orbit_prediction_tuning.h"

#include "orbitsim/coordinate_frames.hpp"
#include "orbitsim/game_sim.hpp"
#include "orbitsim/trajectory_segments.hpp"
#include "orbitsim/trajectory_types.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <optional>
#include <vector>

namespace Game
{
    // ── Constants ──────────────────────────────────────────────────────────────
    constexpr double kEphemerisDurationEpsilonS = 1.0e-6;
    constexpr double kEphemerisDtEpsilonS = 1.0e-9;
    constexpr std::size_t kMaxCachedEphemerides = 64;

    using CancelCheck = std::function<bool()>;

    // ── Inline micro-helpers ──────────────────────────────────────────────────
    inline bool finite_vec3(const glm::dvec3 &v)
    {
        return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
    }

    inline bool finite_state(const orbitsim::State &state)
    {
        return finite_vec3(state.position_m) && finite_vec3(state.velocity_mps);
    }

    inline double prediction_segment_end_time(const orbitsim::TrajectorySegment &segment)
    {
        return segment.t0_s + segment.dt_s;
    }

    // ── Structs ───────────────────────────────────────────────────────────────
    struct PlannedSegmentBoundaryState
    {
        double t_s{0.0};
        orbitsim::State state_before{};
        orbitsim::State state_after{};
    };

    struct MultiBandSampleWindow
    {
        double t0_s{0.0};
        double t1_s{0.0};
        double density{1.0};
        std::size_t interval_count{0};
        double sample_dt_s{0.0};
    };

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

    struct SpacecraftSamplingBudget
    {
        double target_samples_max{OrbitPredictionTuning::kSpacecraftTargetSamplesSoftNormal};
        int soft_max_steps{OrbitPredictionTuning::kSpacecraftMaxStepsSoftNormal};
        int hard_max_steps{OrbitPredictionTuning::kSpacecraftMaxStepsHardNormal};
    };

    struct CelestialPredictionSamplingSpec
    {
        bool valid{false};
        glm::dvec3 rel_pos_m{0.0};
        glm::dvec3 rel_vel_mps{0.0};
        double horizon_s{0.0};
        double sample_dt_s{0.0};
        std::size_t max_samples{0};
    };

    // ── Trajectory helpers (orbit_prediction_service_trajectory.cpp) ──────────
    bool build_maneuver_preview(const orbitsim::State &ship_state,
                                const double node_time_s,
                                OrbitPredictionService::ManeuverNodePreview &out_preview);

    std::vector<orbitsim::TrajectorySegment> trajectory_segments_from_samples(
            const std::vector<orbitsim::TrajectorySample> &samples);

    std::vector<orbitsim::TrajectorySegment> trajectory_segments_from_body_ephemeris(
            const orbitsim::CelestialEphemeris &ephemeris,
            orbitsim::BodyId body_id);

    bool eval_segment_state(const orbitsim::TrajectorySegment &segment,
                            const double t_s,
                            orbitsim::State &out_state);

    bool sample_trajectory_segment_state(const std::vector<orbitsim::TrajectorySegment> &segments,
                                         const double t_s,
                                         orbitsim::State &out_state);

    void append_or_merge_planned_boundary_state(std::vector<PlannedSegmentBoundaryState> &states,
                                                const double t_s,
                                                const orbitsim::State &state_before,
                                                const orbitsim::State &state_after);

    std::vector<orbitsim::TrajectorySegment> split_trajectory_segments_at_known_boundaries(
            const std::vector<orbitsim::TrajectorySegment> &segments,
            const std::vector<PlannedSegmentBoundaryState> &boundaries);

    // ── Sampling helpers (orbit_prediction_service_sampling.cpp) ──────────────
    void distribute_sample_budget(std::vector<MultiBandSampleWindow> &windows,
                                  std::size_t total_sample_budget,
                                  double start_time_s,
                                  double end_time_s,
                                  double horizon_s);

    std::vector<MultiBandSampleWindow> build_base_band_windows(double start_time_s, double end_time_s);

    std::vector<MultiBandSampleWindow> build_multi_band_sample_windows(double start_time_s,
                                                                       double horizon_s,
                                                                       std::size_t total_sample_budget);

    std::vector<MultiBandSampleWindow> build_multi_band_sample_windows(double start_time_s,
                                                                       double horizon_s,
                                                                       std::size_t total_sample_budget,
                                                                       const std::vector<double> &node_times_s);

    std::vector<orbitsim::TrajectorySample> sample_trajectory_segments_multi_band(
            const std::vector<orbitsim::TrajectorySegment> &segments,
            const std::vector<MultiBandSampleWindow> &windows);

    std::vector<orbitsim::TrajectorySample> sample_body_ephemeris_multi_band(
            const orbitsim::CelestialEphemeris &ephemeris,
            orbitsim::BodyId body_id,
            const std::vector<MultiBandSampleWindow> &windows);

    // ── sample_multi_band template (used only by sampling .cpp, kept inline) ──
    template<typename StateSamplerFn>
    std::vector<orbitsim::TrajectorySample> sample_multi_band(
            const std::vector<MultiBandSampleWindow> &windows,
            StateSamplerFn &&sample_state_at)
    {
        std::vector<orbitsim::TrajectorySample> samples{};
        if (windows.empty())
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

                const auto state_opt = sample_state_at(t_s);
                if (!state_opt.has_value())
                {
                    continue;
                }

                samples.push_back(orbitsim::TrajectorySample{
                        .t_s = t_s,
                        .position_m = state_opt->position_m,
                        .velocity_mps = state_opt->velocity_mps,
                });
            }
        }

        return samples;
    }

    // ── select_primary_index_with_hysteresis template ─────────────────────────
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

    // ── Policy helpers (orbit_prediction_service_policy.cpp) ──────────────────
    bool request_uses_long_range_prediction_policy(const OrbitPredictionService::Request &request);
    double resolve_prediction_horizon_cap_s(const OrbitPredictionService::Request &request);
    double resolve_prediction_sample_dt_cap_s(const OrbitPredictionService::Request &request);
    std::size_t resolve_spacecraft_segment_budget(const OrbitPredictionService::Request &request,
                                                  double horizon_s,
                                                  std::size_t sample_budget);
    void apply_lagrange_integrator_profile(orbitsim::GameSimulation::Config &sim_config);
    std::size_t count_future_maneuver_impulses(const OrbitPredictionService::Request &request);
    bool request_needs_control_sensitive_prediction(const OrbitPredictionService::Request &request);
    double resolve_prediction_integrator_max_step_s(const OrbitPredictionService::Request &request);
    void apply_prediction_integrator_profile(orbitsim::GameSimulation::Config &sim_config,
                                             const OrbitPredictionService::Request &request);
    std::size_t resolve_prediction_ephemeris_max_segments(const OrbitPredictionService::Request &request);
    double resolve_prediction_ephemeris_dt_cap_s(const OrbitPredictionService::Request &request);
    double resolve_prediction_ephemeris_dt_s(const OrbitPredictionService::Request &request,
                                             const OrbitPredictionService::EphemerisSamplingSpec &sampling_spec);
    SpacecraftSamplingBudget build_spacecraft_sampling_budget(const OrbitPredictionService::Request &request);
    orbitsim::AdaptiveSegmentOptions build_spacecraft_adaptive_segment_options(
            const OrbitPredictionService::Request &request,
            const OrbitPredictionService::EphemerisSamplingSpec &sampling_spec,
            const CancelCheck &cancel_requested = {});
    orbitsim::AdaptiveEphemerisOptions build_adaptive_ephemeris_options(
            const OrbitPredictionService::Request &request,
            const OrbitPredictionService::EphemerisSamplingSpec &sampling_spec,
            const CancelCheck &cancel_requested = {});

    bool ephemeris_covers_horizon(const OrbitPredictionService::SharedCelestialEphemeris &ephemeris,
                                  double sim_time_s,
                                  double horizon_s);
    bool compatible_cached_ephemeris(const OrbitPredictionService::CachedEphemerisEntry &entry,
                                     const OrbitPredictionService::EphemerisBuildRequest &request);
    OrbitPredictionService::SharedCelestialEphemeris build_ephemeris_from_request(
            const OrbitPredictionService::EphemerisBuildRequest &request,
            const CancelCheck &cancel_requested = {});
    OrbitPredictionService::EphemerisBuildRequest build_ephemeris_build_request(
            const OrbitPredictionService::Request &request,
            const OrbitPredictionService::EphemerisSamplingSpec &sampling_spec);
    CelestialPredictionSamplingSpec build_celestial_prediction_sampling_spec(
            const OrbitPredictionService::Request &request,
            const orbitsim::MassiveBody &subject_body);

} // namespace Game
