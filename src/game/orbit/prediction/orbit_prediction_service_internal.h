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
    constexpr double kContinuityMinTimeEpsilonS = 1.0e-6;
    constexpr double kContinuityMinPosToleranceM = 1.0e-3;
    constexpr double kContinuityMinVelToleranceMps = 1.0e-6;

    using CancelCheck = std::function<bool()>;

    inline bool request_uses_fast_preview(const OrbitPredictionService::Request &request)
    {
        return request.solve_quality == OrbitPredictionService::SolveQuality::FastPreview;
    }

    inline bool request_uses_preview_patch(const OrbitPredictionService::Request &request)
    {
        return request_uses_fast_preview(request) &&
               request.preview_patch.active &&
               request.preview_patch.anchor_state_valid &&
               std::isfinite(request.preview_patch.anchor_time_s) &&
               request.preview_patch.anchor_time_s >= request.sim_time_s;
    }

    inline bool request_can_reuse_spacecraft_baseline(const OrbitPredictionService::Request &request)
    {
        return request.kind == OrbitPredictionService::RequestKind::Spacecraft &&
               request_uses_fast_preview(request) &&
               !request.thrusting &&
               !request.maneuver_impulses.empty();
    }

    inline std::size_t prediction_sample_budget(const OrbitPredictionService::Request &request,
                                                const std::size_t segment_count)
    {
        const std::size_t sample_multiplier = request_uses_fast_preview(request) ? 1u : 2u;
        const std::size_t sample_cap =
                request_uses_fast_preview(request) ? OrbitPredictionTuning::kFastPreviewTrajectorySampleCap : 4'000u;
        return std::clamp<std::size_t>(segment_count * sample_multiplier, 2u, sample_cap);
    }

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

    inline double prediction_segment_span_s(const std::vector<orbitsim::TrajectorySegment> &segments)
    {
        if (segments.empty())
        {
            return 0.0;
        }

        const double t0_s = segments.front().t0_s;
        const double t1_s = prediction_segment_end_time(segments.back());
        if (!std::isfinite(t0_s) || !std::isfinite(t1_s) || !(t1_s > t0_s))
        {
            return 0.0;
        }

        return t1_s - t0_s;
    }

    inline double continuity_time_epsilon_s(const double reference_time_s)
    {
        return std::max(kContinuityMinTimeEpsilonS, std::abs(reference_time_s) * 1.0e-12);
    }

    inline double request_end_time_s(const OrbitPredictionService::Request &request)
    {
        return request.sim_time_s + std::max(0.0, std::isfinite(request.future_window_s) ? request.future_window_s : 0.0);
    }

    inline double preview_patch_remaining_window_s(const OrbitPredictionService::Request &request)
    {
        if (!request_uses_preview_patch(request))
        {
            return 0.0;
        }

        return std::max(0.0, request_end_time_s(request) - request.preview_patch.anchor_time_s);
    }

    inline double preview_visual_window_s(const OrbitPredictionService::Request &request)
    {
        if (!request_uses_preview_patch(request))
        {
            return 0.0;
        }

        return std::max(0.0, request.preview_patch.visual_window_s);
    }

    inline double preview_exact_window_s(const OrbitPredictionService::Request &request)
    {
        if (!request_uses_preview_patch(request))
        {
            return 0.0;
        }

        return std::max(0.0, request.preview_patch.exact_window_s);
    }

    inline double preview_streaming_window_s(const OrbitPredictionService::Request &request)
    {
        if (!request_uses_preview_patch(request))
        {
            return 0.0;
        }

        const double chunk_window_s = preview_exact_window_s(request);
        const double streaming_window_s = chunk_window_s > 0.0 ? (chunk_window_s * 2.0) : 0.0;
        return std::min(preview_patch_remaining_window_s(request), streaming_window_s);
    }

    inline bool trajectory_segments_cover_window(const std::vector<orbitsim::TrajectorySegment> &segments,
                                                 const double sim_time_s,
                                                 const double required_duration_s)
    {
        if (segments.empty() || !std::isfinite(sim_time_s))
        {
            return false;
        }

        const double end_time_s = prediction_segment_end_time(segments.back());
        const double start_epsilon_s = continuity_time_epsilon_s(sim_time_s);
        const double end_epsilon_s = continuity_time_epsilon_s(end_time_s);
        const double required_end_s = sim_time_s + std::max(0.0, required_duration_s);
        return segments.front().t0_s <= (sim_time_s + start_epsilon_s) &&
               end_time_s >= (required_end_s - end_epsilon_s);
    }

    inline double continuity_pos_tolerance_m(const orbitsim::State &a, const orbitsim::State &b)
    {
        const double scale_m =
                std::max({1.0, glm::length(glm::dvec3(a.position_m)), glm::length(glm::dvec3(b.position_m))});
        return std::max(kContinuityMinPosToleranceM, scale_m * 1.0e-10);
    }

    inline double continuity_vel_tolerance_mps(const orbitsim::State &a, const orbitsim::State &b)
    {
        const double scale_mps =
                std::max({1.0, glm::length(glm::dvec3(a.velocity_mps)), glm::length(glm::dvec3(b.velocity_mps))});
        return std::max(kContinuityMinVelToleranceMps, scale_mps * 1.0e-10);
    }

    inline bool states_are_continuous(const orbitsim::State &a, const orbitsim::State &b)
    {
        if (!finite_state(a) || !finite_state(b))
        {
            return false;
        }

        const double pos_gap_m = glm::length(glm::dvec3(a.position_m - b.position_m));
        const double vel_gap_mps = glm::length(glm::dvec3(a.velocity_mps - b.velocity_mps));
        return pos_gap_m <= continuity_pos_tolerance_m(a, b) &&
               vel_gap_mps <= continuity_vel_tolerance_mps(a, b);
    }

    inline bool states_are_position_continuous(const orbitsim::State &a, const orbitsim::State &b)
    {
        if (!finite_state(a) || !finite_state(b))
        {
            return false;
        }

        const double pos_gap_m = glm::length(glm::dvec3(a.position_m - b.position_m));
        return pos_gap_m <= continuity_pos_tolerance_m(a, b);
    }

    inline bool segment_allows_velocity_discontinuity(const orbitsim::TrajectorySegment &segment)
    {
        return (segment.flags & (orbitsim::kTrajectorySegmentFlagImpulseBoundary |
                                 orbitsim::kTrajectorySegmentFlagBurnBoundary)) != 0u;
    }

    inline bool validate_trajectory_segment_continuity(const std::vector<orbitsim::TrajectorySegment> &segments)
    {
        if (segments.empty())
        {
            return false;
        }

        for (std::size_t i = 0; i < segments.size(); ++i)
        {
            const orbitsim::TrajectorySegment &segment = segments[i];
            if (!(segment.dt_s > 0.0) || !std::isfinite(segment.t0_s) || !std::isfinite(segment.dt_s) ||
                !finite_state(segment.start) || !finite_state(segment.end))
            {
                return false;
            }

            if (i == 0)
            {
                continue;
            }

            const orbitsim::TrajectorySegment &prev = segments[i - 1];
            const double prev_t1_s = prediction_segment_end_time(prev);
            if (std::abs(segment.t0_s - prev_t1_s) > continuity_time_epsilon_s(prev_t1_s))
            {
                return false;
            }

            if (segment_allows_velocity_discontinuity(segment))
            {
                if (!states_are_position_continuous(prev.end, segment.start))
                {
                    return false;
                }
                continue;
            }

            if (!states_are_continuous(prev.end, segment.start))
            {
                return false;
            }
        }

        return true;
    }

    inline bool validate_ephemeris_continuity(const orbitsim::CelestialEphemeris &ephemeris)
    {
        if (ephemeris.empty())
        {
            return false;
        }

        for (std::size_t seg_idx = 0; seg_idx < ephemeris.segments.size(); ++seg_idx)
        {
            const orbitsim::CelestialEphemerisSegment &segment = ephemeris.segments[seg_idx];
            if (!(segment.dt_s > 0.0) || !std::isfinite(segment.t0_s) || !std::isfinite(segment.dt_s) ||
                segment.start.size() != ephemeris.body_ids.size() ||
                segment.end.size() != ephemeris.body_ids.size())
            {
                return false;
            }

            for (std::size_t body_idx = 0; body_idx < segment.start.size(); ++body_idx)
            {
                if (!finite_state(segment.start[body_idx]) || !finite_state(segment.end[body_idx]))
                {
                    return false;
                }
            }

            if (seg_idx == 0)
            {
                continue;
            }

            const orbitsim::CelestialEphemerisSegment &prev = ephemeris.segments[seg_idx - 1];
            const double prev_t1_s = prev.t0_s + prev.dt_s;
            if (std::abs(segment.t0_s - prev_t1_s) > continuity_time_epsilon_s(prev_t1_s))
            {
                return false;
            }

            for (std::size_t body_idx = 0; body_idx < segment.start.size(); ++body_idx)
            {
                if (!states_are_continuous(prev.end[body_idx], segment.start[body_idx]))
                {
                    return false;
                }
            }
        }

        return true;
    }

    template<typename AdaptiveDiag>
    OrbitPredictionService::AdaptiveStageDiagnostics make_stage_diagnostics_from_adaptive(
            const AdaptiveDiag &diag,
            const double requested_duration_s,
            const bool cache_reused = false)
    {
        OrbitPredictionService::AdaptiveStageDiagnostics out{};
        out.requested_duration_s = std::max(0.0, requested_duration_s);
        out.covered_duration_s = std::max(0.0, diag.covered_duration_s);
        out.accepted_segments = diag.accepted_segments;
        out.rejected_splits = diag.rejected_splits;
        out.forced_boundary_splits = diag.forced_boundary_splits;
        out.min_dt_s = diag.min_dt_s;
        out.max_dt_s = diag.max_dt_s;
        out.avg_dt_s = diag.avg_dt_s;
        out.hard_cap_hit = diag.hard_cap_hit;
        out.cancelled = diag.cancelled;
        out.cache_reused = cache_reused;
        return out;
    }

    inline OrbitPredictionService::AdaptiveStageDiagnostics make_stage_diagnostics_from_segments(
            const std::vector<orbitsim::TrajectorySegment> &segments,
            const double requested_duration_s,
            const bool cache_reused = false)
    {
        OrbitPredictionService::AdaptiveStageDiagnostics out{};
        out.requested_duration_s = std::max(0.0, requested_duration_s);
        out.covered_duration_s = prediction_segment_span_s(segments);
        out.accepted_segments = segments.size();
        out.cache_reused = cache_reused;

        bool first_dt = true;
        double dt_sum_s = 0.0;
        for (const orbitsim::TrajectorySegment &segment : segments)
        {
            if (!(segment.dt_s > 0.0) || !std::isfinite(segment.dt_s))
            {
                continue;
            }

            if ((segment.flags & (orbitsim::kTrajectorySegmentFlagForcedBoundary |
                                  orbitsim::kTrajectorySegmentFlagImpulseBoundary |
                                  orbitsim::kTrajectorySegmentFlagBurnBoundary)) != 0u)
            {
                ++out.forced_boundary_splits;
            }
            if ((segment.flags & orbitsim::kTrajectorySegmentFlagFrameResegmented) != 0u)
            {
                ++out.frame_resegmentation_count;
            }

            if (first_dt)
            {
                out.min_dt_s = segment.dt_s;
                out.max_dt_s = segment.dt_s;
                first_dt = false;
            }
            else
            {
                out.min_dt_s = std::min(out.min_dt_s, segment.dt_s);
                out.max_dt_s = std::max(out.max_dt_s, segment.dt_s);
            }
            dt_sum_s += segment.dt_s;
        }

        if (out.accepted_segments > 0 && dt_sum_s > 0.0)
        {
            out.avg_dt_s = dt_sum_s / static_cast<double>(out.accepted_segments);
        }

        return out;
    }

    inline OrbitPredictionService::AdaptiveStageDiagnostics make_stage_diagnostics_from_ephemeris(
            const OrbitPredictionService::SharedCelestialEphemeris &ephemeris,
            const double requested_duration_s,
            const bool cache_reused = false)
    {
        OrbitPredictionService::AdaptiveStageDiagnostics out{};
        out.requested_duration_s = std::max(0.0, requested_duration_s);
        out.cache_reused = cache_reused;
        if (!ephemeris || ephemeris->empty())
        {
            return out;
        }

        out.covered_duration_s = std::max(0.0, ephemeris->t_end_s() - ephemeris->t0_s());
        out.accepted_segments = ephemeris->segments.size();

        bool first_dt = true;
        double dt_sum_s = 0.0;
        for (const orbitsim::CelestialEphemerisSegment &segment : ephemeris->segments)
        {
            if (!(segment.dt_s > 0.0) || !std::isfinite(segment.dt_s))
            {
                continue;
            }

            if (first_dt)
            {
                out.min_dt_s = segment.dt_s;
                out.max_dt_s = segment.dt_s;
                first_dt = false;
            }
            else
            {
                out.min_dt_s = std::min(out.min_dt_s, segment.dt_s);
                out.max_dt_s = std::max(out.max_dt_s, segment.dt_s);
            }
            dt_sum_s += segment.dt_s;
        }

        if (out.accepted_segments > 0 && dt_sum_s > 0.0)
        {
            out.avg_dt_s = dt_sum_s / static_cast<double>(out.accepted_segments);
        }

        return out;
    }

    // ── Structs ───────────────────────────────────────────────────────────────
    struct PlannedSegmentBoundaryState
    {
        double t_s{0.0};
        orbitsim::State state_before{};
        orbitsim::State state_after{};
        std::uint32_t flags{orbitsim::kTrajectorySegmentFlagImpulseBoundary};
    };

    struct CelestialPredictionSamplingSpec
    {
        bool valid{false};
        glm::dvec3 rel_pos_m{0.0};
        glm::dvec3 rel_vel_mps{0.0};
        double horizon_s{0.0};
    };

    // ── Trajectory helpers (orbit_prediction_service_trajectory.cpp) ──────────
    bool build_maneuver_preview(const orbitsim::State &ship_state,
                                const double node_time_s,
                                OrbitPredictionService::ManeuverNodePreview &out_preview);

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
                                                const orbitsim::State &state_after,
                                                std::uint32_t flags = orbitsim::kTrajectorySegmentFlagImpulseBoundary);

    std::vector<orbitsim::TrajectorySegment> split_trajectory_segments_at_known_boundaries(
            const std::vector<orbitsim::TrajectorySegment> &segments,
            const std::vector<PlannedSegmentBoundaryState> &boundaries);

    std::vector<orbitsim::TrajectorySegment> slice_trajectory_segments(
            const std::vector<orbitsim::TrajectorySegment> &segments,
            double t0_s,
            double t1_s);

    // ── Sampling helpers (orbit_prediction_service_sampling.cpp) ──────────────
    std::vector<orbitsim::TrajectorySample> resample_segments_uniform(
            const std::vector<orbitsim::TrajectorySegment> &segments,
            std::size_t sample_count);

    std::vector<orbitsim::TrajectorySample> resample_ephemeris_uniform(
            const orbitsim::CelestialEphemeris &ephemeris,
            orbitsim::BodyId body_id,
            double t0_s,
            double t1_s,
            std::size_t sample_count);

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
    void apply_lagrange_integrator_profile(orbitsim::GameSimulation::Config &sim_config);
    std::size_t count_future_maneuver_impulses(const OrbitPredictionService::Request &request);
    bool request_needs_control_sensitive_prediction(const OrbitPredictionService::Request &request);
    double resolve_prediction_integrator_max_step_s(const OrbitPredictionService::Request &request,
                                                    double resolved_horizon_s);
    void apply_prediction_integrator_profile(orbitsim::GameSimulation::Config &sim_config,
                                             const OrbitPredictionService::Request &request,
                                             double resolved_horizon_s);
    OrbitPredictionService::PredictionSolvePlan build_prediction_solve_plan(
            const OrbitPredictionService::Request &request);
    OrbitPredictionService::PredictionProfileId promote_prediction_profile(
            OrbitPredictionService::PredictionProfileId profile_id,
            uint32_t steps = 1u);
    OrbitPredictionService::ChunkActivityProbe classify_chunk_activity(
            const OrbitPredictionService::Request &request,
            const OrbitPredictionService::PredictionChunkPlan &chunk,
            const std::vector<orbitsim::TrajectorySegment> *baseline_segments = nullptr,
            const OrbitPredictionService::SharedCelestialEphemeris &shared_ephemeris = {});
    OrbitPredictionService::PredictionProfileDefinition resolve_prediction_profile_definition(
            const OrbitPredictionService::Request &request,
            const OrbitPredictionService::PredictionChunkPlan &chunk);
    std::size_t prediction_sample_budget_for_chunk(
            const OrbitPredictionService::Request &request,
            const OrbitPredictionService::PredictionChunkPlan &chunk,
            std::size_t segment_count);
    orbitsim::AdaptiveSegmentOptions build_spacecraft_adaptive_segment_options(
            const OrbitPredictionService::Request &request,
            const OrbitPredictionService::EphemerisSamplingSpec &sampling_spec,
            const CancelCheck &cancel_requested = {});
    orbitsim::AdaptiveSegmentOptions build_spacecraft_adaptive_segment_options_for_chunk(
            const OrbitPredictionService::Request &request,
            const OrbitPredictionService::PredictionChunkPlan &chunk,
            const CancelCheck &cancel_requested = {});
    orbitsim::AdaptiveEphemerisOptions build_adaptive_ephemeris_options(
            const OrbitPredictionService::Request &request,
            const OrbitPredictionService::EphemerisSamplingSpec &sampling_spec,
            const CancelCheck &cancel_requested = {});
    orbitsim::AdaptiveEphemerisOptions build_adaptive_ephemeris_options_for_chunk(
            const OrbitPredictionService::Request &request,
            const OrbitPredictionService::PredictionChunkPlan &chunk,
            const CancelCheck &cancel_requested = {});

    bool ephemeris_covers_horizon(const OrbitPredictionService::SharedCelestialEphemeris &ephemeris,
                                  double sim_time_s,
                                  double horizon_s);
    bool compatible_cached_ephemeris(const OrbitPredictionService::CachedEphemerisEntry &entry,
                                     const OrbitPredictionService::EphemerisBuildRequest &request);
    OrbitPredictionService::SharedCelestialEphemeris build_ephemeris_from_request(
            const OrbitPredictionService::EphemerisBuildRequest &request,
            const CancelCheck &cancel_requested = {},
            orbitsim::AdaptiveEphemerisDiagnostics *out_diag = nullptr);
    OrbitPredictionService::EphemerisBuildRequest build_ephemeris_build_request(
            const OrbitPredictionService::Request &request,
            const OrbitPredictionService::EphemerisSamplingSpec &sampling_spec);
    CelestialPredictionSamplingSpec build_celestial_prediction_sampling_spec(
            const OrbitPredictionService::Request &request,
            const orbitsim::MassiveBody &subject_body);

} // namespace Game
