#pragma once

/// Shared types, constants, and inline helpers for orbit_prediction_service split files.
/// Used by orbit_prediction_service.cpp, orbit_prediction_service_trajectory.cpp,
/// orbit_prediction_service_sampling.cpp, and orbit_prediction_service_policy.cpp.

#include "game/orbit/orbit_prediction_service.h"
#include "game/orbit/orbit_prediction_math.h"
#include "game/orbit/orbit_prediction_tuning.h"
#include "game/orbit/prediction/prediction_diagnostics_util.h"
#include "game/orbit/trajectory/trajectory_utils.h"

#include "orbitsim/coordinate_frames.hpp"
#include "orbitsim/game_sim.hpp"
#include "orbitsim/trajectory_segments.hpp"
#include "orbitsim/trajectory_types.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <vector>

namespace Game
{
    // ── Constants ──────────────────────────────────────────────────────────────
    constexpr double kEphemerisDurationEpsilonS = 1.0e-6;
    constexpr double kEphemerisDtEpsilonS = 1.0e-9;
    constexpr std::size_t kMaxCachedEphemerides = 64;

    using CancelCheck = std::function<bool()>;

    inline bool request_can_reuse_spacecraft_baseline(const OrbitPredictionService::Request &request)
    {
        return request.kind == OrbitPredictionService::RequestKind::Spacecraft &&
               !request.thrusting &&
               !request.maneuver_impulses.empty();
    }

    inline std::size_t prediction_sample_budget(const OrbitPredictionService::Request &request,
                                                const std::size_t segment_count)
    {
        (void) request;
        const std::size_t sample_multiplier = 2u;
        const std::size_t sample_cap = 4'000u;
        return std::clamp<std::size_t>(segment_count * sample_multiplier, 2u, sample_cap);
    }

    // ── Inline micro-helpers ──────────────────────────────────────────────────
    inline double request_end_time_s(const OrbitPredictionService::Request &request)
    {
        return request.sim_time_s + std::max(0.0, std::isfinite(request.future_window_s) ? request.future_window_s : 0.0);
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

    void append_or_merge_planned_boundary_state(std::vector<PlannedSegmentBoundaryState> &states,
                                                const double t_s,
                                                const orbitsim::State &state_before,
                                                const orbitsim::State &state_after,
                                                std::uint32_t flags = orbitsim::kTrajectorySegmentFlagImpulseBoundary);

    std::vector<orbitsim::TrajectorySegment> split_trajectory_segments_at_known_boundaries(
            const std::vector<orbitsim::TrajectorySegment> &segments,
            const std::vector<PlannedSegmentBoundaryState> &boundaries);

    // ── Sampling helpers (orbit_prediction_service_sampling.cpp) ──────────────
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

    using EphemerisResolverFn = std::function<OrbitPredictionService::SharedCelestialEphemeris(
            const OrbitPredictionService::EphemerisBuildRequest &,
            const CancelCheck &,
            OrbitPredictionService::AdaptiveStageDiagnostics *)>;
    using PublishFn = std::function<bool(OrbitPredictionService::Result)>;

    struct EphemerisResolveOutcome
    {
        OrbitPredictionService::Status status{OrbitPredictionService::Status::Success};
        OrbitPredictionService::SharedCelestialEphemeris ephemeris{};
        OrbitPredictionService::AdaptiveStageDiagnostics diagnostics{};
    };

    EphemerisResolveOutcome resolve_ephemeris_for_request(
            const OrbitPredictionService::Request &request,
            const OrbitPredictionService::EphemerisSamplingSpec &sampling_spec,
            double horizon_s,
            const CancelCheck &cancel_requested,
            const EphemerisResolverFn &resolve_ephemeris);

    OrbitPredictionService::Status solve_celestial_prediction_route(
            const OrbitPredictionService::Request &request,
            orbitsim::GameSimulation &sim,
            const CancelCheck &cancel_requested,
            const EphemerisResolverFn &resolve_ephemeris,
            OrbitPredictionService::Result &out);

    OrbitPredictionService::Status solve_spacecraft_baseline_trajectory(
            const OrbitPredictionService::Request &request,
            orbitsim::GameSimulation &sim,
            const orbitsim::CelestialEphemeris &ephemeris,
            orbitsim::GameSimulation::SpacecraftHandle ship_handle,
            const orbitsim::AdaptiveSegmentOptions &segment_options,
            double horizon_s,
            const CancelCheck &cancel_requested,
            OrbitPredictionService::Result &out);

    // ── Spacecraft route types (orbit_prediction_service_spacecraft_route.cpp)
    struct ReusableSpacecraftBaseline
    {
        std::vector<orbitsim::TrajectorySample> trajectory_inertial{};
        std::vector<orbitsim::TrajectorySegment> trajectory_segments_inertial{};
    };

    struct PlannedTrajectoryServices
    {
        virtual ~PlannedTrajectoryServices() = default;
        virtual std::optional<OrbitPredictionService::PlannedChunkCacheEntry> find_cached_chunk(
                const OrbitPredictionService::PlannedChunkCacheKey &key,
                const orbitsim::State &expected_start_state) = 0;
        virtual void store_cached_chunk(OrbitPredictionService::PlannedChunkCacheEntry entry) = 0;
        virtual OrbitPredictionService::SharedCelestialEphemeris get_or_build_ephemeris(
                const OrbitPredictionService::EphemerisBuildRequest &request,
                const CancelCheck &cancel_requested) = 0;
    };

    struct SpacecraftPredictionRouteServices : PlannedTrajectoryServices
    {
        virtual std::optional<ReusableSpacecraftBaseline> find_reusable_baseline(
                uint64_t track_id,
                uint64_t request_epoch) = 0;
        virtual void store_reusable_baseline(
                uint64_t track_id,
                uint64_t generation_id,
                uint64_t request_epoch,
                OrbitPredictionService::SharedCelestialEphemeris shared_ephemeris,
                std::vector<orbitsim::TrajectorySample> trajectory_inertial,
                std::vector<orbitsim::TrajectorySegment> trajectory_segments_inertial) = 0;
    };

    struct SpacecraftPredictionRouteEnvironment
    {
        const OrbitPredictionService::Request &request;
        uint64_t generation_id{0};
        uint64_t request_epoch{0};
        orbitsim::GameSimulation &sim;
        const OrbitPredictionService::EphemerisSamplingSpec &sampling_spec;
        const CancelCheck &cancel_requested;
        const EphemerisResolverFn &resolve_ephemeris;
        OrbitPredictionService::Result &out;
        PublishFn publish;
        std::chrono::steady_clock::time_point compute_start;
        SpacecraftPredictionRouteServices &services;
    };

    struct SpacecraftPredictionRouteOutcome
    {
        OrbitPredictionService::Status status{OrbitPredictionService::Status::Success};
        bool published_staged_preview{false};
    };

    SpacecraftPredictionRouteOutcome solve_spacecraft_prediction_route(
            const SpacecraftPredictionRouteEnvironment &env);

    // ── Planned trajectory types (orbit_prediction_service_planned.cpp) ────

    struct PlannedPredictionRouteEnvironment
    {
        const OrbitPredictionService::Request &request;
        const CancelCheck &cancel_requested;
        OrbitPredictionService::Result &out;
        const orbitsim::CelestialEphemeris &ephemeris;
        const orbitsim::State &ship_state;
        PublishFn publish;
        std::chrono::steady_clock::time_point compute_start;
        PlannedTrajectoryServices &services;
    };

    struct PlannedPredictionRouteOutcome
    {
        OrbitPredictionService::Status status{OrbitPredictionService::Status::Success};
        bool published_staged_preview{false};
    };

    struct PlannedTrajectoryContext
    {
        const OrbitPredictionService::Request &request;
        const CancelCheck &cancel_requested;
        OrbitPredictionService::Result &out;
        const orbitsim::CelestialEphemeris &ephemeris;
        PublishFn publish;
        PlannedTrajectoryServices &services;
    };

    struct PlannedChunkPacket
    {
        OrbitPredictionService::PredictionChunkPlan chunk{};
        std::vector<orbitsim::TrajectorySegment> segments{};
        std::vector<orbitsim::TrajectorySample> samples{};
        std::vector<OrbitPredictionService::ManeuverNodePreview> previews{};
        orbitsim::State start_state{};
        orbitsim::State end_state{};
        OrbitPredictionService::AdaptiveStageDiagnostics diagnostics{};
        bool reused_from_cache{false};
    };

    struct PlannedSolveRangeSummary
    {
        orbitsim::State end_state{};
        OrbitPredictionService::AdaptiveStageDiagnostics diagnostics{};
        OrbitPredictionService::Status status{OrbitPredictionService::Status::Success};
    };

    struct PlannedSolveOutput
    {
        std::vector<orbitsim::TrajectorySegment> segments{};
        std::vector<orbitsim::TrajectorySample> samples{};
        std::vector<OrbitPredictionService::ManeuverNodePreview> previews{};
        std::vector<bool> chunk_reused{};
        orbitsim::State end_state{};
        OrbitPredictionService::AdaptiveStageDiagnostics diagnostics{};
        OrbitPredictionService::Status status{OrbitPredictionService::Status::Success};
    };

    struct StagedCoreDataBuilder
    {
        const OrbitPredictionService::Result &source;
        std::shared_ptr<const OrbitPredictionService::Result::CoreData> staged_core_data{};

        std::shared_ptr<const OrbitPredictionService::Result::CoreData> get();
    };

    struct FullStreamBatchPublisher
    {
        using MakeStageResultFn = std::function<OrbitPredictionService::Result(
                const PlannedSolveOutput &,
                const std::vector<OrbitPredictionService::PublishedChunk> &,
                OrbitPredictionService::PublishStage,
                std::vector<OrbitPredictionService::StreamedPlannedChunk>,
                bool)>;

        bool active{false};
        double min_publish_interval_s{OrbitPredictionTuning::kFullStreamPublishMinIntervalS};
        std::chrono::steady_clock::time_point last_publish_tp{};
        bool published_any_batch{false};
        std::vector<OrbitPredictionService::PublishedChunk> pending_published_chunks{};
        std::vector<OrbitPredictionService::StreamedPlannedChunk> pending_streamed_chunks{};

        void append(const OrbitPredictionService::PublishedChunk &published_chunk,
                    const PlannedChunkPacket &packet);
        bool flush(const PlannedSolveOutput &stage_output,
                   const MakeStageResultFn &make_stage_result,
                   const PublishFn &publish,
                   bool force_publish);
    };

    struct ChunkAttemptOutput
    {
        std::vector<orbitsim::TrajectorySegment> segments{};
        std::vector<orbitsim::TrajectorySegment> seam_validation_segments{};
        std::vector<orbitsim::TrajectorySample> samples{};
        std::vector<OrbitPredictionService::ManeuverNodePreview> previews{};
        orbitsim::State end_state{};
        OrbitPredictionService::AdaptiveStageDiagnostics diagnostics{};
        OrbitPredictionService::Status status{OrbitPredictionService::Status::Success};
        bool reused_from_cache{false};
    };

    // ── Planned trajectory helpers (orbit_prediction_service_planned.cpp) ────
    void append_planned_chunk_packet(PlannedSolveOutput &planned, PlannedChunkPacket packet);
    void apply_planned_range_summary(PlannedSolveOutput &planned, const PlannedSolveRangeSummary &summary);

    OrbitPredictionService::PublishedChunk make_published_chunk(
            const PlannedChunkPacket &packet,
            uint32_t chunk_id,
            OrbitPredictionService::ChunkQualityState quality_state);
    void append_published_chunk(std::vector<OrbitPredictionService::PublishedChunk> &dst,
                                const PlannedChunkPacket &packet,
                                uint32_t chunk_id,
                                OrbitPredictionService::ChunkQualityState quality_state);
    void append_published_chunks_as_final(
            std::vector<OrbitPredictionService::PublishedChunk> &dst,
            const std::vector<OrbitPredictionService::PublishedChunk> &src);
    OrbitPredictionService::PublishedChunk make_published_chunk_from_plan(
            const OrbitPredictionService::PredictionChunkPlan &chunk,
            uint32_t chunk_id,
            bool reused_from_cache);
    OrbitPredictionService::Result make_planned_stage_result(
            const OrbitPredictionService::Result &base_result,
            const PlannedSolveOutput &stage_output,
            const std::vector<OrbitPredictionService::PublishedChunk> &published_chunks,
            OrbitPredictionService::PublishStage publish_stage,
            std::shared_ptr<const OrbitPredictionService::Result::CoreData> shared_core_data,
            std::vector<OrbitPredictionService::StreamedPlannedChunk> streamed_planned_chunks = {},
            bool include_cumulative_planned = true);
    bool build_cached_prefix_stream_chunks(
            const OrbitPredictionService::Request &request,
            const OrbitPredictionService::PredictionSolvePlan &solve_plan,
            std::size_t suffix_begin_index,
            const PlannedSolveOutput &prefix_output,
            std::vector<OrbitPredictionService::PublishedChunk> &out_published_chunks,
            std::vector<OrbitPredictionService::StreamedPlannedChunk> &out_streamed_chunks);
    FullStreamBatchPublisher make_full_stream_batch_publisher(
            const OrbitPredictionService::Request &request,
            std::chrono::steady_clock::time_point compute_start);

    PlannedSolveRangeSummary solve_planned_chunk_range(
            PlannedTrajectoryContext &ctx,
            const OrbitPredictionService::PredictionSolvePlan &solve_plan,
            std::size_t chunk_begin_index,
            std::size_t chunk_end_index,
            const orbitsim::State &range_start_state,
            std::function<bool(PlannedChunkPacket &&)> chunk_sink);

    PlannedPredictionRouteOutcome solve_planned_prediction_route(
            const PlannedPredictionRouteEnvironment &env);

} // namespace Game
