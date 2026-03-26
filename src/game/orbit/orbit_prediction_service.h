#pragma once

#include "orbitsim/game_sim.hpp"
#include "orbitsim/trajectory_segments.hpp"
#include "orbitsim/trajectory_types.hpp"

#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <unordered_map>
#include <vector>

namespace Game
{
    class OrbitPredictionService
    {
    public:
        using SharedCelestialEphemeris = std::shared_ptr<const orbitsim::CelestialEphemeris>;

        enum class RequestKind : uint8_t
        {
            Spacecraft = 0,
            Celestial,
        };

        enum class SolveQuality : uint8_t
        {
            Full = 0,
            FastPreview,
        };

        enum class RequestPriority : uint8_t
        {
            BackgroundCelestial = 0,
            BackgroundOrbiter,
            Overlay,
            ActiveTrack,
            ActiveInteractiveTrack,
        };

        enum class Status : uint8_t
        {
            None = 0,
            Success,
            InvalidInput,
            InvalidSubject,
            InvalidSamplingSpec,
            EphemerisUnavailable,
            TrajectorySegmentsUnavailable,
            TrajectorySamplesUnavailable,
            ContinuityFailed,
            Cancelled,
        };

        enum class PublishStage : uint8_t
        {
            Full = 0,
            FastPreviewFP0,
            FastPreviewFP1,
        };

        enum class ChunkQualityState : uint8_t
        {
            Final = 0,
            PreviewPatch,
        };

        struct AdaptiveStageDiagnostics
        {
            double requested_duration_s{0.0};
            double covered_duration_s{0.0};
            std::size_t accepted_segments{0};
            std::size_t rejected_splits{0};
            std::size_t forced_boundary_splits{0};
            std::size_t frame_resegmentation_count{0};
            double min_dt_s{0.0};
            double max_dt_s{0.0};
            double avg_dt_s{0.0};
            bool hard_cap_hit{false};
            bool cancelled{false};
            bool cache_reused{false};
        };

        struct Diagnostics
        {
            Status status{Status::None};
            bool cancelled{false};
            std::size_t ephemeris_segment_count{0};
            std::size_t trajectory_segment_count{0};
            std::size_t trajectory_segment_count_planned{0};
            std::size_t trajectory_sample_count{0};
            std::size_t trajectory_sample_count_planned{0};
            AdaptiveStageDiagnostics ephemeris{};
            AdaptiveStageDiagnostics trajectory_base{};
            AdaptiveStageDiagnostics trajectory_planned{};
        };

        struct ManeuverNodePreview
        {
            int node_id{-1};
            double t_s{0.0};
            bool valid{false};
            orbitsim::Vec3 inertial_position_m{0.0, 0.0, 0.0};
            orbitsim::Vec3 inertial_velocity_mps{0.0, 0.0, 0.0};
        };

        struct ManeuverImpulse
        {
            int node_id{-1};
            double t_s{0.0};
            orbitsim::BodyId primary_body_id{orbitsim::kInvalidBodyId};
            // Gameplay UI authors node DV directly in true RTN using the pre-burn primary-relative state.
            orbitsim::Vec3 dv_rtn_mps{0.0, 0.0, 0.0};
        };

        struct PreviewPatchRequest
        {
            bool active{false};
            bool anchor_state_valid{false};
            uint64_t baseline_generation_id{0};
            double anchor_time_s{std::numeric_limits<double>::quiet_NaN()};
            double visual_window_s{0.0};
            double exact_window_s{0.0};
            double pick_window_s{0.0};
            orbitsim::State anchor_state_inertial{};
        };

        struct PublishedChunk
        {
            uint32_t chunk_id{0};
            ChunkQualityState quality_state{ChunkQualityState::Final};
            double t0_s{std::numeric_limits<double>::quiet_NaN()};
            double t1_s{std::numeric_limits<double>::quiet_NaN()};
            bool includes_planned_path{false};
            bool reused_from_cache{false};
        };

        enum class PredictionProfileId : uint8_t
        {
            InteractiveExact = 0,
            NearBody,
            Transfer,
            Cruise,
            DeepTail,
        };

        enum class PredictionChunkBoundaryFlags : uint32_t
        {
            None = 0u,
            RequestStart = 1u << 0u,
            RequestEnd = 1u << 1u,
            Maneuver = 1u << 2u,
            PreviewAnchor = 1u << 3u,
            PreviewChunk = 1u << 4u,
            KnownDiscontinuity = 1u << 5u,
            TimeBand = 1u << 6u,
        };

        struct PredictionProfileDefinition
        {
            PredictionProfileId profile_id{PredictionProfileId::NearBody};
            double integrator_tolerance_multiplier{1.0};
            double min_dt_s{0.0};
            double max_dt_s{0.0};
            double lookup_max_dt_s{0.0};
            std::size_t soft_max_segments{0};
            double ephemeris_min_dt_s{0.0};
            double ephemeris_max_dt_s{0.0};
            std::size_t ephemeris_soft_max_segments{0};
            double output_sample_density_scale{1.0};
            double seam_overlap_s{0.0};
        };

        struct PredictionChunkPlan
        {
            uint32_t chunk_id{0};
            double t0_s{std::numeric_limits<double>::quiet_NaN()};
            double t1_s{std::numeric_limits<double>::quiet_NaN()};
            PredictionProfileId profile_id{PredictionProfileId::NearBody};
            uint32_t boundary_flags{0u};
            uint32_t priority{0u};
            bool allow_reuse{true};
            bool requires_seam_validation{false};
        };

        struct PredictionSolvePlan
        {
            bool valid{false};
            double t0_s{std::numeric_limits<double>::quiet_NaN()};
            double t1_s{std::numeric_limits<double>::quiet_NaN()};
            std::vector<PredictionChunkPlan> chunks{};
        };

        struct ChunkActivityProbe
        {
            bool valid{false};
            double heading_change_rad{0.0};
            double accel_magnitude_mps2{0.0};
            double jerk_magnitude_mps3{0.0};
            double dominant_gravity_ratio{1.0};
            double maneuver_proximity_s{std::numeric_limits<double>::infinity()};
            orbitsim::BodyId primary_body_id_start{orbitsim::kInvalidBodyId};
            orbitsim::BodyId primary_body_id_mid{orbitsim::kInvalidBodyId};
            orbitsim::BodyId primary_body_id_end{orbitsim::kInvalidBodyId};
            PredictionProfileId recommended_profile_id{PredictionProfileId::NearBody};
            bool should_split{false};
        };

        struct ChunkSeamDiagnostics
        {
            bool valid{false};
            bool success{false};
            double sample_time_s{std::numeric_limits<double>::quiet_NaN()};
            double time_error_s{0.0};
            double position_error_m{0.0};
            double velocity_error_mps{0.0};
            orbitsim::BodyId previous_primary_body_id{orbitsim::kInvalidBodyId};
            orbitsim::BodyId current_primary_body_id{orbitsim::kInvalidBodyId};
            bool primary_flutter{false};
            uint32_t retry_count{0};
        };

        struct Request
        {
            // The worker handles both spacecraft and celestial prediction jobs.
            RequestKind kind{RequestKind::Spacecraft};
            uint64_t track_id{0};
            double sim_time_s{0.0};
            orbitsim::GameSimulation::Config sim_config{};
            std::vector<orbitsim::MassiveBody> massive_bodies;
            SharedCelestialEphemeris shared_ephemeris{};

            // Celestial jobs identify the predicted body directly.
            orbitsim::BodyId subject_body_id{orbitsim::kInvalidBodyId};

            orbitsim::Vec3 ship_bary_position_m{0.0, 0.0, 0.0};
            orbitsim::Vec3 ship_bary_velocity_mps{0.0, 0.0, 0.0};

            bool thrusting{false};
            bool lagrange_sensitive{false};
            SolveQuality solve_quality{SolveQuality::Full};
            RequestPriority priority{RequestPriority::BackgroundOrbiter};
            double future_window_s{600.0};
            double celestial_ephemeris_dt_s{0.0};
            orbitsim::BodyId preferred_primary_body_id{orbitsim::kInvalidBodyId};
            PreviewPatchRequest preview_patch{};
            std::vector<ManeuverImpulse> maneuver_impulses;
        };

        struct Result
        {
            uint64_t track_id{0};
            uint64_t generation_id{0};
            bool valid{false};
            bool baseline_reused{false};
            bool generation_complete{true};
            SolveQuality solve_quality{SolveQuality::Full};
            PublishStage publish_stage{PublishStage::Full};
            double compute_time_ms{0.0};
            Diagnostics diagnostics{};
            std::vector<PublishedChunk> published_chunks{};

            double build_time_s{0.0};
            SharedCelestialEphemeris shared_ephemeris{};
            std::vector<orbitsim::MassiveBody> massive_bodies{};

            std::vector<orbitsim::TrajectorySample> trajectory_inertial;
            std::vector<orbitsim::TrajectorySample> trajectory_inertial_planned;
            std::vector<orbitsim::TrajectorySegment> trajectory_segments_inertial;
            std::vector<orbitsim::TrajectorySegment> trajectory_segments_inertial_planned;
            std::vector<ManeuverNodePreview> maneuver_previews;
        };

        struct EphemerisSamplingSpec
        {
            bool valid{false};
            double horizon_s{0.0};
        };

        struct EphemerisBuildRequest
        {
            double sim_time_s{0.0};
            orbitsim::GameSimulation::Config sim_config{};
            std::vector<orbitsim::MassiveBody> massive_bodies;
            double duration_s{0.0};
            orbitsim::AdaptiveEphemerisOptions adaptive_options{};
        };

        struct CachedEphemerisEntry
        {
            double sim_time_s{0.0};
            double duration_s{0.0};
            orbitsim::GameSimulation::Config sim_config{};
            std::vector<orbitsim::MassiveBody> massive_bodies{};
            orbitsim::AdaptiveEphemerisOptions adaptive_options{};
            SharedCelestialEphemeris ephemeris{};
            AdaptiveStageDiagnostics diagnostics{};
            uint64_t last_use_serial{0};
        };

        OrbitPredictionService();
        ~OrbitPredictionService();

        OrbitPredictionService(const OrbitPredictionService &) = delete;
        OrbitPredictionService &operator=(const OrbitPredictionService &) = delete;

        // Queue or replace the latest prediction job for a track; work runs on the background thread.
        // Returns the assigned generation so runtime can track request ordering.
        uint64_t request(Request request);
        // Non-blocking poll for the next completed prediction result.
        std::optional<Result> poll_completed();
        // Invalidate queued/in-flight work and clear cached ephemerides.
        void reset();
        // Reuse a compatible cached ephemeris when possible, otherwise build and cache one.
        SharedCelestialEphemeris get_or_build_ephemeris(const EphemerisBuildRequest &request);
        // Derive spacecraft prediction horizon and cadence from the current orbital state.
        static EphemerisSamplingSpec build_ephemeris_sampling_spec(const Request &request);

    private:
        struct ReusableBaselineCacheEntry
        {
            uint64_t generation_id{0};
            uint64_t request_epoch{0};
            SharedCelestialEphemeris shared_ephemeris{};
            std::vector<orbitsim::TrajectorySample> trajectory_inertial{};
            std::vector<orbitsim::TrajectorySegment> trajectory_segments_inertial{};
        };

        struct PlannedChunkCacheKey
        {
            uint64_t track_id{0};
            uint64_t baseline_generation_id{0};
            uint64_t upstream_maneuver_hash{0};
            uint64_t frame_independent_generation{0};
            double chunk_t0_s{std::numeric_limits<double>::quiet_NaN()};
            double chunk_t1_s{std::numeric_limits<double>::quiet_NaN()};
            PredictionProfileId profile_id{PredictionProfileId::NearBody};
        };

        struct PlannedChunkCacheEntry
        {
            PlannedChunkCacheKey key{};
            orbitsim::State start_state{};
            orbitsim::State end_state{};
            AdaptiveStageDiagnostics diagnostics{};
            std::vector<orbitsim::TrajectorySegment> segments{};
            std::vector<orbitsim::TrajectorySegment> seam_validation_segments{};
            std::vector<orbitsim::TrajectorySample> samples{};
            std::vector<ManeuverNodePreview> previews{};
            uint64_t last_use_serial{0};
        };

        struct PendingJob
        {
            uint64_t track_id{0};
            uint64_t request_epoch{0};
            uint64_t generation_id{0};
            Request request{};
        };

        // Execute a single queued prediction request on the worker and publish zero or more staged results.
        void compute_prediction(const PendingJob &job);
        std::optional<ReusableBaselineCacheEntry> find_reusable_baseline(uint64_t track_id, uint64_t request_epoch) const;
        void store_reusable_baseline(uint64_t track_id,
                                     uint64_t generation_id,
                                     uint64_t request_epoch,
                                     SharedCelestialEphemeris shared_ephemeris,
                                     std::vector<orbitsim::TrajectorySample> trajectory_inertial,
                                     std::vector<orbitsim::TrajectorySegment> trajectory_segments_inertial);
        bool publish_completed_result(const PendingJob &job, Result result);
        // Drop stale results after reset() or when a newer request supersedes the same track.
        static bool should_publish_result(const PendingJob &job,
                                          uint64_t current_request_epoch,
                                          const std::unordered_map<uint64_t, uint64_t> &latest_requested_generation_by_track);
        bool should_continue_job(uint64_t track_id, uint64_t generation_id, uint64_t request_epoch) const;
        // Background loop that consumes queued jobs and publishes fresh results.
        void worker_loop();
        SharedCelestialEphemeris get_or_build_ephemeris(const EphemerisBuildRequest &request,
                                                        const std::function<bool()> &cancel_requested,
                                                        AdaptiveStageDiagnostics *out_diagnostics = nullptr,
                                                        bool *out_cache_reused = nullptr);

        std::vector<std::thread> _workers;
        mutable std::mutex _mutex;
        std::condition_variable _cv;
        bool _running{true};

        std::deque<PendingJob> _pending_jobs{};
        std::deque<Result> _completed{};

        uint64_t _request_epoch{1};
        uint64_t _next_generation_id{1};
        std::unordered_map<uint64_t, uint64_t> _latest_requested_generation_by_track{};

        mutable std::mutex _baseline_cache_mutex;
        std::unordered_map<uint64_t, ReusableBaselineCacheEntry> _reusable_baseline_by_track{};

        mutable std::mutex _planned_chunk_cache_mutex;
        std::vector<PlannedChunkCacheEntry> _planned_chunk_cache{};
        uint64_t _next_planned_chunk_cache_use_serial{1};

        std::mutex _ephemeris_mutex;
        std::vector<CachedEphemerisEntry> _ephemeris_cache{};
        uint64_t _next_ephemeris_use_serial{1};
    };
} // namespace Game
