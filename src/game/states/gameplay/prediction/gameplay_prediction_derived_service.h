#pragma once

#include "game/states/gameplay/prediction/gameplay_state_prediction_types.h"

#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace Game
{
    class OrbitPredictionDerivedService
    {
    public:
        struct Request
        {
            uint64_t track_id{0};
            uint64_t generation_id{0};
            OrbitPredictionService::Result solver_result{};
            bool reuse_existing_base_frame{false};
            OrbitPredictionService::AdaptiveStageDiagnostics reused_base_frame_diagnostics{};
            WorldVec3 build_pos_world{0.0, 0.0, 0.0};
            glm::dvec3 build_vel_world{0.0, 0.0, 0.0};
            orbitsim::GameSimulation::Config sim_config{};
            orbitsim::TrajectoryFrameSpec resolved_frame_spec{};
            uint64_t display_frame_key{0};
            uint64_t display_frame_revision{0};
            orbitsim::BodyId analysis_body_id{orbitsim::kInvalidBodyId};
            std::vector<orbitsim::TrajectorySegment> player_lookup_segments_inertial{};
        };

        struct Result
        {
            struct TimingStats
            {
                double total_ms{0.0};
                double frame_build_ms{0.0};
                double flatten_ms{0.0};
            };

            uint64_t track_id{0};
            uint64_t generation_id{0};
            uint64_t display_frame_key{0};
            uint64_t display_frame_revision{0};
            orbitsim::BodyId analysis_body_id{orbitsim::kInvalidBodyId};
            bool valid{false};
            bool base_frame_reused{false};
            OrbitPredictionService::SolveQuality solve_quality{OrbitPredictionService::SolveQuality::Full};
            OrbitPredictionService::PublishStage publish_stage{
                    OrbitPredictionService::PublishStage::PreviewFinalizing};
            TimingStats timings{};
            OrbitPredictionDerivedDiagnostics diagnostics{};
            OrbitPredictionCache cache{};
            PredictionChunkAssembly chunk_assembly{};
        };

        OrbitPredictionDerivedService();
        ~OrbitPredictionDerivedService();

        OrbitPredictionDerivedService(const OrbitPredictionDerivedService &) = delete;
        OrbitPredictionDerivedService &operator=(const OrbitPredictionDerivedService &) = delete;

        void request(Request request);
        std::optional<Result> poll_completed();
        void reset();

    private:
        struct PendingJob
        {
            uint64_t track_id{0};
            uint64_t request_epoch{0};
            uint64_t generation_id{0};
            Request request{};
        };

        Result build_cache(PendingJob job) const;
        static bool should_publish_result(uint64_t track_id,
                                          uint64_t generation_id,
                                          uint64_t request_epoch,
                                          uint64_t current_request_epoch,
                                          const std::unordered_map<uint64_t, uint64_t> &latest_requested_generation_by_track);
        bool should_continue_job(uint64_t track_id, uint64_t generation_id, uint64_t request_epoch) const;
        void worker_loop();

        std::vector<std::thread> _workers{};
        mutable std::mutex _mutex;
        std::condition_variable _cv;
        bool _running{true};

        std::deque<PendingJob> _pending_jobs{};
        std::deque<Result> _completed{};

        uint64_t _request_epoch{1};
        std::unordered_map<uint64_t, uint64_t> _latest_requested_generation_by_track{};
        std::unordered_set<uint64_t> _tracks_in_flight{};
    };
} // namespace Game
