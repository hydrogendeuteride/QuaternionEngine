#pragma once

#include "orbitsim/game_sim.hpp"
#include "orbitsim/trajectory_segments.hpp"
#include "orbitsim/trajectory_types.hpp"

#include <condition_variable>
#include <cstdint>
#include <limits>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

namespace Game
{
    class OrbitPredictionService
    {
    public:
        struct ManeuverNodePreview
        {
            int node_id{-1};
            double t_s{0.0};
            bool valid{false};
            orbitsim::Vec3 rel_position_m{0.0, 0.0, 0.0};
            orbitsim::Vec3 rel_velocity_mps{0.0, 0.0, 0.0};
        };

        struct ManeuverImpulse
        {
            int node_id{-1};
            double t_s{0.0};
            orbitsim::BodyId primary_body_id{orbitsim::kInvalidBodyId};
            // Gameplay UI authors node DV in a velocity-aligned maneuver frame.
            // The worker converts this into solver RTN using the prefix-planned pre-burn state.
            orbitsim::Vec3 dv_rtn_mps{0.0, 0.0, 0.0};
        };

        struct Request
        {
            double sim_time_s{0.0};
            orbitsim::GameSimulation::Config sim_config{};
            std::vector<orbitsim::MassiveBody> massive_bodies;

            orbitsim::BodyId reference_body_id{orbitsim::kInvalidBodyId};
            double reference_body_mass_kg{0.0};
            double reference_body_radius_m{0.0};

            orbitsim::Vec3 ship_bary_position_m{0.0, 0.0, 0.0};
            orbitsim::Vec3 ship_bary_velocity_mps{0.0, 0.0, 0.0};

            bool thrusting{false};
            double future_window_s{600.0};
            double max_maneuver_time_s{0.0};
            std::vector<ManeuverImpulse> maneuver_impulses;
        };

        struct Result
        {
            uint64_t generation_id{0};
            bool valid{false};
            double compute_time_ms{0.0};

            double build_time_s{0.0};
            orbitsim::Vec3 ship_rel_position_m{0.0, 0.0, 0.0};
            orbitsim::Vec3 ship_rel_velocity_mps{0.0, 0.0, 0.0};

            std::vector<orbitsim::TrajectorySample> trajectory_bci;
            std::vector<orbitsim::TrajectorySample> trajectory_bci_planned;
            std::vector<orbitsim::TrajectorySegment> trajectory_segments_bci;
            std::vector<orbitsim::TrajectorySegment> trajectory_segments_bci_planned;
            std::vector<ManeuverNodePreview> maneuver_previews;
            std::vector<float> altitude_km;
            std::vector<float> speed_kmps;

            double semi_major_axis_m{0.0};
            double eccentricity{0.0};
            double orbital_period_s{0.0};
            double periapsis_alt_km{0.0};
            double apoapsis_alt_km{std::numeric_limits<double>::infinity()};
        };

        OrbitPredictionService();
        ~OrbitPredictionService();

        OrbitPredictionService(const OrbitPredictionService &) = delete;
        OrbitPredictionService &operator=(const OrbitPredictionService &) = delete;

        void request(Request request);
        std::optional<Result> poll_completed();
        void reset();

    private:
        struct PendingJob
        {
            uint64_t generation_id{0};
            Request request{};
        };

        static Result compute_prediction(uint64_t generation_id, const Request &request);
        void worker_loop();

        std::thread _worker;
        std::mutex _mutex;
        std::condition_variable _cv;
        bool _running{true};

        bool _has_pending{false};
        PendingJob _pending{};
        std::optional<Result> _completed{};

        uint64_t _next_generation_id{1};
        uint64_t _latest_requested_generation_id{0};
    };
} // namespace Game
