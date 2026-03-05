#include "game/orbit/orbit_prediction_service.h"
#include "game/orbit/orbit_prediction_math.h"
#include "game/orbit/orbit_prediction_tuning.h"

#include "orbitsim/maneuvers_types.hpp"
#include "orbitsim/trajectories.hpp"
#include "orbitsim/trajectory_transforms.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace Game
{
    namespace
    {
        bool finite_vec3(const glm::dvec3 &v)
        {
            return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
        }

        orbitsim::TrajectorySegment to_body_centered_segment(const orbitsim::TrajectorySegment &segment,
                                                             const orbitsim::State &reference_start,
                                                             const orbitsim::State &reference_end)
        {
            orbitsim::TrajectorySegment out = segment;
            out.start.position_m = segment.start.position_m - reference_start.position_m;
            out.start.velocity_mps = segment.start.velocity_mps - reference_start.velocity_mps;
            out.end.position_m = segment.end.position_m - reference_end.position_m;
            out.end.velocity_mps = segment.end.velocity_mps - reference_end.velocity_mps;
            return out;
        }

        std::vector<orbitsim::TrajectorySegment> trajectory_segments_to_body_centered_inertial(
                const std::vector<orbitsim::TrajectorySegment> &segments_inertial,
                const orbitsim::CelestialEphemeris &ephemeris,
                const orbitsim::BodyId reference_body_id)
        {
            std::vector<orbitsim::TrajectorySegment> out;
            out.reserve(segments_inertial.size());

            for (const orbitsim::TrajectorySegment &segment : segments_inertial)
            {
                const double t1_s = segment.t0_s + segment.dt_s;
                const orbitsim::State reference_start = ephemeris.body_state_at_by_id(reference_body_id, segment.t0_s);
                const orbitsim::State reference_end = ephemeris.body_state_at_by_id(reference_body_id, t1_s);
                out.push_back(to_body_centered_segment(segment, reference_start, reference_end));
            }

            return out;
        }
    } // namespace

    OrbitPredictionService::OrbitPredictionService()
    {
        _worker = std::thread(&OrbitPredictionService::worker_loop, this);
    }

    OrbitPredictionService::~OrbitPredictionService()
    {
        {
            std::lock_guard<std::mutex> lock(_mutex);
            _running = false;
            _has_pending = false;
            _completed.reset();
        }
        _cv.notify_all();

        if (_worker.joinable())
        {
            _worker.join();
        }
    }

    void OrbitPredictionService::request(Request request)
    {
        {
            std::lock_guard<std::mutex> lock(_mutex);
            const uint64_t generation_id = _next_generation_id++;
            _latest_requested_generation_id = generation_id;
            _pending.generation_id = generation_id;
            _pending.request = std::move(request);
            _has_pending = true;
        }
        _cv.notify_one();
    }

    std::optional<OrbitPredictionService::Result> OrbitPredictionService::poll_completed()
    {
        std::lock_guard<std::mutex> lock(_mutex);
        if (!_completed.has_value())
        {
            return std::nullopt;
        }

        std::optional<Result> out = std::move(_completed);
        _completed.reset();
        return out;
    }

    void OrbitPredictionService::reset()
    {
        std::lock_guard<std::mutex> lock(_mutex);
        _has_pending = false;
        _completed.reset();
        _latest_requested_generation_id = _next_generation_id;
    }

    void OrbitPredictionService::worker_loop()
    {
        while (true)
        {
            PendingJob job{};
            {
                std::unique_lock<std::mutex> lock(_mutex);
                _cv.wait(lock, [this]() {
                    return !_running || _has_pending;
                });

                if (!_running && !_has_pending)
                {
                    return;
                }

                if (!_has_pending)
                {
                    continue;
                }

                job = std::move(_pending);
                _has_pending = false;
            }

            Result result = compute_prediction(job.generation_id, job.request);

            {
                std::lock_guard<std::mutex> lock(_mutex);
                if (!_running)
                {
                    return;
                }

                if (job.generation_id < _latest_requested_generation_id)
                {
                    continue;
                }

                _completed = std::move(result);
            }
        }
    }

    OrbitPredictionService::Result OrbitPredictionService::compute_prediction(const uint64_t generation_id,
                                                                              const Request &request)
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
        out.build_time_s = request.sim_time_s;

        if (!std::isfinite(request.sim_time_s))
        {
            return out;
        }

        if (request.reference_body_id == orbitsim::kInvalidBodyId ||
            !(request.reference_body_mass_kg > 0.0) ||
            !std::isfinite(request.reference_body_mass_kg))
        {
            return out;
        }

        const double mu_ref_m3_s2 = request.sim_config.gravitational_constant * request.reference_body_mass_kg;
        if (!(mu_ref_m3_s2 > 0.0) || !std::isfinite(mu_ref_m3_s2))
        {
            return out;
        }

        orbitsim::GameSimulation sim(request.sim_config);
        if (!sim.set_time_s(request.sim_time_s))
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

        const orbitsim::MassiveBody *ref_sim = sim.body_by_id(request.reference_body_id);
        if (!ref_sim)
        {
            return out;
        }

        const glm::dvec3 ship_bary_pos_m = request.ship_bary_position_m;
        const glm::dvec3 ship_bary_vel_mps = request.ship_bary_velocity_mps;
        const glm::dvec3 ship_rel_pos_m = ship_bary_pos_m - glm::dvec3(ref_sim->state.position_m);
        const glm::dvec3 ship_rel_vel_mps = ship_bary_vel_mps - glm::dvec3(ref_sim->state.velocity_mps);
        if (!finite_vec3(ship_rel_pos_m) || !finite_vec3(ship_rel_vel_mps))
        {
            return out;
        }

        out.ship_rel_position_m = ship_rel_pos_m;
        out.ship_rel_velocity_mps = ship_rel_vel_mps;

        const auto [horizon_s_auto, dt_s_auto] =
                OrbitPredictionMath::select_prediction_horizon_and_dt(mu_ref_m3_s2, ship_rel_pos_m, ship_rel_vel_mps);

        double horizon_s = std::clamp(horizon_s_auto, OrbitPredictionTuning::kMinHorizonS, OrbitPredictionTuning::kMaxHorizonS);
        double dt_s = std::clamp(dt_s_auto, 0.01, OrbitPredictionTuning::kMaxSampleDtS);
        int max_steps = OrbitPredictionTuning::kMaxStepsNormal;
        double dt_min_s = 0.01;
        double dt_max_s = OrbitPredictionTuning::kMaxSampleDtS;
        double min_horizon_s = horizon_s;

        if (!request.maneuver_impulses.empty() &&
            std::isfinite(request.max_maneuver_time_s) &&
            request.max_maneuver_time_s > request.sim_time_s)
        {
            const double extra_s = std::max(OrbitPredictionTuning::kPostNodeCoverageMinS,
                                            std::max(0.0, request.future_window_s));
            min_horizon_s = std::max(min_horizon_s, (request.max_maneuver_time_s - request.sim_time_s) + extra_s);
            horizon_s = std::max(horizon_s, min_horizon_s);
        }

        if (request.thrusting)
        {
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

        // Keep the full horizon visible without exceeding the sample budget.
        // If horizon/dt would produce too many samples, coarsen dt instead of truncating duration.
        const double min_dt_for_step_budget = horizon_s / static_cast<double>(std::max(1, max_steps));
        if (std::isfinite(min_dt_for_step_budget) && min_dt_for_step_budget > 0.0)
        {
            dt_s = std::max(dt_s, min_dt_for_step_budget);
        }
        dt_s = std::clamp(dt_s, dt_min_s, dt_max_s);
        horizon_s = std::clamp(horizon_s, dt_s, OrbitPredictionTuning::kMaxHorizonS);
        const int steps = std::clamp(static_cast<int>(std::ceil(horizon_s / dt_s)), 2, max_steps);

        orbitsim::Spacecraft ship_sc{};
        ship_sc.state = orbitsim::make_state(ship_bary_pos_m, ship_bary_vel_mps);
        ship_sc.dry_mass_kg = 1.0;

        const auto ship_h = sim.create_spacecraft(ship_sc);
        if (!ship_h.valid())
        {
            return out;
        }

        sim.maneuver_plan() = orbitsim::ManeuverPlan{};

        orbitsim::TrajectoryOptions opt{};
        opt.duration_s = horizon_s;
        opt.sample_dt_s = dt_s;
        opt.spacecraft_sample_dt_s = dt_s;
        opt.spacecraft_lookup_dt_s = dt_s;
        opt.celestial_dt_s = dt_s;
        opt.max_samples = static_cast<size_t>(steps) + 1;
        opt.include_start = true;
        opt.include_end = true;
        opt.stop_on_impact = false;

        orbitsim::TrajectorySegmentOptions segment_opt{};
        segment_opt.duration_s = horizon_s;
        segment_opt.max_segments = std::max<std::size_t>(1, (opt.max_samples > 0) ? (opt.max_samples - 1) : 0);
        segment_opt.include_start = true;
        segment_opt.include_end = true;
        segment_opt.stop_on_impact = false;

        const orbitsim::CelestialEphemeris eph = orbitsim::build_celestial_ephemeris(sim, opt);
        const std::vector<orbitsim::TrajectorySegment> traj_segments_inertial_baseline =
                orbitsim::predict_spacecraft_trajectory_segments(sim, eph, ship_h.id, segment_opt);
        if (traj_segments_inertial_baseline.empty())
        {
            return out;
        }

        out.trajectory_segments_bci =
                trajectory_segments_to_body_centered_inertial(traj_segments_inertial_baseline, eph, ref_sim->id);

        const std::vector<orbitsim::TrajectorySample> traj_inertial_baseline =
                orbitsim::sample_trajectory_segments_uniform_dt(traj_segments_inertial_baseline, opt);
        if (traj_inertial_baseline.size() < 2)
        {
            return out;
        }

        std::vector<orbitsim::TrajectorySample> traj_centered_baseline =
                orbitsim::trajectory_to_body_centered_inertial(traj_inertial_baseline, eph, *ref_sim);
        if (traj_centered_baseline.size() < 2)
        {
            return out;
        }

        out.trajectory_bci = std::move(traj_centered_baseline);
        out.altitude_km.reserve(out.trajectory_bci.size());
        out.speed_kmps.reserve(out.trajectory_bci.size());

        for (const orbitsim::TrajectorySample &sample : out.trajectory_bci)
        {
            const double r_m = OrbitPredictionMath::safe_length(sample.position_m);
            const double alt_km = (r_m - request.reference_body_radius_m) * 1.0e-3;
            const double spd_kmps = OrbitPredictionMath::safe_length(sample.velocity_mps) * 1.0e-3;
            out.altitude_km.push_back(static_cast<float>(alt_km));
            out.speed_kmps.push_back(static_cast<float>(spd_kmps));
        }

        const OrbitPredictionMath::OrbitalElementsEstimate elements =
                OrbitPredictionMath::compute_orbital_elements(mu_ref_m3_s2, ship_rel_pos_m, ship_rel_vel_mps);
        if (elements.valid)
        {
            out.semi_major_axis_m = elements.semi_major_axis_m;
            out.eccentricity = elements.eccentricity;
            out.orbital_period_s = elements.orbital_period_s;
            out.periapsis_alt_km = (elements.periapsis_m - request.reference_body_radius_m) * 1.0e-3;
            out.apoapsis_alt_km = std::isfinite(elements.apoapsis_m)
                                      ? (elements.apoapsis_m - request.reference_body_radius_m) * 1.0e-3
                                      : std::numeric_limits<double>::infinity();
        }

        if (!request.maneuver_impulses.empty())
        {
            orbitsim::ManeuverPlan plan{};
            plan.impulses.reserve(request.maneuver_impulses.size());

            for (const ManeuverImpulse &src : request.maneuver_impulses)
            {
                if (!std::isfinite(src.t_s) || !finite_vec3(src.dv_rtn_mps))
                {
                    continue;
                }

                orbitsim::ImpulseSegment impulse{};
                impulse.t_s = src.t_s;
                impulse.primary_body_id = src.primary_body_id;
                impulse.dv_rtn_mps = src.dv_rtn_mps;
                impulse.spacecraft_id = ship_h.id;
                plan.impulses.push_back(impulse);
            }

            orbitsim::sort_impulses_by_time(plan);
            sim.maneuver_plan() = std::move(plan);

            const std::vector<orbitsim::TrajectorySegment> traj_segments_inertial_planned =
                    orbitsim::predict_spacecraft_trajectory_segments(sim, eph, ship_h.id, segment_opt);
            if (!traj_segments_inertial_planned.empty())
            {
                out.trajectory_segments_bci_planned =
                        trajectory_segments_to_body_centered_inertial(traj_segments_inertial_planned, eph, ref_sim->id);

                const std::vector<orbitsim::TrajectorySample> traj_inertial_planned =
                        orbitsim::sample_trajectory_segments_uniform_dt(traj_segments_inertial_planned, opt);
                std::vector<orbitsim::TrajectorySample> traj_centered_planned =
                        orbitsim::trajectory_to_body_centered_inertial(traj_inertial_planned, eph, *ref_sim);
                if (traj_centered_planned.size() >= 2)
                {
                    out.trajectory_bci_planned = std::move(traj_centered_planned);
                }
            }
        }

        out.valid = true;
        return out;
    }
} // namespace Game
