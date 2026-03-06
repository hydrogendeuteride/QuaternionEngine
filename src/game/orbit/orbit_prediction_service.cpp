#include "game/orbit/orbit_prediction_service.h"
#include "game/orbit/orbit_prediction_math.h"
#include "game/orbit/orbit_prediction_tuning.h"

#include "orbitsim/coordinate_frames.hpp"
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

        double safe_length(const glm::dvec3 &v)
        {
            const double len2 = glm::dot(v, v);
            if (!std::isfinite(len2) || len2 <= 0.0)
            {
                return 0.0;
            }
            return std::sqrt(len2);
        }

        glm::dvec3 normalized_or(const glm::dvec3 &v, const glm::dvec3 &fallback)
        {
            const double len = safe_length(v);
            if (!(len > 0.0) || !std::isfinite(len))
            {
                return fallback;
            }
            return v / len;
        }

        orbitsim::Vec3 convert_dv_rtf_to_solver_rtn(const glm::dvec3 &dv_rtf_mps,
                                                    const glm::dvec3 &r_rel_m,
                                                    const glm::dvec3 &v_rel_mps)
        {
            const orbitsim::RtnFrame solver_frame = orbitsim::compute_rtn_frame(r_rel_m, v_rel_mps);

            const glm::dvec3 solver_r = glm::dvec3(solver_frame.R.x, solver_frame.R.y, solver_frame.R.z);
            const glm::dvec3 solver_t = glm::dvec3(solver_frame.T.x, solver_frame.T.y, solver_frame.T.z);
            const glm::dvec3 solver_n = glm::dvec3(solver_frame.N.x, solver_frame.N.y, solver_frame.N.z);

            const glm::dvec3 t_hat = normalized_or(v_rel_mps, solver_t);
            glm::dvec3 n_hat = normalized_or(glm::cross(r_rel_m, v_rel_mps), solver_n);
            glm::dvec3 r_hat = normalized_or(glm::cross(t_hat, n_hat), solver_r);

            const double plane_area = safe_length(glm::cross(r_rel_m, v_rel_mps));
            if (!finite_vec3(r_hat) || !finite_vec3(t_hat) || !finite_vec3(n_hat) || !(plane_area > 1.0e-8))
            {
                return orbitsim::Vec3{dv_rtf_mps.x, dv_rtf_mps.y, dv_rtf_mps.z};
            }

            if (glm::dot(r_hat, r_rel_m) < 0.0)
            {
                r_hat = -r_hat;
                n_hat = -n_hat;
            }

            n_hat = normalized_or(glm::cross(r_hat, t_hat), n_hat);
            r_hat = normalized_or(glm::cross(t_hat, n_hat), r_hat);

            const glm::dvec3 dv_world = r_hat * dv_rtf_mps.x + t_hat * dv_rtf_mps.y + n_hat * dv_rtf_mps.z;
            return orbitsim::Vec3{
                    glm::dot(dv_world, solver_r),
                    glm::dot(dv_world, solver_t),
                    glm::dot(dv_world, solver_n),
            };
        }

        bool build_maneuver_preview(orbitsim::GameSimulation &sim,
                                    const orbitsim::CelestialEphemeris &ephemeris,
                                    const orbitsim::SpacecraftId ship_id,
                                    const orbitsim::BodyId reference_body_id,
                                    const double sim_time_s,
                                    const double node_time_s,
                                    const double sample_dt_s,
                                    const orbitsim::ManeuverPlan &prefix_plan,
                                    OrbitPredictionService::ManeuverNodePreview &out_preview)
        {
            out_preview.valid = false;

            const orbitsim::Spacecraft *ship = sim.spacecraft_by_id(ship_id);
            if (!ship)
            {
                return false;
            }

            double preview_time_s = node_time_s;
            if (preview_time_s > sim_time_s)
            {
                preview_time_s = std::max(sim_time_s, preview_time_s - 1.0e-3);
            }

            if (!std::isfinite(preview_time_s))
            {
                return false;
            }

            preview_time_s = std::max(sim_time_s, preview_time_s);
            orbitsim::State ship_state = ship->state;

            if (preview_time_s > sim_time_s + 1.0e-9)
            {
                sim.maneuver_plan() = prefix_plan;

                orbitsim::TrajectorySegmentOptions preview_opt{};
                preview_opt.duration_s = preview_time_s - sim_time_s;
                preview_opt.max_segments = std::max<std::size_t>(
                        1,
                        static_cast<std::size_t>(std::ceil(preview_opt.duration_s / std::max(0.01, sample_dt_s))));
                preview_opt.include_start = true;
                preview_opt.include_end = true;
                preview_opt.stop_on_impact = false;
                preview_opt.lookup_dt_s = sample_dt_s;

                const std::vector<orbitsim::TrajectorySegment> preview_segments =
                        orbitsim::predict_spacecraft_trajectory_segments(sim, ephemeris, ship_id, preview_opt);
                if (preview_segments.empty())
                {
                    return false;
                }

                ship_state = preview_segments.back().end;
            }

            const orbitsim::State reference_state = ephemeris.body_state_at_by_id(reference_body_id, preview_time_s);
            const glm::dvec3 rel_position_m = ship_state.position_m - reference_state.position_m;
            const glm::dvec3 rel_velocity_mps = ship_state.velocity_mps - reference_state.velocity_mps;
            if (!finite_vec3(rel_position_m) || !finite_vec3(rel_velocity_mps))
            {
                return false;
            }

            out_preview.rel_position_m = rel_position_m;
            out_preview.rel_velocity_mps = rel_velocity_mps;
            out_preview.valid = true;
            return true;
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
            std::vector<ManeuverImpulse> maneuver_impulses = request.maneuver_impulses;
            std::stable_sort(maneuver_impulses.begin(),
                             maneuver_impulses.end(),
                             [](const ManeuverImpulse &a, const ManeuverImpulse &b) { return a.t_s < b.t_s; });

            orbitsim::ManeuverPlan plan{};
            plan.impulses.reserve(maneuver_impulses.size());
            out.maneuver_previews.reserve(maneuver_impulses.size());

            for (const ManeuverImpulse &src : maneuver_impulses)
            {
                if (!std::isfinite(src.t_s) || !finite_vec3(src.dv_rtn_mps))
                {
                    continue;
                }

                ManeuverNodePreview preview{};
                preview.node_id = src.node_id;
                preview.t_s = src.t_s;

                if (build_maneuver_preview(sim,
                                           eph,
                                           ship_h.id,
                                           ref_sim->id,
                                           request.sim_time_s,
                                           src.t_s,
                                           dt_s,
                                           plan,
                                           preview))
                {
                    out.maneuver_previews.push_back(preview);
                }

                orbitsim::Vec3 solver_dv_rtn = src.dv_rtn_mps;
                if (preview.valid)
                {
                    solver_dv_rtn =
                            convert_dv_rtf_to_solver_rtn(src.dv_rtn_mps, preview.rel_position_m, preview.rel_velocity_mps);
                }

                orbitsim::ImpulseSegment impulse{};
                impulse.t_s = src.t_s;
                impulse.primary_body_id = src.primary_body_id;
                impulse.dv_rtn_mps = solver_dv_rtn;
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
