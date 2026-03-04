#include "game/orbit/orbit_prediction_service.h"
#include "game/orbit/orbit_prediction_tuning.h"

#include "orbitsim/maneuvers_types.hpp"
#include "orbitsim/trajectories.hpp"
#include "orbitsim/trajectory_transforms.hpp"

#include <algorithm>
#include <cmath>

namespace Game
{
    namespace
    {
        constexpr double kPi = 3.14159265358979323846;

        double safe_length(const glm::dvec3 &v)
        {
            const double len2 = glm::dot(v, v);
            if (!std::isfinite(len2) || len2 <= 0.0)
            {
                return 0.0;
            }
            return std::sqrt(len2);
        }

        bool finite_vec3(const glm::dvec3 &v)
        {
            return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
        }

        double estimate_orbital_period_s(const double mu_m3_s2, const glm::dvec3 &r_m, const glm::dvec3 &v_mps)
        {
            const double r = safe_length(r_m);
            const double v = safe_length(v_mps);
            if (!(mu_m3_s2 > 0.0) || !(r > 0.0) || !std::isfinite(mu_m3_s2) || !std::isfinite(r) || !std::isfinite(v))
            {
                return OrbitPredictionTuning::kEscapeDefaultPeriodS;
            }

            const double specific_energy = 0.5 * (v * v) - mu_m3_s2 / r;
            if (!std::isfinite(specific_energy) || specific_energy >= 0.0)
            {
                return OrbitPredictionTuning::kEscapeDefaultPeriodS;
            }

            const double a_m = -mu_m3_s2 / (2.0 * specific_energy);
            if (!(a_m > 0.0) || !std::isfinite(a_m))
            {
                return OrbitPredictionTuning::kEscapeDefaultPeriodS;
            }

            const double period_s = 2.0 * kPi * std::sqrt((a_m * a_m * a_m) / mu_m3_s2);
            if (!std::isfinite(period_s) || period_s <= 0.0)
            {
                return OrbitPredictionTuning::kEscapeDefaultPeriodS;
            }
            return period_s;
        }

        struct OrbitalElementsEstimate
        {
            bool valid{false};
            double semi_major_axis_m{0.0};
            double eccentricity{0.0};
            double orbital_period_s{0.0};
            double periapsis_m{0.0};
            double apoapsis_m{std::numeric_limits<double>::infinity()};
        };

        OrbitalElementsEstimate compute_orbital_elements(const double mu_m3_s2, const glm::dvec3 &r_m,
                                                         const glm::dvec3 &v_mps)
        {
            OrbitalElementsEstimate out{};
            if (!(mu_m3_s2 > 0.0) || !std::isfinite(mu_m3_s2))
            {
                return out;
            }

            const double r = safe_length(r_m);
            const double v2 = glm::dot(v_mps, v_mps);
            if (!(r > 0.0) || !std::isfinite(r) || !std::isfinite(v2))
            {
                return out;
            }

            const glm::dvec3 h = glm::cross(r_m, v_mps);
            const double h2 = glm::dot(h, h);

            const glm::dvec3 e_vec = (glm::cross(v_mps, h) / mu_m3_s2) - (r_m / r);
            const double e = safe_length(e_vec);
            if (!std::isfinite(e))
            {
                return out;
            }

            const double specific_energy = 0.5 * v2 - mu_m3_s2 / r;
            if (!std::isfinite(specific_energy))
            {
                return out;
            }

            out.eccentricity = std::max(0.0, e);

            if (std::abs(specific_energy) > 1e-12)
            {
                out.semi_major_axis_m = -mu_m3_s2 / (2.0 * specific_energy);
            }

            if (out.semi_major_axis_m > 0.0 && std::isfinite(out.semi_major_axis_m) && out.eccentricity < 1.0)
            {
                out.orbital_period_s = 2.0 * kPi *
                                       std::sqrt((out.semi_major_axis_m * out.semi_major_axis_m *
                                                  out.semi_major_axis_m) /
                                                 mu_m3_s2);
                out.periapsis_m = out.semi_major_axis_m * (1.0 - out.eccentricity);
                out.apoapsis_m = out.semi_major_axis_m * (1.0 + out.eccentricity);
            }
            else if (h2 > 0.0 && std::isfinite(h2))
            {
                const double denom = mu_m3_s2 * (1.0 + out.eccentricity);
                if (denom > 0.0 && std::isfinite(denom))
                {
                    out.periapsis_m = h2 / denom;
                }
                out.orbital_period_s = 0.0;
                out.apoapsis_m = std::numeric_limits<double>::infinity();
            }

            if (!std::isfinite(out.periapsis_m) || out.periapsis_m <= 0.0)
            {
                out.periapsis_m = r;
            }

            out.valid = true;
            return out;
        }

        std::pair<double, double> select_prediction_horizon_and_dt(const double mu_m3_s2, const glm::dvec3 &r_m,
                                                                    const glm::dvec3 &v_mps)
        {
            const double period_s = estimate_orbital_period_s(mu_m3_s2, r_m, v_mps);
            const double horizon_s = std::clamp(period_s * OrbitPredictionTuning::kBaseHorizonFromPeriodScale,
                                                OrbitPredictionTuning::kMinHorizonS,
                                                OrbitPredictionTuning::kMaxHorizonS);
            const double target_samples = std::clamp(horizon_s / OrbitPredictionTuning::kTargetSamplesDivisorS,
                                                     OrbitPredictionTuning::kTargetSamplesMin,
                                                     OrbitPredictionTuning::kTargetSamplesMax);
            const double dt_s = std::clamp(horizon_s / target_samples, 0.01, OrbitPredictionTuning::kMaxSampleDtS);
            return {horizon_s, dt_s};
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
                select_prediction_horizon_and_dt(mu_ref_m3_s2, ship_rel_pos_m, ship_rel_vel_mps);

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

        const orbitsim::CelestialEphemeris eph = orbitsim::build_celestial_ephemeris(sim, opt);
        const std::vector<orbitsim::TrajectorySample> traj_inertial_baseline =
                orbitsim::predict_spacecraft_trajectory(sim, eph, ship_h.id, opt);
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
            const double r_m = safe_length(sample.position_m);
            const double alt_km = (r_m - request.reference_body_radius_m) * 1.0e-3;
            const double spd_kmps = safe_length(sample.velocity_mps) * 1.0e-3;
            out.altitude_km.push_back(static_cast<float>(alt_km));
            out.speed_kmps.push_back(static_cast<float>(spd_kmps));
        }

        const OrbitalElementsEstimate elements =
                compute_orbital_elements(mu_ref_m3_s2, ship_rel_pos_m, ship_rel_vel_mps);
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

            const std::vector<orbitsim::TrajectorySample> traj_inertial_planned =
                    orbitsim::predict_spacecraft_trajectory(sim, eph, ship_h.id, opt);
            if (traj_inertial_planned.size() >= 2)
            {
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
