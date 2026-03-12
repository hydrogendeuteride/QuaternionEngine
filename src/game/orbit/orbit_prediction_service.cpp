#include "game/orbit/orbit_prediction_service.h"
#include "game/orbit/orbit_prediction_math.h"
#include "game/orbit/orbit_prediction_tuning.h"

#include "orbitsim/coordinate_frames.hpp"
#include "orbitsim/frame_utils.hpp"
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
        constexpr double kEphemerisDurationEpsilonS = 1.0e-6;
        constexpr double kEphemerisDtEpsilonS = 1.0e-9;
        constexpr std::size_t kMaxCachedEphemerides = 64;

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

        bool finite_scalar(const double value)
        {
            return std::isfinite(value);
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
                const OrbitPredictionService::EphemerisBuildRequest &request)
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

            orbitsim::TrajectoryOptions opt{};
            opt.duration_s = request.duration_s;
            opt.sample_dt_s = request.celestial_dt_s;
            opt.spacecraft_sample_dt_s = request.celestial_dt_s;
            opt.spacecraft_lookup_dt_s = request.celestial_dt_s;
            opt.celestial_dt_s = request.celestial_dt_s;
            opt.max_samples = request.max_samples;
            opt.include_start = true;
            opt.include_end = true;
            opt.stop_on_impact = false;

            auto ephemeris = std::make_shared<orbitsim::CelestialEphemeris>(orbitsim::build_celestial_ephemeris(sim, opt));
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
            out.celestial_dt_s = sampling_spec.sample_dt_s;
            out.max_samples = sampling_spec.max_samples;
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

            double horizon_s = std::clamp(horizon_s_auto, OrbitPredictionTuning::kMinHorizonS, OrbitPredictionTuning::kMaxHorizonS);
            horizon_s = std::max(horizon_s, std::max(1.0, request.future_window_s));
            double dt_s = std::clamp(dt_s_auto, 0.01, OrbitPredictionTuning::kMaxSampleDtS);
            const int max_steps = OrbitPredictionTuning::kMaxStepsNormal;
            const double min_dt_for_step_budget = horizon_s / static_cast<double>(std::max(1, max_steps));
            if (std::isfinite(min_dt_for_step_budget) && min_dt_for_step_budget > 0.0)
            {
                dt_s = std::max(dt_s, min_dt_for_step_budget);
            }

            dt_s = std::clamp(dt_s, 0.01, OrbitPredictionTuning::kMaxSampleDtS);
            horizon_s = std::clamp(horizon_s, dt_s, OrbitPredictionTuning::kMaxHorizonS);
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
        // Spin up the dedicated prediction worker immediately so gameplay can stay enqueue-only.
        _worker = std::thread(&OrbitPredictionService::worker_loop, this);
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

        SharedCelestialEphemeris built = build_ephemeris_from_request(request);
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

        const std::size_t primary_index = orbitsim::auto_select_primary_index(
                request.massive_bodies,
                request.ship_bary_position_m,
                [&request](const std::size_t i) -> orbitsim::Vec3 { return request.massive_bodies[i].state.position_m; },
                request.sim_config.softening_length_m);
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
        double horizon_s = std::clamp(horizon_s_auto, OrbitPredictionTuning::kMinHorizonS, OrbitPredictionTuning::kMaxHorizonS);
        double dt_s = std::clamp(dt_s_auto, 0.01, OrbitPredictionTuning::kMaxSampleDtS);
        int max_steps = OrbitPredictionTuning::kSpacecraftMaxStepsNormal;
        double dt_min_s = 0.01;
        double dt_max_s = OrbitPredictionTuning::kMaxSampleDtS;
        // Keep solver coverage aligned with the UI's requested future draw window.
        double min_horizon_s = std::max(1.0, request.future_window_s);
        horizon_s = std::max(horizon_s, min_horizon_s);

        if (!request.maneuver_impulses.empty() &&
            std::isfinite(request.max_maneuver_time_s) &&
            request.max_maneuver_time_s > request.sim_time_s)
        {
            // Keep enough look-ahead to show the last planned node and a small post-node tail.
            const double extra_s = std::max(OrbitPredictionTuning::kPostNodeCoverageMinS,
                                            std::max(0.0, request.future_window_s));
            min_horizon_s = std::max(min_horizon_s, (request.max_maneuver_time_s - request.sim_time_s) + extra_s);
            horizon_s = std::max(horizon_s, min_horizon_s);
        }

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
                                                     OrbitPredictionTuning::kSpacecraftTargetSamplesMaxNormal);
            dt_s = std::clamp(horizon_s / target_samples, dt_min_s, dt_max_s);
        }

        const double min_dt_for_step_budget = horizon_s / static_cast<double>(std::max(1, max_steps));
        if (std::isfinite(min_dt_for_step_budget) && min_dt_for_step_budget > 0.0)
        {
            dt_s = std::max(dt_s, min_dt_for_step_budget);
        }

        dt_s = std::clamp(dt_s, dt_min_s, dt_max_s);
        horizon_s = std::clamp(horizon_s, dt_s, OrbitPredictionTuning::kMaxHorizonS);
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

            // Run the expensive simulation outside the lock so request() stays responsive.
            Result result = compute_prediction(job.generation_id, job.request);

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
        out.track_id = request.track_id;
        out.build_time_s = request.sim_time_s;

        // Invalid inputs produce an invalid result rather than throwing on the worker thread.
        if (!std::isfinite(request.sim_time_s))
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
                shared_ephemeris = get_or_build_ephemeris(build_ephemeris_build_request(request, ephemeris_sampling_spec));
            }
            if (!shared_ephemeris || shared_ephemeris->empty())
            {
                return out;
            }

            orbitsim::TrajectoryOptions opt{};
            opt.duration_s = sampling_spec.horizon_s;
            opt.sample_dt_s = sampling_spec.sample_dt_s;
            opt.spacecraft_sample_dt_s = sampling_spec.sample_dt_s;
            opt.spacecraft_lookup_dt_s = sampling_spec.sample_dt_s;
            opt.celestial_dt_s = sampling_spec.sample_dt_s;
            opt.max_samples = sampling_spec.max_samples;
            opt.include_start = true;
            opt.include_end = true;
            opt.stop_on_impact = false;

            std::vector<orbitsim::TrajectorySample> traj_inertial =
                    orbitsim::predict_body_trajectory(sim, *shared_ephemeris, subject_sim->id, opt);
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
        const double dt_s = sampling_spec.sample_dt_s;

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
        opt.max_samples = sampling_spec.max_samples;
        opt.include_start = true;
        opt.include_end = true;
        opt.stop_on_impact = false;

        orbitsim::TrajectorySegmentOptions segment_opt{};
        segment_opt.duration_s = horizon_s;
        segment_opt.max_segments = std::max<std::size_t>(1, (opt.max_samples > 0) ? (opt.max_samples - 1) : 0);
        segment_opt.include_start = true;
        segment_opt.include_end = true;
        segment_opt.stop_on_impact = false;

        SharedCelestialEphemeris shared_ephemeris = request.shared_ephemeris;
        if (!shared_ephemeris ||
            shared_ephemeris->empty() ||
            (shared_ephemeris->t_end_s() + kEphemerisDurationEpsilonS) < (request.sim_time_s + horizon_s))
        {
            // Spacecraft requests follow the same rule: never build ephemeris on the gameplay thread.
            shared_ephemeris = get_or_build_ephemeris(build_ephemeris_build_request(request, sampling_spec));
        }
        if (!shared_ephemeris || shared_ephemeris->empty())
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

        const std::vector<orbitsim::TrajectorySample> traj_inertial_baseline =
                orbitsim::sample_trajectory_segments_uniform_dt(traj_segments_inertial_baseline, opt);
        if (traj_inertial_baseline.size() < 2)
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

            // Build each node preview from the already-planned prefix so later nodes see earlier burns.
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
                    const orbitsim::MassiveBody *primary_sim = sim.body_by_id(src.primary_body_id);
                    if (primary_sim)
                    {
                        const orbitsim::State primary_state = eph.body_state_at_by_id(src.primary_body_id, src.t_s);
                        const glm::dvec3 rel_position_m = preview.inertial_position_m - primary_state.position_m;
                        const glm::dvec3 rel_velocity_mps = preview.inertial_velocity_mps - primary_state.velocity_mps;
                        solver_dv_rtn =
                                convert_dv_rtf_to_solver_rtn(src.dv_rtn_mps, rel_position_m, rel_velocity_mps);
                    }
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

            // Planned output shows the same prediction window with all maneuver nodes applied.
            const std::vector<orbitsim::TrajectorySegment> traj_segments_inertial_planned =
                    orbitsim::predict_spacecraft_trajectory_segments(sim, eph, ship_h.id, segment_opt);
            if (!traj_segments_inertial_planned.empty())
            {
                out.trajectory_segments_inertial_planned = traj_segments_inertial_planned;

                const std::vector<orbitsim::TrajectorySample> traj_inertial_planned =
                        orbitsim::sample_trajectory_segments_uniform_dt(traj_segments_inertial_planned, opt);
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
