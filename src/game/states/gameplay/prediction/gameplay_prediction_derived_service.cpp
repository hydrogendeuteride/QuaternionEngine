#include "game/states/gameplay/prediction/gameplay_prediction_derived_service.h"

#include "game/orbit/orbit_prediction_math.h"

#include "orbitsim/trajectory_transforms.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace Game
{
    namespace
    {
        bool finite_vec3(const glm::dvec3 &v)
        {
            return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
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

                out.push_back(orbitsim::TrajectorySegment{
                        .t0_s = a.t_s,
                        .dt_s = dt_s,
                        .start = orbitsim::make_state(a.position_m, a.velocity_mps),
                        .end = orbitsim::make_state(b.position_m, b.velocity_mps),
                        .flags = 0u,
                });
            }

            return out;
        }

        std::shared_ptr<const std::vector<OrbitPlotSystem::GpuRootSegment>> build_gpu_root_cache(
                const std::vector<orbitsim::TrajectorySegment> &segments)
        {
            auto out = std::make_shared<std::vector<OrbitPlotSystem::GpuRootSegment>>();
            out->reserve(segments.size());

            double prefix_length_m = 0.0;
            for (const orbitsim::TrajectorySegment &segment : segments)
            {
                if (!(segment.dt_s > 0.0) || !std::isfinite(segment.dt_s))
                {
                    continue;
                }

                const glm::dvec3 p0 = glm::dvec3(segment.start.position_m);
                const glm::dvec3 p1 = glm::dvec3(segment.end.position_m);
                const glm::dvec3 v0 = glm::dvec3(segment.start.velocity_mps);
                const glm::dvec3 v1 = glm::dvec3(segment.end.velocity_mps);
                if (!finite_vec3(p0) || !finite_vec3(p1) || !finite_vec3(v0) || !finite_vec3(v1))
                {
                    continue;
                }

                OrbitPlotSystem::GpuRootSegment root{};
                root.t0_s = segment.t0_s;
                root.p0_bci = p0;
                root.v0_bci = v0;
                root.p1_bci = p1;
                root.v1_bci = v1;
                root.dt_s = segment.dt_s;
                root.prefix_length_m = prefix_length_m;
                out->push_back(root);

                const double chord_m = glm::length(p1 - p0);
                if (std::isfinite(chord_m) && chord_m > 0.0)
                {
                    prefix_length_m += chord_m;
                }
            }

            return out;
        }

        bool sample_prediction_inertial_state(const std::vector<orbitsim::TrajectorySample> &trajectory,
                                              const double query_time_s,
                                              orbitsim::State &out_state)
        {
            out_state = {};
            if (trajectory.empty() || !std::isfinite(query_time_s))
            {
                return false;
            }

            const auto it_hi = std::lower_bound(trajectory.cbegin(),
                                                trajectory.cend(),
                                                query_time_s,
                                                [](const orbitsim::TrajectorySample &sample, const double t_s) {
                                                    return sample.t_s < t_s;
                                                });
            const std::size_t i_hi = static_cast<std::size_t>(std::distance(trajectory.cbegin(), it_hi));
            if (i_hi == 0)
            {
                out_state = orbitsim::make_state(trajectory.front().position_m, trajectory.front().velocity_mps);
                return true;
            }
            if (i_hi >= trajectory.size())
            {
                out_state = orbitsim::make_state(trajectory.back().position_m, trajectory.back().velocity_mps);
                return true;
            }

            const orbitsim::TrajectorySample &a = trajectory[i_hi - 1];
            const orbitsim::TrajectorySample &b = trajectory[i_hi];
            const double h = b.t_s - a.t_s;
            if (!(h > 0.0) || !std::isfinite(h))
            {
                out_state = orbitsim::make_state(a.position_m, a.velocity_mps);
                return true;
            }

            double u = (query_time_s - a.t_s) / h;
            if (!std::isfinite(u))
            {
                u = 0.0;
            }
            u = std::clamp(u, 0.0, 1.0);

            const double u2 = u * u;
            const double u3 = u2 * u;
            const double h00 = (2.0 * u3) - (3.0 * u2) + 1.0;
            const double h10 = u3 - (2.0 * u2) + u;
            const double h01 = (-2.0 * u3) + (3.0 * u2);
            const double h11 = u3 - u2;
            const double dh00 = (6.0 * u2) - (6.0 * u);
            const double dh10 = (3.0 * u2) - (4.0 * u) + 1.0;
            const double dh01 = (-6.0 * u2) + (6.0 * u);
            const double dh11 = (3.0 * u2) - (2.0 * u);

            const glm::dvec3 p0 = glm::dvec3(a.position_m);
            const glm::dvec3 p1 = glm::dvec3(b.position_m);
            const glm::dvec3 m0 = glm::dvec3(a.velocity_mps) * h;
            const glm::dvec3 m1 = glm::dvec3(b.velocity_mps) * h;
            const glm::dvec3 pos = (h00 * p0) + (h10 * m0) + (h01 * p1) + (h11 * m1);
            const glm::dvec3 vel = ((dh00 * p0) + (dh10 * m0) + (dh01 * p1) + (dh11 * m1)) / h;
            if (!finite_vec3(pos) || !finite_vec3(vel))
            {
                return false;
            }

            out_state = orbitsim::make_state(pos, vel);
            return true;
        }

        orbitsim::SpacecraftStateLookup build_player_lookup(
                const std::vector<orbitsim::TrajectorySample> &trajectory)
        {
            if (trajectory.size() < 2)
            {
                return nullptr;
            }

            return [&trajectory](const orbitsim::SpacecraftId spacecraft_id, const double t_s)
                    -> std::optional<orbitsim::State> {
                static constexpr orbitsim::SpacecraftId kPlayerDisplayTargetSpacecraftId =
                        static_cast<orbitsim::SpacecraftId>(0x7000'0001u);
                if (spacecraft_id != kPlayerDisplayTargetSpacecraftId)
                {
                    return std::nullopt;
                }

                orbitsim::State sampled{};
                if (!sample_prediction_inertial_state(trajectory, t_s, sampled))
                {
                    return std::nullopt;
                }
                return sampled;
            };
        }

        std::vector<orbitsim::TrajectorySegment> transform_segments_to_body_centered_inertial(
                const OrbitPredictionService::SharedCelestialEphemeris &shared_ephemeris,
                const std::vector<orbitsim::TrajectorySegment> &segments_inertial,
                const orbitsim::MassiveBody &reference_body)
        {
            std::vector<orbitsim::TrajectorySegment> out;
            out.reserve(segments_inertial.size());

            for (const orbitsim::TrajectorySegment &segment : segments_inertial)
            {
                const double t0_s = segment.t0_s;
                const double t1_s = t0_s + segment.dt_s;
                if (!(segment.dt_s > 0.0) || !std::isfinite(t0_s) || !std::isfinite(t1_s))
                {
                    continue;
                }

                const orbitsim::State ref_start = shared_ephemeris->body_state_at_by_id(reference_body.id, t0_s);
                const orbitsim::State ref_end = shared_ephemeris->body_state_at_by_id(reference_body.id, t1_s);

                out.push_back(orbitsim::TrajectorySegment{
                        .t0_s = t0_s,
                        .dt_s = segment.dt_s,
                        .start = orbitsim::make_state(segment.start.position_m - ref_start.position_m,
                                                      segment.start.velocity_mps - ref_start.velocity_mps),
                        .end = orbitsim::make_state(segment.end.position_m - ref_end.position_m,
                                                    segment.end.velocity_mps - ref_end.velocity_mps),
                        .flags = segment.flags,
                });
            }

            return out;
        }
    } // namespace

    OrbitPredictionDerivedService::OrbitPredictionDerivedService()
    {
        const unsigned int hw_threads = std::max(1u, std::thread::hardware_concurrency());
        const std::size_t worker_count = std::clamp<std::size_t>(static_cast<std::size_t>(hw_threads > 2 ? 2 : 1), 1, 2);
        _workers.reserve(worker_count);
        for (std::size_t i = 0; i < worker_count; ++i)
        {
            _workers.emplace_back(&OrbitPredictionDerivedService::worker_loop, this);
        }
    }

    OrbitPredictionDerivedService::~OrbitPredictionDerivedService()
    {
        {
            std::lock_guard<std::mutex> lock(_mutex);
            _running = false;
            _pending_jobs.clear();
            _completed.clear();
        }
        _cv.notify_all();

        for (std::thread &worker : _workers)
        {
            if (worker.joinable())
            {
                worker.join();
            }
        }
    }

    void OrbitPredictionDerivedService::request(Request request)
    {
        {
            std::lock_guard<std::mutex> lock(_mutex);
            _latest_requested_generation_by_track[request.track_id] = request.generation_id;

            auto existing = std::find_if(_pending_jobs.begin(),
                                         _pending_jobs.end(),
                                         [track_id = request.track_id](const PendingJob &job) {
                                             return job.track_id == track_id;
                                         });
            if (existing != _pending_jobs.end())
            {
                existing->request_epoch = _request_epoch;
                existing->generation_id = request.generation_id;
                existing->request = std::move(request);
            }
            else
            {
                PendingJob job{};
                job.track_id = request.track_id;
                job.request_epoch = _request_epoch;
                job.generation_id = request.generation_id;
                job.request = std::move(request);
                _pending_jobs.push_back(std::move(job));
            }
        }
        _cv.notify_one();
    }

    std::optional<OrbitPredictionDerivedService::Result> OrbitPredictionDerivedService::poll_completed()
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

    void OrbitPredictionDerivedService::reset()
    {
        std::lock_guard<std::mutex> lock(_mutex);
        ++_request_epoch;
        _pending_jobs.clear();
        _completed.clear();
        _latest_requested_generation_by_track.clear();
    }

    bool OrbitPredictionDerivedService::should_publish_result(
            const PendingJob &job,
            const uint64_t current_request_epoch,
            const std::unordered_map<uint64_t, uint64_t> &latest_requested_generation_by_track)
    {
        if (job.request_epoch != current_request_epoch)
        {
            return false;
        }

        const auto latest_it = latest_requested_generation_by_track.find(job.track_id);
        return latest_it == latest_requested_generation_by_track.end() ||
               job.generation_id >= latest_it->second;
    }

    bool OrbitPredictionDerivedService::should_continue_job(const uint64_t track_id,
                                                            const uint64_t generation_id,
                                                            const uint64_t request_epoch) const
    {
        std::lock_guard<std::mutex> lock(_mutex);
        if (!_running || request_epoch != _request_epoch)
        {
            return false;
        }

        const auto latest_it = _latest_requested_generation_by_track.find(track_id);
        return latest_it == _latest_requested_generation_by_track.end() ||
               generation_id >= latest_it->second;
    }

    OrbitPredictionDerivedService::Result OrbitPredictionDerivedService::build_cache(const PendingJob &job) const
    {
        Result out{};
        out.track_id = job.track_id;
        out.generation_id = job.generation_id;

        const auto cancel_requested = [this,
                                       track_id = job.track_id,
                                       generation_id = job.generation_id,
                                       request_epoch = job.request_epoch]() {
            return !should_continue_job(track_id, generation_id, request_epoch);
        };

        const Request &request = job.request;
        const OrbitPredictionService::Result &solver = request.solver_result;
        if (!solver.valid || solver.trajectory_inertial.size() < 2)
        {
            return out;
        }

        OrbitPredictionCache cache{};
        cache.generation_id = job.generation_id;
        cache.build_time_s = solver.build_time_s;
        cache.build_pos_world = request.build_pos_world;
        cache.build_vel_world = request.build_vel_world;
        cache.shared_ephemeris = solver.shared_ephemeris;
        cache.massive_bodies = solver.massive_bodies;
        cache.trajectory_inertial = solver.trajectory_inertial;
        cache.trajectory_inertial_planned = solver.trajectory_inertial_planned;
        cache.trajectory_segments_inertial = solver.trajectory_segments_inertial;
        cache.trajectory_segments_inertial_planned = solver.trajectory_segments_inertial_planned;
        cache.maneuver_previews = solver.maneuver_previews;
        cache.valid = true;

        const orbitsim::TrajectoryFrameSpec &resolved_frame_spec = request.resolved_frame_spec;
        const auto player_lookup = build_player_lookup(request.player_lookup_trajectory_inertial);

        if (resolved_frame_spec.type == orbitsim::TrajectoryFrameType::Inertial)
        {
            cache.trajectory_frame = cache.trajectory_inertial;
            cache.trajectory_frame_planned = cache.trajectory_inertial_planned;
            cache.trajectory_segments_frame = cache.trajectory_segments_inertial;
            cache.trajectory_segments_frame_planned = cache.trajectory_segments_inertial_planned;
        }
        else if (resolved_frame_spec.type == orbitsim::TrajectoryFrameType::BodyCenteredInertial)
        {
            if (!cache.shared_ephemeris || cache.shared_ephemeris->empty())
            {
                return out;
            }

            const auto body_it = std::find_if(cache.massive_bodies.begin(),
                                              cache.massive_bodies.end(),
                                              [body_id = resolved_frame_spec.primary_body_id](const orbitsim::MassiveBody &body) {
                                                  return body.id == body_id;
                                              });
            if (body_it == cache.massive_bodies.end())
            {
                return out;
            }

            cache.trajectory_frame = orbitsim::trajectory_to_frame_spec(cache.trajectory_inertial,
                                                                        *cache.shared_ephemeris,
                                                                        cache.massive_bodies,
                                                                        resolved_frame_spec,
                                                                        player_lookup);
            if (!cache.trajectory_inertial_planned.empty())
            {
                cache.trajectory_frame_planned = orbitsim::trajectory_to_frame_spec(cache.trajectory_inertial_planned,
                                                                                    *cache.shared_ephemeris,
                                                                                    cache.massive_bodies,
                                                                                    resolved_frame_spec,
                                                                                    player_lookup);
            }

            cache.trajectory_segments_frame = transform_segments_to_body_centered_inertial(cache.shared_ephemeris,
                                                                                           cache.trajectory_segments_inertial,
                                                                                           *body_it);
            if (!cache.trajectory_segments_inertial_planned.empty())
            {
                cache.trajectory_segments_frame_planned =
                        transform_segments_to_body_centered_inertial(cache.shared_ephemeris,
                                                                     cache.trajectory_segments_inertial_planned,
                                                                     *body_it);
            }
        }
        else
        {
            if (!cache.shared_ephemeris || cache.shared_ephemeris->empty())
            {
                return out;
            }

            cache.trajectory_frame = orbitsim::trajectory_to_frame_spec(cache.trajectory_inertial,
                                                                        *cache.shared_ephemeris,
                                                                        cache.massive_bodies,
                                                                        resolved_frame_spec,
                                                                        player_lookup);
            if (!cache.trajectory_inertial_planned.empty())
            {
                cache.trajectory_frame_planned = orbitsim::trajectory_to_frame_spec(cache.trajectory_inertial_planned,
                                                                                    *cache.shared_ephemeris,
                                                                                    cache.massive_bodies,
                                                                                    resolved_frame_spec,
                                                                                    player_lookup);
            }
        }

        if (cancel_requested())
        {
            return out;
        }

        if (cache.trajectory_frame.size() < 2)
        {
            return out;
        }

        if (cache.trajectory_segments_frame.empty())
        {
            cache.trajectory_segments_frame = trajectory_segments_from_samples(cache.trajectory_frame);
        }
        if (!cache.trajectory_frame_planned.empty() && cache.trajectory_segments_frame_planned.empty())
        {
            cache.trajectory_segments_frame_planned = trajectory_segments_from_samples(cache.trajectory_frame_planned);
        }

        cache.gpu_roots_frame = build_gpu_root_cache(cache.trajectory_segments_frame);
        cache.gpu_roots_frame_planned = build_gpu_root_cache(cache.trajectory_segments_frame_planned);

        cache.render_curve_frame = cache.trajectory_segments_frame.empty()
                                           ? OrbitRenderCurve{}
                                           : OrbitRenderCurve::build(cache.trajectory_segments_frame);
        cache.render_curve_frame_planned = cache.trajectory_segments_frame_planned.empty()
                                                   ? OrbitRenderCurve{}
                                                   : OrbitRenderCurve::build(cache.trajectory_segments_frame_planned);
        cache.resolved_frame_spec = resolved_frame_spec;
        cache.resolved_frame_spec_valid = true;

        cache.altitude_km.clear();
        cache.speed_kmps.clear();
        cache.semi_major_axis_m = 0.0;
        cache.eccentricity = 0.0;
        cache.orbital_period_s = 0.0;
        cache.periapsis_alt_km = 0.0;
        cache.apoapsis_alt_km = std::numeric_limits<double>::infinity();
        cache.metrics_body_id = request.analysis_body_id;
        cache.metrics_valid = true;

        const auto analysis_it = std::find_if(cache.massive_bodies.begin(),
                                              cache.massive_bodies.end(),
                                              [body_id = request.analysis_body_id](const orbitsim::MassiveBody &body) {
                                                  return body.id == body_id;
                                              });
        if (analysis_it != cache.massive_bodies.end() && analysis_it->mass_kg > 0.0)
        {
            const double mu_ref_m3_s2 = request.sim_config.gravitational_constant * analysis_it->mass_kg;
            if (mu_ref_m3_s2 > 0.0 && std::isfinite(mu_ref_m3_s2))
            {
                std::vector<orbitsim::TrajectorySample> rel_samples;
                if (cache.resolved_frame_spec_valid &&
                    cache.resolved_frame_spec.type == orbitsim::TrajectoryFrameType::BodyCenteredInertial &&
                    cache.resolved_frame_spec.primary_body_id == request.analysis_body_id)
                {
                    rel_samples = cache.trajectory_frame;
                }
                else if (cache.shared_ephemeris && !cache.shared_ephemeris->empty())
                {
                    rel_samples = orbitsim::trajectory_to_frame_spec(
                            cache.trajectory_inertial,
                            *cache.shared_ephemeris,
                            cache.massive_bodies,
                            orbitsim::TrajectoryFrameSpec::body_centered_inertial(request.analysis_body_id));
                }

                if (rel_samples.size() >= 2)
                {
                    cache.altitude_km.reserve(rel_samples.size());
                    cache.speed_kmps.reserve(rel_samples.size());
                    for (const orbitsim::TrajectorySample &sample : rel_samples)
                    {
                        const double r_m = glm::length(sample.position_m);
                        const double alt_km = (r_m - analysis_it->radius_m) * 1.0e-3;
                        const double spd_kmps = glm::length(sample.velocity_mps) * 1.0e-3;
                        cache.altitude_km.push_back(static_cast<float>(alt_km));
                        cache.speed_kmps.push_back(static_cast<float>(spd_kmps));
                    }

                    const OrbitPredictionMath::OrbitalElementsEstimate elements =
                            OrbitPredictionMath::compute_orbital_elements(mu_ref_m3_s2,
                                                                          rel_samples.front().position_m,
                                                                          rel_samples.front().velocity_mps);
                    if (elements.valid)
                    {
                        cache.semi_major_axis_m = elements.semi_major_axis_m;
                        cache.eccentricity = elements.eccentricity;
                        cache.orbital_period_s = elements.orbital_period_s;
                        cache.periapsis_alt_km = (elements.periapsis_m - analysis_it->radius_m) * 1.0e-3;
                        cache.apoapsis_alt_km = std::isfinite(elements.apoapsis_m)
                                                       ? (elements.apoapsis_m - analysis_it->radius_m) * 1.0e-3
                                                       : std::numeric_limits<double>::infinity();
                    }
                }
            }
        }

        if (cancel_requested())
        {
            return out;
        }

        out.cache = std::move(cache);
        out.valid = out.cache.valid;
        return out;
    }

    void OrbitPredictionDerivedService::worker_loop()
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

            if (!should_continue_job(job.track_id, job.generation_id, job.request_epoch))
            {
                continue;
            }

            Result result = build_cache(job);

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
} // namespace Game
