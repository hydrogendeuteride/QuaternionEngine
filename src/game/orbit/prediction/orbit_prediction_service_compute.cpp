#include "game/orbit/orbit_prediction_service.h"
#include "game/orbit/prediction/orbit_prediction_service_internal.h"

#include <chrono>

namespace Game
{
    namespace
    {
        using PredictionClock = std::chrono::steady_clock;
        using Request = OrbitPredictionService::Request;
        using Result = OrbitPredictionService::Result;
        using Status = OrbitPredictionService::Status;

        Result make_initial_prediction_result(
                const uint64_t generation_id,
                const Request &request)
        {
            Result out{};
            out.generation_id = generation_id;
            out.track_id = request.track_id;
            out.maneuver_plan_revision = request.maneuver_plan_revision;
            out.maneuver_plan_signature_valid = request.maneuver_plan_signature_valid;
            out.maneuver_plan_signature = request.maneuver_plan_signature;
            out.build_time_s = request.sim_time_s;
            out.solve_quality = request.solve_quality;
            return out;
        }

        double elapsed_prediction_compute_ms(const PredictionClock::time_point compute_start)
        {
            return std::chrono::duration<double, std::milli>(PredictionClock::now() - compute_start).count();
        }

        void ensure_single_publish_chunk_metadata(Result &result)
        {
            if (!result.published_chunks.empty())
            {
                return;
            }

            const bool has_planned_path = !result.trajectory_segments_inertial_planned.empty();
            const std::vector<orbitsim::TrajectorySegment> &segments =
                    has_planned_path ? result.trajectory_segments_inertial_planned
                                     : result.resolved_trajectory_segments_inertial();
            if (segments.empty())
            {
                return;
            }

            result.published_chunks.push_back(OrbitPredictionService::PublishedChunk{
                    .chunk_id = 0u,
                    .quality_state = OrbitPredictionService::ChunkQualityState::Final,
                    .t0_s = segments.front().t0_s,
                    .t1_s = prediction_segment_end_time(segments.back()),
                    .includes_planned_path = has_planned_path,
                    .reused_from_cache = has_planned_path
                                                 ? result.diagnostics.trajectory_planned.cache_reused
                                                 : result.baseline_reused,
            });
        }

        std::optional<OrbitPredictionService::EphemerisSamplingSpec> resolve_spacecraft_sampling_spec(
                const Request &request,
                Status &out_status)
        {
            if (request.kind != OrbitPredictionService::RequestKind::Spacecraft)
            {
                out_status = Status::Success;
                return std::nullopt;
            }

            OrbitPredictionService::EphemerisSamplingSpec sampling_spec =
                    OrbitPredictionService::build_ephemeris_sampling_spec(request);
            if (!sampling_spec.valid)
            {
                out_status = Status::InvalidSamplingSpec;
                return std::nullopt;
            }

            out_status = Status::Success;
            return sampling_spec;
        }

        double resolve_integrator_horizon_s(
                const Request &request,
                const std::optional<OrbitPredictionService::EphemerisSamplingSpec> &spacecraft_sampling_spec)
        {
            if (spacecraft_sampling_spec.has_value())
            {
                return spacecraft_sampling_spec->horizon_s;
            }

            return std::max(OrbitPredictionTuning::kMinHorizonS,
                            std::isfinite(request.future_window_s) ? std::max(0.0, request.future_window_s) : 0.0);
        }

        orbitsim::GameSimulation::Config build_prediction_sim_config(
                const Request &request,
                const std::optional<OrbitPredictionService::EphemerisSamplingSpec> &spacecraft_sampling_spec)
        {
            orbitsim::GameSimulation::Config sim_config = request.sim_config;
            apply_prediction_integrator_profile(sim_config,
                                                request,
                                                resolve_integrator_horizon_s(request, spacecraft_sampling_spec));
            if (request.lagrange_sensitive)
            {
                apply_lagrange_integrator_profile(sim_config);
            }
            return sim_config;
        }

        Status populate_prediction_bodies(orbitsim::GameSimulation &sim, const Request &request)
        {
            for (const orbitsim::MassiveBody &body : request.massive_bodies)
            {
                const auto body_handle =
                        (body.id != orbitsim::kInvalidBodyId)
                            ? sim.create_body_with_id(body.id, body)
                            : sim.create_body(body);
                if (!body_handle.valid())
                {
                    return Status::InvalidSubject;
                }
            }
            return Status::Success;
        }

        struct PreparedPredictionJob
        {
            Status status{Status::Success};
            std::optional<OrbitPredictionService::EphemerisSamplingSpec> spacecraft_sampling_spec{};
            orbitsim::GameSimulation sim{};
        };

        PreparedPredictionJob prepare_prediction_job(
                const Request &request,
                const CancelCheck &cancel_requested)
        {
            PreparedPredictionJob prepared{};

            if (!std::isfinite(request.sim_time_s))
            {
                prepared.status = Status::InvalidInput;
                return prepared;
            }

            Status sampling_status = Status::Success;
            prepared.spacecraft_sampling_spec = resolve_spacecraft_sampling_spec(request, sampling_status);
            if (sampling_status != Status::Success)
            {
                prepared.status = sampling_status;
                return prepared;
            }

            prepared.sim = orbitsim::GameSimulation(build_prediction_sim_config(request,
                                                                                prepared.spacecraft_sampling_spec));
            if (!prepared.sim.set_time_s(request.sim_time_s))
            {
                prepared.status = Status::InvalidInput;
                return prepared;
            }

            if (cancel_requested())
            {
                prepared.status = Status::Cancelled;
                return prepared;
            }

            prepared.status = populate_prediction_bodies(prepared.sim, request);
            if (prepared.status != Status::Success)
            {
                return prepared;
            }

            if (cancel_requested())
            {
                prepared.status = Status::Cancelled;
                return prepared;
            }

            return prepared;
        }
    } // namespace

    struct OrbitPredictionService::PredictionRouteServiceAdapter final : SpacecraftPredictionRouteServices
    {
        OrbitPredictionService &service;

        explicit PredictionRouteServiceAdapter(OrbitPredictionService &service_)
            : service(service_)
        {
        }

        std::optional<ReusableSpacecraftBaseline> find_reusable_baseline(
                const uint64_t track_id,
                const uint64_t baseline_request_epoch) override
        {
            const std::optional<ReusableBaselineCacheEntry> baseline =
                    service.find_reusable_baseline(track_id, baseline_request_epoch);
            if (!baseline.has_value())
            {
                return std::nullopt;
            }

            return ReusableSpacecraftBaseline{
                    .trajectory_inertial = baseline->trajectory_inertial,
                    .trajectory_segments_inertial = baseline->trajectory_segments_inertial,
            };
        }

        void store_reusable_baseline(
                const uint64_t track_id,
                const uint64_t baseline_generation_id,
                const uint64_t baseline_request_epoch,
                SharedCelestialEphemeris shared_ephemeris,
                std::vector<orbitsim::TrajectorySample> trajectory_inertial,
                std::vector<orbitsim::TrajectorySegment> trajectory_segments_inertial) override
        {
            service.store_reusable_baseline(track_id,
                                            baseline_generation_id,
                                            baseline_request_epoch,
                                            std::move(shared_ephemeris),
                                            std::move(trajectory_inertial),
                                            std::move(trajectory_segments_inertial));
        }

        std::optional<PlannedChunkCacheEntry> find_cached_chunk(
                const PlannedChunkCacheKey &key,
                const orbitsim::State &expected_start_state) override
        {
            return service.find_cached_planned_chunk(key, expected_start_state);
        }

        void store_cached_chunk(PlannedChunkCacheEntry entry) override
        {
            service.store_cached_planned_chunk(std::move(entry));
        }

        SharedCelestialEphemeris get_or_build_ephemeris(
                const EphemerisBuildRequest &request,
                const CancelCheck &cancel_requested) override
        {
            return service.get_or_build_ephemeris(request, cancel_requested);
        }

        SharedCelestialEphemeris resolve_ephemeris(
                const EphemerisBuildRequest &request,
                const CancelCheck &cancel_requested,
                AdaptiveStageDiagnostics *diagnostics)
        {
            return service.get_or_build_ephemeris(request, cancel_requested, diagnostics);
        }
    };

    struct OrbitPredictionService::PredictionJobResultPublisher final
    {
        OrbitPredictionService &service;
        const PendingJob &job;
        PredictionClock::time_point compute_start{};

        PredictionJobResultPublisher(
                OrbitPredictionService &service_,
                const PendingJob &job_,
                const PredictionClock::time_point compute_start_)
            : service(service_),
              job(job_),
              compute_start(compute_start_)
        {
        }

        bool publish(Result result)
        {
            result.compute_time_ms = elapsed_prediction_compute_ms(compute_start);
            return service.publish_completed_result(job, std::move(result));
        }

        void publish_failure(Result result, const Status status)
        {
            result.diagnostics.status = status;
            publish(std::move(result));
        }

        void publish_cancelled(Result result)
        {
            result.diagnostics.cancelled = true;
            result.diagnostics.status = Status::Cancelled;
            publish(std::move(result));
        }

        void publish_success(Result result)
        {
            ensure_single_publish_chunk_metadata(result);
            result.valid = true;
            result.diagnostics.status = Status::Success;
            publish(std::move(result));
        }
    };

    struct OrbitPredictionService::PredictionJobRunner final
    {
        OrbitPredictionService &service;
        PredictionRouteServiceAdapter route_services;
        const PendingJob &job;
        const uint64_t generation_id{0};
        const uint64_t request_epoch{0};
        const Request &request;
        PredictionClock::time_point compute_start{};
        PredictionJobResultPublisher result_publisher;
        Result out{};
        CancelCheck cancel_requested{};
        EphemerisResolverFn resolve_ephemeris{};
        PublishFn publish_result{};

        PredictionJobRunner(OrbitPredictionService &service_, const PendingJob &job_)
            : service(service_),
              route_services(service_),
              job(job_),
              generation_id(job.generation_id),
              request_epoch(job.request_epoch),
              request(job.request),
              compute_start(PredictionClock::now()),
              result_publisher(service_, job_, compute_start),
              out(make_initial_prediction_result(generation_id, request))
        {
            cancel_requested = [this]() {
                return !service.should_continue_job(request.track_id,
                                                    generation_id,
                                                    request_epoch,
                                                    request.maneuver_plan_revision,
                                                    request.solve_quality);
            };
            resolve_ephemeris =
                    [this](const EphemerisBuildRequest &req,
                           const CancelCheck &cancel_check,
                           AdaptiveStageDiagnostics *diagnostics) {
                        return route_services.resolve_ephemeris(req, cancel_check, diagnostics);
                    };
            publish_result = [this](Result result) {
                return result_publisher.publish(std::move(result));
            };
        }

        void run()
        {
            PreparedPredictionJob prepared = prepare_prediction_job(request, cancel_requested);
            if (prepared.status == Status::Cancelled)
            {
                cancel();
                return;
            }
            if (prepared.status != Status::Success)
            {
                fail(prepared.status);
                return;
            }

            out.massive_bodies = prepared.sim.massive_bodies();

            if (request.kind == RequestKind::Celestial)
            {
                run_celestial_route(prepared.sim);
                return;
            }

            if (!prepared.spacecraft_sampling_spec.has_value())
            {
                fail(Status::InvalidSamplingSpec);
                return;
            }
            run_spacecraft_route(prepared.sim, *prepared.spacecraft_sampling_spec);
        }

    private:
        void fail(const Status status)
        {
            result_publisher.publish_failure(out, status);
        }

        void cancel()
        {
            result_publisher.publish_cancelled(out);
        }

        void publish_success()
        {
            result_publisher.publish_success(std::move(out));
        }

        void run_celestial_route(orbitsim::GameSimulation &sim)
        {
            const Status celestial_status = solve_celestial_prediction_route(request,
                                                                             sim,
                                                                             cancel_requested,
                                                                             resolve_ephemeris,
                                                                             out);
            if (celestial_status == Status::Cancelled)
            {
                cancel();
                return;
            }
            if (celestial_status != Status::Success)
            {
                fail(celestial_status);
                return;
            }

            publish_success();
        }

        void run_spacecraft_route(
                orbitsim::GameSimulation &sim,
                const EphemerisSamplingSpec &spacecraft_sampling_spec)
        {
            const SpacecraftPredictionRouteOutcome spacecraft_outcome =
                    solve_spacecraft_prediction_route(SpacecraftPredictionRouteEnvironment{
                            .request = request,
                            .job = PredictionRouteJobIdentity{
                                    .generation_id = generation_id,
                                    .request_epoch = request_epoch,
                            },
                            .state = PredictionRouteMutableState{
                                    .sim = sim,
                                    .out = out,
                            },
                            .sampling_spec = spacecraft_sampling_spec,
                            .resolve_ephemeris = resolve_ephemeris,
                            .callbacks = PredictionRouteCallbacks{
                                    .cancel_requested = cancel_requested,
                                    .publish = publish_result,
                                    .compute_start = compute_start,
                            },
                            .services = route_services,
                    });
            if (spacecraft_outcome.status == Status::Cancelled)
            {
                cancel();
                return;
            }
            if (spacecraft_outcome.status != Status::Success)
            {
                fail(spacecraft_outcome.status);
                return;
            }
            if (spacecraft_outcome.published_staged_preview)
            {
                return;
            }

            publish_success();
        }
    };

    void OrbitPredictionService::compute_prediction(const PendingJob &job)
    {
        PredictionJobRunner runner(*this, job);
        runner.run();
    }
} // namespace Game
