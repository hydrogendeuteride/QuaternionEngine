#pragma once

#include <sstream>

#ifndef VULKAN_ENGINE_GAMEPLAY_TEST_ACCESS
#define VULKAN_ENGINE_GAMEPLAY_TEST_ACCESS 1
#endif
#include "game/states/gameplay/gameplay_state.h"
#include "game/orbit/orbit_prediction_tuning.h"
#include "game/states/gameplay/prediction/runtime/gameplay_state_prediction_runtime_internal.h"

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <mutex>
#include <unordered_map>

namespace GameplayTestHooks
{
    void register_entity(Game::Entity *entity);
    void clear_entities();
} // namespace GameplayTestHooks

namespace
{
    std::unique_ptr<Game::OrbitalScenario> make_reference_orbitsim(const double time_s,
                                                                   const double reference_mass_kg = 5.972e24)
    {
        auto scenario = std::make_unique<Game::OrbitalScenario>();

        orbitsim::GameSimulation::Config cfg{};
        cfg.enable_events = false;
        cfg.spacecraft_integrator.adaptive = true;
        cfg.spacecraft_integrator.max_substeps = 256;
        scenario->sim = orbitsim::GameSimulation(cfg);
        (void) scenario->sim.set_time_s(time_s);

        orbitsim::MassiveBody ref{};
        ref.id = 1;
        ref.mass_kg = reference_mass_kg;
        ref.radius_m = 6'371'000.0;
        ref.state = orbitsim::make_state(glm::dvec3(0.0), glm::dvec3(0.0));

        const auto handle = scenario->sim.create_body_with_id(ref.id, ref);
        if (!handle.valid())
        {
            return nullptr;
        }

        Game::CelestialBodyInfo ref_info{};
        ref_info.sim_id = handle.id;
        ref_info.name = "earth";
        ref_info.radius_m = ref.radius_m;
        ref_info.mass_kg = ref.mass_kg;

        scenario->bodies.push_back(ref_info);
        scenario->world_reference_body_index = 0;
        return scenario;
    }

    orbitsim::TrajectorySample make_sample(const double t_s, const double x_m)
    {
        orbitsim::TrajectorySample sample{};
        sample.t_s = t_s;
        sample.position_m = orbitsim::Vec3{x_m, 0.0, 0.0};
        sample.velocity_mps = orbitsim::Vec3{0.0, 7'500.0, 0.0};
        return sample;
    }

    orbitsim::TrajectorySegment make_segment(const double t0_s, const double t1_s, const double x0_m, const double x1_m)
    {
        orbitsim::TrajectorySegment segment{};
        segment.t0_s = t0_s;
        segment.dt_s = t1_s - t0_s;
        segment.start = orbitsim::make_state(orbitsim::Vec3{x0_m, 0.0, 0.0}, orbitsim::Vec3{0.0, 7'500.0, 0.0});
        segment.end = orbitsim::make_state(orbitsim::Vec3{x1_m, 0.0, 0.0}, orbitsim::Vec3{0.0, 7'500.0, 0.0});
        return segment;
    }

    Game::OrbitChunk make_chunk(const uint32_t chunk_id,
                                const uint64_t generation_id,
                                const double t0_s,
                                const double t1_s,
                                const double x0_m,
                                const double x1_m)
    {
        Game::OrbitChunk chunk{};
        chunk.chunk_id = chunk_id;
        chunk.generation_id = generation_id;
        chunk.quality_state = Game::OrbitPredictionService::ChunkQualityState::Final;
        chunk.t0_s = t0_s;
        chunk.t1_s = t1_s;
        chunk.frame_samples = {make_sample(t0_s, x0_m), make_sample(t1_s, x1_m)};
        chunk.frame_segments = {make_segment(t0_s, t1_s, x0_m, x1_m)};
        chunk.render_curve = Game::OrbitRenderCurve::build(chunk.frame_segments);
        chunk.valid = true;
        return chunk;
    }

    Game::OrbitPredictionCache make_prediction_cache(const uint64_t generation_id,
                                                     const double t0_s,
                                                     const double t1_s,
                                                     const double x0_m,
                                                     const double x1_m)
    {
        Game::OrbitPredictionCache cache{};
        cache.valid = true;
        cache.generation_id = generation_id;
        cache.build_time_s = t0_s;
        cache.trajectory_inertial = {
                make_sample(t0_s, x0_m),
                make_sample(t1_s, x1_m),
        };
        cache.trajectory_segments_inertial = {
                make_segment(t0_s, t1_s, x0_m, x1_m),
        };
        cache.trajectory_frame = cache.trajectory_inertial;
        cache.trajectory_segments_frame = cache.trajectory_segments_inertial;
        return cache;
    }

    Game::OrbitPredictionService::Request make_prediction_request(const double time_s,
                                                                  const double future_window_s = 120.0)
    {
        Game::OrbitPredictionService::Request request{};
        request.track_id = 7;
        request.sim_time_s = time_s;
        request.sim_config.enable_events = false;
        request.sim_config.spacecraft_integrator.adaptive = true;
        request.sim_config.spacecraft_integrator.max_substeps = 256;
        request.ship_bary_position_m = orbitsim::Vec3{7'000'000.0, 0.0, 0.0};
        request.ship_bary_velocity_mps = orbitsim::Vec3{0.0, 7'500.0, 0.0};
        request.future_window_s = future_window_s;
        request.preferred_primary_body_id = 1;

        orbitsim::MassiveBody ref{};
        ref.id = 1;
        ref.mass_kg = 5.972e24;
        ref.radius_m = 6'371'000.0;
        ref.state = orbitsim::make_state(glm::dvec3(0.0), glm::dvec3(0.0));
        request.massive_bodies.push_back(ref);
        return request;
    }

    Game::OrbitPredictionService::Request make_celestial_prediction_request(const double time_s,
                                                                            const double future_window_s = 120.0)
    {
        Game::OrbitPredictionService::Request request{};
        request.kind = Game::OrbitPredictionService::RequestKind::Celestial;
        request.track_id = 9;
        request.sim_time_s = time_s;
        request.sim_config.enable_events = false;
        request.sim_config.spacecraft_integrator.adaptive = true;
        request.sim_config.spacecraft_integrator.max_substeps = 256;
        request.future_window_s = future_window_s;
        request.subject_body_id = 2;

        orbitsim::MassiveBody earth{};
        earth.id = 1;
        earth.mass_kg = 5.972e24;
        earth.radius_m = 6'371'000.0;
        earth.state = orbitsim::make_state(glm::dvec3(0.0), glm::dvec3(0.0));
        request.massive_bodies.push_back(earth);

        orbitsim::MassiveBody moon{};
        moon.id = 2;
        moon.mass_kg = 7.342e22;
        moon.radius_m = 1'737'400.0;
        moon.state = orbitsim::make_state(glm::dvec3(384'400'000.0, 0.0, 0.0),
                                          glm::dvec3(0.0, 1'022.0, 0.0));
        request.massive_bodies.push_back(moon);

        return request;
    }

    Game::OrbitPredictionService::ManeuverImpulse make_maneuver_impulse(
            const int node_id,
            const double time_s,
            const glm::dvec3 &dv_rtn_mps,
            const orbitsim::BodyId primary_body_id = 1)
    {
        Game::OrbitPredictionService::ManeuverImpulse impulse{};
        impulse.node_id = node_id;
        impulse.t_s = time_s;
        impulse.primary_body_id = primary_body_id;
        impulse.dv_rtn_mps = dv_rtn_mps;
        return impulse;
    }

    Game::OrbitPredictionService::ManeuverImpulse &add_maneuver_impulse(
            Game::OrbitPredictionService::Request &request,
            const int node_id,
            const double time_s,
            const glm::dvec3 &dv_rtn_mps,
            const orbitsim::BodyId primary_body_id = 1)
    {
        request.maneuver_impulses.push_back(
                make_maneuver_impulse(node_id, time_s, dv_rtn_mps, primary_body_id));
        return request.maneuver_impulses.back();
    }

    const Game::OrbitPredictionService::ManeuverNodePreview *find_maneuver_preview(
            const Game::OrbitPredictionService::Result &result,
            const int node_id)
    {
        for (const Game::OrbitPredictionService::ManeuverNodePreview &preview : result.maneuver_previews)
        {
            if (preview.node_id == node_id)
            {
                return &preview;
            }
        }
        return nullptr;
    }

    const Game::OrbitPredictionService::ManeuverNodePreview *find_valid_maneuver_preview(
            const Game::OrbitPredictionService::Result &result,
            const int node_id)
    {
        const Game::OrbitPredictionService::ManeuverNodePreview *preview =
                find_maneuver_preview(result, node_id);
        return preview && preview->valid ? preview : nullptr;
    }

    std::vector<Game::OrbitPredictionService::Result> run_prediction_results(
            Game::OrbitPredictionService &service,
            const uint64_t generation_id,
            Game::OrbitPredictionService::Request request,
            const uint64_t request_epoch = 1)
    {
        service._completed.clear();

        Game::OrbitPredictionService::PendingJob job{};
        job.track_id = request.track_id;
        job.request_epoch = request_epoch;
        job.generation_id = generation_id;
        job.request = std::move(request);

        service.compute_prediction(job);

        std::vector<Game::OrbitPredictionService::Result> results;
        while (auto completed = service.poll_completed())
        {
            results.push_back(std::move(*completed));
        }
        return results;
    }

    Game::OrbitPredictionDerivedService::PendingJob make_prediction_derived_job(
            const Game::OrbitPredictionService::SolveQuality solve_quality,
            const Game::OrbitPredictionService::PublishStage publish_stage)
    {
        Game::OrbitPredictionDerivedService::PendingJob job{};
        job.track_id = 17u;
        job.request_epoch = 1u;
        job.generation_id = 9u;
        job.request.track_id = job.track_id;
        job.request.display_frame_key = 1u;
        job.request.display_frame_revision = 1u;
        job.request.resolved_frame_spec = orbitsim::TrajectoryFrameSpec::inertial();

        Game::OrbitPredictionService::Result &solver = job.request.solver_result;
        solver.valid = true;
        solver.solve_quality = solve_quality;
        solver.publish_stage = publish_stage;
        solver.trajectory_inertial = {
                make_sample(0.0, 7'000'000.0),
                make_sample(10.0, 7'100'000.0),
                make_sample(20.0, 7'200'000.0),
        };
        solver.trajectory_segments_inertial = {
                make_segment(0.0, 10.0, 7'000'000.0, 7'100'000.0),
                make_segment(10.0, 20.0, 7'100'000.0, 7'200'000.0),
        };
        solver.trajectory_inertial_planned = {
                make_sample(0.0, 7'000'000.0),
                make_sample(10.0, 7'150'000.0),
                make_sample(20.0, 7'300'000.0),
        };
        solver.trajectory_segments_inertial_planned = {
                make_segment(0.0, 10.0, 7'000'000.0, 7'150'000.0),
                make_segment(10.0, 20.0, 7'150'000.0, 7'300'000.0),
        };
        return job;
    }

} // namespace
