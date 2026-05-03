#include "game/states/gameplay/prediction/prediction_frame_context_builder.h"

#include "game/game_world.h"
#include "game/states/gameplay/maneuver/maneuver_system.h"
#include "game/states/gameplay/orbital_runtime_system.h"
#include "game/states/gameplay/prediction/prediction_subject_state_provider.h"
#include "game/states/gameplay/prediction/prediction_system.h"
#include "game/states/gameplay/prediction/prediction_trajectory_sampler.h"
#include "game/states/gameplay/scenario/scenario_config.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace Game
{
    PredictionFrameContextBuilder::PredictionFrameContextBuilder(GameplayPredictionContext context)
        : _context(std::move(context))
    {
    }

    PredictionSubjectKey PredictionFrameContextBuilder::player_subject_key() const
    {
        return PredictionSubjectStateProvider(PredictionSubjectStateProvider::Context{
                .orbit = _context.orbit,
                .world = _context.world,
                .physics = _context.physics,
                .physics_context = _context.physics_context,
                .scenario_config = _context.scenario_config,
                .orbital_physics = _context.orbital_physics,
                .time_warp = _context.time_warp,
        }).player_subject_key();
    }

    const PredictionTrackState *PredictionFrameContextBuilder::find_track(const PredictionSubjectKey key) const
    {
        return _context.prediction ? _context.prediction->find_track(key) : nullptr;
    }

    const PredictionTrackState *PredictionFrameContextBuilder::player_track() const
    {
        return _context.prediction ? _context.prediction->player_track(player_subject_key()) : nullptr;
    }

    const OrbitPredictionCache *PredictionFrameContextBuilder::effective_cache(const PredictionTrackState *track) const
    {
        return _context.prediction ? _context.prediction->effective_cache(track) : nullptr;
    }

    const PredictionTrackState *PredictionFrameContextBuilder::player_prediction_track() const
    {
        return player_track();
    }

    const OrbitPredictionCache *PredictionFrameContextBuilder::effective_prediction_cache(
            const PredictionTrackState *track) const
    {
        return effective_cache(track);
    }

    const CelestialBodyInfo *PredictionFrameContextBuilder::find_celestial_body_info(
            const orbitsim::BodyId body_id) const
    {
        if (!_context.orbit.scenario_owner() || body_id == orbitsim::kInvalidBodyId)
        {
            return nullptr;
        }

        for (const CelestialBodyInfo &body : _context.orbit.scenario_owner()->bodies)
        {
            if (body.sim_id == body_id)
            {
                return &body;
            }
        }

        return nullptr;
    }

    WorldVec3 PredictionFrameContextBuilder::prediction_body_world_position(
            const orbitsim::BodyId body_id,
            const OrbitPredictionCache *cache,
            const double query_time_s) const
    {
        if (body_id == orbitsim::kInvalidBodyId)
        {
            return prediction_world_reference_body_world();
        }

        if (const CelestialBodyInfo *body_info = find_celestial_body_info(body_id))
        {
            if (body_info->render_entity.is_valid())
            {
                if (const Entity *entity = _context.world.entities().find(body_info->render_entity))
                {
                    return entity->position_world();
                }
            }
        }

        const double sample_time_s =
                std::isfinite(query_time_s)
                    ? query_time_s
                    : (_context.orbit.scenario_owner() ? _context.orbit.scenario_owner()->sim.time_s() : 0.0);
        const OrbitPredictionCache *position_cache = cache;
        if (!position_cache)
        {
            position_cache = effective_cache(player_track());
        }

        const orbitsim::MassiveBody *body = nullptr;
        if (position_cache)
        {
            body = PredictionFrameResolver::find_massive_body(position_cache->resolved_massive_bodies(), body_id);
        }
        if (!body && _context.orbit.scenario_owner())
        {
            body = _context.orbit.scenario_owner()->sim.body_by_id(body_id);
        }
        if (!body)
        {
            return prediction_world_reference_body_world();
        }

        const auto body_state_at = [&](const orbitsim::MassiveBody &massive_body) -> orbitsim::State {
            const auto &ephemeris = position_cache ? position_cache->resolved_shared_ephemeris()
                                                   : OrbitPredictionService::SharedCelestialEphemeris{};
            if (ephemeris && !ephemeris->empty())
            {
                return ephemeris->body_state_at_by_id(massive_body.id, sample_time_s);
            }
            return massive_body.state;
        };

        const WorldVec3 world_ref_world = prediction_world_reference_body_world();
        orbitsim::State world_ref_state{};
        bool have_world_ref_state = false;
        if (_context.orbit.scenario_owner())
        {
            if (const CelestialBodyInfo *world_ref_info = _context.orbit.scenario_owner()->world_reference_body())
            {
                if (position_cache)
                {
                    if (const orbitsim::MassiveBody *world_ref_body =
                                PredictionFrameResolver::find_massive_body(
                                        position_cache->resolved_massive_bodies(),
                                        world_ref_info->sim_id))
                    {
                        world_ref_state = body_state_at(*world_ref_body);
                        have_world_ref_state = true;
                    }
                }
                if (!have_world_ref_state)
                {
                    if (const orbitsim::MassiveBody *world_ref_sim =
                                _context.orbit.scenario_owner()->world_reference_sim_body())
                    {
                        world_ref_state = world_ref_sim->state;
                        have_world_ref_state = true;
                    }
                }
            }
        }

        const glm::dvec3 world_ref_position_m =
                have_world_ref_state ? glm::dvec3(world_ref_state.position_m) : glm::dvec3(0.0);
        const orbitsim::State body_state = body_state_at(*body);
        return world_ref_world + WorldVec3(glm::dvec3(body_state.position_m) - world_ref_position_m);
    }

    WorldVec3 PredictionFrameContextBuilder::prediction_world_reference_body_world() const
    {
        if (!_context.orbit.scenario_owner())
        {
            return _context.scenario_config.system_center;
        }

        const CelestialBodyInfo *ref_info = _context.orbit.scenario_owner()->world_reference_body();
        if (ref_info && ref_info->render_entity.is_valid())
        {
            if (const Entity *ref_entity = _context.world.entities().find(ref_info->render_entity))
            {
                return ref_entity->position_world();
            }
        }

        return _context.scenario_config.system_center;
    }

    bool PredictionFrameContextBuilder::sample_prediction_inertial_state(
            const std::vector<orbitsim::TrajectorySample> &trajectory,
            const double query_time_s,
            orbitsim::State &out_state) const
    {
        return PredictionTrajectorySampler::sample_inertial_state(trajectory, query_time_s, out_state);
    }

    orbitsim::SpacecraftStateLookup PredictionFrameContextBuilder::build_prediction_player_lookup() const
    {
        const OrbitPredictionCache *player_cache = effective_cache(player_track());
        if (!player_cache)
        {
            return nullptr;
        }

        const std::vector<orbitsim::TrajectorySegment> *trajectory_segments = nullptr;
        if (!player_cache->solver.trajectory_segments_inertial_planned.empty())
        {
            trajectory_segments = &player_cache->solver.trajectory_segments_inertial_planned;
        }
        else if (!player_cache->solver.resolved_trajectory_segments_inertial().empty())
        {
            trajectory_segments = &player_cache->solver.resolved_trajectory_segments_inertial();
        }
        if (!trajectory_segments)
        {
            return nullptr;
        }

        return PredictionTrajectorySampler::build_player_lookup(*trajectory_segments);
    }

    PredictionFrameResolverContext PredictionFrameContextBuilder::build_prediction_frame_resolver_context() const
    {
        PredictionFrameResolverContext context{};
        if (_context.prediction)
        {
            context.frame_selection = _context.prediction->state().frame_selection;
        }
        context.system_center = _context.scenario_config.system_center;
        context.world_reference_body_world = prediction_world_reference_body_world();
        context.current_sim_time_s =
                _context.orbit.scenario_owner() ? _context.orbit.scenario_owner()->sim.time_s() : _context.fixed_time_s;
        context.softening_length_m =
                _context.orbit.scenario_owner() ? _context.orbit.scenario_owner()->sim.config().softening_length_m : 0.0;
        context.maneuver_nodes_enabled = _context.maneuver.settings().nodes_enabled;
        context.maneuver_plan_has_nodes = !_context.maneuver.plan().nodes.empty();
        context.maneuver_axis_drag_active =
                _context.maneuver.gizmo_interaction().state == ManeuverGizmoInteraction::State::DragAxis;
        context.maneuver_drag_display_reference_time_s =
                _context.maneuver.gizmo_interaction().drag_display_reference_time_s;
        if (_context.orbit.scenario_owner())
        {
            if (const CelestialBodyInfo *world_ref = _context.orbit.scenario_owner()->world_reference_body())
            {
                context.world_reference_body_id = world_ref->sim_id;
            }
            context.world_reference_sim_body = _context.orbit.scenario_owner()->world_reference_sim_body();
        }
        context.player_lookup = build_prediction_player_lookup();
        return context;
    }

    orbitsim::TrajectoryFrameSpec PredictionFrameContextBuilder::resolve_prediction_display_frame_spec(
            const OrbitPredictionCache &cache,
            const double display_time_s) const
    {
        return PredictionFrameResolver::resolve_display_frame_spec(
                build_prediction_frame_resolver_context(),
                cache,
                display_time_s);
    }

    orbitsim::BodyId PredictionFrameContextBuilder::select_prediction_primary_body_id(
            const std::vector<orbitsim::MassiveBody> &bodies,
            const OrbitPredictionCache *cache,
            const orbitsim::Vec3 &query_pos_m,
            const double query_time_s,
            const orbitsim::BodyId preferred_body_id) const
    {
        return PredictionFrameResolver::select_primary_body_id(
                build_prediction_frame_resolver_context(),
                bodies,
                cache,
                query_pos_m,
                query_time_s,
                preferred_body_id);
    }

    orbitsim::BodyId PredictionFrameContextBuilder::resolve_prediction_analysis_body_id(
            const OrbitPredictionCache &cache,
            const PredictionSubjectKey key,
            const double query_time_s,
            const orbitsim::BodyId preferred_body_id) const
    {
        if (_context.prediction &&
            _context.prediction->state().analysis_selection.spec.mode == PredictionAnalysisMode::FixedBodyBCI &&
            _context.prediction->state().analysis_selection.spec.fixed_body_id != orbitsim::kInvalidBodyId)
        {
            return _context.prediction->state().analysis_selection.spec.fixed_body_id;
        }

        const auto &bodies = cache.solver.resolved_massive_bodies();
        const auto &base_segments = cache.solver.resolved_trajectory_segments_inertial();
        const auto &base_samples = cache.solver.resolved_trajectory_inertial();
        if (bodies.empty())
        {
            return orbitsim::kInvalidBodyId;
        }

        orbitsim::State query_state{};
        if (!PredictionTrajectorySampler::sample_inertial_state(base_segments,
                                                                query_time_s,
                                                                query_state,
                                                                TrajectoryBoundarySide::ContinuousPositionOnly) &&
            !sample_prediction_inertial_state(base_samples, query_time_s, query_state))
        {
            return orbitsim::kInvalidBodyId;
        }

        std::vector<orbitsim::MassiveBody> candidates;
        candidates.reserve(bodies.size());
        for (const orbitsim::MassiveBody &body : bodies)
        {
            if (key.kind == PredictionSubjectKind::Celestial &&
                static_cast<uint32_t>(body.id) == key.value)
            {
                continue;
            }
            candidates.push_back(body);
        }
        if (candidates.empty())
        {
            return orbitsim::kInvalidBodyId;
        }

        orbitsim::BodyId effective_preferred_body_id = preferred_body_id;
        if (effective_preferred_body_id == orbitsim::kInvalidBodyId)
        {
            if (const PredictionTrackState *track = find_track(key))
            {
                effective_preferred_body_id = track->auto_primary_body_id;
            }
        }
        if (effective_preferred_body_id == orbitsim::kInvalidBodyId &&
            cache.display.resolved_frame_spec_valid &&
            cache.display.resolved_frame_spec.primary_body_id != orbitsim::kInvalidBodyId)
        {
            effective_preferred_body_id = cache.display.resolved_frame_spec.primary_body_id;
        }
        if (effective_preferred_body_id == orbitsim::kInvalidBodyId &&
            _context.prediction &&
            _context.prediction->state().frame_selection.spec.primary_body_id != orbitsim::kInvalidBodyId)
        {
            effective_preferred_body_id = _context.prediction->state().frame_selection.spec.primary_body_id;
        }

        return select_prediction_primary_body_id(
                candidates,
                &cache,
                query_state.position_m,
                query_time_s,
                effective_preferred_body_id);
    }
} // namespace Game
