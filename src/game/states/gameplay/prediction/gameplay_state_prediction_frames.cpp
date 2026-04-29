#include "game/states/gameplay/gameplay_state.h"
#include "game/states/gameplay/prediction/prediction_frame_controller.h"
#include "game/states/gameplay/prediction/prediction_frame_resolver.h"
#include "game/states/gameplay/prediction/prediction_trajectory_sampler.h"
#include "game/states/gameplay/prediction/runtime/gameplay_state_prediction_runtime_internal.h"

#include "orbitsim/frame_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>

namespace Game
{
    namespace
    {
        bool same_analysis_spec(const PredictionAnalysisSpec &a, const PredictionAnalysisSpec &b)
        {
            return a.mode == b.mode && a.fixed_body_id == b.fixed_body_id;
        }
    } // namespace

    const CelestialBodyInfo *GameplayState::find_celestial_body_info(const orbitsim::BodyId body_id) const
    {
        if (!_orbitsim || body_id == orbitsim::kInvalidBodyId)
        {
            return nullptr;
        }

        for (const CelestialBodyInfo &body : _orbitsim->bodies)
        {
            if (body.sim_id == body_id)
            {
                return &body;
            }
        }

        return nullptr;
    }

    const orbitsim::MassiveBody *GameplayState::find_massive_body(const std::vector<orbitsim::MassiveBody> &bodies,
                                                                  const orbitsim::BodyId body_id) const
    {
        return PredictionFrameResolver::find_massive_body(bodies, body_id);
    }

    bool GameplayState::prediction_frame_is_lagrange_sensitive(const orbitsim::TrajectoryFrameSpec &spec) const
    {
        return PredictionFrameResolver::is_lagrange_sensitive(spec);
    }

    double GameplayState::resolve_prediction_display_reference_time_s(const OrbitPredictionCache &cache,
                                                                      const double display_time_s) const
    {
        return PredictionFrameResolver::resolve_display_reference_time_s(
                build_prediction_frame_resolver_context(),
                cache,
                display_time_s);
    }

    orbitsim::BodyId GameplayState::select_prediction_primary_body_id(const std::vector<orbitsim::MassiveBody> &bodies,
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

    WorldVec3 GameplayState::prediction_body_world_position(const orbitsim::BodyId body_id,
                                                            const OrbitPredictionCache *cache,
                                                            double query_time_s) const
    {
        if (body_id == orbitsim::kInvalidBodyId)
        {
            return prediction_world_reference_body_world();
        }

        if (const CelestialBodyInfo *body_info = find_celestial_body_info(body_id))
        {
            if (body_info->render_entity.is_valid())
            {
                if (const Entity *entity = _world.entities().find(body_info->render_entity))
                {
                    return entity->position_world();
                }
            }
        }

        const double sample_time_s =
                std::isfinite(query_time_s) ? query_time_s : (_orbitsim ? _orbitsim->sim.time_s() : 0.0);
        const OrbitPredictionCache *position_cache = cache;
        if (!position_cache)
        {
            position_cache = effective_prediction_cache(player_prediction_track());
        }

        const orbitsim::MassiveBody *body = nullptr;
        if (position_cache)
        {
            body = find_massive_body(position_cache->resolved_massive_bodies(), body_id);
        }
        if (!body && _orbitsim)
        {
            body = _orbitsim->sim.body_by_id(body_id);
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
        if (_orbitsim)
        {
            if (const CelestialBodyInfo *world_ref_info = _orbitsim->world_reference_body())
            {
                if (position_cache)
                {
                    if (const orbitsim::MassiveBody *world_ref_body =
                                find_massive_body(position_cache->resolved_massive_bodies(), world_ref_info->sim_id))
                    {
                        world_ref_state = body_state_at(*world_ref_body);
                        have_world_ref_state = true;
                    }
                }
                if (!have_world_ref_state)
                {
                    if (const orbitsim::MassiveBody *world_ref_sim = _orbitsim->world_reference_sim_body())
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

    bool GameplayState::sample_prediction_inertial_state(const std::vector<orbitsim::TrajectorySample> &trajectory,
        const double query_time_s,
        orbitsim::State &out_state) const
    {
        return PredictionTrajectorySampler::sample_inertial_state(trajectory, query_time_s, out_state);
    }

    orbitsim::SpacecraftStateLookup GameplayState::build_prediction_player_lookup() const
    {
        const OrbitPredictionCache *player_cache = effective_prediction_cache(player_prediction_track());
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

    PredictionFrameResolverContext GameplayState::build_prediction_frame_resolver_context() const
    {
        PredictionFrameResolverContext context{};
        context.frame_selection = _prediction.frame_selection;
        context.system_center = _scenario_config.system_center;
        context.world_reference_body_world = prediction_world_reference_body_world();
        context.current_sim_time_s = _orbitsim ? _orbitsim->sim.time_s() : _fixed_time_s;
        context.softening_length_m = _orbitsim ? _orbitsim->sim.config().softening_length_m : 0.0;
        context.maneuver_nodes_enabled = _maneuver_nodes_enabled;
        context.maneuver_plan_has_nodes = !_maneuver_state.nodes.empty();
        context.maneuver_axis_drag_active =
                _maneuver_gizmo_interaction.state == ManeuverGizmoInteraction::State::DragAxis;
        context.maneuver_drag_display_reference_time_s =
                _maneuver_gizmo_interaction.drag_display_reference_time_s;
        if (_orbitsim)
        {
            if (const CelestialBodyInfo *world_ref = _orbitsim->world_reference_body())
            {
                context.world_reference_body_id = world_ref->sim_id;
            }
            context.world_reference_sim_body = _orbitsim->world_reference_sim_body();
        }
        context.player_lookup = build_prediction_player_lookup();
        return context;
    }

    PredictionFrameControllerContext GameplayState::build_prediction_frame_controller_context() const
    {
        PredictionFrameControllerContext context{};
        context.derived_service = const_cast<OrbitPredictionDerivedService *>(&_prediction.derived_service);
        context.selection = _prediction.selection;
        context.display_frame_revision = _prediction.display_frame_revision;
        context.sim_config = _orbitsim ? _orbitsim->sim.config() : orbitsim::GameSimulation::Config{};
        context.resolve_display_frame_spec = [this](const OrbitPredictionCache &cache, const double display_time_s) {
            return resolve_prediction_display_frame_spec(cache, display_time_s);
        };
        context.resolve_analysis_body_id = [this](const OrbitPredictionCache &cache,
                                                  const PredictionSubjectKey key,
                                                  const double query_time_s,
                                                  const orbitsim::BodyId preferred_body_id) {
            return resolve_prediction_analysis_body_id(cache, key, query_time_s, preferred_body_id);
        };
        context.get_subject_world_state = [this](const PredictionSubjectKey key,
                                                 WorldVec3 &out_pos_world,
                                                 glm::dvec3 &out_vel_world,
                                                 glm::vec3 &out_vel_local) {
            return get_prediction_subject_world_state(key, out_pos_world, out_vel_world, out_vel_local);
        };
        context.subject_is_player = [this](const PredictionSubjectKey key) {
            return prediction_subject_is_player(key);
        };
        context.player_track = [this]() -> const PredictionTrackState * {
            return player_prediction_track();
        };
        context.effective_cache = [this](const PredictionTrackState *track) -> const OrbitPredictionCache * {
            return effective_prediction_cache(track);
        };
        context.current_sim_time_s = [this]() {
            return current_sim_time_s();
        };
        return context;
    }

    orbitsim::TrajectoryFrameSpec GameplayState::resolve_prediction_display_frame_spec(const OrbitPredictionCache &cache,
                                                                                       double display_time_s) const
    {
        return PredictionFrameResolver::resolve_display_frame_spec(
                build_prediction_frame_resolver_context(),
                cache,
                display_time_s);
    }

    orbitsim::TrajectoryFrameSpec GameplayState::default_prediction_frame_spec() const
    {
        if (_orbitsim)
        {
            if (const CelestialBodyInfo *world_ref = _orbitsim->world_reference_body())
            {
                return orbitsim::TrajectoryFrameSpec::body_centered_inertial(world_ref->sim_id);
            }
        }

        return orbitsim::TrajectoryFrameSpec::inertial();
    }

    void GameplayState::rebuild_prediction_analysis_options()
    {
        std::vector<PredictionAnalysisOption> options;
        options.push_back(PredictionAnalysisOption{
                .spec = {},
                .label = "Auto Primary (BCI)",
        });

        if (_orbitsim)
        {
            for (const CelestialBodyInfo &body : _orbitsim->bodies)
            {
                options.push_back(PredictionAnalysisOption{
                        .spec = PredictionAnalysisSpec{
                                .mode = PredictionAnalysisMode::FixedBodyBCI,
                                .fixed_body_id = body.sim_id,
                        },
                        .label = body.name + " BCI",
                });
            }
        }

        _prediction.analysis_selection.options = std::move(options);
        if (_prediction.analysis_selection.selected_index < 0)
        {
            _prediction.analysis_selection.spec = {};
        }

        int selected_index = -1;
        for (std::size_t i = 0; i < _prediction.analysis_selection.options.size(); ++i)
        {
            if (same_analysis_spec(_prediction.analysis_selection.options[i].spec, _prediction.analysis_selection.spec))
            {
                selected_index = static_cast<int>(i);
                break;
            }
        }

        if (selected_index < 0)
        {
            _prediction.analysis_selection.spec = {};
            selected_index = 0;
        }

        _prediction.analysis_selection.selected_index = selected_index;
    }

    bool GameplayState::set_prediction_analysis_spec(const PredictionAnalysisSpec &spec)
    {
        if (same_analysis_spec(_prediction.analysis_selection.spec, spec))
        {
            return false;
        }

        _prediction.analysis_selection.spec = spec;
        refresh_all_prediction_derived_caches();
        sync_prediction_dirty_flag();
        return true;
    }

    bool GameplayState::set_prediction_frame_spec(const orbitsim::TrajectoryFrameSpec &spec)
    {
        if (PredictionFrameResolver::same_frame_spec(_prediction.frame_selection.spec, spec))
        {
            return false;
        }

        _prediction.frame_selection.spec = spec;
        ++_prediction.display_frame_revision;
        refresh_all_prediction_derived_caches();
        sync_prediction_dirty_flag();
        return true;
    }

    void GameplayState::rebuild_prediction_frame_options()
    {
        std::vector<PredictionFrameOption> options;
        options.push_back(PredictionFrameOption{
                .spec = orbitsim::TrajectoryFrameSpec::inertial(),
                .label = "Inertial (Barycentric)",
        });

        if (_orbitsim)
        {
            const double display_time_s = _orbitsim->sim.time_s();
            const auto player_lookup = build_prediction_player_lookup();
            const CelestialBodyInfo *world_ref = _orbitsim->world_reference_body();

            for (const CelestialBodyInfo &body : _orbitsim->bodies)
            {
                options.push_back(PredictionFrameOption{
                        .spec = orbitsim::TrajectoryFrameSpec::body_centered_inertial(body.sim_id),
                        .label = body.name + " BCI",
                });

                if (const orbitsim::MassiveBody *sim_body = _orbitsim->sim.body_by_id(body.sim_id))
                {
                    const std::optional<orbitsim::RotatingFrame> body_fixed =
                            orbitsim::make_body_fixed_frame_at(orbitsim::CelestialEphemeris{}, *sim_body, display_time_s);
                    if (body_fixed.has_value())
                    {
                        options.push_back(PredictionFrameOption{
                                .spec = orbitsim::TrajectoryFrameSpec::body_fixed(body.sim_id),
                                .label = body.name + " Fixed",
                        });
                    }
                }
            }

            if (world_ref)
            {
                for (const CelestialBodyInfo &body : _orbitsim->bodies)
                {
                    if (body.sim_id == world_ref->sim_id)
                    {
                        continue;
                    }

                    options.push_back(PredictionFrameOption{
                            .spec = orbitsim::TrajectoryFrameSpec::synodic(world_ref->sim_id, body.sim_id),
                            .label = world_ref->name + "-" + body.name + " Synodic",
                    });
                }
            }

            if (player_lookup)
            {
                options.push_back(PredictionFrameOption{
                        .spec = orbitsim::TrajectoryFrameSpec::lvlh(
                                PredictionTrajectorySampler::kPlayerDisplayTargetSpacecraftId),
                        .label = "Player LVLH",
                });
            }
        }

        _prediction.frame_selection.options = std::move(options);

        if (_prediction.frame_selection.selected_index < 0)
        {
            _prediction.frame_selection.spec = default_prediction_frame_spec();
        }

        int selected_index = -1;
        for (std::size_t i = 0; i < _prediction.frame_selection.options.size(); ++i)
        {
            if (PredictionFrameResolver::same_frame_spec(_prediction.frame_selection.options[i].spec,
                                                         _prediction.frame_selection.spec))
            {
                selected_index = static_cast<int>(i);
                break;
            }
        }

        if (selected_index < 0)
        {
            _prediction.frame_selection.spec = default_prediction_frame_spec();
            for (std::size_t i = 0; i < _prediction.frame_selection.options.size(); ++i)
            {
                if (PredictionFrameResolver::same_frame_spec(_prediction.frame_selection.options[i].spec,
                                                             _prediction.frame_selection.spec))
                {
                    selected_index = static_cast<int>(i);
                    break;
                }
            }
        }

        _prediction.frame_selection.selected_index = selected_index;
    }

    orbitsim::BodyId GameplayState::resolve_prediction_analysis_body_id(const OrbitPredictionCache &cache,
                                                                        const PredictionSubjectKey key,
                                                                        const double query_time_s,
                                                                        const orbitsim::BodyId preferred_body_id) const
    {
        if (_prediction.analysis_selection.spec.mode == PredictionAnalysisMode::FixedBodyBCI &&
            _prediction.analysis_selection.spec.fixed_body_id != orbitsim::kInvalidBodyId)
        {
            return _prediction.analysis_selection.spec.fixed_body_id;
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
            if (const PredictionTrackState *track = find_prediction_track(key))
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
            _prediction.frame_selection.spec.primary_body_id != orbitsim::kInvalidBodyId)
        {
            effective_preferred_body_id = _prediction.frame_selection.spec.primary_body_id;
        }

        return select_prediction_primary_body_id(
                candidates,
                &cache,
                query_state.position_m,
                query_time_s,
                effective_preferred_body_id);
    }

    bool GameplayState::build_prediction_display_frame(const OrbitPredictionCache &cache,
                                                       orbitsim::RotatingFrame &out_frame,
                                                       double display_time_s) const
    {
        return PredictionFrameResolver::build_display_frame(
                build_prediction_frame_resolver_context(),
                cache,
                out_frame,
                display_time_s);
    }

    bool GameplayState::build_prediction_display_transform(const OrbitPredictionCache &cache,
                                                           WorldVec3 &out_origin_world,
                                                           glm::dmat3 &out_frame_to_world,
                                                           double display_time_s) const
    {
        return PredictionFrameResolver::build_display_transform(
                build_prediction_frame_resolver_context(),
                cache,
                out_origin_world,
                out_frame_to_world,
                display_time_s);
    }

    WorldVec3 GameplayState::prediction_world_reference_body_world() const
    {
        if (!_orbitsim)
        {
            return _scenario_config.system_center;
        }

        const CelestialBodyInfo *ref_info = _orbitsim->world_reference_body();
        if (ref_info && ref_info->render_entity.is_valid())
        {
            if (const Entity *ref_entity = _world.entities().find(ref_info->render_entity))
            {
                return ref_entity->position_world();
            }
        }

        return _scenario_config.system_center;
    }

    WorldVec3 GameplayState::prediction_sample_position_world(const OrbitPredictionCache &cache,
                                                              const orbitsim::TrajectorySample &sample,
                                                              double display_time_s) const
    {
        return PredictionFrameResolver::sample_position_world(
                build_prediction_frame_resolver_context(),
                cache,
                sample,
                display_time_s);
    }

    WorldVec3 GameplayState::prediction_sample_hermite_world(const OrbitPredictionCache &cache,
                                                             const orbitsim::TrajectorySample &a,
                                                             const orbitsim::TrajectorySample &b,
                                                             const double t_s,
                                                             double display_time_s) const
    {
        return PredictionFrameResolver::sample_hermite_world(
                build_prediction_frame_resolver_context(),
                cache,
                a,
                b,
                t_s,
                display_time_s);
    }

    WorldVec3 GameplayState::prediction_frame_origin_world(const OrbitPredictionCache &cache,
                                                           double display_time_s) const
    {
        return PredictionFrameResolver::frame_origin_world(
                build_prediction_frame_resolver_context(),
                cache,
                display_time_s);
    }

    void GameplayState::mark_prediction_derived_request_submitted(
            PredictionTrackState &track,
            const OrbitPredictionDerivedService::Request &request)
    {
        PredictionFrameController::mark_derived_request_submitted(track, request);
    }

    bool GameplayState::request_prediction_derived_refresh(PredictionTrackState &track, double display_time_s)
    {
        return PredictionFrameController::request_derived_refresh(
                build_prediction_frame_controller_context(),
                track,
                display_time_s);
    }

    bool GameplayState::prediction_track_has_current_derived_cache(const PredictionTrackState &track,
                                                                   double display_time_s) const
    {
        return PredictionFrameController::has_current_derived_cache(
                build_prediction_frame_controller_context(),
                track,
                display_time_s);
    }

    void GameplayState::refresh_prediction_derived_cache(PredictionTrackState &track,
                                                         double display_time_s)
    {
        PredictionFrameController::refresh_derived_cache(
                build_prediction_frame_controller_context(),
                track,
                display_time_s);
    }

    void GameplayState::refresh_all_prediction_derived_caches()
    {
        rebuild_prediction_frame_options();
        rebuild_prediction_analysis_options();
        _prediction.derived_service.reset();
        for (PredictionTrackState &track : _prediction.tracks)
        {
            PredictionFrameController::reset_track_derived_state(track);
            (void) request_prediction_derived_refresh(track);
        }
    }

    WorldVec3 GameplayState::prediction_reference_body_world() const
    {
        return prediction_world_reference_body_world();
    }

} // namespace Game
