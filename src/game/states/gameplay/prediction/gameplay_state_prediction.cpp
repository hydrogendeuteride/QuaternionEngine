#include "game/states/gameplay/gameplay_state.h"

#include "game/component/ship_controller.h"
#include "game/orbit/orbit_prediction_tuning.h"
#include "physics/physics_context.h"
#include "physics/physics_world.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <utility>

namespace Game
{
    PredictionSubjectKey GameplayState::player_prediction_subject_key() const
    {
        const EntityId player_eid = player_entity();
        return player_eid.is_valid()
                   ? PredictionSubjectKey{PredictionSubjectKind::Orbiter, player_eid.value}
                   : PredictionSubjectKey{};
    }

    std::vector<PredictionSubjectDescriptor> GameplayState::build_prediction_subject_descriptors() const
    {
        std::vector<PredictionSubjectDescriptor> subjects;
        subjects.reserve(_orbiters.size() + (_orbitsim ? _orbitsim->bodies.size() : 0u));

        for (const OrbiterInfo &orbiter : _orbiters)
        {
            if (!orbiter.entity.is_valid())
            {
                continue;
            }

            PredictionSubjectKey key{};
            key.kind = PredictionSubjectKind::Orbiter;
            key.value = orbiter.entity.value;

            subjects.push_back(PredictionSubjectDescriptor{
                    .key = key,
                    .label = orbiter.name,
                    .supports_maneuvers = orbiter.is_player ||
                                           (orbiter.formation_hold_enabled && !orbiter.formation_leader_name.empty()),
                    .is_celestial = false,
                    .orbit_rgb = prediction_subject_orbit_rgb(key),
            });
        }

        if (_orbitsim)
        {
            const CelestialBodyInfo *ref_info = _orbitsim->world_reference_body();
            for (const CelestialBodyInfo &body : _orbitsim->bodies)
            {
                if (ref_info && body.sim_id == ref_info->sim_id)
                {
                    continue;
                }

                PredictionSubjectKey key{};
                key.kind = PredictionSubjectKind::Celestial;
                key.value = static_cast<uint32_t>(body.sim_id);

                subjects.push_back(PredictionSubjectDescriptor{
                        .key = key,
                        .label = body.name,
                        .supports_maneuvers = false,
                        .is_celestial = true,
                        .orbit_rgb = prediction_subject_orbit_rgb(key),
                });
            }
        }

        return subjects;
    }

    PredictionHostContext GameplayState::build_prediction_host_context(const GameStateContext *ctx) const
    {
        PredictionHostContext host{};
        host.orbital_scenario = _orbitsim.get();
        host.subjects = build_prediction_subject_descriptors();
        host.player_subject = player_prediction_subject_key();
        host.current_sim_time_s = current_sim_time_s();
        host.last_sim_step_dt_s = _last_sim_step_dt_s;
        host.fixed_delta_time_s = ctx ? ctx->fixed_delta_time() : 0.0f;
        host.interpolation_alpha = ctx ? std::clamp(ctx->interpolation_alpha(), 0.0f, 1.0f) : 0.0f;
        host.frame_delta_time_s = ctx ? ctx->delta_time() : 0.0f;
        host.debug_draw_enabled = _debug_draw_enabled;

        host.maneuver.plan = &_maneuver.plan();
        host.maneuver.edit_preview = &_maneuver.edit_preview();
        host.maneuver.gizmo_interaction = &_maneuver.gizmo_interaction();
        host.maneuver.plan_horizon = _maneuver.settings().plan_horizon;
        host.maneuver.plan_windows = _maneuver.settings().plan_windows;
        host.maneuver.nodes_enabled = _maneuver.settings().nodes_enabled;
        host.maneuver.live_preview_active = maneuver_live_preview_active(true);
        host.maneuver.edit_in_progress =
                _maneuver.gizmo_interaction().state == ManeuverGizmoInteraction::State::DragAxis ||
                _maneuver.edit_preview().state != ManeuverNodeEditPreview::State::Idle;
        host.maneuver.revision = _maneuver.revision();
        host.maneuver.signature = current_maneuver_plan_signature();
        host.maneuver.active_preview_anchor_node_id = active_maneuver_preview_anchor_node_id();

        host.reference_body_world = [this]() { return prediction_world_reference_body_world(); };
        host.get_subject_world_state = [this](const PredictionSubjectKey key,
                                              WorldVec3 &out_pos_world,
                                              glm::dvec3 &out_vel_world,
                                              glm::vec3 &out_vel_local) {
            return get_prediction_subject_world_state(key, out_pos_world, out_vel_world, out_vel_local);
        };
        host.render_subject_position_world = [this](const PredictionSubjectKey key, const float alpha_f) {
            if (key.kind == PredictionSubjectKind::Orbiter)
            {
                if (const Entity *entity = _world.entities().find(EntityId{key.value}))
                {
                    return entity->get_render_physics_center_of_mass_world(alpha_f);
                }
            }

            WorldVec3 pos_world{0.0, 0.0, 0.0};
            glm::dvec3 vel_world{0.0};
            glm::vec3 vel_local{0.0f};
            (void) get_prediction_subject_world_state(key, pos_world, vel_world, vel_local);
            return pos_world;
        };
        host.future_window_s = [this](const PredictionSubjectKey key) {
            return prediction_future_window_s(key);
        };
        host.required_window_s = [this](const PredictionTrackState &track,
                                        const double now_s,
                                        const bool with_maneuvers) {
            return prediction_required_window_s(track, now_s, with_maneuvers);
        };
        host.preview_exact_window_s = [this](const PredictionTrackState &track,
                                             const double now_s,
                                             const bool with_maneuvers) {
            return prediction_preview_exact_window_s(track, now_s, with_maneuvers);
        };
        host.refresh_preview_anchor = [this](PredictionTrackState &track,
                                             const double now_s,
                                             const bool with_maneuvers) {
            refresh_prediction_preview_anchor(track, now_s, with_maneuvers);
        };
        host.subject_is_player = [this](const PredictionSubjectKey key) {
            return prediction_subject_is_player(key);
        };
        host.subject_thrust_applied_this_tick = [this](const PredictionSubjectKey key) {
            return prediction_subject_thrust_applied_this_tick(key);
        };
        host.resolve_maneuver_node_primary_body_id = [this](const ManeuverNode &node, const double query_time_s) {
            return resolve_maneuver_node_primary_body_id(node, query_time_s);
        };
        host.resolve_display_frame_spec = [this](const OrbitPredictionCache &cache, const double display_time_s) {
            return resolve_prediction_display_frame_spec(cache, display_time_s);
        };
        host.resolve_analysis_body_id = [this](const OrbitPredictionCache &cache,
                                               const PredictionSubjectKey key,
                                               const double query_time_s,
                                               const orbitsim::BodyId preferred_body_id) {
            return resolve_prediction_analysis_body_id(cache, key, query_time_s, preferred_body_id);
        };
        return host;
    }

    bool GameplayState::get_player_world_state(WorldVec3 &out_pos_world,
                                               glm::dvec3 &out_vel_world,
                                               glm::vec3 &out_vel_local) const
    {
        // Resolve the player orbiter into the common world-state format.
        const OrbiterInfo *player = find_player_orbiter();
        return player && get_orbiter_world_state(*player, out_pos_world, out_vel_world, out_vel_local);
    }

    bool GameplayState::get_orbiter_world_state(const OrbiterInfo &orbiter,
                                                WorldVec3 &out_pos_world,
                                                glm::dvec3 &out_vel_world,
                                                glm::vec3 &out_vel_local) const
    {
        // Look up the live entity that currently represents this orbiter.
        if (!orbiter.entity.is_valid())
        {
            return false;
        }

        const Entity *entity = _world.entities().find(orbiter.entity);
        if (!entity)
        {
            return false;
        }

        out_pos_world = entity->physics_center_of_mass_world();
        out_vel_world = glm::dvec3(0.0);
        out_vel_local = glm::vec3(0.0f);

        // Prefer the authoritative orbit-sim spacecraft state whenever this orbiter is currently on rails.
        if (_orbitsim)
        {
            const orbitsim::MassiveBody *ref_sim = _orbitsim->world_reference_sim_body();
            if (orbiter.rails.active() && ref_sim)
            {
                if (const orbitsim::Spacecraft *sc = _orbitsim->sim.spacecraft_by_id(orbiter.rails.sc_id))
                {
                    out_pos_world = _scenario_config.system_center +
                            WorldVec3(sc->state.position_m - ref_sim->state.position_m);
                    out_vel_world = sc->state.velocity_mps - ref_sim->state.velocity_mps;
                    out_vel_local = glm::vec3(0.0f);
                    return true;
                }
            }
        }

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
        // Outside rails warp, derive current velocity from the physics body when available.
        if (_physics && _physics_context && entity->has_physics())
        {
            const Physics::BodyId body_id{entity->physics_body_value()};
            if (_physics->is_body_valid(body_id))
            {
                const glm::quat rotation = _physics->get_rotation(body_id);
                const WorldVec3 body_origin_world =
                        local_to_world_d(_physics->get_position(body_id), _physics_context->origin_world());
                out_pos_world = entity->physics_center_of_mass_world(body_origin_world, rotation);
                out_vel_local = _physics->get_linear_velocity(body_id);
                out_vel_world = _physics_context->velocity_origin_world() + glm::dvec3(out_vel_local);
            }
        }
#endif

        return true;
    }

    bool GameplayState::get_prediction_subject_world_state(PredictionSubjectKey key,
                                                           WorldVec3 &out_pos_world,
                                                           glm::dvec3 &out_vel_world,
                                                           glm::vec3 &out_vel_local) const
    {
        // Normalize any prediction subject into a shared world-space state snapshot.
        out_pos_world = WorldVec3(0.0);
        out_vel_world = glm::dvec3(0.0);
        out_vel_local = glm::vec3(0.0f);

        if (!key.valid())
        {
            return false;
        }

        if (key.kind == PredictionSubjectKind::Orbiter)
        {
            // Orbiters resolve through the gameplay entity/runtime state.
            const OrbiterInfo *orbiter = find_orbiter(EntityId{key.value});
            return orbiter && get_orbiter_world_state(*orbiter, out_pos_world, out_vel_world, out_vel_local);
        }

        if (!_orbitsim)
        {
            return false;
        }

        const CelestialBodyInfo *body_info = nullptr;
        for (const CelestialBodyInfo &candidate : _orbitsim->bodies)
        {
            if (static_cast<uint32_t>(candidate.sim_id) == key.value)
            {
                body_info = &candidate;
                break;
            }
        }

        const orbitsim::MassiveBody *body = body_info ? _orbitsim->sim.body_by_id(body_info->sim_id) : nullptr;
        const orbitsim::MassiveBody *ref_sim = _orbitsim->world_reference_sim_body();
        if (!body || !ref_sim)
        {
            return false;
        }

        // Celestials are expressed relative to the active reference body frame.
        out_pos_world = _scenario_config.system_center + WorldVec3(body->state.position_m - ref_sim->state.position_m);
        out_vel_world = body->state.velocity_mps - ref_sim->state.velocity_mps;
        if (body_info && body_info->render_entity.is_valid())
        {
            if (const Entity *render_entity = _world.entities().find(body_info->render_entity))
            {
                out_pos_world = render_entity->position_world();
            }
        }

        return true;
    }

    void GameplayState::rebuild_prediction_subjects()
    {
        _prediction->sync_subjects(build_prediction_host_context());
    }

    std::vector<PredictionSubjectKey> GameplayState::collect_visible_prediction_subjects() const
    {
        return _prediction->collect_visible_subjects();
    }

    double GameplayState::prediction_future_window_s(const PredictionSubjectKey key) const
    {
        // Celestials and spacecraft use different minimum look-ahead windows.
        if (key.kind == PredictionSubjectKind::Celestial)
        {
            return std::max(0.0, _prediction->state().sampling_policy.celestial_min_window_s);
        }

        return std::max(0.0, _prediction->state().sampling_policy.orbiter_min_window_s);
    }

    double GameplayState::maneuver_plan_horizon_s() const
    {
        return std::max(OrbitPredictionTuning::kPostNodeCoverageMinS,
                        std::max(0.0, _maneuver.settings().plan_horizon.horizon_s));
    }

    uint64_t GameplayState::current_maneuver_plan_signature() const
    {
        const double plan_horizon_s = maneuver_plan_horizon_s();
        return _maneuver.plan_signature(plan_horizon_s);
    }

    PredictionTrackState *GameplayState::find_prediction_track(PredictionSubjectKey key)
    {
        return _prediction->find_track(key);
    }

    const PredictionTrackState *GameplayState::find_prediction_track(PredictionSubjectKey key) const
    {
        return _prediction->find_track(key);
    }

    PredictionTrackState *GameplayState::active_prediction_track()
    {
        return _prediction->active_track();
    }

    const PredictionTrackState *GameplayState::active_prediction_track() const
    {
        return _prediction->active_track();
    }

    PredictionTrackState *GameplayState::player_prediction_track()
    {
        return _prediction->player_track(player_prediction_subject_key());
    }

    const PredictionTrackState *GameplayState::player_prediction_track() const
    {
        return _prediction->player_track(player_prediction_subject_key());
    }

    OrbitPredictionCache *GameplayState::effective_prediction_cache(PredictionTrackState *track)
    {
        return _prediction->effective_cache(track);
    }

    const OrbitPredictionCache *GameplayState::effective_prediction_cache(const PredictionTrackState *track) const
    {
        return _prediction->effective_cache(track);
    }

    OrbitPredictionCache *GameplayState::player_prediction_cache()
    {
        return _prediction->player_cache(player_prediction_subject_key());
    }

    const OrbitPredictionCache *GameplayState::player_prediction_cache() const
    {
        return _prediction->player_cache(player_prediction_subject_key());
    }

    bool GameplayState::prediction_subject_is_player(PredictionSubjectKey key) const
    {
        // Only the player ship is treated as the maneuver-authoring subject.
        return key.kind == PredictionSubjectKind::Orbiter && key.value == player_entity().value;
    }

    bool GameplayState::prediction_subject_supports_maneuvers(PredictionSubjectKey key) const
    {
        // Maneuver planning applies to the player and formation followers that mirror the leader's plan.
        if (prediction_subject_is_player(key))
        {
            return true;
        }
        if (key.kind == PredictionSubjectKind::Orbiter)
        {
            const OrbiterInfo *orbiter = find_orbiter(EntityId(key.value));
            if (orbiter && orbiter->formation_hold_enabled && !orbiter->formation_leader_name.empty())
            {
                return true;
            }
        }
        return false;
    }

    std::string GameplayState::prediction_subject_label(PredictionSubjectKey key) const
    {
        // Build a UI label that reflects subject type and player ownership.
        const PredictionTrackState *track = find_prediction_track(key);
        if (!track)
        {
            return {};
        }

        if (key.kind == PredictionSubjectKind::Orbiter && prediction_subject_is_player(key))
        {
            return track->label + " (player)";
        }
        if (key.kind == PredictionSubjectKind::Celestial)
        {
            return track->label + " (celestial)";
        }
        return track->label;
    }

    glm::vec3 GameplayState::prediction_subject_orbit_rgb(const PredictionSubjectKey key) const
    {
        static constexpr std::array<glm::vec3, 4> kOrbiterPalette{{
                {1.00f, 0.25f, 0.25f},
                {0.25f, 0.52f, 1.00f},
                {0.62f, 0.65f, 0.70f},
                {0.28f, 0.88f, 0.38f},
        }};
        static constexpr glm::vec3 kAnchorOrbiterColor{0.70f, 0.35f, 1.00f};
        static constexpr glm::vec3 kDefaultCelestialColor{0.80f, 0.82f, 0.86f};

        if (!key.valid())
        {
            return kDefaultCelestialColor;
        }

        if (key.kind == PredictionSubjectKind::Celestial)
        {
            const CelestialBodyInfo *body_info = find_celestial_body_info(static_cast<orbitsim::BodyId>(key.value));
            if (!body_info)
            {
                return kDefaultCelestialColor;
            }

            for (const ScenarioConfig::CelestialDef &body_def : _scenario_config.celestials)
            {
                if (body_def.name == body_info->name)
                {
                    return body_def.has_prediction_orbit_color
                               ? body_def.prediction_orbit_color
                               : kDefaultCelestialColor;
                }
            }

            return kDefaultCelestialColor;
        }

        if (prediction_subject_is_player(key))
        {
            return kAnchorOrbiterColor;
        }

        const OrbiterInfo *orbiter = find_orbiter(EntityId{key.value});
        if (orbiter)
        {
            for (const ScenarioConfig::OrbiterDef &orbiter_def : _scenario_config.orbiters)
            {
                if (orbiter_def.name == orbiter->name && orbiter_def.has_prediction_orbit_color)
                {
                    return orbiter_def.prediction_orbit_color;
                }
            }
        }

        std::size_t orbiter_palette_index = 0;
        for (const OrbiterInfo &orbiter : _orbiters)
        {
            if (!orbiter.entity.is_valid() || orbiter.is_player)
            {
                continue;
            }

            if (orbiter.entity.value == key.value)
            {
                return kOrbiterPalette[orbiter_palette_index % kOrbiterPalette.size()];
            }
            ++orbiter_palette_index;
        }

        return kOrbiterPalette[0];
    }

    bool GameplayState::prediction_subject_thrust_applied_this_tick(PredictionSubjectKey key) const
    {
        // Thrust-triggered refresh only matters for the actively piloted subject.
        if (!prediction_subject_is_player(key))
        {
            return false;
        }

        // Rails warp uses a separate thrust path from the physics-driven ship controller.
        if (_rails_warp_active && _time_warp.mode == TimeWarpState::Mode::RailsWarp)
        {
            return _rails_thrust_applied_this_tick;
        }

        const EntityId player_eid = player_entity();
        if (!player_eid.is_valid())
        {
            return false;
        }

        const Entity *player = _world.entities().find(player_eid);
        if (!player)
        {
            return false;
        }

        const ShipController *sc = player->get_component<ShipController>();
        return sc && sc->thrust_applied_this_tick();
    }

} // namespace Game
