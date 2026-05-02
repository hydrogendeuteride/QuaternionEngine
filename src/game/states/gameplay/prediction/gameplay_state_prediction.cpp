#include "game/states/gameplay/gameplay_state.h"

#include "game/component/ship_controller.h"
#include "game/orbit/orbit_prediction_tuning.h"
#include "physics/physics_context.h"
#include "physics/physics_world.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace Game
{
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
        // Rebuild the subject list while preserving any reusable runtime cache/state.
        const PredictionSubjectKey old_active = _prediction.selection.active_subject;
        const std::vector<PredictionSubjectKey> old_overlays = _prediction.selection.overlay_subjects;
        std::vector<PredictionTrackState> old_tracks = std::move(_prediction.tracks);

        _prediction.tracks.clear();
        _prediction.groups.clear();

        const auto find_old_track = [&](PredictionSubjectKey key) -> PredictionTrackState * {
            auto it = std::find_if(old_tracks.begin(),
                                   old_tracks.end(),
                                   [key](const PredictionTrackState &track) { return track.key == key; });
            return (it != old_tracks.end()) ? &(*it) : nullptr;
        };

        // Register every live orbiter as a prediction track.
        for (const OrbiterInfo &orbiter : _orbiters)
        {
            if (!orbiter.entity.is_valid())
            {
                continue;
            }

            PredictionSubjectKey key{};
            key.kind = PredictionSubjectKind::Orbiter;
            key.value = orbiter.entity.value;

            PredictionTrackState track{};
            if (PredictionTrackState *old = find_old_track(key))
            {
                track = std::move(*old);
            }

            track.key = key;
            track.label = orbiter.name;
            track.supports_maneuvers = orbiter.is_player ||
                                      (orbiter.formation_hold_enabled && !orbiter.formation_leader_name.empty());
            track.is_celestial = false;
            _prediction.tracks.push_back(std::move(track));
        }

        // Add non-reference celestial bodies so overlays can compare against them.
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

                PredictionTrackState track{};
                if (PredictionTrackState *old = find_old_track(key))
                {
                    track = std::move(*old);
                }

                track.key = key;
                track.label = body.name;
                track.supports_maneuvers = false;
                track.is_celestial = true;
                _prediction.tracks.push_back(std::move(track));
            }
        }

        const auto track_exists = [&](PredictionSubjectKey key) {
            return find_prediction_track(key) != nullptr;
        };

        // Preserve the user's active subject when it still exists; otherwise fall back to the player.
        if (track_exists(old_active))
        {
            _prediction.selection.active_subject = old_active;
        }
        else if (const PredictionTrackState *player_track = player_prediction_track())
        {
            _prediction.selection.active_subject = player_track->key;
        }
        else if (!_prediction.tracks.empty())
        {
            _prediction.selection.active_subject = _prediction.tracks.front().key;
        }
        else
        {
            _prediction.selection.active_subject = {};
        }

        _prediction.selection.overlay_subjects.clear();
        _prediction.selection.overlay_subjects.reserve(old_overlays.size());
        for (PredictionSubjectKey key : old_overlays)
        {
            if (!track_exists(key))
            {
                continue;
            }

            if (std::find(_prediction.selection.overlay_subjects.begin(),
                          _prediction.selection.overlay_subjects.end(),
                          key) == _prediction.selection.overlay_subjects.end())
            {
                _prediction.selection.overlay_subjects.push_back(key);
            }
        }

        // There is no dedicated subject/overlay picker in the current UI yet, so keep
        // other live prediction subjects visible by default even after narrowing
        // prediction work to the active/overlay set.
        if (_prediction.selection.overlay_subjects.empty())
        {
            for (const PredictionTrackState &track : _prediction.tracks)
            {
                if (track.key == _prediction.selection.active_subject)
                {
                    continue;
                }

                _prediction.selection.overlay_subjects.push_back(track.key);
            }
        }
        _prediction.selection.selected_group_index = -1;

        sync_prediction_dirty_flag();
    }

    std::vector<PredictionSubjectKey> GameplayState::collect_visible_prediction_subjects() const
    {
        // Visible prediction work is centered on the focused subject plus any explicit overlays.
        std::vector<PredictionSubjectKey> out;
        out.reserve(1 + _prediction.selection.overlay_subjects.size());

        const auto append_visible = [&](PredictionSubjectKey key) {
            if (!key.valid() || !find_prediction_track(key))
            {
                return;
            }

            if (std::find(out.begin(), out.end(), key) == out.end())
            {
                out.push_back(key);
            }
        };

        append_visible(_prediction.selection.active_subject);
        for (PredictionSubjectKey key : _prediction.selection.overlay_subjects)
        {
            append_visible(key);
        }

        if (out.empty() && !_prediction.tracks.empty())
        {
            append_visible(_prediction.tracks.front().key);
        }

        return out;
    }

    double GameplayState::prediction_future_window_s(const PredictionSubjectKey key) const
    {
        // Celestials and spacecraft use different minimum look-ahead windows.
        if (key.kind == PredictionSubjectKind::Celestial)
        {
            return std::max(0.0, _prediction.sampling_policy.celestial_min_window_s);
        }

        return std::max(0.0, _prediction.sampling_policy.orbiter_min_window_s);
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
        // Resolve mutable track state by stable subject key.
        auto it = std::find_if(_prediction.tracks.begin(),
                               _prediction.tracks.end(),
                               [key](const PredictionTrackState &track) { return track.key == key; });
        return (it != _prediction.tracks.end()) ? &(*it) : nullptr;
    }

    const PredictionTrackState *GameplayState::find_prediction_track(PredictionSubjectKey key) const
    {
        // Resolve immutable track state by stable subject key.
        auto it = std::find_if(_prediction.tracks.begin(),
                               _prediction.tracks.end(),
                               [key](const PredictionTrackState &track) { return track.key == key; });
        return (it != _prediction.tracks.end()) ? &(*it) : nullptr;
    }

    PredictionTrackState *GameplayState::active_prediction_track()
    {
        // Return the track currently focused by the prediction UI.
        return find_prediction_track(_prediction.selection.active_subject);
    }

    const PredictionTrackState *GameplayState::active_prediction_track() const
    {
        // Return the track currently focused by the prediction UI.
        return find_prediction_track(_prediction.selection.active_subject);
    }

    PredictionTrackState *GameplayState::player_prediction_track()
    {
        // Resolve the local player's prediction track if one exists.
        const EntityId player_eid = player_entity();
        if (!player_eid.is_valid())
        {
            return nullptr;
        }
        return find_prediction_track(PredictionSubjectKey{PredictionSubjectKind::Orbiter, player_eid.value});
    }

    const PredictionTrackState *GameplayState::player_prediction_track() const
    {
        // Resolve the local player's prediction track if one exists.
        const EntityId player_eid = player_entity();
        if (!player_eid.is_valid())
        {
            return nullptr;
        }
        return find_prediction_track(PredictionSubjectKey{PredictionSubjectKind::Orbiter, player_eid.value});
    }

    OrbitPredictionCache *GameplayState::effective_prediction_cache(PredictionTrackState *track)
    {
        if (!track)
        {
            return nullptr;
        }
        return track->cache.identity.valid ? &track->cache : nullptr;
    }

    const OrbitPredictionCache *GameplayState::effective_prediction_cache(const PredictionTrackState *track) const
    {
        if (!track)
        {
            return nullptr;
        }
        return track->cache.identity.valid ? &track->cache : nullptr;
    }

    OrbitPredictionCache *GameplayState::player_prediction_cache()
    {
        // Expose the player's cached prediction data for HUD/UI consumers.
        return effective_prediction_cache(player_prediction_track());
    }

    const OrbitPredictionCache *GameplayState::player_prediction_cache() const
    {
        // Expose the player's cached prediction data for HUD/UI consumers.
        return effective_prediction_cache(player_prediction_track());
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
