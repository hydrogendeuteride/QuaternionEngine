#include "game/states/gameplay/gameplay_state.h"
#include "game/states/gameplay/orbit_helpers.h"
#include "game/orbit/orbit_prediction_math.h"
#include "game/orbit/orbit_prediction_tuning.h"
#include "core/engine.h"
#include "core/game_api.h"
#include "game/component/ship_controller.h"

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
#include "physics/jolt/jolt_physics_world.h"
#endif

#include "orbitsim/coordinate_frames.hpp"
#include "orbitsim/frame_utils.hpp"
#include "orbitsim/trajectory_transforms.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace Game
{
    namespace
    {
        template<typename T>
        // Return true when a subject key is already present in a small linear container.
        bool contains_key(const std::vector<T> &items, const T &needle)
        {
            return std::find(items.begin(), items.end(), needle) != items.end();
        }

        // Guard async solver requests from non-finite positions or velocities.
        bool finite_vec3(const glm::dvec3 &v)
        {
            return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
        }

        bool same_frame_spec(const orbitsim::TrajectoryFrameSpec &a, const orbitsim::TrajectoryFrameSpec &b)
        {
            return a.type == b.type &&
                   a.primary_body_id == b.primary_body_id &&
                   a.secondary_body_id == b.secondary_body_id &&
                   a.target_spacecraft_id == b.target_spacecraft_id;
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

    } // namespace

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

        out_pos_world = entity->position_world();
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
        const PredictionSubjectKey old_active = _prediction_selection.active_subject;
        const std::vector<PredictionSubjectKey> old_overlays = _prediction_selection.overlay_subjects;
        std::vector<PredictionTrackState> old_tracks = std::move(_prediction_tracks);

        _prediction_tracks.clear();
        _prediction_groups.clear();

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
            track.supports_maneuvers = orbiter.is_player;
            track.is_celestial = false;
            _prediction_tracks.push_back(std::move(track));
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
                _prediction_tracks.push_back(std::move(track));
            }
        }

        std::unordered_map<std::string, std::vector<PredictionSubjectKey>> group_members;
        for (const auto &orbiter_def : _scenario_config.orbiters)
        {
            if (orbiter_def.prediction_group.empty())
            {
                continue;
            }

            auto runtime = std::find_if(_orbiters.begin(),
                                        _orbiters.end(),
                                        [&](const OrbiterInfo &orbiter) { return orbiter.name == orbiter_def.name; });
            if (runtime == _orbiters.end() || !runtime->entity.is_valid())
            {
                continue;
            }

            group_members[orbiter_def.prediction_group].push_back(
                    PredictionSubjectKey{PredictionSubjectKind::Orbiter, runtime->entity.value});
        }

        for (auto &[name, members] : group_members)
        {
            if (members.size() < 2)
            {
                continue;
            }

            PredictionGroup group{};
            group.name = name;
            group.members = members;
            group.primary_subject = members.front();

            for (const PredictionSubjectKey member : members)
            {
                if (prediction_subject_is_player(member))
                {
                    group.primary_subject = member;
                    break;
                }
            }

            _prediction_groups.push_back(std::move(group));
        }

        auto track_exists = [&](PredictionSubjectKey key) {
            return find_prediction_track(key) != nullptr;
        };

        // Restore the last selection when possible, otherwise fall back to a sane default.
        if (track_exists(old_active))
        {
            _prediction_selection.active_subject = old_active;
        }
        else if (const PredictionTrackState *player_track = player_prediction_track())
        {
            _prediction_selection.active_subject = player_track->key;
        }
        else if (!_prediction_tracks.empty())
        {
            _prediction_selection.active_subject = _prediction_tracks.front().key;
        }
        else
        {
            _prediction_selection.active_subject = {};
        }

        _prediction_selection.overlay_subjects.clear();
        for (PredictionSubjectKey key : old_overlays)
        {
            if (key != _prediction_selection.active_subject && track_exists(key))
            {
                _prediction_selection.overlay_subjects.push_back(key);
            }
        }

        const bool had_explicit_selection =
                old_active.valid() ||
                !old_overlays.empty() ||
                _prediction_selection.selected_group_index >= 0;
        if (!had_explicit_selection)
        {
            // Prefer an authored group that contains the player so formation overlays appear together.
            bool seeded_from_group = false;
            for (size_t group_index = 0; group_index < _prediction_groups.size(); ++group_index)
            {
                const PredictionGroup &group = _prediction_groups[group_index];
                if (!contains_key(group.members, _prediction_selection.active_subject))
                {
                    continue;
                }

                _prediction_selection.selected_group_index = static_cast<int>(group_index);
                for (PredictionSubjectKey member : group.members)
                {
                    if (member != _prediction_selection.active_subject)
                    {
                        _prediction_selection.overlay_subjects.push_back(member);
                    }
                }
                seeded_from_group = !_prediction_selection.overlay_subjects.empty();
                break;
            }

            if (!seeded_from_group)
            {
                bool orbiter_overlay_added = false;
                for (const PredictionTrackState &track : _prediction_tracks)
                {
                    if (track.key == _prediction_selection.active_subject)
                    {
                        continue;
                    }

                    if (!orbiter_overlay_added && track.key.kind == PredictionSubjectKind::Orbiter)
                    {
                        _prediction_selection.overlay_subjects.push_back(track.key);
                        orbiter_overlay_added = true;
                    }
                }
            }

            for (const PredictionTrackState &track : _prediction_tracks)
            {
                if (track.key.kind == PredictionSubjectKind::Celestial)
                {
                    _prediction_selection.overlay_subjects.push_back(track.key);
                    break;
                }
            }
        }

        sync_prediction_dirty_flag();
    }

    std::vector<GameplayState::PredictionSubjectKey> GameplayState::collect_visible_prediction_subjects() const
    {
        // Return the deduplicated subject set that the UI wants drawn this frame.
        std::vector<PredictionSubjectKey> out;
        if (_prediction_selection.active_subject.valid())
        {
            out.push_back(_prediction_selection.active_subject);
        }
        for (PredictionSubjectKey key : _prediction_selection.overlay_subjects)
        {
            if (key.valid() && !contains_key(out, key))
            {
                out.push_back(key);
            }
        }
        return out;
    }

    double GameplayState::prediction_future_window_s(const PredictionSubjectKey key) const
    {
        // Celestials and spacecraft use different minimum look-ahead windows.
        if (key.kind == PredictionSubjectKind::Celestial)
        {
            return std::max(0.0, _prediction_future_window_celestial_s);
        }

        return std::max(0.0, _prediction_future_window_orbiter_s);
    }

    GameplayState::PredictionTrackState *GameplayState::find_prediction_track(PredictionSubjectKey key)
    {
        // Resolve mutable track state by stable subject key.
        auto it = std::find_if(_prediction_tracks.begin(),
                               _prediction_tracks.end(),
                               [key](const PredictionTrackState &track) { return track.key == key; });
        return (it != _prediction_tracks.end()) ? &(*it) : nullptr;
    }

    const GameplayState::PredictionTrackState *GameplayState::find_prediction_track(PredictionSubjectKey key) const
    {
        // Resolve immutable track state by stable subject key.
        auto it = std::find_if(_prediction_tracks.begin(),
                               _prediction_tracks.end(),
                               [key](const PredictionTrackState &track) { return track.key == key; });
        return (it != _prediction_tracks.end()) ? &(*it) : nullptr;
    }

    GameplayState::PredictionTrackState *GameplayState::active_prediction_track()
    {
        // Return the track currently focused by the prediction UI.
        return find_prediction_track(_prediction_selection.active_subject);
    }

    const GameplayState::PredictionTrackState *GameplayState::active_prediction_track() const
    {
        // Return the track currently focused by the prediction UI.
        return find_prediction_track(_prediction_selection.active_subject);
    }

    GameplayState::PredictionTrackState *GameplayState::player_prediction_track()
    {
        // Resolve the local player's prediction track if one exists.
        const EntityId player_eid = player_entity();
        if (!player_eid.is_valid())
        {
            return nullptr;
        }
        return find_prediction_track(PredictionSubjectKey{PredictionSubjectKind::Orbiter, player_eid.value});
    }

    const GameplayState::PredictionTrackState *GameplayState::player_prediction_track() const
    {
        // Resolve the local player's prediction track if one exists.
        const EntityId player_eid = player_entity();
        if (!player_eid.is_valid())
        {
            return nullptr;
        }
        return find_prediction_track(PredictionSubjectKey{PredictionSubjectKind::Orbiter, player_eid.value});
    }

    OrbitPredictionCache *GameplayState::player_prediction_cache()
    {
        // Expose the player's cached prediction data for HUD/UI consumers.
        PredictionTrackState *track = player_prediction_track();
        return track ? &track->cache : nullptr;
    }

    const OrbitPredictionCache *GameplayState::player_prediction_cache() const
    {
        // Expose the player's cached prediction data for HUD/UI consumers.
        const PredictionTrackState *track = player_prediction_track();
        return track ? &track->cache : nullptr;
    }

    bool GameplayState::prediction_subject_is_player(PredictionSubjectKey key) const
    {
        // Only the player ship is treated as the maneuver-authoring subject.
        return key.kind == PredictionSubjectKind::Orbiter && key.value == player_entity().value;
    }

    bool GameplayState::prediction_subject_supports_maneuvers(PredictionSubjectKey key) const
    {
        // Maneuver planning currently applies only to the player prediction.
        return prediction_subject_is_player(key);
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
        if (body_id == orbitsim::kInvalidBodyId)
        {
            return nullptr;
        }

        for (const orbitsim::MassiveBody &body : bodies)
        {
            if (body.id == body_id)
            {
                return &body;
            }
        }

        return nullptr;
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

    bool GameplayState::set_prediction_frame_spec(const orbitsim::TrajectoryFrameSpec &spec)
    {
        if (same_frame_spec(_prediction_frame_selection.spec, spec))
        {
            return false;
        }

        _prediction_frame_selection.spec = spec;
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
            for (const CelestialBodyInfo &body : _orbitsim->bodies)
            {
                options.push_back(PredictionFrameOption{
                        .spec = orbitsim::TrajectoryFrameSpec::body_centered_inertial(body.sim_id),
                        .label = body.name + " BCI",
                });
            }
        }

        _prediction_frame_selection.options = std::move(options);

        if (_prediction_frame_selection.selected_index < 0)
        {
            _prediction_frame_selection.spec = default_prediction_frame_spec();
        }

        int selected_index = -1;
        for (std::size_t i = 0; i < _prediction_frame_selection.options.size(); ++i)
        {
            if (same_frame_spec(_prediction_frame_selection.options[i].spec, _prediction_frame_selection.spec))
            {
                selected_index = static_cast<int>(i);
                break;
            }
        }

        if (selected_index < 0)
        {
            _prediction_frame_selection.spec = default_prediction_frame_spec();
            for (std::size_t i = 0; i < _prediction_frame_selection.options.size(); ++i)
            {
                if (same_frame_spec(_prediction_frame_selection.options[i].spec, _prediction_frame_selection.spec))
                {
                    selected_index = static_cast<int>(i);
                    break;
                }
            }
        }

        _prediction_frame_selection.selected_index = selected_index;
    }

    orbitsim::BodyId GameplayState::resolve_prediction_analysis_body_id(const OrbitPredictionCache &cache,
                                                                        const PredictionSubjectKey key,
                                                                        const double query_time_s) const
    {
        if (_prediction_frame_selection.spec.type == orbitsim::TrajectoryFrameType::BodyCenteredInertial)
        {
            return _prediction_frame_selection.spec.primary_body_id;
        }

        if (cache.massive_bodies.empty())
        {
            return orbitsim::kInvalidBodyId;
        }

        glm::dvec3 query_position_m(0.0);
        if (!cache.trajectory_inertial.empty())
        {
            query_position_m = cache.trajectory_inertial.front().position_m;
        }
        else
        {
            return orbitsim::kInvalidBodyId;
        }

        std::vector<orbitsim::MassiveBody> candidates;
        candidates.reserve(cache.massive_bodies.size());
        for (const orbitsim::MassiveBody &body : cache.massive_bodies)
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

        const auto body_position_at = [&](const std::size_t i) -> orbitsim::Vec3 {
            const orbitsim::MassiveBody &body = candidates[i];
            if (cache.shared_ephemeris && !cache.shared_ephemeris->empty())
            {
                return cache.shared_ephemeris->body_state_at_by_id(body.id, query_time_s).position_m;
            }
            return body.state.position_m;
        };

        const std::size_t primary_index = orbitsim::auto_select_primary_index(
                candidates, query_position_m, body_position_at, _orbitsim ? _orbitsim->sim.config().softening_length_m : 0.0);
        if (primary_index >= candidates.size())
        {
            return orbitsim::kInvalidBodyId;
        }

        return candidates[primary_index].id;
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

    WorldVec3 GameplayState::prediction_frame_origin_world(const OrbitPredictionCache &cache) const
    {
        if (_prediction_frame_selection.spec.type == orbitsim::TrajectoryFrameType::BodyCenteredInertial)
        {
            if (const CelestialBodyInfo *body_info = find_celestial_body_info(_prediction_frame_selection.spec.primary_body_id))
            {
                if (body_info->render_entity.is_valid())
                {
                    if (const Entity *entity = _world.entities().find(body_info->render_entity))
                    {
                        return entity->position_world();
                    }
                }

                if (const orbitsim::MassiveBody *body = find_massive_body(cache.massive_bodies, body_info->sim_id))
                {
                    const orbitsim::MassiveBody *world_ref_sim = _orbitsim ? _orbitsim->world_reference_sim_body() : nullptr;
                    if (world_ref_sim)
                    {
                        return prediction_world_reference_body_world() +
                               WorldVec3(body->state.position_m - world_ref_sim->state.position_m);
                    }
                }
            }
        }

        return _scenario_config.system_center;
    }

    void GameplayState::refresh_prediction_derived_cache(PredictionTrackState &track)
    {
        track.cache.trajectory_frame.clear();
        track.cache.trajectory_frame_planned.clear();
        track.cache.trajectory_segments_frame.clear();
        track.cache.trajectory_segments_frame_planned.clear();
        track.cache.points_world.clear();
        track.cache.points_world_planned.clear();
        track.cache.altitude_km.clear();
        track.cache.speed_kmps.clear();
        track.cache.semi_major_axis_m = 0.0;
        track.cache.eccentricity = 0.0;
        track.cache.orbital_period_s = 0.0;
        track.cache.periapsis_alt_km = 0.0;
        track.cache.apoapsis_alt_km = std::numeric_limits<double>::infinity();

        if (!track.cache.valid || track.cache.trajectory_inertial.size() < 2)
        {
            return;
        }

        if (_prediction_frame_selection.spec.type == orbitsim::TrajectoryFrameType::Inertial)
        {
            track.cache.trajectory_frame = track.cache.trajectory_inertial;
            track.cache.trajectory_frame_planned = track.cache.trajectory_inertial_planned;
        }
        else
        {
            if (!track.cache.shared_ephemeris || track.cache.shared_ephemeris->empty())
            {
                track.cache.valid = false;
                return;
            }

            track.cache.trajectory_frame = orbitsim::trajectory_to_frame_spec(
                    track.cache.trajectory_inertial,
                    *track.cache.shared_ephemeris,
                    track.cache.massive_bodies,
                    _prediction_frame_selection.spec);
            if (!track.cache.trajectory_inertial_planned.empty())
            {
                track.cache.trajectory_frame_planned = orbitsim::trajectory_to_frame_spec(
                        track.cache.trajectory_inertial_planned,
                        *track.cache.shared_ephemeris,
                        track.cache.massive_bodies,
                        _prediction_frame_selection.spec);
            }
        }

        if (track.cache.trajectory_frame.size() < 2)
        {
            track.cache.valid = false;
            return;
        }

        track.cache.trajectory_segments_frame = trajectory_segments_from_samples(track.cache.trajectory_frame);
        if (!track.cache.trajectory_frame_planned.empty())
        {
            track.cache.trajectory_segments_frame_planned = trajectory_segments_from_samples(track.cache.trajectory_frame_planned);
        }

        const WorldVec3 frame_origin_world = prediction_frame_origin_world(track.cache);
        track.cache.points_world.resize(track.cache.trajectory_frame.size());
        for (std::size_t i = 0; i < track.cache.trajectory_frame.size(); ++i)
        {
            track.cache.points_world[i] = frame_origin_world + WorldVec3(track.cache.trajectory_frame[i].position_m);
        }

        track.cache.points_world_planned.resize(track.cache.trajectory_frame_planned.size());
        for (std::size_t i = 0; i < track.cache.trajectory_frame_planned.size(); ++i)
        {
            track.cache.points_world_planned[i] = frame_origin_world + WorldVec3(track.cache.trajectory_frame_planned[i].position_m);
        }

        const orbitsim::BodyId analysis_body_id =
                resolve_prediction_analysis_body_id(track.cache, track.key, track.cache.trajectory_inertial.front().t_s);
        const orbitsim::MassiveBody *analysis_body = find_massive_body(track.cache.massive_bodies, analysis_body_id);
        if (!analysis_body || !(analysis_body->mass_kg > 0.0))
        {
            return;
        }

        const double mu_ref_m3_s2 = _orbitsim->sim.config().gravitational_constant * analysis_body->mass_kg;
        if (!(mu_ref_m3_s2 > 0.0) || !std::isfinite(mu_ref_m3_s2))
        {
            return;
        }

        std::vector<orbitsim::TrajectorySample> rel_samples;
        if (_prediction_frame_selection.spec.type == orbitsim::TrajectoryFrameType::BodyCenteredInertial &&
            analysis_body_id == _prediction_frame_selection.spec.primary_body_id)
        {
            rel_samples = track.cache.trajectory_frame;
        }
        else
        {
            if (!track.cache.shared_ephemeris || track.cache.shared_ephemeris->empty())
            {
                return;
            }
            rel_samples = orbitsim::trajectory_to_frame_spec(
                    track.cache.trajectory_inertial,
                    *track.cache.shared_ephemeris,
                    track.cache.massive_bodies,
                    orbitsim::TrajectoryFrameSpec::body_centered_inertial(analysis_body_id));
        }

        track.cache.altitude_km.reserve(rel_samples.size());
        track.cache.speed_kmps.reserve(rel_samples.size());
        for (const orbitsim::TrajectorySample &sample : rel_samples)
        {
            const double r_m = glm::length(sample.position_m);
            const double alt_km = (r_m - analysis_body->radius_m) * 1.0e-3;
            const double spd_kmps = glm::length(sample.velocity_mps) * 1.0e-3;
            track.cache.altitude_km.push_back(static_cast<float>(alt_km));
            track.cache.speed_kmps.push_back(static_cast<float>(spd_kmps));
        }

        const OrbitPredictionMath::OrbitalElementsEstimate elements =
                OrbitPredictionMath::compute_orbital_elements(mu_ref_m3_s2,
                                                              rel_samples.front().position_m,
                                                              rel_samples.front().velocity_mps);
        if (elements.valid)
        {
            track.cache.semi_major_axis_m = elements.semi_major_axis_m;
            track.cache.eccentricity = elements.eccentricity;
            track.cache.orbital_period_s = elements.orbital_period_s;
            track.cache.periapsis_alt_km = (elements.periapsis_m - analysis_body->radius_m) * 1.0e-3;
            track.cache.apoapsis_alt_km = std::isfinite(elements.apoapsis_m)
                                                  ? (elements.apoapsis_m - analysis_body->radius_m) * 1.0e-3
                                                  : std::numeric_limits<double>::infinity();
        }
    }

    void GameplayState::refresh_all_prediction_derived_caches()
    {
        rebuild_prediction_frame_options();
        for (PredictionTrackState &track : _prediction_tracks)
        {
            refresh_prediction_derived_cache(track);
        }
    }

    void GameplayState::sync_prediction_dirty_flag()
    {
        // Collapse per-track dirty/pending state into one cheap UI-facing flag.
        _prediction_dirty = false;
        for (const PredictionTrackState &track : _prediction_tracks)
        {
            if (track.dirty || track.request_pending)
            {
                _prediction_dirty = true;
                return;
            }
        }
    }

    void GameplayState::mark_prediction_dirty()
    {
        // Force every cached track to rebuild on the next prediction update.
        for (PredictionTrackState &track : _prediction_tracks)
        {
            track.dirty = true;
        }
        sync_prediction_dirty_flag();
    }

    void GameplayState::clear_prediction_runtime()
    {
        // Drop every cached artifact when the feature is disabled.
        for (PredictionTrackState &track : _prediction_tracks)
        {
            track.cache.clear();
            track.request_pending = false;
            track.dirty = false;
        }
        _prediction_service.reset();
        _prediction_dirty = false;
    }

    void GameplayState::clear_visible_prediction_runtime(const std::vector<PredictionSubjectKey> &visible_subjects)
    {
        // A service reset invalidates every track, even ones that are currently hidden.
        (void) visible_subjects;
        for (PredictionTrackState &track : _prediction_tracks)
        {
            track.clear_runtime();
        }
    }

    void GameplayState::apply_completed_prediction_result(OrbitPredictionService::Result result)
    {
        // Fold worker output back into the owning track cache.
        PredictionTrackState *track = nullptr;
        for (PredictionTrackState &candidate : _prediction_tracks)
        {
            if (candidate.key.track_id() == result.track_id)
            {
                track = &candidate;
                break;
            }
        }

        if (!track)
        {
            return;
        }

        track->request_pending = false;
        const bool keep_dirty_for_followup = track->dirty;
        track->solver_ms_last = std::max(0.0, result.compute_time_ms);

        if (!result.valid || result.trajectory_inertial.size() < 2)
        {
            track->cache.clear();
            track->dirty = true;
            return;
        }

        WorldVec3 build_pos_world{0.0, 0.0, 0.0};
        glm::dvec3 build_vel_world{0.0};
        glm::vec3 build_vel_local{0.0f};
        (void) get_prediction_subject_world_state(track->key, build_pos_world, build_vel_world, build_vel_local);

        track->cache.clear();
        track->cache.build_time_s = result.build_time_s;
        track->cache.build_pos_world = build_pos_world;
        track->cache.build_vel_world = build_vel_world;
        track->cache.shared_ephemeris = std::move(result.shared_ephemeris);
        track->cache.massive_bodies = std::move(result.massive_bodies);
        track->cache.trajectory_inertial = std::move(result.trajectory_inertial);
        track->cache.trajectory_inertial_planned = std::move(result.trajectory_inertial_planned);
        track->cache.trajectory_segments_inertial = std::move(result.trajectory_segments_inertial);
        track->cache.trajectory_segments_inertial_planned = std::move(result.trajectory_segments_inertial_planned);
        track->cache.maneuver_previews = std::move(result.maneuver_previews);
        track->cache.valid = true;
        refresh_prediction_derived_cache(*track);
        track->dirty = keep_dirty_for_followup;
    }

    bool GameplayState::should_rebuild_prediction_track(const PredictionTrackState &track,
                                                        const double now_s,
                                                        const float fixed_dt,
                                                        const bool thrusting,
                                                        const bool with_maneuvers) const
    {
        // Rebuild when cache state, thrusting, timing, or horizon coverage says we must.
        bool rebuild = track.dirty || !track.cache.valid;

        if (!rebuild && thrusting)
        {
            const double dt_since_build_s = now_s - track.cache.build_time_s;
            rebuild = dt_since_build_s >= _prediction_thrust_refresh_s;
        }

        if (!rebuild && _prediction_periodic_refresh_s > 0.0)
        {
            const double dt_since_build_s = now_s - track.cache.build_time_s;
            rebuild = dt_since_build_s >= _prediction_periodic_refresh_s;
        }

        // Throttle live gizmo drags so the async solver does not thrash every tick.
        if (rebuild &&
            track.cache.valid &&
            with_maneuvers &&
            _maneuver_gizmo_interaction.state == ManeuverGizmoInteraction::State::DragAxis)
        {
            const double dt_since_build_s = now_s - track.cache.build_time_s;
            if (dt_since_build_s < OrbitPredictionTuning::kDragRebuildMinIntervalS)
            {
                rebuild = false;
            }
        }

        if (rebuild || !track.cache.valid || track.cache.trajectory_inertial.empty())
        {
            return rebuild;
        }

        double cache_end_s = track.cache.trajectory_inertial.back().t_s;
        if (!track.cache.trajectory_segments_inertial.empty())
        {
            const orbitsim::TrajectorySegment &last_segment = track.cache.trajectory_segments_inertial.back();
            const double segment_end_s = last_segment.t0_s + last_segment.dt_s;
            if (std::isfinite(segment_end_s))
            {
                cache_end_s = segment_end_s;
            }
        }

        double required_ahead_s = std::max(0.0, _prediction_draw_future_segment ? prediction_future_window_s(track.key) : 0.0);
        if (with_maneuvers)
        {
            double max_node_time_s = now_s;
            for (const ManeuverNode &node : _maneuver_state.nodes)
            {
                if (std::isfinite(node.time_s))
                {
                    max_node_time_s = std::max(max_node_time_s, node.time_s);
                }
            }

            if (max_node_time_s > now_s)
            {
                const double future_window_s = prediction_future_window_s(track.key);
                const double post_node_window_s =
                        std::max(OrbitPredictionTuning::kPostNodeCoverageMinS, future_window_s);
                required_ahead_s = std::max(required_ahead_s, (max_node_time_s - now_s) + post_node_window_s);
            }
        }

        // Add a small epsilon so tiny fixed-step jitter does not trigger rebuild churn.
        const double coverage_epsilon_s =
                std::max(1.0e-3, std::min(0.25, std::max(0.0, static_cast<double>(fixed_dt)) * 0.5));
        return (cache_end_s - now_s + coverage_epsilon_s) < required_ahead_s;
    }

    bool GameplayState::request_orbiter_prediction_async(PredictionTrackState &track,
                                                         const WorldVec3 &subject_pos_world,
                                                         const glm::dvec3 &subject_vel_world,
                                                         const double now_s,
                                                         const bool thrusting,
                                                         const bool with_maneuvers)
    {
        // Package the current spacecraft state into a worker request.
        if (!_orbitsim)
        {
            return false;
        }

        const orbitsim::MassiveBody *ref_sim = _orbitsim->world_reference_sim_body();
        if (!ref_sim || ref_sim->id == orbitsim::kInvalidBodyId)
        {
            return false;
        }

        const WorldVec3 ref_body_world = prediction_world_reference_body_world();
        const glm::dvec3 ship_rel_pos_m = glm::dvec3(subject_pos_world - ref_body_world);
        const glm::dvec3 ship_rel_vel_mps = subject_vel_world;
        const glm::dvec3 ship_bary_pos_m = ref_sim->state.position_m + ship_rel_pos_m;
        const glm::dvec3 ship_bary_vel_mps = ref_sim->state.velocity_mps + ship_rel_vel_mps;
        if (!finite_vec3(ship_bary_pos_m) || !finite_vec3(ship_bary_vel_mps))
        {
            return false;
        }

        OrbitPredictionService::Request request{};
        request.track_id = track.key.track_id();
        request.sim_time_s = now_s;
        request.sim_config = _orbitsim->sim.config();
        request.massive_bodies = _orbitsim->sim.massive_bodies();
        request.ship_bary_position_m = ship_bary_pos_m;
        request.ship_bary_velocity_mps = ship_bary_vel_mps;
        request.thrusting = thrusting;
        request.future_window_s = prediction_future_window_s(track.key);
        request.max_maneuver_time_s = now_s;

        // Copy currently authored maneuver nodes so the worker can include planned burns.
        if (with_maneuvers)
        {
            request.maneuver_impulses.reserve(_maneuver_state.nodes.size());
            for (const ManeuverNode &node : _maneuver_state.nodes)
            {
                if (!std::isfinite(node.time_s))
                {
                    continue;
                }

                request.max_maneuver_time_s = std::max(request.max_maneuver_time_s, node.time_s);

                OrbitPredictionService::ManeuverImpulse impulse{};
                impulse.node_id = node.id;
                impulse.t_s = node.time_s;
                impulse.primary_body_id = node.primary_body_id;
                impulse.dv_rtn_mps = orbitsim::Vec3{node.dv_rtn_mps.x, node.dv_rtn_mps.y, node.dv_rtn_mps.z};
                request.maneuver_impulses.push_back(impulse);
            }
        }

        _prediction_service.request(std::move(request));
        track.request_pending = true;
        return true;
    }

    bool GameplayState::request_celestial_prediction_async(PredictionTrackState &track, const double now_s)
    {
        if (!_orbitsim)
        {
            return false;
        }

        const orbitsim::MassiveBody *body = _orbitsim->sim.body_by_id(static_cast<orbitsim::BodyId>(track.key.value));
        if (!body)
        {
            return false;
        }

        OrbitPredictionService::Request request{};
        request.kind = OrbitPredictionService::RequestKind::Celestial;
        request.track_id = track.key.track_id();
        request.sim_time_s = now_s;
        request.sim_config = _orbitsim->sim.config();
        request.massive_bodies = _orbitsim->sim.massive_bodies();
        request.subject_body_id = body->id;
        request.future_window_s = prediction_future_window_s(track.key);

        // Celestial tracks now flow through the same worker queue as spacecraft tracks.
        _prediction_service.request(std::move(request));
        track.request_pending = true;
        return true;
    }

    void GameplayState::update_orbiter_prediction_track(PredictionTrackState &track,
                                                        const double now_s,
                                                        const bool thrusting,
                                                        const bool with_maneuvers)
    {
        // Orbiters rebuild asynchronously from their current world-space snapshot.
        WorldVec3 subject_pos_world{0.0, 0.0, 0.0};
        glm::dvec3 subject_vel_world{0.0};
        glm::vec3 subject_vel_local{0.0f};
        if (!get_prediction_subject_world_state(track.key, subject_pos_world, subject_vel_world, subject_vel_local))
        {
            track.cache.clear();
            track.dirty = true;
            track.request_pending = false;
            return;
        }

        if (track.request_pending)
        {
            track.dirty = true;
            return;
        }

        const bool requested =
                request_orbiter_prediction_async(track, subject_pos_world, subject_vel_world, now_s, thrusting, with_maneuvers);
        track.dirty = !requested;
        if (!requested)
        {
            track.cache.clear();
            track.request_pending = false;
        }
    }

    void GameplayState::update_celestial_prediction_track(PredictionTrackState &track, const double now_s)
    {
        // Celestial predictions rebuild asynchronously so the gameplay thread only packages requests.
        if (!_orbitsim)
        {
            track.cache.clear();
            track.dirty = true;
            track.request_pending = false;
            return;
        }

        if (track.request_pending)
        {
            track.dirty = true;
            return;
        }

        const bool requested = request_celestial_prediction_async(track, now_s);
        // Keep drawing the previous cache until the worker publishes a replacement.
        track.dirty = !requested;
        if (!requested)
        {
            track.cache.clear();
            track.request_pending = false;
        }
    }

    void GameplayState::update_prediction(GameStateContext &ctx, float fixed_dt)
    {
        (void) ctx;

        if (!_prediction_enabled)
        {
            // Fully disable runtime state when the overlay is turned off.
            clear_prediction_runtime();
            return;
        }

        // Keep the subject list aligned with the current gameplay scene.
        rebuild_prediction_subjects();
        rebuild_prediction_frame_options();

        const double now_s = _orbitsim ? _orbitsim->sim.time_s() : _fixed_time_s;
        // Pull any finished async jobs into the visible cache set.
        while (auto completed = _prediction_service.poll_completed())
        {
            apply_completed_prediction_result(std::move(*completed));
        }

        const std::vector<PredictionSubjectKey> visible_subjects = collect_visible_prediction_subjects();
        if (!_orbitsim)
        {
            // Without orbit-sim there is no stable frame to predict against.
            clear_visible_prediction_runtime(visible_subjects);
            _prediction_service.reset();
            sync_prediction_dirty_flag();
            return;
        }

        const orbitsim::MassiveBody *ref_sim = _orbitsim->world_reference_sim_body();
        if (!ref_sim || ref_sim->id == orbitsim::kInvalidBodyId)
        {
            // Bail out when the reference body is not ready for a stable inertial frame.
            clear_visible_prediction_runtime(visible_subjects);
            _prediction_service.reset();
            sync_prediction_dirty_flag();
            return;
        }

        // Rebuild or refresh only the subjects that are actually visible.
        for (PredictionTrackState &track : _prediction_tracks)
        {
            if (!contains_key(visible_subjects, track.key))
            {
                continue;
            }

            const bool with_maneuvers =
                    prediction_subject_supports_maneuvers(track.key) &&
                    _maneuver_nodes_enabled &&
                    !_maneuver_state.nodes.empty();
            const bool thrusting = prediction_subject_thrust_applied_this_tick(track.key);
            const bool rebuild = should_rebuild_prediction_track(track, now_s, fixed_dt, thrusting, with_maneuvers);
            if (!rebuild)
            {
                continue;
            }

            if (track.key.kind == PredictionSubjectKind::Celestial)
            {
                update_celestial_prediction_track(track, now_s);
                continue;
            }

            update_orbiter_prediction_track(track, now_s, thrusting, with_maneuvers);
        }

        // Mirror the active track's last solver time into the shared debug HUD stats.
        PredictionTrackState *active_track = active_prediction_track();
        _orbit_plot_perf.solver_ms_last = active_track ? active_track->solver_ms_last : 0.0;
        sync_prediction_dirty_flag();
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

    WorldVec3 GameplayState::prediction_reference_body_world() const
    {
        return prediction_world_reference_body_world();
    }

    void GameplayState::refresh_prediction_world_points(PredictionTrackState &track)
    {
        refresh_prediction_derived_cache(track);
    }

} // namespace Game
