#include "game/states/gameplay/gameplay_state.h"
#include "game/states/gameplay/prediction/gameplay_prediction_cache_internal.h"

#include "game/orbit/orbit_prediction_math.h"
#include "game/orbit/orbit_prediction_tuning.h"

#include "orbitsim/coordinate_frames.hpp"
#include "orbitsim/frame_utils.hpp"
#include "orbitsim/trajectory_transforms.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>

namespace Game
{
    namespace
    {
        bool same_frame_spec(const orbitsim::TrajectoryFrameSpec &a, const orbitsim::TrajectoryFrameSpec &b)
        {
            return a.type == b.type &&
                   a.primary_body_id == b.primary_body_id &&
                   a.secondary_body_id == b.secondary_body_id &&
                   a.target_spacecraft_id == b.target_spacecraft_id;
        }

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

    bool GameplayState::prediction_frame_is_lagrange_sensitive(const orbitsim::TrajectoryFrameSpec &spec) const
    {
        return spec.type == orbitsim::TrajectoryFrameType::Synodic;
    }

    double GameplayState::resolve_prediction_display_reference_time_s(const OrbitPredictionCache &cache,
                                                                      const double display_time_s) const
    {
        double reference_time_s = display_time_s;
        if (!std::isfinite(reference_time_s))
        {
            reference_time_s = _orbitsim ? _orbitsim->sim.time_s() : cache.build_time_s;
        }

        const orbitsim::TrajectoryFrameSpec frame_spec =
                cache.resolved_frame_spec_valid
                    ? cache.resolved_frame_spec
                    : resolve_prediction_display_frame_spec(cache, reference_time_s);
        const bool dragging_maneuver_axis =
                _maneuver_gizmo_interaction.state == ManeuverGizmoInteraction::State::DragAxis;
        if (dragging_maneuver_axis &&
            prediction_frame_is_lagrange_sensitive(frame_spec) &&
            std::isfinite(_maneuver_gizmo_interaction.drag_display_reference_time_s))
        {
            return _maneuver_gizmo_interaction.drag_display_reference_time_s;
        }
        const bool maneuver_live_preview =
                _maneuver_plan_live_preview_active || dragging_maneuver_axis;
        const bool freeze_planned_synodic =
                prediction_frame_is_lagrange_sensitive(frame_spec) &&
                _maneuver_nodes_enabled &&
                !_maneuver_state.nodes.empty() &&
                !maneuver_live_preview &&
                std::isfinite(cache.build_time_s);
        return freeze_planned_synodic ? cache.build_time_s : reference_time_s;
    }

    orbitsim::BodyId GameplayState::select_prediction_primary_body_id(const std::vector<orbitsim::MassiveBody> &bodies,
                                                                      const OrbitPredictionCache *cache,
                                                                      const orbitsim::Vec3 &query_pos_m,
                                                                      const double query_time_s,
                                                                      const orbitsim::BodyId preferred_body_id) const
    {
        if (bodies.empty())
        {
            return orbitsim::kInvalidBodyId;
        }

        const double softening_length_m = _orbitsim ? _orbitsim->sim.config().softening_length_m : 0.0;
        const double eps2 = softening_length_m * softening_length_m;
        std::size_t best_index = 0;
        double best_metric = -1.0;
        std::optional<std::size_t> preferred_index;
        double preferred_metric = -1.0;

        const auto body_position_at = [&](const std::size_t i) -> orbitsim::Vec3 {
            const orbitsim::MassiveBody &body = bodies[i];
            if (cache && cache->shared_ephemeris && !cache->shared_ephemeris->empty())
            {
                return cache->shared_ephemeris->body_state_at_by_id(body.id, query_time_s).position_m;
            }
            return body.state.position_m;
        };

        for (std::size_t i = 0; i < bodies.size(); ++i)
        {
            const double mass_kg = std::isfinite(bodies[i].mass_kg) ? bodies[i].mass_kg : 0.0;
            if (!(mass_kg >= 0.0))
            {
                continue;
            }

            const orbitsim::Vec3 dr = body_position_at(i) - query_pos_m;
            const double r2 = glm::dot(dr, dr) + eps2;
            if (!(r2 > 0.0) || !std::isfinite(r2))
            {
                continue;
            }

            const double metric = mass_kg / r2;
            if (metric > best_metric)
            {
                best_metric = metric;
                best_index = i;
            }

            if (bodies[i].id == preferred_body_id)
            {
                preferred_index = i;
                preferred_metric = metric;
            }
        }

        if (best_metric <= 0.0)
        {
            return orbitsim::kInvalidBodyId;
        }

        if (preferred_index.has_value() &&
            preferred_metric > 0.0 &&
            preferred_metric >= (best_metric * OrbitPredictionTuning::kPrimaryBodyHysteresisKeepRatio))
        {
            return bodies[*preferred_index].id;
        }

        return bodies[best_index].id;
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
            if (const PredictionTrackState *player_track = player_prediction_track())
            {
                if (player_track->cache.valid)
                {
                    position_cache = &player_track->cache;
                }
            }
        }

        const orbitsim::MassiveBody *body = nullptr;
        if (position_cache)
        {
            body = find_massive_body(position_cache->massive_bodies, body_id);
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
            if (position_cache && position_cache->shared_ephemeris && !position_cache->shared_ephemeris->empty())
            {
                return position_cache->shared_ephemeris->body_state_at_by_id(massive_body.id, sample_time_s);
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
                                find_massive_body(position_cache->massive_bodies, world_ref_info->sim_id))
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
        return PredictionCacheInternal::sample_prediction_inertial_state(trajectory, query_time_s, out_state);
    }

    orbitsim::SpacecraftStateLookup GameplayState::build_prediction_player_lookup() const
    {
        const PredictionTrackState *player_track = player_prediction_track();
        if (!player_track || !player_track->cache.valid)
        {
            return nullptr;
        }

        const std::vector<orbitsim::TrajectorySegment> *trajectory_segments = nullptr;
        if (!player_track->cache.trajectory_segments_inertial_planned.empty())
        {
            trajectory_segments = &player_track->cache.trajectory_segments_inertial_planned;
        }
        else if (!player_track->cache.trajectory_segments_inertial.empty())
        {
            trajectory_segments = &player_track->cache.trajectory_segments_inertial;
        }
        if (!trajectory_segments)
        {
            return nullptr;
        }

        return PredictionCacheInternal::build_player_lookup(*trajectory_segments);
    }

    orbitsim::TrajectoryFrameSpec GameplayState::resolve_prediction_display_frame_spec(const OrbitPredictionCache &cache,
                                                                                       double display_time_s) const
    {
        orbitsim::TrajectoryFrameSpec resolved = _prediction_frame_selection.spec;
        if (resolved.type != orbitsim::TrajectoryFrameType::LVLH ||
            resolved.primary_body_id != orbitsim::kInvalidBodyId ||
            cache.massive_bodies.empty())
        {
            return resolved;
        }

        double reference_time_s = display_time_s;
        if (!std::isfinite(reference_time_s))
        {
            reference_time_s = _orbitsim ? _orbitsim->sim.time_s() : cache.build_time_s;
        }

        const auto player_lookup = build_prediction_player_lookup();
        if (!player_lookup)
        {
            return resolved;
        }

        const std::optional<orbitsim::State> target_state =
                player_lookup(resolved.target_spacecraft_id, reference_time_s);
        if (!target_state.has_value())
        {
            return resolved;
        }

        const orbitsim::BodyId preferred_body_id =
                cache.resolved_frame_spec_valid && cache.resolved_frame_spec.primary_body_id != orbitsim::kInvalidBodyId
                        ? cache.resolved_frame_spec.primary_body_id
                        : _prediction_frame_selection.spec.primary_body_id;
        const orbitsim::BodyId primary_body_id = select_prediction_primary_body_id(
                cache.massive_bodies,
                &cache,
                target_state->position_m,
                reference_time_s,
                preferred_body_id);
        if (primary_body_id != orbitsim::kInvalidBodyId)
        {
            resolved.primary_body_id = primary_body_id;
        }

        return resolved;
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

        _prediction_analysis_selection.options = std::move(options);
        if (_prediction_analysis_selection.selected_index < 0)
        {
            _prediction_analysis_selection.spec = {};
        }

        int selected_index = -1;
        for (std::size_t i = 0; i < _prediction_analysis_selection.options.size(); ++i)
        {
            if (same_analysis_spec(_prediction_analysis_selection.options[i].spec, _prediction_analysis_selection.spec))
            {
                selected_index = static_cast<int>(i);
                break;
            }
        }

        if (selected_index < 0)
        {
            _prediction_analysis_selection.spec = {};
            selected_index = 0;
        }

        _prediction_analysis_selection.selected_index = selected_index;
    }

    bool GameplayState::set_prediction_analysis_spec(const PredictionAnalysisSpec &spec)
    {
        if (same_analysis_spec(_prediction_analysis_selection.spec, spec))
        {
            return false;
        }

        _prediction_analysis_selection.spec = spec;
        refresh_all_prediction_derived_caches();
        sync_prediction_dirty_flag();
        return true;
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
                                PredictionCacheInternal::kPlayerDisplayTargetSpacecraftId),
                        .label = "Player LVLH",
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
                                                                        const double query_time_s,
                                                                        const orbitsim::BodyId preferred_body_id) const
    {
        if (_prediction_analysis_selection.spec.mode == PredictionAnalysisMode::FixedBodyBCI &&
            _prediction_analysis_selection.spec.fixed_body_id != orbitsim::kInvalidBodyId)
        {
            return _prediction_analysis_selection.spec.fixed_body_id;
        }

        if (cache.massive_bodies.empty())
        {
            return orbitsim::kInvalidBodyId;
        }

        orbitsim::State query_state{};
        if (!PredictionCacheInternal::sample_prediction_inertial_state(cache.trajectory_segments_inertial,
                                                                       query_time_s,
                                                                       query_state) &&
            !sample_prediction_inertial_state(cache.trajectory_inertial, query_time_s, query_state))
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

        orbitsim::BodyId effective_preferred_body_id = preferred_body_id;
        if (effective_preferred_body_id == orbitsim::kInvalidBodyId)
        {
            if (const PredictionTrackState *track = find_prediction_track(key))
            {
                effective_preferred_body_id = track->auto_primary_body_id;
            }
        }
        if (effective_preferred_body_id == orbitsim::kInvalidBodyId &&
            cache.resolved_frame_spec_valid &&
            cache.resolved_frame_spec.primary_body_id != orbitsim::kInvalidBodyId)
        {
            effective_preferred_body_id = cache.resolved_frame_spec.primary_body_id;
        }
        if (effective_preferred_body_id == orbitsim::kInvalidBodyId &&
            _prediction_frame_selection.spec.primary_body_id != orbitsim::kInvalidBodyId)
        {
            effective_preferred_body_id = _prediction_frame_selection.spec.primary_body_id;
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
        out_frame = {};
        const double reference_time_s = resolve_prediction_display_reference_time_s(cache, display_time_s);
        const orbitsim::TrajectoryFrameSpec frame_spec =
                cache.resolved_frame_spec_valid
                    ? cache.resolved_frame_spec
                    : resolve_prediction_display_frame_spec(cache, reference_time_s);
        const auto state_at = [&](const orbitsim::MassiveBody &body) -> orbitsim::State {
            if (cache.shared_ephemeris && !cache.shared_ephemeris->empty())
            {
                return cache.shared_ephemeris->body_state_at_by_id(body.id, reference_time_s);
            }
            return body.state;
        };
        switch (frame_spec.type)
        {
            case orbitsim::TrajectoryFrameType::Inertial:
                break;

            case orbitsim::TrajectoryFrameType::BodyCenteredInertial:
            {
                const orbitsim::MassiveBody *body =
                        find_massive_body(cache.massive_bodies, frame_spec.primary_body_id);
                if (!body)
                {
                    return false;
                }
                out_frame = orbitsim::make_body_centered_inertial_frame(state_at(*body));
                break;
            }

            case orbitsim::TrajectoryFrameType::BodyFixed:
            {
                const orbitsim::MassiveBody *body =
                        find_massive_body(cache.massive_bodies, frame_spec.primary_body_id);
                if (!body)
                {
                    return false;
                }
                const std::optional<orbitsim::RotatingFrame> body_fixed = orbitsim::make_body_fixed_frame(state_at(*body));
                if (!body_fixed.has_value())
                {
                    return false;
                }
                out_frame = *body_fixed;
                break;
            }

            case orbitsim::TrajectoryFrameType::Synodic:
            {
                const orbitsim::MassiveBody *body_a =
                        find_massive_body(cache.massive_bodies, frame_spec.primary_body_id);
                const orbitsim::MassiveBody *body_b =
                        find_massive_body(cache.massive_bodies, frame_spec.secondary_body_id);
                if (!body_a || !body_b)
                {
                    return false;
                }

                const std::optional<orbitsim::SynodicFrame> synodic =
                        orbitsim::make_synodic_frame(state_at(*body_a), body_a->mass_kg, state_at(*body_b), body_b->mass_kg);
                if (!synodic.has_value())
                {
                    return false;
                }
                out_frame = *synodic;
                break;
            }

            case orbitsim::TrajectoryFrameType::LVLH:
            {
                if (cache.massive_bodies.empty())
                {
                    return false;
                }

                const auto player_lookup = build_prediction_player_lookup();
                if (!player_lookup)
                {
                    return false;
                }

                const std::optional<orbitsim::State> target_state =
                        player_lookup(frame_spec.target_spacecraft_id, reference_time_s);
                if (!target_state.has_value())
                {
                    return false;
                }

                const orbitsim::MassiveBody *primary =
                        find_massive_body(cache.massive_bodies, frame_spec.primary_body_id);
                if (!primary)
                {
                    return false;
                }

                const std::optional<orbitsim::RotatingFrame> lvlh =
                        orbitsim::make_lvlh_frame(state_at(*primary), *target_state);
                if (!lvlh.has_value())
                {
                    return false;
                }
                out_frame = *lvlh;
                break;
            }
        }

        return true;
    }

    bool GameplayState::build_prediction_display_transform(const OrbitPredictionCache &cache,
                                                           WorldVec3 &out_origin_world,
                                                           glm::dmat3 &out_frame_to_world,
                                                           double display_time_s) const
    {
        out_origin_world = _scenario_config.system_center;
        out_frame_to_world = glm::dmat3(1.0);
        const double reference_time_s = resolve_prediction_display_reference_time_s(cache, display_time_s);
        const WorldVec3 world_ref_world = prediction_world_reference_body_world();

        const auto state_at = [&](const orbitsim::MassiveBody &body) -> orbitsim::State {
            if (cache.shared_ephemeris && !cache.shared_ephemeris->empty())
            {
                return cache.shared_ephemeris->body_state_at_by_id(body.id, reference_time_s);
            }
            return body.state;
        };

        orbitsim::State world_ref_state{};
        bool have_world_ref_state = false;
        if (_orbitsim)
        {
            if (const CelestialBodyInfo *world_ref_info = _orbitsim->world_reference_body())
            {
                if (const orbitsim::MassiveBody *world_ref_body = find_massive_body(cache.massive_bodies, world_ref_info->sim_id))
                {
                    world_ref_state = state_at(*world_ref_body);
                    have_world_ref_state = true;
                }
                else if (const orbitsim::MassiveBody *world_ref_sim = _orbitsim->world_reference_sim_body())
                {
                    world_ref_state = world_ref_sim->state;
                    have_world_ref_state = true;
                }
            }
        }

        orbitsim::RotatingFrame frame{};
        if (!build_prediction_display_frame(cache, frame, reference_time_s))
        {
            return false;
        }

        out_frame_to_world = glm::dmat3(glm::dvec3(frame.ex_i.x, frame.ex_i.y, frame.ex_i.z),
                                        glm::dvec3(frame.ey_i.x, frame.ey_i.y, frame.ey_i.z),
                                        glm::dvec3(frame.ez_i.x, frame.ez_i.y, frame.ez_i.z));

        const glm::dvec3 world_ref_position_m =
                have_world_ref_state ? glm::dvec3(world_ref_state.position_m) : glm::dvec3(0.0);
        out_origin_world = world_ref_world + WorldVec3(glm::dvec3(frame.origin_position_m) - world_ref_position_m);
        return true;
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
        WorldVec3 origin_world{0.0};
        glm::dmat3 frame_to_world(1.0);
        if (!build_prediction_display_transform(cache, origin_world, frame_to_world, display_time_s))
        {
            return _scenario_config.system_center + WorldVec3(glm::dvec3(sample.position_m));
        }

        return origin_world + WorldVec3(frame_to_world * glm::dvec3(sample.position_m));
    }

    WorldVec3 GameplayState::prediction_sample_hermite_world(const OrbitPredictionCache &cache,
                                                             const orbitsim::TrajectorySample &a,
                                                             const orbitsim::TrajectorySample &b,
                                                             const double t_s,
                                                             double display_time_s) const
    {
        const double ta = a.t_s;
        const double tb = b.t_s;
        const double h = tb - ta;
        if (!std::isfinite(h) || !(h > 0.0))
        {
            return prediction_sample_position_world(cache, a, display_time_s);
        }

        double u = (t_s - ta) / h;
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
        const glm::dvec3 p0 = glm::dvec3(a.position_m);
        const glm::dvec3 p1 = glm::dvec3(b.position_m);
        const glm::dvec3 m0 = glm::dvec3(a.velocity_mps) * h;
        const glm::dvec3 m1 = glm::dvec3(b.velocity_mps) * h;
        const glm::dvec3 local = (h00 * p0) + (h10 * m0) + (h01 * p1) + (h11 * m1);

        WorldVec3 origin_world{0.0};
        glm::dmat3 frame_to_world(1.0);
        if (!build_prediction_display_transform(cache, origin_world, frame_to_world, display_time_s))
        {
            return _scenario_config.system_center + WorldVec3(local);
        }

        return origin_world + WorldVec3(frame_to_world * local);
    }

    WorldVec3 GameplayState::prediction_frame_origin_world(const OrbitPredictionCache &cache,
                                                           double display_time_s) const
    {
        WorldVec3 origin_world{0.0};
        glm::dmat3 frame_to_world(1.0);
        if (build_prediction_display_transform(cache, origin_world, frame_to_world, display_time_s))
        {
            return origin_world;
        }

        return _scenario_config.system_center;
    }

    void GameplayState::refresh_prediction_derived_cache(PredictionTrackState &track,
                                                         double display_time_s)
    {
        if (!track.cache.valid || track.cache.trajectory_inertial.size() < 2)
        {
            return;
        }

        const orbitsim::TrajectoryFrameSpec resolved_frame_spec =
                resolve_prediction_display_frame_spec(track.cache, display_time_s);
        const bool rebuild_frame_cache =
                !track.cache.resolved_frame_spec_valid ||
                !same_frame_spec(track.cache.resolved_frame_spec, resolved_frame_spec) ||
                track.cache.trajectory_frame.size() < 2;

        if (rebuild_frame_cache)
        {
            std::vector<orbitsim::TrajectorySegment> player_lookup_segments;
            if (const PredictionTrackState *player_track = player_prediction_track())
            {
                if (!player_track->cache.trajectory_segments_inertial_planned.empty())
                {
                    player_lookup_segments = player_track->cache.trajectory_segments_inertial_planned;
                }
                else if (!player_track->cache.trajectory_segments_inertial.empty())
                {
                    player_lookup_segments = player_track->cache.trajectory_segments_inertial;
                }
            }

            if (!PredictionCacheInternal::rebuild_prediction_frame_cache(
                        track.cache,
                        resolved_frame_spec,
                        player_lookup_segments,
                        {},
                        &track.derived_diagnostics))
            {
                track.cache.valid = false;
                track.cache.resolved_frame_spec_valid = false;
                track.cache.metrics_valid = false;
                return;
            }
        }

        double analysis_time_s = display_time_s;
        if (!std::isfinite(analysis_time_s))
        {
            analysis_time_s = _orbitsim ? _orbitsim->sim.time_s() : track.cache.build_time_s;
        }
        const orbitsim::BodyId analysis_body_id =
                resolve_prediction_analysis_body_id(track.cache, track.key, analysis_time_s);
        const bool rebuild_metrics =
                rebuild_frame_cache ||
                !track.cache.metrics_valid ||
                track.cache.metrics_body_id != analysis_body_id;
        if (!rebuild_metrics)
        {
            return;
        }

        PredictionCacheInternal::rebuild_prediction_metrics(
                track.cache,
                _orbitsim ? _orbitsim->sim.config() : orbitsim::GameSimulation::Config{},
                analysis_body_id);
    }

    void GameplayState::refresh_all_prediction_derived_caches()
    {
        rebuild_prediction_frame_options();
        rebuild_prediction_analysis_options();
        for (PredictionTrackState &track : _prediction_tracks)
        {
            refresh_prediction_derived_cache(track);
        }
    }

    WorldVec3 GameplayState::prediction_reference_body_world() const
    {
        return prediction_world_reference_body_world();
    }

} // namespace Game
