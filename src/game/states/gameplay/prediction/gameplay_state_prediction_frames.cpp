#include "game/states/gameplay/gameplay_state.h"
#include "game/states/gameplay/prediction/gameplay_prediction_cache_internal.h"
#include "game/states/gameplay/prediction/runtime/gameplay_state_prediction_runtime_internal.h"

#include "game/orbit/orbit_prediction_math.h"
#include "game/orbit/orbit_prediction_tuning.h"

#include "orbitsim/coordinate_frames.hpp"
#include "orbitsim/frame_utils.hpp"
#include "orbitsim/math.hpp"
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
        OrbitPredictionService::Result build_prediction_solver_result_from_cache(
                const uint64_t track_id,
                const OrbitPredictionCache &cache)
        {
            OrbitPredictionService::Result result{};
            const auto &base_samples = cache.resolved_trajectory_inertial();
            const auto &base_segments = cache.resolved_trajectory_segments_inertial();
            result.track_id = track_id;
            result.generation_id = cache.generation_id;
            result.maneuver_plan_revision = cache.maneuver_plan_revision;
            result.maneuver_plan_signature_valid = cache.maneuver_plan_signature_valid;
            result.maneuver_plan_signature = cache.maneuver_plan_signature;
            result.valid = base_samples.size() >= 2 && !base_segments.empty();
            result.solve_quality = OrbitPredictionService::SolveQuality::Full;
            result.build_time_s = cache.build_time_s;
            if (cache.shared_solver_core_data)
            {
                result.set_shared_core_data(cache.shared_solver_core_data);
            }
            else
            {
                result.shared_ephemeris = cache.shared_ephemeris;
                result.massive_bodies = cache.massive_bodies;
                result.trajectory_inertial = cache.trajectory_inertial;
                result.trajectory_segments_inertial = cache.trajectory_segments_inertial;
            }
            result.trajectory_inertial_planned = cache.trajectory_inertial_planned;
            result.trajectory_segments_inertial_planned = cache.trajectory_segments_inertial_planned;
            result.maneuver_previews = cache.maneuver_previews;
            return result;
        }

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
        const bool freeze_planned_synodic =
                prediction_frame_is_lagrange_sensitive(frame_spec) &&
                _maneuver_nodes_enabled &&
                !_maneuver_state.nodes.empty() &&
                !dragging_maneuver_axis &&
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
            const auto &ephemeris = cache ? cache->resolved_shared_ephemeris()
                                          : OrbitPredictionService::SharedCelestialEphemeris{};
            if (ephemeris && !ephemeris->empty())
            {
                return ephemeris->body_state_at_by_id(body.id, query_time_s).position_m;
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
        return PredictionCacheInternal::sample_prediction_inertial_state(trajectory, query_time_s, out_state);
    }

    orbitsim::SpacecraftStateLookup GameplayState::build_prediction_player_lookup() const
    {
        const OrbitPredictionCache *player_cache = effective_prediction_cache(player_prediction_track());
        if (!player_cache)
        {
            return nullptr;
        }

        const std::vector<orbitsim::TrajectorySegment> *trajectory_segments = nullptr;
        if (!player_cache->trajectory_segments_inertial_planned.empty())
        {
            trajectory_segments = &player_cache->trajectory_segments_inertial_planned;
        }
        else if (!player_cache->resolved_trajectory_segments_inertial().empty())
        {
            trajectory_segments = &player_cache->resolved_trajectory_segments_inertial();
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
        const auto &bodies = cache.resolved_massive_bodies();
        if (resolved.type != orbitsim::TrajectoryFrameType::LVLH ||
            resolved.primary_body_id != orbitsim::kInvalidBodyId ||
            bodies.empty())
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
                bodies,
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
        ++_prediction_display_frame_revision;
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

        const auto &bodies = cache.resolved_massive_bodies();
        const auto &base_segments = cache.resolved_trajectory_segments_inertial();
        const auto &base_samples = cache.resolved_trajectory_inertial();
        if (bodies.empty())
        {
            return orbitsim::kInvalidBodyId;
        }

        orbitsim::State query_state{};
        if (!PredictionCacheInternal::sample_prediction_inertial_state(base_segments,
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
        const auto &ephemeris = cache.resolved_shared_ephemeris();
        const auto &bodies = cache.resolved_massive_bodies();
        const auto state_at = [&](const orbitsim::MassiveBody &body) -> orbitsim::State {
            if (ephemeris && !ephemeris->empty())
            {
                return ephemeris->body_state_at_by_id(body.id, reference_time_s);
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
                        find_massive_body(bodies, frame_spec.primary_body_id);
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
                        find_massive_body(bodies, frame_spec.primary_body_id);
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
                        find_massive_body(bodies, frame_spec.primary_body_id);
                const orbitsim::MassiveBody *body_b =
                        find_massive_body(bodies, frame_spec.secondary_body_id);
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
                if (bodies.empty())
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
                        find_massive_body(bodies, frame_spec.primary_body_id);
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
        const auto &ephemeris = cache.resolved_shared_ephemeris();
        const auto &bodies = cache.resolved_massive_bodies();

        const auto state_at = [&](const orbitsim::MassiveBody &body) -> orbitsim::State {
            if (ephemeris && !ephemeris->empty())
            {
                return ephemeris->body_state_at_by_id(body.id, reference_time_s);
            }
            return body.state;
        };

        orbitsim::State world_ref_state{};
        bool have_world_ref_state = false;
        if (_orbitsim)
        {
            if (const CelestialBodyInfo *world_ref_info = _orbitsim->world_reference_body())
            {
                if (const orbitsim::MassiveBody *world_ref_body = find_massive_body(bodies, world_ref_info->sim_id))
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
        const glm::dvec3 local = OrbitPredictionMath::sample_pair_position_m(a, b, t_s);

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

    void GameplayState::mark_prediction_derived_request_submitted(
            PredictionTrackState &track,
            const OrbitPredictionDerivedService::Request &request)
    {
        track.derived_request_pending = true;
        track.latest_requested_derived_generation_id = request.generation_id;
        track.latest_requested_derived_display_frame_key = request.display_frame_key;
        track.latest_requested_derived_display_frame_revision = request.display_frame_revision;
        track.latest_requested_derived_analysis_body_id = request.analysis_body_id;
        track.latest_requested_derived_publish_stage = request.solver_result.publish_stage;
    }

    bool GameplayState::request_prediction_derived_refresh(PredictionTrackState &track, double display_time_s)
    {
        if (track.cache.resolved_trajectory_inertial().size() < 2 ||
            track.cache.resolved_trajectory_segments_inertial().empty())
        {
            return false;
        }

        if (!PredictionRuntimeDetail::latest_solver_generation_published(track))
        {
            return false;
        }

        const orbitsim::TrajectoryFrameSpec resolved_frame_spec =
                resolve_prediction_display_frame_spec(track.cache, display_time_s);
        const uint64_t display_frame_key = prediction_display_frame_key(resolved_frame_spec);
        const uint64_t display_frame_revision = _prediction_display_frame_revision;
        double analysis_time_s = display_time_s;
        if (!std::isfinite(analysis_time_s))
        {
            analysis_time_s = _orbitsim ? _orbitsim->sim.time_s() : track.cache.build_time_s;
        }

        const orbitsim::BodyId analysis_body_id =
                resolve_prediction_analysis_body_id(track.cache, track.key, analysis_time_s, track.auto_primary_body_id);
        if (track.derived_request_pending &&
            track.latest_requested_derived_generation_id == track.cache.generation_id &&
            track.latest_requested_derived_display_frame_key == display_frame_key &&
            track.latest_requested_derived_display_frame_revision == display_frame_revision &&
            track.latest_requested_derived_analysis_body_id == analysis_body_id)
        {
            return false;
        }

        WorldVec3 build_pos_world = track.cache.build_pos_world;
        glm::dvec3 build_vel_world = track.cache.build_vel_world;
        glm::vec3 build_vel_local{0.0f};
        (void) get_prediction_subject_world_state(track.key, build_pos_world, build_vel_world, build_vel_local);

        std::vector<orbitsim::TrajectorySegment> player_lookup_segments;
        if (prediction_subject_is_player(track.key))
        {
            if (!track.cache.trajectory_segments_inertial_planned.empty())
            {
                player_lookup_segments = track.cache.trajectory_segments_inertial_planned;
            }
            else
            {
                player_lookup_segments = track.cache.resolved_trajectory_segments_inertial();
            }
        }
        else if (const PredictionTrackState *player_track = player_prediction_track())
        {
            if (const OrbitPredictionCache *player_cache = effective_prediction_cache(player_track))
            {
                if (!player_cache->trajectory_segments_inertial_planned.empty())
                {
                    player_lookup_segments = player_cache->trajectory_segments_inertial_planned;
                }
                else if (!player_cache->resolved_trajectory_segments_inertial().empty())
                {
                    player_lookup_segments = player_cache->resolved_trajectory_segments_inertial();
                }
            }
        }

        OrbitPredictionDerivedService::Request derived_request{};
        derived_request.track_id = track.key.track_id();
        derived_request.generation_id = track.cache.generation_id;
        derived_request.maneuver_plan_revision = track.cache.maneuver_plan_revision;
        derived_request.maneuver_plan_signature_valid = track.cache.maneuver_plan_signature_valid;
        derived_request.maneuver_plan_signature = track.cache.maneuver_plan_signature;
        derived_request.priority = PredictionRuntimeDetail::classify_prediction_subject_priority(
                _prediction_selection,
                track.key,
                track.is_celestial);
        derived_request.solver_result = build_prediction_solver_result_from_cache(track.key.track_id(), track.cache);
        derived_request.build_pos_world = build_pos_world;
        derived_request.build_vel_world = build_vel_world;
        derived_request.sim_config = _orbitsim ? _orbitsim->sim.config() : orbitsim::GameSimulation::Config{};
        derived_request.resolved_frame_spec = resolved_frame_spec;
        derived_request.display_frame_key = display_frame_key;
        derived_request.display_frame_revision = display_frame_revision;
        derived_request.analysis_body_id = analysis_body_id;
        derived_request.player_lookup_segments_inertial = std::move(player_lookup_segments);
        _prediction_derived_service.request(derived_request);
        mark_prediction_derived_request_submitted(track, derived_request);
        return true;
    }

    bool GameplayState::prediction_track_has_current_derived_cache(const PredictionTrackState &track,
                                                                   double display_time_s) const
    {
        if (!track.cache.valid || track.cache.trajectory_frame.size() < 2 || track.cache.trajectory_segments_frame.empty())
        {
            return false;
        }

        const orbitsim::TrajectoryFrameSpec resolved_frame_spec =
                resolve_prediction_display_frame_spec(track.cache, display_time_s);
        const uint64_t display_frame_key = prediction_display_frame_key(resolved_frame_spec);
        const uint64_t display_frame_revision = _prediction_display_frame_revision;
        return track.cache.resolved_frame_spec_valid &&
               track.cache.display_frame_key == display_frame_key &&
               track.cache.display_frame_revision == display_frame_revision;
    }

    void GameplayState::refresh_prediction_derived_cache(PredictionTrackState &track,
                                                         double display_time_s)
    {
        if (track.cache.resolved_trajectory_inertial().size() < 2)
        {
            return;
        }

        const orbitsim::TrajectoryFrameSpec resolved_frame_spec =
                resolve_prediction_display_frame_spec(track.cache, display_time_s);
        const uint64_t display_frame_key = prediction_display_frame_key(resolved_frame_spec);
        const uint64_t display_frame_revision = _prediction_display_frame_revision;
        const auto cache_frame_matches = [&](const OrbitPredictionCache &cache) {
            return cache.display_frame_key == display_frame_key &&
                   cache.display_frame_revision == display_frame_revision;
        };
        if ((track.pick_cache.generation_id != 0u || track.pick_cache.base_valid || track.pick_cache.planned_valid) &&
            !cache_frame_matches(track.cache))
        {
            track.pick_cache.clear();
        }
        if (prediction_track_has_current_derived_cache(track, display_time_s))
        {
            return;
        }

        const double analysis_time_s =
                std::isfinite(display_time_s) ? display_time_s : (_orbitsim ? _orbitsim->sim.time_s() : track.cache.build_time_s);
        const orbitsim::BodyId analysis_body_id =
                resolve_prediction_analysis_body_id(track.cache, track.key, analysis_time_s, track.auto_primary_body_id);
        if (analysis_body_id != orbitsim::kInvalidBodyId)
        {
            track.auto_primary_body_id = analysis_body_id;
        }
        (void) resolved_frame_spec;
        (void) request_prediction_derived_refresh(track, display_time_s);
    }

    void GameplayState::refresh_all_prediction_derived_caches()
    {
        rebuild_prediction_frame_options();
        rebuild_prediction_analysis_options();
        _prediction_derived_service.reset();
        for (PredictionTrackState &track : _prediction_tracks)
        {
            track.derived_request_pending = false;
            track.latest_requested_derived_generation_id = 0;
            track.latest_requested_derived_display_frame_key = 0;
            track.latest_requested_derived_display_frame_revision = 0;
            track.latest_requested_derived_analysis_body_id = orbitsim::kInvalidBodyId;
            track.latest_requested_derived_publish_stage = OrbitPredictionService::PublishStage::Final;
            // Display-frame artifacts are no longer trustworthy after a global derived reset.
            track.preview_overlay.clear();
            track.full_stream_overlay.clear();
            track.pick_cache.clear();
            (void) request_prediction_derived_refresh(track);
        }
    }

    WorldVec3 GameplayState::prediction_reference_body_world() const
    {
        return prediction_world_reference_body_world();
    }

} // namespace Game
