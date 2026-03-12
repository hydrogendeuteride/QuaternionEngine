#include "game/states/gameplay/gameplay_state.h"

#include "game/orbit/orbit_prediction_math.h"

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
    using detail::finite_vec3;

    namespace
    {
        constexpr orbitsim::SpacecraftId kPlayerDisplayTargetSpacecraftId =
                static_cast<orbitsim::SpacecraftId>(0x7000'0001u);

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

        orbitsim::State body_state_for_segment_transform(const OrbitPredictionCache &cache,
                                                         const orbitsim::MassiveBody &body,
                                                         const double t_s)
        {
            if (cache.shared_ephemeris && !cache.shared_ephemeris->empty())
            {
                return cache.shared_ephemeris->body_state_at_by_id(body.id, t_s);
            }
            return body.state;
        }

        std::vector<orbitsim::TrajectorySegment> transform_segments_to_body_centered_inertial(
                const OrbitPredictionCache &cache,
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

                const orbitsim::State ref_start = body_state_for_segment_transform(cache, reference_body, t0_s);
                const orbitsim::State ref_end = body_state_for_segment_transform(cache, reference_body, t1_s);

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

    orbitsim::SpacecraftStateLookup GameplayState::build_prediction_player_lookup() const
    {
        const PredictionTrackState *player_track = player_prediction_track();
        if (!player_track || !player_track->cache.valid)
        {
            return nullptr;
        }

        const std::vector<orbitsim::TrajectorySample> *trajectory = nullptr;
        if (player_track->cache.trajectory_inertial_planned.size() >= 2)
        {
            trajectory = &player_track->cache.trajectory_inertial_planned;
        }
        else if (player_track->cache.trajectory_inertial.size() >= 2)
        {
            trajectory = &player_track->cache.trajectory_inertial;
        }
        if (!trajectory)
        {
            return nullptr;
        }

        return [this, trajectory](const orbitsim::SpacecraftId spacecraft_id, const double t_s)
                -> std::optional<orbitsim::State> {
            if (spacecraft_id != kPlayerDisplayTargetSpacecraftId)
            {
                return std::nullopt;
            }

            orbitsim::State sampled{};
            if (!sample_prediction_inertial_state(*trajectory, t_s, sampled))
            {
                return std::nullopt;
            }
            return sampled;
        };
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

        const auto body_position_at = [&](const std::size_t i) -> orbitsim::Vec3 {
            const orbitsim::MassiveBody &body = cache.massive_bodies[i];
            if (cache.shared_ephemeris && !cache.shared_ephemeris->empty())
            {
                return cache.shared_ephemeris->body_state_at_by_id(body.id, reference_time_s).position_m;
            }
            return body.state.position_m;
        };

        const std::size_t primary_index = orbitsim::auto_select_primary_index(
                cache.massive_bodies,
                target_state->position_m,
                body_position_at,
                _orbitsim ? _orbitsim->sim.config().softening_length_m : 0.0);
        if (primary_index < cache.massive_bodies.size())
        {
            resolved.primary_body_id = cache.massive_bodies[primary_index].id;
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
                        .spec = orbitsim::TrajectoryFrameSpec::lvlh(kPlayerDisplayTargetSpacecraftId),
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
                                                                        const double query_time_s) const
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
        if (!sample_prediction_inertial_state(cache.trajectory_inertial, query_time_s, query_state))
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
                candidates,
                query_state.position_m,
                body_position_at,
                _orbitsim ? _orbitsim->sim.config().softening_length_m : 0.0);
        if (primary_index >= candidates.size())
        {
            return orbitsim::kInvalidBodyId;
        }

        return candidates[primary_index].id;
    }

    bool GameplayState::build_prediction_display_transform(const OrbitPredictionCache &cache,
                                                           WorldVec3 &out_origin_world,
                                                           glm::dmat3 &out_frame_to_world,
                                                           double display_time_s) const
    {
        out_origin_world = _scenario_config.system_center;
        out_frame_to_world = glm::dmat3(1.0);
        double reference_time_s = display_time_s;
        if (!std::isfinite(reference_time_s))
        {
            reference_time_s = _orbitsim ? _orbitsim->sim.time_s() : cache.build_time_s;
        }
        const WorldVec3 world_ref_world = prediction_world_reference_body_world();
        const orbitsim::TrajectoryFrameSpec frame_spec =
                resolve_prediction_display_frame_spec(cache, reference_time_s);

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
                frame = orbitsim::make_body_centered_inertial_frame(state_at(*body));
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
                frame = *body_fixed;
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
                frame = *synodic;
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
                frame = *lvlh;
                break;
            }
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
            track.cache.trajectory_frame.clear();
            track.cache.trajectory_frame_planned.clear();
            track.cache.trajectory_segments_frame.clear();
            track.cache.trajectory_segments_frame_planned.clear();

            if (resolved_frame_spec.type == orbitsim::TrajectoryFrameType::Inertial)
            {
                track.cache.trajectory_frame = track.cache.trajectory_inertial;
                track.cache.trajectory_frame_planned = track.cache.trajectory_inertial_planned;
                track.cache.trajectory_segments_frame = track.cache.trajectory_segments_inertial;
                track.cache.trajectory_segments_frame_planned = track.cache.trajectory_segments_inertial_planned;
            }
            else if (resolved_frame_spec.type == orbitsim::TrajectoryFrameType::BodyCenteredInertial)
            {
                if (!track.cache.shared_ephemeris || track.cache.shared_ephemeris->empty())
                {
                    track.cache.valid = false;
                    track.cache.resolved_frame_spec_valid = false;
                    track.cache.metrics_valid = false;
                    return;
                }

                const orbitsim::MassiveBody *reference_body =
                        find_massive_body(track.cache.massive_bodies, resolved_frame_spec.primary_body_id);
                if (!reference_body)
                {
                    track.cache.valid = false;
                    track.cache.resolved_frame_spec_valid = false;
                    track.cache.metrics_valid = false;
                    return;
                }

                const auto player_lookup = build_prediction_player_lookup();
                track.cache.trajectory_frame = orbitsim::trajectory_to_frame_spec(
                        track.cache.trajectory_inertial,
                        *track.cache.shared_ephemeris,
                        track.cache.massive_bodies,
                        resolved_frame_spec,
                        player_lookup);
                if (!track.cache.trajectory_inertial_planned.empty())
                {
                    track.cache.trajectory_frame_planned = orbitsim::trajectory_to_frame_spec(
                            track.cache.trajectory_inertial_planned,
                            *track.cache.shared_ephemeris,
                            track.cache.massive_bodies,
                            resolved_frame_spec,
                            player_lookup);
                }

                track.cache.trajectory_segments_frame =
                        transform_segments_to_body_centered_inertial(
                                track.cache,
                                track.cache.trajectory_segments_inertial,
                                *reference_body);
                if (!track.cache.trajectory_segments_inertial_planned.empty())
                {
                    track.cache.trajectory_segments_frame_planned =
                            transform_segments_to_body_centered_inertial(
                                    track.cache,
                                    track.cache.trajectory_segments_inertial_planned,
                                    *reference_body);
                }
            }
            else
            {
                if (!track.cache.shared_ephemeris || track.cache.shared_ephemeris->empty())
                {
                    track.cache.valid = false;
                    track.cache.resolved_frame_spec_valid = false;
                    track.cache.metrics_valid = false;
                    return;
                }

                const auto player_lookup = build_prediction_player_lookup();
                track.cache.trajectory_frame = orbitsim::trajectory_to_frame_spec(
                        track.cache.trajectory_inertial,
                        *track.cache.shared_ephemeris,
                        track.cache.massive_bodies,
                        resolved_frame_spec,
                        player_lookup);
                if (!track.cache.trajectory_inertial_planned.empty())
                {
                    track.cache.trajectory_frame_planned = orbitsim::trajectory_to_frame_spec(
                            track.cache.trajectory_inertial_planned,
                            *track.cache.shared_ephemeris,
                            track.cache.massive_bodies,
                            resolved_frame_spec,
                            player_lookup);
                }
            }

            if (track.cache.trajectory_frame.size() < 2)
            {
                track.cache.valid = false;
                track.cache.resolved_frame_spec_valid = false;
                track.cache.metrics_valid = false;
                return;
            }

            if (track.cache.trajectory_segments_frame.empty())
            {
                track.cache.trajectory_segments_frame = trajectory_segments_from_samples(track.cache.trajectory_frame);
            }
            if (!track.cache.trajectory_frame_planned.empty() &&
                track.cache.trajectory_segments_frame_planned.empty())
            {
                track.cache.trajectory_segments_frame_planned =
                        trajectory_segments_from_samples(track.cache.trajectory_frame_planned);
            }

            track.cache.resolved_frame_spec = resolved_frame_spec;
            track.cache.resolved_frame_spec_valid = true;
            track.cache.metrics_valid = false;
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

        track.cache.altitude_km.clear();
        track.cache.speed_kmps.clear();
        track.cache.semi_major_axis_m = 0.0;
        track.cache.eccentricity = 0.0;
        track.cache.orbital_period_s = 0.0;
        track.cache.periapsis_alt_km = 0.0;
        track.cache.apoapsis_alt_km = std::numeric_limits<double>::infinity();
        track.cache.metrics_body_id = analysis_body_id;
        track.cache.metrics_valid = true;

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
        if (track.cache.resolved_frame_spec_valid &&
            track.cache.resolved_frame_spec.type == orbitsim::TrajectoryFrameType::BodyCenteredInertial &&
            analysis_body_id == track.cache.resolved_frame_spec.primary_body_id)
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
        if (rel_samples.size() < 2)
        {
            return;
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
