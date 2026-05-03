#include "game/states/gameplay/gameplay_state.h"
#include "game/states/gameplay/prediction/gameplay_prediction_adapter.h"
#include "game/states/gameplay/prediction/prediction_frame_controller.h"
#include "game/states/gameplay/prediction/prediction_frame_context_builder.h"
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

    const CelestialBodyInfo *GameplayPredictionAdapter::find_celestial_body_info(const orbitsim::BodyId body_id) const
    {
        return PredictionFrameContextBuilder(context()).find_celestial_body_info(body_id);
    }

    const orbitsim::MassiveBody *GameplayPredictionAdapter::find_massive_body(const std::vector<orbitsim::MassiveBody> &bodies,
                                                                  const orbitsim::BodyId body_id) const
    {
        return PredictionFrameResolver::find_massive_body(bodies, body_id);
    }

    bool GameplayPredictionAdapter::prediction_frame_is_lagrange_sensitive(const orbitsim::TrajectoryFrameSpec &spec) const
    {
        return PredictionFrameResolver::is_lagrange_sensitive(spec);
    }

    double GameplayPredictionAdapter::resolve_prediction_display_reference_time_s(const OrbitPredictionCache &cache,
                                                                      const double display_time_s) const
    {
        return PredictionFrameResolver::resolve_display_reference_time_s(
                build_prediction_frame_resolver_context(),
                cache,
                display_time_s);
    }

    orbitsim::BodyId GameplayPredictionAdapter::select_prediction_primary_body_id(const std::vector<orbitsim::MassiveBody> &bodies,
                                                                      const OrbitPredictionCache *cache,
                                                                      const orbitsim::Vec3 &query_pos_m,
                                                                      const double query_time_s,
                                                                      const orbitsim::BodyId preferred_body_id) const
    {
        return PredictionFrameContextBuilder(context()).select_prediction_primary_body_id(
                bodies,
                cache,
                query_pos_m,
                query_time_s,
                preferred_body_id);
    }

    WorldVec3 GameplayPredictionAdapter::prediction_body_world_position(const orbitsim::BodyId body_id,
                                                             const OrbitPredictionCache *cache,
                                                             double query_time_s) const
    {
        return PredictionFrameContextBuilder(context()).prediction_body_world_position(body_id, cache, query_time_s);
    }

    bool GameplayPredictionAdapter::sample_prediction_inertial_state(const std::vector<orbitsim::TrajectorySample> &trajectory,
        const double query_time_s,
        orbitsim::State &out_state) const
    {
        return PredictionFrameContextBuilder(context()).sample_prediction_inertial_state(
                trajectory,
                query_time_s,
                out_state);
    }

    orbitsim::SpacecraftStateLookup GameplayPredictionAdapter::build_prediction_player_lookup() const
    {
        return PredictionFrameContextBuilder(context()).build_prediction_player_lookup();
    }

    PredictionFrameResolverContext GameplayPredictionAdapter::build_prediction_frame_resolver_context() const
    {
        return PredictionFrameContextBuilder(context()).build_prediction_frame_resolver_context();
    }

    PredictionFrameControllerContext GameplayPredictionAdapter::build_prediction_frame_controller_context() const
    {
        PredictionFrameControllerContext context{};
        context.derived_service = const_cast<OrbitPredictionDerivedService *>(&_state._prediction->derived_service());
        context.selection = _state._prediction->state().selection;
        context.display_frame_revision = _state._prediction->state().display_frame_revision;
        context.sim_config = _state._orbit.scenario_owner() ? _state._orbit.scenario_owner()->sim.config() : orbitsim::GameSimulation::Config{};
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

    orbitsim::TrajectoryFrameSpec GameplayPredictionAdapter::resolve_prediction_display_frame_spec(const OrbitPredictionCache &cache,
                                                                                        double display_time_s) const
    {
        return PredictionFrameContextBuilder(context()).resolve_prediction_display_frame_spec(cache, display_time_s);
    }

    orbitsim::TrajectoryFrameSpec GameplayPredictionAdapter::default_prediction_frame_spec() const
    {
        if (_state._orbit.scenario_owner())
        {
            if (const CelestialBodyInfo *world_ref = _state._orbit.scenario_owner()->world_reference_body())
            {
                return orbitsim::TrajectoryFrameSpec::body_centered_inertial(world_ref->sim_id);
            }
        }

        return orbitsim::TrajectoryFrameSpec::inertial();
    }

    void GameplayPredictionAdapter::rebuild_prediction_analysis_options()
    {
        std::vector<PredictionAnalysisOption> options;
        options.push_back(PredictionAnalysisOption{
                .spec = {},
                .label = "Auto Primary (BCI)",
        });

        if (_state._orbit.scenario_owner())
        {
            for (const CelestialBodyInfo &body : _state._orbit.scenario_owner()->bodies)
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

        _state._prediction->state().analysis_selection.options = std::move(options);
        if (_state._prediction->state().analysis_selection.selected_index < 0)
        {
            _state._prediction->state().analysis_selection.spec = {};
        }

        int selected_index = -1;
        for (std::size_t i = 0; i < _state._prediction->state().analysis_selection.options.size(); ++i)
        {
            if (same_analysis_spec(_state._prediction->state().analysis_selection.options[i].spec, _state._prediction->state().analysis_selection.spec))
            {
                selected_index = static_cast<int>(i);
                break;
            }
        }

        if (selected_index < 0)
        {
            _state._prediction->state().analysis_selection.spec = {};
            selected_index = 0;
        }

        _state._prediction->state().analysis_selection.selected_index = selected_index;
    }

    bool GameplayPredictionAdapter::set_prediction_analysis_spec(const PredictionAnalysisSpec &spec)
    {
        if (same_analysis_spec(_state._prediction->state().analysis_selection.spec, spec))
        {
            return false;
        }

        _state._prediction->state().analysis_selection.spec = spec;
        refresh_all_prediction_derived_caches();
        sync_prediction_dirty_flag();
        return true;
    }

    bool GameplayPredictionAdapter::set_prediction_frame_spec(const orbitsim::TrajectoryFrameSpec &spec)
    {
        if (PredictionFrameResolver::same_frame_spec(_state._prediction->state().frame_selection.spec, spec))
        {
            return false;
        }

        _state._prediction->state().frame_selection.spec = spec;
        ++_state._prediction->state().display_frame_revision;
        refresh_all_prediction_derived_caches();
        sync_prediction_dirty_flag();
        return true;
    }

    void GameplayPredictionAdapter::rebuild_prediction_frame_options()
    {
        std::vector<PredictionFrameOption> options;
        options.push_back(PredictionFrameOption{
                .spec = orbitsim::TrajectoryFrameSpec::inertial(),
                .label = "Inertial (Barycentric)",
        });

        if (_state._orbit.scenario_owner())
        {
            const double display_time_s = _state._orbit.scenario_owner()->sim.time_s();
            const auto player_lookup = build_prediction_player_lookup();
            const CelestialBodyInfo *world_ref = _state._orbit.scenario_owner()->world_reference_body();

            for (const CelestialBodyInfo &body : _state._orbit.scenario_owner()->bodies)
            {
                options.push_back(PredictionFrameOption{
                        .spec = orbitsim::TrajectoryFrameSpec::body_centered_inertial(body.sim_id),
                        .label = body.name + " BCI",
                });

                if (const orbitsim::MassiveBody *sim_body = _state._orbit.scenario_owner()->sim.body_by_id(body.sim_id))
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
                for (const CelestialBodyInfo &body : _state._orbit.scenario_owner()->bodies)
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

        _state._prediction->state().frame_selection.options = std::move(options);

        if (_state._prediction->state().frame_selection.selected_index < 0)
        {
            _state._prediction->state().frame_selection.spec = default_prediction_frame_spec();
        }

        int selected_index = -1;
        for (std::size_t i = 0; i < _state._prediction->state().frame_selection.options.size(); ++i)
        {
            if (PredictionFrameResolver::same_frame_spec(_state._prediction->state().frame_selection.options[i].spec,
                                                         _state._prediction->state().frame_selection.spec))
            {
                selected_index = static_cast<int>(i);
                break;
            }
        }

        if (selected_index < 0)
        {
            _state._prediction->state().frame_selection.spec = default_prediction_frame_spec();
            for (std::size_t i = 0; i < _state._prediction->state().frame_selection.options.size(); ++i)
            {
                if (PredictionFrameResolver::same_frame_spec(_state._prediction->state().frame_selection.options[i].spec,
                                                             _state._prediction->state().frame_selection.spec))
                {
                    selected_index = static_cast<int>(i);
                    break;
                }
            }
        }

        _state._prediction->state().frame_selection.selected_index = selected_index;
    }

    orbitsim::BodyId GameplayPredictionAdapter::resolve_prediction_analysis_body_id(const OrbitPredictionCache &cache,
                                                                        const PredictionSubjectKey key,
                                                                        const double query_time_s,
                                                                        const orbitsim::BodyId preferred_body_id) const
    {
        return PredictionFrameContextBuilder(context()).resolve_prediction_analysis_body_id(
                cache,
                key,
                query_time_s,
                preferred_body_id);
    }

    bool GameplayPredictionAdapter::build_prediction_display_frame(const OrbitPredictionCache &cache,
                                                       orbitsim::RotatingFrame &out_frame,
                                                       double display_time_s) const
    {
        return PredictionFrameResolver::build_display_frame(
                build_prediction_frame_resolver_context(),
                cache,
                out_frame,
                display_time_s);
    }

    bool GameplayPredictionAdapter::build_prediction_display_transform(const OrbitPredictionCache &cache,
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

    WorldVec3 GameplayPredictionAdapter::prediction_world_reference_body_world() const
    {
        return PredictionFrameContextBuilder(context()).prediction_world_reference_body_world();
    }

    WorldVec3 GameplayPredictionAdapter::prediction_sample_position_world(const OrbitPredictionCache &cache,
                                                              const orbitsim::TrajectorySample &sample,
                                                              double display_time_s) const
    {
        return PredictionFrameResolver::sample_position_world(
                build_prediction_frame_resolver_context(),
                cache,
                sample,
                display_time_s);
    }

    WorldVec3 GameplayPredictionAdapter::prediction_sample_hermite_world(const OrbitPredictionCache &cache,
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

    WorldVec3 GameplayPredictionAdapter::prediction_frame_origin_world(const OrbitPredictionCache &cache,
                                                           double display_time_s) const
    {
        return PredictionFrameResolver::frame_origin_world(
                build_prediction_frame_resolver_context(),
                cache,
                display_time_s);
    }

    void GameplayPredictionAdapter::mark_prediction_derived_request_submitted(
            PredictionTrackState &track,
            const OrbitPredictionDerivedService::Request &request)
    {
        PredictionFrameController::mark_derived_request_submitted(track, request);
    }

    bool GameplayPredictionAdapter::request_prediction_derived_refresh(PredictionTrackState &track, double display_time_s)
    {
        return PredictionFrameController::request_derived_refresh(
                build_prediction_frame_controller_context(),
                track,
                display_time_s);
    }

    bool GameplayPredictionAdapter::prediction_track_has_current_derived_cache(const PredictionTrackState &track,
                                                                   double display_time_s) const
    {
        return PredictionFrameController::has_current_derived_cache(
                build_prediction_frame_controller_context(),
                track,
                display_time_s);
    }

    void GameplayPredictionAdapter::refresh_prediction_derived_cache(PredictionTrackState &track,
                                                         double display_time_s)
    {
        PredictionFrameController::refresh_derived_cache(
                build_prediction_frame_controller_context(),
                track,
                display_time_s);
    }

    void GameplayPredictionAdapter::refresh_all_prediction_derived_caches()
    {
        rebuild_prediction_frame_options();
        rebuild_prediction_analysis_options();
        _state._prediction->reset_derived_service();
        for (PredictionTrackState &track : _state._prediction->state().tracks)
        {
            PredictionFrameController::reset_track_derived_state(track);
            (void) request_prediction_derived_refresh(track);
        }
    }

    WorldVec3 GameplayPredictionAdapter::prediction_reference_body_world() const
    {
        return prediction_world_reference_body_world();
    }

} // namespace Game
