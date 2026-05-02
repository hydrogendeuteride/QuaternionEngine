#pragma once

#include "game/states/gameplay/gameplay_state.h"
#include "game/states/gameplay/prediction/prediction_frame_controller.h"
#include "game/states/gameplay/prediction/prediction_frame_resolver.h"
#include "game/states/gameplay/prediction/runtime/prediction_runtime_context.h"
#include "orbitsim/spacecraft_lookup.hpp"

#include <limits>

namespace Game
{
    namespace PredictionDrawDetail
    {
        struct PredictionGlobalDrawContext;
        struct PredictionTrackDrawContext;
    } // namespace PredictionDrawDetail

    class GameplayPredictionAdapter
    {
    public:
        explicit GameplayPredictionAdapter(GameplayState &state)
            : _state(state)
            , _orbit(state._orbit)
            , _world(state._world)
            , _physics(state._physics)
            , _physics_context(state._physics_context)
            , _scenario_config(state._scenario_config)
            , _orbiters(state._orbit.orbiters())
            , _orbitsim(state._orbit.scenario_owner())
            , _debug_draw_enabled(state._debug_draw_enabled)
            , _prediction(state._prediction)
            , _maneuver(state._maneuver)
            , _fixed_time_s(state._fixed_time_s)
            , _time_warp(state._time_warp)
            , _orbital_physics(state._orbital_physics)
        {
        }

        explicit GameplayPredictionAdapter(const GameplayState &state)
            : GameplayPredictionAdapter(const_cast<GameplayState &>(state))
        {
        }

        PredictionSubjectKey player_prediction_subject_key() const;
        std::vector<PredictionSubjectDescriptor> build_prediction_subject_descriptors() const;
        bool get_player_world_state(WorldVec3 &out_pos_world,
                                    glm::dvec3 &out_vel_world,
                                    glm::vec3 &out_vel_local) const;
        bool get_orbiter_world_state(const OrbiterInfo &orbiter,
                                     WorldVec3 &out_pos_world,
                                     glm::dvec3 &out_vel_world,
                                     glm::vec3 &out_vel_local) const;
        bool get_prediction_subject_world_state(PredictionSubjectKey key,
                                                WorldVec3 &out_pos_world,
                                                glm::dvec3 &out_vel_world,
                                                glm::vec3 &out_vel_local) const;

        void poll_completed_prediction_results();
        void clear_visible_prediction_runtime(const std::vector<PredictionSubjectKey> &visible_subjects);
        void apply_completed_prediction_result(OrbitPredictionService::Result result);
        void apply_completed_prediction_derived_result(OrbitPredictionDerivedService::Result result);
        PredictionRuntimeContext build_prediction_runtime_context() const;
        bool should_rebuild_prediction_track(const PredictionTrackState &track,
                                             double now_s,
                                             float fixed_dt,
                                             bool thrusting,
                                             bool with_maneuvers) const;
        bool request_orbiter_prediction_async(PredictionTrackState &track,
                                              const WorldVec3 &subject_pos_world,
                                              const glm::dvec3 &subject_vel_world,
                                              double now_s,
                                              bool thrusting,
                                              bool with_maneuvers,
                                              bool *out_throttled = nullptr);
        bool request_celestial_prediction_async(PredictionTrackState &track, double now_s);
        void update_orbiter_prediction_track(PredictionTrackState &track,
                                             double now_s,
                                             bool thrusting,
                                             bool with_maneuvers);
        void update_celestial_prediction_track(PredictionTrackState &track, double now_s);

        void rebuild_prediction_frame_options();
        bool set_prediction_frame_spec(const orbitsim::TrajectoryFrameSpec &spec);
        orbitsim::TrajectoryFrameSpec default_prediction_frame_spec() const;
        void rebuild_prediction_analysis_options();
        bool set_prediction_analysis_spec(const PredictionAnalysisSpec &spec);
        WorldVec3 prediction_reference_body_world() const;
        bool prediction_subject_thrust_applied_this_tick(PredictionSubjectKey key) const;
        void rebuild_prediction_subjects();
        void sync_prediction_dirty_flag();
        std::vector<PredictionSubjectKey> collect_visible_prediction_subjects() const;
        double prediction_future_window_s(PredictionSubjectKey key) const;
        double maneuver_plan_horizon_s() const;
        uint64_t current_maneuver_plan_signature() const;
        double prediction_authored_plan_request_window_s(double now_s) const;
        PredictionTimeContext build_prediction_time_context(
                PredictionSubjectKey key,
                double sim_now_s,
                double trajectory_t0_s = std::numeric_limits<double>::quiet_NaN(),
                double trajectory_t1_s = std::numeric_limits<double>::quiet_NaN()) const;
        PredictionWindowPolicyResult resolve_prediction_window_policy(
                const PredictionTrackState *track,
                const PredictionTimeContext &time_ctx,
                bool with_maneuvers) const;
        double prediction_display_window_s(PredictionSubjectKey key,
                                           double now_s,
                                           bool with_maneuvers) const;
        void refresh_prediction_preview_anchor(PredictionTrackState &track, double now_s, bool with_maneuvers) const;
        double prediction_preview_exact_window_s(const PredictionTrackState &track,
                                                 double now_s,
                                                 bool with_maneuvers) const;
        double prediction_planned_exact_window_s(const PredictionTrackState &track,
                                                 double now_s,
                                                 bool with_maneuvers) const;
        double prediction_required_window_s(const PredictionTrackState &track,
                                            double now_s,
                                            bool with_maneuvers) const;
        double prediction_required_window_s(PredictionSubjectKey key,
                                            double now_s,
                                            bool with_maneuvers) const;
        PredictionTrackState *find_prediction_track(PredictionSubjectKey key);
        const PredictionTrackState *find_prediction_track(PredictionSubjectKey key) const;
        PredictionTrackState *active_prediction_track();
        const PredictionTrackState *active_prediction_track() const;
        PredictionTrackState *player_prediction_track();
        const PredictionTrackState *player_prediction_track() const;
        OrbitPredictionCache *effective_prediction_cache(PredictionTrackState *track);
        const OrbitPredictionCache *effective_prediction_cache(const PredictionTrackState *track) const;
        OrbitPredictionCache *player_prediction_cache();
        const OrbitPredictionCache *player_prediction_cache() const;
        bool prediction_subject_is_player(PredictionSubjectKey key) const;
        bool prediction_subject_supports_maneuvers(PredictionSubjectKey key) const;
        std::string prediction_subject_label(PredictionSubjectKey key) const;
        glm::vec3 prediction_subject_orbit_rgb(PredictionSubjectKey key) const;
        const CelestialBodyInfo *find_celestial_body_info(orbitsim::BodyId body_id) const;
        const orbitsim::MassiveBody *find_massive_body(const std::vector<orbitsim::MassiveBody> &bodies,
                                                       orbitsim::BodyId body_id) const;
        WorldVec3 prediction_body_world_position(orbitsim::BodyId body_id,
                                                 const OrbitPredictionCache *cache = nullptr,
                                                 double query_time_s = std::numeric_limits<double>::quiet_NaN()) const;
        bool sample_prediction_inertial_state(const std::vector<orbitsim::TrajectorySample> &trajectory,
                                              double query_time_s,
                                              orbitsim::State &out_state) const;
        orbitsim::SpacecraftStateLookup build_prediction_player_lookup() const;
        PredictionFrameResolverContext build_prediction_frame_resolver_context() const;
        PredictionFrameControllerContext build_prediction_frame_controller_context() const;
        orbitsim::TrajectoryFrameSpec resolve_prediction_display_frame_spec(
                const OrbitPredictionCache &cache,
                double display_time_s = std::numeric_limits<double>::quiet_NaN()) const;
        bool build_prediction_display_frame(const OrbitPredictionCache &cache,
                                            orbitsim::RotatingFrame &out_frame,
                                            double display_time_s = std::numeric_limits<double>::quiet_NaN()) const;
        double resolve_prediction_display_reference_time_s(
                const OrbitPredictionCache &cache,
                double display_time_s = std::numeric_limits<double>::quiet_NaN()) const;
        bool build_prediction_display_transform(const OrbitPredictionCache &cache,
                                               WorldVec3 &out_origin_world,
                                               glm::dmat3 &out_frame_to_world,
                                               double display_time_s = std::numeric_limits<double>::quiet_NaN()) const;
        WorldVec3 prediction_sample_position_world(const OrbitPredictionCache &cache,
                                                   const orbitsim::TrajectorySample &sample,
                                                   double display_time_s = std::numeric_limits<double>::quiet_NaN()) const;
        WorldVec3 prediction_sample_hermite_world(const OrbitPredictionCache &cache,
                                                  const orbitsim::TrajectorySample &a,
                                                  const orbitsim::TrajectorySample &b,
                                                  double t_s,
                                                  double display_time_s = std::numeric_limits<double>::quiet_NaN()) const;
        bool prediction_frame_is_lagrange_sensitive(const orbitsim::TrajectoryFrameSpec &spec) const;
        orbitsim::BodyId select_prediction_primary_body_id(const std::vector<orbitsim::MassiveBody> &bodies,
                                                           const OrbitPredictionCache *cache,
                                                           const orbitsim::Vec3 &query_pos_m,
                                                           double query_time_s,
                                                           orbitsim::BodyId preferred_body_id = orbitsim::kInvalidBodyId) const;
        orbitsim::BodyId resolve_prediction_analysis_body_id(const OrbitPredictionCache &cache,
                                                             PredictionSubjectKey key,
                                                             double query_time_s,
                                                             orbitsim::BodyId preferred_body_id = orbitsim::kInvalidBodyId) const;
        WorldVec3 prediction_world_reference_body_world() const;
        WorldVec3 prediction_frame_origin_world(const OrbitPredictionCache &cache,
                                                double display_time_s = std::numeric_limits<double>::quiet_NaN()) const;
        void mark_prediction_derived_request_submitted(
                PredictionTrackState &track,
                const OrbitPredictionDerivedService::Request &request);
        bool request_prediction_derived_refresh(
                PredictionTrackState &track,
                double display_time_s = std::numeric_limits<double>::quiet_NaN());
        bool prediction_track_has_current_derived_cache(
                const PredictionTrackState &track,
                double display_time_s = std::numeric_limits<double>::quiet_NaN()) const;
        void refresh_prediction_derived_cache(PredictionTrackState &track,
                                              double display_time_s = std::numeric_limits<double>::quiet_NaN());
        void refresh_all_prediction_derived_caches();

        bool build_orbit_prediction_global_draw_context(
                GameStateContext &ctx,
                PredictionDrawDetail::PredictionGlobalDrawContext &out);
        bool build_orbit_prediction_track_draw_context(
                PredictionTrackState &track,
                const PredictionDrawDetail::PredictionGlobalDrawContext &global_ctx,
                PredictionDrawDetail::PredictionTrackDrawContext &out);
        void draw_orbit_prediction_track_windows(PredictionDrawDetail::PredictionTrackDrawContext &track_ctx);
        void emit_orbit_prediction_track_picks(
                const PredictionDrawDetail::PredictionGlobalDrawContext &global_ctx,
                PredictionDrawDetail::PredictionTrackDrawContext &track_ctx);
        void emit_orbit_prediction_debug(GameStateContext &ctx);

        void mark_maneuver_plan_dirty();
        void clear_maneuver_prediction_artifacts();

    private:
        double current_sim_time_s() const { return _state.current_sim_time_s(); }
        OrbiterInfo *find_player_orbiter() { return _state._orbit.find_player_orbiter(); }
        const OrbiterInfo *find_player_orbiter() const { return _state._orbit.find_player_orbiter(); }
        OrbiterInfo *find_orbiter(EntityId entity) { return _state._orbit.find_orbiter(entity); }
        const OrbiterInfo *find_orbiter(EntityId entity) const { return _state._orbit.find_orbiter(entity); }
        EntityId player_entity() const { return _state._orbit.player_entity(); }
        orbitsim::BodyId resolve_maneuver_node_primary_body_id(const ManeuverNode &node, double query_time_s) const
        {
            return _state.resolve_maneuver_node_primary_body_id(node, query_time_s);
        }
        WorldVec3 compute_maneuver_align_delta(GameStateContext &ctx,
                                               const OrbitPredictionCache &cache,
                                               const std::vector<orbitsim::TrajectorySample> &traj_base)
        {
            return _state.compute_maneuver_align_delta(ctx, cache, traj_base);
        }

        GameplayState &_state;
        OrbitalRuntimeSystem &_orbit;
        GameWorld &_world;
        std::unique_ptr<Physics::PhysicsWorld> &_physics;
        std::unique_ptr<Physics::PhysicsContext> &_physics_context;
        ScenarioConfig &_scenario_config;
        std::vector<OrbiterInfo> &_orbiters;
        std::unique_ptr<OrbitalScenario> &_orbitsim;
        bool &_debug_draw_enabled;
        std::unique_ptr<PredictionSystem> &_prediction;
        ManeuverSystem &_maneuver;
        double &_fixed_time_s;
        TimeWarpState &_time_warp;
        OrbitalPhysicsSystem &_orbital_physics;
    };
} // namespace Game
