#include <sstream>

#define private public
#include "game/states/gameplay/gameplay_state.h"
#undef private

#include "core/input/input_system.h"
#include "game/component/ship_controller.h"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <memory>

namespace GameplayTestHooks
{
    void register_entity(Game::Entity *entity);
    void clear_entities();
    void set_ship_controller_input_override(const Game::ThrustInput &input);
    void clear_ship_controller_input_override();
}

namespace
{
    orbitsim::TrajectorySample make_sample(const double t_s, const double x_m)
    {
        orbitsim::TrajectorySample s{};
        s.t_s = t_s;
        s.position_m = orbitsim::Vec3{x_m, 0.0, 0.0};
        s.velocity_mps = orbitsim::Vec3{0.0, 7500.0, 0.0};
        return s;
    }

    std::unique_ptr<Game::OrbitalScenario> make_reference_orbitsim(const double time_s,
                                                                   const double reference_mass_kg = 5.972e24)
    {
        auto scenario = std::make_unique<Game::OrbitalScenario>();

        orbitsim::GameSimulation::Config cfg{};
        cfg.enable_events = false;
        scenario->sim = orbitsim::GameSimulation(cfg);
        (void) scenario->sim.set_time_s(time_s);

        orbitsim::MassiveBody ref{};
        ref.mass_kg = reference_mass_kg;
        ref.radius_m = 6'371'000.0;
        ref.state = orbitsim::make_state(glm::dvec3(0.0), glm::dvec3(0.0));

        const auto handle = scenario->sim.create_body(ref);
        if (!handle.valid())
        {
            return nullptr;
        }

        Game::CelestialBodyInfo ref_info{};
        ref_info.sim_id = handle.id;
        ref_info.name = "earth";
        ref_info.radius_m = ref.radius_m;
        ref_info.mass_kg = ref.mass_kg;

        scenario->bodies.push_back(ref_info);
        scenario->reference_body_index = 0;
        return scenario;
    }

    std::unique_ptr<Game::OrbitalScenario> make_two_body_orbitsim(const double time_s)
    {
        auto scenario = make_reference_orbitsim(time_s);
        if (!scenario)
        {
            return nullptr;
        }

        orbitsim::MassiveBody moon{};
        moon.mass_kg = 7.342e22;
        moon.radius_m = 1'737'400.0;
        moon.state = orbitsim::make_state(glm::dvec3(384'400'000.0, 0.0, 0.0), glm::dvec3(0.0, 1022.0, 0.0));

        const auto moon_handle = scenario->sim.create_body(moon);
        if (!moon_handle.valid())
        {
            return nullptr;
        }

        Game::CelestialBodyInfo moon_info{};
        moon_info.sim_id = moon_handle.id;
        moon_info.name = "moon";
        moon_info.radius_m = moon.radius_m;
        moon_info.mass_kg = moon.mass_kg;
        scenario->bodies.push_back(moon_info);
        return scenario;
    }

    Game::Entity make_entity(const uint32_t id, const char *name, const WorldVec3 &position_world)
    {
        Game::Entity entity(Game::EntityId{id}, name);
        entity.set_position_world(position_world);
        return entity;
    }

    Game::OrbitPredictionService::Request make_prediction_request(const double time_s)
    {
        Game::OrbitPredictionService::Request request{};
        request.sim_time_s = time_s;
        request.sim_config = orbitsim::GameSimulation::Config{};
        request.sim_config.enable_events = false;
        request.reference_body_mass_kg = 5.972e24;
        request.reference_body_radius_m = 6'371'000.0;
        request.ship_bary_position_m = orbitsim::Vec3{7'000'000.0, 0.0, 0.0};
        request.ship_bary_velocity_mps = orbitsim::Vec3{0.0, 7500.0, 0.0};
        request.future_window_s = 120.0;
        request.max_maneuver_time_s = time_s;

        orbitsim::MassiveBody ref{};
        ref.mass_kg = request.reference_body_mass_kg;
        ref.radius_m = request.reference_body_radius_m;
        ref.id = 1;
        ref.state = orbitsim::make_state(glm::dvec3(0.0), glm::dvec3(0.0));
        request.reference_body_id = ref.id;
        request.massive_bodies.push_back(ref);
        return request;
    }

    class GameplayPredictionManeuverTests : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            GameplayTestHooks::clear_entities();
            GameplayTestHooks::clear_ship_controller_input_override();
        }

        void TearDown() override
        {
            GameplayTestHooks::clear_entities();
            GameplayTestHooks::clear_ship_controller_input_override();
        }

        static void add_player_orbiter(Game::GameplayState &state, const Game::Entity &player)
        {
            Game::OrbiterInfo info{};
            info.entity = player.id();
            info.name = player.name();
            info.is_player = true;
            state._orbiters.push_back(info);
        }

        static void add_orbiter(Game::GameplayState &state,
                                const Game::Entity &entity,
                                const bool is_player = false,
                                const bool is_rebase_anchor = false,
                                const char *name = nullptr)
        {
            Game::OrbiterInfo info{};
            info.entity = entity.id();
            info.name = name ? name : entity.name();
            info.is_player = is_player;
            info.is_rebase_anchor = is_rebase_anchor;
            info.mass_kg = 1'000.0;
            state._orbiters.push_back(info);
        }
    };
} // namespace

TEST_F(GameplayPredictionManeuverTests, UpdatePredictionDisabledClearsCachesAndPendingFlags)
{
    Game::GameplayState state{};
    Game::GameStateContext ctx{};

    Game::Entity player = make_entity(10, "player", WorldVec3(7'000'000.0, 0.0, 0.0));
    GameplayTestHooks::register_entity(&player);
    add_player_orbiter(state, player);

    state.rebuild_prediction_subjects();
    auto *track = state.player_prediction_track();
    ASSERT_NE(track, nullptr);

    track->cache.valid = true;
    track->cache.trajectory_bci = {
            make_sample(0.0, 7'000'000.0),
            make_sample(120.0, 7'100'000.0),
    };
    track->request_pending = true;
    track->dirty = true;
    state._prediction_enabled = false;
    state._prediction_dirty = true;

    state.update_prediction(ctx, 0.016f);

    EXPECT_FALSE(track->cache.valid);
    EXPECT_TRUE(track->cache.trajectory_bci.empty());
    EXPECT_FALSE(track->request_pending);
    EXPECT_FALSE(track->dirty);
    EXPECT_FALSE(state._prediction_dirty);
}

TEST_F(GameplayPredictionManeuverTests, UpdatePredictionKeepsCacheWhenRebuildConditionsAreNotMet)
{
    Game::GameplayState state{};
    Game::GameStateContext ctx{};
    state._orbitsim = make_reference_orbitsim(10.0);
    ASSERT_NE(state._orbitsim, nullptr);

    Game::Entity player(Game::EntityId{1}, "player");
    player.set_position_world(WorldVec3(7'000'000.0, 0.0, 0.0));
    GameplayTestHooks::register_entity(&player);
    add_player_orbiter(state, player);

    state._prediction_enabled = true;
    state._prediction_periodic_refresh_s = 0.0;
    state._prediction_future_window_orbiter_s = 100.0;
    state._fixed_time_s = 10.0;

    state.rebuild_prediction_subjects();
    auto *track = state.player_prediction_track();
    ASSERT_NE(track, nullptr);

    track->cache.clear();
    track->cache.valid = true;
    track->cache.build_time_s = 5.0;
    track->cache.trajectory_bci = {
            make_sample(0.0, 7'000'000.0),
            make_sample(10'000.0, 7'100'000.0),
    };
    track->dirty = false;
    track->request_pending = false;
    state.sync_prediction_dirty_flag();

    state.update_prediction(ctx, 0.016f);

    track = state.player_prediction_track();
    ASSERT_NE(track, nullptr);
    EXPECT_TRUE(track->cache.valid);
    EXPECT_EQ(track->cache.trajectory_bci.size(), 2u);
    EXPECT_DOUBLE_EQ(track->cache.build_time_s, 5.0);
    EXPECT_FALSE(state._prediction_dirty);
}

TEST_F(GameplayPredictionManeuverTests, UpdatePredictionRebuildsWhenDirtyAndClearsInvalidCache)
{
    Game::GameplayState state{};
    Game::GameStateContext ctx{};

    Game::Entity player(Game::EntityId{2}, "player");
    player.set_position_world(WorldVec3(7'000'000.0, 0.0, 0.0));
    GameplayTestHooks::register_entity(&player);
    add_player_orbiter(state, player);

    state._prediction_enabled = true;
    state.rebuild_prediction_subjects();
    auto *track = state.player_prediction_track();
    ASSERT_NE(track, nullptr);

    track->cache.valid = true;
    track->cache.trajectory_bci = {
            make_sample(0.0, 7'000'000.0),
            make_sample(1000.0, 7'050'000.0),
    };
    track->dirty = true;
    state.sync_prediction_dirty_flag();

    // _orbitsim == nullptr => rebuild attempt cannot produce a valid trajectory.
    state.update_prediction(ctx, 0.02f);

    track = state.player_prediction_track();
    ASSERT_NE(track, nullptr);
    EXPECT_FALSE(track->cache.valid);
    EXPECT_TRUE(track->cache.trajectory_bci.empty());
    EXPECT_TRUE(state._prediction_dirty);
}

TEST_F(GameplayPredictionManeuverTests, UpdatePredictionRequestsAsyncWhenCacheCoverageRunsOut)
{
    Game::GameplayState state{};
    Game::GameStateContext ctx{};

    state._orbitsim = make_reference_orbitsim(10.0);
    ASSERT_NE(state._orbitsim, nullptr);

    Game::Entity player = make_entity(11, "player", WorldVec3(7'000'000.0, 0.0, 0.0));
    GameplayTestHooks::register_entity(&player);
    add_player_orbiter(state, player);

    state._prediction_future_window_orbiter_s = 120.0;
    state.rebuild_prediction_subjects();
    auto *track = state.player_prediction_track();
    ASSERT_NE(track, nullptr);

    track->cache.valid = true;
    track->cache.build_time_s = 9.0;
    track->cache.trajectory_bci = {
            make_sample(10.0, 7'000'000.0),
            make_sample(25.0, 7'000'100.0),
    };
    track->dirty = false;
    track->request_pending = false;
    state.sync_prediction_dirty_flag();

    state.update_prediction(ctx, 0.016f);
    track = state.player_prediction_track();
    ASSERT_NE(track, nullptr);

    EXPECT_TRUE(track->request_pending);
    EXPECT_FALSE(track->dirty);
    EXPECT_TRUE(state._prediction_dirty);
}

TEST_F(GameplayPredictionManeuverTests, UpdatePredictionSuppressesDragRebuildForFreshManeuverCache)
{
    Game::GameplayState state{};
    Game::GameStateContext ctx{};

    state._orbitsim = make_reference_orbitsim(10.0);
    ASSERT_NE(state._orbitsim, nullptr);

    Game::Entity player = make_entity(12, "player", WorldVec3(7'000'000.0, 0.0, 0.0));
    GameplayTestHooks::register_entity(&player);
    add_player_orbiter(state, player);

    Game::GameplayState::ManeuverNode node{};
    node.id = 99;
    node.time_s = 20.0;
    node.dv_rtn_mps = glm::dvec3(0.0, 5.0, 0.0);
    node.primary_body_id = state._orbitsim->reference_body()->sim_id;
    state._maneuver_state.nodes.push_back(node);
    state._maneuver_state.selected_node_id = node.id;

    state.rebuild_prediction_subjects();
    auto *track = state.player_prediction_track();
    ASSERT_NE(track, nullptr);

    track->cache.valid = true;
    track->cache.build_time_s = 9.99;
    track->cache.trajectory_bci = {
            make_sample(10.0, 7'000'000.0),
            make_sample(1000.0, 7'100'000.0),
    };
    track->dirty = true;
    track->request_pending = false;
    state._maneuver_gizmo_interaction.state = Game::GameplayState::ManeuverGizmoInteraction::State::DragAxis;
    state._maneuver_gizmo_interaction.node_id = node.id;
    state._maneuver_gizmo_interaction.axis = Game::GameplayState::ManeuverHandleAxis::TangentialPos;

    state.update_prediction(ctx, 0.016f);
    track = state.player_prediction_track();
    ASSERT_NE(track, nullptr);

    EXPECT_FALSE(track->request_pending);
    EXPECT_TRUE(track->dirty);
    EXPECT_EQ(track->cache.trajectory_bci.size(), 2u);
}

TEST_F(GameplayPredictionManeuverTests, UpdatePredictionClearsVisibleRuntimeWhenReferenceBodyIsInvalid)
{
    Game::GameplayState state{};
    Game::GameStateContext ctx{};

    state._orbitsim = make_reference_orbitsim(10.0, 0.0);
    ASSERT_NE(state._orbitsim, nullptr);

    Game::Entity player = make_entity(13, "player", WorldVec3(7'000'000.0, 0.0, 0.0));
    GameplayTestHooks::register_entity(&player);
    add_player_orbiter(state, player);

    state.rebuild_prediction_subjects();
    auto *track = state.player_prediction_track();
    ASSERT_NE(track, nullptr);

    track->cache.valid = true;
    track->cache.trajectory_bci = {
            make_sample(0.0, 7'000'000.0),
            make_sample(100.0, 7'010'000.0),
    };
    track->dirty = false;
    track->request_pending = true;

    state.update_prediction(ctx, 0.016f);
    track = state.player_prediction_track();
    ASSERT_NE(track, nullptr);

    EXPECT_FALSE(track->cache.valid);
    EXPECT_TRUE(track->cache.trajectory_bci.empty());
    EXPECT_TRUE(track->dirty);
    EXPECT_FALSE(track->request_pending);
}

TEST_F(GameplayPredictionManeuverTests, RebuildPredictionSubjectsCreatesDefaultSelectionAndPredictionGroup)
{
    Game::GameplayState state{};
    state._orbitsim = make_two_body_orbitsim(10.0);
    ASSERT_NE(state._orbitsim, nullptr);

    Game::Entity player = make_entity(20, "player", WorldVec3(7'000'000.0, 0.0, 0.0));
    Game::Entity escort = make_entity(21, "escort", WorldVec3(7'050'000.0, 0.0, 0.0));
    GameplayTestHooks::register_entity(&player);
    GameplayTestHooks::register_entity(&escort);

    add_orbiter(state, player, true, false, "player");
    add_orbiter(state, escort, false, false, "escort");

    Game::ScenarioConfig::OrbiterDef player_def{};
    player_def.name = "player";
    player_def.prediction_group = "squad";
    player_def.is_player = true;
    Game::ScenarioConfig::OrbiterDef escort_def{};
    escort_def.name = "escort";
    escort_def.prediction_group = "squad";
    state._scenario_config.orbiters = {player_def, escort_def};

    state.rebuild_prediction_subjects();

    const Game::PredictionSubjectKey player_key{Game::PredictionSubjectKind::Orbiter, player.id().value};

    ASSERT_EQ(state._prediction_tracks.size(), 3u);
    ASSERT_EQ(state._prediction_groups.size(), 1u);
    EXPECT_EQ(state._prediction_groups.front().name, "squad");
    EXPECT_EQ(state._prediction_groups.front().members.size(), 2u);
    EXPECT_EQ(state._prediction_groups.front().primary_subject, player_key);
    EXPECT_EQ(state._prediction_selection.active_subject, player_key);
    EXPECT_EQ(state._prediction_selection.overlay_subjects.size(), 2u);

    const std::vector<Game::PredictionSubjectKey> visible = state.collect_visible_prediction_subjects();
    EXPECT_EQ(visible.size(), 3u);
}

TEST_F(GameplayPredictionManeuverTests, PredictionHelpersUseRenderOverridesAndClampVisibilityMetadata)
{
    Game::GameplayState state{};
    state._scenario_config.system_center = WorldVec3(100.0, 200.0, 300.0);
    state._orbitsim = make_two_body_orbitsim(15.0);
    ASSERT_NE(state._orbitsim, nullptr);

    Game::Entity player = make_entity(22, "player", WorldVec3(7'000'000.0, 0.0, 0.0));
    Game::Entity moon_render = make_entity(23, "moon_render", WorldVec3(9.0, 8.0, 7.0));
    GameplayTestHooks::register_entity(&player);
    GameplayTestHooks::register_entity(&moon_render);
    add_orbiter(state, player, true, false, "player");

    state._orbitsim->bodies[1].render_entity = moon_render.id();
    state._prediction_future_window_orbiter_s = -5.0;
    state._prediction_future_window_celestial_s = -10.0;
    state.rebuild_prediction_subjects();

    const Game::PredictionSubjectKey player_key{Game::PredictionSubjectKind::Orbiter, player.id().value};
    const Game::PredictionSubjectKey moon_key{Game::PredictionSubjectKind::Celestial,
                                              static_cast<uint32_t>(state._orbitsim->bodies[1].sim_id)};

    WorldVec3 pos_world{0.0};
    glm::dvec3 vel_world{0.0};
    glm::vec3 vel_local{0.0f};
    ASSERT_TRUE(state.get_prediction_subject_world_state(moon_key, pos_world, vel_world, vel_local));
    EXPECT_DOUBLE_EQ(pos_world.x, moon_render.position_world().x);
    EXPECT_DOUBLE_EQ(pos_world.y, moon_render.position_world().y);
    EXPECT_DOUBLE_EQ(pos_world.z, moon_render.position_world().z);
    EXPECT_NEAR(vel_world.y, 1022.0, 1.0e-6);

    state._prediction_selection.active_subject = moon_key;
    state._prediction_selection.overlay_subjects = {player_key, moon_key, player_key, {}};
    const std::vector<Game::PredictionSubjectKey> visible = state.collect_visible_prediction_subjects();
    ASSERT_EQ(visible.size(), 2u);
    EXPECT_EQ(visible.front(), moon_key);
    EXPECT_EQ(visible.back(), player_key);

    EXPECT_EQ(state.prediction_future_window_s(player_key), 0.0);
    EXPECT_EQ(state.prediction_future_window_s(moon_key), 0.0);
    EXPECT_EQ(state.prediction_subject_label(player_key), "player (player)");
    EXPECT_EQ(state.prediction_subject_label(moon_key), "moon (celestial)");
}

TEST_F(GameplayPredictionManeuverTests, ManeuverWarpStopsAtTargetAndRestoresWarpLevel)
{
    Game::GameplayState state{};
    Game::GameStateContext ctx{};

    state._orbitsim = make_reference_orbitsim(120.0);
    ASSERT_NE(state._orbitsim, nullptr);

    state._warp_to_time_active = true;
    state._warp_to_time_target_s = 120.0;
    state._warp_to_time_restore_level = 2;

    state._time_warp.warp_level = 5;
    state._time_warp.mode = Game::TimeWarpState::Mode::RailsWarp;
    state._rails_warp_active = false;

    state.update_maneuver_nodes_time_warp(ctx, 0.02f);

    EXPECT_FALSE(state._warp_to_time_active);
    EXPECT_EQ(state._time_warp.warp_level, 2);
    EXPECT_EQ(state._time_warp.mode, Game::TimeWarpState::Mode::PhysicsWarp);
}

TEST_F(GameplayPredictionManeuverTests, ManeuverWarpSelectsFastestSafeRailsLevel)
{
    Game::GameplayState state{};
    Game::GameStateContext ctx{};

    state._orbitsim = make_reference_orbitsim(100.0);
    ASSERT_NE(state._orbitsim, nullptr);

    Game::Entity player = make_entity(24, "player", WorldVec3(7'000'000.0, 0.0, 0.0));
    GameplayTestHooks::register_entity(&player);
    add_player_orbiter(state, player);

    state._warp_to_time_active = true;
    state._warp_to_time_target_s = 190.0;
    state._warp_to_time_restore_level = 1;
    state._time_warp.warp_level = 0;
    state._time_warp.mode = Game::TimeWarpState::Mode::Realtime;

    state.update_maneuver_nodes_time_warp(ctx, 1.0f);

    EXPECT_TRUE(state._warp_to_time_active);
    EXPECT_EQ(state._time_warp.warp_level, 4);
    EXPECT_EQ(state._time_warp.mode, Game::TimeWarpState::Mode::RailsWarp);
}

TEST_F(GameplayPredictionManeuverTests, EnterRailsWarpRollsBackWhenNoPlayerSpacecraftCanBeCreated)
{
    Game::GameplayState state{};
    Game::GameStateContext ctx{};

    state._orbitsim = make_reference_orbitsim(10.0);
    ASSERT_NE(state._orbitsim, nullptr);

    Game::Entity drone = make_entity(30, "drone", WorldVec3(7'000'000.0, 0.0, 0.0));
    GameplayTestHooks::register_entity(&drone);
    add_orbiter(state, drone, false, false, "drone");

    state.enter_rails_warp(ctx);

    EXPECT_FALSE(state._rails_warp_active);
    ASSERT_EQ(state._orbiters.size(), 1u);
    EXPECT_FALSE(state._orbiters.front().rails.active());
}

TEST_F(GameplayPredictionManeuverTests, RailsWarpStepAppliesThrustAndMarksPredictionDirty)
{
    Game::GameplayState state{};
    Game::GameStateContext ctx{};
    InputState input{};
    ctx.input = &input;

    state._orbitsim = make_reference_orbitsim(10.0);
    ASSERT_NE(state._orbitsim, nullptr);

    Game::Entity player = make_entity(31, "player", WorldVec3(7'000'000.0, 0.0, 0.0));
    player.add_component<Game::ShipController>(200.0f, 0.0f);
    GameplayTestHooks::register_entity(&player);
    add_orbiter(state, player, true, false, "player");

    orbitsim::Spacecraft sc{};
    sc.state = orbitsim::make_state(glm::dvec3(7'000'000.0, 0.0, 0.0), glm::dvec3(0.0, 7500.0, 0.0));
    sc.dry_mass_kg = 1000.0;
    const auto sc_handle = state._orbitsim->sim.create_spacecraft(sc);
    ASSERT_TRUE(sc_handle.valid());

    state._orbiters.front().rails.sc_id = sc_handle.id;
    state._orbiters.front().rails.rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    state._rails_warp_active = true;
    state._time_warp.mode = Game::TimeWarpState::Mode::RailsWarp;
    state.rebuild_prediction_subjects();
    ASSERT_NE(state.player_prediction_track(), nullptr);
    state.player_prediction_track()->dirty = false;
    state.player_prediction_track()->request_pending = false;
    state.sync_prediction_dirty_flag();

    Game::ThrustInput thrust_input{};
    thrust_input.local_thrust_dir = glm::vec3(0.0f, 0.0f, 1.0f);
    GameplayTestHooks::set_ship_controller_input_override(thrust_input);

    const orbitsim::Spacecraft *before = state._orbitsim->sim.spacecraft_by_id(sc_handle.id);
    ASSERT_NE(before, nullptr);
    const double vz_before = before->state.velocity_mps.z;

    state.rails_warp_step(ctx, 2.0);

    const orbitsim::Spacecraft *after = state._orbitsim->sim.spacecraft_by_id(sc_handle.id);
    ASSERT_NE(after, nullptr);
    EXPECT_NEAR(after->state.velocity_mps.z, vz_before + 0.4, 1.0e-6);
    EXPECT_TRUE(state._rails_thrust_applied_this_tick);
    EXPECT_TRUE(state.player_prediction_track()->dirty);
    EXPECT_TRUE(state._prediction_dirty);
}

TEST_F(GameplayPredictionManeuverTests, MissingArmedNodeDisarmsExecutionState)
{
    Game::GameplayState state{};
    Game::GameStateContext ctx{};

    state._execute_node_armed = true;
    state._execute_node_id = 404;

    state.update_maneuver_nodes_execution(ctx);

    EXPECT_FALSE(state._execute_node_armed);
    EXPECT_EQ(state._execute_node_id, -1);
}

TEST_F(GameplayPredictionManeuverTests, ExecutingArmedNodeAppliesImpulseAndConsumesNode)
{
    Game::GameplayState state{};
    Game::GameStateContext ctx{};

    state._scenario_config.system_center = WorldVec3(0.0, 0.0, 0.0);
    state._orbitsim = make_reference_orbitsim(10.0);
    ASSERT_NE(state._orbitsim, nullptr);

    Game::Entity player(Game::EntityId{3}, "player");
    player.set_position_world(WorldVec3(7'000'000.0, 0.0, 0.0));
    GameplayTestHooks::register_entity(&player);

    Game::OrbiterInfo orbiter{};
    orbiter.entity = player.id();
    orbiter.is_player = true;
    orbiter.mass_kg = 10'000.0;
    state._orbiters.push_back(orbiter);

    orbitsim::Spacecraft sc{};
    sc.state = orbitsim::make_state(glm::dvec3(7'000'000.0, 0.0, 0.0), glm::dvec3(0.0, 7500.0, 0.0));
    sc.dry_mass_kg = 1000.0;
    const auto sc_handle = state._orbitsim->sim.create_spacecraft(sc);
    ASSERT_TRUE(sc_handle.valid());
    state._orbiters[0].rails.sc_id = sc_handle.id;

    Game::GameplayState::ManeuverNode node{};
    node.id = 42;
    node.time_s = 10.0;
    node.dv_rtn_mps = glm::dvec3(0.0, 25.0, 0.0); // +T
    node.primary_body_id = state._orbitsim->reference_body()->sim_id;
    state._maneuver_state.nodes.push_back(node);
    state._maneuver_state.selected_node_id = node.id;

    state._execute_node_armed = true;
    state._execute_node_id = node.id;
    state.rebuild_prediction_subjects();
    if (auto *track = state.player_prediction_track())
    {
        track->dirty = false;
        track->request_pending = false;
    }
    state.sync_prediction_dirty_flag();

    state._rails_warp_active = true;
    state._time_warp.mode = Game::TimeWarpState::Mode::RailsWarp;

    const auto *before = state._orbitsim->sim.spacecraft_by_id(sc_handle.id);
    ASSERT_NE(before, nullptr);
    const double vy_before = before->state.velocity_mps.y;

    state.update_maneuver_nodes_execution(ctx);

    const auto *after = state._orbitsim->sim.spacecraft_by_id(sc_handle.id);
    ASSERT_NE(after, nullptr);
    EXPECT_NEAR(after->state.velocity_mps.y, vy_before + 25.0, 1e-6);

    EXPECT_TRUE(state._maneuver_state.nodes.empty());
    EXPECT_EQ(state._maneuver_state.selected_node_id, -1);
    EXPECT_FALSE(state._execute_node_armed);
    EXPECT_EQ(state._execute_node_id, -1);
    EXPECT_TRUE(state._prediction_dirty);
}

TEST_F(GameplayPredictionManeuverTests, PredictionServiceBuildsSecondNodePreviewFromPrefixPlan)
{
    Game::OrbitPredictionService::Request request = make_prediction_request(0.0);

    Game::OrbitPredictionService::ManeuverImpulse first{};
    first.node_id = 1;
    first.t_s = 10.0;
    first.primary_body_id = request.reference_body_id;
    first.dv_rtn_mps = glm::dvec3(0.0, 0.0, 50.0);
    request.maneuver_impulses.push_back(first);

    Game::OrbitPredictionService::ManeuverImpulse second{};
    second.node_id = 2;
    second.t_s = 20.0;
    second.primary_body_id = request.reference_body_id;
    second.dv_rtn_mps = glm::dvec3(0.0, 0.0, 0.0);
    request.maneuver_impulses.push_back(second);
    request.max_maneuver_time_s = second.t_s;

    const Game::OrbitPredictionService::Result result =
            Game::OrbitPredictionService::compute_prediction(1, request);

    ASSERT_TRUE(result.valid);
    ASSERT_GE(result.maneuver_previews.size(), 2u);

    const auto find_preview = [&](const int node_id) -> const Game::OrbitPredictionService::ManeuverNodePreview * {
        for (const auto &preview : result.maneuver_previews)
        {
            if (preview.node_id == node_id)
            {
                return &preview;
            }
        }
        return nullptr;
    };

    const auto *first_preview = find_preview(1);
    const auto *second_preview = find_preview(2);
    ASSERT_NE(first_preview, nullptr);
    ASSERT_NE(second_preview, nullptr);
    EXPECT_TRUE(first_preview->valid);
    EXPECT_TRUE(second_preview->valid);
    EXPECT_NEAR(first_preview->rel_velocity_mps.z, 0.0, 1.0e-3);
    EXPECT_GT(std::abs(second_preview->rel_velocity_mps.z), 10.0);
}

TEST_F(GameplayPredictionManeuverTests, PredictionServiceRejectsResultsFromPreviousResetEpoch)
{
    Game::OrbitPredictionService::PendingJob stale_job{};
    stale_job.track_id = 77;
    stale_job.request_epoch = 3;
    stale_job.generation_id = 12;

    const std::unordered_map<uint64_t, uint64_t> latest_requested_generation_by_track{};
    EXPECT_FALSE(Game::OrbitPredictionService::should_publish_result(stale_job, 4, latest_requested_generation_by_track));

    stale_job.request_epoch = 4;
    EXPECT_TRUE(Game::OrbitPredictionService::should_publish_result(stale_job, 4, latest_requested_generation_by_track));

    const std::unordered_map<uint64_t, uint64_t> superseded_generation{{stale_job.track_id, stale_job.generation_id + 1}};
    EXPECT_FALSE(Game::OrbitPredictionService::should_publish_result(stale_job, 4, superseded_generation));
}

TEST_F(GameplayPredictionManeuverTests, RuntimeCacheUsesNodePreviewBeforeBaselineTrajectory)
{
    Game::GameplayState state{};
    Game::GameStateContext ctx{};

    state._scenario_config.system_center = WorldVec3(0.0, 0.0, 0.0);
    state._orbitsim = make_reference_orbitsim(10.0);
    ASSERT_NE(state._orbitsim, nullptr);

    Game::Entity player(Game::EntityId{4}, "player");
    player.set_position_world(WorldVec3(7'000'000.0, 0.0, 0.0));
    GameplayTestHooks::register_entity(&player);
    add_player_orbiter(state, player);

    Game::GameplayState::ManeuverNode node{};
    node.id = 7;
    node.time_s = 20.0;
    node.dv_rtn_mps = glm::dvec3(0.0, 10.0, 0.0);
    node.primary_body_id = state._orbitsim->reference_body()->sim_id;
    state._maneuver_state.nodes.push_back(node);
    state._maneuver_state.selected_node_id = node.id;

    state.rebuild_prediction_subjects();
    auto *track = state.player_prediction_track();
    ASSERT_NE(track, nullptr);

    track->cache.clear();
    track->cache.valid = true;
    track->cache.trajectory_bci = {
            make_sample(10.0, 7'000'000.0),
            make_sample(20.0, 7'000'100.0),
            make_sample(30.0, 7'000'200.0),
    };

    Game::GameplayState::OrbitPredictionCache::ManeuverNodePreview preview{};
    preview.node_id = node.id;
    preview.t_s = node.time_s;
    preview.valid = true;
    preview.rel_position_m = glm::dvec3(7'000'050.0, 0.0, 1500.0);
    preview.rel_velocity_mps = glm::dvec3(0.0, 7490.0, 45.0);
    track->cache.maneuver_previews.push_back(preview);

    state.refresh_maneuver_node_runtime_cache(ctx);

    ASSERT_EQ(state._maneuver_state.nodes.size(), 1u);
    const Game::GameplayState::ManeuverNode &runtime_node = state._maneuver_state.nodes.front();
    EXPECT_TRUE(runtime_node.gizmo_valid);
    EXPECT_NEAR(runtime_node.position_world.z, 1500.0, 1.0e-6);
    EXPECT_GT(std::abs(runtime_node.basis_n_world.x) + std::abs(runtime_node.basis_n_world.y), 1.0e-6);
}

TEST_F(GameplayPredictionManeuverTests, RebaseAnchorSelectionAndUpdatePreferExplicitAnchorThenClear)
{
    Game::GameplayState state{};

    Game::Entity player = make_entity(40, "player", WorldVec3(0.0, 0.0, 0.0));
    Game::Entity anchor = make_entity(41, "anchor", WorldVec3(1.0, 0.0, 0.0));
    GameplayTestHooks::register_entity(&player);
    GameplayTestHooks::register_entity(&anchor);

    add_orbiter(state, player, true, false, "player");
    add_orbiter(state, anchor, false, true, "anchor");

    EXPECT_EQ(state.select_rebase_anchor_entity(), anchor.id());

    state.update_rebase_anchor();
    EXPECT_EQ(state._world.rebase_anchor(), anchor.id());

    state._orbiters.clear();
    state.update_rebase_anchor();
    EXPECT_FALSE(state._world.rebase_anchor().is_valid());
}

TEST_F(GameplayPredictionManeuverTests, ManeuverAxisHelpersMapComponentsLabelsColorsAndHoverPriority)
{
    Game::GameplayState state{};
    Game::GameplayState::ManeuverNode node{};
    node.basis_r_world = glm::dvec3(1.0, 0.0, 0.0);
    node.basis_t_world = glm::dvec3(0.0, 1.0, 0.0);
    node.basis_n_world = glm::dvec3(0.0, 0.0, 1.0);

    glm::dvec3 axis_dir_world{0.0};
    int component = -1;
    double sign = 0.0;

    ASSERT_TRUE(state.resolve_maneuver_axis(node,
                                            Game::GameplayState::ManeuverHandleAxis::RadialNeg,
                                            axis_dir_world,
                                            component,
                                            sign));
    EXPECT_DOUBLE_EQ(axis_dir_world.x, -1.0);
    EXPECT_DOUBLE_EQ(axis_dir_world.y, 0.0);
    EXPECT_DOUBLE_EQ(axis_dir_world.z, 0.0);
    EXPECT_EQ(component, 0);
    EXPECT_DOUBLE_EQ(sign, -1.0);
    EXPECT_FALSE(state.resolve_maneuver_axis(node,
                                             static_cast<Game::GameplayState::ManeuverHandleAxis>(999),
                                             axis_dir_world,
                                             component,
                                             sign));

    EXPECT_STREQ(state.maneuver_axis_label(Game::GameplayState::ManeuverHandleAxis::TangentialPos), "+T");
    EXPECT_STREQ(state.maneuver_axis_label(Game::GameplayState::ManeuverHandleAxis::NormalNeg), "-N");
    EXPECT_EQ(state.maneuver_axis_color(Game::GameplayState::ManeuverHandleAxis::TangentialPos),
              state.maneuver_axis_color(Game::GameplayState::ManeuverHandleAxis::TangentialNeg));
    EXPECT_NE(state.maneuver_axis_color(Game::GameplayState::ManeuverHandleAxis::TangentialPos),
              state.maneuver_axis_color(Game::GameplayState::ManeuverHandleAxis::RadialPos));

    std::vector<Game::GameplayState::ManeuverHubMarker> hubs{
            Game::GameplayState::ManeuverHubMarker{1, glm::vec2(20.0f, 20.0f), 5.0},
            Game::GameplayState::ManeuverHubMarker{2, glm::vec2(100.0f, 100.0f), 10.0},
    };
    std::vector<Game::GameplayState::ManeuverAxisMarker> handles{
            Game::GameplayState::ManeuverAxisMarker{1,
                                                    Game::GameplayState::ManeuverHandleAxis::TangentialPos,
                                                    glm::vec2(20.0f, 20.0f),
                                                    glm::vec2(22.0f, 22.0f),
                                                    0u,
                                                    "+T",
                                                    5.0},
    };

    int hovered_hub_idx = -1;
    int hovered_handle_idx = -1;
    state.find_maneuver_gizmo_hover(hubs,
                                    handles,
                                    glm::vec2(21.0f, 21.0f),
                                    400.0f,
                                    400.0f,
                                    hovered_hub_idx,
                                    hovered_handle_idx);
    EXPECT_EQ(hovered_handle_idx, 0);
    EXPECT_EQ(hovered_hub_idx, -1);

    hovered_hub_idx = -1;
    hovered_handle_idx = -1;
    state.find_maneuver_gizmo_hover(hubs,
                                    {},
                                    glm::vec2(18.0f, 18.0f),
                                    400.0f,
                                    25.0f,
                                    hovered_hub_idx,
                                    hovered_handle_idx);
    EXPECT_EQ(hovered_handle_idx, -1);
    EXPECT_EQ(hovered_hub_idx, 0);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
