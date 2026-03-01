#define private public
#include "game/states/gameplay/gameplay_state.h"
#undef private

#include <gtest/gtest.h>

#include <memory>

namespace GameplayTestHooks
{
    void register_entity(Game::Entity *entity);
    void clear_entities();
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

    std::unique_ptr<Game::OrbitalScenario> make_reference_orbitsim(const double time_s)
    {
        auto scenario = std::make_unique<Game::OrbitalScenario>();

        orbitsim::GameSimulation::Config cfg{};
        cfg.enable_events = false;
        scenario->sim = orbitsim::GameSimulation(cfg);
        (void) scenario->sim.set_time_s(time_s);

        orbitsim::MassiveBody ref{};
        ref.mass_kg = 5.972e24;
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

    class GameplayPredictionManeuverTests : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            GameplayTestHooks::clear_entities();
        }

        void TearDown() override
        {
            GameplayTestHooks::clear_entities();
        }

        static void add_player_orbiter(Game::GameplayState &state, const Game::Entity &player)
        {
            Game::OrbiterInfo info{};
            info.entity = player.id();
            info.is_player = true;
            state._orbiters.push_back(info);
        }
    };
} // namespace

TEST_F(GameplayPredictionManeuverTests, UpdatePredictionKeepsCacheWhenRebuildConditionsAreNotMet)
{
    Game::GameplayState state{};
    Game::GameStateContext ctx{};

    Game::Entity player(Game::EntityId{1}, "player");
    player.set_position_world(WorldVec3(7'000'000.0, 0.0, 0.0));
    GameplayTestHooks::register_entity(&player);
    add_player_orbiter(state, player);

    state._prediction_enabled = true;
    state._prediction_dirty = false;
    state._prediction_periodic_refresh_s = 0.0;
    state._prediction_future_window_s = 100.0;
    state._fixed_time_s = 10.0;

    state._prediction_cache.clear();
    state._prediction_cache.valid = true;
    state._prediction_cache.build_time_s = 5.0;
    state._prediction_cache.trajectory_bci = {
            make_sample(0.0, 7'000'000.0),
            make_sample(10'000.0, 7'100'000.0),
    };

    state.update_prediction(ctx, 0.016f);

    EXPECT_TRUE(state._prediction_cache.valid);
    EXPECT_EQ(state._prediction_cache.trajectory_bci.size(), 2u);
    EXPECT_DOUBLE_EQ(state._prediction_cache.build_time_s, 5.0);
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
    state._prediction_dirty = true;
    state._prediction_cache.valid = true;
    state._prediction_cache.trajectory_bci = {
            make_sample(0.0, 7'000'000.0),
            make_sample(1000.0, 7'050'000.0),
    };

    // _orbitsim == nullptr => rebuild attempt cannot produce a valid trajectory.
    state.update_prediction(ctx, 0.02f);

    EXPECT_FALSE(state._prediction_cache.valid);
    EXPECT_TRUE(state._prediction_cache.trajectory_bci.empty());
    EXPECT_TRUE(state._prediction_dirty);
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
    state._prediction_dirty = false;

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

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
