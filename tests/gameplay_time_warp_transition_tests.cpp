#include "game/states/gameplay/gameplay_state.h"

#include <gtest/gtest.h>

namespace
{
    using Game::GameplayState;
    using Game::TimeWarpState;
    using Mode = Game::TimeWarpState::Mode;
} // namespace

TEST(GameplayStateTimeWarpTransition, RailsRequestWithoutOrbitSimFallsBackToPhysicsWarp)
{
    GameplayState state{};
    Game::GameStateContext ctx{};

    state._orbit.scenario_owner().reset();
    state._orbital_physics.set_rails_warp_active_for_test(false);
    state._time_warp.mode = Mode::Realtime;
    state._time_warp.warp_level = 0;

    state.set_time_warp_level(ctx, TimeWarpState::kMaxPhysicsWarpLevel + 1);

    EXPECT_EQ(state._time_warp.warp_level, TimeWarpState::kMaxPhysicsWarpLevel);
    EXPECT_EQ(state._time_warp.mode, Mode::PhysicsWarp);
    EXPECT_FALSE(state._orbital_physics.rails_warp_active());
}

TEST(GameplayStateTimeWarpTransition, LeavingRailsWarpClearsRailsHandlesAndDisablesRailsState)
{
    GameplayState state{};
    Game::GameStateContext ctx{};

    Game::OrbiterInfo orbiter{};
    orbiter.entity = Game::EntityId{1};
    orbiter.rails.sc_id = 123;
    state._orbit.orbiters().push_back(orbiter);

    state._orbit.scenario_owner().reset();
    state._orbital_physics.set_rails_warp_active_for_test(true);
    state._time_warp.mode = Mode::RailsWarp;
    state._time_warp.warp_level = TimeWarpState::kMaxWarpLevel;

    state.set_time_warp_level(ctx, 0);

    ASSERT_EQ(state._orbit.orbiters().size(), 1u);
    EXPECT_EQ(state._time_warp.mode, Mode::Realtime);
    EXPECT_EQ(state._time_warp.warp_level, 0);
    EXPECT_FALSE(state._orbital_physics.rails_warp_active());
    EXPECT_EQ(state._orbit.orbiters()[0].rails.sc_id, orbitsim::kInvalidSpacecraftId);
}

TEST(GameplayStateTimeWarpTransition, WarpLevelClampsToBounds)
{
    GameplayState state{};
    Game::GameStateContext ctx{};

    state.set_time_warp_level(ctx, -999);
    EXPECT_EQ(state._time_warp.warp_level, 0);
    EXPECT_EQ(state._time_warp.mode, Mode::Realtime);

    // Keep mode in Rails so upper-bound clamp can be asserted without fallback.
    state._orbital_physics.set_rails_warp_active_for_test(true);
    state._time_warp.mode = Mode::RailsWarp;
    state._time_warp.warp_level = TimeWarpState::kMaxWarpLevel;

    state.set_time_warp_level(ctx, 999);
    EXPECT_EQ(state._time_warp.warp_level, TimeWarpState::kMaxWarpLevel);
    EXPECT_EQ(state._time_warp.mode, Mode::RailsWarp);
    EXPECT_TRUE(state._orbital_physics.rails_warp_active());
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
