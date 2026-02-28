#include "game/states/gameplay/time_warp_state.h"

#include <gtest/gtest.h>

namespace
{
    using TimeWarpState = Game::TimeWarpState;
    using Mode = Game::TimeWarpState::Mode;
} // namespace

TEST(TimeWarpState, ModeForLevelUsesExpectedBands)
{
    TimeWarpState state{};

    EXPECT_EQ(state.mode_for_level(-10), Mode::Realtime);
    EXPECT_EQ(state.mode_for_level(0), Mode::Realtime);

    EXPECT_EQ(state.mode_for_level(1), Mode::PhysicsWarp);
    EXPECT_EQ(state.mode_for_level(TimeWarpState::kMaxPhysicsWarpLevel), Mode::PhysicsWarp);

    EXPECT_EQ(state.mode_for_level(TimeWarpState::kMaxPhysicsWarpLevel + 1), Mode::RailsWarp);
    EXPECT_EQ(state.mode_for_level(TimeWarpState::kMaxWarpLevel), Mode::RailsWarp);
    EXPECT_EQ(state.mode_for_level(99), Mode::RailsWarp);
}

TEST(TimeWarpState, FactorClampsWarpLevelToValidRange)
{
    TimeWarpState state{};

    state.warp_level = -5;
    EXPECT_DOUBLE_EQ(state.factor(), 1.0);

    state.warp_level = 0;
    EXPECT_DOUBLE_EQ(state.factor(), 1.0);

    state.warp_level = 1;
    EXPECT_DOUBLE_EQ(state.factor(), 2.0);

    state.warp_level = TimeWarpState::kMaxWarpLevel;
    EXPECT_DOUBLE_EQ(state.factor(), 1000.0);

    state.warp_level = TimeWarpState::kMaxWarpLevel + 10;
    EXPECT_DOUBLE_EQ(state.factor(), 1000.0);
}

TEST(TimeWarpState, WarpFactorTableIsStrictlyIncreasing)
{
    TimeWarpState state{};

    double prev = 0.0;
    for (int level = 0; level <= TimeWarpState::kMaxWarpLevel; ++level)
    {
        state.warp_level = level;
        const double factor = state.factor();
        EXPECT_GT(factor, prev);
        prev = factor;
    }
}

TEST(TimeWarpState, ModeTransitionsFollowExpectedSequenceAcrossLevels)
{
    TimeWarpState state{};

    Mode prev = Mode::Realtime;
    for (int level = 0; level <= TimeWarpState::kMaxWarpLevel; ++level)
    {
        const Mode mode = state.mode_for_level(level);

        if (level == 0)
        {
            EXPECT_EQ(mode, Mode::Realtime);
        }
        else if (level <= TimeWarpState::kMaxPhysicsWarpLevel)
        {
            EXPECT_EQ(mode, Mode::PhysicsWarp);
        }
        else
        {
            EXPECT_EQ(mode, Mode::RailsWarp);
        }

        if (level > 0)
        {
            EXPECT_NE(mode, Mode::Realtime);
        }

        prev = mode;
    }

    EXPECT_EQ(prev, Mode::RailsWarp);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
