#pragma once

#include <algorithm>
#include <array>

namespace Game
{
    struct TimeWarpState
    {
        enum class Mode
        {
            Realtime,
            PhysicsWarp,
            RailsWarp
        };

        Mode mode{Mode::Realtime};
        int warp_level{0}; // 0=x1, 1=x2, 2=x5, 3=x10, 4=x50, 5=x100, 6=x1000

        static constexpr int kMaxPhysicsWarpLevel = 3; // x10
        static constexpr int kMaxWarpLevel = 6;        // x1000

        static constexpr std::array<double, 7> kWarpFactors{1.0, 2.0, 5.0, 10.0, 50.0, 100.0, 1000.0};

        double factor() const
        {
            const int idx = std::clamp(warp_level, 0, kMaxWarpLevel);
            return kWarpFactors[static_cast<size_t>(idx)];
        }

        Mode mode_for_level(int level) const
        {
            if (level <= 0)
            {
                return Mode::Realtime;
            }
            if (level <= kMaxPhysicsWarpLevel)
            {
                return Mode::PhysicsWarp;
            }
            return Mode::RailsWarp;
        }
    };
} // namespace Game
