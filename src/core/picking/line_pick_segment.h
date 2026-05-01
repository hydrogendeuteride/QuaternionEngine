#pragma once

#include "core/world.h"

#include <limits>

namespace Picking
{
    struct LinePickSegmentData
    {
        WorldVec3 a_world{0.0, 0.0, 0.0};
        WorldVec3 b_world{0.0, 0.0, 0.0};
        double a_time_s = std::numeric_limits<double>::quiet_NaN();
        double b_time_s = std::numeric_limits<double>::quiet_NaN();
    };
} // namespace Picking
