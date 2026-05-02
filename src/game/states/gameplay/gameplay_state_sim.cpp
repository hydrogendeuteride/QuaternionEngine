#include "gameplay_state.h"

#include <algorithm>

namespace Game
{
    // ---- Time warp ----

    // Changes warp level and performs rails-warp enter/exit transitions when the mode boundary is crossed.
    void GameplayState::set_time_warp_level(GameStateContext &ctx, const int level)
    {
        const int clamped = std::clamp(level, 0, TimeWarpState::kMaxWarpLevel);
        const TimeWarpState::Mode old_mode = _time_warp.mode;

        _time_warp.warp_level = clamped;
        _time_warp.mode = _time_warp.mode_for_level(clamped);

        OrbitalPhysicsSystem::Context orbital_physics = build_orbital_physics_context();

        if (old_mode == TimeWarpState::Mode::RailsWarp && _time_warp.mode != TimeWarpState::Mode::RailsWarp)
        {
            _orbital_physics.exit_rails_warp(orbital_physics, ctx);
        }

        if (_time_warp.mode == TimeWarpState::Mode::RailsWarp && old_mode != TimeWarpState::Mode::RailsWarp)
        {
            (void) _orbital_physics.enter_rails_warp(orbital_physics, ctx);
            if (!_orbital_physics.rails_warp_active())
            {
                _time_warp.warp_level = TimeWarpState::kMaxPhysicsWarpLevel;
                _time_warp.mode = _time_warp.mode_for_level(_time_warp.warp_level);
            }
        }
    }
} // namespace Game
