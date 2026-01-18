#include "time_manager.h"

#include <algorithm>

namespace GameRuntime
{
    TimeManager::TimeManager()
    {
        _start_time = Clock::now();
        _last_time = _start_time;
    }

    void TimeManager::begin_frame()
    {
        TimePoint now = Clock::now();
        auto elapsed = std::chrono::duration<float>(now - _last_time);
        _last_time = now;

        // Clamp delta time to avoid spiral of death
        _unscaled_delta_time = std::min(elapsed.count(), k_max_delta_time);
        _delta_time = _unscaled_delta_time * _time_scale;

        // Update total times
        _total_time += _delta_time;
        _unscaled_total_time += _unscaled_delta_time;

        // Accumulate for fixed timestep
        _fixed_accumulator += _delta_time;

        ++_frame_count;
    }

    void TimeManager::set_fixed_delta_time(float dt)
    {
        _fixed_delta_time = std::clamp(dt, 1.0f / 240.0f, 1.0f / 10.0f);
    }

    void TimeManager::set_time_scale(float scale)
    {
        _time_scale = std::max(0.0f, scale);
    }

    bool TimeManager::consume_fixed_step()
    {
        if (_fixed_accumulator >= _fixed_delta_time)
        {
            _fixed_accumulator -= _fixed_delta_time;
            return true;
        }
        return false;
    }

    float TimeManager::interpolation_alpha() const
    {
        if (_fixed_delta_time <= 0.0f)
        {
            return 1.0f;
        }
        return _fixed_accumulator / _fixed_delta_time;
    }
} // namespace GameRuntime
