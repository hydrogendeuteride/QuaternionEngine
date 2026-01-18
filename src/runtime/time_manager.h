#pragma once

// TimeManager: Manages game time, time scale, and fixed timestep accumulation.

#include <chrono>

namespace GameRuntime
{

class TimeManager
{
public:
    TimeManager();

    // Begin a new frame. Call at the start of each frame.
    void begin_frame();

    // Get delta time in seconds since last frame (scaled by time_scale, clamped).
    float delta_time() const { return _delta_time; }

    // Get unscaled delta time in seconds since last frame (clamped).
    float unscaled_delta_time() const { return _unscaled_delta_time; }

    // Get fixed delta time for physics updates.
    float fixed_delta_time() const { return _fixed_delta_time; }

    // Set fixed delta time for physics updates (default: 1/60).
    void set_fixed_delta_time(float dt);

    // Get time scale multiplier (default: 1.0).
    float time_scale() const { return _time_scale; }

    // Set time scale multiplier (0 = paused, 0.5 = half speed, 2 = double speed).
    void set_time_scale(float scale);

    // Get total elapsed time in seconds since start.
    float total_time() const { return _total_time; }

    // Get total unscaled elapsed time in seconds since start.
    float unscaled_total_time() const { return _unscaled_total_time; }

    // Get accumulated fixed time (for physics step loop).
    float fixed_accumulator() const { return _fixed_accumulator; }

    // Consume fixed timestep from accumulator. Returns true if step should run.
    bool consume_fixed_step();

    // Get interpolation alpha for rendering (0.0 to 1.0).
    // Use this to interpolate between previous and current physics state for smooth rendering.
    float interpolation_alpha() const;

    // Get frame count since start.
    uint64_t frame_count() const { return _frame_count; }

private:
    using Clock = std::chrono::high_resolution_clock;
    using TimePoint = std::chrono::time_point<Clock>;

    TimePoint _last_time;
    TimePoint _start_time;

    float _delta_time{0.0f};
    float _unscaled_delta_time{0.0f};
    float _fixed_delta_time{1.0f / 60.0f};
    float _time_scale{1.0f};
    float _total_time{0.0f};
    float _unscaled_total_time{0.0f};
    float _fixed_accumulator{0.0f};

    uint64_t _frame_count{0};

    static constexpr float k_max_delta_time = 0.1f; // 100ms cap
};

} // namespace GameRuntime