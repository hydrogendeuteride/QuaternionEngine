#pragma once

#include <array>
#include <cstddef>

namespace Game
{
    class FrameMonitor
    {
    public:
        static constexpr size_t kHistorySize = 300;

        void update(float dt);
        void draw_ui() const;

    private:
        std::array<float, kHistorySize> _frame_times{};
        size_t _write_index{0};
        size_t _sample_count{0};

        float _min_fps{0.0f};
        float _max_fps{0.0f};
        float _avg_fps{0.0f};
    };
} // namespace Game
