#include "frame_monitor.h"

#include "imgui.h"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace Game
{
    void FrameMonitor::update(float dt)
    {
        _frame_times[_write_index] = dt;
        _write_index = (_write_index + 1) % kHistorySize;
        if (_sample_count < kHistorySize)
        {
            ++_sample_count;
        }

        if (_sample_count == 0)
        {
            return;
        }

        float min_dt = _frame_times[0];
        float max_dt = _frame_times[0];
        float sum_dt = 0.0f;

        for (size_t i = 0; i < _sample_count; ++i)
        {
            const float t = _frame_times[i];
            min_dt = std::min(min_dt, t);
            max_dt = std::max(max_dt, t);
            sum_dt += t;
        }

        const float avg_dt = sum_dt / static_cast<float>(_sample_count);

        _min_fps = (max_dt > 0.0f) ? 1.0f / max_dt : 0.0f;
        _max_fps = (min_dt > 0.0f) ? 1.0f / min_dt : 0.0f;
        _avg_fps = (avg_dt > 0.0f) ? 1.0f / avg_dt : 0.0f;
    }

    void FrameMonitor::draw_ui() const
    {
        const ImGuiViewport *viewport = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(
            ImVec2(viewport->WorkPos.x + viewport->WorkSize.x - 10.0f,
                   viewport->WorkPos.y + 10.0f),
            ImGuiCond_FirstUseEver,
            ImVec2(1.0f, 0.0f));
        ImGui::SetNextWindowBgAlpha(0.4f);

        ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration
                               | ImGuiWindowFlags_AlwaysAutoResize
                               | ImGuiWindowFlags_NoFocusOnAppearing
                               | ImGuiWindowFlags_NoNav;

        if (ImGui::Begin("##FrameMonitor", nullptr, flags))
        {
            ImGui::Text("FPS: %.1f", _avg_fps);
            ImGui::Separator();
            ImGui::Text("Avg: %.1f", _avg_fps);
            ImGui::Text("Min: %.1f", _min_fps);
            ImGui::Text("Max: %.1f", _max_fps);

            // Frame time graph
            float history[kHistorySize];
            for (size_t i = 0; i < _sample_count; ++i)
            {
                const size_t idx = (_write_index + kHistorySize - _sample_count + i) % kHistorySize;
                history[i] = _frame_times[idx] * 1000.0f; // ms
            }

            if (_sample_count > 1)
            {
                ImGui::PlotLines("##ft", history, static_cast<int>(_sample_count),
                                 0, "ms/frame", 0.0f, 50.0f, ImVec2(160, 40));
            }
        }
        ImGui::End();
    }
} // namespace Game
