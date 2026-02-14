#include "pause_state.h"
#include "title_screen_state.h"
#include "settings_state.h"

#include "imgui.h"

namespace Game
{

void PauseState::on_enter(GameStateContext &ctx)
{
    (void)ctx;
}

void PauseState::on_exit(GameStateContext &ctx)
{
    (void)ctx;
}

void PauseState::on_update(GameStateContext &ctx, float dt)
{
    (void)ctx;
    (void)dt;

    // ESC to resume
    if (ImGui::IsKeyPressed(ImGuiKey_Escape, false))
    {
        _pending = StateTransition::pop();
    }
}

void PauseState::on_fixed_update(GameStateContext &ctx, float fixed_dt)
{
    (void)ctx;
    (void)fixed_dt;
}

void PauseState::on_draw_ui(GameStateContext &ctx)
{
    // Dim overlay
    const ImGuiViewport *viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(viewport->WorkPos);
    ImGui::SetNextWindowSize(viewport->WorkSize);

    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.0f, 0.0f, 0.0f, 0.5f));
    ImGui::Begin("##PauseDim", nullptr,
                 ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_NoBringToFrontOnFocus);
    ImGui::End();
    ImGui::PopStyleColor();

    // Pause menu
    const ImVec2 center = ImVec2(viewport->WorkPos.x + viewport->WorkSize.x * 0.5f,
                                 viewport->WorkPos.y + viewport->WorkSize.y * 0.5f);
    ImGui::SetNextWindowPos(center, ImGuiCond_Always, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(280, 0));

    ImGuiWindowFlags flags = ImGuiWindowFlags_NoResize
                           | ImGuiWindowFlags_NoMove
                           | ImGuiWindowFlags_NoCollapse
                           | ImGuiWindowFlags_AlwaysAutoResize;

    if (ImGui::Begin("Paused", nullptr, flags))
    {
        const float button_width = ImGui::GetContentRegionAvail().x;

        if (ImGui::Button("Resume", ImVec2(button_width, 36)))
        {
            _pending = StateTransition::pop();
        }

        if (ImGui::Button("Settings", ImVec2(button_width, 36)))
        {
            _pending = StateTransition::push<SettingsState>();
        }

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        if (ImGui::Button("Main Menu", ImVec2(button_width, 36)))
        {
            _pending = StateTransition::switch_to<TitleScreenState>();
        }

        if (ImGui::Button("Quit", ImVec2(button_width, 36)))
        {
            ctx.quit();
        }
    }
    ImGui::End();
}

} // namespace Game
