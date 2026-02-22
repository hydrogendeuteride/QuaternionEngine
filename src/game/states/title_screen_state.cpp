#include "title_screen_state.h"
#include "gameplay/gameplay_state.h"
#include "settings_state.h"

#include "imgui.h"

namespace Game
{

void TitleScreenState::on_enter(GameStateContext &ctx)
{
    (void)ctx;
}

void TitleScreenState::on_exit(GameStateContext &ctx)
{
    (void)ctx;
}

void TitleScreenState::on_update(GameStateContext &ctx, float dt)
{
    (void)ctx;
    (void)dt;
}

void TitleScreenState::on_fixed_update(GameStateContext &ctx, float fixed_dt)
{
    (void)ctx;
    (void)fixed_dt;
}

void TitleScreenState::on_draw_ui(GameStateContext &ctx)
{
    const ImGuiViewport *viewport = ImGui::GetMainViewport();
    const ImVec2 center = ImVec2(viewport->WorkPos.x + viewport->WorkSize.x * 0.5f,
                                 viewport->WorkPos.y + viewport->WorkSize.y * 0.5f);

    ImGui::SetNextWindowPos(center, ImGuiCond_Always, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(320, 0));

    ImGuiWindowFlags flags = ImGuiWindowFlags_NoResize
                           | ImGuiWindowFlags_NoMove
                           | ImGuiWindowFlags_NoCollapse
                           | ImGuiWindowFlags_AlwaysAutoResize;

    if (ImGui::Begin("Space Combat", nullptr, flags))
    {
        // Title
        ImGui::SetCursorPosX((ImGui::GetWindowWidth() - ImGui::CalcTextSize("SPACE COMBAT").x) * 0.5f);
        ImGui::TextUnformatted("SPACE COMBAT");
        ImGui::Separator();
        ImGui::Spacing();

        const float button_width = ImGui::GetContentRegionAvail().x;

        if (ImGui::Button("New Game", ImVec2(button_width, 40)))
        {
            _pending = StateTransition::switch_to<GameplayState>();
        }

        // TODO: Load game (needs save system)
        ImGui::BeginDisabled(true);
        ImGui::Button("Load Game", ImVec2(button_width, 40));
        ImGui::EndDisabled();

        if (ImGui::Button("Settings", ImVec2(button_width, 40)))
        {
            _pending = StateTransition::push<SettingsState>();
        }

        ImGui::Spacing();

        if (ImGui::Button("Quit", ImVec2(button_width, 40)))
        {
            ctx.quit();
        }
    }
    ImGui::End();
}

} // namespace Game
