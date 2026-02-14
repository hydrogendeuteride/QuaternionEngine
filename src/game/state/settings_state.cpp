#include "settings_state.h"
#include "runtime/game_runtime.h"

#include "imgui.h"

namespace Game
{

void SettingsState::on_enter(GameStateContext &ctx)
{
    // Load current settings from audio system
    if (ctx.audio)
    {
        _master_volume = ctx.audio->master_volume();
        _sfx_volume = ctx.audio->get_bus_volume(GameRuntime::IAudioSystem::Bus::Sfx);
        _bgm_volume = ctx.audio->get_bus_volume(GameRuntime::IAudioSystem::Bus::Bgm);
    }
}

void SettingsState::on_exit(GameStateContext &ctx)
{
    (void)ctx;
}

void SettingsState::on_update(GameStateContext &ctx, float dt)
{
    (void)ctx;
    (void)dt;

    if (ImGui::IsKeyPressed(ImGuiKey_Escape, false))
    {
        _pending = StateTransition::pop();
    }
}

void SettingsState::on_fixed_update(GameStateContext &ctx, float fixed_dt)
{
    (void)ctx;
    (void)fixed_dt;
}

void SettingsState::on_draw_ui(GameStateContext &ctx)
{
    const ImGuiViewport *viewport = ImGui::GetMainViewport();
    const ImVec2 center = ImVec2(viewport->WorkPos.x + viewport->WorkSize.x * 0.5f,
                                 viewport->WorkPos.y + viewport->WorkSize.y * 0.5f);

    ImGui::SetNextWindowPos(center, ImGuiCond_Always, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(400, 0));

    ImGuiWindowFlags flags = ImGuiWindowFlags_NoResize
                           | ImGuiWindowFlags_NoMove
                           | ImGuiWindowFlags_NoCollapse
                           | ImGuiWindowFlags_AlwaysAutoResize;

    if (ImGui::Begin("Settings", nullptr, flags))
    {
        // Audio settings
        if (ImGui::CollapsingHeader("Audio", ImGuiTreeNodeFlags_DefaultOpen))
        {
            bool changed = false;
            changed |= ImGui::SliderFloat("Master Volume", &_master_volume, 0.0f, 1.0f, "%.2f");
            changed |= ImGui::SliderFloat("SFX Volume", &_sfx_volume, 0.0f, 1.0f, "%.2f");
            changed |= ImGui::SliderFloat("BGM Volume", &_bgm_volume, 0.0f, 1.0f, "%.2f");

            // Apply immediately for audio feedback
            if (changed && ctx.audio)
            {
                ctx.audio->set_master_volume(_master_volume);
                ctx.audio->set_bus_volume(GameRuntime::IAudioSystem::Bus::Sfx, _sfx_volume);
                ctx.audio->set_bus_volume(GameRuntime::IAudioSystem::Bus::Bgm, _bgm_volume);
            }
        }

        // Graphics settings placeholder
        if (ImGui::CollapsingHeader("Graphics", ImGuiTreeNodeFlags_DefaultOpen))
        {
            ImGui::TextDisabled("(Not yet implemented)");
        }

        // Controls placeholder
        if (ImGui::CollapsingHeader("Controls", ImGuiTreeNodeFlags_DefaultOpen))
        {
            ImGui::TextDisabled("(Not yet implemented)");
        }

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        if (ImGui::Button("Back", ImVec2(ImGui::GetContentRegionAvail().x, 36)))
        {
            _pending = StateTransition::pop();
        }
    }
    ImGui::End();
}

} // namespace Game
