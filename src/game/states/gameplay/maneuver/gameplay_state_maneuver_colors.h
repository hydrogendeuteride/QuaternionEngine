#pragma once

#include "game/states/gameplay/maneuver/gameplay_state_maneuver_types.h"

#include <glm/glm.hpp>

#include "imgui.h"

namespace Game::ManeuverColors
{
    inline constexpr ImU32 kAxisTangential = IM_COL32(95, 235, 105, 220);
    inline constexpr ImU32 kAxisRadial = IM_COL32(240, 100, 100, 220);
    inline constexpr ImU32 kAxisNormal = IM_COL32(95, 160, 255, 220);
    inline constexpr ImU32 kAxisDefault = IM_COL32(170, 170, 170, 180);

    inline const ImVec4 kHubSelectedFill{1.0f, 0.80f, 0.35f, 0.92f};
    inline const ImVec4 kHubSelectedRing{1.0f, 0.90f, 0.55f, 0.86f};

    inline constexpr ImU32 kDragAxisActive = IM_COL32(255, 190, 70, 240);
    inline constexpr ImU32 kDragAxisHovered = IM_COL32(255, 240, 140, 230);

    inline constexpr ImU32 kOrbitPickPlannedHover = IM_COL32(255, 190, 80, 230);
    inline constexpr ImU32 kOrbitPickBaseHover = IM_COL32(170, 120, 255, 220);
    inline constexpr ImU32 kOrbitPickPlannedSelected = IM_COL32(255, 190, 80, 255);
    inline constexpr ImU32 kOrbitPickBaseSelected = IM_COL32(170, 120, 255, 245);
    inline constexpr ImU32 kOrbitPickOuterHover = IM_COL32(255, 255, 255, 80);
    inline constexpr ImU32 kOrbitPickOuterSelected = IM_COL32(255, 255, 255, 120);

    inline constexpr ImU32 kTimelineBackground = IM_COL32(0, 0, 0, 90);
    inline constexpr ImU32 kTimelineBorder = IM_COL32(255, 255, 255, 32);
    inline constexpr ImU32 kTimelineTick = IM_COL32(255, 255, 255, 18);
    inline constexpr ImU32 kTimelineTickText = IM_COL32(255, 255, 255, 110);
    inline constexpr ImU32 kTimelineNodeSelected = IM_COL32(255, 210, 80, 220);
    inline constexpr ImU32 kTimelineNodeUnselected = IM_COL32(80, 200, 255, 190);
    inline constexpr ImU32 kTimelineNodeSelectedRing = IM_COL32(255, 210, 80, 110);

    inline constexpr ManeuverGizmoBasisMode kBasisModes[]{
        ManeuverGizmoBasisMode::ProgradeOutwardNormal,
        ManeuverGizmoBasisMode::RTN,
    };
    inline const ImVec4 kBasisButtonActive[]{
        ImVec4(0.20f, 0.65f, 0.30f, 1.0f),
        ImVec4(0.70f, 0.50f, 0.15f, 1.0f),
    };
    inline const ImVec4 kBasisButtonHovered[]{
        ImVec4(0.25f, 0.55f, 0.30f, 0.70f),
        ImVec4(0.55f, 0.40f, 0.15f, 0.70f),
    };
    inline const ImVec4 kBasisButtonInactive{0.20f, 0.20f, 0.22f, 0.60f};

    inline const glm::vec4 kDebugNode{0.3f, 0.8f, 1.0f, 0.85f};
    inline const glm::vec4 kDebugNodeSelected{1.0f, 0.82f, 0.25f, 0.95f};
    inline const glm::vec4 kDebugDv{0.2f, 0.7f, 1.0f, 0.9f};
    inline const glm::vec4 kDebugRadial{1.0f, 0.25f, 0.25f, 0.75f};
    inline const glm::vec4 kDebugTangential{0.25f, 1.0f, 0.25f, 0.75f};
    inline const glm::vec4 kDebugNormal{0.25f, 0.6f, 1.0f, 0.75f};
} // namespace Game::ManeuverColors
