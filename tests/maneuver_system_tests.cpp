#include "game/states/gameplay/maneuver/maneuver_system.h"

#include <gtest/gtest.h>

#include <limits>

namespace
{
    Game::ManeuverNode make_node(const int id,
                                 const double time_s,
                                 const glm::dvec3 &dv_rtn_mps = glm::dvec3(0.0))
    {
        Game::ManeuverNode node{};
        node.id = id;
        node.time_s = time_s;
        node.dv_rtn_mps = dv_rtn_mps;
        return node;
    }
} // namespace

TEST(ManeuverSystem, ResetSessionClearsPlanRuntimeAndTransientStateButKeepsRevision)
{
    Game::ManeuverSystem system{};
    ASSERT_TRUE(system.apply_command(Game::ManeuverCommand::add_node(make_node(7, 120.0))).applied);

    system.set_revision(42);
    system.runtime().arm_warp_to_time(300.0, 2);
    system.runtime().arm_execute_node(7);
    system.gizmo_interaction().state = Game::ManeuverGizmoInteraction::State::DragAxis;
    system.gizmo_interaction().node_id = 7;
    ASSERT_TRUE(system.begin_dv_edit_preview(7));
    ASSERT_TRUE(system.mark_edit_preview_changed(Game::ManeuverNodeEditPreview::State::EditingDv, 7));

    system.reset_session();

    EXPECT_EQ(system.revision(), 42u);
    EXPECT_TRUE(system.plan().nodes.empty());
    EXPECT_EQ(system.plan().selected_node_id, -1);
    EXPECT_EQ(system.plan().next_node_id, 0);
    EXPECT_FALSE(system.runtime().warp_to_time_active);
    EXPECT_FALSE(system.runtime().execute_node_armed);
    EXPECT_EQ(system.runtime().execute_node_id, -1);
    EXPECT_EQ(system.gizmo_interaction().state, Game::ManeuverGizmoInteraction::State::Idle);
    EXPECT_EQ(system.gizmo_interaction().node_id, -1);
    EXPECT_EQ(system.edit_preview().state, Game::ManeuverNodeEditPreview::State::Idle);
    EXPECT_EQ(system.edit_preview().node_id, -1);
}

TEST(ManeuverSystem, RemovingNodeClearsDependentRuntimeInteractionAndEditPreview)
{
    Game::ManeuverSystem system{};
    ASSERT_TRUE(system.apply_command(Game::ManeuverCommand::add_node(make_node(1, 100.0))).applied);
    ASSERT_TRUE(system.apply_command(Game::ManeuverCommand::add_node(make_node(2, 200.0))).applied);

    system.runtime().arm_execute_node(1);
    system.gizmo_interaction().state = Game::ManeuverGizmoInteraction::State::HoverAxis;
    system.gizmo_interaction().node_id = 1;
    system.gizmo_interaction().axis = Game::ManeuverHandleAxis::TangentialPos;
    ASSERT_TRUE(system.begin_time_edit_preview(1, 90.0));
    ASSERT_TRUE(system.mark_edit_preview_changed(Game::ManeuverNodeEditPreview::State::EditingTime, 1));

    const Game::ManeuverCommandResult result = system.apply_command(Game::ManeuverCommand::remove_node(1));

    EXPECT_TRUE(result.applied);
    EXPECT_TRUE(result.nodes_removed);
    EXPECT_FALSE(system.runtime().execute_node_armed);
    EXPECT_EQ(system.runtime().execute_node_id, -1);
    EXPECT_EQ(system.gizmo_interaction().state, Game::ManeuverGizmoInteraction::State::Idle);
    EXPECT_EQ(system.gizmo_interaction().node_id, -1);
    EXPECT_EQ(system.edit_preview().state, Game::ManeuverNodeEditPreview::State::Idle);
    EXPECT_EQ(system.edit_preview().node_id, -1);
    EXPECT_NE(system.plan().find_node(2), nullptr);
}

TEST(ManeuverSystem, EditPreviewApiTracksChangesAndCancelsMissingNodes)
{
    Game::ManeuverSystem system{};
    ASSERT_TRUE(system.apply_command(Game::ManeuverCommand::add_node(make_node(3, 600.0))).applied);

    EXPECT_TRUE(system.begin_dv_edit_preview(3));
    EXPECT_EQ(system.edit_preview().state, Game::ManeuverNodeEditPreview::State::EditingDv);
    EXPECT_EQ(system.edit_preview().node_id, 3);
    EXPECT_TRUE(system.mark_edit_preview_changed(Game::ManeuverNodeEditPreview::State::EditingDv, 3));
    EXPECT_TRUE(system.finish_edit_preview(Game::ManeuverNodeEditPreview::State::EditingDv, false));
    EXPECT_EQ(system.edit_preview().state, Game::ManeuverNodeEditPreview::State::Idle);

    EXPECT_TRUE(system.begin_time_edit_preview(3, std::numeric_limits<double>::quiet_NaN()));
    EXPECT_DOUBLE_EQ(system.edit_preview().start_time_s, 600.0);
    EXPECT_FALSE(system.finish_edit_preview(Game::ManeuverNodeEditPreview::State::EditingTime, false));

    EXPECT_TRUE(system.begin_time_edit_preview(3, 500.0));
    EXPECT_DOUBLE_EQ(system.edit_preview().start_time_s, 500.0);

    EXPECT_FALSE(system.begin_dv_edit_preview(99));
    EXPECT_EQ(system.edit_preview().state, Game::ManeuverNodeEditPreview::State::Idle);
    EXPECT_EQ(system.edit_preview().node_id, -1);
}

TEST(ManeuverSystem, LivePreviewAndPlanSignatureReflectSystemState)
{
    Game::ManeuverSystem system{};
    ASSERT_TRUE(system.apply_command(Game::ManeuverCommand::add_node(make_node(4, 100.0, glm::dvec3(1.0, 0.0, 0.0)))).applied);

    EXPECT_FALSE(system.live_preview_active(false));
    EXPECT_FALSE(system.live_preview_active(true));

    ASSERT_TRUE(system.begin_dv_edit_preview(4));
    EXPECT_TRUE(system.live_preview_active(true));
    EXPECT_EQ(system.active_preview_anchor_node_id(), 4);
    system.cancel_edit_preview();

    system.gizmo_interaction().state = Game::ManeuverGizmoInteraction::State::DragAxis;
    system.gizmo_interaction().node_id = 4;
    EXPECT_TRUE(system.live_preview_active(true));
    EXPECT_EQ(system.active_preview_anchor_node_id(), 4);

    const uint64_t baseline = system.plan_signature(600.0);
    system.settings().plan_windows.preview_window_s += 10.0;
    EXPECT_NE(system.plan_signature(600.0), baseline);

    const uint64_t after_settings = system.plan_signature(600.0);
    ASSERT_TRUE(system.apply_command(Game::ManeuverCommand::set_node_dv(4, glm::dvec3(2.0, 0.0, 0.0))).applied);
    EXPECT_NE(system.plan_signature(600.0), after_settings);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
