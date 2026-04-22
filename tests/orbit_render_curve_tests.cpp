#include "game/orbit/orbit_render_curve.h"

#include <gtest/gtest.h>

namespace
{
    orbitsim::TrajectorySegment make_segment(const double t0_s,
                                             const double dt_s,
                                             const glm::dvec3 &p0,
                                             const glm::dvec3 &v0,
                                             const glm::dvec3 &p1,
                                             const glm::dvec3 &v1)
    {
        orbitsim::TrajectorySegment segment{};
        segment.t0_s = t0_s;
        segment.dt_s = dt_s;
        segment.start = orbitsim::make_state(p0, v0);
        segment.end = orbitsim::make_state(p1, v1);
        return segment;
    }

    std::vector<orbitsim::TrajectorySegment> make_curved_segments()
    {
        std::vector<orbitsim::TrajectorySegment> segments{};
        segments.push_back(make_segment(0.0,
                                        1.0,
                                        glm::dvec3(0.0, 0.0, 0.0),
                                        glm::dvec3(1.0, 4.0, 0.0),
                                        glm::dvec3(1.0, 2.0, 0.0),
                                        glm::dvec3(1.0, 0.0, 0.0)));
        segments.push_back(make_segment(1.0,
                                        1.0,
                                        glm::dvec3(1.0, 2.0, 0.0),
                                        glm::dvec3(1.0, 0.0, 0.0),
                                        glm::dvec3(2.0, 0.0, 0.0),
                                        glm::dvec3(1.0, -4.0, 0.0)));
        segments.push_back(make_segment(2.0,
                                        1.0,
                                        glm::dvec3(2.0, 0.0, 0.0),
                                        glm::dvec3(1.0, -4.0, 0.0),
                                        glm::dvec3(3.0, -2.0, 0.0),
                                        glm::dvec3(1.0, 0.0, 0.0)));
        segments.push_back(make_segment(3.0,
                                        1.0,
                                        glm::dvec3(3.0, -2.0, 0.0),
                                        glm::dvec3(1.0, 0.0, 0.0),
                                        glm::dvec3(4.0, 0.0, 0.0),
                                        glm::dvec3(1.0, 4.0, 0.0)));
        return segments;
    }

    void expect_vec3_near(const glm::dvec3 &actual, const glm::dvec3 &expected, const double tol)
    {
        EXPECT_NEAR(actual.x, expected.x, tol);
        EXPECT_NEAR(actual.y, expected.y, tol);
        EXPECT_NEAR(actual.z, expected.z, tol);
    }
} // namespace

TEST(TrajectorySegmentEvaluationTests, EvaluatesHermiteStateAndClampsToSegmentBounds)
{
    const orbitsim::TrajectorySegment segment = make_segment(10.0,
                                                             2.0,
                                                             glm::dvec3(0.0, 0.0, 0.0),
                                                             glm::dvec3(5.0, 0.0, 0.0),
                                                             glm::dvec3(10.0, 0.0, 0.0),
                                                             glm::dvec3(5.0, 0.0, 0.0));

    const orbitsim::State mid = orbitsim::trajectory_segment_state_at(segment, 11.0);
    expect_vec3_near(mid.position_m, glm::dvec3(5.0, 0.0, 0.0), 1.0e-12);
    expect_vec3_near(mid.velocity_mps, glm::dvec3(5.0, 0.0, 0.0), 1.0e-12);

    expect_vec3_near(orbitsim::trajectory_segment_position_at(segment, 9.0),
                     segment.start.position_m,
                     1.0e-12);
    expect_vec3_near(orbitsim::trajectory_segment_position_at(segment, 13.0),
                     segment.end.position_m,
                     1.0e-12);
}

TEST(OrbitRenderCurveTests, BuildCreatesHierarchyForCurvedSegments)
{
    const std::vector<orbitsim::TrajectorySegment> segments = make_curved_segments();
    const Game::OrbitRenderCurve curve = Game::OrbitRenderCurve::build(segments);

    ASSERT_FALSE(curve.empty());
    ASSERT_EQ(curve.nodes().size(), (segments.size() * 2u) - 1u);

    const Game::OrbitRenderCurve::Node &root = curve.nodes()[curve.root_index()];
    EXPECT_FALSE(root.is_leaf());
    EXPECT_GT(root.max_error_m, 0.0);
}

TEST(OrbitRenderCurveTests, SelectSegmentsUsesMergedNodeWhenCurveIsDistant)
{
    const std::vector<orbitsim::TrajectorySegment> segments = make_curved_segments();
    const Game::OrbitRenderCurve curve = Game::OrbitRenderCurve::build(segments);

    Game::OrbitRenderCurve::SelectionContext ctx{};
    ctx.camera_world = glm::dvec3(0.0, 0.0, 1.0e6);
    ctx.tan_half_fov = 1.0;
    ctx.viewport_height_px = 1080.0;
    ctx.error_px = 0.5;

    std::vector<orbitsim::TrajectorySegment> selected{};
    Game::OrbitRenderCurve::select_segments(curve, ctx, 0.0, 4.0, selected);

    ASSERT_FALSE(selected.empty());
    EXPECT_LT(selected.size(), segments.size());
}

TEST(OrbitRenderCurveTests, SelectSegmentsDescendsWhenErrorBudgetIsTiny)
{
    const std::vector<orbitsim::TrajectorySegment> segments = make_curved_segments();
    const Game::OrbitRenderCurve curve = Game::OrbitRenderCurve::build(segments);

    Game::OrbitRenderCurve::SelectionContext ctx{};
    ctx.camera_world = glm::dvec3(2.0, 0.0, 0.1);
    ctx.tan_half_fov = 1.0;
    ctx.viewport_height_px = 1.0e9;
    ctx.error_px = 1.0e-6;

    std::vector<orbitsim::TrajectorySegment> selected{};
    Game::OrbitRenderCurve::select_segments(curve, ctx, 0.0, 4.0, selected);

    ASSERT_GT(selected.size(), 1u);
    ASSERT_LE(selected.size(), segments.size());
    EXPECT_DOUBLE_EQ(selected.front().t0_s, segments.front().t0_s);
    EXPECT_DOUBLE_EQ(selected.back().t0_s + selected.back().dt_s,
                     segments.back().t0_s + segments.back().dt_s);
}

TEST(OrbitRenderCurveTests, PickLodKeepsLongSegmentCrossingFrustum)
{
    const std::vector<orbitsim::TrajectorySegment> segments{
            make_segment(0.0,
                         1.0,
                         glm::dvec3(-2.0, 0.0, 0.5),
                         glm::dvec3(12.0, 0.0, 0.0),
                         glm::dvec3(10.0, 0.0, 0.5),
                         glm::dvec3(12.0, 0.0, 0.0)),
    };

    Game::OrbitRenderCurve::FrustumContext frustum{};
    frustum.valid = true;
    frustum.viewproj = glm::mat4(1.0f);

    Game::OrbitRenderCurve::PickSettings settings{};
    settings.max_segments = 8;
    settings.frustum_margin_ratio = 0.0;

    const Game::OrbitRenderCurve::PickResult result =
            Game::OrbitRenderCurve::build_pick_lod(segments,
                                                   WorldVec3(0.0, 0.0, 0.0),
                                                   WorldVec3(0.0, 0.0, 0.0),
                                                   frustum,
                                                   settings,
                                                   0.0,
                                                   1.0);

    EXPECT_EQ(result.segments_before_cull, 1u);
    EXPECT_EQ(result.segments_after_cull, 1u);
    ASSERT_EQ(result.segments.size(), 1u);
    EXPECT_DOUBLE_EQ(result.segments.front().t0_s, 0.0);
    EXPECT_DOUBLE_EQ(result.segments.front().t1_s, 1.0);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
