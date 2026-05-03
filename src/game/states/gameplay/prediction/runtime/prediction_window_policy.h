#pragma once

#include "game/orbit/orbit_prediction_tuning.h"
#include "game/states/gameplay/maneuver/gameplay_state_maneuver_types.h"
#include "game/states/gameplay/prediction/gameplay_state_prediction_types.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace Game::PredictionWindowPolicy
{
    inline constexpr double kTimeEpsilonS = 1.0e-6;

    struct PlannerInput
    {
        const PredictionTrackState *track{nullptr};
        PredictionTimeContext time_ctx{};
        ManeuverPlanWindowSettings plan_windows{};
        double future_window_s{0.0};
        double plan_horizon_s{0.0};
        bool draw_future_segment{false};
        bool draw_full_orbit{false};
        bool with_maneuvers{false};
        bool live_preview_active{false};
    };

    struct WindowAnchor
    {
        double time_s{std::numeric_limits<double>::quiet_NaN()};
        PredictionTimeAnchorKind kind{PredictionTimeAnchorKind::None};
        bool is_future{false};
    };

    struct AuthoredPlanWindow
    {
        bool valid{false};
        double t0_s{std::numeric_limits<double>::quiet_NaN()};
        double t1_s{std::numeric_limits<double>::quiet_NaN()};
    };

    inline double resolve_authored_plan_end_s(const PredictionTimeContext &time_ctx,
                                              const double plan_start_time_s,
                                              const double final_node_time_s,
                                              const double plan_horizon_s,
                                              const double post_node_coverage_s)
    {
        if (!std::isfinite(time_ctx.sim_now_s) ||
            !std::isfinite(plan_start_time_s) ||
            !std::isfinite(final_node_time_s) ||
            !(plan_horizon_s > 0.0))
        {
            return std::numeric_limits<double>::quiet_NaN();
        }

        const double horizon_end_s = plan_start_time_s + plan_horizon_s;
        const double post_node_end_s = final_node_time_s + std::max(0.0, post_node_coverage_s);
        return std::max(horizon_end_s, post_node_end_s);
    }

    inline bool node_time_in_context_range(const PredictionTimeContext &time_ctx, const double node_time_s)
    {
        if (!std::isfinite(node_time_s))
        {
            return false;
        }

        if (std::isfinite(time_ctx.trajectory_t0_s) && node_time_s < (time_ctx.trajectory_t0_s - kTimeEpsilonS))
        {
            return false;
        }

        if (std::isfinite(time_ctx.trajectory_t1_s) && node_time_s > (time_ctx.trajectory_t1_s + kTimeEpsilonS))
        {
            return false;
        }

        return true;
    }

    inline WindowAnchor make_window_anchor(const PredictionTimeContext &time_ctx,
                                           const double time_s,
                                           const PredictionTimeAnchorKind kind)
    {
        WindowAnchor out{};
        out.time_s = time_s;
        out.kind = kind;
        out.is_future =
                std::isfinite(time_s) &&
                std::isfinite(time_ctx.sim_now_s) &&
                time_s + kTimeEpsilonS >= time_ctx.sim_now_s;
        return out;
    }

    inline AuthoredPlanWindow resolve_authored_plan_window(const PredictionTimeContext &time_ctx,
                                                           const double plan_horizon_s,
                                                           const double post_node_coverage_s)
    {
        AuthoredPlanWindow out{};
        if (!(plan_horizon_s > 0.0) ||
            !time_ctx.has_plan ||
            !std::isfinite(time_ctx.sim_now_s) ||
            !std::isfinite(time_ctx.first_future_node_time_s))
        {
            return out;
        }

        out.t0_s = time_ctx.first_future_node_time_s;
        const double final_node_time_s =
                std::isfinite(time_ctx.last_future_node_time_s)
                        ? time_ctx.last_future_node_time_s
                        : out.t0_s;
        out.t1_s = resolve_authored_plan_end_s(time_ctx,
                                               out.t0_s,
                                               final_node_time_s,
                                               plan_horizon_s,
                                               post_node_coverage_s);
        out.valid = std::isfinite(out.t1_s) &&
                    out.t1_s > (out.t0_s + kTimeEpsilonS);
        return out;
    }

    inline WindowAnchor select_visual_anchor(const PredictionTimeContext &time_ctx)
    {
        if (std::isfinite(time_ctx.selected_node_time_s) &&
            time_ctx.selected_node_time_s + kTimeEpsilonS >= time_ctx.sim_now_s)
        {
            return make_window_anchor(time_ctx,
                                      time_ctx.selected_node_time_s,
                                      PredictionTimeAnchorKind::SelectedNode);
        }
        if (std::isfinite(time_ctx.first_future_node_time_s))
        {
            return make_window_anchor(time_ctx,
                                      time_ctx.first_future_node_time_s,
                                      PredictionTimeAnchorKind::FirstFutureNode);
        }
        return make_window_anchor(time_ctx, time_ctx.sim_now_s, PredictionTimeAnchorKind::SimNow);
    }

    inline WindowAnchor select_exact_anchor(const PredictionTimeContext &time_ctx)
    {
        if (std::isfinite(time_ctx.selected_node_time_s))
        {
            return make_window_anchor(time_ctx,
                                      time_ctx.selected_node_time_s,
                                      PredictionTimeAnchorKind::SelectedNode);
        }
        if (std::isfinite(time_ctx.first_future_node_time_s))
        {
            return make_window_anchor(time_ctx,
                                      time_ctx.first_future_node_time_s,
                                      PredictionTimeAnchorKind::FirstFutureNode);
        }
        if (std::isfinite(time_ctx.first_relevant_node_time_s))
        {
            return make_window_anchor(time_ctx,
                                      time_ctx.first_relevant_node_time_s,
                                      PredictionTimeAnchorKind::FirstRelevantNode);
        }
        return make_window_anchor(time_ctx, time_ctx.sim_now_s, PredictionTimeAnchorKind::SimNow);
    }

    inline WindowAnchor select_pick_anchor(const PredictionTimeContext &time_ctx)
    {
        if (std::isfinite(time_ctx.selected_node_time_s) &&
            time_ctx.selected_node_time_s + kTimeEpsilonS >= time_ctx.sim_now_s)
        {
            return make_window_anchor(time_ctx,
                                      time_ctx.selected_node_time_s,
                                      PredictionTimeAnchorKind::SelectedNode);
        }
        if (std::isfinite(time_ctx.first_future_node_time_s))
        {
            return make_window_anchor(time_ctx,
                                      time_ctx.first_future_node_time_s,
                                      PredictionTimeAnchorKind::FirstFutureNode);
        }
        return make_window_anchor(time_ctx, time_ctx.sim_now_s, PredictionTimeAnchorKind::SimNow);
    }

    inline double resolve_live_preview_visual_window_s(const double request_window_s,
                                                       const double anchor_offset_s,
                                                       const double configured_preview_window_s)
    {
        const double available_window_s = std::max(0.0, request_window_s - std::max(0.0, anchor_offset_s));
        const double preview_window_s = std::max(0.0, configured_preview_window_s);
        if (preview_window_s <= 0.0)
        {
            return available_window_s;
        }
        return std::min(available_window_s, preview_window_s);
    }

    inline PredictionWindowPolicyResult plan_windows(const PlannerInput &input)
    {
        const PredictionTrackState *track = input.track;
        const PredictionTimeContext &time_ctx = input.time_ctx;

        PredictionWindowPolicyResult out{};
        out.request_window_s =
                std::max(0.0, input.draw_future_segment ? input.future_window_s : 0.0);
        const double solve_margin_s = std::max(0.0, input.plan_windows.solve_margin_s);
        const double post_node_coverage_s = OrbitPredictionTuning::kPostNodeCoverageMinS;
        const bool supports_maneuver_windows = input.with_maneuvers && time_ctx.has_plan;
        const double plan_horizon_s = input.plan_horizon_s;
        const AuthoredPlanWindow authored_plan_window =
                supports_maneuver_windows
                        ? resolve_authored_plan_window(time_ctx, plan_horizon_s, post_node_coverage_s)
                        : AuthoredPlanWindow{};

        if (std::isfinite(time_ctx.last_future_node_time_s) &&
            std::isfinite(time_ctx.sim_now_s) &&
            time_ctx.last_future_node_time_s > time_ctx.sim_now_s)
        {
            const double required_plan_end_s =
                    authored_plan_window.valid
                            ? authored_plan_window.t1_s
                            : (time_ctx.last_future_node_time_s + post_node_coverage_s);
            out.request_window_s = std::max(
                    out.request_window_s,
                    std::max(0.0, required_plan_end_s - time_ctx.sim_now_s));
        }

        if (!supports_maneuver_windows)
        {
            return out;
        }

        const WindowAnchor visual_anchor = select_visual_anchor(time_ctx);
        const WindowAnchor pick_anchor = select_pick_anchor(time_ctx);
        const WindowAnchor exact_anchor = select_exact_anchor(time_ctx);

        out.visual_window_s = plan_horizon_s;
        out.pick_window_s = plan_horizon_s;
        out.exact_window_s = plan_horizon_s;
        out.visual_anchor_time_s = visual_anchor.time_s;
        out.pick_anchor_time_s = pick_anchor.time_s;
        out.exact_anchor_time_s = exact_anchor.time_s;
        out.visual_anchor_kind = visual_anchor.kind;
        out.pick_anchor_kind = pick_anchor.kind;
        out.exact_anchor_kind = exact_anchor.kind;
        out.visual_anchor_is_future = visual_anchor.is_future;
        out.pick_anchor_is_future = pick_anchor.is_future;
        out.exact_anchor_is_future = exact_anchor.is_future;

        if (input.live_preview_active &&
            std::isfinite(time_ctx.selected_node_time_s) &&
            std::isfinite(time_ctx.sim_now_s))
        {
            const double anchor_offset_s = std::max(0.0, time_ctx.selected_node_time_s - time_ctx.sim_now_s);
            const double preview_exact_window_s =
                    std::min(solve_margin_s,
                             std::max(OrbitPredictionTuning::kPreviewExactWindowMinS,
                                      std::max(0.0, input.plan_windows.preview_window_s)));
            if (preview_exact_window_s > 0.0)
            {
                out.request_window_s = std::max(out.request_window_s,
                                                anchor_offset_s + (2.0 * preview_exact_window_s));
            }
            const double anchored_visual_window_s = resolve_live_preview_visual_window_s(out.request_window_s,
                                                                                         anchor_offset_s,
                                                                                         input.plan_windows.preview_window_s);
            const double exact_window_s = std::max(0.0, input.plan_windows.solve_margin_s);

            out.visual_window_s = anchored_visual_window_s;
            out.pick_window_s = anchored_visual_window_s;
            out.exact_window_s = exact_window_s;
            out.visual_anchor_time_s = time_ctx.selected_node_time_s;
            out.pick_anchor_time_s = time_ctx.selected_node_time_s;
            out.exact_anchor_time_s = time_ctx.selected_node_time_s;
            out.visual_anchor_kind = PredictionTimeAnchorKind::SelectedNode;
            out.pick_anchor_kind = PredictionTimeAnchorKind::SelectedNode;
            out.exact_anchor_kind = PredictionTimeAnchorKind::SelectedNode;
            out.visual_anchor_is_future = true;
            out.pick_anchor_is_future = true;
            out.exact_anchor_is_future = true;
            out.valid =
                    out.visual_window_s > 0.0 &&
                    out.pick_window_s > 0.0 &&
                    out.exact_window_s > 0.0;
            return out;
        }

        if (authored_plan_window.valid)
        {
            out.visual_window_start_time_s = authored_plan_window.t0_s;
            out.visual_window_end_time_s = authored_plan_window.t1_s;
            out.pick_window_start_time_s = authored_plan_window.t0_s;
            out.pick_window_end_time_s = authored_plan_window.t1_s;
        }

        if (std::isfinite(visual_anchor.time_s))
        {
            if (!authored_plan_window.valid && std::isfinite(time_ctx.last_future_node_time_s))
            {
                const double authored_plan_end_s =
                        resolve_authored_plan_end_s(time_ctx,
                                                    std::isfinite(time_ctx.first_future_node_time_s)
                                                            ? time_ctx.first_future_node_time_s
                                                            : visual_anchor.time_s,
                                                    time_ctx.last_future_node_time_s,
                                                    plan_horizon_s,
                                                    post_node_coverage_s);
                const double authored_plan_span_s = std::max(0.0, authored_plan_end_s - visual_anchor.time_s);
                out.visual_window_s = std::max(out.visual_window_s, authored_plan_span_s);
            }

            if (input.draw_full_orbit)
            {
                double orbit_visual_span_s = 0.0;
                if (track &&
                    std::isfinite(track->cache.analysis.orbital_period_s) &&
                    track->cache.analysis.orbital_period_s > 0.0)
                {
                    orbit_visual_span_s =
                            track->cache.analysis.orbital_period_s * OrbitPredictionTuning::kFullOrbitDrawPeriodScale;
                }
                else if (std::isfinite(time_ctx.trajectory_t1_s) && time_ctx.trajectory_t1_s > visual_anchor.time_s)
                {
                    orbit_visual_span_s = time_ctx.trajectory_t1_s - visual_anchor.time_s;
                }

                out.visual_window_s = std::max(out.visual_window_s, orbit_visual_span_s);
            }
        }

        if (std::isfinite(pick_anchor.time_s) &&
            !authored_plan_window.valid &&
            std::isfinite(time_ctx.last_future_node_time_s))
        {
            const double authored_plan_end_s =
                    resolve_authored_plan_end_s(time_ctx,
                                                std::isfinite(time_ctx.first_future_node_time_s)
                                                        ? time_ctx.first_future_node_time_s
                                                        : pick_anchor.time_s,
                                                time_ctx.last_future_node_time_s,
                                                plan_horizon_s,
                                                post_node_coverage_s);
            const double authored_plan_span_s = std::max(0.0, authored_plan_end_s - pick_anchor.time_s);
            out.pick_window_s = std::max(out.pick_window_s, authored_plan_span_s);
        }

        out.valid =
                std::isfinite(out.visual_anchor_time_s) &&
                std::isfinite(out.pick_anchor_time_s) &&
                std::isfinite(out.exact_anchor_time_s) &&
                out.visual_window_s > 0.0 &&
                out.pick_window_s > 0.0 &&
                out.exact_window_s > 0.0;

        return out;
    }
} // namespace Game::PredictionWindowPolicy
