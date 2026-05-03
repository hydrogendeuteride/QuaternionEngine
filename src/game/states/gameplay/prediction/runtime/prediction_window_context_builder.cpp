#include "game/states/gameplay/prediction/runtime/prediction_window_context_builder.h"

#include "game/states/gameplay/maneuver/maneuver_system.h"
#include "game/states/gameplay/prediction/prediction_system.h"
#include "game/states/gameplay/prediction/runtime/gameplay_state_prediction_runtime_internal.h"
#include "game/states/gameplay/prediction/runtime/prediction_lifecycle_reducer.h"

#include "game/orbit/orbit_prediction_tuning.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace Game
{
    PredictionWindowContextBuilder::PredictionWindowContextBuilder(GameplayPredictionContext context)
        : _context(std::move(context))
    {
    }

    double PredictionWindowContextBuilder::future_window_s(const PredictionSubjectKey key) const
    {
        if (!_context.prediction)
        {
            return 0.0;
        }

        if (key.kind == PredictionSubjectKind::Celestial)
        {
            return std::max(0.0, _context.prediction->state().sampling_policy.celestial_min_window_s);
        }

        return std::max(0.0, _context.prediction->state().sampling_policy.orbiter_min_window_s);
    }

    double PredictionWindowContextBuilder::maneuver_plan_horizon_s() const
    {
        return std::max(OrbitPredictionTuning::kPostNodeCoverageMinS,
                        std::max(0.0, _context.maneuver.settings().plan_horizon.horizon_s));
    }

    uint64_t PredictionWindowContextBuilder::current_maneuver_plan_signature() const
    {
        return _context.maneuver.plan_signature(maneuver_plan_horizon_s());
    }

    const PredictionTrackState *PredictionWindowContextBuilder::find_track(const PredictionSubjectKey key) const
    {
        return _context.prediction ? _context.prediction->find_track(key) : nullptr;
    }

    double PredictionWindowContextBuilder::authored_plan_request_window_s(const double now_s) const
    {
        if (!_context.maneuver.settings().nodes_enabled ||
            _context.maneuver.plan().nodes.empty() ||
            !std::isfinite(now_s))
        {
            return 0.0;
        }

        double first_future_node_time_s = std::numeric_limits<double>::quiet_NaN();
        double last_future_node_time_s = std::numeric_limits<double>::quiet_NaN();
        for (const ManeuverNode &node : _context.maneuver.plan().nodes)
        {
            if (!std::isfinite(node.time_s) ||
                node.time_s + PredictionWindowPolicy::kTimeEpsilonS < now_s)
            {
                continue;
            }

            first_future_node_time_s = std::isfinite(first_future_node_time_s)
                                               ? std::min(first_future_node_time_s, node.time_s)
                                               : node.time_s;
            last_future_node_time_s = std::isfinite(last_future_node_time_s)
                                              ? std::max(last_future_node_time_s, node.time_s)
                                              : node.time_s;
        }

        if (!std::isfinite(first_future_node_time_s))
        {
            return 0.0;
        }

        const double request_end_s = PredictionWindowPolicy::resolve_authored_plan_end_s(
                PredictionTimeContext{.sim_now_s = now_s},
                first_future_node_time_s,
                std::isfinite(last_future_node_time_s) ? last_future_node_time_s : first_future_node_time_s,
                maneuver_plan_horizon_s(),
                OrbitPredictionTuning::kPostNodeCoverageMinS);
        if (!std::isfinite(request_end_s) ||
            request_end_s <= now_s + PredictionWindowPolicy::kTimeEpsilonS)
        {
            return 0.0;
        }

        return std::max(0.0, request_end_s - now_s);
    }

    PredictionTimeContext PredictionWindowContextBuilder::build_time_context(
            const PredictionSubjectKey key,
            const double sim_now_s,
            const double trajectory_t0_s,
            const double trajectory_t1_s) const
    {
        (void) key;

        PredictionTimeContext out{};
        out.sim_now_s = sim_now_s;
        out.trajectory_t0_s = trajectory_t0_s;
        out.trajectory_t1_s = trajectory_t1_s;

        if (const ManeuverNode *selected =
                    _context.maneuver.plan().find_node(_context.maneuver.plan().selected_node_id))
        {
            if (PredictionWindowPolicy::node_time_in_context_range(out, selected->time_s))
            {
                out.selected_node_time_s = selected->time_s;
            }
        }

        for (const ManeuverNode &node : _context.maneuver.plan().nodes)
        {
            if (!PredictionWindowPolicy::node_time_in_context_range(out, node.time_s))
            {
                continue;
            }

            out.has_plan = true;
            out.first_relevant_node_time_s = std::isfinite(out.first_relevant_node_time_s)
                                                     ? std::min(out.first_relevant_node_time_s, node.time_s)
                                                     : node.time_s;

            if (node.time_s + PredictionWindowPolicy::kTimeEpsilonS >= sim_now_s)
            {
                out.first_future_node_time_s = std::isfinite(out.first_future_node_time_s)
                                                       ? std::min(out.first_future_node_time_s, node.time_s)
                                                       : node.time_s;
                out.last_future_node_time_s = std::isfinite(out.last_future_node_time_s)
                                                      ? std::max(out.last_future_node_time_s, node.time_s)
                                                      : node.time_s;
            }
        }

        return out;
    }

    PredictionWindowPolicyResult PredictionWindowContextBuilder::resolve_policy(
            const PredictionTrackState *track,
            const PredictionTimeContext &time_ctx,
            const bool with_maneuvers) const
    {
        if (!_context.prediction)
        {
            return {};
        }

        const PredictionSubjectKey key = track ? track->key : PredictionSubjectKey{};
        return PredictionWindowPolicy::plan_windows(PredictionWindowPolicy::PlannerInput{
                .track = track,
                .time_ctx = time_ctx,
                .plan_windows = _context.maneuver.settings().plan_windows,
                .future_window_s = future_window_s(key),
                .plan_horizon_s = maneuver_plan_horizon_s(),
                .draw_future_segment = _context.prediction->state().draw_future_segment,
                .draw_full_orbit = _context.prediction->state().draw_full_orbit,
                .with_maneuvers = with_maneuvers,
                .live_preview_active = _context.maneuver.live_preview_active(with_maneuvers),
        });
    }

    double PredictionWindowContextBuilder::required_window_s(
            const PredictionTrackState &track,
            const double now_s,
            const bool with_maneuvers) const
    {
        const PredictionTimeContext time_ctx = build_time_context(track.key, now_s);
        PredictionWindowPolicyResult policy = resolve_policy(&track, time_ctx, with_maneuvers);
        if (with_maneuvers && track.supports_maneuvers)
        {
            policy.request_window_s = std::max(policy.request_window_s,
                                               authored_plan_request_window_s(now_s));
        }
        return policy.request_window_s;
    }

    double PredictionWindowContextBuilder::required_window_s(
            const PredictionSubjectKey key,
            const double now_s,
            const bool with_maneuvers) const
    {
        if (const PredictionTrackState *track = find_track(key))
        {
            return required_window_s(*track, now_s, with_maneuvers);
        }

        const PredictionTimeContext time_ctx = build_time_context(key, now_s);
        PredictionWindowPolicyResult policy = resolve_policy(nullptr, time_ctx, with_maneuvers);
        if (with_maneuvers)
        {
            policy.request_window_s = std::max(policy.request_window_s,
                                               authored_plan_request_window_s(now_s));
        }
        return policy.request_window_s;
    }

    double PredictionWindowContextBuilder::display_window_s(
            const PredictionSubjectKey key,
            const double now_s,
            const bool with_maneuvers) const
    {
        const PredictionTimeContext time_ctx = build_time_context(key, now_s);
        const PredictionWindowPolicyResult policy = resolve_policy(find_track(key), time_ctx, with_maneuvers);
        return policy.valid ? policy.visual_window_s : policy.request_window_s;
    }

    double PredictionWindowContextBuilder::preview_exact_window_s(
            const PredictionTrackState &track,
            const double /*now_s*/,
            const bool /*with_maneuvers*/) const
    {
        const double configured_exact_window_s =
                std::max(0.0, _context.maneuver.settings().plan_windows.solve_margin_s);
        double preview_exact_cap_s = std::max(OrbitPredictionTuning::kPreviewExactWindowMinS,
                                              _context.maneuver.settings().plan_windows.preview_window_s);
        if (track.preview_anchor.valid && track.preview_anchor.visual_window_s > 0.0)
        {
            preview_exact_cap_s =
                    std::max(OrbitPredictionTuning::kPreviewExactWindowMinS,
                             track.preview_anchor.visual_window_s);
        }

        return std::min(configured_exact_window_s, preview_exact_cap_s);
    }

    double PredictionWindowContextBuilder::planned_exact_window_s(
            const PredictionTrackState &track,
            const double now_s,
            const bool with_maneuvers) const
    {
        const PredictionTimeContext time_ctx = build_time_context(track.key, now_s);
        const PredictionWindowPolicyResult policy = resolve_policy(&track, time_ctx, with_maneuvers);
        return policy.exact_window_s;
    }

    void PredictionWindowContextBuilder::refresh_preview_anchor(
            PredictionTrackState &track,
            const double now_s,
            const bool with_maneuvers) const
    {
        const PredictionRuntimeDetail::PredictionTrackLifecycleSnapshot lifecycle =
                PredictionRuntimeDetail::describe_prediction_track_lifecycle(track);
        const bool preview_active =
                track.supports_maneuvers &&
                _context.maneuver.live_preview_active(with_maneuvers);
        const ManeuverNode *selected =
                _context.maneuver.plan().find_node(_context.maneuver.active_preview_anchor_node_id());

        if (preview_active && selected && std::isfinite(selected->time_s))
        {
            const double request_window_s = required_window_s(track.key, now_s, with_maneuvers);
            const double anchor_offset_s = std::max(0.0, selected->time_s - now_s);
            const double visual_window_s = PredictionWindowPolicy::resolve_live_preview_visual_window_s(
                    request_window_s,
                    anchor_offset_s,
                    _context.maneuver.settings().plan_windows.preview_window_s);
            const double exact_window_s = std::max(0.0, _context.maneuver.settings().plan_windows.solve_margin_s);

            track.preview_anchor.valid = true;
            track.preview_anchor.anchor_node_id = selected->id;
            track.preview_anchor.anchor_time_s = selected->time_s;
            track.preview_anchor.request_window_s = request_window_s;
            track.preview_anchor.visual_window_s = visual_window_s;
            track.preview_anchor.exact_window_s = exact_window_s;
            track.preview_last_anchor_refresh_at_s = now_s;
            if (PredictionRuntimeDetail::prediction_track_can_enter_preview_drag(lifecycle))
            {
                PredictionLifecycleReducer::enter_preview_drag(track, now_s);
            }
            return;
        }

        if (track.preview_anchor.valid &&
            PredictionRuntimeDetail::prediction_track_can_transition_to_await_full_refine(lifecycle))
        {
            PredictionLifecycleReducer::await_full_refine(track, now_s);
            return;
        }

        if (lifecycle.preview_state == PredictionPreviewRuntimeState::Idle)
        {
            track.preview_anchor = {};
        }
    }
} // namespace Game
