#include "game/states/gameplay/prediction/runtime/prediction_request_factory.h"

#include "game/orbit/trajectory/trajectory_utils.h"
#include "game/states/gameplay/prediction/prediction_trajectory_sampler.h"
#include "game/states/gameplay/prediction/runtime/gameplay_state_prediction_runtime_internal.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace Game
{
    namespace
    {
        constexpr double kPreviewAnchorTimeMatchEpsilonS = 1.0e-6;
        constexpr double kSuffixRefineAnchorTimeMatchEpsilonS = 1.0e-6;

        OrbitPredictionService::RequestPriority classify_prediction_subject_priority(
                const PredictionSelectionState &selection,
                const PredictionSubjectKey key,
                const bool is_celestial)
        {
            if (selection.active_subject == key)
            {
                return OrbitPredictionService::RequestPriority::ActiveTrack;
            }

            for (const PredictionSubjectKey overlay : selection.overlay_subjects)
            {
                if (overlay == key)
                {
                    return OrbitPredictionService::RequestPriority::Overlay;
                }
            }

            return is_celestial
                           ? OrbitPredictionService::RequestPriority::BackgroundCelestial
                           : OrbitPredictionService::RequestPriority::BackgroundOrbiter;
        }

        OrbitPredictionService::RequestPriority classify_prediction_request_priority(
                const PredictionSelectionState &selection,
                const PredictionSubjectKey key,
                const bool is_celestial,
                const bool interactive)
        {
            OrbitPredictionService::RequestPriority priority =
                    classify_prediction_subject_priority(selection, key, is_celestial);
            if (interactive && priority == OrbitPredictionService::RequestPriority::ActiveTrack)
            {
                priority = OrbitPredictionService::RequestPriority::ActiveInteractiveTrack;
            }
            return priority;
        }

        bool prediction_request_is_interactive(const PredictionSelectionState &selection,
                                               const PredictionTrackState &track,
                                               const bool live_preview_active,
                                               const bool thrusting)
        {
            return track.key == selection.active_subject && (thrusting || live_preview_active);
        }

        bool request_frame_is_lagrange_sensitive(const orbitsim::TrajectoryFrameSpec &spec)
        {
            return spec.type == orbitsim::TrajectoryFrameType::Synodic;
        }

        bool maneuver_node_matches_preview_anchor(const ManeuverNode &node,
                                                  const PredictionPreviewAnchor &anchor)
        {
            return anchor.valid &&
                   node.id == anchor.anchor_node_id &&
                   std::isfinite(node.time_s) &&
                   std::isfinite(anchor.anchor_time_s) &&
                   std::abs(node.time_s - anchor.anchor_time_s) <= kSuffixRefineAnchorTimeMatchEpsilonS;
        }

        std::vector<OrbitPredictionService::ManeuverNodePreview> collect_prefix_maneuver_previews(
                const PredictionSolverTrajectoryCache &solver,
                const double anchor_time_s)
        {
            std::vector<OrbitPredictionService::ManeuverNodePreview> out;
            out.reserve(solver.maneuver_previews.size());
            for (const OrbitPredictionService::ManeuverNodePreview &preview : solver.maneuver_previews)
            {
                if (preview.valid &&
                    std::isfinite(preview.t_s) &&
                    preview.t_s < (anchor_time_s - kSuffixRefineAnchorTimeMatchEpsilonS))
                {
                    out.push_back(preview);
                }
            }
            return out;
        }

        bool try_build_suffix_refine_prefix_from_cache(
                const PredictionCacheIdentity &identity,
                const PredictionSolverTrajectoryCache &solver,
                const double now_s,
                const double anchor_time_s,
                std::vector<orbitsim::TrajectorySegment> &out_prefix_segments,
                std::vector<OrbitPredictionService::ManeuverNodePreview> &out_prefix_previews)
        {
            out_prefix_segments.clear();
            out_prefix_previews.clear();

            if (!identity.valid ||
                !std::isfinite(now_s) ||
                !std::isfinite(anchor_time_s) ||
                anchor_time_s <= (now_s + kSuffixRefineAnchorTimeMatchEpsilonS) ||
                solver.trajectory_segments_inertial_planned.empty() ||
                !validate_trajectory_segment_continuity(solver.trajectory_segments_inertial_planned))
            {
                return false;
            }

            const double required_duration_s = anchor_time_s - now_s;
            if (!trajectory_segments_cover_window(solver.trajectory_segments_inertial_planned,
                                                  now_s,
                                                  required_duration_s))
            {
                return false;
            }

            std::size_t cursor = 0u;
            if (!PredictionTrajectorySampler::slice_segments_from_cursor(solver.trajectory_segments_inertial_planned,
                                                                         now_s,
                                                                         anchor_time_s,
                                                                         cursor,
                                                                         out_prefix_segments))
            {
                out_prefix_segments.clear();
                return false;
            }
            if (out_prefix_segments.empty() ||
                !validate_trajectory_segment_continuity(out_prefix_segments))
            {
                out_prefix_segments.clear();
                return false;
            }

            const double start_epsilon_s = continuity_time_epsilon_s(now_s);
            const double end_epsilon_s = continuity_time_epsilon_s(anchor_time_s);
            const double prefix_end_s = prediction_segment_end_time(out_prefix_segments.back());
            if (std::abs(out_prefix_segments.front().t0_s - now_s) > start_epsilon_s ||
                std::abs(prefix_end_s - anchor_time_s) > end_epsilon_s)
            {
                out_prefix_segments.clear();
                return false;
            }

            out_prefix_previews = collect_prefix_maneuver_previews(solver, anchor_time_s);
            return true;
        }

        bool try_build_suffix_refine_prefix(
                const PredictionTrackState &track,
                const double now_s,
                const double anchor_time_s,
                std::vector<orbitsim::TrajectorySegment> &out_prefix_segments,
                std::vector<OrbitPredictionService::ManeuverNodePreview> &out_prefix_previews)
        {
            if (track.authoritative_cache.identity.valid &&
                try_build_suffix_refine_prefix_from_cache(track.authoritative_cache.identity,
                                                          track.authoritative_cache.solver,
                                                          now_s,
                                                          anchor_time_s,
                                                          out_prefix_segments,
                                                          out_prefix_previews))
            {
                return true;
            }

            return try_build_suffix_refine_prefix_from_cache(track.cache.identity,
                                                            track.cache.solver,
                                                            now_s,
                                                            anchor_time_s,
                                                            out_prefix_segments,
                                                            out_prefix_previews);
        }

        bool current_plan_has_prior_future_maneuver(const std::vector<ManeuverNode> &nodes,
                                                    const int anchor_node_id,
                                                    const double anchor_time_s,
                                                    const double sim_now_s)
        {
            if (!std::isfinite(anchor_time_s) || !std::isfinite(sim_now_s))
            {
                return true;
            }

            return std::any_of(nodes.begin(),
                               nodes.end(),
                               [&](const ManeuverNode &node) {
                                   return node.id != anchor_node_id &&
                                          std::isfinite(node.time_s) &&
                                          node.time_s + kPreviewAnchorTimeMatchEpsilonS >= sim_now_s &&
                                          node.time_s < (anchor_time_s - kPreviewAnchorTimeMatchEpsilonS);
                               });
        }

        bool sample_unplanned_anchor_state(const PredictionSolverTrajectoryCache &solver,
                                           const double anchor_time_s,
                                           orbitsim::State &out_state)
        {
            return PredictionTrajectorySampler::sample_inertial_state(
                           solver.resolved_trajectory_segments_inertial(),
                           anchor_time_s,
                           out_state,
                           TrajectoryBoundarySide::Before) ||
                   PredictionTrajectorySampler::sample_inertial_state(solver.resolved_trajectory_inertial(),
                                                                      anchor_time_s,
                                                                      out_state);
        }
    } // namespace

    bool PredictionRequestFactory::resolve_preview_anchor_state(const PredictionRuntimeContext &context,
                                                                const PredictionTrackState &track,
                                                                orbitsim::State &out_state)
    {
        bool trusted = false;
        return resolve_preview_anchor_state(context, track, out_state, trusted);
    }

    bool PredictionRequestFactory::resolve_preview_anchor_state(const PredictionRuntimeContext &context,
                                                                const PredictionTrackState &track,
                                                                orbitsim::State &out_state,
                                                                bool &out_trusted)
    {
        out_state = {};
        out_trusted = false;
        if (!context.maneuver_plan ||
            !context.maneuver_edit_preview ||
            !track.preview_anchor.valid ||
            !std::isfinite(track.preview_anchor.anchor_time_s))
        {
            return false;
        }

        const OrbitPredictionCache &anchor_cache =
                track.authoritative_cache.identity.valid ? track.authoritative_cache : track.cache;
        const PredictionSolverTrajectoryCache &anchor_solver = anchor_cache.solver;

        const bool editing_anchor_time =
                context.maneuver_edit_preview->state == ManeuverNodeEditPreview::State::EditingTime &&
                context.maneuver_edit_preview->node_id == track.preview_anchor.anchor_node_id;
        const bool anchor_node_is_current =
                context.maneuver_plan->find_node(track.preview_anchor.anchor_node_id) != nullptr;
        const bool has_prior_future_maneuver =
                anchor_node_is_current &&
                current_plan_has_prior_future_maneuver(context.maneuver_plan->nodes,
                                                       track.preview_anchor.anchor_node_id,
                                                       track.preview_anchor.anchor_time_s,
                                                       context.current_sim_time_s);
        if (anchor_node_is_current && editing_anchor_time)
        {
            if (has_prior_future_maneuver)
            {
                return false;
            }

            out_trusted = sample_unplanned_anchor_state(anchor_solver, track.preview_anchor.anchor_time_s, out_state);
            return out_trusted;
        }

        if (anchor_node_is_current &&
            !has_prior_future_maneuver &&
            sample_unplanned_anchor_state(anchor_solver, track.preview_anchor.anchor_time_s, out_state))
        {
            out_trusted = true;
            return true;
        }

        const auto preview_it =
                std::find_if(anchor_solver.maneuver_previews.begin(),
                             anchor_solver.maneuver_previews.end(),
                             [&track](const OrbitPredictionService::ManeuverNodePreview &preview) {
                                 return preview.valid &&
                                        preview.node_id == track.preview_anchor.anchor_node_id &&
                                        std::isfinite(preview.t_s) &&
                                        std::abs(preview.t_s - track.preview_anchor.anchor_time_s) <=
                                                kPreviewAnchorTimeMatchEpsilonS &&
                                        detail::finite_vec3(preview.inertial_position_m) &&
                                        detail::finite_vec3(preview.inertial_velocity_mps);
                             });
        if (preview_it != anchor_solver.maneuver_previews.end())
        {
            out_state = orbitsim::make_state(preview_it->inertial_position_m,
                                             preview_it->inertial_velocity_mps);
            out_trusted = true;
            return true;
        }

        const bool resolved =
                PredictionTrajectorySampler::sample_inertial_state(anchor_solver.trajectory_segments_inertial_planned,
                                                                   track.preview_anchor.anchor_time_s,
                                                                   out_state,
                                                                   TrajectoryBoundarySide::Before) ||
                PredictionTrajectorySampler::sample_inertial_state(anchor_solver.trajectory_inertial_planned,
                                                                   track.preview_anchor.anchor_time_s,
                                                                   out_state) ||
                PredictionTrajectorySampler::sample_inertial_state(track.cache.solver.resolved_trajectory_segments_inertial(),
                                                                   track.preview_anchor.anchor_time_s,
                                                                   out_state,
                                                                   TrajectoryBoundarySide::Before) ||
                PredictionTrajectorySampler::sample_inertial_state(track.cache.solver.resolved_trajectory_inertial(),
                                                                   track.preview_anchor.anchor_time_s,
                                                                   out_state);
        out_trusted = false;
        return resolved;
    }

    PredictionOrbiterRequestBuildResult PredictionRequestFactory::build_orbiter_request(
            const PredictionRuntimeContext &context,
            PredictionTrackState &track,
            const WorldVec3 &subject_pos_world,
            const glm::dvec3 &subject_vel_world,
            const double now_s,
            const bool thrusting,
            const bool with_maneuvers)
    {
        PredictionOrbiterRequestBuildResult out{};
        if (!context.orbital_scenario ||
            !context.reference_body_world ||
            !context.required_window_s ||
            !context.preview_exact_window_s ||
            !context.refresh_preview_anchor ||
            !context.subject_is_player ||
            !context.resolve_maneuver_node_primary_body_id)
        {
            return out;
        }

        const orbitsim::MassiveBody *ref_sim = context.orbital_scenario->world_reference_sim_body();
        if (!ref_sim || ref_sim->id == orbitsim::kInvalidBodyId)
        {
            return out;
        }

        context.refresh_preview_anchor(track, now_s, with_maneuvers);

        const WorldVec3 reference_body_world = context.reference_body_world();
        const glm::dvec3 ship_rel_pos_m = glm::dvec3(subject_pos_world - reference_body_world);
        const glm::dvec3 ship_rel_vel_mps = subject_vel_world;
        const glm::dvec3 ship_bary_pos_m = ref_sim->state.position_m + ship_rel_pos_m;
        const glm::dvec3 ship_bary_vel_mps = ref_sim->state.velocity_mps + ship_rel_vel_mps;
        if (!detail::finite_vec3(ship_bary_pos_m) || !detail::finite_vec3(ship_bary_vel_mps))
        {
            return out;
        }

        const bool live_preview_active = with_maneuvers && context.maneuver_live_preview_available;
        const bool interactive_request =
                prediction_request_is_interactive(context.selection, track, live_preview_active, thrusting);
        const bool preview_request_active =
                track.preview_anchor.valid &&
                track.supports_maneuvers &&
                with_maneuvers &&
                live_preview_active;
        const OrbitPredictionService::SolveQuality solve_quality =
                preview_request_active
                        ? OrbitPredictionService::SolveQuality::FastPreview
                        : OrbitPredictionService::SolveQuality::Full;
        const double preview_exact_window_s =
                preview_request_active
                        ? context.preview_exact_window_s(track, now_s, with_maneuvers)
                        : 0.0;

        OrbitPredictionService::Request request{};
        request.track_id = track.key.track_id();
        request.maneuver_plan_revision = track.supports_maneuvers ? context.maneuver_plan_revision : 0u;
        request.sim_time_s = now_s;
        request.sim_config = context.orbital_scenario->sim.config();
        request.shared_ephemeris = track.cache.resolved_shared_ephemeris();
        request.massive_bodies = context.orbital_scenario->sim.massive_bodies();
        request.ship_bary_position_m = ship_bary_pos_m;
        request.ship_bary_velocity_mps = ship_bary_vel_mps;
        request.thrusting = thrusting;
        request.solve_quality = solve_quality;
        request.priority = classify_prediction_request_priority(
                context.selection,
                track.key,
                track.is_celestial,
                interactive_request);

        if (preview_request_active)
        {
            const double anchor_offset_s = std::max(0.0, track.preview_anchor.anchor_time_s - now_s);
            request.future_window_s = std::min(track.preview_anchor.request_window_s,
                                                anchor_offset_s + (2.0 * preview_exact_window_s));
        }
        else
        {
            request.future_window_s = context.required_window_s(track, now_s, with_maneuvers);
        }

        orbitsim::State preview_anchor_state{};
        bool preview_anchor_state_trusted = false;
        const bool preview_anchor_state_valid =
                resolve_preview_anchor_state(context, track, preview_anchor_state, preview_anchor_state_trusted);
        if (preview_request_active)
        {
            request.preview_patch.active = true;
            request.preview_patch.anchor_time_s = track.preview_anchor.anchor_time_s;
            request.preview_patch.visual_window_s = track.preview_anchor.visual_window_s;
            request.preview_patch.exact_window_s = preview_exact_window_s;
            if (preview_anchor_state_valid)
            {
                request.preview_patch.anchor_state_valid = true;
                request.preview_patch.anchor_state_trusted = preview_anchor_state_trusted;
                request.preview_patch.anchor_state_inertial = preview_anchor_state;
            }
        }

        const orbitsim::TrajectoryFrameSpec display_frame_spec =
                track.cache.display.resolved_frame_spec_valid
                        ? track.cache.display.resolved_frame_spec
                        : context.frame_selection.spec;
        request.lagrange_sensitive = request_frame_is_lagrange_sensitive(display_frame_spec);
        const bool subject_is_player = context.subject_is_player(track.key);
        const bool auto_primary_may_shift_across_plan = with_maneuvers && subject_is_player;
        request.preferred_primary_body_id =
                auto_primary_may_shift_across_plan ? orbitsim::kInvalidBodyId : track.auto_primary_body_id;
        if (request.preferred_primary_body_id == orbitsim::kInvalidBodyId &&
            context.analysis_selection.spec.mode == PredictionAnalysisMode::FixedBodyBCI &&
            context.analysis_selection.spec.fixed_body_id != orbitsim::kInvalidBodyId)
        {
            request.preferred_primary_body_id = context.analysis_selection.spec.fixed_body_id;
        }
        if (request.preferred_primary_body_id == orbitsim::kInvalidBodyId &&
            display_frame_spec.primary_body_id != orbitsim::kInvalidBodyId)
        {
            request.preferred_primary_body_id = display_frame_spec.primary_body_id;
        }

        if (with_maneuvers && context.maneuver_plan)
        {
            const double maneuver_window_s =
                    preview_request_active
                            ? track.preview_anchor.request_window_s
                            : request.future_window_s;
            const double request_end_s =
                    request.sim_time_s +
                    std::max(0.0, std::isfinite(maneuver_window_s) ? maneuver_window_s : 0.0);
            constexpr double kManeuverRequestTimeEpsilonS = 1.0e-6;
            request.maneuver_impulses.reserve(context.maneuver_plan->nodes.size());
            for (const ManeuverNode &node : context.maneuver_plan->nodes)
            {
                if (!std::isfinite(node.time_s))
                {
                    continue;
                }

                if (node.time_s < (request.sim_time_s - kManeuverRequestTimeEpsilonS) ||
                    node.time_s > (request_end_s + kManeuverRequestTimeEpsilonS))
                {
                    continue;
                }

                OrbitPredictionService::ManeuverImpulse impulse{};
                impulse.node_id = node.id;
                impulse.t_s = node.time_s;
                impulse.primary_body_id =
                        node.primary_body_auto
                                ? orbitsim::kInvalidBodyId
                                : context.resolve_maneuver_node_primary_body_id(node, node.time_s);
                impulse.dv_rtn_mps = orbitsim::Vec3{node.dv_rtn_mps.x, node.dv_rtn_mps.y, node.dv_rtn_mps.z};
                request.maneuver_impulses.push_back(impulse);
            }
        }

        if (!request.maneuver_impulses.empty())
        {
            request.maneuver_plan_signature_valid = true;
            request.maneuver_plan_signature = context.maneuver_plan_signature;
        }

        const bool post_preview_full_refine =
                track.preview_state == PredictionPreviewRuntimeState::AwaitFullRefine &&
                track.preview_anchor.valid;
        const ManeuverNode *selected_node =
                context.maneuver_plan ? context.maneuver_plan->find_node(context.maneuver_plan->selected_node_id) : nullptr;
        if (solve_quality == OrbitPredictionService::SolveQuality::Full &&
            post_preview_full_refine &&
            track.supports_maneuvers &&
            with_maneuvers &&
            selected_node &&
            maneuver_node_matches_preview_anchor(*selected_node, track.preview_anchor) &&
            !request.maneuver_impulses.empty())
        {
            std::vector<orbitsim::TrajectorySegment> prefix_segments;
            std::vector<OrbitPredictionService::ManeuverNodePreview> prefix_previews;
            if (preview_anchor_state_valid &&
                finite_state(preview_anchor_state) &&
                try_build_suffix_refine_prefix(track,
                                               now_s,
                                               track.preview_anchor.anchor_time_s,
                                               prefix_segments,
                                               prefix_previews) &&
                states_are_continuous(prefix_segments.back().end, preview_anchor_state))
            {
                request.planned_suffix_refine.active = true;
                request.planned_suffix_refine.anchor_node_id = track.preview_anchor.anchor_node_id;
                request.planned_suffix_refine.anchor_time_s = track.preview_anchor.anchor_time_s;
                request.planned_suffix_refine.anchor_state_inertial = preview_anchor_state;
                request.planned_suffix_refine.prefix_segments_inertial = std::move(prefix_segments);
                request.planned_suffix_refine.prefix_previews = std::move(prefix_previews);
            }
        }
        request.full_stream_publish.active =
                (interactive_request || post_preview_full_refine) &&
                solve_quality == OrbitPredictionService::SolveQuality::Full &&
                track.key == context.selection.active_subject &&
                subject_is_player &&
                !request.maneuver_impulses.empty();

        out.built = true;
        out.interactive_request = interactive_request;
        out.preview_request_active = preview_request_active;
        out.request = std::move(request);
        return out;
    }

    bool PredictionRequestFactory::build_celestial_request(const PredictionRuntimeContext &context,
                                                           const PredictionTrackState &track,
                                                           const double now_s,
                                                           OrbitPredictionService::Request &out_request)
    {
        out_request = {};
        if (!context.orbital_scenario || !context.future_window_s)
        {
            return false;
        }

        const orbitsim::MassiveBody *body =
                context.orbital_scenario->sim.body_by_id(static_cast<orbitsim::BodyId>(track.key.value));
        if (!body)
        {
            return false;
        }

        OrbitPredictionService::Request request{};
        request.kind = OrbitPredictionService::RequestKind::Celestial;
        request.track_id = track.key.track_id();
        request.sim_time_s = now_s;
        request.sim_config = context.orbital_scenario->sim.config();
        request.massive_bodies = context.orbital_scenario->sim.massive_bodies();
        request.shared_ephemeris = track.cache.resolved_shared_ephemeris();
        request.subject_body_id = body->id;
        request.priority = classify_prediction_request_priority(
                context.selection,
                track.key,
                true,
                false);
        request.future_window_s = context.future_window_s(track.key);
        request.lagrange_sensitive = request_frame_is_lagrange_sensitive(context.frame_selection.spec);
        if (context.analysis_selection.spec.mode == PredictionAnalysisMode::FixedBodyBCI &&
            context.analysis_selection.spec.fixed_body_id != orbitsim::kInvalidBodyId)
        {
            request.preferred_primary_body_id = context.analysis_selection.spec.fixed_body_id;
        }
        if (request.preferred_primary_body_id == orbitsim::kInvalidBodyId &&
            context.frame_selection.spec.primary_body_id != orbitsim::kInvalidBodyId)
        {
            request.preferred_primary_body_id = context.frame_selection.spec.primary_body_id;
        }

        out_request = std::move(request);
        return true;
    }
} // namespace Game
