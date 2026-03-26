#include "game/states/gameplay/prediction/gameplay_state_prediction_draw_internal.h"
#include "game/states/gameplay/prediction/gameplay_prediction_cache_internal.h"

#include "game/orbit/orbit_prediction_tuning.h"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace Game
{
    namespace Draw = PredictionDrawDetail;

    namespace
    {
        constexpr double kPickWindowRebuildEpsilonS = 0.25;
        constexpr double kPickAlignRebuildDistanceM = 25.0;
        constexpr double kPickReferenceRebuildDistanceM = 1.0;
        constexpr double kPickFrustumOriginRebuildDistanceM = 1.0;
        constexpr double kPickCameraRebuildDistanceM = 1.0;
        constexpr double kPickMatrixRebuildEpsilon = 1.0e-6;
        constexpr double kPickScalarRebuildEpsilon = 1.0e-6;
        constexpr float kPickViewprojRebuildEpsilon = 1.0e-5f;
        constexpr uint32_t kMaxPlannedChunkGpuRootBuildsPerFrame = 2u;

        struct ChunkAssemblyDrawResult
        {
            uint32_t chunks_drawn{0};
            uint32_t chunks_built{0};
            double gpu_root_build_ms{0.0};
            std::vector<std::pair<double, double>> covered_ranges{};
        };

        bool same_matrix(const glm::dmat3 &a, const glm::dmat3 &b, const double epsilon)
        {
            for (int col = 0; col < 3; ++col)
            {
                for (int row = 0; row < 3; ++row)
                {
                    if (!std::isfinite(a[col][row]) || !std::isfinite(b[col][row]) ||
                        std::abs(a[col][row] - b[col][row]) > epsilon)
                    {
                        return false;
                    }
                }
            }
            return true;
        }

        bool same_matrix(const glm::mat4 &a, const glm::mat4 &b, const float epsilon)
        {
            for (int col = 0; col < 4; ++col)
            {
                for (int row = 0; row < 4; ++row)
                {
                    if (!std::isfinite(a[col][row]) || !std::isfinite(b[col][row]) ||
                        std::abs(a[col][row] - b[col][row]) > epsilon)
                    {
                        return false;
                    }
                }
            }
            return true;
        }

        void enqueue_cached_orbit_window(OrbitPlotSystem *orbit_plot,
                                         const std::shared_ptr<const std::vector<OrbitPlotSystem::GpuRootSegment>> &cached_roots,
                                         const WorldVec3 &ref_body_world,
                                         const glm::dmat3 &frame_to_world,
                                         const WorldVec3 &align_delta,
                                         const double t_start_s,
                                         const double t_end_s,
                                         const glm::vec4 &color,
                                         const bool dashed,
                                         const float line_overlay_boost)
        {
            if (!orbit_plot || !cached_roots || cached_roots->empty() || !(t_end_s > t_start_s))
            {
                return;
            }

            orbit_plot->add_gpu_root_batch(cached_roots,
                                           t_start_s,
                                           t_end_s,
                                           ref_body_world,
                                           align_delta,
                                           frame_to_world,
                                           color,
                                           dashed,
                                           OrbitPlotDepth::DepthTested);

            if (line_overlay_boost <= 0.0f)
            {
                return;
            }

            glm::vec4 overlay_color = color;
            overlay_color.a = std::clamp(overlay_color.a * line_overlay_boost, 0.0f, 1.0f);
            if (overlay_color.a <= 0.0f)
            {
                return;
            }

            orbit_plot->add_gpu_root_batch(cached_roots,
                                           t_start_s,
                                           t_end_s,
                                           ref_body_world,
                                           align_delta,
                                           frame_to_world,
                                           overlay_color,
                                           dashed,
                                           OrbitPlotDepth::AlwaysOnTop);
        }

        // Draw planned path directly from chunk assembly so preview does not
        // depend on a flattened planned cache. GPU roots are used when available;
        // otherwise CPU adaptive-curve draw consumes each chunk render curve.
        ChunkAssemblyDrawResult draw_chunk_assembly_planned(
                const Draw::OrbitDrawWindowContext &draw_ctx,
                const OrbitPredictionDrawConfig &draw_config,
                OrbitPlotPerfStats &perf,
                PredictionChunkAssembly &assembly,
                const double t_start_s,
                const double t_end_s,
                const glm::vec4 &color,
                const bool dashed,
                const bool use_persistent_gpu_roots,
                const uint32_t max_chunk_builds)
        {
            ChunkAssemblyDrawResult result{};
            if (!assembly.valid || assembly.chunks.empty() || !(t_end_s > t_start_s))
            {
                return result;
            }

            for (OrbitChunk &chunk : assembly.chunks)
            {
                if (!chunk.valid || chunk.frame_segments.empty())
                {
                    continue;
                }

                // Skip chunks entirely outside the visible window.
                if (chunk.t1_s <= t_start_s || chunk.t0_s >= t_end_s)
                {
                    continue;
                }

                // Clamp chunk time bounds to the draw window.
                const double draw_t0 = std::max(chunk.t0_s, t_start_s);
                const double draw_t1 = std::min(chunk.t1_s, t_end_s);
                if (!(draw_t1 > draw_t0))
                {
                    continue;
                }

                bool drawn = false;
                if (use_persistent_gpu_roots && draw_ctx.orbit_plot)
                {
                    // Ensure gpu_roots exist (lazy build from frame segments).
                    if (!chunk.gpu_roots || chunk.gpu_roots->empty())
                    {
                        if (result.chunks_built >= max_chunk_builds)
                        {
                            continue;
                        }
                        const auto build_start_tp = std::chrono::steady_clock::now();
                        chunk.gpu_roots = PredictionCacheInternal::build_gpu_root_cache(chunk.frame_segments);
                        result.gpu_root_build_ms +=
                                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - build_start_tp)
                                        .count();
                        ++result.chunks_built;
                    }

                    if (chunk.gpu_roots && !chunk.gpu_roots->empty())
                    {
                        enqueue_cached_orbit_window(draw_ctx.orbit_plot,
                                                    chunk.gpu_roots,
                                                    draw_ctx.ref_body_world,
                                                    draw_ctx.frame_to_world,
                                                    draw_ctx.align_delta,
                                                    draw_t0,
                                                    draw_t1,
                                                    color,
                                                    dashed,
                                                    draw_ctx.line_overlay_boost);
                        drawn = true;
                    }
                }

                if (!drawn)
                {
                    if (!chunk.render_curve.empty())
                    {
                        Draw::draw_adaptive_curve_window(draw_ctx,
                                                         draw_config,
                                                         perf,
                                                         chunk.render_curve,
                                                         draw_t0,
                                                         draw_t1,
                                                         color,
                                                         dashed);
                    }
                    else
                    {
                        Draw::draw_orbit_window(draw_ctx,
                                                draw_config,
                                                perf,
                                                chunk.frame_segments,
                                                draw_t0,
                                                draw_t1,
                                                color,
                                                dashed);
                    }
                }

                ++result.chunks_drawn;
                result.covered_ranges.emplace_back(draw_t0, draw_t1);
            }

            return result;
        }

        std::vector<std::pair<double, double>> compute_uncovered_ranges(
                const double t_start_s,
                const double t_end_s,
                std::vector<std::pair<double, double>> covered_ranges)
        {
            std::vector<std::pair<double, double>> uncovered_ranges;
            if (!(t_end_s > t_start_s))
            {
                return uncovered_ranges;
            }

            if (covered_ranges.empty())
            {
                uncovered_ranges.emplace_back(t_start_s, t_end_s);
                return uncovered_ranges;
            }

            std::sort(covered_ranges.begin(),
                      covered_ranges.end(),
                      [](const auto &a, const auto &b) {
                          if (a.first == b.first)
                          {
                              return a.second < b.second;
                          }
                          return a.first < b.first;
                      });

            constexpr double kTimeEpsilonS = 1.0e-6;
            double cursor_t_s = t_start_s;
            for (const auto &[covered_t0_s, covered_t1_s] : covered_ranges)
            {
                if (!(covered_t1_s > covered_t0_s))
                {
                    continue;
                }

                const double clamped_t0_s = std::max(covered_t0_s, t_start_s);
                const double clamped_t1_s = std::min(covered_t1_s, t_end_s);
                if (!(clamped_t1_s > clamped_t0_s))
                {
                    continue;
                }

                if (clamped_t0_s > (cursor_t_s + kTimeEpsilonS))
                {
                    uncovered_ranges.emplace_back(cursor_t_s, clamped_t0_s);
                }
                cursor_t_s = std::max(cursor_t_s, clamped_t1_s);
            }

            if (t_end_s > (cursor_t_s + kTimeEpsilonS))
            {
                uncovered_ranges.emplace_back(cursor_t_s, t_end_s);
            }

            return uncovered_ranges;
        }

        bool should_rebuild_pick_cache(const PredictionLinePickCache &cache,
                                       const uint64_t generation_id,
                                       const WorldVec3 &ref_body_world,
                                       const glm::dmat3 &frame_to_world,
                                       const WorldVec3 &align_delta,
                                       const glm::dvec3 &camera_world,
                                       const double tan_half_fov,
                                       const double viewport_height_px,
                                       const double render_error_px,
                                       const OrbitRenderCurve::FrustumContext &pick_frustum,
                                       const double pick_frustum_margin_ratio,
                                       const double t0_s,
                                       const double t1_s,
                                       const std::size_t max_segments,
                                       const bool use_adaptive_curve,
                                       const bool planned)
        {
            const bool valid = planned ? cache.planned_valid : cache.base_valid;
            const double cached_t0_s = planned ? cache.planned_t0_s : cache.base_t0_s;
            const double cached_t1_s = planned ? cache.planned_t1_s : cache.base_t1_s;
            const std::size_t cached_max_segments = planned ? cache.planned_max_segments : cache.base_max_segments;
            const bool cached_use_adaptive_curve =
                    planned ? cache.planned_use_adaptive_curve : cache.base_use_adaptive_curve;
            if (!valid || cache.generation_id != generation_id || cached_max_segments != max_segments)
            {
                return true;
            }

            if (cached_use_adaptive_curve != use_adaptive_curve)
            {
                return true;
            }

            if (!same_matrix(cache.frame_to_world, frame_to_world, kPickMatrixRebuildEpsilon))
            {
                return true;
            }

            if (cache.pick_frustum_valid != pick_frustum.valid ||
                !same_matrix(cache.pick_frustum_viewproj, pick_frustum.viewproj, kPickViewprojRebuildEpsilon) ||
                glm::length(glm::dvec3(cache.pick_frustum_origin_world - pick_frustum.origin_world)) >
                        kPickFrustumOriginRebuildDistanceM ||
                std::abs(cache.pick_frustum_margin_ratio - pick_frustum_margin_ratio) > kPickMatrixRebuildEpsilon)
            {
                return true;
            }

            if (glm::length(glm::dvec3(cache.ref_body_world - ref_body_world)) > kPickReferenceRebuildDistanceM ||
                glm::length(glm::dvec3(cache.align_delta_world - align_delta)) > kPickAlignRebuildDistanceM)
            {
                return true;
            }

            if (use_adaptive_curve &&
                (glm::length(cache.camera_world - camera_world) > kPickCameraRebuildDistanceM ||
                 std::abs(cache.tan_half_fov - tan_half_fov) > kPickScalarRebuildEpsilon ||
                 std::abs(cache.viewport_height_px - viewport_height_px) > kPickScalarRebuildEpsilon ||
                 std::abs(cache.render_error_px - render_error_px) > kPickScalarRebuildEpsilon))
            {
                return true;
            }

            return !std::isfinite(cached_t0_s) || !std::isfinite(cached_t1_s) ||
                   std::abs(cached_t0_s - t0_s) > kPickWindowRebuildEpsilonS ||
                   std::abs(cached_t1_s - t1_s) > kPickWindowRebuildEpsilonS;
        }

        void mark_pick_cache_valid(PredictionLinePickCache &cache,
                                   const uint64_t generation_id,
                                   const WorldVec3 &ref_body_world,
                                   const glm::dmat3 &frame_to_world,
                                   const WorldVec3 &align_delta,
                                   const glm::dvec3 &camera_world,
                                   const double tan_half_fov,
                                   const double viewport_height_px,
                                   const double render_error_px,
                                   const OrbitRenderCurve::FrustumContext &pick_frustum,
                                   const double pick_frustum_margin_ratio,
                                   const double t0_s,
                                   const double t1_s,
                                   const std::size_t max_segments,
                                   const bool use_adaptive_curve,
                                   const bool planned)
        {
            cache.generation_id = generation_id;
            cache.ref_body_world = ref_body_world;
            cache.frame_to_world = frame_to_world;
            cache.align_delta_world = align_delta;
            cache.camera_world = camera_world;
            cache.tan_half_fov = tan_half_fov;
            cache.viewport_height_px = viewport_height_px;
            cache.render_error_px = render_error_px;
            cache.pick_frustum_valid = pick_frustum.valid;
            cache.pick_frustum_viewproj = pick_frustum.viewproj;
            cache.pick_frustum_origin_world = pick_frustum.origin_world;
            cache.pick_frustum_margin_ratio = pick_frustum_margin_ratio;
            if (planned)
            {
                cache.planned_valid = true;
                cache.planned_t0_s = t0_s;
                cache.planned_t1_s = t1_s;
                cache.planned_max_segments = max_segments;
                cache.planned_use_adaptive_curve = use_adaptive_curve;
            }
            else
            {
                cache.base_valid = true;
                cache.base_t0_s = t0_s;
                cache.base_t1_s = t1_s;
                cache.base_max_segments = max_segments;
                cache.base_use_adaptive_curve = use_adaptive_curve;
            }
        }

    } // namespace

    // Build per-frame orbit visuals, pick data, and debug overlays from the latest prediction cache.
    void GameplayState::emit_orbit_prediction_debug(GameStateContext &ctx)
    {
        PickingSystem *picking = (ctx.renderer != nullptr) ? ctx.renderer->picking() : nullptr;
        OrbitPlotSystem *orbit_plot =
                (ctx.renderer && ctx.renderer->_context) ? ctx.renderer->_context->orbit_plot : nullptr;
        Draw::reset_orbit_plot_state(picking, orbit_plot, _orbit_plot_perf, _prediction_enabled);

        if (!_prediction_enabled || !ctx.api || !_orbitsim)
        {
            return;
        }

        const PredictionTrackState *active_track = active_prediction_track();
        if (!active_track)
        {
            return;
        }

        const float alpha_f = std::clamp(ctx.interpolation_alpha(), 0.0f, 1.0f);
        const double display_time_s =
                Draw::compute_prediction_display_time_s(_orbitsim->sim.time_s(),
                                                        _last_sim_step_dt_s,
                                                        ctx.fixed_delta_time(),
                                                        alpha_f);
        if (!std::isfinite(display_time_s))
        {
            return;
        }

        // Debug velocity ray is emitted into DebugDrawSystem, which prunes commands in
        // engine draw begin_frame(dt) after update_scene(), so ttl must be > dt.
        const float ttl_s = std::clamp(ctx.delta_time(), 0.0f, 0.1f) + 0.002f;

        const float line_alpha_scale = std::clamp(_prediction_line_alpha_scale, 0.1f, 8.0f);
        const glm::vec4 color_orbit_plan =
                Draw::scale_line_color(_prediction_draw_config.palette.orbit_planned, line_alpha_scale);

        glm::dvec3 camera_world = ctx.api->get_camera_position_d();
        float viewport_h_px = 720.0f;
        float camera_fov_deg = 70.0f;
        if (ctx.renderer && ctx.renderer->_sceneManager)
        {
            const Camera &cam = ctx.renderer->_sceneManager->getMainCamera();
            camera_world = cam.position_world;
            camera_fov_deg = cam.fovDegrees;
            if (ctx.renderer->_logicalRenderExtent.height > 0)
            {
                viewport_h_px = static_cast<float>(ctx.renderer->_logicalRenderExtent.height);
            }
        }

        const double tan_half_fov = std::tan(glm::radians(static_cast<double>(camera_fov_deg)) * 0.5);
        OrbitRenderCurve::CameraContext lod_camera{};
        lod_camera.camera_world = camera_world;
        lod_camera.tan_half_fov = tan_half_fov;
        lod_camera.viewport_height_px = std::max(1.0, static_cast<double>(viewport_h_px));

        OrbitRenderCurve::FrustumContext render_frustum{};
        if (ctx.renderer && ctx.renderer->_sceneManager)
        {
            render_frustum.valid = true;
            render_frustum.viewproj = ctx.renderer->_sceneManager->getSceneData().viewproj;
            render_frustum.origin_world = WorldVec3(camera_world);
        }

        const double render_error_px =
                (std::isfinite(_orbit_plot_budget.render_error_px) && _orbit_plot_budget.render_error_px > 0.0)
                        ? _orbit_plot_budget.render_error_px
                        : 0.75;
        if (orbit_plot)
        {
            orbit_plot->settings().render_error_px = render_error_px;
        }

        std::vector<PredictionTrackState *> visible_tracks;
        visible_tracks.reserve(1 + _prediction_selection.overlay_subjects.size());
        for (PredictionSubjectKey key : collect_visible_prediction_subjects())
        {
            if (PredictionTrackState *track = find_prediction_track(key))
            {
                visible_tracks.push_back(track);
            }
        }
        if (visible_tracks.empty())
        {
            return;
        }

        for (PredictionTrackState *track : visible_tracks)
        {
            if (!track)
            {
                continue;
            }

            refresh_prediction_derived_cache(*track, display_time_s);
            if (!track->cache.valid || track->cache.trajectory_frame.size() < 2)
            {
                continue;
            }

            OrbitPredictionCache *const stable_cache = &track->cache;
            OrbitPredictionCache *const preview_planned_cache =
                    track->preview_overlay.cache.valid ? &track->preview_overlay.cache : nullptr;
            OrbitPredictionCache *const planned_cache = preview_planned_cache ? preview_planned_cache : stable_cache;
            PredictionChunkAssembly *const planned_chunk_assembly =
                    track->preview_overlay.chunk_assembly.valid ? &track->preview_overlay.chunk_assembly : nullptr;
            const bool has_preview_planned_overlay = track->preview_overlay.valid();
            const bool has_chunk_planned_overlay =
                    planned_chunk_assembly &&
                    planned_chunk_assembly->valid &&
                    !planned_chunk_assembly->chunks.empty();

            const auto &traj_base = stable_cache->trajectory_frame;
            const auto &traj_planned = planned_cache->trajectory_frame_planned;
            WorldVec3 ref_body_world{0.0, 0.0, 0.0};
            glm::dmat3 frame_to_world(1.0);
            OrbitPredictionCache *const display_cache =
                    planned_cache->resolved_frame_spec_valid ? planned_cache : stable_cache;
            if (!build_prediction_display_transform(*display_cache, ref_body_world, frame_to_world, display_time_s))
            {
                ref_body_world = prediction_frame_origin_world(*display_cache, display_time_s);
                frame_to_world = glm::dmat3(1.0);
            }

            const double t0 = traj_base.front().t_s;
            const double t1 = traj_base.back().t_s;
            if (!(t1 > t0))
            {
                continue;
            }

            const double now_s = Draw::compute_prediction_now_s(display_time_s, t0, t1);
            if (!std::isfinite(now_s))
            {
                continue;
            }

            WorldVec3 subject_pos_world_state{0.0, 0.0, 0.0};
            glm::dvec3 subject_vel_world(0.0);
            glm::vec3 subject_vel_local_f(0.0f);
            if (!get_prediction_subject_world_state(track->key,
                                                    subject_pos_world_state,
                                                    subject_vel_world,
                                                    subject_vel_local_f))
            {
                continue;
            }

            WorldVec3 subject_pos_world = subject_pos_world_state;
            if (track->key.kind == PredictionSubjectKind::Orbiter)
            {
                if (const Entity *entity = _world.entities().find(EntityId{track->key.value}))
                {
                    subject_pos_world = entity->get_render_position_world(alpha_f);
                }
            }

            const std::vector<orbitsim::TrajectorySegment> *traj_base_segments = &stable_cache->trajectory_segments_frame;
            if (traj_base_segments->empty())
            {
                continue;
            }

            const std::vector<orbitsim::TrajectorySegment> *traj_planned_segments =
                    &planned_cache->trajectory_segments_frame_planned;
            const std::vector<orbitsim::TrajectorySegment> *planned_window_segments =
                    !stable_cache->trajectory_segments_frame_planned.empty()
                            ? &stable_cache->trajectory_segments_frame_planned
                            : (!planned_cache->trajectory_segments_frame_planned.empty()
                                       ? &planned_cache->trajectory_segments_frame_planned
                                       : traj_planned_segments);

            const bool is_active = track->key == _prediction_selection.active_subject;
            const bool active_player_track = is_active && prediction_subject_is_player(track->key);
            const bool maneuver_drag_active =
                    active_player_track &&
                    _maneuver_gizmo_interaction.state == ManeuverGizmoInteraction::State::DragAxis;
            const bool suppress_stale_planned_preview =
                    maneuver_drag_active &&
                    (track->preview_state == PredictionPreviewRuntimeState::EnterDrag ||
                     track->preview_state == PredictionPreviewRuntimeState::DragPreviewPending);
            const bool drag_anchor_valid =
                    maneuver_drag_active &&
                    track->preview_anchor.valid &&
                    std::isfinite(track->preview_anchor.anchor_time_s);
            const double drag_anchor_time_s =
                    drag_anchor_valid ? track->preview_anchor.anchor_time_s
                                      : std::numeric_limits<double>::quiet_NaN();
            if (is_active)
            {
                _orbit_plot_perf.solver_segments_base = static_cast<uint32_t>(traj_base_segments->size());
                _orbit_plot_perf.solver_segments_planned = static_cast<uint32_t>(traj_planned_segments->size());
            }

            const std::size_t i_hi = Draw::lower_bound_sample_index(traj_base, now_s);
            if (i_hi >= traj_base.size())
            {
                continue;
            }

            const WorldVec3 align_delta =
                    Draw::compute_align_delta(traj_base, i_hi, subject_pos_world, now_s, ref_body_world, frame_to_world);
            const bool direct_world_polyline =
                    Draw::frame_spec_uses_direct_world_polyline(
                            display_cache->resolved_frame_spec_valid ? display_cache->resolved_frame_spec
                                                                     : _prediction_frame_selection.spec);

            Draw::OrbitDrawWindowContext draw_ctx{};
            draw_ctx.orbit_plot = orbit_plot;
            draw_ctx.ref_body_world = ref_body_world;
            draw_ctx.frame_to_world = frame_to_world;
            draw_ctx.align_delta = align_delta;
            draw_ctx.lod_camera = lod_camera;
            draw_ctx.render_frustum = render_frustum;
            draw_ctx.camera_world = camera_world;
            draw_ctx.tan_half_fov = tan_half_fov;
            draw_ctx.viewport_height_px = std::max(1.0, static_cast<double>(viewport_h_px));
            draw_ctx.render_error_px = render_error_px;
            draw_ctx.render_max_segments =
                    static_cast<std::size_t>(std::max(1, _orbit_plot_budget.render_max_segments_cpu));
            draw_ctx.line_overlay_boost = std::clamp(_prediction_line_overlay_boost, 0.0f, 1.0f);
            const bool identity_frame_transform = Draw::frame_transform_is_identity(frame_to_world);
            const bool use_persistent_gpu_roots =
                    orbit_plot && orbit_plot->settings().gpu_generate_enabled &&
                    !direct_world_polyline;
            const bool use_base_adaptive_curve = !stable_cache->render_curve_frame.empty();

            Draw::OrbitDrawWindowContext world_basis_draw_ctx = draw_ctx;
            if (!identity_frame_transform)
            {
                world_basis_draw_ctx.frame_to_world = glm::dmat3(1.0);
            }

            std::vector<orbitsim::TrajectorySegment> traj_base_segments_world_basis{};
            std::vector<orbitsim::TrajectorySegment> traj_stable_planned_segments_world_basis{};
            std::vector<orbitsim::TrajectorySegment> traj_preview_planned_segments_world_basis{};
            const auto get_base_segments_world_basis = [&]() -> const std::vector<orbitsim::TrajectorySegment> & {
                if (identity_frame_transform)
                {
                    return *traj_base_segments;
                }
                if (traj_base_segments_world_basis.empty() && !traj_base_segments->empty())
                {
                    traj_base_segments_world_basis =
                            Draw::transform_segments_to_world_basis(*traj_base_segments, frame_to_world);
                }
                return traj_base_segments_world_basis;
            };
            const auto get_planned_segments_world_basis =
                    [&](OrbitPredictionCache &cache) -> const std::vector<orbitsim::TrajectorySegment> & {
                if (identity_frame_transform)
                {
                    return cache.trajectory_segments_frame_planned;
                }
                std::vector<orbitsim::TrajectorySegment> &world_basis_segments =
                        (&cache == preview_planned_cache)
                                ? traj_preview_planned_segments_world_basis
                                : traj_stable_planned_segments_world_basis;
                if (world_basis_segments.empty() && !cache.trajectory_segments_frame_planned.empty())
                {
                    world_basis_segments =
                            Draw::transform_segments_to_world_basis(cache.trajectory_segments_frame_planned,
                                                                    frame_to_world);
                }
                return world_basis_segments;
            };

            const auto draw_raw_base_window = [&](const double split_t0_s,
                                                  const double split_t1_s,
                                                  const glm::vec4 &color) {
                if (!(split_t1_s > split_t0_s))
                {
                    return;
                }

                Draw::draw_orbit_window(identity_frame_transform ? draw_ctx : world_basis_draw_ctx,
                                        _prediction_draw_config,
                                        _orbit_plot_perf,
                                        identity_frame_transform ? *traj_base_segments
                                                                 : get_base_segments_world_basis(),
                                        split_t0_s,
                                        split_t1_s,
                                        color,
                                        false);
            };

            const auto draw_cpu_base_window = [&](const double window_t0_s,
                                                  const double window_t1_s,
                                                  const glm::vec4 &color) {
                if (!(window_t1_s > window_t0_s))
                {
                    return;
                }

                const auto draw_cpu_base_window_once = [&](const double split_t0_s,
                                                           const double split_t1_s) {
                    if (!(split_t1_s > split_t0_s))
                    {
                        return;
                    }

                    if (!use_base_adaptive_curve)
                    {
                        draw_raw_base_window(split_t0_s, split_t1_s, color);
                        return;
                    }

                    std::size_t anchor_hi = i_hi;
                    if (anchor_hi >= traj_base.size())
                    {
                        anchor_hi = traj_base.size() - 1;
                    }

                    std::size_t anchor_lo = (anchor_hi > 0) ? (anchor_hi - 1) : 0;
                    if (anchor_hi == anchor_lo && (anchor_hi + 1) < traj_base.size())
                    {
                        anchor_hi = anchor_lo + 1;
                    }

                    const double anchor_source_t0_s = traj_base[anchor_lo].t_s;
                    const double anchor_source_t1_s = traj_base[anchor_hi].t_s;
                    const double anchor_t0_s = std::max(split_t0_s, anchor_source_t0_s);
                    const double anchor_t1_s = std::min(split_t1_s, anchor_source_t1_s);
                    const bool anchor_window_valid =
                            is_active &&
                            std::isfinite(anchor_source_t0_s) &&
                            std::isfinite(anchor_source_t1_s) &&
                            (anchor_t1_s > anchor_t0_s);

                    if (!anchor_window_valid)
                    {
                        Draw::draw_adaptive_curve_window(draw_ctx,
                                                         _prediction_draw_config,
                                                         _orbit_plot_perf,
                                                         stable_cache->render_curve_frame,
                                                         split_t0_s,
                                                         split_t1_s,
                                                         color,
                                                         false);
                        return;
                    }

                    if (anchor_t0_s > split_t0_s)
                    {
                        Draw::draw_adaptive_curve_window(draw_ctx,
                                                         _prediction_draw_config,
                                                         _orbit_plot_perf,
                                                         stable_cache->render_curve_frame,
                                                         split_t0_s,
                                                         anchor_t0_s,
                                                         color,
                                                         false);
                    }

                    // Keep the active subject pinned to the exact source segment around "now"
                    // so merged adaptive nodes cannot drift away from the rendered craft.
                    draw_raw_base_window(anchor_t0_s, anchor_t1_s, color);

                    if (split_t1_s > anchor_t1_s)
                    {
                        Draw::draw_adaptive_curve_window(draw_ctx,
                                                         _prediction_draw_config,
                                                         _orbit_plot_perf,
                                                         stable_cache->render_curve_frame,
                                                         anchor_t1_s,
                                                         split_t1_s,
                                                         color,
                                                         false);
                    }
                };

                // Keep the current displayed ship position on an exact segment boundary so
                // CPU LOD changes do not make the orbit line appear to drift around the craft.
                if (now_s > window_t0_s && now_s < window_t1_s)
                {
                    draw_cpu_base_window_once(window_t0_s, now_s);
                    draw_cpu_base_window_once(now_s, window_t1_s);
                    return;
                }

                draw_cpu_base_window_once(window_t0_s, window_t1_s);
            };

            const auto draw_planned_window_from_cache = [&](OrbitPredictionCache &cache,
                                                            const double window_t0_s,
                                                            const double window_t1_s,
                                                            const glm::vec4 &color) {
                if (!(window_t1_s > window_t0_s))
                {
                    return;
                }

                if (direct_world_polyline)
                {
                    Draw::draw_polyline_window(draw_ctx,
                                               _prediction_draw_config,
                                               cache.trajectory_frame_planned,
                                               window_t0_s,
                                               window_t1_s,
                                               color,
                                               _prediction_draw_config.draw_planned_as_dashed);
                    return;
                }

                if (use_persistent_gpu_roots && cache.gpu_roots_frame_planned && !cache.gpu_roots_frame_planned->empty())
                {
                    enqueue_cached_orbit_window(orbit_plot,
                                                cache.gpu_roots_frame_planned,
                                                ref_body_world,
                                                frame_to_world,
                                                align_delta,
                                                window_t0_s,
                                                window_t1_s,
                                                color,
                                                _prediction_draw_config.draw_planned_as_dashed,
                                                draw_ctx.line_overlay_boost);
                    return;
                }

                if (use_persistent_gpu_roots)
                {
                    const auto &gpu_roots_planned = PredictionCacheInternal::ensure_gpu_root_cache(
                            cache.gpu_roots_frame_planned,
                            cache.trajectory_segments_frame_planned);
                    if (gpu_roots_planned && !gpu_roots_planned->empty())
                    {
                        enqueue_cached_orbit_window(orbit_plot,
                                                    gpu_roots_planned,
                                                    ref_body_world,
                                                    frame_to_world,
                                                    align_delta,
                                                    window_t0_s,
                                                    window_t1_s,
                                                    color,
                                                    _prediction_draw_config.draw_planned_as_dashed,
                                                    draw_ctx.line_overlay_boost);
                        return;
                    }
                }

                if (!cache.render_curve_frame_planned.empty())
                {
                    Draw::draw_adaptive_curve_window(draw_ctx,
                                                     _prediction_draw_config,
                                                     _orbit_plot_perf,
                                                     cache.render_curve_frame_planned,
                                                     window_t0_s,
                                                     window_t1_s,
                                                     color,
                                                     _prediction_draw_config.draw_planned_as_dashed);
                    return;
                }

                Draw::draw_orbit_window(identity_frame_transform ? draw_ctx : world_basis_draw_ctx,
                                        _prediction_draw_config,
                                        _orbit_plot_perf,
                                        identity_frame_transform ? cache.trajectory_segments_frame_planned
                                                                 : get_planned_segments_world_basis(cache),
                                        window_t0_s,
                                        window_t1_s,
                                        color,
                                        _prediction_draw_config.draw_planned_as_dashed);
            };

            const auto collect_planned_cache_covered_ranges =
                    [&](const OrbitPredictionCache &cache,
                        const double window_t0_s,
                        const double window_t1_s) {
                std::vector<std::pair<double, double>> covered_ranges;
                if (!(window_t1_s > window_t0_s) || cache.trajectory_segments_frame_planned.empty())
                {
                    return covered_ranges;
                }

                const double cache_t0_s = cache.trajectory_segments_frame_planned.front().t0_s;
                const orbitsim::TrajectorySegment &last_segment = cache.trajectory_segments_frame_planned.back();
                const double cache_t1_s = last_segment.t0_s + last_segment.dt_s;
                if (!std::isfinite(cache_t0_s) || !std::isfinite(cache_t1_s) || !(cache_t1_s > cache_t0_s))
                {
                    return covered_ranges;
                }

                const double covered_t0_s = std::max(window_t0_s, cache_t0_s);
                const double covered_t1_s = std::min(window_t1_s, cache_t1_s);
                if (covered_t1_s > covered_t0_s)
                {
                    covered_ranges.emplace_back(covered_t0_s, covered_t1_s);
                }
                return covered_ranges;
            };

            const auto build_planned_pick_window_from_chunk_assembly =
                    [&](const PredictionChunkAssembly &assembly,
                        const double now_s_local,
                        const double future_window_s_local) {
                Draw::PickWindow planned_window{};
                if (!assembly.valid || assembly.chunks.empty() || _maneuver_state.nodes.empty())
                {
                    return planned_window;
                }

                const double t0p = assembly.start_time_s();
                const double t1p = assembly.end_time_s();
                if (!std::isfinite(t0p) || !std::isfinite(t1p) || !(t1p > t0p))
                {
                    return planned_window;
                }

                double first_future_node_time_s = std::numeric_limits<double>::infinity();
                double first_relevant_node_time_s = std::numeric_limits<double>::infinity();
                for (const ManeuverNode &node : _maneuver_state.nodes)
                {
                    if (!std::isfinite(node.time_s) ||
                        node.time_s < (t0p - _prediction_draw_config.node_time_tolerance_s) ||
                        node.time_s > (t1p + _prediction_draw_config.node_time_tolerance_s))
                    {
                        continue;
                    }

                    first_relevant_node_time_s = std::min(first_relevant_node_time_s, node.time_s);
                    if (node.time_s >= (now_s_local + _prediction_draw_config.node_time_tolerance_s))
                    {
                        first_future_node_time_s = std::min(first_future_node_time_s, node.time_s);
                    }
                }

                double anchor_time_s = first_future_node_time_s;
                if (!std::isfinite(anchor_time_s))
                {
                    anchor_time_s = first_relevant_node_time_s;
                }
                if (!std::isfinite(anchor_time_s))
                {
                    return planned_window;
                }

                double t_plan_start = t0p;
                if (std::isfinite(first_future_node_time_s))
                {
                    t_plan_start = std::clamp(anchor_time_s, t0p, t1p);
                }
                else
                {
                    t_plan_start = std::clamp(now_s_local, t0p, t1p);
                }

                double t_plan_end = t_plan_start;
                if (_prediction_draw_future_segment && future_window_s_local > 0.0)
                {
                    t_plan_end = std::min(t_plan_start + future_window_s_local, t1p);
                }
                else if (_prediction_draw_full_orbit)
                {
                    double t_full_end = t1p;
                    if (stable_cache->orbital_period_s > 0.0 && std::isfinite(stable_cache->orbital_period_s))
                    {
                        t_full_end = std::min(
                                t_plan_start + (stable_cache->orbital_period_s * OrbitPredictionTuning::kFullOrbitDrawPeriodScale),
                                t1p);
                    }
                    t_plan_end = t_full_end;
                }

                if (!(t_plan_end > t_plan_start))
                {
                    return planned_window;
                }

                planned_window.valid = true;
                planned_window.t0_s = t_plan_start;
                planned_window.t1_s = t_plan_end;
                planned_window.anchor_time_s = anchor_time_s;
                return planned_window;
            };

            const glm::vec3 track_rgb = prediction_subject_orbit_rgb(track->key);
            glm::vec4 track_color_full = Draw::scale_line_color(glm::vec4(track_rgb, 0.22f), line_alpha_scale);
            glm::vec4 track_color_future = Draw::scale_line_color(glm::vec4(track_rgb, 0.80f), line_alpha_scale);
            glm::vec4 track_color_plan = color_orbit_plan;
            if (!is_active)
            {
                draw_ctx.line_overlay_boost = std::clamp(_prediction_line_overlay_boost * 0.35f, 0.0f, 1.0f);
            }
            world_basis_draw_ctx.line_overlay_boost = draw_ctx.line_overlay_boost;

            const double future_window_s = prediction_future_window_s(track->key);
            const double planned_future_window_s = maneuver_plan_preview_window_s();
            const Draw::PickWindow planned_pick_window =
                    (active_player_track && _maneuver_nodes_enabled && !_maneuver_state.nodes.empty())
                            ? (!planned_window_segments->empty()
                                       ? Draw::build_planned_pick_window(*planned_window_segments,
                                                                         _prediction_draw_config,
                                                                         _maneuver_state.nodes,
                                                                         now_s,
                                                                         planned_future_window_s,
                                                                         _prediction_draw_future_segment,
                                                                         _prediction_draw_full_orbit,
                                                                         stable_cache->orbital_period_s)
                                       : ((has_preview_planned_overlay && planned_chunk_assembly && planned_chunk_assembly->valid)
                                                  ? build_planned_pick_window_from_chunk_assembly(*planned_chunk_assembly,
                                                                                                 now_s,
                                                                                                 planned_future_window_s)
                                                  : Draw::PickWindow{}))
                            : Draw::PickWindow{};
            Draw::PickWindow base_pick_window{};
            if (_prediction_draw_full_orbit)
            {
                double t_full_end = t1;
                if (stable_cache->orbital_period_s > 0.0 && std::isfinite(stable_cache->orbital_period_s))
                {
                    t_full_end = std::min(t0 + (stable_cache->orbital_period_s * OrbitPredictionTuning::kFullOrbitDrawPeriodScale), t1);
                }

                if (direct_world_polyline)
                {
                    Draw::draw_polyline_window(draw_ctx,
                                               _prediction_draw_config,
                                               traj_base,
                                               t0,
                                               t_full_end,
                                               track_color_full,
                                               false);
                }
                else if (use_persistent_gpu_roots && stable_cache->gpu_roots_frame && !stable_cache->gpu_roots_frame->empty())
                {
                    enqueue_cached_orbit_window(orbit_plot,
                                                stable_cache->gpu_roots_frame,
                                                ref_body_world,
                                                frame_to_world,
                                                align_delta,
                                                t0,
                                                t_full_end,
                                                track_color_full,
                                                false,
                                                draw_ctx.line_overlay_boost);
                }
                else if (use_persistent_gpu_roots)
                {
                    const auto &gpu_roots = PredictionCacheInternal::ensure_gpu_root_cache(
                            stable_cache->gpu_roots_frame,
                            stable_cache->trajectory_segments_frame);
                    if (gpu_roots && !gpu_roots->empty())
                    {
                        enqueue_cached_orbit_window(orbit_plot,
                                                    gpu_roots,
                                                    ref_body_world,
                                                    frame_to_world,
                                                    align_delta,
                                                    t0,
                                                    t_full_end,
                                                    track_color_full,
                                                    false,
                                                    draw_ctx.line_overlay_boost);
                    }
                    else
                    {
                        draw_cpu_base_window(t0, t_full_end, track_color_full);
                    }
                }
                else
                {
                    draw_cpu_base_window(t0, t_full_end, track_color_full);
                }
                if (is_active && !_prediction_draw_future_segment && t_full_end > t0)
                {
                    base_pick_window.valid = true;
                    base_pick_window.t0_s = t0;
                    base_pick_window.t1_s = t_full_end;
                }
            }

            if (_prediction_draw_future_segment)
            {
                const double t_end = (future_window_s > 0.0) ? std::min(now_s + future_window_s, t1) : t1;

                if (direct_world_polyline)
                {
                    Draw::draw_polyline_window(draw_ctx,
                                               _prediction_draw_config,
                                               traj_base,
                                               now_s,
                                               t_end,
                                               track_color_future,
                                               false);
                }
                else if (use_persistent_gpu_roots && stable_cache->gpu_roots_frame && !stable_cache->gpu_roots_frame->empty())
                {
                    enqueue_cached_orbit_window(orbit_plot,
                                                stable_cache->gpu_roots_frame,
                                                ref_body_world,
                                                frame_to_world,
                                                align_delta,
                                                now_s,
                                                t_end,
                                                track_color_future,
                                                false,
                                                draw_ctx.line_overlay_boost);
                }
                else if (use_persistent_gpu_roots)
                {
                    const auto &gpu_roots = PredictionCacheInternal::ensure_gpu_root_cache(
                            stable_cache->gpu_roots_frame,
                            stable_cache->trajectory_segments_frame);
                    if (gpu_roots && !gpu_roots->empty())
                    {
                        enqueue_cached_orbit_window(orbit_plot,
                                                    gpu_roots,
                                                    ref_body_world,
                                                    frame_to_world,
                                                    align_delta,
                                                    now_s,
                                                    t_end,
                                                    track_color_future,
                                                    false,
                                                    draw_ctx.line_overlay_boost);
                    }
                    else
                    {
                        draw_cpu_base_window(now_s, t_end, track_color_future);
                    }
                }
                else
                {
                    draw_cpu_base_window(now_s, t_end, track_color_future);
                }
                if (is_active && t_end > now_s)
                {
                    base_pick_window.valid = true;
                    base_pick_window.t0_s = now_s;
                    base_pick_window.t1_s = t_end;
                }
            }

            if (active_player_track)
            {
                _orbit_plot_perf.planned_window_valid = planned_pick_window.valid;
                _orbit_plot_perf.planned_window_now_s = now_s;
                _orbit_plot_perf.planned_window_anchor_s = planned_pick_window.anchor_time_s;
                _orbit_plot_perf.planned_window_t_start = planned_pick_window.t0_s;
                _orbit_plot_perf.planned_window_t_end = planned_pick_window.t1_s;
                if (!traj_planned_segments->empty())
                {
                    _orbit_plot_perf.planned_window_t0p = traj_planned_segments->front().t0_s;
                }
            }
            const auto compute_drag_prefix_fallback_ranges =
                    [&](const double window_t0_s,
                        const double window_t1_s,
                        std::vector<std::pair<double, double>> covered_ranges = {}) {
                std::vector<std::pair<double, double>> fallback_ranges;
                if (!drag_anchor_valid || !(window_t1_s > window_t0_s))
                {
                    return fallback_ranges;
                }

                const double prefix_t1_s = std::min(window_t1_s, drag_anchor_time_s);
                if (!(prefix_t1_s > window_t0_s))
                {
                    return fallback_ranges;
                }

                return compute_uncovered_ranges(window_t0_s, prefix_t1_s, std::move(covered_ranges));
            };
            if (planned_pick_window.valid)
            {
                if (suppress_stale_planned_preview)
                {
                    if (active_player_track)
                    {
                        _orbit_plot_perf.planned_chunk_count = 0;
                    }

                    OrbitPredictionCache *const stable_planned_prefix_cache =
                            (!stable_cache->trajectory_segments_frame_planned.empty() ||
                             !stable_cache->trajectory_frame_planned.empty())
                                    ? stable_cache
                                    : nullptr;
                    const auto fallback_ranges = (stable_planned_prefix_cache != nullptr)
                                                         ? compute_drag_prefix_fallback_ranges(planned_pick_window.t0_s,
                                                                                              planned_pick_window.t1_s)
                                                         : std::vector<std::pair<double, double>>{};
                    if (active_player_track)
                    {
                        _orbit_plot_perf.planned_fallback_range_count =
                                static_cast<uint32_t>(fallback_ranges.size());
                    }
                    if (!fallback_ranges.empty())
                    {
                        const auto fallback_draw_start_tp = std::chrono::steady_clock::now();
                        for (const auto &[fallback_t0_s, fallback_t1_s] : fallback_ranges)
                        {
                            draw_planned_window_from_cache(*stable_planned_prefix_cache,
                                                           fallback_t0_s,
                                                           fallback_t1_s,
                                                           track_color_plan);
                        }
                        if (active_player_track)
                        {
                            _orbit_plot_perf.planned_fallback_draw_ms_last =
                                    std::chrono::duration<double, std::milli>(
                                            std::chrono::steady_clock::now() - fallback_draw_start_tp)
                                            .count();
                        }
                    }
                }
                else
                {
                    if (active_player_track)
                    {
                        _orbit_plot_perf.planned_chunk_count =
                                has_chunk_planned_overlay
                                        ? static_cast<uint32_t>(planned_chunk_assembly->chunks.size())
                                        : 0;
                    }

                    OrbitPredictionCache *const fallback_planned_cache =
                            (!stable_cache->trajectory_segments_frame_planned.empty() ||
                             !stable_cache->trajectory_frame_planned.empty())
                                    ? stable_cache
                                    : planned_cache;

                if (has_chunk_planned_overlay)
                {
                    // Chunk assembly path: draw preview chunks directly without
                    // rebuilding a flat planned cache first.
                    const auto chunk_draw_start_tp = std::chrono::steady_clock::now();
                    const ChunkAssemblyDrawResult chunk_draw_result = draw_chunk_assembly_planned(
                            draw_ctx,
                            _prediction_draw_config,
                            _orbit_plot_perf,
                            *planned_chunk_assembly,
                            planned_pick_window.t0_s,
                            planned_pick_window.t1_s,
                            track_color_plan,
                            _prediction_draw_config.draw_planned_as_dashed,
                            use_persistent_gpu_roots,
                            kMaxPlannedChunkGpuRootBuildsPerFrame);
                    const double chunk_draw_ms =
                            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - chunk_draw_start_tp)
                                    .count();

                    if (active_player_track)
                    {
                        _orbit_plot_perf.planned_chunks_drawn = chunk_draw_result.chunks_drawn;
                        _orbit_plot_perf.planned_chunk_builds = chunk_draw_result.chunks_built;
                        _orbit_plot_perf.planned_chunk_enqueue_ms_last = chunk_draw_ms;
                        _orbit_plot_perf.planned_chunk_gpu_build_ms_last = chunk_draw_result.gpu_root_build_ms;
                    }

                    const auto fallback_ranges =
                            maneuver_drag_active
                                    ? compute_drag_prefix_fallback_ranges(planned_pick_window.t0_s,
                                                                          planned_pick_window.t1_s,
                                                                          chunk_draw_result.covered_ranges)
                                    : compute_uncovered_ranges(planned_pick_window.t0_s,
                                                               planned_pick_window.t1_s,
                                                               chunk_draw_result.covered_ranges);
                    if (active_player_track)
                    {
                        _orbit_plot_perf.planned_fallback_range_count =
                                static_cast<uint32_t>(fallback_ranges.size());
                    }
                    if (!fallback_ranges.empty())
                    {
                        const auto fallback_draw_start_tp = std::chrono::steady_clock::now();
                        for (const auto &[fallback_t0_s, fallback_t1_s] : fallback_ranges)
                        {
                            draw_planned_window_from_cache(*fallback_planned_cache,
                                                           fallback_t0_s,
                                                           fallback_t1_s,
                                                           track_color_plan);
                        }
                        if (active_player_track)
                        {
                            _orbit_plot_perf.planned_fallback_draw_ms_last =
                                    std::chrono::duration<double, std::milli>(
                                            std::chrono::steady_clock::now() - fallback_draw_start_tp)
                                            .count();
                        }
                    }
                }
                else if (!has_preview_planned_overlay)
                {
                    draw_planned_window_from_cache(*planned_cache,
                                                   planned_pick_window.t0_s,
                                                   planned_pick_window.t1_s,
                                                   track_color_plan);
                }
                else
                {
                    draw_planned_window_from_cache(*planned_cache,
                                                   planned_pick_window.t0_s,
                                                   planned_pick_window.t1_s,
                                                   track_color_plan);

                    const auto covered_ranges = collect_planned_cache_covered_ranges(*planned_cache,
                                                                                     planned_pick_window.t0_s,
                                                                                     planned_pick_window.t1_s);
                    const auto fallback_ranges = (fallback_planned_cache != planned_cache)
                                                         ? (maneuver_drag_active
                                                                    ? compute_drag_prefix_fallback_ranges(
                                                                              planned_pick_window.t0_s,
                                                                              planned_pick_window.t1_s,
                                                                              covered_ranges)
                                                                    : compute_uncovered_ranges(
                                                                              planned_pick_window.t0_s,
                                                                              planned_pick_window.t1_s,
                                                                              covered_ranges))
                                                         : std::vector<std::pair<double, double>>{};
                    if (active_player_track)
                    {
                        _orbit_plot_perf.planned_fallback_range_count =
                                static_cast<uint32_t>(fallback_ranges.size());
                    }

                    if (!fallback_ranges.empty())
                    {
                        const auto fallback_draw_start_tp = std::chrono::steady_clock::now();
                        for (const auto &[fallback_t0_s, fallback_t1_s] : fallback_ranges)
                        {
                            draw_planned_window_from_cache(*fallback_planned_cache,
                                                           fallback_t0_s,
                                                           fallback_t1_s,
                                                           track_color_plan);
                        }
                        if (active_player_track)
                        {
                            _orbit_plot_perf.planned_fallback_draw_ms_last =
                                    std::chrono::duration<double, std::milli>(
                                            std::chrono::steady_clock::now() - fallback_draw_start_tp)
                                            .count();
                        }
                    }
                }
                }
            }

            if (picking && active_player_track)
            {
                const bool allow_base_pick = _maneuver_state.nodes.empty();
                const bool allow_planned_pick = !_maneuver_state.nodes.empty();
                const uint32_t pick_group_base = picking->add_line_pick_group("OrbitPlot/Base");
                const uint32_t pick_group_planned = picking->add_line_pick_group("OrbitPlot/Planned");

                const std::size_t pick_max_segments =
                        static_cast<std::size_t>(std::max(1, _orbit_plot_budget.pick_max_segments));
                const double pick_frustum_margin_ratio = std::max(0.0, _orbit_plot_budget.pick_frustum_margin_ratio);
                OrbitRenderCurve::PickSettings pick_settings{};
                pick_settings.frustum_margin_ratio = pick_frustum_margin_ratio;
                std::vector<double> pick_anchor_times = Draw::collect_maneuver_node_times(_maneuver_state.nodes);
                pick_anchor_times.insert(pick_anchor_times.begin(), now_s);
                if (planned_pick_window.valid && std::isfinite(planned_pick_window.anchor_time_s))
                {
                    pick_anchor_times.push_back(planned_pick_window.anchor_time_s);
                }
                std::sort(pick_anchor_times.begin(), pick_anchor_times.end());
                pick_anchor_times.erase(std::unique(pick_anchor_times.begin(), pick_anchor_times.end()),
                                        pick_anchor_times.end());
                const std::span<const double> pick_anchor_times_span(pick_anchor_times);
                const auto make_pick_selection_context = [&]() {
                    OrbitRenderCurve::SelectionContext selection_ctx{};
                    selection_ctx.reference_body_world = ref_body_world;
                    selection_ctx.align_delta_world = align_delta;
                    selection_ctx.frame_to_world = frame_to_world;
                    selection_ctx.camera_world = camera_world;
                    selection_ctx.tan_half_fov = tan_half_fov;
                    selection_ctx.viewport_height_px = draw_ctx.viewport_height_px;
                    selection_ctx.error_px = render_error_px;
                    selection_ctx.anchor_times_s = pick_anchor_times_span;
                    return selection_ctx;
                };
                const std::size_t pick_planned_reserve_target = std::min(
                        _prediction_draw_config.pick_planned_reserve_segments,
                        static_cast<std::size_t>(std::max<std::size_t>(
                                1,
                                static_cast<std::size_t>(
                                        std::llround(static_cast<double>(pick_max_segments) *
                                                     _prediction_draw_config.pick_planned_reserve_ratio)))));
                std::size_t remaining_pick_budget = pick_max_segments;
                const auto build_pick_curve_cache = [&](const OrbitRenderCurve &curve,
                                                        const double t_start_s,
                                                        const double t_end_s,
                                                        std::vector<PickingSystem::LinePickSegmentData> &out_segments,
                                                        bool &out_cap_hit) {
                    const auto pick_start_tp = std::chrono::steady_clock::now();
                    const OrbitRenderCurve::PickResult lod = OrbitRenderCurve::build_pick_lod(
                            curve, make_pick_selection_context(), render_frustum, pick_settings, t_start_s, t_end_s);
                    _orbit_plot_perf.pick_lod_ms_last +=
                            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - pick_start_tp)
                                    .count();
                    _orbit_plot_perf.pick_segments_before_cull += static_cast<uint32_t>(lod.segments_before_cull);
                    _orbit_plot_perf.pick_segments += static_cast<uint32_t>(lod.segments_after_cull);

                    out_segments.clear();
                    out_segments.reserve(lod.segments.size());
                    for (const OrbitRenderCurve::LineSegment &segment : lod.segments)
                    {
                        out_segments.push_back(PickingSystem::LinePickSegmentData{
                                .a_world = segment.a_world,
                                .b_world = segment.b_world,
                                .a_time_s = segment.t0_s,
                                .b_time_s = segment.t1_s,
                        });
                    }

                    out_cap_hit = lod.cap_hit;
                    _orbit_plot_perf.pick_cap_hit_last_frame = out_cap_hit;
                    if (out_cap_hit)
                    {
                        ++_orbit_plot_perf.pick_cap_hits_total;
                    }
                    return out_segments.size();
                };

                const auto append_pick_segments_from_chunk_assembly =
                        [&](PredictionChunkAssembly &assembly,
                            const double t_start_s,
                            const double t_end_s,
                            const std::size_t max_segments,
                            std::vector<PickingSystem::LinePickSegmentData> &io_segments,
                            std::vector<std::pair<double, double>> &out_covered_ranges) {
                    std::size_t emitted_total = io_segments.size();
                    bool any_cap_hit = false;
                    for (OrbitChunk &chunk : assembly.chunks)
                    {
                        if (!chunk.valid || chunk.frame_segments.empty() ||
                            chunk.t1_s <= t_start_s || chunk.t0_s >= t_end_s)
                        {
                            continue;
                        }

                        const double chunk_t0_s = std::max(chunk.t0_s, t_start_s);
                        const double chunk_t1_s = std::min(chunk.t1_s, t_end_s);
                        if (!(chunk_t1_s > chunk_t0_s) || emitted_total >= max_segments)
                        {
                            continue;
                        }

                        const std::size_t remaining_budget = max_segments - emitted_total;
                        pick_settings.max_segments = std::max<std::size_t>(1, remaining_budget);
                        std::vector<PickingSystem::LinePickSegmentData> chunk_segments;
                        bool cap_hit = false;
                        std::size_t emitted_chunk = 0;
                        if (!chunk.render_curve.empty())
                        {
                            emitted_chunk = build_pick_curve_cache(chunk.render_curve,
                                                                   chunk_t0_s,
                                                                   chunk_t1_s,
                                                                   chunk_segments,
                                                                   cap_hit);
                        }
                        else
                        {
                            emitted_chunk = Draw::build_pick_segment_cache(chunk.frame_segments,
                                                                           ref_body_world,
                                                                           frame_to_world,
                                                                           align_delta,
                                                                           render_frustum,
                                                                           pick_settings,
                                                                           chunk_t0_s,
                                                                           chunk_t1_s,
                                                                           pick_anchor_times_span,
                                                                           false,
                                                                           chunk_segments,
                                                                           cap_hit,
                                                                           _orbit_plot_perf);
                        }

                        if (emitted_chunk == 0 || chunk_segments.empty())
                        {
                            continue;
                        }

                        any_cap_hit = any_cap_hit || cap_hit;
                        emitted_total += chunk_segments.size();
                        io_segments.insert(io_segments.end(), chunk_segments.begin(), chunk_segments.end());
                        out_covered_ranges.emplace_back(chunk_t0_s, chunk_t1_s);
                    }

                    if (any_cap_hit)
                    {
                        _orbit_plot_perf.pick_cap_hit_last_frame = true;
                    }
                    return emitted_total;
                };

                if (allow_base_pick &&
                    base_pick_window.valid &&
                    pick_group_base != Draw::kInvalidPickGroup &&
                    remaining_pick_budget > 0)
                {
                    std::size_t emitted = 0;
                    const std::size_t planned_reserve =
                            planned_pick_window.valid
                                    ? std::min(pick_planned_reserve_target, remaining_pick_budget)
                                    : 0;
                    if (direct_world_polyline)
                    {
                        emitted = Draw::emit_polyline_pick_segments(draw_ctx,
                                                                    picking,
                                                                    pick_group_base,
                                                                    traj_base,
                                                                    base_pick_window.t0_s,
                                                                    base_pick_window.t1_s,
                                                                    remaining_pick_budget - planned_reserve,
                                                                    _orbit_plot_perf);
                    }
                    else
                    {
                        const std::size_t base_pick_budget = remaining_pick_budget - planned_reserve;
                        pick_settings.max_segments = std::max<std::size_t>(1, base_pick_budget);
                        bool rebuilt_pick_cache = false;
                        if (base_pick_budget > 0 &&
                            should_rebuild_pick_cache(track->pick_cache,
                                                      stable_cache->generation_id,
                                                      ref_body_world,
                                                      frame_to_world,
                                                      align_delta,
                                                      camera_world,
                                                      tan_half_fov,
                                                      draw_ctx.viewport_height_px,
                                                      render_error_px,
                                                      render_frustum,
                                                      pick_frustum_margin_ratio,
                                                      base_pick_window.t0_s,
                                                      base_pick_window.t1_s,
                                                      base_pick_budget,
                                                      use_base_adaptive_curve,
                                                      false))
                        {
                            rebuilt_pick_cache = true;
                            bool cap_hit = false;
                            emitted = 0;
                            if (use_base_adaptive_curve)
                            {
                                emitted = build_pick_curve_cache(stable_cache->render_curve_frame,
                                                                 base_pick_window.t0_s,
                                                                 base_pick_window.t1_s,
                                                                 track->pick_cache.base_segments,
                                                                 cap_hit);
                            }
                            else
                            {
                                const std::vector<orbitsim::TrajectorySegment> &pick_base_segments =
                                        identity_frame_transform ? *traj_base_segments : get_base_segments_world_basis();
                                emitted = Draw::build_pick_segment_cache(pick_base_segments,
                                                                         ref_body_world,
                                                                         frame_to_world,
                                                                         align_delta,
                                                                         render_frustum,
                                                                         pick_settings,
                                                                         base_pick_window.t0_s,
                                                                         base_pick_window.t1_s,
                                                                         pick_anchor_times_span,
                                                                         !identity_frame_transform,
                                                                         track->pick_cache.base_segments,
                                                                         cap_hit,
                                                                         _orbit_plot_perf);
                            }
                            if (emitted > 0)
                            {
                                mark_pick_cache_valid(track->pick_cache,
                                                      stable_cache->generation_id,
                                                      ref_body_world,
                                                      frame_to_world,
                                                      align_delta,
                                                      camera_world,
                                                      tan_half_fov,
                                                      draw_ctx.viewport_height_px,
                                                      render_error_px,
                                                      render_frustum,
                                                      pick_frustum_margin_ratio,
                                                      base_pick_window.t0_s,
                                                      base_pick_window.t1_s,
                                                      base_pick_budget,
                                                      use_base_adaptive_curve,
                                                      false);
                            }
                            else
                            {
                                track->pick_cache.base_valid = false;
                                track->pick_cache.base_segments.clear();
                            }
                        }

                        if (base_pick_budget > 0 &&
                            track->pick_cache.base_valid &&
                            !track->pick_cache.base_segments.empty())
                        {
                            emitted = track->pick_cache.base_segments.size();
                            if (!rebuilt_pick_cache)
                            {
                                _orbit_plot_perf.pick_segments_before_cull += static_cast<uint32_t>(emitted);
                                _orbit_plot_perf.pick_segments += static_cast<uint32_t>(emitted);
                            }
                            picking->add_line_pick_segments(
                                    pick_group_base,
                                    std::span<const PickingSystem::LinePickSegmentData>(
                                            track->pick_cache.base_segments.data(),
                                            track->pick_cache.base_segments.size()));
                        }
                    }
                    remaining_pick_budget = (emitted >= remaining_pick_budget) ? 0 : (remaining_pick_budget - emitted);
                }

                if (allow_planned_pick &&
                    planned_pick_window.valid &&
                    pick_group_planned != Draw::kInvalidPickGroup &&
                    remaining_pick_budget > 0 &&
                    !suppress_stale_planned_preview)
                {
                    const bool has_chunk_planned_pick =
                            planned_chunk_assembly &&
                            planned_chunk_assembly->valid &&
                            !planned_chunk_assembly->chunks.empty();
                    PredictionLinePickCache &planned_pick_cache =
                            has_chunk_planned_pick ? track->preview_pick_cache : track->pick_cache;
                    OrbitPredictionCache *const stable_planned_pick_cache =
                            (!maneuver_drag_active &&
                             !stable_cache->trajectory_segments_frame_planned.empty())
                                    ? stable_cache
                                    : nullptr;

                    if (direct_world_polyline && stable_planned_pick_cache &&
                        !stable_planned_pick_cache->trajectory_frame_planned.empty())
                    {
                        Draw::emit_polyline_pick_segments(draw_ctx,
                                                          picking,
                                                          pick_group_planned,
                                                          stable_planned_pick_cache->trajectory_frame_planned,
                                                          planned_pick_window.t0_s,
                                                          planned_pick_window.t1_s,
                                                          remaining_pick_budget,
                                                          _orbit_plot_perf);
                    }
                    else
                    {
                        pick_settings.max_segments = std::max<std::size_t>(1, remaining_pick_budget);
                        bool rebuilt_pick_cache = false;
                        if (should_rebuild_pick_cache(planned_pick_cache,
                                                      has_chunk_planned_pick
                                                              ? planned_chunk_assembly->generation_id
                                                              : (stable_planned_pick_cache
                                                                         ? stable_planned_pick_cache->generation_id
                                                                         : planned_cache->generation_id),
                                                      ref_body_world,
                                                      frame_to_world,
                                                      align_delta,
                                                      camera_world,
                                                      tan_half_fov,
                                                      draw_ctx.viewport_height_px,
                                                      render_error_px,
                                                      render_frustum,
                                                      pick_frustum_margin_ratio,
                                                      planned_pick_window.t0_s,
                                                      planned_pick_window.t1_s,
                                                      remaining_pick_budget,
                                                      has_chunk_planned_pick ||
                                                              (stable_planned_pick_cache &&
                                                               !stable_planned_pick_cache->render_curve_frame_planned.empty()),
                                                      true))
                        {
                            rebuilt_pick_cache = true;
                            planned_pick_cache.planned_segments.clear();
                            if (has_chunk_planned_pick)
                            {
                                std::vector<std::pair<double, double>> covered_ranges;
                                append_pick_segments_from_chunk_assembly(*planned_chunk_assembly,
                                                                         planned_pick_window.t0_s,
                                                                         planned_pick_window.t1_s,
                                                                         remaining_pick_budget,
                                                                         planned_pick_cache.planned_segments,
                                                                         covered_ranges);
                                if (stable_planned_pick_cache &&
                                    planned_pick_cache.planned_segments.size() < remaining_pick_budget)
                                {
                                    const auto fallback_ranges = compute_uncovered_ranges(
                                            planned_pick_window.t0_s,
                                            planned_pick_window.t1_s,
                                            covered_ranges);
                                    for (const auto &[fallback_t0_s, fallback_t1_s] : fallback_ranges)
                                    {
                                        const std::size_t remaining_budget =
                                                remaining_pick_budget - planned_pick_cache.planned_segments.size();
                                        if (remaining_budget == 0)
                                        {
                                            break;
                                        }

                                        pick_settings.max_segments = std::max<std::size_t>(1, remaining_budget);
                                        std::vector<PickingSystem::LinePickSegmentData> fallback_segments;
                                        bool cap_hit = false;
                                        if (!stable_planned_pick_cache->render_curve_frame_planned.empty())
                                        {
                                            build_pick_curve_cache(stable_planned_pick_cache->render_curve_frame_planned,
                                                                   fallback_t0_s,
                                                                   fallback_t1_s,
                                                                   fallback_segments,
                                                                   cap_hit);
                                        }
                                        else
                                        {
                                            const std::vector<orbitsim::TrajectorySegment> &pick_planned_segments =
                                                    identity_frame_transform
                                                            ? stable_planned_pick_cache->trajectory_segments_frame_planned
                                                            : get_planned_segments_world_basis(*stable_planned_pick_cache);
                                            Draw::build_pick_segment_cache(pick_planned_segments,
                                                                           ref_body_world,
                                                                           frame_to_world,
                                                                           align_delta,
                                                                           render_frustum,
                                                                           pick_settings,
                                                                           fallback_t0_s,
                                                                           fallback_t1_s,
                                                                           pick_anchor_times_span,
                                                                           !identity_frame_transform,
                                                                           fallback_segments,
                                                                           cap_hit,
                                                                           _orbit_plot_perf);
                                        }
                                        planned_pick_cache.planned_segments.insert(planned_pick_cache.planned_segments.end(),
                                                                                  fallback_segments.begin(),
                                                                                  fallback_segments.end());
                                    }
                                }
                            }
                            else if (stable_planned_pick_cache)
                            {
                                bool cap_hit = false;
                                if (!stable_planned_pick_cache->render_curve_frame_planned.empty())
                                {
                                    build_pick_curve_cache(stable_planned_pick_cache->render_curve_frame_planned,
                                                           planned_pick_window.t0_s,
                                                           planned_pick_window.t1_s,
                                                           planned_pick_cache.planned_segments,
                                                           cap_hit);
                                }
                                else
                                {
                                    const std::vector<orbitsim::TrajectorySegment> &pick_planned_segments =
                                            identity_frame_transform
                                                    ? stable_planned_pick_cache->trajectory_segments_frame_planned
                                                    : get_planned_segments_world_basis(*stable_planned_pick_cache);
                                    Draw::build_pick_segment_cache(pick_planned_segments,
                                                                   ref_body_world,
                                                                   frame_to_world,
                                                                   align_delta,
                                                                   render_frustum,
                                                                   pick_settings,
                                                                   planned_pick_window.t0_s,
                                                                   planned_pick_window.t1_s,
                                                                   pick_anchor_times_span,
                                                                   !identity_frame_transform,
                                                                   planned_pick_cache.planned_segments,
                                                                   cap_hit,
                                                                   _orbit_plot_perf);
                                }
                            }
                            if (!planned_pick_cache.planned_segments.empty())
                            {
                                mark_pick_cache_valid(planned_pick_cache,
                                                      has_chunk_planned_pick
                                                              ? planned_chunk_assembly->generation_id
                                                              : (stable_planned_pick_cache
                                                                         ? stable_planned_pick_cache->generation_id
                                                                         : planned_cache->generation_id),
                                                      ref_body_world,
                                                      frame_to_world,
                                                      align_delta,
                                                      camera_world,
                                                      tan_half_fov,
                                                      draw_ctx.viewport_height_px,
                                                      render_error_px,
                                                      render_frustum,
                                                      pick_frustum_margin_ratio,
                                                      planned_pick_window.t0_s,
                                                      planned_pick_window.t1_s,
                                                      remaining_pick_budget,
                                                      has_chunk_planned_pick ||
                                                              (stable_planned_pick_cache &&
                                                               !stable_planned_pick_cache->render_curve_frame_planned.empty()),
                                                      true);
                            }
                            else
                            {
                                planned_pick_cache.planned_valid = false;
                                planned_pick_cache.planned_segments.clear();
                            }
                        }

                        if (planned_pick_cache.planned_valid && !planned_pick_cache.planned_segments.empty())
                        {
                            if (!rebuilt_pick_cache)
                            {
                                _orbit_plot_perf.pick_segments_before_cull +=
                                        static_cast<uint32_t>(planned_pick_cache.planned_segments.size());
                                _orbit_plot_perf.pick_segments +=
                                        static_cast<uint32_t>(planned_pick_cache.planned_segments.size());
                            }
                            picking->add_line_pick_segments(
                                    pick_group_planned,
                                    std::span<const PickingSystem::LinePickSegmentData>(
                                            planned_pick_cache.planned_segments.data(),
                                            planned_pick_cache.planned_segments.size()));
                        }
                    }
                }
            }

            if (is_active &&
                _prediction_draw_velocity_ray &&
                _debug_draw_enabled &&
                track->key.kind == PredictionSubjectKind::Orbiter)
            {
                Draw::emit_velocity_ray(ctx.api,
                                        subject_pos_world,
                                        subject_vel_world,
                                        ttl_s,
                                        _prediction_draw_config.palette.velocity_ray);
            }
        }
    }
} // namespace Game
