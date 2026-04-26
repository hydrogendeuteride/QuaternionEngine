#pragma once

#include "core/engine.h"
#include "game/states/gameplay/gameplay_state.h"

#include "game/orbit/orbit_render_curve.h"
#include "core/game_api.h"
#include "core/orbit_plot/orbit_plot.h"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>
#include <utility>
#include <vector>

namespace Game::PredictionDrawDetail
{
    inline constexpr uint32_t kInvalidPickGroup = std::numeric_limits<uint32_t>::max();

    struct OrbitDrawWindowContext
    {
        OrbitPlotSystem *orbit_plot{nullptr};
        WorldVec3 ref_body_world{0.0, 0.0, 0.0};
        glm::dmat3 frame_to_world{1.0};
        WorldVec3 align_delta{0.0, 0.0, 0.0};
        OrbitRenderCurve::CameraContext lod_camera{};
        OrbitRenderCurve::FrustumContext render_frustum{};
        glm::dvec3 camera_world{0.0, 0.0, 0.0};
        double tan_half_fov{0.0};
        double viewport_height_px{1.0};
        double render_error_px{0.75};
        std::size_t render_max_segments{1};
        float line_overlay_boost{0.0f};
    };

    struct PickWindow
    {
        bool valid{false};
        double t0_s{0.0};
        double t1_s{0.0};
        double anchor_time_s{std::numeric_limits<double>::quiet_NaN()};
    };

    struct PredictionGlobalDrawContext
    {
        PickingSystem *picking{nullptr};
        OrbitPlotSystem *orbit_plot{nullptr};
        float alpha_f{0.0f};
        double display_time_s{std::numeric_limits<double>::quiet_NaN()};
        float ttl_s{0.0f};
        glm::dvec3 camera_world{0.0, 0.0, 0.0};
        float viewport_height_px{720.0f};
        double tan_half_fov{0.0};
        OrbitRenderCurve::CameraContext lod_camera{};
        OrbitRenderCurve::FrustumContext render_frustum{};
        double render_error_px{0.75};
        float line_alpha_scale{1.0f};
        glm::vec4 color_orbit_plan{1.0f};
    };

    struct PredictionTrackDrawContext
    {
        PredictionTrackState *track{nullptr};
        OrbitPredictionCache *stable_cache{nullptr};
        OrbitPredictionCache *planned_cache{nullptr};
        OrbitPredictionCache *stale_planned_cache{nullptr};
        OrbitPredictionCache *display_cache{nullptr};
        const std::vector<orbitsim::TrajectorySample> *traj_base{nullptr};
        const std::vector<orbitsim::TrajectorySample> *traj_planned{nullptr};
        const std::vector<orbitsim::TrajectorySegment> *traj_base_segments{nullptr};
        const std::vector<orbitsim::TrajectorySegment> *traj_planned_segments{nullptr};
        const std::vector<orbitsim::TrajectorySegment> *planned_window_segments{nullptr};
        WorldVec3 ref_body_world{0.0, 0.0, 0.0};
        glm::dmat3 frame_to_world{1.0};
        WorldVec3 align_delta{0.0, 0.0, 0.0};
        WorldVec3 subject_pos_world{0.0, 0.0, 0.0};
        WorldVec3 subject_pos_world_state{0.0, 0.0, 0.0};
        glm::dvec3 subject_vel_world{0.0, 0.0, 0.0};
        glm::vec3 subject_vel_local{0.0f, 0.0f, 0.0f};
        double t0_s{0.0};
        double t1_s{0.0};
        double now_s{std::numeric_limits<double>::quiet_NaN()};
        std::size_t i_hi{0};
        bool is_active{false};
        bool active_player_track{false};
        bool maneuver_drag_active{false};
        bool planned_cache_current{false};
        bool planned_cache_drawable{false};
        bool planned_cache_prefix_only{false};
        double planned_cache_prefix_cutoff_s{std::numeric_limits<double>::quiet_NaN()};
        bool stale_planned_cache_drawable{false};
        double stale_planned_cache_prefix_cutoff_s{std::numeric_limits<double>::quiet_NaN()};
        bool direct_world_polyline{false};
        bool identity_frame_transform{true};
        bool use_base_adaptive_curve{false};
        bool use_planned_adaptive_curve{false};
        OrbitDrawWindowContext draw_ctx{};
        OrbitDrawWindowContext world_basis_draw_ctx{};
        glm::vec4 track_color_full{1.0f};
        glm::vec4 track_color_future{1.0f};
        glm::vec4 track_color_plan{1.0f};
        PredictionWindowPolicyResult planned_window_policy{};
        double future_window_s{0.0};
        double planned_visual_window_s{0.0};
        double planned_exact_window_s{0.0};
        double planned_pick_window_s{0.0};
        PickWindow base_pick_window{};
        PickWindow planned_draw_window{};
        PickWindow planned_pick_window{};
        std::vector<orbitsim::TrajectorySegment> traj_base_segments_world_basis{};
        OrbitPredictionCache *traj_planned_segments_world_basis_source{nullptr};
        std::vector<orbitsim::TrajectorySegment> traj_stable_planned_segments_world_basis{};
    };

    void reset_orbit_plot_state(PickingSystem *picking,
                                OrbitPlotSystem *orbit_plot,
                                OrbitPlotPerfStats &perf,
                                bool prediction_enabled);
    bool same_matrix(const glm::dmat3 &a, const glm::dmat3 &b, double epsilon);
    bool same_matrix(const glm::mat4 &a, const glm::mat4 &b, float epsilon);
    glm::vec4 scale_line_color(glm::vec4 color, float line_alpha_scale);
    double compute_prediction_display_time_s(double sim_time_s,
                                             double last_sim_step_dt_s,
                                             float fixed_delta_time,
                                             float alpha_f);
    double compute_prediction_now_s(double display_time_s, double t0, double t1);
    double meters_per_px_at_world(const OrbitDrawWindowContext &ctx, const WorldVec3 &p_world);
    double snap_time_past_straddling_segment(const std::vector<orbitsim::TrajectorySegment> &traj_segments, double t_s);
    std::vector<double> collect_maneuver_node_times(const std::vector<ManeuverNode> &nodes);
    std::size_t lower_bound_sample_index(const std::vector<orbitsim::TrajectorySample> &traj, double t_s);
    bool frame_transform_is_identity(const glm::dmat3 &frame_to_world);
    const std::vector<orbitsim::TrajectorySegment> &base_segments_world_basis(PredictionTrackDrawContext &track_ctx);
    const std::vector<orbitsim::TrajectorySegment> &planned_segments_world_basis(PredictionTrackDrawContext &track_ctx,
                                                                                 OrbitPredictionCache &cache);
    std::vector<orbitsim::TrajectorySegment> transform_segments_to_world_basis(
            const std::vector<orbitsim::TrajectorySegment> &traj_segments,
            const glm::dmat3 &frame_to_world);
    WorldVec3 sample_polyline_world(const WorldVec3 &frame_origin_world,
                                    const glm::dmat3 &frame_to_world,
                                    const std::vector<orbitsim::TrajectorySample> &traj,
                                    std::size_t i_lo,
                                    std::size_t i_hi,
                                    double t_s);
    bool sample_prediction_path_world(const OrbitDrawWindowContext &ctx,
                                      const std::vector<orbitsim::TrajectorySegment> &traj_segments,
                                      const std::vector<orbitsim::TrajectorySample> &traj_samples,
                                      double t_s,
                                      WorldVec3 &out_world);
    WorldVec3 compute_align_delta(const std::vector<orbitsim::TrajectorySegment> &traj_segments,
                                  const std::vector<orbitsim::TrajectorySample> &traj_base,
                                  std::size_t i_hi,
                                  const WorldVec3 &ship_pos_world,
                                  double now_s,
                                  const WorldVec3 &frame_origin_world,
                                  const glm::dmat3 &frame_to_world);
    void draw_orbit_window(const OrbitDrawWindowContext &ctx,
                           const OrbitPredictionDrawConfig &draw_config,
                           OrbitPlotPerfStats &perf,
                           const std::vector<orbitsim::TrajectorySegment> &traj_segments,
                           double t_start_s,
                           double t_end_s,
                           const glm::vec4 &color,
                           bool dashed);
    void draw_adaptive_curve_window(const OrbitDrawWindowContext &ctx,
                                    const OrbitPredictionDrawConfig &draw_config,
                                    OrbitPlotPerfStats &perf,
                                    const OrbitRenderCurve &curve,
                                    double t_start_s,
                                    double t_end_s,
                                    const glm::vec4 &color,
                                    bool dashed);
    std::vector<std::pair<double, double>> compute_uncovered_ranges(
            double t_start_s,
            double t_end_s,
            std::vector<std::pair<double, double>> covered_ranges);
    PickWindow build_planned_draw_window(const std::vector<orbitsim::TrajectorySegment> &traj_planned_segments,
                                         const OrbitPredictionDrawConfig &draw_config,
                                         const PredictionWindowPolicyResult &policy);
    PickWindow build_planned_pick_window(const std::vector<orbitsim::TrajectorySegment> &traj_planned_segments,
                                         const OrbitPredictionDrawConfig &draw_config,
                                         const PredictionWindowPolicyResult &policy);
    std::size_t build_pick_segment_cache(const std::vector<orbitsim::TrajectorySegment> &traj_segments,
                                         const WorldVec3 &ref_body_world,
                                         const glm::dmat3 &frame_to_world,
                                         const WorldVec3 &align_delta,
                                         const OrbitRenderCurve::FrustumContext &pick_frustum,
                                         const OrbitRenderCurve::PickSettings &pick_settings,
                                         double t0_s,
                                         double t1_s,
                                         std::span<const double> anchor_times_s,
                                         bool segments_are_world_basis,
                                         std::vector<PickingSystem::LinePickSegmentData> &out_segments,
                                         bool &out_cap_hit,
                                         OrbitPlotPerfStats &perf);
    bool should_rebuild_pick_cache(const PredictionLinePickCache &cache,
                                   uint64_t generation_id,
                                   uint64_t display_frame_key,
                                   uint64_t display_frame_revision,
                                   const WorldVec3 &ref_body_world,
                                   const glm::dmat3 &frame_to_world,
                                   const WorldVec3 &align_delta,
                                   const glm::dvec3 &camera_world,
                                   double tan_half_fov,
                                   double viewport_height_px,
                                   double render_error_px,
                                   const OrbitRenderCurve::FrustumContext &pick_frustum,
                                   double pick_frustum_margin_ratio,
                                   double t0_s,
                                   double t1_s,
                                   std::size_t max_segments,
                                   bool use_adaptive_curve,
                                   bool planned);
    void mark_pick_cache_valid(PredictionLinePickCache &cache,
                               uint64_t generation_id,
                               uint64_t display_frame_key,
                               uint64_t display_frame_revision,
                               const WorldVec3 &ref_body_world,
                               const glm::dmat3 &frame_to_world,
                               const WorldVec3 &align_delta,
                               const glm::dvec3 &camera_world,
                               double tan_half_fov,
                               double viewport_height_px,
                               double render_error_px,
                               const OrbitRenderCurve::FrustumContext &pick_frustum,
                               double pick_frustum_margin_ratio,
                               double t0_s,
                               double t1_s,
                               std::size_t max_segments,
                               bool use_adaptive_curve,
                               bool planned);
    bool same_pick_time_window(double cached_t0_s,
                               double cached_t1_s,
                               double t0_s,
                               double t1_s);
    bool frame_spec_uses_direct_world_polyline(const orbitsim::TrajectoryFrameSpec &spec);
    void draw_polyline_window(const OrbitDrawWindowContext &ctx,
                              const OrbitPredictionDrawConfig &draw_config,
                              const std::vector<orbitsim::TrajectorySample> &traj,
                              double t_start_s,
                              double t_end_s,
                              const glm::vec4 &color,
                              bool dashed);
    std::size_t emit_polyline_pick_segments(const OrbitDrawWindowContext &ctx,
                                            PickingSystem *picking,
                                            uint32_t pick_group,
                                            const std::vector<orbitsim::TrajectorySample> &traj,
                                            double t0_s,
                                            double t1_s,
                                            std::size_t max_segments,
                                            OrbitPlotPerfStats &perf);
    void emit_velocity_ray(GameAPI::Engine *api,
                           const WorldVec3 &ship_pos_world,
                           const glm::dvec3 &ship_vel_world,
                           float ttl_s,
                           const glm::vec4 &color);
} // namespace Game::PredictionDrawDetail
