#pragma once

#include "core/engine.h"
#include "game/states/gameplay/gameplay_state.h"

#include "game/orbit/orbit_plot_lod_builder.h"
#include "core/game_api.h"
#include "core/orbit_plot/orbit_plot.h"

#include <cstddef>
#include <cstdint>
#include <limits>
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
        OrbitPlotLodBuilder::CameraContext lod_camera{};
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

    void reset_orbit_plot_state(PickingSystem *picking,
                                OrbitPlotSystem *orbit_plot,
                                OrbitPlotPerfStats &perf,
                                bool prediction_enabled);
    glm::vec4 scale_line_color(glm::vec4 color, float line_alpha_scale);
    double compute_prediction_display_time_s(double sim_time_s,
                                             double last_sim_step_dt_s,
                                             float fixed_delta_time,
                                             float alpha_f);
    double compute_prediction_now_s(double display_time_s, double t0, double t1);
    double meters_per_px_at_world(const OrbitDrawWindowContext &ctx, const WorldVec3 &p_world);
    std::vector<orbitsim::TrajectorySegment> trajectory_segments_from_samples(
            const std::vector<orbitsim::TrajectorySample> &samples);
    double snap_time_past_straddling_segment(const std::vector<orbitsim::TrajectorySegment> &traj_segments, double t_s);
    std::vector<double> collect_maneuver_node_times(const std::vector<ManeuverNode> &nodes);
    std::size_t lower_bound_sample_index(const std::vector<orbitsim::TrajectorySample> &traj, double t_s);
    WorldVec3 sample_polyline_world(const WorldVec3 &frame_origin_world,
                                    const glm::dmat3 &frame_to_world,
                                    const std::vector<orbitsim::TrajectorySample> &traj,
                                    const std::vector<WorldVec3> &points_world,
                                    std::size_t i_lo,
                                    std::size_t i_hi,
                                    double t_s);
    WorldVec3 compute_align_delta(const std::vector<orbitsim::TrajectorySample> &traj_base,
                                  const std::vector<WorldVec3> &points_base,
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
    PickWindow build_planned_pick_window(const std::vector<orbitsim::TrajectorySegment> &traj_planned_segments,
                                         const OrbitPredictionDrawConfig &draw_config,
                                         const std::vector<ManeuverNode> &nodes,
                                         double now_s,
                                         double future_window_s,
                                         bool draw_future_segment,
                                         bool draw_full_orbit,
                                         double orbital_period_s);
    std::size_t emit_pick_segments(PickingSystem *picking,
                                   uint32_t pick_group,
                                   const std::vector<orbitsim::TrajectorySegment> &traj_segments,
                                   const WorldVec3 &ref_body_world,
                                   const WorldVec3 &align_delta,
                                   const OrbitPlotLodBuilder::FrustumContext &pick_frustum,
                                   const OrbitPlotLodBuilder::PickSettings &pick_settings,
                                   double t0_s,
                                   double t1_s,
                                   const std::vector<double> &anchor_times,
                                   OrbitPlotPerfStats &perf);
    bool frame_spec_uses_direct_world_polyline(const orbitsim::TrajectoryFrameSpec &spec);
    void draw_polyline_window(const OrbitDrawWindowContext &ctx,
                              const OrbitPredictionDrawConfig &draw_config,
                              const std::vector<orbitsim::TrajectorySample> &traj,
                              const std::vector<WorldVec3> &points_world,
                              double t_start_s,
                              double t_end_s,
                              const glm::vec4 &color,
                              bool dashed);
    std::size_t emit_polyline_pick_segments(const OrbitDrawWindowContext &ctx,
                                            PickingSystem *picking,
                                            uint32_t pick_group,
                                            const std::vector<orbitsim::TrajectorySample> &traj,
                                            const std::vector<WorldVec3> &points_world,
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
