#include "game/states/gameplay/gameplay_state.h"

#include "core/engine.h"
#include "core/game_api.h"
#include "core/device/images.h"
#include "core/input/input_system.h"

#include "game/orbit/orbit_prediction_math.h"
#include "orbitsim/coordinate_frames.hpp"
#include "orbitsim/trajectory_transforms.hpp"

#include "imgui.h"

#include "SDL2/SDL.h"
#include "SDL2/SDL_vulkan.h"

#include <algorithm>
#include <cstdio>
#include <cmath>
#include <limits>
#include <string>

namespace Game
{
    namespace
    {
        using OrbitPredictionMath::safe_length;

        bool finite3(const glm::dvec3 &v)
        {
            return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
        }

        glm::dvec3 compose_basis_vector(const glm::dvec3 &components,
                                        const glm::dvec3 &basis_r,
                                        const glm::dvec3 &basis_t,
                                        const glm::dvec3 &basis_n)
        {
            return basis_r * components.x + basis_t * components.y + basis_n * components.z;
        }

        glm::dvec3 project_basis_vector(const glm::dvec3 &world_vec,
                                        const glm::dvec3 &basis_r,
                                        const glm::dvec3 &basis_t,
                                        const glm::dvec3 &basis_n)
        {
            return glm::dvec3(glm::dot(world_vec, basis_r),
                              glm::dot(world_vec, basis_t),
                              glm::dot(world_vec, basis_n));
        }

        glm::dvec3 normalized_or(const glm::dvec3 &v, const glm::dvec3 &fallback)
        {
            const double len = OrbitPredictionMath::safe_length(v);
            if (!(len > 0.0) || !std::isfinite(len))
            {
                return fallback;
            }
            return v / len;
        }

        const char *maneuver_gizmo_basis_mode_label(const ManeuverGizmoBasisMode mode)
        {
            switch (mode)
            {
                case ManeuverGizmoBasisMode::ProgradeOutwardNormal:
                    return "Prograde / Outward / Normal";
                case ManeuverGizmoBasisMode::RTN:
                    return "RTN";
                default:
                    return "Unknown";
            }
        }

        const char *maneuver_axis_short_label(const ManeuverGizmoBasisMode mode,
                                              const ManeuverHandleAxis axis)
        {
            switch (axis)
            {
                case ManeuverHandleAxis::TangentialPos:
                    return mode == ManeuverGizmoBasisMode::ProgradeOutwardNormal ? "+P" : "+T";
                case ManeuverHandleAxis::TangentialNeg:
                    return mode == ManeuverGizmoBasisMode::ProgradeOutwardNormal ? "-P" : "-T";
                case ManeuverHandleAxis::RadialPos:
                    return mode == ManeuverGizmoBasisMode::ProgradeOutwardNormal ? "+O" : "+R";
                case ManeuverHandleAxis::RadialNeg:
                    return mode == ManeuverGizmoBasisMode::ProgradeOutwardNormal ? "-O" : "-R";
                case ManeuverHandleAxis::NormalPos:
                    return "+N";
                case ManeuverHandleAxis::NormalNeg:
                    return "-N";
                default:
                    return "";
            }
        }

        // Maneuver-node frame uses true RTN so editing, execution, and prediction all share one orthogonal basis:
        // - R: radial, away from the primary
        // - T: tangential / along-track in the orbital plane
        // - N: angular-momentum normal
        orbitsim::RtnFrame compute_maneuver_frame(const glm::dvec3 &r_rel_m,
                                                  const glm::dvec3 &v_rel_mps)
        {
            return orbitsim::compute_rtn_frame(orbitsim::Vec3{r_rel_m.x, r_rel_m.y, r_rel_m.z},
                                               orbitsim::Vec3{v_rel_mps.x, v_rel_mps.y, v_rel_mps.z});
        }

        double clamp_sane(double x, double lo, double hi, double fallback = 0.0)
        {
            if (!std::isfinite(x))
            {
                return fallback;
            }
            return std::clamp(x, lo, hi);
        }

        WorldVec3 hermite_position_world(const WorldVec3 &ref_body_world,
                                         const orbitsim::TrajectorySample &a,
                                         const orbitsim::TrajectorySample &b,
                                         const double t_s)
        {
            // Use sample velocities as tangents so node markers stay visually smooth on the orbit arc.
            const double ta = a.t_s;
            const double tb = b.t_s;
            const double h = tb - ta;
            if (!std::isfinite(h) || !(h > 0.0))
            {
                return ref_body_world + WorldVec3(a.position_m);
            }

            double u = (t_s - ta) / h;
            if (!std::isfinite(u))
            {
                u = 0.0;
            }
            u = std::clamp(u, 0.0, 1.0);

            const double u2 = u * u;
            const double u3 = u2 * u;

            const double h00 = (2.0 * u3) - (3.0 * u2) + 1.0;
            const double h10 = u3 - (2.0 * u2) + u;
            const double h01 = (-2.0 * u3) + (3.0 * u2);
            const double h11 = u3 - u2;

            const glm::dvec3 p0 = glm::dvec3(a.position_m);
            const glm::dvec3 p1 = glm::dvec3(b.position_m);
            const glm::dvec3 m0 = glm::dvec3(a.velocity_mps) * h;
            const glm::dvec3 m1 = glm::dvec3(b.velocity_mps) * h;

            const glm::dvec3 p = (h00 * p0) + (h10 * m0) + (h01 * p1) + (h11 * m1);
            return ref_body_world + WorldVec3(p);
        }

        struct TrajectorySampledState
        {
            bool valid{false};
            glm::dvec3 r_rel_m{0.0, 0.0, 0.0};
            glm::dvec3 v_rel_mps{0.0, 0.0, 0.0};
            WorldVec3 position_world{0.0, 0.0, 0.0};
        };

        TrajectorySampledState sample_trajectory_state(const std::vector<orbitsim::TrajectorySample> &traj_bci,
                                                       const WorldVec3 &ref_body_world,
                                                       const double t_s)
        {
            TrajectorySampledState out{};
            if (traj_bci.size() < 2)
            {
                return out;
            }

            const double t0 = traj_bci.front().t_s;
            const double t1 = traj_bci.back().t_s;
            if (!(t1 > t0) || !std::isfinite(t_s))
            {
                return out;
            }

            const double t_clamped = std::clamp(t_s, t0, t1);

            auto it_hi = std::lower_bound(traj_bci.cbegin(),
                                          traj_bci.cend(),
                                          t_clamped,
                                          [](const orbitsim::TrajectorySample &s, double t) { return s.t_s < t; });
            size_t i_hi = static_cast<size_t>(std::distance(traj_bci.cbegin(), it_hi));
            if (i_hi >= traj_bci.size())
            {
                return out;
            }

            size_t i_lo = (i_hi == 0) ? 0 : (i_hi - 1);
            const orbitsim::TrajectorySample &a = traj_bci[i_lo];
            const orbitsim::TrajectorySample &b = traj_bci[i_hi];

            const double ta = a.t_s;
            const double tb = b.t_s;
            const double h = tb - ta;

            double u = 0.0;
            if (std::isfinite(h) && h > 1e-9)
            {
                u = (t_clamped - ta) / h;
            }
            u = clamp_sane(u, 0.0, 1.0, 0.0);

            // Position: hermite for smoother marker placement.
            out.position_world = hermite_position_world(ref_body_world, a, b, t_clamped);

            // Basis inputs only need to be locally consistent; linear interpolation is stable enough here.
            out.r_rel_m = glm::mix(glm::dvec3(a.position_m), glm::dvec3(b.position_m), u);
            out.v_rel_mps = glm::mix(glm::dvec3(a.velocity_mps), glm::dvec3(b.velocity_mps), u);
            out.valid = finite3(out.r_rel_m) && finite3(out.v_rel_mps);
            return out;
        }

        void draw_diamond(ImDrawList *dl, const ImVec2 &p, float r_px, ImU32 col)
        {
            const ImVec2 pts[4]{
                ImVec2(p.x, p.y - r_px),
                ImVec2(p.x + r_px, p.y),
                ImVec2(p.x, p.y + r_px),
                ImVec2(p.x - r_px, p.y),
            };
            dl->AddConvexPolyFilled(pts, 4, col);
        }

        struct CameraRay
        {
            WorldVec3 camera_world{0.0, 0.0, 0.0};
            glm::dvec3 ray_origin_local{0.0, 0.0, 0.0};
            glm::dvec3 ray_dir_local{0.0, 0.0, -1.0};
        };

        bool compute_camera_ray(const GameStateContext &ctx, const glm::vec2 &window_pos, CameraRay &out_ray)
        {
            if (!ctx.renderer || !ctx.renderer->_sceneManager || !ctx.renderer->_swapchainManager)
            {
                return false;
            }

            int win_w = 0;
            int win_h = 0;
            int draw_w = 0;
            int draw_h = 0;

            if (!ctx.renderer->_window)
            {
                return false;
            }

            SDL_GetWindowSize(ctx.renderer->_window, &win_w, &win_h);
            SDL_Vulkan_GetDrawableSize(ctx.renderer->_window, &draw_w, &draw_h);

            glm::vec2 scale{1.0f, 1.0f};
            if (win_w > 0 && win_h > 0 && draw_w > 0 && draw_h > 0)
            {
                scale.x = static_cast<float>(draw_w) / static_cast<float>(win_w);
                scale.y = static_cast<float>(draw_h) / static_cast<float>(win_h);
            }

            const glm::vec2 drawable_pos{window_pos.x * scale.x, window_pos.y * scale.y};

            VkExtent2D drawable_extent{0, 0};
            if (draw_w > 0 && draw_h > 0)
            {
                drawable_extent.width = static_cast<uint32_t>(draw_w);
                drawable_extent.height = static_cast<uint32_t>(draw_h);
            }
            if ((drawable_extent.width == 0 || drawable_extent.height == 0) && ctx.renderer->_swapchainManager)
            {
                drawable_extent = ctx.renderer->_swapchainManager->windowExtent();
            }

            const VkExtent2D swap_extent = ctx.renderer->_swapchainManager->swapchainExtent();
            if (drawable_extent.width == 0 || drawable_extent.height == 0 ||
                swap_extent.width == 0 || swap_extent.height == 0)
            {
                return false;
            }

            const float sx = static_cast<float>(swap_extent.width) / static_cast<float>(drawable_extent.width);
            const float sy = static_cast<float>(swap_extent.height) / static_cast<float>(drawable_extent.height);
            const glm::vec2 swapchain_pos{drawable_pos.x * sx, drawable_pos.y * sy};

            VkExtent2D logical_extent = ctx.renderer->_logicalRenderExtent;
            if (logical_extent.width == 0 || logical_extent.height == 0)
            {
                logical_extent = VkExtent2D{1280, 720};
            }

            glm::vec2 logical_pos{};
            // Undo the renderer's letterboxing so the ray matches the scene camera rather than the swapchain surface.
            if (!vkutil::map_window_to_letterbox_src(swapchain_pos, logical_extent, swap_extent, logical_pos))
            {
                return false;
            }

            const double width = static_cast<double>(logical_extent.width);
            const double height = static_cast<double>(logical_extent.height);
            if (!(width > 0.0) || !(height > 0.0))
            {
                return false;
            }

            const double ndc_x = (2.0 * static_cast<double>(logical_pos.x) / width) - 1.0;
            const double ndc_y = 1.0 - (2.0 * static_cast<double>(logical_pos.y) / height);

            const Camera &cam = ctx.renderer->_sceneManager->getMainCamera();
            const double fov_rad = glm::radians(static_cast<double>(cam.fovDegrees));
            const double tan_half_fov = std::tan(fov_rad * 0.5);
            const double aspect = width / height;

            glm::dvec3 dir_camera(ndc_x * aspect * tan_half_fov, ndc_y * tan_half_fov, -1.0);
            const double dir_len2 = glm::dot(dir_camera, dir_camera);
            if (!(dir_len2 > 0.0) || !std::isfinite(dir_len2))
            {
                return false;
            }
            dir_camera /= std::sqrt(dir_len2);

            const glm::mat4 cam_rot = cam.getRotationMatrix();
            const glm::vec3 dir_camera_f(static_cast<float>(dir_camera.x),
                                         static_cast<float>(dir_camera.y),
                                         static_cast<float>(dir_camera.z));

            glm::vec3 dir_world_f = glm::vec3(cam_rot * glm::vec4(dir_camera_f, 0.0f));
            const float dir_world_len2 = glm::dot(dir_world_f, dir_world_f);
            if (!(dir_world_len2 > 0.0f) || !std::isfinite(dir_world_len2))
            {
                return false;
            }
            dir_world_f = glm::normalize(dir_world_f);

            out_ray.camera_world = WorldVec3(cam.position_world);
            out_ray.ray_origin_local = glm::dvec3(0.0, 0.0, 0.0);
            out_ray.ray_dir_local = glm::dvec3(dir_world_f);
            return true;
        }

        bool closest_param_ray_line(const glm::dvec3 &ray_origin_local,
                                    const glm::dvec3 &ray_dir_local,
                                    const glm::dvec3 &line_origin_local,
                                    const glm::dvec3 &line_dir_local,
                                    double &out_line_t)
        {
            out_line_t = 0.0;

            // Solve the closest-points problem between a ray and an infinite line by minimizing squared distance.
            // This is the 2x2 least-squares / normal-equations form for two skew lines, then clamped to the ray.
            const double uu = glm::dot(ray_dir_local, ray_dir_local);
            const double vv = glm::dot(line_dir_local, line_dir_local);
            if (!(uu > 0.0) || !(vv > 0.0) || !std::isfinite(uu) || !std::isfinite(vv))
            {
                return false;
            }

            const glm::dvec3 w0 = line_origin_local - ray_origin_local;
            const double uv = glm::dot(ray_dir_local, line_dir_local);
            const double uw = glm::dot(ray_dir_local, w0);
            const double vw = glm::dot(line_dir_local, w0);
            const double denom = uu * vv - uv * uv;

            double s = 0.0; // ray distance
            double t = 0.0; // line parameter
            if (std::abs(denom) > 1.0e-9 && std::isfinite(denom))
            {
                s = (uw * vv - vw * uv) / denom;
                t = (uw * uv - vw * uu) / denom;
                if (!std::isfinite(s) || !std::isfinite(t))
                {
                    return false;
                }
                if (s < 0.0)
                {
                    s = 0.0;
                    t = -vw / vv;
                }
            }
            else
            {
                // Nearly parallel: choose the axis parameter nearest to the ray origin.
                t = -vw / vv;
                if (!std::isfinite(t))
                {
                    return false;
                }
            }

            out_line_t = t;
            return std::isfinite(out_line_t);
        }

    } // namespace

    orbitsim::BodyId GameplayState::resolve_maneuver_node_primary_body_id(const ManeuverNode &node,
                                                                          const double query_time_s) const
    {
        if (!node.primary_body_auto && node.primary_body_id != orbitsim::kInvalidBodyId)
        {
            return node.primary_body_id;
        }

        const PredictionTrackState *player_track = player_prediction_track();
        if (player_track && player_track->cache.valid)
        {
            const OrbitPredictionCache &cache = player_track->cache;
            const auto &traj =
                    cache.trajectory_inertial_planned.size() >= 2 ? cache.trajectory_inertial_planned
                                                                  : cache.trajectory_inertial;

            orbitsim::State sc_state{};
            if (!traj.empty() &&
                sample_prediction_inertial_state(traj, query_time_s, sc_state) &&
                !cache.massive_bodies.empty())
            {
                const orbitsim::BodyId preferred_body_id =
                        node.primary_body_id != orbitsim::kInvalidBodyId ? node.primary_body_id : player_track->auto_primary_body_id;
                const orbitsim::BodyId primary_body_id = select_prediction_primary_body_id(
                        cache.massive_bodies,
                        &cache,
                        sc_state.position_m,
                        query_time_s,
                        preferred_body_id);
                if (primary_body_id != orbitsim::kInvalidBodyId)
                {
                    return primary_body_id;
                }
            }

            if (!cache.trajectory_inertial.empty())
            {
                const orbitsim::BodyId analysis_body_id =
                        resolve_prediction_analysis_body_id(cache, player_track->key, query_time_s, node.primary_body_id);
                if (analysis_body_id != orbitsim::kInvalidBodyId)
                {
                    return analysis_body_id;
                }
            }
        }

        if (node.primary_body_id != orbitsim::kInvalidBodyId)
        {
            return node.primary_body_id;
        }

        if (_orbitsim && _orbitsim->world_reference_body())
        {
            return _orbitsim->world_reference_body()->sim_id;
        }

        return orbitsim::kInvalidBodyId;
    }

    #include "gameplay_state_maneuver_nodes_panel.inc"
    #include "gameplay_state_maneuver_nodes_gizmo.inc"
    #include "gameplay_state_maneuver_nodes_runtime.inc"
} // namespace Game
