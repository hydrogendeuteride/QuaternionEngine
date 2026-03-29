#pragma once

#include "game/states/gameplay/maneuver/gameplay_state_maneuver_types.h"

#include "game/orbit/orbit_prediction_math.h"
#include "orbitsim/coordinate_frames.hpp"

#include <glm/glm.hpp>

#include <cmath>
#include <vector>

namespace Game::ManeuverUtil
{
    // ---- Re-export for convenience ----
    using OrbitPredictionMath::safe_length;

    // ---- Shared numeric helpers ----

    inline bool finite3(const glm::dvec3 &v)
    {
        return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
    }

    inline glm::dvec3 normalized_or(const glm::dvec3 &v, const glm::dvec3 &fallback)
    {
        const double len = safe_length(v);
        if (!(len > 0.0) || !std::isfinite(len))
        {
            return fallback;
        }
        return v / len;
    }

    inline orbitsim::RtnFrame compute_maneuver_frame(const glm::dvec3 &r_rel_m,
                                                      const glm::dvec3 &v_rel_mps)
    {
        return orbitsim::compute_rtn_frame(orbitsim::Vec3{r_rel_m.x, r_rel_m.y, r_rel_m.z},
                                           orbitsim::Vec3{v_rel_mps.x, v_rel_mps.y, v_rel_mps.z});
    }

    inline double clamp_sane(double x, double lo, double hi, double fallback = 0.0)
    {
        if (!std::isfinite(x))
        {
            return fallback;
        }
        return std::clamp(x, lo, hi);
    }

    // ---- Basis composition / projection ----

    inline glm::dvec3 compose_basis_vector(const glm::dvec3 &components,
                                           const glm::dvec3 &basis_r,
                                           const glm::dvec3 &basis_t,
                                           const glm::dvec3 &basis_n)
    {
        return basis_r * components.x + basis_t * components.y + basis_n * components.z;
    }

    inline glm::dvec3 project_basis_vector(const glm::dvec3 &world_vec,
                                           const glm::dvec3 &basis_r,
                                           const glm::dvec3 &basis_t,
                                           const glm::dvec3 &basis_n)
    {
        return glm::dvec3(glm::dot(world_vec, basis_r),
                          glm::dot(world_vec, basis_t),
                          glm::dot(world_vec, basis_n));
    }

    inline glm::dvec3 dv_rtn_to_display_basis(const ManeuverNode &node)
    {
        const glm::dvec3 dv_world = compose_basis_vector(node.dv_rtn_mps,
                                                          node.maneuver_basis_r_world,
                                                          node.maneuver_basis_t_world,
                                                          node.maneuver_basis_n_world);
        return project_basis_vector(dv_world,
                                    node.basis_r_world,
                                    node.basis_t_world,
                                    node.basis_n_world);
    }

    // ---- Axis resolution ----

    struct AxisResolveResult
    {
        glm::dvec3 axis_dir_world{0.0, 1.0, 0.0};
        int component{1};   // 0=R, 1=T, 2=N
        double sign{1.0};
        bool valid{false};
    };

    inline AxisResolveResult resolve_axis(const ManeuverHandleAxis axis,
                                          const glm::dvec3 &basis_r,
                                          const glm::dvec3 &basis_t,
                                          const glm::dvec3 &basis_n)
    {
        AxisResolveResult r{};
        switch (axis)
        {
            case ManeuverHandleAxis::TangentialPos:
                r = {basis_t, 1, +1.0, true};
                break;
            case ManeuverHandleAxis::TangentialNeg:
                r = {-basis_t, 1, -1.0, true};
                break;
            case ManeuverHandleAxis::RadialPos:
                r = {basis_r, 0, +1.0, true};
                break;
            case ManeuverHandleAxis::RadialNeg:
                r = {-basis_r, 0, -1.0, true};
                break;
            case ManeuverHandleAxis::NormalPos:
                r = {basis_n, 2, +1.0, true};
                break;
            case ManeuverHandleAxis::NormalNeg:
                r = {-basis_n, 2, -1.0, true};
                break;
            default:
                break;
        }
        return r;
    }

    // ---- Snapshot lookup ----

    inline const ManeuverNodeDisplaySnapshot *find_display_snapshot(
            const std::vector<ManeuverNodeDisplaySnapshot> &snapshots,
            const int node_id)
    {
        for (const ManeuverNodeDisplaySnapshot &s : snapshots)
        {
            if (s.node_id == node_id)
            {
                return &s;
            }
        }
        return nullptr;
    }

    // ---- Label helpers ----

    inline const char *basis_mode_label(const ManeuverGizmoBasisMode mode)
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

    inline const char *axis_short_label(const ManeuverGizmoBasisMode mode,
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
} // namespace Game::ManeuverUtil
