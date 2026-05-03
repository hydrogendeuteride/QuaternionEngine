#include "game/orbit/prediction/orbit_prediction_service_internal.h"

#include "orbitsim/frame_utils.hpp"
#include "orbitsim/trajectory_transforms.hpp"

namespace Game
{
    namespace
    {
        bool maneuver_impulse_has_nonzero_delta_v(const OrbitPredictionService::ManeuverImpulse &src)
        {
            const double dv2 = glm::dot(src.dv_rtn_mps, src.dv_rtn_mps);
            return dv2 > 0.0 && std::isfinite(dv2);
        }

        std::optional<std::size_t> resolve_maneuver_primary_index(
                const OrbitPredictionService::ManeuverImpulse &src,
                const std::vector<orbitsim::MassiveBody> &massive_bodies,
                const orbitsim::CelestialEphemeris &ephemeris,
                const double softening_length_m,
                const orbitsim::Vec3 &query_position_m)
        {
            if (src.primary_body_id != orbitsim::kInvalidBodyId)
            {
                return orbitsim::body_index_for_id(massive_bodies, src.primary_body_id);
            }

            if (massive_bodies.empty() || !finite_vec3(query_position_m))
            {
                return std::nullopt;
            }

            const double eps2 = softening_length_m * softening_length_m;
            double best_metric = -1.0;
            std::optional<std::size_t> best_index{};
            for (std::size_t i = 0; i < massive_bodies.size(); ++i)
            {
                const double mass_kg = std::isfinite(massive_bodies[i].mass_kg) ? massive_bodies[i].mass_kg : 0.0;
                if (!(mass_kg >= 0.0))
                {
                    continue;
                }

                const orbitsim::Vec3 body_position_m = ephemeris.body_position_at(i, src.t_s);
                if (!finite_vec3(body_position_m))
                {
                    continue;
                }

                const orbitsim::Vec3 dr = body_position_m - query_position_m;
                const double r2 = glm::dot(dr, dr) + eps2;
                if (!(r2 > 0.0) || !std::isfinite(r2))
                {
                    continue;
                }

                const double metric = mass_kg / r2;
                if (std::isfinite(metric) && metric > best_metric)
                {
                    best_metric = metric;
                    best_index = i;
                }
            }
            return best_index;
        }

        bool maneuver_rtn_basis_is_valid(const std::vector<orbitsim::MassiveBody> &massive_bodies,
                                         const orbitsim::CelestialEphemeris &ephemeris,
                                         const std::size_t primary_index,
                                         const double t_s,
                                         const orbitsim::State &spacecraft_state)
        {
            if (primary_index >= massive_bodies.size() || !finite_state(spacecraft_state))
            {
                return false;
            }

            const orbitsim::State primary_state = ephemeris.body_state_at(primary_index, t_s);
            if (!finite_state(primary_state))
            {
                return false;
            }

            const orbitsim::Vec3 relative_position_m =
                    spacecraft_state.position_m - primary_state.position_m;
            const double relative_position_len2 = glm::dot(relative_position_m, relative_position_m);
            return finite_vec3(relative_position_m) &&
                   relative_position_len2 > 0.0 &&
                   std::isfinite(relative_position_len2);
        }
    } // namespace

    bool planned_maneuver_impulse_input_is_valid(const OrbitPredictionService::ManeuverImpulse &src)
    {
        return std::isfinite(src.t_s) && finite_vec3(src.dv_rtn_mps);
    }

    void record_planned_maneuver_apply_failure(OrbitPredictionService::AdaptiveStageDiagnostics &diagnostics,
                                               const int node_id)
    {
        ++diagnostics.maneuver_apply_failed_count;
        if (diagnostics.maneuver_apply_failed_node_id < 0)
        {
            diagnostics.maneuver_apply_failed_node_id = node_id;
        }
    }

    bool apply_planned_maneuver_impulse_to_spacecraft(
            orbitsim::Spacecraft &spacecraft,
            const OrbitPredictionService::ManeuverImpulse &src,
            const std::vector<orbitsim::MassiveBody> &massive_bodies,
            const orbitsim::CelestialEphemeris &ephemeris,
            const double softening_length_m,
            std::vector<OrbitPredictionService::ManeuverNodePreview> *out_previews,
            std::vector<PlannedSegmentBoundaryState> *out_boundaries,
            OrbitPredictionService::AdaptiveStageDiagnostics &diagnostics)
    {
        if (!planned_maneuver_impulse_input_is_valid(src))
        {
            record_planned_maneuver_apply_failure(diagnostics, src.node_id);
            return false;
        }

        OrbitPredictionService::ManeuverNodePreview preview{};
        preview.node_id = src.node_id;
        preview.t_s = src.t_s;

        const orbitsim::State pre_burn_state = spacecraft.state;
        const bool have_preview = build_maneuver_preview(spacecraft.state, src.t_s, preview);
        if (have_preview && out_previews)
        {
            out_previews->push_back(preview);
        }

        if (!have_preview)
        {
            record_planned_maneuver_apply_failure(diagnostics, src.node_id);
            return false;
        }

        if (maneuver_impulse_has_nonzero_delta_v(src))
        {
            const std::optional<std::size_t> primary_index =
                    resolve_maneuver_primary_index(src,
                                                   massive_bodies,
                                                   ephemeris,
                                                   softening_length_m,
                                                   preview.inertial_position_m);
            if (!primary_index.has_value() ||
                !maneuver_rtn_basis_is_valid(massive_bodies,
                                             ephemeris,
                                             *primary_index,
                                             src.t_s,
                                             spacecraft.state))
            {
                record_planned_maneuver_apply_failure(diagnostics, src.node_id);
                return false;
            }

            const orbitsim::Vec3 dv_inertial_mps = orbitsim::rtn_vector_to_inertial(
                    ephemeris,
                    massive_bodies,
                    *primary_index,
                    src.t_s,
                    preview.inertial_position_m,
                    preview.inertial_velocity_mps,
                    src.dv_rtn_mps);
            if (!finite_vec3(dv_inertial_mps))
            {
                record_planned_maneuver_apply_failure(diagnostics, src.node_id);
                return false;
            }

            spacecraft.state.velocity_mps += dv_inertial_mps;
        }

        if (out_boundaries)
        {
            append_or_merge_planned_boundary_state(
                    *out_boundaries,
                    src.t_s,
                    pre_burn_state,
                    spacecraft.state);
        }
        return true;
    }
} // namespace Game
