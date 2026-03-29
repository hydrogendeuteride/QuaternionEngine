#include "game/orbit/prediction/orbit_prediction_service_internal.h"

#include <glm/geometric.hpp>

namespace Game
{
    namespace
    {
        using PredictionChunkBoundaryFlags = OrbitPredictionService::PredictionChunkBoundaryFlags;
        using PredictionProfileId = OrbitPredictionService::PredictionProfileId;

        [[nodiscard]] bool planner_boundary_has_flag(const uint32_t flags, const PredictionChunkBoundaryFlags flag)
        {
            return (flags & static_cast<uint32_t>(flag)) != 0u;
        }

        [[nodiscard]] bool chunk_has_any_boundary(const OrbitPredictionService::PredictionChunkPlan &chunk,
                                                  const std::initializer_list<PredictionChunkBoundaryFlags> flags)
        {
            for (const PredictionChunkBoundaryFlags flag : flags)
            {
                if (planner_boundary_has_flag(chunk.boundary_flags, flag))
                {
                    return true;
                }
            }
            return false;
        }

        [[nodiscard]] double safe_angle_between(const orbitsim::Vec3 &a, const orbitsim::Vec3 &b)
        {
            const double len_a = glm::length(glm::dvec3(a));
            const double len_b = glm::length(glm::dvec3(b));
            if (!(len_a > 0.0) || !(len_b > 0.0) || !std::isfinite(len_a) || !std::isfinite(len_b))
            {
                return 0.0;
            }

            const double dot_value = glm::dot(glm::dvec3(a), glm::dvec3(b)) / (len_a * len_b);
            return std::acos(std::clamp(dot_value, -1.0, 1.0));
        }

        [[nodiscard]] orbitsim::Vec3 body_position_at_time(const OrbitPredictionService::SharedCelestialEphemeris &shared_ephemeris,
                                                           const std::vector<orbitsim::MassiveBody> &bodies,
                                                           const std::size_t index,
                                                           const double time_s)
        {
            if (index >= bodies.size())
            {
                return orbitsim::Vec3(0.0);
            }

            if (shared_ephemeris && !shared_ephemeris->empty())
            {
                std::size_t eph_index = 0u;
                if (bodies[index].id != orbitsim::kInvalidBodyId &&
                    shared_ephemeris->body_index_for_id(bodies[index].id, &eph_index))
                {
                    return shared_ephemeris->body_position_at(eph_index, time_s);
                }
            }

            return bodies[index].state.position_m;
        }

        [[nodiscard]] orbitsim::BodyId select_primary_body_id_for_state(
                const OrbitPredictionService::Request &request,
                const OrbitPredictionService::SharedCelestialEphemeris &shared_ephemeris,
                const orbitsim::State &state,
                const double time_s,
                const orbitsim::BodyId preferred_body_id)
        {
            if (request.massive_bodies.empty() || !finite_state(state))
            {
                return orbitsim::kInvalidBodyId;
            }

            const std::size_t primary_index = select_primary_index_with_hysteresis(
                    request.massive_bodies,
                    state.position_m,
                    [&shared_ephemeris, &request, time_s](const std::size_t i) -> orbitsim::Vec3 {
                        return body_position_at_time(shared_ephemeris, request.massive_bodies, i, time_s);
                    },
                    request.sim_config.softening_length_m,
                    preferred_body_id);
            return primary_index < request.massive_bodies.size()
                           ? request.massive_bodies[primary_index].id
                           : orbitsim::kInvalidBodyId;
        }

        [[nodiscard]] double dominant_gravity_ratio_for_state(
                const OrbitPredictionService::Request &request,
                const OrbitPredictionService::SharedCelestialEphemeris &shared_ephemeris,
                const orbitsim::State &state,
                const double time_s)
        {
            if (request.massive_bodies.empty() || !finite_state(state))
            {
                return 1.0;
            }

            const double eps2 = request.sim_config.softening_length_m * request.sim_config.softening_length_m;
            double total_metric = 0.0;
            double dominant_metric = 0.0;
            for (std::size_t i = 0; i < request.massive_bodies.size(); ++i)
            {
                const double mass_kg = request.massive_bodies[i].mass_kg;
                if (!(mass_kg > 0.0) || !std::isfinite(mass_kg))
                {
                    continue;
                }

                const orbitsim::Vec3 dr = body_position_at_time(shared_ephemeris, request.massive_bodies, i, time_s) -
                                          state.position_m;
                const double r2 = glm::dot(dr, dr) + eps2;
                if (!(r2 > 0.0) || !std::isfinite(r2))
                {
                    continue;
                }

                const double metric = mass_kg / r2;
                total_metric += metric;
                dominant_metric = std::max(dominant_metric, metric);
            }

            if (!(total_metric > 0.0))
            {
                return 1.0;
            }

            return std::clamp(dominant_metric / total_metric, 0.0, 1.0);
        }

        bool eval_activity_segment_state(const orbitsim::TrajectorySegment &segment,
                                         const double t_s,
                                         orbitsim::State &out_state)
        {
            out_state = segment.start;
            if (!(segment.dt_s > 0.0) || !std::isfinite(segment.dt_s) || !std::isfinite(t_s))
            {
                return finite_state(out_state);
            }

            double u = (t_s - segment.t0_s) / segment.dt_s;
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
            const double dh00 = (6.0 * u2) - (6.0 * u);
            const double dh10 = (3.0 * u2) - (4.0 * u) + 1.0;
            const double dh01 = (-6.0 * u2) + (6.0 * u);
            const double dh11 = (3.0 * u2) - (2.0 * u);

            const glm::dvec3 p0 = glm::dvec3(segment.start.position_m);
            const glm::dvec3 p1 = glm::dvec3(segment.end.position_m);
            const glm::dvec3 m0 = glm::dvec3(segment.start.velocity_mps) * segment.dt_s;
            const glm::dvec3 m1 = glm::dvec3(segment.end.velocity_mps) * segment.dt_s;
            const glm::dvec3 pos = (h00 * p0) + (h10 * m0) + (h01 * p1) + (h11 * m1);
            const glm::dvec3 vel = ((dh00 * p0) + (dh10 * m0) + (dh01 * p1) + (dh11 * m1)) / segment.dt_s;
            if (!finite_vec3(pos) || !finite_vec3(vel))
            {
                return false;
            }

            out_state = orbitsim::make_state(pos, vel);
            return true;
        }

        bool sample_activity_segment_state(const std::vector<orbitsim::TrajectorySegment> &segments,
                                           const double t_s,
                                           orbitsim::State &out_state)
        {
            out_state = {};
            if (segments.empty() || !std::isfinite(t_s))
            {
                return false;
            }

            auto it = std::lower_bound(segments.begin(),
                                       segments.end(),
                                       t_s,
                                       [](const orbitsim::TrajectorySegment &segment, const double query_t_s) {
                                           return prediction_segment_end_time(segment) < query_t_s;
                                       });
            if (it == segments.end())
            {
                return eval_activity_segment_state(segments.back(),
                                                   prediction_segment_end_time(segments.back()),
                                                   out_state);
            }
            return eval_activity_segment_state(*it, t_s, out_state);
        }

        [[nodiscard]] double scale_dt_value(const double base_value,
                                            const double scale,
                                            const double floor_value)
        {
            if (!(base_value > 0.0))
            {
                return std::max(0.0, floor_value);
            }

            const double safe_scale = std::isfinite(scale) ? std::max(scale, 1.0e-6) : 1.0;
            const double scaled = base_value * safe_scale;
            return std::max(std::max(0.0, floor_value), scaled);
        }

        [[nodiscard]] std::size_t scale_segment_cap(const std::size_t base_value,
                                                    const double scale,
                                                    const std::size_t floor_value)
        {
            if (base_value == 0u)
            {
                return floor_value;
            }

            const double safe_scale = std::isfinite(scale) ? std::max(scale, 1.0e-6) : 1.0;
            const double scaled = std::ceil(static_cast<double>(base_value) * safe_scale);
            return std::max<std::size_t>(floor_value,
                                         static_cast<std::size_t>(std::max(1.0, scaled)));
        }
    } // namespace

    OrbitPredictionService::ChunkActivityProbe classify_chunk_activity(
            const OrbitPredictionService::Request &request,
            const OrbitPredictionService::PredictionChunkPlan &chunk,
            const std::vector<orbitsim::TrajectorySegment> *baseline_segments,
            const OrbitPredictionService::SharedCelestialEphemeris &shared_ephemeris)
    {
        OrbitPredictionService::ChunkActivityProbe probe{};
        probe.recommended_profile_id = chunk.profile_id;
        if (!baseline_segments || baseline_segments->empty() || !(chunk.t1_s > chunk.t0_s))
        {
            return probe;
        }

        const double mid_t_s = chunk.t0_s + (chunk.t1_s - chunk.t0_s) * 0.5;
        orbitsim::State start_state{};
        orbitsim::State mid_state{};
        orbitsim::State end_state{};
        if (!sample_activity_segment_state(*baseline_segments, chunk.t0_s, start_state) ||
            !sample_activity_segment_state(*baseline_segments, mid_t_s, mid_state) ||
            !sample_activity_segment_state(*baseline_segments, chunk.t1_s, end_state))
        {
            return probe;
        }

        const double dt0_s = std::max(kContinuityMinTimeEpsilonS, mid_t_s - chunk.t0_s);
        const double dt1_s = std::max(kContinuityMinTimeEpsilonS, chunk.t1_s - mid_t_s);
        const orbitsim::Vec3 accel0_mps2 = (mid_state.velocity_mps - start_state.velocity_mps) / dt0_s;
        const orbitsim::Vec3 accel1_mps2 = (end_state.velocity_mps - mid_state.velocity_mps) / dt1_s;
        const double avg_dt_s = std::max(kContinuityMinTimeEpsilonS, (dt0_s + dt1_s) * 0.5);
        const double accel_mag_mps2 =
                std::max(glm::length(glm::dvec3(accel0_mps2)), glm::length(glm::dvec3(accel1_mps2)));
        const double jerk_mag_mps3 = glm::length(glm::dvec3(accel1_mps2 - accel0_mps2)) / avg_dt_s;
        const double speed_scale_mps = std::max(
                {1.0,
                 glm::length(glm::dvec3(start_state.velocity_mps)),
                 glm::length(glm::dvec3(mid_state.velocity_mps)),
                 glm::length(glm::dvec3(end_state.velocity_mps))});
        const double accel_scale_mps2 = std::max(1.0e-6, accel_mag_mps2);
        const double normalized_jerk = (jerk_mag_mps3 * avg_dt_s) / accel_scale_mps2;

        probe.valid = true;
        probe.heading_change_rad = safe_angle_between(start_state.velocity_mps, mid_state.velocity_mps) +
                                   safe_angle_between(mid_state.velocity_mps, end_state.velocity_mps);
        probe.accel_magnitude_mps2 = accel_mag_mps2;
        probe.jerk_magnitude_mps3 = jerk_mag_mps3;
        probe.dominant_gravity_ratio =
                dominant_gravity_ratio_for_state(request, shared_ephemeris, mid_state, mid_t_s);

        probe.primary_body_id_start = select_primary_body_id_for_state(
                request,
                shared_ephemeris,
                start_state,
                chunk.t0_s,
                request.preferred_primary_body_id);
        probe.primary_body_id_mid = select_primary_body_id_for_state(
                request,
                shared_ephemeris,
                mid_state,
                mid_t_s,
                probe.primary_body_id_start);
        probe.primary_body_id_end = select_primary_body_id_for_state(
                request,
                shared_ephemeris,
                end_state,
                chunk.t1_s,
                probe.primary_body_id_mid);

        if (chunk.profile_id == PredictionProfileId::Exact ||
            chunk.profile_id == PredictionProfileId::Near)
        {
            return probe;
        }

        for (const OrbitPredictionService::ManeuverImpulse &impulse : request.maneuver_impulses)
        {
            if (!std::isfinite(impulse.t_s))
            {
                continue;
            }

            const double distance_s = std::min(std::abs(impulse.t_s - chunk.t0_s), std::abs(impulse.t_s - chunk.t1_s));
            probe.maneuver_proximity_s = std::min(probe.maneuver_proximity_s, distance_s);
        }

        uint32_t promote_steps = 0u;
        if (probe.heading_change_rad >= OrbitPredictionTuning::kPredictionActivityProbeHeadingPromoteRad)
        {
            promote_steps = std::max(promote_steps, 1u);
        }
        if (probe.heading_change_rad >= OrbitPredictionTuning::kPredictionActivityProbeHeadingSplitRad)
        {
            promote_steps = std::max(promote_steps, 2u);
            probe.should_split = true;
        }

        if (normalized_jerk >= OrbitPredictionTuning::kPredictionActivityProbeNormalizedJerkPromote)
        {
            promote_steps = std::max(promote_steps, 1u);
        }
        if (normalized_jerk >= OrbitPredictionTuning::kPredictionActivityProbeNormalizedJerkSplit)
        {
            promote_steps = std::max(promote_steps, 2u);
            probe.should_split = true;
        }

        if (probe.dominant_gravity_ratio <= OrbitPredictionTuning::kPredictionActivityProbeDominantGravityPromoteRatio)
        {
            promote_steps = std::max(promote_steps, 1u);
        }
        if (probe.dominant_gravity_ratio <= OrbitPredictionTuning::kPredictionActivityProbeDominantGravitySplitRatio)
        {
            promote_steps = std::max(promote_steps, 2u);
            probe.should_split = true;
        }

        if (probe.primary_body_id_start != orbitsim::kInvalidBodyId &&
            ((probe.primary_body_id_start != probe.primary_body_id_mid) ||
             (probe.primary_body_id_mid != probe.primary_body_id_end)))
        {
            promote_steps = std::max(promote_steps, 2u);
            probe.should_split = true;
        }

        if (std::isfinite(probe.maneuver_proximity_s) &&
            probe.maneuver_proximity_s <= OrbitPredictionTuning::kPredictionChunkSpanNearS)
        {
            promote_steps = std::max(promote_steps, 1u);
        }

        if (speed_scale_mps <= 1.0 && accel_mag_mps2 <= 1.0e-9)
        {
            probe.should_split = false;
        }

        probe.recommended_profile_id = promote_prediction_profile(chunk.profile_id, promote_steps);
        return probe;
    }

    OrbitPredictionService::PredictionProfileDefinition resolve_prediction_profile_definition(
            const OrbitPredictionService::Request &request,
            const OrbitPredictionService::PredictionChunkPlan &chunk)
    {
        OrbitPredictionService::PredictionProfileDefinition def{};
        def.profile_id = chunk.profile_id;
        const double elapsed_s = std::max(0.0, chunk.t0_s - request.sim_time_s);
        const bool deep_tail_window = elapsed_s >= OrbitPredictionTuning::kPredictionChunkBandCruiseEndS;

        const bool preview_sensitive = chunk.profile_id == PredictionProfileId::Exact;
        const bool maneuver_sensitive =
                chunk_has_any_boundary(chunk, {PredictionChunkBoundaryFlags::Maneuver});
        const bool control_sensitive =
                request_needs_control_sensitive_prediction(request) || preview_sensitive || maneuver_sensitive;

        const double base_segment_min_dt_s = control_sensitive
                                                     ? OrbitPredictionTuning::kAdaptiveSegmentMinDtControlledS
                                                     : OrbitPredictionTuning::kAdaptiveSegmentMinDtS;
        double base_segment_max_dt_s = OrbitPredictionTuning::kAdaptiveSegmentMaxDtS;
        if (control_sensitive)
        {
            base_segment_max_dt_s = std::min(base_segment_max_dt_s,
                                             OrbitPredictionTuning::kAdaptiveSegmentMaxDtControlledS);
        }
        const double base_lookup_max_dt_s = control_sensitive
                                                    ? OrbitPredictionTuning::kAdaptiveSegmentLookupMaxDtControlledS
                                                    : OrbitPredictionTuning::kAdaptiveSegmentLookupMaxDtS;
        const std::size_t base_segment_soft_cap =
                OrbitPredictionTuning::kAdaptiveSegmentSoftMaxSegmentsNormal;

        const double base_ephemeris_min_dt_s = control_sensitive
                                                       ? OrbitPredictionTuning::kAdaptiveEphemerisMinDtControlledS
                                                       : OrbitPredictionTuning::kAdaptiveEphemerisMinDtS;
        double base_ephemeris_max_dt_s = OrbitPredictionTuning::kAdaptiveEphemerisMaxDtS;
        if (control_sensitive)
        {
            base_ephemeris_max_dt_s = std::min(base_ephemeris_max_dt_s,
                                               OrbitPredictionTuning::kAdaptiveEphemerisMaxDtControlledS);
        }
        if (request.lagrange_sensitive)
        {
            base_ephemeris_max_dt_s = std::min(base_ephemeris_max_dt_s,
                                               OrbitPredictionTuning::kLagrangeEphemerisMaxDtS);
        }
        const std::size_t base_ephemeris_soft_cap =
                OrbitPredictionTuning::kAdaptiveEphemerisSoftMaxSegments;

        double segment_dt_scale = 1.0;
        double lookup_dt_scale = 1.0;
        double segment_soft_cap_scale = 1.0;
        double ephemeris_dt_scale = 1.0;
        double ephemeris_soft_cap_scale = 1.0;
        double tolerance_scale = 1.0;
        double output_sample_density_scale = 1.0;
        double seam_overlap_s = 0.0;

        switch (chunk.profile_id)
        {
        case PredictionProfileId::Exact:
            segment_dt_scale = 0.25;
            lookup_dt_scale = 0.25;
            segment_soft_cap_scale = 1.75;
            ephemeris_dt_scale = 0.25;
            ephemeris_soft_cap_scale = 1.5;
            tolerance_scale = 0.25;
            output_sample_density_scale = 2.0;
            seam_overlap_s = 2.0 * OrbitPredictionTuning::kSecondsPerMinute;
            break;
        case PredictionProfileId::Near:
            segment_dt_scale = 0.75;
            lookup_dt_scale = 0.75;
            segment_soft_cap_scale = 1.15;
            ephemeris_dt_scale = 0.75;
            ephemeris_soft_cap_scale = 1.1;
            tolerance_scale = 0.75;
            output_sample_density_scale = 1.0;
            seam_overlap_s = 5.0 * OrbitPredictionTuning::kSecondsPerMinute;
            break;
        case PredictionProfileId::Tail:
            segment_dt_scale = deep_tail_window ? 2.5 : 1.25;
            lookup_dt_scale = deep_tail_window ? 2.5 : 1.25;
            segment_soft_cap_scale = deep_tail_window ? 0.3 : 0.7;
            ephemeris_dt_scale = deep_tail_window ? 2.5 : 1.25;
            ephemeris_soft_cap_scale = deep_tail_window ? 0.35 : 0.75;
            tolerance_scale = deep_tail_window ? 2.5 : 1.25;
            output_sample_density_scale = deep_tail_window ? 0.2 : 0.55;
            seam_overlap_s = deep_tail_window ? (1.0 * OrbitPredictionTuning::kSecondsPerDay)
                                              : (2.0 * OrbitPredictionTuning::kSecondsPerHour);
            break;
        }

        if (maneuver_sensitive)
        {
            segment_dt_scale = std::min(segment_dt_scale, 0.75);
            lookup_dt_scale = std::min(lookup_dt_scale, 0.75);
            ephemeris_dt_scale = std::min(ephemeris_dt_scale, 0.75);
            tolerance_scale = std::min(tolerance_scale, 0.75);
            segment_soft_cap_scale = std::max(segment_soft_cap_scale, 1.0);
            ephemeris_soft_cap_scale = std::max(ephemeris_soft_cap_scale, 1.0);
            output_sample_density_scale = std::max(output_sample_density_scale, 1.0);
        }

        if (preview_sensitive)
        {
            segment_dt_scale = std::min(segment_dt_scale, 0.5);
            lookup_dt_scale = std::min(lookup_dt_scale, 0.5);
            ephemeris_dt_scale = std::min(ephemeris_dt_scale, 0.5);
            tolerance_scale = std::min(tolerance_scale, 0.5);
            segment_soft_cap_scale = std::max(segment_soft_cap_scale, 1.5);
            ephemeris_soft_cap_scale = std::max(ephemeris_soft_cap_scale, 1.25);
            output_sample_density_scale = std::max(output_sample_density_scale, 1.5);
            seam_overlap_s = std::max(seam_overlap_s, 60.0);
        }

        def.integrator_tolerance_multiplier = tolerance_scale;
        def.min_dt_s = scale_dt_value(base_segment_min_dt_s, std::min(segment_dt_scale, 1.0), 1.0e-3);
        def.max_dt_s = scale_dt_value(base_segment_max_dt_s, segment_dt_scale, def.min_dt_s);
        def.lookup_max_dt_s = std::clamp(scale_dt_value(base_lookup_max_dt_s,
                                                        lookup_dt_scale,
                                                        def.min_dt_s),
                                         def.min_dt_s,
                                         def.max_dt_s);
        def.soft_max_segments = scale_segment_cap(base_segment_soft_cap, segment_soft_cap_scale, 32u);
        def.ephemeris_min_dt_s =
                scale_dt_value(base_ephemeris_min_dt_s, std::min(ephemeris_dt_scale, 1.0), 1.0e-3);
        def.ephemeris_max_dt_s =
                scale_dt_value(base_ephemeris_max_dt_s, ephemeris_dt_scale, def.ephemeris_min_dt_s);
        def.ephemeris_soft_max_segments =
                scale_segment_cap(base_ephemeris_soft_cap, ephemeris_soft_cap_scale, 32u);
        def.output_sample_density_scale = output_sample_density_scale;
        def.seam_overlap_s = seam_overlap_s;
        return def;
    }
} // namespace Game
