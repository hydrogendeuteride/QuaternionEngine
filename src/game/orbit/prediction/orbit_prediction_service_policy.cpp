#include "game/orbit/prediction/orbit_prediction_service_internal.h"

#include "orbitsim/detail/spacecraft_propagation.hpp"
#include "orbitsim/integrators.hpp"
#include "orbitsim/trajectories.hpp"

namespace Game
{
    void apply_lagrange_integrator_profile(orbitsim::GameSimulation::Config &sim_config)
    {
        orbitsim::DOPRI5Options &integrator = sim_config.spacecraft_integrator;
        if (!(integrator.max_step_s > 0.0) || integrator.max_step_s > OrbitPredictionTuning::kLagrangeIntegratorMaxStepS)
        {
            integrator.max_step_s = OrbitPredictionTuning::kLagrangeIntegratorMaxStepS;
        }
        integrator.abs_tol = std::min(integrator.abs_tol, OrbitPredictionTuning::kLagrangeIntegratorAbsTol);
        integrator.rel_tol = std::min(integrator.rel_tol, OrbitPredictionTuning::kLagrangeIntegratorRelTol);
    }

    std::size_t count_future_maneuver_impulses(const OrbitPredictionService::Request &request)
    {
        return static_cast<std::size_t>(std::count_if(
                request.maneuver_impulses.begin(),
                request.maneuver_impulses.end(),
                [&request](const OrbitPredictionService::ManeuverImpulse &impulse) {
                    return std::isfinite(impulse.t_s) && impulse.t_s >= request.sim_time_s;
                }));
    }

    bool request_needs_control_sensitive_prediction(const OrbitPredictionService::Request &request)
    {
        return request.thrusting || count_future_maneuver_impulses(request) > 0;
    }

    double resolve_prediction_integrator_max_step_s(const OrbitPredictionService::Request &request,
                                                    const double /*resolved_horizon_s*/)
    {
        if (request_needs_control_sensitive_prediction(request))
        {
            if (request_uses_fast_preview(request))
            {
                return std::min(OrbitPredictionTuning::kPredictionIntegratorMaxStepS,
                                OrbitPredictionTuning::kPredictionIntegratorMaxStepPreviewS);
            }

            return OrbitPredictionTuning::kPredictionIntegratorMaxStepControlledS;
        }

        return OrbitPredictionTuning::kPredictionIntegratorMaxStepS;
    }

    void apply_prediction_integrator_profile(orbitsim::GameSimulation::Config &sim_config,
                                             const OrbitPredictionService::Request &request,
                                             const double resolved_horizon_s)
    {
        orbitsim::DOPRI5Options &integrator = sim_config.spacecraft_integrator;
        const double max_step_s = resolve_prediction_integrator_max_step_s(request, resolved_horizon_s);
        if (!(integrator.max_step_s > 0.0) || integrator.max_step_s > max_step_s)
        {
            integrator.max_step_s = max_step_s;
        }

        integrator.max_substeps =
                std::max(integrator.max_substeps, OrbitPredictionTuning::kPredictionIntegratorMaxSubstepsSoft);
        integrator.max_substeps_hard =
                std::max({integrator.max_substeps_hard,
                          integrator.max_substeps,
                          OrbitPredictionTuning::kPredictionIntegratorMaxSubstepsHard});
        integrator.max_interval_splits =
                std::max(integrator.max_interval_splits, OrbitPredictionTuning::kPredictionIntegratorMaxIntervalSplits);
    }

    namespace
    {
        using PredictionChunkBoundaryFlags = OrbitPredictionService::PredictionChunkBoundaryFlags;
        using PredictionChunkPlan = OrbitPredictionService::PredictionChunkPlan;
        using PredictionProfileId = OrbitPredictionService::PredictionProfileId;
        using PredictionSolvePlan = OrbitPredictionService::PredictionSolvePlan;

        struct PlannerBoundaryPoint
        {
            double time_s{0.0};
            uint32_t flags{0u};
        };

        [[nodiscard]] uint32_t planner_boundary_flag_bits(const PredictionChunkBoundaryFlags flag)
        {
            return static_cast<uint32_t>(flag);
        }

        [[nodiscard]] bool planner_boundary_has_flag(const uint32_t flags, const PredictionChunkBoundaryFlags flag)
        {
            return (flags & planner_boundary_flag_bits(flag)) != 0u;
        }

        void append_planner_boundary(std::vector<PlannerBoundaryPoint> &boundaries,
                                     const double request_t0_s,
                                     const double request_t1_s,
                                     const double time_s,
                                     const PredictionChunkBoundaryFlags flag)
        {
            if (!std::isfinite(time_s) || !std::isfinite(request_t0_s) || !std::isfinite(request_t1_s))
            {
                return;
            }

            const double time_epsilon_s = continuity_time_epsilon_s(time_s);
            if (time_s < (request_t0_s - time_epsilon_s) || time_s > (request_t1_s + time_epsilon_s))
            {
                return;
            }

            boundaries.push_back(PlannerBoundaryPoint{
                    .time_s = std::clamp(time_s, request_t0_s, request_t1_s),
                    .flags = planner_boundary_flag_bits(flag),
            });
        }

        void append_time_band_boundaries(std::vector<PlannerBoundaryPoint> &boundaries,
                                         const double request_t0_s,
                                         const double request_t1_s,
                                         const double band_begin_offset_s,
                                         const double band_end_offset_s,
                                         const double chunk_span_s)
        {
            if (!(chunk_span_s > 0.0) || !(band_end_offset_s > band_begin_offset_s))
            {
                return;
            }

            const double band_t0_s = request_t0_s + std::max(0.0, band_begin_offset_s);
            const double band_t1_s = request_t0_s + std::max(0.0, band_end_offset_s);
            if (!(band_t1_s > band_t0_s))
            {
                return;
            }

            const double clamped_band_t0_s = std::clamp(band_t0_s, request_t0_s, request_t1_s);
            const double clamped_band_t1_s = std::clamp(band_t1_s, request_t0_s, request_t1_s);
            if (!(clamped_band_t1_s > clamped_band_t0_s))
            {
                return;
            }

            append_planner_boundary(boundaries,
                                    request_t0_s,
                                    request_t1_s,
                                    clamped_band_t0_s,
                                    PredictionChunkBoundaryFlags::TimeBand);
            append_planner_boundary(boundaries,
                                    request_t0_s,
                                    request_t1_s,
                                    clamped_band_t1_s,
                                    PredictionChunkBoundaryFlags::TimeBand);

            for (double boundary_s = band_t0_s + chunk_span_s; boundary_s < band_t1_s; boundary_s += chunk_span_s)
            {
                append_planner_boundary(boundaries,
                                        request_t0_s,
                                        request_t1_s,
                                        boundary_s,
                                        PredictionChunkBoundaryFlags::TimeBand);
            }
        }

        [[nodiscard]] std::vector<PlannerBoundaryPoint> merge_planner_boundaries(std::vector<PlannerBoundaryPoint> boundaries)
        {
            if (boundaries.empty())
            {
                return boundaries;
            }

            std::sort(boundaries.begin(),
                      boundaries.end(),
                      [](const PlannerBoundaryPoint &a, const PlannerBoundaryPoint &b) { return a.time_s < b.time_s; });

            std::vector<PlannerBoundaryPoint> merged;
            merged.reserve(boundaries.size());
            for (const PlannerBoundaryPoint &boundary : boundaries)
            {
                if (!std::isfinite(boundary.time_s))
                {
                    continue;
                }

                if (!merged.empty())
                {
                    const double merge_epsilon_s = continuity_time_epsilon_s(boundary.time_s);
                    if (std::abs(boundary.time_s - merged.back().time_s) <= merge_epsilon_s)
                    {
                        merged.back().flags |= boundary.flags;
                        continue;
                    }
                }

                merged.push_back(boundary);
            }

            return merged;
        }

        [[nodiscard]] PredictionProfileId resolve_chunk_profile_id(const OrbitPredictionService::Request &request,
                                                                   const double chunk_t0_s,
                                                                   const double chunk_t1_s)
        {
            if (request_uses_preview_patch(request))
            {
                const double preview_t0_s = request.preview_patch.anchor_time_s;
                const double preview_t1_s = preview_t0_s + preview_fp0_window_s(request);
                if (preview_t1_s > preview_t0_s &&
                    chunk_t0_s < preview_t1_s &&
                    chunk_t1_s > preview_t0_s)
                {
                    return PredictionProfileId::InteractiveExact;
                }
            }

            const double elapsed_s = std::max(0.0, chunk_t0_s - request.sim_time_s);
            if (elapsed_s < OrbitPredictionTuning::kPredictionChunkBandNearEndS)
            {
                return PredictionProfileId::NearBody;
            }
            if (elapsed_s < OrbitPredictionTuning::kPredictionChunkBandTransferEndS)
            {
                return PredictionProfileId::Transfer;
            }
            if (elapsed_s < OrbitPredictionTuning::kPredictionChunkBandCruiseEndS)
            {
                return PredictionProfileId::Cruise;
            }
            return PredictionProfileId::DeepTail;
        }

        [[nodiscard]] uint32_t resolve_chunk_priority(const PredictionProfileId profile_id,
                                                      const uint32_t boundary_flags)
        {
            if (profile_id == PredictionProfileId::InteractiveExact)
            {
                return 0u;
            }
            if (planner_boundary_has_flag(boundary_flags, PredictionChunkBoundaryFlags::Maneuver) ||
                planner_boundary_has_flag(boundary_flags, PredictionChunkBoundaryFlags::PreviewAnchor) ||
                planner_boundary_has_flag(boundary_flags, PredictionChunkBoundaryFlags::PreviewChunk))
            {
                return 1u;
            }
            return static_cast<uint32_t>(profile_id) + 1u;
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

        [[nodiscard]] orbitsim::AdaptiveToleranceRamp scale_tolerance_ramp(
                orbitsim::AdaptiveToleranceRamp ramp,
                const double scale)
        {
            const double safe_scale = std::isfinite(scale) ? std::max(scale, 1.0e-6) : 1.0;
            ramp.pos_near_m *= safe_scale;
            ramp.pos_far_m *= safe_scale;
            ramp.vel_near_mps *= safe_scale;
            ramp.vel_far_mps *= safe_scale;
            return ramp;
        }

        [[nodiscard]] double chunk_duration_s(const PredictionChunkPlan &chunk)
        {
            return (std::isfinite(chunk.t0_s) && std::isfinite(chunk.t1_s) && chunk.t1_s > chunk.t0_s)
                           ? (chunk.t1_s - chunk.t0_s)
                           : 0.0;
        }

        [[nodiscard]] bool chunk_has_any_boundary(const PredictionChunkPlan &chunk,
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

        orbitsim::AdaptiveToleranceRamp make_segment_tolerance_ramp(const OrbitPredictionService::Request &request)
        {
            orbitsim::AdaptiveToleranceRamp ramp{};
            ramp.pos_near_m = OrbitPredictionTuning::kAdaptiveSegmentPosTolNearM;
            ramp.pos_far_m = OrbitPredictionTuning::kAdaptiveSegmentPosTolFarM;
            ramp.vel_near_mps = OrbitPredictionTuning::kAdaptiveSegmentVelTolNearMps;
            ramp.vel_far_mps = OrbitPredictionTuning::kAdaptiveSegmentVelTolFarMps;
            ramp.rel_pos_floor = OrbitPredictionTuning::kAdaptiveSegmentRelPosFloor;
            ramp.rel_vel_floor = OrbitPredictionTuning::kAdaptiveSegmentRelVelFloor;

            if (request_needs_control_sensitive_prediction(request))
            {
                ramp.pos_near_m = std::min(ramp.pos_near_m, 0.25);
                ramp.pos_far_m = std::min(ramp.pos_far_m, 2'500.0);
                ramp.vel_near_mps = std::min(ramp.vel_near_mps, 2.5e-4);
                ramp.vel_far_mps = std::min(ramp.vel_far_mps, 2.5);
            }

            if (request.lagrange_sensitive)
            {
                ramp.pos_near_m *= 0.5;
                ramp.pos_far_m *= 0.25;
                ramp.vel_near_mps *= 0.5;
                ramp.vel_far_mps *= 0.25;
            }

            if (request_uses_fast_preview(request))
            {
                ramp.pos_near_m *= OrbitPredictionTuning::kFastPreviewAdaptiveSegmentToleranceScale;
                ramp.pos_far_m *= OrbitPredictionTuning::kFastPreviewAdaptiveSegmentToleranceScale;
                ramp.vel_near_mps *= OrbitPredictionTuning::kFastPreviewAdaptiveSegmentToleranceScale;
                ramp.vel_far_mps *= OrbitPredictionTuning::kFastPreviewAdaptiveSegmentToleranceScale;
            }

            return ramp;
        }

        orbitsim::AdaptiveToleranceRamp make_ephemeris_tolerance_ramp(const OrbitPredictionService::Request &request)
        {
            orbitsim::AdaptiveToleranceRamp ramp{};
            ramp.pos_near_m = OrbitPredictionTuning::kAdaptiveEphemerisPosTolNearM;
            ramp.pos_far_m = OrbitPredictionTuning::kAdaptiveEphemerisPosTolFarM;
            ramp.vel_near_mps = OrbitPredictionTuning::kAdaptiveEphemerisVelTolNearMps;
            ramp.vel_far_mps = OrbitPredictionTuning::kAdaptiveEphemerisVelTolFarMps;
            ramp.rel_pos_floor = OrbitPredictionTuning::kAdaptiveEphemerisRelPosFloor;
            ramp.rel_vel_floor = OrbitPredictionTuning::kAdaptiveEphemerisRelVelFloor;

            if (request_needs_control_sensitive_prediction(request))
            {
                ramp.pos_near_m = std::min(ramp.pos_near_m, 0.5);
                ramp.pos_far_m = std::min(ramp.pos_far_m, 250.0);
                ramp.vel_near_mps = std::min(ramp.vel_near_mps, 5.0e-5);
                ramp.vel_far_mps = std::min(ramp.vel_far_mps, 0.5);
            }

            if (request.lagrange_sensitive)
            {
                ramp.pos_near_m *= 0.5;
                ramp.pos_far_m *= 0.5;
                ramp.vel_near_mps *= 0.5;
                ramp.vel_far_mps *= 0.5;
            }

            if (request_uses_fast_preview(request))
            {
                ramp.pos_near_m *= OrbitPredictionTuning::kFastPreviewAdaptiveEphemerisToleranceScale;
                ramp.pos_far_m *= OrbitPredictionTuning::kFastPreviewAdaptiveEphemerisToleranceScale;
                ramp.vel_near_mps *= OrbitPredictionTuning::kFastPreviewAdaptiveEphemerisToleranceScale;
                ramp.vel_far_mps *= OrbitPredictionTuning::kFastPreviewAdaptiveEphemerisToleranceScale;
            }

            return ramp;
        }

        bool tolerance_covers_request(const orbitsim::AdaptiveToleranceRamp &entry,
                                      const orbitsim::AdaptiveToleranceRamp &request)
        {
            constexpr double kTolEpsilon = 1.0e-12;
            return entry.pos_near_m <= (request.pos_near_m + kTolEpsilon) &&
                   entry.pos_far_m <= (request.pos_far_m + kTolEpsilon) &&
                   entry.vel_near_mps <= (request.vel_near_mps + kTolEpsilon) &&
                   entry.vel_far_mps <= (request.vel_far_mps + kTolEpsilon) &&
                   entry.rel_pos_floor <= (request.rel_pos_floor + kTolEpsilon) &&
                   entry.rel_vel_floor <= (request.rel_vel_floor + kTolEpsilon);
        }

        bool adaptive_ephemeris_options_cover_request(const orbitsim::AdaptiveEphemerisOptions &entry,
                                                      const orbitsim::AdaptiveEphemerisOptions &request)
        {
            return entry.min_dt_s <= (request.min_dt_s + kEphemerisDtEpsilonS) &&
                   entry.max_dt_s <= (request.max_dt_s + kEphemerisDtEpsilonS) &&
                   entry.soft_max_segments >= request.soft_max_segments &&
                   entry.hard_max_segments >= request.hard_max_segments &&
                   tolerance_covers_request(entry.tolerance, request.tolerance);
        }

        bool same_config_for_ephemeris(const orbitsim::GameSimulation::Config &a,
                                       const orbitsim::GameSimulation::Config &b)
        {
            return a.gravitational_constant == b.gravitational_constant &&
                   a.softening_length_m == b.softening_length_m;
        }

        bool same_spin_state(const orbitsim::SpinState &a, const orbitsim::SpinState &b)
        {
            return a.axis == b.axis &&
                   a.angle_rad == b.angle_rad &&
                   a.rate_rad_per_s == b.rate_rad_per_s;
        }

        bool same_state_for_ephemeris(const orbitsim::State &a, const orbitsim::State &b)
        {
            return a.position_m == b.position_m &&
                   a.velocity_mps == b.velocity_mps &&
                   same_spin_state(a.spin, b.spin);
        }

        bool same_massive_body_for_ephemeris(const orbitsim::MassiveBody &a, const orbitsim::MassiveBody &b)
        {
            return a.id == b.id &&
                   a.mass_kg == b.mass_kg &&
                   same_state_for_ephemeris(a.state, b.state);
        }

        bool same_massive_body_set_for_ephemeris(const std::vector<orbitsim::MassiveBody> &a,
                                                 const std::vector<orbitsim::MassiveBody> &b)
        {
            if (a.size() != b.size())
            {
                return false;
            }

            for (std::size_t i = 0; i < a.size(); ++i)
            {
                if (!same_massive_body_for_ephemeris(a[i], b[i]))
                {
                    return false;
                }
            }

            return true;
        }
    } // namespace

    PredictionSolvePlan build_prediction_solve_plan(const OrbitPredictionService::Request &request)
    {
        PredictionSolvePlan plan{};
        if (!std::isfinite(request.sim_time_s))
        {
            return plan;
        }

        const double request_t0_s = request.sim_time_s;
        const double request_t1_s = request_end_time_s(request);
        const double requested_window_s =
                std::max(0.0, std::isfinite(request.future_window_s) ? request.future_window_s : 0.0);
        if (!(request_t1_s > request_t0_s))
        {
            return plan;
        }

        std::vector<PlannerBoundaryPoint> boundaries;
        boundaries.reserve(request.maneuver_impulses.size() + 16u);
        append_planner_boundary(boundaries,
                                request_t0_s,
                                request_t1_s,
                                request_t0_s,
                                PredictionChunkBoundaryFlags::RequestStart);
        append_planner_boundary(boundaries,
                                request_t0_s,
                                request_t1_s,
                                request_t1_s,
                                PredictionChunkBoundaryFlags::RequestEnd);

        for (const OrbitPredictionService::ManeuverImpulse &impulse : request.maneuver_impulses)
        {
            append_planner_boundary(boundaries,
                                    request_t0_s,
                                    request_t1_s,
                                    impulse.t_s,
                                    PredictionChunkBoundaryFlags::Maneuver);
        }

        if (request_uses_preview_patch(request))
        {
            append_planner_boundary(boundaries,
                                    request_t0_s,
                                    request_t1_s,
                                    request.preview_patch.anchor_time_s,
                                    PredictionChunkBoundaryFlags::PreviewAnchor);

            const double preview_chunk_span_s = std::max(0.0, request.preview_patch.patch_window_s);
            if (preview_chunk_span_s > 0.0)
            {
                append_planner_boundary(boundaries,
                                        request_t0_s,
                                        request_t1_s,
                                        request.preview_patch.anchor_time_s + preview_chunk_span_s,
                                        PredictionChunkBoundaryFlags::PreviewChunk);
                append_planner_boundary(boundaries,
                                        request_t0_s,
                                        request_t1_s,
                                        request.preview_patch.anchor_time_s + preview_chunk_span_s * 2.0,
                                        PredictionChunkBoundaryFlags::PreviewChunk);
            }
        }

        append_time_band_boundaries(boundaries,
                                    request_t0_s,
                                    request_t1_s,
                                    0.0,
                                    OrbitPredictionTuning::kPredictionChunkBandNearEndS,
                                    OrbitPredictionTuning::kPredictionChunkSpanNearS);
        append_time_band_boundaries(boundaries,
                                    request_t0_s,
                                    request_t1_s,
                                    OrbitPredictionTuning::kPredictionChunkBandNearEndS,
                                    OrbitPredictionTuning::kPredictionChunkBandTransferEndS,
                                    OrbitPredictionTuning::kPredictionChunkSpanTransferS);
        append_time_band_boundaries(boundaries,
                                    request_t0_s,
                                    request_t1_s,
                                    OrbitPredictionTuning::kPredictionChunkBandTransferEndS,
                                    OrbitPredictionTuning::kPredictionChunkBandCruiseFineEndS,
                                    OrbitPredictionTuning::kPredictionChunkSpanCruiseFineS);
        append_time_band_boundaries(boundaries,
                                    request_t0_s,
                                    request_t1_s,
                                    OrbitPredictionTuning::kPredictionChunkBandCruiseFineEndS,
                                    OrbitPredictionTuning::kPredictionChunkBandCruiseEndS,
                                    OrbitPredictionTuning::kPredictionChunkSpanCruiseS);
        append_time_band_boundaries(boundaries,
                                    request_t0_s,
                                    request_t1_s,
                                    OrbitPredictionTuning::kPredictionChunkBandCruiseEndS,
                                    std::max(requested_window_s,
                                             OrbitPredictionTuning::kPredictionChunkBandDeepTailEndS),
                                    OrbitPredictionTuning::kPredictionChunkSpanDeepTailS);

        const std::vector<PlannerBoundaryPoint> merged_boundaries = merge_planner_boundaries(std::move(boundaries));
        if (merged_boundaries.size() < 2)
        {
            return plan;
        }

        plan.t0_s = request_t0_s;
        plan.t1_s = request_t1_s;
        plan.chunks.reserve(merged_boundaries.size() - 1u);
        for (std::size_t i = 0; (i + 1u) < merged_boundaries.size(); ++i)
        {
            const double chunk_t0_s = merged_boundaries[i].time_s;
            const double chunk_t1_s = merged_boundaries[i + 1u].time_s;
            if (!(chunk_t1_s > chunk_t0_s))
            {
                continue;
            }

            const uint32_t boundary_flags = merged_boundaries[i].flags | merged_boundaries[i + 1u].flags;
            const PredictionProfileId profile_id = resolve_chunk_profile_id(request, chunk_t0_s, chunk_t1_s);
            plan.chunks.push_back(PredictionChunkPlan{
                    .chunk_id = static_cast<uint32_t>(plan.chunks.size()),
                    .t0_s = chunk_t0_s,
                    .t1_s = chunk_t1_s,
                    .profile_id = profile_id,
                    .boundary_flags = boundary_flags,
                    .priority = resolve_chunk_priority(profile_id, boundary_flags),
                    .allow_reuse = profile_id != PredictionProfileId::InteractiveExact,
                    .requires_seam_validation = !plan.chunks.empty(),
            });
        }

        plan.valid = !plan.chunks.empty();
        return plan;
    }

    OrbitPredictionService::PredictionProfileDefinition resolve_prediction_profile_definition(
            const OrbitPredictionService::Request &request,
            const OrbitPredictionService::PredictionChunkPlan &chunk)
    {
        OrbitPredictionService::PredictionProfileDefinition def{};
        def.profile_id = chunk.profile_id;

        const bool fast_preview = request_uses_fast_preview(request);
        const bool preview_sensitive =
                chunk.profile_id == PredictionProfileId::InteractiveExact ||
                chunk_has_any_boundary(chunk,
                                       {PredictionChunkBoundaryFlags::PreviewAnchor,
                                        PredictionChunkBoundaryFlags::PreviewChunk});
        const bool maneuver_sensitive =
                chunk_has_any_boundary(chunk, {PredictionChunkBoundaryFlags::Maneuver});
        const bool control_sensitive =
                request_needs_control_sensitive_prediction(request) || preview_sensitive || maneuver_sensitive;

        const double base_segment_min_dt_s = control_sensitive
                                                     ? (fast_preview
                                                                ? OrbitPredictionTuning::kFastPreviewAdaptiveSegmentMinDtS
                                                                : OrbitPredictionTuning::kAdaptiveSegmentMinDtControlledS)
                                                     : OrbitPredictionTuning::kAdaptiveSegmentMinDtS;
        double base_segment_max_dt_s = OrbitPredictionTuning::kAdaptiveSegmentMaxDtS;
        if (control_sensitive)
        {
            base_segment_max_dt_s = std::min(base_segment_max_dt_s,
                                             fast_preview
                                                     ? OrbitPredictionTuning::kFastPreviewAdaptiveSegmentMaxDtS
                                                     : OrbitPredictionTuning::kAdaptiveSegmentMaxDtControlledS);
        }
        const double base_lookup_max_dt_s = control_sensitive
                                                    ? (fast_preview
                                                               ? OrbitPredictionTuning::kFastPreviewAdaptiveSegmentLookupMaxDtS
                                                               : OrbitPredictionTuning::kAdaptiveSegmentLookupMaxDtControlledS)
                                                    : OrbitPredictionTuning::kAdaptiveSegmentLookupMaxDtS;
        const std::size_t base_segment_soft_cap =
                fast_preview
                        ? OrbitPredictionTuning::kFastPreviewAdaptiveSegmentSoftMaxSegments
                        : OrbitPredictionTuning::kAdaptiveSegmentSoftMaxSegmentsNormal;

        const double base_ephemeris_min_dt_s = control_sensitive
                                                       ? (fast_preview
                                                                  ? OrbitPredictionTuning::kFastPreviewAdaptiveEphemerisMinDtS
                                                                  : OrbitPredictionTuning::kAdaptiveEphemerisMinDtControlledS)
                                                       : OrbitPredictionTuning::kAdaptiveEphemerisMinDtS;
        double base_ephemeris_max_dt_s = OrbitPredictionTuning::kAdaptiveEphemerisMaxDtS;
        if (control_sensitive)
        {
            base_ephemeris_max_dt_s = std::min(base_ephemeris_max_dt_s,
                                               fast_preview
                                                       ? OrbitPredictionTuning::kFastPreviewAdaptiveEphemerisMaxDtS
                                                       : OrbitPredictionTuning::kAdaptiveEphemerisMaxDtControlledS);
        }
        if (request.lagrange_sensitive)
        {
            base_ephemeris_max_dt_s = std::min(base_ephemeris_max_dt_s,
                                               OrbitPredictionTuning::kLagrangeEphemerisMaxDtS);
        }
        const std::size_t base_ephemeris_soft_cap =
                fast_preview
                        ? OrbitPredictionTuning::kFastPreviewAdaptiveEphemerisSoftMaxSegments
                        : OrbitPredictionTuning::kAdaptiveEphemerisSoftMaxSegments;

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
        case PredictionProfileId::InteractiveExact:
            segment_dt_scale = 0.25;
            lookup_dt_scale = 0.25;
            segment_soft_cap_scale = 1.75;
            ephemeris_dt_scale = 0.25;
            ephemeris_soft_cap_scale = 1.5;
            tolerance_scale = 0.25;
            output_sample_density_scale = 2.0;
            seam_overlap_s = 2.0 * OrbitPredictionTuning::kSecondsPerMinute;
            break;
        case PredictionProfileId::NearBody:
            segment_dt_scale = 0.75;
            lookup_dt_scale = 0.75;
            segment_soft_cap_scale = 1.15;
            ephemeris_dt_scale = 0.75;
            ephemeris_soft_cap_scale = 1.1;
            tolerance_scale = 0.75;
            output_sample_density_scale = 1.0;
            seam_overlap_s = 5.0 * OrbitPredictionTuning::kSecondsPerMinute;
            break;
        case PredictionProfileId::Transfer:
            segment_dt_scale = 1.0;
            lookup_dt_scale = 1.0;
            segment_soft_cap_scale = 0.85;
            ephemeris_dt_scale = 1.0;
            ephemeris_soft_cap_scale = 0.85;
            tolerance_scale = 1.0;
            output_sample_density_scale = 0.7;
            seam_overlap_s = 30.0 * OrbitPredictionTuning::kSecondsPerMinute;
            break;
        case PredictionProfileId::Cruise:
            segment_dt_scale = 1.5;
            lookup_dt_scale = 1.5;
            segment_soft_cap_scale = 0.55;
            ephemeris_dt_scale = 1.5;
            ephemeris_soft_cap_scale = 0.6;
            tolerance_scale = 1.5;
            output_sample_density_scale = 0.4;
            seam_overlap_s = 6.0 * OrbitPredictionTuning::kSecondsPerHour;
            break;
        case PredictionProfileId::DeepTail:
            segment_dt_scale = 2.5;
            lookup_dt_scale = 2.5;
            segment_soft_cap_scale = 0.3;
            ephemeris_dt_scale = 2.5;
            ephemeris_soft_cap_scale = 0.35;
            tolerance_scale = 2.5;
            output_sample_density_scale = 0.2;
            seam_overlap_s = 1.0 * OrbitPredictionTuning::kSecondsPerDay;
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

    std::size_t prediction_sample_budget_for_chunk(
            const OrbitPredictionService::Request &request,
            const OrbitPredictionService::PredictionChunkPlan &chunk,
            const std::size_t segment_count)
    {
        const OrbitPredictionService::PredictionProfileDefinition def =
                resolve_prediction_profile_definition(request, chunk);
        const std::size_t base_budget = prediction_sample_budget(request, segment_count);
        const double scaled_budget =
                std::ceil(static_cast<double>(base_budget) * std::max(0.05, def.output_sample_density_scale));
        const std::size_t max_budget = request_uses_fast_preview(request)
                                               ? OrbitPredictionTuning::kFastPreviewTrajectorySampleCap
                                               : 4'000u;
        return std::clamp<std::size_t>(static_cast<std::size_t>(std::max(2.0, scaled_budget)), 2u, max_budget);
    }

    orbitsim::AdaptiveSegmentOptions build_spacecraft_adaptive_segment_options(
            const OrbitPredictionService::Request &request,
            const OrbitPredictionService::EphemerisSamplingSpec &sampling_spec,
            const CancelCheck &cancel_requested)
    {
        orbitsim::AdaptiveSegmentOptions out{};
        if (!(sampling_spec.horizon_s > 0.0))
        {
            return out;
        }

        const bool controlled = request_needs_control_sensitive_prediction(request);

        const std::size_t soft_cap =
                request_uses_fast_preview(request)
                    ? OrbitPredictionTuning::kFastPreviewAdaptiveSegmentSoftMaxSegments
                    : OrbitPredictionTuning::kAdaptiveSegmentSoftMaxSegmentsNormal;
        std::size_t hard_cap =
                request_uses_fast_preview(request)
                    ? std::max<std::size_t>(soft_cap * 2u, soft_cap)
                    : OrbitPredictionTuning::kAdaptiveSegmentHardMaxSegmentsNormal;
        if (sampling_spec.horizon_s > 0.0)
        {
            const auto horizon_scaled = static_cast<std::size_t>(
                    std::ceil(sampling_spec.horizon_s / OrbitPredictionTuning::kAdaptiveSegmentMaxDtS));
            hard_cap = std::max(hard_cap, horizon_scaled);
        }

        double max_dt_s = OrbitPredictionTuning::kAdaptiveSegmentMaxDtS;
        if (controlled)
        {
            max_dt_s = std::min(max_dt_s,
                                request_uses_fast_preview(request)
                                    ? OrbitPredictionTuning::kFastPreviewAdaptiveSegmentMaxDtS
                                    : OrbitPredictionTuning::kAdaptiveSegmentMaxDtControlledS);
        }

        const double lookup_max_dt_s = controlled
                                               ? (request_uses_fast_preview(request)
                                                      ? OrbitPredictionTuning::kFastPreviewAdaptiveSegmentLookupMaxDtS
                                                      : OrbitPredictionTuning::kAdaptiveSegmentLookupMaxDtControlledS)
                                               : OrbitPredictionTuning::kAdaptiveSegmentLookupMaxDtS;

        out.duration_s = sampling_spec.horizon_s;
        out.min_dt_s = request_needs_control_sensitive_prediction(request)
                               ? (request_uses_fast_preview(request)
                                      ? OrbitPredictionTuning::kFastPreviewAdaptiveSegmentMinDtS
                                      : OrbitPredictionTuning::kAdaptiveSegmentMinDtControlledS)
                               : OrbitPredictionTuning::kAdaptiveSegmentMinDtS;
        out.max_dt_s = std::max(out.min_dt_s, max_dt_s);
        out.lookup_max_dt_s = std::clamp(lookup_max_dt_s, out.min_dt_s, out.max_dt_s);
        out.soft_max_segments = soft_cap;
        out.hard_max_segments = hard_cap;
        out.tolerance = make_segment_tolerance_ramp(request);
        out.stop_on_impact = false;
        out.split_at_impulses = true;
        out.split_at_burn_boundaries = true;
        out.cancel_requested = cancel_requested;
        return out;
    }

    orbitsim::AdaptiveSegmentOptions build_spacecraft_adaptive_segment_options_for_chunk(
            const OrbitPredictionService::Request &request,
            const OrbitPredictionService::PredictionChunkPlan &chunk,
            const CancelCheck &cancel_requested)
    {
        const double duration_s = chunk_duration_s(chunk);
        if (!(duration_s > 0.0))
        {
            return {};
        }

        const OrbitPredictionService::EphemerisSamplingSpec sampling_spec{
                .valid = true,
                .horizon_s = duration_s,
        };
        orbitsim::AdaptiveSegmentOptions out =
                build_spacecraft_adaptive_segment_options(request, sampling_spec, cancel_requested);
        if (!(out.duration_s > 0.0))
        {
            return out;
        }

        const OrbitPredictionService::PredictionProfileDefinition def =
                resolve_prediction_profile_definition(request, chunk);
        out.duration_s = duration_s;
        out.min_dt_s = def.min_dt_s;
        out.max_dt_s = std::max(out.min_dt_s, def.max_dt_s);
        out.lookup_max_dt_s = std::clamp(def.lookup_max_dt_s, out.min_dt_s, out.max_dt_s);
        out.soft_max_segments = def.soft_max_segments;
        out.hard_max_segments = std::max(out.hard_max_segments, out.soft_max_segments);
        out.tolerance = scale_tolerance_ramp(out.tolerance, def.integrator_tolerance_multiplier);
        return out;
    }

    orbitsim::AdaptiveEphemerisOptions build_adaptive_ephemeris_options(
            const OrbitPredictionService::Request &request,
            const OrbitPredictionService::EphemerisSamplingSpec &sampling_spec,
            const CancelCheck &cancel_requested)
    {
        orbitsim::AdaptiveEphemerisOptions out{};
        if (!(sampling_spec.horizon_s > 0.0))
        {
            return out;
        }

        const bool controlled = request_needs_control_sensitive_prediction(request);

        const std::size_t soft_cap =
                request_uses_fast_preview(request)
                    ? OrbitPredictionTuning::kFastPreviewAdaptiveEphemerisSoftMaxSegments
                    : OrbitPredictionTuning::kAdaptiveEphemerisSoftMaxSegments;
        const std::size_t hard_cap = std::max<std::size_t>(
                request.lagrange_sensitive ? OrbitPredictionTuning::kLagrangeEphemerisMaxSamples : soft_cap,
                soft_cap);

        out.duration_s = sampling_spec.horizon_s;
        out.min_dt_s = controlled
                               ? (request_uses_fast_preview(request)
                                      ? OrbitPredictionTuning::kFastPreviewAdaptiveEphemerisMinDtS
                                      : OrbitPredictionTuning::kAdaptiveEphemerisMinDtControlledS)
                               : OrbitPredictionTuning::kAdaptiveEphemerisMinDtS;

        double max_dt_s = OrbitPredictionTuning::kAdaptiveEphemerisMaxDtS;
        if (controlled)
        {
            max_dt_s = std::min(max_dt_s,
                                request_uses_fast_preview(request)
                                    ? OrbitPredictionTuning::kFastPreviewAdaptiveEphemerisMaxDtS
                                    : OrbitPredictionTuning::kAdaptiveEphemerisMaxDtControlledS);
        }
        if (request.lagrange_sensitive)
        {
            max_dt_s = std::min(max_dt_s, OrbitPredictionTuning::kLagrangeEphemerisMaxDtS);
        }
        out.max_dt_s = std::max(out.min_dt_s, max_dt_s);
        const double min_max_dt_for_soft_cap =
                sampling_spec.horizon_s / static_cast<double>(std::max<std::size_t>(soft_cap, 1));
        if (std::isfinite(min_max_dt_for_soft_cap) && min_max_dt_for_soft_cap > 0.0)
        {
            // Avoid impossible requests like "30 s max_dt over 54 days with a 24k soft cap".
            out.max_dt_s = std::max(out.max_dt_s, min_max_dt_for_soft_cap);
        }
        out.soft_max_segments = soft_cap;
        out.hard_max_segments = hard_cap;
        out.tolerance = make_ephemeris_tolerance_ramp(request);
        out.cancel_requested = cancel_requested;
        return out;
    }

    orbitsim::AdaptiveEphemerisOptions build_adaptive_ephemeris_options_for_chunk(
            const OrbitPredictionService::Request &request,
            const OrbitPredictionService::PredictionChunkPlan &chunk,
            const CancelCheck &cancel_requested)
    {
        const double duration_s = chunk_duration_s(chunk);
        if (!(duration_s > 0.0))
        {
            return {};
        }

        const OrbitPredictionService::EphemerisSamplingSpec sampling_spec{
                .valid = true,
                .horizon_s = duration_s,
        };
        orbitsim::AdaptiveEphemerisOptions out =
                build_adaptive_ephemeris_options(request, sampling_spec, cancel_requested);
        if (!(out.duration_s > 0.0))
        {
            return out;
        }

        const OrbitPredictionService::PredictionProfileDefinition def =
                resolve_prediction_profile_definition(request, chunk);
        out.duration_s = duration_s;
        out.min_dt_s = def.ephemeris_min_dt_s;
        out.max_dt_s = std::max(out.min_dt_s, def.ephemeris_max_dt_s);
        out.soft_max_segments = def.ephemeris_soft_max_segments;
        out.hard_max_segments = std::max(out.hard_max_segments, out.soft_max_segments);
        out.tolerance = scale_tolerance_ramp(out.tolerance, def.integrator_tolerance_multiplier);
        return out;
    }

    bool ephemeris_covers_horizon(const OrbitPredictionService::SharedCelestialEphemeris &ephemeris,
                                  const double sim_time_s,
                                  const double horizon_s)
    {
        return ephemeris &&
               !ephemeris->empty() &&
               (ephemeris->t_end_s() + kEphemerisDurationEpsilonS) >= (sim_time_s + horizon_s);
    }

    bool compatible_cached_ephemeris(const OrbitPredictionService::CachedEphemerisEntry &entry,
                                     const OrbitPredictionService::EphemerisBuildRequest &request)
    {
        if (!entry.ephemeris || entry.ephemeris->empty())
        {
            return false;
        }

        if (entry.sim_time_s != request.sim_time_s)
        {
            return false;
        }

        if (!same_config_for_ephemeris(entry.sim_config, request.sim_config))
        {
            return false;
        }

        if (!same_massive_body_set_for_ephemeris(entry.massive_bodies, request.massive_bodies))
        {
            return false;
        }

        if ((entry.duration_s + kEphemerisDurationEpsilonS) < request.duration_s)
        {
            return false;
        }

        if (!adaptive_ephemeris_options_cover_request(entry.adaptive_options, request.adaptive_options))
        {
            return false;
        }

        return true;
    }

    OrbitPredictionService::SharedCelestialEphemeris build_ephemeris_from_request(
            const OrbitPredictionService::EphemerisBuildRequest &request,
            const CancelCheck &cancel_requested,
            orbitsim::AdaptiveEphemerisDiagnostics *out_diag)
    {
        if (!std::isfinite(request.sim_time_s) ||
            !(request.duration_s > 0.0) ||
            !(request.adaptive_options.duration_s > 0.0))
        {
            return {};
        }

        orbitsim::GameSimulation sim(request.sim_config);
        if (!sim.set_time_s(request.sim_time_s))
        {
            return {};
        }

        if (cancel_requested && cancel_requested())
        {
            return {};
        }

        for (const orbitsim::MassiveBody &body : request.massive_bodies)
        {
            const auto body_handle =
                    (body.id != orbitsim::kInvalidBodyId)
                            ? sim.create_body_with_id(body.id, body)
                            : sim.create_body(body);
            if (!body_handle.valid())
            {
                return {};
            }
        }

        orbitsim::AdaptiveEphemerisOptions adaptive_opt = request.adaptive_options;
        adaptive_opt.cancel_requested = cancel_requested;
        adaptive_opt.duration_s = request.duration_s;

        auto ephemeris = std::make_shared<orbitsim::CelestialEphemeris>(
                orbitsim::build_celestial_ephemeris_adaptive(sim, adaptive_opt, out_diag));
        if (!ephemeris || ephemeris->empty())
        {
            return {};
        }

        return ephemeris;
    }

    OrbitPredictionService::EphemerisBuildRequest build_ephemeris_build_request(
            const OrbitPredictionService::Request &request,
            const OrbitPredictionService::EphemerisSamplingSpec &sampling_spec)
    {
        OrbitPredictionService::EphemerisBuildRequest out{};
        out.sim_time_s = request.sim_time_s;
        out.sim_config = request.sim_config;
        out.massive_bodies = request.massive_bodies;
        out.duration_s = sampling_spec.horizon_s;
        out.adaptive_options = build_adaptive_ephemeris_options(request, sampling_spec);
        return out;
    }

    CelestialPredictionSamplingSpec build_celestial_prediction_sampling_spec(
            const OrbitPredictionService::Request &request,
            const orbitsim::MassiveBody &subject_body)
    {
        CelestialPredictionSamplingSpec out{};
        if (request.massive_bodies.size() < 2)
        {
            return out;
        }

        std::vector<orbitsim::MassiveBody> candidate_bodies;
        candidate_bodies.reserve(request.massive_bodies.size() - 1);
        for (const orbitsim::MassiveBody &body : request.massive_bodies)
        {
            if (body.id != subject_body.id)
            {
                candidate_bodies.push_back(body);
            }
        }
        if (candidate_bodies.empty())
        {
            return out;
        }

        const std::size_t primary_index = orbitsim::auto_select_primary_index(
                candidate_bodies,
                subject_body.state.position_m,
                [&candidate_bodies](const std::size_t i) -> orbitsim::Vec3 { return candidate_bodies[i].state.position_m; },
                request.sim_config.softening_length_m);
        const orbitsim::MassiveBody &reference_body = candidate_bodies[primary_index];

        out.rel_pos_m = glm::dvec3(subject_body.state.position_m - reference_body.state.position_m);
        out.rel_vel_mps = glm::dvec3(subject_body.state.velocity_mps - reference_body.state.velocity_mps);
        if (!finite_vec3(out.rel_pos_m) || !finite_vec3(out.rel_vel_mps))
        {
            return out;
        }

        const double mu_ref_m3_s2 = request.sim_config.gravitational_constant * reference_body.mass_kg;
        if (!(mu_ref_m3_s2 > 0.0) || !std::isfinite(mu_ref_m3_s2))
        {
            return out;
        }

        const double horizon_s_auto =
                OrbitPredictionMath::select_prediction_horizon(mu_ref_m3_s2, out.rel_pos_m, out.rel_vel_mps);

        const double requested_window_s =
                std::isfinite(request.future_window_s) ? std::max(0.0, request.future_window_s) : 0.0;
        double horizon_s = std::max(OrbitPredictionTuning::kMinHorizonS, horizon_s_auto);
        horizon_s = std::max(horizon_s, std::max(1.0, requested_window_s));
        if (!(horizon_s > 0.0))
        {
            return out;
        }

        out.valid = true;
        out.horizon_s = horizon_s;
        return out;
    }
} // namespace Game
