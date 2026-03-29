#include "game/orbit/prediction/orbit_prediction_service_internal.h"

#include "orbitsim/integrators.hpp"

namespace Game
{
    namespace
    {
        using PredictionChunkPlan = OrbitPredictionService::PredictionChunkPlan;

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

            return ramp;
        }
    } // namespace

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
        const std::size_t max_budget = 4'000u;
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

        const std::size_t soft_cap = OrbitPredictionTuning::kAdaptiveSegmentSoftMaxSegmentsNormal;
        std::size_t hard_cap =
                OrbitPredictionTuning::kAdaptiveSegmentHardMaxSegmentsNormal;
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
                                OrbitPredictionTuning::kAdaptiveSegmentMaxDtControlledS);
        }

        const double lookup_max_dt_s = controlled
                                               ? OrbitPredictionTuning::kAdaptiveSegmentLookupMaxDtControlledS
                                               : OrbitPredictionTuning::kAdaptiveSegmentLookupMaxDtS;

        out.duration_s = sampling_spec.horizon_s;
        out.min_dt_s = request_needs_control_sensitive_prediction(request)
                               ? OrbitPredictionTuning::kAdaptiveSegmentMinDtControlledS
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

        const std::size_t soft_cap = OrbitPredictionTuning::kAdaptiveEphemerisSoftMaxSegments;
        const std::size_t hard_cap = std::max<std::size_t>(
                request.lagrange_sensitive ? OrbitPredictionTuning::kLagrangeEphemerisMaxSamples : soft_cap,
                soft_cap);

        out.duration_s = sampling_spec.horizon_s;
        out.min_dt_s = controlled
                               ? OrbitPredictionTuning::kAdaptiveEphemerisMinDtControlledS
                               : OrbitPredictionTuning::kAdaptiveEphemerisMinDtS;

        double max_dt_s = OrbitPredictionTuning::kAdaptiveEphemerisMaxDtS;
        if (controlled)
        {
            max_dt_s = std::min(max_dt_s,
                                OrbitPredictionTuning::kAdaptiveEphemerisMaxDtControlledS);
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
} // namespace Game
