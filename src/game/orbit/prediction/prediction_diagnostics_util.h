#pragma once

#include "game/orbit/orbit_prediction_service.h"
#include "game/orbit/trajectory/trajectory_utils.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace Game
{
    template<typename AdaptiveDiag>
    OrbitPredictionService::AdaptiveStageDiagnostics make_stage_diagnostics_from_adaptive(
            const AdaptiveDiag &diag,
            const double requested_duration_s,
            const bool cache_reused = false)
    {
        OrbitPredictionService::AdaptiveStageDiagnostics out{};
        out.requested_duration_s = std::max(0.0, requested_duration_s);
        out.covered_duration_s = std::max(0.0, diag.covered_duration_s);
        out.accepted_segments = diag.accepted_segments;
        out.rejected_splits = diag.rejected_splits;
        out.forced_boundary_splits = diag.forced_boundary_splits;
        out.min_dt_s = diag.min_dt_s;
        out.max_dt_s = diag.max_dt_s;
        out.avg_dt_s = diag.avg_dt_s;
        out.hard_cap_hit = diag.hard_cap_hit;
        out.cancelled = diag.cancelled;
        out.cache_reused = cache_reused;
        return out;
    }

    inline OrbitPredictionService::AdaptiveStageDiagnostics make_stage_diagnostics_from_segments(
            const std::vector<orbitsim::TrajectorySegment> &segments,
            const double requested_duration_s,
            const bool cache_reused = false)
    {
        OrbitPredictionService::AdaptiveStageDiagnostics out{};
        out.requested_duration_s = std::max(0.0, requested_duration_s);
        out.covered_duration_s = prediction_segment_span_s(segments);
        out.accepted_segments = segments.size();
        out.cache_reused = cache_reused;

        bool first_dt = true;
        double dt_sum_s = 0.0;
        for (const orbitsim::TrajectorySegment &segment : segments)
        {
            if (!(segment.dt_s > 0.0) || !std::isfinite(segment.dt_s))
            {
                continue;
            }

            if ((segment.flags & (orbitsim::kTrajectorySegmentFlagForcedBoundary |
                                  orbitsim::kTrajectorySegmentFlagImpulseBoundary |
                                  orbitsim::kTrajectorySegmentFlagBurnBoundary)) != 0u)
            {
                ++out.forced_boundary_splits;
            }
            if ((segment.flags & orbitsim::kTrajectorySegmentFlagFrameResegmented) != 0u)
            {
                ++out.frame_resegmentation_count;
            }

            if (first_dt)
            {
                out.min_dt_s = segment.dt_s;
                out.max_dt_s = segment.dt_s;
                first_dt = false;
            }
            else
            {
                out.min_dt_s = std::min(out.min_dt_s, segment.dt_s);
                out.max_dt_s = std::max(out.max_dt_s, segment.dt_s);
            }
            dt_sum_s += segment.dt_s;
        }

        if (out.accepted_segments > 0 && dt_sum_s > 0.0)
        {
            out.avg_dt_s = dt_sum_s / static_cast<double>(out.accepted_segments);
        }

        return out;
    }

    inline OrbitPredictionService::AdaptiveStageDiagnostics make_stage_diagnostics_from_ephemeris(
            const OrbitPredictionService::SharedCelestialEphemeris &ephemeris,
            const double requested_duration_s,
            const bool cache_reused = false)
    {
        OrbitPredictionService::AdaptiveStageDiagnostics out{};
        out.requested_duration_s = std::max(0.0, requested_duration_s);
        out.cache_reused = cache_reused;
        if (!ephemeris || ephemeris->empty())
        {
            return out;
        }

        out.covered_duration_s = std::max(0.0, ephemeris->t_end_s() - ephemeris->t0_s());
        out.accepted_segments = ephemeris->segments.size();

        bool first_dt = true;
        double dt_sum_s = 0.0;
        for (const orbitsim::CelestialEphemerisSegment &segment : ephemeris->segments)
        {
            if (!(segment.dt_s > 0.0) || !std::isfinite(segment.dt_s))
            {
                continue;
            }

            if (first_dt)
            {
                out.min_dt_s = segment.dt_s;
                out.max_dt_s = segment.dt_s;
                first_dt = false;
            }
            else
            {
                out.min_dt_s = std::min(out.min_dt_s, segment.dt_s);
                out.max_dt_s = std::max(out.max_dt_s, segment.dt_s);
            }
            dt_sum_s += segment.dt_s;
        }

        if (out.accepted_segments > 0 && dt_sum_s > 0.0)
        {
            out.avg_dt_s = dt_sum_s / static_cast<double>(out.accepted_segments);
        }

        return out;
    }
} // namespace Game
