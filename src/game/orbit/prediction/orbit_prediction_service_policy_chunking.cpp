#include "game/orbit/prediction/orbit_prediction_service_internal.h"

namespace Game
{
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
            if (request.preview_patch.active &&
                request.solve_quality == OrbitPredictionService::SolveQuality::FastPreview &&
                std::isfinite(request.preview_patch.anchor_time_s) &&
                request.preview_patch.exact_window_s > 0.0)
            {
                const double preview_t0_s = request.preview_patch.anchor_time_s;
                const double preview_t1_s =
                        request.preview_patch.anchor_time_s + (2.0 * request.preview_patch.exact_window_s);
                const double epsilon_s = continuity_time_epsilon_s(preview_t0_s);
                if (chunk_t0_s >= (preview_t0_s - epsilon_s) && chunk_t1_s <= (preview_t1_s + epsilon_s))
                {
                    return PredictionProfileId::Exact;
                }
            }

            const double elapsed_s = std::max(0.0, chunk_t0_s - request.sim_time_s);
            if (elapsed_s < OrbitPredictionTuning::kPredictionChunkBandTransferEndS)
            {
                return PredictionProfileId::Near;
            }
            return PredictionProfileId::Tail;
        }

        [[nodiscard]] uint32_t resolve_chunk_priority(const PredictionProfileId profile_id,
                                                      const uint32_t boundary_flags)
        {
            if (profile_id == PredictionProfileId::Exact)
            {
                return 0u;
            }
            if (planner_boundary_has_flag(boundary_flags, PredictionChunkBoundaryFlags::Maneuver))
            {
                return 1u;
            }
            return static_cast<uint32_t>(profile_id) + 1u;
        }

        [[nodiscard]] uint32_t profile_rank(const PredictionProfileId profile_id)
        {
            return static_cast<uint32_t>(profile_id);
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

        if (request.preview_patch.active &&
            request.solve_quality == OrbitPredictionService::SolveQuality::FastPreview &&
            std::isfinite(request.preview_patch.anchor_time_s) &&
            request.preview_patch.exact_window_s > 0.0)
        {
            const double preview_t0_s = request.preview_patch.anchor_time_s;
            const double preview_t1_s = request.preview_patch.anchor_time_s + request.preview_patch.exact_window_s;
            const double preview_t2_s = request.preview_patch.anchor_time_s + (2.0 * request.preview_patch.exact_window_s);
            append_planner_boundary(boundaries,
                                    request_t0_s,
                                    request_t1_s,
                                    preview_t0_s,
                                    PredictionChunkBoundaryFlags::None);
            append_planner_boundary(boundaries,
                                    request_t0_s,
                                    request_t1_s,
                                    preview_t1_s,
                                    PredictionChunkBoundaryFlags::None);
            append_planner_boundary(boundaries,
                                    request_t0_s,
                                    request_t1_s,
                                    preview_t2_s,
                                    PredictionChunkBoundaryFlags::None);
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

            uint32_t boundary_flags = merged_boundaries[i].flags | merged_boundaries[i + 1u].flags;
            const PredictionProfileId profile_id = resolve_chunk_profile_id(request, chunk_t0_s, chunk_t1_s);
            if (request.preview_patch.active &&
                request.solve_quality == OrbitPredictionService::SolveQuality::FastPreview &&
                std::isfinite(request.preview_patch.anchor_time_s) &&
                request.preview_patch.exact_window_s > 0.0)
            {
                const double preview_t0_s = request.preview_patch.anchor_time_s;
                const double preview_t1_s =
                        request.preview_patch.anchor_time_s + (2.0 * request.preview_patch.exact_window_s);
                const double epsilon_s = continuity_time_epsilon_s(preview_t0_s);
                if (chunk_t0_s >= (preview_t0_s - epsilon_s) && chunk_t1_s <= (preview_t1_s + epsilon_s))
                {
                    boundary_flags |= planner_boundary_flag_bits(PredictionChunkBoundaryFlags::PreviewChunk);
                    if (std::abs(chunk_t0_s - preview_t0_s) <= epsilon_s)
                    {
                        boundary_flags |= planner_boundary_flag_bits(PredictionChunkBoundaryFlags::PreviewAnchor);
                    }
                }
            }
            plan.chunks.push_back(PredictionChunkPlan{
                    .chunk_id = static_cast<uint32_t>(plan.chunks.size()),
                    .t0_s = chunk_t0_s,
                    .t1_s = chunk_t1_s,
                    .profile_id = profile_id,
                    .boundary_flags = boundary_flags,
                    .priority = resolve_chunk_priority(profile_id, boundary_flags),
                    .allow_reuse = true,
                    .requires_seam_validation = !plan.chunks.empty(),
            });
        }

        plan.valid = !plan.chunks.empty();
        return plan;
    }

    OrbitPredictionService::PredictionProfileId promote_prediction_profile(
            OrbitPredictionService::PredictionProfileId profile_id,
            const uint32_t steps)
    {
        if (steps == 0u)
        {
            return profile_id;
        }

        const uint32_t rank = profile_rank(profile_id);
        const uint32_t promoted_rank = (steps >= rank) ? 0u : (rank - steps);
        return static_cast<OrbitPredictionService::PredictionProfileId>(promoted_rank);
    }
} // namespace Game
