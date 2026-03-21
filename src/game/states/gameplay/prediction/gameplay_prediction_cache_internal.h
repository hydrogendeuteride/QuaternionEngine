#pragma once

#include "game/orbit/orbit_prediction_math.h"
#include "game/orbit/orbit_render_curve.h"
#include "game/orbit/prediction/orbit_prediction_service_internal.h"
#include "game/states/gameplay/prediction/gameplay_state_prediction_types.h"

#include "orbitsim/trajectory_transforms.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <vector>

namespace Game::PredictionCacheInternal
{
    inline constexpr orbitsim::SpacecraftId kPlayerDisplayTargetSpacecraftId =
            static_cast<orbitsim::SpacecraftId>(0x7000'0001u);

    inline bool sample_prediction_inertial_state(const std::vector<orbitsim::TrajectorySample> &trajectory,
                                                 const double query_time_s,
                                                 orbitsim::State &out_state)
    {
        out_state = {};
        if (trajectory.empty() || !std::isfinite(query_time_s))
        {
            return false;
        }

        const auto it_hi = std::lower_bound(trajectory.cbegin(),
                                            trajectory.cend(),
                                            query_time_s,
                                            [](const orbitsim::TrajectorySample &sample, const double t_s) {
                                                return sample.t_s < t_s;
                                            });
        const std::size_t i_hi = static_cast<std::size_t>(std::distance(trajectory.cbegin(), it_hi));
        if (i_hi == 0)
        {
            out_state = orbitsim::make_state(trajectory.front().position_m, trajectory.front().velocity_mps);
            return true;
        }
        if (i_hi >= trajectory.size())
        {
            out_state = orbitsim::make_state(trajectory.back().position_m, trajectory.back().velocity_mps);
            return true;
        }

        const orbitsim::TrajectorySample &a = trajectory[i_hi - 1];
        const orbitsim::TrajectorySample &b = trajectory[i_hi];
        const double h = b.t_s - a.t_s;
        if (!(h > 0.0) || !std::isfinite(h))
        {
            out_state = orbitsim::make_state(a.position_m, a.velocity_mps);
            return true;
        }

        double u = (query_time_s - a.t_s) / h;
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

        const glm::dvec3 p0 = glm::dvec3(a.position_m);
        const glm::dvec3 p1 = glm::dvec3(b.position_m);
        const glm::dvec3 m0 = glm::dvec3(a.velocity_mps) * h;
        const glm::dvec3 m1 = glm::dvec3(b.velocity_mps) * h;
        const glm::dvec3 pos = (h00 * p0) + (h10 * m0) + (h01 * p1) + (h11 * m1);
        const glm::dvec3 vel = ((dh00 * p0) + (dh10 * m0) + (dh01 * p1) + (dh11 * m1)) / h;
        if (!finite_vec3(pos) || !finite_vec3(vel))
        {
            return false;
        }

        out_state = orbitsim::make_state(pos, vel);
        return true;
    }

    inline bool sample_prediction_inertial_state(const std::vector<orbitsim::TrajectorySegment> &segments,
                                                 const double query_time_s,
                                                 orbitsim::State &out_state)
    {
        return sample_trajectory_segment_state(segments, query_time_s, out_state);
    }

    inline orbitsim::SpacecraftStateLookup build_player_lookup(
            const std::vector<orbitsim::TrajectorySample> &trajectory)
    {
        if (trajectory.size() < 2)
        {
            return nullptr;
        }

        return [&trajectory](const orbitsim::SpacecraftId spacecraft_id, const double t_s)
                -> std::optional<orbitsim::State> {
            if (spacecraft_id != kPlayerDisplayTargetSpacecraftId)
            {
                return std::nullopt;
            }

            orbitsim::State sampled{};
            if (!sample_prediction_inertial_state(trajectory, t_s, sampled))
            {
                return std::nullopt;
            }
            return sampled;
        };
    }

    inline orbitsim::SpacecraftStateLookup build_player_lookup(
            const std::vector<orbitsim::TrajectorySegment> &segments)
    {
        if (segments.empty())
        {
            return nullptr;
        }

        return [&segments](const orbitsim::SpacecraftId spacecraft_id, const double t_s)
                -> std::optional<orbitsim::State> {
            if (spacecraft_id != kPlayerDisplayTargetSpacecraftId)
            {
                return std::nullopt;
            }

            orbitsim::State sampled{};
            if (!sample_prediction_inertial_state(segments, t_s, sampled))
            {
                return std::nullopt;
            }
            return sampled;
        };
    }

    inline std::shared_ptr<const std::vector<OrbitPlotSystem::GpuRootSegment>> build_gpu_root_cache(
            const std::vector<orbitsim::TrajectorySegment> &segments)
    {
        auto out = std::make_shared<std::vector<OrbitPlotSystem::GpuRootSegment>>();
        out->reserve(segments.size());

        double prefix_length_m = 0.0;
        for (const orbitsim::TrajectorySegment &segment : segments)
        {
            if (!(segment.dt_s > 0.0) || !std::isfinite(segment.dt_s))
            {
                continue;
            }

            const glm::dvec3 p0 = glm::dvec3(segment.start.position_m);
            const glm::dvec3 p1 = glm::dvec3(segment.end.position_m);
            const glm::dvec3 v0 = glm::dvec3(segment.start.velocity_mps);
            const glm::dvec3 v1 = glm::dvec3(segment.end.velocity_mps);
            if (!finite_vec3(p0) || !finite_vec3(p1) || !finite_vec3(v0) || !finite_vec3(v1))
            {
                continue;
            }

            OrbitPlotSystem::GpuRootSegment root{};
            root.t0_s = segment.t0_s;
            root.p0_bci = p0;
            root.v0_bci = v0;
            root.p1_bci = p1;
            root.v1_bci = v1;
            root.dt_s = segment.dt_s;
            root.prefix_length_m = prefix_length_m;
            out->push_back(root);

            const double chord_m = glm::length(p1 - p0);
            if (std::isfinite(chord_m) && chord_m > 0.0)
            {
                prefix_length_m += chord_m;
            }
        }

        return out;
    }

    inline std::vector<double> collect_maneuver_node_times(const OrbitPredictionCache &cache)
    {
        std::vector<double> out;
        out.reserve(cache.maneuver_previews.size());
        for (const auto &preview : cache.maneuver_previews)
        {
            if (std::isfinite(preview.t_s))
            {
                out.push_back(preview.t_s);
            }
        }
        return out;
    }

    inline std::vector<orbitsim::TrajectorySample> sample_prediction_segments(
            const std::vector<orbitsim::TrajectorySegment> &segments,
            std::size_t total_sample_budget,
            const std::vector<double> &node_times_s = {})
    {
        std::vector<orbitsim::TrajectorySample> out;
        if (segments.empty())
        {
            return out;
        }

        total_sample_budget = std::max<std::size_t>(total_sample_budget, 2);
        const double start_time_s = segments.front().t0_s;
        const double end_time_s = prediction_segment_end_time(segments.back());
        const double horizon_s = std::max(0.0, end_time_s - start_time_s);
        if (!(horizon_s > 0.0))
        {
            return out;
        }

        const std::vector<MultiBandSampleWindow> windows =
                node_times_s.empty()
                        ? build_multi_band_sample_windows(start_time_s, horizon_s, total_sample_budget)
                        : build_multi_band_sample_windows(start_time_s, horizon_s, total_sample_budget, node_times_s);
        out = sample_trajectory_segments_multi_band(segments, windows);
        return out;
    }

    inline void update_derived_diagnostics(OrbitPredictionDerivedDiagnostics *diagnostics,
                                           const OrbitPredictionCache &cache,
                                           const PredictionDerivedStatus status)
    {
        if (!diagnostics)
        {
            return;
        }

        diagnostics->status = status;
        diagnostics->frame_segment_count = cache.trajectory_segments_frame.size();
        diagnostics->frame_segment_count_planned = cache.trajectory_segments_frame_planned.size();
        diagnostics->frame_sample_count = cache.trajectory_frame.size();
        diagnostics->frame_sample_count_planned = cache.trajectory_frame_planned.size();
    }

    inline orbitsim::FrameSegmentTransformOptions build_frame_segment_transform_options(
            const orbitsim::TrajectoryFrameSpec &frame_spec,
            const std::vector<orbitsim::TrajectorySegment> &inertial_segments,
            const CancelCheck &cancel_requested = {})
    {
        orbitsim::FrameSegmentTransformOptions out{};
        const double start_time_s = inertial_segments.empty() ? 0.0 : inertial_segments.front().t0_s;
        const double end_time_s =
                inertial_segments.empty() ? 0.0 : prediction_segment_end_time(inertial_segments.back());
        const double horizon_s = std::max(0.0, end_time_s - start_time_s);
        const bool long_range = horizon_s > OrbitPredictionTuning::kLongRangeHorizonThresholdS;
        const bool sensitive = frame_spec.type == orbitsim::TrajectoryFrameType::Synodic ||
                               frame_spec.type == orbitsim::TrajectoryFrameType::LVLH;

        out.min_dt_s = OrbitPredictionTuning::kAdaptiveFrameTransformMinDtS;
        out.max_dt_s = sensitive
                               ? std::min(OrbitPredictionTuning::kAdaptiveFrameTransformMaxDtSensitiveS,
                                          long_range ? OrbitPredictionTuning::kAdaptiveFrameTransformMaxDtLongRangeS
                                                     : OrbitPredictionTuning::kAdaptiveFrameTransformMaxDtS)
                               : (long_range ? OrbitPredictionTuning::kAdaptiveFrameTransformMaxDtLongRangeS
                                             : OrbitPredictionTuning::kAdaptiveFrameTransformMaxDtS);
        out.soft_max_segments =
                std::max<std::size_t>(OrbitPredictionTuning::kAdaptiveFrameTransformSoftMaxSegments,
                                      inertial_segments.size());
        out.hard_max_segments =
                std::max<std::size_t>(OrbitPredictionTuning::kAdaptiveFrameTransformHardMaxSegments,
                                      out.soft_max_segments);
        out.tolerance.pos_near_m = OrbitPredictionTuning::kAdaptiveFrameTransformPosTolNearM;
        out.tolerance.pos_far_m = OrbitPredictionTuning::kAdaptiveFrameTransformPosTolFarM;
        out.tolerance.vel_near_mps = OrbitPredictionTuning::kAdaptiveFrameTransformVelTolNearMps;
        out.tolerance.vel_far_mps = OrbitPredictionTuning::kAdaptiveFrameTransformVelTolFarMps;
        out.tolerance.rel_pos_floor = OrbitPredictionTuning::kAdaptiveFrameTransformRelPosFloor;
        out.tolerance.rel_vel_floor = OrbitPredictionTuning::kAdaptiveFrameTransformRelVelFloor;
        if (sensitive)
        {
            out.tolerance.pos_near_m *= 0.5;
            out.tolerance.pos_far_m *= 0.25;
            out.tolerance.vel_near_mps *= 0.5;
            out.tolerance.vel_far_mps *= 0.25;
        }
        out.cancel_requested = cancel_requested;
        return out;
    }

    inline bool rebuild_prediction_frame_cache(
            OrbitPredictionCache &cache,
            const orbitsim::TrajectoryFrameSpec &resolved_frame_spec,
            const std::vector<orbitsim::TrajectorySegment> &player_lookup_segments_inertial,
            const CancelCheck &cancel_requested = {},
            OrbitPredictionDerivedDiagnostics *diagnostics = nullptr)
    {
        if (diagnostics)
        {
            *diagnostics = {};
        }

        cache.trajectory_frame.clear();
        cache.trajectory_frame_planned.clear();
        cache.trajectory_segments_frame.clear();
        cache.trajectory_segments_frame_planned.clear();
        cache.gpu_roots_frame.reset();
        cache.gpu_roots_frame_planned.reset();
        cache.render_curve_frame.clear();
        cache.render_curve_frame_planned.clear();
        cache.resolved_frame_spec = {};
        cache.resolved_frame_spec_valid = false;
        cache.metrics_valid = false;

        if (resolved_frame_spec.type == orbitsim::TrajectoryFrameType::Inertial)
        {
            cache.trajectory_frame = cache.trajectory_inertial;
            cache.trajectory_frame_planned = cache.trajectory_inertial_planned;
            cache.trajectory_segments_frame = cache.trajectory_segments_inertial;
            cache.trajectory_segments_frame_planned = cache.trajectory_segments_inertial_planned;
        }
        else
        {
            if (!cache.shared_ephemeris || cache.shared_ephemeris->empty())
            {
                update_derived_diagnostics(diagnostics, cache, PredictionDerivedStatus::MissingEphemeris);
                return false;
            }

            const auto player_lookup = build_player_lookup(player_lookup_segments_inertial);
            const std::size_t base_sample_budget = std::max<std::size_t>(cache.trajectory_inertial.size(), 2);
            const std::size_t planned_sample_budget =
                    cache.trajectory_inertial_planned.size() >= 2 ? cache.trajectory_inertial_planned.size()
                                                                  : base_sample_budget;
            const std::vector<double> node_times = collect_maneuver_node_times(cache);

            const orbitsim::FrameSegmentTransformOptions base_opt =
                    build_frame_segment_transform_options(
                            resolved_frame_spec,
                            cache.trajectory_segments_inertial,
                            cancel_requested);
            cache.trajectory_segments_frame = orbitsim::transform_trajectory_segments_to_frame_spec(
                    cache.trajectory_segments_inertial,
                    *cache.shared_ephemeris,
                    cache.massive_bodies,
                    resolved_frame_spec,
                    base_opt,
                    player_lookup);
            if (cancel_requested && cancel_requested())
            {
                update_derived_diagnostics(diagnostics, cache, PredictionDerivedStatus::Cancelled);
                return false;
            }
            if (cache.trajectory_segments_frame.empty())
            {
                update_derived_diagnostics(diagnostics, cache, PredictionDerivedStatus::FrameTransformFailed);
                return false;
            }
            cache.trajectory_frame = sample_prediction_segments(cache.trajectory_segments_frame, base_sample_budget);
            if (cache.trajectory_frame.size() < 2)
            {
                update_derived_diagnostics(diagnostics, cache, PredictionDerivedStatus::FrameSamplesUnavailable);
                return false;
            }

            if (!cache.trajectory_segments_inertial_planned.empty())
            {
                const orbitsim::FrameSegmentTransformOptions planned_opt =
                        build_frame_segment_transform_options(
                                resolved_frame_spec,
                                cache.trajectory_segments_inertial_planned,
                                cancel_requested);
                cache.trajectory_segments_frame_planned = orbitsim::transform_trajectory_segments_to_frame_spec(
                        cache.trajectory_segments_inertial_planned,
                        *cache.shared_ephemeris,
                        cache.massive_bodies,
                        resolved_frame_spec,
                        planned_opt,
                        player_lookup);
                if (cancel_requested && cancel_requested())
                {
                    update_derived_diagnostics(diagnostics, cache, PredictionDerivedStatus::Cancelled);
                    return false;
                }
                if (!cache.trajectory_segments_frame_planned.empty())
                {
                    cache.trajectory_frame_planned = sample_prediction_segments(
                            cache.trajectory_segments_frame_planned,
                            planned_sample_budget,
                            node_times);
                }
            }
        }

        if (cache.trajectory_frame.size() < 2 || cache.trajectory_segments_frame.empty())
        {
            update_derived_diagnostics(diagnostics, cache, PredictionDerivedStatus::MissingSolverData);
            return false;
        }

        cache.gpu_roots_frame = build_gpu_root_cache(cache.trajectory_segments_frame);
        cache.gpu_roots_frame_planned = build_gpu_root_cache(cache.trajectory_segments_frame_planned);
        cache.render_curve_frame = OrbitRenderCurve::build(cache.trajectory_segments_frame);
        cache.render_curve_frame_planned = cache.trajectory_segments_frame_planned.empty()
                                                  ? OrbitRenderCurve{}
                                                  : OrbitRenderCurve::build(cache.trajectory_segments_frame_planned);
        cache.resolved_frame_spec = resolved_frame_spec;
        cache.resolved_frame_spec_valid = true;
        update_derived_diagnostics(diagnostics, cache, PredictionDerivedStatus::Success);
        return true;
    }

    inline void clear_prediction_metrics(OrbitPredictionCache &cache, const orbitsim::BodyId analysis_body_id)
    {
        cache.altitude_km.clear();
        cache.speed_kmps.clear();
        cache.semi_major_axis_m = 0.0;
        cache.eccentricity = 0.0;
        cache.orbital_period_s = 0.0;
        cache.periapsis_alt_km = 0.0;
        cache.apoapsis_alt_km = std::numeric_limits<double>::infinity();
        cache.metrics_body_id = analysis_body_id;
        cache.metrics_valid = true;
    }

    inline void rebuild_prediction_metrics(
            OrbitPredictionCache &cache,
            const orbitsim::GameSimulation::Config &sim_config,
            const orbitsim::BodyId analysis_body_id,
            const CancelCheck &cancel_requested = {})
    {
        clear_prediction_metrics(cache, analysis_body_id);

        const auto analysis_it = std::find_if(cache.massive_bodies.begin(),
                                              cache.massive_bodies.end(),
                                              [analysis_body_id](const orbitsim::MassiveBody &body) {
                                                  return body.id == analysis_body_id;
                                              });
        if (analysis_it == cache.massive_bodies.end() || !(analysis_it->mass_kg > 0.0))
        {
            return;
        }

        const double mu_ref_m3_s2 = sim_config.gravitational_constant * analysis_it->mass_kg;
        if (!(mu_ref_m3_s2 > 0.0) || !std::isfinite(mu_ref_m3_s2))
        {
            return;
        }

        std::vector<orbitsim::TrajectorySample> rel_samples;
        if (cache.resolved_frame_spec_valid &&
            cache.resolved_frame_spec.type == orbitsim::TrajectoryFrameType::BodyCenteredInertial &&
            cache.resolved_frame_spec.primary_body_id == analysis_body_id)
        {
            rel_samples = cache.trajectory_frame;
        }
        else if (cache.shared_ephemeris && !cache.shared_ephemeris->empty())
        {
            const std::size_t sample_budget = std::max<std::size_t>(cache.trajectory_inertial.size(), 2);
            const orbitsim::TrajectoryFrameSpec analysis_frame =
                    orbitsim::TrajectoryFrameSpec::body_centered_inertial(analysis_body_id);
            const orbitsim::FrameSegmentTransformOptions frame_opt =
                    build_frame_segment_transform_options(
                            analysis_frame,
                            cache.trajectory_segments_inertial,
                            cancel_requested);
            const std::vector<orbitsim::TrajectorySegment> rel_segments =
                    orbitsim::transform_trajectory_segments_to_frame_spec(
                            cache.trajectory_segments_inertial,
                            *cache.shared_ephemeris,
                            cache.massive_bodies,
                            analysis_frame,
                            frame_opt);
            if (cancel_requested && cancel_requested())
            {
                return;
            }
            rel_samples = sample_prediction_segments(rel_segments, sample_budget);
        }

        if (rel_samples.size() < 2)
        {
            return;
        }

        cache.altitude_km.reserve(rel_samples.size());
        cache.speed_kmps.reserve(rel_samples.size());
        for (const orbitsim::TrajectorySample &sample : rel_samples)
        {
            const double r_m = glm::length(sample.position_m);
            const double alt_km = (r_m - analysis_it->radius_m) * 1.0e-3;
            const double spd_kmps = glm::length(sample.velocity_mps) * 1.0e-3;
            cache.altitude_km.push_back(static_cast<float>(alt_km));
            cache.speed_kmps.push_back(static_cast<float>(spd_kmps));
        }

        const OrbitPredictionMath::OrbitalElementsEstimate elements =
                OrbitPredictionMath::compute_orbital_elements(mu_ref_m3_s2,
                                                              rel_samples.front().position_m,
                                                              rel_samples.front().velocity_mps);
        if (!elements.valid)
        {
            return;
        }

        cache.semi_major_axis_m = elements.semi_major_axis_m;
        cache.eccentricity = elements.eccentricity;
        cache.orbital_period_s = elements.orbital_period_s;
        cache.periapsis_alt_km = (elements.periapsis_m - analysis_it->radius_m) * 1.0e-3;
        cache.apoapsis_alt_km = std::isfinite(elements.apoapsis_m)
                                        ? (elements.apoapsis_m - analysis_it->radius_m) * 1.0e-3
                                        : std::numeric_limits<double>::infinity();
    }
} // namespace Game::PredictionCacheInternal
