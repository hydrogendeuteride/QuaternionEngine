#include "game/states/gameplay/prediction/prediction_metrics_builder.h"
#include "game/states/gameplay/prediction/gameplay_prediction_cache_internal.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace Game
{
    using namespace PredictionCacheInternal;

    namespace
    {
        struct PredictionRadialExtrema
        {
            bool periapsis_valid{false};
            bool apoapsis_valid{false};
            double periapsis_radius_m{std::numeric_limits<double>::infinity()};
            double apoapsis_radius_m{0.0};
        };

        enum class PredictionRadialExtremumKind : uint8_t
        {
            None = 0,
            Periapsis,
            Apoapsis,
        };

        bool evaluate_radial_motion(const orbitsim::TrajectorySegment &segment,
                                    const double t_s,
                                    double &out_r_dot_v,
                                    double *out_radius_m = nullptr)
        {
            orbitsim::State state{};
            if (!eval_segment_state(segment, t_s, state))
            {
                return false;
            }

            const double radius_m = glm::length(state.position_m);
            const double r_dot_v = glm::dot(state.position_m, state.velocity_mps);
            if (!std::isfinite(radius_m) || !std::isfinite(r_dot_v))
            {
                return false;
            }

            out_r_dot_v = r_dot_v;
            if (out_radius_m)
            {
                *out_radius_m = radius_m;
            }
            return true;
        }

        bool sample_prediction_radius_m(const std::vector<orbitsim::TrajectorySegment> &segments,
                                        const double t_s,
                                        double &out_radius_m)
        {
            orbitsim::State state{};
            if (!sample_trajectory_segment_state(segments,
                                                 t_s,
                                                 state,
                                                 TrajectoryBoundarySide::ContinuousPositionOnly))
            {
                return false;
            }

            const double radius_m = glm::length(state.position_m);
            if (!(radius_m > 0.0) || !std::isfinite(radius_m))
            {
                return false;
            }

            out_radius_m = radius_m;
            return true;
        }

        PredictionRadialExtremumKind classify_radial_extremum_by_radius(
                const std::vector<orbitsim::TrajectorySegment> &segments,
                const orbitsim::TrajectorySegment &reference_segment,
                const double t_s,
                const double radius_m)
        {
            if (segments.empty() || !(radius_m > 0.0) || !std::isfinite(radius_m))
            {
                return PredictionRadialExtremumKind::None;
            }

            const double trajectory_t0_s = segments.front().t0_s;
            const double trajectory_t1_s = prediction_segment_end_time(segments.back());
            if (!std::isfinite(trajectory_t0_s) || !std::isfinite(trajectory_t1_s) || !(trajectory_t1_s > trajectory_t0_s))
            {
                return PredictionRadialExtremumKind::None;
            }

            const double probe_dt_s = std::clamp(reference_segment.dt_s * 1.0e-3, 1.0e-6, 1.0);
            const double tolerance_m = std::max(1.0e-6, radius_m * 1.0e-9);

            double before_radius_m = 0.0;
            double after_radius_m = 0.0;
            const bool have_before =
                    (t_s - probe_dt_s) >= trajectory_t0_s &&
                    sample_prediction_radius_m(segments, t_s - probe_dt_s, before_radius_m);
            const bool have_after =
                    (t_s + probe_dt_s) <= trajectory_t1_s &&
                    sample_prediction_radius_m(segments, t_s + probe_dt_s, after_radius_m);

            if (!have_before && !have_after)
            {
                return PredictionRadialExtremumKind::None;
            }

            const bool lower_than_before = !have_before || radius_m <= (before_radius_m + tolerance_m);
            const bool lower_than_after = !have_after || radius_m <= (after_radius_m + tolerance_m);
            const bool strictly_lower =
                    (have_before && radius_m < (before_radius_m - tolerance_m)) ||
                    (have_after && radius_m < (after_radius_m - tolerance_m));
            if (lower_than_before && lower_than_after && strictly_lower)
            {
                return PredictionRadialExtremumKind::Periapsis;
            }

            const bool higher_than_before = !have_before || radius_m >= (before_radius_m - tolerance_m);
            const bool higher_than_after = !have_after || radius_m >= (after_radius_m - tolerance_m);
            const bool strictly_higher =
                    (have_before && radius_m > (before_radius_m + tolerance_m)) ||
                    (have_after && radius_m > (after_radius_m + tolerance_m));
            if (higher_than_before && higher_than_after && strictly_higher)
            {
                return PredictionRadialExtremumKind::Apoapsis;
            }

            return PredictionRadialExtremumKind::None;
        }

        int radial_motion_sign(const double r_dot_v)
        {
            if (!std::isfinite(r_dot_v))
            {
                return 0;
            }

            constexpr double kRelativeZero = 1.0e-12;
            const double tolerance = std::max(1.0e-12, std::abs(r_dot_v) * kRelativeZero);
            if (r_dot_v > tolerance)
            {
                return 1;
            }
            if (r_dot_v < -tolerance)
            {
                return -1;
            }
            return 0;
        }

        double refine_radial_extremum_time(const orbitsim::TrajectorySegment &segment,
                                           const double t0_s,
                                           const double t1_s,
                                           const double f0)
        {
            if (!(t1_s > t0_s) || !std::isfinite(t0_s) || !std::isfinite(t1_s))
            {
                return t0_s;
            }

            double a = t0_s;
            double b = t1_s;
            double fa = f0;
            const double time_tolerance_s = std::max(1.0e-9, (t1_s - t0_s) * 1.0e-12);

            for (int i = 0; i < 64 && (b - a) > time_tolerance_s; ++i)
            {
                const double m = 0.5 * (a + b);
                double fm = 0.0;
                if (!evaluate_radial_motion(segment, m, fm))
                {
                    break;
                }

                const int sign_a = radial_motion_sign(fa);
                const int sign_m = radial_motion_sign(fm);
                if (sign_m == 0)
                {
                    return m;
                }
                if (sign_a == 0 || sign_a != sign_m)
                {
                    b = m;
                }
                else
                {
                    a = m;
                    fa = fm;
                }
            }

            return 0.5 * (a + b);
        }

        void record_radial_extremum(PredictionRadialExtrema &out,
                                    const std::vector<orbitsim::TrajectorySegment> &segments,
                                    const orbitsim::TrajectorySegment &segment,
                                    const double t_s,
                                    const int sign_before,
                                    const int sign_after,
                                    double &last_recorded_t_s)
        {
            if (!std::isfinite(t_s) || sign_before == sign_after)
            {
                return;
            }

            constexpr double kDuplicateRootToleranceS = 1.0e-6;
            if (std::isfinite(last_recorded_t_s) && std::abs(t_s - last_recorded_t_s) <= kDuplicateRootToleranceS)
            {
                return;
            }

            double r_dot_v = 0.0;
            double radius_m = 0.0;
            if (!evaluate_radial_motion(segment, t_s, r_dot_v, &radius_m) || !(radius_m > 0.0))
            {
                return;
            }

            const PredictionRadialExtremumKind kind =
                    classify_radial_extremum_by_radius(segments, segment, t_s, radius_m);

            if (kind == PredictionRadialExtremumKind::Periapsis)
            {
                out.periapsis_radius_m = std::min(out.periapsis_radius_m, radius_m);
                out.periapsis_valid = true;
            }
            else if (kind == PredictionRadialExtremumKind::Apoapsis)
            {
                out.apoapsis_radius_m = std::max(out.apoapsis_radius_m, radius_m);
                out.apoapsis_valid = true;
            }
            else
            {
                return;
            }

            last_recorded_t_s = t_s;
        }

        PredictionRadialExtrema find_prediction_radial_extrema(
                const std::vector<orbitsim::TrajectorySegment> &segments)
        {
            PredictionRadialExtrema out{};
            if (segments.empty())
            {
                return out;
            }

            constexpr int kRootScanSubstepsPerSegment = 8;
            double last_recorded_t_s = std::numeric_limits<double>::quiet_NaN();

            for (const orbitsim::TrajectorySegment &segment : segments)
            {
                const double seg_t0_s = segment.t0_s;
                const double seg_t1_s = prediction_segment_end_time(segment);
                if (!(segment.dt_s > 0.0) || !std::isfinite(seg_t0_s) || !std::isfinite(seg_t1_s) ||
                    !(seg_t1_s > seg_t0_s))
                {
                    continue;
                }

                double prev_t_s = seg_t0_s;
                double prev_f = 0.0;
                if (!evaluate_radial_motion(segment, prev_t_s, prev_f))
                {
                    continue;
                }

                for (int i = 1; i <= kRootScanSubstepsPerSegment; ++i)
                {
                    const double curr_t_s =
                            (i == kRootScanSubstepsPerSegment)
                                    ? seg_t1_s
                                    : (seg_t0_s + (segment.dt_s * static_cast<double>(i)) /
                                                        static_cast<double>(kRootScanSubstepsPerSegment));
                    double curr_f = 0.0;
                    if (!evaluate_radial_motion(segment, curr_t_s, curr_f))
                    {
                        continue;
                    }

                    int prev_sign = radial_motion_sign(prev_f);
                    int curr_sign = radial_motion_sign(curr_f);
                    if (prev_sign == 0 && curr_sign != 0)
                    {
                        record_radial_extremum(out, segments, segment, prev_t_s, -curr_sign, curr_sign, last_recorded_t_s);
                    }
                    else if (prev_sign != 0 && curr_sign == 0)
                    {
                        record_radial_extremum(out, segments, segment, curr_t_s, prev_sign, -prev_sign, last_recorded_t_s);
                    }
                    else if (prev_sign != 0 && curr_sign != 0 && prev_sign != curr_sign)
                    {
                        const double root_t_s = refine_radial_extremum_time(segment, prev_t_s, curr_t_s, prev_f);
                        record_radial_extremum(out, segments, segment, root_t_s, prev_sign, curr_sign, last_recorded_t_s);
                    }

                    prev_t_s = curr_t_s;
                    prev_f = curr_f;
                }
            }

            return out;
        }
    } // namespace

    void PredictionMetricsBuilder::clear(OrbitPredictionCache &cache, const orbitsim::BodyId analysis_body_id)
    {
        cache.analysis.clear_metrics(analysis_body_id);
    }

    void PredictionMetricsBuilder::rebuild(
            const PredictionSolverTrajectoryCache &solver,
            const PredictionDisplayFrameCache &display,
            PredictionAnalysisCache &analysis,
            const orbitsim::GameSimulation::Config &sim_config,
            const orbitsim::BodyId analysis_body_id,
            const CancelCheck &cancel_requested)
    {
        analysis.clear_metrics(analysis_body_id);

        const auto &base_ephemeris = solver.resolved_shared_ephemeris();
        const auto &base_bodies = solver.resolved_massive_bodies();
        const auto &base_samples = solver.resolved_trajectory_inertial();
        const auto &base_segments = solver.resolved_trajectory_segments_inertial();
        const auto analysis_it = std::find_if(base_bodies.begin(),
                                              base_bodies.end(),
                                              [analysis_body_id](const orbitsim::MassiveBody &body) {
                                                  return body.id == analysis_body_id;
                                              });
        if (analysis_it == base_bodies.end() || !(analysis_it->mass_kg > 0.0))
        {
            return;
        }

        const double mu_ref_m3_s2 = sim_config.gravitational_constant * analysis_it->mass_kg;
        if (!(mu_ref_m3_s2 > 0.0) || !std::isfinite(mu_ref_m3_s2))
        {
            return;
        }

        std::vector<orbitsim::TrajectorySample> rel_samples;
        if (display.resolved_frame_spec_valid &&
            display.resolved_frame_spec.type == orbitsim::TrajectoryFrameType::BodyCenteredInertial &&
            display.resolved_frame_spec.primary_body_id == analysis_body_id)
        {
            rel_samples = display.trajectory_frame;
            analysis.trajectory_segments_analysis_bci = display.trajectory_segments_frame;
            analysis.trajectory_analysis_bci = rel_samples;
            analysis.analysis_cache_body_id = analysis_body_id;
            analysis.analysis_cache_valid = true;
        }
        else if (analysis.analysis_cache_valid && analysis.analysis_cache_body_id == analysis_body_id)
        {
            rel_samples = analysis.trajectory_analysis_bci;
        }
        else if (base_ephemeris && !base_ephemeris->empty())
        {
            const std::size_t sample_budget = std::max<std::size_t>(base_samples.size(), 2);
            const orbitsim::TrajectoryFrameSpec analysis_frame =
                    orbitsim::TrajectoryFrameSpec::body_centered_inertial(analysis_body_id);
            const orbitsim::FrameSegmentTransformOptions frame_opt =
                    PredictionCacheInternal::build_frame_segment_transform_options(
                            analysis_frame,
                            base_segments,
                            cancel_requested);
            std::vector<orbitsim::TrajectorySegment> rel_segments =
                    orbitsim::transform_trajectory_segments_to_frame_spec(
                            base_segments,
                            *base_ephemeris,
                            base_bodies,
                            analysis_frame,
                            frame_opt);
            if (cancel_requested && cancel_requested())
            {
                return;
            }
            rel_samples = sample_prediction_segments(rel_segments, sample_budget);
            analysis.trajectory_segments_analysis_bci = std::move(rel_segments);
            analysis.trajectory_analysis_bci = rel_samples;
            analysis.analysis_cache_body_id = analysis_body_id;
            analysis.analysis_cache_valid = !analysis.trajectory_analysis_bci.empty();
        }

        if (rel_samples.size() < 2)
        {
            return;
        }

        analysis.altitude_km.reserve(rel_samples.size());
        analysis.speed_kmps.reserve(rel_samples.size());
        for (const orbitsim::TrajectorySample &sample : rel_samples)
        {
            const double r_m = glm::length(sample.position_m);
            const double alt_km = (r_m - analysis_it->radius_m) * 1.0e-3;
            const double spd_kmps = glm::length(sample.velocity_mps) * 1.0e-3;
            analysis.altitude_km.push_back(static_cast<float>(alt_km));
            analysis.speed_kmps.push_back(static_cast<float>(spd_kmps));
        }

        const OrbitPredictionMath::OrbitalElementsEstimate elements =
                OrbitPredictionMath::compute_orbital_elements(mu_ref_m3_s2,
                                                              rel_samples.front().position_m,
                                                              rel_samples.front().velocity_mps);
        if (!elements.valid)
        {
            return;
        }

        analysis.semi_major_axis_m = elements.semi_major_axis_m;
        analysis.eccentricity = elements.eccentricity;
        analysis.orbital_period_s = elements.orbital_period_s;
        analysis.periapsis_alt_km = (elements.periapsis_m - analysis_it->radius_m) * 1.0e-3;
        analysis.apoapsis_alt_km = std::isfinite(elements.apoapsis_m)
                                           ? (elements.apoapsis_m - analysis_it->radius_m) * 1.0e-3
                                           : std::numeric_limits<double>::infinity();

        const PredictionRadialExtrema radial_extrema =
                find_prediction_radial_extrema(analysis.trajectory_segments_analysis_bci);
        if (radial_extrema.periapsis_valid)
        {
            analysis.periapsis_alt_km = (radial_extrema.periapsis_radius_m - analysis_it->radius_m) * 1.0e-3;
        }
        if (radial_extrema.apoapsis_valid)
        {
            analysis.apoapsis_alt_km = (radial_extrema.apoapsis_radius_m - analysis_it->radius_m) * 1.0e-3;
        }
    }

    void PredictionMetricsBuilder::rebuild(
            OrbitPredictionCache &cache,
            const orbitsim::GameSimulation::Config &sim_config,
            const orbitsim::BodyId analysis_body_id,
            const CancelCheck &cancel_requested)
    {
        rebuild(cache.solver,
                cache.display,
                cache.analysis,
                sim_config,
                analysis_body_id,
                cancel_requested);
    }
} // namespace Game
