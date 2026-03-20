#pragma once

#include <cstddef>

namespace Game::OrbitPredictionTuning
{
    // Scale applied to the estimated orbital period when selecting the default prediction horizon.
    // 2.0 means "about two periods" for closed orbits (unless clamped by other limits).
    inline constexpr double kBaseHorizonFromPeriodScale = 2.0;

    // How many orbital periods to draw when full-orbit visualization is enabled.
    inline constexpr double kFullOrbitDrawPeriodScale = 2.0;

    // Fallback orbital period used when orbit elements cannot be estimated reliably.
    inline constexpr double kEscapeDefaultPeriodS = 7'200.0;

    // Global horizon bounds (in seconds) used by both gameplay and worker prediction code.
    inline constexpr double kMinHorizonS = 60.0;
    inline constexpr double kMaxHorizonS = 15'552'000.0; // 180 days
    inline constexpr double kLongRangeHorizonThresholdS = kMaxHorizonS;
    inline constexpr double kLongRangeHorizonCapS = 631'152'000.0; // 20 years

    // Generic trajectory sampling limits.
    inline constexpr double kMaxSampleDtS = 900.0;
    inline constexpr double kLongRangeMaxSampleDtS = 21'600.0; // 6 hours
    inline constexpr double kTargetSamplesDivisorS = 2.0; // horizon / 2
    inline constexpr double kTargetSamplesMin = 500.0;
    inline constexpr double kTargetSamplesMax = 24'000.0;
    inline constexpr int kMaxStepsNormal = 24'000;
    inline constexpr double kLongRangeSegmentTargetDtS = 3'600.0; // 1 hour
    inline constexpr std::size_t kLongRangeMaxSegmentsHard = 96'000;

    // Adaptive root-segment generation keeps solver fidelity independent from UI sample counts.
    inline constexpr double kAdaptiveSegmentMinDtS = 0.1;
    inline constexpr double kAdaptiveSegmentMinDtControlledS = 0.02;
    inline constexpr double kAdaptiveSegmentMaxDtS = 900.0;
    inline constexpr double kAdaptiveSegmentMaxDtLongRangeS = 21'600.0; // 6 hours
    inline constexpr double kAdaptiveSegmentMaxDtControlledS = 120.0;
    inline constexpr double kAdaptiveSegmentLookupMaxDtS = 60.0;
    inline constexpr double kAdaptiveSegmentLookupMaxDtLongRangeS = 900.0;
    inline constexpr double kAdaptiveSegmentLookupMaxDtControlledS = 10.0;
    inline constexpr std::size_t kAdaptiveSegmentSoftMaxSegmentsNormal = 12'000;
    inline constexpr std::size_t kAdaptiveSegmentHardMaxSegmentsNormal = 24'000;
    inline constexpr double kAdaptiveSegmentPosTolNearM = 1.0;
    inline constexpr double kAdaptiveSegmentPosTolFarM = 10'000.0;
    inline constexpr double kAdaptiveSegmentVelTolNearMps = 1.0e-3;
    inline constexpr double kAdaptiveSegmentVelTolFarMps = 10.0;
    inline constexpr double kAdaptiveSegmentRelPosFloor = 1.0e-8;
    inline constexpr double kAdaptiveSegmentRelVelFloor = 1.0e-8;

    // Adaptive ephemeris generation uses a separate tolerance/cadence surface from spacecraft roots.
    inline constexpr double kAdaptiveEphemerisMinDtS = 0.1;
    inline constexpr double kAdaptiveEphemerisMinDtControlledS = 0.02;
    inline constexpr double kAdaptiveEphemerisMaxDtS = 30.0;
    inline constexpr double kAdaptiveEphemerisMaxDtLongRangeS = 120.0;
    inline constexpr double kAdaptiveEphemerisMaxDtControlledS = 10.0;
    inline constexpr std::size_t kAdaptiveEphemerisSoftMaxSegments = 24'000;
    inline constexpr double kAdaptiveEphemerisPosTolNearM = 1.0;
    inline constexpr double kAdaptiveEphemerisPosTolFarM = 1'000.0;
    inline constexpr double kAdaptiveEphemerisVelTolNearMps = 1.0e-4;
    inline constexpr double kAdaptiveEphemerisVelTolFarMps = 1.0;
    inline constexpr double kAdaptiveEphemerisRelPosFloor = 1.0e-9;
    inline constexpr double kAdaptiveEphemerisRelVelFloor = 1.0e-9;

    // Frame transforms may need their own re-segmentation in rotating or LVLH frames.
    inline constexpr double kAdaptiveFrameTransformMinDtS = 0.1;
    inline constexpr double kAdaptiveFrameTransformMaxDtS = 900.0;
    inline constexpr double kAdaptiveFrameTransformMaxDtLongRangeS = 21'600.0; // 6 hours
    inline constexpr double kAdaptiveFrameTransformMaxDtSensitiveS = 300.0;
    inline constexpr std::size_t kAdaptiveFrameTransformSoftMaxSegments = 24'000;
    inline constexpr std::size_t kAdaptiveFrameTransformHardMaxSegments = 96'000;
    inline constexpr double kAdaptiveFrameTransformPosTolNearM = 1.0;
    inline constexpr double kAdaptiveFrameTransformPosTolFarM = 10'000.0;
    inline constexpr double kAdaptiveFrameTransformVelTolNearMps = 1.0e-3;
    inline constexpr double kAdaptiveFrameTransformVelTolFarMps = 10.0;
    inline constexpr double kAdaptiveFrameTransformRelPosFloor = 1.0e-8;
    inline constexpr double kAdaptiveFrameTransformRelVelFloor = 1.0e-8;

    // Multi-band cache sampling redistributes a fixed sample budget so near-term motion stays
    // dense while long-range tails become progressively coarser.
    inline constexpr double kMultiBandNearDurationS = 604'800.0;   // 7 days
    inline constexpr double kMultiBandMidDurationS = 5'184'000.0;  // 60 days
    inline constexpr double kMultiBandNearDensity = 4.0;
    inline constexpr double kMultiBandMidDensity = 1.0;
    inline constexpr double kMultiBandFarDensity = 0.25;

    // Maneuver-node sampling windows: high-density region around each burn.
    inline constexpr double kMultiBandNodeWindowHalfWidthS = 3'600.0; // 1 hour each side
    inline constexpr double kMultiBandNodeDensity = 8.0;              // 2x Near density

    // Normal spacecraft predictions start with a softer budget, but can grow up to the global
    // hard caps when long horizons or authored maneuver plans demand it.
    inline constexpr double kSpacecraftTargetSamplesSoftNormal = 12'000.0;
    inline constexpr double kSpacecraftTargetSamplesHardNormal = kTargetSamplesMax;
    inline constexpr double kSpacecraftTargetSamplesBonusPerManeuver = 1'000.0;
    inline constexpr int kSpacecraftMaxStepsSoftNormal = 12'000;
    inline constexpr int kSpacecraftMaxStepsHardNormal = kMaxStepsNormal;
    inline constexpr int kSpacecraftMaxStepsBonusPerManeuver = 1'000;

    // Prediction runs on a background worker, so it can afford a tighter spacecraft integrator budget
    // than the live simulation. These caps reduce phase drift on long arcs without changing runtime stepping.
    inline constexpr double kPredictionIntegratorMaxStepS = 60.0;
    inline constexpr double kPredictionIntegratorMaxStepLongRangeS = 180.0;
    inline constexpr double kPredictionIntegratorMaxStepControlledS = 15.0;
    inline constexpr int kPredictionIntegratorMaxSubstepsSoft = 128;
    inline constexpr int kPredictionIntegratorMaxSubstepsHard = 512;
    inline constexpr int kPredictionIntegratorMaxIntervalSplits = 10;

    // Build body ephemerides at a finer cadence than the final draw samples so spacecraft propagation
    // sees a smoother gravity field, especially for multi-body and maneuver planning views.
    inline constexpr double kPredictionEphemerisMaxDtS = 30.0;
    inline constexpr double kPredictionEphemerisMaxDtLongRangeS = 120.0;
    inline constexpr double kPredictionEphemerisMaxDtControlledS = 10.0;
    inline constexpr std::size_t kPredictionEphemerisMaxSegmentsHard = 96'000;

    // Synodic/Lagrange analysis is sensitive to phase drift. Keep spacecraft steps and body ephemerides tighter.
    inline constexpr double kLagrangeIntegratorMaxStepS = 60.0;
    inline constexpr double kLagrangeIntegratorAbsTol = 1.0e-4;
    inline constexpr double kLagrangeIntegratorRelTol = 1.0e-10;
    inline constexpr double kLagrangeEphemerisMaxDtS = 30.0;
    inline constexpr std::size_t kLagrangeEphemerisMaxSamples = 96'000;

    // Keep the currently selected primary body while its gravity metric remains close to the best candidate.
    inline constexpr double kPrimaryBodyHysteresisKeepRatio = 0.90;

    // Extra coverage requested after the last maneuver node so the post-node path stays visible.
    inline constexpr double kPostNodeCoverageMinS = 120.0;

    // While a maneuver plan is present, refresh the async prediction often enough that the planned
    // path stays anchored to the ship's live state rather than the snapshot from when the plan was authored.
    inline constexpr double kManeuverRefreshS = 0.1;

    // While thrusting, shorten the horizon so updates can happen more often.
    inline constexpr double kThrustHorizonMinS = 120.0;
    inline constexpr double kThrustHorizonWindowScale = 1.25;
    inline constexpr double kThrustHorizonMaxS = 172'800.0; // 48 hours
    inline constexpr double kThrustTargetSamplesMin = 300.0;
    inline constexpr double kThrustTargetSamplesMax = 3'000.0;
    inline constexpr double kThrustMinSampleDtS = 0.02;
    inline constexpr double kThrustMaxSampleDtS = 120.0;
    inline constexpr int kMaxStepsThrust = 24'000;

    // Maneuver-gizmo drag rebuild cap. Lower is more responsive, higher reduces CPU work.
    inline constexpr double kDragRebuildMinIntervalS = 0.03; // ~33 Hz
}
