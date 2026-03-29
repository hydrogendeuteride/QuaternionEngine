#pragma once

#include <cstddef>

namespace Game::OrbitPredictionTuning
{
    inline constexpr double kSecondsPerMinute = 60.0;
    inline constexpr double kSecondsPerHour = 3'600.0;
    inline constexpr double kSecondsPerDay = 86'400.0;
    inline constexpr double kSecondsPerYear = 365.0 * kSecondsPerDay;

    // Scale applied to the estimated orbital period when selecting the default prediction horizon.
    // 2.0 means "about two periods" for closed orbits (unless clamped by other limits).
    inline constexpr double kBaseHorizonFromPeriodScale = 2.0;

    // How many orbital periods to draw when full-orbit visualization is enabled.
    inline constexpr double kFullOrbitDrawPeriodScale = 2.0;

    // Fallback orbital period used when orbit elements cannot be estimated reliably.
    inline constexpr double kEscapeDefaultPeriodS = 7'200.0;

    // Minimum prediction horizon (seconds).
    inline constexpr double kMinHorizonS = 60.0;

    // ── Adaptive spacecraft root-segment tolerance ──────────────────────────────
    inline constexpr double kAdaptiveSegmentMinDtS = 1.0;
    inline constexpr double kAdaptiveSegmentMinDtControlledS = 0.05;
    inline constexpr double kAdaptiveSegmentMaxDtS = 43'200.0;           // 12 hours
    inline constexpr double kAdaptiveSegmentMaxDtControlledS = 300.0;    // 5 min
    inline constexpr double kAdaptiveSegmentLookupMaxDtS = 1'800.0;      // 30 min
    inline constexpr double kAdaptiveSegmentLookupMaxDtControlledS = 30.0;
    inline constexpr std::size_t kAdaptiveSegmentSoftMaxSegmentsNormal = 3'000;
    inline constexpr std::size_t kAdaptiveSegmentHardMaxSegmentsNormal = 6'000;
    inline constexpr double kAdaptiveSegmentPosTolNearM = 10.0;
    inline constexpr double kAdaptiveSegmentPosTolFarM = 10.0;
    inline constexpr double kAdaptiveSegmentVelTolNearMps = 1.0;
    inline constexpr double kAdaptiveSegmentVelTolFarMps = 1.0;
    inline constexpr double kAdaptiveSegmentRelPosFloor = 1.0e-6;
    inline constexpr double kAdaptiveSegmentRelVelFloor = 1.0e-6;

    // ── Adaptive celestial ephemeris tolerance ──────────────────────────────────
    inline constexpr double kAdaptiveEphemerisMinDtS = 1.0;
    inline constexpr double kAdaptiveEphemerisMinDtControlledS = 0.1;
    inline constexpr double kAdaptiveEphemerisMaxDtS = 600.0;            // 10 min
    inline constexpr double kAdaptiveEphemerisMaxDtControlledS = 30.0;
    inline constexpr std::size_t kAdaptiveEphemerisSoftMaxSegments = 3'000;
    inline constexpr double kAdaptiveEphemerisPosTolNearM = 10.0;
    inline constexpr double kAdaptiveEphemerisPosTolFarM = 10.0;
    inline constexpr double kAdaptiveEphemerisVelTolNearMps = 1.0;
    inline constexpr double kAdaptiveEphemerisVelTolFarMps = 1.0;
    inline constexpr double kAdaptiveEphemerisRelPosFloor = 1.0e-7;
    inline constexpr double kAdaptiveEphemerisRelVelFloor = 1.0e-7;

    // ── Adaptive frame transform tolerance ──────────────────────────────────────
    inline constexpr double kAdaptiveFrameTransformMinDtS = 1.0;
    inline constexpr double kAdaptiveFrameTransformMaxDtS = 43'200.0;    // 12 hours
    inline constexpr double kAdaptiveFrameTransformMaxDtSensitiveS = 600.0;
    inline constexpr std::size_t kAdaptiveFrameTransformSoftMaxSegments = 3'000;
    inline constexpr std::size_t kAdaptiveFrameTransformHardMaxSegments = 6'000;
    inline constexpr double kAdaptiveFrameTransformPosTolNearM = 10.0;
    inline constexpr double kAdaptiveFrameTransformPosTolFarM = 10.0;
    inline constexpr double kAdaptiveFrameTransformVelTolNearMps = 1.0;
    inline constexpr double kAdaptiveFrameTransformVelTolFarMps = 1.0;
    inline constexpr double kAdaptiveFrameTransformRelPosFloor = 1.0e-6;
    inline constexpr double kAdaptiveFrameTransformRelVelFloor = 1.0e-6;

    // ── Integrator safety rails ─────────────────────────────────────────────────
    inline constexpr double kPredictionIntegratorMaxStepS = 600.0;       // 10 min
    inline constexpr double kPredictionIntegratorMaxStepControlledS = 30.0;
    inline constexpr double kPredictionIntegratorMaxStepPreviewS = 120.0;
    inline constexpr int kPredictionIntegratorMaxSubstepsSoft = 128;
    inline constexpr int kPredictionIntegratorMaxSubstepsHard = 512;
    inline constexpr int kPredictionIntegratorMaxIntervalSplits = 10;

    // ── Synodic/Lagrange sensitivity ────────────────────────────────────────────
    inline constexpr double kLagrangeIntegratorMaxStepS = 300.0;
    inline constexpr double kLagrangeIntegratorAbsTol = 1.0e-6;
    inline constexpr double kLagrangeIntegratorRelTol = 1.0e-6;
    inline constexpr double kLagrangeEphemerisMaxDtS = 600.0;
    inline constexpr std::size_t kLagrangeEphemerisMaxSamples = 24'000;

    // ── Misc policy ─────────────────────────────────────────────────────────────
    inline constexpr double kPrimaryBodyHysteresisKeepRatio = 0.90;
    inline constexpr double kPostNodeCoverageMinS = 120.0;
    inline constexpr double kManeuverRefreshS = 0.1;
    // Chunk planner bands stay finer-grained than the Exact/Near/Tail profile classes.
    inline constexpr double kPredictionChunkBandNearEndS = 3.0 * kSecondsPerDay;
    inline constexpr double kPredictionChunkBandTransferEndS = 30.0 * kSecondsPerDay;
    inline constexpr double kPredictionChunkBandCruiseFineEndS = 180.0 * kSecondsPerDay;
    inline constexpr double kPredictionChunkBandCruiseEndS = 2.0 * kSecondsPerYear;
    inline constexpr double kPredictionChunkBandDeepTailEndS = 20.0 * kSecondsPerYear;
    inline constexpr double kPredictionChunkSpanNearS = 6.0 * kSecondsPerHour;
    inline constexpr double kPredictionChunkSpanTransferS = 1.0 * kSecondsPerDay;
    inline constexpr double kPredictionChunkSpanCruiseFineS = 5.0 * kSecondsPerDay;
    inline constexpr double kPredictionChunkSpanCruiseS = 30.0 * kSecondsPerDay;
    inline constexpr double kPredictionChunkSpanDeepTailS = 90.0 * kSecondsPerDay;

    // Slice 6 activity probe and seam-retry tuning.
    inline constexpr double kPredictionActivityProbeHeadingPromoteRad = 0.08;
    inline constexpr double kPredictionActivityProbeHeadingSplitRad = 0.20;
    inline constexpr double kPredictionActivityProbeNormalizedJerkPromote = 0.35;
    inline constexpr double kPredictionActivityProbeNormalizedJerkSplit = 0.90;
    inline constexpr double kPredictionActivityProbeDominantGravityPromoteRatio = 0.92;
    inline constexpr double kPredictionActivityProbeDominantGravitySplitRatio = 0.75;
    inline constexpr double kPredictionActivityProbeMinSplitSpanS = 2.0 * kSecondsPerHour;
    inline constexpr std::size_t kPredictionSeamRetryMaxAttempts = 3u;
    inline constexpr double kPredictionSeamPosToleranceFloorM = 25.0;
    inline constexpr double kPredictionSeamPosToleranceScale = 1.0e-8;
    inline constexpr double kPredictionSeamVelToleranceFloorMps = 0.05;
    inline constexpr double kPredictionSeamVelToleranceScale = 1.0e-6;

    // While thrusting, shorten the horizon so updates can happen more often.
    inline constexpr double kThrustHorizonMinS = 120.0;
    inline constexpr double kThrustHorizonWindowScale = 1.25;
    inline constexpr double kThrustHorizonMaxS = 172'800.0; // 48 hours

    // Maneuver-gizmo drag rebuild cap.
    inline constexpr double kDragRebuildMinIntervalS = 0.02; // ~50 Hz
    inline constexpr double kDragStalePreviewGraceS = 0.10;
    inline constexpr double kDragInteractivePreviewWindowMaxS = 43'200.0; // 12 hours
}
