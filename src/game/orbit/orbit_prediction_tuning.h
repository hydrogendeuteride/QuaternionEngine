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

    // Generic trajectory sampling limits.
    inline constexpr double kMaxSampleDtS = 900.0;
    inline constexpr double kTargetSamplesDivisorS = 2.0; // horizon / 2
    inline constexpr double kTargetSamplesMin = 500.0;
    inline constexpr double kTargetSamplesMax = 24'000.0;
    inline constexpr int kMaxStepsNormal = 24'000;

    // Normal spacecraft predictions start with a softer budget, but can grow up to the global
    // hard caps when long horizons or authored maneuver plans demand it.
    inline constexpr double kSpacecraftTargetSamplesSoftNormal = 12'000.0;
    inline constexpr double kSpacecraftTargetSamplesHardNormal = kTargetSamplesMax;
    inline constexpr double kSpacecraftTargetSamplesBonusPerManeuver = 1'000.0;
    inline constexpr int kSpacecraftMaxStepsSoftNormal = 12'000;
    inline constexpr int kSpacecraftMaxStepsHardNormal = kMaxStepsNormal;
    inline constexpr int kSpacecraftMaxStepsBonusPerManeuver = 1'000;

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
