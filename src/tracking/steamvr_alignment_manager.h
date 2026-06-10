#pragma once

#include "core/config.h"
#include "io/steamvr_provider.h"
#include "tracking/body_state.h"
#include "tracking/steamvr_alignment.h"

#include <memory>
#include <mutex>
#include <string>

namespace bt {

// Backend-owned coordinator for SteamVR controller alignment.
//
// Responsibilities:
//   * own the pose provider (real or fake/null)
//   * own the current alignment session and the last solve result
//   * apply solved alignment back to OscConfig only when valid
//   * preserve manual fallback transform when controller alignment is cleared/stale
//   * surface a single SteamVrAlignmentStatus for debug/UI/replay export
//
// Mutexing: all public methods take an internal mutex. The runtime hot loop
// only calls Poll()/Status() at runtime cadence; it never enters OpenVR from the
// realtime tracking thread.
class SteamVrAlignmentManager {
public:
    SteamVrAlignmentManager();
    explicit SteamVrAlignmentManager(std::unique_ptr<ISteamVrPoseProvider> provider);
    ~SteamVrAlignmentManager();

    SteamVrAlignmentManager(const SteamVrAlignmentManager&) = delete;
    SteamVrAlignmentManager& operator=(const SteamVrAlignmentManager&) = delete;

    // Replace the provider (test entry point).
    void SetProvider(std::unique_ptr<ISteamVrPoseProvider> provider);

    // Latest provider snapshot. Caller may discard or feed into Status().
    SteamVrPoseSnapshot Poll();

    // Alignment commands. All return the latest serializable status.
    SteamVrAlignmentStatus StartSession();
    SteamVrAlignmentStatus RecordSample(
        SteamVrAlignmentLandmark landmark,
        SteamVrControllerRole controller_preference,
        const UnifiedBodyState& body_state,
        const CalibrationBundle& calibration,
        double timestamp_seconds);
    SteamVrAlignmentStatus RedoSample(SteamVrAlignmentLandmark landmark);
    SteamVrAlignmentStatus FinishSession(
        const UnifiedBodyState& body_state,
        const CalibrationBundle& calibration,
        OscConfig& osc_config_in_out,
        double timestamp_seconds);
    SteamVrAlignmentStatus ClearAlignment(OscConfig& osc_config_in_out);

    // Recompute the unified status using current internal state and external context.
    SteamVrAlignmentStatus Status(
        const OscConfig& osc_config,
        const CalibrationBundle& calibration,
        bool body_state_stable);
    bool SessionActive() const;

    // Latency probe management (deterministic scaffolding; live correlation requires hardware).
    LatencyProbeSession StartLatencyProbe();
    void RecordLatencyProbeSample(const LatencyProbeSample& sample);
    LatencyProbeSession FinishLatencyProbe();
    LatencyProbeSession LatencyProbeStatus() const;

    // Backend invariants: returns true if the OSC active source matches a usable
    // backend transform (i.e. controller-derived and not stale, OR manual fallback).
    static bool ActiveTransformIsHonest(const OscConfig& osc_config, bool stale);

private:
    SteamVrAlignmentStatus BuildStatusLocked(
        const OscConfig& osc_config,
        const CalibrationBundle& calibration,
        bool body_state_stable) const;

    mutable std::mutex mutex_;
    std::unique_ptr<ISteamVrPoseProvider> provider_;
    SteamVrPoseSnapshot last_snapshot_{};
    SteamVrAlignmentSession session_{};
    SteamVrAlignmentSolveResult last_solve_{};
    LatencyProbeSession latency_probe_{};
    double last_alignment_timestamp_ = 0.0;
};

} // namespace bt
