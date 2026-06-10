#pragma once

#include "calibration/calibration_types.h"
#include "core/config.h"
#include "io/steamvr_provider.h"
#include "tracking/body_state.h"
#include "tracking/tracker_synthesis.h"

#include <array>
#include <cstddef>
#include <string>
#include <vector>

namespace bt {

// Required samples come first; optional samples (forward + upper-body + knees) may be skipped.
enum class SteamVrAlignmentLandmark {
    LeftFoot = 0,
    RightFoot,
    Pelvis,
    Floor,
    Forward,    // optional yaw reference
    Chest,      // optional chest role-offset refinement
    LeftElbow,  // optional upper-body role-offset refinement
    RightElbow, // optional upper-body role-offset refinement
    LeftKnee,   // optional knee role-offset refinement
    RightKnee,  // optional knee role-offset refinement
};

inline constexpr std::size_t kSteamVrAlignmentLandmarkCount = 10;

inline const char* ToString(SteamVrAlignmentLandmark landmark) {
    switch (landmark) {
    case SteamVrAlignmentLandmark::LeftFoot: return "left_foot_marker";
    case SteamVrAlignmentLandmark::RightFoot: return "right_foot_marker";
    case SteamVrAlignmentLandmark::Pelvis: return "pelvis_marker";
    case SteamVrAlignmentLandmark::Floor: return "floor_point";
    case SteamVrAlignmentLandmark::Forward: return "forward_reference";
    case SteamVrAlignmentLandmark::Chest: return "chest_marker";
    case SteamVrAlignmentLandmark::LeftElbow: return "left_elbow_marker";
    case SteamVrAlignmentLandmark::RightElbow: return "right_elbow_marker";
    case SteamVrAlignmentLandmark::LeftKnee: return "left_knee_marker";
    case SteamVrAlignmentLandmark::RightKnee: return "right_knee_marker";
    default: return "unknown";
    }
}

inline const char* LandmarkKey(SteamVrAlignmentLandmark landmark) {
    switch (landmark) {
    case SteamVrAlignmentLandmark::LeftFoot: return "left_foot";
    case SteamVrAlignmentLandmark::RightFoot: return "right_foot";
    case SteamVrAlignmentLandmark::Pelvis: return "pelvis";
    case SteamVrAlignmentLandmark::Floor: return "floor";
    case SteamVrAlignmentLandmark::Forward: return "forward";
    case SteamVrAlignmentLandmark::Chest: return "chest";
    case SteamVrAlignmentLandmark::LeftElbow: return "left_elbow";
    case SteamVrAlignmentLandmark::RightElbow: return "right_elbow";
    case SteamVrAlignmentLandmark::LeftKnee: return "left_knee";
    case SteamVrAlignmentLandmark::RightKnee: return "right_knee";
    default: return "unknown";
    }
}

inline std::size_t LandmarkSlotIndex(SteamVrAlignmentLandmark landmark) {
    return static_cast<std::size_t>(landmark);
}

inline bool LandmarkIsRequired(SteamVrAlignmentLandmark landmark) {
    return landmark == SteamVrAlignmentLandmark::LeftFoot ||
        landmark == SteamVrAlignmentLandmark::RightFoot ||
        landmark == SteamVrAlignmentLandmark::Pelvis ||
        landmark == SteamVrAlignmentLandmark::Floor;
}

// Stable, machine-readable failure reasons. Keep this list in sync with ToString().
enum class SteamVrAlignmentReason {
    None = 0,
    SteamVrUnavailable,
    ProviderUnavailable,
    ProviderCompileDisabled,
    ControllersMissing,
    ControllerPoseInvalid,
    BodyStateUnstable,
    NoUnifiedBodyState,
    BodyCalibrationMissing,
    FloorCalibrationMissing,
    BodyLandmarkMissing,
    SampleConfidenceLow,
    SampleNonfinite,
    NotEnoughSamples,
    LeftRightMismatch,
    SampleTooFarFromSolved,
    YawAmbiguous,
    ScaleMismatch,
    FloorHeightMismatch,
    TransformResidualTooHigh,
    AlignmentStale,
    RoleOffsetMissing,
    ActiveTransformInvalid,
    Idle,
    Sampling,
    Accepted,
    Valid,
    Weak,
    Failed,
};

inline const char* ToString(SteamVrAlignmentReason r) {
    switch (r) {
    case SteamVrAlignmentReason::None: return "";
    case SteamVrAlignmentReason::SteamVrUnavailable: return "steamvr_unavailable";
    case SteamVrAlignmentReason::ProviderUnavailable: return "provider_unavailable";
    case SteamVrAlignmentReason::ProviderCompileDisabled: return "provider_compile_disabled";
    case SteamVrAlignmentReason::ControllersMissing: return "controllers_missing";
    case SteamVrAlignmentReason::ControllerPoseInvalid: return "controller_pose_invalid";
    case SteamVrAlignmentReason::BodyStateUnstable: return "body_state_unstable";
    case SteamVrAlignmentReason::NoUnifiedBodyState: return "no_valid_unified_body_state";
    case SteamVrAlignmentReason::BodyCalibrationMissing: return "body_calibration_missing";
    case SteamVrAlignmentReason::FloorCalibrationMissing: return "floor_calibration_missing";
    case SteamVrAlignmentReason::BodyLandmarkMissing: return "body_landmark_missing";
    case SteamVrAlignmentReason::SampleConfidenceLow: return "sample_confidence_low";
    case SteamVrAlignmentReason::SampleNonfinite: return "sample_nonfinite";
    case SteamVrAlignmentReason::NotEnoughSamples: return "not_enough_samples";
    case SteamVrAlignmentReason::LeftRightMismatch: return "left_right_mismatch";
    case SteamVrAlignmentReason::SampleTooFarFromSolved: return "sample_too_far_from_solved_landmark";
    case SteamVrAlignmentReason::YawAmbiguous: return "yaw_ambiguous";
    case SteamVrAlignmentReason::ScaleMismatch: return "scale_mismatch";
    case SteamVrAlignmentReason::FloorHeightMismatch: return "floor_height_mismatch";
    case SteamVrAlignmentReason::TransformResidualTooHigh: return "transform_residual_too_high";
    case SteamVrAlignmentReason::AlignmentStale: return "alignment_stale";
    case SteamVrAlignmentReason::RoleOffsetMissing: return "role_offset_missing";
    case SteamVrAlignmentReason::ActiveTransformInvalid: return "active_transform_invalid";
    case SteamVrAlignmentReason::Idle: return "idle";
    case SteamVrAlignmentReason::Sampling: return "sampling";
    case SteamVrAlignmentReason::Accepted: return "accepted";
    case SteamVrAlignmentReason::Valid: return "valid";
    case SteamVrAlignmentReason::Weak: return "weak";
    case SteamVrAlignmentReason::Failed: return "failed";
    default: return "unknown";
    }
}

struct SteamVrAlignmentSample {
    SteamVrAlignmentLandmark landmark = SteamVrAlignmentLandmark::LeftFoot;
    SteamVrControllerRole controller = SteamVrControllerRole::Unknown;
    Pose3f steamvr_pose{};
    Vec3f camera_landmark{};
    double timestamp_seconds = 0.0;
    double pose_age_seconds = 0.0;
    bool controller_valid = false;
    bool body_state_valid = false;
    float confidence = 0.0f;
    bool accepted = false;
    float residual_m = 0.0f;
    SteamVrAlignmentReason reason_code = SteamVrAlignmentReason::Idle;
    std::string reason = "not_sampled";
};

struct SteamVrAlignmentSession {
    bool active = false;
    std::array<SteamVrAlignmentSample, kSteamVrAlignmentLandmarkCount> samples{};
    int accepted_sample_count = 0;
    std::string status = "idle";
    std::string reason;
    SteamVrAlignmentReason reason_code = SteamVrAlignmentReason::Idle;
};

struct SteamVrAlignmentSolveResult {
    bool valid = false;
    std::string status = "failed";
    std::string reason;
    SteamVrAlignmentReason reason_code = SteamVrAlignmentReason::Failed;
    Vec3f tracker_space_position_offset{};
    Quatf tracker_space_rotation{};
    float tracker_space_scale = 1.0f;
    std::array<Vec3f, kTrackerPoseCount> role_offsets{};
    std::array<bool, kTrackerPoseCount> role_offsets_present{};
    float confidence = 0.0f;
    float residual_m = 0.0f;
    float floor_residual_m = 0.0f;
    float yaw_offset_rad = 0.0f;
    float yaw_disagreement_rad = 0.0f;
    float scale_ratio = 1.0f;
    float scale_mismatch = 0.0f;
    int required_samples_present = 0;
    int total_samples_accepted = 0;
    std::string body_signature;
    std::string floor_signature;
};

// "active transform source" enum used in OSC reports/UI/debug.
enum class TrackerSpaceSource {
    None = 0,        // OSC enabled, no usable transform
    Manual,          // manual config / manual_json_file
    SteamVrController,
    StaleSteamVr,    // controller alignment exists but stale / not currently active
    Unavailable,     // provider/runtime unavailable
    Unknown,         // config string was not one of the known source enums
};

inline const char* ToString(TrackerSpaceSource src) {
    switch (src) {
    case TrackerSpaceSource::None: return "none";
    case TrackerSpaceSource::Manual: return "manual";
    case TrackerSpaceSource::SteamVrController: return "steamvr_controller_alignment";
    case TrackerSpaceSource::StaleSteamVr: return "steamvr_controller_alignment_stale";
    case TrackerSpaceSource::Unavailable: return "unavailable";
    case TrackerSpaceSource::Unknown: return "unknown";
    default: return "unknown";
    }
}

inline TrackerSpaceSource ParseTrackerSpaceSource(const std::string& s) {
    if (s == "steamvr_controller_alignment") return TrackerSpaceSource::SteamVrController;
    if (s == "steamvr_controller_alignment_stale") return TrackerSpaceSource::StaleSteamVr;
    if (s == "unavailable") return TrackerSpaceSource::Unavailable;
    if (s == "manual" || s == "manual_json_file") return TrackerSpaceSource::Manual;
    if (s == "none" || s.empty()) return TrackerSpaceSource::None;
    return TrackerSpaceSource::Unknown;
}

// Aggregated, serializable alignment status. Shared by debug snapshot, UI state JSON,
// and replay export so the four exporters never disagree.
struct SteamVrAlignmentStatus {
    bool provider_available = false;
    bool provider_runtime_initialized = false;
    bool left_controller_tracked = false;
    bool right_controller_tracked = false;
    bool left_trigger_pressed = false;
    bool right_trigger_pressed = false;
    bool left_trigger_pressed_edge = false;
    bool right_trigger_pressed_edge = false;
    int controller_device_count = 0;
    double last_pose_age_seconds = 0.0;
    double max_allowed_pose_age_seconds = 0.25;
    bool provider_hard_unavailable = false;
    bool provider_compile_disabled = false;
    bool controller_alignment_fresh = false;
    std::string provider_status = "unavailable";
    std::string provider_reason = "SteamVR provider not initialized";

    bool session_active = false;
    int accepted_sample_count = 0;
    int total_samples_recorded = 0;
    int required_samples_present = 0;
    bool required_samples_complete = false;

    // Solve result snapshot.
    bool transform_valid = false;
    bool stale = false;
    bool role_offsets_present = false;
    bool source_known = true;
    std::string raw_active_transform_source;
    std::string state = "missing"; // missing, sampling, valid, weak, failed, stale
    SteamVrAlignmentReason reason_code = SteamVrAlignmentReason::Idle;
    std::string reason;
    float confidence = 0.0f;
    float residual_m = 0.0f;
    float floor_residual_m = 0.0f;
    float yaw_offset_rad = 0.0f;
    float yaw_disagreement_rad = 0.0f;
    float scale_ratio = 1.0f;
    float scale_mismatch = 0.0f;
    double last_alignment_timestamp = 0.0;

    // Active transform applied to OSC.
    TrackerSpaceSource active_transform_source = TrackerSpaceSource::None;
    bool manual_fallback_available = false;
    bool manual_fallback_active = false;
    std::string stale_reason;
    bool body_calibration_valid = false;
    bool floor_calibration_valid = false;
    bool body_state_stable = false;
    std::string body_signature;
    std::string floor_signature;

    std::vector<SteamVrAlignmentSample> samples; // size <= kSteamVrAlignmentLandmarkCount
};

// Public API.
SteamVrAlignmentSession StartSteamVrAlignmentSession(const SteamVrPoseSnapshot& steamvr);
SteamVrAlignmentSample CaptureSteamVrAlignmentSample(
    SteamVrAlignmentLandmark landmark,
    const SteamVrControllerPose& controller,
    const UnifiedBodyState& body_state,
    const CalibrationBundle& calibration,
    double timestamp_seconds);
void StoreSteamVrAlignmentSample(SteamVrAlignmentSession& session, const SteamVrAlignmentSample& sample);
// Redo-only clears a single landmark slot, preserving the rest of the session.
void RedoSteamVrAlignmentSample(SteamVrAlignmentSession& session, SteamVrAlignmentLandmark landmark);
SteamVrAlignmentSolveResult SolveSteamVrAlignment(
    const SteamVrAlignmentSession& session,
    const UnifiedBodyState& body_state,
    const CalibrationBundle& calibration);
bool ManualTrackerSpaceFallbackAvailable(const OscConfig& config);
void StoreActiveTrackerSpaceAsManualFallback(OscConfig& config);
void ActivateManualTrackerSpaceFallback(OscConfig& config);
void ApplySteamVrAlignmentToOscConfig(OscConfig& config, const SteamVrAlignmentSolveResult& result);
// Reset controller-driven fields without clobbering manual fallback values.
void ClearSteamVrAlignmentFromOscConfig(OscConfig& config);

// Signatures (versioned text fingerprints) used for stale detection.
std::string SteamVrBodyCalibrationSignature(const BodyCalibration& body);
std::string SteamVrFloorCalibrationSignature(const CalibrationBundle& calibration);
bool SteamVrAlignmentStale(const OscConfig& config, const CalibrationBundle& calibration);

// Latency probe scaffolding (PHASE 10). Hardware-accurate live correlation is gated
// behind real provider integration; this keeps deterministic scaffolding for tests.
enum class LatencyProbeStatus {
    Unavailable = 0,
    Collecting,
    Valid,
    Weak,
    Failed,
};

inline const char* ToString(LatencyProbeStatus s) {
    switch (s) {
    case LatencyProbeStatus::Unavailable: return "unavailable";
    case LatencyProbeStatus::Collecting: return "collecting";
    case LatencyProbeStatus::Valid: return "valid";
    case LatencyProbeStatus::Weak: return "weak";
    case LatencyProbeStatus::Failed: return "failed";
    default: return "unknown";
    }
}

struct LatencyProbeSample {
    double controller_timestamp_seconds = 0.0;
    double body_timestamp_seconds = 0.0;
    Vec3f controller_position{};
    Vec3f body_position{};
};

struct LatencyProbeSession {
    LatencyProbeStatus status = LatencyProbeStatus::Unavailable;
    std::string reason = "provider_unavailable";
    int sample_count = 0;
    float estimated_latency_seconds = 0.0f;
    float confidence = 0.0f;
    std::vector<LatencyProbeSample> samples;
};

LatencyProbeSession StartLatencyProbe(const SteamVrPoseSnapshot& steamvr);
void RecordLatencyProbeSample(LatencyProbeSession& session, const LatencyProbeSample& s);
void FinishLatencyProbe(LatencyProbeSession& session);

// Build a lightweight unified status struct from current backend state.
struct SteamVrAlignmentStatusInputs {
    SteamVrPoseSnapshot provider_snapshot{};
    SteamVrAlignmentSession session{};
    SteamVrAlignmentSolveResult last_solve{};
    OscConfig osc_config{};
    bool body_state_stable = false;
    bool body_calibration_valid = false;
    bool floor_calibration_valid = false;
    std::string body_signature;
    std::string floor_signature;
    bool stale = false;
    double last_alignment_timestamp = 0.0;
};

SteamVrAlignmentStatus BuildSteamVrAlignmentStatus(const SteamVrAlignmentStatusInputs& in);

} // namespace bt
