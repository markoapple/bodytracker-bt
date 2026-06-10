#pragma once

#include "calibration/calibration_types.h"
#include "core/types.h"
#include "tracking/body_model.h"

#include <array>
#include <cstddef>
#include <string>

namespace bt {

enum class BodyJointRole {
    Pelvis = 0,
    Chest,
    Neck,
    Head,
    LeftShoulder,
    RightShoulder,
    LeftElbow,
    RightElbow,
    LeftWrist,
    RightWrist,
    LeftHip,
    RightHip,
    LeftKnee,
    RightKnee,
    LeftAnkle,
    RightAnkle,
    LeftFoot,
    RightFoot,
    LeftToe,
    RightToe
};

inline constexpr std::size_t kBodyJointRoleCount = 20;
inline constexpr std::array<BodyJointRole, kBodyJointRoleCount> kBodyJointRoles{
    BodyJointRole::Pelvis,
    BodyJointRole::Chest,
    BodyJointRole::Neck,
    BodyJointRole::Head,
    BodyJointRole::LeftShoulder,
    BodyJointRole::RightShoulder,
    BodyJointRole::LeftElbow,
    BodyJointRole::RightElbow,
    BodyJointRole::LeftWrist,
    BodyJointRole::RightWrist,
    BodyJointRole::LeftHip,
    BodyJointRole::RightHip,
    BodyJointRole::LeftKnee,
    BodyJointRole::RightKnee,
    BodyJointRole::LeftAnkle,
    BodyJointRole::RightAnkle,
    BodyJointRole::LeftFoot,
    BodyJointRole::RightFoot,
    BodyJointRole::LeftToe,
    BodyJointRole::RightToe
};

inline std::size_t BodyJointRoleIndex(BodyJointRole role) {
    switch (role) {
    case BodyJointRole::Pelvis: return 0;
    case BodyJointRole::Chest: return 1;
    case BodyJointRole::Neck: return 2;
    case BodyJointRole::Head: return 3;
    case BodyJointRole::LeftShoulder: return 4;
    case BodyJointRole::RightShoulder: return 5;
    case BodyJointRole::LeftElbow: return 6;
    case BodyJointRole::RightElbow: return 7;
    case BodyJointRole::LeftWrist: return 8;
    case BodyJointRole::RightWrist: return 9;
    case BodyJointRole::LeftHip: return 10;
    case BodyJointRole::RightHip: return 11;
    case BodyJointRole::LeftKnee: return 12;
    case BodyJointRole::RightKnee: return 13;
    case BodyJointRole::LeftAnkle: return 14;
    case BodyJointRole::RightAnkle: return 15;
    case BodyJointRole::LeftFoot: return 16;
    case BodyJointRole::RightFoot: return 17;
    case BodyJointRole::LeftToe: return 18;
    case BodyJointRole::RightToe: return 19;
    default: return kBodyJointRoleCount;
    }
}

inline const char* ToString(BodyJointRole role) {
    switch (role) {
    case BodyJointRole::Pelvis: return "pelvis";
    case BodyJointRole::Chest: return "chest";
    case BodyJointRole::Neck: return "neck";
    case BodyJointRole::Head: return "head";
    case BodyJointRole::LeftShoulder: return "left_shoulder";
    case BodyJointRole::RightShoulder: return "right_shoulder";
    case BodyJointRole::LeftElbow: return "left_elbow";
    case BodyJointRole::RightElbow: return "right_elbow";
    case BodyJointRole::LeftWrist: return "left_wrist";
    case BodyJointRole::RightWrist: return "right_wrist";
    case BodyJointRole::LeftHip: return "left_hip";
    case BodyJointRole::RightHip: return "right_hip";
    case BodyJointRole::LeftKnee: return "left_knee";
    case BodyJointRole::RightKnee: return "right_knee";
    case BodyJointRole::LeftAnkle: return "left_ankle";
    case BodyJointRole::RightAnkle: return "right_ankle";
    case BodyJointRole::LeftFoot: return "left_foot";
    case BodyJointRole::RightFoot: return "right_foot";
    case BodyJointRole::LeftToe: return "left_toe";
    case BodyJointRole::RightToe: return "right_toe";
    default: return "unknown";
    }
}

enum class BodyJointVisibility {
    Visible = 0,
    LowConfidence,
    BodyOccluded,
    CameraOccluded,
    FloorOccluded,
    MissingUnknown,
    Predicted,
    Anchored
};

inline const char* ToString(BodyJointVisibility visibility) {
    switch (visibility) {
    case BodyJointVisibility::Visible: return "visible";
    case BodyJointVisibility::LowConfidence: return "low_confidence";
    case BodyJointVisibility::BodyOccluded: return "body_occluded";
    case BodyJointVisibility::CameraOccluded: return "camera_occluded";
    case BodyJointVisibility::FloorOccluded: return "floor_occluded";
    case BodyJointVisibility::MissingUnknown: return "missing_unknown";
    case BodyJointVisibility::Predicted: return "predicted";
    case BodyJointVisibility::Anchored: return "anchored";
    default: return "unknown";
    }
}

enum class BodyFootContactState {
    Swing = 0,
    HeelStrike,
    ToeContact,
    FullPlant,
    SlidingError,
    Unreliable
};

inline const char* ToString(BodyFootContactState state) {
    switch (state) {
    case BodyFootContactState::Swing: return "swing";
    case BodyFootContactState::HeelStrike: return "heel_strike";
    case BodyFootContactState::ToeContact: return "toe_contact";
    case BodyFootContactState::FullPlant: return "full_plant";
    case BodyFootContactState::SlidingError: return "sliding_error";
    case BodyFootContactState::Unreliable: return "unreliable";
    default: return "unknown";
    }
}

struct BodyStateJoint {
    BodyJointRole role = BodyJointRole::Pelvis;
    Vec3f position{};
    Vec3f velocity{};
    float confidence = 0.0f;
    bool valid = false;
    BodyJointVisibility visibility = BodyJointVisibility::MissingUnknown;
    TrackerEvidence evidence{};
    DepthSource depth_source = DepthSource::None;
    bool measured = false;
    bool predicted = false;
    bool camera_a_present = false;
    bool camera_b_present = false;
    float camera_a_confidence = 0.0f;
    float camera_b_confidence = 0.0f;
    float camera_a_weight = 0.0f;
    float camera_b_weight = 0.0f;
    float camera_a_quality = 0.0f;
    float camera_b_quality = 0.0f;
    JointEvidenceSource evidence_source = JointEvidenceSource::None;
    bool triangulated = false;
    bool depth_inferred = false;
    float reprojection_error_px = 0.0f;
    float estimated_depth_m = 0.0f;
    float contact_lock_strength = 0.0f;
    float contact_support_confidence = 0.0f;
    // Telemetry/data-availability flag: true when the solver supplied a
    // geometric/temporal inverse-variance weight scale for this observation.
    // It is not a control-flow flag and does not mean confidence was multiplied
    // by the raw weight scale.
    bool solver_observation_weighted = false;
    // Raw solver-relative inverse-variance scale. Values may exceed 1.0 for
    // precise observations or be near zero for fallback observations; convert
    // through the body-state confidence-ceiling policy before affecting output
    // confidence.
    float solver_observation_weight_scale = 1.0f;
    float solver_observation_confidence_ceiling = 1.0f;
    float identity_confidence = 1.0f;
    std::string reason;
};

struct BodyStateDiagnostics {
    bool active = false;
    bool degraded = false;
    bool triangulation_active = false;
    bool tracking_mode_is_monocular = false;
    bool stereo_fallback_active = false;
    // Legacy field kept for JSON compatibility; no longer means intentional monocular mode.
    bool monocular_fallback = false;
    bool left_right_identity_stable = false;
    bool left_right_identity_uncertain = false;
    bool occlusion_prediction_active = false;
    bool contact_lock_active = false;
    bool floor_support_active = false;
    bool body_calibration_valid = false;
    bool latency_prediction_active = false;
    double latency_prediction_seconds = 0.0;
    int triangulated_count = 0;
    int inferred_depth_count = 0;
    int predicted_joint_count = 0;
    int measured_role_count = 0;
    int anchored_role_count = 0;
    int degraded_role_count = 0;
    int manual_role_count = 0;
    int stale_aged_role_count = 0;
    int invalid_role_count = 0;
    int low_confidence_role_count = 0;
    float mean_reprojection_error_px = 0.0f;
    float role_output_confidence = 0.0f;
    float identity_confidence = 1.0f;
    float left_contact_lock_strength = 0.0f;
    float right_contact_lock_strength = 0.0f;
    std::string tracking_mode = "unknown";
    std::string depth_source = "none";
    std::string reason;
};

struct BodyStateJointMeasurement {
    bool camera_a_present = false;
    bool camera_b_present = false;
    float camera_a_confidence = 0.0f;
    float camera_b_confidence = 0.0f;
    float camera_a_weight = 0.0f;
    float camera_b_weight = 0.0f;
    float camera_a_quality = 0.0f;
    float camera_b_quality = 0.0f;
    JointEvidenceSource evidence_source = JointEvidenceSource::None;
    bool triangulated = false;
    bool depth_inferred = false;
    DepthSource depth_source = DepthSource::None;
    Vec3f world{};
    float confidence = 0.0f;
    float mean_reprojection_error_px = 0.0f;
    float estimated_depth_m = 0.0f;
    float contact_confidence = 0.0f;
    // Data-availability telemetry for solver-relative observation weighting;
    // not a control-flow flag and not a raw confidence multiplier.
    bool solver_observation_weighted = false;
    float solver_observation_weight_scale = 1.0f;
    float solver_observation_confidence_ceiling = 1.0f;
};

struct BodyStateSolverSnapshot {
    TrackingMode tracking_mode = TrackingMode::Stereo;
    DepthSource depth_source = DepthSource::None;
    bool degraded = false;
    bool used_hmd = false;
    std::string reason;
    bool camera_a_identity_swapped = false;
    bool camera_b_identity_swapped = false;
    float camera_a_identity_consistency = 0.0f;
    float camera_b_identity_consistency = 0.0f;
    int triangulated_count = 0;
    int inferred_depth_count = 0;
    float mean_reprojection_error_px = 0.0f;
    std::array<BodyStateJointMeasurement, kHalpe26Count> joints{};
};

struct BodyStateContactEvidence {
    BodyFootContactState contact = BodyFootContactState::Unreliable;
    float support_confidence = 0.0f;
    float lock_strength = 0.0f;
    bool anchor_active = false;
    bool heel_anchor_active = false;
    bool toe_anchor_active = false;
};

struct BodyStateEvidence {
    BodyStateSolverSnapshot solver{};
    BodyStateContactEvidence left_foot{};
    BodyStateContactEvidence right_foot{};
};

struct UnifiedBodyState {
    LowerBodyState lower_body{};
    LowerBodyJointSet joints{};
    std::array<BodyStateJoint, kBodyJointRoleCount> roles{};
    BodyFootContactState left_foot_contact = BodyFootContactState::Unreliable;
    BodyFootContactState right_foot_contact = BodyFootContactState::Unreliable;
    BodyStateDiagnostics diagnostics{};
    bool valid = false;
};

BodyFootContactState BodyContactStateFromSupport(const FootSupportState& support);
BodyStateEvidence BuildBodyStateEvidence(const BodyStateSolverSnapshot& solver, const LowerBodyState& state);

UnifiedBodyState BuildUnifiedBodyState(
    const LowerBodyState& state,
    const LowerBodyModel& model,
    const BodyStateEvidence& evidence,
    const BodyCalibration& body_calibration,
    double dt_seconds,
    bool state_valid);

UnifiedBodyState BuildUnifiedBodyState(
    const LowerBodyState& state,
    const LowerBodyModel& model,
    const BodyStateSolverSnapshot& solver,
    const BodyCalibration& body_calibration,
    double dt_seconds,
    bool state_valid);

} // namespace bt
