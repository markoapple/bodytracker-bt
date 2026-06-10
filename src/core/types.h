#pragma once

#include "core/math.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace bt {

// Canonical solver keypoint count. This is the size of the internal KeypointId
// array consumed by the solver, support model, and tracker synthesis. The
// Cocktail14 133-keypoint model maps to this topology via MapWholeBody133ToInternal26().
constexpr std::size_t kInternalKeypointCount = 26;

// Backward-compatible alias. Prefer kInternalKeypointCount in new code.
constexpr std::size_t kHalpe26Count = kInternalKeypointCount;

enum class CameraId : std::uint8_t {
    A = 0,
    B = 1
};

enum class TrackingMode : std::uint8_t {
    Stereo = 0,
    Monocular = 1
};

enum class DepthSource : std::uint8_t {
    None = 0,
    TriangulatedStereo = 1,
    InferredMonocular = 2
};

enum class JointEvidenceSource : std::uint8_t {
    None = 0,
    Stereo = 1,
    CameraAOnly = 2,
    CameraBOnly = 3,
    TemporalHold = 4,
    Fallback = 5,
    Rejected = 6
};

enum class MonocularScaleSource : std::uint8_t {
    None = 0,
    BodyExtent = 1,
    FloorRay = 2,
    FloorSpacing = 3,
    FloorProjective = 4,
    DefaultDepth = 5,
    WallDepth = 6
};

inline const char* ToString(TrackingMode mode) {
    switch (mode) {
    case TrackingMode::Stereo: return "stereo";
    case TrackingMode::Monocular: return "monocular";
    default: return "unknown";
    }
}

inline const char* ToString(DepthSource source) {
    switch (source) {
    case DepthSource::None: return "none";
    case DepthSource::TriangulatedStereo: return "triangulated_stereo";
    case DepthSource::InferredMonocular: return "inferred_monocular";
    default: return "unknown";
    }
}

inline const char* ToString(JointEvidenceSource source) {
    switch (source) {
    case JointEvidenceSource::None: return "none";
    case JointEvidenceSource::Stereo: return "stereo";
    case JointEvidenceSource::CameraAOnly: return "camera_a_only";
    case JointEvidenceSource::CameraBOnly: return "camera_b_only";
    case JointEvidenceSource::TemporalHold: return "temporal_hold";
    case JointEvidenceSource::Fallback: return "fallback";
    case JointEvidenceSource::Rejected: return "rejected";
    default: return "unknown";
    }
}

inline const char* ToString(MonocularScaleSource source) {
    switch (source) {
    case MonocularScaleSource::None: return "none";
    case MonocularScaleSource::BodyExtent: return "body_extent";
    case MonocularScaleSource::FloorRay: return "floor_ray";
    case MonocularScaleSource::FloorSpacing: return "floor_spacing";
    case MonocularScaleSource::FloorProjective: return "floor_projective";
    case MonocularScaleSource::DefaultDepth: return "default_depth";
    case MonocularScaleSource::WallDepth: return "wall_depth";
    default: return "unknown";
    }
}

enum class KeypointId : std::uint8_t {
    Nose = 0,
    LeftEye = 1,
    RightEye = 2,
    LeftEar = 3,
    RightEar = 4,
    LeftShoulder = 5,
    RightShoulder = 6,
    LeftElbow = 7,
    RightElbow = 8,
    LeftWrist = 9,
    RightWrist = 10,
    LeftHip = 11,
    RightHip = 12,
    LeftKnee = 13,
    RightKnee = 14,
    LeftAnkle = 15,
    RightAnkle = 16,
    HeadTop = 17,
    Neck = 18,
    Pelvis = 19,
    LeftBigToe = 20,
    RightBigToe = 21,
    LeftSmallToe = 22,
    RightSmallToe = 23,
    LeftHeel = 24,
    RightHeel = 25
};

inline const char* ToString(KeypointId id) {
    switch (id) {
    case KeypointId::Nose: return "nose";
    case KeypointId::Neck: return "neck";
    case KeypointId::RightShoulder: return "right_shoulder";
    case KeypointId::RightElbow: return "right_elbow";
    case KeypointId::RightWrist: return "right_wrist";
    case KeypointId::LeftShoulder: return "left_shoulder";
    case KeypointId::LeftElbow: return "left_elbow";
    case KeypointId::LeftWrist: return "left_wrist";
    case KeypointId::RightHip: return "right_hip";
    case KeypointId::RightKnee: return "right_knee";
    case KeypointId::RightAnkle: return "right_ankle";
    case KeypointId::LeftHip: return "left_hip";
    case KeypointId::LeftKnee: return "left_knee";
    case KeypointId::LeftAnkle: return "left_ankle";
    case KeypointId::RightEye: return "right_eye";
    case KeypointId::LeftEye: return "left_eye";
    case KeypointId::RightEar: return "right_ear";
    case KeypointId::LeftEar: return "left_ear";
    case KeypointId::LeftBigToe: return "left_big_toe";
    case KeypointId::LeftSmallToe: return "left_small_toe";
    case KeypointId::LeftHeel: return "left_heel";
    case KeypointId::RightBigToe: return "right_big_toe";
    case KeypointId::RightSmallToe: return "right_small_toe";
    case KeypointId::RightHeel: return "right_heel";
    case KeypointId::HeadTop: return "head";
    case KeypointId::Pelvis: return "hip";
    default: return "unknown";
    }
}

struct Keypoint2D {
    Vec2f pixel{};
    float confidence = 0.0f;
    bool present = false;
};

struct Keypoint3D {
    Vec3f world{};
    float confidence = 0.0f;
    bool present = false;
};

using KeypointArray = std::array<Keypoint2D, kInternalKeypointCount>;
using Keypoint3DArray = std::array<Keypoint3D, kInternalKeypointCount>;

enum class PostureMode : std::uint8_t {
    UprightStanding = 0,
    UprightTransition,
    Crouching,
    Kneeling,
    SeatedSupported,
    ReclinedSupported,
    UnknownFree
};

enum class RootSupportType : std::uint8_t {
    None = 0,
    FeetSupported,
    SeatSupported,
    BodyRestSupported,
    KneeSupported,
    MixedSupported
};

enum class FootSupportType : std::uint8_t {
    None = 0,
    FloorSupport,
    RestSupport
};

enum class FootSupportPhase : std::uint8_t {
    Swing = 0,
    ContactCandidate,
    HeelLock,
    FlatPlant,
    ToePivot,
    ReleasePending,
    Slip,
    RestCandidate,
    RestLock
};

enum class FootContactLoad : std::uint8_t {
    None = 0,
    HeelOnly,
    ToeOnly,
    FullPlant,
    Inferred
};



enum class TrackerEvidenceSource : std::uint8_t {
    None = 0,
    DirectStereo = 1,
    InferredMonocular = 2,
    AnchorHeld = 3,
    HmdPrediction = 4,
    ReplayInput = 5,
    Predicted = 6
};

enum class TrackingSignalKind : std::uint8_t {
    Invalid = 0,
    Measured = 1,
    Predicted = 2,
    Anchored = 3,
    Degraded = 4,
    Manual = 5,
    StaleAged = 6
};

inline const char* ToString(TrackerEvidenceSource source) {
    switch (source) {
    case TrackerEvidenceSource::None: return "none";
    case TrackerEvidenceSource::DirectStereo: return "direct_stereo";
    case TrackerEvidenceSource::InferredMonocular: return "inferred_monocular";
    case TrackerEvidenceSource::AnchorHeld: return "anchor_held";
    case TrackerEvidenceSource::HmdPrediction: return "hmd_prediction";
    case TrackerEvidenceSource::ReplayInput: return "replay_input";
    case TrackerEvidenceSource::Predicted: return "predicted";
    default: return "unknown";
    }
}

inline const char* ToString(TrackingSignalKind kind) {
    switch (kind) {
    case TrackingSignalKind::Invalid: return "invalid";
    case TrackingSignalKind::Measured: return "measured";
    case TrackingSignalKind::Predicted: return "predicted";
    case TrackingSignalKind::Anchored: return "anchored";
    case TrackingSignalKind::Degraded: return "degraded";
    case TrackingSignalKind::Manual: return "manual";
    case TrackingSignalKind::StaleAged: return "stale_aged";
    default: return "unknown";
    }
}

struct TrackerEvidence {
    TrackerEvidenceSource source = TrackerEvidenceSource::None;
    float direct_confidence = 0.0f;
    float support_confidence = 0.0f;
    bool anchor_held = false;
    // Diagnostic flags for fallback/degraded frames. Output gates must still
    // decide from source-specific evidence and confidence, not from these
    // frame-level tags alone.
    bool degraded = false;
    bool stereo_fallback = false;
    bool manual = false;
    bool stale_aged = false;
    bool valid = false;
};

inline TrackingSignalKind EffectiveSignalKind(const TrackerEvidence& evidence) {
    if (!evidence.valid && evidence.source == TrackerEvidenceSource::None && !evidence.anchor_held) {
        return TrackingSignalKind::Invalid;
    }
    if (evidence.manual) {
        return TrackingSignalKind::Manual;
    }
    if (evidence.stale_aged) {
        return TrackingSignalKind::StaleAged;
    }
    if (evidence.anchor_held || evidence.source == TrackerEvidenceSource::AnchorHeld) {
        return TrackingSignalKind::Anchored;
    }
    if (evidence.degraded) {
        return TrackingSignalKind::Degraded;
    }
    switch (evidence.source) {
    case TrackerEvidenceSource::DirectStereo:
    case TrackerEvidenceSource::InferredMonocular:
    case TrackerEvidenceSource::ReplayInput:
        return TrackingSignalKind::Measured;
    case TrackerEvidenceSource::HmdPrediction:
    case TrackerEvidenceSource::Predicted:
        return TrackingSignalKind::Predicted;
    case TrackerEvidenceSource::AnchorHeld:
        return TrackingSignalKind::Anchored;
    case TrackerEvidenceSource::None:
    default:
        return evidence.valid ? TrackingSignalKind::Predicted : TrackingSignalKind::Invalid;
    }
}

struct HmdPoseSample {
    Pose3f pose{};
    double timestamp_seconds = 0.0;
    bool valid = false;
};

struct SupportAnchor {
    Pose3f pose{};
    // Contact history is measured-contact motion while the anchor is active;
    // pose remains the lock target. Solver telemetry uses this for true sliding
    // velocity instead of residual/dt.
    Vec3f previous_contact_position{};
    Vec3f current_contact_position{};
    bool has_contact_history = false;
    float confidence = 0.0f;
    double dwell_seconds = 0.0;
    double release_seconds = 0.0;
    bool active = false;
};

struct FootSupportState {
    FootSupportType type = FootSupportType::None;
    FootSupportPhase phase = FootSupportPhase::Swing;
    FootContactLoad contact_load = FootContactLoad::None;
    float heel_contact_confidence = 0.0f;
    float toe_contact_confidence = 0.0f;
    float transition_quality = 1.0f;
    SupportAnchor anchor{};
    SupportAnchor heel_anchor{};
    SupportAnchor toe_anchor{};
};

struct SupportManifoldState {
    RootSupportType root_support = RootSupportType::None;
    SupportAnchor root_anchor{};
    FootSupportState left_foot{};
    FootSupportState right_foot{};
    SupportAnchor left_knee_anchor{};
    SupportAnchor right_knee_anchor{};
};

struct LowerBodyState {
    Pose3f root{};
    Pose3f left_foot{};
    Pose3f right_foot{};
    float left_hip_flexion = 0.0f;
    float left_hip_abduction = 0.0f;
    float left_knee_flexion = 0.0f;
    float left_ankle_pitch = 0.0f;
    float left_ankle_roll = 0.0f;
    float left_ankle_yaw = 0.0f;
    float right_hip_flexion = 0.0f;
    float right_hip_abduction = 0.0f;
    float right_knee_flexion = 0.0f;
    float right_ankle_pitch = 0.0f;
    float right_ankle_roll = 0.0f;
    float right_ankle_yaw = 0.0f;
    Vec3f linear_velocity{};
    Vec3f angular_velocity{};
    Vec3f left_foot_linear_velocity{};
    Vec3f right_foot_linear_velocity{};
    bool left_leg_reach_clamped = false;
    bool right_leg_reach_clamped = false;
    PostureMode posture_mode = PostureMode::UnknownFree;
    SupportManifoldState support{};
    TrackerEvidence left_foot_evidence{};
    TrackerEvidence right_foot_evidence{};
    float confidence = 0.0f;
};

inline const char* ToString(PostureMode mode) {
    switch (mode) {
    case PostureMode::UprightStanding: return "UPRIGHT_STANDING";
    case PostureMode::UprightTransition: return "UPRIGHT_TRANSITION";
    case PostureMode::Crouching: return "CROUCHING";
    case PostureMode::Kneeling: return "KNEELING";
    case PostureMode::SeatedSupported: return "SEATED_SUPPORTED";
    case PostureMode::ReclinedSupported: return "RECLINED_SUPPORTED";
    case PostureMode::UnknownFree: return "UNKNOWN_FREE";
    default: return "UNKNOWN";
    }
}

inline const char* ToString(RootSupportType type) {
    switch (type) {
    case RootSupportType::None: return "NONE";
    case RootSupportType::FeetSupported: return "FEET_SUPPORTED";
    case RootSupportType::SeatSupported: return "SEAT_SUPPORTED";
    case RootSupportType::BodyRestSupported: return "BODY_REST_SUPPORTED";
    case RootSupportType::KneeSupported: return "KNEE_SUPPORTED";
    case RootSupportType::MixedSupported: return "MIXED_SUPPORTED";
    default: return "UNKNOWN";
    }
}

inline const char* ToString(FootSupportType type) {
    switch (type) {
    case FootSupportType::None: return "NONE";
    case FootSupportType::FloorSupport: return "FLOOR_SUPPORT";
    case FootSupportType::RestSupport: return "REST_SUPPORT";
    default: return "UNKNOWN";
    }
}

inline const char* ToString(FootSupportPhase phase) {
    switch (phase) {
    case FootSupportPhase::Swing: return "SWING";
    case FootSupportPhase::ContactCandidate: return "CONTACT_CANDIDATE";
    case FootSupportPhase::HeelLock: return "HEEL_LOCK";
    case FootSupportPhase::FlatPlant: return "FLAT_PLANT";
    case FootSupportPhase::ToePivot: return "TOE_PIVOT";
    case FootSupportPhase::ReleasePending: return "RELEASE_PENDING";
    case FootSupportPhase::Slip: return "SLIP";
    case FootSupportPhase::RestCandidate: return "REST_CANDIDATE";
    case FootSupportPhase::RestLock: return "REST_LOCK";
    default: return "UNKNOWN";
    }
}

inline const char* ToString(FootContactLoad load) {
    switch (load) {
    case FootContactLoad::None: return "NONE";
    case FootContactLoad::HeelOnly: return "HEEL_ONLY";
    case FootContactLoad::ToeOnly: return "TOE_ONLY";
    case FootContactLoad::FullPlant: return "FULL_PLANT";
    case FootContactLoad::Inferred: return "INFERRED";
    default: return "UNKNOWN";
    }
}

} // namespace bt
