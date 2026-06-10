#include "tracking/tracker_synthesis.h"

#include "tracking/body_state.h"
#include "tracking/body_model.h"
#include "tracking/support_queries.h"

#include <algorithm>
#include <cmath>

namespace bt {
namespace {

float Clamp01(float v) {
    if (!std::isfinite(v)) {
        return 0.0f;
    }
    return std::max(0.0f, std::min(1.0f, v));
}

float CapForSource(TrackerEvidenceSource source) {
    switch (source) {
    case TrackerEvidenceSource::DirectStereo: return 1.0f;
    case TrackerEvidenceSource::ReplayInput: return 1.0f;
    case TrackerEvidenceSource::InferredMonocular: return 0.68f;
    case TrackerEvidenceSource::AnchorHeld: return 1.0f;
    case TrackerEvidenceSource::HmdPrediction: return 0.35f;
    case TrackerEvidenceSource::Predicted: return 0.24f;
    case TrackerEvidenceSource::None:
    default: return 0.0f;
    }
}

TrackerEvidence ResolvedFootEvidence(TrackerEvidence evidence, const FootSupportState& support) {
    evidence.support_confidence = std::max(evidence.support_confidence, FootSupportConfidence(support));
    evidence.anchor_held = evidence.anchor_held || IsActiveFootSupport(support);
    if ((!evidence.valid || evidence.direct_confidence < 0.15f) && evidence.anchor_held) {
        evidence.source = TrackerEvidenceSource::AnchorHeld;
        evidence.valid = evidence.support_confidence > 0.0f;
    }
    return evidence;
}

float FootConfidence(float body_confidence, TrackerEvidence evidence) {
    const float body = Clamp01(body_confidence);
    evidence.direct_confidence = Clamp01(evidence.direct_confidence);
    evidence.support_confidence = Clamp01(evidence.support_confidence);

    if (evidence.source == TrackerEvidenceSource::None && !evidence.anchor_held) {
        return body;
    }

    float evidence_confidence = 0.0f;
    switch (evidence.source) {
    case TrackerEvidenceSource::DirectStereo:
    case TrackerEvidenceSource::ReplayInput:
    case TrackerEvidenceSource::InferredMonocular:
        evidence_confidence = evidence.direct_confidence;
        break;
    case TrackerEvidenceSource::AnchorHeld:
        if (evidence.support_confidence <= 0.0f) {
            return 0.0f;
        }
        return Clamp01(std::min(evidence.support_confidence, CapForSource(evidence.source)));
    case TrackerEvidenceSource::HmdPrediction:
        evidence_confidence = body;
        break;
    case TrackerEvidenceSource::Predicted:
        evidence_confidence = 0.5f * body;
        break;
    case TrackerEvidenceSource::None:
    default:
        evidence_confidence = 0.0f;
        break;
    }

    return Clamp01(std::min({body, evidence_confidence, CapForSource(evidence.source)}));
}

TrackerEvidence PredictedJointEvidence(float body_confidence) {
    TrackerEvidence evidence;
    evidence.source = TrackerEvidenceSource::Predicted;
    evidence.direct_confidence = Clamp01(body_confidence);
    evidence.valid = body_confidence > 0.0f;
    return evidence;
}

float PredictedJointConfidence(float body_confidence) {
    return Clamp01(std::min({Clamp01(body_confidence), 0.5f * Clamp01(body_confidence), CapForSource(TrackerEvidenceSource::Predicted)}));
}

Pose3f PoseFromJointOr(const LowerBodyJointSet& joints, KeypointId id, const Pose3f& fallback) {
    const auto& joint = joints.joints[static_cast<std::size_t>(id)];
    if (!joint.present || !IsFinite(joint.world)) {
        return fallback;
    }
    Pose3f pose = fallback;
    pose.position = joint.world;
    return pose;
}

const BodyStateJoint& BodyJoint(const UnifiedBodyState& body_state, BodyJointRole role) {
    return body_state.roles[BodyJointRoleIndex(role)];
}

bool BodyJointUsable(const UnifiedBodyState& body_state, BodyJointRole role) {
    const auto& joint = BodyJoint(body_state, role);
    return joint.valid && IsFinite(joint.position);
}

Vec3f BodyJointPositionOr(const UnifiedBodyState& body_state, BodyJointRole role, Vec3f fallback) {
    return BodyJointUsable(body_state, role) ? BodyJoint(body_state, role).position : fallback;
}

Quatf ChestOrientationOrRoot(const UnifiedBodyState& body_state) {
    const Quatf root_orientation = body_state.lower_body.root.orientation;
    const Vec3f fallback_right = Rotate(root_orientation, Vec3f{1.0f, 0.0f, 0.0f});
    const Vec3f fallback_up = Rotate(root_orientation, Vec3f{0.0f, 1.0f, 0.0f});
    const Vec3f fallback_forward = Rotate(root_orientation, Vec3f{0.0f, 0.0f, 1.0f});

    if (!BodyJointUsable(body_state, BodyJointRole::LeftShoulder) ||
        !BodyJointUsable(body_state, BodyJointRole::RightShoulder)) {
        return root_orientation;
    }

    const Vec3f left_shoulder = BodyJoint(body_state, BodyJointRole::LeftShoulder).position;
    const Vec3f right_shoulder = BodyJoint(body_state, BodyJointRole::RightShoulder).position;
    const Vec3f chest = BodyJointPositionOr(body_state, BodyJointRole::Chest, body_state.lower_body.root.position);
    const Vec3f neck = BodyJointPositionOr(body_state, BodyJointRole::Neck, Add(chest, Scale(fallback_up, 0.20f)));

    Vec3f right = NormalizeOr(Sub(right_shoulder, left_shoulder), fallback_right);
    Vec3f up_candidate = NormalizeOr(Sub(neck, chest), fallback_up);
    Vec3f forward = NormalizeOr(Cross(right, up_candidate), fallback_forward);
    Vec3f up = NormalizeOr(Cross(forward, right), fallback_up);
    right = NormalizeOr(Cross(up, forward), right);
    return QuatFromBasis(right, up, forward);
}

Quatf LimbOrientationOrRoot(
    const UnifiedBodyState& body_state,
    BodyJointRole proximal_role,
    BodyJointRole joint_role,
    BodyJointRole distal_role) {

    const Quatf root_orientation = body_state.lower_body.root.orientation;
    const Vec3f fallback_right = Rotate(root_orientation, Vec3f{1.0f, 0.0f, 0.0f});
    const Vec3f fallback_up = Rotate(root_orientation, Vec3f{0.0f, 1.0f, 0.0f});
    const Vec3f fallback_forward = Rotate(root_orientation, Vec3f{0.0f, 0.0f, 1.0f});

    if (!BodyJointUsable(body_state, proximal_role) || !BodyJointUsable(body_state, joint_role)) {
        return root_orientation;
    }

    const Vec3f proximal = BodyJoint(body_state, proximal_role).position;
    const Vec3f joint = BodyJoint(body_state, joint_role).position;
    const Vec3f distal = BodyJointPositionOr(body_state, distal_role, joint);
    Vec3f segment = BodyJointUsable(body_state, distal_role)
        ? Sub(distal, proximal)
        : Sub(joint, proximal);
    Vec3f forward = NormalizeOr(segment, fallback_forward);
    Vec3f right = NormalizeOr(Cross(fallback_up, forward), fallback_right);
    Vec3f up = NormalizeOr(Cross(forward, right), fallback_up);
    right = NormalizeOr(Cross(up, forward), right);
    return QuatFromBasis(right, up, forward);
}

Pose3f UpperBodyPoseOrRoot(
    const UnifiedBodyState& body_state,
    BodyJointRole role,
    Quatf orientation) {

    Pose3f pose = body_state.lower_body.root;
    pose.position = BodyJointPositionOr(body_state, role, body_state.lower_body.root.position);
    pose.orientation = orientation;
    return pose;
}

void ApplyLatencyPrediction(TrackerPose& tracker, const BodyStateJoint& joint, double seconds) {
    if (seconds <= 0.0 || !tracker.valid || !joint.valid || !IsFinite(joint.velocity)) {
        return;
    }
    const float strength = Clamp01((tracker.confidence - 0.15f) / 0.55f);
    if (strength <= 0.0f) {
        return;
    }
    tracker.pose.position = Add(
        tracker.pose.position,
        Scale(joint.velocity, static_cast<float>(seconds) * strength));
}

TrackerEvidence EvidenceForBodyJoint(const BodyStateJoint& joint) {
    TrackerEvidence evidence = joint.evidence;
    if (evidence.valid) {
        return evidence;
    }
    if (joint.triangulated) {
        evidence.source = TrackerEvidenceSource::DirectStereo;
        evidence.direct_confidence = joint.confidence;
        evidence.valid = joint.valid && IsFinite(joint.position);
    } else if (joint.depth_inferred) {
        evidence.source = TrackerEvidenceSource::InferredMonocular;
        evidence.direct_confidence = joint.confidence;
        evidence.valid = joint.valid && IsFinite(joint.position);
    } else if (joint.predicted) {
        evidence.source = TrackerEvidenceSource::Predicted;
        evidence.direct_confidence = joint.confidence;
        evidence.valid = joint.valid && IsFinite(joint.position);
    }
    return evidence;
}

TrackerPose TrackerFromBodyJoint(TrackerRole role, const BodyStateJoint& joint, Pose3f fallback_pose) {
    TrackerEvidence evidence = EvidenceForBodyJoint(joint);
    if (joint.contact_lock_strength > 0.0f) {
        evidence.support_confidence = std::max(evidence.support_confidence, joint.contact_lock_strength);
        evidence.anchor_held = evidence.anchor_held || joint.contact_lock_strength > 0.0f;
        if (evidence.source == TrackerEvidenceSource::None || evidence.source == TrackerEvidenceSource::Predicted) {
            evidence.source = TrackerEvidenceSource::AnchorHeld;
            evidence.valid = true;
        }
    }

    Pose3f pose = fallback_pose;
    if (joint.valid && IsFinite(joint.position)) {
        pose.position = joint.position;
    }
    float source_confidence = joint.confidence;
    switch (evidence.source) {
    case TrackerEvidenceSource::DirectStereo:
    case TrackerEvidenceSource::InferredMonocular:
    case TrackerEvidenceSource::ReplayInput:
        source_confidence = std::min(joint.confidence, Clamp01(evidence.direct_confidence));
        break;
    case TrackerEvidenceSource::AnchorHeld:
        source_confidence = evidence.support_confidence;
        break;
    case TrackerEvidenceSource::HmdPrediction:
    case TrackerEvidenceSource::Predicted:
    case TrackerEvidenceSource::None:
    default:
        break;
    }
    const float confidence = Clamp01(std::min(source_confidence, CapForSource(evidence.source)));
    const bool valid = IsFinite(pose.position) &&
        evidence.source != TrackerEvidenceSource::None &&
        (joint.valid || evidence.anchor_held || evidence.valid);
    return TrackerPose{role, pose, confidence, valid, evidence};
}

} // namespace

TrackerPoseArray SynthesizeTrackerPoses(const LowerBodyState& state) {
    return SynthesizeTrackerPoses(state, LowerBodyModel{});
}

TrackerPoseArray SynthesizeTrackerPoses(const LowerBodyState& state, const LowerBodyModel& model) {
    const float pelvis_confidence = Clamp01(state.confidence);
    const TrackerEvidence left_evidence = ResolvedFootEvidence(state.left_foot_evidence, state.support.left_foot);
    const TrackerEvidence right_evidence = ResolvedFootEvidence(state.right_foot_evidence, state.support.right_foot);
    const float left_foot_confidence = FootConfidence(state.confidence, left_evidence);
    const float right_foot_confidence = FootConfidence(state.confidence, right_evidence);
    const LowerBodyJointSet joints = PredictLowerBodyJoints(state, model);

    TrackerEvidence pelvis_evidence;
    pelvis_evidence.source = IsFinite(state.root.position) ? TrackerEvidenceSource::DirectStereo : TrackerEvidenceSource::None;
    pelvis_evidence.direct_confidence = pelvis_confidence;
    pelvis_evidence.valid = IsFinite(state.root.position);

    const TrackerEvidence knee_evidence = PredictedJointEvidence(state.confidence);
    const float knee_confidence = PredictedJointConfidence(state.confidence);
    const Pose3f left_knee = PoseFromJointOr(joints, KeypointId::LeftKnee, state.root);
    const Pose3f right_knee = PoseFromJointOr(joints, KeypointId::RightKnee, state.root);

    return TrackerPoseArray{
        TrackerPose{TrackerRole::Pelvis, state.root, pelvis_confidence, IsFinite(state.root.position) && pelvis_evidence.valid, pelvis_evidence},
        TrackerPose{TrackerRole::LeftFoot, state.left_foot, left_foot_confidence, IsFinite(state.left_foot.position) && left_evidence.valid, left_evidence},
        TrackerPose{TrackerRole::RightFoot, state.right_foot, right_foot_confidence, IsFinite(state.right_foot.position) && right_evidence.valid, right_evidence},
        TrackerPose{TrackerRole::Chest},
        TrackerPose{TrackerRole::LeftElbow},
        TrackerPose{TrackerRole::RightElbow},
        TrackerPose{TrackerRole::LeftKnee, left_knee, knee_confidence, IsFinite(left_knee.position) && knee_evidence.valid, knee_evidence},
        TrackerPose{TrackerRole::RightKnee, right_knee, knee_confidence, IsFinite(right_knee.position) && knee_evidence.valid, knee_evidence}
    };
}

TrackerPoseArray SynthesizeTrackerPoses(const UnifiedBodyState& body_state, const LowerBodyModel& model) {
    TrackerPoseArray trackers{
        TrackerPose{TrackerRole::Pelvis},
        TrackerPose{TrackerRole::LeftFoot},
        TrackerPose{TrackerRole::RightFoot},
        TrackerPose{TrackerRole::Chest},
        TrackerPose{TrackerRole::LeftElbow},
        TrackerPose{TrackerRole::RightElbow},
        TrackerPose{TrackerRole::LeftKnee},
        TrackerPose{TrackerRole::RightKnee}
    };
    const LowerBodyJointSet fallback_joints = body_state.joints.joints[static_cast<std::size_t>(KeypointId::LeftKnee)].present
        ? body_state.joints
        : PredictLowerBodyJoints(body_state.lower_body, model);
    trackers[TrackerRoleIndex(TrackerRole::Pelvis)] = TrackerFromBodyJoint(
        TrackerRole::Pelvis,
        BodyJoint(body_state, BodyJointRole::Pelvis),
        body_state.lower_body.root);
    trackers[TrackerRoleIndex(TrackerRole::LeftFoot)] = TrackerFromBodyJoint(
        TrackerRole::LeftFoot,
        BodyJoint(body_state, BodyJointRole::LeftFoot),
        body_state.lower_body.left_foot);
    trackers[TrackerRoleIndex(TrackerRole::RightFoot)] = TrackerFromBodyJoint(
        TrackerRole::RightFoot,
        BodyJoint(body_state, BodyJointRole::RightFoot),
        body_state.lower_body.right_foot);
    const Quatf chest_orientation = ChestOrientationOrRoot(body_state);
    trackers[TrackerRoleIndex(TrackerRole::Chest)] = TrackerFromBodyJoint(
        TrackerRole::Chest,
        BodyJoint(body_state, BodyJointRole::Chest),
        UpperBodyPoseOrRoot(body_state, BodyJointRole::Chest, chest_orientation));
    trackers[TrackerRoleIndex(TrackerRole::LeftElbow)] = TrackerFromBodyJoint(
        TrackerRole::LeftElbow,
        BodyJoint(body_state, BodyJointRole::LeftElbow),
        UpperBodyPoseOrRoot(
            body_state,
            BodyJointRole::LeftElbow,
            LimbOrientationOrRoot(body_state, BodyJointRole::LeftShoulder, BodyJointRole::LeftElbow, BodyJointRole::LeftWrist)));
    trackers[TrackerRoleIndex(TrackerRole::RightElbow)] = TrackerFromBodyJoint(
        TrackerRole::RightElbow,
        BodyJoint(body_state, BodyJointRole::RightElbow),
        UpperBodyPoseOrRoot(
            body_state,
            BodyJointRole::RightElbow,
            LimbOrientationOrRoot(body_state, BodyJointRole::RightShoulder, BodyJointRole::RightElbow, BodyJointRole::RightWrist)));
    trackers[TrackerRoleIndex(TrackerRole::LeftKnee)] = TrackerFromBodyJoint(
        TrackerRole::LeftKnee,
        BodyJoint(body_state, BodyJointRole::LeftKnee),
        PoseFromJointOr(fallback_joints, KeypointId::LeftKnee, body_state.lower_body.root));
    trackers[TrackerRoleIndex(TrackerRole::RightKnee)] = TrackerFromBodyJoint(
        TrackerRole::RightKnee,
        BodyJoint(body_state, BodyJointRole::RightKnee),
        PoseFromJointOr(fallback_joints, KeypointId::RightKnee, body_state.lower_body.root));

    const double prediction_seconds = body_state.diagnostics.latency_prediction_active
        ? body_state.diagnostics.latency_prediction_seconds
        : 0.0;
    ApplyLatencyPrediction(trackers[TrackerRoleIndex(TrackerRole::Pelvis)], BodyJoint(body_state, BodyJointRole::Pelvis), prediction_seconds);
    ApplyLatencyPrediction(trackers[TrackerRoleIndex(TrackerRole::LeftFoot)], BodyJoint(body_state, BodyJointRole::LeftFoot), prediction_seconds);
    ApplyLatencyPrediction(trackers[TrackerRoleIndex(TrackerRole::RightFoot)], BodyJoint(body_state, BodyJointRole::RightFoot), prediction_seconds);
    ApplyLatencyPrediction(trackers[TrackerRoleIndex(TrackerRole::Chest)], BodyJoint(body_state, BodyJointRole::Chest), prediction_seconds);
    ApplyLatencyPrediction(trackers[TrackerRoleIndex(TrackerRole::LeftElbow)], BodyJoint(body_state, BodyJointRole::LeftElbow), prediction_seconds);
    ApplyLatencyPrediction(trackers[TrackerRoleIndex(TrackerRole::RightElbow)], BodyJoint(body_state, BodyJointRole::RightElbow), prediction_seconds);
    ApplyLatencyPrediction(trackers[TrackerRoleIndex(TrackerRole::LeftKnee)], BodyJoint(body_state, BodyJointRole::LeftKnee), prediction_seconds);
    ApplyLatencyPrediction(trackers[TrackerRoleIndex(TrackerRole::RightKnee)], BodyJoint(body_state, BodyJointRole::RightKnee), prediction_seconds);
    return trackers;
}

void MarkTrackersStereoFallback(TrackerPoseArray& trackers) {
    for (auto& tracker : trackers) {
        tracker.evidence.stereo_fallback = true;
    }
}

} // namespace bt
