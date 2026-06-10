#include "tracking/root_support.h"

#include <algorithm>
#include <cmath>

#include "tracking/support_queries.h"

namespace bt {
namespace {

float Clamp01(float value) {
    if (!std::isfinite(value)) {
        return 0.0f;
    }
    return std::max(0.0f, std::min(1.0f, value));
}

bool AnyActiveFootSupport(const SupportManifoldState& current) {
    return IsActiveFootSupport(current.left_foot) || IsActiveFootSupport(current.right_foot);
}

bool AnyActiveKneeSupport(const SupportManifoldState& current) {
    return current.left_knee_anchor.active || current.right_knee_anchor.active;
}

bool KneeContactAllowed(PostureMode posture_mode) {
    return posture_mode == PostureMode::Kneeling ||
        posture_mode == PostureMode::Crouching ||
        posture_mode == PostureMode::UprightTransition ||
        posture_mode == PostureMode::UnknownFree;
}

RootSupportType DesiredRootSupport(PostureMode posture_mode, const SupportManifoldState& current) {
    const bool feet_supported = AnyActiveFootSupport(current);
    const bool knees_supported = AnyActiveKneeSupport(current);
    switch (posture_mode) {
    case PostureMode::SeatedSupported:
        return RootSupportType::SeatSupported;
    case PostureMode::ReclinedSupported:
        return RootSupportType::BodyRestSupported;
    case PostureMode::Kneeling:
    case PostureMode::Crouching:
        if (feet_supported && knees_supported) {
            return RootSupportType::MixedSupported;
        }
        if (knees_supported) {
            return RootSupportType::KneeSupported;
        }
        return feet_supported ? RootSupportType::FeetSupported : RootSupportType::None;
    case PostureMode::UprightStanding:
        return feet_supported ? RootSupportType::FeetSupported : RootSupportType::None;
    case PostureMode::UprightTransition:
        if (feet_supported && knees_supported) {
            return RootSupportType::MixedSupported;
        }
        if (feet_supported) {
            return RootSupportType::FeetSupported;
        }
        return current.root_support;
    default:
        return knees_supported ? RootSupportType::KneeSupported : RootSupportType::None;
    }
}

SupportAnchor UpdateSingleKneeAnchor(
    const SupportAnchor& previous,
    const Vec3f& knee_position,
    const FloorPlane& floor,
    bool contact_allowed,
    bool evidence_usable,
    float evidence_confidence,
    double dt_seconds,
    const KneeContactConfig& config) {

    SupportAnchor out = previous;
    const double dt = std::isfinite(dt_seconds) && dt_seconds > 0.0 ? dt_seconds : 0.0;
    const float confidence = Clamp01(evidence_confidence);
    const bool reliable = contact_allowed && evidence_usable && confidence >= config.min_contact_confidence;

    if (reliable) {
        Pose3f anchor_pose;
        anchor_pose.position = FloorPlaneUsable(floor) ? ProjectPointToFloorPlane(knee_position, floor) : knee_position;
        anchor_pose.orientation = {};
        if (!out.active) {
            out.pose = anchor_pose;
            out.active = true;
            out.dwell_seconds = 0.0;
            out.release_seconds = 0.0;
            out.confidence = std::min(0.30f, confidence);
            return out;
        }
        out.dwell_seconds += dt;
        out.release_seconds = 0.0;
        out.confidence = std::min(confidence, out.confidence + static_cast<float>(dt * 2.5));
        const float anchor_gain = out.dwell_seconds > config.lock_dwell_seconds ? 0.08f : 0.35f;
        out.pose.position = Lerp(out.pose.position, anchor_pose.position, anchor_gain);
        return out;
    }

    if (!out.active) {
        out.confidence = 0.0f;
        out.dwell_seconds = 0.0;
        out.release_seconds = 0.0;
        return out;
    }

    out.release_seconds += dt;
    if (!evidence_usable) {
        out.confidence = std::max(
            0.0f,
            out.confidence - static_cast<float>(dt * config.missing_evidence_confidence_decay_per_second));
        if (out.release_seconds < config.missing_evidence_release_seconds && out.confidence > 0.02f) {
            return out;
        }
    } else if (out.release_seconds < config.release_seconds) {
        out.confidence = std::max(0.0f, out.confidence - static_cast<float>(dt / std::max(1e-6, config.release_seconds)));
        return out;
    }

    out.active = false;
    out.confidence = 0.0f;
    out.dwell_seconds = 0.0;
    out.release_seconds = 0.0;
    return out;
}

} // namespace

SupportManifoldState UpdateKneeContactSupport(
    const SupportManifoldState& previous,
    const LowerBodyState& solved_state,
    const LowerBodyModel& model,
    const FloorPlane& floor,
    PostureMode posture_mode,
    double dt_seconds,
    const KneeContactEvidence& evidence,
    const KneeContactConfig& config) {

    SupportManifoldState out = previous;
    const LowerBodyJointSet joints = PredictLowerBodyJoints(solved_state, model);
    const auto& left_knee = joints.joints[static_cast<std::size_t>(KeypointId::LeftKnee)];
    const auto& right_knee = joints.joints[static_cast<std::size_t>(KeypointId::RightKnee)];
    const bool allowed = KneeContactAllowed(posture_mode);

    out.left_knee_anchor = UpdateSingleKneeAnchor(
        previous.left_knee_anchor,
        left_knee.world,
        floor,
        allowed,
        evidence.left_usable && left_knee.present,
        evidence.left_confidence,
        dt_seconds,
        config);
    out.right_knee_anchor = UpdateSingleKneeAnchor(
        previous.right_knee_anchor,
        right_knee.world,
        floor,
        allowed,
        evidence.right_usable && right_knee.present,
        evidence.right_confidence,
        dt_seconds,
        config);
    return out;
}

SupportManifoldState UpdateRootSupport(
    const SupportManifoldState& previous,
    const LowerBodyState& solved_state,
    PostureMode posture_mode,
    double dt_seconds,
    const RootSupportConfig& config) {

    SupportManifoldState out = previous;
    const double dt = std::isfinite(dt_seconds) && dt_seconds > 0.0 ? dt_seconds : 0.0;
    const RootSupportType desired = DesiredRootSupport(posture_mode, previous);

    if (desired == out.root_support) {
        if (desired == RootSupportType::None) {
            out.root_anchor.active = false;
            out.root_anchor.confidence = 0.0f;
            out.root_anchor.dwell_seconds = 0.0;
            out.root_anchor.release_seconds = 0.0;
            return out;
        }

        out.root_anchor.dwell_seconds += dt;
        out.root_anchor.release_seconds = 0.0;
        out.root_anchor.confidence = std::min(1.0f, out.root_anchor.confidence + static_cast<float>(dt * 2.0));
        // Once locked, the root anchor is a support constraint, not a jitter follower.
        // It is replaced only by a support-state transition below.
    } else {
        out.root_anchor.release_seconds += dt;
        const double required_release =
            posture_mode == PostureMode::UprightTransition ? config.transition_hold_seconds : config.release_seconds;
        if (out.root_anchor.release_seconds >= required_release) {
            out.root_support = desired;
            out.root_anchor.pose = solved_state.root;
            out.root_anchor.active = desired != RootSupportType::None;
            out.root_anchor.dwell_seconds = 0.0;
            out.root_anchor.release_seconds = 0.0;
            out.root_anchor.confidence = desired == RootSupportType::None ? 0.0f : 0.35f;
        }
    }
    return out;
}

} // namespace bt
