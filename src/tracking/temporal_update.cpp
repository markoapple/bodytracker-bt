#include "tracking/temporal_update.h"

#include "tracking/contact_constraints.h"
#include "tracking/support_queries.h"
#include "tracking/tracking_constants.h"

#include <algorithm>

namespace bt {
namespace {

float Clamp01(float value) {
    return std::clamp(value, 0.0f, 1.0f);
}

float FootMeasurementTrust(
    float body_confidence,
    const TrackerEvidence& evidence,
    const FootSupportState& support) {

    float trust = Clamp01(body_confidence);
    switch (evidence.source) {
    case TrackerEvidenceSource::DirectStereo:
    case TrackerEvidenceSource::ReplayInput:
    case TrackerEvidenceSource::InferredMonocular:
        trust = std::max(trust, Clamp01(evidence.direct_confidence));
        break;
    case TrackerEvidenceSource::AnchorHeld:
        trust = std::max(trust, Clamp01(evidence.support_confidence));
        break;
    case TrackerEvidenceSource::HmdPrediction:
    case TrackerEvidenceSource::Predicted:
    case TrackerEvidenceSource::None:
    default:
        break;
    }
    trust = std::max(trust, Clamp01(FootSupportConfidence(support)));
    return trust;
}

float FootCorrectionGain(
    const FootSupportState& support,
    const TrackerEvidence& evidence,
    float body_confidence,
    const TemporalUpdateConfig& config) {

    const float base = IsActiveFootSupport(support)
        ? config.foot_supported_gain
        : config.foot_free_gain;
    return base * FootMeasurementTrust(body_confidence, evidence, support);
}

Pose3f BlendPose(const Pose3f& a, const Pose3f& b, float gain) {
    Pose3f out;
    out.position = Lerp(a.position, b.position, gain);
    out.orientation = Slerp(a.orientation, b.orientation, gain);
    return out;
}

float FootLengthFor(const LowerBodyModel* model, bool left) {
    if (!model) {
        return tracking_constants::kDefaultFootLengthM;
    }
    return left ? model->left_foot_length : model->right_foot_length;
}

Pose3f PredictFootPose(
    const Pose3f& previous_foot,
    const Vec3f& previous_root_position,
    const Pose3f& predicted_root,
    const Quatf& root_rotation_delta,
    const FootSupportState& support,
    const Vec3f& free_motion_velocity,
    double dt_seconds,
    float foot_length_m) {

    if (IsActiveFootSupport(support)) {
        return ApplyFootContactConstraint(previous_foot, support, foot_length_m);
    }

    Pose3f out = previous_foot;
    const Vec3f previous_offset = Sub(previous_foot.position, previous_root_position);
    out.position = Add(
        Add(predicted_root.position, Rotate(root_rotation_delta, previous_offset)),
        Scale(free_motion_velocity, static_cast<float>(dt_seconds)));
    out.orientation = Multiply(root_rotation_delta, previous_foot.orientation);
    return out;
}

Vec3f UpdateFreeFootVelocity(
    const Pose3f& predicted_foot,
    const Pose3f& corrected_foot,
    const Vec3f& previous_free_motion_velocity,
    double dt_seconds) {

    if (dt_seconds <= 1e-6) {
        return previous_free_motion_velocity;
    }
    const Vec3f predicted_without_free_motion = Sub(
        predicted_foot.position,
        Scale(previous_free_motion_velocity, static_cast<float>(dt_seconds)));
    return Scale(Sub(corrected_foot.position, predicted_without_free_motion), 1.0f / static_cast<float>(dt_seconds));
}

Pose3f CorrectFootPose(
    const Pose3f& predicted,
    const Pose3f& measured,
    const FootSupportState& support,
    float gain,
    float foot_length_m) {

    const float g = std::clamp(gain, 0.0f, 1.0f);
    if (IsActiveFootSupport(support)) {
        const Pose3f constrained_measured = ApplyFootContactConstraint(measured, support, foot_length_m);
        Pose3f blended = BlendPose(predicted, constrained_measured, g);
        // Re-apply the contact manifold so the gain changes how aggressively the
        // free body follows measurements without letting a planted contact slide.
        return ApplyFootContactConstraint(blended, support, foot_length_m);
    }

    Pose3f out = BlendPose(predicted, measured, g);
    out.orientation = Normalize(out.orientation);
    return out;
}

} // namespace

LowerBodyState PredictState(
    const LowerBodyState& previous,
    double dt_seconds,
    const LowerBodyModel* model) {
    LowerBodyState out = previous;
    const Vec3f previous_root_position = previous.root.position;
    const Quatf root_rotation_delta = QuatFromAngularVelocity(previous.angular_velocity, static_cast<float>(dt_seconds));

    out.root.position.x += previous.linear_velocity.x * static_cast<float>(dt_seconds);
    out.root.position.y += previous.linear_velocity.y * static_cast<float>(dt_seconds);
    out.root.position.z += previous.linear_velocity.z * static_cast<float>(dt_seconds);
    out.root.orientation = Multiply(root_rotation_delta, previous.root.orientation);
    out.left_foot = PredictFootPose(
        previous.left_foot,
        previous_root_position,
        out.root,
        root_rotation_delta,
        previous.support.left_foot,
        previous.left_foot_linear_velocity,
        dt_seconds,
        FootLengthFor(model, true));
    out.right_foot = PredictFootPose(
        previous.right_foot,
        previous_root_position,
        out.root,
        root_rotation_delta,
        previous.support.right_foot,
        previous.right_foot_linear_velocity,
        dt_seconds,
        FootLengthFor(model, false));
    if (IsActiveFootSupport(previous.support.left_foot)) {
        out.left_foot_linear_velocity = {};
    }
    if (IsActiveFootSupport(previous.support.right_foot)) {
        out.right_foot_linear_velocity = {};
    }
    return out;
}

LowerBodyState CorrectState(
    const LowerBodyState& predicted,
    const LowerBodyState& measured,
    double dt_seconds,
    const TemporalUpdateConfig& config,
    TemporalPositionCorrectionMode position_mode,
    const LowerBodyModel* model) {

    LowerBodyState out = predicted;
    const bool supported = measured.support.root_support != RootSupportType::None;
    // Confidence controls correction strength, not whether a pose exists. A finite
    // but zero/weak-confidence monocular solve may be useful to expose, but it
    // must not yank the temporal state at full strength and create SteamVR jitter.
    const float gain = (supported ? config.supported_gain : config.free_gain) *
        Clamp01(measured.confidence);
    const float dt = static_cast<float>(dt_seconds);
    const Vec3f previous_root = Sub(predicted.root.position, Scale(predicted.linear_velocity, dt));
    const Quatf root_rotation_delta = QuatFromAngularVelocity(predicted.angular_velocity, dt);
    const Quatf previous_orientation = Multiply(Conjugate(root_rotation_delta), predicted.root.orientation);

    if (position_mode == TemporalPositionCorrectionMode::DirectMeasuredPositions) {
        out.root.position = measured.root.position;
    } else {
        out.root.position.x += gain * (measured.root.position.x - predicted.root.position.x);
        out.root.position.y += gain * (measured.root.position.y - predicted.root.position.y);
        out.root.position.z += gain * (measured.root.position.z - predicted.root.position.z);
    }
    out.root.orientation = Normalize(measured.root.orientation);

    if (dt_seconds > 1e-6) {
        const float inv_dt = 1.0f / dt;
        out.linear_velocity = Scale(Sub(out.root.position, previous_root), inv_dt);
        out.angular_velocity = AngularVelocityBetween(previous_orientation, out.root.orientation, dt);
    }

    if (position_mode == TemporalPositionCorrectionMode::DirectMeasuredPositions) {
        out.left_foot = measured.left_foot;
        out.right_foot = measured.right_foot;
    } else {
        out.left_foot = CorrectFootPose(
            predicted.left_foot,
            measured.left_foot,
            measured.support.left_foot,
            FootCorrectionGain(
                measured.support.left_foot,
                measured.left_foot_evidence,
                measured.confidence,
                config),
            FootLengthFor(model, true));
        out.right_foot = CorrectFootPose(
            predicted.right_foot,
            measured.right_foot,
            measured.support.right_foot,
            FootCorrectionGain(
                measured.support.right_foot,
                measured.right_foot_evidence,
                measured.confidence,
                config),
            FootLengthFor(model, false));
    }

    if (IsActiveFootSupport(measured.support.left_foot)) {
        out.left_foot = ApplyFootContactConstraint(out.left_foot, measured.support.left_foot, FootLengthFor(model, true));
        out.left_foot_linear_velocity = {};
    } else {
        out.left_foot_linear_velocity = UpdateFreeFootVelocity(
            predicted.left_foot,
            out.left_foot,
            predicted.left_foot_linear_velocity,
            dt_seconds);
    }
    if (IsActiveFootSupport(measured.support.right_foot)) {
        out.right_foot = ApplyFootContactConstraint(out.right_foot, measured.support.right_foot, FootLengthFor(model, false));
        out.right_foot_linear_velocity = {};
    } else {
        out.right_foot_linear_velocity = UpdateFreeFootVelocity(
            predicted.right_foot,
            out.right_foot,
            predicted.right_foot_linear_velocity,
            dt_seconds);
    }
    out.left_hip_flexion = Lerp(predicted.left_hip_flexion, measured.left_hip_flexion, gain);
    out.left_hip_abduction = Lerp(predicted.left_hip_abduction, measured.left_hip_abduction, gain);
    out.left_knee_flexion = std::max(0.0f, Lerp(predicted.left_knee_flexion, measured.left_knee_flexion, gain));
    out.left_ankle_pitch = Lerp(predicted.left_ankle_pitch, measured.left_ankle_pitch, gain);
    out.left_ankle_roll = Lerp(predicted.left_ankle_roll, measured.left_ankle_roll, gain);
    out.left_ankle_yaw = Lerp(predicted.left_ankle_yaw, measured.left_ankle_yaw, gain);
    out.right_hip_flexion = Lerp(predicted.right_hip_flexion, measured.right_hip_flexion, gain);
    out.right_hip_abduction = Lerp(predicted.right_hip_abduction, measured.right_hip_abduction, gain);
    out.right_knee_flexion = std::max(0.0f, Lerp(predicted.right_knee_flexion, measured.right_knee_flexion, gain));
    out.right_ankle_pitch = Lerp(predicted.right_ankle_pitch, measured.right_ankle_pitch, gain);
    out.right_ankle_roll = Lerp(predicted.right_ankle_roll, measured.right_ankle_roll, gain);
    out.right_ankle_yaw = Lerp(predicted.right_ankle_yaw, measured.right_ankle_yaw, gain);
    out.posture_mode = measured.posture_mode;
    out.support = measured.support;
    out.left_foot_evidence = measured.left_foot_evidence;
    out.right_foot_evidence = measured.right_foot_evidence;
    out.confidence = measured.confidence;
    return out;
}

} // namespace bt
