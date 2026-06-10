#include "tracking/contact_constraints.h"

#include "tracking/support_queries.h"
#include "tracking/tracking_constants.h"

#include <algorithm>
#include <cmath>

namespace bt {
namespace {

constexpr float kMinFootLengthM = 0.12f;
constexpr float kMaxFootLengthM = 0.38f;

float Clamp01(float value) {
    if (!std::isfinite(value)) {
        return 0.0f;
    }
    return std::max(0.0f, std::min(1.0f, value));
}

float ClampedFootLength(float foot_length_m) {
    if (!std::isfinite(foot_length_m)) {
        return tracking_constants::kDefaultFootLengthM;
    }
    return std::max(kMinFootLengthM, std::min(kMaxFootLengthM, foot_length_m));
}

float HeelOffset(float foot_length_m) {
    return -0.5f * ClampedFootLength(foot_length_m);
}

float ToeOffset(float foot_length_m) {
    return 0.5f * ClampedFootLength(foot_length_m);
}

Vec3f ContactPoint(const Pose3f& foot_pose, float forward_offset_m) {
    return Add(foot_pose.position, Rotate(foot_pose.orientation, Vec3f{0.0f, 0.0f, forward_offset_m}));
}

Pose3f TranslateByContactResidual(
    const Pose3f& measured,
    const Vec3f& measured_contact,
    const SupportAnchor& anchor,
    float gain) {

    Pose3f out = measured;
    const Vec3f residual = Sub(anchor.pose.position, measured_contact);
    out.position = Add(measured.position, Scale(residual, Clamp01(gain * anchor.confidence)));
    return out;
}

Pose3f BlendTowardAnchorPose(const Pose3f& measured, const SupportAnchor& anchor) {
    const float gain = Clamp01(anchor.confidence);
    Pose3f out;
    out.position = Lerp(measured.position, anchor.pose.position, gain);
    out.orientation = Slerp(measured.orientation, anchor.pose.orientation, gain);
    return out;
}

Pose3f ApplyFullPlantManifoldConstraint(const Pose3f& measured, const FootSupportState& support, float foot_length_m) {
    if (!support.heel_anchor.active || !support.toe_anchor.active) {
        return BlendTowardAnchorPose(measured, support.anchor);
    }

    const float gain = Clamp01(std::min(support.heel_anchor.confidence, support.toe_anchor.confidence));
    const float heel_offset = HeelOffset(foot_length_m);
    const float toe_offset = ToeOffset(foot_length_m);
    const float local_span = std::max(1e-5f, toe_offset - heel_offset);

    const Vec3f anchor_vec = Sub(support.toe_anchor.pose.position, support.heel_anchor.pose.position);
    const Vec3f measured_forward = NormalizeOr(
        Rotate(measured.orientation, Vec3f{0.0f, 0.0f, 1.0f}),
        Vec3f{0.0f, 0.0f, 1.0f});
    const Vec3f anchor_forward = NormalizeOr(anchor_vec, measured_forward);

    // Rigid two-contact solve. If anchor spacing matches the calibrated foot,
    // heel and toe are both satisfied exactly. If detector noise made the anchor
    // span non-rigid, fall back to the least-squares rigid fit instead of warping
    // the virtual foot.
    const Vec3f anchor_mid = Scale(Add(support.heel_anchor.pose.position, support.toe_anchor.pose.position), 0.5f);
    const float local_mid = 0.5f * (heel_offset + toe_offset);

    const Vec3f measured_up = NormalizeOr(
        Rotate(measured.orientation, Vec3f{0.0f, 1.0f, 0.0f}),
        Vec3f{0.0f, 1.0f, 0.0f});
    Vec3f right = NormalizeOr(Cross(measured_up, anchor_forward), Rotate(measured.orientation, Vec3f{1.0f, 0.0f, 0.0f}));
    Vec3f up = NormalizeOr(Cross(anchor_forward, right), measured_up);
    right = NormalizeOr(Cross(up, anchor_forward), right);

    Pose3f locked = measured;
    locked.orientation = QuatFromBasis(right, up, anchor_forward);
    locked.position = Sub(anchor_mid, Scale(anchor_forward, local_mid));

    const float anchor_span = Length(anchor_vec);
    const float span_agreement = Clamp01(1.0f - std::abs(anchor_span - local_span) / std::max(0.03f, local_span));
    const float effective_gain = gain * (0.35f + 0.65f * span_agreement);

    Pose3f out;
    out.position = Lerp(measured.position, locked.position, effective_gain);
    out.orientation = Slerp(measured.orientation, locked.orientation, effective_gain);
    return out;
}

} // namespace

Vec3f FootHeelContactPoint(const Pose3f& foot_pose) noexcept {
    return FootHeelContactPoint(foot_pose, tracking_constants::kDefaultFootLengthM);
}

Vec3f FootToeContactPoint(const Pose3f& foot_pose) noexcept {
    return FootToeContactPoint(foot_pose, tracking_constants::kDefaultFootLengthM);
}

Vec3f FootHeelContactPoint(const Pose3f& foot_pose, float calibrated_foot_length_m) noexcept {
    return ContactPoint(foot_pose, HeelOffset(calibrated_foot_length_m));
}

Vec3f FootToeContactPoint(const Pose3f& foot_pose, float calibrated_foot_length_m) noexcept {
    return ContactPoint(foot_pose, ToeOffset(calibrated_foot_length_m));
}

bool FootSupportIsFullPlant(const FootSupportState& support) noexcept {
    if (!IsActiveFootSupport(support) || support.type != FootSupportType::FloorSupport) {
        return false;
    }
    return support.phase == FootSupportPhase::FlatPlant;
}

bool FootSupportHasContactConstraint(const FootSupportState& support) noexcept {
    if (!IsActiveFootSupport(support) || support.type != FootSupportType::FloorSupport) {
        return false;
    }
    return support.phase == FootSupportPhase::ContactCandidate ||
        support.phase == FootSupportPhase::HeelLock ||
        support.phase == FootSupportPhase::FlatPlant ||
        support.phase == FootSupportPhase::ToePivot;
}

Pose3f ApplyFootContactConstraint(const Pose3f& measured, const FootSupportState& support) noexcept {
    return ApplyFootContactConstraint(measured, support, tracking_constants::kDefaultFootLengthM);
}

Pose3f ApplyFootContactConstraint(const Pose3f& measured, const FootSupportState& support, float calibrated_foot_length_m) noexcept {
    if (!FootSupportHasContactConstraint(support)) {
        return measured;
    }

    if (support.phase == FootSupportPhase::ToePivot && support.toe_anchor.active) {
        return TranslateByContactResidual(measured, FootToeContactPoint(measured, calibrated_foot_length_m), support.toe_anchor, 1.0f);
    }

    if ((support.phase == FootSupportPhase::ContactCandidate || support.phase == FootSupportPhase::HeelLock) &&
        support.heel_anchor.active) {
        return TranslateByContactResidual(measured, FootHeelContactPoint(measured, calibrated_foot_length_m), support.heel_anchor, 1.0f);
    }

    if (support.phase == FootSupportPhase::FlatPlant) {
        return ApplyFullPlantManifoldConstraint(measured, support, calibrated_foot_length_m);
    }

    return BlendTowardAnchorPose(measured, support.anchor);
}

FootContactResidual FootSupportResidual(const Pose3f& measured, const FootSupportState& support) noexcept {
    return FootSupportResidual(measured, support, tracking_constants::kDefaultFootLengthM);
}

FootContactResidual FootSupportResidual(const Pose3f& measured, const FootSupportState& support, float calibrated_foot_length_m) noexcept {
    FootContactResidual out;
    if (!FootSupportHasContactConstraint(support)) {
        return out;
    }

    if (support.phase == FootSupportPhase::ToePivot && support.toe_anchor.active) {
        out.residual = Sub(FootToeContactPoint(measured, calibrated_foot_length_m), support.toe_anchor.pose.position);
        out.magnitude_m = Length(out.residual);
        out.valid = true;
        return out;
    }

    if ((support.phase == FootSupportPhase::ContactCandidate || support.phase == FootSupportPhase::HeelLock) &&
        support.heel_anchor.active) {
        out.residual = Sub(FootHeelContactPoint(measured, calibrated_foot_length_m), support.heel_anchor.pose.position);
        out.magnitude_m = Length(out.residual);
        out.valid = true;
        return out;
    }

    if (support.phase == FootSupportPhase::FlatPlant &&
        support.heel_anchor.active &&
        support.toe_anchor.active) {
        const Vec3f heel_residual = Sub(FootHeelContactPoint(measured, calibrated_foot_length_m), support.heel_anchor.pose.position);
        const Vec3f toe_residual = Sub(FootToeContactPoint(measured, calibrated_foot_length_m), support.toe_anchor.pose.position);
        out.residual = Scale(Add(heel_residual, toe_residual), 0.5f);
        out.magnitude_m = std::max(Length(heel_residual), Length(toe_residual));
        out.valid = true;
        return out;
    }

    out.residual = Sub(measured.position, support.anchor.pose.position);
    out.magnitude_m = Length(out.residual);
    out.valid = true;
    return out;
}

} // namespace bt
