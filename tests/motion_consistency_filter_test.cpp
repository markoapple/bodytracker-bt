#include "tracking/motion_consistency_filter.h"
#include "tracking/contact_constraints.h"
#include "test_check.h"

#include <cmath>
#include <cstddef>

namespace {

std::size_t Index(bt::MotionTarget target) {
    return static_cast<std::size_t>(target);
}

bt::Pose3f Pose(float x, float y, float z) {
    bt::Pose3f pose;
    pose.position = bt::Vec3f{x, y, z};
    pose.orientation = bt::Quatf{0.0f, 0.0f, 0.0f, 1.0f};
    return pose;
}


float QuatLength(const bt::Quatf& q) {
    return std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
}

float QuatDot(const bt::Quatf& a, const bt::Quatf& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

bt::Quatf Yaw(float radians) {
    const float half = 0.5f * radians;
    return bt::Quatf{0.0f, std::sin(half), 0.0f, std::cos(half)};
}

bool SameOrientation(const bt::Quatf& a, const bt::Quatf& b, float epsilon) {
    return std::abs(QuatDot(bt::Normalize(a), bt::Normalize(b))) > 1.0f - epsilon;
}

bt::LowerBodyState StateAt(float root_x, float left_x, float right_x) {
    bt::LowerBodyState state;
    state.root = Pose(root_x, 1.0f, 0.0f);
    state.left_foot = Pose(left_x, 0.0f, -0.1f);
    state.right_foot = Pose(right_x, 0.0f, 0.1f);
    state.confidence = 0.9f;
    return state;
}

void MakeFlatPlant(bt::FootSupportState& support, const bt::Pose3f& foot) {
    support.type = bt::FootSupportType::FloorSupport;
    support.phase = bt::FootSupportPhase::FlatPlant;
    support.contact_load = bt::FootContactLoad::FullPlant;
    support.anchor.active = true;
    support.anchor.pose = foot;
    support.anchor.confidence = 1.0f;
    support.heel_anchor.active = true;
    support.heel_anchor.pose = foot;
    support.heel_anchor.pose.position = bt::FootHeelContactPoint(foot);
    support.heel_anchor.confidence = 1.0f;
    support.toe_anchor.active = true;
    support.toe_anchor.pose = foot;
    support.toe_anchor.pose.position = bt::FootToeContactPoint(foot);
    support.toe_anchor.confidence = 1.0f;
}

} // namespace

int main() {
    bt::MotionConsistencyConfig config;
    config.confirm_frames = 2;
    config.min_motion_m = 0.01f;
    config.stationary_deadzone_m = 0.004f;
    config.max_direction_deviation_deg = 35.0f;
    config.max_lateral_deviation_ratio = 0.50f;
    config.max_speed_change_ratio = 2.50f;
    config.planted_foot_release_confirm_frames = 2;
    config.planted_foot_max_drift_m = 0.04f;
    config.one_euro_enabled = false;

    const bt::Quatf identity = Yaw(0.0f);
    const bt::Quatf measured_yaw = Yaw(1.57079632679f);
    const bt::Quatf gain_zero = bt::Slerp(identity, measured_yaw, 0.0f);
    const bt::Quatf gain_one = bt::Slerp(identity, measured_yaw, 1.0f);
    const bt::Quatf gain_half = bt::Slerp(identity, measured_yaw, 0.5f);
    BT_CHECK(SameOrientation(gain_zero, identity, 1e-5f));
    BT_CHECK(SameOrientation(gain_one, measured_yaw, 1e-5f));
    BT_CHECK(!SameOrientation(gain_half, measured_yaw, 1e-5f));
    BT_CHECK_NEAR(QuatLength(gain_half), 1.0, 1e-5);

    const bt::Quatf antipodal_measured{-measured_yaw.x, -measured_yaw.y, -measured_yaw.z, -measured_yaw.w};
    const bt::Quatf antipodal_half = bt::Slerp(measured_yaw, antipodal_measured, 0.5f);
    BT_CHECK(SameOrientation(antipodal_half, measured_yaw, 1e-5f));
    BT_CHECK_NEAR(QuatLength(antipodal_half), 1.0, 1e-5);

    bt::MotionConsistencyFilterState filter;
    auto predicted = StateAt(0.0f, -0.2f, 0.2f);
    auto measured = predicted;
    auto out = bt::ApplyMotionConsistencyFilter(filter, measured, predicted, 1.0 / 60.0, config);
    BT_CHECK(filter.telemetry.targets[Index(bt::MotionTarget::Root)].decision == bt::MotionFilterDecision::Accepted);
    BT_CHECK_NEAR(out.root.position.x, 0.0, 1e-5);

    predicted = StateAt(0.02f, -0.18f, 0.22f);
    measured = predicted;
    out = bt::ApplyMotionConsistencyFilter(filter, measured, predicted, 1.0 / 60.0, config);
    BT_CHECK(filter.telemetry.targets[Index(bt::MotionTarget::Root)].decision == bt::MotionFilterDecision::Accepted);
    BT_CHECK(filter.telemetry.targets[Index(bt::MotionTarget::LeftFoot)].decision == bt::MotionFilterDecision::Accepted);
    BT_CHECK(filter.telemetry.targets[Index(bt::MotionTarget::LeftFoot)].reason == bt::MotionFilterReason::WithinLimits);
    BT_CHECK(filter.telemetry.targets[Index(bt::MotionTarget::LeftFoot)].expected_distance_m > config.min_motion_m);
    BT_CHECK_NEAR(out.root.position.x, 0.02, 1e-5);
    BT_CHECK_NEAR(out.left_foot.position.x, -0.18, 1e-5);

    predicted = StateAt(0.04f, -0.16f, 0.24f);
    measured = predicted;
    measured.root.position = bt::Vec3f{0.02f, 1.0f, 0.20f};
    out = bt::ApplyMotionConsistencyFilter(filter, measured, predicted, 1.0 / 60.0, config);
    BT_CHECK(filter.telemetry.targets[Index(bt::MotionTarget::Root)].decision == bt::MotionFilterDecision::Pending);
    BT_CHECK(filter.telemetry.targets[Index(bt::MotionTarget::Root)].reason == bt::MotionFilterReason::AbsoluteSpeed);
    BT_CHECK(filter.telemetry.targets[Index(bt::MotionTarget::Root)].direction_limit_deg == config.max_direction_deviation_deg);
    BT_CHECK(filter.telemetry.targets[Index(bt::MotionTarget::Root)].pending_frames == 1);
    BT_CHECK(filter.telemetry.targets[Index(bt::MotionTarget::Root)].confirm_frames == config.confirm_frames_max);
    BT_CHECK_NEAR(out.root.position.x, 0.02, 1e-5);
    BT_CHECK_NEAR(out.root.position.z, 0.0, 1e-5);

    predicted = StateAt(0.04f, -0.16f, 0.24f);
    measured = predicted;
    measured.root.position = bt::Vec3f{0.02f, 1.0f, 0.42f};
    out = bt::ApplyMotionConsistencyFilter(filter, measured, predicted, 1.0 / 60.0, config);
    BT_CHECK(filter.telemetry.targets[Index(bt::MotionTarget::Root)].decision == bt::MotionFilterDecision::Pending);
    BT_CHECK(filter.telemetry.targets[Index(bt::MotionTarget::Root)].pending_frames == 2);
    BT_CHECK_NEAR(out.root.position.z, 0.0, 1e-5);

    predicted = out;
    measured = out;
    measured.root.position.x += 0.001f;
    out = bt::ApplyMotionConsistencyFilter(filter, measured, predicted, 1.0 / 60.0, config);
    BT_CHECK(filter.telemetry.targets[Index(bt::MotionTarget::Root)].decision == bt::MotionFilterDecision::LowMotionHeld);

    bt::MotionConsistencyFilterState slow_filter;
    predicted = StateAt(0.0f, -0.2f, 0.2f);
    measured = predicted;
    out = bt::ApplyMotionConsistencyFilter(slow_filter, measured, predicted, 1.0 / 60.0, config);
    BT_CHECK(slow_filter.telemetry.targets[Index(bt::MotionTarget::Root)].decision == bt::MotionFilterDecision::Accepted);
    for (int i = 1; i <= 4; ++i) {
        predicted = out;
        measured = out;
        measured.root.position.x += 0.003f;
        out = bt::ApplyMotionConsistencyFilter(slow_filter, measured, predicted, 1.0 / 60.0, config);
    }
    BT_CHECK(slow_filter.telemetry.targets[Index(bt::MotionTarget::Root)].decision == bt::MotionFilterDecision::Blended);
    BT_CHECK(out.root.position.x > 0.0f);

    bt::MotionConsistencyFilterState alternating_jitter_filter;
    predicted = StateAt(0.0f, -0.2f, 0.2f);
    measured = predicted;
    out = bt::ApplyMotionConsistencyFilter(alternating_jitter_filter, measured, predicted, 1.0 / 60.0, config);
    for (int i = 0; i < 12; ++i) {
        predicted = out;
        measured = out;
        measured.root.position.x = (i % 2 == 0) ? 0.003f : -0.003f;
        out = bt::ApplyMotionConsistencyFilter(alternating_jitter_filter, measured, predicted, 1.0 / 60.0, config);
        BT_CHECK(alternating_jitter_filter.telemetry.targets[Index(bt::MotionTarget::Root)].decision != bt::MotionFilterDecision::Blended);
    }
    BT_CHECK_NEAR(out.root.position.x, 0.0, 1e-5);

    bt::MotionConsistencyFilterState teleport_filter;
    predicted = StateAt(0.0f, -0.2f, 0.2f);
    measured = predicted;
    out = bt::ApplyMotionConsistencyFilter(teleport_filter, measured, predicted, 1.0 / 60.0, config);
    for (int i = 0; i < 2; ++i) {
        predicted = StateAt(0.40f, -0.2f, 0.2f);
        measured = predicted;
        out = bt::ApplyMotionConsistencyFilter(teleport_filter, measured, predicted, 1.0 / 60.0, config);
        BT_CHECK(teleport_filter.telemetry.targets[Index(bt::MotionTarget::Root)].decision == bt::MotionFilterDecision::Pending);
    }
    BT_CHECK(teleport_filter.telemetry.targets[Index(bt::MotionTarget::Root)].pending_frames == 2);
    BT_CHECK_NEAR(out.root.position.x, 0.0, 1e-5);

    bt::MotionConsistencyFilterState orientation_filter;
    predicted = StateAt(0.0f, -0.2f, 0.2f);
    measured = predicted;
    measured.root.orientation = identity;
    out = bt::ApplyMotionConsistencyFilter(orientation_filter, measured, predicted, 1.0 / 60.0, config);
    BT_CHECK(SameOrientation(out.root.orientation, identity, 1e-5f));

    predicted = out;
    measured = out;
    measured.root.position.x += 0.012f;
    measured.root.orientation = measured_yaw;
    out = bt::ApplyMotionConsistencyFilter(orientation_filter, measured, predicted, 1.0 / 60.0, config);
    BT_CHECK(orientation_filter.telemetry.targets[Index(bt::MotionTarget::Root)].decision == bt::MotionFilterDecision::Blended);
    BT_CHECK(!SameOrientation(out.root.orientation, measured_yaw, 1e-5f));
    BT_CHECK(out.root.orientation.y > 0.0f);
    BT_CHECK(out.root.orientation.y < measured_yaw.y);
    BT_CHECK_NEAR(QuatLength(out.root.orientation), 1.0, 1e-5);


    bt::MotionConsistencyFilterState decel_filter;
    predicted = StateAt(0.0f, -0.2f, 0.2f);
    measured = predicted;
    out = bt::ApplyMotionConsistencyFilter(decel_filter, measured, predicted, 1.0 / 60.0, config);
    BT_CHECK(decel_filter.telemetry.targets[Index(bt::MotionTarget::Root)].decision == bt::MotionFilterDecision::Accepted);

    predicted = StateAt(0.03f, -0.2f, 0.2f);
    measured = predicted;
    out = bt::ApplyMotionConsistencyFilter(decel_filter, measured, predicted, 1.0 / 60.0, config);
    BT_CHECK(decel_filter.telemetry.targets[Index(bt::MotionTarget::Root)].decision == bt::MotionFilterDecision::Accepted);

    predicted = StateAt(0.06f, -0.2f, 0.2f);
    measured = predicted;
    out = bt::ApplyMotionConsistencyFilter(decel_filter, measured, predicted, 1.0 / 60.0, config);
    BT_CHECK(decel_filter.telemetry.targets[Index(bt::MotionTarget::Root)].decision == bt::MotionFilterDecision::Accepted);
    BT_CHECK_NEAR(out.root.position.x, 0.06, 1e-5);


    bt::MotionConsistencyFilterState free_foot_filter;
    predicted = StateAt(0.0f, -0.2f, 0.2f);
    measured = predicted;
    out = bt::ApplyMotionConsistencyFilter(free_foot_filter, measured, predicted, 1.0 / 60.0, config);
    BT_CHECK(free_foot_filter.telemetry.targets[Index(bt::MotionTarget::LeftFoot)].decision == bt::MotionFilterDecision::Accepted);

    predicted = out;
    measured = out;
    measured.left_foot.position.x += 0.025f;
    out = bt::ApplyMotionConsistencyFilter(free_foot_filter, measured, predicted, 1.0 / 60.0, config);
    BT_CHECK(free_foot_filter.telemetry.targets[Index(bt::MotionTarget::LeftFoot)].decision == bt::MotionFilterDecision::Pending);
    BT_CHECK(free_foot_filter.telemetry.targets[Index(bt::MotionTarget::LeftFoot)].reason == bt::MotionFilterReason::PendingConfirmation);
    BT_CHECK(free_foot_filter.telemetry.targets[Index(bt::MotionTarget::LeftFoot)].expected_distance_m >= 0.025f);
    BT_CHECK_NEAR(out.left_foot.position.x, -0.2, 1e-5);

    predicted = out;
    measured = out;
    measured.left_foot.position.x += 0.025f;
    out = bt::ApplyMotionConsistencyFilter(free_foot_filter, measured, predicted, 1.0 / 60.0, config);
    BT_CHECK(free_foot_filter.telemetry.targets[Index(bt::MotionTarget::LeftFoot)].decision == bt::MotionFilterDecision::Accepted);
    BT_CHECK(free_foot_filter.telemetry.targets[Index(bt::MotionTarget::LeftFoot)].reason == bt::MotionFilterReason::PendingConfirmation);
    BT_CHECK_NEAR(out.left_foot.position.x, -0.175, 1e-5);

    bt::MotionConsistencyFilterState planted_filter;
    predicted = StateAt(0.0f, -0.2f, 0.2f);
    measured = predicted;
    measured.support.left_foot.type = bt::FootSupportType::FloorSupport;
    measured.support.left_foot.phase = bt::FootSupportPhase::FlatPlant;
    measured.support.left_foot.anchor.active = true;
    measured.support.left_foot.anchor.pose = measured.left_foot;
    measured.support.left_foot.anchor.confidence = 1.0f;
    out = bt::ApplyMotionConsistencyFilter(planted_filter, measured, predicted, 1.0 / 60.0, config);
    BT_CHECK(planted_filter.telemetry.targets[Index(bt::MotionTarget::LeftFoot)].decision == bt::MotionFilterDecision::HeldAnchor);

    predicted = out;
    measured.left_foot.position.x += 0.20f;
    out = bt::ApplyMotionConsistencyFilter(planted_filter, measured, predicted, 1.0 / 60.0, config);
    BT_CHECK(planted_filter.telemetry.targets[Index(bt::MotionTarget::LeftFoot)].decision == bt::MotionFilterDecision::RejectedHeld);
    BT_CHECK(planted_filter.telemetry.targets[Index(bt::MotionTarget::LeftFoot)].reason == bt::MotionFilterReason::AbsoluteSpeed);
    BT_CHECK_NEAR(out.left_foot.position.x, -0.2, 1e-5);

    predicted = out;
    measured.left_foot.position.x += 0.20f;
    out = bt::ApplyMotionConsistencyFilter(planted_filter, measured, predicted, 1.0 / 60.0, config);
    BT_CHECK(planted_filter.telemetry.targets[Index(bt::MotionTarget::LeftFoot)].decision == bt::MotionFilterDecision::RejectedHeld);
    BT_CHECK(planted_filter.telemetry.targets[Index(bt::MotionTarget::LeftFoot)].pending_frames == 2);
    BT_CHECK(planted_filter.telemetry.targets[Index(bt::MotionTarget::LeftFoot)].reason == bt::MotionFilterReason::AbsoluteSpeed);

    bt::MotionConsistencyFilterState root_contact_filter;
    bt::MotionConsistencyConfig root_config = config;
    root_config.contact_root_max_residual_m = 0.05f;
    root_config.contact_root_max_disagreement_m = 0.01f;
    root_config.contact_root_max_correction_m = 0.10f;
    predicted = StateAt(0.0f, -0.2f, 0.2f);
    measured = predicted;
    out = bt::ApplyMotionConsistencyFilter(root_contact_filter, measured, predicted, 1.0 / 60.0, root_config);
    BT_CHECK(root_contact_filter.telemetry.targets[Index(bt::MotionTarget::Root)].decision == bt::MotionFilterDecision::Accepted);

    predicted = StateAt(1.00f, -0.20f, 0.20f);
    measured = predicted;
    MakeFlatPlant(measured.support.left_foot, predicted.left_foot);
    MakeFlatPlant(measured.support.right_foot, predicted.right_foot);
    measured.root.position.x += 0.02f;
    measured.left_foot.position.x += 0.02f;
    measured.right_foot.position.x += 0.02f;
    out = bt::ApplyMotionConsistencyFilter(root_contact_filter, measured, predicted, 1.0 / 60.0, root_config);
    BT_CHECK(root_contact_filter.telemetry.contact_root.reason == bt::MotionFilterReason::ContactRootCommonMode);
    BT_CHECK(root_contact_filter.telemetry.contact_root.root_innovation_m < 0.05f);
    BT_CHECK_NEAR(root_contact_filter.telemetry.contact_root.root_innovation.x, 0.02f, 1e-4);

    return 0;
}
