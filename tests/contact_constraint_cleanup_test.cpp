#include "tracking/contact_constraints.h"
#include "tracking/motion_consistency_filter.h"
#include "test_check.h"

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

bt::FootSupportState Support(bt::FootSupportPhase phase, const bt::Pose3f& anchor) {
    bt::FootSupportState support;
    support.type = bt::FootSupportType::FloorSupport;
    support.phase = phase;
    support.anchor.active = true;
    support.anchor.pose = anchor;
    support.anchor.confidence = 1.0f;
    support.heel_anchor = support.anchor;
    support.heel_anchor.pose.position = bt::FootHeelContactPoint(anchor);
    support.toe_anchor = support.anchor;
    support.toe_anchor.pose.position = bt::FootToeContactPoint(anchor);
    return support;
}

bt::LowerBodyState Standing(float root_x, float left_x, float right_x) {
    bt::LowerBodyState state;
    state.root = Pose(root_x, 0.90f, 0.0f);
    state.left_foot = Pose(left_x, 0.0f, -0.10f);
    state.right_foot = Pose(right_x, 0.0f, 0.10f);
    state.support.left_foot = Support(bt::FootSupportPhase::FlatPlant, Pose(-0.20f, 0.0f, -0.10f));
    state.support.right_foot = Support(bt::FootSupportPhase::FlatPlant, Pose(0.20f, 0.0f, 0.10f));
    state.confidence = 0.95f;
    return state;
}

} // namespace

int main() {
    bt::MotionConsistencyConfig config;
    config.min_motion_m = 0.004f;
    config.stationary_deadzone_m = 0.001f;
    config.planted_foot_max_drift_m = 0.045f;
    config.contact_root_correction_gain = 0.50f;
    config.contact_root_max_correction_m = 0.020f;
    config.contact_root_max_residual_m = 0.030f;
    config.contact_root_max_disagreement_m = 0.006f;
    config.contact_root_min_alignment = 0.70f;
    config.contact_root_min_support_confidence = 0.80f;

    const double dt = 1.0 / 60.0;

    bt::MotionConsistencyFilterState common_mode_filter;
    auto predicted = Standing(0.0f, -0.20f, 0.20f);
    auto measured = predicted;
    auto out = bt::ApplyMotionConsistencyFilter(common_mode_filter, measured, predicted, dt, config);
    BT_CHECK(common_mode_filter.telemetry.targets[Index(bt::MotionTarget::Root)].decision == bt::MotionFilterDecision::Accepted);

    predicted = out;
    measured = Standing(0.012f, -0.188f, 0.212f);
    out = bt::ApplyMotionConsistencyFilter(common_mode_filter, measured, predicted, dt, config);
    BT_CHECK(common_mode_filter.telemetry.contact_root.applied);
    BT_CHECK(common_mode_filter.telemetry.contact_root.reason == bt::MotionFilterReason::ContactRootCommonMode);
    BT_CHECK_NEAR(common_mode_filter.telemetry.contact_root.left_residual.x, 0.012f, 1e-5);
    BT_CHECK_NEAR(common_mode_filter.telemetry.contact_root.right_residual.x, 0.012f, 1e-5);
    BT_CHECK_NEAR(common_mode_filter.telemetry.contact_root.common_residual.x, 0.012f, 1e-5);
    BT_CHECK_NEAR(common_mode_filter.telemetry.contact_root.root_innovation.x, 0.012f, 1e-5);
    BT_CHECK_NEAR(common_mode_filter.telemetry.contact_root.common_residual_m, 0.012f, 1e-5);
    BT_CHECK_NEAR(common_mode_filter.telemetry.contact_root.root_innovation_m, 0.012f, 1e-5);
    BT_CHECK(out.root.position.x < measured.root.position.x);
    BT_CHECK(out.root.position.x > 0.0f);
    BT_CHECK_NEAR(out.left_foot.position.x, -0.20f, 1e-5);
    BT_CHECK_NEAR(out.right_foot.position.x, 0.20f, 1e-5);

    bt::MotionConsistencyFilterState foot_only_filter;
    predicted = Standing(0.0f, -0.20f, 0.20f);
    measured = predicted;
    out = bt::ApplyMotionConsistencyFilter(foot_only_filter, measured, predicted, dt, config);
    predicted = out;
    measured = Standing(0.0f, -0.188f, 0.20f);
    out = bt::ApplyMotionConsistencyFilter(foot_only_filter, measured, predicted, dt, config);
    BT_CHECK(!foot_only_filter.telemetry.contact_root.applied);
    BT_CHECK(foot_only_filter.telemetry.contact_root.reason == bt::MotionFilterReason::ContactRootRejectedDisagreement);
    BT_CHECK_NEAR(out.root.position.x, 0.0f, 1e-5);
    BT_CHECK_NEAR(out.left_foot.position.x, -0.20f, 1e-5);

    bt::MotionConsistencyFilterState body_over_filter;
    predicted = Standing(0.0f, -0.20f, 0.20f);
    measured = predicted;
    out = bt::ApplyMotionConsistencyFilter(body_over_filter, measured, predicted, dt, config);
    predicted = out;
    measured = Standing(0.035f, -0.20f, 0.20f);
    out = bt::ApplyMotionConsistencyFilter(body_over_filter, measured, predicted, dt, config);
    BT_CHECK(!body_over_filter.telemetry.contact_root.applied);
    BT_CHECK(body_over_filter.telemetry.contact_root.reason == bt::MotionFilterReason::ContactRootRejectedRootMismatch);
    BT_CHECK(out.root.position.x > 0.01f);
    BT_CHECK_NEAR(out.left_foot.position.x, -0.20f, 1e-5);

    bt::MotionConsistencyFilterState toe_pivot_filter;
    bt::LowerBodyState toe_state = Standing(0.0f, -0.20f, 0.20f);
    toe_state.support.left_foot = Support(bt::FootSupportPhase::ToePivot, toe_state.left_foot);
    toe_state.support.right_foot.type = bt::FootSupportType::None;
    toe_state.support.right_foot.anchor.active = false;
    out = bt::ApplyMotionConsistencyFilter(toe_pivot_filter, toe_state, toe_state, dt, config);
    predicted = out;
    bt::LowerBodyState toe_measured = toe_state;
    toe_measured.left_foot = Pose(-0.20f, 0.071545f, -0.077500f);
    toe_measured.left_foot.orientation = bt::Quatf{0.30f, 0.0f, 0.0f, 0.953939f};
    out = bt::ApplyMotionConsistencyFilter(toe_pivot_filter, toe_measured, predicted, dt, config);
    BT_CHECK_NEAR(bt::FootToeContactPoint(out.left_foot).x, bt::FootToeContactPoint(toe_state.left_foot).x, 1e-5);
    BT_CHECK_NEAR(bt::FootToeContactPoint(out.left_foot).z, bt::FootToeContactPoint(toe_state.left_foot).z, 1e-5);
    BT_CHECK(out.left_foot.orientation.x > 0.20f);

    bt::MotionConsistencyFilterState slip_filter;
    predicted = Standing(0.0f, -0.20f, 0.20f);
    measured = predicted;
    measured.support.left_foot.phase = bt::FootSupportPhase::Slip;
    out = bt::ApplyMotionConsistencyFilter(slip_filter, measured, predicted, dt, config);
    predicted = out;
    measured.left_foot.position.x -= 0.10f;
    out = bt::ApplyMotionConsistencyFilter(slip_filter, measured, predicted, dt, config);
    BT_CHECK(slip_filter.telemetry.targets[Index(bt::MotionTarget::LeftFoot)].decision == bt::MotionFilterDecision::Accepted);
    BT_CHECK_NEAR(out.left_foot.position.x, measured.left_foot.position.x, 1e-5);


    bt::MotionConsistencyFilterState weak_evidence_root_filter;
    predicted = Standing(0.0f, -0.20f, 0.20f);
    measured = predicted;
    measured.support.left_foot.anchor.confidence = 0.20f;
    measured.support.right_foot.anchor.confidence = 0.20f;
    out = bt::ApplyMotionConsistencyFilter(weak_evidence_root_filter, measured, predicted, dt, config);
    predicted = out;
    measured = Standing(0.012f, -0.188f, 0.212f);
    measured.support.left_foot.anchor.confidence = 0.20f;
    measured.support.right_foot.anchor.confidence = 0.20f;
    out = bt::ApplyMotionConsistencyFilter(weak_evidence_root_filter, measured, predicted, dt, config);
    BT_CHECK(!weak_evidence_root_filter.telemetry.contact_root.applied);
    BT_CHECK(weak_evidence_root_filter.telemetry.contact_root.reason == bt::MotionFilterReason::ContactRootRejectedSingleFoot);
    BT_CHECK(out.root.position.x >= 0.0f);
    BT_CHECK(out.root.position.x <= measured.root.position.x);

    return 0;
}
