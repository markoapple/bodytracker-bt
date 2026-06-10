#include "tracking/temporal_update.h"
#include "tracking/contact_constraints.h"
#include "test_check.h"

namespace {

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

} // namespace

int main() {
    bt::LowerBodyState predicted;
    predicted.left_foot = Pose(-0.20f, 0.08f, 0.10f);
    predicted.right_foot = Pose(0.20f, 0.08f, 0.10f);

    bt::LowerBodyState measured = predicted;
    measured.left_foot = Pose(-0.18f, 0.04f, 0.10f);
    measured.right_foot = Pose(0.18f, 0.04f, 0.10f);
    measured.confidence = 0.9f;

    const bt::Pose3f left_anchor = Pose(-0.25f, 0.0f, 0.12f);
    measured.support.left_foot = Support(bt::FootSupportPhase::FlatPlant, left_anchor);
    measured.support.right_foot = Support(bt::FootSupportPhase::ContactCandidate, Pose(0.25f, 0.0f, 0.12f));

    const auto corrected = bt::CorrectState(predicted, measured, 1.0 / 60.0);
    BT_CHECK_NEAR(corrected.left_foot.position.x, left_anchor.position.x, 1e-6);
    BT_CHECK_NEAR(corrected.left_foot.position.y, left_anchor.position.y, 1e-6);
    BT_CHECK_NEAR(corrected.left_foot.position.z, left_anchor.position.z, 1e-6);
    BT_CHECK_NEAR(bt::FootHeelContactPoint(corrected.right_foot).x, bt::FootHeelContactPoint(measured.support.right_foot.anchor.pose).x, 1e-6);
    BT_CHECK_NEAR(bt::FootHeelContactPoint(corrected.right_foot).y, bt::FootHeelContactPoint(measured.support.right_foot.anchor.pose).y, 1e-6);
    BT_CHECK_NEAR(bt::FootHeelContactPoint(corrected.right_foot).z, bt::FootHeelContactPoint(measured.support.right_foot.anchor.pose).z, 1e-6);

    bt::LowerBodyState measured_heel = predicted;
    const bt::Pose3f heel_anchor = Pose(-0.30f, 0.0f, -0.05f);
    measured_heel.support.left_foot = Support(bt::FootSupportPhase::HeelLock, heel_anchor);
    const auto heel_corrected = bt::CorrectState(predicted, measured_heel, 1.0 / 60.0);
    BT_CHECK_NEAR(heel_corrected.left_foot.position.x, heel_anchor.position.x, 1e-6);
    BT_CHECK_NEAR(heel_corrected.left_foot.position.y, heel_anchor.position.y, 1e-6);
    BT_CHECK_NEAR(heel_corrected.left_foot.position.z, heel_anchor.position.z, 1e-6);

    bt::LowerBodyState custom_predicted;
    custom_predicted.root = Pose(0.0f, 1.0f, 0.0f);
    custom_predicted.left_foot = Pose(-0.20f, 0.05f, 0.0f);

    bt::LowerBodyState custom_measured = custom_predicted;
    custom_measured.root = Pose(1.0f, 1.0f, 0.0f);
    custom_measured.left_foot = Pose(-0.10f, 0.05f, 0.0f);

    bt::TemporalUpdateConfig custom_config;
    custom_config.free_gain = 0.25f;
    custom_config.foot_free_gain = 0.80f;
    const auto custom_corrected = bt::CorrectState(custom_predicted, custom_measured, 1.0 / 60.0, custom_config);
    BT_CHECK_NEAR(custom_corrected.root.position.x, 0.25, 1e-6);
    // foot_free_gain uses normal blend semantics: predicted + gain * (measured - predicted).
    BT_CHECK_NEAR(custom_corrected.left_foot.position.x, -0.12, 1e-6);


    bt::TemporalUpdateConfig direct_config;
    direct_config.free_gain = 0.0f;
    direct_config.supported_gain = 0.0f;
    direct_config.foot_free_gain = 0.0f;
    direct_config.foot_supported_gain = 0.0f;
    bt::LowerBodyState direct_predicted;
    direct_predicted.root = Pose(0.0f, 1.0f, 0.0f);
    direct_predicted.left_foot = Pose(-0.20f, 0.05f, 0.0f);
    direct_predicted.right_foot = Pose(0.20f, 0.05f, 0.0f);

    bt::LowerBodyState direct_measured = direct_predicted;
    direct_measured.root = Pose(1.0f, 1.0f, 0.0f);
    direct_measured.left_foot = Pose(-0.10f, 0.02f, 0.0f);
    direct_measured.right_foot = Pose(0.10f, 0.02f, 0.0f);

    const auto direct_corrected = bt::CorrectState(
        direct_predicted,
        direct_measured,
        1.0 / 60.0,
        direct_config,
        bt::TemporalPositionCorrectionMode::DirectMeasuredPositions);
    BT_CHECK_NEAR(direct_corrected.root.position.x, direct_measured.root.position.x, 1e-6);
    BT_CHECK_NEAR(direct_corrected.left_foot.position.x, direct_measured.left_foot.position.x, 1e-6);
    BT_CHECK_NEAR(direct_corrected.left_foot.position.y, direct_measured.left_foot.position.y, 1e-6);
    BT_CHECK_NEAR(direct_corrected.right_foot.position.x, direct_measured.right_foot.position.x, 1e-6);
    BT_CHECK_NEAR(direct_corrected.right_foot.position.y, direct_measured.right_foot.position.y, 1e-6);

    bt::LowerBodyState moving;
    moving.root = Pose(1.0f, 1.0f, 0.0f);
    moving.left_foot = Pose(0.80f, 0.05f, -0.10f);
    moving.right_foot = Pose(1.20f, 0.05f, 0.10f);
    moving.linear_velocity = bt::Vec3f{0.60f, 0.0f, 0.0f};

    const auto moving_predicted = bt::PredictState(moving, 0.5);
    BT_CHECK_NEAR(moving_predicted.root.position.x, 1.30f, 1e-6);
    BT_CHECK_NEAR(moving_predicted.left_foot.position.x, 1.10f, 1e-6);
    BT_CHECK_NEAR(moving_predicted.right_foot.position.x, 1.50f, 1e-6);


    bt::LowerBodyState velocity_state;
    velocity_state.root = Pose(0.0f, 1.0f, 0.0f);
    velocity_state.left_foot = Pose(-0.20f, 0.05f, 0.0f);
    velocity_state.right_foot = Pose(0.20f, 0.05f, 0.0f);
    velocity_state.linear_velocity = bt::Vec3f{0.60f, 0.0f, 0.0f};

    const auto velocity_predicted = bt::PredictState(velocity_state, 0.5);
    bt::LowerBodyState velocity_measured = velocity_predicted;
    const auto velocity_corrected = bt::CorrectState(
        velocity_predicted,
        velocity_measured,
        0.5,
        bt::TemporalUpdateConfig{},
        bt::TemporalPositionCorrectionMode::DirectMeasuredPositions);
    BT_CHECK_NEAR(velocity_corrected.linear_velocity.x, 0.60f, 1e-6);

    const auto velocity_next = bt::PredictState(velocity_corrected, 0.5);
    BT_CHECK_NEAR(velocity_next.root.position.x, 0.60f, 1e-6);
    BT_CHECK_NEAR(velocity_next.left_foot.position.x, 0.40f, 1e-6);
    BT_CHECK_NEAR(velocity_next.right_foot.position.x, 0.80f, 1e-6);

    moving.support.left_foot = Support(bt::FootSupportPhase::FlatPlant, moving.left_foot);
    const auto planted_predicted = bt::PredictState(moving, 0.5);
    BT_CHECK_NEAR(planted_predicted.root.position.x, 1.30f, 1e-6);
    BT_CHECK_NEAR(planted_predicted.left_foot.position.x, moving.left_foot.position.x, 1e-6);
    BT_CHECK_NEAR(planted_predicted.right_foot.position.x, 1.50f, 1e-6);


    bt::LowerBodyState free_swing;
    free_swing.root = Pose(0.0f, 1.0f, 0.0f);
    free_swing.left_foot = Pose(-0.20f, 0.05f, 0.0f);
    free_swing.right_foot = Pose(0.20f, 0.05f, 0.0f);
    free_swing.left_foot_linear_velocity = bt::Vec3f{0.90f, 0.15f, 0.0f};

    const auto free_swing_predicted = bt::PredictState(free_swing, 0.10);
    BT_CHECK_NEAR(free_swing_predicted.root.position.x, 0.0f, 1e-6);
    BT_CHECK_NEAR(free_swing_predicted.left_foot.position.x, -0.11f, 1e-6);
    BT_CHECK_NEAR(free_swing_predicted.left_foot.position.y, 0.065f, 1e-6);

    bt::LowerBodyState free_swing_measured = free_swing_predicted;
    const auto learned_velocity = bt::CorrectState(
        free_swing_predicted,
        free_swing_measured,
        0.10,
        bt::TemporalUpdateConfig{},
        bt::TemporalPositionCorrectionMode::DirectMeasuredPositions);
    BT_CHECK_NEAR(learned_velocity.left_foot_linear_velocity.x, 0.90f, 1e-5);
    BT_CHECK_NEAR(learned_velocity.left_foot_linear_velocity.y, 0.15f, 1e-5);

    bt::LowerBodyState planted_velocity_guard = free_swing;
    planted_velocity_guard.support.left_foot = Support(bt::FootSupportPhase::FlatPlant, planted_velocity_guard.left_foot);
    const auto planted_velocity_predicted = bt::PredictState(planted_velocity_guard, 0.10);
    BT_CHECK_NEAR(planted_velocity_predicted.left_foot.position.x, planted_velocity_guard.left_foot.position.x, 1e-6);
    BT_CHECK_NEAR(planted_velocity_predicted.left_foot_linear_velocity.x, 0.0f, 1e-6);

    return 0;
}
