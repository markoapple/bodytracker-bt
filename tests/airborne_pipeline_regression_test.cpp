#include "tracking/motion_consistency_filter.h"
#include "tracking/temporal_update.h"
#include "tracking/tracker_ekf.h"
#include "tracking/tracker_synthesis.h"
#include "test_check.h"

#include <cmath>
#include <cstddef>

namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr int kFrames = 72;

std::size_t Index(bt::MotionTarget target) {
    return static_cast<std::size_t>(target);
}

bt::Pose3f Pose(float x, float y, float z) {
    bt::Pose3f pose;
    pose.position = bt::Vec3f{x, y, z};
    pose.orientation = bt::Quatf{0.0f, 0.0f, 0.0f, 1.0f};
    return pose;
}

bt::LowerBodyState AirborneFrame(int frame) {
    const float t = static_cast<float>(frame) / static_cast<float>(kFrames - 1);
    bt::LowerBodyState state;
    state.root = Pose(0.0f, 0.90f, 0.0f);
    state.left_foot = Pose(
        -0.22f + 0.44f * t,
        0.035f + 0.075f * std::sin(kPi * t),
        -0.06f + 0.025f * std::sin(2.0f * kPi * t));
    state.right_foot = Pose(0.20f, 0.04f, 0.10f);
    state.left_knee_flexion = 0.40f + 0.20f * std::sin(kPi * t);
    state.right_knee_flexion = 0.15f;
    state.confidence = 0.95f;
    state.posture_mode = bt::PostureMode::UprightStanding;
    return state;
}

float PathLength(float previous, float current) {
    return std::abs(current - previous);
}

} // namespace

int main() {
    bt::MotionConsistencyConfig motion_config;
    motion_config.min_motion_m = 0.004f;
    motion_config.stationary_deadzone_m = 0.001f;
    motion_config.max_direction_deviation_deg = 60.0f;
    motion_config.max_lateral_deviation_ratio = 0.75f;
    motion_config.max_speed_change_ratio = 3.5f;

    bt::TrackerEkfConfig ekf_config;
    ekf_config.enabled = true;
    ekf_config.process_noise_mps2 = 24.0f;
    ekf_config.min_measurement_variance_m2 = 0.000004f;
    ekf_config.max_measurement_variance_m2 = 0.0025f;
    ekf_config.foot_orientation_gain = 1.0f;

    bt::MotionConsistencyFilterState motion_filter;
    bt::TrackerEkfState ekf;
    const double dt = 1.0 / 60.0;

    bt::LowerBodyState state = AirborneFrame(0);
    state = bt::ApplyMotionConsistencyFilter(motion_filter, state, state, dt, motion_config);
    bt::TrackerEkfTelemetry telemetry;
    state = bt::ApplyTrackerEkf(ekf, state, dt, ekf_config, &telemetry);

    float measured_path = 0.0f;
    float corrected_path = 0.0f;
    float tracker_path = 0.0f;
    float previous_measured_x = state.left_foot.position.x;
    float previous_corrected_x = state.left_foot.position.x;
    float previous_tracker_x = state.left_foot.position.x;
    int low_expected = 0;
    int bad_decisions = 0;

    for (int frame = 1; frame < kFrames; ++frame) {
        const bt::LowerBodyState measured = AirborneFrame(frame);
        const bt::LowerBodyState predicted = bt::PredictState(state, dt);
        const bt::LowerBodyState filtered = bt::ApplyMotionConsistencyFilter(motion_filter, measured, predicted, dt, motion_config);
        const bt::LowerBodyState ekf_state = bt::ApplyTrackerEkf(ekf, filtered, dt, ekf_config, &telemetry);
        state = bt::CorrectState(
            predicted,
            ekf_state,
            dt,
            bt::TemporalUpdateConfig{},
            bt::TemporalPositionCorrectionMode::DirectMeasuredPositions);
        const auto trackers = bt::SynthesizeTrackerPoses(state);

        measured_path += PathLength(previous_measured_x, measured.left_foot.position.x);
        corrected_path += PathLength(previous_corrected_x, state.left_foot.position.x);
        tracker_path += PathLength(previous_tracker_x, trackers[1].pose.position.x);
        previous_measured_x = measured.left_foot.position.x;
        previous_corrected_x = state.left_foot.position.x;
        previous_tracker_x = trackers[1].pose.position.x;

        const auto& foot = motion_filter.telemetry.targets[Index(bt::MotionTarget::LeftFoot)];
        if (frame > 6 && foot.expected_distance_m <= motion_config.min_motion_m) {
            ++low_expected;
        }
        if (foot.decision == bt::MotionFilterDecision::Blended ||
            foot.decision == bt::MotionFilterDecision::LowMotionHeld ||
            foot.decision == bt::MotionFilterDecision::RejectedHeld ||
            foot.decision == bt::MotionFilterDecision::Pending) {
            ++bad_decisions;
        }
    }

    const float corrected_ratio = corrected_path / measured_path;
    const float tracker_ratio = tracker_path / measured_path;
    BT_CHECK(low_expected == 0);
    BT_CHECK(bad_decisions <= 2);
    BT_CHECK(corrected_ratio > 0.88f);
    BT_CHECK(corrected_ratio < 1.08f);
    BT_CHECK(tracker_ratio > 0.88f);
    BT_CHECK(tracker_ratio < 1.08f);
    BT_CHECK(state.left_foot.position.x > 0.20f);

    return 0;
}
