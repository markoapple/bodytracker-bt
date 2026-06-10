#include "tracking/motion_consistency_filter.h"
#include "tracking/temporal_update.h"
#include "test_check.h"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr int kFrames = 500;
constexpr int kStepFrames = 30;
constexpr float kRootStepM = 0.012f;

std::size_t Index(bt::MotionTarget target) {
    return static_cast<std::size_t>(target);
}

bt::Pose3f Pose(float x, float y, float z) {
    bt::Pose3f pose;
    pose.position = bt::Vec3f{x, y, z};
    pose.orientation = bt::Quatf{0.0f, 0.0f, 0.0f, 1.0f};
    return pose;
}

bt::FootSupportState PlantedSupport(const bt::Pose3f& anchor) {
    bt::FootSupportState support;
    support.type = bt::FootSupportType::FloorSupport;
    support.phase = bt::FootSupportPhase::FlatPlant;
    support.anchor.active = true;
    support.anchor.pose = anchor;
    support.anchor.confidence = 1.0f;
    return support;
}

bt::FootSupportState SwingSupport() {
    bt::FootSupportState support;
    support.type = bt::FootSupportType::None;
    support.phase = bt::FootSupportPhase::Swing;
    support.anchor.active = false;
    return support;
}

struct GriddyGenerator {
    bt::Pose3f left_anchor = Pose(-0.18f, 0.0f, -0.11f);
    bt::Pose3f right_anchor = Pose(0.18f, 0.0f, 0.11f);
    bool previous_left_planted = true;
    bool initialized = false;

    bt::LowerBodyState Frame(int frame) {
        const bool left_planted = ((frame / kStepFrames) % 2) == 0;
        const float root_x = kRootStepM * static_cast<float>(frame);
        const float sway = 0.035f * std::sin(2.0f * kPi * static_cast<float>(frame) / static_cast<float>(kStepFrames * 2));
        const float bob = 0.015f * std::sin(2.0f * kPi * static_cast<float>(frame) / 10.0f);
        const bt::Pose3f root = Pose(root_x, 0.90f + bob, sway);

        if (!initialized) {
            previous_left_planted = left_planted;
            initialized = true;
        } else if (left_planted != previous_left_planted) {
            if (left_planted) {
                left_anchor = SwingFoot(root, true, frame);
            } else {
                right_anchor = SwingFoot(root, false, frame);
            }
            previous_left_planted = left_planted;
        }

        bt::LowerBodyState state;
        state.root = root;
        state.left_foot = left_planted ? left_anchor : SwingFoot(root, true, frame);
        state.right_foot = left_planted ? SwingFoot(root, false, frame) : right_anchor;
        state.left_hip_flexion = 0.12f;
        state.right_hip_flexion = 0.12f;
        state.left_knee_flexion = 0.18f;
        state.right_knee_flexion = 0.18f;
        state.confidence = 0.92f;
        state.support.left_foot = left_planted ? PlantedSupport(left_anchor) : SwingSupport();
        state.support.right_foot = left_planted ? SwingSupport() : PlantedSupport(right_anchor);
        state.posture_mode = bt::PostureMode::UprightStanding;
        return state;
    }

    static bt::Pose3f SwingFoot(const bt::Pose3f& root, bool left, int frame) {
        const int phase_frame = frame % kStepFrames;
        const float phase = static_cast<float>(phase_frame) / static_cast<float>(kStepFrames - 1);
        const float lateral = left ? -0.11f : 0.11f;
        const float heel_flick = 0.010f * std::sin(2.0f * kPi * phase);
        const float lift = 0.020f + 0.025f * std::sin(kPi * phase);
        return Pose(
            root.position.x + (left ? -0.18f : 0.18f) + heel_flick,
            lift,
            root.position.z + lateral + 0.006f * std::sin(4.0f * kPi * phase));
    }
};

} // namespace

int main() {
    bt::MotionConsistencyConfig motion_config;
    motion_config.confirm_frames = 2;
    motion_config.min_motion_m = 0.006f;
    motion_config.stationary_deadzone_m = 0.002f;
    motion_config.max_direction_deviation_deg = 55.0f;
    motion_config.max_lateral_deviation_ratio = 0.70f;
    motion_config.max_speed_change_ratio = 3.0f;
    motion_config.planted_foot_release_confirm_frames = 2;
    motion_config.planted_foot_max_drift_m = 0.04f;

    bt::TemporalUpdateConfig temporal_config;
    bt::MotionConsistencyFilterState filter;
    GriddyGenerator generator;

    const double dt = 1.0 / 60.0;
    bt::LowerBodyState state = generator.Frame(0);
    state.linear_velocity = bt::Vec3f{};
    state = bt::ApplyMotionConsistencyFilter(filter, state, state, dt, motion_config);

    int left_swing_frames_checked = 0;
    int right_swing_frames_checked = 0;
    int low_expected_swing_frames = 0;
    int held_swing_frames = 0;
    float min_forward_velocity = 1000.0f;

    for (int frame = 1; frame < kFrames; ++frame) {
        const bt::LowerBodyState measured = generator.Frame(frame);
        const bt::LowerBodyState predicted = bt::PredictState(state, dt);
        const bt::LowerBodyState filtered = bt::ApplyMotionConsistencyFilter(filter, measured, predicted, dt, motion_config);
        state = bt::CorrectState(
            predicted,
            filtered,
            dt,
            temporal_config,
            bt::TemporalPositionCorrectionMode::DirectMeasuredPositions);

        const bool left_planted = measured.support.left_foot.type != bt::FootSupportType::None;
        const auto& left = filter.telemetry.targets[Index(bt::MotionTarget::LeftFoot)];
        const auto& right = filter.telemetry.targets[Index(bt::MotionTarget::RightFoot)];

        const bool transition_frame = (frame % kStepFrames) <= 1;
        if (!transition_frame && frame > 8) {
            const auto& swing = left_planted ? right : left;
            if (left_planted) {
                ++right_swing_frames_checked;
            } else {
                ++left_swing_frames_checked;
            }
            if (swing.expected_distance_m <= motion_config.min_motion_m) {
                ++low_expected_swing_frames;
            }
            if (swing.decision == bt::MotionFilterDecision::LowMotionHeld ||
                swing.decision == bt::MotionFilterDecision::RejectedHeld) {
                ++held_swing_frames;
            }
        }

        if (!transition_frame && frame > 8) {
            min_forward_velocity = std::min(min_forward_velocity, state.linear_velocity.x);
        }

        if (left_planted) {
            BT_CHECK_NEAR(state.left_foot.position.x, measured.support.left_foot.anchor.pose.position.x, 1e-4);
            BT_CHECK_NEAR(state.left_foot.position.z, measured.support.left_foot.anchor.pose.position.z, 1e-4);
        } else {
            BT_CHECK_NEAR(state.right_foot.position.x, measured.support.right_foot.anchor.pose.position.x, 1e-4);
            BT_CHECK_NEAR(state.right_foot.position.z, measured.support.right_foot.anchor.pose.position.z, 1e-4);
        }
    }

    BT_CHECK(left_swing_frames_checked > 150);
    BT_CHECK(right_swing_frames_checked > 150);
    BT_CHECK(low_expected_swing_frames == 0);
    BT_CHECK(held_swing_frames == 0);
    BT_CHECK(min_forward_velocity > 0.55f);
    BT_CHECK(state.root.position.x > 5.5f);

    return 0;
}
