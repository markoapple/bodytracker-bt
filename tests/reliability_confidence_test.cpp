#include "tracking/reliability.h"
#include "test_check.h"

namespace {

bt::DecodedPose2D Pose(float x, float y, float confidence = 0.81f) {
    bt::DecodedPose2D pose;
    pose.valid = true;
    pose.aggregate_confidence = confidence;
    auto& kp = pose.keypoints[static_cast<std::size_t>(bt::KeypointId::LeftAnkle)];
    kp.present = true;
    kp.confidence = confidence;
    kp.pixel = bt::Vec2f{x, y};
    return pose;
}

bt::RoiState Roi() {
    bt::RoiState roi;
    roi.initialized = true;
    roi.rect = bt::Rect2f{0.0f, 0.0f, 200.0f, 200.0f};
    return roi;
}

const bt::JointReliability& LeftAnkle(const bt::ReliabilitySummary& summary) {
    return summary.joints[static_cast<std::size_t>(bt::KeypointId::LeftAnkle)];
}

} // namespace

int main() {
    const auto roi = Roi();
    const auto current = Pose(100.0f, 100.0f);

    const auto first = bt::ComputeViewReliability(
        current,
        roi,
        200,
        200,
        bt::PostureMode::UprightStanding);
    BT_CHECK(first.ok());
    const auto& first_joint = LeftAnkle(first.value());
    BT_CHECK(!first_joint.temporal_computed);
    BT_CHECK_NEAR(first_joint.temporal_term, 0.0f, 1e-6f);
    BT_CHECK_NEAR(first_joint.final_weight, 0.9f, 1e-5f);

    auto previous_missing = current;
    previous_missing.keypoints[static_cast<std::size_t>(bt::KeypointId::LeftAnkle)].present = false;
    const auto missing_previous = bt::ComputeViewReliability(
        current,
        roi,
        200,
        200,
        bt::PostureMode::UprightStanding,
        &previous_missing);
    BT_CHECK(missing_previous.ok());
    BT_CHECK(!LeftAnkle(missing_previous.value()).temporal_computed);
    BT_CHECK_NEAR(LeftAnkle(missing_previous.value()).final_weight, first_joint.final_weight, 1e-6f);

    const auto far_previous_pose = Pose(0.0f, 0.0f);
    const auto far_previous = bt::ComputeViewReliability(
        current,
        roi,
        200,
        200,
        bt::PostureMode::UprightStanding,
        &far_previous_pose);
    BT_CHECK(far_previous.ok());
    const auto& far_joint = LeftAnkle(far_previous.value());
    BT_CHECK(far_joint.temporal_computed);
    BT_CHECK(far_joint.temporal_term > 0.0f);
    BT_CHECK(far_joint.temporal_term < 1.0f);
    BT_CHECK(far_joint.final_weight < first_joint.final_weight);

    return 0;
}
