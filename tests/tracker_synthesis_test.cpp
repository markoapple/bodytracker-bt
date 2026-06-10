#include "tracking/tracker_synthesis.h"
#include "tracking/body_model.h"
#include "tracking/body_state.h"
#include "test_check.h"

static_assert(bt::kTrackerPoseCount == 8);
static_assert(bt::kTrackerRoles.size() == bt::kTrackerPoseCount);
static_assert(std::tuple_size<bt::TrackerPoseArray>::value == bt::kTrackerPoseCount);

namespace {

bt::Pose3f Pose(float x, float y, float z) {
    bt::Pose3f pose;
    pose.position = bt::Vec3f{x, y, z};
    pose.orientation = bt::Quatf{0.0f, 0.0f, 0.0f, 1.0f};
    return pose;
}

} // namespace

int main() {
    bt::LowerBodyState state;
    state.root = Pose(0.0f, 1.0f, 0.0f);
    state.left_foot = Pose(-0.2f, 0.0f, -0.1f);
    state.right_foot = Pose(0.2f, 0.0f, 0.1f);
    state.confidence = 0.10f;
    state.support.left_foot.type = bt::FootSupportType::FloorSupport;
    state.support.left_foot.anchor.active = true;
    state.support.left_foot.anchor.confidence = 1.0f;

    const auto trackers = bt::SynthesizeTrackerPoses(state);
    BT_CHECK(bt::TrackerRoleIndex(bt::TrackerRole::Pelvis) == 0);
    BT_CHECK(bt::TrackerRoleIndex(bt::TrackerRole::LeftFoot) == 1);
    BT_CHECK(bt::TrackerRoleIndex(bt::TrackerRole::RightFoot) == 2);
    BT_CHECK(bt::TrackerRoleIndex(bt::TrackerRole::Chest) == 3);
    BT_CHECK(bt::TrackerRoleIndex(bt::TrackerRole::LeftElbow) == 4);
    BT_CHECK(bt::TrackerRoleIndex(bt::TrackerRole::RightElbow) == 5);
    BT_CHECK(bt::TrackerRoleIndex(bt::TrackerRole::LeftKnee) == 6);
    BT_CHECK(bt::TrackerRoleIndex(bt::TrackerRole::RightKnee) == 7);
    for (std::size_t i = 0; i < trackers.size(); ++i) {
        BT_CHECK(trackers[i].role == bt::kTrackerRoles[i]);
    }
    BT_CHECK(trackers[0].role == bt::TrackerRole::Pelvis);
    BT_CHECK(trackers[1].role == bt::TrackerRole::LeftFoot);
    BT_CHECK(trackers[2].role == bt::TrackerRole::RightFoot);
    BT_CHECK(trackers[3].role == bt::TrackerRole::Chest);
    BT_CHECK(trackers[4].role == bt::TrackerRole::LeftElbow);
    BT_CHECK(trackers[5].role == bt::TrackerRole::RightElbow);
    BT_CHECK(trackers[6].role == bt::TrackerRole::LeftKnee);
    BT_CHECK(trackers[7].role == bt::TrackerRole::RightKnee);
    BT_CHECK_NEAR(trackers[0].confidence, 0.10, 1e-5);
    BT_CHECK_NEAR(trackers[1].confidence, 1.0, 1e-5);
    BT_CHECK_NEAR(trackers[2].confidence, 0.10, 1e-5);
    BT_CHECK(!trackers[3].valid);
    BT_CHECK(!trackers[4].valid);
    BT_CHECK(!trackers[5].valid);
    BT_CHECK(trackers[6].valid);
    BT_CHECK(trackers[7].valid);

    bt::UnifiedBodyState partial_body;
    partial_body.valid = false;
    partial_body.lower_body.left_foot = Pose(-0.2f, 0.0f, -0.1f);
    auto& anchored_left_foot = partial_body.roles[bt::BodyJointRoleIndex(bt::BodyJointRole::LeftFoot)];
    anchored_left_foot.role = bt::BodyJointRole::LeftFoot;
    anchored_left_foot.position = partial_body.lower_body.left_foot.position;
    anchored_left_foot.confidence = 0.75f;
    anchored_left_foot.valid = true;
    anchored_left_foot.visibility = bt::BodyJointVisibility::Anchored;
    anchored_left_foot.evidence.source = bt::TrackerEvidenceSource::AnchorHeld;
    anchored_left_foot.evidence.support_confidence = 0.75f;
    anchored_left_foot.evidence.anchor_held = true;
    anchored_left_foot.evidence.valid = true;

    bt::LowerBodyModel model;
    const auto partial_trackers = bt::SynthesizeTrackerPoses(partial_body, model);
    const auto& partial_left = partial_trackers[bt::TrackerRoleIndex(bt::TrackerRole::LeftFoot)];
    BT_CHECK(partial_left.valid);
    BT_CHECK(partial_left.evidence.source == bt::TrackerEvidenceSource::AnchorHeld);
    BT_CHECK_NEAR(partial_left.confidence, 0.75, 1e-5);

    bt::UnifiedBodyState predicted_body;
    predicted_body.valid = false;
    predicted_body.lower_body.left_foot = Pose(-0.1f, 0.0f, -0.2f);
    auto& predicted_left_foot = predicted_body.roles[bt::BodyJointRoleIndex(bt::BodyJointRole::LeftFoot)];
    predicted_left_foot.role = bt::BodyJointRole::LeftFoot;
    predicted_left_foot.position = predicted_body.lower_body.left_foot.position;
    predicted_left_foot.confidence = 0.12f;
    predicted_left_foot.valid = true;
    predicted_left_foot.visibility = bt::BodyJointVisibility::Predicted;
    predicted_left_foot.predicted = true;
    predicted_left_foot.evidence.source = bt::TrackerEvidenceSource::Predicted;
    predicted_left_foot.evidence.direct_confidence = 0.12f;
    predicted_left_foot.evidence.valid = true;

    const auto predicted_trackers = bt::SynthesizeTrackerPoses(predicted_body, model);
    const auto& predicted_left = predicted_trackers[bt::TrackerRoleIndex(bt::TrackerRole::LeftFoot)];
    BT_CHECK(predicted_left.valid);
    BT_CHECK(predicted_left.evidence.source == bt::TrackerEvidenceSource::Predicted);
    BT_CHECK_NEAR(predicted_left.confidence, 0.12, 1e-5);

    bt::UnifiedBodyState upper_body;
    upper_body.valid = false;
    upper_body.lower_body.root = Pose(0.0f, 1.0f, 0.0f);
    auto& chest = upper_body.roles[bt::BodyJointRoleIndex(bt::BodyJointRole::Chest)];
    chest.role = bt::BodyJointRole::Chest;
    chest.position = bt::Vec3f{0.0f, 1.45f, 0.05f};
    chest.confidence = 0.62f;
    chest.valid = true;
    chest.visibility = bt::BodyJointVisibility::Visible;
    chest.triangulated = true;
    chest.evidence.source = bt::TrackerEvidenceSource::DirectStereo;
    chest.evidence.direct_confidence = 0.62f;
    chest.evidence.valid = true;
    auto& left_elbow = upper_body.roles[bt::BodyJointRoleIndex(bt::BodyJointRole::LeftElbow)];
    left_elbow.role = bt::BodyJointRole::LeftElbow;
    left_elbow.position = bt::Vec3f{-0.35f, 1.18f, 0.04f};
    left_elbow.confidence = 0.58f;
    left_elbow.valid = true;
    left_elbow.visibility = bt::BodyJointVisibility::Visible;
    left_elbow.depth_inferred = true;
    left_elbow.evidence.source = bt::TrackerEvidenceSource::InferredMonocular;
    left_elbow.evidence.direct_confidence = 0.58f;
    left_elbow.evidence.valid = true;
    auto& right_elbow = upper_body.roles[bt::BodyJointRoleIndex(bt::BodyJointRole::RightElbow)];
    right_elbow.role = bt::BodyJointRole::RightElbow;
    right_elbow.position = bt::Vec3f{0.35f, 1.18f, 0.04f};
    right_elbow.confidence = 0.22f;
    right_elbow.valid = true;
    right_elbow.visibility = bt::BodyJointVisibility::Predicted;
    right_elbow.predicted = true;
    right_elbow.evidence.source = bt::TrackerEvidenceSource::Predicted;
    right_elbow.evidence.direct_confidence = 0.22f;
    right_elbow.evidence.valid = true;

    const auto upper_trackers = bt::SynthesizeTrackerPoses(upper_body, model);
    BT_CHECK(upper_trackers[bt::TrackerRoleIndex(bt::TrackerRole::Chest)].valid);
    BT_CHECK(upper_trackers[bt::TrackerRoleIndex(bt::TrackerRole::LeftElbow)].valid);
    BT_CHECK(upper_trackers[bt::TrackerRoleIndex(bt::TrackerRole::RightElbow)].valid);
    BT_CHECK(upper_trackers[bt::TrackerRoleIndex(bt::TrackerRole::Chest)].evidence.source == bt::TrackerEvidenceSource::DirectStereo);
    BT_CHECK(upper_trackers[bt::TrackerRoleIndex(bt::TrackerRole::LeftElbow)].evidence.source == bt::TrackerEvidenceSource::InferredMonocular);
    BT_CHECK(upper_trackers[bt::TrackerRoleIndex(bt::TrackerRole::RightElbow)].evidence.source == bt::TrackerEvidenceSource::Predicted);


    return 0;
}
