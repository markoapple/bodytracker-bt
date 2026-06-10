#include "tracking/body_model.h"
#include "test_check.h"

namespace {

bt::Keypoint3D Joint(float x, float y, float z) {
    bt::Keypoint3D joint;
    joint.world = bt::Vec3f{x, y, z};
    joint.confidence = 1.0f;
    joint.present = true;
    return joint;
}

void Put(bt::LowerBodyJointSet& set, bt::KeypointId id, const bt::Keypoint3D& joint) {
    set.joints[static_cast<std::size_t>(id)] = joint;
}

bt::LowerBodyModel Model() {
    bt::LowerBodyModel model;
    model.pelvis_width = 0.30f;
    model.left_femur = 0.40f;
    model.left_tibia = 0.40f;
    model.left_foot_length = 0.24f;
    model.right_femur = 0.40f;
    model.right_tibia = 0.40f;
    model.right_foot_length = 0.24f;
    return model;
}

} // namespace

int main() {
    const auto model = Model();

    bt::LowerBodyState predicted;
    predicted.root.position = bt::Vec3f{0.0f, 1.0f, 0.0f};
    predicted.left_foot.position = bt::Vec3f{-0.15f, 0.2f, 0.0f};
    predicted.right_foot.position = bt::Vec3f{0.15f, 0.2f, 0.0f};
    predicted.confidence = 0.1f;

    bt::LowerBodyJointSet good;
    Put(good, bt::KeypointId::LeftHip, Joint(-0.15f, 1.0f, 0.0f));
    Put(good, bt::KeypointId::RightHip, Joint(0.15f, 1.0f, 0.0f));
    Put(good, bt::KeypointId::LeftKnee, Joint(-0.15f, 0.60f, 0.0f));
    Put(good, bt::KeypointId::LeftAnkle, Joint(-0.15f, 0.20f, 0.0f));

    const auto good_state = bt::EstimateStateFromJointSeeds(predicted, model, good, 1.0f);
    // The runtime foot pose is the sole/foot center, not the ankle keypoint.
    // With identity foot orientation, the center sits below and slightly forward
    // from the ankle measurement.
    BT_CHECK_NEAR(good_state.left_foot.position.y, 0.165, 1e-5);

    bt::LowerBodyJointSet impossible = good;
    Put(impossible, bt::KeypointId::LeftKnee, Joint(-0.15f, 0.20f, 0.0f));
    Put(impossible, bt::KeypointId::LeftAnkle, Joint(-0.15f, -0.50f, 0.0f));

    const auto bad_state = bt::EstimateStateFromJointSeeds(predicted, model, impossible, 1.0f);
    BT_CHECK(bad_state.left_foot.position.y > -0.14f);
    BT_CHECK(bad_state.left_foot.position.y < 0.18f);

    bt::LowerBodyState far_target;
    far_target.root.position = bt::Vec3f{0.0f, 1.0f, 0.0f};
    far_target.left_foot.position = bt::Vec3f{-0.15f, -0.40f, 0.80f};
    far_target.left_foot.orientation = bt::Quatf{};
    far_target.confidence = 1.0f;
    const bt::Vec3f far_requested_foot = far_target.left_foot.position;
    bt::SolveLeg3DFromFootTarget(far_target, model, true);
    BT_CHECK(far_target.left_leg_reach_clamped);
    BT_CHECK(bt::Distance(far_target.left_foot.position, far_requested_foot) > 0.10f);
    const bt::Vec3f far_left_hip{-0.5f * model.pelvis_width, 1.0f, 0.0f};
    const bt::Vec3f far_left_ankle{
        far_target.left_foot.position.x,
        far_target.left_foot.position.y + 0.035f,
        far_target.left_foot.position.z - 0.20f * model.left_foot_length};
    BT_CHECK(bt::Distance(far_left_hip, far_left_ankle) <= model.left_femur + model.left_tibia + 1e-4f);

    bt::LowerBodyJointSet bad_pelvis = good;
    Put(bad_pelvis, bt::KeypointId::RightHip, Joint(1.5f, 1.0f, 0.0f));
    const auto bad_pelvis_state = bt::EstimateStateFromJointSeeds(predicted, model, bad_pelvis, 1.0f);
    BT_CHECK_NEAR(bad_pelvis_state.root.position.x, 0.0, 1e-5);

    bt::LowerBodyState lateral;
    lateral.root.position = bt::Vec3f{0.0f, 1.0f, 0.0f};
    lateral.left_foot.position = bt::Vec3f{-0.42f, 0.315f, 0.168f};
    lateral.left_foot.orientation = bt::Quatf{};
    lateral.confidence = 1.0f;
    bt::SolveLeg3DFromFootTarget(lateral, model, true);
    const auto lateral_joints = bt::PredictLowerBodyJoints(lateral, model);
    const auto& lateral_hip = lateral_joints.joints[static_cast<std::size_t>(bt::KeypointId::LeftHip)];
    const auto& lateral_knee = lateral_joints.joints[static_cast<std::size_t>(bt::KeypointId::LeftKnee)];
    const auto& lateral_ankle = lateral_joints.joints[static_cast<std::size_t>(bt::KeypointId::LeftAnkle)];
    BT_CHECK_NEAR(bt::Distance(lateral_hip.world, lateral_knee.world), model.left_femur, 1e-4);
    BT_CHECK_NEAR(bt::Distance(lateral_knee.world, lateral_ankle.world), model.left_tibia, 1e-4);
    BT_CHECK_NEAR(lateral_ankle.world.x, -0.42, 1e-4);
    BT_CHECK(std::abs(lateral.left_hip_abduction) > 0.05f);

    return 0;
}
