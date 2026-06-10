#include "test_check.h"

#include "core/types.h"
#include "inference/keypoint_contract.h"
#include "inference/rtmpose_model_contract.h"
#include "tracking/foot_frame.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace {

bt::ModelTensorInfo Tensor(std::string name, std::vector<std::int64_t> dims) {
    bt::ModelTensorInfo t;
    t.name = std::move(name);
    t.element_type = bt::TensorElementType::Float32;
    t.dims = std::move(dims);
    t.is_tensor = true;
    t.has_dynamic_dims = std::any_of(t.dims.begin(), t.dims.end(), [](std::int64_t d) { return d <= 0; });
    return t;
}

bt::ModelSessionInfo SessionWithOutputs(std::vector<bt::ModelTensorInfo> outputs) {
    bt::ModelSessionInfo info;
    info.loaded = true;
    info.inputs.push_back(Tensor("input", {-1, 3, 384, 288}));
    info.outputs = std::move(outputs);
    return info;
}

void Put(bt::LowerBodyJointSet& joints, bt::KeypointId id, bt::Vec3f world, float confidence = 1.0f) {
    auto& kp = joints.joints[static_cast<std::size_t>(id)];
    kp.world = world;
    kp.confidence = confidence;
    kp.present = true;
}

void TestInternalKeypointOrder() {
    static_assert(bt::kInternalKeypointCount == 26);
    static_assert(bt::InternalKeypointOrderIsCanonical());
    // Backward-compat alias must match.
    static_assert(bt::kHalpe26Count == bt::kInternalKeypointCount);

    const std::array<const char*, bt::kInternalKeypointCount> expected{
        "nose",
        "left_eye",
        "right_eye",
        "left_ear",
        "right_ear",
        "left_shoulder",
        "right_shoulder",
        "left_elbow",
        "right_elbow",
        "left_wrist",
        "right_wrist",
        "left_hip",
        "right_hip",
        "left_knee",
        "right_knee",
        "left_ankle",
        "right_ankle",
        "head",
        "neck",
        "hip",
        "left_big_toe",
        "right_big_toe",
        "left_small_toe",
        "right_small_toe",
        "left_heel",
        "right_heel",
    };

    for (std::size_t i = 0; i < expected.size(); ++i) {
        const auto id = bt::kInternalKeypointOrder[i];
        BT_CHECK(static_cast<std::size_t>(id) == i);
        BT_CHECK(std::string(bt::ToString(id)) == expected[i]);
        BT_CHECK(std::string(bt::kInternalKeypointNames[i]) == expected[i]);
    }
    // Backward-compat aliases must work.
    BT_CHECK(bt::kHalpe26KeypointOrder == bt::kInternalKeypointOrder);
    BT_CHECK(bt::Halpe26KeypointOrderIsCanonical());

    BT_CHECK(static_cast<int>(bt::KeypointId::LeftHip) == 11);
    BT_CHECK(static_cast<int>(bt::KeypointId::RightHip) == 12);
    BT_CHECK(static_cast<int>(bt::KeypointId::LeftKnee) == 13);
    BT_CHECK(static_cast<int>(bt::KeypointId::RightKnee) == 14);
    BT_CHECK(static_cast<int>(bt::KeypointId::LeftAnkle) == 15);
    BT_CHECK(static_cast<int>(bt::KeypointId::RightAnkle) == 16);
    BT_CHECK(static_cast<int>(bt::KeypointId::Pelvis) == 19);
    BT_CHECK(static_cast<int>(bt::KeypointId::LeftBigToe) == 20);
    BT_CHECK(static_cast<int>(bt::KeypointId::RightBigToe) == 21);
    BT_CHECK(static_cast<int>(bt::KeypointId::LeftSmallToe) == 22);
    BT_CHECK(static_cast<int>(bt::KeypointId::RightSmallToe) == 23);
    BT_CHECK(static_cast<int>(bt::KeypointId::LeftHeel) == 24);
    BT_CHECK(static_cast<int>(bt::KeypointId::RightHeel) == 25);

    BT_CHECK(bt::IsLowerBodyKeypoint(bt::KeypointId::Pelvis));
    BT_CHECK(bt::IsFootKeypoint(bt::KeypointId::LeftBigToe));
    BT_CHECK(bt::IsFootKeypoint(bt::KeypointId::RightBigToe));
    BT_CHECK(bt::IsFootKeypoint(bt::KeypointId::LeftSmallToe));
    BT_CHECK(bt::IsFootKeypoint(bt::KeypointId::RightSmallToe));
    BT_CHECK(bt::IsFootKeypoint(bt::KeypointId::LeftHeel));
    BT_CHECK(bt::IsFootKeypoint(bt::KeypointId::RightHeel));
    BT_CHECK(!bt::IsFootKeypoint(bt::KeypointId::LeftAnkle));
}

void TestModelContractAcceptsProvidedRtmPoseXHalpe26SimCC() {
    const auto info = SessionWithOutputs({
        Tensor("simcc_x", {1, 26, 576}),
        Tensor("simcc_y", {1, 26, 768}),
    });
    BT_CHECK(bt::ValidateRtmPoseModelContract(info).ok());
}

void TestModelContractAcceptsXYCExport() {
    const auto info = SessionWithOutputs({
        Tensor("keypoints", {1, 26, 3}),
    });
    BT_CHECK(bt::ValidateRtmPoseModelContract(info).ok());
}

void TestModelContractAcceptsRtmw3dWholeBodyExport() {
    const auto info = SessionWithOutputs({
        Tensor("output", {1, 133, 576}),
        Tensor("1554", {1, 133, 768}),
        Tensor("1556", {1, 133, 576}),
    });
    BT_CHECK(bt::ValidateRtmPoseModelContract(info).ok());
}

void TestModelContractAcceptsRtmwWholeBodyExport() {
    const auto info = SessionWithOutputs({
        Tensor("simcc_x", {1, 133, 576}),
        Tensor("simcc_y", {1, 133, 768}),
    });
    BT_CHECK(bt::ValidateRtmPoseModelContract(info).ok());
}

void TestModelContractRejectsWrongKeypointCounts() {
    const auto coco17 = SessionWithOutputs({
        Tensor("simcc_x", {1, 17, 576}),
        Tensor("simcc_y", {1, 17, 768}),
    });
    BT_CHECK(!bt::ValidateRtmPoseModelContract(coco17).ok());

    const auto wrong_rtmw3d_count = SessionWithOutputs({
        Tensor("output", {1, 132, 576}),
        Tensor("1554", {1, 132, 768}),
        Tensor("1556", {1, 132, 576}),
    });
    BT_CHECK(!bt::ValidateRtmPoseModelContract(wrong_rtmw3d_count).ok());
}

void TestModelContractRejectsWrongInputAndUnobservedOutputs() {
    auto nhwc = SessionWithOutputs({
        Tensor("simcc_x", {1, 26, 576}),
        Tensor("simcc_y", {1, 26, 768}),
    });
    nhwc.inputs[0] = Tensor("input", {1, 384, 288, 3});
    BT_CHECK(!bt::ValidateRtmPoseModelContract(nhwc).ok());

    auto old_l_shape = SessionWithOutputs({
        Tensor("simcc_x", {1, 26, 384}),
        Tensor("simcc_y", {1, 26, 512}),
    });
    old_l_shape.inputs[0] = Tensor("input", {1, 3, 256, 192});
    BT_CHECK(!bt::ValidateRtmPoseModelContract(old_l_shape).ok());

    const auto symbolic_keypoint_metadata = SessionWithOutputs({
        Tensor("simcc_x", {-1, -1, 576}),
        Tensor("simcc_y", {-1, -1, 768}),
    });
    BT_CHECK(!bt::ValidateRtmPoseModelContract(symbolic_keypoint_metadata).ok());

    const auto wrong_bins = SessionWithOutputs({
        Tensor("simcc_x", {1, 26, 384}),
        Tensor("simcc_y", {1, 26, 512}),
    });
    BT_CHECK(!bt::ValidateRtmPoseModelContract(wrong_bins).ok());
}

void TestFootFrameUsesHalpe26ToeHeelAnkleInputs() {
    bt::LowerBodyJointSet joints;
    Put(joints, bt::KeypointId::LeftAnkle, bt::Vec3f{0.0f, 1.0f, 0.0f});
    Put(joints, bt::KeypointId::LeftHeel, bt::Vec3f{-0.10f, 0.0f, 0.0f});
    Put(joints, bt::KeypointId::LeftBigToe, bt::Vec3f{0.10f, 0.0f, 0.20f});
    Put(joints, bt::KeypointId::LeftSmallToe, bt::Vec3f{-0.10f, 0.0f, 0.20f});
    Put(joints, bt::KeypointId::LeftKnee, bt::Vec3f{0.0f, 1.5f, 0.0f});

    const bt::FootSupportState previous_support;
    const bt::Pose3f previous_pose;
    const bt::LowerBodyModel model;
    const auto frame = bt::InferFootFrame(bt::BodySide::Left, joints, previous_support, previous_pose, model);

    BT_CHECK(frame.valid);
    BT_CHECK(frame.used_toe_heel);
    BT_CHECK_NEAR(frame.ankle.x, 0.0, 1e-6);
    BT_CHECK_NEAR(frame.ankle.y, 1.0, 1e-6);
    BT_CHECK_NEAR(frame.heel.x, -0.10, 1e-6);
    BT_CHECK_NEAR(frame.heel.z, 0.0, 1e-6);
    BT_CHECK_NEAR(frame.toe.x, 0.0, 1e-6);
    BT_CHECK_NEAR(frame.toe.z, 0.20, 1e-6);
    BT_CHECK_NEAR(frame.sole_center.x, -0.05, 1e-6);
    BT_CHECK_NEAR(frame.sole_center.z, 0.10, 1e-6);
    BT_CHECK_NEAR(frame.foot_pose.position.x, frame.sole_center.x, 1e-6);
    BT_CHECK_NEAR(frame.foot_pose.position.z, frame.sole_center.z, 1e-6);
}

void TestFootFrameUsesRightFootIndicesSeparately() {
    bt::LowerBodyJointSet joints;
    Put(joints, bt::KeypointId::LeftAnkle, bt::Vec3f{-9.0f, -9.0f, -9.0f});
    Put(joints, bt::KeypointId::LeftHeel, bt::Vec3f{-9.0f, -9.0f, -9.0f});
    Put(joints, bt::KeypointId::LeftBigToe, bt::Vec3f{-9.0f, -9.0f, -9.0f});
    Put(joints, bt::KeypointId::LeftSmallToe, bt::Vec3f{-9.0f, -9.0f, -9.0f});

    Put(joints, bt::KeypointId::RightAnkle, bt::Vec3f{1.0f, 1.0f, 1.0f});
    Put(joints, bt::KeypointId::RightHeel, bt::Vec3f{0.8f, 0.0f, 1.0f});
    Put(joints, bt::KeypointId::RightBigToe, bt::Vec3f{1.2f, 0.0f, 1.3f});
    Put(joints, bt::KeypointId::RightSmallToe, bt::Vec3f{0.8f, 0.0f, 1.3f});

    const auto frame = bt::InferFootFrame(bt::BodySide::Right, joints, bt::FootSupportState{}, bt::Pose3f{}, bt::LowerBodyModel{});
    BT_CHECK(frame.valid);
    BT_CHECK(frame.used_toe_heel);
    BT_CHECK_NEAR(frame.ankle.x, 1.0, 1e-6);
    BT_CHECK_NEAR(frame.heel.x, 0.8, 1e-6);
    BT_CHECK_NEAR(frame.toe.x, 1.0, 1e-6);
    BT_CHECK_NEAR(frame.toe.z, 1.3, 1e-6);
}

} // namespace

int main() {
    TestInternalKeypointOrder();
    TestModelContractAcceptsProvidedRtmPoseXHalpe26SimCC();
    TestModelContractAcceptsXYCExport();
    TestModelContractAcceptsRtmw3dWholeBodyExport();
    TestModelContractAcceptsRtmwWholeBodyExport();
    TestModelContractRejectsWrongKeypointCounts();
    TestModelContractRejectsWrongInputAndUnobservedOutputs();
    TestFootFrameUsesHalpe26ToeHeelAnkleInputs();
    TestFootFrameUsesRightFootIndicesSeparately();
    return 0;
}
