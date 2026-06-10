#pragma once

#include "calibration/calibration_types.h"
#include "core/types.h"

#include <array>
#include <map>
#include <string>

namespace bt {

enum class LowerBodySegment {
    Pelvis = 0,
    LeftFemur,
    LeftTibia,
    LeftFoot,
    RightFemur,
    RightTibia,
    RightFoot
};

struct LowerBodyModel {
    float pelvis_width = 0.32f;
    float left_femur = 0.42f;
    float right_femur = 0.42f;
    float left_tibia = 0.42f;
    float right_tibia = 0.42f;
    float left_foot_length = 0.24f;
    float right_foot_length = 0.24f;
    Vec3f standing_hmd_to_pelvis{0.0f, -0.75f, 0.0f};
    Vec3f seated_hmd_to_pelvis{0.0f, -0.55f, -0.15f};
    Vec3f reclined_hmd_to_pelvis{0.0f, -0.25f, -0.45f};
};

LowerBodyModel MakeLowerBodyModel(const BodyCalibration& calibration);
std::string BuildBodyModelSummary(const LowerBodyModel& model);

struct LowerBodyJointSet {
    std::array<Keypoint3D, kHalpe26Count> joints{};
};

LowerBodyJointSet PredictLowerBodyJoints(const LowerBodyState& state, const LowerBodyModel& model);
void SolveLeg3DFromFootTarget(LowerBodyState& state, const LowerBodyModel& model, bool left);
// Compatibility shim for older callers/tests. The runtime solver is now 3D;
// this name no longer means sagittal-only.
void SolveSagittalLegFromFootTarget(LowerBodyState& state, const LowerBodyModel& model, bool left);
Pose3f FootPoseFromAnkleTarget(
    const Vec3f& ankle,
    const Pose3f& previous_foot_pose,
    const LowerBodyModel& model,
    bool left);
LowerBodyState EstimateStateFromJointSeeds(
    const LowerBodyState& predicted,
    const LowerBodyModel& model,
    const LowerBodyJointSet& seeds,
    float seed_weight);

} // namespace bt
