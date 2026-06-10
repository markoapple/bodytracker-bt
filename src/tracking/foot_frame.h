#pragma once

#include "core/types.h"
#include "tracking/body_model.h"

namespace bt {

enum class BodySide {
    Left = 0,
    Right = 1
};

struct FootFrameEstimate {
    Pose3f foot_pose{};
    Vec3f ankle{};
    Vec3f heel{};
    Vec3f toe{};
    Vec3f sole_center{};
    Vec3f forward_axis{0.0f, 0.0f, 1.0f};
    Vec3f lateral_axis{1.0f, 0.0f, 0.0f};
    Vec3f up_axis{0.0f, 1.0f, 0.0f};
    float confidence = 0.0f;
    bool used_toe_heel = false;
    bool valid = false;
};

FootFrameEstimate InferFootFrame(
    BodySide side,
    const LowerBodyJointSet& joints,
    const FootSupportState& previous_support,
    const Pose3f& previous_foot_pose,
    const LowerBodyModel& model);

} // namespace bt
