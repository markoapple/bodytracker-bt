#pragma once

#include "core/types.h"

namespace bt {

struct JointLimitConfig {
    float knee_min_flexion = 0.0f;
    float knee_max_flexion = 2.75f;
    float ankle_pitch_abs_max = 1.1f;
    float ankle_roll_abs_max = 0.75f;
    float ankle_yaw_abs_max = 1.15f;
    float hip_flexion_abs_max = 2.4f;
    float hip_abduction_abs_max = 0.95f;
};

LowerBodyState ApplyJointLimitBounds(const LowerBodyState& state, const JointLimitConfig& config = {});

} // namespace bt
