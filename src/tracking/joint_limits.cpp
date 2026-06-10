#include "tracking/joint_limits.h"

#include <algorithm>
#include <cmath>

namespace bt {
namespace {

float Clamp(float v, float lo, float hi) {
    if (!std::isfinite(v)) {
        return lo;
    }
    return std::max(lo, std::min(hi, v));
}

} // namespace

LowerBodyState ApplyJointLimitBounds(const LowerBodyState& state, const JointLimitConfig& c) {
    LowerBodyState out = state;
    out.left_knee_flexion = Clamp(out.left_knee_flexion, c.knee_min_flexion, c.knee_max_flexion);
    out.right_knee_flexion = Clamp(out.right_knee_flexion, c.knee_min_flexion, c.knee_max_flexion);
    out.left_ankle_pitch = Clamp(out.left_ankle_pitch, -c.ankle_pitch_abs_max, c.ankle_pitch_abs_max);
    out.right_ankle_pitch = Clamp(out.right_ankle_pitch, -c.ankle_pitch_abs_max, c.ankle_pitch_abs_max);
    out.left_ankle_roll = Clamp(out.left_ankle_roll, -c.ankle_roll_abs_max, c.ankle_roll_abs_max);
    out.right_ankle_roll = Clamp(out.right_ankle_roll, -c.ankle_roll_abs_max, c.ankle_roll_abs_max);
    out.left_ankle_yaw = Clamp(out.left_ankle_yaw, -c.ankle_yaw_abs_max, c.ankle_yaw_abs_max);
    out.right_ankle_yaw = Clamp(out.right_ankle_yaw, -c.ankle_yaw_abs_max, c.ankle_yaw_abs_max);
    out.left_hip_flexion = Clamp(out.left_hip_flexion, -c.hip_flexion_abs_max, c.hip_flexion_abs_max);
    out.right_hip_flexion = Clamp(out.right_hip_flexion, -c.hip_flexion_abs_max, c.hip_flexion_abs_max);
    out.left_hip_abduction = Clamp(out.left_hip_abduction, -c.hip_abduction_abs_max, c.hip_abduction_abs_max);
    out.right_hip_abduction = Clamp(out.right_hip_abduction, -c.hip_abduction_abs_max, c.hip_abduction_abs_max);
    return out;
}

} // namespace bt
