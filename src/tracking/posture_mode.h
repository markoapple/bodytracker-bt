#pragma once

#include "core/types.h"

namespace bt {

struct PostureModeScores {
    float upright = 0.0f;
    float crouching = 0.0f;
    float kneeling = 0.0f;
    float seated = 0.0f;
    float reclined = 0.0f;
    float unknown = 1.0f;
};

struct PostureClassifierState {
    PostureMode mode = PostureMode::UnknownFree;
    PostureModeScores scores{};
    float confidence = 0.0f;
};

PostureClassifierState UpdatePostureMode(
    const PostureClassifierState& previous,
    const LowerBodyState& solved_state,
    const HmdPoseSample& hmd,
    double dt_seconds);

} // namespace bt
