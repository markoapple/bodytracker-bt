#include "tracking/posture_mode.h"

#include <algorithm>
#include <cmath>

namespace bt {
namespace {

float Clamp01(float v) {
    if (!std::isfinite(v)) {
        return 0.0f;
    }
    return std::max(0.0f, std::min(1.0f, v));
}

float HeightScore(float value, float center, float half_width) {
    return Clamp01(1.0f - std::abs(value - center) / std::max(0.001f, half_width));
}

float MaxKneeFlexion(const LowerBodyState& s) {
    return std::max(s.left_knee_flexion, s.right_knee_flexion);
}

float MeanFootRootSeparation(const LowerBodyState& s) {
    const float left = std::abs(s.root.position.y - s.left_foot.position.y);
    const float right = std::abs(s.root.position.y - s.right_foot.position.y);
    return 0.5f * (left + right);
}

float HmdUpDot(const HmdPoseSample& hmd) {
    if (!hmd.valid) {
        return 1.0f;
    }
    const Vec3f up = Rotate(hmd.pose.orientation, Vec3f{0.0f, 1.0f, 0.0f});
    return Clamp01(0.5f * (up.y + 1.0f));
}

PostureMode BestMode(const PostureModeScores& scores, float* out_score) {
    PostureMode best_mode = PostureMode::UnknownFree;
    float best = scores.unknown;
    const auto consider = [&](PostureMode mode, float score) {
        if (score > best) {
            best = score;
            best_mode = mode;
        }
    };
    consider(PostureMode::UprightStanding, scores.upright);
    consider(PostureMode::Crouching, scores.crouching);
    consider(PostureMode::Kneeling, scores.kneeling);
    consider(PostureMode::SeatedSupported, scores.seated);
    consider(PostureMode::ReclinedSupported, scores.reclined);
    if (out_score) {
        *out_score = best;
    }
    return best_mode;
}

} // namespace

PostureClassifierState UpdatePostureMode(
    const PostureClassifierState& previous,
    const LowerBodyState& solved_state,
    const HmdPoseSample& hmd,
    double dt_seconds) {
    PostureClassifierState out = previous;
    out.scores = {};
    const float head_height = hmd.valid ? hmd.pose.position.y : solved_state.root.position.y + 0.75f;
    const float root_height = solved_state.root.position.y;
    const float foot_sep = MeanFootRootSeparation(solved_state);
    const float knee_flex = MaxKneeFlexion(solved_state);
    const float hmd_upright = HmdUpDot(hmd);
    const bool feet_supported =
        solved_state.support.left_foot.type == FootSupportType::FloorSupport ||
        solved_state.support.right_foot.type == FootSupportType::FloorSupport;
    const bool root_supported =
        solved_state.support.root_support == RootSupportType::SeatSupported ||
        solved_state.support.root_support == RootSupportType::BodyRestSupported;

    out.scores.upright =
        0.45f * HeightScore(head_height, 1.65f, 0.45f) +
        0.20f * HeightScore(root_height, 0.95f, 0.45f) +
        0.20f * hmd_upright +
        0.15f * (feet_supported ? 1.0f : 0.25f);

    out.scores.crouching =
        0.35f * HeightScore(head_height, 1.10f, 0.35f) +
        0.25f * HeightScore(root_height, 0.55f, 0.30f) +
        0.25f * Clamp01(knee_flex / 1.8f) +
        0.15f * hmd_upright;

    out.scores.kneeling =
        0.30f * HeightScore(head_height, 1.00f, 0.40f) +
        0.25f * HeightScore(root_height, 0.45f, 0.25f) +
        0.25f * Clamp01(knee_flex / 2.1f) +
        0.20f * (foot_sep < 0.55f ? 1.0f : 0.20f);

    out.scores.seated =
        0.35f * HeightScore(head_height, 1.10f, 0.45f) +
        0.25f * HeightScore(root_height, 0.55f, 0.30f) +
        0.20f * (root_supported ? 1.0f : 0.35f) +
        0.20f * Clamp01(knee_flex / 1.7f);

    out.scores.reclined =
        0.35f * HeightScore(head_height, 0.55f, 0.45f) +
        0.30f * (1.0f - hmd_upright) +
        0.20f * (solved_state.support.root_support == RootSupportType::BodyRestSupported ? 1.0f : 0.30f) +
        0.15f * (foot_sep < 0.45f ? 1.0f : 0.25f);

    out.scores.unknown = 0.10f + 0.25f * (1.0f - solved_state.confidence);

    float best = 0.0f;
    const PostureMode candidate = BestMode(out.scores, &best);
    const float enter_margin = candidate == previous.mode ? 0.0f : 0.12f;
    const float persistence_gain = static_cast<float>(std::min(1.0, std::max(0.02, dt_seconds * 5.0)));

    if (candidate == previous.mode || best > previous.confidence + enter_margin || previous.confidence < 0.25f) {
        out.mode = candidate;
        out.confidence = previous.mode == candidate
            ? previous.confidence + persistence_gain * (best - previous.confidence)
            : std::max(0.20f, best);
    } else {
        out.mode = previous.mode;
        out.confidence = std::max(0.0f, previous.confidence - persistence_gain * 0.10f);
    }
    out.confidence = Clamp01(out.confidence);
    return out;
}

} // namespace bt
