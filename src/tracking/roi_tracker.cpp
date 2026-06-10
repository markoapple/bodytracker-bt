#include "tracking/roi_tracker.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <sstream>

namespace bt {
namespace {

float Clamp(float v, float lo, float hi) {
    if (!std::isfinite(v)) {
        return lo;
    }
    return std::max(lo, std::min(hi, v));
}

Rect2f FullFrameRect(int w, int h) {
    return Rect2f{0.0f, 0.0f, static_cast<float>(std::max(1, w)), static_cast<float>(std::max(1, h))};
}

float CenterX(const Rect2f& r) { return r.x + 0.5f * r.width; }
float CenterY(const Rect2f& r) { return r.y + 0.5f * r.height; }

Rect2f RectFromCenter(float cx, float cy, float w, float h) {
    return Rect2f{cx - 0.5f * w, cy - 0.5f * h, w, h};
}

Rect2f ClampRect(Rect2f r, int frame_width, int frame_height) {
    const float fw = static_cast<float>(std::max(1, frame_width));
    const float fh = static_cast<float>(std::max(1, frame_height));
    r.width = Clamp(r.width, 1.0f, fw);
    r.height = Clamp(r.height, 1.0f, fh);
    r.x = Clamp(r.x, 0.0f, fw - r.width);
    r.y = Clamp(r.y, 0.0f, fh - r.height);
    return r;
}

std::optional<Rect2f> ComputeTarget(
    const DecodedPose2D& pose,
    const RoiTrackerConfig& cfg,
    int frame_width,
    int frame_height,
    PostureMode mode,
    int* out_count) {

    float min_x = std::numeric_limits<float>::infinity();
    float min_y = std::numeric_limits<float>::infinity();
    float max_x = -std::numeric_limits<float>::infinity();
    float max_y = -std::numeric_limits<float>::infinity();
    int count = 0;

    for (const auto& kp : pose.keypoints) {
        if (!kp.present ||
            !std::isfinite(kp.confidence) ||
            !std::isfinite(kp.pixel.x) ||
            !std::isfinite(kp.pixel.y) ||
            kp.confidence < cfg.min_keypoint_confidence) {
            continue;
        }
        min_x = std::min(min_x, kp.pixel.x);
        min_y = std::min(min_y, kp.pixel.y);
        max_x = std::max(max_x, kp.pixel.x);
        max_y = std::max(max_y, kp.pixel.y);
        ++count;
    }

    if (out_count) {
        *out_count = count;
    }
    if (count < cfg.min_points_for_lock) {
        return std::nullopt;
    }

    float w = std::max(1.0f, max_x - min_x);
    float h = std::max(1.0f, max_y - min_y);
    float margin_x = std::max(cfg.min_margin_px, cfg.margin_fraction * w);
    float margin_y = std::max(cfg.min_margin_px, cfg.margin_fraction * h);

    if (mode == PostureMode::ReclinedSupported) {
        margin_x *= 1.65f;
        margin_y *= 1.20f;
    } else if (mode == PostureMode::SeatedSupported) {
        margin_x *= 1.35f;
        margin_y *= 1.30f;
    }

    Rect2f target{min_x - margin_x, min_y - margin_y, w + 2.0f * margin_x, h + 2.0f * margin_y};
    const float min_w = cfg.min_width_fraction * static_cast<float>(frame_width);
    const float min_h = cfg.min_height_fraction * static_cast<float>(frame_height);
    if (target.width < min_w || target.height < min_h) {
        const float cx = CenterX(target);
        const float cy = CenterY(target);
        target.width = std::max(target.width, min_w);
        target.height = std::max(target.height, min_h);
        target = RectFromCenter(cx, cy, target.width, target.height);
    }
    return ClampRect(target, frame_width, frame_height);
}

Rect2f Blend(const Rect2f& current, const Rect2f& target, const RoiTrackerConfig& cfg) {
    const float w_gain = (target.width > current.width) ? cfg.expand_gain : cfg.shrink_gain;
    const float h_gain = (target.height > current.height) ? cfg.expand_gain : cfg.shrink_gain;
    const float new_w = current.width + w_gain * (target.width - current.width);
    const float new_h = current.height + h_gain * (target.height - current.height);
    const float new_cx = CenterX(current) + cfg.center_gain * (CenterX(target) - CenterX(current));
    const float new_cy = CenterY(current) + cfg.center_gain * (CenterY(target) - CenterY(current));
    return RectFromCenter(new_cx, new_cy, new_w, new_h);
}

float CenterShift(const Rect2f& a, const Rect2f& b) {
    const float dx = CenterX(b) - CenterX(a);
    const float dy = CenterY(b) - CenterY(a);
    return std::sqrt(dx * dx + dy * dy);
}

} // namespace

RoiTracker::RoiTracker(RoiTrackerConfig config) : config_(config) {}

void RoiTracker::Reset() {
    state_ = {};
}

void RoiTracker::InitializeFullFrame(int frame_width, int frame_height) {
    state_ = {};
    state_.initialized = true;
    state_.rect = FullFrameRect(frame_width, frame_height);
    state_.in_reacquire = true;
}

void RoiTracker::InitializeRect(int frame_width, int frame_height, Rect2f rect) {
    state_ = {};
    state_.initialized = true;
    state_.rect = ClampRect(rect, frame_width, frame_height);
    state_.in_reacquire = true;
}

const RoiState& RoiTracker::GetState() const noexcept {
    return state_;
}

RoiState RoiTracker::Update(int frame_width, int frame_height, const DecodedPose2D* decoded_pose, PostureMode mode_hint) {
    if (!state_.initialized) {
        InitializeFullFrame(frame_width, frame_height);
    }

    const Rect2f previous = state_.rect;
    int contributing = 0;
    state_.mode_hint = mode_hint;

    if (decoded_pose && decoded_pose->valid) {
        const auto target = ComputeTarget(*decoded_pose, config_, frame_width, frame_height, mode_hint, &contributing);
        if (target.has_value()) {
            state_.rect = state_.pose_locked ? ClampRect(Blend(state_.rect, *target, config_), frame_width, frame_height) : *target;
            state_.stable_updates += 1;
            state_.lost_updates = 0;
            state_.in_reacquire = false;
            state_.confidence = std::min(1.0f, state_.confidence + 0.18f);
            state_.pose_locked = state_.stable_updates >= 2 && state_.confidence >= 0.35f;
        } else {
            state_.stable_updates = 0;
            state_.lost_updates += 1;
            state_.confidence *= 0.75f;
            state_.pose_locked = false;
            state_.in_reacquire = true;
        }
    } else {
        state_.stable_updates = 0;
        state_.lost_updates += 1;
        state_.confidence *= 0.70f;
        state_.pose_locked = false;
        state_.in_reacquire = true;
    }

    if (state_.in_reacquire) {
        if (state_.lost_updates > config_.max_lost_updates_before_full_frame) {
            state_.rect = FullFrameRect(frame_width, frame_height);
            state_.confidence = 0.0f;
        } else {
            state_.rect = ClampRect(
                RectFromCenter(CenterX(state_.rect), CenterY(state_.rect), state_.rect.width * config_.reacquire_growth, state_.rect.height * config_.reacquire_growth),
                frame_width,
                frame_height);
        }
    }

    state_.diagnostics.center_shift_px = CenterShift(previous, state_.rect);
    state_.diagnostics.scale_x_ratio = previous.width > 1e-3f ? state_.rect.width / previous.width : 1.0f;
    state_.diagnostics.scale_y_ratio = previous.height > 1e-3f ? state_.rect.height / previous.height : 1.0f;
    state_.diagnostics.contributing_points = contributing;
    return state_;
}

std::string BuildRoiStateSummary(const RoiState& state) {
    std::ostringstream oss;
    oss << "roi initialized=" << (state.initialized ? "true" : "false")
        << " locked=" << (state.pose_locked ? "true" : "false")
        << " reacquire=" << (state.in_reacquire ? "true" : "false")
        << " rect=(" << state.rect.x << "," << state.rect.y << "," << state.rect.width << "," << state.rect.height << ")"
        << " mode_hint=" << ToString(state.mode_hint);
    return oss.str();
}

} // namespace bt
