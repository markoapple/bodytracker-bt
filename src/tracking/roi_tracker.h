#pragma once

#include "core/types.h"
#include "inference/rtmpose_decode.h"

#include <string>

namespace bt {

struct RoiTrackerConfig {
    float min_width_fraction = 0.35f;
    float min_height_fraction = 0.35f;
    float margin_fraction = 0.22f;
    float min_margin_px = 24.0f;
    float expand_gain = 0.65f;
    float shrink_gain = 0.20f;
    float center_gain = 0.45f;
    float reacquire_growth = 1.15f;
    int max_lost_updates_before_full_frame = 8;
    float min_keypoint_confidence = 0.15f;
    int min_points_for_lock = 6;
};

struct RoiDiagnostics {
    float center_shift_px = 0.0f;
    float scale_x_ratio = 1.0f;
    float scale_y_ratio = 1.0f;
    int contributing_points = 0;
};

struct RoiState {
    bool initialized = false;
    bool pose_locked = false;
    bool in_reacquire = false;
    Rect2f rect{};
    float confidence = 0.0f;
    int stable_updates = 0;
    int lost_updates = 0;
    PostureMode mode_hint = PostureMode::UnknownFree;
    RoiDiagnostics diagnostics{};
};

class RoiTracker {
public:
    explicit RoiTracker(RoiTrackerConfig config = {});

    void Reset();
    void InitializeFullFrame(int frame_width, int frame_height);
    void InitializeRect(int frame_width, int frame_height, Rect2f rect);
    [[nodiscard]] const RoiState& GetState() const noexcept;

    RoiState Update(
        int frame_width,
        int frame_height,
        const DecodedPose2D* decoded_pose,
        PostureMode mode_hint = PostureMode::UnknownFree);

private:
    RoiTrackerConfig config_;
    RoiState state_{};
};

std::string BuildRoiStateSummary(const RoiState& state);

} // namespace bt
