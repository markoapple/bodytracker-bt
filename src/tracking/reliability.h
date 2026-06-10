#pragma once

#include "core/status.h"
#include "core/types.h"
#include "inference/rtmpose_decode.h"
#include "tracking/roi_tracker.h"

#include <array>
#include <string>

namespace bt {

struct JointReliability {
    float final_weight = 0.0f;
    float model_term = 0.0f;
    float crop_edge_term = 1.0f;
    float image_edge_term = 1.0f;
    float temporal_term = 0.0f;
    bool temporal_computed = false;
    float crop_stability_term = 1.0f;
    float posture_mode_term = 1.0f;
    bool usable = false;
};

struct ReliabilitySummary {
    std::array<JointReliability, kHalpe26Count> joints{};
    float mean_weight = 0.0f;
    float lower_body_mean = 0.0f;
    float foot_mean = 0.0f;
};

struct ReliabilityConfig {
    float min_present_confidence = 0.05f;
    float crop_edge_soft_fraction = 0.06f;
    float image_edge_soft_px = 20.0f;
    float temporal_sigma_fraction = 0.10f;
    float temporal_floor = 0.15f;
};

Result<ReliabilitySummary> ComputeViewReliability(
    const DecodedPose2D& current_pose,
    const RoiState& roi_used_for_this_frame,
    int frame_width,
    int frame_height,
    PostureMode posture_mode,
    const DecodedPose2D* previous_pose = nullptr,
    const ReliabilityConfig& config = {});

std::string BuildReliabilitySummary(const ReliabilitySummary& summary);

} // namespace bt
