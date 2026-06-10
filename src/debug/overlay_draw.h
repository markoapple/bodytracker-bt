#pragma once

#include "inference/rtmpose_decode.h"
#include "tracking/reliability.h"
#include "tracking/roi_tracker.h"

#include <opencv2/core.hpp>

namespace bt {

void DrawPoseOverlay(cv::Mat& bgr, const DecodedPose2D& pose, const ReliabilitySummary* reliability);
void DrawRoiOverlay(cv::Mat& bgr, const RoiState& roi);

} // namespace bt
