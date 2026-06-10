#include "debug/overlay_draw.h"

#include <opencv2/imgproc.hpp>

namespace bt {

void DrawPoseOverlay(cv::Mat& bgr, const DecodedPose2D& pose, const ReliabilitySummary* reliability) {
    if (bgr.empty() || !pose.valid) {
        return;
    }
    for (std::size_t i = 0; i < kHalpe26Count; ++i) {
        const auto& kp = pose.keypoints[i];
        if (!kp.present) {
            continue;
        }
        const float w = reliability ? reliability->joints[i].final_weight : kp.confidence;
        const cv::Scalar color(0.0, 255.0 * w, 255.0 * (1.0 - w));
        cv::circle(bgr, cv::Point(static_cast<int>(kp.pixel.x), static_cast<int>(kp.pixel.y)), 3, color, -1);
    }
}

void DrawRoiOverlay(cv::Mat& bgr, const RoiState& roi) {
    if (bgr.empty() || !roi.initialized) {
        return;
    }
    cv::rectangle(
        bgr,
        cv::Rect(
            static_cast<int>(roi.rect.x),
            static_cast<int>(roi.rect.y),
            static_cast<int>(roi.rect.width),
            static_cast<int>(roi.rect.height)),
        cv::Scalar(255.0, 200.0, 0.0),
        2);
}

} // namespace bt
