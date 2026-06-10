#pragma once

#include "calibration/calibration_types.h"
#include "core/status.h"

#include <opencv2/core.hpp>
#include <vector>

namespace bt {

struct StereoChessboardObservation {
    std::vector<cv::Point2f> camera_a_points;
    std::vector<cv::Point2f> camera_b_points;
};


Result<CalibrationBundle> CalibrateStereoExtrinsicsFromChessboardObservations(
    const CalibrationBundle& input,
    cv::Size image_size,
    cv::Size board_size,
    float square_size_meters,
    const std::vector<StereoChessboardObservation>& observations);

} // namespace bt
