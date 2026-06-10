#pragma once

#include "calibration/calibration_types.h"
#include "core/status.h"

#include <opencv2/core.hpp>
#include <vector>

namespace bt {

Result<CameraCalibration> CalibrateIntrinsicsFromChessboardFrames(
    const std::vector<cv::Mat>& frames,
    cv::Size board_size,
    float square_size_meters);

} // namespace bt
