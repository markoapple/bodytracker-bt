#pragma once

#include "calibration/calibration_types.h"
#include "core/types.h"
#include "core/status.h"

#include <filesystem>

namespace bt {

Result<CalibrationBundle> LoadCalibration(const std::filesystem::path& path);
Status SaveCalibrationTemplate(const std::filesystem::path& path);
Status SaveCalibrationBundle(const CalibrationBundle& bundle, const std::filesystem::path& path);
CalibrationReadiness EvaluateCalibrationReadiness(const CalibrationBundle& bundle, TrackingMode mode = TrackingMode::Stereo);

} // namespace bt
