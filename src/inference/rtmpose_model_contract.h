#pragma once

#include "core/status.h"
#include "inference/rtmpose_session.h"

namespace bt {

inline constexpr std::int64_t kRtmPoseHalpe26InputHeight = 384;
inline constexpr std::int64_t kRtmPoseHalpe26InputWidth = 288;
inline constexpr std::int64_t kRtmPoseHalpe26SimccXBins = kRtmPoseHalpe26InputWidth * 2;
inline constexpr std::int64_t kRtmPoseHalpe26SimccYBins = kRtmPoseHalpe26InputHeight * 2;
inline constexpr std::int64_t kRtmw3dWholeBodyKeypointCount = 133;
inline constexpr std::int64_t kRtmw3dSimccZBins = kRtmPoseHalpe26InputWidth * 2;

Status ValidateRtmPoseImageInputContract(const ModelSessionInfo& info);
Status ValidateRtmPoseOutputContract(const ModelSessionInfo& info);
Status ValidateRtmPoseModelContract(const ModelSessionInfo& info);

} // namespace bt
