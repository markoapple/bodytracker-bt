#pragma once

#include "tracking/tracking_pipeline.h"

#include <string>

namespace bt {

std::string BuildWorldDebugSummary(const TrackingPipelineSnapshot& snapshot);

} // namespace bt
