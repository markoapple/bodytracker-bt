#include "debug/world_debug.h"

#include <sstream>

namespace bt {

std::string BuildWorldDebugSummary(const TrackingPipelineSnapshot& snapshot) {
    std::ostringstream oss;
    oss << "posture=" << ToString(snapshot.state.posture_mode)
        << " degradation=" << snapshot.degradation_mode
        << " left_foot=" << ToString(snapshot.state.support.left_foot.type)
        << "/" << ToString(snapshot.state.support.left_foot.phase)
        << " right_foot=" << ToString(snapshot.state.support.right_foot.type)
        << "/" << ToString(snapshot.state.support.right_foot.phase);
    return oss.str();
}

} // namespace bt
