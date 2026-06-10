#include "tracking/support_queries.h"

#include <algorithm>
#include <cmath>

namespace bt {

bool IsActiveFootSupport(const FootSupportState& support) noexcept {
    return support.anchor.active && support.type != FootSupportType::None;
}

bool IsLockedFootSupport(const FootSupportState& support) noexcept {
    if (!IsActiveFootSupport(support)) {
        return false;
    }
    switch (support.phase) {
    case FootSupportPhase::FlatPlant:
    case FootSupportPhase::RestLock:
        return true;
    default:
        return false;
    }
}

float FootSupportConfidence(const FootSupportState& support) noexcept {
    if (!IsActiveFootSupport(support) || !std::isfinite(support.anchor.confidence)) {
        return 0.0f;
    }
    return std::clamp(support.anchor.confidence, 0.0f, 1.0f);
}

bool IsActiveRootSupport(const SupportManifoldState& support) noexcept {
    return support.root_anchor.active && support.root_support != RootSupportType::None;
}

} // namespace bt
