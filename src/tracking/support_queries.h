#pragma once

#include "core/types.h"

namespace bt {

[[nodiscard]] bool IsActiveFootSupport(const FootSupportState& support) noexcept;
[[nodiscard]] bool IsLockedFootSupport(const FootSupportState& support) noexcept;
[[nodiscard]] float FootSupportConfidence(const FootSupportState& support) noexcept;
[[nodiscard]] bool IsActiveRootSupport(const SupportManifoldState& support) noexcept;

} // namespace bt
