#pragma once

#include "core/types.h"

namespace bt {

struct FootContactResidual {
    Vec3f residual{};
    float magnitude_m = 0.0f;
    bool valid = false;
};

[[nodiscard]] Vec3f FootHeelContactPoint(const Pose3f& foot_pose) noexcept;
[[nodiscard]] Vec3f FootToeContactPoint(const Pose3f& foot_pose) noexcept;
[[nodiscard]] Vec3f FootHeelContactPoint(const Pose3f& foot_pose, float calibrated_foot_length_m) noexcept;
[[nodiscard]] Vec3f FootToeContactPoint(const Pose3f& foot_pose, float calibrated_foot_length_m) noexcept;
[[nodiscard]] Pose3f ApplyFootContactConstraint(const Pose3f& measured, const FootSupportState& support) noexcept;
[[nodiscard]] Pose3f ApplyFootContactConstraint(const Pose3f& measured, const FootSupportState& support, float calibrated_foot_length_m) noexcept;
[[nodiscard]] FootContactResidual FootSupportResidual(const Pose3f& measured, const FootSupportState& support) noexcept;
[[nodiscard]] FootContactResidual FootSupportResidual(const Pose3f& measured, const FootSupportState& support, float calibrated_foot_length_m) noexcept;
[[nodiscard]] bool FootSupportHasContactConstraint(const FootSupportState& support) noexcept;
[[nodiscard]] bool FootSupportIsFullPlant(const FootSupportState& support) noexcept;

} // namespace bt
