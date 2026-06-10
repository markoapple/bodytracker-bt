#pragma once

#include "core/config.h"
#include "core/types.h"
#include "tracking/body_model.h"

#include <array>
#include <cstddef>
#include <string>

namespace bt {

enum class MotionTarget : std::size_t {
    Root = 0,
    LeftFoot,
    RightFoot,
    Count
};

enum class MotionFilterDecision {
    Accepted = 0,
    Blended,
    Pending,
    RejectedHeld,
    HeldAnchor,
    LowMotionHeld,
    Disabled,
    Count
};

enum class MotionFilterReason {
    None = 0,
    Disabled,
    Initialized,
    WithinLimits,
    LowMotion,
    StationaryAccumulated,
    AnchorHeld,
    AnchorReleasePending,
    AnchorReleaseConfirmed,
    DirectionDeviation,
    LateralDeviation,
    SpeedChange,
    AbsoluteSpeed,
    AbsoluteAcceleration,
    PendingConfirmation,
    RecentFreeFootMotionFallback,
    ContactRootCommonMode,
    ContactRootRejectedSingleFoot,
    ContactRootRejectedDisagreement,
    ContactRootRejectedRootMismatch,
    ContactRootRejectedLargeResidual,
    Count
};

struct MotionTargetTelemetry {
    MotionFilterDecision decision = MotionFilterDecision::Disabled;
    MotionFilterReason reason = MotionFilterReason::Disabled;
    float measured_distance_m = 0.0f;
    float expected_distance_m = 0.0f;
    float direction_deviation_deg = 0.0f;
    float lateral_deviation_ratio = 0.0f;
    float speed_change_ratio = 0.0f;
    float direction_limit_deg = 0.0f;
    float lateral_limit_ratio = 0.0f;
    float speed_change_limit_ratio = 0.0f;
    float measured_speed_mps = 0.0f;
    float acceleration_mps2 = 0.0f;
    float speed_limit_mps = 0.0f;
    float acceleration_limit_mps2 = 0.0f;
    int pending_frames = 0;
    int confirm_frames = 0;
};

struct ContactRootCorrectionTelemetry {
    bool applied = false;
    MotionFilterReason reason = MotionFilterReason::None;
    Vec3f correction{};
    Vec3f left_residual{};
    Vec3f right_residual{};
    Vec3f common_residual{};
    Vec3f root_innovation{};
    float correction_m = 0.0f;
    float left_residual_m = 0.0f;
    float right_residual_m = 0.0f;
    float common_residual_m = 0.0f;
    float root_innovation_m = 0.0f;
    float disagreement_m = 0.0f;
    float root_alignment = 0.0f;
};

struct MotionConsistencyTelemetry {
    std::array<MotionTargetTelemetry, static_cast<std::size_t>(MotionTarget::Count)> targets{};
    ContactRootCorrectionTelemetry contact_root{};
};

struct MotionConsistencyTargetState {
    Pose3f accepted_pose{};
    Pose3f previous_pose{};
    Pose3f pending_pose{};
    Vec3f pending_direction{};
    int pending_frames = 0;
    float confidence = 0.0f;
    Vec3f stationary_accumulator{};
    Vec3f accepted_velocity{};
    Vec3f smoothed_position{};
    Vec3f smoothed_derivative{};
    bool accepted_velocity_valid = false;
    bool smoothing_initialized = false;
    bool initialized = false;
};

struct MotionConsistencyFilterState {
    std::array<MotionConsistencyTargetState, static_cast<std::size_t>(MotionTarget::Count)> targets{};
    MotionConsistencyTelemetry telemetry{};
};

inline constexpr std::array<const char*, static_cast<std::size_t>(MotionFilterDecision::Count)>
kMotionFilterDecisionNames = {
    "ACCEPTED",
    "BLENDED",
    "PENDING",
    "REJECTED_HELD",
    "HELD_ANCHOR",
    "LOW_MOTION_HELD",
    "DISABLED"
};

inline constexpr std::array<const char*, static_cast<std::size_t>(MotionFilterReason::Count)>
kMotionFilterReasonNames = {
    "NONE",
    "DISABLED",
    "INITIALIZED",
    "WITHIN_LIMITS",
    "LOW_MOTION",
    "STATIONARY_ACCUMULATED",
    "ANCHOR_HELD",
    "ANCHOR_RELEASE_PENDING",
    "ANCHOR_RELEASE_CONFIRMED",
    "DIRECTION_DEVIATION",
    "LATERAL_DEVIATION",
    "SPEED_CHANGE",
    "ABSOLUTE_SPEED",
    "ABSOLUTE_ACCELERATION",
    "PENDING_CONFIRMATION",
    "RECENT_FREE_FOOT_MOTION_FALLBACK",
    "CONTACT_ROOT_COMMON_MODE",
    "CONTACT_ROOT_REJECTED_SINGLE_FOOT",
    "CONTACT_ROOT_REJECTED_DISAGREEMENT",
    "CONTACT_ROOT_REJECTED_ROOT_MISMATCH",
    "CONTACT_ROOT_REJECTED_LARGE_RESIDUAL"
};

inline const char* ToString(MotionFilterDecision decision) {
    const auto idx = static_cast<std::size_t>(decision);
    return idx < kMotionFilterDecisionNames.size() ? kMotionFilterDecisionNames[idx] : "UNKNOWN";
}

inline const char* ToString(MotionFilterReason reason) {
    const auto idx = static_cast<std::size_t>(reason);
    return idx < kMotionFilterReasonNames.size() ? kMotionFilterReasonNames[idx] : "UNKNOWN";
}

LowerBodyState ApplyMotionConsistencyFilter(
    MotionConsistencyFilterState& filter,
    const LowerBodyState& measured,
    const LowerBodyState& predicted,
    double dt_seconds,
    const MotionConsistencyConfig& config,
    const LowerBodyModel* model = nullptr);

} // namespace bt
