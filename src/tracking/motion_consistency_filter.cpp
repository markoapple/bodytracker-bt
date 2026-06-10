#include "tracking/motion_consistency_filter.h"

#include "tracking/contact_constraints.h"
#include "tracking/support_queries.h"
#include "tracking/tracking_constants.h"

#include <algorithm>
#include <cmath>

namespace bt {
namespace {

constexpr float kRadiansToDegrees = 57.29577951308232f;
constexpr float kEpsilon = 1e-6f;

std::size_t Index(MotionTarget target) {
    return static_cast<std::size_t>(target);
}

float Clamp01(float value) {
    if (!std::isfinite(value)) {
        return 0.0f;
    }
    return std::max(0.0f, std::min(1.0f, value));
}

float Clamp(float value, float lo, float hi) {
    if (!std::isfinite(value)) {
        return lo;
    }
    return std::max(lo, std::min(hi, value));
}

float VectorLength(const Vec3f& value) {
    const float len = Length(value);
    return std::isfinite(len) ? len : 0.0f;
}

Vec3f DirectionOrZero(const Vec3f& value) {
    const float len = VectorLength(value);
    if (len < kEpsilon) {
        return {};
    }
    return Scale(value, 1.0f / len);
}

float DirectionDeviationDegrees(const Vec3f& expected, const Vec3f& measured) {
    const float expected_len = VectorLength(expected);
    const float measured_len = VectorLength(measured);
    if (expected_len < kEpsilon || measured_len < kEpsilon) {
        return 0.0f;
    }
    const float c = Clamp(Dot(expected, measured) / (expected_len * measured_len), -1.0f, 1.0f);
    return std::acos(c) * kRadiansToDegrees;
}

float LateralDeviationRatio(const Vec3f& expected, const Vec3f& measured) {
    const float expected_len = VectorLength(expected);
    const float measured_len = VectorLength(measured);
    if (expected_len < kEpsilon || measured_len < kEpsilon) {
        return 0.0f;
    }
    const Vec3f dir = Scale(expected, 1.0f / expected_len);
    const float parallel = Dot(measured, dir);
    const Vec3f lateral = Sub(measured, Scale(dir, parallel));
    return VectorLength(lateral) / std::max(measured_len, kEpsilon);
}

float SpeedChangeRatio(float expected_distance, float measured_distance) {
    const float base = std::max(expected_distance, kEpsilon);
    return std::max(measured_distance / base, base / std::max(measured_distance, kEpsilon));
}

Vec3f LimitVectorLength(const Vec3f& value, float max_length) {
    const float len = VectorLength(value);
    if (len <= max_length || len < kEpsilon) {
        return value;
    }
    return Scale(value, max_length / len);
}

float SafeDt(double dt_seconds) {
    return Clamp(static_cast<float>(dt_seconds), 1.0f / 240.0f, 0.10f);
}

float MaxSpeedForTarget(MotionTarget target, const MotionConsistencyConfig& config) {
    return target == MotionTarget::Root
        ? std::max(0.0f, config.root_max_speed_mps)
        : std::max(0.0f, config.foot_max_speed_mps);
}

float MaxAccelerationForTarget(MotionTarget target, const MotionConsistencyConfig& config) {
    return target == MotionTarget::Root
        ? std::max(0.0f, config.root_max_accel_mps2)
        : std::max(0.0f, config.foot_max_accel_mps2);
}

int RequiredConfirmFrames(float jump_m, bool absolute_limit_exceeded, const MotionConsistencyConfig& config) {
    const int base = std::max(1, config.confirm_frames);
    const int max_frames = std::max(base, config.confirm_frames_max);
    int required = base;
    if (config.confirm_scale_m > kEpsilon && jump_m > config.confirm_scale_m) {
        required = std::max(required, base + static_cast<int>(std::ceil(jump_m / config.confirm_scale_m)) - 1);
    }
    if (absolute_limit_exceeded) {
        required = std::max(required, max_frames);
    }
    return std::max(base, std::min(required, max_frames));
}

float LowPassAlpha(float dt, float cutoff_hz) {
    const float cutoff = std::max(0.001f, cutoff_hz);
    const float tau = 1.0f / (2.0f * 3.14159265358979323846f * cutoff);
    return Clamp(dt / (tau + dt), 0.0f, 1.0f);
}

Vec3f LowPassVec(const Vec3f& previous, const Vec3f& value, float alpha) {
    return Lerp(previous, value, alpha);
}

Pose3f SmoothedOutputPose(
    MotionConsistencyTargetState& state,
    const Pose3f& pose,
    float dt,
    const MotionConsistencyConfig& config) {

    if (!config.one_euro_enabled || dt <= 0.0f) {
        state.smoothed_position = pose.position;
        state.smoothed_derivative = {};
        state.smoothing_initialized = true;
        return pose;
    }

    if (!state.smoothing_initialized) {
        state.smoothed_position = pose.position;
        state.smoothed_derivative = {};
        state.smoothing_initialized = true;
        return pose;
    }

    const Vec3f raw_derivative = Scale(Sub(pose.position, state.smoothed_position), 1.0f / dt);
    const float derivative_alpha = LowPassAlpha(dt, config.one_euro_d_cutoff_hz);
    state.smoothed_derivative = LowPassVec(state.smoothed_derivative, raw_derivative, derivative_alpha);
    const float cutoff = std::max(0.001f, config.one_euro_min_cutoff_hz +
        config.one_euro_beta * VectorLength(state.smoothed_derivative));
    const float alpha = LowPassAlpha(dt, cutoff);

    Pose3f out = pose;
    out.position = LowPassVec(state.smoothed_position, pose.position, alpha);
    state.smoothed_position = out.position;
    return out;
}

void InitTelemetry(MotionTargetTelemetry& telemetry, const MotionConsistencyConfig& config, int confirm_frames) {
    telemetry = {};
    telemetry.direction_limit_deg = config.max_direction_deviation_deg;
    telemetry.lateral_limit_ratio = config.max_lateral_deviation_ratio;
    telemetry.speed_change_limit_ratio = config.max_speed_change_ratio;
    telemetry.confirm_frames = confirm_frames;
}

MotionFilterReason PrimaryLimitReason(
    float direction_deviation,
    float lateral_ratio,
    float speed_ratio,
    const MotionConsistencyConfig& config) {

    if (direction_deviation > config.max_direction_deviation_deg) {
        return MotionFilterReason::DirectionDeviation;
    }
    if (lateral_ratio > config.max_lateral_deviation_ratio) {
        return MotionFilterReason::LateralDeviation;
    }
    if (speed_ratio > config.max_speed_change_ratio) {
        return MotionFilterReason::SpeedChange;
    }
    return MotionFilterReason::None;
}

Pose3f PoseForTarget(const LowerBodyState& state, MotionTarget target) {
    switch (target) {
    case MotionTarget::Root:
        return state.root;
    case MotionTarget::LeftFoot:
        return state.left_foot;
    case MotionTarget::RightFoot:
        return state.right_foot;
    default:
        return {};
    }
}

void SetPoseForTarget(LowerBodyState& state, MotionTarget target, const Pose3f& pose) {
    switch (target) {
    case MotionTarget::Root:
        state.root = pose;
        break;
    case MotionTarget::LeftFoot:
        state.left_foot = pose;
        break;
    case MotionTarget::RightFoot:
        state.right_foot = pose;
        break;
    default:
        break;
    }
}

const FootSupportState* FootSupportForTarget(const LowerBodyState& state, MotionTarget target) {
    switch (target) {
    case MotionTarget::LeftFoot:
        return &state.support.left_foot;
    case MotionTarget::RightFoot:
        return &state.support.right_foot;
    default:
        return nullptr;
    }
}

void AcceptMeasurement(
    MotionConsistencyTargetState& state,
    const Pose3f& measured,
    float confidence,
    float dt) {

    const Pose3f previous = state.accepted_pose;
    const bool had_pose = state.initialized;
    state.previous_pose = previous;
    state.accepted_pose = measured;
    if (had_pose && dt > kEpsilon) {
        state.accepted_velocity = Scale(Sub(measured.position, previous.position), 1.0f / dt);
        state.accepted_velocity_valid = true;
    } else {
        state.accepted_velocity = {};
        state.accepted_velocity_valid = false;
    }
    state.confidence = confidence;
    state.pending_frames = 0;
    state.pending_direction = {};
    state.pending_pose = measured;
    state.stationary_accumulator = {};
    state.initialized = true;
}

Pose3f AnchoredFootPose(const Pose3f& measured, const FootSupportState& support, float foot_length_m) {
    return ApplyFootContactConstraint(measured, support, foot_length_m);
}

bool IsFootTarget(MotionTarget target) {
    return target == MotionTarget::LeftFoot || target == MotionTarget::RightFoot;
}

float FootLengthFor(const LowerBodyModel* model, MotionTarget target) {
    if (!model) {
        return tracking_constants::kDefaultFootLengthM;
    }
    if (target == MotionTarget::LeftFoot) {
        return model->left_foot_length;
    }
    if (target == MotionTarget::RightFoot) {
        return model->right_foot_length;
    }
    return tracking_constants::kDefaultFootLengthM;
}

bool RecentFreeFootMotionMatches(
    const MotionConsistencyTargetState& target,
    const Vec3f& measured_delta,
    float measured_distance,
    const MotionConsistencyConfig& config) {

    const Vec3f recent_delta = Sub(target.accepted_pose.position, target.previous_pose.position);
    const float recent_distance = VectorLength(recent_delta);
    if (recent_distance < config.min_motion_m || measured_distance < config.min_motion_m) {
        return false;
    }
    const float direction_deviation = DirectionDeviationDegrees(recent_delta, measured_delta);
    const float speed_ratio = SpeedChangeRatio(recent_distance, measured_distance);
    return direction_deviation <= config.max_direction_deviation_deg &&
        speed_ratio <= config.max_speed_change_ratio;
}

bool StrongFootPlant(const FootSupportState& support, float min_confidence) {
    return FootSupportIsFullPlant(support) && support.anchor.confidence >= min_confidence;
}

LowerBodyState ApplyCommonModeRootContactCorrection(
    const MotionConsistencyFilterState& filter,
    const LowerBodyState& measured,
    const LowerBodyState& predicted,
    const MotionConsistencyConfig& config,
    const LowerBodyModel* model,
    ContactRootCorrectionTelemetry* telemetry) {

    if (telemetry) {
        *telemetry = {};
    }

    const auto& root_target = filter.targets[Index(MotionTarget::Root)];
    if (!root_target.initialized) {
        if (telemetry) {
            telemetry->reason = MotionFilterReason::Initialized;
        }
        return measured;
    }

    const bool left_strong = StrongFootPlant(measured.support.left_foot, config.contact_root_min_support_confidence);
    const bool right_strong = StrongFootPlant(measured.support.right_foot, config.contact_root_min_support_confidence);
    if (!left_strong || !right_strong) {
        if (telemetry) {
            telemetry->reason = MotionFilterReason::ContactRootRejectedSingleFoot;
        }
        return measured;
    }

    const FootContactResidual left = FootSupportResidual(
        measured.left_foot,
        measured.support.left_foot,
        FootLengthFor(model, MotionTarget::LeftFoot));
    const FootContactResidual right = FootSupportResidual(
        measured.right_foot,
        measured.support.right_foot,
        FootLengthFor(model, MotionTarget::RightFoot));
    if (!left.valid || !right.valid) {
        if (telemetry) {
            telemetry->reason = MotionFilterReason::ContactRootRejectedSingleFoot;
        }
        return measured;
    }

    const float disagreement = VectorLength(Sub(left.residual, right.residual));
    const Vec3f common_residual = Scale(Add(left.residual, right.residual), 0.5f);
    const float common_m = VectorLength(common_residual);
    const Vec3f root_innovation = Sub(measured.root.position, predicted.root.position);
    const float root_innovation_m = VectorLength(root_innovation);
    const float alignment = root_innovation_m > kEpsilon && common_m > kEpsilon
        ? Dot(DirectionOrZero(root_innovation), DirectionOrZero(common_residual))
        : 0.0f;

    if (telemetry) {
        telemetry->left_residual = left.residual;
        telemetry->right_residual = right.residual;
        telemetry->common_residual = common_residual;
        telemetry->root_innovation = root_innovation;
        telemetry->left_residual_m = left.magnitude_m;
        telemetry->right_residual_m = right.magnitude_m;
        telemetry->common_residual_m = common_m;
        telemetry->root_innovation_m = root_innovation_m;
        telemetry->disagreement_m = disagreement;
        telemetry->root_alignment = alignment;
    }

    if (left.magnitude_m > config.contact_root_max_residual_m ||
        right.magnitude_m > config.contact_root_max_residual_m) {
        if (telemetry) {
            telemetry->reason = MotionFilterReason::ContactRootRejectedLargeResidual;
        }
        return measured;
    }
    if (disagreement > config.contact_root_max_disagreement_m) {
        if (telemetry) {
            telemetry->reason = MotionFilterReason::ContactRootRejectedDisagreement;
        }
        return measured;
    }
    if (alignment < config.contact_root_min_alignment) {
        if (telemetry) {
            telemetry->reason = MotionFilterReason::ContactRootRejectedRootMismatch;
        }
        return measured;
    }

    const Vec3f correction = LimitVectorLength(
        Scale(common_residual, -config.contact_root_correction_gain),
        config.contact_root_max_correction_m);
    LowerBodyState out = measured;
    out.root.position = Add(out.root.position, correction);
    if (telemetry) {
        telemetry->applied = VectorLength(correction) > 0.0f;
        telemetry->reason = telemetry->applied ? MotionFilterReason::ContactRootCommonMode : MotionFilterReason::None;
        telemetry->correction = correction;
        telemetry->correction_m = VectorLength(correction);
    }
    return out;
}

Pose3f BlendPose(const Pose3f& a, const Pose3f& b, float gain) {
    Pose3f out;
    out.position = Lerp(a.position, b.position, gain);
    out.orientation = Slerp(a.orientation, b.orientation, gain);
    return out;
}

MotionFilterDecision FilterTarget(
    MotionConsistencyTargetState& target,
    MotionTargetTelemetry& telemetry,
    MotionTarget target_id,
    const LowerBodyState& measured_state,
    const LowerBodyState& predicted_state,
    double dt_seconds,
    const MotionConsistencyConfig& config,
    const LowerBodyModel* model,
    Pose3f* out_pose) {

    const float dt = SafeDt(dt_seconds);
    const Pose3f measured = PoseForTarget(measured_state, target_id);
    const Pose3f predicted = PoseForTarget(predicted_state, target_id);
    const auto* support = FootSupportForTarget(measured_state, target_id);
    const float foot_length_m = FootLengthFor(model, target_id);
    InitTelemetry(telemetry, config, config.confirm_frames);
    telemetry.speed_limit_mps = MaxSpeedForTarget(target_id, config);
    telemetry.acceleration_limit_mps2 = MaxAccelerationForTarget(target_id, config);

    auto accept_pose = [&](const Pose3f& pose,
                           MotionFilterDecision decision,
                           MotionFilterReason reason) -> MotionFilterDecision {
        AcceptMeasurement(target, pose, measured_state.confidence, dt);
        *out_pose = SmoothedOutputPose(target, pose, dt, config);
        telemetry.decision = decision;
        telemetry.reason = reason;
        return telemetry.decision;
    };

    auto accept_pose_without_smoothing = [&](const Pose3f& pose,
                                             MotionFilterDecision decision,
                                             MotionFilterReason reason) -> MotionFilterDecision {
        AcceptMeasurement(target, pose, measured_state.confidence, dt);
        target.smoothed_position = pose.position;
        target.smoothed_derivative = {};
        target.smoothing_initialized = true;
        *out_pose = pose;
        telemetry.decision = decision;
        telemetry.reason = reason;
        return telemetry.decision;
    };

    if (!target.initialized) {
        if (support && FootSupportHasContactConstraint(*support)) {
            const Pose3f anchored = AnchoredFootPose(measured, *support, foot_length_m);
            InitTelemetry(telemetry, config, config.planted_foot_release_confirm_frames);
            telemetry.speed_limit_mps = MaxSpeedForTarget(target_id, config);
            telemetry.acceleration_limit_mps2 = MaxAccelerationForTarget(target_id, config);
            return accept_pose_without_smoothing(anchored, MotionFilterDecision::HeldAnchor, MotionFilterReason::AnchorHeld);
        }
        return accept_pose(measured, MotionFilterDecision::Accepted, MotionFilterReason::Initialized);
    }

    const auto fill_absolute_motion_telemetry = [&](float distance_m) {
        telemetry.measured_speed_mps = dt > kEpsilon ? distance_m / dt : 0.0f;
        if (target.accepted_velocity_valid && dt > kEpsilon) {
            const Vec3f velocity = Scale(Sub(measured.position, target.accepted_pose.position), 1.0f / dt);
            telemetry.acceleration_mps2 = VectorLength(Sub(velocity, target.accepted_velocity)) / dt;
        } else {
            telemetry.acceleration_mps2 = 0.0f;
        }
    };

    auto absolute_speed_exceeded = [&]() {
        return telemetry.speed_limit_mps > 0.0f &&
            telemetry.measured_speed_mps > telemetry.speed_limit_mps;
    };

    auto absolute_acceleration_exceeded = [&]() {
        return target.accepted_velocity_valid &&
            telemetry.acceleration_limit_mps2 > 0.0f &&
            telemetry.measured_distance_m >= config.min_motion_m &&
            telemetry.acceleration_mps2 > telemetry.acceleration_limit_mps2;
    };

    auto absolute_limit_exceeded = [&]() {
        return absolute_speed_exceeded() || absolute_acceleration_exceeded();
    };

    auto primary_absolute_reason = [&]() {
        if (absolute_speed_exceeded()) {
            return MotionFilterReason::AbsoluteSpeed;
        }
        if (absolute_acceleration_exceeded()) {
            return MotionFilterReason::AbsoluteAcceleration;
        }
        return MotionFilterReason::None;
    };

    auto update_pending = [&](const Vec3f& measured_delta) {
        const Vec3f new_direction = DirectionOrZero(measured_delta);
        if (target.pending_frames > 0 && Dot(new_direction, target.pending_direction) > 0.80f) {
            target.pending_frames += 1;
        } else {
            target.pending_frames = 1;
            target.pending_direction = new_direction;
            target.pending_pose = measured;
        }
        telemetry.pending_frames = target.pending_frames;
    };

    if (support && IsActiveFootSupport(*support)) {
        if (!FootSupportHasContactConstraint(*support)) {
            return accept_pose_without_smoothing(
                measured,
                MotionFilterDecision::Accepted,
                MotionFilterReason::AnchorReleaseConfirmed);
        }

        const FootContactResidual residual = FootSupportResidual(measured, *support, foot_length_m);
        const float anchor_distance = residual.valid ? residual.magnitude_m : Distance(measured.position, support->anchor.pose.position);
        InitTelemetry(telemetry, config, config.planted_foot_release_confirm_frames);
        telemetry.speed_limit_mps = MaxSpeedForTarget(target_id, config);
        telemetry.acceleration_limit_mps2 = MaxAccelerationForTarget(target_id, config);
        telemetry.measured_distance_m = anchor_distance;
        fill_absolute_motion_telemetry(anchor_distance);
        if (anchor_distance <= config.planted_foot_max_drift_m) {
            const Pose3f anchored = AnchoredFootPose(measured, *support, foot_length_m);
            return accept_pose_without_smoothing(anchored, MotionFilterDecision::HeldAnchor, MotionFilterReason::AnchorHeld);
        }

        const Vec3f measured_delta = Sub(measured.position, target.accepted_pose.position);
        const bool abs_exceeded = absolute_limit_exceeded();
        const int required = std::max(
            std::max(1, config.planted_foot_release_confirm_frames),
            RequiredConfirmFrames(anchor_distance, abs_exceeded, config));
        telemetry.confirm_frames = required;
        update_pending(measured_delta);

        if (target.pending_frames >= required) {
            return accept_pose(measured, MotionFilterDecision::Accepted, MotionFilterReason::AnchorReleaseConfirmed);
        }

        *out_pose = target.accepted_pose;
        telemetry.decision = MotionFilterDecision::RejectedHeld;
        const MotionFilterReason abs_reason = primary_absolute_reason();
        telemetry.reason = abs_reason == MotionFilterReason::None
            ? MotionFilterReason::AnchorReleasePending
            : abs_reason;
        return telemetry.decision;
    }

    const Vec3f measured_delta = Sub(measured.position, target.accepted_pose.position);
    const Vec3f expected_delta = Sub(predicted.position, target.accepted_pose.position);
    const float measured_distance = VectorLength(measured_delta);
    const float expected_distance = VectorLength(expected_delta);
    telemetry.measured_distance_m = measured_distance;
    telemetry.expected_distance_m = expected_distance;
    fill_absolute_motion_telemetry(measured_distance);

    if (measured_distance <= config.stationary_deadzone_m) {
        target.stationary_accumulator = Add(target.stationary_accumulator, measured_delta);
        target.confidence = Clamp01(target.confidence * std::exp(-config.reject_confidence_decay_per_second * dt));
        if (VectorLength(target.stationary_accumulator) >= config.min_motion_m) {
            const Pose3f blended = BlendPose(target.accepted_pose, measured, 0.25f);
            return accept_pose(blended, MotionFilterDecision::Blended, MotionFilterReason::StationaryAccumulated);
        }
        *out_pose = target.accepted_pose;
        telemetry.decision = MotionFilterDecision::LowMotionHeld;
        telemetry.reason = MotionFilterReason::LowMotion;
        return telemetry.decision;
    }

    target.stationary_accumulator = {};

    const bool abs_exceeded = absolute_limit_exceeded();
    if (IsFootTarget(target_id) &&
        (!support || !IsActiveFootSupport(*support)) &&
        target.accepted_velocity_valid &&
        measured_distance >= config.min_motion_m &&
        measured_distance < std::max(0.050f, 2.0f * config.min_motion_m) &&
        !absolute_speed_exceeded()) {
        telemetry.expected_distance_m = std::max(expected_distance, measured_distance);
        target.pending_frames = 0;
        return accept_pose_without_smoothing(
            measured,
            MotionFilterDecision::Accepted,
            MotionFilterReason::RecentFreeFootMotionFallback);
    }

    if (measured_distance < config.min_motion_m || expected_distance < config.min_motion_m) {
        if (IsFootTarget(target_id) &&
            (!support || !IsActiveFootSupport(*support)) &&
            measured_distance >= config.min_motion_m) {
            telemetry.expected_distance_m = std::max(expected_distance, measured_distance);
            const bool recent_motion_matches = RecentFreeFootMotionMatches(
                target,
                measured_delta,
                measured_distance,
                config);
            const float plausible_free_foot_step_m = 0.020f;
            if (!absolute_speed_exceeded() &&
                (recent_motion_matches || measured_distance <= plausible_free_foot_step_m)) {
                target.pending_frames = 0;
                return accept_pose(
                    measured,
                    MotionFilterDecision::Accepted,
                    recent_motion_matches
                        ? MotionFilterReason::RecentFreeFootMotionFallback
                        : MotionFilterReason::WithinLimits);
            }
        }

        const bool low_speed_start =
            target_id == MotionTarget::Root &&
            !absolute_speed_exceeded() &&
            (absolute_acceleration_exceeded() ||
                (target.accepted_velocity_valid && measured_distance >= config.min_motion_m) ||
                measured_distance >= 0.020f);
        if (low_speed_start &&
            measured_distance < std::max(0.050f, 2.0f * config.min_motion_m)) {
            return accept_pose_without_smoothing(
                measured,
                MotionFilterDecision::Blended,
                MotionFilterReason::LowMotion);
        }

        const bool deferred_free_foot =
            IsFootTarget(target_id) &&
            (!support || !IsActiveFootSupport(*support)) &&
            measured_distance >= config.min_motion_m;
        if (!deferred_free_foot &&
            (!abs_exceeded || low_speed_start) &&
            measured_distance < std::max(0.050f, 2.0f * config.min_motion_m)) {
            const float blend_gain = target_id == MotionTarget::Root ? 0.50f : 0.25f;
            const Pose3f blended = BlendPose(target.accepted_pose, measured, blend_gain);
            return accept_pose(blended, MotionFilterDecision::Blended, MotionFilterReason::LowMotion);
        }

        const int required = RequiredConfirmFrames(measured_distance, abs_exceeded, config);
        telemetry.confirm_frames = required;
        update_pending(measured_delta);
        if (target.pending_frames >= required) {
            return accept_pose(measured, MotionFilterDecision::Accepted, MotionFilterReason::PendingConfirmation);
        }
        *out_pose = target.accepted_pose;
        target.confidence = Clamp01(target.confidence * std::exp(-config.reject_confidence_decay_per_second * dt));
        telemetry.decision = MotionFilterDecision::Pending;
        const MotionFilterReason abs_reason = primary_absolute_reason();
        telemetry.reason = abs_reason == MotionFilterReason::None ? MotionFilterReason::PendingConfirmation : abs_reason;
        return telemetry.decision;
    }

    const float direction_deviation = DirectionDeviationDegrees(expected_delta, measured_delta);
    const float lateral_ratio = LateralDeviationRatio(expected_delta, measured_delta);
    const float speed_ratio = SpeedChangeRatio(expected_distance, measured_distance);

    telemetry.direction_deviation_deg = direction_deviation;
    telemetry.lateral_deviation_ratio = lateral_ratio;
    telemetry.speed_change_ratio = speed_ratio;

    const bool consistent =
        direction_deviation <= config.max_direction_deviation_deg &&
        lateral_ratio <= config.max_lateral_deviation_ratio &&
        speed_ratio <= config.max_speed_change_ratio &&
        (!abs_exceeded || (target_id == MotionTarget::Root && !absolute_speed_exceeded()));

    if (consistent) {
        telemetry.pending_frames = 0;
        if (target_id == MotionTarget::Root) {
            return accept_pose_without_smoothing(
                measured,
                MotionFilterDecision::Accepted,
                MotionFilterReason::WithinLimits);
        }
        return accept_pose(measured, MotionFilterDecision::Accepted, MotionFilterReason::WithinLimits);
    }

    MotionFilterReason limit_reason = primary_absolute_reason();
    if (limit_reason == MotionFilterReason::None) {
        limit_reason = PrimaryLimitReason(direction_deviation, lateral_ratio, speed_ratio, config);
    }

    const int required = RequiredConfirmFrames(measured_distance, abs_exceeded, config);
    telemetry.confirm_frames = required;
    update_pending(measured_delta);
    if (target.pending_frames >= required) {
        return accept_pose(measured, MotionFilterDecision::Accepted, MotionFilterReason::PendingConfirmation);
    }

    *out_pose = target.accepted_pose;
    target.confidence = Clamp01(target.confidence * std::exp(-config.reject_confidence_decay_per_second * dt));
    telemetry.decision = MotionFilterDecision::Pending;
    telemetry.reason = limit_reason;
    return telemetry.decision;
}
} // namespace

LowerBodyState ApplyMotionConsistencyFilter(
    MotionConsistencyFilterState& filter,
    const LowerBodyState& measured,
    const LowerBodyState& predicted,
    double dt_seconds,
    const MotionConsistencyConfig& config,
    const LowerBodyModel* model) {

    if (!config.enabled) {
        filter.telemetry = {};
        for (auto& target : filter.telemetry.targets) {
            target.decision = MotionFilterDecision::Disabled;
            target.reason = MotionFilterReason::Disabled;
        }
        return measured;
    }

    const LowerBodyState contact_adjusted = ApplyCommonModeRootContactCorrection(
        filter,
        measured,
        predicted,
        config,
        model,
        &filter.telemetry.contact_root);
    LowerBodyState out = contact_adjusted;
    const MotionTarget targets[] = {MotionTarget::Root, MotionTarget::LeftFoot, MotionTarget::RightFoot};
    for (const MotionTarget target : targets) {
        Pose3f filtered_pose;
        FilterTarget(
            filter.targets[Index(target)],
            filter.telemetry.targets[Index(target)],
            target,
            contact_adjusted,
            predicted,
            dt_seconds,
            config,
            model,
            &filtered_pose);
        SetPoseForTarget(out, target, filtered_pose);
    }
    return out;
}

} // namespace bt
