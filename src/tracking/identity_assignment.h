#pragma once

#include "calibration/calibration_types.h"
#include "core/types.h"
#include "inference/rtmpose_decode.h"
#include "tracking/epipolar_geometry.h"
#include "tracking/stereo_runtime_config.h"

namespace bt {

struct IdentityAssignmentResult {
    DecodedPose2D pose{};
    float consistency = 1.0f;
    bool swapped = false;
};

struct StereoIdentityEpipolarContext {
    const EpipolarGeometry* geometry = nullptr;
    const CameraCalibration* camera_a = nullptr;
    const CameraCalibration* camera_b = nullptr;
    StereoIdentityEpipolarConfig config{};
    StereoTriangulationConfig triangulation{};
    StereoMeasurementUncertaintyConfig uncertainty{};
    SolverObservationWeightingConfig solver_observation_weighting{};
    bool pair_degraded = false;
    bool reused_camera_a = false;
    bool reused_camera_b = false;
    bool duplicate_pair = false;
    bool timestamp_skewed = false;
    // Age of the predicted state anchor relative to the current observations.
    // Identity prior covariance adds q*dt in variance space through the shared
    // solver process-noise model.
    float predicted_state_age_seconds = 0.0f;
    bool valid() const noexcept {
        return geometry && geometry->valid && camera_a && camera_b;
    }
    bool degraded() const noexcept {
        return pair_degraded || reused_camera_a || reused_camera_b || duplicate_pair || timestamp_skewed;
    }
};

struct StereoIdentityAssignmentResult {
    IdentityAssignmentResult camera_a{};
    IdentityAssignmentResult camera_b{};
    bool cross_camera_override_applied = false;
    bool epipolar_arbitration_checked = false;
    bool epipolar_arbitration_applied = false;
    int epipolar_scored_lateral_pairs = 0;
    float epipolar_same_identity_score = 0.0f;
    float epipolar_cross_identity_score = 0.0f;
    float epipolar_cross_geometric_uncertainty = 1.0f;
    float epipolar_detection_support = 0.0f;
    float epipolar_required_swap_margin = 0.0f;
    float identity_same_mahalanobis_sq = 0.0f;
    float identity_cross_mahalanobis_sq = 0.0f;
    float identity_same_negative_log_likelihood = 0.0f;
    float identity_cross_negative_log_likelihood = 0.0f;
    bool identity_cross_within_mahalanobis_gate = false;
    bool identity_score_gate_passed = false;
    bool identity_likelihood_gate_passed = false;
    bool identity_swap_blocked_by_strong_consistency = false;
    bool identity_swap_blocked_by_tie = false;
    int identity_uncertainty_fallback_count = 0;
};

float IdentityAnisotropicMahalanobisSq(
    const Vec3f& measured_world,
    const Vec3f& expected_world,
    const Vec3f& depth_axis_world,
    float lateral_stddev_m,
    float depth_stddev_m) noexcept;

IdentityAssignmentResult ResolveLeftRightIdentity(
    const DecodedPose2D& observed,
    const LowerBodyState& predicted_state);

float IdentityAnisotropicMahalanobisSq(
    const Vec3f& measured_world,
    const Vec3f& expected_world,
    const Vec3f& depth_axis_world,
    float lateral_stddev_m,
    float depth_stddev_m) noexcept;

IdentityAssignmentResult ResolveLeftRightIdentity(
    const DecodedPose2D& observed,
    const LowerBodyState& predicted_state,
    const Mat34f* image_from_world);

StereoIdentityAssignmentResult ResolveStereoLeftRightIdentity(
    const DecodedPose2D& observed_a,
    const DecodedPose2D& observed_b,
    const LowerBodyState& predicted_state,
    const Mat34f* image_from_world_a = nullptr,
    const Mat34f* image_from_world_b = nullptr,
    const StereoIdentityEpipolarContext* epipolar = nullptr);

} // namespace bt
