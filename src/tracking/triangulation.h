#pragma once

#include "core/math.h"
#include "core/status.h"
#include "core/types.h"
#include "tracking/epipolar_geometry.h"
#include "tracking/stereo_runtime_config.h"

#include <array>
#include <cassert>
#include <string>
#include <utility>

namespace bt {

enum class TriangulationFailure {
    None = 0,
    InvalidInput,
    ZeroWeights,
    RowNormalizationFailed,
    DltRowsNonFinite,
    DltNullspaceSolveFailed,
    DltEigensolveFailed,
    DegenerateSingularSpectrum,
    PointAtInfinity,
    NonFiniteWorldPoint,
    NonFiniteConditionDiagnostics,
    IllConditioned,
    BehindCamera,
    ProjectionFailed,
    NonFiniteReprojectionError
};

struct TriangulationStatus {
    StatusCode code = StatusCode::Ok;
    TriangulationFailure failure = TriangulationFailure::None;
    std::string message;

    [[nodiscard]] bool ok() const noexcept {
        return code == StatusCode::Ok && failure == TriangulationFailure::None;
    }

    [[nodiscard]] bool is_ill_conditioned() const noexcept {
        switch (failure) {
        case TriangulationFailure::IllConditioned:
        case TriangulationFailure::DegenerateSingularSpectrum:
        case TriangulationFailure::PointAtInfinity:
            return true;
        default:
            return false;
        }
    }

    static TriangulationStatus OK() {
        return {};
    }

    static TriangulationStatus Error(
        TriangulationFailure failure_mode,
        StatusCode c,
        std::string msg) {
        assert(failure_mode != TriangulationFailure::None);
        assert(c != StatusCode::Ok);
        if (failure_mode == TriangulationFailure::None) {
            failure_mode = TriangulationFailure::DltNullspaceSolveFailed;
        }
        if (c == StatusCode::Ok) {
            c = StatusCode::InternalError;
        }
        return TriangulationStatus{c, failure_mode, std::move(msg)};
    }
};

struct StereoMeasurementUncertainty {
    bool valid = false;
    float baseline_m = 0.0f;
    float mean_depth_m = 0.0f;
    float baseline_to_depth_ratio = 0.0f;
    float effective_focal_px = 0.0f;
    float reprojection_sigma_px = 0.0f;
    float epipolar_sigma_px = 0.0f;
    float image_noise_sigma_px = 0.0f;
    // Conditioning modulation from DLT geometry. It is applied linearly to
    // depth uncertainty and via sqrt() to lateral uncertainty because poor
    // stereo conditioning degrades the weakly observed depth axis
    // disproportionately. A full anisotropic covariance from (A^T A)^-1 is
    // deferred until solver consumption.
    float conditioning_scale = 1.0f;
    // Unclamped component stddevs are the solver-weighting contract. Reporting
    // fields below remain clamped so telemetry stays bounded, but solver
    // consumption must keep distinguishing very-bad from catastrophic geometry.
    float unclamped_lateral_stddev_m = 0.0f;
    float unclamped_depth_stddev_m = 0.0f;
    float unclamped_position_variance_m2 = 0.0f;
    float lateral_stddev_m = 0.0f;
    float depth_stddev_m = 0.0f;
    float position_stddev_m = 0.0f;
    float position_variance_m2 = 0.0f;
};

struct TriangulatedPoint {
    Vec3f world{};
    float reprojection_error_a = 0.0f;
    float reprojection_error_b = 0.0f;
    float confidence = 0.0f;
    // SVD-equivalent homogeneous DLT diagnostics. The condition number uses
    // the largest singular value over the smallest non-null singular value,
    // not the near-zero null singular value expected from a valid two-view
    // system. Higher values mean weaker triangulation geometry.
    float dlt_condition_number = 0.0f;
    float dlt_strength_ratio = 0.0f;
    float dlt_null_residual = 0.0f;
    bool triangulation_ill_conditioned = false;
    bool valid = false;
};

class TriangulationResult {
public:
    TriangulationResult(const TriangulatedPoint& value)
        : value_(value), status_(TriangulationStatus::OK()), has_value_(true) {}

    TriangulationResult(TriangulatedPoint&& value)
        : value_(std::move(value)), status_(TriangulationStatus::OK()), has_value_(true) {}

    TriangulationResult(TriangulationStatus status)
        : value_(), status_(NormalizeErrorStatus(std::move(status))), has_value_(false) {}

    [[nodiscard]] bool ok() const noexcept {
        return has_value_;
    }

    [[nodiscard]] const TriangulationStatus& status() const noexcept {
        return status_;
    }

    [[nodiscard]] const TriangulatedPoint& value() const {
        assert(has_value_);
        return value_;
    }

    [[nodiscard]] TriangulatedPoint& value() {
        assert(has_value_);
        return value_;
    }

private:
    static TriangulationStatus NormalizeErrorStatus(TriangulationStatus status) {
        assert(!status.ok());
        if (status.ok()) {
            return TriangulationStatus::Error(
                TriangulationFailure::DltNullspaceSolveFailed,
                StatusCode::InternalError,
                "TriangulationResult cannot store an OK status as an error");
        }
        return status;
    }

    TriangulatedPoint value_{};
    TriangulationStatus status_{};
    bool has_value_ = false;
};

struct StereoCameraObservation {
    Vec2f pixel{};
    float keypoint_confidence = 0.0f;
    float reliability_weight = 0.0f;
    float age_scale = 1.0f;
    bool present = false;
};

struct StereoCameraModel {
    Mat34f image_from_world{};
    Mat34f world_from_camera{};
    std::array<double, 9> camera_matrix{};
    bool projection_valid = false;
};

struct StereoTemporalReference {
    Vec3f world{};
    float confidence = 0.0f;
    bool valid = false;
};

struct StereoEpipolarContext {
    const EpipolarGeometry* geometry = nullptr;
    const CameraCalibration* camera_a = nullptr;
    const CameraCalibration* camera_b = nullptr;
    StereoEpipolarConfig config{};
    bool pair_degraded = false;
    bool reused_camera_a = false;
    bool reused_camera_b = false;
    bool duplicate_pair = false;
    bool timestamp_skewed = false;
    bool valid() const noexcept {
        return config.enabled && geometry && geometry->valid && camera_a && camera_b;
    }
};

struct StereoJointEvidence {
    Vec3f world{};
    JointEvidenceSource source = JointEvidenceSource::None;
    float confidence = 0.0f;
    float camera_a_quality = 0.0f;
    float camera_b_quality = 0.0f;
    float reprojection_error_a = 0.0f;
    float reprojection_error_b = 0.0f;
    float mean_reprojection_error = 0.0f;
    float triangulation_condition_number = 0.0f;
    float triangulation_strength_ratio = 0.0f;
    float triangulation_null_residual = 0.0f;
    bool triangulation_ill_conditioned = false;
    bool measurement_uncertainty_valid = false;
    float measurement_baseline_m = 0.0f;
    float measurement_mean_depth_m = 0.0f;
    float measurement_baseline_to_depth_ratio = 0.0f;
    float measurement_effective_focal_px = 0.0f;
    float measurement_reprojection_sigma_px = 0.0f;
    float measurement_epipolar_sigma_px = 0.0f;
    float measurement_image_noise_sigma_px = 0.0f;
    float measurement_conditioning_scale = 1.0f;
    // Unclamped stddevs are intentionally not reporting fields: they are for
    // solver weighting so catastrophic observations do not collapse onto the
    // telemetry clamp.
    float measurement_unclamped_lateral_stddev_m = 0.0f;
    float measurement_unclamped_depth_stddev_m = 0.0f;
    float measurement_lateral_stddev_m = 0.0f;
    float measurement_depth_stddev_m = 0.0f;
    float measurement_position_stddev_m = 0.0f;
    float measurement_position_variance_m2 = 0.0f;
    float estimated_depth_m = 0.0f;
    float temporal_confidence = 0.0f;
    // Backward-compatible runtime pixel error used for current epipolar thresholds.
    // For distortion-safe normalized checks this is the isotropic average-focal heuristic.
    float epipolar_error_px = 0.0f;
    float epipolar_error_px_isotropic = 0.0f;
    float epipolar_error_px_anisotropic = 0.0f;
    float epipolar_error_normalized = 0.0f;
    float epipolar_confidence = 0.0f;
    // Pairwise reliability term derived from epipolar geometry. Zero means
    // the term was unavailable/uncomputed; callers must not treat zero as a
    // confidence value unless epipolar_checked is true.
    float epipolar_reliability_term = 0.0f;
    bool epipolar_available = false;
    bool epipolar_checked = false;
    bool epipolar_hard_mismatch = false;
    bool epipolar_pair_rejected = false;
    bool epipolar_degraded_pair_softened = false;
    EpipolarCheckReason epipolar_reason = EpipolarCheckReason::InvalidGeometry;
    EpipolarCoordinateSpace epipolar_coordinate_space = EpipolarCoordinateSpace::NormalizedEssential;
    bool triangulated = false;
    bool depth_inferred = false;
    bool temporal_depth_used = false;
    bool fallback_used = false;
    bool rejected = false;
    bool valid = false;
};

float StereoReprojectionConfidence(float reprojection_error_a_px, float reprojection_error_b_px) noexcept;
float CameraObservationQuality(const StereoCameraObservation& observation) noexcept;
float ProjectionDepth(const Mat34f& projection, const Vec3f& world) noexcept;
StereoMeasurementUncertainty EstimateStereoMeasurementUncertainty(
    const StereoCameraModel& camera_a,
    const StereoCameraModel& camera_b,
    const TriangulatedPoint& triangulated,
    float epipolar_error_px,
    bool epipolar_checked,
    const StereoMeasurementUncertaintyConfig& config = StereoMeasurementUncertaintyConfig{}) noexcept;
Result<Vec3f> BackProjectPixelAtCameraDepth(
    const StereoCameraModel& camera,
    Vec2f pixel,
    float camera_depth_m);
StereoJointEvidence ResolveStereoJointEvidence(
    const StereoCameraModel& camera_a,
    const StereoCameraModel& camera_b,
    const StereoCameraObservation& observation_a,
    const StereoCameraObservation& observation_b,
    const StereoTemporalReference& temporal,
    const StereoEpipolarContext* epipolar = nullptr,
    const StereoJointEvidenceConfig& config = StereoJointEvidenceConfig{});

// Production consumption hook for pairwise epipolar reliability. This is kept
// next to StereoJointEvidence so tests can prove the term changes real seed
// confidence instead of stopping at telemetry/logging. Returns 1.0 when the
// epipolar term is unavailable or not applicable.
float StereoPairEpipolarReliabilityScale(const StereoJointEvidence& evidence) noexcept;
float StereoSeedConfidence(
    const StereoJointEvidence& evidence,
    float foot_separation_scale = 1.0f,
    float geometry_scale = 1.0f) noexcept;

TriangulationResult TriangulateLinearDLT(
    const Mat34f& proj_a,
    const Mat34f& proj_b,
    Vec2f point_a,
    Vec2f point_b,
    float weight_a = 1.0f,
    float weight_b = 1.0f,
    const StereoTriangulationConfig& config = StereoTriangulationConfig{});


} // namespace bt
