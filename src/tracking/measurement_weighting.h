#pragma once

#include "core/math.h"
#include "tracking/stereo_runtime_config.h"

#include <algorithm>
#include <cmath>

namespace bt {

struct SolverMeasurementUncertainty {
    bool valid = false;
    // Unit ray direction in world coordinates for this observation's weakly
    // constrained depth/range axis. This is not a world-Z split; callers should
    // derive it from the camera origin(s) to the measured 3D point.
    Vec3f depth_axis_world{0.0f, 0.0f, 1.0f};
    float lateral_stddev_m = 0.0f;
    float depth_stddev_m = 0.0f;
    // Process noise is a separate temporal uncertainty term, not a confidence
    // penalty. It is added in variance so stale/reused observations become less
    // precise without collapsing occlusion/blur confidence into geometry.
    float temporal_process_stddev_m = 0.0f;
};

struct SolverObservationInformation {
    bool uncertainty_valid = false;
    bool conservative_fallback = true;
    bool temporal_process_noise_applied = false;
    Vec3f depth_axis_world{0.0f, 0.0f, 1.0f};
    float lateral_weight_scale = 1.0f;
    float depth_weight_scale = 1.0f;
    float temporal_process_stddev_m = 0.0f;
};

// Weight scales are relative to config.reference_stddev_m:
//   scale = reference_variance / observation_variance.
// Very small variances are regularized and capped so near-floor observations
// cannot dominate the solve numerically.

inline float SolverTemporalProcessStddevM(
    float age_seconds,
    const SolverObservationWeightingConfig& config = SolverObservationWeightingConfig{}) {

    if (!std::isfinite(age_seconds) || age_seconds <= 0.0f ||
        !std::isfinite(config.temporal_process_variance_m2_per_s) ||
        config.temporal_process_variance_m2_per_s <= 0.0f) {
        return 0.0f;
    }
    const float variance = config.temporal_process_variance_m2_per_s * age_seconds;
    if (!std::isfinite(variance) || variance <= 0.0f) {
        return 0.0f;
    }
    const float cap = (std::isfinite(config.max_temporal_process_stddev_m) &&
        config.max_temporal_process_stddev_m > 0.0f)
        ? config.max_temporal_process_stddev_m
        : SolverObservationWeightingConfig{}.max_temporal_process_stddev_m;
    return std::min(cap, std::sqrt(variance));
}

inline float SolverSafeTemporalProcessStddevM(
    float process_stddev_m,
    const SolverObservationWeightingConfig& config = SolverObservationWeightingConfig{}) {

    if (!std::isfinite(process_stddev_m) || process_stddev_m <= 0.0f) {
        return 0.0f;
    }
    const float cap = (std::isfinite(config.max_temporal_process_stddev_m) &&
        config.max_temporal_process_stddev_m > 0.0f)
        ? config.max_temporal_process_stddev_m
        : SolverObservationWeightingConfig{}.max_temporal_process_stddev_m;
    return std::min(cap, process_stddev_m);
}

inline float SolverStddevWithProcessNoise(
    float measurement_stddev_m,
    float process_stddev_m,
    const SolverObservationWeightingConfig& config = SolverObservationWeightingConfig{}) {

    const float fallback_stddev = (std::isfinite(config.fallback_stddev_m) && config.fallback_stddev_m > 0.0f)
        ? config.fallback_stddev_m
        : SolverObservationWeightingConfig{}.fallback_stddev_m;
    const float safe_measurement = (std::isfinite(measurement_stddev_m) && measurement_stddev_m > 0.0f)
        ? measurement_stddev_m
        : fallback_stddev;
    const float safe_process = SolverSafeTemporalProcessStddevM(process_stddev_m, config);
    return std::sqrt(safe_measurement * safe_measurement + safe_process * safe_process);
}

inline float SolverAxisWeightScaleFromStddev(
    float stddev_m,
    const SolverObservationWeightingConfig& config = SolverObservationWeightingConfig{}) {

    const float fallback_stddev = (std::isfinite(config.fallback_stddev_m) && config.fallback_stddev_m > 0.0f)
        ? config.fallback_stddev_m
        : SolverObservationWeightingConfig{}.fallback_stddev_m;
    const float safe_stddev = (std::isfinite(stddev_m) && stddev_m > 0.0f)
        ? stddev_m
        : fallback_stddev;
    const float min_stddev = (std::isfinite(config.min_stddev_for_weight_m) && config.min_stddev_for_weight_m > 0.0f)
        ? config.min_stddev_for_weight_m
        : SolverObservationWeightingConfig{}.min_stddev_for_weight_m;
    const float regularized_stddev = std::max(safe_stddev, min_stddev);
    const float reference_stddev = (std::isfinite(config.reference_stddev_m) && config.reference_stddev_m > 0.0f)
        ? config.reference_stddev_m
        : SolverObservationWeightingConfig{}.reference_stddev_m;
    const float reference_variance = reference_stddev * reference_stddev;
    const float variance = regularized_stddev * regularized_stddev;
    const float max_scale = (std::isfinite(config.max_weight_scale) && config.max_weight_scale > 0.0f)
        ? config.max_weight_scale
        : SolverObservationWeightingConfig{}.max_weight_scale;
    return std::min(max_scale, reference_variance / variance);
}

inline Vec3f SolverObservationDepthAxisFromOrigin(
    const Vec3f& camera_origin_world,
    const Vec3f& measured_world,
    const Vec3f& fallback_axis_world = Vec3f{0.0f, 0.0f, 1.0f}) {

    // Prefer the actual bearing/range direction from the camera origin to the
    // measured 3D point. The projection/optical axis is only a degenerate
    // fallback for the physically invalid case where the point is at the camera
    // origin; it is never blended with the bearing ray.
    const Vec3f fallback_unit = NormalizeOr(fallback_axis_world, Vec3f{0.0f, 0.0f, 1.0f});
    return NormalizeOr(Sub(measured_world, camera_origin_world), fallback_unit);
}

inline SolverObservationInformation SolverObservationInformationFromUncertainty(
    const SolverMeasurementUncertainty& uncertainty,
    const SolverObservationWeightingConfig& config = SolverObservationWeightingConfig{}) {

    SolverObservationInformation out;
    out.depth_axis_world = NormalizeOr(uncertainty.depth_axis_world, Vec3f{0.0f, 0.0f, 1.0f});
    out.temporal_process_stddev_m = SolverSafeTemporalProcessStddevM(uncertainty.temporal_process_stddev_m, config);
    out.temporal_process_noise_applied = out.temporal_process_stddev_m > 0.0f;
    if (!uncertainty.valid ||
        !std::isfinite(uncertainty.lateral_stddev_m) ||
        !std::isfinite(uncertainty.depth_stddev_m) ||
        uncertainty.lateral_stddev_m <= 0.0f ||
        uncertainty.depth_stddev_m <= 0.0f) {
        const float fallback_stddev = SolverStddevWithProcessNoise(
            config.fallback_stddev_m,
            out.temporal_process_stddev_m,
            config);
        out.lateral_weight_scale = SolverAxisWeightScaleFromStddev(fallback_stddev, config);
        out.depth_weight_scale = SolverAxisWeightScaleFromStddev(fallback_stddev, config);
        return out;
    }

    out.uncertainty_valid = true;
    out.conservative_fallback = false;
    out.lateral_weight_scale = SolverAxisWeightScaleFromStddev(
        SolverStddevWithProcessNoise(uncertainty.lateral_stddev_m, out.temporal_process_stddev_m, config),
        config);
    out.depth_weight_scale = SolverAxisWeightScaleFromStddev(
        SolverStddevWithProcessNoise(uncertainty.depth_stddev_m, out.temporal_process_stddev_m, config),
        config);
    return out;
}

inline float SolverAnisotropicSquaredResidual(
    const Vec3f& candidate_world,
    const Vec3f& measured_world,
    const SolverObservationInformation& information) {

    const Vec3f delta = Sub(candidate_world, measured_world);
    const float depth_delta = Dot(delta, information.depth_axis_world);
    const float total_sq = Dot(delta, delta);
    const float lateral_sq = std::max(0.0f, total_sq - depth_delta * depth_delta);
    return information.lateral_weight_scale * lateral_sq +
        information.depth_weight_scale * depth_delta * depth_delta;
}

// Solver weight scale is an inverse-variance quantity, not a confidence. This
// policy converts the weakest axis scale into a bounded confidence ceiling so
// downstream telemetry can expose both the raw solver weight and its output
// confidence effect without collapsing uncertainty into detection confidence.
inline float SolverObservationConfidenceCeilingFromWeightScale(float weakest_weight_scale) {
    const float scale = std::isfinite(weakest_weight_scale) ? weakest_weight_scale : 0.0f;
    if (scale >= 1.0f) {
        return 1.0f;
    }
    if (scale >= 0.25f) {
        return 0.85f;
    }
    if (scale >= 0.05f) {
        return 0.70f;
    }
    return 0.55f;
}

inline float SolverObservationConfidenceCeiling(const SolverObservationInformation& information) {
    return SolverObservationConfidenceCeilingFromWeightScale(
        std::min(information.lateral_weight_scale, information.depth_weight_scale));
}

} // namespace bt
