#pragma once

namespace bt {

// Runtime knobs for stereo epipolar consistency. These remain separate from
// detection confidence: they describe geometric compatibility thresholds.
struct StereoEpipolarConfig {
    bool enabled = true;
    float soft_threshold_px = 2.5f;
    float hard_threshold_px = 18.0f;
    float min_confidence_floor = 0.10f;
    bool hard_mismatch_rejects_fresh_pair = true;
    bool hard_mismatch_rejects_degraded_pair = false;
};

// Runtime knobs for the triangulation acceptance path. These decide whether a
// geometric observation is structurally usable before uncertainty weighting.
struct StereoTriangulationConfig {
    float min_single_camera_quality = 0.06f;
    float min_stereo_reprojection_confidence = 0.055f;
    float single_camera_depth_confidence_scale = 0.58f;
    float max_dlt_condition_number = 1000.0f;
    float min_dlt_strength_ratio = 1.0e-3f;
    float max_ray_closest_distance_m = 0.18f;
    float min_ray_angle_deg = 0.20f;
};

// Runtime knobs for converting residual evidence and geometry diagnostics into
// stereo measurement uncertainty. The solver consumes the unclamped values;
// clamped values are for bounded telemetry/reporting.
struct StereoMeasurementUncertaintyConfig {
    float min_image_noise_sigma_px = 0.35f;
    float min_lateral_stddev_m = 0.002f;
    float min_depth_stddev_m = 0.005f;
    float max_reported_position_stddev_m = 100.0f;
    float max_conditioning_scale = 10.0f;
};

struct StereoJointEvidenceConfig {
    StereoEpipolarConfig epipolar{};
    StereoTriangulationConfig triangulation{};
    StereoMeasurementUncertaintyConfig uncertainty{};
};

// Solver observation weighting knobs. Confidence, uncertainty, temporal process
// noise, and final weight scale stay separate so missing uncertainty never means
// perfect certainty.
struct SolverObservationWeightingConfig {
    float reference_stddev_m = 0.050f;
    float min_stddev_for_weight_m = 0.015f;
    float max_weight_scale = 10.0f;
    float fallback_stddev_m = 1.0f;
    float temporal_process_variance_m2_per_s = 0.1225f;
    float max_temporal_process_stddev_m = 1.0f;
};

// Identity epipolar arbitration policy. Detection support gates whether an
// arbitration is allowed; uncertainty/Mahalanobis terms choose which assignment
// is geometrically better.
struct StereoIdentityEpipolarConfig {
    float soft_threshold_px = 2.5f;
    float hard_threshold_px = 18.0f;
    float swap_ratio_margin = 1.35f;
    float swap_absolute_margin = 0.18f;
    float uncertainty_swap_margin_scale = 0.20f;
    float partial_coverage_swap_margin = 0.10f;
    float identity_prior_lateral_stddev_m = 0.35f;
    float identity_prior_depth_stddev_m = 2.50f;
    // Cauchy-style ranking scale: score = 1 / (1 + mahalanobis_sq / scale).
    float identity_mahalanobis_score_scale = 25.0f;
    // Hard gate in squared Mahalanobis units.
    float identity_max_mahalanobis_sq = 25.0f;
    // Comparative swap margin in NLL units, where NLL = d^2 + log(det Sigma).
    float identity_swap_nll_margin = 9.0f;
    float strong_consistency_guard = 0.55f;
    float min_assignment_score = 0.45f;
    float min_detection_support = 0.15f;
    int min_scored_lateral_pairs = 2;
};

} // namespace bt
