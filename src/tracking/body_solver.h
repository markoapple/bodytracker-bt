#pragma once

#include "core/status.h"
#include "core/types.h"
#include "core/config.h"
#include "calibration/calibration_types.h"
#include "inference/rtmpose_decode.h"
#include "tracking/body_model.h"
#include "tracking/anchor_space_mapper.h"
#include "tracking/reliability.h"
#include "tracking/tracking_constants.h"
#include "monocular_depth/hmd_depth_scale.h"
#include "tracking/triangulation.h"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace bt {

class StereoGeometryCache;

struct BodySolveQualityConfig {
    TrackingMode tracking_mode = TrackingMode::Stereo;
    int min_triangulated_seed_count = 3;
    float max_mean_reprojection_error_px = tracking_constants::kReprojectionErrorMaxPx;
    bool use_legacy_solver = false;
    StereoJointEvidenceConfig stereo_evidence{};
    SolverObservationWeightingConfig solver_observation_weighting{};
    MonocularTrackingConfig monocular{};
};

struct BodySolveJointTriangulationTelemetry {
    bool camera_a_present = false;
    bool camera_b_present = false;
    float camera_a_confidence = 0.0f;
    float camera_b_confidence = 0.0f;
    float camera_a_weight = 0.0f;
    float camera_b_weight = 0.0f;
    float camera_a_quality = 0.0f;
    float camera_b_quality = 0.0f;
    float temporal_confidence = 0.0f;
    float epipolar_error_px = 0.0f;
    float epipolar_error_px_isotropic = 0.0f;
    float epipolar_error_px_anisotropic = 0.0f;
    float epipolar_error_normalized = 0.0f;
    float epipolar_confidence = 0.0f;
    float epipolar_reliability_term = 0.0f;
    bool epipolar_available = false;
    bool epipolar_checked = false;
    bool epipolar_hard_mismatch = false;
    bool epipolar_pair_rejected = false;
    bool epipolar_degraded_pair_softened = false;
    EpipolarCheckReason epipolar_reason = EpipolarCheckReason::InvalidGeometry;
    EpipolarCoordinateSpace epipolar_coordinate_space = EpipolarCoordinateSpace::NormalizedEssential;
    bool used_temporal_depth = false;
    bool fallback_used = false;
    JointEvidenceSource evidence_source = JointEvidenceSource::None;
    bool triangulated = false;
    bool depth_inferred = false;
    DepthSource depth_source = DepthSource::None;
    Vec3f world{};
    bool anchor_raw_world_present = false;
    Vec3f anchor_raw_world{};
    bool anchor_correction_applied = false;
    Vec3f anchor_corrected_world{};
    float anchor_corrected_depth_m = 0.0f;
    std::string anchor_correction_rejection_reason = "not_evaluated";
    float confidence = 0.0f;
    float reprojection_error_a_px = 0.0f;
    float reprojection_error_b_px = 0.0f;
    float mean_reprojection_error_px = 0.0f;
    float triangulation_condition_number = 0.0f;
    float triangulation_strength_ratio = 0.0f;
    float triangulation_null_residual = 0.0f;
    bool measurement_uncertainty_valid = false;
    float measurement_baseline_m = 0.0f;
    float measurement_mean_depth_m = 0.0f;
    float measurement_baseline_to_depth_ratio = 0.0f;
    float measurement_effective_focal_px = 0.0f;
    float measurement_reprojection_sigma_px = 0.0f;
    float measurement_epipolar_sigma_px = 0.0f;
    float measurement_image_noise_sigma_px = 0.0f;
    float measurement_conditioning_scale = 1.0f;
    float measurement_unclamped_lateral_stddev_m = 0.0f;
    float measurement_unclamped_depth_stddev_m = 0.0f;
    float measurement_unclamped_position_variance_m2 = 0.0f;
    float measurement_lateral_stddev_m = 0.0f;
    float measurement_depth_stddev_m = 0.0f;
    float measurement_position_stddev_m = 0.0f;
    float measurement_position_variance_m2 = 0.0f;
    bool solver_uncertainty_weighted = false;
    bool solver_uncertainty_valid = false;
    bool solver_uncertainty_conservative_fallback = false;
    bool solver_temporal_process_noise_applied = false;
    float solver_lateral_weight_scale = 1.0f;
    float solver_depth_weight_scale = 1.0f;
    float solver_observation_confidence_ceiling = 1.0f;
    float solver_temporal_process_stddev_m = 0.0f;
    float estimated_depth_m = 0.0f;
    float foot_contact_confidence = 0.0f;
};

struct BodySolveStereoTelemetry {
    TrackingMode tracking_mode = TrackingMode::Stereo;
    DepthSource depth_source = DepthSource::None;
    std::array<BodySolveJointTriangulationTelemetry, kHalpe26Count> joints{};
    int triangulated_count = 0;
    int left_foot_triangulated_count = 0;
    int right_foot_triangulated_count = 0;
    int inferred_depth_count = 0;
    int camera_a_present_keypoints = 0;
    int camera_b_present_keypoints = 0;
    int camera_a_usable_keypoints = 0;
    int camera_b_usable_keypoints = 0;
    float camera_a_mean_quality = 0.0f;
    float camera_b_mean_quality = 0.0f;
    float camera_a_age_scale = 1.0f;
    float camera_b_age_scale = 1.0f;
    bool epipolar_geometry_valid = false;
    std::string epipolar_status = "not_available";
    float mean_epipolar_error_px = 0.0f;
    float mean_epipolar_error_px_isotropic = 0.0f;
    float mean_epipolar_error_px_anisotropic = 0.0f;
    float mean_epipolar_error_normalized = 0.0f;
    float mean_epipolar_confidence = 0.0f;
    int epipolar_checked_count = 0;
    int epipolar_hard_mismatch_count = 0;
    int epipolar_pair_rejected_count = 0;
    int epipolar_degraded_pair_softened_count = 0;
    float mean_inferred_depth_m = 0.0f;
    float mean_confidence = 0.0f;
    float foot_mean_confidence = 0.0f;
    float mean_reprojection_error_px = 0.0f;
    float mean_triangulation_condition_number = 0.0f;
    float mean_triangulation_strength_ratio = 0.0f;
    float mean_triangulation_null_residual = 0.0f;
    float mean_measurement_position_stddev_m = 0.0f;
    float mean_measurement_depth_stddev_m = 0.0f;
    float mean_measurement_baseline_to_depth_ratio = 0.0f;
    int measurement_uncertainty_count = 0;
    int solver_uncertainty_weighted_count = 0;
    int solver_uncertainty_valid_count = 0;
    int solver_uncertainty_conservative_fallback_count = 0;
    int solver_temporal_process_noise_count = 0;
    float mean_solver_lateral_weight_scale = 0.0f;
    float mean_solver_depth_weight_scale = 0.0f;
    float mean_solver_observation_confidence_ceiling = 0.0f;
    float mean_solver_temporal_process_stddev_m = 0.0f;
    float foot_mean_reprojection_error_px = 0.0f;
    float max_foot_reprojection_error_px = 0.0f;
    float left_foot_contact_confidence = 0.0f;
    float right_foot_contact_confidence = 0.0f;
    float left_heel_contact_confidence = 0.0f;
    float left_toe_contact_confidence = 0.0f;
    float right_heel_contact_confidence = 0.0f;
    float right_toe_contact_confidence = 0.0f;
    float left_knee_floor_contact_confidence = 0.0f;
    float right_knee_floor_contact_confidence = 0.0f;
    bool left_knee_floor_contact_observed = false;
    bool right_knee_floor_contact_observed = false;
    float left_foot_low_res_separation_px = 0.0f;
    float right_foot_low_res_separation_px = 0.0f;
    MonocularScaleSource monocular_scale_source = MonocularScaleSource::None;
    float monocular_floor_assist_depth_m = 0.0f;
    float monocular_floor_assist_confidence = 0.0f;
    bool floor_geometry_used = false;
    float floor_geometry_confidence = 0.0f;
    int floor_geometry_family_count = 0;
    bool floor_distortion_correction_used = false;
    bool floor_camera_orientation_used = false;
    bool camera_a_geometry_used = false;
    bool camera_b_geometry_used = false;
    bool stereo_geometry_constraints_used = false;
    float stereo_geometry_confidence = 0.0f;
    std::string geometry_stereo_status = "not_available";
    HmdDepthScaleResult hmd_depth_scale{};
    HmdDepthScaleHistory hmd_depth_scale_history{};
    ProjectionCorrection anchor_space_mapping{};
    RoomDepthMapTelemetry room_depth_map{};
};

struct BodySolveConstraintResidualTelemetry {
    bool active = false;
    float weight = 0.0f;
    float residual_m = 0.0f;
    float score = 0.0f;
};

struct BodySolveFootConstraintTelemetry {
    float support_confidence = 0.0f;
    float transition_quality = 1.0f;
    float floor_weight_scale = 0.0f;
    float body_weight_scale = 0.0f;
    BodySolveConstraintResidualTelemetry heel_anchor{};
    BodySolveConstraintResidualTelemetry toe_anchor{};
    BodySolveConstraintResidualTelemetry full_plant{};
    BodySolveConstraintResidualTelemetry floor_penetration{};
    BodySolveConstraintResidualTelemetry sliding_velocity{};
    BodySolveConstraintResidualTelemetry orientation{};
    bool degraded_or_released = false;
};

struct BodySolveSupportConstraintTelemetry {
    float floor_calibration_weight = 0.0f;
    float leg_length_weight = 0.0f;
    float left_foot_length_weight = 0.0f;
    float right_foot_length_weight = 0.0f;
    bool body_calibration_present = false;
    float body_calibration_confidence = 0.0f;
    int body_calibration_sample_count = 0;
    bool left_reach_clamped = false;
    bool right_reach_clamped = false;
    BodySolveConstraintResidualTelemetry bone_length{};
    BodySolveConstraintResidualTelemetry root_support{};
    BodySolveConstraintResidualTelemetry left_knee_floor_anchor{};
    BodySolveConstraintResidualTelemetry right_knee_floor_anchor{};
    BodySolveFootConstraintTelemetry left_foot{};
    BodySolveFootConstraintTelemetry right_foot{};
};

enum class StereoHmdAnchorStateKind {
    Disabled,
    WaitingInterval,
    Applied,
    UnavailableHmdTrackingLost,
    UnavailableStereoHeadMissing,
    UnavailableImplausibleCorrection
};

inline const char* ToString(StereoHmdAnchorStateKind state) {
    switch (state) {
    case StereoHmdAnchorStateKind::Disabled: return "disabled";
    case StereoHmdAnchorStateKind::WaitingInterval: return "waiting_interval";
    case StereoHmdAnchorStateKind::Applied: return "applied";
    case StereoHmdAnchorStateKind::UnavailableHmdTrackingLost: return "unavailable_hmd_tracking_lost";
    case StereoHmdAnchorStateKind::UnavailableStereoHeadMissing: return "unavailable_stereo_head_missing";
    case StereoHmdAnchorStateKind::UnavailableImplausibleCorrection: return "unavailable_implausible_correction";
    default: return "unknown";
    }
}

struct StereoHmdAnchorResult {
    StereoHmdAnchorStateKind state = StereoHmdAnchorStateKind::Disabled;
    std::string reason = "disabled";
    bool applied = false;
    bool due = false;
    double interval_seconds = 5.0;
    double seconds_since_last_anchor = 0.0;
    Vec3f hmd_world{};
    Vec3f stereo_head_world{};
    Vec3f correction_world{};
    Vec3f corrected_root_world{};
    float correction_m = 0.0f;
};

struct BodySolveTelemetry {
    TrackingMode tracking_mode = TrackingMode::Stereo;
    DepthSource depth_source = DepthSource::None;
    int objective_evaluations = 0;
    int coordinate_passes = 0;
    bool optimizer_early_stopped = false;
    bool degraded = false;
    std::string degradation_reason;
    double solve_ms = 0.0;
    BodySolveStereoTelemetry stereo{};
    BodySolveSupportConstraintTelemetry constraints{};
    StereoHmdAnchorResult stereo_hmd_anchor{};
};

struct BodySolveInputs {
    DecodedPose2D camera_a_pose{};
    DecodedPose2D camera_b_pose{};
    // Model-depth companions from RTMW3D (coordinate_frame="model_simcc_body_relative").
    // NOT metric world coordinates. The solver may use these as a relative depth
    // ordering hint but must not mix them with world-space x/y/z without a named transform.
    DecodedPose3D camera_a_pose_3d{};
    DecodedPose3D camera_b_pose_3d{};
    ReliabilitySummary camera_a_reliability{};
    ReliabilitySummary camera_b_reliability{};
    HmdPoseSample hmd{};
    SteamVrAnchorFrame steamvr_anchors{};
    LowerBodyModel model{};
    CameraCalibration camera_a_calibration{};
    CameraCalibration camera_b_calibration{};
    FloorPlane floor{};
    FloorGeometryCalibration floor_geometry{};
    std::vector<WallRectangleCalibration> wall_rectangles{};
    FloorGeometryCalibration camera_a_floor_geometry{};
    FloorGeometryCalibration camera_b_floor_geometry{};
    std::vector<WallRectangleCalibration> camera_a_wall_rectangles{};
    std::vector<WallRectangleCalibration> camera_b_wall_rectangles{};
    BodyCalibration body_calibration{};
    double dt_seconds = 1.0 / 60.0;
    double camera_a_frame_age_ms = 0.0;
    double camera_b_frame_age_ms = 0.0;
    std::uint64_t camera_a_frame_sequence = 0;
    std::uint64_t camera_b_frame_sequence = 0;
    double camera_a_timestamp_seconds = 0.0;
    double camera_b_timestamp_seconds = 0.0;
    bool stereo_pair_degraded = false;
    bool stereo_pair_reused_a = false;
    bool stereo_pair_reused_b = false;
    bool stereo_pair_duplicate = false;
    bool stereo_pair_skewed = false;
    int camera_a_image_width = 0;
    int camera_a_image_height = 0;
    int camera_b_image_width = 0;
    int camera_b_image_height = 0;
    double stale_timeout_ms = 0.0;
    StereoGeometryCache* stereo_geometry_cache = nullptr;
    BodySolveQualityConfig quality{};
    HmdDepthScaleRuntimeInput hmd_depth_scale{};
    AnchorSpaceMappingRuntimeInput anchor_space_mapping{};
    double stereo_hmd_depth_scale_camera_timestamp_seconds = 0.0;
    double stereo_hmd_depth_scale_now_seconds = 0.0;
    bool stereo_hmd_anchor_enabled = false;
    bool stereo_hmd_anchor_due = false;
    double stereo_hmd_anchor_interval_seconds = 5.0;
    double stereo_hmd_anchor_seconds_since_last = 0.0;
    float stereo_hmd_anchor_max_correction_m = 1.25f;
};

struct BodySolveResult {
    LowerBodyState state{};
    float residual = 0.0f;
    int weighted_observation_count = 0;
    BodySolveTelemetry telemetry{};
    HmdDepthScaleResult hmd_depth_scale{};
    HmdDepthScaleHistory hmd_depth_scale_history{};
    StereoHmdAnchorResult stereo_hmd_anchor{};
    bool valid = false;
};

Result<BodySolveResult> RunPreliminaryBodySolve(const BodySolveInputs& inputs, const LowerBodyState& predicted);
Result<BodySolveResult> RunFinalSupportAwareSolve(const BodySolveInputs& inputs, const LowerBodyState& preliminary);

} // namespace bt
