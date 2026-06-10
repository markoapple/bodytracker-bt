#pragma once

#include "calibration/body_calibrator.h"
#include "calibration/calibration_types.h"
#include "core/config.h"
#include "core/status.h"
#include "tracking/body_model.h"
#include "tracking/body_solver.h"
#include "tracking/body_state.h"
#include "tracking/room_depth_map.h"
#include "tracking/geometry_cache.h"
#include "tracking/motion_consistency_filter.h"
#include "tracking/posture_mode.h"
#include "tracking/tracker_ekf.h"
#include "tracking/tracker_synthesis.h"
#include "monocular_depth/hmd_depth_scale.h"

#include <cstdint>
#include <string>

namespace bt {

struct TrackingSolverTelemetry {
    TrackingMode tracking_mode = TrackingMode::Stereo;
    DepthSource depth_source = DepthSource::None;
    bool used_hmd = false;
    bool degraded = false;
    int objective_evaluations = 0;
    int coordinate_passes = 0;
    bool optimizer_early_stopped = false;
    double preliminary_solve_ms = 0.0;
    double final_solve_ms = 0.0;
    float preliminary_residual = 0.0f;
    float final_residual = 0.0f;
    int preliminary_weighted_observation_count = 0;
    int final_weighted_observation_count = 0;
    BodySolveStereoTelemetry preliminary_stereo{};
    BodySolveSupportConstraintTelemetry final_constraints{};
    HmdDepthScaleResult hmd_depth_scale{};
    StereoHmdAnchorResult stereo_hmd_anchor{};
    ProjectionCorrection anchor_space_mapping{};
    RoomDepthMapTelemetry room_depth_map{};
    std::string reason;
    bool camera_a_identity_swapped = false;
    bool camera_b_identity_swapped = false;
    float camera_a_identity_consistency = 0.0f;
    float camera_b_identity_consistency = 0.0f;
    bool identity_epipolar_arbitration_checked = false;
    bool identity_epipolar_arbitration_applied = false;
    int identity_epipolar_scored_lateral_pairs = 0;
    float identity_epipolar_same_score = 0.0f;
    float identity_epipolar_cross_score = 0.0f;
    float identity_epipolar_cross_geometric_uncertainty = 1.0f;
    float identity_epipolar_detection_support = 0.0f;
    float identity_epipolar_required_swap_margin = 0.0f;
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

struct TrackingStagePoseSnapshot {
    bool valid = false;
    Pose3f root{};
    Pose3f left_foot{};
    Pose3f right_foot{};
    float confidence = 0.0f;
};

struct TrackingPipelineStages {
    TrackingStagePoseSnapshot predicted{};
    TrackingStagePoseSnapshot preliminary{};
    TrackingStagePoseSnapshot support_ready{};
    TrackingStagePoseSnapshot measured{};
    TrackingStagePoseSnapshot motion_filtered{};
    TrackingStagePoseSnapshot ekf_filtered{};
    TrackingStagePoseSnapshot corrected{};
};

struct TrackingPipelineSnapshot {
    LowerBodyState state{};
    PostureClassifierState posture{};
    MotionConsistencyTelemetry motion_filter{};
    TrackerEkfTelemetry tracker_ekf{};
    TrackingSolverTelemetry solver{};
    TrackingPipelineStages stages{};
    UnifiedBodyState body_state{};
    TrackerPoseArray trackers{
        TrackerPose{TrackerRole::Pelvis},
        TrackerPose{TrackerRole::LeftFoot},
        TrackerPose{TrackerRole::RightFoot},
        TrackerPose{TrackerRole::Chest},
        TrackerPose{TrackerRole::LeftElbow},
        TrackerPose{TrackerRole::RightElbow},
        TrackerPose{TrackerRole::LeftKnee},
        TrackerPose{TrackerRole::RightKnee}
    };
    BodyCalibrationTelemetry body_calibration{};
    FloorGeometryCalibration floor_geometry{};
    std::string degradation_mode = "bootstrap";
    std::string last_error;
};

class ITrackingPipeline {
public:
    virtual ~ITrackingPipeline() = default;

    virtual void SetParams(const TrackingConfig& config) = 0;

    [[nodiscard]] virtual Result<TrackingPipelineSnapshot> Step(
        const BodySolveInputs& inputs,
        double dt_seconds) = 0;

    [[nodiscard]] virtual TrackingPipelineSnapshot Snapshot() const = 0;
};

class TrackingPipeline final : public ITrackingPipeline {
public:
    explicit TrackingPipeline(CalibrationBundle calibration);

    void SetParams(const TrackingConfig& config) override;
    [[nodiscard]] PostureMode CurrentPostureMode() const noexcept;

    Result<TrackingPipelineSnapshot> Step(
        const BodySolveInputs& inputs,
        double dt_seconds) override;

    Result<TrackingPipelineSnapshot> Step(
        const DecodedPose2D& camera_a_pose,
        const DecodedPose2D& camera_b_pose,
        const ReliabilitySummary& camera_a_reliability,
        const ReliabilitySummary& camera_b_reliability,
        const Pose3f* hmd_pose,
        double timestamp_seconds);

    Result<TrackingPipelineSnapshot> SolveFromRecordedTrackers(
        const TrackerPoseArray& trackers,
        const Pose3f* hmd_pose,
        double timestamp_seconds);

    BodyCalibrationTelemetry PersistBodyCalibrationOnShutdown();

    [[nodiscard]] TrackingPipelineSnapshot Snapshot() const override;

private:
    [[nodiscard]] double DeltaTimeFromTimestamp(double timestamp_seconds);
    CalibrationBundle calibration_;
    TrackingConfig config_{};
    LowerBodyModel model_;
    LowerBodyState state_{};
    PostureClassifierState posture_{};
    MotionConsistencyFilterState motion_filter_{};
    TrackerEkfState tracker_ekf_{};
    BodyCalibrationEstimatorState body_calibrator_{};
    BodyCalibrationTelemetry body_calibration_{};
    StereoGeometryCache stereo_geometry_cache_{};
    bool body_calibration_prev_enabled_ = false;
    TrackingPipelineSnapshot snapshot_{};
    double last_timestamp_seconds_ = 0.0;
    bool has_last_timestamp_ = false;
    bool has_last_camera_measurement_ = false;
    bool last_camera_measurement_has_a_ = false;
    bool last_camera_measurement_has_b_ = false;
    std::uint64_t last_camera_measurement_sequence_a_ = 0;
    std::uint64_t last_camera_measurement_sequence_b_ = 0;
    bool has_last_camera_measurement_timestamp_ = false;
    double last_camera_measurement_timestamp_seconds_ = 0.0;
    HmdDepthScaleHistory hmd_depth_scale_history_{};
    RoomDepthMap room_depth_map_{};
    bool stereo_hmd_anchor_has_last_ = false;
    double stereo_hmd_anchor_last_seconds_ = 0.0;
    double last_camera_measurement_timestamp_a_seconds_ = 0.0;
    double last_camera_measurement_timestamp_b_seconds_ = 0.0;
};

} // namespace bt
