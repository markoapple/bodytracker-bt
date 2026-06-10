#pragma once

#include "core/status.h"
#include "core/math.h"
#include "core/types.h"
#include "tracking/tracking_constants.h"
#include "tracking/stereo_runtime_config.h"

#include <array>
#include <filesystem>
#include <string>
#include <vector>

namespace bt {

inline constexpr std::size_t kOscTrackerRoleCount = 8;

struct AppSectionConfig {
    std::filesystem::path log_file = "bodytracker.log";
    std::filesystem::path recording_dir = "recordings";
};

struct DebugConfig {
    std::filesystem::path replay_log_path{};
};

struct InferenceConfig {
    std::filesystem::path model_path = "models/rtmw-dw-x-l-cocktail14-384x288.onnx";
    std::string device = "directml";
};

struct MotionConsistencyConfig {
    bool enabled = true;
    int confirm_frames = 2;
    float min_motion_m = 0.015f;
    float stationary_deadzone_m = 0.006f;
    float max_direction_deviation_deg = 45.0f;
    float max_lateral_deviation_ratio = 0.65f;
    float max_speed_change_ratio = 2.50f;
    float reject_confidence_decay_per_second = 1.20f;
    float planted_foot_max_drift_m = 0.050f;
    int planted_foot_release_confirm_frames = 2;
    float contact_root_correction_gain = 0.20f;
    float contact_root_max_correction_m = 0.015f;
    float contact_root_max_residual_m = 0.035f;
    float contact_root_max_disagreement_m = 0.012f;
    float contact_root_min_alignment = 0.75f;
    float contact_root_min_support_confidence = 0.75f;

    // Biomechanical absolute caps. Per-measurement jumps that exceed these velocities
    // or accelerations are rejected/held, regardless of whether they happen to
    // be "directionally consistent" with the prediction. Values reflect
    // generous human envelopes (sprint kick / fall) and are deliberately
    // permissive — anything beyond them is implausible motion.
    float root_max_speed_mps = 3.5f;
    float foot_max_speed_mps = 7.0f;
    float root_max_accel_mps2 = 22.0f;
    float foot_max_accel_mps2 = 60.0f;

    // Magnitude-aware confirmation: confirm_frames is the minimum count of
    // fresh camera-measurement updates; large jumps require additional distinct
    // updates of consistent direction before acceptance.
    // confirm_frames_at = floor(confirm_frames + jump_m / scale_m), clamped
    // at confirm_frames_max.
    float confirm_scale_m = 0.04f;
    int confirm_frames_max = 6;

    // One-Euro post-smoothing on the accepted output. Reduces visible jitter
    // without adding lag for fast motion.
    bool one_euro_enabled = true;
    float one_euro_min_cutoff_hz = 1.2f;
    float one_euro_beta = 0.018f;
    float one_euro_d_cutoff_hz = 1.0f;
};

struct TrackerEkfConfig {
    bool enabled = true;
    float process_noise_mps2 = 8.0f;
    float min_measurement_variance_m2 = 0.000025f;
    float max_measurement_variance_m2 = 0.0100f;
    float support_variance_scale = 0.25f;
    float missing_velocity_decay = 0.92f;
    float foot_orientation_gain = 0.35f;

    // Mahalanobis innovation gate. yᵀ S⁻¹ y is computed per axis (combined as
    // a 3-DOF chi-square) before the update. mahalanobis_gate is the chi²
    // threshold above which the measurement is treated as an outlier; when
    // set, the measurement variance is inflated by `outlier_variance_scale`
    // for that frame (Huber-style robust update) instead of being applied raw.
    bool mahalanobis_gate_enabled = true;
    float mahalanobis_gate_chi2 = 16.27f; // 3-DOF, p ≈ 0.001
    float outlier_variance_scale = 64.0f; // Huber-style R inflation
};

struct TemporalUpdateConfig {
    float free_gain = 0.75f;
    float supported_gain = 0.20f;
    float foot_free_gain = 0.55f;
    float foot_supported_gain = 0.12f;
};

struct BodyCalibrationModeConfig {
    bool enabled = false;
    bool auto_persist = true;
    float required_seconds = 2.5f;
    float min_overall_confidence = tracking_constants::kIdentityStableThreshold;
    float max_segment_cv = 0.12f;
};

struct MonocularTrackingConfig {
    int image_width = 1280;
    int image_height = 720;
    float horizontal_fov_deg = 70.0f;
    float user_height_m = 1.70f;
    float camera_height_m = 1.20f;
    float default_depth_m = 2.20f;
    float depth_confidence_scale = 0.55f;
    float min_keypoint_confidence = tracking_constants::kReliabilityMinPresentConfidence;
    int min_seed_count = 4;
    bool floor_scale_assist_enabled = false;
    bool floor_geometry_calibration_enabled = false;
    std::string floor_geometry_type = "unknown";
    float floor_depth_line_spacing_m = 0.0f;
    float floor_depth_line_spacing_px = 0.0f;
    float floor_depth_reference_y_px = 0.0f;
    float floor_depth_reference_m = 0.0f;
    float floor_depth_confidence = 0.65f;
    float floor_second_axis_spacing_m = 0.0f;
    float floor_geometry_confidence = 0.0f;
    bool floor_projective_homography_enabled = false;
    std::array<float, 9> floor_from_image{1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    std::array<float, 9> image_from_floor{1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    float floor_projective_confidence = 0.0f;
    bool floor_distortion_correction_enabled = false;
    float floor_distortion_confidence = 0.0f;
    float floor_radial_k1 = 0.0f;
    float floor_radial_k2 = 0.0f;
    float floor_tangential_p1 = 0.0f;
    float floor_tangential_p2 = 0.0f;
    bool floor_camera_orientation_enabled = false;
    float floor_camera_pitch_rad = 0.0f;
    float floor_camera_roll_rad = 0.0f;
    float floor_camera_orientation_confidence = 0.0f;
    bool wall_depth_assist_enabled = false;
    float wall_depth_assist_m = 0.0f;
    float wall_depth_assist_confidence = 0.0f;
};

enum class HmdDepthScaleStateKind {
    Live,
    HeldHeadMissing,
    HeldHeadOutlier,
    HeldHmdTrackingLost,
    HeldImplausibleScale,
    UnavailableHmdTrackingLost,
    UnavailableCameraExtrinsics,
    UnavailableNoPreviousScale,
    Disabled
};

struct HmdDepthScaleConfig {
    bool enabled = false;
    float min_depth_m = tracking_constants::kMonocularMinDepthM;
    float max_depth_m = tracking_constants::kMonocularMaxDepthM;
    float min_scale = 0.33f;
    float max_scale = 3.0f;
    double max_hold_seconds = 0.25;
    float outlier_sigma = 4.0f;
    int history_size = 15;
};

struct StereoHmdAnchorConfig {
    bool enabled = false;
    double interval_seconds = 5.0;
    float max_correction_m = 1.25f;
};

struct AnchorSpaceMappingConfig {
    bool enabled = true;
    bool use_hmd = true;
    bool use_controllers = true;
    bool allow_hmd_only_scale_fallback = true;
    int min_anchors_for_pose_refine = 3;
    float max_reprojection_error_px = 20.0f;
    float target_reprojection_error_px = 8.0f;
    float min_depth_scale = 0.75f;
    float max_depth_scale = 1.35f;
    float room_map_min_update_depth_scale = 0.75f;
    float room_map_max_update_depth_scale = 1.35f;
    double timestamp_alignment_seconds = 0.05;
    double log_interval_seconds = 1.0;
    Vec3f hmd_to_head_keypoint_offset_m{0.0f, 0.0f, 0.0f};
    Vec3f left_controller_to_wrist_offset_m{0.0f, 0.0f, 0.0f};
    Vec3f right_controller_to_wrist_offset_m{0.0f, 0.0f, 0.0f};
};

struct RoomDepthMapConfig {
    bool enabled = true;
    bool collect_only = true;
    int resolution_width = 320;
    int resolution_height = 180;
    int min_samples_per_cell = 8;
    float max_cell_variance_m2 = 0.01f;
    int min_accepted_frames_before_active = 1000;
    int body_mask_dilation_px = 24;
    bool update_only_when_anchor_quality_good = true;
    std::filesystem::path save_path = "data/room_depth_map.bin";
    bool load_existing = true;
    double save_interval_seconds = 30.0;
};

struct StereoAnchorDepthCorrectionConfig {
    bool enabled = true;
    bool apply_per_frame = true;
    float min_scale = 0.90f;
    float max_scale = 1.10f;
    bool camera_space_depth_only = true;
};

struct TrackingConfig {
    TrackingMode mode = TrackingMode::Stereo;
    std::filesystem::path model_path = "models/rtmw-dw-x-l-cocktail14-384x288.onnx";
    bool depth_postprocess_enabled = false;
    std::filesystem::path depth_postprocess_model_path = "models/rtmw3d-x-cocktail14-384x288.onnx";
    int depth_postprocess_interval_frames = 4;
    bool depth_postprocess_allow_cpu_fallback = false;
    std::filesystem::path calibration_path = "calib/default.json";
    double latest_frame_skew_tolerance_ms = 18.0;
    double stale_frame_timeout_ms = 250.0;
    double max_frame_skew_ms = 18.0;
    int min_triangulated_seed_count = 3;
    double max_mean_reprojection_error_px = tracking_constants::kReprojectionErrorMaxPx;
    bool stereo_monocular_fallback_enabled = false;
    bool use_legacy_solver = false;
    bool enable_replay_recording = false;
    StereoEpipolarConfig stereo_epipolar{};
    StereoTriangulationConfig stereo_triangulation{};
    StereoMeasurementUncertaintyConfig stereo_uncertainty{};
    SolverObservationWeightingConfig solver_observation_weighting{};
    StereoIdentityEpipolarConfig stereo_identity{};
    MotionConsistencyConfig motion_consistency{};
    TrackerEkfConfig tracker_ekf{};
    TemporalUpdateConfig temporal_update{};
    BodyCalibrationModeConfig body_calibration{};
    MonocularTrackingConfig monocular{};
    HmdDepthScaleConfig hmd_depth_scale{};
    StereoHmdAnchorConfig stereo_hmd_anchor{};
    AnchorSpaceMappingConfig anchor_space_mapping{};
    RoomDepthMapConfig room_depth_map{};
    StereoAnchorDepthCorrectionConfig stereo_anchor_depth_correction{};
};

struct HmdProviderConfig {
    std::string mode = "null";
    std::filesystem::path pose_json_path = "hmd_pose.json";
};

struct OscConfig {
    bool enabled = false;
    std::string target_address = "127.0.0.1";
    int target_port = 9000;
    bool send_rotations = true;
    float min_confidence = tracking_constants::kVisibleConfidenceThreshold;
    int pelvis_tracker_index = 1;
    int left_foot_tracker_index = 2;
    int right_foot_tracker_index = 3;
    int chest_tracker_index = 4;
    int left_elbow_tracker_index = 5;
    int right_elbow_tracker_index = 6;
    int left_knee_tracker_index = 0;
    int right_knee_tracker_index = 0;
    // Active camera-world -> VR tracker-space transform used by OSC. This may be
    // manual or controller-solved. Do not treat it as the manual fallback store.
    bool tracker_space_transform_valid = false;
    Vec3f tracker_space_position_offset{0.0f, 0.0f, 0.0f};
    Quatf tracker_space_rotation{0.0f, 0.0f, 0.0f, 1.0f};
    float tracker_space_scale = 1.0f;
    std::array<Vec3f, kOscTrackerRoleCount> tracker_space_role_offsets{};
    std::string tracker_space_source = "manual";

    // Preserved manual fallback. Controller alignment is allowed to become the
    // active transform, but it must not destroy this manual path.
    bool manual_tracker_space_transform_valid = false;
    Vec3f manual_tracker_space_position_offset{0.0f, 0.0f, 0.0f};
    Quatf manual_tracker_space_rotation{0.0f, 0.0f, 0.0f, 1.0f};
    float manual_tracker_space_scale = 1.0f;
    std::array<Vec3f, kOscTrackerRoleCount> manual_tracker_space_role_offsets{};
    std::string manual_tracker_space_source = "manual";
    std::string steamvr_alignment_status = "idle";
    std::string steamvr_alignment_reason;
    float steamvr_alignment_confidence = 0.0f;
    float steamvr_alignment_residual_m = 0.0f;
    float steamvr_floor_residual_m = 0.0f;
    float steamvr_yaw_offset_rad = 0.0f;
    float steamvr_scale_ratio = 1.0f;
    std::string steamvr_alignment_body_signature;
    std::string steamvr_alignment_floor_signature;
};

struct SteamVrTrackerBridgeConfig {
    bool enabled = true;
    std::string target_address = "127.0.0.1";
    int target_port = 39560;
    float min_confidence = tracking_constants::kVisibleConfidenceThreshold;
    bool send_chest = true;
    bool send_elbows = true;
    bool send_knees = false;
};

struct CameraConfig {
    // source=opencv keeps the existing local camera path. source=network_mjpeg
    // listens for the Android sender's raw TCP MJPEG stream and publishes the
    // decoded pixels into the same FrameSlot as any other camera.
    std::string source = "opencv";
    int device_index = 0;
    int width = 1280;
    int height = 720;
    int fps = 60;
    std::string network_bind_address = "0.0.0.0";
    int network_port = 39555;
    int network_read_timeout_ms = 1000;
    int network_max_frame_bytes = 8 * 1024 * 1024;
    bool initial_roi_enabled = false;
    bool initial_roi_normalized = true;
    Rect2f initial_roi{0.0f, 0.0f, 1.0f, 1.0f};
};

struct AppConfig {
    AppSectionConfig app{};
    DebugConfig debug{};
    InferenceConfig inference{};
    TrackingConfig tracking{};
    HmdProviderConfig hmd{};
    OscConfig osc{};
    SteamVrTrackerBridgeConfig steamvr_tracker_bridge{};
    CameraConfig camera_a{};
    CameraConfig camera_b = [] {
        CameraConfig c{};
        c.device_index = 1;
        c.network_port = 39556;
        return c;
    }();
    std::filesystem::path calibration_path = "calib/default.json";
};

enum class ConfigValidationOutcome {
    Invalid,
    Degraded,
    MissingButDefaultable,
    Warning
};

struct ConfigValidationIssue {
    ConfigValidationOutcome outcome = ConfigValidationOutcome::Warning;
    std::string path;
    std::string message;
};

struct ConfigValidationReport {
    std::vector<ConfigValidationIssue> issues;

    [[nodiscard]] bool HasOutcome(ConfigValidationOutcome outcome) const noexcept;
    [[nodiscard]] bool HasInvalid() const noexcept;
};

[[nodiscard]] const char* ToString(ConfigValidationOutcome outcome) noexcept;
[[nodiscard]] ConfigValidationReport ValidateConfig(const AppConfig& cfg);

Result<AppConfig> LoadConfig(const std::filesystem::path& path);
Status SaveDefaultConfig(const std::filesystem::path& path);

} // namespace bt
