#pragma once

#include "capture/capture_health.h"
#include "capture/frame_pairer.h"
#include "core/profiler.h"
#include "tracking/roi_tracker.h"
#include "tracking/reliability.h"
#include "tracking/tracking_pipeline.h"
#include "tracking/steamvr_alignment.h"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace bt {

struct DebugSnapshot {
    std::string phase = "bootstrap";
    std::string degradation_mode;
    std::string last_error;
    double timestamp_seconds = 0.0;
    double time_seconds = 0.0;
    CaptureHealthSnapshot camera_a{};
    CaptureHealthSnapshot camera_b{};
    FramePairerTelemetry frame_pairing{};
    std::uint64_t frame_a_sequence = 0;
    std::uint64_t frame_b_sequence = 0;
    double frame_skew_ms = 0.0;
    bool frame_pair_degraded = false;
    bool frame_pair_reused_a = false;
    bool frame_pair_reused_b = false;
    bool frame_pair_duplicate = false;
    bool frame_pair_skewed = false;
    std::string frame_pair_reason;
    double camera_a_frame_age_ms = 0.0;
    double camera_b_frame_age_ms = 0.0;
    RoiState camera_a_roi{};
    RoiState camera_b_roi{};
    RoiState view_a_roi{};
    RoiState view_b_roi{};
    DecodedPose2D camera_a_pose{};
    DecodedPose2D camera_b_pose{};
    ReliabilitySummary camera_a_reliability_full{};
    ReliabilitySummary camera_b_reliability_full{};
    ReliabilitySummary view_a_reliability{};
    ReliabilitySummary view_b_reliability{};
    TrackingPipelineSnapshot tracking{};
    TrackingSolverTelemetry solver{};
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
    float camera_a_pose_confidence = 0.0f;
    float camera_b_pose_confidence = 0.0f;
    float camera_a_reliability = 0.0f;
    float camera_b_reliability = 0.0f;
    double inference_ms = 0.0;
    double inference_ms_a = 0.0;
    double inference_ms_b = 0.0;
    std::string model_active_device = "directml";
    bool model_ep_fallback = false;
    double capture_ms = 0.0;
    double frame_pair_ms = 0.0;
    double preprocess_ms = 0.0;
    double preprocess_ms_a = 0.0;
    double preprocess_ms_b = 0.0;
    double onnx_ms = 0.0;
    double onnx_ms_a = 0.0;
    double onnx_ms_b = 0.0;
    double decode_ms = 0.0;
    double decode_ms_a = 0.0;
    double decode_ms_b = 0.0;
    double pipeline_ms = 0.0;
    double solver_ms = 0.0;
    double preliminary_solve_ms = 0.0;
    double final_solve_ms = 0.0;
    double osc_ms = 0.0;
    double ui_publish_ms = 0.0;
    double total_ms = 0.0;
    int objective_evaluations = 0;
    int coordinate_passes = 0;
    bool optimizer_early_stopped = false;
    ProfilerSnapshot profiler{};
    bool osc_enabled = false;
    bool osc_open = false;
    bool osc_last_send_ok = false;
    std::string osc_status = "disabled";
    std::string osc_last_error;
    std::vector<std::string> osc_open_attempts{};
    std::string osc_target_address;
    int osc_target_port = 0;
    bool osc_tracker_space_transform_valid = false;
    std::string osc_tracker_space_source = "manual";
    bool osc_manual_tracker_space_fallback_valid = false;
    std::string osc_manual_tracker_space_source = "manual";
    int osc_sent_tracker_count = 0;
    int osc_skipped_tracker_count = 0;
    int osc_sent_message_count = 0;
    std::array<int, kTrackerPoseCount> osc_role_indices{};
    std::array<bool, kTrackerPoseCount> osc_role_configured{};
    std::array<bool, kTrackerPoseCount> osc_role_valid{};
    std::array<bool, kTrackerPoseCount> osc_role_sent{};
    std::array<bool, kTrackerPoseCount> osc_role_degraded{};
    std::array<std::string, kTrackerPoseCount> osc_role_reasons{};
    std::array<std::string, kTrackerPoseCount> osc_role_error_details{};
    bool steamvr_bridge_enabled = false;
    bool steamvr_bridge_open = false;
    bool steamvr_bridge_last_send_ok = false;
    std::string steamvr_bridge_status = "disabled";
    std::string steamvr_bridge_last_error;
    std::string steamvr_bridge_target_address;
    int steamvr_bridge_target_port = 0;
    float steamvr_bridge_min_confidence = 0.0f;
    int steamvr_bridge_sent_tracker_count = 0;
    int steamvr_bridge_skipped_tracker_count = 0;
    int steamvr_bridge_sent_message_count = 0;
    std::uint64_t steamvr_bridge_sequence = 0;
    std::array<bool, kTrackerPoseCount> steamvr_bridge_role_enabled{};
    std::array<bool, kTrackerPoseCount> steamvr_bridge_role_valid{};
    std::array<bool, kTrackerPoseCount> steamvr_bridge_role_sent{};
    std::array<bool, kTrackerPoseCount> steamvr_bridge_role_degraded{};
    std::array<float, kTrackerPoseCount> steamvr_bridge_role_confidence{};
    std::array<std::string, kTrackerPoseCount> steamvr_bridge_role_reasons{};
    bool steamvr_alignment_recorded = false;
    SteamVrAlignmentStatus steamvr_alignment{};
    HmdPoseSample hmd{};
    bool hmd_valid = false;
};

} // namespace bt
