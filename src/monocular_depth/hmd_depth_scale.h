#pragma once

#include "calibration/calibration_types.h"
#include "core/config.h"
#include "core/types.h"

#include <array>
#include <string>

namespace bt {

struct MonocularMeasurementResult;

inline const char* ToString(HmdDepthScaleStateKind state) {
    switch (state) {
    case HmdDepthScaleStateKind::Live: return "live";
    case HmdDepthScaleStateKind::HeldHeadMissing: return "held_head_missing";
    case HmdDepthScaleStateKind::HeldHeadOutlier: return "held_head_outlier";
    case HmdDepthScaleStateKind::HeldHmdTrackingLost: return "held_hmd_tracking_lost";
    case HmdDepthScaleStateKind::HeldImplausibleScale: return "held_implausible_scale";
    case HmdDepthScaleStateKind::UnavailableHmdTrackingLost: return "unavailable_hmd_tracking_lost";
    case HmdDepthScaleStateKind::UnavailableCameraExtrinsics: return "unavailable_camera_extrinsics";
    case HmdDepthScaleStateKind::UnavailableNoPreviousScale: return "unavailable_no_previous_scale";
    case HmdDepthScaleStateKind::Disabled: return "disabled";
    default: return "unknown";
    }
}

struct HmdDepthScaleFrameObservation {
    bool valid = false;
    KeypointId head_keypoint = KeypointId::HeadTop;
    Vec2f head_px{};
    Vec3f head_ray_camera_unit{};
    Vec3f hmd_world{};
    Vec3f hmd_camera{};
    float mono_head_depth_m = 0.0f;
    float true_head_depth_z_m = 0.0f;
    float scale = 1.0f;
    double camera_hmd_timestamp_delta_ms = 0.0;
};

struct HmdDepthScaleResult {
    HmdDepthScaleStateKind state = HmdDepthScaleStateKind::Disabled;
    std::string reason = "disabled";
    bool live = false;
    bool held = false;
    bool usable = false;
    float scale = 1.0f;
    HmdDepthScaleFrameObservation observation{};
    Vec3f corrected_head_world{};
    Vec3f corrected_root_world{};
    bool corrected_root_valid = false;
};

struct HmdDepthScaleHistory {
    static constexpr int kMaxHistory = 15;
    bool has_last_valid = false;
    float last_scale = 1.0f;
    double last_valid_time_seconds = 0.0;
    Vec3f last_hmd_world{};
    Vec3f last_corrected_root_world{};
    bool last_corrected_root_valid = false;
    std::array<float, kMaxHistory> accepted_log_scale{};
    std::array<Vec2f, kMaxHistory> accepted_head_px{};
    int accepted_count = 0;
    int next_index = 0;
};

struct HmdDepthScaleRuntimeInput {
    bool enabled = false;
    HmdDepthScaleConfig config{};
    HmdDepthScaleHistory history{};
    HmdPoseSample hmd{};
    double camera_timestamp_seconds = 0.0;
    double now_seconds = 0.0;
};

Vec3f HmdDepthImageRayCameraUnit(float u, float v, float fx, float fy, float cx, float cy);
Vec3f HmdDepthCameraToWorldPoint(const Mat34f& world_from_camera, Vec3f camera);
Vec3f HmdDepthWorldToCameraPoint(const Mat34f& world_from_camera, Vec3f world);
Vec3f HmdDepthBackProjectCamera(float u, float v, float depth_z_m, float fx, float fy, float cx, float cy);

HmdDepthScaleResult ComputeHmdDepthScale(
    const HmdDepthScaleConfig& config,
    const KeypointArray& keypoints,
    const CameraCalibration& camera,
    const HmdPoseSample& hmd,
    double camera_timestamp_seconds,
    double now_seconds,
    float mono_head_depth_m,
    const HmdDepthScaleHistory& history);

HmdDepthScaleHistory UpdateHmdDepthScaleHistory(
    const HmdDepthScaleHistory& previous,
    const HmdDepthScaleConfig& config,
    const HmdDepthScaleResult& result,
    double now_seconds);

bool ComputeHmdAnchoredRoot(
    const MonocularMeasurementResult& measurements,
    const HmdDepthScaleResult& scale,
    Vec3f* corrected_root_world_out);

} // namespace bt
