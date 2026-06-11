#pragma once

#include "calibration/calibration_types.h"
#include "core/config.h"
#include "core/status.h"
#include "core/types.h"
#include "monocular_depth/hmd_depth_scale.h"
#include "tracking/anchor_space_mapper.h"

#include <array>

namespace bt {

struct MonocularProjectionProfile {
    float fx = 0.0f;
    float fy = 0.0f;
    float cx = 0.0f;
    float cy = 0.0f;
    int image_width = 0;
    int image_height = 0;
    float camera_height_m = 0.0f;
    bool valid = false;
};

struct MonocularJointMeasurement {
    Vec3f world{};
    float confidence = 0.0f;
    float estimated_depth_m = 0.0f;
    bool present = false;
};

struct MonocularMeasurementResult {
    std::array<MonocularJointMeasurement, kHalpe26Count> joints{};
    int valid_count = 0;
    float mean_confidence = 0.0f;
    float estimated_depth_m = 0.0f;
    float depth_confidence = 0.0f;
    DepthSource depth_source = DepthSource::InferredMonocular;
    MonocularScaleSource scale_source = MonocularScaleSource::None;
    float floor_assist_depth_m = 0.0f;
    float floor_assist_confidence = 0.0f;
    bool distortion_correction_used = false;
    bool camera_orientation_correction_used = false;
    // Number of joints whose depth was adjusted by the bone-length
    // perspective lift (torso/head/arm depth structure recovery).
    int perspective_lift_count = 0;
    HmdDepthScaleResult hmd_depth_scale{};
    HmdDepthScaleHistory hmd_depth_scale_history{};
    RawAnchorWorlds anchor_raw_worlds{};
    ProjectionCorrection anchor_space_mapping{};
    AnchorProjectionCorrectionDebug anchor_correction_debug{};
    RoomDepthMapTelemetry room_depth_map{};
};

MonocularProjectionProfile MakeMonocularProjectionProfile(
    const CameraCalibration& manual_camera,
    const MonocularTrackingConfig& config);

CameraCalibration MakeMonocularCameraCalibration(
    const CameraCalibration& manual_camera,
    const MonocularTrackingConfig& config);

FloorPlane MakeMonocularFloorPlane(const MonocularTrackingConfig& config);

Result<Vec3f> BackProjectMonocularPixel(
    const MonocularProjectionProfile& profile,
    const Vec2f& pixel,
    float depth_m);

Result<MonocularMeasurementResult> BuildMonocularJointMeasurements(
    const KeypointArray& keypoints,
    const std::array<float, kHalpe26Count>& reliability_weights,
    const CameraCalibration& manual_camera,
    const MonocularTrackingConfig& config,
    const HmdDepthScaleRuntimeInput* hmd_depth_scale = nullptr,
    const AnchorSpaceMappingRuntimeInput* anchor_mapping = nullptr);

} // namespace bt
