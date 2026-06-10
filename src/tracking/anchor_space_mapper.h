#pragma once

#include "calibration/calibration_types.h"
#include "core/config.h"
#include "core/math.h"
#include "core/types.h"
#include "io/steamvr_provider.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>

namespace bt {

using RawAnchorWorlds = std::array<Keypoint3D, kHalpe26Count>;

struct AnchorProjectionCorrectionDebug {
    RawAnchorWorlds corrected_worlds{};
    std::array<float, kHalpe26Count> corrected_depths{};
    std::array<bool, kHalpe26Count> applied{};
    std::array<std::string, kHalpe26Count> rejection_reasons{};
};

inline AnchorProjectionCorrectionDebug MakeAnchorProjectionCorrectionDebug(const std::string& reason = "not_evaluated") {
    AnchorProjectionCorrectionDebug debug;
    for (auto& r : debug.rejection_reasons) {
        r = reason;
    }
    return debug;
}

inline void CollectRawAnchorWorld(
    KeypointId id,
    const Vec3f& world,
    float confidence,
    bool present,
    RawAnchorWorlds& raw_worlds) {

    if (!present || !IsFinite(world)) {
        return;
    }
    const std::size_t i = static_cast<std::size_t>(id);
    if (i >= raw_worlds.size()) {
        return;
    }
    raw_worlds[i].world = world;
    raw_worlds[i].confidence = confidence;
    raw_worlds[i].present = true;
}

enum class AnchorType : std::uint8_t {
    HmdHead = 0,
    LeftController,
    RightController
};

inline const char* ToString(AnchorType type) {
    switch (type) {
    case AnchorType::HmdHead: return "hmd_head";
    case AnchorType::LeftController: return "left_controller";
    case AnchorType::RightController: return "right_controller";
    default: return "unknown";
    }
}

struct SteamVrAnchorFrame {
    bool hmd_valid = false;
    bool left_controller_valid = false;
    bool right_controller_valid = false;
    Pose3f hmd_pose{};
    Pose3f left_controller_pose{};
    Pose3f right_controller_pose{};
    Vec3f hmd_world{};
    Vec3f left_controller_world{};
    Vec3f right_controller_world{};
    double steamvr_timestamp_seconds = 0.0;
    double predicted_or_sampled_time_offset_seconds = 0.0;
    bool available = false;
    std::string status = "unavailable";
    std::string reason = "SteamVR anchors unavailable";
};

struct ImageAnchorObservation {
    bool present = false;
    Vec2f pixel{};
    AnchorType type = AnchorType::HmdHead;
};

struct ProjectionCorrection {
    bool valid = false;
    bool used_live_anchor = false;
    bool used_room_prior = false;
    bool usable_for_room_map_update = false;
    int anchors_used = 0;
    float depth_scale = 1.0f;
    Vec3f translation_delta_world{};
    Quatf rotation_delta_world{};
    float reprojection_error_px = 0.0f;
    float max_reprojection_error_px = 0.0f;
    double camera_anchor_timestamp_delta_ms = 0.0;
    std::string mode = "disabled";
    std::string fallback_reason = "disabled";
};

struct RoomDepthCell {
    bool valid = false;
    float depth_m = 0.0f;
    float variance_m2 = 0.0f;
    std::uint32_t sample_count = 0;
    double last_update_seconds = 0.0;
};

struct RoomDepthSample {
    bool valid = false;
    float depth_m = 0.0f;
    float variance_m2 = 0.0f;
    std::uint32_t sample_count = 0;
};

struct RoomDepthMapTelemetry {
    std::string state = "disabled";
    std::uint64_t accepted_frames = 0;
    std::uint64_t rejected_frames = 0;
    float coverage = 0.0f;
    float mean_variance_m2 = 0.0f;
    double last_accepted_update_time_seconds = 0.0;
    std::string last_rejection_reason = "disabled";
};

struct ProjectionInputs {
    Vec2f pixel{};
    float raw_depth_m = 0.0f;
    Vec3f raw_world{};
    ProjectionCorrection live_anchor_correction{};
    RoomDepthSample room_prior{};
};

struct ProjectionOutput {
    Vec3f world_position{};
    float corrected_depth_m = 0.0f;
    bool used_live_anchor = false;
    bool used_room_prior = false;
    float live_anchor_scale = 1.0f;
    float room_prior_depth_delta_m = 0.0f;
    std::string fallback_reason;
};

struct AnchorSpaceMappingRuntimeInput {
    AnchorSpaceMappingConfig config{};
    RoomDepthMapConfig room_map_config{};
    StereoAnchorDepthCorrectionConfig stereo_depth_config{};
    SteamVrAnchorFrame anchors{};
    RoomDepthMapTelemetry room_map{};
    double camera_timestamp_seconds = 0.0;
    double now_seconds = 0.0;
};

inline SteamVrAnchorFrame SteamVrAnchorFrameFromSnapshot(const SteamVrPoseSnapshot& snapshot) {
    SteamVrAnchorFrame out;
    out.available = snapshot.available;
    out.status = snapshot.status;
    out.reason = snapshot.reason;
    out.hmd_valid = snapshot.hmd_tracked && snapshot.hmd.valid && IsFinite(snapshot.hmd.pose.position);
    out.left_controller_valid = snapshot.left.valid && IsFinite(snapshot.left.pose.position);
    out.right_controller_valid = snapshot.right.valid && IsFinite(snapshot.right.pose.position);
    out.hmd_pose = snapshot.hmd.pose;
    out.left_controller_pose = snapshot.left.pose;
    out.right_controller_pose = snapshot.right.pose;
    out.hmd_world = snapshot.hmd.pose.position;
    out.left_controller_world = snapshot.left.pose.position;
    out.right_controller_world = snapshot.right.pose.position;
    double t = 0.0;
    if (snapshot.hmd.timestamp_seconds != 0.0) {
        t = snapshot.hmd.timestamp_seconds;
    } else if (snapshot.left.timestamp_seconds != 0.0) {
        t = snapshot.left.timestamp_seconds;
    } else if (snapshot.right.timestamp_seconds != 0.0) {
        t = snapshot.right.timestamp_seconds;
    }
    out.steamvr_timestamp_seconds = t;
    return out;
}

inline bool PixelFiniteInside(const Vec2f& pixel, int width, int height) {
    return std::isfinite(pixel.x) && std::isfinite(pixel.y) &&
        width > 0 && height > 0 &&
        pixel.x >= 0.0f && pixel.y >= 0.0f &&
        pixel.x < static_cast<float>(width) && pixel.y < static_cast<float>(height);
}


inline Vec3f AnchorMapperCameraToWorldPoint(const Mat34f& world_from_camera, const Vec3f& camera) {
    return Vec3f{
        world_from_camera.m[0] * camera.x + world_from_camera.m[1] * camera.y + world_from_camera.m[2] * camera.z + world_from_camera.m[3],
        world_from_camera.m[4] * camera.x + world_from_camera.m[5] * camera.y + world_from_camera.m[6] * camera.z + world_from_camera.m[7],
        world_from_camera.m[8] * camera.x + world_from_camera.m[9] * camera.y + world_from_camera.m[10] * camera.z + world_from_camera.m[11]
    };
}

inline Vec3f AnchorMapperWorldToCameraPoint(const Mat34f& world_from_camera, const Vec3f& world) {
    const Vec3f t{world_from_camera.m[3], world_from_camera.m[7], world_from_camera.m[11]};
    const Vec3f d = Sub(world, t);
    return Vec3f{
        world_from_camera.m[0] * d.x + world_from_camera.m[4] * d.y + world_from_camera.m[8] * d.z,
        world_from_camera.m[1] * d.x + world_from_camera.m[5] * d.y + world_from_camera.m[9] * d.z,
        world_from_camera.m[2] * d.x + world_from_camera.m[6] * d.y + world_from_camera.m[10] * d.z
    };
}

inline bool ProjectWorldToImage(const CameraCalibration& camera, const Vec3f& world, Vec2f* out) {
    if (!out || !camera.extrinsics_valid || !IsFinite(world)) {
        return false;
    }
    const Vec3f projected = ProjectPoint(camera.image_from_world, world);
    if (!std::isfinite(projected.x) || !std::isfinite(projected.y)) {
        return false;
    }
    *out = Vec2f{projected.x, projected.y};
    return true;
}

inline bool AnchorRawWorld(
    AnchorType type,
    const RawAnchorWorlds& raw_worlds,
    Vec3f* out) {
    if (!out) {
        return false;
    }
    const KeypointId id = type == AnchorType::HmdHead
        ? KeypointId::HeadTop
        : (type == AnchorType::LeftController ? KeypointId::LeftWrist : KeypointId::RightWrist);
    const auto& kp = raw_worlds[static_cast<std::size_t>(id)];
    if (!kp.present || !IsFinite(kp.world)) {
        return false;
    }
    *out = kp.world;
    return true;
}

inline bool AnchorPixel(
    AnchorType type,
    const KeypointArray& keypoints,
    int width,
    int height,
    ImageAnchorObservation* out) {
    if (!out) {
        return false;
    }
    const KeypointId id = type == AnchorType::HmdHead
        ? KeypointId::HeadTop
        : (type == AnchorType::LeftController ? KeypointId::LeftWrist : KeypointId::RightWrist);
    const auto& kp = keypoints[static_cast<std::size_t>(id)];
    if (!kp.present || !PixelFiniteInside(kp.pixel, width, height)) {
        return false;
    }
    *out = ImageAnchorObservation{true, kp.pixel, type};
    return true;
}

inline bool AnchorTrueWorld(
    AnchorType type,
    const SteamVrAnchorFrame& anchors,
    const AnchorSpaceMappingConfig& config,
    Vec3f* out) {
    if (!out) {
        return false;
    }
    if (type == AnchorType::HmdHead) {
        if (!anchors.hmd_valid) return false;
        *out = Add(anchors.hmd_world, config.hmd_to_head_keypoint_offset_m);
        return IsFinite(*out);
    }
    if (type == AnchorType::LeftController) {
        if (!anchors.left_controller_valid) return false;
        *out = Add(anchors.left_controller_world, config.left_controller_to_wrist_offset_m);
        return IsFinite(*out);
    }
    if (!anchors.right_controller_valid) return false;
    *out = Add(anchors.right_controller_world, config.right_controller_to_wrist_offset_m);
    return IsFinite(*out);
}

inline Vec3f ApplyDepthScaleInCameraSpace(
    const CameraCalibration& camera,
    const Vec3f& raw_world,
    float scale,
    float* corrected_depth_m = nullptr) {
    if (!camera.extrinsics_valid || !IsFinite(raw_world) || !std::isfinite(scale) || scale <= 0.0f) {
        if (corrected_depth_m) *corrected_depth_m = 0.0f;
        return raw_world;
    }
    const Vec3f camera_point = AnchorMapperWorldToCameraPoint(camera.world_from_camera, raw_world);
    if (!IsFinite(camera_point)) {
        if (corrected_depth_m) *corrected_depth_m = 0.0f;
        return raw_world;
    }
    const Vec3f corrected_camera{camera_point.x, camera_point.y, camera_point.z * scale};
    if (corrected_depth_m) *corrected_depth_m = corrected_camera.z;
    return AnchorMapperCameraToWorldPoint(camera.world_from_camera, corrected_camera);
}

inline Vec3f ApplyProjectionCorrectionToWorldPoint(
    const CameraCalibration& camera,
    const ProjectionCorrection& correction,
    const Vec3f& raw_world,
    float* corrected_depth_m = nullptr) {
    if (!correction.valid || !IsFinite(raw_world)) {
        if (corrected_depth_m) *corrected_depth_m = 0.0f;
        return raw_world;
    }
    Vec3f corrected = ApplyDepthScaleInCameraSpace(camera, raw_world, correction.depth_scale, corrected_depth_m);
    if (correction.anchors_used >= 2) {
        corrected = Add(corrected, correction.translation_delta_world);
    }
    return corrected;
}

inline AnchorProjectionCorrectionDebug ApplyAnchorProjectionCorrectionToRawWorlds(
    const CameraCalibration& camera,
    const ProjectionCorrection& correction,
    const RawAnchorWorlds& raw_worlds) {

    if (!correction.valid) {
        return MakeAnchorProjectionCorrectionDebug(
            correction.fallback_reason.empty() ? "correction_invalid" : correction.fallback_reason);
    }
    if (!camera.extrinsics_valid) {
        return MakeAnchorProjectionCorrectionDebug("camera_extrinsics_invalid");
    }

    AnchorProjectionCorrectionDebug debug = MakeAnchorProjectionCorrectionDebug();
    for (std::size_t i = 0; i < raw_worlds.size(); ++i) {
        const auto& raw = raw_worlds[i];
        if (!raw.present || !IsFinite(raw.world)) {
            debug.rejection_reasons[i] = "raw_world_unavailable";
            continue;
        }
        float corrected_depth = 0.0f;
        const Vec3f corrected = ApplyProjectionCorrectionToWorldPoint(camera, correction, raw.world, &corrected_depth);
        if (!IsFinite(corrected)) {
            debug.rejection_reasons[i] = "corrected_world_invalid";
            continue;
        }
        debug.corrected_worlds[i].world = corrected;
        debug.corrected_worlds[i].confidence = raw.confidence;
        debug.corrected_worlds[i].present = true;
        debug.corrected_depths[i] = corrected_depth;
        debug.applied[i] = true;
        debug.rejection_reasons[i] = "applied";
    }
    return debug;
}

inline ProjectionOutput ProjectKeypointToSteamVr(
    const CameraCalibration& camera,
    const ProjectionInputs& inputs) {
    ProjectionOutput out;
    out.world_position = inputs.raw_world;
    out.corrected_depth_m = inputs.raw_depth_m;
    out.live_anchor_scale = inputs.live_anchor_correction.depth_scale;
    out.fallback_reason = inputs.live_anchor_correction.fallback_reason;
    if (inputs.live_anchor_correction.valid) {
        out.world_position = ApplyProjectionCorrectionToWorldPoint(
            camera, inputs.live_anchor_correction, inputs.raw_world, &out.corrected_depth_m);
        out.used_live_anchor = true;
    }
    if (inputs.room_prior.valid && std::isfinite(inputs.room_prior.depth_m) && inputs.room_prior.depth_m > 0.0f) {
        const float delta = inputs.room_prior.depth_m - out.corrected_depth_m;
        out.room_prior_depth_delta_m = delta;
        // Conservative ramp-in: room prior is telemetry-only until map maturity is active.
        out.used_room_prior = false;
    }
    return out;
}

inline ProjectionCorrection EstimateAnchorProjectionCorrection(
    const AnchorSpaceMappingConfig& config,
    const CameraCalibration& camera,
    const KeypointArray& image_keypoints,
    const RawAnchorWorlds& raw_worlds,
    const SteamVrAnchorFrame& anchors,
    int image_width,
    int image_height,
    double camera_timestamp_seconds) {

    ProjectionCorrection out;
    out.depth_scale = 1.0f;
    out.rotation_delta_world = Quatf{};
    if (!config.enabled) {
        out.mode = "disabled";
        out.fallback_reason = "disabled";
        return out;
    }
    out.mode = "fallback";
    if (!camera.extrinsics_valid || !camera.intrinsics_valid) {
        out.fallback_reason = "camera_calibration_unavailable";
        return out;
    }
    if (!anchors.available) {
        out.fallback_reason = "steamvr_unavailable";
        return out;
    }
    if (config.use_hmd && !anchors.hmd_valid) {
        out.fallback_reason = "hmd_anchor_unavailable";
        return out;
    }
    if (camera_timestamp_seconds != 0.0 && anchors.steamvr_timestamp_seconds != 0.0) {
        const double dt = std::abs(camera_timestamp_seconds - anchors.steamvr_timestamp_seconds);
        out.camera_anchor_timestamp_delta_ms = dt * 1000.0;
        if (config.timestamp_alignment_seconds > 0.0 && dt > config.timestamp_alignment_seconds) {
            out.fallback_reason = "timestamp_misaligned";
            return out;
        }
    }

    struct AnchorSample {
        AnchorType type = AnchorType::HmdHead;
        Vec2f observed_px{};
        Vec3f raw_world{};
        Vec3f true_world{};
        float scale = 1.0f;
        float reprojection_error_px = 0.0f;
    };
    std::array<AnchorSample, 3> samples{};
    int count = 0;
    const std::array<AnchorType, 3> candidates{
        AnchorType::HmdHead,
        AnchorType::LeftController,
        AnchorType::RightController
    };
    for (const AnchorType type : candidates) {
        if (type != AnchorType::HmdHead && !config.use_controllers) {
            continue;
        }
        ImageAnchorObservation obs;
        Vec3f raw_world;
        Vec3f true_world;
        if (!AnchorPixel(type, image_keypoints, image_width, image_height, &obs) ||
            !AnchorRawWorld(type, raw_worlds, &raw_world) ||
            !AnchorTrueWorld(type, anchors, config, &true_world)) {
            continue;
        }
        const Vec3f raw_camera = AnchorMapperWorldToCameraPoint(camera.world_from_camera, raw_world);
        const Vec3f true_camera = AnchorMapperWorldToCameraPoint(camera.world_from_camera, true_world);
        if (!IsFinite(raw_camera) || !IsFinite(true_camera) || raw_camera.z <= 1e-4f || true_camera.z <= 1e-4f) {
            continue;
        }
        Vec2f true_projected;
        if (!ProjectWorldToImage(camera, true_world, &true_projected)) {
            continue;
        }
        AnchorSample s;
        s.type = type;
        s.observed_px = obs.pixel;
        s.raw_world = raw_world;
        s.true_world = true_world;
        s.scale = true_camera.z / raw_camera.z;
        s.reprojection_error_px = Distance(obs.pixel, true_projected);
        samples[static_cast<std::size_t>(count++)] = s;
    }

    if (count == 0) {
        out.fallback_reason = "no_anchor_correspondence";
        return out;
    }
    if (count == 1 && samples[0].type != AnchorType::HmdHead) {
        out.fallback_reason = "single_non_hmd_anchor_rejected";
        return out;
    }
    if (count == 1 && !config.allow_hmd_only_scale_fallback) {
        out.fallback_reason = "hmd_only_disabled";
        return out;
    }
    float scale_sum = 0.0f;
    float error_sum = 0.0f;
    float max_error = 0.0f;
    for (int i = 0; i < count; ++i) {
        scale_sum += samples[i].scale;
        error_sum += samples[i].reprojection_error_px;
        max_error = std::max(max_error, samples[i].reprojection_error_px);
    }
    const float scale = scale_sum / static_cast<float>(count);
    const float mean_error = error_sum / static_cast<float>(count);
    out.reprojection_error_px = mean_error;
    out.max_reprojection_error_px = max_error;
    if (!std::isfinite(scale) || scale < config.min_depth_scale || scale > config.max_depth_scale) {
        out.fallback_reason = "depth_scale_out_of_bounds";
        out.depth_scale = std::isfinite(scale) ? scale : 1.0f;
        return out;
    }
    if (!std::isfinite(mean_error) || mean_error > config.max_reprojection_error_px || max_error > config.max_reprojection_error_px * 2.0f) {
        out.fallback_reason = "anchor_reprojection_error";
        out.depth_scale = scale;
        return out;
    }

    Vec3f translation_sum{};
    int translation_count = 0;
    if (count >= 2) {
        for (int i = 0; i < count; ++i) {
            const Vec3f depth_scaled = ApplyDepthScaleInCameraSpace(camera, samples[i].raw_world, scale);
            if (IsFinite(depth_scaled)) {
                translation_sum = Add(translation_sum, Sub(samples[i].true_world, depth_scaled));
                ++translation_count;
            }
        }
    }
    out.depth_scale = scale;
    if (translation_count > 0) {
        out.translation_delta_world = Scale(translation_sum, 1.0f / static_cast<float>(translation_count));
    }
    out.anchors_used = count;
    out.valid = true;
    out.used_live_anchor = true;
    if (count >= 3) {
        out.mode = "hmd_two_controllers";
    } else if (count == 2) {
        out.mode = "hmd_one_controller";
    } else {
        out.mode = "hmd_only";
    }
    out.fallback_reason.clear();
    out.usable_for_room_map_update = count >= 2 &&
        mean_error <= config.target_reprojection_error_px &&
        scale >= config.room_map_min_update_depth_scale &&
        scale <= config.room_map_max_update_depth_scale;
    return out;
}

inline RoomDepthMapTelemetry UpdateRoomDepthMapTelemetry(
    const RoomDepthMapTelemetry& previous,
    const RoomDepthMapConfig& config,
    const ProjectionCorrection& correction,
    double now_seconds) {

    RoomDepthMapTelemetry out = previous;
    if (!config.enabled) {
        out.state = "disabled";
        out.last_rejection_reason = "disabled";
        return out;
    }
    if (!correction.valid) {
        out.state = out.accepted_frames == 0 ? "warming_up" : "partial";
        ++out.rejected_frames;
        out.last_rejection_reason = correction.fallback_reason.empty() ? "anchor_correction_unavailable" : correction.fallback_reason;
        return out;
    }
    if (config.update_only_when_anchor_quality_good && !correction.usable_for_room_map_update) {
        out.state = out.accepted_frames == 0 ? "warming_up" : "partial";
        ++out.rejected_frames;
        out.last_rejection_reason = "anchor_quality_not_map_safe";
        return out;
    }
    ++out.accepted_frames;
    out.last_accepted_update_time_seconds = now_seconds;
    out.last_rejection_reason.clear();
    const float target = static_cast<float>(std::max(1, config.min_accepted_frames_before_active));
    out.coverage = std::min(1.0f, static_cast<float>(out.accepted_frames) / target);
    out.mean_variance_m2 = std::min(config.max_cell_variance_m2, correction.reprojection_error_px * correction.reprojection_error_px * 1e-6f);
    if (out.accepted_frames < static_cast<std::uint64_t>(std::max(1, config.min_accepted_frames_before_active / 5))) {
        out.state = "warming_up";
    } else if (out.accepted_frames < static_cast<std::uint64_t>(config.min_accepted_frames_before_active)) {
        out.state = "partial";
    } else {
        out.state = config.collect_only ? "collect_only_active" : "active";
    }
    return out;
}

} // namespace bt
