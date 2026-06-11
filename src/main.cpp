#include "calibration/calibration_io.h"
#include "calibration/floor_calibrator.h"
#include "calibration/intrinsic_calibrator.h"
#include "calibration/stereo_calibrator.h"
#include "capture/camera_device.h"
#include "capture/frame_pairer.h"
#include "core/config.h"
#include "core/logging.h"
#include "core/timing.h"
#include "debug/debug_snapshot.h"
#include "debug/replay_log.h"
#include "debug/replay_player.h"
#include "inference/rtmpose_decode.h"
#include "inference/rtmpose_session.h"
#include "io/hmd_provider.h"
#include "io/osc_sender.h"
#include "io/steamvr_provider.h"
#include "io/steamvr_tracker_bridge.h"
#include "tracking/reliability.h"
#include "tracking/roi_tracker.h"
#include "tracking/steamvr_alignment.h"
#include "tracking/steamvr_alignment_manager.h"
#include "tracking/steamvr_alignment_json.h"
#include "tracking/tracking_pipeline.h"
#include "ui/desktop_ui.h"
#include "web/runtime_state.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <exception>
#include <iomanip>
#include <limits>
#include <iostream>
#include <memory>
#include <mutex>
#include <deque>
#include <sstream>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <nlohmann/json.hpp>
#include <string>
#include <system_error>
#include <vector>
#include <thread>

#ifdef _WIN32
#include <windows.h>
#include <shellapi.h>
#endif

namespace {

std::atomic<bool> g_stop_requested = false;

void HandleSignal(int) {
    g_stop_requested = true;
}

struct ViewRuntimeState {
    bt::RoiTracker roi{};
    bt::DecodedPose2D previous_pose{};
    bt::RtmPoseInputPacket input_packet{};
    std::uint64_t depth_postprocess_frames = 0;
};

struct ViewProcessResult {
    bt::DecodedPose2D pose{};
    bt::DecodedPose3D pose3d{};  // model-depth companion; valid only for RTMW3D
    bt::ReliabilitySummary reliability{};
    bt::RoiState roi_used{};
    bt::RoiState roi_next{};
    double inference_ms = 0.0;
    double preprocess_ms = 0.0;
    double onnx_ms = 0.0;
    double decode_ms = 0.0;
    double frame_age_ms = 0.0;
    float age_confidence_scale = 1.0f;
};

constexpr const char* kDashboardWindowName = "bodytracker";

struct UiPalette {
    cv::Scalar base{233, 238, 239};
    cv::Scalar paper{250, 254, 255};
    cv::Scalar ink{4, 4, 4};
    cv::Scalar muted{89, 97, 101};
    cv::Scalar line{4, 4, 4};
    cv::Scalar hair{208, 211, 214};
    cv::Scalar soft{213, 221, 223};
    cv::Scalar accent{110, 0, 255};
    cv::Scalar good{60, 143, 10};
    cv::Scalar bad{31, 75, 255};
    cv::Scalar warn{0, 138, 198};
};

std::string Uppercase(std::string value) {
    for (auto& ch : value) {
        ch = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
    }
    return value;
}

std::string Fmt(double value, int decimals = 1) {
    if (!std::isfinite(value)) {
        return "--";
    }
    std::ostringstream out;
    out << std::fixed << std::setprecision(decimals) << value;
    return out.str();
}

std::string BoolText(bool value) {
    return value ? "YES" : "NO";
}

void ScaleReliability(bt::ReliabilitySummary& reliability, float scale) {
    if (!std::isfinite(scale)) {
        return;
    }
    scale = std::clamp(scale, 0.0f, 1.0f);
    reliability.mean_weight *= scale;
    reliability.lower_body_mean *= scale;
    reliability.foot_mean *= scale;
    for (auto& joint : reliability.joints) {
        joint.final_weight *= scale;
        joint.usable = joint.final_weight >= 0.15f;
    }
}

float PairReliabilityScale(const bt::PairedFrames& pair, bool camera_a) {
    if (!pair.valid || !pair.degraded) {
        return 1.0f;
    }
    if (pair.skewed) {
        return 0.75f;
    }
    if (pair.duplicate) {
        return 0.55f;
    }
    if ((camera_a && pair.reused_a) || (!camera_a && pair.reused_b)) {
        return 0.70f;
    }
    return 1.0f;
}

std::string FramePairDegradationSuffix(const bt::PairedFrames& pair) {
    if (!pair.valid || !pair.degraded) {
        return {};
    }
    return "frame_pair_" + (pair.reason.empty() ? std::string("degraded") : pair.reason);
}

nlohmann::json Vec3ToJson(const bt::Vec3f& v) {
    return {v.x, v.y, v.z};
}

nlohmann::json Vec3ArrayToJson(const std::array<bt::Vec3f, bt::kTrackerPoseCount>& values) {
    nlohmann::json out = nlohmann::json::array();
    for (const auto& v : values) {
        out.push_back(Vec3ToJson(v));
    }
    return out;
}

nlohmann::json Vec2ToJson(const bt::Vec2f& v) {
    return {v.x, v.y};
}

nlohmann::json Mat3ToJson(const std::array<float, 9>& m) {
    return {m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8]};
}

nlohmann::json FloorPlaneToJson(const bt::FloorPlane& floor) {
    return {
        {"valid", floor.valid},
        {"normal", Vec3ToJson(floor.normal)},
        {"distance", floor.distance}
    };
}

nlohmann::json FloorLineFamilyToJson(const bt::FloorGeometryLineFamily& f) {
    return {
        {"valid", f.valid},
        {"confidence", f.confidence},
        {"orientation_rad", f.orientation_rad},
        {"spacing_px", f.spacing_px},
        {"spacing_m", f.spacing_m},
        {"metric_spacing_valid", f.metric_spacing_valid},
        {"reference_rho_px", f.reference_rho_px},
        {"vanishing_point_px", Vec2ToJson(f.vanishing_point_px)},
        {"vanishing_point_valid", f.vanishing_point_valid},
        {"accepted_line_count", f.accepted_line_count},
        {"rejected_line_count", f.rejected_line_count},
        {"reason", f.reason}
    };
}

nlohmann::json LensDistortionToJson(const bt::LensDistortionEstimate& d) {
    return {
        {"available", d.available},
        {"valid", d.valid},
        {"applied_to_runtime", d.applied_to_runtime},
        {"confidence", d.confidence},
        {"radial_k1", d.radial_k1},
        {"radial_k2", d.radial_k2},
        {"tangential_p1", d.tangential_p1},
        {"tangential_p2", d.tangential_p2},
        {"straightness_error_px", d.straightness_error_px},
        {"corrected_straightness_error_px", d.corrected_straightness_error_px},
        {"sampled_seam_count", d.sampled_seam_count},
        {"sampled_point_count", d.sampled_point_count},
        {"model", d.model},
        {"reason", d.reason}
    };
}

std::string FloorGeometrySource(const bt::FloorGeometryCalibration& g) {
    if (!g.valid) {
        return "nothing";
    }
    return g.source.empty() || g.source == "unknown" ? "legacy_json" : g.source;
}

nlohmann::json FloorGeometryToJson(const bt::FloorGeometryCalibration& g) {
    return {
        {"valid", g.valid},
        {"source", FloorGeometrySource(g)},
        {"image_width", g.image_width},
        {"image_height", g.image_height},
        {"floor_type", g.floor_type},
        {"family_count", g.family_count},
        {"family_a", FloorLineFamilyToJson(g.family_a)},
        {"family_b", FloorLineFamilyToJson(g.family_b)},
        {"two_axis_grid_valid", g.two_axis_grid_valid},
        {"homography_valid", g.homography_valid},
        {"floor_from_image", Mat3ToJson(g.floor_from_image)},
        {"image_from_floor", Mat3ToJson(g.image_from_floor)},
        {"homography_reprojection_error_px", g.homography_reprojection_error_px},
        {"homography_inlier_count", g.homography_inlier_count},
        {"homography_intersection_count", g.homography_intersection_count},
        {"homography_reason", g.homography_reason},
        {"floor_plane", FloorPlaneToJson(g.floor_plane)},
        {"floor_plane_confidence", g.floor_plane_confidence},
        {"camera_orientation_valid", g.camera_orientation_valid},
        {"camera_pitch_rad", g.camera_pitch_rad},
        {"camera_roll_rad", g.camera_roll_rad},
        {"camera_yaw_rad", g.camera_yaw_rad},
        {"camera_orientation_confidence", g.camera_orientation_confidence},
        {"camera_orientation_applied_to_runtime", g.camera_orientation_applied_to_runtime},
        {"camera_height_valid", g.camera_height_valid},
        {"camera_height_m", g.camera_height_m},
        {"metric_scale_confidence", g.metric_scale_confidence},
        {"distortion", LensDistortionToJson(g.distortion)},
        {"multi_camera_alignment_valid", g.multi_camera_alignment_valid},
        {"multi_camera_alignment_confidence", g.multi_camera_alignment_confidence},
        {"multi_camera_warning", g.multi_camera_warning},
        {"multi_camera_yaw_delta_rad", g.multi_camera_yaw_delta_rad},
        {"multi_camera_pitch_delta_rad", g.multi_camera_pitch_delta_rad},
        {"multi_camera_roll_delta_rad", g.multi_camera_roll_delta_rad},
        {"multi_camera_height_delta_m", g.multi_camera_height_delta_m},
        {"multi_camera_scale_ratio", g.multi_camera_scale_ratio},
        {"shared_floor_frame_valid", g.shared_floor_frame_valid},
        {"shared_floor_transform", Mat3ToJson(g.shared_floor_transform)},
        {"planted_drift_axis_confidence", g.planted_drift_axis_confidence},
        {"reason", g.reason}
    };
}

std::string KeypointNameForIndex(std::size_t index) {
    if (index < static_cast<std::size_t>(bt::kHalpe26Count)) {
        return bt::ToString(static_cast<bt::KeypointId>(index));
    }
    return "keypoint_" + std::to_string(index);
}

nlohmann::json BodyOverlayViewToJson(const bt::WebRuntimeBodyOverlayView& view) {
    nlohmann::json keypoints = nlohmann::json::array();
    float min_x = std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float max_y = std::numeric_limits<float>::lowest();
    int present_count = 0;
    for (std::size_t i = 0; i < view.pose.keypoints.size(); ++i) {
        const auto& kp = view.pose.keypoints[i];
        if (!kp.present) {
            continue;
        }
        ++present_count;
        min_x = std::min(min_x, kp.pixel.x);
        min_y = std::min(min_y, kp.pixel.y);
        max_x = std::max(max_x, kp.pixel.x);
        max_y = std::max(max_y, kp.pixel.y);
        keypoints.push_back({
            {"index", static_cast<int>(i)},
            {"name", KeypointNameForIndex(i)},
            {"x", kp.pixel.x},
            {"y", kp.pixel.y},
            {"confidence", kp.confidence},
            {"present", kp.present}
        });
    }
    static constexpr std::array<std::array<int, 2>, 18> kSkeleton{{
        {{0, 1}}, {{0, 2}}, {{1, 3}}, {{2, 4}},
        {{5, 6}}, {{5, 7}}, {{7, 9}}, {{6, 8}}, {{8, 10}},
        {{5, 11}}, {{6, 12}}, {{11, 12}},
        {{11, 13}}, {{13, 15}}, {{15, 17}}, {{12, 14}}, {{14, 16}}, {{16, 18}}
    }};
    nlohmann::json skeleton = nlohmann::json::array();
    for (const auto& edge : kSkeleton) {
        skeleton.push_back({edge[0], edge[1]});
    }
    nlohmann::json bbox = {
        {"valid", present_count > 0},
        {"x", present_count > 0 ? min_x : 0.0f},
        {"y", present_count > 0 ? min_y : 0.0f},
        {"width", present_count > 0 ? std::max(0.0f, max_x - min_x) : 0.0f},
        {"height", present_count > 0 ? std::max(0.0f, max_y - min_y) : 0.0f}
    };
    return {
        {"slot", std::string(1, view.slot)},
        {"preview_available", view.preview_available},
        {"preview_width", view.preview_width},
        {"preview_height", view.preview_height},
        {"preview_source_x", view.preview_source_x},
        {"preview_source_y", view.preview_source_y},
        {"preview_source_width", view.preview_source_width},
        {"preview_source_height", view.preview_source_height},
        {"preview_frame_width", view.preview_frame_width},
        {"preview_frame_height", view.preview_frame_height},
        {"preview_sequence", view.preview_sequence},
        {"frame_sequence", view.frame_sequence},
        {"frame_age_ms", view.frame_age_ms},
        {"stale_timeout_ms", view.stale_timeout_ms},
        {"stale", view.stale},
        {"pose_available", view.pose_available},
        {"model_inference_ran", view.model_inference_ran},
        {"pose_source", view.pose_source},
        {"freshness", view.freshness},
        {"reason", view.reason},
        {"confidence", view.confidence},
        {"keypoints", keypoints},
        {"keypoint_count", present_count},
        {"bbox", bbox},
        {"skeleton", skeleton}
    };
}

nlohmann::json BodyOverlayToJson(const bt::WebRuntimeBodyOverlay& overlay) {
    return {
        {"camera_a", BodyOverlayViewToJson(overlay.camera_a)},
        {"camera_b", BodyOverlayViewToJson(overlay.camera_b)}
    };
}

nlohmann::json WallRectangleToJson(const bt::WallRectangleCalibration& w) {
    nlohmann::json corners = nlohmann::json::array();
    for (const auto& p : w.image_corners) {
        corners.push_back({{"x", p.x}, {"y", p.y}});
    }
    return {
        {"valid", w.valid},
        {"image_width", w.image_width},
        {"image_height", w.image_height},
        {"source", w.source},
        {"confidence", w.confidence},
        {"reason", w.reason},
        {"image_corners", corners},
        {"rectangle_width_m", w.rectangle_width_m},
        {"rectangle_height_m", w.rectangle_height_m},
        {"rectangle_aspect_ratio", w.rectangle_aspect_ratio},
        {"wall_homography_valid", w.wall_homography_valid},
        {"wall_from_image", Mat3ToJson(w.wall_from_image)},
        {"image_from_wall", Mat3ToJson(w.image_from_wall)},
        {"homography_reprojection_error_px", w.homography_reprojection_error_px},
        {"metric_scale_valid", w.metric_scale_valid},
        {"metric_scale_confidence", w.metric_scale_confidence},
        {"wall_orientation_valid", w.wall_orientation_valid},
        {"wall_right_camera", Vec3ToJson(w.wall_right_camera)},
        {"wall_down_camera", Vec3ToJson(w.wall_down_camera)},
        {"wall_normal_camera", Vec3ToJson(w.wall_normal_camera)},
        {"wall_orientation_confidence", w.wall_orientation_confidence},
        {"wall_depth_valid", w.wall_depth_valid},
        {"wall_center_depth_m", w.wall_center_depth_m},
        {"wall_depth_confidence", w.wall_depth_confidence},
        {"usable_for_wall_homography", w.usable_for_wall_homography},
        {"usable_for_metric_scale", w.usable_for_metric_scale},
        {"usable_for_orientation", w.usable_for_orientation},
        {"usable_for_depth_assist", w.usable_for_depth_assist},
        {"usable_for_floor_plane", w.usable_for_floor_plane},
        {"usable_for_floor_homography", w.usable_for_floor_homography},
        {"applied_to_runtime", w.applied_to_runtime},
        {"capability_reason", w.capability_reason}
    };
}

nlohmann::json WallRectanglesToJson(const std::vector<bt::WallRectangleCalibration>& walls) {
    auto out = nlohmann::json::array();
    for (const auto& wall : walls) {
        if (wall.valid) {
            out.push_back(WallRectangleToJson(wall));
        }
    }
    return out;
}

nlohmann::json FloorGeometryByCameraToJson(const bt::CalibrationBundle& bundle) {
    const auto& camera_a = bundle.camera_a_floor_geometry.valid
        ? bundle.camera_a_floor_geometry
        : bundle.floor_geometry;
    return {
        {"camera_a", FloorGeometryToJson(camera_a)},
        {"camera_b", FloorGeometryToJson(bundle.camera_b_floor_geometry)}
    };
}

nlohmann::json WallRectanglesByCameraToJson(const bt::CalibrationBundle& bundle) {
    const auto& camera_a = !bundle.camera_a_wall_rectangles.empty()
        ? bundle.camera_a_wall_rectangles
        : bundle.wall_rectangles;
    return {
        {"camera_a", WallRectanglesToJson(camera_a)},
        {"camera_b", WallRectanglesToJson(bundle.camera_b_wall_rectangles)}
    };
}

bool FloorPlaneNearlyEqual(const bt::FloorPlane& a, const bt::FloorPlane& b) {
    return a.valid == b.valid &&
        std::abs(a.normal.x - b.normal.x) <= 1e-5f &&
        std::abs(a.normal.y - b.normal.y) <= 1e-5f &&
        std::abs(a.normal.z - b.normal.z) <= 1e-5f &&
        std::abs(a.distance - b.distance) <= 1e-5f;
}

std::string CalibrationFloorSource(const bt::CalibrationBundle& bundle) {
    if (bundle.floor_geometry.valid && bundle.floor_geometry.floor_plane.valid &&
        (!bundle.floor.valid || FloorPlaneNearlyEqual(bundle.floor, bundle.floor_geometry.floor_plane))) {
        return FloorGeometrySource(bundle.floor_geometry);
    }
    if (bundle.floor.valid) {
        return "manual";
    }
    return "nothing";
}

nlohmann::json CalibrationStateToJson(const bt::CalibrationBundle& bundle, const bt::CalibrationReadiness& readiness) {
    const bool plane_from_geometry = bundle.floor_geometry.floor_plane.valid &&
        bundle.floor.valid &&
        FloorPlaneNearlyEqual(bundle.floor, bundle.floor_geometry.floor_plane);
    return {
        {"tracking_ready", readiness.tracking_ready},
        {"summary", readiness.summary},
        {"floor_source", CalibrationFloorSource(bundle)},
        {"floor_plane", FloorPlaneToJson(bundle.floor)},
        {"floor_plane_from_generated_geometry", plane_from_geometry},
        {"floor_geometry", FloorGeometryToJson(bundle.floor_geometry)},
        {"wall_rectangles", WallRectanglesToJson(bundle.wall_rectangles)},
        {"floor_geometry_by_camera", FloorGeometryByCameraToJson(bundle)},
        {"wall_rectangles_by_camera", WallRectanglesByCameraToJson(bundle)}
    };
}

nlohmann::json FloorSeamLineToJson(const bt::FloorSeamLine2D& line) {
    nlohmann::json samples = nlohmann::json::array();
    for (const auto& p : line.samples) {
        samples.push_back(Vec2ToJson(p));
    }
    return {
        {"a", Vec2ToJson(line.a)},
        {"b", Vec2ToJson(line.b)},
        {"strength", line.strength},
        {"sample_count", static_cast<int>(line.samples.size())},
        {"samples", samples}
    };
}

nlohmann::json FloorSeamCandidateToJson(const bt::FloorSeamCandidateDebug& c) {
    return {
        {"line", FloorSeamLineToJson(c.line)},
        {"angle_rad", c.angle_rad},
        {"rho_px", c.rho_px},
        {"accepted", c.accepted},
        {"reason", c.reason}
    };
}

nlohmann::json FloorFamilyEstimateToJson(const bt::FloorSeamFamilyEstimate& f) {
    nlohmann::json candidates = nlohmann::json::array();
    for (const auto& c : f.candidates) {
        candidates.push_back(FloorSeamCandidateToJson(c));
    }
    return {
        {"valid", f.valid},
        {"confidence", f.confidence},
        {"orientation_rad", f.orientation_rad},
        {"spacing_px", f.spacing_px},
        {"reference_rho_px", f.reference_rho_px},
        {"accepted_line_count", f.accepted_line_count},
        {"rejected_line_count", f.rejected_line_count},
        {"reason", f.reason},
        {"candidates", candidates}
    };
}

nlohmann::json FloorGeometryDetectionDebugToJson(const bt::FloorGeometryDetectionDebug& debug) {
    nlohmann::json candidates = nlohmann::json::array();
    for (const auto& c : debug.candidates) {
        candidates.push_back(FloorSeamCandidateToJson(c));
    }
    nlohmann::json rejected = nlohmann::json::array();
    for (const auto& f : debug.rejected_families) {
        rejected.push_back(FloorFamilyEstimateToJson(f));
    }
    return {
        {"calibration", FloorGeometryToJson(debug.calibration)},
        {"candidates", candidates},
        {"rejected_families", rejected}
    };
}

nlohmann::json QuatToJson(const bt::Quatf& q) {
    return {q.x, q.y, q.z, q.w};
}

bool TrackerSpaceNumbersValid(const bt::OscConfig& cfg) {
    const auto& p = cfg.tracker_space_position_offset;
    const auto& q = cfg.tracker_space_rotation;
    const float len2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    const bool base_valid = std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
        std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w) &&
        std::isfinite(len2) && len2 >= 1e-12f &&
        std::isfinite(cfg.tracker_space_scale) && cfg.tracker_space_scale > 0.0f;
    if (!base_valid) {
        return false;
    }
    return std::all_of(cfg.tracker_space_role_offsets.begin(), cfg.tracker_space_role_offsets.end(), [](const bt::Vec3f& offset) {
        return std::isfinite(offset.x) && std::isfinite(offset.y) && std::isfinite(offset.z);
    });
}

std::string TrackerSpaceValidationMessage(const bt::OscConfig& cfg) {
    const auto& p = cfg.tracker_space_position_offset;
    const auto& q = cfg.tracker_space_rotation;
    const float len2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        return "tracker-space position offset must contain finite x/y/z numbers";
    }
    if (!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w) ||
        !std::isfinite(len2) || len2 < 1e-12f) {
        return "tracker-space rotation must be a non-zero finite quaternion";
    }
    if (!std::isfinite(cfg.tracker_space_scale) || cfg.tracker_space_scale <= 0.0f) {
        return "tracker-space scale must be a positive finite number";
    }
    if (!cfg.tracker_space_transform_valid) {
        return "tracker-space transform is not marked valid; complete manual/json alignment before enabling OSC";
    }
    return "camera-to-VR tracker-space transform is valid";
}

std::string TrackerSpaceStatus(const bt::OscConfig& cfg) {
    if (!TrackerSpaceNumbersValid(cfg)) {
        return "invalid";
    }
    return cfg.tracker_space_transform_valid ? "valid" : "pending";
}

nlohmann::json TrackerSpaceStateToJson(const bt::AppConfig& cfg, const bt::SteamVrAlignmentStatus* alignment = nullptr) {
    std::string status = TrackerSpaceStatus(cfg.osc);
    std::string source = cfg.osc.tracker_space_source.empty()
        ? (cfg.hmd.mode == "json_file" ? "manual_json_file" : "manual")
        : cfg.osc.tracker_space_source;
    std::string reason = TrackerSpaceValidationMessage(cfg.osc);
    bool valid = status == "valid";
    bool osc_blocked = cfg.osc.enabled && status != "valid";

    if (alignment) {
        source = bt::ToString(alignment->active_transform_source);
        if (alignment->stale && alignment->manual_fallback_active) {
            status = bt::ManualTrackerSpaceFallbackAvailable(cfg.osc) ? "valid" : "pending";
            valid = status == "valid";
            reason = "SteamVR controller alignment is stale; preserved manual tracker-space fallback is active";
        } else if (alignment->stale) {
            valid = bt::SteamVrAlignmentManager::ActiveTransformIsHonest(cfg.osc, true);
            status = valid ? "degraded" : "stale";
            reason = alignment->reason.empty() ? "alignment_stale" : alignment->reason;
        } else if (alignment->active_transform_source == bt::TrackerSpaceSource::SteamVrController) {
            status = cfg.osc.tracker_space_transform_valid ? "valid" : alignment->state;
            valid = cfg.osc.tracker_space_transform_valid;
            reason = alignment->state == "failed"
                ? "last SteamVR solve failed; previous non-stale controller alignment is still active"
                : (alignment->reason.empty() ? cfg.osc.steamvr_alignment_reason : alignment->reason);
        } else if (alignment->manual_fallback_active &&
                   alignment->active_transform_source == bt::TrackerSpaceSource::Manual) {
            status = bt::ManualTrackerSpaceFallbackAvailable(cfg.osc) || cfg.osc.tracker_space_transform_valid ? "valid" : "pending";
            valid = status == "valid";
            reason = bt::ManualTrackerSpaceFallbackAvailable(cfg.osc)
                ? "manual tracker-space fallback active"
                : reason;
        }
        if (alignment->manual_fallback_active && alignment->active_transform_source == bt::TrackerSpaceSource::Manual) {
            bt::OscConfig fallback_cfg = cfg.osc;
            bt::ActivateManualTrackerSpaceFallback(fallback_cfg);
            osc_blocked = cfg.osc.enabled && !bt::SteamVrAlignmentManager::ActiveTransformIsHonest(fallback_cfg, false);
        } else {
            osc_blocked = cfg.osc.enabled && !bt::SteamVrAlignmentManager::ActiveTransformIsHonest(cfg.osc, alignment->stale);
        }
    }

    const bool display_manual_fallback = alignment && alignment->manual_fallback_active &&
        alignment->active_transform_source == bt::TrackerSpaceSource::Manual &&
        bt::ManualTrackerSpaceFallbackAvailable(cfg.osc);
    const bt::Vec3f display_offset = display_manual_fallback
        ? cfg.osc.manual_tracker_space_position_offset
        : cfg.osc.tracker_space_position_offset;
    const bt::Quatf display_rotation = display_manual_fallback
        ? cfg.osc.manual_tracker_space_rotation
        : cfg.osc.tracker_space_rotation;
    const float display_scale = display_manual_fallback
        ? cfg.osc.manual_tracker_space_scale
        : cfg.osc.tracker_space_scale;

    return {
        {"status", status},
        {"valid", valid},
        {"source", source},
        {"reason", reason},
        {"action", valid
            ? (status == "degraded"
                ? "OSC may continue sending the last finite tracker-space transform with degraded stale-alignment status"
                : "OSC may transform camera-world trackers into VRChat tracker space")
            : "Calibrate tracker space before enabling OSC; stale SteamVR controller alignment has no usable finite transform"},
        {"osc_blocked", osc_blocked},
        {"position_offset", Vec3ToJson(display_offset)},
        {"rotation", QuatToJson(display_rotation)},
        {"scale", display_scale},
        {"manual_fallback_valid", bt::ManualTrackerSpaceFallbackAvailable(cfg.osc)},
        {"manual_fallback_source", cfg.osc.manual_tracker_space_source},
        {"manual_fallback_position_offset", Vec3ToJson(cfg.osc.manual_tracker_space_position_offset)},
        {"manual_fallback_rotation", QuatToJson(cfg.osc.manual_tracker_space_rotation)},
        {"manual_fallback_scale", cfg.osc.manual_tracker_space_scale}
    };
}



bt::SteamVrAlignmentLandmark ParseSteamVrAlignmentLandmark(const std::string& raw) {
    if (raw == "left_foot" || raw == "left_foot_marker" || raw == "leftFoot") return bt::SteamVrAlignmentLandmark::LeftFoot;
    if (raw == "right_foot" || raw == "right_foot_marker" || raw == "rightFoot") return bt::SteamVrAlignmentLandmark::RightFoot;
    if (raw == "pelvis" || raw == "pelvis_marker") return bt::SteamVrAlignmentLandmark::Pelvis;
    if (raw == "floor" || raw == "floor_point") return bt::SteamVrAlignmentLandmark::Floor;
    if (raw == "forward" || raw == "forward_reference") return bt::SteamVrAlignmentLandmark::Forward;
    if (raw == "chest" || raw == "chest_marker") return bt::SteamVrAlignmentLandmark::Chest;
    if (raw == "left_elbow" || raw == "left_elbow_marker" || raw == "leftElbow") return bt::SteamVrAlignmentLandmark::LeftElbow;
    if (raw == "right_elbow" || raw == "right_elbow_marker" || raw == "rightElbow") return bt::SteamVrAlignmentLandmark::RightElbow;
    if (raw == "left_knee" || raw == "left_knee_marker" || raw == "leftKnee") return bt::SteamVrAlignmentLandmark::LeftKnee;
    if (raw == "right_knee" || raw == "right_knee_marker" || raw == "rightKnee") return bt::SteamVrAlignmentLandmark::RightKnee;
    return bt::SteamVrAlignmentLandmark::Pelvis;
}

bt::SteamVrControllerRole ParseSteamVrControllerRole(const std::string& raw) {
    if (raw == "left" || raw == "left_hand" || raw == "leftHand") return bt::SteamVrControllerRole::LeftHand;
    if (raw == "right" || raw == "right_hand" || raw == "rightHand") return bt::SteamVrControllerRole::RightHand;
    return bt::SteamVrControllerRole::Unknown;
}

bool Vec3NearlyEqual(const bt::Vec3f& a, const bt::Vec3f& b, float eps = 1e-6f) {
    return std::abs(a.x - b.x) <= eps && std::abs(a.y - b.y) <= eps && std::abs(a.z - b.z) <= eps;
}

bool QuatNearlyEqual(const bt::Quatf& a, const bt::Quatf& b, float eps = 1e-6f) {
    return std::abs(a.x - b.x) <= eps && std::abs(a.y - b.y) <= eps &&
        std::abs(a.z - b.z) <= eps && std::abs(a.w - b.w) <= eps;
}

bool TrackerSpaceEditedByUi(const bt::OscConfig& before, const bt::OscConfig& after) {
    return before.tracker_space_transform_valid != after.tracker_space_transform_valid ||
        !Vec3NearlyEqual(before.tracker_space_position_offset, after.tracker_space_position_offset) ||
        !QuatNearlyEqual(before.tracker_space_rotation, after.tracker_space_rotation) ||
        std::abs(before.tracker_space_scale - after.tracker_space_scale) > 1e-6f;
}

bool ShouldPollSteamVrProvider(const bt::OscConfig& osc_cfg, bool session_active = false) {
    if (session_active) {
        return true;
    }
    return osc_cfg.tracker_space_source == "steamvr_controller_alignment" ||
        osc_cfg.tracker_space_source == "steamvr_controller_alignment_stale";
}

bt::OscConfig RuntimeOscConfigForFrame(const bt::WebRuntimeState& web_state, const bt::AppConfig& startup_cfg, const bt::CalibrationBundle& calibration) {
    const auto snapshot = web_state.Snapshot();
    bt::OscConfig osc_cfg = snapshot.config_loaded ? snapshot.config.osc : startup_cfg.osc;
    if (!osc_cfg.tracker_space_transform_valid && bt::ManualTrackerSpaceFallbackAvailable(osc_cfg)) {
        bt::ActivateManualTrackerSpaceFallback(osc_cfg);
        osc_cfg.steamvr_alignment_reason = "manual_fallback_active";
    }

    const bool calibration_stale = bt::SteamVrAlignmentStale(osc_cfg, calibration);
    if (calibration_stale) {
        if (bt::ManualTrackerSpaceFallbackAvailable(osc_cfg)) {
            bt::ActivateManualTrackerSpaceFallback(osc_cfg);
            osc_cfg.steamvr_alignment_status = "stale";
            osc_cfg.steamvr_alignment_reason = "alignment_stale_manual_fallback_active";
        } else {
            osc_cfg.tracker_space_transform_valid = false;
            osc_cfg.tracker_space_source = "steamvr_controller_alignment_stale";
            osc_cfg.steamvr_alignment_status = "blocked";
            osc_cfg.steamvr_alignment_reason = "alignment_stale_no_manual_fallback";
        }
    } else if (osc_cfg.tracker_space_source == "steamvr_controller_alignment_stale" &&
               bt::ManualTrackerSpaceFallbackAvailable(osc_cfg)) {
        bt::ActivateManualTrackerSpaceFallback(osc_cfg);
        osc_cfg.steamvr_alignment_status = "stale";
        osc_cfg.steamvr_alignment_reason = "provider_stale_manual_fallback_active";
    }
    return osc_cfg;
}

bt::SteamVrTrackerBridgeConfig RuntimeSteamVrTrackerBridgeConfigForFrame(
    const bt::WebRuntimeState& web_state,
    const bt::AppConfig& startup_cfg) {
    const auto snapshot = web_state.Snapshot();
    return snapshot.config_loaded ? snapshot.config.steamvr_tracker_bridge : startup_cfg.steamvr_tracker_bridge;
}

bt::OscConfig ApplySteamVrRuntimeFreshnessToOscConfig(
    bt::OscConfig osc_cfg,
    const bt::SteamVrAlignmentStatus& status) {
    if (!status.source_known && osc_cfg.tracker_space_transform_valid) {
        osc_cfg.steamvr_alignment_status = "weak";
        osc_cfg.steamvr_alignment_reason = status.reason.empty() ? "unknown_tracker_space_source_label_using_numeric_transform" : status.reason;
        return osc_cfg;
    }
    const bool active_controller_source = osc_cfg.tracker_space_source == "steamvr_controller_alignment";
    if (!active_controller_source || (!status.stale && status.controller_alignment_fresh)) {
        return osc_cfg;
    }
    const std::string reason = status.stale_reason.empty()
        ? (status.reason.empty() ? "steamvr_alignment_not_fresh" : status.reason)
        : status.stale_reason;
    if (bt::ManualTrackerSpaceFallbackAvailable(osc_cfg)) {
        bt::ActivateManualTrackerSpaceFallback(osc_cfg);
        osc_cfg.steamvr_alignment_status = status.stale ? "stale" : "degraded";
        osc_cfg.steamvr_alignment_reason = reason + "_manual_fallback_active";
    } else if (status.stale) {
        osc_cfg.tracker_space_transform_valid = false;
        osc_cfg.tracker_space_source = "steamvr_controller_alignment_stale";
        osc_cfg.steamvr_alignment_status = "blocked";
        osc_cfg.steamvr_alignment_reason = reason + "_no_manual_fallback";
    } else {
        osc_cfg.tracker_space_source = "steamvr_controller_alignment_stale";
        osc_cfg.steamvr_alignment_status = "degraded";
        osc_cfg.steamvr_alignment_reason = reason + "_using_last_numeric_transform";
    }
    return osc_cfg;
}

bool PositiveFinite(double value) {
    return std::isfinite(value) && value > 0.0;
}

float Clamp01ForTelemetry(float value) {
    if (!std::isfinite(value)) {
        return 0.0f;
    }
    return std::clamp(value, 0.0f, 1.0f);
}

bool FloorAssistHasUsableReference(const bt::MonocularTrackingConfig& mono) {
    constexpr double kMinUsefulReferenceYPx = 20.0;
    if (!PositiveFinite(mono.floor_depth_reference_y_px)) {
        return false;
    }
    if (PositiveFinite(mono.floor_depth_reference_m)) {
        return true;
    }
    return PositiveFinite(mono.floor_depth_line_spacing_px) &&
        mono.floor_depth_reference_y_px > kMinUsefulReferenceYPx;
}

bool FloorAssistAppliesToSingleCameraPath(const bt::AppConfig& cfg, bool effective_monocular = false) {
    return cfg.tracking.mode == bt::TrackingMode::Monocular ||
        effective_monocular ||
        (cfg.tracking.mode == bt::TrackingMode::Stereo && cfg.tracking.stereo_monocular_fallback_enabled);
}

std::string FloorAssistConfigStatus(const bt::AppConfig& cfg, bool effective_monocular = false) {
    if (!FloorAssistAppliesToSingleCameraPath(cfg, effective_monocular) ||
        !cfg.tracking.monocular.floor_scale_assist_enabled) {
        return "disabled";
    }

    const auto& mono = cfg.tracking.monocular;
    if (!PositiveFinite(mono.floor_depth_line_spacing_m) ||
        !PositiveFinite(mono.floor_depth_line_spacing_px) ||
        !FloorAssistHasUsableReference(mono)) {
        return "invalid";
    }
    if (mono.floor_depth_line_spacing_px < 8.0f ||
        !std::isfinite(mono.floor_depth_confidence)) {
        return "invalid";
    }
    return cfg.tracking.mode == bt::TrackingMode::Stereo && !effective_monocular ? "standby" : "active";
}

bool AcceptedRuntimeProjectiveFloorGeometry(const bt::FloorGeometryCalibration& geometry) {
    return geometry.valid && geometry.homography_valid && geometry.metric_scale_confidence > 0.0f;
}

bool AcceptedRuntimeScalarFloorGeometry(const bt::FloorGeometryCalibration& geometry) {
    return geometry.valid &&
        geometry.family_a.valid &&
        geometry.family_a.metric_spacing_valid &&
        geometry.family_a.spacing_m > 0.0f &&
        geometry.family_a.spacing_px > 1.0f;
}

bool AcceptedRuntimeFloorGeometry(const bt::FloorGeometryCalibration& geometry) {
    // Accepted here means usable by the single-camera floor-depth path, not merely
    // present in calibration. A generic floor plane or a second-axis length by
    // itself is useful calibration context, but it is not a projective/spacing
    // depth model the monocular runtime can consume.
    return AcceptedRuntimeProjectiveFloorGeometry(geometry) ||
        AcceptedRuntimeScalarFloorGeometry(geometry);
}

std::string AcceptedFloorGeometrySource(const bt::FloorGeometryCalibration& geometry) {
    if (AcceptedRuntimeProjectiveFloorGeometry(geometry)) {
        return "floor_projective";
    }
    if (AcceptedRuntimeScalarFloorGeometry(geometry)) {
        return "floor_spacing";
    }
    return "none";
}

std::string FloorAssistRuntimeStatus(
    const bt::AppConfig& cfg,
    const bt::TrackingSolverTelemetry& solver,
    const bt::FloorGeometryCalibration& runtime_geometry) {
    const bool effective_monocular = solver.tracking_mode == bt::TrackingMode::Monocular;
    const auto& stereo = solver.preliminary_stereo;
    const bool runtime_floor_geometry_depth =
        stereo.monocular_scale_source == bt::MonocularScaleSource::FloorProjective ||
        stereo.monocular_scale_source == bt::MonocularScaleSource::FloorSpacing ||
        stereo.monocular_scale_source == bt::MonocularScaleSource::WallDepth ||
        stereo.floor_geometry_used;
    const bool accepted_runtime_geometry = AcceptedRuntimeFloorGeometry(runtime_geometry);

    const std::string config_status = FloorAssistConfigStatus(cfg, effective_monocular);
    if (!FloorAssistAppliesToSingleCameraPath(cfg, effective_monocular)) {
        return config_status;
    }
    if (cfg.tracking.mode != bt::TrackingMode::Monocular && !effective_monocular) {
        return accepted_runtime_geometry ? "standby" : config_status;
    }

    if (runtime_floor_geometry_depth || accepted_runtime_geometry) {
        return "active";
    }
    if (config_status == "disabled" || config_status == "invalid") {
        return config_status;
    }
    return stereo.monocular_scale_source == bt::MonocularScaleSource::FloorSpacing ? "active" : "inactive";
}

const bt::TrackingSolverTelemetry& EffectiveSolverTelemetry(const bt::DebugSnapshot& debug) {
    if (debug.solver.used_hmd || debug.solver.degraded || !debug.solver.reason.empty()) {
        return debug.solver;
    }
    return debug.tracking.solver;
}

nlohmann::json FloorAssistStateToJson(const bt::AppConfig& cfg, const bt::DebugSnapshot& debug) {
    const auto& solver = EffectiveSolverTelemetry(debug);
    const auto& stereo = solver.preliminary_stereo;
    const bool effective_monocular = solver.tracking_mode == bt::TrackingMode::Monocular;
    const bool runtime_floor_geometry_depth =
        stereo.monocular_scale_source == bt::MonocularScaleSource::FloorProjective ||
        stereo.monocular_scale_source == bt::MonocularScaleSource::FloorSpacing ||
        stereo.monocular_scale_source == bt::MonocularScaleSource::WallDepth ||
        stereo.floor_geometry_used;
    const bool accepted_runtime_geometry = AcceptedRuntimeFloorGeometry(debug.tracking.floor_geometry);
    const std::string accepted_source = AcceptedFloorGeometrySource(debug.tracking.floor_geometry);
    const std::string used_source = bt::ToString(stereo.monocular_scale_source);
    const std::string source = used_source != "none" ? used_source : accepted_source;
    const float confidence = runtime_floor_geometry_depth
        ? stereo.monocular_floor_assist_confidence
        : (accepted_runtime_geometry ? Clamp01ForTelemetry(debug.tracking.floor_geometry.metric_scale_confidence)
                                     : stereo.monocular_floor_assist_confidence);
    return {
        {"enabled", FloorAssistAppliesToSingleCameraPath(cfg, effective_monocular) &&
            (cfg.tracking.monocular.floor_scale_assist_enabled || runtime_floor_geometry_depth || accepted_runtime_geometry)},
        {"applies_to_stereo_fallback", cfg.tracking.mode == bt::TrackingMode::Stereo &&
            cfg.tracking.stereo_monocular_fallback_enabled &&
            (cfg.tracking.monocular.floor_scale_assist_enabled || runtime_floor_geometry_depth || accepted_runtime_geometry)},
        {"active_single_camera_path", effective_monocular},
        {"status", FloorAssistRuntimeStatus(cfg, solver, debug.tracking.floor_geometry)},
        {"config_status", FloorAssistConfigStatus(cfg, effective_monocular)},
        {"effective_monocular", effective_monocular},
        {"source", source},
        {"used_source", used_source},
        {"accepted_source", accepted_source},
        {"depth_m", stereo.monocular_floor_assist_depth_m},
        {"confidence", confidence},
        {"floor_geometry_accepted", accepted_runtime_geometry},
        {"floor_geometry_projective_accepted", AcceptedRuntimeProjectiveFloorGeometry(debug.tracking.floor_geometry)},
        {"floor_geometry_used", stereo.floor_geometry_used},
        {"floor_geometry_confidence", accepted_runtime_geometry
            ? Clamp01ForTelemetry(debug.tracking.floor_geometry.metric_scale_confidence)
            : stereo.floor_geometry_confidence},
        {"floor_geometry_family_count", accepted_runtime_geometry
            ? debug.tracking.floor_geometry.family_count
            : stereo.floor_geometry_family_count},
        {"physical_spacing_m", cfg.tracking.monocular.floor_depth_line_spacing_m},
        {"marked_spacing_px", cfg.tracking.monocular.floor_depth_line_spacing_px},
        {"reference_y_px", cfg.tracking.monocular.floor_depth_reference_y_px},
        {"reference_depth_m", cfg.tracking.monocular.floor_depth_reference_m}
    };
}


nlohmann::json BodyCalibrationQualityToJson(const bt::BodyCalibrationQuality& q) {
    return {
        {"pelvis_width", q.pelvis_width},
        {"left_femur", q.left_femur},
        {"right_femur", q.right_femur},
        {"left_tibia", q.left_tibia},
        {"right_tibia", q.right_tibia},
        {"left_foot_length", q.left_foot_length},
        {"right_foot_length", q.right_foot_length},
        {"standing_hmd_to_pelvis", q.standing_hmd_to_pelvis},
        {"overall", q.overall},
        {"sample_count", q.sample_count},
        {"source", q.source}
    };
}

nlohmann::json BodyCalibrationToJson(const bt::BodyCalibration& body) {
    return {
        {"standing_neutral_valid", body.standing_neutral_valid},
        {"pelvis_width", body.pelvis_width},
        {"left_femur", body.left_femur},
        {"right_femur", body.right_femur},
        {"left_tibia", body.left_tibia},
        {"right_tibia", body.right_tibia},
        {"left_foot_length", body.left_foot_length},
        {"right_foot_length", body.right_foot_length},
        {"standing_hmd_to_pelvis", Vec3ToJson(body.standing_hmd_to_pelvis)},
        {"quality", BodyCalibrationQualityToJson(body.quality)}
    };
}

nlohmann::json BodyCalibrationTelemetryToJson(const bt::BodyCalibrationTelemetry& telemetry) {
    return {
        {"enabled", telemetry.enabled},
        {"auto_persist", telemetry.auto_persist},
        {"complete", telemetry.complete},
        {"saved_this_frame", telemetry.saved_this_frame},
        {"persisted", telemetry.persisted},
        {"persist_pending", telemetry.persist_pending},
        {"used_stereo", telemetry.used_stereo},
        {"used_monocular_floor_scale", telemetry.used_monocular_floor_scale},
        {"accumulated_seconds", telemetry.accumulated_seconds},
        {"accepted_samples", telemetry.accepted_samples},
        {"overall_confidence", telemetry.overall_confidence},
        {"reason", telemetry.reason},
        {"persist_status", telemetry.persist_status},
        {"persist_error", telemetry.persist_error},
        {"body", BodyCalibrationToJson(telemetry.body)}
    };
}

nlohmann::json RoomDepthMapTelemetryToJson(const bt::RoomDepthMapTelemetry& room) {
    return {
        {"state", room.state},
        {"coverage", room.coverage},
        {"accepted_frames", room.accepted_frames},
        {"rejected_frames", room.rejected_frames},
        {"mean_variance_m2", room.mean_variance_m2},
        {"last_rejection_reason", room.last_rejection_reason}
    };
}

nlohmann::json FloorAssistTelemetryToJson(const bt::TrackingSolverTelemetry& solver) {
    const auto& stereo = solver.preliminary_stereo;
    const bool depth_assist =
        stereo.monocular_scale_source == bt::MonocularScaleSource::FloorSpacing ||
        stereo.monocular_scale_source == bt::MonocularScaleSource::FloorProjective ||
        stereo.monocular_scale_source == bt::MonocularScaleSource::WallDepth;
    const std::string status = depth_assist ? "active" : "inactive";
    return {
        {"status", solver.tracking_mode == bt::TrackingMode::Monocular ? status : "disabled"},
        {"source", bt::ToString(stereo.monocular_scale_source)},
        {"depth_m", stereo.monocular_floor_assist_depth_m},
        {"confidence", stereo.monocular_floor_assist_confidence}
    };
}

nlohmann::json DepthTelemetryToJson(
    const bt::TrackingSolverTelemetry& solver,
    const bt::FloorGeometryCalibration& floor_geometry) {
    const auto& stereo = solver.preliminary_stereo;
    const bool accepted = AcceptedRuntimeFloorGeometry(floor_geometry);
    const std::string used_source = bt::ToString(stereo.monocular_scale_source);
    const std::string accepted_source = AcceptedFloorGeometrySource(floor_geometry);
    nlohmann::json floor_assist = FloorAssistTelemetryToJson(solver);
    if (accepted) {
        const std::string prior_status = floor_assist.value("status", std::string("inactive"));
        floor_assist["status"] = prior_status == "disabled" ? "standby" : "active";
    }
    if (accepted && floor_assist.value("source", std::string("none")) == "none") {
        floor_assist["source"] = accepted_source;
    }
    floor_assist["used_source"] = used_source;
    floor_assist["accepted_source"] = accepted_source;
    floor_assist["floor_geometry_accepted"] = accepted;
    floor_assist["floor_geometry_projective_accepted"] = AcceptedRuntimeProjectiveFloorGeometry(floor_geometry);
    return {
        {"source", bt::ToString(solver.depth_source)},
        {"confidence", stereo.mean_confidence},
        {"foot_confidence", stereo.foot_mean_confidence},
        {"camera_a_quality", stereo.camera_a_mean_quality},
        {"camera_b_quality", stereo.camera_b_mean_quality},
        {"camera_a_present_keypoints", stereo.camera_a_present_keypoints},
        {"camera_b_present_keypoints", stereo.camera_b_present_keypoints},
        {"camera_a_usable_keypoints", stereo.camera_a_usable_keypoints},
        {"camera_b_usable_keypoints", stereo.camera_b_usable_keypoints},
        {"camera_a_age_scale", stereo.camera_a_age_scale},
        {"camera_b_age_scale", stereo.camera_b_age_scale},
        {"epipolar_geometry_valid", stereo.epipolar_geometry_valid},
        {"epipolar_status", stereo.epipolar_status},
        {"epipolar_checked_count", stereo.epipolar_checked_count},
        {"epipolar_hard_mismatch_count", stereo.epipolar_hard_mismatch_count},
        {"epipolar_pair_rejected_count", stereo.epipolar_pair_rejected_count},
        {"epipolar_degraded_pair_softened_count", stereo.epipolar_degraded_pair_softened_count},
        {"mean_epipolar_error_px", stereo.mean_epipolar_error_px},
        {"mean_epipolar_error_px_isotropic_heuristic", stereo.mean_epipolar_error_px_isotropic},
        {"mean_epipolar_error_px_anisotropic", stereo.mean_epipolar_error_px_anisotropic},
        {"mean_epipolar_error_normalized", stereo.mean_epipolar_error_normalized},
        {"mean_epipolar_confidence", stereo.mean_epipolar_confidence},
        {"inferred_count", stereo.inferred_depth_count},
        {"triangulated_count", stereo.triangulated_count},
        {"mean_inferred_depth_m", stereo.mean_inferred_depth_m},
        {"mean_triangulation_condition_number", stereo.mean_triangulation_condition_number},
        {"mean_triangulation_strength_ratio", stereo.mean_triangulation_strength_ratio},
        {"mean_triangulation_null_residual", stereo.mean_triangulation_null_residual},
        {"measurement_uncertainty_count", stereo.measurement_uncertainty_count},
        {"mean_measurement_position_stddev_m", stereo.mean_measurement_position_stddev_m},
        {"mean_measurement_depth_stddev_m", stereo.mean_measurement_depth_stddev_m},
        {"mean_measurement_baseline_to_depth_ratio", stereo.mean_measurement_baseline_to_depth_ratio},
        {"solver_uncertainty_weighted_count", stereo.solver_uncertainty_weighted_count},
        {"solver_uncertainty_valid_count", stereo.solver_uncertainty_valid_count},
        {"solver_uncertainty_conservative_fallback_count", stereo.solver_uncertainty_conservative_fallback_count},
        {"solver_temporal_process_noise_count", stereo.solver_temporal_process_noise_count},
        {"mean_solver_lateral_weight_scale", stereo.mean_solver_lateral_weight_scale},
        {"mean_solver_depth_weight_scale", stereo.mean_solver_depth_weight_scale},
        {"mean_solver_observation_confidence_ceiling", stereo.mean_solver_observation_confidence_ceiling},
        {"mean_solver_temporal_process_stddev_m", stereo.mean_solver_temporal_process_stddev_m},
        {"scale_source", bt::ToString(stereo.monocular_scale_source)},
        {"camera_a_geometry_used", stereo.camera_a_geometry_used},
        {"camera_b_geometry_used", stereo.camera_b_geometry_used},
        {"stereo_geometry_constraints_used", stereo.stereo_geometry_constraints_used},
        {"stereo_geometry_confidence", stereo.stereo_geometry_confidence},
        {"geometry_stereo_status", stereo.geometry_stereo_status},
        {"floor_assist", floor_assist}
    };
}


nlohmann::json EpipolarStereoTelemetryToJson(const bt::BodySolveStereoTelemetry& stereo) {
    return {
        {"geometry_valid", stereo.epipolar_geometry_valid},
        {"status", stereo.epipolar_status},
        {"checked_count", stereo.epipolar_checked_count},
        {"hard_mismatch_count", stereo.epipolar_hard_mismatch_count},
        {"pair_rejected_count", stereo.epipolar_pair_rejected_count},
        {"degraded_pair_softened_count", stereo.epipolar_degraded_pair_softened_count},
        {"mean_error_px", stereo.mean_epipolar_error_px},
        {"mean_error_px_isotropic_heuristic", stereo.mean_epipolar_error_px_isotropic},
        {"mean_error_px_anisotropic", stereo.mean_epipolar_error_px_anisotropic},
        {"mean_error_normalized", stereo.mean_epipolar_error_normalized},
        {"mean_confidence", stereo.mean_epipolar_confidence}
    };
}

nlohmann::json StereoJointTelemetryToJson(
    const bt::BodySolveJointTriangulationTelemetry& joint,
    std::size_t index) {
    const auto id = static_cast<bt::KeypointId>(index);
    return {
        {"id", index},
        {"name", bt::ToString(id)},
        {"camera_a_present", joint.camera_a_present},
        {"camera_b_present", joint.camera_b_present},
        {"camera_a_confidence", joint.camera_a_confidence},
        {"camera_b_confidence", joint.camera_b_confidence},
        {"camera_a_weight", joint.camera_a_weight},
        {"camera_b_weight", joint.camera_b_weight},
        {"camera_a_quality", joint.camera_a_quality},
        {"camera_b_quality", joint.camera_b_quality},
        {"temporal_confidence", joint.temporal_confidence},
        {"epipolar_available", joint.epipolar_available},
        {"epipolar_checked", joint.epipolar_checked},
        {"epipolar_error_px", joint.epipolar_error_px},
        {"epipolar_error_px_isotropic_heuristic", joint.epipolar_error_px_isotropic},
        {"epipolar_error_px_anisotropic", joint.epipolar_error_px_anisotropic},
        {"epipolar_error_normalized", joint.epipolar_error_normalized},
        {"epipolar_confidence", joint.epipolar_confidence},
        {"epipolar_reliability_term", joint.epipolar_reliability_term},
        {"epipolar_hard_mismatch", joint.epipolar_hard_mismatch},
        {"epipolar_pair_rejected", joint.epipolar_pair_rejected},
        {"epipolar_degraded_pair_softened", joint.epipolar_degraded_pair_softened},
        {"epipolar_reason", bt::ToString(joint.epipolar_reason)},
        {"epipolar_coordinate_space", bt::ToString(joint.epipolar_coordinate_space)},
        {"used_temporal_depth", joint.used_temporal_depth},
        {"fallback_used", joint.fallback_used},
        {"evidence_source", bt::ToString(joint.evidence_source)},
        {"triangulated", joint.triangulated},
        {"depth_inferred", joint.depth_inferred},
        {"depth_source", bt::ToString(joint.depth_source)},
        {"world", Vec3ToJson(joint.world)},
        {"anchor_raw_world_present", joint.anchor_raw_world_present},
        {"anchor_raw_world", Vec3ToJson(joint.anchor_raw_world)},
        {"anchor_correction_applied", joint.anchor_correction_applied},
        {"anchor_corrected_world", Vec3ToJson(joint.anchor_corrected_world)},
        {"anchor_corrected_depth_m", joint.anchor_corrected_depth_m},
        {"anchor_correction_rejection_reason", joint.anchor_correction_rejection_reason},
        {"confidence", joint.confidence},
        {"reprojection_error_a_px", joint.reprojection_error_a_px},
        {"reprojection_error_b_px", joint.reprojection_error_b_px},
        {"mean_reprojection_error_px", joint.mean_reprojection_error_px},
        {"triangulation_condition_number", joint.triangulation_condition_number},
        {"triangulation_strength_ratio", joint.triangulation_strength_ratio},
        {"triangulation_null_residual", joint.triangulation_null_residual},
        {"measurement_uncertainty_valid", joint.measurement_uncertainty_valid},
        {"measurement_baseline_m", joint.measurement_baseline_m},
        {"measurement_mean_depth_m", joint.measurement_mean_depth_m},
        {"measurement_baseline_to_depth_ratio", joint.measurement_baseline_to_depth_ratio},
        {"measurement_effective_focal_px", joint.measurement_effective_focal_px},
        {"measurement_reprojection_sigma_px", joint.measurement_reprojection_sigma_px},
        {"measurement_epipolar_sigma_px", joint.measurement_epipolar_sigma_px},
        {"measurement_image_noise_sigma_px", joint.measurement_image_noise_sigma_px},
        {"measurement_conditioning_scale", joint.measurement_conditioning_scale},
        {"measurement_unclamped_lateral_stddev_m", joint.measurement_unclamped_lateral_stddev_m},
        {"measurement_unclamped_depth_stddev_m", joint.measurement_unclamped_depth_stddev_m},
        {"measurement_unclamped_position_variance_m2", joint.measurement_unclamped_position_variance_m2},
        {"measurement_lateral_stddev_m", joint.measurement_lateral_stddev_m},
        {"measurement_depth_stddev_m", joint.measurement_depth_stddev_m},
        {"measurement_position_stddev_m", joint.measurement_position_stddev_m},
        {"measurement_position_variance_m2", joint.measurement_position_variance_m2},
        {"solver_uncertainty_weighted", joint.solver_uncertainty_weighted},
        {"solver_uncertainty_valid", joint.solver_uncertainty_valid},
        {"solver_uncertainty_conservative_fallback", joint.solver_uncertainty_conservative_fallback},
        {"solver_temporal_process_noise_applied", joint.solver_temporal_process_noise_applied},
        {"solver_lateral_weight_scale", joint.solver_lateral_weight_scale},
        {"solver_depth_weight_scale", joint.solver_depth_weight_scale},
        {"solver_observation_confidence_ceiling", joint.solver_observation_confidence_ceiling},
        {"solver_temporal_process_stddev_m", joint.solver_temporal_process_stddev_m},
        {"estimated_depth_m", joint.estimated_depth_m},
        {"foot_contact_confidence", joint.foot_contact_confidence}
    };
}

nlohmann::json StereoTelemetryToJson(const bt::BodySolveStereoTelemetry& stereo) {
    nlohmann::json joints = nlohmann::json::array();
    for (std::size_t i = 0; i < stereo.joints.size(); ++i) {
        joints.push_back(StereoJointTelemetryToJson(stereo.joints[i], i));
    }
    return {
        {"tracking_mode", bt::ToString(stereo.tracking_mode)},
        {"depth_source", bt::ToString(stereo.depth_source)},
        {"triangulated_count", stereo.triangulated_count},
        {"left_foot_triangulated_count", stereo.left_foot_triangulated_count},
        {"right_foot_triangulated_count", stereo.right_foot_triangulated_count},
        {"inferred_depth_count", stereo.inferred_depth_count},
        {"camera_a_present_keypoints", stereo.camera_a_present_keypoints},
        {"camera_b_present_keypoints", stereo.camera_b_present_keypoints},
        {"camera_a_usable_keypoints", stereo.camera_a_usable_keypoints},
        {"camera_b_usable_keypoints", stereo.camera_b_usable_keypoints},
        {"camera_a_mean_quality", stereo.camera_a_mean_quality},
        {"camera_b_mean_quality", stereo.camera_b_mean_quality},
        {"camera_a_age_scale", stereo.camera_a_age_scale},
        {"camera_b_age_scale", stereo.camera_b_age_scale},
        {"epipolar", EpipolarStereoTelemetryToJson(stereo)},
        {"mean_inferred_depth_m", stereo.mean_inferred_depth_m},
        {"mean_confidence", stereo.mean_confidence},
        {"foot_mean_confidence", stereo.foot_mean_confidence},
        {"mean_reprojection_error_px", stereo.mean_reprojection_error_px},
        {"mean_triangulation_condition_number", stereo.mean_triangulation_condition_number},
        {"mean_triangulation_strength_ratio", stereo.mean_triangulation_strength_ratio},
        {"mean_triangulation_null_residual", stereo.mean_triangulation_null_residual},
        {"measurement_uncertainty_count", stereo.measurement_uncertainty_count},
        {"mean_measurement_position_stddev_m", stereo.mean_measurement_position_stddev_m},
        {"mean_measurement_depth_stddev_m", stereo.mean_measurement_depth_stddev_m},
        {"mean_measurement_baseline_to_depth_ratio", stereo.mean_measurement_baseline_to_depth_ratio},
        {"solver_uncertainty_weighted_count", stereo.solver_uncertainty_weighted_count},
        {"solver_uncertainty_valid_count", stereo.solver_uncertainty_valid_count},
        {"solver_uncertainty_conservative_fallback_count", stereo.solver_uncertainty_conservative_fallback_count},
        {"solver_temporal_process_noise_count", stereo.solver_temporal_process_noise_count},
        {"mean_solver_lateral_weight_scale", stereo.mean_solver_lateral_weight_scale},
        {"mean_solver_depth_weight_scale", stereo.mean_solver_depth_weight_scale},
        {"mean_solver_observation_confidence_ceiling", stereo.mean_solver_observation_confidence_ceiling},
        {"mean_solver_temporal_process_stddev_m", stereo.mean_solver_temporal_process_stddev_m},
        {"foot_mean_reprojection_error_px", stereo.foot_mean_reprojection_error_px},
        {"max_foot_reprojection_error_px", stereo.max_foot_reprojection_error_px},
        {"left_foot_contact_confidence", stereo.left_foot_contact_confidence},
        {"right_foot_contact_confidence", stereo.right_foot_contact_confidence},
        {"monocular_scale_source", bt::ToString(stereo.monocular_scale_source)},
        {"monocular_floor_assist_depth_m", stereo.monocular_floor_assist_depth_m},
        {"monocular_floor_assist_confidence", stereo.monocular_floor_assist_confidence},
        {"floor_geometry_used", stereo.floor_geometry_used},
        {"floor_geometry_confidence", stereo.floor_geometry_confidence},
        {"floor_geometry_family_count", stereo.floor_geometry_family_count},
        {"floor_distortion_correction_used", stereo.floor_distortion_correction_used},
        {"floor_camera_orientation_used", stereo.floor_camera_orientation_used},
        {"camera_a_geometry_used", stereo.camera_a_geometry_used},
        {"camera_b_geometry_used", stereo.camera_b_geometry_used},
        {"stereo_geometry_constraints_used", stereo.stereo_geometry_constraints_used},
        {"stereo_geometry_confidence", stereo.stereo_geometry_confidence},
        {"geometry_stereo_status", stereo.geometry_stereo_status},
        {"joints", joints}
    };
}

nlohmann::json TrackerEvidenceToJson(const bt::TrackerEvidence& evidence) {
    return {
        {"source", bt::ToString(evidence.source)},
        {"signal_kind", bt::ToString(bt::EffectiveSignalKind(evidence))},
        {"direct_confidence", evidence.direct_confidence},
        {"support_confidence", evidence.support_confidence},
        {"anchor_held", evidence.anchor_held},
        {"degraded", evidence.degraded},
        {"stereo_fallback", evidence.stereo_fallback},
        {"manual", evidence.manual},
        {"stale_aged", evidence.stale_aged},
        {"valid", evidence.valid}
    };
}

nlohmann::json BodyStateJointToJson(const bt::BodyStateJoint& joint) {
    return {
        {"role", bt::ToString(joint.role)},
        {"valid", joint.valid},
        {"position", Vec3ToJson(joint.position)},
        {"velocity", Vec3ToJson(joint.velocity)},
        {"confidence", joint.confidence},
        {"visibility", bt::ToString(joint.visibility)},
        {"evidence", TrackerEvidenceToJson(joint.evidence)},
        {"depth_source", bt::ToString(joint.depth_source)},
        {"measured", joint.measured},
        {"predicted", joint.predicted},
        {"camera_a_present", joint.camera_a_present},
        {"camera_b_present", joint.camera_b_present},
        {"camera_a_confidence", joint.camera_a_confidence},
        {"camera_b_confidence", joint.camera_b_confidence},
        {"camera_a_weight", joint.camera_a_weight},
        {"camera_b_weight", joint.camera_b_weight},
        {"camera_a_quality", joint.camera_a_quality},
        {"camera_b_quality", joint.camera_b_quality},
        {"evidence_source", bt::ToString(joint.evidence_source)},
        {"triangulated", joint.triangulated},
        {"depth_inferred", joint.depth_inferred},
        {"reprojection_error_px", joint.reprojection_error_px},
        {"estimated_depth_m", joint.estimated_depth_m},
        {"contact_lock_strength", joint.contact_lock_strength},
        {"contact_support_confidence", joint.contact_support_confidence},
        {"solver_observation_weighted", joint.solver_observation_weighted},
        {"solver_observation_weight_scale", joint.solver_observation_weight_scale},
        {"solver_observation_confidence_ceiling", joint.solver_observation_confidence_ceiling},
        {"identity_confidence", joint.identity_confidence},
        {"reason", joint.reason}
    };
}

nlohmann::json BodyStateToJson(const bt::UnifiedBodyState& state) {
    auto roles = nlohmann::json::array();
    for (const auto role : bt::kBodyJointRoles) {
        roles.push_back(BodyStateJointToJson(state.roles[bt::BodyJointRoleIndex(role)]));
    }
    return {
        {"valid", state.valid},
        {"left_foot_contact", bt::ToString(state.left_foot_contact)},
        {"right_foot_contact", bt::ToString(state.right_foot_contact)},
        {"diagnostics", {
            {"active", state.diagnostics.active},
            {"degraded", state.diagnostics.degraded},
            {"triangulation_active", state.diagnostics.triangulation_active},
            {"tracking_mode_is_monocular", state.diagnostics.tracking_mode_is_monocular},
            {"stereo_fallback_active", state.diagnostics.stereo_fallback_active},
            {"monocular_fallback", state.diagnostics.monocular_fallback},
            {"left_right_identity_stable", state.diagnostics.left_right_identity_stable},
            {"left_right_identity_uncertain", state.diagnostics.left_right_identity_uncertain},
            {"occlusion_prediction_active", state.diagnostics.occlusion_prediction_active},
            {"contact_lock_active", state.diagnostics.contact_lock_active},
            {"floor_support_active", state.diagnostics.floor_support_active},
            {"body_calibration_valid", state.diagnostics.body_calibration_valid},
            {"latency_prediction_active", state.diagnostics.latency_prediction_active},
            {"latency_prediction_seconds", state.diagnostics.latency_prediction_seconds},
            {"triangulated_count", state.diagnostics.triangulated_count},
            {"inferred_depth_count", state.diagnostics.inferred_depth_count},
            {"predicted_joint_count", state.diagnostics.predicted_joint_count},
            {"measured_role_count", state.diagnostics.measured_role_count},
            {"anchored_role_count", state.diagnostics.anchored_role_count},
            {"degraded_role_count", state.diagnostics.degraded_role_count},
            {"manual_role_count", state.diagnostics.manual_role_count},
            {"stale_aged_role_count", state.diagnostics.stale_aged_role_count},
            {"invalid_role_count", state.diagnostics.invalid_role_count},
            {"low_confidence_role_count", state.diagnostics.low_confidence_role_count},
            {"mean_reprojection_error_px", state.diagnostics.mean_reprojection_error_px},
            {"role_output_confidence", state.diagnostics.role_output_confidence},
            {"identity_confidence", state.diagnostics.identity_confidence},
            {"left_contact_lock_strength", state.diagnostics.left_contact_lock_strength},
            {"right_contact_lock_strength", state.diagnostics.right_contact_lock_strength},
            {"tracking_mode", state.diagnostics.tracking_mode},
            {"depth_source", state.diagnostics.depth_source},
            {"reason", state.diagnostics.reason}
        }},
        {"roles", roles}
    };
}

void PutText(cv::Mat& canvas, const std::string& text, cv::Point origin, double scale, cv::Scalar color, int thickness = 1) {
    cv::putText(canvas, text, origin, cv::FONT_HERSHEY_SIMPLEX, scale, color, thickness, cv::LINE_AA);
}

std::string FitText(const std::string& text, int max_width, double scale, int thickness) {
    std::string out = text;
    int baseline = 0;
    while (!out.empty() && cv::getTextSize(out, cv::FONT_HERSHEY_SIMPLEX, scale, thickness, &baseline).width > max_width) {
        out.pop_back();
    }
    if (out.size() < text.size() && out.size() > 3) {
        out.resize(out.size() - 3);
        out += "...";
    }
    return out;
}

void DrawCard(cv::Mat& canvas, const cv::Rect& rect, const std::string& label, const std::string& value, const std::string& sub, cv::Scalar stripe) {
    const UiPalette p;
    cv::rectangle(canvas, rect, p.paper, cv::FILLED);
    cv::rectangle(canvas, rect, p.hair, 1);
    cv::line(canvas, rect.tl(), cv::Point(rect.x + rect.width, rect.y), p.line, 1);
    cv::rectangle(canvas, cv::Rect(rect.x, rect.y, 4, rect.height), stripe, cv::FILLED);
    PutText(canvas, Uppercase(label), {rect.x + 16, rect.y + 25}, 0.36, p.muted, 1);
    PutText(canvas, FitText(Uppercase(value), rect.width - 30, 0.86, 2), {rect.x + 16, rect.y + 62}, 0.86, p.ink, 2);
    if (!sub.empty()) {
        PutText(canvas, FitText(sub, rect.width - 30, 0.42, 1), {rect.x + 16, rect.y + 94}, 0.42, p.muted, 1);
    }
}

bool AnyValidTracker(const bt::TrackerPoseArray& trackers) {
    for (const auto& tracker : trackers) {
        if (tracker.valid) {
            return true;
        }
    }
    return false;
}

const bt::TrackerPoseArray& EffectiveTrackersForDashboard(const bt::DebugSnapshot& debug) {
    return AnyValidTracker(debug.trackers) ? debug.trackers : debug.tracking.trackers;
}

std::string TrackerRoleName(bt::TrackerRole role) {
    std::string out = bt::ToString(role);
    std::replace(out.begin(), out.end(), '_', ' ');
    std::transform(out.begin(), out.end(), out.begin(), [](unsigned char c) {
        return static_cast<char>(std::toupper(c));
    });
    return out;
}

void SetOscConfigDefaults(bt::DebugSnapshot& debug, const bt::OscConfig& config) {
    debug.osc_enabled = config.enabled;
    debug.osc_open = false;
    debug.osc_last_send_ok = true;
    debug.osc_status = config.enabled ? "closed" : "disabled";
    debug.osc_last_error.clear();
    debug.osc_open_attempts.clear();
    debug.osc_target_address = config.target_address;
    debug.osc_target_port = config.target_port;
    debug.osc_tracker_space_transform_valid = config.tracker_space_transform_valid;
    debug.osc_tracker_space_source = config.tracker_space_source;
    debug.osc_manual_tracker_space_fallback_valid = bt::ManualTrackerSpaceFallbackAvailable(config);
    debug.osc_manual_tracker_space_source = config.manual_tracker_space_source;
    debug.osc_sent_tracker_count = 0;
    debug.osc_skipped_tracker_count = 0;
    debug.osc_sent_message_count = 0;
    for (std::size_t i = 0; i < bt::kTrackerRoles.size(); ++i) {
        const bt::TrackerRole role = bt::kTrackerRoles[i];
        const int index = bt::DefaultVrchatTrackerIndex(role, config);
        debug.osc_role_indices[i] = index;
        debug.osc_role_configured[i] = index > 0;
        debug.osc_role_valid[i] = false;
        debug.osc_role_sent[i] = false;
        debug.osc_role_degraded[i] = false;
        debug.osc_role_reasons[i] = index > 0 ? "not_sent" : "unmapped";
        debug.osc_role_error_details[i].clear();
    }
}

std::string OscOpenAttemptDebugLine(const bt::OscOpenAttemptState& attempt) {
    std::string line = "#" + std::to_string(attempt.attempt) + " " + attempt.action;
    if (!attempt.address_family.empty()) {
        line += " " + attempt.address_family;
    }
    line += " socktype=" + std::to_string(attempt.socket_type);
    line += " proto=" + std::to_string(attempt.protocol);
    line += attempt.socket_created ? " socket=created" : " socket=not_created";
    line += attempt.connected ? " connected=yes" : " connected=no";
    if (attempt.error_code != 0) {
        line += " error=" + std::to_string(attempt.error_code);
    }
    if (!attempt.detail.empty()) {
        line += " detail=" + attempt.detail;
    }
    return line;
}

void CopyOscReportToDebug(bt::DebugSnapshot& debug, const bt::OscConfig& config, const bt::OscSendReport& report) {
    SetOscConfigDefaults(debug, config);
    debug.osc_enabled = report.enabled;
    debug.osc_open = report.open;
    debug.osc_last_send_ok = report.last_send_ok;
    debug.osc_status = report.status;
    debug.osc_last_error = report.last_error;
    debug.osc_open_attempts.clear();
    for (const auto& attempt : report.open_attempts) {
        debug.osc_open_attempts.push_back(OscOpenAttemptDebugLine(attempt));
    }
    debug.osc_sent_tracker_count = report.sent_tracker_count;
    debug.osc_skipped_tracker_count = report.skipped_tracker_count;
    debug.osc_sent_message_count = report.sent_message_count;
    for (std::size_t i = 0; i < report.roles.size(); ++i) {
        debug.osc_role_indices[i] = report.roles[i].tracker_index;
        debug.osc_role_configured[i] = report.roles[i].configured;
        debug.osc_role_valid[i] = report.roles[i].valid;
        debug.osc_role_sent[i] = report.roles[i].sent;
        debug.osc_role_degraded[i] = report.roles[i].degraded;
        debug.osc_role_reasons[i] = report.roles[i].reason;
        debug.osc_role_error_details[i] = report.roles[i].error_detail;
    }
}

bt::Status SendTrackersAndRecordOsc(bt::OscSender& osc, const bt::OscConfig& config, const bt::TrackerPoseArray& trackers, bt::DebugSnapshot& debug) {
    const auto osc_start = bt::NowQpc();
    const auto status = osc.SendTrackers(trackers);
    debug.osc_ms = bt::QpcDeltaSeconds(osc_start, bt::NowQpc()) * 1000.0;
    CopyOscReportToDebug(debug, config, osc.LastReport());
    return status;
}

void CopySteamVrBridgeReportToDebug(bt::DebugSnapshot& debug, const bt::SteamVrTrackerBridgeReport& report) {
    debug.steamvr_bridge_enabled = report.enabled;
    debug.steamvr_bridge_open = report.open;
    debug.steamvr_bridge_last_send_ok = report.last_send_ok;
    debug.steamvr_bridge_status = report.status;
    debug.steamvr_bridge_last_error = report.last_error;
    debug.steamvr_bridge_target_address = report.target_address;
    debug.steamvr_bridge_target_port = report.target_port;
    debug.steamvr_bridge_min_confidence = report.min_confidence;
    debug.steamvr_bridge_sent_tracker_count = report.sent_tracker_count;
    debug.steamvr_bridge_skipped_tracker_count = report.skipped_tracker_count;
    debug.steamvr_bridge_sent_message_count = report.sent_message_count;
    debug.steamvr_bridge_sequence = report.sequence;
    for (std::size_t i = 0; i < bt::kTrackerRoles.size(); ++i) {
        debug.steamvr_bridge_role_enabled[i] = report.role_enabled[i];
        debug.steamvr_bridge_role_valid[i] = report.role_valid[i];
        debug.steamvr_bridge_role_sent[i] = report.role_sent[i];
        debug.steamvr_bridge_role_degraded[i] = report.role_degraded[i];
        debug.steamvr_bridge_role_confidence[i] = report.role_confidence[i];
        debug.steamvr_bridge_role_reasons[i] = report.role_reasons[i];
    }
}

bt::Status SendTrackersAndRecordSteamVrBridge(
    bt::SteamVrTrackerBridgeSender& bridge,
    const bt::OscConfig& tracker_space,
    const bt::TrackerPoseArray& trackers,
    bt::DebugSnapshot& debug) {
    const auto status = bridge.SendTrackers(trackers, tracker_space, debug.timestamp_seconds);
    CopySteamVrBridgeReportToDebug(debug, bridge.LastReport());
    if (!status.ok() && debug.last_error.empty()) {
        debug.last_error = status.message;
    }
    return status;
}

nlohmann::json OscRoleDebugToJson(const bt::DebugSnapshot& debug, std::size_t index) {
    const int tracker_index = debug.osc_role_indices[index];
    const std::string base_address = tracker_index > 0
        ? "/tracking/trackers/" + std::to_string(tracker_index)
        : std::string{};
    return {
        {"role", bt::ToString(bt::kTrackerRoles[index])},
        {"tracker_index", tracker_index},
        {"configured", debug.osc_role_configured[index]},
        {"osc_address_position", base_address.empty() ? std::string{} : base_address + "/position"},
        {"osc_address_rotation", base_address.empty() ? std::string{} : base_address + "/rotation"},
        {"mapping_kind", "configured_vrchat_index_path"},
        {"vrchat_role_binding_validated", false},
        {"valid", debug.osc_role_valid[index]},
        {"sent", debug.osc_role_sent[index]},
        {"degraded", debug.osc_role_degraded[index]},
        {"reason", debug.osc_role_reasons[index]},
        {"error_detail", debug.osc_role_error_details[index]}
    };
}

bool OscTrackerSpaceStale(const std::string& source) {
    return source == "steamvr_controller_alignment_stale";
}

std::string OscTrackerSpaceState(const bt::DebugSnapshot& debug) {
    if (OscTrackerSpaceStale(debug.osc_tracker_space_source)) {
        return "stale_controller_alignment";
    }
    if (!debug.osc_tracker_space_transform_valid) {
        return "missing_or_invalid_transform";
    }
    return "valid_active_transform";
}

nlohmann::json OscDebugToJson(const bt::DebugSnapshot& debug) {
    nlohmann::json roles = nlohmann::json::array();
    nlohmann::json active_roles = nlohmann::json::array();
    for (std::size_t i = 0; i < bt::kTrackerRoles.size(); ++i) {
        roles.push_back(OscRoleDebugToJson(debug, i));
        if (debug.osc_role_sent[i]) {
            active_roles.push_back(bt::ToString(bt::kTrackerRoles[i]));
        }
    }
    return {
        {"enabled", debug.osc_enabled},
        {"open", debug.osc_open},
        {"status", debug.osc_status},
        {"target_address", debug.osc_target_address},
        {"target_port", debug.osc_target_port},
        {"tracker_space_transform_valid", debug.osc_tracker_space_transform_valid},
        {"tracker_space_source", debug.osc_tracker_space_source},
        {"tracker_space_state", OscTrackerSpaceState(debug)},
        {"tracker_space_stale", OscTrackerSpaceStale(debug.osc_tracker_space_source)},
        {"tracker_space_blocked", debug.osc_status == "blocked_tracker_space"},
        {"tracker_space_block_reason", debug.osc_status == "blocked_tracker_space" ? debug.osc_last_error : ""},
        {"manual_tracker_space_fallback_available", debug.osc_manual_tracker_space_fallback_valid},
        {"manual_tracker_space_fallback_valid", debug.osc_manual_tracker_space_fallback_valid},
        {"manual_tracker_space_source", debug.osc_manual_tracker_space_source},
        {"last_send_ok", debug.osc_last_send_ok},
        {"last_error", debug.osc_last_error},
        {"open_attempts", debug.osc_open_attempts},
        {"sent_tracker_count", debug.osc_sent_tracker_count},
        {"skipped_tracker_count", debug.osc_skipped_tracker_count},
        {"sent_message_count", debug.osc_sent_message_count},
        {"active_roles", active_roles},
        {"roles", roles}
    };
}

nlohmann::json SteamVrBridgeRoleDebugToJson(const bt::DebugSnapshot& debug, std::size_t index) {
    return {
        {"role", bt::ToString(bt::kTrackerRoles[index])},
        {"enabled", debug.steamvr_bridge_role_enabled[index]},
        {"configured", debug.steamvr_bridge_role_enabled[index]},
        {"valid", debug.steamvr_bridge_role_valid[index]},
        {"sent", debug.steamvr_bridge_role_sent[index]},
        {"degraded", debug.steamvr_bridge_role_degraded[index]},
        {"confidence", debug.steamvr_bridge_role_confidence[index]},
        {"reason", debug.steamvr_bridge_role_reasons[index]},
        {"mapping_kind", "steamvr_virtual_tracker_driver"}
    };
}

nlohmann::json SteamVrBridgeDebugToJson(const bt::DebugSnapshot& debug) {
    nlohmann::json roles = nlohmann::json::array();
    nlohmann::json active_roles = nlohmann::json::array();
    for (std::size_t i = 0; i < bt::kTrackerRoles.size(); ++i) {
        roles.push_back(SteamVrBridgeRoleDebugToJson(debug, i));
        if (debug.steamvr_bridge_role_sent[i]) {
            active_roles.push_back(bt::ToString(bt::kTrackerRoles[i]));
        }
    }
    return {
        {"enabled", debug.steamvr_bridge_enabled},
        {"open", debug.steamvr_bridge_open},
        {"status", debug.steamvr_bridge_status},
        {"target_address", debug.steamvr_bridge_target_address},
        {"target_port", debug.steamvr_bridge_target_port},
        {"min_confidence", debug.steamvr_bridge_min_confidence},
        {"last_send_ok", debug.steamvr_bridge_last_send_ok},
        {"last_error", debug.steamvr_bridge_last_error},
        {"sent_tracker_count", debug.steamvr_bridge_sent_tracker_count},
        {"skipped_tracker_count", debug.steamvr_bridge_skipped_tracker_count},
        {"sent_message_count", debug.steamvr_bridge_sent_message_count},
        {"sequence", debug.steamvr_bridge_sequence},
        {"active_roles", active_roles},
        {"roles", roles}
    };
}

std::string DashboardDegradation(const bt::DebugSnapshot& debug) {
    if (!debug.degradation_mode.empty()) {
        return debug.degradation_mode;
    }
    return debug.tracking.degradation_mode.empty() ? "boot" : debug.tracking.degradation_mode;
}

std::string DashboardError(const bt::DebugSnapshot& debug) {
    if (!debug.last_error.empty()) {
        return debug.last_error;
    }
    return debug.tracking.last_error;
}

std::string MotionDecision(const bt::DebugSnapshot& debug, bt::MotionTarget target) {
    const auto index = static_cast<std::size_t>(target);
    const auto& entry = debug.tracking.motion_filter.targets[index];
    return std::string(bt::ToString(entry.decision)) + " / " + bt::ToString(entry.reason);
}

int ShowNativeDashboard(const bt::DebugSnapshot& debug, const std::string& note, int delay_ms) {
    const UiPalette p;
    cv::Mat canvas(760, 1180, CV_8UC3, p.base);

    cv::line(canvas, {28, 106}, {1152, 106}, p.line, 1);
    PutText(canvas, "BT / STEREO / LOCAL RUNTIME", {32, 42}, 0.38, p.muted, 1);
    PutText(canvas, "BODYTRACKER", {28, 88}, 1.28, p.ink, 3);
    PutText(canvas, "RUNTIME", {1032, 42}, 0.35, p.muted, 1);
    const std::string phase = debug.phase.empty() ? "BOOT" : Uppercase(debug.phase);
    PutText(canvas, FitText(phase, 130, 0.72, 2), {1032, 78}, 0.72, p.ink, 2);

    const std::string degradation = DashboardDegradation(debug);
    const bool nominal = degradation == "nominal";
    const bool blocked = !DashboardError(debug).empty() || degradation.find("not") != std::string::npos || degradation.find("failed") != std::string::npos;
    const cv::Scalar accent = nominal ? p.good : (blocked ? p.bad : p.accent);

    constexpr int left = 28;
    constexpr int top = 128;
    constexpr int gap = 12;
    constexpr int card_w = 272;
    constexpr int card_h = 112;

    DrawCard(canvas, {left, top, card_w, card_h}, "solve mode", degradation, DashboardError(debug).empty() ? "no active error" : DashboardError(debug), accent);
    DrawCard(canvas, {left + (card_w + gap), top, card_w, card_h}, "posture", bt::ToString(debug.tracking.state.posture_mode), std::string(bt::ToString(debug.tracking.state.support.left_foot.type)) + " / " + bt::ToString(debug.tracking.state.support.right_foot.type), p.accent);
    DrawCard(canvas, {left + 2 * (card_w + gap), top, card_w, card_h}, "filter", MotionDecision(debug, bt::MotionTarget::Root), "L " + MotionDecision(debug, bt::MotionTarget::LeftFoot) + " / R " + MotionDecision(debug, bt::MotionTarget::RightFoot), p.accent);
    DrawCard(canvas, {left + 3 * (card_w + gap), top, card_w, card_h}, "hmd", debug.hmd_valid || debug.hmd.valid ? "valid" : "none", "json/null provider", debug.hmd_valid || debug.hmd.valid ? p.good : p.muted);

    DrawCard(canvas, {left, top + card_h + gap, card_w, card_h}, "osc", debug.osc_status.empty() ? (debug.osc_enabled ? "closed" : "disabled") : debug.osc_status, std::to_string(debug.osc_sent_tracker_count) + " trackers / " + std::to_string(debug.osc_sent_message_count) + " messages", debug.osc_enabled && debug.osc_open && debug.osc_last_send_ok ? p.good : p.muted);
    DrawCard(canvas, {left + (card_w + gap), top + card_h + gap, card_w, card_h}, "pose A/B", Fmt(debug.camera_a_pose_confidence, 3) + " / " + Fmt(debug.camera_b_pose_confidence, 3), "confidence", p.accent);
    DrawCard(canvas, {left + 2 * (card_w + gap), top + card_h + gap, card_w, card_h}, "reliability A/B", Fmt(debug.camera_a_reliability, 3) + " / " + Fmt(debug.camera_b_reliability, 3), "lower body mean", p.accent);
    DrawCard(canvas, {left + 3 * (card_w + gap), top + card_h + gap, card_w, card_h}, "timings", Fmt(debug.inference_ms, 1) + " ms", "pipeline " + Fmt(debug.pipeline_ms, 1) + " / osc " + Fmt(debug.osc_ms, 2), p.accent);

    const cv::Rect pair_rect(left, top + 2 * (card_h + gap), 556, 132);
    DrawCard(canvas, pair_rect, "frame pairing", "accepted " + std::to_string(debug.frame_pairing.accepted_pairs), "skew " + Fmt(debug.frame_pairing.last_skew_ms, 2) + " ms / degraded " + std::to_string(debug.frame_pairing.degraded_duplicate + debug.frame_pairing.degraded_reused_a + debug.frame_pairing.degraded_reused_b + debug.frame_pairing.degraded_skew), debug.frame_pair_degraded ? p.warn : p.accent);
    PutText(canvas, "SEQ A " + std::to_string(debug.frame_pairing.last_accepted_sequence_a) + " / SEQ B " + std::to_string(debug.frame_pairing.last_accepted_sequence_b), {pair_rect.x + 16, pair_rect.y + 102}, 0.42, p.muted, 1);
    PutText(canvas, "CURRENT " + (debug.frame_pair_reason.empty() ? std::string("none") : debug.frame_pair_reason), {pair_rect.x + 16, pair_rect.y + 122}, 0.42, debug.frame_pair_degraded ? p.warn : p.muted, 1);

    const cv::Rect cam_rect(left + 568, top + 2 * (card_h + gap), 556, 132);
    DrawCard(canvas, cam_rect, "camera health", "A " + BoolText(debug.camera_a.opened) + " / B " + BoolText(debug.camera_b.opened), "frames " + std::to_string(debug.camera_a.delivered_frames) + " / " + std::to_string(debug.camera_b.delivered_frames), debug.camera_a.opened && debug.camera_b.opened ? p.good : p.bad);
    PutText(canvas, "AGE " + Fmt(debug.camera_a_frame_age_ms, 1) + " / " + Fmt(debug.camera_b_frame_age_ms, 1) + " MS", {cam_rect.x + 16, cam_rect.y + 118}, 0.42, p.muted, 1);

    const cv::Rect tracker_rect(left, top + 2 * (card_h + gap) + 144, 556, 170);
    cv::rectangle(canvas, tracker_rect, p.paper, cv::FILLED);
    cv::rectangle(canvas, tracker_rect, p.hair, 1);
    cv::line(canvas, tracker_rect.tl(), {tracker_rect.x + tracker_rect.width, tracker_rect.y}, p.line, 1);
    PutText(canvas, "TRACKERS", {tracker_rect.x + 16, tracker_rect.y + 26}, 0.38, p.muted, 1);
    const auto& dashboard_trackers = EffectiveTrackersForDashboard(debug);
    constexpr int tracker_cols = 2;
    constexpr int tracker_rows_per_col = 4;
    const int tracker_col_w = tracker_rect.width / tracker_cols;
    const int tracker_row_step = 32;
    for (std::size_t tracker_i = 0; tracker_i < dashboard_trackers.size(); ++tracker_i) {
        const auto& tracker = dashboard_trackers[tracker_i];
        const int col = static_cast<int>(tracker_i) / tracker_rows_per_col;
        const int row = static_cast<int>(tracker_i) % tracker_rows_per_col;
        const int x = tracker_rect.x + 18 + col * tracker_col_w;
        const int y = tracker_rect.y + 56 + row * tracker_row_step;
        cv::circle(canvas, {x, y - 5}, 5, tracker.valid ? p.good : p.bad, cv::FILLED);
        PutText(canvas, FitText(TrackerRoleName(tracker.role), 112, 0.42, 1), {x + 16, y}, 0.42, p.ink, 1);
        PutText(canvas, "C " + Fmt(tracker.confidence, 2), {x + 138, y}, 0.38, p.muted, 1);
        PutText(canvas, Fmt(tracker.pose.position.x, 1) + "," +
            Fmt(tracker.pose.position.y, 1) + "," +
            Fmt(tracker.pose.position.z, 1), {x + 192, y}, 0.36, p.muted, 1);
    }

    const cv::Rect raw_rect(left + 568, top + 2 * (card_h + gap) + 144, 556, 170);
    cv::rectangle(canvas, raw_rect, p.paper, cv::FILLED);
    cv::rectangle(canvas, raw_rect, p.hair, 1);
    cv::line(canvas, raw_rect.tl(), {raw_rect.x + raw_rect.width, raw_rect.y}, p.line, 1);
    PutText(canvas, "STATUS", {raw_rect.x + 16, raw_rect.y + 26}, 0.38, p.muted, 1);
    PutText(canvas, FitText("PHASE " + phase, raw_rect.width - 32, 0.48, 1), {raw_rect.x + 16, raw_rect.y + 62}, 0.48, p.ink, 1);
    PutText(canvas, FitText("ROOT " + std::string(bt::ToString(debug.tracking.state.support.root_support)), raw_rect.width - 32, 0.48, 1), {raw_rect.x + 16, raw_rect.y + 92}, 0.48, p.ink, 1);
    PutText(canvas, FitText(note.empty() ? "ESC/Q CLOSES DASHBOARD" : note, raw_rect.width - 32, 0.48, 1), {raw_rect.x + 16, raw_rect.y + 122}, 0.48, p.muted, 1);
    PutText(canvas, FitText(DashboardError(debug), raw_rect.width - 32, 0.44, 1), {raw_rect.x + 16, raw_rect.y + 150}, 0.44, p.bad, 1);

    cv::imshow(kDashboardWindowName, canvas);
    return cv::waitKey(delay_ms);
}

void WaitOnNativeDashboard(bt::DebugSnapshot debug, const std::string& note) {
    debug.phase = debug.phase.empty() ? "blocked" : debug.phase;
    while (!g_stop_requested) {
        const int key = ShowNativeDashboard(debug, note, 50);
        if (key == 27 || key == 'q' || key == 'Q') {
            break;
        }
    }
    cv::destroyWindow(kDashboardWindowName);
}

struct CameraProbe {
    int index = 0;
    bool opened = false;
    int width = 0;
    int height = 0;
    double fps = 0.0;
    int backend_api = 0;
    std::string backend_name;
    std::string preview_data_url;
};

enum class SetupAction {
    Quit,
    Retry
};

struct CaptureBackendCandidate {
    int api = cv::CAP_ANY;
    const char* name = "any";
};

std::vector<CaptureBackendCandidate> CaptureBackends() {
#ifdef _WIN32
    return {
        {cv::CAP_DSHOW, "dshow"},
        {cv::CAP_MSMF, "msmf"},
        {cv::CAP_ANY, "any"}
    };
#else
    return {{cv::CAP_ANY, "any"}};
#endif
}

std::string Base64Encode(const std::vector<unsigned char>& bytes) {
    static constexpr char alphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string out;
    out.reserve(((bytes.size() + 2) / 3) * 4);
    std::size_t i = 0;
    while (i + 2 < bytes.size()) {
        const unsigned int n = (static_cast<unsigned int>(bytes[i]) << 16) |
            (static_cast<unsigned int>(bytes[i + 1]) << 8) |
            static_cast<unsigned int>(bytes[i + 2]);
        out.push_back(alphabet[(n >> 18) & 63]);
        out.push_back(alphabet[(n >> 12) & 63]);
        out.push_back(alphabet[(n >> 6) & 63]);
        out.push_back(alphabet[n & 63]);
        i += 3;
    }
    if (i < bytes.size()) {
        unsigned int n = static_cast<unsigned int>(bytes[i]) << 16;
        if (i + 1 < bytes.size()) {
            n |= static_cast<unsigned int>(bytes[i + 1]) << 8;
        }
        out.push_back(alphabet[(n >> 18) & 63]);
        out.push_back(alphabet[(n >> 12) & 63]);
        out.push_back(i + 1 < bytes.size() ? alphabet[(n >> 6) & 63] : '=');
        out.push_back('=');
    }
    return out;
}

cv::Rect ClampPreviewRect(const cv::Mat& frame, const bt::Rect2f& source) {
    if (frame.empty()) {
        return {};
    }
    const int x1 = std::clamp(static_cast<int>(std::floor(source.x)), 0, frame.cols - 1);
    const int y1 = std::clamp(static_cast<int>(std::floor(source.y)), 0, frame.rows - 1);
    const int x2 = std::clamp(static_cast<int>(std::ceil(source.x + source.width)), x1 + 1, frame.cols);
    const int y2 = std::clamp(static_cast<int>(std::ceil(source.y + source.height)), y1 + 1, frame.rows);
    return cv::Rect(x1, y1, std::max(1, x2 - x1), std::max(1, y2 - y1));
}

bt::Rect2f FullFrameRect(int width, int height) {
    return bt::Rect2f{0.0f, 0.0f, static_cast<float>(std::max(1, width)), static_cast<float>(std::max(1, height))};
}

std::string MakePreviewDataUrl(const cv::Mat& frame, const bt::Rect2f& source) {
    if (frame.empty()) {
        return {};
    }
    const cv::Rect crop = ClampPreviewRect(frame, source);
    cv::Mat source_view = frame(crop);
    cv::Mat preview;
    const int width = std::min(960, std::max(1, source_view.cols));
    const double scale = static_cast<double>(width) / static_cast<double>(std::max(1, source_view.cols));
    cv::resize(source_view, preview, cv::Size(width, std::max(1, static_cast<int>(std::lround(source_view.rows * scale)))));
    std::vector<unsigned char> jpeg;
    if (!cv::imencode(".jpg", preview, jpeg, {cv::IMWRITE_JPEG_QUALITY, 86})) {
        return {};
    }
    return "data:image/jpeg;base64," + Base64Encode(jpeg);
}

std::string MakePreviewDataUrl(const cv::Mat& frame) {
    return MakePreviewDataUrl(frame, FullFrameRect(frame.cols, frame.rows));
}

CameraProbe ProbeCameraIndex(int index) {
    CameraProbe probe;
    probe.index = index;
    cv::VideoCapture cap;
    for (const auto& backend : CaptureBackends()) {
        cap.open(index, backend.api);
        if (cap.isOpened()) {
            probe.backend_api = backend.api;
            probe.backend_name = backend.name;
            break;
        }
        cap.release();
    }
    probe.opened = cap.isOpened();
    if (probe.opened) {
        cv::Mat frame;
        cap.read(frame);
        if (!frame.empty()) {
            probe.width = frame.cols;
            probe.height = frame.rows;
            probe.preview_data_url = MakePreviewDataUrl(frame);
        } else {
            probe.width = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
            probe.height = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
        }
        probe.fps = cap.get(cv::CAP_PROP_FPS);
    }
    return probe;
}

std::vector<CameraProbe> ScanCameraIndices(int max_index = 9) {
    std::vector<CameraProbe> probes;
    for (int index = 0; index <= max_index; ++index) {
        probes.push_back(ProbeCameraIndex(index));
    }
    return probes;
}

const CameraProbe* FindCameraProbe(const std::vector<CameraProbe>& probes, int index) {
    for (const auto& probe : probes) {
        if (probe.index == index) {
            return &probe;
        }
    }
    return nullptr;
}

bt::CameraConfig CameraConfigForProbe(const std::vector<CameraProbe>& probes, int index, bt::CameraConfig fallback) {
    fallback.device_index = index;
    const CameraProbe* probe = FindCameraProbe(probes, index);
    if (!probe || !probe->opened) {
        return fallback;
    }
    if (probe->width > 0) {
        fallback.width = probe->width;
    }
    if (probe->height > 0) {
        fallback.height = probe->height;
    }
    if (std::isfinite(probe->fps) && probe->fps > 0.0) {
        fallback.fps = std::max(1, static_cast<int>(std::lround(probe->fps)));
    }
    return fallback;
}

int SelectNextOpenCamera(const std::vector<CameraProbe>& probes, int current, int direction) {
    if (probes.empty()) {
        return current;
    }
    const int count = static_cast<int>(probes.size());
    int position = std::clamp(current, 0, count - 1);
    for (int step = 0; step < count; ++step) {
        position = (position + direction + count) % count;
        if (probes[static_cast<std::size_t>(position)].opened) {
            return probes[static_cast<std::size_t>(position)].index;
        }
    }
    return std::clamp(current + direction, 0, count - 1);
}

std::string ResolveDisplayPath(const std::filesystem::path& path) {
    try {
        return std::filesystem::absolute(path).string();
    } catch (...) {
        return path.string();
    }
}


bt::Status WriteJsonFileAtomically(const std::filesystem::path& path, const nlohmann::json& j, const char* label) {
    const auto parent = path.parent_path();
    if (!parent.empty()) {
        std::error_code mkdir_ec;
        std::filesystem::create_directories(parent, mkdir_ec);
        if (mkdir_ec) {
            return bt::Status::Error(bt::StatusCode::InvalidArgument, std::string("Failed to create parent directory for ") + label + ": " + mkdir_ec.message());
        }
    }

    std::filesystem::path temp_path = path;
    temp_path += ".tmp";
    std::filesystem::path backup_path = path;
    backup_path += ".bak";

    auto remove_quietly = [](const std::filesystem::path& p) {
        std::error_code ec;
        std::filesystem::remove(p, ec);
    };

    {
        std::ofstream out(temp_path, std::ios::trunc);
        if (!out) {
            return bt::Status::Error(bt::StatusCode::InvalidArgument, std::string("Failed to open temporary ") + label + " file");
        }
        out << j.dump(2) << '\n';
        if (!out) {
            remove_quietly(temp_path);
            return bt::Status::Error(bt::StatusCode::InvalidArgument, std::string("Failed to finish writing temporary ") + label + " file");
        }
    }

    std::error_code ec;
    const bool stale_backup_exists = std::filesystem::exists(backup_path, ec);
    if (ec) {
        remove_quietly(temp_path);
        return bt::Status::Error(bt::StatusCode::InvalidArgument, std::string("Failed to inspect stale backup for ") + label + ": " + ec.message());
    }
    if (stale_backup_exists) {
        std::filesystem::remove(backup_path, ec);
        if (ec) {
            remove_quietly(temp_path);
            return bt::Status::Error(bt::StatusCode::InvalidArgument, std::string("Failed to clear stale backup for ") + label + ": " + ec.message());
        }
    }

    ec.clear();
    const bool target_exists = std::filesystem::exists(path, ec);
    if (ec) {
        remove_quietly(temp_path);
        return bt::Status::Error(bt::StatusCode::InvalidArgument, std::string("Failed to inspect existing ") + label + " file: " + ec.message());
    }

    bool backup_created = false;
    if (target_exists) {
        std::filesystem::rename(path, backup_path, ec);
        if (ec) {
            remove_quietly(temp_path);
            return bt::Status::Error(bt::StatusCode::InvalidArgument, std::string("Failed to move existing ") + label + " file to backup: " + ec.message());
        }
        backup_created = true;
    }

    auto rollback_after_replace_failure = [&](const std::error_code& replace_ec) {
        std::error_code rollback_ec;
        std::filesystem::remove(path, rollback_ec);
        if (rollback_ec) {
            return bt::Status::Error(bt::StatusCode::InvalidArgument,
                std::string("Failed to replace live ") + label + " file: " + replace_ec.message() +
                "; rollback cleanup failed: " + rollback_ec.message());
        }
        if (backup_created) {
            rollback_ec.clear();
            std::filesystem::rename(backup_path, path, rollback_ec);
            if (rollback_ec) {
                return bt::Status::Error(bt::StatusCode::InvalidArgument,
                    std::string("Failed to replace live ") + label + " file: " + replace_ec.message() +
                    "; rollback also failed: " + rollback_ec.message());
            }
        }
        return bt::Status::Error(bt::StatusCode::InvalidArgument, std::string("Failed to replace live ") + label + " file: " + replace_ec.message());
    };

    ec.clear();
    std::filesystem::rename(temp_path, path, ec);
    if (ec) {
        const bt::Status rollback_status = rollback_after_replace_failure(ec);
        remove_quietly(temp_path);
        return rollback_status;
    }

    if (backup_created) {
        remove_quietly(backup_path);
    }
    return bt::Status::OK();
}

bt::Status SaveCameraSelectionToConfig(const std::filesystem::path& config_path, int camera_a_index, int camera_b_index) {
    nlohmann::json j;
    {
        std::ifstream in(config_path);
        if (in) {
            try {
                in >> j;
            } catch (const std::exception& e) {
                return bt::Status::Error(bt::StatusCode::InvalidArgument, std::string("Failed to parse config for camera save: ") + e.what());
            }
        }
    }
    if (!j.is_object()) {
        j = nlohmann::json::object();
    }
    if (!j["camera_a"].is_object()) {
        j["camera_a"] = nlohmann::json::object();
    }
    if (!j["camera_b"].is_object()) {
        j["camera_b"] = nlohmann::json::object();
    }
    j["camera_a"]["source"] = "opencv";
    j["camera_b"]["source"] = "opencv";
    j["camera_a"]["device_index"] = camera_a_index;
    j["camera_b"]["device_index"] = camera_b_index;

    return WriteJsonFileAtomically(config_path, j, "camera selection config");
}

void DrawSetupCard(cv::Mat& canvas, const cv::Rect& rect, const std::string& label, const std::string& value, const std::string& sub, cv::Scalar stripe) {
    DrawCard(canvas, rect, label, value, sub, stripe);
}

SetupAction RunSetupDashboard(
    const std::filesystem::path& config_path,
    const bt::AppConfig& cfg,
    const std::string& reason,
    const std::string& model_note) {

    UiPalette p;
    std::vector<CameraProbe> probes = ScanCameraIndices();
    int selected_a = cfg.camera_a.device_index;
    int selected_b = cfg.camera_b.device_index;
    std::string status = reason.empty() ? "setup ready" : reason;

    while (!g_stop_requested) {
        cv::Mat canvas(800, 1180, CV_8UC3, p.base);
        cv::line(canvas, {28, 106}, {1152, 106}, p.line, 1);
        PutText(canvas, "BT / SETUP / SIGNALONLY LOCAL", {32, 42}, 0.38, p.muted, 1);
        PutText(canvas, "BODYTRACKER SETUP", {28, 88}, 1.18, p.ink, 3);
        PutText(canvas, "ENTER SAVE+RETRY", {940, 42}, 0.38, p.muted, 1);
        PutText(canvas, "Q EXIT", {1046, 78}, 0.58, p.ink, 2);

        std::error_code setup_model_exists_ec;
        const bool model_exists = std::filesystem::exists(cfg.tracking.model_path, setup_model_exists_ec) && !setup_model_exists_ec;
        const std::string model_status_note = setup_model_exists_ec
            ? (std::string("inspect failed: ") + setup_model_exists_ec.message())
            : model_note;
        DrawSetupCard(
            canvas,
            {28, 128, 548, 132},
            "model",
            model_exists ? "found" : "missing",
            model_exists ? ResolveDisplayPath(cfg.tracking.model_path) : model_status_note,
            model_exists ? p.good : p.bad);
        PutText(canvas, "DOWNLOAD: RTMPOSE-X HALPE26 384X288 ONNX", {44, 246}, 0.42, model_exists ? p.muted : p.bad, 1);

        DrawSetupCard(
            canvas,
            {604, 128, 548, 132},
            "camera assignment",
            "A " + std::to_string(selected_a) + " / B " + std::to_string(selected_b),
            "A/Z changes camera A. S/X changes camera B. R rescans.",
            p.accent);
        PutText(canvas, FitText(status, 500, 0.42, 1), {620, 246}, 0.42, p.muted, 1);

        const cv::Rect table(28, 286, 1124, 404);
        cv::rectangle(canvas, table, p.paper, cv::FILLED);
        cv::rectangle(canvas, table, p.hair, 1);
        cv::line(canvas, table.tl(), {table.x + table.width, table.y}, p.line, 1);
        PutText(canvas, "CAMERA SELECTOR", {44, 324}, 0.5, p.ink, 1);
        PutText(canvas, "INDEX", {54, 360}, 0.38, p.muted, 1);
        PutText(canvas, "STATE", {150, 360}, 0.38, p.muted, 1);
        PutText(canvas, "FRAME", {320, 360}, 0.38, p.muted, 1);
        PutText(canvas, "ASSIGNMENT", {500, 360}, 0.38, p.muted, 1);
        PutText(canvas, "Use two different OPEN indices for stereo runtime.", {740, 360}, 0.38, p.muted, 1);

        int y = 392;
        for (const auto& probe : probes) {
            const bool is_a = probe.index == selected_a;
            const bool is_b = probe.index == selected_b;
            const cv::Scalar row_bg = (is_a || is_b) ? p.soft : p.paper;
            cv::rectangle(canvas, {44, y - 22, 1088, 34}, row_bg, cv::FILLED);
            cv::rectangle(canvas, {44, y - 22, 1088, 34}, p.hair, 1);
            cv::circle(canvas, {66, y - 4}, 6, probe.opened ? p.good : p.bad, cv::FILLED);
            PutText(canvas, std::to_string(probe.index), {92, y}, 0.48, p.ink, 1);
            PutText(canvas, probe.opened ? "OPEN" : "NO SIGNAL", {150, y}, 0.48, probe.opened ? p.good : p.bad, 1);
            const std::string frame = probe.opened ? std::to_string(probe.width) + "x" + std::to_string(probe.height) + " @ " + Fmt(probe.fps, 1) : "--";
            PutText(canvas, frame, {320, y}, 0.48, p.muted, 1);
            std::string assign;
            if (is_a) assign += "CAM A ";
            if (is_b) assign += "CAM B";
            PutText(canvas, assign.empty() ? "--" : assign, {500, y}, 0.48, p.ink, 1);
            y += 38;
        }

        DrawSetupCard(canvas, {28, 710, 1124, 62}, "controls", "A/Z  S/X  R  ENTER  Q", "A/Z select camera A, S/X select camera B, R rescan, Enter saves config and retries runtime.", p.accent);

        cv::imshow(kDashboardWindowName, canvas);
        const int key = cv::waitKey(50);
        if (key == 27 || key == 'q' || key == 'Q') {
            return SetupAction::Quit;
        }
        if (key == 'r' || key == 'R') {
            status = "rescanning cameras";
            probes = ScanCameraIndices();
        } else if (key == 'a' || key == 'A') {
            selected_a = SelectNextOpenCamera(probes, selected_a, -1);
        } else if (key == 'z' || key == 'Z') {
            selected_a = SelectNextOpenCamera(probes, selected_a, 1);
        } else if (key == 's' || key == 'S') {
            selected_b = SelectNextOpenCamera(probes, selected_b, -1);
        } else if (key == 'x' || key == 'X') {
            selected_b = SelectNextOpenCamera(probes, selected_b, 1);
        } else if (key == 13 || key == 10) {
            if (selected_a == selected_b) {
                status = "camera A and B must use different indices";
                continue;
            }
            const auto save = SaveCameraSelectionToConfig(config_path, selected_a, selected_b);
            if (!save.ok()) {
                status = save.message;
                continue;
            }
            return SetupAction::Retry;
        }
    }
    return SetupAction::Quit;
}


bt::Rect2f InitialRoiPixels(const bt::CameraConfig& config, int frame_width, int frame_height) {
    bt::Rect2f roi = config.initial_roi;
    if (config.initial_roi_normalized) {
        roi.x *= static_cast<float>(frame_width);
        roi.y *= static_cast<float>(frame_height);
        roi.width *= static_cast<float>(frame_width);
        roi.height *= static_cast<float>(frame_height);
    }
    return roi;
}

bt::Result<ViewProcessResult> ProcessCameraView(
    const bt::FramePacket& frame,
    ViewRuntimeState& view,
    const bt::CameraConfig& camera_config,
    bt::RtmPoseSession& model,
    bt::RtmPoseSession* depth_postprocess_model,
    int depth_postprocess_interval_frames,
    bt::PostureMode posture_mode_hint) {

    if (!view.roi.GetState().initialized) {
        if (camera_config.initial_roi_enabled) {
            view.roi.InitializeRect(frame.width, frame.height, InitialRoiPixels(camera_config, frame.width, frame.height));
        } else {
            view.roi.InitializeFullFrame(frame.width, frame.height);
        }
    }

    const auto timing_start = bt::NowQpc();
    const bt::RoiState roi_used = view.roi.GetState();
    const auto preprocess_start = bt::NowQpc();
    const auto preprocess_status = bt::FillRtmPoseInputPacket(
        frame.bgr,
        roi_used.rect,
        model.GetInfo(),
        view.input_packet);
    if (!preprocess_status.ok()) {
        return preprocess_status;
    }
    const double preprocess_ms = bt::QpcDeltaSeconds(preprocess_start, bt::NowQpc()) * 1000.0;

    const auto onnx_start = bt::NowQpc();
    const auto outputs = model.RunSingleInputF32(view.input_packet.tensor);
    if (!outputs.ok()) {
        return outputs.status();
    }
    const double onnx_ms = bt::QpcDeltaSeconds(onnx_start, bt::NowQpc()) * 1000.0;

    const auto decode_start = bt::NowQpc();
    const auto decoded = bt::DecodeRtmPoseOutputsWithDepth(outputs.value(), view.input_packet.meta);
    if (!decoded.ok()) {
        return decoded.status();
    }
    const double decode_ms = bt::QpcDeltaSeconds(decode_start, bt::NowQpc()) * 1000.0;

    bt::DecodedPose3D postprocess_depth = decoded.value().pose3d;
    const int depth_interval = std::max(1, depth_postprocess_interval_frames);
    const bool run_depth_postprocess =
        decoded.value().pose2d.valid &&
        depth_postprocess_model &&
        depth_postprocess_model->GetInfo().loaded &&
        ((view.depth_postprocess_frames++ % static_cast<std::uint64_t>(depth_interval)) == 0);
    if (run_depth_postprocess) {
        bt::RtmPoseInputPacket depth_packet;
        const auto depth_preprocess_status = bt::FillRtmPoseInputPacket(
            frame.bgr,
            roi_used.rect,
            depth_postprocess_model->GetInfo(),
            depth_packet);
        if (depth_preprocess_status.ok()) {
            const auto depth_outputs = depth_postprocess_model->RunSingleInputF32(depth_packet.tensor);
            if (depth_outputs.ok()) {
                const auto depth_decoded = bt::DecodeRtmPoseOutputsWithDepth(depth_outputs.value(), depth_packet.meta);
                if (depth_decoded.ok() && depth_decoded.value().pose3d.valid) {
                    postprocess_depth = depth_decoded.value().pose3d;
                }
            }
        }
    }

    const auto reliability = bt::ComputeViewReliability(
        decoded.value().pose2d,
        roi_used,
        frame.width,
        frame.height,
        posture_mode_hint,
        view.previous_pose.valid ? &view.previous_pose : nullptr);
    if (!reliability.ok()) {
        return reliability.status();
    }

    const bt::RoiState roi_next = view.roi.Update(frame.width, frame.height, &decoded.value().pose2d, posture_mode_hint);
    view.previous_pose = decoded.value().pose2d;

    ViewProcessResult result;
    result.pose   = decoded.value().pose2d;
    result.pose3d = postprocess_depth;
    result.reliability = reliability.value();
    result.roi_used = roi_used;
    result.roi_next = roi_next;
    result.inference_ms = bt::QpcDeltaSeconds(timing_start, bt::NowQpc()) * 1000.0;
    result.preprocess_ms = preprocess_ms;
    result.onnx_ms = onnx_ms;
    result.decode_ms = decode_ms;
    return result;
}


std::string Lowercase(std::string value) {
    for (auto& ch : value) {
        ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    }
    return value;
}

bool IsSupportedImagePath(const std::filesystem::path& path) {
    const std::string ext = Lowercase(path.extension().string());
    return ext == ".png" || ext == ".jpg" || ext == ".jpeg" ||
        ext == ".bmp" || ext == ".tif" || ext == ".tiff" || ext == ".webp";
}

bt::Result<std::vector<std::filesystem::path>> ListImageFiles(const std::filesystem::path& dir) {
    std::error_code exists_ec;
    const bool exists = std::filesystem::exists(dir, exists_ec);
    if (exists_ec) {
        return bt::Status::Error(bt::StatusCode::InvalidArgument, "Could not inspect image directory: " + exists_ec.message());
    }
    std::error_code dir_ec;
    const bool is_directory = std::filesystem::is_directory(dir, dir_ec);
    if (dir_ec) {
        return bt::Status::Error(bt::StatusCode::InvalidArgument, "Could not inspect image directory type: " + dir_ec.message());
    }
    if (!exists || !is_directory) {
        return bt::Status::Error(bt::StatusCode::InvalidArgument, "Image directory does not exist: " + dir.string());
    }

    std::vector<std::filesystem::path> files;
    for (const auto& entry : std::filesystem::directory_iterator(dir)) {
        if (entry.is_regular_file() && IsSupportedImagePath(entry.path())) {
            files.push_back(entry.path());
        }
    }
    std::sort(files.begin(), files.end());
    if (files.empty()) {
        return bt::Status::Error(bt::StatusCode::InvalidArgument, "Image directory contains no supported image files: " + dir.string());
    }
    return files;
}

bt::Result<std::vector<cv::Mat>> LoadImagesFromDirectory(const std::filesystem::path& dir) {
    const auto files = ListImageFiles(dir);
    if (!files.ok()) {
        return files.status();
    }

    std::vector<cv::Mat> frames;
    frames.reserve(files.value().size());
    for (const auto& file : files.value()) {
        cv::Mat frame = cv::imread(file.string(), cv::IMREAD_COLOR);
        if (!frame.empty()) {
            frames.push_back(std::move(frame));
        }
    }
    if (frames.empty()) {
        return bt::Status::Error(bt::StatusCode::InvalidArgument, "No readable images in directory: " + dir.string());
    }
    return frames;
}

bt::Result<int> ParseInt(const char* text, const char* label) {
    try {
        std::size_t used = 0;
        const int value = std::stoi(text, &used);
        if (used != std::string(text).size()) {
            return bt::Status::Error(bt::StatusCode::InvalidArgument, std::string("Invalid integer for ") + label + ": " + text);
        }
        return value;
    } catch (const std::exception&) {
        return bt::Status::Error(bt::StatusCode::InvalidArgument, std::string("Invalid integer for ") + label + ": " + text);
    }
}

bt::Result<float> ParseFloat(const char* text, const char* label) {
    try {
        std::size_t used = 0;
        const float value = std::stof(text, &used);
        if (used != std::string(text).size() || !std::isfinite(value)) {
            return bt::Status::Error(bt::StatusCode::InvalidArgument, std::string("Invalid number for ") + label + ": " + text);
        }
        return value;
    } catch (const std::exception&) {
        return bt::Status::Error(bt::StatusCode::InvalidArgument, std::string("Invalid number for ") + label + ": " + text);
    }
}

bt::Result<bt::CalibrationBundle> LoadCalibrationForUpdate(const std::filesystem::path& path) {
    std::error_code ec;
    const bool exists = std::filesystem::exists(path, ec);
    if (ec) {
        return bt::Status::Error(bt::StatusCode::InvalidArgument, std::string("Failed to inspect calibration path: ") + ec.message());
    }
    if (exists) {
        return bt::LoadCalibration(path);
    }
    return bt::CalibrationBundle{};
}

int PrintStatusAndReturn(const bt::Status& status) {
    if (!status.ok()) {
        std::cerr << status.message << '\n';
        return 1;
    }
    return 0;
}

void PrintUsage() {
    std::cerr
        << "bodytracker usage:\n"
        << "  bodytracker [config/default.json]          # desktop UI\n"
        << "  bodytracker --run [config/default.json]    # native runtime dashboard\n"
        << "  bodytracker --setup [config/default.json]  # same desktop UI, setup controls included\n"
        << "  bodytracker --capture-chessboard <camera_index> <out_dir> <board_w> <board_h> <target_count> [width height fps min_interval_ms]\n"
        << "  bodytracker --capture-stereo-chessboard <camera_a_index> <camera_b_index> <out_a_dir> <out_b_dir> <board_w> <board_h> <target_count> [width height fps min_interval_ms]\n"
        << "  bodytracker --calibrate-intrinsics camera_a|camera_b <image_dir> <board_w> <board_h> <square_m> <calib_out.json>\n"
        << "  bodytracker --calibrate-stereo <camera_a_image_dir> <camera_b_image_dir> <board_w> <board_h> <square_m> <calib_in_out.json>\n"
        << "  bodytracker --calibrate-floor <calib_in_out.json> <x y z> <x y z> <x y z> [more xyz points]\n"
        << "  bodytracker --calibrate-floor-geometry <calib_in_out.json> <manual_lines.json> <image_w> <image_h> [floor_type] [spacing_m] [second_axis_spacing_m] [camera_height_m]\n"
        << "  (automatic floorboard image detection was removed; use a manual drawn plank outline)\n"
        << "  bodytracker --align-floor-geometry <camera_a_calib_in_out.json> <camera_b_calib.json>\n"
        << "  bodytracker --set-body <calib_in_out.json> <pelvis_w> <left_femur> <right_femur> <left_tibia> <right_tibia> <left_foot> <right_foot>\n"
        << "  bodytracker --status <calib.json>\n"
        << "  bodytracker --replay-solve <calib.json> <runtime.ndjson>\n"
        << "  bodytracker --benchmark-replay <calib.json> <runtime.ndjson> [iterations]\n";
}

bt::Result<std::vector<bt::StereoChessboardObservation>> LoadStereoObservationsFromDirectories(
    const std::filesystem::path& dir_a,
    const std::filesystem::path& dir_b,
    cv::Size board_size,
    cv::Size* out_image_size) {

    const auto files_a = ListImageFiles(dir_a);
    if (!files_a.ok()) {
        return files_a.status();
    }
    const auto files_b = ListImageFiles(dir_b);
    if (!files_b.ok()) {
        return files_b.status();
    }

    if (files_a.value().size() != files_b.value().size()) {
        return bt::Status::Error(
            bt::StatusCode::ValidationError,
            "Stereo calibration directories must contain the same number of readable image files");
    }

    const std::size_t count = files_a.value().size();
    if (count == 0) {
        return bt::Status::Error(bt::StatusCode::InvalidArgument, "Stereo calibration found no paired images");
    }

    std::vector<bt::StereoChessboardObservation> observations;
    observations.reserve(count);

    for (std::size_t i = 0; i < count; ++i) {
        cv::Mat image_a = cv::imread(files_a.value()[i].string(), cv::IMREAD_COLOR);
        cv::Mat image_b = cv::imread(files_b.value()[i].string(), cv::IMREAD_COLOR);
        if (image_a.empty() || image_b.empty()) {
            continue;
        }
        if (image_a.size() != image_b.size()) {
            return bt::Status::Error(bt::StatusCode::ValidationError, "Stereo image pair sizes differ");
        }
        if (out_image_size && out_image_size->width == 0 && out_image_size->height == 0) {
            *out_image_size = image_a.size();
        } else if (out_image_size && image_a.size() != *out_image_size) {
            return bt::Status::Error(bt::StatusCode::ValidationError, "Stereo calibration image pairs must all use the same image size");
        }

        cv::Mat gray_a;
        cv::Mat gray_b;
        cv::cvtColor(image_a, gray_a, cv::COLOR_BGR2GRAY);
        cv::cvtColor(image_b, gray_b, cv::COLOR_BGR2GRAY);

        bt::StereoChessboardObservation obs;
        const bool found_a = cv::findChessboardCorners(
            gray_a,
            board_size,
            obs.camera_a_points,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
        const bool found_b = cv::findChessboardCorners(
            gray_b,
            board_size,
            obs.camera_b_points,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
        if (!found_a || !found_b) {
            continue;
        }

        cv::cornerSubPix(
            gray_a,
            obs.camera_a_points,
            cv::Size(11, 11),
            cv::Size(-1, -1),
            cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, 0.001));
        cv::cornerSubPix(
            gray_b,
            obs.camera_b_points,
            cv::Size(11, 11),
            cv::Size(-1, -1),
            cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, 0.001));

        observations.push_back(std::move(obs));
    }

    if (observations.size() < 5) {
        return bt::Status::Error(bt::StatusCode::ValidationError, "Stereo calibration needs at least five paired chessboard detections");
    }
    return observations;
}

int RunCalibrateIntrinsics(int argc, char** argv) {
    if (argc != 8) {
        PrintUsage();
        return 1;
    }

    const std::string target = argv[2];
    const auto board_w = ParseInt(argv[4], "board_w");
    const auto board_h = ParseInt(argv[5], "board_h");
    const auto square = ParseFloat(argv[6], "square_m");
    if (!board_w.ok()) return PrintStatusAndReturn(board_w.status());
    if (!board_h.ok()) return PrintStatusAndReturn(board_h.status());
    if (!square.ok()) return PrintStatusAndReturn(square.status());

    const auto frames = LoadImagesFromDirectory(argv[3]);
    if (!frames.ok()) {
        return PrintStatusAndReturn(frames.status());
    }

    const auto camera = bt::CalibrateIntrinsicsFromChessboardFrames(
        frames.value(),
        cv::Size(board_w.value(), board_h.value()),
        square.value());
    if (!camera.ok()) {
        return PrintStatusAndReturn(camera.status());
    }

    const std::filesystem::path out_path = argv[7];
    const auto bundle_result = LoadCalibrationForUpdate(out_path);
    if (!bundle_result.ok()) {
        return PrintStatusAndReturn(bundle_result.status());
    }
    bt::CalibrationBundle bundle = bundle_result.value();
    if (target == "camera_a") {
        bundle.camera_a.intrinsics_valid = camera.value().intrinsics_valid;
        bundle.camera_a.camera_matrix = camera.value().camera_matrix;
        bundle.camera_a.distortion = camera.value().distortion;
    } else if (target == "camera_b") {
        bundle.camera_b.intrinsics_valid = camera.value().intrinsics_valid;
        bundle.camera_b.camera_matrix = camera.value().camera_matrix;
        bundle.camera_b.distortion = camera.value().distortion;
    } else {
        return PrintStatusAndReturn(bt::Status::Error(bt::StatusCode::InvalidArgument, "Target must be camera_a or camera_b"));
    }

    const auto save = bt::SaveCalibrationBundle(bundle, out_path);
    if (!save.ok()) {
        return PrintStatusAndReturn(save);
    }
    std::cout << "Saved " << target << " intrinsics to " << out_path.string() << '\n';
    std::cout << bt::EvaluateCalibrationReadiness(bundle).summary << '\n';
    return 0;
}

int RunCalibrateStereo(int argc, char** argv) {
    if (argc != 8) {
        PrintUsage();
        return 1;
    }

    const auto board_w = ParseInt(argv[4], "board_w");
    const auto board_h = ParseInt(argv[5], "board_h");
    const auto square = ParseFloat(argv[6], "square_m");
    if (!board_w.ok()) return PrintStatusAndReturn(board_w.status());
    if (!board_h.ok()) return PrintStatusAndReturn(board_h.status());
    if (!square.ok()) return PrintStatusAndReturn(square.status());

    const std::filesystem::path calib_path = argv[7];
    const auto bundle_result = LoadCalibrationForUpdate(calib_path);
    if (!bundle_result.ok()) {
        return PrintStatusAndReturn(bundle_result.status());
    }

    cv::Size image_size;
    const auto observations = LoadStereoObservationsFromDirectories(
        argv[2],
        argv[3],
        cv::Size(board_w.value(), board_h.value()),
        &image_size);
    if (!observations.ok()) {
        return PrintStatusAndReturn(observations.status());
    }

    const auto calibrated = bt::CalibrateStereoExtrinsicsFromChessboardObservations(
        bundle_result.value(),
        image_size,
        cv::Size(board_w.value(), board_h.value()),
        square.value(),
        observations.value());
    if (!calibrated.ok()) {
        return PrintStatusAndReturn(calibrated.status());
    }

    const auto save = bt::SaveCalibrationBundle(calibrated.value(), calib_path);
    if (!save.ok()) {
        return PrintStatusAndReturn(save);
    }
    std::cout << "Saved stereo extrinsics to " << calib_path.string() << '\n';
    std::cout << bt::EvaluateCalibrationReadiness(calibrated.value()).summary << '\n';
    return 0;
}

int RunCalibrateFloor(int argc, char** argv) {
    if (argc < 12 || ((argc - 3) % 3) != 0) {
        PrintUsage();
        return 1;
    }

    const std::filesystem::path calib_path = argv[2];
    std::vector<bt::Vec3f> points;
    for (int i = 3; i + 2 < argc; i += 3) {
        const auto x = ParseFloat(argv[i], "floor x");
        const auto y = ParseFloat(argv[i + 1], "floor y");
        const auto z = ParseFloat(argv[i + 2], "floor z");
        if (!x.ok()) return PrintStatusAndReturn(x.status());
        if (!y.ok()) return PrintStatusAndReturn(y.status());
        if (!z.ok()) return PrintStatusAndReturn(z.status());
        points.push_back(bt::Vec3f{x.value(), y.value(), z.value()});
    }

    const auto plane = bt::EstimateFloorPlaneFromWorldPoints(points);
    if (!plane.ok()) {
        return PrintStatusAndReturn(plane.status());
    }
    const auto bundle_result = LoadCalibrationForUpdate(calib_path);
    if (!bundle_result.ok()) {
        return PrintStatusAndReturn(bundle_result.status());
    }
    bt::CalibrationBundle bundle = bundle_result.value();
    bundle.floor = plane.value();

    const auto save = bt::SaveCalibrationBundle(bundle, calib_path);
    if (!save.ok()) {
        return PrintStatusAndReturn(save);
    }
    std::cout << "Saved floor plane to " << calib_path.string() << '\n';
    std::cout << bt::EvaluateCalibrationReadiness(bundle).summary << '\n';
    return 0;
}


int RunCalibrateFloorGeometry(int argc, char** argv) {
    if (argc < 6 || argc > 10) {
        PrintUsage();
        return 1;
    }

    const std::filesystem::path calib_path = argv[2];
    const std::filesystem::path lines_path = argv[3];
    const auto image_w = ParseInt(argv[4], "image_w");
    const auto image_h = ParseInt(argv[5], "image_h");
    if (!image_w.ok()) return PrintStatusAndReturn(image_w.status());
    if (!image_h.ok()) return PrintStatusAndReturn(image_h.status());

    bt::FloorGeometryCalibrationOptions options;
    if (argc >= 7) options.floor_type = argv[6];
    if (argc >= 8) {
        const auto spacing = ParseFloat(argv[7], "spacing_m");
        if (!spacing.ok()) return PrintStatusAndReturn(spacing.status());
        options.family_a_spacing_m = spacing.value();
    }
    if (argc >= 9) {
        const auto spacing = ParseFloat(argv[8], "second_axis_spacing_m");
        if (!spacing.ok()) return PrintStatusAndReturn(spacing.status());
        options.family_b_spacing_m = spacing.value();
    }
    if (argc >= 10) {
        const auto height = ParseFloat(argv[9], "camera_height_m");
        if (!height.ok()) return PrintStatusAndReturn(height.status());
        options.camera_height_m = height.value();
    }

    std::ifstream in(lines_path);
    if (!in) {
        return PrintStatusAndReturn(bt::Status::Error(bt::StatusCode::InvalidArgument, "floor geometry line JSON is not readable"));
    }
    nlohmann::json j;
    try {
        in >> j;
    } catch (const std::exception& e) {
        return PrintStatusAndReturn(bt::Status::Error(bt::StatusCode::ValidationError, std::string("line JSON parse failed: ") + e.what()));
    }
    if (!j.is_array()) {
        return PrintStatusAndReturn(bt::Status::Error(bt::StatusCode::ValidationError, "line JSON must be an array"));
    }
    std::vector<bt::FloorSeamLine2D> lines;
    for (const auto& item : j) {
        if (!item.is_object() || !item.contains("a") || !item.contains("b") ||
            !item["a"].is_array() || !item["b"].is_array() || item["a"].size() != 2 || item["b"].size() != 2) {
            continue;
        }
        bt::FloorSeamLine2D line;
        line.a = bt::Vec2f{item["a"][0].get<float>(), item["a"][1].get<float>()};
        line.b = bt::Vec2f{item["b"][0].get<float>(), item["b"][1].get<float>()};
        line.strength = item.value("strength", 1.0f);
        if (item.contains("samples") && item["samples"].is_array()) {
            for (const auto& p : item["samples"]) {
                if (p.is_array() && p.size() == 2 && p[0].is_number() && p[1].is_number()) {
                    line.samples.push_back(bt::Vec2f{p[0].get<float>(), p[1].get<float>()});
                }
            }
        }
        if (line.samples.empty() && item.contains("points") && item["points"].is_array()) {
            for (const auto& p : item["points"]) {
                if (p.is_array() && p.size() == 2 && p[0].is_number() && p[1].is_number()) {
                    line.samples.push_back(bt::Vec2f{p[0].get<float>(), p[1].get<float>()});
                }
            }
        }
        lines.push_back(line);
    }
    if (lines.size() < 3) {
        return PrintStatusAndReturn(bt::Status::Error(bt::StatusCode::ValidationError, "floor geometry calibration needs at least three candidate seam lines"));
    }

    const auto bundle_result = LoadCalibrationForUpdate(calib_path);
    if (!bundle_result.ok()) {
        return PrintStatusAndReturn(bundle_result.status());
    }
    bt::CalibrationBundle bundle = bundle_result.value();
    const auto floor_geometry = bt::EstimateFloorGeometryCalibration(lines, image_w.value(), image_h.value(), options);
    bundle.floor_geometry = floor_geometry.calibration;
    bundle.floor_geometry.source = "cli_lines";
    if (bundle.floor_geometry.floor_plane.valid) {
        bundle.floor = bundle.floor_geometry.floor_plane;
    }
    const auto save = bt::SaveCalibrationBundle(bundle, calib_path);
    if (!save.ok()) {
        return PrintStatusAndReturn(save);
    }
    std::cout << "Saved floor geometry calibration to " << calib_path.string() << '\n'
              << "families=" << bundle.floor_geometry.family_count
              << " confidence=" << bundle.floor_geometry.metric_scale_confidence
              << " reason=" << bundle.floor_geometry.reason << '\n';
    std::cout << bt::EvaluateCalibrationReadiness(bundle).summary << '\n';
    return 0;
}








int RunAlignFloorGeometry(int argc, char** argv) {
    if (argc != 4) {
        PrintUsage();
        return 1;
    }

    const std::filesystem::path camera_a_path = argv[2];
    const std::filesystem::path camera_b_path = argv[3];
    auto a_result = LoadCalibrationForUpdate(camera_a_path);
    if (!a_result.ok()) return PrintStatusAndReturn(a_result.status());
    auto b_result = LoadCalibrationForUpdate(camera_b_path);
    if (!b_result.ok()) return PrintStatusAndReturn(b_result.status());

    auto bundle_a = a_result.value();
    const auto& bundle_b = b_result.value();
    const auto alignment = bt::EstimateMultiCameraFloorAlignment(bundle_a.floor_geometry, bundle_b.floor_geometry);
    bundle_a.floor_geometry.source = bundle_a.floor_geometry.source.empty() || bundle_a.floor_geometry.source == "unknown"
        ? std::string("multi_camera_alignment")
        : bundle_a.floor_geometry.source + "+multi_camera_alignment";
    bundle_a.floor_geometry.multi_camera_alignment_valid = alignment.valid;
    bundle_a.floor_geometry.multi_camera_alignment_confidence = alignment.confidence;
    bundle_a.floor_geometry.multi_camera_yaw_delta_rad = alignment.yaw_delta_rad;
    bundle_a.floor_geometry.multi_camera_pitch_delta_rad = alignment.pitch_delta_rad;
    bundle_a.floor_geometry.multi_camera_roll_delta_rad = alignment.roll_delta_rad;
    bundle_a.floor_geometry.multi_camera_height_delta_m = alignment.height_delta_m;
    bundle_a.floor_geometry.multi_camera_scale_ratio = alignment.scale_ratio;
    bundle_a.floor_geometry.shared_floor_frame_valid = alignment.shared_floor_frame_valid;
    bundle_a.floor_geometry.shared_floor_transform = alignment.floor_b_from_floor_a;
    if (!alignment.reason.empty()) {
        bundle_a.floor_geometry.multi_camera_warning = alignment.reason;
    }

    const auto save = bt::SaveCalibrationBundle(bundle_a, camera_a_path);
    if (!save.ok()) return PrintStatusAndReturn(save);

    std::cout << "Saved floor geometry alignment to " << camera_a_path.string() << '\n'
              << "valid=" << (alignment.valid ? "true" : "false")
              << " confidence=" << alignment.confidence
              << " shared_floor_frame=" << (alignment.shared_floor_frame_valid ? "true" : "false")
              << " reason=" << alignment.reason << '\n';
    return 0;
}


int RunSetBody(int argc, char** argv) {
    if (argc != 10) {
        PrintUsage();
        return 1;
    }

    std::array<float, 7> values{};
    const char* labels[7] = {"pelvis_w", "left_femur", "right_femur", "left_tibia", "right_tibia", "left_foot", "right_foot"};
    for (int i = 0; i < 7; ++i) {
        const auto parsed = ParseFloat(argv[3 + i], labels[i]);
        if (!parsed.ok()) {
            return PrintStatusAndReturn(parsed.status());
        }
        if (parsed.value() <= 0.0f) {
            return PrintStatusAndReturn(bt::Status::Error(bt::StatusCode::InvalidArgument, std::string(labels[i]) + " must be positive"));
        }
        values[static_cast<std::size_t>(i)] = parsed.value();
    }

    const std::filesystem::path calib_path = argv[2];
    const auto bundle_result = LoadCalibrationForUpdate(calib_path);
    if (!bundle_result.ok()) {
        return PrintStatusAndReturn(bundle_result.status());
    }
    bt::CalibrationBundle bundle = bundle_result.value();
    bundle.body.standing_neutral_valid = true;
    bundle.body.pelvis_width = values[0];
    bundle.body.left_femur = values[1];
    bundle.body.right_femur = values[2];
    bundle.body.left_tibia = values[3];
    bundle.body.right_tibia = values[4];
    bundle.body.left_foot_length = values[5];
    bundle.body.right_foot_length = values[6];
    bundle.body.quality.pelvis_width = 1.0f;
    bundle.body.quality.left_femur = 1.0f;
    bundle.body.quality.right_femur = 1.0f;
    bundle.body.quality.left_tibia = 1.0f;
    bundle.body.quality.right_tibia = 1.0f;
    bundle.body.quality.left_foot_length = 1.0f;
    bundle.body.quality.right_foot_length = 1.0f;
    bundle.body.quality.standing_hmd_to_pelvis = 0.0f;
    bundle.body.quality.overall = 0.90f;
    bundle.body.quality.sample_count = 0;
    bundle.body.quality.source = "manual_cli";

    const auto save = bt::SaveCalibrationBundle(bundle, calib_path);
    if (!save.ok()) {
        return PrintStatusAndReturn(save);
    }
    std::cout << "Saved body dimensions to " << calib_path.string() << '\n';
    std::cout << bt::EvaluateCalibrationReadiness(bundle).summary << '\n';
    return 0;
}

int RunCalibrationStatus(int argc, char** argv) {
    if (argc != 3) {
        PrintUsage();
        return 1;
    }
    const auto bundle = bt::LoadCalibration(argv[2]);
    if (!bundle.ok()) {
        return PrintStatusAndReturn(bundle.status());
    }
    std::cout << bt::EvaluateCalibrationReadiness(bundle.value()).summary << '\n';
    return 0;
}



std::filesystem::path NumberedImagePath(const std::filesystem::path& dir, const std::string& prefix, int index) {
    std::ostringstream name;
    name << prefix << "_" << std::setw(4) << std::setfill('0') << index << ".png";
    return dir / name.str();
}

bool DetectChessboard(const cv::Mat& bgr, cv::Size board_size) {
    if (bgr.empty()) {
        return false;
    }
    cv::Mat gray;
    if (bgr.channels() == 1) {
        gray = bgr;
    } else {
        cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
    }
    std::vector<cv::Point2f> corners;
    return cv::findChessboardCorners(
        gray,
        board_size,
        corners,
        cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);
}

void ConfigureCapture(cv::VideoCapture& cap, int width, int height, int fps) {
    if (width > 0) {
        cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
    }
    if (height > 0) {
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    }
    if (fps > 0) {
        cap.set(cv::CAP_PROP_FPS, fps);
    }
}

int RunCaptureChessboard(int argc, char** argv) {
    if (argc != 7 && argc != 11) {
        PrintUsage();
        return 1;
    }

    const auto camera_index = ParseInt(argv[2], "camera_index");
    const auto board_w = ParseInt(argv[4], "board_w");
    const auto board_h = ParseInt(argv[5], "board_h");
    const auto target_count = ParseInt(argv[6], "target_count");
    if (!camera_index.ok()) return PrintStatusAndReturn(camera_index.status());
    if (!board_w.ok()) return PrintStatusAndReturn(board_w.status());
    if (!board_h.ok()) return PrintStatusAndReturn(board_h.status());
    if (!target_count.ok()) return PrintStatusAndReturn(target_count.status());
    if (board_w.value() <= 1 || board_h.value() <= 1 || target_count.value() <= 0) {
        return PrintStatusAndReturn(bt::Status::Error(bt::StatusCode::InvalidArgument, "Invalid board size or target count"));
    }

    int width = 1280;
    int height = 720;
    int fps = 30;
    int min_interval_ms = 700;
    if (argc == 11) {
        const auto parsed_width = ParseInt(argv[7], "width");
        const auto parsed_height = ParseInt(argv[8], "height");
        const auto parsed_fps = ParseInt(argv[9], "fps");
        const auto parsed_interval = ParseInt(argv[10], "min_interval_ms");
        if (!parsed_width.ok()) return PrintStatusAndReturn(parsed_width.status());
        if (!parsed_height.ok()) return PrintStatusAndReturn(parsed_height.status());
        if (!parsed_fps.ok()) return PrintStatusAndReturn(parsed_fps.status());
        if (!parsed_interval.ok()) return PrintStatusAndReturn(parsed_interval.status());
        width = parsed_width.value();
        height = parsed_height.value();
        fps = parsed_fps.value();
        min_interval_ms = parsed_interval.value();
    }

    const std::filesystem::path out_dir = argv[3];
    {
        std::error_code mkdir_ec;
        std::filesystem::create_directories(out_dir, mkdir_ec);
        if (mkdir_ec) {
            return PrintStatusAndReturn(bt::Status::Error(bt::StatusCode::InvalidArgument, "Could not create capture output directory: " + mkdir_ec.message()));
        }
    }

#ifdef _WIN32
    constexpr int capture_api = cv::CAP_DSHOW;
#else
    constexpr int capture_api = cv::CAP_ANY;
#endif
    cv::VideoCapture cap(camera_index.value(), capture_api);
    if (!cap.isOpened()) {
        return PrintStatusAndReturn(bt::Status::Error(bt::StatusCode::DeviceUnavailable, "Could not open camera"));
    }
    ConfigureCapture(cap, width, height, fps);

    std::signal(SIGINT, HandleSignal);
    const cv::Size board_size(board_w.value(), board_h.value());
    auto last_saved = std::chrono::steady_clock::now() - std::chrono::milliseconds(min_interval_ms);
    int saved = 0;
    int seen_frames = 0;
    std::cout << "Capturing chessboard frames. Move the board between saves; Ctrl+C stops early.\n";

    while (!g_stop_requested && saved < target_count.value()) {
        cv::Mat frame;
        if (!cap.read(frame) || frame.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            continue;
        }
        ++seen_frames;
        const auto now = std::chrono::steady_clock::now();
        if (DetectChessboard(frame, board_size)) {
            cv::Mat preview = frame.clone();
            std::vector<cv::Point2f> corners;
            cv::findChessboardCorners(preview, board_size, corners, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);
            cv::drawChessboardCorners(preview, board_size, corners, true);
            cv::imshow("Calibration Preview", preview);

            if (now - last_saved >= std::chrono::milliseconds(min_interval_ms)) {
                const auto path = NumberedImagePath(out_dir, "chessboard", saved + 1);
                if (!cv::imwrite(path.string(), frame)) {
                    return PrintStatusAndReturn(bt::Status::Error(bt::StatusCode::InternalError, "Failed to write image: " + path.string()));
                }
                ++saved;
                last_saved = now;
                std::cout << "saved " << saved << "/" << target_count.value() << ": " << path.string() << '\n';
            }
        } else {
            cv::imshow("Calibration Preview", frame);
        }

        const int key = cv::waitKey(1);
        if (key == 27) { // ESC key
            break;
        }
    }

    std::cout << "Captured " << saved << " usable frames from " << seen_frames << " camera frames into " << out_dir.string() << '\n';
    return saved > 0 ? 0 : 1;
}

int RunCaptureStereoChessboard(int argc, char** argv) {
    if (argc != 9 && argc != 13) {
        PrintUsage();
        return 1;
    }

    const auto camera_a_index = ParseInt(argv[2], "camera_a_index");
    const auto camera_b_index = ParseInt(argv[3], "camera_b_index");
    const auto board_w = ParseInt(argv[6], "board_w");
    const auto board_h = ParseInt(argv[7], "board_h");
    const auto target_count = ParseInt(argv[8], "target_count");
    if (!camera_a_index.ok()) return PrintStatusAndReturn(camera_a_index.status());
    if (!camera_b_index.ok()) return PrintStatusAndReturn(camera_b_index.status());
    if (!board_w.ok()) return PrintStatusAndReturn(board_w.status());
    if (!board_h.ok()) return PrintStatusAndReturn(board_h.status());
    if (!target_count.ok()) return PrintStatusAndReturn(target_count.status());
    if (board_w.value() <= 1 || board_h.value() <= 1 || target_count.value() <= 0) {
        return PrintStatusAndReturn(bt::Status::Error(bt::StatusCode::InvalidArgument, "Invalid board size or target count"));
    }

    int width = 1280;
    int height = 720;
    int fps = 30;
    int min_interval_ms = 900;
    if (argc == 13) {
        const auto parsed_width = ParseInt(argv[9], "width");
        const auto parsed_height = ParseInt(argv[10], "height");
        const auto parsed_fps = ParseInt(argv[11], "fps");
        const auto parsed_interval = ParseInt(argv[12], "min_interval_ms");
        if (!parsed_width.ok()) return PrintStatusAndReturn(parsed_width.status());
        if (!parsed_height.ok()) return PrintStatusAndReturn(parsed_height.status());
        if (!parsed_fps.ok()) return PrintStatusAndReturn(parsed_fps.status());
        if (!parsed_interval.ok()) return PrintStatusAndReturn(parsed_interval.status());
        width = parsed_width.value();
        height = parsed_height.value();
        fps = parsed_fps.value();
        min_interval_ms = parsed_interval.value();
    }

    const std::filesystem::path out_a = argv[4];
    const std::filesystem::path out_b = argv[5];
    {
        std::error_code mkdir_ec;
        std::filesystem::create_directories(out_a, mkdir_ec);
        if (mkdir_ec) {
            return PrintStatusAndReturn(bt::Status::Error(bt::StatusCode::InvalidArgument, "Could not create camera A output directory: " + mkdir_ec.message()));
        }
        mkdir_ec.clear();
        std::filesystem::create_directories(out_b, mkdir_ec);
        if (mkdir_ec) {
            return PrintStatusAndReturn(bt::Status::Error(bt::StatusCode::InvalidArgument, "Could not create camera B output directory: " + mkdir_ec.message()));
        }
    }

#ifdef _WIN32
    constexpr int capture_api = cv::CAP_DSHOW;
#else
    constexpr int capture_api = cv::CAP_ANY;
#endif
    cv::VideoCapture cap_a(camera_a_index.value(), capture_api);
    cv::VideoCapture cap_b(camera_b_index.value(), capture_api);
    if (!cap_a.isOpened() || !cap_b.isOpened()) {
        return PrintStatusAndReturn(bt::Status::Error(bt::StatusCode::DeviceUnavailable, "Could not open both stereo cameras"));
    }
    ConfigureCapture(cap_a, width, height, fps);
    ConfigureCapture(cap_b, width, height, fps);

    std::signal(SIGINT, HandleSignal);
    const cv::Size board_size(board_w.value(), board_h.value());
    auto last_saved = std::chrono::steady_clock::now() - std::chrono::milliseconds(min_interval_ms);
    int saved = 0;
    int seen_pairs = 0;
    std::cout << "Capturing paired stereo chessboard frames. Move the board between saves; Ctrl+C stops early.\n";

    while (!g_stop_requested && saved < target_count.value()) {
        cv::Mat frame_a;
        cv::Mat frame_b;
        if (!cap_a.read(frame_a) || !cap_b.read(frame_b) || frame_a.empty() || frame_b.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            continue;
        }
        ++seen_pairs;
        const auto now = std::chrono::steady_clock::now();
        const bool found_a = DetectChessboard(frame_a, board_size);
        const bool found_b = DetectChessboard(frame_b, board_size);

        cv::Mat preview_a = frame_a.clone();
        cv::Mat preview_b = frame_b.clone();

        if (found_a) {
            std::vector<cv::Point2f> corners_a;
            cv::findChessboardCorners(preview_a, board_size, corners_a, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);
            cv::drawChessboardCorners(preview_a, board_size, corners_a, true);
        }
        if (found_b) {
            std::vector<cv::Point2f> corners_b;
            cv::findChessboardCorners(preview_b, board_size, corners_b, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);
            cv::drawChessboardCorners(preview_b, board_size, corners_b, true);
        }

        cv::Mat preview_combined;
        cv::hconcat(preview_a, preview_b, preview_combined);

        // Scale down for reasonable preview size on screen
        cv::Mat scaled_preview;
        cv::resize(preview_combined, scaled_preview, cv::Size(), 0.5, 0.5);
        cv::imshow("Stereo Calibration Preview", scaled_preview);

        if (found_a && found_b && now - last_saved >= std::chrono::milliseconds(min_interval_ms)) {
            const int index = saved + 1;
            const auto path_a = NumberedImagePath(out_a, "stereo_a", index);
            const auto path_b = NumberedImagePath(out_b, "stereo_b", index);
            if (!cv::imwrite(path_a.string(), frame_a) || !cv::imwrite(path_b.string(), frame_b)) {
                return PrintStatusAndReturn(bt::Status::Error(bt::StatusCode::InternalError, "Failed to write stereo calibration image pair"));
            }
            ++saved;
            last_saved = now;
            std::cout << "saved pair " << saved << "/" << target_count.value() << '\n';
        }

        const int key = cv::waitKey(1);
        if (key == 27) { // ESC key
            break;
        }
    }

    std::cout << "Captured " << saved << " usable stereo pairs from " << seen_pairs << " camera pairs into "
              << out_a.string() << " and " << out_b.string() << '\n';
    return saved > 0 ? 0 : 1;
}


struct ReplaySolveFrame {
    double timestamp_seconds = 0.0;
    bt::DecodedPose2D camera_a_pose{};
    bt::DecodedPose2D camera_b_pose{};
    bt::ReliabilitySummary camera_a_reliability{};
    bt::ReliabilitySummary camera_b_reliability{};
    bt::HmdPoseSample hmd{};
    bool camera_b_pose_present = false;
};

bt::TrackingMode ReplayTrackingMode(const std::vector<ReplaySolveFrame>& frames) {
    for (const auto& frame : frames) {
        if (frame.camera_b_pose_present) {
            return bt::TrackingMode::Stereo;
        }
    }
    return bt::TrackingMode::Monocular;
}


bt::TrackingConfig MakeReplayTrackingConfig(
    bt::TrackingMode mode,
    const bt::CalibrationBundle& calibration,
    const bt::TrackingConfig* recorded_tracking = nullptr) {

    bt::TrackingConfig cfg = recorded_tracking ? *recorded_tracking : bt::TrackingConfig{};
    cfg.mode = mode;
    if (mode == bt::TrackingMode::Monocular && calibration.floor_geometry.valid) {
        cfg.monocular.floor_geometry_calibration_enabled = true;
        cfg.monocular.floor_geometry_type = calibration.floor_geometry.floor_type;
        cfg.monocular.floor_geometry_confidence = calibration.floor_geometry.metric_scale_confidence;
        cfg.monocular.floor_scale_assist_enabled = calibration.floor_geometry.family_a.metric_spacing_valid;
        cfg.monocular.floor_projective_homography_enabled = calibration.floor_geometry.homography_valid;
        cfg.monocular.floor_from_image = calibration.floor_geometry.floor_from_image;
        cfg.monocular.image_from_floor = calibration.floor_geometry.image_from_floor;
        cfg.monocular.floor_projective_confidence = calibration.floor_geometry.metric_scale_confidence;
        cfg.monocular.floor_distortion_correction_enabled = calibration.floor_geometry.distortion.valid;
        cfg.monocular.floor_distortion_confidence = calibration.floor_geometry.distortion.confidence;
        cfg.monocular.floor_radial_k1 = calibration.floor_geometry.distortion.radial_k1;
        cfg.monocular.floor_radial_k2 = calibration.floor_geometry.distortion.radial_k2;
        cfg.monocular.floor_tangential_p1 = calibration.floor_geometry.distortion.tangential_p1;
        cfg.monocular.floor_tangential_p2 = calibration.floor_geometry.distortion.tangential_p2;
        cfg.monocular.floor_camera_orientation_enabled = calibration.floor_geometry.camera_orientation_valid;
        cfg.monocular.floor_camera_pitch_rad = calibration.floor_geometry.camera_pitch_rad;
        cfg.monocular.floor_camera_roll_rad = calibration.floor_geometry.camera_roll_rad;
        cfg.monocular.floor_camera_orientation_confidence = calibration.floor_geometry.camera_orientation_confidence;
        if (calibration.floor_geometry.camera_height_valid && calibration.floor_geometry.camera_height_m > 0.10f) {
            cfg.monocular.camera_height_m = calibration.floor_geometry.camera_height_m;
        }
    }
    return cfg;
}

bt::Result<bt::AppConfig> LoadRecordedAppConfigFromReplay(const std::filesystem::path& replay_path) {
    bt::ReplayPlayer player;
    const auto recording = player.LoadRecording(replay_path);
    if (!recording.ok()) {
        return recording.status();
    }
    if (recording.value().metadata.config_json.empty()) {
        return bt::Status::Error(
            bt::StatusCode::FailedPrecondition,
            "Replay manifest did not contain recorded config JSON");
    }

    std::error_code temp_ec;
    const auto temp_dir = std::filesystem::temp_directory_path(temp_ec);
    if (temp_ec) {
        return bt::Status::Error(
            bt::StatusCode::InternalError,
            "Could not locate temp directory for recorded replay config: " + temp_ec.message());
    }
    const auto temp_path = temp_dir / ("bodytracker_replay_config_" + std::to_string(bt::NowQpc().ticks) + ".json");
    {
        std::ofstream out(temp_path, std::ios::out | std::ios::trunc);
        if (!out) {
            return bt::Status::Error(
                bt::StatusCode::InternalError,
                "Could not write temporary recorded replay config");
        }
        out << recording.value().metadata.config_json;
    }

    auto cfg = bt::LoadConfig(temp_path);
    std::error_code remove_ec;
    std::filesystem::remove(temp_path, remove_ec);
    return cfg;
}

bt::RtmPoseOutputFormat PoseFormatFromString(const std::string& value) {
    if (value == "simcc") {
        return bt::RtmPoseOutputFormat::SimCC;
    }
    if (value == "xyc") {
        return bt::RtmPoseOutputFormat::KeypointsXYConfidence;
    }
    if (value == "rtmw_wholebody133_simcc") {
        return bt::RtmPoseOutputFormat::RtmwWholeBody133SimCC;
    }
    if (value == "rtmw3d_wholebody133_simcc") {
        return bt::RtmPoseOutputFormat::Rtmw3dWholeBody133SimCC;
    }
    return bt::RtmPoseOutputFormat::Unknown;
}

bool ReadPixel2(const nlohmann::json& j, bt::Vec2f* out) {
    if (!j.is_array() || j.size() != 2 || !j[0].is_number() || !j[1].is_number()) {
        return false;
    }
    out->x = j[0].get<float>();
    out->y = j[1].get<float>();
    return std::isfinite(out->x) && std::isfinite(out->y);
}

bool ReadVec3ForReplay(const nlohmann::json& j, bt::Vec3f* out) {
    if (!j.is_array() || j.size() != 3 || !j[0].is_number() || !j[1].is_number() || !j[2].is_number()) {
        return false;
    }
    bt::Vec3f value{
        j[0].get<float>(),
        j[1].get<float>(),
        j[2].get<float>()};
    if (!std::isfinite(value.x) || !std::isfinite(value.y) || !std::isfinite(value.z)) {
        return false;
    }
    *out = value;
    return true;
}

bool ReadQuatForReplay(const nlohmann::json& j, bt::Quatf* out) {
    if (!j.is_array() || j.size() != 4 || !j[0].is_number() || !j[1].is_number() || !j[2].is_number() || !j[3].is_number()) {
        return false;
    }
    bt::Quatf value{
        j[0].get<float>(),
        j[1].get<float>(),
        j[2].get<float>(),
        j[3].get<float>()};
    if (!std::isfinite(value.x) || !std::isfinite(value.y) || !std::isfinite(value.z) || !std::isfinite(value.w)) {
        return false;
    }
    const float len2 = value.x * value.x + value.y * value.y + value.z * value.z + value.w * value.w;
    if (!std::isfinite(len2) || len2 < 1e-12f) {
        return false;
    }
    *out = bt::Normalize(value);
    return true;
}

float Clamp01ForReplay(float value) {
    if (!std::isfinite(value)) {
        return 0.0f;
    }
    return std::clamp(value, 0.0f, 1.0f);
}

float JsonFloat01Or(const nlohmann::json& j, const char* key, float fallback) {
    if (!j.is_object() || !j.contains(key) || !j.at(key).is_number()) {
        return fallback;
    }
    return Clamp01ForReplay(j.at(key).get<float>());
}

double JsonNumberOr(const nlohmann::json& j, const char* key, double fallback) {
    if (!j.is_object() || !j.contains(key) || !j.at(key).is_number()) {
        return fallback;
    }
    const double value = j.at(key).get<double>();
    return std::isfinite(value) ? value : fallback;
}

bool JsonBoolOr(const nlohmann::json& j, const char* key, bool fallback) {
    if (!j.is_object() || !j.contains(key) || !j.at(key).is_boolean()) {
        return fallback;
    }
    return j.at(key).get<bool>();
}

double ReplayStepDt(double timestamp_seconds, double* previous_timestamp_seconds) {
    constexpr double kDefaultReplayDtSeconds = 1.0 / 30.0;
    if (!previous_timestamp_seconds) {
        return kDefaultReplayDtSeconds;
    }
    if (!std::isfinite(timestamp_seconds) || timestamp_seconds <= 0.0) {
        return kDefaultReplayDtSeconds;
    }
    if (!std::isfinite(*previous_timestamp_seconds) || *previous_timestamp_seconds <= 0.0) {
        *previous_timestamp_seconds = timestamp_seconds;
        return kDefaultReplayDtSeconds;
    }
    if (timestamp_seconds <= *previous_timestamp_seconds) {
        return kDefaultReplayDtSeconds;
    }
    const double dt = std::clamp(timestamp_seconds - *previous_timestamp_seconds, 1.0 / 240.0, 0.10);
    *previous_timestamp_seconds = timestamp_seconds;
    return dt;
}

bt::DecodedPose2D ReadPose2dForReplay(const nlohmann::json& j) {
    bt::DecodedPose2D pose;
    if (!j.is_object() || !j.value("valid", false)) {
        return pose;
    }
    pose.valid = true;
    pose.format = PoseFormatFromString(j.value("format", ""));
    pose.aggregate_confidence = JsonFloat01Or(j, "aggregate_confidence", 0.0f);

    const auto keypoints = j.value("keypoints", nlohmann::json::array());
    if (!keypoints.is_array() || keypoints.size() != bt::kHalpe26Count) {
        pose.valid = false;
        return pose;
    }

    for (std::size_t i = 0; i < bt::kHalpe26Count; ++i) {
        const auto& src = keypoints[i];
        auto& dst = pose.keypoints[i];
        dst.present = src.value("present", false);
        dst.confidence = JsonFloat01Or(src, "confidence", 0.0f);
        if (src.contains("pixel")) {
            dst.present = dst.present && ReadPixel2(src["pixel"], &dst.pixel);
        }
    }
    return pose;
}

bt::ReliabilitySummary ReadReliabilityForReplay(const nlohmann::json& j) {
    bt::ReliabilitySummary r;
    if (!j.is_object()) {
        return r;
    }
    r.mean_weight = JsonFloat01Or(j, "mean_weight", 0.0f);
    r.lower_body_mean = JsonFloat01Or(j, "lower_body_mean", 0.0f);
    r.foot_mean = JsonFloat01Or(j, "foot_mean", 0.0f);

    const auto joints = j.value("joints", nlohmann::json::array());
    if (joints.is_array() && joints.size() == bt::kHalpe26Count) {
        for (std::size_t i = 0; i < bt::kHalpe26Count; ++i) {
            const auto& src = joints[i];
            auto& dst = r.joints[i];
            dst.usable = src.value("usable", false);
            dst.final_weight = JsonFloat01Or(src, "final_weight", 0.0f);
            dst.model_term = JsonFloat01Or(src, "model_term", 0.0f);
            dst.crop_edge_term = JsonFloat01Or(src, "crop_edge_term", 1.0f);
            dst.image_edge_term = JsonFloat01Or(src, "image_edge_term", 1.0f);
            dst.temporal_term = JsonFloat01Or(src, "temporal_term", 1.0f);
            dst.crop_stability_term = JsonFloat01Or(src, "crop_stability_term", 1.0f);
            dst.posture_mode_term = JsonFloat01Or(src, "posture_mode_term", 1.0f);
        }
    } else {
        const float fallback = Clamp01ForReplay(r.lower_body_mean > 0.0f ? r.lower_body_mean : r.mean_weight);
        for (auto& joint : r.joints) {
            joint.final_weight = fallback;
            joint.model_term = fallback;
            joint.usable = fallback >= 0.15f;
        }
    }
    return r;
}

bt::Result<std::vector<ReplaySolveFrame>> LoadReplaySolveFrames(const std::filesystem::path& path) {
    std::ifstream in(path);
    if (!in) {
        return bt::Status::Error(bt::StatusCode::InvalidArgument, "Replay file not readable: " + path.string());
    }

    std::vector<ReplaySolveFrame> frames;
    std::string line;
    int line_number = 0;
    while (std::getline(in, line)) {
        ++line_number;
        if (line.empty()) {
            continue;
        }

        nlohmann::json j;
        try {
            j = nlohmann::json::parse(line);
        } catch (const std::exception& e) {
            return bt::Status::Error(bt::StatusCode::ValidationError, "Replay JSON parse failed on line " + std::to_string(line_number) + ": " + e.what());
        }

        if (!j.contains("camera_a_pose")) {
            continue;
        }

        ReplaySolveFrame frame;
        frame.timestamp_seconds = JsonNumberOr(j, "timestamp_seconds", 0.0);
        frame.camera_a_pose = ReadPose2dForReplay(j["camera_a_pose"]);
        frame.camera_b_pose_present = j.contains("camera_b_pose") && j["camera_b_pose"].is_object();
        if (frame.camera_b_pose_present) {
            frame.camera_b_pose = ReadPose2dForReplay(j["camera_b_pose"]);
        }
        frame.camera_a_reliability = ReadReliabilityForReplay(j.value("camera_a_reliability_full", nlohmann::json::object()));
        frame.camera_b_reliability = frame.camera_b_pose_present
            ? ReadReliabilityForReplay(j.value("camera_b_reliability_full", nlohmann::json::object()))
            : bt::ReliabilitySummary{};

        const auto hmd = j.value("hmd", nlohmann::json::object());
        frame.hmd.valid = JsonBoolOr(hmd, "valid", false);
        frame.hmd.timestamp_seconds = JsonNumberOr(hmd, "timestamp_seconds", 0.0);
        if (!std::isfinite(frame.hmd.timestamp_seconds)) {
            frame.hmd.timestamp_seconds = 0.0;
            frame.hmd.valid = false;
        }
        if (hmd.contains("pose") && hmd["pose"].is_object()) {
            const auto& pose = hmd["pose"];
            const bool pose_valid =
                ReadVec3ForReplay(pose.value("position", nlohmann::json{}), &frame.hmd.pose.position) &&
                ReadQuatForReplay(pose.value("orientation", nlohmann::json{}), &frame.hmd.pose.orientation);
            frame.hmd.valid = frame.hmd.valid && pose_valid;
        } else {
            frame.hmd.valid = false;
        }

        if (frame.camera_a_pose.valid && (!frame.camera_b_pose_present || frame.camera_b_pose.valid)) {
            frames.push_back(frame);
        }
    }

    if (frames.empty()) {
        return bt::Status::Error(bt::StatusCode::ValidationError, "Replay contains no decoded pose frames. Record with enable_replay_recording=true using this build.");
    }
    const bool first_has_b = frames.front().camera_b_pose_present;
    for (const auto& frame : frames) {
        if (frame.camera_b_pose_present != first_has_b) {
            return bt::Status::Error(bt::StatusCode::ValidationError, "Replay mixes monocular and stereo decoded pose frames; split the recording before solving.");
        }
    }
    return frames;
}

int RunReplaySolve(int argc, char** argv) {
    if (argc != 4) {
        PrintUsage();
        return 1;
    }

    const auto calibration = bt::LoadCalibration(argv[2]);
    if (!calibration.ok()) {
        return PrintStatusAndReturn(calibration.status());
    }
    const auto frames = LoadReplaySolveFrames(argv[3]);
    if (!frames.ok()) {
        return PrintStatusAndReturn(frames.status());
    }
    const auto replay_mode = ReplayTrackingMode(frames.value());
    const auto readiness = bt::EvaluateCalibrationReadiness(calibration.value(), replay_mode);
    if (!readiness.tracking_ready) {
        std::cerr << "Replay solve continuing with degraded calibration: " << readiness.summary << '\n';
    }

    bt::TrackingPipeline pipeline(calibration.value());
    const auto recorded_config = LoadRecordedAppConfigFromReplay(argv[3]);
    const bt::TrackingConfig tracking_config = MakeReplayTrackingConfig(
        replay_mode,
        calibration.value(),
        recorded_config.ok() ? &recorded_config.value().tracking : nullptr);
    if (!recorded_config.ok()) {
        std::cerr << "Replay solve continuing with fallback tracking config: "
                  << recorded_config.status().message << '\n';
    }
    pipeline.SetParams(tracking_config);
    bt::TrackingPipelineSnapshot last;
    int ok_count = 0;
    int failed_count = 0;
    double previous_timestamp_seconds = 0.0;
    for (const auto& frame : frames.value()) {
        bt::BodySolveInputs inputs;
        inputs.camera_a_pose = frame.camera_a_pose;
        inputs.camera_b_pose = frame.camera_b_pose;
        inputs.camera_a_reliability = frame.camera_a_reliability;
        inputs.camera_b_reliability = frame.camera_b_reliability;
        inputs.hmd = frame.hmd;

        const double dt_seconds = ReplayStepDt(frame.timestamp_seconds, &previous_timestamp_seconds);
        const auto step = pipeline.Step(inputs, dt_seconds);
        if (step.ok() && step.value().degradation_mode == "nominal") {
            ++ok_count;
            last = step.value();
        } else {
            ++failed_count;
            last = pipeline.Snapshot();
        }
    }

    std::cout << "Replay solve frames=" << frames.value().size()
              << " nominal=" << ok_count
              << " failed=" << failed_count
              << " tracking_mode=" << bt::ToString(tracking_config.mode)
              << " final_mode=" << last.degradation_mode
              << " final_confidence=" << last.state.confidence
              << " posture=" << bt::ToString(last.state.posture_mode)
              << '\n';
    return ok_count > 0 ? 0 : 1;
}

double Percentile(std::vector<double> values, double q) {
    if (values.empty()) {
        return 0.0;
    }
    std::sort(values.begin(), values.end());
    const double scaled = std::clamp(q, 0.0, 1.0) * static_cast<double>(values.size() - 1);
    const auto index = static_cast<std::size_t>(std::lround(scaled));
    return values[index];
}

void HashMixByte(std::uint64_t& hash, unsigned char byte) {
    hash ^= static_cast<std::uint64_t>(byte);
    hash *= 1099511628211ull;
}

void HashMixInt(std::uint64_t& hash, std::int64_t value) {
    for (int i = 0; i < 8; ++i) {
        HashMixByte(hash, static_cast<unsigned char>((static_cast<std::uint64_t>(value) >> (i * 8)) & 0xff));
    }
}

void HashMixString(std::uint64_t& hash, const std::string& value) {
    for (const unsigned char byte : value) {
        HashMixByte(hash, byte);
    }
}

void HashMixPose(std::uint64_t& hash, const bt::Pose3f& pose) {
    HashMixInt(hash, static_cast<std::int64_t>(std::llround(pose.position.x * 1000000.0)));
    HashMixInt(hash, static_cast<std::int64_t>(std::llround(pose.position.y * 1000000.0)));
    HashMixInt(hash, static_cast<std::int64_t>(std::llround(pose.position.z * 1000000.0)));
}

int RunBenchmarkReplay(int argc, char** argv) {
    if (argc != 4 && argc != 5) {
        PrintUsage();
        return 1;
    }

    int iterations = 1;
    if (argc == 5) {
        const auto parsed = ParseInt(argv[4], "iterations");
        if (!parsed.ok()) {
            return PrintStatusAndReturn(parsed.status());
        }
        iterations = std::max(1, parsed.value());
    }

    const auto calibration = bt::LoadCalibration(argv[2]);
    if (!calibration.ok()) {
        return PrintStatusAndReturn(calibration.status());
    }
    const auto frames = LoadReplaySolveFrames(argv[3]);
    if (!frames.ok()) {
        return PrintStatusAndReturn(frames.status());
    }
    const auto replay_mode = ReplayTrackingMode(frames.value());
    const auto readiness = bt::EvaluateCalibrationReadiness(calibration.value(), replay_mode);
    if (!readiness.tracking_ready) {
        std::cerr << "Benchmark replay continuing with degraded calibration: " << readiness.summary << '\n';
    }

    std::vector<double> step_ms;
    step_ms.reserve(frames.value().size() * static_cast<std::size_t>(iterations));
    double sum_step_ms = 0.0;
    double sum_preliminary_ms = 0.0;
    double sum_final_ms = 0.0;
    double sum_confidence = 0.0;
    int nominal_count = 0;
    int failed_count = 0;
    int objective_evaluations = 0;
    int coordinate_passes = 0;
    std::uint64_t hash = 1469598103934665603ull;
    bt::TrackingPipelineSnapshot last;

    const auto recorded_config = LoadRecordedAppConfigFromReplay(argv[3]);
    const bt::TrackingConfig tracking_config = MakeReplayTrackingConfig(
        replay_mode,
        calibration.value(),
        recorded_config.ok() ? &recorded_config.value().tracking : nullptr);
    if (!recorded_config.ok()) {
        std::cerr << "Benchmark replay continuing with fallback tracking config: "
                  << recorded_config.status().message << '\n';
    }

    const auto bench_start = bt::NowQpc();
    for (int iteration = 0; iteration < iterations; ++iteration) {
        bt::TrackingPipeline pipeline(calibration.value());
        pipeline.SetParams(tracking_config);
        double previous_timestamp_seconds = 0.0;
        for (const auto& frame : frames.value()) {
            bt::BodySolveInputs inputs;
            inputs.camera_a_pose = frame.camera_a_pose;
            inputs.camera_b_pose = frame.camera_b_pose;
            inputs.camera_a_reliability = frame.camera_a_reliability;
            inputs.camera_b_reliability = frame.camera_b_reliability;
            inputs.hmd = frame.hmd;

            const double dt_seconds = ReplayStepDt(frame.timestamp_seconds, &previous_timestamp_seconds);

            const auto step_start = bt::NowQpc();
            const auto step = pipeline.Step(inputs, dt_seconds);
            const double elapsed_ms = bt::QpcDeltaSeconds(step_start, bt::NowQpc()) * 1000.0;
            step_ms.push_back(elapsed_ms);
            sum_step_ms += elapsed_ms;

            last = step.ok() ? step.value() : pipeline.Snapshot();
            const bool nominal = step.ok() && last.degradation_mode == "nominal";
            nominal ? ++nominal_count : ++failed_count;
            sum_preliminary_ms += last.solver.preliminary_solve_ms;
            sum_final_ms += last.solver.final_solve_ms;
            objective_evaluations += last.solver.objective_evaluations;
            coordinate_passes += last.solver.coordinate_passes;
            sum_confidence += last.state.confidence;
            HashMixString(hash, last.degradation_mode);
            HashMixInt(hash, static_cast<std::int64_t>(std::llround(last.state.confidence * 1000000.0)));
            HashMixPose(hash, last.state.root);
            HashMixPose(hash, last.state.left_foot);
            HashMixPose(hash, last.state.right_foot);
        }
    }
    const double wall_ms = bt::QpcDeltaSeconds(bench_start, bt::NowQpc()) * 1000.0;
    const double samples = static_cast<double>(std::max<std::size_t>(1, step_ms.size()));

    std::ostringstream hash_stream;
    hash_stream << std::hex << std::setw(16) << std::setfill('0') << hash;

    nlohmann::json out = {
        {"calibration", ResolveDisplayPath(argv[2])},
        {"replay", ResolveDisplayPath(argv[3])},
        {"iterations", iterations},
        {"frames", frames.value().size()},
        {"samples", step_ms.size()},
        {"nominal_count", nominal_count},
        {"failed_count", failed_count},
        {"step_ms", {
            {"avg", sum_step_ms / samples},
            {"p95", Percentile(step_ms, 0.95)},
            {"max", step_ms.empty() ? 0.0 : *std::max_element(step_ms.begin(), step_ms.end())}
        }},
        {"solver_ms", {
            {"preliminary_avg", sum_preliminary_ms / samples},
            {"final_avg", sum_final_ms / samples},
            {"total_avg", (sum_preliminary_ms + sum_final_ms) / samples}
        }},
        {"objective_evaluations", objective_evaluations},
        {"coordinate_passes", coordinate_passes},
        {"avg_confidence", sum_confidence / samples},
        {"final_mode", last.degradation_mode},
        {"final_confidence", last.state.confidence},
        {"regression_hash", hash_stream.str()},
        {"wall_ms", wall_ms}
    };

    std::cout << out.dump(2) << '\n';
    return nominal_count > 0 ? 0 : 1;
}

std::filesystem::path ReplayPath(const bt::AppConfig& cfg) {
    if (!cfg.debug.replay_log_path.empty()) {
        return cfg.debug.replay_log_path;
    }
    return cfg.app.recording_dir / "latest-runtime.ndjson";
}

std::string ReadTextFileForReplayManifest(const std::filesystem::path& path) {
    std::ifstream in(path);
    if (!in) {
        return {};
    }
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

bt::ReplayLogSessionInfo MakeReplaySessionInfo(
    const std::filesystem::path& config_path,
    const bt::AppConfig& cfg) {

    bt::ReplayLogSessionInfo session;
    session.schema_version = 3;
    session.config_path = config_path.string();
    session.config_json = ReadTextFileForReplayManifest(config_path);
    session.model_path = cfg.tracking.model_path.string();
    session.calibration_path = cfg.tracking.calibration_path.string();
    session.deterministic = true;
    return session;
}

double TimestampSeconds(bt::QpcTimestamp t) {
    return static_cast<double>(t.ticks) / 1000000000.0;
}

double FrameAgeMs(const bt::FramePacket& frame, bt::QpcTimestamp now) {
    return std::max(0.0, 1000.0 * bt::QpcDeltaSeconds(frame.timestamp, now));
}

float FrameAgeConfidenceScale(double frame_age_ms, double stale_timeout_ms) {
    if (!std::isfinite(frame_age_ms) || frame_age_ms <= 0.0) {
        return 1.0f;
    }
    const double reference_ms = std::isfinite(stale_timeout_ms) && stale_timeout_ms > 1.0
        ? stale_timeout_ms
        : 250.0;
    const double x = frame_age_ms / reference_ms;
    const double scale = 1.0 / (1.0 + x * x);
    return static_cast<float>(std::clamp(scale, 0.20, 1.0));
}

void ApplyFrameAgeConfidence(ViewProcessResult& view, double frame_age_ms, double stale_timeout_ms) {
    view.frame_age_ms = frame_age_ms;
    view.age_confidence_scale = FrameAgeConfidenceScale(frame_age_ms, stale_timeout_ms);
    if (view.age_confidence_scale >= 0.999f) {
        return;
    }

    view.pose.aggregate_confidence = std::clamp(
        view.pose.aggregate_confidence * view.age_confidence_scale,
        0.0f,
        1.0f);
    for (auto& kp : view.pose.keypoints) {
        if (kp.present) {
            kp.confidence = std::clamp(kp.confidence * view.age_confidence_scale, 0.0f, 1.0f);
        }
    }
    for (auto& joint : view.reliability.joints) {
        joint.final_weight = std::clamp(joint.final_weight * view.age_confidence_scale, 0.0f, 1.0f);
        // Deliberately do not flip joint.usable off because of age alone.
        // Age is quality metadata, not permission metadata.
    }
    view.reliability.mean_weight = std::clamp(view.reliability.mean_weight * view.age_confidence_scale, 0.0f, 1.0f);
    view.reliability.lower_body_mean = std::clamp(view.reliability.lower_body_mean * view.age_confidence_scale, 0.0f, 1.0f);
    view.reliability.foot_mean = std::clamp(view.reliability.foot_mean * view.age_confidence_scale, 0.0f, 1.0f);
}

std::string AgeDegradationSuffix(const ViewProcessResult& view, const char* camera_name) {
    if (view.age_confidence_scale >= 0.999f) {
        return {};
    }
    std::ostringstream oss;
    oss << camera_name
        << "_age_degraded:"
        << std::fixed << std::setprecision(1) << view.frame_age_ms
        << "ms@" << std::setprecision(2) << view.age_confidence_scale;
    return oss.str();
}

std::string JoinDegradation(std::string base, const std::string& suffix) {
    if (suffix.empty()) {
        return base;
    }
    if (base.empty() || base == "nominal") {
        return suffix;
    }
    return base + ";" + suffix;
}

bt::BodySolveQualityConfig MakeBodySolveQualityConfig(const bt::TrackingConfig& cfg) {
    bt::BodySolveQualityConfig quality;
    quality.tracking_mode = cfg.mode;
    quality.min_triangulated_seed_count = cfg.min_triangulated_seed_count;
    quality.max_mean_reprojection_error_px = static_cast<float>(cfg.max_mean_reprojection_error_px);
    quality.use_legacy_solver = cfg.use_legacy_solver;
    quality.monocular = cfg.monocular;
    return quality;
}

} // namespace

struct RuntimeRunOptions {
    bool show_native_dashboard = true;
    bool interactive_setup_on_failure = false;
    bt::WebRuntimeState* external_web_state = nullptr;
};

constexpr int kRuntimeRetryRequested = 100;

int RunRuntimeOnce(const std::filesystem::path& config_path, const RuntimeRunOptions& options) {
    std::signal(SIGINT, HandleSignal);
    std::signal(SIGTERM, HandleSignal);

    std::error_code config_exists_ec;
    const bool config_exists = std::filesystem::exists(config_path, config_exists_ec);
    if (config_exists_ec) {
        std::cerr << "Failed to inspect config path: " << config_exists_ec.message() << '\n';
        return 1;
    }
    if (!config_exists) {
        const auto s = bt::SaveDefaultConfig(config_path);
        if (!s.ok()) {
            std::cerr << s.message << '\n';
            return 1;
        }
    }

    const auto cfg_result = bt::LoadConfig(config_path);
    if (!cfg_result.ok()) {
        std::cerr << cfg_result.status().message << '\n';
        return 2;
    }

    const bt::AppConfig cfg = cfg_result.value();
    const bool monocular_mode = cfg.tracking.mode == bt::TrackingMode::Monocular;
    const bool stereo_monocular_fallback_enabled =
        cfg.tracking.mode == bt::TrackingMode::Stereo && cfg.tracking.stereo_monocular_fallback_enabled;
    const bt::BodySolveQualityConfig solve_quality = MakeBodySolveQualityConfig(cfg.tracking);
    bt::Logger::Instance().SetLogFile(cfg.app.log_file);
    bt::Logger::Instance().Write(bt::LogLevel::Info, "bodytracker starting");
    if (options.show_native_dashboard) {
        cv::namedWindow(kDashboardWindowName, cv::WINDOW_AUTOSIZE);
    }

    const auto fail_with_dashboard = [&](int code, const std::string& message, const std::string& mode) {
        bt::DebugSnapshot startup_debug;
        startup_debug.phase = "startup";
        startup_debug.degradation_mode = mode;
        startup_debug.last_error = message;
        SetOscConfigDefaults(startup_debug, cfg.osc);
        if (mode == "osc_open_failed") {
            startup_debug.osc_last_send_ok = false;
            startup_debug.osc_status = "open_failed";
            startup_debug.osc_last_error = message;
        }
        if (options.external_web_state) {
            options.external_web_state->SetConfig(cfg);
            options.external_web_state->SetDebug(startup_debug);
        }
        if (options.show_native_dashboard) {
            WaitOnNativeDashboard(startup_debug, "STARTUP BLOCKED / ESC OR Q TO CLOSE");
        }
        return code;
    };

    bt::WebRuntimeState owned_web_state;
    bt::WebRuntimeState& web_state = options.external_web_state ? *options.external_web_state : owned_web_state;
    web_state.SetConfig(cfg);

    auto inspect_exists = [](const std::filesystem::path& path, const char* label) -> bt::Result<bool> {
        std::error_code ec;
        const bool exists = std::filesystem::exists(path, ec);
        if (ec) {
            return bt::Status::Error(bt::StatusCode::InvalidArgument, std::string("failed to inspect ") + label + ": " + ec.message());
        }
        return exists;
    };

    bt::CalibrationBundle calibration_bundle;
    const auto calibration_exists_result = inspect_exists(cfg.tracking.calibration_path, "calibration path");
    if (!calibration_exists_result.ok()) {
        if (monocular_mode || stereo_monocular_fallback_enabled) {
            bt::Logger::Instance().Write(bt::LogLevel::Warn, calibration_exists_result.status().message);
        } else {
            bt::Logger::Instance().Write(bt::LogLevel::Error, calibration_exists_result.status().message);
            return fail_with_dashboard(4, calibration_exists_result.status().message, "calibration_path_inspect_failed");
        }
    }
    const bool calibration_exists = calibration_exists_result.ok() && calibration_exists_result.value();
    if (monocular_mode) {
        if (calibration_exists) {
            const auto calib_result = bt::LoadCalibration(cfg.tracking.calibration_path);
            if (calib_result.ok()) {
                calibration_bundle = calib_result.value();
            } else {
                bt::Logger::Instance().Write(
                    bt::LogLevel::Warn,
                    "monocular mode ignoring stereo calibration load failure: " + calib_result.status().message);
            }
        } else {
            bt::Logger::Instance().Write(
                bt::LogLevel::Info,
                "monocular mode using markerless camera profile; stereo calibration file is not required");
        }
    } else {
        if (!calibration_exists) {
            const auto s = bt::SaveCalibrationTemplate(cfg.tracking.calibration_path);
            if (!s.ok()) {
                if (stereo_monocular_fallback_enabled) {
                    bt::Logger::Instance().Write(
                        bt::LogLevel::Warn,
                        "stereo calibration template unavailable; continuing with stereo->monocular single-camera fallback: " +
                            s.message);
                } else {
                    bt::Logger::Instance().Write(bt::LogLevel::Error, s.message);
                    return fail_with_dashboard(4, s.message, "calibration_template_failed");
                }
            }
        }

        const auto calibration_exists_after_template = inspect_exists(cfg.tracking.calibration_path, "calibration path after template save");
        if (!calibration_exists_after_template.ok()) {
            if (stereo_monocular_fallback_enabled) {
                bt::Logger::Instance().Write(bt::LogLevel::Warn, calibration_exists_after_template.status().message);
            } else {
                bt::Logger::Instance().Write(bt::LogLevel::Error, calibration_exists_after_template.status().message);
                return fail_with_dashboard(5, calibration_exists_after_template.status().message, "calibration_path_inspect_failed");
            }
        } else if (calibration_exists_after_template.value()) {
            const auto calib_result = bt::LoadCalibration(cfg.tracking.calibration_path);
            if (calib_result.ok()) {
                calibration_bundle = calib_result.value();
            } else if (stereo_monocular_fallback_enabled) {
                bt::Logger::Instance().Write(
                    bt::LogLevel::Warn,
                    "stereo calibration load failed; continuing with stereo->monocular single-camera fallback: " +
                        calib_result.status().message);
            } else {
                bt::Logger::Instance().Write(bt::LogLevel::Error, calib_result.status().message);
                return fail_with_dashboard(5, calib_result.status().message, "calibration_load_failed");
            }
        } else if (stereo_monocular_fallback_enabled) {
            bt::Logger::Instance().Write(
                bt::LogLevel::Warn,
                "stereo calibration file is unavailable; continuing with stereo->monocular single-camera fallback");
        }
    }

    const auto readiness = bt::EvaluateCalibrationReadiness(calibration_bundle, cfg.tracking.mode);
    web_state.SetCalibration(readiness);
    bt::Logger::Instance().Write(bt::LogLevel::Info, readiness.summary);

    bt::RtmPoseSession model;
    std::string model_missing_note;
    const auto model_exists_result = inspect_exists(cfg.tracking.model_path, "model path");
    if (!model_exists_result.ok()) {
        bt::Logger::Instance().Write(bt::LogLevel::Error, model_exists_result.status().message);
        return fail_with_dashboard(6, model_exists_result.status().message, "model_path_inspect_failed");
    }
    if (model_exists_result.value()) {
        const auto s = model.Load(cfg.tracking.model_path, cfg.inference.device);
        if (!s.ok()) {
            bt::Logger::Instance().Write(bt::LogLevel::Error, s.message);
            return fail_with_dashboard(6, s.message, "model_load_failed");
        }
        if (const auto contract = bt::ValidateRtmPoseModelContract(model.GetInfo()); !contract.ok()) {
            bt::Logger::Instance().Write(bt::LogLevel::Error, contract.message);
            return fail_with_dashboard(6, contract.message, "model_contract_failed");
        }
        web_state.SetModel(model.GetInfo());
        bt::Logger::Instance().Write(bt::LogLevel::Info, bt::BuildModelSessionSummary(model.GetInfo()));
    } else {
        model_missing_note = "Download RTMW-DW-X-L Cocktail14 384x288 ONNX to " + ResolveDisplayPath(cfg.tracking.model_path);
        bt::Logger::Instance().Write(bt::LogLevel::Error, "RTMPose model missing; runtime cannot infer poses");
        return fail_with_dashboard(6, model_missing_note, "model_missing");
    }

    bt::RtmPoseSession depth_postprocess_model;
    bt::RtmPoseSession* depth_postprocess_model_ptr = nullptr;
    if (cfg.tracking.depth_postprocess_enabled && !cfg.tracking.depth_postprocess_model_path.empty()) {
        const auto depth_model_exists = inspect_exists(cfg.tracking.depth_postprocess_model_path, "depth postprocess model path");
        if (!depth_model_exists.ok()) {
            bt::Logger::Instance().Write(bt::LogLevel::Warn, "depth postprocess model inspect failed; continuing with 2D tracking only: " + depth_model_exists.status().message);
        } else if (!depth_model_exists.value()) {
            bt::Logger::Instance().Write(bt::LogLevel::Warn, "depth postprocess model missing; continuing with 2D tracking only: " + ResolveDisplayPath(cfg.tracking.depth_postprocess_model_path));
        } else if (cfg.tracking.depth_postprocess_model_path == cfg.tracking.model_path) {
            bt::Logger::Instance().Write(bt::LogLevel::Warn, "depth postprocess model path matches primary pose model; continuing without separate 3D postprocess pass");
        } else {
            const bool depth_cpu_allowed =
                cfg.tracking.depth_postprocess_allow_cpu_fallback || cfg.inference.device == "cpu";
            std::string depth_device = cfg.inference.device;
            if (!depth_cpu_allowed && depth_device == "directml") {
                depth_device = "directml_strict";
            }
            const auto depth_load = depth_postprocess_model.Load(cfg.tracking.depth_postprocess_model_path, depth_device);
            if (!depth_load.ok()) {
                bt::Logger::Instance().Write(bt::LogLevel::Warn, "depth postprocess model load failed; continuing with 2D tracking only: " + depth_load.message);
            } else if (const auto contract = bt::ValidateRtmPoseModelContract(depth_postprocess_model.GetInfo()); !contract.ok()) {
                bt::Logger::Instance().Write(bt::LogLevel::Warn, "depth postprocess model contract failed; continuing with 2D tracking only: " + contract.message);
            } else if (depth_postprocess_model.GetInfo().outputs.size() != 3) {
                bt::Logger::Instance().Write(bt::LogLevel::Warn, "depth postprocess model is not RTMW3D; continuing with 2D tracking only");
            } else if (!depth_cpu_allowed && depth_postprocess_model.GetInfo().active_device == "cpu") {
                bt::Logger::Instance().Write(bt::LogLevel::Warn, "depth postprocess model would run on CPU; continuing with 2D tracking only to protect live FPS");
            } else {
                depth_postprocess_model_ptr = &depth_postprocess_model;
                bt::Logger::Instance().Write(bt::LogLevel::Info, "depth postprocess model loaded: " + bt::BuildModelSessionSummary(depth_postprocess_model.GetInfo()));
            }
        }
    }

    bt::OscSender osc(cfg.osc);
    if (const auto s = osc.Open(); !s.ok()) {
        bt::Logger::Instance().Write(bt::LogLevel::Warn, "OSC open failed; tracking will continue without outbound OSC until reconfigured: " + s.message);
    }

    bt::SteamVrTrackerBridgeSender steamvr_tracker_bridge(cfg.steamvr_tracker_bridge);
    if (const auto s = steamvr_tracker_bridge.Open(); !s.ok()) {
        bt::Logger::Instance().Write(bt::LogLevel::Warn, "SteamVR tracker bridge open failed; tracking will continue without SteamVR virtual tracker packets until reconfigured: " + s.message);
    }

    bt::ReplayLogWriter replay;
    if (cfg.tracking.enable_replay_recording) {
        if (const auto s = replay.Open(ReplayPath(cfg), MakeReplaySessionInfo(config_path, cfg)); !s.ok()) {
            bt::Logger::Instance().Write(bt::LogLevel::Warn, "Replay log open failed; runtime will continue without replay recording: " + s.message);
        }
    }

    bt::TrackingPipeline pipeline(calibration_bundle);
    pipeline.SetParams(cfg.tracking);
    auto hmd_provider = bt::MakeHmdProvider(cfg.hmd);
    ViewRuntimeState view_a;
    ViewRuntimeState view_b;
    bt::CameraDevice camera_a(bt::CameraId::A, cfg.camera_a);
    bt::CameraDevice camera_b(bt::CameraId::B, cfg.camera_b);
    bool camera_b_running = false;
    bt::FramePairer pairer(bt::FramePairerConfig{cfg.tracking.latest_frame_skew_tolerance_ms});
    bt::SteamVrAlignmentManager runtime_steamvr_alignment;

    if (const auto s = camera_a.Start(); !s.ok()) {
        bt::Logger::Instance().Write(bt::LogLevel::Error, s.message);
        if (!options.interactive_setup_on_failure) {
            bt::DebugSnapshot startup_debug;
            startup_debug.phase = "startup";
            startup_debug.degradation_mode = "camera_a_failed";
            startup_debug.last_error = s.message;
            CopyOscReportToDebug(startup_debug, cfg.osc, osc.LastReport());
            web_state.SetDebug(startup_debug);
            replay.Close();
            osc.Close();
            if (options.show_native_dashboard) {
                cv::destroyWindow(kDashboardWindowName);
            }
            return 7;
        }
        const auto action = RunSetupDashboard(config_path, cfg, s.message, model_missing_note.empty() ? "Model file ready" : model_missing_note);
        replay.Close();
        osc.Close();
        if (options.show_native_dashboard) {
            cv::destroyWindow(kDashboardWindowName);
        }
        if (action == SetupAction::Retry) {
            return kRuntimeRetryRequested;
        }
        return 7;
    }
    if (!monocular_mode) {
        if (const auto s = camera_b.Start(); !s.ok()) {
            if (stereo_monocular_fallback_enabled) {
                bt::Logger::Instance().Write(
                    bt::LogLevel::Warn,
                    "camera_b failed; continuing with stereo->monocular single-camera fallback: " + s.message);
            } else {
                bt::Logger::Instance().Write(bt::LogLevel::Error, s.message);
                camera_a.Stop();
                if (!options.interactive_setup_on_failure) {
                    bt::DebugSnapshot startup_debug;
                    startup_debug.phase = "startup";
                    startup_debug.degradation_mode = "camera_b_failed";
                    startup_debug.last_error = s.message;
                    CopyOscReportToDebug(startup_debug, cfg.osc, osc.LastReport());
                    web_state.SetDebug(startup_debug);
                    replay.Close();
                    osc.Close();
                    if (options.show_native_dashboard) {
                        cv::destroyWindow(kDashboardWindowName);
                    }
                    return 8;
                }
                const auto action = RunSetupDashboard(config_path, cfg, s.message, model_missing_note.empty() ? "Model file ready" : model_missing_note);
                replay.Close();
                osc.Close();
                if (options.show_native_dashboard) {
                    cv::destroyWindow(kDashboardWindowName);
                }
                if (action == SetupAction::Retry) {
                    return kRuntimeRetryRequested;
                }
                return 8;
            }
        } else {
            camera_b_running = true;
        }
    } else {
        bt::Logger::Instance().Write(bt::LogLevel::Info, "monocular tracking mode: camera_b capture disabled");
    }

    bt::DebugSnapshot debug;
    bt::QpcTimestamp last_step_time = bt::NowQpc();
    std::uint64_t last_processed_camera_sequence_a = 0;
    std::uint64_t last_processed_camera_sequence_b = 0;
    bool last_processed_camera_has_a = false;
    bool last_processed_camera_has_b = false;
    double last_ui_publish_ms = 0.0;
    std::atomic_bool preview_stop{false};
    std::thread preview_thread([&]() {
        std::uint64_t last_preview_sequence_a = 0;
        std::uint64_t last_preview_sequence_b = 0;
        while (!preview_stop.load(std::memory_order_relaxed) && !g_stop_requested) {
            if (const auto frame = camera_a.GetLatestFrame()) {
                if (frame->sequence != last_preview_sequence_a) {
                    const bt::Rect2f full = FullFrameRect(frame->width, frame->height);
                    const bt::Rect2f source = cfg.camera_a.initial_roi_enabled
                        ? InitialRoiPixels(cfg.camera_a, frame->width, frame->height)
                        : full;
                    const cv::Rect source_rect = ClampPreviewRect(frame->bgr, source);
                    const std::string full_preview = MakePreviewDataUrl(frame->bgr, full);
                    const std::string preview = MakePreviewDataUrl(frame->bgr, source);
                    if (!preview.empty()) {
                        web_state.SetCameraPreview(
                            'a',
                            preview,
                            full_preview.empty() ? preview : full_preview,
                            source_rect.width,
                            source_rect.height,
                            source_rect.x,
                            source_rect.y,
                            source_rect.width,
                            source_rect.height,
                            frame->width,
                            frame->height,
                            frame->sequence);
                        last_preview_sequence_a = frame->sequence;
                    }
                }
            }
            if (camera_b_running) {
                if (const auto frame = camera_b.GetLatestFrame()) {
                    if (frame->sequence != last_preview_sequence_b) {
                        const bt::Rect2f full = FullFrameRect(frame->width, frame->height);
                        const bt::Rect2f source = cfg.camera_b.initial_roi_enabled
                            ? InitialRoiPixels(cfg.camera_b, frame->width, frame->height)
                            : full;
                        const cv::Rect source_rect = ClampPreviewRect(frame->bgr, source);
                        const std::string full_preview = MakePreviewDataUrl(frame->bgr, full);
                        const std::string preview = MakePreviewDataUrl(frame->bgr, source);
                        if (!preview.empty()) {
                            web_state.SetCameraPreview(
                                'b',
                                preview,
                                full_preview.empty() ? preview : full_preview,
                                source_rect.width,
                                source_rect.height,
                                source_rect.x,
                                source_rect.y,
                                source_rect.width,
                                source_rect.height,
                                frame->width,
                                frame->height,
                                frame->sequence);
                            last_preview_sequence_b = frame->sequence;
                        }
                    }
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(66));
        }
    });
    bt::Profiler profiler;
    bt::Logger::Instance().Write(bt::LogLevel::Info, "runtime loop started; press Ctrl+C to stop");
    std::string last_hmd_depth_scale_log_state;
    auto last_hmd_depth_scale_log_time = std::chrono::steady_clock::now() - std::chrono::seconds(1);
    std::string last_stereo_hmd_anchor_log_state;
    auto last_stereo_hmd_anchor_log_time = std::chrono::steady_clock::now() - std::chrono::seconds(1);
    std::string last_anchor_space_mapping_log_state;
    auto last_anchor_space_mapping_log_time = std::chrono::steady_clock::now() - std::chrono::seconds(1);
    std::string last_room_depth_map_log_state;
    auto last_room_depth_map_log_time = std::chrono::steady_clock::now() - std::chrono::seconds(1);

    while (!g_stop_requested) {
        const auto loop_now = bt::NowQpc();
        const auto total_start = loop_now;
        const auto pair_start = bt::NowQpc();
        std::shared_ptr<const bt::FramePacket> frame_a;
        std::shared_ptr<const bt::FramePacket> frame_b;
        bt::PairedFrames pair;
        bool single_camera_this_frame = monocular_mode;
        std::string single_camera_reason = monocular_mode ? "monocular_mode" : "";
        if (monocular_mode || (!camera_b_running && stereo_monocular_fallback_enabled)) {
            frame_a = camera_a.GetLatestFrame();
            single_camera_this_frame = true;
            if (!monocular_mode) {
                single_camera_reason = "camera_b_unavailable";
            }
        } else {
            pair = pairer.PairRecent(camera_a.GetRecentFrames(), camera_b.GetRecentFrames());
            frame_a = pair.camera_a;
            frame_b = pair.camera_b;
            if (!pair.valid && stereo_monocular_fallback_enabled && frame_a) {
                single_camera_this_frame = true;
                single_camera_reason = "frame_pair_" + pair.reason;
                frame_b.reset();
            } else if (!readiness.tracking_ready && stereo_monocular_fallback_enabled && frame_a) {
                single_camera_this_frame = true;
                single_camera_reason = "stereo_calibration_not_ready";
                frame_b.reset();
            }
        }
        const bool stereo_fallback_this_frame = single_camera_this_frame && !monocular_mode;
        const bool calibration_ready_for_frame = readiness.tracking_ready || single_camera_this_frame;
        const double frame_pair_ms = bt::QpcDeltaSeconds(pair_start, bt::NowQpc()) * 1000.0;
        debug = bt::DebugSnapshot{};
        debug.phase = "runtime";
        debug.timestamp_seconds = TimestampSeconds(loop_now);
        debug.camera_a = camera_a.GetHealthSnapshot();
        debug.camera_b = single_camera_this_frame ? bt::CaptureHealthSnapshot{} : camera_b.GetHealthSnapshot();
        debug.frame_pairing = single_camera_this_frame ? bt::FramePairerTelemetry{} : pairer.Telemetry();
        debug.frame_pair_ms = frame_pair_ms;
        debug.frame_a_sequence = single_camera_this_frame
            ? (frame_a ? frame_a->sequence : 0)
            : debug.frame_pairing.last_accepted_sequence_a;
        debug.frame_b_sequence = single_camera_this_frame ? 0 : debug.frame_pairing.last_accepted_sequence_b;
        debug.frame_skew_ms = single_camera_this_frame ? 0.0 : debug.frame_pairing.last_skew_ms;
        if (!single_camera_this_frame && pair.valid) {
            debug.frame_pair_degraded = pair.degraded;
            debug.frame_pair_reused_a = pair.reused_a;
            debug.frame_pair_reused_b = pair.reused_b;
            debug.frame_pair_duplicate = pair.duplicate;
            debug.frame_pair_skewed = pair.skewed;
            debug.frame_pair_reason = pair.reason;
        } else if (!single_camera_this_frame && !pair.reason.empty()) {
            debug.frame_pair_reason = pair.reason;
        }
        debug.model_active_device = model.GetInfo().active_device;
        debug.model_ep_fallback = model.GetInfo().ep_fallback;
        if (frame_a) {
            debug.camera_a_frame_age_ms = FrameAgeMs(*frame_a, loop_now);
        }
        if (frame_b) {
            debug.camera_b_frame_age_ms = FrameAgeMs(*frame_b, loop_now);
        }
        debug.tracking = pipeline.Snapshot();
        bt::OscConfig osc_cfg_for_frame = RuntimeOscConfigForFrame(web_state, cfg, calibration_bundle);
        const bool runtime_should_poll_steamvr = ShouldPollSteamVrProvider(osc_cfg_for_frame) || cfg.tracking.anchor_space_mapping.enabled;
        bt::SteamVrPoseSnapshot runtime_steamvr_snapshot;
        if (runtime_should_poll_steamvr) {
            runtime_steamvr_snapshot = runtime_steamvr_alignment.Poll();
        }
        const bt::SteamVrAnchorFrame runtime_steamvr_anchors = bt::SteamVrAnchorFrameFromSnapshot(runtime_steamvr_snapshot);
        bt::SteamVrAlignmentStatus steamvr_status = runtime_steamvr_alignment.Status(
            osc_cfg_for_frame,
            calibration_bundle,
            debug.tracking.body_state.valid);
        osc_cfg_for_frame = ApplySteamVrRuntimeFreshnessToOscConfig(osc_cfg_for_frame, steamvr_status);
        steamvr_status = runtime_steamvr_alignment.Status(
            osc_cfg_for_frame,
            calibration_bundle,
            debug.tracking.body_state.valid);
        // Runtime SteamVR debug is authoritative only after the runtime actually
        // polled OpenVR for an active/stale SteamVR tracker-space source. When
        // the menu is using the SteamVR calibration wizard while tracker space is
        // still manual/invalid, publishing this unpolled default status would
        // overwrite the wizard with "provider not initialized" even though the
        // menu alignment manager can see the live SteamVR session.
        debug.steamvr_alignment_recorded = runtime_should_poll_steamvr;
        if (runtime_should_poll_steamvr) {
            debug.steamvr_alignment = steamvr_status;
        }
        const auto osc_update_status = osc.UpdateConfig(osc_cfg_for_frame);
        CopyOscReportToDebug(debug, osc_cfg_for_frame, osc.LastReport());
        if (!osc_update_status.ok() && debug.last_error.empty()) {
            debug.last_error = osc_update_status.message;
        }
        const bt::SteamVrTrackerBridgeConfig steamvr_bridge_cfg_for_frame =
            RuntimeSteamVrTrackerBridgeConfigForFrame(web_state, cfg);
        const auto steamvr_bridge_update_status = steamvr_tracker_bridge.UpdateConfig(steamvr_bridge_cfg_for_frame);
        CopySteamVrBridgeReportToDebug(debug, steamvr_tracker_bridge.LastReport());
        if (!steamvr_bridge_update_status.ok() && debug.last_error.empty()) {
            debug.last_error = steamvr_bridge_update_status.message;
        }
        debug.capture_ms = single_camera_this_frame
            ? debug.camera_a.last_read_ms
            : std::max(debug.camera_a.last_read_ms, debug.camera_b.last_read_ms);
        debug.ui_publish_ms = last_ui_publish_ms;

        std::string prediction_only_reason;
        bool duplicate_camera_measurement_this_tick = false;
        if (!calibration_ready_for_frame) {
            prediction_only_reason = "calibration_not_ready";
        } else if (!frame_a) {
            prediction_only_reason = "waiting_for_camera_a_frame";
        } else if (!single_camera_this_frame && !pair.valid) {
            prediction_only_reason = "waiting_for_paired_frames:" + pair.reason;
        }

        if (prediction_only_reason.empty() && model.GetInfo().loaded) {
            const bool selected_has_a = frame_a != nullptr;
            const bool selected_has_b = !single_camera_this_frame && frame_b != nullptr;
            const bool duplicate_selected_camera_measurement =
                selected_has_a == last_processed_camera_has_a &&
                selected_has_b == last_processed_camera_has_b &&
                (!selected_has_a || frame_a->sequence == last_processed_camera_sequence_a) &&
                (!selected_has_b || frame_b->sequence == last_processed_camera_sequence_b);
            if (duplicate_selected_camera_measurement) {
                duplicate_camera_measurement_this_tick = true;
                prediction_only_reason = "duplicate_camera_measurement";
            }
        }

        if (!model.GetInfo().loaded) {
            debug.degradation_mode = "model_not_loaded";
            debug.last_error = model_missing_note;
        } else if (!prediction_only_reason.empty()) {
            const auto pipeline_start = bt::NowQpc();
            const double dt_seconds = std::clamp(bt::QpcDeltaSeconds(last_step_time, pipeline_start), 1.0 / 240.0, 0.10);
            last_step_time = pipeline_start;
            const auto hmd = hmd_provider->Poll(static_cast<double>(pipeline_start.ticks) / 1000000000.0);
            bt::BodySolveInputs inputs;
            inputs.hmd = hmd.ok() ? hmd.value() : bt::HmdPoseSample{};
            inputs.steamvr_anchors = runtime_steamvr_anchors;
            inputs.quality = solve_quality;
            if (duplicate_camera_measurement_this_tick) {
                if (frame_a) {
                    inputs.camera_a_frame_sequence = frame_a->sequence;
                    inputs.camera_a_timestamp_seconds = TimestampSeconds(frame_a->timestamp);
                    inputs.camera_a_frame_age_ms = debug.camera_a_frame_age_ms;
                    inputs.camera_a_image_width = frame_a->width;
                    inputs.camera_a_image_height = frame_a->height;
                }
                if (!single_camera_this_frame && frame_b) {
                    inputs.camera_b_frame_sequence = frame_b->sequence;
                    inputs.camera_b_timestamp_seconds = TimestampSeconds(frame_b->timestamp);
                    inputs.camera_b_frame_age_ms = debug.camera_b_frame_age_ms;
                    inputs.camera_b_image_width = frame_b->width;
                    inputs.camera_b_image_height = frame_b->height;
                }
                inputs.stale_timeout_ms = cfg.tracking.stale_frame_timeout_ms;
            }
            debug.hmd = inputs.hmd;
            debug.hmd_valid = inputs.hmd.valid;
            const auto step = pipeline.Step(inputs, dt_seconds);
            debug.pipeline_ms = bt::QpcDeltaSeconds(pipeline_start, bt::NowQpc()) * 1000.0;
            debug.tracking = step.ok() ? step.value() : pipeline.Snapshot();
            debug.solver = debug.tracking.solver;
            debug.preliminary_solve_ms = debug.solver.preliminary_solve_ms;
            debug.final_solve_ms = debug.solver.final_solve_ms;
            debug.solver_ms = debug.preliminary_solve_ms + debug.final_solve_ms;
            if (debug.solver_ms <= 0.0) {
                debug.solver_ms = debug.pipeline_ms;
            }
            debug.objective_evaluations = debug.solver.objective_evaluations;
            debug.coordinate_passes = debug.solver.coordinate_passes;
            debug.optimizer_early_stopped = debug.solver.optimizer_early_stopped;
            debug.degradation_mode = "prediction_only:" + prediction_only_reason;
            debug.last_error = step.ok() && debug.tracking.last_error.empty()
                ? prediction_only_reason
                : (step.ok() ? debug.tracking.last_error : step.status().message);
            const auto osc_status = SendTrackersAndRecordOsc(osc, osc_cfg_for_frame, debug.tracking.trackers, debug);
            if (!osc_status.ok() && debug.last_error.empty()) {
                debug.last_error = osc_status.message;
            }
            (void)SendTrackersAndRecordSteamVrBridge(steamvr_tracker_bridge, osc_cfg_for_frame, debug.tracking.trackers, debug);
        } else {
            last_processed_camera_has_a = frame_a != nullptr;
            last_processed_camera_has_b = !single_camera_this_frame && frame_b != nullptr;
            last_processed_camera_sequence_a = last_processed_camera_has_a ? frame_a->sequence : 0;
            last_processed_camera_sequence_b = last_processed_camera_has_b ? frame_b->sequence : 0;
            const bt::PostureMode posture_hint = pipeline.Snapshot().state.posture_mode;
            auto a = ProcessCameraView(*frame_a, view_a, cfg.camera_a, model, depth_postprocess_model_ptr, cfg.tracking.depth_postprocess_interval_frames, posture_hint);
            if (!a.ok()) {
                view_a.roi.Update(frame_a->width, frame_a->height, nullptr, posture_hint);
                debug.degradation_mode = "camera_a_pose_failed";
                debug.last_error = a.status().message;
            } else {
                ApplyFrameAgeConfidence(a.value(), debug.camera_a_frame_age_ms, cfg.tracking.stale_frame_timeout_ms);
            }
            if (!a.ok()) {
                // Camera A failed to decode into pose this frame; finite-pixel age degradation only applies after a pose exists.
            } else if (single_camera_this_frame) {
                const auto pipeline_start = bt::NowQpc();
                const double dt_seconds = std::clamp(bt::QpcDeltaSeconds(last_step_time, pipeline_start), 1.0 / 240.0, 0.10);
                last_step_time = pipeline_start;
                const auto hmd = hmd_provider->Poll(static_cast<double>(pipeline_start.ticks) / 1000000000.0);
                bt::BodySolveInputs inputs;
                inputs.camera_a_pose    = a.value().pose;
                inputs.camera_a_pose_3d = a.value().pose3d;
                inputs.camera_a_reliability = a.value().reliability;
                inputs.camera_a_frame_age_ms = debug.camera_a_frame_age_ms;
                inputs.camera_a_frame_sequence = frame_a->sequence;
                inputs.camera_a_timestamp_seconds = TimestampSeconds(frame_a->timestamp);
                inputs.camera_a_image_width = frame_a->width;
                inputs.camera_a_image_height = frame_a->height;
                inputs.stale_timeout_ms = cfg.tracking.stale_frame_timeout_ms;
                inputs.hmd = hmd.ok() ? hmd.value() : bt::HmdPoseSample{};
                inputs.steamvr_anchors = runtime_steamvr_anchors;
                inputs.quality = solve_quality;
                debug.hmd = inputs.hmd;
                debug.hmd_valid = inputs.hmd.valid;
                if (!hmd.ok() && debug.last_error.empty()) {
                    debug.last_error = hmd.status().message;
                }

                if (stereo_fallback_this_frame) {
                    bt::TrackingConfig fallback_tracking = cfg.tracking;
                    fallback_tracking.mode = bt::TrackingMode::Monocular;
                    pipeline.SetParams(fallback_tracking);
                }
                const auto step = pipeline.Step(inputs, dt_seconds);
                if (stereo_fallback_this_frame) {
                    pipeline.SetParams(cfg.tracking);
                }
                debug.pipeline_ms = bt::QpcDeltaSeconds(pipeline_start, bt::NowQpc()) * 1000.0;
                debug.tracking = step.ok() ? step.value() : pipeline.Snapshot();
                debug.solver = debug.tracking.solver;
                debug.preliminary_solve_ms = debug.solver.preliminary_solve_ms;
                debug.final_solve_ms = debug.solver.final_solve_ms;
                debug.solver_ms = debug.preliminary_solve_ms + debug.final_solve_ms;
                if (debug.solver_ms <= 0.0) {
                    debug.solver_ms = debug.pipeline_ms;
                }
                debug.objective_evaluations = debug.solver.objective_evaluations;
                debug.coordinate_passes = debug.solver.coordinate_passes;
                debug.optimizer_early_stopped = debug.solver.optimizer_early_stopped;
                debug.degradation_mode = JoinDegradation(
                    debug.tracking.degradation_mode,
                    AgeDegradationSuffix(a.value(), "camera_a"));
                debug.last_error = step.ok() ? debug.tracking.last_error : step.status().message;
                if (stereo_fallback_this_frame) {
                    debug.degradation_mode = JoinDegradation(
                        "stereo_monocular_fallback:" + single_camera_reason,
                        AgeDegradationSuffix(a.value(), "camera_a"));
                    if (debug.last_error.empty()) {
                        debug.last_error = "Stereo mode is using Camera A monocular fallback because Camera B is unavailable or unpaired";
                    }
                    debug.tracking.body_state.diagnostics.stereo_fallback_active = true;
                    debug.tracking.body_state.diagnostics.monocular_fallback = true;
                    bt::MarkTrackersStereoFallback(debug.tracking.trackers);
                }

                const auto osc_status = SendTrackersAndRecordOsc(osc, osc_cfg_for_frame, debug.tracking.trackers, debug);
                if (!osc_status.ok() && debug.last_error.empty()) {
                    debug.last_error = osc_status.message;
                }
                (void)SendTrackersAndRecordSteamVrBridge(steamvr_tracker_bridge, osc_cfg_for_frame, debug.tracking.trackers, debug);

                debug.camera_a_pose = a.value().pose;
                debug.camera_a_reliability_full = a.value().reliability;
                debug.camera_a_pose_confidence = a.value().pose.aggregate_confidence;
                debug.camera_a_reliability = a.value().reliability.lower_body_mean;
                debug.camera_a_roi = a.value().roi_next;
                debug.inference_ms_a = a.value().inference_ms;
                debug.inference_ms = a.value().inference_ms;
                debug.preprocess_ms_a = a.value().preprocess_ms;
                debug.preprocess_ms = a.value().preprocess_ms;
                debug.onnx_ms_a = a.value().onnx_ms;
                debug.onnx_ms = a.value().onnx_ms;
                debug.decode_ms_a = a.value().decode_ms;
                debug.decode_ms = a.value().decode_ms;
            } else {
                auto b = ProcessCameraView(*frame_b, view_b, cfg.camera_b, model, depth_postprocess_model_ptr, cfg.tracking.depth_postprocess_interval_frames, posture_hint);
                if (b.ok()) {
                    ApplyFrameAgeConfidence(b.value(), debug.camera_b_frame_age_ms, cfg.tracking.stale_frame_timeout_ms);
                    if (pair.degraded) {
                        ScaleReliability(a.value().reliability, PairReliabilityScale(pair, true));
                        ScaleReliability(b.value().reliability, PairReliabilityScale(pair, false));
                    }
                }
                if (!b.ok()) {
                    view_b.roi.Update(frame_b->width, frame_b->height, nullptr, posture_hint);
                    if (stereo_monocular_fallback_enabled) {
                        const auto pipeline_start = bt::NowQpc();
                        const double dt_seconds = std::clamp(bt::QpcDeltaSeconds(last_step_time, pipeline_start), 1.0 / 240.0, 0.10);
                        last_step_time = pipeline_start;
                        const auto hmd = hmd_provider->Poll(static_cast<double>(pipeline_start.ticks) / 1000000000.0);
                        bt::BodySolveInputs inputs;
                        inputs.camera_a_pose    = a.value().pose;
                        inputs.camera_a_pose_3d = a.value().pose3d;
                        inputs.camera_a_reliability = a.value().reliability;
                        inputs.camera_a_frame_age_ms = debug.camera_a_frame_age_ms;
                        inputs.camera_a_frame_sequence = frame_a->sequence;
                        inputs.camera_a_timestamp_seconds = TimestampSeconds(frame_a->timestamp);
                        inputs.camera_a_image_width = frame_a->width;
                        inputs.camera_a_image_height = frame_a->height;
                        inputs.stale_timeout_ms = cfg.tracking.stale_frame_timeout_ms;
                        inputs.hmd = hmd.ok() ? hmd.value() : bt::HmdPoseSample{};
                        inputs.steamvr_anchors = runtime_steamvr_anchors;
                        inputs.quality = solve_quality;
                        debug.hmd = inputs.hmd;
                        debug.hmd_valid = inputs.hmd.valid;
                        if (!hmd.ok() && debug.last_error.empty()) {
                            debug.last_error = hmd.status().message;
                        }

                        bt::TrackingConfig fallback_tracking = cfg.tracking;
                        fallback_tracking.mode = bt::TrackingMode::Monocular;
                        pipeline.SetParams(fallback_tracking);
                        const auto step = pipeline.Step(inputs, dt_seconds);
                        pipeline.SetParams(cfg.tracking);

                        debug.pipeline_ms = bt::QpcDeltaSeconds(pipeline_start, bt::NowQpc()) * 1000.0;
                        debug.tracking = step.ok() ? step.value() : pipeline.Snapshot();
                        debug.solver = debug.tracking.solver;
                        debug.preliminary_solve_ms = debug.solver.preliminary_solve_ms;
                        debug.final_solve_ms = debug.solver.final_solve_ms;
                        debug.solver_ms = debug.preliminary_solve_ms + debug.final_solve_ms;
                        if (debug.solver_ms <= 0.0) {
                            debug.solver_ms = debug.pipeline_ms;
                        }
                        debug.objective_evaluations = debug.solver.objective_evaluations;
                        debug.coordinate_passes = debug.solver.coordinate_passes;
                        debug.optimizer_early_stopped = debug.solver.optimizer_early_stopped;
                        debug.degradation_mode = JoinDegradation(
                            "stereo_monocular_fallback:camera_b_pose_failed",
                            AgeDegradationSuffix(a.value(), "camera_a"));
                        debug.last_error = step.ok() && debug.tracking.last_error.empty()
                            ? "Stereo mode is using Camera A monocular fallback because Camera B pose failed: " + b.status().message
                            : (step.ok() ? debug.tracking.last_error : step.status().message);
                        debug.tracking.body_state.diagnostics.stereo_fallback_active = true;
                        debug.tracking.body_state.diagnostics.monocular_fallback = true;
                        bt::MarkTrackersStereoFallback(debug.tracking.trackers);

                        const auto osc_status = SendTrackersAndRecordOsc(osc, osc_cfg_for_frame, debug.tracking.trackers, debug);
                        if (!osc_status.ok() && debug.last_error.empty()) {
                            debug.last_error = osc_status.message;
                        }
                        (void)SendTrackersAndRecordSteamVrBridge(steamvr_tracker_bridge, osc_cfg_for_frame, debug.tracking.trackers, debug);
                    } else {
                        debug.degradation_mode = "camera_b_pose_failed";
                        debug.last_error = b.status().message;
                    }
                    debug.camera_a_pose = a.value().pose;
                    debug.camera_a_reliability_full = a.value().reliability;
                    debug.camera_a_pose_confidence = a.value().pose.aggregate_confidence;
                    debug.camera_a_reliability = a.value().reliability.lower_body_mean;
                    debug.camera_a_roi = a.value().roi_next;
                    debug.inference_ms = a.value().inference_ms;
                    debug.inference_ms_a = a.value().inference_ms;
                    debug.preprocess_ms = a.value().preprocess_ms;
                    debug.preprocess_ms_a = a.value().preprocess_ms;
                    debug.onnx_ms = a.value().onnx_ms;
                    debug.onnx_ms_a = a.value().onnx_ms;
                    debug.decode_ms = a.value().decode_ms;
                    debug.decode_ms_a = a.value().decode_ms;
                } else {
                    const auto pipeline_start = bt::NowQpc();
                    const double dt_seconds = std::clamp(bt::QpcDeltaSeconds(last_step_time, pipeline_start), 1.0 / 240.0, 0.10);
                    last_step_time = pipeline_start;
                    const auto hmd = hmd_provider->Poll(static_cast<double>(pipeline_start.ticks) / 1000000000.0);
                    bt::BodySolveInputs inputs;
                    inputs.camera_a_pose    = a.value().pose;
                    inputs.camera_a_pose_3d = a.value().pose3d;
                    inputs.camera_b_pose    = b.value().pose;
                    inputs.camera_b_pose_3d = b.value().pose3d;
                    inputs.camera_a_reliability = a.value().reliability;
                    inputs.camera_b_reliability = b.value().reliability;
                    inputs.camera_a_frame_age_ms = debug.camera_a_frame_age_ms;
                    inputs.camera_b_frame_age_ms = debug.camera_b_frame_age_ms;
                    inputs.camera_a_frame_sequence = frame_a->sequence;
                    inputs.camera_b_frame_sequence = frame_b->sequence;
                    inputs.camera_a_timestamp_seconds = TimestampSeconds(frame_a->timestamp);
                    inputs.camera_b_timestamp_seconds = TimestampSeconds(frame_b->timestamp);
                    inputs.stereo_pair_degraded = pair.degraded;
                    inputs.stereo_pair_reused_a = pair.reused_a;
                    inputs.stereo_pair_reused_b = pair.reused_b;
                    inputs.stereo_pair_duplicate = pair.duplicate;
                    inputs.stereo_pair_skewed = pair.skewed;
                    inputs.camera_a_image_width = frame_a->width;
                    inputs.camera_a_image_height = frame_a->height;
                    inputs.camera_b_image_width = frame_b->width;
                    inputs.camera_b_image_height = frame_b->height;
                    inputs.stale_timeout_ms = cfg.tracking.stale_frame_timeout_ms;
                    inputs.hmd = hmd.ok() ? hmd.value() : bt::HmdPoseSample{};
                    inputs.steamvr_anchors = runtime_steamvr_anchors;
                    inputs.quality = solve_quality;
                    debug.hmd = inputs.hmd;
                    debug.hmd_valid = inputs.hmd.valid;
                    if (!hmd.ok() && debug.last_error.empty()) {
                        debug.last_error = hmd.status().message;
                    }

                    const auto step = pipeline.Step(inputs, dt_seconds);
                    debug.pipeline_ms = bt::QpcDeltaSeconds(pipeline_start, bt::NowQpc()) * 1000.0;
                    debug.tracking = step.ok() ? step.value() : pipeline.Snapshot();
                    debug.solver = debug.tracking.solver;
                    debug.preliminary_solve_ms = debug.solver.preliminary_solve_ms;
                    debug.final_solve_ms = debug.solver.final_solve_ms;
                    debug.solver_ms = debug.preliminary_solve_ms + debug.final_solve_ms;
                    if (debug.solver_ms <= 0.0) {
                        debug.solver_ms = debug.pipeline_ms;
                    }
                    debug.objective_evaluations = debug.solver.objective_evaluations;
                    debug.coordinate_passes = debug.solver.coordinate_passes;
                    debug.optimizer_early_stopped = debug.solver.optimizer_early_stopped;
                    debug.degradation_mode = JoinDegradation(
                        JoinDegradation(
                            JoinDegradation(debug.tracking.degradation_mode, FramePairDegradationSuffix(pair)),
                            AgeDegradationSuffix(a.value(), "camera_a")),
                        AgeDegradationSuffix(b.value(), "camera_b"));
                    debug.last_error = step.ok() ? debug.tracking.last_error : step.status().message;

                    const auto osc_status = SendTrackersAndRecordOsc(osc, osc_cfg_for_frame, debug.tracking.trackers, debug);
                    if (!osc_status.ok() && debug.last_error.empty()) {
                        debug.last_error = osc_status.message;
                    }
                    (void)SendTrackersAndRecordSteamVrBridge(steamvr_tracker_bridge, osc_cfg_for_frame, debug.tracking.trackers, debug);

                    debug.camera_a_pose = a.value().pose;
                    debug.camera_b_pose = b.value().pose;
                    debug.camera_a_reliability_full = a.value().reliability;
                    debug.camera_b_reliability_full = b.value().reliability;
                    debug.camera_a_pose_confidence = a.value().pose.aggregate_confidence;
                    debug.camera_b_pose_confidence = b.value().pose.aggregate_confidence;
                    debug.camera_a_reliability = a.value().reliability.lower_body_mean;
                    debug.camera_b_reliability = b.value().reliability.lower_body_mean;
                    debug.camera_a_roi = a.value().roi_next;
                    debug.camera_b_roi = b.value().roi_next;
                    debug.inference_ms_a = a.value().inference_ms;
                    debug.inference_ms_b = b.value().inference_ms;
                    debug.inference_ms = a.value().inference_ms + b.value().inference_ms;
                    debug.preprocess_ms_a = a.value().preprocess_ms;
                    debug.preprocess_ms_b = b.value().preprocess_ms;
                    debug.preprocess_ms = a.value().preprocess_ms + b.value().preprocess_ms;
                    debug.onnx_ms_a = a.value().onnx_ms;
                    debug.onnx_ms_b = b.value().onnx_ms;
                    debug.onnx_ms = a.value().onnx_ms + b.value().onnx_ms;
                    debug.decode_ms_a = a.value().decode_ms;
                    debug.decode_ms_b = b.value().decode_ms;
                    debug.decode_ms = a.value().decode_ms + b.value().decode_ms;
                }
            }
        }

        debug.total_ms = bt::QpcDeltaSeconds(total_start, bt::NowQpc()) * 1000.0;
        profiler.Observe(bt::ProfilerFrameStats{
            debug.total_ms,
            debug.capture_ms,
            debug.frame_pair_ms,
            debug.preprocess_ms,
            debug.inference_ms,
            debug.onnx_ms,
            debug.decode_ms,
            debug.pipeline_ms,
            debug.solver_ms,
            debug.osc_ms,
            debug.ui_publish_ms});
        debug.profiler = profiler.Snapshot();
        const auto& hmd_scale_log = debug.solver.hmd_depth_scale;
        const std::string hmd_scale_state = bt::ToString(hmd_scale_log.state);
        const auto hmd_depth_scale_log_now = std::chrono::steady_clock::now();
        const bool hmd_depth_scale_state_changed = hmd_scale_state != last_hmd_depth_scale_log_state;
        const bool hmd_depth_scale_interval_elapsed =
            hmd_depth_scale_log_now - last_hmd_depth_scale_log_time >= std::chrono::seconds(1);
        if (hmd_depth_scale_state_changed || hmd_depth_scale_interval_elapsed) {
            std::ostringstream hmd_scale_msg;
            hmd_scale_msg << "hmd_depth_scale state=" << hmd_scale_state
                          << " reason=" << hmd_scale_log.reason
                          << " scale=" << hmd_scale_log.scale
                          << " live=" << (hmd_scale_log.live ? "true" : "false")
                          << " held=" << (hmd_scale_log.held ? "true" : "false")
                          << " usable=" << (hmd_scale_log.usable ? "true" : "false")
                          << " hmd_world=(" << hmd_scale_log.observation.hmd_world.x << ","
                          << hmd_scale_log.observation.hmd_world.y << ","
                          << hmd_scale_log.observation.hmd_world.z << ")"
                          << " corrected_root=(" << hmd_scale_log.corrected_root_world.x << ","
                          << hmd_scale_log.corrected_root_world.y << ","
                          << hmd_scale_log.corrected_root_world.z << ")"
                          << " dt_ms=" << hmd_scale_log.observation.camera_hmd_timestamp_delta_ms;
            bt::Logger::Instance().Write(bt::LogLevel::Info, hmd_scale_msg.str());
            last_hmd_depth_scale_log_state = hmd_scale_state;
            last_hmd_depth_scale_log_time = hmd_depth_scale_log_now;
        }
        const auto& stereo_anchor_log = debug.solver.stereo_hmd_anchor;
        const std::string stereo_anchor_state = bt::ToString(stereo_anchor_log.state);
        const auto stereo_anchor_log_now = std::chrono::steady_clock::now();
        const bool stereo_anchor_state_changed = stereo_anchor_state != last_stereo_hmd_anchor_log_state;
        const bool stereo_anchor_interval_elapsed =
            stereo_anchor_log_now - last_stereo_hmd_anchor_log_time >= std::chrono::seconds(1);
        if (stereo_anchor_state_changed || stereo_anchor_interval_elapsed) {
            std::ostringstream stereo_anchor_msg;
            stereo_anchor_msg << "stereo_hmd_anchor state=" << stereo_anchor_state
                              << " reason=" << stereo_anchor_log.reason
                              << " applied=" << (stereo_anchor_log.applied ? "true" : "false")
                              << " due=" << (stereo_anchor_log.due ? "true" : "false")
                              << " interval_s=" << stereo_anchor_log.interval_seconds
                              << " elapsed_s=" << stereo_anchor_log.seconds_since_last_anchor
                              << " correction_m=" << stereo_anchor_log.correction_m
                              << " hmd_world=(" << stereo_anchor_log.hmd_world.x << ","
                              << stereo_anchor_log.hmd_world.y << ","
                              << stereo_anchor_log.hmd_world.z << ")"
                              << " stereo_head=(" << stereo_anchor_log.stereo_head_world.x << ","
                              << stereo_anchor_log.stereo_head_world.y << ","
                              << stereo_anchor_log.stereo_head_world.z << ")"
                              << " corrected_root=(" << stereo_anchor_log.corrected_root_world.x << ","
                              << stereo_anchor_log.corrected_root_world.y << ","
                              << stereo_anchor_log.corrected_root_world.z << ")";
            bt::Logger::Instance().Write(bt::LogLevel::Info, stereo_anchor_msg.str());
            last_stereo_hmd_anchor_log_state = stereo_anchor_state;
            last_stereo_hmd_anchor_log_time = stereo_anchor_log_now;
        }
        const auto& anchor_mapping_log = debug.solver.anchor_space_mapping;
        const std::string anchor_mapping_state = anchor_mapping_log.valid ? anchor_mapping_log.mode : std::string("fallback:") + anchor_mapping_log.fallback_reason;
        const auto anchor_mapping_log_now = std::chrono::steady_clock::now();
        const bool anchor_mapping_state_changed = anchor_mapping_state != last_anchor_space_mapping_log_state;
        const bool anchor_mapping_interval_elapsed =
            anchor_mapping_log_now - last_anchor_space_mapping_log_time >= std::chrono::seconds(1);
        if (anchor_mapping_state_changed || anchor_mapping_interval_elapsed) {
            std::ostringstream anchor_msg;
            anchor_msg << "anchor_space_mapping mode=" << anchor_mapping_log.mode
                       << " valid=" << (anchor_mapping_log.valid ? "true" : "false")
                       << " anchors=" << anchor_mapping_log.anchors_used
                       << " reproj_error_px=" << anchor_mapping_log.reprojection_error_px
                       << " max_reproj_error_px=" << anchor_mapping_log.max_reprojection_error_px
                       << " scale=" << anchor_mapping_log.depth_scale
                       << " room_update_ok=" << (anchor_mapping_log.usable_for_room_map_update ? "true" : "false")
                       << " fallback=" << anchor_mapping_log.fallback_reason;
            bt::Logger::Instance().Write(bt::LogLevel::Info, anchor_msg.str());
            last_anchor_space_mapping_log_state = anchor_mapping_state;
            last_anchor_space_mapping_log_time = anchor_mapping_log_now;
        }
        const auto& room_map_log = debug.solver.room_depth_map;
        const auto room_map_log_now = std::chrono::steady_clock::now();
        const bool room_map_state_changed = room_map_log.state != last_room_depth_map_log_state;
        const bool room_map_interval_elapsed =
            room_map_log_now - last_room_depth_map_log_time >= std::chrono::seconds(1);
        if (room_map_state_changed || room_map_interval_elapsed) {
            std::ostringstream room_msg;
            room_msg << "room_depth_map state=" << room_map_log.state
                     << " coverage=" << room_map_log.coverage
                     << " accepted=" << room_map_log.accepted_frames
                     << " rejected=" << room_map_log.rejected_frames
                     << " mean_variance_m2=" << room_map_log.mean_variance_m2
                     << " last_rejection=" << room_map_log.last_rejection_reason;
            bt::Logger::Instance().Write(bt::LogLevel::Info, room_msg.str());
            last_room_depth_map_log_state = room_map_log.state;
            last_room_depth_map_log_time = room_map_log_now;
        }
        const auto publish_start = bt::NowQpc();
        web_state.SetDebug(debug);
        last_ui_publish_ms = bt::QpcDeltaSeconds(publish_start, bt::NowQpc()) * 1000.0;
        if (options.show_native_dashboard) {
            const int dashboard_key = ShowNativeDashboard(debug, "ESC/Q STOPS RUNTIME", 1);
            if (dashboard_key == 27 || dashboard_key == 'q' || dashboard_key == 'Q') {
                g_stop_requested = true;
            }
        }
        if (cfg.tracking.enable_replay_recording) {
            if (const auto s = replay.WriteSnapshot(debug); !s.ok()) {
                bt::Logger::Instance().Write(bt::LogLevel::Warn, s.message);
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    const auto body_persist = pipeline.PersistBodyCalibrationOnShutdown();
    debug.phase = "shutdown";
    debug.tracking = pipeline.Snapshot();
    debug.solver = debug.tracking.solver;
    if (body_persist.complete || body_persist.enabled) {
        if (!body_persist.persist_error.empty()) {
            bt::Logger::Instance().Write(
                bt::LogLevel::Warn,
                "body calibration persistence status: " + body_persist.persist_status + ": " + body_persist.persist_error);
        } else {
            bt::Logger::Instance().Write(
                bt::LogLevel::Info,
                "body calibration persistence status: " + body_persist.persist_status);
        }
    }
    web_state.SetDebug(debug);
    if (cfg.tracking.enable_replay_recording) {
        if (const auto s = replay.WriteSnapshot(debug); !s.ok()) {
            bt::Logger::Instance().Write(bt::LogLevel::Warn, s.message);
        }
    }

    replay.Close();
    preview_stop.store(true, std::memory_order_relaxed);
    if (preview_thread.joinable()) {
        preview_thread.join();
    }
    camera_a.Stop();
    if (camera_b_running) {
        camera_b.Stop();
    }
    osc.Close();
    if (options.show_native_dashboard) {
        cv::destroyWindow(kDashboardWindowName);
    }
    bt::Logger::Instance().Write(bt::LogLevel::Info, "runtime loop stopped");
    return 0;
}

int RunRuntime(const std::filesystem::path& config_path, const RuntimeRunOptions& options = {}) {
    for (;;) {
        const int code = RunRuntimeOnce(config_path, options);
        if (code != kRuntimeRetryRequested) {
            return code;
        }
        g_stop_requested = false;
    }
}

nlohmann::json NormalizeCommandReply(std::string command, nlohmann::json reply) {
    if (!reply.is_object()) {
        reply = {{"status", "command returned non-object response"}, {"ok", false}};
    }
    const bool ok = reply.value("ok", true);
    const std::string status = reply.value("status", std::string{});
    const std::string warning = reply.value("warning", std::string{});
    std::string status_class = reply.value("status_class", std::string{});
    if (status_class.empty()) {
        if (ok && !warning.empty()) {
            status_class = "warning";
        } else if (ok && (status.find("degraded") != std::string::npos || status.find("warning") != std::string::npos)) {
            status_class = "degraded";
        } else if (ok) {
            status_class = "ok";
        } else if (status.find("invalid") != std::string::npos ||
                   status.find("validation") != std::string::npos ||
                   status.find("must ") != std::string::npos ||
                   status.find("not saved") != std::string::npos) {
            status_class = "rejected_invalid";
        } else {
            status_class = "failed_fatal";
        }
    }
    reply["ok"] = ok;
    reply["command"] = std::move(command);
    reply["status_class"] = status_class;
    reply["degraded"] = status_class == "degraded";
    reply["warning_only"] = status_class == "warning";
    reply["rejected_invalid"] = status_class == "rejected_invalid";
    reply["failed_fatal"] = status_class == "failed_fatal";
    return reply;
}

class BodytrackerDesktopController final : public bt::DesktopUiController {
public:
    explicit BodytrackerDesktopController(std::filesystem::path config_path)
        : config_path_(std::move(config_path)) {
        EnsureConfig();
        ReloadConfig();
        StartCameraScan();
        StartRuntime();
    }

    ~BodytrackerDesktopController() override {
        StopRuntime();
        if (scan_thread_.joinable()) {
            scan_thread_.join();
        }
    }

    nlohmann::json GetStateJson() override {
        std::scoped_lock lock(mutex_);
        const auto runtime_snapshot = runtime_state_.Snapshot();
        const bt::DebugSnapshot& debug = runtime_snapshot.debug;
        const bt::ModelSessionInfo& model_info = runtime_snapshot.model;
        std::error_code model_exists_ec;
        const bool model_exists = std::filesystem::exists(config_.tracking.model_path, model_exists_ec) && !model_exists_ec;

        nlohmann::json cameras = nlohmann::json::array();
        int open_count = 0;
        const auto runtime_camera_json = [&runtime_snapshot](int index, const bt::CameraConfig& config, const bt::CaptureHealthSnapshot& health, char slot) {
            const bool live = health.opened || health.source_state == "connected" || health.source_state == "receiving";
            const std::string& preview = (slot == 'b' || slot == 'B')
                ? runtime_snapshot.camera_b_preview_data_url
                : runtime_snapshot.camera_a_preview_data_url;
            const std::string& full_preview = (slot == 'b' || slot == 'B')
                ? runtime_snapshot.camera_b_full_preview_data_url
                : runtime_snapshot.camera_a_full_preview_data_url;
            const int source_x = (slot == 'b' || slot == 'B') ? runtime_snapshot.camera_b_preview_source_x : runtime_snapshot.camera_a_preview_source_x;
            const int source_y = (slot == 'b' || slot == 'B') ? runtime_snapshot.camera_b_preview_source_y : runtime_snapshot.camera_a_preview_source_y;
            const int source_width = (slot == 'b' || slot == 'B') ? runtime_snapshot.camera_b_preview_source_width : runtime_snapshot.camera_a_preview_source_width;
            const int source_height = (slot == 'b' || slot == 'B') ? runtime_snapshot.camera_b_preview_source_height : runtime_snapshot.camera_a_preview_source_height;
            const int frame_width = (slot == 'b' || slot == 'B') ? runtime_snapshot.camera_b_preview_frame_width : runtime_snapshot.camera_a_preview_frame_width;
            const int frame_height = (slot == 'b' || slot == 'B') ? runtime_snapshot.camera_b_preview_frame_height : runtime_snapshot.camera_a_preview_frame_height;
            return nlohmann::json{
                {"index", index},
                {"opened", live},
                {"width", health.actual_width > 0 ? health.actual_width : config.width},
                {"height", health.actual_height > 0 ? health.actual_height : config.height},
                {"fps", health.actual_fps > 0.0 ? health.actual_fps : static_cast<double>(config.fps)},
                {"backend", health.backend_name.empty() ? std::string("network_mjpeg_tcp") : health.backend_name},
                {"preview", preview},
                {"full_preview", full_preview.empty() ? preview : full_preview},
                {"preview_source_x", source_x},
                {"preview_source_y", source_y},
                {"preview_source_width", source_width > 0 ? source_width : (health.actual_width > 0 ? health.actual_width : config.width)},
                {"preview_source_height", source_height > 0 ? source_height : (health.actual_height > 0 ? health.actual_height : config.height)},
                {"preview_frame_width", frame_width > 0 ? frame_width : (health.actual_width > 0 ? health.actual_width : config.width)},
                {"preview_frame_height", frame_height > 0 ? frame_height : (health.actual_height > 0 ? health.actual_height : config.height)},
                {"source", config.source},
                {"state", health.source_state},
                {"frames", health.delivered_frames},
                {"last_frame_status", health.last_frame_status}
            };
        };
        bool camera_a_listed = false;
        bool camera_b_listed = false;
        for (const auto& camera : cameras_) {
            nlohmann::json item;
            if (config_.camera_a.source == "network_mjpeg" && camera.index == config_.camera_a.device_index) {
                item = runtime_camera_json(camera.index, config_.camera_a, debug.camera_a, 'a');
                camera_a_listed = true;
            } else if (config_.camera_b.source == "network_mjpeg" && camera.index == config_.camera_b.device_index) {
                item = runtime_camera_json(camera.index, config_.camera_b, debug.camera_b, 'b');
                camera_b_listed = true;
            } else {
                item = {
                    {"index", camera.index},
                    {"opened", camera.opened},
                    {"width", camera.width},
                    {"height", camera.height},
                    {"fps", camera.fps},
                    {"backend", camera.backend_name},
                    {"preview", camera.preview_data_url}
                };
            }
            if (item.value("opened", false)) {
                ++open_count;
            }
            cameras.push_back(std::move(item));
        }
        if (config_.camera_a.source == "network_mjpeg" && !camera_a_listed) {
            auto item = runtime_camera_json(config_.camera_a.device_index, config_.camera_a, debug.camera_a, 'a');
            if (item.value("opened", false)) {
                ++open_count;
            }
            cameras.insert(cameras.begin(), std::move(item));
        }
        if (config_.camera_b.source == "network_mjpeg" && !camera_b_listed) {
            auto item = runtime_camera_json(config_.camera_b.device_index, config_.camera_b, debug.camera_b, 'b');
            if (item.value("opened", false)) {
                ++open_count;
            }
            cameras.push_back(std::move(item));
        }

        const bool monocular_mode = config_.tracking.mode == bt::TrackingMode::Monocular;
        bt::CalibrationBundle calibration_bundle_for_alignment;
        nlohmann::json calibration = {{"tracking_ready", false}, {"summary", "calibration not loaded"}};
        std::error_code calibration_exists_ec;
        const bool calibration_exists = std::filesystem::exists(config_.tracking.calibration_path, calibration_exists_ec) && !calibration_exists_ec;
        if (calibration_exists_ec) {
            calibration["summary"] = std::string("failed to inspect calibration path: ") + calibration_exists_ec.message();
            calibration["floor_source"] = "nothing";
        } else if (calibration_exists) {
            const auto loaded = bt::LoadCalibration(config_.tracking.calibration_path);
            if (loaded.ok()) {
                calibration_bundle_for_alignment = loaded.value();
                const auto readiness = bt::EvaluateCalibrationReadiness(calibration_bundle_for_alignment, config_.tracking.mode);
                calibration = CalibrationStateToJson(calibration_bundle_for_alignment, readiness);
            } else if (monocular_mode) {
                const auto readiness = bt::EvaluateCalibrationReadiness(bt::CalibrationBundle{}, config_.tracking.mode);
                calibration = {
                    {"tracking_ready", readiness.tracking_ready},
                    {"summary", std::string("monocular mode using markerless profile; stereo calibration ignored: ") + loaded.status().message},
                    {"floor_source", "nothing"}
                };
            } else {
                calibration["summary"] = loaded.status().message;
                calibration["floor_source"] = "nothing";
            }
        } else if (monocular_mode) {
            const auto readiness = bt::EvaluateCalibrationReadiness(bt::CalibrationBundle{}, config_.tracking.mode);
            calibration = {
                {"tracking_ready", readiness.tracking_ready},
                {"summary", readiness.summary},
                {"floor_source", "nothing"}
            };
        }

        const bool steamvr_session_active = alignment_manager_.SessionActive();
        if (ShouldPollSteamVrProvider(config_.osc, steamvr_session_active)) {
            alignment_manager_.Poll();
        }
        const bool body_state_stable = debug.tracking.body_state.valid;
        bt::SteamVrAlignmentStatus steamvr_status = alignment_manager_.Status(config_.osc, calibration_bundle_for_alignment, body_state_stable);
        // State authority: the menu alignment manager owns the interactive
        // calibration wizard. Runtime debug may report freshness for an already
        // active SteamVR tracker-space transform, but it must not overwrite an
        // in-progress menu calibration session with an unrelated/unpolled runtime
        // snapshot.
        if (!steamvr_session_active && runtime_running_ && debug.steamvr_alignment_recorded) {
            steamvr_status = debug.steamvr_alignment;
        }

        return {
            {"config_path", ResolveDisplayPath(config_path_)},
            {"config", ConfigToJson(config_)},
            {"model", {
                {"path", ResolveDisplayPath(config_.tracking.model_path)},
                {"folder", ResolveDisplayPath(config_.tracking.model_path.parent_path().empty() ? std::filesystem::path("models") : config_.tracking.model_path.parent_path())},
                {"exists", model_exists},
                {"inspect_error", model_exists_ec ? model_exists_ec.message() : std::string()},
                {"active_device", model_info.loaded ? model_info.active_device : debug.model_active_device},
                {"ep_fallback", model_info.loaded ? model_info.ep_fallback : debug.model_ep_fallback}
            }},
            {"calibration", calibration},
            {"floor_assist", FloorAssistStateToJson(config_, debug)},
            {"tracker_space", TrackerSpaceStateToJson(config_, &steamvr_status)},
            {"steamvr_alignment", bt::AlignmentStatusToJson(steamvr_status)},
            {"body_overlay", BodyOverlayToJson(runtime_snapshot.body_overlay)},
            {"phone_site", PhoneSiteStateJsonForPort(config_.camera_a.network_port)},
            {"cameras", {
                {"scanning", scanning_},
                {"status", camera_status_},
                {"open_count", open_count},
                {"items", cameras}
            }},
            {"runtime", {
                {"running", runtime_running_},
                {"last_exit_code", runtime_exit_code_},
                {"last_error", runtime_error_}
            }},
            {"debug", DebugToJson(debug)}
        };
    }

    nlohmann::json HandleCommand(const std::string& command, const nlohmann::json& payload) override {
        return NormalizeCommandReply(command, HandleCommandRaw(command, payload));
    }

    nlohmann::json HandleCommandRaw(const std::string& command, const nlohmann::json& payload) {
        if (command == "scanCameras") {
            StartCameraScan();
            return {{"status", "scan started"}};
        }
        if (command == "enablePhoneWebCamera") {
            return EnablePhoneWebCamera(false);
        }
        if (command == "launchPhoneWebCamera") {
            return EnablePhoneWebCamera(true);
        }
        if (command == "openPhoneWebCamera") {
            return OpenPhoneWebCamera();
        }
        if (command == "disablePhoneWebCamera") {
            return DisablePhoneWebCamera();
        }
        if (command == "rescanModel") {
            return RescanModel();
        }
        if (command == "openModelsFolder") {
            return OpenModelsFolder();
        }
        if (command == "openCalibrationFolder") {
            return OpenCalibrationFolder();
        }
        if (command == "createCalibrationTemplate") {
            return CreateCalibrationTemplate();
        }
        if (command == "openBuildFolder") {
            return OpenBuildFolder();
        }
        if (command == "prepareDeployFolder") {
            return PrepareDeployFolder();
        }
        if (command == "setCamera") {
            const std::string slot = payload.value("slot", "");
            const int index = payload.value("index", 0);
            std::scoped_lock lock(mutex_);
            if (slot == "a") {
                if (config_.camera_a.source == "network_mjpeg" && index == config_.camera_a.device_index) {
                    return {{"status", "phone camera selected"}};
                }
                config_.camera_a = CameraConfigForProbe(cameras_, index, config_.camera_a);
                if (config_.camera_a.source != "network_mjpeg") {
                    runtime_state_.SetConfig(config_);
                }
            } else if (slot == "b") {
                if (config_.camera_b.source == "network_mjpeg" && index == config_.camera_b.device_index) {
                    return {{"status", "phone camera selected"}};
                }
                config_.camera_b = CameraConfigForProbe(cameras_, index, config_.camera_b);
                if (config_.camera_b.source != "network_mjpeg") {
                    runtime_state_.SetConfig(config_);
                }
            }
            return {{"status", "camera selected"}};
        }
        if (command == "calibrateFloorGeometryBackend") {
            return CalibrateFloorGeometryBackend(payload);
        }
        if (command == "refreshCameraPreview") {
            return RefreshCameraPreview(payload);
        }
        if (command == "steamVrAlignmentStart") {
            return SteamVrAlignmentStartCommand();
        }
        if (command == "steamVrAlignmentRecord") {
            return SteamVrAlignmentRecordCommand(payload);
        }
        if (command == "steamVrAlignmentRedo") {
            return SteamVrAlignmentRedoCommand(payload);
        }
        if (command == "steamVrAlignmentFinish") {
            return SteamVrAlignmentFinishCommand();
        }
        if (command == "steamVrAlignmentClear") {
            return SteamVrAlignmentClearCommand();
        }
        if (command == "saveConfig") {
            return SaveConfigFromUi(payload);
        }
        if (command == "startRuntime") {
            return StartRuntime();
        }
        if (command == "stopRuntime") {
            RequestStopRuntime();
            return {{"status", "runtime stop requested"}};
        }
        return {{"status", "unknown command"}, {"ok", false}};
    }

    void OnUiClosed() override {
        StopRuntime();
    }

private:
    void EnsureConfig() {
        std::error_code ec;
        const bool exists = std::filesystem::exists(config_path_, ec);
        if (ec) {
            std::scoped_lock lock(mutex_);
            runtime_error_ = std::string("failed to inspect config path: ") + ec.message();
            return;
        }
        if (exists) {
            return;
        }
        const auto s = bt::SaveDefaultConfig(config_path_);
        if (!s.ok()) {
            std::scoped_lock lock(mutex_);
            runtime_error_ = s.message;
        }
    }

    void ReloadConfig() {
        const auto loaded = bt::LoadConfig(config_path_);
        std::scoped_lock lock(mutex_);
        if (loaded.ok()) {
            config_ = loaded.value();
            runtime_error_.clear();
            runtime_state_.SetConfig(config_);
        } else {
            runtime_error_ = loaded.status().message;
        }
    }

    void StartCameraScan() {
        {
            std::scoped_lock lock(mutex_);
            if (scanning_) {
                return;
            }
        }
        if (scan_thread_.joinable()) {
            scan_thread_.join();
        }
        {
            std::scoped_lock lock(mutex_);
            scanning_ = true;
            camera_status_ = "scanning cameras";
        }
        scan_thread_ = std::thread([this]() {
            auto probes = ScanCameraIndices();
            std::scoped_lock lock(mutex_);
            cameras_ = std::move(probes);
            scanning_ = false;
            camera_status_ = "scan complete";
        });
    }

    nlohmann::json StartRuntime() {
        StopFinishedRuntimeThread();
        {
            std::scoped_lock lock(mutex_);
            if (runtime_running_) {
                return {{"status", "runtime already running"}};
            }
            runtime_error_.clear();
            runtime_exit_code_ = -1;
            runtime_running_ = true;
        }

        g_stop_requested = false;
        runtime_thread_ = std::thread([this]() {
            RuntimeRunOptions options;
            options.show_native_dashboard = false;
            options.interactive_setup_on_failure = false;
            options.external_web_state = &runtime_state_;
            const int code = RunRuntime(config_path_, options);
            std::scoped_lock lock(mutex_);
            runtime_exit_code_ = code;
            runtime_running_ = false;
            if (code != 0 && runtime_error_.empty()) {
                runtime_error_ = "runtime exited with code " + std::to_string(code);
            }
        });
        return {{"status", "runtime started"}};
    }

    void StopRuntime() {
        g_stop_requested = true;
        if (runtime_thread_.joinable()) {
            runtime_thread_.join();
        }
        std::scoped_lock lock(mutex_);
        runtime_running_ = false;
    }

    void RequestStopRuntime() {
        g_stop_requested = true;
        std::scoped_lock lock(mutex_);
        if (!runtime_running_) {
            return;
        }
        runtime_error_.clear();
    }

    void StopFinishedRuntimeThread() {
        bool should_join = false;
        {
            std::scoped_lock lock(mutex_);
            should_join = !runtime_running_ && runtime_thread_.joinable();
        }
        if (should_join) {
            runtime_thread_.join();
        }
    }

    nlohmann::json RescanModel() {
        StopFinishedRuntimeThread();
        ReloadConfig();

        bt::AppConfig cfg_snapshot;
        bool runtime_running_snapshot = false;
        {
            std::scoped_lock lock(mutex_);
            cfg_snapshot = config_;
            runtime_running_snapshot = runtime_running_;
        }

        std::error_code model_exists_ec;
        const bool model_exists = std::filesystem::exists(cfg_snapshot.tracking.model_path, model_exists_ec) && !model_exists_ec;
        if (model_exists_ec) {
            bt::ModelSessionInfo unloaded;
            unloaded.loaded = false;
            unloaded.model_path = cfg_snapshot.tracking.model_path;
            unloaded.active_device = cfg_snapshot.inference.device;
            runtime_state_.SetModel(unloaded);
            return {{"status", std::string("model inspect failed: ") + model_exists_ec.message()}, {"ok", false}};
        }
        if (!model_exists) {
            bt::ModelSessionInfo unloaded;
            unloaded.loaded = false;
            unloaded.model_path = cfg_snapshot.tracking.model_path;
            unloaded.active_device = cfg_snapshot.inference.device;
            runtime_state_.SetModel(unloaded);
            return {{"status", "model missing"}, {"ok", false}};
        }

        bt::RtmPoseSession probe;
        const auto load = probe.Load(cfg_snapshot.tracking.model_path, cfg_snapshot.inference.device);
        if (!load.ok()) {
            bt::ModelSessionInfo unloaded;
            unloaded.loaded = false;
            unloaded.model_path = cfg_snapshot.tracking.model_path;
            unloaded.active_device = cfg_snapshot.inference.device;
            runtime_state_.SetModel(unloaded);
            return {{"status", std::string("model load failed: ") + load.message}, {"ok", false}};
        }
        if (const auto contract = bt::ValidateRtmPoseModelContract(probe.GetInfo()); !contract.ok()) {
            runtime_state_.SetModel(probe.GetInfo());
            return {{"status", std::string("model contract failed: ") + contract.message}, {"ok", false}};
        }
        runtime_state_.SetModel(probe.GetInfo());
        return {
            {"status", runtime_running_snapshot ? "model rescanned; runtime is running" : "model rescanned and load-checked"},
            {"ok", true},
            {"model_loaded", probe.GetInfo().loaded},
            {"active_device", probe.GetInfo().active_device},
            {"ep_fallback", probe.GetInfo().ep_fallback}
        };
    }


    nlohmann::json RefreshCameraPreview(const nlohmann::json& payload) {
        bt::AppConfig cfg_snapshot;
        bt::WebRuntimeSnapshot runtime_snapshot;
        {
            std::scoped_lock lock(mutex_);
            cfg_snapshot = config_;
            runtime_snapshot = runtime_state_.Snapshot();
        }
        const int camera_index = payload.value("camera_a", cfg_snapshot.camera_a.device_index);
        if (cfg_snapshot.camera_a.source == "network_mjpeg" && camera_index == cfg_snapshot.camera_a.device_index) {
            const bool has_preview = !runtime_snapshot.camera_a_preview_data_url.empty();
            return {
                {"ok", has_preview},
                {"status", has_preview ? "phone camera preview refreshed" : "phone camera is connected; waiting for first preview frame"},
                {"camera", camera_index},
                {"source", "network_mjpeg"},
                {"preview_available", has_preview},
                {"preview", runtime_snapshot.camera_a_preview_data_url},
                {"full_preview", runtime_snapshot.camera_a_full_preview_data_url.empty()
                    ? runtime_snapshot.camera_a_preview_data_url
                    : runtime_snapshot.camera_a_full_preview_data_url},
                {"width", runtime_snapshot.camera_a_preview_width > 0 ? runtime_snapshot.camera_a_preview_width : cfg_snapshot.camera_a.width},
                {"height", runtime_snapshot.camera_a_preview_height > 0 ? runtime_snapshot.camera_a_preview_height : cfg_snapshot.camera_a.height},
                {"preview_source_x", runtime_snapshot.camera_a_preview_source_x},
                {"preview_source_y", runtime_snapshot.camera_a_preview_source_y},
                {"preview_source_width", runtime_snapshot.camera_a_preview_source_width > 0 ? runtime_snapshot.camera_a_preview_source_width : cfg_snapshot.camera_a.width},
                {"preview_source_height", runtime_snapshot.camera_a_preview_source_height > 0 ? runtime_snapshot.camera_a_preview_source_height : cfg_snapshot.camera_a.height},
                {"preview_frame_width", runtime_snapshot.camera_a_preview_frame_width > 0 ? runtime_snapshot.camera_a_preview_frame_width : cfg_snapshot.camera_a.width},
                {"preview_frame_height", runtime_snapshot.camera_a_preview_frame_height > 0 ? runtime_snapshot.camera_a_preview_frame_height : cfg_snapshot.camera_a.height},
                {"sequence", runtime_snapshot.camera_a_preview_sequence}
            };
        }
        CameraProbe probe = ProbeCameraIndex(camera_index);
        {
            std::scoped_lock lock(mutex_);
            bool replaced = false;
            for (auto& existing : cameras_) {
                if (existing.index == camera_index) {
                    existing = probe;
                    replaced = true;
                    break;
                }
            }
            if (!replaced) {
                cameras_.push_back(probe);
            }
            camera_status_ = probe.opened
                ? "preview refreshed for camera " + std::to_string(camera_index)
                : "preview refresh failed for camera " + std::to_string(camera_index);
        }
        return {
            {"ok", probe.opened && !probe.preview_data_url.empty()},
            {"status", probe.opened
                ? (!probe.preview_data_url.empty() ? "camera preview refreshed" : "camera opened but preview image was empty")
                : "camera preview refresh failed"},
            {"camera", camera_index},
            {"preview_available", !probe.preview_data_url.empty()},
            {"preview", probe.preview_data_url},
            {"width", probe.width},
            {"height", probe.height}
        };
    }


    nlohmann::json CalibrateFloorGeometryBackend(const nlohmann::json& payload) {
        bt::AppConfig cfg_snapshot;
        {
            std::scoped_lock lock(mutex_);
            cfg_snapshot = config_;
        }

        auto clamp01 = [](float v) {
            return std::clamp(v, 0.0f, 1.0f);
        };
        auto normalize_angle = [](float a) {
            constexpr float kPi = 3.14159265358979323846f;
            while (a <= -kPi) a += 2.0f * kPi;
            while (a > kPi) a -= 2.0f * kPi;
            return a;
        };
        auto axial_delta = [&](float a, float b) {
            constexpr float kPi = 3.14159265358979323846f;
            float d = std::abs(normalize_angle(a - b));
            if (d > 0.5f * kPi) d = kPi - d;
            return std::abs(d);
        };
        auto parse_point = [](const nlohmann::json& value, bt::Vec2f& out) {
            try {
                if (value.is_array() && value.size() >= 2) {
                    out.x = value.at(0).get<float>();
                    out.y = value.at(1).get<float>();
                    return std::isfinite(out.x) && std::isfinite(out.y);
                }
                if (value.is_object()) {
                    out.x = value.value("x", 0.0f);
                    out.y = value.value("y", 0.0f);
                    return std::isfinite(out.x) && std::isfinite(out.y);
                }
            } catch (...) {
            }
            return false;
        };
        auto line_length = [](const bt::FloorSeamLine2D& line) {
            const float dx = line.b.x - line.a.x;
            const float dy = line.b.y - line.a.y;
            return std::sqrt(dx * dx + dy * dy);
        };
        auto line_angle = [&](const bt::FloorSeamLine2D& line) {
            return normalize_angle(std::atan2(line.b.y - line.a.y, line.b.x - line.a.x));
        };
        auto line_midpoint = [](const bt::FloorSeamLine2D& line) {
            return bt::Vec2f{0.5f * (line.a.x + line.b.x), 0.5f * (line.a.y + line.b.y)};
        };
        auto line_intersection = [](const bt::FloorSeamLine2D& a, const bt::FloorSeamLine2D& b, bt::Vec2f& out) {
            const float x1 = a.a.x;
            const float y1 = a.a.y;
            const float x2 = a.b.x;
            const float y2 = a.b.y;
            const float x3 = b.a.x;
            const float y3 = b.a.y;
            const float x4 = b.b.x;
            const float y4 = b.b.y;
            const float den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
            if (!std::isfinite(den) || std::abs(den) < 1.0e-4f) return false;
            out.x = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / den;
            out.y = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / den;
            return std::isfinite(out.x) && std::isfinite(out.y);
        };
        auto point_json = [](const bt::Vec2f& p) {
            return nlohmann::json::array({p.x, p.y});
        };
        auto point_finite_reasonable = [](const bt::Vec2f& p) {
            constexpr float kMaxUsefulVanishingCoordinate = 1.0e6f;
            return std::isfinite(p.x) && std::isfinite(p.y) &&
                std::abs(p.x) <= kMaxUsefulVanishingCoordinate &&
                std::abs(p.y) <= kMaxUsefulVanishingCoordinate;
        };

        int image_w = payload.value("image_width", payload.value("image_w", cfg_snapshot.camera_a.width));
        int image_h = payload.value("image_height", payload.value("image_h", cfg_snapshot.camera_a.height));
        if (image_w <= 0 || image_h <= 0) {
            return {
                {"ok", false},
                {"status", "manual plank calibration needs preview image dimensions"},
                {"reason", "missing_image_dimensions"}
            };
        }

        if (payload.contains("wall_corners") && payload["wall_corners"].is_array()) {
            if (payload["wall_corners"].size() < 4) {
                return {
                    {"ok", false},
                    {"status", "draw wall rectangle: four corners required"},
                    {"reason", "manual_wall_rectangle_incomplete"}
                };
            }
            std::array<bt::Vec2f, 4> corners{};
            for (std::size_t i = 0; i < corners.size(); ++i) {
                if (!parse_point(payload["wall_corners"].at(i), corners[i])) {
                    return {
                        {"ok", false},
                        {"status", "draw wall rectangle: corner point invalid"},
                        {"reason", "manual_wall_rectangle_corner_invalid"},
                        {"corner", static_cast<int>(i)}
                    };
                }
            }
            bt::WallRectangleCalibrationOptions options;
            options.source = payload.value("source", std::string("manual_wall_rectangle"));
            options.rectangle_width_m = payload.value("wall_width_m", payload.value("family_a_spacing_m", 0.0f));
            options.rectangle_height_m = payload.value("wall_height_m", payload.value("family_b_spacing_m", 0.0f));
            options.rectangle_aspect_ratio = payload.value("wall_aspect_ratio", 0.0f);
            options.intrinsics_available = true;
            options.horizontal_fov_deg = payload.value("horizontal_fov_deg", cfg_snapshot.tracking.monocular.horizontal_fov_deg);
            const auto wall = bt::EstimateWallRectangleCalibration(corners, image_w, image_h, options);
            return {
                {"ok", wall.valid},
                {"status", wall.valid ? "wall rectangle captured as runtime wall evidence" : "wall rectangle captured but unusable"},
                {"source", wall.source},
                {"wall_geometry", WallRectangleToJson(wall)},
                {"reason", wall.reason},
                {"capability_reason", wall.capability_reason}
            };
        }

        std::vector<bt::FloorSeamLine2D> manual_lines;
        if (payload.contains("lines") && payload["lines"].is_array()) {
            for (const auto& item : payload["lines"]) {
                bt::FloorSeamLine2D line;
                bool ok = false;
                if (item.is_object()) {
                    ok = parse_point(item.value("a", nlohmann::json{}), line.a) &&
                         parse_point(item.value("b", nlohmann::json{}), line.b);
                    line.strength = clamp01(item.value("strength", 1.0f));
                } else if (item.is_array() && item.size() >= 2) {
                    ok = parse_point(item.at(0), line.a) && parse_point(item.at(1), line.b);
                }
                if (!ok || line_length(line) < 12.0f) {
                    continue;
                }
                line.samples = {line.a, line.b};
                manual_lines.push_back(line);
            }
        }

        if (manual_lines.size() < 2) {
            return {
                {"ok", false},
                {"status", "draw one plank: mark at least the two long edges"},
                {"reason", "manual_plank_outline_incomplete"},
                {"candidate_count", static_cast<int>(manual_lines.size())}
            };
        }
        if (manual_lines.size() == 2) {
            const bt::FloorSeamLine2D long_a = manual_lines[0];
            const bt::FloorSeamLine2D long_b = manual_lines[1];
            const float theta_a = line_angle(long_a);
            const float theta_b = line_angle(long_b);
            const float long_parallel_error = axial_delta(theta_a, theta_b);
            const float len_a = line_length(long_a);
            const float len_b = line_length(long_b);
            const float orientation = normalize_angle(0.5f * std::atan2(
                std::sin(2.0f * theta_a) * len_a + std::sin(2.0f * theta_b) * len_b,
                std::cos(2.0f * theta_a) * len_a + std::cos(2.0f * theta_b) * len_b));
            const float nx = -std::sin(orientation);
            const float ny = std::cos(orientation);
            const auto mid_a = line_midpoint(long_a);
            const auto mid_b = line_midpoint(long_b);
            const float rho_a = mid_a.x * nx + mid_a.y * ny;
            const float rho_b = mid_b.x * nx + mid_b.y * ny;
            const float width_px = std::abs(rho_b - rho_a);
            const float width_m = payload.value("family_a_spacing_m", payload.value("spacing_m", 0.0f));
            const bool width_metric_valid = width_m > 0.0f && std::isfinite(width_m);
            const float user_confidence = clamp01(payload.value("floor_confidence", 1.0f));
            const float width_quality = clamp01(width_px / std::max(16.0f, 0.02f * static_cast<float>(std::min(image_w, image_h))));
            const float length_quality = clamp01(std::min(len_a, len_b) / std::max(48.0f, 0.08f * static_cast<float>(std::min(image_w, image_h))));
            const float parallel_quality = clamp01(1.0f - long_parallel_error / 0.70f);
            const float confidence = clamp01((0.42f * width_quality + 0.38f * length_quality + 0.20f * parallel_quality) * user_confidence);
            if (!std::isfinite(width_px) || width_px <= 2.0f) {
                return {{"ok", false}, {"status", "plank long-edge spacing is too small"}, {"reason", "manual_plank_width_px_invalid"}, {"candidate_count", 2}};
            }
            nlohmann::json geometry = {
                {"valid", true},
                {"backend_owned", true},
                {"image_width", image_w},
                {"image_height", image_h},
                {"source", "manual_plank_two_edge_spacing"},
                {"floor_type", "manual_plank"},
                {"family_count", 1},
                {"family_a", {
                    {"valid", true},
                    {"confidence", confidence},
                    {"orientation_rad", orientation},
                    {"spacing_px", width_px},
                    {"spacing_m", width_metric_valid ? width_m : 0.0f},
                    {"metric_spacing_valid", width_metric_valid},
                    {"reference_rho_px", std::min(rho_a, rho_b)},
                    {"accepted_line_count", 2},
                    {"reason", width_metric_valid ? "two_long_edges_metric_width" : "two_long_edges_metric_width_missing"}
                }},
                {"family_b", {{"valid", false}}},
                {"two_axis_grid_valid", false},
                {"homography_valid", false},
                {"homography_reason", "two_long_edges_only_scalar_spacing"},
                {"metric_scale_confidence", width_metric_valid ? confidence : 0.0f},
                {"planted_drift_axis_confidence", width_metric_valid ? confidence : 0.0f},
                {"camera_height_valid", true},
                {"camera_height_m", payload.value("camera_height_m", cfg_snapshot.tracking.monocular.camera_height_m)},
                {"reason", width_metric_valid ? "two_long_edges_floor_spacing_ready" : "two_long_edges_need_real_width_m"},
                {"manual_plank", {
                    {"valid", true},
                    {"width_px", width_px},
                    {"width_m", width_metric_valid ? width_m : 0.0f},
                    {"width_metric_valid", width_metric_valid},
                    {"end_cap_valid", false},
                    {"full_quad_valid", false},
                    {"scalar_floor_spacing_usable", width_metric_valid}
                }}
            };
            return {
                {"ok", true},
                {"status", width_metric_valid ? "two plank edges accepted as scalar floor calibration" : "two plank edges captured; enter real plank width to use metric scale"},
                {"source", "manual_plank_two_edge_spacing"},
                {"floor_geometry", geometry},
                {"candidate_count", 2},
                {"spacing_px", width_px},
                {"spacing_m", width_metric_valid ? width_m : 0.0f}
            };
        }

        const bt::FloorSeamLine2D long_a = manual_lines[0];
        const bt::FloorSeamLine2D long_b = manual_lines[1];
        const bt::FloorSeamLine2D cap_a = manual_lines[2];
        const bool has_cap_b = manual_lines.size() >= 4;
        const bt::FloorSeamLine2D cap_b = has_cap_b ? manual_lines[3] : bt::FloorSeamLine2D{};

        auto point_distance = [](const bt::Vec2f& a, const bt::Vec2f& b) {
            const float dx = b.x - a.x;
            const float dy = b.y - a.y;
            return std::sqrt(dx * dx + dy * dy);
        };
        auto point_mid = [](const bt::Vec2f& a, const bt::Vec2f& b) {
            return bt::Vec2f{0.5f * (a.x + b.x), 0.5f * (a.y + b.y)};
        };

        const float theta_a = line_angle(long_a);
        const float theta_b = line_angle(long_b);
        const float long_parallel_error = axial_delta(theta_a, theta_b);
        const float len_a = line_length(long_a);
        const float len_b = line_length(long_b);
        const float cap_len = line_length(cap_a);
        const float orientation = normalize_angle(0.5f * std::atan2(
            std::sin(2.0f * theta_a) * len_a + std::sin(2.0f * theta_b) * len_b,
            std::cos(2.0f * theta_a) * len_a + std::cos(2.0f * theta_b) * len_b));
        const float cap_orientation = line_angle(cap_a);
        const float cap_perp_error = std::abs(axial_delta(cap_orientation, orientation) - 1.57079632679f);

        bt::Vec2f corner_aa{}, corner_ba{}, corner_ab{}, corner_bb{};
        const bool end_cap_intersections = line_intersection(long_a, cap_a, corner_aa) &&
            line_intersection(long_b, cap_a, corner_ba);
        if (!end_cap_intersections) {
            return {
                {"ok", false},
                {"status", "short end cap must cross both long plank edges"},
                {"reason", "manual_end_cap_does_not_cross_long_edges"}
            };
        }

        const float cap_a_width_px = point_distance(corner_aa, corner_ba);
        if (!std::isfinite(cap_a_width_px) || cap_a_width_px <= 2.0f) {
            return {
                {"ok", false},
                {"status", "manual plank pixel width is invalid"},
                {"reason", "manual_plank_width_px_invalid"},
                {"end_cap_span_px", cap_a_width_px}
            };
        }

        bool full_quad = false;
        float cap_b_width_px = 0.0f;
        float cap_spacing_px = 0.0f;
        float cap_parallel_error = 0.0f;
        if (has_cap_b) {
            const bool cap_b_intersections = line_intersection(long_a, cap_b, corner_ab) &&
                line_intersection(long_b, cap_b, corner_bb);
            if (!cap_b_intersections) {
                return {
                    {"ok", false},
                    {"status", "opposite short end cap must cross both long plank edges"},
                    {"reason", "manual_opposite_end_cap_does_not_cross_long_edges"}
                };
            }
            cap_b_width_px = point_distance(corner_ab, corner_bb);
            if (!std::isfinite(cap_b_width_px) || cap_b_width_px <= 2.0f) {
                return {
                    {"ok", false},
                    {"status", "opposite short end cap produced an invalid plank width"},
                    {"reason", "manual_opposite_end_cap_width_px_invalid"},
                    {"opposite_end_cap_span_px", cap_b_width_px}
                };
            }
            const bt::Vec2f mid_cap_a = point_mid(corner_aa, corner_ba);
            const bt::Vec2f mid_cap_b = point_mid(corner_ab, corner_bb);
            cap_spacing_px = point_distance(mid_cap_a, mid_cap_b);
            const float width_ratio = cap_b_width_px / std::max(1.0f, cap_a_width_px);
            cap_parallel_error = axial_delta(cap_orientation, line_angle(cap_b));
            if (!std::isfinite(cap_spacing_px) || cap_spacing_px <= 2.0f) {
                return {
                    {"ok", false},
                    {"status", "opposite short end cap is too close to the first end cap"},
                    {"reason", "manual_opposite_end_cap_length_px_invalid"},
                    {"plank_length_px", cap_spacing_px}
                };
            }
            if (!std::isfinite(width_ratio) || width_ratio < 0.25f || width_ratio > 4.0f) {
                return {
                    {"ok", false},
                    {"status", "opposite short end cap width is wildly inconsistent with the first cap"},
                    {"reason", "manual_opposite_end_cap_span_mismatch"},
                    {"first_end_cap_span_px", cap_a_width_px},
                    {"opposite_end_cap_span_px", cap_b_width_px}
                };
            }
            full_quad = true;
        }

        const float width_m = payload.value("family_a_spacing_m", payload.value("spacing_m", 0.0f));
        const float length_m = payload.value("family_b_spacing_m", payload.value("plank_length_m", 0.0f));
        const bool width_metric_valid = width_m > 0.0f && std::isfinite(width_m);
        const bool length_metric_valid = full_quad && cap_spacing_px > 2.0f && length_m > 0.0f && std::isfinite(length_m);

        // Manual plank width is metric evidence.  Full-quad homography remains the
        // richer solve, but single-plank width can still drive degraded scalar assist.
        const bool scalar_spacing_usable = width_metric_valid;
        const float width_px = full_quad ? 0.5f * (cap_a_width_px + cap_b_width_px) : cap_a_width_px;

        const float nx = -std::sin(orientation);
        const float ny = std::cos(orientation);
        const auto mid_a = line_midpoint(long_a);
        const auto mid_b = line_midpoint(long_b);
        const float rho_a = mid_a.x * nx + mid_a.y * ny;
        const float rho_b = mid_b.x * nx + mid_b.y * ny;
        bt::Vec2f long_vp{};
        const bool long_vp_valid = line_intersection(long_a, long_b, long_vp) && point_finite_reasonable(long_vp);
        bt::Vec2f cap_vp{};
        const bool cap_vp_valid = full_quad && line_intersection(cap_a, cap_b, cap_vp) && point_finite_reasonable(cap_vp);
        const float cap_width_ratio = full_quad ? cap_b_width_px / std::max(1.0f, cap_a_width_px) : 0.0f;
        const float long_edge_length_ratio = len_b / std::max(1.0f, len_a);
        const bool projective_perspective_observed = full_quad &&
            (long_vp_valid || cap_vp_valid || std::abs(cap_width_ratio - 1.0f) > 0.08f || std::abs(long_edge_length_ratio - 1.0f) > 0.08f);

        const float width_quality = clamp01(width_px / std::max(20.0f, 0.03f * static_cast<float>(std::min(image_w, image_h))));
        const float length_quality = clamp01(std::min(len_a, len_b) / std::max(60.0f, 0.10f * static_cast<float>(std::min(image_w, image_h))));
        const float cap_quality = clamp01(cap_len / std::max(12.0f, 0.50f * width_px));
        const float parallel_quality = clamp01(1.0f - long_parallel_error / 0.70f);
        const float perp_quality = clamp01(1.0f - cap_perp_error / 1.10f);
        const float cap_b_quality = full_quad ? clamp01(1.0f - cap_parallel_error / 1.10f) : 0.65f;
        const float projective_quality = full_quad ? clamp01(0.65f + 0.35f * cap_b_quality) : 0.65f;
        const float user_confidence = clamp01(payload.value("floor_confidence", 1.0f));
        const float confidence = clamp01((0.24f * width_quality + 0.24f * length_quality + 0.18f * cap_quality + 0.12f * parallel_quality + 0.12f * perp_quality + 0.10f * projective_quality) * user_confidence);

        auto identity_h = []() {
            return std::array<float, 9>{1.0f, 0.0f, 0.0f,
                                        0.0f, 1.0f, 0.0f,
                                        0.0f, 0.0f, 1.0f};
        };
        auto invert_h = [](const std::array<float, 9>& h, std::array<float, 9>& out) {
            const float det = h[0] * (h[4] * h[8] - h[5] * h[7]) -
                h[1] * (h[3] * h[8] - h[5] * h[6]) +
                h[2] * (h[3] * h[7] - h[4] * h[6]);
            if (!std::isfinite(det) || std::abs(det) <= 1.0e-8f) {
                return false;
            }
            const float inv = 1.0f / det;
            out = {
                (h[4] * h[8] - h[5] * h[7]) * inv,
                (h[2] * h[7] - h[1] * h[8]) * inv,
                (h[1] * h[5] - h[2] * h[4]) * inv,
                (h[5] * h[6] - h[3] * h[8]) * inv,
                (h[0] * h[8] - h[2] * h[6]) * inv,
                (h[2] * h[3] - h[0] * h[5]) * inv,
                (h[3] * h[7] - h[4] * h[6]) * inv,
                (h[1] * h[6] - h[0] * h[7]) * inv,
                (h[0] * h[4] - h[1] * h[3]) * inv};
            return std::all_of(out.begin(), out.end(), [](float v) { return std::isfinite(v); });
        };
        auto solve_rect_to_image = [&](float w_m, float l_m, const std::array<bt::Vec2f, 4>& img, std::array<float, 9>& h) {
            if (!(w_m > 0.0f) || !(l_m > 0.0f) || !std::isfinite(w_m) || !std::isfinite(l_m)) {
                return false;
            }
            const float src[4][2] = {{0.0f, 0.0f}, {w_m, 0.0f}, {w_m, l_m}, {0.0f, l_m}};
            float a[8][9]{};
            for (int i = 0; i < 4; ++i) {
                const float x = src[i][0];
                const float y = src[i][1];
                const float u = img[i].x;
                const float v = img[i].y;
                if (!std::isfinite(u) || !std::isfinite(v)) return false;
                const int r = 2 * i;
                a[r][0] = x; a[r][1] = y; a[r][2] = 1.0f; a[r][6] = -u * x; a[r][7] = -u * y; a[r][8] = u;
                a[r + 1][3] = x; a[r + 1][4] = y; a[r + 1][5] = 1.0f; a[r + 1][6] = -v * x; a[r + 1][7] = -v * y; a[r + 1][8] = v;
            }
            for (int col = 0; col < 8; ++col) {
                int pivot = col;
                float best = std::abs(a[col][col]);
                for (int row = col + 1; row < 8; ++row) {
                    const float score = std::abs(a[row][col]);
                    if (score > best) { best = score; pivot = row; }
                }
                if (!std::isfinite(best) || best <= 1.0e-8f) return false;
                if (pivot != col) {
                    for (int c = col; c < 9; ++c) std::swap(a[col][c], a[pivot][c]);
                }
                const float inv_pivot = 1.0f / a[col][col];
                for (int c = col; c < 9; ++c) a[col][c] *= inv_pivot;
                for (int row = 0; row < 8; ++row) {
                    if (row == col) continue;
                    const float f = a[row][col];
                    for (int c = col; c < 9; ++c) a[row][c] -= f * a[col][c];
                }
            }
            h = {a[0][8], a[1][8], a[2][8], a[3][8], a[4][8], a[5][8], a[6][8], a[7][8], 1.0f};
            return true;
        };

        bt::FloorGeometryDetectionDebug debug;
        debug.pattern.valid = true;
        debug.pattern.pattern_type = "manual_plank_outline";
        debug.pattern.confidence = confidence;
        debug.pattern.spacing_confidence = width_quality;
        debug.pattern.orientation_rad = orientation;
        debug.pattern.secondary_orientation_rad = cap_orientation;
        debug.pattern.secondary_valid = true;
        debug.pattern.spacing_px = width_px;
        debug.pattern.secondary_spacing_px = cap_spacing_px;
        debug.pattern.major_seam_count = 2;
        debug.pattern.secondary_seam_count = has_cap_b ? 2 : 1;
        debug.pattern.major_seams = {long_a, long_b};
        debug.pattern.secondary_seams = has_cap_b ? std::vector<bt::FloorSeamLine2D>{cap_a, cap_b}
                                                 : std::vector<bt::FloorSeamLine2D>{cap_a};
        debug.pattern.reason = "manual_user_drawn_plank_outline";

        for (std::size_t i = 0; i < manual_lines.size(); ++i) {
            const auto& line = manual_lines[i];
            bt::FloorSeamCandidateDebug candidate;
            candidate.line = line;
            candidate.angle_rad = line_angle(line);
            const auto mid = line_midpoint(line);
            candidate.rho_px = mid.x * nx + mid.y * ny;
            candidate.accepted = i < 4;
            candidate.reason = i < 2 ? "manual_user_drawn_long_edge" : "manual_user_drawn_end_cap";
            debug.candidates.push_back(std::move(candidate));
        }

        auto& g = debug.calibration;
        g.valid = true;
        g.image_width = image_w;
        g.image_height = image_h;
        g.source = "manual_plank_outline";
        g.floor_type = payload.value("floor_type", std::string("manual_plank"));
        g.family_count = has_cap_b ? 2 : 1;
        g.family_a.valid = true;
        g.family_a.confidence = confidence;
        g.family_a.orientation_rad = orientation;
        g.family_a.spacing_px = width_px;
        g.family_a.spacing_m = width_metric_valid ? width_m : 0.0f;
        g.family_a.metric_spacing_valid = width_metric_valid;
        g.family_a.reference_rho_px = std::min(rho_a, rho_b);
        g.family_a.vanishing_point_px = long_vp;
        g.family_a.vanishing_point_valid = long_vp_valid;
        g.family_a.accepted_line_count = 2;
        g.family_a.reason = width_metric_valid
            ? "manual_plank_width_metric_evidence"
            : "manual_plank_width_edges_metric_unknown";
        g.family_b.valid = full_quad;
        g.family_b.confidence = full_quad ? confidence : 0.0f;
        g.family_b.orientation_rad = cap_orientation;
        g.family_b.spacing_px = full_quad ? cap_spacing_px : 0.0f;
        g.family_b.spacing_m = length_metric_valid ? length_m : 0.0f;
        g.family_b.metric_spacing_valid = length_metric_valid;
        g.family_b.vanishing_point_px = cap_vp;
        g.family_b.vanishing_point_valid = cap_vp_valid;
        g.family_b.accepted_line_count = full_quad ? 2 : 0;
        g.family_b.reason = full_quad ? "manual_plank_end_caps" : "single_end_cap_geometry_recorded_in_manual_plank_not_a_line_family";
        g.two_axis_grid_valid = full_quad && cap_spacing_px > 2.0f;
        g.homography_valid = false;
        g.floor_from_image = identity_h();
        g.image_from_floor = identity_h();
        g.homography_reason = "manual_plank_outline_geometry_only_no_projective_solve";
        g.distortion.available = false;
        g.distortion.valid = false;
        g.distortion.confidence = 0.0f;
        g.distortion.model = "none";
        g.distortion.sampled_seam_count = 0;
        g.distortion.sampled_point_count = 0;
        g.distortion.reason = "manual_plank_endpoint_lines_do_not_estimate_lens_distortion_projective_perspective_is_homography";
        if (full_quad && width_metric_valid && length_metric_valid) {
            std::array<float, 9> image_from_floor{};
            const std::array<bt::Vec2f, 4> quad{corner_aa, corner_ba, corner_bb, corner_ab};
            std::array<float, 9> floor_from_image{};
            if (solve_rect_to_image(width_m, length_m, quad, image_from_floor) &&
                invert_h(image_from_floor, floor_from_image)) {
                g.image_from_floor = image_from_floor;
                g.floor_from_image = floor_from_image;
                g.homography_valid = true;
                g.homography_reprojection_error_px = 0.0f;
                g.homography_inlier_count = 4;
                g.homography_intersection_count = 4;
                g.homography_reason = "manual_plank_quad_projective_homography_from_four_corners";
            }
        }
        g.floor_plane.valid = false;
        g.floor_plane.normal = bt::Vec3f{0.0f, 1.0f, 0.0f};
        g.floor_plane.distance = 0.0f;
        g.floor_plane_confidence = 0.0f;
        g.camera_yaw_rad = orientation;
        // Manual plank edges give a useful floor yaw/orientation hint. They do not, by
        // themselves, prove camera pitch/roll. Do not route yaw-only evidence into the
        // runtime camera-orientation correction path, which expects pitch/roll values.
        g.camera_orientation_valid = false;
        g.camera_orientation_confidence = 0.0f;
        const float camera_height = payload.value("camera_height_m", cfg_snapshot.tracking.monocular.camera_height_m);
        if (camera_height > 0.10f && std::isfinite(camera_height)) {
            g.camera_height_valid = true;
            g.camera_height_m = camera_height;
        }
        const bool metric_scale_usable = width_metric_valid && (scalar_spacing_usable || g.homography_valid);
        g.metric_scale_confidence = metric_scale_usable ? confidence : 0.0f;
        g.floor_plane.valid = metric_scale_usable;
        g.floor_plane_confidence = metric_scale_usable ? clamp01(0.35f * confidence) : 0.0f;
        g.planted_drift_axis_confidence = (width_metric_valid && scalar_spacing_usable) ? confidence : 0.0f;
        g.reason = metric_scale_usable
            ? (g.homography_valid ? "manual_plank_outline_with_projective_quad" : "manual_plank_width_metric_assist")
            : "manual_plank_outline_metric_unknown";

        nlohmann::json plank = {
            {"valid", true},
            {"width_px", width_px},
            {"width_m", width_metric_valid ? width_m : 0.0f},
            {"width_metric_valid", width_metric_valid},
            {"end_cap_valid", end_cap_intersections},
            {"full_quad_valid", full_quad},
            {"length_px", cap_spacing_px},
            {"length_m", length_metric_valid ? length_m : 0.0f},
            {"length_metric_valid", length_metric_valid},
            {"scalar_floor_spacing_usable", scalar_spacing_usable},
            {"first_end_cap_width_px", cap_a_width_px},
            {"opposite_end_cap_width_px", full_quad ? cap_b_width_px : 0.0f},
            {"long_edge_a_length_px", len_a},
            {"long_edge_b_length_px", len_b},
            {"first_end_cap_length_px", cap_len},
            {"opposite_end_cap_length_px", has_cap_b ? line_length(cap_b) : 0.0f},
            {"long_parallel_error_rad", long_parallel_error},
            {"first_end_cap_perpendicular_error_rad", cap_perp_error},
            {"end_cap_parallel_error_rad", full_quad ? cap_parallel_error : 0.0f},
            {"orientation_rad", orientation},
            {"end_cap_orientation_rad", cap_orientation},
            {"reference_rho_px", std::min(rho_a, rho_b)},
            {"projective_perspective_observed", projective_perspective_observed},
            {"cap_width_ratio", full_quad ? cap_width_ratio : 0.0f},
            {"long_edge_length_ratio", long_edge_length_ratio},
            {"long_edge_vanishing_point_valid", long_vp_valid},
            {"long_edge_vanishing_point_px", long_vp_valid ? point_json(long_vp) : nlohmann::json::array()},
            {"end_cap_vanishing_point_valid", cap_vp_valid},
            {"end_cap_vanishing_point_px", cap_vp_valid ? point_json(cap_vp) : nlohmann::json::array()}
        };
        if (end_cap_intersections) {
            plank["end_cap_corners"] = nlohmann::json::array({point_json(corner_aa), point_json(corner_ba)});
        }
        if (full_quad) {
            plank["corners"] = nlohmann::json::array({
                point_json(corner_aa), point_json(corner_ba), point_json(corner_bb), point_json(corner_ab)
            });
        }

        const bool metric_ready = g.metric_scale_confidence > 0.0f;
        nlohmann::json floor_geometry_json = FloorGeometryToJson(g);
        floor_geometry_json["scalar_floor_spacing_usable"] = scalar_spacing_usable;
        floor_geometry_json["manual_plank"] = plank;
        return {
            {"ok", true},
            {"status", metric_ready
                ? "manual plank quad captured with projective metric geometry"
                : "manual plank outline captured; metric scale remains optional/unknown"},
            {"reason", g.reason},
            {"source", g.source},
            {"candidate_count", static_cast<int>(std::min<std::size_t>(manual_lines.size(), 4))},
            {"image_width", image_w},
            {"image_height", image_h},
            {"spacing_px", width_px},
            {"spacing_m", width_metric_valid ? width_m : 0.0f},
            {"metric_scale_ready", metric_ready},
            {"scalar_floor_spacing_usable", scalar_spacing_usable},
            {"manual_plank", plank},
            {"floor_geometry", floor_geometry_json},
            {"detection_debug", FloorGeometryDetectionDebugToJson(debug)}
        };
    }

    bt::CalibrationBundle LoadCurrentCalibrationForAlignment() {
        bt::AppConfig cfg_snapshot;
        {
            std::scoped_lock lock(mutex_);
            cfg_snapshot = config_;
        }
        std::error_code ec;
        const bool exists = std::filesystem::exists(cfg_snapshot.tracking.calibration_path, ec);
        if (ec || !exists) {
            return bt::CalibrationBundle{};
        }
        const auto loaded = bt::LoadCalibration(cfg_snapshot.tracking.calibration_path);
        return loaded.ok() ? loaded.value() : bt::CalibrationBundle{};
    }

    bt::UnifiedBodyState CurrentBodyStateForAlignment() {
        return runtime_state_.Snapshot().debug.tracking.body_state;
    }

    static double CommandTimestampSeconds() {
        const auto now = bt::NowQpc();
        return static_cast<double>(now.ticks) / 1000000000.0;
    }

    nlohmann::json SteamVrAlignmentReply(const std::string& text, bool ok, const bt::CalibrationBundle& calibration, bool body_state_stable) {
        bt::AppConfig cfg_snapshot;
        {
            std::scoped_lock lock(mutex_);
            cfg_snapshot = config_;
        }
        const auto status = alignment_manager_.Status(cfg_snapshot.osc, calibration, body_state_stable);
        return {
            {"ok", ok},
            {"status", text},
            {"steamvr_alignment", bt::AlignmentStatusToJson(status)},
            {"tracker_space", TrackerSpaceStateToJson(cfg_snapshot, &status)}
        };
    }

    nlohmann::json PersistOscConfig(const bt::OscConfig& osc, const char* success_message) {
        nlohmann::json j = nlohmann::json::object();
        {
            std::ifstream in(config_path_);
            if (in) {
                try {
                    in >> j;
                } catch (const std::exception& e) {
                    return {{"ok", false}, {"status", std::string("config parse failed before OSC save: ") + e.what()}};
                }
            }
        }
        if (!j.is_object()) {
            j = nlohmann::json::object();
        }
        EnsureObject(j, "osc");
        auto& o = j["osc"];
        o["enabled"] = osc.enabled;
        o["target_address"] = osc.target_address;
        o["target_port"] = osc.target_port;
        o["send_rotations"] = osc.send_rotations;
        o["min_confidence"] = osc.min_confidence;
        o["pelvis_tracker_index"] = osc.pelvis_tracker_index;
        o["left_foot_tracker_index"] = osc.left_foot_tracker_index;
        o["right_foot_tracker_index"] = osc.right_foot_tracker_index;
        o["chest_tracker_index"] = osc.chest_tracker_index;
        o["left_elbow_tracker_index"] = osc.left_elbow_tracker_index;
        o["right_elbow_tracker_index"] = osc.right_elbow_tracker_index;
        o["left_knee_tracker_index"] = osc.left_knee_tracker_index;
        o["right_knee_tracker_index"] = osc.right_knee_tracker_index;
        o["tracker_space_transform_valid"] = osc.tracker_space_transform_valid;
        o["tracker_space_position_offset"] = Vec3ToJson(osc.tracker_space_position_offset);
        o["tracker_space_rotation"] = QuatToJson(osc.tracker_space_rotation);
        o["tracker_space_scale"] = osc.tracker_space_scale;
        nlohmann::json role_offsets = nlohmann::json::array();
        for (const auto& offset : osc.tracker_space_role_offsets) {
            role_offsets.push_back(Vec3ToJson(offset));
        }
        o["tracker_space_role_offsets"] = role_offsets;
        o["tracker_space_source"] = osc.tracker_space_source;
        o["manual_tracker_space_transform_valid"] = osc.manual_tracker_space_transform_valid;
        o["manual_tracker_space_position_offset"] = Vec3ToJson(osc.manual_tracker_space_position_offset);
        o["manual_tracker_space_rotation"] = QuatToJson(osc.manual_tracker_space_rotation);
        o["manual_tracker_space_scale"] = osc.manual_tracker_space_scale;
        nlohmann::json manual_role_offsets = nlohmann::json::array();
        for (const auto& offset : osc.manual_tracker_space_role_offsets) {
            manual_role_offsets.push_back(Vec3ToJson(offset));
        }
        o["manual_tracker_space_role_offsets"] = manual_role_offsets;
        o["manual_tracker_space_source"] = osc.manual_tracker_space_source;
        o["steamvr_alignment_status"] = osc.steamvr_alignment_status;
        o["steamvr_alignment_reason"] = osc.steamvr_alignment_reason;
        o["steamvr_alignment_confidence"] = osc.steamvr_alignment_confidence;
        o["steamvr_alignment_residual_m"] = osc.steamvr_alignment_residual_m;
        o["steamvr_floor_residual_m"] = osc.steamvr_floor_residual_m;
        o["steamvr_yaw_offset_rad"] = osc.steamvr_yaw_offset_rad;
        o["steamvr_scale_ratio"] = osc.steamvr_scale_ratio;
        o["steamvr_alignment_body_signature"] = osc.steamvr_alignment_body_signature;
        o["steamvr_alignment_floor_signature"] = osc.steamvr_alignment_floor_signature;

        const auto parent = config_path_.parent_path();
        if (!parent.empty()) {
            std::error_code mkdir_ec;
            std::filesystem::create_directories(parent, mkdir_ec);
            if (mkdir_ec) {
                return {{"ok", false}, {"status", std::string("OSC config not saved; failed to create config directory: ") + mkdir_ec.message()}};
            }
        }
        std::filesystem::path temp_config_path = config_path_;
        temp_config_path += ".tmp";
        {
            std::ofstream out(temp_config_path);
            if (!out) {
                return {{"ok", false}, {"status", "failed to open temporary config for OSC save"}};
            }
            out << j.dump(2) << '\n';
            if (!out) {
                std::error_code remove_ec;
                std::filesystem::remove(temp_config_path, remove_ec);
                return {{"ok", false}, {"status", "OSC config not saved; failed to finish writing temporary config"}};
            }
        }
        const auto loaded = bt::LoadConfig(temp_config_path);
        if (!loaded.ok()) {
            std::error_code remove_ec;
            std::filesystem::remove(temp_config_path, remove_ec);
            return {{"ok", false}, {"status", std::string("OSC config not saved; proposed config failed validation: ") + loaded.status().message}};
        }
        std::error_code remove_ec;
        std::filesystem::remove(temp_config_path, remove_ec);
        const auto saved_config = WriteJsonFileAtomically(config_path_, j, "OSC config");
        if (!saved_config.ok()) {
            return {{"ok", false}, {"status", std::string("validated OSC config could not replace live config: ") + saved_config.message}};
        }
        {
            std::scoped_lock lock(mutex_);
            config_ = loaded.value();
            runtime_state_.SetConfig(config_);
            runtime_error_.clear();
        }
        return {{"ok", true}, {"status", success_message}};
    }

    nlohmann::json SteamVrAlignmentStartCommand() {
        alignment_manager_.StartSession();
        const auto calibration = LoadCurrentCalibrationForAlignment();
        const bool body_stable = CurrentBodyStateForAlignment().valid;
        bt::AppConfig cfg_snapshot;
        {
            std::scoped_lock lock(mutex_);
            cfg_snapshot = config_;
        }
        const auto status = alignment_manager_.Status(cfg_snapshot.osc, calibration, body_stable);
        const bool ok = status.session_active;
        const std::string text = ok
            ? "SteamVR alignment session started"
            : std::string("SteamVR alignment session not started: ") + (status.reason.empty() ? status.provider_reason : status.reason);
        return {
            {"ok", ok},
            {"status", text},
            {"steamvr_alignment", bt::AlignmentStatusToJson(status)},
            {"tracker_space", TrackerSpaceStateToJson(cfg_snapshot, &status)}
        };
    }

    nlohmann::json SteamVrAlignmentRecordCommand(const nlohmann::json& payload) {
        const auto landmark = ParseSteamVrAlignmentLandmark(payload.value("landmark", std::string("pelvis")));
        const auto controller = ParseSteamVrControllerRole(payload.value("controller", std::string("unknown")));
        const auto body_state = CurrentBodyStateForAlignment();
        const auto calibration = LoadCurrentCalibrationForAlignment();
        auto status = alignment_manager_.RecordSample(landmark, controller, body_state, calibration, CommandTimestampSeconds());
        bool accepted = false;
        for (const auto& sample : status.samples) {
            if (sample.landmark == landmark) {
                accepted = sample.accepted;
                break;
            }
        }
        return SteamVrAlignmentReply(accepted ? "SteamVR alignment sample accepted" : "SteamVR alignment sample rejected", accepted, calibration, body_state.valid);
    }

    nlohmann::json SteamVrAlignmentRedoCommand(const nlohmann::json& payload) {
        const auto landmark = ParseSteamVrAlignmentLandmark(payload.value("landmark", std::string("pelvis")));
        alignment_manager_.RedoSample(landmark);
        const auto calibration = LoadCurrentCalibrationForAlignment();
        const bool body_stable = CurrentBodyStateForAlignment().valid;
        return SteamVrAlignmentReply("SteamVR alignment sample cleared", true, calibration, body_stable);
    }

    nlohmann::json SteamVrAlignmentFinishCommand() {
        const auto body_state = CurrentBodyStateForAlignment();
        const auto calibration = LoadCurrentCalibrationForAlignment();
        bt::OscConfig proposed_osc;
        {
            std::scoped_lock lock(mutex_);
            proposed_osc = config_.osc;
        }
        const auto status = alignment_manager_.FinishSession(body_state, calibration, proposed_osc, CommandTimestampSeconds());
        const bool solved = status.state == "valid" || status.state == "weak";
        const auto saved = PersistOscConfig(proposed_osc, solved ? "SteamVR alignment solved and saved" : "SteamVR alignment solve failed; previous tracker-space transform preserved");
        if (saved.value("ok", false)) {
            return SteamVrAlignmentReply(saved.value("status", std::string("SteamVR alignment saved")), solved, calibration, body_state.valid);
        }
        return {
            {"ok", false},
            {"status", saved.value("status", "SteamVR alignment save failed")},
            {"steamvr_alignment", bt::AlignmentStatusToJson(status)}
        };
    }

    nlohmann::json SteamVrAlignmentClearCommand() {
        bt::OscConfig proposed_osc;
        {
            std::scoped_lock lock(mutex_);
            proposed_osc = config_.osc;
        }
        alignment_manager_.ClearAlignment(proposed_osc);
        const auto saved = PersistOscConfig(proposed_osc, proposed_osc.enabled ? "SteamVR alignment cleared" : "SteamVR alignment cleared; OSC disabled because tracker-space transform is now invalid");
        const auto calibration = LoadCurrentCalibrationForAlignment();
        const bool body_stable = CurrentBodyStateForAlignment().valid;
        if (saved.value("ok", false)) {
            return SteamVrAlignmentReply("SteamVR alignment cleared", true, calibration, body_stable);
        }
        return saved;
    }


    nlohmann::json SaveConfigFromUi(const nlohmann::json& payload) {
        bt::AppConfig cfg_snapshot;
        std::vector<CameraProbe> cameras_snapshot;
        bool runtime_running_snapshot = false;
        {
            std::scoped_lock lock(mutex_);
            cfg_snapshot = config_;
            cameras_snapshot = cameras_;
            runtime_running_snapshot = runtime_running_;
        }

        const std::string tracking_mode = payload.value("tracking_mode", bt::ToString(cfg_snapshot.tracking.mode));
        if (tracking_mode != "stereo" && tracking_mode != "monocular") {
            return {{"status", "tracking mode must be stereo or monocular"}, {"ok", false}};
        }
        const bool stereo_mode = tracking_mode == "stereo";

        const int camera_a = payload.value("camera_a", cfg_snapshot.camera_a.device_index);
        const int camera_b = payload.value("camera_b", cfg_snapshot.camera_b.device_index);
        if (stereo_mode && camera_a == camera_b) {
            return {{"status", "camera A and B must use different indices in stereo mode"}, {"ok", false}};
        }
        const bt::CameraConfig auto_a = CameraConfigForProbe(cameras_snapshot, camera_a, cfg_snapshot.camera_a);
        const bt::CameraConfig auto_b = stereo_mode
            ? CameraConfigForProbe(cameras_snapshot, camera_b, cfg_snapshot.camera_b)
            : cfg_snapshot.camera_b;

        nlohmann::json j = nlohmann::json::object();
        {
            std::ifstream in(config_path_);
            if (in) {
                try {
                    in >> j;
                } catch (const std::exception& e) {
                    return {{"status", std::string("config parse failed: ") + e.what()}, {"ok", false}};
                }
            }
        }
        if (!j.is_object()) {
            j = nlohmann::json::object();
        }
        auto floor_family_has_metric_image_space = [](const nlohmann::json& floor_geometry, const char* key) {
            if (!floor_geometry.is_object() || !floor_geometry.contains(key) || !floor_geometry[key].is_object()) {
                return false;
            }
            const auto& family = floor_geometry[key];
            return family.value("metric_spacing_valid", false);
        };
        auto floor_geometry_uses_raw_image_space = [&](const nlohmann::json& floor_geometry) {
            if (!floor_geometry.is_object()) {
                return false;
            }
            const bool distortion_valid = floor_geometry.contains("distortion") &&
                floor_geometry["distortion"].is_object() &&
                floor_geometry["distortion"].value("valid", false);
            return floor_geometry.value("homography_valid", false) ||
                floor_family_has_metric_image_space(floor_geometry, "family_a") ||
                floor_family_has_metric_image_space(floor_geometry, "family_b") ||
                distortion_valid ||
                floor_geometry.value("camera_orientation_valid", false);
        };
        if (payload.contains("floor_geometry_auto") && payload["floor_geometry_auto"].is_object()) {
            const auto& floor_geometry_preflight = payload["floor_geometry_auto"];
            (void)floor_geometry_uses_raw_image_space;
            (void)floor_geometry_preflight;
            // Save the user's/draft geometry as data. Runtime sanitizes stale raw
            // image-space capabilities; the save path must not veto unrelated config.
        }

        EnsureObject(j, "camera_a");
        EnsureObject(j, "camera_b");
        EnsureObject(j, "app");
        EnsureObject(j, "tracking");
        EnsureObject(j, "inference");
        EnsureObject(j, "debug");
        EnsureObject(j, "osc");
        EnsureObject(j, "steamvr_tracker_bridge");
        EnsureObject(j, "hmd");

        const std::string legacy_camera_source = payload.value("camera_source", std::string());
        const std::string camera_a_source = payload.value(
            "camera_a_source",
            legacy_camera_source.empty() ? cfg_snapshot.camera_a.source : legacy_camera_source);
        const std::string camera_b_source = payload.value(
            "camera_b_source",
            legacy_camera_source.empty() ? cfg_snapshot.camera_b.source : legacy_camera_source);
        if (camera_a_source != "opencv" && camera_a_source != "network_mjpeg") {
            return {{"status", "camera_a_source must be opencv or network_mjpeg"}, {"ok", false}};
        }
        if (camera_b_source != "opencv" && camera_b_source != "network_mjpeg") {
            return {{"status", "camera_b_source must be opencv or network_mjpeg"}, {"ok", false}};
        }
        j["camera_a"]["source"] = camera_a_source;
        j["camera_b"]["source"] = camera_b_source;
        j["camera_a"]["device_index"] = camera_a;
        j["camera_b"]["device_index"] = camera_b;
        j["camera_a"]["width"] = auto_a.width;
        j["camera_a"]["height"] = auto_a.height;
        j["camera_a"]["fps"] = auto_a.fps;
        j["camera_b"]["width"] = auto_b.width;
        j["camera_b"]["height"] = auto_b.height;
        j["camera_b"]["fps"] = auto_b.fps;
        j["camera_a"]["initial_roi_enabled"] = payload.value("camera_a_roi_enabled", cfg_snapshot.camera_a.initial_roi_enabled);
        j["camera_a"]["initial_roi_normalized"] = payload.value("camera_a_roi_normalized", cfg_snapshot.camera_a.initial_roi_normalized);
        if (payload.contains("camera_a_roi") && payload["camera_a_roi"].is_array() && payload["camera_a_roi"].size() == 4) {
            j["camera_a"]["initial_roi"] = payload["camera_a_roi"];
        } else {
            j["camera_a"]["initial_roi"] = {
                cfg_snapshot.camera_a.initial_roi.x,
                cfg_snapshot.camera_a.initial_roi.y,
                cfg_snapshot.camera_a.initial_roi.width,
                cfg_snapshot.camera_a.initial_roi.height
            };
        }

        j["app"]["log_file"] = payload.value("log_file", cfg_snapshot.app.log_file.string());
        j["app"]["recording_dir"] = payload.value("recording_dir", cfg_snapshot.app.recording_dir.string());

        const std::string model_path = payload.value("model_path", cfg_snapshot.tracking.model_path.string());
        const std::string calibration_path = payload.value("calibration_path", cfg_snapshot.tracking.calibration_path.string());
        j["tracking"]["mode"] = tracking_mode;
        j["tracking"]["model_path"] = model_path;
        j["inference"].erase("model_path");
        j["inference"]["device"] = payload.value("inference_device", cfg_snapshot.inference.device);
        j["tracking"]["calibration_path"] = calibration_path;
        // Keep tracking.calibration_path canonical. Remove the old root mirror so
        // future hand-edits cannot silently override the tracking value.
        j.erase("calibration_path");
        j["tracking"]["latest_frame_skew_tolerance_ms"] = payload.value(
            "latest_frame_skew_tolerance_ms",
            cfg_snapshot.tracking.latest_frame_skew_tolerance_ms);
        j["tracking"]["max_frame_skew_ms"] = payload.value("max_frame_skew_ms", cfg_snapshot.tracking.max_frame_skew_ms);
        j["tracking"]["stale_frame_timeout_ms"] = payload.value("stale_frame_timeout_ms", cfg_snapshot.tracking.stale_frame_timeout_ms);
        j["tracking"]["min_triangulated_seed_count"] = payload.value("min_triangulated_seed_count", cfg_snapshot.tracking.min_triangulated_seed_count);
        j["tracking"]["max_mean_reprojection_error_px"] = payload.value("max_mean_reprojection_error_px", cfg_snapshot.tracking.max_mean_reprojection_error_px);
        j["tracking"]["stereo_monocular_fallback_enabled"] = payload.value("stereo_monocular_fallback_enabled", cfg_snapshot.tracking.stereo_monocular_fallback_enabled);
        j["tracking"]["use_legacy_solver"] = payload.value("use_legacy_solver", cfg_snapshot.tracking.use_legacy_solver);
        j["tracking"]["enable_replay_recording"] = payload.value("enable_replay_recording", cfg_snapshot.tracking.enable_replay_recording);
        if (!j["tracking"].contains("room_depth_map") || !j["tracking"]["room_depth_map"].is_object()) {
            j["tracking"]["room_depth_map"] = nlohmann::json::object();
        }
        auto& room_depth_map = j["tracking"]["room_depth_map"];
        const auto& current_room_map = cfg_snapshot.tracking.room_depth_map;
        room_depth_map["enabled"] = payload.value("room_depth_map_enabled", current_room_map.enabled);
        room_depth_map["collect_only"] = payload.value("room_depth_map_collect_only", current_room_map.collect_only);
        room_depth_map["min_accepted_frames_before_active"] = payload.value(
            "room_depth_map_min_accepted_frames_before_active",
            current_room_map.min_accepted_frames_before_active);
        room_depth_map["resolution_width"] = payload.value("room_depth_map_resolution_width", current_room_map.resolution_width);
        room_depth_map["resolution_height"] = payload.value("room_depth_map_resolution_height", current_room_map.resolution_height);
        room_depth_map["min_samples_per_cell"] = payload.value("room_depth_map_min_samples_per_cell", current_room_map.min_samples_per_cell);
        room_depth_map["max_cell_variance_m2"] = payload.value("room_depth_map_max_cell_variance_m2", current_room_map.max_cell_variance_m2);
        room_depth_map["body_mask_dilation_px"] = payload.value("room_depth_map_body_mask_dilation_px", current_room_map.body_mask_dilation_px);
        room_depth_map["update_only_when_anchor_quality_good"] = payload.value("room_depth_map_update_only_when_anchor_quality_good", current_room_map.update_only_when_anchor_quality_good);
        room_depth_map["save_path"] = payload.value("room_depth_map_save_path", current_room_map.save_path.string());
        room_depth_map["load_existing"] = payload.value("room_depth_map_load_existing", current_room_map.load_existing);
        room_depth_map["save_interval_seconds"] = payload.value("room_depth_map_save_interval_seconds", current_room_map.save_interval_seconds);
        if (!j["tracking"].contains("body_calibration") || !j["tracking"]["body_calibration"].is_object()) {
            j["tracking"]["body_calibration"] = nlohmann::json::object();
        }
        auto& body_calibration = j["tracking"]["body_calibration"];
        const auto& current_body_cal = cfg_snapshot.tracking.body_calibration;
        body_calibration["enabled"] = payload.value("body_calibration_enabled", current_body_cal.enabled);
        body_calibration["auto_persist"] = payload.value("body_calibration_auto_persist", current_body_cal.auto_persist);
        body_calibration["required_seconds"] = payload.value("body_calibration_required_seconds", current_body_cal.required_seconds);
        body_calibration["min_overall_confidence"] = payload.value("body_calibration_min_overall_confidence", current_body_cal.min_overall_confidence);
        body_calibration["max_segment_cv"] = payload.value("body_calibration_max_segment_cv", current_body_cal.max_segment_cv);
        if (!j["tracking"].contains("monocular") || !j["tracking"]["monocular"].is_object()) {
            j["tracking"]["monocular"] = nlohmann::json::object();
        }
        auto& monocular = j["tracking"]["monocular"];
        const auto& current_monocular = cfg_snapshot.tracking.monocular;
        monocular["image_width"] = payload.value("monocular_image_width", current_monocular.image_width);
        monocular["image_height"] = payload.value("monocular_image_height", current_monocular.image_height);
        monocular["horizontal_fov_deg"] = payload.value("monocular_horizontal_fov_deg", current_monocular.horizontal_fov_deg);
        monocular["user_height_m"] = payload.value("monocular_user_height_m", current_monocular.user_height_m);
        monocular["camera_height_m"] = payload.value("monocular_camera_height_m", current_monocular.camera_height_m);
        monocular["default_depth_m"] = payload.value("monocular_default_depth_m", current_monocular.default_depth_m);
        monocular["depth_confidence_scale"] = payload.value("monocular_depth_confidence_scale", current_monocular.depth_confidence_scale);
        monocular["min_keypoint_confidence"] = payload.value("monocular_min_keypoint_confidence", current_monocular.min_keypoint_confidence);
        monocular["min_seed_count"] = payload.value("monocular_min_seed_count", current_monocular.min_seed_count);
        monocular["floor_scale_assist_enabled"] = payload.value("monocular_floor_scale_assist_enabled", current_monocular.floor_scale_assist_enabled);
        monocular["floor_geometry_calibration_enabled"] = payload.value("monocular_floor_geometry_calibration_enabled", current_monocular.floor_geometry_calibration_enabled);
        monocular["floor_geometry_type"] = payload.value("monocular_floor_geometry_type", current_monocular.floor_geometry_type);
        monocular["floor_depth_line_spacing_m"] = payload.value("monocular_floor_depth_line_spacing_m", current_monocular.floor_depth_line_spacing_m);
        monocular["floor_depth_line_spacing_px"] = payload.value("monocular_floor_depth_line_spacing_px", current_monocular.floor_depth_line_spacing_px);
        monocular["floor_depth_reference_y_px"] = payload.value("monocular_floor_depth_reference_y_px", current_monocular.floor_depth_reference_y_px);
        monocular["floor_depth_reference_m"] = payload.value("monocular_floor_depth_reference_m", current_monocular.floor_depth_reference_m);
        monocular["floor_depth_confidence"] = payload.value("monocular_floor_depth_confidence", current_monocular.floor_depth_confidence);
        monocular["floor_second_axis_spacing_m"] = payload.value("monocular_floor_second_axis_spacing_m", current_monocular.floor_second_axis_spacing_m);
        monocular["floor_geometry_confidence"] = payload.value("monocular_floor_geometry_confidence", current_monocular.floor_geometry_confidence);
        monocular["floor_projective_homography_enabled"] = payload.value("monocular_floor_projective_homography_enabled", current_monocular.floor_projective_homography_enabled);
        monocular["floor_from_image"] = Mat3ToJson(current_monocular.floor_from_image);
        monocular["image_from_floor"] = Mat3ToJson(current_monocular.image_from_floor);
        monocular["floor_projective_confidence"] = payload.value("monocular_floor_projective_confidence", current_monocular.floor_projective_confidence);
        monocular["floor_distortion_correction_enabled"] = payload.value("monocular_floor_distortion_correction_enabled", current_monocular.floor_distortion_correction_enabled);
        monocular["floor_distortion_confidence"] = payload.value("monocular_floor_distortion_confidence", current_monocular.floor_distortion_confidence);
        monocular["floor_radial_k1"] = payload.value("monocular_floor_radial_k1", current_monocular.floor_radial_k1);
        monocular["floor_radial_k2"] = payload.value("monocular_floor_radial_k2", current_monocular.floor_radial_k2);
        monocular["floor_tangential_p1"] = payload.value("monocular_floor_tangential_p1", current_monocular.floor_tangential_p1);
        monocular["floor_tangential_p2"] = payload.value("monocular_floor_tangential_p2", current_monocular.floor_tangential_p2);
        monocular["floor_camera_orientation_enabled"] = payload.value("monocular_floor_camera_orientation_enabled", current_monocular.floor_camera_orientation_enabled);
        monocular["floor_camera_pitch_rad"] = payload.value("monocular_floor_camera_pitch_rad", current_monocular.floor_camera_pitch_rad);
        monocular["floor_camera_roll_rad"] = payload.value("monocular_floor_camera_roll_rad", current_monocular.floor_camera_roll_rad);
        monocular["floor_camera_orientation_confidence"] = payload.value("monocular_floor_camera_orientation_confidence", current_monocular.floor_camera_orientation_confidence);

        if (!j["tracking"].contains("motion_consistency") || !j["tracking"]["motion_consistency"].is_object()) {
            j["tracking"]["motion_consistency"] = nlohmann::json::object();
        }
        auto& motion = j["tracking"]["motion_consistency"];
        const auto& current_motion = cfg_snapshot.tracking.motion_consistency;
        motion["enabled"] = payload.value("motion_consistency_enabled", current_motion.enabled);
        motion["confirm_frames"] = payload.value("motion_confirm_frames", current_motion.confirm_frames);
        motion["min_motion_m"] = payload.value("motion_min_motion_m", current_motion.min_motion_m);
        motion["stationary_deadzone_m"] = payload.value("motion_stationary_deadzone_m", current_motion.stationary_deadzone_m);
        motion["max_direction_deviation_deg"] = payload.value("motion_max_direction_deviation_deg", current_motion.max_direction_deviation_deg);
        motion["max_lateral_deviation_ratio"] = payload.value("motion_max_lateral_deviation_ratio", current_motion.max_lateral_deviation_ratio);
        motion["max_speed_change_ratio"] = payload.value("motion_max_speed_change_ratio", current_motion.max_speed_change_ratio);
        motion["reject_confidence_decay_per_second"] = payload.value("motion_reject_confidence_decay_per_second", current_motion.reject_confidence_decay_per_second);
        motion["planted_foot_max_drift_m"] = payload.value("planted_foot_max_drift_m", current_motion.planted_foot_max_drift_m);
        motion["planted_foot_release_confirm_frames"] = payload.value("planted_foot_release_confirm_frames", current_motion.planted_foot_release_confirm_frames);
        motion["contact_root_correction_gain"] = payload.value("contact_root_correction_gain", current_motion.contact_root_correction_gain);
        motion["contact_root_max_correction_m"] = payload.value("contact_root_max_correction_m", current_motion.contact_root_max_correction_m);
        motion["contact_root_max_residual_m"] = payload.value("contact_root_max_residual_m", current_motion.contact_root_max_residual_m);
        motion["contact_root_max_disagreement_m"] = payload.value("contact_root_max_disagreement_m", current_motion.contact_root_max_disagreement_m);
        motion["contact_root_min_alignment"] = payload.value("contact_root_min_alignment", current_motion.contact_root_min_alignment);
        motion["contact_root_min_support_confidence"] = payload.value("contact_root_min_support_confidence", current_motion.contact_root_min_support_confidence);
        motion["one_euro_enabled"] = payload.value("motion_one_euro_enabled", current_motion.one_euro_enabled);
        motion["one_euro_min_cutoff_hz"] = payload.value("motion_one_euro_min_cutoff_hz", current_motion.one_euro_min_cutoff_hz);
        motion["one_euro_beta"] = payload.value("motion_one_euro_beta", current_motion.one_euro_beta);
        motion["one_euro_d_cutoff_hz"] = payload.value("motion_one_euro_d_cutoff_hz", current_motion.one_euro_d_cutoff_hz);

        j["debug"]["replay_log_path"] = payload.value("replay_log_path", cfg_snapshot.debug.replay_log_path.string());

        j["osc"]["enabled"] = payload.value("osc_enabled", cfg_snapshot.osc.enabled);
        j["osc"]["target_address"] = payload.value("osc_host", cfg_snapshot.osc.target_address);
        j["osc"]["target_port"] = payload.value("osc_port", cfg_snapshot.osc.target_port);
        j["osc"]["send_rotations"] = payload.value("osc_send_rotations", cfg_snapshot.osc.send_rotations);
        j["osc"]["min_confidence"] = payload.value("osc_min_confidence", cfg_snapshot.osc.min_confidence);
        j["osc"]["pelvis_tracker_index"] = payload.value("pelvis_tracker_index", cfg_snapshot.osc.pelvis_tracker_index);
        j["osc"]["left_foot_tracker_index"] = payload.value("left_foot_tracker_index", cfg_snapshot.osc.left_foot_tracker_index);
        j["osc"]["right_foot_tracker_index"] = payload.value("right_foot_tracker_index", cfg_snapshot.osc.right_foot_tracker_index);
        j["osc"]["chest_tracker_index"] = payload.value("chest_tracker_index", cfg_snapshot.osc.chest_tracker_index);
        j["osc"]["left_elbow_tracker_index"] = payload.value("left_elbow_tracker_index", cfg_snapshot.osc.left_elbow_tracker_index);
        j["osc"]["right_elbow_tracker_index"] = payload.value("right_elbow_tracker_index", cfg_snapshot.osc.right_elbow_tracker_index);
        j["osc"]["left_knee_tracker_index"] = payload.value("left_knee_tracker_index", cfg_snapshot.osc.left_knee_tracker_index);
        j["osc"]["right_knee_tracker_index"] = payload.value("right_knee_tracker_index", cfg_snapshot.osc.right_knee_tracker_index);
        j["steamvr_tracker_bridge"]["enabled"] = payload.value("steamvr_tracker_bridge_enabled", cfg_snapshot.steamvr_tracker_bridge.enabled);
        j["steamvr_tracker_bridge"]["target_address"] = payload.value("steamvr_tracker_bridge_host", cfg_snapshot.steamvr_tracker_bridge.target_address);
        j["steamvr_tracker_bridge"]["target_port"] = payload.value("steamvr_tracker_bridge_port", cfg_snapshot.steamvr_tracker_bridge.target_port);
        j["steamvr_tracker_bridge"]["min_confidence"] = payload.value("steamvr_tracker_bridge_min_confidence", cfg_snapshot.steamvr_tracker_bridge.min_confidence);
        j["steamvr_tracker_bridge"]["send_chest"] = payload.value("steamvr_tracker_bridge_send_chest", cfg_snapshot.steamvr_tracker_bridge.send_chest);
        j["steamvr_tracker_bridge"]["send_elbows"] = payload.value("steamvr_tracker_bridge_send_elbows", cfg_snapshot.steamvr_tracker_bridge.send_elbows);
        j["steamvr_tracker_bridge"]["send_knees"] = payload.value("steamvr_tracker_bridge_send_knees", cfg_snapshot.steamvr_tracker_bridge.send_knees);
        bt::OscConfig proposed_osc = cfg_snapshot.osc;
        proposed_osc.enabled = j["osc"]["enabled"].get<bool>();
        proposed_osc.target_address = j["osc"]["target_address"].get<std::string>();
        proposed_osc.target_port = j["osc"]["target_port"].get<int>();
        proposed_osc.send_rotations = j["osc"]["send_rotations"].get<bool>();
        proposed_osc.min_confidence = j["osc"]["min_confidence"].get<float>();
        proposed_osc.pelvis_tracker_index = j["osc"]["pelvis_tracker_index"].get<int>();
        proposed_osc.left_foot_tracker_index = j["osc"]["left_foot_tracker_index"].get<int>();
        proposed_osc.right_foot_tracker_index = j["osc"]["right_foot_tracker_index"].get<int>();
        proposed_osc.chest_tracker_index = j["osc"]["chest_tracker_index"].get<int>();
        proposed_osc.left_elbow_tracker_index = j["osc"]["left_elbow_tracker_index"].get<int>();
        proposed_osc.right_elbow_tracker_index = j["osc"]["right_elbow_tracker_index"].get<int>();
        proposed_osc.left_knee_tracker_index = j["osc"]["left_knee_tracker_index"].get<int>();
        proposed_osc.right_knee_tracker_index = j["osc"]["right_knee_tracker_index"].get<int>();
        proposed_osc.tracker_space_transform_valid = payload.value("tracker_space_transform_valid", cfg_snapshot.osc.tracker_space_transform_valid);
        proposed_osc.tracker_space_position_offset = bt::Vec3f{
            payload.value("tracker_space_offset_x", cfg_snapshot.osc.tracker_space_position_offset.x),
            payload.value("tracker_space_offset_y", cfg_snapshot.osc.tracker_space_position_offset.y),
            payload.value("tracker_space_offset_z", cfg_snapshot.osc.tracker_space_position_offset.z)
        };
        proposed_osc.tracker_space_rotation = bt::Quatf{
            payload.value("tracker_space_rotation_x", cfg_snapshot.osc.tracker_space_rotation.x),
            payload.value("tracker_space_rotation_y", cfg_snapshot.osc.tracker_space_rotation.y),
            payload.value("tracker_space_rotation_z", cfg_snapshot.osc.tracker_space_rotation.z),
            payload.value("tracker_space_rotation_w", cfg_snapshot.osc.tracker_space_rotation.w)
        };
        proposed_osc.tracker_space_scale = payload.value("tracker_space_scale", cfg_snapshot.osc.tracker_space_scale);
        if (TrackerSpaceEditedByUi(cfg_snapshot.osc, proposed_osc)) {
            proposed_osc.tracker_space_source = cfg_snapshot.hmd.mode == "json_file" ? "manual_json_file" : "manual";
            for (auto& offset : proposed_osc.tracker_space_role_offsets) {
                offset = bt::Vec3f{};
            }
            bt::StoreActiveTrackerSpaceAsManualFallback(proposed_osc);
            proposed_osc.steamvr_alignment_status = "idle";
            proposed_osc.steamvr_alignment_reason.clear();
            proposed_osc.steamvr_alignment_confidence = 0.0f;
            proposed_osc.steamvr_alignment_residual_m = 0.0f;
            proposed_osc.steamvr_floor_residual_m = 0.0f;
            proposed_osc.steamvr_yaw_offset_rad = 0.0f;
            proposed_osc.steamvr_scale_ratio = 1.0f;
            proposed_osc.steamvr_alignment_body_signature.clear();
            proposed_osc.steamvr_alignment_floor_signature.clear();
        }
        const std::string tracker_space_status = TrackerSpaceStatus(proposed_osc);
        if (tracker_space_status == "invalid") {
            return {
                {"status", TrackerSpaceValidationMessage(proposed_osc)},
                {"ok", false},
                {"tracker_space_status", tracker_space_status}
            };
        }
        std::string osc_save_warning;
        if (proposed_osc.enabled && tracker_space_status != "valid") {
            osc_save_warning = TrackerSpaceValidationMessage(proposed_osc) +
                "; OSC output will stay blocked until tracker space is calibrated";
        }
        j["osc"]["tracker_space_transform_valid"] = proposed_osc.tracker_space_transform_valid;
        j["osc"]["tracker_space_position_offset"] = {
            proposed_osc.tracker_space_position_offset.x,
            proposed_osc.tracker_space_position_offset.y,
            proposed_osc.tracker_space_position_offset.z
        };
        j["osc"]["tracker_space_rotation"] = {
            proposed_osc.tracker_space_rotation.x,
            proposed_osc.tracker_space_rotation.y,
            proposed_osc.tracker_space_rotation.z,
            proposed_osc.tracker_space_rotation.w
        };
        j["osc"]["tracker_space_scale"] = proposed_osc.tracker_space_scale;
        nlohmann::json role_offsets = nlohmann::json::array();
        for (const auto& offset : proposed_osc.tracker_space_role_offsets) {
            role_offsets.push_back(Vec3ToJson(offset));
        }
        j["osc"]["tracker_space_role_offsets"] = role_offsets;
        j["osc"]["tracker_space_source"] = proposed_osc.tracker_space_source;
        j["osc"]["manual_tracker_space_transform_valid"] = proposed_osc.manual_tracker_space_transform_valid;
        j["osc"]["manual_tracker_space_position_offset"] = Vec3ToJson(proposed_osc.manual_tracker_space_position_offset);
        j["osc"]["manual_tracker_space_rotation"] = QuatToJson(proposed_osc.manual_tracker_space_rotation);
        j["osc"]["manual_tracker_space_scale"] = proposed_osc.manual_tracker_space_scale;
        nlohmann::json manual_role_offsets = nlohmann::json::array();
        for (const auto& offset : proposed_osc.manual_tracker_space_role_offsets) {
            manual_role_offsets.push_back(Vec3ToJson(offset));
        }
        j["osc"]["manual_tracker_space_role_offsets"] = manual_role_offsets;
        j["osc"]["manual_tracker_space_source"] = proposed_osc.manual_tracker_space_source;
        j["osc"]["steamvr_alignment_status"] = proposed_osc.steamvr_alignment_status;
        j["osc"]["steamvr_alignment_reason"] = proposed_osc.steamvr_alignment_reason;
        j["osc"]["steamvr_alignment_confidence"] = proposed_osc.steamvr_alignment_confidence;
        j["osc"]["steamvr_alignment_residual_m"] = proposed_osc.steamvr_alignment_residual_m;
        j["osc"]["steamvr_floor_residual_m"] = proposed_osc.steamvr_floor_residual_m;
        j["osc"]["steamvr_yaw_offset_rad"] = proposed_osc.steamvr_yaw_offset_rad;
        j["osc"]["steamvr_scale_ratio"] = proposed_osc.steamvr_scale_ratio;
        j["osc"]["steamvr_alignment_body_signature"] = proposed_osc.steamvr_alignment_body_signature;
        j["osc"]["steamvr_alignment_floor_signature"] = proposed_osc.steamvr_alignment_floor_signature;

        j["hmd"]["mode"] = payload.value("hmd_mode", cfg_snapshot.hmd.mode);
        j["hmd"]["pose_json_path"] = payload.value("hmd_path", cfg_snapshot.hmd.pose_json_path.string());
        const auto config_parent = config_path_.parent_path();
        if (!config_parent.empty()) {
            std::error_code mkdir_ec;
            std::filesystem::create_directories(config_parent, mkdir_ec);
            if (mkdir_ec) {
                return {{"status", std::string("config not saved; failed to create config directory: ") + mkdir_ec.message()}, {"ok", false}};
            }
        }
        std::filesystem::path temp_config_path = config_path_;
        temp_config_path += ".tmp";
        {
            std::ofstream out(temp_config_path);
            if (!out) {
                return {{"status", "failed to open temporary config for save"}, {"ok", false}};
            }
            out << j.dump(2) << '\n';
            if (!out) {
                std::error_code remove_ec;
                std::filesystem::remove(temp_config_path, remove_ec);
                return {{"status", "config not saved; failed to finish writing temporary config"}, {"ok", false}};
            }
        }

        const bool clear_floor_geometry = payload.value("floor_geometry_clear", false);
        const bool has_floor_geometry_save = !clear_floor_geometry && payload.contains("floor_geometry_auto") && payload["floor_geometry_auto"].is_object();
        const bool has_floor_geometry_by_camera_save = !clear_floor_geometry && payload.contains("floor_geometry_by_camera") && payload["floor_geometry_by_camera"].is_object();
        const bool has_wall_rectangles_save = payload.contains("wall_rectangles_auto") && payload["wall_rectangles_auto"].is_array();
        const bool has_wall_rectangles_by_camera_save = payload.contains("wall_rectangles_by_camera") && payload["wall_rectangles_by_camera"].is_object();

        const auto loaded = bt::LoadConfig(temp_config_path);
        if (!loaded.ok()) {
            std::error_code remove_ec;
            std::filesystem::remove(temp_config_path, remove_ec);
            return {{"status", std::string("config not saved; proposed values failed validation: ") + loaded.status().message}, {"ok", false}};
        }
        bool restart_required_after_save = false;
        if (runtime_running_snapshot) {
            nlohmann::json before_hot = ConfigToJson(cfg_snapshot);
            nlohmann::json after_hot = ConfigToJson(loaded.value());
            before_hot.erase("osc");
            after_hot.erase("osc");
            before_hot.erase("steamvr_tracker_bridge");
            after_hot.erase("steamvr_tracker_bridge");
            restart_required_after_save = before_hot != after_hot || clear_floor_geometry ||
                has_floor_geometry_save || has_floor_geometry_by_camera_save ||
                has_wall_rectangles_save || has_wall_rectangles_by_camera_save;
        }

        std::filesystem::path temp_calibration_path;
        if (has_floor_geometry_save || has_floor_geometry_by_camera_save ||
            clear_floor_geometry || has_wall_rectangles_save || has_wall_rectangles_by_camera_save) {
            nlohmann::json floor_geometry = has_floor_geometry_save
                ? payload["floor_geometry_auto"]
                : nlohmann::json::object();
            if (has_floor_geometry_save && floor_geometry.is_object() && !floor_geometry.contains("source")) {
                floor_geometry["source"] = "backend_floor_geometry_calibrator";
            }
            if (has_floor_geometry_save && floor_geometry.is_object()) {
                // Persisted calibration is source-of-truth data, not runtime telemetry.
                // The solver/debug snapshot reports whether orientation/distortion were
                // actually consumed on a given frame.
                floor_geometry["camera_orientation_applied_to_runtime"] = false;
                if (floor_geometry.contains("distortion") && floor_geometry["distortion"].is_object()) {
                    floor_geometry["distortion"]["applied_to_runtime"] = false;
                }
            }
            nlohmann::json calibration_json = nlohmann::json::object();
            {
                std::ifstream calib_in(calibration_path);
                if (calib_in) {
                    try {
                        calib_in >> calibration_json;
                    } catch (const std::exception& e) {
                        std::error_code remove_ec;
                        std::filesystem::remove(temp_config_path, remove_ec);
                        return {{"status", std::string("config not saved; calibration parse failed while preparing floor geometry: ") + e.what()}, {"ok", false}};
                    }
                }
            }
            if (!calibration_json.is_object()) {
                calibration_json = nlohmann::json::object();
            }
            calibration_json["schema_version"] = calibration_json.value("schema_version", 1);
            calibration_json["world_handedness"] = calibration_json.value("world_handedness", "right_handed");
            calibration_json["world_up_axis"] = calibration_json.value("world_up_axis", "Y");
            if (clear_floor_geometry) {
                if (calibration_json.contains("floor_geometry") && calibration_json["floor_geometry"].is_object()) {
                    const auto& old_geometry = calibration_json["floor_geometry"];
                    if (old_geometry.contains("floor_plane") && calibration_json.contains("floor_plane") &&
                        calibration_json["floor_plane"] == old_geometry["floor_plane"]) {
                        calibration_json["floor_plane"] = {
                            {"normal", {0.0, 1.0, 0.0}},
                            {"distance", 0.0},
                            {"valid", false}
                        };
                        calibration_json["floor_plane_confidence"] = 0.0;
                    }
                }
                calibration_json.erase("floor_geometry");
            } else if (has_floor_geometry_save) {
                calibration_json["floor_geometry"] = floor_geometry;
                calibration_json["floor_geometry_by_camera"]["camera_a"] = floor_geometry;
                if (floor_geometry.contains("floor_plane") && floor_geometry["floor_plane"].is_object()) {
                    calibration_json["floor_plane"] = floor_geometry["floor_plane"];
                    calibration_json["floor_plane_confidence"] = floor_geometry.value("floor_plane_confidence", calibration_json.value("floor_plane_confidence", 0.0));
                }
            }
            if (has_floor_geometry_by_camera_save) {
                nlohmann::json by_camera = nlohmann::json::object();
                for (const auto& camera_key : {"camera_a", "camera_b"}) {
                    if (payload["floor_geometry_by_camera"].contains(camera_key) &&
                        payload["floor_geometry_by_camera"][camera_key].is_object()) {
                        auto geometry = payload["floor_geometry_by_camera"][camera_key];
                        if (!geometry.contains("source")) {
                            geometry["source"] = "backend_floor_geometry_calibrator";
                        }
                        geometry["camera_orientation_applied_to_runtime"] = false;
                        if (geometry.contains("distortion") && geometry["distortion"].is_object()) {
                            geometry["distortion"]["applied_to_runtime"] = false;
                        }
                        by_camera[camera_key] = std::move(geometry);
                    }
                }
                calibration_json["floor_geometry_by_camera"] = std::move(by_camera);
                if (calibration_json["floor_geometry_by_camera"].contains("camera_a")) {
                    calibration_json["floor_geometry"] = calibration_json["floor_geometry_by_camera"]["camera_a"];
                    const auto& geometry = calibration_json["floor_geometry"];
                    if (geometry.contains("floor_plane") && geometry["floor_plane"].is_object()) {
                        calibration_json["floor_plane"] = geometry["floor_plane"];
                        calibration_json["floor_plane_confidence"] = geometry.value("floor_plane_confidence", calibration_json.value("floor_plane_confidence", 0.0));
                    }
                }
            }
            if (has_wall_rectangles_save) {
                nlohmann::json walls = nlohmann::json::array();
                for (const auto& item : payload["wall_rectangles_auto"]) {
                    if (!item.is_object() || !item.value("valid", false)) {
                        continue;
                    }
                    auto wall = item;
                    wall["applied_to_runtime"] = false;
                    walls.push_back(std::move(wall));
                }
                calibration_json["wall_rectangles"] = std::move(walls);
                calibration_json["wall_rectangles_by_camera"]["camera_a"] = calibration_json["wall_rectangles"];
            }
            if (has_wall_rectangles_by_camera_save) {
                nlohmann::json by_camera = nlohmann::json::object();
                for (const auto& camera_key : {"camera_a", "camera_b"}) {
                    nlohmann::json walls = nlohmann::json::array();
                    if (payload["wall_rectangles_by_camera"].contains(camera_key) &&
                        payload["wall_rectangles_by_camera"][camera_key].is_array()) {
                        for (const auto& item : payload["wall_rectangles_by_camera"][camera_key]) {
                            if (!item.is_object() || !item.value("valid", false)) {
                                continue;
                            }
                            auto wall = item;
                            wall["applied_to_runtime"] = false;
                            walls.push_back(std::move(wall));
                        }
                    }
                    by_camera[camera_key] = std::move(walls);
                }
                calibration_json["wall_rectangles_by_camera"] = std::move(by_camera);
                calibration_json["wall_rectangles"] = calibration_json["wall_rectangles_by_camera"].value("camera_a", nlohmann::json::array());
            }
            const auto parent = std::filesystem::path(calibration_path).parent_path();
            if (!parent.empty()) {
                std::error_code mkdir_ec;
                std::filesystem::create_directories(parent, mkdir_ec);
                if (mkdir_ec) {
                    std::error_code remove_ec;
                    std::filesystem::remove(temp_config_path, remove_ec);
                    return {{"status", std::string("config not saved; failed to create calibration directory: ") + mkdir_ec.message()}, {"ok", false}};
                }
            }
            temp_calibration_path = std::filesystem::path(calibration_path);
            temp_calibration_path += ".tmp";
            {
                std::ofstream calib_out(temp_calibration_path);
                if (!calib_out) {
                    std::error_code remove_ec;
                    std::filesystem::remove(temp_config_path, remove_ec);
                    return {{"status", "config not saved; failed to open temporary calibration file for floor geometry save"}, {"ok", false}};
                }
                calib_out << calibration_json.dump(2) << '\n';
                if (!calib_out) {
                    std::error_code remove_ec;
                    std::filesystem::remove(temp_config_path, remove_ec);
                    std::filesystem::remove(temp_calibration_path, remove_ec);
                    return {{"status", "config not saved; failed to finish writing temporary calibration for floor geometry save"}, {"ok", false}};
                }
            }
            const auto calibration_loaded = bt::LoadCalibration(temp_calibration_path);
            if (!calibration_loaded.ok()) {
                std::error_code remove_ec;
                std::filesystem::remove(temp_config_path, remove_ec);
                std::filesystem::remove(temp_calibration_path, remove_ec);
                return {{"status", std::string("config not saved; proposed floor geometry calibration failed validation: ") + calibration_loaded.status().message}, {"ok", false}};
            }
        }

        std::error_code remove_ec;
        auto remove_quietly = [](const std::filesystem::path& path) {
            std::error_code ec;
            std::filesystem::remove(path, ec);
        };

        struct BackupState {
            std::filesystem::path path;
            std::filesystem::path backup;
            bool target_existed = false;
            bool backup_created = false;
        };
        auto make_backup_state = [](const std::filesystem::path& target) {
            BackupState state;
            state.path = target;
            state.backup = target;
            state.backup += ".bak";
            return state;
        };
        auto backup_if_present = [](BackupState& state, std::string* error) {
            std::error_code ec;
            const bool stale_backup_exists = std::filesystem::exists(state.backup, ec);
            if (ec) {
                if (error) {
                    *error = std::string("could not inspect stale backup: ") + ec.message();
                }
                return false;
            }
            if (stale_backup_exists) {
                std::filesystem::remove(state.backup, ec);
                if (ec) {
                    if (error) {
                        *error = std::string("could not clear stale backup: ") + ec.message();
                    }
                    return false;
                }
            }

            ec.clear();
            state.target_existed = std::filesystem::exists(state.path, ec);
            if (ec) {
                if (error) {
                    *error = std::string("could not inspect live file: ") + ec.message();
                }
                return false;
            }
            if (!state.target_existed) {
                return true;
            }
            std::filesystem::rename(state.path, state.backup, ec);
            if (ec) {
                if (error) {
                    *error = std::string("could not move live file to transaction backup: ") + ec.message();
                }
                return false;
            }
            state.backup_created = true;
            return true;
        };
        auto restore_from_backup = [](const BackupState& state, std::string* error) {
            std::error_code ec;
            std::filesystem::remove(state.path, ec);
            if (ec) {
                if (error) {
                    *error = std::string("could not clear partial live file: ") + ec.message();
                }
                return false;
            }
            if (state.backup_created) {
                ec.clear();
                std::filesystem::rename(state.backup, state.path, ec);
                if (ec) {
                    if (error) {
                        *error = std::string("could not restore transaction backup: ") + ec.message();
                    }
                    return false;
                }
            }
            return true;
        };
        auto discard_backup = [](const BackupState& state, std::string* error) {
            if (!state.backup_created) {
                return true;
            }
            std::error_code ec;
            std::filesystem::remove(state.backup, ec);
            if (ec) {
                if (error) {
                    *error = ec.message();
                }
                return false;
            }
            return true;
        };
        auto replace_from_temp = [&](const std::filesystem::path& temp_path, const BackupState& state, const char* label, std::string* error) {
            std::error_code ec;
            std::filesystem::rename(temp_path, state.path, ec);
            if (!ec) {
                return true;
            }
            std::string rollback_error;
            const bool restored = restore_from_backup(state, &rollback_error);
            if (error) {
                *error = std::string("could not install validated ") + label + ": " + ec.message();
                if (!restored) {
                    *error += "; rollback failed: " + rollback_error;
                }
            }
            return false;
        };

        const bool writes_calibration = has_floor_geometry_save || has_floor_geometry_by_camera_save ||
            clear_floor_geometry || has_wall_rectangles_save || has_wall_rectangles_by_camera_save;
        BackupState config_backup = make_backup_state(config_path_);
        BackupState calibration_backup = make_backup_state(calibration_path);
        std::string backup_error;
        if (!backup_if_present(config_backup, &backup_error)) {
            remove_quietly(temp_config_path);
            if (writes_calibration) {
                remove_quietly(temp_calibration_path);
            }
            return {{"status", std::string("config not saved; could not prepare live config for commit: ") + backup_error}, {"ok", false}};
        }
        if (writes_calibration && !backup_if_present(calibration_backup, &backup_error)) {
            std::string config_restore_error;
            const bool config_restored = restore_from_backup(config_backup, &config_restore_error);
            remove_quietly(temp_config_path);
            remove_quietly(temp_calibration_path);
            std::string status = "config not saved; could not prepare live calibration for commit: " + backup_error;
            if (!config_restored) {
                status += "; config rollback failed: " + config_restore_error;
            }
            return {{"status", status}, {"ok", false}};
        }

        std::string replace_error;
        if (!replace_from_temp(temp_config_path, config_backup, "config", &replace_error)) {
            remove_quietly(temp_config_path);
            if (writes_calibration) {
                std::string calibration_restore_error;
                restore_from_backup(calibration_backup, &calibration_restore_error);
                remove_quietly(temp_calibration_path);
            }
            return {{"status", std::string("config not saved; ") + replace_error}, {"ok", false}};
        }

        if (writes_calibration) {
            if (!replace_from_temp(temp_calibration_path, calibration_backup, "calibration", &replace_error)) {
                std::string config_restore_error;
                const bool config_restored = restore_from_backup(config_backup, &config_restore_error);
                remove_quietly(temp_config_path);
                remove_quietly(temp_calibration_path);
                std::string restore_status = "config not saved; " + replace_error;
                if (!config_restored) {
                    restore_status += "; rollback incomplete; config restore failed: " + config_restore_error;
                } else {
                    restore_status += "; restored previous config; calibration rollback handled by failed calibration commit";
                }
                return {{"status", restore_status}, {"ok", false}};
            }
        }

        remove_quietly(temp_config_path);
        if (writes_calibration) {
            remove_quietly(temp_calibration_path);
        }
        std::string cleanup_error;
        std::string cleanup_warning;
        if (!discard_backup(config_backup, &cleanup_error)) {
            cleanup_warning = std::string("failed to remove transaction backup: ") + cleanup_error;
        }
        if (writes_calibration && !discard_backup(calibration_backup, &cleanup_error)) {
            if (!cleanup_warning.empty()) {
                cleanup_warning += "; ";
            }
            cleanup_warning += std::string("failed to remove calibration transaction backup: ") + cleanup_error;
        }

        std::scoped_lock lock(mutex_);
        config_ = loaded.value();
        runtime_state_.SetConfig(config_);
        runtime_error_.clear();
        std::string combined_warning = cleanup_warning;
        if (!osc_save_warning.empty()) {
            if (!combined_warning.empty()) {
                combined_warning += "; ";
            }
            combined_warning += osc_save_warning;
        }
        nlohmann::json response = {{"status", combined_warning.empty() ? "config saved" : std::string("config saved; ") + combined_warning}, {"ok", true}};
        if (!combined_warning.empty()) {
            response["warning"] = combined_warning;
            response["warning_type"] = cleanup_warning.empty()
                ? "osc_tracker_space_invalid"
                : (osc_save_warning.empty() ? "cleanup_warning" : "multiple_warnings");
        }
        if (!osc_save_warning.empty()) {
            response["tracker_space_status"] = tracker_space_status;
        }
        if (restart_required_after_save) {
            response["restart_required"] = true;
            response["status"] = std::string(response["status"].get<std::string>()) + "; restart runtime to apply non-hot fields";
        }
        return response;
    }

    nlohmann::json OpenModelsFolder() {
        std::filesystem::path dir;
        {
            std::scoped_lock lock(mutex_);
            dir = config_.tracking.model_path.parent_path();
            if (dir.empty()) {
                dir = "models";
            }
        }
        std::error_code ec;
        std::filesystem::create_directories(dir, ec);
        if (ec) {
            return {{"status", std::string("failed to create models folder: ") + ec.message()}, {"ok", false}};
        }
        return OpenFolder(dir, "models folder opened");
    }

    nlohmann::json OpenCalibrationFolder() {
        std::filesystem::path dir;
        {
            std::scoped_lock lock(mutex_);
            dir = config_.tracking.calibration_path.parent_path();
            if (dir.empty()) {
                dir = "calib";
            }
        }
        std::error_code ec;
        std::filesystem::create_directories(dir, ec);
        if (ec) {
            return {{"status", std::string("failed to create calibration folder: ") + ec.message()}, {"ok", false}};
        }
        return OpenFolder(dir, "calibration folder opened");
    }

    nlohmann::json CreateCalibrationTemplate() {
        std::filesystem::path path;
        {
            std::scoped_lock lock(mutex_);
            path = config_.tracking.calibration_path;
        }
        if (path.empty()) {
            path = "calib/default.json";
        }
        std::error_code exists_ec;
        const bool exists = std::filesystem::exists(path, exists_ec);
        if (exists_ec) {
            return {{"status", std::string("failed to inspect calibration file: ") + exists_ec.message()}, {"ok", false}};
        }
        if (exists) {
            return {{"status", "calibration file already exists"}, {"ok", true}};
        }
        const auto status = bt::SaveCalibrationTemplate(path);
        if (!status.ok()) {
            return {{"status", status.message}, {"ok", false}};
        }
        return {{"status", "calibration template created"}, {"ok", true}};
    }

    nlohmann::json OpenBuildFolder() {
        return OpenFolder(ExecutableDirectory(), "build folder opened");
    }

    static bool PathExistsNoThrow(const std::filesystem::path& path) {
        std::error_code ec;
        const bool exists = std::filesystem::exists(path, ec);
        return exists && !ec;
    }

    static std::string JsonStringOr(const nlohmann::json& object, const char* key, std::string fallback = {}) {
        if (object.contains(key) && object[key].is_string()) {
            return object[key].get<std::string>();
        }
        return fallback;
    }

    nlohmann::json PhoneSiteStateJsonForPort(int target_port) const {
        const std::filesystem::path phone_dir = ExecutableDirectory() / "ui" / "phone";
        const std::filesystem::path state_path = phone_dir / "phone-site.json";
        const std::filesystem::path launcher_path = phone_dir / "start-phone-camera.ps1";
        const std::filesystem::path stop_path = phone_dir / "stop-phone-camera.ps1";
        const std::vector<std::filesystem::path> apk_candidates = {
            phone_dir / "app-debug.apk",
            phone_dir / ".." / ".." / ".." / "android" / "FBTPhoneCamera" / "app" / "build" / "outputs" / "apk" / "debug" / "app-debug.apk",
            phone_dir / ".." / ".." / ".." / ".." / "android" / "FBTPhoneCamera" / "app" / "build" / "outputs" / "apk" / "debug" / "app-debug.apk",
        };
        std::filesystem::path apk_path;
        for (const auto& candidate : apk_candidates) {
            if (PathExistsNoThrow(candidate)) {
                apk_path = candidate;
                break;
            }
        }
        const bool apk_available = !apk_path.empty();
        const std::string configured_target = "127.0.0.1:" + std::to_string(target_port);
        nlohmann::json state = {
            {"enabled", false},
            {"status", "disabled"},
            {"url", ""},
            {"target", configured_target},
            {"configured_target", configured_target},
            {"target_port", target_port},
            {"apk", apk_available},
            {"apk_path", apk_available ? ResolveDisplayPath(apk_path) : std::string()},
            {"launcher_available", PathExistsNoThrow(launcher_path)},
            {"stop_available", PathExistsNoThrow(stop_path)},
            {"state_path", ResolveDisplayPath(state_path)}
        };
        std::ifstream in(state_path);
        if (in) {
            try {
                nlohmann::json saved;
                in >> saved;
                if (saved.is_object()) {
                    for (auto it = saved.begin(); it != saved.end(); ++it) {
                        state[it.key()] = it.value();
                    }
                }
            } catch (const std::exception& e) {
                state["status"] = "state unreadable";
                state["state_error"] = e.what();
                state["enabled"] = false;
            }
        }
        const bool enabled = state.contains("enabled") && state["enabled"].is_boolean() && state["enabled"].get<bool>();
        const std::string url = JsonStringOr(state, "url");
        int active_target_port = target_port;
        if (state.contains("target_port") && state["target_port"].is_number_integer()) {
            active_target_port = state["target_port"].get<int>();
        }
        if (active_target_port <= 0 || active_target_port > 65535) {
            active_target_port = target_port;
        }
        state["enabled"] = enabled && !url.empty();
        state["url"] = url;
        state["target_port"] = active_target_port;
        state["target"] = "127.0.0.1:" + std::to_string(active_target_port);
        state["configured_target"] = configured_target;
        state["apk"] = apk_available;
        state["apk_path"] = apk_available ? ResolveDisplayPath(apk_path) : std::string();
        state["launcher_available"] = PathExistsNoThrow(launcher_path);
        state["stop_available"] = PathExistsNoThrow(stop_path);
        state["state_path"] = ResolveDisplayPath(state_path);
        const std::string status = JsonStringOr(state, "status", "disabled");
        if (enabled && !url.empty() && status == "disabled") {
            state["status"] = "enabled";
        } else if (enabled && url.empty() && status == "enabled") {
            state["status"] = "state missing url";
        } else if (!state["enabled"].get<bool>() && status == "enabled") {
            state["status"] = "disabled";
        }
        return state;
    }

    nlohmann::json PhoneSiteStateJson() {
        int target_port = 39555;
        {
            std::scoped_lock lock(mutex_);
            target_port = config_.camera_a.network_port;
        }
        return PhoneSiteStateJsonForPort(target_port);
    }

    nlohmann::json EnablePhoneWebCamera(bool open_site_when_ready) {
#ifdef _WIN32
        std::filesystem::path script = ExecutableDirectory() / "ui" / "phone" / "start-phone-camera.ps1";
        std::error_code ec;
        if (!std::filesystem::exists(script, ec) || ec) {
            return {{"status", "phone web launcher missing"}, {"ok", false}, {"path", ResolveDisplayPath(script)}};
        }
        int port = 39555;
        {
            std::scoped_lock lock(mutex_);
            port = config_.camera_a.network_port;
        }
        const std::wstring params =
            L"-NoProfile -ExecutionPolicy Bypass -File \"" +
            script.wstring() +
            L"\" -TargetPort " +
            std::to_wstring(port) +
            (open_site_when_ready ? L" -Open" : L"");
        const auto result = ShellExecuteW(nullptr, L"open", L"powershell.exe", params.c_str(), script.parent_path().wstring().c_str(), SW_HIDE);
        if (reinterpret_cast<intptr_t>(result) <= 32) {
            return {{"status", "failed to launch phone web camera"}, {"ok", false}};
        }
        return {{"status", "phone camera site launch requested"}, {"ok", true}, {"port", port}, {"phone_site", PhoneSiteStateJsonForPort(port)}};
#else
        return {{"status", "phone web camera launcher is Windows-only"}, {"ok", false}};
#endif
    }

    nlohmann::json OpenPhoneWebCamera() {
#ifdef _WIN32
        const auto state = PhoneSiteStateJson();
        const std::string url = state.value("url", std::string{});
        if (url.empty()) {
            return {{"status", "phone camera site is not enabled"}, {"ok", false}, {"phone_site", state}};
        }
        const std::wstring wide(url.begin(), url.end());
        const auto result = ShellExecuteW(nullptr, L"open", wide.c_str(), nullptr, nullptr, SW_SHOWNORMAL);
        if (reinterpret_cast<intptr_t>(result) <= 32) {
            return {{"status", "failed to open phone camera site"}, {"ok", false}};
        }
        return {{"status", "phone camera site opened"}, {"ok", true}, {"url", url}};
#else
        return {{"status", "opening phone camera site is Windows-only"}, {"ok", false}};
#endif
    }

    nlohmann::json DisablePhoneWebCamera() {
#ifdef _WIN32
        const std::filesystem::path script = ExecutableDirectory() / "ui" / "phone" / "stop-phone-camera.ps1";
        if (!PathExistsNoThrow(script)) {
            return {{"status", "phone web stop script missing"}, {"ok", false}, {"path", ResolveDisplayPath(script)}};
        }
        const std::wstring params = L"-NoProfile -ExecutionPolicy Bypass -File \"" + script.wstring() + L"\"";
        const auto result = ShellExecuteW(nullptr, L"open", L"powershell.exe", params.c_str(), script.parent_path().wstring().c_str(), SW_HIDE);
        if (reinterpret_cast<intptr_t>(result) <= 32) {
            return {{"status", "failed to disable phone camera site"}, {"ok", false}};
        }
        return {{"status", "phone camera site stop requested"}, {"ok", true}};
#else
        return {{"status", "phone camera site disable is Windows-only"}, {"ok", false}};
#endif
    }

    nlohmann::json PrepareDeployFolder() {
#ifdef _WIN32
        try {
            const std::filesystem::path exe_path = ExecutablePath();
            const std::filesystem::path exe_dir = exe_path.parent_path();
            const std::filesystem::path deploy_dir = std::filesystem::absolute("dist/bodytracker-debug");
            std::filesystem::create_directories(deploy_dir);
            if (std::filesystem::exists(exe_path)) {
                std::filesystem::copy_file(exe_path, deploy_dir / exe_path.filename(), std::filesystem::copy_options::overwrite_existing);
            }
            for (const auto& entry : std::filesystem::directory_iterator(exe_dir)) {
                if (!entry.is_regular_file()) {
                    continue;
                }
                std::string ext = entry.path().extension().string();
                std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
                if (ext == ".dll") {
                    std::filesystem::copy_file(entry.path(), deploy_dir / entry.path().filename(), std::filesystem::copy_options::overwrite_existing);
                }
            }
            const std::filesystem::path ui_dir = exe_dir / "ui";
            if (std::filesystem::exists(ui_dir)) {
                std::filesystem::copy(ui_dir, deploy_dir / "ui",
                    std::filesystem::copy_options::recursive | std::filesystem::copy_options::overwrite_existing);
            }
            if (std::filesystem::exists(config_path_)) {
                std::filesystem::create_directories(deploy_dir / "config");
                std::filesystem::copy_file(config_path_, deploy_dir / "config" / config_path_.filename(), std::filesystem::copy_options::overwrite_existing);
            }
            for (const auto& folder : {std::filesystem::path("calib"), std::filesystem::path("models")}) {
                if (std::filesystem::exists(folder)) {
                    std::filesystem::copy(folder, deploy_dir / folder.filename(),
                        std::filesystem::copy_options::recursive | std::filesystem::copy_options::overwrite_existing);
                } else {
                    std::filesystem::create_directories(deploy_dir / folder.filename());
                }
            }
            return {{"status", "deploy folder prepared"}, {"ok", true}, {"path", ResolveDisplayPath(deploy_dir)}};
        } catch (const std::exception& e) {
            return {{"status", std::string("deploy failed: ") + e.what()}, {"ok", false}};
        }
#else
        return {{"status", "deploy folder is Windows-only for this UI"}, {"ok", false}};
#endif
    }

    static nlohmann::json OpenFolder(const std::filesystem::path& dir, const char* success) {
#ifdef _WIN32
        const std::wstring wide = ToWidePath(std::filesystem::absolute(dir).wstring());
        const auto result = ShellExecuteW(nullptr, L"open", wide.c_str(), nullptr, nullptr, SW_SHOWNORMAL);
        if (reinterpret_cast<intptr_t>(result) <= 32) {
            return {{"status", "failed to open folder"}, {"ok", false}};
        }
        return {{"status", success}, {"ok", true}};
#else
        return {{"status", "opening folders is only wired on Windows"}, {"ok", false}};
#endif
    }

    static std::filesystem::path ExecutablePath() {
#ifdef _WIN32
        std::wstring buffer(MAX_PATH, L'\0');
        DWORD size = GetModuleFileNameW(nullptr, buffer.data(), static_cast<DWORD>(buffer.size()));
        while (size == buffer.size()) {
            buffer.resize(buffer.size() * 2);
            size = GetModuleFileNameW(nullptr, buffer.data(), static_cast<DWORD>(buffer.size()));
        }
        buffer.resize(size);
        return std::filesystem::path(buffer);
#else
        return std::filesystem::current_path();
#endif
    }

    static std::filesystem::path ExecutableDirectory() {
        const auto path = ExecutablePath();
        return path.has_parent_path() ? path.parent_path() : std::filesystem::current_path();
    }

    static void EnsureObject(nlohmann::json& j, const char* key) {
        if (!j[key].is_object()) {
            j[key] = nlohmann::json::object();
        }
    }

    static std::wstring ToWidePath(const std::wstring& value) {
        return value;
    }

    static float FootSupportConfidenceForUi(const bt::FootSupportState& support) {
        float confidence = support.anchor.active ? Clamp01ForTelemetry(support.anchor.confidence) : 0.0f;
        if (support.heel_anchor.active) {
            confidence = std::max(confidence, Clamp01ForTelemetry(support.heel_anchor.confidence));
        }
        if (support.toe_anchor.active) {
            confidence = std::max(confidence, Clamp01ForTelemetry(support.toe_anchor.confidence));
        }
        return Clamp01ForTelemetry(confidence);
    }

    static nlohmann::json FootSupportUiToJson(const bt::FootSupportState& support) {
        return {
            {"type", bt::ToString(support.type)},
            {"phase", bt::ToString(support.phase)},
            {"support_confidence", FootSupportConfidenceForUi(support)},
            {"anchor_active", support.anchor.active},
            {"heel_anchor_active", support.heel_anchor.active},
            {"toe_anchor_active", support.toe_anchor.active}
        };
    }

    static nlohmann::json ContactRootUiToJson(const bt::ContactRootCorrectionTelemetry& contact) {
        return {
            {"applied", contact.applied},
            {"reason", bt::ToString(contact.reason)},
            {"correction_m", contact.correction_m},
            {"left_residual_m", contact.left_residual_m},
            {"right_residual_m", contact.right_residual_m},
            {"foot_disagreement_m", contact.disagreement_m},
            {"root_alignment", contact.root_alignment}
        };
    }

    static nlohmann::json SolveConstraintResidualUiToJson(const bt::BodySolveConstraintResidualTelemetry& residual) {
        return {
            {"active", residual.active},
            {"weight", residual.weight},
            {"residual", residual.residual_m},
            {"score", residual.score}
        };
    }

    static nlohmann::json FootSolveConstraintsUiToJson(const bt::BodySolveFootConstraintTelemetry& foot) {
        return {
            {"support_confidence", foot.support_confidence},
            {"transition_quality", foot.transition_quality},
            {"floor_weight_scale", foot.floor_weight_scale},
            {"body_weight_scale", foot.body_weight_scale},
            {"heel_anchor", SolveConstraintResidualUiToJson(foot.heel_anchor)},
            {"toe_anchor", SolveConstraintResidualUiToJson(foot.toe_anchor)},
            {"full_plant", SolveConstraintResidualUiToJson(foot.full_plant)},
            {"floor_penetration", SolveConstraintResidualUiToJson(foot.floor_penetration)},
            {"sliding_velocity", SolveConstraintResidualUiToJson(foot.sliding_velocity)},
            {"orientation", SolveConstraintResidualUiToJson(foot.orientation)},
            {"degraded_or_released", foot.degraded_or_released}
        };
    }

    static nlohmann::json SupportSolveConstraintsUiToJson(const bt::BodySolveSupportConstraintTelemetry& constraints) {
        return {
            {"floor_calibration_weight", constraints.floor_calibration_weight},
            {"leg_length_weight", constraints.leg_length_weight},
            {"left_foot_length_weight", constraints.left_foot_length_weight},
            {"right_foot_length_weight", constraints.right_foot_length_weight},
            {"body_calibration_present", constraints.body_calibration_present},
            {"body_calibration_confidence", constraints.body_calibration_confidence},
            {"body_calibration_sample_count", constraints.body_calibration_sample_count},
            {"left_reach_clamped", constraints.left_reach_clamped},
            {"right_reach_clamped", constraints.right_reach_clamped},
            {"bone_length", SolveConstraintResidualUiToJson(constraints.bone_length)},
            {"root_support", SolveConstraintResidualUiToJson(constraints.root_support)},
            {"left_knee_floor_anchor", SolveConstraintResidualUiToJson(constraints.left_knee_floor_anchor)},
            {"right_knee_floor_anchor", SolveConstraintResidualUiToJson(constraints.right_knee_floor_anchor)},
            {"left_foot", FootSolveConstraintsUiToJson(constraints.left_foot)},
            {"right_foot", FootSolveConstraintsUiToJson(constraints.right_foot)}
        };
    }

    static nlohmann::json MotionTargetUiToJson(const bt::MotionTargetTelemetry& entry) {
        return {
            {"decision", bt::ToString(entry.decision)},
            {"reason", bt::ToString(entry.reason)},
            {"measured_distance_m", entry.measured_distance_m},
            {"expected_distance_m", entry.expected_distance_m},
            {"direction_deviation_deg", entry.direction_deviation_deg},
            {"lateral_deviation_ratio", entry.lateral_deviation_ratio},
            {"speed_change_ratio", entry.speed_change_ratio},
            {"direction_limit_deg", entry.direction_limit_deg},
            {"lateral_limit_ratio", entry.lateral_limit_ratio},
            {"speed_change_limit_ratio", entry.speed_change_limit_ratio},
            {"pending_frames", entry.pending_frames},
            {"confirm_frames", entry.confirm_frames}
        };
    }

    static nlohmann::json MotionFilterUiToJson(const bt::MotionConsistencyTelemetry& telemetry) {
        return {
            {"root", MotionTargetUiToJson(telemetry.targets[static_cast<std::size_t>(bt::MotionTarget::Root)])},
            {"left_foot", MotionTargetUiToJson(telemetry.targets[static_cast<std::size_t>(bt::MotionTarget::LeftFoot)])},
            {"right_foot", MotionTargetUiToJson(telemetry.targets[static_cast<std::size_t>(bt::MotionTarget::RightFoot)])},
            {"contact_root", ContactRootUiToJson(telemetry.contact_root)}
        };
    }

    static nlohmann::json SolverUiToJson(const bt::DebugSnapshot& debug, const bt::TrackingSolverTelemetry& solver) {
        return {
            {"tracking_mode", bt::ToString(solver.tracking_mode)},
            {"depth_source", bt::ToString(solver.depth_source)},
            {"depth", DepthTelemetryToJson(solver, debug.tracking.floor_geometry)},
            {"used_hmd", solver.used_hmd},
            {"degraded", solver.degraded},
            {"reason", solver.reason},
            {"camera_a_identity_swapped", solver.camera_a_identity_swapped},
            {"camera_b_identity_swapped", solver.camera_b_identity_swapped},
            {"camera_a_identity_consistency", solver.camera_a_identity_consistency},
            {"camera_b_identity_consistency", solver.camera_b_identity_consistency},
            {"identity_epipolar_arbitration_checked", solver.identity_epipolar_arbitration_checked},
            {"identity_epipolar_arbitration_applied", solver.identity_epipolar_arbitration_applied},
            {"identity_epipolar_scored_lateral_pairs", solver.identity_epipolar_scored_lateral_pairs},
            {"identity_epipolar_same_score", solver.identity_epipolar_same_score},
            {"identity_epipolar_cross_score", solver.identity_epipolar_cross_score},
            {"identity_epipolar_cross_geometric_uncertainty", solver.identity_epipolar_cross_geometric_uncertainty},
            {"identity_epipolar_detection_support", solver.identity_epipolar_detection_support},
            {"identity_epipolar_required_swap_margin", solver.identity_epipolar_required_swap_margin},
            {"identity_same_mahalanobis_sq", solver.identity_same_mahalanobis_sq},
            {"identity_cross_mahalanobis_sq", solver.identity_cross_mahalanobis_sq},
            {"identity_same_negative_log_likelihood", solver.identity_same_negative_log_likelihood},
            {"identity_cross_negative_log_likelihood", solver.identity_cross_negative_log_likelihood},
            {"identity_cross_within_mahalanobis_gate", solver.identity_cross_within_mahalanobis_gate},
            {"identity_score_gate_passed", solver.identity_score_gate_passed},
            {"identity_likelihood_gate_passed", solver.identity_likelihood_gate_passed},
            {"identity_swap_blocked_by_strong_consistency", solver.identity_swap_blocked_by_strong_consistency},
            {"identity_swap_blocked_by_tie", solver.identity_swap_blocked_by_tie},
            {"identity_uncertainty_fallback_count", solver.identity_uncertainty_fallback_count},
            {"preliminary_residual", solver.preliminary_residual},
            {"final_residual", solver.final_residual},
            {"preliminary_weighted_observation_count", solver.preliminary_weighted_observation_count},
            {"final_weighted_observation_count", solver.final_weighted_observation_count},
            {"left_foot_contact_confidence", solver.preliminary_stereo.left_foot_contact_confidence},
            {"right_foot_contact_confidence", solver.preliminary_stereo.right_foot_contact_confidence},
            {"left_foot_low_res_separation_px", solver.preliminary_stereo.left_foot_low_res_separation_px},
            {"right_foot_low_res_separation_px", solver.preliminary_stereo.right_foot_low_res_separation_px},
            {"inferred_depth_count", solver.preliminary_stereo.inferred_depth_count},
            {"mean_inferred_depth_m", solver.preliminary_stereo.mean_inferred_depth_m},
            {"mean_reprojection_error_px", solver.preliminary_stereo.mean_reprojection_error_px},
            {"measurement_uncertainty_count", solver.preliminary_stereo.measurement_uncertainty_count},
            {"mean_measurement_position_stddev_m", solver.preliminary_stereo.mean_measurement_position_stddev_m},
            {"mean_measurement_depth_stddev_m", solver.preliminary_stereo.mean_measurement_depth_stddev_m},
            {"mean_measurement_baseline_to_depth_ratio", solver.preliminary_stereo.mean_measurement_baseline_to_depth_ratio},
            {"solver_uncertainty_weighted_count", solver.preliminary_stereo.solver_uncertainty_weighted_count},
            {"solver_uncertainty_valid_count", solver.preliminary_stereo.solver_uncertainty_valid_count},
            {"solver_uncertainty_conservative_fallback_count", solver.preliminary_stereo.solver_uncertainty_conservative_fallback_count},
            {"mean_solver_lateral_weight_scale", solver.preliminary_stereo.mean_solver_lateral_weight_scale},
            {"mean_solver_depth_weight_scale", solver.preliminary_stereo.mean_solver_depth_weight_scale},
            {"foot_mean_reprojection_error_px", solver.preliminary_stereo.foot_mean_reprojection_error_px},
            {"max_foot_reprojection_error_px", solver.preliminary_stereo.max_foot_reprojection_error_px},
            {"camera_a_geometry_used", solver.preliminary_stereo.camera_a_geometry_used},
            {"camera_b_geometry_used", solver.preliminary_stereo.camera_b_geometry_used},
            {"stereo_geometry_constraints_used", solver.preliminary_stereo.stereo_geometry_constraints_used},
            {"stereo_geometry_confidence", solver.preliminary_stereo.stereo_geometry_confidence},
            {"geometry_stereo_status", solver.preliminary_stereo.geometry_stereo_status},
            {"epipolar_geometry_valid", solver.preliminary_stereo.epipolar_geometry_valid},
            {"epipolar_status", solver.preliminary_stereo.epipolar_status},
            {"epipolar_checked_count", solver.preliminary_stereo.epipolar_checked_count},
            {"epipolar_hard_mismatch_count", solver.preliminary_stereo.epipolar_hard_mismatch_count},
            {"epipolar_pair_rejected_count", solver.preliminary_stereo.epipolar_pair_rejected_count},
            {"epipolar_degraded_pair_softened_count", solver.preliminary_stereo.epipolar_degraded_pair_softened_count},
            {"mean_epipolar_error_px", solver.preliminary_stereo.mean_epipolar_error_px},
            {"mean_epipolar_error_px_isotropic_heuristic", solver.preliminary_stereo.mean_epipolar_error_px_isotropic},
            {"mean_epipolar_error_px_anisotropic", solver.preliminary_stereo.mean_epipolar_error_px_anisotropic},
            {"mean_epipolar_error_normalized", solver.preliminary_stereo.mean_epipolar_error_normalized},
            {"mean_epipolar_confidence", solver.preliminary_stereo.mean_epipolar_confidence},
            {"epipolar", EpipolarStereoTelemetryToJson(solver.preliminary_stereo)},
            {"stereo", StereoTelemetryToJson(solver.preliminary_stereo)},
            {"support_constraints", SupportSolveConstraintsUiToJson(solver.final_constraints)},
            {"monocular_scale_source", bt::ToString(solver.preliminary_stereo.monocular_scale_source)},
            {"monocular_floor_assist_depth_m", solver.preliminary_stereo.monocular_floor_assist_depth_m},
            {"monocular_floor_assist_confidence", solver.preliminary_stereo.monocular_floor_assist_confidence},
            {"room_depth_map", RoomDepthMapTelemetryToJson(solver.room_depth_map)}
        };
    }

    static nlohmann::json ConfigToJson(const bt::AppConfig& cfg) {
        return {
            {"app", {
                {"log_file", cfg.app.log_file.string()},
                {"recording_dir", cfg.app.recording_dir.string()}
            }},
            {"camera_a", {
                {"source", cfg.camera_a.source},
                {"device_index", cfg.camera_a.device_index},
                {"width", cfg.camera_a.width},
                {"height", cfg.camera_a.height},
                {"fps", cfg.camera_a.fps},
                {"network_bind_address", cfg.camera_a.network_bind_address},
                {"network_port", cfg.camera_a.network_port},
                {"network_read_timeout_ms", cfg.camera_a.network_read_timeout_ms},
                {"network_max_frame_bytes", cfg.camera_a.network_max_frame_bytes},
                {"initial_roi_enabled", cfg.camera_a.initial_roi_enabled},
                {"initial_roi_normalized", cfg.camera_a.initial_roi_normalized},
                {"initial_roi", {cfg.camera_a.initial_roi.x, cfg.camera_a.initial_roi.y, cfg.camera_a.initial_roi.width, cfg.camera_a.initial_roi.height}}
            }},
            {"camera_b", {
                {"source", cfg.camera_b.source},
                {"device_index", cfg.camera_b.device_index},
                {"width", cfg.camera_b.width},
                {"height", cfg.camera_b.height},
                {"fps", cfg.camera_b.fps},
                {"network_bind_address", cfg.camera_b.network_bind_address},
                {"network_port", cfg.camera_b.network_port},
                {"network_read_timeout_ms", cfg.camera_b.network_read_timeout_ms},
                {"network_max_frame_bytes", cfg.camera_b.network_max_frame_bytes},
                {"initial_roi_enabled", cfg.camera_b.initial_roi_enabled},
                {"initial_roi_normalized", cfg.camera_b.initial_roi_normalized},
                {"initial_roi", {cfg.camera_b.initial_roi.x, cfg.camera_b.initial_roi.y, cfg.camera_b.initial_roi.width, cfg.camera_b.initial_roi.height}}
            }},
            {"tracking", {
                {"mode", bt::ToString(cfg.tracking.mode)},
                {"model_path", cfg.tracking.model_path.string()},
                {"calibration_path", cfg.tracking.calibration_path.string()},
                {"latest_frame_skew_tolerance_ms", cfg.tracking.latest_frame_skew_tolerance_ms},
                {"max_frame_skew_ms", cfg.tracking.max_frame_skew_ms},
                {"stale_frame_timeout_ms", cfg.tracking.stale_frame_timeout_ms},
                {"min_triangulated_seed_count", cfg.tracking.min_triangulated_seed_count},
                {"max_mean_reprojection_error_px", cfg.tracking.max_mean_reprojection_error_px},
                {"stereo_monocular_fallback_enabled", cfg.tracking.stereo_monocular_fallback_enabled},
                {"use_legacy_solver", cfg.tracking.use_legacy_solver},
                {"enable_replay_recording", cfg.tracking.enable_replay_recording},
                {"body_calibration", {
                    {"enabled", cfg.tracking.body_calibration.enabled},
                    {"auto_persist", cfg.tracking.body_calibration.auto_persist},
                    {"required_seconds", cfg.tracking.body_calibration.required_seconds},
                    {"min_overall_confidence", cfg.tracking.body_calibration.min_overall_confidence},
                    {"max_segment_cv", cfg.tracking.body_calibration.max_segment_cv}
                }},
                {"monocular", {
                    {"image_width", cfg.tracking.monocular.image_width},
                    {"image_height", cfg.tracking.monocular.image_height},
                    {"horizontal_fov_deg", cfg.tracking.monocular.horizontal_fov_deg},
                    {"user_height_m", cfg.tracking.monocular.user_height_m},
                    {"camera_height_m", cfg.tracking.monocular.camera_height_m},
                    {"default_depth_m", cfg.tracking.monocular.default_depth_m},
                    {"depth_confidence_scale", cfg.tracking.monocular.depth_confidence_scale},
                    {"min_keypoint_confidence", cfg.tracking.monocular.min_keypoint_confidence},
                    {"min_seed_count", cfg.tracking.monocular.min_seed_count},
                    {"floor_scale_assist_enabled", cfg.tracking.monocular.floor_scale_assist_enabled},
                    {"floor_geometry_calibration_enabled", cfg.tracking.monocular.floor_geometry_calibration_enabled},
                    {"floor_geometry_type", cfg.tracking.monocular.floor_geometry_type},
                    {"floor_depth_line_spacing_m", cfg.tracking.monocular.floor_depth_line_spacing_m},
                    {"floor_depth_line_spacing_px", cfg.tracking.monocular.floor_depth_line_spacing_px},
                    {"floor_depth_reference_y_px", cfg.tracking.monocular.floor_depth_reference_y_px},
                    {"floor_depth_reference_m", cfg.tracking.monocular.floor_depth_reference_m},
                    {"floor_depth_confidence", cfg.tracking.monocular.floor_depth_confidence},
                    {"floor_second_axis_spacing_m", cfg.tracking.monocular.floor_second_axis_spacing_m},
                    {"floor_geometry_confidence", cfg.tracking.monocular.floor_geometry_confidence},
                    {"floor_projective_homography_enabled", cfg.tracking.monocular.floor_projective_homography_enabled},
                    {"floor_from_image", Mat3ToJson(cfg.tracking.monocular.floor_from_image)},
                    {"image_from_floor", Mat3ToJson(cfg.tracking.monocular.image_from_floor)},
                    {"floor_projective_confidence", cfg.tracking.monocular.floor_projective_confidence},
                    {"floor_distortion_correction_enabled", cfg.tracking.monocular.floor_distortion_correction_enabled},
                    {"floor_distortion_confidence", cfg.tracking.monocular.floor_distortion_confidence},
                    {"floor_radial_k1", cfg.tracking.monocular.floor_radial_k1},
                    {"floor_radial_k2", cfg.tracking.monocular.floor_radial_k2},
                    {"floor_tangential_p1", cfg.tracking.monocular.floor_tangential_p1},
                    {"floor_tangential_p2", cfg.tracking.monocular.floor_tangential_p2},
                    {"floor_camera_orientation_enabled", cfg.tracking.monocular.floor_camera_orientation_enabled},
                    {"floor_camera_pitch_rad", cfg.tracking.monocular.floor_camera_pitch_rad},
                    {"floor_camera_roll_rad", cfg.tracking.monocular.floor_camera_roll_rad},
                    {"floor_camera_orientation_confidence", cfg.tracking.monocular.floor_camera_orientation_confidence}
                }},
                {"motion_consistency", {
                    {"enabled", cfg.tracking.motion_consistency.enabled},
                    {"confirm_frames", cfg.tracking.motion_consistency.confirm_frames},
                    {"min_motion_m", cfg.tracking.motion_consistency.min_motion_m},
                    {"stationary_deadzone_m", cfg.tracking.motion_consistency.stationary_deadzone_m},
                    {"max_direction_deviation_deg", cfg.tracking.motion_consistency.max_direction_deviation_deg},
                    {"max_lateral_deviation_ratio", cfg.tracking.motion_consistency.max_lateral_deviation_ratio},
                    {"max_speed_change_ratio", cfg.tracking.motion_consistency.max_speed_change_ratio},
                    {"reject_confidence_decay_per_second", cfg.tracking.motion_consistency.reject_confidence_decay_per_second},
                    {"planted_foot_max_drift_m", cfg.tracking.motion_consistency.planted_foot_max_drift_m},
                    {"planted_foot_release_confirm_frames", cfg.tracking.motion_consistency.planted_foot_release_confirm_frames},
                    {"contact_root_correction_gain", cfg.tracking.motion_consistency.contact_root_correction_gain},
                    {"contact_root_max_correction_m", cfg.tracking.motion_consistency.contact_root_max_correction_m},
                    {"contact_root_max_residual_m", cfg.tracking.motion_consistency.contact_root_max_residual_m},
                    {"contact_root_max_disagreement_m", cfg.tracking.motion_consistency.contact_root_max_disagreement_m},
                    {"contact_root_min_alignment", cfg.tracking.motion_consistency.contact_root_min_alignment},
                    {"contact_root_min_support_confidence", cfg.tracking.motion_consistency.contact_root_min_support_confidence}
                }},
                {"tracker_ekf", {
                    {"enabled", cfg.tracking.tracker_ekf.enabled},
                    {"process_noise_mps2", cfg.tracking.tracker_ekf.process_noise_mps2},
                    {"min_measurement_variance_m2", cfg.tracking.tracker_ekf.min_measurement_variance_m2},
                    {"max_measurement_variance_m2", cfg.tracking.tracker_ekf.max_measurement_variance_m2},
                    {"support_variance_scale", cfg.tracking.tracker_ekf.support_variance_scale},
                    {"missing_velocity_decay", cfg.tracking.tracker_ekf.missing_velocity_decay},
                    {"foot_orientation_gain", cfg.tracking.tracker_ekf.foot_orientation_gain}
                }},
                {"temporal_update", {
                    {"free_gain", cfg.tracking.temporal_update.free_gain},
                    {"supported_gain", cfg.tracking.temporal_update.supported_gain},
                    {"foot_free_gain", cfg.tracking.temporal_update.foot_free_gain},
                    {"foot_supported_gain", cfg.tracking.temporal_update.foot_supported_gain}
                }}
            }},
            {"inference", {{"device", cfg.inference.device}}},
            {"debug", {{"replay_log_path", cfg.debug.replay_log_path.string()}}},
            {"osc", {
                {"enabled", cfg.osc.enabled},
                {"target_address", cfg.osc.target_address},
                {"target_port", cfg.osc.target_port},
                {"send_rotations", cfg.osc.send_rotations},
                {"min_confidence", cfg.osc.min_confidence},
                {"pelvis_tracker_index", cfg.osc.pelvis_tracker_index},
                {"left_foot_tracker_index", cfg.osc.left_foot_tracker_index},
                {"right_foot_tracker_index", cfg.osc.right_foot_tracker_index},
                {"chest_tracker_index", cfg.osc.chest_tracker_index},
                {"left_elbow_tracker_index", cfg.osc.left_elbow_tracker_index},
                {"right_elbow_tracker_index", cfg.osc.right_elbow_tracker_index},
                {"left_knee_tracker_index", cfg.osc.left_knee_tracker_index},
                {"right_knee_tracker_index", cfg.osc.right_knee_tracker_index},
                {"tracker_space_transform_valid", cfg.osc.tracker_space_transform_valid},
                {"tracker_space_position_offset", Vec3ToJson(cfg.osc.tracker_space_position_offset)},
                {"tracker_space_rotation", QuatToJson(cfg.osc.tracker_space_rotation)},
                {"tracker_space_scale", cfg.osc.tracker_space_scale},
                {"tracker_space_source", cfg.osc.tracker_space_source},
                {"tracker_space_role_offsets", [&]() {
                    nlohmann::json offsets = nlohmann::json::array();
                    for (const auto& offset : cfg.osc.tracker_space_role_offsets) {
                        offsets.push_back(Vec3ToJson(offset));
                    }
                    return offsets;
                }()},
                {"manual_tracker_space_transform_valid", cfg.osc.manual_tracker_space_transform_valid},
                {"manual_tracker_space_position_offset", Vec3ToJson(cfg.osc.manual_tracker_space_position_offset)},
                {"manual_tracker_space_rotation", QuatToJson(cfg.osc.manual_tracker_space_rotation)},
                {"manual_tracker_space_scale", cfg.osc.manual_tracker_space_scale},
                {"manual_tracker_space_source", cfg.osc.manual_tracker_space_source},
                {"manual_tracker_space_role_offsets", [&]() {
                    nlohmann::json offsets = nlohmann::json::array();
                    for (const auto& offset : cfg.osc.manual_tracker_space_role_offsets) {
                        offsets.push_back(Vec3ToJson(offset));
                    }
                    return offsets;
                }()},
                {"steamvr_alignment_status", cfg.osc.steamvr_alignment_status},
                {"steamvr_alignment_reason", cfg.osc.steamvr_alignment_reason},
                {"steamvr_alignment_confidence", cfg.osc.steamvr_alignment_confidence},
                {"steamvr_alignment_residual_m", cfg.osc.steamvr_alignment_residual_m},
                {"steamvr_floor_residual_m", cfg.osc.steamvr_floor_residual_m},
                {"steamvr_yaw_offset_rad", cfg.osc.steamvr_yaw_offset_rad},
                {"steamvr_scale_ratio", cfg.osc.steamvr_scale_ratio},
                {"steamvr_alignment_body_signature", cfg.osc.steamvr_alignment_body_signature},
                {"steamvr_alignment_floor_signature", cfg.osc.steamvr_alignment_floor_signature}
            }},
            {"steamvr_tracker_bridge", {
                {"enabled", cfg.steamvr_tracker_bridge.enabled},
                {"target_address", cfg.steamvr_tracker_bridge.target_address},
                {"target_port", cfg.steamvr_tracker_bridge.target_port},
                {"min_confidence", cfg.steamvr_tracker_bridge.min_confidence},
                {"send_chest", cfg.steamvr_tracker_bridge.send_chest},
                {"send_elbows", cfg.steamvr_tracker_bridge.send_elbows},
                {"send_knees", cfg.steamvr_tracker_bridge.send_knees}
            }},
            {"hmd", {{"mode", cfg.hmd.mode}, {"pose_json_path", cfg.hmd.pose_json_path.string()}}}
        };
    }

    static nlohmann::json CaptureHealthToJson(const bt::CaptureHealthSnapshot& camera, double frame_age_ms) {
        return {
            {"opened", camera.opened},
            {"running", camera.running},
            {"source_state", camera.source_state},
            {"last_frame_status", camera.last_frame_status},
            {"last_degraded_reason", camera.last_degraded_reason},
            {"last_decode_error", camera.last_decode_error},
            {"last_error", camera.last_error_message},
            {"frame_age_ms", frame_age_ms},
            {"capture_reported_frame_age_ms", camera.last_frame_age_ms},
            {"network_jitter_ms", camera.network_jitter_ms},
            {"network_reconnect_count", camera.network_reconnect_count},
            {"network_accept_count", camera.network_accept_count},
            {"network_stream_header_count", camera.network_stream_header_count},
            {"network_stream_header_failures", camera.network_stream_header_failures},
            {"network_frame_header_count", camera.network_frame_header_count},
            {"network_frame_header_failures", camera.network_frame_header_failures},
            {"network_frame_payload_count", camera.network_frame_payload_count},
            {"network_frame_payload_failures", camera.network_frame_payload_failures},
            {"network_bad_frame_size_count", camera.network_bad_frame_size_count},
            {"network_decode_successes", camera.network_decode_successes},
            {"network_decode_failures", camera.network_decode_failures},
            {"last_health_message", camera.last_health_message},
            {"last_read_ms", camera.last_read_ms},
            {"actual_width", camera.actual_width},
            {"actual_height", camera.actual_height},
            {"actual_fps", camera.actual_fps},
            {"backend", camera.backend_name},
            {"delivered_frames", camera.delivered_frames},
            {"read_failures", camera.read_failures},
            {"consecutive_read_failures", camera.consecutive_read_failures},
            {"slot_replacements", camera.slot_replacements}
        };
    }


    static nlohmann::json ProfilerStageToJson(const bt::ProfilerStageStats& stage) {
        return {
            {"last_ms", stage.last_ms},
            {"avg_ms", stage.avg_ms},
            {"p95_ms", stage.p95_ms},
            {"max_ms", stage.max_ms},
            {"budget_ms", stage.budget_ms},
            {"over_budget", stage.over_budget}
        };
    }

    static nlohmann::json ProfilerToJson(const bt::ProfilerSnapshot& profiler) {
        return {
            {"sample_count", profiler.sample_count},
            {"bottleneck_stage", profiler.bottleneck_stage},
            {"bottleneck_ratio", profiler.bottleneck_ratio},
            {"any_budget_exceeded", profiler.any_budget_exceeded},
            {"stages", {
                {"total", ProfilerStageToJson(profiler.total)},
                {"capture", ProfilerStageToJson(profiler.capture)},
                {"frame_pair", ProfilerStageToJson(profiler.frame_pair)},
                {"preprocess", ProfilerStageToJson(profiler.preprocess)},
                {"inference", ProfilerStageToJson(profiler.inference)},
                {"onnx", ProfilerStageToJson(profiler.onnx)},
                {"decode", ProfilerStageToJson(profiler.decode)},
                {"pipeline", ProfilerStageToJson(profiler.pipeline)},
                {"solver", ProfilerStageToJson(profiler.solver)},
                {"osc", ProfilerStageToJson(profiler.osc)},
                {"ui_publish", ProfilerStageToJson(profiler.ui_publish)}
            }}
        };
    }

    static nlohmann::json DebugToJson(const bt::DebugSnapshot& debug) {
        const std::string degradation = debug.degradation_mode.empty() ? debug.tracking.degradation_mode : debug.degradation_mode;
        const std::string last_error = debug.last_error.empty() ? debug.tracking.last_error : debug.last_error;
        const bt::TrackingSolverTelemetry solver =
            (debug.solver.used_hmd || debug.solver.degraded || !debug.solver.reason.empty())
                ? debug.solver
                : debug.tracking.solver;
        return {
            {"phase", debug.phase},
            {"degradation_mode", degradation},
            {"last_error", last_error},
            {"inference_ms", debug.inference_ms},
            {"inference_ms_a", debug.inference_ms_a},
            {"inference_ms_b", debug.inference_ms_b},
            {"model_active_device", debug.model_active_device},
            {"model_ep_fallback", debug.model_ep_fallback},
            {"pipeline_ms", debug.pipeline_ms},
            {"capture_ms", debug.capture_ms},
            {"frame_pair_ms", debug.frame_pair_ms},
            {"frame_pairing", {
                {"accepted_pairs", debug.frame_pairing.accepted_pairs},
                {"missing_a", debug.frame_pairing.missing_a},
                {"missing_b", debug.frame_pairing.missing_b},
                {"rejected_skew", debug.frame_pairing.rejected_skew},
                {"rejected_duplicate", debug.frame_pairing.rejected_duplicate},
                {"rejected_reused_a", debug.frame_pairing.rejected_reused_a},
                {"rejected_reused_b", debug.frame_pairing.rejected_reused_b},
                {"degraded_skew", debug.frame_pairing.degraded_skew},
                {"degraded_duplicate", debug.frame_pairing.degraded_duplicate},
                {"degraded_reused_a", debug.frame_pairing.degraded_reused_a},
                {"degraded_reused_b", debug.frame_pairing.degraded_reused_b},
                {"last_accepted_sequence_a", debug.frame_pairing.last_accepted_sequence_a},
                {"last_accepted_sequence_b", debug.frame_pairing.last_accepted_sequence_b},
                {"last_skew_ms", debug.frame_pairing.last_skew_ms},
                {"current_degraded", debug.frame_pair_degraded},
                {"current_reused_a", debug.frame_pair_reused_a},
                {"current_reused_b", debug.frame_pair_reused_b},
                {"current_duplicate", debug.frame_pair_duplicate},
                {"current_skewed", debug.frame_pair_skewed},
                {"current_reason", debug.frame_pair_reason}
            }},
            {"preprocess_ms", debug.preprocess_ms},
            {"preprocess_ms_a", debug.preprocess_ms_a},
            {"preprocess_ms_b", debug.preprocess_ms_b},
            {"onnx_ms", debug.onnx_ms},
            {"onnx_ms_a", debug.onnx_ms_a},
            {"onnx_ms_b", debug.onnx_ms_b},
            {"decode_ms", debug.decode_ms},
            {"decode_ms_a", debug.decode_ms_a},
            {"decode_ms_b", debug.decode_ms_b},
            {"solver_ms", debug.solver_ms},
            {"preliminary_solve_ms", debug.preliminary_solve_ms},
            {"final_solve_ms", debug.final_solve_ms},
            {"osc_ms", debug.osc_ms},
            {"cameras", {
                {"a", CaptureHealthToJson(debug.camera_a, debug.camera_a_frame_age_ms)},
                {"b", CaptureHealthToJson(debug.camera_b, debug.camera_b_frame_age_ms)}
            }},
            {"osc", OscDebugToJson(debug)},
        {"steamvr_bridge", SteamVrBridgeDebugToJson(debug)},
            {"steamvr_alignment", debug.steamvr_alignment_recorded ? bt::AlignmentStatusToJson(debug.steamvr_alignment) : nlohmann::json{{"recorded", false}}},
            {"ui_publish_ms", debug.ui_publish_ms},
            {"total_ms", debug.total_ms},
            {"objective_evaluations", debug.objective_evaluations},
            {"coordinate_passes", debug.coordinate_passes},
            {"optimizer_early_stopped", debug.optimizer_early_stopped},
            {"profiler", ProfilerToJson(debug.profiler)},
            {"posture_mode", bt::ToString(debug.tracking.state.posture_mode)},
            {"root_support", bt::ToString(debug.tracking.state.support.root_support)},
            {"support", {
                {"left_foot", FootSupportUiToJson(debug.tracking.state.support.left_foot)},
                {"right_foot", FootSupportUiToJson(debug.tracking.state.support.right_foot)}
            }},
            {"motion_filter", MotionFilterUiToJson(debug.tracking.motion_filter)},
            {"body_state", BodyStateToJson(debug.tracking.body_state)},
            {"solver", SolverUiToJson(debug, solver)},
            {"stereo", StereoTelemetryToJson(solver.preliminary_stereo)},
            {"floor_assist", FloorAssistTelemetryToJson(solver)},
            {"floor_geometry", FloorGeometryToJson(debug.tracking.floor_geometry)},
            {"body_calibration", BodyCalibrationTelemetryToJson(debug.tracking.body_calibration)},
            {"tracker_ekf", {
                {"left_foot_support_confidence", debug.tracking.tracker_ekf.left_foot.support_confidence},
                {"right_foot_support_confidence", debug.tracking.tracker_ekf.right_foot.support_confidence}
            }}
        };
    }

    std::filesystem::path config_path_;
    std::mutex mutex_;
    bt::AppConfig config_{};
    std::vector<CameraProbe> cameras_;
    bool scanning_ = false;
    std::string camera_status_ = "camera scan not started";
    bool runtime_running_ = false;
    int runtime_exit_code_ = -1;
    std::string runtime_error_;
    std::thread scan_thread_;
    std::thread runtime_thread_;
    bt::WebRuntimeState runtime_state_;
    bt::SteamVrAlignmentManager alignment_manager_{};
};

int RunDesktopMode(const std::filesystem::path& config_path) {
    BodytrackerDesktopController controller(config_path);
    const auto status = bt::RunDesktopUi(controller, "bodytracker");
    if (!status.ok()) {
        std::cerr << status.message << '\n';
        return 1;
    }
    return 0;
}

int main(int argc, char** argv) {
    if (argc >= 2) {
        const std::string mode = argv[1];
        if (mode == "--help" || mode == "-h") {
            PrintUsage();
            return 0;
        }
        if (mode == "--run") {
            const std::filesystem::path config_path =
                argc >= 3 ? std::filesystem::path(argv[2]) : std::filesystem::path("config/default.json");
            return RunRuntime(config_path);
        }
        if (mode == "--setup") {
            const std::filesystem::path config_path =
                argc >= 3 ? std::filesystem::path(argv[2]) : std::filesystem::path("config/default.json");
            return RunDesktopMode(config_path);
        }
        if (mode == "--capture-chessboard") {
            return RunCaptureChessboard(argc, argv);
        }
        if (mode == "--capture-stereo-chessboard") {
            return RunCaptureStereoChessboard(argc, argv);
        }
        if (mode == "--calibrate-intrinsics") {
            return RunCalibrateIntrinsics(argc, argv);
        }
        if (mode == "--calibrate-stereo") {
            return RunCalibrateStereo(argc, argv);
        }
        if (mode == "--calibrate-floor") {
            return RunCalibrateFloor(argc, argv);
        }
        if (mode == "--calibrate-floor-geometry") {
            return RunCalibrateFloorGeometry(argc, argv);
        }
        if (mode == "--align-floor-geometry") {
            return RunAlignFloorGeometry(argc, argv);
        }
        if (mode == "--set-body") {
            return RunSetBody(argc, argv);
        }
        if (mode == "--status") {
            return RunCalibrationStatus(argc, argv);
        }
        if (mode == "--replay-solve") {
            return RunReplaySolve(argc, argv);
        }
        if (mode == "--benchmark-replay") {
            return RunBenchmarkReplay(argc, argv);
        }
        if (!mode.empty() && mode.rfind("--", 0) == 0) {
            PrintUsage();
            return 1;
        }
    }

    const std::filesystem::path config_path =
        argc >= 2 ? std::filesystem::path(argv[1]) : std::filesystem::path("config/default.json");
    return RunDesktopMode(config_path);
}
