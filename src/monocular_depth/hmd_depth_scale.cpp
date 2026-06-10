#include "monocular_depth/hmd_depth_scale.h"

#include "tracking/monocular_projection.h"
#include "tracking/tracking_constants.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace bt {
namespace {

bool FinitePositive(float v) {
    return std::isfinite(v) && v > 0.0f;
}

bool FinitePixel(const Keypoint2D& kp) {
    return kp.present && std::isfinite(kp.pixel.x) && std::isfinite(kp.pixel.y);
}

bool FiniteTransform(const Mat34f& t) {
    return std::all_of(t.m.begin(), t.m.end(), [](float v) { return std::isfinite(v); });
}

float Clamp(float v, float lo, float hi) {
    return std::max(lo, std::min(hi, v));
}

float Median(std::vector<float> values) {
    if (values.empty()) {
        return 0.0f;
    }
    const std::size_t mid = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(mid), values.end());
    float m = values[mid];
    if ((values.size() % 2) == 0) {
        auto lower = values;
        std::nth_element(lower.begin(), lower.begin() + static_cast<std::ptrdiff_t>(mid - 1), lower.end());
        m = 0.5f * (m + lower[mid - 1]);
    }
    return m;
}

float RobustSigma(float value, const std::vector<float>& history) {
    if (history.size() < 5 || !std::isfinite(value)) {
        return 0.0f;
    }
    const float med = Median(history);
    std::vector<float> deviations;
    deviations.reserve(history.size());
    for (float v : history) {
        deviations.push_back(std::abs(v - med));
    }
    const float mad = Median(deviations);
    if (!std::isfinite(mad) || mad < 1e-6f) {
        return 0.0f;
    }
    return std::abs(value - med) / (1.4826f * mad);
}

std::vector<float> RecentLogScales(const HmdDepthScaleHistory& history, int max_count) {
    std::vector<float> out;
    const int count = std::min({history.accepted_count, HmdDepthScaleHistory::kMaxHistory, std::max(0, max_count)});
    out.reserve(static_cast<std::size_t>(count));
    for (int n = 0; n < count; ++n) {
        const int idx = (history.next_index - 1 - n + HmdDepthScaleHistory::kMaxHistory) % HmdDepthScaleHistory::kMaxHistory;
        const float v = history.accepted_log_scale[static_cast<std::size_t>(idx)];
        if (std::isfinite(v)) {
            out.push_back(v);
        }
    }
    return out;
}

std::vector<float> RecentHeadAxis(const HmdDepthScaleHistory& history, int max_count, bool x_axis) {
    std::vector<float> out;
    const int count = std::min({history.accepted_count, HmdDepthScaleHistory::kMaxHistory, std::max(0, max_count)});
    out.reserve(static_cast<std::size_t>(count));
    for (int n = 0; n < count; ++n) {
        const int idx = (history.next_index - 1 - n + HmdDepthScaleHistory::kMaxHistory) % HmdDepthScaleHistory::kMaxHistory;
        const Vec2f v = history.accepted_head_px[static_cast<std::size_t>(idx)];
        const float axis = x_axis ? v.x : v.y;
        if (std::isfinite(axis)) {
            out.push_back(axis);
        }
    }
    return out;
}

HmdDepthScaleResult HeldOrUnavailable(
    HmdDepthScaleStateKind held_state,
    HmdDepthScaleStateKind unavailable_state,
    const char* reason,
    const HmdDepthScaleConfig& config,
    const HmdDepthScaleHistory& history,
    double now_seconds) {

    HmdDepthScaleResult out;
    out.reason = reason ? reason : ToString(unavailable_state);
    const double age = now_seconds - history.last_valid_time_seconds;
    if (history.has_last_valid && std::isfinite(age) && age >= 0.0 && age <= config.max_hold_seconds) {
        out.state = held_state;
        out.held = true;
        out.usable = true;
        out.scale = history.last_scale;
        out.observation.scale = history.last_scale;
        out.observation.hmd_world = history.last_hmd_world;
        out.corrected_root_world = history.last_corrected_root_world;
        out.corrected_root_valid = history.last_corrected_root_valid;
        return out;
    }
    out.state = unavailable_state;
    out.usable = false;
    out.scale = 1.0f;
    return out;
}

bool MeasurementPresentFinite(const MonocularJointMeasurement& m) {
    return m.present && IsFinite(m.world) && FinitePositive(m.estimated_depth_m);
}

} // namespace

Vec3f HmdDepthImageRayCameraUnit(float u, float v, float fx, float fy, float cx, float cy) {
    return NormalizeOr(Vec3f{(u - cx) / fx, (v - cy) / fy, 1.0f}, Vec3f{0.0f, 0.0f, 1.0f});
}

Vec3f HmdDepthCameraToWorldPoint(const Mat34f& t, Vec3f c) {
    return Vec3f{
        t.m[0] * c.x + t.m[1] * c.y + t.m[2] * c.z + t.m[3],
        t.m[4] * c.x + t.m[5] * c.y + t.m[6] * c.z + t.m[7],
        t.m[8] * c.x + t.m[9] * c.y + t.m[10] * c.z + t.m[11]};
}

Vec3f HmdDepthWorldToCameraPoint(const Mat34f& t, Vec3f w) {
    const Vec3f p{w.x - t.m[3], w.y - t.m[7], w.z - t.m[11]};
    return Vec3f{
        t.m[0] * p.x + t.m[4] * p.y + t.m[8] * p.z,
        t.m[1] * p.x + t.m[5] * p.y + t.m[9] * p.z,
        t.m[2] * p.x + t.m[6] * p.y + t.m[10] * p.z};
}

Vec3f HmdDepthBackProjectCamera(float u, float v, float depth_z_m, float fx, float fy, float cx, float cy) {
    return Vec3f{(u - cx) * depth_z_m / fx, (v - cy) * depth_z_m / fy, depth_z_m};
}

HmdDepthScaleResult ComputeHmdDepthScale(
    const HmdDepthScaleConfig& config,
    const KeypointArray& keypoints,
    const CameraCalibration& camera,
    const HmdPoseSample& hmd,
    double camera_timestamp_seconds,
    double now_seconds,
    float mono_head_depth_m,
    const HmdDepthScaleHistory& history) {

    HmdDepthScaleResult out;
    out.state = HmdDepthScaleStateKind::Disabled;
    out.reason = "disabled";
    out.scale = 1.0f;
    if (!config.enabled) {
        return out;
    }

    if (!camera.intrinsics_valid || !camera.extrinsics_valid || !FiniteTransform(camera.world_from_camera)) {
        out.state = HmdDepthScaleStateKind::UnavailableCameraExtrinsics;
        out.reason = "camera extrinsics unavailable";
        return out;
    }

    const float fx = static_cast<float>(camera.camera_matrix[0]);
    const float fy = static_cast<float>(camera.camera_matrix[4]);
    const float cx = static_cast<float>(camera.camera_matrix[2]);
    const float cy = static_cast<float>(camera.camera_matrix[5]);
    if (!FinitePositive(fx) || !FinitePositive(fy) || !std::isfinite(cx) || !std::isfinite(cy)) {
        out.state = HmdDepthScaleStateKind::UnavailableCameraExtrinsics;
        out.reason = "camera intrinsics unavailable";
        return out;
    }

    if (!hmd.valid || !IsFinite(hmd.pose.position)) {
        return HeldOrUnavailable(
            HmdDepthScaleStateKind::HeldHmdTrackingLost,
            HmdDepthScaleStateKind::UnavailableHmdTrackingLost,
            "hmd tracking unavailable",
            config,
            history,
            now_seconds);
    }

    const std::size_t head_index = static_cast<std::size_t>(KeypointId::HeadTop);
    const Keypoint2D& head = keypoints[head_index];
    if (!FinitePixel(head)) {
        return HeldOrUnavailable(
            HmdDepthScaleStateKind::HeldHeadMissing,
            HmdDepthScaleStateKind::UnavailableNoPreviousScale,
            "head keypoint missing",
            config,
            history,
            now_seconds);
    }

    if (!FinitePositive(mono_head_depth_m) || mono_head_depth_m < config.min_depth_m || mono_head_depth_m > config.max_depth_m) {
        return HeldOrUnavailable(
            HmdDepthScaleStateKind::HeldImplausibleScale,
            HmdDepthScaleStateKind::UnavailableNoPreviousScale,
            "monocular head depth implausible",
            config,
            history,
            now_seconds);
    }

    const Vec3f ray = HmdDepthImageRayCameraUnit(head.pixel.x, head.pixel.y, fx, fy, cx, cy);
    const Vec3f q{(head.pixel.x - cx) / fx, (head.pixel.y - cy) / fy, 1.0f};
    const float q_len = Length(q);
    const Vec3f hmd_camera = HmdDepthWorldToCameraPoint(camera.world_from_camera, hmd.pose.position);
    const float lambda_true = Dot(hmd_camera, ray);
    const float true_depth_z = lambda_true / std::max(1e-6f, q_len);
    const float scale = true_depth_z / mono_head_depth_m;

    out.observation.valid = true;
    out.observation.head_keypoint = KeypointId::HeadTop;
    out.observation.head_px = head.pixel;
    out.observation.head_ray_camera_unit = ray;
    out.observation.hmd_world = hmd.pose.position;
    out.observation.hmd_camera = hmd_camera;
    out.observation.mono_head_depth_m = mono_head_depth_m;
    out.observation.true_head_depth_z_m = true_depth_z;
    out.observation.scale = scale;
    out.observation.camera_hmd_timestamp_delta_ms =
        (std::isfinite(camera_timestamp_seconds) && std::isfinite(hmd.timestamp_seconds))
            ? std::abs(camera_timestamp_seconds - hmd.timestamp_seconds) * 1000.0
            : 0.0;

    if (!std::isfinite(true_depth_z) || true_depth_z < config.min_depth_m || true_depth_z > config.max_depth_m ||
        !std::isfinite(scale) || scale < config.min_scale || scale > config.max_scale) {
        return HeldOrUnavailable(
            HmdDepthScaleStateKind::HeldImplausibleScale,
            HmdDepthScaleStateKind::UnavailableNoPreviousScale,
            "scale implausible",
            config,
            history,
            now_seconds);
    }

    const int history_count = std::clamp(config.history_size, 1, HmdDepthScaleHistory::kMaxHistory);
    const float log_scale = std::log(scale);
    const float scale_z = RobustSigma(log_scale, RecentLogScales(history, history_count));
    const float u_z = RobustSigma(head.pixel.x, RecentHeadAxis(history, history_count, true));
    const float v_z = RobustSigma(head.pixel.y, RecentHeadAxis(history, history_count, false));
    if (scale_z > config.outlier_sigma || u_z > config.outlier_sigma || v_z > config.outlier_sigma) {
        return HeldOrUnavailable(
            HmdDepthScaleStateKind::HeldHeadOutlier,
            HmdDepthScaleStateKind::UnavailableNoPreviousScale,
            "head/scale outlier",
            config,
            history,
            now_seconds);
    }

    out.state = HmdDepthScaleStateKind::Live;
    out.reason = "live";
    out.live = true;
    out.usable = true;
    out.scale = scale;
    out.corrected_head_world = hmd.pose.position;
    return out;
}

HmdDepthScaleHistory UpdateHmdDepthScaleHistory(
    const HmdDepthScaleHistory& previous,
    const HmdDepthScaleConfig& config,
    const HmdDepthScaleResult& result,
    double now_seconds) {

    HmdDepthScaleHistory next = previous;
    if (!result.live || !result.usable || !std::isfinite(result.scale) || result.scale <= 0.0f) {
        return next;
    }
    next.has_last_valid = true;
    next.last_scale = result.scale;
    next.last_valid_time_seconds = now_seconds;
    next.last_hmd_world = result.observation.hmd_world;
    next.last_corrected_root_world = result.corrected_root_world;
    next.last_corrected_root_valid = result.corrected_root_valid;

    const int max_history = std::clamp(config.history_size, 1, HmdDepthScaleHistory::kMaxHistory);
    const int idx = next.next_index % HmdDepthScaleHistory::kMaxHistory;
    next.accepted_log_scale[static_cast<std::size_t>(idx)] = std::log(result.scale);
    next.accepted_head_px[static_cast<std::size_t>(idx)] = result.observation.head_px;
    next.next_index = (idx + 1) % HmdDepthScaleHistory::kMaxHistory;
    next.accepted_count = std::min(next.accepted_count + 1, max_history);
    return next;
}

bool ComputeHmdAnchoredRoot(
    const MonocularMeasurementResult& measurements,
    const HmdDepthScaleResult& scale,
    Vec3f* corrected_root_world_out) {

    if (!corrected_root_world_out || !scale.usable || !IsFinite(scale.observation.hmd_world)) {
        return false;
    }
    const auto& head = measurements.joints[static_cast<std::size_t>(KeypointId::HeadTop)];
    if (!MeasurementPresentFinite(head)) {
        return false;
    }

    Vec3f root{};
    bool root_valid = false;
    const auto& pelvis = measurements.joints[static_cast<std::size_t>(KeypointId::Pelvis)];
    if (MeasurementPresentFinite(pelvis)) {
        root = pelvis.world;
        root_valid = true;
    } else {
        const auto& left = measurements.joints[static_cast<std::size_t>(KeypointId::LeftHip)];
        const auto& right = measurements.joints[static_cast<std::size_t>(KeypointId::RightHip)];
        if (MeasurementPresentFinite(left) && MeasurementPresentFinite(right)) {
            root = Scale(Add(left.world, right.world), 0.5f);
            root_valid = true;
        }
    }
    if (!root_valid) {
        return false;
    }
    *corrected_root_world_out = Add(scale.observation.hmd_world, Sub(root, head.world));
    return IsFinite(*corrected_root_world_out);
}

} // namespace bt
