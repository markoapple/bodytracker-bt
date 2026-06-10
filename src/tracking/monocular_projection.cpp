#include "tracking/monocular_projection.h"

#include "inference/keypoint_contract.h"
#include "tracking/tracking_constants.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace bt {
namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kFloorRayMinDeltaYpx = 8.0f;
constexpr float kRadiansToDegrees = 57.29577951308232f;

constexpr std::array<KeypointId, 13> kMonocularLowerBodyKeypoints{
    KeypointId::RightHip,
    KeypointId::RightKnee,
    KeypointId::RightAnkle,
    KeypointId::LeftHip,
    KeypointId::LeftKnee,
    KeypointId::LeftAnkle,
    KeypointId::LeftBigToe,
    KeypointId::LeftSmallToe,
    KeypointId::LeftHeel,
    KeypointId::RightBigToe,
    KeypointId::RightSmallToe,
    KeypointId::RightHeel,
    KeypointId::Pelvis
};

constexpr std::array<KeypointId, 5> kTopKeypoints{
    KeypointId::HeadTop,
    KeypointId::Nose,
    KeypointId::Neck,
    KeypointId::LeftShoulder,
    KeypointId::RightShoulder
};

constexpr std::array<KeypointId, 8> kBottomKeypoints{
    KeypointId::LeftAnkle,
    KeypointId::RightAnkle,
    KeypointId::LeftBigToe,
    KeypointId::RightBigToe,
    KeypointId::LeftSmallToe,
    KeypointId::RightSmallToe,
    KeypointId::LeftHeel,
    KeypointId::RightHeel
};

constexpr std::array<KeypointId, 6> kFloorContactKeypoints{
    KeypointId::LeftBigToe,
    KeypointId::LeftSmallToe,
    KeypointId::LeftHeel,
    KeypointId::RightBigToe,
    KeypointId::RightSmallToe,
    KeypointId::RightHeel
};

float Clamp(float v, float lo, float hi) {
    if (!std::isfinite(v)) {
        return lo;
    }
    return std::max(lo, std::min(hi, v));
}

float Clamp01(float v) {
    return Clamp(v, 0.0f, 1.0f);
}

bool FinitePositive(float v) {
    return std::isfinite(v) && v > 0.0f;
}

bool PixelUsable(const Keypoint2D& kp, float /*weight*/, const MonocularTrackingConfig& /*config*/) {
    return kp.present &&
        std::isfinite(kp.pixel.x) &&
        std::isfinite(kp.pixel.y);
}

bool ManualIntrinsicsUsable(const CameraCalibration& camera) {
    return camera.intrinsics_valid &&
        std::isfinite(camera.camera_matrix[0]) &&
        std::isfinite(camera.camera_matrix[4]) &&
        camera.camera_matrix[0] > 1.0f &&
        camera.camera_matrix[4] > 1.0f;
}

float ConfiguredFocalLengthPx(const MonocularTrackingConfig& config) {
    const float fov = Clamp(
        config.horizontal_fov_deg,
        tracking_constants::kMonocularMinFovDeg,
        tracking_constants::kMonocularMaxFovDeg) * kPi / 180.0f;
    return 0.5f * static_cast<float>(std::max(1, config.image_width)) / std::tan(0.5f * fov);
}

bool IsFloorContactKeypoint(KeypointId id) {
    return id == KeypointId::LeftBigToe ||
        id == KeypointId::LeftSmallToe ||
        id == KeypointId::LeftHeel ||
        id == KeypointId::RightBigToe ||
        id == KeypointId::RightSmallToe ||
        id == KeypointId::RightHeel;
}

bool ScaleSourceUsesFloorContactAssist(MonocularScaleSource source) {
    return source == MonocularScaleSource::FloorSpacing ||
        source == MonocularScaleSource::FloorProjective;
}

bool IsAnkleKeypoint(KeypointId id) {
    return id == KeypointId::LeftAnkle || id == KeypointId::RightAnkle;
}

bool HomographyUsable(const std::array<float, 9>& h) {
    for (const auto v : h) {
        if (!std::isfinite(v)) {
            return false;
        }
    }
    return std::abs(h[0]) + std::abs(h[1]) + std::abs(h[3]) + std::abs(h[4]) > 1e-6f;
}

Vec2f ApplyHomography2D(const std::array<float, 9>& h, const Vec2f& p) {
    const float w = h[6] * p.x + h[7] * p.y + h[8];
    if (!std::isfinite(w) || std::abs(w) <= 1e-6f) {
        return Vec2f{std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN()};
    }
    return Vec2f{
        (h[0] * p.x + h[1] * p.y + h[2]) / w,
        (h[3] * p.x + h[4] * p.y + h[5]) / w};
}

bool FinitePoint(const Vec2f& p) {
    return std::isfinite(p.x) && std::isfinite(p.y);
}

Vec2f CorrectRuntimeDistortedPixel(
    const Vec2f& pixel,
    const MonocularProjectionProfile& profile,
    const MonocularTrackingConfig& config) {

    if (!config.floor_distortion_correction_enabled ||
        config.floor_distortion_confidence < 0.20f ||
        !profile.valid ||
        !std::isfinite(pixel.x) ||
        !std::isfinite(pixel.y)) {
        return pixel;
    }

    const float f = std::max(1.0f, 0.5f * (profile.fx + profile.fy));
    const float x = (pixel.x - profile.cx) / f;
    const float y = (pixel.y - profile.cy) / f;
    const float r2 = x * x + y * y;
    const float radial = 1.0f + config.floor_radial_k1 * r2 + config.floor_radial_k2 * r2 * r2;
    const float xu = x * radial + 2.0f * config.floor_tangential_p1 * x * y + config.floor_tangential_p2 * (r2 + 2.0f * x * x);
    const float yu = y * radial + config.floor_tangential_p1 * (r2 + 2.0f * y * y) + 2.0f * config.floor_tangential_p2 * x * y;
    return Vec2f{profile.cx + f * xu, profile.cy + f * yu};
}

KeypointArray ApplyRuntimeFloorUndistortion(
    const KeypointArray& keypoints,
    const MonocularProjectionProfile& profile,
    const MonocularTrackingConfig& config,
    bool& used) {

    used = false;
    KeypointArray out = keypoints;
    if (!config.floor_distortion_correction_enabled || config.floor_distortion_confidence < 0.20f || !profile.valid) {
        return out;
    }
    for (auto& kp : out) {
        if (!kp.present || !std::isfinite(kp.pixel.x) || !std::isfinite(kp.pixel.y)) {
            continue;
        }
        kp.pixel = CorrectRuntimeDistortedPixel(kp.pixel, profile, config);
        used = true;
    }
    return out;
}

Vec3f ApplyRuntimeFloorOrientationCorrection(
    const Vec3f& world,
    const MonocularTrackingConfig& config,
    bool& used) {

    if (!config.floor_camera_orientation_enabled || config.floor_camera_orientation_confidence < 0.30f) {
        return world;
    }
    const float pitch = config.floor_camera_pitch_rad;
    const float roll = config.floor_camera_roll_rad;
    const float cp = std::cos(pitch);
    const float sp = std::sin(pitch);
    const float cr = std::cos(roll);
    const float sr = std::sin(roll);

    // Small floor-frame correction: pitch rotates the vertical/depth plane,
    // roll rotates the lateral/vertical plane. Use it only when the floor
    // geometry confidence said the camera orientation is actually measured.
    Vec3f p = world;
    p = Vec3f{p.x, cp * p.y - sp * p.z, sp * p.y + cp * p.z};
    p = Vec3f{cr * p.x - sr * p.y, sr * p.x + cr * p.y, p.z};
    used = true;
    return p;
}

float FloorPointPlausibility(float d_m, float expected_m, float tolerance_m) {
    if (!std::isfinite(d_m) || d_m <= 0.0f) {
        return 0.0f;
    }
    return Clamp01(1.0f - std::abs(d_m - expected_m) / std::max(0.01f, tolerance_m));
}

float KneeFlexionDegrees(
    const KeypointArray& keypoints,
    const std::array<float, kHalpe26Count>& weights,
    const MonocularTrackingConfig& config,
    KeypointId hip_id,
    KeypointId knee_id,
    KeypointId ankle_id) {

    const std::size_t hip_index = static_cast<std::size_t>(hip_id);
    const std::size_t knee_index = static_cast<std::size_t>(knee_id);
    const std::size_t ankle_index = static_cast<std::size_t>(ankle_id);
    if (!PixelUsable(keypoints[hip_index], weights[hip_index], config) ||
        !PixelUsable(keypoints[knee_index], weights[knee_index], config) ||
        !PixelUsable(keypoints[ankle_index], weights[ankle_index], config)) {
        return std::numeric_limits<float>::quiet_NaN();
    }

    const Vec2f thigh{
        keypoints[hip_index].pixel.x - keypoints[knee_index].pixel.x,
        keypoints[hip_index].pixel.y - keypoints[knee_index].pixel.y
    };
    const Vec2f shank{
        keypoints[ankle_index].pixel.x - keypoints[knee_index].pixel.x,
        keypoints[ankle_index].pixel.y - keypoints[knee_index].pixel.y
    };
    const float thigh_len = Distance(Vec2f{}, thigh);
    const float shank_len = Distance(Vec2f{}, shank);
    if (thigh_len <= 1.0f || shank_len <= 1.0f) {
        return std::numeric_limits<float>::quiet_NaN();
    }

    const float c = Clamp(
        (thigh.x * shank.x + thigh.y * shank.y) / (thigh_len * shank_len),
        -1.0f,
        1.0f);
    const float joint_angle_deg = std::acos(c) * kRadiansToDegrees;
    return Clamp(180.0f - joint_angle_deg, 0.0f, 180.0f);
}

float MeanObservedKneeFlexionDegrees(
    const KeypointArray& keypoints,
    const std::array<float, kHalpe26Count>& weights,
    const MonocularTrackingConfig& config) {

    const float left = KneeFlexionDegrees(
        keypoints,
        weights,
        config,
        KeypointId::LeftHip,
        KeypointId::LeftKnee,
        KeypointId::LeftAnkle);
    const float right = KneeFlexionDegrees(
        keypoints,
        weights,
        config,
        KeypointId::RightHip,
        KeypointId::RightKnee,
        KeypointId::RightAnkle);
    float sum = 0.0f;
    int count = 0;
    if (std::isfinite(left)) {
        sum += left;
        ++count;
    }
    if (std::isfinite(right)) {
        sum += right;
        ++count;
    }
    return count > 0 ? sum / static_cast<float>(count) : 0.0f;
}

struct DepthEstimate {
    float depth_m = 0.0f;
    float confidence = 0.0f;
    float floor_depth_m = 0.0f;
    float floor_confidence = 0.0f;
    MonocularScaleSource scale_source = MonocularScaleSource::None;
};

struct ProjectiveFloorPose {
    bool valid = false;
    Vec3f axis_width{};
    Vec3f axis_length{};
    Vec3f origin{};
    float quality = 0.0f;
};

Vec3f ApplyInverseIntrinsicsToHomogeneousColumn(
    const MonocularProjectionProfile& profile,
    float u,
    float v,
    float w) {

    return Vec3f{
        (u - profile.cx * w) / profile.fx,
        (v - profile.cy * w) / profile.fy,
        w};
}

ProjectiveFloorPose DecomposeImageFromFloorHomography(
    const MonocularProjectionProfile& profile,
    const MonocularTrackingConfig& config) {

    ProjectiveFloorPose pose;
    if (!profile.valid ||
        !FinitePositive(profile.fx) ||
        !FinitePositive(profile.fy) ||
        !HomographyUsable(config.image_from_floor)) {
        return pose;
    }

    const auto& h = config.image_from_floor;
    const Vec3f c0 = ApplyInverseIntrinsicsToHomogeneousColumn(profile, h[0], h[3], h[6]);
    const Vec3f c1 = ApplyInverseIntrinsicsToHomogeneousColumn(profile, h[1], h[4], h[7]);
    const Vec3f c2 = ApplyInverseIntrinsicsToHomogeneousColumn(profile, h[2], h[5], h[8]);
    const float n0 = Length(c0);
    const float n1 = Length(c1);
    if (!FinitePositive(n0) || !FinitePositive(n1)) {
        return pose;
    }

    float scale = 2.0f / std::max(1e-6f, n0 + n1);
    Vec3f axis_width = Scale(c0, scale);
    Vec3f axis_length = Scale(c1, scale);
    Vec3f origin = Scale(c2, scale);

    // The homography has arbitrary global sign. Keep the camera-depth direction
    // positive so floor points can become real monocular depths.
    if (origin.z < 0.0f) {
        scale = -scale;
        axis_width = Scale(c0, scale);
        axis_length = Scale(c1, scale);
        origin = Scale(c2, scale);
    }

    const float width_norm = Length(axis_width);
    const float length_norm = Length(axis_length);
    const float normal_norm = Length(Cross(axis_width, axis_length));
    if (!FinitePositive(width_norm) || !FinitePositive(length_norm) || normal_norm < 0.10f ||
        !std::isfinite(origin.x) || !std::isfinite(origin.y) || !std::isfinite(origin.z)) {
        return pose;
    }

    const float axis_length_quality = Clamp01(1.0f - 0.5f * (std::abs(width_norm - 1.0f) + std::abs(length_norm - 1.0f)));
    const float orthogonality = std::abs(Dot(axis_width, axis_length)) / std::max(1e-5f, width_norm * length_norm);
    const float orthogonality_quality = Clamp01(1.0f - orthogonality / 0.20f);
    const float front_quality = origin.z > tracking_constants::kMonocularMinDepthM ? 1.0f : 0.0f;

    pose.valid = axis_length_quality > 0.20f && orthogonality_quality > 0.20f && front_quality > 0.0f;
    pose.axis_width = axis_width;
    pose.axis_length = axis_length;
    pose.origin = origin;
    pose.quality = Clamp01(axis_length_quality * orthogonality_quality * front_quality);
    return pose;
}

float ProjectiveFloorDepthForPixel(
    const ProjectiveFloorPose& pose,
    const std::array<float, 9>& floor_from_image,
    const Vec2f& pixel) {

    if (!pose.valid || !HomographyUsable(floor_from_image)) {
        return 0.0f;
    }
    const Vec2f floor_xy = ApplyHomography2D(floor_from_image, pixel);
    if (!FinitePoint(floor_xy)) {
        return 0.0f;
    }
    const Vec3f camera_point = Add(
        Add(Scale(pose.axis_width, floor_xy.x), Scale(pose.axis_length, floor_xy.y)),
        pose.origin);
    if (!std::isfinite(camera_point.z) || camera_point.z <= 0.0f) {
        return 0.0f;
    }
    return Clamp(camera_point.z, tracking_constants::kMonocularMinDepthM, tracking_constants::kMonocularMaxDepthM);
}

float FloorRayDepthForPixelY(const MonocularProjectionProfile& profile, float y_px) {
    const float delta_y = y_px - profile.cy;
    if (!profile.valid || !FinitePositive(profile.camera_height_m) || delta_y <= kFloorRayMinDeltaYpx) {
        return 0.0f;
    }
    return Clamp(
        profile.camera_height_m * profile.fy / delta_y,
        tracking_constants::kMonocularMinDepthM,
        tracking_constants::kMonocularMaxDepthM);
}

DepthEstimate EstimateDepthFromBodyExtent(
    const KeypointArray& keypoints,
    const std::array<float, kHalpe26Count>& weights,
    const MonocularProjectionProfile& profile,
    const MonocularTrackingConfig& config) {

    float top_y = std::numeric_limits<float>::infinity();
    float bottom_y = -std::numeric_limits<float>::infinity();
    bool has_top = false;
    bool has_bottom = false;

    for (const KeypointId id : kTopKeypoints) {
        const std::size_t i = static_cast<std::size_t>(id);
        if (!PixelUsable(keypoints[i], weights[i], config)) {
            continue;
        }
        top_y = std::min(top_y, keypoints[i].pixel.y);
        has_top = true;
    }
    for (const KeypointId id : kBottomKeypoints) {
        const std::size_t i = static_cast<std::size_t>(id);
        if (!PixelUsable(keypoints[i], weights[i], config)) {
            continue;
        }
        bottom_y = std::max(bottom_y, keypoints[i].pixel.y);
        has_bottom = true;
    }

    float pixel_height = 0.0f;
    float metric_height = config.user_height_m;
    float confidence = 0.0f;

    const float knee_flexion_deg = MeanObservedKneeFlexionDegrees(keypoints, weights, config);
    if (has_top && has_bottom) {
        pixel_height = bottom_y - top_y;
        confidence = 1.0f;
        // A crouch/squat shortens the visible top-to-floor silhouette. Treating
        // that compressed silhouette as full standing height makes depth grow,
        // which then makes the whole body get wider instead of lower. Use the
        // observed 2D knee bend as a posture cue and reduce the metric silhouette
        // height before converting pixels to depth.
        const float crouch_t = Clamp01((knee_flexion_deg - 10.0f) / 50.0f);
        if (crouch_t > 0.0f) {
            metric_height *= Lerp(1.0f, 0.58f, crouch_t);
            confidence *= Lerp(1.0f, 0.55f, crouch_t);
        }
    } else {
        const std::size_t pelvis = static_cast<std::size_t>(KeypointId::Pelvis);
        if (PixelUsable(keypoints[pelvis], weights[pelvis], config) && has_bottom) {
            if (knee_flexion_deg <= 55.0f) {
                pixel_height = bottom_y - keypoints[pelvis].pixel.y;
                const float crouch_t = Clamp01((knee_flexion_deg - 10.0f) / 50.0f);
                metric_height = std::max(0.34f, config.user_height_m * Lerp(0.53f, 0.34f, crouch_t));
                confidence = knee_flexion_deg > 15.0f ? 0.35f : 0.70f;
            }
        }
    }

    if (pixel_height >= tracking_constants::kMonocularMinBodyPixelHeight && FinitePositive(metric_height) && FinitePositive(profile.fy)) {
        const float depth = Clamp(
            profile.fy * metric_height / pixel_height,
            tracking_constants::kMonocularMinDepthM,
            tracking_constants::kMonocularMaxDepthM);
        const float extent_quality = Clamp01(pixel_height / (0.40f * static_cast<float>(std::max(1, profile.image_height))));
        return DepthEstimate{depth, Clamp01(confidence * std::max(0.25f, extent_quality)), 0.0f, 0.0f, MonocularScaleSource::BodyExtent};
    }

    if (FinitePositive(config.default_depth_m)) {
        return DepthEstimate{
            Clamp(config.default_depth_m, tracking_constants::kMonocularMinDepthM, tracking_constants::kMonocularMaxDepthM),
            0.30f,
            0.0f,
            0.0f,
            MonocularScaleSource::DefaultDepth};
    }
    return {};
}

DepthEstimate EstimateDepthFromFloorRay(
    const KeypointArray& keypoints,
    const std::array<float, kHalpe26Count>& weights,
    const MonocularProjectionProfile& profile,
    const MonocularTrackingConfig& config) {

    float depth_sum = 0.0f;
    float confidence_sum = 0.0f;
    int count = 0;
    float deepest_y = -std::numeric_limits<float>::infinity();

    for (const KeypointId id : kFloorContactKeypoints) {
        const std::size_t i = static_cast<std::size_t>(id);
        if (!PixelUsable(keypoints[i], weights[i], config)) {
            continue;
        }
        const float depth = FloorRayDepthForPixelY(profile, keypoints[i].pixel.y);
        if (!FinitePositive(depth)) {
            continue;
        }
        const float y_quality = Clamp01((keypoints[i].pixel.y - profile.cy) / (0.45f * static_cast<float>(std::max(1, profile.image_height))));
        const float confidence = Clamp01(weights[i] * keypoints[i].confidence * y_quality);
        depth_sum += confidence * depth;
        confidence_sum += confidence;
        deepest_y = std::max(deepest_y, keypoints[i].pixel.y);
        ++count;
    }

    if (confidence_sum <= 0.0f || count == 0) {
        return {};
    }

    const float count_quality = Clamp01(static_cast<float>(count) / 4.0f);
    const float confidence = Clamp01(0.70f * count_quality * confidence_sum / static_cast<float>(count));
    const float depth = Clamp(depth_sum / confidence_sum, tracking_constants::kMonocularMinDepthM, tracking_constants::kMonocularMaxDepthM);
    return DepthEstimate{depth, confidence, depth, confidence, MonocularScaleSource::FloorRay};
}

DepthEstimate EstimateDepthFromFloorHomography(
    const KeypointArray& keypoints,
    const std::array<float, kHalpe26Count>& weights,
    const MonocularProjectionProfile& profile,
    const MonocularTrackingConfig& config) {

    if (!config.floor_projective_homography_enabled ||
        !HomographyUsable(config.floor_from_image) ||
        !HomographyUsable(config.image_from_floor) ||
        config.floor_projective_confidence <= 0.0f) {
        return {};
    }

    const ProjectiveFloorPose pose = DecomposeImageFromFloorHomography(profile, config);
    if (!pose.valid || pose.quality <= 0.0f) {
        return {};
    }

    float depth_sum = 0.0f;
    float confidence_sum = 0.0f;
    int contact_count = 0;
    float floor_valid_weight = 0.0f;
    float floor_weight_sum = 0.0f;

    for (const KeypointId id : kFloorContactKeypoints) {
        const std::size_t i = static_cast<std::size_t>(id);
        if (!PixelUsable(keypoints[i], weights[i], config)) {
            continue;
        }
        const float w = Clamp01(weights[i] * keypoints[i].confidence);
        floor_weight_sum += w;
        const float projective_depth = ProjectiveFloorDepthForPixel(pose, config.floor_from_image, keypoints[i].pixel);
        if (!FinitePositive(projective_depth)) {
            continue;
        }
        depth_sum += w * projective_depth;
        confidence_sum += w;
        floor_valid_weight += w;
        ++contact_count;
    }

    if (confidence_sum <= 0.0f || contact_count == 0) {
        return {};
    }

    const auto foot_quality = [&](KeypointId heel_id, KeypointId toe_a_id, KeypointId toe_b_id) {
        const auto valid_projected = [&](KeypointId id, Vec2f& out, float& weight) {
            const std::size_t i = static_cast<std::size_t>(id);
            if (!PixelUsable(keypoints[i], weights[i], config)) {
                return false;
            }
            out = ApplyHomography2D(config.floor_from_image, keypoints[i].pixel);
            weight = Clamp01(weights[i] * keypoints[i].confidence);
            return FinitePoint(out) && weight > 0.0f;
        };

        Vec2f heel{}, toe_a{}, toe_b{};
        float wh = 0.0f, wa = 0.0f, wb = 0.0f;
        const bool has_heel = valid_projected(heel_id, heel, wh);
        const bool has_a = valid_projected(toe_a_id, toe_a, wa);
        const bool has_b = valid_projected(toe_b_id, toe_b, wb);
        if (!has_heel || (!has_a && !has_b)) {
            return 0.0f;
        }
        Vec2f toe = has_a && has_b
            ? Vec2f{(wa * toe_a.x + wb * toe_b.x) / std::max(1e-5f, wa + wb),
                    (wa * toe_a.y + wb * toe_b.y) / std::max(1e-5f, wa + wb)}
            : (has_a ? toe_a : toe_b);
        const float heel_to_toe = Distance(heel, toe);
        const float length_quality = FloorPointPlausibility(heel_to_toe, tracking_constants::kDefaultFootLengthM, 0.18f);
        if (has_a && has_b) {
            const float toe_width = Distance(toe_a, toe_b);
            const float width_quality = FloorPointPlausibility(toe_width, 0.08f, 0.12f);
            return Clamp01(0.70f * length_quality + 0.30f * width_quality);
        }
        return length_quality;
    };

    const float left_quality = foot_quality(KeypointId::LeftHeel, KeypointId::LeftBigToe, KeypointId::LeftSmallToe);
    const float right_quality = foot_quality(KeypointId::RightHeel, KeypointId::RightBigToe, KeypointId::RightSmallToe);
    const float geometry_quality = Clamp01(0.5f * (left_quality + right_quality));
    const float finite_quality = Clamp01(floor_valid_weight / std::max(1e-5f, floor_weight_sum));
    const float count_quality = Clamp01(static_cast<float>(contact_count) / 4.0f);
    const float foot_geometry_weight = (left_quality > 0.0f || right_quality > 0.0f)
        ? std::max(0.30f, geometry_quality)
        : 0.0f;
    const float confidence = Clamp01(config.floor_projective_confidence * pose.quality * finite_quality * count_quality * foot_geometry_weight);
    if (confidence <= 0.0f) {
        return {};
    }

    const float depth = Clamp(depth_sum / confidence_sum, tracking_constants::kMonocularMinDepthM, tracking_constants::kMonocularMaxDepthM);
    return DepthEstimate{depth, confidence, depth, confidence, MonocularScaleSource::FloorProjective};
}

DepthEstimate EstimateDepthFromFloorSpacing(
    const KeypointArray& keypoints,
    const std::array<float, kHalpe26Count>& weights,
    const MonocularProjectionProfile& profile,
    const MonocularTrackingConfig& config) {

    if (!config.floor_scale_assist_enabled ||
        !FinitePositive(config.floor_depth_line_spacing_m) ||
        !FinitePositive(config.floor_depth_line_spacing_px)) {
        return {};
    }

    float foot_y_sum = 0.0f;
    float weight_sum = 0.0f;
    int count = 0;
    for (const KeypointId id : kFloorContactKeypoints) {
        const std::size_t i = static_cast<std::size_t>(id);
        if (!PixelUsable(keypoints[i], weights[i], config)) {
            continue;
        }
        const float w = Clamp01(weights[i] * keypoints[i].confidence);
        foot_y_sum += w * keypoints[i].pixel.y;
        weight_sum += w;
        ++count;
    }
    if (weight_sum <= 0.0f || count == 0) {
        return {};
    }

    const float foot_y = foot_y_sum / weight_sum;
    const float ray_depth = FloorRayDepthForPixelY(profile, foot_y);
    if (!FinitePositive(ray_depth)) {
        return {};
    }

    float spacing_depth = 0.0f;
    bool has_projective_reference = false;
    {
        const float reference_y = config.floor_depth_reference_y_px > 0.0f
            ? config.floor_depth_reference_y_px
            : foot_y;
        const float reference_delta_y = reference_y - profile.cy;
        const float foot_delta_y = foot_y - profile.cy;
        if (reference_delta_y > kFloorRayMinDeltaYpx && foot_delta_y > kFloorRayMinDeltaYpx) {
            float reference_depth = FinitePositive(config.floor_depth_reference_m)
                ? Clamp(config.floor_depth_reference_m, tracking_constants::kMonocularMinDepthM, tracking_constants::kMonocularMaxDepthM)
                : 0.0f;

            // If the user marked two neighboring floor-depth seams but did not
            // supply an absolute reference depth, infer that reference depth from
            // perspective: on a flat floor, (y - cy) is proportional to 1 / depth.
            // This avoids the common vibe-coded bug where plank/tile spacing is
            // treated as a linear meters-per-pixel ruler across the whole floor.
            if (!FinitePositive(reference_depth)) {
                const float far_delta_y = reference_delta_y - config.floor_depth_line_spacing_px;
                if (far_delta_y > kFloorRayMinDeltaYpx) {
                    const float depth_ratio = reference_delta_y / far_delta_y;
                    if (depth_ratio > 1.001f) {
                        reference_depth = Clamp(
                            config.floor_depth_line_spacing_m / (depth_ratio - 1.0f),
                            tracking_constants::kMonocularMinDepthM,
                            tracking_constants::kMonocularMaxDepthM);
                    }
                }
            }

            if (FinitePositive(reference_depth)) {
                spacing_depth = Clamp(
                    reference_depth * reference_delta_y / foot_delta_y,
                    tracking_constants::kMonocularMinDepthM,
                    tracking_constants::kMonocularMaxDepthM);
                has_projective_reference = true;
            }
        }
    }

    // A spacing value without an explicit reference seam can still be a local
    // metric observation from a manual plank/tile outline. In that case use the
    // current floor-contact y as the local seam reference, guarded by the same
    // perspective checks above.
    if (!has_projective_reference) {
        return {};
    }

    // The measured floor pitch is a local metric scale observation. Keep the
    // camera-height ray as a sanity anchor, then let the known manual seam/plank width
    // pull depth when a usable explicit or local reference seam exists.
    const float depth = Clamp(
        Lerp(ray_depth, spacing_depth, 0.75f),
        tracking_constants::kMonocularMinDepthM,
        tracking_constants::kMonocularMaxDepthM);
    const float count_quality = Clamp01(static_cast<float>(count) / 4.0f);
    const float spacing_quality = Clamp01(config.floor_depth_line_spacing_px / 80.0f);
    const float confidence = Clamp01(config.floor_depth_confidence * count_quality * std::max(0.35f, spacing_quality));
    return DepthEstimate{depth, confidence, depth, confidence, MonocularScaleSource::FloorSpacing};
}

DepthEstimate EstimateDepthFromWallDepth(const MonocularTrackingConfig& config) {
    if (!config.wall_depth_assist_enabled ||
        !FinitePositive(config.wall_depth_assist_m) ||
        config.wall_depth_assist_confidence <= 0.0f) {
        return {};
    }

    const float depth = Clamp(
        config.wall_depth_assist_m,
        tracking_constants::kMonocularMinDepthM,
        tracking_constants::kMonocularMaxDepthM);
    const float confidence = Clamp01(config.wall_depth_assist_confidence);
    return DepthEstimate{depth, confidence, depth, confidence, MonocularScaleSource::WallDepth};
}

DepthEstimate CombineFloorWithBody(const DepthEstimate& floor, const DepthEstimate& body) {
    if (floor.confidence <= 0.0f) {
        return body;
    }
    if (body.confidence <= 0.0f) {
        return floor;
    }

    DepthEstimate out = floor;
    const float total = std::max(1e-5f, floor.confidence + body.confidence);
    const float base_floor_gain = Clamp(floor.confidence / total, 0.35f, 0.85f);

    // Body extent and floor evidence are two different monocular scale cues. A
    // bad camera-height/floor-ray cue can explode depth from a few pixels of y
    // motion, while a noisy body extent can under-estimate a crouch. When they
    // disagree, keep both but reduce the floor pull instead of letting either cue
    // silently dominate the metric scale.
    const float disagreement = std::abs(floor.depth_m - body.depth_m) / std::max(0.75f, body.depth_m);
    const float agreement = Clamp01(1.0f - (disagreement - 0.20f) / 0.80f);
    const float weak_floor_min_gain = floor.scale_source == MonocularScaleSource::FloorRay ? 0.10f : 0.22f;
    const float floor_gain = Lerp(weak_floor_min_gain, base_floor_gain, agreement);
    out.depth_m = Clamp(
        Lerp(body.depth_m, floor.depth_m, floor_gain),
        tracking_constants::kMonocularMinDepthM,
        tracking_constants::kMonocularMaxDepthM);

    const float disagreement_penalty = 1.0f - 0.45f * Clamp01((disagreement - 0.25f) / 0.75f);
    out.confidence = Clamp01((0.70f * floor.confidence + 0.30f * body.confidence) * disagreement_penalty);
    out.floor_depth_m = floor.depth_m;
    out.floor_confidence = floor.confidence;
    return out;
}

float BoundedLocalDepth(float local_depth_m, float body_depth_m, float absolute_slack_m, float relative_slack) {
    if (!FinitePositive(local_depth_m)) {
        return body_depth_m;
    }
    if (!FinitePositive(body_depth_m)) {
        return Clamp(local_depth_m, tracking_constants::kMonocularMinDepthM, tracking_constants::kMonocularMaxDepthM);
    }
    const float slack = std::max(absolute_slack_m, relative_slack * body_depth_m);
    return Clamp(
        local_depth_m,
        std::max(tracking_constants::kMonocularMinDepthM, body_depth_m - slack),
        std::min(tracking_constants::kMonocularMaxDepthM, body_depth_m + slack));
}

DepthEstimate EstimateMonocularDepth(
    const KeypointArray& keypoints,
    const std::array<float, kHalpe26Count>& weights,
    const MonocularProjectionProfile& profile,
    const MonocularTrackingConfig& config) {

    const auto body = EstimateDepthFromBodyExtent(keypoints, weights, profile, config);
    const auto floor_projective = EstimateDepthFromFloorHomography(keypoints, weights, profile, config);
    if (floor_projective.confidence > 0.0f) {
        return CombineFloorWithBody(floor_projective, body);
    }

    const auto floor_spacing = EstimateDepthFromFloorSpacing(keypoints, weights, profile, config);
    if (floor_spacing.confidence > 0.0f) {
        return CombineFloorWithBody(floor_spacing, body);
    }

    const auto wall_depth = EstimateDepthFromWallDepth(config);
    if (wall_depth.confidence > 0.0f) {
        return CombineFloorWithBody(wall_depth, body);
    }

    const auto floor_ray = EstimateDepthFromFloorRay(keypoints, weights, profile, config);
    if (floor_ray.confidence > 0.0f) {
        return CombineFloorWithBody(floor_ray, body);
    }
    return body;
}

float JointDepthForPixel(
    KeypointId id,
    const MonocularProjectionProfile& profile,
    const Keypoint2D& kp,
    const DepthEstimate& depth) {

    if (IsFloorContactKeypoint(id)) {
        if ((depth.scale_source == MonocularScaleSource::FloorSpacing ||
             depth.scale_source == MonocularScaleSource::FloorProjective) &&
            FinitePositive(depth.floor_depth_m)) {
            return BoundedLocalDepth(depth.floor_depth_m, depth.depth_m, 0.45f, 0.22f);
        }
        const float foot_depth = FloorRayDepthForPixelY(profile, kp.pixel.y);
        if (FinitePositive(foot_depth)) {
            const float bounded = BoundedLocalDepth(foot_depth, depth.depth_m, 0.45f, 0.24f);
            const float local_gain = depth.scale_source == MonocularScaleSource::FloorRay ? 0.55f : 0.35f;
            return Clamp(
                Lerp(depth.depth_m, bounded, local_gain),
                tracking_constants::kMonocularMinDepthM,
                tracking_constants::kMonocularMaxDepthM);
        }
    }
    if (IsAnkleKeypoint(id) && FinitePositive(depth.floor_depth_m)) {
        const float bounded_floor = BoundedLocalDepth(depth.floor_depth_m, depth.depth_m, 0.50f, 0.25f);
        return Clamp(
            Lerp(depth.depth_m, bounded_floor, 0.35f),
            tracking_constants::kMonocularMinDepthM,
            tracking_constants::kMonocularMaxDepthM);
    }
    return depth.depth_m;
}

} // namespace

MonocularProjectionProfile MakeMonocularProjectionProfile(
    const CameraCalibration& manual_camera,
    const MonocularTrackingConfig& config) {

    MonocularProjectionProfile profile;
    profile.image_width = config.image_width;
    profile.image_height = config.image_height;
    profile.camera_height_m = config.camera_height_m;

    if (profile.image_width <= 0 || profile.image_height <= 0 ||
        !std::isfinite(profile.camera_height_m)) {
        return profile;
    }

    if (ManualIntrinsicsUsable(manual_camera)) {
        profile.fx = static_cast<float>(manual_camera.camera_matrix[0]);
        profile.fy = static_cast<float>(manual_camera.camera_matrix[4]);
        profile.cx = std::isfinite(manual_camera.camera_matrix[2])
            ? static_cast<float>(manual_camera.camera_matrix[2])
            : 0.5f * static_cast<float>(profile.image_width);
        profile.cy = std::isfinite(manual_camera.camera_matrix[5])
            ? static_cast<float>(manual_camera.camera_matrix[5])
            : 0.5f * static_cast<float>(profile.image_height);
    } else {
        profile.fx = ConfiguredFocalLengthPx(config);
        profile.fy = profile.fx;
        profile.cx = 0.5f * static_cast<float>(profile.image_width);
        profile.cy = 0.5f * static_cast<float>(profile.image_height);
    }

    profile.valid = FinitePositive(profile.fx) && FinitePositive(profile.fy) &&
        std::isfinite(profile.cx) && std::isfinite(profile.cy);
    return profile;
}

CameraCalibration MakeMonocularCameraCalibration(
    const CameraCalibration& manual_camera,
    const MonocularTrackingConfig& config) {

    const auto profile = MakeMonocularProjectionProfile(manual_camera, config);

    CameraCalibration out = manual_camera;
    if (!profile.valid) {
        return out;
    }

    out.intrinsics_valid = true;
    out.extrinsics_valid = true;
    out.camera_matrix = {
        profile.fx, 0.0, profile.cx,
        0.0, profile.fy, profile.cy,
        0.0, 0.0, 1.0
    };
    if (!manual_camera.intrinsics_valid) {
        out.distortion = {0.0, 0.0, 0.0, 0.0, 0.0};
    }

    // Virtual markerless camera: world x right, y up, z forward. The projection
    // uses camera_height_m as the vertical camera offset above the floor.
    if (!manual_camera.extrinsics_valid) {
        out.extrinsics_valid = true;
        out.image_from_world = {
            profile.fx, 0.0f, profile.cx, 0.0f,
            0.0f, -profile.fy, profile.cy, profile.fy * profile.camera_height_m,
            0.0f, 0.0f, 1.0f, 0.0f
        };
        out.world_from_camera = {
            1.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 1.0f, 0.0f, profile.camera_height_m,
            0.0f, 0.0f, 1.0f, 0.0f
        };
    }
    return out;
}

FloorPlane MakeMonocularFloorPlane(const MonocularTrackingConfig& config) {
    FloorPlane floor;
    floor.normal = Vec3f{0.0f, 1.0f, 0.0f};
    floor.distance = 0.0f;
    // camera_height_m can be negative (camera below floor level, e.g. floor-level mount).
    // Only reject NaN/Inf. The sign matters for back-projection but the floor plane
    // itself (normal, distance) is still valid as a constraint.
    floor.valid = std::isfinite(config.camera_height_m);
    return floor;
}

Result<Vec3f> BackProjectMonocularPixel(
    const MonocularProjectionProfile& profile,
    const Vec2f& pixel,
    float depth_m) {

    if (!profile.valid || !FinitePositive(depth_m) ||
        !std::isfinite(pixel.x) || !std::isfinite(pixel.y)) {
        return Status::Error(StatusCode::InvalidArgument, "Invalid monocular projection input");
    }

    const float x = (pixel.x - profile.cx) * depth_m / profile.fx;
    // Y = camera_height_m - (pixel.y - cy) * depth_m / fy
    // camera_height_m is the vertical offset of the camera above (positive) or
    // below (negative) the floor. The minus here corrects for image y increasing
    // downward while world Y increases upward.
    const float y = profile.camera_height_m - (pixel.y - profile.cy) * depth_m / profile.fy;
    return Vec3f{x, y, depth_m};
}

CameraCalibration MakeMonocularAnchorCameraCalibration(
    const CameraCalibration& manual_camera,
    const MonocularProjectionProfile& profile) {

    CameraCalibration out = manual_camera;
    if (!profile.valid) {
        return out;
    }
    out.intrinsics_valid = true;
    out.extrinsics_valid = true;
    out.camera_matrix = {
        profile.fx, 0.0, profile.cx,
        0.0, profile.fy, profile.cy,
        0.0, 0.0, 1.0
    };
    out.image_from_world = {
        profile.fx, 0.0f, profile.cx, 0.0f,
        0.0f, -profile.fy, profile.cy, profile.fy * profile.camera_height_m,
        0.0f, 0.0f, 1.0f, 0.0f
    };
    out.world_from_camera = {
        1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, profile.camera_height_m,
        0.0f, 0.0f, 1.0f, 0.0f
    };
    return out;
}

Result<MonocularMeasurementResult> BuildMonocularJointMeasurements(
    const KeypointArray& keypoints,
    const std::array<float, kHalpe26Count>& reliability_weights,
    const CameraCalibration& manual_camera,
    const MonocularTrackingConfig& config,
    const HmdDepthScaleRuntimeInput* hmd_depth_scale,
    const AnchorSpaceMappingRuntimeInput* anchor_mapping) {

    const auto profile = MakeMonocularProjectionProfile(manual_camera, config);
    if (!profile.valid) {
        return Status::Error(StatusCode::ValidationError, "Monocular camera profile is invalid");
    }

    bool distortion_used = false;
    const KeypointArray working_keypoints = ApplyRuntimeFloorUndistortion(
        keypoints,
        profile,
        config,
        distortion_used);

    const auto depth = EstimateMonocularDepth(working_keypoints, reliability_weights, profile, config);
    if (!FinitePositive(depth.depth_m) || !std::isfinite(depth.confidence) || depth.confidence < 0.0f) {
        return Status::Error(StatusCode::FailedPrecondition, "Monocular depth could not be estimated from current keypoints");
    }

    MonocularMeasurementResult result;
    result.estimated_depth_m = depth.depth_m;
    result.depth_confidence = Clamp01(depth.confidence);
    result.depth_source = DepthSource::InferredMonocular;
    result.scale_source = depth.scale_source;
    result.floor_assist_depth_m = depth.floor_depth_m;
    result.floor_assist_confidence = Clamp01(depth.floor_confidence);
    result.distortion_correction_used = distortion_used;
    result.camera_orientation_correction_used = false;

    float mono_head_depth_m = 0.0f;
    const std::size_t head_index = static_cast<std::size_t>(KeypointId::HeadTop);
    if (head_index < working_keypoints.size() && working_keypoints[head_index].present &&
        std::isfinite(working_keypoints[head_index].pixel.x) &&
        std::isfinite(working_keypoints[head_index].pixel.y)) {
        mono_head_depth_m = JointDepthForPixel(KeypointId::HeadTop, profile, working_keypoints[head_index], depth);
    }
    if (hmd_depth_scale && hmd_depth_scale->enabled) {
        result.hmd_depth_scale = ComputeHmdDepthScale(
            hmd_depth_scale->config,
            working_keypoints,
            manual_camera,
            hmd_depth_scale->hmd,
            hmd_depth_scale->camera_timestamp_seconds,
            hmd_depth_scale->now_seconds,
            mono_head_depth_m,
            hmd_depth_scale->history);
    } else {
        result.hmd_depth_scale.state = HmdDepthScaleStateKind::Disabled;
        result.hmd_depth_scale.reason = "disabled";
    }
    const bool hmd_scale_usable = result.hmd_depth_scale.usable &&
        std::isfinite(result.hmd_depth_scale.scale) && result.hmd_depth_scale.scale > 0.0f &&
        manual_camera.extrinsics_valid;
    if (hmd_scale_usable) {
        result.scale_source = MonocularScaleSource::BodyExtent;
        result.estimated_depth_m = depth.depth_m * result.hmd_depth_scale.scale;
    }

    float confidence_sum = 0.0f;
    int lower_body_valid_count = 0;
    RawAnchorWorlds raw_anchor_worlds{};
    for (const KeypointId id : kInternalKeypointOrder) {
        const std::size_t i = static_cast<std::size_t>(id);
        const float weight = Clamp01(reliability_weights[i]);
        if (!PixelUsable(working_keypoints[i], weight, config)) {
            continue;
        }

        const float raw_joint_depth = JointDepthForPixel(id, profile, working_keypoints[i], depth);
        const float joint_depth = hmd_scale_usable
            ? raw_joint_depth * result.hmd_depth_scale.scale
            : raw_joint_depth;
        Result<Vec3f> world = Status::Error(StatusCode::InvalidArgument, "Invalid monocular projection input");
        if (hmd_scale_usable) {
            const Vec3f camera_point = HmdDepthBackProjectCamera(
                working_keypoints[i].pixel.x,
                working_keypoints[i].pixel.y,
                joint_depth,
                profile.fx,
                profile.fy,
                profile.cx,
                profile.cy);
            if (IsFinite(camera_point)) {
                world = HmdDepthCameraToWorldPoint(manual_camera.world_from_camera, camera_point);
            }
        } else {
            world = BackProjectMonocularPixel(profile, working_keypoints[i].pixel, joint_depth);
        }
        if (!world.ok()) {
            continue;
        }

        const bool floor_contact_assist = ScaleSourceUsesFloorContactAssist(result.scale_source) &&
            result.floor_assist_confidence > 0.0f;
        const float floor_contact_bonus = IsFloorContactKeypoint(id) && floor_contact_assist ? 1.10f : 1.0f;
        // Confidence is output quality, not the existence of a finite back-projected
        // 2D->3D measurement. Keep zero-confidence points so an operator-set output
        // floor of 0 can still see degraded geometry instead of a silent dropout.
        const float confidence = Clamp01(weight * working_keypoints[i].confidence * config.depth_confidence_scale * result.depth_confidence * floor_contact_bonus);

        bool orientation_used = false;
        auto& out = result.joints[i];
        out.world = hmd_scale_usable
            ? world.value()
            : ApplyRuntimeFloorOrientationCorrection(world.value(), config, orientation_used);
        result.camera_orientation_correction_used = result.camera_orientation_correction_used || orientation_used;
        if (!hmd_scale_usable && IsFloorContactKeypoint(id) && floor_contact_assist && FinitePositive(result.floor_assist_depth_m)) {
            out.world.y = 0.0f;
        }
        out.confidence = confidence;
        out.estimated_depth_m = joint_depth;
        out.present = true;
        CollectRawAnchorWorld(id, out.world, confidence, out.present, raw_anchor_worlds);
        confidence_sum += confidence;
        ++result.valid_count;
        if (IsLowerBodyKeypoint(id)) {
            ++lower_body_valid_count;
        }
    }

    result.anchor_raw_worlds = raw_anchor_worlds;
    if (anchor_mapping && anchor_mapping->config.enabled) {
        const CameraCalibration anchor_camera = hmd_scale_usable
            ? manual_camera
            : MakeMonocularAnchorCameraCalibration(manual_camera, profile);
        result.anchor_space_mapping = EstimateAnchorProjectionCorrection(
            anchor_mapping->config,
            anchor_camera,
            working_keypoints,
            raw_anchor_worlds,
            anchor_mapping->anchors,
            profile.image_width,
            profile.image_height,
            anchor_mapping->camera_timestamp_seconds);
        result.anchor_correction_debug = ApplyAnchorProjectionCorrectionToRawWorlds(
            anchor_camera,
            result.anchor_space_mapping,
            raw_anchor_worlds);
        for (std::size_t i = 0; i < result.joints.size(); ++i) {
            auto& joint = result.joints[i];
            if (!joint.present || !result.anchor_correction_debug.applied[i]) {
                continue;
            }
            joint.world = result.anchor_correction_debug.corrected_worlds[i].world;
            const float corrected_depth = result.anchor_correction_debug.corrected_depths[i];
            if (std::isfinite(corrected_depth) && corrected_depth > 0.0f) {
                joint.estimated_depth_m = corrected_depth;
            }
        }
        result.room_depth_map = UpdateRoomDepthMapTelemetry(
            anchor_mapping->room_map,
            anchor_mapping->room_map_config,
            result.anchor_space_mapping,
            anchor_mapping->now_seconds);
    }

    if (lower_body_valid_count < std::max(1, config.min_seed_count)) {
        return Status::Error(
            StatusCode::FailedPrecondition,
            "Monocular lower-body solve rejected: not enough usable lower-body 2D keypoints");
    }

    result.mean_confidence = confidence_sum / static_cast<float>(result.valid_count);
    if (result.hmd_depth_scale.usable) {
        Vec3f corrected_root{};
        if (ComputeHmdAnchoredRoot(result, result.hmd_depth_scale, &corrected_root)) {
            result.hmd_depth_scale.corrected_root_world = corrected_root;
            result.hmd_depth_scale.corrected_root_valid = true;
        }
    }
    if (hmd_depth_scale && hmd_depth_scale->enabled) {
        result.hmd_depth_scale_history = UpdateHmdDepthScaleHistory(
            hmd_depth_scale->history,
            hmd_depth_scale->config,
            result.hmd_depth_scale,
            hmd_depth_scale->now_seconds);
    }
    return result;
}

} // namespace bt
