#include "tracking/body_solver.h"

#include "core/timing.h"
#include "inference/keypoint_contract.h"
#include "tracking/contact_constraints.h"
#include "tracking/foot_frame.h"
#include "tracking/geometry_cache.h"
#include "tracking/joint_limits.h"
#include "tracking/monocular_projection.h"
#include "tracking/measurement_weighting.h"
#include "tracking/support_queries.h"
#include "tracking/triangulation.h"

#include <algorithm>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <array>
#include <cmath>
#include <initializer_list>
#include <limits>
#include <string>
#include <vector>

namespace bt {
namespace {

struct WeightedJointSeed {
    Vec3f world{};
    float weight = 0.0f;
    float reprojection_error_px = 0.0f;
    bool valid = false;
    bool use_uncertainty_weighting = false;
    SolverMeasurementUncertainty uncertainty{};
    SolverObservationInformation solver_information{};
};

bool SeedHasFinitePose(const WeightedJointSeed& seed) {
    return seed.valid && IsFinite(seed.world);
}

bool SeedHasSolverWeight(const WeightedJointSeed& seed) {
    return SeedHasFinitePose(seed) && seed.weight > 0.0f;
}

bool StereoHeadRootOffsetFromTelemetry(
    const BodySolveStereoTelemetry& telemetry,
    Vec3f* head_world_out,
    Vec3f* root_minus_head_out) {

    if (!head_world_out || !root_minus_head_out) {
        return false;
    }
    const auto& head = telemetry.joints[static_cast<std::size_t>(KeypointId::HeadTop)];
    if (!head.triangulated || !IsFinite(head.world)) {
        return false;
    }
    Vec3f root{};
    bool root_valid = false;
    const auto& pelvis = telemetry.joints[static_cast<std::size_t>(KeypointId::Pelvis)];
    if (pelvis.triangulated && IsFinite(pelvis.world)) {
        root = pelvis.world;
        root_valid = true;
    } else {
        const auto& left = telemetry.joints[static_cast<std::size_t>(KeypointId::LeftHip)];
        const auto& right = telemetry.joints[static_cast<std::size_t>(KeypointId::RightHip)];
        if (left.triangulated && right.triangulated && IsFinite(left.world) && IsFinite(right.world)) {
            root = Scale(Add(left.world, right.world), 0.5f);
            root_valid = true;
        }
    }
    if (!root_valid) {
        return false;
    }
    *head_world_out = head.world;
    *root_minus_head_out = Sub(root, head.world);
    return IsFinite(*root_minus_head_out);
}

HmdDepthScaleResult ComputeStereoHmdDepthScale(
    const BodySolveInputs& inputs,
    const BodySolveStereoTelemetry& telemetry) {

    HmdDepthScaleResult out;
    out.state = inputs.hmd_depth_scale.enabled
        ? HmdDepthScaleStateKind::UnavailableNoPreviousScale
        : HmdDepthScaleStateKind::Disabled;
    out.reason = inputs.hmd_depth_scale.enabled ? "stereo hmd depth scale not computed" : "disabled";
    out.scale = 1.0f;
    if (!inputs.hmd_depth_scale.enabled) {
        return out;
    }
    const auto& config = inputs.hmd_depth_scale.config;
    if (!inputs.camera_a_calibration.intrinsics_valid ||
        !inputs.camera_a_calibration.extrinsics_valid) {
        out.state = HmdDepthScaleStateKind::UnavailableCameraExtrinsics;
        out.reason = "camera A extrinsics unavailable";
        return out;
    }
    if (!inputs.hmd.valid || !IsFinite(inputs.hmd.pose.position)) {
        out.state = HmdDepthScaleStateKind::UnavailableHmdTrackingLost;
        out.reason = "hmd tracking unavailable";
        return out;
    }
    const std::size_t head_index = static_cast<std::size_t>(KeypointId::HeadTop);
    const auto& head = telemetry.joints[head_index];
    if (!head.triangulated || !IsFinite(head.world)) {
        out.state = HmdDepthScaleStateKind::HeldHeadMissing;
        out.reason = "stereo head triangulation unavailable";
        return out;
    }

    const Vec3f stereo_head_camera = HmdDepthWorldToCameraPoint(
        inputs.camera_a_calibration.world_from_camera,
        head.world);
    const Vec3f hmd_camera = HmdDepthWorldToCameraPoint(
        inputs.camera_a_calibration.world_from_camera,
        inputs.hmd.pose.position);
    const float stereo_head_depth_z = stereo_head_camera.z;
    const float true_head_depth_z = hmd_camera.z;
    const float scale = true_head_depth_z / stereo_head_depth_z;

    out.observation.valid = true;
    out.observation.head_keypoint = KeypointId::HeadTop;
    out.observation.head_px = inputs.camera_a_pose.keypoints[head_index].pixel;
    const float fx = static_cast<float>(inputs.camera_a_calibration.camera_matrix[0]);
    const float fy = static_cast<float>(inputs.camera_a_calibration.camera_matrix[4]);
    const float cx = static_cast<float>(inputs.camera_a_calibration.camera_matrix[2]);
    const float cy = static_cast<float>(inputs.camera_a_calibration.camera_matrix[5]);
    if (std::isfinite(fx) && std::isfinite(fy) && fx > 0.0f && fy > 0.0f &&
        std::isfinite(out.observation.head_px.x) && std::isfinite(out.observation.head_px.y)) {
        out.observation.head_ray_camera_unit = HmdDepthImageRayCameraUnit(
            out.observation.head_px.x,
            out.observation.head_px.y,
            fx,
            fy,
            cx,
            cy);
    }
    out.observation.hmd_world = inputs.hmd.pose.position;
    out.observation.hmd_camera = hmd_camera;
    out.observation.mono_head_depth_m = stereo_head_depth_z;
    out.observation.true_head_depth_z_m = true_head_depth_z;
    out.observation.scale = scale;
    out.observation.camera_hmd_timestamp_delta_ms =
        (std::isfinite(inputs.stereo_hmd_depth_scale_camera_timestamp_seconds) && std::isfinite(inputs.hmd.timestamp_seconds))
            ? std::abs(inputs.stereo_hmd_depth_scale_camera_timestamp_seconds - inputs.hmd.timestamp_seconds) * 1000.0
            : 0.0;

    if (!std::isfinite(stereo_head_depth_z) || !std::isfinite(true_head_depth_z) ||
        stereo_head_depth_z < config.min_depth_m || stereo_head_depth_z > config.max_depth_m ||
        true_head_depth_z < config.min_depth_m || true_head_depth_z > config.max_depth_m ||
        !std::isfinite(scale) || scale < config.min_scale || scale > config.max_scale) {
        out.state = HmdDepthScaleStateKind::HeldImplausibleScale;
        out.reason = "stereo hmd depth scale implausible";
        return out;
    }

    out.state = HmdDepthScaleStateKind::Live;
    out.reason = "live stereo hmd depth scale";
    out.live = true;
    out.usable = true;
    out.scale = scale;
    out.corrected_head_world = inputs.hmd.pose.position;
    return out;
}

void ApplyStereoHmdDepthScale(
    const BodySolveInputs& inputs,
    const HmdDepthScaleResult& scale,
    std::array<WeightedJointSeed, kHalpe26Count>& seeds,
    BodySolveStereoTelemetry& telemetry) {

    if (!scale.usable || !std::isfinite(scale.scale) || scale.scale <= 0.0f ||
        !inputs.camera_a_calibration.extrinsics_valid) {
        return;
    }
    for (const KeypointId id : kInternalKeypointOrder) {
        const std::size_t i = static_cast<std::size_t>(id);
        auto& joint = telemetry.joints[i];
        if (!joint.triangulated || !IsFinite(joint.world)) {
            continue;
        }
        const Vec3f camera_point = HmdDepthWorldToCameraPoint(
            inputs.camera_a_calibration.world_from_camera,
            joint.world);
        if (!IsFinite(camera_point)) {
            continue;
        }
        const Vec3f scaled_camera{camera_point.x, camera_point.y, camera_point.z * scale.scale};
        const Vec3f scaled_world = HmdDepthCameraToWorldPoint(
            inputs.camera_a_calibration.world_from_camera,
            scaled_camera);
        if (!IsFinite(scaled_world)) {
            continue;
        }
        joint.world = scaled_world;
        joint.estimated_depth_m = scaled_camera.z;
        joint.measurement_mean_depth_m = std::isfinite(joint.measurement_mean_depth_m)
            ? joint.measurement_mean_depth_m * scale.scale
            : scaled_camera.z;
        if (seeds[i].valid) {
            seeds[i].world = scaled_world;
            seeds[i].weight = 1.0f;
        }
    }
}

StereoHmdAnchorResult ComputeStereoHmdAnchor(
    const BodySolveInputs& inputs,
    const BodySolveStereoTelemetry& telemetry) {

    StereoHmdAnchorResult out;
    out.interval_seconds = inputs.stereo_hmd_anchor_interval_seconds;
    out.seconds_since_last_anchor = inputs.stereo_hmd_anchor_seconds_since_last;
    if (!inputs.stereo_hmd_anchor_enabled) {
        out.state = StereoHmdAnchorStateKind::Disabled;
        out.reason = "disabled";
        return out;
    }
    if (!inputs.stereo_hmd_anchor_due) {
        out.state = StereoHmdAnchorStateKind::WaitingInterval;
        out.reason = "waiting for 5-second stereo hmd anchor interval";
        return out;
    }
    out.due = true;
    if (!inputs.hmd.valid || !IsFinite(inputs.hmd.pose.position)) {
        out.state = StereoHmdAnchorStateKind::UnavailableHmdTrackingLost;
        out.reason = "hmd tracking unavailable";
        return out;
    }
    Vec3f stereo_head{};
    Vec3f root_minus_head{};
    if (!StereoHeadRootOffsetFromTelemetry(telemetry, &stereo_head, &root_minus_head)) {
        out.state = StereoHmdAnchorStateKind::UnavailableStereoHeadMissing;
        out.reason = "stereo head/root geometry unavailable";
        out.hmd_world = inputs.hmd.pose.position;
        return out;
    }
    out.hmd_world = inputs.hmd.pose.position;
    out.stereo_head_world = stereo_head;
    out.correction_world = Sub(inputs.hmd.pose.position, stereo_head);
    out.correction_m = Length(out.correction_world);
    if (!std::isfinite(out.correction_m) || out.correction_m > inputs.stereo_hmd_anchor_max_correction_m) {
        out.state = StereoHmdAnchorStateKind::UnavailableImplausibleCorrection;
        out.reason = "stereo hmd anchor correction implausible";
        return out;
    }
    out.corrected_root_world = Add(inputs.hmd.pose.position, root_minus_head);
    if (!IsFinite(out.corrected_root_world)) {
        out.state = StereoHmdAnchorStateKind::UnavailableImplausibleCorrection;
        out.reason = "stereo hmd corrected root non-finite";
        return out;
    }
    out.state = StereoHmdAnchorStateKind::Applied;
    out.reason = "5-second stereo hmd world anchor applied";
    out.applied = true;
    return out;
}

struct PreparedObservedKeypoints {
    std::array<Vec2f, kHalpe26Count> pixels{};
    std::array<bool, kHalpe26Count> usable{};
};

struct SolverParams {
    float root_x = 0.0f;
    float root_y = 0.0f;
    float root_z = 0.0f;
    float root_yaw = 0.0f;
    float root_pitch = 0.0f;
    float root_roll = 0.0f;
    float left_foot_x = 0.0f;
    float left_foot_y = 0.0f;
    float left_foot_z = 0.0f;
    float left_foot_yaw = 0.0f;
    float left_foot_pitch = 0.0f;
    float left_foot_roll = 0.0f;
    float left_bend_hint = 0.0f;
    float right_foot_x = 0.0f;
    float right_foot_y = 0.0f;
    float right_foot_z = 0.0f;
    float right_foot_yaw = 0.0f;
    float right_foot_pitch = 0.0f;
    float right_foot_roll = 0.0f;
    float right_bend_hint = 0.0f;
};

constexpr std::array<float SolverParams::*, 20> kSolverParamMembers{
    &SolverParams::root_x,
    &SolverParams::root_y,
    &SolverParams::root_z,
    &SolverParams::root_yaw,
    &SolverParams::root_pitch,
    &SolverParams::root_roll,
    &SolverParams::left_foot_x,
    &SolverParams::left_foot_y,
    &SolverParams::left_foot_z,
    &SolverParams::left_foot_yaw,
    &SolverParams::left_foot_pitch,
    &SolverParams::left_foot_roll,
    &SolverParams::left_bend_hint,
    &SolverParams::right_foot_x,
    &SolverParams::right_foot_y,
    &SolverParams::right_foot_z,
    &SolverParams::right_foot_yaw,
    &SolverParams::right_foot_pitch,
    &SolverParams::right_foot_roll,
    &SolverParams::right_bend_hint
};
constexpr std::size_t kSolverParamCount = kSolverParamMembers.size();
static_assert(kSolverParamCount == 20, "Solver parameter mapping must cover root, constrained foot targets and bend-plane hints");
constexpr float kOptimizerStepDecay = 0.58f; // Shrinks coordinate-search step sizes between passes and after repeated inactive DOFs.

struct CandidateState {
    LowerBodyState state{};
    LowerBodyJointSet joints{};
};

struct ObjectiveResult {
    float score = 0.0f;
    CandidateState candidate{};
};

struct OptimizerResult {
    CandidateState best{};
    BodySolveTelemetry telemetry{};
};

constexpr std::array<KeypointId, 13> kSolverKeypoints{
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

std::size_t SolverKeypointIndex(KeypointId id) {
    return static_cast<std::size_t>(id);
}

bool IsLeftFootKeypoint(KeypointId id) {
    return id == KeypointId::LeftAnkle ||
        id == KeypointId::LeftBigToe ||
        id == KeypointId::LeftSmallToe ||
        id == KeypointId::LeftHeel;
}

bool IsRightFootKeypoint(KeypointId id) {
    return id == KeypointId::RightAnkle ||
        id == KeypointId::RightBigToe ||
        id == KeypointId::RightSmallToe ||
        id == KeypointId::RightHeel;
}

bool IsSolverFootKeypoint(KeypointId id) {
    return IsLeftFootKeypoint(id) || IsRightFootKeypoint(id);
}

// Contact evidence must come from actual floor-contact landmarks. Ankles are
// solver foot keypoints, but treating an isolated ankle as contact evidence
// lets a partially occluded foot become "planted" without any heel/toe support.
constexpr std::array<KeypointId, 3> kLeftFootContactKeypoints{
    KeypointId::LeftHeel,
    KeypointId::LeftBigToe,
    KeypointId::LeftSmallToe
};

constexpr std::array<KeypointId, 3> kRightFootContactKeypoints{
    KeypointId::RightHeel,
    KeypointId::RightBigToe,
    KeypointId::RightSmallToe
};

constexpr float kLowResFootSeparationSoftPx = 4.0f;

constexpr std::size_t kFootContactKeypointCount = kLeftFootContactKeypoints.size();
static_assert(kFootContactKeypointCount == kRightFootContactKeypoints.size(), "Foot contact summaries must use symmetric left/right landmark counts");

const std::array<KeypointId, kFootContactKeypointCount>& FootContactKeypoints(bool left) {
    return left ? kLeftFootContactKeypoints : kRightFootContactKeypoints;
}

Vec3f ProjectionDepthAxisWorld(const StereoCameraModel& camera) {
    return NormalizeOr(
        Vec3f{
            camera.image_from_world.m[8],
            camera.image_from_world.m[9],
            camera.image_from_world.m[10]
        },
        Vec3f{0.0f, 0.0f, 1.0f});
}

Vec3f CameraOriginWorld(const StereoCameraModel& camera) {
    return Vec3f{
        camera.world_from_camera.m[3],
        camera.world_from_camera.m[7],
        camera.world_from_camera.m[11]
    };
}

Vec3f StereoObservationDepthAxisWorld(
    const StereoCameraModel& camera_a,
    const StereoCameraModel& camera_b,
    const Vec3f& observed_world) {

    const Vec3f axis_a = SolverObservationDepthAxisFromOrigin(
        CameraOriginWorld(camera_a),
        observed_world,
        ProjectionDepthAxisWorld(camera_a));
    const Vec3f axis_b = SolverObservationDepthAxisFromOrigin(
        CameraOriginWorld(camera_b),
        observed_world,
        ProjectionDepthAxisWorld(camera_b));
    // If the two per-camera range axes cancel (for example cameras on opposite
    // sides of the point), keep a normalized single-camera axis rather than
    // depending on NormalizeOr's fallback normalization contract.
    return NormalizeOr(Add(axis_a, axis_b), NormalizeOr(axis_a, Vec3f{0.0f, 0.0f, 1.0f}));
}

float SolverEvidenceAgeSeconds(
    const BodySolveInputs& inputs,
    const StereoJointEvidence& evidence) {

    auto ms_to_seconds = [](double ms) -> float {
        return std::isfinite(ms) && ms > 0.0 ? static_cast<float>(ms * 0.001) : 0.0f;
    };

    if (evidence.source == JointEvidenceSource::CameraAOnly) {
        return ms_to_seconds(inputs.camera_a_frame_age_ms);
    }
    if (evidence.source == JointEvidenceSource::CameraBOnly) {
        return ms_to_seconds(inputs.camera_b_frame_age_ms);
    }
    if (evidence.source == JointEvidenceSource::Stereo) {
        return std::max(
            ms_to_seconds(inputs.camera_a_frame_age_ms),
            ms_to_seconds(inputs.camera_b_frame_age_ms));
    }
    return 0.0f;
}

float SolverTemporalProcessStddevForEvidence(
    const BodySolveInputs& inputs,
    const StereoJointEvidence& evidence) {

    float age_seconds = SolverEvidenceAgeSeconds(inputs, evidence);
    if (inputs.stereo_pair_reused_a && evidence.source != JointEvidenceSource::CameraBOnly) {
        age_seconds = std::max(age_seconds, static_cast<float>(std::max(0.0, inputs.dt_seconds)));
    }
    if (inputs.stereo_pair_reused_b && evidence.source != JointEvidenceSource::CameraAOnly) {
        age_seconds = std::max(age_seconds, static_cast<float>(std::max(0.0, inputs.dt_seconds)));
    }
    if (inputs.stereo_pair_duplicate || inputs.stereo_pair_skewed || inputs.stereo_pair_degraded) {
        age_seconds = std::max(age_seconds, static_cast<float>(std::max(0.0, inputs.dt_seconds)));
    }
    return SolverTemporalProcessStddevM(age_seconds, inputs.quality.solver_observation_weighting);
}

SolverMeasurementUncertainty SolverUncertaintyFromStereoEvidence(
    const StereoJointEvidence& evidence,
    const StereoCameraModel& camera_a,
    const StereoCameraModel& camera_b,
    float temporal_process_stddev_m) {

    SolverMeasurementUncertainty out;
    out.depth_axis_world = StereoObservationDepthAxisWorld(camera_a, camera_b, evidence.world);
    out.temporal_process_stddev_m = temporal_process_stddev_m;
    if (!evidence.measurement_uncertainty_valid ||
        !evidence.triangulated ||
        evidence.measurement_unclamped_lateral_stddev_m <= 0.0f ||
        evidence.measurement_unclamped_depth_stddev_m <= 0.0f) {
        return out;
    }
    out.valid = true;
    // Solver weighting must consume the unclamped component stddevs. The
    // clamped measurement_*_stddev_m fields are bounded telemetry/reporting
    // values and would collapse all observations beyond the reporting cap onto
    // identical weights.
    out.lateral_stddev_m = evidence.measurement_unclamped_lateral_stddev_m;
    out.depth_stddev_m = evidence.measurement_unclamped_depth_stddev_m;
    return out;
}

float Clamp01(float v) {
    if (!std::isfinite(v)) {
        return 0.0f;
    }
    return std::max(0.0f, std::min(1.0f, v));
}

float Clamp(float v, float lo, float hi) {
    if (!std::isfinite(v)) {
        return lo;
    }
    return std::max(lo, std::min(hi, v));
}

float YawFromQuat(const Quatf& orientation) {
    const Quatf q = Normalize(orientation);
    const Vec3f forward = Rotate(q, Vec3f{0.0f, 0.0f, 1.0f});
    return std::atan2(forward.x, forward.z);
}

Quatf QuatFromYaw(float yaw) {
    const float half = 0.5f * yaw;
    return Quatf{0.0f, std::sin(half), 0.0f, std::cos(half)};
}

Quatf QuatFromAxisAngle(const Vec3f& axis, float angle) {
    const Vec3f n = NormalizeOr(axis, Vec3f{0.0f, 1.0f, 0.0f});
    const float half = 0.5f * angle;
    const float s = std::sin(half);
    return Normalize(Quatf{n.x * s, n.y * s, n.z * s, std::cos(half)});
}

Quatf QuatFromYawPitchRoll(float yaw, float pitch, float roll) {
    const Quatf q_yaw = QuatFromAxisAngle(Vec3f{0.0f, 1.0f, 0.0f}, yaw);
    const Quatf q_pitch = QuatFromAxisAngle(Vec3f{1.0f, 0.0f, 0.0f}, pitch);
    const Quatf q_roll = QuatFromAxisAngle(Vec3f{0.0f, 0.0f, 1.0f}, roll);
    return Multiply(q_yaw, Multiply(q_pitch, q_roll));
}

float PitchFromQuat(const Quatf& orientation) {
    const Quatf q = Normalize(orientation);
    const Vec3f forward = Rotate(q, Vec3f{0.0f, 0.0f, 1.0f});
    const float horizontal = std::sqrt(forward.x * forward.x + forward.z * forward.z);
    return std::atan2(-forward.y, std::max(1e-5f, horizontal));
}

float RollFromQuat(const Quatf& orientation) {
    const Quatf q = Normalize(orientation);
    const Vec3f right = Rotate(q, Vec3f{1.0f, 0.0f, 0.0f});
    const Vec3f up = Rotate(q, Vec3f{0.0f, 1.0f, 0.0f});
    return std::atan2(right.y, std::max(1e-5f, up.y));
}

bool ProjectionReady(const CameraCalibration& c) {
    return c.intrinsics_valid && c.extrinsics_valid;
}

Vec2f ProjectToImage(const Mat34f& p, const Vec3f& world) {
    const Vec3f image = ProjectPoint(p, world);
    return Vec2f{image.x, image.y};
}

bool HasMeaningfulDistortion(const CameraCalibration& camera) {
    if (!camera.intrinsics_valid) {
        return false;
    }
    for (const auto value : camera.distortion) {
        if (std::abs(value) > 1e-10) {
            return true;
        }
    }
    return false;
}

cv::Mat CameraMatrixToCv(const CameraCalibration& camera) {
    cv::Mat k(3, 3, CV_64F);
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            k.at<double>(r, c) = camera.camera_matrix[static_cast<std::size_t>(3 * r + c)];
        }
    }
    return k;
}

cv::Mat DistortionToCv(const CameraCalibration& camera) {
    cv::Mat d(5, 1, CV_64F);
    for (int i = 0; i < 5; ++i) {
        d.at<double>(i, 0) = camera.distortion[static_cast<std::size_t>(i)];
    }
    return d;
}

float ObservationWeight(const JointReliability& r) {
    return r.usable ? Clamp01(r.final_weight) : 0.0f;
}

float FrameAgeQualityScale(double frame_age_ms, double stale_timeout_ms) {
    if (!std::isfinite(frame_age_ms) || frame_age_ms <= 0.0 ||
        !std::isfinite(stale_timeout_ms) || stale_timeout_ms <= 1.0) {
        return 1.0f;
    }
    const double x = frame_age_ms / stale_timeout_ms;
    double scale = 1.0 / (1.0 + x * x);
    if (frame_age_ms > stale_timeout_ms) {
        scale *= 0.35;
    }
    return Clamp01(static_cast<float>(scale));
}

float CameraFrameAgeScale(const BodySolveInputs& inputs, bool camera_a) {
    return FrameAgeQualityScale(
        camera_a ? inputs.camera_a_frame_age_ms : inputs.camera_b_frame_age_ms,
        inputs.stale_timeout_ms);
}

StereoCameraModel StereoCameraModelFromCalibration(const CameraCalibration& camera) {
    StereoCameraModel out;
    out.image_from_world = camera.image_from_world;
    out.world_from_camera = camera.world_from_camera;
    out.camera_matrix = camera.camera_matrix;
    out.projection_valid = ProjectionReady(camera);
    return out;
}

PreparedObservedKeypoints PrepareObservedKeypoints(const DecodedPose2D& pose, const CameraCalibration& camera) {
    PreparedObservedKeypoints out;
    if (!pose.valid) {
        return out;
    }

    const bool undistort = HasMeaningfulDistortion(camera);
    std::vector<cv::Point2f> src;
    std::vector<cv::Point2f> dst;
    std::array<std::size_t, kInternalKeypointOrder.size()> remap{};
    if (undistort) {
        src.reserve(kInternalKeypointOrder.size());
    }

    for (const KeypointId id : kInternalKeypointOrder) {
        const std::size_t i = SolverKeypointIndex(id);
        const auto& kp = pose.keypoints[i];
        if (!kp.present || !std::isfinite(kp.pixel.x) || !std::isfinite(kp.pixel.y)) {
            continue;
        }
        if (!undistort) {
            out.pixels[i] = kp.pixel;
            out.usable[i] = true;
            continue;
        }
        remap[src.size()] = i;
        src.emplace_back(kp.pixel.x, kp.pixel.y);
    }

    if (!undistort) {
        return out;
    }
    if (src.empty()) {
        return out;
    }

    const cv::Mat k = CameraMatrixToCv(camera);
    const cv::Mat d = DistortionToCv(camera);
    cv::undistortPoints(src, dst, k, d, cv::noArray(), k);
    for (std::size_t j = 0; j < dst.size(); ++j) {
        if (!std::isfinite(dst[j].x) || !std::isfinite(dst[j].y)) {
            continue;
        }
        const std::size_t i = remap[j];
        out.pixels[i] = Vec2f{dst[j].x, dst[j].y};
        out.usable[i] = true;
    }
    return out;
}

template <std::size_t N>
float MeanPairwiseSeparationPx(
    const PreparedObservedKeypoints& observed,
    const std::array<KeypointId, N>& ids) {

    float sum = 0.0f;
    int count = 0;
    for (std::size_t a = 0; a < ids.size(); ++a) {
        const std::size_t ia = SolverKeypointIndex(ids[a]);
        if (!observed.usable[ia]) {
            continue;
        }
        for (std::size_t b = a + 1; b < ids.size(); ++b) {
            const std::size_t ib = SolverKeypointIndex(ids[b]);
            if (!observed.usable[ib]) {
                continue;
            }
            const float d = Distance(observed.pixels[ia], observed.pixels[ib]);
            if (std::isfinite(d)) {
                sum += d;
                ++count;
            }
        }
    }
    return count > 0 ? sum / static_cast<float>(count) : 0.0f;
}

float FootLowResSeparationPx(
    const PreparedObservedKeypoints& camera_a_pixels,
    const PreparedObservedKeypoints& camera_b_pixels,
    bool left) {

    const auto& ids = FootContactKeypoints(left);
    const float sep_a = MeanPairwiseSeparationPx(camera_a_pixels, ids);
    const float sep_b = MeanPairwiseSeparationPx(camera_b_pixels, ids);
    if (sep_a > 0.0f && sep_b > 0.0f) {
        return 0.5f * (sep_a + sep_b);
    }
    return std::max(sep_a, sep_b);
}

float FootSeparationConfidence(float separation_px) {
    if (!std::isfinite(separation_px) || separation_px <= 0.0f) {
        return 0.0f;
    }
    return Clamp01(separation_px / kLowResFootSeparationSoftPx);
}

float FootSeparationConfidenceForKeypoint(
    KeypointId id,
    float left_separation_px,
    float right_separation_px) {

    if (IsLeftFootKeypoint(id)) {
        return FootSeparationConfidence(left_separation_px);
    }
    if (IsRightFootKeypoint(id)) {
        return FootSeparationConfidence(right_separation_px);
    }
    return 1.0f;
}

float ReprojectionResidualForView(
    const LowerBodyJointSet& predicted_joints,
    const PreparedObservedKeypoints& observed_pixels,
    const ReliabilitySummary& reliability,
    const CameraCalibration& camera,
    int* out_count) {

    if (!ProjectionReady(camera)) {
        return 0.0f;
    }

    float weighted_sum = 0.0f;
    float weight_sum = 0.0f;
    int count = 0;

    for (const KeypointId id : kSolverKeypoints) {
        const std::size_t i = SolverKeypointIndex(id);
        const auto& pred = predicted_joints.joints[i];
        const float w = ObservationWeight(reliability.joints[i]);
        if (!pred.present || !observed_pixels.usable[i] || w <= 0.0f) {
            continue;
        }

        const Vec2f projected = ProjectToImage(camera.image_from_world, pred.world);
        const float d = Distance(projected, observed_pixels.pixels[i]);
        weighted_sum += w * d * d;
        weight_sum += w;
        ++count;
    }

    if (out_count) {
        *out_count += count;
    }
    return weight_sum > 0.0f ? weighted_sum / weight_sum : 0.0f;
}

void FinalizeStereoTelemetry(BodySolveStereoTelemetry& telemetry) {
    float confidence_sum = 0.0f;
    float reprojection_sum = 0.0f;
    float triangulation_condition_number_sum = 0.0f;
    float triangulation_strength_ratio_sum = 0.0f;
    float triangulation_null_residual_sum = 0.0f;
    float measurement_position_uncertainty_sum = 0.0f;
    float measurement_depth_uncertainty_sum = 0.0f;
    float measurement_baseline_to_depth_sum = 0.0f;
    float solver_lateral_weight_scale_sum = 0.0f;
    float solver_depth_weight_scale_sum = 0.0f;
    float solver_confidence_ceiling_sum = 0.0f;
    float solver_temporal_process_stddev_sum = 0.0f;
    float inferred_depth_sum = 0.0f;
    float foot_confidence_sum = 0.0f;
    float foot_reprojection_sum = 0.0f;
    float camera_a_quality_sum = 0.0f;
    float camera_b_quality_sum = 0.0f;
    float epipolar_error_sum = 0.0f;
    float epipolar_error_isotropic_sum = 0.0f;
    float epipolar_error_anisotropic_sum = 0.0f;
    float epipolar_error_normalized_sum = 0.0f;
    float epipolar_confidence_sum = 0.0f;
    int measurement_count = 0;
    int reprojection_count = 0;
    int foot_count = 0;
    int foot_reprojection_count = 0;

    for (const KeypointId id : kInternalKeypointOrder) {
        const auto& joint = telemetry.joints[SolverKeypointIndex(id)];
        if (joint.camera_a_present) {
            ++telemetry.camera_a_present_keypoints;
            camera_a_quality_sum += joint.camera_a_quality;
        }
        if (joint.camera_b_present) {
            ++telemetry.camera_b_present_keypoints;
            camera_b_quality_sum += joint.camera_b_quality;
        }
        if (joint.camera_a_quality > 0.0f) {
            ++telemetry.camera_a_usable_keypoints;
        }
        if (joint.camera_b_quality > 0.0f) {
            ++telemetry.camera_b_usable_keypoints;
        }
        if (joint.epipolar_checked) {
            ++telemetry.epipolar_checked_count;
            epipolar_error_sum += joint.epipolar_error_px;
            epipolar_error_isotropic_sum += joint.epipolar_error_px_isotropic;
            epipolar_error_anisotropic_sum += joint.epipolar_error_px_anisotropic;
            epipolar_error_normalized_sum += joint.epipolar_error_normalized;
            epipolar_confidence_sum += joint.epipolar_confidence;
        }
        if (joint.epipolar_hard_mismatch) {
            ++telemetry.epipolar_hard_mismatch_count;
        }
        if (joint.epipolar_pair_rejected) {
            ++telemetry.epipolar_pair_rejected_count;
        }
        if (joint.epipolar_degraded_pair_softened) {
            ++telemetry.epipolar_degraded_pair_softened_count;
        }
        const bool has_measurement = joint.triangulated || joint.depth_inferred;
        if (!has_measurement) {
            continue;
        }

        ++measurement_count;
        confidence_sum += joint.confidence;
        if (joint.triangulated) {
            ++telemetry.triangulated_count;
            reprojection_sum += joint.mean_reprojection_error_px;
            triangulation_condition_number_sum += joint.triangulation_condition_number;
            triangulation_strength_ratio_sum += joint.triangulation_strength_ratio;
            triangulation_null_residual_sum += joint.triangulation_null_residual;
            ++reprojection_count;
            if (joint.measurement_uncertainty_valid) {
                ++telemetry.measurement_uncertainty_count;
                measurement_position_uncertainty_sum += joint.measurement_position_stddev_m;
                measurement_depth_uncertainty_sum += joint.measurement_depth_stddev_m;
                measurement_baseline_to_depth_sum += joint.measurement_baseline_to_depth_ratio;
            }
        }
        if (joint.solver_uncertainty_weighted) {
            ++telemetry.solver_uncertainty_weighted_count;
            solver_lateral_weight_scale_sum += joint.solver_lateral_weight_scale;
            solver_depth_weight_scale_sum += joint.solver_depth_weight_scale;
            solver_confidence_ceiling_sum += joint.solver_observation_confidence_ceiling;
            if (joint.solver_uncertainty_valid) {
                ++telemetry.solver_uncertainty_valid_count;
            }
            if (joint.solver_uncertainty_conservative_fallback) {
                ++telemetry.solver_uncertainty_conservative_fallback_count;
            }
            if (joint.solver_temporal_process_noise_applied) {
                ++telemetry.solver_temporal_process_noise_count;
                solver_temporal_process_stddev_sum += joint.solver_temporal_process_stddev_m;
            }
        }
        if (joint.depth_inferred) {
            ++telemetry.inferred_depth_count;
            inferred_depth_sum += joint.estimated_depth_m;
        }

        if (IsSolverFootKeypoint(id)) {
            ++foot_count;
            foot_confidence_sum += joint.confidence;
            if (joint.triangulated) {
                foot_reprojection_sum += joint.mean_reprojection_error_px;
                ++foot_reprojection_count;
                telemetry.max_foot_reprojection_error_px =
                    std::max(telemetry.max_foot_reprojection_error_px, joint.mean_reprojection_error_px);
            }
        }
        if (joint.triangulated && IsLeftFootKeypoint(id)) {
            ++telemetry.left_foot_triangulated_count;
        } else if (joint.triangulated && IsRightFootKeypoint(id)) {
            ++telemetry.right_foot_triangulated_count;
        }
    }

    if (measurement_count > 0) {
        telemetry.mean_confidence = confidence_sum / static_cast<float>(measurement_count);
    }
    if (reprojection_count > 0) {
        telemetry.mean_reprojection_error_px = reprojection_sum / static_cast<float>(reprojection_count);
        telemetry.mean_triangulation_condition_number =
            triangulation_condition_number_sum / static_cast<float>(reprojection_count);
        telemetry.mean_triangulation_strength_ratio =
            triangulation_strength_ratio_sum / static_cast<float>(reprojection_count);
        telemetry.mean_triangulation_null_residual =
            triangulation_null_residual_sum / static_cast<float>(reprojection_count);
    }
    if (telemetry.measurement_uncertainty_count > 0) {
        telemetry.mean_measurement_position_stddev_m =
            measurement_position_uncertainty_sum / static_cast<float>(telemetry.measurement_uncertainty_count);
        telemetry.mean_measurement_depth_stddev_m =
            measurement_depth_uncertainty_sum / static_cast<float>(telemetry.measurement_uncertainty_count);
        telemetry.mean_measurement_baseline_to_depth_ratio =
            measurement_baseline_to_depth_sum / static_cast<float>(telemetry.measurement_uncertainty_count);
    }
    if (telemetry.solver_uncertainty_weighted_count > 0) {
        telemetry.mean_solver_lateral_weight_scale =
            solver_lateral_weight_scale_sum / static_cast<float>(telemetry.solver_uncertainty_weighted_count);
        telemetry.mean_solver_depth_weight_scale =
            solver_depth_weight_scale_sum / static_cast<float>(telemetry.solver_uncertainty_weighted_count);
        telemetry.mean_solver_observation_confidence_ceiling =
            solver_confidence_ceiling_sum / static_cast<float>(telemetry.solver_uncertainty_weighted_count);
    }
    if (telemetry.solver_temporal_process_noise_count > 0) {
        telemetry.mean_solver_temporal_process_stddev_m =
            solver_temporal_process_stddev_sum / static_cast<float>(telemetry.solver_temporal_process_noise_count);
    }
    if (telemetry.inferred_depth_count > 0) {
        telemetry.mean_inferred_depth_m = inferred_depth_sum / static_cast<float>(telemetry.inferred_depth_count);
    }
    if (foot_count > 0) {
        telemetry.foot_mean_confidence = foot_confidence_sum / static_cast<float>(foot_count);
        telemetry.foot_mean_reprojection_error_px =
            foot_reprojection_count > 0 ? foot_reprojection_sum / static_cast<float>(foot_reprojection_count) : 0.0f;
    }
    if (telemetry.camera_a_present_keypoints > 0) {
        telemetry.camera_a_mean_quality = camera_a_quality_sum / static_cast<float>(telemetry.camera_a_present_keypoints);
    }
    if (telemetry.camera_b_present_keypoints > 0) {
        telemetry.camera_b_mean_quality = camera_b_quality_sum / static_cast<float>(telemetry.camera_b_present_keypoints);
    }
    if (telemetry.epipolar_checked_count > 0) {
        telemetry.mean_epipolar_error_px = epipolar_error_sum / static_cast<float>(telemetry.epipolar_checked_count);
        telemetry.mean_epipolar_error_px_isotropic = epipolar_error_isotropic_sum / static_cast<float>(telemetry.epipolar_checked_count);
        telemetry.mean_epipolar_error_px_anisotropic = epipolar_error_anisotropic_sum / static_cast<float>(telemetry.epipolar_checked_count);
        telemetry.mean_epipolar_error_normalized = epipolar_error_normalized_sum / static_cast<float>(telemetry.epipolar_checked_count);
        telemetry.mean_epipolar_confidence = epipolar_confidence_sum / static_cast<float>(telemetry.epipolar_checked_count);
        if (telemetry.epipolar_pair_rejected_count > 0) {
            telemetry.epipolar_status = "hard_mismatch_degraded";
        } else if (telemetry.epipolar_hard_mismatch_count > 0) {
            telemetry.epipolar_status = "mismatch_softened";
        } else {
            telemetry.epipolar_status = "available_used";
        }
    } else if (telemetry.epipolar_geometry_valid) {
        telemetry.epipolar_status = "available_unused";
    }
    if (telemetry.triangulated_count == 0 && telemetry.inferred_depth_count > 0) {
        telemetry.depth_source = DepthSource::InferredMonocular;
    } else if (telemetry.triangulated_count > 0) {
        telemetry.depth_source = DepthSource::TriangulatedStereo;
    }

    const auto foot_confidence = [&telemetry](const std::array<KeypointId, kFootContactKeypointCount>& ids) {
        float sum = 0.0f;
        for (const KeypointId id : ids) {
            sum += telemetry.joints[SolverKeypointIndex(id)].foot_contact_confidence;
        }
        return Clamp01(sum / static_cast<float>(ids.size()));
    };
    telemetry.left_foot_contact_confidence = foot_confidence(kLeftFootContactKeypoints);
    telemetry.right_foot_contact_confidence = foot_confidence(kRightFootContactKeypoints);
}


float FloorContactConfidenceForJoint(
    const BodySolveJointTriangulationTelemetry& joint,
    const FloorPlane& floor,
    float near_distance_m,
    float far_distance_m) {

    if ((!joint.triangulated && !joint.depth_inferred) || joint.confidence <= 0.0f || !FloorPlaneUsable(floor)) {
        return 0.0f;
    }
    const float distance_m = std::abs(SignedDistanceToFloorPlane(joint.world, floor));
    if (!std::isfinite(distance_m) || distance_m >= far_distance_m) {
        return 0.0f;
    }
    if (distance_m <= near_distance_m) {
        return Clamp01(joint.confidence);
    }
    const float t = 1.0f - (distance_m - near_distance_m) / std::max(1e-5f, far_distance_m - near_distance_m);
    return Clamp01(joint.confidence * t);
}

float MeanNonZero(float a, float b) {
    const bool av = a > 0.0f;
    const bool bv = b > 0.0f;
    if (av && bv) {
        return 0.5f * (a + b);
    }
    return std::max(a, b);
}

void AnnotateSupportTelemetry(BodySolveStereoTelemetry& telemetry, const FloorPlane& floor) {
    constexpr float kFootNearM = 0.040f;
    constexpr float kFootFarM = 0.115f;
    constexpr float kKneeNearM = 0.070f;
    constexpr float kKneeFarM = 0.180f;

    const auto contact = [&telemetry, &floor](KeypointId id, float near_m, float far_m) {
        return FloorContactConfidenceForJoint(
            telemetry.joints[SolverKeypointIndex(id)],
            floor,
            near_m,
            far_m);
    };

    telemetry.left_heel_contact_confidence = contact(KeypointId::LeftHeel, kFootNearM, kFootFarM);
    telemetry.left_toe_contact_confidence = MeanNonZero(
        contact(KeypointId::LeftBigToe, kFootNearM, kFootFarM),
        contact(KeypointId::LeftSmallToe, kFootNearM, kFootFarM));
    telemetry.right_heel_contact_confidence = contact(KeypointId::RightHeel, kFootNearM, kFootFarM);
    telemetry.right_toe_contact_confidence = MeanNonZero(
        contact(KeypointId::RightBigToe, kFootNearM, kFootFarM),
        contact(KeypointId::RightSmallToe, kFootNearM, kFootFarM));
    telemetry.left_foot_contact_confidence = std::max(
        telemetry.left_heel_contact_confidence,
        telemetry.left_toe_contact_confidence);
    telemetry.right_foot_contact_confidence = std::max(
        telemetry.right_heel_contact_confidence,
        telemetry.right_toe_contact_confidence);

    const auto& left_knee = telemetry.joints[SolverKeypointIndex(KeypointId::LeftKnee)];
    const auto& right_knee = telemetry.joints[SolverKeypointIndex(KeypointId::RightKnee)];
    telemetry.left_knee_floor_contact_observed = left_knee.triangulated || left_knee.depth_inferred;
    telemetry.right_knee_floor_contact_observed = right_knee.triangulated || right_knee.depth_inferred;
    telemetry.left_knee_floor_contact_confidence = contact(KeypointId::LeftKnee, kKneeNearM, kKneeFarM);
    telemetry.right_knee_floor_contact_confidence = contact(KeypointId::RightKnee, kKneeNearM, kKneeFarM);
}

float CameraGeometryRuntimeScore(
    const FloorGeometryCalibration& floor_geometry,
    const std::vector<WallRectangleCalibration>& walls);

float StereoGeometryScaleForWorldPoint(
    const Vec3f& world,
    const FloorPlane& floor,
    bool stereo_geometry_used);

void CollectRawAnchorWorldFromStereoEvidence(
    KeypointId id,
    const StereoJointEvidence& evidence,
    float confidence,
    RawAnchorWorlds& raw_worlds);

AnchorProjectionCorrectionDebug ApplyAnchorProjectionCorrectionToSeeds(
    const BodySolveInputs& inputs,
    const ProjectionCorrection& correction,
    const RawAnchorWorlds& raw_worlds,
    std::array<WeightedJointSeed, kHalpe26Count>& seeds);

void MirrorAnchorProjectionCorrectionToTelemetry(
    const RawAnchorWorlds& raw_worlds,
    const ProjectionCorrection& correction,
    const AnchorProjectionCorrectionDebug& debug,
    BodySolveStereoTelemetry& telemetry);

std::array<WeightedJointSeed, kHalpe26Count> BuildTriangulatedSeeds(
    const BodySolveInputs& inputs,
    const PreparedObservedKeypoints& camera_a_pixels,
    const PreparedObservedKeypoints& camera_b_pixels,
    const LowerBodyState& predicted,
    BodySolveStereoTelemetry* telemetry = nullptr) {

    std::array<WeightedJointSeed, kHalpe26Count> seeds{};
    RawAnchorWorlds raw_worlds{};
    ProjectionCorrection anchor_correction{};
    AnchorProjectionCorrectionDebug anchor_correction_debug{};
    if (telemetry) {
        *telemetry = BodySolveStereoTelemetry{};
        telemetry->tracking_mode = TrackingMode::Stereo;
        telemetry->depth_source = DepthSource::TriangulatedStereo;
    }

    if (!inputs.camera_a_pose.valid || !inputs.camera_b_pose.valid ||
        !ProjectionReady(inputs.camera_a_calibration) ||
        !ProjectionReady(inputs.camera_b_calibration)) {
        return seeds;
    }

    const float left_foot_separation_px = FootLowResSeparationPx(camera_a_pixels, camera_b_pixels, true);
    const float right_foot_separation_px = FootLowResSeparationPx(camera_a_pixels, camera_b_pixels, false);
    const LowerBodyJointSet predicted_joints = PredictLowerBodyJoints(predicted, inputs.model);
    const StereoCameraModel camera_a_model = StereoCameraModelFromCalibration(inputs.camera_a_calibration);
    const StereoCameraModel camera_b_model = StereoCameraModelFromCalibration(inputs.camera_b_calibration);
    EpipolarGeometry epipolar_geometry{};
    StereoEpipolarContext epipolar_context;
    const auto epipolar_geometry_result = GetOrComputeEpipolarGeometry(
        inputs.stereo_geometry_cache,
        inputs.camera_a_calibration,
        inputs.camera_b_calibration,
        inputs.quality.stereo_evidence.epipolar,
        inputs.quality.stereo_evidence.triangulation);
    const bool epipolar_geometry_valid = epipolar_geometry_result.ok() && epipolar_geometry_result.value().valid;
    if (epipolar_geometry_valid) {
        epipolar_geometry = epipolar_geometry_result.value();
        epipolar_context.geometry = &epipolar_geometry;
        epipolar_context.camera_a = &inputs.camera_a_calibration;
        epipolar_context.camera_b = &inputs.camera_b_calibration;
        epipolar_context.config = inputs.quality.stereo_evidence.epipolar;
        epipolar_context.pair_degraded = inputs.stereo_pair_degraded;
        epipolar_context.reused_camera_a = inputs.stereo_pair_reused_a;
        epipolar_context.reused_camera_b = inputs.stereo_pair_reused_b;
        epipolar_context.duplicate_pair = inputs.stereo_pair_duplicate;
        epipolar_context.timestamp_skewed = inputs.stereo_pair_skewed;
    }
    const float camera_a_age_scale = CameraFrameAgeScale(inputs, true);
    const float camera_b_age_scale = CameraFrameAgeScale(inputs, false);
    const float camera_a_geometry_score = CameraGeometryRuntimeScore(
        inputs.camera_a_floor_geometry,
        inputs.camera_a_wall_rectangles);
    const float camera_b_geometry_score = CameraGeometryRuntimeScore(
        inputs.camera_b_floor_geometry,
        inputs.camera_b_wall_rectangles);
    const bool camera_a_geometry_used = camera_a_geometry_score >= 0.30f;
    const bool camera_b_geometry_used = camera_b_geometry_score >= 0.30f;
    const bool stereo_geometry_used = camera_a_geometry_used && camera_b_geometry_used;
    if (telemetry) {
        telemetry->left_foot_low_res_separation_px = left_foot_separation_px;
        telemetry->right_foot_low_res_separation_px = right_foot_separation_px;
        telemetry->camera_a_age_scale = camera_a_age_scale;
        telemetry->camera_b_age_scale = camera_b_age_scale;
        telemetry->epipolar_geometry_valid = epipolar_geometry_valid;
        telemetry->epipolar_status = epipolar_geometry_valid ? "available_unused" : "invalid_calibration";
        telemetry->camera_a_geometry_used = camera_a_geometry_used;
        telemetry->camera_b_geometry_used = camera_b_geometry_used;
        telemetry->stereo_geometry_constraints_used = stereo_geometry_used;
        telemetry->stereo_geometry_confidence = stereo_geometry_used
            ? std::min(camera_a_geometry_score, camera_b_geometry_score)
            : 0.0f;
        telemetry->geometry_stereo_status = stereo_geometry_used
            ? "paired_used"
            : ((camera_a_geometry_used || camera_b_geometry_used) ? "partial_not_used" : "not_available");
    }

    for (const KeypointId id : kInternalKeypointOrder) {
        const std::size_t i = SolverKeypointIndex(id);
        const float wa = ObservationWeight(inputs.camera_a_reliability.joints[i]);
        const float wb = ObservationWeight(inputs.camera_b_reliability.joints[i]);
        if (telemetry) {
            auto& joint = telemetry->joints[i];
            joint.camera_a_present = camera_a_pixels.usable[i];
            joint.camera_b_present = camera_b_pixels.usable[i];
            joint.camera_a_confidence = inputs.camera_a_pose.keypoints[i].confidence;
            joint.camera_b_confidence = inputs.camera_b_pose.keypoints[i].confidence;
            joint.camera_a_weight = wa;
            joint.camera_b_weight = wb;
        }
        const auto& predicted_joint = predicted_joints.joints[i];
        const StereoTemporalReference temporal{
            predicted_joint.world,
            Clamp01(std::max(predicted_joint.confidence, predicted.confidence)),
            predicted_joint.present && IsFinite(predicted_joint.world)
        };
        const StereoCameraObservation observation_a{
            camera_a_pixels.pixels[i],
            inputs.camera_a_pose.keypoints[i].confidence,
            wa,
            camera_a_age_scale,
            camera_a_pixels.usable[i]
        };
        const StereoCameraObservation observation_b{
            camera_b_pixels.pixels[i],
            inputs.camera_b_pose.keypoints[i].confidence,
            wb,
            camera_b_age_scale,
            camera_b_pixels.usable[i]
        };
        const StereoJointEvidence evidence = ResolveStereoJointEvidence(
            camera_a_model,
            camera_b_model,
            observation_a,
            observation_b,
            temporal,
            epipolar_geometry_valid ? &epipolar_context : nullptr,
            inputs.quality.stereo_evidence);

        const float foot_separation_penalty =
            FootSeparationConfidenceForKeypoint(id, left_foot_separation_px, right_foot_separation_px);
        const float geometry_penalty = IsLowerBodyKeypoint(id)
            ? StereoGeometryScaleForWorldPoint(
                evidence.world,
                inputs.floor,
                stereo_geometry_used)
            : 1.0f;
        const float confidence = StereoSeedConfidence(
            evidence,
            foot_separation_penalty,
            geometry_penalty);
        if (telemetry) {
            auto& joint = telemetry->joints[i];
            joint.camera_a_quality = evidence.camera_a_quality;
            joint.camera_b_quality = evidence.camera_b_quality;
            joint.temporal_confidence = evidence.temporal_confidence;
            joint.epipolar_error_px = evidence.epipolar_error_px;
            joint.epipolar_error_px_isotropic = evidence.epipolar_error_px_isotropic;
            joint.epipolar_error_px_anisotropic = evidence.epipolar_error_px_anisotropic;
            joint.epipolar_error_normalized = evidence.epipolar_error_normalized;
            joint.epipolar_confidence = evidence.epipolar_confidence;
            joint.epipolar_reliability_term = evidence.epipolar_reliability_term;
            joint.epipolar_available = evidence.epipolar_available;
            joint.epipolar_checked = evidence.epipolar_checked;
            joint.epipolar_hard_mismatch = evidence.epipolar_hard_mismatch;
            joint.epipolar_pair_rejected = evidence.epipolar_pair_rejected;
            joint.epipolar_degraded_pair_softened = evidence.epipolar_degraded_pair_softened;
            joint.epipolar_reason = evidence.epipolar_reason;
            joint.epipolar_coordinate_space = evidence.epipolar_coordinate_space;
            joint.used_temporal_depth = evidence.temporal_depth_used;
            joint.fallback_used = evidence.fallback_used;
            joint.evidence_source = evidence.source;
        }
        if (!evidence.valid || !IsFinite(evidence.world) || !std::isfinite(confidence)) {
            continue;
        }

        const float solver_temporal_process_stddev_m = SolverTemporalProcessStddevForEvidence(inputs, evidence);
        auto solver_uncertainty = SolverUncertaintyFromStereoEvidence(
            evidence,
            camera_a_model,
            camera_b_model,
            solver_temporal_process_stddev_m);
        auto solver_information = SolverObservationInformationFromUncertainty(
            solver_uncertainty,
            inputs.quality.solver_observation_weighting);
        seeds[i] = WeightedJointSeed{
            evidence.world,
            confidence,
            evidence.mean_reprojection_error,
            true,
            true,
            solver_uncertainty,
            solver_information
        };
        CollectRawAnchorWorldFromStereoEvidence(id, evidence, confidence, raw_worlds);
        if (telemetry) {
            auto& joint = telemetry->joints[i];
            joint.triangulated = evidence.triangulated;
            joint.depth_inferred = evidence.depth_inferred;
            joint.depth_source = evidence.triangulated ? DepthSource::TriangulatedStereo : DepthSource::InferredMonocular;
            joint.world = evidence.world;
            joint.confidence = confidence;
            joint.reprojection_error_a_px = evidence.reprojection_error_a;
            joint.reprojection_error_b_px = evidence.reprojection_error_b;
            joint.mean_reprojection_error_px = evidence.mean_reprojection_error;
            joint.triangulation_condition_number = evidence.triangulation_condition_number;
            joint.triangulation_strength_ratio = evidence.triangulation_strength_ratio;
            joint.triangulation_null_residual = evidence.triangulation_null_residual;
            joint.measurement_uncertainty_valid = evidence.measurement_uncertainty_valid;
            joint.measurement_baseline_m = evidence.measurement_baseline_m;
            joint.measurement_mean_depth_m = evidence.measurement_mean_depth_m;
            joint.measurement_baseline_to_depth_ratio = evidence.measurement_baseline_to_depth_ratio;
            joint.measurement_effective_focal_px = evidence.measurement_effective_focal_px;
            joint.measurement_reprojection_sigma_px = evidence.measurement_reprojection_sigma_px;
            joint.measurement_epipolar_sigma_px = evidence.measurement_epipolar_sigma_px;
            joint.measurement_image_noise_sigma_px = evidence.measurement_image_noise_sigma_px;
            joint.measurement_conditioning_scale = evidence.measurement_conditioning_scale;
            joint.measurement_unclamped_lateral_stddev_m = evidence.measurement_unclamped_lateral_stddev_m;
            joint.measurement_unclamped_depth_stddev_m = evidence.measurement_unclamped_depth_stddev_m;
            joint.measurement_unclamped_position_variance_m2 =
                2.0f * evidence.measurement_unclamped_lateral_stddev_m * evidence.measurement_unclamped_lateral_stddev_m +
                evidence.measurement_unclamped_depth_stddev_m * evidence.measurement_unclamped_depth_stddev_m;
            joint.measurement_lateral_stddev_m = evidence.measurement_lateral_stddev_m;
            joint.measurement_depth_stddev_m = evidence.measurement_depth_stddev_m;
            joint.measurement_position_stddev_m = evidence.measurement_position_stddev_m;
            joint.measurement_position_variance_m2 = evidence.measurement_position_variance_m2;
            joint.solver_uncertainty_weighted = true;
            joint.solver_uncertainty_valid = solver_information.uncertainty_valid;
            joint.solver_uncertainty_conservative_fallback = solver_information.conservative_fallback;
            joint.solver_temporal_process_noise_applied = solver_information.temporal_process_noise_applied;
            joint.solver_lateral_weight_scale = solver_information.lateral_weight_scale;
            joint.solver_depth_weight_scale = solver_information.depth_weight_scale;
            joint.solver_observation_confidence_ceiling =
                SolverObservationConfidenceCeiling(solver_information);
            joint.solver_temporal_process_stddev_m = solver_information.temporal_process_stddev_m;
            joint.estimated_depth_m = evidence.estimated_depth_m;
            if (IsSolverFootKeypoint(id)) {
                joint.foot_contact_confidence = confidence;
            }
        }
    }
    anchor_correction = EstimateAnchorProjectionCorrection(
        inputs.anchor_space_mapping.config,
        inputs.camera_a_calibration,
        inputs.camera_a_pose.keypoints,
        raw_worlds,
        inputs.steamvr_anchors,
        inputs.camera_a_image_width,
        inputs.camera_a_image_height,
        inputs.camera_a_timestamp_seconds);
    anchor_correction_debug = ApplyAnchorProjectionCorrectionToSeeds(inputs, anchor_correction, raw_worlds, seeds);
    if (telemetry) {
        telemetry->anchor_space_mapping = anchor_correction;
        MirrorAnchorProjectionCorrectionToTelemetry(raw_worlds, anchor_correction, anchor_correction_debug, *telemetry);
        telemetry->room_depth_map = UpdateRoomDepthMapTelemetry(
            inputs.anchor_space_mapping.room_map,
            inputs.anchor_space_mapping.room_map_config,
            anchor_correction,
            inputs.anchor_space_mapping.now_seconds);
        telemetry->hmd_depth_scale = ComputeStereoHmdDepthScale(inputs, *telemetry);
        telemetry->hmd_depth_scale_history = UpdateHmdDepthScaleHistory(
            inputs.hmd_depth_scale.history,
            inputs.hmd_depth_scale.config,
            telemetry->hmd_depth_scale,
            inputs.stereo_hmd_depth_scale_now_seconds);
        FinalizeStereoTelemetry(*telemetry);
    }
    return seeds;
}



void CollectRawAnchorWorldFromStereoEvidence(
    KeypointId id,
    const StereoJointEvidence& evidence,
    float confidence,
    RawAnchorWorlds& raw_worlds) {

    if (!evidence.valid || !IsFinite(evidence.world)) {
        return;
    }
    if (!evidence.triangulated && !evidence.depth_inferred) {
        return;
    }
    CollectRawAnchorWorld(id, evidence.world, confidence, true, raw_worlds);
}

AnchorProjectionCorrectionDebug ApplyAnchorProjectionCorrectionToSeeds(
    const BodySolveInputs& inputs,
    const ProjectionCorrection& correction,
    const RawAnchorWorlds& raw_worlds,
    std::array<WeightedJointSeed, kHalpe26Count>& seeds) {

    if (!correction.valid) {
        return MakeAnchorProjectionCorrectionDebug(
            correction.fallback_reason.empty() ? "correction_invalid" : correction.fallback_reason);
    }
    if (!inputs.camera_a_calibration.extrinsics_valid) {
        return MakeAnchorProjectionCorrectionDebug("camera_a_extrinsics_invalid");
    }
    const float scale_min = inputs.anchor_space_mapping.stereo_depth_config.min_scale;
    const float scale_max = inputs.anchor_space_mapping.stereo_depth_config.max_scale;
    const bool stereo_depth_scale_allowed = inputs.anchor_space_mapping.stereo_depth_config.enabled &&
        inputs.anchor_space_mapping.stereo_depth_config.apply_per_frame &&
        inputs.anchor_space_mapping.stereo_depth_config.camera_space_depth_only &&
        correction.depth_scale >= scale_min && correction.depth_scale <= scale_max;
    if (!stereo_depth_scale_allowed && correction.anchors_used < 2) {
        return MakeAnchorProjectionCorrectionDebug("depth_scale_disallowed_without_translation_anchor");
    }
    ProjectionCorrection applied = correction;
    if (!stereo_depth_scale_allowed) {
        applied.depth_scale = 1.0f;
    }
    AnchorProjectionCorrectionDebug debug = ApplyAnchorProjectionCorrectionToRawWorlds(
        inputs.camera_a_calibration,
        applied,
        raw_worlds);
    for (const KeypointId id : kInternalKeypointOrder) {
        const std::size_t i = SolverKeypointIndex(id);
        if (!debug.applied[i]) {
            continue;
        }
        if (seeds[i].valid) {
            seeds[i].world = debug.corrected_worlds[i].world;
        } else {
            debug.rejection_reasons[i] = "seed_invalid_debug_only";
        }
    }
    return debug;
}

void MirrorAnchorProjectionCorrectionToTelemetry(
    const RawAnchorWorlds& raw_worlds,
    const ProjectionCorrection& correction,
    const AnchorProjectionCorrectionDebug& debug,
    BodySolveStereoTelemetry& telemetry) {

    for (const KeypointId id : kInternalKeypointOrder) {
        const std::size_t i = SolverKeypointIndex(id);
        auto& joint = telemetry.joints[i];
        const auto& raw = raw_worlds[i];
        joint.anchor_raw_world_present = raw.present && IsFinite(raw.world);
        if (joint.anchor_raw_world_present) {
            joint.anchor_raw_world = raw.world;
        }
        joint.anchor_correction_applied = debug.applied[i];
        joint.anchor_correction_rejection_reason = debug.rejection_reasons[i];
        if (debug.applied[i]) {
            joint.anchor_corrected_world = debug.corrected_worlds[i].world;
            joint.world = debug.corrected_worlds[i].world;
            if (std::isfinite(debug.corrected_depths[i]) && debug.corrected_depths[i] > 0.0f) {
                joint.anchor_corrected_depth_m = debug.corrected_depths[i];
                joint.estimated_depth_m = debug.corrected_depths[i];
            }
        } else if (!correction.valid && joint.anchor_correction_rejection_reason == "not_evaluated") {
            joint.anchor_correction_rejection_reason = correction.fallback_reason.empty()
                ? "correction_invalid"
                : correction.fallback_reason;
        }
    }
}

std::array<float, kHalpe26Count> ReliabilityWeights(const ReliabilitySummary& reliability) {
    std::array<float, kHalpe26Count> weights{};
    for (std::size_t i = 0; i < weights.size(); ++i) {
        weights[i] = ObservationWeight(reliability.joints[i]);
    }
    return weights;
}



void ClearBackendFloorGeometryRuntimeFields(MonocularTrackingConfig& config) {
    config.floor_geometry_calibration_enabled = false;
    config.floor_geometry_type = "unknown";
    config.floor_geometry_confidence = 0.0f;
    config.floor_projective_homography_enabled = false;
    config.floor_from_image = {1.0f, 0.0f, 0.0f,
                               0.0f, 1.0f, 0.0f,
                               0.0f, 0.0f, 1.0f};
    config.image_from_floor = {1.0f, 0.0f, 0.0f,
                               0.0f, 1.0f, 0.0f,
                               0.0f, 0.0f, 1.0f};
    config.floor_projective_confidence = 0.0f;
    config.floor_distortion_correction_enabled = false;
    config.floor_distortion_confidence = 0.0f;
    config.floor_radial_k1 = 0.0f;
    config.floor_radial_k2 = 0.0f;
    config.floor_tangential_p1 = 0.0f;
    config.floor_tangential_p2 = 0.0f;
    config.floor_camera_orientation_enabled = false;
    config.floor_camera_pitch_rad = 0.0f;
    config.floor_camera_roll_rad = 0.0f;
    config.floor_camera_orientation_confidence = 0.0f;
}

bool FloorGeometryUsesRawImageSpace(const FloorGeometryCalibration& geometry) {
    return geometry.homography_valid ||
        geometry.family_a.metric_spacing_valid ||
        geometry.family_b.metric_spacing_valid ||
        geometry.distortion.valid ||
        geometry.camera_orientation_valid;
}

bool FloorGeometryImageSpaceMatches(
    const FloorGeometryCalibration& geometry,
    const MonocularTrackingConfig& config) {

    if (!geometry.valid) {
        return true;
    }
    if (geometry.image_width <= 0 || geometry.image_height <= 0) {
        return !FloorGeometryUsesRawImageSpace(geometry);
    }
    return geometry.image_width == config.image_width && geometry.image_height == config.image_height;
}

bool FloorGeometryWasRejectedForRuntimeImageSpace(const FloorGeometryCalibration& geometry) {
    return geometry.reason.rfind("floor_geometry_image_size_mismatch_saved_", 0) == 0;
}

void ClearLegacyScalarFloorAssist(MonocularTrackingConfig& config) {
    config.floor_scale_assist_enabled = false;
    config.floor_depth_line_spacing_m = 0.0f;
    config.floor_depth_line_spacing_px = 0.0f;
    config.floor_depth_reference_y_px = 0.0f;
    config.floor_depth_reference_m = 0.0f;
    config.floor_depth_confidence = 0.0f;
    config.floor_second_axis_spacing_m = 0.0f;
}

MonocularTrackingConfig ApplyFloorGeometryToMonocularConfig(
    MonocularTrackingConfig config,
    const FloorGeometryCalibration& geometry) {

    // Projective homographies, lens correction, and camera-orientation correction
    // are backend calibration products. Do not let old tracking.monocular fields
    // act as stale runtime carriers when calibration.floor_geometry is missing,
    // rejected, or downgraded. Scalar floor spacing remains a legacy explicit UI
    // assist unless a backend geometry object is present and supersedes it.
    ClearBackendFloorGeometryRuntimeFields(config);

    if (!geometry.valid) {
        if (FloorGeometryWasRejectedForRuntimeImageSpace(geometry)) {
            ClearLegacyScalarFloorAssist(config);
        }
        return config;
    }

    if (!FloorGeometryImageSpaceMatches(geometry, config)) {
        ClearLegacyScalarFloorAssist(config);
        return config;
    }

    const bool backend_metric_replacement =
        (geometry.homography_valid && geometry.metric_scale_confidence > 0.0f) ||
        (geometry.family_a.valid && geometry.family_a.metric_spacing_valid) ||
        (geometry.family_b.valid && geometry.family_b.metric_spacing_valid);
    if (backend_metric_replacement) {
        ClearLegacyScalarFloorAssist(config);
    }
    config.floor_geometry_calibration_enabled = true;
    config.floor_geometry_type = geometry.floor_type;
    config.floor_geometry_confidence = Clamp01(std::max(config.floor_geometry_confidence, geometry.metric_scale_confidence));

    if (geometry.family_a.valid && geometry.family_a.metric_spacing_valid) {
        const float nx = -std::sin(geometry.family_a.orientation_rad);
        const float ny = std::cos(geometry.family_a.orientation_rad);
        if (std::abs(ny) >= 0.20f && geometry.family_a.spacing_px > 1.0f && geometry.family_a.spacing_m > 0.0f) {
            const float center_x = 0.5f * static_cast<float>(std::max(1, config.image_width));
            const float reference_y = (geometry.family_a.reference_rho_px - center_x * nx) / ny;
            config.floor_scale_assist_enabled = true;
            config.floor_depth_line_spacing_m = geometry.family_a.spacing_m;
            config.floor_depth_line_spacing_px = geometry.family_a.spacing_px / std::abs(ny);
            config.floor_depth_reference_y_px = Clamp(reference_y, 0.0f, static_cast<float>(std::max(1, config.image_height)));
            config.floor_depth_confidence = Clamp01(std::max(config.floor_depth_confidence, geometry.metric_scale_confidence));
        }
    }
    if (geometry.family_b.valid && geometry.family_b.metric_spacing_valid) {
        config.floor_second_axis_spacing_m = geometry.family_b.spacing_m;
    }
    if (geometry.valid && geometry.homography_valid && geometry.metric_scale_confidence > 0.0f) {
        config.floor_projective_homography_enabled = true;
        config.floor_from_image = geometry.floor_from_image;
        config.image_from_floor = geometry.image_from_floor;
        config.floor_projective_confidence = Clamp01(geometry.metric_scale_confidence);
    }
    if (geometry.camera_height_valid && geometry.camera_height_m > 0.10f) {
        config.camera_height_m = geometry.camera_height_m;
    }
    if (geometry.distortion.valid && geometry.distortion.confidence >= 0.20f) {
        config.floor_distortion_correction_enabled = true;
        config.floor_distortion_confidence = Clamp01(geometry.distortion.confidence);
        config.floor_radial_k1 = geometry.distortion.radial_k1;
        config.floor_radial_k2 = geometry.distortion.radial_k2;
        config.floor_tangential_p1 = geometry.distortion.tangential_p1;
        config.floor_tangential_p2 = geometry.distortion.tangential_p2;
    }
    if (geometry.camera_orientation_valid && geometry.camera_orientation_confidence >= 0.30f) {
        config.floor_camera_orientation_enabled = true;
        config.floor_camera_pitch_rad = geometry.camera_pitch_rad;
        config.floor_camera_roll_rad = geometry.camera_roll_rad;
        config.floor_camera_orientation_confidence = Clamp01(geometry.camera_orientation_confidence);
    }
    return config;
}

bool WallRectangleImageSpaceMatches(const WallRectangleCalibration& wall, const MonocularTrackingConfig& config) {
    if (wall.image_width <= 0 || wall.image_height <= 0) {
        return false;
    }
    return wall.image_width == config.image_width && wall.image_height == config.image_height;
}

bool WallRectangleOrientationUsable(const WallRectangleCalibration& wall) {
    return (wall.valid || wall.usable_for_orientation || wall.wall_orientation_valid) &&
        wall.wall_orientation_valid &&
        wall.wall_orientation_confidence >= 0.30f;
}

bool WallRectangleDepthUsable(const WallRectangleCalibration& wall) {
    return (wall.valid || wall.usable_for_depth_assist || wall.wall_depth_valid) &&
        wall.wall_depth_valid &&
        std::isfinite(wall.wall_center_depth_m) &&
        wall.wall_center_depth_m >= tracking_constants::kMonocularMinDepthM &&
        wall.wall_center_depth_m <= tracking_constants::kMonocularMaxDepthM &&
        wall.wall_depth_confidence > 0.0f;
}

float WallRectangleRuntimeScore(const WallRectangleCalibration& wall) {
    if (!WallRectangleOrientationUsable(wall)) {
        return 0.0f;
    }
    return Clamp01(0.65f * wall.wall_orientation_confidence +
        0.20f * std::max(wall.confidence, wall.wall_orientation_confidence) +
        0.15f * wall.wall_depth_confidence);
}

float WallRectangleDepthRuntimeScore(const WallRectangleCalibration& wall) {
    if (!WallRectangleDepthUsable(wall)) {
        return 0.0f;
    }
    return Clamp01(0.75f * wall.wall_depth_confidence +
        0.25f * std::max(wall.confidence, wall.wall_depth_confidence));
}

float BestWallRectangleRuntimeScore(const std::vector<WallRectangleCalibration>& walls) {
    float best = 0.0f;
    for (const auto& wall : walls) {
        best = std::max(best, WallRectangleRuntimeScore(wall));
        best = std::max(best, WallRectangleDepthRuntimeScore(wall));
    }
    return best;
}

float FloorGeometryRuntimeScore(const FloorGeometryCalibration& geometry) {
    if (!geometry.valid) {
        return 0.0f;
    }
    float score = 0.0f;
    if (geometry.camera_orientation_valid) {
        score = std::max(score, geometry.camera_orientation_confidence);
    }
    if (geometry.homography_valid || geometry.family_a.metric_spacing_valid || geometry.family_b.metric_spacing_valid) {
        score = std::max(score, geometry.metric_scale_confidence);
    }
    if (geometry.floor_plane.valid) {
        score = std::max(score, geometry.floor_plane_confidence);
    }
    return Clamp01(score);
}

float CameraGeometryRuntimeScore(
    const FloorGeometryCalibration& floor_geometry,
    const std::vector<WallRectangleCalibration>& walls) {
    return Clamp01(std::max(FloorGeometryRuntimeScore(floor_geometry), BestWallRectangleRuntimeScore(walls)));
}

float StereoGeometryScaleForWorldPoint(const Vec3f& world, const FloorPlane& floor, bool stereo_geometry_used) {
    if (!stereo_geometry_used || !FloorPlaneUsable(floor) || !IsFinite(world)) {
        return 1.0f;
    }
    const float signed_floor_distance = SignedDistanceToFloorPlane(world, floor);
    if (!std::isfinite(signed_floor_distance) || signed_floor_distance >= -0.04f) {
        return 1.0f;
    }
    if (signed_floor_distance <= -0.25f) {
        return 0.15f;
    }
    return Clamp01(1.0f + signed_floor_distance / 0.25f);
}

MonocularTrackingConfig ApplyWallRectanglesToMonocularConfig(
    MonocularTrackingConfig config,
    const std::vector<WallRectangleCalibration>& walls) {

    const WallRectangleCalibration* best_orientation = nullptr;
    float best_orientation_score = 0.0f;
    const WallRectangleCalibration* best_depth = nullptr;
    float best_depth_score = 0.0f;
    for (const auto& wall : walls) {
        if (!WallRectangleImageSpaceMatches(wall, config)) {
            continue;
        }
        const float orientation_score = WallRectangleRuntimeScore(wall);
        if (orientation_score > best_orientation_score) {
            best_orientation = &wall;
            best_orientation_score = orientation_score;
        }
        const float depth_score = WallRectangleDepthRuntimeScore(wall);
        if (depth_score > best_depth_score) {
            best_depth = &wall;
            best_depth_score = depth_score;
        }
    }

    if (best_depth) {
        config.wall_depth_assist_enabled = true;
        config.wall_depth_assist_m = Clamp(
            best_depth->wall_center_depth_m,
            tracking_constants::kMonocularMinDepthM,
            tracking_constants::kMonocularMaxDepthM);
        config.wall_depth_assist_confidence = Clamp01(best_depth_score);
    }

    if (!best_orientation) {
        return config;
    }
    const Vec3f down = NormalizeOr(best_orientation->wall_down_camera, Vec3f{0.0f, 1.0f, 0.0f});
    if (!IsFinite(down) || down.y <= 0.10f) {
        return config;
    }

    const float wall_pitch = Clamp(std::atan2(down.z, std::max(1e-5f, down.y)), -0.65f, 0.65f);
    const float wall_roll = Clamp(std::atan2(-down.x, std::max(1e-5f, down.y)), -0.65f, 0.65f);
    const float wall_confidence = Clamp01(best_orientation_score);
    if (config.floor_camera_orientation_enabled &&
        config.floor_camera_orientation_confidence >= wall_confidence) {
        return config;
    }

    config.floor_camera_orientation_enabled = true;
    config.floor_camera_pitch_rad = wall_pitch;
    config.floor_camera_roll_rad = wall_roll;
    config.floor_camera_orientation_confidence = wall_confidence;
    return config;
}

// Apply model-depth z-offset from RTMW3D to monocular seeds.
// The RTMW3D z-SimCC bins encode body-relative depth normalized to body scale.
// Bin 0 = nearest, bin (kRtmw3dSimccZBins-1) = farthest.
// Center bin (kRtmw3dSimccZBins/2) corresponds to the body midpoint depth.
// We compute each joint's offset from the pelvis reference bin and apply it
// to the monocular depth estimate (world.z = depth in camera space).
//
// This only fires when:
//   - pose_3d.valid is true (RTMW3D model was used)
//   - both pelvis and the joint have z_decoded = true
//   - a valid user_height_m is available to scale bins to meters
//   - the resulting offset is finite and within a plausible range
//
// coordinate_frame note: world.z in monocular = forward depth (camera-space z).
//   The model z offset is applied along the same axis because RTMW3D z encodes
//   the depth (forward/back) axis relative to a front-facing camera.
//   This is NOT a metric world-space transform â€” it is a relative depth adjustment
//   in camera-forward units.  The transform_note "model_z_applied_as_monocular_depth_offset"
//   must appear in any depth trace that reads this value.
static void ApplyModelDepthToMonocularSeeds(
    std::array<WeightedJointSeed, kHalpe26Count>& seeds,
    const DecodedPose3D& pose_3d,
    float user_height_m) {

    if (!pose_3d.valid) { return; }
    if (!std::isfinite(user_height_m) || user_height_m <= 0.1f) { return; }

    constexpr std::size_t kPelvisIdx = static_cast<std::size_t>(KeypointId::Pelvis);
    const auto& pelvis_dep = pose_3d.model_depth[kPelvisIdx];
    if (!pelvis_dep.z_decoded || pelvis_dep.refined_z <= 0.0f) { return; }

    // Scale: full z bin range spans ~2 body heights of depth.
    // normalized_offset = (joint_bin - pelvis_bin) / (kRtmw3dSimccZBins / 2)
    // metric_offset_m   = normalized_offset * user_height_m
    constexpr float kZBinsHalf = static_cast<float>(kRtmw3dSimccZBins) * 0.5f;
    constexpr float kMaxOffsetM = 1.5f;  // clamp: implausible body-depth offsets

    for (const KeypointId id : kInternalKeypointOrder) {
        const std::size_t i = SolverKeypointIndex(id);
        if (!seeds[i].valid) { continue; }
        const auto& dep = pose_3d.model_depth[i];
        if (!dep.z_decoded) { continue; }

        const float bin_offset  = dep.refined_z - pelvis_dep.refined_z;
        const float norm_offset = bin_offset / kZBinsHalf;
        const float z_offset_m  = norm_offset * user_height_m;

        if (!std::isfinite(z_offset_m)) { continue; }
        const float clamped = std::max(-kMaxOffsetM, std::min(kMaxOffsetM, z_offset_m));

        // Blend: model z offset at low weight so the heuristic depth remains dominant.
        // Weight 0.35 means the model contribution shifts depth noticeably but
        // does not override the floor/body-extent estimate entirely.
        constexpr float kModelZBlend = 0.35f;
        seeds[i].world.z += kModelZBlend * clamped;
    }
}

std::array<WeightedJointSeed, kHalpe26Count> BuildMonocularSeeds(
    const BodySolveInputs& inputs,
    BodySolveStereoTelemetry* telemetry = nullptr) {

    std::array<WeightedJointSeed, kHalpe26Count> seeds{};
    if (telemetry) {
        *telemetry = BodySolveStereoTelemetry{};
        telemetry->tracking_mode = TrackingMode::Monocular;
        telemetry->depth_source = DepthSource::InferredMonocular;
        telemetry->hmd_depth_scale.state = inputs.hmd_depth_scale.enabled
            ? HmdDepthScaleStateKind::UnavailableNoPreviousScale
            : HmdDepthScaleStateKind::Disabled;
        telemetry->hmd_depth_scale.reason = inputs.hmd_depth_scale.enabled
            ? "not computed: no monocular measurement"
            : "disabled";
    }

    if (!inputs.camera_a_pose.valid) {
        return seeds;
    }

    const auto weights = ReliabilityWeights(inputs.camera_a_reliability);
    const MonocularTrackingConfig effective_monocular = ApplyWallRectanglesToMonocularConfig(
        ApplyFloorGeometryToMonocularConfig(
            inputs.quality.monocular,
            inputs.floor_geometry),
        inputs.wall_rectangles);
    const auto measurements = BuildMonocularJointMeasurements(
        inputs.camera_a_pose.keypoints,
        weights,
        inputs.camera_a_calibration,
        effective_monocular,
        inputs.hmd_depth_scale.enabled ? &inputs.hmd_depth_scale : nullptr,
        inputs.anchor_space_mapping.config.enabled ? &inputs.anchor_space_mapping : nullptr);
    if (!measurements.ok()) {
        return seeds;
    }
    if (telemetry) {
        telemetry->monocular_scale_source = measurements.value().scale_source;
        telemetry->monocular_floor_assist_depth_m = measurements.value().floor_assist_depth_m;
        telemetry->monocular_floor_assist_confidence = measurements.value().floor_assist_confidence;
        telemetry->camera_a_age_scale = CameraFrameAgeScale(inputs, true);
        telemetry->camera_b_age_scale = 0.0f;
        telemetry->floor_distortion_correction_used = measurements.value().distortion_correction_used;
        telemetry->floor_camera_orientation_used = measurements.value().camera_orientation_correction_used;
        telemetry->hmd_depth_scale = measurements.value().hmd_depth_scale;
        telemetry->hmd_depth_scale_history = measurements.value().hmd_depth_scale_history;
        telemetry->anchor_space_mapping = measurements.value().anchor_space_mapping;
        telemetry->room_depth_map = measurements.value().room_depth_map;
        const bool geometry_depth_used = measurements.value().scale_source == MonocularScaleSource::FloorProjective ||
            measurements.value().scale_source == MonocularScaleSource::FloorSpacing ||
            measurements.value().scale_source == MonocularScaleSource::WallDepth;
        telemetry->floor_geometry_used = inputs.floor_geometry.valid && geometry_depth_used;
        telemetry->floor_geometry_confidence = telemetry->floor_geometry_used
            ? measurements.value().floor_assist_confidence
            : 0.0f;
        telemetry->floor_geometry_family_count = telemetry->floor_geometry_used
            ? inputs.floor_geometry.family_count
            : 0;
    }

    for (const KeypointId id : kInternalKeypointOrder) {
        const std::size_t i = SolverKeypointIndex(id);
        const float wa = weights[i];
        if (telemetry) {
            auto& joint = telemetry->joints[i];
            joint.camera_a_present = inputs.camera_a_pose.keypoints[i].present;
            joint.camera_b_present = false;
            joint.camera_a_confidence = inputs.camera_a_pose.keypoints[i].confidence;
            joint.camera_b_confidence = 0.0f;
            joint.camera_a_weight = wa;
            joint.camera_b_weight = 0.0f;
            joint.camera_a_quality = wa;
            joint.camera_b_quality = 0.0f;
            joint.evidence_source = inputs.camera_a_pose.keypoints[i].present
                ? JointEvidenceSource::Rejected
                : JointEvidenceSource::None;
        }

        const auto& measurement = measurements.value().joints[i];
        if (!measurement.present || !IsFinite(measurement.world) || !std::isfinite(measurement.confidence)) {
            continue;
        }

        seeds[i] = WeightedJointSeed{
            measurement.world,
            measurements.value().hmd_depth_scale.usable ? 1.0f : measurement.confidence,
            0.0f,
            true,
            false
        };
        if (telemetry) {
            auto& joint = telemetry->joints[i];
            joint.depth_inferred = true;
            joint.depth_source = DepthSource::InferredMonocular;
            joint.evidence_source = JointEvidenceSource::CameraAOnly;
            joint.world = measurement.world;
            joint.confidence = measurement.confidence;
            joint.estimated_depth_m = measurement.estimated_depth_m;
            if (IsSolverFootKeypoint(id)) {
                joint.foot_contact_confidence = measurement.confidence;
            }
        }
    }

    if (telemetry) {
        MirrorAnchorProjectionCorrectionToTelemetry(
            measurements.value().anchor_raw_worlds,
            measurements.value().anchor_space_mapping,
            measurements.value().anchor_correction_debug,
            *telemetry);
    }

    // Wire model z from RTMW3D into the monocular depth estimates.
    // See ApplyModelDepthToMonocularSeeds for coordinate-frame notes.
    ApplyModelDepthToMonocularSeeds(seeds, inputs.camera_a_pose_3d, inputs.quality.monocular.user_height_m);

    if (telemetry) {
        FinalizeStereoTelemetry(*telemetry);
    }
    return seeds;
}

LowerBodyJointSet SeedsToJointSet(const std::array<WeightedJointSeed, kHalpe26Count>& seeds) {
    LowerBodyJointSet set;
    for (const KeypointId id : kSolverKeypoints) {
        const std::size_t i = SolverKeypointIndex(id);
        if (!SeedHasSolverWeight(seeds[i])) {
            continue;
        }
        set.joints[i].world = seeds[i].world;
        set.joints[i].confidence = seeds[i].weight;
        set.joints[i].present = true;
    }
    return set;
}

float MeanSeedWeight(const std::array<WeightedJointSeed, kHalpe26Count>& seeds) {
    float sum = 0.0f;
    int count = 0;
    for (const KeypointId id : kSolverKeypoints) {
        const auto& seed = seeds[SolverKeypointIndex(id)];
        if (SeedHasFinitePose(seed)) {
            sum += Clamp01(seed.weight);
            ++count;
        }
    }
    return count > 0 ? sum / static_cast<float>(count) : 0.0f;
}

int CountValidSeeds(const std::array<WeightedJointSeed, kHalpe26Count>& seeds) {
    int count = 0;
    for (const KeypointId id : kSolverKeypoints) {
        const auto& seed = seeds[SolverKeypointIndex(id)];
        if (SeedHasFinitePose(seed)) {
            ++count;
        }
    }
    return count;
}

bool HasUsableSeed(const std::array<WeightedJointSeed, kHalpe26Count>& seeds, KeypointId id) {
    const auto& seed = seeds[SolverKeypointIndex(id)];
    return SeedHasSolverWeight(seed);
}

int CountUsableSeedsFor(const std::array<WeightedJointSeed, kHalpe26Count>& seeds,
                        const std::initializer_list<KeypointId> ids) {
    int count = 0;
    for (const KeypointId id : ids) {
        if (HasUsableSeed(seeds, id)) {
            ++count;
        }
    }
    return count;
}

bool HasStructuredCoordinateEvidence(const std::array<WeightedJointSeed, kHalpe26Count>& seeds,
                                     int min_seed_count) {
    const int seed_count = CountValidSeeds(seeds);
    if (seed_count < std::max(1, min_seed_count)) {
        return false;
    }

    const bool has_pelvis_seed = HasUsableSeed(seeds, KeypointId::Pelvis);
    const bool has_left_hip_seed = HasUsableSeed(seeds, KeypointId::LeftHip);
    const bool has_right_hip_seed = HasUsableSeed(seeds, KeypointId::RightHip);
    const bool has_core_seed = has_pelvis_seed || has_left_hip_seed || has_right_hip_seed;

    const int left_leg_seed_count = CountUsableSeedsFor(seeds, {
        KeypointId::LeftHip,
        KeypointId::LeftKnee,
        KeypointId::LeftAnkle,
        KeypointId::LeftHeel,
        KeypointId::LeftBigToe,
        KeypointId::LeftSmallToe,
    });
    const int right_leg_seed_count = CountUsableSeedsFor(seeds, {
        KeypointId::RightHip,
        KeypointId::RightKnee,
        KeypointId::RightAnkle,
        KeypointId::RightHeel,
        KeypointId::RightBigToe,
        KeypointId::RightSmallToe,
    });

    const bool left_has_mid_or_distal =
        HasUsableSeed(seeds, KeypointId::LeftKnee) ||
        HasUsableSeed(seeds, KeypointId::LeftAnkle) ||
        HasUsableSeed(seeds, KeypointId::LeftHeel) ||
        HasUsableSeed(seeds, KeypointId::LeftBigToe) ||
        HasUsableSeed(seeds, KeypointId::LeftSmallToe);
    const bool right_has_mid_or_distal =
        HasUsableSeed(seeds, KeypointId::RightKnee) ||
        HasUsableSeed(seeds, KeypointId::RightAnkle) ||
        HasUsableSeed(seeds, KeypointId::RightHeel) ||
        HasUsableSeed(seeds, KeypointId::RightBigToe) ||
        HasUsableSeed(seeds, KeypointId::RightSmallToe);

    const bool left_leg_structured =
        (has_left_hip_seed && left_has_mid_or_distal) ||
        (HasUsableSeed(seeds, KeypointId::LeftKnee) &&
            (HasUsableSeed(seeds, KeypointId::LeftAnkle) ||
             HasUsableSeed(seeds, KeypointId::LeftHeel) ||
             HasUsableSeed(seeds, KeypointId::LeftBigToe) ||
             HasUsableSeed(seeds, KeypointId::LeftSmallToe)));
    const bool right_leg_structured =
        (has_right_hip_seed && right_has_mid_or_distal) ||
        (HasUsableSeed(seeds, KeypointId::RightKnee) &&
            (HasUsableSeed(seeds, KeypointId::RightAnkle) ||
             HasUsableSeed(seeds, KeypointId::RightHeel) ||
             HasUsableSeed(seeds, KeypointId::RightBigToe) ||
             HasUsableSeed(seeds, KeypointId::RightSmallToe)));

    const bool bilateral_support = left_leg_seed_count > 0 && right_leg_seed_count > 0;
    const bool core_with_limb = has_core_seed &&
        (left_leg_structured || right_leg_structured || bilateral_support);

    // Two feet can constrain yaw/scale only if both sides have enough distal evidence.
    // A single foot with several toe/heel points still cannot safely drive a full-body
    // coordinate solve; let the limb estimators/anchors own that sparse frame instead.
    const bool bilateral_distal_without_core =
        !has_core_seed &&
        left_leg_seed_count >= 2 &&
        right_leg_seed_count >= 2 &&
        seed_count >= std::max(4, min_seed_count);

    return core_with_limb || bilateral_distal_without_core;
}

float MeanSeedReprojectionError(const std::array<WeightedJointSeed, kHalpe26Count>& seeds) {
    std::vector<float> errors;
    for (const KeypointId id : kSolverKeypoints) {
        const auto& seed = seeds[SolverKeypointIndex(id)];
        if (SeedHasSolverWeight(seed) && std::isfinite(seed.reprojection_error_px)) {
            errors.push_back(seed.reprojection_error_px);
        }
    }
    if (errors.empty()) {
        return std::numeric_limits<float>::infinity();
    }
    if (errors.size() < 4) {
        float sum = 0.0f;
        for (const float error : errors) {
            sum += error;
        }
        return sum / static_cast<float>(errors.size());
    }

    std::sort(errors.begin(), errors.end());
    const std::size_t trim_count = std::max<std::size_t>(1, errors.size() * 15 / 100);
    const std::size_t kept_count = std::max<std::size_t>(1, errors.size() - trim_count);
    float sum = 0.0f;
    for (std::size_t i = 0; i < kept_count; ++i) {
        sum += errors[i];
    }
    return sum / static_cast<float>(kept_count);
}

Vec3f ModeHmdOffset(const LowerBodyModel& model, PostureMode mode) {
    switch (mode) {
    case PostureMode::SeatedSupported:
        return model.seated_hmd_to_pelvis;
    case PostureMode::ReclinedSupported:
        return model.reclined_hmd_to_pelvis;
    default:
        return model.standing_hmd_to_pelvis;
    }
}

LowerBodyState ApplyHmdPrior(const LowerBodyState& state, const BodySolveInputs& inputs, float gain) {
    if (!inputs.hmd.valid) {
        return state;
    }
    LowerBodyState out = state;
    const Vec3f offset = ModeHmdOffset(inputs.model, state.posture_mode);
    const Vec3f hmd_root{
        inputs.hmd.pose.position.x + offset.x,
        inputs.hmd.pose.position.y + offset.y,
        inputs.hmd.pose.position.z + offset.z
    };
    out.root.position = Lerp(out.root.position, hmd_root, gain);
    return out;
}

float ComputeResidualPrepared(
    const BodySolveInputs& inputs,
    const PreparedObservedKeypoints& camera_a_pixels,
    const PreparedObservedKeypoints& camera_b_pixels,
    const LowerBodyJointSet& joints,
    int* out_observations) {

    float residual = 0.0f;
    int obs_count = 0;
    residual += ReprojectionResidualForView(joints, camera_a_pixels, inputs.camera_a_reliability, inputs.camera_a_calibration, &obs_count);
    residual += ReprojectionResidualForView(joints, camera_b_pixels, inputs.camera_b_reliability, inputs.camera_b_calibration, &obs_count);
    if (out_observations) {
        *out_observations = obs_count;
    }
    return residual;
}

float ComputeResidualPrepared(
    const BodySolveInputs& inputs,
    const PreparedObservedKeypoints& camera_a_pixels,
    const PreparedObservedKeypoints& camera_b_pixels,
    const LowerBodyState& state,
    int* out_observations) {

    return ComputeResidualPrepared(
        inputs,
        camera_a_pixels,
        camera_b_pixels,
        PredictLowerBodyJoints(state, inputs.model),
        out_observations);
}

SolverParams ParamsFromState(const LowerBodyState& state) {
    return SolverParams{
        state.root.position.x,
        state.root.position.y,
        state.root.position.z,
        YawFromQuat(state.root.orientation),
        PitchFromQuat(state.root.orientation),
        RollFromQuat(state.root.orientation),
        state.left_foot.position.x,
        state.left_foot.position.y,
        state.left_foot.position.z,
        YawFromQuat(state.left_foot.orientation),
        PitchFromQuat(state.left_foot.orientation),
        RollFromQuat(state.left_foot.orientation),
        state.left_hip_flexion,
        state.right_foot.position.x,
        state.right_foot.position.y,
        state.right_foot.position.z,
        YawFromQuat(state.right_foot.orientation),
        PitchFromQuat(state.right_foot.orientation),
        RollFromQuat(state.right_foot.orientation),
        state.right_hip_flexion
    };
}

CandidateState CandidateFromParams(const LowerBodyState& base, const LowerBodyModel& model, const SolverParams& p) {
    CandidateState candidate;
    candidate.state = base;
    candidate.state.root.position = Vec3f{p.root_x, p.root_y, p.root_z};
    candidate.state.root.orientation = QuatFromYawPitchRoll(
        p.root_yaw,
        Clamp(p.root_pitch, -0.55f, 0.55f),
        Clamp(p.root_roll, -0.55f, 0.55f));

    candidate.state.left_foot.position = Vec3f{p.left_foot_x, p.left_foot_y, p.left_foot_z};
    candidate.state.left_foot.orientation = QuatFromYawPitchRoll(
        p.left_foot_yaw,
        Clamp(p.left_foot_pitch, -1.10f, 1.10f),
        Clamp(p.left_foot_roll, -0.75f, 0.75f));
    candidate.state.right_foot.position = Vec3f{p.right_foot_x, p.right_foot_y, p.right_foot_z};
    candidate.state.right_foot.orientation = QuatFromYawPitchRoll(
        p.right_foot_yaw,
        Clamp(p.right_foot_pitch, -1.10f, 1.10f),
        Clamp(p.right_foot_roll, -0.75f, 0.75f));

    // These are not forward-kinematic controls anymore. They are only bend-plane
    // priors used by the two-bone IK solve; the solved hip/knee angles overwrite
    // them below.
    candidate.state.left_hip_flexion = Clamp(p.left_bend_hint, -1.4f, 1.4f);
    candidate.state.right_hip_flexion = Clamp(p.right_bend_hint, -1.4f, 1.4f);

    SolveLeg3DFromFootTarget(candidate.state, model, true);
    SolveLeg3DFromFootTarget(candidate.state, model, false);
    candidate.state = ApplyJointLimitBounds(candidate.state);
    candidate.joints = PredictLowerBodyJoints(candidate.state, model);
    return candidate;
}

void SetParamAt(SolverParams& p, int index, float value) {
    if (index < 0 || static_cast<std::size_t>(index) >= kSolverParamCount) {
        return;
    }
    p.*(kSolverParamMembers[static_cast<std::size_t>(index)]) = value;
}

float GetParamAt(const SolverParams& p, int index) {
    if (index < 0 || static_cast<std::size_t>(index) >= kSolverParamCount) {
        return 0.0f;
    }
    return p.*(kSolverParamMembers[static_cast<std::size_t>(index)]);
}

// Seed-pull weight tiers. Higher weights pull the optimizer harder toward
// measured landmark seeds instead of the kinematic prior.
//
// Tier 1, 650: foot contact landmarks. When heel/toe/ankle evidence is present,
// it is the closest thing the lower-body solver has to world-locked ground truth,
// so it must dominate the pose fit.
//
// Tier 2, 420: structural limb landmarks such as knees. These are useful for
// bend-plane and limb-shape correction, but should not overpower planted-foot
// evidence.
//
// Tier 3, 320: internal skeleton landmarks such as hips and pelvis. Root/HMD and
// support constraints already anchor the body core, so these get a softer pull.
float SeedPullWeight(KeypointId id) {
    if (IsSolverFootKeypoint(id)) {
        return 650.0f;  // Tier 1: contact landmarks.
    }
    if (id == KeypointId::LeftHip || id == KeypointId::RightHip || id == KeypointId::Pelvis) {
        return 320.0f;  // Tier 3: internal skeleton.
    }
    return 420.0f;  // Tier 2: structural limb landmarks.
}

float ConfidenceWeightedSeedPull(
    const LowerBodyJointSet& predicted,
    const std::array<WeightedJointSeed, kHalpe26Count>& seeds) {

    float objective = 0.0f;
    for (const KeypointId id : kSolverKeypoints) {
        const std::size_t i = SolverKeypointIndex(id);
        if (!SeedHasSolverWeight(seeds[i]) || !predicted.joints[i].present) {
            continue;
        }
        const float residual = seeds[i].use_uncertainty_weighting
            ? SolverAnisotropicSquaredResidual(
                predicted.joints[i].world,
                seeds[i].world,
                seeds[i].solver_information)
            : Dot(
                Sub(predicted.joints[i].world, seeds[i].world),
                Sub(predicted.joints[i].world, seeds[i].world));
        objective += SeedPullWeight(id) * Clamp01(seeds[i].weight) * residual;
    }
    return objective;
}

float JointConePenalty(const LowerBodyState& state) {
    const auto excess = [](float v, float limit) {
        return std::max(0.0f, std::abs(v) - limit);
    };
    const float hip =
        excess(state.left_hip_abduction, 0.95f) * excess(state.left_hip_abduction, 0.95f) +
        excess(state.right_hip_abduction, 0.95f) * excess(state.right_hip_abduction, 0.95f) +
        excess(state.left_hip_flexion, 2.40f) * excess(state.left_hip_flexion, 2.40f) +
        excess(state.right_hip_flexion, 2.40f) * excess(state.right_hip_flexion, 2.40f);
    const float ankle =
        excess(state.left_ankle_pitch, 1.10f) * excess(state.left_ankle_pitch, 1.10f) +
        excess(state.right_ankle_pitch, 1.10f) * excess(state.right_ankle_pitch, 1.10f) +
        excess(state.left_ankle_roll, 0.75f) * excess(state.left_ankle_roll, 0.75f) +
        excess(state.right_ankle_roll, 0.75f) * excess(state.right_ankle_roll, 0.75f) +
        excess(state.left_ankle_yaw, 1.15f) * excess(state.left_ankle_yaw, 1.15f) +
        excess(state.right_ankle_yaw, 1.15f) * excess(state.right_ankle_yaw, 1.15f);
    const float knee =
        std::max(0.0f, -state.left_knee_flexion) * std::max(0.0f, -state.left_knee_flexion) +
        std::max(0.0f, state.left_knee_flexion - 2.75f) * std::max(0.0f, state.left_knee_flexion - 2.75f) +
        std::max(0.0f, -state.right_knee_flexion) * std::max(0.0f, -state.right_knee_flexion) +
        std::max(0.0f, state.right_knee_flexion - 2.75f) * std::max(0.0f, state.right_knee_flexion - 2.75f);
    return 1200.0f * (hip + ankle + knee);
}


struct ConstraintWeightProfile {
    float floor = 0.0f;
    float leg = 0.0f;
    float left_foot = 0.0f;
    float right_foot = 0.0f;
    bool body_calibration_present = false;
    float body_calibration_confidence = 0.0f;
    int body_calibration_sample_count = 0;
};

float BodyQualityOr(float value, float fallback) {
    return std::isfinite(value) && value > 0.0f ? Clamp01(value) : fallback;
}

float FloorCalibrationConfidence(const BodySolveInputs& inputs) {
    if (!FloorPlaneUsable(inputs.floor)) {
        return 0.0f;
    }

    float confidence = 0.45f;
    const auto& g = inputs.floor_geometry;
    if (g.valid) {
        confidence = std::max(confidence, Clamp01(g.metric_scale_confidence));
        confidence = std::max(confidence, Clamp01(g.floor_plane_confidence));
        confidence = std::max(confidence, Clamp01(g.family_a.confidence));
        confidence = std::max(confidence, Clamp01(g.family_b.confidence));
        confidence = std::max(confidence, Clamp01(g.planted_drift_axis_confidence));
        if (g.homography_valid) {
            const float reproj = Clamp01(1.0f - g.homography_reprojection_error_px / 8.0f);
            const float inliers = Clamp01(static_cast<float>(g.homography_inlier_count) / 12.0f);
            confidence = std::max(confidence, reproj * inliers);
        }
        if (g.camera_orientation_valid) {
            confidence = std::max(confidence, Clamp01(g.camera_orientation_confidence));
        }
        if (g.distortion.valid) {
            confidence = std::max(confidence, 0.5f * Clamp01(g.distortion.confidence));
        }
    }
    return Clamp01(confidence);
}

ConstraintWeightProfile BuildConstraintWeights(const BodySolveInputs& inputs) {
    ConstraintWeightProfile weights;
    const auto& quality = inputs.body_calibration.quality;
    weights.body_calibration_present =
        inputs.body_calibration.standing_neutral_valid ||
        quality.sample_count > 0 ||
        (std::isfinite(quality.overall) && quality.overall > 0.0f);
    weights.body_calibration_sample_count = std::max(0, quality.sample_count);
    weights.body_calibration_confidence = weights.body_calibration_present
        ? Clamp01(quality.overall)
        : 0.0f;
    const float body_fallback = inputs.body_calibration.standing_neutral_valid ? 0.55f : 0.30f;
    const float leg_quality = Clamp01(0.20f * BodyQualityOr(quality.pelvis_width, body_fallback) +
        0.20f * BodyQualityOr(quality.left_femur, body_fallback) +
        0.20f * BodyQualityOr(quality.right_femur, body_fallback) +
        0.20f * BodyQualityOr(quality.left_tibia, body_fallback) +
        0.20f * BodyQualityOr(quality.right_tibia, body_fallback));

    const float floor_confidence = FloorCalibrationConfidence(inputs);
    weights.floor = FloorPlaneUsable(inputs.floor) ? (0.15f + 0.85f * floor_confidence) : 0.0f;
    weights.leg = 0.20f + 0.80f * leg_quality;
    weights.left_foot = 0.20f + 0.80f * BodyQualityOr(quality.left_foot_length, leg_quality);
    weights.right_foot = 0.20f + 0.80f * BodyQualityOr(quality.right_foot_length, leg_quality);
    return weights;
}

float PositiveDtSeconds(double dt_seconds) {
    if (!std::isfinite(dt_seconds) || dt_seconds <= 1e-5) {
        return 1.0f / 60.0f;
    }
    return Clamp(static_cast<float>(dt_seconds), 1.0f / 240.0f, 0.10f);
}

float QuaternionAngleRad(const Quatf& a, const Quatf& b) {
    const float d = std::abs(Dot(Normalize(a), Normalize(b)));
    return 2.0f * std::acos(Clamp(d, 0.0f, 1.0f));
}

void RecordConstraint(
    BodySolveConstraintResidualTelemetry* telemetry,
    bool active,
    float weight,
    float residual,
    float score) {

    if (!telemetry) {
        return;
    }
    telemetry->active = active;
    telemetry->weight = active ? weight : 0.0f;
    telemetry->residual_m = active ? residual : 0.0f;
    telemetry->score = active ? score : 0.0f;
}

float AddConstraint(
    BodySolveConstraintResidualTelemetry* telemetry,
    bool active,
    float weight,
    float residual) {

    if (!active || weight <= 0.0f || !std::isfinite(residual)) {
        RecordConstraint(telemetry, false, 0.0f, 0.0f, 0.0f);
        return 0.0f;
    }
    const float score = weight * residual * residual;
    RecordConstraint(telemetry, true, weight, residual, score);
    return score;
}

float SupportWeight(const FootSupportState& support) {
    return Clamp01(FootSupportConfidence(support) * support.transition_quality);
}

float AnchorResidualMagnitude(const SupportAnchor& anchor, const Vec3f& measured_contact) {
    if (!anchor.active) {
        return 0.0f;
    }
    return Distance(measured_contact, anchor.pose.position);
}

Vec3f FloorTangentDelta(const Vec3f& delta, const FloorPlane& floor) {
    const Vec3f normal = FloorPlaneUsable(floor)
        ? NormalizeOr(floor.normal, Vec3f{0.0f, 1.0f, 0.0f})
        : Vec3f{0.0f, 1.0f, 0.0f};
    return Sub(delta, Scale(normal, Dot(delta, normal)));
}

float AnchorSlidingVelocityMps(const SupportAnchor& anchor, const FloorPlane& floor, double dt_seconds) {
    if (!anchor.active || !anchor.has_contact_history) {
        return 0.0f;
    }
    const Vec3f delta = Sub(anchor.current_contact_position, anchor.previous_contact_position);
    return Length(FloorTangentDelta(delta, floor)) / PositiveDtSeconds(dt_seconds);
}

float FootSlidingVelocityMps(const FootSupportState& support, const FloorPlane& floor, double dt_seconds) {
    if (!FootSupportHasContactConstraint(support)) {
        return 0.0f;
    }
    switch (support.phase) {
    case FootSupportPhase::HeelLock:
    case FootSupportPhase::ContactCandidate:
        return AnchorSlidingVelocityMps(support.heel_anchor, floor, dt_seconds);
    case FootSupportPhase::ToePivot:
        return AnchorSlidingVelocityMps(support.toe_anchor, floor, dt_seconds);
    case FootSupportPhase::FlatPlant:
        return std::max(
            AnchorSlidingVelocityMps(support.heel_anchor, floor, dt_seconds),
            AnchorSlidingVelocityMps(support.toe_anchor, floor, dt_seconds));
    default:
        return AnchorSlidingVelocityMps(support.anchor, floor, dt_seconds);
    }
}

float FloorPenetrationMagnitude(const FloorPlane& floor, const std::vector<Vec3f>& points) {
    if (!FloorPlaneUsable(floor)) {
        return 0.0f;
    }
    float max_penetration = 0.0f;
    for (const auto& point : points) {
        const float d = SignedDistanceToFloorPlane(point, floor);
        if (std::isfinite(d)) {
            max_penetration = std::max(max_penetration, std::max(0.0f, -d));
        }
    }
    return max_penetration;
}

float BoneLengthResidual(const LowerBodyJointSet& joints, const LowerBodyModel& model) {
    struct Segment {
        KeypointId a;
        KeypointId b;
        float expected;
    };
    const std::array<Segment, 7> segments{{
        {KeypointId::LeftHip, KeypointId::RightHip, model.pelvis_width},
        {KeypointId::LeftHip, KeypointId::LeftKnee, model.left_femur},
        {KeypointId::LeftKnee, KeypointId::LeftAnkle, model.left_tibia},
        {KeypointId::RightHip, KeypointId::RightKnee, model.right_femur},
        {KeypointId::RightKnee, KeypointId::RightAnkle, model.right_tibia},
        {KeypointId::LeftHeel, KeypointId::LeftBigToe, model.left_foot_length},
        {KeypointId::RightHeel, KeypointId::RightBigToe, model.right_foot_length}
    }};

    float sum_sq = 0.0f;
    int count = 0;
    for (const auto& segment : segments) {
        const auto& a = joints.joints[SolverKeypointIndex(segment.a)];
        const auto& b = joints.joints[SolverKeypointIndex(segment.b)];
        if (!a.present || !b.present || segment.expected <= 0.0f) {
            continue;
        }
        const float err = Distance(a.world, b.world) - segment.expected;
        if (std::isfinite(err)) {
            sum_sq += err * err;
            ++count;
        }
    }
    return count > 0 ? std::sqrt(sum_sq / static_cast<float>(count)) : 0.0f;
}

float FootSupportConstraintPenalty(
    const Pose3f& foot,
    const FootSupportState& support,
    const FloorPlane& floor,
    float foot_length_m,
    float floor_weight,
    float body_weight,
    double dt_seconds,
    BodySolveFootConstraintTelemetry* telemetry) {

    const float support_weight = SupportWeight(support);
    if (telemetry) {
        telemetry->support_confidence = FootSupportConfidence(support);
        telemetry->transition_quality = support.transition_quality;
        telemetry->floor_weight_scale = floor_weight;
        telemetry->body_weight_scale = body_weight;
    }

    float objective = 0.0f;
    const float anchor_scale = support_weight * floor_weight * body_weight;
    const Vec3f heel = FootHeelContactPoint(foot, foot_length_m);
    const Vec3f toe = FootToeContactPoint(foot, foot_length_m);

    if (support.heel_anchor.active) {
        const float w = 6200.0f * anchor_scale * Clamp01(support.heel_anchor.confidence);
        objective += AddConstraint(
            telemetry ? &telemetry->heel_anchor : nullptr,
            true,
            w,
            AnchorResidualMagnitude(support.heel_anchor, heel));
    } else {
        RecordConstraint(telemetry ? &telemetry->heel_anchor : nullptr, false, 0.0f, 0.0f, 0.0f);
    }

    if (support.toe_anchor.active) {
        const float w = 6200.0f * anchor_scale * Clamp01(support.toe_anchor.confidence);
        objective += AddConstraint(
            telemetry ? &telemetry->toe_anchor : nullptr,
            true,
            w,
            AnchorResidualMagnitude(support.toe_anchor, toe));
    } else {
        RecordConstraint(telemetry ? &telemetry->toe_anchor : nullptr, false, 0.0f, 0.0f, 0.0f);
    }

    const auto full_plant = FootSupportResidual(foot, support, foot_length_m);
    const bool full_plant_active = support.phase == FootSupportPhase::FlatPlant && full_plant.valid;
    objective += AddConstraint(
        telemetry ? &telemetry->full_plant : nullptr,
        full_plant_active,
        7600.0f * anchor_scale,
        full_plant.magnitude_m);

    const float floor_penetration = FloorPenetrationMagnitude(floor, {foot.position, heel, toe});
    const float floor_support_scale = std::max(0.25f, support_weight);
    objective += AddConstraint(
        telemetry ? &telemetry->floor_penetration : nullptr,
        FloorPlaneUsable(floor),
        4200.0f * floor_weight * floor_support_scale,
        floor_penetration);

    const bool anchored = FootSupportHasContactConstraint(support);
    const float slide_mps = anchored ? FootSlidingVelocityMps(support, floor, dt_seconds) : 0.0f;
    objective += AddConstraint(
        telemetry ? &telemetry->sliding_velocity : nullptr,
        anchored,
        950.0f * anchor_scale,
        slide_mps);

    const bool orientation_active =
        support.phase == FootSupportPhase::FlatPlant &&
        support.anchor.active &&
        support.anchor.confidence > 0.0f;
    objective += AddConstraint(
        telemetry ? &telemetry->orientation : nullptr,
        orientation_active,
        700.0f * anchor_scale * Clamp01(support.anchor.confidence),
        orientation_active ? QuaternionAngleRad(foot.orientation, support.anchor.pose.orientation) : 0.0f);

    return objective;
}

float RootSupportWeight(RootSupportType type) {
    if (type == RootSupportType::SeatSupported || type == RootSupportType::BodyRestSupported) {
        return 980.0f;
    }
    if (type == RootSupportType::KneeSupported || type == RootSupportType::MixedSupported) {
        return 760.0f;
    }
    if (type == RootSupportType::FeetSupported) {
        return 560.0f;
    }
    return 0.0f;
}

float SupportConstraintPenalty(
    const LowerBodyState& state,
    const BodySolveInputs& inputs,
    BodySolveSupportConstraintTelemetry* telemetry = nullptr) {

    const ConstraintWeightProfile weights = BuildConstraintWeights(inputs);
    if (telemetry) {
        *telemetry = BodySolveSupportConstraintTelemetry{};
        telemetry->floor_calibration_weight = weights.floor;
        telemetry->leg_length_weight = weights.leg;
        telemetry->left_foot_length_weight = weights.left_foot;
        telemetry->right_foot_length_weight = weights.right_foot;
        telemetry->body_calibration_present = weights.body_calibration_present;
        telemetry->body_calibration_confidence = weights.body_calibration_confidence;
        telemetry->body_calibration_sample_count = weights.body_calibration_sample_count;
        telemetry->left_reach_clamped = state.left_leg_reach_clamped;
        telemetry->right_reach_clamped = state.right_leg_reach_clamped;
    }

    float objective = 0.0f;
    objective += FootSupportConstraintPenalty(
        state.left_foot,
        state.support.left_foot,
        inputs.floor,
        inputs.model.left_foot_length,
        weights.floor,
        weights.left_foot,
        inputs.dt_seconds,
        telemetry ? &telemetry->left_foot : nullptr);
    objective += FootSupportConstraintPenalty(
        state.right_foot,
        state.support.right_foot,
        inputs.floor,
        inputs.model.right_foot_length,
        weights.floor,
        weights.right_foot,
        inputs.dt_seconds,
        telemetry ? &telemetry->right_foot : nullptr);

    const LowerBodyJointSet joints = PredictLowerBodyJoints(state, inputs.model);
    objective += AddConstraint(
        telemetry ? &telemetry->bone_length : nullptr,
        true,
        5200.0f * weights.leg,
        BoneLengthResidual(joints, inputs.model));

    if (state.support.root_anchor.active && state.support.root_support != RootSupportType::None) {
        const float d = Distance(state.root.position, state.support.root_anchor.pose.position);
        objective += AddConstraint(
            telemetry ? &telemetry->root_support : nullptr,
            true,
            RootSupportWeight(state.support.root_support) * weights.leg * Clamp01(state.support.root_anchor.confidence),
            d);
    } else {
        RecordConstraint(telemetry ? &telemetry->root_support : nullptr, false, 0.0f, 0.0f, 0.0f);
    }

    const auto add_knee = [&](KeypointId id, const SupportAnchor& anchor, BodySolveConstraintResidualTelemetry* out) {
        if (!anchor.active) {
            RecordConstraint(out, false, 0.0f, 0.0f, 0.0f);
            return 0.0f;
        }
        const auto& knee = joints.joints[SolverKeypointIndex(id)];
        if (!knee.present) {
            RecordConstraint(out, false, 0.0f, 0.0f, 0.0f);
            return 0.0f;
        }
        return AddConstraint(
            out,
            true,
            3200.0f * weights.floor * weights.leg * Clamp01(anchor.confidence),
            Distance(knee.world, anchor.pose.position));
    };
    objective += add_knee(
        KeypointId::LeftKnee,
        state.support.left_knee_anchor,
        telemetry ? &telemetry->left_knee_floor_anchor : nullptr);
    objective += add_knee(
        KeypointId::RightKnee,
        state.support.right_knee_anchor,
        telemetry ? &telemetry->right_knee_floor_anchor : nullptr);

    return objective;
}

ObjectiveResult Objective(
    const BodySolveInputs& inputs,
    const PreparedObservedKeypoints& camera_a_pixels,
    const PreparedObservedKeypoints& camera_b_pixels,
    const std::array<WeightedJointSeed, kHalpe26Count>& seeds,
    const LowerBodyState& base,
    const SolverParams& params) {

    ObjectiveResult result;
    result.candidate = CandidateFromParams(base, inputs.model, params);
    int observations = 0;
    float objective = ComputeResidualPrepared(inputs, camera_a_pixels, camera_b_pixels, result.candidate.joints, &observations);
    objective += ConfidenceWeightedSeedPull(result.candidate.joints, seeds);
    objective += JointConePenalty(result.candidate.state);
    objective += SupportConstraintPenalty(result.candidate.state, inputs);

    if (inputs.hmd.valid) {
        const Vec3f offset = ModeHmdOffset(inputs.model, result.candidate.state.posture_mode);
        const Vec3f expected_root = Add(inputs.hmd.pose.position, offset);
        const float d = Distance(result.candidate.state.root.position, expected_root);
        objective += 400.0f * d * d;
    }

    const float pitch_excess = std::max(0.0f, std::abs(params.root_pitch) - 0.55f);
    const float roll_excess = std::max(0.0f, std::abs(params.root_roll) - 0.55f);
    objective += 700.0f * (pitch_excess * pitch_excess + roll_excess * roll_excess);

    if (observations == 0 && !inputs.hmd.valid) {
        objective += 1.0e6f;
    }
    result.score = objective;
    return result;
}

OptimizerResult OptimizeWeightedReprojection(
    const BodySolveInputs& inputs,
    const PreparedObservedKeypoints& camera_a_pixels,
    const PreparedObservedKeypoints& camera_b_pixels,
    const std::array<WeightedJointSeed, kHalpe26Count>& seeds,
    const LowerBodyState& initial) {

    const auto solve_start = NowQpc();
    SolverParams params = ParamsFromState(initial);
    std::array<float, kSolverParamCount> steps{
        0.035f, 0.035f, 0.035f,
        0.040f, 0.030f, 0.030f,
        0.030f, 0.030f, 0.030f,
        0.045f, 0.035f, 0.035f,
        0.045f,
        0.030f, 0.030f, 0.030f,
        0.045f, 0.035f, 0.035f,
        0.045f
    };
    const std::array<float, kSolverParamCount> initial_steps = steps;
    ObjectiveResult best = Objective(inputs, camera_a_pixels, camera_b_pixels, seeds, initial, params);
    BodySolveTelemetry telemetry;
    telemetry.objective_evaluations = 1;

    for (int iter = 0; iter < 7; ++iter) {
        const SolverParams pass_start_params = params;
        bool improved_this_pass = false;
        ++telemetry.coordinate_passes;
        for (int i = 0; i < static_cast<int>(kSolverParamCount); ++i) {
            const float original = GetParamAt(params, i);

            SolverParams plus = params;
            SetParamAt(plus, i, original + steps[static_cast<std::size_t>(i)]);
            const ObjectiveResult plus_score = Objective(inputs, camera_a_pixels, camera_b_pixels, seeds, initial, plus);

            SolverParams minus = params;
            SetParamAt(minus, i, original - steps[static_cast<std::size_t>(i)]);
            const ObjectiveResult minus_score = Objective(inputs, camera_a_pixels, camera_b_pixels, seeds, initial, minus);
            telemetry.objective_evaluations += 2;

            if (plus_score.score < best.score && plus_score.score <= minus_score.score) {
                params = plus;
                best = plus_score;
                improved_this_pass = true;
            } else if (minus_score.score < best.score) {
                params = minus;
                best = minus_score;
                improved_this_pass = true;
            }
        }

        if (!inputs.quality.use_legacy_solver && improved_this_pass) {
            SolverParams pattern = params;
            bool has_pattern_delta = false;
            for (int i = 0; i < static_cast<int>(kSolverParamCount); ++i) {
                const float delta = GetParamAt(params, i) - GetParamAt(pass_start_params, i);
                if (!std::isfinite(delta) || std::abs(delta) <= 1.0e-6f) {
                    continue;
                }
                SetParamAt(pattern, i, GetParamAt(params, i) + 0.50f * delta);
                has_pattern_delta = true;
            }
            if (has_pattern_delta) {
                const ObjectiveResult pattern_score = Objective(
                    inputs,
                    camera_a_pixels,
                    camera_b_pixels,
                    seeds,
                    initial,
                    pattern);
                telemetry.objective_evaluations += 1;
                if (pattern_score.score < best.score) {
                    params = pattern;
                    best = pattern_score;
                }
            }
        }

        bool all_steps_at_floor = true;
        if (inputs.quality.use_legacy_solver || !improved_this_pass) {
            for (std::size_t i = 0; i < steps.size(); ++i) {
                steps[i] *= kOptimizerStepDecay;
                if (!inputs.quality.use_legacy_solver) {
                    const float min_step = initial_steps[i] * 0.15f;
                    steps[i] = std::max(steps[i], min_step);
                    all_steps_at_floor = all_steps_at_floor && steps[i] <= min_step + 1e-7f;
                }
            }
        }
        if (!inputs.quality.use_legacy_solver && !improved_this_pass && all_steps_at_floor) {
            telemetry.optimizer_early_stopped = true;
            break;
        }
    }

    telemetry.solve_ms = QpcDeltaSeconds(solve_start, NowQpc()) * 1000.0;
    return OptimizerResult{best.candidate, telemetry};
}

LowerBodyState PreconditionRootFromSupport(const LowerBodyState& state, const BodySolveInputs& inputs) {
    LowerBodyState out = state;
    if (!out.support.root_anchor.active || out.support.root_support == RootSupportType::None) {
        return out;
    }
    const ConstraintWeightProfile weights = BuildConstraintWeights(inputs);
    const float gain = Clamp01(0.30f * weights.leg * Clamp01(out.support.root_anchor.confidence));
    out.root.position = Lerp(out.root.position, out.support.root_anchor.pose.position, gain);
    return out;
}

void ReleaseFootSupport(FootSupportState& support) {
    support.type = FootSupportType::None;
    support.phase = FootSupportPhase::Swing;
    support.contact_load = FootContactLoad::None;
    support.heel_contact_confidence = 0.0f;
    support.toe_contact_confidence = 0.0f;
    support.transition_quality = 1.0f;
    support.anchor.active = false;
    support.anchor.confidence = 0.0f;
    support.anchor.release_seconds = 0.0;
    support.anchor.dwell_seconds = 0.0;
    support.heel_anchor.active = false;
    support.heel_anchor.confidence = 0.0f;
    support.toe_anchor.active = false;
    support.toe_anchor.confidence = 0.0f;
}

void DegradeFootSupportFromResiduals(
    FootSupportState& support,
    BodySolveFootConstraintTelemetry& telemetry) {

    const auto excessive = [](const BodySolveConstraintResidualTelemetry& t, float limit) {
        return t.active && t.residual_m > limit;
    };

    const bool bad_anchor =
        excessive(telemetry.heel_anchor, 0.135f) ||
        excessive(telemetry.toe_anchor, 0.135f) ||
        excessive(telemetry.full_plant, 0.150f);
    const bool bad_slide = telemetry.sliding_velocity.active && telemetry.sliding_velocity.residual_m > 2.40f;
    const bool bad_floor = excessive(telemetry.floor_penetration, 0.080f);
    const bool bad_orientation = telemetry.orientation.active && telemetry.orientation.residual_m > 0.85f;

    if (!bad_anchor && !bad_slide && !bad_floor && !bad_orientation) {
        telemetry.degraded_or_released = false;
        return;
    }

    telemetry.degraded_or_released = true;
    support.transition_quality = std::min(support.transition_quality, 0.45f);
    support.anchor.confidence *= 0.35f;
    support.heel_anchor.confidence *= 0.35f;
    support.toe_anchor.confidence *= 0.35f;
    support.heel_contact_confidence *= 0.35f;
    support.toe_contact_confidence *= 0.35f;

    if (support.anchor.confidence <= 0.04f ||
        (bad_anchor && bad_slide) ||
        (bad_floor && bad_orientation)) {
        ReleaseFootSupport(support);
        return;
    }

    if (support.heel_anchor.confidence <= 0.04f) {
        support.heel_anchor.active = false;
    }
    if (support.toe_anchor.confidence <= 0.04f) {
        support.toe_anchor.active = false;
    }
    if (!support.heel_anchor.active && !support.toe_anchor.active) {
        ReleaseFootSupport(support);
    }
}

void DegradeSupportFromResiduals(
    LowerBodyState& state,
    BodySolveSupportConstraintTelemetry& telemetry) {

    DegradeFootSupportFromResiduals(state.support.left_foot, telemetry.left_foot);
    DegradeFootSupportFromResiduals(state.support.right_foot, telemetry.right_foot);

    if (telemetry.root_support.active && telemetry.root_support.residual_m > 0.36f) {
        state.support.root_anchor.confidence *= 0.40f;
        if (state.support.root_anchor.confidence <= 0.05f) {
            state.support.root_anchor.active = false;
            state.support.root_support = RootSupportType::None;
        }
    }

    const auto degrade_knee = [](SupportAnchor& anchor, const BodySolveConstraintResidualTelemetry& residual) {
        if (!residual.active || residual.residual_m <= 0.16f) {
            return;
        }
        anchor.confidence *= 0.40f;
        if (anchor.confidence <= 0.05f) {
            anchor.active = false;
            anchor.confidence = 0.0f;
            anchor.dwell_seconds = 0.0;
            anchor.release_seconds = 0.0;
        }
    };
    degrade_knee(state.support.left_knee_anchor, telemetry.left_knee_floor_anchor);
    degrade_knee(state.support.right_knee_anchor, telemetry.right_knee_floor_anchor);
}

float SolverObservationConfidenceCeiling(const BodySolveJointTriangulationTelemetry& joint) {
    if (!joint.solver_uncertainty_weighted) {
        return 1.0f;
    }
    const float scale = std::isfinite(joint.solver_lateral_weight_scale) &&
            std::isfinite(joint.solver_depth_weight_scale)
        ? std::min(joint.solver_lateral_weight_scale, joint.solver_depth_weight_scale)
        : 0.0f;
    return SolverObservationConfidenceCeilingFromWeightScale(scale);
}

TrackerEvidence FootTrackerEvidenceFromStereo(const BodySolveStereoTelemetry& telemetry, bool left) {
    TrackerEvidence evidence;
    float direct_sum = 0.0f;
    int direct_count = 0;
    bool has_stereo = false;
    bool has_monocular = false;

    for (const KeypointId id : kSolverKeypoints) {
        if ((left && !IsLeftFootKeypoint(id)) || (!left && !IsRightFootKeypoint(id))) {
            continue;
        }
        const auto& joint = telemetry.joints[SolverKeypointIndex(id)];
        if (!joint.triangulated && !joint.depth_inferred) {
            continue;
        }
        direct_sum += Clamp01(std::min(joint.confidence, SolverObservationConfidenceCeiling(joint)));
        ++direct_count;
        has_stereo = has_stereo || joint.triangulated;
        has_monocular = has_monocular || joint.depth_inferred;
    }

    if (direct_count <= 0) {
        evidence.source = TrackerEvidenceSource::Predicted;
        evidence.valid = false;
        return evidence;
    }

    evidence.source = has_stereo ? TrackerEvidenceSource::DirectStereo :
        (has_monocular ? TrackerEvidenceSource::InferredMonocular : TrackerEvidenceSource::Predicted);
    evidence.direct_confidence = Clamp01(direct_sum / static_cast<float>(direct_count));
    evidence.valid = direct_count > 0;
    return evidence;
}

TrackerEvidence ApplySupportToFootEvidence(TrackerEvidence evidence, const FootSupportState& support) {
    evidence.support_confidence = FootSupportConfidence(support);
    evidence.anchor_held = IsActiveFootSupport(support);
    if (evidence.direct_confidence < 0.15f && IsActiveFootSupport(support)) {
        evidence.source = TrackerEvidenceSource::AnchorHeld;
        evidence.valid = evidence.support_confidence > 0.0f;
    }
    return evidence;
}


} // namespace

Result<BodySolveResult> RunPreliminaryBodySolve(const BodySolveInputs& inputs, const LowerBodyState& predicted) {
    const auto solve_start = NowQpc();
    const auto camera_a_pixels = PrepareObservedKeypoints(inputs.camera_a_pose, inputs.camera_a_calibration);
    const auto camera_b_pixels = PrepareObservedKeypoints(inputs.camera_b_pose, inputs.camera_b_calibration);
    const bool monocular_mode = inputs.quality.tracking_mode == TrackingMode::Monocular;
    BodySolveStereoTelemetry stereo_telemetry;
    const auto seeds = monocular_mode
        ? BuildMonocularSeeds(inputs, &stereo_telemetry)
        : BuildTriangulatedSeeds(inputs, camera_a_pixels, camera_b_pixels, predicted, &stereo_telemetry);
    AnnotateSupportTelemetry(stereo_telemetry, inputs.floor);
    const int seed_count = CountValidSeeds(seeds);
    const int min_seed_count = std::max(
        1,
        monocular_mode ? inputs.quality.monocular.min_seed_count : inputs.quality.min_triangulated_seed_count);

    const float mean_reprojection_error = monocular_mode ? 0.0f : MeanSeedReprojectionError(seeds);
    const bool reprojection_non_finite = !monocular_mode && !std::isfinite(mean_reprojection_error);
    const bool reprojection_degraded = !monocular_mode &&
        !reprojection_non_finite &&
        mean_reprojection_error > inputs.quality.max_mean_reprojection_error_px;

    const float seed_weight = MeanSeedWeight(seeds);
    LowerBodyState state = predicted;

    if (seed_weight > 0.0f) {
        const auto seed_joints = SeedsToJointSet(seeds);
        state = EstimateStateFromJointSeeds(predicted, inputs.model, seed_joints, Clamp01(seed_weight));
        const auto left_foot = InferFootFrame(BodySide::Left, seed_joints, predicted.support.left_foot, predicted.left_foot, inputs.model);
        const auto right_foot = InferFootFrame(BodySide::Right, seed_joints, predicted.support.right_foot, predicted.right_foot, inputs.model);
        if (left_foot.valid) {
            state.left_foot = left_foot.foot_pose;
        }
        if (right_foot.valid) {
            state.right_foot = right_foot.foot_pose;
        }
    }

    const bool hmd_depth_root_anchor = monocular_mode && stereo_telemetry.hmd_depth_scale.corrected_root_valid;
    const StereoHmdAnchorResult stereo_hmd_anchor = ComputeStereoHmdAnchor(inputs, stereo_telemetry);
    if (hmd_depth_root_anchor) {
        state.root.position = stereo_telemetry.hmd_depth_scale.corrected_root_world;
    } else if (stereo_hmd_anchor.applied) {
        state.root.position = stereo_hmd_anchor.corrected_root_world;
    } else {
        state = ApplyHmdPrior(state, inputs, seed_weight > 0.0f ? 0.35f : 0.80f);
    }

    const bool insufficient_global_seeds = seed_count < min_seed_count;
    const bool has_live_seed_evidence = seed_count > 0;
    const bool has_weighted_seed_evidence = seed_weight > 0.0f;
    const bool can_coordinate_optimize = HasStructuredCoordinateEvidence(seeds, min_seed_count);
    const bool can_emit_partial_solve = has_live_seed_evidence;

    if (reprojection_non_finite && !can_emit_partial_solve) {
        int residual_observations = 0;
        const float residual = ComputeResidualPrepared(inputs, camera_a_pixels, camera_b_pixels, state, &residual_observations);
        BodySolveTelemetry telemetry;
        telemetry.tracking_mode = inputs.quality.tracking_mode;
        telemetry.depth_source = stereo_telemetry.depth_source;
        telemetry.stereo = stereo_telemetry;
        telemetry.stereo_hmd_anchor = stereo_hmd_anchor;
        telemetry.degraded = true;
        telemetry.degradation_reason = "non_finite_reprojection_error";
        telemetry.solve_ms = QpcDeltaSeconds(solve_start, NowQpc()) * 1000.0;
        state.confidence = 0.0f;
        state.left_foot_evidence = FootTrackerEvidenceFromStereo(stereo_telemetry, true);
        state.right_foot_evidence = FootTrackerEvidenceFromStereo(stereo_telemetry, false);
        return BodySolveResult{state, residual, residual_observations, telemetry, stereo_telemetry.hmd_depth_scale, stereo_telemetry.hmd_depth_scale_history, stereo_hmd_anchor, false};
    }

    BodySolveTelemetry telemetry;
    if (can_coordinate_optimize) {
        const auto optimized = OptimizeWeightedReprojection(inputs, camera_a_pixels, camera_b_pixels, seeds, state);
        state = optimized.best.state;
        if (hmd_depth_root_anchor) {
            state.root.position = stereo_telemetry.hmd_depth_scale.corrected_root_world;
        } else if (stereo_hmd_anchor.applied) {
            state.root.position = stereo_hmd_anchor.corrected_root_world;
        }
        telemetry = optimized.telemetry;
    } else if (has_live_seed_evidence) {
        telemetry.degraded = true;
        telemetry.degradation_reason = "partial_seed_estimator_only";
    } else {
        telemetry.degraded = true;
        telemetry.degradation_reason = "predictive_hold_without_live_body_seeds";
    }

    int residual_observations = 0;
    const float residual = ComputeResidualPrepared(inputs, camera_a_pixels, camera_b_pixels, state, &residual_observations);
    const float seed_count_factor = insufficient_global_seeds && has_live_seed_evidence
        ? std::max(0.35f, Clamp01(static_cast<float>(seed_count) / static_cast<float>(min_seed_count)))
        : Clamp01(static_cast<float>(seed_count) / static_cast<float>(min_seed_count));
    const float reprojection_factor = monocular_mode
        ? 1.0f
        : (reprojection_degraded
            ? Clamp01(0.20f * inputs.quality.max_mean_reprojection_error_px / std::max(1.0f, mean_reprojection_error))
            : Clamp01(1.0f - mean_reprojection_error / inputs.quality.max_mean_reprojection_error_px));
    const float camera_quality = Clamp01(seed_weight * seed_count_factor * reprojection_factor);
    const float hmd_bonus = inputs.hmd.valid ? 0.10f : 0.0f;
    const float predicted_quality = insufficient_global_seeds
        ? Clamp01(predicted.confidence * 0.70f)
        : 0.0f;
    const float seed_floor = has_weighted_seed_evidence
        ? Clamp01(0.18f + 0.22f * seed_weight + hmd_bonus)
        : hmd_bonus;
    state.confidence = Clamp01(std::max({camera_quality + hmd_bonus, predicted_quality, seed_floor}));
    state.left_foot_evidence = FootTrackerEvidenceFromStereo(stereo_telemetry, true);
    state.right_foot_evidence = FootTrackerEvidenceFromStereo(stereo_telemetry, false);

    telemetry.tracking_mode = inputs.quality.tracking_mode;
    telemetry.depth_source = stereo_telemetry.depth_source;
    telemetry.stereo = stereo_telemetry;
    if (insufficient_global_seeds) {
        telemetry.degraded = true;
        if (telemetry.degradation_reason.empty()) {
            telemetry.degradation_reason = std::string("partial_") + (monocular_mode ? "monocular" : "stereo") +
                "_seeds_" + std::to_string(seed_count) + "_of_" + std::to_string(min_seed_count);
        } else {
            telemetry.degradation_reason += std::string("+partial_") + (monocular_mode ? "monocular" : "stereo") +
                "_seeds_" + std::to_string(seed_count) + "_of_" + std::to_string(min_seed_count);
        }
    }
    if (reprojection_non_finite) {
        telemetry.degraded = true;
        telemetry.degradation_reason = "non_finite_reprojection_error_with_partial_state";
    } else if (reprojection_degraded) {
        telemetry.degraded = true;
        telemetry.degradation_reason = "high_reprojection_error_" + std::to_string(mean_reprojection_error) + "px";
    }
    telemetry.solve_ms = QpcDeltaSeconds(solve_start, NowQpc()) * 1000.0;
    return BodySolveResult{state, residual, residual_observations, telemetry, stereo_telemetry.hmd_depth_scale, stereo_telemetry.hmd_depth_scale_history, stereo_hmd_anchor, can_emit_partial_solve};
}

Result<BodySolveResult> RunFinalSupportAwareSolve(const BodySolveInputs& inputs, const LowerBodyState& preliminary) {
    const auto solve_start = NowQpc();
    LowerBodyState state = PreconditionRootFromSupport(preliminary, inputs);

    const auto camera_a_pixels = PrepareObservedKeypoints(inputs.camera_a_pose, inputs.camera_a_calibration);
    const auto camera_b_pixels = PrepareObservedKeypoints(inputs.camera_b_pose, inputs.camera_b_calibration);
    const bool monocular_mode = inputs.quality.tracking_mode == TrackingMode::Monocular;
    BodySolveStereoTelemetry support_telemetry;
    const auto seeds = monocular_mode
        ? BuildMonocularSeeds(inputs, &support_telemetry)
        : BuildTriangulatedSeeds(inputs, camera_a_pixels, camera_b_pixels, state, &support_telemetry);
    AnnotateSupportTelemetry(support_telemetry, inputs.floor);

    const int seed_count = CountValidSeeds(seeds);
    const int min_seed_count = std::max(
        1,
        monocular_mode ? inputs.quality.monocular.min_seed_count : inputs.quality.min_triangulated_seed_count);
    const bool can_coordinate_optimize = HasStructuredCoordinateEvidence(seeds, min_seed_count);

    BodySolveTelemetry telemetry;
    const bool hmd_depth_root_anchor = monocular_mode && support_telemetry.hmd_depth_scale.corrected_root_valid;
    const StereoHmdAnchorResult stereo_hmd_anchor = ComputeStereoHmdAnchor(inputs, support_telemetry);
    if (hmd_depth_root_anchor) {
        state.root.position = support_telemetry.hmd_depth_scale.corrected_root_world;
    } else if (stereo_hmd_anchor.applied) {
        state.root.position = stereo_hmd_anchor.corrected_root_world;
    }
    if (can_coordinate_optimize) {
        const auto optimized = OptimizeWeightedReprojection(inputs, camera_a_pixels, camera_b_pixels, seeds, state);
        state = optimized.best.state;
        if (hmd_depth_root_anchor) {
            state.root.position = support_telemetry.hmd_depth_scale.corrected_root_world;
        } else if (stereo_hmd_anchor.applied) {
            state.root.position = stereo_hmd_anchor.corrected_root_world;
        }
        telemetry = optimized.telemetry;
    } else {
        telemetry.degraded = true;
        telemetry.degradation_reason = std::string("support_partial_seed_estimator_only_") +
            std::to_string(seed_count) + "_of_" + std::to_string(min_seed_count);
    }
    state.support = preliminary.support;

    BodySolveSupportConstraintTelemetry initial_constraint_telemetry;
    (void)SupportConstraintPenalty(state, inputs, &initial_constraint_telemetry);
    DegradeSupportFromResiduals(state, initial_constraint_telemetry);
    BodySolveSupportConstraintTelemetry constraint_telemetry;
    (void)SupportConstraintPenalty(state, inputs, &constraint_telemetry);
    constraint_telemetry.left_foot.degraded_or_released =
        initial_constraint_telemetry.left_foot.degraded_or_released;
    constraint_telemetry.right_foot.degraded_or_released =
        initial_constraint_telemetry.right_foot.degraded_or_released;
    state.left_foot_evidence = ApplySupportToFootEvidence(state.left_foot_evidence, state.support.left_foot);
    state.right_foot_evidence = ApplySupportToFootEvidence(state.right_foot_evidence, state.support.right_foot);

    int residual_observations = 0;
    const float residual = ComputeResidualPrepared(inputs, camera_a_pixels, camera_b_pixels, state, &residual_observations);
    telemetry.tracking_mode = inputs.quality.tracking_mode;
    telemetry.depth_source = inputs.quality.tracking_mode == TrackingMode::Monocular
        ? DepthSource::InferredMonocular
        : DepthSource::TriangulatedStereo;
    telemetry.stereo = support_telemetry;
    telemetry.constraints = constraint_telemetry;
    if (constraint_telemetry.left_foot.degraded_or_released ||
        constraint_telemetry.right_foot.degraded_or_released) {
        telemetry.degraded = true;
        if (telemetry.degradation_reason.empty()) {
            telemetry.degradation_reason = "support_constraints_degraded";
        } else {
            telemetry.degradation_reason += "+support_constraints_degraded";
        }
    }
    telemetry.solve_ms = QpcDeltaSeconds(solve_start, NowQpc()) * 1000.0;
    return BodySolveResult{state, residual, residual_observations, telemetry, support_telemetry.hmd_depth_scale, support_telemetry.hmd_depth_scale_history, stereo_hmd_anchor, true};
}

} // namespace bt


