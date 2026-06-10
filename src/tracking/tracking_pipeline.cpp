#include "tracking/tracking_pipeline.h"

#include "calibration/calibration_io.h"
#include "tracking/foot_support.h"
#include "tracking/geometry_cache.h"
#include "tracking/identity_assignment.h"
#include "tracking/joint_limits.h"
#include "tracking/monocular_projection.h"
#include "tracking/root_support.h"
#include "tracking/support_queries.h"
#include "tracking/temporal_update.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <utility>

namespace bt {
namespace {

constexpr float kOcclusionMinConfidenceToSend = 0.16f;
constexpr float kBootstrapHmdConfidence = 0.18f;
constexpr float kSupportedFootAnchorGain = 0.96f;
constexpr float kUnsupportedFootRootDrift = 0.35f;
constexpr double kDefaultDtSeconds = 1.0 / 60.0;
constexpr double kMinDtSeconds = 1.0 / 240.0;
constexpr double kMaxDtSeconds = 0.10;
constexpr float kDefaultMonocularUserHeightM = 1.70f;

float Clamp01(float value) {
    if (!std::isfinite(value)) {
        return 0.0f;
    }
    return std::max(0.0f, std::min(1.0f, value));
}

float Clamp(float value, float lo, float hi) {
    if (!std::isfinite(value)) {
        return lo;
    }
    return std::max(lo, std::min(hi, value));
}

double ClampDt(double value) {
    if (!std::isfinite(value) || value <= 0.0) {
        return kDefaultDtSeconds;
    }
    return std::max(kMinDtSeconds, std::min(kMaxDtSeconds, value));
}

bool QuatFinite(const Quatf& q) {
    const float len_sq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    return std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) &&
        std::isfinite(q.w) && std::isfinite(len_sq) && len_sq > 1e-12f;
}

bool PoseFinite(const Pose3f& pose) {
    return IsFinite(pose.position) && QuatFinite(pose.orientation);
}

bool LowerBodyStateOutputValid(const LowerBodyState& state) {
    return PoseFinite(state.root) && PoseFinite(state.left_foot) && PoseFinite(state.right_foot);
}

bool CameraEvidencePresent(
    std::uint64_t sequence,
    const DecodedPose2D& pose,
    const ReliabilitySummary& reliability) {

    return sequence != 0 || pose.valid || reliability.mean_weight > 0.0f ||
        reliability.lower_body_mean > 0.0f || reliability.foot_mean > 0.0f;
}

bool CameraTimestampValid(double timestamp_seconds) {
    return std::isfinite(timestamp_seconds) && timestamp_seconds > 0.0;
}

bool CameraEvidenceIdentityAvailable(std::uint64_t sequence, double timestamp_seconds) {
    return sequence != 0 || CameraTimestampValid(timestamp_seconds);
}

bool SameCameraEvidenceIdentity(
    std::uint64_t sequence,
    double timestamp_seconds,
    std::uint64_t last_sequence,
    double last_timestamp_seconds) {

    if (sequence != 0 && last_sequence != 0) {
        return sequence == last_sequence;
    }
    if (CameraTimestampValid(timestamp_seconds) && CameraTimestampValid(last_timestamp_seconds)) {
        return std::abs(timestamp_seconds - last_timestamp_seconds) < 1e-9;
    }
    return false;
}

double CameraEvidenceTimestampSeconds(const BodySolveInputs& inputs, bool has_a, bool has_b) {
    double latest = 0.0;
    if (has_a && CameraTimestampValid(inputs.camera_a_timestamp_seconds)) {
        latest = std::max(latest, inputs.camera_a_timestamp_seconds);
    }
    if (has_b && CameraTimestampValid(inputs.camera_b_timestamp_seconds)) {
        latest = std::max(latest, inputs.camera_b_timestamp_seconds);
    }
    return latest;
}

TrackerPoseArray InvalidTrackers() {
    return TrackerPoseArray{
        TrackerPose{TrackerRole::Pelvis, Pose3f{}, 0.0f, false},
        TrackerPose{TrackerRole::LeftFoot, Pose3f{}, 0.0f, false},
        TrackerPose{TrackerRole::RightFoot, Pose3f{}, 0.0f, false},
        TrackerPose{TrackerRole::Chest, Pose3f{}, 0.0f, false},
        TrackerPose{TrackerRole::LeftElbow, Pose3f{}, 0.0f, false},
        TrackerPose{TrackerRole::RightElbow, Pose3f{}, 0.0f, false},
        TrackerPose{TrackerRole::LeftKnee, Pose3f{}, 0.0f, false},
        TrackerPose{TrackerRole::RightKnee, Pose3f{}, 0.0f, false}
    };
}

PostureMode StablePostureMode(PostureMode current, PostureMode fallback) {
    return current == PostureMode::UnknownFree ? fallback : current;
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

FloorGeometryCalibration SanitizeMonocularFloorGeometryForImageSpace(
    FloorGeometryCalibration geometry,
    const MonocularTrackingConfig& config) {

    if (FloorGeometryImageSpaceMatches(geometry, config)) {
        return geometry;
    }

    const auto identity = std::array<float, 9>{1.0f, 0.0f, 0.0f,
                                                0.0f, 1.0f, 0.0f,
                                                0.0f, 0.0f, 1.0f};
    geometry.valid = false;
    geometry.family_a.valid = false;
    geometry.family_a.metric_spacing_valid = false;
    geometry.family_b.valid = false;
    geometry.family_b.metric_spacing_valid = false;
    geometry.family_count = 0;
    geometry.metric_scale_confidence = 0.0f;
    geometry.floor_plane.valid = false;
    geometry.floor_plane_confidence = 0.0f;
    geometry.homography_valid = false;
    geometry.floor_from_image = identity;
    geometry.image_from_floor = identity;
    geometry.homography_reason = "disabled_image_size_mismatch";
    geometry.camera_orientation_valid = false;
    geometry.camera_orientation_confidence = 0.0f;
    geometry.camera_orientation_applied_to_runtime = false;
    geometry.distortion.valid = false;
    geometry.distortion.applied_to_runtime = false;
    geometry.reason = "floor_geometry_image_size_mismatch_saved_" +
        std::to_string(geometry.image_width) + "x" + std::to_string(geometry.image_height) +
        "_runtime_" + std::to_string(config.image_width) + "x" + std::to_string(config.image_height);
    return geometry;
}

MonocularTrackingConfig MonocularImageSpaceForSize(int width, int height) {
    MonocularTrackingConfig config;
    config.image_width = width;
    config.image_height = height;
    return config;
}

std::vector<WallRectangleCalibration> SanitizeWallRectanglesForImageSpace(
    const std::vector<WallRectangleCalibration>& walls,
    int width,
    int height) {

    std::vector<WallRectangleCalibration> out;
    for (auto wall : walls) {
        if (!wall.valid) {
            continue;
        }
        if (wall.image_width != width || wall.image_height != height) {
            wall.valid = false;
            wall.wall_orientation_valid = false;
            wall.applied_to_runtime = false;
            wall.reason = "wall_rectangle_image_size_mismatch_saved_" +
                std::to_string(wall.image_width) + "x" + std::to_string(wall.image_height) +
                "_runtime_" + std::to_string(width) + "x" + std::to_string(height);
        }
        if (wall.valid) {
            out.push_back(wall);
        }
    }
    return out;
}

LowerBodyModel ApplyMonocularUserScale(LowerBodyModel model, const MonocularTrackingConfig& config) {
    if (!std::isfinite(config.user_height_m) || config.user_height_m <= 0.0f) {
        return model;
    }

    // The default body model is calibrated for the default 1.70 m markerless
    // profile. In monocular mode user height is the only explicit body-scale
    // input, so scale lower-body segment priors consistently while leaving
    // stereo/body-calibrated mode untouched.
    const float scale = config.user_height_m / kDefaultMonocularUserHeightM;
    model.pelvis_width *= scale;
    model.left_femur *= scale;
    model.right_femur *= scale;
    model.left_tibia *= scale;
    model.right_tibia *= scale;
    model.left_foot_length *= scale;
    model.right_foot_length *= scale;
    model.standing_hmd_to_pelvis = Scale(model.standing_hmd_to_pelvis, scale);
    model.seated_hmd_to_pelvis = Scale(model.seated_hmd_to_pelvis, scale);
    model.reclined_hmd_to_pelvis = Scale(model.reclined_hmd_to_pelvis, scale);
    return model;
}

Quatf YawOnly(const Quatf& orientation) {
    const Quatf q = Normalize(orientation);
    const Vec3f forward = Rotate(q, Vec3f{0.0f, 0.0f, 1.0f});
    const float yaw = std::atan2(forward.x, forward.z);
    const float half = 0.5f * yaw;
    return Quatf{0.0f, std::sin(half), 0.0f, std::cos(half)};
}

float DecayConfidence(float confidence, bool hmd_valid, double dt_seconds) {
    const float rate = hmd_valid ? 1.10f : 2.00f;
    return Clamp01(confidence * std::exp(-rate * static_cast<float>(dt_seconds)));
}

Pose3f HeldOrDriftedFoot(
    const Pose3f& previous_foot,
    const FootSupportState& support,
    const Vec3f& root_delta) {

    if (IsActiveFootSupport(support)) {
        Pose3f out = previous_foot;
        const float gain = kSupportedFootAnchorGain * support.anchor.confidence;
        out.position = Lerp(previous_foot.position, support.anchor.pose.position, gain);
        out.orientation = Slerp(previous_foot.orientation, support.anchor.pose.orientation, gain);
        return out;
    }

    Pose3f out = previous_foot;
    out.position = Add(previous_foot.position, Scale(root_delta, kUnsupportedFootRootDrift));
    return out;
}

void Solve3DLegsFromFootTargets(LowerBodyState& state, const LowerBodyModel& model) {
    SolveLeg3DFromFootTarget(state, model, true);
    SolveLeg3DFromFootTarget(state, model, false);
}

void MarkOccludedFootEvidence(LowerBodyState& out, const LowerBodyState& previous, bool hmd_valid) {
    const auto make_evidence = [hmd_valid](const FootSupportState& support) {
        TrackerEvidence evidence;
        evidence.support_confidence = FootSupportConfidence(support);
        evidence.anchor_held = IsActiveFootSupport(support);
        if (IsActiveFootSupport(support)) {
            evidence.source = TrackerEvidenceSource::AnchorHeld;
            evidence.valid = evidence.support_confidence > 0.0f;
        } else {
            evidence.source = hmd_valid ? TrackerEvidenceSource::HmdPrediction : TrackerEvidenceSource::Predicted;
            evidence.valid = hmd_valid;
        }
        return evidence;
    };
    out.left_foot_evidence = make_evidence(previous.support.left_foot);
    out.right_foot_evidence = make_evidence(previous.support.right_foot);
}


void DecayOccludedFootSupport(
    LowerBodyState& out,
    const LowerBodyState& previous,
    const LowerBodyModel& model,
    PostureMode mode,
    const FloorPlane& floor,
    double dt_seconds);

LowerBodyState BootstrapFromHmd(const BodySolveInputs& inputs, PostureMode mode) {
    LowerBodyState state;
    state.posture_mode = mode;
    state.root.position = Add(inputs.hmd.pose.position, ModeHmdOffset(inputs.model, mode));
    state.root.orientation = YawOnly(inputs.hmd.pose.orientation);
    state.confidence = kBootstrapHmdConfidence;

    const auto joints = PredictLowerBodyJoints(state, inputs.model);
    const auto left_ankle = joints.joints[static_cast<std::size_t>(KeypointId::LeftAnkle)];
    const auto right_ankle = joints.joints[static_cast<std::size_t>(KeypointId::RightAnkle)];
    if (left_ankle.present) {
        state.left_foot = FootPoseFromAnkleTarget(left_ankle.world, state.left_foot, inputs.model, true);
    }
    if (right_ankle.present) {
        state.right_foot = FootPoseFromAnkleTarget(right_ankle.world, state.right_foot, inputs.model, false);
    }
    state.left_foot_evidence.source = TrackerEvidenceSource::HmdPrediction;
    state.left_foot_evidence.valid = inputs.hmd.valid;
    state.right_foot_evidence.source = TrackerEvidenceSource::HmdPrediction;
    state.right_foot_evidence.valid = inputs.hmd.valid;
    return state;
}

LowerBodyState EstimateOccludedState(
    const LowerBodyState& previous,
    const LowerBodyState& predicted,
    const BodySolveInputs& inputs,
    const PostureClassifierState& posture,
    double dt_seconds) {

    const bool has_history = previous.confidence > 0.01f;
    const PostureMode mode = StablePostureMode(posture.mode, previous.posture_mode);

    if (!has_history && !inputs.hmd.valid) {
        LowerBodyState untracked = previous;
        untracked.posture_mode = mode;
        untracked.linear_velocity = {};
        untracked.angular_velocity = {};
        untracked.confidence = 0.0f;
        return ApplyJointLimitBounds(untracked);
    }

    LowerBodyState out = has_history ? predicted : BootstrapFromHmd(inputs, mode);
    out.posture_mode = mode;
    out.support = previous.support;

    const Vec3f root_before = out.root.position;
    if (inputs.hmd.valid) {
        const Vec3f hmd_root = Add(inputs.hmd.pose.position, ModeHmdOffset(inputs.model, mode));
        const bool anchored_root = IsActiveRootSupport(previous.support);
        const float hmd_gain = anchored_root ? 0.30f : 0.55f;
        out.root.position = Lerp(out.root.position, hmd_root, hmd_gain);
        out.root.orientation = YawOnly(inputs.hmd.pose.orientation);
    }

    const Vec3f root_delta = Sub(out.root.position, root_before);
    out.left_foot = HeldOrDriftedFoot(previous.left_foot, previous.support.left_foot, root_delta);
    out.right_foot = HeldOrDriftedFoot(previous.right_foot, previous.support.right_foot, root_delta);
    DecayOccludedFootSupport(out, previous, inputs.model, mode, inputs.floor, dt_seconds);
    MarkOccludedFootEvidence(out, previous, inputs.hmd.valid);

    Solve3DLegsFromFootTargets(out, inputs.model);

    const float decayed = has_history
        ? DecayConfidence(previous.confidence, inputs.hmd.valid, dt_seconds)
        : (inputs.hmd.valid ? kBootstrapHmdConfidence : 0.0f);
    out.confidence = inputs.hmd.valid
        ? std::max(kOcclusionMinConfidenceToSend, decayed)
        : decayed;
    if (!has_history && !inputs.hmd.valid) {
        out.confidence = 0.0f;
    }
    return ApplyJointLimitBounds(out);
}

TrackingStagePoseSnapshot StagePoseFromState(const LowerBodyState& state, bool valid = true) {
    TrackingStagePoseSnapshot out;
    out.valid = valid;
    out.root = state.root;
    out.left_foot = state.left_foot;
    out.right_foot = state.right_foot;
    out.confidence = state.confidence;
    return out;
}

float SolverObservationWeightScale(const BodySolveJointTriangulationTelemetry& joint) {
    if (!joint.solver_uncertainty_weighted) {
        return 1.0f;
    }
    const float scale = std::min(joint.solver_lateral_weight_scale, joint.solver_depth_weight_scale);
    return std::isfinite(scale) && scale > 0.0f ? scale : 1.0f;
}

TrackingPipelineSnapshot SnapshotFromState(
    const LowerBodyState& state,
    const LowerBodyModel& model,
    const BodyCalibration& body_calibration,
    const PostureClassifierState& posture,
    const MotionConsistencyTelemetry& motion_filter,
    TrackerEkfTelemetry tracker_ekf,
    TrackingSolverTelemetry solver,
    TrackingPipelineStages stages,
    double dt_seconds,
    std::string mode,
    std::string error) {

    TrackingPipelineSnapshot snapshot;
    snapshot.state = state;
    snapshot.posture = posture;
    snapshot.motion_filter = motion_filter;
    snapshot.tracker_ekf = tracker_ekf;
    snapshot.solver = std::move(solver);
    snapshot.stages = stages;
    BodyStateSolverSnapshot body_solver;
    body_solver.tracking_mode = snapshot.solver.tracking_mode;
    body_solver.depth_source = snapshot.solver.depth_source;
    body_solver.degraded = snapshot.solver.degraded;
    body_solver.used_hmd = snapshot.solver.used_hmd;
    body_solver.reason = snapshot.solver.reason;
    body_solver.camera_a_identity_swapped = snapshot.solver.camera_a_identity_swapped;
    body_solver.camera_b_identity_swapped = snapshot.solver.camera_b_identity_swapped;
    body_solver.camera_a_identity_consistency = snapshot.solver.camera_a_identity_consistency;
    body_solver.camera_b_identity_consistency = snapshot.solver.camera_b_identity_consistency;
    body_solver.triangulated_count = snapshot.solver.preliminary_stereo.triangulated_count;
    body_solver.inferred_depth_count = snapshot.solver.preliminary_stereo.inferred_depth_count;
    body_solver.mean_reprojection_error_px = snapshot.solver.preliminary_stereo.mean_reprojection_error_px;
    for (std::size_t i = 0; i < body_solver.joints.size(); ++i) {
        const auto& src = snapshot.solver.preliminary_stereo.joints[i];
        auto& dst = body_solver.joints[i];
        dst.camera_a_present = src.camera_a_present;
        dst.camera_b_present = src.camera_b_present;
        dst.camera_a_confidence = src.camera_a_confidence;
        dst.camera_b_confidence = src.camera_b_confidence;
        dst.camera_a_weight = src.camera_a_weight;
        dst.camera_b_weight = src.camera_b_weight;
        dst.camera_a_quality = src.camera_a_quality;
        dst.camera_b_quality = src.camera_b_quality;
        dst.evidence_source = src.evidence_source;
        dst.triangulated = src.triangulated;
        dst.depth_inferred = src.depth_inferred;
        dst.depth_source = src.depth_source;
        dst.world = src.world;
        dst.confidence = src.confidence;
        dst.mean_reprojection_error_px = src.mean_reprojection_error_px;
        dst.estimated_depth_m = src.estimated_depth_m;
        dst.contact_confidence = src.foot_contact_confidence;
        dst.solver_observation_weighted = src.solver_uncertainty_weighted;
        dst.solver_observation_weight_scale = SolverObservationWeightScale(src);
        dst.solver_observation_confidence_ceiling = src.solver_observation_confidence_ceiling;
    }
    snapshot.body_state = BuildUnifiedBodyState(
        state,
        model,
        BuildBodyStateEvidence(body_solver, state),
        body_calibration,
        dt_seconds,
        LowerBodyStateOutputValid(state));
    snapshot.trackers = SynthesizeTrackerPoses(snapshot.body_state, model);
    snapshot.degradation_mode = std::move(mode);
    snapshot.last_error = std::move(error);
    return snapshot;
}

BodyStateSolverSnapshot BodyStateSolverFromTelemetry(const TrackingSolverTelemetry& solver) {
    BodyStateSolverSnapshot out;
    out.tracking_mode = solver.tracking_mode;
    out.depth_source = solver.depth_source;
    out.degraded = solver.degraded;
    out.used_hmd = solver.used_hmd;
    out.reason = solver.reason;
    out.camera_a_identity_swapped = solver.camera_a_identity_swapped;
    out.camera_b_identity_swapped = solver.camera_b_identity_swapped;
    out.camera_a_identity_consistency = solver.camera_a_identity_consistency;
    out.camera_b_identity_consistency = solver.camera_b_identity_consistency;
    out.triangulated_count = solver.preliminary_stereo.triangulated_count;
    out.inferred_depth_count = solver.preliminary_stereo.inferred_depth_count;
    out.mean_reprojection_error_px = solver.preliminary_stereo.mean_reprojection_error_px;
    for (std::size_t i = 0; i < out.joints.size(); ++i) {
        const auto& src = solver.preliminary_stereo.joints[i];
        auto& dst = out.joints[i];
        dst.camera_a_present = src.camera_a_present;
        dst.camera_b_present = src.camera_b_present;
        dst.camera_a_confidence = src.camera_a_confidence;
        dst.camera_b_confidence = src.camera_b_confidence;
        dst.camera_a_weight = src.camera_a_weight;
        dst.camera_b_weight = src.camera_b_weight;
        dst.camera_a_quality = src.camera_a_quality;
        dst.camera_b_quality = src.camera_b_quality;
        dst.evidence_source = src.evidence_source;
        dst.triangulated = src.triangulated;
        dst.depth_inferred = src.depth_inferred;
        dst.depth_source = src.depth_source;
        dst.world = src.world;
        dst.confidence = src.confidence;
        dst.mean_reprojection_error_px = src.mean_reprojection_error_px;
        dst.estimated_depth_m = src.estimated_depth_m;
        dst.contact_confidence = src.foot_contact_confidence;
        dst.solver_observation_weighted = src.solver_uncertainty_weighted;
        dst.solver_observation_weight_scale = SolverObservationWeightScale(src);
        dst.solver_observation_confidence_ceiling = src.solver_observation_confidence_ceiling;
    }
    return out;
}

void CopyIdentityTelemetry(
    TrackingSolverTelemetry& solver,
    const IdentityAssignmentResult& camera_a,
    const IdentityAssignmentResult& camera_b) {

    solver.camera_a_identity_swapped = camera_a.swapped;
    solver.camera_b_identity_swapped = camera_b.swapped;
    solver.camera_a_identity_consistency = camera_a.consistency;
    solver.camera_b_identity_consistency = camera_b.consistency;
}

void CopyIdentityTelemetry(
    TrackingSolverTelemetry& solver,
    const StereoIdentityAssignmentResult& identity) {

    CopyIdentityTelemetry(solver, identity.camera_a, identity.camera_b);
    solver.identity_epipolar_arbitration_checked = identity.epipolar_arbitration_checked;
    solver.identity_epipolar_arbitration_applied = identity.epipolar_arbitration_applied;
    solver.identity_epipolar_scored_lateral_pairs = identity.epipolar_scored_lateral_pairs;
    solver.identity_epipolar_same_score = identity.epipolar_same_identity_score;
    solver.identity_epipolar_cross_score = identity.epipolar_cross_identity_score;
    solver.identity_epipolar_cross_geometric_uncertainty = identity.epipolar_cross_geometric_uncertainty;
    solver.identity_epipolar_detection_support = identity.epipolar_detection_support;
    solver.identity_epipolar_required_swap_margin = identity.epipolar_required_swap_margin;
    solver.identity_same_mahalanobis_sq = identity.identity_same_mahalanobis_sq;
    solver.identity_cross_mahalanobis_sq = identity.identity_cross_mahalanobis_sq;
    solver.identity_same_negative_log_likelihood = identity.identity_same_negative_log_likelihood;
    solver.identity_cross_negative_log_likelihood = identity.identity_cross_negative_log_likelihood;
    solver.identity_cross_within_mahalanobis_gate = identity.identity_cross_within_mahalanobis_gate;
    solver.identity_score_gate_passed = identity.identity_score_gate_passed;
    solver.identity_likelihood_gate_passed = identity.identity_likelihood_gate_passed;
    solver.identity_swap_blocked_by_strong_consistency = identity.identity_swap_blocked_by_strong_consistency;
    solver.identity_swap_blocked_by_tie = identity.identity_swap_blocked_by_tie;
    solver.identity_uncertainty_fallback_count = identity.identity_uncertainty_fallback_count;
}

TrackingSolverTelemetry SolverTelemetry(
    bool hmd_valid,
    bool degraded,
    std::string reason,
    TrackingMode mode = TrackingMode::Stereo,
    DepthSource source = DepthSource::None) {

    TrackingSolverTelemetry solver;
    solver.tracking_mode = mode;
    solver.depth_source = source;
    solver.used_hmd = hmd_valid;
    solver.degraded = degraded;
    solver.reason = std::move(reason);
    return solver;
}

bool TrackerPoseUsable(const TrackerPose& tracker) {
    return tracker.valid &&
        tracker.confidence > 0.0f &&
        IsFinite(tracker.pose.position) &&
        std::isfinite(tracker.pose.orientation.x) &&
        std::isfinite(tracker.pose.orientation.y) &&
        std::isfinite(tracker.pose.orientation.z) &&
        std::isfinite(tracker.pose.orientation.w);
}

bool PrimaryReplayTrackerRole(TrackerRole role) {
    return role == TrackerRole::Pelvis ||
        role == TrackerRole::LeftFoot ||
        role == TrackerRole::RightFoot;
}

Pose3f TrackerPoseOr(const TrackerPoseArray& trackers, TrackerRole role, Pose3f fallback) {
    const std::size_t role_index = TrackerRoleIndex(role);
    if (role_index < trackers.size() && TrackerPoseUsable(trackers[role_index])) {
        return trackers[role_index].pose;
    }
    return fallback;
}

float TrackerConfidence(const TrackerPoseArray& trackers) {
    float sum = 0.0f;
    int count = 0;
    for (const TrackerRole role : kTrackerRoles) {
        if (!PrimaryReplayTrackerRole(role)) {
            continue;
        }
        const std::size_t role_index = TrackerRoleIndex(role);
        if (role_index < trackers.size() && TrackerPoseUsable(trackers[role_index])) {
            sum += trackers[role_index].confidence;
            ++count;
        }
    }
    return count > 0 ? Clamp01(sum / static_cast<float>(count)) : 0.0f;
}

const TrackerPose* ValidTrackerForRole(const TrackerPoseArray& trackers, TrackerRole role) {
    const std::size_t role_index = TrackerRoleIndex(role);
    if (role_index < trackers.size() && TrackerPoseUsable(trackers[role_index])) {
        return &trackers[role_index];
    }
    return nullptr;
}

float TrackerConfidenceForRole(const TrackerPoseArray& trackers, TrackerRole role) {
    if (const auto* tracker = ValidTrackerForRole(trackers, role)) {
        return Clamp01(tracker->confidence);
    }
    return 0.0f;
}

bool BodyRoleForTrackerRole(TrackerRole tracker_role, BodyJointRole& body_role_out) {
    switch (tracker_role) {
    case TrackerRole::Pelvis:
        body_role_out = BodyJointRole::Pelvis;
        return true;
    case TrackerRole::LeftFoot:
        body_role_out = BodyJointRole::LeftFoot;
        return true;
    case TrackerRole::RightFoot:
        body_role_out = BodyJointRole::RightFoot;
        return true;
    case TrackerRole::Chest:
        body_role_out = BodyJointRole::Chest;
        return true;
    case TrackerRole::LeftElbow:
        body_role_out = BodyJointRole::LeftElbow;
        return true;
    case TrackerRole::RightElbow:
        body_role_out = BodyJointRole::RightElbow;
        return true;
    case TrackerRole::LeftKnee:
        body_role_out = BodyJointRole::LeftKnee;
        return true;
    case TrackerRole::RightKnee:
        body_role_out = BodyJointRole::RightKnee;
        return true;
    default:
        return false;
    }
}

void ApplyReplayTrackerRoleToBodyState(
    UnifiedBodyState& body_state,
    TrackerRole tracker_role,
    const TrackerPose& tracker) {

    BodyJointRole body_role;
    if (!BodyRoleForTrackerRole(tracker_role, body_role)) {
        return;
    }

    auto& joint = body_state.roles[BodyJointRoleIndex(body_role)];
    joint.role = body_role;
    joint.position = tracker.pose.position;
    joint.velocity = Vec3f{};
    joint.confidence = Clamp01(tracker.confidence);
    joint.valid = joint.confidence > 0.0f && IsFinite(joint.position);
    joint.visibility = joint.valid ? BodyJointVisibility::Visible : BodyJointVisibility::MissingUnknown;
    joint.evidence = tracker.evidence;
    if (joint.evidence.source == TrackerEvidenceSource::None) {
        joint.evidence.source = TrackerEvidenceSource::ReplayInput;
    }
    joint.evidence.direct_confidence = std::max(joint.evidence.direct_confidence, joint.confidence);
    joint.evidence.valid = joint.valid;
    joint.measured = joint.valid;
    joint.predicted = false;
    joint.depth_source = DepthSource::None;
    joint.evidence_source = JointEvidenceSource::Fallback;
    joint.triangulated = false;
    joint.depth_inferred = false;
    joint.reason = "replay_tracker_input";
}

bool IsSupplementalReplayTrackerRole(TrackerRole role) {
    return role == TrackerRole::Chest ||
        role == TrackerRole::LeftElbow ||
        role == TrackerRole::RightElbow ||
        role == TrackerRole::LeftKnee ||
        role == TrackerRole::RightKnee;
}

void ApplyReplayTrackerRolesToBodyState(
    UnifiedBodyState& body_state,
    const TrackerPoseArray& replay_trackers) {

    for (const TrackerRole role : kTrackerRoles) {
        if (!IsSupplementalReplayTrackerRole(role)) {
            continue;
        }
        if (const auto* tracker = ValidTrackerForRole(replay_trackers, role)) {
            ApplyReplayTrackerRoleToBodyState(body_state, role, *tracker);
        }
    }
}

void PreserveReplayTrackerPoses(
    TrackerPoseArray& synthesized,
    const TrackerPoseArray& replay_trackers) {

    for (const TrackerRole role : kTrackerRoles) {
        if (!IsSupplementalReplayTrackerRole(role)) {
            continue;
        }
        if (const auto* replay = ValidTrackerForRole(replay_trackers, role)) {
            auto out = *replay;
            out.role = role;
            out.confidence = Clamp01(out.confidence);
            out.valid = out.confidence > 0.0f;
            if (out.evidence.source == TrackerEvidenceSource::None) {
                out.evidence.source = TrackerEvidenceSource::ReplayInput;
            }
            out.evidence.direct_confidence = std::max(out.evidence.direct_confidence, out.confidence);
            out.evidence.valid = out.valid;
            synthesized[TrackerRoleIndex(role)] = out;
        }
    }
}

FootSupportEvidence FootEvidence(float contact_confidence, float heel_confidence, float toe_confidence) {
    FootSupportEvidence evidence;
    evidence.confidence = Clamp01(contact_confidence);
    evidence.heel_confidence = Clamp01(heel_confidence);
    evidence.toe_confidence = Clamp01(toe_confidence);
    evidence.heel_usable = evidence.heel_confidence > 0.0f;
    evidence.toe_usable = evidence.toe_confidence > 0.0f;
    evidence.usable = evidence.confidence > 0.0f || evidence.heel_usable || evidence.toe_usable;
    return evidence;
}

FootSupportEvidence FootEvidence(float contact_confidence) {
    return FootEvidence(contact_confidence, contact_confidence, contact_confidence);
}

FootSupportEvidence MissingFootEvidence() {
    FootSupportEvidence evidence;
    evidence.confidence = 0.0f;
    evidence.heel_confidence = 0.0f;
    evidence.toe_confidence = 0.0f;
    evidence.heel_usable = false;
    evidence.toe_usable = false;
    evidence.usable = false;
    return evidence;
}

KneeContactEvidence MissingKneeEvidence() {
    return KneeContactEvidence{};
}

KneeContactEvidence KneeEvidence(const BodySolveStereoTelemetry& stereo) {
    KneeContactEvidence evidence;
    evidence.left_confidence = Clamp01(stereo.left_knee_floor_contact_confidence);
    evidence.right_confidence = Clamp01(stereo.right_knee_floor_contact_confidence);
    evidence.left_usable = stereo.left_knee_floor_contact_observed;
    evidence.right_usable = stereo.right_knee_floor_contact_observed;
    return evidence;
}

void DecayOccludedFootSupport(
    LowerBodyState& out,
    const LowerBodyState& previous,
    const LowerBodyModel& model,
    PostureMode mode,
    const FloorPlane& floor,
    double dt_seconds) {

    const auto missing_evidence = MissingFootEvidence();
    out.support.left_foot = UpdateFootSupportCalibrated(
        previous.support.left_foot,
        out.left_foot,
        previous.left_foot,
        mode,
        floor,
        dt_seconds,
        model.left_foot_length,
        FootSupportConfig{},
        &missing_evidence);
    out.support.right_foot = UpdateFootSupportCalibrated(
        previous.support.right_foot,
        out.right_foot,
        previous.right_foot,
        mode,
        floor,
        dt_seconds,
        model.right_foot_length,
        FootSupportConfig{},
        &missing_evidence);
    out.support = UpdateKneeContactSupport(
        out.support,
        out,
        model,
        floor,
        mode,
        dt_seconds,
        MissingKneeEvidence());
    out.support = UpdateRootSupport(out.support, out, mode, dt_seconds);
}

bool Differs(float a, float b) {
    return std::abs(a - b) > 1e-6f;
}

void CopyBodyCalibrationPersistState(
    BodyCalibrationTelemetry& telemetry,
    const BodyCalibrationEstimatorState& estimator,
    const BodyCalibrationModeConfig& config) {
    telemetry.auto_persist = config.auto_persist;
    telemetry.persisted = estimator.persisted;
    telemetry.persist_pending = estimator.persist_pending;
    telemetry.persist_status = estimator.persist_status;
    telemetry.persist_error = estimator.persist_error;
}

void SetBodyCalibrationPersistState(
    BodyCalibrationEstimatorState& estimator,
    BodyCalibrationTelemetry& telemetry,
    const BodyCalibrationModeConfig& config,
    bool persisted,
    bool pending,
    std::string status,
    std::string error = {}) {
    estimator.persisted = persisted;
    estimator.persist_pending = pending;
    estimator.persist_status = std::move(status);
    estimator.persist_error = std::move(error);
    CopyBodyCalibrationPersistState(telemetry, estimator, config);
}

void MaybeUpdateBodyCalibration(
    CalibrationBundle& calibration,
    LowerBodyModel& model,
    BodyCalibrationEstimatorState& estimator,
    BodyCalibrationTelemetry& telemetry,
    const TrackingConfig& config,
    const BodySolveStereoTelemetry& solve_telemetry,
    const HmdPoseSample& hmd,
    double dt_seconds) {

    telemetry = UpdateBodyCalibrationEstimator(
        estimator,
        config.body_calibration,
        solve_telemetry,
        hmd,
        dt_seconds);

    if (!config.body_calibration.enabled) {
        SetBodyCalibrationPersistState(estimator, telemetry, config.body_calibration, false, false, "disabled");
        return;
    }
    if (!telemetry.complete) {
        SetBodyCalibrationPersistState(estimator, telemetry, config.body_calibration, false, false, "not_complete");
        return;
    }
    if (estimator.dirty) {
        calibration.body = telemetry.body;
        model = MakeLowerBodyModel(calibration.body);
    }
    if (!config.body_calibration.auto_persist) {
        SetBodyCalibrationPersistState(estimator, telemetry, config.body_calibration, false, false, "auto_persist_disabled");
        return;
    }
    if (!estimator.dirty) {
        const bool persisted = estimator.persisted;
        SetBodyCalibrationPersistState(
            estimator,
            telemetry,
            config.body_calibration,
            persisted,
            !persisted,
            persisted ? "persisted" : "complete_pending_persist",
            estimator.persist_error);
        return;
    }

    const Status saved = SaveCalibrationBundle(calibration, config.calibration_path);
    telemetry.saved_this_frame = saved.ok();
    if (saved.ok()) {
        estimator.dirty = false;
        SetBodyCalibrationPersistState(
            estimator,
            telemetry,
            config.body_calibration,
            true,
            false,
            "saved_this_frame");
    } else {
        SetBodyCalibrationPersistState(
            estimator,
            telemetry,
            config.body_calibration,
            false,
            true,
            "save_failed",
            saved.message);
        telemetry.reason = std::string("complete_but_save_failed: ") + saved.message;
    }
}

bool MotionConsistencyConfigChanged(const MotionConsistencyConfig& a, const MotionConsistencyConfig& b) {
    return a.enabled != b.enabled ||
        a.confirm_frames != b.confirm_frames ||
        Differs(a.min_motion_m, b.min_motion_m) ||
        Differs(a.stationary_deadzone_m, b.stationary_deadzone_m) ||
        Differs(a.max_direction_deviation_deg, b.max_direction_deviation_deg) ||
        Differs(a.max_lateral_deviation_ratio, b.max_lateral_deviation_ratio) ||
        Differs(a.max_speed_change_ratio, b.max_speed_change_ratio) ||
        Differs(a.reject_confidence_decay_per_second, b.reject_confidence_decay_per_second) ||
        Differs(a.planted_foot_max_drift_m, b.planted_foot_max_drift_m) ||
        a.planted_foot_release_confirm_frames != b.planted_foot_release_confirm_frames ||
        Differs(a.contact_root_correction_gain, b.contact_root_correction_gain) ||
        Differs(a.contact_root_max_correction_m, b.contact_root_max_correction_m) ||
        Differs(a.contact_root_max_residual_m, b.contact_root_max_residual_m) ||
        Differs(a.contact_root_max_disagreement_m, b.contact_root_max_disagreement_m) ||
        Differs(a.contact_root_min_alignment, b.contact_root_min_alignment) ||
        Differs(a.contact_root_min_support_confidence, b.contact_root_min_support_confidence);
}

} // namespace

TrackingPipeline::TrackingPipeline(CalibrationBundle calibration)
    : calibration_(std::move(calibration)),
      model_(MakeLowerBodyModel(calibration_.body)) {
    ResetBodyCalibrationEstimator(body_calibrator_, calibration_.body);
}

void TrackingPipeline::SetParams(const TrackingConfig& config) {
    if (config_.mode != config.mode) {
        hmd_depth_scale_history_ = {};
        stereo_hmd_anchor_has_last_ = false;
        stereo_hmd_anchor_last_seconds_ = 0.0;
        has_last_camera_measurement_ = false;
        last_camera_measurement_has_a_ = false;
        last_camera_measurement_has_b_ = false;
        last_camera_measurement_sequence_a_ = 0;
        last_camera_measurement_sequence_b_ = 0;
        has_last_camera_measurement_timestamp_ = false;
        last_camera_measurement_timestamp_seconds_ = 0.0;
        last_camera_measurement_timestamp_a_seconds_ = 0.0;
        last_camera_measurement_timestamp_b_seconds_ = 0.0;
    }
    if (body_calibration_prev_enabled_ != config.body_calibration.enabled) {
        ResetBodyCalibrationEstimator(body_calibrator_, calibration_.body);
        body_calibration_ = {};
        body_calibration_prev_enabled_ = config.body_calibration.enabled;
    }
    if (MotionConsistencyConfigChanged(config_.motion_consistency, config.motion_consistency)) {
        motion_filter_ = {};
    }
    if (config_.tracker_ekf.enabled != config.tracker_ekf.enabled || !config.tracker_ekf.enabled) {
        ResetTrackerEkf(tracker_ekf_);
    }
    config_ = config;
    room_depth_map_.Configure(config_.room_depth_map);
}

PostureMode TrackingPipeline::CurrentPostureMode() const noexcept {
    return posture_.mode;
}

double TrackingPipeline::DeltaTimeFromTimestamp(double timestamp_seconds) {
    if (!std::isfinite(timestamp_seconds)) {
        return kDefaultDtSeconds;
    }
    if (!has_last_timestamp_) {
        last_timestamp_seconds_ = timestamp_seconds;
        has_last_timestamp_ = true;
        return kDefaultDtSeconds;
    }
    const double dt = timestamp_seconds - last_timestamp_seconds_;
    if (!std::isfinite(dt) || dt <= 0.0) {
        return kDefaultDtSeconds;
    }
    last_timestamp_seconds_ = timestamp_seconds;
    return std::max(kMinDtSeconds, std::min(kMaxDtSeconds, dt));
}

Result<TrackingPipelineSnapshot> TrackingPipeline::Step(
    const DecodedPose2D& camera_a_pose,
    const DecodedPose2D& camera_b_pose,
    const ReliabilitySummary& camera_a_reliability,
    const ReliabilitySummary& camera_b_reliability,
    const Pose3f* hmd_pose,
    double timestamp_seconds) {

    BodySolveInputs inputs;
    inputs.camera_a_pose = camera_a_pose;
    inputs.camera_b_pose = camera_b_pose;
    inputs.camera_a_reliability = camera_a_reliability;
    inputs.camera_b_reliability = camera_b_reliability;
    if (camera_a_pose.valid || camera_a_reliability.mean_weight > 0.0f ||
        camera_a_reliability.lower_body_mean > 0.0f || camera_a_reliability.foot_mean > 0.0f) {
        inputs.camera_a_timestamp_seconds = timestamp_seconds;
    }
    if (camera_b_pose.valid || camera_b_reliability.mean_weight > 0.0f ||
        camera_b_reliability.lower_body_mean > 0.0f || camera_b_reliability.foot_mean > 0.0f) {
        inputs.camera_b_timestamp_seconds = timestamp_seconds;
    }
    if (hmd_pose) {
        inputs.hmd.pose = *hmd_pose;
        inputs.hmd.timestamp_seconds = timestamp_seconds;
        inputs.hmd.valid = true;
    }
    return Step(inputs, DeltaTimeFromTimestamp(timestamp_seconds));
}

Result<TrackingPipelineSnapshot> TrackingPipeline::Step(const BodySolveInputs& inputs, double dt_seconds) {
    if (!std::isfinite(dt_seconds) || dt_seconds <= 0.0) {
        dt_seconds = kDefaultDtSeconds;
    }
    dt_seconds = std::max(kMinDtSeconds, std::min(kMaxDtSeconds, dt_seconds));

    const bool camera_a_evidence = CameraEvidencePresent(
        inputs.camera_a_frame_sequence,
        inputs.camera_a_pose,
        inputs.camera_a_reliability);
    const bool camera_b_evidence = CameraEvidencePresent(
        inputs.camera_b_frame_sequence,
        inputs.camera_b_pose,
        inputs.camera_b_reliability);
    const bool has_camera_evidence = camera_a_evidence || camera_b_evidence;
    const bool measurement_identity_available = has_camera_evidence &&
        (!camera_a_evidence || CameraEvidenceIdentityAvailable(
            inputs.camera_a_frame_sequence, inputs.camera_a_timestamp_seconds)) &&
        (!camera_b_evidence || CameraEvidenceIdentityAvailable(
            inputs.camera_b_frame_sequence, inputs.camera_b_timestamp_seconds));
    const bool duplicate_camera_measurement = measurement_identity_available &&
        has_last_camera_measurement_ &&
        camera_a_evidence == last_camera_measurement_has_a_ &&
        camera_b_evidence == last_camera_measurement_has_b_ &&
        (!camera_a_evidence || SameCameraEvidenceIdentity(
            inputs.camera_a_frame_sequence,
            inputs.camera_a_timestamp_seconds,
            last_camera_measurement_sequence_a_,
            last_camera_measurement_timestamp_a_seconds_)) &&
        (!camera_b_evidence || SameCameraEvidenceIdentity(
            inputs.camera_b_frame_sequence,
            inputs.camera_b_timestamp_seconds,
            last_camera_measurement_sequence_b_,
            last_camera_measurement_timestamp_b_seconds_));
    const bool new_camera_measurement = has_camera_evidence && !duplicate_camera_measurement;
    const double camera_timestamp_seconds = CameraEvidenceTimestampSeconds(inputs, camera_a_evidence, camera_b_evidence);
    double camera_measurement_dt_seconds = dt_seconds;
    if (new_camera_measurement && CameraTimestampValid(camera_timestamp_seconds)) {
        if (has_last_camera_measurement_timestamp_) {
            camera_measurement_dt_seconds = ClampDt(camera_timestamp_seconds - last_camera_measurement_timestamp_seconds_);
        }
        last_camera_measurement_timestamp_seconds_ = camera_timestamp_seconds;
        has_last_camera_measurement_timestamp_ = true;
    }
    if (new_camera_measurement && measurement_identity_available) {
        has_last_camera_measurement_ = true;
        last_camera_measurement_has_a_ = camera_a_evidence;
        last_camera_measurement_has_b_ = camera_b_evidence;
        last_camera_measurement_sequence_a_ = camera_a_evidence ? inputs.camera_a_frame_sequence : 0;
        last_camera_measurement_sequence_b_ = camera_b_evidence ? inputs.camera_b_frame_sequence : 0;
        last_camera_measurement_timestamp_a_seconds_ = camera_a_evidence ? inputs.camera_a_timestamp_seconds : 0.0;
        last_camera_measurement_timestamp_b_seconds_ = camera_b_evidence ? inputs.camera_b_timestamp_seconds : 0.0;
    }

    auto solve_inputs = inputs;
    solve_inputs.stereo_geometry_cache = &stereo_geometry_cache_;
    solve_inputs.model = config_.mode == TrackingMode::Monocular
        ? ApplyMonocularUserScale(model_, config_.monocular)
        : model_;
    const int camera_a_width = inputs.camera_a_image_width > 0 ? inputs.camera_a_image_width : config_.monocular.image_width;
    const int camera_a_height = inputs.camera_a_image_height > 0 ? inputs.camera_a_image_height : config_.monocular.image_height;
    const int camera_b_width = inputs.camera_b_image_width > 0 ? inputs.camera_b_image_width : camera_a_width;
    const int camera_b_height = inputs.camera_b_image_height > 0 ? inputs.camera_b_image_height : camera_a_height;
    const FloorGeometryCalibration camera_a_floor_geometry =
        SanitizeMonocularFloorGeometryForImageSpace(
            calibration_.camera_a_floor_geometry.valid ? calibration_.camera_a_floor_geometry : calibration_.floor_geometry,
            config_.mode == TrackingMode::Monocular ? config_.monocular : MonocularImageSpaceForSize(camera_a_width, camera_a_height));
    const FloorGeometryCalibration camera_b_floor_geometry =
        SanitizeMonocularFloorGeometryForImageSpace(
            calibration_.camera_b_floor_geometry,
            MonocularImageSpaceForSize(camera_b_width, camera_b_height));
    const auto camera_a_wall_rectangles = SanitizeWallRectanglesForImageSpace(
        !calibration_.camera_a_wall_rectangles.empty() ? calibration_.camera_a_wall_rectangles : calibration_.wall_rectangles,
        camera_a_width,
        camera_a_height);
    const auto camera_b_wall_rectangles = SanitizeWallRectanglesForImageSpace(
        calibration_.camera_b_wall_rectangles,
        camera_b_width,
        camera_b_height);

    // Saved calibration is the runtime source of truth. Legacy global
    // floor/wall geometry migrates to Camera A only; Camera B must be drawn on
    // Camera B before stereo may consume paired geometry.
    const bool has_monocular_floor_geometry_status =
        config_.mode == TrackingMode::Monocular &&
        (camera_a_floor_geometry.valid || !camera_a_floor_geometry.reason.empty());
    solve_inputs.floor_geometry = has_monocular_floor_geometry_status
        ? camera_a_floor_geometry
        : FloorGeometryCalibration{};
    solve_inputs.wall_rectangles = config_.mode == TrackingMode::Monocular
        ? camera_a_wall_rectangles
        : std::vector<WallRectangleCalibration>{};
    solve_inputs.camera_a_floor_geometry = camera_a_floor_geometry;
    solve_inputs.camera_b_floor_geometry = config_.mode == TrackingMode::Stereo
        ? camera_b_floor_geometry
        : FloorGeometryCalibration{};
    solve_inputs.camera_a_wall_rectangles = camera_a_wall_rectangles;
    solve_inputs.camera_b_wall_rectangles = config_.mode == TrackingMode::Stereo
        ? camera_b_wall_rectangles
        : std::vector<WallRectangleCalibration>{};
    solve_inputs.body_calibration = calibration_.body;
    solve_inputs.dt_seconds = new_camera_measurement ? camera_measurement_dt_seconds : dt_seconds;
    solve_inputs.floor = config_.mode == TrackingMode::Monocular
        ? (solve_inputs.floor_geometry.valid && solve_inputs.floor_geometry.floor_plane.valid
            ? solve_inputs.floor_geometry.floor_plane
            : MakeMonocularFloorPlane(config_.monocular))
        : (calibration_.floor.valid ? calibration_.floor : calibration_.floor_geometry.floor_plane);
    solve_inputs.quality.tracking_mode = config_.mode;
    solve_inputs.quality.min_triangulated_seed_count = config_.min_triangulated_seed_count;
    solve_inputs.quality.max_mean_reprojection_error_px = static_cast<float>(config_.max_mean_reprojection_error_px);
    solve_inputs.quality.use_legacy_solver = config_.use_legacy_solver;
    solve_inputs.quality.stereo_evidence.epipolar = config_.stereo_epipolar;
    solve_inputs.quality.stereo_evidence.triangulation = config_.stereo_triangulation;
    solve_inputs.quality.stereo_evidence.uncertainty = config_.stereo_uncertainty;
    solve_inputs.quality.solver_observation_weighting = config_.solver_observation_weighting;
    solve_inputs.quality.monocular = config_.monocular;
    solve_inputs.hmd_depth_scale.enabled =
        (config_.mode == TrackingMode::Monocular || config_.mode == TrackingMode::Stereo) &&
        config_.hmd_depth_scale.enabled;
    solve_inputs.hmd_depth_scale.config = config_.hmd_depth_scale;
    solve_inputs.hmd_depth_scale.history = hmd_depth_scale_history_;
    solve_inputs.hmd_depth_scale.hmd = solve_inputs.hmd;
    solve_inputs.hmd_depth_scale.camera_timestamp_seconds = camera_timestamp_seconds;
    solve_inputs.hmd_depth_scale.now_seconds = solve_inputs.hmd.timestamp_seconds != 0.0
        ? solve_inputs.hmd.timestamp_seconds
        : (camera_timestamp_seconds != 0.0 ? camera_timestamp_seconds : last_timestamp_seconds_);
    solve_inputs.anchor_space_mapping.config = config_.anchor_space_mapping;
    solve_inputs.anchor_space_mapping.room_map_config = config_.room_depth_map;
    solve_inputs.anchor_space_mapping.stereo_depth_config = config_.stereo_anchor_depth_correction;
    solve_inputs.anchor_space_mapping.anchors = solve_inputs.steamvr_anchors;
    solve_inputs.anchor_space_mapping.room_map = room_depth_map_.Telemetry();
    solve_inputs.anchor_space_mapping.camera_timestamp_seconds = camera_timestamp_seconds;
    solve_inputs.anchor_space_mapping.now_seconds = solve_inputs.steamvr_anchors.steamvr_timestamp_seconds != 0.0
        ? solve_inputs.steamvr_anchors.steamvr_timestamp_seconds
        : (camera_timestamp_seconds != 0.0 ? camera_timestamp_seconds : last_timestamp_seconds_);
    solve_inputs.stereo_hmd_depth_scale_camera_timestamp_seconds = camera_timestamp_seconds;
    solve_inputs.stereo_hmd_depth_scale_now_seconds = solve_inputs.hmd.timestamp_seconds != 0.0
        ? solve_inputs.hmd.timestamp_seconds
        : (camera_timestamp_seconds != 0.0 ? camera_timestamp_seconds : last_timestamp_seconds_);
    const double stereo_anchor_now_seconds = solve_inputs.stereo_hmd_depth_scale_now_seconds;
    const double stereo_anchor_elapsed = stereo_hmd_anchor_has_last_
        ? stereo_anchor_now_seconds - stereo_hmd_anchor_last_seconds_
        : config_.stereo_hmd_anchor.interval_seconds;
    solve_inputs.stereo_hmd_anchor_enabled = config_.mode == TrackingMode::Stereo && config_.stereo_hmd_anchor.enabled;
    solve_inputs.stereo_hmd_anchor_interval_seconds = config_.stereo_hmd_anchor.interval_seconds;
    solve_inputs.stereo_hmd_anchor_seconds_since_last = stereo_hmd_anchor_has_last_ ? stereo_anchor_elapsed : config_.stereo_hmd_anchor.interval_seconds;
    solve_inputs.stereo_hmd_anchor_due = solve_inputs.stereo_hmd_anchor_enabled &&
        (!stereo_hmd_anchor_has_last_ || stereo_anchor_elapsed >= config_.stereo_hmd_anchor.interval_seconds);
    solve_inputs.stereo_hmd_anchor_max_correction_m = config_.stereo_hmd_anchor.max_correction_m;
    if (config_.mode == TrackingMode::Monocular) {
        solve_inputs.camera_a_calibration = MakeMonocularCameraCalibration(calibration_.camera_a, config_.monocular);
        solve_inputs.camera_b_calibration = {};
    } else {
        solve_inputs.camera_a_calibration = calibration_.camera_a;
        solve_inputs.camera_b_calibration = calibration_.camera_b;
    }

    const LowerBodyState predicted = PredictState(state_, dt_seconds, &solve_inputs.model);
    const bool camera_a_projection_valid =
        solve_inputs.camera_a_calibration.intrinsics_valid &&
        solve_inputs.camera_a_calibration.extrinsics_valid;
    const bool camera_b_projection_valid =
        solve_inputs.camera_b_calibration.intrinsics_valid &&
        solve_inputs.camera_b_calibration.extrinsics_valid;
    EpipolarGeometry identity_epipolar_geometry{};
    StereoIdentityEpipolarContext identity_epipolar_context{};
    const StereoIdentityEpipolarContext* identity_epipolar = nullptr;
    if (config_.mode == TrackingMode::Stereo &&
        !duplicate_camera_measurement &&
        camera_a_projection_valid &&
        camera_b_projection_valid &&
        !solve_inputs.stereo_pair_degraded &&
        !solve_inputs.stereo_pair_reused_a &&
        !solve_inputs.stereo_pair_reused_b &&
        !solve_inputs.stereo_pair_duplicate &&
        !solve_inputs.stereo_pair_skewed) {
        const auto epipolar_geometry = GetOrComputeEpipolarGeometry(
            &stereo_geometry_cache_,
            solve_inputs.camera_a_calibration,
            solve_inputs.camera_b_calibration,
            config_.stereo_epipolar,
            config_.stereo_triangulation);
        if (epipolar_geometry.ok() && epipolar_geometry.value().valid) {
            identity_epipolar_geometry = epipolar_geometry.value();
            identity_epipolar_context.geometry = &identity_epipolar_geometry;
            identity_epipolar_context.camera_a = &solve_inputs.camera_a_calibration;
            identity_epipolar_context.camera_b = &solve_inputs.camera_b_calibration;
            identity_epipolar_context.config = config_.stereo_identity;
            identity_epipolar_context.triangulation = config_.stereo_triangulation;
            identity_epipolar_context.uncertainty = config_.stereo_uncertainty;
            identity_epipolar_context.solver_observation_weighting = config_.solver_observation_weighting;
            identity_epipolar_context.pair_degraded = solve_inputs.stereo_pair_degraded;
            identity_epipolar_context.reused_camera_a = solve_inputs.stereo_pair_reused_a;
            identity_epipolar_context.reused_camera_b = solve_inputs.stereo_pair_reused_b;
            identity_epipolar_context.duplicate_pair = solve_inputs.stereo_pair_duplicate;
            identity_epipolar_context.timestamp_skewed = solve_inputs.stereo_pair_skewed;
            identity_epipolar_context.predicted_state_age_seconds = static_cast<float>(std::max(0.0, dt_seconds));
            identity_epipolar = &identity_epipolar_context;
        }
    }

    // Identity arbitration uses the process-predicted state before the current
    // frame's stereo/body-solver measurement update. This avoids circularly
    // comparing a same-camera candidate against a state that was already pulled
    // toward that same candidate in the current frame.
    const StereoIdentityAssignmentResult identity = ResolveStereoLeftRightIdentity(
        solve_inputs.camera_a_pose,
        solve_inputs.camera_b_pose,
        predicted,
        camera_a_projection_valid ? &solve_inputs.camera_a_calibration.image_from_world : nullptr,
        camera_b_projection_valid ? &solve_inputs.camera_b_calibration.image_from_world : nullptr,
        identity_epipolar);
    const IdentityAssignmentResult identity_a = identity.camera_a;
    const IdentityAssignmentResult identity_b = identity.camera_b;
    solve_inputs.camera_a_pose = identity_a.pose;
    solve_inputs.camera_b_pose = identity_b.pose;
    TrackingPipelineStages stages;
    stages.predicted = StagePoseFromState(predicted);
    if (duplicate_camera_measurement) {
        BodySolveInputs hold_inputs = solve_inputs;
        hold_inputs.camera_a_pose = {};
        hold_inputs.camera_b_pose = {};
        hold_inputs.camera_a_reliability = {};
        hold_inputs.camera_b_reliability = {};
        hold_inputs.camera_a_frame_sequence = 0;
        hold_inputs.camera_b_frame_sequence = 0;
        hold_inputs.camera_a_frame_age_ms = 0.0;
        hold_inputs.camera_b_frame_age_ms = 0.0;
        const LowerBodyState held = EstimateOccludedState(state_, predicted, hold_inputs, posture_, dt_seconds);
        stages.measured = StagePoseFromState(held, held.confidence > 0.0f);
        stages.corrected = StagePoseFromState(held, held.confidence > 0.0f);
        state_ = held;
        auto solver = SolverTelemetry(
            hold_inputs.hmd.valid,
            true,
            "duplicate_camera_measurement_hold",
            config_.mode,
            config_.mode == TrackingMode::Monocular ? DepthSource::InferredMonocular : DepthSource::TriangulatedStereo);
        if (solve_inputs.hmd_depth_scale.enabled) {
            solver.hmd_depth_scale.state = HmdDepthScaleStateKind::UnavailableNoPreviousScale;
            solver.hmd_depth_scale.reason = "not computed: duplicate camera measurement hold";
        }
        if (solve_inputs.stereo_hmd_anchor_enabled) {
            solver.stereo_hmd_anchor.state = StereoHmdAnchorStateKind::WaitingInterval;
            solver.stereo_hmd_anchor.reason = "not computed: duplicate camera measurement hold";
            solver.stereo_hmd_anchor.interval_seconds = solve_inputs.stereo_hmd_anchor_interval_seconds;
            solver.stereo_hmd_anchor.seconds_since_last_anchor = solve_inputs.stereo_hmd_anchor_seconds_since_last;
        }
        TrackerEkfTelemetry ekf_telemetry;
        snapshot_ = SnapshotFromState(
            state_,
            solve_inputs.model,
            calibration_.body,
            posture_,
            motion_filter_.telemetry,
            ekf_telemetry,
            std::move(solver),
            stages,
            dt_seconds,
            "duplicate_camera_measurement_hold",
            "same camera sequence already consumed; temporal filters held");
        snapshot_.body_calibration = body_calibration_;
        snapshot_.floor_geometry = solve_inputs.floor_geometry;
        return snapshot_;
    }
    const auto preliminary = RunPreliminaryBodySolve(solve_inputs, predicted);
    const bool preliminary_unusable = !preliminary.ok() || !preliminary.value().valid;
    if (preliminary_unusable) {
        const std::string reason = preliminary.ok()
            ? preliminary.value().telemetry.degradation_reason
            : preliminary.status().message;
        auto solver = SolverTelemetry(solve_inputs.hmd.valid, true, reason, config_.mode, config_.mode == TrackingMode::Monocular ? DepthSource::InferredMonocular : DepthSource::TriangulatedStereo);
        CopyIdentityTelemetry(solver, identity);
        if (preliminary.ok()) {
            solver.preliminary_solve_ms = preliminary.value().telemetry.solve_ms;
            solver.preliminary_residual = preliminary.value().residual;
            solver.preliminary_weighted_observation_count = preliminary.value().weighted_observation_count;
            solver.preliminary_stereo = preliminary.value().telemetry.stereo;
            solver.hmd_depth_scale = preliminary.value().hmd_depth_scale;
            solver.stereo_hmd_anchor = preliminary.value().stereo_hmd_anchor;
            solver.anchor_space_mapping = preliminary.value().telemetry.stereo.anchor_space_mapping;
            solver.room_depth_map = preliminary.value().telemetry.stereo.room_depth_map;
            if (preliminary.value().stereo_hmd_anchor.applied) {
                stereo_hmd_anchor_has_last_ = true;
                stereo_hmd_anchor_last_seconds_ = stereo_anchor_now_seconds;
            }
            solver.objective_evaluations = preliminary.value().telemetry.objective_evaluations;
            solver.coordinate_passes = preliminary.value().telemetry.coordinate_passes;
            solver.optimizer_early_stopped = preliminary.value().telemetry.optimizer_early_stopped;
        } else if (solve_inputs.hmd_depth_scale.enabled) {
            solver.hmd_depth_scale.state = HmdDepthScaleStateKind::UnavailableNoPreviousScale;
            solver.hmd_depth_scale.reason = "not computed: preliminary solve failed before hmd depth scale";
        }
        TrackerEkfTelemetry ekf_telemetry;
        const LowerBodyState occluded_estimate = EstimateOccludedState(state_, predicted, solve_inputs, posture_, dt_seconds);
        stages.measured = StagePoseFromState(occluded_estimate, occluded_estimate.confidence > 0.0f);
        state_ = ApplyTrackerEkf(
            tracker_ekf_,
            occluded_estimate,
            dt_seconds,
            config_.tracker_ekf,
            &ekf_telemetry,
            &solve_inputs.model);
        stages.ekf_filtered = StagePoseFromState(state_, state_.confidence > 0.0f);
        stages.corrected = StagePoseFromState(state_, state_.confidence > 0.0f);
        snapshot_ = SnapshotFromState(
            state_,
            solve_inputs.model,
            calibration_.body,
            posture_,
            motion_filter_.telemetry,
            ekf_telemetry,
            std::move(solver),
            stages,
            dt_seconds,
            state_.confidence > 0.0f ? "occluded_predictive_hold" : "occluded_untracked",
            reason);
        snapshot_.body_calibration = body_calibration_;
        snapshot_.floor_geometry = solve_inputs.floor_geometry;
        return snapshot_;
    }

    posture_ = UpdatePostureMode(posture_, preliminary.value().state, inputs.hmd, camera_measurement_dt_seconds);
    stages.preliminary = StagePoseFromState(preliminary.value().state, preliminary.value().valid);

    LowerBodyState mode_state = preliminary.value().state;
    mode_state.posture_mode = posture_.mode;
    const auto left_foot_evidence = FootEvidence(
        preliminary.value().telemetry.stereo.left_foot_contact_confidence,
        preliminary.value().telemetry.stereo.left_heel_contact_confidence,
        preliminary.value().telemetry.stereo.left_toe_contact_confidence);
    const auto right_foot_evidence = FootEvidence(
        preliminary.value().telemetry.stereo.right_foot_contact_confidence,
        preliminary.value().telemetry.stereo.right_heel_contact_confidence,
        preliminary.value().telemetry.stereo.right_toe_contact_confidence);
    mode_state.support.left_foot = UpdateFootSupportCalibrated(
        mode_state.support.left_foot,
        mode_state.left_foot,
        state_.left_foot,
        posture_.mode,
        solve_inputs.floor,
        camera_measurement_dt_seconds,
        solve_inputs.model.left_foot_length,
        FootSupportConfig{},
        &left_foot_evidence);
    mode_state.support.right_foot = UpdateFootSupportCalibrated(
        mode_state.support.right_foot,
        mode_state.right_foot,
        state_.right_foot,
        posture_.mode,
        solve_inputs.floor,
        camera_measurement_dt_seconds,
        solve_inputs.model.right_foot_length,
        FootSupportConfig{},
        &right_foot_evidence);
    mode_state.support = UpdateKneeContactSupport(
        mode_state.support,
        mode_state,
        solve_inputs.model,
        solve_inputs.floor,
        posture_.mode,
        camera_measurement_dt_seconds,
        KneeEvidence(preliminary.value().telemetry.stereo));
    mode_state.support = UpdateRootSupport(mode_state.support, mode_state, posture_.mode, camera_measurement_dt_seconds);
    stages.support_ready = StagePoseFromState(mode_state);

    const auto final_solve = RunFinalSupportAwareSolve(solve_inputs, mode_state);
    const bool final_unusable = !final_solve.ok() || !final_solve.value().valid;
    if (final_unusable) {
        const std::string reason = final_solve.ok()
            ? final_solve.value().telemetry.degradation_reason
            : final_solve.status().message;
        auto solver = SolverTelemetry(solve_inputs.hmd.valid, true, reason, config_.mode, preliminary.value().telemetry.depth_source);
        CopyIdentityTelemetry(solver, identity);
        solver.preliminary_solve_ms = preliminary.value().telemetry.solve_ms;
        solver.preliminary_residual = preliminary.value().residual;
        solver.preliminary_weighted_observation_count = preliminary.value().weighted_observation_count;
        solver.preliminary_stereo = preliminary.value().telemetry.stereo;
        solver.hmd_depth_scale = final_solve.ok()
            ? (final_solve.value().hmd_depth_scale.usable ? final_solve.value().hmd_depth_scale : preliminary.value().hmd_depth_scale)
            : preliminary.value().hmd_depth_scale;
        solver.stereo_hmd_anchor = final_solve.ok()
            ? final_solve.value().stereo_hmd_anchor
            : preliminary.value().stereo_hmd_anchor;
        solver.anchor_space_mapping = final_solve.ok()
            ? final_solve.value().telemetry.stereo.anchor_space_mapping
            : preliminary.value().telemetry.stereo.anchor_space_mapping;
        solver.room_depth_map = final_solve.ok()
            ? final_solve.value().telemetry.stereo.room_depth_map
            : preliminary.value().telemetry.stereo.room_depth_map;
        if (solver.stereo_hmd_anchor.applied) {
            stereo_hmd_anchor_has_last_ = true;
            stereo_hmd_anchor_last_seconds_ = stereo_anchor_now_seconds;
        }
        solver.objective_evaluations = preliminary.value().telemetry.objective_evaluations;
        solver.coordinate_passes = preliminary.value().telemetry.coordinate_passes;
        solver.optimizer_early_stopped = preliminary.value().telemetry.optimizer_early_stopped;
        TrackerEkfTelemetry ekf_telemetry;
        const LowerBodyState occluded_estimate = EstimateOccludedState(state_, predicted, solve_inputs, posture_, dt_seconds);
        stages.measured = StagePoseFromState(occluded_estimate, occluded_estimate.confidence > 0.0f);
        state_ = ApplyTrackerEkf(
            tracker_ekf_,
            occluded_estimate,
            dt_seconds,
            config_.tracker_ekf,
            &ekf_telemetry,
            &solve_inputs.model);
        stages.ekf_filtered = StagePoseFromState(state_, state_.confidence > 0.0f);
        stages.corrected = StagePoseFromState(state_, state_.confidence > 0.0f);
        snapshot_ = SnapshotFromState(
            state_,
            solve_inputs.model,
            calibration_.body,
            posture_,
            motion_filter_.telemetry,
            ekf_telemetry,
            std::move(solver),
            stages,
            dt_seconds,
            state_.confidence > 0.0f ? "support_fallback_hold" : "final_solve_failed",
            reason);
        snapshot_.body_calibration = body_calibration_;
        snapshot_.floor_geometry = solve_inputs.floor_geometry;
        return snapshot_;
    }

    MaybeUpdateBodyCalibration(
        calibration_,
        model_,
        body_calibrator_,
        body_calibration_,
        config_,
        preliminary.value().telemetry.stereo,
        solve_inputs.hmd,
        camera_measurement_dt_seconds);

    LowerBodyState measured = ApplyJointLimitBounds(final_solve.value().state);
    stages.measured = StagePoseFromState(measured, final_solve.value().valid);
    const LowerBodyState motion_filtered = ApplyMotionConsistencyFilter(motion_filter_, measured, predicted, camera_measurement_dt_seconds, config_.motion_consistency, &solve_inputs.model);
    stages.motion_filtered = StagePoseFromState(motion_filtered);
    TrackerEkfTelemetry ekf_telemetry;
    const LowerBodyState ekf_filtered = ApplyTrackerEkf(tracker_ekf_, motion_filtered, camera_measurement_dt_seconds, config_.tracker_ekf, &ekf_telemetry, &solve_inputs.model);
    stages.ekf_filtered = StagePoseFromState(ekf_filtered);
    state_ = CorrectState(
        predicted,
        ekf_filtered,
        camera_measurement_dt_seconds,
        config_.temporal_update,
        TemporalPositionCorrectionMode::BlendPositions,
        &solve_inputs.model);
    stages.corrected = StagePoseFromState(state_);
    snapshot_.state = state_;
    snapshot_.posture = posture_;
    snapshot_.motion_filter = motion_filter_.telemetry;
    snapshot_.tracker_ekf = ekf_telemetry;
    const bool solver_degraded = preliminary.value().telemetry.degraded || final_solve.value().telemetry.degraded;
    const std::string solver_degradation_reason = !final_solve.value().telemetry.degradation_reason.empty()
        ? final_solve.value().telemetry.degradation_reason
        : preliminary.value().telemetry.degradation_reason;
    snapshot_.solver = SolverTelemetry(
        solve_inputs.hmd.valid,
        solver_degraded,
        solver_degradation_reason,
        config_.mode,
        preliminary.value().telemetry.depth_source);
    CopyIdentityTelemetry(snapshot_.solver, identity);
    snapshot_.stages = stages;
    snapshot_.solver.preliminary_solve_ms = preliminary.value().telemetry.solve_ms;
    snapshot_.solver.final_solve_ms = final_solve.value().telemetry.solve_ms;
    snapshot_.solver.preliminary_residual = preliminary.value().residual;
    snapshot_.solver.final_residual = final_solve.value().residual;
    snapshot_.solver.preliminary_weighted_observation_count = preliminary.value().weighted_observation_count;
    snapshot_.solver.final_weighted_observation_count = final_solve.value().weighted_observation_count;
    snapshot_.solver.preliminary_stereo = preliminary.value().telemetry.stereo;
    snapshot_.solver.final_constraints = final_solve.value().telemetry.constraints;
    snapshot_.solver.hmd_depth_scale = final_solve.value().hmd_depth_scale.usable
        ? final_solve.value().hmd_depth_scale
        : preliminary.value().hmd_depth_scale;
    snapshot_.solver.stereo_hmd_anchor = final_solve.value().stereo_hmd_anchor;
    snapshot_.solver.anchor_space_mapping = final_solve.value().telemetry.stereo.anchor_space_mapping;
    room_depth_map_.ObserveAnchorCorrection(
        snapshot_.solver.anchor_space_mapping,
        solve_inputs.anchor_space_mapping.now_seconds);
    snapshot_.solver.room_depth_map = room_depth_map_.Telemetry();
    if (snapshot_.solver.stereo_hmd_anchor.applied) {
        stereo_hmd_anchor_has_last_ = true;
        stereo_hmd_anchor_last_seconds_ = stereo_anchor_now_seconds;
    }
    if (final_solve.value().hmd_depth_scale.live || final_solve.value().hmd_depth_scale.held || final_solve.value().hmd_depth_scale.usable) {
        hmd_depth_scale_history_ = final_solve.value().hmd_depth_scale_history;
    } else if (preliminary.value().hmd_depth_scale.live || preliminary.value().hmd_depth_scale.held || preliminary.value().hmd_depth_scale.usable) {
        hmd_depth_scale_history_ = preliminary.value().hmd_depth_scale_history;
    }
    snapshot_.solver.objective_evaluations = preliminary.value().telemetry.objective_evaluations;
    snapshot_.solver.coordinate_passes = preliminary.value().telemetry.coordinate_passes;
    snapshot_.solver.optimizer_early_stopped = preliminary.value().telemetry.optimizer_early_stopped;
    snapshot_.body_state = BuildUnifiedBodyState(
        state_,
        solve_inputs.model,
        BuildBodyStateEvidence(BodyStateSolverFromTelemetry(snapshot_.solver), state_),
        calibration_.body,
        camera_measurement_dt_seconds,
        LowerBodyStateOutputValid(state_));
    snapshot_.trackers = SynthesizeTrackerPoses(snapshot_.body_state, solve_inputs.model);
    snapshot_.body_calibration = body_calibration_;
    snapshot_.floor_geometry = solve_inputs.floor_geometry;
    snapshot_.degradation_mode = solver_degraded ? "solver_degraded" : "nominal";
    snapshot_.last_error.clear();
    return snapshot_;
}

Result<TrackingPipelineSnapshot> TrackingPipeline::SolveFromRecordedTrackers(
    const TrackerPoseArray& trackers,
    const Pose3f* hmd_pose,
    double timestamp_seconds) {

    const double dt_seconds = DeltaTimeFromTimestamp(timestamp_seconds);
    TrackingPipelineStages stages;
    LowerBodyState measured = state_;
    measured.root = TrackerPoseOr(trackers, TrackerRole::Pelvis, measured.root);
    measured.left_foot = TrackerPoseOr(trackers, TrackerRole::LeftFoot, measured.left_foot);
    measured.right_foot = TrackerPoseOr(trackers, TrackerRole::RightFoot, measured.right_foot);
    measured.confidence = TrackerConfidence(trackers);
    if (measured.confidence <= 0.0f && hmd_pose) {
        measured.root = *hmd_pose;
        measured.confidence = kBootstrapHmdConfidence;
    }
    measured.posture_mode = posture_.mode;
    measured.support = state_.support;

    const LowerBodyState predicted = PredictState(state_, dt_seconds, &model_);
    stages.predicted = StagePoseFromState(predicted);
    if (hmd_pose) {
        measured.root.position = Lerp(measured.root.position, hmd_pose->position, 0.35f);
        measured.root.orientation = YawOnly(hmd_pose->orientation);
    }
    const auto left_foot_evidence = FootEvidence(TrackerConfidenceForRole(trackers, TrackerRole::LeftFoot));
    const auto right_foot_evidence = FootEvidence(TrackerConfidenceForRole(trackers, TrackerRole::RightFoot));
    measured.support.left_foot = UpdateFootSupportCalibrated(
        measured.support.left_foot,
        measured.left_foot,
        state_.left_foot,
        posture_.mode,
        calibration_.floor,
        dt_seconds,
        model_.left_foot_length,
        FootSupportConfig{},
        &left_foot_evidence);
    measured.support.right_foot = UpdateFootSupportCalibrated(
        measured.support.right_foot,
        measured.right_foot,
        state_.right_foot,
        posture_.mode,
        calibration_.floor,
        dt_seconds,
        model_.right_foot_length,
        FootSupportConfig{},
        &right_foot_evidence);
    measured.support = UpdateKneeContactSupport(
        measured.support,
        measured,
        model_,
        calibration_.floor,
        posture_.mode,
        dt_seconds,
        MissingKneeEvidence());
    measured.support = UpdateRootSupport(measured.support, measured, posture_.mode, dt_seconds);
    measured.left_foot_evidence.source = TrackerEvidenceSource::ReplayInput;
    measured.left_foot_evidence.direct_confidence = TrackerConfidenceForRole(trackers, TrackerRole::LeftFoot);
    measured.left_foot_evidence.support_confidence = FootSupportConfidence(measured.support.left_foot);
    measured.left_foot_evidence.anchor_held = IsActiveFootSupport(measured.support.left_foot);
    measured.left_foot_evidence.valid = measured.left_foot_evidence.direct_confidence > 0.0f;
    if (!measured.left_foot_evidence.valid && measured.left_foot_evidence.anchor_held) {
        measured.left_foot_evidence.source = TrackerEvidenceSource::AnchorHeld;
        measured.left_foot_evidence.valid = measured.left_foot_evidence.support_confidence > 0.0f;
    }
    measured.right_foot_evidence.source = TrackerEvidenceSource::ReplayInput;
    measured.right_foot_evidence.direct_confidence = TrackerConfidenceForRole(trackers, TrackerRole::RightFoot);
    measured.right_foot_evidence.support_confidence = FootSupportConfidence(measured.support.right_foot);
    measured.right_foot_evidence.anchor_held = IsActiveFootSupport(measured.support.right_foot);
    measured.right_foot_evidence.valid = measured.right_foot_evidence.direct_confidence > 0.0f;
    if (!measured.right_foot_evidence.valid && measured.right_foot_evidence.anchor_held) {
        measured.right_foot_evidence.source = TrackerEvidenceSource::AnchorHeld;
        measured.right_foot_evidence.valid = measured.right_foot_evidence.support_confidence > 0.0f;
    }
    stages.measured = StagePoseFromState(measured, measured.confidence > 0.0f);

    measured = ApplyMotionConsistencyFilter(motion_filter_, ApplyJointLimitBounds(measured), predicted, dt_seconds, config_.motion_consistency, &model_);
    stages.motion_filtered = StagePoseFromState(measured, measured.confidence > 0.0f);
    TrackerEkfTelemetry ekf_telemetry;
    measured = ApplyTrackerEkf(tracker_ekf_, measured, dt_seconds, config_.tracker_ekf, &ekf_telemetry, &model_);
    stages.ekf_filtered = StagePoseFromState(measured, measured.confidence > 0.0f);
    state_ = CorrectState(predicted, measured, dt_seconds, config_.temporal_update, TemporalPositionCorrectionMode::BlendPositions, &model_);
    stages.corrected = StagePoseFromState(state_, state_.confidence > 0.0f);

    snapshot_.state = state_;
    snapshot_.posture = posture_;
    snapshot_.motion_filter = motion_filter_.telemetry;
    snapshot_.tracker_ekf = ekf_telemetry;
    snapshot_.solver = SolverTelemetry(hmd_pose != nullptr, true, "replay_tracker_input");
    snapshot_.stages = stages;
    snapshot_.body_state = BuildUnifiedBodyState(
        state_,
        model_,
        BuildBodyStateEvidence(BodyStateSolverFromTelemetry(snapshot_.solver), state_),
        calibration_.body,
        dt_seconds,
        LowerBodyStateOutputValid(state_));
    ApplyReplayTrackerRolesToBodyState(snapshot_.body_state, trackers);
    snapshot_.trackers = SynthesizeTrackerPoses(snapshot_.body_state, model_);
    PreserveReplayTrackerPoses(snapshot_.trackers, trackers);
    snapshot_.body_calibration = body_calibration_;
    // Recorded tracker replay consumes tracker poses plus the calibrated floor plane,
    // not the image-space floor geometry model. Do not advertise saved geometry as
    // runtime-used geometry on this path.
    snapshot_.floor_geometry = FloorGeometryCalibration{};
    snapshot_.floor_geometry.reason = "not_used_replay_tracker_input";
    snapshot_.degradation_mode = "replay_tracker_input";
    snapshot_.last_error.clear();
    return snapshot_;
}

BodyCalibrationTelemetry TrackingPipeline::PersistBodyCalibrationOnShutdown() {
    body_calibration_.enabled = config_.body_calibration.enabled;
    body_calibration_.auto_persist = config_.body_calibration.auto_persist;
    body_calibration_.saved_this_frame = false;
    CopyBodyCalibrationPersistState(body_calibration_, body_calibrator_, config_.body_calibration);

    if (!config_.body_calibration.enabled) {
        SetBodyCalibrationPersistState(body_calibrator_, body_calibration_, config_.body_calibration, false, false, "disabled");
        snapshot_.body_calibration = body_calibration_;
        return body_calibration_;
    }
    if (!body_calibration_.complete && !body_calibrator_.complete) {
        SetBodyCalibrationPersistState(body_calibrator_, body_calibration_, config_.body_calibration, false, false, "not_complete");
        snapshot_.body_calibration = body_calibration_;
        return body_calibration_;
    }

    if (body_calibrator_.complete && !body_calibration_.complete) {
        body_calibration_.complete = true;
        body_calibration_.body = body_calibrator_.body;
        body_calibration_.accumulated_seconds = body_calibrator_.elapsed_seconds;
        body_calibration_.accepted_samples = body_calibrator_.accepted_samples;
        body_calibration_.overall_confidence = body_calibrator_.body.quality.overall;
        body_calibration_.reason = "complete";
    }

    calibration_.body = body_calibration_.body;
    model_ = MakeLowerBodyModel(calibration_.body);

    if (!config_.body_calibration.auto_persist) {
        SetBodyCalibrationPersistState(
            body_calibrator_,
            body_calibration_,
            config_.body_calibration,
            false,
            false,
            "auto_persist_disabled");
        snapshot_.body_calibration = body_calibration_;
        return body_calibration_;
    }

    const Status saved = SaveCalibrationBundle(calibration_, config_.calibration_path);
    if (saved.ok()) {
        body_calibrator_.dirty = false;
        SetBodyCalibrationPersistState(
            body_calibrator_,
            body_calibration_,
            config_.body_calibration,
            true,
            false,
            "saved_on_shutdown");
    } else {
        SetBodyCalibrationPersistState(
            body_calibrator_,
            body_calibration_,
            config_.body_calibration,
            body_calibrator_.persisted,
            !body_calibrator_.persisted,
            "shutdown_save_failed",
            saved.message);
        body_calibration_.reason = std::string("complete_but_shutdown_save_failed: ") + saved.message;
    }

    snapshot_.body_calibration = body_calibration_;
    // Shutdown persistence is a save operation, not a tracking frame. Leave the
    // existing floor-geometry snapshot untouched so status/replay continues to
    // reflect the last runtime solve path rather than merely saved calibration.
    return body_calibration_;
}

TrackingPipelineSnapshot TrackingPipeline::Snapshot() const {
    return snapshot_;
}

} // namespace bt
