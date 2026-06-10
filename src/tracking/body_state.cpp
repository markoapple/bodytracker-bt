#include "tracking/body_state.h"

#include "tracking/measurement_weighting.h"
#include "tracking/support_queries.h"
#include "tracking/tracking_constants.h"

#include <algorithm>
#include <cmath>

namespace bt {
namespace {

float Clamp01(float value) {
    if (!std::isfinite(value)) {
        return 0.0f;
    }
    return std::max(0.0f, std::min(1.0f, value));
}

bool MeasurementPresent(const BodyStateJointMeasurement& telemetry) {
    return telemetry.triangulated || telemetry.depth_inferred;
}

bool MeasurementWorldPresent(const BodyStateJointMeasurement& telemetry) {
    return MeasurementPresent(telemetry) && IsFinite(telemetry.world);
}

bool CameraEvidencePresent(const BodyStateJointMeasurement& telemetry) {
    return telemetry.camera_a_present || telemetry.camera_b_present;
}

float ReprojectionQuality(const BodyStateJointMeasurement& telemetry) {
    if (!telemetry.triangulated || !std::isfinite(telemetry.mean_reprojection_error_px) ||
        telemetry.mean_reprojection_error_px <= 0.0f) {
        return 1.0f;
    }
    return Clamp01(1.0f - telemetry.mean_reprojection_error_px / tracking_constants::kReprojectionErrorMaxPx);
}

float SolverObservationConfidenceCeiling(const BodyStateJointMeasurement& telemetry) {
    if (!telemetry.solver_observation_weighted) {
        return 1.0f;
    }

    // solver_observation_weight_scale is an inverse-variance scale relative to
    // the solver reference observation. It is not a confidence value: fallback
    // observations can legitimately have scales near 0.0025 while still being
    // visually confident. Map it to a conservative output-confidence ceiling
    // instead of multiplying confidence by the raw scale.
    return SolverObservationConfidenceCeilingFromWeightScale(telemetry.solver_observation_weight_scale);
}

float ApplySolverObservationConfidenceCeiling(float confidence, const BodyStateJointMeasurement& telemetry) {
    return std::min(Clamp01(confidence), SolverObservationConfidenceCeiling(telemetry));
}

float IdentityConfidence(const BodyStateSolverSnapshot& solver) {
    float sum = 0.0f;
    int count = 0;
    if (solver.camera_a_identity_consistency > 0.0f) {
        sum += Clamp01(solver.camera_a_identity_consistency);
        ++count;
    }
    if (solver.tracking_mode == TrackingMode::Stereo && solver.camera_b_identity_consistency > 0.0f) {
        sum += Clamp01(solver.camera_b_identity_consistency);
        ++count;
    }
    return count > 0 ? Clamp01(sum / static_cast<float>(count)) : 1.0f;
}

float IdentityRoleFactor(float identity_confidence) {
    const float t = Clamp01(identity_confidence);
    return tracking_constants::kIdentityRoleFactorMin +
        (tracking_constants::kIdentityRoleFactorMax - tracking_constants::kIdentityRoleFactorMin) * t;
}

BodyJointVisibility VisibilityFromTelemetry(
    const BodyStateJointMeasurement& telemetry,
    bool degraded,
    bool valid_state) {

    if (MeasurementPresent(telemetry)) {
        return telemetry.confidence >= tracking_constants::kVisibleConfidenceThreshold
            ? BodyJointVisibility::Visible
            : BodyJointVisibility::LowConfidence;
    }
    if (telemetry.evidence_source == JointEvidenceSource::TemporalHold) {
        return BodyJointVisibility::Predicted;
    }
    if (CameraEvidencePresent(telemetry)) {
        return BodyJointVisibility::LowConfidence;
    }
    if (degraded && valid_state) {
        return BodyJointVisibility::Predicted;
    }
    if (valid_state) {
        return BodyJointVisibility::CameraOccluded;
    }
    return BodyJointVisibility::MissingUnknown;
}

TrackerEvidence EvidenceFromTelemetry(const BodyStateJointMeasurement& telemetry, float fallback_confidence) {
    TrackerEvidence evidence;
    if (telemetry.triangulated) {
        evidence.source = TrackerEvidenceSource::DirectStereo;
        evidence.direct_confidence = Clamp01(ApplySolverObservationConfidenceCeiling(telemetry.confidence, telemetry));
        evidence.valid = MeasurementWorldPresent(telemetry);
        return evidence;
    }
    if (telemetry.depth_inferred) {
        evidence.source = TrackerEvidenceSource::InferredMonocular;
        evidence.direct_confidence = Clamp01(ApplySolverObservationConfidenceCeiling(telemetry.confidence, telemetry));
        evidence.valid = MeasurementWorldPresent(telemetry);
        return evidence;
    }
    evidence.source = TrackerEvidenceSource::Predicted;
    evidence.direct_confidence = Clamp01(0.5f * fallback_confidence);
    evidence.valid = fallback_confidence > 0.0f;
    return evidence;
}

Keypoint3D KeypointOrEmpty(const LowerBodyJointSet& joints, KeypointId id) {
    return joints.joints[static_cast<std::size_t>(id)];
}

Vec3f JointPositionOr(const LowerBodyJointSet& joints, KeypointId id, Vec3f fallback) {
    const auto joint = KeypointOrEmpty(joints, id);
    return joint.present && IsFinite(joint.world) ? joint.world : fallback;
}

Vec3f ToePosition(const LowerBodyJointSet& joints, bool left, Vec3f fallback) {
    const KeypointId big = left ? KeypointId::LeftBigToe : KeypointId::RightBigToe;
    const KeypointId small = left ? KeypointId::LeftSmallToe : KeypointId::RightSmallToe;
    const auto big_toe = KeypointOrEmpty(joints, big);
    const auto small_toe = KeypointOrEmpty(joints, small);
    if (big_toe.present && small_toe.present && IsFinite(big_toe.world) && IsFinite(small_toe.world)) {
        return Scale(Add(big_toe.world, small_toe.world), 0.5f);
    }
    if (big_toe.present && IsFinite(big_toe.world)) {
        return big_toe.world;
    }
    if (small_toe.present && IsFinite(small_toe.world)) {
        return small_toe.world;
    }
    return fallback;
}

BodyStateJointMeasurement JointTelemetryOrEmpty(
    const BodyStateSolverSnapshot& solver,
    KeypointId id) {

    return solver.joints[static_cast<std::size_t>(id)];
}

float RoleConfidence(
    const BodyStateJointMeasurement& telemetry,
    float fallback_confidence,
    bool valid_state,
    float identity_confidence) {

    if (MeasurementPresent(telemetry)) {
        return Clamp01(
            std::min(
                telemetry.confidence * ReprojectionQuality(telemetry) * IdentityRoleFactor(identity_confidence),
                SolverObservationConfidenceCeiling(telemetry)));
    }
    const float camera_factor = CameraEvidencePresent(telemetry)
        ? tracking_constants::kCameraSeenUnusableFallbackFactor
        : tracking_constants::kNoCameraEvidencePredictionFactor;
    return valid_state ? Clamp01(camera_factor * fallback_confidence * IdentityRoleFactor(identity_confidence)) : 0.0f;
}

BodyStateJoint MakeMeasuredJoint(
    BodyJointRole role,
    const LowerBodyJointSet& joints,
    KeypointId id,
    Vec3f fallback,
    Vec3f velocity,
    const BodyStateSolverSnapshot& solver,
    float fallback_confidence,
    bool valid_state,
    float identity_confidence) {

    const auto telemetry = JointTelemetryOrEmpty(solver, id);
    BodyStateJoint out;
    out.role = role;
    out.position = MeasurementPresent(telemetry) && IsFinite(telemetry.world)
        ? telemetry.world
        : JointPositionOr(joints, id, fallback);
    out.velocity = velocity;
    out.confidence = RoleConfidence(telemetry, fallback_confidence, valid_state, identity_confidence);
    out.valid = valid_state && IsFinite(out.position) && (out.confidence > 0.0f || MeasurementWorldPresent(telemetry));
    out.visibility = VisibilityFromTelemetry(telemetry, solver.degraded, valid_state);
    out.evidence = EvidenceFromTelemetry(telemetry, fallback_confidence);
    out.depth_source = telemetry.depth_source;
    out.measured = MeasurementPresent(telemetry);
    out.predicted = !out.measured && valid_state;
    out.camera_a_present = telemetry.camera_a_present;
    out.camera_b_present = telemetry.camera_b_present;
    out.camera_a_confidence = telemetry.camera_a_confidence;
    out.camera_b_confidence = telemetry.camera_b_confidence;
    out.camera_a_weight = telemetry.camera_a_weight;
    out.camera_b_weight = telemetry.camera_b_weight;
    out.camera_a_quality = telemetry.camera_a_quality;
    out.camera_b_quality = telemetry.camera_b_quality;
    out.evidence_source = telemetry.evidence_source;
    out.triangulated = telemetry.triangulated;
    out.depth_inferred = telemetry.depth_inferred;
    out.reprojection_error_px = telemetry.mean_reprojection_error_px;
    out.estimated_depth_m = telemetry.estimated_depth_m;
    out.contact_support_confidence = telemetry.contact_confidence;
    out.solver_observation_weighted = telemetry.solver_observation_weighted;
    out.solver_observation_weight_scale = telemetry.solver_observation_weight_scale;
    out.solver_observation_confidence_ceiling = SolverObservationConfidenceCeiling(telemetry);
    out.identity_confidence = Clamp01(identity_confidence);
    if (out.measured) {
        out.reason = std::string("measured_") + ToString(out.evidence_source);
    } else if (telemetry.evidence_source == JointEvidenceSource::TemporalHold) {
        out.reason = "temporal_hold";
    } else if (telemetry.evidence_source == JointEvidenceSource::Rejected) {
        out.reason = "rejected";
    } else if (CameraEvidencePresent(telemetry)) {
        out.reason = "camera_seen_unusable";
    } else {
        out.reason = out.valid ? "predicted" : "untracked";
    }
    return out;
}

template <std::size_t N>
BodyStateJointMeasurement MergeMeasurements(
    const BodyStateSolverSnapshot& solver,
    const std::array<KeypointId, N>& ids,
    Vec3f fallback_world) {

    BodyStateJointMeasurement out;
    Vec3f world_sum{};
    float confidence_sum = 0.0f;
    float reprojection_sum = 0.0f;
    float depth_sum = 0.0f;
    float solver_weight_sum = 0.0f;
    float solver_confidence_ceiling_sum = 0.0f;
    int world_count = 0;
    int confidence_count = 0;
    int reprojection_count = 0;
    int depth_count = 0;
    int solver_weight_count = 0;
    for (const KeypointId id : ids) {
        const auto telemetry = JointTelemetryOrEmpty(solver, id);
        out.camera_a_present = out.camera_a_present || telemetry.camera_a_present;
        out.camera_b_present = out.camera_b_present || telemetry.camera_b_present;
        out.camera_a_confidence = std::max(out.camera_a_confidence, telemetry.camera_a_confidence);
        out.camera_b_confidence = std::max(out.camera_b_confidence, telemetry.camera_b_confidence);
        out.camera_a_weight = std::max(out.camera_a_weight, telemetry.camera_a_weight);
        out.camera_b_weight = std::max(out.camera_b_weight, telemetry.camera_b_weight);
        out.camera_a_quality = std::max(out.camera_a_quality, telemetry.camera_a_quality);
        out.camera_b_quality = std::max(out.camera_b_quality, telemetry.camera_b_quality);
        out.triangulated = out.triangulated || telemetry.triangulated;
        out.depth_inferred = out.depth_inferred || telemetry.depth_inferred;
        if (telemetry.triangulated) {
            out.depth_source = DepthSource::TriangulatedStereo;
            out.evidence_source = JointEvidenceSource::Stereo;
        } else if (out.depth_source == DepthSource::None && telemetry.depth_inferred) {
            out.depth_source = telemetry.depth_source;
            out.evidence_source = telemetry.evidence_source;
        } else if (out.evidence_source == JointEvidenceSource::None ||
            out.evidence_source == JointEvidenceSource::TemporalHold) {
            out.evidence_source = telemetry.evidence_source;
        }
        if (MeasurementPresent(telemetry)) {
            confidence_sum += Clamp01(telemetry.confidence);
            ++confidence_count;
            if (IsFinite(telemetry.world)) {
                world_sum = Add(world_sum, telemetry.world);
                ++world_count;
            }
            if (telemetry.depth_inferred && std::isfinite(telemetry.estimated_depth_m) && telemetry.estimated_depth_m > 0.0f) {
                depth_sum += telemetry.estimated_depth_m;
                ++depth_count;
            }
        }
        if (telemetry.triangulated && std::isfinite(telemetry.mean_reprojection_error_px)) {
            reprojection_sum += telemetry.mean_reprojection_error_px;
            ++reprojection_count;
        }
        if (telemetry.solver_observation_weighted) {
            out.solver_observation_weighted = true;
            solver_weight_sum += telemetry.solver_observation_weight_scale;
            solver_confidence_ceiling_sum += SolverObservationConfidenceCeiling(telemetry);
            ++solver_weight_count;
        }
    }
    out.world = world_count > 0 ? Scale(world_sum, 1.0f / static_cast<float>(world_count)) : fallback_world;
    out.confidence = confidence_count > 0 ? confidence_sum / static_cast<float>(confidence_count) : 0.0f;
    out.mean_reprojection_error_px = reprojection_count > 0 ? reprojection_sum / static_cast<float>(reprojection_count) : 0.0f;
    out.estimated_depth_m = depth_count > 0 ? depth_sum / static_cast<float>(depth_count) : 0.0f;
    out.solver_observation_weight_scale = solver_weight_count > 0
        ? solver_weight_sum / static_cast<float>(solver_weight_count)
        : 1.0f;
    out.solver_observation_confidence_ceiling = solver_weight_count > 0
        ? solver_confidence_ceiling_sum / static_cast<float>(solver_weight_count)
        : 1.0f;
    return out;
}

bool UsableMeasurementWorld(const BodyStateJointMeasurement& telemetry) {
    return MeasurementPresent(telemetry) && IsFinite(telemetry.world);
}

BodyStateJointMeasurement ChestMeasurement(
    const BodyStateSolverSnapshot& solver,
    Vec3f fallback_world) {

    auto out = MergeMeasurements(
        solver,
        std::array<KeypointId, 3>{KeypointId::LeftShoulder, KeypointId::RightShoulder, KeypointId::Neck},
        fallback_world);

    const auto left = JointTelemetryOrEmpty(solver, KeypointId::LeftShoulder);
    const auto right = JointTelemetryOrEmpty(solver, KeypointId::RightShoulder);
    const auto neck = JointTelemetryOrEmpty(solver, KeypointId::Neck);
    const bool left_ok = UsableMeasurementWorld(left);
    const bool right_ok = UsableMeasurementWorld(right);
    const bool neck_ok = UsableMeasurementWorld(neck);

    float coverage_factor = 0.0f;
    Vec3f measured_world = fallback_world;
    if (left_ok && right_ok) {
        const Vec3f shoulder_mid = Scale(Add(left.world, right.world), 0.5f);
        measured_world = neck_ok ? Lerp(shoulder_mid, neck.world, 0.20f) : shoulder_mid;
        coverage_factor = neck_ok ? 1.0f : 0.85f;
    } else if ((left_ok || right_ok) && neck_ok) {
        const Vec3f shoulder = left_ok ? left.world : right.world;
        const Vec3f partial_mid = Scale(Add(shoulder, neck.world), 0.5f);
        measured_world = Lerp(fallback_world, partial_mid, 0.55f);
        coverage_factor = 0.65f;
    } else if (neck_ok) {
        measured_world = Lerp(fallback_world, neck.world, 0.45f);
        coverage_factor = 0.50f;
    } else if (left_ok || right_ok) {
        const Vec3f shoulder = left_ok ? left.world : right.world;
        measured_world = Lerp(fallback_world, shoulder, 0.20f);
        coverage_factor = 0.35f;
    }

    out.world = measured_world;
    out.confidence = Clamp01(out.confidence * coverage_factor);
    if (coverage_factor <= 0.0f) {
        out.triangulated = false;
        out.depth_inferred = false;
        out.evidence_source = JointEvidenceSource::None;
        out.depth_source = DepthSource::None;
    }
    return out;
}

struct UpperBodyDimensions {
    float torso_height = 0.54f;
    float shoulder_half_span = 0.20f;
    float upper_arm = 0.31f;
    float lower_arm = 0.27f;
};

struct UpperBodyFallback {
    Pose3f chest{};
    Vec3f neck{};
    Vec3f head{};
    Vec3f left_shoulder{};
    Vec3f right_shoulder{};
    Vec3f left_elbow{};
    Vec3f right_elbow{};
    Vec3f left_wrist{};
    Vec3f right_wrist{};
};

float PlausibleLength(float candidate, float fallback, float min_value, float max_value) {
    if (std::isfinite(candidate) && candidate >= min_value && candidate <= max_value) {
        return candidate;
    }
    return std::clamp(fallback, min_value, max_value);
}

UpperBodyDimensions EstimateUpperBodyDimensions(
    const LowerBodyModel& model,
    const BodyCalibration& body_calibration) {

    const float model_left_leg = std::max(0.1f, model.left_femur + model.left_tibia);
    const float model_right_leg = std::max(0.1f, model.right_femur + model.right_tibia);
    const float model_leg = 0.5f * (model_left_leg + model_right_leg);

    const float calibrated_left_leg = body_calibration.left_femur + body_calibration.left_tibia;
    const float calibrated_right_leg = body_calibration.right_femur + body_calibration.right_tibia;
    const bool calibrated_lengths_usable =
        body_calibration.quality.overall > 0.2f &&
        std::isfinite(calibrated_left_leg) &&
        std::isfinite(calibrated_right_leg) &&
        calibrated_left_leg > 0.45f &&
        calibrated_left_leg < 1.20f &&
        calibrated_right_leg > 0.45f &&
        calibrated_right_leg < 1.20f;
    const float leg = calibrated_lengths_usable
        ? 0.5f * (calibrated_left_leg + calibrated_right_leg)
        : model_leg;

    const float pelvis_width = PlausibleLength(
        body_calibration.quality.pelvis_width > 0.2f ? body_calibration.pelvis_width : model.pelvis_width,
        model.pelvis_width,
        0.18f,
        0.48f);

    UpperBodyDimensions out;
    out.torso_height = std::clamp(0.64f * leg, 0.42f, 0.72f);
    out.shoulder_half_span = std::clamp(0.62f * pelvis_width, 0.16f, 0.29f);

    // Arm lengths are not calibrated by the lower-body estimator. Derive a
    // conservative adult proportion from leg length, then clamp tightly enough
    // to reject obviously impossible 3D keypoint jumps while still accepting
    // camera/model bias.
    const float estimated_height = std::clamp(2.04f * leg, 1.30f, 2.15f);
    out.upper_arm = std::clamp(0.186f * estimated_height, 0.22f, 0.40f);
    out.lower_arm = std::clamp(0.164f * estimated_height, 0.19f, 0.36f);
    return out;
}

UpperBodyFallback EstimateUpperBodyFallback(
    const LowerBodyState& state,
    const LowerBodyModel& model,
    const BodyCalibration& body_calibration) {

    const UpperBodyDimensions dims = EstimateUpperBodyDimensions(model, body_calibration);
    const Vec3f up = Rotate(state.root.orientation, Vec3f{0.0f, 1.0f, 0.0f});
    const Vec3f right = Rotate(state.root.orientation, Vec3f{1.0f, 0.0f, 0.0f});
    const Vec3f forward = Rotate(state.root.orientation, Vec3f{0.0f, 0.0f, 1.0f});
    const Vec3f pelvis = state.root.position;
    UpperBodyFallback out;
    out.chest.orientation = state.root.orientation;
    out.chest.position = Add(pelvis, Scale(up, 0.62f * dims.torso_height));
    out.neck = Add(pelvis, Scale(up, 0.86f * dims.torso_height));
    out.head = Add(pelvis, Scale(up, 1.03f * dims.torso_height));
    out.left_shoulder = Add(out.neck, Add(Scale(right, -dims.shoulder_half_span), Scale(up, -0.06f)));
    out.right_shoulder = Add(out.neck, Add(Scale(right, dims.shoulder_half_span), Scale(up, -0.06f)));
    out.left_elbow = Add(out.left_shoulder, Add(Scale(right, -0.55f * dims.upper_arm), Add(Scale(up, -0.72f * dims.upper_arm), Scale(forward, 0.08f * dims.upper_arm))));
    out.right_elbow = Add(out.right_shoulder, Add(Scale(right, 0.55f * dims.upper_arm), Add(Scale(up, -0.72f * dims.upper_arm), Scale(forward, 0.08f * dims.upper_arm))));
    out.left_wrist = Add(out.left_elbow, Add(Scale(right, -0.35f * dims.lower_arm), Scale(up, -0.94f * dims.lower_arm)));
    out.right_wrist = Add(out.right_elbow, Add(Scale(right, 0.35f * dims.lower_arm), Scale(up, -0.94f * dims.lower_arm)));
    return out;
}

float SegmentLengthConsistency(float observed, float target) {
    if (!std::isfinite(observed) || !std::isfinite(target) || target <= 1e-5f) {
        return 0.45f;
    }
    const float relative_error = std::abs(observed - target) / target;
    if (relative_error <= 0.18f) {
        return 1.0f;
    }
    if (relative_error >= 0.85f) {
        return 0.30f;
    }
    const float t = (relative_error - 0.18f) / (0.85f - 0.18f);
    return Lerp(1.0f, 0.30f, t);
}

Vec3f ClampVectorLength(Vec3f v, float max_length, Vec3f fallback_dir) {
    const float len = Length(v);
    if (!std::isfinite(len) || len <= 1e-5f) {
        return Scale(NormalizeOr(fallback_dir, Vec3f{0.0f, 1.0f, 0.0f}), max_length);
    }
    if (len <= max_length) {
        return v;
    }
    return Scale(v, max_length / len);
}

Vec3f FixedSegmentPoint(Vec3f parent, Vec3f candidate, Vec3f fallback, float segment_length, float* confidence_scale) {
    const Vec3f fallback_dir = Sub(fallback, parent);
    Vec3f dir = NormalizeOr(Sub(candidate, parent), NormalizeOr(fallback_dir, Vec3f{0.0f, -1.0f, 0.0f}));
    const float observed = Distance(parent, candidate);
    if (confidence_scale) {
        *confidence_scale *= SegmentLengthConsistency(observed, segment_length);
    }
    return Add(parent, Scale(dir, segment_length));
}


bool MonocularInferred(const BodyStateJointMeasurement& telemetry) {
    return telemetry.depth_inferred && !telemetry.triangulated;
}

float MonocularMeasurementTrust(const BodyStateJointMeasurement& telemetry, float base) {
    if (!MonocularInferred(telemetry)) {
        return 1.0f;
    }
    const float confidence = Clamp01(telemetry.confidence);
    const float ceiling = telemetry.solver_observation_weighted
        ? SolverObservationConfidenceCeiling(telemetry)
        : 1.0f;
    return Clamp01(base * std::max(0.20f, std::min(confidence, ceiling)));
}

Vec3f StabilizeMonocularSegmentTarget(
    Vec3f parent,
    Vec3f candidate,
    Vec3f fallback,
    float segment_length,
    const BodyStateJointMeasurement& telemetry,
    float max_error_fraction,
    float base_trust,
    float* confidence_scale) {

    if (!MonocularInferred(telemetry)) {
        return candidate;
    }
    const Vec3f fallback_delta = Sub(fallback, parent);
    const Vec3f measured_delta = Sub(candidate, parent);
    const float max_error = std::max(0.045f, max_error_fraction * std::max(0.05f, segment_length));
    const Vec3f error = ClampVectorLength(Sub(measured_delta, fallback_delta), max_error, Sub(measured_delta, fallback_delta));
    const float trust = MonocularMeasurementTrust(telemetry, base_trust);
    if (confidence_scale) {
        *confidence_scale *= std::max(0.35f, trust);
    }
    return Add(parent, Add(fallback_delta, Scale(error, trust)));
}

Vec3f SolveArmElbow(
    Vec3f shoulder,
    Vec3f wrist,
    Vec3f fallback_elbow,
    const LowerBodyState& state,
    const UpperBodyDimensions& dims,
    bool left_side,
    Vec3f* adjusted_wrist,
    float* confidence_scale);

Vec3f MonocularArmPlaneTarget(
    Vec3f shoulder,
    Vec3f measured,
    Vec3f fallback,
    float max_distance,
    const BodyStateJointMeasurement& telemetry,
    float* confidence_scale) {

    // Monocular arm depth is not observable enough to publish raw elbow/wrist z.
    // Keep screen-plane motion (x/y relative to the shoulder), keep depth near
    // the kinematic fallback, then let the fixed-length two-bone solve own the
    // actual elbow position. This avoids tiny inferred-z errors turning into
    // huge elbow tracker jumps.
    const Vec3f fallback_delta = Sub(fallback, shoulder);
    Vec3f measured_delta = Sub(measured, shoulder);
    if (!IsFinite(measured_delta)) {
        measured_delta = fallback_delta;
    }
    const float depth_delta = std::clamp(measured_delta.z - fallback_delta.z, -0.045f, 0.045f);
    Vec3f planar_delta{measured_delta.x, measured_delta.y, fallback_delta.z + depth_delta};
    const Vec3f raw_error = Sub(planar_delta, fallback_delta);
    const float max_error = std::max(0.08f, 0.55f * std::max(0.05f, max_distance));
    planar_delta = Add(fallback_delta, ClampVectorLength(raw_error, max_error, raw_error));
    planar_delta = ClampVectorLength(planar_delta, std::max(0.04f, max_distance), fallback_delta);
    if (confidence_scale) {
        const float raw_length = Length(measured_delta);
        const float length_consistency = SegmentLengthConsistency(raw_length, std::max(0.05f, max_distance));
        const float trust = MonocularMeasurementTrust(telemetry, 0.80f);
        *confidence_scale *= std::max(0.35f, std::min(length_consistency, trust));
    }
    return Add(shoulder, planar_delta);
}

Vec3f SolveMonocularArmElbow(
    Vec3f shoulder,
    const BodyStateJointMeasurement& elbow_telemetry,
    const BodyStateJointMeasurement& wrist_telemetry,
    Vec3f fallback_elbow,
    Vec3f fallback_wrist,
    const LowerBodyState& state,
    const UpperBodyDimensions& dims,
    bool left_side,
    Vec3f* out_wrist,
    float* elbow_confidence_scale,
    float* wrist_confidence_scale) {

    const float max_reach = std::max(0.05f, dims.upper_arm + dims.lower_arm - 0.015f);
    const bool wrist_ok = UsableMeasurementWorld(wrist_telemetry);
    Vec3f wrist_target = fallback_wrist;
    if (wrist_ok) {
        wrist_target = MonocularArmPlaneTarget(
            shoulder,
            wrist_telemetry.world,
            fallback_wrist,
            max_reach,
            wrist_telemetry,
            wrist_confidence_scale);
    }

    Vec3f elbow_hint = fallback_elbow;
    if (UsableMeasurementWorld(elbow_telemetry)) {
        elbow_hint = MonocularArmPlaneTarget(
            shoulder,
            elbow_telemetry.world,
            fallback_elbow,
            dims.upper_arm,
            elbow_telemetry,
            elbow_confidence_scale);
    }

    Vec3f adjusted_wrist = wrist_target;
    const Vec3f elbow = SolveArmElbow(
        shoulder,
        wrist_target,
        elbow_hint,
        state,
        dims,
        left_side,
        &adjusted_wrist,
        wrist_confidence_scale);
    if (out_wrist) {
        *out_wrist = adjusted_wrist;
    }
    return elbow;
}

BodyStateJoint MakeKinematicKneeJoint(
    BodyJointRole role,
    const LowerBodyJointSet& joints,
    KeypointId id,
    Vec3f fallback,
    Vec3f velocity,
    const BodyStateSolverSnapshot& solver,
    float fallback_confidence,
    bool valid_state,
    float identity_confidence) {

    const auto telemetry = JointTelemetryOrEmpty(solver, id);
    const Vec3f kinematic = JointPositionOr(joints, id, fallback);
    BodyStateJoint out = MakeMeasuredJoint(
        role, joints, id, fallback, velocity, solver, fallback_confidence, valid_state, identity_confidence);
    if (MeasurementPresent(telemetry) && MonocularInferred(telemetry) && IsFinite(kinematic)) {
        const float trust = MonocularMeasurementTrust(telemetry, 0.45f);
        const float max_error = 0.16f;
        const Vec3f correction = ClampVectorLength(Sub(out.position, kinematic), max_error, Sub(out.position, kinematic));
        out.position = Add(kinematic, Scale(correction, trust));
        out.confidence = Clamp01(out.confidence * std::max(0.45f, trust));
        out.valid = valid_state && IsFinite(out.position) && (out.confidence > 0.0f || MeasurementWorldPresent(telemetry));
        out.reason = "kinematic_knee_monocular_bounded";
    }
    return out;
}

Vec3f ConstrainChestToTorso(
    const LowerBodyState& state,
    Vec3f candidate,
    const UpperBodyFallback& fallback,
    const UpperBodyDimensions& dims,
    float* confidence_scale) {

    const Vec3f pelvis = state.root.position;
    const Vec3f up = Rotate(state.root.orientation, Vec3f{0.0f, 1.0f, 0.0f});
    const Vec3f offset = Sub(candidate, pelvis);
    const float vertical = Dot(offset, up);
    const float min_vertical = 0.48f * dims.torso_height;
    const float max_vertical = 0.74f * dims.torso_height;
    const float clamped_vertical = std::clamp(
        std::isfinite(vertical) ? vertical : 0.62f * dims.torso_height,
        min_vertical,
        max_vertical);

    Vec3f lateral = Sub(offset, Scale(up, std::isfinite(vertical) ? vertical : 0.0f));
    lateral = ClampVectorLength(lateral, std::max(0.08f, 0.75f * dims.shoulder_half_span), lateral);
    const Vec3f constrained = Add(pelvis, Add(Scale(up, clamped_vertical), lateral));

    if (confidence_scale) {
        const float correction = Distance(candidate, constrained);
        const float tolerance = std::max(0.08f, 0.30f * dims.torso_height);
        *confidence_scale *= Clamp01(1.0f - correction / (2.0f * tolerance));
        *confidence_scale = std::max(*confidence_scale, 0.25f);
    }
    return IsFinite(constrained) ? constrained : fallback.chest.position;
}

struct ShoulderPairEstimate {
    Vec3f left{};
    Vec3f right{};
    float left_scale = 1.0f;
    float right_scale = 1.0f;
};

ShoulderPairEstimate ConstrainShoulders(
    const BodyStateSolverSnapshot& solver,
    const UpperBodyFallback& fallback,
    const UpperBodyDimensions& dims,
    const LowerBodyState& state) {

    const auto left = JointTelemetryOrEmpty(solver, KeypointId::LeftShoulder);
    const auto right = JointTelemetryOrEmpty(solver, KeypointId::RightShoulder);
    const bool left_ok = UsableMeasurementWorld(left);
    const bool right_ok = UsableMeasurementWorld(right);
    const Vec3f fallback_right = Rotate(state.root.orientation, Vec3f{1.0f, 0.0f, 0.0f});

    ShoulderPairEstimate out;
    out.left = fallback.left_shoulder;
    out.right = fallback.right_shoulder;
    if (left_ok && right_ok) {
        const Vec3f fallback_mid = Scale(Add(fallback.left_shoulder, fallback.right_shoulder), 0.5f);
        const Vec3f measured_mid = Scale(Add(left.world, right.world), 0.5f);
        const Vec3f measured_axis = Sub(right.world, left.world);
        const float measured_span = Length(measured_axis);
        const Vec3f axis = NormalizeOr(measured_axis, fallback_right);
        const Vec3f mid_delta = Sub(measured_mid, fallback_mid);
        const float max_mid_delta = std::max(0.18f, 1.60f * dims.shoulder_half_span);
        const Vec3f clamped_mid_delta = ClampVectorLength(mid_delta, max_mid_delta, mid_delta);
        const Vec3f constrained_mid = Add(fallback_mid, clamped_mid_delta);
        out.left = Add(constrained_mid, Scale(axis, -dims.shoulder_half_span));
        out.right = Add(constrained_mid, Scale(axis, dims.shoulder_half_span));
        float scale = SegmentLengthConsistency(0.5f * measured_span, dims.shoulder_half_span);
        if (Length(mid_delta) > max_mid_delta) {
            scale *= 0.55f;
        }
        out.left_scale = scale;
        out.right_scale = scale;
        return out;
    }

    if (left_ok || right_ok) {
        const bool left_present = left_ok;
        const Vec3f measured = left_present ? left.world : right.world;
        const Vec3f fallback_measured = left_present ? fallback.left_shoulder : fallback.right_shoulder;
        const Vec3f fallback_other = left_present ? fallback.right_shoulder : fallback.left_shoulder;
        const Vec3f raw_delta = Sub(measured, fallback_measured);
        const float max_delta = std::max(0.16f, 1.35f * dims.shoulder_half_span);
        const Vec3f clamped_delta = ClampVectorLength(raw_delta, max_delta, raw_delta);
        const Vec3f nudged = Add(fallback_measured, Scale(clamped_delta, 0.65f));
        const Vec3f shoulder_delta = Sub(nudged, fallback_measured);
        const Vec3f other = Add(fallback_other, Scale(shoulder_delta, 0.35f));
        const float single_scale = Length(raw_delta) > max_delta ? 0.45f : 0.85f;
        if (left_present) {
            out.left = nudged;
            out.right = other;
            out.left_scale = single_scale;
            out.right_scale = 0.55f;
        } else {
            out.right = nudged;
            out.left = other;
            out.right_scale = single_scale;
            out.left_scale = 0.55f;
        }
    }
    return out;
}

Vec3f SolveArmElbow(
    Vec3f shoulder,
    Vec3f wrist,
    Vec3f fallback_elbow,
    const LowerBodyState& state,
    const UpperBodyDimensions& dims,
    bool left_side,
    Vec3f* adjusted_wrist,
    float* confidence_scale) {

    Vec3f to_wrist = Sub(wrist, shoulder);
    float distance = Length(to_wrist);
    const float max_reach = std::max(0.05f, dims.upper_arm + dims.lower_arm - 0.015f);
    const float min_reach = std::max(0.02f, std::abs(dims.upper_arm - dims.lower_arm) + 0.015f);
    if (!std::isfinite(distance) || distance <= 1e-5f) {
        to_wrist = Sub(fallback_elbow, shoulder);
        distance = Length(to_wrist);
    }
    Vec3f dir = NormalizeOr(to_wrist, NormalizeOr(Sub(fallback_elbow, shoulder), Vec3f{0.0f, -1.0f, 0.0f}));
    if (distance > max_reach) {
        wrist = Add(shoulder, Scale(dir, max_reach));
        distance = max_reach;
        if (confidence_scale) {
            *confidence_scale *= 0.55f;
        }
    } else if (distance < min_reach) {
        wrist = Add(shoulder, Scale(dir, min_reach));
        distance = min_reach;
        if (confidence_scale) {
            *confidence_scale *= 0.65f;
        }
    }

    const float x = std::clamp(
        (dims.upper_arm * dims.upper_arm + distance * distance - dims.lower_arm * dims.lower_arm) /
            std::max(1e-5f, 2.0f * distance),
        0.0f,
        dims.upper_arm);
    const float h_sq = std::max(0.0f, dims.upper_arm * dims.upper_arm - x * x);
    const float h = std::sqrt(h_sq);

    const Vec3f up = Rotate(state.root.orientation, Vec3f{0.0f, 1.0f, 0.0f});
    const Vec3f right = Rotate(state.root.orientation, Vec3f{1.0f, 0.0f, 0.0f});
    const Vec3f forward = Rotate(state.root.orientation, Vec3f{0.0f, 0.0f, 1.0f});
    const Vec3f side = left_side ? Scale(right, -1.0f) : right;
    Vec3f bend_hint = Add(Scale(side, 0.35f), Add(Scale(up, -0.90f), Scale(forward, 0.10f)));
    bend_hint = Sub(bend_hint, Scale(dir, Dot(bend_hint, dir)));
    bend_hint = NormalizeOr(bend_hint, NormalizeOr(Sub(fallback_elbow, Add(shoulder, Scale(dir, x))), side));

    if (adjusted_wrist) {
        *adjusted_wrist = wrist;
    }
    return Add(Add(shoulder, Scale(dir, x)), Scale(bend_hint, h));
}

struct ArmEstimate {
    Vec3f shoulder{};
    Vec3f elbow{};
    Vec3f wrist{};
    float shoulder_scale = 1.0f;
    float elbow_scale = 1.0f;
    float wrist_scale = 1.0f;
};

ArmEstimate ConstrainArm(
    const BodyStateSolverSnapshot& solver,
    const LowerBodyState& state,
    const UpperBodyFallback& fallback,
    const UpperBodyDimensions& dims,
    bool left_side,
    Vec3f shoulder,
    float shoulder_scale) {

    const KeypointId elbow_id = left_side ? KeypointId::LeftElbow : KeypointId::RightElbow;
    const KeypointId wrist_id = left_side ? KeypointId::LeftWrist : KeypointId::RightWrist;
    const auto elbow_telemetry = JointTelemetryOrEmpty(solver, elbow_id);
    const auto wrist_telemetry = JointTelemetryOrEmpty(solver, wrist_id);
    const bool elbow_ok = UsableMeasurementWorld(elbow_telemetry);
    const bool wrist_ok = UsableMeasurementWorld(wrist_telemetry);
    const Vec3f fallback_elbow = left_side ? fallback.left_elbow : fallback.right_elbow;
    const Vec3f fallback_wrist = left_side ? fallback.left_wrist : fallback.right_wrist;

    ArmEstimate out;
    out.shoulder = shoulder;
    out.shoulder_scale = shoulder_scale;
    out.elbow = fallback_elbow;
    out.wrist = fallback_wrist;
    out.elbow_scale = 0.62f;
    out.wrist_scale = 0.55f;

    if (MonocularInferred(elbow_telemetry) || MonocularInferred(wrist_telemetry)) {
        out.elbow_scale = elbow_ok ? 0.92f : 0.66f;
        out.wrist_scale = wrist_ok ? 0.86f : 0.52f;
        Vec3f adjusted_wrist = fallback_wrist;
        out.elbow = SolveMonocularArmElbow(
            shoulder,
            elbow_telemetry,
            wrist_telemetry,
            fallback_elbow,
            fallback_wrist,
            state,
            dims,
            left_side,
            &adjusted_wrist,
            &out.elbow_scale,
            &out.wrist_scale);
        out.wrist = wrist_ok
            ? adjusted_wrist
            : FixedSegmentPoint(out.elbow, fallback_wrist, fallback_wrist, dims.lower_arm, &out.wrist_scale);
        return out;
    }

    if (elbow_ok) {
        out.elbow_scale = 1.0f;
        out.elbow = FixedSegmentPoint(shoulder, elbow_telemetry.world, fallback_elbow, dims.upper_arm, &out.elbow_scale);
        if (wrist_ok) {
            out.wrist_scale = 1.0f;
            out.wrist = FixedSegmentPoint(out.elbow, wrist_telemetry.world, fallback_wrist, dims.lower_arm, &out.wrist_scale);
        } else {
            out.wrist = FixedSegmentPoint(out.elbow, fallback_wrist, fallback_wrist, dims.lower_arm, &out.wrist_scale);
        }
        return out;
    }

    if (wrist_ok) {
        out.wrist_scale = 1.0f;
        Vec3f adjusted_wrist = wrist_telemetry.world;
        out.elbow_scale = 0.72f;
        out.elbow = SolveArmElbow(
            shoulder,
            wrist_telemetry.world,
            fallback_elbow,
            state,
            dims,
            left_side,
            &adjusted_wrist,
            &out.wrist_scale);
        out.wrist = adjusted_wrist;
        return out;
    }

    out.elbow = FixedSegmentPoint(shoulder, fallback_elbow, fallback_elbow, dims.upper_arm, &out.elbow_scale);
    out.wrist = FixedSegmentPoint(out.elbow, fallback_wrist, fallback_wrist, dims.lower_arm, &out.wrist_scale);
    return out;
}

BodyStateJoint MakeConstrainedTelemetryJoint(
    BodyJointRole role,
    const BodyStateJointMeasurement& telemetry,
    Vec3f constrained_position,
    Vec3f velocity,
    float fallback_confidence,
    float confidence_scale,
    bool valid_state,
    bool degraded,
    float identity_confidence,
    const char* constrained_reason) {

    BodyStateJoint out;
    out.role = role;
    out.position = constrained_position;
    out.velocity = velocity;
    out.confidence = Clamp01(RoleConfidence(telemetry, fallback_confidence, valid_state, identity_confidence) * confidence_scale);
    out.valid = valid_state && IsFinite(out.position) && (out.confidence > 0.0f || MeasurementWorldPresent(telemetry));
    out.visibility = MeasurementPresent(telemetry)
        ? VisibilityFromTelemetry(telemetry, degraded, valid_state)
        : (out.valid ? BodyJointVisibility::Predicted : BodyJointVisibility::MissingUnknown);
    out.evidence = EvidenceFromTelemetry(telemetry, fallback_confidence);
    if (!out.evidence.valid && out.valid) {
        out.evidence.source = TrackerEvidenceSource::Predicted;
        out.evidence.direct_confidence = out.confidence;
        out.evidence.valid = out.confidence > 0.0f;
    }
    out.depth_source = telemetry.depth_source;
    out.measured = MeasurementPresent(telemetry);
    out.predicted = !out.measured && out.valid;
    out.camera_a_present = telemetry.camera_a_present;
    out.camera_b_present = telemetry.camera_b_present;
    out.camera_a_confidence = telemetry.camera_a_confidence;
    out.camera_b_confidence = telemetry.camera_b_confidence;
    out.camera_a_weight = telemetry.camera_a_weight;
    out.camera_b_weight = telemetry.camera_b_weight;
    out.camera_a_quality = telemetry.camera_a_quality;
    out.camera_b_quality = telemetry.camera_b_quality;
    out.evidence_source = telemetry.evidence_source;
    out.triangulated = telemetry.triangulated;
    out.depth_inferred = telemetry.depth_inferred;
    out.reprojection_error_px = telemetry.mean_reprojection_error_px;
    out.estimated_depth_m = telemetry.estimated_depth_m;
    out.contact_support_confidence = telemetry.contact_confidence;
    out.solver_observation_weighted = telemetry.solver_observation_weighted;
    out.solver_observation_weight_scale = telemetry.solver_observation_weight_scale;
    out.solver_observation_confidence_ceiling = SolverObservationConfidenceCeiling(telemetry);
    out.identity_confidence = Clamp01(identity_confidence);
    if (out.measured) {
        out.reason = constrained_reason;
    } else if (out.valid) {
        out.reason = "kinematic_prediction";
    } else {
        out.reason = "untracked";
    }
    return out;
}


BodyStateJoint MakePoseJoint(
    BodyJointRole role,
    const Pose3f& pose,
    Vec3f velocity,
    TrackerEvidence evidence,
    float confidence,
    bool valid_state) {

    BodyStateJoint out;
    out.role = role;
    out.position = pose.position;
    out.velocity = velocity;
    out.confidence = Clamp01(confidence);
    out.valid = valid_state && IsFinite(out.position) && (out.confidence > 0.0f || evidence.valid || evidence.source != TrackerEvidenceSource::None);
    const bool anchored = evidence.source == TrackerEvidenceSource::AnchorHeld;
    out.measured = anchored ||
        evidence.source == TrackerEvidenceSource::DirectStereo ||
        evidence.source == TrackerEvidenceSource::InferredMonocular ||
        evidence.source == TrackerEvidenceSource::ReplayInput;
    out.predicted = !out.measured && valid_state;
    out.visibility = anchored
        ? BodyJointVisibility::Anchored
        : (evidence.valid ? BodyJointVisibility::Visible : BodyJointVisibility::Predicted);
    out.evidence = evidence;
    out.reason = anchored ? "anchored_contact" : (out.valid ? "state_pose" : "untracked");
    return out;
}

void ApplyAnchoredContact(
    BodyStateJoint& joint,
    const BodyStateContactEvidence& contact) {

    if (contact.lock_strength <= 0.0f || joint.measured) {
        return;
    }
    joint.evidence.anchor_held = true;
    joint.evidence.support_confidence = std::max(joint.evidence.support_confidence, contact.lock_strength);
    if (joint.evidence.source == TrackerEvidenceSource::None ||
        joint.evidence.source == TrackerEvidenceSource::Predicted) {
        joint.evidence.source = TrackerEvidenceSource::AnchorHeld;
    }
    joint.evidence.valid = joint.evidence.support_confidence > 0.0f;
    joint.measured = joint.evidence.valid;
    joint.predicted = !joint.measured && joint.valid;
    if (joint.measured) {
        joint.visibility = BodyJointVisibility::Anchored;
        joint.reason = "anchored_contact";
    }
}

bool CountsAsPredictionVisibility(BodyJointVisibility visibility) {
    return visibility == BodyJointVisibility::Predicted ||
        visibility == BodyJointVisibility::CameraOccluded ||
        visibility == BodyJointVisibility::BodyOccluded ||
        visibility == BodyJointVisibility::FloorOccluded;
}

BodyStateJointMeasurement MergeFootMeasurement(const BodyStateSolverSnapshot& solver, bool left) {
    const std::array<KeypointId, 4> ids = left
        ? std::array<KeypointId, 4>{KeypointId::LeftAnkle, KeypointId::LeftBigToe, KeypointId::LeftSmallToe, KeypointId::LeftHeel}
        : std::array<KeypointId, 4>{KeypointId::RightAnkle, KeypointId::RightBigToe, KeypointId::RightSmallToe, KeypointId::RightHeel};
    BodyStateJointMeasurement out;
    float confidence_sum = 0.0f;
    float reprojection_sum = 0.0f;
    float depth_sum = 0.0f;
    float solver_weight_sum = 0.0f;
    float solver_confidence_ceiling_sum = 0.0f;
    int confidence_count = 0;
    int reprojection_count = 0;
    int depth_count = 0;
    int solver_weight_count = 0;
    for (const KeypointId id : ids) {
        const auto telemetry = JointTelemetryOrEmpty(solver, id);
        out.camera_a_present = out.camera_a_present || telemetry.camera_a_present;
        out.camera_b_present = out.camera_b_present || telemetry.camera_b_present;
        out.camera_a_confidence = std::max(out.camera_a_confidence, telemetry.camera_a_confidence);
        out.camera_b_confidence = std::max(out.camera_b_confidence, telemetry.camera_b_confidence);
        out.camera_a_weight = std::max(out.camera_a_weight, telemetry.camera_a_weight);
        out.camera_b_weight = std::max(out.camera_b_weight, telemetry.camera_b_weight);
        out.camera_a_quality = std::max(out.camera_a_quality, telemetry.camera_a_quality);
        out.camera_b_quality = std::max(out.camera_b_quality, telemetry.camera_b_quality);
        out.triangulated = out.triangulated || telemetry.triangulated;
        out.depth_inferred = out.depth_inferred || telemetry.depth_inferred;
        out.contact_confidence = std::max(out.contact_confidence, telemetry.contact_confidence);
        if (telemetry.solver_observation_weighted) {
            out.solver_observation_weighted = true;
            solver_weight_sum += telemetry.solver_observation_weight_scale;
            solver_confidence_ceiling_sum += SolverObservationConfidenceCeiling(telemetry);
            ++solver_weight_count;
        }
        if (telemetry.triangulated) {
            out.depth_source = DepthSource::TriangulatedStereo;
            out.evidence_source = JointEvidenceSource::Stereo;
        } else if (out.depth_source == DepthSource::None && telemetry.depth_inferred) {
            out.depth_source = telemetry.depth_source;
            out.evidence_source = telemetry.evidence_source;
        } else if (out.evidence_source == JointEvidenceSource::None ||
            out.evidence_source == JointEvidenceSource::TemporalHold) {
            out.evidence_source = telemetry.evidence_source;
        }
        if (MeasurementPresent(telemetry)) {
            confidence_sum += Clamp01(telemetry.confidence);
            ++confidence_count;
            if (telemetry.depth_inferred && std::isfinite(telemetry.estimated_depth_m) && telemetry.estimated_depth_m > 0.0f) {
                depth_sum += telemetry.estimated_depth_m;
                ++depth_count;
            }
        }
        if (telemetry.triangulated && std::isfinite(telemetry.mean_reprojection_error_px)) {
            reprojection_sum += telemetry.mean_reprojection_error_px;
            ++reprojection_count;
        }
    }
    out.confidence = confidence_count > 0 ? confidence_sum / static_cast<float>(confidence_count) : 0.0f;
    out.mean_reprojection_error_px = reprojection_count > 0 ? reprojection_sum / static_cast<float>(reprojection_count) : 0.0f;
    out.estimated_depth_m = depth_count > 0 ? depth_sum / static_cast<float>(depth_count) : 0.0f;
    out.solver_observation_weight_scale = solver_weight_count > 0
        ? solver_weight_sum / static_cast<float>(solver_weight_count)
        : 1.0f;
    out.solver_observation_confidence_ceiling = solver_weight_count > 0
        ? solver_confidence_ceiling_sum / static_cast<float>(solver_weight_count)
        : 1.0f;
    return out;
}

void AttachMeasurementEvidence(
    BodyStateJoint& joint,
    const BodyStateJointMeasurement& telemetry,
    float fallback_confidence,
    float identity_confidence) {

    if (!MeasurementPresent(telemetry) && !CameraEvidencePresent(telemetry)) {
        return;
    }
    joint.measured = MeasurementPresent(telemetry);
    joint.predicted = !joint.measured && joint.valid;
    joint.visibility = VisibilityFromTelemetry(telemetry, false, joint.valid);
    joint.camera_a_present = telemetry.camera_a_present;
    joint.camera_b_present = telemetry.camera_b_present;
    joint.camera_a_confidence = telemetry.camera_a_confidence;
    joint.camera_b_confidence = telemetry.camera_b_confidence;
    joint.camera_a_weight = telemetry.camera_a_weight;
    joint.camera_b_weight = telemetry.camera_b_weight;
    joint.camera_a_quality = telemetry.camera_a_quality;
    joint.camera_b_quality = telemetry.camera_b_quality;
    joint.evidence_source = telemetry.evidence_source;
    joint.triangulated = telemetry.triangulated;
    joint.depth_inferred = telemetry.depth_inferred;
    joint.depth_source = telemetry.depth_source;
    joint.reprojection_error_px = telemetry.mean_reprojection_error_px;
    joint.estimated_depth_m = telemetry.estimated_depth_m;
    joint.contact_support_confidence = std::max(joint.contact_support_confidence, telemetry.contact_confidence);
    joint.solver_observation_weighted = telemetry.solver_observation_weighted;
    joint.solver_observation_weight_scale = telemetry.solver_observation_weight_scale;
    joint.solver_observation_confidence_ceiling = SolverObservationConfidenceCeiling(telemetry);
    joint.identity_confidence = Clamp01(identity_confidence);
    if (joint.measured) {
        joint.evidence = EvidenceFromTelemetry(telemetry, fallback_confidence);
        joint.confidence = Clamp01(std::max(
            joint.confidence,
            RoleConfidence(telemetry, fallback_confidence, joint.valid, identity_confidence)));
        joint.reason = std::string("state_pose_with_") + ToString(joint.evidence_source);
    } else if (telemetry.evidence_source == JointEvidenceSource::TemporalHold) {
        joint.reason = "state_pose_temporal_hold";
    } else if (telemetry.evidence_source == JointEvidenceSource::Rejected) {
        joint.reason = "state_pose_rejected_camera_evidence";
    } else {
        joint.reason = "state_pose_camera_seen_unusable";
    }
}

double LatencyPredictionSeconds(double dt_seconds, const BodyStateSolverSnapshot& solver, float confidence) {
    if (!std::isfinite(dt_seconds) || dt_seconds <= 0.0 || confidence < tracking_constants::kVisibleConfidenceThreshold) {
        return 0.0;
    }
    (void)solver;
    return std::clamp(0.35 * dt_seconds, 0.0, 0.025);
}

} // namespace

BodyFootContactState BodyContactStateFromSupport(const FootSupportState& support) {
    if (support.phase == FootSupportPhase::Slip) {
        return BodyFootContactState::SlidingError;
    }
    if (support.phase == FootSupportPhase::FlatPlant || support.contact_load == FootContactLoad::FullPlant) {
        return BodyFootContactState::FullPlant;
    }
    if (support.phase == FootSupportPhase::ToePivot || support.contact_load == FootContactLoad::ToeOnly) {
        return BodyFootContactState::ToeContact;
    }
    if (support.phase == FootSupportPhase::HeelLock ||
        support.phase == FootSupportPhase::ContactCandidate ||
        support.contact_load == FootContactLoad::HeelOnly) {
        return BodyFootContactState::HeelStrike;
    }
    if (support.phase == FootSupportPhase::Swing || support.phase == FootSupportPhase::ReleasePending) {
        return BodyFootContactState::Swing;
    }
    return BodyFootContactState::Unreliable;
}

BodyStateEvidence BuildBodyStateEvidence(const BodyStateSolverSnapshot& solver, const LowerBodyState& state) {
    BodyStateEvidence out;
    out.solver = solver;
    out.left_foot.contact = BodyContactStateFromSupport(state.support.left_foot);
    out.left_foot.support_confidence = FootSupportConfidence(state.support.left_foot);
    out.left_foot.anchor_active = state.support.left_foot.anchor.active;
    out.left_foot.heel_anchor_active = state.support.left_foot.heel_anchor.active;
    out.left_foot.toe_anchor_active = state.support.left_foot.toe_anchor.active;
    out.left_foot.lock_strength = Clamp01(std::max({
        out.left_foot.support_confidence,
        state.support.left_foot.anchor.confidence,
        state.support.left_foot.heel_anchor.confidence,
        state.support.left_foot.toe_anchor.confidence
    }));

    out.right_foot.contact = BodyContactStateFromSupport(state.support.right_foot);
    out.right_foot.support_confidence = FootSupportConfidence(state.support.right_foot);
    out.right_foot.anchor_active = state.support.right_foot.anchor.active;
    out.right_foot.heel_anchor_active = state.support.right_foot.heel_anchor.active;
    out.right_foot.toe_anchor_active = state.support.right_foot.toe_anchor.active;
    out.right_foot.lock_strength = Clamp01(std::max({
        out.right_foot.support_confidence,
        state.support.right_foot.anchor.confidence,
        state.support.right_foot.heel_anchor.confidence,
        state.support.right_foot.toe_anchor.confidence
    }));
    return out;
}

UnifiedBodyState BuildUnifiedBodyState(
    const LowerBodyState& state,
    const LowerBodyModel& model,
    const BodyStateEvidence& evidence,
    const BodyCalibration& body_calibration,
    double dt_seconds,
    bool state_valid) {

    const BodyStateSolverSnapshot& solver = evidence.solver;
    const float identity_confidence = IdentityConfidence(solver);
    const float identity_factor = IdentityRoleFactor(identity_confidence);
    UnifiedBodyState out;
    out.lower_body = state;
    out.joints = PredictLowerBodyJoints(state, model);
    out.valid = state_valid && IsFinite(state.root.position);
    out.left_foot_contact = evidence.left_foot.contact;
    out.right_foot_contact = evidence.right_foot.contact;

    TrackerEvidence pelvis_evidence;
    pelvis_evidence.source = out.valid
        ? (solver.degraded
            ? (solver.used_hmd ? TrackerEvidenceSource::HmdPrediction : TrackerEvidenceSource::Predicted)
            : TrackerEvidenceSource::DirectStereo)
        : TrackerEvidenceSource::None;
    pelvis_evidence.direct_confidence = Clamp01(state.confidence);
    pelvis_evidence.degraded = solver.degraded;
    pelvis_evidence.valid = out.valid;

    out.roles[BodyJointRoleIndex(BodyJointRole::Pelvis)] =
        MakePoseJoint(BodyJointRole::Pelvis, state.root, state.linear_velocity, pelvis_evidence, state.confidence * identity_factor, out.valid);
    out.roles[BodyJointRoleIndex(BodyJointRole::LeftFoot)] =
        MakePoseJoint(BodyJointRole::LeftFoot, state.left_foot, state.left_foot_linear_velocity, state.left_foot_evidence, state.confidence * identity_factor, out.valid);
    out.roles[BodyJointRoleIndex(BodyJointRole::RightFoot)] =
        MakePoseJoint(BodyJointRole::RightFoot, state.right_foot, state.right_foot_linear_velocity, state.right_foot_evidence, state.confidence * identity_factor, out.valid);
    AttachMeasurementEvidence(
        out.roles[BodyJointRoleIndex(BodyJointRole::LeftFoot)],
        MergeFootMeasurement(solver, true),
        state.confidence,
        identity_confidence);
    AttachMeasurementEvidence(
        out.roles[BodyJointRoleIndex(BodyJointRole::RightFoot)],
        MergeFootMeasurement(solver, false),
        state.confidence,
        identity_confidence);
    out.roles[BodyJointRoleIndex(BodyJointRole::LeftFoot)].contact_lock_strength = evidence.left_foot.lock_strength;
    out.roles[BodyJointRoleIndex(BodyJointRole::LeftFoot)].contact_support_confidence = std::max(
        out.roles[BodyJointRoleIndex(BodyJointRole::LeftFoot)].contact_support_confidence,
        evidence.left_foot.support_confidence);
    out.roles[BodyJointRoleIndex(BodyJointRole::LeftFoot)].identity_confidence = identity_confidence;
    ApplyAnchoredContact(out.roles[BodyJointRoleIndex(BodyJointRole::LeftFoot)], evidence.left_foot);
    out.roles[BodyJointRoleIndex(BodyJointRole::RightFoot)].contact_lock_strength = evidence.right_foot.lock_strength;
    out.roles[BodyJointRoleIndex(BodyJointRole::RightFoot)].contact_support_confidence = std::max(
        out.roles[BodyJointRoleIndex(BodyJointRole::RightFoot)].contact_support_confidence,
        evidence.right_foot.support_confidence);
    out.roles[BodyJointRoleIndex(BodyJointRole::RightFoot)].identity_confidence = identity_confidence;
    ApplyAnchoredContact(out.roles[BodyJointRoleIndex(BodyJointRole::RightFoot)], evidence.right_foot);
    out.roles[BodyJointRoleIndex(BodyJointRole::Pelvis)].identity_confidence = identity_confidence;

    const UpperBodyDimensions upper_dims = EstimateUpperBodyDimensions(model, body_calibration);
    const UpperBodyFallback upper = EstimateUpperBodyFallback(state, model, body_calibration);
    const Vec3f torso_velocity = state.linear_velocity;
    const Vec3f arm_velocity = state.linear_velocity;
    const float upper_fallback_confidence = Clamp01(0.85f * state.confidence);

    const auto chest_measurement = ChestMeasurement(solver, upper.chest.position);
    float chest_scale = 1.0f;
    const Vec3f chest_candidate = UsableMeasurementWorld(chest_measurement)
        ? chest_measurement.world
        : upper.chest.position;
    const Vec3f constrained_chest = ConstrainChestToTorso(state, chest_candidate, upper, upper_dims, &chest_scale);
    out.roles[BodyJointRoleIndex(BodyJointRole::Chest)] = MakeConstrainedTelemetryJoint(
        BodyJointRole::Chest, chest_measurement, constrained_chest, torso_velocity,
        upper_fallback_confidence, chest_scale, out.valid, solver.degraded, identity_confidence,
        "kinematic_chest");

    const ShoulderPairEstimate shoulders = ConstrainShoulders(solver, upper, upper_dims, state);
    const ArmEstimate left_arm = ConstrainArm(
        solver, state, upper, upper_dims, true, shoulders.left, shoulders.left_scale);
    const ArmEstimate right_arm = ConstrainArm(
        solver, state, upper, upper_dims, false, shoulders.right, shoulders.right_scale);

    out.roles[BodyJointRoleIndex(BodyJointRole::Neck)] = MakeMeasuredJoint(
        BodyJointRole::Neck, out.joints, KeypointId::Neck, upper.neck, torso_velocity,
        solver, upper_fallback_confidence, out.valid, identity_confidence);
    out.roles[BodyJointRoleIndex(BodyJointRole::Head)] = MakeMeasuredJoint(
        BodyJointRole::Head, out.joints, KeypointId::HeadTop, upper.head, torso_velocity,
        solver, upper_fallback_confidence, out.valid, identity_confidence);
    out.roles[BodyJointRoleIndex(BodyJointRole::LeftShoulder)] = MakeConstrainedTelemetryJoint(
        BodyJointRole::LeftShoulder,
        JointTelemetryOrEmpty(solver, KeypointId::LeftShoulder),
        left_arm.shoulder,
        torso_velocity,
        upper_fallback_confidence,
        left_arm.shoulder_scale,
        out.valid,
        solver.degraded,
        identity_confidence,
        "kinematic_shoulder");
    out.roles[BodyJointRoleIndex(BodyJointRole::RightShoulder)] = MakeConstrainedTelemetryJoint(
        BodyJointRole::RightShoulder,
        JointTelemetryOrEmpty(solver, KeypointId::RightShoulder),
        right_arm.shoulder,
        torso_velocity,
        upper_fallback_confidence,
        right_arm.shoulder_scale,
        out.valid,
        solver.degraded,
        identity_confidence,
        "kinematic_shoulder");
    out.roles[BodyJointRoleIndex(BodyJointRole::LeftElbow)] = MakeConstrainedTelemetryJoint(
        BodyJointRole::LeftElbow,
        JointTelemetryOrEmpty(solver, KeypointId::LeftElbow),
        left_arm.elbow,
        arm_velocity,
        upper_fallback_confidence,
        left_arm.elbow_scale,
        out.valid,
        solver.degraded,
        identity_confidence,
        UsableMeasurementWorld(JointTelemetryOrEmpty(solver, KeypointId::LeftElbow)) ? "kinematic_elbow" : "ik_elbow_from_wrist");
    out.roles[BodyJointRoleIndex(BodyJointRole::RightElbow)] = MakeConstrainedTelemetryJoint(
        BodyJointRole::RightElbow,
        JointTelemetryOrEmpty(solver, KeypointId::RightElbow),
        right_arm.elbow,
        arm_velocity,
        upper_fallback_confidence,
        right_arm.elbow_scale,
        out.valid,
        solver.degraded,
        identity_confidence,
        UsableMeasurementWorld(JointTelemetryOrEmpty(solver, KeypointId::RightElbow)) ? "kinematic_elbow" : "ik_elbow_from_wrist");
    out.roles[BodyJointRoleIndex(BodyJointRole::LeftWrist)] = MakeConstrainedTelemetryJoint(
        BodyJointRole::LeftWrist,
        JointTelemetryOrEmpty(solver, KeypointId::LeftWrist),
        left_arm.wrist,
        arm_velocity,
        upper_fallback_confidence,
        left_arm.wrist_scale,
        out.valid,
        solver.degraded,
        identity_confidence,
        "kinematic_wrist");
    out.roles[BodyJointRoleIndex(BodyJointRole::RightWrist)] = MakeConstrainedTelemetryJoint(
        BodyJointRole::RightWrist,
        JointTelemetryOrEmpty(solver, KeypointId::RightWrist),
        right_arm.wrist,
        arm_velocity,
        upper_fallback_confidence,
        right_arm.wrist_scale,
        out.valid,
        solver.degraded,
        identity_confidence,
        "kinematic_wrist");

    const Vec3f leg_velocity = Scale(Add(state.linear_velocity, Add(state.left_foot_linear_velocity, state.right_foot_linear_velocity)), 1.0f / 3.0f);
    out.roles[BodyJointRoleIndex(BodyJointRole::LeftHip)] = MakeMeasuredJoint(
        BodyJointRole::LeftHip, out.joints, KeypointId::LeftHip, state.root.position, state.linear_velocity, solver, state.confidence, out.valid, identity_confidence);
    out.roles[BodyJointRoleIndex(BodyJointRole::RightHip)] = MakeMeasuredJoint(
        BodyJointRole::RightHip, out.joints, KeypointId::RightHip, state.root.position, state.linear_velocity, solver, state.confidence, out.valid, identity_confidence);
    out.roles[BodyJointRoleIndex(BodyJointRole::LeftKnee)] = MakeKinematicKneeJoint(
        BodyJointRole::LeftKnee, out.joints, KeypointId::LeftKnee, state.root.position, leg_velocity, solver, state.confidence, out.valid, identity_confidence);
    out.roles[BodyJointRoleIndex(BodyJointRole::RightKnee)] = MakeKinematicKneeJoint(
        BodyJointRole::RightKnee, out.joints, KeypointId::RightKnee, state.root.position, leg_velocity, solver, state.confidence, out.valid, identity_confidence);
    out.roles[BodyJointRoleIndex(BodyJointRole::LeftAnkle)] = MakeMeasuredJoint(
        BodyJointRole::LeftAnkle, out.joints, KeypointId::LeftAnkle, state.left_foot.position, state.left_foot_linear_velocity, solver, state.confidence, out.valid, identity_confidence);
    out.roles[BodyJointRoleIndex(BodyJointRole::RightAnkle)] = MakeMeasuredJoint(
        BodyJointRole::RightAnkle, out.joints, KeypointId::RightAnkle, state.right_foot.position, state.right_foot_linear_velocity, solver, state.confidence, out.valid, identity_confidence);

    auto left_toe = MakeMeasuredJoint(
        BodyJointRole::LeftToe, out.joints, KeypointId::LeftBigToe, ToePosition(out.joints, true, state.left_foot.position), state.left_foot_linear_velocity, solver, state.confidence, out.valid, identity_confidence);
    left_toe.position = ToePosition(out.joints, true, left_toe.position);
    out.roles[BodyJointRoleIndex(BodyJointRole::LeftToe)] = left_toe;
    auto right_toe = MakeMeasuredJoint(
        BodyJointRole::RightToe, out.joints, KeypointId::RightBigToe, ToePosition(out.joints, false, state.right_foot.position), state.right_foot_linear_velocity, solver, state.confidence, out.valid, identity_confidence);
    right_toe.position = ToePosition(out.joints, false, right_toe.position);
    out.roles[BodyJointRoleIndex(BodyJointRole::RightToe)] = right_toe;

    for (auto& role : out.roles) {
        if (solver.degraded) {
            role.evidence.degraded = true;
        }
    }

    float confidence_sum = 0.0f;
    int confidence_count = 0;
    for (const auto& role : out.roles) {
        const TrackingSignalKind signal_kind = EffectiveSignalKind(role.evidence);
        if (CountsAsPredictionVisibility(role.visibility)) {
            ++out.diagnostics.predicted_joint_count;
        }
        if (role.measured) {
            ++out.diagnostics.measured_role_count;
        }
        if (signal_kind == TrackingSignalKind::Anchored) {
            ++out.diagnostics.anchored_role_count;
        }
        if (signal_kind == TrackingSignalKind::Degraded) {
            ++out.diagnostics.degraded_role_count;
        }
        if (signal_kind == TrackingSignalKind::Manual) {
            ++out.diagnostics.manual_role_count;
        }
        if (signal_kind == TrackingSignalKind::StaleAged) {
            ++out.diagnostics.stale_aged_role_count;
        }
        if (signal_kind == TrackingSignalKind::Invalid) {
            ++out.diagnostics.invalid_role_count;
        }
        if (role.visibility == BodyJointVisibility::LowConfidence) {
            ++out.diagnostics.low_confidence_role_count;
        }
        if (role.valid) {
            confidence_sum += role.confidence;
            ++confidence_count;
        }
    }

    out.diagnostics.active = out.valid;
    out.diagnostics.degraded = solver.degraded || !out.valid;
    out.diagnostics.triangulation_active = solver.tracking_mode == TrackingMode::Stereo && solver.triangulated_count > 0;
    out.diagnostics.tracking_mode_is_monocular = solver.tracking_mode == TrackingMode::Monocular;
    out.diagnostics.stereo_fallback_active = false;
    out.diagnostics.monocular_fallback = out.diagnostics.stereo_fallback_active;
    out.diagnostics.left_right_identity_stable =
        solver.camera_a_identity_consistency >= tracking_constants::kIdentityStableThreshold &&
        (solver.camera_b_identity_consistency >= tracking_constants::kIdentityStableThreshold || solver.tracking_mode == TrackingMode::Monocular);
    out.diagnostics.left_right_identity_uncertain =
        solver.camera_a_identity_consistency > 0.0f && solver.camera_a_identity_consistency < tracking_constants::kIdentityUncertainThreshold;
    if (solver.tracking_mode == TrackingMode::Stereo && solver.camera_b_identity_consistency > 0.0f) {
        out.diagnostics.left_right_identity_uncertain =
            out.diagnostics.left_right_identity_uncertain || solver.camera_b_identity_consistency < tracking_constants::kIdentityUncertainThreshold;
    }
    out.diagnostics.occlusion_prediction_active =
        out.valid && (solver.degraded || out.diagnostics.predicted_joint_count > 0);
    out.diagnostics.contact_lock_active =
        IsActiveFootSupport(state.support.left_foot) || IsActiveFootSupport(state.support.right_foot);
    out.diagnostics.floor_support_active =
        state.support.left_foot.type == FootSupportType::FloorSupport ||
        state.support.right_foot.type == FootSupportType::FloorSupport;
    out.diagnostics.body_calibration_valid =
        body_calibration.standing_neutral_valid || body_calibration.quality.overall > 0.0f;
    out.diagnostics.triangulated_count = solver.triangulated_count;
    out.diagnostics.inferred_depth_count = solver.inferred_depth_count;
    out.diagnostics.mean_reprojection_error_px = solver.mean_reprojection_error_px;
    out.diagnostics.role_output_confidence = confidence_count > 0
        ? Clamp01(confidence_sum / static_cast<float>(confidence_count))
        : 0.0f;
    out.diagnostics.identity_confidence = identity_confidence;
    out.diagnostics.left_contact_lock_strength = evidence.left_foot.lock_strength;
    out.diagnostics.right_contact_lock_strength = evidence.right_foot.lock_strength;
    out.diagnostics.tracking_mode = ToString(solver.tracking_mode);
    out.diagnostics.depth_source = ToString(solver.depth_source);
    out.diagnostics.reason = solver.reason.empty()
        ? (out.valid ? "nominal" : "untracked")
        : solver.reason;
    out.diagnostics.latency_prediction_seconds = LatencyPredictionSeconds(dt_seconds, solver, state.confidence);
    out.diagnostics.latency_prediction_active = out.diagnostics.latency_prediction_seconds > 0.0;

    return out;
}

UnifiedBodyState BuildUnifiedBodyState(
    const LowerBodyState& state,
    const LowerBodyModel& model,
    const BodyStateSolverSnapshot& solver,
    const BodyCalibration& body_calibration,
    double dt_seconds,
    bool state_valid) {

    return BuildUnifiedBodyState(
        state,
        model,
        BuildBodyStateEvidence(solver, state),
        body_calibration,
        dt_seconds,
        state_valid);
}

} // namespace bt
