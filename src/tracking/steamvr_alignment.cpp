#include "tracking/steamvr_alignment.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace bt {
namespace {

constexpr float kMinBodyConfidence = 0.20f;
constexpr float kMinCalibrationBodyConfidence = 0.08f;
constexpr float kMaxMeanResidualM = 0.18f;
constexpr float kMaxFloorResidualM = 0.08f;
constexpr float kMinScaleRatio = 0.85f;
constexpr float kMaxScaleRatio = 1.15f;
constexpr float kMaxLeftRightMismatchM = 0.16f;
constexpr float kMaxYawDisagreementRad = 0.45f; // ~26 deg
constexpr float kMaxPerSampleResidualM = 0.20f;
constexpr double kMaxControllerPoseAgeSeconds = 0.25;

float Clamp01(float v) {
    if (!std::isfinite(v)) {
        return 0.0f;
    }
    return std::max(0.0f, std::min(1.0f, v));
}

Quatf YawQuat(float yaw_rad) {
    if (!std::isfinite(yaw_rad)) {
        return Quatf{0.0f, 0.0f, 0.0f, 1.0f};
    }
    const float half = 0.5f * yaw_rad;
    return Normalize(Quatf{0.0f, std::sin(half), 0.0f, std::cos(half)});
}

float XzAngle(const Vec3f& v) {
    return std::atan2(v.z, v.x);
}

float WrapAngleSigned(float a) {
    while (a > 3.14159265358979323846f) a -= 6.28318530717958647692f;
    while (a < -3.14159265358979323846f) a += 6.28318530717958647692f;
    return a;
}

Vec3f TransformPoint(const Vec3f& p, const Quatf& rotation, const Vec3f& offset, float scale) {
    return Add(Rotate(rotation, Scale(p, scale)), offset);
}

struct PlanarSimilarityFit {
    bool valid = false;
    float yaw_rad = 0.0f;
    float scale = 1.0f;
    Vec3f offset{};
    float rms_xz_residual_m = 0.0f;
};

PlanarSimilarityFit FitPlanarSimilarity3(
    const Vec3f& cam_a,
    const Vec3f& vr_a,
    const Vec3f& cam_b,
    const Vec3f& vr_b,
    const Vec3f& cam_c,
    const Vec3f& vr_c) {

    struct P2 { float x = 0.0f; float z = 0.0f; };
    const std::array<P2, 3> cam{{{cam_a.x, cam_a.z}, {cam_b.x, cam_b.z}, {cam_c.x, cam_c.z}}};
    const std::array<P2, 3> vr{{{vr_a.x, vr_a.z}, {vr_b.x, vr_b.z}, {vr_c.x, vr_c.z}}};
    P2 cam_cent{}, vr_cent{};
    for (int i = 0; i < 3; ++i) {
        cam_cent.x += cam[i].x; cam_cent.z += cam[i].z;
        vr_cent.x += vr[i].x; vr_cent.z += vr[i].z;
    }
    cam_cent.x /= 3.0f; cam_cent.z /= 3.0f;
    vr_cent.x /= 3.0f; vr_cent.z /= 3.0f;

    float dot = 0.0f;
    float cross = 0.0f;
    float cam_energy = 0.0f;
    for (int i = 0; i < 3; ++i) {
        const float cx = cam[i].x - cam_cent.x;
        const float cz = cam[i].z - cam_cent.z;
        const float vx = vr[i].x - vr_cent.x;
        const float vz = vr[i].z - vr_cent.z;
        dot += cx * vx + cz * vz;
        cross += cx * vz - cz * vx;
        cam_energy += cx * cx + cz * cz;
    }

    PlanarSimilarityFit fit;
    if (!std::isfinite(dot) || !std::isfinite(cross) || !std::isfinite(cam_energy) || cam_energy < 1.0e-5f) {
        return fit;
    }
    fit.yaw_rad = WrapAngleSigned(std::atan2(cross, dot));
    fit.scale = std::sqrt(dot * dot + cross * cross) / cam_energy;
    if (!std::isfinite(fit.scale) || fit.scale <= 0.0f || !std::isfinite(fit.yaw_rad)) {
        return fit;
    }
    fit.offset.y = vr_c.y - cam_c.y * fit.scale;
    const Quatf yaw = YawQuat(fit.yaw_rad);
    const Vec3f rotated_centroid = Rotate(yaw, Scale(Vec3f{cam_cent.x, 0.0f, cam_cent.z}, fit.scale));
    fit.offset.x = vr_cent.x - rotated_centroid.x;
    fit.offset.z = vr_cent.z - rotated_centroid.z;

    float residual_sum_sq = 0.0f;
    for (int i = 0; i < 3; ++i) {
        const Vec3f pred = TransformPoint(Vec3f{cam[i].x, 0.0f, cam[i].z}, yaw, fit.offset, fit.scale);
        const float dx = pred.x - vr[i].x;
        const float dz = pred.z - vr[i].z;
        residual_sum_sq += dx * dx + dz * dz;
    }
    fit.rms_xz_residual_m = std::sqrt(residual_sum_sq / 3.0f);
    fit.valid = IsFinite(fit.offset) && std::isfinite(fit.rms_xz_residual_m);
    return fit;
}

const BodyStateJoint& BodyRole(const UnifiedBodyState& body_state, BodyJointRole role) {
    return body_state.roles[BodyJointRoleIndex(role)];
}

bool BodyRoleUsableForCalibration(const BodyStateJoint& joint) {
    if (!joint.valid || !IsFinite(joint.position) || joint.confidence <= 0.0f) {
        return false;
    }
    if (joint.confidence >= kMinBodyConfidence) {
        return true;
    }
    const bool estimated_but_owned =
        joint.visibility == BodyJointVisibility::Predicted ||
        joint.visibility == BodyJointVisibility::Anchored ||
        joint.predicted ||
        joint.evidence.anchor_held ||
        joint.evidence.valid;
    return estimated_but_owned && joint.confidence >= kMinCalibrationBodyConfidence;
}

BodyJointRole BodyRoleForCalibrationLandmark(SteamVrAlignmentLandmark landmark) {
    switch (landmark) {
    case SteamVrAlignmentLandmark::LeftFoot:
        // The wizard asks the user to place the controller on the ankle; sample the
        // same body landmark instead of silently calibrating against foot centre.
        return BodyJointRole::LeftAnkle;
    case SteamVrAlignmentLandmark::RightFoot:
        return BodyJointRole::RightAnkle;
    case SteamVrAlignmentLandmark::Pelvis:
        return BodyJointRole::Pelvis;
    case SteamVrAlignmentLandmark::Chest:
        return BodyJointRole::Chest;
    case SteamVrAlignmentLandmark::LeftElbow:
        return BodyJointRole::LeftElbow;
    case SteamVrAlignmentLandmark::RightElbow:
        return BodyJointRole::RightElbow;
    case SteamVrAlignmentLandmark::LeftKnee:
        return BodyJointRole::LeftKnee;
    case SteamVrAlignmentLandmark::RightKnee:
        return BodyJointRole::RightKnee;
    default:
        return BodyJointRole::Pelvis;
    }
}

Vec3f CameraLandmarkFor(SteamVrAlignmentLandmark landmark, const UnifiedBodyState& body_state, const CalibrationBundle& calibration) {
    switch (landmark) {
    case SteamVrAlignmentLandmark::LeftFoot:
    case SteamVrAlignmentLandmark::RightFoot:
        return BodyRole(body_state, BodyRoleForCalibrationLandmark(landmark)).position;
    case SteamVrAlignmentLandmark::Pelvis:
        return BodyRole(body_state, BodyJointRole::Pelvis).position;
    case SteamVrAlignmentLandmark::Chest:
    case SteamVrAlignmentLandmark::LeftElbow:
    case SteamVrAlignmentLandmark::RightElbow:
        return BodyRole(body_state, BodyRoleForCalibrationLandmark(landmark)).position;
    case SteamVrAlignmentLandmark::Floor:
        if (FloorPlaneUsable(calibration.floor)) {
            const float denom = Dot(calibration.floor.normal, calibration.floor.normal);
            return denom > 1e-6f ? Scale(calibration.floor.normal, calibration.floor.distance / denom) : Vec3f{};
        }
        // SteamVR floor sampling is itself the floor calibration for tracker-space
        // vertical scope. Monocular mode often has no stereo floor plane; use the
        // solver's conventional y=0 floor instead of rejecting the sample forever.
        return Vec3f{0.0f, 0.0f, 0.0f};
    case SteamVrAlignmentLandmark::Forward:
        return Rotate(body_state.lower_body.root.orientation, Vec3f{0.0f, 0.0f, 1.0f});
    case SteamVrAlignmentLandmark::LeftKnee:
        return BodyRole(body_state, BodyJointRole::LeftKnee).position;
    case SteamVrAlignmentLandmark::RightKnee:
        return BodyRole(body_state, BodyJointRole::RightKnee).position;
    default:
        return {};
    }
}

bool LandmarkBodyUsable(SteamVrAlignmentLandmark landmark, const UnifiedBodyState& body_state, const CalibrationBundle&) {
    switch (landmark) {
    case SteamVrAlignmentLandmark::LeftFoot:
    case SteamVrAlignmentLandmark::RightFoot:
    case SteamVrAlignmentLandmark::Pelvis:
    case SteamVrAlignmentLandmark::Chest:
    case SteamVrAlignmentLandmark::LeftElbow:
    case SteamVrAlignmentLandmark::RightElbow:
    case SteamVrAlignmentLandmark::LeftKnee:
    case SteamVrAlignmentLandmark::RightKnee:
        return BodyRoleUsableForCalibration(BodyRole(body_state, BodyRoleForCalibrationLandmark(landmark)));
    case SteamVrAlignmentLandmark::Floor:
        return true;
    case SteamVrAlignmentLandmark::Forward:
        return body_state.valid || body_state.diagnostics.role_output_confidence >= kMinCalibrationBodyConfidence;
    default:
        return false;
    }
}

float LandmarkConfidence(SteamVrAlignmentLandmark landmark, const UnifiedBodyState& body_state, const CalibrationBundle&) {
    switch (landmark) {
    case SteamVrAlignmentLandmark::LeftFoot:
    case SteamVrAlignmentLandmark::RightFoot:
    case SteamVrAlignmentLandmark::Pelvis:
    case SteamVrAlignmentLandmark::Chest:
    case SteamVrAlignmentLandmark::LeftElbow:
    case SteamVrAlignmentLandmark::RightElbow:
        return BodyRole(body_state, BodyRoleForCalibrationLandmark(landmark)).confidence;
    case SteamVrAlignmentLandmark::Floor:
        return 1.0f;
    case SteamVrAlignmentLandmark::Forward:
        return body_state.diagnostics.role_output_confidence;
    case SteamVrAlignmentLandmark::LeftKnee:
    case SteamVrAlignmentLandmark::RightKnee:
        return BodyRole(body_state, BodyRoleForCalibrationLandmark(landmark)).confidence;
    default:
        return 0.0f;
    }
}

std::string SignatureFloat(float v) {
    if (!std::isfinite(v)) {
        return "nan";
    }
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3) << v;
    return oss.str();
}

std::string RoleSignature(const Vec3f& v) {
    return SignatureFloat(v.x) + "," + SignatureFloat(v.y) + "," + SignatureFloat(v.z);
}

template <std::size_t N>
std::string ArraySignature(const std::array<float, N>& values) {
    std::string out;
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (i > 0) {
            out += ",";
        }
        out += SignatureFloat(values[i]);
    }
    return out;
}

std::string FloorGeometrySourceForSignature(const FloorGeometryCalibration& g) {
    if (!g.valid) {
        return "nothing";
    }
    return g.source.empty() || g.source == "unknown" ? "legacy_json" : g.source;
}

const SteamVrAlignmentSample& RequiredSample(const SteamVrAlignmentSession& session, SteamVrAlignmentLandmark landmark) {
    return session.samples[LandmarkSlotIndex(landmark)];
}

void SetReason(SteamVrAlignmentSolveResult& out, SteamVrAlignmentReason code) {
    out.reason_code = code;
    out.reason = ToString(code);
}

bool BodyCalibrationLooksValid(const BodyCalibration& body) {
    return body.standing_neutral_valid || body.quality.overall > 0.0f;
}

bool ReasonLooksCompileDisabled(const std::string& reason) {
    return reason.find("OpenVR support was not built") != std::string::npos ||
        reason.find("provider_compile_disabled") != std::string::npos;
}

double MaxValidControllerPoseAgeSeconds(const SteamVrPoseSnapshot& snapshot) {
    double age = 0.0;
    bool any_valid = false;
    if (snapshot.left.valid && std::isfinite(snapshot.left.pose_age_seconds)) {
        age = std::max(age, snapshot.left.pose_age_seconds);
        any_valid = true;
    }
    if (snapshot.right.valid && std::isfinite(snapshot.right.pose_age_seconds)) {
        age = std::max(age, snapshot.right.pose_age_seconds);
        any_valid = true;
    }
    return any_valid ? age : 0.0;
}

SteamVrAlignmentReason ProviderStaleReason(const SteamVrPoseSnapshot& snapshot) {
    if (!snapshot.available) {
        return ReasonLooksCompileDisabled(snapshot.reason)
            ? SteamVrAlignmentReason::ProviderCompileDisabled
            : SteamVrAlignmentReason::ProviderUnavailable;
    }
    if (!snapshot.left.valid && !snapshot.right.valid) {
        return SteamVrAlignmentReason::ControllersMissing;
    }
    const double age = MaxValidControllerPoseAgeSeconds(snapshot);
    if (age > kMaxControllerPoseAgeSeconds) {
        return SteamVrAlignmentReason::ControllerPoseInvalid;
    }
    return SteamVrAlignmentReason::None;
}

bool ControllerAlignmentFresh(const SteamVrPoseSnapshot& snapshot) {
    return ProviderStaleReason(snapshot) == SteamVrAlignmentReason::None;
}

} // namespace

SteamVrAlignmentSession StartSteamVrAlignmentSession(const SteamVrPoseSnapshot& steamvr) {
    SteamVrAlignmentSession session;
    if (!steamvr.available) {
        session.active = false;
        session.status = "failed";
        session.reason_code = SteamVrAlignmentReason::SteamVrUnavailable;
        session.reason = steamvr.reason.empty() ? std::string(ToString(session.reason_code)) : steamvr.reason;
        return session;
    }
    if (!steamvr.left.valid && !steamvr.right.valid) {
        session.active = false;
        session.status = "failed";
        session.reason_code = SteamVrAlignmentReason::ControllersMissing;
        session.reason = "controllers missing or untracked";
        return session;
    }
    session.active = true;
    session.status = "sampling";
    session.reason_code = SteamVrAlignmentReason::Sampling;
    session.reason = "ready";
    return session;
}

SteamVrAlignmentSample CaptureSteamVrAlignmentSample(
    SteamVrAlignmentLandmark landmark,
    const SteamVrControllerPose& controller,
    const UnifiedBodyState& body_state,
    const CalibrationBundle& calibration,
    double timestamp_seconds) {

    SteamVrAlignmentSample sample;
    sample.landmark = landmark;
    sample.controller = controller.role;
    sample.steamvr_pose = controller.pose;
    sample.timestamp_seconds = timestamp_seconds;
    sample.pose_age_seconds = controller.pose_age_seconds;
    sample.controller_valid = controller.valid && IsFinite(controller.pose.position);
    sample.body_state_valid = body_state.valid || landmark == SteamVrAlignmentLandmark::Floor ||
        LandmarkBodyUsable(landmark, body_state, calibration);
    if (!sample.controller_valid) {
        sample.reason_code = SteamVrAlignmentReason::ControllerPoseInvalid;
        sample.reason = controller.reason.empty() ? std::string(ToString(sample.reason_code)) : controller.reason;
        return sample;
    }
    if (!LandmarkBodyUsable(landmark, body_state, calibration)) {
        sample.reason_code = (landmark == SteamVrAlignmentLandmark::Floor)
            ? SteamVrAlignmentReason::FloorCalibrationMissing
            : SteamVrAlignmentReason::BodyLandmarkMissing;
        sample.reason = ToString(sample.reason_code);
        return sample;
    }
    sample.camera_landmark = CameraLandmarkFor(landmark, body_state, calibration);
    if (!IsFinite(sample.camera_landmark)) {
        sample.reason_code = SteamVrAlignmentReason::SampleNonfinite;
        sample.reason = ToString(sample.reason_code);
        return sample;
    }
    sample.confidence = Clamp01(LandmarkConfidence(landmark, body_state, calibration));
    const float min_confidence = landmark == SteamVrAlignmentLandmark::Floor
        ? 0.0f
        : kMinCalibrationBodyConfidence;
    const bool conf_ok = sample.confidence >= min_confidence;
    if (!conf_ok) {
        sample.reason_code = SteamVrAlignmentReason::SampleConfidenceLow;
        sample.reason = ToString(sample.reason_code);
        return sample;
    }
    sample.accepted = true;
    sample.reason_code = SteamVrAlignmentReason::Accepted;
    sample.reason = ToString(sample.reason_code);
    return sample;
}

void StoreSteamVrAlignmentSample(SteamVrAlignmentSession& session, const SteamVrAlignmentSample& sample) {
    const std::size_t slot = LandmarkSlotIndex(sample.landmark);
    if (sample.accepted || !session.samples[slot].accepted) {
        session.samples[slot] = sample;
    }
    session.accepted_sample_count = 0;
    for (const auto& stored : session.samples) {
        if (stored.accepted) {
            ++session.accepted_sample_count;
        }
    }
    if (session.active) {
        session.status = "sampling";
        session.reason_code = SteamVrAlignmentReason::Sampling;
    }
    if (!sample.reason.empty()) {
        session.reason = sample.reason;
    }
}

void RedoSteamVrAlignmentSample(SteamVrAlignmentSession& session, SteamVrAlignmentLandmark landmark) {
    const std::size_t slot = LandmarkSlotIndex(landmark);
    if (slot >= session.samples.size()) {
        return;
    }
    session.samples[slot] = SteamVrAlignmentSample{};
    session.samples[slot].landmark = landmark;
    session.samples[slot].reason_code = SteamVrAlignmentReason::Idle;
    session.samples[slot].reason = "not_sampled";
    session.accepted_sample_count = 0;
    for (const auto& stored : session.samples) {
        if (stored.accepted) {
            ++session.accepted_sample_count;
        }
    }
    if (session.active) {
        session.status = "sampling";
        session.reason_code = SteamVrAlignmentReason::Sampling;
        session.reason = "ready";
    }
}

SteamVrAlignmentSolveResult SolveSteamVrAlignment(
    const SteamVrAlignmentSession& session,
    const UnifiedBodyState& body_state,
    const CalibrationBundle& calibration) {

    SteamVrAlignmentSolveResult out;
    out.tracker_space_rotation = Quatf{0.0f, 0.0f, 0.0f, 1.0f};
    out.body_signature = SteamVrBodyCalibrationSignature(calibration.body);
    out.floor_signature = SteamVrFloorCalibrationSignature(calibration);

    // Count required samples accepted up-front so reasons reflect input completeness.
    int required_accepted = 0;
    int total_accepted = 0;
    for (const auto& s : session.samples) {
        if (s.accepted) {
            ++total_accepted;
            if (LandmarkIsRequired(s.landmark)) {
                ++required_accepted;
            }
        }
    }
    out.required_samples_present = required_accepted;
    out.total_samples_accepted = total_accepted;

    if (!session.active) {
        SetReason(out, session.reason_code != SteamVrAlignmentReason::None
            ? session.reason_code
            : SteamVrAlignmentReason::Failed);
        if (!session.reason.empty()) {
            out.reason = session.reason;
        }
        return out;
    }
    const auto& left = RequiredSample(session, SteamVrAlignmentLandmark::LeftFoot);
    const auto& right = RequiredSample(session, SteamVrAlignmentLandmark::RightFoot);
    const auto& pelvis = RequiredSample(session, SteamVrAlignmentLandmark::Pelvis);
    const auto& floor = RequiredSample(session, SteamVrAlignmentLandmark::Floor);
    if (!left.accepted || !right.accepted || !pelvis.accepted || !floor.accepted) {
        SetReason(out, SteamVrAlignmentReason::NotEnoughSamples);
        return out;
    }

    // Fit camera-to-VR similarity from the three body landmarks that actually
    // define the user's tracker scope. The old solve used only the ankle-to-ankle
    // baseline for yaw/scale and used pelvis only as a translation anchor; that
    // made small foot-baseline mistakes enlarge the whole play-space transform.
    const Vec3f cam_lr = Sub(right.camera_landmark, left.camera_landmark);
    const Vec3f vr_lr = Sub(right.steamvr_pose.position, left.steamvr_pose.position);
    const float cam_lr_len = std::sqrt(cam_lr.x * cam_lr.x + cam_lr.z * cam_lr.z);
    const float vr_lr_len = std::sqrt(vr_lr.x * vr_lr.x + vr_lr.z * vr_lr.z);
    if (cam_lr_len < 0.05f || vr_lr_len < 0.05f) {
        SetReason(out, SteamVrAlignmentReason::YawAmbiguous);
        return out;
    }
    const auto fit = FitPlanarSimilarity3(
        left.camera_landmark, left.steamvr_pose.position,
        right.camera_landmark, right.steamvr_pose.position,
        pelvis.camera_landmark, pelvis.steamvr_pose.position);
    if (!fit.valid) {
        SetReason(out, SteamVrAlignmentReason::YawAmbiguous);
        return out;
    }
    out.scale_ratio = fit.scale;
    out.scale_mismatch = std::abs(out.scale_ratio - 1.0f);
    if (!std::isfinite(out.scale_ratio) ||
        out.scale_ratio < kMinScaleRatio || out.scale_ratio > kMaxScaleRatio) {
        SetReason(out, SteamVrAlignmentReason::ScaleMismatch);
        return out;
    }

    const float yaw_from_feet = WrapAngleSigned(XzAngle(vr_lr) - XzAngle(cam_lr));
    out.yaw_offset_rad = fit.yaw_rad;
    out.yaw_disagreement_rad = std::abs(WrapAngleSigned(fit.yaw_rad - yaw_from_feet));

    const auto& fwd = RequiredSample(session, SteamVrAlignmentLandmark::Forward);
    if (fwd.accepted) {
        // Forward sample provides a yaw direction; controller pose forward axis is z column.
        const Vec3f cam_fwd = fwd.camera_landmark;
        const Vec3f vr_fwd = Rotate(fwd.steamvr_pose.orientation, Vec3f{0.0f, 0.0f, 1.0f});
        const float cam_fwd_len = std::sqrt(cam_fwd.x * cam_fwd.x + cam_fwd.z * cam_fwd.z);
        const float vr_fwd_len = std::sqrt(vr_fwd.x * vr_fwd.x + vr_fwd.z * vr_fwd.z);
        if (cam_fwd_len > 0.10f && vr_fwd_len > 0.10f) {
            const float yaw_from_fwd = WrapAngleSigned(XzAngle(vr_fwd) - XzAngle(cam_fwd));
            const float disagreement = std::abs(WrapAngleSigned(yaw_from_fwd - yaw_from_feet));
            out.yaw_disagreement_rad = disagreement;
            if (disagreement <= kMaxYawDisagreementRad) {
                // Average the two yaw references when they agree.
                out.yaw_offset_rad = WrapAngleSigned(yaw_from_feet + 0.5f * WrapAngleSigned(yaw_from_fwd - yaw_from_feet));
            }
        }
    }

    out.tracker_space_rotation = YawQuat(out.yaw_offset_rad);
    out.tracker_space_scale = out.scale_ratio;
    out.tracker_space_position_offset = fit.offset;
    out.tracker_space_position_offset.y = pelvis.steamvr_pose.position.y - pelvis.camera_landmark.y * out.tracker_space_scale;

    // Vertical lock from the required floor sample.
    const Vec3f transformed_floor = TransformPoint(floor.camera_landmark, out.tracker_space_rotation, out.tracker_space_position_offset, out.tracker_space_scale);
    out.tracker_space_position_offset.y += floor.steamvr_pose.position.y - transformed_floor.y;

    auto predict = [&](const Vec3f& cam) {
        return TransformPoint(cam, out.tracker_space_rotation, out.tracker_space_position_offset, out.tracker_space_scale);
    };

    const Vec3f pred_left = predict(left.camera_landmark);
    const Vec3f pred_right = predict(right.camera_landmark);
    const Vec3f pred_pelvis = predict(pelvis.camera_landmark);
    const Vec3f pred_floor = predict(floor.camera_landmark);

    const float left_residual = Distance(pred_left, left.steamvr_pose.position);
    const float right_residual = Distance(pred_right, right.steamvr_pose.position);
    const float pelvis_residual = Distance(pred_pelvis, pelvis.steamvr_pose.position);
    out.floor_residual_m = std::abs(pred_floor.y - floor.steamvr_pose.position.y);

    // Residuals shape confidence. They do not get a second veto after the solve.
    const float lr_diff = std::abs(left_residual - right_residual);
    const float lr_penalty = lr_diff > kMaxLeftRightMismatchM ? (lr_diff - kMaxLeftRightMismatchM) : 0.0f;
    const float sample_penalty = std::max({
        std::max(0.0f, left_residual - kMaxPerSampleResidualM),
        std::max(0.0f, right_residual - kMaxPerSampleResidualM),
        std::max(0.0f, pelvis_residual - kMaxPerSampleResidualM)});
    const float floor_penalty = std::max(0.0f, out.floor_residual_m - kMaxFloorResidualM);
    const float residual_sum = left_residual + right_residual + pelvis_residual + out.floor_residual_m;
    const float residual_count = 4.0f;
    out.residual_m = residual_sum / residual_count + lr_penalty + sample_penalty + floor_penalty;

    // Required role offsets. The left/right "foot" wizard samples are ankle body
    // landmarks, not foot-center tracker attachments, so they constrain the global
    // SteamVR transform but do not add an ankle-shaped offset to the VRChat foot
    // tracker roles.
    out.role_offsets[TrackerRoleIndex(TrackerRole::Pelvis)] = Sub(pelvis.steamvr_pose.position, pred_pelvis);
    out.role_offsets[TrackerRoleIndex(TrackerRole::LeftFoot)] = Vec3f{};
    out.role_offsets[TrackerRoleIndex(TrackerRole::RightFoot)] = Vec3f{};
    out.role_offsets_present[TrackerRoleIndex(TrackerRole::Pelvis)] = true;
    out.role_offsets_present[TrackerRoleIndex(TrackerRole::LeftFoot)] = true;
    out.role_offsets_present[TrackerRoleIndex(TrackerRole::RightFoot)] = true;

    // Optional role offsets. Missing optional samples derive a zero role offset from
    // the solved body landmark; explicit controller samples only refine tracker attachment
    // deltas and cannot fail the whole alignment.
    auto try_optional_role = [&](SteamVrAlignmentLandmark lm, BodyJointRole body_role, TrackerRole tracker_role) {
        const auto& sample = RequiredSample(session, lm);
        if (sample.accepted) {
            const Vec3f pred = predict(sample.camera_landmark);
            const Vec3f offset = Sub(sample.steamvr_pose.position, pred);
            const float r = Distance(pred, sample.steamvr_pose.position);
            if (IsFinite(offset) && r <= kMaxPerSampleResidualM) {
                out.role_offsets[TrackerRoleIndex(tracker_role)] = offset;
                out.role_offsets_present[TrackerRoleIndex(tracker_role)] = true;
                return;
            }
        }
        const auto& joint = BodyRole(body_state, body_role);
        if (BodyRoleUsableForCalibration(joint)) {
            const Vec3f pred = predict(joint.position);
            if (IsFinite(pred)) {
                out.role_offsets[TrackerRoleIndex(tracker_role)] = Vec3f{};
                out.role_offsets_present[TrackerRoleIndex(tracker_role)] = true;
            }
        }
    };
    try_optional_role(SteamVrAlignmentLandmark::Chest, BodyJointRole::Chest, TrackerRole::Chest);
    try_optional_role(SteamVrAlignmentLandmark::LeftElbow, BodyJointRole::LeftElbow, TrackerRole::LeftElbow);
    try_optional_role(SteamVrAlignmentLandmark::RightElbow, BodyJointRole::RightElbow, TrackerRole::RightElbow);
    try_optional_role(SteamVrAlignmentLandmark::LeftKnee, BodyJointRole::LeftKnee, TrackerRole::LeftKnee);
    try_optional_role(SteamVrAlignmentLandmark::RightKnee, BodyJointRole::RightKnee, TrackerRole::RightKnee);

    float confidence = Clamp01(1.0f - out.residual_m / kMaxMeanResidualM);
    if (out.yaw_disagreement_rad > kMaxYawDisagreementRad) {
        confidence *= 0.75f;
    }
    if (!BodyCalibrationLooksValid(calibration.body)) {
        confidence *= 0.90f;
    }
    if (!FloorPlaneUsable(calibration.floor)) {
        confidence *= 0.95f;
    }
    out.confidence = Clamp01(confidence);
    out.valid = out.confidence >= 0.20f;
    if (out.valid) {
        SetReason(out, out.confidence >= 0.65f ? SteamVrAlignmentReason::Valid : SteamVrAlignmentReason::Weak);
        out.status = (out.confidence >= 0.65f) ? "valid" : "weak";
    } else {
        SetReason(out, SteamVrAlignmentReason::TransformResidualTooHigh);
        out.status = "failed";
    }
    return out;
}

bool ManualTrackerSpaceFallbackAvailable(const OscConfig& config) {
    const auto& q = config.manual_tracker_space_rotation;
    const float len2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (!config.manual_tracker_space_transform_valid ||
        !IsFinite(config.manual_tracker_space_position_offset) ||
        !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w) ||
        !std::isfinite(len2) || len2 < 1e-12f ||
        !std::isfinite(config.manual_tracker_space_scale) ||
        config.manual_tracker_space_scale <= 0.0f) {
        return false;
    }
    return std::all_of(config.manual_tracker_space_role_offsets.begin(), config.manual_tracker_space_role_offsets.end(), [](const Vec3f& offset) {
        return IsFinite(offset);
    });
}

void StoreActiveTrackerSpaceAsManualFallback(OscConfig& config) {
    if (!config.tracker_space_transform_valid ||
        !IsFinite(config.tracker_space_position_offset) ||
        !std::isfinite(config.tracker_space_rotation.x) ||
        !std::isfinite(config.tracker_space_rotation.y) ||
        !std::isfinite(config.tracker_space_rotation.z) ||
        !std::isfinite(config.tracker_space_rotation.w) ||
        !std::isfinite(config.tracker_space_scale) ||
        config.tracker_space_scale <= 0.0f) {
        return;
    }
    config.manual_tracker_space_transform_valid = true;
    config.manual_tracker_space_position_offset = config.tracker_space_position_offset;
    config.manual_tracker_space_rotation = config.tracker_space_rotation;
    config.manual_tracker_space_scale = config.tracker_space_scale;
    config.manual_tracker_space_role_offsets = config.tracker_space_role_offsets;
    config.manual_tracker_space_source = "manual";
}

void ActivateManualTrackerSpaceFallback(OscConfig& config) {
    if (!ManualTrackerSpaceFallbackAvailable(config)) {
        return;
    }
    config.tracker_space_transform_valid = true;
    config.tracker_space_position_offset = config.manual_tracker_space_position_offset;
    config.tracker_space_rotation = config.manual_tracker_space_rotation;
    config.tracker_space_scale = config.manual_tracker_space_scale;
    config.tracker_space_role_offsets = config.manual_tracker_space_role_offsets;
    config.tracker_space_source = config.manual_tracker_space_source.empty()
        ? "manual"
        : config.manual_tracker_space_source;
}

void ApplySteamVrAlignmentToOscConfig(OscConfig& config, const SteamVrAlignmentSolveResult& result) {
    if (result.valid) {
        StoreActiveTrackerSpaceAsManualFallback(config);
        config.tracker_space_transform_valid = true;
        config.tracker_space_position_offset = result.tracker_space_position_offset;
        config.tracker_space_rotation = result.tracker_space_rotation;
        config.tracker_space_scale = result.tracker_space_scale;
        config.tracker_space_role_offsets = result.role_offsets;
        config.tracker_space_source = "steamvr_controller_alignment";
        if (!ManualTrackerSpaceFallbackAvailable(config)) {
            StoreActiveTrackerSpaceAsManualFallback(config);
        }
    }
    config.steamvr_alignment_status = result.status;
    config.steamvr_alignment_reason = result.reason;
    config.steamvr_alignment_confidence = result.confidence;
    config.steamvr_alignment_residual_m = result.residual_m;
    config.steamvr_floor_residual_m = result.floor_residual_m;
    config.steamvr_yaw_offset_rad = result.yaw_offset_rad;
    config.steamvr_scale_ratio = result.scale_ratio;
    config.steamvr_alignment_body_signature = result.body_signature;
    config.steamvr_alignment_floor_signature = result.floor_signature;
}

void ClearSteamVrAlignmentFromOscConfig(OscConfig& config) {
    if (config.tracker_space_source == "steamvr_controller_alignment" ||
        config.tracker_space_source == "steamvr_controller_alignment_stale") {
        if (config.tracker_space_transform_valid) {
            StoreActiveTrackerSpaceAsManualFallback(config);
        }
        if (ManualTrackerSpaceFallbackAvailable(config)) {
            ActivateManualTrackerSpaceFallback(config);
        } else {
            config.tracker_space_source = "manual";
        }
    }
    config.steamvr_alignment_status = "idle";
    config.steamvr_alignment_reason.clear();
    config.steamvr_alignment_confidence = 0.0f;
    config.steamvr_alignment_residual_m = 0.0f;
    config.steamvr_floor_residual_m = 0.0f;
    config.steamvr_yaw_offset_rad = 0.0f;
    config.steamvr_scale_ratio = 1.0f;
    config.steamvr_alignment_body_signature.clear();
    config.steamvr_alignment_floor_signature.clear();
}

std::string SteamVrBodyCalibrationSignature(const BodyCalibration& body) {
    return std::string(body.standing_neutral_valid ? "standing:1:" : "standing:0:") +
        SignatureFloat(body.pelvis_width) + ":" +
        SignatureFloat(body.left_femur) + ":" +
        SignatureFloat(body.right_femur) + ":" +
        SignatureFloat(body.left_tibia) + ":" +
        SignatureFloat(body.right_tibia) + ":" +
        SignatureFloat(body.left_foot_length) + ":" +
        SignatureFloat(body.right_foot_length) + ":" +
        SignatureFloat(body.quality.overall) + ":" +
        std::to_string(body.quality.sample_count);
}

std::string SteamVrFloorCalibrationSignature(const CalibrationBundle& calibration) {
    const auto& g = calibration.floor_geometry;
    return std::string(calibration.floor.valid ? "floor:1:" : "floor:0:") +
        RoleSignature(calibration.floor.normal) + ":" +
        SignatureFloat(calibration.floor.distance) + ":" +
        std::string(g.valid ? "geom:1:" : "geom:0:") +
        FloorGeometrySourceForSignature(g) + ":" +
        g.floor_type + ":" +
        std::to_string(g.family_count) + ":" +
        SignatureFloat(g.floor_plane_confidence) + ":" +
        std::string(g.homography_valid ? "H1:" : "H0:") +
        ArraySignature(g.floor_from_image) + ":" +
        ArraySignature(g.image_from_floor) + ":" +
        SignatureFloat(g.homography_reprojection_error_px) + ":" +
        std::string(g.camera_orientation_valid ? "O1:" : "O0:") +
        SignatureFloat(g.camera_pitch_rad) + ":" +
        SignatureFloat(g.camera_roll_rad) + ":" +
        SignatureFloat(g.camera_yaw_rad) + ":" +
        SignatureFloat(g.camera_orientation_confidence) + ":" +
        std::string(g.distortion.valid ? "D1:" : "D0:") +
        SignatureFloat(g.distortion.radial_k1) + ":" +
        SignatureFloat(g.distortion.radial_k2) + ":" +
        SignatureFloat(g.distortion.tangential_p1) + ":" +
        SignatureFloat(g.distortion.tangential_p2) + ":" +
        SignatureFloat(g.distortion.confidence) + ":" +
        std::string(g.multi_camera_alignment_valid ? "M1:" : "M0:") +
        SignatureFloat(g.multi_camera_yaw_delta_rad) + ":" +
        SignatureFloat(g.multi_camera_pitch_delta_rad) + ":" +
        SignatureFloat(g.multi_camera_roll_delta_rad) + ":" +
        SignatureFloat(g.multi_camera_height_delta_m) + ":" +
        SignatureFloat(g.multi_camera_scale_ratio) + ":" +
        std::string(g.shared_floor_frame_valid ? "S1:" : "S0:") +
        ArraySignature(g.shared_floor_transform);
}

bool SteamVrAlignmentStale(const OscConfig& config, const CalibrationBundle& calibration) {
    if (config.tracker_space_source != "steamvr_controller_alignment") {
        return false;
    }
    if (!config.tracker_space_transform_valid) {
        return true;
    }
    return config.steamvr_alignment_body_signature != SteamVrBodyCalibrationSignature(calibration.body) ||
        config.steamvr_alignment_floor_signature != SteamVrFloorCalibrationSignature(calibration);
}

LatencyProbeSession StartLatencyProbe(const SteamVrPoseSnapshot& steamvr) {
    LatencyProbeSession s;
    if (!steamvr.available) {
        s.status = LatencyProbeStatus::Unavailable;
        s.reason = "provider_unavailable";
        return s;
    }
    s.status = LatencyProbeStatus::Collecting;
    s.reason = "collecting";
    return s;
}

void RecordLatencyProbeSample(LatencyProbeSession& session, const LatencyProbeSample& s) {
    if (session.status != LatencyProbeStatus::Collecting) {
        return;
    }
    if (session.samples.size() >= 256) {
        // Cap to bound memory.
        return;
    }
    session.samples.push_back(s);
    session.sample_count = static_cast<int>(session.samples.size());
}

void FinishLatencyProbe(LatencyProbeSession& session) {
    if (session.status != LatencyProbeStatus::Collecting) {
        if (session.status == LatencyProbeStatus::Unavailable) {
            session.reason = "provider_unavailable";
        }
        return;
    }
    if (session.samples.size() < 4) {
        session.status = LatencyProbeStatus::Failed;
        session.reason = "insufficient_motion";
        return;
    }
    // Estimate latency by averaging timestamp deltas of correlated motion samples.
    double sum = 0.0;
    int correlated = 0;
    Vec3f prev_c{}, prev_b{};
    bool have_prev = false;
    for (const auto& s : session.samples) {
        if (have_prev) {
            const float dc = Distance(s.controller_position, prev_c);
            const float db = Distance(s.body_position, prev_b);
            if (dc > 0.005f && db > 0.005f) {
                sum += (s.body_timestamp_seconds - s.controller_timestamp_seconds);
                ++correlated;
            }
        }
        prev_c = s.controller_position;
        prev_b = s.body_position;
        have_prev = true;
    }
    if (correlated < 2) {
        session.status = LatencyProbeStatus::Failed;
        session.reason = "insufficient_motion";
        return;
    }
    const float avg = static_cast<float>(sum / correlated);
    session.estimated_latency_seconds = avg;
    const float conf = std::min(1.0f, static_cast<float>(correlated) / 16.0f);
    session.confidence = conf;
    if (conf < 0.25f) {
        session.status = LatencyProbeStatus::Weak;
        session.reason = "confidence_too_low";
    } else {
        session.status = LatencyProbeStatus::Valid;
        session.reason = "valid";
    }
}

SteamVrAlignmentStatus BuildSteamVrAlignmentStatus(const SteamVrAlignmentStatusInputs& in) {
    SteamVrAlignmentStatus s;
    // Provider.
    s.provider_available = in.provider_snapshot.available;
    s.provider_status = in.provider_snapshot.status;
    s.provider_reason = in.provider_snapshot.reason;
    s.provider_runtime_initialized = in.provider_snapshot.runtime_initialized;
    s.left_controller_tracked = in.provider_snapshot.left.valid;
    s.right_controller_tracked = in.provider_snapshot.right.valid;
    s.left_trigger_pressed = in.provider_snapshot.left.trigger_pressed;
    s.right_trigger_pressed = in.provider_snapshot.right.trigger_pressed;
    s.left_trigger_pressed_edge = in.provider_snapshot.left.trigger_pressed_edge;
    s.right_trigger_pressed_edge = in.provider_snapshot.right.trigger_pressed_edge;
    s.controller_device_count = in.provider_snapshot.device_count;
    s.last_pose_age_seconds = MaxValidControllerPoseAgeSeconds(in.provider_snapshot);
    s.max_allowed_pose_age_seconds = kMaxControllerPoseAgeSeconds;
    s.provider_compile_disabled = ReasonLooksCompileDisabled(in.provider_snapshot.reason);
    // "Unavailable" is often recoverable in normal SteamVR use: the headset may be
    // asleep, Virtual Desktop/Link may not have connected yet, or controllers may
    // be off. Only mark the provider structurally unavailable when this build or
    // backend truly cannot construct an OpenVR provider. The UI still reports the
    // live provider.reason, but it must leave Start calibration retryable so the
    // next click can poll SteamVR again and surface errors such as Hmd Not Found.
    s.provider_hard_unavailable = s.provider_compile_disabled ||
        in.provider_snapshot.reason.find("provider not constructed") != std::string::npos;
    s.controller_alignment_fresh = ControllerAlignmentFresh(in.provider_snapshot);

    // Session/solve.
    s.session_active = in.session.active;
    s.accepted_sample_count = in.session.accepted_sample_count;

    int recorded = 0;
    for (const auto& sample : in.session.samples) {
        if (sample.reason_code != SteamVrAlignmentReason::Idle) {
            ++recorded;
            s.samples.push_back(sample);
        }
    }
    s.total_samples_recorded = recorded;

    int required_present = 0;
    for (auto lm : {SteamVrAlignmentLandmark::LeftFoot,
                    SteamVrAlignmentLandmark::RightFoot,
                    SteamVrAlignmentLandmark::Pelvis,
                    SteamVrAlignmentLandmark::Floor}) {
        const auto& sample = in.session.samples[LandmarkSlotIndex(lm)];
        if (sample.accepted) {
            ++required_present;
        }
    }
    s.required_samples_present = required_present;
    s.required_samples_complete = required_present == 4;

    // Solve.
    const auto& r = in.last_solve;
    const TrackerSpaceSource parsed_source = ParseTrackerSpaceSource(in.osc_config.tracker_space_source);
    const bool source_known = parsed_source != TrackerSpaceSource::Unknown;
    const bool config_marked_stale = in.osc_config.tracker_space_source == "steamvr_controller_alignment_stale";
    const bool config_says_controller_active =
        in.osc_config.tracker_space_transform_valid &&
        in.osc_config.tracker_space_source == "steamvr_controller_alignment";
    const bool have_real_solve = r.required_samples_present > 0 || r.valid;
    const bool persisted_controller_alignment =
        config_says_controller_active &&
        !r.valid &&
        !have_real_solve &&
        !in.session.active &&
        recorded == 0;

    s.transform_valid = r.valid || persisted_controller_alignment || in.osc_config.tracker_space_transform_valid;
    s.source_known = source_known;
    s.raw_active_transform_source = in.osc_config.tracker_space_source;
    if (!source_known) {
        s.reason_code = SteamVrAlignmentReason::Weak;
        s.reason = "unknown tracker_space_source label; using numeric transform";
    }
    s.role_offsets_present = config_says_controller_active ||
        (r.role_offsets_present[TrackerRoleIndex(TrackerRole::Pelvis)] &&
         r.role_offsets_present[TrackerRoleIndex(TrackerRole::LeftFoot)] &&
         r.role_offsets_present[TrackerRoleIndex(TrackerRole::RightFoot)] &&
         r.role_offsets_present[TrackerRoleIndex(TrackerRole::Chest)] &&
         r.role_offsets_present[TrackerRoleIndex(TrackerRole::LeftElbow)] &&
         r.role_offsets_present[TrackerRoleIndex(TrackerRole::RightElbow)]);
    s.confidence = persisted_controller_alignment ? in.osc_config.steamvr_alignment_confidence : r.confidence;
    s.residual_m = persisted_controller_alignment ? in.osc_config.steamvr_alignment_residual_m : r.residual_m;
    s.floor_residual_m = persisted_controller_alignment ? in.osc_config.steamvr_floor_residual_m : r.floor_residual_m;
    s.yaw_offset_rad = persisted_controller_alignment ? in.osc_config.steamvr_yaw_offset_rad : r.yaw_offset_rad;
    s.yaw_disagreement_rad = r.yaw_disagreement_rad;
    s.scale_ratio = persisted_controller_alignment ? in.osc_config.steamvr_scale_ratio : r.scale_ratio;
    s.scale_mismatch = r.scale_mismatch;
    // Prefer the session's reason when no real solve has happened yet (default solve
    // reason is Failed, which would mask provider/controller-availability reasons).
    if (have_real_solve) {
        s.reason_code = r.reason_code != SteamVrAlignmentReason::None ? r.reason_code : in.session.reason_code;
        s.reason = !r.reason.empty() ? r.reason : in.session.reason;
    } else {
        s.reason_code = in.session.reason_code != SteamVrAlignmentReason::None
            ? in.session.reason_code
            : (r.reason_code != SteamVrAlignmentReason::None ? r.reason_code : SteamVrAlignmentReason::Idle);
        s.reason = !in.session.reason.empty() ? in.session.reason : r.reason;
    }

    // High-level state. Staleness is driven by calibration signatures AND by the
    // live SteamVR provider/controller freshness. A persisted controller transform
    // is not current truth if OpenVR is unavailable, controllers disappear, or the
    // last controller pose is too old.
    const SteamVrAlignmentReason provider_stale_reason = ProviderStaleReason(in.provider_snapshot);
    const bool provider_stale = config_says_controller_active && provider_stale_reason != SteamVrAlignmentReason::None;
    if (!source_known && in.osc_config.tracker_space_transform_valid) {
        s.state = "weak";
        s.stale = false;
        s.stale_reason = "unknown_tracker_space_source_label";
        s.reason_code = SteamVrAlignmentReason::Weak;
        s.reason = "unknown tracker_space_source label; using numeric transform";
        s.transform_valid = true;
    } else if ((in.stale && (r.valid || config_says_controller_active)) || provider_stale || config_marked_stale) {
        s.state = "stale";
        s.reason_code = provider_stale ? provider_stale_reason : SteamVrAlignmentReason::AlignmentStale;
        s.reason = ToString(s.reason_code);
        s.stale_reason = s.reason;
        s.stale = true;
        s.transform_valid = in.osc_config.tracker_space_transform_valid || r.valid || persisted_controller_alignment;
    } else if (r.valid || persisted_controller_alignment || in.osc_config.tracker_space_transform_valid) {
        s.state = (s.confidence >= 0.65f) ? "valid" : "weak";
        if (persisted_controller_alignment) {
            s.reason_code = s.confidence >= 0.65f ? SteamVrAlignmentReason::Valid : SteamVrAlignmentReason::Weak;
            s.reason = in.osc_config.steamvr_alignment_reason.empty()
                ? ToString(s.reason_code)
                : in.osc_config.steamvr_alignment_reason;
        }
    } else if (in.session.active) {
        s.state = required_present == 4 ? "failed" : "sampling";
    } else if (recorded > 0) {
        s.state = "failed";
    } else {
        s.state = "missing";
    }

    // Active transform source. Precedence: valid controller > manual > stale > unavailable/none.
    // When alignment is stale AND manual fallback is available, prefer manual.
    // Stale controller alignment must not silently substitute for the manual fallback.
    s.body_calibration_valid = in.body_calibration_valid;
    s.floor_calibration_valid = in.floor_calibration_valid;
    s.body_state_stable = in.body_state_stable;
    s.body_signature = in.body_signature;
    s.floor_signature = in.floor_signature;
    s.last_alignment_timestamp = in.last_alignment_timestamp;
    s.manual_fallback_available = ManualTrackerSpaceFallbackAvailable(in.osc_config);

    const std::string& src = in.osc_config.tracker_space_source;
    if (!source_known && in.osc_config.tracker_space_transform_valid) {
        s.active_transform_source = TrackerSpaceSource::Unknown;
    } else if (s.stale || config_marked_stale) {
        // Stale alignment must prefer manual fallback when available.
        // Stale controller alignment is degraded evidence, not proof that the
        // fallback transform is invalid.
        if (ManualTrackerSpaceFallbackAvailable(in.osc_config)) {
            s.active_transform_source = TrackerSpaceSource::Manual;
            s.manual_fallback_active = true;
        } else if (in.osc_config.tracker_space_transform_valid) {
            s.active_transform_source = TrackerSpaceSource::StaleSteamVr;
        } else {
            s.active_transform_source = TrackerSpaceSource::None;
        }
    } else if (in.osc_config.tracker_space_transform_valid && src == "steamvr_controller_alignment") {
        s.active_transform_source = TrackerSpaceSource::SteamVrController;
    } else if (in.osc_config.tracker_space_transform_valid && parsed_source == TrackerSpaceSource::Manual) {
        s.active_transform_source = TrackerSpaceSource::Manual;
        s.manual_fallback_active = true;
    } else if (ManualTrackerSpaceFallbackAvailable(in.osc_config)) {
        s.active_transform_source = TrackerSpaceSource::Manual;
        s.manual_fallback_active = true;
    } else if (!in.provider_snapshot.available) {
        s.active_transform_source = TrackerSpaceSource::Unavailable;
    } else {
        s.active_transform_source = TrackerSpaceSource::None;
    }

    return s;
}

} // namespace bt
