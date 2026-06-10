#include "calibration/body_calibrator.h"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace bt {
namespace {

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

std::size_t Idx(KeypointId id) {
    return static_cast<std::size_t>(id);
}

const BodySolveJointTriangulationTelemetry& Joint(const BodySolveStereoTelemetry& telemetry, KeypointId id) {
    return telemetry.joints[Idx(id)];
}

bool HasJoint(const BodySolveJointTriangulationTelemetry& joint) {
    return (joint.triangulated || joint.depth_inferred) && IsFinite(joint.world) && joint.confidence > 0.0f;
}

float JointWeight(const BodySolveStereoTelemetry& telemetry, KeypointId id) {
    const auto& joint = Joint(telemetry, id);
    if (!HasJoint(joint)) {
        return 0.0f;
    }
    const float source_gain = joint.triangulated
        ? 1.0f
        : Clamp(0.35f + 0.45f * telemetry.monocular_floor_assist_confidence, 0.25f, 0.80f);
    return Clamp01(joint.confidence * source_gain);
}

float PairWeight(const BodySolveStereoTelemetry& telemetry, KeypointId a, KeypointId b) {
    const float wa = JointWeight(telemetry, a);
    const float wb = JointWeight(telemetry, b);
    if (wa <= 0.0f || wb <= 0.0f) {
        return 0.0f;
    }
    return std::sqrt(wa * wb);
}

void AddScalar(BodyCalibrationEstimatorState::ScalarAccumulator& acc, float value, float weight, float lo, float hi) {
    if (!std::isfinite(value) || value < lo || value > hi || weight <= 0.0f) {
        return;
    }
    const float w = Clamp01(weight);
    acc.weighted_sum += value * w;
    acc.weighted_sq_sum += value * value * w;
    acc.weight_sum += w;
    acc.count += 1;
}

void AddVec(BodyCalibrationEstimatorState::VecAccumulator& acc, Vec3f value, float weight) {
    if (!IsFinite(value) || weight <= 0.0f) {
        return;
    }
    const float w = Clamp01(weight);
    acc.weighted_sum = Add(acc.weighted_sum, Scale(value, w));
    acc.weight_sum += w;
    acc.count += 1;
}

float Mean(const BodyCalibrationEstimatorState::ScalarAccumulator& acc, float fallback) {
    return acc.weight_sum > 1e-5f ? acc.weighted_sum / acc.weight_sum : fallback;
}

Vec3f Mean(const BodyCalibrationEstimatorState::VecAccumulator& acc, Vec3f fallback) {
    return acc.weight_sum > 1e-5f ? Scale(acc.weighted_sum, 1.0f / acc.weight_sum) : fallback;
}

float CoeffVar(const BodyCalibrationEstimatorState::ScalarAccumulator& acc) {
    if (acc.weight_sum <= 1e-5f) {
        return 1.0f;
    }
    const float mean = Mean(acc, 0.0f);
    if (mean <= 1e-5f) {
        return 1.0f;
    }
    const float ex2 = acc.weighted_sq_sum / acc.weight_sum;
    const float var = std::max(0.0f, ex2 - mean * mean);
    return std::sqrt(var) / mean;
}

float Quality(const BodyCalibrationEstimatorState::ScalarAccumulator& acc, float required_seconds, float max_cv) {
    if (acc.weight_sum <= 1e-5f) {
        return 0.0f;
    }
    const float count_quality = Clamp01(acc.weight_sum / std::max(1.0f, required_seconds * 18.0f));
    const float stability = Clamp01(1.0f - CoeffVar(acc) / std::max(1e-3f, max_cv));
    return Clamp01(0.35f * count_quality + 0.65f * stability);
}

float VecQuality(const BodyCalibrationEstimatorState::VecAccumulator& acc, float required_seconds) {
    if (acc.weight_sum <= 1e-5f) {
        return 0.0f;
    }
    return Clamp01(acc.weight_sum / std::max(1.0f, required_seconds * 18.0f));
}

bool HasMeasuredScalar(const BodyCalibrationEstimatorState::ScalarAccumulator& acc) {
    return acc.count > 0 && acc.weight_sum > 1e-5f;
}

bool RequiredBodyCoverageComplete(const BodyCalibrationEstimatorState& state) {
    return HasMeasuredScalar(state.pelvis_width) &&
        HasMeasuredScalar(state.left_femur) &&
        HasMeasuredScalar(state.right_femur) &&
        HasMeasuredScalar(state.left_tibia) &&
        HasMeasuredScalar(state.right_tibia);
}

bool PlausibleNeutralStance(const BodySolveStereoTelemetry& telemetry) {
    const auto& lh = Joint(telemetry, KeypointId::LeftHip);
    const auto& rh = Joint(telemetry, KeypointId::RightHip);
    const auto& lk = Joint(telemetry, KeypointId::LeftKnee);
    const auto& rk = Joint(telemetry, KeypointId::RightKnee);
    const auto& la = Joint(telemetry, KeypointId::LeftAnkle);
    const auto& ra = Joint(telemetry, KeypointId::RightAnkle);
    if (!HasJoint(lh) || !HasJoint(rh) || !HasJoint(lk) || !HasJoint(rk) || !HasJoint(la) || !HasJoint(ra)) {
        return false;
    }

    const float pelvis_width = Distance(lh.world, rh.world);
    const float left_femur = Distance(lh.world, lk.world);
    const float right_femur = Distance(rh.world, rk.world);
    const float left_tibia = Distance(lk.world, la.world);
    const float right_tibia = Distance(rk.world, ra.world);
    if (pelvis_width < 0.18f || pelvis_width > 0.55f ||
        left_femur < 0.25f || left_femur > 0.70f || right_femur < 0.25f || right_femur > 0.70f ||
        left_tibia < 0.25f || left_tibia > 0.70f || right_tibia < 0.25f || right_tibia > 0.70f) {
        return false;
    }

    const auto ratio_ok = [](float a, float b) {
        const float r = a / std::max(1e-5f, b);
        return r > 0.70f && r < 1.45f;
    };
    if (!ratio_ok(left_femur, right_femur) || !ratio_ok(left_tibia, right_tibia)) {
        return false;
    }

    const Vec3f pelvis = Scale(Add(lh.world, rh.world), 0.5f);
    const Vec3f ankles = Scale(Add(la.world, ra.world), 0.5f);
    return pelvis.y > ankles.y + 0.45f;
}

bool AddSegmentSample(
    BodyCalibrationEstimatorState::ScalarAccumulator& acc,
    const BodySolveStereoTelemetry& telemetry,
    KeypointId a,
    KeypointId b,
    float lo,
    float hi) {
    const float w = PairWeight(telemetry, a, b);
    if (w <= 0.0f) {
        return false;
    }
    const float value = Distance(Joint(telemetry, a).world, Joint(telemetry, b).world);
    const int before = acc.count;
    AddScalar(acc, value, w, lo, hi);
    return acc.count > before;
}

bool AddFootSample(
    BodyCalibrationEstimatorState::ScalarAccumulator& acc,
    const BodySolveStereoTelemetry& telemetry,
    bool left) {
    const KeypointId heel_id = left ? KeypointId::LeftHeel : KeypointId::RightHeel;
    const KeypointId toe_id = left ? KeypointId::LeftBigToe : KeypointId::RightBigToe;
    const KeypointId small_id = left ? KeypointId::LeftSmallToe : KeypointId::RightSmallToe;
    const float w = PairWeight(telemetry, heel_id, toe_id);
    if (w <= 0.0f) {
        return false;
    }
    Vec3f toe = Joint(telemetry, toe_id).world;
    if (HasJoint(Joint(telemetry, small_id))) {
        toe = Scale(Add(toe, Joint(telemetry, small_id).world), 0.5f);
    }
    const float value = Distance(Joint(telemetry, heel_id).world, toe);
    const int before = acc.count;
    AddScalar(acc, value, w, 0.12f, 0.38f);
    return acc.count > before;
}

BodyCalibration BuildCalibration(const BodyCalibrationEstimatorState& state, const BodyCalibrationModeConfig& cfg) {
    BodyCalibration out = state.body;
    out.pelvis_width = Mean(state.pelvis_width, out.pelvis_width);
    out.left_femur = Mean(state.left_femur, out.left_femur);
    out.right_femur = Mean(state.right_femur, out.right_femur);
    out.left_tibia = Mean(state.left_tibia, out.left_tibia);
    out.right_tibia = Mean(state.right_tibia, out.right_tibia);
    out.left_foot_length = Mean(state.left_foot_length, out.left_foot_length);
    out.right_foot_length = Mean(state.right_foot_length, out.right_foot_length);
    out.standing_hmd_to_pelvis = Mean(state.standing_hmd_to_pelvis, out.standing_hmd_to_pelvis);

    out.quality.pelvis_width = Quality(state.pelvis_width, cfg.required_seconds, cfg.max_segment_cv);
    out.quality.left_femur = Quality(state.left_femur, cfg.required_seconds, cfg.max_segment_cv);
    out.quality.right_femur = Quality(state.right_femur, cfg.required_seconds, cfg.max_segment_cv);
    out.quality.left_tibia = Quality(state.left_tibia, cfg.required_seconds, cfg.max_segment_cv);
    out.quality.right_tibia = Quality(state.right_tibia, cfg.required_seconds, cfg.max_segment_cv);
    out.quality.left_foot_length = Quality(state.left_foot_length, cfg.required_seconds, cfg.max_segment_cv * 1.25f);
    out.quality.right_foot_length = Quality(state.right_foot_length, cfg.required_seconds, cfg.max_segment_cv * 1.25f);
    out.quality.standing_hmd_to_pelvis = VecQuality(state.standing_hmd_to_pelvis, cfg.required_seconds);
    out.quality.sample_count = state.accepted_samples;

    const float required_sum =
        out.quality.pelvis_width + out.quality.left_femur + out.quality.right_femur +
        out.quality.left_tibia + out.quality.right_tibia;
    const float optional_foot = 0.5f * (out.quality.left_foot_length + out.quality.right_foot_length);
    const float hmd = out.quality.standing_hmd_to_pelvis;
    out.quality.overall = Clamp01(0.78f * (required_sum / 5.0f) + 0.12f * optional_foot + 0.10f * hmd);
    const bool required_coverage_complete = RequiredBodyCoverageComplete(state);
    out.quality.source = required_coverage_complete
        ? "neutral_pose_runtime"
        : "neutral_pose_runtime_partial_with_priors";
    out.standing_neutral_valid = required_coverage_complete &&
        out.quality.overall >= cfg.min_overall_confidence;
    return out;
}

} // namespace

void ResetBodyCalibrationEstimator(BodyCalibrationEstimatorState& state, const BodyCalibration& current) {
    state = BodyCalibrationEstimatorState{};
    state.body = current;
}

BodyCalibrationTelemetry UpdateBodyCalibrationEstimator(
    BodyCalibrationEstimatorState& state,
    const BodyCalibrationModeConfig& config,
    const BodySolveStereoTelemetry& telemetry,
    const HmdPoseSample& hmd,
    double dt_seconds) {

    BodyCalibrationTelemetry out;
    out.enabled = config.enabled;
    out.auto_persist = config.auto_persist;
    out.body = state.body;
    out.accumulated_seconds = state.elapsed_seconds;
    out.accepted_samples = state.accepted_samples;
    out.persisted = state.persisted;
    out.persist_pending = state.persist_pending;
    out.persist_status = state.persist_status;
    out.persist_error = state.persist_error;
    if (!config.enabled) {
        out.reason = "disabled";
        out.persist_status = "disabled";
        return out;
    }
    if (!std::isfinite(dt_seconds) || dt_seconds <= 0.0) {
        dt_seconds = 1.0 / 60.0;
    }
    if (state.complete) {
        out.complete = true;
        out.body = state.body;
        out.used_stereo = state.body.quality.source.find("stereo") != std::string::npos;
        out.used_monocular_floor_scale = state.body.quality.source.find("monocular_floor_scale") != std::string::npos;
        out.accumulated_seconds = state.elapsed_seconds;
        out.accepted_samples = state.accepted_samples;
        out.overall_confidence = state.body.quality.overall;
        out.reason = "complete";
        out.persist_status = config.auto_persist ? state.persist_status : "auto_persist_disabled";
        return out;
    }

    out.used_stereo = telemetry.depth_source == DepthSource::TriangulatedStereo || telemetry.triangulated_count > 0;
    out.used_monocular_floor_scale = telemetry.tracking_mode == TrackingMode::Monocular &&
        telemetry.monocular_scale_source == MonocularScaleSource::FloorSpacing &&
        telemetry.monocular_floor_assist_confidence > 0.20f;

    bool accepted_any = false;
    accepted_any |= AddSegmentSample(state.pelvis_width, telemetry, KeypointId::LeftHip, KeypointId::RightHip, 0.18f, 0.55f);
    accepted_any |= AddSegmentSample(state.left_femur, telemetry, KeypointId::LeftHip, KeypointId::LeftKnee, 0.25f, 0.70f);
    accepted_any |= AddSegmentSample(state.right_femur, telemetry, KeypointId::RightHip, KeypointId::RightKnee, 0.25f, 0.70f);
    accepted_any |= AddSegmentSample(state.left_tibia, telemetry, KeypointId::LeftKnee, KeypointId::LeftAnkle, 0.25f, 0.70f);
    accepted_any |= AddSegmentSample(state.right_tibia, telemetry, KeypointId::RightKnee, KeypointId::RightAnkle, 0.25f, 0.70f);
    accepted_any |= AddFootSample(state.left_foot_length, telemetry, true);
    accepted_any |= AddFootSample(state.right_foot_length, telemetry, false);

    if (hmd.valid && HasJoint(Joint(telemetry, KeypointId::LeftHip)) && HasJoint(Joint(telemetry, KeypointId::RightHip))) {
        const float w = 0.5f * (JointWeight(telemetry, KeypointId::LeftHip) + JointWeight(telemetry, KeypointId::RightHip));
        const Vec3f pelvis = Scale(Add(Joint(telemetry, KeypointId::LeftHip).world, Joint(telemetry, KeypointId::RightHip).world), 0.5f);
        AddVec(state.standing_hmd_to_pelvis, Sub(pelvis, hmd.pose.position), w);
        accepted_any = accepted_any || w > 0.0f;
    }

    if (!accepted_any) {
        out.reason = "neutral_pose_visible_but_no_usable_segment_samples";
        return out;
    }

    state.elapsed_seconds += static_cast<float>(dt_seconds);
    state.accepted_samples += 1;
    state.body = BuildCalibration(state, config);
    const bool required_coverage_complete = RequiredBodyCoverageComplete(state);
    const std::string source_base = out.used_stereo
        ? "neutral_pose_runtime_stereo"
        : (out.used_monocular_floor_scale ? "neutral_pose_runtime_monocular_floor_scale" : "neutral_pose_runtime_inferred");
    state.body.quality.source = required_coverage_complete
        ? source_base
        : source_base + "_partial_with_priors";
    state.complete = state.elapsed_seconds >= config.required_seconds &&
        state.body.standing_neutral_valid;
    state.dirty = state.complete;

    out.complete = state.complete;
    out.body = state.body;
    out.accumulated_seconds = state.elapsed_seconds;
    out.accepted_samples = state.accepted_samples;
    out.overall_confidence = state.body.quality.overall;
    out.reason = state.complete ? "complete" : "accumulating_neutral_samples";
    out.persist_status = state.complete
        ? (config.auto_persist ? "complete_pending_persist" : "auto_persist_disabled")
        : "not_complete";
    return out;
}

} // namespace bt
