#include "tracking/tracker_ekf.h"

#include "tracking/contact_constraints.h"

#include "tracking/support_queries.h"
#include "tracking/tracking_constants.h"

#include <algorithm>
#include <cmath>

namespace bt {
namespace {

float Clamp(float v, float lo, float hi) {
    if (!std::isfinite(v)) {
        return lo;
    }
    return std::max(lo, std::min(hi, v));
}

float Clamp01(float v) {
    return Clamp(v, 0.0f, 1.0f);
}

float Coord(const Vec3f& v, int axis) {
    switch (axis) {
    case 0: return v.x;
    case 1: return v.y;
    default: return v.z;
    }
}

void SetCoord(Vec3f& v, int axis, float value) {
    switch (axis) {
    case 0: v.x = value; break;
    case 1: v.y = value; break;
    default: v.z = value; break;
    }
}

struct AxisCorrectionStats {
    float gain = 0.0f;
    float residual = 0.0f;
};

struct PositionFilterStats {
    bool filtered = false;
    bool outlier_inflated = false;
    float support_confidence = 0.0f;
    float measurement_variance_m2 = 0.0f;
    float innovation_m = 0.0f;
    float mahalanobis_chi2 = 0.0f;
    float mean_position_gain = 0.0f;
};

float MeasurementVariance(float confidence, float support_confidence, const TrackerEkfConfig& config) {
    const float c = Clamp01(confidence);
    float variance = config.max_measurement_variance_m2 +
        (config.min_measurement_variance_m2 - config.max_measurement_variance_m2) * c;
    if (support_confidence > 0.0f) {
        const float scale = 1.0f - Clamp01(support_confidence) * (1.0f - Clamp01(config.support_variance_scale));
        variance *= scale;
    }
    return Clamp(variance, config.min_measurement_variance_m2, config.max_measurement_variance_m2);
}

void Predict(AxisKalmanState& axis, float dt, float process_noise_mps2, float missing_velocity_decay, bool evidence_missing) {
    axis.position += axis.velocity * dt;
    if (evidence_missing) {
        axis.velocity *= missing_velocity_decay;
    }

    const float dt2 = dt * dt;
    const float dt3 = dt2 * dt;
    const float dt4 = dt2 * dt2;
    const float q = std::max(0.0f, process_noise_mps2 * process_noise_mps2);

    const float p00 = axis.p00 + dt * (axis.p10 + axis.p01) + dt2 * axis.p11 + 0.25f * dt4 * q;
    const float p01 = axis.p01 + dt * axis.p11 + 0.5f * dt3 * q;
    const float p10 = axis.p10 + dt * axis.p11 + 0.5f * dt3 * q;
    const float p11 = axis.p11 + dt2 * q;

    axis.p00 = p00;
    axis.p01 = p01;
    axis.p10 = p10;
    axis.p11 = p11;
}

AxisCorrectionStats Correct(AxisKalmanState& axis, float measurement, float variance) {
    const float s = axis.p00 + std::max(variance, 1e-8f);
    if (!std::isfinite(s) || s <= 1e-8f) {
        const float residual = measurement - axis.position;
        axis.position = measurement;
        axis.velocity = 0.0f;
        axis.p00 = variance;
        axis.p01 = 0.0f;
        axis.p10 = 0.0f;
        axis.p11 = 1.0f;
        return AxisCorrectionStats{1.0f, residual};
    }

    const float k0 = axis.p00 / s;
    const float k1 = axis.p10 / s;
    const float residual = measurement - axis.position;

    const float p00 = (1.0f - k0) * axis.p00;
    const float p01 = (1.0f - k0) * axis.p01;
    const float p10 = axis.p10 - k1 * axis.p00;
    const float p11 = axis.p11 - k1 * axis.p01;

    axis.position += k0 * residual;
    axis.velocity += k1 * residual;
    axis.p00 = std::max(0.0f, p00);
    axis.p01 = p01;
    axis.p10 = p10;
    axis.p11 = std::max(0.0f, p11);
    return AxisCorrectionStats{k0, residual};
}

Vec3f FilterPose(
    TrackerEkfRoleState& role,
    const Vec3f& measured,
    float confidence,
    float support_confidence,
    float dt,
    const TrackerEkfConfig& config,
    PositionFilterStats* stats) {

    const float base_variance = MeasurementVariance(confidence, support_confidence, config);
    if (stats) {
        *stats = {};
        stats->support_confidence = Clamp01(support_confidence);
        stats->measurement_variance_m2 = base_variance;
    }

    if (!role.initialized) {
        for (int axis = 0; axis < 3; ++axis) {
            auto& s = role.axes[static_cast<std::size_t>(axis)];
            s.position = Coord(measured, axis);
            s.velocity = 0.0f;
            s.p00 = config.max_measurement_variance_m2;
            s.p01 = 0.0f;
            s.p10 = 0.0f;
            s.p11 = 1.0f;
            s.initialized = true;
        }
        role.initialized = true;
        role.confidence = Clamp01(confidence);
        if (stats) {
            stats->filtered = true;
            stats->mean_position_gain = 1.0f;
        }
        return measured;
    }

    const bool evidence_missing = confidence <= 0.0f;
    Vec3f out;
    for (int axis = 0; axis < 3; ++axis) {
        auto& s = role.axes[static_cast<std::size_t>(axis)];
        Predict(s, dt, config.process_noise_mps2, config.missing_velocity_decay, evidence_missing);
        SetCoord(out, axis, s.position);
    }

    if (evidence_missing) {
        role.confidence = Clamp01(confidence);
        if (stats) {
            stats->filtered = true;
            stats->measurement_variance_m2 = base_variance;
            stats->mean_position_gain = 0.0f;
            stats->innovation_m = 0.0f;
        }
        return out;
    }

    Vec3f pre_residual{};
    float chi2 = 0.0f;
    for (int axis = 0; axis < 3; ++axis) {
        const auto& s = role.axes[static_cast<std::size_t>(axis)];
        const float residual = Coord(measured, axis) - s.position;
        SetCoord(pre_residual, axis, residual);
        const float innovation_variance = std::max(s.p00 + base_variance, 1e-8f);
        chi2 += (residual * residual) / innovation_variance;
    }

    float variance = base_variance;
    const bool outlier =
        config.mahalanobis_gate_enabled &&
        std::isfinite(chi2) &&
        chi2 > std::max(0.0f, config.mahalanobis_gate_chi2);
    if (outlier) {
        variance = Clamp(
            base_variance * std::max(1.0f, config.outlier_variance_scale),
            config.min_measurement_variance_m2,
            std::max(config.max_measurement_variance_m2, base_variance * std::max(1.0f, config.outlier_variance_scale)));
    }

    float gain_sum = 0.0f;
    Vec3f residual{};
    for (int axis = 0; axis < 3; ++axis) {
        auto& s = role.axes[static_cast<std::size_t>(axis)];
        const auto corrected = Correct(s, Coord(measured, axis), variance);
        gain_sum += corrected.gain;
        SetCoord(residual, axis, corrected.residual);
        SetCoord(out, axis, s.position);
    }
    role.confidence = Clamp01(confidence);
    if (stats) {
        stats->filtered = true;
        stats->outlier_inflated = outlier;
        stats->measurement_variance_m2 = variance;
        stats->mean_position_gain = gain_sum / 3.0f;
        stats->innovation_m = Length(pre_residual);
        stats->mahalanobis_chi2 = chi2;
    }
    return out;
}
Quatf FilterFootOrientation(
    TrackerEkfRoleState& role,
    const Quatf& measured_orientation,
    const TrackerEkfConfig& config) {

    const Quatf measured = Normalize(measured_orientation);
    if (!role.orientation_initialized) {
        role.orientation = measured;
        role.orientation_initialized = true;
        return measured;
    }

    role.orientation = Slerp(role.orientation, measured, config.foot_orientation_gain);
    return role.orientation;
}

void ResetRoleToPose(
    TrackerEkfRoleState& role,
    const Pose3f& pose,
    float confidence,
    const TrackerEkfConfig& config) {

    for (int axis = 0; axis < 3; ++axis) {
        auto& s = role.axes[static_cast<std::size_t>(axis)];
        s.position = Coord(pose.position, axis);
        s.velocity = 0.0f;
        s.p00 = config.max_measurement_variance_m2;
        s.p01 = 0.0f;
        s.p10 = 0.0f;
        s.p11 = 1.0f;
        s.initialized = true;
    }
    role.orientation = Normalize(pose.orientation);
    role.confidence = Clamp01(confidence);
    role.initialized = true;
    role.orientation_initialized = true;
}

void CopyPositionStats(TrackerEkfRoleTelemetry& telemetry, const PositionFilterStats& stats) {
    telemetry.filtered = stats.filtered;
    telemetry.support_confidence = stats.support_confidence;
    telemetry.measurement_variance_m2 = stats.measurement_variance_m2;
    telemetry.innovation_m = stats.innovation_m;
    telemetry.mahalanobis_chi2 = stats.mahalanobis_chi2;
    telemetry.mean_position_gain = stats.mean_position_gain;
    telemetry.outlier_inflated = stats.outlier_inflated;
}


float FootMeasurementConfidence(float body_confidence, const TrackerEvidence& evidence, const FootSupportState& support) {
    float c = 0.0f;
    switch (evidence.source) {
    case TrackerEvidenceSource::DirectStereo:
    case TrackerEvidenceSource::ReplayInput:
    case TrackerEvidenceSource::InferredMonocular:
        c = evidence.direct_confidence;
        break;
    case TrackerEvidenceSource::AnchorHeld:
        c = evidence.support_confidence;
        break;
    case TrackerEvidenceSource::HmdPrediction:
    case TrackerEvidenceSource::Predicted:
        c = 0.5f * body_confidence;
        break;
    case TrackerEvidenceSource::None:
    default:
        c = body_confidence;
        break;
    }
    c = std::max(c, FootSupportConfidence(support));
    if (evidence.source == TrackerEvidenceSource::AnchorHeld) {
        return Clamp01(c);
    }
    return Clamp01(std::min(Clamp01(body_confidence), c));
}

float FootLengthFor(const LowerBodyModel* model, bool left) {
    if (!model) {
        return tracking_constants::kDefaultFootLengthM;
    }
    return left ? model->left_foot_length : model->right_foot_length;
}

Pose3f FilterFootPose(
    TrackerEkfRoleState& role,
    const Pose3f& measured_pose,
    const FootSupportState& support,
    float confidence,
    float dt,
    const TrackerEkfConfig& config,
    TrackerEkfRoleTelemetry* telemetry,
    float foot_length_m) {

    if (telemetry) {
        *telemetry = {};
        telemetry->support_confidence = FootSupportConfidence(support);
        telemetry->orientation_gain = config.foot_orientation_gain;
    }

    const bool full_plant = FootSupportIsFullPlant(support);
    const bool became_full_plant =
        full_plant && role.last_phase_for_transition_detect != FootSupportPhase::FlatPlant;
    role.last_phase_for_transition_detect = support.phase;

    if (full_plant) {
        const Pose3f constrained = ApplyFootContactConstraint(measured_pose, support, foot_length_m);
        if (became_full_plant) {
            ResetRoleToPose(role, constrained, support.anchor.confidence, config);
        }
        if (telemetry) {
            telemetry->initialized = role.initialized;
            telemetry->locked_reset = became_full_plant;
            telemetry->filtered = false;
        }
        return constrained;
    }

    PositionFilterStats stats;
    Pose3f out = FootSupportHasContactConstraint(support)
        ? ApplyFootContactConstraint(measured_pose, support, foot_length_m)
        : measured_pose;
    out.position = FilterPose(role, out.position, confidence, FootSupportConfidence(support), dt, config, &stats);
    const bool had_orientation = role.orientation_initialized;
    out.orientation = FilterFootOrientation(role, measured_pose.orientation, config);
    if (telemetry) {
        telemetry->initialized = role.initialized;
        telemetry->orientation_gain = had_orientation ? config.foot_orientation_gain : 1.0f;
        CopyPositionStats(*telemetry, stats);
    }
    return out;
}

void FillTelemetry(
    TrackerEkfTelemetry* telemetry,
    const TrackerEkfState& filter,
    const TrackerEkfConfig& config,
    float confidence,
    bool applied,
    bool reset) {

    if (!telemetry) {
        return;
    }
    telemetry->enabled = config.enabled;
    telemetry->applied = applied;
    telemetry->reset = reset;
    telemetry->input_confidence = Clamp01(confidence);
    telemetry->root.initialized = filter.root.initialized;
    telemetry->left_foot.initialized = filter.left_foot.initialized;
    telemetry->right_foot.initialized = filter.right_foot.initialized;
    telemetry->root_initialized = filter.root.initialized;
    telemetry->left_foot_initialized = filter.left_foot.initialized;
    telemetry->right_foot_initialized = filter.right_foot.initialized;
}

} // namespace

void ResetTrackerEkf(TrackerEkfState& filter) {
    filter = {};
}

LowerBodyState ApplyTrackerEkf(
    TrackerEkfState& filter,
    const LowerBodyState& measured,
    double dt_seconds,
    const TrackerEkfConfig& config,
    TrackerEkfTelemetry* telemetry,
    const LowerBodyModel* model) {

    if (telemetry) {
        *telemetry = {};
    }
    FillTelemetry(telemetry, filter, config, measured.confidence, false, false);
    if (!config.enabled) {
        return measured;
    }
    if (measured.confidence <= 0.0f) {
        const float dt = Clamp(static_cast<float>(dt_seconds), 1.0f / 240.0f, 0.10f);
        LowerBodyState predicted = measured;
        predicted.root.position = FilterPose(filter.root, measured.root.position, 0.0f, 0.0f, dt, config, nullptr);
        predicted.left_foot.position = FilterPose(filter.left_foot, measured.left_foot.position, 0.0f, FootSupportConfidence(measured.support.left_foot), dt, config, nullptr);
        predicted.right_foot.position = FilterPose(filter.right_foot, measured.right_foot.position, 0.0f, FootSupportConfidence(measured.support.right_foot), dt, config, nullptr);
        predicted.confidence = 0.0f;
        FillTelemetry(telemetry, filter, config, measured.confidence, true, true);
        return predicted;
    }

    const float dt = Clamp(static_cast<float>(dt_seconds), 1.0f / 240.0f, 0.10f);
    LowerBodyState out = measured;

    PositionFilterStats root_stats;
    const float root_confidence = Clamp01(measured.confidence);
    out.root.position = FilterPose(filter.root, measured.root.position, root_confidence, 0.0f, dt, config, &root_stats);
    if (telemetry) {
        telemetry->root.initialized = filter.root.initialized;
        CopyPositionStats(telemetry->root, root_stats);
    }

    out.left_foot = FilterFootPose(
        filter.left_foot,
        measured.left_foot,
        measured.support.left_foot,
        FootMeasurementConfidence(measured.confidence, measured.left_foot_evidence, measured.support.left_foot),
        dt,
        config,
        telemetry ? &telemetry->left_foot : nullptr,
        FootLengthFor(model, true));
    out.right_foot = FilterFootPose(
        filter.right_foot,
        measured.right_foot,
        measured.support.right_foot,
        FootMeasurementConfidence(measured.confidence, measured.right_foot_evidence, measured.support.right_foot),
        dt,
        config,
        telemetry ? &telemetry->right_foot : nullptr,
        FootLengthFor(model, false));
    FillTelemetry(telemetry, filter, config, measured.confidence, true, false);
    return out;
}

} // namespace bt
