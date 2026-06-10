#pragma once

#include "calibration/calibration_types.h"
#include "core/config.h"
#include "core/types.h"
#include "tracking/body_solver.h"

namespace bt {

struct BodyCalibrationTelemetry {
    bool enabled = false;
    bool auto_persist = false;
    bool complete = false;
    bool saved_this_frame = false;
    bool persisted = false;
    bool persist_pending = false;
    bool used_stereo = false;
    bool used_monocular_floor_scale = false;
    float accumulated_seconds = 0.0f;
    int accepted_samples = 0;
    float overall_confidence = 0.0f;
    BodyCalibration body{};
    std::string reason = "disabled";
    std::string persist_status = "disabled";
    std::string persist_error;
};

struct BodyCalibrationEstimatorState {
    BodyCalibration body{};
    float elapsed_seconds = 0.0f;
    int accepted_samples = 0;
    bool complete = false;
    bool dirty = false;
    bool persisted = false;
    bool persist_pending = false;
    std::string persist_status = "disabled";
    std::string persist_error;

    struct ScalarAccumulator {
        float weighted_sum = 0.0f;
        float weighted_sq_sum = 0.0f;
        float weight_sum = 0.0f;
        int count = 0;
    };

    struct VecAccumulator {
        Vec3f weighted_sum{};
        float weight_sum = 0.0f;
        int count = 0;
    };

    ScalarAccumulator pelvis_width{};
    ScalarAccumulator left_femur{};
    ScalarAccumulator right_femur{};
    ScalarAccumulator left_tibia{};
    ScalarAccumulator right_tibia{};
    ScalarAccumulator left_foot_length{};
    ScalarAccumulator right_foot_length{};
    VecAccumulator standing_hmd_to_pelvis{};
};

void ResetBodyCalibrationEstimator(BodyCalibrationEstimatorState& state, const BodyCalibration& current);
BodyCalibrationTelemetry UpdateBodyCalibrationEstimator(
    BodyCalibrationEstimatorState& state,
    const BodyCalibrationModeConfig& config,
    const BodySolveStereoTelemetry& telemetry,
    const HmdPoseSample& hmd,
    double dt_seconds);

} // namespace bt
