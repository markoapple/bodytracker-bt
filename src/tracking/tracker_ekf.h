#pragma once

// Linear Kalman filter for tracker pose smoothing.
//
// Three independent 1D constant-velocity KFs (one per axis per role: root,
// left_foot, right_foot), plus a SLERP-based orientation smoother for feet.
//
// This is NOT an Extended Kalman Filter: the state model (position+velocity)
// and measurement model (z=position) are both linear. There is no Jacobian,
// no nonlinear model, and no linearization step. The naming in the codebase
// ("tracker_ekf") is a historical misnomer; the implementation is a standard
// linear KF with Mahalanobis-gated outlier rejection.

#include "core/config.h"
#include "core/types.h"
#include "tracking/body_model.h"

#include <array>

namespace bt {

struct AxisKalmanState {
    float position = 0.0f;
    float velocity = 0.0f;
    float p00 = 1.0f;
    float p01 = 0.0f;
    float p10 = 0.0f;
    float p11 = 1.0f;
    bool initialized = false;
};

struct TrackerEkfRoleState {
    std::array<AxisKalmanState, 3> axes{};
    Quatf orientation{};
    FootSupportPhase last_phase_for_transition_detect = FootSupportPhase::Swing;
    float confidence = 0.0f;
    bool initialized = false;
    bool orientation_initialized = false;
};

struct TrackerEkfState {
    TrackerEkfRoleState root{};
    TrackerEkfRoleState left_foot{};
    TrackerEkfRoleState right_foot{};
};

struct TrackerEkfRoleTelemetry {
    bool initialized = false;
    bool filtered = false;
    bool locked_reset = false;
    float support_confidence = 0.0f;
    float measurement_variance_m2 = 0.0f;
    float innovation_m = 0.0f;
    float mahalanobis_chi2 = 0.0f;
    float mean_position_gain = 0.0f;
    float orientation_gain = 0.0f;
    bool outlier_inflated = false;
};

struct TrackerEkfTelemetry {
    bool enabled = false;
    bool applied = false;
    bool reset = false;
    float input_confidence = 0.0f;
    TrackerEkfRoleTelemetry root{};
    TrackerEkfRoleTelemetry left_foot{};
    TrackerEkfRoleTelemetry right_foot{};
    bool root_initialized = false;
    bool left_foot_initialized = false;
    bool right_foot_initialized = false;
};

void ResetTrackerEkf(TrackerEkfState& filter);

LowerBodyState ApplyTrackerEkf(
    TrackerEkfState& filter,
    const LowerBodyState& measured,
    double dt_seconds,
    const TrackerEkfConfig& config = {},
    TrackerEkfTelemetry* telemetry = nullptr,
    const LowerBodyModel* model = nullptr);

} // namespace bt
