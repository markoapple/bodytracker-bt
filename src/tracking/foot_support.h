#pragma once

#include "calibration/calibration_types.h"
#include "core/types.h"

namespace bt {

struct FootSupportEvidence {
    float confidence = 0.0f;
    float heel_confidence = 0.0f;
    float toe_confidence = 0.0f;
    bool heel_usable = false;
    bool toe_usable = false;
    bool usable = false;
};

struct FootSupportConfig {
    float floor_height_epsilon = 0.055f;
    float rest_motion_epsilon = 0.035f;
    float swing_motion_epsilon = 0.090f;
    float heel_lock_pitch_threshold = 0.20f;
    float toe_pivot_pitch_threshold = 0.35f;
    double lock_dwell_seconds = 0.18;
    double release_seconds = 0.08;
    double missing_evidence_release_seconds = 0.38;
    float missing_evidence_confidence_decay_per_second = 1.35f;
    float min_contact_evidence_confidence = 0.35f;
};

FootSupportState UpdateFootSupport(
    const FootSupportState& previous,
    const Pose3f& foot_pose,
    const Pose3f& previous_foot_pose,
    PostureMode posture_mode,
    const FloorPlane& floor,
    double dt_seconds,
    const FootSupportConfig& config = {},
    const FootSupportEvidence* evidence = nullptr);

FootSupportState UpdateFootSupportCalibrated(
    const FootSupportState& previous,
    const Pose3f& foot_pose,
    const Pose3f& previous_foot_pose,
    PostureMode posture_mode,
    const FloorPlane& floor,
    double dt_seconds,
    float calibrated_foot_length_m,
    const FootSupportConfig& config = {},
    const FootSupportEvidence* evidence = nullptr);

} // namespace bt
