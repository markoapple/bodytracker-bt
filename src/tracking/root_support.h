#pragma once

#include "core/types.h"
#include "tracking/body_model.h"

namespace bt {

struct RootSupportConfig {
    double lock_dwell_seconds = 0.25;
    double release_seconds = 0.12;
    double transition_hold_seconds = 0.18;
};

struct KneeContactEvidence {
    float left_confidence = 0.0f;
    float right_confidence = 0.0f;
    bool left_usable = false;
    bool right_usable = false;
};

struct KneeContactConfig {
    float min_contact_confidence = 0.42f;
    double lock_dwell_seconds = 0.12;
    double release_seconds = 0.10;
    double missing_evidence_release_seconds = 0.34;
    float missing_evidence_confidence_decay_per_second = 1.60f;
};

SupportManifoldState UpdateKneeContactSupport(
    const SupportManifoldState& previous,
    const LowerBodyState& solved_state,
    const LowerBodyModel& model,
    const FloorPlane& floor,
    PostureMode posture_mode,
    double dt_seconds,
    const KneeContactEvidence& evidence,
    const KneeContactConfig& config = {});

SupportManifoldState UpdateRootSupport(
    const SupportManifoldState& previous,
    const LowerBodyState& solved_state,
    PostureMode posture_mode,
    double dt_seconds,
    const RootSupportConfig& config = {});

} // namespace bt
