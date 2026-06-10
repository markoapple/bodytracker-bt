#include "tracking/foot_support.h"

#include <algorithm>
#include <cmath>

#include "tracking/contact_constraints.h"
#include "tracking/tracking_constants.h"

namespace bt {
namespace {

float Clamp01(float value);

bool FloorSupportAllowed(PostureMode posture_mode) {
    return posture_mode == PostureMode::UprightStanding ||
        posture_mode == PostureMode::UprightTransition ||
        posture_mode == PostureMode::Crouching ||
        posture_mode == PostureMode::Kneeling ||
        posture_mode == PostureMode::SeatedSupported;
}

bool RestSupportAllowed(PostureMode posture_mode) {
    return posture_mode == PostureMode::SeatedSupported ||
        posture_mode == PostureMode::ReclinedSupported;
}

bool HeelAnchorAllowed(FootContactLoad load) {
    return load == FootContactLoad::FullPlant ||
        load == FootContactLoad::HeelOnly ||
        load == FootContactLoad::Inferred;
}

bool ToeAnchorAllowed(FootContactLoad load) {
    return load == FootContactLoad::FullPlant ||
        load == FootContactLoad::ToeOnly ||
        load == FootContactLoad::Inferred;
}

void SetContactAnchors(
    FootSupportState& out,
    const Pose3f& pose,
    float foot_length_m,
    float heel_confidence,
    float toe_confidence) {

    out.heel_contact_confidence = Clamp01(heel_confidence);
    out.toe_contact_confidence = Clamp01(toe_confidence);

    const bool heel_allowed = HeelAnchorAllowed(out.contact_load);
    const bool toe_allowed = ToeAnchorAllowed(out.contact_load);

    out.heel_anchor.pose = pose;
    out.heel_anchor.pose.position = FootHeelContactPoint(pose, foot_length_m);
    out.heel_anchor.active = heel_allowed && out.heel_contact_confidence > 0.0f;
    out.heel_anchor.dwell_seconds = out.anchor.dwell_seconds;
    out.heel_anchor.release_seconds = out.anchor.release_seconds;
    out.heel_anchor.confidence = out.heel_anchor.active ? std::min(out.anchor.confidence, out.heel_contact_confidence) : 0.0f;

    out.toe_anchor.pose = pose;
    out.toe_anchor.pose.position = FootToeContactPoint(pose, foot_length_m);
    out.toe_anchor.active = toe_allowed && out.toe_contact_confidence > 0.0f;
    out.toe_anchor.dwell_seconds = out.anchor.dwell_seconds;
    out.toe_anchor.release_seconds = out.anchor.release_seconds;
    out.toe_anchor.confidence = out.toe_anchor.active ? std::min(out.anchor.confidence, out.toe_contact_confidence) : 0.0f;
}

void DisableContactAnchors(FootSupportState& out) {
    out.contact_load = FootContactLoad::None;
    out.heel_contact_confidence = 0.0f;
    out.toe_contact_confidence = 0.0f;
    out.heel_anchor.active = false;
    out.heel_anchor.confidence = 0.0f;
    out.heel_anchor.has_contact_history = false;
    out.toe_anchor.active = false;
    out.toe_anchor.confidence = 0.0f;
    out.toe_anchor.has_contact_history = false;
}

void UpdateAnchorContactHistory(
    SupportAnchor& out,
    const SupportAnchor& previous,
    const Vec3f& measured_contact) {

    if (!out.active) {
        out.has_contact_history = false;
        out.previous_contact_position = {};
        out.current_contact_position = {};
        return;
    }
    out.previous_contact_position = previous.has_contact_history
        ? previous.current_contact_position
        : measured_contact;
    out.current_contact_position = measured_contact;
    out.has_contact_history = true;
}

void UpdateFootContactHistory(
    FootSupportState& out,
    const FootSupportState& previous,
    const Pose3f& foot_pose,
    float foot_length_m) {

    UpdateAnchorContactHistory(out.anchor, previous.anchor, foot_pose.position);
    UpdateAnchorContactHistory(out.heel_anchor, previous.heel_anchor, FootHeelContactPoint(foot_pose, foot_length_m));
    UpdateAnchorContactHistory(out.toe_anchor, previous.toe_anchor, FootToeContactPoint(foot_pose, foot_length_m));
}

void RefreshContactAnchors(FootSupportState& out) {
    if (!out.anchor.active || out.type != FootSupportType::FloorSupport) {
        DisableContactAnchors(out);
        return;
    }
    out.heel_anchor.active = HeelAnchorAllowed(out.contact_load) && out.heel_contact_confidence > 0.0f;
    out.heel_anchor.dwell_seconds = out.anchor.dwell_seconds;
    out.heel_anchor.release_seconds = out.anchor.release_seconds;
    out.heel_anchor.confidence = out.heel_anchor.active ? std::min(out.anchor.confidence, out.heel_contact_confidence) : 0.0f;
    out.toe_anchor.active = ToeAnchorAllowed(out.contact_load) && out.toe_contact_confidence > 0.0f;
    out.toe_anchor.dwell_seconds = out.anchor.dwell_seconds;
    out.toe_anchor.release_seconds = out.anchor.release_seconds;
    out.toe_anchor.confidence = out.toe_anchor.active ? std::min(out.anchor.confidence, out.toe_contact_confidence) : 0.0f;
}

float Clamp01(float value) {
    if (!std::isfinite(value)) {
        return 0.0f;
    }
    return std::max(0.0f, std::min(1.0f, value));
}

float EvidenceConfidence(const FootSupportEvidence* evidence) {
    if (!evidence) {
        return 0.0f;
    }
    const float aggregate = std::max(
        std::isfinite(evidence->confidence) ? evidence->confidence : 0.0f,
        std::max(
            std::isfinite(evidence->heel_confidence) ? evidence->heel_confidence : 0.0f,
            std::isfinite(evidence->toe_confidence) ? evidence->toe_confidence : 0.0f));
    if (!evidence->usable && aggregate <= 0.0f) {
        return 0.0f;
    }
    return Clamp01(aggregate);
}

float HeelEvidenceConfidence(const FootSupportEvidence* evidence) {
    if (!evidence) {
        return 0.0f;
    }
    if (evidence->heel_usable && std::isfinite(evidence->heel_confidence)) {
        return Clamp01(evidence->heel_confidence);
    }
    return Clamp01(0.5f * EvidenceConfidence(evidence));
}

float ToeEvidenceConfidence(const FootSupportEvidence* evidence) {
    if (!evidence) {
        return 0.0f;
    }
    if (evidence->toe_usable && std::isfinite(evidence->toe_confidence)) {
        return Clamp01(evidence->toe_confidence);
    }
    return Clamp01(0.5f * EvidenceConfidence(evidence));
}

FootContactLoad ContactLoadFromEvidence(float heel_confidence, float toe_confidence, float threshold) {
    const bool heel = heel_confidence >= threshold;
    const bool toe = toe_confidence >= threshold;
    if (heel && toe) {
        return FootContactLoad::FullPlant;
    }
    if (heel) {
        return FootContactLoad::HeelOnly;
    }
    if (toe) {
        return FootContactLoad::ToeOnly;
    }
    return FootContactLoad::None;
}

FootContactLoad NormalizedPreviousContactLoad(const FootSupportState& previous) {
    if (!previous.anchor.active || previous.type != FootSupportType::FloorSupport) {
        return FootContactLoad::None;
    }
    if (previous.contact_load != FootContactLoad::Inferred) {
        return previous.contact_load;
    }
    switch (previous.phase) {
    case FootSupportPhase::HeelLock:
    case FootSupportPhase::ContactCandidate:
        return FootContactLoad::HeelOnly;
    case FootSupportPhase::FlatPlant:
        return FootContactLoad::FullPlant;
    case FootSupportPhase::ToePivot:
        return FootContactLoad::ToeOnly;
    default:
        return FootContactLoad::None;
    }
}

float ContactTransitionQuality(
    const FootSupportState& previous,
    FootContactLoad next_load,
    bool has_explicit_contact_split,
    bool near_floor,
    bool swing_motion,
    float motion_magnitude_m) {

    if (!has_explicit_contact_split ||
        !previous.anchor.active ||
        previous.type != FootSupportType::FloorSupport ||
        !near_floor ||
        swing_motion ||
        next_load == FootContactLoad::None ||
        next_load == FootContactLoad::Inferred) {
        return 1.0f;
    }

    const FootContactLoad prev_load = NormalizedPreviousContactLoad(previous);
    if (prev_load == FootContactLoad::None || prev_load == next_load) {
        return 1.0f;
    }

    // Prefer physically sane gait order: heel strike -> full plant -> toe push-off -> swing.
    // Directly jumping from toe-only support back to heel-only support reuses a stale
    // stance anchor unless the evidence first releases or carries real backward-step motion.
    if (prev_load == FootContactLoad::ToeOnly && next_load == FootContactLoad::HeelOnly) {
        if (motion_magnitude_m > 0.05f) {
            return 0.55f;
        }
        return 0.20f;
    }
    if (prev_load == FootContactLoad::HeelOnly && next_load == FootContactLoad::ToeOnly) {
        return 0.55f;
    }
    if (prev_load == FootContactLoad::FullPlant && next_load == FootContactLoad::HeelOnly) {
        return 0.65f;
    }
    return 1.0f;
}

void BeginSupport(
    FootSupportState& out,
    FootSupportType type,
    FootSupportPhase phase,
    FootContactLoad contact_load,
    const Pose3f& pose,
    float evidence_confidence,
    float heel_confidence,
    float toe_confidence,
    float foot_length_m) {

    out.type = type;
    out.phase = phase;
    out.contact_load = contact_load;
    out.anchor.pose = pose;
    out.anchor.active = true;
    out.anchor.dwell_seconds = 0.0;
    out.anchor.release_seconds = 0.0;
    out.anchor.confidence = std::min(0.25f, evidence_confidence);
    if (type == FootSupportType::FloorSupport) {
        SetContactAnchors(out, pose, foot_length_m, heel_confidence, toe_confidence);
    } else {
        DisableContactAnchors(out);
        out.contact_load = FootContactLoad::Inferred;
    }
}

float FootPitchFromOrientation(const Pose3f& foot_pose) {
    const Vec3f forward = Rotate(foot_pose.orientation, Vec3f{0.0f, 0.0f, 1.0f});
    const float horizontal = std::sqrt(forward.x * forward.x + forward.z * forward.z);
    return std::atan2(forward.y, std::max(1e-5f, horizontal));
}

bool FootContactLoadNearFloorPlane(
    const Pose3f& foot_pose,
    const FloorPlane& floor,
    const FootSupportConfig& config,
    float calibrated_foot_length_m,
    FootContactLoad load) {

    if (!FloorPlaneUsable(floor)) {
        return false;
    }
    const Vec3f heel = FootHeelContactPoint(foot_pose, calibrated_foot_length_m);
    const Vec3f toe = FootToeContactPoint(foot_pose, calibrated_foot_length_m);
    const float heel_d = std::abs(SignedDistanceToFloorPlane(heel, floor));
    const float toe_d = std::abs(SignedDistanceToFloorPlane(toe, floor));
    const bool heel_near = heel_d <= config.floor_height_epsilon;
    const bool toe_near = toe_d <= config.floor_height_epsilon;
    switch (load) {
    case FootContactLoad::HeelOnly:
        return heel_near;
    case FootContactLoad::ToeOnly:
        return toe_near;
    case FootContactLoad::FullPlant:
        return heel_near && toe_near;
    case FootContactLoad::Inferred:
        return heel_near || toe_near;
    case FootContactLoad::None:
    default:
        return false;
    }
}

} // namespace

FootSupportState UpdateFootSupportCalibrated(
    const FootSupportState& previous,
    const Pose3f& foot_pose,
    const Pose3f& previous_foot_pose,
    PostureMode posture_mode,
    const FloorPlane& floor,
    double dt_seconds,
    float calibrated_foot_length_m,
    const FootSupportConfig& config,
    const FootSupportEvidence* evidence) {

    FootSupportState out = previous;
    out.transition_quality = 1.0f;
    const double dt = std::isfinite(dt_seconds) && dt_seconds > 0.0 ? dt_seconds : 0.0;
    const float motion = Distance(foot_pose.position, previous_foot_pose.position);
    const bool near_floor = FootContactLoadNearFloorPlane(
        foot_pose,
        floor,
        config,
        calibrated_foot_length_m,
        FootContactLoad::Inferred);
    const bool low_motion = motion <= config.rest_motion_epsilon;
    const bool swing_motion = motion >= config.swing_motion_epsilon;
    const float foot_pitch = FootPitchFromOrientation(foot_pose);
    const float raw_evidence_confidence = EvidenceConfidence(evidence);
    const float raw_heel_evidence_confidence = HeelEvidenceConfidence(evidence);
    const float raw_toe_evidence_confidence = ToeEvidenceConfidence(evidence);
    const float contact_threshold = std::max(0.0f, config.min_contact_evidence_confidence);
    const bool has_explicit_contact_split = evidence && (evidence->heel_usable || evidence->toe_usable);
    const FootContactLoad raw_evidence_load = has_explicit_contact_split
        ? ContactLoadFromEvidence(
            raw_heel_evidence_confidence,
            raw_toe_evidence_confidence,
            contact_threshold)
        : FootContactLoad::Inferred;
    const bool raw_load_near_floor = FootContactLoadNearFloorPlane(
        foot_pose,
        floor,
        config,
        calibrated_foot_length_m,
        raw_evidence_load == FootContactLoad::None ? FootContactLoad::Inferred : raw_evidence_load);
    const float transition_quality = ContactTransitionQuality(
        previous,
        raw_evidence_load,
        has_explicit_contact_split,
        raw_load_near_floor,
        swing_motion,
        motion);
    const float evidence_confidence = Clamp01(raw_evidence_confidence * transition_quality);
    const float heel_evidence_confidence = Clamp01(raw_heel_evidence_confidence * transition_quality);
    const float toe_evidence_confidence = Clamp01(raw_toe_evidence_confidence * transition_quality);
    const FootContactLoad evidence_load = has_explicit_contact_split
        ? ContactLoadFromEvidence(
            heel_evidence_confidence,
            toe_evidence_confidence,
            contact_threshold)
        : FootContactLoad::Inferred;
    out.transition_quality = transition_quality;
    const bool load_near_floor = FootContactLoadNearFloorPlane(
        foot_pose,
        floor,
        config,
        calibrated_foot_length_m,
        evidence_load == FootContactLoad::None ? FootContactLoad::Inferred : evidence_load);
    const bool contact_evidence_present = evidence && evidence->usable;
    const bool reliable_contact_evidence = contact_evidence_present && evidence_confidence >= contact_threshold;
    const bool missing_contact_evidence = !contact_evidence_present;
    const bool previous_support_active = previous.anchor.active && previous.type != FootSupportType::None;
    const bool geometry_contradicts_floor_anchor =
        previous.type == FootSupportType::FloorSupport && (!near_floor || swing_motion);
    const bool geometry_contradicts_rest_anchor =
        previous.type == FootSupportType::RestSupport && (near_floor || swing_motion);
    const bool geometry_contradicts_previous_anchor =
        geometry_contradicts_floor_anchor || geometry_contradicts_rest_anchor;
    const bool active_contact_contradiction =
        previous_support_active && contact_evidence_present &&
        (!reliable_contact_evidence || geometry_contradicts_previous_anchor);

    FootSupportType desired_type = FootSupportType::None;
    FootSupportPhase desired_phase = FootSupportPhase::Swing;
    FootContactLoad desired_load = FootContactLoad::None;

    if (load_near_floor && FloorSupportAllowed(posture_mode) && !swing_motion && reliable_contact_evidence) {
        desired_type = FootSupportType::FloorSupport;
        desired_load = evidence_load == FootContactLoad::None ? FootContactLoad::Inferred : evidence_load;
        if (has_explicit_contact_split && evidence_load == FootContactLoad::FullPlant) {
            desired_phase = previous.phase == FootSupportPhase::ContactCandidate && previous.anchor.dwell_seconds < config.lock_dwell_seconds
                ? FootSupportPhase::ContactCandidate
                : FootSupportPhase::FlatPlant;
        } else if (has_explicit_contact_split && evidence_load == FootContactLoad::ToeOnly) {
            desired_phase = FootSupportPhase::ToePivot;
        } else if (has_explicit_contact_split && evidence_load == FootContactLoad::HeelOnly) {
            desired_phase = FootSupportPhase::HeelLock;
        } else if (foot_pitch > config.heel_lock_pitch_threshold) {
            desired_phase = FootSupportPhase::HeelLock;
        } else if (foot_pitch < -config.toe_pivot_pitch_threshold) {
            desired_phase = FootSupportPhase::ToePivot;
        } else {
            desired_phase = previous.phase == FootSupportPhase::ContactCandidate && previous.anchor.dwell_seconds < config.lock_dwell_seconds
                ? FootSupportPhase::ContactCandidate
                : FootSupportPhase::FlatPlant;
        }
    } else if (RestSupportAllowed(posture_mode) && low_motion && !near_floor && reliable_contact_evidence) {
        desired_type = FootSupportType::RestSupport;
        desired_load = FootContactLoad::Inferred;
        desired_phase = previous.phase == FootSupportPhase::RestCandidate && previous.anchor.dwell_seconds < config.lock_dwell_seconds
            ? FootSupportPhase::RestCandidate
            : FootSupportPhase::RestLock;
    }

    if (desired_type == out.type && desired_phase == out.phase) {
        if (desired_type == FootSupportType::None) {
            out.anchor.active = false;
            out.anchor.confidence = 0.0f;
            out.anchor.has_contact_history = false;
            out.anchor.dwell_seconds = 0.0;
            out.anchor.release_seconds = 0.0;
            DisableContactAnchors(out);
            return out;
        }
        out.anchor.dwell_seconds += dt;
        out.anchor.release_seconds = 0.0;
        out.anchor.confidence = std::min(evidence_confidence, out.anchor.confidence + static_cast<float>(dt * 3.0));
        out.contact_load = desired_load;
        out.heel_contact_confidence = heel_evidence_confidence;
        out.toe_contact_confidence = toe_evidence_confidence;
        RefreshContactAnchors(out);
        if (out.phase == FootSupportPhase::ContactCandidate && out.anchor.dwell_seconds >= config.lock_dwell_seconds) {
            out.phase = FootSupportPhase::FlatPlant;
            out.anchor.dwell_seconds = 0.0;
        }
        if (out.phase == FootSupportPhase::RestCandidate && out.anchor.dwell_seconds >= config.lock_dwell_seconds) {
            out.phase = FootSupportPhase::RestLock;
            out.anchor.dwell_seconds = 0.0;
        }
    } else if (desired_type == FootSupportType::None) {
        out.anchor.release_seconds += dt;
        if (previous_support_active && missing_contact_evidence && !geometry_contradicts_previous_anchor) {
            out.anchor.confidence = std::max(
                0.0f,
                out.anchor.confidence - static_cast<float>(dt * config.missing_evidence_confidence_decay_per_second));
            RefreshContactAnchors(out);
            if (out.anchor.release_seconds >= config.missing_evidence_release_seconds || out.anchor.confidence <= 0.02f) {
                out.type = FootSupportType::None;
                out.phase = FootSupportPhase::Swing;
                out.anchor.active = false;
                out.anchor.confidence = 0.0f;
                out.anchor.has_contact_history = false;
                DisableContactAnchors(out);
            }
            UpdateFootContactHistory(out, previous, foot_pose, calibrated_foot_length_m);
            return out;
        }
        out.phase = swing_motion ? FootSupportPhase::Slip : FootSupportPhase::ReleasePending;
        const double confidence_release_seconds = active_contact_contradiction
            ? std::max(1e-6, config.release_seconds * 0.5)
            : std::max(1e-6, config.release_seconds);
        const double required_release_seconds = active_contact_contradiction
            ? std::max(0.0, config.release_seconds * 0.5)
            : config.release_seconds;
        out.anchor.confidence = std::max(0.0f, out.anchor.confidence - static_cast<float>(dt / confidence_release_seconds));
        RefreshContactAnchors(out);
        if (out.anchor.release_seconds >= required_release_seconds) {
            out.type = FootSupportType::None;
            out.phase = FootSupportPhase::Swing;
            out.anchor.active = false;
            out.anchor.confidence = 0.0f;
            out.anchor.has_contact_history = false;
            DisableContactAnchors(out);
        }
    } else if (desired_type == out.type) {
        out.phase = desired_phase;
        out.contact_load = desired_load;
        out.anchor.dwell_seconds += dt;
        out.anchor.release_seconds = 0.0;
        out.anchor.confidence = std::min(evidence_confidence, out.anchor.confidence + static_cast<float>(dt * 3.0));
        out.heel_contact_confidence = heel_evidence_confidence;
        out.toe_contact_confidence = toe_evidence_confidence;
        RefreshContactAnchors(out);
    } else {
        const FootSupportPhase entry_phase =
            desired_type == FootSupportType::FloorSupport
                ? (desired_phase == FootSupportPhase::FlatPlant ? FootSupportPhase::ContactCandidate : desired_phase)
                : FootSupportPhase::RestCandidate;
        Pose3f anchor_pose = foot_pose;
        if (desired_type == FootSupportType::FloorSupport && FloorPlaneUsable(floor)) {
            anchor_pose.position = ProjectPointToFloorPlane(anchor_pose.position, floor);
        }
        BeginSupport(
            out,
            desired_type,
            entry_phase,
            desired_load,
            anchor_pose,
            evidence_confidence,
            heel_evidence_confidence,
            toe_evidence_confidence,
            calibrated_foot_length_m);
    }

    UpdateFootContactHistory(out, previous, foot_pose, calibrated_foot_length_m);
    return out;
}

FootSupportState UpdateFootSupport(
    const FootSupportState& previous,
    const Pose3f& foot_pose,
    const Pose3f& previous_foot_pose,
    PostureMode posture_mode,
    const FloorPlane& floor,
    double dt_seconds,
    const FootSupportConfig& config,
    const FootSupportEvidence* evidence) {

    return UpdateFootSupportCalibrated(
        previous,
        foot_pose,
        previous_foot_pose,
        posture_mode,
        floor,
        dt_seconds,
        tracking_constants::kDefaultFootLengthM,
        config,
        evidence);
}

} // namespace bt
