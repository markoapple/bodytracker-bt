#include "tracking/steamvr_alignment_manager.h"

#include "core/math.h"

#include <algorithm>
#include <utility>

namespace bt {

namespace {

// Pick the controller pose to use for a given preferred role; fall back to whichever
// is valid. Either controller can sample any landmark.
const SteamVrControllerPose& PickController(const SteamVrPoseSnapshot& snapshot, SteamVrControllerRole pref) {
    if (pref == SteamVrControllerRole::LeftHand && snapshot.left.valid) {
        return snapshot.left;
    }
    if (pref == SteamVrControllerRole::RightHand && snapshot.right.valid) {
        return snapshot.right;
    }
    if (snapshot.left.valid) {
        return snapshot.left;
    }
    return snapshot.right;
}

bool BodyCalibrationLooksValid(const BodyCalibration& body) {
    return body.standing_neutral_valid || body.quality.overall > 0.0f;
}

void ClearTriggerEdgeForController(SteamVrPoseSnapshot& snapshot, SteamVrControllerRole role) {
    if (role == SteamVrControllerRole::LeftHand) {
        snapshot.left.trigger_pressed_edge = false;
    } else if (role == SteamVrControllerRole::RightHand) {
        snapshot.right.trigger_pressed_edge = false;
    }
}

bool SnapshotHasTriggerEdgeFor(const SteamVrPoseSnapshot& snapshot, SteamVrControllerRole preference) {
    if (!snapshot.available) {
        return false;
    }
    if (preference == SteamVrControllerRole::LeftHand) {
        return snapshot.left.valid && snapshot.left.trigger_pressed_edge;
    }
    if (preference == SteamVrControllerRole::RightHand) {
        return snapshot.right.valid && snapshot.right.trigger_pressed_edge;
    }
    return (snapshot.left.valid && snapshot.left.trigger_pressed_edge) ||
        (snapshot.right.valid && snapshot.right.trigger_pressed_edge);
}

} // namespace

SteamVrAlignmentManager::SteamVrAlignmentManager()
    : provider_(std::make_unique<SteamVrPoseProvider>()) {}

SteamVrAlignmentManager::SteamVrAlignmentManager(std::unique_ptr<ISteamVrPoseProvider> provider)
    : provider_(std::move(provider)) {}

SteamVrAlignmentManager::~SteamVrAlignmentManager() = default;

void SteamVrAlignmentManager::SetProvider(std::unique_ptr<ISteamVrPoseProvider> provider) {
    std::scoped_lock lock(mutex_);
    provider_ = std::move(provider);
    last_snapshot_ = MakeUnavailableSnapshot("provider replaced");
    session_ = SteamVrAlignmentSession{};
    last_solve_ = SteamVrAlignmentSolveResult{};
}

SteamVrPoseSnapshot SteamVrAlignmentManager::Poll() {
    std::scoped_lock lock(mutex_);
    if (!provider_) {
        last_snapshot_ = MakeUnavailableSnapshot("provider not constructed");
        return last_snapshot_;
    }
    last_snapshot_ = provider_->Poll();
    return last_snapshot_;
}

SteamVrAlignmentStatus SteamVrAlignmentManager::StartSession() {
    SteamVrPoseSnapshot snap;
    {
        std::scoped_lock lock(mutex_);
        if (provider_) {
            last_snapshot_ = provider_->Poll();
        } else {
            last_snapshot_ = MakeUnavailableSnapshot("provider not constructed");
        }
        snap = last_snapshot_;
        session_ = StartSteamVrAlignmentSession(snap);
        last_solve_ = SteamVrAlignmentSolveResult{};
    }
    OscConfig empty;
    CalibrationBundle empty_calib;
    return Status(empty, empty_calib, /*body_state_stable=*/false);
}

SteamVrAlignmentStatus SteamVrAlignmentManager::RecordSample(
    SteamVrAlignmentLandmark landmark,
    SteamVrControllerRole controller_preference,
    const UnifiedBodyState& body_state,
    const CalibrationBundle& calibration,
    double timestamp_seconds) {

    {
        std::scoped_lock lock(mutex_);
        if (provider_ && !SnapshotHasTriggerEdgeFor(last_snapshot_, controller_preference)) {
            last_snapshot_ = provider_->Poll();
        }
        if (!session_.active) {
            session_ = StartSteamVrAlignmentSession(last_snapshot_);
        }
        if (!session_.active) {
            // Session failed to start (provider unavailable or controllers missing).
            return BuildStatusLocked(OscConfig{}, calibration,
                /*body_state_stable=*/body_state.valid);
        }
        auto controller = PickController(last_snapshot_, controller_preference);
        ClearTriggerEdgeForController(last_snapshot_, controller.role);
        auto sample = CaptureSteamVrAlignmentSample(
            landmark, controller, body_state, calibration, timestamp_seconds);
        StoreSteamVrAlignmentSample(session_, sample);
    }
    return Status(OscConfig{}, calibration, /*body_state_stable=*/body_state.valid);
}

SteamVrAlignmentStatus SteamVrAlignmentManager::RedoSample(SteamVrAlignmentLandmark landmark) {
    {
        std::scoped_lock lock(mutex_);
        RedoSteamVrAlignmentSample(session_, landmark);
    }
    return Status(OscConfig{}, CalibrationBundle{}, /*body_state_stable=*/false);
}

SteamVrAlignmentStatus SteamVrAlignmentManager::FinishSession(
    const UnifiedBodyState& body_state,
    const CalibrationBundle& calibration,
    OscConfig& osc_config_in_out,
    double timestamp_seconds) {

    {
        std::scoped_lock lock(mutex_);
        last_solve_ = SolveSteamVrAlignment(session_, body_state, calibration);
        if (last_solve_.valid) {
            ApplySteamVrAlignmentToOscConfig(osc_config_in_out, last_solve_);
            last_alignment_timestamp_ = timestamp_seconds;
        } else {
            // Failed solve must NOT overwrite a last good controller alignment. The
            // in-memory solve result already explains the failure for the current UI
            // response; persisted config should keep describing the active transform.
            const bool preserving_previous_controller_alignment =
                osc_config_in_out.tracker_space_transform_valid &&
                osc_config_in_out.tracker_space_source == "steamvr_controller_alignment";
            if (!preserving_previous_controller_alignment) {
                osc_config_in_out.steamvr_alignment_status = last_solve_.status;
                osc_config_in_out.steamvr_alignment_reason = last_solve_.reason;
                osc_config_in_out.steamvr_alignment_confidence = last_solve_.confidence;
                osc_config_in_out.steamvr_alignment_residual_m = last_solve_.residual_m;
                osc_config_in_out.steamvr_floor_residual_m = last_solve_.floor_residual_m;
                osc_config_in_out.steamvr_yaw_offset_rad = last_solve_.yaw_offset_rad;
                osc_config_in_out.steamvr_scale_ratio = last_solve_.scale_ratio;
            }
        }
    }
    return Status(osc_config_in_out, calibration, /*body_state_stable=*/body_state.valid);
}

SteamVrAlignmentStatus SteamVrAlignmentManager::ClearAlignment(OscConfig& osc_config_in_out) {
    {
        std::scoped_lock lock(mutex_);
        ClearSteamVrAlignmentFromOscConfig(osc_config_in_out);
        session_ = SteamVrAlignmentSession{};
        last_solve_ = SteamVrAlignmentSolveResult{};
        last_alignment_timestamp_ = 0.0;
    }
    return Status(osc_config_in_out, CalibrationBundle{}, /*body_state_stable=*/false);
}

SteamVrAlignmentStatus SteamVrAlignmentManager::Status(
    const OscConfig& osc_config,
    const CalibrationBundle& calibration,
    bool body_state_stable) {
    std::scoped_lock lock(mutex_);
    return BuildStatusLocked(osc_config, calibration, body_state_stable);
}

bool SteamVrAlignmentManager::SessionActive() const {
    std::scoped_lock lock(mutex_);
    return session_.active;
}

SteamVrAlignmentStatus SteamVrAlignmentManager::BuildStatusLocked(
    const OscConfig& osc_config,
    const CalibrationBundle& calibration,
    bool body_state_stable) const {

    SteamVrAlignmentStatusInputs in;
    in.provider_snapshot = last_snapshot_;
    in.session = session_;
    in.last_solve = last_solve_;
    in.osc_config = osc_config;
    in.body_state_stable = body_state_stable;
    in.body_calibration_valid = BodyCalibrationLooksValid(calibration.body);
    in.floor_calibration_valid = FloorPlaneUsable(calibration.floor);
    in.body_signature = SteamVrBodyCalibrationSignature(calibration.body);
    in.floor_signature = SteamVrFloorCalibrationSignature(calibration);
    in.stale = SteamVrAlignmentStale(osc_config, calibration);
    in.last_alignment_timestamp = last_alignment_timestamp_;
    return BuildSteamVrAlignmentStatus(in);
}

LatencyProbeSession SteamVrAlignmentManager::StartLatencyProbe() {
    std::scoped_lock lock(mutex_);
    latency_probe_ = bt::StartLatencyProbe(last_snapshot_);
    return latency_probe_;
}

void SteamVrAlignmentManager::RecordLatencyProbeSample(const LatencyProbeSample& sample) {
    std::scoped_lock lock(mutex_);
    bt::RecordLatencyProbeSample(latency_probe_, sample);
}

LatencyProbeSession SteamVrAlignmentManager::FinishLatencyProbe() {
    std::scoped_lock lock(mutex_);
    bt::FinishLatencyProbe(latency_probe_);
    return latency_probe_;
}

LatencyProbeSession SteamVrAlignmentManager::LatencyProbeStatus() const {
    std::scoped_lock lock(mutex_);
    return latency_probe_;
}

bool SteamVrAlignmentManager::ActiveTransformIsHonest(const OscConfig& osc_config, bool stale) {
    if (!osc_config.tracker_space_transform_valid ||
        !TrackerSpaceTransformFinite(
            osc_config.tracker_space_position_offset,
            osc_config.tracker_space_rotation,
            osc_config.tracker_space_scale,
            osc_config.tracker_space_role_offsets)) {
        return false;
    }

    if (osc_config.tracker_space_source == "steamvr_controller_alignment") {
        return !stale;
    }
    if (osc_config.tracker_space_source == "steamvr_controller_alignment_stale") {
        return true;
    }
    return osc_config.tracker_space_source.empty() ||
        osc_config.tracker_space_source == "manual" ||
        osc_config.tracker_space_source == "manual_json_file";
}

} // namespace bt
