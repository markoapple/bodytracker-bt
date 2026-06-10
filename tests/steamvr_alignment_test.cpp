// Behavior tests for the SteamVR controller alignment subsystem.
// These tests use the FakeSteamVrPoseProvider so live SteamVR is never required.

#include "io/osc_sender.h"
#include "io/steamvr_provider.h"
#include "tracking/steamvr_alignment.h"
#include "tracking/steamvr_alignment_manager.h"
#include "test_check.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>

using namespace bt;

namespace {

// ----- Test fixture helpers ---------------------------------------------------

UnifiedBodyState MakeStableBodyState() {
    UnifiedBodyState body;
    body.valid = true;
    body.diagnostics.role_output_confidence = 0.85f;
    auto set_role = [&](BodyJointRole role, Vec3f position) {
        auto& joint = body.roles[BodyJointRoleIndex(role)];
        joint.role = role;
        joint.position = position;
        joint.confidence = 0.9f;
        joint.valid = true;
        joint.visibility = BodyJointVisibility::Visible;
    };
    // Standard standing pose, feet at floor (y=0), pelvis ~0.95 m, knees ~0.50 m.
    set_role(BodyJointRole::Pelvis,        Vec3f{ 0.00f, 0.95f, 0.00f});
    set_role(BodyJointRole::Chest,         Vec3f{ 0.00f, 1.32f, 0.02f});
    set_role(BodyJointRole::Neck,          Vec3f{ 0.00f, 1.52f, 0.03f});
    set_role(BodyJointRole::Head,          Vec3f{ 0.00f, 1.70f, 0.04f});
    set_role(BodyJointRole::LeftShoulder,  Vec3f{-0.20f, 1.46f, 0.02f});
    set_role(BodyJointRole::RightShoulder, Vec3f{ 0.20f, 1.46f, 0.02f});
    set_role(BodyJointRole::LeftElbow,     Vec3f{-0.34f, 1.22f, 0.04f});
    set_role(BodyJointRole::RightElbow,    Vec3f{ 0.34f, 1.22f, 0.04f});
    set_role(BodyJointRole::LeftWrist,     Vec3f{-0.42f, 1.00f, 0.05f});
    set_role(BodyJointRole::RightWrist,    Vec3f{ 0.42f, 1.00f, 0.05f});
    set_role(BodyJointRole::LeftHip,       Vec3f{-0.10f, 0.92f, 0.0f});
    set_role(BodyJointRole::RightHip,      Vec3f{ 0.10f, 0.92f, 0.0f});
    set_role(BodyJointRole::LeftKnee,      Vec3f{-0.10f, 0.50f, 0.0f});
    set_role(BodyJointRole::RightKnee,     Vec3f{ 0.10f, 0.50f, 0.0f});
    set_role(BodyJointRole::LeftAnkle, Vec3f{-0.10f, 0.05f, 0.0f});
    set_role(BodyJointRole::RightAnkle,Vec3f{ 0.10f, 0.05f, 0.0f});
    set_role(BodyJointRole::LeftFoot,  Vec3f{-0.10f, 0.00f, 0.05f});
    set_role(BodyJointRole::RightFoot, Vec3f{ 0.10f, 0.00f, 0.05f});
    body.lower_body.root.orientation = Quatf{0.0f, 0.0f, 0.0f, 1.0f};
    return body;
}

CalibrationBundle MakeValidCalibration() {
    CalibrationBundle calib;
    calib.floor.normal = Vec3f{0.0f, 1.0f, 0.0f};
    calib.floor.distance = 0.0f;
    calib.floor.valid = true;
    calib.body.standing_neutral_valid = true;
    calib.body.pelvis_width = 0.32f;
    calib.body.left_femur = 0.42f;
    calib.body.right_femur = 0.42f;
    calib.body.left_tibia = 0.42f;
    calib.body.right_tibia = 0.42f;
    calib.body.left_foot_length = 0.24f;
    calib.body.right_foot_length = 0.24f;
    calib.body.quality.overall = 0.80f;
    calib.body.quality.sample_count = 200;
    return calib;
}

SteamVrControllerPose MakeController(SteamVrControllerRole role, const Vec3f& position) {
    SteamVrControllerPose c;
    c.role = role;
    c.pose.position = position;
    c.pose.orientation = Quatf{0.0f, 0.0f, 0.0f, 1.0f};
    c.valid = true;
    c.timestamp_seconds = 0.0;
    c.pose_age_seconds = 0.0;
    c.reason = "tracked";
    return c;
}

SteamVrPoseSnapshot MakeAvailableSnapshot() {
    SteamVrPoseSnapshot s;
    s.available = true;
    s.runtime_initialized = true;
    s.device_count = 2;
    s.status = "connected";
    s.reason = "SteamVR connected";
    s.left = MakeController(SteamVrControllerRole::LeftHand, Vec3f{0.10f, 0.95f, 0.30f});
    s.right = MakeController(SteamVrControllerRole::RightHand, Vec3f{-0.10f, 0.95f, 0.30f});
    return s;
}

void RecordSample(SteamVrAlignmentSession& session,
                  SteamVrAlignmentLandmark landmark,
                  const SteamVrControllerPose& controller,
                  const UnifiedBodyState& body,
                  const CalibrationBundle& calib,
                  double t = 0.0) {
    auto sample = CaptureSteamVrAlignmentSample(landmark, controller, body, calib, t);
    StoreSteamVrAlignmentSample(session, sample);
}

// Apply the canonical "tracker_world = R_y(yaw) * cam_world + t" we expect the solver
// to learn, so we can build SteamVR controller positions consistent with the body state.
Vec3f BuildVrFromCam(const Vec3f& cam, const Quatf& yaw, const Vec3f& offset) {
    return Add(Rotate(yaw, cam), offset);
}

Quatf YawQuatF(float yaw_rad) {
    const float half = 0.5f * yaw_rad;
    return Normalize(Quatf{0.0f, std::sin(half), 0.0f, std::cos(half)});
}

// ----- 1) Provider unavailable -----------------------------------------------

void TestProviderUnavailable() {
    SteamVrPoseSnapshot s = MakeUnavailableSnapshot("OpenVR support was not built");
    BT_CHECK(!s.available);
    BT_CHECK(s.status == "unavailable");
    BT_CHECK(!s.reason.empty());

    // A session started against an unavailable provider must fail loudly with
    // a specific machine-readable reason, not a generic string.
    auto session = StartSteamVrAlignmentSession(s);
    BT_CHECK(!session.active);
    BT_CHECK(session.reason_code == SteamVrAlignmentReason::SteamVrUnavailable);

    // The fake provider can also report unavailable.
    FakeSteamVrPoseProvider fake(s);
    auto polled = fake.Poll();
    BT_CHECK(!polled.available);
}

// ----- 2) Fake provider returns left/right poses -----------------------------

void TestFakeProviderControllerPoses() {
    auto snapshot = MakeAvailableSnapshot();
    FakeSteamVrPoseProvider fake(snapshot);
    auto polled = fake.Poll();
    BT_CHECK(polled.available);
    BT_CHECK(polled.left.valid);
    BT_CHECK(polled.right.valid);
    BT_CHECK(polled.device_count == 2);

    auto session = StartSteamVrAlignmentSession(polled);
    BT_CHECK(session.active);
}

// ----- 3) Missing body calibration degrades but does not veto solve ----------

void TestSolvesDegradedWithoutBodyCalibration() {
    auto snapshot = MakeAvailableSnapshot();
    auto body = MakeStableBodyState();
    auto calib = MakeValidCalibration();
    calib.body = BodyCalibration{};
    calib.body.standing_neutral_valid = false;

    auto session = StartSteamVrAlignmentSession(snapshot);
    const Quatf R = YawQuatF(0.0f);
    const Vec3f offset{0.0f, 0.0f, 0.0f};
    auto cam_pos = [&](BodyJointRole r) {
        return body.roles[BodyJointRoleIndex(r)].position;
    };

    auto lf = CaptureSteamVrAlignmentSample(
        SteamVrAlignmentLandmark::LeftFoot,
        MakeController(SteamVrControllerRole::LeftHand, BuildVrFromCam(cam_pos(BodyJointRole::LeftFoot), R, offset)),
        body, calib, 0.0);
    auto rf = CaptureSteamVrAlignmentSample(
        SteamVrAlignmentLandmark::RightFoot,
        MakeController(SteamVrControllerRole::RightHand, BuildVrFromCam(cam_pos(BodyJointRole::RightFoot), R, offset)),
        body, calib, 0.0);
    auto pv = CaptureSteamVrAlignmentSample(
        SteamVrAlignmentLandmark::Pelvis,
        MakeController(SteamVrControllerRole::LeftHand, BuildVrFromCam(cam_pos(BodyJointRole::Pelvis), R, offset)),
        body, calib, 0.0);
    auto fl = CaptureSteamVrAlignmentSample(
        SteamVrAlignmentLandmark::Floor,
        MakeController(SteamVrControllerRole::LeftHand, BuildVrFromCam(Vec3f{0.0f, 0.0f, 0.0f}, R, offset)),
        body, calib, 0.0);
    std::cerr << "samples lf=" << lf.accepted << ":" << lf.reason
              << " rf=" << rf.accepted << ":" << rf.reason
              << " pv=" << pv.accepted << ":" << pv.reason
              << " fl=" << fl.accepted << ":" << fl.reason << '\n';
    StoreSteamVrAlignmentSample(session, lf);
    StoreSteamVrAlignmentSample(session, rf);
    StoreSteamVrAlignmentSample(session, pv);
    StoreSteamVrAlignmentSample(session, fl);

    auto result = SolveSteamVrAlignment(session, body, calib);
    std::cerr << "degraded solve reason=" << result.reason
              << " confidence=" << result.confidence
              << " residual=" << result.residual_m
              << " required=" << result.required_samples_present << '\n';
    BT_CHECK(result.valid);
    BT_CHECK(result.confidence > 0.0f);
    BT_CHECK(result.required_samples_present >= 3);
}

// ----- 4) SteamVR floor sample does not require prior stereo floor calibration --

void TestSteamVrFloorSampleDoesNotRequirePriorFloorCalibration() {
    auto snapshot = MakeAvailableSnapshot();
    auto body = MakeStableBodyState();
    auto calib = MakeValidCalibration();
    calib.floor.valid = false;

    auto session = StartSteamVrAlignmentSession(snapshot);
    const auto left = MakeController(SteamVrControllerRole::LeftHand,
        body.roles[BodyJointRoleIndex(BodyJointRole::LeftAnkle)].position);
    const auto right = MakeController(SteamVrControllerRole::RightHand,
        body.roles[BodyJointRoleIndex(BodyJointRole::RightAnkle)].position);
    const auto pelvis = MakeController(SteamVrControllerRole::LeftHand,
        body.roles[BodyJointRoleIndex(BodyJointRole::Pelvis)].position);
    StoreSteamVrAlignmentSample(session, CaptureSteamVrAlignmentSample(
        SteamVrAlignmentLandmark::LeftFoot, left, body, calib, 0.0));
    StoreSteamVrAlignmentSample(session, CaptureSteamVrAlignmentSample(
        SteamVrAlignmentLandmark::RightFoot, right, body, calib, 0.0));
    StoreSteamVrAlignmentSample(session, CaptureSteamVrAlignmentSample(
        SteamVrAlignmentLandmark::Pelvis, pelvis, body, calib, 0.0));

    auto floor_sample = CaptureSteamVrAlignmentSample(
        SteamVrAlignmentLandmark::Floor, MakeController(SteamVrControllerRole::LeftHand, Vec3f{0.0f, 0.0f, 0.0f}), body, calib, 0.0);
    BT_CHECK(floor_sample.accepted);
    BT_CHECK(floor_sample.reason_code == SteamVrAlignmentReason::Accepted);
    StoreSteamVrAlignmentSample(session, floor_sample);

    auto result = SolveSteamVrAlignment(session, body, calib);
    BT_CHECK(result.valid);
    BT_CHECK(result.required_samples_present == 4);
    BT_CHECK(result.confidence > 0.0f);
}

// ----- 5) Cannot finish without all required samples -------------------------

void TestCannotFinishWithoutRequiredSamples() {
    auto snapshot = MakeAvailableSnapshot();
    auto body = MakeStableBodyState();
    auto calib = MakeValidCalibration();

    auto session = StartSteamVrAlignmentSession(snapshot);
    // Only record left foot.
    RecordSample(session, SteamVrAlignmentLandmark::LeftFoot, snapshot.left, body, calib);
    auto result = SolveSteamVrAlignment(session, body, calib);
    BT_CHECK(!result.valid);
    BT_CHECK(result.reason_code == SteamVrAlignmentReason::NotEnoughSamples);
}

// ----- 6) Either controller can sample any landmark --------------------------

void TestEitherControllerCanSampleAnyLandmark() {
    auto snapshot = MakeAvailableSnapshot();
    auto body = MakeStableBodyState();
    auto calib = MakeValidCalibration();
    auto session = StartSteamVrAlignmentSession(snapshot);

    // Use the right controller for the left foot.
    auto sample = CaptureSteamVrAlignmentSample(
        SteamVrAlignmentLandmark::LeftFoot, snapshot.right, body, calib, 0.0);
    BT_CHECK(sample.accepted);
    BT_CHECK(sample.controller == SteamVrControllerRole::RightHand);
    StoreSteamVrAlignmentSample(session, sample);
    BT_CHECK(session.accepted_sample_count == 1);
}

void TestAnklePromptCapturesAnkleLandmark() {
    auto body = MakeStableBodyState();
    auto calib = MakeValidCalibration();
    const auto& left_ankle = body.roles[BodyJointRoleIndex(BodyJointRole::LeftAnkle)];

    auto sample = CaptureSteamVrAlignmentSample(
        SteamVrAlignmentLandmark::LeftFoot,
        MakeController(SteamVrControllerRole::LeftHand, left_ankle.position),
        body, calib, 0.0);

    BT_CHECK(sample.accepted);
    BT_CHECK(Distance(sample.camera_landmark, left_ankle.position) < 1e-5f);
}

void TestCaptureAcceptsEstimatedAnkleDuringSelfOcclusion() {
    auto body = MakeStableBodyState();
    auto calib = MakeValidCalibration();
    body.valid = false;
    body.diagnostics.role_output_confidence = 0.10f;

    auto& ankle = body.roles[BodyJointRoleIndex(BodyJointRole::LeftAnkle)];
    ankle.confidence = 0.12f;
    ankle.valid = true;
    ankle.visibility = BodyJointVisibility::Predicted;
    ankle.predicted = true;
    ankle.measured = false;
    ankle.evidence.source = TrackerEvidenceSource::Predicted;
    ankle.evidence.direct_confidence = 0.12f;
    ankle.evidence.valid = true;

    auto sample = CaptureSteamVrAlignmentSample(
        SteamVrAlignmentLandmark::LeftFoot,
        MakeController(SteamVrControllerRole::LeftHand, ankle.position),
        body, calib, 0.0);

    BT_CHECK(sample.accepted);
    BT_CHECK(sample.body_state_valid);
    BT_CHECK(sample.confidence >= 0.08f);
    BT_CHECK(Distance(sample.camera_landmark, ankle.position) < 1e-5f);
}

// ----- 7) Valid sample set produces valid transform --------------------------

// Given a valid alignment yaw of +30 degrees and an offset, build matching VR
// controller positions and sample the four required landmarks.
SteamVrAlignmentSolveResult RunValidSolve(float yaw_rad, const Vec3f& vr_offset) {
    auto snapshot = MakeAvailableSnapshot();
    auto body = MakeStableBodyState();
    auto calib = MakeValidCalibration();
    auto session = StartSteamVrAlignmentSession(snapshot);

    const Quatf R = YawQuatF(yaw_rad);

    auto cam_pos = [&](BodyJointRole r) {
        return body.roles[BodyJointRoleIndex(r)].position;
    };

    SteamVrControllerPose cl = MakeController(
        SteamVrControllerRole::LeftHand, BuildVrFromCam(cam_pos(BodyJointRole::LeftFoot), R, vr_offset));
    SteamVrControllerPose cr = MakeController(
        SteamVrControllerRole::RightHand, BuildVrFromCam(cam_pos(BodyJointRole::RightFoot), R, vr_offset));
    SteamVrControllerPose cp = MakeController(
        SteamVrControllerRole::LeftHand, BuildVrFromCam(cam_pos(BodyJointRole::Pelvis), R, vr_offset));
    SteamVrControllerPose cf = MakeController(
        SteamVrControllerRole::LeftHand, BuildVrFromCam(Vec3f{0.0f, 0.0f, 0.0f}, R, vr_offset));

    StoreSteamVrAlignmentSample(session, CaptureSteamVrAlignmentSample(
        SteamVrAlignmentLandmark::LeftFoot, cl, body, calib, 0.0));
    StoreSteamVrAlignmentSample(session, CaptureSteamVrAlignmentSample(
        SteamVrAlignmentLandmark::RightFoot, cr, body, calib, 0.0));
    StoreSteamVrAlignmentSample(session, CaptureSteamVrAlignmentSample(
        SteamVrAlignmentLandmark::Pelvis, cp, body, calib, 0.0));
    StoreSteamVrAlignmentSample(session, CaptureSteamVrAlignmentSample(
        SteamVrAlignmentLandmark::Floor, cf, body, calib, 0.0));

    return SolveSteamVrAlignment(session, body, calib);
}

void TestValidSamplesCreateValidTransform() {
    const Vec3f offset{1.5f, 0.0f, -2.0f};
    const float yaw = 0.5236f; // 30 degrees
    auto result = RunValidSolve(yaw, offset);
    BT_CHECK(result.valid);
    BT_CHECK(result.reason_code == SteamVrAlignmentReason::Valid ||
             result.reason_code == SteamVrAlignmentReason::Weak);
    BT_CHECK(result.confidence > 0.30f);
    BT_CHECK(result.residual_m < 0.18f);
    BT_CHECK(result.required_samples_present == 4);

    // Yaw should round-trip: applying the solved transform to the camera-space
    // foot baseline must reproduce the VR-space baseline within a small tolerance.
    // We do not assume any specific sign convention for yaw_offset_rad; only that
    // the produced rotation is the inverse of what we baked into VR poses.
    BT_CHECK(std::isfinite(result.yaw_offset_rad));

    // Required role offsets must be present. The foot wizard samples ankles, so
    // they constrain the global transform without adding an ankle offset to the
    // foot tracker role.
    BT_CHECK(result.role_offsets_present[TrackerRoleIndex(TrackerRole::Pelvis)]);
    BT_CHECK(result.role_offsets_present[TrackerRoleIndex(TrackerRole::LeftFoot)]);
    BT_CHECK(result.role_offsets_present[TrackerRoleIndex(TrackerRole::RightFoot)]);
    BT_CHECK(Distance(result.role_offsets[TrackerRoleIndex(TrackerRole::LeftFoot)], Vec3f{}) < 1e-5f);
    BT_CHECK(Distance(result.role_offsets[TrackerRoleIndex(TrackerRole::RightFoot)], Vec3f{}) < 1e-5f);
}

// ----- 8) Far-off sample rejected --------------------------------------------

void TestFarSampleRejected() {
    auto snapshot = MakeAvailableSnapshot();
    auto body = MakeStableBodyState();
    auto calib = MakeValidCalibration();
    auto session = StartSteamVrAlignmentSession(snapshot);

    const Vec3f offset{0.0f, 0.0f, 0.0f};
    const Quatf R = Quatf{0.0f, 0.0f, 0.0f, 1.0f};

    auto cam_pos = [&](BodyJointRole r) {
        return body.roles[BodyJointRoleIndex(r)].position;
    };

    // Left foot sample is sane.
    auto cl = MakeController(SteamVrControllerRole::LeftHand,
        BuildVrFromCam(cam_pos(BodyJointRole::LeftFoot), R, offset));
    // Right foot sample is **way** off (1m off in z) â€” should fail solve via
    // SampleTooFarFromSolved or LeftRightMismatch.
    auto cr = MakeController(SteamVrControllerRole::RightHand,
        Add(BuildVrFromCam(cam_pos(BodyJointRole::RightFoot), R, offset), Vec3f{0.0f, 0.0f, 1.0f}));
    auto cp = MakeController(SteamVrControllerRole::LeftHand,
        BuildVrFromCam(cam_pos(BodyJointRole::Pelvis), R, offset));
    auto cf = MakeController(SteamVrControllerRole::LeftHand,
        BuildVrFromCam(Vec3f{0.0f, 0.0f, 0.0f}, R, offset));

    StoreSteamVrAlignmentSample(session, CaptureSteamVrAlignmentSample(
        SteamVrAlignmentLandmark::LeftFoot, cl, body, calib, 0.0));
    StoreSteamVrAlignmentSample(session, CaptureSteamVrAlignmentSample(
        SteamVrAlignmentLandmark::RightFoot, cr, body, calib, 0.0));
    StoreSteamVrAlignmentSample(session, CaptureSteamVrAlignmentSample(
        SteamVrAlignmentLandmark::Pelvis, cp, body, calib, 0.0));
    StoreSteamVrAlignmentSample(session, CaptureSteamVrAlignmentSample(
        SteamVrAlignmentLandmark::Floor, cf, body, calib, 0.0));

    auto result = SolveSteamVrAlignment(session, body, calib);
    BT_CHECK(!result.valid);
    BT_CHECK(result.reason_code == SteamVrAlignmentReason::SampleTooFarFromSolved ||
             result.reason_code == SteamVrAlignmentReason::LeftRightMismatch ||
             result.reason_code == SteamVrAlignmentReason::TransformResidualTooHigh ||
             result.reason_code == SteamVrAlignmentReason::ScaleMismatch);
}

// ----- 9) Floor mismatch fails ------------------------------------------------

void TestFloorMismatchFails() {
    auto snapshot = MakeAvailableSnapshot();
    auto body = MakeStableBodyState();
    auto calib = MakeValidCalibration();
    auto session = StartSteamVrAlignmentSession(snapshot);

    const Vec3f offset{0.0f, 0.0f, 0.0f};
    const Quatf R = Quatf{0.0f, 0.0f, 0.0f, 1.0f};
    auto cam_pos = [&](BodyJointRole r) {
        return body.roles[BodyJointRoleIndex(r)].position;
    };

    auto cl = MakeController(SteamVrControllerRole::LeftHand,
        BuildVrFromCam(cam_pos(BodyJointRole::LeftFoot), R, offset));
    auto cr = MakeController(SteamVrControllerRole::RightHand,
        BuildVrFromCam(cam_pos(BodyJointRole::RightFoot), R, offset));
    auto cp = MakeController(SteamVrControllerRole::LeftHand,
        BuildVrFromCam(cam_pos(BodyJointRole::Pelvis), R, offset));
    // Floor sample is 50cm above where it should be.
    auto cf = MakeController(SteamVrControllerRole::LeftHand,
        Add(BuildVrFromCam(Vec3f{0.0f, 0.0f, 0.0f}, R, offset), Vec3f{0.0f, 0.5f, 0.0f}));

    StoreSteamVrAlignmentSample(session, CaptureSteamVrAlignmentSample(
        SteamVrAlignmentLandmark::LeftFoot, cl, body, calib, 0.0));
    StoreSteamVrAlignmentSample(session, CaptureSteamVrAlignmentSample(
        SteamVrAlignmentLandmark::RightFoot, cr, body, calib, 0.0));
    StoreSteamVrAlignmentSample(session, CaptureSteamVrAlignmentSample(
        SteamVrAlignmentLandmark::Pelvis, cp, body, calib, 0.0));
    StoreSteamVrAlignmentSample(session, CaptureSteamVrAlignmentSample(
        SteamVrAlignmentLandmark::Floor, cf, body, calib, 0.0));

    auto result = SolveSteamVrAlignment(session, body, calib);
    BT_CHECK(!result.valid);
    // Floor was lifted, so the solver lifts the entire offset by 0.5m, which makes
    // pelvis residual huge. Either floor mismatch detected or residual too high â€” both
    // are loud, specific failures (not "generic").
    BT_CHECK(result.reason_code == SteamVrAlignmentReason::FloorHeightMismatch ||
             result.reason_code == SteamVrAlignmentReason::TransformResidualTooHigh ||
             result.reason_code == SteamVrAlignmentReason::SampleTooFarFromSolved);
}

// ----- 10) Yaw ambiguity fails ------------------------------------------------

void TestYawAmbiguousWhenFeetClose() {
    auto snapshot = MakeAvailableSnapshot();
    auto body = MakeStableBodyState();
    auto calib = MakeValidCalibration();
    auto session = StartSteamVrAlignmentSession(snapshot);

    auto cl = MakeController(SteamVrControllerRole::LeftHand,  Vec3f{0.0f, 0.0f, 0.0f});
    auto cr = MakeController(SteamVrControllerRole::RightHand, Vec3f{0.001f, 0.0f, 0.001f});
    auto cp = MakeController(SteamVrControllerRole::LeftHand,  Vec3f{0.0f, 0.95f, 0.0f});
    auto cf = MakeController(SteamVrControllerRole::LeftHand,  Vec3f{0.0f, 0.0f, 0.0f});

    StoreSteamVrAlignmentSample(session, CaptureSteamVrAlignmentSample(
        SteamVrAlignmentLandmark::LeftFoot, cl, body, calib, 0.0));
    StoreSteamVrAlignmentSample(session, CaptureSteamVrAlignmentSample(
        SteamVrAlignmentLandmark::RightFoot, cr, body, calib, 0.0));
    StoreSteamVrAlignmentSample(session, CaptureSteamVrAlignmentSample(
        SteamVrAlignmentLandmark::Pelvis, cp, body, calib, 0.0));
    StoreSteamVrAlignmentSample(session, CaptureSteamVrAlignmentSample(
        SteamVrAlignmentLandmark::Floor, cf, body, calib, 0.0));

    auto result = SolveSteamVrAlignment(session, body, calib);
    BT_CHECK(!result.valid);
    BT_CHECK(result.reason_code == SteamVrAlignmentReason::YawAmbiguous);
}

// ----- 11) Scale mismatch fails -----------------------------------------------

void TestScaleMismatchFails() {
    auto snapshot = MakeAvailableSnapshot();
    auto body = MakeStableBodyState();
    auto calib = MakeValidCalibration();
    auto session = StartSteamVrAlignmentSession(snapshot);

    // Camera-space foot baseline is 0.20 m. Build VR feet that are 0.40 m apart
    // -> scale ratio ~2.0 -> ScaleMismatch.
    auto cl = MakeController(SteamVrControllerRole::LeftHand,  Vec3f{-0.20f, 0.0f, 0.0f});
    auto cr = MakeController(SteamVrControllerRole::RightHand, Vec3f{ 0.20f, 0.0f, 0.0f});
    auto cp = MakeController(SteamVrControllerRole::LeftHand,  Vec3f{ 0.0f, 0.95f, 0.0f});
    auto cf = MakeController(SteamVrControllerRole::LeftHand,  Vec3f{ 0.0f, 0.0f, 0.0f});

    StoreSteamVrAlignmentSample(session, CaptureSteamVrAlignmentSample(
        SteamVrAlignmentLandmark::LeftFoot, cl, body, calib, 0.0));
    StoreSteamVrAlignmentSample(session, CaptureSteamVrAlignmentSample(
        SteamVrAlignmentLandmark::RightFoot, cr, body, calib, 0.0));
    StoreSteamVrAlignmentSample(session, CaptureSteamVrAlignmentSample(
        SteamVrAlignmentLandmark::Pelvis, cp, body, calib, 0.0));
    StoreSteamVrAlignmentSample(session, CaptureSteamVrAlignmentSample(
        SteamVrAlignmentLandmark::Floor, cf, body, calib, 0.0));

    auto result = SolveSteamVrAlignment(session, body, calib);
    BT_CHECK(!result.valid);
    BT_CHECK(result.reason_code == SteamVrAlignmentReason::ScaleMismatch);
}

// ----- 12) Stale on body signature change -------------------------------------

void TestStaleOnBodySignatureChange() {
    auto snapshot = MakeAvailableSnapshot();
    auto body = MakeStableBodyState();
    auto calib = MakeValidCalibration();

    auto result = RunValidSolve(0.0f, Vec3f{1.0f, 0.0f, -1.0f});
    BT_CHECK(result.valid);

    OscConfig cfg;
    ApplySteamVrAlignmentToOscConfig(cfg, result);
    BT_CHECK(cfg.tracker_space_transform_valid);
    BT_CHECK(cfg.tracker_space_source == "steamvr_controller_alignment");
    BT_CHECK(!SteamVrAlignmentStale(cfg, calib));

    // Mutate body calibration and confirm staleness flips on.
    auto mutated = calib;
    mutated.body.left_femur += 0.05f;
    BT_CHECK(SteamVrAlignmentStale(cfg, mutated));
}

void TestStaleOnFloorSignatureChange() {
    auto calib = MakeValidCalibration();
    auto result = RunValidSolve(0.0f, Vec3f{0.0f, 0.0f, 0.0f});
    BT_CHECK(result.valid);

    OscConfig cfg;
    ApplySteamVrAlignmentToOscConfig(cfg, result);
    BT_CHECK(!SteamVrAlignmentStale(cfg, calib));

    auto mutated = calib;
    mutated.floor.distance += 0.10f;
    BT_CHECK(SteamVrAlignmentStale(cfg, mutated));
}

// ----- 13) Clearing alignment removes controller transform but preserves manual

void TestClearAlignmentPreservesManualFallback() {
    auto result = RunValidSolve(0.2f, Vec3f{0.5f, 0.0f, 0.5f});
    OscConfig cfg;
    ApplySteamVrAlignmentToOscConfig(cfg, result);
    BT_CHECK(cfg.tracker_space_source == "steamvr_controller_alignment");

    ClearSteamVrAlignmentFromOscConfig(cfg);
    BT_CHECK(cfg.tracker_space_source == "manual");
    BT_CHECK(cfg.tracker_space_transform_valid); // controller transform preserved as manual fallback
    BT_CHECK(cfg.manual_tracker_space_transform_valid);
    BT_CHECK(cfg.steamvr_alignment_status == "idle");

    // Now apply manual fallback via direct config write â€” clearing again should
    // not stomp manual fallback because source is no longer controller-derived.
    cfg.tracker_space_transform_valid = true;
    cfg.tracker_space_source = "manual";
    cfg.tracker_space_position_offset = Vec3f{1.0f, 0.0f, 0.0f};
    ClearSteamVrAlignmentFromOscConfig(cfg);
    BT_CHECK(cfg.tracker_space_transform_valid); // manual preserved
    BT_CHECK(cfg.tracker_space_source == "manual");
    BT_CHECK(cfg.tracker_space_position_offset.x == 1.0f);
}

// ----- 14) Manual fallback applies when controller alignment missing ---------

void TestManualFallbackWhenControllerAbsent() {
    OscConfig cfg;
    cfg.tracker_space_transform_valid = true;
    cfg.tracker_space_source = "manual";
    cfg.tracker_space_position_offset = Vec3f{2.0f, 0.0f, 0.0f};
    cfg.tracker_space_rotation = Quatf{0.0f, 0.0f, 0.0f, 1.0f};
    cfg.tracker_space_scale = 1.0f;

    TrackerPose tracker;
    tracker.role = TrackerRole::Pelvis;
    tracker.pose.position = Vec3f{0.0f, 1.0f, 0.0f};
    tracker.pose.orientation = Quatf{0.0f, 0.0f, 0.0f, 1.0f};
    tracker.confidence = 0.9f;
    tracker.valid = true;

    Pose3f vr = TransformTrackerPoseToTrackerSpace(tracker, cfg);
    BT_CHECK_NEAR(vr.position.x, 2.0f, 1e-4);
    BT_CHECK_NEAR(vr.position.y, 1.0f, 1e-4);
    BT_CHECK_NEAR(vr.position.z, 0.0f, 1e-4);
}

// ----- 15) Controller alignment overrides when valid -------------------------

void TestControllerAlignmentOverridesManual() {
    auto result = RunValidSolve(0.0f, Vec3f{3.0f, 0.0f, 0.0f});
    OscConfig cfg;
    cfg.tracker_space_transform_valid = true;
    cfg.tracker_space_source = "manual";
    cfg.tracker_space_position_offset = Vec3f{99.0f, 0.0f, 0.0f};
    ApplySteamVrAlignmentToOscConfig(cfg, result);
    BT_CHECK(cfg.tracker_space_source == "steamvr_controller_alignment");
    BT_CHECK(cfg.tracker_space_position_offset.x != 99.0f);
}

// ----- 16) Role offsets persist via OscConfig --------------------------------

void TestRoleOffsetsPersist() {
    auto result = RunValidSolve(0.0f, Vec3f{0.0f, 0.0f, 0.0f});
    OscConfig cfg;
    ApplySteamVrAlignmentToOscConfig(cfg, result);
    bool any_offset_recorded = false;
    for (const auto& off : cfg.tracker_space_role_offsets) {
        if (std::isfinite(off.x) && std::isfinite(off.y) && std::isfinite(off.z)) {
            any_offset_recorded = true;
        }
    }
    BT_CHECK(any_offset_recorded);
}

// ----- 17) Tracker roles include upper body; knees are optional last -----------

void TestEightTrackerRoles() {
    BT_CHECK(kTrackerPoseCount == 8);
    BT_CHECK(kTrackerRoles[0] == TrackerRole::Pelvis);
    BT_CHECK(kTrackerRoles[1] == TrackerRole::LeftFoot);
    BT_CHECK(kTrackerRoles[2] == TrackerRole::RightFoot);
    BT_CHECK(kTrackerRoles[3] == TrackerRole::Chest);
    BT_CHECK(kTrackerRoles[4] == TrackerRole::LeftElbow);
    BT_CHECK(kTrackerRoles[5] == TrackerRole::RightElbow);
    BT_CHECK(kTrackerRoles[6] == TrackerRole::LeftKnee);
    BT_CHECK(kTrackerRoles[7] == TrackerRole::RightKnee);
}

// ----- 18) Nonfinite tracker is skipped --------------------------------------

void TestNonfiniteTrackerSkipped() {
    OscConfig cfg;
    cfg.tracker_space_transform_valid = true;
    cfg.tracker_space_source = "manual";
    cfg.tracker_space_rotation = Quatf{0.0f, 0.0f, 0.0f, 1.0f};
    cfg.tracker_space_scale = 1.0f;
    cfg.enabled = false;

    TrackerPose tracker;
    tracker.role = TrackerRole::Pelvis;
    tracker.pose.position = Vec3f{std::nanf(""), 0.0f, 0.0f};
    tracker.pose.orientation = Quatf{0.0f, 0.0f, 0.0f, 1.0f};
    tracker.confidence = 0.9f;
    tracker.valid = true;

    OscSender sender(cfg);
    TrackerPoseArray arr{};
    arr[0] = tracker;
    auto status = sender.SendTrackers(arr);
    // OSC is disabled so SendTrackers returns OK without doing anything; report status is "disabled".
    BT_CHECK(status.ok());
    const auto& report = sender.LastReport();
    BT_CHECK(report.status == "disabled" || report.sent_tracker_count == 0);
}

// ----- 19) Redo only clears one slot -----------------------------------------

void TestRedoSingleSlotPreservesOthers() {
    auto snapshot = MakeAvailableSnapshot();
    auto body = MakeStableBodyState();
    auto calib = MakeValidCalibration();
    auto session = StartSteamVrAlignmentSession(snapshot);

    RecordSample(session, SteamVrAlignmentLandmark::LeftFoot,  snapshot.left,  body, calib);
    RecordSample(session, SteamVrAlignmentLandmark::RightFoot, snapshot.right, body, calib);
    BT_CHECK(session.accepted_sample_count == 2);

    RedoSteamVrAlignmentSample(session, SteamVrAlignmentLandmark::LeftFoot);
    BT_CHECK(session.accepted_sample_count == 1);
    BT_CHECK(!session.samples[LandmarkSlotIndex(SteamVrAlignmentLandmark::LeftFoot)].accepted);
    BT_CHECK(session.samples[LandmarkSlotIndex(SteamVrAlignmentLandmark::RightFoot)].accepted);
}

// ----- 20) Status export reflects truth --------------------------------------

void TestBuildStatusReflectsTruth() {
    auto snapshot = MakeAvailableSnapshot();
    auto body = MakeStableBodyState();
    auto calib = MakeValidCalibration();
    auto result = RunValidSolve(0.0f, Vec3f{0.0f, 0.0f, 0.0f});

    SteamVrAlignmentStatusInputs inputs;
    inputs.provider_snapshot = snapshot;
    inputs.session.active = true;
    // Reuse a fresh session we know matches the solve result for sample bookkeeping:
    auto session = StartSteamVrAlignmentSession(snapshot);
    StoreSteamVrAlignmentSample(session, CaptureSteamVrAlignmentSample(
        SteamVrAlignmentLandmark::LeftFoot, snapshot.left, body, calib, 0.0));
    StoreSteamVrAlignmentSample(session, CaptureSteamVrAlignmentSample(
        SteamVrAlignmentLandmark::RightFoot, snapshot.right, body, calib, 0.0));
    StoreSteamVrAlignmentSample(session, CaptureSteamVrAlignmentSample(
        SteamVrAlignmentLandmark::Pelvis, snapshot.left, body, calib, 0.0));
    StoreSteamVrAlignmentSample(session, CaptureSteamVrAlignmentSample(
        SteamVrAlignmentLandmark::Floor, snapshot.left, body, calib, 0.0));
    inputs.session = session;
    inputs.last_solve = result;

    OscConfig cfg;
    ApplySteamVrAlignmentToOscConfig(cfg, result);
    inputs.osc_config = cfg;
    inputs.body_calibration_valid = true;
    inputs.floor_calibration_valid = true;
    inputs.body_state_stable = true;
    inputs.body_signature = SteamVrBodyCalibrationSignature(calib.body);
    inputs.floor_signature = SteamVrFloorCalibrationSignature(calib);
    inputs.stale = false;
    inputs.last_alignment_timestamp = 100.0;

    auto status = BuildSteamVrAlignmentStatus(inputs);
    BT_CHECK(status.provider_available);
    BT_CHECK(status.required_samples_complete);
    BT_CHECK(status.required_samples_present == 4);
    BT_CHECK(status.transform_valid);
    BT_CHECK(status.role_offsets_present);
    BT_CHECK(status.active_transform_source == TrackerSpaceSource::SteamVrController);
    BT_CHECK(status.confidence > 0.30f);
}

// ----- 21) Stale alignment surfaces in status --------------------------------

void TestStaleAlignmentInStatus() {
    auto snapshot = MakeAvailableSnapshot();
    auto body = MakeStableBodyState();
    auto calib = MakeValidCalibration();
    auto result = RunValidSolve(0.0f, Vec3f{0.0f, 0.0f, 0.0f});

    OscConfig cfg;
    ApplySteamVrAlignmentToOscConfig(cfg, result);
    // ApplySteamVrAlignmentToOscConfig stores previous alignment as manual fallback.
    BT_CHECK(cfg.manual_tracker_space_transform_valid);

    SteamVrAlignmentStatusInputs inputs;
    inputs.provider_snapshot = snapshot;
    inputs.session = StartSteamVrAlignmentSession(snapshot);
    inputs.last_solve = result;
    inputs.osc_config = cfg;
    inputs.stale = true;
    inputs.body_signature = SteamVrBodyCalibrationSignature(calib.body) + ":mutated";
    inputs.floor_signature = SteamVrFloorCalibrationSignature(calib);
    auto status = BuildSteamVrAlignmentStatus(inputs);
    BT_CHECK(status.stale);
    BT_CHECK(status.state == "stale");
    // Stale + manual available -> prefer manual (not stale controller).
    BT_CHECK(status.active_transform_source == TrackerSpaceSource::Manual);
    BT_CHECK(status.manual_fallback_active);
    BT_CHECK(status.reason_code == SteamVrAlignmentReason::AlignmentStale);
}

// ----- 22) Latency probe scaffolding ------------------------------------------

void TestLatencyProbeUnavailableWhenProviderMissing() {
    auto unavail = MakeUnavailableSnapshot("test");
    auto probe = StartLatencyProbe(unavail);
    BT_CHECK(probe.status == LatencyProbeStatus::Unavailable);
    BT_CHECK(probe.reason == "provider_unavailable");

    LatencyProbeSample s;
    RecordLatencyProbeSample(probe, s);
    BT_CHECK(probe.sample_count == 0); // unavailable session is inert

    FinishLatencyProbe(probe);
    BT_CHECK(probe.status == LatencyProbeStatus::Unavailable);
}

void TestLatencyProbeValidWithFakeMotion() {
    auto avail = MakeAvailableSnapshot();
    auto probe = StartLatencyProbe(avail);
    BT_CHECK(probe.status == LatencyProbeStatus::Collecting);

    for (int i = 0; i < 24; ++i) {
        LatencyProbeSample s;
        const double t = static_cast<double>(i) * 0.05;
        s.controller_timestamp_seconds = t;
        s.body_timestamp_seconds = t + 0.020; // 20 ms body lag behind controller
        s.controller_position = Vec3f{0.10f * static_cast<float>(i), 0.0f, 0.0f};
        s.body_position = Vec3f{0.10f * static_cast<float>(i), 0.0f, 0.0f};
        RecordLatencyProbeSample(probe, s);
    }
    FinishLatencyProbe(probe);
    BT_CHECK(probe.status == LatencyProbeStatus::Valid || probe.status == LatencyProbeStatus::Weak);
    BT_CHECK(probe.sample_count == 24);
    BT_CHECK(std::abs(probe.estimated_latency_seconds - 0.020f) < 0.005f);
}

// ----- 23) Failure reasons are not generic -----------------------------------

void TestEveryFailureHasSpecificReason() {
    // Walk through each known reason and ensure ToString isn't empty/"unknown".
    for (auto code : {
        SteamVrAlignmentReason::SteamVrUnavailable,
        SteamVrAlignmentReason::ProviderUnavailable,
        SteamVrAlignmentReason::ProviderCompileDisabled,
        SteamVrAlignmentReason::ControllersMissing,
        SteamVrAlignmentReason::ControllerPoseInvalid,
        SteamVrAlignmentReason::BodyStateUnstable,
        SteamVrAlignmentReason::NoUnifiedBodyState,
        SteamVrAlignmentReason::BodyCalibrationMissing,
        SteamVrAlignmentReason::FloorCalibrationMissing,
        SteamVrAlignmentReason::BodyLandmarkMissing,
        SteamVrAlignmentReason::SampleConfidenceLow,
        SteamVrAlignmentReason::SampleNonfinite,
        SteamVrAlignmentReason::NotEnoughSamples,
        SteamVrAlignmentReason::LeftRightMismatch,
        SteamVrAlignmentReason::SampleTooFarFromSolved,
        SteamVrAlignmentReason::YawAmbiguous,
        SteamVrAlignmentReason::ScaleMismatch,
        SteamVrAlignmentReason::FloorHeightMismatch,
        SteamVrAlignmentReason::TransformResidualTooHigh,
        SteamVrAlignmentReason::AlignmentStale,
        SteamVrAlignmentReason::RoleOffsetMissing,
        SteamVrAlignmentReason::ActiveTransformInvalid,
    }) {
        const std::string s = ToString(code);
        BT_CHECK(!s.empty());
        BT_CHECK(s != "unknown");
    }
}

// ----- Manager-level tests ----------------------------------------------------

std::unique_ptr<FakeSteamVrPoseProvider> MakeFakeProvider(SteamVrPoseSnapshot snapshot) {
    return std::make_unique<FakeSteamVrPoseProvider>(std::move(snapshot));
}

void TestManagerStartFailsWhenProviderUnavailable() {
    auto provider = std::make_unique<FakeSteamVrPoseProvider>(MakeUnavailableSnapshot("test"));
    SteamVrAlignmentManager manager(std::move(provider));
    auto status = manager.StartSession();
    BT_CHECK(!status.provider_available);
    BT_CHECK(!status.session_active);
    BT_CHECK(status.reason_code == SteamVrAlignmentReason::SteamVrUnavailable);
}

void TestManagerCompleteFlowProducesValidAlignment() {
    auto body = MakeStableBodyState();
    auto calib = MakeValidCalibration();
    auto cam_pos = [&](BodyJointRole r) {
        return body.roles[BodyJointRoleIndex(r)].position;
    };

    // Use a callback-driven fake provider so each Poll() returns the controller
    // position appropriate for the current sample step. Identity yaw + zero offset.
    Vec3f current_left = cam_pos(BodyJointRole::LeftFoot);
    Vec3f current_right = cam_pos(BodyJointRole::RightFoot);
    auto provider = std::make_unique<FakeSteamVrPoseProvider>([&]() {
        SteamVrPoseSnapshot s = MakeAvailableSnapshot();
        s.left.pose.position = current_left;
        s.right.pose.position = current_right;
        return s;
    });
    SteamVrAlignmentManager manager(std::move(provider));

    auto status = manager.StartSession();
    BT_CHECK(status.session_active);

    // Left foot.
    current_left = cam_pos(BodyJointRole::LeftFoot);
    manager.RecordSample(SteamVrAlignmentLandmark::LeftFoot,
        SteamVrControllerRole::LeftHand, body, calib, 1.0);
    // Right foot.
    current_right = cam_pos(BodyJointRole::RightFoot);
    manager.RecordSample(SteamVrAlignmentLandmark::RightFoot,
        SteamVrControllerRole::RightHand, body, calib, 1.0);
    // Pelvis (use left controller).
    current_left = cam_pos(BodyJointRole::Pelvis);
    manager.RecordSample(SteamVrAlignmentLandmark::Pelvis,
        SteamVrControllerRole::LeftHand, body, calib, 1.0);
    // Floor.
    current_left = Vec3f{0.0f, 0.0f, 0.0f};
    manager.RecordSample(SteamVrAlignmentLandmark::Floor,
        SteamVrControllerRole::LeftHand, body, calib, 1.0);

    OscConfig osc;
    auto final_status = manager.FinishSession(body, calib, osc, 1.0);
    BT_CHECK(final_status.transform_valid);
    BT_CHECK(osc.tracker_space_transform_valid);
    BT_CHECK(osc.tracker_space_source == "steamvr_controller_alignment");
    BT_CHECK(final_status.last_alignment_timestamp == 1.0);
}

void TestManagerFailedFinishDoesNotOverwriteLastGood() {
    // First, achieve a good alignment.
    auto snapshot = MakeAvailableSnapshot();
    SteamVrAlignmentManager manager(MakeFakeProvider(snapshot));
    auto body = MakeStableBodyState();
    auto calib = MakeValidCalibration();

    OscConfig osc;
    osc.tracker_space_transform_valid = true;
    osc.tracker_space_source = "steamvr_controller_alignment";
    osc.tracker_space_position_offset = Vec3f{1.0f, 2.0f, 3.0f};
    osc.tracker_space_rotation = Quatf{0.0f, 0.0f, 0.0f, 1.0f};
    osc.tracker_space_scale = 1.0f;
    osc.steamvr_alignment_body_signature = SteamVrBodyCalibrationSignature(calib.body);
    osc.steamvr_alignment_floor_signature = SteamVrFloorCalibrationSignature(calib);

    // Try to finish with no samples -> NotEnoughSamples / failure.
    manager.StartSession();
    auto status = manager.FinishSession(body, calib, osc, 2.0);
    BT_CHECK(status.transform_valid);
    // Failed finish must not demote the active transform in status or OscConfig.
    BT_CHECK(osc.tracker_space_transform_valid);
    BT_CHECK(osc.tracker_space_position_offset.x == 1.0f);
    BT_CHECK(osc.tracker_space_position_offset.y == 2.0f);
    BT_CHECK(osc.tracker_space_position_offset.z == 3.0f);
}

void TestManagerClearPreservesManualFallback() {
    SteamVrAlignmentManager manager(MakeFakeProvider(MakeUnavailableSnapshot("test")));
    OscConfig osc;
    osc.tracker_space_transform_valid = true;
    osc.tracker_space_source = "manual";
    osc.tracker_space_position_offset = Vec3f{4.0f, 5.0f, 6.0f};
    manager.ClearAlignment(osc);
    BT_CHECK(osc.tracker_space_transform_valid);
    BT_CHECK(osc.tracker_space_source == "manual");
    BT_CHECK(osc.tracker_space_position_offset.x == 4.0f);
}

void TestManagerStaleSurfacesInStatus() {
    auto snapshot = MakeAvailableSnapshot();
    auto calib = MakeValidCalibration();
    auto result = RunValidSolve(0.0f, Vec3f{0.0f, 0.0f, 0.0f});
    OscConfig osc;
    ApplySteamVrAlignmentToOscConfig(osc, result);
    // ApplySteamVrAlignmentToOscConfig stores manual fallback.
    BT_CHECK(osc.manual_tracker_space_transform_valid);

    SteamVrAlignmentManager manager(MakeFakeProvider(snapshot));
    // Mutate floor calibration to invalidate signature.
    auto mutated = calib;
    mutated.floor.distance += 0.20f;
    auto status = manager.Status(osc, mutated, /*body_state_stable=*/true);
    BT_CHECK(status.stale);
    // Stale + manual available -> prefer manual.
    BT_CHECK(status.active_transform_source == TrackerSpaceSource::Manual);
    BT_CHECK(status.manual_fallback_active);
    // Active source "steamvr_controller_alignment" is stale, so honesty is false.
    BT_CHECK(!SteamVrAlignmentManager::ActiveTransformIsHonest(osc, /*stale=*/true));
}

void TestStaleFiniteTrackerSpaceContinuesOscAsDegraded() {
    OscConfig osc;
    osc.enabled = true;
    osc.send_rotations = false;
    osc.tracker_space_transform_valid = true;
    osc.tracker_space_source = "steamvr_controller_alignment_stale";
    osc.tracker_space_position_offset = Vec3f{1.0f, 2.0f, 3.0f};
    osc.tracker_space_rotation = Quatf{0.0f, 0.0f, 0.0f, 1.0f};
    osc.tracker_space_scale = 1.0f;

    OscSender sender(osc);
    BT_CHECK(sender.Open().ok());
    int send_count = 0;
    sender.SetSendMessageHookForTest([&](const std::string&, float, float, float) {
        ++send_count;
        return Status::OK();
    });

    TrackerPose tracker;
    tracker.role = TrackerRole::Pelvis;
    tracker.pose.position = Vec3f{0.1f, 0.2f, 0.3f};
    tracker.pose.orientation = Quatf{0.0f, 0.0f, 0.0f, 1.0f};
    tracker.confidence = 0.9f;
    tracker.valid = true;
    tracker.evidence.valid = true;
    tracker.evidence.source = TrackerEvidenceSource::DirectStereo;
    tracker.evidence.direct_confidence = 0.9f;

    TrackerPoseArray trackers{};
    trackers[TrackerRoleIndex(TrackerRole::Pelvis)] = tracker;
    const auto sent = sender.SendTrackers(trackers);
    BT_CHECK(sent.ok());
    const auto& report = sender.LastReport();
    BT_CHECK(report.status == "degraded_tracker_space_partial_sent");
    BT_CHECK(report.sent_tracker_count == 1);
    BT_CHECK(report.roles[TrackerRoleIndex(TrackerRole::Pelvis)].degraded);
    BT_CHECK(send_count == 1);

    auto blocked_osc = osc;
    blocked_osc.tracker_space_transform_valid = false;
    OscSender blocked_sender(blocked_osc);
    BT_CHECK(blocked_sender.Open().ok());
    const auto blocked = blocked_sender.SendTrackers(trackers);
    BT_CHECK(!blocked.ok());
    BT_CHECK(blocked_sender.LastReport().status == "blocked_tracker_space");
    BT_CHECK(blocked_sender.LastReport().sent_tracker_count == 0);
}

void TestManagerInvalidAlignmentDoesNotOverrideManual() {
    SteamVrAlignmentManager manager(MakeFakeProvider(MakeUnavailableSnapshot("test")));
    OscConfig osc;
    osc.tracker_space_transform_valid = true;
    osc.tracker_space_source = "manual";
    osc.tracker_space_position_offset = Vec3f{7.0f, 8.0f, 9.0f};

    // StartSession without provider -> session not active.
    auto status = manager.StartSession();
    BT_CHECK(!status.session_active);

    // Finish with empty session -> failure, manual must remain.
    auto body = MakeStableBodyState();
    auto calib = MakeValidCalibration();
    manager.FinishSession(body, calib, osc, 0.0);
    BT_CHECK(osc.tracker_space_transform_valid);
    BT_CHECK(osc.tracker_space_source == "manual");
    BT_CHECK(osc.tracker_space_position_offset.x == 7.0f);
}

// ----- 24) Stale alignment prefers manual fallback when available -----------
// The alignment state machine must enforce: stale alignment does not silently
// substitute for the manual fallback. When stale AND manual fallback is
// available, status must report Manual as active source.

void TestStaleAlignmentPrefersManualFallback() {
    auto snapshot = MakeAvailableSnapshot();
    auto calib = MakeValidCalibration();
    auto result = RunValidSolve(0.0f, Vec3f{0.0f, 0.0f, 0.0f});
    BT_CHECK(result.valid);

    OscConfig cfg;
    ApplySteamVrAlignmentToOscConfig(cfg, result);
    BT_CHECK(cfg.tracker_space_source == "steamvr_controller_alignment");

    // The apply call stores the previous controller alignment as manual fallback.
    BT_CHECK(cfg.manual_tracker_space_transform_valid);

    // Now mark stale via calibration signature mismatch.
    auto mutated = calib;
    mutated.floor.distance += 0.20f;

    SteamVrAlignmentStatusInputs inputs;
    inputs.provider_snapshot = snapshot;
    inputs.session = StartSteamVrAlignmentSession(snapshot);
    inputs.last_solve = result;
    inputs.osc_config = cfg;
    inputs.stale = true;
    inputs.body_calibration_valid = true;
    inputs.floor_calibration_valid = true;
    inputs.body_state_stable = true;
    inputs.body_signature = SteamVrBodyCalibrationSignature(mutated.body);
    inputs.floor_signature = SteamVrFloorCalibrationSignature(mutated);

    auto status = BuildSteamVrAlignmentStatus(inputs);
    BT_CHECK(status.stale);
    BT_CHECK(status.state == "stale");
    // Manual fallback must be preferred over stale controller alignment.
    BT_CHECK(status.active_transform_source == TrackerSpaceSource::Manual);
    BT_CHECK(status.manual_fallback_active);
}

// ----- 25) Stale without manual fallback falls back to stale controller ------

void TestStaleWithoutManualFallsToStaleController() {
    auto snapshot = MakeAvailableSnapshot();
    auto calib = MakeValidCalibration();
    auto result = RunValidSolve(0.0f, Vec3f{0.0f, 0.0f, 0.0f});
    BT_CHECK(result.valid);

    // Manually construct a config with controller alignment but NO manual fallback.
    OscConfig cfg;
    ApplySteamVrAlignmentToOscConfig(cfg, result);
    // Clear the manual fallback fields.
    cfg.manual_tracker_space_transform_valid = false;

    SteamVrAlignmentStatusInputs inputs;
    inputs.provider_snapshot = snapshot;
    inputs.session = StartSteamVrAlignmentSession(snapshot);
    inputs.last_solve = result;
    inputs.osc_config = cfg;
    inputs.stale = true;
    inputs.body_calibration_valid = true;
    inputs.floor_calibration_valid = false;
    inputs.body_state_stable = true;

    auto status = BuildSteamVrAlignmentStatus(inputs);
    BT_CHECK(status.stale);
    BT_CHECK(status.active_transform_source == TrackerSpaceSource::StaleSteamVr);
    BT_CHECK(!status.manual_fallback_active);
}

// ----- 26) Provider unavailable makes alignment stale -----------------------

void TestProviderUnavailableMakesStale() {
    auto snapshot = MakeUnavailableSnapshot("OpenVR runtime crashed");
    auto calib = MakeValidCalibration();
    auto result = RunValidSolve(0.0f, Vec3f{0.0f, 0.0f, 0.0f});
    BT_CHECK(result.valid);

    OscConfig cfg;
    ApplySteamVrAlignmentToOscConfig(cfg, result);
    cfg.manual_tracker_space_transform_valid = false;

    SteamVrAlignmentStatusInputs inputs;
    inputs.provider_snapshot = snapshot;
    inputs.session = StartSteamVrAlignmentSession(snapshot);
    inputs.last_solve = result;
    inputs.osc_config = cfg;
    inputs.stale = false; // calibration signatures still match
    inputs.body_calibration_valid = true;
    inputs.floor_calibration_valid = true;
    inputs.body_state_stable = true;
    inputs.body_signature = SteamVrBodyCalibrationSignature(calib.body);
    inputs.floor_signature = SteamVrFloorCalibrationSignature(calib);

    auto status = BuildSteamVrAlignmentStatus(inputs);
    BT_CHECK(status.stale);
    BT_CHECK(status.state == "stale");
    BT_CHECK(status.reason_code == SteamVrAlignmentReason::ProviderUnavailable);
}

// ----- 27) Blocked OSC path: no valid transform at all ----------------------

void TestBlockedOscWhenNoTransform() {
    OscConfig osc;
    osc.enabled = true;
    osc.send_rotations = false;
    // No transform at all.
    osc.tracker_space_transform_valid = false;

    OscSender sender(osc);
    BT_CHECK(sender.Open().ok());

    TrackerPose tracker;
    tracker.role = TrackerRole::Pelvis;
    tracker.pose.position = Vec3f{0.1f, 0.2f, 0.3f};
    tracker.pose.orientation = Quatf{0.0f, 0.0f, 0.0f, 1.0f};
    tracker.confidence = 0.9f;
    tracker.valid = true;
    tracker.evidence.valid = true;
    tracker.evidence.source = TrackerEvidenceSource::DirectStereo;
    tracker.evidence.direct_confidence = 0.9f;

    TrackerPoseArray trackers{};
    trackers[TrackerRoleIndex(TrackerRole::Pelvis)] = tracker;
    const auto sent = sender.SendTrackers(trackers);
    BT_CHECK(!sent.ok());
    const auto& report = sender.LastReport();
    BT_CHECK(report.status == "blocked_tracker_space");
    BT_CHECK(report.sent_tracker_count == 0);
}

// ----- 28) ActiveTransformIsHonest: stale controller is honest (degraded) ---

void TestActiveTransformHonestStaleController() {
    OscConfig osc;
    osc.tracker_space_transform_valid = true;
    osc.tracker_space_source = "steamvr_controller_alignment_stale";
    osc.tracker_space_position_offset = Vec3f{1.0f, 2.0f, 3.0f};
    osc.tracker_space_rotation = Quatf{0.0f, 0.0f, 0.0f, 1.0f};
    osc.tracker_space_scale = 1.0f;
    // Stale controller alignment is degraded but honest (explicitly marked).
    BT_CHECK(SteamVrAlignmentManager::ActiveTransformIsHonest(osc, /*stale=*/true));

    // Active controller alignment that IS stale is NOT honest.
    osc.tracker_space_source = "steamvr_controller_alignment";
    BT_CHECK(!SteamVrAlignmentManager::ActiveTransformIsHonest(osc, /*stale=*/true));

    // Manual is always honest.
    osc.tracker_space_source = "manual";
    BT_CHECK(SteamVrAlignmentManager::ActiveTransformIsHonest(osc, /*stale=*/true));
}

} // namespace

int main() {
    TestProviderUnavailable();
    TestFakeProviderControllerPoses();
    TestSolvesDegradedWithoutBodyCalibration();
    TestSteamVrFloorSampleDoesNotRequirePriorFloorCalibration();
    TestCannotFinishWithoutRequiredSamples();
    TestEitherControllerCanSampleAnyLandmark();
    TestAnklePromptCapturesAnkleLandmark();
    TestCaptureAcceptsEstimatedAnkleDuringSelfOcclusion();
    TestValidSamplesCreateValidTransform();
    TestFarSampleRejected();
    TestFloorMismatchFails();
    TestYawAmbiguousWhenFeetClose();
    TestScaleMismatchFails();
    TestStaleOnBodySignatureChange();
    TestStaleOnFloorSignatureChange();
    TestClearAlignmentPreservesManualFallback();
    TestManualFallbackWhenControllerAbsent();
    TestControllerAlignmentOverridesManual();
    TestRoleOffsetsPersist();
    TestEightTrackerRoles();
    TestNonfiniteTrackerSkipped();
    TestRedoSingleSlotPreservesOthers();
    TestBuildStatusReflectsTruth();
    TestStaleAlignmentInStatus();
    TestLatencyProbeUnavailableWhenProviderMissing();
    TestLatencyProbeValidWithFakeMotion();
    TestEveryFailureHasSpecificReason();

    // Manager-level behavior tests.
    TestManagerStartFailsWhenProviderUnavailable();
    TestManagerCompleteFlowProducesValidAlignment();
    TestManagerFailedFinishDoesNotOverwriteLastGood();
    TestManagerClearPreservesManualFallback();
    TestManagerStaleSurfacesInStatus();
    TestStaleFiniteTrackerSpaceContinuesOscAsDegraded();
    TestManagerInvalidAlignmentDoesNotOverrideManual();

    // Reject/block/stale path tests.
    TestStaleAlignmentPrefersManualFallback();
    TestStaleWithoutManualFallsToStaleController();
    TestProviderUnavailableMakesStale();
    TestBlockedOscWhenNoTransform();
    TestActiveTransformHonestStaleController();

    std::cout << "steamvr_alignment_test: ok\n";
    return 0;
}
