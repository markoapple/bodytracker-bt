#include "tracking/body_state.h"
#include "tracking/tracker_synthesis.h"
#include "test_check.h"

namespace {

bt::Pose3f Pose(float x, float y, float z) {
    bt::Pose3f pose;
    pose.position = bt::Vec3f{x, y, z};
    pose.orientation = bt::Quatf{0.0f, 0.0f, 0.0f, 1.0f};
    return pose;
}

} // namespace

int main() {
    bt::LowerBodyState state;
    state.root = Pose(0.0f, 1.0f, 0.0f);
    state.left_foot = Pose(-0.2f, 0.0f, 0.0f);
    state.right_foot = Pose(0.2f, 0.0f, 0.0f);
    state.confidence = 0.8f;
    state.linear_velocity = bt::Vec3f{0.6f, 0.0f, 0.0f};
    state.left_foot_linear_velocity = bt::Vec3f{0.2f, 0.0f, 0.0f};
    state.right_foot_linear_velocity = bt::Vec3f{0.1f, 0.0f, 0.0f};
    state.support.left_foot.type = bt::FootSupportType::FloorSupport;
    state.support.left_foot.phase = bt::FootSupportPhase::FlatPlant;
    state.support.left_foot.contact_load = bt::FootContactLoad::FullPlant;
    state.support.left_foot.anchor.active = true;
    state.support.left_foot.anchor.confidence = 0.9f;
    state.support.right_foot.phase = bt::FootSupportPhase::ToePivot;

    bt::LowerBodyModel model;
    bt::BodyCalibration calibration;
    calibration.standing_neutral_valid = true;
    calibration.quality.overall = 0.82f;

    bt::BodyStateSolverSnapshot solver;
    solver.tracking_mode = bt::TrackingMode::Stereo;
    solver.depth_source = bt::DepthSource::TriangulatedStereo;
    solver.camera_a_identity_consistency = 0.8f;
    solver.camera_b_identity_consistency = 0.75f;
    solver.triangulated_count = 5;
    solver.mean_reprojection_error_px = 3.0f;
    auto& left_knee = solver.joints[static_cast<std::size_t>(bt::KeypointId::LeftKnee)];
    left_knee.triangulated = true;
    left_knee.depth_source = bt::DepthSource::TriangulatedStereo;
    left_knee.evidence_source = bt::JointEvidenceSource::Stereo;
    left_knee.camera_a_present = true;
    left_knee.camera_b_present = true;
    left_knee.camera_a_confidence = 0.8f;
    left_knee.camera_b_confidence = 0.75f;
    left_knee.camera_a_weight = 0.8f;
    left_knee.camera_b_weight = 0.75f;
    left_knee.camera_a_quality = 0.72f;
    left_knee.camera_b_quality = 0.68f;
    left_knee.world = bt::Vec3f{-0.2f, 0.45f, 0.0f};
    left_knee.confidence = 0.7f;
    left_knee.mean_reprojection_error_px = 2.0f;
    left_knee.contact_confidence = 0.25f;

    const auto body = bt::BuildUnifiedBodyState(state, model, solver, calibration, 1.0 / 60.0, true);
    BT_CHECK(body.valid);
    BT_CHECK(body.diagnostics.active);
    BT_CHECK(body.diagnostics.triangulation_active);
    BT_CHECK(body.diagnostics.left_right_identity_stable);
    BT_CHECK(body.diagnostics.contact_lock_active);
    BT_CHECK(body.diagnostics.floor_support_active);
    BT_CHECK(body.diagnostics.body_calibration_valid);
    BT_CHECK(body.diagnostics.latency_prediction_active);
    BT_CHECK(body.diagnostics.measured_role_count >= 2);
    BT_CHECK(body.diagnostics.anchored_role_count >= 1);
    BT_CHECK(body.diagnostics.invalid_role_count < static_cast<int>(bt::kBodyJointRoleCount));
    BT_CHECK(body.diagnostics.identity_confidence > 0.7f);
    BT_CHECK(body.diagnostics.left_contact_lock_strength >= 0.9f);
    BT_CHECK(body.diagnostics.role_output_confidence > 0.0f);
    BT_CHECK(body.left_foot_contact == bt::BodyFootContactState::FullPlant);
    BT_CHECK(body.right_foot_contact == bt::BodyFootContactState::ToeContact);

    const auto& left_foot = body.roles[bt::BodyJointRoleIndex(bt::BodyJointRole::LeftFoot)];
    BT_CHECK(left_foot.visibility == bt::BodyJointVisibility::Anchored);
    BT_CHECK(left_foot.measured);
    BT_CHECK(!left_foot.predicted);
    BT_CHECK(left_foot.evidence.source == bt::TrackerEvidenceSource::AnchorHeld);
    BT_CHECK(bt::EffectiveSignalKind(left_foot.evidence) == bt::TrackingSignalKind::Anchored);
    BT_CHECK(body.diagnostics.predicted_joint_count < static_cast<int>(bt::kBodyJointRoleCount));

    const auto& knee = body.roles[bt::BodyJointRoleIndex(bt::BodyJointRole::LeftKnee)];
    BT_CHECK(knee.valid);
    BT_CHECK(knee.measured);
    BT_CHECK(!knee.predicted);
    BT_CHECK(knee.camera_a_present);
    BT_CHECK(knee.camera_b_present);
    BT_CHECK(knee.triangulated);
    BT_CHECK(knee.evidence_source == bt::JointEvidenceSource::Stereo);
    BT_CHECK_NEAR(knee.camera_a_quality, 0.72f, 1e-5f);
    BT_CHECK_NEAR(knee.camera_b_quality, 0.68f, 1e-5f);
    BT_CHECK(knee.visibility == bt::BodyJointVisibility::Visible);
    BT_CHECK(knee.confidence < 0.7f);
    BT_CHECK(knee.confidence > 0.55f);
    BT_CHECK_NEAR(knee.contact_support_confidence, 0.25f, 1e-5f);

    const auto& right_knee = body.roles[bt::BodyJointRoleIndex(bt::BodyJointRole::RightKnee)];
    BT_CHECK(right_knee.predicted);
    BT_CHECK(right_knee.visibility == bt::BodyJointVisibility::CameraOccluded);

    const auto trackers = bt::SynthesizeTrackerPoses(body, model);
    BT_CHECK(trackers[bt::TrackerRoleIndex(bt::TrackerRole::Pelvis)].valid);
    BT_CHECK(trackers[bt::TrackerRoleIndex(bt::TrackerRole::LeftKnee)].valid);
    BT_CHECK(trackers[bt::TrackerRoleIndex(bt::TrackerRole::LeftKnee)].confidence > 0.55f);
    BT_CHECK(std::isfinite(trackers[bt::TrackerRoleIndex(bt::TrackerRole::Pelvis)].pose.position.x));
    BT_CHECK_NEAR(trackers[bt::TrackerRoleIndex(bt::TrackerRole::LeftKnee)].pose.position.y, 0.45f, 1e-5f);

    bt::BodyStateSolverSnapshot low_solver_weight = solver;
    auto& low_weight_knee = low_solver_weight.joints[static_cast<std::size_t>(bt::KeypointId::LeftKnee)];
    low_weight_knee.solver_observation_weighted = true;
    low_weight_knee.solver_observation_weight_scale = 0.01f;
    const auto low_weight_body = bt::BuildUnifiedBodyState(
        state,
        model,
        low_solver_weight,
        calibration,
        1.0 / 60.0,
        true);
    const auto& low_weight_role = low_weight_body.roles[bt::BodyJointRoleIndex(bt::BodyJointRole::LeftKnee)];
    BT_CHECK(low_weight_role.solver_observation_weighted);
    BT_CHECK_NEAR(low_weight_role.solver_observation_weight_scale, 0.01f, 1e-6f);
    BT_CHECK_NEAR(low_weight_role.solver_observation_confidence_ceiling, 0.55f, 1e-6f);
    // Low solver weight is a geometric/temporal reliability ceiling, not a raw
    // confidence multiplier. It must prevent high-confidence output without
    // making visually present fallback-quality observations disappear.
    BT_CHECK(low_weight_role.confidence < knee.confidence);
    BT_CHECK(low_weight_role.confidence > 0.50f);
    BT_CHECK(low_weight_role.evidence.direct_confidence < knee.evidence.direct_confidence);
    BT_CHECK(low_weight_role.evidence.direct_confidence > 0.50f);
    const auto low_weight_trackers = bt::SynthesizeTrackerPoses(low_weight_body, model);
    BT_CHECK(low_weight_trackers[bt::TrackerRoleIndex(bt::TrackerRole::LeftKnee)].confidence <
             trackers[bt::TrackerRoleIndex(bt::TrackerRole::LeftKnee)].confidence);
    BT_CHECK(low_weight_trackers[bt::TrackerRoleIndex(bt::TrackerRole::LeftKnee)].confidence > 0.50f);

    bt::BodyStateSolverSnapshot weak_identity = solver;
    weak_identity.camera_a_identity_consistency = 0.2f;
    weak_identity.camera_b_identity_consistency = 0.2f;
    const auto weak = bt::BuildUnifiedBodyState(state, model, weak_identity, calibration, 1.0 / 60.0, true);
    BT_CHECK(!weak.diagnostics.left_right_identity_stable);
    BT_CHECK(weak.diagnostics.left_right_identity_uncertain);

    bt::BodyStateSolverSnapshot no_arm_solver = solver;
    no_arm_solver.tracking_mode = bt::TrackingMode::Monocular;
    no_arm_solver.depth_source = bt::DepthSource::InferredMonocular;
    no_arm_solver.triangulated_count = 0;
    no_arm_solver.inferred_depth_count = 8;
    no_arm_solver.joints[static_cast<std::size_t>(bt::KeypointId::LeftKnee)] = {};
    const auto no_arm_body = bt::BuildUnifiedBodyState(state, model, no_arm_solver, calibration, 1.0 / 60.0, true);
    const auto no_arm_elbow = no_arm_body.roles[bt::BodyJointRoleIndex(bt::BodyJointRole::LeftElbow)].position;

    bt::BodyStateSolverSnapshot amplified_arm_solver = no_arm_solver;
    auto& noisy_elbow = amplified_arm_solver.joints[static_cast<std::size_t>(bt::KeypointId::LeftElbow)];
    noisy_elbow.depth_inferred = true;
    noisy_elbow.depth_source = bt::DepthSource::InferredMonocular;
    noisy_elbow.evidence_source = bt::JointEvidenceSource::CameraAOnly;
    noisy_elbow.camera_a_present = true;
    noisy_elbow.camera_a_confidence = 0.95f;
    noisy_elbow.camera_a_weight = 1.0f;
    noisy_elbow.camera_a_quality = 0.90f;
    noisy_elbow.world = bt::Vec3f{-1.60f, 1.20f, 1.10f};
    noisy_elbow.confidence = 0.95f;
    noisy_elbow.solver_observation_weighted = true;
    noisy_elbow.solver_observation_weight_scale = 1.0f;
    auto& noisy_wrist = amplified_arm_solver.joints[static_cast<std::size_t>(bt::KeypointId::LeftWrist)];
    noisy_wrist = noisy_elbow;
    noisy_wrist.world = bt::Vec3f{-2.20f, 1.10f, 1.40f};

    const auto bounded_arm_body = bt::BuildUnifiedBodyState(state, model, amplified_arm_solver, calibration, 1.0 / 60.0, true);
    const auto& bounded_elbow_role = bounded_arm_body.roles[bt::BodyJointRoleIndex(bt::BodyJointRole::LeftElbow)];
    const auto& bounded_shoulder_role = bounded_arm_body.roles[bt::BodyJointRoleIndex(bt::BodyJointRole::LeftShoulder)];
    BT_CHECK(bounded_elbow_role.valid);
    BT_CHECK(bounded_elbow_role.depth_inferred);
    BT_CHECK(bt::Distance(bounded_elbow_role.position, no_arm_elbow) < 0.20f);
    BT_CHECK(bt::Distance(bounded_elbow_role.position, bounded_shoulder_role.position) < 0.38f);

    bt::BodyStateSolverSnapshot noisy_knee_solver = no_arm_solver;
    auto& noisy_mono_knee = noisy_knee_solver.joints[static_cast<std::size_t>(bt::KeypointId::LeftKnee)];
    noisy_mono_knee.depth_inferred = true;
    noisy_mono_knee.depth_source = bt::DepthSource::InferredMonocular;
    noisy_mono_knee.evidence_source = bt::JointEvidenceSource::CameraAOnly;
    noisy_mono_knee.camera_a_present = true;
    noisy_mono_knee.camera_a_confidence = 0.90f;
    noisy_mono_knee.camera_a_weight = 1.0f;
    noisy_mono_knee.camera_a_quality = 0.85f;
    noisy_mono_knee.world = bt::Vec3f{-1.20f, 0.65f, 0.90f};
    noisy_mono_knee.confidence = 0.90f;
    noisy_mono_knee.solver_observation_weighted = true;
    noisy_mono_knee.solver_observation_weight_scale = 1.0f;
    const auto bounded_knee_body = bt::BuildUnifiedBodyState(state, model, noisy_knee_solver, calibration, 1.0 / 60.0, true);
    const auto predicted_knee = no_arm_body.roles[bt::BodyJointRoleIndex(bt::BodyJointRole::LeftKnee)].position;
    const auto bounded_knee = bounded_knee_body.roles[bt::BodyJointRoleIndex(bt::BodyJointRole::LeftKnee)].position;
    BT_CHECK(bt::Distance(bounded_knee, predicted_knee) < 0.20f);

    return 0;
}
