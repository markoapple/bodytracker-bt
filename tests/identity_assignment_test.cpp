#include "tracking/identity_assignment.h"
#include "test_check.h"

namespace {

void Put(bt::DecodedPose2D& pose, bt::KeypointId id, float x, float confidence = 0.9f) {
    auto& kp = pose.keypoints[static_cast<std::size_t>(id)];
    kp.pixel = bt::Vec2f{x, 100.0f};
    kp.confidence = confidence;
    kp.present = true;
}


bt::CameraCalibration MakeIdentityCameraA() {
    bt::CameraCalibration camera;
    camera.intrinsics_valid = true;
    camera.extrinsics_valid = true;
    camera.camera_matrix = {800.0, 0.0, 320.0,
                            0.0, 800.0, 240.0,
                            0.0, 0.0, 1.0};
    camera.distortion = {0.0, 0.0, 0.0, 0.0, 0.0};
    camera.world_from_camera = bt::Mat34f{{{
        1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f
    }}};
    camera.image_from_world = bt::Mat34f{{{
        800.0f, 0.0f, 320.0f, 0.0f,
        0.0f, 800.0f, 240.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f
    }}};
    return camera;
}

bt::CameraCalibration MakeIdentityCameraB() {
    bt::CameraCalibration camera = MakeIdentityCameraA();
    camera.world_from_camera = bt::Mat34f{{{
        1.0f, 0.0f, 0.0f, 1.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f
    }}};
    camera.image_from_world = bt::Mat34f{{{
        800.0f, 0.0f, 320.0f, -800.0f,
        0.0f, 800.0f, 240.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f
    }}};
    return camera;
}

bt::Vec2f ProjectIdentity(const bt::CameraCalibration& camera, bt::Vec3f world) {
    const bt::Vec3f p = bt::ProjectPoint(camera.image_from_world, world);
    return bt::Vec2f{p.x, p.y};
}

void PutProjected(bt::DecodedPose2D& pose, bt::KeypointId id, bt::Vec2f pixel, float confidence = 0.9f) {
    auto& kp = pose.keypoints[static_cast<std::size_t>(id)];
    kp.pixel = pixel;
    kp.confidence = confidence;
    kp.present = true;
}

void PutProjectedPair(
    bt::DecodedPose2D& pose,
    bt::KeypointId left_id,
    bt::KeypointId right_id,
    bt::Vec2f left_pixel,
    bt::Vec2f right_pixel,
    bool swap_labels,
    float confidence) {

    PutProjected(pose, left_id, swap_labels ? right_pixel : left_pixel, confidence);
    PutProjected(pose, right_id, swap_labels ? left_pixel : right_pixel, confidence);
}

void OffsetPoseY(bt::DecodedPose2D& pose, float dy) {
    for (auto& kp : pose.keypoints) {
        if (kp.present) {
            kp.pixel.y += dy;
        }
    }
}

} // namespace

int main() {
    bt::LowerBodyState predicted;
    predicted.confidence = 0.9f;
    predicted.left_foot.position.x = 0.2f;
    predicted.right_foot.position.x = -0.2f;

    bt::DecodedPose2D observed;
    observed.valid = true;
    Put(observed, bt::KeypointId::LeftHip, 420.0f);
    Put(observed, bt::KeypointId::RightHip, 220.0f);
    Put(observed, bt::KeypointId::LeftKnee, 430.0f);
    Put(observed, bt::KeypointId::RightKnee, 210.0f);
    Put(observed, bt::KeypointId::LeftAnkle, 440.0f);
    Put(observed, bt::KeypointId::RightAnkle, 200.0f);

    const auto kept = bt::ResolveLeftRightIdentity(observed, predicted);
    BT_CHECK(!kept.swapped);
    BT_CHECK(kept.consistency > 0.8f);
    BT_CHECK(kept.pose.keypoints[static_cast<std::size_t>(bt::KeypointId::LeftHip)].pixel.x == 420.0f);
    BT_CHECK(kept.pose.keypoints[static_cast<std::size_t>(bt::KeypointId::RightHip)].pixel.x == 220.0f);

    bt::LowerBodyState crossed_prediction;
    crossed_prediction.confidence = 0.9f;
    crossed_prediction.left_foot.position.x = -0.2f;
    crossed_prediction.right_foot.position.x = 0.2f;
    const auto swapped = bt::ResolveLeftRightIdentity(observed, crossed_prediction);
    BT_CHECK(swapped.swapped);
    BT_CHECK(swapped.consistency > 0.8f);
    BT_CHECK(swapped.pose.keypoints[static_cast<std::size_t>(bt::KeypointId::LeftHip)].pixel.x == 220.0f);
    BT_CHECK(swapped.pose.keypoints[static_cast<std::size_t>(bt::KeypointId::RightHip)].pixel.x == 420.0f);



    bt::Mat34f flipped_projection{{{
        -1.0f, 0.0f, 0.0f, 0.0f,
         0.0f, 1.0f, 0.0f, 0.0f,
         0.0f, 0.0f, 1.0f, 1.0f
    }}};
    const auto projected_swap = bt::ResolveLeftRightIdentity(observed, predicted, &flipped_projection);
    BT_CHECK(projected_swap.swapped);
    BT_CHECK(projected_swap.pose.keypoints[static_cast<std::size_t>(bt::KeypointId::LeftHip)].pixel.x == 220.0f);

    bt::DecodedPose2D weak_camera_b;
    weak_camera_b.valid = true;
    Put(weak_camera_b, bt::KeypointId::LeftHip, 100.0f, 0.9f);
    Put(weak_camera_b, bt::KeypointId::RightHip, 110.0f, 0.9f);
    const auto stereo_identity = bt::ResolveStereoLeftRightIdentity(observed, weak_camera_b, predicted);
    BT_CHECK(stereo_identity.cross_camera_override_applied);
    BT_CHECK(!stereo_identity.camera_a.swapped);
    BT_CHECK(!stereo_identity.camera_b.swapped);


    const auto camera_a = MakeIdentityCameraA();
    const auto camera_b = MakeIdentityCameraB();
    const auto geometry = bt::ComputeEpipolarGeometry(camera_a, camera_b);
    BT_CHECK(geometry.ok());
    const auto geometry_value = geometry.value();

    bt::DecodedPose2D epipolar_a;
    bt::DecodedPose2D epipolar_b;
    epipolar_a.valid = true;
    epipolar_b.valid = true;

    const bt::Vec3f left_hip{0.65f, 0.55f, 4.0f};
    const bt::Vec3f right_hip{-0.65f, -0.45f, 4.0f};
    const bt::Vec3f left_knee{0.72f, 0.35f, 4.1f};
    const bt::Vec3f right_knee{-0.72f, -0.25f, 4.1f};
    const bt::Vec3f left_ankle{0.78f, 0.15f, 4.2f};
    const bt::Vec3f right_ankle{-0.78f, -0.05f, 4.2f};

    PutProjectedPair(
        epipolar_a,
        bt::KeypointId::LeftHip,
        bt::KeypointId::RightHip,
        ProjectIdentity(camera_a, left_hip),
        ProjectIdentity(camera_a, right_hip),
        false,
        0.95f);
    PutProjectedPair(
        epipolar_a,
        bt::KeypointId::LeftKnee,
        bt::KeypointId::RightKnee,
        ProjectIdentity(camera_a, left_knee),
        ProjectIdentity(camera_a, right_knee),
        false,
        0.95f);
    PutProjectedPair(
        epipolar_a,
        bt::KeypointId::LeftAnkle,
        bt::KeypointId::RightAnkle,
        ProjectIdentity(camera_a, left_ankle),
        ProjectIdentity(camera_a, right_ankle),
        false,
        0.95f);

    PutProjectedPair(
        epipolar_b,
        bt::KeypointId::LeftHip,
        bt::KeypointId::RightHip,
        ProjectIdentity(camera_b, left_hip),
        ProjectIdentity(camera_b, right_hip),
        true,
        0.20f);
    PutProjectedPair(
        epipolar_b,
        bt::KeypointId::LeftKnee,
        bt::KeypointId::RightKnee,
        ProjectIdentity(camera_b, left_knee),
        ProjectIdentity(camera_b, right_knee),
        true,
        0.20f);
    PutProjectedPair(
        epipolar_b,
        bt::KeypointId::LeftAnkle,
        bt::KeypointId::RightAnkle,
        ProjectIdentity(camera_b, left_ankle),
        ProjectIdentity(camera_b, right_ankle),
        true,
        0.20f);

    bt::StereoIdentityEpipolarContext epipolar_context;
    epipolar_context.geometry = &geometry_value;
    epipolar_context.camera_a = &camera_a;
    epipolar_context.camera_b = &camera_b;

    bt::Mat34f camera_b_wrong_local_direction{{{
        -1.0f, 0.0f, 0.0f, 0.0f,
         0.0f, 1.0f, 0.0f, 0.0f,
         0.0f, 0.0f, 1.0f, 1.0f
    }}};

    const auto epipolar_identity = bt::ResolveStereoLeftRightIdentity(
        epipolar_a,
        epipolar_b,
        predicted,
        &camera_a.image_from_world,
        &camera_b_wrong_local_direction,
        &epipolar_context);
    BT_CHECK(epipolar_identity.epipolar_arbitration_checked);
    BT_CHECK(epipolar_identity.epipolar_arbitration_applied);
    BT_CHECK(epipolar_identity.cross_camera_override_applied);
    BT_CHECK(!epipolar_identity.camera_a.swapped);
    BT_CHECK(epipolar_identity.camera_b.swapped);
    BT_CHECK(epipolar_identity.epipolar_scored_lateral_pairs >= 2);
    BT_CHECK(epipolar_identity.epipolar_cross_identity_score > epipolar_identity.epipolar_same_identity_score + 0.18f);

    auto strict_mahalanobis_context = epipolar_context;
    strict_mahalanobis_context.config.identity_max_mahalanobis_sq = 1.0f;
    const auto strict_mahalanobis_identity = bt::ResolveStereoLeftRightIdentity(
        epipolar_a,
        epipolar_b,
        predicted,
        &camera_a.image_from_world,
        &camera_b_wrong_local_direction,
        &strict_mahalanobis_context);
    BT_CHECK(strict_mahalanobis_identity.epipolar_arbitration_checked);
    BT_CHECK(!strict_mahalanobis_identity.epipolar_arbitration_applied);
    BT_CHECK(strict_mahalanobis_identity.identity_cross_mahalanobis_sq >
             strict_mahalanobis_context.config.identity_max_mahalanobis_sq);

    auto degraded_context = epipolar_context;
    degraded_context.reused_camera_b = true;
    const auto degraded_identity = bt::ResolveStereoLeftRightIdentity(
        epipolar_a,
        epipolar_b,
        predicted,
        &camera_a.image_from_world,
        &camera_b_wrong_local_direction,
        &degraded_context);
    BT_CHECK(!degraded_identity.epipolar_arbitration_checked);
    BT_CHECK(!degraded_identity.epipolar_arbitration_applied);

    auto weak_margin_context = epipolar_context;
    weak_margin_context.config.swap_absolute_margin = 1.10f;
    const auto weak_margin_identity = bt::ResolveStereoLeftRightIdentity(
        epipolar_a,
        epipolar_b,
        predicted,
        &camera_a.image_from_world,
        &camera_b_wrong_local_direction,
        &weak_margin_context);
    BT_CHECK(weak_margin_identity.epipolar_arbitration_checked);
    BT_CHECK(!weak_margin_identity.epipolar_arbitration_applied);

    bt::DecodedPose2D guarded_b;
    guarded_b.valid = true;
    PutProjectedPair(
        guarded_b,
        bt::KeypointId::LeftHip,
        bt::KeypointId::RightHip,
        ProjectIdentity(camera_b, left_hip),
        ProjectIdentity(camera_b, right_hip),
        true,
        0.95f);
    PutProjectedPair(
        guarded_b,
        bt::KeypointId::LeftKnee,
        bt::KeypointId::RightKnee,
        ProjectIdentity(camera_b, left_knee),
        ProjectIdentity(camera_b, right_knee),
        true,
        0.95f);
    PutProjectedPair(
        guarded_b,
        bt::KeypointId::LeftAnkle,
        bt::KeypointId::RightAnkle,
        ProjectIdentity(camera_b, left_ankle),
        ProjectIdentity(camera_b, right_ankle),
        true,
        0.95f);
    const auto both_guarded_identity = bt::ResolveStereoLeftRightIdentity(
        epipolar_a,
        guarded_b,
        predicted,
        &camera_a.image_from_world,
        &camera_b_wrong_local_direction,
        &epipolar_context);
    BT_CHECK(both_guarded_identity.epipolar_arbitration_checked);
    BT_CHECK(!both_guarded_identity.epipolar_arbitration_applied);


    bt::DecodedPose2D uncertain_b = epipolar_b;
    OffsetPoseY(uncertain_b, 6.0f);
    auto uncertain_context = epipolar_context;
    uncertain_context.config.uncertainty_swap_margin_scale = 4.0f;
    const auto uncertain_identity = bt::ResolveStereoLeftRightIdentity(
        epipolar_a,
        uncertain_b,
        predicted,
        &camera_a.image_from_world,
        &camera_b_wrong_local_direction,
        &uncertain_context);
    BT_CHECK(uncertain_identity.epipolar_arbitration_checked);
    BT_CHECK(!uncertain_identity.epipolar_arbitration_applied);
    BT_CHECK(uncertain_identity.epipolar_cross_geometric_uncertainty > 0.0f);
    BT_CHECK(uncertain_identity.epipolar_required_swap_margin >
             uncertain_context.config.swap_absolute_margin);

    bt::DecodedPose2D low_support_b = epipolar_b;
    for (auto& kp : low_support_b.keypoints) {
        if (kp.present) {
            kp.confidence = 0.01f;
        }
    }
    auto low_support_context = epipolar_context;
    low_support_context.config.min_detection_support = 0.70f;
    const auto low_support_identity = bt::ResolveStereoLeftRightIdentity(
        epipolar_a,
        low_support_b,
        predicted,
        &camera_a.image_from_world,
        &camera_b_wrong_local_direction,
        &low_support_context);
    BT_CHECK(low_support_identity.epipolar_arbitration_checked);
    BT_CHECK(!low_support_identity.epipolar_arbitration_applied);
    BT_CHECK(low_support_identity.epipolar_cross_identity_score >
             low_support_identity.epipolar_same_identity_score);
    BT_CHECK(low_support_identity.epipolar_cross_geometric_uncertainty < 1.0f);
    BT_CHECK(low_support_identity.epipolar_detection_support < low_support_context.config.min_detection_support);

    bt::CameraCalibration anisotropic_a = camera_a;
    anisotropic_a.camera_matrix = {1100.0, 0.0, 320.0,
                                  0.0, 650.0, 240.0,
                                  0.0, 0.0, 1.0};
    anisotropic_a.image_from_world = bt::Mat34f{{{
        1100.0f, 0.0f, 320.0f, 0.0f,
        0.0f, 650.0f, 240.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f
    }}};
    bt::CameraCalibration anisotropic_b = camera_b;
    anisotropic_b.camera_matrix = {900.0, 0.0, 300.0,
                                  0.0, 720.0, 250.0,
                                  0.0, 0.0, 1.0};
    anisotropic_b.image_from_world = bt::Mat34f{{{
        900.0f, 0.0f, 300.0f, -900.0f,
        0.0f, 720.0f, 250.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f
    }}};
    const auto anisotropic_geometry = bt::ComputeEpipolarGeometry(anisotropic_a, anisotropic_b);
    BT_CHECK(anisotropic_geometry.ok());
    bt::DecodedPose2D anisotropic_pose_a;
    bt::DecodedPose2D anisotropic_pose_b;
    anisotropic_pose_a.valid = true;
    anisotropic_pose_b.valid = true;
    PutProjectedPair(
        anisotropic_pose_a,
        bt::KeypointId::LeftHip,
        bt::KeypointId::RightHip,
        ProjectIdentity(anisotropic_a, left_hip),
        ProjectIdentity(anisotropic_a, right_hip),
        false,
        0.95f);
    PutProjectedPair(
        anisotropic_pose_a,
        bt::KeypointId::LeftKnee,
        bt::KeypointId::RightKnee,
        ProjectIdentity(anisotropic_a, left_knee),
        ProjectIdentity(anisotropic_a, right_knee),
        false,
        0.95f);
    PutProjectedPair(
        anisotropic_pose_b,
        bt::KeypointId::LeftHip,
        bt::KeypointId::RightHip,
        ProjectIdentity(anisotropic_b, left_hip),
        ProjectIdentity(anisotropic_b, right_hip),
        true,
        0.30f);
    PutProjectedPair(
        anisotropic_pose_b,
        bt::KeypointId::LeftKnee,
        bt::KeypointId::RightKnee,
        ProjectIdentity(anisotropic_b, left_knee),
        ProjectIdentity(anisotropic_b, right_knee),
        true,
        0.30f);
    bt::StereoIdentityEpipolarContext anisotropic_context;
    const auto anisotropic_geometry_value = anisotropic_geometry.value();
    anisotropic_context.geometry = &anisotropic_geometry_value;
    anisotropic_context.camera_a = &anisotropic_a;
    anisotropic_context.camera_b = &anisotropic_b;
    const auto anisotropic_identity = bt::ResolveStereoLeftRightIdentity(
        anisotropic_pose_a,
        anisotropic_pose_b,
        predicted,
        &anisotropic_a.image_from_world,
        &camera_b_wrong_local_direction,
        &anisotropic_context);
    BT_CHECK(anisotropic_identity.epipolar_arbitration_checked);
    BT_CHECK(anisotropic_identity.epipolar_arbitration_applied);

    bt::DecodedPose2D partial_a;
    bt::DecodedPose2D partial_b;
    partial_a.valid = true;
    partial_b.valid = true;
    PutProjectedPair(
        partial_a,
        bt::KeypointId::LeftHip,
        bt::KeypointId::RightHip,
        ProjectIdentity(camera_a, left_hip),
        ProjectIdentity(camera_a, right_hip),
        false,
        0.95f);
    PutProjectedPair(
        partial_b,
        bt::KeypointId::LeftHip,
        bt::KeypointId::RightHip,
        ProjectIdentity(camera_b, left_hip),
        ProjectIdentity(camera_b, right_hip),
        true,
        0.95f);
    const auto partial_identity = bt::ResolveStereoLeftRightIdentity(
        partial_a,
        partial_b,
        predicted,
        &camera_a.image_from_world,
        &camera_b_wrong_local_direction,
        &epipolar_context);
    BT_CHECK(partial_identity.epipolar_arbitration_checked);
    BT_CHECK(!partial_identity.epipolar_arbitration_applied);
    BT_CHECK(partial_identity.epipolar_scored_lateral_pairs == 1);


    bt::LowerBodyState uncertainty_predicted;
    uncertainty_predicted.confidence = 0.9f;
    uncertainty_predicted.left_foot.position = bt::Vec3f{0.65f, 0.15f, 4.0f};
    uncertainty_predicted.right_foot.position = bt::Vec3f{-0.65f, -0.05f, 4.0f};

    auto anisotropic_identity_context = epipolar_context;
    anisotropic_identity_context.config.min_scored_lateral_pairs = 1;
    anisotropic_identity_context.config.min_detection_support = 0.0f;
    anisotropic_identity_context.config.swap_absolute_margin = 2.0f;

    auto make_single_pair_identity_pose = [](
        const bt::CameraCalibration& ca,
        const bt::CameraCalibration& cb,
        bt::Vec3f left_world,
        bt::Vec3f right_world) {
        std::pair<bt::DecodedPose2D, bt::DecodedPose2D> poses;
        poses.first.valid = true;
        poses.second.valid = true;
        PutProjectedPair(
            poses.first,
            bt::KeypointId::LeftAnkle,
            bt::KeypointId::RightAnkle,
            ProjectIdentity(ca, left_world),
            ProjectIdentity(ca, right_world),
            false,
            0.95f);
        PutProjectedPair(
            poses.second,
            bt::KeypointId::LeftAnkle,
            bt::KeypointId::RightAnkle,
            ProjectIdentity(cb, left_world),
            ProjectIdentity(cb, right_world),
            false,
            0.95f);
        return poses;
    };

    const auto lateral_offset_pair = make_single_pair_identity_pose(
        camera_a,
        camera_b,
        bt::Vec3f{0.69f, 0.15f, 4.0f},
        bt::Vec3f{-0.61f, -0.05f, 4.0f});
    const auto depth_offset_pair = make_single_pair_identity_pose(
        camera_a,
        camera_b,
        bt::Vec3f{0.65f, 0.15f, 4.04f},
        bt::Vec3f{-0.65f, -0.05f, 4.04f});

    const auto lateral_offset_identity = bt::ResolveStereoLeftRightIdentity(
        lateral_offset_pair.first,
        lateral_offset_pair.second,
        uncertainty_predicted,
        &camera_a.image_from_world,
        &camera_b.image_from_world,
        &anisotropic_identity_context);
    const auto depth_offset_identity = bt::ResolveStereoLeftRightIdentity(
        depth_offset_pair.first,
        depth_offset_pair.second,
        uncertainty_predicted,
        &camera_a.image_from_world,
        &camera_b.image_from_world,
        &anisotropic_identity_context);

    BT_CHECK(lateral_offset_identity.epipolar_arbitration_checked);
    BT_CHECK(depth_offset_identity.epipolar_arbitration_checked);
    BT_CHECK(lateral_offset_identity.identity_same_mahalanobis_sq >
             depth_offset_identity.identity_same_mahalanobis_sq);

    bt::LowerBodyState fresh_prior_predicted = uncertainty_predicted;
    fresh_prior_predicted.left_foot_evidence.valid = true;
    fresh_prior_predicted.left_foot_evidence.source = bt::TrackerEvidenceSource::DirectStereo;
    fresh_prior_predicted.left_foot_evidence.direct_confidence = 1.0f;
    fresh_prior_predicted.right_foot_evidence.valid = true;
    fresh_prior_predicted.right_foot_evidence.source = bt::TrackerEvidenceSource::DirectStereo;
    fresh_prior_predicted.right_foot_evidence.direct_confidence = 1.0f;

    auto fresh_prior_context = anisotropic_identity_context;
    fresh_prior_context.predicted_state_age_seconds = 0.0f;
    const auto fresh_prior_identity = bt::ResolveStereoLeftRightIdentity(
        lateral_offset_pair.first,
        lateral_offset_pair.second,
        fresh_prior_predicted,
        &camera_a.image_from_world,
        &camera_b.image_from_world,
        &fresh_prior_context);

    auto aged_prior_context = fresh_prior_context;
    aged_prior_context.predicted_state_age_seconds = 1.0f;
    const auto aged_prior_identity = bt::ResolveStereoLeftRightIdentity(
        lateral_offset_pair.first,
        lateral_offset_pair.second,
        fresh_prior_predicted,
        &camera_a.image_from_world,
        &camera_b.image_from_world,
        &aged_prior_context);

    BT_CHECK(aged_prior_identity.identity_same_mahalanobis_sq <
             fresh_prior_identity.identity_same_mahalanobis_sq);

    const auto missing_prior_identity = bt::ResolveStereoLeftRightIdentity(
        lateral_offset_pair.first,
        lateral_offset_pair.second,
        uncertainty_predicted,
        &camera_a.image_from_world,
        &camera_b.image_from_world,
        &fresh_prior_context);
    BT_CHECK(missing_prior_identity.identity_same_mahalanobis_sq <
             fresh_prior_identity.identity_same_mahalanobis_sq);
    BT_CHECK(missing_prior_identity.identity_uncertainty_fallback_count > 0);

    auto comparative_swap_context = epipolar_context;
    comparative_swap_context.config.swap_absolute_margin = 10.0f;
    comparative_swap_context.config.uncertainty_swap_margin_scale = 0.0f;
    comparative_swap_context.config.partial_coverage_swap_margin = 0.0f;
    comparative_swap_context.config.min_assignment_score = 0.0f;
    comparative_swap_context.config.identity_swap_nll_margin = 1.0f;
    const auto comparative_swap_identity = bt::ResolveStereoLeftRightIdentity(
        epipolar_a,
        epipolar_b,
        uncertainty_predicted,
        &camera_a.image_from_world,
        &camera_b_wrong_local_direction,
        &comparative_swap_context);
    BT_CHECK(comparative_swap_identity.epipolar_arbitration_checked);
    BT_CHECK(comparative_swap_identity.epipolar_arbitration_applied);
    BT_CHECK(comparative_swap_identity.identity_same_mahalanobis_sq >
             comparative_swap_identity.identity_cross_mahalanobis_sq);
    BT_CHECK(comparative_swap_identity.identity_same_negative_log_likelihood >
             comparative_swap_identity.identity_cross_negative_log_likelihood);
    BT_CHECK(comparative_swap_identity.identity_cross_within_mahalanobis_gate);
    BT_CHECK(comparative_swap_identity.identity_likelihood_gate_passed);

    bt::DecodedPose2D invalid;
    invalid.valid = false;
    const auto single_camera_identity = bt::ResolveStereoLeftRightIdentity(observed, invalid, predicted);
    BT_CHECK(single_camera_identity.camera_a.pose.valid);
    BT_CHECK(single_camera_identity.camera_a.consistency > 0.0f);
    BT_CHECK(!single_camera_identity.cross_camera_override_applied);
    BT_CHECK(!single_camera_identity.epipolar_arbitration_checked);

    const auto invalid_result = bt::ResolveLeftRightIdentity(invalid, predicted);
    BT_CHECK(!invalid_result.swapped);
    BT_CHECK(invalid_result.consistency == 0.0f);
    return 0;
}
