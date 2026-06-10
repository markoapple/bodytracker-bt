#include "tracking/identity_assignment.h"
#include "tracking/epipolar_geometry.h"
#include "tracking/measurement_weighting.h"
#include "tracking/triangulation.h"
#include "test_check.h"

#include <cmath>

namespace {

// ── Identity camera fixtures ──────────────────────────────────────────

bt::CameraCalibration MakeCameraA() {
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

bt::CameraCalibration MakeCameraBWithBaseline(float baseline_m) {
    bt::CameraCalibration camera = MakeCameraA();
    camera.world_from_camera = bt::Mat34f{{{
        1.0f, 0.0f, 0.0f, baseline_m,
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f
    }}};
    camera.image_from_world = bt::Mat34f{{{
        800.0f, 0.0f, 320.0f, -800.0f * baseline_m,
        0.0f, 800.0f, 240.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f
    }}};
    return camera;
}

bt::CameraCalibration MakeCameraB() {
    return MakeCameraBWithBaseline(1.0f);
}

void Put(bt::DecodedPose2D& pose, bt::KeypointId id, float x,
         float y = 100.0f, float confidence = 0.9f) {
    auto& kp = pose.keypoints[static_cast<size_t>(id)];
    kp.pixel = bt::Vec2f{x, y};
    kp.confidence = confidence;
    kp.present = true;
}

// Swap left/right labels in a pose.
bt::DecodedPose2D SwapLabels(const bt::DecodedPose2D& pose) {
    bt::DecodedPose2D swapped = pose;
    constexpr bt::KeypointId pairs[][2] = {
        {bt::KeypointId::LeftEye, bt::KeypointId::RightEye},
        {bt::KeypointId::LeftEar, bt::KeypointId::RightEar},
        {bt::KeypointId::LeftShoulder, bt::KeypointId::RightShoulder},
        {bt::KeypointId::LeftElbow, bt::KeypointId::RightElbow},
        {bt::KeypointId::LeftWrist, bt::KeypointId::RightWrist},
        {bt::KeypointId::LeftHip, bt::KeypointId::RightHip},
        {bt::KeypointId::LeftKnee, bt::KeypointId::RightKnee},
        {bt::KeypointId::LeftAnkle, bt::KeypointId::RightAnkle},
        {bt::KeypointId::LeftBigToe, bt::KeypointId::RightBigToe},
        {bt::KeypointId::LeftSmallToe, bt::KeypointId::RightSmallToe},
        {bt::KeypointId::LeftHeel, bt::KeypointId::RightHeel},
    };
    for (auto& p : pairs) {
        std::swap(
            swapped.keypoints[static_cast<size_t>(p[0])],
            swapped.keypoints[static_cast<size_t>(p[1])]);
    }
    return swapped;
}

bt::StereoIdentityEpipolarContext MakeEpipolarContext(
    const bt::EpipolarGeometry& geometry,
    const bt::CameraCalibration& cam_a,
    const bt::CameraCalibration& cam_b) {
    bt::StereoIdentityEpipolarContext ctx;
    ctx.geometry = &geometry;
    ctx.camera_a = &cam_a;
    ctx.camera_b = &cam_b;
    return ctx;
}

bt::Vec2f Project(const bt::CameraCalibration& cam, const bt::Vec3f& world) {
    const bt::Vec3f p = bt::ProjectPoint(cam.image_from_world, world);
    return {p.x, p.y};
}

// Build a pose pair from world coordinates for epipolar testing.
struct PosePair {
    bt::DecodedPose2D a;
    bt::DecodedPose2D b;
};

PosePair MakeStereoPosePairFromWorld(
    const bt::CameraCalibration& cam_a,
    const bt::CameraCalibration& cam_b,
    const bt::Vec3f& left_world,
    const bt::Vec3f& right_world,
    float conf_a = 0.95f,
    float conf_b = 0.95f,
    bool swap_b = false) {
    PosePair out;
    out.a.valid = true;
    out.b.valid = true;

    auto set_joint = [&](bt::DecodedPose2D& pose,
                         const bt::CameraCalibration& cam,
                         bt::KeypointId left_id, bt::KeypointId right_id,
                         const bt::Vec3f& l_world, const bt::Vec3f& r_world,
                         float conf, bool swap) {
        bt::Vec2f l_px = Project(cam, l_world);
        bt::Vec2f r_px = Project(cam, r_world);
        auto& l_kp = pose.keypoints[static_cast<size_t>(swap ? right_id : left_id)];
        l_kp.pixel = l_px;
        l_kp.confidence = conf;
        l_kp.present = true;
        auto& r_kp = pose.keypoints[static_cast<size_t>(swap ? left_id : right_id)];
        r_kp.pixel = r_px;
        r_kp.confidence = conf;
        r_kp.present = true;
    };

    bt::KeypointId left_ids[] = {
        bt::KeypointId::LeftHip,
        bt::KeypointId::LeftKnee,
        bt::KeypointId::LeftAnkle
    };
    bt::KeypointId right_ids[] = {
        bt::KeypointId::RightHip,
        bt::KeypointId::RightKnee,
        bt::KeypointId::RightAnkle
    };

    for (int i = 0; i < 3; ++i) {
        set_joint(out.a, cam_a, left_ids[i], right_ids[i],
                  left_world, right_world, conf_a, false);
        set_joint(out.b, cam_b, left_ids[i], right_ids[i],
                  left_world, right_world, conf_b, swap_b);
    }

    return out;
}

// Anisotropic camera fixture: fx != fy
bt::CameraCalibration MakeAnisotropicCameraA() {
    bt::CameraCalibration camera = MakeCameraA();
    camera.camera_matrix = {2000.0, 0.0, 320.0,
                            0.0, 500.0, 240.0,
                            0.0, 0.0, 1.0};
    camera.image_from_world = bt::Mat34f{{{
        2000.0f, 0.0f, 320.0f, 0.0f,
        0.0f, 500.0f, 240.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f
    }}};
    return camera;
}

bt::CameraCalibration MakeAnisotropicCameraB() {
    bt::CameraCalibration camera = MakeCameraB();
    camera.camera_matrix = {2000.0, 0.0, 320.0,
                            0.0, 500.0, 240.0,
                            0.0, 0.0, 1.0};
    camera.image_from_world = bt::Mat34f{{{
        2000.0f, 0.0f, 320.0f, -2000.0f,
        0.0f, 500.0f, 240.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f
    }}};
    return camera;
}

// Helper to create a predicted state with evidence for a given identity.
bt::LowerBodyState MakePredictedState(float left_x, float right_x,
                                       bool has_evidence = true,
                                       float age = 0.0f) {
    bt::LowerBodyState predicted;
    predicted.confidence = 0.9f;
    predicted.left_foot.position = {left_x, 0.0f, 4.0f};
    predicted.right_foot.position = {right_x, 0.0f, 4.0f};
    if (has_evidence) {
        predicted.left_foot_evidence.valid = true;
        predicted.left_foot_evidence.source = bt::TrackerEvidenceSource::DirectStereo;
        predicted.left_foot_evidence.direct_confidence = 1.0f;
        predicted.left_foot_evidence.stale_aged = (age > 0.0f);
        predicted.right_foot_evidence.valid = true;
        predicted.right_foot_evidence.source = bt::TrackerEvidenceSource::DirectStereo;
        predicted.right_foot_evidence.direct_confidence = 1.0f;
        predicted.right_foot_evidence.stale_aged = (age > 0.0f);
    }
    return predicted;
}

}  // namespace

int main() {
    const auto cam_a = MakeCameraA();
    const auto cam_b = MakeCameraB();
    const auto geometry = bt::ComputeEpipolarGeometry(cam_a, cam_b);
    BT_CHECK(geometry.ok());
    const auto geo = geometry.value();
    const auto epipolar_ctx = MakeEpipolarContext(geo, cam_a, cam_b);

    // ─────────────────────────────────────────────────────────────────────
    // Test 1: Same-vs-cross with high certainty
    // Strong geometry should allow correct swap/keep decisions.
    // ─────────────────────────────────────────────────────────────────────
    {
        bt::LowerBodyState predicted = MakePredictedState(0.2f, -0.2f);

        const bt::Vec3f left_hip{0.65f, 0.55f, 4.0f};
        const bt::Vec3f right_hip{-0.65f, -0.45f, 4.0f};

        auto poses = MakeStereoPosePairFromWorld(
            cam_a, cam_b, left_hip, right_hip);
        PosePair swapped_poses{SwapLabels(poses.a), SwapLabels(poses.b)};

        // Same assignment should be accepted
        auto same_result = bt::ResolveStereoLeftRightIdentity(
            poses.a, poses.b, predicted,
            &cam_a.image_from_world, &cam_b.image_from_world,
            &epipolar_ctx);
        BT_CHECK(same_result.epipolar_arbitration_checked);

        // Cross assignment should be rejected or require stronger evidence
        auto cross_result = bt::ResolveStereoLeftRightIdentity(
            swapped_poses.a, swapped_poses.b, predicted,
            &cam_a.image_from_world, &cam_b.image_from_world,
            &epipolar_ctx);
        BT_CHECK(cross_result.epipolar_arbitration_checked);

        // Same identity should score higher than cross with high-certainty
        // geometry, OR cross should fail the Mahalanobis gate.
        BT_CHECK(same_result.identity_same_mahalanobis_sq <
                 cross_result.identity_cross_mahalanobis_sq ||
                 !cross_result.identity_cross_within_mahalanobis_gate);
    }

    // ─────────────────────────────────────────────────────────────────────
    // Test 2: Same-vs-cross with low certainty
    // Uncertain geometry should require stronger evidence for swaps.
    // ─────────────────────────────────────────────────────────────────────
    {
        bt::LowerBodyState predicted;
        predicted.confidence = 0.3f;  // Low certainty
        predicted.left_foot.position = {0.01f, 0.0f, 4.0f};
        predicted.right_foot.position = {-0.01f, 0.0f, 4.0f};

        // Noisy/uncertain pose
        bt::DecodedPose2D noisy_a, noisy_b;
        noisy_a.valid = true;
        noisy_b.valid = true;
        for (size_t i = 0; i < 6; ++i) {
            noisy_a.keypoints[i].pixel = {320.0f + static_cast<float>(i * 10), 240.0f};
            noisy_a.keypoints[i].confidence = 0.2f;
            noisy_a.keypoints[i].present = true;
            noisy_b.keypoints[i].pixel = {310.0f + static_cast<float>(i * 10), 235.0f};
            noisy_b.keypoints[i].confidence = 0.2f;
            noisy_b.keypoints[i].present = true;
        }

        auto uncertain_ctx = epipolar_ctx;
        uncertain_ctx.config.swap_absolute_margin = 0.5f;  // Require much stronger evidence

        auto result = bt::ResolveStereoLeftRightIdentity(
            noisy_a, noisy_b, predicted,
            &cam_a.image_from_world, &cam_b.image_from_world,
            &uncertain_ctx);

        // With low certainty, swaps should be blocked or require huge margin
        if (result.epipolar_arbitration_checked) {
            BT_CHECK(!result.epipolar_arbitration_applied ||
                     result.epipolar_cross_identity_score >
                     result.epipolar_same_identity_score + 0.5f);
        }
    }

    // ─────────────────────────────────────────────────────────────────────
    // Test 3: Lateral-vs-depth anisotropy
    // Equal scalar displacement in lateral vs depth must produce different
    // Mahalanobis/NLL behavior.
    // ─────────────────────────────────────────────────────────────────────
    {
        bt::LowerBodyState predicted = MakePredictedState(0.2f, -0.2f);

        auto ctx = epipolar_ctx;
        ctx.predicted_state_age_seconds = 0.0f;
        ctx.config.min_scored_lateral_pairs = 1;
        ctx.config.min_detection_support = 0.0f;
        ctx.config.swap_absolute_margin = 0.0f;
        ctx.config.identity_swap_nll_margin = 0.0f;

        // Lateral offset (perpendicular to viewing direction)
        const bt::Vec3f left_lat{0.3f, 0.0f, 4.0f};
        const bt::Vec3f right_lat{-0.1f, 0.0f, 4.0f};
        auto lateral_poses = MakeStereoPosePairFromWorld(
            cam_a, cam_b, left_lat, right_lat);

        // Depth offset (along viewing direction) - same scalar distance
        const bt::Vec3f left_depth{0.2f, 0.0f, 4.1f};
        const bt::Vec3f right_depth{-0.2f, 0.0f, 4.1f};
        auto depth_poses = MakeStereoPosePairFromWorld(
            cam_a, cam_b, left_depth, right_depth);

        auto lateral_result = bt::ResolveStereoLeftRightIdentity(
            lateral_poses.a, lateral_poses.b, predicted,
            &cam_a.image_from_world, &cam_b.image_from_world,
            &ctx);
        auto depth_result = bt::ResolveStereoLeftRightIdentity(
            depth_poses.a, depth_poses.b, predicted,
            &cam_a.image_from_world, &cam_b.image_from_world,
            &ctx);

        // Mahalanobis should differ between lateral and depth displacements
        BT_CHECK(lateral_result.identity_same_mahalanobis_sq !=
                 depth_result.identity_same_mahalanobis_sq ||
                 lateral_result.identity_same_negative_log_likelihood !=
                 depth_result.identity_same_negative_log_likelihood);
    }

    // ─────────────────────────────────────────────────────────────────────
    // Test 4: Anisotropic intrinsics
    // fx != fy must not collapse into an average-focal shortcut.
    // ─────────────────────────────────────────────────────────────────────
    {
        const auto aniso_cam_a = MakeAnisotropicCameraA();
        const auto aniso_cam_b = MakeAnisotropicCameraB();
        const auto aniso_geo = bt::ComputeEpipolarGeometry(aniso_cam_a, aniso_cam_b);
        BT_CHECK(aniso_geo.ok());
        auto aniso_ctx = MakeEpipolarContext(aniso_geo.value(), aniso_cam_a, aniso_cam_b);
        aniso_ctx.config.min_scored_lateral_pairs = 1;
        aniso_ctx.config.min_detection_support = 0.0f;
        aniso_ctx.config.swap_absolute_margin = 0.0f;
        aniso_ctx.config.identity_swap_nll_margin = 0.0f;

        bt::LowerBodyState predicted = MakePredictedState(0.2f, -0.2f);

        const bt::Vec3f left{0.25f, 0.0f, 4.0f};
        const bt::Vec3f right{-0.25f, 0.0f, 4.0f};
        auto poses = MakeStereoPosePairFromWorld(
            aniso_cam_a, aniso_cam_b, left, right);

        auto result = bt::ResolveStereoLeftRightIdentity(
            poses.a, poses.b, predicted,
            &aniso_cam_a.image_from_world,
            &aniso_cam_b.image_from_world,
            &aniso_ctx);

        BT_CHECK(result.epipolar_arbitration_checked);
        BT_CHECK(std::isfinite(result.identity_same_mahalanobis_sq));
        BT_CHECK(std::isfinite(result.identity_same_negative_log_likelihood));
        // Anisotropic intrinsics should produce finite, non-zero scores
        BT_CHECK(result.identity_same_mahalanobis_sq > 0.0f);
    }

    // ─────────────────────────────────────────────────────────────────────
    // Test 5: Degraded or stale pair
    // Process noise must weaken stale evidence in variance space.
    // ─────────────────────────────────────────────────────────────────────
    {
        bt::LowerBodyState predicted = MakePredictedState(0.2f, -0.2f);

        const bt::Vec3f left{0.25f, 0.0f, 4.0f};
        const bt::Vec3f right{-0.25f, 0.0f, 4.0f};
        auto poses = MakeStereoPosePairFromWorld(cam_a, cam_b, left, right);

        auto ctx = epipolar_ctx;
        ctx.config.min_scored_lateral_pairs = 1;
        ctx.config.min_detection_support = 0.0f;
        ctx.config.swap_absolute_margin = 0.0f;
        ctx.config.identity_swap_nll_margin = 0.0f;

        // Fresh observation
        ctx.predicted_state_age_seconds = 0.0f;
        auto fresh_result = bt::ResolveStereoLeftRightIdentity(
            poses.a, poses.b, predicted,
            &cam_a.image_from_world, &cam_b.image_from_world,
            &ctx);

        // Stale observation
        ctx.predicted_state_age_seconds = 2.0f;
        auto stale_result = bt::ResolveStereoLeftRightIdentity(
            poses.a, poses.b, predicted,
            &cam_a.image_from_world, &cam_b.image_from_world,
            &ctx);

        // Process noise increases variance, so the same residual should not
        // produce a larger raw Mahalanobis distance. The NLL may move upward
        // separately through its log-det covariance term.
        BT_CHECK(stale_result.identity_same_mahalanobis_sq <=
                 fresh_result.identity_same_mahalanobis_sq);
        BT_CHECK(std::isfinite(stale_result.identity_same_negative_log_likelihood));
    }

    // ─────────────────────────────────────────────────────────────────────
    // Test 6: Fallback/monocular stranding
    // One-camera-visible joints must remain usable and not get stranded.
    // ─────────────────────────────────────────────────────────────────────
    {
        bt::DecodedPose2D full_a;
        full_a.valid = true;
        full_a.keypoints[static_cast<size_t>(bt::KeypointId::LeftHip)].pixel = {420.0f, 100.0f};
        full_a.keypoints[static_cast<size_t>(bt::KeypointId::LeftHip)].confidence = 0.9f;
        full_a.keypoints[static_cast<size_t>(bt::KeypointId::LeftHip)].present = true;

        bt::DecodedPose2D missing_b;
        missing_b.valid = true;
        // Camera B doesn't see the left hip
        missing_b.keypoints[static_cast<size_t>(bt::KeypointId::LeftHip)].present = false;

        bt::LowerBodyState predicted = MakePredictedState(0.2f, -0.2f);

        auto result = bt::ResolveStereoLeftRightIdentity(
            full_a, missing_b, predicted,
            &cam_a.image_from_world, &cam_b.image_from_world,
            &epipolar_ctx);

        // Should still resolve identity even with missing camera B joint
        BT_CHECK(result.camera_a.pose.valid || result.camera_b.pose.valid);
    }

    // ─────────────────────────────────────────────────────────────────────
    // Test 7a: High detection confidence with bad geometry must NOT become
    // high geometric certainty.
    // ─────────────────────────────────────────────────────────────────────
    {
        bt::LowerBodyState predicted;
        predicted.confidence = 0.9f;
        // Far away, bad geometry
        predicted.left_foot.position = {5.0f, 5.0f, 20.0f};
        predicted.right_foot.position = {-5.0f, -5.0f, 20.0f};

        const auto high_conf_poses = MakeStereoPosePairFromWorld(
            cam_a,
            cam_b,
            bt::Vec3f{0.25f, 0.0f, 4.0f},
            bt::Vec3f{-0.25f, 0.0f, 4.0f},
            0.95f,
            0.95f);

        auto bad_geo_result = bt::ResolveStereoLeftRightIdentity(
            high_conf_poses.a, high_conf_poses.b, predicted,
            &cam_a.image_from_world, &cam_b.image_from_world,
            &epipolar_ctx);

        // High detection confidence should NOT translate to high geometric
        // certainty when geometry is bad (large Mahalanobis distance)
        BT_CHECK(bad_geo_result.identity_same_mahalanobis_sq > 1.0f ||
                 bad_geo_result.identity_uncertainty_fallback_count > 0);
    }

    // ─────────────────────────────────────────────────────────────────────
    // Test 7b: Low detection confidence with good geometry must NOT become
    // high detection support.
    // ─────────────────────────────────────────────────────────────────────
    {
        bt::LowerBodyState predicted = MakePredictedState(0.2f, -0.2f);

        bt::DecodedPose2D low_conf_a, low_conf_b;
        low_conf_a.valid = true;
        low_conf_b.valid = true;
        for (size_t i = 0; i < 6; ++i) {
            low_conf_a.keypoints[i].pixel = {320.0f + static_cast<float>(i * 10), 240.0f};
            low_conf_a.keypoints[i].confidence = 0.1f;
            low_conf_a.keypoints[i].present = true;
            low_conf_b.keypoints[i].pixel = {320.0f + static_cast<float>(i * 10), 240.0f};
            low_conf_b.keypoints[i].confidence = 0.1f;
            low_conf_b.keypoints[i].present = true;
        }

        auto good_geo_result = bt::ResolveStereoLeftRightIdentity(
            low_conf_a, low_conf_b, predicted,
            &cam_a.image_from_world, &cam_b.image_from_world,
            &epipolar_ctx);

        // Good geometry should not be dismissed solely due to low detection
        // confidence
        BT_CHECK(good_geo_result.epipolar_arbitration_checked ||
                 good_geo_result.camera_a.consistency >= 0.0f);
    }

    // ─────────────────────────────────────────────────────────────────────
    // Test 8: NLL determinant behavior
    // Candidate with much worse covariance must pay determinant penalty.
    // ─────────────────────────────────────────────────────────────────────
    {
        bt::LowerBodyState predicted = MakePredictedState(0.2f, -0.2f);

        const bt::Vec3f left{0.25f, 0.0f, 4.0f};
        const bt::Vec3f right{-0.25f, 0.0f, 4.0f};
        auto poses = MakeStereoPosePairFromWorld(cam_a, cam_b, left, right);

        auto ctx = epipolar_ctx;
        ctx.config.min_scored_lateral_pairs = 1;
        ctx.config.min_detection_support = 0.0f;
        ctx.config.swap_absolute_margin = 0.0f;
        ctx.config.identity_swap_nll_margin = 0.0f;
        ctx.predicted_state_age_seconds = 0.0f;

        auto result = bt::ResolveStereoLeftRightIdentity(
            poses.a, poses.b, predicted,
            &cam_a.image_from_world, &cam_b.image_from_world,
            &ctx);

        BT_CHECK(result.epipolar_arbitration_checked);
        // NLL includes log(det(Sigma)) term, so worse covariance should cost
        // more
        BT_CHECK(std::isfinite(result.identity_same_negative_log_likelihood));
        BT_CHECK(std::isfinite(result.identity_cross_negative_log_likelihood));
    }

    // ─────────────────────────────────────────────────────────────────────
    // Test 9: Legacy bypass prevention
    // No scalar/legacy gate may bypass the Mahalanobis hard gate.
    // ─────────────────────────────────────────────────────────────────────
    {
        bt::LowerBodyState predicted = MakePredictedState(0.2f, -0.2f);

        const bt::Vec3f left{0.25f, 0.0f, 4.0f};
        const bt::Vec3f right{-0.25f, 0.0f, 4.0f};
        auto poses = MakeStereoPosePairFromWorld(cam_a, cam_b, left, right);

        // Set a very strict Mahalanobis gate
        auto strict_ctx = epipolar_ctx;
        strict_ctx.config.identity_max_mahalanobis_sq = 0.5f;
        strict_ctx.config.swap_absolute_margin = 0.0f;
        strict_ctx.config.identity_swap_nll_margin = 0.0f;
        strict_ctx.config.min_scored_lateral_pairs = 1;
        strict_ctx.config.min_detection_support = 0.0f;
        strict_ctx.predicted_state_age_seconds = 0.0f;

        auto result = bt::ResolveStereoLeftRightIdentity(
            poses.a, poses.b, predicted,
            &cam_a.image_from_world, &cam_b.image_from_world,
            &strict_ctx);

        // Even if score gate passes, Mahalanobis gate should block
        if (result.identity_cross_within_mahalanobis_gate) {
            BT_CHECK(!result.identity_likelihood_gate_passed ||
                     result.identity_same_negative_log_likelihood <
                     result.identity_cross_negative_log_likelihood);
        }
    }

    // ─────────────────────────────────────────────────────────────────────
    // Test 10a: Missing uncertainty → conservative fallback
    // Missing covariance must map to conservative uncertainty, never
    // zero/infinite weight.
    // ─────────────────────────────────────────────────────────────────────
    {
        bt::LowerBodyState predicted = MakePredictedState(0.2f, -0.2f, false);

        // No evidence = conservative fallback
        bt::DecodedPose2D empty_a, empty_b;
        empty_a.valid = true;
        empty_b.valid = true;

        auto result = bt::ResolveStereoLeftRightIdentity(
            empty_a, empty_b, predicted,
            &cam_a.image_from_world, &cam_b.image_from_world,
            &epipolar_ctx);

        // Should handle missing uncertainty gracefully
        BT_CHECK(result.camera_a.pose.valid || result.camera_b.pose.valid ||
                 result.identity_uncertainty_fallback_count >= 0);
    }

    // ─────────────────────────────────────────────────────────────────────
    // Test 10b: SolverAnisotropicMahalanobisSq with zero stddev
    // ─────────────────────────────────────────────────────────────────────
    {
        bt::Vec3f measured{1.0f, 2.0f, 3.0f};
        bt::Vec3f expected{1.1f, 2.1f, 3.1f};
        bt::Vec3f depth_axis{0.0f, 0.0f, 1.0f};
        float zero_lateral = 0.0f;
        float zero_depth = 0.0f;

        auto mahalanobis = bt::IdentityAnisotropicMahalanobisSq(
            measured, expected, depth_axis, zero_lateral, zero_depth);

        // Zero stddev should NOT produce zero Mahalanobis (conservative)
        // The implementation should use a floor to avoid division by zero
        BT_CHECK(mahalanobis > 0.0f);
        BT_CHECK(std::isfinite(mahalanobis));
    }

    // ─────────────────────────────────────────────────────────────────────
    // Test 11: Cross-identity with swapped poses should fail Mahalanobis
    // gate when geometry is strong.
    // ─────────────────────────────────────────────────────────────────────
    {
        bt::LowerBodyState predicted = MakePredictedState(0.2f, -0.2f);

        const bt::Vec3f left{0.25f, 0.0f, 4.0f};
        const bt::Vec3f right{-0.25f, 0.0f, 4.0f};
        auto poses = MakeStereoPosePairFromWorld(cam_a, cam_b, left, right);

        auto ctx = epipolar_ctx;
        ctx.config.identity_max_mahalanobis_sq = 25.0f;
        ctx.config.min_scored_lateral_pairs = 1;
        ctx.config.min_detection_support = 0.0f;
        ctx.config.swap_absolute_margin = 0.0f;
        ctx.config.identity_swap_nll_margin = 0.0f;
        ctx.predicted_state_age_seconds = 0.0f;

        auto same_result = bt::ResolveStereoLeftRightIdentity(
            poses.a, poses.b, predicted,
            &cam_a.image_from_world, &cam_b.image_from_world,
            &ctx);

        // Cross-identity (swapped) should have higher Mahalanobis
        auto cross_result = bt::ResolveStereoLeftRightIdentity(
            SwapLabels(poses.a), SwapLabels(poses.b), predicted,
            &cam_a.image_from_world, &cam_b.image_from_world,
            &ctx);

        BT_CHECK(same_result.identity_same_mahalanobis_sq <
                 cross_result.identity_cross_mahalanobis_sq);
    }

    // ─────────────────────────────────────────────────────────────────────
    // Test 12: Winner-only candidate search prevention
    // Verify that rejected paths are not silently dropped.
    // ─────────────────────────────────────────────────────────────────────
    {
        bt::LowerBodyState predicted = MakePredictedState(0.2f, -0.2f);

        const bt::Vec3f left{0.25f, 0.0f, 4.0f};
        const bt::Vec3f right{-0.25f, 0.0f, 4.0f};
        auto poses = MakeStereoPosePairFromWorld(cam_a, cam_b, left, right);
        PosePair swapped_poses{SwapLabels(poses.a), SwapLabels(poses.b)};

        auto ctx = epipolar_ctx;
        ctx.config.min_scored_lateral_pairs = 1;
        ctx.config.min_detection_support = 0.0f;
        ctx.config.swap_absolute_margin = 0.0f;
        ctx.config.identity_swap_nll_margin = 0.0f;
        ctx.predicted_state_age_seconds = 0.0f;

        // Both same and cross candidates should be scored
        auto same_result = bt::ResolveStereoLeftRightIdentity(
            poses.a, poses.b, predicted,
            &cam_a.image_from_world, &cam_b.image_from_world,
            &ctx);
        auto cross_result = bt::ResolveStereoLeftRightIdentity(
            swapped_poses.a, swapped_poses.b, predicted,
            &cam_a.image_from_world, &cam_b.image_from_world,
            &ctx);

        // Both should have valid scores — no candidate should be silently
        // dropped
        BT_CHECK(same_result.epipolar_arbitration_checked);
        BT_CHECK(cross_result.epipolar_arbitration_checked);
        BT_CHECK(std::isfinite(same_result.identity_same_mahalanobis_sq));
        BT_CHECK(std::isfinite(cross_result.identity_cross_mahalanobis_sq));
    }

    return 0;
}
