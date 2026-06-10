#include "tracking/monocular_projection.h"
#include "test_check.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace {

std::size_t Index(bt::KeypointId id) {
    return static_cast<std::size_t>(id);
}

void Put(bt::KeypointArray& keypoints, bt::KeypointId id, float x, float y, float confidence = 0.95f) {
    auto& kp = keypoints[Index(id)];
    kp.pixel = bt::Vec2f{x, y};
    kp.confidence = confidence;
    kp.present = true;
}

std::array<float, bt::kHalpe26Count> FullWeights() {
    std::array<float, bt::kHalpe26Count> weights{};
    weights.fill(1.0f);
    return weights;
}

bt::KeypointArray GoodSingleCameraLowerBodyPose() {
    bt::KeypointArray keypoints{};
    Put(keypoints, bt::KeypointId::HeadTop, 640.0f, 120.0f);
    Put(keypoints, bt::KeypointId::Nose, 640.0f, 150.0f);
    Put(keypoints, bt::KeypointId::Neck, 640.0f, 205.0f);
    Put(keypoints, bt::KeypointId::LeftShoulder, 580.0f, 215.0f);
    Put(keypoints, bt::KeypointId::RightShoulder, 700.0f, 215.0f);
    Put(keypoints, bt::KeypointId::LeftElbow, 545.0f, 300.0f);
    Put(keypoints, bt::KeypointId::RightElbow, 735.0f, 300.0f);
    Put(keypoints, bt::KeypointId::LeftWrist, 520.0f, 385.0f);
    Put(keypoints, bt::KeypointId::RightWrist, 760.0f, 385.0f);
    Put(keypoints, bt::KeypointId::Pelvis, 640.0f, 390.0f);
    Put(keypoints, bt::KeypointId::LeftHip, 600.0f, 385.0f);
    Put(keypoints, bt::KeypointId::RightHip, 680.0f, 385.0f);
    Put(keypoints, bt::KeypointId::LeftKnee, 590.0f, 495.0f);
    Put(keypoints, bt::KeypointId::RightKnee, 690.0f, 495.0f);
    Put(keypoints, bt::KeypointId::LeftAnkle, 585.0f, 620.0f);
    Put(keypoints, bt::KeypointId::RightAnkle, 695.0f, 620.0f);
    Put(keypoints, bt::KeypointId::LeftHeel, 575.0f, 632.0f);
    Put(keypoints, bt::KeypointId::RightHeel, 705.0f, 632.0f);
    Put(keypoints, bt::KeypointId::LeftBigToe, 595.0f, 642.0f);
    Put(keypoints, bt::KeypointId::RightBigToe, 685.0f, 642.0f);
    Put(keypoints, bt::KeypointId::LeftSmallToe, 570.0f, 640.0f);
    Put(keypoints, bt::KeypointId::RightSmallToe, 710.0f, 640.0f);
    return keypoints;
}

bt::KeypointArray LowerBodyOnlyPose(bool crouched) {
    bt::KeypointArray keypoints{};
    Put(keypoints, bt::KeypointId::Pelvis, 640.0f, 390.0f);
    Put(keypoints, bt::KeypointId::LeftHip, 600.0f, 390.0f);
    Put(keypoints, bt::KeypointId::RightHip, 680.0f, 390.0f);
    if (crouched) {
        Put(keypoints, bt::KeypointId::LeftKnee, 520.0f, 540.0f);
        Put(keypoints, bt::KeypointId::RightKnee, 760.0f, 540.0f);
        Put(keypoints, bt::KeypointId::LeftAnkle, 680.0f, 575.0f);
        Put(keypoints, bt::KeypointId::RightAnkle, 600.0f, 575.0f);
    } else {
        Put(keypoints, bt::KeypointId::LeftKnee, 590.0f, 500.0f);
        Put(keypoints, bt::KeypointId::RightKnee, 690.0f, 500.0f);
        Put(keypoints, bt::KeypointId::LeftAnkle, 585.0f, 630.0f);
        Put(keypoints, bt::KeypointId::RightAnkle, 695.0f, 630.0f);
    }
    return keypoints;
}

bt::MonocularTrackingConfig TestMonocularConfig() {
    bt::MonocularTrackingConfig config;
    config.image_width = 1280;
    config.image_height = 720;
    config.horizontal_fov_deg = 70.0f;
    config.user_height_m = 1.70f;
    config.camera_height_m = 1.20f;
    config.default_depth_m = 2.20f;
    config.depth_confidence_scale = 0.55f;
    config.min_keypoint_confidence = 0.05f;
    config.min_seed_count = 4;
    return config;
}

bool FiniteVec3(const bt::Vec3f& v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

bool InvertHomography(const std::array<float, 9>& h, std::array<float, 9>& out) {
    const float det = h[0] * (h[4] * h[8] - h[5] * h[7]) -
        h[1] * (h[3] * h[8] - h[5] * h[6]) +
        h[2] * (h[3] * h[7] - h[4] * h[6]);
    if (!std::isfinite(det) || std::abs(det) <= 1.0e-8f) {
        return false;
    }
    const float inv = 1.0f / det;
    out = {
        (h[4] * h[8] - h[5] * h[7]) * inv,
        (h[2] * h[7] - h[1] * h[8]) * inv,
        (h[1] * h[5] - h[2] * h[4]) * inv,
        (h[5] * h[6] - h[3] * h[8]) * inv,
        (h[0] * h[8] - h[2] * h[6]) * inv,
        (h[2] * h[3] - h[0] * h[5]) * inv,
        (h[3] * h[7] - h[4] * h[6]) * inv,
        (h[1] * h[6] - h[0] * h[7]) * inv,
        (h[0] * h[4] - h[1] * h[3]) * inv
    };
    return true;
}

bt::Vec2f ProjectFloorPoint(const bt::MonocularProjectionProfile& profile, float x_m, float z_m, float camera_height_m) {
    return bt::Vec2f{
        profile.fx * x_m / z_m + profile.cx,
        profile.fy * camera_height_m / z_m + profile.cy
    };
}


bt::Vec2f ProjectWorldPoint(const bt::MonocularProjectionProfile& profile, const bt::Vec3f& world) {
    return bt::Vec2f{
        profile.fx * world.x / world.z + profile.cx,
        profile.cy + (profile.camera_height_m - world.y) * profile.fy / world.z
    };
}

void PutWorld(bt::KeypointArray& keypoints, bt::KeypointId id, const bt::MonocularProjectionProfile& profile, const bt::Vec3f& world, float confidence = 0.95f) {
    const auto p = ProjectWorldPoint(profile, world);
    Put(keypoints, id, p.x, p.y, confidence);
}

bt::KeypointArray SyntheticStandingWorldPose(const bt::MonocularProjectionProfile& profile, float lateral_m = 0.0f, float depth_m = 3.35f) {
    bt::KeypointArray keypoints{};
    const auto world = [&](float x, float y, float z_offset = 0.0f) {
        return bt::Vec3f{lateral_m + x, y, depth_m + z_offset};
    };

    PutWorld(keypoints, bt::KeypointId::HeadTop, profile, world(0.00f, 1.70f));
    PutWorld(keypoints, bt::KeypointId::Nose, profile, world(0.00f, 1.58f));
    PutWorld(keypoints, bt::KeypointId::Neck, profile, world(0.00f, 1.43f));
    PutWorld(keypoints, bt::KeypointId::LeftShoulder, profile, world(-0.19f, 1.38f));
    PutWorld(keypoints, bt::KeypointId::RightShoulder, profile, world(0.19f, 1.38f));
    PutWorld(keypoints, bt::KeypointId::LeftElbow, profile, world(-0.32f, 1.12f));
    PutWorld(keypoints, bt::KeypointId::RightElbow, profile, world(0.32f, 1.12f));
    PutWorld(keypoints, bt::KeypointId::LeftWrist, profile, world(-0.37f, 0.88f));
    PutWorld(keypoints, bt::KeypointId::RightWrist, profile, world(0.37f, 0.88f));
    PutWorld(keypoints, bt::KeypointId::Pelvis, profile, world(0.00f, 0.96f));
    PutWorld(keypoints, bt::KeypointId::LeftHip, profile, world(-0.13f, 0.96f));
    PutWorld(keypoints, bt::KeypointId::RightHip, profile, world(0.13f, 0.96f));
    PutWorld(keypoints, bt::KeypointId::LeftKnee, profile, world(-0.14f, 0.52f, 0.01f));
    PutWorld(keypoints, bt::KeypointId::RightKnee, profile, world(0.14f, 0.52f, 0.01f));
    PutWorld(keypoints, bt::KeypointId::LeftAnkle, profile, world(-0.14f, 0.08f, 0.02f));
    PutWorld(keypoints, bt::KeypointId::RightAnkle, profile, world(0.14f, 0.08f, 0.02f));
    PutWorld(keypoints, bt::KeypointId::LeftHeel, profile, world(-0.14f, 0.00f, -0.09f));
    PutWorld(keypoints, bt::KeypointId::RightHeel, profile, world(0.14f, 0.00f, -0.09f));
    PutWorld(keypoints, bt::KeypointId::LeftBigToe, profile, world(-0.10f, 0.00f, 0.16f));
    PutWorld(keypoints, bt::KeypointId::RightBigToe, profile, world(0.10f, 0.00f, 0.16f));
    PutWorld(keypoints, bt::KeypointId::LeftSmallToe, profile, world(-0.18f, 0.00f, 0.15f));
    PutWorld(keypoints, bt::KeypointId::RightSmallToe, profile, world(0.18f, 0.00f, 0.15f));
    return keypoints;
}

bt::KeypointArray SyntheticCrouchWorldPose(const bt::MonocularProjectionProfile& profile, float lateral_m = 0.0f, float depth_m = 3.35f) {
    bt::KeypointArray keypoints{};
    const auto world = [&](float x, float y, float z_offset = 0.0f) {
        return bt::Vec3f{lateral_m + x, y, depth_m + z_offset};
    };

    PutWorld(keypoints, bt::KeypointId::HeadTop, profile, world(0.00f, 1.34f));
    PutWorld(keypoints, bt::KeypointId::Nose, profile, world(0.00f, 1.25f));
    PutWorld(keypoints, bt::KeypointId::Neck, profile, world(0.00f, 1.12f));
    PutWorld(keypoints, bt::KeypointId::LeftShoulder, profile, world(-0.19f, 1.08f));
    PutWorld(keypoints, bt::KeypointId::RightShoulder, profile, world(0.19f, 1.08f));
    PutWorld(keypoints, bt::KeypointId::LeftElbow, profile, world(-0.32f, 0.90f));
    PutWorld(keypoints, bt::KeypointId::RightElbow, profile, world(0.32f, 0.90f));
    PutWorld(keypoints, bt::KeypointId::LeftWrist, profile, world(-0.37f, 0.70f));
    PutWorld(keypoints, bt::KeypointId::RightWrist, profile, world(0.37f, 0.70f));
    PutWorld(keypoints, bt::KeypointId::Pelvis, profile, world(0.00f, 0.58f));
    PutWorld(keypoints, bt::KeypointId::LeftHip, profile, world(-0.13f, 0.58f));
    PutWorld(keypoints, bt::KeypointId::RightHip, profile, world(0.13f, 0.58f));
    PutWorld(keypoints, bt::KeypointId::LeftKnee, profile, world(-0.18f, 0.45f, 0.22f));
    PutWorld(keypoints, bt::KeypointId::RightKnee, profile, world(0.18f, 0.45f, 0.22f));
    PutWorld(keypoints, bt::KeypointId::LeftAnkle, profile, world(-0.14f, 0.08f, 0.02f));
    PutWorld(keypoints, bt::KeypointId::RightAnkle, profile, world(0.14f, 0.08f, 0.02f));
    PutWorld(keypoints, bt::KeypointId::LeftHeel, profile, world(-0.14f, 0.00f, -0.09f));
    PutWorld(keypoints, bt::KeypointId::RightHeel, profile, world(0.14f, 0.00f, -0.09f));
    PutWorld(keypoints, bt::KeypointId::LeftBigToe, profile, world(-0.10f, 0.00f, 0.16f));
    PutWorld(keypoints, bt::KeypointId::RightBigToe, profile, world(0.10f, 0.00f, 0.16f));
    PutWorld(keypoints, bt::KeypointId::LeftSmallToe, profile, world(-0.18f, 0.00f, 0.15f));
    PutWorld(keypoints, bt::KeypointId::RightSmallToe, profile, world(0.18f, 0.00f, 0.15f));
    return keypoints;
}

bt::KeypointArray ShiftPixels(bt::KeypointArray keypoints, float dx, float dy) {
    for (auto& kp : keypoints) {
        if (kp.present) {
            kp.pixel.x += dx;
            kp.pixel.y += dy;
        }
    }
    return keypoints;
}

float MaxPresentDepthSpread(const bt::MonocularMeasurementResult& measurements) {
    float min_z = 1.0e9f;
    float max_z = -1.0e9f;
    bool any = false;
    for (const auto id : {
        bt::KeypointId::HeadTop,
        bt::KeypointId::Nose,
        bt::KeypointId::Neck,
        bt::KeypointId::LeftShoulder,
        bt::KeypointId::RightShoulder,
        bt::KeypointId::LeftElbow,
        bt::KeypointId::RightElbow,
        bt::KeypointId::Pelvis,
        bt::KeypointId::LeftHip,
        bt::KeypointId::RightHip,
        bt::KeypointId::LeftKnee,
        bt::KeypointId::RightKnee,
        bt::KeypointId::LeftAnkle,
        bt::KeypointId::RightAnkle,
        bt::KeypointId::LeftHeel,
        bt::KeypointId::RightHeel,
        bt::KeypointId::LeftBigToe,
        bt::KeypointId::RightBigToe,
        bt::KeypointId::LeftSmallToe,
        bt::KeypointId::RightSmallToe}) {
        const auto& joint = measurements.joints[Index(id)];
        if (!joint.present) {
            continue;
        }
        min_z = std::min(min_z, joint.world.z);
        max_z = std::max(max_z, joint.world.z);
        any = true;
    }
    return any ? max_z - min_z : 0.0f;
}

} // namespace

int main() {
    const auto config = TestMonocularConfig();
    const bt::CameraCalibration no_chessboard_calibration{};
    const auto keypoints = GoodSingleCameraLowerBodyPose();
    const auto weights = FullWeights();

    const auto profile = bt::MakeMonocularProjectionProfile(no_chessboard_calibration, config);
    BT_CHECK(profile.valid);
    BT_CHECK(profile.fx > 1.0f);
    BT_CHECK(profile.fy > 1.0f);

    auto below_floor_camera = config;
    below_floor_camera.camera_height_m = -0.20f;
    const auto below_floor_profile = bt::MakeMonocularProjectionProfile(no_chessboard_calibration, below_floor_camera);
    BT_CHECK(below_floor_profile.valid);
    const auto below_floor_plane = bt::MakeMonocularFloorPlane(below_floor_camera);
    BT_CHECK(bt::FloorPlaneUsable(below_floor_plane));

    const auto virtual_camera = bt::MakeMonocularCameraCalibration(no_chessboard_calibration, config);
    BT_CHECK(virtual_camera.intrinsics_valid);
    BT_CHECK(virtual_camera.extrinsics_valid);

    const auto virtual_floor = bt::MakeMonocularFloorPlane(config);
    BT_CHECK(bt::FloorPlaneUsable(virtual_floor));
    BT_CHECK_NEAR(virtual_floor.normal.y, 1.0f, 1e-6f);
    BT_CHECK_NEAR(virtual_floor.distance, 0.0f, 1e-6f);

    const auto measurements = bt::BuildMonocularJointMeasurements(
        keypoints,
        weights,
        no_chessboard_calibration,
        config);
    BT_CHECK(measurements.ok());
    BT_CHECK(measurements.value().depth_source == bt::DepthSource::InferredMonocular);
    BT_CHECK(measurements.value().scale_source == bt::MonocularScaleSource::FloorRay ||
        measurements.value().scale_source == bt::MonocularScaleSource::BodyExtent);
    BT_CHECK(measurements.value().valid_count >= config.min_seed_count);
    BT_CHECK(measurements.value().estimated_depth_m > 0.0f);
    BT_CHECK(std::isfinite(measurements.value().estimated_depth_m));
    BT_CHECK(measurements.value().mean_confidence > 0.0f);
    BT_CHECK(measurements.value().mean_confidence < 0.75f);

    const auto pelvis = measurements.value().joints[Index(bt::KeypointId::Pelvis)];
    const auto left_ankle = measurements.value().joints[Index(bt::KeypointId::LeftAnkle)];
    const auto neck = measurements.value().joints[Index(bt::KeypointId::Neck)];
    const auto left_elbow = measurements.value().joints[Index(bt::KeypointId::LeftElbow)];
    BT_CHECK(pelvis.present);
    BT_CHECK(left_ankle.present);
    BT_CHECK(neck.present);
    BT_CHECK(left_elbow.present);
    BT_CHECK(FiniteVec3(pelvis.world));
    BT_CHECK(FiniteVec3(left_ankle.world));
    BT_CHECK(FiniteVec3(neck.world));
    BT_CHECK(FiniteVec3(left_elbow.world));
    BT_CHECK(pelvis.confidence > 0.0f);
    BT_CHECK(left_elbow.confidence > 0.0f);
    BT_CHECK(pelvis.confidence < 1.0f);
    BT_CHECK(pelvis.estimated_depth_m == measurements.value().estimated_depth_m);

    // The inferred-depth measurements are explicitly distinguishable from perfect
    // stereo triangulation: they are not labeled stereo and have a confidence cap.
    BT_CHECK(measurements.value().depth_source != bt::DepthSource::TriangulatedStereo);
    BT_CHECK(measurements.value().mean_confidence <= config.depth_confidence_scale);

    const auto lower_only_standing = bt::BuildMonocularJointMeasurements(
        LowerBodyOnlyPose(false),
        weights,
        no_chessboard_calibration,
        config);
    BT_CHECK(lower_only_standing.ok());
    BT_CHECK(lower_only_standing.value().scale_source == bt::MonocularScaleSource::BodyExtent);

    const auto lower_only_crouched = bt::BuildMonocularJointMeasurements(
        LowerBodyOnlyPose(true),
        weights,
        no_chessboard_calibration,
        config);
    BT_CHECK(lower_only_crouched.ok());
    BT_CHECK(lower_only_crouched.value().scale_source != bt::MonocularScaleSource::BodyExtent);

    auto floor_config = config;
    floor_config.floor_scale_assist_enabled = true;
    floor_config.floor_depth_line_spacing_m = 0.30f;
    floor_config.floor_depth_line_spacing_px = 42.0f;
    floor_config.floor_depth_reference_y_px = 570.0f;
    floor_config.floor_depth_reference_m = 2.35f;
    floor_config.floor_depth_confidence = 0.80f;
    const auto floor_assisted = bt::BuildMonocularJointMeasurements(
        keypoints,
        weights,
        no_chessboard_calibration,
        floor_config);
    BT_CHECK(floor_assisted.ok());
    BT_CHECK(floor_assisted.value().depth_source == bt::DepthSource::InferredMonocular);
    BT_CHECK(floor_assisted.value().scale_source == bt::MonocularScaleSource::FloorSpacing);
    BT_CHECK(floor_assisted.value().floor_assist_confidence > 0.0f);
    BT_CHECK(floor_assisted.value().floor_assist_depth_m > 0.0f);
    const auto floor_toe = floor_assisted.value().joints[Index(bt::KeypointId::LeftBigToe)];
    BT_CHECK(floor_toe.present);
    BT_CHECK_NEAR(floor_toe.world.y, 0.0f, 1e-5f);
    BT_CHECK_NEAR(floor_toe.estimated_depth_m, floor_assisted.value().floor_assist_depth_m, 1e-4f);

    auto projective_floor_config = config;
    projective_floor_config.floor_scale_assist_enabled = true;
    projective_floor_config.floor_depth_line_spacing_m = 0.30f;
    projective_floor_config.floor_depth_line_spacing_px = 42.0f;
    projective_floor_config.floor_depth_reference_y_px = 570.0f;
    projective_floor_config.floor_depth_reference_m = 0.0f;
    projective_floor_config.floor_depth_confidence = 0.80f;
    const auto projective_floor_assisted = bt::BuildMonocularJointMeasurements(
        keypoints,
        weights,
        no_chessboard_calibration,
        projective_floor_config);
    BT_CHECK(projective_floor_assisted.ok());
    BT_CHECK(projective_floor_assisted.value().scale_source == bt::MonocularScaleSource::FloorSpacing);
    BT_CHECK(projective_floor_assisted.value().floor_assist_depth_m > 0.0f);
    BT_CHECK(projective_floor_assisted.value().floor_assist_depth_m < floor_assisted.value().floor_assist_depth_m);

    auto no_reference_floor_config = config;
    no_reference_floor_config.floor_scale_assist_enabled = true;
    no_reference_floor_config.floor_depth_line_spacing_m = 0.30f;
    no_reference_floor_config.floor_depth_line_spacing_px = 42.0f;
    no_reference_floor_config.floor_depth_reference_y_px = 0.0f;
    no_reference_floor_config.floor_depth_confidence = 0.80f;
    const auto no_reference_floor_assisted = bt::BuildMonocularJointMeasurements(
        keypoints,
        weights,
        no_chessboard_calibration,
        no_reference_floor_config);
    BT_CHECK(no_reference_floor_assisted.ok());
    BT_CHECK(no_reference_floor_assisted.value().scale_source == bt::MonocularScaleSource::FloorSpacing);
    BT_CHECK(no_reference_floor_assisted.value().floor_assist_depth_m > 0.0f);

    auto wall_depth_config = config;
    wall_depth_config.wall_depth_assist_enabled = true;
    wall_depth_config.wall_depth_assist_m = 3.10f;
    wall_depth_config.wall_depth_assist_confidence = 0.90f;
    const auto wall_depth_assisted = bt::BuildMonocularJointMeasurements(
        keypoints,
        weights,
        no_chessboard_calibration,
        wall_depth_config);
    BT_CHECK(wall_depth_assisted.ok());
    BT_CHECK(wall_depth_assisted.value().scale_source == bt::MonocularScaleSource::WallDepth);
    BT_CHECK(wall_depth_assisted.value().floor_assist_confidence > 0.0f);
    BT_CHECK_NEAR(wall_depth_assisted.value().floor_assist_depth_m, 3.10f, 1e-5f);
    const auto wall_depth_toe = wall_depth_assisted.value().joints[Index(bt::KeypointId::LeftBigToe)];
    BT_CHECK(wall_depth_toe.present);
    BT_CHECK(wall_depth_toe.estimated_depth_m != wall_depth_assisted.value().floor_assist_depth_m);
    BT_CHECK(std::abs(wall_depth_assisted.value().estimated_depth_m - 3.10f) <
        std::abs(measurements.value().estimated_depth_m - 3.10f));

    auto homography_keypoints = keypoints;
    const float homography_camera_height_m = 1.80f;
    const float floor_origin_x_m = -0.10f;
    const float floor_origin_z_m = 5.00f;
    const auto put_floor_contact = [&](bt::KeypointId id, float floor_x, float floor_z, float confidence = 0.95f) {
        const auto p = ProjectFloorPoint(
            profile,
            floor_origin_x_m + floor_x,
            floor_origin_z_m + floor_z,
            homography_camera_height_m);
        Put(homography_keypoints, id, p.x, p.y, confidence);
    };
    put_floor_contact(bt::KeypointId::LeftHeel, 0.08f, 0.52f);
    put_floor_contact(bt::KeypointId::LeftBigToe, 0.08f, 0.76f);
    put_floor_contact(bt::KeypointId::LeftSmallToe, 0.16f, 0.76f);
    put_floor_contact(bt::KeypointId::RightHeel, 0.30f, 0.52f);
    put_floor_contact(bt::KeypointId::RightBigToe, 0.30f, 0.76f);
    put_floor_contact(bt::KeypointId::RightSmallToe, 0.38f, 0.76f);

    auto homography_floor_config = config;
    homography_floor_config.floor_projective_homography_enabled = true;
    homography_floor_config.floor_projective_confidence = 0.90f;
    homography_floor_config.image_from_floor = {
        profile.fx,
        profile.cx,
        profile.fx * floor_origin_x_m + profile.cx * floor_origin_z_m,
        0.0f,
        profile.cy,
        profile.fy * homography_camera_height_m + profile.cy * floor_origin_z_m,
        0.0f,
        1.0f,
        floor_origin_z_m
    };
    BT_CHECK(InvertHomography(homography_floor_config.image_from_floor, homography_floor_config.floor_from_image));
    const auto homography_floor_assisted = bt::BuildMonocularJointMeasurements(
        homography_keypoints,
        weights,
        no_chessboard_calibration,
        homography_floor_config);
    BT_CHECK(homography_floor_assisted.ok());
    BT_CHECK(homography_floor_assisted.value().scale_source == bt::MonocularScaleSource::FloorProjective);
    BT_CHECK(homography_floor_assisted.value().floor_assist_depth_m > 5.30f);
    BT_CHECK(homography_floor_assisted.value().floor_assist_depth_m < 5.90f);
    const auto projective_toe = homography_floor_assisted.value().joints[Index(bt::KeypointId::LeftBigToe)];
    BT_CHECK(projective_toe.present);
    BT_CHECK_NEAR(projective_toe.world.y, 0.0f, 1e-5f);
    BT_CHECK(projective_toe.estimated_depth_m > homography_floor_assisted.value().estimated_depth_m - 0.05f);
    BT_CHECK(projective_toe.estimated_depth_m < homography_floor_assisted.value().floor_assist_depth_m);
    BT_CHECK(std::abs(projective_toe.estimated_depth_m - homography_floor_assisted.value().estimated_depth_m) < 1.25f);

    auto incomplete_foot_geometry = homography_keypoints;
    incomplete_foot_geometry[Index(bt::KeypointId::LeftHeel)] = bt::Keypoint2D{};
    incomplete_foot_geometry[Index(bt::KeypointId::RightHeel)] = bt::Keypoint2D{};
    const auto incomplete_homography = bt::BuildMonocularJointMeasurements(
        incomplete_foot_geometry,
        weights,
        no_chessboard_calibration,
        homography_floor_config);
    BT_CHECK(incomplete_homography.ok());
    BT_CHECK(incomplete_homography.value().scale_source != bt::MonocularScaleSource::FloorProjective);

    auto no_floor_contacts = homography_keypoints;
    for (const auto id : {
        bt::KeypointId::LeftHeel,
        bt::KeypointId::LeftBigToe,
        bt::KeypointId::LeftSmallToe,
        bt::KeypointId::RightHeel,
        bt::KeypointId::RightBigToe,
        bt::KeypointId::RightSmallToe}) {
        no_floor_contacts[Index(id)] = bt::Keypoint2D{};
    }
    const auto no_contact_homography = bt::BuildMonocularJointMeasurements(
        no_floor_contacts,
        weights,
        no_chessboard_calibration,
        homography_floor_config);
    BT_CHECK(no_contact_homography.ok());
    BT_CHECK(no_contact_homography.value().scale_source != bt::MonocularScaleSource::FloorProjective);
    BT_CHECK(no_contact_homography.value().floor_assist_confidence == 0.0f);

    auto bad_keypoints = keypoints;
    for (auto& kp : bad_keypoints) {
        kp.confidence = 0.01f;
    }
    std::array<float, bt::kHalpe26Count> bad_weights{};

    const auto synthetic_pose = SyntheticStandingWorldPose(profile, 0.0f, 3.35f);
    const auto synthetic_measurements = bt::BuildMonocularJointMeasurements(
        synthetic_pose,
        weights,
        no_chessboard_calibration,
        config);
    BT_CHECK(synthetic_measurements.ok());
    const auto synthetic_pelvis = synthetic_measurements.value().joints[Index(bt::KeypointId::Pelvis)];
    const auto synthetic_left_toe = synthetic_measurements.value().joints[Index(bt::KeypointId::LeftBigToe)];
    BT_CHECK(synthetic_pelvis.present);
    BT_CHECK(synthetic_left_toe.present);
    BT_CHECK(synthetic_pelvis.world.z > 2.85f);
    BT_CHECK(synthetic_pelvis.world.z < 3.85f);
    BT_CHECK(MaxPresentDepthSpread(synthetic_measurements.value()) < 0.95f);

    const auto shifted_right = bt::BuildMonocularJointMeasurements(
        ShiftPixels(synthetic_pose, 24.0f, 0.0f),
        weights,
        no_chessboard_calibration,
        config);
    BT_CHECK(shifted_right.ok());
    const auto shifted_pelvis = shifted_right.value().joints[Index(bt::KeypointId::Pelvis)];
    const float lateral_delta = shifted_pelvis.world.x - synthetic_pelvis.world.x;
    const float expected_lateral_delta = 24.0f * synthetic_pelvis.world.z / profile.fx;
    BT_CHECK(lateral_delta > 0.035f);
    BT_CHECK(lateral_delta < 0.16f);
    BT_CHECK_NEAR(lateral_delta, expected_lateral_delta, 0.06f);
    BT_CHECK(std::abs(shifted_pelvis.world.z - synthetic_pelvis.world.z) < 0.18f);

    const auto shifted_down = bt::BuildMonocularJointMeasurements(
        ShiftPixels(synthetic_pose, 0.0f, 12.0f),
        weights,
        no_chessboard_calibration,
        config);
    BT_CHECK(shifted_down.ok());
    const auto shifted_down_pelvis = shifted_down.value().joints[Index(bt::KeypointId::Pelvis)];
    BT_CHECK(std::abs(shifted_down_pelvis.world.z - synthetic_pelvis.world.z) < 0.35f);
    BT_CHECK(MaxPresentDepthSpread(shifted_down.value()) < 1.05f);

    const auto crouch_pose = SyntheticCrouchWorldPose(profile, 0.0f, 3.35f);
    const auto crouch_measurements = bt::BuildMonocularJointMeasurements(
        crouch_pose,
        weights,
        no_chessboard_calibration,
        config);
    BT_CHECK(crouch_measurements.ok());
    const auto crouch_pelvis = crouch_measurements.value().joints[Index(bt::KeypointId::Pelvis)];
    const auto crouch_left_hip = crouch_measurements.value().joints[Index(bt::KeypointId::LeftHip)];
    const auto crouch_right_hip = crouch_measurements.value().joints[Index(bt::KeypointId::RightHip)];
    const auto stand_left_hip = synthetic_measurements.value().joints[Index(bt::KeypointId::LeftHip)];
    const auto stand_right_hip = synthetic_measurements.value().joints[Index(bt::KeypointId::RightHip)];
    BT_CHECK(crouch_pelvis.present);
    BT_CHECK(crouch_pelvis.world.y < synthetic_pelvis.world.y - 0.18f);
    BT_CHECK(std::abs(crouch_pelvis.world.z - synthetic_pelvis.world.z) < 0.38f);
    const float standing_hip_span = std::abs(stand_right_hip.world.x - stand_left_hip.world.x);
    const float crouch_hip_span = std::abs(crouch_right_hip.world.x - crouch_left_hip.world.x);
    BT_CHECK(crouch_hip_span < standing_hip_span * 1.30f);

    auto hostile_floor_ray_pose = synthetic_pose;
    for (const auto id : {
        bt::KeypointId::LeftHeel,
        bt::KeypointId::RightHeel,
        bt::KeypointId::LeftBigToe,
        bt::KeypointId::RightBigToe,
        bt::KeypointId::LeftSmallToe,
        bt::KeypointId::RightSmallToe}) {
        hostile_floor_ray_pose[Index(id)].pixel.y = profile.cy + 82.0f;
    }
    const auto hostile_floor_ray = bt::BuildMonocularJointMeasurements(
        hostile_floor_ray_pose,
        weights,
        no_chessboard_calibration,
        config);
    BT_CHECK(hostile_floor_ray.ok());
    const auto hostile_pelvis = hostile_floor_ray.value().joints[Index(bt::KeypointId::Pelvis)];
    const auto hostile_toe = hostile_floor_ray.value().joints[Index(bt::KeypointId::LeftBigToe)];
    BT_CHECK(hostile_pelvis.present);
    BT_CHECK(hostile_toe.present);
    BT_CHECK(std::abs(hostile_toe.world.z - hostile_pelvis.world.z) < 1.25f);
    BT_CHECK(MaxPresentDepthSpread(hostile_floor_ray.value()) < 1.35f);

    const auto rejected = bt::BuildMonocularJointMeasurements(
        bad_keypoints,
        bad_weights,
        no_chessboard_calibration,
        config);
    BT_CHECK(!rejected.ok());

    return 0;
}
