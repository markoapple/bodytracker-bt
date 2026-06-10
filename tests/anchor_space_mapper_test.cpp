#include "tracking/anchor_space_mapper.h"
#include "tracking/monocular_projection.h"
#include "test_check.h"

#include <array>
#include <cmath>

namespace {

bt::CameraCalibration MakeCamera() {
    bt::CameraCalibration camera;
    camera.intrinsics_valid = true;
    camera.extrinsics_valid = true;
    camera.camera_matrix = {100.0, 0.0, 320.0, 0.0, 100.0, 240.0, 0.0, 0.0, 1.0};
    camera.world_from_camera.m = {
        1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f
    };
    camera.image_from_world.m = {
        100.0f, 0.0f, 320.0f, 0.0f,
        0.0f, 100.0f, 240.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f
    };
    return camera;
}

bt::KeypointArray MakeKeypoints(float head_x = 320.0f, float head_y = 240.0f) {
    bt::KeypointArray keypoints{};
    auto& head = keypoints[static_cast<std::size_t>(bt::KeypointId::HeadTop)];
    head.present = true;
    head.confidence = 0.0f;
    head.pixel = bt::Vec2f{head_x, head_y};
    return keypoints;
}

bt::RawAnchorWorlds MakeRawWorlds() {
    bt::RawAnchorWorlds raw{};
    auto& head = raw[static_cast<std::size_t>(bt::KeypointId::HeadTop)];
    head.present = true;
    head.confidence = 0.0f;
    head.world = bt::Vec3f{0.0f, 0.0f, 2.0f};
    return raw;
}

bt::SteamVrAnchorFrame MakeAnchors() {
    bt::SteamVrAnchorFrame anchors;
    anchors.available = true;
    anchors.status = "connected";
    anchors.reason = "test";
    anchors.hmd_valid = true;
    anchors.hmd_world = bt::Vec3f{0.0f, 0.0f, 2.2f};
    anchors.hmd_pose.position = anchors.hmd_world;
    anchors.steamvr_timestamp_seconds = 10.0;
    return anchors;
}


bt::KeypointArray MakeMonocularKeypoints() {
    bt::KeypointArray keypoints{};
    const auto put = [&](bt::KeypointId id, float x, float y, float confidence = 0.9f) {
        auto& kp = keypoints[static_cast<std::size_t>(id)];
        kp.present = true;
        kp.confidence = confidence;
        kp.pixel = bt::Vec2f{x, y};
    };
    put(bt::KeypointId::HeadTop, 320.0f, 140.0f);
    put(bt::KeypointId::Nose, 320.0f, 170.0f);
    put(bt::KeypointId::Neck, 320.0f, 215.0f);
    put(bt::KeypointId::LeftShoulder, 285.0f, 225.0f);
    put(bt::KeypointId::RightShoulder, 355.0f, 225.0f);
    put(bt::KeypointId::LeftWrist, 250.0f, 350.0f);
    put(bt::KeypointId::RightWrist, 390.0f, 350.0f);
    put(bt::KeypointId::Pelvis, 320.0f, 365.0f);
    put(bt::KeypointId::LeftHip, 295.0f, 365.0f);
    put(bt::KeypointId::RightHip, 345.0f, 365.0f);
    put(bt::KeypointId::LeftKnee, 292.0f, 485.0f);
    put(bt::KeypointId::RightKnee, 348.0f, 485.0f);
    put(bt::KeypointId::LeftAnkle, 290.0f, 610.0f);
    put(bt::KeypointId::RightAnkle, 350.0f, 610.0f);
    return keypoints;
}

std::array<float, bt::kHalpe26Count> FullWeights() {
    std::array<float, bt::kHalpe26Count> weights{};
    weights.fill(1.0f);
    return weights;
}

bt::MonocularTrackingConfig MakeMonocularConfig() {
    bt::MonocularTrackingConfig config;
    config.image_width = 640;
    config.image_height = 480;
    config.horizontal_fov_deg = 70.0f;
    config.user_height_m = 1.7f;
    config.camera_height_m = 1.2f;
    config.default_depth_m = 2.0f;
    config.depth_confidence_scale = 1.0f;
    config.min_keypoint_confidence = 0.0f;
    config.min_seed_count = 4;
    return config;
}

} // namespace

int main() {
    const auto camera = MakeCamera();
    bt::AnchorSpaceMappingConfig config;
    config.enabled = true;
    config.timestamp_alignment_seconds = 0.10;
    config.min_depth_scale = 0.75f;
    config.max_depth_scale = 1.35f;

    const auto correction = bt::EstimateAnchorProjectionCorrection(
        config,
        camera,
        MakeKeypoints(),
        MakeRawWorlds(),
        MakeAnchors(),
        640,
        480,
        10.02);
    BT_CHECK(correction.valid);
    BT_CHECK(correction.mode == "hmd_only");
    BT_CHECK(correction.anchors_used == 1);
    BT_CHECK_NEAR(correction.depth_scale, 1.1f, 1e-5f);

    float corrected_depth = 0.0f;
    const auto corrected = bt::ApplyProjectionCorrectionToWorldPoint(
        camera,
        correction,
        bt::Vec3f{0.0f, 0.0f, 2.0f},
        &corrected_depth);
    BT_CHECK_NEAR(corrected.z, 2.2f, 1e-5f);
    BT_CHECK_NEAR(corrected_depth, 2.2f, 1e-5f);

    const auto low_conf_correction = bt::EstimateAnchorProjectionCorrection(
        config,
        camera,
        MakeKeypoints(),
        MakeRawWorlds(),
        MakeAnchors(),
        640,
        480,
        10.0);
    BT_CHECK(low_conf_correction.valid);

    const auto bad_reprojection = bt::EstimateAnchorProjectionCorrection(
        config,
        camera,
        MakeKeypoints(20.0f, 20.0f),
        MakeRawWorlds(),
        MakeAnchors(),
        640,
        480,
        10.0);
    BT_CHECK(!bad_reprojection.valid);
    BT_CHECK(bad_reprojection.fallback_reason == "anchor_reprojection_error");

    bt::RoomDepthMapTelemetry map;
    bt::RoomDepthMapConfig map_config;
    map_config.enabled = true;
    map_config.min_accepted_frames_before_active = 2;
    const auto updated = bt::UpdateRoomDepthMapTelemetry(map, map_config, correction, 11.0);
    BT_CHECK(updated.accepted_frames == 0);
    BT_CHECK(updated.rejected_frames == 1);

    bt::ProjectionCorrection strong = correction;
    strong.anchors_used = 2;
    strong.usable_for_room_map_update = true;
    const auto accepted = bt::UpdateRoomDepthMapTelemetry(updated, map_config, strong, 12.0);
    BT_CHECK(accepted.accepted_frames == 1);
    BT_CHECK(accepted.state == "partial" || accepted.state == "warming_up");


    const auto mono_keypoints = MakeMonocularKeypoints();
    const auto mono_weights = FullWeights();
    const auto mono_config = MakeMonocularConfig();
    bt::CameraCalibration mono_camera = camera;
    mono_camera.extrinsics_valid = false;
    const auto mono_anchor_camera = bt::MakeMonocularCameraCalibration(mono_camera, mono_config);
    const auto uncorrected_measurements = bt::BuildMonocularJointMeasurements(
        mono_keypoints,
        mono_weights,
        mono_camera,
        mono_config);
    BT_CHECK(uncorrected_measurements.ok());
    const auto raw_head_world = uncorrected_measurements.value().joints[static_cast<std::size_t>(bt::KeypointId::HeadTop)].world;
    const float raw_head_z = raw_head_world.z;
    BT_CHECK(raw_head_z > 0.0f);

    bt::AnchorSpaceMappingRuntimeInput mono_anchor_mapping;
    mono_anchor_mapping.config = config;
    mono_anchor_mapping.anchors = MakeAnchors();
    mono_anchor_mapping.camera_timestamp_seconds = 10.0;
    mono_anchor_mapping.now_seconds = 10.0;
    float expected_corrected_head_depth = 0.0f;
    mono_anchor_mapping.anchors.hmd_world = bt::ApplyDepthScaleInCameraSpace(
        mono_anchor_camera,
        raw_head_world,
        1.1f,
        &expected_corrected_head_depth);
    mono_anchor_mapping.anchors.hmd_pose.position = mono_anchor_mapping.anchors.hmd_world;
    const auto single_camera_measurements = bt::BuildMonocularJointMeasurements(
        mono_keypoints,
        mono_weights,
        mono_camera,
        mono_config,
        nullptr,
        &mono_anchor_mapping);
    BT_CHECK(single_camera_measurements.ok());
    BT_CHECK(single_camera_measurements.value().anchor_space_mapping.valid);
    BT_CHECK(single_camera_measurements.value().anchor_space_mapping.mode == "hmd_only");
    BT_CHECK(single_camera_measurements.value().anchor_raw_worlds[static_cast<std::size_t>(bt::KeypointId::HeadTop)].present);
    BT_CHECK(single_camera_measurements.value().anchor_correction_debug.applied[static_cast<std::size_t>(bt::KeypointId::HeadTop)]);
    const auto corrected_head_world = single_camera_measurements.value().joints[static_cast<std::size_t>(bt::KeypointId::HeadTop)].world;
    BT_CHECK_NEAR(corrected_head_world.x, mono_anchor_mapping.anchors.hmd_world.x, 1e-4f);
    BT_CHECK_NEAR(corrected_head_world.y, mono_anchor_mapping.anchors.hmd_world.y, 1e-4f);
    BT_CHECK_NEAR(corrected_head_world.z, mono_anchor_mapping.anchors.hmd_world.z, 1e-4f);
    BT_CHECK_NEAR(single_camera_measurements.value().joints[static_cast<std::size_t>(bt::KeypointId::HeadTop)].estimated_depth_m, expected_corrected_head_depth, 1e-4f);

    return 0;
}
