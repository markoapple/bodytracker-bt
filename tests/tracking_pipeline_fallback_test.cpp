#include "tracking/tracking_pipeline.h"
#include "test_check.h"

#include <string>

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

bt::DecodedPose2D GoodMonocularPose() {
    bt::DecodedPose2D pose;
    pose.valid = true;
    pose.aggregate_confidence = 0.90f;
    Put(pose.keypoints, bt::KeypointId::Pelvis, 640.0f, 390.0f);
    Put(pose.keypoints, bt::KeypointId::LeftHip, 600.0f, 385.0f);
    Put(pose.keypoints, bt::KeypointId::RightHip, 680.0f, 385.0f);
    Put(pose.keypoints, bt::KeypointId::LeftKnee, 590.0f, 495.0f);
    Put(pose.keypoints, bt::KeypointId::RightKnee, 690.0f, 495.0f);
    Put(pose.keypoints, bt::KeypointId::LeftAnkle, 585.0f, 620.0f);
    Put(pose.keypoints, bt::KeypointId::RightAnkle, 695.0f, 620.0f);
    Put(pose.keypoints, bt::KeypointId::LeftHeel, 575.0f, 632.0f);
    Put(pose.keypoints, bt::KeypointId::RightHeel, 705.0f, 632.0f);
    Put(pose.keypoints, bt::KeypointId::LeftBigToe, 595.0f, 642.0f);
    Put(pose.keypoints, bt::KeypointId::RightBigToe, 685.0f, 642.0f);
    Put(pose.keypoints, bt::KeypointId::LeftSmallToe, 570.0f, 640.0f);
    Put(pose.keypoints, bt::KeypointId::RightSmallToe, 710.0f, 640.0f);
    return pose;
}

bt::ReliabilitySummary GoodReliability() {
    bt::ReliabilitySummary reliability;
    for (auto& joint : reliability.joints) {
        joint.usable = true;
        joint.final_weight = 1.0f;
    }
    reliability.mean_weight = 1.0f;
    reliability.lower_body_mean = 1.0f;
    reliability.foot_mean = 1.0f;
    return reliability;
}

bool AnyValidTracker(const bt::TrackerPoseArray& trackers) {
    for (const auto& tracker : trackers) {
        if (tracker.valid) {
            return true;
        }
    }
    return false;
}

} // namespace

int main() {
    bt::CalibrationBundle calibration;
    bt::TrackingConfig config;

    bt::TrackingPipeline no_hmd_pipeline(calibration);
    no_hmd_pipeline.SetParams(config);

    bt::BodySolveInputs no_hmd_inputs;
    const auto no_hmd_step = no_hmd_pipeline.Step(no_hmd_inputs, 1.0 / 60.0);
    BT_CHECK(no_hmd_step.ok());

    const auto& no_hmd = no_hmd_step.value();
    BT_CHECK(no_hmd.degradation_mode == "occluded_untracked");
    BT_CHECK_NEAR(no_hmd.state.confidence, 0.0, 1e-6);
    BT_CHECK(!AnyValidTracker(no_hmd.trackers));
    BT_CHECK_NEAR(no_hmd.state.root.position.x, 0.0, 1e-6);
    BT_CHECK_NEAR(no_hmd.state.root.position.y, 0.0, 1e-6);
    BT_CHECK_NEAR(no_hmd.state.root.position.z, 0.0, 1e-6);

    bt::TrackingPipeline hmd_pipeline(calibration);
    hmd_pipeline.SetParams(config);

    bt::BodySolveInputs hmd_inputs;
    hmd_inputs.hmd.valid = true;
    hmd_inputs.hmd.pose.position = bt::Vec3f{1.0f, 2.0f, 3.0f};
    hmd_inputs.hmd.pose.orientation = bt::Quatf{0.0f, 0.0f, 0.0f, 1.0f};

    const auto hmd_step = hmd_pipeline.Step(hmd_inputs, 1.0 / 60.0);
    BT_CHECK(hmd_step.ok());

    const auto& hmd = hmd_step.value();
    BT_CHECK(hmd.degradation_mode == "occluded_predictive_hold");
    BT_CHECK(hmd.state.confidence > 0.0f);
    BT_CHECK(AnyValidTracker(hmd.trackers));
    BT_CHECK_NEAR(hmd.state.root.position.x, 1.0, 1e-5);
    BT_CHECK_NEAR(hmd.state.root.position.y, 1.25, 1e-5);
    BT_CHECK_NEAR(hmd.state.root.position.z, 3.0, 1e-5);

    bt::CalibrationBundle accepted_calibration;
    accepted_calibration.floor_geometry.valid = true;
    accepted_calibration.floor_geometry.source = "manual_plank";
    accepted_calibration.floor_geometry.image_width = 1280;
    accepted_calibration.floor_geometry.image_height = 720;
    accepted_calibration.floor_geometry.family_count = 1;
    accepted_calibration.floor_geometry.family_a.valid = true;
    accepted_calibration.floor_geometry.family_a.metric_spacing_valid = true;
    accepted_calibration.floor_geometry.family_a.spacing_m = 0.20f;
    accepted_calibration.floor_geometry.family_a.spacing_px = 42.0f;
    accepted_calibration.floor_geometry.homography_valid = true;
    accepted_calibration.floor_geometry.metric_scale_confidence = 0.8f;
    accepted_calibration.floor_geometry.floor_plane.valid = true;
    accepted_calibration.floor_geometry.floor_plane.normal = bt::Vec3f{0.0f, 1.0f, 0.0f};
    accepted_calibration.floor_geometry.floor_plane.distance = 0.0f;
    accepted_calibration.floor_geometry.floor_from_image = {1.0f, 0.0f, 0.0f,
                                                           0.0f, 1.0f, 0.0f,
                                                           0.0f, 0.0f, 1.0f};
    accepted_calibration.floor_geometry.image_from_floor = accepted_calibration.floor_geometry.floor_from_image;

    bt::TrackingConfig accepted_config;
    accepted_config.mode = bt::TrackingMode::Monocular;
    accepted_config.monocular.image_width = 1280;
    accepted_config.monocular.image_height = 720;

    bt::TrackingPipeline accepted_pipeline(accepted_calibration);
    accepted_pipeline.SetParams(accepted_config);
    bt::BodySolveInputs accepted_inputs;
    accepted_inputs.hmd.valid = true;
    accepted_inputs.hmd.pose.position = bt::Vec3f{0.0f, 1.5f, 0.0f};
    accepted_inputs.hmd.pose.orientation = bt::Quatf{0.0f, 0.0f, 0.0f, 1.0f};
    const auto accepted_step = accepted_pipeline.Step(accepted_inputs, 1.0 / 60.0);
    BT_CHECK(accepted_step.ok());
    BT_CHECK(accepted_step.value().floor_geometry.valid);
    BT_CHECK(accepted_step.value().floor_geometry.homography_valid);
    BT_CHECK(accepted_step.value().floor_geometry.family_a.valid);
    BT_CHECK(accepted_step.value().floor_geometry.metric_scale_confidence > 0.0f);

    bt::CalibrationBundle mismatched_calibration;
    mismatched_calibration.floor_geometry.valid = true;
    mismatched_calibration.floor_geometry.source = "manual_plank";
    mismatched_calibration.floor_geometry.image_width = 640;
    mismatched_calibration.floor_geometry.image_height = 480;
    mismatched_calibration.floor_geometry.family_count = 1;
    mismatched_calibration.floor_geometry.family_a.valid = true;
    mismatched_calibration.floor_geometry.family_a.metric_spacing_valid = true;
    mismatched_calibration.floor_geometry.family_a.spacing_m = 0.20f;
    mismatched_calibration.floor_geometry.family_a.spacing_px = 42.0f;
    mismatched_calibration.floor_geometry.homography_valid = true;
    mismatched_calibration.floor_geometry.metric_scale_confidence = 0.8f;
    mismatched_calibration.floor_geometry.floor_plane.valid = true;
    mismatched_calibration.floor_geometry.floor_plane.normal = bt::Vec3f{0.0f, 1.0f, 0.0f};
    mismatched_calibration.floor_geometry.floor_plane.distance = 0.0f;
    mismatched_calibration.floor_geometry.floor_from_image = {1.0f, 0.0f, 0.0f,
                                                             0.0f, 1.0f, 0.0f,
                                                             0.0f, 0.0f, 1.0f};
    mismatched_calibration.floor_geometry.image_from_floor = mismatched_calibration.floor_geometry.floor_from_image;

    bt::TrackingConfig monocular_config;
    monocular_config.mode = bt::TrackingMode::Monocular;
    monocular_config.monocular.image_width = 1280;
    monocular_config.monocular.image_height = 720;
    monocular_config.monocular.floor_scale_assist_enabled = true;
    monocular_config.monocular.floor_depth_line_spacing_m = 0.30f;
    monocular_config.monocular.floor_depth_line_spacing_px = 42.0f;
    monocular_config.monocular.floor_depth_reference_y_px = 570.0f;
    monocular_config.monocular.floor_depth_reference_m = 2.35f;
    monocular_config.monocular.floor_depth_confidence = 0.80f;

    bt::TrackingPipeline mismatched_pipeline(mismatched_calibration);
    mismatched_pipeline.SetParams(monocular_config);
    bt::BodySolveInputs mismatched_inputs;
    mismatched_inputs.camera_a_pose = GoodMonocularPose();
    mismatched_inputs.camera_a_reliability = GoodReliability();
    mismatched_inputs.hmd.valid = true;
    mismatched_inputs.hmd.pose.position = bt::Vec3f{0.0f, 1.5f, 0.0f};
    mismatched_inputs.hmd.pose.orientation = bt::Quatf{0.0f, 0.0f, 0.0f, 1.0f};
    const auto mismatched_step = mismatched_pipeline.Step(mismatched_inputs, 1.0 / 60.0);
    BT_CHECK(mismatched_step.ok());
    BT_CHECK(!mismatched_step.value().floor_geometry.valid);
    BT_CHECK(!mismatched_step.value().floor_geometry.family_a.valid);
    BT_CHECK(!mismatched_step.value().floor_geometry.homography_valid);
    BT_CHECK(mismatched_step.value().floor_geometry.reason.find("floor_geometry_image_size_mismatch_saved_640x480_runtime_1280x720") != std::string::npos);
    BT_CHECK(mismatched_step.value().solver.preliminary_stereo.monocular_scale_source != bt::MonocularScaleSource::FloorSpacing);
    BT_CHECK(!mismatched_step.value().solver.preliminary_stereo.floor_geometry_used);
    BT_CHECK(!mismatched_step.value().solver.final_constraints.body_calibration_present);

    bt::TrackingPipeline duplicate_sequence_pipeline(mismatched_calibration);
    duplicate_sequence_pipeline.SetParams(monocular_config);
    bt::BodySolveInputs first_sequence_inputs = mismatched_inputs;
    first_sequence_inputs.camera_a_frame_sequence = 42;
    first_sequence_inputs.camera_a_timestamp_seconds = 10.0;
    const auto first_sequence_step = duplicate_sequence_pipeline.Step(first_sequence_inputs, 1.0 / 60.0);
    BT_CHECK(first_sequence_step.ok());
    bt::BodySolveInputs duplicate_sequence_inputs = first_sequence_inputs;
    duplicate_sequence_inputs.camera_a_pose.keypoints[Index(bt::KeypointId::Pelvis)].pixel.x += 200.0f;
    duplicate_sequence_inputs.camera_a_timestamp_seconds = 10.0;
    const auto duplicate_sequence_step = duplicate_sequence_pipeline.Step(duplicate_sequence_inputs, 1.0 / 1000.0);
    BT_CHECK(duplicate_sequence_step.ok());
    BT_CHECK(duplicate_sequence_step.value().degradation_mode == "duplicate_camera_measurement_hold");
    BT_CHECK(duplicate_sequence_step.value().solver.reason == "duplicate_camera_measurement_hold");
    BT_CHECK(duplicate_sequence_step.value().last_error.find("same camera sequence already consumed") != std::string::npos);
    bt::BodySolveInputs next_sequence_inputs = duplicate_sequence_inputs;
    next_sequence_inputs.camera_a_frame_sequence = 43;
    next_sequence_inputs.camera_a_timestamp_seconds = 10.033;
    const auto next_sequence_step = duplicate_sequence_pipeline.Step(next_sequence_inputs, 1.0 / 1000.0);
    BT_CHECK(next_sequence_step.ok());
    BT_CHECK(next_sequence_step.value().degradation_mode != "duplicate_camera_measurement_hold");

    bt::TrackingPipeline duplicate_timestamp_pipeline(mismatched_calibration);
    duplicate_timestamp_pipeline.SetParams(monocular_config);
    bt::Pose3f timestamp_hmd;
    timestamp_hmd.position = bt::Vec3f{0.0f, 1.5f, 0.0f};
    timestamp_hmd.orientation = bt::Quatf{0.0f, 0.0f, 0.0f, 1.0f};
    const bt::DecodedPose2D timestamp_first_pose = GoodMonocularPose();
    const auto timestamp_first_step = duplicate_timestamp_pipeline.Step(
        timestamp_first_pose,
        bt::DecodedPose2D{},
        GoodReliability(),
        bt::ReliabilitySummary{},
        &timestamp_hmd,
        20.0);
    BT_CHECK(timestamp_first_step.ok());
    bt::DecodedPose2D timestamp_duplicate_pose = timestamp_first_pose;
    timestamp_duplicate_pose.keypoints[Index(bt::KeypointId::Pelvis)].pixel.x += 200.0f;
    const auto timestamp_duplicate_step = duplicate_timestamp_pipeline.Step(
        timestamp_duplicate_pose,
        bt::DecodedPose2D{},
        GoodReliability(),
        bt::ReliabilitySummary{},
        &timestamp_hmd,
        20.0);
    BT_CHECK(timestamp_duplicate_step.ok());
    BT_CHECK(timestamp_duplicate_step.value().degradation_mode == "duplicate_camera_measurement_hold");
    const auto timestamp_next_step = duplicate_timestamp_pipeline.Step(
        timestamp_duplicate_pose,
        bt::DecodedPose2D{},
        GoodReliability(),
        bt::ReliabilitySummary{},
        &timestamp_hmd,
        20.033);
    BT_CHECK(timestamp_next_step.ok());
    BT_CHECK(timestamp_next_step.value().degradation_mode != "duplicate_camera_measurement_hold");

    auto body_calibrated = mismatched_calibration;
    body_calibrated.body.standing_neutral_valid = true;
    body_calibrated.body.quality.overall = 0.82f;
    body_calibrated.body.quality.sample_count = 42;
    bt::TrackingPipeline body_calibrated_pipeline(body_calibrated);
    body_calibrated_pipeline.SetParams(monocular_config);
    const auto body_calibrated_step = body_calibrated_pipeline.Step(mismatched_inputs, 1.0 / 60.0);
    BT_CHECK(body_calibrated_step.ok());
    BT_CHECK(body_calibrated_step.value().solver.final_constraints.body_calibration_present);
    BT_CHECK_NEAR(body_calibrated_step.value().solver.final_constraints.body_calibration_confidence, 0.82f, 1e-5f);

    bt::CalibrationBundle wall_calibration;
    bt::WallRectangleCalibration wall;
    wall.valid = true;
    wall.image_width = 1280;
    wall.image_height = 720;
    wall.source = "manual_wall_rectangle_1";
    wall.confidence = 0.90f;
    wall.wall_orientation_valid = true;
    wall.wall_orientation_confidence = 0.85f;
    wall.wall_down_camera = bt::Vec3f{0.10f, 0.97f, 0.20f};
    wall.wall_depth_valid = true;
    wall.wall_center_depth_m = 3.10f;
    wall.wall_depth_confidence = 0.65f;
    wall_calibration.wall_rectangles.push_back(wall);

    bt::TrackingPipeline wall_pipeline(wall_calibration);
    wall_pipeline.SetParams(monocular_config);
    const auto wall_step = wall_pipeline.Step(mismatched_inputs, 1.0 / 60.0);
    BT_CHECK(wall_step.ok());
    BT_CHECK(wall_step.value().solver.preliminary_stereo.floor_camera_orientation_used);

    auto wall_depth_config = monocular_config;
    wall_depth_config.monocular.floor_scale_assist_enabled = false;
    wall_depth_config.monocular.floor_depth_line_spacing_m = 0.0f;
    wall_depth_config.monocular.floor_depth_line_spacing_px = 0.0f;
    wall_depth_config.monocular.floor_depth_reference_y_px = 0.0f;
    wall_depth_config.monocular.floor_depth_reference_m = 0.0f;
    wall_depth_config.monocular.floor_depth_confidence = 0.0f;
    bt::TrackingPipeline wall_depth_pipeline(wall_calibration);
    wall_depth_pipeline.SetParams(wall_depth_config);
    const auto wall_depth_step = wall_depth_pipeline.Step(mismatched_inputs, 1.0 / 60.0);
    BT_CHECK(wall_depth_step.ok());
    BT_CHECK(wall_depth_step.value().solver.preliminary_stereo.monocular_scale_source == bt::MonocularScaleSource::WallDepth);
    BT_CHECK(wall_depth_step.value().solver.preliminary_stereo.monocular_floor_assist_confidence > 0.0f);

    auto stale_wall_calibration = wall_calibration;
    stale_wall_calibration.wall_rectangles[0].image_width = 640;
    stale_wall_calibration.wall_rectangles[0].image_height = 480;
    bt::TrackingPipeline stale_wall_pipeline(stale_wall_calibration);
    stale_wall_pipeline.SetParams(monocular_config);
    const auto stale_wall_step = stale_wall_pipeline.Step(mismatched_inputs, 1.0 / 60.0);
    BT_CHECK(stale_wall_step.ok());
    BT_CHECK(!stale_wall_step.value().solver.preliminary_stereo.floor_camera_orientation_used);

    const auto recorded_step = mismatched_pipeline.SolveFromRecordedTrackers(bt::TrackerPoseArray{}, nullptr, 2.0);
    BT_CHECK(recorded_step.ok());
    BT_CHECK(!recorded_step.value().floor_geometry.valid);
    BT_CHECK(recorded_step.value().floor_geometry.reason == "not_used_replay_tracker_input");

    return 0;
}
