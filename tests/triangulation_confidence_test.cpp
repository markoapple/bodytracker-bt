#include "tracking/triangulation.h"
#include "tracking/measurement_weighting.h"
#include "test_check.h"

#include <cmath>

namespace {

bt::StereoCameraModel CameraA() {
    bt::StereoCameraModel camera;
    camera.image_from_world = bt::Mat34f{{{
        1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f
    }}};
    camera.world_from_camera = bt::Mat34f{{{
        1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f
    }}};
    camera.camera_matrix = {1.0, 0.0, 0.0,
                            0.0, 1.0, 0.0,
                            0.0, 0.0, 1.0};
    camera.projection_valid = true;
    return camera;
}

bt::StereoCameraModel CameraBWithBaseline(float baseline_m) {
    bt::StereoCameraModel camera;
    camera.image_from_world = bt::Mat34f{{{
        1.0f, 0.0f, 0.0f, -baseline_m,
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f
    }}};
    camera.world_from_camera = bt::Mat34f{{{
        1.0f, 0.0f, 0.0f, baseline_m,
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f
    }}};
    camera.camera_matrix = {1.0, 0.0, 0.0,
                            0.0, 1.0, 0.0,
                            0.0, 0.0, 1.0};
    camera.projection_valid = true;
    return camera;
}

bt::StereoCameraModel CameraB() {
    return CameraBWithBaseline(1.0f);
}

bt::StereoCameraModel FocalCameraA(float focal_px) {
    bt::StereoCameraModel camera;
    camera.image_from_world = bt::Mat34f{{{
        focal_px, 0.0f, 0.0f, 0.0f,
        0.0f, focal_px, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f
    }}};
    camera.world_from_camera = CameraA().world_from_camera;
    camera.camera_matrix = {static_cast<double>(focal_px), 0.0, 0.0,
                            0.0, static_cast<double>(focal_px), 0.0,
                            0.0, 0.0, 1.0};
    camera.projection_valid = true;
    return camera;
}

bt::StereoCameraModel FocalCameraB(float focal_px, float baseline_m) {
    bt::StereoCameraModel camera;
    camera.image_from_world = bt::Mat34f{{{
        focal_px, 0.0f, 0.0f, -focal_px * baseline_m,
        0.0f, focal_px, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f
    }}};
    camera.world_from_camera = bt::Mat34f{{{
        1.0f, 0.0f, 0.0f, baseline_m,
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f
    }}};
    camera.camera_matrix = {static_cast<double>(focal_px), 0.0, 0.0,
                            0.0, static_cast<double>(focal_px), 0.0,
                            0.0, 0.0, 1.0};
    camera.projection_valid = true;
    return camera;
}

bt::TriangulatedPoint SyntheticTriangulatedPoint(bt::Vec3f world) {
    bt::TriangulatedPoint tri;
    tri.world = world;
    tri.confidence = 1.0f;
    tri.dlt_condition_number = 1.0f;
    tri.dlt_strength_ratio = 1.0f;
    tri.valid = true;
    return tri;
}

bt::StereoCameraModel HighResolutionCameraA() {
    bt::StereoCameraModel camera;
    camera.image_from_world = bt::Mat34f{{{
        2200.0f, 0.0f, 1920.0f, 0.0f,
        0.0f, 1800.0f, 1080.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f
    }}};
    camera.world_from_camera = CameraA().world_from_camera;
    camera.camera_matrix = {2200.0, 0.0, 1920.0,
                            0.0, 1800.0, 1080.0,
                            0.0, 0.0, 1.0};
    camera.projection_valid = true;
    return camera;
}

bt::StereoCameraModel HighResolutionCameraB(float baseline_m) {
    bt::StereoCameraModel camera;
    camera.image_from_world = bt::Mat34f{{{
        2200.0f, 0.0f, 1920.0f, -2200.0f * baseline_m,
        0.0f, 1800.0f, 1080.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f
    }}};
    camera.world_from_camera = bt::Mat34f{{{
        1.0f, 0.0f, 0.0f, baseline_m,
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f
    }}};
    camera.camera_matrix = {2200.0, 0.0, 1920.0,
                            0.0, 1800.0, 1080.0,
                            0.0, 0.0, 1.0};
    camera.projection_valid = true;
    return camera;
}

bt::CameraCalibration CameraCalibrationA() {
    bt::CameraCalibration camera;
    camera.intrinsics_valid = true;
    camera.extrinsics_valid = true;
    camera.camera_matrix = {1.0, 0.0, 0.0,
                            0.0, 1.0, 0.0,
                            0.0, 0.0, 1.0};
    camera.distortion = {0.0, 0.0, 0.0, 0.0, 0.0};
    camera.world_from_camera = CameraA().world_from_camera;
    camera.image_from_world = CameraA().image_from_world;
    return camera;
}

bt::CameraCalibration CameraCalibrationB() {
    bt::CameraCalibration camera;
    camera.intrinsics_valid = true;
    camera.extrinsics_valid = true;
    camera.camera_matrix = {1.0, 0.0, 0.0,
                            0.0, 1.0, 0.0,
                            0.0, 0.0, 1.0};
    camera.distortion = {0.0, 0.0, 0.0, 0.0, 0.0};
    camera.world_from_camera = CameraB().world_from_camera;
    camera.image_from_world = CameraB().image_from_world;
    return camera;
}

bt::StereoCameraObservation Obs(bt::Vec2f pixel, float confidence, float weight, float age = 1.0f) {
    bt::StereoCameraObservation out;
    out.pixel = pixel;
    out.keypoint_confidence = confidence;
    out.reliability_weight = weight;
    out.age_scale = age;
    out.present = true;
    return out;
}

bt::StereoTemporalReference TemporalLeg() {
    return bt::StereoTemporalReference{bt::Vec3f{2.0f, 0.5f, 4.0f}, 0.80f, true};
}

} // namespace

int main() {
    const float perfect = bt::StereoReprojectionConfidence(0.0f, 0.0f);
    const float mild = bt::StereoReprojectionConfidence(1.0f, 1.0f);
    const float severe = bt::StereoReprojectionConfidence(8.0f, 7.0f);

    BT_CHECK_NEAR(perfect, 1.0f, 1e-6f);
    BT_CHECK(mild < perfect);
    BT_CHECK(mild > 0.30f);
    BT_CHECK(severe < 0.07f);
    BT_CHECK_NEAR(bt::StereoReprojectionConfidence(-1.0f, 0.0f), 0.0f, 1e-6f);

    const auto status_ill_conditioned = bt::TriangulationStatus::Error(
        bt::TriangulationFailure::IllConditioned,
        bt::StatusCode::ValidationError,
        "condition number rejected");
    const auto status_degenerate_spectrum = bt::TriangulationStatus::Error(
        bt::TriangulationFailure::DegenerateSingularSpectrum,
        bt::StatusCode::ValidationError,
        "singular spectrum collapsed");
    const auto status_point_at_infinity = bt::TriangulationStatus::Error(
        bt::TriangulationFailure::PointAtInfinity,
        bt::StatusCode::ValidationError,
        "homogeneous W rejected");
    const auto status_behind_camera = bt::TriangulationStatus::Error(
        bt::TriangulationFailure::BehindCamera,
        bt::StatusCode::ValidationError,
        "behind camera");
    const auto status_projection_failed = bt::TriangulationStatus::Error(
        bt::TriangulationFailure::ProjectionFailed,
        bt::StatusCode::InternalError,
        "projection failed");
    const auto status_row_normalization_failed = bt::TriangulationStatus::Error(
        bt::TriangulationFailure::RowNormalizationFailed,
        bt::StatusCode::ValidationError,
        "row normalization failed");
    const auto status_nonfinite_reprojection = bt::TriangulationStatus::Error(
        bt::TriangulationFailure::NonFiniteReprojectionError,
        bt::StatusCode::ValidationError,
        "non-finite reprojection");
    BT_CHECK(status_ill_conditioned.is_ill_conditioned());
    BT_CHECK(status_degenerate_spectrum.is_ill_conditioned());
    BT_CHECK(status_point_at_infinity.is_ill_conditioned());
    BT_CHECK(!status_behind_camera.is_ill_conditioned());
    BT_CHECK(!status_projection_failed.is_ill_conditioned());
    BT_CHECK(!status_row_normalization_failed.is_ill_conditioned());
    BT_CHECK(!status_nonfinite_reprojection.is_ill_conditioned());

    const auto camera_a = CameraA();
    const auto camera_b = CameraB();
    const bt::Vec2f pixel_a{0.5f, 0.125f};
    const bt::Vec2f pixel_b{0.25f, 0.125f};

    const auto stereo = bt::ResolveStereoJointEvidence(
        camera_a,
        camera_b,
        Obs(pixel_a, 0.95f, 0.95f),
        Obs(pixel_b, 0.95f, 0.95f),
        TemporalLeg());
    BT_CHECK(stereo.valid);
    BT_CHECK(stereo.triangulated);
    BT_CHECK(stereo.source == bt::JointEvidenceSource::Stereo);
    BT_CHECK_NEAR(stereo.world.x, 2.0f, 1e-3f);
    BT_CHECK_NEAR(stereo.world.y, 0.5f, 1e-3f);
    BT_CHECK_NEAR(stereo.world.z, 4.0f, 1e-3f);
    BT_CHECK(stereo.triangulation_condition_number > 1.0f);
    BT_CHECK(stereo.triangulation_condition_number < 10.0f);
    BT_CHECK(stereo.triangulation_strength_ratio > 0.10f);
    BT_CHECK(stereo.triangulation_null_residual < 1e-4f);
    BT_CHECK(!stereo.triangulation_ill_conditioned);
    BT_CHECK(stereo.measurement_uncertainty_valid);
    BT_CHECK_NEAR(stereo.measurement_baseline_m, 1.0f, 1e-6f);
    BT_CHECK_NEAR(stereo.measurement_baseline_to_depth_ratio, 0.25f, 1e-5f);
    BT_CHECK(stereo.measurement_position_stddev_m > 0.0f);
    BT_CHECK(stereo.measurement_position_variance_m2 > 0.0f);

    const auto direct_tri = bt::TriangulateLinearDLT(
        camera_a.image_from_world,
        camera_b.image_from_world,
        pixel_a,
        pixel_b,
        0.95f,
        0.95f);
    BT_CHECK(direct_tri.ok());
    BT_CHECK(direct_tri.value().valid);
    BT_CHECK_NEAR(direct_tri.value().world.x, 2.0f, 1e-3f);
    BT_CHECK_NEAR(direct_tri.value().world.y, 0.5f, 1e-3f);
    BT_CHECK_NEAR(direct_tri.value().world.z, 4.0f, 1e-3f);
    BT_CHECK(direct_tri.value().dlt_condition_number > 1.0f);
    BT_CHECK(direct_tri.value().dlt_condition_number < 10.0f);
    BT_CHECK(direct_tri.value().dlt_strength_ratio > 0.10f);
    BT_CHECK(direct_tri.value().dlt_null_residual < 1e-4f);

    const auto perfect_uncertainty = bt::EstimateStereoMeasurementUncertainty(
        camera_a,
        camera_b,
        direct_tri.value(),
        0.0f,
        false);
    BT_CHECK(perfect_uncertainty.valid);
    BT_CHECK_NEAR(perfect_uncertainty.baseline_m, 1.0f, 1e-6f);
    BT_CHECK_NEAR(perfect_uncertainty.mean_depth_m, 4.0f, 1e-3f);
    BT_CHECK_NEAR(perfect_uncertainty.baseline_to_depth_ratio, 0.25f, 1e-5f);
    BT_CHECK(perfect_uncertainty.image_noise_sigma_px >= 0.35f);
    BT_CHECK(perfect_uncertainty.depth_stddev_m >= 0.005f);
    BT_CHECK(perfect_uncertainty.position_variance_m2 > 0.0f);

    const auto epipolar_inflated_uncertainty = bt::EstimateStereoMeasurementUncertainty(
        camera_a,
        camera_b,
        direct_tri.value(),
        0.40f,
        true);
    BT_CHECK(epipolar_inflated_uncertainty.valid);
    BT_CHECK(epipolar_inflated_uncertainty.image_noise_sigma_px > perfect_uncertainty.image_noise_sigma_px);
    BT_CHECK(epipolar_inflated_uncertainty.depth_stddev_m > perfect_uncertainty.depth_stddev_m);

    const auto shallow_baseline_camera_b = CameraBWithBaseline(0.25f);
    const auto shallow_baseline_tri = bt::TriangulateLinearDLT(
        camera_a.image_from_world,
        shallow_baseline_camera_b.image_from_world,
        pixel_a,
        bt::Vec2f{(2.0f - 0.25f) / 4.0f, 0.125f},
        0.95f,
        0.95f);
    BT_CHECK(shallow_baseline_tri.ok());
    const auto shallow_baseline_uncertainty = bt::EstimateStereoMeasurementUncertainty(
        camera_a,
        shallow_baseline_camera_b,
        shallow_baseline_tri.value(),
        0.0f,
        false);
    BT_CHECK(shallow_baseline_uncertainty.valid);
    BT_CHECK(shallow_baseline_uncertainty.baseline_to_depth_ratio < perfect_uncertainty.baseline_to_depth_ratio);
    BT_CHECK(shallow_baseline_uncertainty.depth_stddev_m > perfect_uncertainty.depth_stddev_m);

    const auto noisy_tri = bt::TriangulateLinearDLT(
        camera_a.image_from_world,
        camera_b.image_from_world,
        pixel_a,
        bt::Vec2f{0.25f, 0.145f},
        0.95f,
        0.95f);
    BT_CHECK(noisy_tri.ok());
    BT_CHECK(noisy_tri.value().valid);
    BT_CHECK(noisy_tri.value().reprojection_error_a + noisy_tri.value().reprojection_error_b > 0.005f);
    BT_CHECK(noisy_tri.value().dlt_condition_number > 1.0f);
    BT_CHECK(!noisy_tri.value().triangulation_ill_conditioned);
    const auto noisy_uncertainty = bt::EstimateStereoMeasurementUncertainty(
        camera_a,
        camera_b,
        noisy_tri.value(),
        0.0f,
        false);
    BT_CHECK(noisy_uncertainty.valid);
    BT_CHECK(noisy_uncertainty.image_noise_sigma_px > perfect_uncertainty.image_noise_sigma_px);

    auto residual_inflated_tri = direct_tri.value();
    residual_inflated_tri.reprojection_error_a = 0.25f;
    residual_inflated_tri.reprojection_error_b = 0.25f;
    const auto residual_inflated_uncertainty = bt::EstimateStereoMeasurementUncertainty(
        camera_a,
        camera_b,
        residual_inflated_tri,
        0.0f,
        false);
    BT_CHECK(residual_inflated_uncertainty.valid);
    BT_CHECK(residual_inflated_uncertainty.image_noise_sigma_px > perfect_uncertainty.image_noise_sigma_px);
    BT_CHECK(residual_inflated_uncertainty.position_stddev_m > perfect_uncertainty.position_stddev_m);

    auto reprojection_covers_epipolar_tri = direct_tri.value();
    reprojection_covers_epipolar_tri.reprojection_error_a = 0.50f;
    reprojection_covers_epipolar_tri.reprojection_error_b = 0.50f;
    const auto reprojection_only_uncertainty = bt::EstimateStereoMeasurementUncertainty(
        camera_a,
        camera_b,
        reprojection_covers_epipolar_tri,
        0.0f,
        false);
    const auto epipolar_subset_uncertainty = bt::EstimateStereoMeasurementUncertainty(
        camera_a,
        camera_b,
        reprojection_covers_epipolar_tri,
        0.25f,
        true);
    const auto epipolar_excess_uncertainty = bt::EstimateStereoMeasurementUncertainty(
        camera_a,
        camera_b,
        reprojection_covers_epipolar_tri,
        0.75f,
        true);
    BT_CHECK(reprojection_only_uncertainty.valid);
    BT_CHECK(epipolar_subset_uncertainty.valid);
    BT_CHECK(epipolar_excess_uncertainty.valid);
    BT_CHECK_NEAR(
        epipolar_subset_uncertainty.image_noise_sigma_px,
        reprojection_only_uncertainty.image_noise_sigma_px,
        1e-6f);
    BT_CHECK(epipolar_excess_uncertainty.image_noise_sigma_px >
             reprojection_only_uncertainty.image_noise_sigma_px);

    const auto far_tri = bt::TriangulateLinearDLT(
        camera_a.image_from_world,
        camera_b.image_from_world,
        bt::Vec2f{2.0f / 80.0f, 0.5f / 80.0f},
        bt::Vec2f{1.0f / 80.0f, 0.5f / 80.0f},
        0.95f,
        0.95f);
    BT_CHECK(far_tri.ok());
    BT_CHECK(far_tri.value().valid);
    BT_CHECK_NEAR(far_tri.value().world.z, 80.0f, 5e-2f);
    BT_CHECK(far_tri.value().dlt_condition_number < 10.0f);
    const auto far_uncertainty = bt::EstimateStereoMeasurementUncertainty(
        camera_a,
        camera_b,
        far_tri.value(),
        0.0f,
        false);
    BT_CHECK(far_uncertainty.valid);
    BT_CHECK(far_uncertainty.mean_depth_m > perfect_uncertainty.mean_depth_m);
    BT_CHECK(far_uncertainty.depth_stddev_m > perfect_uncertainty.depth_stddev_m * 10.0f);

    const auto focal_camera_a = FocalCameraA(100.0f);
    const auto focal_camera_b = FocalCameraB(100.0f, 1.0f);
    auto near_synthetic_tri = SyntheticTriangulatedPoint(bt::Vec3f{2.0f, 0.5f, 4.0f});
    near_synthetic_tri.reprojection_error_a = 1.0f;
    near_synthetic_tri.reprojection_error_b = 1.0f;
    auto doubled_depth_synthetic_tri = SyntheticTriangulatedPoint(bt::Vec3f{4.0f, 1.0f, 8.0f});
    doubled_depth_synthetic_tri.reprojection_error_a = 1.0f;
    doubled_depth_synthetic_tri.reprojection_error_b = 1.0f;
    const auto near_synthetic_uncertainty = bt::EstimateStereoMeasurementUncertainty(
        focal_camera_a,
        focal_camera_b,
        near_synthetic_tri,
        0.0f,
        false);
    const auto doubled_depth_synthetic_uncertainty = bt::EstimateStereoMeasurementUncertainty(
        focal_camera_a,
        focal_camera_b,
        doubled_depth_synthetic_tri,
        0.0f,
        false);
    BT_CHECK(near_synthetic_uncertainty.valid);
    BT_CHECK(doubled_depth_synthetic_uncertainty.valid);
    const float synthetic_depth_ratio =
        doubled_depth_synthetic_uncertainty.mean_depth_m / near_synthetic_uncertainty.mean_depth_m;
    BT_CHECK_NEAR(synthetic_depth_ratio, 2.0f, 1e-5f);
    BT_CHECK_NEAR(
        doubled_depth_synthetic_uncertainty.depth_stddev_m / near_synthetic_uncertainty.depth_stddev_m,
        synthetic_depth_ratio * synthetic_depth_ratio,
        1e-4f);
    BT_CHECK_NEAR(
        doubled_depth_synthetic_uncertainty.lateral_stddev_m / near_synthetic_uncertainty.lateral_stddev_m,
        synthetic_depth_ratio,
        1e-4f);
    BT_CHECK_NEAR(
        near_synthetic_uncertainty.depth_stddev_m / near_synthetic_uncertainty.lateral_stddev_m,
        (near_synthetic_uncertainty.mean_depth_m *
         std::sqrt(near_synthetic_uncertainty.conditioning_scale)) /
            near_synthetic_uncertainty.baseline_m,
        1e-4f);

    auto high_condition_tri = near_synthetic_tri;
    high_condition_tri.dlt_condition_number = 1.0e6f;
    high_condition_tri.dlt_strength_ratio = 1.0e-6f;
    const auto high_condition_uncertainty = bt::EstimateStereoMeasurementUncertainty(
        focal_camera_a,
        focal_camera_b,
        high_condition_tri,
        0.0f,
        false);
    BT_CHECK(high_condition_uncertainty.valid);
    BT_CHECK(high_condition_uncertainty.conditioning_scale > near_synthetic_uncertainty.conditioning_scale);
    BT_CHECK(high_condition_uncertainty.conditioning_scale <= 10.0f);

    auto severe_depth_tri = SyntheticTriangulatedPoint(bt::Vec3f{0.0f, 0.0f, 150.0f});
    severe_depth_tri.reprojection_error_a = 1.0f;
    severe_depth_tri.reprojection_error_b = 1.0f;
    auto catastrophic_depth_tri = SyntheticTriangulatedPoint(bt::Vec3f{0.0f, 0.0f, 300.0f});
    catastrophic_depth_tri.reprojection_error_a = 1.0f;
    catastrophic_depth_tri.reprojection_error_b = 1.0f;
    const auto severe_depth_uncertainty = bt::EstimateStereoMeasurementUncertainty(
        focal_camera_a,
        focal_camera_b,
        severe_depth_tri,
        0.0f,
        false);
    const auto catastrophic_depth_uncertainty = bt::EstimateStereoMeasurementUncertainty(
        focal_camera_a,
        focal_camera_b,
        catastrophic_depth_tri,
        0.0f,
        false);
    BT_CHECK(severe_depth_uncertainty.valid);
    BT_CHECK(catastrophic_depth_uncertainty.valid);
    BT_CHECK(severe_depth_uncertainty.unclamped_depth_stddev_m > severe_depth_uncertainty.depth_stddev_m);
    BT_CHECK(catastrophic_depth_uncertainty.unclamped_depth_stddev_m >
             severe_depth_uncertainty.unclamped_depth_stddev_m);
    BT_CHECK_NEAR(severe_depth_uncertainty.depth_stddev_m,
                  catastrophic_depth_uncertainty.depth_stddev_m,
                  1e-6f);

    const bt::SolverMeasurementUncertainty severe_solver_uncertainty{
        true,
        bt::Vec3f{0.0f, 0.0f, 1.0f},
        severe_depth_uncertainty.unclamped_lateral_stddev_m,
        severe_depth_uncertainty.unclamped_depth_stddev_m
    };
    const bt::SolverMeasurementUncertainty catastrophic_solver_uncertainty{
        true,
        bt::Vec3f{0.0f, 0.0f, 1.0f},
        catastrophic_depth_uncertainty.unclamped_lateral_stddev_m,
        catastrophic_depth_uncertainty.unclamped_depth_stddev_m
    };
    const auto severe_solver_info =
        bt::SolverObservationInformationFromUncertainty(severe_solver_uncertainty);
    const auto catastrophic_solver_info =
        bt::SolverObservationInformationFromUncertainty(catastrophic_solver_uncertainty);
    BT_CHECK(catastrophic_solver_info.depth_weight_scale < severe_solver_info.depth_weight_scale);

    const auto highres_camera_a = HighResolutionCameraA();
    const auto highres_camera_b = HighResolutionCameraB(0.18f);
    const bt::Vec3f highres_world{2.0f, 0.5f, 4.0f};
    const bt::Vec2f highres_pixel_a{
        (2200.0f * highres_world.x + 1920.0f * highres_world.z) / highres_world.z,
        (1800.0f * highres_world.y + 1080.0f * highres_world.z) / highres_world.z};
    const bt::Vec2f highres_pixel_b{
        (2200.0f * (highres_world.x - 0.18f) + 1920.0f * highres_world.z) / highres_world.z,
        highres_pixel_a.y};
    const auto highres_tri = bt::TriangulateLinearDLT(
        highres_camera_a.image_from_world,
        highres_camera_b.image_from_world,
        highres_pixel_a,
        highres_pixel_b,
        0.95f,
        0.95f);
    BT_CHECK(highres_tri.ok());
    BT_CHECK(highres_tri.value().valid);
    BT_CHECK_NEAR(highres_tri.value().world.x, highres_world.x, 1e-3f);
    BT_CHECK_NEAR(highres_tri.value().world.y, highres_world.y, 1e-3f);
    BT_CHECK_NEAR(highres_tri.value().world.z, highres_world.z, 1e-3f);
    BT_CHECK(!highres_tri.value().triangulation_ill_conditioned);

    const auto narrow_camera_b = CameraBWithBaseline(0.0001f);
    const auto narrow_tri = bt::TriangulateLinearDLT(
        camera_a.image_from_world,
        narrow_camera_b.image_from_world,
        pixel_a,
        bt::Vec2f{(2.0f - 0.0001f) / 4.0f, 0.125f},
        0.95f,
        0.95f);
    BT_CHECK(!narrow_tri.ok());
    BT_CHECK(narrow_tri.status().failure == bt::TriangulationFailure::IllConditioned);
    BT_CHECK(narrow_tri.status().is_ill_conditioned());

    const auto narrow_fallback = bt::ResolveStereoJointEvidence(
        camera_a,
        narrow_camera_b,
        Obs(pixel_a, 0.95f, 0.95f),
        Obs(bt::Vec2f{(2.0f - 0.0001f) / 4.0f, 0.125f}, 0.95f, 0.95f),
        TemporalLeg());
    BT_CHECK(narrow_fallback.valid);
    BT_CHECK(!narrow_fallback.triangulated);
    BT_CHECK(narrow_fallback.triangulation_ill_conditioned);
    BT_CHECK(!narrow_fallback.measurement_uncertainty_valid);
    BT_CHECK(narrow_fallback.fallback_used);

    const auto epipolar_geometry = bt::ComputeEpipolarGeometry(CameraCalibrationA(), CameraCalibrationB());
    BT_CHECK(epipolar_geometry.ok());
    bt::StereoEpipolarContext epipolar_context;
    const auto cal_a = CameraCalibrationA();
    const auto cal_b = CameraCalibrationB();
    epipolar_context.geometry = &epipolar_geometry.value();
    epipolar_context.camera_a = &cal_a;
    epipolar_context.camera_b = &cal_b;
    epipolar_context.config.soft_threshold_px = 0.10f;
    epipolar_context.config.hard_threshold_px = 0.30f;

    const auto epipolar_good = bt::ResolveStereoJointEvidence(
        camera_a,
        camera_b,
        Obs(pixel_a, 0.95f, 0.95f),
        Obs(pixel_b, 0.95f, 0.95f),
        TemporalLeg(),
        &epipolar_context);
    BT_CHECK(epipolar_good.valid);
    BT_CHECK(epipolar_good.triangulated);
    BT_CHECK(epipolar_good.epipolar_available);
    BT_CHECK(epipolar_good.epipolar_checked);
    BT_CHECK(!epipolar_good.epipolar_hard_mismatch);
    BT_CHECK(epipolar_good.epipolar_confidence > 0.95f);
    BT_CHECK(epipolar_good.epipolar_reliability_term > 0.95f);
    BT_CHECK(epipolar_good.measurement_uncertainty_valid);
    BT_CHECK_NEAR(bt::StereoPairEpipolarReliabilityScale(epipolar_good), epipolar_good.epipolar_reliability_term, 1e-6f);

    // Phase 6.5 proof-of-consumption: a valid soft epipolar mismatch must not
    // merely appear in telemetry. It must expose a reliability scale that the
    // body solver consumes when turning triangulated evidence into seed weight.
    auto epipolar_soft_context = epipolar_context;
    epipolar_soft_context.config.soft_threshold_px = 0.005f;
    epipolar_soft_context.config.hard_threshold_px = 0.50f;
    const auto epipolar_soft = bt::ResolveStereoJointEvidence(
        camera_a,
        camera_b,
        Obs(pixel_a, 0.95f, 0.95f),
        Obs(bt::Vec2f{0.25f, 0.135f}, 0.95f, 0.95f),
        TemporalLeg(),
        &epipolar_soft_context);
    BT_CHECK(epipolar_soft.valid);
    BT_CHECK(epipolar_soft.triangulated);
    BT_CHECK(epipolar_soft.epipolar_checked);
    BT_CHECK(!epipolar_soft.epipolar_hard_mismatch);
    BT_CHECK(epipolar_soft.epipolar_reliability_term < epipolar_good.epipolar_reliability_term);
    BT_CHECK(epipolar_soft.epipolar_reliability_term >= epipolar_soft_context.config.min_confidence_floor);
    BT_CHECK(epipolar_soft.measurement_uncertainty_valid);
    BT_CHECK(epipolar_soft.measurement_image_noise_sigma_px >= epipolar_good.measurement_image_noise_sigma_px);
    BT_CHECK_NEAR(bt::StereoPairEpipolarReliabilityScale(epipolar_soft), epipolar_soft.epipolar_reliability_term, 1e-6f);
    const float good_seed_confidence = bt::StereoSeedConfidence(epipolar_good, 1.0f, 1.0f);
    const float soft_seed_confidence = bt::StereoSeedConfidence(epipolar_soft, 1.0f, 1.0f);
    BT_CHECK(soft_seed_confidence < good_seed_confidence);
    BT_CHECK_NEAR(soft_seed_confidence,
                  epipolar_soft.confidence * epipolar_soft.epipolar_reliability_term,
                  1e-6f);
    BT_CHECK_NEAR(bt::StereoSeedConfidence(epipolar_soft, 0.5f, 0.25f),
                  soft_seed_confidence * 0.5f * 0.25f,
                  1e-6f);

    auto epipolar_invalid_threshold_context = epipolar_context;
    epipolar_invalid_threshold_context.config.soft_threshold_px = 0.0f;
    const auto epipolar_invalid_threshold = bt::ResolveStereoJointEvidence(
        camera_a,
        camera_b,
        Obs(pixel_a, 0.95f, 0.95f),
        Obs(pixel_b, 0.95f, 0.95f),
        TemporalLeg(),
        &epipolar_invalid_threshold_context);
    BT_CHECK(epipolar_invalid_threshold.valid);
    BT_CHECK(epipolar_invalid_threshold.triangulated);
    BT_CHECK(epipolar_invalid_threshold.epipolar_available);
    BT_CHECK(!epipolar_invalid_threshold.epipolar_checked);
    BT_CHECK(epipolar_invalid_threshold.epipolar_reason == bt::EpipolarCheckReason::InvalidThreshold);
    BT_CHECK_NEAR(bt::StereoPairEpipolarReliabilityScale(epipolar_invalid_threshold), 1.0f, 1e-6f);

    BT_CHECK_NEAR(bt::StereoPairEpipolarReliabilityScale(bt::StereoJointEvidence{}), 1.0f, 1e-6f);

    const auto epipolar_bad = bt::ResolveStereoJointEvidence(
        camera_a,
        camera_b,
        Obs(pixel_a, 0.95f, 0.95f),
        Obs(bt::Vec2f{0.25f, 1.0f}, 0.95f, 0.95f),
        TemporalLeg(),
        &epipolar_context);
    BT_CHECK(epipolar_bad.valid);
    BT_CHECK(!epipolar_bad.triangulated);
    BT_CHECK(epipolar_bad.fallback_used);
    BT_CHECK(epipolar_bad.temporal_depth_used);
    BT_CHECK(epipolar_bad.epipolar_hard_mismatch);
    BT_CHECK(epipolar_bad.epipolar_pair_rejected);
    BT_CHECK(epipolar_bad.epipolar_reliability_term > 0.0f);
    BT_CHECK_NEAR(bt::StereoPairEpipolarReliabilityScale(epipolar_bad), 1.0f, 1e-6f);
    BT_CHECK_NEAR(bt::StereoSeedConfidence(epipolar_bad, 1.0f, 1.0f), epipolar_bad.confidence, 1e-6f);
    BT_CHECK(epipolar_bad.source == bt::JointEvidenceSource::CameraAOnly ||
             epipolar_bad.source == bt::JointEvidenceSource::CameraBOnly);

    // Exact A/B no-bureaucrat check: once a hard epipolar mismatch has been
    // converted into single-camera temporal fallback, the pairwise epipolar
    // penalty must be neutral. Compare against the same fallback evidence with
    // the epipolar bookkeeping stripped off; seed confidence must be identical.
    auto fallback_without_epipolar_penalty = epipolar_bad;
    fallback_without_epipolar_penalty.epipolar_checked = false;
    fallback_without_epipolar_penalty.epipolar_hard_mismatch = false;
    fallback_without_epipolar_penalty.epipolar_pair_rejected = false;
    fallback_without_epipolar_penalty.epipolar_reliability_term = 1.0f;
    fallback_without_epipolar_penalty.epipolar_confidence = 1.0f;
    BT_CHECK(fallback_without_epipolar_penalty.fallback_used);
    BT_CHECK(!fallback_without_epipolar_penalty.triangulated);
    BT_CHECK_NEAR(bt::StereoSeedConfidence(epipolar_bad, 1.0f, 1.0f),
                  bt::StereoSeedConfidence(fallback_without_epipolar_penalty, 1.0f, 1.0f),
                  1e-6f);
    BT_CHECK_NEAR(bt::StereoSeedConfidence(epipolar_bad, 0.5f, 0.25f),
                  bt::StereoSeedConfidence(fallback_without_epipolar_penalty, 0.5f, 0.25f),
                  1e-6f);

    epipolar_context.pair_degraded = true;
    const auto degraded_bad = bt::ResolveStereoJointEvidence(
        camera_a,
        camera_b,
        Obs(pixel_a, 0.95f, 0.95f),
        Obs(bt::Vec2f{0.25f, 1.0f}, 0.95f, 0.95f),
        TemporalLeg(),
        &epipolar_context);
    BT_CHECK(degraded_bad.epipolar_hard_mismatch);
    BT_CHECK(!degraded_bad.epipolar_pair_rejected);
    BT_CHECK(degraded_bad.epipolar_reliability_term > 0.0f);
    BT_CHECK(degraded_bad.epipolar_degraded_pair_softened);

    auto reused_b_context = epipolar_context;
    reused_b_context.reused_camera_b = true;
    const auto reused_b_bad = bt::ResolveStereoJointEvidence(
        camera_a,
        camera_b,
        Obs(pixel_a, 0.95f, 0.95f),
        Obs(bt::Vec2f{0.25f, 1.0f}, 0.95f, 0.95f),
        TemporalLeg(),
        &reused_b_context);
    BT_CHECK(reused_b_bad.epipolar_hard_mismatch);
    BT_CHECK(!reused_b_bad.epipolar_pair_rejected);
    BT_CHECK(reused_b_bad.epipolar_degraded_pair_softened);

    auto degraded_reject_context = epipolar_context;
    degraded_reject_context.pair_degraded = true;
    degraded_reject_context.config.hard_mismatch_rejects_degraded_pair = true;
    const auto degraded_rejected = bt::ResolveStereoJointEvidence(
        camera_a,
        camera_b,
        Obs(pixel_a, 0.95f, 0.95f),
        Obs(bt::Vec2f{0.25f, 1.0f}, 0.95f, 0.95f),
        TemporalLeg(),
        &degraded_reject_context);
    BT_CHECK(degraded_rejected.epipolar_hard_mismatch);
    BT_CHECK(degraded_rejected.epipolar_pair_rejected);
    BT_CHECK(!degraded_rejected.epipolar_degraded_pair_softened);
    BT_CHECK(degraded_rejected.fallback_used);

    const auto b_better = bt::ResolveStereoJointEvidence(
        camera_a,
        camera_b,
        Obs(pixel_a, 0.04f, 0.03f),
        Obs(pixel_b, 0.95f, 0.95f),
        TemporalLeg());
    BT_CHECK(b_better.valid);
    BT_CHECK(!b_better.triangulated);
    BT_CHECK(b_better.depth_inferred);
    BT_CHECK(b_better.temporal_depth_used);
    BT_CHECK(b_better.fallback_used);
    BT_CHECK(b_better.source == bt::JointEvidenceSource::CameraBOnly);
    BT_CHECK(b_better.camera_b_quality > b_better.camera_a_quality);
    BT_CHECK_NEAR(b_better.world.x, 2.0f, 1e-3f);
    BT_CHECK_NEAR(b_better.world.y, 0.5f, 1e-3f);
    BT_CHECK_NEAR(b_better.world.z, 4.0f, 1e-3f);



    const auto crossed_rays = bt::ResolveStereoJointEvidence(
        camera_a,
        camera_b,
        Obs(pixel_a, 0.95f, 0.95f),
        Obs(bt::Vec2f{0.25f, 1.0f}, 0.95f, 0.95f),
        TemporalLeg());
    BT_CHECK(!crossed_rays.valid);
    BT_CHECK(crossed_rays.rejected);
    BT_CHECK(crossed_rays.source == bt::JointEvidenceSource::Rejected);

    auto a_occluded = Obs(pixel_a, 0.0f, 0.0f);    a_occluded.present = false;
    const auto rear_visible = bt::ResolveStereoJointEvidence(
        camera_a,
        camera_b,
        a_occluded,
        Obs(pixel_b, 0.90f, 0.90f),
        TemporalLeg());
    BT_CHECK(rear_visible.valid);
    BT_CHECK(rear_visible.source == bt::JointEvidenceSource::CameraBOnly);
    BT_CHECK(rear_visible.confidence > 0.0f);

    const auto stale_low = bt::ResolveStereoJointEvidence(
        camera_a,
        camera_b,
        Obs(pixel_a, 0.95f, 0.95f, 0.02f),
        Obs(pixel_b, 0.95f, 0.95f, 0.02f),
        TemporalLeg());
    BT_CHECK(!stale_low.valid);
    BT_CHECK(stale_low.rejected);
    BT_CHECK(stale_low.source == bt::JointEvidenceSource::Rejected);

    return 0;
}
