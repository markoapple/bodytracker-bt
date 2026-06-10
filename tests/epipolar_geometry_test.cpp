#include "tracking/epipolar_geometry.h"
#include "test_check.h"

#include <cmath>
#include <limits>
#include <string>

namespace {

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
    bt::CameraCalibration camera;
    camera.intrinsics_valid = true;
    camera.extrinsics_valid = true;
    camera.camera_matrix = {800.0, 0.0, 320.0,
                            0.0, 800.0, 240.0,
                            0.0, 0.0, 1.0};
    camera.distortion = {0.0, 0.0, 0.0, 0.0, 0.0};
    // Camera B is baseline_m metres to the right in the shared world frame,
    // with the same orientation as A. Its world-to-camera projection therefore
    // uses translation -baseline_m on X.
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

bt::CameraCalibration MakeForwardCameraB() {
    bt::CameraCalibration camera = MakeCameraA();
    camera.world_from_camera = bt::Mat34f{{{
        1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 1.0f
    }}};
    camera.image_from_world = bt::Mat34f{{{
        800.0f, 0.0f, 320.0f, 0.0f,
        0.0f, 800.0f, 240.0f, 0.0f,
        0.0f, 0.0f, 1.0f, -1.0f
    }}};
    return camera;
}

bt::Vec2f Project(const bt::CameraCalibration& camera, bt::Vec3f world) {
    const bt::Vec3f p = bt::ProjectPoint(camera.image_from_world, world);
    return bt::Vec2f{p.x, p.y};
}

bt::Vec2f ProjectNormalizedA(bt::Vec3f world) {
    return bt::Vec2f{world.x / world.z, world.y / world.z};
}

bt::Vec2f ProjectNormalizedB(bt::Vec3f world) {
    const float xb = world.x - 1.0f;
    return bt::Vec2f{xb / world.z, world.y / world.z};
}

bt::Vec2f DistortNormalizedToPixel(const bt::CameraCalibration& camera, bt::Vec2f normalized) {
    const double x = normalized.x;
    const double y = normalized.y;
    const double k1 = camera.distortion[0];
    const double k2 = camera.distortion[1];
    const double p1 = camera.distortion[2];
    const double p2 = camera.distortion[3];
    const double k3 = camera.distortion[4];
    const double r2 = x * x + y * y;
    const double r4 = r2 * r2;
    const double r6 = r4 * r2;
    const double radial = 1.0 + k1 * r2 + k2 * r4 + k3 * r6;
    const double xd = x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
    const double yd = y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;
    return bt::Vec2f{
        static_cast<float>(camera.camera_matrix[0] * xd + camera.camera_matrix[2]),
        static_cast<float>(camera.camera_matrix[4] * yd + camera.camera_matrix[5])
    };
}


bt::Mat3f ScaleMat3(const bt::Mat3f& in, float scale) {
    bt::Mat3f out = in;
    for (float& value : out.m) {
        value *= scale;
    }
    return out;
}

float Mat3FrobeniusNorm(const bt::Mat3f& in) {
    double sum_sq = 0.0;
    for (float value : in.m) {
        sum_sq += static_cast<double>(value) * static_cast<double>(value);
    }
    return static_cast<float>(std::sqrt(sum_sq));
}

bt::Mat3f IdentityMat3() {
    bt::Mat3f out{};
    out.m = {1.0f, 0.0f, 0.0f,
             0.0f, 1.0f, 0.0f,
             0.0f, 0.0f, 1.0f};
    return out;
}

bt::Mat3f RankOneMat3() {
    bt::Mat3f out{};
    out.m = {1.0f, 2.0f, 3.0f,
             2.0f, 4.0f, 6.0f,
             3.0f, 6.0f, 9.0f};
    return out;
}

bt::EpipolarGeometry ScaleGeometry(const bt::EpipolarGeometry& geometry, float scale) {
    bt::EpipolarGeometry out;
    out.fundamental_a_to_b = ScaleMat3(geometry.fundamental_a_to_b, scale);
    out.fundamental_b_to_a = ScaleMat3(geometry.fundamental_b_to_a, scale);
    out.essential_a_to_b = ScaleMat3(geometry.essential_a_to_b, scale);
    out.essential_b_to_a = ScaleMat3(geometry.essential_b_to_a, scale);
    out.diagnostics = geometry.diagnostics;
    out.valid = geometry.valid;
    return out;
}

bt::EpipolarGeometry ReverseGeometry(const bt::EpipolarGeometry& geometry) {
    bt::EpipolarGeometry reverse;
    reverse.fundamental_a_to_b = geometry.fundamental_b_to_a;
    reverse.fundamental_b_to_a = geometry.fundamental_a_to_b;
    reverse.essential_a_to_b = geometry.essential_b_to_a;
    reverse.essential_b_to_a = geometry.essential_a_to_b;
    reverse.diagnostics = geometry.diagnostics;
    reverse.valid = geometry.valid;
    return reverse;
}

} // namespace

int main() {
    const auto camera_a = MakeCameraA();
    const auto camera_b = MakeCameraB();

    const auto geometry_result = bt::ComputeEpipolarGeometry(camera_a, camera_b);
    BT_CHECK(geometry_result.ok());
    const auto geometry = geometry_result.value();
    BT_CHECK(geometry.valid);
    BT_CHECK(std::isfinite(geometry.diagnostics.camera_a_intrinsics_determinant));
    BT_CHECK(std::isfinite(geometry.diagnostics.camera_b_intrinsics_determinant));
    BT_CHECK(std::isfinite(geometry.diagnostics.camera_a_intrinsics_condition));
    BT_CHECK(std::isfinite(geometry.diagnostics.camera_b_intrinsics_condition));
    BT_CHECK(std::isfinite(geometry.diagnostics.camera_a_rotation_condition));
    BT_CHECK(std::isfinite(geometry.diagnostics.camera_b_rotation_condition));
    BT_CHECK(geometry.diagnostics.camera_a_intrinsics_determinant > 0.0f);
    BT_CHECK(geometry.diagnostics.camera_b_intrinsics_determinant > 0.0f);
    BT_CHECK(geometry.diagnostics.camera_a_intrinsics_condition > 1.0f);
    BT_CHECK(geometry.diagnostics.camera_b_intrinsics_condition > 1.0f);
    BT_CHECK(geometry.diagnostics.camera_a_rotation_condition >= 1.0f);
    BT_CHECK(geometry.diagnostics.camera_b_rotation_condition >= 1.0f);
    BT_CHECK_NEAR(geometry.diagnostics.camera_a_rotation_determinant, 1.0f, 1.0e-5f);
    BT_CHECK_NEAR(geometry.diagnostics.camera_b_rotation_determinant, 1.0f, 1.0e-5f);
    BT_CHECK_NEAR(geometry.diagnostics.relative_rotation_determinant, 1.0f, 1.0e-5f);
    BT_CHECK_NEAR(geometry.diagnostics.baseline_meters, 1.0f, 1.0e-5f);
    BT_CHECK(geometry.diagnostics.essential_a_to_b_rank_residual <= 1.0e-5f);
    BT_CHECK(geometry.diagnostics.essential_b_to_a_rank_residual <= 1.0e-5f);
    BT_CHECK(geometry.diagnostics.fundamental_a_to_b_rank_residual <= 1.0e-5f);
    BT_CHECK(geometry.diagnostics.fundamental_b_to_a_rank_residual <= 1.0e-5f);
    BT_CHECK_NEAR(Mat3FrobeniusNorm(geometry.essential_a_to_b), 1.0f, 1.0e-5f);
    BT_CHECK(Mat3FrobeniusNorm(geometry.fundamental_a_to_b) < 0.01f);

    auto reflected_camera_b = camera_b;
    reflected_camera_b.world_from_camera = bt::Mat34f{{{
        -1.0f, 0.0f, 0.0f, 1.0f,
         0.0f, 1.0f, 0.0f, 0.0f,
         0.0f, 0.0f, 1.0f, 0.0f
    }}};
    const auto reflected_result = bt::ComputeEpipolarGeometry(camera_a, reflected_camera_b);
    BT_CHECK(!reflected_result.ok());
    BT_CHECK(reflected_result.status().message.find("determinant +1") != std::string::npos);

    const bt::Vec3f world{2.0f, 0.5f, 4.0f};
    const bt::Vec2f point_a = Project(camera_a, world);
    const bt::Vec2f point_b = Project(camera_b, world);
    const auto perfect = bt::ComputePixelSampsonEpipolarCheck(geometry, point_a, point_b, 2.5f, 18.0f);
    BT_CHECK(perfect.valid);
    BT_CHECK(perfect.reason == bt::EpipolarCheckReason::Ok);
    BT_CHECK(perfect.coordinate_space == bt::EpipolarCoordinateSpace::PixelFundamental);
    BT_CHECK_NEAR(perfect.sampson_error_px, 0.0f, 1.0e-3f);
    BT_CHECK_NEAR(perfect.sampson_error_px_isotropic, perfect.sampson_error_px, 1.0e-6f);
    BT_CHECK_NEAR(perfect.sampson_error_px_anisotropic, perfect.sampson_error_px, 1.0e-6f);
    BT_CHECK(perfect.confidence > 0.999f);
    BT_CHECK(!perfect.hard_mismatch);

    const auto normalized_perfect = bt::ComputeNormalizedSampsonEpipolarCheck(
        geometry,
        ProjectNormalizedA(world),
        ProjectNormalizedB(world),
        0.0025f,
        0.018f);
    BT_CHECK(normalized_perfect.valid);
    BT_CHECK(normalized_perfect.coordinate_space == bt::EpipolarCoordinateSpace::NormalizedEssential);
    BT_CHECK_NEAR(normalized_perfect.sampson_error_px, 0.0f, 1.0e-6f);
    BT_CHECK_NEAR(normalized_perfect.sampson_error_px_isotropic, 0.0f, 1.0e-6f);
    BT_CHECK_NEAR(normalized_perfect.sampson_error_px_anisotropic, 0.0f, 1.0e-6f);
    BT_CHECK_NEAR(normalized_perfect.sampson_error_normalized, 0.0f, 1.0e-6f);

    const bt::Vec2f normalized_b_offset{
        ProjectNormalizedB(world).x,
        ProjectNormalizedB(world).y + 0.004f
    };
    const auto normalized_offset = bt::ComputeNormalizedSampsonEpipolarCheck(
        geometry,
        ProjectNormalizedA(world),
        normalized_b_offset,
        0.0025f,
        0.018f);
    BT_CHECK(normalized_offset.valid);
    BT_CHECK(normalized_offset.sampson_error_px == 0.0f);
    BT_CHECK(normalized_offset.sampson_error_normalized > 0.002f);
    BT_CHECK(normalized_offset.sampson_error_normalized < 0.004f);
    BT_CHECK(normalized_offset.confidence < normalized_perfect.confidence);
    BT_CHECK(!normalized_offset.hard_mismatch);

    const auto tiny_scaled_geometry = ScaleGeometry(geometry, 1.0e-6f);
    const auto scaled_normalized_offset = bt::ComputeNormalizedSampsonEpipolarCheck(
        tiny_scaled_geometry,
        ProjectNormalizedA(world),
        normalized_b_offset,
        0.0025f,
        0.018f);
    BT_CHECK(scaled_normalized_offset.valid);
    BT_CHECK_NEAR(scaled_normalized_offset.sampson_error_normalized, normalized_offset.sampson_error_normalized, 1.0e-5f);
    BT_CHECK_NEAR(scaled_normalized_offset.confidence, normalized_offset.confidence, 1.0e-5f);
    BT_CHECK(scaled_normalized_offset.hard_mismatch == normalized_offset.hard_mismatch);

    const auto mild = bt::ComputePixelSampsonEpipolarCheck(
        geometry,
        point_a,
        bt::Vec2f{point_b.x, point_b.y + 3.0f},
        2.5f,
        18.0f);
    BT_CHECK(mild.valid);
    BT_CHECK(mild.sampson_error_px > perfect.sampson_error_px);
    BT_CHECK(mild.confidence < perfect.confidence);
    BT_CHECK(!mild.hard_mismatch);

    const auto scaled_mild = bt::ComputePixelSampsonEpipolarCheck(
        tiny_scaled_geometry,
        point_a,
        bt::Vec2f{point_b.x, point_b.y + 3.0f},
        2.5f,
        18.0f);
    BT_CHECK(scaled_mild.valid);
    BT_CHECK_NEAR(scaled_mild.sampson_error_px, mild.sampson_error_px, 1.0e-4f);
    BT_CHECK_NEAR(scaled_mild.confidence, mild.confidence, 1.0e-5f);
    BT_CHECK(scaled_mild.hard_mismatch == mild.hard_mismatch);

    auto rank3_pixel_geometry = geometry;
    rank3_pixel_geometry.fundamental_a_to_b = IdentityMat3();
    const auto rank3_pixel_check = bt::ComputePixelSampsonEpipolarCheck(
        rank3_pixel_geometry,
        point_a,
        bt::Vec2f{point_b.x, point_b.y + 3.0f},
        2.5f,
        18.0f);
    BT_CHECK(!rank3_pixel_check.valid);
    BT_CHECK(rank3_pixel_check.reason == bt::EpipolarCheckReason::InvalidGeometry);

    auto rank3_normalized_geometry = geometry;
    rank3_normalized_geometry.essential_a_to_b = IdentityMat3();
    const auto rank3_normalized_check = bt::ComputeNormalizedSampsonEpipolarCheck(
        rank3_normalized_geometry,
        ProjectNormalizedA(world),
        normalized_b_offset,
        0.0025f,
        0.018f);
    BT_CHECK(!rank3_normalized_check.valid);
    BT_CHECK(rank3_normalized_check.reason == bt::EpipolarCheckReason::InvalidGeometry);

    auto rank1_pixel_geometry = geometry;
    rank1_pixel_geometry.fundamental_a_to_b = RankOneMat3();
    const auto rank1_pixel_check = bt::ComputePixelSampsonEpipolarCheck(
        rank1_pixel_geometry,
        point_a,
        point_b,
        2.5f,
        18.0f);
    BT_CHECK(!rank1_pixel_check.valid);
    BT_CHECK(rank1_pixel_check.reason == bt::EpipolarCheckReason::InvalidGeometry);

    const auto wide_camera_b = MakeCameraBWithBaseline(10.0f);
    const auto wide_geometry_result = bt::ComputeEpipolarGeometry(camera_a, wide_camera_b);
    BT_CHECK(wide_geometry_result.ok());
    const bt::Vec2f wide_point_b = Project(wide_camera_b, world);
    const auto wide_mild = bt::ComputePixelSampsonEpipolarCheck(
        wide_geometry_result.value(),
        point_a,
        bt::Vec2f{wide_point_b.x, wide_point_b.y + 3.0f},
        2.5f,
        18.0f);
    BT_CHECK(wide_mild.valid);
    BT_CHECK_NEAR(wide_mild.sampson_error_px, mild.sampson_error_px, 1.0e-4f);
    BT_CHECK_NEAR(wide_mild.confidence, mild.confidence, 1.0e-5f);

    const auto hard = bt::ComputePixelSampsonEpipolarCheck(
        geometry,
        point_a,
        bt::Vec2f{point_b.x, point_b.y + 40.0f},
        2.5f,
        18.0f);
    BT_CHECK(hard.valid);
    BT_CHECK(hard.sampson_error_px > mild.sampson_error_px);
    BT_CHECK(hard.confidence < mild.confidence);
    BT_CHECK(hard.hard_mismatch);

    const auto tiny_soft_threshold = bt::ComputePixelSampsonEpipolarCheck(
        geometry,
        point_a,
        bt::Vec2f{point_b.x, point_b.y + 40.0f},
        std::numeric_limits<float>::min(),
        18.0f);
    BT_CHECK(tiny_soft_threshold.valid);
    BT_CHECK(std::isfinite(tiny_soft_threshold.confidence));
    BT_CHECK(tiny_soft_threshold.confidence >= 0.0f);
    BT_CHECK(tiny_soft_threshold.confidence <= 1.0f);

    const auto reverse = bt::ComputePixelSampsonEpipolarCheck(
        ReverseGeometry(geometry),
        point_b,
        point_a,
        2.5f,
        18.0f);
    BT_CHECK(reverse.valid);
    BT_CHECK_NEAR(reverse.sampson_error_px, perfect.sampson_error_px, 1.0e-3f);

    auto distorted_a = camera_a;
    auto distorted_b = camera_b;
    distorted_a.distortion = {0.25, -0.08, 0.002, -0.001, 0.015};
    distorted_b.distortion = {0.25, -0.08, 0.002, -0.001, 0.015};
    const auto distorted_geometry = bt::ComputeEpipolarGeometry(distorted_a, distorted_b);
    BT_CHECK(distorted_geometry.ok());
    const bt::Vec3f off_axis_world{2.9f, 1.25f, 3.2f};
    const bt::Vec2f distorted_point_a = DistortNormalizedToPixel(distorted_a, ProjectNormalizedA(off_axis_world));
    const bt::Vec2f distorted_point_b = DistortNormalizedToPixel(distorted_b, ProjectNormalizedB(off_axis_world));
    const auto raw_distorted_pixel_check = bt::ComputePixelSampsonEpipolarCheck(
        distorted_geometry.value(),
        distorted_point_a,
        distorted_point_b,
        2.5f,
        18.0f);
    const auto distortion_safe_check = bt::ComputeDistortionSafePixelSampsonEpipolarCheck(
        distorted_geometry.value(),
        distorted_a,
        distorted_b,
        distorted_point_a,
        distorted_point_b,
        2.5f,
        18.0f);
    BT_CHECK(raw_distorted_pixel_check.valid);
    BT_CHECK(distortion_safe_check.valid);
    BT_CHECK(distortion_safe_check.coordinate_space == bt::EpipolarCoordinateSpace::NormalizedEssential);
    BT_CHECK_NEAR(distortion_safe_check.sampson_error_px, distortion_safe_check.sampson_error_px_isotropic, 1.0e-6f);
    BT_CHECK(distortion_safe_check.sampson_error_px_anisotropic >= 0.0f);
    BT_CHECK(distortion_safe_check.sampson_error_px < 0.05f);
    BT_CHECK(raw_distorted_pixel_check.sampson_error_px > distortion_safe_check.sampson_error_px + 0.5f);

    const auto anisotropic_a = MakeAnisotropicCameraA();
    const auto anisotropic_b = MakeAnisotropicCameraB();
    const auto anisotropic_geometry = bt::ComputeEpipolarGeometry(anisotropic_a, anisotropic_b);
    BT_CHECK(anisotropic_geometry.ok());
    const bt::Vec2f anisotropic_point_a = Project(anisotropic_a, world);
    const bt::Vec2f anisotropic_point_b = Project(anisotropic_b, world);
    const bt::Vec2f anisotropic_point_b_offset{anisotropic_point_b.x, anisotropic_point_b.y + 2.0f};
    const auto anisotropic_raw_pixel = bt::ComputePixelSampsonEpipolarCheck(
        anisotropic_geometry.value(),
        anisotropic_point_a,
        anisotropic_point_b_offset,
        2.5f,
        18.0f);
    const auto anisotropic_safe = bt::ComputeDistortionSafePixelSampsonEpipolarCheck(
        anisotropic_geometry.value(),
        anisotropic_a,
        anisotropic_b,
        anisotropic_point_a,
        anisotropic_point_b_offset,
        2.5f,
        18.0f);
    BT_CHECK(anisotropic_raw_pixel.valid);
    BT_CHECK(anisotropic_safe.valid);
    BT_CHECK_NEAR(anisotropic_safe.sampson_error_px, anisotropic_safe.sampson_error_px_isotropic, 1.0e-6f);
    BT_CHECK(anisotropic_safe.sampson_error_px_isotropic > anisotropic_safe.sampson_error_px_anisotropic * 2.0f);
    BT_CHECK_NEAR(anisotropic_safe.sampson_error_px_anisotropic, anisotropic_raw_pixel.sampson_error_px, 1.0e-3f);
    BT_CHECK(anisotropic_safe.sampson_error_normalized > 0.0f);

    const auto undistorted_a = bt::UndistortPixelToNormalized(distorted_a, distorted_point_a);
    const auto undistorted_b = bt::UndistortPixelToNormalized(distorted_b, distorted_point_b);
    BT_CHECK(undistorted_a.ok());
    BT_CHECK(undistorted_b.ok());
    BT_CHECK_NEAR(undistorted_a.value().x, ProjectNormalizedA(off_axis_world).x, 1.0e-4f);
    BT_CHECK_NEAR(undistorted_a.value().y, ProjectNormalizedA(off_axis_world).y, 1.0e-4f);
    BT_CHECK_NEAR(undistorted_b.value().x, ProjectNormalizedB(off_axis_world).x, 1.0e-4f);
    BT_CHECK_NEAR(undistorted_b.value().y, ProjectNormalizedB(off_axis_world).y, 1.0e-4f);

    auto missing_intrinsics = camera_a;
    missing_intrinsics.intrinsics_valid = false;
    BT_CHECK(!bt::ComputeEpipolarGeometry(missing_intrinsics, camera_b).ok());
    BT_CHECK(!bt::UndistortPixelToNormalized(missing_intrinsics, point_a).ok());

    auto sub_pixel_focal = camera_a;
    sub_pixel_focal.camera_matrix = {0.5, 0.0, 320.0,
                                     0.0, 0.5, 240.0,
                                     0.0, 0.0, 1.0};
    BT_CHECK(!bt::ComputeEpipolarGeometry(sub_pixel_focal, camera_b).ok());
    BT_CHECK(!bt::UndistortPixelToNormalized(sub_pixel_focal, point_a).ok());

    auto near_zero_focal = camera_a;
    near_zero_focal.camera_matrix = {1.0e-8, 0.0, 320.0,
                                     0.0, 1.0e-8, 240.0,
                                     0.0, 0.0, 1.0};
    BT_CHECK(!bt::ComputeEpipolarGeometry(near_zero_focal, camera_b).ok());
    BT_CHECK(!bt::UndistortPixelToNormalized(near_zero_focal, point_a).ok());

    auto singular_intrinsics = camera_a;
    singular_intrinsics.camera_matrix = {800.0, 0.0, 320.0,
                                         0.0, 800.0, 240.0,
                                         0.0, 0.0, 0.0};
    BT_CHECK(!bt::ComputeEpipolarGeometry(singular_intrinsics, camera_b).ok());
    BT_CHECK(!bt::UndistortPixelToNormalized(singular_intrinsics, point_a).ok());

    auto ill_conditioned_intrinsics = camera_a;
    ill_conditioned_intrinsics.camera_matrix = {800.0, 0.0, 1000000.0,
                                                0.0, 800.0, 1000000.0,
                                                0.0, 0.0, 1.0};
    const auto ill_conditioned_intrinsics_result = bt::ComputeEpipolarGeometry(ill_conditioned_intrinsics, camera_b);
    BT_CHECK(!ill_conditioned_intrinsics_result.ok());
    BT_CHECK(ill_conditioned_intrinsics_result.status().message.find("well-conditioned") != std::string::npos);
    BT_CHECK(!bt::UndistortPixelToNormalized(ill_conditioned_intrinsics, point_a).ok());

    auto ill_conditioned_extrinsics = camera_b;
    ill_conditioned_extrinsics.world_from_camera = bt::Mat34f{{{
        1.0f, 0.0f, 0.0f, 1.0f,
        0.0f, 1.0e-8f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f
    }}};
    const auto ill_conditioned_extrinsics_result = bt::ComputeEpipolarGeometry(camera_a, ill_conditioned_extrinsics);
    BT_CHECK(!ill_conditioned_extrinsics_result.ok());
    BT_CHECK(ill_conditioned_extrinsics_result.status().message.find("well-conditioned extrinsics") != std::string::npos);

    auto bad_distortion = camera_a;
    bad_distortion.distortion[0] = NAN;
    BT_CHECK(!bt::ComputeEpipolarGeometry(bad_distortion, camera_b).ok());
    BT_CHECK(!bt::UndistortPixelToNormalized(bad_distortion, point_a).ok());

    auto zero_baseline = camera_a;
    zero_baseline.extrinsics_valid = true;
    zero_baseline.world_from_camera = camera_a.world_from_camera;
    BT_CHECK(!bt::ComputeEpipolarGeometry(camera_a, zero_baseline).ok());

    const auto near_zero_baseline_result = bt::ComputeEpipolarGeometry(camera_a, MakeCameraBWithBaseline(0.0005f));
    BT_CHECK(!near_zero_baseline_result.ok());
    BT_CHECK(near_zero_baseline_result.status().message.find("1 mm") != std::string::npos);
    BT_CHECK(bt::ComputeEpipolarGeometry(camera_a, MakeCameraBWithBaseline(0.002f)).ok());

    const auto invalid_point = bt::ComputePixelSampsonEpipolarCheck(
        geometry,
        bt::Vec2f{NAN, point_a.y},
        point_b,
        2.5f,
        18.0f);
    BT_CHECK(!invalid_point.valid);
    BT_CHECK(invalid_point.reason == bt::EpipolarCheckReason::NonFinitePoint);

    const auto invalid_threshold = bt::ComputeDistortionSafePixelSampsonEpipolarCheck(
        geometry,
        camera_a,
        camera_b,
        point_a,
        point_b,
        0.0f,
        18.0f);
    BT_CHECK(!invalid_threshold.valid);
    BT_CHECK(invalid_threshold.reason == bt::EpipolarCheckReason::InvalidThreshold);

    const auto forward_geometry_result = bt::ComputeEpipolarGeometry(camera_a, MakeForwardCameraB());
    BT_CHECK(forward_geometry_result.ok());
    const auto degenerate = bt::ComputeNormalizedSampsonEpipolarCheck(
        forward_geometry_result.value(),
        bt::Vec2f{0.0f, 0.0f},
        bt::Vec2f{0.0f, 0.0f},
        0.0025f,
        0.018f);
    BT_CHECK(!degenerate.valid);
    BT_CHECK(degenerate.reason == bt::EpipolarCheckReason::DegenerateDenominator);
    BT_CHECK(!degenerate.hard_mismatch);

    const bt::Vec2f forward_epipole_px{320.0f, 240.0f};
    const auto pixel_degenerate = bt::ComputePixelSampsonEpipolarCheck(
        forward_geometry_result.value(),
        forward_epipole_px,
        forward_epipole_px,
        2.5f,
        18.0f);
    BT_CHECK(!pixel_degenerate.valid);
    BT_CHECK(pixel_degenerate.reason == bt::EpipolarCheckReason::DegenerateDenominator);
    BT_CHECK(!pixel_degenerate.hard_mismatch);

    const auto distortion_safe_degenerate = bt::ComputeDistortionSafePixelSampsonEpipolarCheck(
        forward_geometry_result.value(),
        camera_a,
        MakeForwardCameraB(),
        forward_epipole_px,
        forward_epipole_px,
        2.5f,
        18.0f);
    BT_CHECK(!distortion_safe_degenerate.valid);
    BT_CHECK(distortion_safe_degenerate.reason == bt::EpipolarCheckReason::DegenerateDenominator);
    BT_CHECK(!distortion_safe_degenerate.hard_mismatch);

    return 0;
}
