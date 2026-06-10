#pragma once

#include "core/math.h"

#include <array>
#include <cmath>
#include <string>
#include <vector>

namespace bt {

struct CameraCalibration {
    bool intrinsics_valid = false;
    bool extrinsics_valid = false;
    std::array<double, 9> camera_matrix{};
    std::array<double, 5> distortion{};
    Mat34f world_from_camera{};
    Mat34f image_from_world{};
};

struct FloorPlane {
    Vec3f normal{0.0f, 1.0f, 0.0f};
    float distance = 0.0f;
    bool valid = false;
};

struct FloorGeometryLineFamily {
    bool valid = false;
    float confidence = 0.0f;
    float orientation_rad = 0.0f;
    float spacing_px = 0.0f;
    float spacing_m = 0.0f;
    bool metric_spacing_valid = false;
    float reference_rho_px = 0.0f;
    Vec2f vanishing_point_px{};
    bool vanishing_point_valid = false;
    int accepted_line_count = 0;
    int rejected_line_count = 0;
    std::string reason;
};

struct LensDistortionEstimate {
    bool available = false;
    bool valid = false;
    bool applied_to_runtime = false;
    float confidence = 0.0f;
    float radial_k1 = 0.0f;
    float radial_k2 = 0.0f;
    float tangential_p1 = 0.0f;
    float tangential_p2 = 0.0f;
    float straightness_error_px = 0.0f;
    float corrected_straightness_error_px = 0.0f;
    int sampled_seam_count = 0;
    int sampled_point_count = 0;
    std::string model = "none"; // none, radial, radial_tangential.
    std::string reason = "unavailable";
};

struct FloorGeometryCalibration {
    bool valid = false;
    int image_width = 0;
    int image_height = 0;
    // Preserves where the calibration came from instead of collapsing every valid
    // payload into "backend_generated" in UI/replay/debug JSON. Expected values
    // include backend_floor_geometry_calibrator, cli_lines, cli_frame, imported_json,
    // manual_json, legacy_json, and multi_camera_alignment.
    std::string source = "unknown";
    std::string floor_type = "unknown"; // unknown, planks, tiles.
    int family_count = 0;
    FloorGeometryLineFamily family_a{};
    FloorGeometryLineFamily family_b{};
    bool two_axis_grid_valid = false;
    bool homography_valid = false;
    std::array<float, 9> floor_from_image{};
    std::array<float, 9> image_from_floor{};
    float homography_reprojection_error_px = 0.0f;
    int homography_inlier_count = 0;
    int homography_intersection_count = 0;
    std::string homography_reason = "unavailable";
    FloorPlane floor_plane{};
    float floor_plane_confidence = 0.0f;
    bool camera_orientation_valid = false;
    float camera_pitch_rad = 0.0f;
    float camera_roll_rad = 0.0f;
    float camera_yaw_rad = 0.0f;
    float camera_orientation_confidence = 0.0f;
    bool camera_orientation_applied_to_runtime = false;
    bool camera_height_valid = false;
    float camera_height_m = 0.0f;
    float metric_scale_confidence = 0.0f;
    LensDistortionEstimate distortion{};
    float multi_camera_alignment_confidence = 0.0f;
    bool multi_camera_alignment_valid = false;
    std::string multi_camera_warning;
    float multi_camera_yaw_delta_rad = 0.0f;
    float multi_camera_pitch_delta_rad = 0.0f;
    float multi_camera_roll_delta_rad = 0.0f;
    float multi_camera_height_delta_m = 0.0f;
    float multi_camera_scale_ratio = 1.0f;
    bool shared_floor_frame_valid = false;
    std::array<float, 9> shared_floor_transform{};
    float planted_drift_axis_confidence = 0.0f;
    std::string reason;
};

struct WallRectangleCalibration {
    bool valid = false;
    int image_width = 0;
    int image_height = 0;
    std::string source = "unknown";
    float confidence = 0.0f;
    std::string reason;

    std::array<Vec2f, 4> image_corners{};
    float rectangle_width_m = 0.0f;
    float rectangle_height_m = 0.0f;
    float rectangle_aspect_ratio = 0.0f;

    bool wall_homography_valid = false;
    std::array<float, 9> wall_from_image{};
    std::array<float, 9> image_from_wall{};
    float homography_reprojection_error_px = 0.0f;

    bool metric_scale_valid = false;
    float metric_scale_confidence = 0.0f;

    bool wall_orientation_valid = false;
    Vec3f wall_right_camera{};
    Vec3f wall_down_camera{};
    Vec3f wall_normal_camera{};
    float wall_orientation_confidence = 0.0f;

    bool wall_depth_valid = false;
    float wall_center_depth_m = 0.0f;
    float wall_depth_confidence = 0.0f;

    bool usable_for_wall_homography = false;
    bool usable_for_metric_scale = false;
    bool usable_for_orientation = false;
    bool usable_for_depth_assist = false;
    bool usable_for_floor_plane = false;
    bool usable_for_floor_homography = false;
    bool applied_to_runtime = false;
    std::string capability_reason;
};

inline bool FloorPlaneUsable(const FloorPlane& floor) {
    return floor.valid &&
        IsFinite(floor.normal) &&
        std::isfinite(floor.distance) &&
        Dot(floor.normal, floor.normal) > 1e-8f;
}

inline float SignedDistanceToFloorPlane(const Vec3f& point, const FloorPlane& floor) {
    const float normal_len = Length(floor.normal);
    if (!FloorPlaneUsable(floor) || normal_len <= 1e-6f) {
        return 0.0f;
    }
    return (Dot(floor.normal, point) - floor.distance) / normal_len;
}

inline Vec3f ProjectPointToFloorPlane(const Vec3f& point, const FloorPlane& floor) {
    const float normal_len2 = Dot(floor.normal, floor.normal);
    if (!FloorPlaneUsable(floor) || normal_len2 <= 1e-8f) {
        return point;
    }
    const float signed_numerator = Dot(floor.normal, point) - floor.distance;
    return Sub(point, Scale(floor.normal, signed_numerator / normal_len2));
}

inline float FloorYAtXZOr(const FloorPlane& floor, float x, float z, float fallback_y) {
    if (!FloorPlaneUsable(floor) || std::abs(floor.normal.y) <= 1e-6f) {
        return fallback_y;
    }
    return (floor.distance - floor.normal.x * x - floor.normal.z * z) / floor.normal.y;
}

struct BodyCalibrationQuality {
    float pelvis_width = 0.0f;
    float left_femur = 0.0f;
    float right_femur = 0.0f;
    float left_tibia = 0.0f;
    float right_tibia = 0.0f;
    float left_foot_length = 0.0f;
    float right_foot_length = 0.0f;
    float standing_hmd_to_pelvis = 0.0f;
    float overall = 0.0f;
    int sample_count = 0;
    std::string source = "defaults";
};

struct BodyCalibration {
    bool standing_neutral_valid = false;
    bool seated_neutral_valid = false;
    bool reclined_neutral_valid = false;
    float pelvis_width = 0.32f;
    float left_femur = 0.42f;
    float right_femur = 0.42f;
    float left_tibia = 0.42f;
    float right_tibia = 0.42f;
    float left_foot_length = 0.24f;
    float right_foot_length = 0.24f;
    Vec3f standing_hmd_to_pelvis{0.0f, -0.75f, 0.0f};
    Vec3f seated_hmd_to_pelvis{0.0f, -0.55f, -0.15f};
    Vec3f reclined_hmd_to_pelvis{0.0f, -0.25f, -0.45f};
    BodyCalibrationQuality quality{};
};

struct CalibrationBundle {
    int schema_version = 1;
    std::string world_handedness = "right_handed";
    std::string world_up_axis = "Y";
    CameraCalibration camera_a{};
    CameraCalibration camera_b{};
    FloorPlane floor{};
    FloorGeometryCalibration floor_geometry{};
    std::vector<WallRectangleCalibration> wall_rectangles{};
    FloorGeometryCalibration camera_a_floor_geometry{};
    FloorGeometryCalibration camera_b_floor_geometry{};
    std::vector<WallRectangleCalibration> camera_a_wall_rectangles{};
    std::vector<WallRectangleCalibration> camera_b_wall_rectangles{};
    BodyCalibration body{};
};

struct CalibrationReadiness {
    bool camera_a_ready = false;
    bool camera_b_ready = false;
    bool cameras_ready = false;
    bool floor_ready = false;
    bool lower_body_ready = false;
    bool tracking_ready = false;
    std::string summary;
};

} // namespace bt
