#include "calibration/calibration_io.h"

#include <algorithm>
#include <fstream>
#include <exception>
#include <cmath>
#include <nlohmann/json.hpp>
#include <sstream>
#include <string>
#include <system_error>
#include <type_traits>

namespace bt {
namespace {

Status WriteTextFileAtomically(const std::filesystem::path& path, const std::string& contents, const char* label) {
    const auto parent = path.parent_path();
    if (!parent.empty()) {
        std::error_code mkdir_ec;
        std::filesystem::create_directories(parent, mkdir_ec);
        if (mkdir_ec) {
            return Status::Error(StatusCode::InvalidArgument, std::string("Could not create parent directory for ") + label + ": " + mkdir_ec.message());
        }
    }

    std::filesystem::path temp_path = path;
    temp_path += ".tmp";
    std::filesystem::path backup_path = path;
    backup_path += ".bak";

    auto remove_quietly = [](const std::filesystem::path& p) {
        std::error_code ec;
        std::filesystem::remove(p, ec);
    };

    {
        std::ofstream out(temp_path, std::ios::trunc);
        if (!out) {
            return Status::Error(StatusCode::InvalidArgument, std::string("Could not write ") + label + " temp file: " + temp_path.string());
        }
        out << contents;
        if (!out) {
            remove_quietly(temp_path);
            return Status::Error(StatusCode::InvalidArgument, std::string("Could not finish writing ") + label + " temp file: " + temp_path.string());
        }
    }

    std::error_code ec;
    const bool stale_backup_exists = std::filesystem::exists(backup_path, ec);
    if (ec) {
        remove_quietly(temp_path);
        return Status::Error(StatusCode::InvalidArgument, std::string("Could not inspect stale backup for ") + label + ": " + ec.message());
    }
    if (stale_backup_exists) {
        std::filesystem::remove(backup_path, ec);
        if (ec) {
            remove_quietly(temp_path);
            return Status::Error(StatusCode::InvalidArgument, std::string("Could not clear stale backup for ") + label + ": " + ec.message());
        }
    }

    ec.clear();
    const bool target_exists = std::filesystem::exists(path, ec);
    if (ec) {
        remove_quietly(temp_path);
        return Status::Error(StatusCode::InvalidArgument, std::string("Could not inspect existing ") + label + " before save: " + ec.message());
    }

    bool backup_created = false;
    if (target_exists) {
        std::filesystem::rename(path, backup_path, ec);
        if (ec) {
            remove_quietly(temp_path);
            return Status::Error(StatusCode::InvalidArgument, std::string("Could not move existing ") + label + " to backup before save: " + ec.message());
        }
        backup_created = true;
    }

    auto rollback_after_replace_failure = [&](const std::error_code& replace_ec) {
        std::error_code rollback_ec;
        std::filesystem::remove(path, rollback_ec);
        if (rollback_ec) {
            return Status::Error(StatusCode::InvalidArgument,
                std::string("Could not replace ") + label + ": " + replace_ec.message() +
                "; rollback cleanup failed: " + rollback_ec.message());
        }
        if (backup_created) {
            rollback_ec.clear();
            std::filesystem::rename(backup_path, path, rollback_ec);
            if (rollback_ec) {
                return Status::Error(StatusCode::InvalidArgument,
                    std::string("Could not replace ") + label + ": " + replace_ec.message() +
                    "; rollback also failed: " + rollback_ec.message());
            }
        }
        return Status::Error(StatusCode::InvalidArgument, std::string("Could not replace ") + label + ": " + replace_ec.message());
    };

    ec.clear();
    std::filesystem::rename(temp_path, path, ec);
    if (ec) {
        const Status rollback_status = rollback_after_replace_failure(ec);
        remove_quietly(temp_path);
        return rollback_status;
    }

    if (backup_created) {
        remove_quietly(backup_path);
    }
    return Status::OK();
}

template <typename T, std::size_t N>
Result<std::array<T, N>> ReadArray(const nlohmann::json& j, const char* key) {
    if (!j.contains(key) || !j.at(key).is_array() || j.at(key).size() != N) {
        std::ostringstream oss;
        oss << "Calibration key \"" << key << "\" must be an array of " << N << " numbers";
        return Status::Error(StatusCode::ValidationError, oss.str());
    }

    std::array<T, N> out{};
    for (std::size_t i = 0; i < N; ++i) {
        if (!j.at(key).at(i).is_number()) {
            std::ostringstream oss;
            oss << "Calibration key \"" << key << "\" contains a non-number at index " << i;
            return Status::Error(StatusCode::ValidationError, oss.str());
        }
        out[i] = j.at(key).at(i).get<T>();
        if constexpr (std::is_floating_point_v<T>) {
            if (!std::isfinite(out[i])) {
                std::ostringstream oss;
                oss << "Calibration key \"" << key << "\" contains a non-finite value at index " << i;
                return Status::Error(StatusCode::ValidationError, oss.str());
            }
        }
    }
    return out;
}

bool HasNonZeroFinite(const std::array<float, 12>& values) {
    bool any_nonzero = false;
    for (const auto v : values) {
        if (!std::isfinite(v)) {
            return false;
        }
        any_nonzero = any_nonzero || std::abs(v) > 1e-8f;
    }
    return any_nonzero;
}

bool HasNonZeroFinite(const std::array<double, 9>& values) {
    bool any_nonzero = false;
    for (const auto v : values) {
        if (!std::isfinite(v)) {
            return false;
        }
        any_nonzero = any_nonzero || std::abs(v) > 1e-10;
    }
    return any_nonzero;
}

bool HasFiniteDistortion(const std::array<double, 5>& values) {
    for (const auto v : values) {
        if (!std::isfinite(v)) {
            return false;
        }
    }
    return true;
}


bool Matrix3x3Usable(const std::array<float, 9>& values) {
    bool any_nonzero = false;
    for (const auto v : values) {
        if (!std::isfinite(v)) {
            return false;
        }
        any_nonzero = any_nonzero || std::abs(v) > 1e-8f;
    }
    return any_nonzero;
}

std::string LegacyFloorGeometrySource(const nlohmann::json& j, const FloorGeometryCalibration& g) {
    if (j.contains("source") && j.at("source").is_string()) {
        return j.at("source").get<std::string>();
    }
    if (!g.valid) {
        return "nothing";
    }
    if (g.family_count > 0 || g.homography_valid || g.distortion.valid || g.camera_orientation_valid) {
        return "legacy_json";
    }
    return "manual_json";
}

Result<CameraCalibration> ReadCameraCalibration(const nlohmann::json& j, const char* label) {
    CameraCalibration cam;
    cam.intrinsics_valid = j.value("intrinsics_valid", false);
    cam.extrinsics_valid = j.value("extrinsics_valid", false);

    if (cam.intrinsics_valid) {
        const auto camera_matrix = ReadArray<double, 9>(j, "camera_matrix");
        if (!camera_matrix.ok()) {
            return Status::Error(camera_matrix.status().code, std::string(label) + ": " + camera_matrix.status().message);
        }
        cam.camera_matrix = camera_matrix.value();

        if (j.contains("distortion")) {
            const auto distortion = ReadArray<double, 5>(j, "distortion");
            if (!distortion.ok()) {
                return Status::Error(distortion.status().code, std::string(label) + ": " + distortion.status().message);
            }
            cam.distortion = distortion.value();
        }
    }

    if (cam.extrinsics_valid) {
        const auto world_from_camera = ReadArray<float, 12>(j, "world_from_camera");
        if (!world_from_camera.ok()) {
            return Status::Error(world_from_camera.status().code, std::string(label) + ": " + world_from_camera.status().message);
        }
        cam.world_from_camera.m = world_from_camera.value();

        const auto image_from_world = ReadArray<float, 12>(j, "image_from_world");
        if (!image_from_world.ok()) {
            return Status::Error(image_from_world.status().code, std::string(label) + ": " + image_from_world.status().message);
        }
        cam.image_from_world.m = image_from_world.value();
    }

    return cam;
}

bool CameraReady(const CameraCalibration& c) {
    return c.intrinsics_valid &&
        c.extrinsics_valid &&
        HasNonZeroFinite(c.camera_matrix) &&
        HasFiniteDistortion(c.distortion) &&
        HasNonZeroFinite(c.world_from_camera.m) &&
        HasNonZeroFinite(c.image_from_world.m);
}

bool FloorReady(const FloorPlane& f) {
    return f.valid &&
        std::isfinite(f.normal.x) &&
        std::isfinite(f.normal.y) &&
        std::isfinite(f.normal.z) &&
        std::isfinite(f.distance) &&
        (std::abs(f.normal.x) + std::abs(f.normal.y) + std::abs(f.normal.z) > 1e-5f);
}

Vec3f ReadVec3Or(const nlohmann::json& j, const char* key, Vec3f fallback) {
    if (!j.contains(key) || !j.at(key).is_array() || j.at(key).size() != 3) {
        return fallback;
    }
    const auto& a = j.at(key);
    if (!a[0].is_number() || !a[1].is_number() || !a[2].is_number()) {
        return fallback;
    }
    Vec3f out{a[0].get<float>(), a[1].get<float>(), a[2].get<float>()};
    return (std::isfinite(out.x) && std::isfinite(out.y) && std::isfinite(out.z)) ? out : fallback;
}

float ReadQualityOr(const nlohmann::json& j, const char* key, float fallback = 0.0f) {
    const float value = j.value(key, fallback);
    if (!std::isfinite(value)) {
        return fallback;
    }
    return std::max(0.0f, std::min(1.0f, value));
}


Vec2f ReadVec2Or(const nlohmann::json& j, const char* key, Vec2f fallback = {}) {
    if (!j.contains(key) || !j.at(key).is_array() || j.at(key).size() != 2) {
        return fallback;
    }
    const auto& a = j.at(key);
    if (!a[0].is_number() || !a[1].is_number()) {
        return fallback;
    }
    Vec2f out{a[0].get<float>(), a[1].get<float>()};
    return (std::isfinite(out.x) && std::isfinite(out.y)) ? out : fallback;
}

FloorGeometryLineFamily ReadFloorLineFamily(const nlohmann::json& j) {
    FloorGeometryLineFamily f;
    f.valid = j.value("valid", false);
    f.confidence = ReadQualityOr(j, "confidence");
    f.orientation_rad = j.value("orientation_rad", 0.0f);
    f.spacing_px = j.value("spacing_px", 0.0f);
    f.spacing_m = j.value("spacing_m", 0.0f);
    f.metric_spacing_valid = j.value("metric_spacing_valid", false);
    f.reference_rho_px = j.value("reference_rho_px", 0.0f);
    f.vanishing_point_px = ReadVec2Or(j, "vanishing_point_px");
    f.vanishing_point_valid = j.value("vanishing_point_valid", false);
    f.accepted_line_count = j.value("accepted_line_count", 0);
    f.rejected_line_count = j.value("rejected_line_count", 0);
    f.reason = j.value("reason", std::string{});
    return f;
}

LensDistortionEstimate ReadDistortionEstimate(const nlohmann::json& j) {
    LensDistortionEstimate d;
    d.available = j.value("available", false);
    d.valid = j.value("valid", false);
    d.applied_to_runtime = j.value("applied_to_runtime", false);
    d.confidence = ReadQualityOr(j, "confidence");
    d.radial_k1 = j.value("radial_k1", 0.0f);
    d.radial_k2 = j.value("radial_k2", 0.0f);
    d.tangential_p1 = j.value("tangential_p1", 0.0f);
    d.tangential_p2 = j.value("tangential_p2", 0.0f);
    d.straightness_error_px = j.value("straightness_error_px", 0.0f);
    d.corrected_straightness_error_px = j.value("corrected_straightness_error_px", 0.0f);
    d.sampled_seam_count = j.value("sampled_seam_count", 0);
    d.sampled_point_count = j.value("sampled_point_count", 0);
    d.model = j.value("model", std::string("none"));
    d.reason = j.value("reason", std::string("unavailable"));
    return d;
}

Result<FloorGeometryCalibration> ReadFloorGeometry(const nlohmann::json& j) {
    FloorGeometryCalibration g;
    g.valid = j.value("valid", false);
    g.image_width = j.value("image_width", 0);
    g.image_height = j.value("image_height", 0);
    g.floor_type = j.value("floor_type", std::string("unknown"));
    g.family_count = j.value("family_count", 0);
    if (j.contains("family_a") && j["family_a"].is_object()) g.family_a = ReadFloorLineFamily(j["family_a"]);
    if (j.contains("family_b") && j["family_b"].is_object()) g.family_b = ReadFloorLineFamily(j["family_b"]);
    g.two_axis_grid_valid = j.value("two_axis_grid_valid", false);
    g.homography_valid = j.value("homography_valid", false);

    const bool needs_homography = g.homography_valid || j.contains("floor_from_image") || j.contains("image_from_floor");
    if (needs_homography) {
        const auto floor_from_image = ReadArray<float, 9>(j, "floor_from_image");
        const auto image_from_floor = ReadArray<float, 9>(j, "image_from_floor");
        if (floor_from_image.ok() && image_from_floor.ok()) {
            g.floor_from_image = floor_from_image.value();
            g.image_from_floor = image_from_floor.value();
            if (g.homography_valid && (!Matrix3x3Usable(g.floor_from_image) || !Matrix3x3Usable(g.image_from_floor))) {
                g.homography_valid = false;
                g.homography_reason = "homography_invalid_matrix_ignored";
            }
        } else {
            g.homography_valid = false;
            g.homography_reason = "homography_malformed_ignored";
        }
    }

    g.homography_reprojection_error_px = j.value("homography_reprojection_error_px", 0.0f);
    g.homography_inlier_count = j.value("homography_inlier_count", 0);
    g.homography_intersection_count = j.value("homography_intersection_count", 0);
    g.homography_reason = j.value("homography_reason", std::string("unavailable"));
    const auto floor = j.value("floor_plane", nlohmann::json::object());
    g.floor_plane.valid = floor.value("valid", false);
    g.floor_plane.distance = floor.value("distance", 0.0f);
    g.floor_plane.normal = ReadVec3Or(floor, "normal", g.floor_plane.normal);
    g.floor_plane_confidence = ReadQualityOr(j, "floor_plane_confidence");
    g.camera_orientation_valid = j.value("camera_orientation_valid", false);
    g.camera_pitch_rad = j.value("camera_pitch_rad", 0.0f);
    g.camera_roll_rad = j.value("camera_roll_rad", 0.0f);
    g.camera_yaw_rad = j.value("camera_yaw_rad", 0.0f);
    g.camera_orientation_confidence = ReadQualityOr(j, "camera_orientation_confidence");
    g.camera_orientation_applied_to_runtime = j.value("camera_orientation_applied_to_runtime", false);
    g.camera_height_valid = j.value("camera_height_valid", false);
    g.camera_height_m = j.value("camera_height_m", 0.0f);
    g.metric_scale_confidence = ReadQualityOr(j, "metric_scale_confidence");
    if (j.contains("distortion") && j["distortion"].is_object()) g.distortion = ReadDistortionEstimate(j["distortion"]);
    g.multi_camera_alignment_confidence = ReadQualityOr(j, "multi_camera_alignment_confidence");
    g.multi_camera_alignment_valid = j.value("multi_camera_alignment_valid", false);
    g.multi_camera_warning = j.value("multi_camera_warning", std::string{});
    g.multi_camera_yaw_delta_rad = j.value("multi_camera_yaw_delta_rad", 0.0f);
    g.multi_camera_pitch_delta_rad = j.value("multi_camera_pitch_delta_rad", 0.0f);
    g.multi_camera_roll_delta_rad = j.value("multi_camera_roll_delta_rad", 0.0f);
    g.multi_camera_height_delta_m = j.value("multi_camera_height_delta_m", 0.0f);
    g.multi_camera_scale_ratio = j.value("multi_camera_scale_ratio", 1.0f);
    g.shared_floor_frame_valid = j.value("shared_floor_frame_valid", false);
    if (g.shared_floor_frame_valid || j.contains("shared_floor_transform")) {
        const auto shared = ReadArray<float, 9>(j, "shared_floor_transform");
        if (shared.ok()) {
            g.shared_floor_transform = shared.value();
            if (g.shared_floor_frame_valid && !Matrix3x3Usable(g.shared_floor_transform)) {
                g.shared_floor_frame_valid = false;
                g.multi_camera_warning = "shared_floor_transform_invalid_ignored";
            }
        } else {
            g.shared_floor_frame_valid = false;
            g.multi_camera_warning = "shared_floor_transform_malformed_ignored";
        }
    }
    g.planted_drift_axis_confidence = ReadQualityOr(j, "planted_drift_axis_confidence");
    g.reason = j.value("reason", std::string{});
    g.source = LegacyFloorGeometrySource(j, g);
    return g;
}

WallRectangleCalibration ReadWallRectangle(const nlohmann::json& j) {
    WallRectangleCalibration w;
    w.valid = j.value("valid", false);
    w.image_width = j.value("image_width", 0);
    w.image_height = j.value("image_height", 0);
    w.source = j.value("source", std::string("manual_wall_rectangle"));
    w.confidence = ReadQualityOr(j, "confidence");
    w.reason = j.value("reason", std::string{});
    w.rectangle_width_m = j.value("rectangle_width_m", 0.0f);
    w.rectangle_height_m = j.value("rectangle_height_m", 0.0f);
    w.rectangle_aspect_ratio = j.value("rectangle_aspect_ratio", 0.0f);

    if (j.contains("image_corners") && j["image_corners"].is_array()) {
        const auto& corners = j["image_corners"];
        for (std::size_t i = 0; i < w.image_corners.size() && i < corners.size(); ++i) {
            if (corners[i].is_array() && corners[i].size() == 2 &&
                corners[i][0].is_number() && corners[i][1].is_number()) {
                const Vec2f p{corners[i][0].get<float>(), corners[i][1].get<float>()};
                if (std::isfinite(p.x) && std::isfinite(p.y)) {
                    w.image_corners[i] = p;
                }
            } else if (corners[i].is_object() &&
                corners[i].contains("x") && corners[i].contains("y") &&
                corners[i]["x"].is_number() && corners[i]["y"].is_number()) {
                const Vec2f p{corners[i]["x"].get<float>(), corners[i]["y"].get<float>()};
                if (std::isfinite(p.x) && std::isfinite(p.y)) {
                    w.image_corners[i] = p;
                }
            }
        }
    }

    w.wall_homography_valid = j.value("wall_homography_valid", false);
    const bool needs_homography = w.wall_homography_valid || j.contains("wall_from_image") || j.contains("image_from_wall");
    if (needs_homography) {
        const auto wall_from_image = ReadArray<float, 9>(j, "wall_from_image");
        const auto image_from_wall = ReadArray<float, 9>(j, "image_from_wall");
        if (wall_from_image.ok() && image_from_wall.ok()) {
            w.wall_from_image = wall_from_image.value();
            w.image_from_wall = image_from_wall.value();
            if (w.wall_homography_valid && (!Matrix3x3Usable(w.wall_from_image) || !Matrix3x3Usable(w.image_from_wall))) {
                w.wall_homography_valid = false;
                w.reason = "wall_homography_invalid_matrix_ignored";
            }
        } else {
            w.wall_homography_valid = false;
            w.reason = "wall_homography_malformed_ignored";
        }
    }
    w.homography_reprojection_error_px = j.value("homography_reprojection_error_px", 0.0f);
    w.metric_scale_valid = j.value("metric_scale_valid", false);
    w.metric_scale_confidence = ReadQualityOr(j, "metric_scale_confidence");
    w.wall_orientation_valid = j.value("wall_orientation_valid", false);
    w.wall_right_camera = ReadVec3Or(j, "wall_right_camera", w.wall_right_camera);
    w.wall_down_camera = ReadVec3Or(j, "wall_down_camera", w.wall_down_camera);
    w.wall_normal_camera = ReadVec3Or(j, "wall_normal_camera", w.wall_normal_camera);
    w.wall_orientation_confidence = ReadQualityOr(j, "wall_orientation_confidence");
    w.wall_depth_valid = j.value("wall_depth_valid", false);
    w.wall_center_depth_m = j.value("wall_center_depth_m", 0.0f);
    w.wall_depth_confidence = ReadQualityOr(j, "wall_depth_confidence");
    w.usable_for_wall_homography = j.value("usable_for_wall_homography", false);
    w.usable_for_metric_scale = j.value("usable_for_metric_scale", false);
    w.usable_for_orientation = j.value("usable_for_orientation", false);
    w.usable_for_depth_assist = j.value("usable_for_depth_assist", false);
    w.usable_for_floor_plane = j.value("usable_for_floor_plane", false);
    w.usable_for_floor_homography = j.value("usable_for_floor_homography", false);
    w.applied_to_runtime = j.value("applied_to_runtime", false);
    w.capability_reason = j.value("capability_reason", std::string{});
    return w;
}

std::vector<WallRectangleCalibration> ReadWallRectangleArray(const nlohmann::json& walls_json) {
    std::vector<WallRectangleCalibration> walls;
    if (!walls_json.is_array()) {
        return walls;
    }
    for (const auto& item : walls_json) {
        if (!item.is_object()) {
            continue;
        }
        auto wall = ReadWallRectangle(item);
        if (wall.valid) {
            wall.applied_to_runtime = false;
            walls.push_back(wall);
        }
    }
    return walls;
}

bool BodyReady(const BodyCalibration& b) {
    const auto positive = [](float v) { return std::isfinite(v) && v > 0.0f; };
    return b.standing_neutral_valid &&
        positive(b.pelvis_width) &&
        positive(b.left_femur) &&
        positive(b.right_femur) &&
        positive(b.left_tibia) &&
        positive(b.right_tibia) &&
        positive(b.left_foot_length) &&
        positive(b.right_foot_length);
}

} // namespace

Result<CalibrationBundle> LoadCalibration(const std::filesystem::path& path) {
    std::ifstream in(path);
    if (!in) {
        return Status::Error(StatusCode::InvalidArgument, "Calibration file not readable: " + path.string());
    }

    nlohmann::json j;
    try {
        in >> j;
    } catch (const std::exception& e) {
        return Status::Error(StatusCode::ValidationError, std::string("Calibration JSON parse failed: ") + e.what());
    }

    try {
        CalibrationBundle bundle;
        bundle.schema_version = j.value("schema_version", bundle.schema_version);
        bundle.world_handedness = j.value("world_handedness", bundle.world_handedness);
        bundle.world_up_axis = j.value("world_up_axis", bundle.world_up_axis);

        const auto floor = j.value("floor_plane", nlohmann::json::object());
        bundle.floor.valid = floor.value("valid", false);
        bundle.floor.distance = floor.value("distance", 0.0f);
        if (floor.contains("normal") && floor["normal"].is_array() && floor["normal"].size() == 3) {
            bundle.floor.normal = Vec3f{
                floor["normal"][0].get<float>(),
                floor["normal"][1].get<float>(),
                floor["normal"][2].get<float>()
            };
        }
        if (j.contains("floor_geometry") && j["floor_geometry"].is_object()) {
            const auto floor_geometry = ReadFloorGeometry(j["floor_geometry"]);
            if (floor_geometry.ok()) {
                bundle.floor_geometry = floor_geometry.value();
                bundle.camera_a_floor_geometry = bundle.floor_geometry;
                if (bundle.floor_geometry.floor_plane.valid && !bundle.floor.valid) {
                    bundle.floor = bundle.floor_geometry.floor_plane;
                }
            }
        }
        if (j.contains("wall_rectangles") && j["wall_rectangles"].is_array()) {
            bundle.wall_rectangles = ReadWallRectangleArray(j["wall_rectangles"]);
            bundle.camera_a_wall_rectangles = bundle.wall_rectangles;
        }
        if (j.contains("floor_geometry_by_camera") && j["floor_geometry_by_camera"].is_object()) {
            const auto& by_camera = j["floor_geometry_by_camera"];
            if (by_camera.contains("camera_a") && by_camera["camera_a"].is_object()) {
                const auto floor_geometry = ReadFloorGeometry(by_camera["camera_a"]);
                if (floor_geometry.ok()) {
                    bundle.camera_a_floor_geometry = floor_geometry.value();
                    bundle.floor_geometry = bundle.camera_a_floor_geometry;
                    if (bundle.floor_geometry.floor_plane.valid && !bundle.floor.valid) {
                        bundle.floor = bundle.floor_geometry.floor_plane;
                    }
                }
            }
            if (by_camera.contains("camera_b") && by_camera["camera_b"].is_object()) {
                const auto floor_geometry = ReadFloorGeometry(by_camera["camera_b"]);
                if (floor_geometry.ok()) {
                    bundle.camera_b_floor_geometry = floor_geometry.value();
                }
            }
        }
        if (j.contains("wall_rectangles_by_camera") && j["wall_rectangles_by_camera"].is_object()) {
            const auto& by_camera = j["wall_rectangles_by_camera"];
            if (by_camera.contains("camera_a")) {
                bundle.camera_a_wall_rectangles = ReadWallRectangleArray(by_camera["camera_a"]);
                bundle.wall_rectangles = bundle.camera_a_wall_rectangles;
            }
            if (by_camera.contains("camera_b")) {
                bundle.camera_b_wall_rectangles = ReadWallRectangleArray(by_camera["camera_b"]);
            }
        }

        const auto ca = j.value("camera_a", nlohmann::json::object());
        const auto camera_a = ReadCameraCalibration(ca, "camera_a");
        if (!camera_a.ok()) {
            return camera_a.status();
        }
        bundle.camera_a = camera_a.value();

        const auto cb = j.value("camera_b", nlohmann::json::object());
        const auto camera_b = ReadCameraCalibration(cb, "camera_b");
        if (!camera_b.ok()) {
            return camera_b.status();
        }
        bundle.camera_b = camera_b.value();

        const auto body = j.value("body", nlohmann::json::object());
        bundle.body.standing_neutral_valid = body.value("standing_neutral_valid", false);
        bundle.body.seated_neutral_valid = body.value("seated_neutral_valid", false);
        bundle.body.reclined_neutral_valid = body.value("reclined_neutral_valid", false);
        bundle.body.pelvis_width = body.value("pelvis_width", bundle.body.pelvis_width);
        bundle.body.left_femur = body.value("left_femur", bundle.body.left_femur);
        bundle.body.right_femur = body.value("right_femur", bundle.body.right_femur);
        bundle.body.left_tibia = body.value("left_tibia", bundle.body.left_tibia);
        bundle.body.right_tibia = body.value("right_tibia", bundle.body.right_tibia);
        bundle.body.left_foot_length = body.value("left_foot_length", bundle.body.left_foot_length);
        bundle.body.right_foot_length = body.value("right_foot_length", bundle.body.right_foot_length);
        bundle.body.standing_hmd_to_pelvis = ReadVec3Or(body, "standing_hmd_to_pelvis", bundle.body.standing_hmd_to_pelvis);
        bundle.body.seated_hmd_to_pelvis = ReadVec3Or(body, "seated_hmd_to_pelvis", bundle.body.seated_hmd_to_pelvis);
        bundle.body.reclined_hmd_to_pelvis = ReadVec3Or(body, "reclined_hmd_to_pelvis", bundle.body.reclined_hmd_to_pelvis);
        const auto quality = body.value("quality", nlohmann::json::object());
        bundle.body.quality.pelvis_width = ReadQualityOr(quality, "pelvis_width");
        bundle.body.quality.left_femur = ReadQualityOr(quality, "left_femur");
        bundle.body.quality.right_femur = ReadQualityOr(quality, "right_femur");
        bundle.body.quality.left_tibia = ReadQualityOr(quality, "left_tibia");
        bundle.body.quality.right_tibia = ReadQualityOr(quality, "right_tibia");
        bundle.body.quality.left_foot_length = ReadQualityOr(quality, "left_foot_length");
        bundle.body.quality.right_foot_length = ReadQualityOr(quality, "right_foot_length");
        bundle.body.quality.standing_hmd_to_pelvis = ReadQualityOr(quality, "standing_hmd_to_pelvis");
        bundle.body.quality.overall = ReadQualityOr(quality, "overall");
        bundle.body.quality.sample_count = quality.value("sample_count", 0);
        bundle.body.quality.source = quality.value("source", std::string("defaults"));

        return bundle;
    } catch (const std::exception& e) {
        return Status::Error(StatusCode::ValidationError, std::string("Calibration JSON validation failed: ") + e.what());
    }

}

Status SaveCalibrationTemplate(const std::filesystem::path& path) {
    static constexpr const char* kTemplate = R"JSON({
  "schema_version": 1,
  "world_handedness": "right_handed",
  "world_up_axis": "Y",
  "tracking_ready": false,
  "floor_plane": { "normal": [0.0, 1.0, 0.0], "distance": 0.0, "valid": false },
  "floor_geometry": {
    "valid": false,
    "source": "nothing",
    "floor_type": "unknown",
    "family_count": 0,
    "two_axis_grid_valid": false,
    "homography_valid": false,
    "metric_scale_confidence": 0.0,
    "camera_orientation_valid": false,
    "camera_orientation_confidence": 0.0,
    "distortion": { "available": false, "valid": false, "confidence": 0.0, "reason": "unavailable" },
    "reason": "not_calibrated"
  },
  "wall_rectangles": [],
  "camera_a": {
    "intrinsics_valid": false,
    "extrinsics_valid": false,
    "camera_matrix": [0, 0, 0, 0, 0, 0, 0, 0, 0],
    "distortion": [0, 0, 0, 0, 0],
    "world_from_camera": [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    "image_from_world": [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
  },
  "camera_b": {
    "intrinsics_valid": false,
    "extrinsics_valid": false,
    "camera_matrix": [0, 0, 0, 0, 0, 0, 0, 0, 0],
    "distortion": [0, 0, 0, 0, 0],
    "world_from_camera": [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    "image_from_world": [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
  },
  "body": {
    "standing_neutral_valid": false,
    "seated_neutral_valid": false,
    "reclined_neutral_valid": false,
    "pelvis_width": 0.32,
    "left_femur": 0.42,
    "right_femur": 0.42,
    "left_tibia": 0.42,
    "right_tibia": 0.42,
    "left_foot_length": 0.24,
    "right_foot_length": 0.24,
    "standing_hmd_to_pelvis": [0.0, -0.75, 0.0],
    "seated_hmd_to_pelvis": [0.0, -0.55, -0.15],
    "reclined_hmd_to_pelvis": [0.0, -0.25, -0.45],
    "quality": {
      "pelvis_width": 0.0,
      "left_femur": 0.0,
      "right_femur": 0.0,
      "left_tibia": 0.0,
      "right_tibia": 0.0,
      "left_foot_length": 0.0,
      "right_foot_length": 0.0,
      "standing_hmd_to_pelvis": 0.0,
      "overall": 0.0,
      "sample_count": 0,
      "source": "defaults"
    }
  }
}
)JSON";
    return WriteTextFileAtomically(path, std::string(kTemplate) + '\n', "calibration template");
}


namespace {

nlohmann::json Vec3ToJson(const Vec3f& v) {
    return nlohmann::json::array({v.x, v.y, v.z});
}

template <typename T, std::size_t N>
nlohmann::json ArrayToJson(const std::array<T, N>& values) {
    auto out = nlohmann::json::array();
    for (const auto& value : values) {
        out.push_back(value);
    }
    return out;
}

nlohmann::json Mat34ToJson(const Mat34f& m) {
    auto out = nlohmann::json::array();
    for (const auto value : m.m) {
        out.push_back(value);
    }
    return out;
}

nlohmann::json CameraToJson(const CameraCalibration& c) {
    return {
        {"intrinsics_valid", c.intrinsics_valid},
        {"extrinsics_valid", c.extrinsics_valid},
        {"camera_matrix", ArrayToJson(c.camera_matrix)},
        {"distortion", ArrayToJson(c.distortion)},
        {"world_from_camera", Mat34ToJson(c.world_from_camera)},
        {"image_from_world", Mat34ToJson(c.image_from_world)}
    };
}

nlohmann::json BodyQualityToJson(const BodyCalibrationQuality& q) {
    return {
        {"pelvis_width", q.pelvis_width},
        {"left_femur", q.left_femur},
        {"right_femur", q.right_femur},
        {"left_tibia", q.left_tibia},
        {"right_tibia", q.right_tibia},
        {"left_foot_length", q.left_foot_length},
        {"right_foot_length", q.right_foot_length},
        {"standing_hmd_to_pelvis", q.standing_hmd_to_pelvis},
        {"overall", q.overall},
        {"sample_count", q.sample_count},
        {"source", q.source}
    };
}


nlohmann::json Vec2ToJson(const Vec2f& v) {
    return nlohmann::json::array({v.x, v.y});
}

nlohmann::json FloorLineFamilyToJson(const FloorGeometryLineFamily& f) {
    return {
        {"valid", f.valid},
        {"confidence", f.confidence},
        {"orientation_rad", f.orientation_rad},
        {"spacing_px", f.spacing_px},
        {"spacing_m", f.spacing_m},
        {"metric_spacing_valid", f.metric_spacing_valid},
        {"reference_rho_px", f.reference_rho_px},
        {"vanishing_point_px", Vec2ToJson(f.vanishing_point_px)},
        {"vanishing_point_valid", f.vanishing_point_valid},
        {"accepted_line_count", f.accepted_line_count},
        {"rejected_line_count", f.rejected_line_count},
        {"reason", f.reason}
    };
}

nlohmann::json DistortionToJson(const LensDistortionEstimate& d) {
    return {
        {"available", d.available},
        {"valid", d.valid},
        {"applied_to_runtime", d.applied_to_runtime},
        {"confidence", d.confidence},
        {"radial_k1", d.radial_k1},
        {"radial_k2", d.radial_k2},
        {"tangential_p1", d.tangential_p1},
        {"tangential_p2", d.tangential_p2},
        {"straightness_error_px", d.straightness_error_px},
        {"corrected_straightness_error_px", d.corrected_straightness_error_px},
        {"sampled_seam_count", d.sampled_seam_count},
        {"sampled_point_count", d.sampled_point_count},
        {"model", d.model},
        {"reason", d.reason}
    };
}

nlohmann::json WallRectangleToJson(const WallRectangleCalibration& w) {
    auto corners = nlohmann::json::array();
    for (const auto& p : w.image_corners) {
        corners.push_back(Vec2ToJson(p));
    }
    return {
        {"valid", w.valid},
        {"image_width", w.image_width},
        {"image_height", w.image_height},
        {"source", w.source},
        {"confidence", w.confidence},
        {"reason", w.reason},
        {"image_corners", corners},
        {"rectangle_width_m", w.rectangle_width_m},
        {"rectangle_height_m", w.rectangle_height_m},
        {"rectangle_aspect_ratio", w.rectangle_aspect_ratio},
        {"wall_homography_valid", w.wall_homography_valid},
        {"wall_from_image", ArrayToJson(w.wall_from_image)},
        {"image_from_wall", ArrayToJson(w.image_from_wall)},
        {"homography_reprojection_error_px", w.homography_reprojection_error_px},
        {"metric_scale_valid", w.metric_scale_valid},
        {"metric_scale_confidence", w.metric_scale_confidence},
        {"wall_orientation_valid", w.wall_orientation_valid},
        {"wall_right_camera", Vec3ToJson(w.wall_right_camera)},
        {"wall_down_camera", Vec3ToJson(w.wall_down_camera)},
        {"wall_normal_camera", Vec3ToJson(w.wall_normal_camera)},
        {"wall_orientation_confidence", w.wall_orientation_confidence},
        {"wall_depth_valid", w.wall_depth_valid},
        {"wall_center_depth_m", w.wall_center_depth_m},
        {"wall_depth_confidence", w.wall_depth_confidence},
        {"usable_for_wall_homography", w.usable_for_wall_homography},
        {"usable_for_metric_scale", w.usable_for_metric_scale},
        {"usable_for_orientation", w.usable_for_orientation},
        {"usable_for_depth_assist", w.usable_for_depth_assist},
        {"usable_for_floor_plane", w.usable_for_floor_plane},
        {"usable_for_floor_homography", w.usable_for_floor_homography},
        {"applied_to_runtime", w.applied_to_runtime},
        {"capability_reason", w.capability_reason}
    };
}

nlohmann::json WallRectanglesToJson(const std::vector<WallRectangleCalibration>& walls) {
    auto out = nlohmann::json::array();
    for (const auto& wall : walls) {
        if (wall.valid) {
            out.push_back(WallRectangleToJson(wall));
        }
    }
    return out;
}

nlohmann::json FloorGeometryToJson(const FloorGeometryCalibration& g) {
    return {
        {"valid", g.valid},
        {"source", g.valid ? ((g.source.empty() || g.source == "unknown") ? std::string("legacy_json") : g.source) : std::string("nothing")},
        {"image_width", g.image_width},
        {"image_height", g.image_height},
        {"floor_type", g.floor_type},
        {"family_count", g.family_count},
        {"family_a", FloorLineFamilyToJson(g.family_a)},
        {"family_b", FloorLineFamilyToJson(g.family_b)},
        {"two_axis_grid_valid", g.two_axis_grid_valid},
        {"homography_valid", g.homography_valid},
        {"homography_reprojection_error_px", g.homography_reprojection_error_px},
        {"homography_inlier_count", g.homography_inlier_count},
        {"homography_intersection_count", g.homography_intersection_count},
        {"homography_reason", g.homography_reason},
        {"floor_from_image", ArrayToJson(g.floor_from_image)},
        {"image_from_floor", ArrayToJson(g.image_from_floor)},
        {"floor_plane", {
            {"normal", Vec3ToJson(g.floor_plane.normal)},
            {"distance", g.floor_plane.distance},
            {"valid", g.floor_plane.valid}
        }},
        {"floor_plane_confidence", g.floor_plane_confidence},
        {"camera_orientation_valid", g.camera_orientation_valid},
        {"camera_pitch_rad", g.camera_pitch_rad},
        {"camera_roll_rad", g.camera_roll_rad},
        {"camera_yaw_rad", g.camera_yaw_rad},
        {"camera_orientation_confidence", g.camera_orientation_confidence},
        {"camera_orientation_applied_to_runtime", g.camera_orientation_applied_to_runtime},
        {"camera_height_valid", g.camera_height_valid},
        {"camera_height_m", g.camera_height_m},
        {"metric_scale_confidence", g.metric_scale_confidence},
        {"distortion", DistortionToJson(g.distortion)},
        {"multi_camera_alignment_confidence", g.multi_camera_alignment_confidence},
        {"multi_camera_alignment_valid", g.multi_camera_alignment_valid},
        {"multi_camera_warning", g.multi_camera_warning},
        {"multi_camera_yaw_delta_rad", g.multi_camera_yaw_delta_rad},
        {"multi_camera_pitch_delta_rad", g.multi_camera_pitch_delta_rad},
        {"multi_camera_roll_delta_rad", g.multi_camera_roll_delta_rad},
        {"multi_camera_height_delta_m", g.multi_camera_height_delta_m},
        {"multi_camera_scale_ratio", g.multi_camera_scale_ratio},
        {"shared_floor_frame_valid", g.shared_floor_frame_valid},
        {"shared_floor_transform", ArrayToJson(g.shared_floor_transform)},
        {"planted_drift_axis_confidence", g.planted_drift_axis_confidence},
        {"reason", g.reason}
    };
}

} // namespace

Status SaveCalibrationBundle(const CalibrationBundle& bundle, const std::filesystem::path& path) {
    nlohmann::json j = {
        {"schema_version", bundle.schema_version},
        {"world_handedness", bundle.world_handedness},
        {"world_up_axis", bundle.world_up_axis},
        {"tracking_ready", EvaluateCalibrationReadiness(bundle).tracking_ready},
        {"floor_plane", {
            {"normal", Vec3ToJson(bundle.floor.normal)},
            {"distance", bundle.floor.distance},
            {"valid", bundle.floor.valid}
        }},
        {"floor_geometry", FloorGeometryToJson(bundle.floor_geometry)},
        {"wall_rectangles", WallRectanglesToJson(bundle.wall_rectangles)},
        {"floor_geometry_by_camera", {
            {"camera_a", FloorGeometryToJson(bundle.camera_a_floor_geometry.valid ? bundle.camera_a_floor_geometry : bundle.floor_geometry)},
            {"camera_b", FloorGeometryToJson(bundle.camera_b_floor_geometry)}
        }},
        {"wall_rectangles_by_camera", {
            {"camera_a", WallRectanglesToJson(!bundle.camera_a_wall_rectangles.empty() ? bundle.camera_a_wall_rectangles : bundle.wall_rectangles)},
            {"camera_b", WallRectanglesToJson(bundle.camera_b_wall_rectangles)}
        }},
        {"camera_a", CameraToJson(bundle.camera_a)},
        {"camera_b", CameraToJson(bundle.camera_b)},
        {"body", {
            {"standing_neutral_valid", bundle.body.standing_neutral_valid},
            {"seated_neutral_valid", bundle.body.seated_neutral_valid},
            {"reclined_neutral_valid", bundle.body.reclined_neutral_valid},
            {"pelvis_width", bundle.body.pelvis_width},
            {"left_femur", bundle.body.left_femur},
            {"right_femur", bundle.body.right_femur},
            {"left_tibia", bundle.body.left_tibia},
            {"right_tibia", bundle.body.right_tibia},
            {"left_foot_length", bundle.body.left_foot_length},
            {"right_foot_length", bundle.body.right_foot_length},
            {"standing_hmd_to_pelvis", Vec3ToJson(bundle.body.standing_hmd_to_pelvis)},
            {"seated_hmd_to_pelvis", Vec3ToJson(bundle.body.seated_hmd_to_pelvis)},
            {"reclined_hmd_to_pelvis", Vec3ToJson(bundle.body.reclined_hmd_to_pelvis)},
            {"quality", BodyQualityToJson(bundle.body.quality)}
        }}
    };

    return WriteTextFileAtomically(path, j.dump(2) + '\n', "calibration bundle");
}

CalibrationReadiness EvaluateCalibrationReadiness(const CalibrationBundle& bundle, TrackingMode mode) {
    CalibrationReadiness r;
    if (mode == TrackingMode::Monocular) {
        // Monocular mode intentionally avoids stereo/chessboard calibration. Runtime
        // readiness comes from the single-camera profile plus markerless metric
        // assumptions: camera height, user scale, and optional floor-scale assist.
        r.camera_a_ready = true;
        r.camera_b_ready = false;
        r.cameras_ready = true;
        r.floor_ready = true;
        r.lower_body_ready = true;
        r.tracking_ready = true;
        r.summary = "mode=monocular camera=single_profile floor=virtual_y0_or_floor_assist body=configured_scale tracking=ready";
        return r;
    }

    r.camera_a_ready = CameraReady(bundle.camera_a);
    r.camera_b_ready = CameraReady(bundle.camera_b);
    r.cameras_ready = r.camera_a_ready && r.camera_b_ready;
    r.floor_ready = FloorReady(bundle.floor) || (bundle.floor_geometry.valid && bundle.floor_geometry.floor_plane.valid);
    r.lower_body_ready = BodyReady(bundle.body);
    r.tracking_ready = r.camera_a_ready || r.camera_b_ready || r.floor_ready || r.lower_body_ready;

    std::ostringstream oss;
    oss << "cameras=" << (r.cameras_ready ? "ready" : "not_ready")
        << " floor=" << (r.floor_ready ? "ready" : "not_ready")
        << " body=" << (r.lower_body_ready ? "ready" : "not_ready")
        << " tracking=" << (r.tracking_ready ? "ready" : "not_ready");
    r.summary = oss.str();
    return r;
}

} // namespace bt
