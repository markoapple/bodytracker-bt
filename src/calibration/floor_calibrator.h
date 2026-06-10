#pragma once

#include "calibration/calibration_types.h"
#include "core/status.h"

#include <array>
#include <string>
#include <vector>

namespace bt {

struct FloorSeamLine2D {
    Vec2f a{};
    Vec2f b{};
    float strength = 1.0f;

    // Optional sampled edge points along the same physical seam. Segment endpoints
    // are enough for repeated-line detection, but distortion estimation needs the
    // curve itself. Empty means "only endpoints were observed".
    std::vector<Vec2f> samples{};
};

struct FloorSeamCandidateDebug {
    FloorSeamLine2D line{};
    float angle_rad = 0.0f;
    float rho_px = 0.0f;
    bool accepted = false;
    std::string reason;
};

struct FloorSeamFamilyEstimate {
    bool valid = false;
    float confidence = 0.0f;
    float orientation_rad = 0.0f;
    float spacing_px = 0.0f;
    float reference_rho_px = 0.0f;
    int accepted_line_count = 0;
    int rejected_line_count = 0;
    std::vector<FloorSeamCandidateDebug> candidates{};
    std::string reason;
};

struct FloorPatternPatchVote {
    int x = 0;
    int y = 0;
    int width = 0;
    int height = 0;
    float score = 0.0f;
    bool accepted = false;
};

struct FloorPatternDetectionDebug {
    bool valid = false;
    std::string pattern_type = "not_floor_pattern";
    float confidence = 0.0f;
    float spacing_confidence = 0.0f;
    float orientation_rad = 0.0f;
    float secondary_orientation_rad = 0.0f;
    bool secondary_valid = false;
    float spacing_px = 0.0f;
    float secondary_spacing_px = 0.0f;
    int raw_segment_count = 0;
    int rejected_grain_segment_count = 0;
    int patch_count = 0;
    int accepted_patch_count = 0;
    int bottom_patch_count = 0;
    int major_seam_count = 0;
    int secondary_seam_count = 0;
    int finite_supported_seam_count = 0;
    int lattice_inlier_count = 0;
    int lattice_run_length = 0;
    float lattice_residual_px = 0.0f;
    float seam_evidence_confidence = 0.0f;
    float butt_joint_confidence = 0.0f;
    int butt_joint_lane_count = 0;
    int butt_joint_axial_bucket_count = 0;
    std::vector<FloorSeamLine2D> major_seams{};
    std::vector<FloorSeamLine2D> secondary_seams{};
    std::vector<FloorPatternPatchVote> patches{};
    std::string reason;
};

struct FloorGeometryCalibrationOptions {
    std::string floor_type = "unknown";
    float family_a_spacing_m = 0.0f;
    float family_b_spacing_m = 0.0f;
    bool intrinsics_available = false;
    float horizontal_fov_deg = 0.0f;
    float camera_height_m = 0.0f;
};

struct WallRectangleCalibrationOptions {
    std::string source = "manual_wall_rectangle";
    float rectangle_width_m = 0.0f;
    float rectangle_height_m = 0.0f;
    float rectangle_aspect_ratio = 0.0f;
    bool intrinsics_available = false;
    float horizontal_fov_deg = 0.0f;
};

struct MultiCameraFloorAlignmentEstimate {
    bool valid = false;
    float confidence = 0.0f;
    float yaw_delta_rad = 0.0f;
    float pitch_delta_rad = 0.0f;
    float roll_delta_rad = 0.0f;
    float height_delta_m = 0.0f;
    float scale_ratio = 1.0f;
    bool shared_floor_frame_valid = false;
    std::array<float, 9> floor_b_from_floor_a{};
    std::string reason;
};

struct FloorGeometryDetectionDebug {
    FloorGeometryCalibration calibration{};
    FloorPatternDetectionDebug pattern{};
    std::vector<FloorSeamCandidateDebug> candidates{};
    std::vector<FloorSeamFamilyEstimate> rejected_families{};
};

Result<FloorPlane> EstimateFloorPlaneFromWorldPoints(const std::vector<Vec3f>& points);
FloorSeamFamilyEstimate EstimateRepeatedFloorSeamFamily(
    const std::vector<FloorSeamLine2D>& lines,
    int image_width,
    int image_height);

FloorGeometryDetectionDebug EstimateFloorGeometryCalibration(
    const std::vector<FloorSeamLine2D>& lines,
    int image_width,
    int image_height,
    const FloorGeometryCalibrationOptions& options = {});

WallRectangleCalibration EstimateWallRectangleCalibration(
    const std::array<Vec2f, 4>& image_points,
    int image_width,
    int image_height,
    const WallRectangleCalibrationOptions& options = {});

MultiCameraFloorAlignmentEstimate EstimateMultiCameraFloorAlignment(
    const FloorGeometryCalibration& camera_a,
    const FloorGeometryCalibration& camera_b);

} // namespace bt
