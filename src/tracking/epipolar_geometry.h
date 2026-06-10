#pragma once

#include "calibration/calibration_types.h"
#include "core/math.h"
#include "core/status.h"

namespace bt {

enum class EpipolarCheckReason {
    Ok,
    InvalidGeometry,
    InvalidCamera,
    NonFinitePoint,
    InvalidThreshold,
    DegenerateDenominator,
    NonFiniteError
};

enum class EpipolarCoordinateSpace {
    PixelFundamental,
    NormalizedEssential
};

struct EpipolarGeometryDiagnostics {
    float camera_a_intrinsics_determinant = 0.0f;
    float camera_b_intrinsics_determinant = 0.0f;
    float camera_a_intrinsics_condition = 0.0f;
    float camera_b_intrinsics_condition = 0.0f;
    float camera_a_rotation_determinant = 0.0f;
    float camera_b_rotation_determinant = 0.0f;
    float relative_rotation_determinant = 0.0f;
    float camera_a_rotation_condition = 0.0f;
    float camera_b_rotation_condition = 0.0f;
    float baseline_meters = 0.0f;
    float essential_a_to_b_rank_residual = 0.0f;
    float essential_b_to_a_rank_residual = 0.0f;
    float fundamental_a_to_b_rank_residual = 0.0f;
    float fundamental_b_to_a_rank_residual = 0.0f;
};

struct EpipolarGeometry {
    Mat3f fundamental_a_to_b{};
    Mat3f fundamental_b_to_a{};
    Mat3f essential_a_to_b{};
    Mat3f essential_b_to_a{};
    EpipolarGeometryDiagnostics diagnostics{};
    bool valid = false;
};

struct EpipolarCheck {
    // Primary pixel-space error currently used by runtime pixel thresholds.
    // For distortion-safe normalized checks this is the legacy isotropic
    // average-focal heuristic kept for compatibility; use the explicitly named
    // fields below when inspecting metric meaning.
    float sampson_error_px = 0.0f;
    float sampson_error_px_isotropic = 0.0f;
    float sampson_error_px_anisotropic = 0.0f;
    float sampson_error_normalized = 0.0f;
    float confidence = 0.0f;
    bool hard_mismatch = false;
    bool valid = false;
    EpipolarCheckReason reason = EpipolarCheckReason::InvalidGeometry;
    EpipolarCoordinateSpace coordinate_space = EpipolarCoordinateSpace::PixelFundamental;
};

inline const char* ToString(EpipolarCheckReason reason) {
    switch (reason) {
    case EpipolarCheckReason::Ok: return "ok";
    case EpipolarCheckReason::InvalidGeometry: return "invalid_geometry";
    case EpipolarCheckReason::InvalidCamera: return "invalid_camera";
    case EpipolarCheckReason::NonFinitePoint: return "non_finite_point";
    case EpipolarCheckReason::InvalidThreshold: return "invalid_threshold";
    case EpipolarCheckReason::DegenerateDenominator: return "degenerate_denominator";
    case EpipolarCheckReason::NonFiniteError: return "non_finite_error";
    default: return "unknown";
    }
}

inline const char* ToString(EpipolarCoordinateSpace space) {
    switch (space) {
    case EpipolarCoordinateSpace::PixelFundamental: return "pixel_fundamental";
    case EpipolarCoordinateSpace::NormalizedEssential: return "normalized_essential";
    default: return "unknown";
    }
}

Result<EpipolarGeometry> ComputeEpipolarGeometry(
    const CameraCalibration& camera_a,
    const CameraCalibration& camera_b);

Result<Vec2f> UndistortPixelToNormalized(
    const CameraCalibration& camera,
    Vec2f distorted_pixel);

EpipolarCheck ComputePixelSampsonEpipolarCheck(
    const EpipolarGeometry& geometry,
    Vec2f pixel_point_a,
    Vec2f pixel_point_b,
    float soft_threshold_px,
    float hard_threshold_px);

EpipolarCheck ComputeNormalizedSampsonEpipolarCheck(
    const EpipolarGeometry& geometry,
    Vec2f normalized_point_a,
    Vec2f normalized_point_b,
    float soft_threshold_normalized,
    float hard_threshold_normalized);

EpipolarCheck ComputeDistortionSafePixelSampsonEpipolarCheck(
    const EpipolarGeometry& geometry,
    const CameraCalibration& camera_a,
    const CameraCalibration& camera_b,
    Vec2f distorted_pixel_a,
    Vec2f distorted_pixel_b,
    float soft_threshold_px,
    float hard_threshold_px);

} // namespace bt
