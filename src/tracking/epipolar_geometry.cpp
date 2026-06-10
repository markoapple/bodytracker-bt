#include "tracking/epipolar_geometry.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace bt {
namespace {

// A calibrated image-space focal length below one pixel is below the
// sampling resolution of the image itself. Letting sub-pixel focal lengths
// through would make pixel-to-ray normalization numerically explosive while
// still looking "valid" to a finite/nonzero matrix check.
constexpr double kMinFocalPixels = 1.0;
constexpr double kMinHomogeneousScale = 1.0e-6;
constexpr double kMinCameraMatrixDeterminant = 1.0e-9;
constexpr double kMinRotationMatrixDeterminant = 1.0e-9;
constexpr double kMaxCameraMatrixCondition = 1.0e8;
constexpr double kMaxRotationMatrixCondition = 1.0e6;
constexpr double kRotationDeterminantTolerance = 1.0e-3;
constexpr double kMaxEpipolarRankResidual = 1.0e-5;
constexpr double kMinEpipolarRank2MinorResidual = 1.0e-8;
constexpr double kMinEpipolarMatrixNormSquared = 1.0e-18;
// Below one millimetre the stereo baseline is physically too small for
// stable body-tracking triangulation and makes the epipolar denominator
// behave like a calibration-scale artifact rather than useful geometry.
constexpr double kMinBaselineMeters = 1.0e-3;
constexpr double kMinSampsonDenominator = 1.0e-12;
constexpr double kMinDistortionRadial = 1.0e-9;
constexpr int kUndistortIterations = 8;

using Mat3d = std::array<double, 9>;
using Vec3d = std::array<double, 3>;

bool IsFinite(double value) {
    return std::isfinite(value);
}

bool IsFinite(float value) {
    return std::isfinite(value);
}

bool IsFinite(const Vec2f& value) {
    return IsFinite(value.x) && IsFinite(value.y);
}

bool IsFinite(const Mat3d& m) {
    for (double value : m) {
        if (!IsFinite(value)) {
            return false;
        }
    }
    return true;
}

bool IsFinite(const Mat34f& m) {
    for (float value : m.m) {
        if (!IsFinite(value)) {
            return false;
        }
    }
    return true;
}

bool IsFinite(const Vec3d& v) {
    return IsFinite(v[0]) && IsFinite(v[1]) && IsFinite(v[2]);
}

Mat3d CameraMatrixToMat3d(const CameraCalibration& camera) {
    Mat3d out{};
    for (std::size_t i = 0; i < out.size(); ++i) {
        out[i] = camera.camera_matrix[i];
    }
    return out;
}

Mat3d RotationFromTransform(const Mat34f& transform) {
    return Mat3d{
        static_cast<double>(transform.m[0]), static_cast<double>(transform.m[1]), static_cast<double>(transform.m[2]),
        static_cast<double>(transform.m[4]), static_cast<double>(transform.m[5]), static_cast<double>(transform.m[6]),
        static_cast<double>(transform.m[8]), static_cast<double>(transform.m[9]), static_cast<double>(transform.m[10])
    };
}

Vec3d TranslationFromTransform(const Mat34f& transform) {
    return Vec3d{
        static_cast<double>(transform.m[3]),
        static_cast<double>(transform.m[7]),
        static_cast<double>(transform.m[11])
    };
}

Mat3d Transpose(const Mat3d& a) {
    return Mat3d{
        a[0], a[3], a[6],
        a[1], a[4], a[7],
        a[2], a[5], a[8]
    };
}

Mat3d Multiply(const Mat3d& a, const Mat3d& b) {
    Mat3d out{};
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            double value = 0.0;
            for (int k = 0; k < 3; ++k) {
                value += a[static_cast<std::size_t>(3 * r + k)] * b[static_cast<std::size_t>(3 * k + c)];
            }
            out[static_cast<std::size_t>(3 * r + c)] = value;
        }
    }
    return out;
}

Vec3d Multiply(const Mat3d& a, const Vec3d& v) {
    return Vec3d{
        a[0] * v[0] + a[1] * v[1] + a[2] * v[2],
        a[3] * v[0] + a[4] * v[1] + a[5] * v[2],
        a[6] * v[0] + a[7] * v[1] + a[8] * v[2]
    };
}

Vec3d Add(const Vec3d& a, const Vec3d& b) {
    return Vec3d{a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}

double Length(const Vec3d& v) {
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

double Determinant(const Mat3d& m) {
    return
        m[0] * (m[4] * m[8] - m[5] * m[7]) -
        m[1] * (m[3] * m[8] - m[5] * m[6]) +
        m[2] * (m[3] * m[7] - m[4] * m[6]);
}

double FrobeniusNorm(const Mat3d& m) {
    double sum_sq = 0.0;
    for (double value : m) {
        sum_sq += value * value;
    }
    if (!IsFinite(sum_sq) || sum_sq < 0.0) {
        return std::numeric_limits<double>::infinity();
    }
    return std::sqrt(sum_sq);
}

struct ConditionedInverse {
    Mat3d inverse{};
    double determinant = 0.0;
    double frobenius_norm = 0.0;
    double inverse_frobenius_norm = 0.0;
    double condition_estimate = std::numeric_limits<double>::infinity();
    bool finite = false;
    bool invertible = false;
    bool well_conditioned = false;
};

ConditionedInverse ComputeConditionedInverse(
    const Mat3d& m,
    double min_abs_determinant,
    double max_condition_estimate) {

    ConditionedInverse out;
    out.finite = IsFinite(m);
    if (!out.finite) {
        return out;
    }

    out.determinant = Determinant(m);
    out.frobenius_norm = FrobeniusNorm(m);
    if (!IsFinite(out.determinant) || !IsFinite(out.frobenius_norm) ||
        out.frobenius_norm <= 0.0 || std::abs(out.determinant) < min_abs_determinant) {
        return out;
    }

    const double inv_det = 1.0 / out.determinant;
    out.inverse = Mat3d{
        (m[4] * m[8] - m[5] * m[7]) * inv_det,
        (m[2] * m[7] - m[1] * m[8]) * inv_det,
        (m[1] * m[5] - m[2] * m[4]) * inv_det,
        (m[5] * m[6] - m[3] * m[8]) * inv_det,
        (m[0] * m[8] - m[2] * m[6]) * inv_det,
        (m[2] * m[3] - m[0] * m[5]) * inv_det,
        (m[3] * m[7] - m[4] * m[6]) * inv_det,
        (m[1] * m[6] - m[0] * m[7]) * inv_det,
        (m[0] * m[4] - m[1] * m[3]) * inv_det
    };
    if (!IsFinite(out.inverse)) {
        return out;
    }

    out.invertible = true;
    out.inverse_frobenius_norm = FrobeniusNorm(out.inverse);
    out.condition_estimate = out.frobenius_norm * out.inverse_frobenius_norm;
    out.well_conditioned =
        IsFinite(out.inverse_frobenius_norm) &&
        IsFinite(out.condition_estimate) &&
        out.condition_estimate <= max_condition_estimate;
    return out;
}

bool MatrixConditionedForUse(
    const Mat3d& m,
    double min_abs_determinant,
    double max_condition_estimate) {
    const ConditionedInverse inverse = ComputeConditionedInverse(
        m,
        min_abs_determinant,
        max_condition_estimate);
    return inverse.invertible && inverse.well_conditioned;
}

Mat3d SkewSymmetric(const Vec3d& t) {
    return Mat3d{
        0.0, -t[2], t[1],
        t[2], 0.0, -t[0],
        -t[1], t[0], 0.0
    };
}

bool CameraIntrinsicsUsable(const CameraCalibration& camera) {
    if (!camera.intrinsics_valid) {
        return false;
    }
    const Mat3d k = CameraMatrixToMat3d(camera);
    if (!IsFinite(k)) {
        return false;
    }

    const double fx = k[0];
    const double fy = k[4];
    const double cx = k[2];
    const double cy = k[5];
    const double homogeneous_scale = k[8];
    if (!IsFinite(fx) || !IsFinite(fy) || !IsFinite(cx) || !IsFinite(cy) ||
        !IsFinite(homogeneous_scale)) {
        return false;
    }
    if (std::abs(fx) < kMinFocalPixels || std::abs(fy) < kMinFocalPixels ||
        std::abs(homogeneous_scale) < kMinHomogeneousScale) {
        return false;
    }

    return MatrixConditionedForUse(
        k,
        kMinCameraMatrixDeterminant,
        kMaxCameraMatrixCondition);
}

bool CameraDistortionUsable(const CameraCalibration& camera) {
    for (double value : camera.distortion) {
        if (!IsFinite(value)) {
            return false;
        }
    }
    return true;
}

double MatrixNormSquared(const Mat3d& m) {
    double scale_sq = 0.0;
    for (double value : m) {
        scale_sq += value * value;
    }
    return scale_sq;
}

Mat3d NormalizeMatrix(const Mat3d& m) {
    const double scale_sq = MatrixNormSquared(m);
    if (!IsFinite(scale_sq) || scale_sq <= 1.0e-24) {
        return m;
    }
    const double inv_scale = 1.0 / std::sqrt(scale_sq);
    Mat3d out{};
    for (std::size_t i = 0; i < out.size(); ++i) {
        out[i] = m[i] * inv_scale;
    }
    return out;
}

Mat3f ToMat3f(const Mat3d& in) {
    Mat3f out{};
    for (std::size_t i = 0; i < out.m.size(); ++i) {
        out.m[i] = static_cast<float>(in[i]);
    }
    return out;
}

Mat3d ToMat3d(const Mat3f& in) {
    Mat3d out{};
    for (std::size_t i = 0; i < out.size(); ++i) {
        out[i] = static_cast<double>(in.m[i]);
    }
    return out;
}

float ToDiagnosticFloat(double value) {
    if (!IsFinite(value)) {
        return 0.0f;
    }
    return static_cast<float>(value);
}

bool EpipolarMatrixNumericallyUsable(const Mat3d& m) {
    if (!IsFinite(m)) {
        return false;
    }
    const double norm_sq = MatrixNormSquared(m);
    return IsFinite(norm_sq) && norm_sq > kMinEpipolarMatrixNormSquared;
}


Mat3d Adjugate(const Mat3d& m) {
    return Mat3d{
        m[4] * m[8] - m[5] * m[7],
        m[2] * m[7] - m[1] * m[8],
        m[1] * m[5] - m[2] * m[4],
        m[5] * m[6] - m[3] * m[8],
        m[0] * m[8] - m[2] * m[6],
        m[2] * m[3] - m[0] * m[5],
        m[3] * m[7] - m[4] * m[6],
        m[1] * m[6] - m[0] * m[7],
        m[0] * m[4] - m[1] * m[3]
    };
}

double EpipolarRankResidual(const Mat3d& m) {
    if (!EpipolarMatrixNumericallyUsable(m)) {
        return std::numeric_limits<double>::infinity();
    }
    const double norm = FrobeniusNorm(m);
    const double determinant = Determinant(m);
    const double denom = norm * norm * norm;
    if (!IsFinite(determinant) || !IsFinite(denom) || denom <= 0.0) {
        return std::numeric_limits<double>::infinity();
    }
    return std::abs(determinant) / denom;
}

double EpipolarRank2MinorResidual(const Mat3d& m) {
    if (!EpipolarMatrixNumericallyUsable(m)) {
        return 0.0;
    }
    const double norm = FrobeniusNorm(m);
    const double denom = norm * norm;
    const double adjugate_norm = FrobeniusNorm(Adjugate(m));
    if (!IsFinite(adjugate_norm) || !IsFinite(denom) || denom <= 0.0) {
        return 0.0;
    }
    return adjugate_norm / denom;
}

bool EpipolarRank2Usable(const Mat3d& m) {
    const double determinant_residual = EpipolarRankResidual(m);
    const double rank2_minor_residual = EpipolarRank2MinorResidual(m);
    return IsFinite(determinant_residual) &&
        determinant_residual <= kMaxEpipolarRankResidual &&
        IsFinite(rank2_minor_residual) &&
        rank2_minor_residual >= kMinEpipolarRank2MinorResidual;
}

bool ProperRotationDeterminant(double determinant) {
    return IsFinite(determinant) &&
        std::abs(determinant - 1.0) <= kRotationDeterminantTolerance;
}

float Clamp01(float value) {
    if (!std::isfinite(value)) {
        return 0.0f;
    }
    return std::max(0.0f, std::min(1.0f, value));
}

float ConfidenceFromError(float error, float soft_threshold) {
    if (!std::isfinite(error) || error < 0.0f ||
        !std::isfinite(soft_threshold) || soft_threshold <= 0.0f) {
        return 0.0f;
    }
    const double ratio = static_cast<double>(error) / static_cast<double>(soft_threshold);
    const double squared = ratio * ratio;
    if (!std::isfinite(squared)) {
        return 0.0f;
    }
    return Clamp01(static_cast<float>(1.0 / (1.0 + squared)));
}

float IsotropicFocalHeuristicPixels(const CameraCalibration& camera_a, const CameraCalibration& camera_b) {
    const float fx_a = static_cast<float>(std::abs(camera_a.camera_matrix[0]));
    const float fy_a = static_cast<float>(std::abs(camera_a.camera_matrix[4]));
    const float fx_b = static_cast<float>(std::abs(camera_b.camera_matrix[0]));
    const float fy_b = static_cast<float>(std::abs(camera_b.camera_matrix[4]));
    const float average = 0.25f * (fx_a + fy_a + fx_b + fy_b);
    if (!std::isfinite(average) || average <= 0.0f) {
        return 1.0f;
    }
    return average;
}

struct SampsonError {
    float value = 0.0f;
    EpipolarCheckReason reason = EpipolarCheckReason::InvalidGeometry;
    bool valid = false;
};

bool ThresholdsUsable(float soft_threshold, float hard_threshold) {
    return std::isfinite(soft_threshold) && soft_threshold > 0.0f &&
        std::isfinite(hard_threshold) && hard_threshold > 0.0f;
}

SampsonError ComputeSampsonErrorFromMatrix(
    const Mat3d& matrix_a_to_b,
    Vec2f point_a,
    Vec2f point_b) {

    SampsonError out;
    if (!IsFinite(point_a) || !IsFinite(point_b)) {
        out.reason = EpipolarCheckReason::NonFinitePoint;
        return out;
    }
    if (!EpipolarMatrixNumericallyUsable(matrix_a_to_b) ||
        !EpipolarRank2Usable(matrix_a_to_b)) {
        out.reason = EpipolarCheckReason::InvalidGeometry;
        return out;
    }

    const Mat3d matrix = NormalizeMatrix(matrix_a_to_b);
    if (!EpipolarMatrixNumericallyUsable(matrix)) {
        out.reason = EpipolarCheckReason::InvalidGeometry;
        return out;
    }

    const Vec3d xa{static_cast<double>(point_a.x), static_cast<double>(point_a.y), 1.0};
    const Vec3d xb{static_cast<double>(point_b.x), static_cast<double>(point_b.y), 1.0};
    const Vec3d m_xa = Multiply(matrix, xa);
    const Vec3d mt_xb = Multiply(Transpose(matrix), xb);
    const double xbm_xa = xb[0] * m_xa[0] + xb[1] * m_xa[1] + xb[2] * m_xa[2];
    const double denom = m_xa[0] * m_xa[0] + m_xa[1] * m_xa[1] +
        mt_xb[0] * mt_xb[0] + mt_xb[1] * mt_xb[1];

    if (!IsFinite(xbm_xa) || !IsFinite(denom) || denom <= kMinSampsonDenominator) {
        out.reason = EpipolarCheckReason::DegenerateDenominator;
        return out;
    }

    const double d2 = (xbm_xa * xbm_xa) / denom;
    if (!IsFinite(d2) || d2 < 0.0) {
        out.reason = EpipolarCheckReason::NonFiniteError;
        return out;
    }

    const float error = static_cast<float>(std::sqrt(d2));
    if (!std::isfinite(error)) {
        out.reason = EpipolarCheckReason::NonFiniteError;
        return out;
    }

    out.value = error;
    out.valid = true;
    out.reason = EpipolarCheckReason::Ok;
    return out;
}

SampsonError ComputeAnisotropicPixelSampsonErrorFromNormalizedMatrix(
    const Mat3d& essential_a_to_b,
    Vec2f normalized_point_a,
    Vec2f normalized_point_b,
    const CameraCalibration& camera_a,
    const CameraCalibration& camera_b) {

    SampsonError out;
    if (!IsFinite(normalized_point_a) || !IsFinite(normalized_point_b)) {
        out.reason = EpipolarCheckReason::NonFinitePoint;
        return out;
    }
    if (!CameraIntrinsicsUsable(camera_a) || !CameraIntrinsicsUsable(camera_b)) {
        out.reason = EpipolarCheckReason::InvalidCamera;
        return out;
    }
    if (!EpipolarMatrixNumericallyUsable(essential_a_to_b) ||
        !EpipolarRank2Usable(essential_a_to_b)) {
        out.reason = EpipolarCheckReason::InvalidGeometry;
        return out;
    }

    const double fx_a = std::abs(camera_a.camera_matrix[0]);
    const double fy_a = std::abs(camera_a.camera_matrix[4]);
    const double fx_b = std::abs(camera_b.camera_matrix[0]);
    const double fy_b = std::abs(camera_b.camera_matrix[4]);
    if (!IsFinite(fx_a) || !IsFinite(fy_a) || !IsFinite(fx_b) || !IsFinite(fy_b) ||
        fx_a < kMinFocalPixels || fy_a < kMinFocalPixels ||
        fx_b < kMinFocalPixels || fy_b < kMinFocalPixels) {
        out.reason = EpipolarCheckReason::InvalidCamera;
        return out;
    }

    const Mat3d matrix = NormalizeMatrix(essential_a_to_b);
    if (!EpipolarMatrixNumericallyUsable(matrix)) {
        out.reason = EpipolarCheckReason::InvalidGeometry;
        return out;
    }

    const Vec3d xa{static_cast<double>(normalized_point_a.x), static_cast<double>(normalized_point_a.y), 1.0};
    const Vec3d xb{static_cast<double>(normalized_point_b.x), static_cast<double>(normalized_point_b.y), 1.0};
    const Vec3d e_xa = Multiply(matrix, xa);
    const Vec3d et_xb = Multiply(Transpose(matrix), xb);
    const double residual = xb[0] * e_xa[0] + xb[1] * e_xa[1] + xb[2] * e_xa[2];

    // Convert the first-order Sampson gradient from normalized-camera units
    // back to pixel units through each camera's anisotropic projection
    // Jacobian. This is the least-squares pixel displacement metric for the
    // linearized epipolar constraint, unlike normalized_error * average_focal.
    const double denom =
        (et_xb[0] / fx_a) * (et_xb[0] / fx_a) +
        (et_xb[1] / fy_a) * (et_xb[1] / fy_a) +
        (e_xa[0] / fx_b) * (e_xa[0] / fx_b) +
        (e_xa[1] / fy_b) * (e_xa[1] / fy_b);

    if (!IsFinite(residual) || !IsFinite(denom) || denom <= kMinSampsonDenominator) {
        out.reason = EpipolarCheckReason::DegenerateDenominator;
        return out;
    }

    const double d2 = (residual * residual) / denom;
    if (!IsFinite(d2) || d2 < 0.0) {
        out.reason = EpipolarCheckReason::NonFiniteError;
        return out;
    }

    const float error = static_cast<float>(std::sqrt(d2));
    if (!std::isfinite(error)) {
        out.reason = EpipolarCheckReason::NonFiniteError;
        return out;
    }

    out.value = error;
    out.valid = true;
    out.reason = EpipolarCheckReason::Ok;
    return out;
}

EpipolarCheck MakeFailedCheck(EpipolarCoordinateSpace coordinate_space, EpipolarCheckReason reason) {
    EpipolarCheck out;
    out.coordinate_space = coordinate_space;
    out.reason = reason;
    return out;
}

EpipolarCheck BuildPixelThresholdedCheck(
    const SampsonError& error,
    EpipolarCoordinateSpace coordinate_space,
    float sampson_error_px,
    float sampson_error_px_isotropic,
    float sampson_error_px_anisotropic,
    float sampson_error_normalized,
    float soft_threshold_px,
    float hard_threshold_px) {

    if (!ThresholdsUsable(soft_threshold_px, hard_threshold_px)) {
        return MakeFailedCheck(coordinate_space, EpipolarCheckReason::InvalidThreshold);
    }
    if (!error.valid) {
        return MakeFailedCheck(coordinate_space, error.reason);
    }
    if (!std::isfinite(sampson_error_px) || sampson_error_px < 0.0f ||
        !std::isfinite(sampson_error_px_isotropic) || sampson_error_px_isotropic < 0.0f ||
        !std::isfinite(sampson_error_px_anisotropic) || sampson_error_px_anisotropic < 0.0f ||
        !std::isfinite(sampson_error_normalized) || sampson_error_normalized < 0.0f) {
        return MakeFailedCheck(coordinate_space, EpipolarCheckReason::NonFiniteError);
    }

    EpipolarCheck out;
    out.coordinate_space = coordinate_space;
    out.sampson_error_px = sampson_error_px;
    out.sampson_error_px_isotropic = sampson_error_px_isotropic;
    out.sampson_error_px_anisotropic = sampson_error_px_anisotropic;
    out.sampson_error_normalized = sampson_error_normalized;
    out.confidence = ConfidenceFromError(sampson_error_px, soft_threshold_px);
    out.hard_mismatch = sampson_error_px > hard_threshold_px;
    out.valid = true;
    out.reason = EpipolarCheckReason::Ok;
    return out;
}

EpipolarCheck BuildNormalizedThresholdedCheck(
    const SampsonError& error,
    float soft_threshold_normalized,
    float hard_threshold_normalized) {

    constexpr EpipolarCoordinateSpace kSpace = EpipolarCoordinateSpace::NormalizedEssential;
    if (!ThresholdsUsable(soft_threshold_normalized, hard_threshold_normalized)) {
        return MakeFailedCheck(kSpace, EpipolarCheckReason::InvalidThreshold);
    }
    if (!error.valid) {
        return MakeFailedCheck(kSpace, error.reason);
    }

    if (!std::isfinite(error.value) || error.value < 0.0f) {
        return MakeFailedCheck(kSpace, EpipolarCheckReason::NonFiniteError);
    }

    EpipolarCheck out;
    out.coordinate_space = kSpace;
    out.sampson_error_px = 0.0f;
    out.sampson_error_px_isotropic = 0.0f;
    out.sampson_error_px_anisotropic = 0.0f;
    out.sampson_error_normalized = error.value;
    out.confidence = ConfidenceFromError(error.value, soft_threshold_normalized);
    out.hard_mismatch = error.value > hard_threshold_normalized;
    out.valid = true;
    out.reason = EpipolarCheckReason::Ok;
    return out;
}

} // namespace

Result<EpipolarGeometry> ComputeEpipolarGeometry(
    const CameraCalibration& camera_a,
    const CameraCalibration& camera_b) {

    if (!CameraIntrinsicsUsable(camera_a) || !CameraIntrinsicsUsable(camera_b)) {
        return Status::Error(StatusCode::FailedPrecondition, "Epipolar geometry requires valid, well-conditioned intrinsics for both cameras");
    }
    if (!CameraDistortionUsable(camera_a) || !CameraDistortionUsable(camera_b)) {
        return Status::Error(StatusCode::InvalidArgument, "Epipolar geometry received non-finite distortion coefficients");
    }
    if (!camera_a.extrinsics_valid || !camera_b.extrinsics_valid) {
        return Status::Error(StatusCode::FailedPrecondition, "Epipolar geometry requires valid extrinsics for both cameras");
    }
    if (!IsFinite(camera_a.world_from_camera) || !IsFinite(camera_b.world_from_camera)) {
        return Status::Error(StatusCode::InvalidArgument, "Epipolar geometry received non-finite camera extrinsics");
    }

    const Mat3d k_a = CameraMatrixToMat3d(camera_a);
    const Mat3d k_b = CameraMatrixToMat3d(camera_b);
    const ConditionedInverse k_a_inverse = ComputeConditionedInverse(
        k_a,
        kMinCameraMatrixDeterminant,
        kMaxCameraMatrixCondition);
    const ConditionedInverse k_b_inverse = ComputeConditionedInverse(
        k_b,
        kMinCameraMatrixDeterminant,
        kMaxCameraMatrixCondition);
    if (!k_a_inverse.invertible || !k_b_inverse.invertible) {
        return Status::Error(StatusCode::FailedPrecondition, "Epipolar geometry requires invertible camera intrinsics");
    }
    if (!k_a_inverse.well_conditioned || !k_b_inverse.well_conditioned) {
        return Status::Error(StatusCode::FailedPrecondition, "Epipolar geometry requires well-conditioned camera intrinsics");
    }
    const Mat3d& k_a_inv = k_a_inverse.inverse;
    const Mat3d& k_b_inv = k_b_inverse.inverse;

    // CameraCalibration::world_from_camera maps camera coordinates into the
    // shared world frame. Convert it to the relative transform from camera A
    // coordinates to camera B coordinates:
    //   X_b = R_ba * X_a + t_ba
    const Mat3d r_wa = RotationFromTransform(camera_a.world_from_camera);
    const Mat3d r_wb = RotationFromTransform(camera_b.world_from_camera);
    const ConditionedInverse r_wa_diagnostics = ComputeConditionedInverse(
        r_wa,
        kMinRotationMatrixDeterminant,
        kMaxRotationMatrixCondition);
    const ConditionedInverse r_wb_diagnostics = ComputeConditionedInverse(
        r_wb,
        kMinRotationMatrixDeterminant,
        kMaxRotationMatrixCondition);
    if (!r_wa_diagnostics.invertible || !r_wb_diagnostics.invertible ||
        !r_wa_diagnostics.well_conditioned || !r_wb_diagnostics.well_conditioned) {
        return Status::Error(StatusCode::FailedPrecondition, "Epipolar geometry requires finite, well-conditioned extrinsics for both cameras");
    }
    if (!ProperRotationDeterminant(r_wa_diagnostics.determinant) ||
        !ProperRotationDeterminant(r_wb_diagnostics.determinant)) {
        return Status::Error(StatusCode::FailedPrecondition, "Epipolar geometry requires proper rotation matrices with determinant +1");
    }
    const Vec3d t_wa = TranslationFromTransform(camera_a.world_from_camera);
    const Vec3d t_wb = TranslationFromTransform(camera_b.world_from_camera);

    const Mat3d r_bw = Transpose(r_wb);
    const Vec3d t_bw = Multiply(r_bw, Vec3d{-t_wb[0], -t_wb[1], -t_wb[2]});
    const Mat3d r_ba = Multiply(r_bw, r_wa);
    const Vec3d t_ba = Add(Multiply(r_bw, t_wa), t_bw);

    if (!IsFinite(r_ba) || !IsFinite(t_ba)) {
        return Status::Error(StatusCode::InvalidArgument, "Epipolar geometry produced non-finite relative pose");
    }
    const double relative_rotation_determinant = Determinant(r_ba);
    if (!ProperRotationDeterminant(relative_rotation_determinant)) {
        return Status::Error(StatusCode::FailedPrecondition, "Epipolar geometry produced a relative rotation that is not a proper +1 rotation");
    }
    const double metric_baseline_meters = Length(t_ba);
    if (metric_baseline_meters <= kMinBaselineMeters) {
        return Status::Error(StatusCode::FailedPrecondition, "Epipolar geometry requires a stereo baseline larger than the 1 mm numerical floor");
    }

    // E_ab = [t_ab]x R_ab maps normalized camera-A points to camera-B epipolar lines.
    // E scale is arbitrary in normalized coordinates, so store a stable normalized E.
    // F_ab is the direct pixel-space product K_b^-T E_ab K_a^-1. Do not
    // renormalize F here; callers that consume pixel-space F must not inherit a
    // hidden construction-time scale convention. Sampson evaluation remains
    // homogeneous/scale-invariant internally.
    const Mat3d essential_ab = NormalizeMatrix(Multiply(SkewSymmetric(t_ba), r_ba));
    const Mat3d essential_ba = Transpose(essential_ab);
    const Mat3d f_ab = Multiply(Transpose(k_b_inv), Multiply(essential_ab, k_a_inv));
    const Mat3d f_ba = Transpose(f_ab);
    if (!EpipolarMatrixNumericallyUsable(essential_ab) || !EpipolarMatrixNumericallyUsable(essential_ba) ||
        !EpipolarMatrixNumericallyUsable(f_ab) || !EpipolarMatrixNumericallyUsable(f_ba)) {
        return Status::Error(StatusCode::InternalError, "Epipolar geometry produced an unusable matrix");
    }
    if (!EpipolarRank2Usable(essential_ab) || !EpipolarRank2Usable(essential_ba) ||
        !EpipolarRank2Usable(f_ab) || !EpipolarRank2Usable(f_ba)) {
        return Status::Error(StatusCode::FailedPrecondition, "Epipolar geometry produced a matrix that does not satisfy rank-2 epipolar structure");
    }

    EpipolarGeometry out;
    out.fundamental_a_to_b = ToMat3f(f_ab);
    out.fundamental_b_to_a = ToMat3f(f_ba);
    out.essential_a_to_b = ToMat3f(essential_ab);
    out.essential_b_to_a = ToMat3f(essential_ba);
    out.diagnostics.camera_a_intrinsics_determinant = ToDiagnosticFloat(k_a_inverse.determinant);
    out.diagnostics.camera_b_intrinsics_determinant = ToDiagnosticFloat(k_b_inverse.determinant);
    out.diagnostics.camera_a_intrinsics_condition = ToDiagnosticFloat(k_a_inverse.condition_estimate);
    out.diagnostics.camera_b_intrinsics_condition = ToDiagnosticFloat(k_b_inverse.condition_estimate);
    out.diagnostics.camera_a_rotation_determinant = ToDiagnosticFloat(r_wa_diagnostics.determinant);
    out.diagnostics.camera_b_rotation_determinant = ToDiagnosticFloat(r_wb_diagnostics.determinant);
    out.diagnostics.relative_rotation_determinant = ToDiagnosticFloat(relative_rotation_determinant);
    out.diagnostics.camera_a_rotation_condition = ToDiagnosticFloat(r_wa_diagnostics.condition_estimate);
    out.diagnostics.camera_b_rotation_condition = ToDiagnosticFloat(r_wb_diagnostics.condition_estimate);
    out.diagnostics.baseline_meters = ToDiagnosticFloat(metric_baseline_meters);
    out.diagnostics.essential_a_to_b_rank_residual = ToDiagnosticFloat(EpipolarRankResidual(essential_ab));
    out.diagnostics.essential_b_to_a_rank_residual = ToDiagnosticFloat(EpipolarRankResidual(essential_ba));
    out.diagnostics.fundamental_a_to_b_rank_residual = ToDiagnosticFloat(EpipolarRankResidual(f_ab));
    out.diagnostics.fundamental_b_to_a_rank_residual = ToDiagnosticFloat(EpipolarRankResidual(f_ba));
    out.valid = true;
    return out;
}

Result<Vec2f> UndistortPixelToNormalized(
    const CameraCalibration& camera,
    Vec2f distorted_pixel) {

    if (!CameraIntrinsicsUsable(camera)) {
        return Status::Error(StatusCode::FailedPrecondition, "UndistortPixelToNormalized requires valid, well-conditioned intrinsics");
    }
    if (!CameraDistortionUsable(camera)) {
        return Status::Error(StatusCode::InvalidArgument, "UndistortPixelToNormalized received non-finite distortion coefficients");
    }
    if (!IsFinite(distorted_pixel)) {
        return Status::Error(StatusCode::InvalidArgument, "UndistortPixelToNormalized received non-finite pixel coordinates");
    }

    const double fx = camera.camera_matrix[0];
    const double fy = camera.camera_matrix[4];
    const double cx = camera.camera_matrix[2];
    const double cy = camera.camera_matrix[5];
    if (std::abs(fx) < kMinFocalPixels || std::abs(fy) < kMinFocalPixels) {
        return Status::Error(StatusCode::FailedPrecondition, "UndistortPixelToNormalized requires finite, well-conditioned focal lengths");
    }

    const double xd = (static_cast<double>(distorted_pixel.x) - cx) / fx;
    const double yd = (static_cast<double>(distorted_pixel.y) - cy) / fy;
    if (!IsFinite(xd) || !IsFinite(yd)) {
        return Status::Error(StatusCode::InvalidArgument, "UndistortPixelToNormalized produced non-finite normalized coordinates");
    }

    const double k1 = camera.distortion[0];
    const double k2 = camera.distortion[1];
    const double p1 = camera.distortion[2];
    const double p2 = camera.distortion[3];
    const double k3 = camera.distortion[4];

    double x = xd;
    double y = yd;
    for (int i = 0; i < kUndistortIterations; ++i) {
        const double r2 = x * x + y * y;
        const double r4 = r2 * r2;
        const double r6 = r4 * r2;
        const double radial = 1.0 + k1 * r2 + k2 * r4 + k3 * r6;
        if (!IsFinite(radial) || std::abs(radial) <= kMinDistortionRadial) {
            return Status::Error(StatusCode::FailedPrecondition, "UndistortPixelToNormalized encountered degenerate radial distortion");
        }
        const double tangential_x = 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
        const double tangential_y = p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;
        const double next_x = (xd - tangential_x) / radial;
        const double next_y = (yd - tangential_y) / radial;
        if (!IsFinite(next_x) || !IsFinite(next_y)) {
            return Status::Error(StatusCode::InvalidArgument, "UndistortPixelToNormalized iteration produced non-finite coordinates");
        }
        x = next_x;
        y = next_y;
    }

    return Vec2f{static_cast<float>(x), static_cast<float>(y)};
}

EpipolarCheck ComputePixelSampsonEpipolarCheck(
    const EpipolarGeometry& geometry,
    Vec2f pixel_point_a,
    Vec2f pixel_point_b,
    float soft_threshold_px,
    float hard_threshold_px) {

    constexpr EpipolarCoordinateSpace kSpace = EpipolarCoordinateSpace::PixelFundamental;
    if (!geometry.valid) {
        return MakeFailedCheck(kSpace, EpipolarCheckReason::InvalidGeometry);
    }
    const SampsonError error = ComputeSampsonErrorFromMatrix(
        ToMat3d(geometry.fundamental_a_to_b),
        pixel_point_a,
        pixel_point_b);
    return BuildPixelThresholdedCheck(
        error,
        kSpace,
        error.value,
        error.value,
        error.value,
        0.0f,
        soft_threshold_px,
        hard_threshold_px);
}

EpipolarCheck ComputeNormalizedSampsonEpipolarCheck(
    const EpipolarGeometry& geometry,
    Vec2f normalized_point_a,
    Vec2f normalized_point_b,
    float soft_threshold_normalized,
    float hard_threshold_normalized) {

    constexpr EpipolarCoordinateSpace kSpace = EpipolarCoordinateSpace::NormalizedEssential;
    if (!geometry.valid) {
        return MakeFailedCheck(kSpace, EpipolarCheckReason::InvalidGeometry);
    }
    const SampsonError error = ComputeSampsonErrorFromMatrix(
        ToMat3d(geometry.essential_a_to_b),
        normalized_point_a,
        normalized_point_b);
    return BuildNormalizedThresholdedCheck(
        error,
        soft_threshold_normalized,
        hard_threshold_normalized);
}

EpipolarCheck ComputeDistortionSafePixelSampsonEpipolarCheck(
    const EpipolarGeometry& geometry,
    const CameraCalibration& camera_a,
    const CameraCalibration& camera_b,
    Vec2f distorted_pixel_a,
    Vec2f distorted_pixel_b,
    float soft_threshold_px,
    float hard_threshold_px) {

    constexpr EpipolarCoordinateSpace kSpace = EpipolarCoordinateSpace::NormalizedEssential;
    if (!geometry.valid) {
        return MakeFailedCheck(kSpace, EpipolarCheckReason::InvalidGeometry);
    }
    if (!ThresholdsUsable(soft_threshold_px, hard_threshold_px)) {
        return MakeFailedCheck(kSpace, EpipolarCheckReason::InvalidThreshold);
    }

    const auto normalized_a = UndistortPixelToNormalized(camera_a, distorted_pixel_a);
    const auto normalized_b = UndistortPixelToNormalized(camera_b, distorted_pixel_b);
    if (!normalized_a.ok() || !normalized_b.ok()) {
        return MakeFailedCheck(
            kSpace,
            !IsFinite(distorted_pixel_a) || !IsFinite(distorted_pixel_b)
                ? EpipolarCheckReason::NonFinitePoint
                : EpipolarCheckReason::InvalidCamera);
    }

    const Mat3d essential = ToMat3d(geometry.essential_a_to_b);
    const SampsonError error = ComputeSampsonErrorFromMatrix(
        essential,
        normalized_a.value(),
        normalized_b.value());
    const SampsonError anisotropic_error = ComputeAnisotropicPixelSampsonErrorFromNormalizedMatrix(
        essential,
        normalized_a.value(),
        normalized_b.value(),
        camera_a,
        camera_b);
    if (error.valid && !anisotropic_error.valid) {
        return MakeFailedCheck(kSpace, anisotropic_error.reason);
    }

    const float isotropic_pixel_heuristic = error.value * IsotropicFocalHeuristicPixels(camera_a, camera_b);
    const float anisotropic_pixel_metric = anisotropic_error.valid ? anisotropic_error.value : 0.0f;
    return BuildPixelThresholdedCheck(
        error,
        kSpace,
        isotropic_pixel_heuristic,
        isotropic_pixel_heuristic,
        anisotropic_pixel_metric,
        error.value,
        soft_threshold_px,
        hard_threshold_px);
}

} // namespace bt
