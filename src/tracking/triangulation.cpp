#include "tracking/triangulation.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

namespace bt {
namespace {

constexpr float kMinHomogeneousW = 1e-5f;
constexpr float kMinPositiveDepth = 1e-5f;
constexpr double kMinDltSingularValue = 1.0e-12;
constexpr double kMinDltHomogeneousW = 1.0e-9;

bool IsFinite(float value) {
    return std::isfinite(value);
}

bool IsFinite(const Vec2f& value) {
    return IsFinite(value.x) && IsFinite(value.y);
}

bool IsFinite(const Mat34f& value) {
    for (const float entry : value.m) {
        if (!IsFinite(entry)) {
            return false;
        }
    }
    return true;
}

bool IsFiniteDouble(double value) {
    return std::isfinite(value);
}

struct SymmetricEigen4 {
    double values[4]{};
    double vectors[4][4]{}; // Column-major eigenvectors: vectors[row][column].
    bool valid = false;
};

void SortEigenpairsAscending(SymmetricEigen4& eigen) {
    for (int i = 0; i < 3; ++i) {
        int best = i;
        for (int j = i + 1; j < 4; ++j) {
            if (eigen.values[j] < eigen.values[best]) {
                best = j;
            }
        }
        if (best == i) {
            continue;
        }
        std::swap(eigen.values[i], eigen.values[best]);
        for (int row = 0; row < 4; ++row) {
            std::swap(eigen.vectors[row][i], eigen.vectors[row][best]);
        }
    }
}

SymmetricEigen4 JacobiEigenSymmetric4(const double input[4][4]) {
    SymmetricEigen4 out;
    double a[4][4]{};
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            if (!IsFiniteDouble(input[r][c])) {
                return out;
            }
            a[r][c] = input[r][c];
            out.vectors[r][c] = (r == c) ? 1.0 : 0.0;
        }
    }

    for (int sweep = 0; sweep < 64; ++sweep) {
        int p = 0;
        int q = 1;
        double max_offdiag = std::abs(a[p][q]);
        double max_diag = 0.0;
        for (int r = 0; r < 4; ++r) {
            max_diag = std::max(max_diag, std::abs(a[r][r]));
            for (int c = r + 1; c < 4; ++c) {
                const double offdiag = std::abs(a[r][c]);
                if (offdiag > max_offdiag) {
                    max_offdiag = offdiag;
                    p = r;
                    q = c;
                }
            }
        }

        if (max_offdiag <= std::max(1.0e-14, 1.0e-14 * max_diag)) {
            break;
        }

        const double app = a[p][p];
        const double aqq = a[q][q];
        const double apq = a[p][q];
        if (std::abs(apq) <= 0.0) {
            continue;
        }

        const double tau = (aqq - app) / (2.0 * apq);
        const double sign = tau >= 0.0 ? 1.0 : -1.0;
        const double t = sign / (std::abs(tau) + std::sqrt(1.0 + tau * tau));
        const double c = 1.0 / std::sqrt(1.0 + t * t);
        const double s = t * c;

        for (int k = 0; k < 4; ++k) {
            if (k == p || k == q) {
                continue;
            }
            const double akp = a[k][p];
            const double akq = a[k][q];
            a[k][p] = a[p][k] = c * akp - s * akq;
            a[k][q] = a[q][k] = s * akp + c * akq;
        }

        a[p][p] = c * c * app - 2.0 * s * c * apq + s * s * aqq;
        a[q][q] = s * s * app + 2.0 * s * c * apq + c * c * aqq;
        a[p][q] = a[q][p] = 0.0;

        for (int k = 0; k < 4; ++k) {
            const double vkp = out.vectors[k][p];
            const double vkq = out.vectors[k][q];
            out.vectors[k][p] = c * vkp - s * vkq;
            out.vectors[k][q] = s * vkp + c * vkq;
        }
    }

    for (int i = 0; i < 4; ++i) {
        if (!IsFiniteDouble(a[i][i])) {
            return out;
        }
        out.values[i] = a[i][i];
    }
    SortEigenpairsAscending(out);
    out.valid = true;
    return out;
}

struct HomogeneousDltSolution {
    Vec3f world{};
    float condition_number = 0.0f;
    float strength_ratio = 0.0f;
    float null_residual = 0.0f;
    TriangulationFailure failure = TriangulationFailure::DltNullspaceSolveFailed;
    const char* failure_message = "Triangulation DLT nullspace solve failed";
    bool ill_conditioned = false;
    bool valid = false;
};

bool NormalizeEquationRow(double row[4]) {
    double norm_sq = 0.0;
    for (int i = 0; i < 4; ++i) {
        if (!IsFiniteDouble(row[i])) {
            return false;
        }
        norm_sq += row[i] * row[i];
    }
    if (!IsFiniteDouble(norm_sq) || norm_sq <= 1.0e-30) {
        return false;
    }
    const double inv_norm = 1.0 / std::sqrt(norm_sq);
    for (int i = 0; i < 4; ++i) {
        row[i] *= inv_norm;
    }
    return true;
}

void BuildDltRowsForView(
    const Mat34f& projection,
    Vec2f point,
    double row_u[4],
    double row_v[4]) {

    // Single-correspondence triangulation cannot use true multi-point Hartley
    // centroid normalization. The numerical guard used here is double-precision
    // row-norm equilibration after building the textbook homogeneous DLT rows;
    // this prevents raw pixel/projection magnitude from dominating A^T A while
    // keeping the threshold semantics tied to the equilibrated linear system.
    const auto& p = projection.m;
    for (int i = 0; i < 4; ++i) {
        row_u[i] = static_cast<double>(point.x) * static_cast<double>(p[8 + i]) -
                   static_cast<double>(p[i]);
        row_v[i] = static_cast<double>(point.y) * static_cast<double>(p[8 + i]) -
                   static_cast<double>(p[4 + i]);
    }
}

void ScaleEquationRow(double row[4], double weight) {
    const double safe_weight = std::max(weight, 1.0e-12);
    for (int i = 0; i < 4; ++i) {
        row[i] *= safe_weight;
    }
}

HomogeneousDltSolution SolveHomogeneousDltBySymmetricEigen(const double rows[4][4], const StereoTriangulationConfig& config) {
    HomogeneousDltSolution out;
    double normal[4][4]{};
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            double sum = 0.0;
            for (int row = 0; row < 4; ++row) {
                if (!IsFiniteDouble(rows[row][r]) || !IsFiniteDouble(rows[row][c])) {
                    out.failure = TriangulationFailure::DltRowsNonFinite;
                    out.failure_message = "Triangulation DLT rows contain non-finite values";
                    return out;
                }
                sum += rows[row][r] * rows[row][c];
            }
            normal[r][c] = sum;
        }
    }

    const SymmetricEigen4 eigen = JacobiEigenSymmetric4(normal);
    if (!eigen.valid) {
        out.failure = TriangulationFailure::DltEigensolveFailed;
        out.failure_message = "Triangulation DLT eigensolve failed";
        return out;
    }

    const double lambda0 = std::max(0.0, eigen.values[0]);
    const double lambda1 = std::max(0.0, eigen.values[1]);
    const double lambda3 = std::max(0.0, eigen.values[3]);
    const double sigma0 = std::sqrt(lambda0);
    const double sigma1 = std::sqrt(lambda1);
    const double sigma3 = std::sqrt(lambda3);
    if (!IsFiniteDouble(sigma0) || !IsFiniteDouble(sigma1) || !IsFiniteDouble(sigma3) ||
        sigma1 <= kMinDltSingularValue || sigma3 <= kMinDltSingularValue) {
        out.failure = TriangulationFailure::DegenerateSingularSpectrum;
        out.failure_message = "Triangulation DLT singular spectrum is degenerate";
        return out;
    }

    const double w = eigen.vectors[3][0];
    if (!IsFiniteDouble(w) || std::abs(w) <= kMinDltHomogeneousW) {
        out.failure = TriangulationFailure::PointAtInfinity;
        out.failure_message = "Triangulated homogeneous point is at infinity";
        return out;
    }

    out.world = Vec3f{
        static_cast<float>(eigen.vectors[0][0] / w),
        static_cast<float>(eigen.vectors[1][0] / w),
        static_cast<float>(eigen.vectors[2][0] / w)};
    if (!IsFinite(out.world)) {
        out.failure = TriangulationFailure::NonFiniteWorldPoint;
        out.failure_message = "Triangulated world point is non-finite";
        return out;
    }

    const double condition = sigma3 / sigma1;
    const double strength = sigma1 / sigma3;
    if (!IsFiniteDouble(condition) || !IsFiniteDouble(strength)) {
        out.failure = TriangulationFailure::NonFiniteConditionDiagnostics;
        out.failure_message = "Triangulation DLT condition diagnostics are non-finite";
        return out;
    }
    out.condition_number = static_cast<float>(condition);
    out.strength_ratio = static_cast<float>(strength);
    out.null_residual = static_cast<float>(sigma0);
    out.ill_conditioned = condition > config.max_dlt_condition_number || strength < config.min_dlt_strength_ratio;
    out.valid = true;
    return out;
}


float CameraDepth(const Mat34f& p, const Vec3f& x) {
    return p.m[8] * x.x + p.m[9] * x.y + p.m[10] * x.z + p.m[11];
}

Vec3f TransformPoint(const Mat34f& transform, const Vec3f& point) {
    return Vec3f{
        transform.m[0] * point.x + transform.m[1] * point.y + transform.m[2] * point.z + transform.m[3],
        transform.m[4] * point.x + transform.m[5] * point.y + transform.m[6] * point.z + transform.m[7],
        transform.m[8] * point.x + transform.m[9] * point.y + transform.m[10] * point.z + transform.m[11]
    };
}

float Clamp(float value, float lo, float hi) {
    if (!std::isfinite(value)) {
        return lo;
    }
    return std::max(lo, std::min(hi, value));
}

float Clamp01(float value) {
    return Clamp(value, 0.0f, 1.0f);
}

bool IntrinsicsUsable(const std::array<double, 9>& k) {
    return
        std::isfinite(k[0]) && std::isfinite(k[4]) &&
        std::isfinite(k[2]) && std::isfinite(k[5]) &&
        std::abs(k[0]) > 1e-5 && std::abs(k[4]) > 1e-5;
}

bool CameraModelUsable(const StereoCameraModel& camera) {
    return camera.projection_valid &&
        IsFinite(camera.image_from_world) &&
        IsFinite(camera.world_from_camera) &&
        IntrinsicsUsable(camera.camera_matrix);
}

constexpr float kRadiansToDegrees = 57.29577951308232f;

struct WorldRay {
    Vec3f origin{};
    Vec3f direction{};
    bool valid = false;
};

Vec3f RotateCameraVectorToWorld(const Mat34f& world_from_camera, const Vec3f& camera_vector) {
    return Vec3f{
        world_from_camera.m[0] * camera_vector.x + world_from_camera.m[1] * camera_vector.y + world_from_camera.m[2] * camera_vector.z,
        world_from_camera.m[4] * camera_vector.x + world_from_camera.m[5] * camera_vector.y + world_from_camera.m[6] * camera_vector.z,
        world_from_camera.m[8] * camera_vector.x + world_from_camera.m[9] * camera_vector.y + world_from_camera.m[10] * camera_vector.z
    };
}

WorldRay PixelRayInWorld(const StereoCameraModel& camera, Vec2f pixel) {
    WorldRay ray;
    if (!CameraModelUsable(camera) || !IsFinite(pixel)) {
        return ray;
    }

    const float fx = static_cast<float>(camera.camera_matrix[0]);
    const float fy = static_cast<float>(camera.camera_matrix[4]);
    const float cx = static_cast<float>(camera.camera_matrix[2]);
    const float cy = static_cast<float>(camera.camera_matrix[5]);
    if (!std::isfinite(fx) || !std::isfinite(fy) || std::abs(fx) <= 1e-5f || std::abs(fy) <= 1e-5f) {
        return ray;
    }

    const Vec3f camera_direction = Normalize(Vec3f{
        (pixel.x - cx) / fx,
        (pixel.y - cy) / fy,
        1.0f
    });
    const Vec3f world_direction = Normalize(RotateCameraVectorToWorld(camera.world_from_camera, camera_direction));
    const Vec3f origin = TransformPoint(camera.world_from_camera, Vec3f{0.0f, 0.0f, 0.0f});
    if (!IsFinite(origin) || !IsFinite(world_direction) || Length(world_direction) <= 1e-5f) {
        return ray;
    }

    ray.origin = origin;
    ray.direction = world_direction;
    ray.valid = true;
    return ray;
}

struct RayPairCheck {
    float closest_distance_m = 0.0f;
    float angle_deg = 0.0f;
    bool valid = false;
    bool usable_for_rejection = false;
};

RayPairCheck CheckRayPair(
    const StereoCameraModel& camera_a,
    const StereoCameraModel& camera_b,
    Vec2f point_a,
    Vec2f point_b,
    const StereoTriangulationConfig& config) {
    RayPairCheck out;
    const WorldRay a = PixelRayInWorld(camera_a, point_a);
    const WorldRay b = PixelRayInWorld(camera_b, point_b);
    if (!a.valid || !b.valid) {
        return out;
    }

    const float dot = Clamp(Dot(a.direction, b.direction), -1.0f, 1.0f);
    out.angle_deg = std::acos(std::abs(dot)) * kRadiansToDegrees;

    const Vec3f w0 = Sub(a.origin, b.origin);
    const float aa = Dot(a.direction, a.direction);
    const float bb = Dot(a.direction, b.direction);
    const float cc = Dot(b.direction, b.direction);
    const float dd = Dot(a.direction, w0);
    const float ee = Dot(b.direction, w0);
    const float denom = aa * cc - bb * bb;

    if (!std::isfinite(denom) || std::abs(denom) <= 1e-6f) {
        out.valid = true;
        out.closest_distance_m = Length(Cross(w0, a.direction));
        out.usable_for_rejection = false;
        return out;
    }

    const float s = (bb * ee - cc * dd) / denom;
    const float t = (aa * ee - bb * dd) / denom;
    const Vec3f pa = Add(a.origin, Scale(a.direction, s));
    const Vec3f pb = Add(b.origin, Scale(b.direction, t));
    out.closest_distance_m = Distance(pa, pb);
    out.valid = std::isfinite(out.closest_distance_m) && std::isfinite(out.angle_deg);
    out.usable_for_rejection = out.valid && s > 0.0f && t > 0.0f && out.angle_deg >= config.min_ray_angle_deg;
    return out;
}

Result<Vec2f> Project(const Mat34f& p, const Vec3f& x) {
    const float u = p.m[0] * x.x + p.m[1] * x.y + p.m[2] * x.z + p.m[3];
    const float v = p.m[4] * x.x + p.m[5] * x.y + p.m[6] * x.z + p.m[7];
    const float w = CameraDepth(p, x);
    if (!IsFinite(u) || !IsFinite(v) || !IsFinite(w)) {
        return Status::Error(StatusCode::ValidationError, "Projection produced non-finite coordinates");
    }
    if (std::abs(w) < kMinHomogeneousW) {
        return Status::Error(StatusCode::ValidationError, "Projection is too close to the camera plane");
    }
    return Vec2f{u / w, v / w};
}

} // namespace

float StereoReprojectionConfidence(float reprojection_error_a_px, float reprojection_error_b_px) noexcept {
    if (!std::isfinite(reprojection_error_a_px) || !std::isfinite(reprojection_error_b_px) ||
        reprojection_error_a_px < 0.0f || reprojection_error_b_px < 0.0f) {
        return 0.0f;
    }

    // A triangulated lower-body point with several pixels of residual may still
    // reproject acceptably in both images while landing metres away in world
    // space under stereo disagreement or calibration bias. Make that residual
    // a first-class confidence term before the point can influence support.
    const float total_error = reprojection_error_a_px + reprojection_error_b_px;
    return 1.0f / (1.0f + total_error);
}

float CameraObservationQuality(const StereoCameraObservation& observation) noexcept {
    if (!observation.present ||
        !std::isfinite(observation.pixel.x) ||
        !std::isfinite(observation.pixel.y)) {
        return 0.0f;
    }
    const float confidence = Clamp01(observation.keypoint_confidence);
    const float reliability = Clamp01(observation.reliability_weight);
    const float age = std::isfinite(observation.age_scale) ? Clamp01(observation.age_scale) : 1.0f;
    return Clamp01(std::sqrt(confidence) * reliability * age);
}

float ProjectionDepth(const Mat34f& projection, const Vec3f& world) noexcept {
    const float depth = CameraDepth(projection, world);
    return std::isfinite(depth) ? depth : 0.0f;
}

Result<Vec3f> BackProjectPixelAtCameraDepth(
    const StereoCameraModel& camera,
    Vec2f pixel,
    float camera_depth_m) {

    if (!CameraModelUsable(camera) ||
        !std::isfinite(pixel.x) ||
        !std::isfinite(pixel.y) ||
        !std::isfinite(camera_depth_m) ||
        camera_depth_m <= kMinPositiveDepth) {
        return Status::Error(StatusCode::ValidationError, "Camera-only backprojection input is invalid");
    }

    const float fx = static_cast<float>(camera.camera_matrix[0]);
    const float fy = static_cast<float>(camera.camera_matrix[4]);
    const float cx = static_cast<float>(camera.camera_matrix[2]);
    const float cy = static_cast<float>(camera.camera_matrix[5]);
    const Vec3f camera_point{
        (pixel.x - cx) * camera_depth_m / fx,
        (pixel.y - cy) * camera_depth_m / fy,
        camera_depth_m
    };
    const Vec3f world = TransformPoint(camera.world_from_camera, camera_point);
    if (!IsFinite(world)) {
        return Status::Error(StatusCode::ValidationError, "Camera-only backprojection produced non-finite world point");
    }
    return world;
}

namespace {

float EffectiveFocalPx(const StereoCameraModel& camera) {
    if (!IntrinsicsUsable(camera.camera_matrix)) {
        return 0.0f;
    }
    const double fx = std::abs(camera.camera_matrix[0]);
    const double fy = std::abs(camera.camera_matrix[4]);
    const double focal = std::sqrt(fx * fy);
    return std::isfinite(focal) ? static_cast<float>(focal) : 0.0f;
}

Vec3f CameraOriginWorld(const StereoCameraModel& camera) {
    return TransformPoint(camera.world_from_camera, Vec3f{0.0f, 0.0f, 0.0f});
}

float SafePositive(float value, float fallback) {
    return std::isfinite(value) && value > 0.0f ? value : fallback;
}

} // namespace

StereoMeasurementUncertainty EstimateStereoMeasurementUncertainty(
    const StereoCameraModel& camera_a,
    const StereoCameraModel& camera_b,
    const TriangulatedPoint& triangulated,
    float epipolar_error_px,
    bool epipolar_checked,
    const StereoMeasurementUncertaintyConfig& config) noexcept {

    StereoMeasurementUncertainty out;
    if (!triangulated.valid ||
        !CameraModelUsable(camera_a) ||
        !CameraModelUsable(camera_b) ||
        !IsFinite(triangulated.world) ||
        triangulated.triangulation_ill_conditioned) {
        return out;
    }

    const Vec3f origin_a = CameraOriginWorld(camera_a);
    const Vec3f origin_b = CameraOriginWorld(camera_b);
    const float baseline_m = Distance(origin_a, origin_b);
    const float depth_a = std::abs(ProjectionDepth(camera_a.image_from_world, triangulated.world));
    const float depth_b = std::abs(ProjectionDepth(camera_b.image_from_world, triangulated.world));
    const float mean_depth_m = 0.5f * (depth_a + depth_b);
    const float focal_a_px = EffectiveFocalPx(camera_a);
    const float focal_b_px = EffectiveFocalPx(camera_b);
    const float effective_focal_px = 0.5f * (focal_a_px + focal_b_px);
    if (!std::isfinite(baseline_m) || baseline_m <= kMinPositiveDepth ||
        !std::isfinite(mean_depth_m) || mean_depth_m <= kMinPositiveDepth ||
        !std::isfinite(effective_focal_px) || effective_focal_px <= kMinPositiveDepth) {
        return out;
    }

    const float reprojection_sigma_px = std::sqrt(
        0.5f * (triangulated.reprojection_error_a * triangulated.reprojection_error_a +
                triangulated.reprojection_error_b * triangulated.reprojection_error_b));
    const float safe_reprojection_sigma_px = SafePositive(reprojection_sigma_px, 0.0f);
    const float safe_epipolar_sigma_px = epipolar_checked ? SafePositive(epipolar_error_px, 0.0f) : 0.0f;

    // Residuals are observations. This maps them into a conservative image-plane
    // noise model with an irreducible floor; it is not claiming the residual itself
    // is a covariance. Reprojection is the primary image-plane residual. Epipolar
    // error is only allowed to add the part not already covered by reprojection, so
    // the same geometric inconsistency is not counted twice as independent variance.
    const float epipolar_excess_sigma_px =
        epipolar_checked ? std::max(0.0f, safe_epipolar_sigma_px - safe_reprojection_sigma_px) : 0.0f;
    const float image_noise_sigma_px = std::sqrt(
        config.min_image_noise_sigma_px * config.min_image_noise_sigma_px +
        safe_reprojection_sigma_px * safe_reprojection_sigma_px +
        epipolar_excess_sigma_px * epipolar_excess_sigma_px);

    const float condition_number = SafePositive(triangulated.dlt_condition_number, 1.0f);
    const float strength_ratio = SafePositive(triangulated.dlt_strength_ratio, 1.0f);
    const float strength_as_condition = 1.0f / strength_ratio;
    const float geometry_condition = std::max(condition_number, strength_as_condition);
    // The DLT condition number is a worst-case relative-error bound, not an
    // expected uncertainty multiplier. Use slow logarithmic growth with saturation.
    // dlt_strength_ratio is the reciprocal diagnostic, so it contributes through
    // the same effective condition estimate rather than as an independent factor.
    const float conditioning_scale = std::min(
        config.max_conditioning_scale,
        std::max(1.0f, std::log(std::max(1.0f, geometry_condition))));

    const float unclamped_depth_stddev_m = std::max(
        config.min_depth_stddev_m,
        (mean_depth_m * mean_depth_m * image_noise_sigma_px * conditioning_scale) /
            std::max(kMinPositiveDepth, baseline_m * effective_focal_px));
    const float unclamped_lateral_stddev_m = std::max(
        config.min_lateral_stddev_m,
        (mean_depth_m * image_noise_sigma_px * std::sqrt(conditioning_scale)) /
            std::max(kMinPositiveDepth, effective_focal_px));
    const float unclamped_position_variance_m2 =
        2.0f * unclamped_lateral_stddev_m * unclamped_lateral_stddev_m +
        unclamped_depth_stddev_m * unclamped_depth_stddev_m;
    if (!std::isfinite(unclamped_position_variance_m2) || unclamped_position_variance_m2 <= 0.0f) {
        return out;
    }

    // Reporting fields are clamped to keep telemetry bounded. Solver weighting
    // consumes the unclamped component stddevs so max-clamped observations do not
    // stagnate at identical weights.
    const float clamped_depth_stddev_m = std::min(unclamped_depth_stddev_m, config.max_reported_position_stddev_m);
    const float clamped_lateral_stddev_m = std::min(unclamped_lateral_stddev_m, config.max_reported_position_stddev_m);
    const float position_variance_m2 =
        2.0f * clamped_lateral_stddev_m * clamped_lateral_stddev_m +
        clamped_depth_stddev_m * clamped_depth_stddev_m;

    out.valid = true;
    out.baseline_m = baseline_m;
    out.mean_depth_m = mean_depth_m;
    out.baseline_to_depth_ratio = baseline_m / mean_depth_m;
    out.effective_focal_px = effective_focal_px;
    out.reprojection_sigma_px = safe_reprojection_sigma_px;
    out.epipolar_sigma_px = safe_epipolar_sigma_px;
    out.image_noise_sigma_px = image_noise_sigma_px;
    out.conditioning_scale = conditioning_scale;
    out.unclamped_lateral_stddev_m = unclamped_lateral_stddev_m;
    out.unclamped_depth_stddev_m = unclamped_depth_stddev_m;
    out.unclamped_position_variance_m2 = unclamped_position_variance_m2;
    out.lateral_stddev_m = clamped_lateral_stddev_m;
    out.depth_stddev_m = clamped_depth_stddev_m;
    out.position_variance_m2 = position_variance_m2;
    out.position_stddev_m = std::sqrt(position_variance_m2);
    return out;
}

StereoJointEvidence ResolveStereoJointEvidence(
    const StereoCameraModel& camera_a,
    const StereoCameraModel& camera_b,
    const StereoCameraObservation& observation_a,
    const StereoCameraObservation& observation_b,
    const StereoTemporalReference& temporal,
    const StereoEpipolarContext* epipolar,
    const StereoJointEvidenceConfig& config) {

    StereoJointEvidence out;
    out.camera_a_quality = CameraObservationQuality(observation_a);
    out.camera_b_quality = CameraObservationQuality(observation_b);
    out.temporal_confidence = Clamp01(temporal.confidence);

    const bool good_a = out.camera_a_quality >= config.triangulation.min_single_camera_quality;
    const bool good_b = out.camera_b_quality >= config.triangulation.min_single_camera_quality;
    const bool epipolar_available = epipolar && epipolar->valid();
    bool epipolar_blocks_triangulation = false;
    float epipolar_reliability_term = 1.0f;

    if (good_a && good_b && epipolar_available) {
        out.epipolar_available = true;
        const auto check = ComputeDistortionSafePixelSampsonEpipolarCheck(
            *epipolar->geometry,
            *epipolar->camera_a,
            *epipolar->camera_b,
            observation_a.pixel,
            observation_b.pixel,
            epipolar->config.soft_threshold_px,
            epipolar->config.hard_threshold_px);
        out.epipolar_reason = check.reason;
        out.epipolar_coordinate_space = check.coordinate_space;
        out.epipolar_error_px = check.sampson_error_px;
        out.epipolar_error_px_isotropic = check.sampson_error_px_isotropic;
        out.epipolar_error_px_anisotropic = check.sampson_error_px_anisotropic;
        out.epipolar_error_normalized = check.sampson_error_normalized;
        out.epipolar_confidence = check.valid ? check.confidence : 0.0f;
        out.epipolar_checked = check.valid;
        out.epipolar_hard_mismatch = check.valid && check.hard_mismatch;
        if (check.valid) {
            epipolar_reliability_term = std::max(epipolar->config.min_confidence_floor, check.confidence);
            out.epipolar_reliability_term = epipolar_reliability_term;
            const bool degraded_pair = epipolar->pair_degraded ||
                epipolar->reused_camera_a ||
                epipolar->reused_camera_b ||
                epipolar->duplicate_pair ||
                epipolar->timestamp_skewed;
            if (check.hard_mismatch) {
                out.epipolar_degraded_pair_softened = degraded_pair &&
                    !epipolar->config.hard_mismatch_rejects_degraded_pair;
                epipolar_blocks_triangulation = degraded_pair
                    ? epipolar->config.hard_mismatch_rejects_degraded_pair
                    : epipolar->config.hard_mismatch_rejects_fresh_pair;
                out.epipolar_pair_rejected = epipolar_blocks_triangulation;
            }
        }
        // Invalid or degenerate epipolar checks mean "do not apply this term".
        // They are not hard mismatches and they must not erase otherwise useful
        // stereo or single-camera evidence.
    }

    if (good_a && good_b && camera_a.projection_valid && camera_b.projection_valid && !epipolar_blocks_triangulation) {
        const RayPairCheck ray_check = CheckRayPair(camera_a, camera_b, observation_a.pixel, observation_b.pixel, config.triangulation);
        if (ray_check.usable_for_rejection && ray_check.closest_distance_m > config.triangulation.max_ray_closest_distance_m) {
            out.rejected = true;
            out.source = JointEvidenceSource::Rejected;
            out.world = temporal.world;
            return out;
        }

        const auto tri = TriangulateLinearDLT(
            camera_a.image_from_world,
            camera_b.image_from_world,
            observation_a.pixel,
            observation_b.pixel,
            out.camera_a_quality,
            out.camera_b_quality,
            config.triangulation);
        if (tri.ok() && tri.value().valid) {
            const float reprojection_confidence = StereoReprojectionConfidence(
                tri.value().reprojection_error_a,
                tri.value().reprojection_error_b);
            if (reprojection_confidence >= config.triangulation.min_stereo_reprojection_confidence) {
                out.world = tri.value().world;
                out.source = JointEvidenceSource::Stereo;
                out.confidence = Clamp01(
                    std::sqrt(out.camera_a_quality * out.camera_b_quality) *
                    reprojection_confidence);
                out.reprojection_error_a = tri.value().reprojection_error_a;
                out.reprojection_error_b = tri.value().reprojection_error_b;
                out.mean_reprojection_error = 0.5f * (out.reprojection_error_a + out.reprojection_error_b);
                out.triangulation_condition_number = tri.value().dlt_condition_number;
                out.triangulation_strength_ratio = tri.value().dlt_strength_ratio;
                out.triangulation_null_residual = tri.value().dlt_null_residual;
                out.triangulation_ill_conditioned = tri.value().triangulation_ill_conditioned;
                const auto uncertainty = EstimateStereoMeasurementUncertainty(
                    camera_a,
                    camera_b,
                    tri.value(),
                    out.epipolar_error_px_anisotropic > 0.0f ? out.epipolar_error_px_anisotropic : out.epipolar_error_px,
                    out.epipolar_checked,
                    config.uncertainty);
                out.measurement_uncertainty_valid = uncertainty.valid;
                out.measurement_baseline_m = uncertainty.baseline_m;
                out.measurement_mean_depth_m = uncertainty.mean_depth_m;
                out.measurement_baseline_to_depth_ratio = uncertainty.baseline_to_depth_ratio;
                out.measurement_effective_focal_px = uncertainty.effective_focal_px;
                out.measurement_reprojection_sigma_px = uncertainty.reprojection_sigma_px;
                out.measurement_epipolar_sigma_px = uncertainty.epipolar_sigma_px;
                out.measurement_image_noise_sigma_px = uncertainty.image_noise_sigma_px;
                out.measurement_conditioning_scale = uncertainty.conditioning_scale;
                out.measurement_unclamped_lateral_stddev_m = uncertainty.unclamped_lateral_stddev_m;
                out.measurement_unclamped_depth_stddev_m = uncertainty.unclamped_depth_stddev_m;
                out.measurement_lateral_stddev_m = uncertainty.lateral_stddev_m;
                out.measurement_depth_stddev_m = uncertainty.depth_stddev_m;
                out.measurement_position_stddev_m = uncertainty.position_stddev_m;
                out.measurement_position_variance_m2 = uncertainty.position_variance_m2;
                out.triangulated = true;
                out.valid = out.confidence > 0.0f;
                return out;
            }
        } else if (!tri.ok() && tri.status().is_ill_conditioned()) {
            out.triangulation_ill_conditioned = true;
        }
    }

    const bool prefer_b = out.camera_b_quality > out.camera_a_quality;
    const bool selected_b = prefer_b && good_b;
    const bool selected_a = !selected_b && good_a;
    if ((selected_a || selected_b) && temporal.valid && out.temporal_confidence > 0.0f) {
        const StereoCameraModel& selected_camera = selected_b ? camera_b : camera_a;
        const StereoCameraObservation& selected_observation = selected_b ? observation_b : observation_a;
        const float selected_quality = selected_b ? out.camera_b_quality : out.camera_a_quality;
        const float depth = ProjectionDepth(selected_camera.image_from_world, temporal.world);
        const auto world = BackProjectPixelAtCameraDepth(selected_camera, selected_observation.pixel, depth);
        if (world.ok()) {
            out.world = world.value();
            out.source = selected_b ? JointEvidenceSource::CameraBOnly : JointEvidenceSource::CameraAOnly;
            out.confidence = Clamp01(selected_quality * out.temporal_confidence * config.triangulation.single_camera_depth_confidence_scale);
            out.estimated_depth_m = depth;
            out.depth_inferred = true;
            out.temporal_depth_used = true;
            out.fallback_used = true;
            out.valid = out.confidence > 0.0f;
            return out;
        }
    }

    out.rejected = observation_a.present || observation_b.present;
    out.source = out.rejected ? JointEvidenceSource::Rejected :
        (temporal.valid && out.temporal_confidence > 0.0f ? JointEvidenceSource::TemporalHold : JointEvidenceSource::None);
    out.world = temporal.world;
    out.confidence = 0.0f;
    return out;
}


float StereoPairEpipolarReliabilityScale(const StereoJointEvidence& evidence) noexcept {
    if (!evidence.triangulated ||
        !evidence.epipolar_checked ||
        !std::isfinite(evidence.epipolar_reliability_term) ||
        evidence.epipolar_reliability_term <= 0.0f) {
        return 1.0f;
    }
    return Clamp01(evidence.epipolar_reliability_term);
}

float StereoSeedConfidence(
    const StereoJointEvidence& evidence,
    float foot_separation_scale,
    float geometry_scale) noexcept {

    return Clamp01(
        evidence.confidence *
        StereoPairEpipolarReliabilityScale(evidence) *
        foot_separation_scale *
        geometry_scale);
}

TriangulationResult TriangulateLinearDLT(
    const Mat34f& pa,
    const Mat34f& pb,
    Vec2f a,
    Vec2f b,
    float weight_a,
    float weight_b,
    const StereoTriangulationConfig& config) {
    if (!IsFinite(pa) || !IsFinite(pb) || !IsFinite(a) || !IsFinite(b)) {
        return TriangulationStatus::Error(
            TriangulationFailure::InvalidInput,
            StatusCode::ValidationError,
            "Triangulation input contains non-finite values");
    }

    const float wa = std::sqrt(std::max(0.0f, weight_a));
    const float wb = std::sqrt(std::max(0.0f, weight_b));
    if (wa <= 0.0f && wb <= 0.0f) {
        return TriangulationStatus::Error(
            TriangulationFailure::ZeroWeights,
            StatusCode::ValidationError,
            "Triangulation weights are zero");
    }

    // Homogeneous DLT rows: (u * P3 - P1) X_h = 0 and (v * P3 - P2) X_h = 0.
    // Phase 6.12 intentionally solves the 4D null vector directly instead of
    // collapsing the system into 3x3 normal equations against an implicit w=1.
    // The rows are built in double and row-equilibrated before forming A^T A;
    // otherwise high-resolution pixel/projection magnitudes can dominate the
    // normal matrix and make usable stereo pairs look singular.
    double rows[4][4] = {};
    BuildDltRowsForView(pa, a, rows[0], rows[1]);
    BuildDltRowsForView(pb, b, rows[2], rows[3]);
    for (int row = 0; row < 4; ++row) {
        if (!NormalizeEquationRow(rows[row])) {
            return TriangulationStatus::Error(
                TriangulationFailure::RowNormalizationFailed,
                StatusCode::ValidationError,
                "Triangulation DLT row normalization failed");
        }
    }
    ScaleEquationRow(rows[0], wa);
    ScaleEquationRow(rows[1], wa);
    ScaleEquationRow(rows[2], wb);
    ScaleEquationRow(rows[3], wb);

    const HomogeneousDltSolution dlt = SolveHomogeneousDltBySymmetricEigen(rows, config);
    if (!dlt.valid) {
        return TriangulationStatus::Error(
            dlt.failure,
            StatusCode::ValidationError,
            dlt.failure_message);
    }
    if (dlt.ill_conditioned) {
        return TriangulationStatus::Error(
            TriangulationFailure::IllConditioned,
            StatusCode::ValidationError,
            "Triangulation geometry is ill-conditioned");
    }

    const Vec3f x = dlt.world;
    const float depth_a = CameraDepth(pa, x);
    const float depth_b = CameraDepth(pb, x);
    if (!IsFinite(depth_a) || !IsFinite(depth_b) || depth_a <= kMinPositiveDepth || depth_b <= kMinPositiveDepth) {
        return TriangulationStatus::Error(
            TriangulationFailure::BehindCamera,
            StatusCode::ValidationError,
            "Triangulated point is behind one or both cameras");
    }

    const auto projected_a = Project(pa, x);
    if (!projected_a.ok()) {
        return TriangulationStatus::Error(
            TriangulationFailure::ProjectionFailed,
            projected_a.status().code,
            projected_a.status().message);
    }
    const auto projected_b = Project(pb, x);
    if (!projected_b.ok()) {
        return TriangulationStatus::Error(
            TriangulationFailure::ProjectionFailed,
            projected_b.status().code,
            projected_b.status().message);
    }

    const float ea = Distance(projected_a.value(), a);
    const float eb = Distance(projected_b.value(), b);
    if (!IsFinite(ea) || !IsFinite(eb)) {
        return TriangulationStatus::Error(
            TriangulationFailure::NonFiniteReprojectionError,
            StatusCode::ValidationError,
            "Triangulation reprojection error is non-finite");
    }

    TriangulatedPoint out;
    out.world = x;
    out.reprojection_error_a = ea;
    out.reprojection_error_b = eb;
    out.confidence = StereoReprojectionConfidence(ea, eb);
    out.dlt_condition_number = dlt.condition_number;
    out.dlt_strength_ratio = dlt.strength_ratio;
    out.dlt_null_residual = dlt.null_residual;
    out.triangulation_ill_conditioned = dlt.ill_conditioned;
    out.valid = true;
    return out;
}

} // namespace bt
