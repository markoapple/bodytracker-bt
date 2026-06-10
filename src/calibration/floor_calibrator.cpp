#include "calibration/floor_calibrator.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <numeric>

namespace bt {
namespace {

constexpr float kPi = 3.14159265358979323846f;

float Clamp01(float value) {
    if (!std::isfinite(value)) {
        return 0.0f;
    }
    return std::max(0.0f, std::min(1.0f, value));
}

float NormalizeAnglePi(float angle) {
    while (angle < 0.0f) angle += kPi;
    while (angle >= kPi) angle -= kPi;
    return angle;
}

float AngleDistancePi(float a, float b) {
    float d = std::abs(NormalizeAnglePi(a) - NormalizeAnglePi(b));
    if (d > 0.5f * kPi) d = kPi - d;
    return d;
}

float Median(std::vector<float> values, float fallback = 0.0f) {
    if (values.empty()) return fallback;
    std::sort(values.begin(), values.end());
    const std::size_t mid = values.size() / 2;
    if (values.size() % 2 == 1) return values[mid];
    return 0.5f * (values[mid - 1] + values[mid]);
}

float DominantAdjacentSpacing(std::vector<float> deltas) {
    if (deltas.empty()) {
        return 0.0f;
    }
    std::sort(deltas.begin(), deltas.end());

    std::vector<float> best_values;
    float best_median = deltas.front();
    for (std::size_t i = 0; i < deltas.size(); ++i) {
        const float seed = deltas[i];
        const float tolerance = std::max(4.0f, 0.16f * std::max(1.0f, seed));
        std::vector<float> values;
        for (const auto d : deltas) {
            if (std::abs(d - seed) <= tolerance) {
                values.push_back(d);
            }
        }
        const float median = Median(values, seed);
        if (values.size() > best_values.size() ||
            (values.size() == best_values.size() && median > best_median)) {
            best_values = std::move(values);
            best_median = median;
        }
    }
    return Median(best_values, Median(deltas));
}

float LineLength(const FloorSeamLine2D& line) {
    return Distance(line.a, line.b);
}

float LineAngle(const FloorSeamLine2D& line) {
    return NormalizeAnglePi(std::atan2(line.b.y - line.a.y, line.b.x - line.a.x));
}

Vec2f LineMid(const FloorSeamLine2D& line) {
    return Vec2f{0.5f * (line.a.x + line.b.x), 0.5f * (line.a.y + line.b.y)};
}

float RhoForLine(const FloorSeamLine2D& line, float orientation) {
    const Vec2f mid = LineMid(line);
    const float nx = -std::sin(orientation);
    const float ny = std::cos(orientation);
    return mid.x * nx + mid.y * ny;
}

float DominantOrientationPi(const std::vector<FloorSeamLine2D>& lines, const std::vector<std::size_t>& indices) {
    double x = 0.0;
    double y = 0.0;
    for (const auto index : indices) {
        if (index >= lines.size()) {
            continue;
        }
        const auto angle = LineAngle(lines[index]);
        const double weight =
            static_cast<double>(std::max(0.05f, lines[index].strength) * std::max(1.0f, LineLength(lines[index])));
        x += weight * std::cos(2.0 * static_cast<double>(angle));
        y += weight * std::sin(2.0 * static_cast<double>(angle));
    }
    if (!std::isfinite(x) || !std::isfinite(y) || (std::abs(x) + std::abs(y)) < 1e-6) {
        return indices.empty() || indices.front() >= lines.size() ? 0.0f : LineAngle(lines[indices.front()]);
    }
    return NormalizeAnglePi(0.5f * static_cast<float>(std::atan2(y, x)));
}

struct RhoCluster {
    float rho = 0.0f;
    float weight = 0.0f;
    float length_sum = 0.0f;
    std::vector<std::size_t> indices{};
};

struct GroupScore {
    float orientation = 0.0f;
    std::vector<std::size_t> indices{};
    std::vector<RhoCluster> seam_clusters{};
    float spacing = 0.0f;
    float reference = 0.0f;
    float confidence = 0.0f;
    std::string reason;
};

std::vector<RhoCluster> ClusterParallelSeamFragments(
    const std::vector<FloorSeamLine2D>& lines,
    const std::vector<std::size_t>& indices,
    float orientation) {

    std::vector<RhoCluster> clusters;
    struct Item { float rho; std::size_t index; float weight; };
    std::vector<Item> items;
    items.reserve(indices.size());
    for (const auto index : indices) {
        const float len = LineLength(lines[index]);
        const float weight = std::max(0.05f, lines[index].strength) * std::max(1.0f, len);
        items.push_back(Item{RhoForLine(lines[index], orientation), index, weight});
    }
    std::sort(items.begin(), items.end(), [](const Item& a, const Item& b) { return a.rho < b.rho; });

    constexpr float kSameSeamRhoPx = 8.0f;
    for (const auto& item : items) {
        if (clusters.empty() || std::abs(item.rho - clusters.back().rho) > kSameSeamRhoPx) {
            RhoCluster c;
            c.rho = item.rho;
            clusters.push_back(c);
        }
        auto& c = clusters.back();
        const float total = c.weight + item.weight;
        c.rho = total > 0.0f ? (c.rho * c.weight + item.rho * item.weight) / total : item.rho;
        c.weight = total;
        c.length_sum += LineLength(lines[item.index]);
        c.indices.push_back(item.index);
    }
    return clusters;
}

GroupScore ScoreGroup(
    const std::vector<FloorSeamLine2D>& lines,
    const std::vector<std::size_t>& indices,
    float orientation,
    int image_width,
    int image_height) {

    GroupScore score;
    score.orientation = orientation;
    score.indices = indices;
    score.seam_clusters = ClusterParallelSeamFragments(lines, indices, orientation);
    if (score.seam_clusters.size() < 3) {
        score.reason = "not_enough_parallel_seams";
        return score;
    }

    std::vector<float> rhos;
    rhos.reserve(score.seam_clusters.size());
    for (const auto& cluster : score.seam_clusters) {
        rhos.push_back(cluster.rho);
    }
    std::sort(rhos.begin(), rhos.end());

    std::vector<float> deltas;
    std::vector<float> expected_deltas;
    for (std::size_t i = 1; i < rhos.size(); ++i) {
        const float d = rhos[i] - rhos[i - 1];
        if (d >= 6.0f && d <= 0.55f * static_cast<float>(std::max(1, image_height))) {
            deltas.push_back(d);
        }
    }
    if (deltas.size() < 2) {
        score.reason = "spacing_not_repeated";
        return score;
    }

    const float spacing = DominantAdjacentSpacing(deltas);
    std::vector<float> abs_err;
    abs_err.reserve(deltas.size());
    for (const auto d : deltas) {
        // Missing/occluded seams commonly produce a 2x/3x gap. Fold those gaps
        // back to the nearest integer multiple instead of calling the floor inconsistent.
        const float multiple = std::max(1.0f, std::round(d / std::max(1.0f, spacing)));
        const float normalized = d / multiple;
        expected_deltas.push_back(normalized);
        abs_err.push_back(std::abs(normalized - spacing));
    }
    const float robust_spacing = Median(expected_deltas, spacing);
    abs_err.clear();
    for (const auto d : deltas) {
        const float multiple = std::max(1.0f, std::round(d / std::max(1.0f, robust_spacing)));
        abs_err.push_back(std::abs(d / multiple - robust_spacing));
    }
    const float mad = Median(abs_err);
    const float spread_ratio = mad / std::max(1.0f, robust_spacing);
    if (spread_ratio > 0.45f) {
        score.reason = "spacing_family_inconsistent";
        return score;
    }

    float strength = 0.0f;
    float length_sum = 0.0f;
    int fragment_count = 0;
    for (const auto& cluster : score.seam_clusters) {
        strength += cluster.weight / std::max(1.0f, cluster.length_sum);
        length_sum += cluster.length_sum;
        fragment_count += static_cast<int>(cluster.indices.size());
    }
    strength /= static_cast<float>(std::max<std::size_t>(1, score.seam_clusters.size()));
    const float count_quality = Clamp01(static_cast<float>(score.seam_clusters.size()) / 7.0f);
    const float stability = Clamp01(1.0f - spread_ratio / 0.45f);
    const float strength_quality = Clamp01(strength);
    // Fragmented/occluded seams are allowed. The useful signal is total coverage
    // across the family, not one uninterrupted line spanning the floor.
    const float coverage_target = 0.80f * static_cast<float>(std::max(1, image_width));
    const float coverage_quality = Clamp01(length_sum / std::max(1.0f, coverage_target * std::max<int>(1, static_cast<int>(score.seam_clusters.size()) / 2)));
    const float fragmentation_penalty = Clamp01(static_cast<float>(score.seam_clusters.size()) / std::max(1.0f, static_cast<float>(fragment_count)));

    score.spacing = robust_spacing;
    score.reference = rhos.back();
    score.confidence = Clamp01(
        0.30f * count_quality +
        0.34f * stability +
        0.18f * strength_quality +
        0.14f * coverage_quality +
        0.04f * fragmentation_penalty);
    score.reason = fragment_count > static_cast<int>(score.seam_clusters.size())
        ? "stable_repeated_floor_seam_family_with_fragmented_occlusion_tolerance"
        : "stable_repeated_floor_seam_family";
    return score;
}

} // namespace

Result<FloorPlane> EstimateFloorPlaneFromWorldPoints(const std::vector<Vec3f>& points) {
    if (points.size() < 3) {
        return Status::Error(StatusCode::InvalidArgument, "Floor calibration requires at least three points");
    }

    const Vec3f origin = points.front();
    Vec3f normal{};
    bool found_non_collinear = false;
    for (std::size_t i = 1; i + 1 < points.size(); ++i) {
        const Vec3f a = Sub(points[i], origin);
        for (std::size_t j = i + 1; j < points.size(); ++j) {
            const Vec3f b = Sub(points[j], origin);
            normal = Cross(a, b);
            if (Length(normal) > 1e-5f) {
                found_non_collinear = true;
                break;
            }
        }
        if (found_non_collinear) {
            break;
        }
    }

    if (!found_non_collinear) {
        return Status::Error(StatusCode::ValidationError, "Floor calibration points are collinear");
    }

    normal = Normalize(normal);
    if (normal.y < 0.0f) {
        normal = Scale(normal, -1.0f);
    }

    Vec3f centroid{};
    for (const auto& p : points) {
        centroid = Add(centroid, p);
    }
    centroid = Scale(centroid, 1.0f / static_cast<float>(points.size()));

    float max_abs_residual = 0.0f;
    const float distance = Dot(normal, centroid);
    for (const auto& p : points) {
        max_abs_residual = std::max(max_abs_residual, std::abs(Dot(normal, p) - distance));
    }
    if (max_abs_residual > 0.035f) {
        return Status::Error(StatusCode::ValidationError, "Floor calibration residual is too large");
    }

    FloorPlane plane;
    plane.normal = normal;
    plane.distance = distance;
    plane.valid = true;
    return plane;
}

FloorSeamFamilyEstimate EstimateRepeatedFloorSeamFamily(
    const std::vector<FloorSeamLine2D>& lines,
    int image_width,
    int image_height) {

    FloorSeamFamilyEstimate out;
    if (image_width <= 0 || image_height <= 0) {
        out.reason = "invalid_image_size";
        return out;
    }
    if (lines.size() < 3) {
        out.reason = "not_enough_candidate_lines";
        return out;
    }

    out.candidates.reserve(lines.size());
    std::vector<float> angles;
    std::vector<std::size_t> usable;
    const float min_length = 0.045f * static_cast<float>(image_width);
    for (std::size_t i = 0; i < lines.size(); ++i) {
        FloorSeamCandidateDebug dbg;
        dbg.line = lines[i];
        dbg.angle_rad = LineAngle(lines[i]);
        dbg.rho_px = RhoForLine(lines[i], dbg.angle_rad);
        if (LineLength(lines[i]) < min_length && lines[i].samples.size() < 4) {
            dbg.reason = "short_clutter";
        } else if (!std::isfinite(lines[i].strength) || lines[i].strength <= 0.0f) {
            dbg.reason = "weak_line";
        } else {
            dbg.reason = "candidate";
            usable.push_back(i);
            angles.push_back(dbg.angle_rad);
        }
        out.candidates.push_back(dbg);
    }

    if (usable.size() < 3) {
        out.rejected_line_count = static_cast<int>(lines.size());
        out.reason = "not_enough_floor_roi_lines";
        return out;
    }

    GroupScore best;
    constexpr float kAngleTol = 16.0f * kPi / 180.0f;
    for (const auto seed_index : usable) {
        const float seed_angle = LineAngle(lines[seed_index]);
        std::vector<std::size_t> group;
        for (const auto index : usable) {
            if (AngleDistancePi(LineAngle(lines[index]), seed_angle) <= kAngleTol) {
                group.push_back(index);
            }
        }
        const float group_orientation = DominantOrientationPi(lines, group);
        const GroupScore score = ScoreGroup(lines, group, group_orientation, image_width, image_height);
        if (score.confidence > best.confidence) {
            best = score;
        }
    }

    if (best.confidence <= 0.0f) {
        out.reason = best.reason.empty() ? "no_stable_orientation_spacing_family" : best.reason;
        out.rejected_line_count = static_cast<int>(lines.size());
        return out;
    }

    out.valid = best.confidence >= 0.25f;
    out.confidence = best.confidence;
    out.orientation_rad = best.orientation;
    out.spacing_px = best.spacing;
    out.reference_rho_px = best.reference;
    out.accepted_line_count = static_cast<int>(best.indices.size());
    out.rejected_line_count = static_cast<int>(lines.size()) - out.accepted_line_count;
    out.reason = best.reason;
    for (const auto index : best.indices) {
        if (index < out.candidates.size()) {
            out.candidates[index].accepted = true;
            out.candidates[index].reason = "accepted_repeated_family";
        }
    }
    return out;
}


namespace {

struct LineEquation2D {
    float a = 0.0f;
    float b = 0.0f;
    float c = 0.0f;
};

LineEquation2D EquationForLine(const FloorSeamLine2D& line) {
    const float a = line.a.y - line.b.y;
    const float b = line.b.x - line.a.x;
    const float c = line.a.x * line.b.y - line.b.x * line.a.y;
    const float len = std::sqrt(a * a + b * b);
    if (len <= 1e-5f || !std::isfinite(len)) {
        return {};
    }
    return LineEquation2D{a / len, b / len, c / len};
}

bool IntersectLines(const LineEquation2D& l0, const LineEquation2D& l1, Vec2f& out) {
    const float det = l0.a * l1.b - l1.a * l0.b;
    if (std::abs(det) <= 1e-5f || !std::isfinite(det)) {
        return false;
    }
    out.x = (l0.b * l1.c - l1.b * l0.c) / det;
    out.y = (l0.c * l1.a - l1.c * l0.a) / det;
    return std::isfinite(out.x) && std::isfinite(out.y);
}

Vec2f MedianPoint(std::vector<Vec2f> points, Vec2f fallback = {}) {
    if (points.empty()) return fallback;
    std::vector<float> xs;
    std::vector<float> ys;
    xs.reserve(points.size());
    ys.reserve(points.size());
    for (const auto& p : points) {
        xs.push_back(p.x);
        ys.push_back(p.y);
    }
    return Vec2f{Median(xs, fallback.x), Median(ys, fallback.y)};
}

std::vector<std::size_t> AcceptedCandidateIndices(const FloorSeamFamilyEstimate& family) {
    std::vector<std::size_t> out;
    for (std::size_t i = 0; i < family.candidates.size(); ++i) {
        if (family.candidates[i].accepted) {
            out.push_back(i);
        }
    }
    return out;
}

FloorGeometryLineFamily ToCalibrationFamily(
    const FloorSeamFamilyEstimate& estimate,
    float spacing_m,
    const std::vector<FloorSeamLine2D>& source_lines) {

    FloorGeometryLineFamily out;
    out.valid = estimate.valid;
    out.confidence = estimate.confidence;
    out.orientation_rad = estimate.orientation_rad;
    out.spacing_px = estimate.spacing_px;
    out.reference_rho_px = estimate.reference_rho_px;
    out.accepted_line_count = estimate.accepted_line_count;
    out.rejected_line_count = estimate.rejected_line_count;
    out.reason = estimate.reason;
    if (std::isfinite(spacing_m) && spacing_m > 0.0f) {
        out.spacing_m = spacing_m;
        out.metric_spacing_valid = true;
    }

    std::vector<Vec2f> intersections;
    const auto accepted = AcceptedCandidateIndices(estimate);
    for (std::size_t i = 0; i < accepted.size(); ++i) {
        const auto ia = accepted[i];
        if (ia >= source_lines.size()) continue;
        const auto ea = EquationForLine(source_lines[ia]);
        for (std::size_t j = i + 1; j < accepted.size(); ++j) {
            const auto ib = accepted[j];
            if (ib >= source_lines.size()) continue;
            const float angle_delta = AngleDistancePi(LineAngle(source_lines[ia]), LineAngle(source_lines[ib]));
            // Perfectly parallel line segments cannot produce a finite vanishing point. Slight
            // convergence across several seams is real perspective evidence; tiny deltas are noise.
            if (angle_delta < 1.5f * kPi / 180.0f || angle_delta > 18.0f * kPi / 180.0f) {
                continue;
            }
            Vec2f p{};
            if (IntersectLines(ea, EquationForLine(source_lines[ib]), p)) {
                intersections.push_back(p);
            }
        }
    }
    if (intersections.size() >= 3) {
        out.vanishing_point_px = MedianPoint(intersections);
        out.vanishing_point_valid = true;
    }
    return out;
}

float AngleOrthogonalityQuality(float a, float b) {
    const float d = AngleDistancePi(a, b);
    const float err = std::abs(d - 0.5f * kPi);
    return Clamp01(1.0f - err / (18.0f * kPi / 180.0f));
}

std::array<float, 9> IdentityHomography() {
    return {1.0f, 0.0f, 0.0f,
            0.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 1.0f};
}

std::array<float, 9> InvertAffineHomography(const std::array<float, 9>& h) {
    const float a = h[0];
    const float b = h[1];
    const float c = h[2];
    const float d = h[3];
    const float e = h[4];
    const float f = h[5];
    const float det = a * e - b * d;
    if (!std::isfinite(det) || std::abs(det) <= 1e-6f) {
        return IdentityHomography();
    }
    const float inv_det = 1.0f / det;
    const float ia = e * inv_det;
    const float ib = -b * inv_det;
    const float id = -d * inv_det;
    const float ie = a * inv_det;
    return {ia, ib, -(ia * c + ib * f),
            id, ie, -(id * c + ie * f),
            0.0f, 0.0f, 1.0f};
}

std::array<float, 9> OrientedAffineFloorFromImage(
    const FloorGeometryLineFamily& family_a,
    const FloorGeometryLineFamily& family_b,
    float scale_a,
    float scale_b,
    int image_width,
    int image_height) {

    const float cx = 0.5f * static_cast<float>(std::max(1, image_width));
    const float cy = 0.5f * static_cast<float>(std::max(1, image_height));
    const float nax = -std::sin(family_a.orientation_rad);
    const float nay = std::cos(family_a.orientation_rad);
    const float nbx = -std::sin(family_b.orientation_rad);
    const float nby = std::cos(family_b.orientation_rad);

    // This is intentionally an affine floor-grid map, not a fake perspective solve.
    // It respects the detected seam-family orientations and metric spacing. Full
    // projective homography still requires a stable vanishing geometry or labeled points.
    return {scale_a * nax, scale_a * nay, -scale_a * (cx * nax + cy * nay),
            scale_b * nbx, scale_b * nby, -scale_b * (cx * nbx + cy * nby),
            0.0f, 0.0f, 1.0f};
}

float PixelToMetricScale(const FloorGeometryLineFamily& family) {
    if (!family.valid || !family.metric_spacing_valid || family.spacing_px <= 1e-5f) {
        return 0.0f;
    }
    return family.spacing_m / family.spacing_px;
}

float Det3(const std::array<float, 9>& h) {
    return h[0] * (h[4] * h[8] - h[5] * h[7]) -
        h[1] * (h[3] * h[8] - h[5] * h[6]) +
        h[2] * (h[3] * h[7] - h[4] * h[6]);
}

std::array<float, 9> InvertHomography3x3(const std::array<float, 9>& h) {
    const float det = Det3(h);
    if (!std::isfinite(det) || std::abs(det) <= 1e-8f) {
        return IdentityHomography();
    }
    const float inv = 1.0f / det;
    return {
        (h[4] * h[8] - h[5] * h[7]) * inv,
        (h[2] * h[7] - h[1] * h[8]) * inv,
        (h[1] * h[5] - h[2] * h[4]) * inv,
        (h[5] * h[6] - h[3] * h[8]) * inv,
        (h[0] * h[8] - h[2] * h[6]) * inv,
        (h[2] * h[3] - h[0] * h[5]) * inv,
        (h[3] * h[7] - h[4] * h[6]) * inv,
        (h[1] * h[6] - h[0] * h[7]) * inv,
        (h[0] * h[4] - h[1] * h[3]) * inv
    };
}

Vec2f ApplyHomography(const std::array<float, 9>& h, const Vec2f& p) {
    const float w = h[6] * p.x + h[7] * p.y + h[8];
    if (!std::isfinite(w) || std::abs(w) <= 1e-8f) {
        return {};
    }
    return Vec2f{
        (h[0] * p.x + h[1] * p.y + h[2]) / w,
        (h[3] * p.x + h[4] * p.y + h[5]) / w};
}

bool SolveLinear8x8(float a[8][8], float b[8], float x[8]) {
    for (int col = 0; col < 8; ++col) {
        int pivot = col;
        float best = std::abs(a[col][col]);
        for (int row = col + 1; row < 8; ++row) {
            const float v = std::abs(a[row][col]);
            if (v > best) { best = v; pivot = row; }
        }
        if (best <= 1e-8f || !std::isfinite(best)) {
            return false;
        }
        if (pivot != col) {
            for (int k = col; k < 8; ++k) std::swap(a[pivot][k], a[col][k]);
            std::swap(b[pivot], b[col]);
        }
        const float inv = 1.0f / a[col][col];
        for (int k = col; k < 8; ++k) a[col][k] *= inv;
        b[col] *= inv;
        for (int row = 0; row < 8; ++row) {
            if (row == col) continue;
            const float f = a[row][col];
            if (std::abs(f) <= 1e-12f) continue;
            for (int k = col; k < 8; ++k) a[row][k] -= f * a[col][k];
            b[row] -= f * b[col];
        }
    }
    for (int i = 0; i < 8; ++i) {
        x[i] = b[i];
        if (!std::isfinite(x[i])) return false;
    }
    return true;
}

struct HomographyPair {
    Vec2f floor{};
    Vec2f image{};
};

struct ProjectiveHomographyResult {
    bool valid = false;
    std::array<float, 9> image_from_floor{};
    std::array<float, 9> floor_from_image{};
    float mean_reprojection_error_px = 0.0f;
    int intersection_count = 0;
    int inlier_count = 0;
    std::string reason = "unavailable";
};

bool SolveHomographyFromPairs(
    const std::vector<HomographyPair>& pairs,
    std::array<float, 9>& image_from_floor) {

    if (pairs.size() < 4) {
        return false;
    }

    float ata[8][8]{};
    float atb[8]{};
    for (const auto& pair : pairs) {
        const float X = pair.floor.x;
        const float Y = pair.floor.y;
        const float u = pair.image.x;
        const float v = pair.image.y;
        if (!std::isfinite(X) || !std::isfinite(Y) || !std::isfinite(u) || !std::isfinite(v)) {
            continue;
        }
        const float r0[8] = {X, Y, 1.0f, 0.0f, 0.0f, 0.0f, -u * X, -u * Y};
        const float r1[8] = {0.0f, 0.0f, 0.0f, X, Y, 1.0f, -v * X, -v * Y};
        for (int i = 0; i < 8; ++i) {
            atb[i] += r0[i] * u + r1[i] * v;
            for (int j = 0; j < 8; ++j) {
                ata[i][j] += r0[i] * r0[j] + r1[i] * r1[j];
            }
        }
    }

    float x[8]{};
    if (!SolveLinear8x8(ata, atb, x)) {
        return false;
    }
    image_from_floor = {x[0], x[1], x[2], x[3], x[4], x[5], x[6], x[7], 1.0f};
    return true;
}

float MeanReprojectionError(
    const std::array<float, 9>& image_from_floor,
    const std::vector<HomographyPair>& pairs) {

    if (pairs.empty()) return 999.0f;
    float err = 0.0f;
    int count = 0;
    for (const auto& pair : pairs) {
        const Vec2f pred = ApplyHomography(image_from_floor, pair.floor);
        if (!std::isfinite(pred.x) || !std::isfinite(pred.y)) continue;
        err += Distance(pred, pair.image);
        ++count;
    }
    return count > 0 ? err / static_cast<float>(count) : 999.0f;
}

std::vector<HomographyPair> BuildGridIntersectionPairs(
    const std::vector<FloorSeamLine2D>& family_a_lines,
    const std::vector<FloorSeamLine2D>& family_b_lines,
    const FloorGeometryLineFamily& family_a,
    const FloorGeometryLineFamily& family_b) {

    struct RLine { FloorSeamLine2D line; float rho = 0.0f; int index = 0; };
    std::vector<RLine> a_lines;
    std::vector<RLine> b_lines;
    for (const auto& l : family_a_lines) a_lines.push_back(RLine{l, RhoForLine(l, family_a.orientation_rad), 0});
    for (const auto& l : family_b_lines) b_lines.push_back(RLine{l, RhoForLine(l, family_b.orientation_rad), 0});
    std::sort(a_lines.begin(), a_lines.end(), [](const RLine& a, const RLine& b) { return a.rho < b.rho; });
    std::sort(b_lines.begin(), b_lines.end(), [](const RLine& a, const RLine& b) { return a.rho < b.rho; });

    const float ref_a = family_a.reference_rho_px;
    const float ref_b = family_b.reference_rho_px;
    std::vector<HomographyPair> pairs;
    for (const auto& la : a_lines) {
        const auto ea = EquationForLine(la.line);
        const float grid_a = std::round((la.rho - ref_a) / std::max(1.0f, family_a.spacing_px));
        const float floor_x = grid_a * family_a.spacing_m;
        for (const auto& lb : b_lines) {
            Vec2f p{};
            if (!IntersectLines(ea, EquationForLine(lb.line), p)) continue;
            if (!std::isfinite(p.x) || !std::isfinite(p.y)) continue;
            const float grid_b = std::round((lb.rho - ref_b) / std::max(1.0f, family_b.spacing_px));
            const float floor_y = grid_b * family_b.spacing_m;
            pairs.push_back(HomographyPair{Vec2f{floor_x, floor_y}, p});
        }
    }
    return pairs;
}

ProjectiveHomographyResult EstimateProjectiveHomographyFromFloorGrid(
    const std::vector<FloorSeamLine2D>& family_a_lines,
    const std::vector<FloorSeamLine2D>& family_b_lines,
    const FloorGeometryLineFamily& family_a,
    const FloorGeometryLineFamily& family_b) {

    ProjectiveHomographyResult out;
    out.image_from_floor = IdentityHomography();
    out.floor_from_image = IdentityHomography();
    if (!family_a.metric_spacing_valid || !family_b.metric_spacing_valid ||
        family_a_lines.size() < 2 || family_b_lines.size() < 2) {
        out.reason = "projective_requires_two_metric_line_families";
        return out;
    }

    const std::vector<HomographyPair> pairs = BuildGridIntersectionPairs(
        family_a_lines,
        family_b_lines,
        family_a,
        family_b);
    out.intersection_count = static_cast<int>(pairs.size());
    if (pairs.size() < 4) {
        out.reason = "not_enough_grid_intersections";
        return out;
    }

    // Deterministic RANSAC over grid intersections. Furniture/shadows can inject
    // wrong lines, so do not let every intersection vote equally into the homography.
    const float inlier_threshold_px = 7.0f;
    std::array<float, 9> best_h = IdentityHomography();
    std::vector<HomographyPair> best_inliers;
    float best_mean = 999.0f;
    const std::size_t n = pairs.size();
    std::size_t trials = 0;
    const std::size_t stride = std::max<std::size_t>(1, n / 11);
    for (std::size_t i0 = 0; i0 < n && trials < 800; i0 += stride) {
        for (std::size_t i1 = i0 + 1; i1 < n && trials < 800; i1 += stride) {
            for (std::size_t i2 = i1 + 1; i2 < n && trials < 800; i2 += stride) {
                for (std::size_t i3 = i2 + 1; i3 < n && trials < 800; i3 += stride) {
                    ++trials;
                    std::vector<HomographyPair> sample{pairs[i0], pairs[i1], pairs[i2], pairs[i3]};
                    std::array<float, 9> h = IdentityHomography();
                    if (!SolveHomographyFromPairs(sample, h)) continue;
                    std::vector<HomographyPair> inliers;
                    inliers.reserve(pairs.size());
                    for (const auto& pair : pairs) {
                        const Vec2f pred = ApplyHomography(h, pair.floor);
                        if (!std::isfinite(pred.x) || !std::isfinite(pred.y)) continue;
                        if (Distance(pred, pair.image) <= inlier_threshold_px) {
                            inliers.push_back(pair);
                        }
                    }
                    if (inliers.size() < 4) continue;
                    const float mean = MeanReprojectionError(h, inliers);
                    if (inliers.size() > best_inliers.size() ||
                        (inliers.size() == best_inliers.size() && mean < best_mean)) {
                        best_h = h;
                        best_inliers = std::move(inliers);
                        best_mean = mean;
                    }
                }
            }
        }
    }

    const int min_inliers = std::max(4, static_cast<int>(std::ceil(0.55f * static_cast<float>(pairs.size()))));
    if (static_cast<int>(best_inliers.size()) < min_inliers) {
        out.reason = "projective_ransac_rejected_unstable_grid_intersections";
        out.inlier_count = static_cast<int>(best_inliers.size());
        out.mean_reprojection_error_px = best_mean;
        return out;
    }

    std::array<float, 9> refined = best_h;
    if (!SolveHomographyFromPairs(best_inliers, refined)) {
        out.reason = "projective_refit_failed";
        return out;
    }
    const float refined_mean = MeanReprojectionError(refined, best_inliers);
    if (!std::isfinite(refined_mean) || refined_mean > 6.5f) {
        out.reason = "projective_reprojection_error_too_high";
        out.inlier_count = static_cast<int>(best_inliers.size());
        out.mean_reprojection_error_px = refined_mean;
        return out;
    }

    out.valid = true;
    out.image_from_floor = refined;
    out.floor_from_image = InvertHomography3x3(refined);
    out.inlier_count = static_cast<int>(best_inliers.size());
    out.mean_reprojection_error_px = refined_mean;
    out.reason = "two_line_family_projective_floor_grid_ransac_inliers";
    return out;
}

std::vector<FloorSeamLine2D> AcceptedLinesFromFamily(
    const FloorSeamFamilyEstimate& family,
    const std::vector<FloorSeamLine2D>& source) {

    std::vector<FloorSeamLine2D> out;
    for (std::size_t i = 0; i < family.candidates.size() && i < source.size(); ++i) {
        if (family.candidates[i].accepted) {
            out.push_back(source[i]);
        }
    }
    return out;
}

struct CorrectedPointSet {
    std::vector<Vec2f> points{};
};

Vec2f CorrectDistortedPoint(
    const Vec2f& pixel,
    float cx,
    float cy,
    float f,
    float k1,
    float k2,
    float p1,
    float p2) {

    const float x = (pixel.x - cx) / f;
    const float y = (pixel.y - cy) / f;
    const float r2 = x * x + y * y;
    const float radial = 1.0f + k1 * r2 + k2 * r2 * r2;
    const float xu = x * radial + 2.0f * p1 * x * y + p2 * (r2 + 2.0f * x * x);
    const float yu = y * radial + p1 * (r2 + 2.0f * y * y) + 2.0f * p2 * x * y;
    return Vec2f{cx + f * xu, cy + f * yu};
}

float MeanLineResidual(const std::vector<Vec2f>& points) {
    if (points.size() < 4) {
        return 999.0f;
    }
    float mx = 0.0f;
    float my = 0.0f;
    for (const auto& p : points) { mx += p.x; my += p.y; }
    mx /= static_cast<float>(points.size());
    my /= static_cast<float>(points.size());
    float sxx = 0.0f;
    float sxy = 0.0f;
    float syy = 0.0f;
    for (const auto& p : points) {
        const float x = p.x - mx;
        const float y = p.y - my;
        sxx += x * x;
        sxy += x * y;
        syy += y * y;
    }
    const float theta = 0.5f * std::atan2(2.0f * sxy, sxx - syy);
    const float nx = -std::sin(theta);
    const float ny = std::cos(theta);
    float sum = 0.0f;
    for (const auto& p : points) {
        const float d = (p.x - mx) * nx + (p.y - my) * ny;
        sum += d * d;
    }
    return std::sqrt(sum / static_cast<float>(points.size()));
}

LensDistortionEstimate EstimateDistortionFromSampledSeams(
    const std::vector<FloorSeamLine2D>& accepted_lines,
    int image_width,
    int image_height,
    const FloorGeometryCalibrationOptions& options) {

    LensDistortionEstimate out;
    std::vector<std::vector<Vec2f>> curves;
    int point_count = 0;
    for (const auto& line : accepted_lines) {
        if (line.samples.size() >= 5) {
            curves.push_back(line.samples);
            point_count += static_cast<int>(line.samples.size());
        }
    }
    out.sampled_seam_count = static_cast<int>(curves.size());
    out.sampled_point_count = point_count;
    if (curves.size() < 3 || point_count < 24) {
        out.available = false;
        out.valid = false;
        out.reason = "unavailable_without_sampled_seam_curves";
        return out;
    }

    const float cx = 0.5f * static_cast<float>(std::max(1, image_width));
    const float cy = 0.5f * static_cast<float>(std::max(1, image_height));
    float f = 0.0f;
    if (options.intrinsics_available && options.horizontal_fov_deg > 1.0f) {
        const float fov = std::max(20.0f, std::min(140.0f, options.horizontal_fov_deg)) * kPi / 180.0f;
        f = 0.5f * static_cast<float>(std::max(1, image_width)) / std::tan(0.5f * fov);
    } else {
        const float fov = std::max(30.0f, std::min(130.0f, options.horizontal_fov_deg > 1.0f ? options.horizontal_fov_deg : 70.0f)) * kPi / 180.0f;
        f = 0.5f * static_cast<float>(std::max(1, image_width)) / std::tan(0.5f * fov);
    }
    if (!std::isfinite(f) || f <= 1.0f) {
        out.reason = "unavailable_without_camera_focal_length";
        return out;
    }

    auto residual_only = [&](float k1, float k2, float p1, float p2) {
        float sum = 0.0f;
        int n = 0;
        for (const auto& curve : curves) {
            std::vector<Vec2f> corrected;
            corrected.reserve(curve.size());
            for (const auto& p : curve) {
                corrected.push_back(CorrectDistortedPoint(p, cx, cy, f, k1, k2, p1, p2));
            }
            const float residual = MeanLineResidual(corrected);
            if (std::isfinite(residual)) {
                sum += residual;
                ++n;
            }
        }
        return n > 0 ? sum / static_cast<float>(n) : 999.0f;
    };

    auto objective = [&](float k1, float k2, float p1, float p2) {
        const float penalty = 2.0f * (std::abs(k1) + 0.5f * std::abs(k2)) + 8.0f * (std::abs(p1) + std::abs(p2));
        return residual_only(k1, k2, p1, p2) + penalty;
    };

    const float baseline = residual_only(0.0f, 0.0f, 0.0f, 0.0f);
    float best_score = baseline;
    float best_k1 = 0.0f, best_k2 = 0.0f, best_p1 = 0.0f, best_p2 = 0.0f;
    for (float k1 = -0.45f; k1 <= 0.451f; k1 += 0.05f) {
        for (float k2 = -0.25f; k2 <= 0.251f; k2 += 0.05f) {
            const float score = objective(k1, k2, 0.0f, 0.0f);
            if (score < best_score) {
                best_score = score;
                best_k1 = k1;
                best_k2 = k2;
            }
        }
    }
    // Only estimate tangential terms after radial correction has a real win and enough off-axis curves.
    if (baseline - best_score > 0.08f && curves.size() >= 5) {
        for (float p1 = -0.030f; p1 <= 0.031f; p1 += 0.010f) {
            for (float p2 = -0.030f; p2 <= 0.031f; p2 += 0.010f) {
                const float score = objective(best_k1, best_k2, p1, p2);
                if (score < best_score) {
                    best_score = score;
                    best_p1 = p1;
                    best_p2 = p2;
                }
            }
        }
    }

    const float corrected_residual = residual_only(best_k1, best_k2, best_p1, best_p2);
    out.available = true;
    out.straightness_error_px = baseline;
    out.corrected_straightness_error_px = corrected_residual;
    const float improvement = baseline > 1e-5f ? (baseline - corrected_residual) / baseline : 0.0f;
    out.confidence = Clamp01(0.70f * improvement + 0.30f * Clamp01(static_cast<float>(point_count) / 90.0f));
    out.valid = improvement >= 0.08f && out.confidence >= 0.20f && corrected_residual < baseline;
    if (out.valid) {
        out.radial_k1 = best_k1;
        out.radial_k2 = best_k2;
        out.tangential_p1 = best_p1;
        out.tangential_p2 = best_p2;
        out.model = (std::abs(best_p1) > 1e-5f || std::abs(best_p2) > 1e-5f) ? "radial_tangential" : "radial";
        out.reason = "sampled_seam_straightness_improved_after_undistortion";
    } else {
        out.radial_k1 = 0.0f;
        out.radial_k2 = 0.0f;
        out.tangential_p1 = 0.0f;
        out.tangential_p2 = 0.0f;
        out.model = "none";
        out.confidence = Clamp01(0.25f * Clamp01(static_cast<float>(point_count) / 90.0f));
        out.reason = "sampled_seams_do_not_support_stable_distortion_fit";
    }
    return out;
}

void EstimateCameraOrientationFromVanishingGeometry(
    FloorGeometryCalibration& out,
    int image_width,
    int image_height,
    const FloorGeometryCalibrationOptions& options) {

    out.camera_yaw_rad = out.family_a.orientation_rad;
    const float cx = 0.5f * static_cast<float>(std::max(1, image_width));
    const float cy = 0.5f * static_cast<float>(std::max(1, image_height));
    const bool two_vps = out.family_a.vanishing_point_valid && out.family_b.vanishing_point_valid;
    if (two_vps) {
        const Vec2f v0 = out.family_a.vanishing_point_px;
        const Vec2f v1 = out.family_b.vanishing_point_px;
        float f2 = -((v0.x - cx) * (v1.x - cx) + (v0.y - cy) * (v1.y - cy));
        if (!std::isfinite(f2) || f2 < 16.0f) {
            const float fov = std::max(30.0f, std::min(130.0f, options.horizontal_fov_deg > 1.0f ? options.horizontal_fov_deg : 70.0f)) * kPi / 180.0f;
            const float f = 0.5f * static_cast<float>(std::max(1, image_width)) / std::tan(0.5f * fov);
            f2 = f * f;
        }
        const float f = std::sqrt(f2);
        Vec3f d0 = NormalizeOr(Vec3f{(v0.x - cx) / f, (v0.y - cy) / f, 1.0f}, Vec3f{1.0f, 0.0f, 0.0f});
        Vec3f d1 = NormalizeOr(Vec3f{(v1.x - cx) / f, (v1.y - cy) / f, 1.0f}, Vec3f{0.0f, 0.0f, 1.0f});
        Vec3f n = NormalizeOr(Cross(d0, d1), Vec3f{0.0f, -1.0f, 0.0f});
        if (n.y < 0.0f) n = Scale(n, -1.0f);
        out.camera_roll_rad = std::atan2(n.x, std::max(1e-5f, n.y));
        out.camera_pitch_rad = std::atan2(-n.z, std::sqrt(n.x * n.x + n.y * n.y));
        out.camera_orientation_valid = true;
        out.camera_orientation_confidence = Clamp01(0.55f * out.family_a.confidence + 0.45f * out.family_b.confidence);
        out.reason = out.reason.empty() ? "camera_orientation_from_orthogonal_vanishing_points" : out.reason;
        return;
    }

    const bool has_real_vanishing_point = out.family_a.vanishing_point_valid || out.family_b.vanishing_point_valid;
    if (has_real_vanishing_point) {
        const Vec2f vp = out.family_a.vanishing_point_valid ? out.family_a.vanishing_point_px : out.family_b.vanishing_point_px;
        const float fov = std::max(30.0f, std::min(130.0f, options.horizontal_fov_deg > 1.0f ? options.horizontal_fov_deg : 70.0f)) * kPi / 180.0f;
        const float focal_px = std::max(1.0f, 0.5f * static_cast<float>(std::max(1, image_width)) / std::tan(0.5f * fov));
        out.camera_pitch_rad = std::atan2(cy - vp.y, focal_px);
        out.camera_roll_rad = std::atan2(vp.x - cx, focal_px);
        out.camera_orientation_valid = false;
        out.camera_orientation_confidence = Clamp01(out.floor_plane_confidence * (out.two_axis_grid_valid ? 0.35f : 0.20f));
        if (out.reason.empty()) {
            out.reason = "single_vanishing_point_orientation_estimate_degraded";
        }
    } else {
        // No real vanishing point: yaw is line-family evidence, not a solved
        // camera pitch/roll orientation model. Keep it out of the runtime
        // correction path and report it only as unsolved orientation evidence.
        out.camera_pitch_rad = 0.0f;
        out.camera_roll_rad = 0.0f;
        out.camera_orientation_valid = false;
        out.camera_orientation_confidence = 0.0f;
        if (out.reason.empty()) {
            out.reason = "yaw_only_floor_geometry_no_vanishing_point";
        }
    }
}

} // namespace

FloorGeometryDetectionDebug EstimateFloorGeometryCalibration(
    const std::vector<FloorSeamLine2D>& lines,
    int image_width,
    int image_height,
    const FloorGeometryCalibrationOptions& options) {

    FloorGeometryDetectionDebug debug;
    auto& out = debug.calibration;
    out.image_width = image_width;
    out.image_height = image_height;
    out.floor_type = options.floor_type.empty() ? "unknown" : options.floor_type;
    out.floor_from_image = IdentityHomography();
    out.image_from_floor = IdentityHomography();

    const FloorSeamFamilyEstimate primary = EstimateRepeatedFloorSeamFamily(lines, image_width, image_height);
    debug.candidates = primary.candidates;
    if (!primary.valid) {
        out.valid = false;
        out.reason = primary.reason.empty() ? "no_floor_geometry" : primary.reason;
        out.distortion.reason = "unavailable_without_stable_seams";
        return debug;
    }

    out.family_a = ToCalibrationFamily(primary, options.family_a_spacing_m, lines);
    out.family_count = 1;
    out.valid = out.family_a.valid;
    out.reason = "one_line_family_floor_geometry";

    // Try to find a second, genuinely different floor family. This intentionally treats
    // multiple parallel seams as signal and treats ambiguity as inconsistent orientation/spacing.
    std::vector<FloorSeamLine2D> remaining;
    std::vector<std::size_t> remaining_source_indices;
    const auto accepted_primary = AcceptedCandidateIndices(primary);
    for (std::size_t i = 0; i < lines.size(); ++i) {
        if (std::find(accepted_primary.begin(), accepted_primary.end(), i) != accepted_primary.end()) {
            continue;
        }
        remaining.push_back(lines[i]);
        remaining_source_indices.push_back(i);
    }
    const FloorSeamFamilyEstimate secondary_local = EstimateRepeatedFloorSeamFamily(remaining, image_width, image_height);
    if (secondary_local.valid) {
        const float orth_quality = AngleOrthogonalityQuality(primary.orientation_rad, secondary_local.orientation_rad);
        if (orth_quality >= 0.35f) {
            // Re-run on original index order for debug consistency by materializing the selected secondary lines.
            FloorSeamFamilyEstimate secondary = secondary_local;
            out.family_b = ToCalibrationFamily(secondary, options.family_b_spacing_m, remaining);
            out.family_b.confidence *= orth_quality;
            out.family_b.reason = "accepted_second_floor_line_family";
            out.family_count = 2;
            out.two_axis_grid_valid = out.family_b.confidence >= 0.20f;
            out.reason = out.two_axis_grid_valid ? "two_line_family_floor_grid" : "weak_second_floor_family";
        } else {
            debug.rejected_families.push_back(secondary_local);
        }
    }

    const float scale_a = PixelToMetricScale(out.family_a);
    const float scale_b = PixelToMetricScale(out.family_b);
    const auto accepted_a_lines = AcceptedLinesFromFamily(primary, lines);
    const auto accepted_b_lines = out.family_b.valid ? AcceptedLinesFromFamily(secondary_local, remaining) : std::vector<FloorSeamLine2D>{};
    if (scale_a > 0.0f && scale_b > 0.0f && out.two_axis_grid_valid) {
        out.metric_scale_confidence = Clamp01(0.5f * (out.family_a.confidence + out.family_b.confidence));
        const auto projective = EstimateProjectiveHomographyFromFloorGrid(
            accepted_a_lines,
            accepted_b_lines,
            out.family_a,
            out.family_b);
        out.homography_reprojection_error_px = projective.mean_reprojection_error_px;
        out.homography_inlier_count = projective.inlier_count;
        out.homography_intersection_count = projective.intersection_count;
        out.homography_reason = projective.reason;
        if (projective.valid) {
            out.image_from_floor = projective.image_from_floor;
            out.floor_from_image = projective.floor_from_image;
            out.homography_valid = out.metric_scale_confidence >= 0.25f;
            out.reason = "two_line_family_projective_floor_grid_ransac_inliers";
        } else {
            // Fallback is explicitly affine/orientation-aware. It is useful for local scale,
            // but not claimed as full projective camera calibration.
            out.floor_from_image = OrientedAffineFloorFromImage(
                out.family_a,
                out.family_b,
                scale_a,
                scale_b,
                image_width,
                image_height);
            out.image_from_floor = InvertAffineHomography(out.floor_from_image);
            out.homography_valid = false;
            out.homography_reason = projective.reason;
            out.reason = "two_line_family_grid_without_stable_projective_intersections";
        }
    } else if (scale_a > 0.0f) {
        out.metric_scale_confidence = out.family_a.confidence * 0.65f;
        out.planted_drift_axis_confidence = out.metric_scale_confidence;
        out.homography_valid = false; // one family gives one metric axis, not a full floor homography.
    } else {
        out.metric_scale_confidence = 0.0f;
        out.homography_valid = false;
    }

    out.floor_plane.normal = Vec3f{0.0f, 1.0f, 0.0f};
    out.floor_plane.distance = 0.0f;
    out.floor_plane.valid = true;
    out.floor_plane_confidence = out.two_axis_grid_valid
        ? Clamp01(0.70f * out.family_a.confidence + 0.30f * out.family_b.confidence)
        : Clamp01(0.45f * out.family_a.confidence);

    EstimateCameraOrientationFromVanishingGeometry(out, image_width, image_height, options);

    if (std::isfinite(options.camera_height_m) && options.camera_height_m > 0.10f) {
        out.camera_height_valid = out.metric_scale_confidence > 0.0f || options.intrinsics_available || out.homography_valid;
        out.camera_height_m = options.camera_height_m;
    }

    std::vector<FloorSeamLine2D> distortion_lines = accepted_a_lines;
    distortion_lines.insert(distortion_lines.end(), accepted_b_lines.begin(), accepted_b_lines.end());
    out.distortion = EstimateDistortionFromSampledSeams(distortion_lines, image_width, image_height, options);

    out.valid = out.family_a.valid;
    return debug;
}

WallRectangleCalibration EstimateWallRectangleCalibration(
    const std::array<Vec2f, 4>& image_points,
    int image_width,
    int image_height,
    const WallRectangleCalibrationOptions& options) {

    WallRectangleCalibration out;
    out.image_width = image_width;
    out.image_height = image_height;
    out.source = options.source.empty() ? "manual_wall_rectangle" : options.source;
    out.image_corners = image_points;
    out.wall_from_image = IdentityHomography();
    out.image_from_wall = IdentityHomography();
    out.rectangle_width_m = options.rectangle_width_m;
    out.rectangle_height_m = options.rectangle_height_m;
    out.rectangle_aspect_ratio = options.rectangle_aspect_ratio;
    out.usable_for_floor_plane = false;
    out.usable_for_floor_homography = false;

    if (image_width <= 0 || image_height <= 0) {
        out.reason = "wall rectangle image dimensions are invalid";
        out.capability_reason = "no wall calibration without finite preview size";
        return out;
    }

    for (const Vec2f& p : image_points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
            out.reason = "wall rectangle has non-finite image point";
            out.capability_reason = "non-finite wall points rejected";
            return out;
        }
    }

    const float top = Distance(image_points[0], image_points[1]);
    const float right = Distance(image_points[1], image_points[2]);
    const float bottom = Distance(image_points[2], image_points[3]);
    const float left = Distance(image_points[3], image_points[0]);
    const float min_edge = std::min(std::min(top, right), std::min(bottom, left));
    const float avg_w_px = 0.5f * (top + bottom);
    const float avg_h_px = 0.5f * (left + right);
    const float signed_area =
        0.5f * ((image_points[0].x * image_points[1].y - image_points[1].x * image_points[0].y) +
                (image_points[1].x * image_points[2].y - image_points[2].x * image_points[1].y) +
                (image_points[2].x * image_points[3].y - image_points[3].x * image_points[2].y) +
                (image_points[3].x * image_points[0].y - image_points[0].x * image_points[3].y));
    const float area = std::abs(signed_area);
    if (min_edge < 8.0f || area < 64.0f) {
        out.reason = "wall rectangle is too small or degenerate";
        out.capability_reason = "draw a visible four-corner wall rectangle";
        return out;
    }

    if (!(out.rectangle_width_m > 0.0f) || !(out.rectangle_height_m > 0.0f)) {
        const float aspect = out.rectangle_aspect_ratio > 0.01f
            ? out.rectangle_aspect_ratio
            : std::max(0.05f, avg_w_px / std::max(1.0f, avg_h_px));
        out.rectangle_width_m = aspect;
        out.rectangle_height_m = 1.0f;
        out.rectangle_aspect_ratio = aspect;
    } else {
        out.rectangle_aspect_ratio = out.rectangle_width_m / std::max(1e-5f, out.rectangle_height_m);
    }

    const std::vector<HomographyPair> pairs{
        {Vec2f{0.0f, 0.0f}, image_points[0]},
        {Vec2f{out.rectangle_width_m, 0.0f}, image_points[1]},
        {Vec2f{out.rectangle_width_m, out.rectangle_height_m}, image_points[2]},
        {Vec2f{0.0f, out.rectangle_height_m}, image_points[3]}
    };
    std::array<float, 9> image_from_wall = IdentityHomography();
    if (SolveHomographyFromPairs(pairs, image_from_wall)) {
        out.image_from_wall = image_from_wall;
        out.wall_from_image = InvertHomography3x3(image_from_wall);
        out.homography_reprojection_error_px = MeanReprojectionError(image_from_wall, pairs);
        out.wall_homography_valid = std::isfinite(out.homography_reprojection_error_px) &&
            out.homography_reprojection_error_px <= std::max(3.0f, 0.02f * std::max(avg_w_px, avg_h_px));
    }

    const bool metric_dimensions = options.rectangle_width_m > 0.0f && options.rectangle_height_m > 0.0f;
    out.metric_scale_valid = metric_dimensions && out.wall_homography_valid;
    out.metric_scale_confidence = out.metric_scale_valid ? 0.72f : 0.0f;

    if (options.horizontal_fov_deg > 1.0f) {
        const float fov = std::max(30.0f, std::min(130.0f, options.horizontal_fov_deg)) * kPi / 180.0f;
        const float focal_px = 0.5f * static_cast<float>(std::max(1, image_width)) / std::tan(0.5f * fov);
        const float cx = 0.5f * static_cast<float>(image_width);
        const float cy = 0.5f * static_cast<float>(image_height);
        const auto ray = [&](const Vec2f& p) {
            return NormalizeOr(Vec3f{(p.x - cx) / focal_px, (p.y - cy) / focal_px, 1.0f}, Vec3f{0.0f, 0.0f, 1.0f});
        };
        const Vec3f r0 = ray(image_points[0]);
        const Vec3f r1 = ray(image_points[1]);
        const Vec3f r2 = ray(image_points[2]);
        const Vec3f r3 = ray(image_points[3]);
        out.wall_right_camera = NormalizeOr(Add(Sub(r1, r0), Sub(r2, r3)), Vec3f{1.0f, 0.0f, 0.0f});
        out.wall_down_camera = NormalizeOr(Add(Sub(r3, r0), Sub(r2, r1)), Vec3f{0.0f, 1.0f, 0.0f});
        out.wall_normal_camera = NormalizeOr(Cross(out.wall_right_camera, out.wall_down_camera), Vec3f{0.0f, 0.0f, 1.0f});
        out.wall_orientation_valid = out.wall_homography_valid;
        out.wall_orientation_confidence = out.wall_orientation_valid ? 0.60f : 0.0f;
        if (metric_dimensions && avg_w_px > 1.0f) {
            out.wall_center_depth_m = options.rectangle_width_m * focal_px / avg_w_px;
            out.wall_depth_valid = std::isfinite(out.wall_center_depth_m) && out.wall_center_depth_m > 0.05f;
            out.wall_depth_confidence = out.wall_depth_valid ? 0.55f : 0.0f;
        }
    }

    out.usable_for_wall_homography = out.wall_homography_valid;
    out.usable_for_metric_scale = out.metric_scale_valid;
    out.usable_for_orientation = out.wall_orientation_valid;
    out.usable_for_depth_assist = out.wall_depth_valid;
    out.confidence = Clamp01(0.45f * (out.wall_homography_valid ? 1.0f : 0.0f) +
        0.25f * out.metric_scale_confidence +
        0.20f * out.wall_orientation_confidence +
        0.10f * out.wall_depth_confidence);
    out.valid = out.usable_for_wall_homography ||
        out.usable_for_metric_scale ||
        out.usable_for_orientation ||
        out.usable_for_depth_assist;
    out.reason = out.valid ? "manual wall rectangle captured as runtime wall calibration" : "wall rectangle captured but not geometrically usable";
    out.capability_reason = out.valid
        ? "usable for scoped wall homography/orientation/depth assist; paired with plank floor warp when both are saved"
        : "insufficient wall rectangle geometry";
    return out;
}

MultiCameraFloorAlignmentEstimate EstimateMultiCameraFloorAlignment(
    const FloorGeometryCalibration& camera_a,
    const FloorGeometryCalibration& camera_b) {

    MultiCameraFloorAlignmentEstimate out;
    out.floor_b_from_floor_a = IdentityHomography();
    if (!camera_a.valid || !camera_b.valid) {
        out.reason = "missing_floor_geometry_for_one_or_both_cameras";
        return out;
    }
    const float yaw_delta = AngleDistancePi(camera_a.camera_yaw_rad, camera_b.camera_yaw_rad);
    const float pitch_delta = std::abs(camera_a.camera_pitch_rad - camera_b.camera_pitch_rad);
    const float roll_delta = std::abs(camera_a.camera_roll_rad - camera_b.camera_roll_rad);
    out.yaw_delta_rad = yaw_delta;
    out.pitch_delta_rad = pitch_delta;
    out.roll_delta_rad = roll_delta;
    if (camera_a.camera_height_valid && camera_b.camera_height_valid) {
        out.height_delta_m = std::abs(camera_a.camera_height_m - camera_b.camera_height_m);
    }
    const float scale_a = camera_a.family_a.metric_spacing_valid ? camera_a.family_a.spacing_m / std::max(1.0f, camera_a.family_a.spacing_px) : 0.0f;
    const float scale_b = camera_b.family_a.metric_spacing_valid ? camera_b.family_a.spacing_m / std::max(1.0f, camera_b.family_a.spacing_px) : 0.0f;
    if (scale_a > 0.0f && scale_b > 0.0f) {
        out.scale_ratio = scale_b / scale_a;
    }
    const float yaw_quality = Clamp01(1.0f - yaw_delta / (18.0f * kPi / 180.0f));
    const float pitch_quality = Clamp01(1.0f - pitch_delta / (12.0f * kPi / 180.0f));
    const float roll_quality = Clamp01(1.0f - roll_delta / (12.0f * kPi / 180.0f));
    const float height_quality = (camera_a.camera_height_valid && camera_b.camera_height_valid)
        ? Clamp01(1.0f - out.height_delta_m / 0.20f)
        : 0.55f;
    const float scale_quality = (scale_a > 0.0f && scale_b > 0.0f)
        ? Clamp01(1.0f - std::abs(std::log(std::max(1e-5f, out.scale_ratio))) / std::log(1.25f))
        : 0.50f;
    const float base_conf = std::min(camera_a.metric_scale_confidence + camera_a.floor_plane_confidence,
        camera_b.metric_scale_confidence + camera_b.floor_plane_confidence) * 0.5f;
    out.confidence = Clamp01(base_conf * (0.25f * yaw_quality + 0.22f * pitch_quality + 0.22f * roll_quality + 0.16f * height_quality + 0.15f * scale_quality));
    out.valid = out.confidence >= 0.30f;
    out.shared_floor_frame_valid = out.valid && camera_a.homography_valid && camera_b.homography_valid;
    if (out.shared_floor_frame_valid) {
        // The floor grids are unlabeled, so translation between cameras is not solved here.
        // What we can solve honestly from passive floor geometry is the shared 2D metric
        // axis agreement: rotation/yaw plus relative scale. Full camera-to-camera translation
        // still belongs to stereo calibration or labeled correspondences.
        const float c = std::cos(yaw_delta);
        const float s = std::sin(yaw_delta);
        const float scale = (scale_a > 0.0f && scale_b > 0.0f) ? out.scale_ratio : 1.0f;
        out.floor_b_from_floor_a = {
            scale * c, -scale * s, 0.0f,
            scale * s,  scale * c, 0.0f,
            0.0f,       0.0f,      1.0f};
    }
    out.reason = out.valid ? "floor_geometry_agrees_across_cameras" : "floor_geometry_disagreement_or_insufficient_projective_evidence";
    return out;
}

} // namespace bt
