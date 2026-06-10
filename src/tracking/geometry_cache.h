#pragma once

#include "calibration/calibration_types.h"
#include "core/math.h"
#include "core/status.h"
#include "tracking/epipolar_geometry.h"
#include "tracking/stereo_runtime_config.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <utility>

namespace bt {

// Phase 11 — Geometry cache for derived stereo geometry.
//
// Ownership: this is a value object with clear lifecycle, NOT a global singleton.
// The tracking pipeline owns the cache instance and controls its lifetime.
//
// Cache key includes EVERY input that can change derived geometry:
//   - camera intrinsics (fx, fy, cx, cy for both cameras)
//   - camera extrinsics (world_from_camera for both cameras)
//   - distortion coefficients (5 values for both cameras)
//   - stereo/epipolar config thresholds
//   - triangulation config values that affect derived diagnostics
//
// Swapping camera A/B produces a distinct directed key — the key encodes
// (camera_a_fields, camera_b_fields) in positional order.

struct GeometryCacheKey {
    // Camera A intrinsics
    double camera_a_fx = 0.0;
    double camera_a_fy = 0.0;
    double camera_a_cx = 0.0;
    double camera_a_cy = 0.0;
    bool camera_a_intrinsics_valid = false;
    bool camera_a_extrinsics_valid = false;
    // Camera B intrinsics
    double camera_b_fx = 0.0;
    double camera_b_fy = 0.0;
    double camera_b_cx = 0.0;
    double camera_b_cy = 0.0;
    bool camera_b_intrinsics_valid = false;
    bool camera_b_extrinsics_valid = false;
    // Camera A extrinsics (world_from_camera, 12 elements flattened)
    std::array<double, 12> camera_a_world_from_camera{};
    // Camera B extrinsics (world_from_camera, 12 elements flattened)
    std::array<double, 12> camera_b_world_from_camera{};
    // Camera A distortion coefficients
    std::array<double, 5> camera_a_distortion{};
    // Camera B distortion coefficients
    std::array<double, 5> camera_b_distortion{};
    // Epipolar config thresholds (affect derived geometry diagnostics)
    float epipolar_soft_threshold_px = 0.0f;
    float epipolar_hard_threshold_px = 0.0f;
    // Triangulation config values that affect derived diagnostics
    float triangulation_max_dlt_condition = 0.0f;
    float triangulation_min_dlt_strength = 0.0f;
    float max_ray_closest_distance_m = 0.0f;
    float min_ray_angle_deg = 0.0f;

    [[nodiscard]] bool operator==(const GeometryCacheKey& other) const {
        return camera_a_fx == other.camera_a_fx &&
               camera_a_fy == other.camera_a_fy &&
               camera_a_cx == other.camera_a_cx &&
               camera_a_cy == other.camera_a_cy &&
               camera_a_intrinsics_valid == other.camera_a_intrinsics_valid &&
               camera_a_extrinsics_valid == other.camera_a_extrinsics_valid &&
               camera_b_fx == other.camera_b_fx &&
               camera_b_fy == other.camera_b_fy &&
               camera_b_cx == other.camera_b_cx &&
               camera_b_cy == other.camera_b_cy &&
               camera_b_intrinsics_valid == other.camera_b_intrinsics_valid &&
               camera_b_extrinsics_valid == other.camera_b_extrinsics_valid &&
               camera_a_world_from_camera == other.camera_a_world_from_camera &&
               camera_b_world_from_camera == other.camera_b_world_from_camera &&
               camera_a_distortion == other.camera_a_distortion &&
               camera_b_distortion == other.camera_b_distortion &&
               epipolar_soft_threshold_px == other.epipolar_soft_threshold_px &&
               epipolar_hard_threshold_px == other.epipolar_hard_threshold_px &&
               triangulation_max_dlt_condition == other.triangulation_max_dlt_condition &&
               triangulation_min_dlt_strength == other.triangulation_min_dlt_strength &&
               max_ray_closest_distance_m == other.max_ray_closest_distance_m &&
               min_ray_angle_deg == other.min_ray_angle_deg;
    }
    [[nodiscard]] bool operator!=(const GeometryCacheKey& other) const {
        return !(*this == other);
    }
};

struct GeometryCacheKeyHash {
    [[nodiscard]] std::size_t operator()(const GeometryCacheKey& key) const noexcept {
        std::size_t h = 0x9e3779b97f4a7c15ULL;
        auto combine = [&h](double v) {
            std::size_t bits = 0;
            std::memcpy(&bits, &v, sizeof(double));
            h ^= bits + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
        };
        auto combinef = [&h](float v) {
            std::size_t bits = 0;
            std::memcpy(&bits, &v, sizeof(float));
            h ^= bits + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
        };
        combine(key.camera_a_fx);
        combine(key.camera_a_fy);
        combine(key.camera_a_cx);
        combine(key.camera_a_cy);
        h ^= static_cast<std::size_t>(key.camera_a_intrinsics_valid ? 0x51f15e1u : 0u) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
        h ^= static_cast<std::size_t>(key.camera_a_extrinsics_valid ? 0xe871a5u : 0u) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
        combine(key.camera_b_fx);
        combine(key.camera_b_fy);
        combine(key.camera_b_cx);
        combine(key.camera_b_cy);
        h ^= static_cast<std::size_t>(key.camera_b_intrinsics_valid ? 0x51f15e1u : 0u) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
        h ^= static_cast<std::size_t>(key.camera_b_extrinsics_valid ? 0xe871a5u : 0u) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
        for (double v : key.camera_a_world_from_camera) combine(v);
        for (double v : key.camera_b_world_from_camera) combine(v);
        for (double v : key.camera_a_distortion) combine(v);
        for (double v : key.camera_b_distortion) combine(v);
        combinef(key.epipolar_soft_threshold_px);
        combinef(key.epipolar_hard_threshold_px);
        combinef(key.triangulation_max_dlt_condition);
        combinef(key.triangulation_min_dlt_strength);
        combinef(key.max_ray_closest_distance_m);
        combinef(key.min_ray_angle_deg);
        return h;
    }
};

struct GeometryCacheEntry {
    Result<EpipolarGeometry> geometry_result;
    EpipolarGeometryDiagnostics diagnostics{};
    bool geometry_computed_successfully = false;
    bool diagnostics_valid = false;
};

struct GeometryCacheStats {
    int hits = 0;
    int misses = 0;
    int stale_invalidations = 0;
    int cache_errors = 0;
};

class StereoGeometryCache {
public:
    StereoGeometryCache() = default;

    // Clear the entire cache (e.g. on calibration reload).
    void Clear() {
        current_key_.reset();
        current_entry_.reset();
        stats_.stale_invalidations++;
    }

    // Try to get cached geometry. Returns nullopt on miss or stale key.
    [[nodiscard]] std::optional<const GeometryCacheEntry*> Get(
        const GeometryCacheKey& key) const {
        if (!current_key_.has_value() || current_key_.value() != key) {
            return std::nullopt;
        }
        if (!current_entry_.has_value()) {
            return std::nullopt;
        }
        return &current_entry_.value();
    }

    // Store geometry in cache for a key. Replaces any existing entry.
    void Put(const GeometryCacheKey& key, GeometryCacheEntry entry) {
        current_key_ = key;
        current_entry_ = std::move(entry);
    }

    void RecordMiss() { stats_.misses++; }
    void RecordHit() { stats_.hits++; }
    void RecordStaleInvalidation() { stats_.stale_invalidations++; }
    void RecordCacheError() { stats_.cache_errors++; }

    [[nodiscard]] const GeometryCacheStats& GetStats() const { return stats_; }
    void ResetStats() { stats_ = {}; }

private:
    std::optional<GeometryCacheKey> current_key_;
    std::optional<GeometryCacheEntry> current_entry_;
    GeometryCacheStats stats_{};
};

// Build a cache key from calibration and config. Extracts exactly the fields
// that can change derived geometry, ensuring config/calibration changes
// produce different keys. Swapping camera_a/camera_b produces a different key
// because the fields are positionally distinct.
[[nodiscard]] inline GeometryCacheKey MakeGeometryCacheKey(
    const CameraCalibration& camera_a,
    const CameraCalibration& camera_b,
    const StereoEpipolarConfig& epipolar_config,
    const StereoTriangulationConfig& triangulation_config) {
    GeometryCacheKey key;
    key.camera_a_fx = camera_a.camera_matrix[0];
    key.camera_a_fy = camera_a.camera_matrix[4];
    key.camera_a_cx = camera_a.camera_matrix[2];
    key.camera_a_cy = camera_a.camera_matrix[5];
    key.camera_a_intrinsics_valid = camera_a.intrinsics_valid;
    key.camera_a_extrinsics_valid = camera_a.extrinsics_valid;
    key.camera_b_fx = camera_b.camera_matrix[0];
    key.camera_b_fy = camera_b.camera_matrix[4];
    key.camera_b_cx = camera_b.camera_matrix[2];
    key.camera_b_cy = camera_b.camera_matrix[5];
    key.camera_b_intrinsics_valid = camera_b.intrinsics_valid;
    key.camera_b_extrinsics_valid = camera_b.extrinsics_valid;

    for (int i = 0; i < 12; ++i) {
        key.camera_a_world_from_camera[static_cast<size_t>(i)] =
            static_cast<double>(camera_a.world_from_camera.m[static_cast<size_t>(i)]);
        key.camera_b_world_from_camera[static_cast<size_t>(i)] =
            static_cast<double>(camera_b.world_from_camera.m[static_cast<size_t>(i)]);
    }
    for (int i = 0; i < 5; ++i) {
        key.camera_a_distortion[static_cast<size_t>(i)] = camera_a.distortion[i];
        key.camera_b_distortion[static_cast<size_t>(i)] = camera_b.distortion[i];
    }

    key.epipolar_soft_threshold_px = epipolar_config.soft_threshold_px;
    key.epipolar_hard_threshold_px = epipolar_config.hard_threshold_px;
    key.triangulation_max_dlt_condition = triangulation_config.max_dlt_condition_number;
    key.triangulation_min_dlt_strength = triangulation_config.min_dlt_strength_ratio;
    key.max_ray_closest_distance_m = triangulation_config.max_ray_closest_distance_m;
    key.min_ray_angle_deg = triangulation_config.min_ray_angle_deg;

    return key;
}

// Compute or retrieve cached epipolar geometry.
// This is the primary entry point for geometry cache consumption.
[[nodiscard]] inline Result<EpipolarGeometry> GetOrComputeEpipolarGeometry(
    StereoGeometryCache* cache,
    const CameraCalibration& camera_a,
    const CameraCalibration& camera_b,
    const StereoEpipolarConfig& epipolar_config,
    const StereoTriangulationConfig& triangulation_config) {
    const GeometryCacheKey key = MakeGeometryCacheKey(
        camera_a, camera_b, epipolar_config, triangulation_config);

    if (cache) {
        const auto cached = cache->Get(key);
        if (cached.has_value()) {
            cache->RecordHit();
            const auto& entry = *cached.value();
            if (entry.geometry_computed_successfully && entry.geometry_result.ok()) {
                return entry.geometry_result.value();
            }
            // Return cached error result — don't recompute failed geometry,
            // but also don't poison later valid entries (key changes on
            // calibration/config change will invalidate this entry).
            return entry.geometry_result.status();
        }
        cache->RecordMiss();
    }

    // Compute fresh geometry
    auto result = ComputeEpipolarGeometry(camera_a, camera_b);

    if (cache) {
        GeometryCacheEntry entry{result};
        entry.geometry_computed_successfully = result.ok();
        if (result.ok()) {
            entry.diagnostics = result.value().diagnostics;
            entry.diagnostics_valid = true;
        }
        cache->Put(key, std::move(entry));
    }

    return result;
}

// Compute or retrieve cached diagnostics independently.
[[nodiscard]] inline EpipolarGeometryDiagnostics GetOrComputeDiagnostics(
    StereoGeometryCache* cache,
    const CameraCalibration& camera_a,
    const CameraCalibration& camera_b) {
    // Config doesn't affect diagnostics, so use empty config for key.
    const GeometryCacheKey key = MakeGeometryCacheKey(
        camera_a, camera_b, StereoEpipolarConfig{}, StereoTriangulationConfig{});

    if (cache) {
        const auto cached = cache->Get(key);
        if (cached.has_value() && (*cached)->diagnostics_valid) {
            cache->RecordHit();
            return (*cached)->diagnostics;
        }
        if (cached.has_value()) {
            cache->RecordCacheError();
        } else {
            cache->RecordMiss();
        }
    }

    auto result = ComputeEpipolarGeometry(camera_a, camera_b);
    EpipolarGeometryDiagnostics diagnostics{};
    if (result.ok()) {
        diagnostics = result.value().diagnostics;
        if (cache) {
            GeometryCacheEntry entry{result};
            entry.geometry_computed_successfully = true;
            entry.diagnostics = diagnostics;
            entry.diagnostics_valid = true;
            cache->Put(key, std::move(entry));
        }
    } else if (cache) {
        cache->RecordCacheError();
    }
    return diagnostics;
}

}  // namespace bt
