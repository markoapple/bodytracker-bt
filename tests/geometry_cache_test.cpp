#include "tracking/geometry_cache.h"
#include "tracking/epipolar_geometry.h"
#include "test_check.h"

#include <cmath>

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
    bt::CameraCalibration camera = MakeCameraA();
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

bt::StereoEpipolarConfig DefaultEpipolarConfig() {
    bt::StereoEpipolarConfig cfg;
    cfg.enabled = true;
    cfg.soft_threshold_px = 2.5f;
    cfg.hard_threshold_px = 18.0f;
    return cfg;
}

bt::StereoTriangulationConfig DefaultTriangulationConfig() {
    bt::StereoTriangulationConfig cfg;
    cfg.max_dlt_condition_number = 1000.0f;
    cfg.min_dlt_strength_ratio = 1.0e-3f;
    cfg.max_ray_closest_distance_m = 0.18f;
    cfg.min_ray_angle_deg = 0.20f;
    return cfg;
}

}  // namespace

int main() {
    // Test 1: same calibration/config reuses cache
    {
        bt::StereoGeometryCache cache;
        const auto camera_a = MakeCameraA();
        const auto camera_b = MakeCameraB();
        const auto epi_config = DefaultEpipolarConfig();
        const auto tri_config = DefaultTriangulationConfig();

        // First call — miss
        auto result1 = bt::GetOrComputeEpipolarGeometry(
            &cache, camera_a, camera_b, epi_config, tri_config);
        BT_CHECK(result1.ok());
        BT_CHECK(result1.value().valid);
        BT_CHECK(cache.GetStats().misses == 1);
        BT_CHECK(cache.GetStats().hits == 0);

        // Second call with same key — hit
        auto result2 = bt::GetOrComputeEpipolarGeometry(
            &cache, camera_a, camera_b, epi_config, tri_config);
        BT_CHECK(result2.ok());
        BT_CHECK(result2.value().valid);
        BT_CHECK(cache.GetStats().misses == 1);
        BT_CHECK(cache.GetStats().hits == 1);

        // Verify cached result matches computed result
        BT_CHECK(result1.value().fundamental_a_to_b.m ==
                 result2.value().fundamental_a_to_b.m);
    }

    // Test 2: changed intrinsics invalidates cache
    {
        bt::StereoGeometryCache cache;
        auto camera_a = MakeCameraA();
        const auto camera_b = MakeCameraB();
        const auto epi_config = DefaultEpipolarConfig();
        const auto tri_config = DefaultTriangulationConfig();

        auto result1 = bt::GetOrComputeEpipolarGeometry(
            &cache, camera_a, camera_b, epi_config, tri_config);
        BT_CHECK(result1.ok());
        BT_CHECK(cache.GetStats().hits == 0);
        BT_CHECK(cache.GetStats().misses == 1);

        // Change intrinsics
        camera_a.camera_matrix = {900.0, 0.0, 320.0,
                                  0.0, 900.0, 240.0,
                                  0.0, 0.0, 1.0};

        auto result2 = bt::GetOrComputeEpipolarGeometry(
            &cache, camera_a, camera_b, epi_config, tri_config);
        BT_CHECK(result2.ok());
        // Different intrinsics → different key → miss
        BT_CHECK(cache.GetStats().hits == 0);
        BT_CHECK(cache.GetStats().misses == 2);
    }

    // Test 3: changed extrinsics invalidates cache
    {
        bt::StereoGeometryCache cache;
        const auto camera_a = MakeCameraA();
        auto camera_b = MakeCameraB();
        const auto epi_config = DefaultEpipolarConfig();
        const auto tri_config = DefaultTriangulationConfig();

        auto result1 = bt::GetOrComputeEpipolarGeometry(
            &cache, camera_a, camera_b, epi_config, tri_config);
        BT_CHECK(result1.ok());
        BT_CHECK(cache.GetStats().misses == 1);

        // Change extrinsics (move camera B further right)
        camera_b = MakeCameraBWithBaseline(2.0f);

        auto result2 = bt::GetOrComputeEpipolarGeometry(
            &cache, camera_a, camera_b, epi_config, tri_config);
        BT_CHECK(result2.ok());
        // Different extrinsics → different key → miss
        BT_CHECK(cache.GetStats().misses == 2);
        BT_CHECK(cache.GetStats().hits == 0);

        // Baseline should be different
        BT_CHECK(result1.value().diagnostics.baseline_meters !=
                 result2.value().diagnostics.baseline_meters);
    }

    // Test 4: changed epipolar config invalidates cache
    {
        bt::StereoGeometryCache cache;
        const auto camera_a = MakeCameraA();
        const auto camera_b = MakeCameraB();
        auto epi_config = DefaultEpipolarConfig();
        const auto tri_config = DefaultTriangulationConfig();

        auto result1 = bt::GetOrComputeEpipolarGeometry(
            &cache, camera_a, camera_b, epi_config, tri_config);
        BT_CHECK(result1.ok());
        BT_CHECK(cache.GetStats().misses == 1);

        // Change config threshold
        epi_config.soft_threshold_px = 5.0f;

        auto result2 = bt::GetOrComputeEpipolarGeometry(
            &cache, camera_a, camera_b, epi_config, tri_config);
        BT_CHECK(result2.ok());
        BT_CHECK(cache.GetStats().misses == 2);
        BT_CHECK(cache.GetStats().hits == 0);
    }

    // Test 5: invalid calibration does not poison a later valid cache entry
    {
        bt::StereoGeometryCache cache;
        auto camera_a = MakeCameraA();
        const auto camera_b = MakeCameraB();
        const auto epi_config = DefaultEpipolarConfig();
        const auto tri_config = DefaultTriangulationConfig();

        // First call with invalid calibration
        camera_a.intrinsics_valid = false;
        auto result1 = bt::GetOrComputeEpipolarGeometry(
            &cache, camera_a, camera_b, epi_config, tri_config);
        BT_CHECK(!result1.ok());
        BT_CHECK(cache.GetStats().misses == 1);

        // Fix calibration — this produces a different key
        camera_a.intrinsics_valid = true;
        auto result2 = bt::GetOrComputeEpipolarGeometry(
            &cache, camera_a, camera_b, epi_config, tri_config);
        BT_CHECK(result2.ok());
        BT_CHECK(result2.value().valid);
        // Different key, so miss
        BT_CHECK(cache.GetStats().misses == 2);
        BT_CHECK(cache.GetStats().hits == 0);

        // Same valid calibration should hit
        auto result3 = bt::GetOrComputeEpipolarGeometry(
            &cache, camera_a, camera_b, epi_config, tri_config);
        BT_CHECK(result3.ok());
        BT_CHECK(cache.GetStats().hits == 1);
    }

    // Test 6: A/B directedness is correct
    {
        bt::StereoGeometryCache cache_a_b;
        bt::StereoGeometryCache cache_b_a;
        const auto camera_a = MakeCameraA();
        const auto camera_b = MakeCameraB();
        const auto epi_config = DefaultEpipolarConfig();
        const auto tri_config = DefaultTriangulationConfig();

        // Cache A→B
        auto result_ab = bt::GetOrComputeEpipolarGeometry(
            &cache_a_b, camera_a, camera_b, epi_config, tri_config);
        BT_CHECK(result_ab.ok());

        // Cache B→A (different directed key)
        auto result_ba = bt::GetOrComputeEpipolarGeometry(
            &cache_b_a, camera_b, camera_a, epi_config, tri_config);
        BT_CHECK(result_ba.ok());

        // Keys should be different (directedness)
        const auto key_ab = bt::MakeGeometryCacheKey(
            camera_a, camera_b, epi_config, tri_config);
        const auto key_ba = bt::MakeGeometryCacheKey(
            camera_b, camera_a, epi_config, tri_config);
        BT_CHECK(key_ab != key_ba);

        // Geometry should be consistent (F_a_to_b == F_b_to_a^T)
        BT_CHECK_NEAR(result_ab.value().fundamental_a_to_b.m[0],
                      result_ba.value().fundamental_b_to_a.m[0], 1e-6f);
    }

    // Test 7: cached and uncached paths produce equivalent geometry diagnostics
    {
        const auto camera_a = MakeCameraA();
        const auto camera_b = MakeCameraB();
        const auto epi_config = DefaultEpipolarConfig();
        const auto tri_config = DefaultTriangulationConfig();

        // Uncached
        auto result_uncached = bt::ComputeEpipolarGeometry(camera_a, camera_b);
        BT_CHECK(result_uncached.ok());

        // Cached
        bt::StereoGeometryCache cache;
        auto result_cached = bt::GetOrComputeEpipolarGeometry(
            &cache, camera_a, camera_b, epi_config, tri_config);
        BT_CHECK(result_cached.ok());

        // Diagnostics should match
        BT_CHECK_NEAR(result_uncached.value().diagnostics.baseline_meters,
                      result_cached.value().diagnostics.baseline_meters, 1e-6f);
        BT_CHECK_NEAR(result_uncached.value().diagnostics.camera_a_intrinsics_determinant,
                      result_cached.value().diagnostics.camera_a_intrinsics_determinant, 1e-6f);
        BT_CHECK_NEAR(result_uncached.value().diagnostics.essential_a_to_b_rank_residual,
                      result_cached.value().diagnostics.essential_a_to_b_rank_residual, 1e-6f);
    }

    // Test 8: cache telemetry distinguishes hit, miss, and invalidation
    {
        bt::StereoGeometryCache cache;
        const auto camera_a = MakeCameraA();
        const auto camera_b = MakeCameraB();
        const auto epi_config = DefaultEpipolarConfig();
        const auto tri_config = DefaultTriangulationConfig();

        // Initial miss
        auto result1 = bt::GetOrComputeEpipolarGeometry(
            &cache, camera_a, camera_b, epi_config, tri_config);
        BT_CHECK(cache.GetStats().misses == 1);
        BT_CHECK(cache.GetStats().hits == 0);

        // Cache hit
        auto result2 = bt::GetOrComputeEpipolarGeometry(
            &cache, camera_a, camera_b, epi_config, tri_config);
        BT_CHECK(cache.GetStats().misses == 1);
        BT_CHECK(cache.GetStats().hits == 1);

        // Config change causes miss (invalidation)
        auto new_epi_config = epi_config;
        new_epi_config.hard_threshold_px = 20.0f;
        auto result3 = bt::GetOrComputeEpipolarGeometry(
            &cache, camera_a, camera_b, new_epi_config, tri_config);
        BT_CHECK(cache.GetStats().misses == 2);
        BT_CHECK(cache.GetStats().hits == 1);

        // Clear cache causes invalidation
        cache.Clear();
        auto result4 = bt::GetOrComputeEpipolarGeometry(
            &cache, camera_a, camera_b, epi_config, tri_config);
        BT_CHECK(cache.GetStats().misses == 3);
        BT_CHECK(cache.GetStats().hits == 1);
        BT_CHECK(cache.GetStats().stale_invalidations == 1);
    }

    // Test 9: empty cache degrades to recomputation
    {
        bt::StereoGeometryCache cache;
        const auto camera_a = MakeCameraA();
        const auto camera_b = MakeCameraB();
        const auto epi_config = DefaultEpipolarConfig();
        const auto tri_config = DefaultTriangulationConfig();

        // Fresh cache — should compute
        auto result = bt::GetOrComputeEpipolarGeometry(
            &cache, camera_a, camera_b, epi_config, tri_config);
        BT_CHECK(result.ok());
        BT_CHECK(result.value().valid);
    }

    return 0;
}
