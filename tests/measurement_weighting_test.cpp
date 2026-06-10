#include "tracking/measurement_weighting.h"
#include "test_check.h"

#include <cmath>

int main() {
    const bt::SolverObservationWeightingConfig default_weighting{};
    const bt::SolverMeasurementUncertainty precise{
        true,
        bt::Vec3f{0.0f, 0.0f, 1.0f},
        0.025f,
        0.50f,
        0.0f
    };
    const auto info = bt::SolverObservationInformationFromUncertainty(precise);
    BT_CHECK(info.uncertainty_valid);
    BT_CHECK(!info.conservative_fallback);
    BT_CHECK(info.lateral_weight_scale > info.depth_weight_scale);

    const float same_lateral_m = bt::SolverAnisotropicSquaredResidual(
        bt::Vec3f{0.10f, 0.0f, 0.0f},
        bt::Vec3f{0.0f, 0.0f, 0.0f},
        info);
    const float same_depth_m = bt::SolverAnisotropicSquaredResidual(
        bt::Vec3f{0.0f, 0.0f, 0.10f},
        bt::Vec3f{0.0f, 0.0f, 0.0f},
        info);
    BT_CHECK(same_lateral_m > same_depth_m);


    const auto off_axis_depth = bt::SolverObservationDepthAxisFromOrigin(
        bt::Vec3f{2.0f, 0.0f, 0.0f},
        bt::Vec3f{5.0f, 0.0f, 0.0f});
    BT_CHECK_NEAR(off_axis_depth.x, 1.0f, 1e-6f);
    BT_CHECK_NEAR(off_axis_depth.y, 0.0f, 1e-6f);
    BT_CHECK_NEAR(off_axis_depth.z, 0.0f, 1e-6f);

    const auto degenerate_depth_fallback = bt::SolverObservationDepthAxisFromOrigin(
        bt::Vec3f{1.0f, 2.0f, 3.0f},
        bt::Vec3f{1.0f, 2.0f, 3.0f},
        bt::Vec3f{0.0f, 0.0f, 5.0f});
    BT_CHECK_NEAR(degenerate_depth_fallback.x, 0.0f, 1e-6f);
    BT_CHECK_NEAR(degenerate_depth_fallback.y, 0.0f, 1e-6f);
    BT_CHECK_NEAR(degenerate_depth_fallback.z, 1.0f, 1e-6f);
    BT_CHECK_NEAR(bt::Length(degenerate_depth_fallback), 1.0f, 1e-6f);

    const bt::SolverMeasurementUncertainty off_axis_precise{
        true,
        off_axis_depth,
        0.025f,
        0.50f,
        0.0f
    };
    const auto off_axis_info = bt::SolverObservationInformationFromUncertainty(off_axis_precise);
    const float off_axis_lateral_m = bt::SolverAnisotropicSquaredResidual(
        bt::Vec3f{0.0f, 0.0f, 0.10f},
        bt::Vec3f{0.0f, 0.0f, 0.0f},
        off_axis_info);
    const float off_axis_depth_m = bt::SolverAnisotropicSquaredResidual(
        bt::Vec3f{0.10f, 0.0f, 0.0f},
        bt::Vec3f{0.0f, 0.0f, 0.0f},
        off_axis_info);
    BT_CHECK(off_axis_lateral_m > off_axis_depth_m);

    const bt::SolverMeasurementUncertainty missing{};
    const auto fallback = bt::SolverObservationInformationFromUncertainty(missing);
    BT_CHECK(!fallback.uncertainty_valid);
    BT_CHECK(fallback.conservative_fallback);
    BT_CHECK(fallback.lateral_weight_scale > 0.0f);
    BT_CHECK(fallback.depth_weight_scale > 0.0f);
    BT_CHECK(fallback.lateral_weight_scale < info.lateral_weight_scale);
    BT_CHECK(fallback.depth_weight_scale < info.depth_weight_scale);

    BT_CHECK(bt::SolverAxisWeightScaleFromStddev(1000.0f) <
             bt::SolverAxisWeightScaleFromStddev(100.0f));

    const bt::SolverMeasurementUncertainty zero_variance{
        true,
        bt::Vec3f{0.0f, 0.0f, 1.0f},
        0.0f,
        0.0f,
        0.0f
    };
    const auto zero_fallback = bt::SolverObservationInformationFromUncertainty(zero_variance);
    BT_CHECK(!zero_fallback.uncertainty_valid);
    BT_CHECK(zero_fallback.conservative_fallback);
    BT_CHECK(std::isfinite(zero_fallback.lateral_weight_scale));
    BT_CHECK(std::isfinite(zero_fallback.depth_weight_scale));

    const bt::SolverMeasurementUncertainty near_floor{
        true,
        bt::Vec3f{0.0f, 0.0f, 1.0f},
        0.001f,
        0.001f,
        0.0f
    };
    const auto capped = bt::SolverObservationInformationFromUncertainty(near_floor);
    BT_CHECK(capped.uncertainty_valid);
    BT_CHECK(capped.lateral_weight_scale <= default_weighting.max_weight_scale);
    BT_CHECK(capped.depth_weight_scale <= default_weighting.max_weight_scale);

    const bt::SolverMeasurementUncertainty very_bad{
        true,
        bt::Vec3f{0.0f, 0.0f, 1.0f},
        10.0f,
        20.0f,
        0.0f
    };
    const auto weak = bt::SolverObservationInformationFromUncertainty(very_bad);
    BT_CHECK(weak.uncertainty_valid);
    BT_CHECK(weak.lateral_weight_scale < fallback.lateral_weight_scale);
    BT_CHECK(weak.depth_weight_scale < fallback.depth_weight_scale);


    BT_CHECK_NEAR(
        bt::SolverTemporalProcessStddevM(0.25f) * bt::SolverTemporalProcessStddevM(0.25f),
        default_weighting.temporal_process_variance_m2_per_s * 0.25f,
        1e-6f);
    BT_CHECK(bt::SolverTemporalProcessStddevM(0.50f) >
             bt::SolverTemporalProcessStddevM(0.25f));

    const bt::SolverMeasurementUncertainty fresh_for_process{
        true,
        bt::Vec3f{0.0f, 0.0f, 1.0f},
        0.050f,
        0.050f,
        0.0f
    };
    auto stale_for_process = fresh_for_process;
    stale_for_process.temporal_process_stddev_m = 0.50f;
    const auto fresh_process_info = bt::SolverObservationInformationFromUncertainty(fresh_for_process);
    const auto stale_process_info = bt::SolverObservationInformationFromUncertainty(stale_for_process);
    BT_CHECK(!fresh_process_info.temporal_process_noise_applied);
    BT_CHECK(stale_process_info.temporal_process_noise_applied);
    BT_CHECK(stale_process_info.lateral_weight_scale < fresh_process_info.lateral_weight_scale);
    BT_CHECK(stale_process_info.depth_weight_scale < fresh_process_info.depth_weight_scale);

    const bt::SolverMeasurementUncertainty stale_missing{
        false,
        bt::Vec3f{0.0f, 0.0f, 1.0f},
        0.0f,
        0.0f,
        0.50f
    };
    const auto stale_missing_info = bt::SolverObservationInformationFromUncertainty(stale_missing);
    BT_CHECK(stale_missing_info.conservative_fallback);
    BT_CHECK(stale_missing_info.temporal_process_noise_applied);
    BT_CHECK(stale_missing_info.lateral_weight_scale < fallback.lateral_weight_scale);


    const bt::SolverMeasurementUncertainty mixed_fresh_conditioned{
        true,
        bt::Vec3f{0.0f, 0.0f, 1.0f},
        0.020f,
        0.250f,
        0.0f
    };
    const auto mixed_fresh_info = bt::SolverObservationInformationFromUncertainty(mixed_fresh_conditioned);
    auto mixed_stale_conditioned = mixed_fresh_conditioned;
    mixed_stale_conditioned.temporal_process_stddev_m = bt::SolverTemporalProcessStddevM(0.10f);
    const auto mixed_stale_info = bt::SolverObservationInformationFromUncertainty(mixed_stale_conditioned);
    BT_CHECK(mixed_stale_info.uncertainty_valid);
    BT_CHECK(!mixed_stale_info.conservative_fallback);
    BT_CHECK(mixed_stale_info.temporal_process_noise_applied);
    BT_CHECK(mixed_stale_info.lateral_weight_scale < mixed_fresh_info.lateral_weight_scale);
    BT_CHECK(mixed_stale_info.depth_weight_scale < mixed_fresh_info.depth_weight_scale);
    BT_CHECK(mixed_stale_info.lateral_weight_scale > mixed_stale_info.depth_weight_scale);

    auto older_mixed_stale = mixed_stale_conditioned;
    older_mixed_stale.temporal_process_stddev_m = bt::SolverTemporalProcessStddevM(0.40f);
    const auto older_mixed_stale_info = bt::SolverObservationInformationFromUncertainty(older_mixed_stale);
    BT_CHECK(older_mixed_stale_info.temporal_process_stddev_m > mixed_stale_info.temporal_process_stddev_m);
    BT_CHECK(older_mixed_stale_info.lateral_weight_scale < mixed_stale_info.lateral_weight_scale);
    BT_CHECK(older_mixed_stale_info.depth_weight_scale < mixed_stale_info.depth_weight_scale);

    auto process_dominated_conditioned = mixed_fresh_conditioned;
    process_dominated_conditioned.temporal_process_stddev_m = bt::SolverTemporalProcessStddevM(20.0f);
    const auto process_dominated_info = bt::SolverObservationInformationFromUncertainty(process_dominated_conditioned);
    const float fresh_anisotropic_weight_ratio =
        mixed_fresh_info.lateral_weight_scale / mixed_fresh_info.depth_weight_scale;
    const float process_dominated_weight_ratio =
        process_dominated_info.lateral_weight_scale / process_dominated_info.depth_weight_scale;
    BT_CHECK(process_dominated_info.temporal_process_noise_applied);
    BT_CHECK(process_dominated_info.lateral_weight_scale < older_mixed_stale_info.lateral_weight_scale);
    BT_CHECK(process_dominated_info.depth_weight_scale < older_mixed_stale_info.depth_weight_scale);
    BT_CHECK(process_dominated_weight_ratio < fresh_anisotropic_weight_ratio);
    BT_CHECK(process_dominated_weight_ratio < 2.0f);

    const bt::SolverMeasurementUncertainty mixed_stale_missing{
        false,
        bt::Vec3f{0.0f, 0.0f, 1.0f},
        0.0f,
        0.0f,
        bt::SolverTemporalProcessStddevM(0.40f)
    };
    const auto mixed_stale_missing_info = bt::SolverObservationInformationFromUncertainty(mixed_stale_missing);
    BT_CHECK(!mixed_stale_missing_info.uncertainty_valid);
    BT_CHECK(mixed_stale_missing_info.conservative_fallback);
    BT_CHECK(mixed_stale_missing_info.temporal_process_noise_applied);
    BT_CHECK(mixed_stale_missing_info.lateral_weight_scale < fallback.lateral_weight_scale);
    BT_CHECK(mixed_stale_missing_info.depth_weight_scale < fallback.depth_weight_scale);

    BT_CHECK_NEAR(bt::SolverObservationConfidenceCeilingFromWeightScale(10.0f), 1.0f, 1e-6f);
    BT_CHECK_NEAR(bt::SolverObservationConfidenceCeilingFromWeightScale(0.25f), 0.85f, 1e-6f);
    BT_CHECK_NEAR(bt::SolverObservationConfidenceCeilingFromWeightScale(0.05f), 0.70f, 1e-6f);
    BT_CHECK_NEAR(bt::SolverObservationConfidenceCeilingFromWeightScale(0.0025f), 0.55f, 1e-6f);
    BT_CHECK_NEAR(bt::SolverObservationConfidenceCeiling(fallback), 0.55f, 1e-6f);
    BT_CHECK(bt::SolverObservationConfidenceCeiling(capped) > bt::SolverObservationConfidenceCeiling(fallback));

    return 0;
}
