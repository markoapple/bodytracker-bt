#include "core/profiler.h"
#include "test_check.h"

int main() {
    bt::Profiler profiler;
    bt::ProfilerBudget budget;
    budget.total_ms = 10.0;
    budget.pipeline_ms = 4.0;
    budget.inference_ms = 2.0;
    profiler.SetBudget(budget);

    for (int i = 0; i < 32; ++i) {
        bt::ProfilerFrameStats sample;
        sample.total_ms = 8.0;
        sample.pipeline_ms = 3.0;
        sample.inference_ms = 1.5;
        profiler.Observe(sample);
    }
    auto snap = profiler.Snapshot();
    BT_CHECK(snap.sample_count == 32);
    BT_CHECK(!snap.any_budget_exceeded);
    BT_CHECK(snap.bottleneck_stage == "none");

    for (int i = 0; i < 32; ++i) {
        bt::ProfilerFrameStats sample;
        sample.total_ms = 18.0;
        sample.pipeline_ms = 7.0;
        sample.inference_ms = 5.0;
        profiler.Observe(sample);
    }
    snap = profiler.Snapshot();
    BT_CHECK(snap.sample_count == 64);
    BT_CHECK(snap.any_budget_exceeded);
    BT_CHECK(snap.total.over_budget);
    BT_CHECK(snap.pipeline.over_budget);
    BT_CHECK(snap.inference.over_budget);
    BT_CHECK(snap.bottleneck_stage == "inference");
    BT_CHECK(snap.bottleneck_ratio > 2.0);
    return 0;
}
