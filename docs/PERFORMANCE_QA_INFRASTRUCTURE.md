# Performance and QA Infrastructure

This project now has concrete performance/QA plumbing instead of roadmap prose.

## Runtime profiler dashboard

`src/core/profiler.h` provides a dependency-free rolling profiler with a 240-frame window. Runtime publishes its rolling snapshot through `DebugSnapshot.profiler`, `DebugToJson()`, and the desktop UI diagnostics panel.

The profiler tracks last/average/p95/max plus budget status for:

- total frame time
- capture
- frame pairing
- preprocessing
- inference
- ONNX execution
- decode
- tracking pipeline
- solver
- OSC
- UI publish

The UI shows sample count, bottleneck stage, bottleneck ratio, and p95/budget pairs. This makes a slow frame visible in the same runtime debug path as solver/OSC/camera state.

## Performance budget gate

`qa/performance/default_budget.json` defines the source-controlled budget for replay benchmarks. `tools/check_performance_budget.py` reads the JSON emitted by `bodytracker --benchmark-replay` and fails if any metric crosses its bound.

Example:

```bash
bodytracker --benchmark-replay calib/default.json diagnostics/synthetic_stereo_example/synthetic_stereo_trace.ndjson 3 > benchmark.json
python3 tools/check_performance_budget.py benchmark.json qa/performance/default_budget.json --summary perf_summary.json
```

Budget keys use dotted JSON paths. Suffix `.min` means a lower bound; other values are upper bounds.

## Fuzz/property checks

`tests/reliability_fuzz_test.cpp` runs deterministic pseudo-random pose/ROI inputs through `ComputeViewReliability()` and verifies invariants: finite values, clamped confidence ranges, and no bogus temporal scaffold evidence escaping as invalid weights.

The fuzz test is deterministic on purpose. CI failures should be reproducible without saving a random seed from a runner log.

## CI gates

`.github/workflows/ci.yml` keeps the useful checks:

1. `qa-static-contracts`: validates JSON contracts and exercises the performance-budget tool.
2. `windows-build-test`: keeps the Windows/vcpkg app build and release tests.

## What this still does not pretend

These checks do not prove VRChat quality or live camera realism by themselves. They make regressions visible and repeatable. Real tracking quality still needs replay inspection and live bringup on the target machine.
