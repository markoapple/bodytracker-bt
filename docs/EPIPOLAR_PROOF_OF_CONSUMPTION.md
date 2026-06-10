# Epipolar proof of consumption

This document records the Phase 6.5/6.6/6.6R shell-integration audit for stereo epipolar tracking.

The rule is: an epipolar field is not considered integrated merely because it exists in a struct, replay log, UI panel, or static string test. It must be computed, consumed by a runtime decision or confidence path, and covered by a behavioral test.

## Runtime path

| Concept | Computed in | Stored in | Consumed in | Behavioral effect | Test coverage |
| --- | --- | --- | --- | --- | --- |
| Sampson epipolar residual | `ComputeDistortionSafePixelSampsonEpipolarCheck` | `StereoJointEvidence::epipolar_error_*` | `ResolveStereoJointEvidence` | marks valid check, hard mismatch, or invalid/degenerate no-op | `epipolar_geometry_test`, `triangulation_confidence_test` |
| Distortion-safe normalized path | `UndistortPixelToNormalized` + essential matrix | `EpipolarCheck::coordinate_space` | runtime stereo evidence and identity arbitration | raw distorted pixels are scored after undistortion instead of raw one-sided line distance | `epipolar_geometry_test` |
| Fresh-pair hard mismatch | `ResolveStereoJointEvidence` | `epipolar_pair_rejected` | triangulation gate | skips stereo triangulation but keeps temporal single-camera fallback alive | `triangulation_confidence_test` |
| Degraded/reused-pair mismatch softening | `ResolveStereoJointEvidence` | `epipolar_degraded_pair_softened` | triangulation gate | hard mismatch lowers reliability but does not reject mixed-stale evidence by default | `triangulation_confidence_test` |
| Pairwise reliability term | `ResolveStereoJointEvidence` | `epipolar_reliability_term` | `StereoPairEpipolarReliabilityScale` + `StereoSeedConfidence`, called by `body_solver.cpp` | lowers triangulated stereo seed confidence; fallback evidence is not penalized | `triangulation_confidence_test` |
| Telemetry counters | `FinalizeStereoTelemetry` | `BodySolveStereoTelemetry` | debug JSON, replay JSON, UI solver panel | reports actual branch outcomes: checked, mismatch, rejected, softened | component behavior tests plus `runtime_control_wiring_test.py` smoke coverage |
| Epipolar identity arbitration | `ApplyEpipolarIdentityArbitration` | `identity_epipolar_*` telemetry | `ResolveStereoLeftRightIdentity` | only swaps a weaker camera when cross-label score beats same-label score by required margins | `identity_assignment_test` |

## Non-shell checks

The following properties are intentional and behavior-tested; static wiring checks are kept only as smoke coverage:

- Invalid/degenerate epipolar math returns no penalty instead of becoming a fake hard mismatch.
- `ResolveStereoJointEvidence` no longer multiplies epipolar confidence directly into `StereoJointEvidence::confidence`.
- The body solver applies the pairwise epipolar term through `StereoPairEpipolarReliabilityScale`, so the term affects seed weight instead of stopping at telemetry.
- Degraded or reused stereo pairs cannot be hard-rejected by epipolar alone unless explicitly configured.
- Epipolar identity arbitration is disabled for degraded/reused/skewed/duplicate stereo evidence.

## Phase 6.6/6.6R test policy

`runtime_control_wiring_test.py` is not a semantic authority. It may prove that files, targets, and runtime/debug surfaces are wired, but it must not assert exact comments, private local variable names, formatting, or the contents of C++ tests as a proxy for behavior. Real proof belongs in compiled C++ component tests that execute the geometry, triangulation, reliability, seed-confidence, and identity paths.

Current behavioral anchors:

- `epipolar_geometry_test.cpp` proves distortion-safe Sampson behavior, raw distorted-pixel degradation, invalid thresholds, near-zero/singular intrinsics rejection, and realistic forward-baseline degenerate-denominator no-op behavior. It does not use near-zero camera matrices or near-zero fundamental matrices to fake validity.
- `triangulation_confidence_test.cpp` proves soft reliability reduction, seed-confidence consumption through `StereoSeedConfidence`, fresh hard-mismatch fallback, degraded-pair softening, and explicit degraded-pair rejection when configured.
- `identity_assignment_test.cpp` proves epipolar arbitration applies only when margins pass, refuses degraded pairs, refuses weak-margin swaps, and refuses to override when both cameras are strongly guarded.

## Phase 6.6R corrections

- Removed the Python test that inspected C++ test files for semantic marker strings. That pattern tested the existence of tests, not runtime behavior.
- Tightened camera intrinsic validation so near-zero focal lengths, singular homogeneous scale, and near-singular camera matrices fail before inversion or epipolar geometry generation.
- Replaced the synthetic near-zero fundamental-matrix degeneracy test with a realistic forward-baseline camera geometry whose epipole lies at the image center; the Sampson denominator degenerates because of point geometry, not because the matrix was hacked near zero.
- Added `StereoSeedConfidence` as the production seed-confidence hook used by `body_solver.cpp`; component tests now prove epipolar reliability changes the seed confidence value consumed by the solver path.
- Telemetry truth assertions must be scoped. Forcing an epipolar branch may legitimately change triangulation counts, fallback counts, reprojection summaries, or solve residuals. Strict non-change assertions are allowed only for fields that are actually independent within the epipolar telemetry subsystem.

## Not complete yet

The remaining work is config threading, duplicate/mixed-sequence pipeline regressions, telemetry truth tests that force counters through full `TrackingPipeline`, geometry caching, and adversarial identity tests beyond the first behavioral coverage.

## Phase 6.7 API threshold contract

Epipolar threshold units are now explicit at the API boundary:

- `ComputePixelSampsonEpipolarCheck(...)` uses pixel-space points, the fundamental matrix, and pixel thresholds.
- `ComputeNormalizedSampsonEpipolarCheck(...)` uses undistorted normalized coordinates, the essential matrix, and normalized-coordinate thresholds only. It does not accept or synthesize pixel thresholds.
- `ComputeDistortionSafePixelSampsonEpipolarCheck(...)` accepts distorted pixel points, undistorts them to normalized coordinates, evaluates the essential-matrix Sampson error, then reports both normalized error and pixel-equivalent error while comparing against pixel thresholds.

This keeps the runtime distortion-safe path compatible with the existing pixel-threshold config without letting the normalized API secretly consume pixel thresholds.

