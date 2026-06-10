# Implementation Notes

This repo is a Windows-first native C++20 camera-based VR tracking MVP. It is not a finished consumer FBT product. The current archive has real runtime wiring for stereo and monocular lower-body solving plus upper-body landmark tracker output, but live quality still depends on model file availability, camera setup, calibration/scale setup, tracker-space alignment, and real footage tuning.

## Current implementation shape

The runtime path is `src/main.cpp` into `TrackingPipeline::Step()`, with mode-specific solve inputs feeding lower-body solving, upper-body landmark state synthesis, support/contact logic, motion consistency, tracker EKF smoothing, temporal update, tracker synthesis, replay/debug export, and optional OSC output.

The project supports two explicit tracking modes:

- `stereo`: two calibrated RGB cameras with triangulated depth.
- `monocular`: Camera A only, inferred depth from profile/body/floor evidence.

Stereo has a degraded Camera A fallback when `tracking.stereo_monocular_fallback_enabled=true`. That does not fake stereo depth. It temporarily sets the tracking config to `TrackingMode::Monocular` for that frame, uses Camera A only, reports a `stereo_monocular_fallback:<reason>` degradation mode, then restores stereo config.

## Floor/plank assist semantics

The floor/plank implementation is a floor-scale assist, not magic plank-object recognition. The persisted values are physical floor-depth spacing in meters, marked image spacing in pixels, optional reference image row, optional reference depth, and confidence.

This can be used for repeated tile rows, plank board pitch, rug edges, or other repeated floor-depth lines. Auto mode is backend-owned: the UI requests Camera A floor-geometry calibration, then displays and saves the returned backend evidence. The C++ estimator looks for repeated floor-seam families: many parallel seams, stable orientation, stable projective spacing, and enough contrast in the floor ROI. Multiple seams are expected. Ambiguity means the seam family is inconsistent or clutter wins. The C++ runtime consumes saved scalar spacing, projective homography, distortion correction, and camera-orientation evidence only when those fields are valid and `tracking.monocular.floor_geometry_calibration_enabled=true`.

## Runtime body calibration

`tracking.body_calibration.enabled=true` turns on neutral-stance runtime calibration. It consumes raw stereo/monocular lifted joint evidence before the IK solver clamps anatomy, so it does not re-measure its own default limb lengths. The estimator accumulates stable neutral samples, prefers triangulated stereo joints, accepts monocular floor-scale evidence at lower confidence, then persists pelvis width, left/right femur, left/right tibia, left/right foot length, standing HMD-to-pelvis offset, and per-value quality into the active calibration JSON when `auto_persist=true`. Failed completion-frame saves stay pending and are retried, and a completed calibration is serialized again on clean shutdown through `SaveCalibrationBundle()`.

The calibrated values feed `MakeLowerBodyModel()` and become hard segment constraints for the lower-body IK path. They are not updated frame-by-frame during normal tracking; only the calibration estimator mutates them after enough stable samples and quality pass configured thresholds. Debug/replay/web output exposes `body_calibration` with sample count, source, completion state, `persist_status`, `persist_error`, and the calibrated values.

## Documentation and handoff rules

Before editing code, read `docs/CODEX_FULL_KNOWLEDGE.md`. The repo has enough UI/config/runtime duplication that grep hits alone are misleading.

For any feature claim, prove this path:

```text
config/default.json
  -> src/core/config.h
  -> src/core/config.cpp
  -> src/ui/app/index.html
  -> src/ui/app/app.js
  -> src/main.cpp
  -> src/tracking/* runtime consumer
  -> debug/replay/status telemetry
  -> tests
  -> docs
```

Missing one link means the feature may be only partially wired.

## Native validation expectations

Python/static tests are useful guards, but they are not proof of full runtime correctness. Full validation means CMake configure, native build, CTest, model load, camera smoke test, calibration or monocular scale setup, replay/debug inspection, and tracker-space/OSC validation where relevant.

If a sandbox cannot run the native dependency stack, state that clearly. Do not imply compile/link/runtime correctness from static checks.

## Rejected Feature Ledger

Feature ideas rejected for being useless, wrong-scope, or fake-roadmap bait are tracked in [`REJECTED_FEATURES.md`](REJECTED_FEATURES.md). Do not resurrect them from older roadmap text unless there is a concrete runtime consumer and a proof path through config, UI, persistence, debug/replay output, and OSC behavior.

