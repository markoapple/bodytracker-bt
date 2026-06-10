# Config Contracts

Config validation preserves useful imperfect state. It blocks only invalid,
non-finite, or structurally unusable values. Recoverable setup gaps load as
degraded, missing-but-defaultable, or warning issues so the app can launch and
show the user what to fix.

## Validation Outcomes

| Outcome | Meaning | Runtime behavior |
| --- | --- | --- |
| `invalid` | Non-finite, impossible, or structurally unusable value | Config load/save must fail |
| `degraded` | Finite config that cannot drive a specific output yet | Config loads; affected output blocks or lowers confidence with reason |
| `missing-but-defaultable` | Optional setup data absent or incomplete | Config loads; runtime uses fallback/default path |
| `warning` | Drift-prone or unusual but usable value | Config loads; UI/status should show warning text |

OSC with `osc.enabled=true` and no finite tracker-space transform is
`degraded`, not `invalid`. The config must load so the user can calibrate.
Runtime OSC remains blocked until `osc.tracker_space_*` or
`osc.manual_tracker_space_*` contains a finite transform with positive scale
and non-zero quaternion.

## Shared Constants

Shared tracking thresholds live in
`src/tracking/tracking_constants.h`. Consumers should include that header
instead of copying literals.

| Constant | Meaning | Current default |
| --- | --- | --- |
| `kDefaultFootLengthM` | Uncalibrated foot length fallback | `0.24` |
| `kReprojectionErrorMaxPx` | Solver/body-state reprojection quality scale | `45.0` |
| `kReliabilityUsableThreshold` | Reliability term usable floor | `0.15` |
| `kReliabilityMinPresentConfidence` | Minimum present keypoint confidence | `0.05` |
| `kMonocularMinDepthM` / `kMonocularMaxDepthM` | Monocular depth clamp | `0.30` / `8.00` |
| `kMonocularMinFovDeg` / `kMonocularMaxFovDeg` | Monocular FOV clamp | `30.0` / `130.0` |
| `kMonocularMinBodyPixelHeight` | Body extent depth minimum | `40.0` |
| `kFootMinContactConfidence` | Foot contact confidence floor | `0.35` |
| `kKneeMinContactConfidence` | Knee contact confidence floor | `0.42` |
| `kVisibleConfidenceThreshold` | Visible/OSC confidence threshold | `0.20` |
| `kIdentityStableThreshold` | Stable identity threshold | `0.55` |
| `kIdentityUncertainThreshold` | Uncertain identity threshold | `0.35` |

`config/bodytracker-config.schema.json` is now part of load-time validation.
`LoadConfig()` first validates the raw JSON shape/types/ranges against the
schema, then builds `AppConfig`, then runs the C++ semantic checks that need
cross-field or runtime-aware logic. The C++ checks remain authoritative for
recoverable setup gaps: a finite-but-incomplete setup still loads as degraded,
missing-but-defaultable, or warning instead of being killed by the schema.

## Load-Time Validation Order

1. Parse JSON. Syntax errors fail immediately.
2. Validate the raw JSON against `bodytracker-config.schema.json`. This catches
   wrong types, enum drift, impossible scalar ranges, and malformed arrays before
   ad-hoc readers touch them.
3. Read canonical/legacy fields into `AppConfig`. Compatibility aliases still
   work here.
4. Run semantic validation on the assembled config. This is where cross-field
   rules live, such as unique OSC tracker indices, non-singular enabled
   homographies, and stereo camera source conflicts.
5. Emit degraded/missing/warning issues for usable but incomplete setup data.

## Migration Table

| Old field or literal | Current owner | Migration behavior |
| --- | --- | --- |
| `tracking.max_frame_skew_ms` as live pair tolerance | `tracking.latest_frame_skew_tolerance_ms` | If the new field is absent, `max_frame_skew_ms` seeds it for compatibility |
| `inference.model_path` | `tracking.model_path` | Used only when `tracking.model_path` is absent |
| Top-level `calibration_path` | `tracking.calibration_path` | Used only when `tracking.calibration_path` is absent |
| Active manual `osc.tracker_space_*` without manual fallback | `osc.manual_tracker_space_*` | Load preserves valid manual/json active transform into fallback fields |
| Local `0.24f` foot fallback | `tracking_constants::kDefaultFootLengthM` | Use the shared constant unless a calibrated body model supplies foot length |
| Local `45.0f` reprojection scale | `tracking_constants::kReprojectionErrorMaxPx` | Use the shared constant for solver defaults and body-state quality |
