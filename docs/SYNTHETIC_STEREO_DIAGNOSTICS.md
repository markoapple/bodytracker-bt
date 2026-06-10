# Synthetic stereo lower-body diagnostics

This repo includes a deterministic, dependency-free diagnostic generator for low-resolution
stereo lower-body tracking scenarios:

```bash
/usr/bin/python3 -S tools/synthetic_stereo_diagnostic.py \
  --out diagnostics/synthetic_stereo_example \
  --frames-per-scenario 10 \
  --seed 11
```

The generated files are intentionally inspectable and small enough to commit as a reference
sample:

- `manifest.json` lists the schema version, camera size, chessboard reference, scenarios, and intended risks.
- `synthetic_stereo_trace.ndjson` contains one JSON object per frame.
- `diagnostic_summary.json` summarizes phases, occlusion coverage, stereo reprojection/world-error metrics, disagreement, calibration imperfection, and scenario-level regression scoring.
- `camera_views.svg` shows representative low-resolution camera views with the chessboard and projected keypoints.

## What each frame contains

Each trace frame includes:

- ground-truth root and foot poses
- ground-truth support/contact phase per foot
- heel/toe anchor and contact-point positions
- HALPE-26 3D joints
- camera A/B nominal calibration
- camera A/B actual projection calibration
- projected 2D keypoints per camera
- per-keypoint confidence, visibility, noise, occlusion, and disagreement metadata
- per-keypoint camera-pair blame data: A/B visibility, confidence, nominal-stereo triangulation, reprojection error, and world error
- left/right foot summaries for ankle/heel/big-toe/small-toe triangulation confidence and error
- a visible synthetic chessboard reference
- scenario-specific failure probes

The generator is not a runtime correctness proof. It is a blame-assignment tool: a bad runtime
frame can be compared against known ground truth, projected 2D evidence, support phase, and
occlusion/disagreement conditions.


## Regression scoring

`diagnostic_summary.json` now includes a `regression` object for every scenario. The status is
one of:

- `PASS`: the synthetic ground truth and nominal-stereo diagnostic reconstruction preserve the
  scenario invariant.
- `WARN`: the trace contains ambiguous or degraded evidence that a runtime fallback must handle.
- `FAIL`: the raw diagnostic evidence violates a lower-body invariant or would expose a failure if
  runtime output followed it unmitigated.

The scorer is not a replacement for live runtime validation. It compares known synthetic ground
truth against the generated noisy/occluded nominal-stereo reconstruction and records why a frame
family is dangerous.

The main metrics are:

- `airborne_foot_path_ratio`: reconstructed free-foot path length divided by ground-truth path length.
  Low values indicate held or underpredicted swing motion.
- `airborne_foot_lag_frames`: simple best-lag estimate between ground-truth and reconstructed
  free-foot centroids.
- `planted_foot_skate_distance_m`: raw nominal-stereo planted-foot centroid drift while the foot is
  marked planted.
- `root_jitter_inherited_from_foot_only_noise_m`: planted-foot jitter risk when ground-truth root is
  stationary; this is the value a bad common-mode root correction could accidentally inherit.
- `body_over_stance_root_displacement_preservation_ratio`: whether the body-over-planted-stance
  scenario actually preserves the intended root displacement.
- `toe_pivot_toe_anchor_error_m`: toe-anchor drift during `TOE_PIVOT`.
- `heel_lock_heel_anchor_error_m`: heel-anchor drift during `HEEL_LOCK`.
- `flat_plant_anchor_error_m`: anchor/heel/toe drift during `FLAT_PLANT`.
- `slip_release_snap_back_amount_m`: backward progress toward a stale anchor after release/slip.
- `support_phase_flicker_count`: immediate A/B/A support-phase flicker count.
- `camera_disagreement_vs_3d_world_error`: high-disagreement keypoint counts, bad-3D counts,
  correlation, and whether high disagreement coincides with active support/contact risk.

Use the `reasons` array beside each status first. It is intentionally concise so a developer can
jump from a failed scenario to the relevant trace frames.

## Diagnostic scenarios

The sample covers:

- pure airborne leg swing
- body moving over planted stance foot
- planted foot jitter from 2D noise
- HeelLock
- ToePivot
- FlatPlant
- Slip/Release
- support transition / griddy-like stepping
- one-camera foot occlusion
- both-camera foot occlusion
- two-camera disagreement
- mild calibration imperfection
- low-resolution heel/toe/ankle ambiguity

## Runtime fields this is meant to exercise

When replay/runtime output is compared to the synthetic trace, the useful blame fields are:

- `tracking.stages.predicted`
- `tracking.stages.preliminary`
- `tracking.stages.support_ready`
- `tracking.stages.measured`
- `tracking.stages.motion_filtered`
- `tracking.stages.ekf_filtered`
- `tracking.stages.corrected`
- `tracking.motion_filter.contact_root`
- `tracking.support.left_foot.*_residual`
- `tracking.support.right_foot.*_residual`
- `camera_*_pose.keypoints[*].id/name/confidence/present`
- `camera_*_reliability_full.joints[*].id/name/final_weight/epipolar_term/occlusion_term`
- `tracking.solver.triangulation.preliminary.joints[*].triangulated/confidence/reprojection_error_*`
- `tracking.solver.triangulation.preliminary.left_foot_triangulated_count`
- `tracking.solver.triangulation.preliminary.right_foot_triangulated_count`

The trace intentionally includes cases where confidence remains high despite bad geometry
(`two_camera_disagreement`) and cases where phase information is ambiguous even with visible
keypoints (`low_res_heel_toe_ankle_ambiguity`).


## Synthetic vs live proof boundary

Synthetic diagnostics are useful for checking solver contracts, reprojection/error scoring, and known failure signatures. They do not replace live camera validation. A synthetic pass does not prove RTMPose robustness, exposure stability, USB timing, occlusion handling, clothing/lighting behavior, floor pattern detection, or VR tracker-space alignment.

Use synthetic tests to catch obvious regressions. Use real replay/live footage to tune tracking quality.
