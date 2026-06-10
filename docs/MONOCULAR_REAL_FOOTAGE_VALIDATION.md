# Monocular real-footage validation

This project treats monocular tracking as validated only after replay logs from
real camera footage pass the scenario gates in `tools/validate_monocular_footage.py`.

## Required clips

Record at least one replay for each scenario:

- standing
- stepping
- turning
- crouching
- seated
- reclined
- partial_occlusion
- poor_lighting

Use `tracking.mode=monocular` and `tracking.enable_replay_recording=true` while
capturing. The replay log is NDJSON and should include the exported tracking
solver block.

## Manifest

Create a JSON manifest with one entry per real clip:

```json
{
  "thresholds": {
    "max_foot_step_m": 0.42,
    "max_foot_speed_mps": 4.25,
    "min_monocular_confidence": 0.08,
    "stereo_confidence_margin": 0.02
  },
  "clips": [
    {
      "name": "standing neutral",
      "scenario": "standing",
      "monocular_replay": "recordings/standing-mono.ndjson",
      "stereo_replay": "recordings/standing-stereo.ndjson"
    }
  ]
}
```

`stereo_replay` is optional per clip, but include it when the same motion was
also recorded with stereo. The validator then checks that monocular inferred
depth confidence remains below comparable triangulated stereo confidence.

## Run

```powershell
python tools/validate_monocular_footage.py diagnostics/monocular_real_footage_manifest.example.json --report diagnostics/monocular_real_footage_report.json
```

A passing report means:

- every required real-world scenario is represented;
- every monocular frame is marked `tracking_mode=monocular`;
- monocular frames expose `depth.source=inferred_monocular`;
- monocular frames do not claim stereo triangulated seeds;
- bad keypoints degrade through approved fallback modes instead of exploding;
- root and feet stay finite;
- foot step/speed limits catch visible plant/contact snapping;
- replay/debug output contains depth source, confidence, scale source, and
  floor-assist telemetry.

## Tuning from failures

When a clip fails, tune the code/config against the reported failure type:

- confidence failures: adjust monocular `depth_confidence_scale`,
  `min_keypoint_confidence`, floor-assist confidence, or keypoint reliability
  weighting.
- foot snap failures: tune support/contact transitions, EKF measurement
  variance, motion-consistency limits, or plant-entry blending.
- missing depth schema failures: fix replay/debug export before tuning motion.
- degradation failures: route invalid/occluded keypoints into predictive hold or
  untracked fallback instead of producing invalid tracker poses.

Do not mark monocular tracking as validated from synthetic tests alone. The
validator is the acceptance gate for recorded camera footage.


## Floor/plank validation notes

When validating monocular footage, separate three different things:

- physical spacing input: the user-entered real repeated floor-depth spacing in meters;
- image spacing input: detected, clicked, or typed pixel spacing between neighboring floor-depth lines;
- runtime depth effect: whether `solver.depth.scale_source` / `floor_assist` telemetry shows the saved spacing actually influenced inferred foot-contact depth.

Do not mark a clip as “plank tracking works” unless the test specifically exercises the feature being claimed. The current implementation supports known repeated floor-depth spacing and manual plank quads. It does not prove semantic automatic plank detection, diagonal seam understanding beyond the user-drawn lines, or continuous runtime seam tracking.
