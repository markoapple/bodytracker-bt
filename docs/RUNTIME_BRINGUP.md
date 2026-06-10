# Live Bring-Up Guide

This guide is the practical path for taking this repo from checkout to a first local live tracking session. It documents the current runtime controls and debug cards; it does **not** claim the full live runtime has already been validated on your machine.


## Claim/validation matrix

Use this matrix when deciding whether a feature is actually working, not merely present in strings.

| Claim | Minimum proof |
| --- | --- |
| Manual floor/plank scale assist exists | UI fields exist, values save/load through config, C++ monocular projection consumes them, telemetry reports floor-spacing status/source, tests cover config and runtime depth effect. |
| Manual plank calibration exists | Camera A preview lets the user draw one plank outline: two long edges, one short end cap, and optionally the opposite end cap; real plank dimensions are optional, and no automatic floorboard detection is run. |
| Manual override exists | After manual plank-line calibration |
| Stereo fallback uses monocular features | The fallback branch sets `TrackingMode::Monocular`, calls the same pipeline path, reports `stereo_monocular_fallback:<reason>`, restores stereo params, and lowers/degrades depth confidence. |
| Normal stereo remains triangulated | In calibrated stereo frames, depth comes from stereo triangulation and floor assist remains disabled/standby rather than being reported as the stereo source. |
| OSC tracker-space gating exists | Active tracker-space output requires finite, configured transform values. A stale SteamVR/controller provider is age/freshness evidence only; it must not erase, suppress, or disqualify an already-solved finite transform. Raw camera-world trackers are not sent directly; tracker-space output must come from a finite active transform or a finite fallback transform. |

If an agent is taking over, make it read [`AGENTS.md`](../AGENTS.md) and [`docs/CODEX_FULL_KNOWLEDGE.md`](CODEX_FULL_KNOWLEDGE.md) before changing runtime files.

## Prerequisites

Use the intended Windows + vcpkg machine for real bring-up.

- Windows desktop machine with the repo cloned locally.
- CMake presets from this repo and vcpkg dependencies installed by the normal project flow.
- Stereo mode: two fixed RGB cameras that can see the lower body at the same time, plus a printed chessboard for calibration.
- Monocular mode: one fixed RGB camera with visible lower body, user/camera scale inputs, and optional known tile/plank dimensions for floor-scale assist. No chessboard or Camera B is required for monocular bring-up.

Stereo mode also has an explicit `tracking.stereo_monocular_fallback_enabled` fallback path. When it is enabled, the runtime still prefers calibrated stereo, but if Camera B cannot start, frame pairing fails, stereo calibration is not ready, or Camera B pose inference fails, Camera A can be solved through the monocular path with lower inferred-depth confidence instead of stopping tracking entirely.
- The RTMW-DW-X-L Cocktail14 384x288 ONNX model at the primary tracking path.
- Optional RTMW3D-X Cocktail14 384x288 ONNX model for model-z experiments. Live VRChat transfer uses calculated 3D from the Cocktail 2D solver path by default.
- Optional but recommended: VRChat/SteamVR/OSC receiver ready only after local tracking looks sane, tracker-space alignment is valid, OpenVR support is actually built when using controller alignment, and VRChat tracker indices are intentionally bound.

Build commands are documented in [`docs/BUILD_ENVIRONMENT.md`](BUILD_ENVIRONMENT.md). Do not debug tracking quality until the intended local build succeeds there.


## Preflight doctor

Before the first live launch, run the cheap preflight doctor from the repo root:

```powershell
python tools\live_preflight_doctor.py --repo-root .
```

Machine-readable output:

```powershell
python tools\live_preflight_doctor.py --repo-root . --json
```

The doctor does **not** build the app, install vcpkg packages, load ONNX Runtime, or open cameras. It checks whether the operator-facing setup is ready enough to attempt live bring-up: model file presence and SHA, config parsing, mode-specific camera/calibration requirements, replay recording path, `VCPKG_ROOT`, stale old-model labels, and debug/replay settings. In `tracking.mode=stereo`, Camera A, Camera B, stereo calibration, and different camera indices are required. In `tracking.mode=monocular`, only Camera A plus a usable monocular scale profile are required; Camera B and stereo calibration are skipped. Fix `FAIL` items before launching. Treat `WARN` items as bring-up blockers when they concern the active mode.

## First 10 minutes checklist

1. Run `python tools\live_preflight_doctor.py --repo-root .` and fix every `FAIL`.
2. Confirm the primary tracking model exists at `models/rtmw-dw-x-l-cocktail14-384x288.onnx`.
3. Verify the primary model SHA-256 against `models/rtmw-dw-x-l-cocktail14-384x288.onnx.sha256`.
4. Confirm the primary Cocktail model exists. RTMW3D is optional and only needed when `tracking.depth_postprocess_enabled` is deliberately turned on.
5. Open `config/default.json` and confirm `tracking.mode`, `tracking.model_path`, `tracking.depth_postprocess_model_path`, and the active mode's setup fields.
6. Start the desktop UI with `bodytracker.exe config\default.json`.
7. Stereo path: pick two different open camera indices, capture chessboard images without moving cameras, then run intrinsics, stereo, floor, and body calibration commands.
8. Monocular path: pick Camera A only, enter user height, camera height, FOV/default depth, and optionally use manual plank calibration or floor-scale assist for known repeated floor-depth spacing. The UI can manual plank-line calibration
9. Launch the app and confirm model, active camera path, tracking mode, depth source, and floor-assist status are live.
10. Watch foot support, depth/floor-assist telemetry, and contact-root cards while stepping slowly, then record one short replay before changing thresholds.
11. Keep OSC off until tracker-space alignment reports `valid`.

## Model file and SHA check

Current primary tracking model: RTMW-DW-X-L Cocktail14 384x288.

Expected model path:

```text
models/rtmw-dw-x-l-cocktail14-384x288.onnx
```

Expected SHA-256:

```text
bd033156e5104c4f5d2edfe0453e02661e30a2f3da453ec93c8764d561b83054
```

Optional RTMW3D model:

```text
models/rtmw3d-x-cocktail14-384x288.onnx
```

Expected optional RTMW3D SHA-256:

```text
4a289c0e99d47eb595e99679d9d4a2d1def1b4241f9adcbafba44b9ff585ebcd
```

PowerShell check:

```powershell
Get-FileHash models\rtmw-dw-x-l-cocktail14-384x288.onnx -Algorithm SHA256
type models\rtmw-dw-x-l-cocktail14-384x288.onnx.sha256
Get-FileHash models\rtmw3d-x-cocktail14-384x288.onnx -Algorithm SHA256
type models\rtmw3d-x-cocktail14-384x288.onnx.sha256
```

The runtime performs model-contract checks at startup. The primary 2D model owns tracking quality. Live VRChat transfer calculates 3D from Cocktail 2D observations through the monocular/stereo body solver. The RTMW3D model is optional postprocessing depth evidence and is disabled by default.

## Camera setup

Use the camera setup that matches `tracking.mode`. Choose `tracking.mode=monocular` for intentional single-camera operation. Stereo mode only switches to the monocular solver as a degraded fallback when `tracking.stereo_monocular_fallback_enabled=true` and Camera B, frame pairing, pose decode, or stereo calibration readiness fails.

Recommended setup:

- rigid mounts; cameras must not move after calibration or monocular scale setup
- visible feet, knees, hips, and pelvis region
- stable lighting and minimal motion blur
- no mirror/reflection view of the user
- stereo mode: enough baseline to recover depth, but not so wide that one camera loses feet
- monocular mode: Camera A sees the floor contact area; Camera B is ignored
- matching resolution/FPS where possible in stereo mode

The desktop setup dashboard can scan camera indices and save camera choices. In stereo mode, Camera A and B must be different open devices. In monocular mode, Camera A is sufficient and Camera B can be absent or duplicated without blocking preflight.

## Chessboard capture and calibration

Single-camera capture:

```powershell
.\build\release\bodytracker.exe --capture-chessboard 0 calib\camera_a 9 6 25 1280 720 30 700
.\build\release\bodytracker.exe --capture-chessboard 1 calib\camera_b 9 6 25 1280 720 30 700
```

Stereo paired capture:

```powershell
.\build\release\bodytracker.exe --capture-stereo-chessboard 0 1 calib\camera_a calib\camera_b 9 6 25 1280 720 30 900
```

Move the chessboard through the whole shared field of view. Do not only capture the image center.

Intrinsics:

```powershell
.\build\release\bodytracker.exe --calibrate-intrinsics camera_a calib\camera_a 9 6 0.024 calib\default.json
.\build\release\bodytracker.exe --calibrate-intrinsics camera_b calib\camera_b 9 6 0.024 calib\default.json
```

Stereo:

```powershell
.\build\release\bodytracker.exe --calibrate-stereo calib\camera_a calib\camera_b 9 6 0.024 calib\default.json
```

Floor plane:

```powershell
.\build\release\bodytracker.exe --calibrate-floor calib\default.json 0 0 0 1 0 0 0 0 1
```

Body dimensions:

```powershell
.\build\release\bodytracker.exe --set-body calib\default.json 0.28 0.45 0.45 0.43 0.43 0.25 0.25
```

Calibration readiness check:

```powershell
.\build\release\bodytracker.exe --status calib\default.json
```

Do not continue to OSC/VR until calibration status reports the expected camera, stereo, floor, and body values.

With `tracking.stereo_monocular_fallback_enabled=true`, missing or unloadable stereo calibration no longer blocks startup; the runtime continues through Camera A inferred-depth fallback and reports `stereo_monocular_fallback:stereo_calibration_not_ready` once frames are available. With fallback disabled, missing or unloadable stereo calibration remains a startup error.


## Neutral body calibration mode

To replace generic limb defaults with user-specific anatomy, enable:

```json
"tracking": {
  "body_calibration": {
    "enabled": true,
    "auto_persist": true,
    "required_seconds": 2.5,
    "min_overall_confidence": 0.55,
    "max_segment_cv": 0.12
  }
}
```

Stand neutrally with hips, knees, ankles, heels, toes, and HMD visible for a few seconds. Stereo triangulation is preferred. Monocular + floor-scale evidence is accepted at lower confidence when the floor scale is valid. With `auto_persist=true`, the estimator writes pelvis width, left/right femur, left/right tibia, left/right foot length, standing HMD-to-pelvis offset, and per-value quality into the active calibration file, retries failed writes, and saves again on clean shutdown. These values then feed the IK model as fixed segment constraints; they do not drift every frame.

Watch `body_calibration.reason`, `persist_status`, `persist_error`, `accepted_samples`, `overall_confidence`, and `body.quality.*` in the web status/replay log. Common failure states: not enough lower-body visibility, posture not neutral, foot/hip segment lengths outside plausible human ranges, unstable monocular floor-scale evidence, or a calibration file write error.

## Monocular no-chessboard setup

Set `tracking.mode` to `monocular` in the desktop UI or `config/default.json`.

Required monocular inputs:

- Camera A device index, width, height, and FPS
- `tracking.monocular.horizontal_fov_deg`
- `tracking.monocular.user_height_m`
- `tracking.monocular.camera_height_m`
- `tracking.monocular.default_depth_m`

Optional Guided floor-scale assist:

1. Enable scalar floor-scale assist only when visible tile rows, rug edges, or floor-depth lines have a known physical spacing. For planks, use the drawn projective quad for full width/length geometry; do not treat the varying full board length as scalar repeated spacing.
2. Enter the real physical floor-depth spacing in meters, for example `0.30` for 30 cm tile depth. Do not enter the apparent pixel length as the physical spacing.
3. Use manual floor geometry for a visible plank, or enter scalar floor assist only for a stable known repeated floor-depth spacing. The UI writes `floor_depth_line_spacing_px`, `floor_depth_reference_y_px`, and confidence when the drawn geometry provides usable scalar spacing; otherwise the saved projective quad is used when available.
4. Runtime status reports floor assist as `disabled`, `standby`, `active`, `weak`, `inactive`, or `invalid`; `standby` means stereo is currently using triangulation but the same inputs will apply if Camera A monocular fallback engages, while `inactive` means the single-camera path ran but did not actually use floor spacing.
5. Manual override is allowed: edit the physical depth spacing, marked pixel spacing, reference y, or reference depth fields before saving.
6. Weak/inactive/invalid floor inputs do not make monocular config unusable; the solver falls back to body/default/floor-ray monocular depth and reports lower confidence.

Useful status fields:

- `tracking_mode`
- `depth_source`
- `floor_assist.status`
- `floor_assist.config_status`
- `floor_assist.source`
- `floor_assist.depth_m`
- `floor_assist.confidence`
- `solver.depth.source`
- `solver.depth.scale_source`
- `solver.depth.floor_assist`

Monocular mode does not require Camera B, stereo triangulation, chessboard calibration, or `calib/default.json`. If a stereo calibration file exists it may still be loaded as optional context, but a missing or bad stereo calibration file does not block monocular runtime startup.

## First launch

Desktop UI:

```powershell
.\build\release\bodytracker.exe config\default.json
```

Native runtime dashboard:

```powershell
.\build\release\bodytracker.exe --run config\default.json
```

Setup-focused desktop UI:

```powershell
.\build\release\bodytracker.exe --setup config\default.json
```

The old local browser server has been removed from the source tree. Use the WebView2 desktop UI plus replay logs for status; do not add a second HTTP status surface.

## Confirm the model loaded

In the desktop UI, the model card should show the expected model path and a found/ready state. In logs/status, look for successful model load and no model-contract failure.

If the model fails:

- verify `tracking.model_path`
- verify `tracking.depth_postprocess_model_path` if VRChat depth transfer is enabled
- verify the SHA-256
- verify the file is the RTMW-DW-X-L Cocktail14 primary export expected by `models/README.md`
- do not continue with calibration or tracking debug while model load is failing

## Confirm cameras are paired

Use the desktop/local status cards:

- stereo mode: camera A/B are open
- monocular mode: camera A is open and Camera B can remain closed
- frame ages stay fresh
- stereo mode: `frame_pairing.accepted_pairs` increases and `frame_pairing.last_skew_ms` remains small
- monocular mode: `tracking_mode=monocular`, `depth_source=inferred_monocular`, and frame skew stays zero/unused
- duplicate/drop counters do not continuously climb

If pairing looks wrong, fix camera indices, FPS/resolution, exposure, or USB bandwidth before tuning tracking.

## Confirm tracking is live

Before evaluating feet, confirm the whole chain is producing fresh data:

- degradation mode is not stuck in setup or camera failure
- pose A/B valid counts are nonzero
- reliability A/B confidence is not near zero
- tracker outputs have recent timestamps
- synthesized tracker confidence is not clamped to zero
- OSC is disabled until local tracking looks correct, or the receiver is known-good

## Tracker-space alignment before OSC

OSC tracker output is gated by a finite, configured tracker-space transform. The app can solve trackers in camera/world coordinates, but VRChat expects those trackers in the VR tracker space. Do not enable OSC until the desktop UI shows tracker-space status `valid` and the configured tracker indices are what you intend to bind in VRChat. For controller-derived alignment, OpenVR/provider/controller freshness tells you whether you can sample or improve the alignment right now; it does not automatically kill the last solved transform.

The hard rule: stale is not invalid. Stale means aged, degraded, or warning-worthy. It does not mean suppressed, deleted, zeroed, hidden, or treated like a null transform. Only invalid/non-finite/unconfigured/cleared tracker-space data blocks OSC. If the last controller-solved transform is finite and still the selected active transform, keep using it with honest freshness/age telemetry. If a finite manual/json fallback is selected instead, use that. Do not write logic or docs that throw away usable tracker-space math just because the provider stopped being fresh.

Manual/json alignment path:

1. Set `hmd.mode` to `json_file` when using an external HMD pose JSON, or leave it `null` for a fully manual transform.
2. Enter the camera-to-VR `osc.tracker_space_position_offset`, `osc.tracker_space_rotation`, and `osc.tracker_space_scale` in the Guided tracker-space alignment panel.
3. Click **Validate alignment**. This only validates finite numbers, positive scale, and non-zero quaternion; it does not prove the transform is physically correct.
4. Mark **Transform valid**, save config, then enable OSC. Saving a valid manual/json transform also stores it in `osc.manual_tracker_space_*` as the preserved fallback.
5. If SteamVR controller alignment later succeeds, it becomes the active `osc.tracker_space_*` transform but does not overwrite `osc.manual_tracker_space_*`.
6. If controller alignment is cleared, missing, unconfigured, non-finite, OpenVR is unavailable/not built, controllers are missing/untracked before any solve exists, or no finite controller-solved transform has ever been produced, runtime OSC uses the valid manual fallback. Without any finite active transform or finite fallback, OSC is blocked.
7. If controller poses become stale **after** a finite controller transform has been solved, do not delete that transform and do not call it invalid. Keep the solved transform available, report the provider age/freshness problem, and either continue using the stale-but-finite transform or switch to a valid manual/json fallback according to explicit active-source policy.

Relevant status fields:

- `tracker_space.status`
- `tracker_space.valid`
- `tracker_space.source`
- `tracker_space.osc_blocked`
- `tracker_space.position_offset`
- `tracker_space.rotation`
- `tracker_space.scale`
- `osc.tracker_space_transform_valid`
- `tracker_space.manual_fallback_valid`
- `tracker_space.manual_fallback_source`

Current HMD input providers are `null` and `json_file`. SteamVR/OpenVR is used for controller-sampled tracker-space alignment, not as the body-tracking source. If OpenVR support was not built, treat that as a hard capability absence for controller alignment, not a temporary tracking hiccup.

VRChat tracker-role caveat:

The OSC sender publishes configured `/tracking/trackers/N` index paths for pelvis, feet, and knees. That proves packet formatting and configured index mapping only. It does not prove VRChat performed role negotiation or bound those indices to the intended body roles. Check the UI/replay role-index diagnostics and bind/verify roles in VRChat before calling the integration ready.

## How to read the foot and root cards

The UI and `/api/status` expose these fields so a bad frame can be blamed without guessing.

Foot support:

- `support.left_foot.phase`
- `support.right_foot.phase`
- `support.left_foot.support_confidence`
- `support.right_foot.support_confidence`
- `support.*.contact_residual`
- `support.*.anchor_residual`
- `support.*.heel_residual`
- `support.*.toe_residual`

Depth and floor-scale evidence:

- `tracking_mode`
- `depth_source`
- `floor_assist.status`
- `floor_assist.config_status`
- `solver.depth.source`
- `solver.depth.confidence`
- `solver.depth.scale_source`
- `solver.depth.floor_assist`

Stereo foot evidence:

- `solver.triangulation.preliminary.left_foot_contact_confidence`
- `solver.triangulation.preliminary.right_foot_contact_confidence`
- `solver.triangulation.preliminary.left_foot_low_res_separation_px`
- `solver.triangulation.preliminary.right_foot_low_res_separation_px`
- `solver.triangulation.preliminary.foot_mean_reprojection_error_px`
- `solver.triangulation.preliminary.max_foot_reprojection_error_px`

Root correction:

- `motion_filter.contact_root.reason`
- `motion_filter.contact_root.correction_m`
- `motion_filter.contact_root.common_residual_m`
- `motion_filter.contact_root.root_innovation_m`
- `motion_filter.contact_root.foot_disagreement_m`
- `motion_filter.contact_root.root_alignment`
- `support.left_foot.support_confidence`
- `support.right_foot.support_confidence`

Good bring-up behavior: support phases change smoothly, foot-contact confidence is high during true plants, contact residuals stay small during planted feet, and root correction either stays off or reports `CONTACT_ROOT_COMMON_MODE` only when both foot support confidences are strong and the feet agree.

## Recording a replay

Enable replay recording in config or the desktop UI:

```json
{
  "tracking": {
    "enable_replay_recording": true
  },
  "debug": {
    "replay_log_path": ""
  }
}
```

If `debug.replay_log_path` is set, runtime recording writes there. If it is empty, recording uses `app.recording_dir` and writes `latest-runtime.ndjson`.

Replay solve:

```powershell
.\build\release\bodytracker.exe --replay-solve calib\default.json recordings\latest-runtime.ndjson
```

Replay export should include support confidence, contact residuals, root-correction details, and preliminary triangulation blame fields. Replay is closer to live semantics now, but it is still not a complete proof of live/runtime parity.

## If feet look wrong, check this in order

1. **Camera visibility/confidence**: In stereo, are both cameras seeing the ankle, heel, big toe, and small toe? In monocular, is Camera A seeing the lower body and floor contact area?
2. **Depth source**: In monocular, is `depth_source=inferred_monocular`, is `solver.depth.confidence` low, or is `floor_assist.status` weak/inactive/invalid?
3. **Stereo disagreement**: In stereo, are `foot_mean_reprojection_error_px` or `max_foot_reprojection_error_px` high, or does one camera disagree with the other?
4. **Foot-contact confidence**: Are `left_foot_contact_confidence` / `right_foot_contact_confidence` low during a supposed plant?
5. **Low-res ambiguity**: Is `*_low_res_separation_px` tiny, making ankle/heel/toes indistinguishable?
6. **Support phase**: Is the foot incorrectly stuck in `FlatPlant`, `HeelLock`, or `ToePivot` instead of `Swing`, `ReleasePending`, or `Slip`?
7. **Contact residuals**: Are `heel_residual`, `toe_residual`, or `anchor_residual` growing while support remains high?
8. **Motion filter decision**: Is the free foot being held/blended when it should swing?
9. **Root correction reason**: Is `motion_filter.contact_root.reason` applying correction when support confidence is low or disagreement is high?
10. **Tracker confidence**: Did synthesized foot tracker confidence drop when support evidence became weak?
11. **Replay comparison**: Record the clip before tuning. Use replay/debug fields to confirm whether the failure starts at camera evidence, triangulation, support classification, motion filtering, or tracker synthesis.

## Common failure modes

| Symptom | Most likely checks |
| --- | --- |
| Model never starts | `tracking.model_path`, SHA-256, RTMW-DW-X-L Cocktail14 384x288 contract |
| One foot sticks planted during occlusion | camera visibility, foot-contact confidence, support phase decay |
| Body jitters when a planted foot jitters | root correction reason, support confidence, contact residuals |
| Toe pivot looks like whole-foot sliding | support phase, toe residual, final contact residual |
| Heel lock snaps or drifts | heel residual, low-res separation, stereo reprojection |
| Both feet wrong after camera move | stereo: calibration status and recapture/recalibrate; monocular: recheck camera height/FOV/floor-scale marks |
| Monocular depth swims | `solver.depth.confidence`, user height, camera height, FOV, floor-assist status |
| OSC target receives bad trackers | disable OSC until local UI/replay evidence is sane, tracker-space alignment is finite/configured, provider age is understood, and VRChat tracker index binding is confirmed. Do not treat provider staleness as transform invalidity. |

## Config defaults to know

Important defaults in `config/default.json`:

- `tracking.mode`: `stereo` by default; set to `monocular` for single-camera markerless setup
- `tracking.model_path`: `models/rtmw-dw-x-l-cocktail14-384x288.onnx`
- `tracking.depth_postprocess_model_path`: `models/rtmw3d-x-cocktail14-384x288.onnx`
- `tracking.depth_postprocess_enabled`: `false`
- `tracking.depth_postprocess_interval_frames`: `4`
- `tracking.depth_postprocess_allow_cpu_fallback`: `false`
- `tracking.calibration_path`: `calib/default.json` for stereo; not required by monocular mode
- `tracking.enable_replay_recording`: false
- `app.recording_dir`: `recordings`
- `debug.replay_log_path`: empty means use `app.recording_dir/latest-runtime.ndjson`
- `osc.enabled`: false for local bring-up
- `motion_consistency.contact_root_correction_gain`: root correction strength
- `motion_consistency.contact_root_min_support_confidence`: support confidence gate for root correction
- `tracking.monocular.*`: single-camera FOV, user height, camera height, default depth, and optional floor-scale assist fields
- `osc.tracker_space_transform_valid`: false until tracker-space alignment is intentionally configured

Threshold defaults shared by solver, config, and status live in
`src/tracking/tracking_constants.h`; config comments and
`config/bodytracker-config.schema.json` point back to those owners. Config
validation outcomes are documented in
[`CONFIG_CONTRACTS.md`](CONFIG_CONTRACTS.md): `invalid` blocks load, while
`degraded`, `missing-but-defaultable`, and `warning` must preserve launch and
show the blocked/degraded runtime path.

Synthetic diagnostics and regression scoring are documented in [`docs/SYNTHETIC_STEREO_DIAGNOSTICS.md`](SYNTHETIC_STEREO_DIAGNOSTICS.md). Use them to understand failure signatures; do not treat them as a replacement for real camera validation.
