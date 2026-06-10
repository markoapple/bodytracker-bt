# bodytracker

Native C++20 camera-based VR tracker foundation for Windows.

The goal is a mode-aware body estimator for either two fixed RGB cameras or a single RGB camera, RTMW-DW-X-L Cocktail14 384x288 whole-body observations mapped into the internal 26-joint tracker contract, calculated 3D lower-body solving plus direct upper-body landmark output for VRChat transfer, HMD anchoring, support-manifold logic, and VRChat OSC output.

This is an MVP tracker foundation, not a finished plug-and-play VRChat full-body tracker. The core runtime path is present, but real use still depends on a valid ONNX model export, either stereo calibration or a configured monocular scale profile, camera-to-VR playspace alignment, and footage-based tuning.

## Scope

This project is intentionally not standing-only. The canonical state model includes:

- upright standing
- upright transition
- crouching
- kneeling
- seated supported
- reclined supported
- unknown/free

The support model distinguishes floor support from non-floor rest support. A seated or reclined user can have a stable foot on a couch, bed, chair, or other unknown surface without snapping that foot to the floor. Rest support still needs usable contact evidence; low motion alone must not create a fake couch/bed anchor during occlusion. Kneeling is a posture mode, not proof of knee contact: root support only reports knee support when knee-anchor evidence exists, otherwise it falls back to active foot support or no support.


## Documentation Map

Read these in this order when taking over the repo:

1. [`AGENTS.md`](AGENTS.md) for the no-Bureaucrat-Logic rule Codex must follow.
2. [`docs/CODEX_FULL_KNOWLEDGE.md`](docs/CODEX_FULL_KNOWLEDGE.md) for the verbatim consolidated Codex/build/review knowledge bundle.
3. [`docs/RUNTIME_BRINGUP.md`](docs/RUNTIME_BRINGUP.md) for the practical local bring-up path.
4. [`docs/BUILD_ENVIRONMENT.md`](docs/BUILD_ENVIRONMENT.md) before local native build/debug work.
5. [`docs/CONFIG_CONTRACTS.md`](docs/CONFIG_CONTRACTS.md) for config validation outcomes, shared threshold ownership, and migration behavior.
6. [`docs/MONOCULAR_REAL_FOOTAGE_VALIDATION.md`](docs/MONOCULAR_REAL_FOOTAGE_VALIDATION.md) and [`docs/SYNTHETIC_STEREO_DIAGNOSTICS.md`](docs/SYNTHETIC_STEREO_DIAGNOSTICS.md) for replay/diagnostic validation.
7. [`docs/PERFORMANCE_QA_INFRASTRUCTURE.md`](docs/PERFORMANCE_QA_INFRASTRUCTURE.md) for profiler and performance-budget checks.
9. [`docs/REJECTED_FEATURES.md`](docs/REJECTED_FEATURES.md) for features deliberately removed from the active plan because they are useless, wrong-scope, or fake-roadmap bait.

Do not let an agent start from grep hits or broad search counts. For this repo, every feature claim must be traced through config, UI, save/load, runtime logic, telemetry/debug output, and docs before tests are treated as anything useful. Tests are guards; they are not proof that an architectural path is real.

## Runtime Constraints

- Windows-first native C++20
- No Python runtime
- OpenCV for capture, calibration, image work, and calibration previews
- ONNX Runtime for RTMPose inference
- WinSock UDP for OSC on Windows
- Minimal dependencies beyond those pieces

## Current Build Target

The current codebase has a first live MVP runtime path wired through the existing contracts:

- canonical internal 26-joint body/feet keypoints
- canonical tracking state
- posture modes
- support manifold state
- RTMPose/RTMW3D decode contracts
- ROI tracking
- measurement reliability terms
- triangulation helpers
- explicit stereo/monocular tracking modes
- monocular inferred-depth lifting with user scale, camera height, and optional floor-depth spacing assist
- lower-body model and solver scaffolding
- direct upper-body landmark state for chest/elbow tracker output
- root support, foot support, temporal update, tracker synthesis
- replay/debug snapshots
- live paired-frame RTMPose inference path
- CPU or DirectML ONNX Runtime execution with CPU fallback status export
- OSC adapter boundary with default VRChat roles for pelvis, left foot, right foot, chest, left elbow, and right elbow; knees are optional and disabled by default

Unsupported RTMPose exports fail loudly with exact output shapes. No fake success path is allowed.

## Build

Use `docs/BUILD_ENVIRONMENT.md` when you are actually doing local native build/debug work. Do not start a feature audit by building; first prove the source-of-truth path, UI/backend contract, runtime consumer, persistence path, and telemetry behavior.

This archive may be the source-only/small package. If `models/rtmw-dw-x-l-cocktail14-384x288.onnx` or `models/rtmw3d-x-cocktail14-384x288.onnx` is missing, the native build can still compile and most tests can still run, but live inference/postprocessing and the ONNX metadata inspection tests require the real models. Copy both models into `models/` for full-package validation. Set `BODYTRACKER_REQUIRE_ONNX_ASSET=1` before running the model asset test if you want missing-model packages to fail hard.

```powershell
cmake --preset windows-vcpkg-release
cmake --build --preset release --parallel
ctest --preset release
```

For source/package sanity without native runtime dependencies:

```powershell
cmake --preset source-sanity
cmake --build --preset source-sanity --parallel
ctest --preset source-sanity
```

That preset builds and runs only dependency-light native source/contract checks (`BODYTRACKER_BUILD_APP=OFF`, `BODYTRACKER_BUILD_FULL_TESTS=OFF`, `BODYTRACKER_REGISTER_PYTHON_TESTS=OFF`). It is not a substitute for the full Windows vcpkg build, Python/static checks, full CTest pass, model load, or live runtime validation.

Manual fallback:

```powershell
cmake -B build/release -S . `
  -DCMAKE_BUILD_TYPE=Release `
  -DCMAKE_TOOLCHAIN_FILE=$env:VCPKG_ROOT\scripts\buildsystems\vcpkg.cmake

cmake --build build/release --config Release --parallel
ctest --test-dir build/release --output-on-failure
```

The checked-in `vcpkg.json` pins the project-level dependency list. GitHub Actions now runs the same configure/build/test path on Windows.

## Run

```powershell
.\release\bodytracker\bodytracker.exe --run release\bodytracker\config\default.json
```

Equivalent explicit run mode:

```powershell
.\release\bodytracker\bodytracker.exe --run release\bodytracker\config\default.json
```

## Minimum Real-World Bring-Up Path

Start with the practical live checklist in [`docs/RUNTIME_BRINGUP.md`](docs/RUNTIME_BRINGUP.md). Before launching on a local machine, run `python tools\live_preflight_doctor.py --repo-root .` from the repo root to catch missing model/calibration/config issues. For agent rules, read [`AGENTS.md`](AGENTS.md). The short version is:

1. Put the RTMW-DW-X-L Cocktail14 384x288 ONNX export at `models/rtmw-dw-x-l-cocktail14-384x288.onnx` and verify SHA-256 `bd033156e5104c4f5d2edfe0453e02661e30a2f3da453ec93c8764d561b83054`.
2. The live VRChat path calculates 3D from the Cocktail 2D output through the monocular/stereo body solver. RTMW3D at `models/rtmw3d-x-cocktail14-384x288.onnx` is optional and disabled by default; enable `tracking.depth_postprocess_enabled` only for experiments that explicitly need model z bins.
2. Choose `tracking.mode`. Use `stereo` for calibrated two-camera depth; use `monocular` for one-camera inferred depth.
3. For stereo, capture chessboard frames for both cameras, run intrinsics, run stereo calibration, set floor plane, then set body dimensions.
4. For monocular, set `tracking.monocular.user_height_m`, `camera_height_m`, `horizontal_fov_deg`, and optionally enable floor scale assist with a measured floor-depth tile/plank spacing plus observed pixel spacing. In `tracking.body_calibration.enabled=true`, stand neutral for a few seconds so the runtime can persist real pelvis width, femur/tibia, foot length, and standing HMD-to-pelvis offset with per-value quality; `auto_persist=true` retries failed writes and saves again on clean shutdown.
5. Run `--status calib\default.json` and do not continue until readiness is clean for the chosen mode.
6. Run with OSC disabled first and inspect the WebView2 desktop UI plus replay logs.
7. Calibrate camera-world coordinates into VRChat/SteamVR tracker space.
8. Only then enable OSC. Manual/json alignment must contain finite tracker-space numbers; SteamVR controller alignment can continue from the last solved transform even if the live provider later goes stale.


## Monocular Mode

Set `tracking.mode` to `monocular` to run with only camera A. Camera B is disabled instead of faked. Depth is exported as `inferred_monocular`, not `triangulated_stereo`. Stereo mode remains a calibrated two-camera path, and `tracking.stereo_monocular_fallback_enabled=true` lets it temporarily reuse the Camera A monocular solver when Camera B is unavailable, unpaired, fails pose decode, or stereo calibration is missing/not ready/unloadable. With fallback enabled, bad or missing stereo calibration degrades into Camera A inferred-depth tracking instead of killing startup; with fallback disabled, calibration errors still fail fast.

The minimum monocular profile is `image_width`, `image_height`, `horizontal_fov_deg`, `user_height_m`, `camera_height_m`, `default_depth_m`, `depth_confidence_scale`, and `min_seed_count`. The solver creates a virtual single-camera projection profile from those values and uses a flat `y=0` floor plane for support logic when no stereo floor calibration exists.

Optional floor scale assist is configured with `floor_scale_assist_enabled=true`, `floor_depth_line_spacing_m`, `floor_depth_line_spacing_px`, and a usable `floor_depth_reference_y_px`. Use it for tile rows, rug edges, or another known repeated floor-depth spacing; do not enter full plank length as a scalar spacing. The desktop UI now defaults to manual plank geometry: draw two long plank edges, one short end cap, and optionally the opposite end cap. Three lines give a local outline/width hint; four lines plus real width and length save a projective plank quad. The runtime reports `floor_spacing` when the drawn lines provide usable scalar parallel spacing; otherwise it uses the projective quad when available or falls back to the camera-height floor ray/body scale instead of treating spacing as a fake meters-per-pixel ruler. The same saved monocular floor-assist settings are inherited by stereo monocular fallback.

## Calibration Capture CLI

Single-camera chessboard capture:

```powershell
.\build\release\bodytracker.exe --capture-chessboard 0 calib\camera_a 9 6 25 1280 720 30 700
.\build\release\bodytracker.exe --capture-chessboard 1 calib\camera_b 9 6 25 1280 720 30 700
```

Stereo chessboard pair capture:

```powershell
.\build\release\bodytracker.exe --capture-stereo-chessboard 0 1 calib\camera_a calib\camera_b 9 6 25 1280 720 30 900
```

The capture modes save frames only when a chessboard is detected, with a minimum interval so you can move the board between samples. The current CLI also shows an OpenCV preview window during capture; press ESC to stop early.

## Calibration CLI

Intrinsic calibration from chessboard images:

```powershell
.\build\release\bodytracker.exe --calibrate-intrinsics camera_a calib\camera_a 9 6 0.024 calib\default.json
.\build\release\bodytracker.exe --calibrate-intrinsics camera_b calib\camera_b 9 6 0.024 calib\default.json
```

Stereo extrinsics from paired chessboard image directories. Files are paired by sorted filename order; both folders must contain the same number of readable images and all pairs must use the same image size:

```powershell
.\build\release\bodytracker.exe --calibrate-stereo calib\camera_a calib\camera_b 9 6 0.024 calib\default.json
```

Floor plane from at least three already-triangulated world points:

```powershell
.\build\release\bodytracker.exe --calibrate-floor calib\default.json 0 0 0 1 0 0 0 0 1
```

Body dimensions:

```powershell
.\build\release\bodytracker.exe --set-body calib\default.json 0.32 0.42 0.42 0.42 0.42 0.24 0.24
```

Calibration readiness check:

```powershell
.\build\release\bodytracker.exe --status calib\default.json
```

Offline replay solve from a runtime NDJSON recorded with `enable_replay_recording=true`:

```powershell
.\build\release\bodytracker.exe --replay-solve calib\default.json recordings\latest-runtime.ndjson
```

## Runtime Alignment Notes

OSC output is blocked only when there is no finite, configured tracker-space transform to send. The active `osc.tracker_space_*` fields may be either manual/json or SteamVR-controller solved. Manual fallback is stored separately in `osc.manual_tracker_space_*`; controller alignment must never overwrite or destroy it.

Do not teach the code that `stale` means `invalid`. A stale SteamVR/controller provider means the live provider is not fresh enough to sample a new alignment right now. It does **not** poison an already-solved finite tracker-space transform. Keep the last finite solved transform available, mark its age/freshness honestly, lower confidence or warn if needed, and prefer a valid manual/json fallback only when policy says that fallback is the better active transform. Suppress OSC only for missing, unconfigured, non-finite, zero-scale, or explicitly cleared transforms. Stale-but-finite is degraded evidence, not garbage.

While runtime is running, desktop saves may hot-apply OSC/tracker-space only. Camera, model, tracking, calibration, floor-geometry, and HMD changes are cold settings; stop runtime before saving them. Floor geometry is backend-owned and consumed only when `tracking.monocular.floor_geometry_calibration_enabled` is true. Clearing it must disable the flag and remove the persisted generated evidence, not just hide the UI marks.

Config load/save validation uses four outcomes: `invalid`, `degraded`, `missing-but-defaultable`, and `warning`. Only invalid/non-finite/structurally unusable values block config load. Recoverable setup gaps, including OSC enabled before tracker-space calibration, must load with warnings so runtime can report the exact blocked output.

For top-down RTMPose models, use per-camera `initial_roi_*` settings when the whole frame is too wide or includes background bodies or clutter. The ROI can be normalized `[x, y, width, height]` in 0..1 frame coordinates, or pixel coordinates when `initial_roi_normalized` is false.

## CI Coverage

CI verifies static QA contracts, dependency-light coverage artifacts, source-sanity tests, and the Windows configure/build/test path. Static QA also checks JSON contracts, API-doc freshness, and the performance-budget tool. It still does not validate real cameras, real ONNX inference quality, VRChat OSC behavior in-client, or SteamVR playspace alignment because those require hardware/assets not present in CI.

## Not Yet

This is not yet a plug-and-play VRChat tracker. The live runtime path is wired, including ONNX inference, lower-body solving, upper-body landmark output, eight-role tracker synthesis with knees optional, replay logging, calibration-file writing, webcam chessboard capture, image-directory calibration CLI modes, stereo triangulation/reprojection, single-camera inferred-depth monocular solving, stale-frame age/freshness handling, partial-evidence seed handling, invalid/unconfigured OSC suppression, offline replay solving from recorded decoded poses/timestamps, optional static/JSON-file HMD priors, and OSC output.

The remaining hard pieces are real camera/calibration footage, in-client VRChat/SteamVR receiver validation, hardware validation of SteamVR controller-derived tracker-space alignment, continuous runtime floor-pattern tracking beyond setup-time seam-family detection, and iterative tuning against actual camera footage. SteamVR controller alignment and manual tracker-space fallback are implemented paths, but synthetic or image-local debug markers do not prove in-client SteamVR correctness.


## Floor Geometry Calibration

The runtime can use repeated floor seams as passive calibration evidence. Planks provide one-axis floor scale/orientation when spacing is known; tile grids can provide two-axis floor geometry. See `docs/FLOOR_GEOMETRY_CALIBRATION.md`.

## ChatGPT Browser Upload Package Notes

The `bodytracker-chatgpt-browser-package.zip` archive is a small review/upload package, not the full local runtime bundle. It intentionally excludes build folders, logs, WebView2 profile/cache data, generated Android intermediates, and all large `*.onnx` model files so it can stay under browser upload limits.

For live inference, restore the model after unpacking:

```powershell
Copy-Item C:\path\to\rtmw-dw-x-l-cocktail14-384x288.onnx .\models\rtmw-dw-x-l-cocktail14-384x288.onnx
Copy-Item C:\path\to\rtmw3d-x-cocktail14-384x288.onnx .\models\rtmw3d-x-cocktail14-384x288.onnx
```

Then run `python tools\live_preflight_doctor.py --repo-root .`. Missing models should fail model/inference checks loudly but must not be patched into fake success. DirectML/GPU use depends on the local ONNX Runtime DirectML DLLs and Windows GPU driver state; a CPU fallback warning means the runtime could still compute, but performance/latency may be worse. Do not make CPU fallback silent, and do not turn a recoverable provider warning into a tracking kill switch.

Phone camera bring-up can fail from USB install state, network binding, firewall/private-network permissions, wrong camera index, battery/sleep, or iPhone/browser camera permission. Use the desktop UI phone-site control when available, verify the shown host/port from the phone browser, keep the phone awake, allow camera access in the mobile browser, and check Windows Firewall if the page loads locally but not from the phone. The APK path in the release payload is convenience-only; the browser/site path exists so capture is not dependent on custom APK install success.

Per-camera wall/plank setup is explicit. Camera A drawings save only to Camera A; Camera B drawings save only to Camera B. Stereo wall/plank constraints are used only when both cameras have valid matching evidence for their current image sizes. A-only or B-only geometry is useful setup progress, but stereo should report it as partial and not silently apply it to both cameras.

SteamVR/OSC alignment keeps last finite tracker-space evidence alive. A stale SteamVR/controller sample is degraded evidence, not proof that the solved transform is invalid. Suppress OSC only for missing, unconfigured, non-finite, zero-scale, or explicitly cleared transforms.

Before changing runtime behavior, read `AGENTS.md`. The No Bureaucrat Logic rule is part of the package contract: degraded, stale, partial, fallback, or draft evidence should reach the right consumer with honest confidence/scope/status instead of being erased by broad validation gates.
