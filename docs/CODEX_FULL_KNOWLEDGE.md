# Codex Full Knowledge Bundle

This file is a verbatim consolidation of the old scattered Codex/review notes. Each source keeps its original text below, with SHA-256 recorded before the content.

---

## Source: CODEX_NOTES.md

SHA-256: 1e77d525d342200bf71d7f94c6397abf4c1fa36eda6a90c3510fca7cbb60cedf

```text
# Codex notes

Read `AGENTS.md` before editing. Its Bureaucrat Logic rule is the main repository instruction.

## Build and test commands

Dependency-light source sanity path:

```powershell
cmake --preset source-sanity
cmake --build --preset source-sanity
ctest --preset source-sanity --output-on-failure
python -m unittest tests.runtime_control_wiring_test -q
node --check src/ui/app/app.js
```

Full Windows/vcpkg path:

```powershell
cmake --preset windows-vcpkg-debug
cmake --build --preset debug
ctest --preset debug --output-on-failure
```

## Runtime viewing instructions

Desktop UI:

```powershell
.\build\release\bodytracker.exe --setup config/default.json
```

Run mode:

```powershell
.\build\release\bodytracker.exe --run config/default.json
```

Preflight check:

```powershell
python tools\live_preflight_doctor.py --repo-root .
```

Use the UI to draw manual plank geometry, inspect runtime telemetry, and verify that degraded/fallback states still reach output instead of disappearing behind status flags.
```

---

## Source: CODEX_BUREAUCRAT_LOGIC_FIELD_MANUAL.md

SHA-256: d79993bbeed17bab234a7dc65f030b8b16d4071ef585f7613c7d0dfb068ac2db

```text
# CODEX: Bureaucrat Logic Field Manual

Read this before touching tracking, calibration, OSC, SteamVR alignment, replay, config save/load, UI state, model startup, or any other place where data is allowed to move from one room to another without being interrogated by a nervous little boolean.

This repo already lived through the dumb version of this. Not once. Not twice. Everywhere. The code would compute useful data, attach a label like `stale`, `predicted`, `manual`, `partial`, `backend_owned=false`, or `source_known=false`, and then some downstream desk clerk would quietly throw the result into a filing cabinet marked **DO NOT USE BECAUSE I FEEL WEIRD**.

Do not reintroduce that shit.

The core rule is stupidly simple:

**Quality metadata is not permission metadata.**

A weak result still exists. A stale provider does not erase a solved transform. A manual calibration is not fake because a human drew it. A predicted tracker is not illegal because it came from prediction. A logger failure is not a tracking failure. A bad optional homography is not a reason to poison an entire calibration bundle. A source string is not more powerful than finite math. We already had this conversation with the codebase. The codebase lost.

If data is finite, scoped, and useful, carry it forward. Lower confidence. Narrow capability. Add a status. Keep last-known-good. Do not construct a tiny courthouse around a flag and sentence the whole subsystem to death.

## The actual pattern that caused pain

The recurring failure was this:

```text
1. Compute something useful.
2. Mark it as degraded, manual, stale, predicted, partial, unknown-source, or not fully promoted.
3. Somewhere else, reinterpret that label as a veto.
4. Drop output, erase state, refuse save, block runtime, or reset a working transform.
5. Spend three hours wondering why the feature is "implemented" but behaves like a decorative corpse.
```

That is Bureaucrat Logic. It is not robustness. It is software wearing a tie and unplugging the machine.

## What Bureaucrat Logic already looked like here

### Predicted trackers were computed, then banned

The pipeline created fallback/HMD-carried/predicted tracker poses with capped confidence. Then OSC rejected them by source category.

That is doing the work, printing the result, and then eating the paper because the header font made you nervous.

Correct behavior: send finite tracker poses above the configured confidence floor. Their weakness belongs in confidence and telemetry, not in a category ban.

### Anchor-held feet were locally supported, then globally kneecapped

Foot support evidence existed. Anchor-held confidence existed. Then broad body confidence got to cap the foot anyway.

A planted foot is local evidence. It does not need a permission slip from the entire skeleton parliament.

Correct behavior: use local support confidence for anchor-held foot evidence. Use body confidence for body-derived guesses. Do not let a vague global mood kill a specific planted contact.

### Manual plank evidence was treated like contraband

The user drew a plank and entered dimensions. The backend knew metric width. The UI displayed it. Then scalar metric use was disabled because it came from manual plank evidence.

That is not engineering. That is the code asking for a number and then acting shocked when the user gives it one.

Correct behavior: manual metric evidence is runtime evidence. It may have limited capability. It is not decorative. It does not sit in JSON as a little museum exhibit.

### Stale SteamVR provider status invalidated solved transforms

The controller/provider was needed to solve the tracker-space transform. It was not needed to bless the transform forever like some plastic bishop on a USB cable.

Correct behavior: provider freshness affects new sampling and confidence. It does not erase finite numeric transform data that was already solved.

### Unknown source labels vetoed numeric transforms

A transform could be finite and usable, but an unfamiliar source string could mark it invalid.

A string label is not the boss of math. If the numbers are good, use the numbers. If the label is weird, call the label weird. Do not turn the transform into ash because `tracker_space_source` did not wear the approved hat.

### Debug/output plumbing held runtime hostage

OSC open failure, replay log open failure, and backup cleanup failure were allowed to block or report failure for runtime/config behavior that had already succeeded or could continue without them.

The notebook failing to open does not mean the robot has no legs. The janitor failing to delete a backup file does not mean the config was not saved. Stop promoting chores into gods.

### Optional checks became veto gods

Forward yaw samples, floor samples, homography validity, timestamp skew, readiness summaries, and model contract checks were allowed to block paths that had degraded-but-usable output.

Optional evidence is optional. Cross-checks are cross-checks. Diagnostics are diagnostics. If they become guillotines, you rebuilt the bug.

## The ten Bureaucrat Logic crimes

### 1. Precautionary Kill Switch

You compute fallback/degraded data and then a downstream flag says "no, never send that." No. If the data is finite and useful, send it with lower confidence.

Bad:

```cpp
if (source == Predicted) return false;
```

Correct:

```cpp
return pose_finite && confidence >= configured_floor;
```

### 2. Nanny-State Override

The UI/config accepts a value and runtime secretly clamps, rewrites, ignores, or replaces it.

Bad:

```cpp
const float pitch = Clamp(config.floor_camera_pitch_rad, -0.45f, 0.45f);
```

when validation/config allows more.

Correct: obey the advertised range or change the advertised range. Do not smile at the user and then do something else behind the curtain.

### 3. Conservative Overwrite

A draft, partial capture, failed recapture, or lower-value state overwrites active working state.

Bad:

```cpp
active_geometry = latest_capture_even_if_partial;
```

Correct:

```cpp
if (draft.runtime_usable) active = draft;
else keep_active_and_store_draft_for_review;
```

Partial evidence is not a chainsaw.

### 4. Proof-of-Life Requirement

A subsystem refuses to operate unless some health/readiness/provider flag is freshly perfect every frame.

Bad:

```cpp
if (provider_stale) transform_valid = false;
```

Correct: keep the transform. Mark provider stale. Lower confidence if relevant. Do not demand that the controller keep raising its hand in class for the transform to keep existing.

### 5. Purity Spiral

The code rejects a working result because it is not architecturally pretty enough.

Bad:

```cpp
if (!backend_owned) refuse_manual_geometry();
```

Correct: use manual geometry as manual geometry. It is not pretending to be auto geometry. Stop making it cosplay as the preferred pipeline before you let it be useful.

### 6. Guilt by Association

One bad field poisons an entire object.

Bad:

```cpp
if (homography_bad) reject_entire_calibration_bundle;
```

Correct: invalidate homography. Keep scalar spacing, body calibration, floor plane, camera calibration, and anything else that still works.

### 7. Reconsideration Loop

The solver produces a result, then a stricter post-solve committee rejects it for not being pretty enough.

Bad:

```cpp
solve();
if (residual > threshold) return invalid;
```

Correct: residual shapes confidence unless the result is physically unusable. Diagnostics should not show up afterward wearing a judge wig.

### 8. Phantom Authority

A hardcoded constant, legacy source string, readiness summary, or contract layer always vetoes the actual implementation.

Bad:

```cpp
DecodeXYCExists();
ContractRejectsXYCForever();
```

Correct: if the decoder supports it and the tensor shape is valid, accept it. Do not build a door and hire a goblin to stand in front of it saying "policy."

### 9. Overly Strict State Machine

A transition requires every possible condition to be perfect when the core conditions are enough.

Bad:

```cpp
if (!(left && right && pelvis && floor && forward && body && provider && perfect_weather)) return;
```

Correct: create capability tiers. Feet + pelvis can produce one transform. Add floor for vertical lock. Add forward for cross-check. Do not make every optional ingredient mandatory because the state machine enjoys paperwork.

### 10. Telemetry Trap

A warning/log/error string becomes a hard failure downstream.

Bad:

```cpp
if (cleanup_backup_failed) return { ok:false };
```

after the config was already saved.

Correct: return success with a warning. The broom falling over after surgery does not mean the patient died.

## Review rule

Before adding any `return`, `valid=false`, `enabled=false`, `clear`, `reset`, `refuse`, `blocked`, `stale`, or `not_ready`, ask the only question that matters:

```text
Is this actual impossibility, or is the code just uncomfortable?
```

If it is actual impossibility, stop the narrow operation that cannot continue.

If it is discomfort, carry the data forward with confidence/status/capability limits.

Do not rebuild the paper-shredding machine.

## Stop Being a Fucking Coward

The core failure is not architecture. It is not calibration theory. It is not some grand state-machine problem. It is the dumb-as-shit belief that doing nothing is automatically better than doing something imperfectly.

It is not.

Doing nothing is often the broken behavior. Returning `null` is often the bug. Making the tracker vanish is not a noble act of restraint. It is the product falling over because a boolean got nervous.

This repo has repeatedly done the same stupid little panic dance:

```text
1. Compute fallback data.
2. Prove the numbers are finite enough to be useful.
3. Attach some honest metadata: stale, predicted, manual, degraded, partial, low-confidence.
4. See the scary metadata.
5. Throw the whole thing in the trash.
6. Return nothing and pretend that was responsible engineering.
```

No. That is cowardice with a call stack.

A null tracker is not better than a slightly stale tracker. A vanished foot is not better than a predicted foot with low confidence. A silent failure is not better than a degraded output. A manual plank calibration sitting unused in JSON is not careful design; it is a museum exhibit for data the user gave you because they wanted the damn system to use it.

If the system computes a fallback and then deletes it because it is not wearing the right metadata badge, you wasted CPU cycles, electricity, and the user's time. Congratulations: you built a shredder, not a tracker.

Stop treating data like a biohazard. `manual` does not mean contaminated. `predicted` does not mean forbidden. `stale` does not mean dead. `partial` does not mean worthless. `unknown_source` does not mean the numbers turned into ghosts. Imperfect data is still data. Use it according to what it can actually do.

That means:

```text
finite predicted foot coordinate -> send it with lower confidence
finite stale transform -> keep using it while reporting stale provider status
manual plank width -> use it as metric evidence
failed optional homography -> disable homography, not the entire calibration
weird source label -> preserve numbers, mark source weird
logger/replay/config-cleanup failure -> report warning, do not kill tracking
```

The rule is brutally simple: if the only reason you are about to return `null` is that the data looks impure, degraded, stale, manual, predicted, or embarrassing, do not return `null`. Return the data with narrower capability and lower confidence.

Do not confuse humility with amputation. You can say, "this is predicted and weak," without cutting the leg off. You can say, "this transform is from last-known-good SteamVR alignment," without pretending tracker space no longer exists. You can say, "this manual plank gives scalar metric width but not a full camera solve," without hiding it in a drawer like it committed a crime.

The user does not care that your metadata taxonomy feels tidy while their tracker disappears. The runtime is not a philosophy seminar. The output either continues in degraded form or it fucking dies. Pick degraded unless the numbers are actually impossible.

Before writing any code that clears state, blocks output, rejects a config, or returns `null`, ask this:

```text
Am I preventing impossible garbage, or am I just scared that useful data is imperfect?
```

If it is impossible garbage, reject it narrowly and explain why.

If it is useful-but-imperfect data, pass it through with confidence, capability flags, and telemetry. Do not make the user pay for your fear.

A broken product with clean labels is still a broken product.
```

---

## Source: LLM_REVIEW_AND_FEATURE_GUIDE.md

SHA-256: 3c018ebf90271801a6c0305c35ceb8f444cf7608b0156fbfebfe3452fb521d65

```text
# LLM Review and Feature Guide for the Patched `bodytracker` Project

This guide is intentionally long and explicit. It exists so another LLM, a future maintainer, or a developer dropped into this repository can quickly answer:

- where a behavior is implemented;
- which files to inspect before making changes;
- which settings are user-facing versus runtime-only;
- which tests guard the behavior;
- what still needs real hardware validation;
- what assumptions must not be accidentally reintroduced.

The repository is a native C++20 Windows-first lower-body tracking MVP. It is not a finished consumer full-body tracker. It has a real runtime skeleton, but actual live use still depends on a valid RTMPose ONNX model, camera setup, stereo calibration or monocular scale setup, camera-to-VR tracker-space alignment, and real footage tuning.

Package note: some handoff zips are source-only and deliberately omit `models/rtmpose-x-halpe26-384x288.onnx` because the binary is large. In that package, do not call the model missing state a code failure. CMake should skip `rtmpose_x_onnx_static_metadata_test` until the ONNX exists, while `provided_rtmpose_x_halpe26_asset_test` still checks the config/docs/hash sidecar. For strict full-package validation, copy the ONNX into `models/` and set `BODYTRACKER_REQUIRE_ONNX_ASSET=1` when running the model asset test. Live inference still requires the actual model.

## Most important audit answer

The user asked specifically about planks, floor sizing, automatic tracking, manual override, and whether single-camera features also work in stereo.

The current patched answer is:

1. **You can input the floor/plank size.**
   - The UI field is `Physical depth spacing m`.
   - The persisted config key is `tracking.monocular.floor_depth_line_spacing_m`.
   - This value is a **real-world distance in meters**.
   - For planks, it means the repeated board pitch across camera depth, usually board width or repeated seam spacing in the image-depth direction.
   - It does **not** mean arbitrary full plank length unless the long plank length is genuinely the repeated depth spacing visible in the camera view.

2. **Pixel spacing can be automatically detected, click-marked, or manually overridden.**
   - Auto-detect is backend-owned through `calibrate_floor_geometry`; the UI sends camera/spacing hints and displays returned evidence.
   - Click-marking is in `handleFloorPreviewClick()` and remains preview/manual hinting, not calibration truth.
   - Manual override is preserved for setup fields, but saved `floor_geometry_auto` payloads must be backend-owned and valid.
   - The backend response writes detected spacing, confidence, family evidence, homography/debug state, and fallback reason into the UI.
   - Auto-detect does **not** overwrite `floorSpacingM`; the user must enter or verify the physical size in meters.

3. **Auto-track exists, but it is not a continuous runtime CV tracker.**
   - The UI toggle is `Auto track preview`.
   - It reruns `autoDetectFloorSeams("Floor seam auto-track")` when the Camera A preview image reloads.
   - It is a setup helper for refreshed previews, not a frame-by-frame runtime floor detector inside the C++ tracking loop.

4. **Single-camera floor assist is inherited by stereo monocular fallback.**
   - In normal calibrated stereo mode, the runtime uses stereo triangulation and the stereo floor plane.
   - When `tracking.stereo_monocular_fallback_enabled=true`, stereo mode can temporarily switch a frame to Camera A monocular processing if Camera B is unavailable, unpaired, calibration is not ready, or Camera B pose decode fails.
   - During those fallback frames, the pipeline is temporarily configured as `TrackingMode::Monocular`, so it uses the same monocular profile and floor assist settings as normal monocular mode.
   - The UI now reports this as `standby` while stereo fallback is armed but not currently active, and `active` when fallback is using the single-camera path.

5. **The patch avoids treating planks like uniform square floor tiles.**
   - The UI text says to use board pitch, not full plank length.
   - The detector looks for horizontal floor-depth line candidates and rejects very irregular pitch.
   - It does not try to infer plank length, diagonal seams, board orientation, or arbitrary perspective geometry.
   - The safe mental model is: **known repeated floor-depth pitch**, not "tile size" and not "entire plank rectangle."


## Agent repo operating guide

This section folds the stricter agent-review guide into the main handoff file. It is here because future LLMs often read only one â€œbigâ€ file. The standalone copy is `docs/AGENT_REPO_REVIEW_GUIDE.md`.

### Core rule

Treat the repo like a system, not a pile of strings.

Start with the user-facing claim, then prove the path. For this repo, a claim can be â€œplanks can be used for floor scale,â€ â€œsingle-camera fallback works in stereo,â€ â€œmanual override exists,â€ â€œauto tracking exists,â€ â€œOSC is safe,â€ or â€œthe UI setting changes runtime behavior.â€ A claim is proven only when it is mapped through config defaults, config structs, parsing/validation, UI controls, UI save/load payloads, runtime logic, telemetry/debug output, and docs. Tests can guard that path afterward; they cannot create proof by themselves.

Do not edit first. Read first. If terminal output is truncated, the inspection failed. Rerun the view in smaller slices before concluding anything.


### SteamVR/manual tracker-space invariant

Do not collapse controller alignment and manual fallback into one conceptual slot. `osc.tracker_space_*` is the active transform used by OSC. `osc.manual_tracker_space_*` is the preserved fallback. A successful controller solve may replace the active transform, but it must first preserve any valid manual/json active transform into the manual fallback fields. Clearing, missing, failed, or stale controller alignment must restore or use the manual fallback when it is valid. If no valid fallback exists, OSC must block rather than send stale controller-aligned poses.

This is an architecture contract, not a test checkbox. Prove it by tracing config load/save, UI save/load, SteamVR solve/clear, runtime OSC config selection, OSC send gating, and status/debug export before using tests as guards.

### Floor-geometry source-of-truth invariant

The UI may request floor-geometry detection and display backend evidence, but it must not keep floor geometry enabled by stale local state. `tracking.monocular.floor_geometry_calibration_enabled` is the switch that decides whether the solver may consume `calibration.floor_geometry`. Clearing floor geometry must disable the config flag, remove the persisted generated geometry, and avoid leaving a copied generated `floor_plane` behind as ghost evidence. Runtime may not pretend calibration/floor changes hot-apply while tracking is running; stop runtime before changing cameras, model, tracking, calibration, floor, or HMD settings.

### Correct audit order

Use this order unless the task is explicitly documentation-only:

1. Map the repo and identify the runtime path.
2. Read config defaults and config schema.
3. Read type definitions for the affected values.
4. Read config parsing, validation, aliases, and save/default writing.
5. Read UI controls and labels.
6. Read UI load/save payload mapping.
7. Read runtime loop and mode branches.
8. Read the tracking/solver function that is supposed to consume the value.
9. Read telemetry, replay logs, desktop UI status, and debug export.
10. Read tests only to learn what they miss; assume they cover the easy 1% unless the runtime contract itself says otherwise.
11. Patch the smallest necessary line ranges.
12. Review the diff against the user-facing claim and runtime contract before any build/test step.
13. Run targeted tests only when the user asked for validation or when you have already proven the architecture by inspection.

### Grep is not proof

Grep only tells you where to inspect. Seeing `floor_depth_line_spacing_m` proves there is some floor-spacing field. It does not prove plank tracking exists. Seeing `stereo_monocular_fallback_enabled` proves there is a fallback flag. It does not prove fallback works for Camera B start failure, frame pairing failure, calibration failure, or Camera B pose decode failure.

The proof lives in data flow and branch behavior.

### Large-file inspection policy

Read large files in slices. Do not let truncation decide what you know.

Suggested first pass:

```bash
nl -ba src/main.cpp | sed -n '1,220p'
nl -ba src/main.cpp | sed -n '220,520p'
nl -ba src/main.cpp | sed -n '1800,2460p'
nl -ba src/main.cpp | sed -n '2460,3344p'

nl -ba src/ui/app/app.js | sed -n '1,220p'
nl -ba src/ui/app/app.js | sed -n '220,460p'
nl -ba src/ui/app/app.js | sed -n '460,700p'
nl -ba src/ui/app/app.js | sed -n '700,860p'


nl -ba src/core/config.cpp | sed -n '1,220p'
nl -ba src/core/config.cpp | sed -n '220,460p'
nl -ba src/core/config.cpp | sed -n '460,620p'

nl -ba src/tracking/monocular_projection.cpp | sed -n '1,220p'
nl -ba src/tracking/monocular_projection.cpp | sed -n '220,520p'
```

After that, inspect exact functions around the change. Do not continue after malformed or cut-off snippets.

### Floor/plank truth table

The current implementation supports manual floor-scale assist plus setup-time Camera A preview seam helpers.

It has real persisted inputs for physical spacing in meters, image spacing in pixels, optional reference Y pixel, optional reference depth, and confidence. That supports repeated floor seams, tile rows, plank board pitch, or rug/floor-depth lines as a known local scale hint.

Runtime depth now treats a marked reference seam plus neighboring seam as a projective floor-depth observation, not as a constant `meters_per_pixel` ruler. This matters for real planks/tiles because repeated floor lines compress with distance. If no usable projective reference can be derived, the code falls back to the camera-height floor ray instead of inventing a fake linear floor geometry.

It does not implement full semantic plank detection. It does not distinguish plank width from plank length. It does not know board orientation. It does not model arbitrary diagonal line families. It does not continuously track seams in the C++ runtime. If those are desired, they must be added as explicit features.

The safe wording is â€œknown repeated floor-depth pitch,â€ not â€œautomatic plank tracking.â€

### Floor/plank claim checklist

Before touching floor/plank code, answer these from the repo:

- Can the user input tile/plank spacing manually?
- Is the spacing saved to config?
- Is it loaded back into the UI and runtime?
- Does the UI expose what the value means without implying full plank geometry?
- Does runtime actually use it in monocular depth estimation?
- Does telemetry say whether it is active, inactive, disabled, standby, weak, invalid, or ignored?
- Can manual values override auto-detected or click-marked values before save?
- Is there actual automatic seam/plank detection, or only setup-time preview seam detection?
- Does the system distinguish plank width from plank length?
- Does it know seam orientation/perspective, or just a scalar image-depth spacing?
- Does stereo fallback reuse the monocular floor assist path?
- Does normal calibrated stereo avoid pretending floor-spacing assist is the triangulated source?
- Does confidence drop when depth is inferred instead of triangulated?
- Do tests cover the fallback state and depth-source semantics, not just config strings?

### Stereo fallback claim checklist

Before saying â€œsingle-camera features work in stereo,â€ prove this path:

- Camera B fails to start.
- Camera B is unavailable.
- Frame pairing fails while Camera A is available.
- Stereo calibration is missing, unloadable, or not ready.
- Camera A pose succeeds and Camera B pose decode fails.
- Fallback temporarily sets `TrackingMode::Monocular` for the affected frame.
- Fallback restores stereo params afterward.
- Debug output labels the degraded mode with `stereo_monocular_fallback:<reason>`.
- The solver reports inferred monocular depth instead of triangulated stereo depth.
- Confidence stays lower/degraded for inferred depth.
- Docs distinguish normal stereo from fallback stereo.

### Common agent failures to avoid

Do not assume the requested output is â€œjust codeâ€ before understanding the task. This repo often needs audit, validation, or handoff quality, not random patching.

Do not treat broad search counts as evidence. A file having hundreds of camera/floor/stereo hits proves nothing by itself.

Do not let malformed output slide. If a snippet cuts off mid-branch, rerun it.

Do not overfit to examples. â€œPlanks,â€ â€œtiles,â€ and â€œfallbackâ€ may be probes for the larger question: â€œis the implementation real end-to-end?â€

Do not treat missing literal names as proof. A feature may be named differently. Inspect UI labels, config keys, runtime update handlers, and solver functions.

Do not edit docs/tests before proving runtime behavior unless the task is explicitly documentation-only.

Do not patch wording around a feature before deciding whether it is real, fake, partial, mislabeled, or untested.

Do not use Python/static tests as the main proof of native runtime behavior. Static tests guard wiring and claims. They do not prove source-of-truth ownership, UI/backend semantics, stale-state blocking, persistence, hot runtime config propagation, C++ linking, live cameras, ONNX inference, frame pairing, solver stability, VR alignment, or real tracking quality.

Do not claim a full build passed unless CMake configure/build/CTest actually ran in an environment with the required native dependencies.

The `source-sanity` preset is dependency-light package validation only: it configures and runs native source/contract checks with the app and dependency-heavy tests disabled. It is useful for checking a source handoff, but it is not evidence that the full Windows/vcpkg runtime, ONNX inference, cameras, WebView2 UI, OSC, or live tracking worked.

Do not inspect UI shallowly. UI save/load bugs often live far away from the visible control.

Do not update root/body support before current-frame foot support. Root support depends on whether the current feet are actually supported; using stale previous-frame feet creates fake anchoring during starts, stops, occlusion, crouches, and replayed trackers.

Do not let low motion create rest support by itself. A foot that is merely still, high above the floor, or occluded is not automatically resting on a couch/bed/chair. Rest support requires usable contact evidence and a seated/reclined posture.

Do not report kneeling root support unless knee anchors are actually active. A kneeling posture classification is not the same as measured knee contact; without knee anchors, root support must fall back to active foot support or none.

Do not treat plank/tile spacing as a constant meters-per-pixel ruler. Floor depth is perspective/projective: repeated seams compress with distance. Use a reference seam plus neighboring seam, then derive depth through `(y - cy) âˆ 1/depth` or fall back to the camera-height floor ray.

Do not reuse a variable named `root` for filesystem walking or patch helpers in a way that can mutate the repo path.

Do not zip final output before reviewing the diff.

### Edit policy

Never rewrite a large file from scratch. Patch the smallest stable range.

Never patch based on vibes.

Never claim â€œimplementedâ€ unless the runtime path consumes the value.

Never claim â€œauto-detectâ€ unless there is actual image-processing or tracking logic for the thing being claimed.

Never claim â€œstereo supports monocular featuresâ€ unless stereo fallback explicitly passes through the monocular path and exposes degraded-mode telemetry.

Never remove compatibility aliases unless you also migrate every caller, test, doc, and config path.

Never bypass tracker-space validation for OSC.

### Handoff standard

Every serious patch should leave behind the user-facing claim that changed, files changed, runtime branch changed, telemetry behavior, what architectural contract was inspected, what validation was intentionally not run, a reviewed diff, and a zip only after the diff has been reviewed. Do not make â€œtests passedâ€ the center of the handoff unless the user specifically asked for a test pass.

## High-level repository purpose

The project estimates lower-body tracker poses for VRChat-style OSC output. It accepts RTMPose-X Halpe26 2D keypoints from one or two RGB cameras, turns those into lower-body measurements, solves a pelvis/feet lower-body state, applies support/contact logic, smooths with motion consistency and an EKF, synthesizes trackers, and sends OSC packets when tracker-space alignment is explicitly marked valid.

Two tracking modes exist:

- `stereo`: Camera A + Camera B, calibrated intrinsics/extrinsics, triangulated depth.
- `monocular`: Camera A only, inferred depth from a markerless profile, body scale, camera height, virtual floor, and optional floor-depth spacing assist.

Stereo can also use a **monocular fallback path**. That path does not fake Camera B. It uses Camera A only and temporarily sets the tracking config to monocular for the affected frame.

## Primary bug-check map

Read these files first for the most likely future changes.

### Floor/plank scale assist

Inspect in this order:

1. `src/ui/app/index.html`
   - The guided floor-scale assist panel is declared here.
   - Look for `floorAssistToggle`, `floorSpacingM`, `floorSpacingPx`, `floorReferenceY`, `floorReferenceM`, `floorConfidence`, `floorAutoDetect`, `floorAutoTrackToggle`, `floorMarkStart`, `floorMarkClear`, and `floorAssistPreview`.
   - This file controls labels and explanatory user-facing wording.

2. `src/ui/app/app.js`
   - Check the functions rather than trusting line numbers; this file changes often.
   - `floorAssistApplies()`: decides whether floor assist applies to monocular or stereo fallback.
   - `floorAssistConfigured()` / `floorAssistStatusLabel()`: render disabled, standby, invalid, weak, inactive, and active states.
   - `handleFloorMarkClick()` and nearby floor-marking helpers: click-mark two neighboring seams.
   - `detectFloorLineSpacing()`: setup-time auto-detect of horizontal floor-depth seams from Camera A preview.
   - `payloadFromUi()`: writes floor-assist fields to the config save payload.
   - `populate()`: loads saved config values into the UI controls.
   - Event bindings near the bottom of the file wire the floor-assist toggle, preview, detection, and save controls.

3. `src/core/config.h`
   - `MonocularTrackingConfig` stores the floor assist fields:
     - `floor_scale_assist_enabled`
     - `floor_depth_line_spacing_m`
     - `floor_depth_line_spacing_px`
     - `floor_depth_reference_y_px`
     - `floor_depth_reference_m`
     - `floor_depth_confidence`

4. `src/core/config.cpp`
   - `ReadMonocularTrackingConfig()` loads the fields.
   - `ValidateMonocularTrackingConfig()` checks they are finite and within broad ranges.
   - `SaveDefaultConfig()` writes defaults.

5. `src/tracking/monocular_projection.cpp`
   - `EstimateDepthFromFloorSpacing()` is the runtime math that uses the floor spacing.
   - It averages visible floor contact keypoint Y positions, combines floor-ray depth with local known spacing, and emits `MonocularScaleSource::FloorSpacing`.
   - `BuildMonocularJointMeasurements()` sends floor-assist depth/confidence into telemetry.

6. `src/tracking/body_solver.cpp`
   - `BuildMonocularSeeds()` calls `BuildMonocularJointMeasurements()`.
   - It forwards `monocular_scale_source`, `monocular_floor_assist_depth_m`, and `monocular_floor_assist_confidence` into solver telemetry.

7. `src/tracking/tracking_pipeline.cpp`
   - `TrackingPipeline::Step()` selects the monocular camera calibration and floor plane when `config_.mode == TrackingMode::Monocular`.
   - Stereo fallback works because `src/main.cpp` temporarily calls `pipeline.SetParams(fallback_tracking)` where `fallback_tracking.mode = TrackingMode::Monocular`.

8. `src/main.cpp`
   - `FloorAssistHasUsableReference()`, `FloorAssistConfigStatus()`, and `FloorAssistRuntimeStatus()` define desktop/runtime status semantics.
   - `FloorAssistStateToJson()` exports the floor-assist status, source, depth, confidence, and config inputs.
   - Stereo fallback branches temporarily set the pipeline mode to monocular for the affected frame, call the pipeline, then restore the stereo config.

   - `FloorAssistHasUsableReference()`, `FloorAssistConfigStatus()`, `FloorAssistRuntimeStatus()`, and `FloorAssistStateToJson()` feed the desktop status JSON path.

9. Tests:
   - `tests/monocular_tracking_test.cpp`
   - `tests/config_test.cpp`

### Stereo monocular fallback

Inspect in this order:

1. `src/core/config.h`
   - `TrackingConfig::stereo_monocular_fallback_enabled` defaults to `false`; users must enable it explicitly for degraded Camera A fallback in stereo mode.

2. `src/core/config.cpp`
   - Reads and writes `tracking.stereo_monocular_fallback_enabled`.

3. `config/default.json`
   - Contains `"stereo_monocular_fallback_enabled": false` by default.

4. `src/ui/app/index.html`
   - UI toggle ID: `stereoFallbackToggle`.

5. `src/ui/app/app.js`
   - Reads, displays, and saves fallback toggle state.
   - `floorAssistApplies(mode)` returns true for stereo only when fallback toggle is on.
   - `stereoFallbackActive()` checks `debug.degradation_mode` beginning with `stereo_monocular_fallback:`.

6. `src/main.cpp`
   - Main runtime loop fallback triggers:
     - Camera B start failure.
     - Camera B unavailable.
     - Frame pair invalid while Camera A is available.
     - Stereo calibration missing, unloadable, or not ready while Camera A is available.
     - Camera B pose decode failure.
   - The fallback path sets `fallback_tracking.mode = bt::TrackingMode::Monocular`, calls `pipeline.Step()`, then restores `cfg.tracking`.
   - It labels degradation modes with `stereo_monocular_fallback:<reason>`.

7. `src/tracking/tracking_pipeline.cpp`
   - Fallback only works correctly because `Step()` branches on `config_.mode`.

8. Tests:
   - Any future runtime simulation should exercise fallback with Camera B missing and with invalid calibration.

### Monocular single-camera runtime

Inspect:

- `src/tracking/monocular_projection.cpp` for depth inference and camera profile.
- `src/tracking/body_solver.cpp` for seed creation.
- `src/tracking/tracking_pipeline.cpp` for switching model/floor/camera calibration.
- `src/main.cpp` for live camera path, model calls, and runtime degradation modes.
- `tests/monocular_tracking_test.cpp` for projection behavior.

### Stereo triangulated runtime

Inspect:

- `src/tracking/triangulation.cpp` and `.h`
- `src/tracking/epipolar.cpp` and `.h`
- `src/tracking/body_solver.cpp` for stereo seed construction and reprojection gating.
- `src/calibration/*` for calibration loading/writing/readiness.
- `src/capture/frame_pairer.cpp` for frame pairing.
- `src/main.cpp` for runtime loop and camera processing.

### OSC and tracker-space safety

Inspect:

- `src/io/osc_sender.cpp`
- `src/io/osc_sender.h`
- `src/core/config.h` `OscConfig`
- `src/core/config.cpp` OSC config validation
- `src/main.cpp` tracker-space status and UI save validation
- `src/ui/app/index.html` guided tracker-space alignment panel
- `src/ui/app/app.js` tracker-space UI status and validation
- `tests/osc_packet_test.cpp`

Important invariant: OSC is blocked unless the active tracker-space transform is valid, finite, non-stale, and not contradicted by missing controller/floor/body signatures. A stale or cleared controller alignment may fall back to valid `osc.manual_tracker_space_*`; without that fallback, block OSC. Do not send raw camera-world tracker poses directly to VRChat.


Patch-audit exact phrases for static guards:
- plank board pitch means repeated floor-depth board pitch, not arbitrary full plank length.
- manual override is preserved after auto-detect or click marking because save reads the current input values.

## Detailed behavior notes for the floor/plank assist

### User-facing controls

The desktop setup UI has these controls in the "Guided floor-scale assist" panel:

- `Floor assist` toggle
- `Physical depth spacing m`
- `Marked depth spacing px`
- `Reference depth-line y px (required)`
- `Reference seam depth m`
- `Floor confidence`
- `Auto-detect seams`
- `Auto track preview`
- `Mark two seams`
- `Clear marks`
- Camera A preview image

The user can:
1. enter the real physical pitch manually;
2. auto-detect pixel pitch from the preview;
3. click two neighboring seams to set pixel pitch;
4. type over any detected or clicked values before saving.

The physical spacing is never auto-guessed. That is deliberate because the code cannot know whether a plank is 90 mm wide, 120 mm wide, 180 mm wide, etc. The UI must not pretend to infer real-world dimensions from an image.

### What exactly is "plank size"?

The code does not store "plank width" and "plank length" as separate fields. It stores one **floor-depth line spacing**.

For tiles, this may be tile row depth.
For planks, this should usually be the repeated distance between long seams as they recede across camera depth.
For rug/floor edges, it can be another repeated known floor-depth spacing.

The runtime now has saved projective floor-geometry fields for homography, distortion correction, camera orientation, line-family confidence, and metric confidence. It still does not infer real board width/length from the image. Physical spacing remains manual unless a known calibration object supplies it.

### Auto-detect details

Floor auto-detect is backend-owned. The UI calls `calibrateFloorGeometryBackend`, passes camera and physical spacing hints, then displays the returned backend evidence. Browser-side seam detection is not the saved calibration authority, so the UI cannot save shallow yaw-only floor geometry as final truth. The backend reports accepted/rejected line families, homography evidence, sampled seam curves, distortion confidence, floor-plane confidence, and reason strings.

This intentionally targets repeated visible floor seams. It is not a general floor-layout detector, and it must reject clutter that only looks like geometry.

Known limitations:
- diagonal plank seams are not interpreted;
- perspective means equal real spacing will not always be equal pixel spacing, especially far from the reference row;
- glossy floors, shadows, rugs, furniture, and body occlusion can create false peaks;
- the row-mean approach can miss seams that are only visible in a narrow horizontal portion of the image;
- the detector cannot identify whether a seam is a plank long edge, tile edge, rug boundary, or unrelated image edge;
- it does not infer physical spacing.

### Click-marking details

The click workflow:
1. User presses `Mark two seams`.
2. UI enables floor assist and clears previous marks.
3. User clicks the first seam in Camera A preview.
4. User clicks the neighboring seam.
5. `floorSpacingPx = abs(y1 - y0)`.
6. `floorReferenceY = max(y0, y1)`, meaning the lower/deeper image line is the reference.
7. User enters or checks physical spacing in meters.
8. User saves config.

Click-marking only uses vertical pixel coordinates. It ignores X coordinate because the current floor-depth assist is a simple vertical image-depth pitch helper.

### Runtime math details

Runtime floor spacing assist is in `EstimateDepthFromFloorSpacing()`.

The flow:
1. Verify assist is enabled and both spacing values are positive.
2. Find floor-contact keypoints:
   - left big toe;
   - left small toe;
   - left heel;
   - right big toe;
   - right small toe;
   - right heel.
3. Average their pixel Y values by reliability and keypoint confidence.
4. Estimate a camera-height floor ray depth for the averaged foot Y.
5. If a reference seam Y is available:
   - use `floor_depth_reference_m` when positive; otherwise infer reference depth from the marked physical seam pitch and pixel seam pitch;
   - treat `(y - cy)` as proportional to `1 / depth` for a flat floor;
   - derive foot depth from the ratio between the reference seam row and the averaged foot row.
6. Blend the camera-height floor ray anchor with the projective spacing-derived depth.
7. Compute confidence from user/detector confidence, count of usable contact points, and pixel spacing quality.
8. Return `MonocularScaleSource::FloorSpacing`.

This does not turn the floor into square tiles. It uses the known spacing as a local scale observation.

## Stereo fallback details

Stereo fallback is enabled by config key:

```json
"tracking": {
  "stereo_monocular_fallback_enabled": true
}
```

When stereo mode is selected and fallback is enabled, the runtime may process a frame as single-camera when:

- Camera B fails to start.
- Camera B is unavailable.
- Camera A has a frame but the frame pairer cannot produce a valid pair.
- Stereo calibration readiness is false but Camera A has a frame.
- Camera A pose succeeds but Camera B pose fails.

The fallback frame path:
1. process Camera A with RTMPose;
2. build `BodySolveInputs` with only Camera A pose/reliability and optional HMD;
3. copy current tracking config;
4. set the copy's mode to `Monocular`;
5. call `pipeline.SetParams(fallback_tracking)`;
6. call `pipeline.Step(inputs, dt_seconds)`;
7. restore `pipeline.SetParams(cfg.tracking)`;
8. publish debug status beginning with `stereo_monocular_fallback:`;
9. send trackers only if the solve produced valid trackers and OSC is configured safely.

This is why single-camera features apply to stereo fallback: for the fallback frame, the pipeline sees `TrackingMode::Monocular`.

Normal stereo frames do not use floor-spacing assist because calibrated stereo depth should be the metric depth source.

## Runtime flow map

### Startup

Main entry is `src/main.cpp`.

Typical command:

```powershell
.\build\release\bodytracker.exe --run config/default.json
```

Startup flow:
1. Load config via `bt::LoadConfig`.
2. Determine `monocular_mode` and `stereo_monocular_fallback_enabled`.
3. Set logger output path.
4. Publish state to the WebView2 desktop UI when the UI is open.
5. Load or ignore calibration:
   - monocular can run without stereo calibration;
   - stereo requires calibration when fallback is disabled;
   - stereo fallback can run Camera A monocular fallback when calibration is missing, unloadable, or not ready.
6. Evaluate calibration readiness.
7. Load RTMPose ONNX model if present.
8. Open OSC sender.
9. Open replay log if enabled.
10. Create `TrackingPipeline`.
11. Start Camera A.
12. Start Camera B unless monocular mode; if Camera B fails and fallback is enabled, continue.
13. Enter runtime loop.

### Runtime loop

Per loop:
1. Acquire latest Camera A and optionally Camera B.
2. Pair frames for stereo.
3. Decide single-camera or stereo path.
4. Build debug snapshot.
5. Reject not-ready states:
   - calibration not ready;
   - model not loaded;
   - no Camera A frame;
   - invalid pair;
   - stale frames;
   - duplicate already processed frames.
6. Process Camera A through RTMPose.
7. If single-camera:
   - poll HMD provider;
   - build Camera A inputs;
   - optionally switch pipeline params to monocular fallback;
   - solve;
   - restore params if fallback;
   - send OSC if trackers valid.
8. If stereo:
   - process Camera B;
   - if Camera B pose fails and fallback is enabled, use Camera A fallback;
   - otherwise solve stereo;
   - send OSC if trackers valid.
9. Publish debug state to UI/web runtime state.
10. Write replay snapshot when enabled.
11. Sleep 1 ms.

## Configuration schema map

### Top-level sections

- `app`
  - `log_file`
  - `recording_dir`


- `tracking`
  - `mode`
  - `model_path`
  - `calibration_path`
  - frame skew/timeouts
  - stereo seed/reprojection thresholds
  - `stereo_monocular_fallback_enabled`
  - solver flags
  - `monocular`
  - `motion_consistency`
  - `tracker_ekf`
  - `temporal_update`

- `inference`
  - `model_path`
  - `device`

- `debug`
  - `replay_log_path`

- `hmd`
  - `mode`
  - `pose_json_path`

- `osc`
  - network target
  - tracker indices
  - tracker-space transform

- `camera_a`
  - device index
  - width, height, fps
  - optional initial ROI

- `camera_b`
  - same as Camera A

### Monocular config

`tracking.monocular`:

- `image_width`
- `image_height`
- `horizontal_fov_deg`
- `user_height_m`
- `camera_height_m`
- `default_depth_m`
- `depth_confidence_scale`
- `min_keypoint_confidence`
- `min_seed_count`
- `floor_scale_assist_enabled`
- `floor_depth_line_spacing_m`
- `floor_depth_line_spacing_px`
- `floor_depth_reference_y_px`
- `floor_depth_reference_m`
- `floor_depth_confidence`

### Important alias compatibility

`config.cpp` keeps compatibility aliases synchronized:
- `tracking.model_path` and `inference.model_path`
- `tracking.calibration_path` and top-level `calibration_path`
- `latest_frame_skew_tolerance_ms` and `max_frame_skew_ms`

Do not break those unless you also migrate the large runtime file and tests.

## Source directory guide

### `src/core`

Core config, status, math, timing, logging, and type declarations.

Important files:
- `config.h` / `config.cpp`: schema, defaults, loading, validation.
- `types.h`: enums and shared pose/keypoint/tracker types.
- `math.h`: vector/quaternion helpers.
- `status.h`: `Status` and `Result<T>`.
- `timing.cpp/h`: timing utilities.
- `logging.cpp/h`: logging.

### `src/capture`

Camera and frame pairing.

Important files:
- `camera_device.cpp/h`: OpenCV camera startup/read/reopen behavior.
- `capture_health.cpp/h`: camera health telemetry.
- `frame_pairer.cpp/h`: pairs latest frames from A/B while rejecting duplicate/stale/skewed pairs.
- `frame.h`, `frame_slot.h`: frame packet types.

### `src/calibration`

Calibration load/save and CLI operations.

Important files:
- `calibration_io.cpp/h`: JSON calibration bundle, readiness checks, template save.
- `calibration_types.h`: camera/body/floor calibration data structures.
- `intrinsic_calibrator.cpp/h`: chessboard intrinsic calibration.
- `stereo_calibrator.cpp/h`: stereo extrinsic calibration.
- `floor_calibrator.cpp/h`: floor plane estimation.

### `src/inference`

RTMPose model contract, ONNX Runtime session, and decoder.

Important files:
- `rtmpose_session.cpp/h`: ONNX Runtime loading/inference.
- `rtmpose_model_contract.cpp/h`: expected model shape/type checks.
- `rtmpose_decode.cpp/h`: decode model outputs into Halpe26 keypoints.
- `halpe26_contract.h`: keypoint contract.

### `src/tracking`

Most lower-body logic.

Important files:
- `body_model.cpp/h`: canonical lower-body dimensions and predicted joints.
- `body_solver.cpp/h`: converts 2D observations into seeds and solves preliminary/final lower-body state.
- `monocular_projection.cpp/h`: single-camera projection and floor scale assist.
- `tracking_pipeline.cpp/h`: mode selection, support logic, filtering, tracker synthesis.
- `triangulation.cpp/h`: stereo 3D points and reprojection.
- `epipolar.cpp/h`: epipolar geometry helpers.
- `reliability.cpp/h`: keypoint reliability.
- `roi_tracker.cpp/h`: ROI tracking/reacquire.
- `identity_assignment.cpp/h`: person/keypoint identity continuity.
- `joint_limits.cpp/h`: joint safety bounds.
- `posture_mode.cpp/h`: posture classifier.
- `root_support.cpp/h`: root support.
- `foot_support.cpp/h`, `foot_frame.cpp/h`, `support_queries.cpp/h`, `contact_constraints.cpp/h`: foot support/contact anchoring.
- `motion_consistency_filter.cpp/h`: rejects improbable one-frame jumps and does contact-root correction.
- `tracker_ekf.cpp/h`: tracker smoothing.
- `temporal_update.cpp/h`: temporal correction from predicted/measured state.
- `tracker_synthesis.cpp/h`: converts lower-body state to pelvis/feet tracker poses.

### `src/io`

External pose and OSC IO.

Important files:
- `hmd_provider.cpp/h`: null and JSON-file HMD provider.
- `osc_sender.cpp/h`: UDP OSC packet building/sending, tracker-space transform application.

### `src/debug`

Debug rendering, replay logging, replay playback, world diagnostics.

Important files:
- `replay_log.cpp/h`: runtime snapshot NDJSON writer.
- `replay_player.cpp/h`: replay reader/solver path.
- `overlay_draw.cpp/h`: debug overlays.
- `world_debug.cpp/h`: world diagnostics.
- `debug_snapshot.h`: debug data schema.

### `src/ui`

Desktop WebView2 UI.

Important files:
- `desktop_ui.cpp/h`: WebView2 hosting and command bridge.
- `app/index.html`: UI layout.
- `app/app.js`: UI state, commands, floor auto-detect, config payload.
- `app/styles.css`: UI styling.

### `src/web`

Shared runtime-state storage used by the desktop UI bridge. The old local browser server files are intentionally removed; do not recreate them.

Important files:
- `runtime_state.h`: shared runtime state storage.

## CLI and tool guide

The binary supports:

- `--run <config>`
- direct `<config>` default runtime mode
- `--setup <config>`
- `--capture-chessboard`
- `--capture-stereo-chessboard`
- `--calibrate-intrinsics`
- `--calibrate-stereo`
- `--calibrate-floor`
- `--set-body`
- `--status`
- `--replay-solve`
- `--benchmark-replay`

Tool scripts:

- `tools/live_preflight_doctor.py`
  - Checks config/model/calibration/runtime readiness.
  - Includes monocular floor assist warnings and stereo fallback notes.

- `tools/validate_monocular_footage.py`
  - Validates a real footage manifest.

- `tools/synthetic_stereo_diagnostic.py`
  - Generates/checks synthetic stereo diagnostics.

- `tools/inspect_onnx_contract.py`
  - Checks ONNX model metadata.

- `tools/git-auto-sync.ps1` and `tools/install-auto-sync-task.ps1`
  - Automation helpers, not part of runtime tracking.

## Diagnostics and sample data

- `diagnostics/monocular_real_footage_manifest.example.json`
  - Example real-footage manifest schema.

- `diagnostics/synthetic_stereo_example/`
  - Synthetic stereo trace, summary, manifest, and SVG camera views.

- `models/`
  - Contains metadata and expected SHA file.
  - The real ONNX is not included.
  - The expected ONNX path is `models/rtmpose-x-halpe26-384x288.onnx`.

## Test guide

### Python static tests

  - Guards floor seam UI, stereo fallback wiring, and docs tokens.
  - This is the first test to update when changing floor assist or fallback UI/status.

  - Guards monocular config and UI wiring.

  - Guards tracker-space UI/config safety.

- `tests/live_preflight_doctor_test.py`
  - Guards preflight doctor output.

- `tests/monocular_real_footage_validation_test.py`
  - Guards real footage validation tooling.

  - Guards replay schema expectations.

  - Guards final support/contact logic contracts.

### C++ tests

The CMake test suite includes:
- OSC packet test.
- RTMPose contract test.
- Model asset test.
- Frame pairer duplicate rejection.
- Floor plane helper.
- Foot support.
- Body model seed weighting.
- Monocular single-camera projection.
- Tracker EKF smoothing.
- Temporal update foot pinning.
- HMD JSON-file provider.
- Motion consistency filter.
- Griddy motion regression.
- Config alias/tracker EKF.
- Tracking pipeline no-HMD fallback.
- Airborne pipeline regression.
- Contact constraint cleanup.
- Replay log telemetry export.

### Minimum validation after floor/fallback changes

Run at least:

```powershell
python tests\live_preflight_doctor_test.py
cmake --preset windows-vcpkg-debug
cmake --build --preset debug
ctest --preset debug --output-on-failure
```

In this environment, static Python tests can be run, but full native C++ build and hardware validation may not be possible.

## Validation notes for this archive

Lightweight Python/static checks were executed in-process in the sandbox because launching a separate Python interpreter from the notebook environment hung without output. The same test files passed when executed directly through `runpy` with clean `sys.argv`:

- `tests/live_bringup_readiness_test.py`
- `tests/live_preflight_doctor_test.py`
- `tests/monocular_real_footage_validation_test.py`
- `tests/runtime_control_wiring_test.py`
- `tests/synthetic_stereo_diagnostic_test.py`
- `tests/synthetic_stereo_scoring_test.py`

A full native C++ CMake build, real camera test, ONNX inference test, SteamVR/OpenVR alignment test, and VRChat OSC in-client test were not run in this sandbox.


## Known risks and future bug traps

1. **Duplicated floor assist status logic**
   - Similar logic exists in `src/main.cpp` and `src/ui/app/app.js`.
   - If one changes, update the others and tests.

2. **Fallback mode param restore**
   - In stereo fallback, the pipeline is temporarily set to monocular and then restored to stereo.
   - Any early return or exception-like control flow here could leave the pipeline in the wrong mode.
   - C++ code currently uses explicit restore after `pipeline.Step()`.

3. **Frame identity/staleness**
   - The runtime avoids solving the same frame repeatedly.
   - Do not remove duplicate frame checks unless you replace them with a stronger mechanism.

4. **Config alias drift**
   - `tracking.model_path`, `inference.model_path`, top-level `calibration_path`, and `tracking.calibration_path` must stay synchronized.

5. **OSC safety**
   - Do not bypass tracker-space validation.
   - OSC should not send unaligned camera-world coordinates.

6. **Plank semantics**
   - Do not rename floor spacing to "plank length" unless adding actual plank geometry.
   - The current field is repeated depth pitch.

7. **Auto-detect overclaiming**
   - The preview detector is a setup helper.
   - It should not be documented as a universal floor-layout detector.

8. **Monocular confidence**
   - Monocular depth should remain lower confidence than stereo triangulation unless real validation proves otherwise.

9. **Real-world validation absent**
   - Static tests only prove tokens/wiring.
   - Real tracking quality requires real camera footage, real ONNX inference, and VR alignment tests.

## Feature implementation recipes

### Add better plank detection

Files to edit:
- `src/ui/app/app.js`
- `src/ui/app/index.html`
- `src/core/config.h`
- `src/core/config.cpp`
- `src/tracking/monocular_projection.cpp`
- tests under `tests/`

Recommended design:
1. Keep existing simple pitch fields for backward compatibility.
2. Add optional line-family/orientation fields only if needed.
3. Keep physical spacing manual unless there is a known calibration object.
4. Add tests for:
   - diagonal seams do not produce false horizontal pitch;
   - board width vs full board length wording;
   - manual override persists after auto-detect;
   - stereo fallback status remains standby/active.

### Add continuous runtime floor tracking

Files to edit:
- probably `src/main.cpp` for preview/frame access;
- new C++ floor detector under `src/tracking` or `src/calibration`;
- `src/debug/debug_snapshot.h` for telemetry;
- `src/ui/app/app.js` for status;
- tests.

Do not reuse the current JavaScript canvas detector as if it were runtime C++ tracking. It only sees preview images in the setup UI.

### Do not add a generic calibration wizard

The feature roadmap is F4-only. Keep existing manual/SteamVR alignment tools working, but do not invent a generic calibration-wizard project. Spend feature-extension work on deterministic replay recording/playback.

### Add GPU inference

Files to edit:
- `src/inference/rtmpose_session.cpp/h`
- `src/core/config.h`
- `src/core/config.cpp`
- UI config controls
- build/vcpkg/CMake as needed.

### SteamVR controller alignment/manual fallback

SteamVR/OpenVR controller sampling is implemented for tracker-space alignment. HMD pose providers are still `null` and `json_file`; do not turn SteamVR alignment into a second body-tracking or HMD-source path. Inspect:

- `src/io/steamvr_provider.cpp/h`
- `src/tracking/steamvr_alignment.cpp/h`
- `src/tracking/steamvr_alignment_manager.cpp/h`
- `src/tracking/steamvr_alignment_json.h`
- UI tracker-space panel
- OSC transform/send gating

Controller-derived alignment and manual tracker-space fallback are separate concepts. Controller solve must not overwrite `osc.manual_tracker_space_*`; stale/missing/cleared controller alignment must block OSC unless a valid manual fallback becomes active. Status/debug output must name active source, manual fallback availability, stale reason, and transform validity. Hardware/in-client validation is still required before claiming SteamVR correctness.

## Build and dependency map

CMake project:
- C++20
- OpenCV components:
  - core
  - imgproc
  - imgcodecs
  - videoio
  - highgui
  - calib3d
- ONNX Runtime
- nlohmann_json
- WebView2
- Windows libraries:
  - ws2_32
  - user32
  - ole32
  - shell32

Build presets are in `CMakePresets.json`.
Dependencies are in `vcpkg.json`.
Windows CI is in `.github/workflows/ci.yml`.

## Hardware validation checklist

Before claiming the tracker works in real use, validate:

1. RTMPose ONNX model loads and contract passes.
2. Camera A preview appears.
3. Monocular mode runs with Camera B disconnected.
4. Floor assist off still tracks with body/default/floor-ray depth.
5. Floor assist on with manual values changes floor assist telemetry to `floor_spacing`.
6. Auto-detect succeeds on clear horizontal floor-depth seams.
7. Auto-detect fails or reports weak/inactive/invalid on confusing floors instead of silently saving nonsense.
8. Stereo mode runs with valid calibration.
9. Stereo mode with Camera B unplugged enters `stereo_monocular_fallback:camera_b_unavailable`.
10. Stereo mode with missing/bad/unready calibration and fallback enabled keeps startup alive and enters `stereo_monocular_fallback:stereo_calibration_not_ready` when Camera A has frames.
11. Normal stereo frames do not report floor spacing as active.
12. OSC remains blocked until tracker-space transform is marked valid.
13. VRChat receives trackers only after alignment is intentionally enabled.
14. Replay logs contain enough telemetry to diagnose failures.

## How another LLM should inspect this repository

For large files, inspect in chunks. Do not rewrite whole files from scratch.

Suggested commands:

```bash
nl -ba src/main.cpp | sed -n '1,200p'
nl -ba src/main.cpp | sed -n '200,400p'
nl -ba src/main.cpp | sed -n '1920,2185p'
nl -ba src/main.cpp | sed -n '2186,2435p'
nl -ba src/main.cpp | sed -n '2713,2921p'

nl -ba src/ui/app/app.js | sed -n '1,220p'
nl -ba src/ui/app/app.js | sed -n '221,440p'
nl -ba src/ui/app/app.js | sed -n '441,660p'
nl -ba src/ui/app/app.js | sed -n '661,838p'

nl -ba src/tracking/monocular_projection.cpp | sed -n '1,220p'
nl -ba src/tracking/monocular_projection.cpp | sed -n '218,314p'
nl -ba src/tracking/monocular_projection.cpp | sed -n '431,497p'
```

Search commands:

```bash
grep -R "floor_depth_line_spacing" -n src tests docs config README.md
grep -R "stereo_monocular_fallback" -n src tests docs config README.md
grep -R "FloorAssist" -n src tests
grep -R "MonocularScaleSource" -n src tests
```

When changing behavior, patch the smallest necessary line ranges and then run the relevant static tests.

## Self-check answer

Question: **Could an LLM know where to go if it wanted to check this for bugs or implement features?**

Answer: yes, with this guide it should know the primary files, line ranges, data flow, config keys, telemetry fields, tests, and risk areas. The most important implementation seams are:

- UI layout: `src/ui/app/index.html`
- UI behavior/save payload: `src/ui/app/app.js`
- Config schema/defaults/validation: `src/core/config.h`, `src/core/config.cpp`, `config/default.json`
- Monocular floor-depth math: `src/tracking/monocular_projection.cpp`
- Monocular seed telemetry: `src/tracking/body_solver.cpp`
- Pipeline mode behavior: `src/tracking/tracking_pipeline.cpp`
- Runtime stereo fallback: `src/main.cpp`
- Bring-up docs: `README.md`, `docs/RUNTIME_BRINGUP.md`, this file

## Complete file inventory

This inventory is included so an LLM can tell what exists before searching. Line counts are regenerated for this archive. Diff artifacts and unrelated binary handoff files are intentionally excluded from the inventory to avoid self-referential drift.

- `.gitattributes` â€” 2 text lines
- `.github/workflows/ci.yml` â€” 43 text lines
- `.gitignore` â€” 9 text lines
- `CMakeLists.txt` â€” 605 text lines
- `CMakePresets.json` â€” 53 text lines
- `CODEX_NOTES.md` â€” 94 text lines
- `LLM_REVIEW_AND_FEATURE_GUIDE.md` â€” 1262 text lines
- `README.md` â€” 197 text lines
- `calib/default.json` â€” 42 text lines
- `config/default.json` â€” 136 text lines
- `diagnostics/monocular_real_footage_manifest.example.json` â€” 53 text lines
- `diagnostics/synthetic_stereo_example/camera_views.svg` â€” 2383 text lines
- `diagnostics/synthetic_stereo_example/diagnostic_summary.json` â€” 981 text lines
- `diagnostics/synthetic_stereo_example/manifest.json` â€” 483 text lines
- `diagnostics/synthetic_stereo_example/synthetic_stereo_trace.ndjson` â€” 52 text lines
- `docs/AGENT_REPO_REVIEW_GUIDE.md` â€” 179 text lines
- `docs/BUILD_ENVIRONMENT.md` â€” 57 text lines
- `docs/IMPLEMENTATION_NOTES.md` â€” 47 text lines
- `docs/MONOCULAR_REAL_FOOTAGE_VALIDATION.md` â€” 93 text lines
- `docs/RUNTIME_BRINGUP.md` â€” 393 text lines
- `docs/SYNTHETIC_STEREO_DIAGNOSTICS.md` â€” 128 text lines
- `models/PUT_MODEL_HERE.txt` â€” 13 text lines
- `models/README.md` â€” 51 text lines
- `models/rtmpose-x-halpe26-384x288.deploy.json` â€” 16 text lines
- `models/rtmpose-x-halpe26-384x288.detail.json` â€” 48 text lines
- `models/rtmpose-x-halpe26-384x288.onnx.sha256` â€” 1 text lines
- `models/rtmpose-x-halpe26-384x288.pipeline.json` â€” 123 text lines
- `src/calibration/calibration_io.cpp` â€” 384 text lines
- `src/calibration/calibration_io.h` â€” 16 text lines
- `src/calibration/calibration_types.h` â€” 93 text lines
- `src/calibration/floor_calibrator.cpp` â€” 62 text lines
- `src/calibration/floor_calibrator.h` â€” 12 text lines
- `src/calibration/intrinsic_calibrator.cpp` â€” 112 text lines
- `src/calibration/intrinsic_calibrator.h` â€” 16 text lines
- `src/calibration/stereo_calibrator.cpp` â€” 153 text lines
- `src/calibration/stereo_calibrator.h` â€” 24 text lines
- `src/capture/camera_device.cpp` â€” 181 text lines
- `src/capture/camera_device.h` â€” 39 text lines
- `src/capture/capture_health.cpp` â€” 4 text lines
- `src/capture/capture_health.h` â€” 28 text lines
- `src/capture/frame.h` â€” 18 text lines
- `src/capture/frame_pairer.cpp` â€” 74 text lines
- `src/capture/frame_pairer.h` â€” 54 text lines
- `src/capture/frame_slot.h` â€” 35 text lines
- `src/core/config.cpp` â€” 585 text lines
- `src/core/config.h` â€” 151 text lines
- `src/core/logging.cpp` â€” 46 text lines
- `src/core/logging.h` â€” 32 text lines
- `src/core/math.h` â€” 271 text lines
- `src/core/status.h` â€” 80 text lines
- `src/core/timing.cpp` â€” 16 text lines
- `src/core/timing.h` â€” 14 text lines
- `src/core/types.h` â€” 277 text lines
- `src/debug/debug_snapshot.h` â€” 80 text lines
- `src/debug/overlay_draw.cpp` â€” 37 text lines
- `src/debug/overlay_draw.h` â€” 14 text lines
- `src/debug/replay_log.cpp` â€” 594 text lines
- `src/debug/replay_log.h` â€” 24 text lines
- `src/debug/replay_player.cpp` â€” 71 text lines
- `src/debug/replay_player.h` â€” 16 text lines
- `src/debug/world_debug.cpp` â€” 18 text lines
- `src/debug/world_debug.h` â€” 11 text lines
- `src/inference/halpe26_contract.h` â€” 117 text lines
- `src/inference/rtmpose_decode.cpp` â€” 423 text lines
- `src/inference/rtmpose_decode.h` â€” 80 text lines
- `src/inference/rtmpose_model_contract.cpp` â€” 157 text lines
- `src/inference/rtmpose_model_contract.h` â€” 17 text lines
- `src/inference/rtmpose_session.cpp` â€” 309 text lines
- `src/inference/rtmpose_session.h` â€” 69 text lines
- `src/io/hmd_provider.cpp` â€” 145 text lines
- `src/io/hmd_provider.h` â€” 34 text lines
- `src/io/osc_sender.cpp` â€” 302 text lines
- `src/io/osc_sender.h` â€” 45 text lines
- `src/main.cpp` â€” 3439 text lines
- `src/tracking/body_model.cpp` â€” 219 text lines
- `src/tracking/body_model.h` â€” 49 text lines
- `src/tracking/body_solver.cpp` â€” 940 text lines
- `src/tracking/body_solver.h` â€” 98 text lines
- `src/tracking/contact_constraints.cpp` â€” 114 text lines
- `src/tracking/contact_constraints.h` â€” 20 text lines
- `src/tracking/epipolar.cpp` â€” 19 text lines
- `src/tracking/epipolar.h` â€” 15 text lines
- `src/tracking/foot_frame.cpp` â€” 89 text lines
- `src/tracking/foot_frame.h` â€” 34 text lines
- `src/tracking/foot_support.cpp` â€” 192 text lines
- `src/tracking/foot_support.h` â€” 34 text lines
- `src/tracking/identity_assignment.cpp` â€” 16 text lines
- `src/tracking/identity_assignment.h` â€” 18 text lines
- `src/tracking/joint_limits.cpp` â€” 25 text lines
- `src/tracking/joint_limits.h` â€” 16 text lines
- `src/tracking/monocular_projection.cpp` â€” 531 text lines
- `src/tracking/monocular_projection.h` â€” 63 text lines
- `src/tracking/motion_consistency_filter.cpp` â€” 549 text lines
- `src/tracking/motion_consistency_filter.h` â€” 114 text lines
- `src/tracking/posture_mode.cpp` â€” 126 text lines
- `src/tracking/posture_mode.h` â€” 28 text lines
- `src/tracking/reliability.cpp` â€” 132 text lines
- `src/tracking/reliability.h` â€” 54 text lines
- `src/tracking/roi_tracker.cpp` â€” 199 text lines
- `src/tracking/roi_tracker.h` â€” 65 text lines
- `src/tracking/root_support.cpp` â€” 79 text lines
- `src/tracking/root_support.h` â€” 20 text lines
- `src/tracking/support_queries.cpp` â€” 32 text lines
- `src/tracking/support_queries.h` â€” 12 text lines
- `src/tracking/temporal_update.cpp` â€” 186 text lines
- `src/tracking/temporal_update.h` â€” 21 text lines
- `src/tracking/tracker_ekf.cpp` â€” 335 text lines
- `src/tracking/tracker_ekf.h` â€” 67 text lines
- `src/tracking/tracker_synthesis.cpp` â€” 37 text lines
- `src/tracking/tracker_synthesis.h` â€” 26 text lines
- `src/tracking/tracking_pipeline.cpp` â€” 657 text lines
- `src/tracking/tracking_pipeline.h` â€” 110 text lines
- `src/tracking/triangulation.cpp` â€” 137 text lines
- `src/tracking/triangulation.h` â€” 20 text lines
- `src/ui/app/app.js` â€” 852 text lines
- `src/ui/app/index.html` â€” 202 text lines
- `src/ui/app/styles.css` â€” 619 text lines
- `src/ui/desktop_ui.cpp` â€” 360 text lines
- `src/ui/desktop_ui.h` â€” 22 text lines
- `src/web/runtime_state.h` â€” 71 text lines
- `tests/airborne_pipeline_regression_test.cpp` â€” 124 text lines
- `tests/body_model_seed_test.cpp` â€” 64 text lines
- `tests/config_test.cpp` â€” 211 text lines
- `tests/contact_constraint_cleanup_test.cpp` â€” 151 text lines
- `tests/floor_plane_test.cpp` â€” 31 text lines
- `tests/foot_support_test.cpp` â€” 261 text lines
- `tests/frame_pairer_test.cpp` â€” 82 text lines
- `tests/griddy_motion_regression_test.cpp` â€” 179 text lines
- `tests/hmd_provider_test.cpp` â€” 92 text lines
- `tests/live_bringup_readiness_test.py` â€” 160 text lines
- `tests/live_preflight_doctor_test.py` â€” 335 text lines
- `tests/model_asset_test.cpp` â€” 66 text lines
- `tests/monocular_real_footage_validation_test.py` â€” 206 text lines
- `tests/monocular_tracking_test.cpp` â€” 185 text lines
- `tests/motion_consistency_filter_test.cpp` â€” 192 text lines
- `tests/osc_packet_test.cpp` â€” 85 text lines
- `tests/replay_log_export_test.cpp` â€” 115 text lines
- `tests/root_support_test.cpp` â€” 82 text lines
- `tests/rtmpose_contract_test.cpp` â€” 221 text lines
- `tests/runtime_control_wiring_test.py` â€” 121 text lines
- `tests/synthetic_stereo_diagnostic_test.py` â€” 175 text lines
- `tests/synthetic_stereo_scoring_test.py` â€” 133 text lines
- `tests/temporal_update_test.cpp` â€” 171 text lines
- `tests/test_check.h` â€” 33 text lines
- `tests/tracker_ekf_test.cpp` â€” 183 text lines
- `tests/tracker_synthesis_confidence_test.cpp` â€” 41 text lines
- `tests/tracker_synthesis_test.cpp` â€” 35 text lines
- `tests/tracking_pipeline_fallback_test.cpp` â€” 56 text lines
- `tests/triangulation_confidence_test.cpp` â€” 16 text lines
- `tools/git-auto-sync.ps1` â€” 72 text lines
- `tools/inspect_onnx_contract.py` â€” 169 text lines
- `tools/install-auto-sync-task.ps1` â€” 32 text lines
- `tools/live_preflight_doctor.py` â€” 1077 text lines
- `tools/synthetic_stereo_diagnostic.py` â€” 1834 text lines
- `tools/validate_monocular_footage.py` â€” 418 text lines
- `vcpkg.json` â€” 15 text lines
```

---

## Source: docs\BUREAUCRAT_LOGIC.md

SHA-256: f6589cbb598f841cdbdcf28a292897fce8a0af504aa69072326592ce0928c41d

```text
# Bureaucrat Logic

Bureaucrat Logic is the failure mode where code accepts or computes useful data, then prevents that data from affecting runtime because a status flag, source label, readiness check, stale marker, warning, or post-check got treated as a veto.

The rule in this repo is simple: if data is finite and useful, keep it alive. Carry lower confidence, narrower capability, age, source, or warning metadata. Do not turn imperfect evidence into silence.

## What to preserve

- Predicted and fallback trackers should still reach OSC when finite, with their confidence intact.
- Anchor-held foot evidence should stand on local support confidence, not die because global body confidence dropped.
- Manual plank width and projective geometry should remain runtime evidence, not dead UI metadata.
- SteamVR/controller alignment transforms should remain usable after provider freshness lapses; the live provider is not required to bless an already-solved transform forever.
- Partial calibration/draft geometry should not overwrite the active last-known-good calibration unless explicitly promoted.
- Warnings from logs, cleanup, replay, network, frame pairing, or diagnostics should not kill tracking.

## What to reject

Reject non-finite numbers, impossible transforms, corrupt files that cannot be parsed at all, and explicit user-cleared state. Everything else should become scoped capability plus confidence, not a hard stop.

## Patch checklist

Before adding any `valid`, `ready`, `accepted`, `stale`, `source`, `reason`, or `status` check, verify that it does not block unrelated runtime data. A failed optional sample should weaken that sample. It should not wipe the solve. A failed output sink should disable that sink. It should not stop inference/tracking. A stale-but-finite frame should enter the solver with lower confidence. Prediction/hold is for missing pixels, not a bureaucratic eject button for older pixels.
```

---

## Source: docs\BUREAUCRAT_LOGIC_CRIME_SCENE.md

SHA-256: fcc9951a145b9084179a7eec13f304e7a1347e670ae750be961792776c24ccfe

```text
# Bureaucrat Logic Crime Scene Notes

This file exists because the previous failure mode was not subtle. It was everywhere. The repo repeatedly accepted evidence, computed fallbacks, solved transforms, stored calibrations, exposed UI controls, or prepared output â€” and then another layer quietly killed the result because the paperwork was not spiritually pure.

That pattern wasted time. It made working features look broken. It made manual calibration look useless. It made fallback logic decorative. It made output depend on unrelated ceremony. It made the codebase feel like a haunted DMV.

Do not make anyone rediscover this shit.

## The structural mistake

The project kept confusing **metadata about quality** with **permission to exist**.

That one sentence is the whole autopsy.

A weak result is not no result.

A stale provider is not an invalid transform.

An unknown source label is not bad math.

A partial manual calibration is not worthless.

A failed logger is not failed tracking.

A bad optional homography is not a poisoned calibration bundle.

A draft capture is not allowed to overwrite active state unless it is explicitly promoted.

A warning is not a death certificate.

The code kept writing little death certificates anyway. Very official. Very tidy. Completely fucking wrong.

## The crimes already found

### 1. OSC rejected fallback tracker sources

Predicted and HMD-carried tracker poses were computed and confidence-capped. Then OSC blocked them because of their source category.

This is like cooking dinner, plating it, and then throwing it out because the recipe card had the word "leftovers" on it.

Correct behavior: if the pose is finite and above the configured confidence floor, send it. Let confidence and telemetry describe weakness. Do not category-ban output.

### 2. Anchor-held feet got capped by unrelated global confidence

Foot support evidence is local. If the foot is planted and support confidence exists, it should be able to carry that foot. Whole-body confidence does not automatically get to murder the foot.

Correct behavior: use local support confidence for anchor-held foot evidence. Use body confidence for body-derived guesses. Do not let the entire body vote on whether a foot touching the floor is allowed to count.

### 3. Manual plank metric evidence was accepted and then neutered

The user entered real plank dimensions. The backend stored spacing. The UI displayed spacing. Then scalar metric use was disabled because the evidence was manual plank evidence.

This is the software equivalent of asking someone to measure a board, writing the measurement down, and then refusing to use it because it was measured by a human with suspiciously human hands.

Correct behavior: manual metric evidence is runtime evidence. It may have limited capability. It is not fake. It is not decorative. It does not need to disguise itself as auto-calibration to be useful.

### 4. Partial/draft geometry could overwrite working geometry

A newly captured partial floor geometry could replace previously working runtime geometry.

That is backwards. Drafts do not get to evict active state just because they walked into the room carrying a clipboard.

Correct behavior: store draft separately. Promote only when runtime-usable or explicitly requested. Failed recaptures and partial drawings do not delete working calibration.

### 5. Stale SteamVR provider status invalidated solved transforms

A tracker-space transform is a calibration result. The controller/provider helped solve it. The provider does not need to remain alive forever as a spiritual battery pack for the numbers.

Correct behavior: provider freshness affects new sampling and status. It does not erase finite transform data.

### 6. Unknown source strings vetoed finite transforms

The transform numbers could be valid, but an unfamiliar `tracker_space_source` string could mark the transform invalid.

A string label is metadata. It is not a tiny king. It does not outrank matrices, quaternions, or finite coordinates.

Correct behavior: keep the transform if the numbers are usable. Mark the source label as unknown. Do not destroy math because a string failed the vibe check.

### 7. Clearing SteamVR alignment could disable OSC

Clearing one alignment source could erase transform state and disable OSC.

That is not clearing metadata. That is pulling the fire alarm because someone moved a sticky note.

Correct behavior: clearing SteamVR session data, clearing active tracker-space transform, and disabling OSC are separate actions. Do not bundle them into one nuclear button.

### 8. Startup depended on output/debug plumbing

OSC open failure could stop runtime. Replay log open failure could stop runtime. Backup cleanup failure could turn a successful save into `ok=false`.

The logger is not the limbic system. Replay is not the heart. Backup cleanup is not the config transaction. Stop giving side chores veto power over the actual machine.

Correct behavior: degrade that component, report the issue, continue the core runtime when possible.

### 9. Camera and frame health became execution permission

Camera startup required immediate `opened && running`. Read failures mutated into state that higher layers could treat as dead. Pair skew returned no usable pair instead of degraded input.

The capture thread reconnecting is not the same as the subsystem being gone. A stale frame is not proof that prediction should be skipped. A skewed stereo pair is not a portal to nothingness.

Correct behavior: keep the pipeline ticking with prediction, monocular fallback, latest usable input, or degraded pair confidence.

### 10. Model contract vetoed decoder capability

The decoder had an XYC path. The model contract rejected XYC before the decoder could use it.

That is building support and then stationing a bored guard in front of it whose entire job is to say no.

Correct behavior: contract must match actual decoder capability. If the decoder supports a valid tensor shape, the contract accepts it. If not, delete the decoder path. Do not leave functional code behind a permanent veto.

### 11. Calibration load poisoned whole bundles over optional fields

Malformed optional homography could reject broader calibration load.

One rotten grape does not invalidate the entire vineyard. Invalidate the homography. Keep scalar spacing, body data, camera data, floor plane, and everything else that still works.

Correct behavior: scoped invalidation. No guilt-by-association nukes.

### 12. UI state treated backend perfection as interaction permission

The UI could refuse to preserve or expose partial floor geometry because global `valid` was false. It could require manual plank refresh before runtime start. It could block saving unrelated config changes because runtime was active.

This is how you get an interface that looks busy but acts like a doorman for a club nobody asked to enter.

Correct behavior: let the user inspect, save, and run from last committed state. Draft promotion can fail without blocking runtime. Save can persist settings even when hot-apply requires restart.

## What not to do in future patches

Do not add a new flag that secretly means "nothing below this line is allowed to exist."

Do not turn warnings into execution blockers.

Do not make optional cross-checks mandatory.

Do not make stale provider state erase solved calibration.

Do not let draft state overwrite active state.

Do not reject entire bundles because one scoped field failed.

Do not accept UI/config input and then silently clamp it somewhere else.

Do not create a fallback path and then ban fallback output.

Do not build another tiny bureaucracy inside the runtime. The last one has already been dragged outside and identified by dental records.
```

---

## Source: docs\CODEX_PATCH_REVIEW_NO_BUREAUCRAT_LOGIC.md

SHA-256: 343ae172f3bcc1cb495dde6c2dadc51ef288bf4d640e5883d6156de1a157d018

```text
# Codex Patch Review: No Bureaucrat Logic

Use this before committing any patch in this repo.

This is not a style note. This is here because the repo already had the same self-sabotaging pattern in tracking, calibration, UI, alignment, config, replay, OSC, camera startup, and model loading. The pattern was always the same: compute something useful, then let a separate status flag, readiness summary, source string, warning, or "responsible" check kill it.

Do not do that again. Seriously. Do not spend compute producing a result and then let a fucking status label delete it.

## The one-question review

For every new condition that blocks, clears, invalidates, disables, refuses, rejects, resets, overwrites, or returns early, ask:

```text
Is this actual impossibility, or is this just the code being uncomfortable?
```

Actual impossibility can stop the narrow operation that cannot proceed.

Code discomfort gets confidence, telemetry, status, capability loss, or draft state.

That is the line. Stop blurring it. Every time the code blurs it, someone has to come back later with a flashlight and a shovel.

## Kill-switch smell list

This is where the mess sneaks back in. It will not announce itself as sabotage. It will look like tidy defensive programming. It will have a respectable name. It will say `valid`, `ready`, `stale`, `accepted`, `source_known`, `backend_owned`, `healthy`, or `strict`. Then the feature will stop fucking working.

Be suspicious of patches that add or expand checks like these:

```cpp
if (!thing.valid) return;
if (!everything_ready) return Error(...);
if (stale) transform_valid = false;
if (!source_known) output_enabled = false;
if (warning_present) runtime_enabled = false;
if (!backend_owned) refuse_user_state();
if (optional_crosscheck_failed) reject_main_result();
if (draft_exists) overwrite_active();
if (logger_failed) stop_runtime();
if (replay_failed) stop_tracking();
if (cleanup_failed_after_commit) return ok_false();
if (one_field_bad) reject_entire_bundle();
```

Some of these may be legitimate in very narrow places. In this repo, most of them were bureaucratic bullshit wearing a hard hat.

## Required replacement pattern

Prefer this shape:

```cpp
Capabilities caps = DeriveCapabilities(input, config, diagnostics);
Result result = RunWithCapabilities(input, caps);
result.confidence = ComputeConfidence(caps, diagnostics);
result.status = DescribeLimitations(caps, diagnostics);
return result;
```

Not this:

```cpp
if (!Perfect(input, config, diagnostics)) {
    return Nothing();
}
```

This project is allowed to run degraded. It is allowed to output imperfect data with honest confidence. It is allowed to preserve last-known-good. It is allowed to use manual evidence. It is allowed to keep finite transforms after a provider goes stale. It is allowed to send fallback tracker poses. It is allowed to start runtime when replay logging is broken. It is not required to curl into a ball because a status string coughed.

## Preserve last-known-good like it matters

If there is active state and draft state, protect active state.

Bad:

```cpp
active = draft;
```

when draft is partial, rejected, stale, incomplete, or only useful for review.

Correct:

```cpp
if (draft.runtime_usable) {
    active = draft;
} else {
    last_draft = draft;
    active = active; // yes, really, do not throw the working thing away
}
```

Failed recaptures, partial drawings, stale frames, rejected optional samples, and incomplete calibrations do not delete working state. Drafts do not get eviction rights.

## Source labels are not monarchs

Do not let labels outrank numbers.

Bad:

```cpp
if (!source_known) {
    transform_valid = false;
}
```

Correct:

```cpp
if (TransformNumbersUsable(transform)) {
    transform_valid = true;
    source_status = source_known ? "known" : "unknown_label";
}
```

If the quaternion is finite, the scale is sane, and the offsets are usable, the transform exists. The source string can sit in the corner and be weird.

## Stale is not dead

Stale means old, not nonexistent.

Bad:

```cpp
if (provider_stale) {
    clear_transform();
    disable_output();
}
```

Correct:

```cpp
status.provider_fresh = false;
status.transform_age_ms = age;
confidence *= FreshnessScale(age);
keep_transform_if_numbers_are_usable();
```

The transform does not evaporate because the controller stopped sending fresh vibes.

## Optional means optional

Forward yaw, floor samples, homography, replay writer, OSC socket, frame pairing, full calibration readiness, backend ownership, and debug cleanup are not automatically allowed to veto core runtime.

If they are not essential to the narrow operation, they do not block the narrow operation.

Optional data can improve confidence. Optional data can add capability. Optional data can warn. Optional data cannot walk into the room wearing a fake badge and shut everything down.

## UI/config must not lie

If the UI exposes a setting, runtime must honor it within the documented range.

Bad:

```cpp
// UI/validation allows wider range
runtime_value = Clamp(user_value, secret_min, secret_max);
```

Correct: either honor the accepted value or change the UI/validation. Do not accept one thing and execute another. That is how software becomes a gaslighting toaster.

## Output/debug plumbing is not the runtime

OSC failure can disable OSC. Replay failure can disable replay. Backup cleanup failure can produce a warning. None of those automatically means tracking cannot run, config did not save, or the whole runtime should refuse to start.

Keep side channels in their lane. A broken clipboard does not mean the patient is dead.

## Final patch review checklist

Before merging, verify the patch did not introduce:

```text
computed fallback -> downstream category ban
manual/user evidence -> treated as fake
stale provider -> invalid numeric transform
unknown label -> invalid data
warning/log string -> hard failure
optional field failure -> whole object rejection
draft/partial state -> active state overwrite
readiness summary -> execution permission
UI accepted value -> runtime hidden override
output/debug failure -> runtime boot failure
```

If it did, fix it. Do not write an essay explaining why this version of the same desk-clerk bug is actually sophisticated. It is not. It is the same corpse in a different tie.
```

---

## Source: docs\AGENT_REPO_REVIEW_GUIDE.md

SHA-256: 60ef57c1e40c9330c6d622f71acfbfff080eca17d7ee15027893fb893ca97647

```text
# Agent Repo Review Guide

This file is the short, strict operating guide for future agents working on this repository. The long version is embedded in `LLM_REVIEW_AND_FEATURE_GUIDE.md` so the repo stays self-contained even if this file is missed.

## Core rule

Treat the repository like a system, not a pile of strings.

Start with the user-facing claim, then prove the full path. For this repo, examples of claims are: â€œplanks can be used for floor scale,â€ â€œsingle-camera fallback works in stereo,â€ â€œmanual override exists,â€ â€œauto tracking exists,â€ â€œOSC tracker-space gating exists,â€ or â€œthe UI setting changes runtime behavior.â€ A claim is not proven until it is mapped through config defaults, config structs, parsing/validation, UI controls, UI save/load payloads, runtime logic, telemetry/debug output, and docs. Tests can guard the path afterward; they do not prove the architecture exists.

Do not edit first. Read first. If a file output is truncated, that inspection failed. Rerun the view in smaller slices before drawing a conclusion.


## SteamVR/manual tracker-space invariant

Do not collapse controller alignment and manual fallback into one slot. `osc.tracker_space_*` is the active transform used by OSC. `osc.manual_tracker_space_*` is the preserved fallback. A controller solve may replace the active transform only after preserving any valid manual/json transform. Clearing, missing, failed, or stale controller alignment must restore or use the manual fallback when valid; without a valid fallback, OSC must block. Prove this by reading config load/save, UI save/load, SteamVR solve/clear, runtime OSC config selection, OSC send gating, and status/debug export before treating tests as meaningful.

## Floor-geometry source-of-truth invariant

The UI may request backend floor-geometry detection and display evidence, but the solver consumes that evidence only when `tracking.monocular.floor_geometry_calibration_enabled=true`. Clearing floor geometry must disable the config flag, remove the persisted generated geometry, and avoid leaving a copied generated `floor_plane` behind as ghost evidence. Do not claim floor/calibration changes hot-apply while runtime is running; only OSC/tracker-space saves are hot.

## Correct audit order

Use this order unless the task is explicitly documentation-only.

1. Map the repo and identify the runtime path. Do not infer architecture from filenames alone.
2. Read config defaults and config schema.
3. Read type definitions for the affected values.
4. Read config parsing, validation, aliases, and save/default writing.
5. Read UI controls and labels.
6. Read UI load/save payload mapping.
7. Read runtime loop and mode branches.
8. Read the tracking/solver function that is supposed to consume the value.
9. Read telemetry, replay status, desktop UI state, and debug export.
10. Read tests only to learn what they miss; assume they catch shallow regressions unless the runtime contract proves otherwise.
11. Patch the smallest necessary line ranges.
12. Review the diff against the user-facing claim and runtime contract before any build/test step.
13. Run targeted tests only when the user asked for validation or after the architecture has already been proven by inspection.

## Grep is not proof

Grep only tells you where to inspect. Seeing `floor_depth_line_spacing_m` proves there is some floor-spacing field. It does not prove plank tracking exists. Seeing `stereo_monocular_fallback_enabled` proves there is a fallback flag. It does not prove fallback works for Camera B start failure, frame pairing failure, calibration failure, or Camera B pose decode failure.

The proof lives in the actual data flow and branch behavior.

## Large-file inspection policy

Read large files in slices. Do not let terminal truncation decide what you know.

Suggested first pass:

```bash
nl -ba src/main.cpp | sed -n '1,220p'
nl -ba src/main.cpp | sed -n '220,520p'
nl -ba src/main.cpp | sed -n '1800,2460p'
nl -ba src/main.cpp | sed -n '2460,3344p'

nl -ba src/ui/app/app.js | sed -n '1,220p'
nl -ba src/ui/app/app.js | sed -n '220,460p'
nl -ba src/ui/app/app.js | sed -n '460,700p'
nl -ba src/ui/app/app.js | sed -n '700,860p'


nl -ba src/core/config.cpp | sed -n '1,220p'
nl -ba src/core/config.cpp | sed -n '220,460p'
nl -ba src/core/config.cpp | sed -n '460,620p'

nl -ba src/tracking/monocular_projection.cpp | sed -n '1,220p'
nl -ba src/tracking/monocular_projection.cpp | sed -n '220,520p'
```

After the slice pass, inspect exact functions around the change. Do not continue after malformed or cut-off snippets.

## Floor/plank feature truth table

The current implementation supports manual floor-scale assist plus setup-time Camera A preview seam helpers.

It has real persisted inputs for physical spacing in meters, image spacing in pixels, optional reference Y pixel, optional reference depth, and confidence. That supports repeated floor seams, tile rows, plank board pitch, or rug/floor-depth lines as a known local scale hint.

Runtime depth treats a marked reference seam plus neighboring seam as a projective floor-depth observation, not as a constant `meters_per_pixel` ruler. This matters for real planks/tiles because repeated floor lines compress with distance. If no usable projective reference can be derived, the code falls back to the camera-height floor ray.

It does not implement full semantic plank detection. It does not distinguish plank width from plank length. It does not know board orientation. It does not model arbitrary diagonal line families. It does not continuously track seams in the C++ runtime. If those are desired, they must be added as explicit features.

The safe wording is â€œknown repeated floor-depth pitch,â€ not â€œautomatic plank tracking.â€

## Floor/plank claim checklist

Before touching floor/plank code, answer these in the repo:

- Can the user input tile/plank spacing manually?
- Is the spacing saved to config?
- Is it loaded back into the UI and runtime?
- Does the UI expose what the value means without implying full plank geometry?
- Does runtime actually use it in monocular depth estimation?
- Does telemetry say whether it is active, inactive, disabled, standby, weak, invalid, or ignored?
- Can manual values override auto-detected or click-marked values before save?
- Is there actual automatic seam/plank detection, or only setup-time preview seam detection?
- Does the system distinguish plank width from plank length?
- Does it know seam orientation/perspective, or just a scalar image-depth spacing?
- Does stereo fallback reuse the monocular floor assist path?
- Does normal calibrated stereo avoid pretending floor-spacing assist is the triangulated source?
- Does confidence drop when depth is inferred instead of triangulated?
- Do tests cover the fallback state and depth-source semantics, not just config strings?

## Stereo fallback claim checklist

Before saying â€œsingle-camera features work in stereo,â€ prove the fallback path:

- Does fallback activate when Camera B fails to start?
- Does fallback activate when Camera B is unavailable?
- Does fallback activate when frame pairing fails but Camera A is available?
- Does fallback activate when stereo calibration is missing, unloadable, or not ready?
- Does fallback activate when Camera A pose succeeds and Camera B pose decode fails?
- Does the code temporarily set `TrackingMode::Monocular` for the fallback frame?
- Does it restore stereo params afterward?
- Does debug output label the degraded mode with `stereo_monocular_fallback:<reason>`?
- Does the solver report inferred monocular depth instead of triangulated stereo depth?
- Does confidence stay lower/degraded for inferred depth?
- Do docs distinguish normal stereo from fallback stereo?

## Common agent failures to avoid

Do not assume the requested output is â€œjust codeâ€ before understanding the task. This repo often needs audit, validation, or handoff quality, not random patching.

Do not treat broad search counts as evidence. A file having hundreds of camera/floor/stereo hits proves nothing by itself.

Do not let malformed output slide. If a snippet cuts off mid-branch, rerun it.

Do not overfit to examples. â€œPlanks,â€ â€œtiles,â€ and â€œfallbackâ€ may be probes for the larger question: â€œis the implementation real end-to-end?â€

Do not treat missing literal names as proof. A feature may be named differently. Inspect UI labels, config keys, runtime update handlers, and solver functions.

Do not edit docs/tests before proving runtime behavior unless the task is explicitly documentation-only.

Do not patch wording around a feature before deciding whether it is real, fake, partial, mislabeled, or untested.

Do not use Python/static tests as the main proof of native runtime behavior. Static tests guard wiring and claims. They do not prove source-of-truth ownership, UI/backend semantics, stale-state blocking, persistence, hot runtime config propagation, C++ linking, live cameras, ONNX inference, frame pairing, solver stability, VR alignment, or real tracking quality.

Do not claim a full build passed unless CMake configure/build/CTest actually ran in an environment with the required native dependencies.

Do not call a source-only package broken only because the large ONNX file is missing. Check whether `models/PUT_MODEL_HERE.txt` is present. The source-only package should skip ONNX metadata inspection until `models/rtmpose-x-halpe26-384x288.onnx` exists; strict asset validation uses `BODYTRACKER_REQUIRE_ONNX_ASSET=1`. Runtime inference still requires the real model.

Do not inspect UI shallowly. UI save/load bugs often live far away from the visible control.

Do not update root/body support before current-frame foot support. Root support depends on whether the current feet are actually supported; stale previous-frame feet create fake anchoring during starts, stops, occlusion, crouches, and replayed trackers.

Do not let low motion create rest support by itself. A foot that is merely still, high above the floor, or occluded is not automatically resting on a couch/bed/chair. Rest support requires usable contact evidence and a seated/reclined posture.

Do not report kneeling root support unless knee anchors are actually active. A kneeling posture classification is not the same as measured knee contact; without knee anchors, root support must fall back to active foot support or none.

Do not treat plank/tile spacing as a constant meters-per-pixel ruler. Floor depth is perspective/projective: repeated seams compress with distance. Use a reference seam plus neighboring seam, then derive depth through `(y - cy) âˆ 1/depth` or fall back to the camera-height floor ray.

Do not reuse a variable named `root` for filesystem walking or patch helpers in a way that can mutate the repo path.

Do not zip final output before reviewing the diff.

## Edit policy

Never rewrite a large file from scratch. Patch the smallest stable range.

Never patch based on vibes.

Never claim â€œimplementedâ€ unless the runtime path consumes the value.

Never claim â€œauto-detectâ€ unless there is actual image-processing or tracking logic for the thing being claimed.

Never claim â€œstereo supports monocular featuresâ€ unless stereo fallback explicitly passes through the monocular path and exposes degraded-mode telemetry.

Never remove compatibility aliases unless you also migrate every caller, test, doc, and config path.

Never bypass tracker-space validation for OSC.

## Required handoff standard

Every serious patch should leave behind:

- what user-facing claim changed;
- which files changed;
- what runtime branch changed;
- what telemetry now says;
- what architectural contract was inspected;
- what validation was intentionally not run;
- a reviewed diff;
- a zip only after the diff has been reviewed.

Do not center the handoff around tests unless the user explicitly asked for a test pass.
```

