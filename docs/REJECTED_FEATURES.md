# Rejected Features

This document records feature ideas that were deliberately removed from the active plan because they add ceremony, fake scope, or platform cosplay instead of improving the tracker.

The rule is simple: if a feature does not improve the actual body-tracking runtime, calibration evidence, replay/debug proof, OSC output, or maintainability of those paths, it does not belong in the roadmap.

## Rejected

### Multi-user tracking

Rejected because this project outputs a single-user SteamVR/VRChat tracker stream. Trying to track multiple bodies would create fake product scope without a real runtime target. The exported tracker stream is still for one user. Default roles are pelvis, left foot, right foot, chest, left elbow, and right elbow; knee trackers remain optional and disabled by default.

Do not confuse this with left/right identity repair. Left/right disambiguation is still valid and necessary because it keeps one user's limbs assigned correctly.

### One-click auto-calibration wizard

Rejected because it turns calibration into UI theater. The useful work is explicit calibration, visible evidence, diagnostics, replay logs, and clear status reporting. A wizard that implies the hard math has been magically solved is not useful.

### Generic calibration-wizard roadmap

Rejected for the same reason. Guided UI is only useful when it exposes real backend evidence and clear next actions. It is not useful as a broad roadmap item that hides missing calibration logic behind friendly screens.

### Adaptive quality scaling

Rejected because dynamically changing inference quality or resolution makes tracking behavior a moving target. That makes regressions harder to reproduce. The correct order is profiler, fixed budgets, evidence, and deliberate operator-controlled tuning, not automatic quality roulette.

### Plugin ecosystem

Rejected as premature platform cosplay. The core tracker must stay understandable and reliable before anyone pretends there is a stable third-party plugin surface.

### Third-party SDK

Rejected for the same reason. Packaging an SDK around internals that are still being actively shaped would freeze bad interfaces and multiply maintenance work.

### Multi-language support

Rejected because it does not improve tracking correctness, runtime stability, calibration, OSC output, replay/debugging, or code architecture. It is polish, not core work.

### Community/forum/monthly-release ecosystem goals

Rejected because those are not implementation items. They belong in a public product/community plan, not in a source-level engineering roadmap.

### User satisfaction and support-ticket KPIs

Rejected because they do not tell an implementer what code to write or what runtime behavior to prove. They are product-management wallpaper unless backed by concrete engineering work.

### Professional motion-capture platform ambition

Rejected as overbroad. The active project scope is camera-based VR body tracking with SteamVR/VRChat OSC output, not a universal mocap platform.

### UI architecture as future-roadmap text

Rejected in its old form because “refactor the UI someday” is useless. The useful version is the implemented architecture layer: state store, command bus, dependency injection, component shell, config-event history, and static wiring tests.

### Performance/QA as vague goals

Rejected in its old form because “80% coverage” and “performance dashboard” are empty unless they are wired to tools and runtime data. The useful version is concrete: profiler plumbing, performance budget files, budget gates, fuzz tests, API-doc generation, and CI/static checks.

### Decorative config schema

Rejected because schema-as-documentation is fake safety. The useful version is load-time validation before ad-hoc readers touch config values.

### Model/package tests that fail source-only zips

Rejected because they waste time proving that a huge ONNX payload is not present in a small source archive. Source sanity should validate source contracts. Strict asset validation belongs behind an explicit full-package/asset-required mode.

### Stale future-roadmap promises in docs

Rejected because dead roadmap text re-infects the repo with work nobody actually wants. If a feature is not wanted, delete it from the active roadmap instead of leaving it as a ghost assignment for the next agent.

## Still accepted

The following are not rejected and should not be deleted by accident:

- Single-user left/right identity repair.
- Manual and SteamVR tracker-space alignment.
- Replay recording/playback and deterministic debug artifacts.
- Runtime profiler and performance budget tooling.
- Load-time config schema validation plus C++ semantic validation.
- UI architecture primitives that are wired into the real WebView runtime.
- Source-sanity checks that prove contracts without requiring the full model payload.

