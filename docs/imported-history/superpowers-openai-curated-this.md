# Imported BodyTracker history: superpowers OpenAI curated source

Source repository: `markoapple/2026-04-23-superpowers-plugin-superpowers-openai-curated-this`

Generated at: `2026-06-11T00:28:14+00:00`

## Import policy

This repository intentionally imports the previous BodyTracker development history as metadata only.
The source repository's file tree, archives, binaries, and source blobs were not merged into the current `bodytracker-bt` tree.
Each imported source commit is represented by an empty commit whose message records the original source commit SHA, subject, author date, and import mode.

## Source refs

| Ref | Commit | Date | Subject |
| --- | --- | --- | --- |
| `fbt-final-latest` | `4dc5e0bbf2a4` | 2026-05-03T04:36:56+02:00 | Fix free foot motion prediction for consistency filter |
| `main` | `5c1134b5e441` | 2026-06-08T14:33:55Z | Replace repository contents with bt.zip |
| `pre-replace-backup` | `38af9781343d` | 2026-06-02T10:59:59Z | Add .gitignore to exclude build/vendor artifacts from archives |
| `pull/1/head` | `172f611ab8b1` | 2026-05-01T20:27:58+02:00 | Add runtime bring-up runbook |
| `pull/2/head` | `8737bf3e750b` | 2026-05-01T21:00:07+02:00 | Fail camera startup on health timeout |

## Source commit changelog

| # | Source commit | Date | Author | Subject |
| ---: | --- | --- | --- | --- |
| 1 | `0681757` | 2026-05-01T05:49:20+02:00 | Pands | Initial commit |
| 2 | `563202e` | 2026-05-01T20:22:55+02:00 | Pands | Fix OpenCV highgui build wiring |
| 3 | `aa3dfcd` | 2026-05-01T20:25:44+02:00 | Pands | Document vcpkg dependencies |
| 4 | `d5c1168` | 2026-05-01T20:26:09+02:00 | Pands | Add Windows CI build and test workflow |
| 5 | `aa2a011` | 2026-05-01T20:26:41+02:00 | Pands | Clarify build status and runtime readiness |
| 6 | `eb0f16c` | 2026-05-01T20:27:11+02:00 | Pands | Expand Windows build environment docs |
| 7 | `172f611` | 2026-05-01T20:27:58+02:00 | Pands | Add runtime bring-up runbook |
| 8 | `f37c5f9` | 2026-05-01T20:31:04+02:00 | Pands | Holistic review: add CI and tighten build/runtime readiness (#1) |
| 9 | `ea413c5` | 2026-05-01T20:33:36+02:00 | Pands | Reject duplicate stereo frame pairs |
| 10 | `291ad49` | 2026-05-01T20:35:47+02:00 | Pands | Reject duplicate stereo frame pairs |
| 11 | `e9cd7bf` | 2026-05-01T20:37:00+02:00 | Pands | Add frame pairer duplicate rejection tests |
| 12 | `864c709` | 2026-05-01T20:58:28+02:00 | Pands | Wire frame pairer tests into CMake |
| 13 | `8737bf3` | 2026-05-01T21:00:07+02:00 | Pands | Fail camera startup on health timeout |
| 14 | `a255886` | 2026-05-01T21:04:59+02:00 | Pands | Harden runtime frame pairing and camera startup (#2) |
| 15 | `18df8a7` | 2026-05-01T21:17:43+02:00 | Pands | Make JSON HMD provider strict |
| 16 | `74005d6` | 2026-05-01T21:18:30+02:00 | Pands | Add JSON HMD provider tests |
| 17 | `59697c9` | 2026-05-01T21:22:58+02:00 | Pands | Wire JSON HMD provider tests into CMake |
| 18 | `bf11186` | 2026-05-01T21:25:14+02:00 | Pands | Expose duplicate frame-pair telemetry in web UI |
| 19 | `061ff56` | 2026-05-01T21:26:05+02:00 | Pands | Record duplicate frame-pair telemetry in replay logs |
| 20 | `fbcb705` | 2026-05-01T21:27:20+02:00 | Pands | Remove fake static HMD config fields |
| 21 | `fb278d4` | 2026-05-01T21:29:30+02:00 | Pands | Remove static HMD provider |
| 22 | `dc0847b` | 2026-05-01T21:32:31+02:00 | Pands | Remove static HMD provider implementation |
| 23 | `c9dfa80` | 2026-05-01T21:34:41+02:00 | Pands | Remove static HMD mode from config |
| 24 | `ef2a3bd` | 2026-05-01T21:35:18+02:00 | Pands | Remove fake static HMD fields from default config |
| 25 | `4d68474` | 2026-05-01T21:36:47+02:00 | Pands | Restyle local web dashboard as control surface |
| 26 | `a62c4e1` | 2026-05-01T21:47:55+02:00 | Pands | Match SignalOnly dashboard font stack |
| 27 | `a210007` | 2026-05-01T22:17:44+02:00 | Pands | Add release-safe test checks |
| 28 | `9ec34c9` | 2026-05-01T22:23:40+02:00 | Pands | Make HMD provider tests release-safe |
| 29 | `2919187` | 2026-05-01T22:24:05+02:00 | Pands | Make frame pairer tests release-safe |
| 30 | `772804b` | 2026-05-01T22:26:08+02:00 | Pands | Make OSC and triangulation tests release-safe |
| 31 | `5f45232` | 2026-05-01T22:41:01+02:00 | Pands | Fix FrameSlot replacement counter race |
| 32 | `7355fdb` | 2026-05-01T22:42:05+02:00 | Pands | Serve dashboard font assets and fix web server running race |
| 33 | `13e1e52` | 2026-05-01T22:44:10+02:00 | Pands | Make Windows socket initialization thread-safe |
| 34 | `a7629ec` | 2026-05-01T22:48:07+02:00 | Pands | Make config parsing reject malformed structured values |
| 35 | `d780d2c` | 2026-05-01T22:57:02+02:00 | Pands | Remove forced OpenCV highgui include and link test deps |
| 36 | `d2edb54` | 2026-05-01T23:04:27+02:00 | Pands | Keep highgui build path stable and link frame pairer test deps |
| 37 | `0ba8eb9` | 2026-05-01T23:12:40+02:00 | Pands | Add explicit CMake preset configurations |
| 38 | `532b027` | 2026-05-01T23:14:00+02:00 | Pands | Harden Result against OK error status |
| 39 | `143901f` | 2026-05-01T23:18:04+02:00 | Pands | Validate triangulation inputs, projection, and chirality |
| 40 | `050ab7a` | 2026-05-01T23:20:29+02:00 | Pands | Cover triangulation rejection paths in tests |
| 41 | `043d564` | 2026-05-01T23:27:25+02:00 | Pands | Add predictive occlusion fallback for lower-body tracking |
| 42 | `a07edb8` | 2026-05-01T23:30:42+02:00 | Pands | Add foot-specific temporal correction gains |
| 43 | `6e253b1` | 2026-05-01T23:31:30+02:00 | Pands | Smooth foot and joint correction through temporal filter |
| 44 | `0a7b6bf` | 2026-05-01T23:33:39+02:00 | Pands | Use ankle pitch in lower-body foot keypoint model |
| 45 | `463c925` | 2026-05-01T23:37:30+02:00 | Pands | Add configurable motion consistency settings |
| 46 | `5facffd` | 2026-05-01T23:56:26+02:00 | Pands | Add motion consistency filter interface |
| 47 | `8022fc8` | 2026-05-02T00:10:25+02:00 | Pands | Implement motion direction consistency filter |
| 48 | `60d8792` | 2026-05-02T00:12:55+02:00 | Pands | Add tracking config and motion filter state to pipeline |
| 49 | `c182acd` | 2026-05-02T00:14:47+02:00 | Pands | Apply motion consistency filter in tracking pipeline |
| 50 | `f2ceac3` | 2026-05-02T00:16:02+02:00 | Pands | Load and validate motion consistency config |
| 51 | `fa051e4` | 2026-05-02T00:18:26+02:00 | Pands | Expose motion consistency defaults in config |
| 52 | `0947d41` | 2026-05-02T00:18:56+02:00 | Pands | Add motion consistency filter behavior tests |
| 53 | `a18b1aa` | 2026-05-02T00:19:25+02:00 | Pands | Build and test motion consistency filter |
| 54 | `958c23f` | 2026-05-02T00:22:07+02:00 | Pands | Add compatibility tracking pipeline API surface |
| 55 | `e64062e` | 2026-05-02T00:22:51+02:00 | Pands | Implement compatibility pipeline entrypoints and telemetry |
| 56 | `2c686c1` | 2026-05-02T00:23:26+02:00 | Pands | Track accumulated low-motion drift in filter state |
| 57 | `605db37` | 2026-05-02T00:24:06+02:00 | Pands | Prevent low-motion drift from being erased forever |
| 58 | `cda0b5e` | 2026-05-02T00:24:44+02:00 | Pands | Test accumulated low-motion drift handling |
| 59 | `98dcfc5` | 2026-05-02T00:26:43+02:00 | Pands | Include cstddef for motion target enum type |
| 60 | `de1389b` | 2026-05-02T00:27:20+02:00 | Pands | Initialize planted targets from support anchors |
| 61 | `40f0c58` | 2026-05-02T00:28:56+02:00 | Pands | Add Codex notes for main cpp and tracking filter follow-up |
| 62 | `d74535d` | 2026-05-02T00:30:42+02:00 | Pands | Add string names for root support telemetry |
| 63 | `39ec7c7` | 2026-05-02T00:31:15+02:00 | Pands | Log motion filter and solver telemetry in replay output |
| 64 | `fe58120` | 2026-05-02T00:31:39+02:00 | Pands | Restore runtime-compatible config aliases |
| 65 | `593dd80` | 2026-05-02T00:32:34+02:00 | Pands | Populate runtime-compatible config aliases |
| 66 | `4f469be` | 2026-05-02T00:33:07+02:00 | Pands | Keep default config aligned with runtime aliases |
| 67 | `47b331b` | 2026-05-02T00:33:47+02:00 | Pands | Declare replay append compatibility method |
| 68 | `b1d3b76` | 2026-05-02T00:34:26+02:00 | Pands | Implement replay append compatibility method |
| 69 | `391bafe` | 2026-05-02T00:34:53+02:00 | Pands | Add HMD ReadLatest compatibility wrapper |
| 70 | `efe75cd` | 2026-05-02T00:35:24+02:00 | Pands | Add debug snapshot compatibility fields |
| 71 | `5666fcb` | 2026-05-02T00:38:08+02:00 | Pands | Prefer live debug aliases when writing replay logs |
| 72 | `506df60` | 2026-05-02T00:38:59+02:00 | Pands | Keep replay log stream open |
| 73 | `26fcd52` | 2026-05-02T00:58:40+02:00 | Pands | Avoid reopening replay log for every frame |
| 74 | `edaf810` | 2026-05-02T01:00:09+02:00 | Pands | Precompute undistorted keypoints in body solver |
| 75 | `b3d8faf` | 2026-05-02T01:02:24+02:00 | Pands | Add OSC address and packet caches |
| 76 | `e875ac6` | 2026-05-02T01:03:01+02:00 | Pands | Cache OSC tracker addresses and packet buffer |
| 77 | `9bf49f4` | 2026-05-02T01:04:51+02:00 | Pands | Restrict body solver loops to used lower-body keypoints |
| 78 | `6804edb` | 2026-05-02T12:58:36+02:00 | Pands | Update Codex notes after optimization and compatibility passes |
| 79 | `9f888fd` | 2026-05-02T13:21:02+02:00 | Pands | Restore runtime state publish compatibility |
| 80 | `641106e` | 2026-05-02T13:24:21+02:00 | Pands | Use live debug aliases in web status output |
| 81 | `61e25b8` | 2026-05-02T13:27:40+02:00 | Pands | Harden replay writer alias fallback and reopen behavior |
| 82 | `5d5a637` | 2026-05-02T20:40:00+02:00 | Pands | Add desktop WebView app UI and auto sync |
| 83 | `f34ef8a` | 2026-05-02T20:40:30+02:00 | Pands | Auto-sync desktop changes 2026-05-02 20:40:30 |
| 84 | `c59c2d9` | 2026-05-02T23:40:00+02:00 | Pands | Reject one-sided stale frame reuse in pairer |
| 85 | `40fcaed` | 2026-05-02T23:45:50+02:00 | Pands | Reject one-sided stale frame reuse in pairer |
| 86 | `4ef90c1` | 2026-05-02T23:46:10+02:00 | Pands | Test one-sided stale frame rejection |
| 87 | `817aec9` | 2026-05-02T23:47:45+02:00 | Pands | Add floor plane geometry helpers |
| 88 | `c7ec44b` | 2026-05-02T23:53:50+02:00 | Pands | Test floor plane helpers |
| 89 | `dd66584` | 2026-05-02T23:54:23+02:00 | Pands | Add floor plane helper test target |
| 90 | `f5bbb04` | 2026-05-02T23:58:39+02:00 | Pands | Use calibrated floor plane for foot support |
| 91 | `cb770ba` | 2026-05-02T23:59:38+02:00 | Pands | Use calibrated floor plane for foot support |
| 92 | `c10189d` | 2026-05-03T00:01:04+02:00 | Pands | Pass calibrated floor plane into support classification |
| 93 | `710fd73` | 2026-05-03T00:02:18+02:00 | Pands | Project floor-supported feet onto calibrated plane |
| 94 | `82ab0a4` | 2026-05-03T00:03:01+02:00 | Pands | Test floor-plane foot support classification |
| 95 | `9244b3f` | 2026-05-03T00:05:16+02:00 | Pands | Add foot support test target |
| 96 | `8945e06` | 2026-05-03T00:09:15+02:00 | Pands | Weight seed ingestion by calibrated bone lengths |
| 97 | `30f98c1` | 2026-05-03T00:10:05+02:00 | Pands | Test calibrated bone length seed weighting |
| 98 | `84fdf46` | 2026-05-03T00:11:51+02:00 | Pands | Add body model seed weighting test target |
| 99 | `d349b35` | 2026-05-03T00:12:44+02:00 | Pands | Add tracker EKF filter |
| 100 | `61f2950` | 2026-05-03T00:13:53+02:00 | Pands | Add tracker EKF filter |
| 101 | `5320802` | 2026-05-03T00:15:40+02:00 | Pands | Wire tracker EKF into pipeline state |
| 102 | `f999b74` | 2026-05-03T00:16:42+02:00 | Pands | Apply tracker EKF in runtime pipeline |
| 103 | `3f51bad` | 2026-05-03T00:18:11+02:00 | Pands | Add tracker EKF config |
| 104 | `f1bbef8` | 2026-05-03T00:19:30+02:00 | Pands | Use shared tracker EKF config type |
| 105 | `39de461` | 2026-05-03T00:20:32+02:00 | Pands | Test tracker EKF smoothing |
| 106 | `624167c` | 2026-05-03T00:21:07+02:00 | Pands | Build tracker EKF module and tests |
| 107 | `e46496b` | 2026-05-03T00:22:14+02:00 | Pands | Expose tracker EKF defaults in config |
| 108 | `30ea090` | 2026-05-03T00:24:04+02:00 | Pands | Use role-specific tracker confidence |
| 109 | `86a665a` | 2026-05-03T00:25:40+02:00 | Pands | Test role-specific tracker confidence |
| 110 | `172f76b` | 2026-05-03T03:53:15+02:00 | Pands | Add final FBT latest patch state |
| 111 | `4dc5e0b` | 2026-05-03T04:36:56+02:00 | Pands | Fix free foot motion prediction for consistency filter |
| 112 | `c5dc153` | 2026-05-07T03:17:54+02:00 | Pands | Update install-auto-sync-task.ps1 |
| 113 | `18c1c2b` | 2026-05-30T23:33:36Z | Pands | Fix tracker-space alignment and VRChat OSC output |
| 114 | `5b3eb30` | 2026-05-30T23:35:29Z | Pands | Revert "Fix tracker-space alignment and VRChat OSC output" |
| 115 | `cf23e15` | 2026-05-30T23:39:03Z | Pands | Fix tracker-space alignment and VRChat OSC output |
| 116 | `8efce53` | 2026-05-30T23:41:22Z | Pands | Revert "Fix tracker-space alignment and VRChat OSC output" |
| 117 | `b69a255` | 2026-05-30T23:52:46Z | Pands | Replace repo with finished BodyTracker workspace |
| 118 | `2cf2034` | 2026-05-31T01:13:45Z | Pands | Replace repo with latest fixed BodyTracker |
| 119 | `2daf50f` | 2026-05-31T01:16:30Z | Pands | Remove uploaded BodyTracker archive |
| 120 | `452a4e7` | 2026-05-31T02:55:18Z | Pands | Replace repo with latest fixed BodyTracker |
| 121 | `ffde3f7` | 2026-06-02T10:16:52Z | Pands | Replace bt source and project files from newest archive |
| 122 | `38af978` | 2026-06-02T10:59:59Z | Pands | Add .gitignore to exclude build/vendor artifacts from archives |
| 123 | `5c1134b` | 2026-06-08T14:33:55Z | Pands | Replace repository contents with bt.zip |

## Verification note

After import, `git diff --name-status origin/master..HEAD` should show only files under `docs/imported-history/`; all prior imported commits are empty metadata commits.
