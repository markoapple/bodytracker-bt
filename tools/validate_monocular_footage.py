#!/usr/bin/env python3
"""Validate monocular real-footage replay logs.

The validator intentionally works on replay NDJSON instead of raw video so the
same checks can be run after live capture, replay solve, or CI fixture export.
It does not claim validation until all required real-world scenarios are present
and every replay passes the mode, confidence, graceful-failure, snap, and debug
schema gates.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable

REQUIRED_SCENARIOS = {
    "standing",
    "stepping",
    "turning",
    "crouching",
    "seated",
    "reclined",
    "partial_occlusion",
    "poor_lighting",
}

ALLOWED_GRACEFUL_DEGRADATION = {
    "nominal",
    "occluded_predictive_hold",
    "occluded_untracked",
    "support_fallback_hold",
    "final_solve_failed",
    "bootstrap",
}

DEFAULT_MAX_FOOT_STEP_M = 0.42
DEFAULT_MAX_FOOT_SPEED_MPS = 4.25
DEFAULT_MIN_MONOCULAR_CONFIDENCE = 0.08
DEFAULT_STEREO_CONFIDENCE_MARGIN = 0.02


@dataclass
class CheckResult:
    ok: bool
    message: str
    severity: str = "fail"
    details: dict[str, Any] = field(default_factory=dict)


@dataclass
class ReplayMetrics:
    frames: int = 0
    nominal_frames: int = 0
    graceful_degraded_frames: int = 0
    bad_degradation_frames: int = 0
    mean_tracking_confidence: float = 0.0
    mean_solver_confidence: float = 0.0
    mean_foot_confidence: float = 0.0
    max_foot_step_m: float = 0.0
    max_foot_speed_mps: float = 0.0
    inferred_depth_frames: int = 0
    triangulated_frames: int = 0
    finite_pose_frames: int = 0
    missing_depth_schema_frames: int = 0
    mode_mismatch_frames: int = 0


def _is_finite_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(float(value))


def _vec3(value: Any) -> tuple[float, float, float] | None:
    if not isinstance(value, list) or len(value) != 3:
        return None
    if not all(_is_finite_number(v) for v in value):
        return None
    return (float(value[0]), float(value[1]), float(value[2]))


def _pose_position(frame: dict[str, Any], key: str) -> tuple[float, float, float] | None:
    tracking = frame.get("tracking", {})
    pose = tracking.get(key, {})
    if not isinstance(pose, dict):
        return None
    return _vec3(pose.get("position"))


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def _mean(values: Iterable[float]) -> float:
    values = list(values)
    return sum(values) / len(values) if values else 0.0


def _load_ndjson(path: Path) -> list[dict[str, Any]]:
    frames: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, start=1):
            stripped = line.strip()
            if not stripped:
                continue
            try:
                frame = json.loads(stripped)
            except json.JSONDecodeError as exc:
                raise ValueError(f"{path}: line {line_number}: invalid JSON: {exc}") from exc
            if not isinstance(frame, dict):
                raise ValueError(f"{path}: line {line_number}: each replay line must be a JSON object")
            frames.append(frame)
    if not frames:
        raise ValueError(f"{path}: replay contains no frames")
    return frames


def _solver(frame: dict[str, Any]) -> dict[str, Any]:
    tracking = frame.get("tracking", {})
    if isinstance(tracking, dict):
        solver = tracking.get("solver", {})
        if isinstance(solver, dict):
            return solver
    return {}


def _triangulation(solver: dict[str, Any]) -> dict[str, Any]:
    tri = solver.get("triangulation", {})
    if not isinstance(tri, dict):
        return {}
    prelim = tri.get("preliminary", {})
    return prelim if isinstance(prelim, dict) else {}


def _depth_schema_present(solver: dict[str, Any]) -> bool:
    if solver.get("tracking_mode") != "monocular":
        return False
    if solver.get("depth_source") != "inferred_monocular":
        return False
    depth = solver.get("depth", {})
    tri = _triangulation(solver)
    if isinstance(depth, dict):
        required = (
            depth.get("source") == "inferred_monocular",
            _is_finite_number(depth.get("confidence")),
            _is_finite_number(depth.get("mean_inferred_depth_m")),
            isinstance(depth.get("scale_source"), str),
            isinstance(depth.get("floor_assist"), dict),
        )
        if all(required):
            return True
    # Backward-compatible check for step-3/4 replay logs.
    return (
        tri.get("depth_source") == "inferred_monocular"
        and isinstance(tri.get("monocular_scale_source"), str)
        and _is_finite_number(tri.get("mean_confidence"))
        and _is_finite_number(tri.get("mean_inferred_depth_m"))
        and _is_finite_number(tri.get("monocular_floor_assist_confidence"))
    )


def _frame_confidence(frame: dict[str, Any]) -> float:
    tracking = frame.get("tracking", {})
    if isinstance(tracking, dict) and _is_finite_number(tracking.get("confidence")):
        return float(tracking["confidence"])
    return 0.0


def _solver_confidence(solver: dict[str, Any]) -> float:
    depth = solver.get("depth", {})
    if isinstance(depth, dict) and _is_finite_number(depth.get("confidence")):
        return float(depth["confidence"])
    tri = _triangulation(solver)
    if _is_finite_number(tri.get("mean_confidence")):
        return float(tri["mean_confidence"])
    return 0.0


def _foot_confidence(solver: dict[str, Any]) -> float:
    tri = _triangulation(solver)
    values: list[float] = []
    for key in ("left_foot_contact_confidence", "right_foot_contact_confidence", "foot_mean_confidence"):
        if _is_finite_number(tri.get(key)):
            values.append(float(tri[key]))
    return _mean(values)


def _timestamp(frame: dict[str, Any], fallback: float) -> float:
    value = frame.get("timestamp_seconds")
    return float(value) if _is_finite_number(value) else fallback


def analyze_replay(frames: list[dict[str, Any]]) -> ReplayMetrics:
    metrics = ReplayMetrics(frames=len(frames))
    tracking_confidences: list[float] = []
    solver_confidences: list[float] = []
    foot_confidences: list[float] = []

    previous_time: float | None = None
    previous_left: tuple[float, float, float] | None = None
    previous_right: tuple[float, float, float] | None = None

    for index, frame in enumerate(frames):
        solver = _solver(frame)
        tri = _triangulation(solver)
        degradation = str(frame.get("degradation_mode") or frame.get("tracking", {}).get("degradation_mode") or "nominal")

        if solver.get("tracking_mode") != "monocular":
            metrics.mode_mismatch_frames += 1
        if degradation == "nominal":
            metrics.nominal_frames += 1
        elif degradation in ALLOWED_GRACEFUL_DEGRADATION:
            metrics.graceful_degraded_frames += 1
        else:
            metrics.bad_degradation_frames += 1

        if _depth_schema_present(solver):
            metrics.inferred_depth_frames += 1
        else:
            metrics.missing_depth_schema_frames += 1

        if tri.get("triangulated_count", 0):
            metrics.triangulated_frames += 1

        tracking_confidences.append(_frame_confidence(frame))
        solver_confidences.append(_solver_confidence(solver))
        foot_confidences.append(_foot_confidence(solver))

        left = _pose_position(frame, "left_foot")
        right = _pose_position(frame, "right_foot")
        if left is not None and right is not None and _pose_position(frame, "root") is not None:
            metrics.finite_pose_frames += 1

        time = _timestamp(frame, fallback=index / 30.0)
        if previous_time is not None:
            dt = max(1.0 / 240.0, min(0.25, time - previous_time if time > previous_time else 1.0 / 30.0))
            for current, previous in ((left, previous_left), (right, previous_right)):
                if current is None or previous is None:
                    continue
                step = _distance(current, previous)
                metrics.max_foot_step_m = max(metrics.max_foot_step_m, step)
                metrics.max_foot_speed_mps = max(metrics.max_foot_speed_mps, step / dt)
        previous_time = time
        previous_left = left
        previous_right = right

    metrics.mean_tracking_confidence = _mean(tracking_confidences)
    metrics.mean_solver_confidence = _mean(solver_confidences)
    metrics.mean_foot_confidence = _mean(foot_confidences)
    return metrics


def _read_manifest(path: Path) -> dict[str, Any]:
    try:
        manifest = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise ValueError(f"{path}: invalid manifest JSON: {exc}") from exc
    if not isinstance(manifest, dict):
        raise ValueError(f"{path}: manifest must be a JSON object")
    if not isinstance(manifest.get("clips"), list):
        raise ValueError(f"{path}: manifest must contain a clips array")
    return manifest


def _resolve_path(base: Path, value: str | Path) -> Path:
    path = Path(value)
    return path if path.is_absolute() else base / path


def validate_manifest(manifest_path: Path) -> tuple[dict[str, Any], bool]:
    manifest = _read_manifest(manifest_path)
    base = manifest_path.parent
    thresholds = manifest.get("thresholds", {})
    max_foot_step_m = float(thresholds.get("max_foot_step_m", DEFAULT_MAX_FOOT_STEP_M))
    max_foot_speed_mps = float(thresholds.get("max_foot_speed_mps", DEFAULT_MAX_FOOT_SPEED_MPS))
    min_monocular_confidence = float(thresholds.get("min_monocular_confidence", DEFAULT_MIN_MONOCULAR_CONFIDENCE))
    stereo_confidence_margin = float(thresholds.get("stereo_confidence_margin", DEFAULT_STEREO_CONFIDENCE_MARGIN))

    clips_report: list[dict[str, Any]] = []
    checks: list[CheckResult] = []

    covered: set[str] = set()
    for clip in manifest["clips"]:
        if not isinstance(clip, dict):
            checks.append(CheckResult(False, "clip entries must be JSON objects"))
            continue
        name = str(clip.get("name", "unnamed"))
        scenario = str(clip.get("scenario", ""))
        covered.add(scenario)

        replay_value = clip.get("monocular_replay")
        if not replay_value:
            checks.append(CheckResult(False, f"{name}: missing monocular_replay path"))
            continue
        replay_path = _resolve_path(base, replay_value)

        clip_report: dict[str, Any] = {"name": name, "scenario": scenario, "monocular_replay": str(replay_path)}
        try:
            frames = _load_ndjson(replay_path)
            metrics = analyze_replay(frames)
        except Exception as exc:  # noqa: BLE001 - report validation failures as data
            checks.append(CheckResult(False, f"{name}: {exc}"))
            clip_report["error"] = str(exc)
            clips_report.append(clip_report)
            continue

        clip_report["metrics"] = metrics.__dict__

        checks.append(CheckResult(
            metrics.frames > 0,
            f"{name}: replay has frames",
            details={"frames": metrics.frames},
        ))
        checks.append(CheckResult(
            metrics.mode_mismatch_frames == 0,
            f"{name}: every frame is marked MONOCULAR",
            details={"mode_mismatch_frames": metrics.mode_mismatch_frames},
        ))
        checks.append(CheckResult(
            metrics.inferred_depth_frames == metrics.frames,
            f"{name}: every frame exposes inferred monocular depth telemetry",
            details={
                "inferred_depth_frames": metrics.inferred_depth_frames,
                "missing_depth_schema_frames": metrics.missing_depth_schema_frames,
            },
        ))
        checks.append(CheckResult(
            metrics.triangulated_frames == 0,
            f"{name}: monocular replay does not claim triangulated stereo seeds",
            details={"triangulated_frames": metrics.triangulated_frames},
        ))
        checks.append(CheckResult(
            metrics.bad_degradation_frames == 0,
            f"{name}: bad keypoints degrade through approved modes only",
            details={"bad_degradation_frames": metrics.bad_degradation_frames},
        ))
        checks.append(CheckResult(
            metrics.finite_pose_frames == metrics.frames,
            f"{name}: root and feet remain finite",
            details={"finite_pose_frames": metrics.finite_pose_frames, "frames": metrics.frames},
        ))
        checks.append(CheckResult(
            metrics.mean_tracking_confidence >= min_monocular_confidence,
            f"{name}: monocular tracking confidence stays above floor",
            details={"mean_tracking_confidence": metrics.mean_tracking_confidence, "minimum": min_monocular_confidence},
        ))
        checks.append(CheckResult(
            metrics.max_foot_step_m <= max_foot_step_m and metrics.max_foot_speed_mps <= max_foot_speed_mps,
            f"{name}: feet do not snap during plant/contact transitions",
            details={
                "max_foot_step_m": metrics.max_foot_step_m,
                "limit_step_m": max_foot_step_m,
                "max_foot_speed_mps": metrics.max_foot_speed_mps,
                "limit_speed_mps": max_foot_speed_mps,
            },
        ))

        stereo_value = clip.get("stereo_replay")
        if stereo_value:
            stereo_path = _resolve_path(base, stereo_value)
            try:
                stereo_metrics = analyze_replay(_load_ndjson(stereo_path))
                clip_report["stereo_metrics"] = stereo_metrics.__dict__
                checks.append(CheckResult(
                    metrics.mean_solver_confidence + stereo_confidence_margin <= stereo_metrics.mean_solver_confidence,
                    f"{name}: monocular confidence remains lower than comparable stereo confidence",
                    details={
                        "monocular_mean_solver_confidence": metrics.mean_solver_confidence,
                        "stereo_mean_solver_confidence": stereo_metrics.mean_solver_confidence,
                        "margin": stereo_confidence_margin,
                    },
                ))
            except Exception as exc:  # noqa: BLE001
                checks.append(CheckResult(False, f"{name}: stereo comparison replay invalid: {exc}"))

        clips_report.append(clip_report)

    missing = sorted(REQUIRED_SCENARIOS - covered)
    checks.append(CheckResult(
        not missing,
        "required real-footage scenario coverage is complete",
        details={"missing": missing, "covered": sorted(covered)},
    ))

    report = {
        "manifest": str(manifest_path),
        "required_scenarios": sorted(REQUIRED_SCENARIOS),
        "clips": clips_report,
        "checks": [check.__dict__ for check in checks],
        "passed": all(check.ok for check in checks),
    }
    return report, bool(report["passed"])


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Validate monocular tracking against recorded real-footage replay logs.")
    parser.add_argument("manifest", type=Path, help="JSON manifest listing real-footage replay logs and scenarios.")
    parser.add_argument("--report", type=Path, help="Optional JSON report output path.")
    args = parser.parse_args(argv)

    try:
        report, ok = validate_manifest(args.manifest)
    except Exception as exc:  # noqa: BLE001
        report = {"manifest": str(args.manifest), "passed": False, "error": str(exc)}
        ok = False

    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
