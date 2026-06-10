#!/usr/bin/env python3
"""Score floor-geometry calibration use in bodytracker replay NDJSON.

This is deliberately replay/debug based: it checks whether calibration evidence
is stable and actually used at runtime instead of trusting UI labels.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from statistics import mean, pstdev
from typing import Any


def finite(v: Any) -> bool:
    return isinstance(v, (int, float)) and math.isfinite(float(v))


def load_ndjson(path: Path) -> list[dict[str, Any]]:
    frames: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as f:
        for line_no, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            obj = json.loads(line)
            if not isinstance(obj, dict):
                raise ValueError(f"{path}:{line_no}: replay line is not a JSON object")
            frames.append(obj)
    if not frames:
        raise ValueError(f"{path}: replay contains no frames")
    return frames


def dig(obj: dict[str, Any], *keys: str) -> Any:
    cur: Any = obj
    for key in keys:
        if not isinstance(cur, dict):
            return None
        cur = cur.get(key)
    return cur


def values(frames: list[dict[str, Any]], *keys: str) -> list[float]:
    out: list[float] = []
    for frame in frames:
        v = dig(frame, *keys)
        if finite(v):
            out.append(float(v))
    return out


def values_any(frames: list[dict[str, Any]], *paths: tuple[str, ...]) -> list[float]:
    out: list[float] = []
    for frame in frames:
        for path in paths:
            v = dig(frame, *path)
            if finite(v):
                out.append(float(v))
                break
    return out


def bool_rate(frames: list[dict[str, Any]], *keys: str) -> float:
    if not frames:
        return 0.0
    return sum(1 for frame in frames if bool(dig(frame, *keys))) / len(frames)


def score(frames: list[dict[str, Any]]) -> dict[str, Any]:
    floor_conf = values_any(
        frames,
        ("tracking", "solver", "preliminary_stereo", "floor_geometry", "confidence"),
        ("solver", "preliminary_stereo", "floor_geometry", "confidence"),
    )
    floor_assist_conf = values_any(
        frames,
        ("tracking", "solver", "depth", "floor_assist", "confidence"),
        ("depth", "floor_assist", "confidence"),
    )
    drift_left = values_any(
        frames,
        ("tracking", "support", "left_foot", "anchor_drift_floor_m"),
        ("support", "left_foot", "anchor_drift_floor_m"),
    )
    drift_right = values_any(
        frames,
        ("tracking", "support", "right_foot", "anchor_drift_floor_m"),
        ("support", "right_foot", "anchor_drift_floor_m"),
    )
    reproj = values_any(
        frames,
        ("tracking", "floor_geometry", "homography_reprojection_error_px"),
        ("floor_geometry", "homography_reprojection_error_px"),
    )

    def stat(xs: list[float]) -> dict[str, float]:
        if not xs:
            return {"count": 0, "mean": 0.0, "stdev": 0.0, "max": 0.0}
        return {"count": len(xs), "mean": mean(xs), "stdev": pstdev(xs), "max": max(xs)}

    disagreement = []
    for key in ("multi_camera_yaw_delta_rad", "multi_camera_pitch_delta_rad", "multi_camera_roll_delta_rad", "multi_camera_height_delta_m"):
        disagreement.extend(values_any(frames, ("tracking", "floor_geometry", key), ("floor_geometry", key)))

    return {
        "frame_count": len(frames),
        "floor_geometry_confidence": stat(floor_conf),
        "floor_assist_confidence": stat(floor_assist_conf),
        "homography_reprojection_error_px": stat(reproj),
        "left_planted_drift_floor_m": stat(drift_left),
        "right_planted_drift_floor_m": stat(drift_right),
        "distortion_correction_used_rate": max(
            bool_rate(frames, "tracking", "solver", "depth", "floor_assist", "distortion_correction_used"),
            bool_rate(frames, "depth", "floor_assist", "distortion_correction_used"),
        ),
        "camera_orientation_used_rate": max(
            bool_rate(frames, "tracking", "solver", "depth", "floor_assist", "camera_orientation_used"),
            bool_rate(frames, "depth", "floor_assist", "camera_orientation_used"),
        ),
        "multi_camera_disagreement_signal": stat(disagreement),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Score floor-geometry calibration behavior from bodytracker replay NDJSON.")
    parser.add_argument("replay", type=Path)
    parser.add_argument("--json", action="store_true", help="Print machine-readable JSON only")
    args = parser.parse_args()
    report = score(load_ndjson(args.replay))
    if args.json:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        for key, value in report.items():
            print(f"{key}: {value}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
