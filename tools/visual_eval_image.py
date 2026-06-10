#!/usr/bin/env python3
"""Visual image sanity eval for bodytracker.

This tool is intentionally strict:
* floor evidence must pass preflight before eval can proceed
* backend floor-image doctor provides floor candidates/calibration evidence
* manual keypoints are debug input only and cap the verdict
* synthetic tracker space is debug input only and cap the verdict
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import shutil
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Any
from urllib.request import Request, urlopen

try:
    import cv2
except ModuleNotFoundError:  # Logic tests do not need image I/O.
    cv2 = None
try:
    import numpy as np
except ModuleNotFoundError:
    np = None


def require_image_libs() -> None:
    if cv2 is None or np is None:
        raise RuntimeError("visual eval image operations require cv2 and numpy")


@dataclass(frozen=True)
class ImageCandidate:
    candidate_id: str
    page_url: str
    download_url: str
    license_note: str
    selection_reason: str
    file_name: str
    floor_y_min: float
    keypoints_norm: dict[str, tuple[float, float]]
    evidence_boxes_norm: dict[str, tuple[float, float, float, float]]


SELECTED_PEXELS = ImageCandidate(
    candidate_id="pexels_6217736_gym_wood_floor",
    page_url="https://www.pexels.com/photo/person-standing-on-wooden-floor-6217736/",
    download_url="https://images.pexels.com/photos/6217736/pexels-photo-6217736.jpeg?cs=srgb&dl=pexels-cottonbro-6217736.jpg&fm=jpg",
    license_note="Pexels License: free to use; attribution not required; modifications allowed under Pexels terms.",
    selection_reason="clear repeated wood-plank gym floor, visible feet, full-body standing person, usable perspective, low signage clutter",
    file_name="source_pexels_6217736.jpg",
    floor_y_min=0.73,
    keypoints_norm={
        "head": (0.505, 0.465),
        "neck": (0.505, 0.505),
        "pelvis": (0.505, 0.585),
        "left_hip": (0.470, 0.585),
        "right_hip": (0.540, 0.585),
        "left_knee": (0.468, 0.675),
        "right_knee": (0.542, 0.675),
        "left_ankle": (0.458, 0.755),
        "right_ankle": (0.555, 0.755),
        "left_foot": (0.455, 0.778),
        "right_foot": (0.565, 0.778),
    },
    evidence_boxes_norm={
        "pelvis": (0.450, 0.555, 0.560, 0.615),
        "left_knee": (0.430, 0.635, 0.498, 0.705),
        "right_knee": (0.512, 0.635, 0.580, 0.705),
        "left_foot": (0.420, 0.745, 0.498, 0.800),
        "right_foot": (0.532, 0.745, 0.602, 0.800),
        "left_thigh": (0.430, 0.575, 0.505, 0.690),
        "right_thigh": (0.505, 0.575, 0.580, 0.690),
        "left_shin": (0.430, 0.660, 0.488, 0.775),
        "right_shin": (0.525, 0.660, 0.585, 0.775),
    },
)

PREVIOUS_BAD_STAGE = ImageCandidate(
    candidate_id="previous_unsplash_bodybuilding_stage",
    page_url="https://unsplash.com/photos/a-man-standing-on-top-of-a-wooden-floor-vKoExePJGpc",
    download_url="https://unsplash.com/photos/vKoExePJGpc/download?force=true&w=1200",
    license_note="Unsplash License; source recorded; rejected for this eval.",
    selection_reason="known bad negative-control image: stage/signage/body edges contaminate floor evidence",
    file_name="source_unsplash_vKoExePJGpc.jpg",
    floor_y_min=0.70,
    keypoints_norm={
        "head": (0.520, 0.245),
        "neck": (0.520, 0.345),
        "pelvis": (0.515, 0.585),
        "left_hip": (0.425, 0.585),
        "right_hip": (0.592, 0.590),
        "left_knee": (0.405, 0.755),
        "right_knee": (0.595, 0.740),
        "left_ankle": (0.325, 0.910),
        "right_ankle": (0.603, 0.902),
        "left_foot": (0.350, 0.930),
        "right_foot": (0.625, 0.918),
    },
    evidence_boxes_norm={
        "pelvis": (0.45, 0.55, 0.58, 0.65),
        "left_knee": (0.33, 0.68, 0.46, 0.81),
        "right_knee": (0.49, 0.58, 0.57, 0.68),
        "left_foot": (0.24, 0.88, 0.39, 0.98),
        "right_foot": (0.68, 0.88, 0.80, 0.98),
        "left_thigh": (0.34, 0.55, 0.48, 0.77),
        "right_thigh": (0.47, 0.54, 0.58, 0.70),
        "left_shin": (0.25, 0.72, 0.42, 0.95),
        "right_shin": (0.63, 0.70, 0.79, 0.93),
    },
)

CANDIDATES = {
    SELECTED_PEXELS.candidate_id: SELECTED_PEXELS,
    PREVIOUS_BAD_STAGE.candidate_id: PREVIOUS_BAD_STAGE,
}
# Default eval must run the known bad stage image first as a negative control.
# The selector only proceeds to the positive candidate after the negative fails
# strict floor/body evidence gates.
DEFAULT_SELECTION_ORDER = (PREVIOUS_BAD_STAGE.candidate_id, SELECTED_PEXELS.candidate_id)

SKELETON = (
    ("neck", "pelvis"),
    ("pelvis", "left_hip"),
    ("pelvis", "right_hip"),
    ("left_hip", "left_knee"),
    ("left_knee", "left_ankle"),
    ("left_ankle", "left_foot"),
    ("right_hip", "right_knee"),
    ("right_knee", "right_ankle"),
    ("right_ankle", "right_foot"),
)


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def find_bodytracker_exe(root: Path, override: str | None) -> Path | None:
    if override:
        p = Path(override)
        return p if p.exists() else None
    for rel in ("build/debug/Debug/bodytracker.exe", "build/Debug/bodytracker.exe", "build/bodytracker.exe"):
        p = root / rel
        if p.exists():
            return p
    return None


def download_image(candidate: ImageCandidate, path: Path) -> None:
    if path.exists() and path.stat().st_size > 0:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    req = Request(candidate.download_url, headers={"User-Agent": "bodytracker-visual-eval/2.0"})
    with urlopen(req, timeout=60) as response:
        path.write_bytes(response.read())


def file_sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def safe_file_sha256(path: Path) -> str:
    try:
        return file_sha256(path)
    except OSError:
        return ""


def floor_source_from_report(floor_report: dict[str, Any]) -> str:
    calibration = floor_report.get("detection_debug", {}).get("calibration", {})
    if not calibration.get("valid"):
        return "missing"
    source = calibration.get("source") or floor_report.get("source")
    if isinstance(source, str) and source.strip():
        return source.strip()
    return "backend_floor_doctor"


def run_floor_doctor(root: Path, exe: Path, image_path: Path, output_dir: Path) -> dict[str, Any]:
    input_dir = output_dir / "floor_doctor_input"
    doctor_dir = output_dir / "floor_doctor"
    input_dir.mkdir(parents=True, exist_ok=True)
    doctor_dir.mkdir(parents=True, exist_ok=True)
    doctor_image = input_dir / image_path.name
    if doctor_image.resolve() != image_path.resolve():
        shutil.copyfile(image_path, doctor_image)
    cmd = [str(exe), "--floor-image-doctor", str(input_dir), str(doctor_dir)]
    proc = subprocess.run(cmd, cwd=root, text=True, capture_output=True, timeout=120)
    if proc.returncode != 0:
        return {
            "ok": False,
            "status": "floor-image doctor failed",
            "command": cmd,
            "stdout": proc.stdout,
            "stderr": proc.stderr,
            "detection_debug": {"calibration": {}, "candidates": []},
        }
    report_path = doctor_dir / f"{doctor_image.stem}.json"
    if not report_path.exists():
        return {
            "ok": False,
            "status": f"floor-image doctor did not write {report_path}",
            "command": cmd,
            "stdout": proc.stdout,
            "stderr": proc.stderr,
            "detection_debug": {"calibration": {}, "candidates": []},
        }
    with report_path.open("r", encoding="utf-8") as f:
        report = json.load(f)
    report["_doctor_ok"] = True
    report["_doctor_json"] = str(report_path)
    report["_doctor_stdout"] = proc.stdout
    return report


def point_px(norm: tuple[float, float], w: int, h: int) -> tuple[int, int]:
    return int(round(norm[0] * w)), int(round(norm[1] * h))


def keypoints_px(candidate: ImageCandidate, w: int, h: int) -> dict[str, tuple[int, int]]:
    return {name: point_px(value, w, h) for name, value in candidate.keypoints_norm.items()}


def box_px(box: tuple[float, float, float, float], w: int, h: int) -> tuple[int, int, int, int]:
    x0, y0, x1, y1 = box
    return int(round(x0 * w)), int(round(y0 * h)), int(round(x1 * w)), int(round(y1 * h))


def boxes_px(candidate: ImageCandidate, w: int, h: int) -> dict[str, tuple[int, int, int, int]]:
    return {name: box_px(value, w, h) for name, value in candidate.evidence_boxes_norm.items()}


def line_angle(line: dict[str, Any]) -> float | None:
    a = line.get("a")
    b = line.get("b")
    if not (isinstance(a, list) and isinstance(b, list) and len(a) == 2 and len(b) == 2):
        return None
    return math.atan2(float(b[1]) - float(a[1]), float(b[0]) - float(a[0]))


def line_mid_y(line: dict[str, Any]) -> float | None:
    a = line.get("a")
    b = line.get("b")
    if not (isinstance(a, list) and isinstance(b, list) and len(a) == 2 and len(b) == 2):
        return None
    return 0.5 * (float(a[1]) + float(b[1]))


def line_length(line: dict[str, Any]) -> float:
    a = line.get("a")
    b = line.get("b")
    if not (isinstance(a, list) and isinstance(b, list) and len(a) == 2 and len(b) == 2):
        return 0.0
    return math.hypot(float(b[0]) - float(a[0]), float(b[1]) - float(a[1]))


def angle_coherence(angles: list[float]) -> float:
    if len(angles) < 2:
        return 0.0
    doubled = np.array([2.0 * a for a in angles], dtype=np.float64)
    r = math.hypot(float(np.cos(doubled).mean()), float(np.sin(doubled).mean()))
    return max(0.0, min(1.0, r))


def orientation_family_count(angles: list[float]) -> int:
    if not angles:
        return 0
    bins: dict[int, int] = {}
    for angle in angles:
        wrapped = (angle + math.pi) % math.pi
        bucket = int(round(wrapped / (math.pi / 12.0))) % 12
        bins[bucket] = bins.get(bucket, 0) + 1
    return sum(1 for count in bins.values() if count >= 3)


def dominant_angle(angles: list[float], fallback: float = 0.0) -> float:
    if not angles:
        return fallback
    doubled = np.array([2.0 * a for a in angles], dtype=np.float64)
    return 0.5 * math.atan2(float(np.sin(doubled).mean()), float(np.cos(doubled).mean()))


def clamp01(value: float) -> float:
    if not math.isfinite(value):
        return 0.0
    return max(0.0, min(1.0, value))


def preflight_image(
    image_path: Path,
    candidate: ImageCandidate,
    floor_report: dict[str, Any],
) -> dict[str, Any]:
    img = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if img is None:
        return {
            "accepted": False,
            "reasons": [f"image not readable: {image_path}"],
            "metrics": {},
            "selected_image_url": candidate.page_url,
            "license": candidate.license_note,
            "selection_note": "rejected",
            "candidate_id": candidate.candidate_id,
            "file_name": candidate.file_name,
            "sha256": safe_file_sha256(image_path),
        }
    h, w = img.shape[:2]
    debug = floor_report.get("detection_debug", {})
    calibration = debug.get("calibration", {})
    candidates = debug.get("candidates", [])
    accepted = [c for c in candidates if c.get("accepted")]
    floor_y = candidate.floor_y_min * float(h)
    accepted_floor = [c for c in accepted if (line_mid_y(c.get("line", {})) or 0.0) >= floor_y]
    floor_lengths = [line_length(c.get("line", {})) for c in accepted_floor]
    accepted_lengths = [line_length(c.get("line", {})) for c in accepted]
    floor_length = sum(floor_lengths)
    accepted_length = max(1.0, sum(accepted_lengths))
    floor_ys = [line_mid_y(c.get("line", {})) for c in accepted_floor]
    floor_ys = [y for y in floor_ys if y is not None]
    floor_span = (max(floor_ys) - min(floor_ys)) if len(floor_ys) >= 2 else 0.0
    visible_floor_region = clamp01((h - floor_y) / max(1.0, 0.26 * h))
    line_span_score = clamp01(floor_span / max(1.0, 0.12 * (h - floor_y)))
    line_count_score = clamp01(len(accepted_floor) / 10.0)
    floor_coverage = clamp01(0.45 * visible_floor_region + 0.35 * line_count_score + 0.20 * line_span_score)
    line_purity = clamp01(floor_length / accepted_length)
    floor_angles = [a for c in accepted_floor for a in [line_angle(c.get("line", {}))] if a is not None]
    perspective_coherence = angle_coherence(floor_angles)
    family_count = orientation_family_count(floor_angles)
    pattern_count_score = clamp01(len(accepted_floor) / 8.0)
    backend_conf = float(calibration.get("metric_scale_confidence", 0.0) or calibration.get("family_a", {}).get("confidence", 0.0) or 0.0)
    family_score = 1.0 if family_count >= 2 else 0.75 if len(accepted_floor) >= 10 and perspective_coherence >= 0.75 else 0.0
    pattern_strength = clamp01(0.40 * backend_conf + 0.30 * pattern_count_score + 0.20 * perspective_coherence + 0.10 * family_score)

    boxes = boxes_px(candidate, w, h)
    foot_visibility = 1.0 if "left_foot" in boxes and "right_foot" in boxes else 0.0
    limb_visibility = 1.0 if all(k in boxes for k in ("left_knee", "right_knee", "left_thigh", "right_thigh", "left_shin", "right_shin")) else 0.0
    floor_too_small = h - floor_y < 0.24 * h
    floor_backend_valid = bool(calibration.get("valid"))
    reasons: list[str] = []
    floor_line_fraction = len(accepted_floor) / max(1, len(accepted))
    if floor_too_small or floor_coverage < 0.55:
        reasons.append("large visible lower-floor region was not supported by backend floor-line evidence")
    if pattern_strength < 0.62:
        reasons.append("floor pattern was not an obvious repeated plank/tile structure")
    if family_count < 2 and not (len(accepted_floor) >= 10 and perspective_coherence >= 0.75 and backend_conf >= 0.35):
        reasons.append("floor seams did not form two line families or one strong repeated family")
    if line_purity < 0.78 or floor_line_fraction < 0.72:
        reasons.append("dominant detected lines came from signage/stage trim/body/clutter, not floor seams")
    if perspective_coherence < 0.60:
        reasons.append("accepted floor lines were not perspective-coherent enough")
    if foot_visibility < 0.75:
        reasons.append("feet were not visible enough to judge foot-floor contact")
    if limb_visibility < 0.75:
        reasons.append("lower limbs were not visible enough to judge body overlay correctness")
    if backend_conf < 0.62:
        reasons.append("backend metric-scale confidence was too weak for runtime evidence")
    if not floor_backend_valid:
        reasons.append(f"backend floor calibration rejected image: {calibration.get('reason', 'unknown')}")

    metrics = {
        "floor_coverage": round(float(floor_coverage), 3),
        "pattern_strength": round(float(pattern_strength), 3),
        "line_purity": round(float(line_purity), 3),
        "perspective_coherence": round(float(perspective_coherence), 3),
        "foot_visibility": round(float(foot_visibility), 3),
        "limb_visibility": round(float(limb_visibility), 3),
        "accepted_floor_line_count": len(accepted_floor),
        "accepted_line_count": len(accepted),
        "accepted_orientation_families": family_count,
        "floor_line_fraction": round(float(floor_line_fraction), 3),
        "backend_metric_scale_confidence": round(float(backend_conf), 3),
    }
    accepted_preflight = not reasons
    return {
        "accepted": accepted_preflight,
        "reasons": reasons,
        "metrics": metrics,
        "selected_image_url": candidate.page_url,
        "license": candidate.license_note,
        "selection_note": candidate.selection_reason if accepted_preflight else "rejected",
        "candidate_id": candidate.candidate_id,
        "file_name": candidate.file_name,
        "sha256": safe_file_sha256(image_path),
    }


def select_candidate(
    root: Path,
    output_dir: Path,
    exe: Path | None,
    requested_candidate: str | None,
) -> tuple[ImageCandidate, Path, dict[str, Any], dict[str, Any], list[dict[str, Any]], bool]:
    rejected: list[dict[str, Any]] = []
    candidate_ids = (requested_candidate,) if requested_candidate else DEFAULT_SELECTION_ORDER
    if requested_candidate and requested_candidate not in CANDIDATES:
        raise ValueError(f"unknown candidate {requested_candidate}; choices: {', '.join(CANDIDATES)}")
    last: tuple[ImageCandidate, Path, dict[str, Any], dict[str, Any], bool] | None = None
    for candidate_id in candidate_ids:
        candidate = CANDIDATES[candidate_id]
        image_path = output_dir / candidate.file_name
        download_image(candidate, image_path)
        if exe:
            floor_report = run_floor_doctor(root, exe, image_path, output_dir)
            used_runtime_paths = bool(floor_report.get("_doctor_ok"))
        else:
            floor_report = {
                "ok": False,
                "status": "bodytracker.exe not found; floor backend not run",
                "detection_debug": {"calibration": {}, "candidates": []},
            }
            used_runtime_paths = False
        preflight = preflight_image(image_path, candidate, floor_report)
        last = (candidate, image_path, floor_report, preflight, used_runtime_paths)
        if preflight.get("accepted"):
            return candidate, image_path, floor_report, preflight, rejected, used_runtime_paths
        rejected.append(preflight)
        if requested_candidate:
            break
    if last is None:
        raise RuntimeError("no image candidates configured")
    candidate, image_path, floor_report, preflight, used_runtime_paths = last
    return candidate, image_path, floor_report, preflight, rejected, used_runtime_paths


def draw_label(img: np.ndarray, text: str, xy: tuple[int, int], color: tuple[int, int, int]) -> None:
    x, y = xy
    cv2.putText(img, text, (x + 5, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(img, text, (x + 5, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.44, color, 1, cv2.LINE_AA)


def draw_floor_grid(img: np.ndarray, calibration: dict[str, Any], candidates: list[dict[str, Any]], candidate: ImageCandidate) -> None:
    h, w = img.shape[:2]
    floor_y = int(round(candidate.floor_y_min * h))
    accepted_floor = [c for c in candidates if c.get("accepted") and (line_mid_y(c.get("line", {})) or 0.0) >= floor_y]
    angles = [a for c in accepted_floor for a in [line_angle(c.get("line", {}))] if a is not None]
    theta = dominant_angle(angles, float(calibration.get("family_a", {}).get("orientation_rad", 0.0) or 0.0))
    floor_poly = np.array([[0, floor_y], [w - 1, floor_y], [w - 1, h - 1], [0, h - 1]], dtype=np.int32)
    layer = img.copy()
    cv2.fillPoly(layer, [floor_poly], (25, 95, 80))
    cv2.addWeighted(layer, 0.18, img, 0.82, 0, img)

    dx = math.cos(theta)
    dy = math.sin(theta)
    nx = -dy
    ny = dx
    spacing = float(calibration.get("family_a", {}).get("spacing_px", 50.0) or 50.0)
    spacing = max(18.0, min(spacing, 130.0))
    center = np.array([w * 0.5, h * 0.78], dtype=np.float32)
    length = max(w, h)
    for k in range(-12, 13):
        c = center + np.array([nx, ny], dtype=np.float32) * (k * spacing)
        p1 = c - np.array([dx, dy], dtype=np.float32) * length
        p2 = c + np.array([dx, dy], dtype=np.float32) * length
        cv2.line(img, tuple(np.round(p1).astype(int)), tuple(np.round(p2).astype(int)), (70, 210, 225), 1, cv2.LINE_AA)
    cv2.putText(img, "inferred floor/grid from accepted lower-floor seams", (18, floor_y + 24), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (70, 210, 225), 2, cv2.LINE_AA)


def draw_candidates(img: np.ndarray, candidates: list[dict[str, Any]], candidate: ImageCandidate) -> None:
    h = img.shape[0]
    floor_y = candidate.floor_y_min * h
    for c in candidates:
        line = c.get("line", {})
        a = line.get("a")
        b = line.get("b")
        if not (isinstance(a, list) and isinstance(b, list) and len(a) == 2 and len(b) == 2):
            continue
        mid_y = line_mid_y(line) or 0.0
        if c.get("accepted") and mid_y >= floor_y:
            color = (0, 235, 80)
            thickness = 2
        elif c.get("accepted"):
            color = (0, 165, 255)
            thickness = 2
        else:
            color = (40, 40, 190)
            thickness = 1
        cv2.line(img, (int(a[0]), int(a[1])), (int(b[0]), int(b[1])), color, thickness, cv2.LINE_AA)
        if c.get("accepted") and mid_y >= floor_y:
            for p in line.get("samples", [])[:40]:
                if isinstance(p, list) and len(p) == 2:
                    cv2.circle(img, (int(p[0]), int(p[1])), 2, (0, 240, 255), -1, cv2.LINE_AA)


def tracker_positions(kp: dict[str, tuple[int, int]]) -> dict[str, tuple[int, int]]:
    return {
        "pelvis/waist": kp["pelvis"],
        "left foot": (kp["left_foot"][0], kp["left_foot"][1] - 8),
        "right foot": (kp["right_foot"][0], kp["right_foot"][1] - 8),
        "left knee": kp["left_knee"],
        "right knee": kp["right_knee"],
    }


def point_box_score(point: tuple[int, int], box: tuple[int, int, int, int]) -> float:
    x, y = point
    x0, y0, x1, y1 = box
    if x0 <= x <= x1 and y0 <= y <= y1:
        return 1.0
    dx = max(x0 - x, 0, x - x1)
    dy = max(y0 - y, 0, y - y1)
    diag = max(1.0, math.hypot(x1 - x0, y1 - y0))
    return clamp01(1.0 - math.hypot(dx, dy) / (0.65 * diag))


def segment_box_score(a: tuple[int, int], b: tuple[int, int], box: tuple[int, int, int, int]) -> float:
    mx = 0.5 * (a[0] + b[0])
    my = 0.5 * (a[1] + b[1])
    return point_box_score((int(round(mx)), int(round(my))), box)


def body_checks(kp: dict[str, tuple[int, int]], boxes: dict[str, tuple[int, int, int, int]]) -> dict[str, dict[str, float]]:
    checks = {
        "left_leg": {
            "hip_to_knee_alignment": segment_box_score(kp["left_hip"], kp["left_knee"], boxes["left_thigh"]),
            "knee_on_visible_joint": point_box_score(kp["left_knee"], boxes["left_knee"]),
            "knee_to_foot_alignment": segment_box_score(kp["left_knee"], kp["left_foot"], boxes["left_shin"]),
            "foot_on_visible_contact": point_box_score(kp["left_foot"], boxes["left_foot"]),
        },
        "right_leg": {
            "hip_to_knee_alignment": segment_box_score(kp["right_hip"], kp["right_knee"], boxes["right_thigh"]),
            "knee_on_visible_joint": point_box_score(kp["right_knee"], boxes["right_knee"]),
            "knee_to_foot_alignment": segment_box_score(kp["right_knee"], kp["right_foot"], boxes["right_shin"]),
            "foot_on_visible_contact": point_box_score(kp["right_foot"], boxes["right_foot"]),
        },
    }
    return {side: {name: round(float(score), 3) for name, score in values.items()} for side, values in checks.items()}


def draw_body_and_trackers(
    img: np.ndarray,
    kp: dict[str, tuple[int, int]],
    trackers: dict[str, tuple[int, int]],
    boxes: dict[str, tuple[int, int, int, int]],
    checks: dict[str, dict[str, float]],
) -> list[str]:
    flags: list[str] = []
    for name, box in boxes.items():
        if name in {"left_thigh", "right_thigh", "left_shin", "right_shin"}:
            continue
        x0, y0, x1, y1 = box
        cv2.rectangle(img, (x0, y0), (x1, y1), (80, 130, 255), 1, cv2.LINE_AA)
    for a, b in SKELETON:
        cv2.line(img, kp[a], kp[b], (255, 210, 40), 4, cv2.LINE_AA)
        cv2.line(img, kp[a], kp[b], (20, 20, 20), 1, cv2.LINE_AA)
    for name, p in kp.items():
        if name in {"head", "neck", "left_hip", "right_hip"}:
            continue
        cv2.circle(img, p, 5, (255, 210, 40), -1, cv2.LINE_AA)

    if checks["left_leg"]["knee_on_visible_joint"] < 0.65:
        flags.append("left knee marker not on visible knee")
        draw_label(img, "BAD left knee", kp["left_knee"], (0, 80, 255))
    if checks["right_leg"]["knee_on_visible_joint"] < 0.65:
        flags.append("right knee marker not on visible knee")
        draw_label(img, "BAD right knee", kp["right_knee"], (0, 80, 255))
    if checks["left_leg"]["knee_to_foot_alignment"] < 0.55 or checks["left_leg"]["hip_to_knee_alignment"] < 0.55:
        flags.append("left leg segment does not follow visible limb")
    if checks["right_leg"]["knee_to_foot_alignment"] < 0.55 or checks["right_leg"]["hip_to_knee_alignment"] < 0.55:
        flags.append("right leg segment does not follow visible limb")

    for label, p in trackers.items():
        cv2.circle(img, p, 11, (255, 70, 210), 2, cv2.LINE_AA)
        cv2.drawMarker(img, p, (255, 70, 210), cv2.MARKER_CROSS, 18, 2, cv2.LINE_AA)
        draw_label(img, label, p, (255, 70, 210))
    origin = trackers["pelvis/waist"]
    cv2.arrowedLine(img, origin, (origin[0] + 60, origin[1]), (40, 80, 255), 2, cv2.LINE_AA, tipLength=0.20)
    cv2.arrowedLine(img, origin, (origin[0], origin[1] - 60), (40, 255, 80), 2, cv2.LINE_AA, tipLength=0.20)
    draw_label(img, "synthetic axes", (origin[0] + 8, origin[1] - 65), (255, 255, 255))
    return flags


def compute_scores(
    preflight: dict[str, Any],
    floor_report: dict[str, Any],
    checks: dict[str, dict[str, float]],
    trackers: dict[str, tuple[int, int]],
    boxes: dict[str, tuple[int, int, int, int]],
    pose_source: str,
    tracker_space_source: str,
) -> dict[str, Any]:
    metrics = preflight.get("metrics", {})
    calibration = floor_report.get("detection_debug", {}).get("calibration", {})
    floor_conf = float(metrics.get("backend_metric_scale_confidence", 0.0))
    floor_score = clamp01(0.55 * floor_conf + 0.45 * float(metrics.get("floor_coverage", 0.0)))
    pattern_score = clamp01(0.50 * float(metrics.get("pattern_strength", 0.0)) + 0.30 * float(metrics.get("line_purity", 0.0)) + 0.20 * float(metrics.get("perspective_coherence", 0.0)))
    left_values = list(checks["left_leg"].values())
    right_values = list(checks["right_leg"].values())
    left_leg_score = min(left_values)
    right_leg_score = min(right_values)
    body_score = clamp01(0.5 * (sum(left_values) / len(left_values)) + 0.5 * (sum(right_values) / len(right_values)))
    left_tracker_score = point_box_score(trackers["left foot"], boxes["left_foot"])
    right_tracker_score = point_box_score(trackers["right foot"], boxes["right_foot"])
    pelvis_tracker_score = point_box_score(trackers["pelvis/waist"], boxes["pelvis"])
    tracker_score = clamp01(0.35 * left_tracker_score + 0.35 * right_tracker_score + 0.30 * pelvis_tracker_score)

    flags: list[str] = []
    if pose_source != "model":
        flags.append("pose was manual_keypoints, not model output")
    if tracker_space_source == "synthetic":
        flags.append("SteamVR transform unavailable; tracker positions shown in image-local synthetic debug space")
    if tracker_space_source == "missing":
        flags.append("tracker_space_source missing")
    if not preflight.get("accepted"):
        flags.append("floor preflight did not pass")
    if not calibration.get("valid"):
        flags.append(f"backend floor calibration rejected image: {calibration.get('reason', 'unknown')}")
    if left_tracker_score < 0.65:
        flags.append("left foot tracker not aligned with visible foot-floor contact")
    if right_tracker_score < 0.65:
        flags.append("right foot tracker not aligned with visible foot-floor contact")
    if left_tracker_score < 0.65 or right_tracker_score < 0.65:
        flags.append("tracker marker exists but does not correspond to visible anatomy")
        tracker_score = min(tracker_score, 0.50)
    if checks["left_leg"]["knee_on_visible_joint"] < 0.65:
        flags.append("left knee marker not on visible knee")
        body_score = min(body_score, 0.60)
    if checks["right_leg"]["knee_on_visible_joint"] < 0.65:
        flags.append("right knee marker not on visible knee")
        body_score = min(body_score, 0.60)
    if left_leg_score < 0.55:
        flags.append("left leg segment does not follow visible limb")
    if right_leg_score < 0.55:
        flags.append("right leg segment does not follow visible limb")
    if left_leg_score < 0.55 or right_leg_score < 0.55:
        flags.append("manual keypoints failed image-evidence validation")
        flags.append("body score capped by failed limb evidence")
        body_score = min(body_score, 0.50)

    raw_scores = {
        "floor_plane_plausibility": floor_score,
        "pattern_alignment": pattern_score,
        "body_outline_plausibility": body_score,
        "tracker_position_plausibility": tracker_score,
    }

    pose_cap = 1.00 if pose_source == "model" else 0.60 if pose_source == "manual_keypoints" else 0.25
    if tracker_space_source in {"steamvr_controller", "manual_fallback"}:
        tracker_cap = 1.00
    elif tracker_space_source == "synthetic":
        tracker_cap = 0.45 if pose_source == "model" else 0.35
    elif tracker_space_source == "missing":
        tracker_cap = 0.10
    else:
        tracker_cap = 0.25
    floor_strong = preflight.get("accepted") and min(
        float(metrics.get("floor_coverage", 0.0)),
        float(metrics.get("pattern_strength", 0.0)),
        float(metrics.get("line_purity", 0.0)),
        float(metrics.get("perspective_coherence", 0.0)),
    ) >= 0.50
    floor_cap = 1.00 if floor_strong else 0.25
    body_cap = 1.00
    if min(left_leg_score, right_leg_score) < 0.55:
        body_cap = 0.50
    if min(left_leg_score, right_leg_score) < 0.35:
        body_cap = 0.35
    if tracker_space_source == "missing":
        body_cap = min(body_cap, 0.10)

    capped_body_score = body_score
    capped_tracker_score = tracker_score
    if pose_source == "manual_keypoints":
        capped_body_score = min(capped_body_score, pose_cap)
        flags.append("body score capped by manual keypoint debug evidence")
    elif pose_source != "model":
        capped_body_score = min(capped_body_score, pose_cap)
        flags.append(f"body score capped by {pose_source} pose evidence")
    if tracker_space_source == "synthetic":
        capped_tracker_score = min(capped_tracker_score, tracker_cap)
        flags.append("tracker score capped by synthetic tracker-space debug evidence")
    elif tracker_space_source != "steamvr_controller" and tracker_space_source != "manual_fallback":
        capped_tracker_score = min(capped_tracker_score, tracker_cap)
        flags.append(f"tracker score capped by {tracker_space_source} tracker-space evidence")

    capped_scores = {
        "floor_plane_plausibility": min(floor_score, floor_cap),
        "pattern_alignment": pattern_score,
        "body_outline_plausibility": min(capped_body_score, body_cap),
        "tracker_position_plausibility": capped_tracker_score,
    }

    applied_cap = min(pose_cap, tracker_cap, floor_cap, body_cap)
    raw_overall = clamp01(0.30 * floor_score + 0.25 * pattern_score + 0.25 * body_score + 0.20 * tracker_score)
    capped_overall = clamp01(
        0.30 * capped_scores["floor_plane_plausibility"] +
        0.25 * capped_scores["pattern_alignment"] +
        0.25 * capped_scores["body_outline_plausibility"] +
        0.20 * capped_scores["tracker_position_plausibility"]
    )
    overall = min(capped_overall, applied_cap)
    debug_only_evidence = pose_source != "model" or tracker_space_source == "synthetic"
    if tracker_space_source == "missing":
        verdict = "fail"
    elif overall >= 0.75 and applied_cap >= 0.95 and not debug_only_evidence:
        verdict = "pass"
    elif overall >= 0.25:
        verdict = "partial"
    else:
        verdict = "fail"
    if debug_only_evidence:
        flags.append("debug-only evidence cannot produce a pass verdict")

    return {
        "raw_scores": {k: round(float(v), 3) for k, v in raw_scores.items()},
        "capped_scores": {k: round(float(v), 3) for k, v in capped_scores.items()},
        "raw_overall": round(float(raw_overall), 3),
        "body_checks": checks,
        "evidence_caps": {
            "pose_cap": round(float(pose_cap), 3),
            "tracker_cap": round(float(tracker_cap), 3),
            "floor_cap": round(float(floor_cap), 3),
            "body_cap": round(float(body_cap), 3),
            "applied_cap": round(float(applied_cap), 3),
        },
        "overall": round(float(overall), 3),
        "verdict": verdict,
        "flags": flags,
    }


def write_overlay(
    image_path: Path,
    candidate: ImageCandidate,
    floor_report: dict[str, Any],
    preflight: dict[str, Any],
    scoring: dict[str, Any],
    out_path: Path,
) -> None:
    img = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if img is None:
        raise ValueError(f"could not read image: {image_path}")
    debug = floor_report.get("detection_debug", {})
    calibration = debug.get("calibration", {})
    floor_source = floor_source_from_report(floor_report)
    candidates = debug.get("candidates", [])
    draw_floor_grid(img, calibration, candidates, candidate)
    draw_candidates(img, candidates, candidate)
    h, w = img.shape[:2]
    kp = keypoints_px(candidate, w, h)
    boxes = boxes_px(candidate, w, h)
    trackers = tracker_positions(kp)
    draw_flags = draw_body_and_trackers(img, kp, trackers, boxes, scoring["body_checks"])
    for flag in draw_flags:
        if flag not in scoring["flags"]:
            scoring["flags"].append(flag)

    banner = "DEBUG ONLY / NOT A PASS: manual_keypoints + synthetic tracker_space"
    cv2.rectangle(img, (0, 0), (w - 1, 42), (0, 0, 0), -1)
    cv2.putText(img, banner, (18, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (0, 220, 255), 2, cv2.LINE_AA)

    panel_w = 500
    panel = np.zeros((h, panel_w, 3), dtype=np.uint8)
    panel[:] = (28, 31, 34)
    rows = [
        "bodytracker visual eval",
        f"candidate: {candidate.candidate_id}",
        f"preflight: {'PASS' if preflight.get('accepted') else 'FAIL'}",
        f"floor: {floor_source}",
        "pose: manual_keypoints",
        "tracker space: synthetic",
        f"floor: {scoring['raw_scores']['floor_plane_plausibility']:.2f}",
        f"pattern: {scoring['raw_scores']['pattern_alignment']:.2f}",
        f"body: {scoring['capped_scores']['body_outline_plausibility']:.2f}",
        f"trackers: {scoring['capped_scores']['tracker_position_plausibility']:.2f}",
        f"cap: {scoring['evidence_caps']['applied_cap']:.2f}",
        f"overall: {scoring['overall']:.2f} {scoring['verdict']}",
    ]
    y = 32
    for i, row in enumerate(rows):
        color = (240, 240, 240)
        if i == 2 and not preflight.get("accepted"):
            color = (0, 90, 255)
        if i == len(rows) - 1:
            color = (80, 220, 80) if scoring["verdict"] == "pass" else (0, 210, 255) if scoring["verdict"] == "partial" else (0, 80, 255)
        cv2.putText(panel, row, (18, y), cv2.FONT_HERSHEY_SIMPLEX, 0.58, color, 1, cv2.LINE_AA)
        y += 30
    y += 8
    for flag in scoring["flags"][:12]:
        for part in [flag[i:i + 54] for i in range(0, len(flag), 54)]:
            cv2.putText(panel, part, (18, y), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (185, 215, 255), 1, cv2.LINE_AA)
            y += 22

    combined = np.hstack([img, panel])
    out_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out_path), combined)


def build_report(
    candidate: ImageCandidate,
    preflight: dict[str, Any],
    floor_report: dict[str, Any],
    scoring: dict[str, Any],
    overlay_path: Path,
    debug_path: Path,
    used_runtime_paths: bool,
    image_sha256: str,
) -> dict[str, Any]:
    floor_source = floor_source_from_report(floor_report)
    explanation = (
        "Preflight passed and backend floor detector ran, but pose is manual and tracker space is synthetic, so result is debug-only partial evidence."
        if scoring["verdict"] == "partial"
        else "Evaluation failed: image/preflight/evidence-source gates did not support runtime correctness."
    )
    return {
        "image_source": {
            "url": candidate.page_url,
            "license": candidate.license_note,
            "selection_reason": candidate.selection_reason,
            "preflight_passed": bool(preflight.get("accepted")),
            "sha256": image_sha256,
            "file_name": candidate.file_name,
        },
        "pipeline": {
            "floor_source": floor_source,
            "pose_source": "manual_keypoints",
            "tracker_space_source": "synthetic",
            "used_runtime_paths": used_runtime_paths,
        },
        "preflight": preflight.get("metrics", {}),
        "raw_scores": scoring["raw_scores"],
        "capped_scores": scoring["capped_scores"],
        "raw_overall": scoring["raw_overall"],
        "body_checks": scoring["body_checks"],
        "evidence_caps": scoring["evidence_caps"],
        "overall": scoring["overall"],
        "verdict": scoring["verdict"],
        "flags": scoring["flags"],
        "outputs": {
            "overlay_image": str(overlay_path),
            "debug_json": str(debug_path),
        },
        "explanation": explanation,
    }


def write_blocked_report(output_dir: Path, preflight: dict[str, Any], rejected: list[dict[str, Any]]) -> None:
    (output_dir / "image_preflight.json").write_text(json.dumps({**preflight, "rejected_candidates": rejected}, indent=2), encoding="utf-8")
    report = {
        "image_source": {
            "url": preflight.get("selected_image_url", ""),
            "license": preflight.get("license", ""),
            "selection_reason": preflight.get("selection_note", "rejected"),
            "preflight_passed": False,
            "sha256": preflight.get("sha256", ""),
            "file_name": preflight.get("file_name", ""),
            "candidate_id": preflight.get("candidate_id", ""),
        },
        "pipeline": {
            "floor_source": "missing",
            "pose_source": "missing",
            "tracker_space_source": "missing",
            "used_runtime_paths": False,
        },
        "preflight": preflight.get("metrics", {}),
        "raw_scores": {
            "floor_plane_plausibility": 0.0,
            "pattern_alignment": 0.0,
            "body_outline_plausibility": 0.0,
            "tracker_position_plausibility": 0.0,
        },
        "capped_scores": {
            "floor_plane_plausibility": 0.0,
            "pattern_alignment": 0.0,
            "body_outline_plausibility": 0.0,
            "tracker_position_plausibility": 0.0,
        },
        "raw_overall": 0.0,
        "body_checks": {"left_leg": {}, "right_leg": {}},
        "evidence_caps": {"pose_cap": 0.0, "tracker_cap": 0.0, "floor_cap": 0.25, "body_cap": 0.0, "applied_cap": 0.0},
        "overall": 0.0,
        "verdict": "fail",
        "flags": list(preflight.get("reasons", [])),
        "explanation": "No image passed strict repeated-floor preflight; visual eval did not proceed.",
    }
    (output_dir / "visual_eval_report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description="Strict visual sanity eval for bodytracker floor/body/tracker wiring.")
    parser.add_argument("--output-dir", type=Path, default=Path("generated/visual_eval"))
    parser.add_argument("--bodytracker-exe", help="Path to bodytracker.exe; defaults to build/debug/Debug/bodytracker.exe")
    parser.add_argument("--candidate", choices=sorted(CANDIDATES), help="Run one named candidate instead of selector order")
    args = parser.parse_args()

    root = repo_root()
    output_dir = (root / args.output_dir).resolve() if not args.output_dir.is_absolute() else args.output_dir
    if output_dir.exists():
        shutil.rmtree(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    exe = find_bodytracker_exe(root, args.bodytracker_exe)

    candidate, image_path, floor_report, preflight, rejected, used_runtime_paths = select_candidate(root, output_dir, exe, args.candidate)
    preflight_doc = {**preflight, "rejected_candidates": rejected}
    (output_dir / "image_preflight.json").write_text(json.dumps(preflight_doc, indent=2), encoding="utf-8")
    if not preflight.get("accepted"):
        write_blocked_report(output_dir, preflight, rejected)
        print(json.dumps({"accepted": False, "preflight": preflight}, indent=2))
        return 2

    img = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if img is None:
        raise ValueError(f"could not read image: {image_path}")
    h, w = img.shape[:2]
    kp = keypoints_px(candidate, w, h)
    boxes = boxes_px(candidate, w, h)
    trackers = tracker_positions(kp)
    checks = body_checks(kp, boxes)
    scoring = compute_scores(
        preflight,
        floor_report,
        checks,
        trackers,
        boxes,
        pose_source="manual_keypoints",
        tracker_space_source="synthetic",
    )
    debug_path = output_dir / "visual_eval_debug.json"
    debug_path.write_text(json.dumps(floor_report, indent=2), encoding="utf-8")
    overlay_path = output_dir / "visual_eval_overlay.png"
    write_overlay(image_path, candidate, floor_report, preflight, scoring, overlay_path)
    report = build_report(candidate, preflight, floor_report, scoring, overlay_path, debug_path, used_runtime_paths, file_sha256(image_path))
    report_path = output_dir / "visual_eval_report.json"
    report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
