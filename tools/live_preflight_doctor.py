#!/usr/bin/env python3
"""Cheap live bring-up preflight checks for the FBT repo.

This intentionally does not build the app, load ONNX Runtime, open cameras, or
touch vcpkg. It verifies the files and config values an operator needs before a
real local Windows/vcpkg bring-up.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import sys
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional


EXPECTED_MODEL_REL = "models/rtmw-dw-x-l-cocktail14-384x288.onnx"
EXPECTED_MODEL_NAME = "RTMW-DW-X-L Cocktail14 384x288"
EXPECTED_DEPTH_MODEL_REL = "models/rtmw3d-x-cocktail14-384x288.onnx"
EXPECTED_DEPTH_MODEL_NAME = "RTMW3D-X Cocktail14 384x288 depth postprocess"
CONTRACT_REFERENCE_FILES = (
    "README.md",
    "docs/RUNTIME_BRINGUP.md",
    "src/ui/app/index.html",
    "models/README.md",
)
DEFAULT_CONFIG_REL = "config/default.json"
DEFAULT_CALIBRATION_REL = "calib/default.json"

SEVERITY_ORDER = ("FAIL", "WARN", "PASS", "INFO")
OPERATOR_FACING_FILES = (
    "README.md",
    "docs/BUILD_ENVIRONMENT.md",
    "docs/RUNTIME_BRINGUP.md",
    "AGENTS.md",
    "docs/BUREAUCRAT_LOGIC.md",
    "docs/SYNTHETIC_STEREO_DIAGNOSTICS.md",
    "models/README.md",
    "config/default.json",
    "src/ui/app/index.html",
    "src/ui/app/app.js",
    "src/main.cpp",
)
STALE_RTMPOSE_L_RE = re.compile(r"\brtmpose[\s_-]*l\b", re.IGNORECASE)


def _rel(path: Path, root: Path) -> str:
    try:
        return path.relative_to(root).as_posix()
    except ValueError:
        return path.as_posix()


def _result(severity: str, check_id: str, title: str, detail: str, action: str) -> Dict[str, str]:
    return {
        "severity": severity,
        "id": check_id,
        "title": title,
        "detail": detail,
        "action": action or "No action.",
    }


def _load_json(path: Path) -> tuple[Optional[Any], Optional[str]]:
    try:
        return json.loads(path.read_text(encoding="utf-8")), None
    except FileNotFoundError:
        return None, f"{path} does not exist"
    except json.JSONDecodeError as exc:
        return None, f"{path}:{exc.lineno}:{exc.colno}: {exc.msg}"
    except OSError as exc:
        return None, str(exc)


def _nested(config: Mapping[str, Any], keys: Iterable[str], default: Any = None) -> Any:
    value: Any = config
    for key in keys:
        if not isinstance(value, Mapping) or key not in value:
            return default
        value = value[key]
    return value


VALID_TRACKING_MODES = ("stereo", "monocular")


def _as_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _floor_reference_usable(
    image_height: float,
    spacing_px: float,
    reference_y_px: float,
    reference_depth_m: float,
) -> bool:
    principal_y = 0.5 * max(1.0, image_height)
    min_delta_px = 8.0
    if reference_y_px <= principal_y + min_delta_px:
        return False
    if reference_depth_m > 0.0:
        return True
    return spacing_px > 0.0 and reference_y_px - spacing_px > principal_y + min_delta_px


def _tracking_mode_from_config(config: Mapping[str, Any]) -> tuple[str, Optional[str]]:
    raw_mode = _nested(config, ("tracking", "mode"), "stereo")
    if not isinstance(raw_mode, str):
        return "stereo", f"tracking.mode must be 'stereo' or 'monocular', not {type(raw_mode).__name__}."
    mode = raw_mode.strip().lower()
    if mode in VALID_TRACKING_MODES:
        return mode, None
    return "stereo", f"tracking.mode={raw_mode!r}; expected one of {', '.join(VALID_TRACKING_MODES)}."


def _tracking_mode_for_report(repo_root: Path) -> str:
    config, error = _load_json(repo_root / DEFAULT_CONFIG_REL)
    if error or not isinstance(config, Mapping):
        return "unknown"
    mode, mode_error = _tracking_mode_from_config(config)
    return "invalid" if mode_error else mode


def _check_tracking_mode(config: Mapping[str, Any], results: List[Dict[str, str]]) -> str:
    mode, error = _tracking_mode_from_config(config)
    if error:
        results.append(_result(
            "FAIL",
            "tracking.mode",
            "tracking mode is invalid",
            error,
            "Set tracking.mode to either 'stereo' or 'monocular'.",
        ))
        return mode

    label = mode.upper()
    results.append(_result(
        "PASS",
        "tracking.mode",
        f"{label} tracking mode selected",
        f"tracking.mode={mode!r}.",
        f"Using {label}-specific preflight checks.",
    ))
    return mode


def _read_expected_sha(sidecar: Path) -> tuple[Optional[str], Optional[str]]:
    try:
        text = sidecar.read_text(encoding="utf-8").strip()
    except FileNotFoundError:
        return None, f"{sidecar} does not exist"
    except OSError as exc:
        return None, str(exc)
    match = re.search(r"\b([0-9a-fA-F]{64})\b", text)
    if not match:
        return None, f"{sidecar} does not contain a SHA-256 digest"
    return match.group(1).lower(), None


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _check_config(repo_root: Path, results: List[Dict[str, str]]) -> Optional[Dict[str, Any]]:
    config_path = repo_root / DEFAULT_CONFIG_REL
    config, error = _load_json(config_path)
    if error:
        results.append(_result(
            "FAIL",
            "config.default_json",
            "config/default.json does not parse",
            error,
            "Fix config/default.json before launching the runtime.",
        ))
        return None

    if not isinstance(config, dict):
        results.append(_result(
            "FAIL",
            "config.default_json",
            "config/default.json is not a JSON object",
            f"{DEFAULT_CONFIG_REL} parsed as {type(config).__name__}",
            "Replace it with the repo default config object.",
        ))
        return None

    results.append(_result(
        "PASS",
        "config.default_json",
        "config/default.json parses",
        f"{DEFAULT_CONFIG_REL} loaded successfully.",
        "No action.",
    ))
    return config


def _check_model(repo_root: Path, config: Mapping[str, Any], results: List[Dict[str, str]]) -> None:
    tracking_model = _nested(config, ("tracking", "model_path"), "")
    inference_model = _nested(config, ("inference", "model_path"), None)
    effective_model = tracking_model or inference_model or ""
    depth_enabled = bool(_nested(config, ("tracking", "depth_postprocess_enabled"), False))
    depth_model = _nested(config, ("tracking", "depth_postprocess_model_path"), "")
    depth_interval = _nested(config, ("tracking", "depth_postprocess_interval_frames"), 1)
    depth_allow_cpu = bool(_nested(config, ("tracking", "depth_postprocess_allow_cpu_fallback"), False))

    if effective_model != EXPECTED_MODEL_REL:
        results.append(_result(
            "FAIL",
            "model.config_path",
            "config model path does not match the primary Cocktail 2D tracking model",
            f"tracking.model_path={tracking_model!r}, legacy inference.model_path={inference_model!r}, expected {EXPECTED_MODEL_REL!r}.",
            "Set tracking.model_path to models/rtmw-dw-x-l-cocktail14-384x288.onnx unless you intentionally update the runtime contract.",
        ))
    else:
        legacy_note = " legacy inference.model_path is absent, as expected." if inference_model is None else f" legacy inference.model_path={inference_model!r} was ignored."
        results.append(_result(
            "PASS",
            "model.config_path",
            "config points at the primary Cocktail 2D tracking model",
            f"tracking.model_path uses {EXPECTED_MODEL_REL};{legacy_note}",
            "No action.",
        ))

    if not depth_enabled:
        results.append(_result(
            "PASS",
            "model.depth_postprocess_config",
            "live RTMW3D depth postprocess is disabled",
            f"tracking.depth_postprocess_enabled={depth_enabled!r}; calculated 3D from the Cocktail 2D solver remains active for live VRChat transfer.",
            "No action.",
        ))
    elif depth_model != EXPECTED_DEPTH_MODEL_REL:
        results.append(_result(
            "WARN",
            "model.depth_postprocess_config",
            "3D depth postprocess model is not enabled at the expected path",
            f"depth_postprocess_enabled={depth_enabled!r}, depth_postprocess_model_path={depth_model!r}, expected {EXPECTED_DEPTH_MODEL_REL!r}.",
            "Enable tracking.depth_postprocess_enabled and set tracking.depth_postprocess_model_path to the RTMW3D model for VRChat depth transfer.",
        ))
    else:
        results.append(_result(
            "PASS",
            "model.depth_postprocess_config",
            "3D depth postprocess model is configured",
            f"tracking.depth_postprocess_model_path uses {EXPECTED_DEPTH_MODEL_REL}; interval={depth_interval}; allow_cpu_fallback={depth_allow_cpu}.",
            "No action.",
        ))

    if depth_enabled and (not isinstance(depth_interval, int) or depth_interval < 1):
        results.append(_result(
            "FAIL",
            "model.depth_postprocess_interval",
            "3D depth postprocess interval is invalid",
            f"tracking.depth_postprocess_interval_frames={depth_interval!r}; expected integer >= 1.",
            "Set tracking.depth_postprocess_interval_frames to 4 for low-rate live depth assist, or 1 only when GPU can keep up.",
        ))
    elif depth_enabled:
        results.append(_result(
            "PASS",
            "model.depth_postprocess_interval",
            "3D depth postprocess runs at a bounded live rate",
            f"Runs once every {depth_interval} frame(s); CPU fallback allowed={depth_allow_cpu}.",
            "No action.",
        ))

    def check_asset(model_rel: str, check_prefix: str, label: str, required: bool = True) -> None:
        model_path = repo_root / model_rel
        sidecar_path = repo_root / f"{model_rel}.sha256"
        expected_sha, sidecar_error = _read_expected_sha(sidecar_path)
        if sidecar_error:
            results.append(_result(
                "WARN" if required else "INFO",
                f"{check_prefix}.sha_sidecar",
                f"{label} SHA sidecar is missing or invalid",
                sidecar_error,
                f"Restore {model_rel}.sha256 from the repo or document the expected SHA before bring-up." if required else "No action unless enabling optional RTMW3D postprocess.",
            ))
        else:
            results.append(_result(
                "PASS",
                f"{check_prefix}.sha_sidecar",
                f"{label} SHA sidecar is present",
                f"{_rel(sidecar_path, repo_root)} expects {expected_sha}.",
                "No action.",
            ))

        if not model_path.exists():
            results.append(_result(
                "FAIL" if required else "INFO",
                f"{check_prefix}.file",
                f"{label} file missing" if required else f"optional {label} file missing",
                f"Expected {_rel(model_path, repo_root)}.",
                f"Place the ONNX model at {model_rel}." if required else "No action; live VRChat transfer uses calculated 3D from Cocktail 2D.",
            ))
            return
        if not model_path.is_file():
            results.append(_result(
                "FAIL",
                f"{check_prefix}.file",
                f"{label} path is not a file",
                f"{_rel(model_path, repo_root)} exists but is not a regular file.",
                f"Replace it with the {label} ONNX file.",
            ))
            return

        size_mb = model_path.stat().st_size / (1024 * 1024)
        results.append(_result(
            "PASS",
            f"{check_prefix}.file",
            f"{label} file is present",
            f"{_rel(model_path, repo_root)} exists ({size_mb:.1f} MiB).",
            "No action.",
        ))

        if expected_sha:
            actual_sha = _sha256_file(model_path)
            if actual_sha != expected_sha:
                results.append(_result(
                    "FAIL",
                    f"{check_prefix}.sha256",
                    f"{label} SHA mismatch",
                    f"actual={actual_sha}, expected={expected_sha}.",
                    f"Replace the ONNX file with the expected {label} export, or update the sidecar only after intentionally changing the runtime contract.",
                ))
            else:
                results.append(_result(
                    "PASS",
                    f"{check_prefix}.sha256",
                    f"{label} SHA matches",
                    f"{_rel(model_path, repo_root)} matches {expected_sha}.",
                    "No action.",
                ))

    check_asset(EXPECTED_MODEL_REL, "model", EXPECTED_MODEL_NAME)
    check_asset(EXPECTED_DEPTH_MODEL_REL, "model.depth_postprocess", EXPECTED_DEPTH_MODEL_NAME, required=depth_enabled)


def _check_stereo_calibration(repo_root: Path, config: Mapping[str, Any], results: List[Dict[str, str]]) -> None:
    calibration_rel = _nested(config, ("tracking", "calibration_path"), DEFAULT_CALIBRATION_REL)
    calibration_path = repo_root / str(calibration_rel)
    if not calibration_path.exists():
        results.append(_result(
            "FAIL",
            "stereo.calibration.file",
            "STEREO calibration file missing",
            f"tracking.calibration_path resolves to {_rel(calibration_path, repo_root)}.",
            "Run the chessboard/intrinsic/stereo/floor/body calibration flow or set tracking.calibration_path to an existing calibration JSON.",
        ))
        return

    calibration, error = _load_json(calibration_path)
    if error or not isinstance(calibration, dict):
        results.append(_result(
            "FAIL",
            "stereo.calibration.json",
            "STEREO calibration file does not parse",
            error or f"{_rel(calibration_path, repo_root)} is not a JSON object.",
            "Fix or regenerate the stereo calibration JSON before live stereo bring-up.",
        ))
        return

    results.append(_result(
        "PASS",
        "stereo.calibration.file",
        "STEREO calibration file parses",
        f"{_rel(calibration_path, repo_root)} loaded successfully.",
        "No action.",
    ))

    if calibration.get("tracking_ready") is True:
        results.append(_result(
            "PASS",
            "stereo.calibration.tracking_ready",
            "STEREO calibration is marked tracking_ready",
            "tracking_ready=true.",
            "No action.",
        ))
    else:
        results.append(_result(
            "WARN",
            "stereo.calibration.tracking_ready",
            "STEREO calibration is not tracking_ready",
            f"tracking_ready={calibration.get('tracking_ready')!r}.",
            "Complete intrinsics, stereo, floor plane, and body calibration; then run bodytracker.exe --status calib\\default.json before enabling OSC.",
        ))

    camera_a = calibration.get("camera_a", {}) if isinstance(calibration.get("camera_a"), dict) else {}
    camera_b = calibration.get("camera_b", {}) if isinstance(calibration.get("camera_b"), dict) else {}
    floor = calibration.get("floor_plane", {}) if isinstance(calibration.get("floor_plane"), dict) else {}
    missing = []
    for label, section in (("camera_a", camera_a), ("camera_b", camera_b)):
        if not section.get("intrinsics_valid"):
            missing.append(f"{label}.intrinsics_valid")
        if not section.get("extrinsics_valid"):
            missing.append(f"{label}.extrinsics_valid")
    if not floor.get("valid"):
        missing.append("floor_plane.valid")
    if missing:
        results.append(_result(
            "WARN",
            "stereo.calibration.components",
            "STEREO calibration components are not all valid",
            ", ".join(missing),
            "Finish the missing stereo calibration steps before judging tracking quality.",
        ))
    else:
        results.append(_result(
            "PASS",
            "stereo.calibration.components",
            "STEREO camera and floor calibration flags are valid",
            "camera_a/camera_b intrinsics and extrinsics plus floor_plane.valid are true.",
            "No action.",
        ))


def _check_monocular_scale_profile(config: Mapping[str, Any], results: List[Dict[str, str]]) -> None:
    profile = _nested(config, ("tracking", "monocular"), {})
    if not isinstance(profile, Mapping):
        results.append(_result(
            "FAIL",
            "monocular.scale_profile",
            "MONOCULAR scale profile is missing",
            "tracking.monocular is not a JSON object.",
            "Restore tracking.monocular with user height, camera height, FOV, default depth, and optional floor-scale assist fields.",
        ))
        return

    image_height = _as_float(profile.get("image_height"))
    fov = _as_float(profile.get("horizontal_fov_deg"))
    user_height = _as_float(profile.get("user_height_m"))
    camera_height = _as_float(profile.get("camera_height_m"))
    default_depth = _as_float(profile.get("default_depth_m"))
    confidence_scale = _as_float(profile.get("depth_confidence_scale"))
    min_seed_count = int(_as_float(profile.get("min_seed_count")))

    hard_invalid = []
    if default_depth <= 0.0:
        hard_invalid.append("default_depth_m")
    if confidence_scale <= 0.0:
        hard_invalid.append("depth_confidence_scale")
    if min_seed_count <= 0:
        hard_invalid.append("min_seed_count")

    if hard_invalid:
        results.append(_result(
            "FAIL",
            "monocular.scale_profile",
            "MONOCULAR scale profile is not usable",
            ", ".join(hard_invalid),
            "Set positive monocular fallback depth, confidence scale, and seed-count values.",
        ))
    else:
        results.append(_result(
            "PASS",
            "monocular.scale_profile",
            "MONOCULAR scale profile is usable",
            f"default_depth_m={default_depth}, depth_confidence_scale={confidence_scale}, min_seed_count={min_seed_count}.",
            "No action.",
        ))

    if 30.0 <= fov <= 130.0:
        results.append(_result(
            "PASS",
            "monocular.horizontal_fov",
            "MONOCULAR horizontal FOV is configured",
            f"horizontal_fov_deg={fov}.",
            "No action.",
        ))
    else:
        results.append(_result(
            "WARN",
            "monocular.horizontal_fov",
            "MONOCULAR horizontal FOV is weak or missing",
            f"horizontal_fov_deg={profile.get('horizontal_fov_deg')!r}.",
            "Enter the camera horizontal FOV or choose a camera profile before judging monocular depth.",
        ))

    user_height_valid = 0.8 <= user_height <= 2.5
    camera_height_valid = 0.2 <= camera_height <= 3.0

    if user_height_valid:
        results.append(_result(
            "PASS",
            "monocular.user_height",
            "MONOCULAR user height scale is configured",
            f"user_height_m={user_height}.",
            "No action.",
        ))
    else:
        results.append(_result(
            "WARN",
            "monocular.user_height",
            "MONOCULAR user height scale is weak or missing",
            f"user_height_m={profile.get('user_height_m')!r}.",
            "Enter the user's real height in meters to improve monocular body scale.",
        ))

    if camera_height_valid:
        results.append(_result(
            "PASS",
            "monocular.camera_height",
            "MONOCULAR camera height scale is configured",
            f"camera_height_m={camera_height}.",
            "No action.",
        ))
    else:
        results.append(_result(
            "WARN",
            "monocular.camera_height",
            "MONOCULAR camera height scale is weak or missing",
            f"camera_height_m={profile.get('camera_height_m')!r}.",
            "Enter the camera height above the floor in meters to improve floor-ray depth.",
        ))

    floor_enabled = bool(profile.get("floor_scale_assist_enabled", False))
    floor_spacing_m = _as_float(profile.get("floor_depth_line_spacing_m"))
    floor_spacing_px = _as_float(profile.get("floor_depth_line_spacing_px"))
    floor_reference_y = _as_float(profile.get("floor_depth_reference_y_px"))
    floor_reference_m = _as_float(profile.get("floor_depth_reference_m"))
    floor_confidence = _as_float(profile.get("floor_depth_confidence"))
    floor_valid = (
        floor_enabled and
        floor_spacing_m > 0.0 and
        floor_spacing_px > 0.0 and
        floor_confidence > 0.0 and
        _floor_reference_usable(image_height, floor_spacing_px, floor_reference_y, floor_reference_m)
    )

    if floor_enabled and floor_valid:
        results.append(_result(
            "PASS",
            "monocular.floor_scale_assist",
            "MONOCULAR floor-scale assist is configured",
            f"floor_depth_line_spacing_m={floor_spacing_m}, floor_depth_line_spacing_px={floor_spacing_px}, floor_depth_reference_y_px={floor_reference_y}, floor_depth_reference_m={floor_reference_m}, floor_depth_confidence={floor_confidence}.",
            "No action.",
        ))
    elif floor_enabled:
        results.append(_result(
            "WARN",
            "monocular.floor_scale_assist",
            "MONOCULAR floor-scale assist is enabled but incomplete",
            f"floor_depth_line_spacing_m={profile.get('floor_depth_line_spacing_m')!r}, floor_depth_line_spacing_px={profile.get('floor_depth_line_spacing_px')!r}, floor_depth_reference_y_px={profile.get('floor_depth_reference_y_px')!r}, floor_depth_reference_m={profile.get('floor_depth_reference_m')!r}, floor_depth_confidence={profile.get('floor_depth_confidence')!r}.",
            "Enter real floor-depth tile spacing or plank board pitch plus matching image spacing and a reference row below the image horizon, or disable floor-scale assist.",
        ))
    else:
        results.append(_result(
            "INFO",
            "monocular.floor_scale_assist",
            "MONOCULAR floor-scale assist is disabled",
            "tracking.monocular.floor_scale_assist_enabled=false.",
            "Optional: enable it only when visible floor-depth seams, tile rows, or plank board pitch have known real spacing.",
        ))

    if user_height_valid or camera_height_valid or floor_valid:
        sources = []
        if user_height_valid:
            sources.append("user_height_m")
        if camera_height_valid:
            sources.append("camera_height_m")
        if floor_valid:
            sources.append("floor_scale_assist")
        results.append(_result(
            "PASS",
            "monocular.metric_scale_source",
            "MONOCULAR metric scale source is available",
            ", ".join(sources),
            "No action.",
        ))
    else:
        results.append(_result(
            "FAIL",
            "monocular.metric_scale_source",
            "MONOCULAR metric scale source is missing",
            "No valid user height, camera height, or floor-scale assist input was found.",
            "Provide at least one real-world scale input before using monocular tracking.",
        ))


def _check_stereo_monocular_fallback_profile(config: Mapping[str, Any], results: List[Dict[str, str]]) -> None:
    tracking = _nested(config, ("tracking",), {})
    enabled = bool(tracking.get("stereo_monocular_fallback_enabled", True)) if isinstance(tracking, Mapping) else True
    if not enabled:
        results.append(_result(
            "INFO",
            "stereo.monocular_fallback.profile",
            "STEREO Camera A fallback is disabled",
            "tracking.stereo_monocular_fallback_enabled=false.",
            "No action unless you want stereo mode to keep running when Camera B is unavailable.",
        ))
        return

    profile = _nested(config, ("tracking", "monocular"), {})
    if not isinstance(profile, Mapping):
        results.append(_result(
            "WARN",
            "stereo.monocular_fallback.profile",
            "STEREO Camera A fallback profile is missing",
            "tracking.stereo_monocular_fallback_enabled=true but tracking.monocular is not a JSON object.",
            "Restore tracking.monocular so stereo fallback can reuse the single-camera solver.",
        ))
        return

    image_width = _as_float(profile.get("image_width"))
    image_height = _as_float(profile.get("image_height"))
    fov = _as_float(profile.get("horizontal_fov_deg"))
    user_height = _as_float(profile.get("user_height_m"))
    camera_height = _as_float(profile.get("camera_height_m"))
    default_depth = _as_float(profile.get("default_depth_m"))
    floor_enabled = bool(profile.get("floor_scale_assist_enabled", False))
    floor_spacing_m = _as_float(profile.get("floor_depth_line_spacing_m"))
    floor_spacing_px = _as_float(profile.get("floor_depth_line_spacing_px"))
    floor_reference_y = _as_float(profile.get("floor_depth_reference_y_px"))
    floor_reference_m = _as_float(profile.get("floor_depth_reference_m"))
    floor_confidence = _as_float(profile.get("floor_depth_confidence"))
    floor_valid = (
        floor_enabled and
        floor_spacing_m > 0.0 and
        floor_spacing_px > 0.0 and
        floor_confidence > 0.0 and
        _floor_reference_usable(image_height, floor_spacing_px, floor_reference_y, floor_reference_m)
    )
    projection_valid = image_width > 0 and image_height > 0 and 30.0 <= fov <= 130.0 and default_depth > 0.0
    metric_scale_valid = user_height > 0.0 or camera_height > 0.0 or floor_valid

    if projection_valid and metric_scale_valid:
        results.append(_result(
            "PASS",
            "stereo.monocular_fallback.profile",
            "STEREO Camera A fallback profile is usable",
            f"fallback enabled; image={image_width:.0f}x{image_height:.0f}, fov={fov}, user_height_m={user_height}, camera_height_m={camera_height}.",
            "No action.",
        ))
    else:
        results.append(_result(
            "WARN",
            "stereo.monocular_fallback.profile",
            "STEREO Camera A fallback profile is weak",
            f"fallback enabled; projection_valid={projection_valid}, metric_scale_valid={metric_scale_valid}.",
            "Set tracking.monocular image size/FOV/default depth and at least one metric scale input, because stereo fallback uses the same single-camera features.",
        ))

    if floor_valid:
        results.append(_result(
            "PASS",
            "stereo.monocular_fallback.floor_scale_assist",
            "STEREO fallback floor-scale assist is configured",
            f"floor_depth_line_spacing_m={floor_spacing_m}, floor_depth_line_spacing_px={floor_spacing_px}, floor_depth_reference_y_px={floor_reference_y}, floor_depth_reference_m={floor_reference_m}, floor_depth_confidence={floor_confidence}.",
            "No action.",
        ))
    elif floor_enabled:
        results.append(_result(
            "WARN",
            "stereo.monocular_fallback.floor_scale_assist",
            "STEREO fallback floor-scale assist is incomplete",
            f"floor_depth_line_spacing_m={profile.get('floor_depth_line_spacing_m')!r}, floor_depth_line_spacing_px={profile.get('floor_depth_line_spacing_px')!r}, floor_depth_reference_y_px={profile.get('floor_depth_reference_y_px')!r}, floor_depth_reference_m={profile.get('floor_depth_reference_m')!r}, floor_depth_confidence={profile.get('floor_depth_confidence')!r}.",
            "Enter real floor-depth tile spacing or plank board pitch plus matching image spacing and a reference row below the image horizon, or disable floor-scale assist.",
        ))
    else:
        results.append(_result(
            "INFO",
            "stereo.monocular_fallback.floor_scale_assist",
            "STEREO fallback floor-scale assist is disabled",
            "tracking.monocular.floor_scale_assist_enabled=false.",
            "Optional: enable it only when visible floor-depth seams, tile rows, or plank board pitch have known real spacing.",
        ))


def _check_monocular_calibration_skip(config: Mapping[str, Any], results: List[Dict[str, str]]) -> None:
    calibration_rel = _nested(config, ("tracking", "calibration_path"), DEFAULT_CALIBRATION_REL)
    results.append(_result(
        "INFO",
        "monocular.stereo_calibration.skipped",
        "MONOCULAR mode does not require stereo calibration",
        f"tracking.calibration_path={calibration_rel!r} is not required for monocular preflight.",
        "No action unless you switch tracking.mode back to 'stereo'.",
    ))


def _check_replay_paths(repo_root: Path, config: Mapping[str, Any], results: List[Dict[str, str]]) -> None:
    recording_dir_rel = _nested(config, ("app", "recording_dir"), "recordings")
    recording_dir = repo_root / str(recording_dir_rel)
    replay_log_rel = _nested(config, ("debug", "replay_log_path"), "")
    replay_enabled = bool(_nested(config, ("tracking", "enable_replay_recording"), False))

    if recording_dir.exists() and recording_dir.is_dir():
        results.append(_result(
            "PASS",
            "replay.recording_dir",
            "recording directory exists",
            f"app.recording_dir={recording_dir_rel!r}.",
            "No action.",
        ))
    else:
        severity = "WARN" if replay_enabled else "INFO"
        action = (
            f"Create {recording_dir_rel} before recording, or confirm the runtime can create it on your machine."
            if replay_enabled else
            f"No action until replay recording is enabled; the runtime writes {recording_dir_rel}/latest-runtime.ndjson when recording."
        )
        results.append(_result(
            severity,
            "replay.recording_dir",
            "recording directory does not exist yet",
            f"app.recording_dir={recording_dir_rel!r}; enable_replay_recording={replay_enabled}.",
            action,
        ))

    if replay_log_rel:
        replay_path = repo_root / str(replay_log_rel)
        parent = replay_path.parent
        if parent.exists():
            results.append(_result(
                "PASS",
                "replay.explicit_path",
                "explicit replay log parent exists",
                f"debug.replay_log_path={replay_log_rel!r}.",
                "No action.",
            ))
        else:
            results.append(_result(
                "WARN",
                "replay.explicit_path",
                "explicit replay log parent is missing",
                f"debug.replay_log_path={replay_log_rel!r}.",
                f"Create {parent.as_posix()} or clear debug.replay_log_path to use app.recording_dir.",
            ))
    else:
        results.append(_result(
            "INFO",
            "replay.explicit_path",
            "debug.replay_log_path is empty",
            "Runtime replay recording will use app.recording_dir/latest-runtime.ndjson when enabled.",
            "Set debug.replay_log_path only if you want an explicit file path.",
        ))


def _check_vcpkg(repo_root: Path, env: Mapping[str, str], results: List[Dict[str, str]]) -> None:
    vcpkg_json = repo_root / "vcpkg.json"
    if vcpkg_json.exists():
        results.append(_result(
            "PASS",
            "vcpkg.manifest",
            "vcpkg manifest exists",
            "vcpkg.json is present.",
            "No action.",
        ))
    else:
        results.append(_result(
            "FAIL",
            "vcpkg.manifest",
            "vcpkg manifest missing",
            "vcpkg.json is missing.",
            "Restore vcpkg.json before attempting the intended Windows/vcpkg build.",
        ))

    vcpkg_root = env.get("VCPKG_ROOT", "")
    if not vcpkg_root:
        results.append(_result(
            "WARN",
            "vcpkg.root",
            "VCPKG_ROOT is not set",
            "The doctor does not install dependencies, but the intended Windows build expects VCPKG_ROOT.",
            "Set VCPKG_ROOT to your vcpkg checkout before configuring on the live bring-up machine.",
        ))
        return

    vcpkg_path = Path(vcpkg_root)
    if vcpkg_path.exists():
        results.append(_result(
            "PASS",
            "vcpkg.root",
            "VCPKG_ROOT exists",
            f"VCPKG_ROOT={vcpkg_root}.",
            "No action.",
        ))
    else:
        results.append(_result(
            "WARN",
            "vcpkg.root",
            "VCPKG_ROOT path does not exist",
            f"VCPKG_ROOT={vcpkg_root}.",
            "Fix VCPKG_ROOT before configuring the Windows/vcpkg build.",
        ))


def _camera_mode_invalid(label: str, section: Mapping[str, Any]) -> List[str]:
    invalid = []
    for field in ("width", "height", "fps"):
        if _as_float(section.get(field)) <= 0.0:
            invalid.append(f"{label}.{field}")
    return invalid


def _check_stereo_cameras(config: Mapping[str, Any], results: List[Dict[str, str]]) -> None:
    cams = []
    for key in ("camera_a", "camera_b"):
        section = _nested(config, (key,), {})
        if not isinstance(section, Mapping):
            results.append(_result(
                "FAIL",
                f"stereo.{key}.config",
                f"STEREO {key} config is missing",
                f"{key} is not a JSON object.",
                "Restore camera_a and camera_b sections in config/default.json for stereo mode.",
            ))
            return
        cams.append(section)

    a_index = cams[0].get("device_index")
    b_index = cams[1].get("device_index")
    detail = (
        f"camera_a index={a_index}, {cams[0].get('width')}x{cams[0].get('height')}@{cams[0].get('fps')}; "
        f"camera_b index={b_index}, {cams[1].get('width')}x{cams[1].get('height')}@{cams[1].get('fps')}."
    )

    if a_index == b_index:
        results.append(_result(
            "FAIL",
            "stereo.camera.indices",
            "STEREO camera A and B use the same device index",
            detail,
            "Choose two different camera indices in the desktop setup UI or config/default.json.",
        ))
    else:
        results.append(_result(
            "PASS",
            "stereo.camera.indices",
            "STEREO camera A and B use different indices",
            detail,
            "No action.",
        ))

    invalid: List[str] = []
    invalid.extend(_camera_mode_invalid("camera_a", cams[0]))
    invalid.extend(_camera_mode_invalid("camera_b", cams[1]))
    if invalid:
        results.append(_result(
            "FAIL",
            "stereo.camera.modes",
            "STEREO camera resolution/FPS values are invalid",
            ", ".join(invalid),
            "Set positive width, height, and fps values for both stereo cameras.",
        ))
    else:
        results.append(_result(
            "PASS",
            "stereo.camera.modes",
            "STEREO camera resolution/FPS values are configured",
            detail,
            "No action.",
        ))


def _check_monocular_camera(config: Mapping[str, Any], results: List[Dict[str, str]]) -> None:
    camera_a = _nested(config, ("camera_a",), {})
    if not isinstance(camera_a, Mapping):
        results.append(_result(
            "FAIL",
            "monocular.camera_a.config",
            "MONOCULAR Camera A config is missing",
            "camera_a is not a JSON object.",
            "Restore camera_a in config/default.json; monocular mode needs only Camera A.",
        ))
        return

    detail = f"camera_a index={camera_a.get('device_index')}, {camera_a.get('width')}x{camera_a.get('height')}@{camera_a.get('fps')}."
    invalid = _camera_mode_invalid("camera_a", camera_a)
    if invalid:
        results.append(_result(
            "FAIL",
            "monocular.camera_a.mode",
            "MONOCULAR Camera A resolution/FPS values are invalid",
            ", ".join(invalid),
            "Set positive width, height, and fps values for Camera A.",
        ))
    else:
        results.append(_result(
            "PASS",
            "monocular.camera_a.mode",
            "MONOCULAR Camera A resolution/FPS values are configured",
            detail,
            "No action.",
        ))

    camera_b = _nested(config, ("camera_b",), None)
    if isinstance(camera_b, Mapping):
        results.append(_result(
            "INFO",
            "monocular.camera_b.ignored",
            "MONOCULAR mode ignores Camera B",
            f"camera_b index={camera_b.get('device_index')} is present but not required.",
            "No action; Camera B is only checked when tracking.mode='stereo'.",
        ))
    else:
        results.append(_result(
            "INFO",
            "monocular.camera_b.ignored",
            "MONOCULAR mode does not require Camera B",
            "camera_b is absent or not configured.",
            "No action; Camera B is only checked when tracking.mode='stereo'.",
        ))


def _check_stale_references(repo_root: Path, results: List[Dict[str, str]]) -> None:
    hits = []
    for rel in OPERATOR_FACING_FILES:
        path = repo_root / rel
        if not path.exists() or not path.is_file():
            continue
        try:
            text = _read_text(path)
        except OSError:
            continue
        for match in STALE_RTMPOSE_L_RE.finditer(text):
            line = text.count("\n", 0, match.start()) + 1
            hits.append(f"{rel}:{line}:{match.group(0)}")
    if hits:
        results.append(_result(
            "FAIL",
            "docs.stale_rtmpose_l",
            "stale RTMPose-L reference found in operator-facing text",
            "; ".join(hits[:8]) + ("; ..." if len(hits) > 8 else ""),
            f"Replace stale RTMPose-L text with {EXPECTED_MODEL_NAME} and keep model path {EXPECTED_MODEL_REL}.",
        ))
    else:
        results.append(_result(
            "PASS",
            "docs.stale_rtmpose_l",
            "no stale RTMPose-L operator-facing references found",
            f"Scanned {len(OPERATOR_FACING_FILES)} expected docs/config/UI files.",
            "No action.",
        ))


def _read_text(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8")
    except UnicodeDecodeError:
        return path.read_text(encoding="utf-8", errors="ignore")


def _check_operator_contract_references(repo_root: Path, expected_sha: Optional[str], results: List[Dict[str, str]]) -> None:
    missing_path = []
    missing_sha = []
    for rel in CONTRACT_REFERENCE_FILES:
        path = repo_root / rel
        if not path.exists() or not path.is_file():
            missing_path.append(f"{rel}:missing")
            if expected_sha:
                missing_sha.append(f"{rel}:missing")
            continue
        text = _read_text(path)
        if EXPECTED_MODEL_REL not in text:
            missing_path.append(rel)
        if expected_sha and expected_sha not in text:
            missing_sha.append(rel)

    if missing_path:
        results.append(_result(
            "FAIL",
            "docs.model_path_reference",
            "operator-facing model path reference is stale or missing",
            "; ".join(missing_path),
            f"Update bring-up docs/UI text to name {EXPECTED_MODEL_REL}.",
        ))
    else:
        results.append(_result(
            "PASS",
            "docs.model_path_reference",
            "operator-facing model path references are current",
            f"Checked {len(CONTRACT_REFERENCE_FILES)} contract reference files.",
            "No action.",
        ))

    if not expected_sha:
        results.append(_result(
            "INFO",
            "docs.model_sha_reference",
            "operator-facing SHA references skipped",
            "No valid model SHA sidecar was available.",
            f"Restore {EXPECTED_MODEL_REL}.sha256, then rerun the doctor.",
        ))
    elif missing_sha:
        results.append(_result(
            "FAIL",
            "docs.model_sha_reference",
            "operator-facing model SHA reference is stale or missing",
            "; ".join(missing_sha),
            "Update bring-up docs/UI text to match the SHA sidecar.",
        ))
    else:
        results.append(_result(
            "PASS",
            "docs.model_sha_reference",
            "operator-facing model SHA references are current",
            f"Checked SHA {expected_sha} in {len(CONTRACT_REFERENCE_FILES)} contract reference files.",
            "No action.",
        ))


def _check_debug_paths(config: Mapping[str, Any], results: List[Dict[str, str]]) -> None:
    debug = _nested(config, ("debug",), {})
    if not isinstance(debug, Mapping):
        debug = {}

    replay_path = debug.get("replay_log_path", "")
    results.append(_result(
        "INFO",
        "debug.replay_log_path",
        "debug replay path checked",
        f"debug.replay_log_path={replay_path!r}.",
        "Leave empty for app.recording_dir/latest-runtime.ndjson, or set an explicit file before recording a repro clip.",
    ))


def run_checks(repo_root: Path, env: Optional[Mapping[str, str]] = None) -> List[Dict[str, str]]:
    repo_root = repo_root.resolve()
    env = env if env is not None else os.environ
    results: List[Dict[str, str]] = []

    config = _check_config(repo_root, results)
    if config is not None:
        mode = _check_tracking_mode(config, results)
        _check_model(repo_root, config, results)
        if mode == "monocular":
            _check_monocular_scale_profile(config, results)
            _check_monocular_calibration_skip(config, results)
            _check_monocular_camera(config, results)
        else:
            _check_stereo_calibration(repo_root, config, results)
            _check_stereo_cameras(config, results)
            _check_stereo_monocular_fallback_profile(config, results)
        _check_replay_paths(repo_root, config, results)
        _check_vcpkg(repo_root, env, results)
        _check_debug_paths(config, results)
    else:
        _check_vcpkg(repo_root, env, results)

    expected_sha, _ = _read_expected_sha(repo_root / f"{EXPECTED_MODEL_REL}.sha256")
    _check_stale_references(repo_root, results)
    _check_operator_contract_references(repo_root, expected_sha, results)
    return results


def build_report(repo_root: Path, env: Optional[Mapping[str, str]] = None) -> Dict[str, Any]:
    results = run_checks(repo_root, env=env)
    counts = {severity.lower(): sum(1 for r in results if r["severity"] == severity) for severity in SEVERITY_ORDER}
    return {
        "schema_version": 1,
        "tool": "tools/live_preflight_doctor.py",
        "repo_root": str(repo_root.resolve()),
        "expected_model": EXPECTED_MODEL_REL,
        "expected_depth_postprocess_model": EXPECTED_DEPTH_MODEL_REL,
        "tracking_mode": _tracking_mode_for_report(repo_root.resolve()),
        "summary": {
            **counts,
            "ready": counts["fail"] == 0,
            "result_count": len(results),
        },
        "results": results,
    }


def format_report(report: Mapping[str, Any]) -> str:
    summary = report["summary"]
    lines = [
        "FBT live preflight doctor",
        f"repo: {report['repo_root']}",
        f"expected model: {report['expected_model']}",
        f"expected depth postprocess model: {report['expected_depth_postprocess_model']}",
        f"tracking mode: {report.get('tracking_mode', 'unknown')}",
        f"summary: {summary['fail']} FAIL / {summary['warn']} WARN / {summary['pass']} PASS / {summary['info']} INFO",
        "",
    ]
    results = list(report["results"])
    for severity in SEVERITY_ORDER:
        bucket = [r for r in results if r["severity"] == severity]
        if not bucket:
            continue
        lines.append(f"{severity}")
        for item in bucket:
            lines.append(f"  [{item['id']}] {item['title']}")
            lines.append(f"    {item['detail']}")
            lines.append(f"    ACTION {item['action']}")
        lines.append("")
    if summary["ready"]:
        lines.append("NEXT ACTION launch locally and verify model/camera mode-specific live cards before enabling OSC.")
    else:
        lines.append("NEXT ACTION fix FAIL items first; WARN items can be handled during bring-up but should not be ignored.")
    return "\n".join(lines)


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Cheap FBT live bring-up preflight doctor.")
    parser.add_argument("--repo-root", default=str(Path(__file__).resolve().parents[1]), help="Repository root to inspect.")
    parser.add_argument("--json", action="store_true", help="Emit machine-readable JSON.")
    args = parser.parse_args(argv)

    report = build_report(Path(args.repo_root))
    if args.json:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        print(format_report(report))
    return 1 if report["summary"]["fail"] else 0


if __name__ == "__main__":
    raise SystemExit(main())

