#!/usr/bin/env python3
"""Generate deterministic low-resolution synthetic stereo diagnostics for lower-body tracking.

The generator is intentionally dependency-free. It creates a JSONL trace with ground-truth
lower-body pose/contact state, per-camera projected HALPE-26 keypoints, confidence,
visibility/occlusion/noise annotations, a chessboard reference, and SVG camera previews.

This is a diagnostic instrument, not a renderer. The output is designed to expose where the
runtime pipeline needs better blame data: free-foot underprediction, root correction inheriting
foot noise, support flicker, toe/heel constraint fights, and stereo disagreement.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import random
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Dict, Iterable, List, Mapping, MutableMapping, Optional, Sequence, Tuple


SCHEMA_VERSION = 3
DEFAULT_WIDTH = 384
DEFAULT_HEIGHT = 288
DEFAULT_FPS = 30.0

KEYPOINT_NAMES = [
    "nose",
    "left_eye",
    "right_eye",
    "left_ear",
    "right_ear",
    "left_shoulder",
    "right_shoulder",
    "left_elbow",
    "right_elbow",
    "left_wrist",
    "right_wrist",
    "left_hip",
    "right_hip",
    "left_knee",
    "right_knee",
    "left_ankle",
    "right_ankle",
    "head",
    "neck",
    "hip",
    "left_big_toe",
    "right_big_toe",
    "left_small_toe",
    "right_small_toe",
    "left_heel",
    "right_heel",
]

LOWER_BODY_NAMES = {
    "left_hip",
    "right_hip",
    "left_knee",
    "right_knee",
    "left_ankle",
    "right_ankle",
    "hip",
    "left_big_toe",
    "right_big_toe",
    "left_small_toe",
    "right_small_toe",
    "left_heel",
    "right_heel",
}

FOOT_KEYPOINTS = {
    "left": {"left_ankle", "left_big_toe", "left_small_toe", "left_heel"},
    "right": {"right_ankle", "right_big_toe", "right_small_toe", "right_heel"},
}

SKELETON_EDGES = [
    ("hip", "left_hip"),
    ("hip", "right_hip"),
    ("left_hip", "left_knee"),
    ("left_knee", "left_ankle"),
    ("left_ankle", "left_heel"),
    ("left_ankle", "left_big_toe"),
    ("left_big_toe", "left_small_toe"),
    ("right_hip", "right_knee"),
    ("right_knee", "right_ankle"),
    ("right_ankle", "right_heel"),
    ("right_ankle", "right_big_toe"),
    ("right_big_toe", "right_small_toe"),
    ("hip", "neck"),
    ("neck", "head"),
]

SCENARIO_ORDER = [
    "pure_airborne_leg_swing",
    "body_over_planted_stance",
    "planted_foot_jitter_from_2d_noise",
    "heel_lock",
    "toe_pivot",
    "flat_plant",
    "slip_release",
    "support_transition_griddy",
    "one_camera_foot_occlusion",
    "both_camera_foot_occlusion",
    "two_camera_disagreement",
    "mild_calibration_imperfection",
    "low_res_heel_toe_ankle_ambiguity",
]

SCENARIO_DESCRIPTIONS = {
    "pure_airborne_leg_swing": "Right foot swings through the air while the left foot remains flat planted.",
    "body_over_planted_stance": "Pelvis translates over a left stance foot without common-mode foot motion.",
    "planted_foot_jitter_from_2d_noise": "Both feet are planted but low-res 2D detections jitter at the feet.",
    "heel_lock": "Left heel is anchored while the toe lifts, stressing heel-lock support.",
    "toe_pivot": "Left toe is anchored while the heel lifts, stressing toe-pivot support.",
    "flat_plant": "Both feet remain flat with clean high-confidence detections.",
    "slip_release": "Left foot transitions from flat plant to release/slip and should not snap the body back.",
    "support_transition_griddy": "Alternating quick support transitions stress support hysteresis and griddy-like steps.",
    "one_camera_foot_occlusion": "Right foot is hidden from camera B only, stressing triangulation fallback.",
    "both_camera_foot_occlusion": "Right foot is hidden from both cameras, stressing prediction/HMD fallback.",
    "two_camera_disagreement": "Camera B reports a coherent but wrong right-foot position.",
    "mild_calibration_imperfection": "Camera B uses a mildly perturbed actual calibration while nominal metadata remains ideal.",
    "low_res_heel_toe_ankle_ambiguity": "Heel/toe/ankle land within only a few pixels and carry reduced confidence.",
}

SCENARIO_RISKS = {
    "pure_airborne_leg_swing": [
        "free_foot_underprediction",
        "airborne_swing_damped_by_final_correction",
    ],
    "body_over_planted_stance": [
        "body_over_stance_wrongly_rejected",
        "root_correction_inherits_foot_noise",
    ],
    "planted_foot_jitter_from_2d_noise": [
        "single_foot_jitter_pushes_root",
        "support_classification_flicker",
    ],
    "heel_lock": [
        "heel_anchor_residual_not_exported",
        "heel_lock_fights_generic_foot_anchor",
    ],
    "toe_pivot": [
        "toe_anchor_residual_not_exported",
        "toe_pivot_fights_generic_foot_anchor",
    ],
    "flat_plant": [
        "baseline_contact_residual_drift",
    ],
    "slip_release": [
        "release_pending_snapback",
        "slip_misclassified_as_planted",
    ],
    "support_transition_griddy": [
        "support_transition_snap",
        "griddy_like_free_foot_damping",
    ],
    "one_camera_foot_occlusion": [
        "single_view_foot_overconfidence",
        "triangulation_fallback_unobservable",
    ],
    "both_camera_foot_occlusion": [
        "occlusion_prediction_decay",
        "hmd_fallback_hides_foot_loss",
    ],
    "two_camera_disagreement": [
        "camera_disagreement_confidence_not_propagated",
        "bad_camera_common_mode_root_pull",
    ],
    "mild_calibration_imperfection": [
        "calibration_bias_mistaken_for_contact_slip",
    ],
    "low_res_heel_toe_ankle_ambiguity": [
        "heel_toe_phase_flicker",
        "ankle_toe_swap_unobservable",
    ],
}


@dataclass(frozen=True)
class Vec3:
    x: float
    y: float
    z: float

    def add(self, other: "Vec3") -> "Vec3":
        return Vec3(self.x + other.x, self.y + other.y, self.z + other.z)

    def sub(self, other: "Vec3") -> "Vec3":
        return Vec3(self.x - other.x, self.y - other.y, self.z - other.z)

    def scale(self, value: float) -> "Vec3":
        return Vec3(self.x * value, self.y * value, self.z * value)

    def as_list(self) -> List[float]:
        return [round(self.x, 6), round(self.y, 6), round(self.z, 6)]


@dataclass(frozen=True)
class FootPose:
    center: Vec3
    pitch_rad: float = 0.0

    def as_pose_json(self) -> Dict[str, object]:
        return {
            "position": self.center.as_list(),
            "orientation": pitch_quat(self.pitch_rad),
            "pitch_rad": round(self.pitch_rad, 6),
        }


@dataclass(frozen=True)
class Camera:
    name: str
    camera_id: str
    x: float
    y: float
    z: float
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int

    def with_delta(self, *, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0,
                   dfx: float = 0.0, dfy: float = 0.0) -> "Camera":
        return replace(
            self,
            x=self.x + dx,
            y=self.y + dy,
            z=self.z + dz,
            fx=self.fx + dfx,
            fy=self.fy + dfy,
        )

    def to_json(self) -> Dict[str, object]:
        return {
            "id": self.camera_id,
            "position": [round(self.x, 6), round(self.y, 6), round(self.z, 6)],
            "intrinsics": {
                "fx": round(self.fx, 6),
                "fy": round(self.fy, 6),
                "cx": round(self.cx, 6),
                "cy": round(self.cy, 6),
                "width": self.width,
                "height": self.height,
            },
            "model": "simple_pinhole_xz_forward_y_down",
        }


@dataclass(frozen=True)
class ScenarioFrame:
    scenario: str
    scenario_frame_index: int
    t: float
    root: Vec3
    left_foot: FootPose
    right_foot: FootPose
    left_phase: str
    right_phase: str
    left_support_confidence: float
    right_support_confidence: float
    left_contact_type: str = "FLOOR_SUPPORT"
    right_contact_type: str = "FLOOR_SUPPORT"
    left_planted: bool = True
    right_planted: bool = True
    active_free_foot: Optional[str] = None
    stance_foot: Optional[str] = None
    noise_px: float = 1.0
    foot_noise_px: float = 1.5
    left_foot_confidence: float = 0.88
    right_foot_confidence: float = 0.88
    camera_b_right_foot_offset_px: Tuple[float, float] = (0.0, 0.0)
    low_res_foot_ambiguity: bool = False
    mild_calibration_imperfection: bool = False
    occlude_camera_a_foot: Optional[str] = None
    occlude_camera_b_foot: Optional[str] = None
    occlusion_reason: str = ""


def clamp01(value: float) -> float:
    return max(0.0, min(1.0, value))


def smoothstep(t: float) -> float:
    t = clamp01(t)
    return t * t * (3.0 - 2.0 * t)


def sine01(t: float) -> float:
    return 0.5 - 0.5 * math.cos(2.0 * math.pi * t)


def pitch_quat(pitch_rad: float) -> List[float]:
    half = 0.5 * pitch_rad
    return [round(math.sin(half), 6), 0.0, 0.0, round(math.cos(half), 6)]


def rotate_pitch(v: Vec3, pitch_rad: float) -> Vec3:
    c = math.cos(pitch_rad)
    s = math.sin(pitch_rad)
    return Vec3(v.x, c * v.y - s * v.z, s * v.y + c * v.z)


def foot_contact_point(foot: FootPose, forward_offset_m: float) -> Vec3:
    return foot.center.add(rotate_pitch(Vec3(0.0, 0.0, forward_offset_m), foot.pitch_rad))


def foot_from_toe_anchor(toe_anchor: Vec3, pitch_rad: float) -> FootPose:
    return FootPose(toe_anchor.sub(rotate_pitch(Vec3(0.0, 0.0, 0.125), pitch_rad)), pitch_rad)


def foot_from_heel_anchor(heel_anchor: Vec3, pitch_rad: float) -> FootPose:
    return FootPose(heel_anchor.sub(rotate_pitch(Vec3(0.0, 0.0, -0.095), pitch_rad)), pitch_rad)


def interpolate(a: Vec3, b: Vec3, t: float) -> Vec3:
    return Vec3(
        a.x + (b.x - a.x) * t,
        a.y + (b.y - a.y) * t,
        a.z + (b.z - a.z) * t,
    )


def lower_body_joints(root: Vec3, left_foot: FootPose, right_foot: FootPose) -> Dict[str, Vec3]:
    pelvis_width = 0.32
    shoulder_width = 0.45
    left_hip = root.add(Vec3(-0.5 * pelvis_width, 0.0, 0.0))
    right_hip = root.add(Vec3(0.5 * pelvis_width, 0.0, 0.0))

    left_ankle = left_foot.center.add(rotate_pitch(Vec3(0.0, 0.075, 0.01), left_foot.pitch_rad))
    right_ankle = right_foot.center.add(rotate_pitch(Vec3(0.0, 0.075, 0.01), right_foot.pitch_rad))
    left_knee = interpolate(left_hip, left_ankle, 0.52).add(Vec3(-0.02, 0.03, -0.055))
    right_knee = interpolate(right_hip, right_ankle, 0.52).add(Vec3(0.02, 0.03, -0.055))

    left_toe = foot_contact_point(left_foot, 0.125)
    right_toe = foot_contact_point(right_foot, 0.125)
    left_heel = foot_contact_point(left_foot, -0.095)
    right_heel = foot_contact_point(right_foot, -0.095)

    neck = root.add(Vec3(0.0, 0.54, -0.015))
    head = root.add(Vec3(0.0, 0.73, -0.02))
    nose = head.add(Vec3(0.0, 0.0, 0.09))
    left_shoulder = neck.add(Vec3(-0.5 * shoulder_width, -0.03, 0.0))
    right_shoulder = neck.add(Vec3(0.5 * shoulder_width, -0.03, 0.0))

    joints = {
        "nose": nose,
        "left_eye": head.add(Vec3(-0.035, 0.025, 0.06)),
        "right_eye": head.add(Vec3(0.035, 0.025, 0.06)),
        "left_ear": head.add(Vec3(-0.08, 0.01, 0.0)),
        "right_ear": head.add(Vec3(0.08, 0.01, 0.0)),
        "left_shoulder": left_shoulder,
        "right_shoulder": right_shoulder,
        "left_elbow": left_shoulder.add(Vec3(-0.13, -0.18, 0.015)),
        "right_elbow": right_shoulder.add(Vec3(0.13, -0.18, 0.015)),
        "left_wrist": left_shoulder.add(Vec3(-0.20, -0.37, 0.02)),
        "right_wrist": right_shoulder.add(Vec3(0.20, -0.37, 0.02)),
        "left_hip": left_hip,
        "right_hip": right_hip,
        "left_knee": left_knee,
        "right_knee": right_knee,
        "left_ankle": left_ankle,
        "right_ankle": right_ankle,
        "head": head,
        "neck": neck,
        "hip": root,
        "left_big_toe": left_toe.add(Vec3(-0.022, 0.0, 0.0)),
        "right_big_toe": right_toe.add(Vec3(0.022, 0.0, 0.0)),
        "left_small_toe": left_toe.add(Vec3(0.030, 0.0, -0.006)),
        "right_small_toe": right_toe.add(Vec3(-0.030, 0.0, -0.006)),
        "left_heel": left_heel,
        "right_heel": right_heel,
    }
    return joints


def scenario_frame(name: str, index: int, frames_per_scenario: int) -> ScenarioFrame:
    denom = max(1, frames_per_scenario - 1)
    t = index / denom
    root = Vec3(0.0, 0.96, 2.25)
    left_flat = FootPose(Vec3(-0.18, 0.020, 2.22), 0.0)
    right_flat = FootPose(Vec3(0.18, 0.020, 2.24), 0.0)

    if name == "pure_airborne_leg_swing":
        swing = math.sin(math.pi * t)
        right = FootPose(Vec3(0.18 + 0.06 * math.sin(2.0 * math.pi * t), 0.020 + 0.33 * swing, 2.02 + 0.48 * t), 0.45 * math.sin(2.0 * math.pi * t))
        return ScenarioFrame(name, index, t, root, left_flat, right, "FLAT_PLANT", "SWING", 0.98, 0.08, right_contact_type="NONE", right_planted=False, active_free_foot="right", stance_foot="left", noise_px=0.8, foot_noise_px=1.2)

    if name == "body_over_planted_stance":
        root = Vec3(-0.09 + 0.28 * smoothstep(t), 0.96, 2.25)
        right = FootPose(Vec3(0.30, 0.060 + 0.10 * sine01(t), 2.28 + 0.08 * t), 0.18 * math.sin(math.pi * t))
        return ScenarioFrame(name, index, t, root, left_flat, right, "FLAT_PLANT", "SWING", 0.99, 0.12, right_contact_type="NONE", right_planted=False, active_free_foot="right", stance_foot="left", noise_px=0.8, foot_noise_px=1.1)

    if name == "planted_foot_jitter_from_2d_noise":
        return ScenarioFrame(name, index, t, root, left_flat, right_flat, "FLAT_PLANT", "FLAT_PLANT", 0.98, 0.98, stance_foot="both", noise_px=0.7, foot_noise_px=6.5)

    if name == "heel_lock":
        heel_anchor = foot_contact_point(left_flat, -0.095)
        left = foot_from_heel_anchor(heel_anchor, -0.55 * smoothstep(t))
        right = FootPose(Vec3(0.19, 0.030 + 0.08 * sine01(t), 2.34), 0.08)
        return ScenarioFrame(name, index, t, root, left, right, "HEEL_LOCK", "SWING", 0.96, 0.15, right_contact_type="NONE", right_planted=False, active_free_foot="right", stance_foot="left", noise_px=0.9, foot_noise_px=1.4)

    if name == "toe_pivot":
        toe_anchor = foot_contact_point(left_flat, 0.125)
        left = foot_from_toe_anchor(toe_anchor, 0.68 * smoothstep(t))
        right = FootPose(Vec3(0.22, 0.040 + 0.07 * sine01(t), 2.35), 0.12)
        return ScenarioFrame(name, index, t, root, left, right, "TOE_PIVOT", "SWING", 0.96, 0.14, right_contact_type="NONE", right_planted=False, active_free_foot="right", stance_foot="left", noise_px=0.9, foot_noise_px=1.4)

    if name == "flat_plant":
        root = Vec3(0.02 * math.sin(2 * math.pi * t), 0.96, 2.25 + 0.015 * math.sin(2 * math.pi * t))
        return ScenarioFrame(name, index, t, root, left_flat, right_flat, "FLAT_PLANT", "FLAT_PLANT", 0.99, 0.99, stance_foot="both", noise_px=0.5, foot_noise_px=0.8)

    if name == "slip_release":
        phase = "FLAT_PLANT" if t < 0.35 else ("RELEASE_PENDING" if t < 0.55 else "SLIP")
        support_conf = 0.98 if t < 0.35 else (0.55 if t < 0.55 else 0.20)
        slip_t = smoothstep((t - 0.35) / 0.65)
        left = FootPose(Vec3(-0.18 - 0.16 * slip_t, 0.020, 2.22 + 0.05 * slip_t), 0.02)
        root = Vec3(-0.02 - 0.05 * slip_t, 0.96, 2.25)
        return ScenarioFrame(name, index, t, root, left, right_flat, phase, "FLAT_PLANT", support_conf, 0.96, left_planted=t < 0.55, stance_foot="right", noise_px=0.8, foot_noise_px=1.2)

    if name == "support_transition_griddy":
        step = math.sin(4.0 * math.pi * t)
        left_lift = max(0.0, step)
        right_lift = max(0.0, -step)
        left_phase = "SWING" if left_lift > 0.10 else ("CONTACT_CANDIDATE" if abs(step) < 0.18 else "FLAT_PLANT")
        right_phase = "SWING" if right_lift > 0.10 else ("CONTACT_CANDIDATE" if abs(step) < 0.18 else "FLAT_PLANT")
        left = FootPose(Vec3(-0.20 + 0.10 * t, 0.020 + 0.17 * left_lift, 2.16 + 0.18 * t), 0.18 * left_lift)
        right = FootPose(Vec3(0.18 + 0.10 * t, 0.020 + 0.17 * right_lift, 2.27 + 0.18 * t), 0.18 * right_lift)
        root = Vec3(0.02 + 0.10 * t, 0.96, 2.24 + 0.16 * t)
        return ScenarioFrame(name, index, t, root, left, right, left_phase, right_phase, 0.15 if left_phase == "SWING" else 0.86, 0.15 if right_phase == "SWING" else 0.86, left_contact_type="NONE" if left_phase == "SWING" else "FLOOR_SUPPORT", right_contact_type="NONE" if right_phase == "SWING" else "FLOOR_SUPPORT", left_planted=left_phase != "SWING", right_planted=right_phase != "SWING", active_free_foot="alternating", stance_foot="alternating", noise_px=1.0, foot_noise_px=1.8)

    if name == "one_camera_foot_occlusion":
        swing = math.sin(math.pi * t)
        right = FootPose(Vec3(0.20, 0.020 + 0.12 * swing, 2.22 + 0.18 * t), 0.05)
        return ScenarioFrame(name, index, t, root, left_flat, right, "FLAT_PLANT", "CONTACT_CANDIDATE", 0.98, 0.70, stance_foot="left", noise_px=0.9, foot_noise_px=1.4, occlude_camera_b_foot="right", occlusion_reason="synthetic_body_blocks_camera_b_right_foot")

    if name == "both_camera_foot_occlusion":
        right = FootPose(Vec3(0.18, 0.110 + 0.10 * sine01(t), 2.30 + 0.04 * t), 0.12)
        return ScenarioFrame(name, index, t, root, left_flat, right, "FLAT_PLANT", "SWING", 0.98, 0.05, right_contact_type="NONE", right_planted=False, active_free_foot="right", stance_foot="left", noise_px=0.8, foot_noise_px=1.4, occlude_camera_a_foot="right", occlude_camera_b_foot="right", occlusion_reason="synthetic_full_right_foot_occlusion")

    if name == "two_camera_disagreement":
        right = FootPose(Vec3(0.18, 0.020, 2.24 + 0.05 * math.sin(2 * math.pi * t)), 0.0)
        return ScenarioFrame(name, index, t, root, left_flat, right, "FLAT_PLANT", "FLAT_PLANT", 0.98, 0.92, stance_foot="both", noise_px=0.6, foot_noise_px=1.0, camera_b_right_foot_offset_px=(22.0, -11.0))

    if name == "mild_calibration_imperfection":
        root = Vec3(0.07 * math.sin(2 * math.pi * t), 0.96, 2.25 + 0.08 * t)
        return ScenarioFrame(name, index, t, root, left_flat, right_flat, "FLAT_PLANT", "FLAT_PLANT", 0.98, 0.98, stance_foot="both", noise_px=0.5, foot_noise_px=0.8, mild_calibration_imperfection=True)

    if name == "low_res_heel_toe_ankle_ambiguity":
        left = FootPose(Vec3(-0.16, 0.020, 3.15), 0.02 * math.sin(2 * math.pi * t))
        right = FootPose(Vec3(0.16, 0.020, 3.16), -0.02 * math.sin(2 * math.pi * t))
        root = Vec3(0.0, 0.96, 3.18)
        return ScenarioFrame(name, index, t, root, left, right, "FLAT_PLANT", "FLAT_PLANT", 0.78, 0.78, stance_foot="both", noise_px=1.4, foot_noise_px=3.6, left_foot_confidence=0.56, right_foot_confidence=0.56, low_res_foot_ambiguity=True)

    raise ValueError(f"Unknown scenario: {name}")


def nominal_cameras(width: int, height: int) -> Dict[str, Camera]:
    fx = 260.0 * (width / DEFAULT_WIDTH)
    fy = 260.0 * (height / DEFAULT_HEIGHT)
    cx = width * 0.5
    cy = height * 0.52
    return {
        "camera_a": Camera("camera_a", "A", -0.24, 0.82, -1.45, fx, fy, cx, cy, width, height),
        "camera_b": Camera("camera_b", "B", 0.24, 0.82, -1.45, fx, fy, cx, cy, width, height),
    }


def actual_cameras(frame: ScenarioFrame, cameras: Mapping[str, Camera]) -> Dict[str, Camera]:
    if not frame.mild_calibration_imperfection:
        return dict(cameras)
    return {
        "camera_a": cameras["camera_a"],
        "camera_b": cameras["camera_b"].with_delta(dx=0.035, dy=-0.010, dz=0.015, dfx=cameras["camera_b"].fx * 0.018, dfy=-cameras["camera_b"].fy * 0.012),
    }


def project(camera: Camera, world: Vec3) -> Tuple[Optional[Tuple[float, float]], bool]:
    z = world.z - camera.z
    if z <= 0.05:
        return None, False
    x = world.x - camera.x
    y_down = camera.y - world.y
    u = camera.fx * x / z + camera.cx
    v = camera.fy * y_down / z + camera.cy
    inside = -32.0 <= u <= camera.width + 32.0 and -32.0 <= v <= camera.height + 32.0
    return (u, v), inside


def reproject_error(camera: Camera, world: Vec3, observed_pixel: Tuple[float, float]) -> float:
    pixel, visible = project(camera, world)
    if pixel is None or not visible:
        return float("inf")
    return math.hypot(pixel[0] - observed_pixel[0], pixel[1] - observed_pixel[1])


def triangulate_nominal_pair(camera_a: Camera, camera_b: Camera,
                             pixel_a: Tuple[float, float],
                             pixel_b: Tuple[float, float]) -> Optional[Vec3]:
    """Triangulate a pair using the same simple pinhole model encoded in Camera.

    The synthetic rig has parallel cameras. Runtime calibration bias is represented by projecting
    with the actual camera but triangulating with nominal camera metadata, which makes the
    resulting world error/reprojection error a deliberate blame signal.
    """

    ax = (pixel_a[0] - camera_a.cx) / camera_a.fx
    bx = (pixel_b[0] - camera_b.cx) / camera_b.fx
    denom = ax - bx
    if abs(denom) < 1.0e-6:
        return None

    z = (camera_b.x - camera_a.x + ax * camera_a.z - bx * camera_b.z) / denom
    if not math.isfinite(z) or z <= max(camera_a.z, camera_b.z) + 0.05:
        return None

    x_a = camera_a.x + ax * (z - camera_a.z)
    x_b = camera_b.x + bx * (z - camera_b.z)
    ay = (pixel_a[1] - camera_a.cy) / camera_a.fy
    by = (pixel_b[1] - camera_b.cy) / camera_b.fy
    y_a = camera_a.y - ay * (z - camera_a.z)
    y_b = camera_b.y - by * (z - camera_b.z)
    world = Vec3(0.5 * (x_a + x_b), 0.5 * (y_a + y_b), z)
    if not math.isfinite(world.x) or not math.isfinite(world.y) or not math.isfinite(world.z):
        return None
    return world


def foot_side_for_keypoint(name: str) -> Optional[str]:
    if name in FOOT_KEYPOINTS["left"]:
        return "left"
    if name in FOOT_KEYPOINTS["right"]:
        return "right"
    return None


def board_world_points(rows: int = 5, cols: int = 7, square_m: float = 0.09) -> List[Dict[str, object]]:
    points = []
    origin_x = -0.5 * (cols - 1) * square_m
    origin_z = 1.62
    for r in range(rows):
        for c in range(cols):
            points.append({
                "row": r,
                "col": c,
                "world": Vec3(origin_x + c * square_m, 0.001, origin_z + r * square_m).as_list(),
                "black": (r + c) % 2 == 0,
            })
    return points


def projected_board(camera: Camera) -> List[Dict[str, object]]:
    out = []
    for point in board_world_points():
        p = Vec3(*point["world"])  # type: ignore[arg-type]
        pixel, visible = project(camera, p)
        out.append({
            "row": point["row"],
            "col": point["col"],
            "black": point["black"],
            "visible": visible and pixel is not None,
            "pixel": [round(pixel[0], 3), round(pixel[1], 3)] if pixel is not None else None,
        })
    return out


def maybe_ambiguous_pixel(name: str, pixel: Tuple[float, float], joints_pixels: Mapping[str, Tuple[float, float]], frame: ScenarioFrame) -> Tuple[float, float]:
    if not frame.low_res_foot_ambiguity or name not in LOWER_BODY_NAMES:
        return pixel
    side = "left" if name.startswith("left_") else ("right" if name.startswith("right_") else "")
    if side and name in FOOT_KEYPOINTS[side]:
        ankle_name = f"{side}_ankle"
        ankle_pixel = joints_pixels.get(ankle_name, pixel)
        # Collapse heel/toe toward ankle so a low-res model cannot reliably separate phase.
        return (
            ankle_pixel[0] + 0.28 * (pixel[0] - ankle_pixel[0]),
            ankle_pixel[1] + 0.28 * (pixel[1] - ankle_pixel[1]),
        )
    return pixel


def project_keypoints(frame: ScenarioFrame, camera_name: str, actual_camera: Camera,
                      rng: random.Random) -> Tuple[List[Dict[str, object]], Dict[str, float]]:
    joints = lower_body_joints(frame.root, frame.left_foot, frame.right_foot)
    raw_pixels: Dict[str, Tuple[float, float]] = {}
    raw_visible: Dict[str, bool] = {}
    for name, world in joints.items():
        pixel, visible = project(actual_camera, world)
        if pixel is not None:
            raw_pixels[name] = pixel
        raw_visible[name] = visible and pixel is not None

    keypoints: List[Dict[str, object]] = []
    visible_count = 0
    confidence_sum = 0.0
    foot_visible = 0
    foot_total = 0

    for idx, name in enumerate(KEYPOINT_NAMES):
        world = joints[name]
        base_pixel = raw_pixels.get(name)
        projected = base_pixel is not None and raw_visible.get(name, False)

        side = "left" if name in FOOT_KEYPOINTS["left"] else ("right" if name in FOOT_KEYPOINTS["right"] else None)
        explicit_occluded = False
        occlusion_reason = ""
        if side is not None:
            if camera_name == "camera_a" and frame.occlude_camera_a_foot == side:
                explicit_occluded = True
                occlusion_reason = frame.occlusion_reason
            if camera_name == "camera_b" and frame.occlude_camera_b_foot == side:
                explicit_occluded = True
                occlusion_reason = frame.occlusion_reason

        if base_pixel is None:
            base_pixel = (float("nan"), float("nan"))

        ambiguous_pixel = maybe_ambiguous_pixel(name, base_pixel, raw_pixels, frame)
        noise_scale = frame.foot_noise_px if side is not None else frame.noise_px
        if frame.low_res_foot_ambiguity and side is not None:
            # Keep detections visible but phase-ambiguous: heel/toe/ankle occupy nearly the same low-res pixels.
            noise_scale = min(noise_scale, 1.2)

        du = rng.gauss(0.0, noise_scale)
        dv = rng.gauss(0.0, noise_scale)
        offset = (0.0, 0.0)
        disagreement_px = 0.0
        if camera_name == "camera_b" and side == "right" and frame.camera_b_right_foot_offset_px != (0.0, 0.0):
            offset = frame.camera_b_right_foot_offset_px
            disagreement_px = math.hypot(offset[0], offset[1])

        noisy = (ambiguous_pixel[0] + du + offset[0], ambiguous_pixel[1] + dv + offset[1])
        in_image = 0.0 <= noisy[0] <= actual_camera.width - 1 and 0.0 <= noisy[1] <= actual_camera.height - 1
        visible = projected and in_image and not explicit_occluded

        base_conf = 0.92 if name not in LOWER_BODY_NAMES else 0.86
        if side == "left":
            base_conf = frame.left_foot_confidence
        elif side == "right":
            base_conf = frame.right_foot_confidence
        if frame.low_res_foot_ambiguity and side is not None:
            base_conf = min(base_conf, 0.58)
        if frame.camera_b_right_foot_offset_px != (0.0, 0.0) and camera_name == "camera_b" and side == "right":
            # Coherent wrong detections are intentionally still confident.
            base_conf = max(base_conf, 0.82)

        confidence = clamp01(base_conf - 0.012 * math.hypot(du, dv))
        if not visible:
            confidence = 0.02 if explicit_occluded else 0.05

        if visible:
            visible_count += 1
            confidence_sum += confidence
        if side is not None:
            foot_total += 1
            if visible:
                foot_visible += 1

        keypoints.append({
            "id": idx,
            "name": name,
            "world": world.as_list(),
            "ground_truth_pixel": [round(ambiguous_pixel[0], 3), round(ambiguous_pixel[1], 3)] if projected else None,
            "pixel": [round(noisy[0], 3), round(noisy[1], 3)] if visible else None,
            "noise_px": [round(du, 3), round(dv, 3)],
            "camera_offset_px": [round(offset[0], 3), round(offset[1], 3)],
            "disagreement_px": round(disagreement_px, 3),
            "confidence": round(confidence, 4),
            "visible": visible,
            "occluded": explicit_occluded,
            "occlusion_reason": occlusion_reason,
            "low_res_ambiguous": frame.low_res_foot_ambiguity and side is not None,
        })

    metrics = {
        "visible_keypoints": visible_count,
        "mean_visible_confidence": confidence_sum / visible_count if visible_count else 0.0,
        "visible_foot_keypoints": foot_visible,
        "foot_keypoints": foot_total,
    }
    return keypoints, metrics


def build_stereo_blame_diagnostics(cameras_json: Mapping[str, object],
                                    nominal: Mapping[str, Camera],
                                    joints: Mapping[str, Vec3]) -> Dict[str, object]:
    keypoints_a = {
        str(kp["name"]): kp
        for kp in cameras_json["camera_a"]["keypoints"]  # type: ignore[index]
    }
    keypoints_b = {
        str(kp["name"]): kp
        for kp in cameras_json["camera_b"]["keypoints"]  # type: ignore[index]
    }

    keypoints: List[Dict[str, object]] = []
    foot_summary: Dict[str, MutableMapping[str, object]] = {
        "left": {
            "keypoints": list(sorted(FOOT_KEYPOINTS["left"])),
            "both_visible": 0,
            "either_visible": 0,
            "triangulated": 0,
            "mean_pair_confidence": 0.0,
            "mean_triangulated_confidence": 0.0,
            "mean_reprojection_error_px": 0.0,
            "max_reprojection_error_px": 0.0,
            "mean_world_error_m": 0.0,
            "max_world_error_m": 0.0,
            "min_camera_confidence": 1.0,
        },
        "right": {
            "keypoints": list(sorted(FOOT_KEYPOINTS["right"])),
            "both_visible": 0,
            "either_visible": 0,
            "triangulated": 0,
            "mean_pair_confidence": 0.0,
            "mean_triangulated_confidence": 0.0,
            "mean_reprojection_error_px": 0.0,
            "max_reprojection_error_px": 0.0,
            "mean_world_error_m": 0.0,
            "max_world_error_m": 0.0,
            "min_camera_confidence": 1.0,
        },
    }
    totals = {
        "triangulated_keypoints": 0,
        "mean_triangulated_confidence": 0.0,
        "mean_reprojection_error_px": 0.0,
        "max_reprojection_error_px": 0.0,
        "mean_world_error_m": 0.0,
        "max_world_error_m": 0.0,
    }

    for idx, name in enumerate(KEYPOINT_NAMES):
        kp_a = keypoints_a[name]
        kp_b = keypoints_b[name]
        visible_a = bool(kp_a["visible"])
        visible_b = bool(kp_b["visible"])
        conf_a = float(kp_a["confidence"])
        conf_b = float(kp_b["confidence"])
        pair_confidence = math.sqrt(max(0.0, conf_a) * max(0.0, conf_b)) if visible_a and visible_b else 0.0

        pixel_a = kp_a["pixel"] if visible_a else None
        pixel_b = kp_b["pixel"] if visible_b else None
        triangulated_world: Optional[Vec3] = None
        reproj_a = 0.0
        reproj_b = 0.0
        mean_reproj = 0.0
        world_error = 0.0
        triangulated_confidence = 0.0
        horizontal_disparity_px = 0.0

        if pixel_a is not None and pixel_b is not None:
            pa = (float(pixel_a[0]), float(pixel_a[1]))  # type: ignore[index]
            pb = (float(pixel_b[0]), float(pixel_b[1]))  # type: ignore[index]
            horizontal_disparity_px = abs(pa[0] - pb[0])
            triangulated_world = triangulate_nominal_pair(nominal["camera_a"], nominal["camera_b"], pa, pb)
            if triangulated_world is not None:
                reproj_a = reproject_error(nominal["camera_a"], triangulated_world, pa)
                reproj_b = reproject_error(nominal["camera_b"], triangulated_world, pb)
                mean_reproj = 0.5 * (reproj_a + reproj_b)
                world_error = math.sqrt(
                    (triangulated_world.x - joints[name].x) ** 2 +
                    (triangulated_world.y - joints[name].y) ** 2 +
                    (triangulated_world.z - joints[name].z) ** 2
                )
                triangulated_confidence = clamp01(pair_confidence / (1.0 + mean_reproj))

        side = foot_side_for_keypoint(name)
        if side is not None:
            summary = foot_summary[side]
            if visible_a or visible_b:
                summary["either_visible"] = int(summary["either_visible"]) + 1
            if visible_a and visible_b:
                summary["both_visible"] = int(summary["both_visible"]) + 1
                summary["mean_pair_confidence"] = float(summary["mean_pair_confidence"]) + pair_confidence
                summary["min_camera_confidence"] = min(float(summary["min_camera_confidence"]), conf_a, conf_b)
            elif visible_a:
                summary["min_camera_confidence"] = min(float(summary["min_camera_confidence"]), conf_a)
            elif visible_b:
                summary["min_camera_confidence"] = min(float(summary["min_camera_confidence"]), conf_b)
            if triangulated_world is not None:
                summary["triangulated"] = int(summary["triangulated"]) + 1
                summary["mean_triangulated_confidence"] = float(summary["mean_triangulated_confidence"]) + triangulated_confidence
                summary["mean_reprojection_error_px"] = float(summary["mean_reprojection_error_px"]) + mean_reproj
                summary["max_reprojection_error_px"] = max(float(summary["max_reprojection_error_px"]), mean_reproj)
                summary["mean_world_error_m"] = float(summary["mean_world_error_m"]) + world_error
                summary["max_world_error_m"] = max(float(summary["max_world_error_m"]), world_error)

        if triangulated_world is not None:
            totals["triangulated_keypoints"] = int(totals["triangulated_keypoints"]) + 1
            totals["mean_triangulated_confidence"] = float(totals["mean_triangulated_confidence"]) + triangulated_confidence
            totals["mean_reprojection_error_px"] = float(totals["mean_reprojection_error_px"]) + mean_reproj
            totals["max_reprojection_error_px"] = max(float(totals["max_reprojection_error_px"]), mean_reproj)
            totals["mean_world_error_m"] = float(totals["mean_world_error_m"]) + world_error
            totals["max_world_error_m"] = max(float(totals["max_world_error_m"]), world_error)

        keypoints.append({
            "id": idx,
            "name": name,
            "foot_side": side,
            "camera_a": {
                "visible": visible_a,
                "confidence": round(conf_a, 4),
                "pixel": pixel_a,
                "noise_px": kp_a["noise_px"],
                "occluded": kp_a["occluded"],
            },
            "camera_b": {
                "visible": visible_b,
                "confidence": round(conf_b, 4),
                "pixel": pixel_b,
                "noise_px": kp_b["noise_px"],
                "occluded": kp_b["occluded"],
                "synthetic_offset_px": kp_b["camera_offset_px"],
            },
            "both_visible": visible_a and visible_b,
            "pair_confidence": round(pair_confidence, 4),
            "horizontal_disparity_px": round(horizontal_disparity_px, 3),
            "triangulated": triangulated_world is not None,
            "triangulated_world": triangulated_world.as_list() if triangulated_world is not None else None,
            "triangulated_confidence": round(triangulated_confidence, 4),
            "reprojection_error_px": {
                "camera_a": round(reproj_a, 4),
                "camera_b": round(reproj_b, 4),
                "mean": round(mean_reproj, 4),
            },
            "world_error_m": round(world_error, 5),
        })

    for side, summary in foot_summary.items():
        both_visible = int(summary["both_visible"])
        triangulated = int(summary["triangulated"])
        if both_visible > 0:
            summary["mean_pair_confidence"] = round(float(summary["mean_pair_confidence"]) / both_visible, 4)
        else:
            summary["mean_pair_confidence"] = 0.0
            summary["min_camera_confidence"] = 0.0
        if triangulated > 0:
            summary["mean_triangulated_confidence"] = round(float(summary["mean_triangulated_confidence"]) / triangulated, 4)
            summary["mean_reprojection_error_px"] = round(float(summary["mean_reprojection_error_px"]) / triangulated, 4)
            summary["mean_world_error_m"] = round(float(summary["mean_world_error_m"]) / triangulated, 5)
        else:
            summary["mean_triangulated_confidence"] = 0.0
            summary["mean_reprojection_error_px"] = 0.0
            summary["mean_world_error_m"] = 0.0
        summary["max_reprojection_error_px"] = round(float(summary["max_reprojection_error_px"]), 4)
        summary["max_world_error_m"] = round(float(summary["max_world_error_m"]), 5)
        summary["min_camera_confidence"] = round(float(summary["min_camera_confidence"]), 4)

    tri_count = int(totals["triangulated_keypoints"])
    if tri_count > 0:
        totals["mean_triangulated_confidence"] = round(float(totals["mean_triangulated_confidence"]) / tri_count, 4)
        totals["mean_reprojection_error_px"] = round(float(totals["mean_reprojection_error_px"]) / tri_count, 4)
        totals["mean_world_error_m"] = round(float(totals["mean_world_error_m"]) / tri_count, 5)
    totals["max_reprojection_error_px"] = round(float(totals["max_reprojection_error_px"]), 4)
    totals["max_world_error_m"] = round(float(totals["max_world_error_m"]), 5)

    return {
        "schema_version": SCHEMA_VERSION,
        "description": "Per-joint synthetic camera A/B evidence and nominal-stereo triangulation blame data.",
        "summary": totals,
        "foot_summary": foot_summary,
        "keypoints": keypoints,
    }


def support_json(frame: ScenarioFrame, side: str, foot: FootPose, phase: str,
                 contact_type: str, support_confidence: float, planted: bool) -> Dict[str, object]:
    heel = foot_contact_point(foot, -0.095)
    toe = foot_contact_point(foot, 0.125)
    active = contact_type != "NONE" and phase not in {"SWING", "SLIP"}
    heel_active = active and phase in {"CONTACT_CANDIDATE", "HEEL_LOCK", "FLAT_PLANT"}
    toe_active = active and phase in {"TOE_PIVOT", "FLAT_PLANT"}
    return {
        "side": side,
        "type": contact_type,
        "phase": phase,
        "planted": planted,
        "support_confidence": round(support_confidence, 4),
        "pose": foot.as_pose_json(),
        "anchor": {
            "active": active,
            "position": foot.center.as_list(),
            "confidence": round(support_confidence, 4) if active else 0.0,
        },
        "heel_anchor": {
            "active": heel_active,
            "position": heel.as_list(),
            "confidence": round(support_confidence, 4) if heel_active else 0.0,
        },
        "toe_anchor": {
            "active": toe_active,
            "position": toe.as_list(),
            "confidence": round(support_confidence, 4) if toe_active else 0.0,
        },
        "heel_contact_point": heel.as_list(),
        "toe_contact_point": toe.as_list(),
    }


def frame_to_record(frame: ScenarioFrame, global_index: int, timestamp_seconds: float,
                    nominal: Mapping[str, Camera], seed: int) -> Dict[str, object]:
    actual = actual_cameras(frame, nominal)
    joints = lower_body_joints(frame.root, frame.left_foot, frame.right_foot)
    cameras_json: Dict[str, object] = {}
    camera_metrics: Dict[str, object] = {}

    for camera_name in ("camera_a", "camera_b"):
        rng = random.Random(seed * 1000003 + global_index * 97 + (0 if camera_name == "camera_a" else 41))
        keypoints, metrics = project_keypoints(frame, camera_name, actual[camera_name], rng)
        nominal_camera = nominal[camera_name]
        actual_camera = actual[camera_name]
        calibration_delta = {
            "position_delta_m": [
                round(actual_camera.x - nominal_camera.x, 6),
                round(actual_camera.y - nominal_camera.y, 6),
                round(actual_camera.z - nominal_camera.z, 6),
            ],
            "fx_delta_px": round(actual_camera.fx - nominal_camera.fx, 6),
            "fy_delta_px": round(actual_camera.fy - nominal_camera.fy, 6),
            "imperfect": frame.mild_calibration_imperfection and camera_name == "camera_b",
        }
        cameras_json[camera_name] = {
            "camera_id": nominal_camera.camera_id,
            "nominal": nominal_camera.to_json(),
            "actual_used_for_projection": actual_camera.to_json(),
            "calibration_delta": calibration_delta,
            "keypoints": keypoints,
            "chessboard": projected_board(actual_camera),
        }
        camera_metrics[camera_name] = metrics

    stereo_blame = build_stereo_blame_diagnostics(cameras_json, nominal, joints)

    left_support = support_json(
        frame, "left", frame.left_foot, frame.left_phase, frame.left_contact_type,
        frame.left_support_confidence, frame.left_planted)
    right_support = support_json(
        frame, "right", frame.right_foot, frame.right_phase, frame.right_contact_type,
        frame.right_support_confidence, frame.right_planted)

    root_motion_note = "stationary"
    if frame.scenario == "body_over_planted_stance":
        root_motion_note = "root_moves_over_left_stance_without_foot_common_mode"
    elif frame.scenario == "pure_airborne_leg_swing":
        root_motion_note = "root_stationary_right_foot_airborne"
    elif frame.scenario == "slip_release":
        root_motion_note = "left_slip_should_not_drag_root_back_to_anchor"

    return {
        "schema_version": SCHEMA_VERSION,
        "frame_index": global_index,
        "scenario": frame.scenario,
        "scenario_description": SCENARIO_DESCRIPTIONS[frame.scenario],
        "scenario_frame_index": frame.scenario_frame_index,
        "scenario_t": round(frame.t, 6),
        "timestamp_seconds": round(timestamp_seconds, 6),
        "image_size": [nominal["camera_a"].width, nominal["camera_a"].height],
        "ground_truth": {
            "root": {
                "pose": {
                    "position": frame.root.as_list(),
                    "orientation": [0.0, 0.0, 0.0, 1.0],
                }
            },
            "left_foot": left_support,
            "right_foot": right_support,
            "joints": {name: joints[name].as_list() for name in KEYPOINT_NAMES},
            "active_free_foot": frame.active_free_foot,
            "stance_foot": frame.stance_foot,
            "failure_probes": SCENARIO_RISKS[frame.scenario],
            "root_motion_note": root_motion_note,
        },
        "cameras": cameras_json,
        "diagnostics": {
            "camera_metrics": camera_metrics,
            "stereo_blame": stereo_blame,
            "scenario_risks": SCENARIO_RISKS[frame.scenario],
            "expected_runtime_blame_fields": [
                "tracking.stages.predicted/preliminary/measured/motion_filtered/ekf_filtered/corrected",
                "tracking.motion_filter.contact_root",
                "tracking.support.left_foot.heel_residual/toe_residual",
                "tracking.support.right_foot.heel_residual/toe_residual",
                "camera_*_pose.keypoints[*].id/name/confidence/present",
                "camera_*_reliability_full.joints[*].id/name/final_weight/occlusion_term/epipolar_term",
                "tracking.solver.triangulation.preliminary.joints[*].triangulated/confidence/reprojection_error_*",
            ],
        },
    }


def generate_records(frames_per_scenario: int, width: int, height: int, seed: int) -> List[Dict[str, object]]:
    nominal = nominal_cameras(width, height)
    records: List[Dict[str, object]] = []
    global_index = 0
    for scenario in SCENARIO_ORDER:
        for frame_index in range(frames_per_scenario):
            frame = scenario_frame(scenario, frame_index, frames_per_scenario)
            records.append(frame_to_record(frame, global_index, global_index / DEFAULT_FPS, nominal, seed))
            global_index += 1
    return records


def point3(value: object) -> Optional[Tuple[float, float, float]]:
    if not isinstance(value, list) or len(value) != 3:
        return None
    try:
        return (float(value[0]), float(value[1]), float(value[2]))
    except (TypeError, ValueError):
        return None


def distance3(a: Optional[Tuple[float, float, float]], b: Optional[Tuple[float, float, float]]) -> Optional[float]:
    if a is None or b is None:
        return None
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def vector_sub3(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> Tuple[float, float, float]:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def vector_dot3(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def vector_norm3(a: Tuple[float, float, float]) -> float:
    return math.sqrt(vector_dot3(a, a))


def mean_point3(points: Sequence[Tuple[float, float, float]]) -> Optional[Tuple[float, float, float]]:
    if not points:
        return None
    inv = 1.0 / len(points)
    return (
        sum(p[0] for p in points) * inv,
        sum(p[1] for p in points) * inv,
        sum(p[2] for p in points) * inv,
    )


def path_length3(points: Sequence[Optional[Tuple[float, float, float]]]) -> Optional[float]:
    filtered = [p for p in points if p is not None]
    if len(filtered) < 2:
        return None
    total = 0.0
    for previous, current in zip(filtered, filtered[1:]):
        total += distance3(previous, current) or 0.0
    return total


def max_displacement3(points: Sequence[Optional[Tuple[float, float, float]]]) -> Optional[float]:
    filtered = [p for p in points if p is not None]
    if len(filtered) < 2:
        return None
    origin = filtered[0]
    return max(distance3(origin, p) or 0.0 for p in filtered[1:])


def rounded_metric(value: Optional[float], digits: int = 5) -> Optional[float]:
    if value is None or not math.isfinite(value):
        return None
    return round(value, digits)


def stereo_keypoint_by_name(record: Mapping[str, object], name: str) -> Optional[Mapping[str, object]]:
    try:
        keypoints = record["diagnostics"]["stereo_blame"]["keypoints"]  # type: ignore[index]
    except (KeyError, TypeError):
        return None
    if not isinstance(keypoints, list):
        return None
    for keypoint in keypoints:
        if isinstance(keypoint, Mapping) and keypoint.get("name") == name:
            return keypoint
    return None


def triangulated_keypoint(record: Mapping[str, object], name: str) -> Optional[Tuple[float, float, float]]:
    keypoint = stereo_keypoint_by_name(record, name)
    if keypoint is None or not keypoint.get("triangulated"):
        return None
    return point3(keypoint.get("triangulated_world"))


def gt_joint(record: Mapping[str, object], name: str) -> Optional[Tuple[float, float, float]]:
    try:
        return point3(record["ground_truth"]["joints"][name])  # type: ignore[index]
    except (KeyError, TypeError):
        return None


def gt_root_position(record: Mapping[str, object]) -> Optional[Tuple[float, float, float]]:
    try:
        return point3(record["ground_truth"]["root"]["pose"]["position"])  # type: ignore[index]
    except (KeyError, TypeError):
        return None


def gt_foot_pose_position(record: Mapping[str, object], side: str) -> Optional[Tuple[float, float, float]]:
    try:
        return point3(record["ground_truth"][f"{side}_foot"]["pose"]["position"])  # type: ignore[index]
    except (KeyError, TypeError):
        return None


def gt_foot_anchor(record: Mapping[str, object], side: str, anchor_name: str) -> Optional[Tuple[float, float, float]]:
    try:
        anchor = record["ground_truth"][f"{side}_foot"][anchor_name]  # type: ignore[index]
        if isinstance(anchor, Mapping):
            return point3(anchor.get("position"))
    except (KeyError, TypeError):
        return None
    return None


def gt_foot_phase(record: Mapping[str, object], side: str) -> str:
    try:
        return str(record["ground_truth"][f"{side}_foot"]["phase"])  # type: ignore[index]
    except (KeyError, TypeError):
        return "UNKNOWN"


def gt_support_confidence(record: Mapping[str, object], side: str) -> float:
    try:
        return float(record["ground_truth"][f"{side}_foot"]["support_confidence"])  # type: ignore[index]
    except (KeyError, TypeError, ValueError):
        return 0.0


def gt_foot_planted(record: Mapping[str, object], side: str) -> bool:
    try:
        return bool(record["ground_truth"][f"{side}_foot"]["planted"])  # type: ignore[index]
    except (KeyError, TypeError):
        return False


def reconstructed_foot_centroid(record: Mapping[str, object], side: str) -> Optional[Tuple[float, float, float]]:
    points: List[Tuple[float, float, float]] = []
    for name in sorted(FOOT_KEYPOINTS[side]):
        point = triangulated_keypoint(record, name)
        if point is not None:
            points.append(point)
    if len(points) < 2:
        return None
    return mean_point3(points)


def gt_foot_centroid(record: Mapping[str, object], side: str) -> Optional[Tuple[float, float, float]]:
    points: List[Tuple[float, float, float]] = []
    for name in sorted(FOOT_KEYPOINTS[side]):
        point = gt_joint(record, name)
        if point is not None:
            points.append(point)
    return mean_point3(points)


def best_lag_frames(
    reference: Sequence[Optional[Tuple[float, float, float]]],
    observed: Sequence[Optional[Tuple[float, float, float]]],
    max_lag: int = 4,
) -> Optional[int]:
    """Estimate how many frames the observed path lags the reference.

    A positive lag means observed[i + lag] best matches reference[i]. This is deliberately simple:
    it is a diagnostic tripwire for held/delayed free-foot output, not a mocap synchronizer.
    """

    best_lag: Optional[int] = None
    best_rmse: Optional[float] = None
    max_lag = min(max_lag, max(0, len(reference) - 2))
    for lag in range(max_lag + 1):
        sq_error = 0.0
        count = 0
        for i in range(0, len(reference) - lag):
            ref = reference[i]
            obs = observed[i + lag]
            if ref is None or obs is None:
                continue
            err = distance3(ref, obs)
            if err is None:
                continue
            sq_error += err * err
            count += 1
        if count < 3:
            continue
        rmse = math.sqrt(sq_error / count)
        if best_rmse is None or rmse < best_rmse:
            best_rmse = rmse
            best_lag = lag
    return best_lag


def phase_flicker_count(phases: Sequence[str]) -> int:
    flickers = 0
    for i in range(2, len(phases)):
        if phases[i] == phases[i - 2] and phases[i] != phases[i - 1]:
            flickers += 1
    return flickers


def anchor_drift(
    records: Sequence[Mapping[str, object]],
    side: str,
    anchor_name: str,
    active_phases: Iterable[str],
) -> Optional[float]:
    active = set(active_phases)
    points = [
        gt_foot_anchor(record, side, anchor_name)
        for record in records
        if gt_foot_phase(record, side) in active
    ]
    return max_displacement3(points)


def slip_release_snap_back_amount(records: Sequence[Mapping[str, object]], side: str = "left") -> Optional[float]:
    centers = [gt_foot_pose_position(record, side) for record in records]
    phases = [gt_foot_phase(record, side) for record in records]
    release_index = next((i for i, phase in enumerate(phases) if phase in {"RELEASE_PENDING", "SLIP"}), None)
    if release_index is None or release_index == 0 or centers[release_index - 1] is None:
        return None
    valid_after = [(i, center) for i, center in enumerate(centers[release_index:], start=release_index) if center is not None]
    if len(valid_after) < 2:
        return None
    anchor = centers[release_index - 1]
    assert anchor is not None
    # Use the farthest released/slipping point as the intended slip direction. Using the final
    # point would miss the exact failure this metric is meant to catch: a stale-anchor snap-back
    # on the last frame.
    farthest = max(valid_after, key=lambda item: distance3(anchor, item[1]) or 0.0)[1]
    direction = vector_sub3(farthest, anchor)
    norm = vector_norm3(direction)
    if norm < 1.0e-6:
        return 0.0
    unit = (direction[0] / norm, direction[1] / norm, direction[2] / norm)
    running_max = 0.0
    snap_back = 0.0
    for _, center in valid_after:
        progress = vector_dot3(vector_sub3(center, anchor), unit)
        if progress < running_max:
            snap_back = max(snap_back, running_max - progress)
        running_max = max(running_max, progress)
    return snap_back


def pearson_correlation(xs: Sequence[float], ys: Sequence[float]) -> Optional[float]:
    if len(xs) != len(ys) or len(xs) < 2:
        return None
    mean_x = sum(xs) / len(xs)
    mean_y = sum(ys) / len(ys)
    dx = [x - mean_x for x in xs]
    dy = [y - mean_y for y in ys]
    denom = math.sqrt(sum(x * x for x in dx) * sum(y * y for y in dy))
    if denom <= 1.0e-12:
        return None
    return sum(x * y for x, y in zip(dx, dy)) / denom


def camera_disagreement_metrics(records: Sequence[Mapping[str, object]]) -> Dict[str, object]:
    disagreement_values: List[float] = []
    world_error_values: List[float] = []
    high_disagreement_count = 0
    high_disagreement_bad_3d_count = 0
    high_disagreement_active_support_frames = 0
    high_disagreement_active_support_bad_3d_frames = 0

    for record in records:
        foot_frame_max: Dict[str, Tuple[float, float]] = {"left": (0.0, 0.0), "right": (0.0, 0.0)}
        try:
            stereo_keypoints = record["diagnostics"]["stereo_blame"]["keypoints"]  # type: ignore[index]
        except (KeyError, TypeError):
            stereo_keypoints = []

        if isinstance(stereo_keypoints, list):
            for keypoint in stereo_keypoints:
                if not isinstance(keypoint, Mapping):
                    continue
                side = keypoint.get("foot_side")
                if side not in ("left", "right"):
                    continue
                try:
                    disagreement = float(keypoint["camera_b"]["synthetic_offset_px"][0]) ** 2 + float(keypoint["camera_b"]["synthetic_offset_px"][1]) ** 2  # type: ignore[index]
                    disagreement = math.sqrt(disagreement)
                except (KeyError, TypeError, ValueError):
                    disagreement = 0.0
                try:
                    world_error = float(keypoint.get("world_error_m", 0.0))
                except (TypeError, ValueError):
                    world_error = 0.0

                disagreement_values.append(disagreement)
                world_error_values.append(world_error)
                if disagreement >= 8.0:
                    high_disagreement_count += 1
                    if world_error >= 0.75:
                        high_disagreement_bad_3d_count += 1
                current_disagreement, current_error = foot_frame_max[str(side)]
                foot_frame_max[str(side)] = (max(current_disagreement, disagreement), max(current_error, world_error))

        for side in ("left", "right"):
            max_disagreement, max_world_error = foot_frame_max[side]
            if max_disagreement >= 8.0 and gt_support_confidence(record, side) >= 0.70 and gt_foot_phase(record, side) not in {"SWING", "SLIP"}:
                high_disagreement_active_support_frames += 1
                if max_world_error >= 0.75:
                    high_disagreement_active_support_bad_3d_frames += 1

    correlation = pearson_correlation(disagreement_values, world_error_values)
    precision = (
        high_disagreement_bad_3d_count / high_disagreement_count
        if high_disagreement_count > 0 else None
    )
    support_precision = (
        high_disagreement_active_support_bad_3d_frames / high_disagreement_active_support_frames
        if high_disagreement_active_support_frames > 0 else None
    )
    return {
        "max_camera_disagreement_px": rounded_metric(max(disagreement_values) if disagreement_values else 0.0, 3),
        "max_world_error_when_disagreed_m": rounded_metric(max((err for dis, err in zip(disagreement_values, world_error_values) if dis >= 8.0), default=0.0), 5),
        "camera_disagreement_world_error_correlation": rounded_metric(correlation, 4),
        "high_camera_disagreement_keypoints": high_disagreement_count,
        "high_camera_disagreement_bad_3d_keypoints": high_disagreement_bad_3d_count,
        "high_camera_disagreement_bad_3d_precision": rounded_metric(precision, 4),
        "high_camera_disagreement_active_support_frames": high_disagreement_active_support_frames,
        "high_camera_disagreement_active_support_bad_3d_frames": high_disagreement_active_support_bad_3d_frames,
        "high_camera_disagreement_predicts_bad_support_contact_decisions": (
            support_precision is not None and support_precision >= 0.75
        ),
        "high_camera_disagreement_support_contact_precision": rounded_metric(support_precision, 4),
    }


def low_res_ambiguity_metric(records: Sequence[Mapping[str, object]]) -> Optional[float]:
    separations: List[float] = []
    for record in records:
        try:
            keypoints = record["cameras"]["camera_a"]["keypoints"]  # type: ignore[index]
        except (KeyError, TypeError):
            continue
        if not isinstance(keypoints, list):
            continue
        by_name = {kp.get("name"): kp for kp in keypoints if isinstance(kp, Mapping)}
        for side in ("left", "right"):
            ankle = by_name.get(f"{side}_ankle")
            heel = by_name.get(f"{side}_heel")
            toe = by_name.get(f"{side}_big_toe")
            if not isinstance(ankle, Mapping) or not isinstance(heel, Mapping) or not isinstance(toe, Mapping):
                continue
            if not (ankle.get("pixel") and heel.get("pixel") and toe.get("pixel")):
                continue
            ax, ay = ankle["pixel"]  # type: ignore[index]
            hx, hy = heel["pixel"]  # type: ignore[index]
            tx, ty = toe["pixel"]  # type: ignore[index]
            separations.append(math.hypot(float(ax) - float(hx), float(ay) - float(hy)))
            separations.append(math.hypot(float(ax) - float(tx), float(ay) - float(ty)))
    if not separations:
        return None
    return sum(separations) / len(separations)


def score_scenario(scenario: str, records: Sequence[Mapping[str, object]]) -> Dict[str, object]:
    metrics: Dict[str, object] = {
        "airborne_foot_path_ratio": None,
        "airborne_foot_lag_frames": None,
        "airborne_foot_lag_seconds": None,
        "planted_foot_skate_distance_m": None,
        "root_jitter_inherited_from_foot_only_noise_m": None,
        "body_over_stance_root_displacement_preservation_ratio": None,
        "toe_pivot_toe_anchor_error_m": None,
        "heel_lock_heel_anchor_error_m": None,
        "flat_plant_anchor_error_m": None,
        "slip_release_snap_back_amount_m": None,
        "support_phase_flicker_count": 0,
        "support_phase_change_count": 0,
        "camera_disagreement_vs_3d_world_error": {},
        "low_res_heel_toe_ankle_mean_separation_px": None,
    }
    thresholds = {
        "airborne_foot_path_ratio_min": 0.75,
        "airborne_foot_lag_warn_frames": 1,
        "planted_foot_skate_warn_m": 0.08,
        "root_jitter_from_foot_noise_warn_m": 0.04,
        "body_over_stance_root_displacement_preservation_ratio_min": 0.85,
        "toe_pivot_toe_anchor_error_fail_m": 0.015,
        "heel_lock_heel_anchor_error_fail_m": 0.015,
        "flat_plant_anchor_error_fail_m": 0.020,
        "slip_release_snap_back_fail_m": 0.020,
        "support_phase_flicker_warn_count": 1,
        "camera_disagreement_bad_3d_fail_m": 0.75,
        "low_res_heel_toe_ankle_ambiguity_warn_px": 8.0,
    }
    reasons: List[str] = []
    status = "PASS"

    def warn(reason: str) -> None:
        nonlocal status
        if status == "PASS":
            status = "WARN"
        reasons.append(reason)

    def fail(reason: str) -> None:
        nonlocal status
        status = "FAIL"
        reasons.append(reason)

    phases_by_side = {side: [gt_foot_phase(record, side) for record in records] for side in ("left", "right")}
    phase_flickers = sum(phase_flicker_count(phases) for phases in phases_by_side.values())
    phase_changes = sum(
        1 for phases in phases_by_side.values() for previous, current in zip(phases, phases[1:]) if previous != current
    )
    metrics["support_phase_flicker_count"] = phase_flickers
    metrics["support_phase_change_count"] = phase_changes
    if phase_flickers > thresholds["support_phase_flicker_warn_count"]:
        warn(f"support phase flickered {phase_flickers} times")

    camera_metrics = camera_disagreement_metrics(records)
    metrics["camera_disagreement_vs_3d_world_error"] = camera_metrics
    if camera_metrics["high_camera_disagreement_predicts_bad_support_contact_decisions"]:
        fail(
            "high camera disagreement coincides with active support and bad 3D foot reconstruction; "
            "runtime support/contact must down-weight this evidence"
        )
    elif int(camera_metrics["high_camera_disagreement_keypoints"]) > 0:
        warn("camera disagreement is present and should be visible in runtime replay blame fields")

    # Generic planted skate metric: how far raw nominal-stereo planted foot centroids move while GT says planted.
    planted_skate: List[float] = []
    for side in ("left", "right"):
        planted_points = [
            reconstructed_foot_centroid(record, side)
            for record in records
            if gt_foot_planted(record, side) and gt_foot_phase(record, side) not in {"SWING", "SLIP"}
        ]
        skate = max_displacement3(planted_points)
        if skate is not None:
            planted_skate.append(skate)
    if planted_skate:
        metrics["planted_foot_skate_distance_m"] = rounded_metric(max(planted_skate), 5)

    if scenario in {"pure_airborne_leg_swing", "body_over_planted_stance", "heel_lock", "toe_pivot"}:
        sides = {
            str(record["ground_truth"].get("active_free_foot"))  # type: ignore[union-attr]
            for record in records
            if isinstance(record.get("ground_truth"), Mapping)
        }
        active_sides = sorted(side for side in sides if side in {"left", "right"})
        if active_sides:
            side = active_sides[0]
            gt_path_points = [gt_foot_centroid(record, side) for record in records]
            observed_path_points = [reconstructed_foot_centroid(record, side) for record in records]
            gt_path = path_length3(gt_path_points)
            observed_path = path_length3(observed_path_points)
            if gt_path is not None and gt_path > 1.0e-6 and observed_path is not None:
                ratio = observed_path / gt_path
                metrics["airborne_foot_path_ratio"] = rounded_metric(ratio, 4)
                if ratio < thresholds["airborne_foot_path_ratio_min"]:
                    fail(f"airborne {side} foot path ratio {ratio:.2f} indicates underprediction/holding")
            lag = best_lag_frames(gt_path_points, observed_path_points)
            if lag is not None:
                metrics["airborne_foot_lag_frames"] = lag
                metrics["airborne_foot_lag_seconds"] = rounded_metric(lag / DEFAULT_FPS, 4)
                if lag > thresholds["airborne_foot_lag_warn_frames"]:
                    warn(f"airborne {side} foot appears delayed by {lag} frames")

    if scenario == "planted_foot_jitter_from_2d_noise":
        root_jitter = max_displacement3([gt_root_position(record) for record in records]) or 0.0
        risk = float(metrics["planted_foot_skate_distance_m"] or 0.0) if root_jitter < 0.005 else 0.0
        metrics["root_jitter_inherited_from_foot_only_noise_m"] = rounded_metric(risk, 5)
        if risk > thresholds["root_jitter_from_foot_noise_warn_m"]:
            warn(
                f"raw planted-foot stereo skate is {risk:.3f}m while GT root is stationary; "
                "common-mode root correction must reject foot-only noise"
            )

    if scenario == "body_over_planted_stance":
        roots = [gt_root_position(record) for record in records]
        displacement = distance3(roots[0], roots[-1]) if roots else None
        expected_displacement = 0.28
        if displacement is not None:
            ratio = displacement / expected_displacement
            metrics["body_over_stance_root_displacement_preservation_ratio"] = rounded_metric(ratio, 4)
            if ratio < thresholds["body_over_stance_root_displacement_preservation_ratio_min"]:
                fail(f"body-over-stance root displacement ratio {ratio:.2f} is too low")

    if scenario == "toe_pivot":
        error = anchor_drift(records, "left", "toe_anchor", {"TOE_PIVOT"})
        metrics["toe_pivot_toe_anchor_error_m"] = rounded_metric(error)
        if error is not None and error > thresholds["toe_pivot_toe_anchor_error_fail_m"]:
            fail(f"toe pivot toe anchor drifted {error:.3f}m")

    if scenario == "heel_lock":
        error = anchor_drift(records, "left", "heel_anchor", {"HEEL_LOCK"})
        metrics["heel_lock_heel_anchor_error_m"] = rounded_metric(error)
        if error is not None and error > thresholds["heel_lock_heel_anchor_error_fail_m"]:
            fail(f"heel lock heel anchor drifted {error:.3f}m")

    if scenario == "flat_plant":
        errors = [
            anchor_drift(records, side, anchor_name, {"FLAT_PLANT"})
            for side in ("left", "right")
            for anchor_name in ("anchor", "heel_anchor", "toe_anchor")
        ]
        finite_errors = [error for error in errors if error is not None]
        flat_error = max(finite_errors) if finite_errors else None
        metrics["flat_plant_anchor_error_m"] = rounded_metric(flat_error)
        if flat_error is not None and flat_error > thresholds["flat_plant_anchor_error_fail_m"]:
            fail(f"flat-plant anchor drifted {flat_error:.3f}m")

    if scenario == "slip_release":
        snap = slip_release_snap_back_amount(records, "left")
        metrics["slip_release_snap_back_amount_m"] = rounded_metric(snap)
        if snap is not None and snap > thresholds["slip_release_snap_back_fail_m"]:
            fail(f"slip/release snapped back {snap:.3f}m toward stale anchor")

    if scenario == "mild_calibration_imperfection":
        max_world_error = float(camera_metrics.get("max_world_error_when_disagreed_m") or 0.0)
        # Calibration bias is not encoded as synthetic disagreement, so also inspect scenario summary below.
        try:
            stereo_world = max(
                float(record["diagnostics"]["stereo_blame"]["foot_summary"][side]["max_world_error_m"])  # type: ignore[index]
                for record in records
                for side in ("left", "right")
            )
        except (KeyError, TypeError, ValueError):
            stereo_world = max_world_error
        if stereo_world > 0.25:
            warn(
                f"mild calibration imperfection creates {stereo_world:.3f}m max foot world error; "
                "runtime replay must expose calibration/reprojection blame"
            )

    if scenario in {"one_camera_foot_occlusion", "both_camera_foot_occlusion"}:
        missing_triangulation = any(
            reconstructed_foot_centroid(record, "right") is None
            for record in records
        )
        if missing_triangulation:
            warn("right-foot stereo triangulation is unavailable in at least one occluded frame; fallback path must be validated")

    if scenario == "low_res_heel_toe_ankle_ambiguity":
        separation = low_res_ambiguity_metric(records)
        metrics["low_res_heel_toe_ankle_mean_separation_px"] = rounded_metric(separation, 3)
        if separation is not None and separation < thresholds["low_res_heel_toe_ankle_ambiguity_warn_px"]:
            warn(
                f"heel/toe/ankle mean separation is only {separation:.2f}px; "
                "support phase should not become overconfident or flicker"
            )

    return {
        "status": status,
        "reasons": reasons if reasons else ["synthetic trace preserves the expected lower-body invariant for this scenario"],
        "metrics": metrics,
        "thresholds": thresholds,
    }


def summarize_records(records: Sequence[Mapping[str, object]]) -> Dict[str, object]:
    scenario_summary: Dict[str, MutableMapping[str, object]] = {}
    scenario_records: Dict[str, List[Mapping[str, object]]] = {name: [] for name in SCENARIO_ORDER}

    for record in records:
        scenario = str(record["scenario"])
        scenario_records.setdefault(scenario, []).append(record)
        entry = scenario_summary.setdefault(scenario, {
            "frames": 0,
            "phases": {"left": {}, "right": {}},
            "min_camera_visible_foot_keypoints": {"camera_a": 99, "camera_b": 99},
            "max_camera_disagreement_px": 0.0,
            "max_stereo_reprojection_error_px": 0.0,
            "max_triangulated_foot_world_error_m": 0.0,
            "min_left_foot_triangulated": 99,
            "min_right_foot_triangulated": 99,
            "has_calibration_imperfection": False,
            "risks": SCENARIO_RISKS[scenario],
        })
        entry["frames"] = int(entry["frames"]) + 1
        gt = record["ground_truth"]  # type: ignore[index]
        for side in ("left", "right"):
            phase = gt[f"{side}_foot"]["phase"]  # type: ignore[index]
            phases = entry["phases"][side]  # type: ignore[index]
            phases[phase] = phases.get(phase, 0) + 1
        cameras = record["cameras"]  # type: ignore[index]
        for camera_name in ("camera_a", "camera_b"):
            metrics = record["diagnostics"]["camera_metrics"][camera_name]  # type: ignore[index]
            current_min = entry["min_camera_visible_foot_keypoints"][camera_name]  # type: ignore[index]
            entry["min_camera_visible_foot_keypoints"][camera_name] = min(current_min, metrics["visible_foot_keypoints"])  # type: ignore[index]
            if cameras[camera_name]["calibration_delta"]["imperfect"]:  # type: ignore[index]
                entry["has_calibration_imperfection"] = True
            for kp in cameras[camera_name]["keypoints"]:  # type: ignore[index]
                entry["max_camera_disagreement_px"] = max(float(entry["max_camera_disagreement_px"]), float(kp["disagreement_px"]))
        stereo = record["diagnostics"]["stereo_blame"]  # type: ignore[index]
        entry["max_stereo_reprojection_error_px"] = max(
            float(entry["max_stereo_reprojection_error_px"]),
            float(stereo["summary"]["max_reprojection_error_px"]))  # type: ignore[index]
        for side in ("left", "right"):
            foot = stereo["foot_summary"][side]  # type: ignore[index]
            entry[f"min_{side}_foot_triangulated"] = min(
                int(entry[f"min_{side}_foot_triangulated"]),
                int(foot["triangulated"]))
            entry["max_triangulated_foot_world_error_m"] = max(
                float(entry["max_triangulated_foot_world_error_m"]),
                float(foot["max_world_error_m"]))

    status_counts = {"PASS": 0, "WARN": 0, "FAIL": 0}
    for scenario, scenario_entry in scenario_summary.items():
        regression = score_scenario(scenario, scenario_records.get(scenario, []))
        scenario_entry["regression"] = regression
        status = str(regression["status"])
        status_counts[status] = status_counts.get(status, 0) + 1

    return {
        "schema_version": SCHEMA_VERSION,
        "records": len(records),
        "scenarios": scenario_summary,
        "regression_status_counts": status_counts,
        "diagnostic_intent": {
            "not_runtime_proof": True,
            "purpose": "Feed or inspect traces to identify whether runtime debug/replay data can assign blame for bad lower-body/contact frames.",
            "primary_failure_modes": sorted({risk for risks in SCENARIO_RISKS.values() for risk in risks}),
            "scoring_note": (
                "Regression scoring compares synthetic ground truth with the noisy/occluded nominal-stereo "
                "diagnostic reconstruction. FAIL/WARN means the trace would expose a lower-body failure if "
                "runtime output follows the bad evidence or violates the scenario invariant; it is not a full "
                "live-runtime verdict."
            ),
        },
    }

def svg_escape(text: str) -> str:
    return (
        text.replace("&", "&amp;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
        .replace('"', "&quot;")
    )


def keypoints_by_name(camera_record: Mapping[str, object]) -> Dict[str, Mapping[str, object]]:
    return {str(kp["name"]): kp for kp in camera_record["keypoints"]}  # type: ignore[index]


def draw_camera_panel(record: Mapping[str, object], camera_name: str, x0: float, y0: float,
                      width: int, height: int) -> List[str]:
    camera = record["cameras"][camera_name]  # type: ignore[index]
    kps = keypoints_by_name(camera)  # type: ignore[arg-type]
    parts: List[str] = []
    parts.append(f'<g transform="translate({x0:.1f},{y0:.1f})">')
    parts.append(f'<rect x="0" y="0" width="{width}" height="{height}" fill="#f8f8f8" stroke="#222" stroke-width="1"/>')
    parts.append(f'<text x="4" y="12" font-size="10" font-family="monospace">{svg_escape(camera_name)} frame {record["scenario_frame_index"]}</text>')

    # Draw low-res pixel grid hints every 48 px.
    for gx in range(48, width, 48):
        parts.append(f'<line x1="{gx}" y1="0" x2="{gx}" y2="{height}" stroke="#ddd" stroke-width="0.5"/>')
    for gy in range(48, height, 48):
        parts.append(f'<line x1="0" y1="{gy}" x2="{width}" y2="{gy}" stroke="#ddd" stroke-width="0.5"/>')

    for point in camera["chessboard"]:  # type: ignore[index]
        if not point["visible"] or point["pixel"] is None:
            continue
        u, v = point["pixel"]
        fill = "#222" if point["black"] else "#fff"
        parts.append(f'<rect x="{u - 2:.2f}" y="{v - 2:.2f}" width="4" height="4" fill="{fill}" stroke="#555" stroke-width="0.4"/>')

    for a, b in SKELETON_EDGES:
        pa = kps.get(a)
        pb = kps.get(b)
        if not pa or not pb or not pa.get("visible") or not pb.get("visible"):
            continue
        au, av = pa["pixel"]  # type: ignore[index]
        bu, bv = pb["pixel"]  # type: ignore[index]
        parts.append(f'<line x1="{au:.2f}" y1="{av:.2f}" x2="{bu:.2f}" y2="{bv:.2f}" stroke="#3b6ea8" stroke-width="1.6" opacity="0.85"/>')

    for name, kp in kps.items():
        if not kp.get("visible"):
            continue
        u, v = kp["pixel"]  # type: ignore[index]
        conf = float(kp["confidence"])
        radius = 2.0 + 2.0 * conf
        fill = "#c0392b" if name in FOOT_KEYPOINTS["left"] or name in FOOT_KEYPOINTS["right"] else "#1f7a3a"
        if kp.get("low_res_ambiguous"):
            fill = "#8e44ad"
        if float(kp.get("disagreement_px", 0.0)) > 0.0:
            fill = "#d35400"
        parts.append(f'<circle cx="{u:.2f}" cy="{v:.2f}" r="{radius:.2f}" fill="{fill}" stroke="#111" stroke-width="0.4" opacity="0.92"/>')

    # Mark explicitly occluded foot keypoints at their ground-truth location with a hollow cross.
    for name, kp in kps.items():
        if not kp.get("occluded") or kp.get("ground_truth_pixel") is None:
            continue
        u, v = kp["ground_truth_pixel"]  # type: ignore[index]
        parts.append(f'<line x1="{u - 4:.2f}" y1="{v - 4:.2f}" x2="{u + 4:.2f}" y2="{v + 4:.2f}" stroke="#000" stroke-width="1"/>')
        parts.append(f'<line x1="{u + 4:.2f}" y1="{v - 4:.2f}" x2="{u - 4:.2f}" y2="{v + 4:.2f}" stroke="#000" stroke-width="1"/>')

    parts.append("</g>")
    return parts


def write_svg(records: Sequence[Mapping[str, object]], out_path: Path, width: int, height: int) -> None:
    selected: List[Mapping[str, object]] = []
    seen = set()
    # Choose a representative mid-frame for every scenario, regardless of sample length.
    for record in records:
        scenario = record["scenario"]
        if scenario not in seen and float(record.get("scenario_t", 0.0)) >= 0.5:
            selected.append(record)
            seen.add(scenario)
    # Fallback for small frame counts.
    for record in records:
        if record["scenario"] not in seen:
            selected.append(record)
            seen.add(record["scenario"])

    margin = 12
    panel_gap = 12
    row_gap = 36
    total_width = 2 * width + panel_gap + 2 * margin
    total_height = len(selected) * (height + row_gap) + 2 * margin
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{total_width}" height="{total_height}" viewBox="0 0 {total_width} {total_height}">',
        '<rect x="0" y="0" width="100%" height="100%" fill="#ffffff"/>',
        '<text x="12" y="18" font-size="14" font-family="monospace">Synthetic stereo diagnostics: low-res camera views with chessboard + projected HALPE-26 keypoints</text>',
    ]

    y = margin + 18
    for record in selected:
        scenario = str(record["scenario"])
        desc = SCENARIO_DESCRIPTIONS[scenario]
        phase_l = record["ground_truth"]["left_foot"]["phase"]  # type: ignore[index]
        phase_r = record["ground_truth"]["right_foot"]["phase"]  # type: ignore[index]
        label = f'{scenario}: L={phase_l} R={phase_r} — {desc}'
        parts.append(f'<text x="{margin}" y="{y + 13}" font-size="11" font-family="monospace">{svg_escape(label)}</text>')
        row_y = y + 20
        parts.extend(draw_camera_panel(record, "camera_a", margin, row_y, width, height))
        parts.extend(draw_camera_panel(record, "camera_b", margin + width + panel_gap, row_y, width, height))
        y += height + row_gap

    parts.append("</svg>")
    out_path.write_text("\n".join(parts), encoding="utf-8")


def write_manifest(out_dir: Path, records: Sequence[Mapping[str, object]], width: int, height: int,
                   frames_per_scenario: int, seed: int) -> None:
    manifest = {
        "schema_version": SCHEMA_VERSION,
        "generator": "tools/synthetic_stereo_diagnostic.py",
        "seed": seed,
        "fps": DEFAULT_FPS,
        "image_size": [width, height],
        "frames_per_scenario": frames_per_scenario,
        "total_frames": len(records),
        "trace": "synthetic_stereo_trace.ndjson",
        "summary": "diagnostic_summary.json",
        "camera_views_svg": "camera_views.svg",
        "scenarios": [
            {
                "name": name,
                "description": SCENARIO_DESCRIPTIONS[name],
                "risks": SCENARIO_RISKS[name],
            }
            for name in SCENARIO_ORDER
        ],
        "chessboard": {
            "rows": 5,
            "cols": 7,
            "square_m": 0.09,
            "world_points": board_world_points(),
        },
        "notes": [
            "This trace does not prove runtime correctness; it exposes failure modes and missing blame fields.",
            "Camera keypoints include both ground_truth_pixel and noisy/visible pixel.",
            "Mild calibration imperfection stores nominal and actual projection cameras per frame.",
            "Two-camera disagreement intentionally keeps camera-B right-foot confidence high.",
            "diagnostic_summary.json contains scenario-level PASS/WARN/FAIL regression scoring.",
        ],
    }
    (out_dir / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")


def generate_diagnostics(out_dir: Path, frames_per_scenario: int = 18,
                         width: int = DEFAULT_WIDTH, height: int = DEFAULT_HEIGHT,
                         seed: int = 7) -> None:
    if frames_per_scenario < 2:
        raise ValueError("frames_per_scenario must be at least 2")
    if width < 160 or height < 120:
        raise ValueError("image size is too small for useful low-res diagnostics")

    out_dir.mkdir(parents=True, exist_ok=True)
    records = generate_records(frames_per_scenario, width, height, seed)

    trace_path = out_dir / "synthetic_stereo_trace.ndjson"
    with trace_path.open("w", encoding="utf-8") as f:
        for record in records:
            f.write(json.dumps(record, separators=(",", ":")) + "\n")

    summary = summarize_records(records)
    (out_dir / "diagnostic_summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    write_manifest(out_dir, records, width, height, frames_per_scenario, seed)
    write_svg(records, out_dir / "camera_views.svg", width, height)


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=Path("diagnostics/synthetic_stereo_example"),
                        help="Output directory for JSONL/SVG diagnostic artifacts.")
    parser.add_argument("--frames-per-scenario", type=int, default=18,
                        help="Number of frames per scenario.")
    parser.add_argument("--width", type=int, default=DEFAULT_WIDTH,
                        help="Synthetic camera width in pixels.")
    parser.add_argument("--height", type=int, default=DEFAULT_HEIGHT,
                        help="Synthetic camera height in pixels.")
    parser.add_argument("--seed", type=int, default=7,
                        help="Deterministic random seed.")
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    generate_diagnostics(args.out, args.frames_per_scenario, args.width, args.height, args.seed)
    print(f"Wrote synthetic stereo diagnostics to {args.out}")
    print(f"  trace:   {args.out / 'synthetic_stereo_trace.ndjson'}")
    print(f"  summary: {args.out / 'diagnostic_summary.json'}")
    print(f"  views:   {args.out / 'camera_views.svg'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
