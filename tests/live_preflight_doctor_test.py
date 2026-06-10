#!/usr/bin/env python3
"""Static tests for tools/live_preflight_doctor.py.

These tests use tiny temporary repo fixtures and intentionally avoid CMake,
OpenCV, ONNX Runtime, cameras, WebView2, or vcpkg setup.
"""

from __future__ import annotations

import hashlib
import importlib.util
import json
import os
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
TOOL_PATH = REPO_ROOT / "tools" / "live_preflight_doctor.py"
EXPECTED_MODEL = "models/rtmw-dw-x-l-cocktail14-384x288.onnx"
EXPECTED_DEPTH_MODEL = "models/rtmw3d-x-cocktail14-384x288.onnx"


def load_doctor():
    spec = importlib.util.spec_from_file_location("live_preflight_doctor", TOOL_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


DOCTOR = load_doctor()


class LivePreflightDoctorTest(unittest.TestCase):

    def make_repo(
        self,
        tmp: Path,
        *,
        model_present: bool = True,
        sha_matches: bool = True,
        tracking_ready: bool = True,
        stale_reference: bool = False,
        same_camera_index: bool = False,
        create_recording_dir: bool = True,
        replay_recording_enabled: bool = False,
        tracking_model_path: str = EXPECTED_MODEL,
        inference_model_path: str | None = None,
        tracking_mode: str = "stereo",
        calibration_present: bool = True,
        camera_b_present: bool = True,
        monocular_overrides: dict | None = None,
    ) -> Path:
        root = tmp / "repo"
        (root / "config").mkdir(parents=True)
        (root / "models").mkdir()
        (root / "calib").mkdir()
        (root / "docs").mkdir()
        (root / "src" / "ui" / "app").mkdir(parents=True)
        if create_recording_dir:
            (root / "recordings").mkdir()
        (root / "vcpkg").mkdir()

        monocular = {
            "image_width": 1280,
            "image_height": 720,
            "horizontal_fov_deg": 70.0,
            "user_height_m": 1.70,
            "camera_height_m": 1.20,
            "default_depth_m": 2.20,
            "depth_confidence_scale": 0.55,
            "min_keypoint_confidence": 0.05,
            "min_seed_count": 4,
            "floor_scale_assist_enabled": False,
            "floor_depth_line_spacing_m": 0.0,
            "floor_depth_line_spacing_px": 0.0,
            "floor_depth_reference_y_px": 0.0,
            "floor_depth_reference_m": 0.0,
            "floor_depth_confidence": 0.65,
        }
        if monocular_overrides:
            monocular.update(monocular_overrides)

        config = {
            "app": {"recording_dir": "recordings"},
            "tracking": {
                "mode": tracking_mode,
                "model_path": tracking_model_path,
                "depth_postprocess_enabled": False,
                "depth_postprocess_model_path": EXPECTED_DEPTH_MODEL,
                "depth_postprocess_interval_frames": 4,
                "depth_postprocess_allow_cpu_fallback": False,
                "calibration_path": "calib/default.json",
                "enable_replay_recording": replay_recording_enabled,
                "monocular": monocular,
            },
            "inference": {"device": "cpu"},
            "debug": {"replay_log_path": ""},
            "camera_a": {"device_index": 0, "width": 1280, "height": 720, "fps": 60},
            "osc": {"enabled": False},
        }
        if inference_model_path is not None:
            config["inference"]["model_path"] = inference_model_path
        if camera_b_present:
            config["camera_b"] = {"device_index": 0 if same_camera_index else 1, "width": 1280, "height": 720, "fps": 60}
        (root / "config" / "default.json").write_text(json.dumps(config), encoding="utf-8")

        if calibration_present:
            calibration = {
                "tracking_ready": tracking_ready,
                "floor_plane": {"valid": tracking_ready},
                "camera_a": {"intrinsics_valid": tracking_ready, "extrinsics_valid": tracking_ready},
                "camera_b": {"intrinsics_valid": tracking_ready, "extrinsics_valid": tracking_ready},
            }
            (root / "calib" / "default.json").write_text(json.dumps(calibration), encoding="utf-8")

        model_bytes = b"synthetic onnx bytes for preflight unit test"
        expected_sha = hashlib.sha256(model_bytes).hexdigest()
        if model_present:
            (root / EXPECTED_MODEL).write_bytes(model_bytes)
            (root / EXPECTED_DEPTH_MODEL).write_bytes(model_bytes)
        sidecar_sha = expected_sha if sha_matches else "0" * 64
        (root / f"{EXPECTED_MODEL}.sha256").write_text(f"{sidecar_sha}  {EXPECTED_MODEL}\n", encoding="utf-8")
        (root / f"{EXPECTED_DEPTH_MODEL}.sha256").write_text(f"{sidecar_sha}  {EXPECTED_DEPTH_MODEL}\n", encoding="utf-8")

        operator_text = (
            f"RTMPose-L stale label {EXPECTED_MODEL} {expected_sha}"
            if stale_reference else
            f"RTMW-DW-X-L Cocktail14 384x288 {EXPECTED_MODEL} {expected_sha} RTMW3D-X Cocktail14 384x288 {EXPECTED_DEPTH_MODEL}"
        )
        for rel in (
            "README.md",
            "docs/BUILD_ENVIRONMENT.md",
            "docs/RUNTIME_BRINGUP.md",
            "AGENTS.md",
            "docs/BUREAUCRAT_LOGIC.md",
            "docs/SYNTHETIC_STEREO_DIAGNOSTICS.md",
            "models/README.md",
            "src/ui/app/index.html",
            "src/ui/app/app.js",
            "src/main.cpp",
        ):
            path = root / rel
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(operator_text, encoding="utf-8")
        (root / "vcpkg.json").write_text("{}", encoding="utf-8")
        return root

    def report(self, root: Path) -> dict:
        return DOCTOR.build_report(root, env={"VCPKG_ROOT": str(root / "vcpkg")})

    def by_id(self, report: dict, check_id: str) -> list[dict]:
        return [item for item in report["results"] if item["id"] == check_id]

    def test_doctor_passes_when_required_files_are_present_and_ready(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            root = self.make_repo(Path(td))
            report = self.report(root)
            self.assertEqual(0, report["summary"]["fail"], json.dumps(report, indent=2))
            self.assertTrue(report["summary"]["ready"])
            for check_id in ("tracking.mode", "model.file", "model.sha256", "model.depth_postprocess_config", "model.depth_postprocess.file", "model.depth_postprocess.sha256", "stereo.calibration.tracking_ready", "stereo.camera.indices", "stereo.monocular_fallback.profile"):
                self.assertEqual("PASS", self.by_id(report, check_id)[0]["severity"])
            self.assertEqual("INFO", self.by_id(report, "stereo.monocular_fallback.floor_scale_assist")[0]["severity"])

    def test_doctor_fails_when_model_file_is_missing(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            root = self.make_repo(Path(td), model_present=False)
            report = self.report(root)
            self.assertFalse(report["summary"]["ready"])
            model_file = self.by_id(report, "model.file")[0]
            self.assertEqual("FAIL", model_file["severity"])
            self.assertIn(EXPECTED_MODEL, model_file["detail"])

    def test_doctor_separates_wrong_config_path_from_present_expected_model(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            root = self.make_repo(Path(td), tracking_model_path="models/wrong.onnx")
            report = self.report(root)
            config_path = self.by_id(report, "model.config_path")[0]
            model_file = self.by_id(report, "model.file")[0]
            self.assertEqual("FAIL", config_path["severity"])
            self.assertEqual("PASS", model_file["severity"])
            self.assertIn("tracking.model_path='models/wrong.onnx'", config_path["detail"])

    def test_recording_dir_missing_is_only_warn_when_recording_enabled(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            root = self.make_repo(Path(td), create_recording_dir=False, replay_recording_enabled=False)
            report = self.report(root)
            directory = self.by_id(report, "replay.recording_dir")[0]
            self.assertEqual("INFO", directory["severity"])
            self.assertIn("No action until replay recording is enabled", directory["action"])

        with tempfile.TemporaryDirectory() as td:
            root = self.make_repo(Path(td), create_recording_dir=False, replay_recording_enabled=True)
            report = self.report(root)
            directory = self.by_id(report, "replay.recording_dir")[0]
            self.assertEqual("WARN", directory["severity"])
            self.assertIn("Create recordings", directory["action"])

    def test_doctor_warns_when_calibration_tracking_ready_is_false(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            root = self.make_repo(Path(td), tracking_ready=False)
            report = self.report(root)
            ready = self.by_id(report, "stereo.calibration.tracking_ready")[0]
            components = self.by_id(report, "stereo.calibration.components")[0]
            self.assertEqual("WARN", ready["severity"])
            self.assertEqual("WARN", components["severity"])
            self.assertIn("--status", ready["action"])

    def test_doctor_catches_stale_rtmpose_l_operator_reference(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            root = self.make_repo(Path(td), stale_reference=True)
            report = self.report(root)
            stale = self.by_id(report, "docs.stale_rtmpose_l")[0]
            self.assertEqual("FAIL", stale["severity"])
            self.assertIn("RTMPose-L", stale["title"])

    def test_doctor_catches_stale_model_sha_references_in_operator_text(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            root = self.make_repo(Path(td))
            index = root / "src" / "ui" / "app" / "index.html"
            index.write_text(f"RTMW3D-X Cocktail14 384x288 {EXPECTED_MODEL} {'0' * 64}", encoding="utf-8")
            report = self.report(root)
            stale = self.by_id(report, "docs.model_sha_reference")[0]
            self.assertEqual("FAIL", stale["severity"])
            self.assertIn("src/ui/app/index.html", stale["detail"])

    def test_json_output_contains_stable_result_fields(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            root = self.make_repo(Path(td), tracking_ready=False)
            completed = subprocess.run(
                [sys.executable, "-S", str(TOOL_PATH), "--repo-root", str(root), "--json"],
                env={**os.environ, "VCPKG_ROOT": str(root / "vcpkg")},
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=False,
            )
            self.assertEqual(0, completed.returncode, completed.stderr + completed.stdout)
            parsed = json.loads(completed.stdout)
            self.assertIn("summary", parsed)
            self.assertIn("results", parsed)
            self.assertEqual("stereo", parsed["tracking_mode"])
            self.assertGreater(len(parsed["results"]), 0)
            for item in parsed["results"]:
                self.assertIn(item["severity"], {"PASS", "WARN", "FAIL", "INFO"})
                for key in ("id", "severity", "title", "detail", "action"):
                    self.assertIn(key, item)
                    self.assertIsInstance(item[key], str)
                    self.assertTrue(item[key])

    def test_monocular_mode_does_not_require_camera_b_or_stereo_calibration(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            root = self.make_repo(
                Path(td),
                tracking_mode="monocular",
                calibration_present=False,
                camera_b_present=False,
            )
            report = self.report(root)
            self.assertEqual(0, report["summary"]["fail"], json.dumps(report, indent=2))
            self.assertTrue(report["summary"]["ready"])
            self.assertEqual("monocular", report["tracking_mode"])
            self.assertEqual([], self.by_id(report, "stereo.calibration.file"))
            self.assertEqual("PASS", self.by_id(report, "monocular.camera_a.mode")[0]["severity"])
            self.assertEqual("PASS", self.by_id(report, "monocular.scale_profile")[0]["severity"])
            self.assertEqual("PASS", self.by_id(report, "monocular.metric_scale_source")[0]["severity"])
            self.assertEqual("INFO", self.by_id(report, "monocular.stereo_calibration.skipped")[0]["severity"])
            self.assertEqual("INFO", self.by_id(report, "monocular.camera_b.ignored")[0]["severity"])

    def test_monocular_mode_warns_about_weak_scale_inputs(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            root = self.make_repo(
                Path(td),
                tracking_mode="monocular",
                monocular_overrides={
                    "user_height_m": 0.0,
                    "camera_height_m": 0.0,
                    "horizontal_fov_deg": 0.0,
                    "floor_scale_assist_enabled": True,
                    "floor_depth_line_spacing_m": 0.30,
                    "floor_depth_line_spacing_px": 0.0,
                },
            )
            report = self.report(root)
            self.assertFalse(report["summary"]["ready"])
            self.assertEqual("WARN", self.by_id(report, "monocular.user_height")[0]["severity"])
            self.assertEqual("WARN", self.by_id(report, "monocular.camera_height")[0]["severity"])
            self.assertEqual("WARN", self.by_id(report, "monocular.horizontal_fov")[0]["severity"])
            self.assertEqual("WARN", self.by_id(report, "monocular.floor_scale_assist")[0]["severity"])
            self.assertEqual("FAIL", self.by_id(report, "monocular.metric_scale_source")[0]["severity"])

    def test_stereo_fallback_warns_when_single_camera_profile_is_weak(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            root = self.make_repo(
                Path(td),
                tracking_mode="stereo",
                monocular_overrides={
                    "horizontal_fov_deg": 0.0,
                    "user_height_m": 0.0,
                    "camera_height_m": 0.0,
                    "default_depth_m": 0.0,
                    "floor_scale_assist_enabled": True,
                    "floor_depth_line_spacing_m": 0.30,
                    "floor_depth_line_spacing_px": 0.0,
                },
            )
            report = self.report(root)
            self.assertEqual("WARN", self.by_id(report, "stereo.monocular_fallback.profile")[0]["severity"])
            self.assertEqual("WARN", self.by_id(report, "stereo.monocular_fallback.floor_scale_assist")[0]["severity"])

    def test_invalid_tracking_mode_is_a_fail(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            root = self.make_repo(Path(td), tracking_mode="dual_fisheye")
            report = self.report(root)
            mode = self.by_id(report, "tracking.mode")[0]
            self.assertEqual("FAIL", mode["severity"])
            self.assertIn("stereo", mode["action"])
            self.assertIn("monocular", mode["action"])

    def test_current_repo_default_reports_model_binary_state(self) -> None:
        report = DOCTOR.build_report(REPO_ROOT, env={"VCPKG_ROOT": str(REPO_ROOT)})
        self.assertGreaterEqual(report["summary"]["result_count"], 1)
        model_files = self.by_id(report, "model.file")
        self.assertEqual(1, len(model_files))
        if not (REPO_ROOT / EXPECTED_MODEL).exists():
            self.assertEqual("FAIL", model_files[0]["severity"])
            self.assertIn("Place the ONNX model", model_files[0]["action"])
        else:
            self.assertEqual("PASS", model_files[0]["severity"])

    def test_camera_index_collision_is_a_fail(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            root = self.make_repo(Path(td), same_camera_index=True)
            report = self.report(root)
            indices = self.by_id(report, "stereo.camera.indices")[0]
            self.assertEqual("FAIL", indices["severity"])
            self.assertIn("different camera indices", indices["action"])


if __name__ == "__main__":
    unittest.main()

