#!/usr/bin/env python3
"""Static live-bringup readiness checks.

These tests keep operator-facing docs and UI labels aligned with the runtime
fields added for model, camera, foot-contact, root-correction, and replay bring-up.
They intentionally avoid building the app or launching WebView/OpenCV.
"""

from __future__ import annotations

import json
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
EXPECTED_MODEL = "models/rtmw-dw-x-l-cocktail14-384x288.onnx"
EXPECTED_SHA = "bd033156e5104c4f5d2edfe0453e02661e30a2f3da453ec93c8764d561b83054"
EXPECTED_DEPTH_MODEL = "models/rtmw3d-x-cocktail14-384x288.onnx"
EXPECTED_DEPTH_SHA = "4a289c0e99d47eb595e99679d9d4a2d1def1b4241f9adcbafba44b9ff585ebcd"


class LiveBringupReadinessTest(unittest.TestCase):

    def test_bringup_guide_names_current_model_and_sha_check(self) -> None:
        guide = (REPO_ROOT / "docs" / "RUNTIME_BRINGUP.md").read_text(encoding="utf-8")
        model_sha_file = (REPO_ROOT / "models" / "rtmw-dw-x-l-cocktail14-384x288.onnx.sha256").read_text(encoding="utf-8")
        depth_model_sha_file = (REPO_ROOT / "models" / "rtmw3d-x-cocktail14-384x288.onnx.sha256").read_text(encoding="utf-8")

        self.assertIn(EXPECTED_MODEL, guide)
        self.assertIn(EXPECTED_SHA, guide)
        self.assertIn(EXPECTED_DEPTH_MODEL, guide)
        self.assertIn(EXPECTED_DEPTH_SHA, guide)
        self.assertIn(EXPECTED_SHA, model_sha_file)
        self.assertIn(EXPECTED_DEPTH_SHA, depth_model_sha_file)
        self.assertIn("Get-FileHash", guide)
        self.assertIn("RTMW-DW-X-L Cocktail14 384x288", guide)
        self.assertIn("calculated 3D", guide)
        self.assertNotIn("RTMPose-l", guide)
        self.assertNotIn("RTMPOSE-L", guide)

    def test_bringup_guide_covers_operator_workflow_and_failure_triage(self) -> None:
        guide = (REPO_ROOT / "docs" / "RUNTIME_BRINGUP.md").read_text(encoding="utf-8")

        required_sections = (
            "Preflight doctor",
            "First 10 minutes checklist",
            "Camera setup",
            "Chessboard capture and calibration",
            "Monocular no-chessboard setup",
            "First launch",
            "Confirm the model loaded",
            "Confirm cameras are paired",
            "Confirm tracking is live",
            "How to read the foot and root cards",
            "Recording a replay",
            "If feet look wrong, check this in order",
            "Common failure modes",
            "Config defaults to know",
        )
        for section in required_sections:
            self.assertIn(section, guide)

        required_runtime_fields = (
            "frame_pairing.accepted_pairs",
            "frame_pairing.last_skew_ms",
            "support.left_foot.phase",
            "support.right_foot.phase",
            "support.*.contact_residual",
            "tracking_mode",
            "depth_source",
            "floor_assist.status",
            "solver.depth.source",
            "solver.depth.floor_assist",
            "solver.triangulation.preliminary.left_foot_contact_confidence",
            "solver.triangulation.preliminary.foot_mean_reprojection_error_px",
            "solver.triangulation.preliminary.max_foot_reprojection_error_px",
            "motion_filter.contact_root.reason",
            "motion_filter.contact_root.correction_m",
            "motion_filter.contact_root.common_residual_m",
            "motion_filter.contact_root.foot_disagreement_m",
            "motion_filter.contact_root.root_alignment",
            "support.left_foot.support_confidence",
            "tracking.enable_replay_recording",
            "debug.replay_log_path",
            "docs/SYNTHETIC_STEREO_DIAGNOSTICS.md",
            "tools\\live_preflight_doctor.py",
        )
        for field in required_runtime_fields:
            self.assertIn(field, guide)

        stale_fields = (
            "motion_filter.contact_root.amount_m",
            "motion_filter.contact_root.residual_m",
            "motion_filter.contact_root.disagreement_m",
            "motion_filter.contact_root.alignment",
            "motion_filter.contact_root.support_confidence",
            "solver.triangulation.preliminary.max_reprojection_error_px",
            "tracking.recording_dir/latest-runtime.ndjson",
        )
        for field in stale_fields:
            self.assertNotIn(field, guide)

    def test_operator_facing_text_has_no_stale_rtmpose_l_label(self) -> None:
        paths = (
            REPO_ROOT / "README.md",
            REPO_ROOT / "docs" / "RUNTIME_BRINGUP.md",
            REPO_ROOT / "src" / "main.cpp",
            REPO_ROOT / "src" / "ui" / "app" / "index.html",
        )
        for path in paths:
            text = path.read_text(encoding="utf-8")
            self.assertNotIn("RTMPose-l", text, msg=str(path))
            self.assertNotIn("RTMPOSE-L", text, msg=str(path))
            self.assertNotIn("rtmpose-l", text, msg=str(path))

    def test_desktop_labels_are_plain_and_match_bringup_guide(self) -> None:
        index_html = (REPO_ROOT / "src" / "ui" / "app" / "index.html").read_text(encoding="utf-8")
        app_js = (REPO_ROOT / "src" / "ui" / "app" / "app.js").read_text(encoding="utf-8")
        guide = (REPO_ROOT / "docs" / "RUNTIME_BRINGUP.md").read_text(encoding="utf-8")

        self.assertNotIn("Local web UI", index_html)
        self.assertNotIn("Old web UI", index_html)
        self.assertIn(EXPECTED_MODEL, index_html)
        self.assertIn(EXPECTED_DEPTH_MODEL, index_html)
        self.assertNotIn('"webToggle"', app_js)
        self.assertIn("foot contact", index_html.lower())
        self.assertIn("root correction", index_html.lower())

    def test_default_config_paths_match_bringup_guide(self) -> None:
        default_config = json.loads((REPO_ROOT / "config" / "default.json").read_text(encoding="utf-8"))
        guide = (REPO_ROOT / "docs" / "RUNTIME_BRINGUP.md").read_text(encoding="utf-8")

        self.assertEqual(EXPECTED_MODEL, default_config["tracking"]["model_path"])
        self.assertFalse(default_config["tracking"]["depth_postprocess_enabled"])
        self.assertEqual(EXPECTED_DEPTH_MODEL, default_config["tracking"]["depth_postprocess_model_path"])
        self.assertEqual(4, default_config["tracking"]["depth_postprocess_interval_frames"])
        self.assertFalse(default_config["tracking"]["depth_postprocess_allow_cpu_fallback"])
        self.assertEqual("monocular", default_config["tracking"]["mode"])
        self.assertEqual("calib/default.json", default_config["tracking"]["calibration_path"])
        self.assertFalse(default_config["tracking"]["enable_replay_recording"])
        self.assertFalse(default_config["osc"]["enabled"])
        self.assertNotIn("web_ui", default_config)
        self.assertFalse(default_config["osc"]["tracker_space_transform_valid"])

        for value in (
            default_config["tracking"]["model_path"],
            default_config["tracking"]["depth_postprocess_model_path"],
            default_config["tracking"]["calibration_path"],
            default_config["app"]["recording_dir"],
        ):
            self.assertIn(value, guide)
        self.assertIn("app.recording_dir/latest-runtime.ndjson", guide)


    def test_bringup_guide_documents_monocular_no_chessboard_path(self) -> None:
        guide = (REPO_ROOT / "docs" / "RUNTIME_BRINGUP.md").read_text(encoding="utf-8")

        for required in (
            "tracking.mode=monocular",
            "No chessboard or Camera B is required for monocular bring-up",
            "Monocular no-chessboard setup",
            "Guided floor-scale assist",
            "floor_assist.status",
            "depth_source=inferred_monocular",
            "missing or bad stereo calibration file does not block monocular runtime startup",
        ):
            self.assertIn(required, guide)


if __name__ == "__main__":
    unittest.main()

