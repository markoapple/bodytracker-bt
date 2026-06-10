#!/usr/bin/env python3
"""Logic smokes for tools/visual_eval_image.py.

These stay image/network/runtime-free: they lock the evidence-honesty rules that
previously let manual/synthetic debug artifacts look stronger than they were.
"""
from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "tools" / "visual_eval_image.py"
spec = importlib.util.spec_from_file_location("visual_eval_image", MODULE_PATH)
visual_eval = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = visual_eval
assert spec.loader is not None
spec.loader.exec_module(visual_eval)


class VisualEvalImageLogicTest(unittest.TestCase):
    def test_default_selector_runs_negative_control_first(self) -> None:
        self.assertEqual(
            visual_eval.DEFAULT_SELECTION_ORDER[0],
            visual_eval.PREVIOUS_BAD_STAGE.candidate_id,
        )
        self.assertIn(visual_eval.SELECTED_PEXELS.candidate_id, visual_eval.DEFAULT_SELECTION_ORDER)

    def test_manual_synthetic_can_never_pass_and_raw_scores_stay_raw(self) -> None:
        preflight = {
            "accepted": True,
            "metrics": {
                "backend_metric_scale_confidence": 1.0,
                "floor_coverage": 1.0,
                "pattern_strength": 1.0,
                "line_purity": 1.0,
                "perspective_coherence": 1.0,
            },
        }
        floor_report = {"detection_debug": {"calibration": {"valid": True}}}
        checks = {
            "left_leg": {
                "hip_to_knee_alignment": 1.0,
                "knee_on_visible_joint": 1.0,
                "knee_to_foot_alignment": 1.0,
                "foot_on_visible_contact": 1.0,
            },
            "right_leg": {
                "hip_to_knee_alignment": 1.0,
                "knee_on_visible_joint": 1.0,
                "knee_to_foot_alignment": 1.0,
                "foot_on_visible_contact": 1.0,
            },
        }
        trackers = {
            "left foot": (10, 10),
            "right foot": (30, 10),
            "pelvis/waist": (20, 0),
        }
        boxes = {
            "left_foot": (0, 0, 20, 20),
            "right_foot": (20, 0, 40, 20),
            "pelvis": (10, -10, 30, 10),
        }
        scoring = visual_eval.compute_scores(
            preflight,
            floor_report,
            checks,
            trackers,
            boxes,
            pose_source="manual_keypoints",
            tracker_space_source="synthetic",
        )
        self.assertEqual(scoring["verdict"], "partial")
        self.assertLess(scoring["overall"], 0.95)
        self.assertEqual(scoring["raw_scores"]["body_outline_plausibility"], 1.0)
        self.assertLess(scoring["capped_scores"]["body_outline_plausibility"], 1.0)
        self.assertIn("debug-only evidence cannot produce a pass verdict", scoring["flags"])

    def test_blocked_report_records_no_runtime_paths(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)
            visual_eval.write_blocked_report(
                out,
                {
                    "accepted": False,
                    "selected_image_url": "negative-control",
                    "license": "test",
                    "selection_note": "rejected",
                    "metrics": {},
                    "reasons": ["negative control rejected"],
                    "candidate_id": "bad-stage",
                    "file_name": "bad-stage.jpg",
                    "sha256": "abc123",
                },
                [],
            )
            report = json.loads((out / "visual_eval_report.json").read_text(encoding="utf-8"))
            self.assertFalse(report["pipeline"]["used_runtime_paths"])
            self.assertEqual(report["verdict"], "fail")
            self.assertEqual(report["image_source"]["sha256"], "abc123")
            self.assertEqual(report["image_source"]["file_name"], "bad-stage.jpg")
            self.assertEqual(report["image_source"]["candidate_id"], "bad-stage")

    def test_sha256_is_stable(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp) / "x.bin"
            p.write_bytes(b"bodytracker")
            self.assertEqual(visual_eval.file_sha256(p), visual_eval.file_sha256(p))


if __name__ == "__main__":
    unittest.main()
