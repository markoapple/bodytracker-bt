#!/usr/bin/env python3
"""Isolated checks for the dependency-free synthetic stereo diagnostic generator."""

from __future__ import annotations

import importlib.util
import json
import math
import sys
import tempfile
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
GENERATOR_PATH = REPO_ROOT / "tools" / "synthetic_stereo_diagnostic.py"


def load_generator():
    spec = importlib.util.spec_from_file_location("synthetic_stereo_diagnostic", GENERATOR_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class SyntheticStereoDiagnosticTest(unittest.TestCase):
    def setUp(self) -> None:
        self.generator = load_generator()

    def generate(self, frames_per_scenario: int = 4):
        tmp = tempfile.TemporaryDirectory()
        out_dir = Path(tmp.name)
        self.generator.generate_diagnostics(
            out_dir,
            frames_per_scenario=frames_per_scenario,
            width=384,
            height=288,
            seed=123,
        )
        records = [
            json.loads(line)
            for line in (out_dir / "synthetic_stereo_trace.ndjson").read_text(encoding="utf-8").splitlines()
        ]
        manifest = json.loads((out_dir / "manifest.json").read_text(encoding="utf-8"))
        summary = json.loads((out_dir / "diagnostic_summary.json").read_text(encoding="utf-8"))
        return tmp, out_dir, records, manifest, summary

    def test_manifest_and_trace_cover_all_required_scenarios(self) -> None:
        tmp, out_dir, records, manifest, summary = self.generate(frames_per_scenario=4)
        self.addCleanup(tmp.cleanup)

        self.assertEqual(manifest["schema_version"], self.generator.SCHEMA_VERSION)
        self.assertEqual(manifest["image_size"], [384, 288])
        self.assertTrue((out_dir / "camera_views.svg").exists())
        self.assertTrue((out_dir / "synthetic_stereo_trace.ndjson").exists())

        scenario_names = [scenario["name"] for scenario in manifest["scenarios"]]
        self.assertEqual(scenario_names, self.generator.SCENARIO_ORDER)
        self.assertEqual(set(summary["scenarios"].keys()), set(self.generator.SCENARIO_ORDER))
        self.assertEqual(len(records), 4 * len(self.generator.SCENARIO_ORDER))

        first = records[0]
        self.assertEqual(first["schema_version"], self.generator.SCHEMA_VERSION)
        self.assertIn("ground_truth", first)
        self.assertIn("cameras", first)
        for camera_name in ("camera_a", "camera_b"):
            self.assertEqual(len(first["cameras"][camera_name]["keypoints"]), 26)
            visible_board = [
                p for p in first["cameras"][camera_name]["chessboard"]
                if p["visible"] and p["pixel"] is not None
            ]
            self.assertGreaterEqual(len(visible_board), 20)

        stereo = first["diagnostics"]["stereo_blame"]
        self.assertEqual(stereo["schema_version"], self.generator.SCHEMA_VERSION)
        self.assertEqual(len(stereo["keypoints"]), 26)
        pelvis = next(kp for kp in stereo["keypoints"] if kp["name"] == "hip")
        self.assertTrue(pelvis["both_visible"])
        self.assertTrue(pelvis["triangulated"])
        self.assertIn("world_error_m", pelvis)
        for side in ("left", "right"):
            self.assertIn("mean_triangulated_confidence", stereo["foot_summary"][side])
            self.assertIn("max_world_error_m", stereo["foot_summary"][side])

    def test_occlusion_scenarios_mark_the_expected_cameras(self) -> None:
        tmp, _, records, _, summary = self.generate(frames_per_scenario=4)
        self.addCleanup(tmp.cleanup)

        one_camera = next(r for r in records if r["scenario"] == "one_camera_foot_occlusion")
        both_camera = next(r for r in records if r["scenario"] == "both_camera_foot_occlusion")

        def right_foot_visibility(record, camera_name):
            by_name = {kp["name"]: kp for kp in record["cameras"][camera_name]["keypoints"]}
            names = ("right_ankle", "right_big_toe", "right_small_toe", "right_heel")
            return [by_name[name]["visible"] for name in names], [by_name[name]["occluded"] for name in names]

        visible_a, occluded_a = right_foot_visibility(one_camera, "camera_a")
        visible_b, occluded_b = right_foot_visibility(one_camera, "camera_b")
        self.assertTrue(any(visible_a))
        self.assertFalse(any(occluded_a))
        self.assertFalse(any(visible_b))
        self.assertTrue(all(occluded_b))

        visible_a, occluded_a = right_foot_visibility(both_camera, "camera_a")
        visible_b, occluded_b = right_foot_visibility(both_camera, "camera_b")
        self.assertFalse(any(visible_a))
        self.assertFalse(any(visible_b))
        self.assertTrue(all(occluded_a))
        self.assertTrue(all(occluded_b))

        self.assertEqual(summary["scenarios"]["one_camera_foot_occlusion"]["min_camera_visible_foot_keypoints"]["camera_b"], 4)
        self.assertEqual(summary["scenarios"]["both_camera_foot_occlusion"]["min_camera_visible_foot_keypoints"]["camera_a"], 4)
        self.assertEqual(
            both_camera["diagnostics"]["stereo_blame"]["foot_summary"]["right"]["triangulated"],
            0,
        )
        self.assertEqual(summary["scenarios"]["both_camera_foot_occlusion"]["min_right_foot_triangulated"], 0)

    def test_disagreement_and_calibration_imperfection_are_serialized(self) -> None:
        tmp, _, records, _, summary = self.generate(frames_per_scenario=4)
        self.addCleanup(tmp.cleanup)

        disagreement = next(r for r in records if r["scenario"] == "two_camera_disagreement")
        camera_b = disagreement["cameras"]["camera_b"]
        right_foot = [
            kp for kp in camera_b["keypoints"]
            if kp["name"] in {"right_ankle", "right_big_toe", "right_small_toe", "right_heel"}
        ]
        self.assertTrue(all(kp["visible"] for kp in right_foot))
        self.assertTrue(all(kp["confidence"] > 0.75 for kp in right_foot))
        self.assertTrue(all(kp["disagreement_px"] > 20.0 for kp in right_foot))
        right_foot_blame = disagreement["diagnostics"]["stereo_blame"]["foot_summary"]["right"]
        self.assertEqual(right_foot_blame["triangulated"], 4)
        self.assertGreater(right_foot_blame["max_world_error_m"], 0.15)
        self.assertGreater(summary["scenarios"]["two_camera_disagreement"]["max_camera_disagreement_px"], 20.0)
        self.assertGreater(summary["scenarios"]["two_camera_disagreement"]["max_triangulated_foot_world_error_m"], 0.15)

        calibration = next(r for r in records if r["scenario"] == "mild_calibration_imperfection")
        delta_a = calibration["cameras"]["camera_a"]["calibration_delta"]
        delta_b = calibration["cameras"]["camera_b"]["calibration_delta"]
        self.assertFalse(delta_a["imperfect"])
        self.assertTrue(delta_b["imperfect"])
        self.assertGreater(math.hypot(*delta_b["position_delta_m"][:2]), 0.01)
        self.assertTrue(summary["scenarios"]["mild_calibration_imperfection"]["has_calibration_imperfection"])
        self.assertGreater(
            summary["scenarios"]["mild_calibration_imperfection"]["max_triangulated_foot_world_error_m"],
            0.01,
        )

    def test_low_res_ambiguity_collapses_heel_toe_ankle_pixels(self) -> None:
        tmp, _, records, _, _ = self.generate(frames_per_scenario=4)
        self.addCleanup(tmp.cleanup)

        low_res = next(r for r in records if r["scenario"] == "low_res_heel_toe_ankle_ambiguity")
        by_name = {kp["name"]: kp for kp in low_res["cameras"]["camera_a"]["keypoints"]}

        for side in ("left", "right"):
            ankle = by_name[f"{side}_ankle"]
            heel = by_name[f"{side}_heel"]
            toe = by_name[f"{side}_big_toe"]
            self.assertTrue(ankle["low_res_ambiguous"])
            self.assertTrue(heel["low_res_ambiguous"])
            self.assertTrue(toe["low_res_ambiguous"])
            self.assertLess(ankle["confidence"], 0.60)
            ax, ay = ankle["pixel"]
            hx, hy = heel["pixel"]
            tx, ty = toe["pixel"]
            self.assertLess(math.hypot(ax - hx, ay - hy), 9.5)
            self.assertLess(math.hypot(ax - tx, ay - ty), 9.5)


if __name__ == "__main__":
    unittest.main()
