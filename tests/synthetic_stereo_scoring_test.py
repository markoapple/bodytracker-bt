#!/usr/bin/env python3
"""Regression-scoring checks for synthetic stereo diagnostics.

These tests intentionally mutate otherwise-valid synthetic traces to prove the scorer catches
the lower-body failures it is meant to guard against. They do not build or run the app.
"""

from __future__ import annotations

import copy
import importlib.util
import sys
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


class SyntheticStereoScoringTest(unittest.TestCase):
    def setUp(self) -> None:
        self.generator = load_generator()

    def records(self, frames_per_scenario: int = 8):
        return self.generator.generate_records(
            frames_per_scenario=frames_per_scenario,
            width=384,
            height=288,
            seed=321,
        )

    def test_summary_contains_pass_fail_scoring_and_required_metrics(self) -> None:
        summary = self.generator.summarize_records(self.records(frames_per_scenario=4))

        self.assertEqual(summary["schema_version"], self.generator.SCHEMA_VERSION)
        self.assertIn("regression_status_counts", summary)
        self.assertEqual(
            sum(summary["regression_status_counts"].values()),
            len(self.generator.SCENARIO_ORDER),
        )

        required_metrics = {
            "airborne_foot_path_ratio",
            "airborne_foot_lag_frames",
            "planted_foot_skate_distance_m",
            "root_jitter_inherited_from_foot_only_noise_m",
            "body_over_stance_root_displacement_preservation_ratio",
            "toe_pivot_toe_anchor_error_m",
            "heel_lock_heel_anchor_error_m",
            "flat_plant_anchor_error_m",
            "slip_release_snap_back_amount_m",
            "support_phase_flicker_count",
            "camera_disagreement_vs_3d_world_error",
        }
        for scenario in self.generator.SCENARIO_ORDER:
            regression = summary["scenarios"][scenario]["regression"]
            self.assertIn(regression["status"], {"PASS", "WARN", "FAIL"})
            self.assertTrue(regression["reasons"])
            self.assertTrue(required_metrics.issubset(regression["metrics"].keys()))

        self.assertEqual(summary["scenarios"]["two_camera_disagreement"]["regression"]["status"], "FAIL")
        self.assertEqual(summary["scenarios"]["planted_foot_jitter_from_2d_noise"]["regression"]["status"], "WARN")
        self.assertEqual(summary["scenarios"]["low_res_heel_toe_ankle_ambiguity"]["regression"]["status"], "WARN")

    def test_scoring_catches_underpredicted_airborne_foot_path(self) -> None:
        records = copy.deepcopy(self.records(frames_per_scenario=8))
        swing_records = [record for record in records if record["scenario"] == "pure_airborne_leg_swing"]
        first_world_by_name = {}
        for keypoint in swing_records[0]["diagnostics"]["stereo_blame"]["keypoints"]:
            if keypoint["name"] in self.generator.FOOT_KEYPOINTS["right"]:
                first_world_by_name[keypoint["name"]] = keypoint["triangulated_world"]

        for record in swing_records:
            for keypoint in record["diagnostics"]["stereo_blame"]["keypoints"]:
                if keypoint["name"] in first_world_by_name:
                    keypoint["triangulated"] = True
                    keypoint["triangulated_world"] = first_world_by_name[keypoint["name"]]

        summary = self.generator.summarize_records(records)
        regression = summary["scenarios"]["pure_airborne_leg_swing"]["regression"]

        self.assertEqual(regression["status"], "FAIL")
        self.assertLess(regression["metrics"]["airborne_foot_path_ratio"], 0.75)
        self.assertTrue(any("underprediction" in reason for reason in regression["reasons"]))

    def test_scoring_catches_toe_pivot_anchor_drift(self) -> None:
        records = copy.deepcopy(self.records(frames_per_scenario=8))

        for record in records:
            if record["scenario"] != "toe_pivot":
                continue
            drift = 0.004 * record["scenario_frame_index"]
            toe_anchor = record["ground_truth"]["left_foot"]["toe_anchor"]["position"]
            toe_anchor[0] += drift

        summary = self.generator.summarize_records(records)
        regression = summary["scenarios"]["toe_pivot"]["regression"]

        self.assertEqual(regression["status"], "FAIL")
        self.assertGreater(regression["metrics"]["toe_pivot_toe_anchor_error_m"], 0.015)
        self.assertTrue(any("toe pivot toe anchor drifted" in reason for reason in regression["reasons"]))

    def test_scoring_catches_slip_release_snap_back(self) -> None:
        records = copy.deepcopy(self.records(frames_per_scenario=8))
        slip_records = [record for record in records if record["scenario"] == "slip_release"]
        old_anchor = slip_records[0]["ground_truth"]["left_foot"]["pose"]["position"]

        # Force the last slip frame back near its old planted anchor. A correct release should not
        # snap the body/foot back to this stale contact point.
        final_left_pose = slip_records[-1]["ground_truth"]["left_foot"]["pose"]["position"]
        final_left_pose[0] = old_anchor[0]
        final_left_pose[2] = old_anchor[2]

        summary = self.generator.summarize_records(records)
        regression = summary["scenarios"]["slip_release"]["regression"]

        self.assertEqual(regression["status"], "FAIL")
        self.assertGreater(regression["metrics"]["slip_release_snap_back_amount_m"], 0.02)
        self.assertTrue(any("slip/release snapped back" in reason for reason in regression["reasons"]))


if __name__ == "__main__":
    unittest.main()
