#!/usr/bin/env python3
"""Tests for the monocular real-footage validation harness."""

from __future__ import annotations

import contextlib
import io
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import validate_monocular_footage as validator  # noqa: E402


REQUIRED = [
    "standing",
    "stepping",
    "turning",
    "crouching",
    "seated",
    "reclined",
    "partial_occlusion",
    "poor_lighting",
]


def frame(
    t: float,
    *,
    mode: str = "monocular",
    depth_source: str = "inferred_monocular",
    degradation: str = "nominal",
    left_x: float = -0.14,
    right_x: float = 0.14,
    z: float = 1.6,
    confidence: float = 0.42,
    solver_confidence: float = 0.34,
    triangulated_count: int = 0,
) -> dict:
    return {
        "timestamp_seconds": t,
        "degradation_mode": degradation,
        "tracking": {
            "confidence": confidence,
            "root": {"position": [0.0, 0.95, z], "orientation": [0, 0, 0, 1]},
            "left_foot": {"position": [left_x, 0.0, z], "orientation": [0, 0, 0, 1]},
            "right_foot": {"position": [right_x, 0.0, z], "orientation": [0, 0, 0, 1]},
            "solver": {
                "tracking_mode": mode,
                "depth_source": depth_source,
                "depth": {
                    "source": depth_source,
                    "confidence": solver_confidence,
                    "mean_inferred_depth_m": z,
                    "inferred_count": 13 if mode == "monocular" else 0,
                    "scale_source": "floor_spacing" if mode == "monocular" else "none",
                    "floor_assist": {
                        "status": "active" if mode == "monocular" else "disabled",
                        "source": "floor_spacing" if mode == "monocular" else "none",
                        "depth_m": z,
                        "confidence": 0.41 if mode == "monocular" else 0.0,
                    },
                },
                "triangulation": {
                    "preliminary": {
                        "tracking_mode": mode,
                        "depth_source": depth_source,
                        "triangulated_count": triangulated_count,
                        "inferred_depth_count": 13 if mode == "monocular" else 0,
                        "mean_inferred_depth_m": z,
                        "mean_confidence": solver_confidence,
                        "foot_mean_confidence": solver_confidence,
                        "left_foot_contact_confidence": solver_confidence,
                        "right_foot_contact_confidence": solver_confidence,
                        "monocular_scale_source": "floor_spacing" if mode == "monocular" else "none",
                        "monocular_floor_assist_status": "active" if mode == "monocular" else "disabled",
                        "monocular_floor_assist_depth_m": z if mode == "monocular" else 0.0,
                        "monocular_floor_assist_confidence": 0.41 if mode == "monocular" else 0.0,
                    }
                },
            },
        },
    }


def write_replay(path: Path, frames: list[dict]) -> None:
    path.write_text("\n".join(json.dumps(item) for item in frames) + "\n", encoding="utf-8")


class MonocularRealFootageValidationTest(unittest.TestCase):
    def test_full_manifest_passes_with_required_scenarios_and_stereo_comparison(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            base = Path(tmp)
            clips = []
            for index, scenario in enumerate(REQUIRED):
                mono_path = base / f"{scenario}_mono.ndjson"
                stereo_path = base / f"{scenario}_stereo.ndjson"
                write_replay(
                    mono_path,
                    [
                        frame(0.00, z=1.5 + 0.01 * index),
                        frame(0.033, left_x=-0.13, right_x=0.15, z=1.51 + 0.01 * index),
                    ],
                )
                write_replay(
                    stereo_path,
                    [
                        frame(
                            0.00,
                            mode="stereo",
                            depth_source="triangulated_stereo",
                            solver_confidence=0.68,
                            triangulated_count=13,
                        ),
                        frame(
                            0.033,
                            mode="stereo",
                            depth_source="triangulated_stereo",
                            left_x=-0.13,
                            right_x=0.15,
                            solver_confidence=0.69,
                            triangulated_count=13,
                        ),
                    ],
                )
                clips.append(
                    {
                        "name": f"{scenario} clip",
                        "scenario": scenario,
                        "monocular_replay": mono_path.name,
                        "stereo_replay": stereo_path.name,
                    }
                )

            manifest = base / "manifest.json"
            manifest.write_text(json.dumps({"clips": clips}), encoding="utf-8")
            report, ok = validator.validate_manifest(manifest)

            self.assertTrue(ok, json.dumps(report, indent=2))
            self.assertTrue(report["passed"])
            self.assertEqual(len(report["clips"]), len(REQUIRED))

    def test_missing_required_scenario_fails(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            base = Path(tmp)
            replay = base / "standing.ndjson"
            write_replay(replay, [frame(0.0), frame(0.033)])
            manifest = base / "manifest.json"
            manifest.write_text(
                json.dumps({"clips": [{"name": "standing only", "scenario": "standing", "monocular_replay": replay.name}]}),
                encoding="utf-8",
            )

            report, ok = validator.validate_manifest(manifest)

            self.assertFalse(ok)
            coverage = next(check for check in report["checks"] if check["message"] == "required real-footage scenario coverage is complete")
            self.assertIn("poor_lighting", coverage["details"]["missing"])

    def test_foot_snap_fails(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            base = Path(tmp)
            replay = base / "snap.ndjson"
            write_replay(
                replay,
                [
                    frame(0.000, left_x=-0.14),
                    frame(0.033, left_x=1.25),
                ],
            )
            clips = []
            for scenario in REQUIRED:
                clips.append({"name": scenario, "scenario": scenario, "monocular_replay": replay.name})
            manifest = base / "manifest.json"
            manifest.write_text(json.dumps({"clips": clips}), encoding="utf-8")

            report, ok = validator.validate_manifest(manifest)

            self.assertFalse(ok)
            snap_checks = [check for check in report["checks"] if "feet do not snap" in check["message"]]
            self.assertTrue(any(not check["ok"] for check in snap_checks))

    def test_cli_writes_report_and_returns_nonzero_on_failure(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            base = Path(tmp)
            manifest = base / "manifest.json"
            manifest.write_text(json.dumps({"clips": []}), encoding="utf-8")
            report_path = base / "report.json"

            with contextlib.redirect_stdout(io.StringIO()):
                exit_code = validator.main([str(manifest), "--report", str(report_path)])

            self.assertEqual(exit_code, 1)
            self.assertTrue(report_path.exists())
            self.assertFalse(json.loads(report_path.read_text(encoding="utf-8"))["passed"])


if __name__ == "__main__":
    unittest.main()
