#!/usr/bin/env python3
"""Source-sanity harness checks.

These tests keep cheap test files visible to CTest. They intentionally inspect
only build metadata, not runtime hardware, model loading, OpenCV, or vcpkg.
"""

from __future__ import annotations

import json
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]


class CTestRegistrationTest(unittest.TestCase):
    def test_top_level_test_files_are_registered_in_cmake(self) -> None:
        cmake = (REPO_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
        test_files = sorted((REPO_ROOT / "tests").glob("*_test.*"))
        self.assertGreater(len(test_files), 10)

        missing = []
        for path in test_files:
            rel = path.relative_to(REPO_ROOT).as_posix()
            if rel not in cmake and path.name not in cmake:
                missing.append(rel)
        self.assertEqual([], missing)

    def test_source_sanity_preset_stays_dependency_light_but_runs_static_python(self) -> None:
        presets = json.loads((REPO_ROOT / "CMakePresets.json").read_text(encoding="utf-8"))
        source_sanity = next(item for item in presets["configurePresets"] if item["name"] == "source-sanity")
        cache = source_sanity["cacheVariables"]

        self.assertEqual("OFF", cache["BODYTRACKER_BUILD_APP"])
        self.assertEqual("OFF", cache["BODYTRACKER_BUILD_FULL_TESTS"])
        self.assertEqual("OFF", cache["BODYTRACKER_BUILD_CONFIG_CPP_TEST"])
        self.assertEqual("ON", cache["BODYTRACKER_REGISTER_PYTHON_TESTS"])
        self.assertEqual("OFF", cache["BODYTRACKER_REGISTER_ONNX_METADATA_TEST"])


if __name__ == "__main__":
    unittest.main()
