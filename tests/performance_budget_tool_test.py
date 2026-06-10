#!/usr/bin/env python3
"""Regression test for the dependency-free performance budget checker.

The original test spawned a nested Python process through sys.executable. That is
fragile under CTest on Windows when Python is discovered through launcher shims
or when nested process creation is restricted. The behavior under test is the
budget checker itself, so exercise its functions in-process and keep the failure
messages visible.
"""
from __future__ import annotations

import importlib.util
import json
import tempfile
from pathlib import Path

root = Path(__file__).resolve().parents[1]
tool_path = root / "tools" / "check_performance_budget.py"
budget_path = root / "qa" / "performance" / "default_budget.json"

spec = importlib.util.spec_from_file_location("check_performance_budget", tool_path)
if spec is None or spec.loader is None:
    raise SystemExit(f"could not load {tool_path}")
tool = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tool)

ok_benchmark = {
    "step_ms": {"avg": 4.0, "p95": 8.0, "max": 12.0},
    "solver_ms": {"total_avg": 2.0, "preliminary_avg": 1.0, "final_avg": 1.0},
    "failed_count": 0,
    "avg_confidence": 0.8,
    "wall_ms": 100.0,
}
fail_benchmark = {
    **ok_benchmark,
    "step_ms": {"avg": 40.0, "p95": 80.0, "max": 100.0},
}


def run_check(benchmark: dict, budget: dict) -> tuple[bool, list[str]]:
    budgets = budget.get("budgets", {})
    if not isinstance(budgets, dict) or not budgets:
        raise AssertionError("default budget must contain a non-empty 'budgets' object")

    messages: list[str] = []
    failed = False
    for metric, limit in sorted(budgets.items()):
        lookup_metric = metric[:-4] if metric.endswith(".min") else metric
        try:
            actual = tool.dotted(benchmark, lookup_metric)
        except KeyError:
            messages.append(f"FAIL {metric}: missing benchmark path {lookup_metric}")
            failed = True
            continue
        ok, message = tool.check_value(metric, actual, float(limit))
        messages.append(("PASS " if ok else "FAIL ") + message)
        failed = failed or not ok
    return not failed, messages


budget = tool.read_json(budget_path)
ok, ok_messages = run_check(ok_benchmark, budget)
bad, bad_messages = run_check(fail_benchmark, budget)

for line in ok_messages + bad_messages:
    print(line)

if not ok:
    raise SystemExit("expected the nominal benchmark to pass")
if bad:
    raise SystemExit("expected the slow benchmark to fail")
