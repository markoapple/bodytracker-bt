#!/usr/bin/env python3
"""Check bodytracker benchmark JSON against an explicit performance budget.

Expected input is the JSON emitted by `bodytracker --benchmark-replay`. The script
is deliberately tiny and dependency-free so CI can run it before the full Windows
runtime stack exists. Budget keys are dotted JSON paths. Keys ending in `.min`
are lower bounds; all other numeric budgets are upper bounds.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any


def read_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        data = json.load(handle)
    if not isinstance(data, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return data


def dotted(data: dict[str, Any], path: str) -> Any:
    cur: Any = data
    for part in path.split("."):
        if not isinstance(cur, dict) or part not in cur:
            raise KeyError(path)
        cur = cur[part]
    return cur


def check_value(metric: str, actual: Any, budget: float) -> tuple[bool, str]:
    if not isinstance(actual, (int, float)) or not math.isfinite(float(actual)):
        return False, f"{metric}: actual value is not finite numeric: {actual!r}"
    actual_f = float(actual)
    if metric.endswith(".min") or metric.endswith("_min"):
        ok = actual_f >= budget
        op = ">="
    else:
        ok = actual_f <= budget
        op = "<="
    return ok, f"{metric}: actual {actual_f:.6g} {op} budget {budget:.6g}"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("benchmark_json", type=Path)
    parser.add_argument("budget_json", type=Path, nargs="?", default=Path("qa/performance/default_budget.json"))
    parser.add_argument("--summary", type=Path, help="Optional path to write a machine-readable result summary")
    args = parser.parse_args()

    benchmark = read_json(args.benchmark_json)
    budget = read_json(args.budget_json)
    budgets = budget.get("budgets", {})
    if not isinstance(budgets, dict) or not budgets:
        raise ValueError("budget JSON must contain a non-empty 'budgets' object")

    results: list[dict[str, Any]] = []
    failed = False
    for metric, limit in sorted(budgets.items()):
        if not isinstance(limit, (int, float)) or not math.isfinite(float(limit)):
            raise ValueError(f"budget for {metric} must be finite numeric")
        lookup_metric = metric[:-4] if metric.endswith(".min") else metric
        try:
            actual = dotted(benchmark, lookup_metric)
        except KeyError:
            results.append({"metric": metric, "ok": False, "message": f"{metric}: missing benchmark path {lookup_metric}"})
            failed = True
            continue
        ok, message = check_value(metric, actual, float(limit))
        results.append({"metric": metric, "ok": ok, "message": message})
        failed = failed or not ok

    for result in results:
        print(("PASS " if result["ok"] else "FAIL ") + result["message"])

    summary = {"ok": not failed, "results": results}
    if args.summary:
        args.summary.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
