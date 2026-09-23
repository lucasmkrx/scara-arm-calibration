"""Command-line interface for SCARA calibration."""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import asdict
from pathlib import Path

from .solver import Arm, Observation, solve_calibration


def _pair(value: object, label: str) -> tuple[float, float]:
    if not isinstance(value, list) or len(value) != 2:
        raise ValueError(f"{label} must be an [x, y] array")
    try:
        return float(value[0]), float(value[1])
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must contain two numbers") from exc


def _load_input(path: Path) -> tuple[Arm, list[Observation], dict[str, float], str]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("Input must be a JSON object")
    try:
        arm_data = data["nominal_arm"]
        nominal = Arm(*(float(arm_data[name]) for name in Arm.__dataclass_fields__))
        observations = [
            Observation(_pair(item["target"], "target"), _pair(item["measured"], "measured"))
            for item in data["observations"]
        ]
        ranges = data.get("parameter_ranges", {})
        options = {
            "length_range": float(ranges.get("length", 10)),
            "angle_range": float(ranges.get("angle", 10)),
            "base_range": float(ranges.get("base", 10)),
        }
        unit = str(data.get("unit", "units"))
    except (KeyError, TypeError, AttributeError) as exc:
        raise ValueError(f"Missing or invalid input field: {exc}") from exc
    return nominal, observations, options, unit


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Calibrate a planar two-link SCARA arm.")
    parser.add_argument("--input", required=True, type=Path, help="JSON file of nominal geometry and XY pairs")
    parser.add_argument("--json", action="store_true", help="print machine-readable JSON")
    args = parser.parse_args(argv)
    try:
        nominal, observations, options, unit = _load_input(args.input)
        result = solve_calibration(nominal, observations, **options)
    except (OSError, json.JSONDecodeError, ValueError) as exc:
        parser.exit(2, f"error: {exc}\n")

    if args.json:
        print(json.dumps({"unit": unit, **asdict(result)}, indent=2))
    else:
        print(f"Calibration from {len(observations)} XY pairs ({unit})")
        print(f"RMS position error: {result.initial_rmse:.4f} -> {result.final_rmse:.4f} {unit}")
        print(f"Max position error: {result.initial_max_error:.4f} -> {result.final_max_error:.4f} {unit}")
        print(f"Iterations: {result.iterations}; converged: {result.converged}")
        print("\nFitted parameters")
        for name, value in asdict(result.arm).items():
            suffix = "deg" if name.endswith("angle") else unit
            print(f"  {name:18} {value:11.5f} {suffix}")
        print("\nFinal XY residuals (predicted - target)")
        for index, (dx, dy) in enumerate(result.residuals, 1):
            print(f"  point {index:2}: dx={dx:+.4f}, dy={dy:+.4f} {unit}")
    return 0 if result.converged else 1


if __name__ == "__main__":
    sys.exit(main())
