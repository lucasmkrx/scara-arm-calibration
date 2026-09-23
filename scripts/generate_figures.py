"""Regenerate the SVG figures in docs/figures from the included example.

Run from any directory with: python3 scripts/generate_figures.py
Only Python's standard library and this package are used.
"""

from __future__ import annotations

import json
import math
import sys
from html import escape
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from scara_arm_calibration import Arm, Observation, predict_position, solve_calibration  # noqa: E402


OUT = ROOT / "docs" / "figures"
INK = "#202a35"
BLUE = "#245b78"
RUST = "#aa573f"
MUTED = "#65717c"
GRID = "#d9dee2"
PALE = "#f6f7f7"

STYLE = """
  .title { font: 600 20px Georgia, 'Times New Roman', serif; fill: #202a35; }
  .label { font: 16px Arial, Helvetica, sans-serif; fill: #202a35; }
  .small { font: 14px Arial, Helvetica, sans-serif; fill: #65717c; }
  .tick { font: 14px Arial, Helvetica, sans-serif; fill: #4c5965; }
  .math { font: italic 20px Georgia, 'Times New Roman', serif; fill: #202a35; }
"""


def txt(x: float, y: float, value: str, css: str = "label", anchor: str = "start",
        extra: str = "") -> str:
    return (
        f'<text x="{x:.2f}" y="{y:.2f}" class="{css}" '
        f'text-anchor="{anchor}" {extra}>{escape(value)}</text>'
    )


def line(x1: float, y1: float, x2: float, y2: float, color: str = INK,
         width: float = 1.5, extra: str = "") -> str:
    return (
        f'<line x1="{x1:.2f}" y1="{y1:.2f}" x2="{x2:.2f}" y2="{y2:.2f}" '
        f'stroke="{color}" stroke-width="{width}" {extra}/>'
    )


def circle(x: float, y: float, radius: float, fill: str, stroke: str = "none",
           width: float = 0) -> str:
    return (
        f'<circle cx="{x:.2f}" cy="{y:.2f}" r="{radius}" fill="{fill}" '
        f'stroke="{stroke}" stroke-width="{width}"/>'
    )


def svg(title: str, desc: str, width: int, height: int, pieces: list[str]) -> str:
    header = (
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
        f'viewBox="0 0 {width} {height}" role="img" aria-labelledby="title desc">'
        f'<title id="title">{escape(title)}</title>'
        f'<desc id="desc">{escape(desc)}</desc>'
        f'<style>{STYLE}</style>'
        '<defs><marker id="arrow" markerWidth="8" markerHeight="8" refX="7" '
        'refY="4" orient="auto" markerUnits="strokeWidth">'
        f'<path d="M0,0 L8,4 L0,8 Z" fill="{MUTED}"/></marker></defs>'
        '<rect width="100%" height="100%" fill="#fff"/>'
    )
    return header + "".join(pieces) + "</svg>\n"


def arc(cx: float, cy: float, radius: float, start: float, end: float,
        color: str = MUTED) -> str:
    count = 24
    points = []
    for step in range(count + 1):
        angle = math.radians(start + (end - start) * step / count)
        points.append(f"{cx + radius * math.cos(angle):.2f},{cy - radius * math.sin(angle):.2f}")
    return f'<polyline points="{" ".join(points)}" fill="none" stroke="{color}" stroke-width="2"/>'


def model_figure() -> str:
    width, height = 1000, 560
    base = (270.0, 414.0)
    proximal_angle, distal_angle = 25.0, 70.0
    elbow = (
        base[0] + 245 * math.cos(math.radians(proximal_angle)),
        base[1] - 245 * math.sin(math.radians(proximal_angle)),
    )
    tip = (
        elbow[0] + 195 * math.cos(math.radians(proximal_angle + distal_angle)),
        elbow[1] - 195 * math.sin(math.radians(proximal_angle + distal_angle)),
    )
    pieces = [
        txt(56, 52, "A  |  PLANAR KINEMATIC MODEL", "title"),
        line(56, 69, 944, 69, GRID, 1),
        line(125, 460, 785, 460, MUTED, 1.5, 'marker-end="url(#arrow)"'),
        line(155, 486, 155, 106, MUTED, 1.5, 'marker-end="url(#arrow)"'),
        txt(792, 466, "x", "math"),
        txt(147, 95, "y", "math"),
        txt(140, 482, "O", "math"),
        line(base[0], base[1], base[0], 460, GRID, 1.5, 'stroke-dasharray="5 5"'),
        line(155, base[1], base[0], base[1], GRID, 1.5, 'stroke-dasharray="5 5"'),
        line(base[0], base[1], elbow[0], elbow[1], BLUE, 7, 'stroke-linecap="round"'),
        line(elbow[0], elbow[1], tip[0], tip[1], RUST, 7, 'stroke-linecap="round"'),
        line(base[0], base[1], base[0] + 108, base[1], GRID, 1.4, 'stroke-dasharray="5 5"'),
        line(elbow[0], elbow[1],
             elbow[0] + 106 * math.cos(math.radians(proximal_angle)),
             elbow[1] - 106 * math.sin(math.radians(proximal_angle)),
             GRID, 1.4, 'stroke-dasharray="5 5"'),
        arc(base[0], base[1], 58, 0, proximal_angle, BLUE),
        arc(elbow[0], elbow[1], 52, proximal_angle, proximal_angle + distal_angle, RUST),
        circle(base[0], base[1], 9, "#fff", INK, 2.5),
        circle(elbow[0], elbow[1], 9, "#fff", INK, 2.5),
        circle(tip[0], tip[1], 7, INK),
        txt(base[0] + 20, base[1] + 34, "b = (b_x, b_y)", "math"),
        txt((base[0] + elbow[0]) / 2 - 16, (base[1] + elbow[1]) / 2 - 20, "L_p", "math"),
        txt(elbow[0] + 22, (elbow[1] + tip[1]) / 2, "L_d", "math"),
        txt(base[0] + 78, base[1] - 19, "θ_p", "math"),
        txt(elbow[0] + 76, elbow[1] - 72, "θ_d", "math"),
        txt(tip[0] + 21, tip[1] - 4, "p = (x, y)", "math"),
        txt(55, 535, "Two revolute joints; positive distal-angle branch shown.", "small"),
    ]
    return svg(
        "Two-link planar SCARA kinematic model",
        "Coordinate axes, base offset, proximal and distal links, joint angles, and predicted end-effector position.",
        width, height, pieces,
    )


def log_y(value: float, top: float, bottom: float, maximum: float, minimum: float) -> float:
    return top + (math.log10(maximum) - math.log10(value)) / (
        math.log10(maximum) - math.log10(minimum)
    ) * (bottom - top)


def axes(pieces: list[str], left: float, right: float, top: float, bottom: float,
         minimum: float, maximum: float) -> None:
    for tick in (0.01, 0.1, 1.0):
        y = log_y(tick, top, bottom, maximum, minimum)
        pieces.append(line(left, y, right, y, GRID, 1))
        pieces.append(txt(left - 18, y + 5, f"{tick:g}", "tick", "end"))
    pieces.append(line(left, top, left, bottom, INK, 1.5))
    pieces.append(line(left, bottom, right, bottom, INK, 1.5))


def residual_figure(observations: list[Observation], nominal: Arm, fitted: Arm,
                    initial_rmse: float, final_rmse: float) -> str:
    width, height = 1000, 620
    left, right, top, bottom = 125, 915, 135, 500
    before = [math.dist(obs.measured, obs.target) for obs in observations]
    after = [math.dist(predict_position(obs.measured, nominal, fitted), obs.target) for obs in observations]
    pieces = [
        txt(56, 52, "B  |  POSITION ERROR BY CALIBRATION POINT", "title"),
        line(56, 69, 944, 69, GRID, 1),
        txt(left, 111, "Euclidean position error (mm; logarithmic axis)", "label"),
    ]
    axes(pieces, left, right, top, bottom, 0.005, 2.0)
    for index, (before_error, after_error) in enumerate(zip(before, after), 1):
        x = left + (right - left) * index / (len(observations) + 1)
        y_before = log_y(before_error, top, bottom, 2.0, 0.005)
        y_after = log_y(after_error, top, bottom, 2.0, 0.005)
        pieces.extend([
            line(x, y_before, x, y_after, "#a9b2b8", 1.5, 'stroke-dasharray="5 5"'),
            circle(x, y_before, 7, BLUE),
            f'<rect x="{x - 6:.2f}" y="{y_after - 6:.2f}" width="12" height="12" fill="{RUST}"/>',
            txt(x, bottom + 29, str(index), "tick", "middle"),
        ])
    pieces.extend([
        txt((left + right) / 2, bottom + 63, "Calibration point", "label", "middle"),
        circle(690, 100, 6, BLUE),
        txt(705, 105, "Before", "small"),
        f'<rect x="790" y="94" width="12" height="12" fill="{RUST}"/>',
        txt(809, 105, "After", "small"),
        txt(56, 586, f"RMS: {initial_rmse:.4f} → {final_rmse:.4f} mm  |  Same four points used to fit the model", "small"),
    ])
    return svg(
        "Per-point calibration errors",
        "Logarithmic plot of Euclidean errors before and after fitting for each of the four example points.",
        width, height, pieces,
    )


def convergence_figure(history: tuple[float, ...]) -> str:
    width, height = 1000, 620
    left, right, top, bottom = 125, 915, 135, 500
    pieces = [
        txt(56, 52, "C  |  OPTIMIZATION CONVERGENCE", "title"),
        line(56, 69, 944, 69, GRID, 1),
        txt(left, 111, "RMS position error (mm; logarithmic axis)", "label"),
    ]
    axes(pieces, left, right, top, bottom, 0.005, 2.0)
    locations = []
    for index, value in enumerate(history):
        x = left + (right - left) * index / (len(history) - 1)
        y = log_y(value, top, bottom, 2.0, 0.005)
        locations.append((x, y))
        pieces.append(txt(x, bottom + 28, str(index), "tick", "middle"))
    coordinates = " ".join(f"{x:.2f},{y:.2f}" for x, y in locations)
    pieces.append(f'<polyline points="{coordinates}" fill="none" stroke="{BLUE}" stroke-width="2.5"/>')
    for x, y in locations:
        pieces.append(circle(x, y, 6, "#fff", BLUE, 2.5))
    pieces.extend([
        txt(locations[0][0] + 12, locations[0][1] - 12, f"{history[0]:.4f} mm", "small"),
        txt(locations[-1][0] - 10, locations[-1][1] - 17, f"{history[-1]:.4f} mm", "small", "end"),
        txt((left + right) / 2, bottom + 63, "Accepted solver update", "label", "middle"),
        txt(56, 586, "Iteration 0 is the nominal geometry; only accepted updates are plotted.", "small"),
    ])
    return svg(
        "Convergence of the calibration solver",
        "RMS position error at the nominal geometry and after each accepted least-squares update.",
        width, height, pieces,
    )


def main() -> None:
    data = json.loads((ROOT / "examples" / "sample.json").read_text(encoding="utf-8"))
    arm = Arm(*(float(data["nominal_arm"][name]) for name in Arm.__dataclass_fields__))
    observations = [
        Observation(tuple(item["target"]), tuple(item["measured"]))
        for item in data["observations"]
    ]
    ranges = data.get("parameter_ranges", {})
    result = solve_calibration(
        arm, observations,
        length_range=ranges.get("length", 10),
        angle_range=ranges.get("angle", 10),
        base_range=ranges.get("base", 10),
    )
    if data.get("unit") != "mm":
        raise ValueError("These figures are labeled in mm; update the labels for another unit")
    OUT.mkdir(parents=True, exist_ok=True)
    figures = {
        "01-kinematic-model.svg": model_figure(),
        "02-position-errors.svg": residual_figure(
            observations, arm, result.arm, result.initial_rmse, result.final_rmse
        ),
        "03-convergence.svg": convergence_figure(result.rmse_history),
    }
    for name, content in figures.items():
        (OUT / name).write_text(content, encoding="utf-8")
        print(OUT / name)


if __name__ == "__main__":
    main()
