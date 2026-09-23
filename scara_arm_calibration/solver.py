"""Bounded least-squares calibration for a two-link planar SCARA arm.

Measured XY positions are converted to joint angles using the nominal geometry.
The angle change from each nominal end-stop is then held fixed while fitting the
six geometry and end-stop parameters to the target XY positions.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import acos, atan2, cos, degrees, hypot, isfinite, radians, sin, sqrt
from typing import Sequence


@dataclass(frozen=True)
class Arm:
    proximal_length: float
    distal_length: float
    proximal_angle: float
    distal_angle: float
    base_x: float
    base_y: float

    def values(self) -> tuple[float, ...]:
        return (
            self.proximal_length, self.distal_length,
            self.proximal_angle, self.distal_angle,
            self.base_x, self.base_y,
        )


@dataclass(frozen=True)
class Observation:
    target: tuple[float, float]
    measured: tuple[float, float]


@dataclass(frozen=True)
class CalibrationResult:
    arm: Arm
    initial_rmse: float
    final_rmse: float
    initial_max_error: float
    final_max_error: float
    residuals: tuple[tuple[float, float], ...]
    rmse_history: tuple[float, ...]
    iterations: int
    converged: bool


def _inverse_kinematics(point: tuple[float, float], arm: Arm) -> tuple[float, float]:
    dx, dy = point[0] - arm.base_x, point[1] - arm.base_y
    distance = hypot(dx, dy)
    low = abs(arm.proximal_length - arm.distal_length)
    high = arm.proximal_length + arm.distal_length
    if not low < distance < high:
        raise ValueError(f"Measured point {point} is outside the nominal arm's reachable workspace")

    cosine = (distance * distance - arm.proximal_length**2 - arm.distal_length**2) / (
        2 * arm.proximal_length * arm.distal_length
    )
    distal = acos(max(-1.0, min(1.0, cosine)))
    proximal = atan2(dy, dx) - atan2(
        arm.distal_length * sin(distal),
        arm.proximal_length + arm.distal_length * cos(distal),
    )
    return degrees(proximal), degrees(distal)


def predict_position(measured: tuple[float, float], nominal: Arm, calibrated: Arm) -> tuple[float, float]:
    """Predict the corrected XY position of a new measured point.

    The measured point is converted to joint angles with the same nominal arm
    and positive distal-angle branch used during fitting.
    """
    proximal, distal = _inverse_kinematics(measured, nominal)
    proximal = radians(proximal + calibrated.proximal_angle - nominal.proximal_angle)
    combined = proximal + radians(distal + calibrated.distal_angle - nominal.distal_angle)
    return (
        calibrated.base_x + calibrated.proximal_length * cos(proximal) + calibrated.distal_length * cos(combined),
        calibrated.base_y + calibrated.proximal_length * sin(proximal) + calibrated.distal_length * sin(combined),
    )


def _predict_and_jacobian(
    values: Sequence[float], offsets: Sequence[tuple[float, float]], observations: Sequence[Observation]
) -> tuple[list[float], list[list[float]]]:
    proximal_length, distal_length, proximal_angle, distal_angle, base_x, base_y = values
    residuals: list[float] = []
    jacobian: list[list[float]] = []
    angle_scale = radians(1.0)
    for (proximal_offset, distal_offset), observation in zip(offsets, observations):
        proximal = radians(proximal_angle + proximal_offset)
        combined = radians(proximal_angle + proximal_offset + distal_angle + distal_offset)
        cp, sp, cc, sc = cos(proximal), sin(proximal), cos(combined), sin(combined)
        x = base_x + proximal_length * cp + distal_length * cc
        y = base_y + proximal_length * sp + distal_length * sc
        residuals.extend((x - observation.target[0], y - observation.target[1]))
        jacobian.append([
            cp, cc, -angle_scale * (proximal_length * sp + distal_length * sc),
            -angle_scale * distal_length * sc, 1.0, 0.0,
        ])
        jacobian.append([
            sp, sc, angle_scale * (proximal_length * cp + distal_length * cc),
            angle_scale * distal_length * cc, 0.0, 1.0,
        ])
    return residuals, jacobian


def _linear_solve(matrix: list[list[float]], rhs: list[float]) -> list[float]:
    """Solve a small dense system with partial pivoting."""
    size = len(rhs)
    rows = [row[:] + [rhs[index]] for index, row in enumerate(matrix)]
    for column in range(size):
        pivot = max(range(column, size), key=lambda index: abs(rows[index][column]))
        if abs(rows[pivot][column]) < 1e-14:
            raise ValueError("Calibration geometry is insufficient to identify six parameters")
        rows[column], rows[pivot] = rows[pivot], rows[column]
        factor = rows[column][column]
        for index in range(column + 1, size):
            ratio = rows[index][column] / factor
            for item in range(column, size + 1):
                rows[index][item] -= ratio * rows[column][item]
    solution = [0.0] * size
    for index in range(size - 1, -1, -1):
        solution[index] = (
            rows[index][size] - sum(rows[index][item] * solution[item] for item in range(index + 1, size))
        ) / rows[index][index]
    return solution


def _metrics(residuals: Sequence[float]) -> tuple[float, float]:
    distances = [hypot(residuals[index], residuals[index + 1]) for index in range(0, len(residuals), 2)]
    return sqrt(sum(distance * distance for distance in distances) / len(distances)), max(distances)


def _check_identifiability(jacobian: Sequence[Sequence[float]]) -> None:
    """Reject point layouts whose normalized Jacobian has fewer than six directions."""
    basis: list[list[float]] = []
    for column in range(6):
        vector = [row[column] for row in jacobian]
        magnitude = sqrt(sum(value * value for value in vector))
        if magnitude < 1e-12:
            raise ValueError("Calibration points cannot identify all six parameters")
        vector = [value / magnitude for value in vector]
        for direction in basis:
            projection = sum(value * other for value, other in zip(vector, direction))
            vector = [value - projection * other for value, other in zip(vector, direction)]
        remaining = sqrt(sum(value * value for value in vector))
        if remaining < 1e-8:
            raise ValueError("Calibration points cannot identify all six parameters")
        basis.append([value / remaining for value in vector])


def _projected_gradient(gradient: Sequence[float], values: Sequence[float],
                        lower: Sequence[float], upper: Sequence[float]) -> float:
    active = []
    for index, value in enumerate(gradient):
        if values[index] <= lower[index] + 1e-9 and value > 0:
            continue
        if values[index] >= upper[index] - 1e-9 and value < 0:
            continue
        active.append(abs(value))
    return max(active, default=0.0)


def solve_calibration(
    nominal: Arm,
    observations: Sequence[Observation],
    *,
    length_range: float = 10.0,
    angle_range: float = 10.0,
    base_range: float = 10.0,
    max_iterations: int = 200,
) -> CalibrationResult:
    """Fit arm parameters; angles are degrees and all positions share one length unit.

    At least four paired, distinct observations are required. The positive-angle IK
    branch is assumed for every measured point. Range arguments bound each
    fitted parameter around its nominal value.
    """
    values = list(nominal.values())
    if not all(isfinite(value) for value in values) or min(values[:2]) <= 0:
        raise ValueError("Nominal arm values must be finite and lengths must be positive")
    if len(observations) < 4:
        raise ValueError("At least four paired observations are required")
    if len({observation.measured for observation in observations}) != len(observations):
        raise ValueError("Measured points must be distinct")
    for observation in observations:
        if not all(isfinite(value) for value in (*observation.target, *observation.measured)):
            raise ValueError("Observation coordinates must be finite")
    if not all(isfinite(value) and value > 0 for value in (length_range, angle_range, base_range)):
        raise ValueError("Parameter ranges must be finite and positive")
    if max_iterations < 1:
        raise ValueError("max_iterations must be positive")

    offsets = []
    for observation in observations:
        proximal, distal = _inverse_kinematics(observation.measured, nominal)
        offsets.append((proximal - nominal.proximal_angle, distal - nominal.distal_angle))

    ranges = (length_range, length_range, angle_range, angle_range, base_range, base_range)
    lower = [value - width for value, width in zip(values, ranges)]
    upper = [value + width for value, width in zip(values, ranges)]
    lower[0] = max(lower[0], 1e-9)
    lower[1] = max(lower[1], 1e-9)
    residuals, jacobian = _predict_and_jacobian(values, offsets, observations)
    _check_identifiability(jacobian)
    initial_rmse, initial_max_error = _metrics(residuals)
    rmse_history = [initial_rmse]
    cost = sum(value * value for value in residuals)
    damping = 1e-3
    converged = False
    iterations = 0

    for iterations in range(1, max_iterations + 1):
        normal = [[sum(row[i] * row[j] for row in jacobian) for j in range(6)] for i in range(6)]
        gradient = [sum(row[i] * error for row, error in zip(jacobian, residuals)) for i in range(6)]
        diagonal = [max(normal[index][index], 1.0) for index in range(6)]
        accepted = False
        for _ in range(20):
            system = [row[:] for row in normal]
            for index in range(6):
                system[index][index] += damping * diagonal[index]
            step = _linear_solve(system, [-value for value in gradient])
            candidate = [min(upper[i], max(lower[i], values[i] + step[i])) for i in range(6)]
            trial_residuals, trial_jacobian = _predict_and_jacobian(candidate, offsets, observations)
            trial_cost = sum(value * value for value in trial_residuals)
            if trial_cost < cost - 1e-12:
                previous_cost = cost
                values, residuals, jacobian, cost = candidate, trial_residuals, trial_jacobian, trial_cost
                rmse_history.append(sqrt(cost / len(observations)))
                damping = max(damping / 3, 1e-12)
                accepted = True
                if previous_cost - cost < 1e-10 * max(1.0, previous_cost):
                    converged = True
                break
            damping *= 10
        if converged:
            break
        if not accepted:
            converged = _projected_gradient(gradient, values, lower, upper) < 1e-6
            break

    final_rmse, final_max_error = _metrics(residuals)
    return CalibrationResult(
        Arm(*values), initial_rmse, final_rmse, initial_max_error, final_max_error,
        tuple((residuals[i], residuals[i + 1]) for i in range(0, len(residuals), 2)),
        tuple(rmse_history),
        iterations, converged,
    )
