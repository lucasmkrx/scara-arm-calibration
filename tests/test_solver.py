import math
import unittest

from scara_arm_calibration import Arm, Observation, predict_position, solve_calibration
from scara_arm_calibration.solver import _inverse_kinematics


NOMINAL = Arm(220, 220, -49, 166, 150, -50)
MEASURED = (
    (50.4, 49.2),
    (49.1, 248.7),
    (249.1, 249.7),
    (250.6, 49.3),
    (150, 325),
    (100, 175),
)


def forward(arm, proximal_degrees, distal_degrees):
    proximal = math.radians(proximal_degrees)
    combined = math.radians(proximal_degrees + distal_degrees)
    return (
        arm.base_x + arm.proximal_length * math.cos(proximal) + arm.distal_length * math.cos(combined),
        arm.base_y + arm.proximal_length * math.sin(proximal) + arm.distal_length * math.sin(combined),
    )


class SolverTests(unittest.TestCase):
    def test_recovers_known_geometry_from_synthetic_pairs(self):
        actual = Arm(220.7, 219.3, -48.5, 165.6, 150.4, -49.7)
        observations = []
        for measured in MEASURED:
            proximal, distal = _inverse_kinematics(measured, NOMINAL)
            target = forward(
                actual,
                proximal + actual.proximal_angle - NOMINAL.proximal_angle,
                distal + actual.distal_angle - NOMINAL.distal_angle,
            )
            observations.append(Observation(target, measured))

        result = solve_calibration(NOMINAL, observations)

        self.assertTrue(result.converged)
        self.assertLess(result.final_rmse, 1e-5)
        for estimate, expected in zip(result.arm.values(), actual.values()):
            self.assertAlmostEqual(estimate, expected, places=3)

    def test_rejects_unreachable_points(self):
        observations = [Observation((0, 0), (700 + i, 0)) for i in range(4)]
        with self.assertRaisesRegex(ValueError, "reachable workspace"):
            solve_calibration(NOMINAL, observations)

    def test_predicts_new_point_from_calibrated_arm(self):
        measured = (120, 200)
        corrected = predict_position(measured, NOMINAL, NOMINAL)
        self.assertAlmostEqual(corrected[0], measured[0], places=9)
        self.assertAlmostEqual(corrected[1], measured[1], places=9)

    def test_rejects_unidentifiable_layout(self):
        observations = []
        for angle in (0, 30, 60, 90):
            measured = (
                NOMINAL.base_x + 300 * math.cos(math.radians(angle)),
                NOMINAL.base_y + 300 * math.sin(math.radians(angle)),
            )
            observations.append(Observation(measured, measured))
        with self.assertRaisesRegex(ValueError, "identify"):
            solve_calibration(NOMINAL, observations)

    def test_rejects_insufficient_and_duplicate_points(self):
        point = Observation((50, 50), (50.4, 49.2))
        with self.assertRaisesRegex(ValueError, "At least four"):
            solve_calibration(NOMINAL, [point] * 3)
        with self.assertRaisesRegex(ValueError, "distinct"):
            solve_calibration(NOMINAL, [point] * 4)

    def test_rejects_invalid_geometry_and_ranges(self):
        observations = [Observation(point, point) for point in MEASURED[:4]]
        with self.assertRaisesRegex(ValueError, "positive"):
            solve_calibration(Arm(0, 220, -49, 166, 150, -50), observations)
        with self.assertRaisesRegex(ValueError, "positive"):
            solve_calibration(NOMINAL, observations, length_range=0)


if __name__ == "__main__":
    unittest.main()
