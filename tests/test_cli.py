import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SAMPLE = ROOT / "examples" / "sample.json"


class CliTests(unittest.TestCase):
    def run_cli(self, path, *options):
        return subprocess.run(
            [sys.executable, "-m", "scara_arm_calibration", "--input", str(path), *options],
            cwd=ROOT,
            capture_output=True,
            text=True,
            check=False,
        )

    def test_example_outputs_valid_json_and_improves_fit(self):
        process = self.run_cli(SAMPLE, "--json")
        self.assertEqual(process.returncode, 0, process.stderr)
        result = json.loads(process.stdout)
        self.assertEqual(result["unit"], "mm")
        self.assertTrue(result["converged"])
        self.assertLess(result["final_rmse"], result["initial_rmse"])
        self.assertEqual(len(result["residuals"]), 4)
        self.assertAlmostEqual(result["rmse_history"][0], result["initial_rmse"])
        self.assertAlmostEqual(result["rmse_history"][-1], result["final_rmse"])

    def test_invalid_input_reports_clean_error(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "invalid.json"
            path.write_text('{"nominal_arm": {}}', encoding="utf-8")
            process = self.run_cli(path)
        self.assertEqual(process.returncode, 2)
        self.assertIn("error:", process.stderr)
        self.assertNotIn("Traceback", process.stderr)


if __name__ == "__main__":
    unittest.main()
