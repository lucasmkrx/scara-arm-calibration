# SCARA Arm Calibration

A dependency-free Python package for fitting a planar two-link SCARA arm to paired measured and target XY positions. It estimates two link lengths, two joint reference angles, and the base position.

## Quick start

Python 3.10 or newer is required. From the repository root:

```bash
python3 -m scara_arm_calibration --input examples/sample.json
```

The included four-point example reports an RMS position error of 1.1236 mm before fitting and 0.0137 mm after fitting. These are errors on the points used for calibration, not an independent hardware accuracy measurement.

Copy `examples/sample.json` to use your own nominal geometry, parameter ranges, and paired XY observations. At least four distinct, reachable measured points are required. Add `--json` for machine-readable results.

## Python API

```python
from scara_arm_calibration import Arm, Observation, predict_position, solve_calibration

nominal = Arm(220, 220, -49, 166, 150, -50)
pairs = [
    Observation((50, 50), (50.4, 49.2)),
    Observation((50, 250), (49.1, 248.7)),
    Observation((250, 250), (249.1, 249.7)),
    Observation((250, 50), (250.6, 49.3)),
]
result = solve_calibration(nominal, pairs)
print(result.arm, result.final_rmse)
```

The solver infers the positive distal-angle joint branch from each measured point and fits the six parameters with bounded least squares. Validate the result on separate positions before applying it to a robot. The model covers planar positioning; it does not estimate Z-axis motion, tool orientation, backlash, or compliance.

The package has no runtime dependencies and is available under the [MIT license](LICENSE).
