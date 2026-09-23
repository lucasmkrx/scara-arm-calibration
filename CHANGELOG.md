# Changelog

Notable changes to this repository are recorded here. The current revision is **unreleased**; the `0.1.0` package version is project metadata, not a published release or a hardware validation result.

## Unreleased

### Calibration model and solver

- Replaced the SciPy `trust-constr` call and sum of per-point distances with a bounded, damped least-squares fit of Cartesian X and Y residuals. The new solver uses an analytic Jacobian and a small dense linear-system routine, so calibration has no runtime dependencies.
- Replaced the left/right `asin` inverse-kinematics branches with `atan2` and the cosine law. The implementation consistently uses the positive distal-angle branch.
- Replaced module-level result variables with immutable `Arm`, `Observation`, and `CalibrationResult` data objects. Results now include fitted parameters, per-point residuals, RMS and maximum position errors, convergence status, and the RMS history used by the convergence figure.
- Added checks for finite and positive inputs, at least four distinct measured points, nominal reachability, and point layouts that cannot identify all six parameters.
- Added `predict_position` to apply a fitted arm to a new measured XY point. It can be used with held-out target positions to assess generalization.

### Interface and project structure

- Moved the hard-coded calibration example out of Python source into [`examples/sample.json`](examples/sample.json). Each observation pairs a measured position with its intended target; parameter ranges are configurable in the same file.
- Added `python3 -m scara_arm_calibration --input <file>`, optional `--json` output, and an installable `scara-calibrate` command.
- Added [`pyproject.toml`](pyproject.toml) and removed `requirements.txt`, the old `main.py` script, and terminal-color utility.
- Rewrote the [README](README.md) with a working quick start, model equations, input format, API example, assumptions, and limits.

### Evidence and presentation

- Added three standalone SVG figures: a planar kinematic schematic, a per-point error plot, and a solver convergence plot. [`scripts/generate_figures.py`](scripts/generate_figures.py) regenerates them from the included example using only the standard library.
- Added eight automated tests covering synthetic parameter recovery, predictions, input rejection, the CLI, and JSON output. [GitHub Actions CI](.github/workflows/ci.yml) is configured to test multiple Python versions, the installed command, and figure reproducibility.
- On the **four example points used for fitting**, RMS position error changes from **1.1236 mm to 0.0137 mm**. This is a training fit, not a measurement of performance on unseen positions or physical hardware.

### Migration from the previous layout

| Previous workflow | Current workflow |
| --- | --- |
| Edit `Points` and `Initial` classes in `scara_arm_calibration/main.py` | Edit a copy of `examples/sample.json` |
| Run `python3 scara_arm_calibration/main.py` | Run `python3 -m scara_arm_calibration --input path/to/input.json` |
| Read solver globals and an unlabeled list of six fitted values | Read fields on `CalibrationResult` and `result.arm`, or use `--json` |
| Install NumPy and SciPy from `requirements.txt` | Run directly with Python 3.10+, or install the package with `python3 -m pip install -e .` |

The solver's objective and result interface changed, so numerical outputs should be compared using the stated RMS and residual definitions rather than against the old objective value. The example data are retained for continuity; validate any calibration on separate, well-spaced positions before using it to adjust a robot.
