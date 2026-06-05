# Phase 3B Validation Results

## Scope

Phase 3B added deterministic Jacobian validation tooling only. No MSCKF equations, covariance update code, threading architecture, camera-state pruning policy, or runtime source behavior were changed.

## What Was Tested

`tools/phase3b_jacobian_check.py` uses synthetic deterministic inputs and does not require EuRoC data. It checks:

- `FeatureObservation.cost()` against the squared residual returned by `FeatureObservation.jacobian()`.
- `FeatureObservation.jacobian()` against central finite differences of the inverse-depth residual `z_hat - z`.
- `MSCKF.measurement_jacobian()` residual against direct stereo projection residuals.
- `MSCKF.measurement_jacobian()` camera-state block against central finite differences with respect to camera orientation and position.
- `MSCKF.measurement_jacobian()` feature-position block against central finite differences with respect to feature world position.
- `MSCKF.feature_jacobian()` projected residual dimensions after nullspace projection.
- Nullspace orthogonality, `A.T @ H_fj ~= 0`, for the Jacobian used by `feature_jacobian()`.

## What Could Not Be Fully Validated Yet

- The intended FEJ/observability projection inside `MSCKF.measurement_jacobian()` could not be proven correct from the current code alone.
- The tool compares returned `H_x` and `H_f` to raw finite differences of the implemented stereo residual. A mismatch is strong evidence that the returned blocks are not raw measurement Jacobians, but a manual derivation is still needed before changing equations.
- Direct Jacobian columns for IMU orientation, IMU position, and camera extrinsics were not validated because the current measurement function writes only camera-state blocks; those variables are coupled through state augmentation and covariance cross terms.
- No EuRoC ATE/RPE evaluation was performed in this phase.

## Numerical Tolerances

- Feature inverse-depth Jacobian: `atol=1e-6`, `rtol=1e-5`, central-difference epsilon `1e-6`.
- MSCKF camera orientation perturbation: epsilon `1e-7`.
- MSCKF camera/feature position perturbation: epsilon `1e-6`.
- MSCKF measurement Jacobian comparisons: `atol=1e-5`, `rtol=1e-4`.
- Nullspace orthogonality: expected max absolute error <= `1e-8`.

## Validation Output Summary

```text
[PASS] FeatureObservation.cost == residual^2 max_abs=0.000e+00 max_rel=0.000e+00
[PASS] FeatureObservation.jacobian finite difference max_abs=2.101e-11 max_rel=2.135e-11
[PASS] MSCKF.measurement_jacobian residual max_abs=0.000e+00 max_rel=0.000e+00
[FAIL] MSCKF.measurement_jacobian camera block max_abs=2.021e+00 max_rel=2.000e+00
[FAIL] MSCKF.measurement_jacobian feature block max_abs=4.187e-01 max_rel=2.000e+00
[PASS] MSCKF.feature_jacobian projected dimensions max_abs=0.000e+00 max_rel=0.000e+00
[PASS] MSCKF.feature_jacobian nullspace orthogonality max_abs=4.616e-17 max_rel=0.000e+00
```

Overall result: `FAIL` because the MSCKF measurement Jacobian camera and feature blocks do not match raw central finite differences.

## Commands Run

```powershell
python tools/phase3b_jacobian_check.py
python tools/phase2_sanity_check.py
python tools/smoke_check.py --dataset ./datasets/MH_01_easy
```

`phase3b_jacobian_check.py` intentionally returned exit code 1 because it found failing Jacobian blocks. The existing Phase 2/3A sanity check and dataset smoke check both passed.

## Suspicious Jacobian Areas

- `MSCKF.measurement_jacobian()` camera-state block is suspicious. The returned `H_x` differs from raw finite differences by about `2.021` max absolute error and relative error about `2.0`, which is consistent with sign/frame/projection inconsistency.
- `MSCKF.measurement_jacobian()` feature block is suspicious. The returned `H_f` differs from raw finite differences by about `0.4187` max absolute error and relative error about `2.0`.
- The line that overwrites `H_f` with `-H_x[:4, 3:6]` remains the strongest specific suspect. It may be intentional FEJ/observability logic, but it is not documented and does not match raw finite differences.
- Nullspace projection dimensions and orthogonality are okay for the Jacobians as returned, but that does not prove the underlying `H_f` is mathematically correct.

## Recommended Next Fixes

Do not change equations until this evidence is reviewed manually. Recommended next steps:

1. Manually derive `MSCKF.measurement_jacobian()` for the implemented residual sign `z - h(x)` and current frame conventions.
2. Decide whether the local observability projection in `measurement_jacobian()` is intended FEJ logic or accidental corruption of the raw Jacobian.
3. If the raw finite-difference comparison is the intended standard, replace the camera-state and feature-position Jacobian blocks with derivation-backed formulas in the next approved phase.
4. Add the Phase 3B tool to the regular smoke checklist after the Jacobian decision is made.
5. Keep covariance update, threading, and pruning changes for separate phases.
