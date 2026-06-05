# Phase 3E FEJ-Aware Validation Results

## Goal

Phase 3E updates the Jacobian validation strategy after the rejected Phase 3C
experiment. The goal is to distinguish:

1. the raw residual derivative `d(z - h) / dx`;
2. the measurement-model derivative `dh / dx`;
3. the filter-compatible Jacobian used by `MSCKF.measurement_update()`;
4. the FEJ / observability-constrained projection currently implemented in
   `MSCKF.measurement_jacobian()`.

No MSCKF source behavior was changed.

## What The Phase 3B Checker Got Wrong Or Incomplete

The Phase 3B checker finite-differenced:

```python
measurement - project_stereo(...)
```

and compared that residual derivative directly against the `H_x` and `H_f`
returned by `MSCKF.measurement_jacobian()`.

That was incomplete for this project because `measurement_update()` uses:

```python
r = z - h(x)
delta_x = K @ r
```

With that update convention, the filter-compatible measurement matrix is
`H = dh/dx`, not `d(z-h)/dx`. Therefore the raw residual derivative is expected
to be the negative of the measurement-model derivative.

Phase 3B also did not account for the observability projection in
`measurement_jacobian()`:

```python
H_x = A - (A @ u)[:, None] * u / (u @ u)
H_f = -H_x[:, 3:6]
```

So it could detect a raw finite-difference mismatch, but it could not decide
whether the current FEJ/observability-constrained Jacobian was wrong.

## New Checker

Created:

```text
tools/phase3e_fej_aware_check.py
```

The new checker uses deterministic synthetic fixtures with:

- fixed camera poses;
- fixed feature position with positive stereo depth;
- deterministic nonzero measurement residuals;
- camera current pose and frozen null pose intentionally different;
- no EuRoC dataset requirement.

It validates:

1. source residual equals `z - h(x)`;
2. raw residual derivative equals negative measurement-model derivative;
3. unprojected analytic `dh/dx` camera and feature blocks match central finite
   differences of `h(x)`;
4. current returned `H_x` and `H_f` are not incorrectly judged against raw
   residual finite differences;
5. current returned `H_x` and `H_f` match the code-derived FEJ/OC projection;
6. `H_x @ u ~= 0` for the gravity/null-state observability direction;
7. `H_f = -H_x[:, 3:6]`;
8. `feature_jacobian()` nullspace dimensions and orthogonality remain valid;
9. projected `H` and `r` match explicit `A.T @ H_x` and `A.T @ r`;
10. the projected residual/Jacobian pair has the sign convention expected by
    `measurement_update()`.

## Tolerances

```text
finite-difference atol = 1e-5
finite-difference rtol = 1e-4
algebraic atol = 1e-10
observability atol = 1e-8
rotation finite-difference epsilon = 1e-7
position finite-difference epsilon = 1e-6
```

These tolerances were not weakened from the Phase 3B finite-difference scale.
The stricter algebraic checks are used only where no finite differencing is
involved.

## Command Results

Command:

```text
python tools/phase3e_fej_aware_check.py
```

Result:

```text
Overall: PASS
```

Important check outputs:

```text
[PASS] residual check: source r equals z - h(x)
[PASS] raw residual derivative check: d(z-h)/dx = -dh/dx camera block
[PASS] raw residual derivative check: d(z-h)/dx = -dh/dx feature block
[PASS] measurement model derivative check: raw camera dh/dx max_abs=2.028e-10 max_rel=2.007e-10
[PASS] measurement model derivative check: raw feature dh/dx max_abs=1.564e-11 max_rel=7.471e-11
[PASS] current MSCKF returned H check: not raw residual derivative max_abs=2.021e+00 max_rel=1.999e+00
[PASS] current MSCKF returned H check: not unprojected raw dh/dx max_abs=1.136e-02 max_rel=1.997e-02
[PASS] current MSCKF returned H check: FEJ/OC camera block
[PASS] current MSCKF returned H check: FEJ/OC feature block
[PASS] FEJ / observability consistency check: H_x @ u ~= 0 max_abs=1.110e-16
[PASS] FEJ / observability consistency check: H_f = -H_x position block
[PASS] nullspace projection check: projected dimensions
[PASS] nullspace projection check: A.T @ H_f ~= 0 max_abs=6.201e-17
[PASS] nullspace projection check: H projection matches explicit A.T @ H_x
[PASS] nullspace projection check: residual projection matches explicit A.T @ r
[PASS] filter-compatible Jacobian check: projected update sign ||r||=7.743e-03, ||r-H*dx||=5.693e-03, ||r+H*dx||=1.063e-02
```

## Interpretation

The new checker supports the Phase 3D conclusion:

- the old raw residual finite-difference failure was not enough evidence for a
  source-code Jacobian replacement;
- the unprojected analytic measurement-model derivative `dh/dx` is numerically
  correct for the deterministic synthetic fixture;
- the current `MSCKF.measurement_jacobian()` returns an FEJ/OC-style projected
  measurement-model Jacobian, not a raw residual derivative;
- `H_f = -H_x[:, 3:6]` is consistent with the current projection/update path in
  the checked algebraic sense;
- the current projected residual/Jacobian pair has the sign convention expected
  by `measurement_update()`.

## What Still Cannot Be Validated Yet

The tool still does not prove the full MSCKF formulation correct. It does not
fully validate:

- the literature derivation of the OC/FEJ projection vector `u`;
- whether using the current initialized feature position inside `u` is the
  correct first-estimate choice;
- all observability nullspace dimensions of the full IMU/camera/extrinsic
  system;
- covariance consistency or positive semi-definiteness after repeated updates;
- camera-state pruning interactions with long feature tracks;
- EuRoC ATE/RPE accuracy against ground truth.

## Is A Source Fix Recommended?

No source-code Jacobian fix is justified by Phase 3E evidence.

The Phase 3C raw replacement should remain rejected. The current source passes
the FEJ-aware synthetic validation added in this phase, so changing
`src/msckf.py` now would be speculative.

## Exact Next Source-Code Change

None.

Only if a future, stronger FEJ/observability derivation proves a mismatch should
the next source change be a minimal edit inside `MSCKF.measurement_jacobian()`
that preserves:

- `H = dh/dx` for `measurement_update()`;
- the FEJ/observability projection intent;
- `feature_jacobian()` nullspace compatibility;
- the stable MH_01/MH_02 behavior recovered before the rejected raw Jacobian
  experiment.

## Recommended Next Phase

Proceed to a broader consistency phase rather than a Jacobian source patch:

1. compare the current OC/FEJ projection against the intended MSCKF/OC-MSCKF
   derivation;
2. add full-system observability checks for the stacked update;
3. evaluate MH_01/MH_02 against EuRoC ground truth using ATE/RPE;
4. defer covariance update changes to a separate, explicitly scoped phase.
