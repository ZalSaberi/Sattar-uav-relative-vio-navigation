# Phase 3D FEJ / Observability / MSCKF Consistency Review

## Goal

Phase 3D reviews why the Phase 3B raw finite-difference Jacobian checker flagged
`MSCKF.measurement_jacobian()`, why the Phase 3C raw replacement passed that
checker but failed EuRoC runs, and what should be validated before any further
MSCKF source changes.

No Python source changes are part of this phase.

## Reviewed Files

| File | Review focus |
| --- | --- |
| `src/msckf.py` | `measurement_jacobian()`, `feature_jacobian()`, `gating_test()`, `measurement_update()`, null-state usage, camera-state augmentation/pruning. |
| `src/feature/feature_position_initializer.py` | Feature initialization, depth validity, residual/Jacobian use during inverse-depth optimization. |
| `src/feature/feature_observation.py` | Feature observation residual sign and Jacobian convention for initializer optimization. |
| `src/feature/feature_depth_estimator.py` | Initial depth denominator and positive-depth guards. |
| `tools/phase3b_jacobian_check.py` | Synthetic residual model, finite-difference perturbations, local MSCKF Jacobian checks, nullspace checks. |

## Executive Findings

| Finding | Classification | Summary |
| --- | --- | --- |
| Phase 3B checks `d(z-h)/dx`, while the EKF update uses `H = dh/dx`. | definite bug | `measurement_update()` computes `delta_x = K @ r` with `r = z - h(x)`. In this EKF convention, the matrix called `H` is the measurement-model derivative `dh/dx`, not the residual derivative. Comparing source `H_x` and `H_f` directly against finite differences of `measurement - projection` introduces a sign error. |
| Phase 3C raw replacement inverted the Jacobian convention used by the filter. | definite bug | The raw patch made the tool pass by returning residual derivatives, but that made the update apply corrections in the wrong direction for the current filter convention. This explains why local finite differences passed while MH_01/MH_02 diverged. |
| `measurement_jacobian()` is attempting observability-constrained/FEJ-style logic. | design/scientific concern | The function first builds raw stereo projection derivatives, then projects the camera-state block using `orientation_null`, `position_null`, feature position, and gravity. This is not a plain projection Jacobian. |
| `H_f = -H_x[:4, 3:6]` is likely deliberate, not a raw-Jacobian bug by itself. | needs manual/literature review | For a measurement-model Jacobian, camera translation and feature position derivatives should oppose each other under global translation. Recomputing `H_f` from the adjusted position block appears to preserve that observability relationship, but the derivation is undocumented and must be validated against the intended OC/FEJ MSCKF formulation. |
| `orientation_null` and `position_null` are used consistently as frozen first-estimate/null-state values in the observed paths. | okay for now | IMU null values are updated after propagation. Camera null values are set during state augmentation and left frozen through later measurement updates, which is consistent with a first-estimate style design. |
| `feature_jacobian()` applies the standard MSCKF feature nullspace projection, but its correctness depends on the local `H_f` convention. | design/scientific concern | The SVD projection `A = U[:, 3:]`, `H = A.T @ H_xj`, `r = A.T @ r_j` is structurally standard. If the local `H_f` is the wrong convention, the nullspace removes the wrong subspace. |
| `gating_test()` uses post-projection residual length after Phase 3A. | okay for now | The DOF is `len(r)` after nullspace projection, and thresholds use 0.95 confidence. |
| The Phase 3B validation tool is currently raw-only. | definite bug | It can validate raw residual finite differences, but it cannot yet decide whether the current FEJ/observability-constrained Jacobian is wrong. |

## What The Old `measurement_jacobian()` Was Trying To Compute

The current implementation appears to compute a measurement-model Jacobian for
the stereo projection, not a residual-function Jacobian.

The function computes:

1. Current cam0 and cam1 feature coordinates:
   - `p_c0 = R_w_c0 @ (p_w - t_c0_w)`
   - `p_c1 = R_w_c1 @ (p_w - t_c1_w)`
2. Projection derivatives with respect to camera-frame feature coordinates:
   - `dz_dpc0`
   - `dz_dpc1`
3. Raw measurement derivatives with respect to the camera-state error and feature
   world position:
   - `H_x = dz_dpc0 @ dpc0_dxc + dz_dpc1 @ dpc1_dxc`
   - `H_f = dz_dpc0 @ dpc0_dpg + dz_dpc1 @ dpc1_dpg`
4. An observability-style camera-state correction:
   - `u[:3] = R(orientation_null) @ gravity`
   - `u[3:] = skew(p_w - position_null) @ gravity`
   - `H_x = A - (A @ u)[:, None] * u / (u @ u)`
5. A replacement feature block:
   - `H_f = -H_x[:4, 3:6]`
6. A residual:
   - `r = z - projection`

This is not a raw finite-difference Jacobian of `r`. It is closer to an
observability-constrained MSCKF measurement Jacobian where `H` is the derivative
of the predicted measurement `h(x)`, and `r = z - h(x)` is passed separately to
the Kalman update.

## Why Raw Finite Differences Passed But End-To-End VIO Failed

`tools/phase3b_jacobian_check.py` defines:

```python
stereo_residual = measurement - project_stereo(...)
```

Then it finite-differences that residual function and compares the result
directly to `H_x` and `H_f`.

That checks:

```text
d(z - h(x)) / dx = -dh(x) / dx
```

The filter update in `measurement_update()` uses the standard linearized
measurement equation:

```text
z = h(x) + H * delta_x + noise
r = z - h(x)
delta_x = K * r
K = P * H.T * inv(H * P * H.T + R)
```

Under that convention, `H` should be `dh/dx`, not `d(z-h)/dx`. If `H` is
replaced with `-dh/dx`, the Kalman gain changes sign and the state correction
is applied in the wrong direction, while the covariance innovation term is
unchanged in magnitude. That is a high-confidence explanation for the Phase 3C
outcome: the synthetic residual finite-difference check passed, but full VIO
updates diverged badly.

The raw finite-difference replacement also removed the observability-constrained
projection. Even if the sign convention were corrected, removing that projection
would still be unsafe without validating the filter's FEJ/observability design.

## `H_f = -H_x[:4, 3:6]`

Classification: needs manual/literature review.

This line does not match a raw finite-difference feature-position derivative
after `H_x` has been observability-adjusted. However, it looks deliberate.

For the raw measurement model:

```text
h = project(R * (p_f - p_c))
```

camera translation and feature position enter with opposite signs. A global
translation of both camera and feature should be unobservable. After the code
modifies the camera-state block to enforce an observability/nullspace condition,
setting:

```python
H_f = -H_x[:, 3:6]
```

preserves that translation relationship for the adjusted Jacobian. Therefore it
should not be treated as a definite bug merely because it fails a raw residual
finite-difference test.

The risk is that this relation is undocumented and may only be correct for a
specific OC/FEJ derivation. Phase 3E should validate the intended property
directly before Phase 3F changes any source.

## Null-State Usage

| Area | Classification | Review |
| --- | --- | --- |
| IMU propagation null state | okay for now | Propagation uses `orientation_null`, `velocity_null`, and `position_null` to adjust transition blocks, then updates the null values to the propagated state. This is consistent with an observability-constrained propagation path. |
| Camera-state null values | okay for now | `state_augmentation()` copies current camera orientation and position into `orientation_null` and `position_null`. Measurement updates modify `cam_state.orientation` and `cam_state.position` but do not update the null values, which is consistent with first-estimate behavior for cloned camera states. |
| Measurement null vector `u` | needs manual/literature review | `u` uses the frozen camera null orientation/position, gravity, and current initialized feature position. This resembles OC-MSCKF logic, but the exact derivation should be checked against the intended formulation. |
| Feature position first estimate | design/scientific concern | Feature position is initialized from current camera states and then used in the measurement Jacobian. There is no separate stored feature-null estimate. This may be acceptable for MSCKF feature elimination, but it should be reviewed when validating FEJ consistency. |

## `feature_jacobian()` And Nullspace Projection

Classification: okay for now, with dependency risk.

`feature_jacobian()` stacks one 4-row stereo residual/Jacobian block per camera
state, stacks `H_f`, and applies:

```python
U, _, _ = np.linalg.svd(H_fj)
A = U[:, 3:]
H_x = A.T @ H_xj
r = A.T @ r_j
```

For `n` stereo observations this yields `4*n - 3` projected residual rows. This
is the expected MSCKF feature-nullspace elimination shape. Phase 3B's dimension
and orthogonality checks are useful, but they only prove that `A.T @ H_fj` is
near zero for whatever `H_fj` the current code returns. They do not prove that
`H_fj` is the correct FEJ/OC feature block.

## `gating_test()`

Classification: okay for now.

After Phase 3A, `gating_test()` uses:

```python
dof = len(r)
threshold = chi2.ppf(0.95, dof)
```

Since callers pass the nullspace-projected residual from `feature_jacobian()`,
the degrees of freedom now match the residual dimension actually being gated.

## `measurement_update()`

Classification: design/scientific concern.

The update uses:

```python
delta_x = K @ r
```

with `r = z - h(x)`. This confirms that `H` should be interpreted as the
measurement-model derivative `dh/dx`.

The covariance update still uses the simplified form:

```python
state_cov = (I - K H) @ P
```

This was explicitly out of scope for Phase 3A and remains out of scope here. It
should not be changed during Phase 3D or Phase 3E.

## Is The Project Using FEJ Or Observability-Constrained Logic?

Classification: design/scientific concern.

The project is not a plain raw-Jacobian EKF. It contains multiple signs of an
observability-constrained or FEJ-inspired MSCKF:

1. IMU propagation modifies transition blocks using null-state values and
   gravity.
2. Camera clones store `orientation_null` and `position_null` at augmentation.
3. Measurement Jacobians use those null camera values to build `u`.
4. `H_x` is projected to remove the component along `u`.
5. `H_f` is overwritten from the adjusted camera-position block.

The implementation may be copied from or inspired by an OC-MSCKF formulation.
However, the code does not document the derivation, and the validation tooling
currently treats the function as a raw residual Jacobian. That is the main Phase
3D consistency problem.

## Answers To Phase 3D Questions

1. What exactly was the old `measurement_jacobian()` trying to compute?

   It was trying to compute a stereo measurement-model Jacobian `dh/dx` plus an
   observability-constrained adjustment based on frozen null-state values. It
   was not trying to return the raw finite-difference derivative of
   `measurement - projection`.

2. Why does the raw finite-difference Jacobian pass the synthetic test but fail
   end-to-end VIO?

   The synthetic test finite-differences the residual `z - h(x)`, so a
   residual-derivative Jacobian passes. The filter update expects `dh/dx`.
   Replacing `dh/dx` with `-dh/dx` reverses the state correction direction.
   The raw patch also removed the observability-constrained projection, which
   further breaks consistency assumptions.

3. Is `H_f = -H_x[:4, 3:6]` deliberate FEJ/observability approximation, a bug,
   or unclear?

   It is likely deliberate OC/FEJ-style logic, not a definite bug. It remains
   unclear whether the exact implementation is correct until the intended
   observability constraint is derived and validated.

4. Are `orientation_null` and `position_null` used consistently?

   In the reviewed paths, yes. IMU null states are used during propagation and
   refreshed afterward. Camera null states are set at augmentation and kept
   frozen through later updates. That is consistent with first-estimate style
   camera clones.

5. Is the Phase 3B validation tool testing the correct Jacobian for this MSCKF
   formulation, or only a raw Jacobian?

   It is testing only a raw residual Jacobian. It is not testing the
   measurement-model sign convention or the FEJ/observability-constrained
   Jacobian that the current filter appears to use.

6. What should be changed in the validation tool before source code fixes?

   The tool should separate three checks:

   - residual check: verify `r == z - h(x)`;
   - raw measurement-model Jacobian check: compare source raw formulas to
     `dh/dx`, or compare `-H_raw` to finite differences of `z - h(x)`;
   - OC/FEJ Jacobian check: independently reproduce the current null-vector
     projection and verify observability properties such as `H_x @ u ~= 0`.

   It should also use synthetic fixtures where the current camera pose differs
   from `orientation_null` and `position_null`, otherwise the FEJ path is weakly
   exercised.

7. What source-code fix should be tried first, but only after the validation
   tool is corrected?

   First try no MSCKF source fix if the corrected tool shows the existing source
   is consistent with the intended OC/FEJ formulation. If a bug remains proven,
   the first source change should be a minimal `measurement_jacobian()` fix that
   preserves the measurement-model sign convention and the observability
   projection. Do not repeat the Phase 3C raw replacement.

8. What should not be changed yet?

   Do not change covariance update form, threading, camera-state pruning,
   feature initialization math, gating confidence/DOF, or the OC/FEJ projection
   itself until Phase 3E validation proves a specific source bug.

## Safe Implementation Roadmap

### Phase 3E: Update Validation Tool To Be FEJ-Aware

1. Keep `FeatureObservation.cost()` and `FeatureObservation.jacobian()` checks.
2. Rename the current MSCKF finite-difference check so it is explicit that it
   tests `d(z-h)/dx`.
3. Add a measurement-model Jacobian check:
   - finite-difference `project_stereo(...)`;
   - compare against raw `dh/dx` formulas, not residual derivatives.
4. Add an OC/FEJ expected-Jacobian check in the tool:
   - compute raw `A = dh/dx_cam`;
   - compute `u` from `orientation_null`, `position_null`, feature position,
     and gravity;
   - compute `H_x_oc = A - (A @ u)[:, None] * u / (u @ u)`;
   - compute expected `H_f_oc = -H_x_oc[:, 3:6]`;
   - compare source `measurement_jacobian()` against those expected OC blocks.
5. Add direct observability checks:
   - `H_x_oc @ u` should be near zero;
   - global translation consistency should hold for the adjusted camera and
     feature blocks;
   - finite values and positive depth should still be enforced.
6. Add at least one fixture where current pose and null pose differ.
7. Keep tolerances near the existing Phase 3B values:
   - raw finite differences: `atol <= 1e-5`, `rtol <= 1e-4`;
   - algebraic OC projection equality: `atol <= 1e-10` where no finite
     differencing is involved;
   - observability orthogonality: `<= 1e-8`.

### Phase 3F: Apply Minimal Source Fix Only If FEJ-Aware Validation Proves It

1. If only the Phase 3B sign convention was wrong, do not change
   `src/msckf.py`.
2. If the FEJ-aware tool proves a source bug, change only the smallest part of
   `MSCKF.measurement_jacobian()` needed to satisfy the derived OC/FEJ
   property.
3. Preserve `H = dh/dx` for `measurement_update()`.
4. Preserve feature nullspace projection in `feature_jacobian()`.
5. Do not modify covariance update, threading, or camera-state pruning in this
   phase.

### Phase 3G: Retest EuRoC Behavior

1. Run `tools/phase3b_jacobian_check.py` after it becomes FEJ-aware.
2. Run `tools/phase2_sanity_check.py`.
3. Run `tools/smoke_check.py --dataset ./datasets/MH_01_easy`.
4. Run MH_01 and MH_02 offset-10 end-to-end tests with isolated output dirs.
5. Compare final tails against Phase 3A stable outputs and the rejected Phase 3C
   divergent outputs.
6. Only then decide whether a source fix is safe to commit.

## Recommended Next Step

Proceed to Phase 3E: update `tools/phase3b_jacobian_check.py` so it is explicit
about residual-vs-measurement Jacobian sign and can validate the current
FEJ/observability-constrained formulation. Source-code fixes should wait until
that tool distinguishes a real MSCKF bug from a raw-validation mismatch.
