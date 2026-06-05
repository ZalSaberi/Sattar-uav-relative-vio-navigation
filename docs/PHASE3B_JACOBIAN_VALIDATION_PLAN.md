# Phase 3B Jacobian Validation Plan

## Goal

Phase 3B is a review and validation phase only. The goal is to validate measurement residuals, analytic Jacobians, nullspace projection, and post-Phase-3A gating behavior before any MSCKF equation changes are approved. No source code, covariance update, threading architecture, or camera-state pruning changes are part of this plan.

- Baseline branch: `codex/runtime-baseline`
- Working branch: `codex/phase3b-jacobian-validation`

## Residual Functions

| Function | Residual computed | Notes |
|---|---|---|
| `src/feature/feature_observation.py::FeatureObservation.cost()` | Squared reprojection error `sum((z_hat - z)^2)` for inverse-depth feature initialization. | Used only inside feature position initialization optimization. |
| `src/feature/feature_observation.py::FeatureObservation.jacobian()` | Two-row residual `r = z_hat - z` and inverse-depth Jacobian. | Residual sign is opposite of the MSCKF update residual; this is acceptable only if optimizer equations are consistent. |
| `src/msckf.py::MSCKF.measurement_jacobian()` | Four-row stereo residual `r = z - [u0_hat, v0_hat, u1_hat, v1_hat]`. | Computes one camera-state block Jacobian and one feature-position Jacobian. |
| `src/msckf.py::MSCKF.feature_jacobian()` | Stacked residuals for one feature, then projected with the left nullspace of `H_f`. | Produces the residual actually passed to gating/update. |
| `src/msckf.py::MSCKF.gating_test()` | Mahalanobis statistic `gamma = r.T S^-1 r`. | After Phase 3A, DOF is `len(r)` after nullspace projection. |

## Measurement Jacobian Functions

| Function | Jacobian | State variables involved |
|---|---|---|
| `FeatureObservation.jacobian()` | `J = d(z_hat - z) / d[alpha, beta, rho]` for inverse-depth initialization. | Depends directly on inverse-depth feature variables and relative camera pose `T_c0_ci`; indirectly depends on camera state orientation/position and stereo extrinsics used to build those poses. |
| `MSCKF.measurement_jacobian()` | Local `H_x` for one camera state and `H_f` for the 3D feature position. | Depends directly on camera state orientation/position, static cam0-cam1 extrinsics, feature world position, observation vector, and gravity/null-state projection terms. It does not directly fill IMU orientation, IMU position, or extrinsic columns in the global matrix; those are coupled through camera-state covariance from augmentation. |
| `MSCKF.feature_jacobian()` | Stacked global `H_xj`, stacked `H_fj`, then projected `H_x = A.T H_xj`. | Depends on all camera states that observed the feature and the feature position. The output dimensions should be `(4*n - 3, 21 + 6*num_cam_states)` after nullspace projection. |
| `MSCKF.state_augmentation()` | Camera-state augmentation covariance Jacobian `J`, not a measurement Jacobian. | Depends on IMU orientation, IMU position, cam0 extrinsic rotation, and cam0 extrinsic translation. It should be manually reviewed because measurement updates can affect IMU/extrinsics through cross-covariance. |

## State Dependency Map

| Variable | Direct measurement dependency | Current path |
|---|---|---|
| IMU orientation | Indirect for measurement updates; direct for state augmentation. | Camera states are created from IMU orientation in `state_augmentation()`. Measurement rows are written into camera-state blocks, not direct IMU orientation columns. |
| IMU position | Indirect for measurement updates; direct for state augmentation. | Camera-state position is created from IMU position and cam0 extrinsic translation. |
| Camera extrinsics | Indirect in current MSCKF measurement matrix; direct in augmentation and stereo cam1 projection convention. | Extrinsic state can update through covariance cross-blocks. The measurement Jacobian does not explicitly fill extrinsic columns. |
| Feature position | Direct. | `MSCKF.measurement_jacobian()` computes `H_f`; `feature_jacobian()` removes it by nullspace projection. |
| Camera state orientation/position | Direct. | `MSCKF.measurement_jacobian()` computes the 6-column local camera-state Jacobian and inserts it into the global camera-state block. |

## Nullspace Projection

Nullspace projection is applied in `MSCKF.feature_jacobian()`:

1. Stack per-observation `H_xi`, `H_fi`, and `r_i`.
2. Compute `U, _, _ = np.linalg.svd(H_fj)`.
3. Use `A = U[:, 3:]` as the left nullspace basis for the 3-column feature Jacobian.
4. Return `H_x = A.T @ H_xj` and `r = A.T @ r_j`.

Expected dimensions: for `n` stereo camera-state observations, raw residual rows are `4*n`, feature nullity removes 3 rows, and projected residual rows should be `4*n - 3`. After Phase 3A, features with `stack_count <= 3` are rejected before projection.

`MSCKF.measurement_jacobian()` also applies an observability-style projection to the local camera Jacobian and then overwrites `H_f` with `-H_x[:4, 3:6]`. This is not the MSCKF feature nullspace projection and needs manual derivation plus finite-difference validation before any code change.

## Chi-Square Gating After Phase 3A

Gating is applied after `feature_jacobian()` has returned projected `H_xj, r_j`:

- `remove_lost_features()` calls `gating_test(H_xj, r_j)` before adding a lost feature to the update stack.
- `prune_cam_state_buffer()` calls `gating_test(H_xj, r_j)` before adding constraints from pruned camera states.
- `gating_test()` uses `dof = len(r)` and the 0.95 chi-square threshold.

This is okay for now, but Phase 3B tests should pin the projected residual size and threshold behavior so later Jacobian changes do not silently change gating semantics.

## Finite-Difference Check Plan

Use deterministic central differences. Keep all synthetic points safely in front of both cameras and away from near-zero depth.

| Target | Perturb variables | Compare against | Suggested epsilon | Tolerance |
|---|---|---|---|---|
| `FeatureObservation.jacobian()` | Inverse-depth variables `[alpha, beta, rho]`. | Numerical derivative of `z_hat - z`. | `1e-6` for all three variables. | `atol <= 1e-6`, `rtol <= 1e-5`. |
| `MSCKF.measurement_jacobian()` camera block | Camera orientation small-angle perturbation and camera position perturbation. | Numerical derivative of `z - h(cam_state, feature)` for the four stereo residual rows. | `1e-7` for rotation, `1e-6` m for position. | Start with `atol <= 1e-5`, `rtol <= 1e-4`; tighten if stable. |
| `MSCKF.measurement_jacobian()` feature block | Feature world position perturbation. | Numerical derivative of `z - h(cam_state, feature)`. | `1e-6` m. | `atol <= 1e-5`, `rtol <= 1e-4`. |
| `MSCKF.feature_jacobian()` nullspace output | Camera state and feature perturbations across two or more observations. | `A.T` times the finite-differenced raw residual stack. | Same as local checks. | `atol <= 1e-5`, `rtol <= 1e-4`. |
| Nullspace orthogonality | None. | Verify `A.T @ H_fj ~= 0` and projected row count `4*n - 3`. | Not applicable. | `atol <= 1e-8`. |
| Gating input shape | None. | Verify `len(r_projected)` equals DOF and gate passes zero residual/rejects large residual. | Not applicable. | Exact shape checks; boolean gate checks. |

Manual sign convention review is required before declaring a mismatch a bug. The finite-difference residual must use the same residual sign as the function under test.

## Synthetic Test Cases

These can run without EuRoC data:

1. Identity cam0 pose with a feature at `[0.5, 0.1, 4.0]` and a small stereo baseline.
2. Nontrivial camera orientation and translation with feature depth between 3 m and 8 m.
3. Two and three camera-state observations of the same feature to validate nullspace dimensions `5` and `9` respectively.
4. Feature initialized from stereo observations with known positive depth.
5. Near-zero or negative depth feature to confirm Phase 3A guards reject it.
6. Large residual synthetic observation to confirm chi-square rejection after projection.
7. Small residual synthetic observation to confirm chi-square acceptance.
8. Perturb cam0-cam1 extrinsic convention manually in the synthetic fixture to see which residual rows are sensitive to baseline sign.

## Signs of Incorrect Jacobians

- Finite-difference and analytic columns disagree in sign or scale.
- Projected residual dimension is not `4*n - 3`.
- `A.T @ H_fj` is not close to zero.
- Very small pixel residuals produce large state updates or `[Warning] Update change is too large`.
- Gating rejects near-zero residual features or accepts deliberately large residuals.
- Runs remain bounded but show biased drift, oscillatory corrections, or abrupt jumps around lost-feature/pruning updates.
- State updates improve one camera residual while worsening the other stereo residual consistently.

## Issue Classification

| Issue | Classification | Rationale |
|---|---|---|
| Missing finite-difference tests for `FeatureObservation.jacobian()` | scientific/design concern | The formula looks plausible, but there is no automated proof against sign/convention drift. |
| `MSCKF.measurement_jacobian()` overwrites `H_f` after local observability projection | likely bug | The feature Jacobian should represent derivative with respect to feature position before nullspace elimination. This may be intentional FEJ-style logic, but it is undocumented and must be derived/tested. |
| Direct measurement Jacobian does not fill IMU/extrinsic columns | scientific/design concern | This may be acceptable because camera states are separate state variables with cross-covariance from augmentation, but the intended observability/extrinsic update path needs manual review. |
| Nullspace projection row count and Phase 3A DOF behavior | okay for now | The implementation now uses projected residual length; tests should lock this in. |
| Residual sign mismatch between feature initialization and MSCKF update | okay for now | Different optimizers can use opposite residual signs if their update equations are consistent, but tests should document it. |
| Camera-state Jacobian signs/frame convention | likely bug until validated | The code mixes `R_w_c0`, `R_w_c1`, `t_c0_w`, and stereo extrinsics. A sign error here would directly corrupt updates. |
| State augmentation Jacobian | scientific/design concern | Not a measurement Jacobian, but it controls how camera measurements affect IMU/extrinsic state through covariance. It should be reviewed before changing measurement equations. |

No definite Jacobian bug should be fixed until a finite-difference test or written derivation isolates it.

## Proposed Implementation Plan for Next Phase

### Files to Test

- `src/feature/feature_observation.py`
- `src/feature/feature_position_initializer.py`
- `src/msckf.py`
- `src/utils.py`
- `src/feature/utils.py`

### Functions to Test

- `FeatureObservation.cost()`
- `FeatureObservation.jacobian()`
- `FeatureDepthEstimator.generate_initial_guess()` as setup/guard coverage
- `FeaturePositionInitializer.initialize_position()` as integration coverage
- `MSCKF.measurement_jacobian()`
- `MSCKF.feature_jacobian()`
- `MSCKF.gating_test()` shape/threshold behavior
- `MSCKF.state_augmentation()` only as a related covariance-coupling check

### Recommended Test File Names

If a test framework is introduced:

- `tests/test_feature_observation_jacobian.py`
- `tests/test_msckf_measurement_jacobian.py`
- `tests/test_msckf_nullspace_gating.py`

If keeping the current lightweight-tool style:

- `tools/phase3b_jacobian_check.py`

The first implementation should probably use `tools/phase3b_jacobian_check.py` to stay consistent with the current smoke-check approach and avoid adding a broad test framework in the same phase.

### Expected Numerical Tolerances

- Inverse-depth Jacobian: `atol=1e-6`, `rtol=1e-5`.
- Camera orientation Jacobian: `atol=1e-5`, `rtol=1e-4` with small-angle perturbations of `1e-7`.
- Camera/feature position Jacobians: `atol=1e-5`, `rtol=1e-4` with perturbations of `1e-6` m.
- Nullspace orthogonality: `atol=1e-8`.
- Gating threshold/DOF: exact DOF checks; boolean pass/fail checks for zero and large residuals.

### Risks of Changing Each Jacobian

- `FeatureObservation.jacobian()`: affects feature initialization convergence and depth acceptance; a sign change can make the optimizer move away from the minimum.
- `MSCKF.measurement_jacobian()` camera block: high risk; sign/frame mistakes directly destabilize pose, velocity, and bias through the Kalman update.
- `MSCKF.measurement_jacobian()` feature block: high risk; wrong `H_f` makes nullspace projection remove the wrong subspace.
- `feature_jacobian()` nullspace projection: high risk; dimension or basis errors change gating DOF and update content.
- `state_augmentation()` Jacobian: high risk but separate from measurement residuals; changing it affects covariance cross-correlations and extrinsic observability.

### Manual Review Before Code Changes

- Confirm frame notation for `R_w_c`, `t_c_w`, `T_imu_cam0`, and `T_cn_cnm1` against EuRoC/Kalibr conventions.
- Confirm left vs right quaternion perturbation convention used by `small_angle_quaternion()` and `quaternion_multiplication()`.
- Confirm whether the local observability projection in `measurement_jacobian()` is intended FEJ logic.
- Confirm whether extrinsic states should receive direct measurement Jacobian columns or only cross-covariance updates through camera states.
- Confirm residual sign conventions separately for feature initialization and MSCKF update.
- Confirm the baseline direction for cam0-to-cam1 projection and the derivative of the cam1 residual with respect to cam0 camera-state perturbations.

## Recommended Next Steps

1. Add `tools/phase3b_jacobian_check.py` with deterministic synthetic fixtures and central-difference helpers.
2. Validate `FeatureObservation.jacobian()` first because it is isolated and lower risk.
3. Validate raw `MSCKF.measurement_jacobian()` camera and feature blocks before nullspace projection.
4. Validate `feature_jacobian()` nullspace dimensions, orthogonality, and projected finite differences.
5. Only after a failing check is reproducible, propose the smallest source-code fix with before/after numeric evidence.
