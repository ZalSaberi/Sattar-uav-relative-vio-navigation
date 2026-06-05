# Phase 3C Raw Jacobian Experiment

## Summary

Phase 3C tested a minimal raw finite-difference fix for `MSCKF.measurement_jacobian()`. The experiment made `tools/phase3b_jacobian_check.py` pass, but it caused severe end-to-end VIO divergence on `MH_01_easy` and `MH_02_easy`. The source patch should not be committed as a fix.

## What Was Changed

The experimental patch changed only `src/msckf.py` inside `MSCKF.measurement_jacobian()`.

The current residual is:

```python
r = measurement - projection
```

The experiment returned raw Jacobians for that residual:

```python
H_x = -(projection derivative wrt camera pose)
H_f = -(projection derivative wrt feature position)
```

It also removed the existing local observability-style projection:

```python
H_x = A - (A @ u)[:, None] * u / (u @ u)
H_f = -H_x[:4, 3:6]
```

## Why The Finite-Difference Check Passed

`tools/phase3b_jacobian_check.py` compares `H_x` and `H_f` against central finite differences of the implemented raw stereo residual `measurement - projection`.

With the experimental patch, the returned `H_x` and `H_f` matched that raw residual convention:

```text
PASS MSCKF.measurement_jacobian camera block  max_abs=2.028e-10 max_rel=2.007e-10
PASS MSCKF.measurement_jacobian feature block max_abs=1.564e-11 max_rel=7.471e-11
```

Before the experiment, those blocks failed the raw finite-difference check:

```text
FAIL MSCKF.measurement_jacobian camera block  max_abs=2.021e+00 max_rel=2.000e+00
FAIL MSCKF.measurement_jacobian feature block max_abs=4.187e-01 max_rel=2.000e+00
```

## Why End-To-End VIO Regressed

Although the raw Jacobian matched the synthetic finite-difference residual, it was not compatible with the current filter formulation. The existing code appears to rely on a local observability/FEJ-style projection or another consistency convention inside `measurement_jacobian()`. Removing that projection made the local Jacobian mathematically raw, but the rest of the filter was still tuned around the previous formulation.

The result was severe divergence in full EuRoC runs, despite the synthetic Jacobian tool passing.

## Divergent Tail Values

Final `MH_01_easy --offset 10` tail under the raw Jacobian experiment:

```text
1403636763.463556 -1669.904897960 -9095.173539264 -138394.507286376 0.767088220 0.290383639 0.081652911 -0.566202973
1403636763.513556 -1669.845407961 -9098.729908185 -138477.125815081 0.736930629 0.267684919 0.223911457 -0.578914235
1403636763.563556 -1669.770527764 -9102.290819496 -138559.750342348 0.690533691 0.239727325 0.361956778 -0.578516483
1403636763.613555 -1669.685631080 -9105.861687408 -138642.378029162 0.629051528 0.206143732 0.491663517 -0.565743690
1403636763.663556 -1669.596579443 -9109.446189663 -138725.010710386 0.553554656 0.168232506 0.610945986 -0.540388812
1403636763.713556 -1669.510018642 -9113.047388233 -138807.649311973 0.466128021 0.126459946 0.716400487 -0.503490707
1403636763.763556 -1669.431959794 -9116.666827190 -138890.298080258 0.368541104 0.082083764 0.806592061 -0.454806504
1403636763.813555 -1669.368338585 -9120.303860130 -138972.960447574 0.262514600 0.035797198 0.878624884 -0.397269377
```

Final `MH_02_easy --offset 10` tail under the raw Jacobian experiment:

```text
1403637010.251667 6363.476123422 15142.819582610 -31899.349572451 0.877097307 0.363123147 0.015774470 0.313995320
1403637010.301667 6368.835346959 15158.308713547 -31924.314832955 0.842274877 0.470742748 -0.048268127 0.258155930
1403637010.351666 6374.203648022 15173.819204287 -31949.307545578 0.790557631 0.569058669 -0.110968225 0.197172301
1403637010.401667 6379.576058158 15189.352562036 -31974.329106749 0.723092128 0.655885880 -0.171676477 0.132206939
1403637010.451667 6384.947686771 15204.908982774 -31999.378885720 0.641200512 0.729582992 -0.228883206 0.064676418
1403637010.501667 6390.313758241 15220.488168720 -32024.455624898 0.546408903 0.788793210 -0.281468622 -0.004242459
1403637010.551666 6395.669873132 15236.088678411 -32049.557035286 0.440713914 0.832224462 -0.328391441 -0.073025700
1403637010.601667 6401.012774933 15251.708556379 -32074.680087815 0.326302005 0.858906865 -0.368919476 -0.140372432
```

## Why This Should Not Be Committed

The patch solved the raw local Jacobian test but failed the system-level VIO behavior. A local finite-difference match is necessary for a raw EKF measurement model, but it is not sufficient when the current MSCKF implementation includes FEJ/observability-style terms, nullspace projection, gating, pruning, covariance cross-correlations, and update assumptions that may depend on the existing Jacobian convention.

Committing the raw replacement would trade one isolated validation failure for severe trajectory divergence.

## Lesson Learned

This experiment shows that the current Jacobian path is coupled to filter consistency choices, not merely to the raw projection residual. The existing `measurement_jacobian()` may be attempting to preserve observability through a FEJ-like correction. Whether that implementation is correct remains unproven, but replacing it with raw finite-difference Jacobians without updating the surrounding filter formulation is unsafe.

## Recommended Next Direction

Proceed with FEJ-aware or filter-consistency-aware Jacobian validation instead of raw Jacobian replacement.

Recommended next steps:

1. Derive the intended FEJ/observability projection currently applied to `H_x`.
2. Extend `tools/phase3b_jacobian_check.py` with a mode that validates raw projection Jacobians separately from FEJ-projected Jacobians.
3. Add checks for observability constraints, not just raw finite-difference agreement.
4. Review how `H_f` should be formed before nullspace projection when FEJ correction is applied.
5. Only propose a source fix after the intended filter formulation is documented and validated end to end.
