# Phase 3A P0 Fixes Report

## 1. Goal

Phase 3A fixed only the approved P0 MSCKF/VIO correctness bugs from the Phase 3 review. The scope was limited to chi-square gating correctness and invalid numeric/depth guards before feature initialization and measurement updates. It did not change covariance update form, threading architecture, or camera-state pruning policy.

## 2. Branch and Commit Context

- Baseline branch: `codex/runtime-baseline`
- Working branch: `codex/phase3-p0-msckf-fixes`

## 3. Fixes Applied

- Changed chi-square gating confidence from the incorrect 0.05 quantile to the documented 0.95 confidence level.
- Derived gating degrees of freedom from the projected residual length actually passed to gating, instead of using camera-state counts.
- Added non-finite guards before measurement updates so NaN/Inf residuals, Jacobians, solves, or update vectors do not enter the filter.
- Added denominator and depth guards in feature depth estimation to reject degenerate initial depth guesses.
- Added invalid reprojection and Jacobian guards for near-zero, negative, or non-finite depth.
- Added positive-depth validation across all observing cameras before accepting initialized features.

## 4. Tests Run

```powershell
python tools/phase2_sanity_check.py
python -m py_compile src/msckf.py src/feature/feature_depth_estimator.py src/feature/feature_observation.py src/feature/feature_position_initializer.py tools/phase2_sanity_check.py
git diff --check
python tools/smoke_check.py --dataset ./datasets/MH_01_easy
python main.py --path ./datasets/MH_01_easy --offset 10 --output-dir ./results/phase3a_mh01
python main.py --path ./datasets/MH_02_easy --offset 10 --output-dir ./results/phase3a_mh02
```

The smoke and full dataset runs were executed from the dataset-bearing worktree where `./datasets/MH_01_easy` and `./datasets/MH_02_easy` are available.

## 5. Latest Tail Summary

Final MH_01 tail:

```text
1403636763.463556 -0.183191336 0.048971032 -0.103316048 0.074705712 -0.802073403 0.099899242 0.584052612
1403636763.513556 -0.181701009 0.049694966 -0.102280100 0.074273111 -0.802185341 0.100455964 0.583858530
1403636763.563556 -0.179638344 0.049872454 -0.105617909 0.074454967 -0.802201976 0.100162146 0.583862991
1403636763.613555 -0.178016912 0.050880993 -0.104351065 0.074215125 -0.802279477 0.100358000 0.583753396
1403636763.663556 -0.175756401 0.051152744 -0.107430744 0.074334162 -0.802189637 0.100292351 0.583872986
1403636763.713556 -0.174312718 0.051972886 -0.105967915 0.074028576 -0.802292366 0.100567297 0.583723349
1403636763.763556 -0.171934907 0.052527809 -0.108803406 0.074238695 -0.802353742 0.100077474 0.583696486
1403636763.813555 -0.170546422 0.052404755 -0.106767499 0.073846867 -0.802320838 0.100688723 0.583686298
```

Final MH_02 tail:

```text
1403637010.251667 -0.136744669 0.410899909 -0.029284085 -0.028166431 -0.790145190 0.026922390 0.611679994
1403637010.301667 -0.137015889 0.411246179 -0.029734800 -0.028126805 -0.790303656 0.027088765 0.611469716
1403637010.351666 -0.137672371 0.410314259 -0.029507512 -0.028180500 -0.790207573 0.026991548 0.611595705
1403637010.401667 -0.137895831 0.410776684 -0.029782060 -0.028213117 -0.790254347 0.027146341 0.611526911
1403637010.451667 -0.138388375 0.409875123 -0.029744323 -0.028313832 -0.790270613 0.027178907 0.611499790
1403637010.501667 -0.138522481 0.410648836 -0.029916979 -0.028343558 -0.790283976 0.027206331 0.611479922
1403637010.551666 -0.138764417 0.409886690 -0.029911699 -0.028390567 -0.790260257 0.027150145 0.611510893
1403637010.601667 -0.138708585 0.410682316 -0.029839353 -0.028475739 -0.790288302 0.027375499 0.611460640
```

Both runs completed and remained bounded in the final tail. These are smoke/stability checks, not accuracy claims.

## 6. Remaining Risks

- Covariance update form was not changed.
- Threading and shared-buffer race conditions were not changed.
- Camera-state pruning policy was not changed.
- Measurement Jacobians still need validation against derivation and finite differences.
- No ATE/RPE ground-truth evaluation has been performed yet.

## 7. Next Recommended Phase

Phase 3B: Jacobian validation or threading/synchronization review.
