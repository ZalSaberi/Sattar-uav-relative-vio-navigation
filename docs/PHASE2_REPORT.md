# Phase 2 Report

## 1. Phase 2 Goal

Phase 2 focused on making high-confidence runtime and algorithmic bug fixes that could directly cause incorrect VIO output, without rewriting the project or changing the core MSCKF/VIO math. The scope was limited to calibration usage, stereo matching correctness, feature tracking robustness, dataset validation, and small numeric guards.

## 2. Starting Branch and Commit

- Base branch: `codex/runtime-baseline`
- Working branch: `codex/phase2-critical-bugs`
- Phase 2 commit: `6974718 fix: stabilize stereo feature pipeline`

## 3. Bugs Fixed

- Added separate cam0 and cam1 camera models in the image-processing pipeline.
- Fixed stereo matcher calibration handling: it previously used cam0 calibration for both cameras.
- Fixed epipolar point-line error calculation: it used element-wise multiplication instead of the homogeneous dot product.
- Added optical-flow `None` and status-mask guards for forward and reverse LK tracking.
- Flattened OpenCV `Nx1` masks before list selection and filtering.
- Added safe feature grid indexing to avoid invalid grid cells at image boundaries.
- Clipped feature suppression mask boundaries near image borders to avoid negative-slice wraparound.
- Added grayscale image validation and clear errors for unreadable or invalid camera frames.
- Added stereo timestamp synchronization validation.
- Fixed `Stereo.start_time()` to return the camera reader start time instead of a stale attribute.
- Added an `arccos` clipping guard in MSCKF camera-state pruning to prevent NaN from tiny floating-point overshoot.

## 4. Tests Run

```powershell
python tools/phase2_sanity_check.py
python tools/smoke_check.py --dataset ./datasets/MH_01_easy
python main.py --path ./datasets/MH_01_easy --offset 10
python main.py --path ./datasets/MH_02_easy --offset 10
```

## 5. Before / After Behavior

Before Phase 2, the MH_01 output diverged to very large position and velocity values near the end of the run, including position values in the tens of thousands, z approaching about -90000, and velocity values around -1400.

After Phase 2, MH_01 and MH_02 completed with small, stable final output values. This does not prove estimator accuracy against ground truth, but it confirms that the major runtime divergence observed before Phase 2 is no longer reproduced in these offset-10 smoke runs.

Final MH_01 tail sample:

```text
1403636763.713556 -0.116330901 0.102243368 -0.110328604 0.076285769 -0.802136151 0.102162403 0.583370311
1403636763.763556 -0.114032563 0.102787031 -0.113188217 0.076496291 -0.802196931 0.101672844 0.583344697
1403636763.813555 -0.112688625 0.102691171 -0.111160489 0.076103318 -0.802164080 0.102285538 0.583334160
```

Final MH_02 tail values:

```text
1403637010.251667 -0.110459898 0.455517231 -0.020377416 -0.026150191 -0.790023406 0.028632844 0.611849120
1403637010.301667 -0.110716126 0.455887535 -0.020826075 -0.026110476 -0.790184194 0.028798710 0.611635363
1403637010.351666 -0.111242594 0.455009455 -0.020589802 -0.026164523 -0.790087895 0.028702431 0.611761969
1403637010.401667 -0.111459908 0.455487804 -0.020863461 -0.026197148 -0.790137299 0.028856809 0.611689498
1403637010.451667 -0.111822472 0.454642937 -0.020816227 -0.026297859 -0.790153522 0.028889936 0.611662657
1403637010.501667 -0.111965268 0.455421801 -0.020989413 -0.026327398 -0.790169734 0.028916740 0.611639176
1403637010.551666 -0.112078639 0.454718879 -0.020974348 -0.026374540 -0.790145845 0.028861245 0.611670627
1403637010.601667 -0.112039077 0.455513896 -0.020902157 -0.026459294 -0.790176929 0.029085789 0.611616173
```

## 6. Remaining Limitations

- Core MSCKF equations have not been fully reviewed yet.
- Covariance update, gating, and feature initialization still need Phase 3 review.
- No ATE/RPE quantitative evaluation has been performed yet for the Phase 2 outputs.
- The viewer is still not the focus of this phase.
- Results need comparison against EuRoC ground truth before accuracy claims can be made.

## 7. Next Recommended Phase

Phase 3: MSCKF/VIO math and consistency review.
