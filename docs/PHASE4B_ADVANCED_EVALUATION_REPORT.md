# Phase 4B Advanced Evaluation Report

## Goal

Phase 4B extends trajectory evaluation from single-file ATE-style position RMSE
to a repeatable EuRoC batch workflow. This phase does not change VIO, MSCKF,
image processing, or dataset streaming behavior.

## What Was Added

### `tools/evaluate_trajectory.py`

The single-trajectory evaluator now exposes reusable structured results for
batch callers and reports:

- ATE-style translation RMSE;
- mean, median, std, min, and max translation error;
- aligned sample count;
- estimate, ground-truth, and overlap timestamp ranges;
- optional alignment modes: `none`, `translation`, `se3`, and existing `sim3`;
- simple translation RPE over a fixed time delta.

Default RPE delta:

```text
1.0 second
```

RPE can be disabled with:

```powershell
--no-rpe
```

The tool still imports matplotlib lazily only when `--plot` is used.

### `tools/evaluate_euroc_batch.py`

The new batch evaluator:

- accepts `--datasets-root`;
- accepts `--results-root` or `--estimates-root`;
- discovers EuRoC Machine Hall dataset folders;
- recursively finds matching `output_<dataset>_offset*.txt` estimates;
- evaluates every available estimate/ground-truth pair;
- skips missing estimates with clear warnings;
- prints a summary table;
- optionally writes a small summary CSV only when `--save-csv` is provided.

No generated CSV, PNG, log, dataset, or cache files are required or committed.

## How To Run Single Evaluation

```powershell
python tools\evaluate_trajectory.py --estimate results\phase3a_mh01\output_MH_01_easy_offset10.txt --dataset .\datasets\MH_01_easy
```

Use a direct ground-truth CSV path if needed:

```powershell
python tools\evaluate_trajectory.py --estimate results\phase3a_mh01\output_MH_01_easy_offset10.txt --groundtruth .\datasets\MH_01_easy\mav0\state_groundtruth_estimate0\data.csv
```

Optional plot:

```powershell
python tools\evaluate_trajectory.py --estimate results\phase3a_mh01\output_MH_01_easy_offset10.txt --dataset .\datasets\MH_01_easy --plot results\phase4b_mh01\evaluation.png
```

Matplotlib is optional and only needed for `--plot`.

## How To Run Batch Evaluation

```powershell
python tools\evaluate_euroc_batch.py --datasets-root ./datasets --results-root ./results
```

Optional summary CSV:

```powershell
python tools\evaluate_euroc_batch.py --datasets-root ./datasets --results-root ./results --save-csv results\phase4b_summary.csv
```

Do not commit generated CSV or PNG files.

## Metrics Computed

### ATE Translation

The evaluator aligns estimated positions to interpolated EuRoC ground-truth
positions and computes:

```text
sqrt(mean(||p_est_aligned - p_gt||^2))
```

The default alignment is SE(3), which preserves metric scale.

### Translation RPE

The evaluator computes a simple translation-only RPE over a fixed time delta:

```text
||(p_est(t + delta) - p_est(t)) - (p_gt(t + delta) - p_gt(t))||
```

The default delta is `1.0` second. This is a first useful RPE translation
diagnostic, not a full SE(3) RPE implementation.

## Metrics Not Computed Yet

- orientation error;
- rotation RPE;
- full SE(3) RPE;
- segment-length RPE;
- covariance consistency or NEES/NIS metrics.

These should not be inferred from the current output.

## Commands Run

```powershell
python -m py_compile tools\evaluate_trajectory.py tools\evaluate_euroc_batch.py
python tools\evaluate_trajectory.py --estimate results/phase3a_mh01/output_MH_01_easy_offset10.txt --dataset ./datasets/MH_01_easy
python tools\evaluate_trajectory.py --estimate results/phase3a_mh02/output_MH_02_easy_offset10.txt --dataset ./datasets/MH_02_easy
python tools\evaluate_euroc_batch.py --datasets-root ./datasets --results-root ./results
git diff --check
```

`py_compile` created a temporary `tools/__pycache__` directory during checking;
it should be removed before commit.

## Single Evaluation Results

Default alignment: `se3`.

| Dataset | Estimate | Aligned samples | ATE RMSE (m) | ATE mean (m) | ATE median (m) | ATE max (m) | RPE 1s RMSE (m) |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| MH_01_easy | `results/phase3a_mh01/output_MH_01_easy_offset10.txt` | 3440 | 0.114361 | 0.104686 | 0.105760 | 0.344431 | 0.054536 |
| MH_02_easy | `results/phase3a_mh02/output_MH_02_easy_offset10.txt` | 2797 | 0.177166 | 0.167147 | 0.162438 | 0.322933 | 0.047733 |

These match the Phase 4A ATE baseline while adding translation RPE.

## Batch Evaluation Results

Command:

```powershell
python tools\evaluate_euroc_batch.py --datasets-root ./datasets --results-root ./results
```

Default alignment: `se3`.

| Dataset | Estimate | Aligned samples | ATE RMSE (m) | ATE mean (m) | ATE median (m) | ATE max (m) | RPE 1s RMSE (m) |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| MH_01_easy | `phase2_mh01/output_MH_01_easy_offset10.txt` | 3440 | 0.114292 | 0.104466 | 0.105405 | 0.326734 | 0.054701 |
| MH_01_easy | `phase3a_mh01/output_MH_01_easy_offset10.txt` | 3440 | 0.114361 | 0.104686 | 0.105760 | 0.344431 | 0.054536 |
| MH_01_easy | `phase3c_mh01/output_MH_01_easy_offset10.txt` | 3440 | 41133.400885 | 35383.808247 | 35948.689930 | 92892.755572 | 934.994793 |
| MH_01_easy | `txts/output_MH_01_easy_offset0.txt` | 3638 | 0.421570 | 0.363258 | 0.284936 | 0.983577 | 0.086806 |
| MH_01_easy | `txts/output_MH_01_easy_offset40.txt` | 2840 | 0.074005 | 0.066211 | 0.062219 | 0.200577 | 0.050622 |
| MH_02_easy | `phase2_mh02/output_MH_02_easy_offset10.txt` | 2797 | 0.186846 | 0.176169 | 0.169151 | 0.352072 | 0.047746 |
| MH_02_easy | `phase3a_mh02/output_MH_02_easy_offset10.txt` | 2797 | 0.177166 | 0.167147 | 0.162438 | 0.322933 | 0.047733 |
| MH_02_easy | `phase3c_mh02/output_MH_02_easy_offset10.txt` | 2797 | 10449.737931 | 8991.448058 | 8948.618326 | 24494.105821 | 304.904127 |
| MH_02_easy | `txts/output_MH_02_easy_offset30.txt` | 2361 | 0.239026 | 0.213916 | 0.203586 | 0.489998 | 0.049482 |
| MH_03_medium | `txts/output_MH_03_medium_offset10.txt` | 2459 | 0.182146 | 0.167464 | 0.173398 | 0.350797 | 0.064665 |
| MH_05_difficult | `txts/output_MH_05_difficult_offset40.txt` | 295 | 0.161963 | 0.151800 | 0.143351 | 0.392182 | 0.122997 |

The rejected Phase 3C raw-Jacobian outputs appear clearly as severe outliers,
which is useful for regression detection.

## Missing Outputs

No matching estimate file was found for:

```text
MH_04_difficult
```

Generate it with:

```powershell
python main.py --path .\datasets\MH_04_difficult --offset 10 --output-dir .\results\phase4b_mh04
```

Then rerun:

```powershell
python tools\evaluate_euroc_batch.py --datasets-root ./datasets --results-root ./results
```

## Known Limitations

- Batch mode evaluates every matching output file. If multiple experimental
  outputs exist for one dataset, all appear in the table.
- The simple RPE implementation is translation-only and fixed-time-delta based.
- Orientation error and rotation RPE are intentionally not reported yet.
- Ground-truth positions are linearly interpolated at estimate timestamps.
- The current project output is treated as an IMU-position trajectory exactly as
  written by `MSCKF._write_state()`.
- Existing result files are local/generated artifacts and should not be added to
  git.

## Next Recommended Phase

Phase 4C: add full SE(3) trajectory evaluation:

1. document the output pose frame convention precisely;
2. interpolate ground-truth orientation with quaternion SLERP;
3. compute orientation error;
4. compute SE(3) RPE over fixed time and/or fixed distance intervals;
5. optionally add a small regression threshold report for known stable outputs.
