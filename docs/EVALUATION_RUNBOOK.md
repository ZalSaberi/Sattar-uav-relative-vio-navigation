# Evaluation Runbook

This runbook explains how to evaluate project trajectory output against EuRoC
ground truth.

## Prerequisites

Install the normal runtime dependencies:

```powershell
python -m pip install -r requirements.txt
```

Optional plotting uses matplotlib. The evaluator works without it unless
`--plot` is requested:

```powershell
python -m pip install matplotlib
```

Viewer dependencies are not required.

## Generate An Estimate

Run VIO on one EuRoC sequence:

```powershell
python main.py --path .\datasets\MH_01_easy --offset 10 --output-dir .\results\phase4_mh01
```

The estimate file will be:

```text
results\phase4_mh01\output_MH_01_easy_offset10.txt
```

For MH_02:

```powershell
python main.py --path .\datasets\MH_02_easy --offset 10 --output-dir .\results\phase4_mh02
```

The estimate file will be:

```text
results\phase4_mh02\output_MH_02_easy_offset10.txt
```

Do not commit generated trajectory, CSV, PNG, or result files.

## Evaluate With Dataset Path

Use `--dataset` when the EuRoC sequence directory is available:

```powershell
python tools\evaluate_trajectory.py --estimate .\results\phase4_mh01\output_MH_01_easy_offset10.txt --dataset .\datasets\MH_01_easy
```

The tool automatically resolves:

```text
<dataset>\mav0\state_groundtruth_estimate0\data.csv
```

## Evaluate With Ground-Truth Path

Use `--groundtruth` when the CSV is not inside a standard dataset directory:

```powershell
python tools\evaluate_trajectory.py --estimate .\results\phase4_mh01\output_MH_01_easy_offset10.txt --groundtruth .\datasets\MH_01_easy\mav0\state_groundtruth_estimate0\data.csv
```

## Alignment Modes

Default:

```powershell
python tools\evaluate_trajectory.py --estimate .\results\phase4_mh01\output_MH_01_easy_offset10.txt --dataset .\datasets\MH_01_easy --align se3
```

Alignment choices:

- `se3`: rigid alignment, default and recommended for ATE-style reporting.
- `translation`: mean translation offset only.
- `sim3`: rigid alignment plus scale, useful for diagnosing scale drift.
- `none`: no alignment, useful only if both trajectories are already in the
  same frame.

## Save Aligned Samples

Write a small aligned CSV when detailed inspection is needed:

```powershell
python tools\evaluate_trajectory.py --estimate .\results\phase4_mh01\output_MH_01_easy_offset10.txt --dataset .\datasets\MH_01_easy --csv .\results\phase4_mh01\aligned.csv
```

The CSV columns are:

```text
timestamp, estimate_x, estimate_y, estimate_z, groundtruth_x, groundtruth_y, groundtruth_z, error_m
```

Generated CSV files should not be committed.

## Optional Plot

If matplotlib is installed, save an XY trajectory and translation-error plot:

```powershell
python tools\evaluate_trajectory.py --estimate .\results\phase4_mh01\output_MH_01_easy_offset10.txt --dataset .\datasets\MH_01_easy --plot .\results\phase4_mh01\evaluation.png
```

Generated PNG files should not be committed.

## Existing Phase 3A Outputs

If the existing Phase 3A outputs are present, they can be evaluated directly:

```powershell
python tools\evaluate_trajectory.py --estimate .\results\phase3a_mh01\output_MH_01_easy_offset10.txt --dataset .\datasets\MH_01_easy
python tools\evaluate_trajectory.py --estimate .\results\phase3a_mh02\output_MH_02_easy_offset10.txt --dataset .\datasets\MH_02_easy
```

Observed Phase 4A baseline metrics with default SE(3) alignment:

```text
MH_01_easy offset 10 RMSE: 0.114361 m
MH_02_easy offset 10 RMSE: 0.177166 m
```

## Troubleshooting

If the tool says `Estimate file does not exist`, run VIO first or check the
`--output-dir`, dataset name, and offset in the output filename.

If the tool says `Ground truth file does not exist`, check that `--dataset`
points to a EuRoC sequence directory such as `datasets\MH_01_easy`, not the
parent `datasets` directory.

If the tool says no estimate timestamps overlap ground truth, verify that the
estimate and ground-truth files come from the same EuRoC sequence and that the
estimate timestamp units are seconds or nanoseconds.

If `--plot` fails with a matplotlib import error, install matplotlib or rerun
without `--plot`.

If `--align none` reports very large errors, use the default `--align se3`.
The current estimator output starts from a local origin, while EuRoC ground
truth is stored in a global frame.
