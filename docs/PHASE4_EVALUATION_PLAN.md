# Phase 4 Trajectory Evaluation Plan

## Goal

Phase 4A adds headless tooling to evaluate the estimated VIO trajectory against
EuRoC ground truth. This phase does not change VIO, MSCKF, image processing, or
dataset streaming behavior.

## Input Formats

### Estimated Trajectory

The project writes a TUM-like text file:

```text
# timestamp p_x p_y p_z q_x q_y q_z q_w
1403636590.763556 0.000000000 0.000000000 0.000000000 0.031254365 -0.906124741 0.000000000 0.421854380
```

Timestamps are seconds. Positions are the estimated IMU position from
`MSCKF._write_state()`.

### EuRoC Ground Truth

EuRoC ground truth is read from:

```text
<dataset>/mav0/state_groundtruth_estimate0/data.csv
```

The CSV uses nanosecond timestamps and stores position in columns 1 through 3:

```text
#timestamp, p_RS_R_x [m], p_RS_R_y [m], p_RS_R_z [m], q_RS_w [], q_RS_x [], q_RS_y [], q_RS_z [], ...
1403636580838555648,4.688319,-1.786938,0.783338,0.534108,-0.153029,-0.827383,-0.082152,...
```

## Tool

Created:

```text
tools/evaluate_trajectory.py
```

The tool:

- accepts `--estimate`;
- accepts either `--groundtruth` or `--dataset`;
- reads the current project trajectory format;
- reads EuRoC ground-truth CSV files;
- converts nanosecond timestamps to seconds;
- interpolates ground-truth positions at estimate timestamps;
- computes translation-error metrics;
- supports `none`, `translation`, `se3`, and `sim3` alignment modes;
- defaults to SE(3) alignment, because this project's estimate starts from a
  local origin while EuRoC ground truth is in a global map frame;
- optionally saves aligned samples with `--csv`;
- optionally writes a PNG plot with `--plot` if matplotlib is installed;
- does not require viewer dependencies.

## Metrics

The primary metric is SE(3)-aligned ATE-style translation RMSE:

```text
rmse = sqrt(mean(||p_est_aligned - p_gt||^2))
```

The tool also prints mean, median, standard deviation, min, and max translation
error in meters.

## Alignment Policy

Default:

```text
--align se3
```

This estimates one rigid transform from estimated positions to ground-truth
positions. Scale remains fixed at `1.0`, which is appropriate for stereo VIO
when metric scale is expected.

Other modes:

- `--align none`: direct coordinate comparison, mostly useful for debugging.
- `--align translation`: subtracts only the mean offset.
- `--align sim3`: estimates rigid transform plus scale, useful as a diagnostic
  if metric scale is suspected to be wrong.

## Initial Evaluation Runs

Existing Phase 3A offset-10 output files were available and used for the first
evaluation. No new large result files were generated.

### MH_01_easy

Command:

```powershell
python tools\evaluate_trajectory.py --estimate results\phase3a_mh01\output_MH_01_easy_offset10.txt --dataset .\datasets\MH_01_easy
```

Result:

```text
samples: estimate=3462 groundtruth=36382 aligned=3440
translation RMSE: 0.114361 m
mean: 0.104686 m
median: 0.105760 m
max: 0.344431 m
```

### MH_02_easy

Command:

```powershell
python tools\evaluate_trajectory.py --estimate results\phase3a_mh02\output_MH_02_easy_offset10.txt --dataset .\datasets\MH_02_easy
```

Result:

```text
samples: estimate=2820 groundtruth=29993 aligned=2797
translation RMSE: 0.177166 m
mean: 0.167147 m
median: 0.162438 m
max: 0.322933 m
```

## Limitations

- This is position-only ATE-style evaluation; it does not compute RPE yet.
- Orientation error is not evaluated yet.
- The tool aligns positions only, not full timestamped SE(3) poses.
- Ground-truth position is linearly interpolated; quaternion interpolation is
  not needed for the current position-only metric.
- The estimated timestamp comes from `imu_state.timestamp`, which may be the
  last propagated IMU timestamp rather than the exact image timestamp.
- The tool does not decide whether the estimate is IMU, body, or camera pose
  beyond using the current project output as written.
- Existing result files are not added to git and should remain untracked or
  ignored.

## Next Steps

1. Add RPE translation and rotation metrics in a separate phase.
2. Add orientation interpolation and orientation error if the output convention
   is finalized.
3. Evaluate all EuRoC Machine Hall and Vicon Room sequences with the same
   alignment mode.
4. Record metrics in a small summary table, not as large generated result files.
