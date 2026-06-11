# Phase 9 Bug Audit, Evaluation Hygiene, and Current VIO Status Report

Repository: `ZalSaberi/Sattar-uav-relative-vio-navigation`
Local working directory used during debugging: `J:\Sattar Run`
Primary dataset family: EuRoC Machine Hall stereo + IMU sequences
Document purpose: consolidate the fragmented bug-audit, MSCKF review, evaluation, GUI, and warm-up investigations into one readable project-status report.

---

## 1. Executive Summary

This project is a Python stereo visual-inertial odometry prototype based on an MSCKF-style backend. The runtime data flow is:

```text
EuRoC dataset
-> stereo image and IMU readers
-> publisher queues
-> VIO image / IMU / feature worker threads
-> image-processing frontend
-> MSCKF update backend
-> trajectory output
-> evaluation tools and dashboard
```

The project has gone through several debugging phases. Early phases focused on launch/runtime hygiene, stereo frontend correctness, MSCKF guards, Jacobian validation, trajectory evaluation tooling, and dashboard reliability. Later phases focused on a more specific problem: an unstable initial section of the trajectory, especially visible on `MH_05_difficult`.

The current conclusion is:

```text
The project now has a reliable evaluation path and a warm-up-aware metric mode,
but the underlying VIO algorithm still has an initial transient problem during
roughly the first 10-20 seconds after startup.
```

The investigation did not prove that the live dashboard or GUI synchronization caused the initial trajectory drift. Headless evaluation still reproduced the issue. Multiple backend and frontend candidate fixes were tested, but most did not improve the full trajectory. The strongest confirmed pattern is that excluding the early warm-up interval from evaluation substantially improves ATE for many existing outputs.

This does not mean the algorithm is fully fixed. It means the evaluation system can now separate two different questions:

1. How well does the estimator perform after it has stabilized?
2. Why is the estimator unstable during the initial warm-up interval?

Phase 9 addresses the first question by adding warm-up-aware evaluation. The next research/debug phase should address the second question directly.

---

## 2. What Was Already Known From Earlier Audits

The initial full-repository audit found that the project contained several categories of risk:

* stale launch documentation and command inconsistencies;
* optional GUI dependencies imported too early;
* dataset validation gaps;
* threading and queue lifecycle risks;
* stereo calibration and feature-tracking risks;
* MSCKF gating, covariance, and Jacobian risks;
* missing trajectory evaluation tools;
* missing separation between generated result artifacts and source documentation.

The most important architecture-level finding was that the system is a stereo + IMU VIO prototype, not a fully validated navigation product. Therefore, the debugging strategy needed to avoid large speculative rewrites and instead use controlled phases, isolated branches, and numeric evidence from EuRoC evaluation.

---

## 3. Bug Investigation Methodology

The debugging process followed a phased and evidence-driven method.

First, the runtime and output infrastructure were stabilized enough to produce deterministic trajectory files. The project gained explicit output directories so that experiments could be compared without mixing outputs from different runs.

Second, ground-truth evaluation was added. The evaluator reads the project output format:

```text
timestamp p_x p_y p_z q_x q_y q_z q_w
```

and compares it against EuRoC ground truth from:

```text
<dataset>/mav0/state_groundtruth_estimate0/data.csv
```

The default alignment is SE(3), because the estimator trajectory starts from a local frame while EuRoC ground truth is stored in a global frame.

Third, the dashboard was converted from a visual prototype into a real evaluation interface. The dashboard now uses real evaluation results, not fake metric values, and it can show live image preview, live/final trajectories, ATE/RPE plots, and global comparison tables.

Fourth, specific suspected root causes were tested one at a time. Each experiment was evaluated against the same target sequence where possible, especially `MH_05_difficult --offset 10`. Experiments that worsened the metrics were rejected instead of being merged.

Fifth, the team avoided deleting dataset images or editing EuRoC data. When the initial transient became suspicious, the correct test was to crop the estimate during evaluation only, not to alter the dataset or the VIO input stream.

---

## 4. Accepted Improvements

### 4.1 Runtime and frontend stability improvements

Earlier phases fixed several high-confidence frontend/runtime problems:

* separate cam0 and cam1 camera models were introduced where needed;
* stereo epipolar residual calculation and calibration usage were reviewed and corrected in earlier frontend work;
* LK output masks and `None` returns were guarded;
* grid indexing and image mask boundaries were made safer;
* unreadable image frames and stereo timestamp mismatches received clearer handling;
* dataset/output paths were made more deterministic.

These changes made the project more runnable and reduced obvious frontend failure modes. They did not, by themselves, prove final VIO accuracy.

### 4.2 MSCKF gating and guard improvements

A high-confidence MSCKF bug was found in the chi-square gating logic. The original review found a mismatch between the intended confidence level and the implemented threshold. The gate was corrected to use the intended 0.95 confidence behavior and the actual projected residual dimension.

Additional finite-value and depth guards were added around feature initialization and measurement update paths. These guards prevent NaN/Inf residuals, invalid depths, and invalid Jacobian/update vectors from entering the filter.

### 4.3 FEJ-aware Jacobian validation

The first raw finite-difference Jacobian check appeared to show that the MSCKF measurement Jacobian was wrong. A direct raw-Jacobian replacement made the synthetic checker pass but caused severe end-to-end divergence. This was an important lesson: the checker was testing the derivative of the residual `z - h(x)`, while the current filter update expects the measurement-model derivative `dh/dx`.

A later FEJ-aware checker separated these concepts:

```text
raw residual derivative:       d(z - h) / dx
measurement-model derivative:  dh / dx
filter-compatible H matrix:    H = dh / dx
FEJ / OC projected Jacobian:   projected measurement-model Jacobian
```

The FEJ-aware checker passed and supported the conclusion that the current `measurement_jacobian()` is not a plain raw residual Jacobian. It appears to implement an observability-constrained or FEJ-style projected measurement-model Jacobian. Therefore, no source-code Jacobian fix is currently justified.

### 4.4 Joseph covariance update

The simplified covariance update in the MSCKF backend was identified as mathematically risky. A Joseph-form covariance update was implemented and kept as a correct backend improvement:

```text
P = (I - K H) P (I - K H)^T + K R K^T
```

This change did not solve the initial trajectory drift by itself, but it is still a correct consistency improvement and remains the preferred backend base for later experiments.

### 4.5 Evaluation tooling and dashboard

The project now has:

* single-trajectory evaluation;
* batch EuRoC evaluation;
* SE(3), Sim(3), translation-only, and no-alignment modes;
* ATE-style translation metrics;
* translation-only 1-second RPE;
* optional CSV and PNG plot exports;
* a dashboard with dataset registry support;
* live and final trajectory visualization;
* a global comparison table;
* dashboard self-check and data-flow validation modes.

The dashboard is useful for running and comparing outputs, but it must ignore generated/cropped analysis artifacts so that derived files are not mistaken for raw VIO outputs.

---

## 5. Rejected or Non-Final Experiments

Several experiments were useful diagnostically but should not be treated as final fixes.

### 5.1 Raw Jacobian replacement

The raw Jacobian experiment made a synthetic checker pass but caused catastrophic divergence on EuRoC runs. It was rejected.

Reason:

```text
The filter update convention expects H = dh/dx, not d(z-h)/dx.
The raw replacement effectively reversed the update direction and removed the
existing observability-constrained projection.
```

### 5.2 Temporal RANSAC in the feature tracker

A temporal Fundamental Matrix RANSAC path was tested. It rejected almost nothing at the important bad timestamps and worsened `MH_05_difficult` performance. It was rejected.

### 5.3 Forward-backward temporal LK filtering

A forward-backward LK consistency check was tested in Phase 8A. It improved short-term smoothness in some sense but worsened overall ATE:

```text
Phase 8A MH_05 offset 10:
ATE RMSE: 0.536106 m
RPE 1s RMSE: 0.133555 m
```

This was worse than the baseline and was rejected.

### 5.4 Re-enabling feature motion gate with a hard threshold

The disabled feature motion gate was re-enabled with `translation_threshold = 0.2`. This was too aggressive and made the trajectory worse:

```text
Phase 8B MH_05 offset 10:
ATE RMSE: 0.572929 m
ATE max: 2.848016 m
RPE 1s RMSE: 0.275843 m
```

This experiment showed that simply removing weak features can starve or destabilize the filter. It was rejected.

### 5.5 Full-track promotion and aggressive geometry pruning

Several geometry-based pruning, weighting, and full-track promotion experiments were tested. They revealed that early prune updates often use weak geometry, but aggressive fixes either had limited benefit or worsened performance. These experiments are diagnostic, not final fixes.

---

## 6. Phase 6 and Phase 7 Diagnostic Findings

The deeper diagnostics focused on `MH_05_difficult --offset 10`.

### 6.1 Update spikes with small residuals

Diagnostic logs showed that some updates produced large state corrections even when the residual norm was small. Examples included early updates around 6-10 seconds where position corrections were large relative to the residual norm.

This suggested that the issue was not simply one obvious residual outlier. Instead, the filter could produce high-gain corrections from weak or ill-conditioned measurement geometry.

### 6.2 Conditioning and gain analysis

Conditioning diagnostics showed that some update matrices had very poor conditioning and high correction-to-residual ratios. This supported the hypothesis that small residuals could still create large state corrections when the geometry or covariance structure was unfavorable.

### 6.3 Feature geometry analysis

Feature geometry logs showed that many early accepted features had weak temporal geometry:

* very small used baseline;
* very small used parallax;
* far or weakly constrained depth;
* much stronger full-track geometry than the subset used during pruning.

This was a real diagnostic clue. However, the later experiments that tried to exploit this by pruning or promoting full tracks did not produce a safe general fix.

### 6.4 IMU initialization audit

The IMU initialization window for `MH_05_difficult --offset 10` was not stationary. That means treating the mean initial angular velocity as gyro bias is unsafe for moving starts.

A stationary-aware initialization test was attempted. It improved some values slightly but did not solve the overall problem. This remains an important research direction, but it is not yet a final fix.

---

## 7. Phase 8 Frontend Hypotheses

Phase 8 investigated whether frontend tracking and initialization were directly causing the initial trajectory instability.

Two ideas were tested:

1. rejecting temporal LK tracks using a forward-backward check;
2. re-enabling the feature motion gate.

Both worsened the full `MH_05_difficult` trajectory. This does not prove the frontend is correct. It proves only that these specific aggressive frontend filters are not the right final fix.

The current frontend still deserves review. Known concerns include:

* feature tracker RANSAC claims versus actual filtering behavior;
* stereo disparity/depth sanity;
* how weak stereo/temporal geometry reaches MSCKF updates;
* whether startup feature initialization produces biased landmarks.

However, future frontend fixes should be tested carefully because simple feature rejection can reduce the number of constraints and make the estimator worse.

---

## 8. Discovery of the Warm-Up / Initial Transient Effect

After multiple algorithmic patches failed to solve the drift, the investigation tested whether the error was concentrated near the beginning of the output.

A controlled test cropped only the estimate used by the evaluator. The dataset and VIO input were not modified.

For `MH_05_difficult` Phase 7A output, skipping the first 15 seconds during evaluation produced:

```text
ATE RMSE: 0.336507 m
RPE 1s RMSE: 0.134346 m
```

This was better than the full-output baseline and showed that a significant part of the error comes from the initial transient.

A broader sensitivity test across existing Machine Hall outputs produced:

```text
Rows inspected: 74
Datasets: MH_01_easy, MH_02_easy, MH_03_medium, MH_05_difficult
Improved rows: 67
Not improved rows: 7
Improved percentage: 90.54%
Mean ATE gain: 18.66%
Median ATE gain: 17.46%
Best skip distribution:
  0 s:   7 rows
  10 s:  6 rows
  20 s: 61 rows
```

This strongly suggests that warm-up sensitivity is a general evaluation effect across the available Machine Hall outputs, not only a one-off `MH_05` issue.

Important limitation:

```text
The sensitivity table included many experimental outputs, including rejected or
non-final branches. Therefore, the 90.54% value should not be presented as a
final scientific benchmark. It is strong engineering evidence for adding an
optional warm-up skip to the evaluation workflow.
```

---

## 9. Phase 9: Warm-Up-Aware Evaluation

Phase 9 fixes the evaluation hygiene problem, not the underlying estimator startup problem.

### 9.1 CLI evaluation

The single-trajectory evaluator gained:

```text
--skip-estimate-seconds <seconds>
```

This option ignores the first N seconds of the estimate before alignment and metric calculation.

Example:

```bash
python tools/evaluate_trajectory.py \
  --estimate results/phase7a_msckf_core_correctness/mh05/runtime_run/output_MH_05_difficult_offset10.txt \
  --dataset ./datasets/MH_05_difficult \
  --align se3 \
  --skip-estimate-seconds 15
```

The output should report the warm-up skip and compute metrics only on the remaining trajectory.

### 9.2 Batch evaluation

The batch evaluator was extended to pass the same warm-up skip argument into `evaluate_files()` and to record the skip value in summary CSV outputs.

### 9.3 Dashboard integration

The dashboard should expose a `Warm-up Skip [s]` control in the single-evaluation panel. This should affect evaluation metrics only. It must not alter the dataset, VIO processing, or raw output file.

### 9.4 Generated/cropped output filtering

During warm-up sensitivity testing, generated cropped estimate files were written under:

```text
results/warmup_sensitivity_existing_outputs/cropped_estimates/
```

The dashboard must not treat these files as raw VIO outputs. If it does, it may apply warm-up skip twice and remove too many samples.

Therefore, dashboard candidate discovery should reject:

```text
warmup_sensitivity_existing_outputs
cropped_estimates
output files whose names contain _skipNs
```

This is data hygiene, not an estimator change.

---

## 10. Current State of the Project

At the time of this report, the most reliable working base is:

```text
phase7a-msckf-core-correctness
```

with Phase 9 evaluation changes on top.

Current status:

```text
Done:
- runtime and evaluation tooling are usable;
- dashboard is mostly reliable and real-metric based;
- FEJ-aware Jacobian validation no longer supports a raw Jacobian replacement;
- Joseph covariance update is the preferred backend base;
- warm-up-aware evaluation has been added to CLI evaluation;
- warm-up sensitivity is confirmed across many existing outputs.

Not done:
- the true algorithmic cause of the first 10-20 seconds of instability is not fixed;
- full SE(3) RPE and orientation error are still missing;
- threading and timestamp-ordering risks are not fully solved;
- IMU initialization for moving starts is not fully solved;
- frontend weak-geometry handling is not fully solved;
- dashboard filtering of derived cropped outputs must be kept in place and validated.
```

---

## 11. What Should Not Be Claimed Yet

The following claims should not be made:

```text
The VIO algorithm is fully fixed.
The first 20 seconds can simply be deleted from the dataset.
The frontend is proven correct.
The MSCKF formulation is fully proven.
Warm-up skip is a scientific benchmark improvement.
```

The correct claim is narrower:

```text
The evaluator and dashboard now support warm-up-aware metrics, which separate
stable post-startup performance from startup transient behavior. The startup
transient remains an algorithmic issue for the next debugging phase.
```

---

## 12. Recommended Next Phase

The next phase should be:

```text
Phase 10: Initial Transient Root-Cause Audit
```

The goal should be to debug only the first 20 seconds, not the whole trajectory.

Recommended questions:

1. Which exact MSCKF updates create the first large pose deviations?
2. Are those updates caused by remove-lost features, prune-camera-state-buffer, or both?
3. Are early features initialized with weak stereo/temporal geometry?
4. Is moving-start IMU initialization biasing the initial orientation, velocity, or gyro bias?
5. Are covariance values too confident during the first visual updates?
6. Is IMU/image timestamp ordering contributing to early inconsistency?
7. Does the first stable post-warm-up estimate converge from a biased initial state or simply align away the early transient?

Recommended diagnostics:

```text
- Log only 0-20 seconds of MH_05 offset 10.
- Compare full metrics and skip-15/skip-20 metrics from the same raw output.
- Record first update timestamps, context, residual norm, gain norm, delta position, delta velocity, and feature geometry.
- Avoid new aggressive feature rejection until the exact failing update path is isolated.
```

---

## 13. Recommended Documentation and Git Hygiene

Generated artifacts should not be committed:

```text
results/
*.csv generated from evaluations
*.png generated from plots
runtime logs
cropped warm-up estimates
phase review bundles
```

Documentation that should be committed:

```text
docs/PHASE9_BUG_AUDIT_AND_WARMUP_EVALUATION_REPORT.md
docs/EVALUATION_RUNBOOK.md, after adding the new warm-up option
docs/PHASE10_INITIAL_TRANSIENT_AUDIT_PLAN.md, when Phase 10 begins
```

Recommended commit for this report:

```bash
git add docs/PHASE9_BUG_AUDIT_AND_WARMUP_EVALUATION_REPORT.md
git commit -m "docs: add phase9 bug audit and warmup evaluation report"
```

---

## 14. Practical Commands

Evaluate without warm-up skip:

```bash
python tools/evaluate_trajectory.py \
  --estimate results/phase7a_msckf_core_correctness/mh05/runtime_run/output_MH_05_difficult_offset10.txt \
  --dataset ./datasets/MH_05_difficult \
  --align se3
```

Evaluate with 15-second warm-up skip:

```bash
python tools/evaluate_trajectory.py \
  --estimate results/phase7a_msckf_core_correctness/mh05/runtime_run/output_MH_05_difficult_offset10.txt \
  --dataset ./datasets/MH_05_difficult \
  --align se3 \
  --skip-estimate-seconds 15
```

Run dashboard with warm-up skip:

```bash
python tools/evaluation_dashboard.py \
  --datasets-root ./datasets \
  --results-root ./results \
  --warmup-skip-seconds 20
```

Validate dashboard data flow:

```bash
python tools/evaluation_dashboard.py \
  --validate-data-flow \
  --datasets-root ./datasets \
  --results-root ./results \
  --warmup-skip-seconds 20
```

---

## 15. Final Conclusion

The project has moved from unstructured debugging to evidence-based evaluation.

The major outcome is not that the estimator is solved. The major outcome is that the team can now measure the estimator in two modes:

```text
raw startup-inclusive performance
stable post-warm-up performance
```

The warm-up analysis showed that the initial transient is a broad effect in the available Machine Hall outputs. Therefore, warm-up-aware evaluation is necessary for honest comparison and dashboard usability.

The next technical objective is to fix or reduce the initial transient inside the estimator, not to hide it. Phase 10 should focus narrowly on the first 20 seconds and should use the new warm-up-aware evaluator as a diagnostic tool rather than as a replacement for algorithmic debugging.
