# Final Evaluation Report

## Current Software State

This report summarizes the current evaluation-ready state on branch:

```text
codex/phase4c-final-evaluation-report
```

The project has completed runtime stabilization, high-confidence feature/MSCKF
bug fixes, Jacobian consistency review, FEJ-aware Jacobian validation, and
trajectory evaluation tooling. Phase 4C adds only documentation and a read-only
result-listing helper. It does not change VIO, MSCKF, image processing, or
dataset streaming behavior.

## Dataset List

Machine Hall datasets inspected:

| Dataset | Ground truth available | Estimate available |
| --- | --- | --- |
| MH_01_easy | yes | yes |
| MH_02_easy | yes | yes |
| MH_03_medium | yes | yes |
| MH_04_difficult | yes | yes, generated locally for Phase 4C |
| MH_05_difficult | yes | yes |

MH_04 was generated with:

```powershell
python main.py --path .\datasets\MH_04_difficult --offset 10 --output-dir .\results\phase4b_mh04
```

The generated trajectory is a local result artifact and must not be committed.

## Evaluation Method

Evaluation uses:

```powershell
python tools\evaluate_euroc_batch.py --datasets-root ./datasets --results-root ./results
```

The evaluator:

- reads project trajectory files in `timestamp p_x p_y p_z q_x q_y q_z q_w`
  format;
- reads EuRoC ground truth from
  `mav0/state_groundtruth_estimate0/data.csv`;
- converts EuRoC nanosecond timestamps to seconds;
- linearly interpolates ground-truth positions at estimate timestamps;
- aligns estimated positions to ground truth with SE(3) alignment by default;
- reports ATE-style translation RMSE, mean, median, and max error;
- reports simple translation RPE over a fixed 1-second time delta.

The metric is position-only. Orientation error and full SE(3) RPE are not
reported yet.

## Release Snapshot Metrics

These are representative available outputs for each Machine Hall dataset. They
are not all from the same offset sweep, so use them as a current software
snapshot rather than a controlled benchmark suite.

| Dataset | Estimate | Aligned samples | ATE RMSE (m) | ATE mean (m) | ATE median (m) | ATE max (m) | RPE 1s RMSE (m) |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| MH_01_easy | `phase3a_mh01/output_MH_01_easy_offset10.txt` | 3440 | 0.114361 | 0.104686 | 0.105760 | 0.344431 | 0.054536 |
| MH_02_easy | `phase3a_mh02/output_MH_02_easy_offset10.txt` | 2797 | 0.177166 | 0.167147 | 0.162438 | 0.322933 | 0.047733 |
| MH_03_medium | `txts/output_MH_03_medium_offset10.txt` | 2459 | 0.182146 | 0.167464 | 0.173398 | 0.350797 | 0.064665 |
| MH_04_difficult | `phase4b_mh04/output_MH_04_difficult_offset10.txt` | 1789 | 1.039380 | 0.824681 | 0.606184 | 3.092630 | 0.393334 |
| MH_05_difficult | `txts/output_MH_05_difficult_offset40.txt` | 295 | 0.161963 | 0.151800 | 0.143351 | 0.392182 | 0.122997 |

## All Available Machine Hall Outputs

The batch evaluator found these matching trajectory files under `results`.

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
| MH_04_difficult | `phase4b_mh04/output_MH_04_difficult_offset10.txt` | 1789 | 1.039380 | 0.824681 | 0.606184 | 3.092630 | 0.393334 |
| MH_05_difficult | `txts/output_MH_05_difficult_offset40.txt` | 295 | 0.161963 | 0.151800 | 0.143351 | 0.392182 | 0.122997 |

## Missing Outputs

No Machine Hall dataset is missing an output after the local Phase 4C MH_04
run. The MH_04 output remains a generated artifact and should not be committed.

## Rejected Phase 3C Outliers

The Phase 3C raw-Jacobian experiment is intentionally rejected. Its outputs
remain visible in local `results` and are severe batch-evaluation outliers:

| Dataset | Phase 3C ATE RMSE (m) | Phase 3C RPE 1s RMSE (m) |
| --- | ---: | ---: |
| MH_01_easy | 41133.400885 | 934.994793 |
| MH_02_easy | 10449.737931 | 304.904127 |

These numbers reinforce the Phase 3C conclusion: raw residual-compatible
Jacobians should not be committed as a source-code fix for the current MSCKF
formulation.

## Interpretation

The easy and medium Machine Hall outputs are in a plausible sub-meter range
after SE(3) alignment:

- MH_01 and MH_02 offset-10 Phase 3A outputs are around `0.11 m` and `0.18 m`
  ATE RMSE.
- MH_03 medium is around `0.18 m` ATE RMSE for the available offset-10 output.
- MH_05 difficult has a short available output with around `0.16 m` ATE RMSE,
  but only `295` aligned samples, so it is not a full-sequence result.
- MH_04 difficult is much weaker at around `1.04 m` ATE RMSE and `0.39 m`
  1-second translation RPE. This needs follow-up before claiming robust
  difficult-sequence performance.

## Ready For Delivery

The current software is ready for a release-quality runtime/evaluation baseline:

- Windows setup and run instructions exist.
- Dataset validation and clear runtime errors exist.
- Output files are deterministic per dataset/offset/output directory.
- Smoke checks exist.
- Single and batch trajectory evaluation tools exist.
- A final evaluation table can be regenerated from local outputs.

## Still Research / Validation Work

The following are not complete yet:

- full SE(3) RPE;
- orientation error metrics;
- covariance consistency review and Joseph-form update decision;
- threading/race-condition architecture review;
- camera-state pruning policy validation;
- FEJ/observability derivation beyond synthetic consistency checks;
- controlled same-offset runs for all Machine Hall sequences;
- Vicon Room evaluation.

## Recommended Next Phase

Phase 5 should focus on controlled benchmark runs and robustness:

1. regenerate all Machine Hall outputs with one agreed offset policy;
2. add full SE(3) RPE and orientation error;
3. investigate MH_04 difficult performance;
4. add non-generated summary reports only;
5. continue MSCKF consistency work separately from evaluation tooling.
