# Release Checklist

Use this checklist before handing off or tagging the current runtime/evaluation
baseline.

## Install

From the repository root:

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

Optional plotting for evaluation:

```powershell
python -m pip install matplotlib
```

Optional viewer mode:

```powershell
python -m pip install PyQt5 pyqtgraph
```

Optional evaluation dashboard:

```powershell
python -m pip install PyQt5 pyqtgraph PyOpenGL matplotlib
```

## Dataset Placement

Place EuRoC sequences under `datasets` or another local directory that is not
committed to git. Each sequence path passed to `main.py` must contain:

```text
<sequence>\mav0\imu0\data.csv
<sequence>\mav0\cam0\data.csv
<sequence>\mav0\cam0\data\*.png
<sequence>\mav0\cam1\data.csv
<sequence>\mav0\cam1\data\*.png
<sequence>\mav0\state_groundtruth_estimate0\data.csv
```

## Smoke Checks

Import-only smoke check:

```powershell
python tools\smoke_check.py
```

Dataset smoke check:

```powershell
python tools\smoke_check.py --dataset .\datasets\MH_01_easy
```

Optional viewer dependency check:

```powershell
python tools\smoke_check.py --check-viewer
```

EuRoC dataset registry check:

```powershell
python tools\dataset_registry.py --datasets-root .\datasets
```

## Run Commands

Recommended headless runs:

```powershell
python main.py --path .\datasets\MH_01_easy --offset 10 --output-dir .\results\release_mh01
python main.py --path .\datasets\MH_02_easy --offset 10 --output-dir .\results\release_mh02
python main.py --path .\datasets\MH_03_medium --offset 10 --output-dir .\results\release_mh03
python main.py --path .\datasets\MH_04_difficult --offset 10 --output-dir .\results\release_mh04
python main.py --path .\datasets\MH_05_difficult --offset 10 --output-dir .\results\release_mh05
```

Default output file format:

```text
# timestamp p_x p_y p_z q_x q_y q_z q_w
```

Repeated runs replace the same output file unless `--append-output` is used.

## Evaluation Commands

Single trajectory:

```powershell
python tools\evaluate_trajectory.py --estimate .\results\release_mh01\output_MH_01_easy_offset10.txt --dataset .\datasets\MH_01_easy
```

Batch evaluation:

```powershell
python tools\evaluate_euroc_batch.py --datasets-root .\datasets --results-root .\results
```

List available local results:

```powershell
python tools\list_available_results.py --datasets-root .\datasets --results-root .\results
```

Launch the graphical evaluation dashboard:

```powershell
python tools\evaluation_dashboard.py --datasets-root .\datasets --results-root .\results
```

Validate dashboard data flow without launching the GUI:

```powershell
python tools\evaluation_dashboard.py --validate-data-flow --datasets-root .\datasets --results-root .\results
```

Optional batch summary CSV, only when explicitly needed:

```powershell
python tools\evaluate_euroc_batch.py --datasets-root .\datasets --results-root .\results --save-csv .\results\release_summary.csv
```

Do not commit generated CSV files.

## Files Not To Commit

Do not commit:

- EuRoC datasets under `datasets`;
- generated trajectory text files under `results`;
- generated aligned CSV files;
- generated dashboard metrics summaries such as
  `results\dashboard_metrics_summary.json`;
- generated PNG plots;
- generated videos;
- logs;
- Python `__pycache__` directories or `.pyc` files;
- virtual environments such as `.venv`;
- local temporary output directories.

## Known Risks

- MH_04_difficult currently has much weaker ATE/RPE than the easier sequences.
- Evaluation is position-only ATE plus simple translation RPE; orientation error
  and full SE(3) RPE are not implemented yet.
- Current outputs are generated with mixed offsets and historical runs, so they
  are useful as a baseline snapshot but not a controlled benchmark sweep.
- The covariance update form and threading architecture remain known review
  items.
- FEJ/observability consistency has synthetic validation, but not a full
  literature-derived proof.
- Viewer mode remains optional and is not part of release validation.

## Next Development Phases

1. Regenerate all Machine Hall outputs with one controlled offset policy.
2. Add full SE(3) RPE and orientation error metrics.
3. Investigate MH_04_difficult accuracy and robustness.
4. Review covariance update consistency and PSD behavior.
5. Review threading/synchronization architecture.
6. Extend evaluation to Vicon Room sequences.
